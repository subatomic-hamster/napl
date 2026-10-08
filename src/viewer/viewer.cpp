#include "viewer/viewer.h"

#include <volk.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "napl_generated.h"
#include "render/window_renderer.h"
#include "viewer/design.h"
#include "viewer/wire2d.h"

namespace {
using Env = std::vector<std::uint8_t>;
using Reply = flatbuffers::DetachedBuffer;
constexpr const char* kSource = "viewer";

// ---- command builders (the one place envelopes are made: UI controls and scripts both use them) ------------------------
template <class F>
Env envelope(const std::string& source, napl::Command type, F make) {
  static std::uint64_t next_id = 1;  // viewer thread only (the script test helper is single-threaded too)
  flatbuffers::FlatBufferBuilder fbb;
  const auto src = fbb.CreateString(source);
  const auto cmd = make(fbb);
  fbb.Finish(napl::CreateEnvelope(fbb, next_id++, src, type, cmd));
  return Env(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize());
}
Env env_run(const std::string& src, double rate) {
  return envelope(src, napl::Command::Run, [&](auto& f) { return napl::CreateRun(f, rate).Union(); });
}
Env env_pause(const std::string& src) {
  return envelope(src, napl::Command::Pause, [&](auto& f) { return napl::CreatePause(f).Union(); });
}
Env env_step(const std::string& src, int n) {
  return envelope(src, napl::Command::Step, [&](auto& f) { return napl::CreateStep(f, n).Union(); });
}
Env env_setattr(const std::string& src, const std::string& path, const std::string& attr, const std::vector<double>& v) {
  return envelope(src, napl::Command::SetAttr,
                  [&](auto& f) { return napl::CreateSetAttrDirect(f, path.c_str(), attr.c_str(), "", &v).Union(); });
}
Env env_load(const std::string& src, const std::string& path) {
  return envelope(src, napl::Command::LoadModel, [&](auto& f) { return napl::CreateLoadModelDirect(f, path.c_str()).Union(); });
}
Env env_query(const std::string& src, const std::string& path, const std::string& attr) {
  return envelope(src, napl::Command::Query,
                  [&](auto& f) { return napl::CreateQueryDirect(f, path.c_str(), attr.c_str(), "").Union(); });
}

// Design-window script actions (see viewer.h): {min, max} argument counts; {-1, -1} = not one.
std::pair<int, int> design_action_args(const std::string& a) {
  static const std::map<std::string, std::pair<int, int>> t = {
      {"dsuggest", {1, 2}}, {"dsnap", {2, 3}}, {"dcycle", {1, 1}}, {"dswap", {2, 2}}, {"dremove", {1, 1}}, {"dautowire", {1, 2}},
      {"dautowire_all", {0, 0}}, {"dundo", {0, 0}}, {"dredo", {0, 0}}, {"dsave", {0, 1}}, {"dsim", {1, 1}}, {"dfinding", {1, 1}},
      {"dsync", {0, 0}}};
  const auto it = t.find(a);
  return it == t.end() ? std::pair<int, int>{-1, -1} : it->second;
}

double to_double(const std::string& s, const std::string& what) {
  std::size_t used = 0;
  double v = 0;
  try {
    v = std::stod(s, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (used != s.size() || s.empty()) throw std::runtime_error("script: " + what + " must be a number, got '" + s + "'");
  return v;
}

// ---- GLFW / ImGui lifetime (destroyed in reverse order of declaration) ---------------------------------------------------
std::string g_glfw_error;
struct GlfwInit {
  GlfwInit() {
    glfwSetErrorCallback([](int, const char* d) { g_glfw_error = d ? d : ""; });
#if defined(__linux__)
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);  // the only backend built here; gives the specific "no DISPLAY" message
#endif
    if (!glfwInit()) throw std::runtime_error("cannot open a window: " + (g_glfw_error.empty() ? "GLFW init failed" : g_glfw_error));
  }
  ~GlfwInit() { glfwTerminate(); }
};
struct GlfwWindow {
  GLFWwindow* w;
  GlfwWindow(int width, int height) : w(glfwCreateWindow(width, height, "napl", nullptr, nullptr)) {
    if (!w) throw std::runtime_error("cannot open a window: " + (g_glfw_error.empty() ? "glfwCreateWindow failed" : g_glfw_error));
  }
  ~GlfwWindow() { glfwDestroyWindow(w); }
};
void vk_check(VkResult r) {
  if (r != VK_SUCCESS) throw std::runtime_error("ImGui Vulkan backend error (VkResult " + std::to_string(r) + ")");
}
struct ImGuiBackends {
  ImGuiBackends(GLFWwindow* win, const WindowRenderer::Handles& h) : fmt(h.color_format) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // no imgui.ini in the working directory
    ImGui_ImplGlfw_InitForVulkan(win, true);
    ImGui_ImplVulkan_InitInfo ii = {};
    ii.ApiVersion = VK_API_VERSION_1_3;
    ii.Instance = h.instance;
    ii.PhysicalDevice = h.physical_device;
    ii.Device = h.device;
    ii.QueueFamily = h.queue_family;
    ii.Queue = h.queue;
    ii.DescriptorPoolSize = IMGUI_IMPL_VULKAN_MINIMUM_SAMPLED_IMAGE_POOL_SIZE;
    ii.MinImageCount = 2;
    ii.ImageCount = 3;
    ii.UseDynamicRendering = true;
    ii.PipelineInfoMain.PipelineRenderingCreateInfo = {VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR};
    ii.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    ii.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats = &fmt;  // fmt outlives the backend
    ii.CheckVkResultFn = vk_check;
    if (!ImGui_ImplVulkan_Init(&ii)) throw std::runtime_error("ImGui Vulkan backend failed to initialise");
  }
  ~ImGuiBackends() {
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }
  VkFormat fmt;
};

// ---- scene tree --------------------------------------------------------------------------------------------------------
struct Node {
  std::string path;
  bool prim = false;
  std::map<std::string, Node> kids;
};
void add_path(Node& root, const std::string& path) {
  Node* n = &root;
  std::string acc;
  for (std::size_t i = path.find_first_not_of('/'); i != std::string::npos;) {
    const std::size_t j = std::min(path.find('/', i), path.size());
    const std::string seg = path.substr(i, j - i);
    acc += "/" + seg;
    n = &n->kids[seg];
    n->path = acc;
    i = j < path.size() ? j + 1 : std::string::npos;
  }
  n->prim = true;
}

// Attributes the inspector asks for; a prim answers only the ones it has (the rest come back not_found / invalid).
constexpr const char* kAttrs[] = {"pos", "quat", "qpos", "qvel", "ctrl", "kp", "kv", "value"};
bool editable(const std::string& a) { return a == "qpos" || a == "qvel" || a == "ctrl" || a == "kp" || a == "kv"; }

struct Shown {
  std::string attr, unit;
  std::vector<double> values;
};

// Orbit camera around `target` (world frame, +z up).
struct Orbit {
  double yaw = -0.6, pitch = 0.35, dist = 4, target[3] = {0, 0, 0};
  Camera camera() const {
    Camera c;
    c.lookat[0] = target[0];
    c.lookat[1] = target[1];
    c.lookat[2] = target[2];
    c.pos[0] = target[0] + dist * std::cos(pitch) * std::cos(yaw);
    c.pos[1] = target[1] + dist * std::cos(pitch) * std::sin(yaw);
    c.pos[2] = target[2] + dist * std::sin(pitch);
    return c;
  }
  // Frame the movable geoms (planes are unbounded floors: ignored).
  void fit(const std::vector<GeomVisual>& vis, const std::vector<GeomPose>& poses) {
    double sum[3] = {0, 0, 0};
    int n = 0;
    for (std::size_t i = 0; i < vis.size() && i < poses.size(); ++i)
      if (vis[i].type != GeomType::plane && vis[i].group != 3) {
        for (int k = 0; k < 3; ++k) sum[k] += poses[i].pos[k];
        ++n;
      }
    double radius = 1;
    if (n) {
      for (int k = 0; k < 3; ++k) target[k] = sum[k] / n;
      for (std::size_t i = 0; i < vis.size() && i < poses.size(); ++i)
        if (vis[i].type != GeomType::plane && vis[i].group != 3) {
          double d2 = 0;
          for (int k = 0; k < 3; ++k) d2 += (poses[i].pos[k] - target[k]) * (poses[i].pos[k] - target[k]);
          radius = std::max(radius, std::sqrt(d2) + std::max({vis[i].size[0], vis[i].size[1], vis[i].size[2]}));
        }
    }
    dist = 2.5 * radius;
  }
};

class Viewer {
 public:
  Viewer(Host& h, const ViewerOptions& o) : host_(h), opts_(o), script_(parse_script(o.script)) {
    if (!o.design_report.empty()) {
      design_ = design::load_report(o.design_report);  // throws a plain-language error before any window opens
      has_design_ = true;
    }
    for (const ScriptCmd& c : script_)
      if (design_action_args(c.action).first >= 0 && !o.design_api)
        throw std::runtime_error("script: " + c.action + " needs a design editor (Sim.show(design=...) / napl.design.viewer.view)");
  }

  void run() {
    GlfwInit glfw;
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GlfwWindow window(opts_.width, opts_.height);
    if (!glfwVulkanSupported()) throw std::runtime_error("no Vulkan loader found (GLFW could not load libvulkan)");
    WindowRenderer::Surface surf;
    std::uint32_t n = 0;
    const char** exts = glfwGetRequiredInstanceExtensions(&n);
    if (!exts) throw std::runtime_error("this window system cannot present Vulkan images (" + g_glfw_error + ")");
    surf.instance_extensions.assign(exts, exts + n);
    surf.create = [&](VkInstance inst) {
      VkSurfaceKHR s = nullptr;
      const VkResult r = glfwCreateWindowSurface(inst, window.w, nullptr, &s);
      if (r != VK_SUCCESS)
        throw std::runtime_error("cannot create a Vulkan surface for the window (VkResult " + std::to_string(r) + "): " + g_glfw_error);
      return s;
    };
    WindowRenderer renderer(surf);
    if (opts_.design_api) {  // the build view looks like a studio; NAPL_RENDER_STYLE still overrides
      RenderStyle st = default_render_style();
      st.sky = RenderStyle::Sky::studio;
      set_default_render_style(st);
    }
    ImGuiBackends gui(window.w, renderer.handles());

    if (opts_.design_api) d_state(true);  // the design behind the model on screen (quiet: an MCP server may not have one yet)
    for (int n_frame = 0; !glfwWindowShouldClose(window.w) && !(opts_.frames > 0 && n_frame >= opts_.frames) &&
                          !(opts_.stop && opts_.stop->load());
         ++n_frame) {
      glfwPollEvents();
      for (const ScriptCmd& c : script_) {
        if (c.frame != n_frame) continue;
        if (c.action == "select") select(c.args[0]);
        else if (c.action == "inst") select_instance(c.args[0]);
        else if (design_action_args(c.action).first >= 0) script_design(c);
        else send(script_envelope(c, kSource), c.action);
      }
      Frame f;
      if (host_.take_frame(f)) adopt(f, renderer);
      poll();

      ImGui_ImplVulkan_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      ui();
      mouse();
      ImGui::Render();

      int fw = 0, fh = 0;
      glfwGetFramebufferSize(window.w, &fw, &fh);
      renderer.set_tubes(cur_.tubes);
      const bool drawn = renderer.frame(cur_.poses, orbit_.camera(), fw, fh, [](VkCommandBuffer cb) {
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cb);
      });
      if (!drawn) glfwWaitEventsTimeout(0.05);  // minimised or swapchain rebuilding: do not spin
    }
    draining_ = true;  // edits still queued or running finish (and their model reloads are issued) before we return
    while (dfut_.valid() || !dq_.empty()) {
      poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (auto& p : cmds_) p.second.wait();  // scripted / UI commands issued last are applied before we return
    poll();
  }

 private:
  void send(Env e, const std::string& what, bool elevate = false) { cmds_.emplace_back(what, host_.submit(std::move(e), elevate)); }

  template <class T>
  static bool ready(std::future<T>& f) { return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }

  void request_prims() { prims_ = host_.submit(env_query(kSource, "", "")); }

  void adopt(Frame& f, WindowRenderer& renderer) {
    if (f.new_scene) {
      renderer.set_scene(f.visuals);
      if (!keep_camera_) orbit_.fit(f.visuals, f.poses);  // a Design-window rebuild keeps the camera where the person put it
    }
    if (f.scene != scene_) {  // first frame, or the model was replaced: the prim list changed too
      const design::Instance* si = keep_camera_ ? sel_instance() : nullptr;
      const std::string keep = si ? si->id : "";
      if (scene_ != 0) select("");  // (a first-frame selection, e.g. scripted, is kept)
      scene_ = f.scene;
      request_prims();
      keep_camera_ = false;
      if (!keep.empty()) pending_body_ = keep;  // the selected instance stays selected once the new prim list arrives
    }
    f.visuals.clear();
    cur_ = std::move(f);
  }

  // Collects finished replies without blocking the frame.
  void poll() {
    design_poll();
    for (auto it = cmds_.begin(); it != cmds_.end();) {
      if (!ready(it->second)) {
        ++it;
        continue;
      }
      const Reply r = it->second.get();
      const napl::Reply* rep = flatbuffers::GetRoot<napl::Reply>(r.data());
      status_ = rep->ok() ? "" : it->first + ": " + napl::EnumNameErr(rep->error()) + ": " + (rep->message() ? rep->message()->str() : "");
      it = cmds_.erase(it);
    }
    if (prims_.valid() && ready(prims_)) {
      const Reply r = prims_.get();
      tree_ = Node{};
      prim_paths_.clear();
      if (const auto* p = flatbuffers::GetRoot<napl::Reply>(r.data())->prims())
        for (const auto* s : *p) {
          add_path(tree_, s->str());
          prim_paths_.push_back(s->str());
        }
      if (!pending_body_.empty()) select_instance(pending_body_);
    }
    if (!inspect_.empty() && std::all_of(inspect_.begin(), inspect_.end(), [](auto& p) { return ready(p.second); })) {
      shown_.clear();
      for (auto& [attr, fut] : inspect_) {
        const Reply r = fut.get();
        const napl::Reply* rep = flatbuffers::GetRoot<napl::Reply>(r.data());
        if (!rep->ok() || !rep->values()) continue;  // this prim has no such attribute
        shown_.push_back({attr, rep->unit() ? rep->unit()->str() : "", {rep->values()->begin(), rep->values()->end()}});
      }
      inspect_.clear();
    }
    if (inspect_.empty() && !selected_.empty())
      for (const char* a : kAttrs) inspect_.emplace_back(a, host_.submit(env_query(kSource, selected_, a)));
  }

  void mouse() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      orbit_.yaw -= 0.01 * io.MouseDelta.x;
      orbit_.pitch = std::clamp(orbit_.pitch + 0.01 * io.MouseDelta.y, -1.5, 1.5);
    }
    if (io.MouseWheel != 0) orbit_.dist = std::max(0.05, orbit_.dist * std::pow(0.9, io.MouseWheel));
  }

  void select(const std::string& path) {
    selected_ = path;
    shown_.clear();
    inspect_.clear();
  }

  // A Design-window click: selects the body prim of design instance `inst` (by id or body name). Before the prim list
  // has arrived the request is parked and retried when it does.
  void select_instance(const std::string& inst) {
    std::string body = inst;
    for (const design::Instance& i : design_.instances)
      if (i.id == inst) body = i.body;
    const std::string p = design::body_path(prim_paths_, body);
    pending_body_ = p.empty() && prim_paths_.empty() ? inst : "";
    if (!p.empty()) select(p);
  }

  // The design instance whose body prim is selected (nullptr: none / a prim that is not a design instance).
  const design::Instance* sel_instance() const {
    for (const design::Instance& i : design_.instances)
      if (selected_.size() > i.body.size() && selected_.ends_with("/" + i.body)) return &i;
    return nullptr;
  }

  // ---- Design editing: the viewer builds requests and shows replies; napl.design.Design (via opts_.design_api) does the work ----
  // One request at a time on a worker thread; a queue keeps UI clicks and scripted actions in order. `make` builds the request
  // when it reaches the head of the queue (so "cycle placement" sees the placement of the snap queued before it); an empty
  // request is a local action only.
  struct DReq {
    std::string kind;
    bool quiet = false;                   // a failure is not shown (polls, the first state)
    std::function<std::string()> make;
  };
  struct Alt {
    std::string a, b;  // port on the target instance / port on the new part
    int k = 0;
    double score = 0;
    bool interferes = false;
    std::vector<std::string> why;
  };
  struct Placement {
    std::string part, id, target;  // target = instance the part is snapped to
    std::vector<Alt> alts;         // [0] = the placement the snap chose, then the ranked alternatives
    int cur = 0;
  };
  struct Sugg {
    std::string part, port, category, name;
    double score = 0, mass = 0, mass_delta = 0;
    int kept = 0, dangling = 0;
    std::vector<std::string> why, lost;
  };
  struct Decision {  // an ambiguous autowire: the user decides which port goes to which candidate
    std::vector<std::string> ports, cands;
    std::string why;
  };

  static std::vector<std::string> strs(const design::Json& j) {
    std::vector<std::string> v;
    for (const design::Json& e : j.arr) v.push_back(e.kind == design::Json::Str ? e.str : "");
    return v;
  }
  static std::string join(const std::vector<std::string>& v, const char* sep) {
    std::string o;
    for (const std::string& s : v) o += (o.empty() ? "" : sep) + s;
    return o;
  }
  static std::string key(const std::string& code, const std::vector<std::string>& where) { return code + "|" + join(where, ","); }
  static std::string q(const std::string& s) { return design::quote(s); }

  void dreq(const std::string& kind, std::function<std::string()> make, bool quiet = false) {
    dq_.push_back({kind, quiet, std::move(make)});
  }
  void dreq(const std::string& kind, std::string json, bool quiet = false) {
    dreq(kind, [j = std::move(json)] { return j; }, quiet);
  }
  static std::string edit_req(const std::string& op, bool replace_last = false) {
    return design::request("design_edit", "{\"ops\":[" + op + "]}", replace_last);
  }

  void d_state(bool quiet = false) { dreq("state", design::request("state"), quiet); }
  void d_suggest(const std::string& target, const std::string& category) {
    sugg_target_ = target;
    sugg_swap_ = target.find('.') == std::string::npos;
    suggs_.clear();
    dreq("suggest", design::request("design_suggest", "{\"target\":" + q(target) + ",\"limit\":12" +
                                                         (category.empty() ? "" : ",\"category\":" + q(category)) + "}"));
  }
  // "snap here": add `part` mated to the best open port pair of `to` ('inst' or 'inst.port'); child_port pins the new part's port.
  void d_snap(const std::string& part, const std::string& to, const std::string& id = "", const std::string& child_port = "") {
    place_ = Placement{part, id, to.substr(0, to.find('.')), {}, 0};
    dreq("edit", edit_req("{\"op\":\"add\",\"part\":" + q(part) + ",\"to\":" + q(to) + (id.empty() ? "" : ",\"id\":" + q(id)) +
                          (child_port.empty() ? "" : ",\"port\":" + q(child_port)) + "}"));
  }
  // Preview placement alternative n of the last snap: undo it and add again with that port pair and rotation (one undo step).
  void d_cycle(int n) {
    dreq("edit", [this, n]() -> std::string {
      if (n < 0 || n >= static_cast<int>(place_.alts.size())) {
        dstatus_ = "cycle: no placement alternative " + std::to_string(n);
        return "";
      }
      const Alt& a = place_.alts[n];
      cycling_ = n;
      return edit_req("{\"op\":\"add\",\"part\":" + q(place_.part) + ",\"id\":" + q(place_.id) + ",\"to\":" + q(place_.target + "." + a.a) +
                          ",\"port\":" + q(a.b) + ",\"k\":" + std::to_string(a.k) + "}", true);
    });
  }
  void d_swap(const std::string& id, const std::string& part, bool auto_dev) {
    dreq("edit", edit_req("{\"op\":\"swap\",\"id\":" + q(id) + ",\"part\":" + q(part) + ",\"auto\":" + (auto_dev ? "true" : "false") + "}"));
  }
  void d_remove(const std::string& id) { dreq("edit", edit_req("{\"op\":\"remove\",\"id\":" + q(id) + "}")); }
  void d_autowire(const std::string& port, const std::string& to = "") {
    dreq("edit", edit_req("{\"op\":\"autowire\",\"a\":" + q(port) + (to.empty() ? "" : ",\"b\":" + q(to)) + "}"));
  }
  void d_autowire_all() { dreq("edit", edit_req("{\"op\":\"autowire_all\"}")); }
  void d_undo(bool redo) { dreq("edit", design::request("design_undo", redo ? "{\"redo\":true}" : "{}")); }
  void d_save(const std::string& path) { dreq("save", design::request("design_save", path.empty() ? "{}" : "{\"path\":" + q(path) + "}")); }
  void d_simulate(const std::string& kind) {
    sim_rows_.clear();
    sim_title_ = kind + ": running...";
    dreq("sim", design::request("design_simulate", "{\"kind\":" + q(kind) + "}"));
  }
  // Click finding n of the DRC list: select the instances it involves (the first in the inspector, all marked in the lists).
  void d_finding(int n) {
    dreq("local", [this, n]() -> std::string {
      if (n < 0 || n >= static_cast<int>(design_.findings.size())) {
        dstatus_ = "finding " + std::to_string(n) + ": no such finding";
        return "";
      }
      finding_ = n;
      hl_.clear();
      const design::Finding& f = design_.findings[n];
      for (const std::string& i : f.instances) hl_.insert(i);
      if (!f.instances.empty()) select_instance(f.instances[0]);
      return "";
    });
  }
  void script_design(const ScriptCmd& c) {
    const auto& a = c.args;
    const std::string arg1 = a.size() > 1 ? a[1] : "";
    if (c.action == "dsuggest") d_suggest(a[0], arg1);
    else if (c.action == "dsnap") d_snap(a[0], a[1], a.size() > 2 ? a[2] : "");
    else if (c.action == "dcycle") d_cycle(static_cast<int>(to_double(a[0], "N")));
    else if (c.action == "dswap") d_swap(a[0], a[1], true);
    else if (c.action == "dremove") d_remove(a[0]);
    else if (c.action == "dautowire") d_autowire(a[0], arg1);
    else if (c.action == "dautowire_all") d_autowire_all();
    else if (c.action == "dundo") d_undo(false);
    else if (c.action == "dredo") d_undo(true);
    else if (c.action == "dsave") d_save(a.empty() ? "" : a[0]);
    else if (c.action == "dsim") d_simulate(a[0]);
    else if (c.action == "dfinding") d_finding(static_cast<int>(to_double(a[0], "N")));
    else if (c.action == "dsync") d_state();
  }

  // Runs on the viewer thread each frame: collects the reply of the request in flight, starts the next one, polls for changes
  // another client (an MCP agent sharing the design) made.
  void design_poll() {
    if (!opts_.design_api) return;
    for (;;) {
      if (dfut_.valid()) {
        if (!ready(dfut_)) return;
        const std::string reply = dfut_.get();
        design_done(dcur_, reply);
      }
      if (dq_.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (!draining_ && now - last_poll_ > (flat_.active() ? std::chrono::milliseconds(250) : std::chrono::milliseconds(1000))) {
          last_poll_ = now;
          dq_.push_back({"rev", true, [] { return design::request("rev"); }});
        } else {
          return;
        }
      }
      dcur_ = std::move(dq_.front());
      dq_.pop_front();
      const std::string json = dcur_.make();
      if (json.empty()) continue;  // a local action (or one that could not be built: dstatus_ says why)
      dfut_ = std::async(std::launch::async, [api = opts_.design_api, json] {
        try {
          return api(json);
        } catch (const std::exception& e) {
          return std::string("{\"ok\":false,\"error\":") + design::quote(e.what()) + "}";
        }
      });
    }
  }

  void design_done(const DReq& rq, const std::string& text) {
    design::Json j;
    try {
      j = design::Parser(text).parse();
    } catch (const std::exception& e) {
      dstatus_ = rq.kind + ": " + e.what();
      return;
    }
    if (rq.kind.rfind("flatten.", 0) == 0) {  // the 2D wiring canvas (wire2d.cpp) owns these replies
      flat_.on_reply(rq.kind, j);
      if (j.at("ok").num != 0 && j.at("state").kind == design::Json::Obj) apply_state(j.at("state"));
      return;
    }
    if (j.at("ok").num == 0) {
      if (!rq.quiet) dstatus_ = rq.kind + ": " + j.s("error");
      if (rq.kind == "edit") cycling_ = -1;
      return;
    }
    dstatus_.clear();
    const design::Json& res = j.at("result");
    if (rq.kind == "rev") {
      flat_.poll(static_cast<int>(res.n("flat_rev")));
      if (!res.at("rev").str.empty() && res.at("rev").str != dstate_.rev) d_state(true);
      return;
    }
    if (j.at("state").kind == design::Json::Obj) apply_state(j.at("state"));
    if (rq.kind == "suggest") {
      want_tab_ = 0;
      suggs_.clear();
      for (const design::Json& r : res.at("suggestions").arr)
        suggs_.push_back({r.s("part"), r.s("port"), r.s("category"), r.s("name"), r.n("score"), r.n("mass"), r.n("mass_delta"),
                          static_cast<int>(r.n("mates_kept")), static_cast<int>(r.at("dangling").arr.size()), strs(r.at("why")),
                          strs(r.at("mates_lost"))});
    } else if (rq.kind == "sim") {
      want_tab_ = 2;
      sim_title_ = res.s("kind");
      sim_rows_.clear();
      for (const auto& [k, v] : res.obj) {
        char buf[64];
        if (v.kind == design::Json::Num) std::snprintf(buf, sizeof buf, "%.5g", v.num);
        else if (v.kind == design::Json::Bool) std::snprintf(buf, sizeof buf, "%s", v.num != 0 ? "true" : "false");
        else continue;  // lists / nested tables / prose: not a quick-check number
        sim_rows_.emplace_back(k, buf);
      }
    } else if (rq.kind == "save") {
      saved_ = res.s("saved");
    } else if (rq.kind == "edit") {
      edit_done(res);
    }
  }

  void edit_done(const design::Json& res) {
    const design::Json& drc = res.at("drc");
    new_keys_.clear();
    for (const design::Json& f : drc.at("new").arr) new_keys_.insert(key(f.s("code"), strs(f.at("where"))));
    const auto counts = [](const design::Json& c) {
      return std::to_string(static_cast<int>(c.n("error"))) + " errors, " + std::to_string(static_cast<int>(c.n("warning"))) + " warnings";
    };
    edit_summary_ = "DRC " + counts(drc.at("after")) + " (before: " + counts(drc.at("before")) + "); " + std::to_string(drc.at("new").arr.size()) +
                    " new, " + std::to_string(drc.at("resolved").arr.size()) + " resolved";
    static const design::Json none;
    const design::Json& ch = res.at("changes").arr.empty() ? none : res.at("changes").arr[0];
    const std::string op = ch.s("op");
    edit_lines_.clear();
    want_tab_ = op == "autowire" || op == "autowire_all" ? 1 : op == "add" || op == "swap" ? 0 : -1;
    if (op != "autowire" && op != "autowire_all") decisions_.clear();  // (a pick is an autowire: the rest of its decisions stay)
    if (op != "add") place_.alts.clear();  // cycling undoes the last edit: only valid right after the snap
    if (op == "add") {
      const design::Json& snap = ch.at("snap");
      if (cycling_ < 0 || place_.alts.empty()) {  // a fresh snap (not a cycle through the alternatives of the last one)
        place_.id = ch.s("id");
        place_.alts.clear();
        const design::Json& m = ch.at("mate");
        const auto port_of = [](const std::string& ref) { return ref.substr(ref.find('.') == std::string::npos ? ref.size() : ref.find('.') + 1); };
        place_.alts.push_back({port_of(m.s("a")), port_of(m.s("b")), static_cast<int>(snap.n("k")), snap.n("score"),
                               !snap.at("interferes").arr.empty(), strs(snap.at("why"))});
        for (const design::Json& a : snap.at("alternatives").arr)  // {to: 'inst.port', port, k, score, why, interferes}
          place_.alts.push_back({a.s("to").empty() ? a.s("a") : port_of(a.s("to")), a.s("port").empty() ? a.s("b") : a.s("port"),
                                 static_cast<int>(a.n("k")), a.n("score"), !a.at("interferes").arr.empty(), strs(a.at("why"))});
        place_.cur = 0;
      } else {
        place_.cur = cycling_;
      }
      cycling_ = -1;
      edit_lines_.push_back("added " + ch.s("id") + " (" + ch.s("part") + ")" + (ch.s("note").empty() ? "" : ": " + ch.s("note")));
    } else if (op == "swap") {
      edit_lines_.push_back("swapped " + ch.s("id") + ": " + ch.s("from") + " -> " + ch.s("to"));
      for (const std::string& d : strs(ch.at("dangling"))) edit_lines_.push_back("dangling: " + d);
      const design::Json& ad = ch.at("auto_devices");
      if (ad.kind == design::Json::Obj)
        edit_lines_.push_back("devices: " + std::to_string(ad.at("created").arr.size()) + " created, " + std::to_string(ad.at("refreshed").arr.size()) +
                              " refreshed, " + std::to_string(ad.at("unset").arr.size()) + " with unset fields");
    } else if (op == "remove") {
      edit_lines_.push_back("removed " + ch.s("id") + " (" + std::to_string(ch.at("mates_removed").arr.size()) + " mates, " +
                            std::to_string(ch.at("wires_removed").arr.size()) + " wires dropped)");
    } else if (op == "autowire") {
      const design::Json& w = ch.at("wire");
      edit_lines_.push_back("wired " + w.s("from") + " -> " + w.s("to") + " with " + w.s("part") + ", " +
                            std::to_string(w.at("length").n("value")).substr(0, 6) + " m");
      edit_lines_.push_back("why: " + ch.at("reasons").s("part"));
    } else if (op == "autowire_all") {
      for (const design::Json& w : ch.at("wired").arr)
        edit_lines_.push_back("wired " + w.at("wire").s("from") + " -> " + w.at("wire").s("to") + " (" + w.at("wire").s("part") + ")");
      decisions_.clear();
      for (const design::Json& a : ch.at("ambiguous").arr) decisions_.push_back({strs(a.at("ports")), strs(a.at("candidates")), a.s("why")});
      for (const design::Json& u : ch.at("unmatched").arr) edit_lines_.push_back("unmatched " + u.s("port") + ": " + u.s("why"));
    } else if (res.at("drc").kind == design::Json::Obj && ch.s("op").empty()) {
      edit_lines_.push_back("no change record");
    } else {
      edit_lines_.push_back(op);  // undo / redo
    }
  }

  // The backend's state after a request: shows its report, and when the built model changed loads it into the World.
  void apply_state(const design::Json& js) {
    try {
      dstate_ = design::state_from(js);
    } catch (const std::exception& e) {
      dstatus_ = std::string("state: ") + e.what();
      return;
    }
    design_ = dstate_.report;
    has_design_ = true;
    if (finding_ >= static_cast<int>(design_.findings.size())) {
      finding_ = -1;
      hl_.clear();
    }
    if (loaded_rev_.empty()) loaded_rev_ = dstate_.rev;  // the first state describes the model already on screen
    if (dstate_.rev != loaded_rev_ && !dstate_.xml.empty()) {
      loaded_rev_ = dstate_.rev;
      keep_camera_ = true;
      send(env_load(kSource, dstate_.xml), "load " + dstate_.xml, true);
    }
  }

  void instance_list() {
    std::map<std::string, std::vector<const design::Instance*>> by_group;
    for (const design::Instance& i : design_.instances) by_group[i.group.empty() ? "(none)" : i.group].push_back(&i);
    const design::Instance* sel = sel_instance();
    for (const auto& [g, list] : by_group) {
      if (!ImGui::TreeNodeEx(g.c_str(), ImGuiTreeNodeFlags_DefaultOpen, "%s (%zu)", g.c_str(), list.size())) continue;
      for (const design::Instance* i : list) {
        const std::string label = i->id + "   " + i->part + "   " + std::to_string(i->mass * 1000).substr(0, 7) + " g";
        const bool hl = hl_.count(i->id) > 0;
        if (hl) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{1, 0.85f, 0.2f, 1});
        if (ImGui::Selectable(label.c_str(), sel == i)) select_instance(i->id);
        if (hl) ImGui::PopStyleColor();
      }
      ImGui::TreePop();
    }
  }

  void why_tip(const std::vector<std::string>& why) const {
    if (why.empty() || !ImGui::BeginItemTooltip()) return;
    for (const std::string& w : why) ImGui::BulletText("%s", w.c_str());
    ImGui::EndTooltip();
  }

  // Library tab: parts for the selected port, snap with ranked placements, swap.
  void library_tab(const design::Instance* sel) {
    if (!sel) {
      ImGui::TextUnformatted("select an instance (list above, or a body in the scene tree)");
      return;
    }
    const std::vector<design::Port>& ports = dstate_.ports.count(sel->id) ? dstate_.ports.at(sel->id) : std::vector<design::Port>{};
    if (sel_port_inst_ != sel->id) {
      sel_port_inst_ = sel->id;
      sel_port_.clear();
    }
    ImGui::Text("%s   %s", sel->id.c_str(), sel->part.c_str());
    ImGui::SetNextItemWidth(260);
    if (ImGui::BeginCombo("port", sel_port_.empty() ? "(any open port)" : sel_port_.c_str())) {
      if (ImGui::Selectable("(any open port)", sel_port_.empty())) sel_port_.clear();
      for (const design::Port& p : ports) {
        const std::string label = p.name + "   " + p.kind + " " + p.type + (p.used ? "   (in use)" : "");
        if (ImGui::Selectable(label.c_str(), sel_port_ == p.name)) sel_port_ = p.name;
      }
      ImGui::EndCombo();
    }
    const std::string target = sel->id + (sel_port_.empty() ? "" : "." + sel_port_);
    ImGui::SetNextItemWidth(160);
    const bool enter = ImGui::InputText("category", cat_buf_, sizeof cat_buf_, ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SetItemTooltip("dotted category prefix, e.g. esc, sensor.gnss, motor.bldc (empty: all)");
    ImGui::SameLine();
    if (ImGui::Button("suggest for port") || enter) d_suggest(target, cat_buf_);
    ImGui::SameLine();
    if (ImGui::Button("suggest swaps")) d_suggest(sel->id, "");
    ImGui::SetNextItemWidth(260);
    ImGui::InputText("part id", part_buf_, sizeof part_buf_);
    ImGui::SameLine();
    ImGui::BeginDisabled(!part_buf_[0]);
    if (ImGui::Button("snap here")) d_snap(part_buf_, target);
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("add this part mated to the best open port pair of %s", target.c_str());
    if (!suggs_.empty()) {
      ImGui::Separator();
      ImGui::Text("%s for %s", sugg_swap_ ? "swap candidates" : "library parts", sugg_target_.c_str());
      if (sugg_swap_) ImGui::Checkbox("refresh device bindings (auto)", &auto_dev_);
      if (ImGui::BeginTable("sugg", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, {0, 100})) {
        ImGui::TableSetupColumn("part", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(sugg_swap_ ? "mates kept/lost" : "port");
        ImGui::TableSetupColumn(sugg_swap_ ? "dangling" : "score");
        ImGui::TableSetupColumn(sugg_swap_ ? "d mass g" : "mass g");
        ImGui::TableSetupColumn("##go", ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableHeadersRow();
        int n = 0;
        for (const Sugg& s : suggs_) {
          ImGui::PushID(n++);
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(s.part.c_str());
          why_tip(s.why);
          if (ImGui::IsItemHovered() && !s.name.empty()) ImGui::SetTooltip("%s (%s)", s.name.c_str(), s.category.c_str());
          ImGui::TableNextColumn();
          if (sugg_swap_) ImGui::Text("%d / %zu", s.kept, s.lost.size());
          else ImGui::TextUnformatted(s.port.c_str());
          ImGui::TableNextColumn();
          if (sugg_swap_) ImGui::Text("%d", s.dangling);
          else ImGui::Text("%.0f", s.score);
          ImGui::TableNextColumn();
          ImGui::Text("%.1f", (sugg_swap_ ? s.mass_delta : s.mass) * 1000);
          ImGui::TableNextColumn();
          if (ImGui::SmallButton(sugg_swap_ ? "swap" : "snap here")) {
            if (sugg_swap_) d_swap(sugg_target_, s.part, auto_dev_);
            else d_snap(s.part, sugg_target_, "", s.port);
          }
          ImGui::PopID();
        }
        ImGui::EndTable();
      }
    }
    placement_ui();
  }

  void placement_ui() {
    if (place_.alts.empty()) return;
    ImGui::Separator();
    ImGui::Text("placement of %s on %s: %zu candidate(s)", place_.id.c_str(), place_.target.c_str(), place_.alts.size());
    ImGui::BeginDisabled(place_.alts.size() < 2);
    const int n = static_cast<int>(place_.alts.size());
    if (ImGui::SmallButton("< prev")) d_cycle((place_.cur + n - 1) % n);
    ImGui::SameLine();
    if (ImGui::SmallButton("next >")) d_cycle((place_.cur + 1) % n);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("(each is applied as the edit; undo drops it)");
    if (ImGui::BeginTable("alts", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, {0, 100})) {
      ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 24);
      ImGui::TableSetupColumn("ports", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthFixed, 30);
      ImGui::TableSetupColumn("score", ImGuiTableColumnFlags_WidthFixed, 80);
      ImGui::TableHeadersRow();
      for (int i = 0; i < n; ++i) {
        const Alt& a = place_.alts[i];
        ImGui::PushID(i);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        if (a.interferes) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{1, 0.4f, 0.4f, 1});
        if (ImGui::Selectable(std::to_string(i).c_str(), i == place_.cur, ImGuiSelectableFlags_SpanAllColumns)) {
          if (i != place_.cur) d_cycle(i);
        }
        why_tip(a.why);
        ImGui::TableNextColumn();
        ImGui::Text("%s.%s <-> %s.%s", place_.target.c_str(), a.a.c_str(), place_.id.c_str(), a.b.c_str());
        ImGui::TableNextColumn();
        ImGui::Text("%d", a.k);
        ImGui::TableNextColumn();
        ImGui::Text("%.1f%s", a.score, a.interferes ? " X" : "");
        if (a.interferes) ImGui::PopStyleColor();
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
  }

  void wiring_tab(const design::Instance* sel) {
    const std::string port = sel && !sel_port_.empty() && sel_port_inst_ == sel->id ? sel->id + "." + sel_port_ : "";
    ImGui::BeginDisabled(port.empty());
    if (ImGui::Button(("autowire " + (port.empty() ? std::string("selected port") : port)).c_str())) d_autowire(port);
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("pick the port in the Library tab; wire part and length are chosen by the Design API");
    ImGui::SameLine();
    if (ImGui::Button("autowire all")) d_autowire_all();
    ImGui::SetItemTooltip("wires every open port that has one sensible target; the ambiguous ones are listed for you to pick");
    if (!decisions_.empty()) {
      ImGui::Separator();
      ImGui::TextColored({1, 0.8f, 0.3f, 1}, "%zu decision(s) for you (nothing was guessed)", decisions_.size());
    }
    for (std::size_t di = 0; di < decisions_.size(); ++di) {
      Decision& d = decisions_[di];
      if (d.ports.empty()) continue;
      ImGui::PushID(static_cast<int>(di));
      ImGui::TextWrapped("%s", d.why.c_str());
      for (std::size_t pi = 0; pi < d.ports.size(); ++pi) {
        ImGui::PushID(static_cast<int>(pi));
        ImGui::SetNextItemWidth(220);
        if (ImGui::BeginCombo(("##" + d.ports[pi]).c_str(), ("wire " + d.ports[pi] + " to ...").c_str())) {
          for (const std::string& c : d.cands)
            if (ImGui::Selectable(c.c_str())) {
              d_autowire(d.ports[pi], c);
              const std::string p = d.ports[pi];
              d.cands.erase(std::remove(d.cands.begin(), d.cands.end(), c), d.cands.end());  // taken
              d.ports.erase(d.ports.begin() + static_cast<long>(pi));
              ImGui::EndCombo();
              ImGui::PopID();
              ImGui::PopID();
              return;
            }
          ImGui::EndCombo();
        }
        ImGui::PopID();
      }
      ImGui::PopID();
    }
  }

  void simulate_tab() {
    if (ImGui::Button("hover")) d_simulate("hover");
    ImGui::SetItemTooltip("multirotor: built-in altitude + attitude controller, 2 m for 10 s (the Design API's simulate)");
    ImGui::SameLine();
    if (ImGui::Button("hold")) d_simulate("hold");
    ImGui::SetItemTooltip("arm: servos torque-on at a pose, reports sag and current");
    if (dfut_.valid() && dcur_.kind == "sim") ImGui::TextUnformatted("running...");
    else if (!sim_title_.empty()) ImGui::Text("last: %s", sim_title_.c_str());
    if (ImGui::BeginTable("sim", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, {0, 220})) {
      for (const auto& [k, v] : sim_rows_) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(k.c_str());
        ImGui::TableNextColumn();
        const bool bad = k == "pass" && v == "false";
        if (bad) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{1, 0.35f, 0.35f, 1});
        ImGui::TextUnformatted(v.c_str());
        if (bad) ImGui::PopStyleColor();
      }
      ImGui::EndTable();
    }
  }

  void design_ui() {
    const bool edit = static_cast<bool>(opts_.design_api);
    ImGui::SetNextWindowPos(edit ? ImVec2{640, 10} : ImVec2{10, 440}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(edit ? ImVec2{630, 700} : ImVec2{440, 300}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Design");
    ImGui::Text("%s   total %.4f kg   %zu instances", design_.assembly.c_str(), design_.mass_total, design_.instances.size());
    if (edit) {
      ImGui::BeginDisabled(dstate_.undo == 0);
      if (ImGui::Button(("undo (" + std::to_string(dstate_.undo) + ")###undo").c_str())) d_undo(false);
      ImGui::EndDisabled();
      ImGui::SameLine();
      ImGui::BeginDisabled(dstate_.redo == 0);
      if (ImGui::Button(("redo (" + std::to_string(dstate_.redo) + ")###redo").c_str())) d_undo(true);
      ImGui::EndDisabled();
      ImGui::SameLine();
      const design::Instance* sel = sel_instance();
      ImGui::BeginDisabled(!sel);
      if (ImGui::Button("remove selected") && sel) d_remove(sel->id);
      ImGui::EndDisabled();
      ImGui::SetItemTooltip("removes the instance with the mates, wires and device bindings that name it");
      ImGui::SameLine();
      if (ImGui::Button("refresh")) d_state();
      ImGui::SameLine();
      if (ImGui::Button("save")) d_save(save_buf_);
      ImGui::SetItemTooltip("writes the design TOML (comments of a hand-written file are not kept); empty path = the file it was opened from");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(180);
      if (save_buf_[0] == 0 && !dstate_.path.empty()) std::snprintf(save_buf_, sizeof save_buf_, "%s", dstate_.path.c_str());
      ImGui::InputText("##savepath", save_buf_, sizeof save_buf_);
      if (!saved_.empty()) ImGui::TextDisabled("saved %s", saved_.c_str());
      if (dfut_.valid()) ImGui::TextDisabled("working: %s ...", dcur_.kind.c_str());
      if (!dstatus_.empty()) ImGui::TextColored({1, 0.45f, 0.3f, 1}, "%s", dstatus_.c_str());
      if (!dstate_.build_error.empty()) ImGui::TextColored({1, 0.45f, 0.3f, 1}, "does not build (model not updated): %s", dstate_.build_error.c_str());
      if (!edit_summary_.empty()) ImGui::TextWrapped("last edit: %s", edit_summary_.c_str());
      for (std::size_t i = 0; i < edit_lines_.size() && i < 6; ++i) ImGui::TextWrapped("  %s", edit_lines_[i].c_str());
      if (edit_lines_.size() > 6) ImGui::TextDisabled("  ... %zu more", edit_lines_.size() - 6);
    }
    if (ImGui::CollapsingHeader("mass budget")) {
      for (const auto& [g, m] : design_.by_group) ImGui::Text("%-16s %9.4f kg", g.c_str(), m);
      for (const design::Target& t : design_.targets) {
        ImGui::TextColored(t.within ? ImVec4{0.5f, 1, 0.5f, 1} : ImVec4{1, 0.6f, 0.2f, 1}, "target %s: %.4f vs %.4f kg (%+.4f) %s",
                           t.name.c_str(), t.model, t.target, t.gap, t.within ? "within" : "OUTSIDE");
      }
    }
    if (ImGui::CollapsingHeader("instances", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::BeginChild("instances", {0, edit ? 115.f : 0.f}, ImGuiChildFlags_Borders);
      instance_list();
      ImGui::EndChild();
    }
    if (edit && ImGui::BeginTabBar("edit")) {
      const design::Instance* sel = sel_instance();
      const auto flag = [&](int i) { return want_tab_ == i ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None; };
      if (ImGui::BeginTabItem("Library", nullptr, flag(0))) {
        library_tab(sel);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Wiring", nullptr, flag(1))) {
        wiring_tab(sel);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Simulate", nullptr, flag(2))) {
        simulate_tab();
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
      want_tab_ = -1;
    }
    ImGui::End();
    drc_ui(edit);
  }

  // The DRC panel: the findings of the design as it is now, re-read after every edit.
  void drc_ui(bool edit) {
    ImGui::SetNextWindowPos(edit ? ImVec2{10, 395} : ImVec2{460, 440}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(edit ? ImVec2{620, 315} : ImVec2{440, 300}, ImGuiCond_FirstUseEver);
    std::size_t errors = 0, warns = 0, infos = 0;
    for (const design::Finding& f : design_.findings) {
      if (f.severity == "error") ++errors;
      else if (f.severity == "warning") ++warns;
      else ++infos;
    }
    ImGui::Begin("DRC");
    ImGui::TextColored(errors ? ImVec4{1, 0.35f, 0.35f, 1} : ImVec4{0.5f, 1, 0.5f, 1}, "%zu errors", errors);
    ImGui::SameLine();
    ImGui::TextColored({1, 0.8f, 0.3f, 1}, "%zu warnings", warns);
    ImGui::SameLine();
    ImGui::TextColored({0.7f, 0.8f, 1, 1}, "%zu info", infos);
    if (!new_keys_.empty()) {
      ImGui::SameLine();
      ImGui::Text("   %zu new from the last edit", new_keys_.size());
    }
    if (ImGui::BeginTable("drc", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, {0, edit ? 190.f : 0.f})) {
      ImGui::TableSetupColumn("sev", ImGuiTableColumnFlags_WidthFixed, 56);
      ImGui::TableSetupColumn("code / where", ImGuiTableColumnFlags_WidthFixed, 190);
      ImGui::TableSetupColumn("message", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableHeadersRow();
      int n = 0;
      for (const design::Finding& f : design_.findings) {
        const ImVec4 col = f.severity == "error" ? ImVec4{1, 0.35f, 0.35f, 1}
                           : f.severity == "warning" ? ImVec4{1, 0.8f, 0.3f, 1}
                                                     : ImVec4{0.7f, 0.8f, 1, 1};
        ImGui::PushID(n);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text, col);
        const bool fresh = new_keys_.count(key(f.code, f.where)) > 0;
        if (ImGui::Selectable((f.severity.substr(0, 4) + (fresh ? "*" : "")).c_str(), finding_ == n, ImGuiSelectableFlags_SpanAllColumns)) d_finding(n);
        ImGui::PopStyleColor();
        if (!f.fix.empty()) ImGui::SetItemTooltip("fix: %s", f.fix.c_str());
        ImGui::TableNextColumn();
        ImGui::TextUnformatted((f.code + " " + join(f.instances.empty() ? f.where : f.instances, ",")).c_str());
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(f.message.c_str());
        ImGui::PopID();
        ++n;
      }
      ImGui::EndTable();
    }
    if (finding_ >= 0 && finding_ < static_cast<int>(design_.findings.size())) {
      const design::Finding& f = design_.findings[finding_];
      ImGui::Separator();
      ImGui::TextWrapped("%s", f.message.c_str());
      if (!f.fix.empty()) ImGui::TextWrapped("fix: %s", f.fix.c_str());
      for (const std::string& i : f.instances) {
        ImGui::SameLine();
        if (ImGui::SmallButton(i.c_str())) select_instance(i);
      }
    }
    ImGui::End();
  }

  void draw_node(const std::string& name, const Node& n) {
    ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
    if (n.kids.empty()) fl |= ImGuiTreeNodeFlags_Leaf;
    if (n.prim && n.path == selected_) fl |= ImGuiTreeNodeFlags_Selected;
    const bool open = ImGui::TreeNodeEx(n.path.c_str(), fl, "%s", name.c_str());
    if (ImGui::IsItemClicked() && n.prim && n.path != selected_) select(n.path);
    if (open) {
      for (const auto& [k, child] : n.kids) draw_node(k, child);
      ImGui::TreePop();
    }
  }

  void ui() {
    ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({320, opts_.design_api ? 375.f : 420.f}, ImGuiCond_FirstUseEver);  // (the DRC panel sits below it when editing)
    ImGui::Begin("napl");
    ImGui::Text("tick %llu   t = %.4f s", static_cast<unsigned long long>(cur_.tick), static_cast<double>(cur_.tick) * cur_.dt);
    if (cur_.diverged) ImGui::TextColored({1, 0.3f, 0.3f, 1}, "solver diverged");
    else if (cur_.running) ImGui::Text("running (rate %.3g)", cur_.rate);
    else ImGui::Text("paused");
    if (cur_.running) {
      if (ImGui::Button("Pause")) send(env_pause(kSource), "pause");
    } else if (ImGui::Button("Run")) {
      send(env_run(kSource, rate_), "run");
    }
    ImGui::SameLine();
    if (ImGui::InputDouble("rate", &rate_, 0, 0, "%.3g", ImGuiInputTextFlags_EnterReturnsTrue) && cur_.running)
      send(env_run(kSource, rate_), "run");
    ImGui::SetItemTooltip("sim-seconds per wall-second; 0 = as fast as possible (Enter applies while running)");
    if (ImGui::Button("Step")) send(env_step(kSource, std::max(1, steps_)), "step");
    ImGui::SameLine();
    ImGui::InputInt("ticks", &steps_);
    if (!status_.empty()) ImGui::TextColored({1, 0.6f, 0.2f, 1}, "%s", status_.c_str());
    ImGui::Separator();
    ImGui::TextUnformatted("scene");
    for (const auto& [k, child] : tree_.kids) draw_node(k, child);
    ImGui::End();

    ImGui::SetNextWindowPos({340, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({340, 220}, ImGuiCond_FirstUseEver);
    ImGui::Begin("inspector");
    if (selected_.empty()) ImGui::TextUnformatted("select a prim in the scene tree");
    else ImGui::TextUnformatted(selected_.c_str());
    for (const Shown& s : shown_) {
      const std::string unit = s.unit.empty() ? "" : " [" + s.unit + "]";
      if (s.values.size() == 1 && editable(s.attr)) {
        double v = s.values[0];
        ImGui::SetNextItemWidth(140);
        if (ImGui::InputDouble((s.attr + unit).c_str(), &v, 0, 0, "%.6g", ImGuiInputTextFlags_EnterReturnsTrue))
          send(env_setattr(kSource, selected_, s.attr, {v}), "set " + selected_ + "." + s.attr);
      } else {
        std::string line = s.attr + " =";
        char buf[32];
        for (double v : s.values) {
          std::snprintf(buf, sizeof buf, " %.6g", v);
          line += buf;
        }
        ImGui::Text("%s%s", line.c_str(), unit.c_str());
      }
    }
    ImGui::End();
    if (has_design_) design_ui();
    if (opts_.design_api) flat_.ui(dstate_, sel_instance() ? sel_instance()->id : "");  // 2D wiring (wire2d.cpp)
  }

  Host& host_;
  ViewerOptions opts_;
  std::vector<ScriptCmd> script_;
  Frame cur_;
  std::uint64_t scene_ = 0;
  Orbit orbit_;
  Node tree_;
  design::Report design_;  // the report of the `napl design build` this model came from (with a backend: of the design as edited)
  bool has_design_ = false;
  // Design editing (all driven through opts_.design_api, see viewer.h)
  design::State dstate_;
  std::deque<DReq> dq_;
  wire2d::Canvas flat_{[this](const std::string& tool, const std::string& args, bool quiet) {
    dq_.push_back({tool, quiet, [tool, args] { return design::request(tool, args); }});
  }};
  DReq dcur_;
  std::future<std::string> dfut_;
  bool draining_ = false, keep_camera_ = false;
  std::string loaded_rev_, dstatus_, edit_summary_, saved_, sugg_target_, sim_title_, sel_port_, sel_port_inst_;
  std::chrono::steady_clock::time_point last_poll_ = std::chrono::steady_clock::now();
  Placement place_;
  int cycling_ = -1, finding_ = -1, want_tab_ = -1;  // want_tab_: the tab the last reply is about (0 library, 1 wiring, 2 simulate)
  std::vector<Sugg> suggs_;
  bool sugg_swap_ = false, auto_dev_ = true;
  std::vector<Decision> decisions_;
  std::vector<std::string> edit_lines_;
  std::vector<std::pair<std::string, std::string>> sim_rows_;
  std::set<std::string> new_keys_, hl_;  // findings new in the last edit / instances the clicked finding involves
  char cat_buf_[64] = "", part_buf_[96] = "", save_buf_[256] = "";
  std::vector<std::string> prim_paths_;
  std::string pending_body_;
  std::string selected_, status_;
  std::vector<Shown> shown_;
  std::vector<std::pair<std::string, std::future<Reply>>> inspect_, cmds_;
  std::future<Reply> prims_;
  double rate_ = 1.0;
  int steps_ = 1;
};
}  // namespace

void run_viewer(Host& host, const ViewerOptions& opts) { Viewer(host, opts).run(); }

std::vector<ScriptCmd> parse_script(const std::string& script) {
  std::vector<ScriptCmd> out;
  std::istringstream entries(script);
  for (std::string entry; std::getline(entries, entry, ';');) {
    std::istringstream words(entry);
    ScriptCmd c;
    std::string frame;
    if (!(words >> frame)) continue;  // empty entry (trailing ';')
    if (!(words >> c.action)) throw std::runtime_error("script entry '" + entry + "': expected FRAME ACTION [ARGS]");
    c.frame = static_cast<int>(to_double(frame, "frame"));
    for (std::string a; words >> a;) c.args.push_back(a);
    if (c.action == "select" || c.action == "inst") {  // UI actions with no bus command: select a prim / a design instance's body
      if (c.args.size() != 1) throw std::runtime_error("script: " + c.action + (c.action == "inst" ? " INSTANCE" : " PATH"));
    } else if (const auto [lo, hi] = design_action_args(c.action); lo >= 0) {
      const int n = static_cast<int>(c.args.size());
      if (n < lo || n > hi) throw std::runtime_error("script entry '" + entry + "': " + c.action + " takes " + std::to_string(lo) + (hi > lo ? ".." + std::to_string(hi) : "") + " argument(s)");
    } else {
      script_envelope(c);  // validate now: bad scripts fail before a window opens
    }
    out.push_back(std::move(c));
  }
  return out;
}

std::vector<std::uint8_t> script_envelope(const ScriptCmd& c, const std::string& source) {
  const auto need = [&](std::size_t n, const char* usage) {
    if (c.args.size() != n) throw std::runtime_error(std::string("script: ") + usage);
  };
  if (c.action == "run") {
    need(1, "run RATE");
    return env_run(source, to_double(c.args[0], "rate"));
  }
  if (c.action == "pause") {
    need(0, "pause takes no arguments");
    return env_pause(source);
  }
  if (c.action == "step") {
    need(1, "step N");
    return env_step(source, static_cast<int>(to_double(c.args[0], "N")));
  }
  if (c.action == "setattr") {
    if (c.args.size() < 3) throw std::runtime_error("script: setattr PATH ATTR V [V...]");
    std::vector<double> v;
    for (std::size_t i = 2; i < c.args.size(); ++i) v.push_back(to_double(c.args[i], "value"));
    return env_setattr(source, c.args[0], c.args[1], v);
  }
  throw std::runtime_error("script: unknown action '" + c.action + "' (run, pause, step, setattr, select, inst, d* design actions)");
}
