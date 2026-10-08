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
#include "viewer/build3d.h"
#include "viewer/design.h"
#include "viewer/library_ui.h"
#include "viewer/theme.h"
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

// UI-only script actions (window state, no bus command, no editor needed): {min, max} argument counts; {-1, -1} = not one.
std::pair<int, int> ui_action_args(const std::string& a) {
  static const std::map<std::string, std::pair<int, int>> t = {
      {"mode", {1, 1}}, {"uisel", {1, 1}}, {"search", {1, 12}}, {"uicam", {1, 1}}, {"uiframe", {0, 0}}};
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
    theme::init();
    libui::Thumbs thumbs;
    thumbs_ = &thumbs;

    if (opts_.design_api) {
      d_state(true);  // the design behind the model on screen (quiet: an MCP server may not have one yet)
      dreq("cats", design::request("library.categories"), true);
      dreq("search", [this] { return search_req(); }, true);  // the first page of the library (an empty query lists everything)
    }
    for (int n_frame = 0; !glfwWindowShouldClose(window.w) && !(opts_.frames > 0 && n_frame >= opts_.frames) &&
                          !(opts_.stop && opts_.stop->load());
         ++n_frame) {
      glfwPollEvents();
      for (const ScriptCmd& c : script_) {
        if (c.frame != n_frame) continue;
        if (c.action == "select") select(c.args[0]);
        else if (c.action == "inst") select_instance(c.args[0]);
        else if (ui_action_args(c.action).first >= 0) script_ui(c);
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
      ImGui::Render();

      int fw = 0, fh = 0;
      glfwGetFramebufferSize(window.w, &fw, &fh);
      renderer.set_tubes(cur_.tubes);
      const bool drawn = renderer.frame(cur_.poses, view_cam_, fw, fh, [](VkCommandBuffer cb) {
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
    thumbs_ = nullptr;  // (the textures are unregistered by `thumbs` before the ImGui context goes)
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
    if (f.scene != frame_scene_) {  // first frame, or the model was replaced: the prim list changed too
      const design::Instance* si = keep_camera_ ? sel_instance() : nullptr;
      const std::string keep = si ? si->id : "";
      if (frame_scene_ != 0) select("");  // (a first-frame selection, e.g. scripted, is kept)
      frame_scene_ = f.scene;
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
    if (!sel_.empty())
      for (const design::Instance& i : design_.instances)
        if (i.id == sel_.back()) return &i;
    for (const design::Instance& i : design_.instances)
      if (selected_.size() > i.body.size() && selected_.ends_with("/" + i.body)) return &i;
    return nullptr;
  }
  // Selects design instances (the last is the primary one: gizmo, inspector); the prim inspector follows the primary.
  void set_sel(std::vector<std::string> ids) {
    std::erase_if(ids, [&](const std::string& i) { return std::none_of(design_.instances.begin(), design_.instances.end(), [&](const design::Instance& x) { return x.id == i; }); });
    if (!ids.empty()) select_instance(ids.back());
    else select("");
    sel_ = std::move(ids);
    if (opts_.design_api && sel_synced_ != sel_) {  // the backend's idea of the selection (agents read it with ui.state)
      sel_synced_ = sel_;
      dreq("quiet", design::request("select", "{\"silent\":true,\"instances\":" + json_list(sel_) + "}"), true);
    }
  }
  bool is_selected(const std::string& id) const { return std::find(sel_.begin(), sel_.end(), id) != sel_.end(); }
  static std::string json_list(const std::vector<std::string>& v) {
    std::string o = "[";
    for (const std::string& s : v) o += (o.size() > 1 ? "," : "") + design::quote(s);
    return o + "]";
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
    snap_requested_ = true;
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
      if (!f.instances.empty()) set_sel(f.instances);
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
        if (!draining_ && now - last_ev_ > std::chrono::milliseconds(100)) {  // UI events scripted agents queue (ui.mode, drags ...)
          last_ev_ = now;
          dq_.push_back({"uievents", true, [] { return design::request("ui_poll"); }});
        } else if (!draining_ && now - last_poll_ > std::chrono::milliseconds(300)) {
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
    if (rq.kind == "search") lib_busy_ = false;  // answered (or failed): the next dirty query may go out
    design::Json j;
    try {
      j = design::Parser(text).parse();
    } catch (const std::exception& e) {
      dstatus_ = rq.kind + ": " + e.what();
      return;
    }
    if (rq.kind.rfind("flatten.", 0) == 0) {  // the 2D wiring canvas (wire2d.cpp) owns these replies
      const bool was_flat = flat_.active();
      flat_.on_reply(rq.kind, j);
      if (flat_.active() != was_flat) set_mode(flat_.active() ? "wiring" : "build");  // flatten.begin / .end (UI or agent)
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
      if (const int tr = static_cast<int>(res.n("thumb_rev")); tr != thumb_rev_) {  // library thumbnails landed: ask the library again
        thumb_rev_ = tr;
        lib_.dirty = true;
      }
      if (!res.at("rev").str.empty() && res.at("rev").str != dstate_.rev) d_state(true);
      return;
    }
    if (rq.kind == "uievents") {
      for (const design::Json& e : res.at("events").arr) ui_event(e);
      return;
    }
    if (rq.kind == "quiet") return;
    if (rq.kind == "cats") {
      lib_.cats.clear();
      for (const design::Json& c : res.at("categories").arr) lib_.cats.push_back({c.s("path"), static_cast<int>(c.n("count")), static_cast<int>(c.n("depth"))});
      return;
    }
    if (rq.kind == "ranges") {
      lib_.ranges.clear();
      for (const auto& [k, r] : res.at("ranges").obj)
        if (r.arr.size() >= 2) lib_.ranges[k] = {r.arr[0].num, r.arr[1].num, r.arr.size() > 2 ? r.arr[2].str : ""};
      return;
    }
    if (rq.kind == "search") {
      lib_.hits.clear();
      lib_.total = static_cast<int>(res.n("total"));
      for (const design::Json& h : res.at("parts").arr) {
        libui::Hit x{h.s("id"), h.s("name"), h.s("vendor"), h.s("category"), h.s("thumb")};
        x.has_mass = h.at("mass").kind == design::Json::Num;
        x.mass = h.n("mass");
        lib_.hits.push_back(std::move(x));
      }
      return;
    }
    if (rq.kind == "info") {
      libui::Info& i = lib_.infos[res.s("id")];
      i = {};
      i.id = res.s("id"), i.name = res.s("name"), i.vendor = res.s("vendor"), i.category = res.s("category");
      i.part_number = res.s("part_number"), i.url = res.s("url"), i.thumb = res.s("thumb");
      for (const design::Json& p : res.at("params").arr) {
        char v[48];
        const design::Json& val = p.at("value");
        if (val.kind == design::Json::Num) std::snprintf(v, sizeof v, "%.6g", val.num);
        else std::snprintf(v, sizeof v, "%s", val.kind == design::Json::Str ? val.str.c_str() : "(table)");
        i.params.push_back({p.s("name"), p.s("unit"), p.s("fidelity"), p.s("src"), p.s("url"), p.s("quote"), v});
      }
      for (const design::Json& p : res.at("ports").arr) i.ports.push_back({p.s("name"), p.s("kind"), p.s("type"), p.s("gender")});
      for (const design::Json& b : res.at("boxes").arr) i.boxes.push_back(box_from(b));
      return;
    }
    if (rq.kind == "targets") {
      Targets& t = targets_[res.s("part")];
      t = {};
      t.requested = t.loaded = true;
      t.rev = dstate_.rev;
      for (const design::Json& b : res.at("boxes").arr) t.boxes.push_back(box_from(b));
      for (const design::Json& x : res.at("targets").arr) {
        Target g;
        g.to = x.s("to"), g.port = x.s("port");
        double v[3], r[9];
        design::vec_from(x.at("pos"), v, 3), design::vec_from(x.at("R"), r, 9);
        g.pos = {v[0], v[1], v[2]};
        std::copy(r, r + 9, g.R.m);
        design::vec_from(x.at("port_pos"), v, 3);
        g.port_pos = {v[0], v[1], v[2]};
        t.list.push_back(std::move(g));
      }
      return;
    }
    if (rq.kind == "text") {
      program_text_ = res.s("text");
      return;
    }
    if (j.at("state").kind == design::Json::Obj) apply_state(j.at("state"));
    if (rq.kind == "suggest") {
      right_tab_ = 3;  // PLACE
      suggs_.clear();
      for (const design::Json& r : res.at("suggestions").arr)
        suggs_.push_back({r.s("part"), r.s("port"), r.s("category"), r.s("name"), r.n("score"), r.n("mass"), r.n("mass_delta"),
                          static_cast<int>(r.n("mates_kept")), static_cast<int>(r.at("dangling").arr.size()), strs(r.at("why")),
                          strs(r.at("mates_lost"))});
    } else if (rq.kind == "sim") {
      mode_ = Mode::Simulate;
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
    if (op == "add" && snap_requested_) right_tab_ = 3;  // the placement alternatives of a "snap here" are on the PLACE tab
    snap_requested_ = false;
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
    if (!has_design_ && mode_ == Mode::Simulate && pending_mode_.empty()) mode_ = Mode::Build;  // an editable design opens on BUILD
    has_design_ = true;
    rebuild_scene();
    for (auto& [part, t] : targets_) t.requested = false;  // the open ports changed: ghost targets are asked again on the next drag
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
        if (ImGui::Selectable(label.c_str(), is_selected(i->id) || sel == i)) {
          std::vector<std::string> next = ImGui::GetIO().KeyShift || ImGui::GetIO().KeyCtrl ? sel_ : std::vector<std::string>{};
          if (!std::erase(next, i->id)) next.push_back(i->id);
          set_sel(next);
        }
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

  // ================================ layout: top bar, status bar, per-mode panes =====================================================
  enum class Mode { Build, Library, Wiring, Simulate, Program };
  static constexpr float kTop = 38, kStatus = 26;
  static constexpr ImGuiWindowFlags kPane = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                                            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav;

  static const char* mode_name(Mode m) {
    static const char* const n[] = {"build", "library", "wiring", "simulate", "program"};
    return n[static_cast<int>(m)];
  }
  bool set_mode(const std::string& name) {
    for (Mode m : {Mode::Build, Mode::Library, Mode::Wiring, Mode::Simulate, Mode::Program})
      if (name == mode_name(m)) {
        if (!has_design_ && m != Mode::Simulate && m != Mode::Program) return false;
        mode_ = m;
        if (m == Mode::Program) dreq("text", design::request("design_text"), true);
        return true;
      }
    return false;
  }

  // The viewport = the part of the window the panes leave free; the 3D scene is rendered full-window and the camera is shifted so
  // its target sits in the middle of the viewport (WindowRenderer draws the whole swapchain image).
  void layout(float W, float H) {
    const bool narrow = W < 900;
    float l = 0, r = 0;
    switch (mode_) {
      case Mode::Build: l = narrow ? 0 : 272, r = narrow ? 0 : 322; break;
      case Mode::Library: l = std::round(W * 0.78f); break;
      case Mode::Simulate: l = narrow ? 0 : 300, r = narrow ? 0 : 300; break;
      default: break;
    }
    pane_l_ = l;
    pane_r_ = r;
    full_page_ = mode_ == Mode::Wiring || mode_ == Mode::Program;
    vp_goal_ = full_page_ ? ImVec4{0, kTop, W, H - kStatus} : mode_ == Mode::Library ? ImVec4{l, kTop, W, H - kStatus} : ImVec4{l, kTop, W - r, H - kStatus};
  }
  // The 3D viewport follows its goal with an ease (~0.2 s), so the scene glides when a mode change opens or closes side panels.
  void smooth_viewport(float dt) {
    const float k = vp_init_ ? 1.f - std::exp(-14.f * std::min(dt, 0.1f)) : 1.f;
    const float* g = &vp_goal_.x;
    float* v = &vp_.x;
    for (int i = 0; i < 4; ++i) v[i] = std::abs(g[i] - v[i]) < 0.5f ? g[i] : v[i] + (g[i] - v[i]) * k;
    vp_init_ = true;
  }
  Camera view_camera(double W, double H) const {
    Camera c = orbit_.camera();
    if (full_page_) return c;
    const double tanh = std::tan(c.fovy * 3.14159265358979 / 360), aspect = W / H;
    const double offx = 0.5 * (vp_.x + vp_.z) - 0.5 * W, offy = 0.5 * (vp_.y + vp_.w) - 0.5 * H;  // px the target must move
    const b3::Cam v = b3::Cam::make(c, W, H);
    const b3::V3 shift = v.r * (-offx / (0.5 * W) * orbit_.dist * tanh * aspect) + v.u * (offy / (0.5 * H) * orbit_.dist * tanh);
    for (int k = 0; k < 3; ++k) {
      c.pos[k] += shift[k];
      c.lookat[k] += shift[k];
    }
    return c;
  }

  void top_bar(float W) {
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({W, kTop});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4{0.02f, 0.02f, 0.025f, 0.92f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {16, 0});
    ImGui::Begin("##top", nullptr, kPane | ImGuiWindowFlags_NoScrollbar);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::PushFont(nullptr, 17.f);
    dl->AddText({16, 9}, theme::u32(theme::kText), "NAPL");
    ImGui::PopFont();
    ImGui::SetCursorScreenPos({96, 0});
    std::vector<const char*> labels;
    std::vector<Mode> modes;
    for (Mode m : {Mode::Build, Mode::Library, Mode::Wiring, Mode::Simulate, Mode::Program}) {
      if (!has_design_ && m != Mode::Simulate && m != Mode::Program) continue;
      static const char* const caps[] = {"BUILD", "LIBRARY", "WIRING", "SIMULATE", "PROGRAM"};
      labels.push_back(caps[static_cast<int>(m)]);
      modes.push_back(m);
    }
    int active = 0;
    for (std::size_t i = 0; i < modes.size(); ++i)
      if (modes[i] == mode_) active = static_cast<int>(i);
    const int hit = theme::tabs(labels.data(), static_cast<int>(labels.size()), active);
    if (hit >= 0) set_mode(mode_name(modes[hit]));
    if (opts_.design_api) {  // right side: undo / redo / save, as quiet text buttons
      ImGui::PushFont(nullptr, 13.f);
      const auto tb = [&](const char* text, float& x, bool enabled) {
        const float w = ImGui::CalcTextSize(text).x;
        x -= w + 18;
        ImGui::SetCursorScreenPos({x, 10});
        ImGui::BeginDisabled(!enabled);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0, 0, 0, 0});
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {0, 2});
        const bool c = ImGui::Button(text);
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::EndDisabled();
        return c;
      };
      float x = W - 8;
      if (tb("SAVE", x, true)) d_save(save_buf_);
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("write the design TOML%s%s", dstate_.path.empty() ? "" : ": ", dstate_.path.c_str());
      if (tb(("REDO " + std::to_string(dstate_.redo)).c_str(), x, dstate_.redo > 0)) d_undo(true);
      if (tb(("UNDO " + std::to_string(dstate_.undo)).c_str(), x, dstate_.undo > 0)) d_undo(false);
      ImGui::PopFont();
      if (save_buf_[0] == 0 && !dstate_.path.empty()) std::snprintf(save_buf_, sizeof save_buf_, "%s", dstate_.path.c_str());
    }
    dl->AddLine({0, kTop - 0.5f}, {W, kTop - 0.5f}, theme::u32(theme::kLine));
    ImGui::End();
  }

  void status_bar(float W, float H) {
    ImGui::SetNextWindowPos({0, H - kStatus});
    ImGui::SetNextWindowSize({W, kStatus});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4{0.02f, 0.02f, 0.025f, 0.92f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {16, 0});
    ImGui::Begin("##status", nullptr, kPane | ImGuiWindowFlags_NoScrollbar);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddLine({0, H - kStatus + 0.5f}, {W, H - kStatus + 0.5f}, theme::u32(theme::kLine));
    ImGui::PushFont(nullptr, 12.f);
    const float y = H - kStatus + 6;
    std::string left = "NO SELECTION";
    if (!sel_.empty()) {
      left = "SELECTED  " + sel_.back();
      if (const design::Instance* i = sel_instance()) left += "  " + i->part;
      if (sel_.size() > 1) left += "  +" + std::to_string(sel_.size() - 1);
    } else if (!selected_.empty()) {
      left = "PRIM  " + selected_;
    }
    dl->AddText({16, y}, theme::u32(theme::kText), left.c_str());
    std::string msg = !dstatus_.empty() ? dstatus_ : !dstate_.build_error.empty() ? "does not build: " + dstate_.build_error : !status_.empty() ? status_ : dfut_.valid() ? "working: " + dcur_.kind + " ..." : "";
    if (msg.empty() && !edit_summary_.empty()) msg = edit_summary_;
    if (msg.empty() && has_design_ && W > 1000) {  // keyboard hints of the page, while nothing else needs the space
      const char* hint = mode_ == Mode::Build     ? "G MOVE   R ROTATE   F FRAME   1-4 VIEWS   DEL REMOVE   CTRL+Z UNDO"
                         : mode_ == Mode::Library ? "CLICK A CARD FOR DETAILS   DRAG ONE INTO THE 3D VIEW"
                         : mode_ == Mode::Wiring  ? "DRAG PIN TO PIN   WHEEL ZOOMS   MIDDLE DRAG PANS   DEL REMOVES THE SELECTED WIRE"
                                                  : "";
      dl->PushClipRect({W * 0.30f, 0}, {W * 0.70f, H}, true);
      dl->AddText({W * 0.30f, y}, theme::u32(theme::kDim, 0.65f), hint);
      dl->PopClipRect();
    }
    if (!msg.empty()) {
      const bool bad = !dstatus_.empty() || !status_.empty() || !dstate_.build_error.empty();
      dl->PushClipRect({W * 0.30f, 0}, {W * 0.70f, H}, true);
      dl->AddText({W * 0.30f, y}, theme::u32(bad ? theme::kWarn : theme::kDim), msg.c_str());
      dl->PopClipRect();
    }
    std::size_t errors = 0, warns = 0;
    for (const design::Finding& f : design_.findings) {
      if (f.severity == "error") ++errors;
      else if (f.severity == "warning") ++warns;
    }
    char t[48];
    std::snprintf(t, sizeof t, "T %.3f s", static_cast<double>(cur_.tick) * cur_.dt);
    float x = W - 16 - ImGui::CalcTextSize(t).x;
    dl->AddText({x, y}, theme::u32(theme::kDim), t);
    if (has_design_) {
      char b[32];
      std::snprintf(b, sizeof b, "%zu WARN", warns);
      x -= 12 + ImGui::CalcTextSize(b).x;
      dl->AddText({x, y}, theme::u32(warns ? theme::kWarn : theme::kDim), b);
      std::snprintf(b, sizeof b, "%zu ERR", errors);
      x -= 16 + ImGui::CalcTextSize(b).x;
      dl->AddText({x, y}, theme::u32(errors ? theme::kErr : theme::kOk), b);
      x -= 18;
      dl->AddText({x - ImGui::CalcTextSize("DRC").x, y}, theme::u32(theme::kDim), "DRC");
    }
    ImGui::PopFont();
    ImGui::End();
  }

  ImVec2 pane_pos(bool right, float W) const { return {right ? W - pane_r_ : 0, kTop}; }
  void begin_pane(const char* id, ImVec2 pos, ImVec2 size, bool solid = false) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, solid ? theme::kBg : theme::kPanel);
    ImGui::Begin(id, nullptr, kPane);
    ImGui::PopStyleColor();
  }

  // ---------------------------------------- BUILD ------------------------------------------------------------------------------
  void build_panes(float W, float H) {
    const float ph = H - kTop - kStatus;
    if (pane_l_ > 0) {
      libui::Actions a;
      libui::sidebar(lib_, *thumbs_, {0, kTop}, {pane_l_, ph}, a);
      lib_actions(a);
    }
    if (pane_r_ > 0) inspector_pane(W, ph);
    viewport_tools();
  }

  void inspector_pane(float W, float ph) {
    begin_pane("##right", pane_pos(true, W), {pane_r_, ph});
    static const char* const tabs[] = {"INSPECT", "PARTS", "DRC", "PLACE", "HISTORY"};
    const int hit = theme::tabs(tabs, 5, right_tab_, 13.f);
    if (hit >= 0) right_tab_ = hit;
    ImGui::SetCursorPos({ImGui::GetStyle().WindowPadding.x, ImGui::GetCursorPosY() + 30});  // theme::tabs leaves the cursor on its row
    ImGui::BeginChild("##rbody", {0, 0});
    const design::Instance* sel = sel_instance();
    if (right_tab_ == 0) inspect_tab(sel);
    else if (right_tab_ == 1) instance_list();
    else if (right_tab_ == 2) drc_body(true);
    else if (right_tab_ == 4) history_tab();
    else {
      if (place_.alts.empty()) ImGui::TextDisabled("Snap a part to see its placement alternatives here.");
      placement_ui();
    }
    ImGui::EndChild();
    ImGui::End();
  }

  // Undo history (python Design._undo_notes): the steps behind the document, newest last, then the steps UNDO took back. A click
  // on a row travels there (one undo / redo request per step).
  void history_tab() {
    theme::caption("EDIT HISTORY");
    if (dstate_.hist_undo.empty() && dstate_.hist_redo.empty()) {
      ImGui::Spacing();
      ImGui::TextDisabled("No edits yet. Add, move or wire a part and each step is listed here.");
      ImGui::TextDisabled("Ctrl+Z undoes the last one, Ctrl+Shift+Z brings it back.");
      return;
    }
    const int n = static_cast<int>(dstate_.hist_undo.size());
    const int total = dstate_.undo;  // steps older than the list (the backend keeps the last 14)
    if (total > n) ImGui::TextDisabled("... %d earlier steps", total - n);
    for (int i = 0; i < n; ++i) {
      const bool last = i == n - 1;
      ImGui::PushID(i);
      ImGui::PushStyleColor(ImGuiCol_Text, last ? theme::kText : theme::kDim);
      const std::string row = std::to_string(total - n + i + 1) + "   " + dstate_.hist_undo[i];
      if (ImGui::Selectable(row.c_str(), last)) {
        for (int k = 0; k < n - 1 - i; ++k) d_undo(false);  // back to just after this step
      }
      ImGui::PopStyleColor();
      ImGui::PopID();
    }
    if (!dstate_.hist_redo.empty()) {
      ImGui::Spacing();
      theme::caption("UNDONE (CTRL+SHIFT+Z)");
      for (std::size_t i = 0; i < dstate_.hist_redo.size(); ++i) {
        ImGui::PushID(1000 + static_cast<int>(i));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{theme::kDim.x, theme::kDim.y, theme::kDim.z, 0.6f});
        if (ImGui::Selectable(dstate_.hist_redo[i].c_str(), false))
          for (std::size_t k = 0; k <= i; ++k) d_undo(true);
        ImGui::PopStyleColor();
        ImGui::PopID();
      }
    }
  }

  // The primary selection's pose in the document's terms: world position (mm) and rpy (deg).
  void inspect_tab(const design::Instance* sel) {
    if (!opts_.design_api && !has_design_) return;
    ImGui::TextDisabled("%s   total %.4f kg   %zu parts", design_.assembly.c_str(), design_.mass_total, design_.instances.size());
    ImGui::Spacing();
    if (!sel) {
      ImGui::TextDisabled("click a part in the 3D view or the PARTS tab");
      snapping_controls();
      return;
    }
    ImGui::PushFont(nullptr, 17.f);
    ImGui::TextUnformatted(sel->id.c_str());
    ImGui::PopFont();
    ImGui::TextDisabled("%s", sel->part.c_str());
    ImGui::TextDisabled("%s   %.1f g", sel->group.empty() ? "(no group)" : sel->group.c_str(), sel->mass * 1000);
    const bool root = sel->id == dstate_.root;
    const bool is_free = std::find(dstate_.free.begin(), dstate_.free.end(), sel->id) != dstate_.free.end();
    ImGui::TextColored(root ? theme::kAccent : is_free ? theme::kWarn : theme::kDim, "%s", root ? "ROOT" : is_free ? "FREE-PLACED" : "MATED");
    if (sel_.size() > 1) {
      ImGui::SameLine();
      ImGui::TextDisabled("  +%zu more selected", sel_.size() - 1);
    }
    const auto git = scene_.find(sel->id);
    if (git != scene_.end() && !root && opts_.design_api) {
      const b3::InstGeo& g = git->second;
      ImGui::Spacing();
      theme::caption("POSITION  MM");
      double p[3] = {g.pos.x * 1000, g.pos.y * 1000, g.pos.z * 1000};
      bool commit = false;
      ImGui::PushID("pos");
      for (int k = 0; k < 3; ++k) {
        ImGui::PushID(k);
        ImGui::SetNextItemWidth((pane_r_ - 24 - 16) / 3);
        ImGui::DragScalar("##v", ImGuiDataType_Double, &p[k], 0.5f, nullptr, nullptr, "%.1f");
        commit = commit || ImGui::IsItemDeactivatedAfterEdit();
        ImGui::PopID();
        if (k < 2) ImGui::SameLine();
      }
      ImGui::PopID();
      if (commit) place_world(sel->id, "pos", {p[0] / 1000, p[1] / 1000, p[2] / 1000});
      theme::caption("ROTATION  DEG  (R P Y)");
      const b3::V3 rpy = b3::mat_to_rpy(g.R);
      double r[3] = {rpy.x * 180 / 3.14159265358979, rpy.y * 180 / 3.14159265358979, rpy.z * 180 / 3.14159265358979};
      bool rc = false;
      ImGui::PushID("rot");
      for (int k = 0; k < 3; ++k) {
        ImGui::PushID(k);
        ImGui::SetNextItemWidth((pane_r_ - 24 - 16) / 3);
        ImGui::DragScalar("##v", ImGuiDataType_Double, &r[k], 0.5f, nullptr, nullptr, "%.1f");
        rc = rc || ImGui::IsItemDeactivatedAfterEdit();
        ImGui::PopID();
        if (k < 2) ImGui::SameLine();
      }
      ImGui::PopID();
      if (rc) place_world(sel->id, "rpy", {r[0] * 3.14159265358979 / 180, r[1] * 3.14159265358979 / 180, r[2] * 3.14159265358979 / 180});
      ImGui::Spacing();
      theme::caption("COLOUR");
      float col[4] = {0.6f, 0.6f, 0.65f, 1};
      if (g.has_color) std::copy(g.color, g.color + 4, col);
      else if (const auto it = color_pick_.find(sel->id); it != color_pick_.end()) std::copy(it->second.begin(), it->second.end(), col);
      if (ImGui::ColorButton("##swatch", {col[0], col[1], col[2], col[3]}, ImGuiColorEditFlags_AlphaPreviewHalf, {pane_r_ - 24, 22})) ImGui::OpenPopup("##colour");
      if (color_flash_ > ImGui::GetTime() && !ImGui::IsPopupOpen("##colour")) ImGui::OpenPopup("##colour");  // a scripted ui.* colour op shows the picker
      if (ImGui::BeginPopup("##colour")) {
        std::array<float, 4>& pick = color_pick_[sel->id];
        if (pick == std::array<float, 4>{}) std::copy(col, col + 4, pick.begin());
        if (ImGui::ColorPicker4("##picker", pick.data(), ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_DisplayRGB)) color_dirty_ = true;
        if (color_dirty_ && !ImGui::IsMouseDown(0) && color_flash_ <= ImGui::GetTime()) {  // released: one undo step
          color_dirty_ = false;
          dreq("edit", design::request("set_color", "{\"silent\":true,\"instance\":" + q(sel->id) + ",\"rgba\":[" + std::to_string(pick[0]) + "," + std::to_string(pick[1]) + "," + std::to_string(pick[2]) + "," + std::to_string(pick[3]) + "]}"));
        }
        if (ImGui::SmallButton("RESET TO PART COLOUR")) {
          color_pick_.erase(sel->id);
          dreq("edit", design::request("set_color", "{\"silent\":true,\"instance\":" + q(sel->id) + ",\"rgba\":null}"));
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
    }
    ImGui::Spacing();
    theme::caption("ACTIONS");
    const bool can = opts_.design_api && !root;
    ImGui::BeginDisabled(!can);
    if (ImGui::Button("DUPLICATE") && can) dreq("edit", design::request("duplicate", "{\"silent\":true,\"instance\":" + q(sel->id) + "}"));
    ImGui::SameLine();
    static int mirror_axis = 1;
    if (ImGui::Button("MIRROR")) dreq("edit", design::request("mirror", "{\"silent\":true,\"instances\":" + json_list(sel_) + ",\"axis\":\"" + "xyz"[mirror_axis] + "\"}"));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(46);
    const char* const axes[] = {"X", "Y", "Z"};
    if (ImGui::BeginCombo("##maxis", axes[mirror_axis])) {
      for (int k = 0; k < 3; ++k)
        if (ImGui::Selectable(axes[k], k == mirror_axis)) mirror_axis = k;
      ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("mirror across the world plane through the origin normal to this axis (free-placed copies, mates are not mirrored)");
    if (ImGui::Button("REMOVE") && can) d_remove(sel->id);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("FRAME")) frame_selection();
    if (sel_.size() > 1) {
      ImGui::Spacing();
      theme::caption("ALIGN / DISTRIBUTE");
      const char* const to[] = {"min", "center", "max"};
      for (int ax = 0; ax < 3; ++ax) {
        ImGui::TextUnformatted(axes[ax]);
        for (int t = 0; t < 3; ++t) {
          ImGui::SameLine();
          ImGui::PushID(ax * 3 + t);
          if (ImGui::SmallButton(to[t])) place_group("align", ax, to[t]);
          ImGui::PopID();
        }
        ImGui::SameLine();
        ImGui::PushID(100 + ax);
        if (ImGui::SmallButton("distribute")) place_group("distribute", ax, "");
        ImGui::PopID();
      }
    }
    if (!edit_lines_.empty()) {
      ImGui::Spacing();
      theme::caption("LAST EDIT");
      ImGui::PushTextWrapPos();
      for (std::size_t i = 0; i < edit_lines_.size() && i < 6; ++i) ImGui::TextDisabled("%s", edit_lines_[i].c_str());
      ImGui::PopTextWrapPos();
    }
    snapping_controls();
  }

  void snapping_controls() {
    ImGui::Spacing();
    theme::caption("SNAPPING");
    static const double grids[] = {0, 0.001, 0.005, 0.01, 0.05};
    static const char* const gl[] = {"off", "1 mm", "5 mm", "10 mm", "50 mm"};
    int gi = 2;
    for (int k = 0; k < 5; ++k)
      if (std::abs(snap_.grid - grids[k]) < 1e-9) gi = k;
    ImGui::SetNextItemWidth(88);
    if (ImGui::BeginCombo("grid", gl[gi])) {
      for (int k = 0; k < 5; ++k)
        if (ImGui::Selectable(gl[k], k == gi)) snap_.grid = grids[k];
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    static const double angs[] = {0, 5, 15, 45, 90};
    static const char* const al[] = {"off", "5 deg", "15 deg", "45 deg", "90 deg"};
    int ai = 2;
    for (int k = 0; k < 5; ++k)
      if (std::abs(snap_.angle - angs[k]) < 1e-9) ai = k;
    ImGui::SetNextItemWidth(88);
    if (ImGui::BeginCombo("angle", al[ai])) {
      for (int k = 0; k < 5; ++k)
        if (ImGui::Selectable(al[k], k == ai)) snap_.angle = angs[k];
      ImGui::EndCombo();
    }
    ImGui::Checkbox("edges", &snap_.edges);
    ImGui::SameLine();
    ImGui::Checkbox("ports", &snap_.ports);
    ImGui::SameLine();
    ImGui::Checkbox("guides", &snap_.guides);
    ImGui::SetItemTooltip("hold Ctrl while dragging to turn all snapping off for that drag");
  }

  // A numeric / group edit of the selection in world terms (mm fields, align): confirm first when it would detach a mated part.
  void place_world(const std::string& id, const char* key, b3::V3 v) {
    const std::string tool = std::string(key) == "pos" ? "gizmo.move" : "gizmo.rotate";
    char b[160];
    std::snprintf(b, sizeof b, "[%.6f,%.6f,%.6f]", v.x, v.y, v.z);
    const std::string vec = b;
    run_placement(id, [tool, id, key, vec](bool unmate) {
      return design::request(tool, "{\"silent\":true,\"instance\":" + design::quote(id) + ",\"" + key + "\":" + vec + (unmate ? ",\"unmate\":true" : "") + "}");
    });
  }
  void place_group(const char* tool, int ax, const std::string& to) {
    const std::string ids = json_list(sel_);
    bool held = false;
    for (const std::string& i : sel_) held = held || is_held(i);
    auto make = [tool = std::string(tool), ax, to, ids](bool unmate) {
      return design::request(tool, "{\"silent\":true,\"instances\":" + ids + ",\"axis\":\"" + "xyz"[ax] + "\",\"to\":\"" + to + "\"" + (unmate ? ",\"unmate\":true" : "") + "}");
    };
    if (held) confirm_unmate(sel_.back(), make);
    else dreq("edit", make(false));
  }
  bool is_held(const std::string& id) const {
    return id != dstate_.root && std::find(dstate_.free.begin(), dstate_.free.end(), id) == dstate_.free.end();
  }
  void run_placement(const std::string& id, std::function<std::string(bool)> make) {
    if (is_held(id)) confirm_unmate(id, std::move(make));
    else dreq("edit", make(false));
  }
  void confirm_unmate(const std::string& id, std::function<std::string(bool)> make) {
    confirm_ = std::move(make);
    confirm_id_ = id;
    confirm_open_ = true;
  }
  void confirm_popup() {
    if (confirm_open_) {
      ImGui::OpenPopup("DETACH FROM MATE?");
      confirm_open_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, {0.5f, 0.5f});
    if (ImGui::BeginPopupModal("DETACH FROM MATE?", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
      ImGui::Text("%s is held by a mate. Placing it freely drops that mate\n(its fasteners go with it; parts mated to it stay on it).", confirm_id_.c_str());
      ImGui::Spacing();
      if (theme::button("DETACH AND PLACE", true) || ImGui::IsKeyPressed(ImGuiKey_Enter)) {
        dreq("edit", confirm_(true));
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine();
      if (ImGui::Button("CANCEL") || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
      ImGui::EndPopup();
    }
  }

  void viewport_tools() {
    ImGui::SetNextWindowPos({vp_.x + 12, vp_.y + 12});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4{0.02f, 0.02f, 0.025f, 0.80f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, 5});
    ImGui::Begin("##vptools", nullptr, kPane | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    const auto mode_btn = [&](const char* label, b3::Gizmo::Mode m, const char* tip) {
      const bool on = gizmo_.mode == m;
      ImGui::PushStyleColor(ImGuiCol_Text, on ? theme::kAccent : theme::kDim);
      if (ImGui::SmallButton(label)) gizmo_.mode = m;
      ImGui::PopStyleColor();
      ImGui::SetItemTooltip("%s", tip);
      ImGui::SameLine();
    };
    mode_btn("MOVE", b3::Gizmo::Move, "G: move along an axis or on the ground plane");
    mode_btn("ROTATE", b3::Gizmo::Rotate, "R: rotate about an axis (snaps to the angle step)");
    mode_btn("OFF", b3::Gizmo::Off, "no gizmo");
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    const struct { const char* l; const char* p; } views[] = {{"TOP", "top"}, {"FRONT", "front"}, {"SIDE", "side"}, {"ISO", "iso"}};
    for (const auto& v : views) {
      ImGui::PushStyleColor(ImGuiCol_Text, theme::kDim);
      if (ImGui::SmallButton(v.l)) camera_preset(v.p);
      ImGui::PopStyleColor();
      ImGui::SameLine();
    }
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    if (ImGui::SmallButton("FRAME")) frame_selection();
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, emph_wires_ ? theme::kAccent : theme::kDim);
    if (ImGui::SmallButton("WIRES")) emph_wires_ = !emph_wires_;
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("emphasise cables: at least 4 px wide at any zoom (their true diameter is unchanged)");
    ImGui::End();
  }

  void camera_preset(const std::string& p) {
    static const std::map<std::string, std::pair<double, double>> pre = {{"front", {-1.5707963, 0.0}}, {"side", {0.0, 0.0}}, {"top", {-1.5707963, 1.5}}, {"iso", {-0.6, 0.55}}};
    const auto it = pre.find(p);
    if (it == pre.end()) return;
    cam_anim_ = {true, ImGui::GetTime(), orbit_.yaw, orbit_.pitch, orbit_.dist, it->second.first, it->second.second, orbit_.dist};
  }
  void frame_selection() {
    b3::V3 lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
    bool any = false;
    for (const std::string& id : sel_) {
      const auto it = scene_.find(id);
      if (it == scene_.end()) continue;
      for (int k = 0; k < 3; ++k) {
        lo[k] = std::min(lo[k], it->second.lo[k]);
        hi[k] = std::max(hi[k], it->second.hi[k]);
      }
      any = true;
    }
    if (!any) {
      for (const auto& [id, g] : scene_)
        for (int k = 0; k < 3; ++k) {
          lo[k] = std::min(lo[k], g.lo[k]);
          hi[k] = std::max(hi[k], g.hi[k]);
          any = true;
        }
    }
    if (!any) return;
    const b3::V3 c = (lo + hi) * 0.5;
    const double r = std::max(0.03, 0.5 * b3::len(hi - lo));
    cam_anim_ = {true, ImGui::GetTime(), orbit_.yaw, orbit_.pitch, orbit_.dist, orbit_.yaw, orbit_.pitch, 2.6 * r};
    cam_anim_.target_from = {orbit_.target[0], orbit_.target[1], orbit_.target[2]};
    cam_anim_.target_to = c;
    cam_anim_.move_target = true;
  }

  // The sidebar / page asked for something.
  void lib_actions(const libui::Actions& a) {
    if (!a.info.empty()) want_info(a.info);
    if (!a.open_page.empty()) {
      lib_.selected = a.open_page;
      set_mode("library");
      want_info(a.open_page);
    }
    if (!a.add.empty()) add_part(a.add);
    if (!a.swap.empty()) {
      if (const design::Instance* s = sel_instance()) d_swap(s->id, a.swap, true);
    }
  }
  void want_info(const std::string& id) {
    if (opts_.design_api && !lib_.infos.count(id)) {
      lib_.infos[id].id = "";  // (marks the request as sent)
      dreq("info", design::request("library.info", "{\"part\":" + q(id) + "}"), true);
    }
  }
  // ADD: onto the selected instance's best open compatible port; with nothing selected, onto the first (best) open port or loose.
  void add_part(const std::string& part) {
    std::string to;
    if (const design::Instance* s = sel_instance()) to = s->id;
    dreq("edit", design::request("library.drop", "{\"silent\":true,\"part\":" + q(part) + (to.empty() ? "" : ",\"to\":" + q(to)) +
                                                       (to.empty() ? ",\"point\":" + point_json(free_drop_point()) : "") + "}"));
  }
  b3::V3 free_drop_point() const {
    b3::V3 c{orbit_.target[0], orbit_.target[1], orbit_.target[2]};
    return c;
  }
  static std::string point_json(b3::V3 p) {
    char b[120];
    std::snprintf(b, sizeof b, "[%.6f,%.6f,%.6f]", p.x, p.y, p.z);
    return b;
  }

  std::string search_req() {
    lib_.dirty = false;
    if (lib_.ranges_for != lib_.category || lib_.ranges.empty()) {
      lib_.ranges_for = lib_.category;
      dreq("ranges", design::request("library.ranges", "{\"category\":" + q(lib_.category) + "}"), true);
    }
    return design::request("library.search", "{\"silent\":true,\"text\":" + q(lib_.query) + ",\"category\":" + q(lib_.category) + ",\"filters\":" + lib_.filters_json() + ",\"limit\":150}");
  }

  static b3::Box box_from(const design::Json& b) {
    double c[3], r[9], h[3];
    design::vec_from(b.at("c"), c, 3), design::vec_from(b.at("R"), r, 9), design::vec_from(b.at("h"), h, 3);
    b3::Box x;
    x.c = {c[0], c[1], c[2]};
    std::copy(r, r + 9, x.R.m);
    x.h = {h[0], h[1], h[2]};
    return x;
  }

  // ---------------------------------------- LIBRARY page -----------------------------------------------------------------------
  void library_page(float H) {
    libui::Actions a;
    const design::Instance* s = sel_instance();
    int targets = -1;
    if (!lib_.selected.empty()) {
      const auto it = targets_.find(lib_.selected);
      if (it != targets_.end()) targets = static_cast<int>(it->second.list.size());
      else want_targets(lib_.selected);
    }
    libui::page(lib_, *thumbs_, {0, kTop}, {pane_l_, H - kTop - kStatus}, s ? s->id : "", targets, a);
    lib_actions(a);
  }
  void want_targets(const std::string& part) {
    Targets& t = targets_[part];
    if (!t.requested) {
      t.requested = true;
      t.rev = dstate_.rev;
      dreq("targets", design::request("library.targets", "{\"part\":" + q(part) + "}"), true);
    }
  }

  // ---------------------------------------- WIRING / SIMULATE / PROGRAM ------------------------------------------------------
  void wiring_page(float W, float H) {
    const float lw = flat_.active() ? 0.f : 300.f;  // the flattened canvas takes the whole page; the autowire panel belongs to the picker
    if (lw > 0) {
      begin_pane("##wiring_side", {0, kTop}, {lw, H - kTop - kStatus});
      theme::caption("AUTOWIRE");
      wiring_tab(sel_instance());
      ImGui::End();
    }
    flat_.set_tooltips(tip_mode_);
    const ImVec2 pos{lw, kTop}, size{W - lw, H - kTop - kStatus};
    begin_pane("##wire2d", pos, size, true);
    ImGui::SetCursorPos({0, 0});
    flat_.page(size, dstate_, sel_.empty() ? "" : sel_.back());  // the picker, or the flattened canvas (wire2d.cpp)
    ImGui::End();
  }

  void simulate_panes(float W, float H) {
    const float ph = H - kTop - kStatus;
    if (pane_l_ > 0) {
      begin_pane("##sim_l", {0, kTop}, {pane_l_, ph});
      sim_controls();
      if (has_design_ && opts_.design_api) {
        ImGui::Spacing();
        theme::caption("DESIGN CHECK RUN");
        simulate_tab();
      }
      ImGui::End();
    }
    if (pane_r_ > 0) {
      begin_pane("##sim_r", pane_pos(true, W), {pane_r_, ph});
      prim_inspector();
      ImGui::End();
    }
  }

  void sim_controls() {
    theme::caption("SIMULATION");
    ImGui::Text("tick %llu   t = %.4f s", static_cast<unsigned long long>(cur_.tick), static_cast<double>(cur_.tick) * cur_.dt);
    if (cur_.diverged) ImGui::TextColored(theme::kErr, "solver diverged");
    else if (cur_.running) ImGui::Text("running (rate %.3g)", cur_.rate);
    else ImGui::TextDisabled("paused");
    if (cur_.running) {
      if (theme::button("PAUSE")) send(env_pause(kSource), "pause");
    } else if (theme::button("RUN", true)) {
      send(env_run(kSource, rate_), "run");
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    if (ImGui::InputDouble("rate", &rate_, 0, 0, "%.3g", ImGuiInputTextFlags_EnterReturnsTrue) && cur_.running) send(env_run(kSource, rate_), "run");
    ImGui::SetItemTooltip("sim-seconds per wall-second; 0 = as fast as possible (Enter applies while running)");
    if (ImGui::Button("STEP")) send(env_step(kSource, std::max(1, steps_)), "step");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90);
    ImGui::InputInt("ticks", &steps_);
    if (!status_.empty()) ImGui::TextColored(theme::kWarn, "%s", status_.c_str());
    ImGui::Spacing();
    theme::caption("SCENE");
    ImGui::BeginChild("##scenetree", {0, has_design_ && opts_.design_api ? 260.f : 0.f});
    for (const auto& [k, child] : tree_.kids) draw_node(k, child);
    ImGui::EndChild();
  }

  void prim_inspector() {
    theme::caption("INSPECTOR");
    if (selected_.empty()) ImGui::TextDisabled("select a prim in the scene tree");
    else ImGui::TextWrapped("%s", selected_.c_str());
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
        ImGui::TextWrapped("%s%s", line.c_str(), unit.c_str());
      }
    }
  }

  void program_page(float W, float H) {
    begin_pane("##program", {0, kTop}, {W, H - kTop - kStatus}, true);
    const float half = (W - 48) / 2;
    theme::caption("DESIGN  (READ-ONLY TOML)");
    ImGui::SameLine(half + 40);
    theme::caption("LOG");
    ImGui::PushFont(theme::mono(), 14.f);
    ImGui::BeginChild("##toml", {half, 0});
    if (program_text_.empty()) ImGui::TextDisabled("%s", opts_.design_api ? "(loading)" : "(no design editor attached)");
    else ImGui::TextUnformatted(program_text_.c_str());
    ImGui::EndChild();
    ImGui::SameLine(0, 24);
    ImGui::BeginChild("##log", {0, 0});
    for (const std::string& l : log_) ImGui::TextUnformatted(l.c_str());
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4) ImGui::SetScrollHereY(1.f);
    ImGui::EndChild();
    ImGui::PopFont();
    ImGui::End();
  }
  void log_line(const std::string& s) {
    char t[24];
    std::snprintf(t, sizeof t, "%8.2f  ", ImGui::GetTime());
    log_.push_back(t + s);
    if (log_.size() > 500) log_.pop_front();
  }

  // The DRC list, as the body of the DRC tab (compact) and with its detail.
  void drc_body(bool compact) {
    std::size_t errors = 0, warns = 0, infos = 0;
    for (const design::Finding& f : design_.findings) {
      if (f.severity == "error") ++errors;
      else if (f.severity == "warning") ++warns;
      else ++infos;
    }
    ImGui::TextColored(errors ? theme::kErr : theme::kOk, "%zu errors", errors);
    ImGui::SameLine();
    ImGui::TextColored(theme::kWarn, "%zu warnings", warns);
    ImGui::SameLine();
    ImGui::TextDisabled("%zu info", infos);
    if (!new_keys_.empty()) {
      ImGui::SameLine();
      ImGui::TextDisabled("   %zu new", new_keys_.size());
    }
    if (ImGui::BeginTable("drc", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, {0, compact ? ImGui::GetContentRegionAvail().y - 120 : 0.f})) {
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
        if (ImGui::SmallButton(i.c_str())) set_sel({i});
      }
    }
  }

  void draw_node(const std::string& name, const Node& n) {
    ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
    if (n.kids.empty()) fl |= ImGuiTreeNodeFlags_Leaf;
    if (n.prim && n.path == selected_) fl |= ImGuiTreeNodeFlags_Selected;
    const bool open = ImGui::TreeNodeEx(n.path.c_str(), fl, "%s", name.c_str());
    if (ImGui::IsItemClicked() && n.prim && n.path != selected_) {
      select(n.path);
      sel_.clear();  // sel_instance() then falls back to the instance whose body is this prim
      for (const design::Instance& i : design_.instances)
        if (n.path.size() > i.body.size() && n.path.ends_with("/" + i.body)) sel_ = {i.id};
    }
    if (open) {
      for (const auto& [k, child] : n.kids) draw_node(k, child);
      ImGui::TreePop();
    }
  }

  // ================================ per frame ===========================================================================================
  void ui() {
    const ImGuiIO& io = ImGui::GetIO();
    const float W = io.DisplaySize.x, H = io.DisplaySize.y;
    const double now = ImGui::GetTime();
    if (cam_anim_.active) step_cam_anim(now);
    layout(W, H);
    smooth_viewport(io.DeltaTime);
    view_cam_ = view_camera(W, H);
    if (opts_.design_api) {  // cables stay visible at any zoom in the design views; SIMULATE shows true widths
      const float want = emph_wires_ && mode_ != Mode::Simulate ? 4.f : 0.f;
      RenderStyle st = default_render_style();
      if (st.wire_px != want) {
        st.wire_px = want;
        set_default_render_style(st);
      }
    }
    libui::Actions la;
    if (!pending_mode_.empty() && set_mode(pending_mode_)) pending_mode_.clear();
    libui::tick_typing(lib_, now, la);
    if (opts_.design_api && lib_.dirty && !lib_busy_) {
      lib_busy_ = true;
      dreq("search", [this] { return search_req(); }, true);
    }
    top_bar(W);
    switch (mode_) {
      case Mode::Build: build_panes(W, H); break;
      case Mode::Library: library_page(H); break;
      case Mode::Wiring: wiring_page(W, H); break;
      case Mode::Simulate: simulate_panes(W, H); break;
      case Mode::Program: program_page(W, H); break;
    }
    status_bar(W, H);
    confirm_popup();
    if (!full_page_) viewport_layer(W, H, now);
    if (opts_.overlay && !full_page_) {
      OverlayCtx o;
      o.draw_list = ImGui::GetBackgroundDrawList();
      o.x = vp_.x, o.y = vp_.y, o.w = vp_.z - vp_.x, o.h = vp_.w - vp_.y;
      for (int k = 0; k < 3; ++k) o.cam_pos[k] = view_cam_.pos[k], o.cam_look[k] = view_cam_.lookat[k];
      o.cam_fovy = view_cam_.fovy;
      o.window_w = W, o.window_h = H;
      o.tick = cur_.tick;
      o.sim_time = static_cast<double>(cur_.tick) * cur_.dt;
      opts_.overlay(o);
    }
  }

  void step_cam_anim(double now) {
    const double u = std::clamp((now - cam_anim_.t0) / 0.5, 0.0, 1.0), e = u * u * (3 - 2 * u);
    orbit_.yaw = cam_anim_.yaw0 + std::remainder(cam_anim_.yaw1 - cam_anim_.yaw0, 6.283185307179586) * e;
    orbit_.pitch = cam_anim_.pitch0 + (cam_anim_.pitch1 - cam_anim_.pitch0) * e;
    orbit_.dist = cam_anim_.dist0 + (cam_anim_.dist1 - cam_anim_.dist0) * e;
    if (cam_anim_.move_target)
      for (int k = 0; k < 3; ++k) orbit_.target[k] = cam_anim_.target_from[k] + (cam_anim_.target_to[k] - cam_anim_.target_from[k]) * e;
    if (u >= 1) cam_anim_.active = false;
  }

  // Everything drawn over / done in the 3D viewport: selection, hover, gizmo, guides, the drag-and-drop ghost, camera navigation.
  void viewport_layer(float W, float H, double now) {
    ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const b3::Cam cam = b3::Cam::make(view_cam_, W, H);
    const ImVec2 m = io.MousePos;
    const bool in_vp = m.x >= vp_.x && m.x < vp_.z && m.y >= vp_.y && m.y < vp_.w;
    const bool popup = ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel);
    const bool dragging_part = part_drag_active();
    const bool free_mouse = in_vp && !io.WantCaptureMouse && !popup && !dragging_part && !anim_.active;
    dl->PushClipRect({vp_.x, vp_.y}, {vp_.z, vp_.w}, true);
    const bool build = mode_ == Mode::Build && has_design_;
    b3::GizmoOut go;
    if (build) {
      const design::Instance* sel = sel_instance();
      const auto git = sel ? scene_.find(sel->id) : scene_.end();
      const bool can_edit = opts_.design_api && git != scene_.end() && sel->id != dstate_.root;
      // hover + selection marks
      if (free_mouse && !gizmo_.dragging() && !nav_drag_) hover_inst_ = b3::pick(cam, m, scene_).id;
      else if (!free_mouse && !gizmo_.dragging()) hover_inst_.clear();
      if (tip_force_until_ > now && scene_.count(tip_force_)) hover_inst_ = tip_force_;  // scripted ui.hover
      if (hover_inst_ != hover_prev_) { hover_prev_ = hover_inst_; hover_since_ = now; }
      for (const auto& [id, g] : scene_) {
        const bool s = is_selected(id), h = id == hover_inst_;
        if (!s && !h) continue;
        b3::draw_brackets(dl, cam, g.lo, g.hi, s ? theme::u32(theme::kAccent) : theme::u32({1, 1, 1, 1}, 0.55f), s ? 1.8f : 1.2f);
        if (s && id == sel_.back() && !gizmo_.dragging()) for (const b3::Box& b : g.boxes) b3::draw_box(dl, cam, b, theme::u32(theme::kAccent, 0.35f), 1.f);
      }
      if (can_edit) {
        go = gizmo_.update(cam, scene_, git->second, snap_, free_mouse && !nav_drag_, io.KeyCtrl, vp_);
        if (go.released) finish_gizmo(go, now);
      }
      if (flash_until_ > now && snap_.guides) b3::draw_guides(dl, cam, flash_guides_, static_cast<float>(std::min(1.0, (flash_until_ - now) / 0.5)));
      // open ports of the hovered / selected part: where something could be mated
      if (const auto h = scene_.find(hover_inst_); h != scene_.end() && !gizmo_.dragging())
        for (const b3::Port& p : h->second.ports) {
          ImVec2 s;
          if (!p.used && cam.project(p.pos, s)) dl->AddCircle(s, 4.f, theme::u32(theme::kAccent, 0.7f), 12, 1.2f);
        }
    }
    // navigation + picking
    if (!go.hot || !build) navigation(cam, free_mouse);
    if (m.x != mouse_prev_.x || m.y != mouse_prev_.y) mouse_move_t_ = now;
    mouse_prev_ = m;
    if (build && !hover_inst_.empty() && !go.dragging && !go.hot && now - hover_since_ > 0.35 && !part_drag_active()) {
      const bool forced = tip_force_until_ > now && hover_inst_ == tip_force_;
      if (forced) {  // anchored at the part, offset up and to the right so the card does not hide it
        const auto g = scene_.find(hover_inst_);
        ImVec2 s;
        if (g != scene_.end() && cam.project((g->second.lo + g->second.hi) * 0.5, s)) hover_tip({s.x + 20, s.y - 60});
      } else if (tip_allowed(now) && !nav_drag_ && free_mouse) {
        hover_tip(m);
      }
    }
    drag_drop_layer(cam, dl, now);
    dl->PopClipRect();
    if (build) shortcuts();
  }

  bool part_drag_active() const {
    const ImGuiPayload* pl = ImGui::GetDragDropPayload();
    return (pl && pl->IsDataType("NAPL_PART") && pl->Data) || anim_.active;
  }

  void navigation(const b3::Cam& cam, bool free_mouse) {
    ImGuiIO& io = ImGui::GetIO();
    const double per_px = 2 * orbit_.dist * std::tan(view_cam_.fovy * 3.14159265358979 / 360) / std::max(1.f, io.DisplaySize.y);
    if (free_mouse && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      press_ = true;
      press_moved_ = false;
      press_pos_ = io.MousePos;
    }
    if (free_mouse && (ImGui::IsMouseClicked(ImGuiMouseButton_Right) || ImGui::IsMouseClicked(ImGuiMouseButton_Middle))) pan_ = true;
    if (pan_) {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        for (int k = 0; k < 3; ++k) orbit_.target[k] += (-cam.r[k] * io.MouseDelta.x + cam.u[k] * io.MouseDelta.y) * per_px;
        cam_anim_.active = false;
      } else {
        pan_ = false;
      }
    }
    nav_drag_ = pan_;
    if (press_) {
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (!press_moved_ && std::hypot(io.MousePos.x - press_pos_.x, io.MousePos.y - press_pos_.y) > 4) press_moved_ = true;
        if (press_moved_) {
          orbit_.yaw -= 0.01 * io.MouseDelta.x;
          orbit_.pitch = std::clamp(orbit_.pitch + 0.01 * io.MouseDelta.y, -1.5, 1.5);
          cam_anim_.active = false;
          nav_drag_ = true;
        }
      } else {
        if (!press_moved_ && mode_ == Mode::Build && has_design_) click_select(cam, press_pos_);
        press_ = false;
      }
    }
    if (free_mouse && io.MouseWheel != 0) {
      orbit_.dist = std::max(0.05, orbit_.dist * std::pow(0.9, io.MouseWheel));
      cam_anim_.active = false;
    }
  }

  void click_select(const b3::Cam& cam, ImVec2 px) {
    const std::string id = b3::pick(cam, px, scene_).id;
    const ImGuiIO& io = ImGui::GetIO();
    std::vector<std::string> next = io.KeyShift || io.KeyCtrl ? sel_ : std::vector<std::string>{};
    if (!id.empty() && !std::erase(next, id)) next.push_back(id);
    set_sel(next);
  }

  // Compact hover card: name, "id . mass", three key parameters; offset from `anchor` (the cursor, or the part for a scripted hover) so it
  // never sits under the pointer. When it is shown at all: tip_mode_ (ui.view) and tip_allowed().
  void hover_tip(ImVec2 anchor) {
    const auto it = std::find_if(design_.instances.begin(), design_.instances.end(), [&](const design::Instance& i) { return i.id == hover_inst_; });
    if (it == design_.instances.end()) return;
    want_info(it->part);
    const auto inf = lib_.infos.find(it->part);
    std::string name = inf != lib_.infos.end() && !inf->second.name.empty() ? inf->second.name : it->part;
    if (name.size() > 34) name = name.substr(0, 33) + "...";
    ImGui::SetNextWindowPos({anchor.x + 18, anchor.y + 22});
    ImGui::SetNextWindowSizeConstraints({0, 0}, {250, 1e4f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 7});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 4);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {8, 3});
    ImGui::PushStyleColor(ImGuiCol_PopupBg, ImVec4{0.045f, 0.045f, 0.052f, 0.93f});
    ImGui::BeginTooltip();
    ImGui::PushFont(nullptr, 14.f);
    ImGui::TextUnformatted(name.c_str());
    ImGui::PopFont();
    ImGui::PushFont(nullptr, 12.f);
    ImGui::TextDisabled("%s  |  %.1f g", it->id.c_str(), it->mass * 1000);
    if (inf != lib_.infos.end()) {
      int shown = 0;
      for (const libui::ParamRow& p : inf->second.params) {
        if (shown >= 3) break;
        if (p.name == "mass" || p.value == "(table)" || p.value.empty()) continue;
        ImGui::Text("%s", p.name.c_str());
        ImGui::SameLine(112);
        ImGui::TextDisabled("%s %s", p.value.c_str(), p.unit.c_str());
        ++shown;
      }
    }
    ImGui::PopFont();
    ImGui::EndTooltip();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
  }
  // Hover cards appear for a person who is moving the mouse (auto), always (on) or never (off); a scripted ui.hover overrides.
  bool tip_allowed(double now) const { return tip_mode_ == 1 || (tip_mode_ == 0 && now - mouse_move_t_ < 10.0); }

  void shortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput || !opts_.design_api) return;
    const bool ctrl = io.KeyCtrl;
    if (ctrl) {
      if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) d_undo(io.KeyShift);
      else if (ImGui::IsKeyPressed(ImGuiKey_Y, false)) d_undo(true);
      else if (ImGui::IsKeyPressed(ImGuiKey_D, false) && !sel_.empty()) dreq("edit", design::request("duplicate", "{\"silent\":true,\"instance\":" + q(sel_.back()) + "}"));
      else if (ImGui::IsKeyPressed(ImGuiKey_A, false)) {
        std::vector<std::string> all;
        for (const auto& [id, g] : scene_) all.push_back(id);
        set_sel(all);
      }
      return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_G, false)) gizmo_.mode = b3::Gizmo::Move;
    if (ImGui::IsKeyPressed(ImGuiKey_R, false)) gizmo_.mode = b3::Gizmo::Rotate;
    if (ImGui::IsKeyPressed(ImGuiKey_F, false)) frame_selection();
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
      if (gizmo_.dragging()) gizmo_.cancel();
      else set_sel({});
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_X, false)) {
      std::string ops;
      for (const std::string& id : sel_)
        if (id != dstate_.root) ops += (ops.empty() ? "" : ",") + std::string("{\"op\":\"remove\",\"id\":") + q(id) + "}";
      if (!ops.empty()) dreq("edit", edit_req(ops));  // one undo step for the whole selection
    }
    static const std::pair<ImGuiKey, const char*> views[] = {{ImGuiKey_Keypad1, "front"}, {ImGuiKey_Keypad3, "side"}, {ImGuiKey_Keypad7, "top"},
                                                              {ImGuiKey_Keypad5, "iso"}, {ImGuiKey_Keypad0, "iso"},  {ImGuiKey_1, "front"},
                                                              {ImGuiKey_2, "side"},      {ImGuiKey_3, "top"},        {ImGuiKey_4, "iso"}};
    for (const auto& [k, name] : views)
      if (ImGui::IsKeyPressed(k, false)) camera_preset(name);
  }

  // A gizmo drag ended: send it (one request = one undo step), flash the guides that held it.
  void finish_gizmo(const b3::GizmoOut& go, double now) {
    flash_guides_ = go.guides;
    flash_until_ = now + 1.0;
    const std::string ids = json_list(sel_);
    char b[200];
    std::function<std::string(bool)> make;
    if (go.rotate) {
      std::snprintf(b, sizeof b, "\"axis\":[%.6f,%.6f,%.6f],\"angle_deg\":%.6f", go.axis.x, go.axis.y, go.axis.z, go.angle);
      make = [ids, s = std::string(b)](bool un) { return design::request("gizmo.rotate", "{\"silent\":true,\"instances\":" + ids + "," + s + (un ? ",\"unmate\":true" : "") + "}"); };
    } else {
      std::snprintf(b, sizeof b, "\"delta\":[%.6f,%.6f,%.6f]", go.delta.x, go.delta.y, go.delta.z);
      make = [ids, s = std::string(b)](bool un) { return design::request("gizmo.move", "{\"silent\":true,\"instances\":" + ids + "," + s + (un ? ",\"unmate\":true" : "") + "}"); };
    }
    bool held = false;
    for (const std::string& i : sel_) held = held || is_held(i);
    if (held) confirm_unmate(sel_.back(), make);
    else dreq("edit", make(false));
  }

  // ---- drag and drop of a library part into the viewport (user drag from a card, or a scripted `library.drag` animation) ----
  struct Target {
    std::string to, port;
    b3::V3 pos, port_pos;
    b3::Mat3 R;
  };
  struct Targets {
    bool requested = false, loaded = false;
    std::string rev;
    std::vector<b3::Box> boxes;
    std::vector<Target> list;
  };
  struct Ghost {
    bool valid = false, snapped = false;
    std::string to, port;
    b3::V3 pos;
    b3::Mat3 R;
  };
  struct Anim {  // a scripted drag: the cursor travels from `from` to `to_px`, then (then == "drop") the part is dropped
    bool active = false, ready = false;
    std::string part, to, port, then;
    bool has_point = false;
    b3::V3 point;
    ImVec2 from{0, 0}, to_px{0, 0}, cursor{0, 0};
    double t0 = 0, dur = 1.5, requested_at = 0;
  };

  Ghost ghost_for(const b3::Cam& cam, const std::string& part, ImVec2 mouse, double progress) {
    Ghost g;
    const auto it = targets_.find(part);
    if (it == targets_.end() || !it->second.loaded) return g;
    const Targets& t = it->second;
    g.valid = true;
    double best = 70;
    const Target* pick = nullptr;
    for (const Target& x : t.list) {
      if (anim_.active && !anim_.to.empty() && progress > 0.8) {
        if (x.to == anim_.to && (anim_.port.empty() || x.port == anim_.port)) pick = &x;
        continue;
      }
      ImVec2 s;
      if (!cam.project(x.port_pos, s)) continue;
      const double d = std::hypot(s.x - mouse.x, s.y - mouse.y);
      if (d < best) { best = d; pick = &x; }
    }
    if (anim_.active && !anim_.to.empty() && progress > 0.8 && !pick)  // that port is not in the list (taken?): any target on that instance
      for (const Target& x : t.list)
        if (x.to.substr(0, x.to.find('.')) == anim_.to.substr(0, anim_.to.find('.'))) { pick = &x; break; }
    if (pick) {
      g.snapped = true;
      g.to = pick->to, g.port = pick->port, g.pos = pick->pos, g.R = pick->R;
      return g;
    }
    // free: the cursor ray onto the scene / the ground; the part rests on what it touches
    const b3::Ray r = cam.ray(mouse);
    const b3::Hit h = b3::pick(cam, mouse, scene_);
    b3::V3 p, n{0, 0, 1};
    bool got = false;
    if (!h.id.empty()) { p = h.point, n = h.normal, got = true; }
    b3::V3 gp;
    if (b3::ray_plane(r, {0, 0, 0}, {0, 0, 1}, gp) && (!got || b3::len(gp - r.o) < h.t)) { p = gp, n = {0, 0, 1}, got = true; }
    if (!got) { g.valid = false; return g; }
    double lowest = 0, ext = 0.01;
    for (const b3::Box& b : t.boxes) {
      lowest = std::min(lowest, b.c.z - (std::abs(b.R.m[6]) * b.h.x + std::abs(b.R.m[7]) * b.h.y + std::abs(b.R.m[8]) * b.h.z));
      ext = std::max({ext, b.h.x, b.h.y, b.h.z});
    }
    g.pos = std::abs(n.z) > 0.5 ? b3::V3{p.x, p.y, p.z - lowest * (n.z > 0 ? 1 : -1)} : p + n * ext;
    return g;
  }

  void drag_drop_layer(const b3::Cam& cam, ImDrawList* dl, double now) {
    const ImGuiIO& io = ImGui::GetIO();
    const ImGuiPayload* pl = ImGui::GetDragDropPayload();
    std::string part;
    bool scripted = anim_.active;
    bool user = pl && pl->IsDataType("NAPL_PART") && pl->Data;
    if (user) part = static_cast<const char*>(pl->Data);
    double progress = 0;
    ImVec2 mouse = io.MousePos;
    if (scripted) {
      part = anim_.part;
      want_targets(part);
      const Targets& t = targets_[part];
      if (!anim_.ready) {
        if (t.loaded || now - anim_.requested_at > 5.0) {  // (no design editor / a failed request: fly anyway)
          anim_.ready = true;
          anim_.t0 = now;
          anim_.to_px = anim_target_px(cam, t);
        }
        mouse = anim_.from;
      } else {
        progress = std::clamp((now - anim_.t0) / anim_.dur, 0.0, 1.0);
        const double e = progress * progress * (3 - 2 * progress);
        mouse = {static_cast<float>(anim_.from.x + (anim_.to_px.x - anim_.from.x) * e), static_cast<float>(anim_.from.y + (anim_.to_px.y - anim_.from.y) * e - 60 * std::sin(3.14159265 * e))};
      }
      anim_.cursor = mouse;
    }
    const bool dragging = user || scripted;
    if (dragging) {
      was_dragging_ = true;
      drag_part_ = part;
      if (user) want_targets(part);
      ghost_in_vp_ = mouse.x >= vp_.x && mouse.x < vp_.z && mouse.y >= vp_.y && mouse.y < vp_.w;
      ghost_ = ghost_in_vp_ && (!scripted || anim_.ready) ? ghost_for(cam, part, mouse, progress) : Ghost{};
      const Targets& t = targets_[part];
      if (ghost_.valid && ghost_in_vp_) {
        for (const Target& x : t.list) {  // compatible seats
          ImVec2 s;
          if (cam.project(x.port_pos, s)) dl->AddCircle(s, 5.5f, theme::u32(theme::kAccent, 0.55f), 14, 1.5f);
        }
        std::vector<b3::Box> boxes = t.boxes;
        if (boxes.empty()) boxes.push_back({{0, 0, 0}, {}, {0.01, 0.01, 0.01}});
        b3::draw_ghost(dl, cam, boxes, ghost_.pos, ghost_.R, ghost_.snapped ? theme::u32(theme::kAccent) : theme::u32({1, 1, 1, 1}, 0.8f));
        if (ghost_.snapped) {
          for (const Target& x : t.list)
            if (x.to == ghost_.to && x.port == ghost_.port) {
              ImVec2 s;
              if (cam.project(x.port_pos, s)) {
                dl->AddCircle(s, 11.f, theme::u32(theme::kAccent), 24, 2.f);
                dl->AddText({s.x + 14, s.y - 8}, theme::u32(theme::kAccent), x.to.c_str());
              }
              break;
            }
        }
      }
      // the chip that follows the cursor (above the panels)
      ImDrawList* fg = ImGui::GetForegroundDrawList();
      const ImVec2 sz = ImGui::CalcTextSize(part.c_str());
      fg->AddRectFilled({mouse.x + 12, mouse.y + 10}, {mouse.x + 28 + sz.x, mouse.y + 14 + sz.y + 6}, theme::u32({0, 0, 0, 0.82f}), 3);
      fg->AddRect({mouse.x + 12, mouse.y + 10}, {mouse.x + 28 + sz.x, mouse.y + 14 + sz.y + 6}, theme::u32(theme::kAccent, 0.8f), 3);
      fg->AddText({mouse.x + 20, mouse.y + 14}, theme::u32(theme::kText), part.c_str());
      if (scripted) {  // a visible cursor for recordings
        const ImVec2 c = mouse;
        fg->AddTriangleFilled(c, {c.x + 11, c.y + 14}, {c.x + 3, c.y + 11}, IM_COL32(255, 255, 255, 240));
        fg->AddTriangle(c, {c.x + 11, c.y + 14}, {c.x + 3, c.y + 11}, IM_COL32(0, 0, 0, 255), 1.f);
      }
    }
    const bool finish_user = was_dragging_ && !scripted && ImGui::IsMouseReleased(ImGuiMouseButton_Left);
    const bool finish_script = scripted && anim_.ready && progress >= 1.0;
    if (finish_user || finish_script) {
      if (ghost_.valid && ghost_in_vp_ && (!finish_script || anim_.then == "drop")) drop_ghost();
      was_dragging_ = false;
      ghost_ = {};
      anim_.active = false;
    }
  }
  ImVec2 anim_target_px(const b3::Cam& cam, const Targets& t) const {
    ImVec2 s{(vp_.x + vp_.z) / 2, (vp_.y + vp_.w) / 2};
    if (!anim_.to.empty()) {
      for (const Target& x : t.list)
        if (x.to == anim_.to && (anim_.port.empty() || x.port == anim_.port)) {
          cam.project(x.port_pos, s);
          return s;
        }
      const auto g = scene_.find(anim_.to.substr(0, anim_.to.find('.')));
      if (g != scene_.end()) cam.project(g->second.pos, s);
    } else if (anim_.has_point) {
      cam.project(anim_.point, s);
    }
    return s;
  }

  void drop_ghost() {
    const std::string args = ghost_.snapped
        ? "{\"silent\":true,\"part\":" + q(drag_part_) + ",\"to\":" + q(ghost_.to) + ",\"port\":" + q(ghost_.port) + "}"
        : "{\"silent\":true,\"part\":" + q(drag_part_) + ",\"point\":" + point_json(ghost_.pos) + "}";
    dreq("edit", design::request("library.drop", args));
  }

  // ---- scripted UI events (backend ui_poll) and script actions ----
  void ui_event(const design::Json& e) {
    const std::string cmd = e.s("cmd");
    const double now = ImGui::GetTime();
    log_line("ui " + cmd);
    if (cmd == "mode") set_mode(e.s("name"));
    else if (cmd == "camera") {
      if (!e.s("preset").empty()) camera_preset(e.s("preset"));
      else cam_anim_ = {true, now, orbit_.yaw, orbit_.pitch, orbit_.dist, e.n("yaw", orbit_.yaw), e.n("pitch", orbit_.pitch), e.n("dist", orbit_.dist)};
    } else if (cmd == "frame") {
      std::vector<std::string> ids;
      for (const design::Json& i : e.at("instances").arr) ids.push_back(i.str);
      if (!ids.empty()) set_sel(ids);
      frame_selection();
    } else if (cmd == "gizmo") {
      const std::string m = e.s("mode");
      gizmo_.mode = m == "rotate" ? b3::Gizmo::Rotate : m == "none" ? b3::Gizmo::Off : b3::Gizmo::Move;
    } else if (cmd == "view") {
      const std::string t = e.s("tooltips");
      tip_mode_ = t == "on" ? 1 : t == "off" ? 2 : 0;
      emph_wires_ = e.at("emphasise_wires").kind == design::Json::Bool || e.at("emphasise_wires").kind == design::Json::Num ? e.at("emphasise_wires").num != 0 : emph_wires_;
    } else if (cmd == "toast") {
      flat_.notify(e.s("text"), e.at("bad").num != 0);
    } else if (cmd == "hover") {
      tip_force_ = e.s("instance");
      tip_force_until_ = now + e.n("seconds", 6.0);
      hover_since_ = now - 1;
      if (mode_ != Mode::Build) set_mode("build");
    } else if (cmd == "guides") snap_.guides = e.at("on").num != 0;
    else if (cmd == "snap") {
      snap_.grid = e.n("grid_m", snap_.grid), snap_.angle = e.n("angle_deg", snap_.angle);
      snap_.edges = e.at("snap_edges").num != 0, snap_.ports = e.at("snap_ports").num != 0;
    } else if (cmd == "select") {
      std::vector<std::string> ids = e.at("add").num != 0 ? sel_ : std::vector<std::string>{};
      for (const design::Json& i : e.at("instances").arr) ids.push_back(i.str);
      sel_synced_ = ids;  // (the backend already knows)
      set_sel(ids);
    } else if (cmd == "search") {
      lib_.category = e.s("category");
      lib_.filters.clear();
      for (const auto& [k, r] : e.at("filters").obj)
        if (r.arr.size() >= 2) {
          const auto rg = lib_.ranges.find(k);
          lib_.filters[k] = {true, r.arr[0].kind == design::Json::Num ? static_cast<float>(r.arr[0].num) : rg != lib_.ranges.end() ? static_cast<float>(rg->second.lo) : 0.f,
                             r.arr[1].kind == design::Json::Num ? static_cast<float>(r.arr[1].num) : rg != lib_.ranges.end() ? static_cast<float>(rg->second.hi) : 1e9f};
        }
      const double ms = e.n("type_ms");
      if (ms > 0) {
        lib_.query[0] = 0;
        lib_.type_target = e.s("text");
        lib_.type_ms = ms;
        lib_.type_next = now;
      } else {
        std::snprintf(lib_.query, sizeof lib_.query, "%s", e.s("text").c_str());
      }
      lib_.dirty = true;
    } else if (cmd == "color") {
      const std::string id = e.s("instance");
      set_sel({id});
      if (e.at("rgba").arr.size() >= 3) {
        std::array<float, 4> c{0, 0, 0, 1};
        for (std::size_t i = 0; i < 4 && i < e.at("rgba").arr.size(); ++i) c[i] = static_cast<float>(e.at("rgba").arr[i].num);
        color_pick_[id] = c;
      }
      right_tab_ = 0;
      color_flash_ = now + 1.8;
    } else if (cmd == "move" || cmd == "rotate") {
      set_sel({e.s("instance")});
      flash_guides_.clear();
      for (const design::Json& g : e.at("guides").arr) {
        b3::Guide x;
        x.axis = static_cast<int>(g.n("axis"));
        double v[3];
        design::vec_from(g.at("a"), v, 3), x.a = {v[0], v[1], v[2]};
        design::vec_from(g.at("b"), v, 3), x.b = {v[0], v[1], v[2]};
        flash_guides_.push_back(x);
      }
      flash_until_ = now + 1.4;
    } else if (cmd == "drag") {
      anim_ = {};
      anim_.active = true;
      anim_.part = e.s("part");
      anim_.to = e.s("to");
      anim_.port = e.s("port");
      anim_.then = e.s("then");
      anim_.dur = e.n("duration", 1.5);
      if (e.at("point").kind == design::Json::Arr) {
        double v[3];
        design::vec_from(e.at("point"), v, 3);
        anim_.has_point = true;
        anim_.point = {v[0], v[1], v[2]};
      }
      if (e.at("from").kind == design::Json::Arr && e.at("from").arr.size() >= 2) anim_.from = {static_cast<float>(e.at("from").arr[0].num), static_cast<float>(e.at("from").arr[1].num)};
      else if (const auto c = lib_.card_at.find(anim_.part); c != lib_.card_at.end()) anim_.from = c->second;
      else anim_.from = {vp_.x > 60 ? 130.f : 60.f, vp_.y + 140};
      if (mode_ == Mode::Library) set_mode("build");  // the drop target is the 3D view
      want_targets(anim_.part);
      anim_.requested_at = now;
    }
  }

  void script_ui(const ScriptCmd& c) {
    if (c.action == "mode") {
      if (!set_mode(c.args[0]) && opts_.design_api) pending_mode_ = c.args[0];  // the first state has not arrived yet: retry each frame
    } else if (c.action == "uisel") {
      std::vector<std::string> ids;
      std::stringstream ss(c.args[0]);
      for (std::string s; std::getline(ss, s, ',');) ids.push_back(s);
      set_sel(ids);
    } else if (c.action == "search") {
      std::string t;
      for (const std::string& a : c.args) t += (t.empty() ? "" : " ") + a;
      std::snprintf(lib_.query, sizeof lib_.query, "%s", t.c_str());
      lib_.dirty = true;
    } else if (c.action == "uicam") camera_preset(c.args[0]);
    else if (c.action == "uiframe") frame_selection();
  }

  void rebuild_scene() {
    scene_.clear();
    for (const auto& [id, g] : dstate_.geo) {
      b3::InstGeo o;
      o.id = id;
      o.pos = {g.pos[0], g.pos[1], g.pos[2]};
      std::copy(g.R, g.R + 9, o.R.m);
      o.lo = {g.lo[0], g.lo[1], g.lo[2]};
      o.hi = {g.hi[0], g.hi[1], g.hi[2]};
      for (const design::GeoBox& b : g.boxes) {
        b3::Box x;
        x.c = {b.c[0], b.c[1], b.c[2]};
        std::copy(b.R, b.R + 9, x.R.m);
        x.h = {b.h[0], b.h[1], b.h[2]};
        o.boxes.push_back(x);
      }
      o.has_color = g.has_color;
      std::copy(g.color, g.color + 4, o.color);
      o.root = id == dstate_.root;
      o.free = std::find(dstate_.free.begin(), dstate_.free.end(), id) != dstate_.free.end();
      if (const auto p = dstate_.ports.find(id); p != dstate_.ports.end())
        for (const design::Port& q : p->second)
          if (q.placed && q.kind == "mech") o.ports.push_back({q.name, {q.pos[0], q.pos[1], q.pos[2]}, {q.axis[0], q.axis[1], q.axis[2]}, q.used});
      for (const design::Instance& i : dstate_.report.instances)
        if (i.id == id) o.part = i.part;
      scene_[id] = std::move(o);
    }
  }

  struct CamAnim {
    bool active = false;
    double t0 = 0, yaw0 = 0, pitch0 = 0, dist0 = 0, yaw1 = 0, pitch1 = 0, dist1 = 0;
    b3::V3 target_from, target_to;
    bool move_target = false;
  };

  Host& host_;
  ViewerOptions opts_;
  std::vector<ScriptCmd> script_;
  Frame cur_;
  std::uint64_t frame_scene_ = 0;
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
  std::chrono::steady_clock::time_point last_poll_ = std::chrono::steady_clock::now(), last_ev_ = std::chrono::steady_clock::now();
  Placement place_;
  int cycling_ = -1, finding_ = -1;
  // ---- the split UI (modes, panes, library, 3D interaction) ----
  Mode mode_ = Mode::Simulate;
  ImVec4 vp_{0, 0, 0, 0};  // viewport rectangle x0 y0 x1 y1 (window pixels), eased toward vp_goal_
  ImVec4 vp_goal_{0, 0, 0, 0};
  bool vp_init_ = false;
  float pane_l_ = 0, pane_r_ = 0;
  bool full_page_ = false;
  Camera view_cam_;
  int right_tab_ = 0;  // BUILD right pane: 0 inspect, 1 parts, 2 drc, 3 place
  libui::Model lib_;
  libui::Thumbs* thumbs_ = nullptr;
  bool lib_busy_ = false, snap_requested_ = false;
  std::string pending_mode_;
  std::map<std::string, Targets> targets_;
  b3::Scene scene_;
  b3::Gizmo gizmo_;
  b3::Settings snap_;
  std::vector<std::string> sel_, sel_synced_;
  std::string hover_inst_, hover_prev_;
  double hover_since_ = 0;
  int tip_mode_ = 0;  // ui.view tooltips: 0 auto (while a person moves the mouse), 1 on, 2 off
  std::string tip_force_;  // ui.hover: this instance's card is shown until tip_force_until_
  double tip_force_until_ = 0, mouse_move_t_ = -1e9;
  ImVec2 mouse_prev_{-1, -1};
  bool emph_wires_ = true;  // ui.view emphasise_wires: cables >= 4 px wide in BUILD / LIBRARY / WIRING (SIMULATE keeps true widths)
  int thumb_rev_ = 0;       // backend: library thumbnails finished so far (changes -> ask the library again)
  bool press_ = false, press_moved_ = false, pan_ = false, nav_drag_ = false;
  ImVec2 press_pos_{0, 0};
  std::vector<b3::Guide> flash_guides_;
  double flash_until_ = 0, color_flash_ = 0;
  std::map<std::string, std::array<float, 4>> color_pick_;
  bool color_dirty_ = false;
  CamAnim cam_anim_;
  Anim anim_;
  Ghost ghost_;
  bool ghost_in_vp_ = false, was_dragging_ = false;
  std::string drag_part_;
  std::function<std::string(bool)> confirm_;
  std::string confirm_id_;
  bool confirm_open_ = false;
  std::deque<std::string> log_;
  std::string program_text_;
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
    } else if (const auto [ulo, uhi] = ui_action_args(c.action); ulo >= 0) {
      const int n = static_cast<int>(c.args.size());
      if (n < ulo || n > uhi) throw std::runtime_error("script entry '" + entry + "': " + c.action + " takes " + std::to_string(ulo) + (uhi > ulo ? ".." + std::to_string(uhi) : "") + " argument(s)");
      if (c.action == "mode" && c.args[0] != "build" && c.args[0] != "library" && c.args[0] != "wiring" && c.args[0] != "simulate" && c.args[0] != "program")
        throw std::runtime_error("script: mode build|library|wiring|simulate|program");
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
