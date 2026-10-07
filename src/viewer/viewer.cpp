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
#include <future>
#include <map>
#include <sstream>
#include <stdexcept>

#include "napl_generated.h"
#include "render/window_renderer.h"
#include "viewer/design.h"

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
Env env_query(const std::string& src, const std::string& path, const std::string& attr) {
  return envelope(src, napl::Command::Query,
                  [&](auto& f) { return napl::CreateQueryDirect(f, path.c_str(), attr.c_str(), "").Union(); });
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
    ImGuiBackends gui(window.w, renderer.handles());

    for (int n_frame = 0; !glfwWindowShouldClose(window.w) && !(opts_.frames > 0 && n_frame >= opts_.frames) &&
                          !(opts_.stop && opts_.stop->load());
         ++n_frame) {
      glfwPollEvents();
      for (const ScriptCmd& c : script_) {
        if (c.frame != n_frame) continue;
        if (c.action == "select") select(c.args[0]);
        else if (c.action == "inst") select_instance(c.args[0]);
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
      const bool drawn = renderer.frame(cur_.poses, orbit_.camera(), fw, fh, [](VkCommandBuffer cb) {
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cb);
      });
      if (!drawn) glfwWaitEventsTimeout(0.05);  // minimised or swapchain rebuilding: do not spin
    }
    for (auto& p : cmds_) p.second.wait();  // scripted / UI commands issued last are applied before we return
    poll();
  }

 private:
  void send(Env e, const std::string& what) { cmds_.emplace_back(what, host_.submit(std::move(e))); }

  static bool ready(std::future<Reply>& f) { return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }

  void request_prims() { prims_ = host_.submit(env_query(kSource, "", "")); }

  void adopt(Frame& f, WindowRenderer& renderer) {
    if (f.new_scene) {
      renderer.set_scene(f.visuals);
      orbit_.fit(f.visuals, f.poses);
    }
    if (f.scene != scene_) {  // first frame, or the model was replaced: the prim list changed too
      if (scene_ != 0) select("");  // (a first-frame selection, e.g. scripted, is kept)
      scene_ = f.scene;
      request_prims();
    }
    f.visuals.clear();
    cur_ = std::move(f);
  }

  // Collects finished replies without blocking the frame.
  void poll() {
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

  void design_ui() {
    ImGui::SetNextWindowPos({10, 440}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({440, 300}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Design");
    ImGui::Text("%s   total %.4f kg   %zu instances", design_.assembly.c_str(), design_.mass_total, design_.instances.size());
    if (ImGui::CollapsingHeader("mass budget")) {
      for (const auto& [g, m] : design_.by_group) ImGui::Text("%-16s %9.4f kg", g.c_str(), m);
      for (const design::Target& t : design_.targets) {
        ImGui::TextColored(t.within ? ImVec4{0.5f, 1, 0.5f, 1} : ImVec4{1, 0.6f, 0.2f, 1}, "target %s: %.4f vs %.4f kg (%+.4f) %s",
                           t.name.c_str(), t.model, t.target, t.gap, t.within ? "within" : "OUTSIDE");
      }
    }
    if (ImGui::CollapsingHeader("instances", ImGuiTreeNodeFlags_DefaultOpen)) {
      std::map<std::string, std::vector<const design::Instance*>> by_group;
      for (const design::Instance& i : design_.instances) by_group[i.group.empty() ? "(none)" : i.group].push_back(&i);
      for (const auto& [g, list] : by_group) {
        if (!ImGui::TreeNodeEx(g.c_str(), ImGuiTreeNodeFlags_DefaultOpen, "%s (%zu)", g.c_str(), list.size())) continue;
        for (const design::Instance* i : list) {
          const std::string label = i->id + "   " + i->part + "   " + std::to_string(i->mass * 1000).substr(0, 7) + " g";
          if (ImGui::Selectable(label.c_str(), selected_.size() > i->body.size() && selected_.ends_with("/" + i->body)))
            select_instance(i->id);
        }
        ImGui::TreePop();
      }
    }
    std::size_t errors = 0, warns = 0;
    for (const design::Finding& f : design_.findings) {
      if (f.severity == "error") ++errors;
      else if (f.severity == "warning") ++warns;
    }
    if (ImGui::CollapsingHeader(("DRC findings: " + std::to_string(errors) + " errors, " + std::to_string(warns) + " warnings###drc").c_str(),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      int n = 0;
      for (const design::Finding& f : design_.findings) {
        const ImVec4 col = f.severity == "error" ? ImVec4{1, 0.35f, 0.35f, 1}
                           : f.severity == "warning" ? ImVec4{1, 0.8f, 0.3f, 1}
                                                     : ImVec4{0.7f, 0.8f, 1, 1};
        ImGui::PushID(n++);
        ImGui::PushStyleColor(ImGuiCol_Text, col);
        const bool hit = ImGui::Selectable((f.severity + "  " + f.code + "  " + f.message).c_str());
        ImGui::PopStyleColor();
        if (hit)
          for (const std::string& w : f.where)  // selects the first named id that is an instance
            if (std::any_of(design_.instances.begin(), design_.instances.end(), [&](const design::Instance& i) { return i.id == w; })) {
              select_instance(w);
              break;
            }
        if (!f.fix.empty()) ImGui::SetItemTooltip("fix: %s", f.fix.c_str());
        ImGui::PopID();
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
    ImGui::SetNextWindowSize({320, 420}, ImGuiCond_FirstUseEver);
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
  }

  Host& host_;
  ViewerOptions opts_;
  std::vector<ScriptCmd> script_;
  Frame cur_;
  std::uint64_t scene_ = 0;
  Orbit orbit_;
  Node tree_;
  design::Report design_;  // read-only: the report of the `napl design build` this model came from
  bool has_design_ = false;
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
  throw std::runtime_error("script: unknown action '" + c.action + "' (run, pause, step, setattr)");
}
