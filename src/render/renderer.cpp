#include "render/renderer.h"

#include <volk.h>

#define VMA_STATIC_VULKAN_FUNCTIONS 0
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 1
#define VMA_VULKAN_VERSION 1003000
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geom_spv.h"
#include "render/window_renderer.h"

namespace {
constexpr VkFormat kColorFmt = VK_FORMAT_R8G8B8A8_UNORM;  // headless colour target; a window uses its swapchain format
constexpr VkFormat kLinFmt = VK_FORMAT_R32_SFLOAT;  // linear depth in metres
constexpr VkFormat kDepthFmt = VK_FORMAT_D32_SFLOAT;
constexpr double kNear = 0.05, kFar = 1000;  // metres; depth testing only (the linear-depth output is exact)
constexpr double kPi = 3.14159265358979323846;
constexpr double kUnboundedPlane = 500;  // half extent drawn for a plane of size <= 0 (inside kFar)
constexpr float kClearColor[4] = {0.10f, 0.12f, 0.16f, 1.0f};

void check(VkResult r, const char* what) {
  if (r != VK_SUCCESS) throw std::runtime_error(std::string("Vulkan: ") + what + " failed (VkResult " + std::to_string(r) + ")");
}

// ---- CPU tessellation: interleaved position xyz + normal xyz in the geom frame, size baked in -------------------------
using Verts = std::vector<float>;
void vert(Verts& o, double px, double py, double pz, double nx, double ny, double nz) {
  for (double d : {px, py, pz, nx, ny, nz}) o.push_back(static_cast<float>(d));
}
struct Pn { double p[3], n[3]; };
void quad(Verts& o, const Pn& a, const Pn& b, const Pn& c, const Pn& d) {
  for (const Pn* q : {&a, &b, &c, &a, &c, &d}) vert(o, q->p[0], q->p[1], q->p[2], q->n[0], q->n[1], q->n[2]);
}

// Surface of revolution about local z. A strip is a polyline of (rho, z, n_rho, n_z) profile points.
struct Prof { double rho, z, nr, nz; };
void revolve(Verts& o, const std::vector<Prof>& strip, int segs) {
  auto at = [&](const Prof& q, int j) {
    const double phi = 2 * kPi * j / segs, c = std::cos(phi), s = std::sin(phi);
    return Pn{{q.rho * c, q.rho * s, q.z}, {q.nr * c, q.nr * s, q.nz}};
  };
  for (std::size_t i = 0; i + 1 < strip.size(); ++i)
    for (int j = 0; j < segs; ++j) quad(o, at(strip[i], j), at(strip[i], j + 1), at(strip[i + 1], j + 1), at(strip[i + 1], j));
}

// Capsule along z with hemispherical caps; radius r, half length h (h = 0: sphere).
std::vector<Prof> capsule_profile(double r, double h, int rings) {
  std::vector<Prof> s;
  for (int i = 0; i <= rings; ++i) {  // bottom pole -> equator
    const double th = -kPi / 2 + (kPi / 2) * i / rings;
    s.push_back({r * std::cos(th), -h + r * std::sin(th), std::cos(th), std::sin(th)});
  }
  for (int i = 0; i <= rings; ++i) {  // equator -> top pole
    const double th = (kPi / 2) * i / rings;
    s.push_back({r * std::cos(th), h + r * std::sin(th), std::cos(th), std::sin(th)});
  }
  return s;
}

Verts tessellate(const GeomVisual& g) {
  Verts o;
  const double a = g.size[0], b = g.size[1], c = g.size[2];
  constexpr int kSegs = 32, kRings = 8;
  switch (g.type) {
    case GeomType::sphere: revolve(o, capsule_profile(a, 0, kRings), kSegs); break;
    case GeomType::capsule: revolve(o, capsule_profile(a, b, kRings), kSegs); break;
    case GeomType::ellipsoid: {  // unit sphere scaled; normal of x^2/a^2+.. = 1 is (x/a^2, ..) = (nx/a, ..)
      Verts s;
      revolve(s, capsule_profile(1, 0, kRings), kSegs);
      for (std::size_t i = 0; i < s.size(); i += 6) {
        const double sc[3] = {a, b, c};
        double n[3] = {s[i + 3] / sc[0], s[i + 4] / sc[1], s[i + 5] / sc[2]};
        const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        vert(o, s[i] * a, s[i + 1] * b, s[i + 2] * c, n[0] / len, n[1] / len, n[2] / len);
      }
      break;
    }
    case GeomType::cylinder:
      revolve(o, {{0, -b, 0, -1}, {a, -b, 0, -1}}, kSegs);
      revolve(o, {{a, -b, 1, 0}, {a, b, 1, 0}}, kSegs);
      revolve(o, {{a, b, 0, 1}, {0, b, 0, 1}}, kSegs);
      break;
    case GeomType::box: {
      const double h[3] = {a, b, c};
      for (int ax = 0; ax < 3; ++ax)
        for (double sgn : {-1.0, 1.0}) {
          const int u = (ax + 1) % 3, v = (ax + 2) % 3;
          auto corner = [&](double su, double sv) {
            Pn q{};
            q.p[ax] = sgn * h[ax];
            q.p[u] = su * h[u];
            q.p[v] = sv * h[v];
            q.n[ax] = sgn;
            return q;
          };
          quad(o, corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1));
        }
      break;
    }
    case GeomType::plane: {
      const double hx = a > 0 ? a : kUnboundedPlane, hy = b > 0 ? b : kUnboundedPlane;
      const Pn q0{{-hx, -hy, 0}, {0, 0, 1}}, q1{{hx, -hy, 0}, {0, 0, 1}}, q2{{hx, hy, 0}, {0, 0, 1}}, q3{{-hx, hy, 0}, {0, 0, 1}};
      quad(o, q0, q1, q2, q3);
      break;
    }
    case GeomType::mesh: o = g.mesh; break;
    case GeomType::other: break;
  }
  return o;
}

std::uint64_t hash_visuals(std::span<const GeomVisual> v) {  // FNV-1a 64 over everything tessellation depends on
  std::uint64_t h = 14695981039346656037ull;
  auto mix = [&](const void* p, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) h = (h ^ static_cast<const unsigned char*>(p)[i]) * 1099511628211ull;
  };
  for (const GeomVisual& g : v) {
    mix(&g.type, sizeof g.type);
    mix(g.size, sizeof g.size);
    mix(g.rgba, sizeof g.rgba);
    mix(&g.group, sizeof g.group);
    mix(g.mesh.data(), g.mesh.size() * sizeof(float));
  }
  return h;
}

// ---- camera-relative matrices (double) ---------------------------------------------------------------------------------
struct V3 { double x, y, z; };
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
V3 unit(V3 a) {
  const double l = std::sqrt(dot(a, a));
  return {a.x / l, a.y / l, a.z / l};
}

struct Push {  // 128 bytes = the guaranteed push-constant minimum; mirrors `Push` in geom.slang
  float mvp[16];  // 4 columns
  float rot[12];  // 3 columns (xyz + pad)
  float color[4];
};
static_assert(sizeof(Push) == 128);

struct View {
  V3 pos, r, u, f;  // camera position and basis rows: right, up, forward
  double sx, sy;    // clip scale
};
View make_view(const Camera& c, int w, int h) {
  View v;
  v.pos = {c.pos[0], c.pos[1], c.pos[2]};
  v.f = unit(V3{c.lookat[0], c.lookat[1], c.lookat[2]} - v.pos);
  const V3 up0 = std::abs(v.f.z) > 0.9999 ? V3{0, 1, 0} : V3{0, 0, 1};  // straight up/down: image top = +y
  v.r = unit(cross(v.f, up0));
  v.u = cross(v.r, v.f);
  const double t = 1 / std::tan(c.fovy * kPi / 360);
  v.sx = t * h / w;
  v.sy = -t;  // Vulkan clip y points down
  return v;
}

Push make_push(const View& v, const GeomPose& g, const float rgba[4]) {
  constexpr double A = kFar / (kFar - kNear), B = -kFar * kNear / (kFar - kNear);
  const V3 t_rel = V3{g.pos[0], g.pos[1], g.pos[2]} - v.pos;  // the double-precision camera-relative step
  const V3 rows[3] = {v.r, v.u, v.f};
  double m[3][3], tv[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) m[i][j] = rows[i].x * g.rot[j] + rows[i].y * g.rot[3 + j] + rows[i].z * g.rot[6 + j];
    tv[i] = dot(rows[i], t_rel);
  }
  Push p{};
  for (int j = 0; j < 3; ++j) {
    const double col[4] = {v.sx * m[0][j], v.sy * m[1][j], A * m[2][j], m[2][j]};
    for (int k = 0; k < 4; ++k) p.mvp[4 * j + k] = static_cast<float>(col[k]);
    for (int k = 0; k < 3; ++k) p.rot[4 * j + k] = static_cast<float>(g.rot[3 * k + j]);
  }
  const double col3[4] = {v.sx * tv[0], v.sy * tv[1], A * tv[2] + B, tv[2]};
  for (int k = 0; k < 4; ++k) p.mvp[12 + k] = static_cast<float>(col3[k]);
  std::memcpy(p.color, rgba, sizeof p.color);
  return p;
}
}  // namespace

struct Renderer::Impl {
  struct Draw {
    std::uint32_t first = 0, count = 0;
    float rgba[4] = {};
    bool visible = false;
  };
  struct Target {
    int w = 0, h = 0;
    VkImage color = nullptr, lin = nullptr, depth = nullptr;
    VmaAllocation color_a = nullptr, lin_a = nullptr, depth_a = nullptr;
    VkImageView color_v = nullptr, lin_v = nullptr, depth_v = nullptr;
    VkBuffer rb_color = nullptr, rb_lin = nullptr;  // host-visible readback
    VmaAllocation rb_color_a = nullptr, rb_lin_a = nullptr;
    void *color_ptr = nullptr, *lin_ptr = nullptr;
  };

  VkInstance inst = nullptr;
  VkPhysicalDevice pd = nullptr;
  VkDevice dev = nullptr;
  VkQueue queue = nullptr;
  std::uint32_t qfam = 0;
  VmaAllocator vma = nullptr;
  VkCommandPool pool = nullptr;
  VkCommandBuffer cb = nullptr;
  VkFence fence = nullptr;
  VkShaderModule shader = nullptr;
  VkPipelineLayout layout = nullptr;
  VkPipeline pipe = nullptr;
  std::string name;
  VkBuffer vbuf = nullptr;
  VmaAllocation vbuf_a = nullptr;
  std::vector<Draw> draws;
  Target tgt;
  VkFormat color_fmt = kColorFmt;

  // Window mode (WindowRenderer): surface + swapchain, one frame in flight.
  const WindowRenderer::Surface* win = nullptr;
  VkSurfaceKHR surface = nullptr;
  VkColorSpaceKHR color_space = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
  VkSwapchainKHR swapchain = nullptr;
  VkExtent2D sc_extent{};
  int req_w = 0, req_h = 0;  // framebuffer size the swapchain was last built for
  bool sc_stale = false;
  std::vector<VkImage> sc_images;
  std::vector<VkImageView> sc_views;
  std::vector<VkSemaphore> sc_done;  // render finished, per swapchain image
  VkSemaphore acquire_sem = nullptr;

  explicit Impl(const WindowRenderer::Surface* window = nullptr) : win(window) {
    if (volkInitialize() != VK_SUCCESS) throw std::runtime_error("no Vulkan loader (libvulkan) found on this machine");
    try {
      init();
    } catch (...) {
      destroy();
      throw;
    }
  }
  ~Impl() { destroy(); }

  void init() {
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "napl";
    app.apiVersion = VK_API_VERSION_1_3;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app;
    if (win) {
      ici.enabledExtensionCount = static_cast<std::uint32_t>(win->instance_extensions.size());
      ici.ppEnabledExtensionNames = win->instance_extensions.data();
    }
    const VkResult ir = vkCreateInstance(&ici, nullptr, &inst);
    if (ir == VK_ERROR_INCOMPATIBLE_DRIVER)
      throw std::runtime_error("no Vulkan device: no usable Vulkan driver (ICD) found (check VK_DRIVER_FILES)");
    if (ir == VK_ERROR_EXTENSION_NOT_PRESENT)
      throw std::runtime_error("no Vulkan device: the driver lacks the window-system extensions needed to show a window");
    check(ir, "vkCreateInstance");
    volkLoadInstance(inst);
    if (win) surface = win->create(inst);

    std::uint32_t n = 0;
    check(vkEnumeratePhysicalDevices(inst, &n, nullptr), "vkEnumeratePhysicalDevices");
    std::vector<VkPhysicalDevice> pds(n);
    check(vkEnumeratePhysicalDevices(inst, &n, pds.data()), "vkEnumeratePhysicalDevices");
    for (VkPhysicalDevice cand : pds) {
      VkPhysicalDeviceProperties props;
      vkGetPhysicalDeviceProperties(cand, &props);
      if (props.apiVersion < VK_API_VERSION_1_3) continue;
      VkPhysicalDeviceVulkan13Features f13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
      VkPhysicalDeviceFeatures2 f2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2, &f13};
      vkGetPhysicalDeviceFeatures2(cand, &f2);
      if (!f13.dynamicRendering || !f13.synchronization2) continue;
      VkFormatProperties fp;
      vkGetPhysicalDeviceFormatProperties(cand, kDepthFmt, &fp);
      if (!(fp.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)) continue;
      std::uint32_t nq = 0;
      vkGetPhysicalDeviceQueueFamilyProperties(cand, &nq, nullptr);
      std::vector<VkQueueFamilyProperties> qs(nq);
      vkGetPhysicalDeviceQueueFamilyProperties(cand, &nq, qs.data());
      if (win && !can_present(cand)) continue;
      for (std::uint32_t i = 0; i < nq && !pd; ++i) {
        VkBool32 present = !win;
        if (win && vkGetPhysicalDeviceSurfaceSupportKHR(cand, i, surface, &present) != VK_SUCCESS) present = VK_FALSE;
        if ((qs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && present) {
          pd = cand;
          qfam = i;
          name = props.deviceName;
        }
      }
      if (pd) break;
    }
    if (!pd)
      throw std::runtime_error("no Vulkan device: none of the " + std::to_string(n) + " physical device(s) supports Vulkan 1.3 with dynamic rendering" +
                               (win ? " and presenting to this window (an X server without DRI3 cannot present from a hardware GPU)" : ""));

    const float prio = 1;
    VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = qfam;
    qci.queueCount = 1;
    qci.pQueuePriorities = &prio;
    VkPhysicalDeviceVulkan13Features f13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
    f13.dynamicRendering = VK_TRUE;
    f13.synchronization2 = VK_TRUE;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO, &f13};
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    const char* swapchain_ext = VK_KHR_SWAPCHAIN_EXTENSION_NAME;
    if (win) {
      dci.enabledExtensionCount = 1;
      dci.ppEnabledExtensionNames = &swapchain_ext;
    }
    check(vkCreateDevice(pd, &dci, nullptr, &dev), "vkCreateDevice");
    volkLoadDevice(dev);
    vkGetDeviceQueue(dev, qfam, 0, &queue);

    VmaVulkanFunctions fns{};
    fns.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
    fns.vkGetDeviceProcAddr = vkGetDeviceProcAddr;
    VmaAllocatorCreateInfo aci{};
    aci.physicalDevice = pd;
    aci.device = dev;
    aci.instance = inst;
    aci.vulkanApiVersion = VK_API_VERSION_1_3;
    aci.pVulkanFunctions = &fns;
    check(vmaCreateAllocator(&aci, &vma), "vmaCreateAllocator");

    VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = qfam;
    check(vkCreateCommandPool(dev, &pci, nullptr, &pool), "vkCreateCommandPool");
    VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cai.commandPool = pool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    check(vkAllocateCommandBuffers(dev, &cai, &cb), "vkAllocateCommandBuffers");
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    check(vkCreateFence(dev, &fci, nullptr, &fence), "vkCreateFence");
    if (win) {
      VkSemaphoreCreateInfo semi{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
      check(vkCreateSemaphore(dev, &semi, nullptr, &acquire_sem), "vkCreateSemaphore");
      std::uint32_t nf = 0;
      check(vkGetPhysicalDeviceSurfaceFormatsKHR(pd, surface, &nf, nullptr), "vkGetPhysicalDeviceSurfaceFormatsKHR");
      std::vector<VkSurfaceFormatKHR> fmts(nf);
      check(vkGetPhysicalDeviceSurfaceFormatsKHR(pd, surface, &nf, fmts.data()), "vkGetPhysicalDeviceSurfaceFormatsKHR");
      auto find = [&](VkFormat f) { return std::find_if(fmts.begin(), fmts.end(), [f](const auto& s) { return s.format == f; }); };
      // UNORM first: ImGui does no sRGB conversion. can_present() guaranteed at least one format.
      auto it = find(VK_FORMAT_B8G8R8A8_UNORM);
      if (it == fmts.end()) it = find(VK_FORMAT_R8G8B8A8_UNORM);
      const VkSurfaceFormatKHR pick = it != fmts.end() ? *it : fmts.at(0);
      color_fmt = pick.format;
      color_space = pick.colorSpace;
    }
    make_pipeline();
  }

  // Window mode: the device must offer the swapchain extension and at least one surface format.
  bool can_present(VkPhysicalDevice cand) const {
    std::uint32_t n = 0;
    vkEnumerateDeviceExtensionProperties(cand, nullptr, &n, nullptr);
    std::vector<VkExtensionProperties> exts(n);
    vkEnumerateDeviceExtensionProperties(cand, nullptr, &n, exts.data());
    bool has = false;
    for (const VkExtensionProperties& e : exts) has = has || std::string(e.extensionName) == VK_KHR_SWAPCHAIN_EXTENSION_NAME;
    std::uint32_t nf = 0;
    return has && vkGetPhysicalDeviceSurfaceFormatsKHR(cand, surface, &nf, nullptr) == VK_SUCCESS && nf > 0;
  }

  void make_pipeline() {
    std::vector<std::uint32_t> code(sizeof kSpv_geom / 4);
    std::memcpy(code.data(), kSpv_geom, code.size() * 4);
    VkShaderModuleCreateInfo smi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    smi.codeSize = code.size() * 4;
    smi.pCode = code.data();
    check(vkCreateShaderModule(dev, &smi, nullptr, &shader), "vkCreateShaderModule");

    VkPushConstantRange pcr{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(Push)};
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pli.pushConstantRangeCount = 1;
    pli.pPushConstantRanges = &pcr;
    check(vkCreatePipelineLayout(dev, &pli, nullptr, &layout), "vkCreatePipelineLayout");

    VkPipelineShaderStageCreateInfo stages[2] = {
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, shader, "vsMain", nullptr},
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, shader, "fsMain", nullptr}};
    VkVertexInputBindingDescription bind{0, 6 * sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription attrs[2] = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
                                                  {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 3 * sizeof(float)}};
    VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vi.vertexBindingDescriptionCount = 1;
    vi.pVertexBindingDescriptions = &bind;
    vi.vertexAttributeDescriptionCount = 2;
    vi.pVertexAttributeDescriptions = attrs;
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;  // planes and meshes are seen from both sides
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    ds.depthTestEnable = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS;
    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendAttachmentState cbas[2] = {cba, cba};
    VkPipelineColorBlendStateCreateInfo cbs{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cbs.attachmentCount = 2;
    cbs.pAttachments = cbas;
    const VkDynamicState dyn[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dsi{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dsi.dynamicStateCount = 2;
    dsi.pDynamicStates = dyn;
    const VkFormat colors[2] = {color_fmt, kLinFmt};
    VkPipelineRenderingCreateInfo rci{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    rci.colorAttachmentCount = 2;
    rci.pColorAttachmentFormats = colors;
    rci.depthAttachmentFormat = kDepthFmt;
    VkGraphicsPipelineCreateInfo gpi{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO, &rci};
    gpi.stageCount = 2;
    gpi.pStages = stages;
    gpi.pVertexInputState = &vi;
    gpi.pInputAssemblyState = &ia;
    gpi.pViewportState = &vp;
    gpi.pRasterizationState = &rs;
    gpi.pMultisampleState = &ms;
    gpi.pDepthStencilState = &ds;
    gpi.pColorBlendState = &cbs;
    gpi.pDynamicState = &dsi;
    gpi.layout = layout;
    check(vkCreateGraphicsPipelines(dev, nullptr, 1, &gpi, nullptr, &pipe), "vkCreateGraphicsPipelines");
  }

  void upload(const Verts& all) {
    if (vbuf) vmaDestroyBuffer(vma, vbuf, vbuf_a);
    vbuf = nullptr;
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = std::max<std::size_t>(all.size() * sizeof(float), 24);
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VmaAllocationCreateInfo ai{};
    ai.usage = VMA_MEMORY_USAGE_AUTO;
    ai.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo info;
    check(vmaCreateBuffer(vma, &bi, &ai, &vbuf, &vbuf_a, &info), "vmaCreateBuffer(vertex)");
    if (!all.empty()) std::memcpy(info.pMappedData, all.data(), all.size() * sizeof(float));
    check(vmaFlushAllocation(vma, vbuf_a, 0, VK_WHOLE_SIZE), "vmaFlushAllocation");
  }

  void destroy_target() {
    if (!tgt.w) return;
    vkDestroyImageView(dev, tgt.color_v, nullptr);
    vkDestroyImageView(dev, tgt.lin_v, nullptr);
    vkDestroyImageView(dev, tgt.depth_v, nullptr);
    vmaDestroyImage(vma, tgt.color, tgt.color_a);
    vmaDestroyImage(vma, tgt.lin, tgt.lin_a);
    vmaDestroyImage(vma, tgt.depth, tgt.depth_a);
    vmaDestroyBuffer(vma, tgt.rb_color, tgt.rb_color_a);
    vmaDestroyBuffer(vma, tgt.rb_lin, tgt.rb_lin_a);
    tgt = {};
  }

  void image(int w, int h, VkFormat fmt, VkImageUsageFlags usage, VkImageAspectFlags aspect, VkImage& img,
             VmaAllocation& alloc, VkImageView& view) {
    VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ii.imageType = VK_IMAGE_TYPE_2D;
    ii.format = fmt;
    ii.extent = {static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h), 1};
    ii.mipLevels = 1;
    ii.arrayLayers = 1;
    ii.samples = VK_SAMPLE_COUNT_1_BIT;
    ii.tiling = VK_IMAGE_TILING_OPTIMAL;
    ii.usage = usage;
    ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VmaAllocationCreateInfo ai{};
    ai.usage = VMA_MEMORY_USAGE_AUTO;
    check(vmaCreateImage(vma, &ii, &ai, &img, &alloc, nullptr), "vmaCreateImage");
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = img;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = fmt;
    vi.subresourceRange = {aspect, 0, 1, 0, 1};
    check(vkCreateImageView(dev, &vi, nullptr, &view), "vkCreateImageView");
  }

  void readback_buffer(std::size_t bytes, VkBuffer& buf, VmaAllocation& alloc, void*& ptr) {
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = bytes;
    bi.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo ai{};
    ai.usage = VMA_MEMORY_USAGE_AUTO;
    ai.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    VmaAllocationInfo info;
    check(vmaCreateBuffer(vma, &bi, &ai, &buf, &alloc, &info), "vmaCreateBuffer(readback)");
    ptr = info.pMappedData;
  }

  // readback = false (window mode): the colour target is the swapchain image, nothing is read back; only the unused
  // linear-depth output and the depth test attachment are allocated.
  void ensure_target(int w, int h, bool readback = true) {
    if (tgt.w == w && tgt.h == h) return;
    destroy_target();
    Target t;
    try {
      if (readback)
        image(w, h, color_fmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_IMAGE_ASPECT_COLOR_BIT,
              t.color, t.color_a, t.color_v);
      image(w, h, kLinFmt, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_IMAGE_ASPECT_COLOR_BIT,
            t.lin, t.lin_a, t.lin_v);
      image(w, h, kDepthFmt, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_IMAGE_ASPECT_DEPTH_BIT, t.depth, t.depth_a, t.depth_v);
      if (readback) {
        readback_buffer(static_cast<std::size_t>(w) * h * 4, t.rb_color, t.rb_color_a, t.color_ptr);
        readback_buffer(static_cast<std::size_t>(w) * h * 4, t.rb_lin, t.rb_lin_a, t.lin_ptr);
      }
    } catch (...) {
      tgt = t;  // free whatever was created
      tgt.w = w;
      destroy_target();
      throw;
    }
    t.w = w;
    t.h = h;
    tgt = t;
  }

  void transition(VkImage img, VkImageAspectFlags aspect, VkImageLayout from, VkImageLayout to, VkPipelineStageFlags2 src_stage,
                  VkAccessFlags2 src_access, VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access) {
    VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
    b.srcStageMask = src_stage;
    b.srcAccessMask = src_access;
    b.dstStageMask = dst_stage;
    b.dstAccessMask = dst_access;
    b.oldLayout = from;
    b.newLayout = to;
    b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = img;
    b.subresourceRange = {aspect, 0, 1, 0, 1};
    VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    di.imageMemoryBarrierCount = 1;
    di.pImageMemoryBarriers = &b;
    vkCmdPipelineBarrier2(cb, &di);
  }

  // Records the scene into the given attachments (the part a swapchain viewer reuses with its own colour view).
  void record(VkImageView color_v, VkImageView lin_v, VkImageView depth_v, int w, int h, const View& view,
              std::span<const GeomPose> poses) {
    VkRenderingAttachmentInfo ca[2]{{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO}, {VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO}};
    for (VkRenderingAttachmentInfo& a : ca) {
      a.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
      a.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      a.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    }
    ca[0].imageView = color_v;
    std::memcpy(ca[0].clearValue.color.float32, kClearColor, sizeof kClearColor);
    ca[1].imageView = lin_v;
    ca[1].clearValue.color.float32[0] = std::numeric_limits<float>::infinity();  // no hit
    VkRenderingAttachmentInfo da{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    da.imageView = depth_v;
    da.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    da.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    da.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    da.clearValue.depthStencil = {1.0f, 0};
    VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
    ri.renderArea = {{0, 0}, {static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h)}};
    ri.layerCount = 1;
    ri.colorAttachmentCount = 2;
    ri.pColorAttachments = ca;
    ri.pDepthAttachment = &da;
    vkCmdBeginRendering(cb, &ri);
    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
    const VkViewport vp{0, 0, static_cast<float>(w), static_cast<float>(h), 0, 1};
    const VkRect2D sc{{0, 0}, {static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h)}};
    vkCmdSetViewport(cb, 0, 1, &vp);
    vkCmdSetScissor(cb, 0, 1, &sc);
    const VkDeviceSize off = 0;
    vkCmdBindVertexBuffers(cb, 0, 1, &vbuf, &off);
    for (std::size_t i = 0; i < draws.size(); ++i) {
      if (!draws[i].visible) continue;
      const Push p = make_push(view, poses[i], draws[i].rgba);
      vkCmdPushConstants(cb, layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof p, &p);
      vkCmdDraw(cb, draws[i].count, 1, draws[i].first, 0);
    }
    vkCmdEndRendering(cb);
  }

  Image draw(std::span<const GeomPose> poses, const Camera& cam, int w, int h) {
    if (w < 1 || h < 1 || w > 16384 || h > 16384) throw std::invalid_argument("render size out of range");
    if (poses.size() != draws.size())
      throw std::invalid_argument("draw: " + std::to_string(poses.size()) + " poses for a scene of " + std::to_string(draws.size()) + " geoms");
    const double d2 = dot(V3{cam.lookat[0], cam.lookat[1], cam.lookat[2]} - V3{cam.pos[0], cam.pos[1], cam.pos[2]},
                          V3{cam.lookat[0], cam.lookat[1], cam.lookat[2]} - V3{cam.pos[0], cam.pos[1], cam.pos[2]});
    if (!(d2 > 0) || !(cam.fovy > 0 && cam.fovy < 180)) throw std::invalid_argument("bad camera");
    ensure_target(w, h);

    check(vkResetCommandBuffer(cb, 0), "vkResetCommandBuffer");
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(cb, &bi), "vkBeginCommandBuffer");
    constexpr auto kColorOut = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    constexpr auto kDepthTest = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    constexpr auto kColorW = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    for (VkImage img : {tgt.color, tgt.lin})
      transition(img, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                 VK_PIPELINE_STAGE_2_NONE, 0, kColorOut, kColorW);
    transition(tgt.depth, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
               VK_PIPELINE_STAGE_2_NONE, 0, kDepthTest,
               VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

    record(tgt.color_v, tgt.lin_v, tgt.depth_v, w, h, make_view(cam, w, h), poses);

    for (VkImage img : {tgt.color, tgt.lin})
      transition(img, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                 kColorOut, kColorW, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
    VkBufferImageCopy region{};
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.imageExtent = {static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h), 1};
    vkCmdCopyImageToBuffer(cb, tgt.color, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, tgt.rb_color, 1, &region);
    vkCmdCopyImageToBuffer(cb, tgt.lin, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, tgt.rb_lin, 1, &region);
    VkMemoryBarrier2 mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
    mb.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
    mb.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    mb.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
    mb.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
    VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    di.memoryBarrierCount = 1;
    di.pMemoryBarriers = &mb;
    vkCmdPipelineBarrier2(cb, &di);
    check(vkEndCommandBuffer(cb), "vkEndCommandBuffer");

    VkCommandBufferSubmitInfo cbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
    cbi.commandBuffer = cb;
    VkSubmitInfo2 si{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
    si.commandBufferInfoCount = 1;
    si.pCommandBufferInfos = &cbi;
    check(vkResetFences(dev, 1, &fence), "vkResetFences");
    check(vkQueueSubmit2(queue, 1, &si, fence), "vkQueueSubmit2");
    check(vkWaitForFences(dev, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");
    check(vmaInvalidateAllocation(vma, tgt.rb_color_a, 0, VK_WHOLE_SIZE), "vmaInvalidateAllocation");
    check(vmaInvalidateAllocation(vma, tgt.rb_lin_a, 0, VK_WHOLE_SIZE), "vmaInvalidateAllocation");

    Image out;
    out.width = w;
    out.height = h;
    const std::size_t n = static_cast<std::size_t>(w) * h;
    out.rgba.resize(n * 4);
    out.depth.resize(n);
    std::memcpy(out.rgba.data(), tgt.color_ptr, n * 4);
    std::memcpy(out.depth.data(), tgt.lin_ptr, n * 4);
    return out;
  }

  void set_scene(std::span<const GeomVisual> visuals) {
    Verts all;
    std::vector<Draw> next(visuals.size());
    for (std::size_t i = 0; i < visuals.size(); ++i) {
      const GeomVisual& g = visuals[i];
      Draw& d = next[i];
      Verts v = tessellate(g);
      d.first = static_cast<std::uint32_t>(all.size() / 6);
      d.count = static_cast<std::uint32_t>(v.size() / 6);
      std::memcpy(d.rgba, g.rgba, sizeof d.rgba);
      d.visible = d.count > 0 && g.group < 3 && g.rgba[3] > 0;  // group >= 3: MuJoCo's hidden collision geoms
      all.insert(all.end(), v.begin(), v.end());
    }
    upload(all);
    draws = std::move(next);
  }

  // ---- window mode ----------------------------------------------------------------------------------------------------
  void destroy_swapchain_views() {
    for (VkImageView v : sc_views) vkDestroyImageView(dev, v, nullptr);
    for (VkSemaphore s : sc_done) vkDestroySemaphore(dev, s, nullptr);
    sc_views.clear();
    sc_done.clear();
    sc_images.clear();
  }

  // (Re)builds the swapchain for a (w, h) framebuffer. false = the surface has zero extent (minimised).
  bool make_swapchain(int w, int h) {
    vkDeviceWaitIdle(dev);
    VkSurfaceCapabilitiesKHR caps;
    check(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(pd, surface, &caps), "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");
    VkExtent2D ext = caps.currentExtent;
    if (ext.width == 0xFFFFFFFFu)  // the surface leaves it to us
      ext = {std::clamp<std::uint32_t>(static_cast<std::uint32_t>(w), caps.minImageExtent.width, caps.maxImageExtent.width),
             std::clamp<std::uint32_t>(static_cast<std::uint32_t>(h), caps.minImageExtent.height, caps.maxImageExtent.height)};
    if (ext.width == 0 || ext.height == 0) return false;
    VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    ci.surface = surface;
    ci.minImageCount = std::max(2u, caps.minImageCount);
    if (caps.maxImageCount) ci.minImageCount = std::min(ci.minImageCount, caps.maxImageCount);
    ci.imageFormat = color_fmt;
    ci.imageColorSpace = color_space;
    ci.imageExtent = ext;
    ci.imageArrayLayers = 1;
    ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.preTransform = caps.currentTransform;
    ci.compositeAlpha = (caps.supportedCompositeAlpha & VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR) ? VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR
                                                                                           : VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR;
    ci.presentMode = VK_PRESENT_MODE_FIFO_KHR;  // the one mode every driver has
    ci.clipped = VK_TRUE;
    ci.oldSwapchain = swapchain;
    VkSwapchainKHR next = nullptr;
    check(vkCreateSwapchainKHR(dev, &ci, nullptr, &next), "vkCreateSwapchainKHR");
    destroy_swapchain_views();
    if (swapchain) vkDestroySwapchainKHR(dev, swapchain, nullptr);
    swapchain = next;
    sc_extent = ext;
    std::uint32_t n = 0;
    check(vkGetSwapchainImagesKHR(dev, swapchain, &n, nullptr), "vkGetSwapchainImagesKHR");
    sc_images.resize(n);
    check(vkGetSwapchainImagesKHR(dev, swapchain, &n, sc_images.data()), "vkGetSwapchainImagesKHR");
    for (VkImage img : sc_images) {
      VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      vi.image = img;
      vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
      vi.format = color_fmt;
      vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
      sc_views.emplace_back();
      check(vkCreateImageView(dev, &vi, nullptr, &sc_views.back()), "vkCreateImageView");
      VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
      sc_done.emplace_back();
      check(vkCreateSemaphore(dev, &si, nullptr, &sc_done.back()), "vkCreateSemaphore");
    }
    return true;
  }

  bool window_frame(std::span<const GeomPose> poses, const Camera& cam, int w, int h,
                    const std::function<void(VkCommandBuffer)>& overlay) {
    if (poses.size() != draws.size())
      throw std::invalid_argument("frame: " + std::to_string(poses.size()) + " poses for a scene of " + std::to_string(draws.size()) + " geoms");
    const V3 look{cam.lookat[0] - cam.pos[0], cam.lookat[1] - cam.pos[1], cam.lookat[2] - cam.pos[2]};
    if (!(dot(look, look) > 0) || !(cam.fovy > 0 && cam.fovy < 180)) throw std::invalid_argument("bad camera");
    if (w < 1 || h < 1) return false;
    if (!swapchain || sc_stale || w != req_w || h != req_h) {
      sc_stale = false;
      req_w = w;
      req_h = h;
      if (!make_swapchain(w, h)) return false;
    }
    std::uint32_t idx = 0;
    const VkResult ar = vkAcquireNextImageKHR(dev, swapchain, UINT64_MAX, acquire_sem, nullptr, &idx);
    if (ar == VK_ERROR_OUT_OF_DATE_KHR) {
      sc_stale = true;
      return false;
    }
    if (ar != VK_SUBOPTIMAL_KHR) check(ar, "vkAcquireNextImageKHR");
    const int fw = static_cast<int>(sc_extent.width), fh = static_cast<int>(sc_extent.height);
    ensure_target(fw, fh, /*readback=*/false);

    check(vkResetFences(dev, 1, &fence), "vkResetFences");
    check(vkResetCommandBuffer(cb, 0), "vkResetCommandBuffer");
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    check(vkBeginCommandBuffer(cb, &bi), "vkBeginCommandBuffer");
    constexpr auto kColorOut = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    constexpr auto kDepthTest = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    constexpr auto kColorW = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    for (VkImage img : {sc_images[idx], tgt.lin})
      transition(img, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                 VK_PIPELINE_STAGE_2_NONE, 0, kColorOut, kColorW);
    transition(tgt.depth, VK_IMAGE_ASPECT_DEPTH_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
               VK_PIPELINE_STAGE_2_NONE, 0, kDepthTest,
               VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

    record(sc_views[idx], tgt.lin_v, tgt.depth_v, fw, fh, make_view(cam, fw, fh), poses);

    if (overlay) {  // second pass on the same colour image: scene writes -> overlay blend reads/writes
      VkMemoryBarrier2 mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
      mb.srcStageMask = kColorOut;
      mb.srcAccessMask = kColorW;
      mb.dstStageMask = kColorOut;
      mb.dstAccessMask = kColorW | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT;
      VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
      di.memoryBarrierCount = 1;
      di.pMemoryBarriers = &mb;
      vkCmdPipelineBarrier2(cb, &di);
      VkRenderingAttachmentInfo ca{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
      ca.imageView = sc_views[idx];
      ca.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
      ca.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
      ca.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
      VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
      ri.renderArea = {{0, 0}, sc_extent};
      ri.layerCount = 1;
      ri.colorAttachmentCount = 1;
      ri.pColorAttachments = &ca;
      vkCmdBeginRendering(cb, &ri);
      overlay(cb);
      vkCmdEndRendering(cb);
    }
    transition(sc_images[idx], VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
               kColorOut, kColorW, VK_PIPELINE_STAGE_2_NONE, 0);
    check(vkEndCommandBuffer(cb), "vkEndCommandBuffer");

    VkSemaphoreSubmitInfo wait{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
    wait.semaphore = acquire_sem;
    wait.stageMask = kColorOut;
    VkSemaphoreSubmitInfo signal{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
    signal.semaphore = sc_done[idx];
    signal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    VkCommandBufferSubmitInfo cbi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
    cbi.commandBuffer = cb;
    VkSubmitInfo2 si{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
    si.waitSemaphoreInfoCount = 1;
    si.pWaitSemaphoreInfos = &wait;
    si.commandBufferInfoCount = 1;
    si.pCommandBufferInfos = &cbi;
    si.signalSemaphoreInfoCount = 1;
    si.pSignalSemaphoreInfos = &signal;
    check(vkQueueSubmit2(queue, 1, &si, fence), "vkQueueSubmit2");

    VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores = &sc_done[idx];
    pi.swapchainCount = 1;
    pi.pSwapchains = &swapchain;
    pi.pImageIndices = &idx;
    const VkResult pr = vkQueuePresentKHR(queue, &pi);
    if (pr == VK_ERROR_OUT_OF_DATE_KHR || pr == VK_SUBOPTIMAL_KHR) sc_stale = true;  // rebuilt on the next frame
    else check(pr, "vkQueuePresentKHR");
    check(vkWaitForFences(dev, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences");  // one frame in flight
    return true;
  }

  void destroy() noexcept {
    if (dev) vkDeviceWaitIdle(dev);
    if (dev) {
      destroy_swapchain_views();
      if (swapchain) vkDestroySwapchainKHR(dev, swapchain, nullptr);
      if (acquire_sem) vkDestroySemaphore(dev, acquire_sem, nullptr);
    }
    swapchain = nullptr;
    acquire_sem = nullptr;
    if (vma) {
      destroy_target();
      if (vbuf) vmaDestroyBuffer(vma, vbuf, vbuf_a);
    }
    if (dev) {
      if (pipe) vkDestroyPipeline(dev, pipe, nullptr);
      if (layout) vkDestroyPipelineLayout(dev, layout, nullptr);
      if (shader) vkDestroyShaderModule(dev, shader, nullptr);
      if (fence) vkDestroyFence(dev, fence, nullptr);
      if (pool) vkDestroyCommandPool(dev, pool, nullptr);
    }
    if (vma) vmaDestroyAllocator(vma);
    if (dev) vkDestroyDevice(dev, nullptr);
    if (surface) vkDestroySurfaceKHR(inst, surface, nullptr);  // the window must still exist
    surface = nullptr;
    if (inst) vkDestroyInstance(inst, nullptr);
    vbuf = nullptr;
    pipe = nullptr;
    layout = nullptr;
    shader = nullptr;
    fence = nullptr;
    pool = nullptr;
    vma = nullptr;
    dev = nullptr;
    inst = nullptr;
  }
};

Renderer::Renderer() : p_(std::make_unique<Impl>()) {}
Renderer::~Renderer() = default;
const std::string& Renderer::device_name() const { return p_->name; }

void Renderer::set_scene(std::span<const GeomVisual> visuals) {
  p_->set_scene(visuals);
  scene_hash_ = hash_visuals(visuals);
}

Image Renderer::draw(std::span<const GeomPose> poses, const Camera& cam, int width, int height) {
  return p_->draw(poses, cam, width, height);
}

// ponytail: visuals() is rebuilt and hashed per render (mesh models: a copy of every mesh); cache a model generation
// counter in the Solver seam if that ever shows up in a profile.
Image Renderer::render(const Solver& s, const Camera& cam, int width, int height) {
  const std::vector<GeomVisual> v = s.visuals();
  if (hash_visuals(v) != scene_hash_ || p_->draws.size() != v.size()) set_scene(v);
  const std::vector<GeomPose> poses = s.poses();
  return p_->draw(poses, cam, width, height);
}

WindowRenderer::WindowRenderer(const Surface& surface) : p_(std::make_unique<Renderer::Impl>(&surface)) {}
WindowRenderer::~WindowRenderer() = default;
const std::string& WindowRenderer::device_name() const { return p_->name; }
WindowRenderer::Handles WindowRenderer::handles() const {
  return {p_->inst, p_->pd, p_->dev, p_->queue, p_->qfam, p_->color_fmt};
}
void WindowRenderer::set_scene(std::span<const GeomVisual> visuals) { p_->set_scene(visuals); }
bool WindowRenderer::frame(std::span<const GeomPose> poses, const Camera& cam, int width, int height,
                           const std::function<void(VkCommandBuffer)>& overlay) {
  return p_->window_frame(poses, cam, width, height, overlay);
}

RenderHook make_render_hook() {
  return [](const Solver& s, const Camera& cam, int w, int h) {
    static std::mutex mu;
    static Renderer* r = nullptr;  // leaked on purpose: tearing a GPU driver down during static destruction is fragile
    std::lock_guard<std::mutex> lock(mu);
    if (!r) r = new Renderer();  // ttsr-ignore: cpp/no-raw-new  (deliberately never freed, see above)
    return r->render(s, cam, w, h);
  };
}
