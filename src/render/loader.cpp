#include "render/loader.h"

#include <mutex>
#include <stdexcept>
#include <string>
#ifdef __APPLE__
#include <dlfcn.h>
#include <filesystem>
#endif

PFN_vkGetInstanceProcAddr initialize_vulkan(const char* library_path) {
#ifdef __APPLE__
  static std::mutex mu;
  static void* handle = nullptr;  // Intentionally retained: GLFW/volk may outlive renderer instances.
  static PFN_vkGetInstanceProcAddr proc = nullptr;
  static std::string loaded;
  std::lock_guard<std::mutex> lock(mu);
  std::string path;
  if (library_path) {
    path = library_path;
  } else {
    Dl_info info{};
    if (!dladdr(reinterpret_cast<const void*>(&initialize_vulkan), &info) || !info.dli_fname)
      throw std::runtime_error("Vulkan: cannot locate containing Mach-O for bundled MoltenVK");
    const auto dir = std::filesystem::path(info.dli_fname).parent_path();
    auto file = dir / "libMoltenVK.dylib";
    if (!std::filesystem::exists(file)) file = dir.parent_path() / "libMoltenVK.dylib";
    path = file.string();
  }
  if (handle) {
    if (loaded != path) throw std::runtime_error("Vulkan: cannot replace the active MoltenVK loader");
    return proc;
  }
  void* candidate = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!candidate) throw std::runtime_error("Vulkan: cannot load bundled MoltenVK '" + path + "': " + dlerror());
  auto get_proc = reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(candidate, "vkGetInstanceProcAddr"));
  if (!get_proc) {
    dlclose(candidate);
    throw std::runtime_error("Vulkan: library '" + path + "' lacks vkGetInstanceProcAddr");
  }
  volkInitializeCustom(get_proc);
  handle = candidate;
  proc = get_proc;
  loaded = path;
  return proc;
#else
  (void)library_path;
  if (volkInitialize() != VK_SUCCESS) throw std::runtime_error("no Vulkan loader (libvulkan) found on this machine");
  return vkGetInstanceProcAddr;
#endif
}
