#pragma once
// The viewer's renderer: same scene drawing as Renderer (Impl::record, camera-relative double precision) but into a
// swapchain image, with a caller-supplied overlay (Dear ImGui) drawn on top in a second dynamic-rendering pass. One
// frame in flight, FIFO present. GLFW (the viewer) supplies the instance extensions and the surface; this file needs
// no window-system header.
#include <volk.h>

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "render/renderer.h"

class WindowRenderer {
 public:
  struct Surface {
    std::vector<const char*> instance_extensions;                // e.g. VK_KHR_surface + the platform surface extension
    std::function<VkSurfaceKHR(VkInstance)> create;              // throws std::runtime_error with the reason on failure
  };
  // What an ImGui Vulkan backend needs.
  struct Handles {
    VkInstance instance;
    VkPhysicalDevice physical_device;
    VkDevice device;
    VkQueue queue;
    std::uint32_t queue_family;
    VkFormat color_format;  // swapchain format (UNORM preferred: ImGui draws without an sRGB conversion)
  };

  // Picks the first Vulkan 1.3 device (dynamic rendering) that can present to the surface. Throws
  // std::runtime_error with a plain-language reason when there is none (no driver, no window-system support, ...).
  explicit WindowRenderer(const Surface& surface);
  ~WindowRenderer();
  WindowRenderer(const WindowRenderer&) = delete;
  WindowRenderer& operator=(const WindowRenderer&) = delete;

  const std::string& device_name() const;
  Handles handles() const;
  void set_scene(std::span<const GeomVisual> visuals);

  // Draws the scene, then overlay(command_buffer) inside a second rendering pass on the same colour target, and
  // presents. (width, height) = the window's framebuffer size; 0 = minimised. Returns false if nothing was presented
  // (minimised, or the swapchain was out of date and is rebuilt on the next call). Throws std::invalid_argument for
  // a pose count that differs from the scene or a bad camera.
  bool frame(std::span<const GeomPose> poses, const Camera& cam, int width, int height,
             const std::function<void(VkCommandBuffer)>& overlay);

 private:
  std::unique_ptr<Renderer::Impl> p_;
};
