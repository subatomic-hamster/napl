#pragma once
// Headless Vulkan 1.3 renderer (dynamic rendering, no window system, no swapchain): Lambert shading with one
// directional light, offscreen RGBA8 colour + linear-depth (R32F) + D32 depth-test attachments, CPU readback.
//
// Precision: poses arrive in double. Per geom the renderer subtracts the camera position IN DOUBLE, builds the
// clip-from-geom matrix in double relative to the camera, and only then rounds to float -- so a scene (and camera)
// 1e5 m from the origin renders like one at the origin.
//
// Swapchain reuse (M10): everything that draws is Impl::record(), which takes attachment views and a command buffer
// and knows nothing about readback; a viewer adds a swapchain image view as colour target and skips the readback.
#include <memory>
#include <span>
#include <string>

#include "core/render.h"
#include "core/solver.h"

class WindowRenderer;  // window_renderer.h: shares Impl (the draw path) with the headless renderer

class Renderer {
  friend class WindowRenderer;

 public:
  // Picks the first Vulkan 1.3 device with dynamic rendering (the loader honours VK_DRIVER_FILES). Throws
  // std::runtime_error with a plain-language reason when there is no loader, driver or suitable device.
  Renderer();
  ~Renderer();
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  const std::string& device_name() const;

  // Tessellates (primitives, once) and uploads the scene; draw() then needs one pose per visual.
  void set_scene(std::span<const GeomVisual> visuals);
  // Throws std::invalid_argument for bad sizes / a pose count that differs from the scene.
  Image draw(std::span<const GeomPose> poses, const Camera& cam, int width, int height);
  // Convenience: re-uploads the scene only if solver.visuals() changed since the last call.
  Image render(const Solver& solver, const Camera& cam, int width, int height);

 private:
  struct Impl;
  std::unique_ptr<Impl> p_;
  std::uint64_t scene_hash_ = 0;
};

// Hook for World::set_render_hook: one process-wide Renderer, created on the first render (so merely installing the
// hook never touches Vulkan), serialised by a mutex. A failed creation throws every time until a device exists.
RenderHook make_render_hook();
