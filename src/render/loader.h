#pragma once
#include <volk.h>

// Apple: exact bundled library, retained until process exit. Linux: system Vulkan loader.
// Explicit paths are for loader failure fixtures; a successful process uses one library only.
PFN_vkGetInstanceProcAddr initialize_vulkan(const char* library_path = nullptr);
