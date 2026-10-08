cmake_minimum_required(VERSION 3.25)
project(napl LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Determinism (plan Risk 1): no FMA contraction, no fast-math. Set before deps so they inherit it.
if(NOT MSVC)
  add_compile_options(-ffp-contract=off)
endif()

# napl-view / Sim.show(): GLFW + Dear ImGui. Linux (X11) only until the Windows/macOS GLFW build is written and tested.
if(UNIX AND NOT APPLE)
  option(NAPL_VIEWER "Build the Vulkan viewer (napl-view)" ON)
else()
  option(NAPL_VIEWER "Build the Vulkan viewer (napl-view)" OFF)
endif()

include(cmake/deps.cmake)

enable_testing()
include(GoogleTest)

# napl_schema: napl.fbs -> C++ header, embedded .bfbs header (runtime reflection/JSON), JSON Schema (MCP, M8).
# Re-runs whenever schema/napl.fbs or flatc changes.
set(NAPL_GEN_DIR ${CMAKE_BINARY_DIR}/generated)
add_custom_command(
  OUTPUT ${NAPL_GEN_DIR}/napl_generated.h ${NAPL_GEN_DIR}/napl_bfbs_generated.h ${NAPL_GEN_DIR}/napl.schema.json
  COMMAND flatc --cpp --scoped-enums --bfbs-gen-embed --jsonschema -o ${NAPL_GEN_DIR} ${CMAKE_SOURCE_DIR}/schema/napl.fbs
  DEPENDS flatc ${CMAKE_SOURCE_DIR}/schema/napl.fbs
  COMMENT "flatc: napl.fbs")
# devices.fbs (N2): device config for MujocoSolver, separate from napl.fbs (whose hash is in replay logs).
add_custom_command(
  OUTPUT ${NAPL_GEN_DIR}/devices_generated.h ${NAPL_GEN_DIR}/devices_bfbs_generated.h
  COMMAND flatc --cpp --scoped-enums --bfbs-gen-embed -o ${NAPL_GEN_DIR} ${CMAKE_SOURCE_DIR}/schema/devices.fbs
  DEPENDS flatc ${CMAKE_SOURCE_DIR}/schema/devices.fbs
  COMMENT "flatc: devices.fbs")
add_custom_target(napl_schema_gen DEPENDS ${NAPL_GEN_DIR}/napl_generated.h ${NAPL_GEN_DIR}/devices_generated.h)
add_library(napl_schema INTERFACE)
add_dependencies(napl_schema napl_schema_gen)
target_include_directories(napl_schema INTERFACE ${NAPL_GEN_DIR})
target_link_libraries(napl_schema INTERFACE flatbuffers)

# Seam enforced by the build graph: napl_core gets no mujoco include dirs, so including mujoco.h fails.
find_package(Threads REQUIRED)
add_library(napl_core STATIC src/core/world.cpp src/core/bus.cpp src/core/replay.cpp src/core/host.cpp
  src/core/monitor.cpp src/core/shm.cpp)
target_include_directories(napl_core PUBLIC src include)  # include/: napl/shm_client.hpp, the dependency-free shm client
target_link_libraries(napl_core PUBLIC napl_schema Threads::Threads)

add_library(napl_mujoco STATIC src/mujoco/mujoco_solver.cpp src/mujoco/devices/devices.cpp
  src/mujoco/devices/electrical.cpp src/mujoco/devices/aero.cpp src/mujoco/devices/servo.cpp
  src/mujoco/devices/sensors.cpp src/mujoco/devices/launcher.cpp)
target_link_libraries(napl_mujoco PUBLIC napl_core mujoco)

# Static libs end up inside the _napl shared module: they need -fPIC.
set_target_properties(napl_core napl_mujoco flatbuffers lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)

# napl_render: headless Vulkan 1.3 renderer (no window system). napl_core never links it: the app installs it into a
# World as a RenderHook (src/core/render.h). volk loads libvulkan at run time, so binaries start without a GPU.
include(cmake/shaders.cmake)
add_library(napl_volk STATIC ${volk_SOURCE_DIR}/volk.c)
target_include_directories(napl_volk PUBLIC ${volk_SOURCE_DIR} ${vulkan_headers_SOURCE_DIR}/include)
target_compile_definitions(napl_volk PUBLIC VK_NO_PROTOTYPES)
target_link_libraries(napl_volk PUBLIC ${CMAKE_DL_LIBS})
napl_embed_shader(geom)
add_library(napl_render STATIC src/render/renderer.cpp ${NAPL_GEN_DIR}/shaders/geom_spv.h)
target_include_directories(napl_render PUBLIC src PRIVATE ${vma_SOURCE_DIR}/include ${NAPL_GEN_DIR}/shaders)
target_link_libraries(napl_render PUBLIC napl_core PRIVATE napl_volk)
set_target_properties(napl_render napl_volk PROPERTIES POSITION_INDEPENDENT_CODE ON)  # linked into _napl.so

# napl_viewer (M10): GLFW window + Vulkan swapchain + Dear ImGui over a Host. napl-view is its command line; _napl links it
# for Sim.show(). GLFW and ImGui are built from fetched sources (no find_package): GLFW dlopen()s libX11 & co, so only the
# X11 headers (fetched, cmake/deps.cmake) are needed to compile it.
if(NAPL_VIEWER)
  set(_x11_gen ${CMAKE_BINARY_DIR}/x11-gen)  # the one generated X11 header: Xcursor.h from its (unsubstituted) .in
  configure_file(${libxcursor_SOURCE_DIR}/include/X11/Xcursor/Xcursor.h.in ${_x11_gen}/X11/Xcursor/Xcursor.h COPYONLY)
  set(_glfw_src ${glfw_SOURCE_DIR}/src)
  add_library(napl_glfw STATIC
    ${_glfw_src}/context.c ${_glfw_src}/init.c ${_glfw_src}/input.c ${_glfw_src}/monitor.c ${_glfw_src}/platform.c
    ${_glfw_src}/vulkan.c ${_glfw_src}/window.c ${_glfw_src}/egl_context.c ${_glfw_src}/osmesa_context.c
    ${_glfw_src}/null_init.c ${_glfw_src}/null_monitor.c ${_glfw_src}/null_window.c ${_glfw_src}/null_joystick.c
    ${_glfw_src}/posix_time.c ${_glfw_src}/posix_thread.c ${_glfw_src}/posix_module.c ${_glfw_src}/posix_poll.c
    ${_glfw_src}/linux_joystick.c ${_glfw_src}/x11_init.c ${_glfw_src}/x11_monitor.c ${_glfw_src}/x11_window.c
    ${_glfw_src}/xkb_unicode.c ${_glfw_src}/glx_context.c)
  target_compile_definitions(napl_glfw PRIVATE _GLFW_X11 INTERFACE GLFW_INCLUDE_NONE)  # NONE (for users, not GLFW itself): no GL headers; Vulkan comes from volk
  target_include_directories(napl_glfw PUBLIC ${glfw_SOURCE_DIR}/include)
  target_include_directories(napl_glfw SYSTEM PRIVATE ${_x11_gen}
    ${xorgproto_SOURCE_DIR}/include ${libx11_SOURCE_DIR}/include ${libxrandr_SOURCE_DIR}/include
    ${libxrender_SOURCE_DIR}/include ${libxi_SOURCE_DIR}/include ${libxinerama_SOURCE_DIR}/include
    ${libxext_SOURCE_DIR}/include ${libxfixes_SOURCE_DIR}/include)
  target_link_libraries(napl_glfw PUBLIC ${CMAKE_DL_LIBS} Threads::Threads m)

  set(_imgui_src ${imgui_SOURCE_DIR})
  add_library(napl_imgui STATIC ${_imgui_src}/imgui.cpp ${_imgui_src}/imgui_draw.cpp ${_imgui_src}/imgui_tables.cpp
    ${_imgui_src}/imgui_widgets.cpp ${_imgui_src}/backends/imgui_impl_glfw.cpp ${_imgui_src}/backends/imgui_impl_vulkan.cpp)
  target_include_directories(napl_imgui SYSTEM PUBLIC ${_imgui_src} ${_imgui_src}/backends)
  target_compile_definitions(napl_imgui PUBLIC IMGUI_IMPL_VULKAN_USE_VOLK)
  target_link_libraries(napl_imgui PUBLIC napl_volk napl_glfw)

  # UI font: Roboto-Medium.ttf (Apache-2.0, from the pinned imgui checkout) as a byte array, rewritten only when it changes.
  file(READ ${_imgui_src}/misc/fonts/Roboto-Medium.ttf _font_hex HEX)
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _font_hex "${_font_hex}")
  file(READ ${_imgui_src}/misc/fonts/Cousine-Regular.ttf _mono_hex HEX)  # Cousine (Apache-2.0): the log / code view
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _mono_hex "${_mono_hex}")
  file(CONFIGURE OUTPUT ${NAPL_GEN_DIR}/ui_font.cpp @ONLY CONTENT
    "#include <cstddef>\nextern unsigned char kUiFont[] = {@_font_hex@};\nextern const std::size_t kUiFontSize = sizeof kUiFont;\nextern unsigned char kUiMono[] = {@_mono_hex@};\nextern const std::size_t kUiMonoSize = sizeof kUiMono;\n")
  add_library(napl_viewer STATIC src/viewer/viewer.cpp src/viewer/theme.cpp src/viewer/build3d.cpp src/viewer/library_ui.cpp
    ${NAPL_GEN_DIR}/ui_font.cpp)
  target_include_directories(napl_viewer PRIVATE ${stb_image_SOURCE_DIR})
  # LOOK's 2D wiring mode (src/viewer/wire2d.cpp, `void wire2d_draw(const Wire2dSlot&)`, see viewer/wire2d_slot.h) is linked in
  # when the file exists; without it the WIRING tab shows a placeholder.
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/viewer/wire2d.cpp)
    target_sources(napl_viewer PRIVATE src/viewer/wire2d.cpp)
    target_compile_definitions(napl_viewer PRIVATE NAPL_HAS_WIRE2D)
  endif()
  target_link_libraries(napl_viewer PUBLIC napl_render napl_core PRIVATE napl_imgui napl_glfw napl_volk)
  set_target_properties(napl_glfw napl_imgui napl_viewer PROPERTIES POSITION_INDEPENDENT_CODE ON)  # linked into _napl.so

  add_executable(napl_view src/viewer/main.cpp)
  set_target_properties(napl_view PROPERTIES OUTPUT_NAME napl-view)
  target_link_libraries(napl_view PRIVATE napl_viewer napl_mujoco mujoco)
endif()

add_executable(napl_cli src/cli/main.cpp)
set_target_properties(napl_cli PROPERTIES OUTPUT_NAME napl-replay)  # the Python CLI owns `napl`
target_link_libraries(napl_cli PRIVATE napl_core napl_mujoco mujoco)

add_executable(napl_shm_serve src/cli/shm_serve.cpp)  # shm lockstep server (M11); the client is include/napl/shm_client.hpp
set_target_properties(napl_shm_serve PROPERTIES OUTPUT_NAME napl-shm-serve)
target_link_libraries(napl_shm_serve PRIVATE napl_core napl_mujoco mujoco)

# napl-px4-bridge (N5): PX4 SITL lockstep bridge. Header-only deps (mavlink: SYSTEM, its generated code is not ours to lint).
add_library(napl_px4_deps INTERFACE)
target_include_directories(napl_px4_deps SYSTEM INTERFACE ${mavlink_SOURCE_DIR} ${tomlplusplus_SOURCE_DIR}/include)
add_executable(napl_px4_bridge src/px4/main.cpp)
set_target_properties(napl_px4_bridge PROPERTIES OUTPUT_NAME napl-px4-bridge)
target_link_libraries(napl_px4_bridge PRIVATE napl_core napl_mujoco mujoco napl_px4_deps)

# The shm test's child process: header-only client only, so it links nothing of napl (proves the header stands alone).
add_executable(napl_shm_test_client tests/cpp/shm_test_client.cpp)
target_include_directories(napl_shm_test_client PRIVATE include tests/cpp)

add_executable(napl_tests tests/cpp/test_smoke.cpp tests/cpp/test_determinism.cpp tests/cpp/test_bus.cpp
  tests/cpp/test_replay.cpp tests/cpp/test_host.cpp tests/cpp/test_shm.cpp tests/cpp/test_devices.cpp
  tests/cpp/test_launcher.cpp tests/cpp/test_px4.cpp)
add_dependencies(napl_tests napl_cli napl_shm_test_client napl_px4_bridge)  # test_replay / test_shm / test_px4 run these executables
target_include_directories(napl_tests PRIVATE tests/cpp)
target_link_libraries(napl_tests PRIVATE napl_core napl_mujoco mujoco napl_px4_deps GTest::gtest_main)
target_compile_definitions(napl_tests PRIVATE
  NAPL_CLI_EXE="$<TARGET_FILE:napl_cli>"
  NAPL_PX4_BRIDGE_EXE="$<TARGET_FILE:napl_px4_bridge>"
  NAPL_SHM_CLIENT_EXE="$<TARGET_FILE:napl_shm_test_client>"
  NAPL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
  NAPL_HUMANOID_XML="${mujoco_SOURCE_DIR}/model/humanoid/humanoid.xml")
if(WIN32)  # no rpath on Windows: put mujoco.dll next to the test exe
  add_custom_command(TARGET napl_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:mujoco> $<TARGET_FILE_DIR:napl_tests>)
  add_custom_command(TARGET napl_cli POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:mujoco> $<TARGET_FILE_DIR:napl_cli>)
endif()
gtest_discover_tests(napl_tests PROPERTIES LABELS cpu)

# GPU tests: their own executable and ctest label, so `ctest --preset dev` (cpu) never touches a GPU.
# Run them with `ctest --preset gpu` (see CMakePresets.json); pick the driver with VK_DRIVER_FILES.
add_executable(napl_render_tests tests/cpp/test_render.cpp)
target_link_libraries(napl_render_tests PRIVATE napl_render napl_mujoco mujoco lodepng GTest::gtest_main)
target_compile_definitions(napl_render_tests PRIVATE
  NAPL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
  NAPL_RENDER_OUT_DIR="${CMAKE_BINARY_DIR}/render-out")
if(WIN32)
  add_custom_command(TARGET napl_render_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:mujoco> $<TARGET_FILE_DIR:napl_render_tests>)
endif()
gtest_discover_tests(napl_render_tests PROPERTIES LABELS gpu)

if(NAPL_VIEWER)  # needs a window system too: run under a display or `xvfb-run -a ctest --preset gpu`
  add_executable(napl_viewer_tests tests/cpp/test_viewer.cpp)
  add_dependencies(napl_viewer_tests napl_view napl_cli)  # the tests run the napl-view / napl-replay executables
  target_link_libraries(napl_viewer_tests PRIVATE napl_viewer napl_mujoco mujoco GTest::gtest_main)
  target_compile_definitions(napl_viewer_tests PRIVATE
    NAPL_VIEW_EXE="$<TARGET_FILE:napl_view>"
    NAPL_CLI_EXE="$<TARGET_FILE:napl_cli>"
    NAPL_VIEWER_OUT_DIR="${CMAKE_BINARY_DIR}/viewer-out"
    NAPL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
  gtest_discover_tests(napl_viewer_tests PROPERTIES LABELS gpu)
endif()

# _napl: nanobind extension (CPython >= 3.12 stable ABI). Off by default; scikit-build-core turns it on
# (pyproject.toml) and shares this build tree, so the dependency sources/objects exist exactly once.
option(NAPL_PYTHON "Build the _napl Python extension" OFF)
if(NAPL_PYTHON)
  find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
  FetchContent_MakeAvailable(nanobind)
  # The flatc JSON Schema (MCP tool input schemas) is embedded in the module as `_napl.schema_json`.
  add_custom_command(
    OUTPUT ${NAPL_GEN_DIR}/napl_schema_json.h
    COMMAND ${CMAKE_COMMAND} -DIN=${NAPL_GEN_DIR}/napl.schema.json -DOUT=${NAPL_GEN_DIR}/napl_schema_json.h
            -DNAME=kSchemaJson -P ${CMAKE_SOURCE_DIR}/cmake/embed_text.cmake
    DEPENDS ${NAPL_GEN_DIR}/napl.schema.json ${CMAKE_SOURCE_DIR}/cmake/embed_text.cmake)
  nanobind_add_module(_napl STABLE_ABI src/python/bindings.cpp ${NAPL_GEN_DIR}/napl_schema_json.h)
  target_link_libraries(_napl PRIVATE napl_core napl_mujoco mujoco napl_render lodepng)
  if(NAPL_VIEWER)
    target_link_libraries(_napl PRIVATE napl_viewer)
    target_compile_definitions(_napl PRIVATE NAPL_HAS_VIEWER)
  endif()
  set_target_properties(_napl PROPERTIES INSTALL_RPATH "$ORIGIN")  # libmujoco.so is installed next to it
  install(TARGETS _napl mujoco LIBRARY DESTINATION napl COMPONENT napl_python)
endif()
