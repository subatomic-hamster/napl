# All FetchContent pins live here. Tags verified on GitHub 2026-10-06.
include(FetchContent)

# MuJoCo 3.7.0 (Apache-2.0). Only the library: no simulate/samples/tests/python utils.
# Its own transitive deps (tinyxml2, qhull, lodepng, ccd, tinyobjloader, MarchingCubeCpp)
# are fetched by MuJoCo itself into the same _deps dir (pinned by its cmake/MujocoDependencies.cmake).
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
set(MUJOCO_WITH_USD OFF CACHE BOOL "" FORCE)
set(MUJOCO_USE_FILAMENT OFF CACHE BOOL "" FORCE)
# MuJoCo auto-enables LTO and prefers lld; GCC's LTO objects through lld yield an EMPTY libmujoco.so
# (link "succeeds", symbols missing). Pre-seed its lld probe result so it falls back to the default linker.
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  set(SUPPORTS_LLD OFF CACHE INTERNAL "")
endif()
FetchContent_Declare(mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG 3.7.0  # 72cb2b210da666617924de709406d6aadbe60c71
  GIT_SHALLOW TRUE)

# FlatBuffers v25.12.19 (Apache-2.0), target `flatc` is built for codegen in later milestones.
set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC ON CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATHASH OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(flatbuffers
  GIT_REPOSITORY https://github.com/google/flatbuffers.git
  GIT_TAG v25.12.19  # 282dcb1c3266b45600510da4810092f6ec4c85f2
  GIT_SHALLOW TRUE)

# GoogleTest v1.18.0 (BSD-3-Clause)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.18.0  # 063de7e9578f82b369302001269680b4b1553359
  GIT_SHALLOW TRUE)

# nanobind v3.1.0 (BSD-3-Clause; vendors robin_map, MIT, as a submodule). Declared here, made available
# in CMakeLists.txt only when NAPL_PYTHON=ON (it needs Python found first).
FetchContent_Declare(nanobind
  GIT_REPOSITORY https://github.com/wjakob/nanobind.git
  GIT_TAG v3.1.0  # 68480a9e6883bb1bf65e1925423321e2a89d07f9 (annotated tag; tag object 1852d88c)
  GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(mujoco flatbuffers googletest)

# Risk 2: mju_user_error handler throws a C++ exception through MuJoCo's C frames -> C needs unwind tables.
if(NOT MSVC)
  target_compile_options(mujoco PRIVATE $<$<COMPILE_LANGUAGE:C>:-fexceptions>)
endif()

# MuJoCo 3.7.0 hardcodes -Werror; GCC 16 + new glibc const-correct strchr() trips
# -Wdiscarded-qualifiers in engine_print.c. Demote that one warning (appended after MuJoCo's -Werror).
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  target_compile_options(mujoco PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wno-error=discarded-qualifiers>)
elseif(CMAKE_C_COMPILER_ID MATCHES "Clang")  # clang names it differently
  target_compile_options(mujoco PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wno-error=incompatible-pointer-types-discards-qualifiers>)
endif()
