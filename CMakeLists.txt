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
add_custom_target(napl_schema_gen DEPENDS ${NAPL_GEN_DIR}/napl_generated.h)
add_library(napl_schema INTERFACE)
add_dependencies(napl_schema napl_schema_gen)
target_include_directories(napl_schema INTERFACE ${NAPL_GEN_DIR})
target_link_libraries(napl_schema INTERFACE flatbuffers)

# Seam enforced by the build graph: napl_core gets no mujoco include dirs, so including mujoco.h fails.
find_package(Threads REQUIRED)
add_library(napl_core STATIC src/core/world.cpp src/core/bus.cpp src/core/replay.cpp src/core/host.cpp
  src/core/monitor.cpp)
target_include_directories(napl_core PUBLIC src)
target_link_libraries(napl_core PUBLIC napl_schema Threads::Threads)

add_library(napl_mujoco STATIC src/mujoco/mujoco_solver.cpp)
target_link_libraries(napl_mujoco PUBLIC napl_core mujoco)

# Static libs end up inside the _napl shared module: they need -fPIC.
set_target_properties(napl_core napl_mujoco flatbuffers PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(napl_cli src/cli/main.cpp)
set_target_properties(napl_cli PROPERTIES OUTPUT_NAME napl-replay)  # the Python CLI owns `napl`
target_link_libraries(napl_cli PRIVATE napl_core napl_mujoco mujoco)

add_executable(napl_tests tests/cpp/test_smoke.cpp tests/cpp/test_determinism.cpp tests/cpp/test_bus.cpp
  tests/cpp/test_replay.cpp tests/cpp/test_host.cpp)
add_dependencies(napl_tests napl_cli)  # test_replay runs the napl-replay executable
target_link_libraries(napl_tests PRIVATE napl_core napl_mujoco mujoco GTest::gtest_main)
target_compile_definitions(napl_tests PRIVATE
  NAPL_CLI_EXE="$<TARGET_FILE:napl_cli>"
  NAPL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
  NAPL_HUMANOID_XML="${mujoco_SOURCE_DIR}/model/humanoid/humanoid.xml")
if(WIN32)  # no rpath on Windows: put mujoco.dll next to the test exe
  add_custom_command(TARGET napl_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:mujoco> $<TARGET_FILE_DIR:napl_tests>)
  add_custom_command(TARGET napl_cli POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:mujoco> $<TARGET_FILE_DIR:napl_cli>)
endif()
gtest_discover_tests(napl_tests PROPERTIES LABELS cpu)

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
  target_link_libraries(_napl PRIVATE napl_core napl_mujoco mujoco)
  set_target_properties(_napl PROPERTIES INSTALL_RPATH "$ORIGIN")  # libmujoco.so is installed next to it
  install(TARGETS _napl mujoco LIBRARY DESTINATION napl COMPONENT napl_python)
endif()
