# loco_wbc_cpp (SP4 Task 7 Step 6): loco/wbc.py in C++ (Eigen + OSQP C). Built from the top-level CMakeLists.txt with -DNAPL_LOCO_CPP=ON:
#   cmake -S . -B build -DNAPL_LOCO_CPP=ON && cmake --build build --target loco_wbc_cpp      -> build/loco/loco_wbc_cpp*.so
# loco.wbc.make() puts build/loco on sys.path and uses the module when it is there.
find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
FetchContent_MakeAvailable(nanobind)

FetchContent_MakeAvailable(eigen)  # SOURCE_SUBDIR points at nothing: sources only

set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PROFILING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(osqp)

nanobind_add_module(loco_wbc_cpp STABLE_ABI NB_STATIC wbc.cpp)
target_include_directories(loco_wbc_cpp SYSTEM PRIVATE ${eigen_SOURCE_DIR})
target_link_libraries(loco_wbc_cpp PRIVATE osqpstatic)
target_compile_options(loco_wbc_cpp PRIVATE -O3 -ffp-contract=off)
target_compile_definitions(loco_wbc_cpp PRIVATE EIGEN_MPL2_ONLY)  # an #include of an LGPL Eigen file becomes a compile error
set_target_properties(loco_wbc_cpp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/loco)
