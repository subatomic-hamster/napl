# Host (x86) build of the firmware core: the shared library the Python tests load and a stand-alone C selftest. No Arm toolchain needed.
#   cmake -S firmware/cuas-lite-rp2040/host -B build/fw-host -G Ninja && cmake --build build/fw-host && ctest --test-dir build/fw-host
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.16)
project(cuas_lite_fw_host C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(FW ${CMAKE_CURRENT_LIST_DIR}/..)
set(CORE ${FW}/src/cobs.c ${FW}/src/crc16.c ${FW}/src/proto.c ${FW}/src/config.c ${FW}/src/fsm.c ${FW}/src/planner.c ${FW}/src/homing.c)
add_compile_options(-Wall -Wextra -Wshadow -fno-strict-aliasing)

add_library(fwcore SHARED ${CORE} ${CMAKE_CURRENT_LIST_DIR}/fw_host.c)
target_include_directories(fwcore PRIVATE ${FW}/include ${FW}/src)
target_link_libraries(fwcore PRIVATE m)
set_target_properties(fwcore PROPERTIES POSITION_INDEPENDENT_CODE ON OUTPUT_NAME cuas_lite_fwcore)

add_executable(fw_selftest ${CORE} ${FW}/tests/selftest.c)
target_include_directories(fw_selftest PRIVATE ${FW}/include ${FW}/src)
target_link_libraries(fw_selftest PRIVATE m)

enable_testing()
add_test(NAME fw_selftest COMMAND fw_selftest)
