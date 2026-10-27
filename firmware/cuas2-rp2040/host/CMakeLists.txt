# Host (x86) build of the cuas2 firmware core: the shared library the Python tests load and a stand-alone C selftest. No Arm toolchain needed.
#   cmake -S firmware/cuas2-rp2040/host -B build/cuas2-fw-host -G Ninja && cmake --build build/cuas2-fw-host && ctest --test-dir build/cuas2-fw-host
# The lite files (cobs.c crc16.c proto.c planner.c homing.c) are compiled by relative path against THIS tree's include/fw_core.h; lite's include/ is never on
# the include path. -ffp-contract=off keeps the one floating-point expression (rpm cap) identical to the python twin.
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.16)
project(cuas2_fw_host C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(FW ${CMAKE_CURRENT_LIST_DIR}/..)
set(LITE ${FW}/../cuas-lite-rp2040)
set(CORE ${LITE}/src/cobs.c ${LITE}/src/crc16.c ${LITE}/src/proto.c ${LITE}/src/planner.c ${LITE}/src/homing.c
         ${FW}/src/config.c ${FW}/src/fsm.c ${FW}/src/launcher.c ${FW}/src/feeder.c ${FW}/src/dshot.c)
add_compile_options(-Wall -Wextra -Wshadow -fno-strict-aliasing -ffp-contract=off)

add_library(fwcore SHARED ${CORE} ${CMAKE_CURRENT_LIST_DIR}/fw_host.c)
target_include_directories(fwcore PRIVATE ${FW}/include ${FW}/src)
target_link_libraries(fwcore PRIVATE m)
set_target_properties(fwcore PROPERTIES POSITION_INDEPENDENT_CODE ON OUTPUT_NAME cuas2_fwcore)

add_executable(fw_selftest ${CORE} ${FW}/tests/selftest.c)
target_include_directories(fw_selftest PRIVATE ${FW}/include ${FW}/src)
target_link_libraries(fw_selftest PRIVATE m)

enable_testing()
add_test(NAME fw_selftest COMMAND fw_selftest)
