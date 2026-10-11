# cuas-lite RP2040 firmware (UF2), pico-sdk + TinyUSB. Needs the Arm toolchain and PICO_SDK_PATH: tools/fw_build.sh sets both up on hfbox.
# The hardware-independent core is also built for x86 by host/CMakeLists.txt (tools/fw_host_test.sh); napl's own build never includes this directory.
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico)
if(NOT DEFINED ENV{PICO_SDK_PATH})
  message(FATAL_ERROR "set PICO_SDK_PATH (pico-sdk 2.3.1 with the tinyusb submodule); see firmware/cuas-lite-rp2040/README.md")
endif()
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(cuas_lite_rp2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(CORE src/cobs.c src/crc16.c src/proto.c src/config.c src/fsm.c src/planner.c src/homing.c)
add_executable(cuas_lite_rp2040 ${CORE} src/pico/main.c src/pico/hal_pico.c src/pico/tmc2209.c)
target_include_directories(cuas_lite_rp2040 PRIVATE include src src/pico)
target_compile_options(cuas_lite_rp2040 PRIVATE -Wall -Wextra -Wshadow)
# CDC only, no vendor reset interface: the only way into the bootloader is the BOOTSEL message (SAFE state only) or the BOOTSEL button.
target_compile_definitions(cuas_lite_rp2040 PRIVATE
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0
  PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
  PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
# (the default USB strings stay: /dev/serial/by-id/usb-Raspberry_Pi_Pico_<uid>-if00 is what the app and the udev rule look for)
target_link_libraries(cuas_lite_rp2040 pico_stdlib pico_stdio_usb pico_unique_id pico_flash hardware_pwm hardware_uart hardware_timer hardware_watchdog
  hardware_flash hardware_irq pico_bootrom m)
pico_enable_stdio_usb(cuas_lite_rp2040 1)
pico_enable_stdio_uart(cuas_lite_rp2040 0)
pico_add_extra_outputs(cuas_lite_rp2040)
