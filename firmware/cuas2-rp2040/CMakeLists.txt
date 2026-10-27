# cuas2 RP2040 firmware (UF2), pico-sdk + TinyUSB + PIO (bidirectional DShot). Needs the Arm toolchain and PICO_SDK_PATH: tools/cuas2_fw_build.sh sets both up on
# hfbox. The hardware-independent core is also built for x86 by host/CMakeLists.txt (tools/cuas2_fw_host_test.sh). The v1 sources (cobs, crc16, proto, planner,
# homing, tmc2209) are compiled from firmware/cuas-lite-rp2040 by path; nothing there is edited.
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico)
if(NOT DEFINED ENV{PICO_SDK_PATH})
  message(FATAL_ERROR "set PICO_SDK_PATH (pico-sdk 2.x with the tinyusb submodule); see firmware/cuas2-rp2040/README.md")
endif()
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(cuas2_rp2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(LITE ${CMAKE_CURRENT_LIST_DIR}/../cuas-lite-rp2040)
set(CORE ${LITE}/src/cobs.c ${LITE}/src/crc16.c ${LITE}/src/proto.c ${LITE}/src/planner.c ${LITE}/src/homing.c
         src/config.c src/fsm.c src/launcher.c src/feeder.c src/dshot.c)
add_executable(cuas2_rp2040 ${CORE} src/pico/main.c src/pico/hal_cuas2.c src/pico/dshot_pio.c ${LITE}/src/pico/tmc2209.c)
pico_generate_pio_header(cuas2_rp2040 ${CMAKE_CURRENT_LIST_DIR}/src/pico/dshot.pio)
# this tree's include/ first: its fw_core.h and pins.h shadow lite's (lite's tmc2209.c only needs the UART pin names, which pins.h keeps)
target_include_directories(cuas2_rp2040 PRIVATE include src src/pico ${LITE}/src/pico)
target_compile_options(cuas2_rp2040 PRIVATE -Wall -Wextra -Wshadow)
# CDC only, no vendor reset interface: the only way into the bootloader is the BOOTSEL message (SAFE state only) or the BOOTSEL button.
target_compile_definitions(cuas2_rp2040 PRIVATE
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0
  PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
  PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
target_link_libraries(cuas2_rp2040 pico_stdlib pico_stdio_usb pico_unique_id pico_flash hardware_pwm hardware_uart hardware_timer hardware_watchdog
  hardware_flash hardware_irq hardware_pio hardware_clocks pico_bootrom m)
pico_enable_stdio_usb(cuas2_rp2040 1)
pico_enable_stdio_uart(cuas2_rp2040 0)
pico_add_extra_outputs(cuas2_rp2040)
