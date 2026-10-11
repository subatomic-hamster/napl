#!/usr/bin/env bash
# Cross-build the RP2040 UF2 on hfbox with the cached Arm GNU toolchain + pico-sdk (no apt). Optional: napl's default build never calls this.
#   tools/remote.sh run --slots 1 tools/fw_build.sh        -> build/fw-rp2040/cuas_lite_rp2040.uf2 (pull it with tools/remote.sh pull)
# One-time cache setup (versions pinned in firmware/cuas-lite-rp2040/README.md): tools/fw_setup_toolchain.sh
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CACHE=${NAPL_CACHE_DIR:-/workspace/wt/napl-cache}
export PICO_SDK_PATH=${PICO_SDK_PATH:-$CACHE/pico-sdk}
TC=${ARM_TOOLCHAIN:-$CACHE/arm-gnu/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi}
export PICO_TOOLCHAIN_PATH=$TC
export PATH="$TC/bin:$PATH"
B=${FW_BUILD:-$ROOT/build/fw-rp2040}
cmake -S "$ROOT/firmware/cuas-lite-rp2040" -B "$B" -G Ninja -DPICO_NO_PICOTOOL=1 "$@" >/dev/null
cmake --build "$B"
arm-none-eabi-size "$B/cuas_lite_rp2040.elf"
# the SDK's own UF2 step needs picotool (libusb headers); convert the .bin instead
python3 "$ROOT/tools/bin2uf2.py" "$B/cuas_lite_rp2040.bin" "$B/cuas_lite_rp2040.uf2"
ls -l "$B/cuas_lite_rp2040.bin" "$B/cuas_lite_rp2040.uf2"
