#!/usr/bin/env bash
# Cross-build the cuas2 RP2040 UF2 on hfbox with the cached Arm GNU toolchain + pico-sdk (same cache as tools/fw_build.sh; no apt).
#   tools/remote.sh run --slots 1 tools/cuas2_fw_build.sh   -> build/cuas2-fw-rp2040/cuas2_rp2040.uf2
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CACHE=${NAPL_CACHE_DIR:-/workspace/wt/napl-cache}
export PICO_SDK_PATH=${PICO_SDK_PATH:-$CACHE/pico-sdk}
TC=${ARM_TOOLCHAIN:-$CACHE/arm-gnu/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi}
export PICO_TOOLCHAIN_PATH=$TC
export PATH="$TC/bin:$PATH"
B=${FW_BUILD:-$ROOT/build/cuas2-fw-rp2040}
cmake -S "$ROOT/firmware/cuas2-rp2040" -B "$B" -G Ninja -DPICO_NO_PICOTOOL=1 "$@" >/dev/null
cmake --build "$B"
arm-none-eabi-size "$B/cuas2_rp2040.elf"
python3 "$ROOT/tools/bin2uf2.py" "$B/cuas2_rp2040.bin" "$B/cuas2_rp2040.uf2"
ls -l "$B/cuas2_rp2040.bin" "$B/cuas2_rp2040.uf2"
