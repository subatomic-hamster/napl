#!/usr/bin/env bash
# One-time, idempotent: the Arm GNU Toolchain (arm-none-eabi) and pico-sdk (+ TinyUSB submodule) into the shared cache, no apt. Versions are pinned here
# and recorded in firmware/cuas-lite-rp2040/README.md. Run on hfbox:  tools/remote.sh run --nolock tools/fw_setup_toolchain.sh
set -euo pipefail
CACHE=${NAPL_CACHE_DIR:-/workspace/wt/napl-cache}
ARM=15.2.rel1
ARM_SHA256=597893282ac8c6ab1a4073977f2362990184599643b4c5ee34870a8215783a16   # from the .sha256asc next to the tarball on Arm's download host
ARM_TAR=arm-gnu-toolchain-$ARM-x86_64-arm-none-eabi.tar.xz
PICO_TAG=2.3.1
D=$CACHE/arm-gnu
mkdir -p "$D"
if [ ! -x "$D/${ARM_TAR%.tar.xz}/bin/arm-none-eabi-gcc" ]; then
  [ -f "$D/$ARM_TAR" ] || curl -sSL -o "$D/$ARM_TAR" "https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/$ARM/binrel/$ARM_TAR"
  echo "$ARM_SHA256  $D/$ARM_TAR" | sha256sum -c -
  tar -C "$D" -xf "$D/$ARM_TAR"
fi
if [ ! -d "$CACHE/pico-sdk/.git" ]; then
  git clone --depth 1 --branch "$PICO_TAG" https://github.com/raspberrypi/pico-sdk.git "$CACHE/pico-sdk"
  git -C "$CACHE/pico-sdk" submodule update --init --depth 1 lib/tinyusb
fi
"$D/${ARM_TAR%.tar.xz}/bin/arm-none-eabi-gcc" --version | head -1
git -C "$CACHE/pico-sdk" describe --tags
git -C "$CACHE/pico-sdk/lib/tinyusb" rev-parse HEAD
