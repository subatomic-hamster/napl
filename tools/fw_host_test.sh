#!/usr/bin/env bash
# Build and test the RP2040 firmware core on the host (x86 gcc, cmake, ninja). Optional: napl's default build never calls this and needs no Arm toolchain.
#   tools/fw_host_test.sh              C selftest (ctest) + shared vectors + gate matrix + the twin rigs (test_twin, test_gates) run against the C core
#   tools/remote.sh run --slots 1 tools/fw_host_test.sh      on hfbox, as everything heavy
# The UF2 cross build is tools/fw_build.sh.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
B=${FW_HOST_BUILD:-$ROOT/build/fw-host}
cmake -S "$ROOT/firmware/cuas-lite-rp2040/host" -B "$B" -G Ninja >/dev/null
cmake --build "$B"
ctest --test-dir "$B" --output-on-failure
PY=${PYTHON:-$ROOT/.venv/bin/python}
[ -x "$PY" ] || PY=python3
cd "$ROOT"
"$PY" -m pytest -q -p no:cacheprovider tests/lite/test_fw_vectors.py tests/lite/test_fw_core.py
"$PY" -m pytest -q -p no:cacheprovider tests/lite/test_twin.py tests/lite/test_gates.py --fw-core="$B/libcuas_lite_fwcore.so"
