#!/usr/bin/env bash
# Host (x86) build of the cuas2 firmware core + ctest + the cuas2 python tests. No Arm toolchain needed. Run on hfbox:
#   tools/remote.sh run --slots 8 tools/cuas2_fw_host_test.sh        (extra args go to pytest, e.g. -k fuzz)
# The library lands in build/cuas2-fw-host/libcuas2_fwcore.so, where tests/cuas2/fwharness.py looks for it (or NAPL_CUAS2_FW=PATH).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
B=${CUAS2_FW_HOST_BUILD:-$ROOT/build/cuas2-fw-host}
cmake -S "$ROOT/firmware/cuas2-rp2040/host" -B "$B" -G Ninja >/dev/null
cmake --build "$B"
ctest --test-dir "$B" --output-on-failure
"$ROOT/tools/cuas2_fsm_drift.sh"
cd "$ROOT"
export CUAS2_FW_REQUIRED=1  # a missing library is a failure here, not a skip
export PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}"
python -m pytest -n "${PYTEST_WORKERS:-8}" -p no:cacheprovider -q tests/cuas2/test_protocol2.py tests/cuas2/test_fsm.py tests/cuas2/test_fw_vectors2.py "$@"
