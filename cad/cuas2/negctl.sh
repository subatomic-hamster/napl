#!/usr/bin/env bash
# Acceptance 2 negative controls for cad/cuas2 (run on the build host, from the repo root; PY = the build123d interpreter):
#   PY=/workspace/wt/napl-phys/cad-venv/bin/python cad/cuas2/negctl.sh
# Each variant regenerates the whole CAD with one parameter changed through its CUAS2_* override and prints the check lines
# that depend on it; the pocket control must exit non-zero with C-POCKET FAIL.
set -u
PY=${PY:-python3}
run() { # name envs...
  local name=$1; shift
  echo "=== $name ($*)"
  env "$@" "$PY" cad/cuas2/build.py --no-preview --out "build/cad/cuas2-neg-$name" 2>&1 | grep -E '^\s+\[(PASS|FAIL)\] C-(GAP|POCKET|FEED|NIP|INTERLOCK|BAL|CYCLE)|^(PASS|FAIL)$' | cut -c1-260
  echo "exit ${PIPESTATUS[0]}"
}
run base
run swing CUAS2_GATE_SWING_DEG=90
run wheel CUAS2_WHEEL_D=50
run ball CUAS2_BALL_D=23.0
run pocket CUAS2_POCKET_D_BALLS=2.2
