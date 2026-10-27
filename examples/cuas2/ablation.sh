#!/usr/bin/env bash
# The Monte Carlo of docs/cuas2/sim_results.md: baseline (imm2-gnn, Ph >= 0.3) and the ablations (ab-nn tracker, imm3-gnn tracker, Ph gate off), same paired seeds.
#   tools/remote.sh bg --slots 2 mc 'examples/cuas2/ablation.sh'        (hfbox: 4 worker processes, OMP_NUM_THREADS=1; ~25 min per configuration at RUNS=10; six configurations)
# Outputs build/mc/<label>/mc.json (+ .md); the cross-configuration tables come from `mc.py --report`.
set -u
cd "$(dirname "$0")/../.." || exit 1
export OMP_NUM_THREADS=1
PY=${PY:-.venv/bin/python}
RUNS=${RUNS:-10}
PERSON=${PERSON:-15}
W=${WORKERS:-4}
SEED0=${SEED0:-0}
run() { # label fire-cfg-json
  $PY examples/cuas2/mc.py --runs "$RUNS" --person-runs "$PERSON" --workers "$W" --seed0 "$SEED0" --label "$1" --fire-cfg "$2" --out "build/mc/$1" --md "build/mc/$1.md" > "build/mc_$1.out" 2>&1
  echo "$1 exit $?"
}
mkdir -p build/mc
run imm2 '{}'
run noph '{"ph_gate": false}'
run ph0 '{"ph_tau": 0.0}'
run abnn '{"tracker": "ab-nn"}'
run imm3 '{"tracker": "imm3-gnn"}'
run imm2r05 '{"range_rel": 0.05, "sigma_px": 1.5}'
$PY examples/cuas2/mc.py --report imm2=build/mc/imm2 noph=build/mc/noph ph0=build/mc/ph0 abnn=build/mc/abnn imm3=build/mc/imm3 imm2r05=build/mc/imm2r05 --md build/mc/ablation.md
