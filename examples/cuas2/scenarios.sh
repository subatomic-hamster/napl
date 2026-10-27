#!/usr/bin/env bash
# Every named cuas2 scenario once (the cuas-lite list: s1..s7 incl. the five s7 fault scenarios), 4 at a time; one verdict line each. Exit 1 if any episode fails.
#   tools/remote.sh run --slots 2 examples/cuas2/scenarios.sh
cd "$(dirname "$0")/../.." || exit 1
export OMP_NUM_THREADS=1
SEED=${SEED:-1}
mkdir -p build/sc
for s in s1_balloon_static s2_balloon_drift s3_hanging_targets s4_quad_3ms s4_quad_8ms s5_mix s6_person_walk s6b_person_beside_balloon \
         s7_fault_deadman s7_fault_detector s7_fault_estop s7_fault_key s7_fault_link; do
  (.venv/bin/python examples/cuas2/run.py "$s" --seed "$SEED" --brief --out "build/sc/$s" 2>&1 | tail -1 >"build/sc/$s.line"; rc=${PIPESTATUS[0]}; echo "$rc" >"build/sc/$s.rc") &
  (( $(jobs -r | wc -l) >= 4 )) && wait -n
done
wait
fail=0
for s in build/sc/*.line; do cat "$s"; done
for r in build/sc/*.rc; do [ "$(cat "$r")" = 0 ] || fail=1; done
exit $fail
