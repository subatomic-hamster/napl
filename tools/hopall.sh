#!/usr/bin/env bash
# hopall.sh OUTROOT   every remote step of the hopper video in ONE job (one CPU-lock wait): the build in four recordings, the frames of divert25 and gusts,
# the VP9 encode and sample frames. SKIP_BUILD=1 reuses the finished build recordings (OUTROOT/hop/build{A,B,W,C}), SKIP_FRAMES=1 the sim frames.
# Run from the remote source dir via `tools/remote.sh bg`. Needs OUTROOT/runs_ref (napl program runs: divert25, gusts) and OUTROOT/../vidvenv (av, Pillow).
set -u
O=$1
export LP_NUM_THREADS=${LP_NUM_THREADS:-6} NAPL_ROUTE_CACHE=$O/routecache  # 16 once the host rule allows all cores
PY=.venv/bin/python
VV=$(dirname "$O")/vidvenv/bin/python
mkdir -p "$O/hop"

# the build, split where no flatten session is open (oplog step indices): [0,20) library, [20,32) rest of the parts and colours, [32,40) wiring,
# [40,48) devices, program, save. Each part loads the design the previous one saved; the last must equal assemblies/hopper.toml.
B() {  # B NAME STEPS [extra args]
  name=$1 steps=$2; shift 2
  rm -rf "$O/hop/build$name"
  tools/hopwrap.sh 640 $PY -u tools/record_hopper.py build docs/showcase/hopper/oplog.json "$O/hop/build$name" \
    --expect assemblies/hopper.toml --steps "$steps" --timeout 560 "$@" > "$O/hop/build$name.log" 2>&1
  echo "build $name rc=$? frames=$(ls "$O/hop/build$name" | grep -c '^f') $(grep -E 'differ|expected|error' "$O/hop/build$name.log" | grep -v leaked | tail -2 | cut -c1-200)"
}
if [ -z "${SKIP_BUILD:-}" ]; then
  B A 0:20 --save "$O/hop/partA.toml"
  B B 20:32 --load "$O/hop/partA.toml" --save "$O/hop/partB.toml"
  B W 32:40 --load "$O/hop/partB.toml" --save "$O/hop/partW.toml"
  B C 40:48 --load "$O/hop/partW.toml" --save "$O/hop/partC.toml" --final
fi

if [ -z "${SKIP_FRAMES:-}" ]; then
  rm -rf "$O/hop/divert25" "$O/hop/gusts"
  tools/hopframes.sh divert25 "$O" "0 4.5 20 0" "4.5 8 60 90" "8 18 5 300" "18 25.6 10 350" "25.6 28 20 426" "28 42 5 474" "42 44.5 20 544" "44.5 - 60 594"
  tools/hopframes.sh gusts "$O" "0 4.5 10 0" "4.5 6.5 20 45" "6.5 13.5 5 85" "13.5 19.5 20 120" "19.5 30.5 5 240" "30.5 33 20 295" "33 - 40 345"
fi

$VV -u tools/record_hopper.py encode "$O/hop/hopper.webm" --build "$O/hop/buildA" "$O/hop/buildB" "$O/hop/buildW" "$O/hop/buildC" \
  --sim divert25="$O/hop/divert25" gusts="$O/hop/gusts" 2>&1 | tail -5 | cut -c1-300
rm -rf "$O/hop/samples"
$VV tools/hopsample.py "$O/hop/hopper.webm" "$O/hop/samples" 40 2>&1 | tail -3
echo ALL_DONE
