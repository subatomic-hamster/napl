#!/usr/bin/env bash
# hopframes.sh KEY OUTROOT  "T0 T1 FPS START" ...   one record_hopper.py `frames` chunk per quadruple (T1 "-" = to the end of the run), each under a hard
# timeout. The renders are llvmpipe under Xvfb (no GPU), at most 6 threads. OUTROOT holds runs_ref/hopper-KEY-*.
set -u
export LP_NUM_THREADS=${LP_NUM_THREADS:-6}
key=$1 root=$2; shift 2
run=$(ls -d "$root"/runs_ref/hopper-"$key"-* | tail -1)
for q in "$@"; do
  set -- $q
  t1=(); [ "$2" != "-" ] && t1=(--t1 "$2")
  echo "chunk $key $q"
  tools/hopwrap.sh 480 .venv/bin/python -u tools/record_hopper.py frames "$run" "$root/hop/$key" \
    --key "$key" --t0 "$1" "${t1[@]}" --fps "$3" --start-index "$4" --timeout 440 2>&1 | grep -E '^\{|Error|error:|frame run failed' | cut -c1-400
done
echo FR_DONE
