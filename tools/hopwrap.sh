#!/usr/bin/env bash
# hopwrap.sh SECONDS CMD...  run CMD in its own session under `timeout`, then kill whatever is left of its process group
# (a timed-out napl / Xvfb run must not orphan children on the shared host). Exit status = CMD's.
set -u
pg=$(mktemp)
setsid -w bash -c 'echo $$ > "$0"; shift_t=$1; shift; exec timeout "$shift_t" "$@"' "$pg" "$@"
rc=$?
kill -- "-$(cat "$pg")" 2>/dev/null
rm -f "$pg"
exit $rc
