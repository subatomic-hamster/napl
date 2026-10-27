#!/usr/bin/env bash
# Spec 12 FW check 6: the safety-FSM functions cuas2 shares with lite (watchdog, ARM, heartbeat, E-stop/key edge handling, state transitions) must stay
# byte-identical to firmware/cuas-lite-rp2040/src/fsm.c. Prints the diff stat of every function in tests/cuas2/fsm_shared_functions.txt and exits 1 if any
# differs. The weapon functions (FIRE, FLYWHEEL, STATUS, CFG_SET, outputs, feeder stage) are not listed: they are expected to differ.
#   tools/cuas2_fsm_drift.sh            # real files
#   CUAS2_FSM=/path/to/fsm.c tools/cuas2_fsm_drift.sh   # another cuas2 copy (negative control)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LITE="$ROOT/firmware/cuas-lite-rp2040/src/fsm.c"
NEW="${CUAS2_FSM:-$ROOT/firmware/cuas2-rp2040/src/fsm.c}"
LIST="$ROOT/tests/cuas2/fsm_shared_functions.txt"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# body of C function $2 in file $1: from its definition line (name followed by "(" at the start of a declaration) to the first line that is exactly "}"
extract() {
  awk -v fn="$2" '
    !on && $0 ~ ("^[A-Za-z_][A-Za-z0-9_ \\*]* \\*?" fn "\\(") && $0 !~ /;[ \t]*$/ { on = 1 }
    on { print }
    on && !seen { seen = 1; o = gsub(/\{/, "{"); c = gsub(/\}/, "}"); if (o > 0 && o == c) exit }  # one-line function
    on && $0 == "}" { exit }
  ' "$1"
}

bad=0
n=0
while read -r fn; do
  case "$fn" in ''|\#*) continue ;; esac
  n=$((n + 1))
  extract "$LITE" "$fn" > "$TMP/lite.$fn"
  extract "$NEW" "$fn" > "$TMP/new.$fn"
  if [ ! -s "$TMP/lite.$fn" ] || [ ! -s "$TMP/new.$fn" ]; then
    echo "MISSING $fn (lite: $(wc -l < "$TMP/lite.$fn") lines, cuas2: $(wc -l < "$TMP/new.$fn") lines)"
    bad=$((bad + 1))
    continue
  fi
  if diff -q "$TMP/lite.$fn" "$TMP/new.$fn" > /dev/null; then
    echo "same    $fn ($(wc -l < "$TMP/lite.$fn") lines)"
  else
    echo "DIFFERS $fn"
    diff -u "$TMP/lite.$fn" "$TMP/new.$fn" | diffstat 2>/dev/null || diff -u "$TMP/lite.$fn" "$TMP/new.$fn" | grep -c '^[+-][^+-]' | sed 's/^/        changed lines: /'
    bad=$((bad + 1))
  fi
done < "$LIST"
echo "checked $n shared functions, $bad drifted"
[ "$bad" -eq 0 ]
