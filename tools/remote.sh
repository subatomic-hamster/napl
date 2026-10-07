#!/usr/bin/env bash
# Run napl builds/tests on a remote build host. The laptop only edits, runs git and syncs.
#
#   tools/remote.sh bootstrap         one-time: user-local toolchain venv (cmake, ninja, uv) on the host
#   tools/remote.sh sync              rsync the working tree to the host
#   tools/remote.sh run [--gpu] CMD   sync, then run CMD in the remote source dir (blocks, streams output)
#   tools/remote.sh bg NAME CMD       sync, then run CMD in tmux session napl-NAME; output to logs/NAME.log
#   tools/remote.sh log NAME          tail that log
#   tools/remote.sh pull PATH...      copy remote paths (relative to the remote dir) back into ./remote-out/
#
# Example:
#   tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && ctest --preset dev --output-on-failure'
#
# Host-agnostic: today hfbox, later an AWS EC2 Ubuntu 24.04 instance. Point at another host with
#   NAPL_REMOTE=<ssh alias>  NAPL_REMOTE_DIR=<abs dir>  NAPL_GPU_LOCK=  (empty = dedicated host, no shared GPU lock)
# Baseline the code must build on: Ubuntu 24.04, GCC 13, Python 3.12.
set -euo pipefail

REMOTE=${NAPL_REMOTE:-hfbox}
DIR=${NAPL_REMOTE_DIR:-/workspace/wt/napl}
JOBS=${NAPL_JOBS:-6}  # hfbox house rule: heavy CPU at most 6 threads
GPU_LOCK=${NAPL_GPU_LOCK-/workspace/airtools/.gpu.lock}
ROOT=$(cd "$(dirname "$0")/.." && pwd)

# ssh exit 255 = connection failure (hfbox funnel outages last ~15 min); retry with backoff.
# retry "<connection-failure exit codes>" cmd...
retry() {
  local codes=" $1 " delay=15 rc; shift
  for _ in 1 2 3 4 5 6; do
    rc=0; "$@" || rc=$?
    [[ $codes != *" $rc "* ]] && return $rc
    echo "remote.sh: $REMOTE unreachable (rc=$rc), retrying in ${delay}s" >&2
    sleep "$delay"; delay=$((delay * 2))
  done
  return $rc
}
rssh() { retry 255 ssh -o BatchMode=yes -o ConnectTimeout=20 "$REMOTE" "$@"; }
# rsync: 255 = ssh failed, 10/12 = socket/stream dropped mid-transfer
rrsync() { retry "255 10 12" rsync -e 'ssh -o BatchMode=yes -o ConnectTimeout=20' "$@"; }

sync_tree() {
  rssh "mkdir -p $DIR/src $DIR/logs"
  rrsync -az --delete \
    --exclude /build/ --exclude /remote-out/ --exclude .git/ --exclude .venv/ \
    --exclude __pycache__/ --exclude .ruff_cache/ \
    "$ROOT/" "$REMOTE:$DIR/src/"
}

# Remote shell prefix: toolchain on PATH, thread cap, own CPU lock so napl jobs never stack.
prefix() {
  local gpu=$1 p="cd $DIR/src && export PATH=$DIR/env/bin:\$PATH CMAKE_BUILD_PARALLEL_LEVEL=$JOBS CTEST_PARALLEL_LEVEL=$JOBS && flock $DIR/.cpu.lock"
  [[ $gpu == 1 && -n $GPU_LOCK ]] && p+=" flock $GPU_LOCK"
  printf '%s' "$p"
}

cmd=${1:-}; shift || true
case $cmd in
  bootstrap)
    sync_tree
    rssh "set -e; command -v g++ rsync flock tmux >/dev/null || { echo 'need g++ rsync flock tmux on host' >&2; exit 1; }
          [ -x $DIR/env/bin/cmake ] || { python3 -m venv $DIR/env && $DIR/env/bin/pip install -q -r $DIR/src/tools/remote-requirements.txt; }
          $DIR/env/bin/cmake --version | head -1; $DIR/env/bin/ninja --version; $DIR/env/bin/uv --version; g++ --version | head -1"
    ;;
  sync) sync_tree ;;
  run)
    gpu=0; [[ ${1:-} == --gpu ]] && { gpu=1; shift; }
    sync_tree
    rssh "$(prefix $gpu) bash -c $(printf '%q' "$*")"
    ;;
  bg)
    name=$1; shift
    sync_tree
    rssh "tmux new -d -s napl-$name $(printf '%q' "$(prefix 0) bash -c $(printf '%q' "$*") >$DIR/logs/$name.log 2>&1")"
    echo "started tmux session napl-$name on $REMOTE; tools/remote.sh log $name"
    ;;
  log) rssh "tail -n 50 -f $DIR/logs/$1.log" ;;
  pull)
    mkdir -p "$ROOT/remote-out"
    for p in "$@"; do rrsync -az "$REMOTE:$DIR/$p" "$ROOT/remote-out/"; done
    ;;
  *) sed -n '2,17p' "$0"; exit 2 ;;
esac
