#!/usr/bin/env bash
# Run napl builds/tests on a remote build host. The laptop only edits, runs git and syncs.
#
#   tools/remote.sh bootstrap         one-time: user-local toolchain venv (cmake, ninja, uv) on the host
#   tools/remote.sh sync              rsync the working tree to the host
#   tools/remote.sh run [--gpu] [--nolock] CMD
#                                     sync, then run CMD in the remote source dir (blocks, streams output);
#                                     holds the CPU lock unless --nolock; killed after NAPL_TIMEOUT s (3600)
#   tools/remote.sh bg NAME CMD       sync, then run CMD in tmux session napl-NAME; output to logs/NAME.log
#   tools/remote.sh log NAME [N]      last N lines of that log (no follow) + whether it is still running
#   tools/remote.sh pull PATH...      copy remote paths (relative to the remote dir) back into ./remote-out/
#
# Example:
#   tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && ctest --preset dev --output-on-failure'
#
# Host-agnostic: today hfbox, later an AWS EC2 Ubuntu 24.04 instance. Point at another host with
#   NAPL_REMOTE=<ssh alias>  NAPL_REMOTE_DIR=<abs dir>  NAPL_GPU_LOCK=  (empty = dedicated host, no shared GPU lock)
# Parallel git worktrees: one NAPL_REMOTE_DIR each; the default CPU lock is per remote dir, so worktrees run
# concurrently (set NAPL_CPU_LOCK to one shared file to serialise them on a shared host).
# Baseline the code must build on: Ubuntu 24.04, GCC 13, Python 3.12.
set -euo pipefail

REMOTE=${NAPL_REMOTE:-hfbox}
DIR=${NAPL_REMOTE_DIR:-/workspace/wt/napl}
JOBS=${NAPL_JOBS:-16}  # hfbox is dedicated to napl (user, 2026-10-10): use all 16 cores
GPU_LOCK=${NAPL_GPU_LOCK-/workspace/airtools/.gpu.lock}
CPU_LOCK=${NAPL_CPU_LOCK:-$DIR/.cpu.lock}  # per remote dir: one heavy job at a time per worktree
# Shared sccache (CMakeLists uses it when on PATH; a no-op on hosts without $CACHE/bin/sccache). The sccache
# server is one per host and reads BASEDIRS at start, so list every worktree dir: object keys then ignore the
# /workspace/wt/napl-X prefix and clean builds of the same commit in different worktrees hit each other.
CACHE=${NAPL_CACHE_DIR:-/workspace/wt/napl-cache}
BASEDIRS=${NAPL_SCCACHE_BASEDIRS:-\$(ls -d /workspace/wt/napl /workspace/wt/napl-* | grep -v -- -cache | paste -sd:)}
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

# Remote shell prefix: toolchain on PATH, thread count, CPU lock so one worktree's jobs never stack.
# lock: 1 = take the CPU lock (default), 0 = --nolock (light commands, long-lived servers).
prefix() {
  local gpu=$1 lock=${2:-1} p="cd $DIR/src && export PATH=$DIR/env/bin:$CACHE/bin:\$PATH CMAKE_BUILD_PARALLEL_LEVEL=$JOBS CTEST_PARALLEL_LEVEL=$JOBS SCCACHE_DIR=$CACHE/sccache SCCACHE_CACHE_SIZE=4G SCCACHE_BASEDIRS=$BASEDIRS &&"
  [[ $lock == 1 ]] && p+=" flock $CPU_LOCK"
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
    gpu=0 lock=1
    while [[ ${1:-} == --* ]]; do
      case $1 in --gpu) gpu=1 ;; --nolock) lock=0 ;; *) echo "unknown option $1" >&2; exit 2 ;; esac; shift
    done
    sync_tree
    # A foreground job never holds the lock past NAPL_TIMEOUT (default 1 h): a hung or
    # forgotten server must not block every other napl job. Long work belongs in `bg`.
    rssh "$(prefix $gpu $lock) timeout ${NAPL_TIMEOUT:-3600} bash -c $(printf '%q' "$*")"
    ;;
  bg)
    name=$1; shift
    sync_tree
    rssh "tmux new -d -s napl-$name $(printf '%q' "$(prefix 0) bash -c $(printf '%q' "$*") >$DIR/logs/$name.log 2>&1")"
    echo "started tmux session napl-$name on $REMOTE; tools/remote.sh log $name"
    ;;
  log) rssh "tail -n ${2:-50} $DIR/logs/$1.log; tmux has-session -t napl-$1 2>/dev/null && echo '[running]' || echo '[finished]'" ;;
  pull)
    mkdir -p "$ROOT/remote-out"
    for p in "$@"; do rrsync -az "$REMOTE:$DIR/$p" "$ROOT/remote-out/"; done
    ;;
  *) sed -n '2,17p' "$0"; exit 2 ;;
esac
