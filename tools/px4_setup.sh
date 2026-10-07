#!/usr/bin/env bash
# Run ON the build host (tools/remote.sh run 'bash tools/px4_setup.sh', cwd = repo dir). Idempotent.
# 1) PX4 python venv  2) install our airframes into the PX4 tree (the only way that tree is modified).
set -euo pipefail

PX4=${PX4_DIR:-/workspace/wt/napl/px4/src}
VENV=$(dirname "$PX4")/venv
HERE=$(cd "$(dirname "$0")/.." && pwd)
export UV_CACHE_DIR=${UV_CACHE_DIR:-/workspace/wt/napl/uv-cache}

if [ ! -x "$VENV/bin/python" ]; then
  uv venv "$VENV"
  uv pip install --python "$VENV/bin/python" -r "$PX4/Tools/setup/requirements.txt"
fi

AF=$PX4/ROMFS/px4fmu_common/init.d-posix/airframes
for f in "$HERE"/px4/airframes/*; do
  n=$(basename "$f")
  cp "$f" "$AF/$n"
  if ! grep -qxF "	$n" "$AF/CMakeLists.txt"; then
    # PX4 lists posix airframes by name; insert before the "Reserve for custom models" comment
    awk -v n="$n" '/Reserve for custom models/ { print "\t" n } { print }' "$AF/CMakeLists.txt" >"$AF/CMakeLists.txt.new"
    mv "$AF/CMakeLists.txt.new" "$AF/CMakeLists.txt"
  fi
done
echo "px4_setup: ok"
