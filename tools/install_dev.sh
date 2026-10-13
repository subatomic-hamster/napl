#!/usr/bin/env bash
# The editable install that ALWAYS rebuilds the C++ extension (napl/_napl.abi3.so): a plain `uv pip install -e .` sees an
# unchanged pyproject and silently keeps the stale .so already in site-packages (scikit-build-core's editable mode only
# redirects the Python files). --reinstall-package + --refresh-package force a build, which is incremental in build/ (ninja
# recompiles only what changed; sccache runs in preprocessor mode, see CMakeLists.txt).
#   tools/install_dev.sh [extra uv pip args]          (inside the venv, or with UV_PYTHON / VIRTUAL_ENV set)
#   on hfbox: tools/remote.sh run 'PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh'
set -euo pipefail
cd "$(dirname "$0")/.."
uv pip install --reinstall-package napl --refresh-package napl -e ".[dev,px4]" "$@"
py=$(uv python find)  # the interpreter uv just installed into (.venv or the active env)
so=$("$py" -c 'import napl; print(napl._napl.__file__)')  # redirect mode: the .so lives in site-packages, not python/napl
ls -l --time-style=full-iso "$so"
"$py" -c 'import napl; print("napl._napl has_viewer =", napl._napl.has_viewer)'
