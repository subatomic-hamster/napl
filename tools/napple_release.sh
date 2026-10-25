#!/bin/bash
# napple (Apple Silicon) release: build, qualify and publish the arm64 wheel from a commit.
#
#   tools/napple_release.sh all [REF] [OUT]        build REF (default HEAD), then qualify on Python 3.12 (with a
#                                                  Cocoa window) and 3.13
#   tools/napple_release.sh build REF OUT          export REF to OUT/src and build OUT/dist/<wheel> + metadata
#   tools/napple_release.sh qualify OUT PY [--visible | --no-render REASON]
#                                                  install the wheel in a fresh PY venv and run the sandboxed smoke
#   tools/napple_release.sh publish OUT TAG NOTES  tag the wheel's source commit and create a GitHub pre-release
#
# OUT defaults to ~/Library/Caches/napl/napple/<sha>. Fetched dependency sources are shared across builds in
# NAPL_DEPS_CACHE (default ~/Library/Caches/napl/napple/deps). Local builds run at background QoS with 2 jobs so the
# machine stays usable; override with NAPL_RELEASE_QOS=default and NAPL_RELEASE_JOBS=N (CI sets both).
# Needs uv, cmake, git and Xcode command line tools on an arm64 macOS 26 host (the pinned slangc requires it);
# qualify alone runs on any arm64 macOS >= 14. publish also needs gh. Nothing is published automatically.
set -euo pipefail

die() { echo "napple_release: $*" >&2; exit 1; }
[ "$(uname -m)" = arm64 ] && [ "$(uname -s)" = Darwin ] || die "arm64 macOS only"
CACHE_ROOT="$HOME/Library/Caches/napl/napple"
DEPS="${NAPL_DEPS_CACHE:-$CACHE_ROOT/deps}"
JOBS="${NAPL_RELEASE_JOBS:-2}"
QOS=(taskpolicy -b)
[ "${NAPL_RELEASE_QOS:-background}" = background ] || QOS=()
CHECKOUT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel 2>/dev/null || true)"

build() {
  local ref=$1 out=$2 sha
  [ -n "$CHECKOUT" ] || die "build must run from a napl checkout"
  sha=$(git -C "$CHECKOUT" rev-parse --verify "$ref^{commit}")
  rm -rf "$out" && mkdir -p "$out/src" "$out/dist" "$DEPS"
  out=$(cd "$out" && pwd -P)
  git -C "$CHECKOUT" archive "$sha" | tar -x -C "$out/src"
  uv venv -q -p 3.12 "$out/build-venv"
  uv pip install -q --python "$out/build-venv/bin/python" build scikit-build-core==1.1.1 ninja 'cmake>=3.25'
  export PATH="$out/build-venv/bin:$PATH"
  (cd "$out/src" && NAPL_SHA=$sha OUT=$out "$out/build-venv/bin/python" - <<'PY'
import hashlib, json, os, pathlib, platform, subprocess, sys
def cmd(*args):
    return subprocess.check_output(args, text=True).strip()
run = os.environ.get('GITHUB_RUN_ID')
meta = dict(source_sha=os.environ['NAPL_SHA'], source_dirty=False,
            builder=f"github-actions {os.environ['GITHUB_SERVER_URL']}/{os.environ['GITHUB_REPOSITORY']}/actions/runs/{run}" if run else 'local',
            toolchain=dict(compiler=cmd('clang', '--version'), sdk=cmd('xcrun', '--sdk', 'macosx', '--show-sdk-version'),
                           sdk_path=cmd('xcrun', '--sdk', 'macosx', '--show-sdk-path'), cmake=cmd('cmake', '--version'),
                           architecture=platform.machine(), python=sys.version, configuration='RelWithDebInfo',
                           deployment_target='14.0', viewer=True, testing=False),
            dependency_manifest_sha256=hashlib.sha256(pathlib.Path('cmake/deps.cmake').read_bytes()).hexdigest(),
            hardware=dict(chip=cmd('sysctl', '-n', 'machdep.cpu.brand_string'),
                          physical_cores=int(cmd('sysctl', '-n', 'hw.physicalcpu')),
                          memory_bytes=int(cmd('sysctl', '-n', 'hw.memsize')), os=platform.platform()))
pathlib.Path(os.environ['OUT'], 'build-metadata.json').write_text(json.dumps(meta, indent=2) + '\n')
PY
  )
  local t0=$SECONDS
  (cd "$out/src" && MACOSX_DEPLOYMENT_TARGET=14.0 CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
    ${QOS[@]+"${QOS[@]}"} "$out/build-venv/bin/python" -m build --wheel --no-isolation \
      --config-setting=build-dir="$out/build" \
      --config-setting=wheel.build-tag="1napple${sha:0:12}" \
      --config-setting=cmake.define.FETCHCONTENT_BASE_DIR="$DEPS" \
      --config-setting=cmake.define.CMAKE_OSX_ARCHITECTURES=arm64 \
      --config-setting=cmake.define.CMAKE_OSX_DEPLOYMENT_TARGET=14.0 \
      --config-setting=cmake.define.NAPL_PYTHON=ON \
      --config-setting=cmake.define.NAPL_VIEWER=ON \
      --config-setting=cmake.define.BUILD_TESTING=OFF -o "$out/dist" > "$out/build.log" 2>&1) \
    || { tail -40 "$out/build.log" >&2; die "wheel build failed (full log: $out/build.log)"; }
  echo "wheel built in $((SECONDS - t0)) s"
  cp "$out/src/cmake/deps.cmake" "$out/dist/dependency-pins.cmake"
  cp "$out/build-metadata.json" "$out/dist/build-metadata.json"
  # -ffp-contract=off: the smoke's Python reference evaluates the SHM control sequence without arm64 FMA contraction.
  xcrun clang++ -std=c++20 -ffp-contract=off -arch arm64 -mmacosx-version-min=14.0 -I "$out/src/include" \
    -I "$out/src/tests/cpp" "$out/src/tests/cpp/shm_test_client.cpp" -o "$out/shm-smoke-client"
  cp "$out/src/tools/apple_wheel_smoke.py" "$out/apple_wheel_smoke.py"
  sums "$out"
}

sums() { (cd "$1/dist" && shasum -a 256 *.whl > SHA256SUMS && cat SHA256SUMS); }

qualify() {
  local out=$1 py=$2 venv pkg macos wheel
  shift 2
  local extra=("$@")
  out=$(cd "$out" && pwd -P)  # resolved, e.g. /tmp -> /private/tmp, so venv and package paths agree
  wheel=$(ls "$out"/dist/*.whl) || die "no wheel in $out/dist"
  venv="$out/smoke$py"
  rm -rf "$venv" "$out/qual$py"
  uv venv -q -p "$py" "$venv"
  uv pip install -q --python "$venv/bin/python" "$wheel"
  pkg=$("$venv/bin/python" -c 'import pathlib, napl; print(pathlib.Path(napl.__file__).parent)')
  # No source tree is readable and the installed package is read-only: the wheel must stand alone.
  {
    printf '(version 1)(allow default)'
    for d in "$out/src" "$CHECKOUT" "${GITHUB_WORKSPACE:-}"; do
      [ -n "$d" ] && [ -d "$d" ] && printf '(deny file-read* (subpath "%s"))' "$(cd "$d" && pwd -P)"
    done
    printf '(deny file-write* (subpath "%s"))' "$(cd "$pkg" && pwd -P)"
  } > "$out/qual$py.sb"
  (cd "$out" && env -u NAPL_PROFILE -u PYTHONPATH -u NAPL_REPLAY_EXE /usr/bin/sandbox-exec -f "$out/qual$py.sb" \
    "$venv/bin/python" "$out/apple_wheel_smoke.py" "$wheel" --output "$out/qual$py" \
    --metadata "$out/build-metadata.json" --shm-client "$out/shm-smoke-client" ${extra[@]+"${extra[@]}"} \
    > "$out/qual$py.log" 2>&1) \
    || { tail -40 "$out/qual$py.log" >&2; die "qualification failed on Python $py"; }
  (cd /tmp && /usr/bin/sandbox-exec -f "$out/qual$py.sb" "$venv/bin/napl" --help > /dev/null) || die "napl --help failed"
  macos=$(sw_vers -productVersion)
  cp "$out/qual$py/qualification.json" "$out/dist/qualification-py$py-macos$macos.json"
  echo "qualified: Python $py, macOS $macos, $(sysctl -n machdep.cpu.brand_string) ${extra[*]:-}"
}

publish() {
  local out=$1 tag=$2 notes=$3 sha
  command -v gh > /dev/null || die "publish needs gh"
  [ -f "$notes" ] || die "release notes file not found: $notes"
  (cd "$out/dist" && shasum -a 256 -c SHA256SUMS) || die "SHA256SUMS mismatch"
  ls "$out"/dist/qualification-py3.12-*.json "$out"/dist/qualification-py3.13-*.json > /dev/null 2>&1 \
    || die "qualify on Python 3.12 and 3.13 before publishing"
  sha=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["source_sha"])' "$out/dist/build-metadata.json")
  git -C "$CHECKOUT" tag -a "$tag" "$sha" -m "$tag: napple (Apple Silicon) build of ${sha:0:12}"
  git -C "$CHECKOUT" push origin "$tag"
  gh release create "$tag" --verify-tag --prerelease --title "$tag (Apple Silicon pre-release)" \
    --notes-file "$notes" "$out"/dist/*
}

case "${1:-}" in
  all)
    ref=${2:-HEAD}
    out=${3:-$CACHE_ROOT/$(git -C "$CHECKOUT" rev-parse --verify "$ref^{commit}")}
    build "$ref" "$out"
    qualify "$out" 3.12 --visible
    qualify "$out" 3.13
    echo "release candidate: $out/dist"
    ;;
  build) [ $# -eq 3 ] || die "usage: build REF OUT"; build "$2" "$3" ;;
  qualify) [ $# -ge 3 ] || die "usage: qualify OUT PY [--visible | --no-render REASON]"; shift; qualify "$@" ;;
  publish) [ $# -eq 4 ] || die "usage: publish OUT TAG NOTES"; publish "$2" "$3" "$4" ;;
  *) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
