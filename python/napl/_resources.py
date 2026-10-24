"""napl._resources — single source of truth for package-rooted paths.

Resolves paths to bundled data, executables, and the `napl-replay` binary
regardless of whether napl is:
  - installed from a wheel (importlib.resources / napl/data, napl/bin)
  - in a live source checkout  (parents[N] navigation still works)
  - in an editable install     (package files in source tree)

All callers should use these helpers instead of ``Path(__file__).parents[N]``.

Public surface
--------------
data_dir() -> Path          napl/data/ (bundled parts, assemblies, examples)
parts_dir() -> Path         napl/data/parts/
bin_dir() -> Path           napl/bin/  (napl-replay, napl-view, napl-shm-serve …)
replay_exe() -> Path        napl-replay binary (honours NAPL_REPLAY_EXE env var)
repo_root() -> Path | None  Source checkout root (None when installed from wheel)
"""

import os
from importlib.resources import files
from pathlib import Path

# ── Internal helpers ──────────────────────────────────────────────────────────

def _package_dir() -> Path:
    """Directory containing this file (napl/)."""
    return Path(__file__).resolve().parent


def _source_repo() -> "Path | None":
    """Return <repo> if this is an editable/source install, else None.

    In a source layout: napl/ lives at python/napl/, so the repo root is
    _package_dir().parents[1]  (python/napl → python → repo).
    A wheel places napl/ directly in site-packages; parents[1] is site-packages
    itself, which will not have CMakeLists.txt.
    """
    # parents[0] = python/   (wrapper dir)
    # parents[1] = repo/     (has CMakeLists.txt in source checkout)
    candidate = _package_dir().parents[1]
    if (candidate / "CMakeLists.txt").is_file():
        return candidate
    return None


# ── Public API ────────────────────────────────────────────────────────────────

def repo_root() -> "Path | None":
    """Source checkout root, or None when installed from a wheel."""
    return _source_repo()


def data_dir() -> Path:
    """Root of bundled data (parts, assemblies, examples).

    Wheel layout: napl/data/
    Source layout: <repo>/   (parts, assemblies, examples live at repo root)
    """
    # Editable/source imports must never accidentally use stale installed assets.
    src = _source_repo()
    if src is not None:
        return src
    wheel_data = Path(str(files("napl").joinpath("data")))
    if wheel_data.is_dir():
        return wheel_data

    raise FileNotFoundError("napl runtime data is missing; install a wheel with bundled assets")


def parts_dir() -> Path:
    """parts/ directory (bundled or checkout-relative)."""
    return data_dir() / "parts"


def bin_dir() -> Path:
    """napl/bin/ directory for bundled executables.

    Wheel layout: napl/bin/
    Source layout: <repo>/build/  (built artefacts land here)
    """
    pkg = _package_dir()
    wheel_bin = Path(str(files("napl").joinpath("bin")))
    if wheel_bin.is_dir():
        return wheel_bin

    src = _source_repo()
    if src is not None:
        return src / "build"

    return pkg / "bin"


def replay_exe() -> Path:
    """Path to napl-replay binary.

    Resolution order:
    1. NAPL_REPLAY_EXE environment variable (explicit override)
    2. napl/bin/napl-replay  (installed wheel)
    3. <repo>/build/napl-replay  (source / editable)

    Raises FileNotFoundError for a missing or non-executable override or default.
    """
    env_override = os.environ.get("NAPL_REPLAY_EXE")
    candidate = Path(env_override).expanduser().resolve() if env_override else bin_dir() / "napl-replay"
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return candidate

    # Not found: raise with an actionable message.
    raise FileNotFoundError(
        f"napl-replay not found at '{candidate}'. "
        "Set NAPL_REPLAY_EXE to its path, or install the napl wheel that bundles it."
    )
