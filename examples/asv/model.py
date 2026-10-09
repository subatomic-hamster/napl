"""[model] script of the ASV scenarios (napl program loads it as a plain file, so the package import is done by path): build(out) -> the default scene.xml.
The programs (asv/run.py) rebuild the scene of their own encounter / seed / sea state into <run dir>/model, which is also what the replay uses."""

import sys
from pathlib import Path

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1])
)  # examples/ : the package `asv`

from asv.scene import build  # noqa: F401
