"""Model script of clearing.scenario.toml: the forest scene for the scenario's `seed` parameter (land_scene.py, the same value land_fly.py uses)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import land_scene


def build(out):
    return land_scene.build_for(out, __file__)
