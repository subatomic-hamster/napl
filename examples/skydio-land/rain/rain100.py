"""Model script of rain100.scenario.toml: the scene is built by land_scene.py."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import land_scene


def build(out):
    return land_scene.build_for(out, __file__)
