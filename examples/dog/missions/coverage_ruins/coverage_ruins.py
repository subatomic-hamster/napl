"""Model script of coverage_ruins.scenario.toml: the carrier scene is built by autonomy/scene.py."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from autonomy import scene


def build(out):
    return scene.build_for(out, __file__)
