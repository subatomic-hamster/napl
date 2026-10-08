"""Model script of goal.scenario.toml: the forest / wall / arch scene for SKYDIO_SEED (default 1)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import scene


def build(out):
    return scene.build(out, "goal")
