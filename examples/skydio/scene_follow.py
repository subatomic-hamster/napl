"""Model script of follow.scenario.toml: the forest with the walking subject for the scenario's `seed` parameter (skydio_scene.seed, the same value fly.py uses)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import skydio_scene as scene  # not `scene`: another example's scene.py may already be in sys.modules


def build(out):
    return scene.build(out, "follow")
