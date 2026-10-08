"""Model script of follow.scenario.toml: the forest with the walking subject for SKYDIO_SEED (default 1)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import scene


def build(out):
    return scene.build(out, "follow")
