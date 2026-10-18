"""[model] script of push.scenario.toml: that scenario's scene at its default parameters (dogcourse.build_for)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import dogcourse

build = dogcourse.build_for(Path(__file__).with_name("push.scenario.toml"))
