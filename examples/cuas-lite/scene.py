"""Scene of the cuas-lite lockstep scenarios (the turret is assemblies/cuas-lite.toml; see
python/napl/lite/sim_io.py). `napl program` calls build(outdir) -> MJCF path; run.py imports TARGETS / SECTOR from here.

Scenario format: a `*.scenario.toml` with [model] script = "scene.py", [interfaces.native], [program] cmd = ["python", "{dir}/run.py"], and monitors on the
trace.csv columns run.py writes (see run.py). Targets are `Balloon(id, anchor, length, size_m, swing_deg, swing_az, kind)` pendulums; CUAS_SCENARIO picks the set."""

import os

from napl.cuas.scene import Environment
from napl.lite.fire import Sector
from napl.lite.sim_io import Balloon, build_scene

SCENARIOS = {
    "s1_balloon_static": [
        Balloon("b1", (5.0, -1.0, 3.0), 1.5),
        Balloon("b2", (8.0, 0.5, 3.2), 1.7),
        Balloon("b3", (10.0, 2.0, 3.3), 1.8),
    ],
    "smoke": [Balloon("b1", (6.0, 0.5, 3.3), 1.5)],
}
SECTOR = Sector(-60, 60, 0, 60)


def scenario():
    return os.environ.get("CUAS_SCENARIO", "s1_balloon_static")


def seed():
    return int(os.environ.get("CUAS_SEED", 1))


def build(out):
    return build_scene(
        out, SCENARIOS[scenario()], env=Environment(seed=seed()), seed=seed()
    )["model"]
