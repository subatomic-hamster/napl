"""Scene of the counter-UAS demo: the turret on a 1 m mast plus four drones in the air at the swarm's start positions, one wind.

`napl program` calls build(outdir) -> Path of the MJCF (scenario [model] script). CUAS_SEED (env) seeds the swarm and the turbulence."""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from napl.cuas.scene import Environment, Vehicle, compose
from swarm import DEFAULT_SEED, Swarm

DRONES = [
    ("drone1", "x500v2"),
    ("drone2", "aos5"),
    ("drone3", "x500v2"),
    ("drone4", "x500v2"),
]
WIND = (
    3.0,
    -2.0,
    0.0,
)  # m/s steady (4 m/s from the south-west), plus Dryden turbulence below
MAST = 1.0  # m: the turret's pan axis height


def seed():
    return int(os.environ.get("CUAS_SEED", DEFAULT_SEED))


def build(out):
    start = Swarm([n for n, _ in DRONES], seed()).start()
    # CUAS_TURRET = path of the turret assembly to fly (default assemblies/turret.toml; the video flies the one built in the viewer)
    asm = os.environ.get("CUAS_TURRET", "turret")
    vs = [Vehicle("turret", asm, pos=(0.0, 0.0, MAST), pedestal=MAST)]
    vs += [Vehicle(n, a, pos=tuple(start[n]), yaw=0.0) for n, a in DRONES]
    return compose(out, vs, Environment(wind=WIND, w20=3.0, seed=seed()), seed=seed())[
        "model"
    ]
