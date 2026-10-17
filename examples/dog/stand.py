"""Stand / squat sanity program for examples/dog/stand.scenario.toml ([interfaces.native]): straight legs -> squat -> stand on a
plain joint PD over the Qdd MIT-mode inputs. Writes trace.csv / replay.naplog / native.json for the runner."""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dogsim as D
import napl

PHASES = [
    (0.5, 1.0, 1.0),
    (1.5, 1.0, D.SQUAT),
    (1.0, D.SQUAT, D.SQUAT),
    (1.0, D.SQUAT, D.STAND),
    (1.0, D.STAND, D.STAND),
]  # (s, from, to)


def main():
    model, run = Path(os.environ["NAPL_MODEL"]), Path(os.environ["NAPL_RUN_DIR"])
    rep = json.loads((model.parent / f"{model.stem}.report.json").read_text())
    sim = napl.Sim.load(model)
    sim.start_log()
    dt, sg = D.timestep(model), D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    rows, z0 = [], None
    for T, a, b in PHASES:
        z0 = D.ramp(sim, sg, l1, l2, a, b, T, rows, z0, dt)
    D.finish(sim, run, rows, dt)


if __name__ == "__main__":
    main()
