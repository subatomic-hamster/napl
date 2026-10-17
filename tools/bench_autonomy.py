"""SP5 sim budget: sim-seconds per wall-second of the kinematic carrier (rosette lidar + RTK GNSS, no legs) driving 1.5 m/s with a
slow turn over each 100 x 100 m terrain kind, per rays/frame and patch size. Kernel only (the autonomy's Python is measured by
the missions, Task 9). Run on hfbox, one job at a time:

    .venv/bin/python tools/bench_autonomy.py [--kinds ...] [--rays 2500,5000,10000,20000] [--patch 10,5] [--sim 5.0]"""

import argparse
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
from autonomy import carrier, config
from napl.terrain import field

START = {"ruins": (2.5, 2.5, 0.0)}  # a ruins cell centre (walls on the 5 m grid)


def run(kind, rays, patch, sim_s, tmp):
    cfg = config.load()
    f = field(kind, seed=3, patch=patch)
    start = START.get(kind, (0.0, 0.0, 0.0))
    s = napl.Sim.load(carrier.build(f, cfg, rays, start, tmp))
    c = carrier.Carrier(s, f, cfg, start)
    c.command(1.5, 0.0, 0.1)
    c.advance(0.5)  # first frames (lazy setup)
    t0 = time.perf_counter()
    c.advance(sim_s)
    return sim_s / (time.perf_counter() - t0)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--kinds", default="rubble,slopes,scree,ruins")
    a.add_argument("--rays", default="2500,5000,10000,20000")
    a.add_argument("--patch", default="10,5")
    a.add_argument("--sim", type=float, default=5.0)
    a = a.parse_args()
    rays = [int(r) for r in a.rays.split(",")]
    print("| kind | patch | " + " | ".join(f"{r / 1000:g}k" for r in rays) + " |")
    print("|---|---|" + "---|" * len(rays))
    with tempfile.TemporaryDirectory() as tmp:
        for kind in a.kinds.split(","):
            for patch in (float(p) for p in a.patch.split(",")):
                row = [f"{run(kind, r, patch, a.sim, tmp):.2f}" for r in rays]
                print(f"| {kind} | {patch:g} m | " + " | ".join(row) + " |", flush=True)


if __name__ == "__main__":
    main()
