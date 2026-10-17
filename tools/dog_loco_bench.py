"""Compute budget of the dog's 500 Hz loop (SP4 Task 1): physics alone, + the controller's reads, + MIT writes (constant / changing kp, kd).
Prints a Markdown table and the decision line for Task 1 Step 6 (> 1.0 ms of I/O per 2 ms control tick -> batch writes).
    python tools/dog_loco_bench.py --sim 2.0"""

import argparse
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples" / "dog"))
import dogcourse as C
import dogsim as D
import napl
from loco import hw as H


def run(mode, secs, tmp):
    sim = napl.Sim.load(C.scene(tmp, "flat", seed=1))
    hw = H.Hw(sim)
    hw.enable()
    q_hold = np.zeros(
        12
    )  # the build pose (straight legs): the I/O cost does not depend on the pose
    rng = np.random.default_rng(0)
    kp0, kd0 = np.full(12, D.KP), np.full(12, D.KD)
    hw.send(q_hold, np.zeros(12), kp0, kd0, np.zeros(12))
    n = int(secs / 0.002)
    t_read = t_write = 0.0
    t0 = time.perf_counter()
    for _ in range(n):
        if mode >= 1:
            a = time.perf_counter()
            hw.joints()
            hw.imu()
            t_read += time.perf_counter() - a
        if mode >= 2:
            a = time.perf_counter()
            j = rng.normal(0, 1e-3, 12)
            kp = kp0 * (1 + rng.uniform(0, 1e-3, 12)) if mode == 3 else kp0
            kd = kd0 * (1 + rng.uniform(0, 1e-3, 12)) if mode == 3 else kd0
            hw.send(q_hold + j, j, kp, kd, j)
            t_write += time.perf_counter() - a
        sim.step(2)
    wall = time.perf_counter() - t0
    return secs / wall, 1e6 * t_read / n, 1e6 * t_write / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", type=float, default=2.0)
    a = ap.parse_args()
    import tempfile

    rows = []
    names = [
        "(a) physics only, held stand",
        "(b) + Hw.joints() + Hw.imu()",
        "(c) + Hw.send, kp/kd constant",
        "(d) + Hw.send, kp/kd changing",
    ]
    for mode, name in enumerate(names):
        with tempfile.TemporaryDirectory() as tmp:
            rows.append((name, *run(mode, a.sim, tmp)))
    print(
        "| run | sim-s per wall-s | read us / tick | write us / tick |\n|---|---|---|---|"
    )
    for name, rt, r, w in rows:
        print(f"| {name} | {rt:.2f} | {r:.0f} | {w:.0f} |")
    io = (rows[2][2] + rows[2][3]) / 1e3
    print(f"\nI/O per control tick: {io:.2f} ms (rule: > 1.0 ms -> Step 6)")


if __name__ == "__main__":
    main()
