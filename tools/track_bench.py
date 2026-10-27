#!/usr/bin/env python3
"""Tracking benchmark: every napl.track variant over the textbook scenarios with seeded Monte Carlo (paired seeds).

  OMP_NUM_THREADS=1 python tools/track_bench.py --runs 100 --jobs 12 --out docs/cuas2/tracking-bench.md --json docs/cuas2/tracking-bench.json

Prints (and writes) one table per scenario: GOSPA (alpha=2, p=2, cut-off c in the scenario) with its localisation / missed / false
parts, ANEES (6 dof: p and v of matched truth/track pairs, first 10 matches of a pair skipped; ideal 6), track breaks, ID switches,
time to confirm, wall ms per tracker.step (a frame). All means are over runs; +- is the 95 % half-width of the GOSPA mean."""

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import numpy as np
from napl.track.scenarios import NAMES, build
from napl.track.variants import VARIANTS, run

KEYS = (
    "gospa",
    "loc",
    "missed",
    "false",
    "ospa",
    "nees",
    "breaks",
    "switches",
    "ttc",
    "lost",
    "cpu_ms",
)


def _job(a):
    sc, v, seed = a
    return sc, v, seed, run(sc, v, seed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=100)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--scenarios", default=",".join(NAMES))
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--out", default="")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    scs, vs = a.scenarios.split(","), a.variants.split(",")
    jobs = [(s, v, seed) for s in scs for v in vs for seed in range(1, a.runs + 1)]
    t0 = time.time()
    with Pool(a.jobs) as p:
        res = p.map(_job, jobs, chunksize=4)
    agg = {}
    for sc, v, seed, r in res:
        if r is not None:
            agg.setdefault((sc, v), []).append(r)
    out = {"runs": a.runs, "wall_s": round(time.time() - t0, 1), "results": {}}
    lines = [
        f"Tracking benchmark: {a.runs} seeded Monte Carlo runs per cell (seeds 1..{a.runs}, paired across variants), "
        f"{len(jobs)} runs in {out['wall_s']} s on {a.jobs} processes (OMP_NUM_THREADS=1).",
        "",
    ]
    for sc in scs:
        s = build(sc, 1)
        lines += [
            f"### {sc}  (targets {len(s.ids)}, dt {s.dt} s, {len(s.frames)} frames, Pd {s.pd}, clutter {s.n_clutter}/scan, GOSPA c = {s.c} m)",
            "",
            "| variant | GOSPA | +-95% | loc | missed | false | ANEES | breaks | ID sw | t_confirm s | not confirmed | ms/update |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for v in vs:
            rs = agg.get((sc, v))
            if not rs:
                lines.append(f"| {v} | n/a | | | | | | | | | | |")
                continue
            m = {k: float(np.nanmean([r[k] for r in rs])) for k in KEYS}
            ci = (
                1.96
                * float(np.std([r["gospa"] for r in rs], ddof=1))
                / np.sqrt(len(rs))
            )
            out["results"][f"{sc}/{v}"] = dict(m, gospa_ci95=ci, n=len(rs))
            lines.append(
                f"| {v} | {m['gospa']:.3f} | {ci:.3f} | {m['loc']:.3f} | {m['missed']:.3f} | {m['false']:.3f} | {m['nees']:.2f} | "
                f"{m['breaks']:.2f} | {m['switches']:.2f} | {m['ttc']:.2f} | {m['lost']:.2f} | {m['cpu_ms']:.2f} |"
            )
        lines.append("")
    text = "\n".join(lines)
    print(text)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(text + "\n")
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
