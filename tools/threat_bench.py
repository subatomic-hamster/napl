#!/usr/bin/env python3
"""Threat-suite benchmarks (docs/cuas2/threat-bench.md).

  OMP_NUM_THREADS=1 python tools/threat_bench.py track  --runs 50 --jobs 6 --json out/threat-track.json
  OMP_NUM_THREADS=1 python tools/threat_bench.py discrim --train 400 --test 150
  OMP_NUM_THREADS=1 python tools/threat_bench.py fusion --runs 50 --jobs 6

track: GOSPA / ID switches / breaks / ms per update of the tracker variants on every suite scene (paired seeds 1..runs).
discrim: logistic-regression class confusion on held-out seeds (train seeds 1..N, test seeds 10001..). fusion: camera vs camera + range."""

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from napl.threats import LABELS, NAMES
from napl.threats import bench as B
from napl.threats import discrim as D

VARIANTS = "ab-nn,imm2-gnn,imm2-gnn-s,imm3-gnn,imm2-jpda,imm2-mht,phd-singer"
SCENES = (
    ",".join(n for n in NAMES if not n.startswith("random"))
    + ",random,random_cam,close_cam_lidar"
)


def _cells(a, scs, vs):
    jobs = [(s, v, seed) for s in scs for v in vs for seed in range(1, a.runs + 1)]
    t0 = time.time()
    with Pool(a.jobs) as p:
        res = p.map(B.job, jobs, chunksize=2)
    return B.aggregate(res), time.time() - t0, len(jobs)


def cmd_track(a):
    scs, vs = a.scenes.split(","), a.variants.split(",")
    agg, wall, nj = _cells(a, scs, vs)
    print(
        f"{a.runs} seeded runs per cell (seeds 1..{a.runs}, paired across variants), {nj} runs in {wall:.0f} s on {a.jobs} processes.\n"
    )
    print("\n".join(B.table(agg, scs, vs)))
    if a.json:
        Path(a.json).write_text(
            json.dumps({f"{k[0]}/{k[1]}": v for k, v in agg.items()}, indent=1)
        )


def cmd_fusion(a):
    vs = a.variants.split(",")
    scs = ["close_cam", "close_cam_lidar"]
    agg, wall, nj = _cells(a, scs, vs)
    print(
        "| variant | camera GOSPA | +-95% | camera+range GOSPA | +-95% | breaks cam | breaks cam+range | ID sw cam | ID sw cam+range |\n|---|---|---|---|---|---|---|---|---|"
    )
    for v in vs:
        c, l = agg[("close_cam", v)], agg[("close_cam_lidar", v)]
        print(
            f"| {v} | {c['gospa']:.3f} | {c['gospa_ci95']:.3f} | {l['gospa']:.3f} | {l['gospa_ci95']:.3f} | {c['breaks']:.2f} | {l['breaks']:.2f} | {c['switches']:.2f} | {l['switches']:.2f} |"
        )
    print(f"\n{a.runs} seeds, {wall:.0f} s")


def cmd_discrim(a):
    tr, te = range(1, a.train + 1), range(10001, 10001 + a.test)
    for sg in (0.1, 0.5):
        data = (*D.dataset(tr, sg), *D.dataset(te, sg))
        for ab, drop in D.ABLATIONS.items():
            r = D.report(tr, te, sg, drop, data)
            C, nm = r["C"], LABELS
            print(
                f"\n#### position noise {sg} m, features: {ab}  (train {r['n_train']} objects, test {r['n_test']}, accuracy {r['acc']:.3f})\n"
            )
            print(
                "| true \\ predicted | "
                + " | ".join(nm)
                + " | recall |\n|---|"
                + "---|" * (len(nm) + 1)
            )
            for i, n in enumerate(nm):
                print(
                    f"| {n} | "
                    + " | ".join(str(x) for x in C[i])
                    + f" | {r['recall'][i]:.2f} |"
                )
            print(
                f"\nbird -> drone (costly): {r['bird_as_drone']} / {r['n_bird']} = {r['bird_as_drone'] / max(r['n_bird'], 1):.3f}; drone -> bird: {r['drone_as_bird']} / {r['n_drone']}"
            )


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    for n, f in (("track", cmd_track), ("fusion", cmd_fusion)):
        p = sub.add_parser(n)
        p.add_argument("--runs", type=int, default=50)
        p.add_argument("--jobs", type=int, default=6)
        p.add_argument("--scenes", default=SCENES)
        p.add_argument(
            "--variants",
            default=VARIANTS if n == "track" else "imm2-gnn,imm2-gnn-s,imm2-jpda",
        )
        p.add_argument("--json", default="")
        p.set_defaults(f=f)
    p = sub.add_parser("discrim")
    p.add_argument("--train", type=int, default=400)
    p.add_argument("--test", type=int, default=150)
    p.set_defaults(f=cmd_discrim)
    a = ap.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
