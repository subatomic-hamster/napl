#!/usr/bin/env python3
"""Tuning harness for napl.track trackers: paired-seed Monte Carlo of one or more specs on chosen scenarios.

  OMP_NUM_THREADS=1 python tools/track_tune.py --runs 40 --jobs 2 --scenarios cross,swarm5 \
      imm2-jpda  imm2-jpda:pd=0.95,lamx=3  imm2-mht:kmax=8,nscan=5

A spec is BASE[:key=val,...]. BASE is a variants.VARIANTS name. Keys go to the tracker constructor (e.g. m, n, max_miss, kmax,
nscan, prune_gap, score_birth, w_birth, ps ...), except pdx / lamx which replace the pd / multiply the clutter density the tracker is
told (the scenario itself is never changed). Prints GOSPA, ID switches, breaks, false, missed, ms/update per scenario and spec."""

import argparse
import os
import sys
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import numpy as np
from napl.track import variants as V
from napl.track.scenarios import NAMES, build


def parse(spec):
    base, _, kv = spec.partition(":")
    kw = {}
    for p in filter(None, kv.split(",")):
        k, v = p.split("=")
        kw[k] = float(v) if "." in v or "e" in v else int(v)
    return base, kw


def _job(a):
    sc, spec, seed = a
    base, kw = parse(spec)
    pdx, lamx = kw.pop("pdx", None), kw.pop("lamx", 1.0)
    scn = build(sc, seed)
    cls, mk = V.factory(base, seed)
    kw = {**V.KW.get(base, {}), **kw}
    tr = cls(mk, pd=pdx if pdx is not None else scn.pd, lam=scn.lam * lamx, **kw)
    ev, dts = V.Evaluator(scn.c), []
    for k, (t, meas, zs) in enumerate(scn.frames):
        t0 = V.time.perf_counter()
        tracks = tr.step(t, zs, meas)
        dts.append(V.time.perf_counter() - t0)
        ev.add(t, scn.ids, scn.truth[k], tracks)
    s = ev.summary(scn.ids)
    s["cpu_ms"] = 1e3 * float(np.mean(dts))
    return sc, spec, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("specs", nargs="+")
    ap.add_argument("--runs", type=int, default=40)
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--scenarios", default=",".join(n for n in NAMES if n != "cam"))
    a = ap.parse_args()
    scs = a.scenarios.split(",")
    jobs = [
        (s, sp, seed) for s in scs for sp in a.specs for seed in range(1, a.runs + 1)
    ]
    with Pool(a.jobs) as p:
        res = p.map(_job, jobs, chunksize=2)
    agg = {}
    for sc, sp, r in res:
        agg.setdefault((sc, sp), []).append(r)
    for sc in scs:
        print(f"## {sc}  ({a.runs} seeds)")
        print(f"{'spec':44s} GOSPA   +-     loc   missed false  breaks  IDsw   ms")
        for sp in a.specs:
            rs = agg[(sc, sp)]
            m = {
                k: float(np.nanmean([r[k] for r in rs]))
                for k in (
                    "gospa",
                    "loc",
                    "missed",
                    "false",
                    "breaks",
                    "switches",
                    "cpu_ms",
                )
            }
            ci = (
                1.96
                * float(np.std([r["gospa"] for r in rs], ddof=1))
                / np.sqrt(len(rs))
            )
            print(
                f"{sp:44s} {m['gospa']:.3f} {ci:.3f}  {m['loc']:.3f} {m['missed']:.3f} {m['false']:.3f} "
                f"{m['breaks']:6.2f} {m['switches']:6.2f} {m['cpu_ms']:6.2f}"
            )
        print()


if __name__ == "__main__":
    main()
