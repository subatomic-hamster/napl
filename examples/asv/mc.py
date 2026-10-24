"""Monte Carlo of the ASV encounters: every run draws its geometry (angles, speeds, offsets, meeting point, wave direction; encounters.py with jitter = 1) and its sea
state (Hs uniform 0.2 - 0.8 m) from its seed; the sensor noise, wave phases and AIS losses follow the same seed.

    python examples/asv/mc.py --seeds 30 --workers 5 --out runs/asv-mc.json [--kinds head_on overtaking ...] [--no-ais]

Seed blocks: the planner and controller constants were tuned on the seeds 5000 - 5199 (fastsim.py --fast runs and the first kernel Monte Carlos), so those numbers are
optimistic. The reported results are flown ONCE on a held-out block (--seed-base 20000, never used during tuning). --fast runs the same planner / tracker / judge in the
synthetic plant of fastsim.py (tuning only, never reported as a result); --noncomplying makes the traffic keep its course (Rule 17(a)(ii) test). A run SUCCEEDS when the mission completes (the last waypoint reached and
the true position within 8 m of it), there is no collision (centre distance to the traffic >= 1.2 m, to a fixed object >= 0.75 m) and the judgement of the encounter's Rule
holds (examples/asv/judge.py: closest approach >= 5 m, and the Rule-specific manoeuvre). The summary reports each of these separately, with 95 % Wilson intervals, and
the closest-approach and cross-track-error distributions as they came out. The truth cross-track error includes the GNSS bias (about 3.7 m RMS of position error).

Run it under the CPU lock with an outer timeout (workers <= 6): flock /workspace/wt/napl/.cpu.lock timeout 3600 python examples/asv/mc.py ..."""

import argparse
import json
import math
from napl.runtime import BatchPool
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from asv import encounters as enc

SEED_BASE = 5000  # --seed-base overrides it
HS_RANGE = (0.2, 0.8)
SCRATCH = Path(os.environ.get("NAPL_MC_TMP", "/tmp/napl-asv-mc"))
KEEP = (
    "ok",
    "mission_complete",
    "collision",
    "cpa_m",
    "colreg_ok",
    "side_ok",
    "cpa_ok",
    "xte_rms_m",
    "xte_max_m",
    "min_static_clear_m",
    "end_dist_m",
    "est_pos_err_rms_m",
    "est_hdg_err_rms_deg",
    "max_port_deg",
    "max_stbd_deg",
    "min_speed_ms",
    "t_end",
)


def hs_for(seed):
    """Significant wave height of one run, from its own stream."""
    return float(np.random.default_rng([int(seed), 2020]).uniform(*HS_RANGE))


def one(args):
    """One run -> dict of the metrics (picklable, for the pool). A crash counts as a failure and keeps its message."""
    kind, seed, ais, fast, complies = args
    hs = hs_for(seed)
    out = SCRATCH / f"w{os.getpid()}"
    shutil.rmtree(out, ignore_errors=True)
    r = dict(kind=kind, seed=int(seed), hs=hs)
    try:
        p = dict(
            seed=int(seed),
            hs=hs,
            jitter=1.0,
            ais=bool(ais),
            traffic_complies=bool(complies),
            visual=False,
        )
        if fast:  # the planner in the loop with a kinematic-dynamic boat (fastsim.py): for tuning, never for a claim
            from asv.fastsim import fly_fast

            res = fly_fast(kind, p)
        else:
            from asv.run import fly

            res = fly(kind, p, out, telemetry=False)["res"]
        r.update({k: res[k] for k in KEEP if k in res})
    except Exception:  # a crashed run is a failed run, with its reason
        r.update(ok=False, error=traceback.format_exc(limit=3)[-400:])
    finally:
        shutil.rmtree(out, ignore_errors=True)
    return r


def wilson(k, n, z=1.96):
    """95 % Wilson score interval of a proportion."""
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def q(x, f):
    x = [v for v in x if v is not None and np.isfinite(v)]
    return float(np.quantile(x, f)) if x else math.nan


def summarize(results):
    """-> dict of counts, rates (with intervals) and distributions of one set of runs of one kind."""
    n = len(results)
    cnt = lambda key: sum(1 for r in results if r.get(key))
    s = dict(n=n)
    for key, name in (
        ("ok", "success"),
        ("mission_complete", "mission_complete"),
        ("colreg_ok", "colreg_compliant"),
        ("collision", "collision"),
    ):
        k = cnt(key)
        lo, hi = wilson(k, n)
        s[name] = dict(k=k, rate=k / n if n else math.nan, lo=lo, hi=hi)
    cpa = [r.get("cpa_m") for r in results if r.get("cpa_m") is not None]
    s["cpa_m"] = dict(
        min=min(cpa) if cpa else math.nan, p10=q(cpa, 0.1), median=q(cpa, 0.5)
    )
    xt = [r.get("xte_rms_m") for r in results]
    s["xte_rms_m"] = dict(median=q(xt, 0.5), p90=q(xt, 0.9))
    s["xte_max_m"] = dict(
        median=q([r.get("xte_max_m") for r in results], 0.5),
        p90=q([r.get("xte_max_m") for r in results], 0.9),
    )
    s["min_static_clear_m"] = dict(
        min=q([r.get("min_static_clear_m") for r in results], 0.0)
    )
    s["errors"] = sum(1 for r in results if "error" in r)
    s["failures"] = [
        dict(seed=r["seed"], hs=round(r["hs"], 2))
        | {
            k: (round(v, 2) if isinstance(v, float) else v)
            for k, v in r.items()
            if k
            in (
                "mission_complete",
                "collision",
                "cpa_m",
                "colreg_ok",
                "side_ok",
                "end_dist_m",
                "error",
            )
        }
        for r in results
        if not r.get("ok")
    ]
    return s


def fmt(kind, s):
    f = lambda d: (
        f"{100 * d['rate']:.0f} % ({d['k']}/{s['n']}; {100 * d['lo']:.0f}-{100 * d['hi']:.0f})"
    )
    return (
        f"{kind:18s} success {f(s['success'])}  colreg {f(s['colreg_compliant'])}  collisions {s['collision']['k']}  "
        f"CPA min/p10/med {s['cpa_m']['min']:.1f}/{s['cpa_m']['p10']:.1f}/{s['cpa_m']['median']:.1f} m  "
        f"xte rms med/p90 {s['xte_rms_m']['median']:.1f}/{s['xte_rms_m']['p90']:.1f} m"
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seeds", type=int, default=30, help="runs per kind")
    ap.add_argument("--kinds", nargs="*", default=list(enc.KINDS))
    ap.add_argument(
        "--workers", type=int, default=None, help="upper bound; default follows the execution policy"
    )
    ap.add_argument(
        "--no-ais", action="store_true", help="traffic carries no AIS: lidar only"
    )
    ap.add_argument("--out", default="")
    ap.add_argument(
        "--seed-base",
        type=int,
        default=SEED_BASE,
        help="first seed (the seeds are base ... base + seeds - 1)",
    )
    ap.add_argument(
        "--fast",
        action="store_true",
        help="fastsim.py instead of the kernel: tuning only",
    )
    ap.add_argument(
        "--noncomplying",
        action="store_true",
        help="the traffic does not give way / turn (Rule 17(a)(ii) / (b) test of the stand-on vessel)",
    )
    a = ap.parse_args(argv)
    # Explicit workers are capped by native CPU/memory/thermal admission.
    jobs = [
        (k, a.seed_base + i, not a.no_ais, a.fast, not a.noncomplying)
        for k in a.kinds
        for i in range(a.seeds)
    ]
    t0 = time.time()
    SCRATCH.mkdir(parents=True, exist_ok=True)
    with BatchPool(a.workers) as pool:
        results = pool.map(one, jobs, chunksize=1)
    summary = {}
    for k in a.kinds:
        rs = [r for r in results if r["kind"] == k]
        summary[k] = summarize(rs)
        sys.stdout.write(fmt(k, summary[k]) + "\n")
        for f in summary[k]["failures"]:
            sys.stdout.write(f"    fail {f}\n")
    sys.stdout.write(f"{len(jobs)} runs in {time.time() - t0:.0f} s\n")
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(
            json.dumps(
                dict(
                    ais=not a.no_ais,
                    seeds=a.seeds,
                    seed_base=a.seed_base,
                    fast=a.fast,
                    traffic_complies=not a.noncomplying,
                    hs_range=HS_RANGE,
                    summary=summary,
                    results=results,
                ),
                indent=1,
                default=float,
            )
        )
    shutil.rmtree(SCRATCH, ignore_errors=True)
    return summary


if __name__ == "__main__":
    main()
