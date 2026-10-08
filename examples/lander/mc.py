"""Monte Carlo of the TVC landing: wind, igniter delay, motor impulse scatter, thrust misalignment, sensor biases and noise, all seeded.

    python examples/lander/mc.py --train 60 --test 300 --workers 6 --out runs/lander-mc.json

Protocol (the point of keeping two seed ranges apart):
  1. TRAINING seeds (1000...) pick the one tunable landing-ignition `aim_bias_m` from a short list. It is the only thing tuned.
  2. TEST seeds (5000...) are then flown ONCE with that bias, and the success rate and the touchdown speed / position distributions are reported as they came out.
  3. The same test seeds are re-flown with one scatter changed at a time (igniter-delay jitter 0 / 0.02 / 0.08 s, motor impulse sigma 0 / 2 / 6.67 % = the NFPA 1125
     cap, wind fixed at 0 / 4 m/s) for a sensitivity table, bias unchanged.
Each flight draws its own wind (speed uniform 0..wind_max, direction uniform; the Dryden intensity `w20` of the environment device is the same speed) from the
seed, so the wind is part of the scatter. A flight is a success when it lands (not aborted) with all of: touchdown speed < 2 m/s, touchdown tilt < 10 deg, within
`MAX_DIST` of the pad, upright at the end, servo saturation < 10 % of the powered time (the monitors of land.scenario.toml).

Run it under the CPU lock with an outer timeout (workers <= 6): flock /workspace/wt/napl/.cpu.lock timeout 3600 python examples/lander/mc.py ..."""

import argparse
import json
import multiprocessing as mp
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

import land
import scene
from napl import params as nparams

MAX_DIST = 3.0  # m from the pad
MAX_SPEED = 2.0  # m/s
MAX_TILT = 10.0  # deg at touchdown
MAX_FINAL_TILT = 15.0  # deg at the end of the run
MAX_SAT = 0.10
AIM_CHOICES = (-0.6, -0.3, 0.0, 0.3, 0.6)  # m, the landing-ignition aim bias candidates
TRAIN_BASE, TEST_BASE = 1000, 5000
WIND_MAX = 4.0  # m/s
SCRATCH = Path(os.environ.get("NAPL_MC_TMP", "/tmp/napl-lander-mc"))


def wind_for(seed):
    """Wind speed, direction (deg) of one flight, from its own stream (independent of land.draw_hidden's)."""
    r = np.random.default_rng([int(seed), 7771])
    return float(r.uniform(0.0, WIND_MAX)), float(r.uniform(0.0, 360.0))


def one(args):
    """One flight -> dict of the metrics (picklable, for the pool)."""
    seed, over = args
    wind, wdir = wind_for(seed)
    o = dict(over)
    o.setdefault("wind_ms", wind)
    o.setdefault("wind_dir_deg", wdir)
    o["seed"] = int(seed)
    p = nparams.resolve(land.SCENARIO, o)
    out = SCRATCH / f"w{os.getpid()}"
    model = scene.build_seeded(
        out, seed, p["w20_ms"] if p["w20_ms"] >= 0 else p["wind_ms"]
    )
    r = land.fly(seed, p, model, log=False, telemetry=False)
    res, rows = r["res"], r["rows"]
    final_tilt = rows[-1][5] if rows else 99.0
    return dict(
        seed=int(seed),
        wind_ms=p["wind_ms"],
        landed=int(res["landed"]),
        aborted=int(res["state"] == "ABORT"),
        td_speed=float(res["td_speed"]),
        td_tilt=float(res["td_tilt"]),
        td_dist=float(res["td_dist"]),
        final_tilt=float(final_tilt),
        sat=float(res["sat_frac"]),
        apogee=float(res["apogee_m"]),
        diverged=int(res["diverged"]),
    )


def ok(r):
    return bool(
        r["landed"]
        and not r["aborted"]
        and r["td_speed"] < MAX_SPEED
        and r["td_tilt"] < MAX_TILT
        and r["td_dist"] < MAX_DIST
        and r["final_tilt"] < MAX_FINAL_TILT
        and r["sat"] < MAX_SAT
    )


def summarize(results):
    """-> dict: n, success rate (all criteria), per-criterion rates, quantiles of touchdown speed / distance, outcome counts."""
    n = len(results)
    if n == 0:
        raise ValueError("no results")
    sp = np.array([r["td_speed"] if r["landed"] else np.nan for r in results])
    di = np.array([r["td_dist"] if r["landed"] else np.nan for r in results])
    q = lambda a, p: float(np.nanpercentile(a, p)) if np.isfinite(a).any() else None
    frac = lambda f: sum(1 for r in results if f(r)) / n
    return dict(
        n=n,
        success=frac(ok),
        landed=frac(lambda r: r["landed"] and not r["aborted"]),
        speed_lt2=frac(lambda r: r["landed"] and r["td_speed"] < MAX_SPEED),
        speed_lt5=frac(lambda r: r["landed"] and r["td_speed"] < 5.0),
        tilt_lt10=frac(lambda r: r["landed"] and r["td_tilt"] < MAX_TILT),
        dist_lt3=frac(lambda r: r["landed"] and r["td_dist"] < MAX_DIST),
        upright=frac(lambda r: r["landed"] and r["final_tilt"] < MAX_FINAL_TILT),
        sat_lt10=frac(lambda r: r["sat"] < MAX_SAT),
        aborted=frac(lambda r: r["aborted"]),
        speed_ms=dict(p10=q(sp, 10), p50=q(sp, 50), p90=q(sp, 90), p99=q(sp, 99)),
        dist_m=dict(p10=q(di, 10), p50=q(di, 50), p90=q(di, 90), p99=q(di, 99)),
        speed_mean=float(np.nanmean(sp)) if np.isfinite(sp).any() else None,
    )


def run(pool, seeds, over):
    return pool.map(one, [(s, over) for s in seeds], chunksize=2)


def fmt(tag, s):
    sp, di = s["speed_ms"], s["dist_m"]
    return (
        f"{tag:<28} n {s['n']:4d}  success {100 * s['success']:5.1f}%  v<2 {100 * s['speed_lt2']:5.1f}%  v<5 {100 * s['speed_lt5']:5.1f}%  "
        f"tilt<10 {100 * s['tilt_lt10']:5.1f}%  dist<3 {100 * s['dist_lt3']:5.1f}%  upright {100 * s['upright']:5.1f}%  abort {100 * s['aborted']:4.1f}%  "
        f"speed p50/p90/p99 {sp['p50']:.2f}/{sp['p90']:.2f}/{sp['p99']:.2f}  dist p50/p90/p99 {di['p50']:.2f}/{di['p90']:.2f}/{di['p99']:.2f} m"
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--train", type=int, default=60, help="training seeds per aim-bias candidate"
    )
    ap.add_argument("--test", type=int, default=300, help="report seeds")
    ap.add_argument("--workers", type=int, default=6, help="processes (<= 6)")
    ap.add_argument("--out", default="")
    ap.add_argument(
        "--aim",
        type=float,
        default=None,
        help="skip the training and use this aim bias",
    )
    ap.add_argument(
        "--aims",
        default=",".join(str(x) for x in AIM_CHOICES),
        help="comma list of aim-bias candidates (m) for the training",
    )
    ap.add_argument(
        "--no-sweep",
        action="store_true",
        help="skip the igniter-jitter sensitivity table",
    )
    ap.add_argument(
        "--over",
        default="{}",
        help="JSON of scenario parameter overrides for every flight",
    )
    a = ap.parse_args(argv)
    base = json.loads(a.over)
    if not 1 <= a.workers <= 6:
        raise SystemExit("--workers must be 1..6 (shared box)")
    t0 = time.time()
    out = {
        "protocol": __doc__.split("\n\n")[1],
        "max_dist_m": MAX_DIST,
        "max_speed_ms": MAX_SPEED,
        "wind_max_ms": WIND_MAX,
    }
    with mp.get_context("spawn").Pool(a.workers) as pool:
        train = {}
        if a.aim is None:
            tr_seeds = range(TRAIN_BASE, TRAIN_BASE + a.train)
            aims = [float(x) for x in a.aims.split(",")]
            for aim in aims:
                s = summarize(run(pool, tr_seeds, {**base, "aim_bias_m": aim}))
                train[str(aim)] = s
                sys.stdout.write(fmt(f"train aim {aim:+.1f} m", s) + "\n")
                sys.stdout.flush()
            # best success rate, ties by the lower median speed
            aim = max(
                aims,
                key=lambda x: (
                    train[str(x)]["success"],
                    -(train[str(x)]["speed_ms"]["p50"] or 99),
                ),
            )
        else:
            aim = a.aim
        out["train"] = train
        out["aim_bias_m"] = aim
        sys.stdout.write(f"chosen aim bias {aim:+.1f} m\n")
        te_seeds = list(range(TEST_BASE, TEST_BASE + a.test))
        res = run(pool, te_seeds, {**base, "aim_bias_m": aim})
        out["test"] = summarize(res)
        out["test_flights"] = res
        sys.stdout.write(fmt("TEST (report seeds)", out["test"]) + "\n")
        sys.stdout.flush()
        out["sweep"] = {}
        if not a.no_sweep:
            cases = [
                ("delay_jitter_s", v, f"igniter jitter sigma {v:.2f} s")
                for v in (0.0, 0.02, 0.08)
            ]
            cases += [
                ("impulse_sigma", v, f"impulse sigma {100 * v:.2f} %")
                for v in (0.0, 0.02, 0.0667)
            ]
            cases += [("wind_ms", v, f"wind fixed {v:.0f} m/s") for v in (0.0, 4.0)]
            for key, v, tag in cases:
                s = summarize(run(pool, te_seeds, {**base, "aim_bias_m": aim, key: v}))
                out["sweep"][f"{key}={v}"] = s
                sys.stdout.write(fmt(tag, s) + "\n")
                sys.stdout.flush()
    out["wall_s"] = round(time.time() - t0, 1)
    shutil.rmtree(SCRATCH, ignore_errors=True)  # only this tool's own scene files
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(out, indent=1))
    sys.stdout.write(f"wall {out['wall_s']} s\n")
    return out


if __name__ == "__main__":
    main()
