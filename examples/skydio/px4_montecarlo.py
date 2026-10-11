"""Monte Carlo of the PX4 path: python examples/skydio/px4_montecarlo.py [--mission goal|follow|both] [--seeds 12] [--first 100] [--workers 2] [--out build/skydio_px4_mc]

One `napl program examples/skydio/<mission>.px4.scenario.toml` per seed (forest, wind, sensor noise: the same seeds as montecarlo.py, so the native table is
the comparison), on its own PX4 instance (NAPL_PX4_INSTANCE = 5 + worker: tcp 4565+, udp 14545+...). Every flight is ~3-4 min of wall clock (PX4 boots in real
time, the flight is lockstep at speed 1.0), so the default is a modest 12 seeds and 2 workers. Success is montecarlo.py's definition (goal: reached within 45 s,
zero contacts; follow: zero contacts, subject tracked >= 50 %, median follow distance in 3-8 m) AND the PX4 flight completed (the program exited 0).
The report adds Wilson 95 % intervals on the success rate and the PX4-specific rows (EKF2 vs the companion's estimate, scenario monitors passed, replay match).
Run it under the shared CPU lock (tools/remote.sh bg)."""

import argparse
import json
import math
import os
import statistics as st
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from queue import Queue

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import montecarlo as native

EXTRA_ENV = {}  # --env / --param: added to every flight's environment


def wilson(k, n, z=1.96):
    """Wilson score interval of k successes in n trials."""
    if n == 0:
        return 0.0, 0.0
    p, d = k / n, 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def one(job):
    mission, seed, root, instances = job
    inst = instances.get()
    try:
        runs = Path(root) / f"{mission}-{seed}"
        env = {
            **os.environ,
            **EXTRA_ENV,
            "NAPL_PARAMS": json.dumps({"seed": seed}),
            "NAPL_PX4_INSTANCE": str(inst),
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
        }
        t0 = time.time()
        r = subprocess.run([str(Path(sys.executable).parent / "napl"), "program", str(HERE / f"{mission}.px4.scenario.toml"), "--runs-dir", str(runs)],
                           capture_output=True, text=True, env=env, timeout=1500)  # fmt: skip
        d = next(iter(sorted(runs.glob("*-*"))), None)
        out = dict(mission=mission, seed=seed, wall_s=time.time() - t0, rc=r.returncode)
        res = (
            json.loads((d / "result.json").read_text())
            if d and (d / "result.json").exists()
            else None
        )
        if res is None or not (d / "summary.json").exists():
            return {
                **out,
                "error": (
                    r.stderr.strip().splitlines()
                    or r.stdout.strip().splitlines()
                    or ["no result"]
                )[-1:],
                "success": False,
                "contact_steps": 0,
            }
        s = json.loads((d / "summary.json").read_text())
        out.update(s)
        out["monitors_passed"] = sum(m["pass"] for m in res["monitors"])
        out["monitors"] = len(res["monitors"])
        out["monitors_failed"] = [m["name"] for m in res["monitors"] if not m["pass"]]
        out["replay_match"] = bool(res["replay"].get("match"))
        done = res["program"]["returncode"] == 0
        if mission == "goal":
            out["success"] = bool(
                done
                and s["reached_goal"]
                and s["contact_steps"] == 0
                and s["time_to_goal_s"] <= 45
            )
        else:
            out["success"] = bool(
                done
                and s["contact_steps"] == 0
                and s["subject_seen_frac"] >= 0.5
                and 3.0 <= s["follow_dist_median"] <= 8.0
            )
        return out
    except (
        Exception
    ) as e:  # a hung / killed run is a failed run, not a dead Monte Carlo
        return dict(
            mission=mission, seed=seed, error=[repr(e)], success=False, contact_steps=0
        )
    finally:
        instances.put(inst)


def px4_rows(rows):
    ok = [r for r in rows if "error" not in r]
    n = len(rows)
    k = sum(r["success"] for r in rows)
    lo, hi = wilson(k, n)
    lines = [
        f"| success, Wilson 95 % interval | {k}/{n} = {100 * k / n:.0f} % [{100 * lo:.0f}, {100 * hi:.0f}] % |"
    ]
    if ok:
        e = [r["px4_vs_companion"] for r in ok if "px4_vs_companion" in r]
        lines += [
            f"| scenario monitors all passed (runner verdict) | {sum(r['monitors_passed'] == r['monitors'] for r in ok)} of {len(ok)} |",
            f"| replay hash matches the run's own | {sum(r['replay_match'] for r in ok)} of {len(ok)} |",
            f"| wall time per run, mean (s) | {st.mean(r['wall_s'] for r in ok):.0f} |",
        ]
        if e:
            lines += [
                f"| PX4 EKF2 vs the companion estimate, position RMS mean / worst run (m) | {st.mean(x['pos_rms_m'] for x in e):.2f} / {max(x['pos_rms_m'] for x in e):.2f} |"
            ]
        fails = {r["seed"]: r["monitors_failed"] for r in ok if r["monitors_failed"]}
        if fails:
            lines += [f"| failed monitors by seed | {fails} |"]
    errs = {r["seed"]: r["error"] for r in rows if "error" in r}
    if errs:
        lines += [f"| runs that did not finish | {errs} |"]
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default="both")
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--first", type=int, default=100)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--out", default="build/skydio_px4_mc")
    ap.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="NAME=VAL",
        help="PX4 parameter on top of the scenario's (tuning sweeps; NAPL_PX4_PARAMS)",
    )
    ap.add_argument(
        "--env",
        action="append",
        default=[],
        metavar="NAME=VAL",
        help="environment variable for the flights (companion knobs, e.g. SKYDIO_LEAD_S)",
    )
    a = ap.parse_args()
    EXTRA_ENV.update(e.split("=", 1) for e in a.env)
    if a.param:
        EXTRA_ENV["NAPL_PX4_PARAMS"] = json.dumps(
            {k: float(v) for k, v in (p.split("=", 1) for p in a.param)}
        )
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    root = Path(str(out) + "_runs")
    missions = ["goal", "follow"] if a.mission == "both" else [a.mission]
    instances = Queue()
    for i in range(a.workers):
        instances.put(
            int(os.environ.get("NAPL_PX4_INSTANCE", 5)) + i
        )  # first instance NAPL_PX4_INSTANCE (default 5); instances 0-9 only: PX4's rcS points the offboard link of every instance above 9 at udp 14549 (not 14540 + n)
    jobs = [
        (m, s, root, instances)
        for m in missions
        for s in range(a.first, a.first + a.seeds)
    ]
    t0 = time.time()
    with ThreadPoolExecutor(a.workers) as ex:
        rows = list(ex.map(one, jobs))
    res = {m: [r for r in rows if r["mission"] == m] for m in missions}
    md = [
        "# Skydio Monte Carlo on PX4 SITL",
        "",
        f"{a.seeds} seeds per mission (first seed {a.first}), {time.time() - t0:.0f} s wall on {a.workers} workers.",
        "",
    ]
    for m in missions:
        t = native.table(res[m], m)
        i = next((j for j, ln in enumerate(t) if ln.startswith("| metric")), None)
        md += (
            t[: i + 2] + px4_rows(res[m]) + t[i + 2 :]
            if i is not None
            else t + px4_rows(res[m])
        )
    Path(str(out) + ".json").write_text(json.dumps(res, indent=1))
    Path(str(out) + ".md").write_text("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
