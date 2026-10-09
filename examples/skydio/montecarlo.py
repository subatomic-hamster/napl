"""Monte Carlo over forest seeds: python examples/skydio/montecarlo.py [--mission goal|follow|both] [--seeds 24] [--first 100] [--workers 5] [--out build/skydio_mc]

Every seed is a different forest (tree / pole / boulder placement, wall doorway, arch, goal), a different wind-turbulence draw and a different
sensor-noise draw. One process per run (a Sim is not shared). Writes <out>.json (every run's summary) and <out>.md (the table the README quotes).
Success: goal = reached within 45 s with zero contacts; follow = zero contacts, subject tracked >= 50 % of the flight, median follow distance
inside the 3-8 m band. Min clearance is the exact analytic surface distance minus the 0.40 m airframe sphere."""

import argparse
import json
import os
import statistics as st
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent


def one(args):
    mission, seed, root, cal = args
    out = Path(root) / f"{mission}-{seed}"
    r = subprocess.run(
        [sys.executable, str(HERE / "fly.py"), str(out), mission, str(seed)],
        capture_output=True,
        text=True,
        env={**os.environ, "SKYDIO_CAL": str(cal)},
    )
    if r.returncode:
        return dict(
            mission=mission,
            seed=seed,
            error=r.stderr.strip().splitlines()[-1:],
            success=False,
        )
    s = json.loads(r.stdout.strip().splitlines()[-1])
    if mission == "goal":
        s["success"] = bool(
            s["reached_goal"] and s["contact_steps"] == 0 and s["time_to_goal_s"] <= 45
        )
    else:
        s["success"] = bool(
            s["contact_steps"] == 0
            and s["subject_seen_frac"] >= 0.5
            and 3.0 <= s["follow_dist_median"] <= 8.0
        )
    return s


def table(rows, mission):
    ok = [r for r in rows if "error" not in r]
    n_ok = sum(r["success"] for r in rows)
    lines = [
        f"### {mission}: {n_ok}/{len(rows)} succeeded ({100 * n_ok / len(rows):.0f} %)",
        "",
    ]
    if ok:
        mean = lambda k: st.mean(r[k] for r in ok)
        lines += [
            "| metric | value |",
            "|---|---|",
            f"| runs | {len(rows)} ({len(rows) - len(ok)} crashed) |",
            f"| runs with a contact | {sum(r['contact_steps'] > 0 for r in ok)} |",
            f"| min clearance, worst run (m) | {min(r['min_clearance_m'] for r in ok):.2f} |",
            f"| min clearance, mean of runs (m) | {mean('min_clearance_m'):.2f} |",
            f"| mean speed in flight (m/s) | {mean('mean_speed_mps'):.2f} |",
            f"| 90th-percentile speed (m/s) | {mean('cruise_speed_p90_mps'):.2f} |",
            f"| max speed (m/s) | {max(r['max_speed_mps'] for r in ok):.2f} |",
        ]
        if mission == "goal":
            t = [r["time_to_goal_s"] for r in ok if r["time_to_goal_s"]]
            lines += [
                f"| time to goal, mean / worst (s) | {st.mean(t):.1f} / {max(t):.1f} |"
                if t
                else "| time to goal | none |"
            ]
        else:
            med = [r["follow_dist_median"] for r in ok if r.get("follow_dist_median")]
            q1, q2, q3 = np.percentile(med, [25, 50, 75]) if med else (0, 0, 0)
            strict = [
                r
                for r in ok
                if r.get("follow_band_min_max")
                and 3.0 <= r["follow_band_min_max"][0]
                and r["follow_band_min_max"][1] <= 8.0
            ]
            lines += [
                f"| runs inside the strict 3-8 m band for the whole walk (the scenario monitor) | {len(strict)} of {len(rows)} |",
                f"| follow distance, median of per-run medians / IQR (m) | {q2:.2f} / {q1:.2f}-{q3:.2f} |",
                f"| subject seen (fraction) | {mean('subject_seen_frac'):.2f} |",
            ]
            det = [r["detection"] for r in ok if "detection" in r]
            if det:
                lines += [
                    f"| detector recall on visible frames, mean | {st.mean(d['recall'] for d in det):.2f} |",
                    f"| detector false positives per frame, mean / worst run | {st.mean(d['fp_per_frame'] for d in det):.3f} / {max(d['fp_per_frame'] for d in det):.3f} |",
                ]
        est = [r["estimation"] for r in ok if "estimation" in r]
        if est:
            lines += [
                f"| estimate position error RMS, mean / worst run (m) | {st.mean(e['pos_rms_m'] for e in est):.2f} / {max(e['pos_rms_m'] for e in est):.2f} |",
                f"| estimate position error max, worst run (m) | {max(e['pos_max_m'] for e in est):.2f} |",
                f"| drift at the end, mean / worst (% of path) | {st.mean(e['drift_pct_of_path'] for e in est):.2f} / {max(e['drift_pct_of_path'] for e in est):.2f} |",
                f"| attitude error RMS, mean (deg) | {st.mean(e['att_rms_deg'] for e in est):.2f} |",
            ]
        lines += [
            f"| planner replan, mean / worst (ms) | {st.mean(r['planner']['mean_ms'] for r in ok):.0f} / {max(r['planner']['max_ms'] for r in ok):.0f} |"
        ]
    bad = [r["seed"] for r in rows if not r["success"]]
    lines += ["", f"failed seeds: {bad if bad else 'none'}", ""]
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default="both")
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--first", type=int, default=100)
    ap.add_argument("--workers", type=int, default=5)
    ap.add_argument("--out", default="build/skydio_mc")
    ap.add_argument(
        "--thin",
        action="store_true",
        help="thin-pole variant of every forest (forest.py)",
    )
    a = ap.parse_args()
    if a.thin:
        os.environ["SKYDIO_THIN"] = "1"  # inherited by the per-run processes
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    root, cal = Path(str(out) + "_runs"), Path(str(out) + "_cal")
    cal.mkdir(parents=True, exist_ok=True)
    missions = ["goal", "follow"] if a.mission == "both" else [a.mission]
    one(
        (missions[0], a.first - 1, root, cal)
    )  # one run first so the workers only read the thrust-curve cache
    jobs = [
        (m, s, root, cal) for m in missions for s in range(a.first, a.first + a.seeds)
    ]
    t0 = time.time()
    with ThreadPoolExecutor(a.workers) as ex:
        rows = list(ex.map(one, jobs))
    res = {m: [r for r in rows if r["mission"] == m] for m in missions}
    md = [
        "# Skydio Monte Carlo",
        "",
        f"{a.seeds} seeds per mission (first seed {a.first}), {time.time() - t0:.0f} s wall on {a.workers} workers.",
        "",
    ]
    for m in missions:
        md += table(res[m], m)
    Path(str(out) + ".json").write_text(json.dumps(res, indent=1))
    Path(str(out) + ".md").write_text("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
