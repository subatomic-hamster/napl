"""Monte Carlo of the dog's locomotion per terrain family (SP4, spec s7: '>= 90 % no-fall per terrain family (MC n per family stated, with a
95 % lower bound)', 'rocky field mean speed >= 1.5 m/s', '25-degree slopes and 15 cm steps traversed').
Per run, from its seed: a new 30 x 30 m field of the family (terrain seed = seed; criterion: the fixed course), a truth route of 15-20 m, plant
dispersion the controller does not know (payload 0-1 kg at +-3 / +-2 cm on the trunk, IMU gyro bias N(0, 0.005 rad/s) and accel bias N(0, 0.05
m/s^2) per axis, two lateral pushes of 0.2-0.5 m/s at random times), pose-fix noise. Lidar and depth camera are OFF (removed from the scene;
summarize() refuses a row that had them). Verdicts from GROUND TRUTH (dogtruth.Truth.fallen), never from the controller's own fall detector.
Seeds: TRAIN 1000... (any tuning), TEST 21000... (held out, flown once with the final code)."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse
import json
import multiprocessing as mp
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dogcourse
import dogwalk

FAMILIES = {  # family: (course, commanded speed m/s)
    "rubble": (
        "rubble",
        1.8,
    ),  # spec s7 rocky mean >= 1.5: commanded above it, the follower slows in turns
    "slopes": ("slopes", 1.0),
    "scree": ("scree", 1.2),
    "ruins": ("ruins", 1.2),
    "criterion": ("criterion", 0.8),
}
TRAIN_BASE, TEST_BASE = (
    1000,
    41000,
)  # 21000-21015 were spent on the flat push tuning (dogflat) in LOCO fix round 1; 31000-31039 were the round-1 test seeds, 41000+ are fresh for round 2
SCRATCH = Path(
    os.environ.get("NAPL_MC_TMP", Path(tempfile.gettempdir()) / "napl-dog-mc")
)


def wilson(
    k, n, z=1.96
):  # = examples/hopper/mc.py wilson (copied: importing it pulls the hopper's `hop` / `mc` module names into a shared worker)
    if n == 0:
        return (0.0, 1.0)
    p, d = k / n, 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, float(c - h)), min(1.0, float(c + h)))


def draw(seed, fam):
    rng = np.random.default_rng([int(seed), 4242])
    return dict(payload=(float(rng.uniform(0, 1.0)), float(rng.uniform(-0.03, 0.03)), float(rng.uniform(-0.02, 0.02))),
                gyro_bias=rng.normal(0, 0.005, 3).tolist(), accel_bias=rng.normal(0, 0.05, 3).tolist(),
                pushes=[(float(t), (0.0, float(s * m), 0.0)) for t, s, m in zip(rng.uniform(4, 12, 2), rng.choice([-1, 1], 2), rng.uniform(0.2, 0.5, 2))])  # fmt: skip


def route_length(model):
    r = np.asarray(dogcourse.info(model)["route"], float)
    return float(np.linalg.norm(np.diff(r, axis=0), axis=1).sum())


def one(args):
    seed, fam, over = args
    course, v = FAMILIES[fam]
    h = draw(seed, fam)
    if over.get(
        "no_disp"
    ):  # diagnostic variant: the terrain alone (no payload, IMU bias or pushes); not the spec MC
        h = dict(payload=None, gyro_bias=[0.0] * 3, accel_bias=[0.0] * 3, pushes=[])
    SCRATCH.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(dir=SCRATCH, prefix=f"w{os.getpid()}-"))
    try:
        model = dogcourse.scene(out, course, seed, payload=h["payload"])
    except RuntimeError as e:  # dogcourse.route: no 15-20 m truth route on this field (a property of the field, not of the controller)
        return dict(
            seed=int(seed), fam=fam, leg_ctl=over.get("leg_ctl", "wbc"), skipped=str(e)
        )
    length = route_length(model)
    v = over.get("v_cmd", v)
    p = dict(dogwalk.DEFAULTS, seed=seed, course=course, v_cmd=v, duration=over.get("duration", 3.0 + 2.5 * length / v), pushes=h["pushes"],
             imu_bias=(h["gyro_bias"], h["accel_bias"]), leg_ctl=over.get("leg_ctl", "wbc"), h_frac=over.get("h_frac"))  # fmt: skip
    r = dogwalk.run(model, p, log=False)["res"]
    return dict(seed=int(seed), fam=fam, leg_ctl=p["leg_ctl"], route_length=length, **r)


def ok(r):
    return r["reached"] and not (r["fallen"] or r["fault"] or r["diverged"])


def why(r):
    return (
        "fall"
        if r["fallen"]
        else "fault"
        if r["fault"]
        else "diverged"
        if r["diverged"]
        else "timeout"
        if not r["reached"]
        else "ok"
    )


def summarize(rows):
    n_skip = sum(
        "skipped" in r for r in rows
    )  # fields with no route: not a controller outcome, counted and reported
    rows = [r for r in rows if "skipped" not in r]
    if not rows:
        raise ValueError("no runs")
    if any(r["rangefinders"] for r in rows):
        raise ValueError(
            "a run had the lidar / depth camera on: locomotion MC runs without rays (dogcourse.strip_lidar)"
        )
    n, k_s, k_nf = len(rows), sum(map(ok, rows)), sum(not r["fallen"] for r in rows)
    fails = {}
    for r in rows:
        if not ok(r):
            fails[why(r)] = fails.get(why(r), 0) + 1
    sp = [r["mean_speed"] for r in rows if ok(r)]
    return dict(n=n, n_no_route=n_skip, k_success=k_s, k_nofall=k_nf, wilson_success=wilson(k_s, n), wilson_nofall=wilson(k_nf, n), failures=fails,
                speed_mean=float(np.mean(sp)) if sp else 0.0, speed_median=float(np.median(sp)) if sp else 0.0,
                slips=int(sum(r["n_slip"] for r in rows)), mpc_fail=int(sum(r["mpc_fail"] for r in rows)))  # fmt: skip


def write_md(out, data):
    """<out>/locomotion-mc.md from data = {"cmd", "rev", "leg", "base", "families": {fam: summarize(rows)}}."""
    f = lambda w: f"{100 * w[0]:.0f}-{100 * w[1]:.0f} %"
    L = [f"# Dog locomotion Monte Carlo ({data['leg']} leg controller, seeds {data['base']}...)", "",
         f"Command: `{data['cmd']}`; code revision `{data['rev']}`.", "",
         "## How to read", "",
         "- Verdicts come from GROUND TRUTH (`dogtruth.Truth.fallen`: tilt / body clearance of the sim), never from the controller's own fall detector.",
         "- Seeds: the held-out block starts at 21000 (no tuning used it); the tuning block starts at 1000. Each seed is a new 30 x 30 m field of the family, "
         "a truth A* route of 15-20 m (the criterion family: the fixed 24 m course), a payload of 0-1 kg, IMU biases, two lateral pushes of 0.2-0.5 m/s, and pose-fix noise.",
         "- Lidar and depth camera are off; the height map is the ground truth (SP5 stand-in); routes are ground-truth A* (SP5 plans its own).",
         "- success = route end reached within 3 s + 2.5 x length / commanded speed, no fall, no driver fault, no divergence. No-fall = the truth never fell "
         "(a run that timed out upright counts as no-fall). The number to compare with the 90 % criterion is the Wilson 95 % LOWER bound of no-fall.",
         "- Speeds are the mean speed over the route of the successful runs only.",
         "- A seed whose field has no 15-20 m truth route (`dogcourse.route` gives up after 50 draws) is not flown and is not counted in n (shown as 'no route'): it says nothing about the controller.", "",
         "| family | n | success k/n (Wilson 95 %) | no-fall k/n (Wilson 95 %) | no-fall lower bound | speed mean / median m/s (successful) | failures | slips | MPC failures |",
         "|---|---|---|---|---|---|---|---|---|"]  # fmt: skip
    for fam, s in data["families"].items():
        L.append(f"| {fam} | {s['n']}" + (f" (+{s['n_no_route']} no route)" if s["n_no_route"] else "") + f" | {s['k_success']}/{s['n']} ({f(s['wilson_success'])}) | {s['k_nofall']}/{s['n']} ({f(s['wilson_nofall'])}) | "
                 f"{100 * s['wilson_nofall'][0]:.0f} % | {s['speed_mean']:.2f} / {s['speed_median']:.2f} | {s['failures'] or '-'} | {s['slips']} | {s['mpc_fail']} |")  # fmt: skip
    (Path(out) / "locomotion-mc.md").write_text("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--family", default="all", help="all | " + " | ".join(FAMILIES))
    ap.add_argument("--leg", default="wbc", choices=["wbc", "jt"])
    ap.add_argument(
        "--train", type=int, default=0, help="N runs on the tuning seeds (1000...)"
    )
    ap.add_argument(
        "--test", type=int, default=0, help="N runs on the held-out seeds (21000...)"
    )
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--over", default="{}", help="JSON overrides: v_cmd, duration")
    ap.add_argument("--rev", default="unknown")
    ap.add_argument("--out", default="build/dog-mc")
    a = ap.parse_args()
    fams = list(FAMILIES) if a.family == "all" else [a.family]
    over = {**json.loads(a.over), "leg_ctl": a.leg}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for base, n in ((TRAIN_BASE, a.train), (TEST_BASE, a.test)):
        if not n:
            continue
        todo = [(base + i, fam, over) for fam in fams for i in range(n)]
        part = (
            out / f"rows-{a.leg}-{base}.jsonl"
        )  # each finished run is appended: a crash loses one run, a rerun resumes
        rows = (
            [json.loads(x) for x in part.read_text().splitlines()]
            if part.exists()
            else []
        )
        done = {(r["fam"], r["seed"]) for r in rows}
        todo = [t for t in todo if (t[1], t[0]) not in done]
        with mp.Pool(a.jobs) as pool, part.open("a") as fh:
            for r in pool.imap_unordered(one, todo, chunksize=1):
                rows.append(r)
                fh.write(json.dumps(r) + "\n")
                fh.flush()
        rows = [r for r in rows if r["fam"] in fams]
        data = dict(cmd=" ".join(sys.argv), rev=a.rev, leg=a.leg, base=base,
                    families={fam: summarize([r for r in rows if r["fam"] == fam]) for fam in fams}, rows=rows)  # fmt: skip
        tag = "" if base == TEST_BASE else "-train"
        (out / f"locomotion-mc{tag}.json").write_text(json.dumps(data, indent=1))
        if not tag:
            write_md(out, data)
        sys.stdout.write(
            json.dumps({k: v for k, v in data["families"].items()}, indent=1) + "\n"
        )


if __name__ == "__main__":
    main()
