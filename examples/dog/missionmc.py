"""Full-stack Monte Carlo of the dog per terrain family (SP6; spec s7: '>= 90 % no-fall per terrain family (MC n per family stated, with a 95 %
lower bound)', 'full autonomous coverage ... <= 20 min, map height RMS <= 5 cm, >= 95 % mapped'). One automission.py process per seed.
Verdicts are ground truth: a fall is summary['fell_ever'] (max of the ground-truth `fallen` column), a crash or a timeout stays in the denominator.
Seeds: TUNE 5001+ (any tuning, each change logged), HELD 33001+ (flown once with the final code; every row records the code revision).

    python examples/dog/missionmc.py --family all|rubble|slopes|scree|ruins --block tune|held --n 12 | rubble=20,slopes=12,... --jobs 4 --rev REV --out DIR

Writes DIR/<family>.jsonl (one row per run, appended as each finishes), DIR/mc.json and DIR/mc.md."""

import os

os.environ.setdefault(
    "OMP_NUM_THREADS", "1"
)  # MPC is 366 ms per solve with OpenMP threads, 6 ms with one (docs/dog/locomotion.md)
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse
import json
import statistics
import subprocess
import sys
import threading
import time
import tomllib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "skydio-land"))
from land_mc import wilson

FAMILIES = {
    "rubble": "fam_rubble",
    "slopes": "fam_slopes",
    "scree": "fam_scree",
    "ruins": "fam_ruins",
}
TUNE_BASE, HELD_BASE = (
    5001,
    33001,
)  # 31001+ (this plan's first choice) was taken by SP4's fresh terrain MC (31000-31039): moved
CRITERIA = [  # (key, text, op, target): the spec section 7 rows SP6 reports
    ("top_speed_flat", "flat top speed >= 2.0 m/s", ">=", 2.0),  # was 3.5: user decision 2026-10-18 (spec s7)
    ("rocky_speed", "rocky field mean speed >= 1.5 m/s", ">=", 1.5),
    ("slope_step", "25 deg slopes and 15 cm steps traversed", "bool", True),
    ("nofall", ">= 90 % no-fall per terrain family (Wilson 95 % lower bound shown)", ">=", 0.90),
    ("coverage_mission", "100 x 100 m rocky coverage <= 20 min, RMS <= 5 cm, >= 95 % mapped", "bool", True),
    ("bom_usd", "BOM about 3-4.5 k USD", "<=", 4500.0),
]  # fmt: skip
FAIL_KEYS = ("crashed", "fell", "coverage", "rms", "not_home", "late")


def seeds(block, n):
    base = {"tune": TUNE_BASE, "held": HELD_BASE}[block]
    return list(range(base, base + n))


def check_heldout_rev(rows):
    revs = {r["rev"] for r in rows}
    if len(revs) != 1:
        raise ValueError(
            f"held-out rows were flown with different code revisions {sorted(revs)}: rerun on a new seed block"
        )


def verdict(r, t_budget):
    """Reasons a run is not a mission success (empty = success). A run that errored has no other fields and is only `crashed`."""
    if "error" in r:
        return ["crashed"]
    out = []
    if r.get("fell_ever", r.get("fallen", 0)):
        out.append("fell")
    if not r["coverage"] >= 0.95:
        out.append("coverage")
    if not r["rms"] <= 0.05:
        out.append("rms")
    if r["phase"] != 4:
        out.append("not_home")
    if r["t"] > t_budget:
        out.append("late")
    return out


def summarize(rows, t_budget):
    if not rows:
        raise ValueError("no runs")
    why = [verdict(r, t_budget) for r in rows]
    n = len(rows)
    k_nf = sum("error" not in r and "fell" not in w for r, w in zip(rows, why))
    k_ok = sum(not w for w in why)
    ok = [r for r in rows if "error" not in r]
    med = (
        {
            c: statistics.median(r[c] for r in ok)
            for c in ("coverage", "rms", "t", "distance")
        }
        if ok
        else {}
    )
    return dict(n=n, k_nofall=k_nf, k_success=k_ok, wilson_nofall=wilson(k_nf, n), wilson_success=wilson(k_ok, n),
                fails={k: sum(k in w for w in why) for k in FAIL_KEYS}, medians=med, covs=sorted(r["coverage"] for r in ok))  # fmt: skip


def results_table(by_family, extra):
    """Markdown: one row per family (k/n, rate, Wilson interval, failure causes, medians), then one row per spec criterion with its measured value and
    MET / NOT MET / not measured. The no-fall row is MET only if the POINT rate is >= 0.90 for every family measured; a lower bound under 0.90 is stated."""
    L = ["| family | no-fall k/n | rate | Wilson 95 % | mission success k/n | failures (crashed fell coverage rms not_home late) | median coverage / RMS / time / distance |",
         "|---|---|---|---|---|---|---|"]  # fmt: skip
    for fam, s in by_family.items():
        lo, hi = s["wilson_nofall"]
        m = s["medians"]
        med = (
            f"{100 * m['coverage']:.1f} % / {100 * m['rms']:.1f} cm / {m['t']:.0f} s / {m['distance']:.0f} m"
            if m
            else "no completed run"
        )
        L.append(f"| {fam} | {s['k_nofall']}/{s['n']} | {s['k_nofall'] / s['n']:.2f} | [{lo:.2f}, {hi:.2f}] | {s['k_success']}/{s['n']} | "
                 + " ".join(str(s["fails"][k]) for k in FAIL_KEYS) + f" | {med} |")  # fmt: skip
    L += ["", "| spec criterion | measured | verdict |", "|---|---|---|"]
    L += [
        f"| {r['text']} | {r['measured']} | {r['verdict']} |"
        for r in criteria_rows(by_family, extra)
    ]
    return "\n".join(L) + "\n"


def criteria_rows(by_family, extra):
    """[{key, text, measured, verdict}] for every entry of CRITERIA: verdict is 'MET', 'NOT MET', 'MET on rate, lower bound under 0.90 for <families>' (no-fall only) or
    'not measured' (no value: extra[key] is None / absent, no family run). Shared by the Markdown table and the JSON the README, the video card and their tests read."""
    out = []
    for key, text, op, target in CRITERIA:
        meas, verdict = "", "not measured"
        if key == "nofall" and by_family:
            rate = {f: s["k_nofall"] / s["n"] for f, s in by_family.items()}
            meas = "; ".join(
                f"{f} {s['k_nofall']}/{s['n']} = {rate[f]:.2f}, lower bound {s['wilson_nofall'][0]:.2f}"
                for f, s in by_family.items()
            )
            met = all(v >= target for v in rate.values())
            low = [f for f, s in by_family.items() if s["wilson_nofall"][0] < target]
            verdict = ("MET" if met else "NOT MET") + (
                f" on rate, lower bound under {target:.2f} for {', '.join(low)}"
                if met and low
                else ""
            )
        elif key != "nofall" and extra.get(key) is not None:
            v = extra[key]
            met = (
                bool(v)
                if op == "bool"
                else (v >= target if op == ">=" else v <= target)
            )
            meas, verdict = (
                (str(v) if op == "bool" else format(v, "g")),
                ("MET" if met else "NOT MET"),
            )
        out.append(dict(key=key, text=text, measured=meas, verdict=verdict))
    return out


def results_json(by_family, extra, rev, missions=None):
    """The numbers docs/dog/missions.json carries: per-family summaries, the criteria verdicts, the 100 m mission rows, the code revision."""
    return dict(
        rev=rev,
        families=by_family,
        criteria=criteria_rows(by_family, extra),
        extra=extra,
        missions=missions or {},
    )


def scenario_path(family):
    name = FAMILIES[family]
    return HERE / "missions" / name / f"{name}.scenario.toml"


def one(family, seed, out, rev):
    """One full-stack run in its own process -> its summary.json fields + seed, rev, wall_s; a failed or timed-out process is {"seed", "error", ...}."""
    sc = scenario_path(family)
    d = Path(out) / f"seed{seed}"
    d.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "NAPL_PARAMS": json.dumps({"seed": seed})}
    wall = tomllib.loads(sc.read_text())["limits"]["wall_s"]
    t0 = time.perf_counter()
    try:
        r = subprocess.run(
            [sys.executable, str(HERE / "automission.py"), str(sc), str(d)],
            capture_output=True,
            text=True,
            env=env,
            timeout=wall,
        )
    except subprocess.TimeoutExpired:
        return {
            "seed": seed,
            "error": f"timeout after {wall} s wall",
            "rev": rev,
            "wall_s": time.perf_counter() - t0,
        }
    w = time.perf_counter() - t0
    if r.returncode != 0:
        return {"seed": seed, "error": r.stderr[-1500:], "rev": rev, "wall_s": w}
    return {
        "seed": seed,
        **json.loads((d / "summary.json").read_text()),
        "rev": rev,
        "wall_s": w,
    }


def parse_n(spec, fams):
    """'12' -> every family 12 runs; 'rubble=20,slopes=12' -> per family (each requested family must be listed)."""
    if "=" not in spec:
        return {f: int(spec) for f in fams}
    got = {k: int(v) for k, v in (p.split("=") for p in spec.split(","))}
    missing = [f for f in fams if f not in got]
    if missing:
        raise SystemExit(f"--n lacks {missing}")
    return {f: got[f] for f in fams}


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--family", default="all")
    ap.add_argument("--block", choices=("tune", "held"), required=True)
    ap.add_argument("--n", required=True)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument(
        "--rev",
        help="code revision (git rev-parse --short HEAD); required for --block held",
    )
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    fams = list(FAMILIES) if a.family == "all" else [a.family]
    if a.block == "held" and not a.rev:
        raise SystemExit(
            "--block held needs --rev: held-out rows record the code revision they were flown with"
        )
    ns, out = parse_n(a.n, fams), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.block == "held" and any((out / f"{f}.jsonl").exists() for f in fams):
        raise SystemExit(
            "held-out seeds fly once: a results file already exists (a changed code needs a new seed block, stated in docs/dog/missions.md)"
        )
    by, budgets, lock = {}, {}, threading.Lock()
    for fam in fams:
        budgets[fam] = tomllib.loads(scenario_path(fam).read_text())["auto"][
            "t_budget"
        ]["value"]
        rows, path = [], out / f"{fam}.jsonl"

        def run(seed, fam=fam, rows=rows, path=path):
            row = one(fam, seed, out / fam, a.rev or "")
            with (
                lock
            ):  # appended as each run finishes: a dropped connection keeps what ran
                rows.append(row)
                with open(path, "a") as f:
                    f.write(json.dumps(row) + "\n")
            return row

        with ThreadPoolExecutor(a.jobs) as ex:
            list(ex.map(run, seeds(a.block, ns[fam])))
        if a.block == "held":
            check_heldout_rev(rows)
        by[fam] = summarize(rows, budgets[fam])
    (out / "mc.json").write_text(
        json.dumps(
            dict(block=a.block, rev=a.rev, t_budget=budgets, families=by), indent=1
        )
    )
    md = f"# Dog full-stack MC, block {a.block}, rev {a.rev}\n\n" + results_table(
        by, {}
    )
    (out / "mc.md").write_text(md)
    print(md)  # ttsr-ignore: py/no-print


if __name__ == "__main__":
    main()
