"""Monte Carlo of a landing scenario: one flight per seed (land_fly.py in its own process), the success definition of land_fly.finish (landed, no obstacle contact, no
crash, touchdown <= 1.0 m/s, tilt <= 12 deg, within 1.5 m of a truth-safe zone centre, final tilt < 25 deg), a Wilson interval, and where it fails.

    python examples/skydio-land/land_mc.py <scenario.scenario.toml> --seeds 1001:1013 --jobs 4 --out build/mc/forest [--set wind_ms=4 --set rain_mm_h=100]

DEV seeds (used while tuning the autonomy and the thresholds): 1-30. HELD-OUT seeds (never flown before the numbers in README.md were taken): 1001 and up.
`--set key=value` overrides a scenario parameter (NAPL_PARAMS) for the whole batch: the envelope sweeps are batches at one wind / rain level each."""

import argparse
import json
import math
import os
import subprocess
import sys
from napl.runtime import BatchPool, child_environment
from pathlib import Path

HERE = Path(__file__).resolve().parent


def wilson(k, n, z=1.96):
    """Wilson score interval of a binomial proportion -> (lo, hi)."""
    if n == 0:
        return 0.0, 1.0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def fly(scenario, seed, out, overrides, cal):
    """One flight in a subprocess -> its summary dict (a failed process counts as a failure with the tail of its stderr)."""
    run = Path(out) / f"seed{seed}"
    run.mkdir(parents=True, exist_ok=True)
    env = child_environment({"NAPL_PARAMS": json.dumps({**overrides, "seed": seed})})
    env.pop("NAPL_MODEL", None)
    if cal:
        env["SKYDIO_CAL"] = str(cal)
    p = subprocess.run(
        [sys.executable, str(HERE / "land_fly.py"), str(scenario), str(run)],
        env=env,
        capture_output=True,
        text=True,
    )
    f = run / "summary.json"
    if p.returncode != 0 or not f.exists():
        return dict(seed=seed, success=False, landed=False, error=p.stderr[-400:])
    s = json.loads(f.read_text())
    for big in ("assessments", "zones_truth"):
        s.pop(big, None)
    return s


def summarize(rows):
    n = len(rows)
    k = sum(bool(r.get("success")) for r in rows)
    lo, hi = wilson(k, n)
    fails = [r for r in rows if not r.get("success")]
    td = [r["touchdown"] for r in rows if r.get("touchdown")]
    out = dict(n=n, successes=k, rate=k / max(n, 1), wilson95=[lo, hi], landed=sum(bool(r.get("landed")) for r in rows),
               rejected_designated=sum("designated" in r.get("rejected", []) for r in rows), force_home=sum(bool(r.get("force_home")) for r in rows),
               failures=[dict(seed=r["seed"], landed=r.get("landed"), td=r.get("touchdown"), crash=r.get("crash_steps"), contact=r.get("contact_steps"),
                              rejected=r.get("rejected"), error=r.get("error")) for r in fails])  # fmt: skip
    if td:
        out["touchdown_speed_ms"] = dict(
            mean=sum(t["speed"] for t in td) / len(td), max=max(t["speed"] for t in td)
        )
        out["touchdown_dist_m"] = dict(
            mean=sum(t["dist_safe_m"] for t in td) / len(td),
            max=max(t["dist_safe_m"] for t in td),
        )
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument(
        "--seeds", default="1001:1011", help="a:b (b exclusive) or a comma list"
    )
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", required=True)
    ap.add_argument("--set", action="append", default=[], metavar="key=value")
    ap.add_argument(
        "--cal", default=None, help="calibration cache dir shared by the flights"
    )
    a = ap.parse_args()
    if ":" in a.seeds:
        lo, hi = (int(x) for x in a.seeds.split(":"))
        seeds = list(range(lo, hi))
    else:
        seeds = [int(x) for x in a.seeds.split(",")]
    ov = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        ov[k] = (
            json.loads(v) if v.replace(".", "", 1).replace("-", "", 1).isdigit() else v
        )
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cal = Path(a.cal) if a.cal else out / "cal"
    # Calibrate through the same admission and child-environment path.
    with BatchPool(1, threads=True) as calibration:
        rows = calibration.map(lambda s: fly(a.scenario, s, out, ov, cal), seeds[:1])
    with BatchPool(a.jobs, threads=True) as ex:
        rows += list(ex.map(lambda s: fly(a.scenario, s, out, ov, cal), seeds[1:]))
    res = summarize(rows)
    res.update(scenario=str(a.scenario), overrides=ov, seeds=seeds, rows=rows)
    (out / "mc.json").write_text(json.dumps(res, indent=1))
    sys.stdout.write(
        json.dumps({k: v for k, v in res.items() if k != "rows"}, indent=1) + "\n"
    )


if __name__ == "__main__":
    main()
