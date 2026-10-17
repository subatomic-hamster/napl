"""Monte Carlo of a dog autonomy mission: one run per seed (automission.py in its own process), success = the spec's mission
criteria (coverage >= 95 % of the reachable traversable area, map RMS <= 5 cm, home by the 20 min budget), a Wilson interval.

    python examples/dog/automc.py <scenario.scenario.toml> --seeds 1001:1025 --jobs 8 --out <dir>

DEV seeds (tuning): 1-30. HELD-OUT seeds (never run before the numbers in docs/dog/autonomy.md were taken): 1001 and up."""

import argparse
import json
import os
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "skydio-land"))
from autonomy import scene
from land_mc import wilson


def one(scenario, seed, out):
    d = out / f"seed{seed}"
    d.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "NAPL_PARAMS": json.dumps({"seed": seed})}
    r = subprocess.run(
        [sys.executable, str(HERE / "automission.py"), str(scenario), str(d)],
        capture_output=True,
        text=True,
        env=env,
    )
    if r.returncode != 0:
        return {"seed": seed, "error": r.stderr[-1500:]}
    return {"seed": seed, **json.loads((d / "summary.json").read_text())}


def reasons(s, t_budget):
    if "error" in s:
        return ["crashed"]
    out = []
    if not s["coverage"] >= 0.95:
        out.append("coverage")
    if not s["rms"] <= 0.05:
        out.append("rms")
    if s["phase"] != 4:
        out.append("not_home")
    if s["t"] > t_budget:
        out.append("late")
    return out


def main():
    a = argparse.ArgumentParser()
    a.add_argument("scenario")
    a.add_argument("--seeds", default="1001:1025")
    a.add_argument("--jobs", type=int, default=8)
    a.add_argument("--out", required=True)
    a = a.parse_args()
    scenario, out = Path(a.scenario).resolve(), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    lo, hi = (int(v) for v in a.seeds.split(":"))
    auto, _, _ = scene.scenario(scenario)
    with ThreadPoolExecutor(a.jobs) as ex:
        runs = list(ex.map(lambda s: one(scenario, s, out), range(lo, hi)))
    with open(out / "runs.jsonl", "w") as f:
        f.writelines(json.dumps(s) + "\n" for s in runs)
    why = {s["seed"]: reasons(s, auto["t_budget"]) for s in runs}
    k, n = sum(not w for w in why.values()), len(runs)
    wl, wh = wilson(k, n)
    ok = [s for s in runs if "error" not in s]
    med = (
        {
            c: statistics.median(s[c] for s in ok)
            for c in ("coverage", "rms", "t", "distance")
        }
        if ok
        else {}
    )
    fails = {
        r: sum(r in w for w in why.values())
        for r in ("coverage", "rms", "not_home", "late", "crashed")
    }
    rep = [f"# MC {scenario.name}, seeds {a.seeds}", "", f"success {k}/{n}, Wilson 95 % [{wl:.3f}, {wh:.3f}]", "",
           "| median | value |", "|---|---|", *[f"| {c} | {v:.4g} |" for c, v in med.items()], "",
           "| failure | count |", "|---|---|", *[f"| {r} | {c} |" for r, c in fails.items()]]  # fmt: skip
    (out / "report.md").write_text("\n".join(rep) + "\n")
    print("\n".join(rep))


if __name__ == "__main__":
    main()
