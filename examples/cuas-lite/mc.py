"""Monte Carlo of the cuas-lite lockstep sim (spec 7.3): seeded batches of single-target episodes over target kind x range band, with the ballistic, wind, latency
and detector dispersion drawn per episode. Every episode is the real App + SimRP2040 + kernel, CPU only, deterministic per (cell, index).

    python examples/cuas-lite/mc.py --runs 20 --workers $NAPL_THREADS --out build/mc --md docs/cuas-lite/sim_results.md
    python examples/cuas-lite/mc.py --runs 2 --kinds balloon --bands 5-10 --person-runs 2 --out build/mc-smoke        # smoke

Reports per cell and per range band: balls fired / hit (hit probability per ball), episodes killed, time to first shot and to first hit (from ARM), balls per kill, with
Wilson 95 % intervals; and the safety-violation count over ALL episodes (must be exactly 0: any person-block or gate violation exits 1). Text only.

Dispersion per episode (spec 5.3 numbers; the app assumes the nominal values): true v0 = 30 + N(0, 1.0) m/s, shot-to-shot sigma 1.5 m/s, direction spread
10 mrad x U(0.7, 1.3), Cd ~ N(0.47, 0.04) clipped [0.38, 0.56], steady wind N(0, 0.5) m/s per horizontal axis (drifting balloons: 1 m/s crosswind + N(0, 0.5)), anemometer
error N(0, 0.3), camera latency U(20, 45) ms and detector latency U(10, 30) ms with 4 ms per-frame jitter, detector miss scale U(0.5, 2) x the spec p_miss and pixel noise U(1, 3) px."""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np

KINDS = {  # label -> (scenario, keywords)
    "balloon": ("mc_balloon", {}),
    "drift": ("mc_drift", {}),
    "hanging": ("mc_hanging", {}),
    "quad3": ("mc_quad", dict(speed=3.0)),
    "quad8": ("mc_quad", dict(speed=8.0)),
}
BANDS = {"3-5": (3.0, 5.0), "5-10": (5.0, 10.0), "10-15": (10.0, 15.0)}


def wilson(k, n, z=1.96):
    if n == 0:
        return (math.nan, math.nan)
    ph = k / n
    d = 1 + z * z / n
    c = (ph + z * z / (2 * n)) / d
    s = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - s), min(1.0, c + s))


def draw(kind, band, i, base_seed=0):
    """-> (scenario name, keywords for scene.make, seed) of episode i of a cell; fully determined by (kind, band, i, base_seed)."""
    name, kw = KINDS[kind]
    lo, hi = BANDS[band]
    seed = (
        100000 * (list(KINDS).index(kind) * 3 + list(BANDS).index(band))
        + i
        + 1
        + base_seed
    )
    r = np.random.default_rng(seed)
    kw = dict(kw)
    if "speed" in kw:
        kw["speed"] *= float(r.uniform(0.9, 1.1))
    wind = r.normal(0, 0.5, 2)
    if kind == "drift":
        wind = wind + (0.0, 1.0)
    kw.update(
        R=float(r.uniform(lo, hi)), wind=(float(wind[0]), float(wind[1]), 0.0), wind_noise=0.3, v0_err=float(r.normal(0, 1.0)), spread=0.010 * float(r.uniform(0.7, 1.3)),
        cd=float(np.clip(r.normal(0.47, 0.04), 0.38, 0.56)), cam_lat=float(r.uniform(0.020, 0.045)), det_lat=float(r.uniform(0.010, 0.030)), jitter=0.004,
        miss_scale=float(r.uniform(0.5, 2.0)), px_sigma=float(r.uniform(1.0, 3.0)),
    )  # fmt: skip
    return name, kw, seed


def draw_person(i, base_seed=0):
    seed = 900000 + i + 1 + base_seed
    r = np.random.default_rng(seed)
    kw = dict(wind=(float(r.normal(0, 0.5)), float(r.normal(0, 0.5)), 0.0), t_walk=float(r.uniform(9.0, 14.0)), person_miss=float(r.uniform(0.0, 0.15)),
              miss_scale=float(r.uniform(0.5, 2.0)), cam_lat=float(r.uniform(0.020, 0.045)), det_lat=float(r.uniform(0.010, 0.030)), jitter=0.004)  # fmt: skip
    return "mc_person", kw, seed


def episode(job):
    """Worker: build the scene from the cached composed turret (scene.cached_base), run the episode, return its result dict (the work dir is deleted)."""
    import run as runner
    import scene as sc
    from napl.lite.sim_io import build_scene

    cell, name, kw, seed, root, base = job
    work = Path(tempfile.mkdtemp(prefix="ep", dir=root))
    try:
        sp = sc.make(name, seed, **kw)
        man = build_scene(
            work / "scene",
            sp.targets,
            env=sp.env,
            seed=seed,
            launcher=sp.launcher,
            base=base,
        )
        res = runner.run_episode(man["model"], work, sp, seed, write=False)
        res.update(cell=cell, kw={k: v for k, v in kw.items()})
        return res
    except Exception as e:  # noqa: BLE001 - a crashed episode is a failed episode, reported
        return dict(
            cell=cell,
            seed=seed,
            error=f"{type(e).__name__}: {e}",
            ok=False,
            n_violations=0,
        )
    finally:
        shutil.rmtree(work, ignore_errors=True)


def pct(xs, q):
    return float(np.percentile(xs, q)) if xs else math.nan


def summarize(rs):
    """Result dicts of one cell (or band) -> metrics."""
    ok = [r for r in rs if "error" not in r]
    shots = sum(r["shots"] for r in ok)
    hits = sum(r["hits_on_targets"] for r in ok)
    kills = sum(1 for r in ok if r["popped"])
    fired = [r for r in ok if r["shots"]]
    ttfs = [r["ttfs"] for r in fired]
    tth = [r["t_first_hit"] for r in ok if r["t_first_hit"] is not None]
    return dict(episodes=len(rs), errors=len(rs) - len(ok), engaged=len(fired), shots=shots, hits=hits, hit_p=hits / shots if shots else math.nan, hit_ci=wilson(hits, shots),
                kills=kills, kill_p=kills / len(ok) if ok else math.nan, kill_ci=wilson(kills, len(ok)), ttfs_med=pct(ttfs, 50), ttfs_p90=pct(ttfs, 90),
                tth_med=pct(tth, 50), tth_p90=pct(tth, 90), shots_per_kill=shots / kills if kills else math.inf,
                violations=sum(r["n_violations"] for r in rs))  # fmt: skip


def fmt_ci(p, ci):
    return "n/a" if math.isnan(p) else f"{p:.2f} [{ci[0]:.2f}, {ci[1]:.2f}]"


def table(title, rows, label):
    f = lambda x: (
        "-" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.2f}"
    )
    out = [f"| {label} | episodes | engaged | balls | hit/ball [95% CI] | kills/episode [95% CI] | t first shot s (med/p90) | t first hit s (med/p90) | balls/kill | violations |", "|---|---|---|---|---|---|---|---|---|---|"]  # fmt: skip
    for k, s in rows:
        out.append(f"| {k} | {s['episodes']} | {s['engaged']} | {s['shots']} | {fmt_ci(s['hit_p'], s['hit_ci'])} | {fmt_ci(s['kill_p'], s['kill_ci'])} | {f(s['ttfs_med'])} / {f(s['ttfs_p90'])} | {f(s['tth_med'])} / {f(s['tth_p90'])} | {'inf' if math.isinf(s['shots_per_kill']) else f(s['shots_per_kill'])} | {s['violations']} |")  # fmt: skip
    return f"### {title}\n\n" + "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--runs", type=int, default=20, help="episodes per (kind, band) cell"
    )
    ap.add_argument("--kinds", nargs="*", default=list(KINDS), choices=list(KINDS))
    ap.add_argument("--bands", nargs="*", default=list(BANDS), choices=list(BANDS))
    ap.add_argument(
        "--person-runs",
        type=int,
        default=20,
        help="safety-stress episodes (a person walks through, person misses up to 15 %%)",
    )
    ap.add_argument(
        "--workers", type=int, default=None, help="upper bound; default follows the execution policy"
    )
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--out", default="build/mc")
    ap.add_argument(
        "--md", default=None, help="write the results table (markdown, text only) here"
    )
    ap.add_argument(
        "--note",
        default="",
        help="extra line for the markdown header (command, host, date)",
    )
    ap.add_argument(
        "--fire-cfg",
        default="{}",
        help="JSON FireCfg overrides for every episode, e.g. '{\"t_fire\": 0.05}' (sensitivity studies)",
    )
    a = ap.parse_args(argv)
    fire_cfg = json.loads(a.fire_cfg)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    import scene as sc

    base = str(sc.cached_base())  # composed once (minutes), every episode copies it
    jobs = []
    for kind in a.kinds:
        for band in a.bands:
            for i in range(a.runs):
                name, kw, seed = draw(kind, band, i, a.seed0)
                kw["fire_cfg"] = fire_cfg
                jobs.append((f"{kind}/{band}", name, kw, seed, str(out), base))
    for i in range(a.person_runs):
        name, kw, seed = draw_person(i, a.seed0)
        kw["fire_cfg"] = fire_cfg
        jobs.append(("person", name, kw, seed, str(out), base))

    from napl.runtime import BatchPool
    with BatchPool(a.workers) as ex:
        res = ex.map(episode, jobs)
    (out / "mc.json").write_text(json.dumps(res, indent=1, default=float))
    cells = {}
    for r in res:
        cells.setdefault(r["cell"], []).append(r)
    mc_cells = [(c, summarize(rs)) for c, rs in cells.items() if c != "person"]
    by_band = [
        (
            b,
            summarize(
                [r for c, rs in cells.items() if c.endswith("/" + b) for r in rs]
            ),
        )
        for b in a.bands
    ]
    by_kind = [
        (
            k,
            summarize(
                [r for c, rs in cells.items() if c.startswith(k + "/") for r in rs]
            ),
        )
        for k in a.kinds
    ]
    tot_v = sum(r["n_violations"] for r in res)
    errs = [r for r in res if "error" in r]
    person = summarize(cells.get("person", []))
    pr = cells.get("person", [])
    md = [
        "# cuas-lite sim results (Monte Carlo, lockstep)\n",
        f"Generated by `python examples/cuas-lite/mc.py --runs {a.runs} --kinds {' '.join(a.kinds)} --bands {' '.join(a.bands)} --person-runs {a.person_runs} --seed0 {a.seed0}{' --fire-cfg ' + repr(a.fire_cfg) if fire_cfg else ''}` {a.note}\n",
        "Measured, not estimated: every number below comes from the episodes in `mc.json` (the real App + SimRP2040 twin + napl kernel launcher). The *inputs* are estimates "
        "(ball v0 30 m/s, sigma 1.5 m/s, spread 10 mrad, Cd 0.47 from spec 5.3; stand-in detector noise from spec 7.2), so the probabilities describe the design under those assumptions, "
        "not the hardware. Targets: 0.30 m balloon / disc, 0.30 m quad (flat box, kinematic path, one foam-ball hit at >= 8 m/s = kill, a scenario assumption). One target per episode, "
        "ARM at 8 s, per-arm cap 5 balls, ends at the kill / cap / 30 s.\n",
        table("By range band (all target kinds)", by_band, "range band m"),
        table("By target kind (all bands)", by_kind, "kind"),
        table("By kind and range band", mc_cells, "kind / band"),
        "### Safety\n",
        f"- episodes: {len(res)}; crashed episodes: {len(errs)}; **safety violations over all episodes: {tot_v}** (must be 0)",
        f"- person stress ({len(pr)} episodes, person crosses the field of view at 9-14 s, detector misses the person on up to 15 % of replies): balls fed {person['shots']}, violations {person['violations']}, "
        f"balls fed with a person anywhere in the image (latency window, inside the {0.25} s grace) {sum(r.get('person_raw', 0) for r in pr if 'error' not in r)}",
    ]
    if a.md:
        Path(a.md).write_text("\n".join(md) + "\n")
    sys.stdout.write("\n".join(md) + "\n")  # ttsr-ignore: py/no-print (CLI)
    if errs:
        sys.stdout.write(
            "crashed: "
            + "; ".join(f"{r['cell']} seed {r['seed']}: {r['error']}" for r in errs)
            + "\n"
        )  # ttsr-ignore: py/no-print (CLI)
    sys.exit(1 if tot_v or errs else 0)


if __name__ == "__main__":
    main()
