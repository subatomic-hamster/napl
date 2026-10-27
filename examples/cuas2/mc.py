"""Monte Carlo of the cuas2 lockstep sim: the cuas-lite design (examples/cuas-lite/mc.py: seeded single-target episodes over target kind x range band, the same dispersion
draws, the same referee, the same tables) with the cuas2 app and weapon, and the switches of the ablation: tracker variant and Ph gate.

    python examples/cuas2/mc.py --runs 15 --workers 3 --label imm2 --out build/mc/imm2 --md build/mc/imm2.md
    python examples/cuas2/mc.py --runs 15 --workers 3 --label abnn --fire-cfg '{"tracker": "ab-nn"}' --out build/mc/abnn
    python examples/cuas2/mc.py --runs 15 --workers 3 --label noph --fire-cfg '{"ph_gate": false}' --out build/mc/noph
    python examples/cuas2/mc.py --report imm2=build/mc/imm2 abnn=build/mc/abnn noph=build/mc/noph --md docs/cuas2/ablation.md      # tables from the mc.json files

Seeds are those of the lite design: cell (kind, band) index i -> seed 100000 * (kind * 3 + band) + i + 1 + seed0, person episodes 900000 + i + 1 + seed0, so an
episode is identical across configurations (paired comparison) and reproducible. The Ph columns compare the mean Ph of the FIREs the app sent with the balls that hit."""

import argparse
import json
import math
import shutil
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
import scene as sc

lmc = sc.load("cuas_lite_mc", "cuas-lite/mc.py")
sys.path.insert(
    0, str(Path(__file__).parent)
)  # the lite mc.py put its own directory first: `import run` must find examples/cuas2/run.py
KINDS, BANDS, summarize, table, wilson = (
    lmc.KINDS,
    lmc.BANDS,
    lmc.summarize,
    lmc.table,
    lmc.wilson,
)


def episode(job):
    """Worker: build the scene from the cached composed turret, run the episode, return its result dict (the work dir is deleted)."""
    import run as runner
    from napl.lite.sim_io import build_scene

    cell, name, kw, seed, root, base, fire_cfg = job
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
        res = runner.run_episode(
            man["model"], work, sp, seed, write=False, fire_cfg=fire_cfg
        )
        res.update(cell=cell, kw=dict(kw))
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


def ph_line(rs):
    ok = [r for r in rs if "error" not in r]
    ph = [x for r in ok for x in r.get("ph_fired", []) if x is not None]
    shots = sum(r["shots"] for r in ok)
    hits = sum(r["hits_on_targets"] for r in ok)
    return (
        (float(np.mean(ph)) if ph else math.nan),
        (hits / shots if shots else math.nan),
        len(ph),
    )


def load_results(path):
    return json.loads((Path(path) / "mc.json").read_text())


PH_BINS = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0001)


def reliability(
    res,
    title="Ph reliability (every ball fed: the Ph of its decision vs whether it hit)",
):
    """Bins of Ph -> balls, hits, hit/ball with Wilson interval, mean Ph. Balls without a Ph (Ph never computed) are left out."""
    pts = [
        x
        for r in res
        if "error" not in r and r["cell"] != "person"
        for x in r.get("ph_shot_hit", [])
        if x[0] is not None
    ]
    rows = [
        "| Ph bin | balls | hits | hit/ball [95% CI] | mean Ph |",
        "|---|---|---|---|---|",
    ]
    for lo, hi in zip(PH_BINS, PH_BINS[1:]):
        b = [x for x in pts if lo <= x[0] < hi]
        if b:
            k, n = sum(x[1] for x in b), len(b)
            rows.append(
                f"| {lo:.1f}-{min(hi, 1.0):.1f} | {n} | {k} | {lmc.fmt_ci(k / n, wilson(k, n))} | {np.mean([x[0] for x in b]):.2f} |"
            )
    n = len(pts)
    return (
        f"### {title}\n\n"
        + "\n".join(rows)
        + f"\n\n{n} balls, mean Ph {np.mean([x[0] for x in pts]) if n else math.nan:.2f}, hit/ball {np.mean([x[1] for x in pts]) if n else math.nan:.2f}\n"
    )


def bias(res):
    """Mean closest-approach vector (ball - target, B frame, m) of the shots at static targets (balloon / hanging cells): a systematic vertical or lateral offset shows here."""
    v = [
        m["vec"]
        for r in res
        if "error" not in r and r["cell"].split("/")[0] in ("balloon", "hanging")
        for m in (r.get("miss") or {}).values()
        if m["d"] < 0.6
    ]
    if not v:
        return "no shots at static targets"
    v = np.array(v)
    return f"static-target closest approach (n={len(v)}): mean x {v[:, 0].mean():+.3f} y {v[:, 1].mean():+.3f} z {v[:, 2].mean():+.3f} m, std y {v[:, 1].std():.3f} z {v[:, 2].std():.3f} (se of the z mean {v[:, 2].std() / math.sqrt(len(v)):.3f})"


def tables(res, kinds, bands):
    cells = {}
    for r in res:
        cells.setdefault(r["cell"], []).append(r)
    by_band = [
        (
            b,
            summarize(
                [r for c, rs in cells.items() if c.endswith("/" + b) for r in rs]
            ),
        )
        for b in bands
    ]
    by_kind = [
        (
            k,
            summarize(
                [r for c, rs in cells.items() if c.startswith(k + "/") for r in rs]
            ),
        )
        for k in kinds
    ]
    mc_cells = [(c, summarize(rs)) for c, rs in cells.items() if c != "person"]
    return cells, by_band, by_kind, mc_cells


def report(specs, kinds, bands, md):
    """Ablation table: one row per configuration (label=dir), by band and by kind."""
    out = ["# cuas2 ablation (paired seeds)\n"]
    for title, key in (("By range band", "band"), ("By target kind", "kind")):
        rows = []
        for lab, d in specs:
            res = load_results(d)
            cells, by_band, by_kind, _ = tables(res, kinds, bands)
            for k, s in by_band if key == "band" else by_kind:
                rows.append((f"{lab} / {k}", s))
        out.append(table(title, rows, "config / " + key))
    tot = []
    for lab, d in specs:
        res = load_results(d)
        s = summarize([r for r in res if r["cell"] != "person"])
        pr = [r for r in res if r["cell"] == "person"]
        mph, hb, n = ph_line([r for r in res if r["cell"] != "person"])
        tot.append(f"| {lab} | {s['episodes']} | {s['shots']} | {lmc.fmt_ci(s['hit_p'], s['hit_ci'])} | {lmc.fmt_ci(s['kill_p'], s['kill_ci'])} | {s['shots_per_kill']:.2f} | {mph:.2f} | {hb:.2f} | {sum(r['n_violations'] for r in res)} | {sum(r.get('shots', 0) for r in pr if 'error' not in r)} |")  # fmt: skip
    out.append("### Totals\n\n| config | episodes | balls | hit/ball [95% CI] | kills/episode [95% CI] | balls/kill | mean Ph of FIREs | hit/ball | violations (all) | balls in person episodes |\n|---|---|---|---|---|---|---|---|---|---|\n" + "\n".join(tot) + "\n")  # fmt: skip
    allres = [r for _, d in specs for r in load_results(d)]
    out.append(reliability(allres, "Ph reliability, all configurations pooled"))
    out.append("- " + bias(allres) + "\n")
    text = "\n".join(out)
    if md:
        Path(md).write_text(text + "\n")
    sys.stdout.write(text + "\n")  # ttsr-ignore: py/no-print (CLI)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--runs", type=int, default=15, help="episodes per (kind, band) cell"
    )
    ap.add_argument("--kinds", nargs="*", default=list(KINDS), choices=list(KINDS))
    ap.add_argument("--bands", nargs="*", default=list(BANDS), choices=list(BANDS))
    ap.add_argument("--person-runs", type=int, default=20)
    ap.add_argument(
        "--workers",
        type=int,
        default=3,
        help="processes (the box allows 4 MC jobs in total)",
    )
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--out", default="build/mc")
    ap.add_argument("--md", default=None)
    ap.add_argument("--label", default="run")
    ap.add_argument("--note", default="")
    ap.add_argument(
        "--fire-cfg",
        default="{}",
        help='JSON FireCfg2 overrides for every episode, e.g. \'{"tracker": "imm3-gnn", "ph_tau": 0.2}\'',
    )
    ap.add_argument("--report", nargs="*", default=None, metavar="LABEL=DIR")
    a = ap.parse_args(argv)
    if a.report is not None:
        return report([s.split("=", 1) for s in a.report], a.kinds, a.bands, a.md)
    fire_cfg = json.loads(a.fire_cfg)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    base = str(sc.cached_base())
    jobs = []
    for kind in a.kinds:
        for band in a.bands:
            for i in range(a.runs):
                name, kw, seed = lmc.draw(kind, band, i, a.seed0)
                jobs.append(
                    (f"{kind}/{band}", name, kw, seed, str(out), base, fire_cfg)
                )
    for i in range(a.person_runs):
        name, kw, seed = lmc.draw_person(i, a.seed0)
        jobs.append(("person", name, kw, seed, str(out), base, fire_cfg))
    import multiprocessing as mp

    with ProcessPoolExecutor(
        max_workers=a.workers, mp_context=mp.get_context("spawn")
    ) as ex:
        res = list(ex.map(episode, jobs, chunksize=1))
    (out / "mc.json").write_text(json.dumps(res, indent=1, default=float))
    cells, by_band, by_kind, mc_cells = tables(res, a.kinds, a.bands)
    tot_v = sum(r["n_violations"] for r in res)
    errs = [r for r in res if "error" in r]
    pr = cells.get("person", [])
    person = summarize(pr)
    mph, hb, n = ph_line([r for r in res if r["cell"] != "person"])
    md = [
        f"# cuas2 sim results: {a.label}\n",
        f"Generated by `python examples/cuas2/mc.py --runs {a.runs} --kinds {' '.join(a.kinds)} --bands {' '.join(a.bands)} --person-runs {a.person_runs} --seed0 {a.seed0} --label {a.label}"
        f"{' --fire-cfg ' + repr(a.fire_cfg) if fire_cfg else ''}` {a.note}\n",
        table("By range band (all target kinds)", by_band, "range band m"),
        table("By target kind (all bands)", by_kind, "kind"),
        table("By kind and range band", mc_cells, "kind / band"),
        "### Safety\n",
        f"- episodes: {len(res)}; crashed: {len(errs)}; **safety violations over all episodes: {tot_v}** (must be 0)",
        f"- person stress ({len(pr)} episodes): balls fed {person['shots']}, violations {person['violations']}, balls fed with a person anywhere in the image {sum(r.get('person_raw', 0) for r in pr if 'error' not in r)}",
        f"- Ph calibration: mean Ph of the {n} FIREs sent {mph:.2f}, realised hit/ball {hb:.2f}",
        f"- {bias(res)}\n",
        reliability(res),
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
