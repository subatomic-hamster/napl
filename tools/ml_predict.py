#!/usr/bin/env python3
"""Learned motion prediction for fire-control lead (docs/cuas2/ml-prediction.md).

  python tools/ml_predict.py gen   --out DATA.npz [--train 160 --val 40 --test 60 --jobs 2]
  python tools/ml_predict.py train --data DATA.npz --out DIR [--seeds 3 --epochs 40] [--exclude KIND]
  python tools/ml_predict.py eval  --data DATA.npz --models DIR [--md OUT.md --json OUT.json] [--heldout DIR2]
  python tools/ml_predict.py bench --data DATA.npz --models DIR
  ML-3 (randomised threat suite, napl.threats; docs/cuas2/ml-prediction.md):
  python tools/ml_predict.py gen   --suite threats --out DATA.npz [--rand-mult 6 --stride 2]
  python tools/ml_predict.py train --data DATA.npz --out DIR --suite threats [--aug 1] [--exclude-group weave]
  python tools/ml_predict.py eval-threats --data DATA.npz --models DIR --logo DIR2 [--md OUT.md --json OUT.json]

Seeds: train 1001.., val 2001.., test 3001.. (disjoint from the 1..100 of the tracking benchmark and tuning)."""

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import numpy as np
from napl.track import calib as C
from napl.track.learned import data as D
from napl.track.learned import hybrid as HY
from napl.track.learned import metrics as M
from napl.track.learned import models as ML
from napl.track.scenarios import NAMES

HZ = D.HORIZONS


def base(d, k):  # IMM extrapolation (k = "2" or "3") as (mean, cov)
    return d[f"pm{k}"].astype(np.float64), M.tri_to_cov(d[f"pc{k}"].astype(np.float64))


def sub(d, mask):
    return {k: v[mask] for k, v in d.items()}


def kinds(d):
    return d["meta"][:, 0].astype(int)


def models_for(te, va, mdir):
    """name -> (mean, cov) on the test split. Calibrations are fit on the val split only."""
    out = {}
    for k in "23":
        mv, cv = base(va, k)
        m, c = base(te, k)
        out[f"IMM{k}"] = (m, c)
        if k == "2":
            out["IMM2 x s (ML scalar)"] = (
                m,
                M.inflate(c, M.fit_inflation(mv, cv, va["y"], False)),
            )
            out["IMM2 x s (q90 scalar)"] = (
                m,
                M.inflate(c, M.fit_quantile(mv, cv, va["y"], 0.9, False)),
            )
        out[f"IMM{k} x s_h (ML per horizon)"] = (
            m,
            M.inflate(c, M.fit_inflation(mv, cv, va["y"])),
        )
        out[f"IMM{k} x s_h (q90 per horizon)"] = (
            m,
            M.inflate(c, M.fit_quantile(mv, cv, va["y"], 0.9)),
        )
    # oracle: per-kind per-horizon ML inflation (needs the scenario kind, which a real tracker does not know)
    cc = out["IMM2"][1].copy()
    for kd in np.unique(kinds(te)):
        mk, vk = kinds(te) == kd, kinds(va) == kd
        if vk.any():
            mv, cv = base(sub(va, vk), "2")
            cc[mk] = M.inflate(cc[mk], M.fit_inflation(mv, cv, va["y"][vk]))
    out["IMM2 x s_h,kind (oracle)"] = (out["IMM2"][0], cc)
    for kind in ("mlp", "gru"):
        fs = sorted(Path(mdir).glob(f"{kind}_s*.npz"))
        for f in fs:
            out[f"{kind.upper()} s{f.stem.split('_s')[1]}"] = ML.predict_np(
                ML.load(f), te
            )
    return out


def ens(res, kind):
    """Average the per-training-seed metric dicts of one learned kind into one entry."""
    ks = [k for k in res if k.startswith(kind.upper() + " s")]
    return {m: np.mean([res[k][m] for k in ks], 0) for m in res[ks[0]]} if ks else None


def table(res, sel=None):
    """Markdown rows of a result dict."""
    rows = [
        "| model | RMSE m @0.2 / 0.6 / 1.0 s | NLL (mean over h) | cover 50 % (mean h) | cover 90 % (mean h) | cover 90 % @1.0 s | d2/3 |",
        "|---|---|---|---|---|---|---|",
    ]
    for n, r in res.items():
        if sel and n not in sel:
            continue
        rows.append(
            f"| {n} | {r['rmse'][0]:.3f} / {r['rmse'][2]:.3f} / {r['rmse'][4]:.3f} | {r['nll'].mean():.3f} | {r['c50'].mean():.3f} | "
            f"{r['c90'].mean():.3f} | {r['c90'][4]:.3f} | {r['d2'].mean():.2f} |"
        )
    return rows


def paired_ci(a, b, groups, n=1000, seed=0):
    """Bootstrap (by run group) 95 % CI of mean(a - b)."""
    u, inv = np.unique(groups, return_inverse=True)
    sa, sb, cnt = (np.bincount(inv, w, len(u)) for w in (a, b, np.ones(len(a))))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(u), (n, len(u)))
    dif = (sa[idx].sum(1) - sb[idx].sum(1)) / cnt[idx].sum(1)
    return (
        float((sa.sum() - sb.sum()) / cnt.sum()),
        float(np.quantile(dif, 0.025)),
        float(np.quantile(dif, 0.975)),
    )


def cmd_gen(a):
    t = time.time()
    sp = {
        "train": range(1001, 1001 + a.train),
        "val": range(2001, 2001 + a.val),
        "test": range(3001, 3001 + a.test),
    }
    sp = {k: v for k, v in sp.items() if k in a.splits.split(",")}
    if a.suite == "threats":
        from napl.threats import NAMES as TN

        ds = D.build_dataset(
            sp,
            TN,
            jobs=a.jobs,
            suite="threats",
            stride=a.stride,
            extra={"random": a.rand_mult, "random_cam": a.rand_mult},
        )
    else:
        ds = D.build_dataset(sp, jobs=a.jobs)
    D.save(a.out, ds)
    print({s: len(d["y"]) for s, d in ds.items()}, f"{time.time() - t:.0f} s")


def cmd_train(a):
    ds = D.load(a.data)
    tr, va = ds["train"], ds["val"]
    if a.exclude:
        k = NAMES.index(a.exclude)
        tr, va = sub(tr, kinds(tr) != k), sub(va, kinds(va) != k)
    if a.exclude_group:  # threats: drop every sample whose OBJECT came from the generator group (scene-independent)
        gs = [
            D.GROUPS.index(g) for g in a.exclude_group.split("+")
        ]  # "route+swarm" drops both
        tr, va = (sub(x, ~np.isin(x["meta"][:, 5], gs)) for x in (tr, va))
    if a.aug:  # yaw + world-scale augmentation: a.aug extra randomised copies of the training split
        rng = np.random.default_rng(12345)
        parts = [{k: tr[k] for k in D.AUG_KEYS}] + [
            D.augment(tr, rng) for _ in range(a.aug)
        ]
        tr = {k: np.concatenate([p[k] for p in parts]) for k in D.AUG_KEYS}
        print(f"train samples {len(tr['y'])} (x{1 + a.aug} with augmentation)")
    Path(a.out).mkdir(parents=True, exist_ok=True)
    for kind in a.kinds.split(","):
        for s in range(a.first_seed, a.seeds):
            t = time.time()
            W = ML.train(tr, va, kind, s, epochs=a.epochs, device=a.device)
            ML.save(Path(a.out) / f"{kind}_s{s}.npz", W)
            print(
                f"{kind} s{s}: val NLL {W['val_nll']:.3f} @ep {W['best_epoch']}, {time.time() - t:.0f} s",
                flush=True,
            )


def cmd_eval(a):
    ds = D.load(a.data)
    te, va = ds["test"], ds["val"]
    R, md, js = models_for(te, va, a.models), [], {}
    scopes = [("all kinds pooled", np.ones(len(te["y"]), bool))] + [
        (k, kinds(te) == i) for i, k in enumerate(NAMES)
    ]
    for name, mask in scopes:
        res = {
            n: M.evaluate(m[mask], c[mask], te["y"][mask])
            for n, (m, c) in {k: v for k, v in R.items()}.items()
        }
        for kind in ("mlp", "gru"):
            e = ens(res, kind)
            if e:
                res = {
                    k: v
                    for k, v in res.items()
                    if not k.startswith(kind.upper() + " s")
                }
                res[f"{kind.upper()} (mean of seeds)"] = e
        js[name] = {n: {k: v.tolist() for k, v in r.items()} for n, r in res.items()}
        md += [f"#### {name} (n = {int(mask.sum())} test samples)", ""]
        if name == "all kinds pooled":
            md += ["Per horizon (s = 0.2 .. 1.0):", ""]
            md += [
                "| model | RMSE m | NLL | cover 50 % | cover 90 % |",
                "|---|---|---|---|---|",
            ]
            for n, r in res.items():
                f = lambda v, p: " / ".join(f"{x:.{p}f}" for x in v)
                md.append(
                    f"| {n} | {f(r['rmse'], 3)} | {f(r['nll'], 2)} | {f(r['c50'], 2)} | {f(r['c90'], 2)} |"
                )
            md.append("")
        md += table(res) + [""]
    # paired bootstrap by run group: learned (seed-0 models) minus IMM2
    md += [
        "#### Paired differences vs IMM2 (bootstrap over runs, 95 % CI; negative = learned better)",
        "",
        "| kind | model | d NLL (mean h) | d sq. err m^2 @0.6 s | d sq. err m^2 @1.0 s |",
        "|---|---|---|---|---|",
    ]
    grp = te["meta"][:, 0] * 10000 + te["meta"][:, 1]
    for name, mask in scopes[1:]:
        for mdl in ("MLP s0", "GRU s0", "IMM2 x s_h (ML per horizon)"):
            if mdl not in R:
                continue
            m, c = R[mdl]
            m0, c0 = R["IMM2"]
            y = te["y"]
            d_a, C_a = M.mahal2(m, c, y)
            d_b, C_b = M.mahal2(m0, c0, y)
            nll_a = 0.5 * (d_a + np.linalg.slogdet(C_a)[1]).mean(1)
            nll_b = 0.5 * (d_b + np.linalg.slogdet(C_b)[1]).mean(1)
            se = lambda mm, h: ((y[:, h] - mm[:, h]) ** 2).sum(-1)
            cells = [paired_ci(nll_a[mask], nll_b[mask], grp[mask])] + [
                paired_ci(se(m, h)[mask], se(m0, h)[mask], grp[mask]) for h in (2, 4)
            ]
            md.append(
                f"| {name} | {mdl} | "
                + " | ".join(f"{x[0]:+.3f} [{x[1]:+.3f}, {x[2]:+.3f}]" for x in cells)
                + " |"
            )
    if a.heldout:
        md += [
            "",
            "#### Leave-one-kind-out (MLP trained WITHOUT the kind, tested on it; seed-0 model per kind)",
            "",
        ]
        rows = {}
        for i, k in enumerate(NAMES):
            f = Path(a.heldout) / f"x_{k}" / "mlp_s0.npz"
            if not f.exists():
                continue
            mask = kinds(te) == i
            m, c = ML.predict_np(ML.load(f), sub(te, mask))
            rows[f"{k}: MLP (held out)"] = M.evaluate(m, c, te["y"][mask])
            rows[f"{k}: IMM2"] = M.evaluate(*base(sub(te, mask), "2"), te["y"][mask])
        md += table(rows)
    text = "\n".join(md)
    print(text)
    if a.md:
        Path(a.md).write_text(text + "\n")
    if a.json:
        Path(a.json).write_text(json.dumps(js))


PASS_COVER = (
    0.80,
    0.97,
)  # held-out 90 % cover (mean over the five horizons) for "usable in fire control"


def _sqerr(m, y, h):
    return ((y[:, h] - m[:, h]) ** 2).sum(-1)


def _nll(m, c, y):
    d, Cm = M.mahal2(m, c, y)
    return (0.5 * (d + np.linalg.slogdet(Cm)[1])).mean(1)


def cmd_hybrid(a):
    """ML-4: leave-one-generator-out pass table for learned-mean + safe-covariance hybrids (napl.track.learned.hybrid).
    (k, g) are fitted on the val split WITHOUT the held-out group, per LOGO model, then applied to the test split."""
    te, va = D.load(a.data)["test"], D.load(a.val)["val"]
    va = sub(va, np.arange(len(va["y"])) % a.fit_stride == 0)
    y, gt, gv = te["y"], te["meta"][:, 5].astype(int), va["meta"][:, 5].astype(int)
    I2t, I2v = base(te, "2"), base(va, "2")
    ldir = Path(a.logo)
    hdr = [
        "| group | variant | fit | k / g (mean over seeds) | cover 90 % mean h (min-max h) | NLL (mean h) | RMSE m @1.0 s (IMM2) | PASS mean-h | PASS every-h |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    md, js = [], {}
    inr = lambda c: (PASS_COVER[0] <= c) & (c <= PASS_COVER[1])
    yn = lambda b: "yes" if b else "no"
    for g in D.GROUPS + ("route+swarm",):
        files = sorted((ldir / f"x_{g}").glob("mlp_s*.npz"))
        if not files:
            continue
        ix = [D.GROUPS.index(x) for x in g.split("+")]
        tm, vm = np.isin(gt, ix), ~np.isin(gv, ix)
        yt, mt2, Ct2 = y[tm], I2t[0][tm], I2t[1][tm]
        i2 = M.evaluate(mt2, Ct2, yt)
        variants = {}  # (name, fit) -> list over seeds of (metrics, k, g)
        for f in files:
            W = ML.load(f)
            ml, Cl = ML.predict_np(W, sub(te, tm))
            mv, Cv = ML.predict_np(W, sub(va, vm))
            variants.setdefault(("learned", "-"), []).append(
                (M.evaluate(ml, Cl, yt), 0, 0)
            )
            for mode in HY.MODES:
                for obj in ("nll", "guard"):
                    k, gg, _ = HY.fit(
                        mode, mv, Cv, I2v[0][vm], I2v[1][vm], va["y"][vm], gv[vm], obj
                    )
                    m, C = HY.hybrid(mode, ml, Cl, mt2, Ct2, k, gg)
                    variants.setdefault((mode, obj), []).append(
                        (M.evaluate(m, C, yt), k, gg)
                    )
            print(g, f.stem, "done", flush=True, file=sys.stderr)
        md += [
            f"#### {g} held out (n = {int(tm.sum())} test samples, {len(files)} LOGO seeds)",
            "",
        ] + hdr
        cell = lambda r: (
            f"{r['c90'].mean():.3f} ({r['c90'].min():.2f}-{r['c90'].max():.2f}) | {r['nll'].mean():.2f}"
        )
        md.append(
            f"| {g} | IMM2 | - | - | {cell(i2)} | {i2['rmse'][4]:.3f} | {yn(inr(i2['c90'].mean()))} | {yn(inr(i2['c90']).all())} |"
        )
        js[g] = {"imm2": {q: v.tolist() for q, v in i2.items()}}
        for (name, obj), rs in variants.items():
            mean_r = {q: np.mean([r[q] for r, _, _ in rs], 0) for q in rs[0][0]}
            p1 = sum(
                bool(inr(r["c90"].mean())) and r["rmse"][4] <= i2["rmse"][4]
                for r, _, _ in rs
            )
            p2 = sum(
                bool(inr(r["c90"]).all()) and r["rmse"][4] <= i2["rmse"][4]
                for r, _, _ in rs
            )
            kk = f"{np.mean([k for _, k, _ in rs]):.2f} / {np.mean([q for _, _, q in rs]):.2f}"
            md.append(
                f"| {g} | {name} | {obj} | {kk if name != 'learned' else '-'} | {cell(mean_r)} | {mean_r['rmse'][4]:.3f} ({i2['rmse'][4]:.3f}) | {p1}/{len(rs)} | {p2}/{len(rs)} |"
            )
            js[g][f"{name}/{obj}"] = dict(
                pass_mean_h=p1,
                pass_every_h=p2,
                seeds=len(rs),
                kg=[(k, q) for _, k, q in rs],
                metrics={q: v.tolist() for q, v in mean_r.items()},
            )
        md.append("")
    text = "\n".join(md)
    print(text)
    if a.md:
        Path(a.md).write_text(text + "\n")
    if a.json:
        Path(a.json).write_text(json.dumps(js))


def cmd_eval_threats(a):
    """ML-3: in-distribution and leave-one-generator-out evaluation on the threat-suite test split.
    --hybrid: the ML-4 hybrid pass table instead (needs --val; --models unused)."""
    if a.hybrid:
        return cmd_hybrid(a)
    assert a.models, "--models is required without --hybrid"
    te = D.load(a.data)["test"]
    y, grp = (
        te["y"],
        te["meta"][:, 0].astype(np.int64) * 1_000_000
        + te["meta"][:, 1].astype(np.int64),
    )
    g_of = te["meta"][:, 5].astype(int)
    lab = te["meta"][:, 6].astype(int)
    IMM = {"IMM2": base(te, "2"), "IMM3": base(te, "3")}
    mdir, ldir = Path(a.models), Path(a.logo)
    main = {
        f"{k.upper()} s{f.stem.split('_s')[1]}": ML.predict_np(ML.load(f), te)
        for k in ("mlp", "gru")
        for f in sorted(mdir.glob(f"{k}_s*.npz"))
    }
    if a.noaug:  # ablation: MLPs trained without the yaw/scale augmentation
        main.update(
            {
                f"MLPna s{f.stem.split('_s')[1]}": ML.predict_np(ML.load(f), te)
                for f in sorted(Path(a.noaug).glob("mlp_s*.npz"))
            }
        )
    cell = lambda r: (
        f"{r['rmse'][0]:.3f} / {r['rmse'][2]:.3f} / {r['rmse'][4]:.3f} | {r['nll'].mean():.2f} | "
        f"{r['c50'].mean():.3f} | {r['c90'].mean():.3f} ({r['c90'].min():.2f}-{r['c90'].max():.2f}) | {r['d2'].mean():.2f}"
    )
    hdr = [
        "| group / scope | model | RMSE m @0.2 / 0.6 / 1.0 s | NLL (mean h) | cover 50 % | cover 90 % (min-max over h) | d2/3 |",
        "|---|---|---|---|---|---|---|",
    ]
    md, js, passes = [], {}, {}

    def ev(m, c, mask):
        return M.evaluate(m[mask], c[mask], y[mask])

    def seed_mean(res, kind):
        ks = [k for k in res if k.startswith(kind + " s")]
        return {q: np.mean([res[k][q] for k in ks], 0) for q in res[ks[0]]}, len(ks)

    def paired(m, c, mask, ref):
        m0, c0 = IMM[ref]
        nm, n0 = _nll(m, c, y), _nll(m0, c0, y)
        cells = [paired_ci(nm[mask], n0[mask], grp[mask])] + [
            paired_ci(_sqerr(m, y, h)[mask], _sqerr(m0, y, h)[mask], grp[mask])
            for h in (2, 4)
        ]
        return " | ".join(
            f"{x[0]:+.3f} [{x[1]:+.3f}, {x[2]:+.3f}]" for x in cells
        ), cells

    # --- in-distribution (main models saw the group in training)
    md += ["### In-distribution (main models, test split)", ""] + hdr
    scopes = [("pooled", np.ones(len(y), bool))] + [
        (g, g_of == i) for i, g in enumerate(D.GROUPS)
    ]
    scopes += [(f"label {n}", lab == i) for i, n in enumerate(_labels())]
    for sname, mask in scopes:
        if not mask.any():
            continue
        res = {n: ev(*v, mask) for n, v in {**IMM, **main}.items()}
        rows = {k: res[k] for k in IMM}
        for kind in ("MLP", "GRU", "MLPna"):
            if any(k.startswith(kind + " s") for k in res):
                rows[f"{kind} (mean of {seed_mean(res, kind)[1]} seeds)"] = seed_mean(
                    res, kind
                )[0]
        md += [
            f"| {sname} (n={int(mask.sum())}) | {n} | {cell(r)} |"
            for n, r in rows.items()
        ]
        js[f"indist/{sname}"] = {
            n: {k: v.tolist() for k, v in r.items()} for n, r in rows.items()
        }
    # --- leave-one-generator-out
    md += [
        "",
        "### Leave-one-generator-out (MLP trained without the group, tested on it; test split)",
        "",
    ] + hdr
    ci_rows = []
    # route and swarm overlap by construction (a swarm follows a leader route), so route+swarm is held out together as well
    for g in D.GROUPS + ("route+swarm",):
        files = sorted((ldir / f"x_{g}").glob("mlp_s*.npz"))
        if not files:
            continue
        mask = np.isin(g_of, [D.GROUPS.index(x) for x in g.split("+")])
        per = {f.stem: ML.predict_np(ML.load(f), te) for f in files}
        res = {n: ev(*v, mask) for n, v in per.items()}
        mean_r, ns = seed_mean(
            {k.replace("mlp_", "MLP "): v for k, v in res.items()}, "MLP"
        )
        i2, i3 = ev(*IMM["IMM2"], mask), ev(*IMM["IMM3"], mask)
        ok = []
        for f_, r in res.items():
            cover = PASS_COVER[0] <= r["c90"].mean() <= PASS_COVER[1]
            cover_all = bool(
                ((r["c90"] >= PASS_COVER[0]) & (r["c90"] <= PASS_COVER[1])).all()
            )
            rm = r["rmse"][4] <= i2["rmse"][4]
            ok.append((cover, cover_all, rm))
        passes[g] = ok
        md += [
            f"| {g} held out (n={int(mask.sum())}) | MLP LOGO (mean of {ns} seed{'s' * (ns > 1)}) | {cell(mean_r)} |"
        ]
        md += [
            f"| {g} held out | IMM2 | {cell(i2)} |",
            f"| {g} held out | IMM3 | {cell(i3)} |",
        ]
        if "MLP s0" in main:
            md += [
                f"| {g} held out | MLP main (saw {g}) | {cell(ev(*main['MLP s0'], mask))} |"
            ]
        js[f"logo/{g}"] = {
            "mlp": {k: v.tolist() for k, v in mean_r.items()},
            "imm2": {k: v.tolist() for k, v in i2.items()},
            "imm3": {k: v.tolist() for k, v in i3.items()},
        }
        m0, c0 = per[files[0].stem]
        for ref in ("IMM2", "IMM3"):
            txt, cells = paired(m0, c0, mask, ref)
            ci_rows.append(f"| {g} | LOGO MLP seed 0 - {ref} | {txt} |")
        for ref in ("IMM2", "IMM3"):
            if "MLP s0" in main:
                txt, _ = paired(*main["MLP s0"], mask, ref)
                ci_rows.append(f"| {g} | main MLP s0 (in-dist) - {ref} | {txt} |")
    for ref in ("IMM2", "IMM3"):  # pooled in-distribution
        for nm in ("MLP s0", "GRU s0"):
            if nm in main:
                txt, _ = paired(*main[nm], np.ones(len(y), bool), ref)
                ci_rows.append(f"| pooled | main {nm} (in-dist) - {ref} | {txt} |")
    md += [
        "",
        "### Paired differences (bootstrap over runs, 95 % CI; negative = learned better)",
        "",
        "| group | comparison | d NLL (mean h) | d sq. err m^2 @0.6 s | d sq. err m^2 @1.0 s |",
        "|---|---|---|---|---|",
    ] + ci_rows
    md += [
        "",
        f"### Pass criterion (fire-control usable): held-out 90 % cover (mean over horizons) in {list(PASS_COVER)} AND RMSE@1.0 s <= IMM2",
        "",
        "| group | seeds | cover in range (mean h) | cover in range at EVERY h | RMSE@1.0 s <= IMM2 | PASS (mean-h rule) | PASS (every-h rule) |",
        "|---|---|---|---|---|---|---|",
    ]
    for g, ok in passes.items():
        n = len(ok)
        c1, c2, r1 = (sum(o[j] for o in ok) for j in range(3))
        p1, p2 = sum(o[0] and o[2] for o in ok), sum(o[1] and o[2] for o in ok)
        md.append(
            f"| {g} | {n} | {c1}/{n} | {c2}/{n} | {r1}/{n} | {p1}/{n} | {p2}/{n} |"
        )
    text = "\n".join(md)
    print(text)
    if a.md:
        Path(a.md).write_text(text + "\n")
    if a.json:
        Path(a.json).write_text(json.dumps(js))


def _labels():
    from napl.threats import LABELS

    return LABELS


def cmd_bench(a):
    ds = D.load(a.data)
    te = sub(ds["test"], np.arange(len(ds["test"]["y"])) % 50 == 0)
    te = {k: v[:300] for k, v in te.items()}
    one = lambda i: {k: v[i : i + 1] for k, v in te.items()}
    res = {}
    for kind in ("mlp", "gru"):
        W = ML.load(Path(a.models) / f"{kind}_s0.npz")
        ts = []
        for r in range(3):
            for i in range(len(te["y"])):
                d = one(i)
                t0 = time.perf_counter()
                ML.predict_np(W, d)
                ts.append(time.perf_counter() - t0)
        res[f"{kind} featurize+forward+decode (numpy, 1 thread)"] = 1e6 * np.median(ts)
        res[f"{kind} parameters"] = sum(
            int(np.size(v))
            for k, v in W.items()
            if k[0] in "wb" and k not in ("val_nll", "best_epoch")
        )
    from napl.track.scenarios import build
    from napl.track.variants import make_tracker

    scn = build("cv", 1)
    tr = make_tracker("imm2-gnn", scn, 1)
    for t, meas, zs in scn.frames[:40]:
        trk = tr.step(t, zs, meas)
    ts = []
    for _ in range(300):
        t0 = time.perf_counter()
        D._extrap(trk[0], t, [2, 4, 6, 8, 10], scn.dt)
        ts.append(time.perf_counter() - t0)
    res["IMM2 extrapolation, 5 horizons (copy + predict + pv each)"] = 1e6 * np.median(
        ts
    )
    try:
        import torch

        torch.set_num_threads(1)
    except ImportError:
        pass
    for k, v in res.items():
        print(f"{k}: {v:.1f}" + ("" if "parameters" in k else " us"))


CALIB_VARIANTS = {  # columns of napl.track.calib.features used
    "IMM2 raw": None,
    "scalar s": [0],
    "s(h)": [0, 1, 2],
    "s(h, mode)": [0, 1, 2, 3, 4],
    "s(h, NIS)": [0, 1, 2, 5, 6],
    "s(h, mode, NIS)": [0, 1, 2, 3, 4, 5, 6],
    "s(h, accel)": [0, 1, 2, 7, 8],
    "s(h, mode, accel)": [0, 1, 2, 3, 4, 7, 8],
    "s(h, mode, NIS, accel)": list(range(9)),
}


def cmd_calib(a):
    """Mode / NIS-conditioned covariance calibration of IMM2: fit on val, report on test (in-distribution and leave-one-kind-out)."""
    ds = D.load(a.data)
    va, te = ds["val"], ds["test"]
    pre = {}
    for nm, d in (("val", va), ("test", te)):
        m, c = base(d, "2")
        n = C.nis_feature(d["hq"][:, -a.k :])
        dt = d["meta"][:, 4]
        ka = np.minimum(
            np.round(0.5 / dt).astype(int), d["hm"].shape[1] - 1
        )  # ~0.5 s of velocity history
        vp = d["hm"][np.arange(len(dt)), d["hm"].shape[1] - 1 - ka, 3:6]
        acc = C.accel_feature(d["hm"][:, -1, 3:6], vp, ka * dt)
        phi = C.features(
            d["hu"][:, -1, 1][:, None], n[:, None], np.array(HZ)[None], acc[:, None]
        )  # (S, H, 9)
        pre[nm] = (m, c, phi, M.mahal2(m, c, d["y"])[0], d["y"])
    kv, kt = kinds(va), kinds(te)
    D2 = {"val": pre["val"][3], "test": pre["test"][3]}

    def fitw(cols, vmask):
        return C.fit(pre["val"][2][vmask][..., cols], D2["val"][vmask])

    def apply(w, cols, tmask):
        m, c, phi, _, y = pre["test"]
        s = (
            np.exp(phi[tmask][..., cols] @ w)
            if w is not None
            else np.ones(phi[tmask].shape[:2])
        )
        return M.evaluate(m[tmask], c[tmask] * s[..., None, None], y[tmask])

    out = {}  # (scope, model) -> metrics
    allv = np.ones(len(kv), bool)
    for scope in ("fit on all val kinds", "leave-one-kind-out (fit excludes the kind)"):
        for name, cols in CALIB_VARIANTS.items():
            for i, kn in enumerate(NAMES):
                tm = kt == i
                if cols is None:
                    w = None
                elif scope.startswith("fit"):
                    w = fitw(cols, allv)
                else:
                    w = fitw(cols, kv != i)
                out[(scope, name, kn)] = apply(w, cols, tm)
        if scope.startswith(
            "fit"
        ):  # oracle: per-kind s(h) fitted on the kind's val samples (needs the kind)
            for i, kn in enumerate(NAMES):
                out[(scope, "oracle s(h) per kind", kn)] = apply(
                    fitw([0, 1, 2], kv == i), [0, 1, 2], kt == i
                )
    names = list(CALIB_VARIANTS) + ["oracle s(h) per kind"]
    md = []
    for scope in ("fit on all val kinds", "leave-one-kind-out (fit excludes the kind)"):
        md += [
            f"#### {scope}",
            "",
            "cover 50 % / cover 90 % (mean over the 5 horizons) / cover 90 % @1.0 s / NLL (mean h); worst = max over kinds of |cover90 - 0.90|",
            "",
            "| model | " + " | ".join(NAMES) + " | worst |",
            "|---|" + "---|" * (len(NAMES) + 1),
        ]
        for nm in names:
            if (scope, nm, NAMES[0]) not in out:
                continue
            cells, worst = [], 0.0
            for kn in NAMES:
                r = out[(scope, nm, kn)]
                worst = max(worst, abs(r["c90"].mean() - 0.9))
                cells.append(
                    f"{r['c50'].mean():.2f} / {r['c90'].mean():.3f} / {r['c90'][4]:.3f} / {r['nll'].mean():.2f}"
                )
            md.append(f"| {nm} | " + " | ".join(cells) + f" | {worst:.3f} |")
        md.append("")
    for nm, cols in (
        ("s(h, mode, NIS, accel)", range(9)),
        ("s(h, accel)", [0, 1, 2, 7, 8]),
    ):
        w = np.zeros(9)
        w[list(cols)] = fitw(list(cols), allv)
        md.append(
            f"Fitted w (all val kinds, {nm}, 9 columns of calib.features): `{np.array2string(w, precision=4, separator=', ')}`"
        )
    md.append(f"NIS window k = {a.k}")
    text = "\n".join(md)
    print(text)
    if a.md:
        Path(a.md).write_text(text + "\n")


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    g = sp.add_parser("gen")
    g.add_argument("--out", required=True)
    g.add_argument("--train", type=int, default=160)
    g.add_argument("--val", type=int, default=40)
    g.add_argument("--test", type=int, default=60)
    g.add_argument("--jobs", type=int, default=2)
    g.add_argument("--suite", default="track", choices=("track", "threats"))
    g.add_argument(
        "--splits", default="train,val,test"
    )  # one file per split survives a killed job
    g.add_argument("--stride", type=int, default=1)  # keep every n-th scan
    g.add_argument(
        "--rand-mult", type=int, default=1
    )  # threats: random* families run this many times more seeds
    t = sp.add_parser("train")
    t.add_argument(
        "--exclude-group", default=""
    )  # a GROUPS name, or names joined with +
    t.add_argument("--aug", type=int, default=0)
    t.add_argument(
        "--first-seed", type=int, default=0
    )  # resume a killed multi-seed run
    t.add_argument("--suite", default="track")  # informational; the data decide
    t.add_argument("--data", required=True)
    t.add_argument("--out", required=True)
    t.add_argument("--seeds", type=int, default=3)
    t.add_argument("--epochs", type=int, default=40)
    t.add_argument("--kinds", default="mlp,gru")
    t.add_argument("--exclude", default="")
    t.add_argument("--device", default="cuda")
    e = sp.add_parser("eval")
    e.add_argument("--data", required=True)
    e.add_argument("--models", required=True)
    e.add_argument("--heldout", default="")
    e.add_argument("--md", default="")
    e.add_argument("--json", default="")
    et = sp.add_parser("eval-threats")
    et.add_argument("--data", required=True)
    et.add_argument("--models", default="")
    et.add_argument(
        "--hybrid", action="store_true"
    )  # ML-4 hybrids (learned mean + IMM2-floored covariance)
    et.add_argument("--val", default="")  # val data npz (hybrid fit)
    et.add_argument(
        "--fit-stride", type=int, default=2
    )  # hybrid fit uses every n-th val sample
    et.add_argument("--noaug", default="")
    et.add_argument("--logo", required=True)  # dir holding x_<group>/mlp_s*.npz
    et.add_argument("--md", default="")
    et.add_argument("--json", default="")
    b = sp.add_parser("bench")
    b.add_argument("--data", required=True)
    b.add_argument("--models", required=True)
    c = sp.add_parser("calib")
    c.add_argument("--data", required=True)
    c.add_argument("--k", type=int, default=C.K_NIS)
    c.add_argument("--md", default="")
    a = ap.parse_args()
    {
        "gen": cmd_gen,
        "train": cmd_train,
        "eval": cmd_eval,
        "eval-threats": cmd_eval_threats,
        "bench": cmd_bench,
        "calib": cmd_calib,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
