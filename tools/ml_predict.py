#!/usr/bin/env python3
"""Learned motion prediction for fire-control lead (docs/cuas2/ml-prediction.md).

  python tools/ml_predict.py gen   --out DATA.npz [--train 160 --val 40 --test 60 --jobs 2]
  python tools/ml_predict.py train --data DATA.npz --out DIR [--seeds 3 --epochs 40] [--exclude KIND]
  python tools/ml_predict.py eval  --data DATA.npz --models DIR [--md OUT.md --json OUT.json] [--heldout DIR2]
  python tools/ml_predict.py bench --data DATA.npz --models DIR

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
    ds = D.build_dataset(sp, jobs=a.jobs)
    D.save(a.out, ds)
    print({s: len(d["y"]) for s, d in ds.items()}, f"{time.time() - t:.0f} s")


def cmd_train(a):
    ds = D.load(a.data)
    tr, va = ds["train"], ds["val"]
    if a.exclude:
        k = NAMES.index(a.exclude)
        tr, va = sub(tr, kinds(tr) != k), sub(va, kinds(va) != k)
    Path(a.out).mkdir(parents=True, exist_ok=True)
    for kind in a.kinds.split(","):
        for s in range(a.seeds):
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
    t = sp.add_parser("train")
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
        "bench": cmd_bench,
        "calib": cmd_calib,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
