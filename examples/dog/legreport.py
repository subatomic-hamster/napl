"""Leg study report (docs/dog/leg-study.md): tables from the legdiag.py rows of a baseline and its variants.

    python legreport.py --dir logs/leg-eval --vdir variants --base gim8108-r1.5-L220 [--names a,b,c] [--diag] [--md out.md]

* --diag: the step-1 table (fall classification and window statistics of the BASE rows, per family).
* summary: per variant mass, BOM, terrain no-fall k/n per family with Wilson 95 %, route progress, flat push dispersion up k/16, flat top speed, the leg saturation
  of the runs (fraction of control ticks with any joint at 95 % of the torque the driver can deliver, normal operation = baseline window), pack peak current.
* paired: per family the same seeds as the base: runs fixed (base fell, variant did not), broken (the reverse), mean route-progress change, exact two-sided sign
  test on the progress change of the runs where it differs.
"""

import argparse
import json
import sys
from math import comb
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dogmc import wilson

FAMS = ("rubble", "slopes", "scree", "ruins")


def load(d, name):
    f = Path(d) / f"{name}.jsonl"
    rows = [json.loads(x) for x in f.read_text().splitlines()] if f.exists() else []
    out = {}
    for r in rows:
        if "error" not in r and "skipped" not in r:
            out[(r["fam"], r["seed"])] = r  # a re-run replaces
    return out


def fam_rows(rows, fam):
    return sorted(
        (r for (f, _), r in rows.items() if f == fam), key=lambda r: r["seed"]
    )


def pct(w):
    return f"{100 * w[0]:.0f}-{100 * w[1]:.0f} %"


def nofall(rs):
    k = sum(not r["fallen"] for r in rs)
    return k, len(rs), wilson(k, len(rs))


def sign_p(a, b):
    """Exact two-sided sign test on the paired differences b - a (ties dropped)."""
    d = [y - x for x, y in zip(a, b) if abs(y - x) > 1e-9]
    n, k = len(d), sum(x > 0 for x in d)
    if not n:
        return 1.0
    return min(1.0, 2 * sum(comb(n, i) for i in range(min(k, n - k) + 1)) / 2**n)


def sat_of(rs):
    """Mean over runs of the fraction of ticks with any joint saturated in normal operation (the run's baseline window; runs too short for one are skipped)."""
    v = [r["diag"]["base"]["sat_any"] for r in rs if r["diag"].get("base")]
    return float(np.mean(v)) if v else float("nan")


def summary(d, vdir, names):
    L = [
        "| variant | mass kg | BOM USD | "
        + " | ".join(f"{f} no-fall" for f in FAMS)
        + " | terrain mean progress | flat push up | flat top m/s | leg saturation (terrain / flat) | pack peak A |",
        "|---|---|---|" + "---|" * (len(FAMS) + 5),
    ]
    for n in names:
        rows = load(d, n)
        vj = Path(vdir) / n / "variant.json"
        v = json.loads(vj.read_text()) if vj.exists() else {}
        cells = []
        for f in FAMS:
            k, m, w = nofall(fam_rows(rows, f)) if fam_rows(rows, f) else (0, 0, (0, 0))
            cells.append(f"{k}/{m} ({pct(w)})")
        ter = [r for f in FAMS for r in fam_rows(rows, f)]
        fl = fam_rows(rows, "flat")
        k, m, w = nofall(fl) if fl else (0, 0, (0, 0))
        top = rows.get(("top", 1001))
        pk = max([r["diag"]["i_peak_run"] for r in ter + fl] or [0.0])
        L.append(
            f"| {n} | {v.get('mass', float('nan')):.2f} | {v.get('cost', float('nan')):,.0f} | "
            + " | ".join(cells)
            + f" | {np.mean([r['prog'] for r in ter]) if ter else float('nan'):.2f} | {k}/{m} ({pct(w)}) | {top['v_top'] if top else float('nan'):.2f} |"
            + f" {100 * sat_of(ter):.0f} % / {100 * sat_of(fl):.0f} % | {pk:.0f} |"
        )
    return L


def paired(d, base, names):
    b = load(d, base)
    L = [
        "| variant | family | n | fixed | broken | mean progress base -> variant | sign test p (progress) |",
        "|---|---|---|---|---|---|---|",
    ]
    for n in names:
        if n == base:
            continue
        v = load(d, n)
        for f in (*FAMS, "flat"):
            seeds = sorted(
                {s for (g, s) in b if g == f} & {s for (g, s) in v if g == f}
            )
            if not seeds:
                continue
            fixed = sum(b[(f, s)]["fallen"] and not v[(f, s)]["fallen"] for s in seeds)
            broken = sum(v[(f, s)]["fallen"] and not b[(f, s)]["fallen"] for s in seeds)
            pa, pb = (
                [b[(f, s)]["prog"] for s in seeds],
                [v[(f, s)]["prog"] for s in seeds],
            )
            ta, tb = (
                [b[(f, s)]["t_stop"] for s in seeds],
                [v[(f, s)]["t_stop"] for s in seeds],
            )
            prog = (
                f"{np.mean(pa):.3f} -> {np.mean(pb):.3f}"
                if f != "flat"
                else f"stop time {np.mean(ta):.1f} -> {np.mean(tb):.1f} s"
            )
            L.append(
                f"| {n} | {f} | {len(seeds)} | {fixed} | {broken} | {prog} | {sign_p(pa, pb) if f != 'flat' else sign_p(ta, tb):.3f} |"
            )
    return L


def diag(d, base):
    rows = load(d, base)
    L = ["| family | n | falls | actuator-limited | kinematic | control / planning | sat. W_on | sat. baseline window | sat. flat runs that stayed up | range-stop W_on | speed-above-corner knee W_on | slips (falls) | WBC relaxed ticks (W_on) |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]  # fmt: skip
    ref = sat_of([r for r in fam_rows(rows, "flat") if not r["fallen"]])
    for f in (*FAMS, "flat"):
        rs = fam_rows(rows, f)
        fell = [r for r in rs if r["fallen"]]
        c = {
            k: sum(r["diag"].get("cls") == k for r in fell)
            for k in ("actuator", "kinematic", "control")
        }
        w = [r["diag"]["w_on"] for r in fell if r["diag"].get("w_on")]
        m = lambda key: f"{100 * np.mean([x[key] for x in w]):.0f} %" if w else "-"
        sb = sat_of(fell)
        L.append(f"| {f} | {len(rs)} | {len(fell)} | {c['actuator']} | {c['kinematic']} | {c['control']} | {m('sat_any')} | {100 * sb:.0f} % | {100 * ref:.0f} % | {m('rng_any')} | "
                 f"{100 * np.mean([x['vel'][2] for x in w]) if w else 0:.0f} % | {sum(r['n_slip'] for r in fell)} | {sum(x['relaxed'] for x in w)} |")  # fmt: skip
    return L


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", required=True)
    ap.add_argument("--vdir", default="")
    ap.add_argument("--base", required=True)
    ap.add_argument("--names", default="")
    ap.add_argument("--diag", action="store_true")
    ap.add_argument("--md", default="")
    a = ap.parse_args()
    names = [a.base, *filter(None, a.names.split(","))]
    L = (["## Diagnosis (base)", "", *diag(a.dir, a.base), ""] if a.diag else []) + [
        "## Summary",
        "",
        *summary(a.dir, a.vdir, names),
        "",
        "## Paired vs base",
        "",
        *paired(a.dir, a.base, names),
    ]
    sys.stdout.write("\n".join(L) + "\n")
    if a.md:
        Path(a.md).write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
