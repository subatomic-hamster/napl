#!/usr/bin/env python3
"""Visual coverage of the parts library by category prefix (first two category segments).

A geometry-bearing part is 'covered' when it has an explicit [visual], or its category default is a purpose-built family
(anything but the `shapes` fallback that only redraws the primitives with soft edges). Parts without [geometry] are exempt
and listed with --exempt. Exit 1 when coverage of geometry-bearing parts is below --min (percent).

    .venv/bin/python tools/visual_coverage.py [--min 95] [--exempt] [--uncovered]
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
from napl.design import visuals
from napl.design.library import Library


def classify(part):
    g = part.geometry
    if part.raw.get("visual") is not None:
        return "explicit"
    fam = visuals.default_family(part)
    if fam and fam != "shapes":
        return "family"  # (a part without [geometry] is drawn by thumbnails / library page only, never in a build)
    if any(s.get("shape") == "mesh" for s in g.get("shapes") or [g]):
        return "family"  # vendor / printed-part CAD mesh redrawn with a print finish: the real shape already
    return "fallback" if (g.get("shapes") or "shape" in g) else "exempt"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min", type=float, default=0.0)
    ap.add_argument("--exempt", action="store_true")
    ap.add_argument("--uncovered", action="store_true")
    a = ap.parse_args()
    lib = Library(REPO / "parts")
    rows = defaultdict(lambda: defaultdict(int))
    lists = defaultdict(list)
    for p in lib.parts.values():
        k = classify(p)
        cat = ".".join(p.category.split(".")[:2])
        rows[cat][k] += 1
        rows["TOTAL"][k] += 1
        lists[k].append(f"{p.id} [{p.category}]")
    print(
        f"{'category':28s} {'explicit':>8s} {'family':>7s} {'fallback':>8s} {'exempt':>6s}  covered%"
    )
    for cat in sorted(rows, key=lambda c: (c == "TOTAL", c)):
        r = rows[cat]
        geo = r["explicit"] + r["family"] + r["fallback"]
        pct = 100.0 * (r["explicit"] + r["family"]) / geo if geo else 100.0
        print(
            f"{cat:28s} {r['explicit']:8d} {r['family']:7d} {r['fallback']:8d} {r['exempt']:6d}  {pct:5.1f}"
        )
    if a.exempt:
        print("\nexempt (no [geometry]):", *lists["exempt"], sep="\n  ")
    if a.uncovered:
        print(
            "\nfallback (geometry, no purpose-built visual):",
            *lists["fallback"],
            sep="\n  ",
        )
    r = rows["TOTAL"]
    geo = r["explicit"] + r["family"] + r["fallback"]
    pct = 100.0 * (r["explicit"] + r["family"]) / geo
    if pct < a.min:
        print(f"coverage {pct:.1f}% < {a.min}%")
        sys.exit(1)


if __name__ == "__main__":
    main()
