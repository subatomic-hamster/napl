#!/usr/bin/env python3
"""Visual coverage of the whole parts library by category prefix (first two category segments).

A part is 'covered' when a build draws it with a purpose-built visual: an explicit [visual], a category-default family
(anything but the `shapes` fallback that only redraws bare primitives) or a redrawn vendor mesh. That holds for parts with and
without [geometry] (a part without one is drawn from its params at the instance frame). Exempt = true non-objects with
nothing to draw: raw materials, OCV curves, wires (drawn as tubes by the wire layer) and aggregate frame-kit data records.
`was` = what a build drew before geometry-less parts were drawn (covered AND has [geometry], or an explicit [visual]).

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

EXEMPT_CATS = (
    "material",
    "battery.cell_curve",
    "wire",
    "frame",
)  # `frame` = the two frame-kit data records, no shape


def has_geometry(part):
    g = part.geometry
    return bool(g.get("shapes") or "shape" in g)


def classify(part):
    """explicit / family (covered), fallback (primitives only), missing (no geometry and nothing to draw), exempt."""
    g = part.geometry
    if part.raw.get("visual") is not None:
        return "explicit"
    if part.category in EXEMPT_CATS or part.category.startswith("wire"):
        return "exempt"
    fam = visuals.default_family(part)
    if fam and fam != "shapes":
        return "family"
    if any(s.get("shape") == "mesh" for s in g.get("shapes") or [g]):
        return "family"  # vendor / printed-part CAD mesh redrawn with a finish: the real shape already
    return "fallback" if has_geometry(part) else "missing"


def was_drawn(part):
    """Whether a build drew a purpose-built visual for the part before geometry-less parts were drawn."""
    return classify(part) in ("explicit", "family") and (
        part.raw.get("visual") is not None or has_geometry(part)
    )


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
        for cat in (".".join(p.category.split(".")[:2]), "TOTAL"):
            rows[cat][k] += 1
            rows[cat]["was"] += was_drawn(p)
        lists[k].append(f"{p.id} [{p.category}]")
    print(
        f"{'category':26s} {'explicit':>8s} {'family':>6s} {'fallback':>8s} {'missing':>7s} {'exempt':>6s}  {'was':>5s} {'now':>5s}  covered%"
    )
    for cat in sorted(rows, key=lambda c: (c == "TOTAL", c)):
        r = rows[cat]
        obj = r["explicit"] + r["family"] + r["fallback"] + r["missing"]
        now = r["explicit"] + r["family"]
        pct = 100.0 * now / obj if obj else 100.0
        print(
            f"{cat:26s} {r['explicit']:8d} {r['family']:6d} {r['fallback']:8d} {r['missing']:7d} {r['exempt']:6d}  {r['was']:5d} {now:5d}  {pct:5.1f}"
        )
    r = rows["TOTAL"]
    obj = r["explicit"] + r["family"] + r["fallback"] + r["missing"]
    print(
        f"\nobjects {obj} (exempt {r['exempt']}): drawn before {r['was']} = {100.0 * r['was'] / obj:.1f} %, "
        f"now {r['explicit'] + r['family']} = {100.0 * (r['explicit'] + r['family']) / obj:.1f} %"
    )
    if a.exempt:
        print("\nexempt (not an object):", *lists["exempt"], sep="\n  ")
    if a.uncovered:
        print("\nmissing (no geometry, no visual):", *lists["missing"], sep="\n  ")
        print(
            "\nfallback (geometry, no purpose-built visual):",
            *lists["fallback"],
            sep="\n  ",
        )
    pct = 100.0 * (r["explicit"] + r["family"]) / obj
    if pct < a.min:
        print(f"coverage {pct:.1f}% < {a.min}%")
        sys.exit(1)


if __name__ == "__main__":
    main()
