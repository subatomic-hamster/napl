#!/usr/bin/env python3
"""Interference check of the "-al" leg (docs/dog/leg-survey.md): boolean intersection volumes (mm3) between the knee push rod (10 mm bar + 16 x 8 mm rod ends at
the real four-bar positions, y = 25) / the shank head, tube and foot cap and the thigh's plates and printed bands, at 9 knee angles across the whole knee range.
Needs build123d (the cad venv on the build host):  cd src && ../cad-venv/bin/python tools/dog_leg_clearance.py   Exit code 1 if any pair overlaps by > 1 mm3."""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cad", "dog"))
sys.path.insert(0, HERE)
import dog_leg_struct as S
import params as P
import parts_al as A
from build123d import Cylinder, Pos, Rot

AL, PA = A._thigh_split()
HEAD, TUBE, FOOT = A._shank_head(), A._shank_tube(), A.foot_core_cf()


def vol(a, b):
    try:
        r = a & b
        return float(r.volume) if r is not None else 0.0
    except Exception:  # an empty boolean result raises in some OCC versions
        return 0.0


def bar(p0, p1, d, y):
    n = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    th = math.degrees(math.atan2(p1[0] - p0[0], p1[1] - p0[1]))
    return (
        Pos((p0[0] + p1[0]) / 2, y, (p0[1] + p1[1]) / 2)
        * Rot(0, th, 0)
        * Cylinder(d / 2, n)
    )


def main():
    lo, hi = P.linkage(P.L_THIGH)["knee_range"]
    worst = {}
    for i in range(9):
        q = lo + (hi - lo) * i / 8
        (lx, lz), (cx, cz), _ = S.rod_geometry(q)
        p0, p1 = (lx * 1e3, lz * 1e3), (cx * 1e3, cz * 1e3)
        rod = (
            bar(p0, p1, 10, P.ROD_Y)
            + A.cyl_y(16, 8, x=p0[0], y=P.ROD_Y, z=p0[1])
            + A.cyl_y(16, 8, x=p1[0], y=P.ROD_Y, z=p1[1])
        )
        sh = Pos(0, P.SH_Y, -P.L_THIGH) * Rot(0, math.degrees(q), 0)
        head, tube, foot = sh * HEAD, sh * TUBE, sh * Pos(0, 0, -P.L_SHANK - 25) * FOOT
        row = {
            "rod/plates": vol(rod, AL), "rod/bands": vol(rod, PA), "rod/head": vol(rod, head), "rod/tube": vol(rod, tube),
            "head/bands": vol(head, PA), "tube/bands": vol(tube, PA), "head/plates": vol(head, AL), "tube/plates": vol(tube, AL),
            "foot/plates": vol(foot, AL), "foot/bands": vol(foot, PA),
        }  # fmt: skip
        sys.stdout.write(
            f"q {q:+.2f}  " + " ".join(f"{k} {v:.1f}" for k, v in row.items()) + "\n"
        )
        for k, v in row.items():
            worst[k] = max(worst.get(k, 0.0), v)
    sys.stdout.write(
        "worst overlap per pair (mm3): "
        + str({k: round(v, 1) for k, v in worst.items()})
        + "\n"
    )
    return 1 if any(v > 1.0 for v in worst.values()) else 0


if __name__ == "__main__":
    sys.exit(main())
