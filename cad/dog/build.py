#!/usr/bin/env python3
"""Build the dog's printed parts (run on the build host with the cad venv; never on the laptop):

    cad-venv/bin/python cad/dog/build.py [--out build/cad/dog]

Writes <out>/napl/<name>[-l|-r].stl (metres, part frame) and <out>/manifest.json (volume, mass, bbox, COM per part, with the
shell + infill mass model of assemblies/README.md "Mass properties"). Copy the napl/*.stl to meshes/dog/ and the numbers into
the part TOMLs. Handed parts are built as the left (+y) version and mirrored about the XZ plane for the right.
"""

import argparse
import json
import sys
from pathlib import Path

import trimesh
from build123d import Plane, export_stl

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P

import parts as K


def tess(shape, path):
    export_stl(shape, str(path), tolerance=0.01, angular_tolerance=0.15)
    m = trimesh.load(str(path), force="mesh", process=True)
    m.merge_vertices()
    return m


def record(m, material):
    V, A = float(m.volume), float(m.area)
    shell = min(V, A * P.WALLS * P.NOZZLE)
    fil = shell + P.INFILL * (V - shell)
    rho = P.DENSITY[material]
    ext = m.extents
    return dict(
        volume_mm3=round(V, 1),
        filament_g=round(fil * rho, 2),
        solid_g=round(V * rho, 2),
        print_time_h=round(fil / P.FLOW_MM3_S * P.TIME_OVERHEAD / 3600, 2),
        bbox_min_mm=[round(float(v), 2) for v in m.bounds[0]],
        bbox_max_mm=[round(float(v), 2) for v in m.bounds[1]],
        com_mm=[round(float(v), 2) for v in m.center_mass],
        watertight=bool(m.is_watertight),
        fits_bed=bool(
            sorted(ext)[2] <= 300 and sorted(ext)[1] <= 200
        ),  # long axis printed on the bed diagonal (220 x 220 bed -> 311 mm),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="build/cad/dog")
    out = Path(ap.parse_args().out)
    (out / "napl").mkdir(parents=True, exist_ok=True)
    man = {
        "params": dict(L_THIGH=P.L_THIGH, L_SHANK=P.L_SHANK, TILT_DEG=P.TILT_DEG),
        "parts": {},
    }
    for name, (fn, count, mat, handed) in K.REGISTRY.items():
        shape = fn()
        variants = (
            {name + "-l": shape, name + "-r": shape.mirror(Plane.XZ)}
            if handed
            else {name: shape}
        )
        for vn, sh in variants.items():
            tmp = out / "napl" / f"{vn}.mm.stl"
            m = tess(sh, tmp)
            tmp.unlink()
            r = record(m, mat)
            r.update(count=count // 2 if handed else count, material=mat)
            m.apply_scale(1e-3)
            m.export(str(out / "napl" / f"{vn}.stl"))
            man["parts"][vn] = r
            print(
                f"{vn:28s} V={r['volume_mm3'] / 1e3:8.1f}cm3 fil={r['filament_g']:7.1f}g wt={r['watertight']} bed={r['fits_bed']}"
            )
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    tot = sum(r["filament_g"] * r["count"] for r in man["parts"].values())
    print(f"TOTAL filament {tot:.0f} g")
    return 0 if all(r["watertight"] for r in man["parts"].values()) else 1


if __name__ == "__main__":
    sys.exit(main())
