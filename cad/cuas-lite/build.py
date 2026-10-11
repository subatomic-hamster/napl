#!/usr/bin/env python3
"""Regenerate everything for the cuas-lite turret CAD with one command (run on the build host, not the laptop):

    cad-venv/bin/python cad/cuas-lite/build.py [--out build/cad/cuas-lite] [--only NAME ...] [--no-checks] [--no-preview]

Per printed part it writes (all under --out, gitignored; never commit them):
    step/<part>.step            STEP, design frame, mm
    stl/<part>.stl              print-ready STL, mm, best of 6 axis orientations (fewest overhangs)
    napl/<part>.stl             napl visual/collision mesh, design frame, metres
    napl/<part>.json            mass / COM / inertia in SI for the assembly worker
    parts/<part>.json           full manifest (volume, mass, COM, inertia, bbox, print time, orientation, supports)
    manifest.json               totals, interface (frames, axes, mount points), check results, parameters
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import trimesh
from build123d import export_step, export_stl

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P

import parts as K

UPS = [
    (0, 0, 1),
    (0, 0, -1),
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
]  # design direction that points up while printing
SHELL = P.WALLS * P.NOZZLE


def tessellate(shape, path):
    export_stl(shape, str(path), tolerance=0.01, angular_tolerance=0.15)
    m = trimesh.load(str(path), force="mesh", process=True)
    m.merge_vertices()
    return m


def mass_model(m, material):
    V, A = float(m.volume), float(m.area)
    shell = min(V, A * SHELL)  # same model as assemblies/README.md "Mass properties"
    fil_v = shell + P.INFILL * (V - shell)
    rho = P.DENSITY[material]
    return dict(
        V=V,
        A=A,
        solid_g=V * rho,
        fil_g=fil_v * rho,
        rho_eff=fil_v * rho / V,
        time_h=fil_v / P.FLOW_MM3_S * P.TIME_OVERHEAD / 3600.0,
    )


def orient(m):
    """Pick the axis-aligned orientation with the least overhang that fits the bed."""
    best = None
    cands = []
    for d in UPS:
        R = trimesh.geometry.align_vectors(d, [0, 0, 1])
        mm = m.copy()
        mm.apply_transform(R)
        lo, hi = mm.bounds
        ext = hi - lo
        fits = (
            ext[0] <= P.BED[0] - 2 * P.BED_MARGIN
            and ext[1] <= P.BED[1] - 2 * P.BED_MARGIN
            and ext[2] <= P.BED[2]
        )
        n, a = mm.face_normals, mm.area_faces
        zc = mm.triangles_center[:, 2] - lo[2]
        # overhang = steeper than ~46 deg from vertical; flat = horizontal ceiling (bridge); bed = on the plate
        down, flat, bed = n[:, 2] < -0.72, n[:, 2] < -0.99, zc < 0.05
        contact = float(a[flat & bed].sum())
        bridge = float(a[flat & ~bed].sum())
        over = float(a[down & ~flat & ~bed].sum())
        score = over + 0.25 * bridge - 0.01 * contact + (0 if fits else 1e9)
        cands.append(
            dict(
                up=list(d),
                overhang_mm2=round(over),
                bridge_mm2=round(bridge),
                contact_mm2=round(contact),
                fits=bool(fits),
            )
        )
        if best is None or score < best[0]:
            mm2 = m.copy()
            mm2.apply_transform(R)
            T2 = np.eye(4)
            T2[:3, 3] = -mm2.bounds[0]
            best = (score, d, T2 @ R, over, bridge, contact, ext, fits)
    score, d, T, over, bridge, contact, ext, fits = best
    foot = ext[0] * ext[1]
    ov = over + bridge
    supports = (
        "none"
        if over < 80 and bridge < 800
        else ("minor" if ov <= 0.3 * foot else "yes")
    )
    return dict(
        up_in_design=list(d),
        transform=T.tolist(),
        size_mm=[round(float(e), 2) for e in ext],
        fits_bed=bool(fits),
        overhang_mm2=round(over, 1),
        bridge_mm2=round(bridge, 1),
        contact_mm2=round(contact, 1),
        supports=supports,
        candidates=cands,
    ), T


def build_one(name, out, shapes):
    fn, group, count, material = K.REGISTRY[name]
    t0 = time.time()
    shape = fn()
    shapes[name] = shape
    export_step(shape, str(out / "step" / f"{name}.step"))
    m = tessellate(shape, out / "stl" / f"{name}.design.stl")
    props = mass_model(m, material)
    com = m.center_mass
    I1 = m.moment_inertia  # density 1 -> about COM, mm^5
    inertia = (I1 * props["rho_eff"] * 1e-3 * 1e-6).tolist()
    ori, T = orient(m)
    mp = m.copy()
    mp.apply_transform(T)
    mp.export(str(out / "stl" / f"{name}.stl"))
    (out / "stl" / f"{name}.design.stl").unlink()
    ms = m.copy()
    ms.apply_scale(1e-3)
    ms.export(str(out / "napl" / f"{name}.stl"))
    rec = dict(
        name=name,
        group=group,
        frame={
            "static": "B",
            "pan": "B (rotates about z with pan)",
            "tilt": "H (tilt axis origin)",
        }[group],
        count=count,
        material=material,
        density_g_cm3=P.DENSITY[material] * 1e3,
        assumptions=dict(
            walls=P.WALLS,
            nozzle_mm=P.NOZZLE,
            infill=P.INFILL,
            layer_mm=P.LAYER,
            flow_mm3_s=P.FLOW_MM3_S,
            time_overhead=P.TIME_OVERHEAD,
            mass_model="shell=min(V,A*walls*nozzle), infill elsewhere (napl assemblies/README.md)",
        ),
        valid=bool(shape.is_valid),
        n_solids=len(shape.solids()),
        watertight=bool(m.is_watertight),
        winding_ok=bool(m.is_winding_consistent),
        volume_mm3=round(props["V"], 1),
        volume_occ_mm3=round(float(shape.volume), 1),
        surface_area_mm2=round(props["A"], 1),
        solid_mass_g=round(props["solid_g"], 2),
        filament_g=round(props["fil_g"], 2),
        print_time_h=round(props["time_h"], 2),
        bbox_design_mm=dict(
            min=[round(float(v), 2) for v in m.bounds[0]],
            max=[round(float(v), 2) for v in m.bounds[1]],
            size=[round(float(v), 2) for v in m.extents],
        ),
        com_mm=[round(float(v), 3) for v in com],
        inertia_kgm2_about_com=inertia,
        print=ori,
        files=dict(
            step=f"step/{name}.step",
            stl=f"stl/{name}.stl",
            napl_mesh=f"napl/{name}.stl",
        ),
    )
    (out / "parts" / f"{name}.json").write_text(json.dumps(rec, indent=1))
    (out / "napl" / f"{name}.json").write_text(
        json.dumps(
            dict(
                mesh=f"{name}.stl",
                units="m",
                frame=rec["frame"],
                group=group,
                mass_kg=props["fil_g"] * 1e-3,
                solid_mass_kg=props["solid_g"] * 1e-3,
                com_m=[float(v) * 1e-3 for v in com],
                inertia_kgm2_about_com=inertia,
                count=count,
            ),
            indent=1,
        )
    )
    print(
        f"{name:20s} V={props['V'] / 1e3:8.1f}cm3 fil={props['fil_g']:7.1f}g t={props['time_h']:5.2f}h size={ori['size_mm']} "
        f"sup={ori['supports']} wt={rec['watertight']} ({time.time() - t0:.1f}s)",
        flush=True,
    )
    return rec, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="build/cad/cuas-lite")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--no-checks", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    a = ap.parse_args()
    out = Path(a.out)
    for d in ("step", "stl", "napl", "parts"):
        (out / d).mkdir(parents=True, exist_ok=True)
    names = a.only or list(K.REGISTRY)
    recs, meshes, shapes = {}, {}, {}
    for n in names:
        recs[n], meshes[n] = build_one(n, out, shapes)
    tot_g = sum(r["filament_g"] * r["count"] for r in recs.values())
    tot_h = sum(r["print_time_h"] * r["count"] for r in recs.values())
    manifest = dict(
        generated_by="cad/cuas-lite/build.py",
        ball=dict(
            d_mm=P.BALL_D,
            m_g=P.BALL_M,
            barrel_id_mm=P.BARREL_ID,
            flywheel_gap_mm=P.FW_GAP,
        ),
        totals=dict(
            filament_g=round(tot_g, 1),
            cost_usd=round(tot_g / 1000 * P.COST_PER_KG, 2),
            print_time_h=round(tot_h, 1),
            usd_per_kg=P.COST_PER_KG,
        ),
        parts={
            n: dict(
                group=r["group"],
                count=r["count"],
                filament_g=r["filament_g"],
                print_time_h=r["print_time_h"],
                size_print_mm=r["print"]["size_mm"],
                supports=r["print"]["supports"],
                material=r["material"],
            )
            for n, r in recs.items()
        },
    )
    print(
        f"TOTAL filament {tot_g:.0f} g  cost ${tot_g / 1000 * P.COST_PER_KG:.2f}  print time {tot_h:.0f} h"
    )
    if not a.no_checks and not a.only:
        import checks

        manifest["checks"] = checks.run(recs, meshes, shapes, out)
        manifest["interface"] = checks.interface(recs)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1, default=float))
    if not a.no_preview and not a.only:
        import preview

        preview.run(out, recs, meshes)
    ok = all(c["pass"] for c in manifest.get("checks", {}).values())
    print("ALL CHECKS PASS" if ok else "CHECK FAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
