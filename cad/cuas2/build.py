#!/usr/bin/env python3
"""Regenerate everything for the cuas2 turret CAD with one command (run on the build host: the laptop has no build123d):

    cad-venv/bin/python cad/cuas2/build.py [--out build/cad/cuas2] [--only NAME ...] [--no-checks] [--no-preview] [--meshes DIR]

Per printed part under --out (gitignored, never commit): step/, stl/ (print orientation), napl/ (metres, design frame) + json (mass, COM,
inertia), parts/<part>.json, manifest.json (totals, bought items, interface poses, check results). --meshes DIR also writes the napl
meshes (binary STL, metres) to DIR (the repo's meshes/cuas2 for the assembly). Exit 0 and a final PASS line only if every check passes.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import trimesh
from build123d import Pos, export_step, export_stl

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import params as P
from modules import bought_items, registry

UPS = [(0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
SHELL = P.WALLS * P.NOZZLE


def tessellate(shape, path):
    export_stl(shape, str(path), tolerance=0.02, angular_tolerance=0.2)
    m = trimesh.load(str(path), force="mesh", process=True)
    m.merge_vertices()
    return m


def mass_model(m, material):
    V, A = float(m.volume), float(m.area)
    shell = min(V, A * SHELL)
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
    best = None
    for d in UPS:
        R = trimesh.geometry.align_vectors(d, [0, 0, 1])
        mm = m.copy()
        mm.apply_transform(R)
        lo, hi = mm.bounds
        ext = hi - lo
        fits = (
            ext[0] <= P.PRINT_MAX + 1e-6
            and ext[1] <= P.PRINT_MAX + 1e-6
            and ext[2] <= P.PRINT_MAX + 1e-6
        )
        n, a = mm.face_normals, mm.area_faces
        zc = mm.triangles_center[:, 2] - lo[2]
        down, flat, bed = n[:, 2] < -0.72, n[:, 2] < -0.99, zc < 0.05
        contact, bridge = float(a[flat & bed].sum()), float(a[flat & ~bed].sum())
        over = float(a[down & ~flat & ~bed].sum())
        score = over + 0.25 * bridge - 0.01 * contact + (0 if fits else 1e9)
        if best is None or score < best[0]:
            T2 = np.eye(4)
            T2[:3, 3] = -mm.bounds[0]
            best = (score, d, T2 @ R, over, bridge, contact, ext, fits)
    _, d, T, over, bridge, contact, ext, fits = best
    foot = ext[0] * ext[1]
    supports = (
        "none"
        if over < 80 and bridge < 800
        else ("minor" if over + bridge <= 0.3 * foot else "yes")
    )
    return dict(
        up_in_design=list(d),
        size_mm=[round(float(e), 2) for e in ext],
        fits_bed=bool(fits),
        overhang_mm2=round(over, 1),
        bridge_mm2=round(bridge, 1),
        supports=supports,
    ), T


def build_one(name, rec, out, S, meshdir):
    t0 = time.time()
    L = rec["fn"]()  # authored frame (L for tilt group, B otherwise)
    S[name] = L
    shape = Pos(-P.X_A, 0, -P.Z_A) * L if rec["group"] == "tilt" else L
    export_step(shape, str(out / "step" / f"{name}.step"))
    m = tessellate(shape, out / "stl" / f"{name}.design.stl")
    props = mass_model(m, rec["material"])
    com = m.center_mass
    inertia = (m.moment_inertia * props["rho_eff"] * 1e-3 * 1e-6).tolist()
    ori, T = orient(m)
    mp = m.copy()
    mp.apply_transform(T)
    mp.export(str(out / "stl" / f"{name}.stl"))
    (out / "stl" / f"{name}.design.stl").unlink()
    ms = m.copy()
    ms.apply_scale(1e-3)
    ms.export(str(out / "napl" / f"{name}.stl"))
    if meshdir:
        ms.export(str(meshdir / f"{name}.stl"))
    off = np.array([P.X_A, 0.0, P.Z_A]) if rec["group"] == "tilt" else np.zeros(3)
    r = dict(
        name=name,
        group=rec["group"],
        module=rec["module"],
        count=rec["count"],
        material=rec["material"],
        at=rec.get("at"),
        frame={
            "static": "B",
            "pan": "B (rotates with pan)",
            "tilt": "H (tilt axis origin)",
        }[rec["group"]],
        valid=bool(shape.is_valid),
        watertight=bool(m.is_watertight),
        n_solids=len(shape.solids()),
        volume_mm3=round(props["V"], 1),
        solid_mass_g=round(props["solid_g"], 2),
        filament_g=round(props["fil_g"], 2),
        print_time_h=round(props["time_h"], 2),
        bbox_mm=dict(
            min=[round(float(v), 3) for v in m.bounds[0]],
            max=[round(float(v), 3) for v in m.bounds[1]],
        ),
        com_mm=[round(float(v), 3) for v in com],
        com_mm_L=[round(float(v), 3) for v in (com + off)],
        inertia_kgm2_about_com=inertia,
        print=ori,
    )
    (out / "parts" / f"{name}.json").write_text(json.dumps(r, indent=1))
    (out / "napl" / f"{name}.json").write_text(
        json.dumps(
            dict(
                mesh=f"{name}.stl",
                units="m",
                frame=r["frame"],
                group=rec["group"],
                mass_kg=props["fil_g"] * 1e-3,
                solid_mass_kg=props["solid_g"] * 1e-3,
                com_m=[float(v) * 1e-3 for v in com],
                inertia_kgm2_about_com=inertia,
                count=rec["count"],
            ),
            indent=1,
        )
    )
    print(
        f"{name:16s} V={props['V'] / 1e3:8.1f}cm3 fil={props['fil_g']:6.1f}g t={props['time_h']:5.2f}h size={ori['size_mm']} sup={ori['supports']} wt={r['watertight']} ({time.time() - t0:.1f}s)",
        flush=True,
    )
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="build/cad/cuas2")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--no-checks", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--meshes", default=None)
    a = ap.parse_args()
    out = Path(a.out)
    for d in ("step", "stl", "napl", "parts"):
        (out / d).mkdir(parents=True, exist_ok=True)
    meshdir = Path(a.meshes) if a.meshes else None
    if meshdir:
        meshdir.mkdir(parents=True, exist_ok=True)
    reg = registry()
    names = a.only or list(reg)
    recs, S = {}, {}
    for n in names:
        recs[n] = build_one(n, reg[n], out, S, meshdir)
    tot_g = sum(r["filament_g"] * r["count"] for r in recs.values())
    tot_h = sum(r["print_time_h"] * r["count"] for r in recs.values())
    bought = bought_items()
    manifest = dict(
        generated_by="cad/cuas2/build.py",
        params=dict(
            ball_d=P.BALL_D,
            wheel_d=P.WHEEL_D,
            gate_swing_deg=P.GATE_SWING_DEG,
            pocket_len=P.POCKET_LEN,
            rpm_cap=P.RPM_CAP,
            x_a=P.X_A,
            z_a=P.Z_A,
            ballast_g=P.BALLAST_G,
            h_t=P.H_T,
        ),
        totals=dict(
            filament_g=round(tot_g, 1),
            cost_usd=round(tot_g / 1000 * P.COST_PER_KG, 2),
            print_time_h=round(tot_h, 1),
            usd_per_kg=P.COST_PER_KG,
        ),
        parts={
            n: {
                k: r[k]
                for k in (
                    "module",
                    "group",
                    "count",
                    "material",
                    "filament_g",
                    "print_time_h",
                    "com_mm",
                    "bbox_mm",
                    "at",
                )
            }
            | dict(size_print_mm=r["print"]["size_mm"], supports=r["print"]["supports"])
            for n, r in recs.items()
        },
        bought=[
            dict(
                name=b["name"],
                lib=b["lib"],
                mass_g=b["mass"],
                group=b["group"],
                module=b["module"],
                optional=b.get("optional", False),
                com_L_or_B_mm=[round(v, 2) for v in tuple(b["shape"].center())],
            )
            for b in bought
        ],
    )
    print(
        f"TOTAL filament {tot_g:.0f} g  cost ${tot_g / 1000 * P.COST_PER_KG:.2f}  print time {tot_h:.0f} h"
    )
    ok = True
    if not a.no_checks and not a.only:
        import checks

        res = checks.run(S, recs, bought)
        manifest["checks"] = res
        ok = all(c["pass"] for c in res.values())
        ok = ok and all(c["pass"] for c in res.values())
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1, default=float))
    if not a.no_preview and not a.only:
        import preview

        preview.run(out, S, recs, bought)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
