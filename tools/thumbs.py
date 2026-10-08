#!/usr/bin/env python3
"""Render every part (its [visual] layer or category default; else the plain [geometry] primitive) to a 256x256 3/4-view
thumbnail: build/thumbs/<part_id>.png, plus build/thumbs/index.json {id: {name, category, vendor, thumb}} for the library page.
Needs a Vulkan device (GPU) -- run on the build host:

    tools/remote.sh run --gpu '.venv/bin/python tools/thumbs.py [--out build/thumbs] [--size 256] [--az 35 --el 25] [ID ...]'

Thumbnails are build output (gitignored), never committed.
"""

import argparse
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
from napl import Sim, _napl
from napl.design import visuals
from napl.design.build import appearance_rgba, shape_frame, vec
from napl.design.library import Library
from napl.design.visuals.stl import read_stl

VERSION = 2  # the look of the tiles; python/napl/design/thumbjob.py compares it with the cache's stamp (a bump re-renders everything)
TARGET_MEAN, MIN_GAMMA = 0.42, 0.45  # brightness lift of dark parts (finish())


def _pose(pos, R):
    from napl.design.build import pose_attrs

    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, pos
    return pose_attrs(T)


def part_scene(lib, part, cache, k=1.0):
    """-> (mjcf geoms text, assets text, (lo, hi) bounds) or None when the part has nothing to draw. Everything is scaled
    by `k` (small parts are enlarged: the renderer clips closer than its near plane)."""
    sc = f'scale="{k} {k} {k}"'
    geoms, assets, lo, hi = [], [], np.full(3, np.inf), np.full(3, -np.inf)

    def grow(pts):
        nonlocal lo, hi
        lo, hi = np.minimum(lo, pts.min(0)), np.maximum(hi, pts.max(0))

    seen = set()
    for p in visuals.pieces(part, cache, lib.root) if visuals.has_visual(part) else []:
        m = read_stl(p.path)
        grow(k * (m.v @ p.R.T + p.pos))
        if p.name not in seen:
            seen.add(p.name)
            assets.append(
                f'<mesh name="v_{p.name}" file="{p.path}" {sc} inertia="convex"/>'
            )
        mat = p.material
        mn = f"m_{mat['name']}"
        if mn not in seen:
            seen.add(mn)
            assets.append(
                f'<material name="{mn}" rgba="{vec(mat["rgba"])}" specular="{mat["specular"]}" shininess="{mat["shininess"]}"'
                f' metallic="{mat["metallic"]}" roughness="{mat["roughness"]}"/>'
            )
        geoms.append(
            f'<geom type="mesh" mesh="v_{p.name}" material="{mn}" rgba="{vec(mat["rgba"])}"{_pose(k * p.pos, p.R)} contype="0" conaffinity="0" mass="0"/>'
        )
    if not geoms:  # primitive from [geometry]
        g = part.geometry
        rgba = vec(appearance_rgba(lib, part))
        for i, s in enumerate(g.get("shapes") or ([g] if "shape" in g else [])):
            kind, sz = s.get("shape"), s.get("size")
            pos, R = shape_frame(s, part.id)
            if kind == "mesh":
                f = lib.root / s["mesh"]
                m = read_stl(f)
                grow(k * (m.v @ R.T + pos))
                assets.append(f'<mesh name="g{i}" file="{f}" {sc} inertia="convex"/>')
                geoms.append(
                    f'<geom type="mesh" mesh="g{i}" rgba="{rgba}"{_pose(k * pos, R)} contype="0" conaffinity="0" mass="0"/>'
                )
            elif kind in ("box", "cylinder", "sphere", "capsule"):
                ext = {
                    "box": sz,
                    "cylinder": [sz[0], sz[0], sz[1]],
                    "sphere": [sz[0]] * 3,
                    "capsule": [sz[0], sz[0], sz[0] + sz[1]],
                }[kind]
                c = (
                    np.array(
                        [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
                    )
                    * ext
                )
                grow(k * (c @ R.T + pos))
                geoms.append(
                    f'<geom type="{kind}" size="{vec(k * np.array(sz))}" rgba="{rgba}"{_pose(k * pos, R)} contype="0" conaffinity="0" mass="0"/>'
                )
    if not geoms:
        return None
    return "\n    ".join(geoms), "\n    ".join(assets), (lo, hi)


def finish(rgba, depth, size):
    """Neutral light-grey gradient where the ray hit nothing (the renderer's sky is dark), then 2x2 box-filter down."""
    h = rgba.shape[0]
    t = np.linspace(0, 1, h)[:, None, None]
    bg = np.concatenate(
        [np.broadcast_to(236 - 26 * t, (h, h, 1))] * 3 + [np.full((h, h, 1), 255.0)],
        axis=2,
    )
    hit = np.isfinite(depth)
    obj = rgba.astype(np.float32)
    if hit.any():  # dark parts (carbon, IMU / rangefinder PCBs) get a gamma lift toward a mean brightness of ~0.42; scene lighting is untouched
        mean = float(np.clip((obj[..., :3][hit] / 255.0).mean(), 1e-3, 0.99))
        gamma = float(np.clip(np.log(TARGET_MEAN) / np.log(mean), MIN_GAMMA, 1.0))
        obj[..., :3] = 255.0 * (obj[..., :3] / 255.0) ** gamma
    img = np.where(hit[..., None], obj, bg)
    img = img.reshape(size, 2, size, 2, 4).mean((1, 3))
    return img.round().astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*")
    ap.add_argument("--out", default="build/thumbs")
    ap.add_argument("--size", type=int, default=256)
    ap.add_argument("--az", type=float, default=35)
    ap.add_argument("--el", type=float, default=25)
    ap.add_argument("--parts", default=str(REPO / "parts"))
    ap.add_argument(
        "--sheet", help="also write a contact sheet PNG of the rendered tiles"
    )
    ap.add_argument("--sheet-cols", type=int, default=4)
    ap.add_argument(
        "--sheet-dir",
        help="also write per-category contact sheets <dir>/<category>-<n>.png (--sheet-cols x --sheet-rows tiles each) and print "
        "their id order (row-major)",
    )
    ap.add_argument("--sheet-rows", type=int, default=5)
    ap.add_argument(
        "--missing",
        action="store_true",
        help="keep the cache in --out, render only the parts it lacks (the viewer's on-demand job)",
    )
    ap.add_argument(
        "--priority", default="", help="comma-separated part ids to render first"
    )
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    lib = Library(a.parts)
    cache = out.resolve() / "visuals"
    index, skipped, tiles, by_cat = {}, [], [], {}
    ids = a.ids or sorted(lib.parts)
    if a.priority:  # these first (the viewer asks for the parts of the open design)
        first = [p for p in a.priority.split(",") if p in lib.parts]
        ids = first + [p for p in ids if p not in first]
    if a.missing:  # keep the cache: render only what it lacks
        try:
            old = json.loads((out / "index.json").read_text())
        except (OSError, ValueError):
            old = {}
        if old.get("_v") == VERSION:
            index = {k: v for k, v in old.items() if k != "_v"}
        ids = [p for p in ids if p not in index and not (out / f"{p}.png").is_file()]
    done = 0

    def save_index():
        tmp = out / "index.json.tmp"
        tmp.write_text(json.dumps({**index, "_v": VERSION}, indent=1, sort_keys=True))
        tmp.replace(out / "index.json")

    for pid in ids:
        done += 1
        if a.missing and done % 20 == 0:
            save_index()
        part = lib.get(pid)
        try:
            sc = part_scene(lib, part, cache)
            if sc is not None:
                diag = float(np.linalg.norm(sc[2][1] - sc[2][0]))
                if (
                    diag < 0.12
                ):  # near plane: enlarge so the camera stays >= ~0.3 m away
                    sc = part_scene(lib, part, cache, 0.12 / max(diag, 1e-4))
        except (
            Exception
        ) as e:  # a generator bug must not stop the sweep; it is reported at the end
            print(f"{pid}: visual failed: {type(e).__name__}: {e}", file=sys.stderr)
            skipped.append(pid)
            if a.missing:  # do not retry on every launch
                index[pid] = {
                    "name": pid,
                    "category": part.category,
                    "vendor": "",
                    "thumb": None,
                    "failed": True,
                }
            continue
        meta = {
            "name": part.raw.get("name", pid),
            "category": part.category,
            "vendor": part.raw.get("vendor", ""),
            "thumb": None,
        }
        index[pid] = meta
        if sc is None:
            continue
        geoms, assets, (lo, hi) = sc
        xml = f"<mujoco><asset>\n    {assets}\n  </asset><worldbody>\n    {geoms}\n  </worldbody></mujoco>"
        with tempfile.NamedTemporaryFile(
            "w", suffix=".xml", dir=out, delete=False
        ) as fh:
            fh.write(xml)
        try:
            sim = Sim.load(fh.name)
            ctr = (lo + hi) / 2
            rad = float(np.linalg.norm(hi - lo)) / 2
            az, el = math.radians(a.az), math.radians(a.el)
            fovy = 30.0
            dist = rad / math.sin(math.radians(fovy / 2)) * 1.05
            pos = ctr + dist * np.array(
                [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
            )
            rgba = finish(
                *sim.render(
                    pos=pos, lookat=ctr, fovy=fovy, width=2 * a.size, height=2 * a.size
                ),
                a.size,
            )
            tmp_png = (
                out / f".{pid}.png.tmp"
            )  # atomic: the viewer may be reading the folder
            tmp_png.write_bytes(_napl.encode_png(rgba))
            tmp_png.replace(out / f"{pid}.png")
            tiles.append(rgba)
            by_cat.setdefault(".".join(part.category.split(".")[:2]), []).append(
                (pid, rgba)
            )
            meta["thumb"] = f"{pid}.png"
        finally:
            Path(fh.name).unlink(missing_ok=True)
    save_index()
    if (
        a.sheet and tiles
    ):  # contact sheet of the rendered tiles in order (for eyeballing a batch)
        cols = min(len(tiles), a.sheet_cols)
        rows = -(-len(tiles) // cols)
        sheet = np.full((rows * a.size, cols * a.size, 4), 255, np.uint8)
        for i, t in enumerate(tiles):
            r, c = divmod(i, cols)
            sheet[r * a.size : (r + 1) * a.size, c * a.size : (c + 1) * a.size] = t
        Path(a.sheet).write_bytes(_napl.encode_png(sheet))
    if a.sheet_dir:
        sd = Path(a.sheet_dir)
        sd.mkdir(parents=True, exist_ok=True)
        per = a.sheet_cols * a.sheet_rows
        for cat, items in sorted(by_cat.items()):
            for k in range(0, len(items), per):
                chunk = items[k : k + per]
                rows = -(-len(chunk) // a.sheet_cols)
                sheet = np.full(
                    (rows * a.size, a.sheet_cols * a.size, 4), 255, np.uint8
                )
                for i, (_, t) in enumerate(chunk):
                    r, c = divmod(i, a.sheet_cols)
                    sheet[
                        r * a.size : (r + 1) * a.size, c * a.size : (c + 1) * a.size
                    ] = t
                name = f"{cat}-{k // per}.png"
                (sd / name).write_bytes(_napl.encode_png(sheet))
                print(f"sheet {name}: " + " ".join(p for p, _ in chunk))
    n = sum(1 for m in index.values() if m["thumb"])
    if not a.quiet:
        print(f"{n}/{len(ids)} thumbnails in {out}; {len(skipped)} failed {skipped}")
    return 1 if skipped else 0


if __name__ == "__main__":
    sys.exit(main())
