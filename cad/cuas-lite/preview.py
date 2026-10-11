"""Quick orthographic preview of the assembled turret (painter's algorithm + flat shading, numpy + Pillow only).

Writes <out>/preview/*.png: iso at tilt 0 / +70 / -15, side (from +y), front, top. Images are never committed (build/ is ignored).
"""

import bought as BT
import numpy as np
import params as P
from PIL import Image, ImageDraw

import parts as K

COL = {
    "static": (150, 170, 200),
    "pan": (200, 150, 110),
    "tilt": (120, 190, 130),
    "bought": (170, 170, 170),
}


def _tess(shape, tol=0.2):
    v, t = shape.tessellate(tol)
    v = np.array([[p.X, p.Y, p.Z] for p in v])
    return v[np.array(t)] if len(t) else np.zeros((0, 3, 3))


def scene(meshes, phi=0.0):
    tris, cols = [], []
    rot = np.radians(-phi)
    Ry = np.array(
        [[np.cos(rot), 0, np.sin(rot)], [0, 1, 0], [-np.sin(rot), 0, np.cos(rot)]]
    )
    for n, m in meshes.items():
        g = K.REGISTRY[n][1]
        if n in ("cable_clip", "camera_shim"):
            continue
        t = np.asarray(m.triangles)
        if g == "tilt":
            t = t @ Ry.T + np.array([0, 0, P.H_T])
        tris.append(t)
        cols.append(np.tile(COL[g], (len(t), 1)))
    for it in BT.head_items():
        if it["shape"] is None:
            continue
        t = _tess(it["shape"]) @ Ry.T + np.array([0, 0, P.H_T])
        tris.append(t)
        cols.append(np.tile(COL["bought"], (len(t), 1)))
    for it in BT.pan_items() + BT.static_items():
        t = _tess(it["shape"])
        tris.append(t)
        cols.append(np.tile(COL["bought"], (len(t), 1)))
    return np.concatenate(tris), np.concatenate(cols)


def subdivide(tris, cols, max_edge=12.0, rounds=3):
    """4-way split of triangles with long edges so the painter's sort has fewer artefacts."""
    for _ in range(rounds):
        e = np.stack(
            [
                np.linalg.norm(tris[:, 1] - tris[:, 0], axis=1),
                np.linalg.norm(tris[:, 2] - tris[:, 1], axis=1),
                np.linalg.norm(tris[:, 0] - tris[:, 2], axis=1),
            ]
        ).max(0)
        big = e > max_edge
        if not big.any():
            break
        t, c = tris[big], cols[big]
        a, b, d = t[:, 0], t[:, 1], t[:, 2]
        ab, bc, ca = (a + b) / 2, (b + d) / 2, (d + a) / 2
        new = np.concatenate(
            [
                np.stack([a, ab, ca], 1),
                np.stack([ab, b, bc], 1),
                np.stack([ca, bc, d], 1),
                np.stack([ab, bc, ca], 1),
            ]
        )
        tris = np.concatenate([tris[~big], new])
        cols = np.concatenate([cols[~big], np.tile(c, (4, 1))])
    return tris, cols


def render(tris, cols, az, el, path, size=1100):
    tris, cols = subdivide(tris, cols)
    a, e = np.radians(az), np.radians(el)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    r = np.array([-np.sin(a), np.cos(a), 0.0])
    u = np.array([-np.sin(e) * np.cos(a), -np.sin(e) * np.sin(a), np.cos(e)])
    pts = tris.reshape(-1, 3)
    uu, vv, dd = pts @ r, pts @ u, pts @ d
    lo = np.array([uu.min(), vv.min()])
    sc = (size - 40) / max(uu.max() - lo[0], vv.max() - lo[1])
    X = ((uu - lo[0]) * sc + 20).reshape(-1, 3)
    Y = (size - 20 - (vv - lo[1]) * sc).reshape(-1, 3)
    D = dd.reshape(-1, 3).mean(1)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = d * 0.7 + np.array([0.3, -0.3, 0.8])
    light /= np.linalg.norm(light)
    sh = 0.35 + 0.65 * np.abs(n @ light)
    img = Image.new("RGB", (size, size), (245, 245, 245))
    dr = ImageDraw.Draw(img)
    for i in np.argsort(D):
        c = tuple((cols[i] * sh[i]).astype(int))
        dr.polygon([(X[i, 0], Y[i, 0]), (X[i, 1], Y[i, 1]), (X[i, 2], Y[i, 2])], fill=c)
    img.save(path)


def run(out, recs, meshes):
    d = out / "preview"
    d.mkdir(exist_ok=True)
    for name, phi, az, el in (
        ("iso_tilt0", 0, 35, 25),
        ("iso_tilt70", 70, 35, 25),
        ("iso_tiltm15", -15, 35, 25),
        ("side_tilt0", 0, 90, 0),
        ("front_tilt0", 0, 0, 0),
        ("top_tilt0", 0, 0, 89.9),
        ("rear_iso", 0, 215, 25),
        ("side_tilt70", 70, 90, 0),
    ):
        tris, cols = scene(meshes, phi)
        render(tris, cols, az, el, d / f"{name}.png")
    print(f"preview written to {d}")
