"""Orthographic preview stills (painter's algorithm + flat shading, numpy + Pillow only): the assembled turret and the launcher
cartridge + feeder cut-away (half-space y <= 0 removed). Written to <out>/preview/*.png (build/ is gitignored; never commit images).
"""

import numpy as np
import params as P
from build123d import Box, Pos, Rot
from PIL import Image, ImageDraw

COL = {
    "M1": (150, 170, 200),
    "M2": (200, 150, 110),
    "M3": (110, 180, 140),
    "M4": (225, 190, 90),
    "M5": (170, 130, 200),
    "M6": (120, 120, 150),
    "bought": (175, 175, 175),
    "ball": (240, 120, 60),
}


def tess(shape, tol=0.25):
    v, t = shape.tessellate(tol)
    v = np.array([[p.X, p.Y, p.Z] for p in v])
    return v[np.array(t)] if len(t) else np.zeros((0, 3, 3))


def subdivide(tris, cols, max_edge=14.0, rounds=3):
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


def render(tris, cols, az, el, path, size=1200, crop=None):
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
        dr.polygon(
            [(X[i, 0], Y[i, 0]), (X[i, 1], Y[i, 1]), (X[i, 2], Y[i, 2])],
            fill=tuple((cols[i] * sh[i]).astype(int)),
        )
    img.save(path)


def _add(tris, cols, shape, color, cut=None):
    if cut is not None:
        shape = shape & cut
        if shape is None:
            return
    try:
        t = tess(shape)
    except ValueError:  # the cut removed the whole shape
        return
    if len(t):
        tris.append(t)
        cols.append(np.tile(color, (len(t), 1)))


def assembled(S, recs, bought, tilt):
    tris, cols = [], []
    head = Pos(0, 0, P.H_T) * Rot(0, -tilt, 0) * Pos(-P.X_A, 0, -P.Z_A)  # L -> B
    for n, r in recs.items():
        s = S[n]
        if r["group"] == "tilt":
            places = [Pos(*p) for p in r["at"]] if r.get("at") else [Pos(0, 0, 0)]
            for pl in places:
                _add(tris, cols, head * pl * s, COL[r["module"]])
        else:
            _add(tris, cols, s, COL[r["module"]])
    for b in bought:
        if b["group"] == "tilt":
            _add(
                tris,
                cols,
                head * b["shape"],
                COL["ball"] if b["name"] == "balls" else COL["bought"],
            )
        else:
            _add(tris, cols, b["shape"], COL["bought"])
    # wheels (tyres/hubs) are placed twice
    return np.concatenate(tris), np.concatenate(cols)


def cutaway(S, recs, bought):
    """Launcher cartridge + feeder in L, the y > 0 half removed so the nip, the pocket and the rod are visible."""
    cut = Pos(0, -150, 0) * Box(600, 300, 600)  # keeps y <= 0
    tris, cols = [], []
    for n, r in recs.items():
        if r["module"] not in ("M3", "M4"):
            continue
        places = [Pos(*p) for p in r["at"]] if r.get("at") else [Pos(0, 0, 0)]
        for pl in places:
            _add(tris, cols, pl * S[n], COL[r["module"]], cut)
    for b in bought:
        if b["module"] in ("M3", "M4"):
            _add(
                tris,
                cols,
                b["shape"],
                COL["ball"] if b["name"] == "balls" else COL["bought"],
                cut,
            )
    return np.concatenate(tris), np.concatenate(cols)


def run(out, S, recs, bought):
    d = out / "preview"
    d.mkdir(exist_ok=True)
    for name, tilt, az, el in (
        ("iso_tilt20", 20, 40, 22),
        ("iso_tilt0", 0, 130, 22),
        ("side_tilt35", 35, 90, 0),
        ("front_tilt0", 0, 0, 0),
    ):
        t, c = assembled(S, recs, bought, tilt)
        render(t, c, az, el, d / f"cuas2-assembled-{name}.png")
    t, c = cutaway(S, recs, bought)
    render(t, c, 90, 0, d / "cuas2-cartridge-cutaway-side.png")
    render(t, c, 60, 25, d / "cuas2-cartridge-cutaway-iso.png")
    print(f"preview written to {d}")
