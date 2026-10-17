"""Ground-truth scoring of a map. Harness only: the robot modules never import this."""

import math

import numpy as np

from . import traverse

ROWS = 200  # truth_grid works in row bands of 200 to bound memory


def truth_grid(f, res):
    n = int(round(f.size / res))
    xc = -f.size / 2 + (np.arange(n) + 0.5) * res
    g = np.empty((n, n))
    for r0 in range(0, n, ROWS):
        X, Y = np.meshgrid(xc, xc[r0 : r0 + ROWS])
        g[r0 : r0 + ROWS] = f.hfield_height(X, Y)
    for b in f.boxes:  # rasterise each box inside its own bounding box only
        ext = (
            abs(b.size[0] * math.cos(b.rpy[2])) + abs(b.size[1] * math.sin(b.rpy[2])),
            abs(b.size[0] * math.sin(b.rpy[2])) + abs(b.size[1] * math.cos(b.rpy[2])),
        )
        c0, c1 = (
            max(int((b.pos[0] - ext[0] + f.size / 2) / res) - 1, 0),
            min(int((b.pos[0] + ext[0] + f.size / 2) / res) + 2, n),
        )
        r0, r1 = (
            max(int((b.pos[1] - ext[1] + f.size / 2) / res) - 1, 0),
            min(int((b.pos[1] + ext[1] + f.size / 2) / res) + 2, n),
        )
        X, Y = np.meshgrid(xc[c0:c1], xc[r0:r1])
        c, s = math.cos(b.rpy[2]), math.sin(b.rpy[2])
        dx, dy = X - b.pos[0], Y - b.pos[1]
        inside = (np.abs(c * dx + s * dy) <= b.size[0]) & (
            np.abs(-s * dx + c * dy) <= b.size[1]
        )
        g[r0:r1, c0:c1] = np.where(
            inside, np.maximum(g[r0:r1, c0:c1], b.pos[2] + b.size[2]), g[r0:r1, c0:c1]
        )
    return g


def edge_mask(gt, cfg):
    p = np.pad(gt, 1, mode="edge")
    n = gt.shape
    sh = [
        p[1 + a : 1 + a + n[0], 1 + b : 1 + b + n[1]]
        for a in (-1, 0, 1)
        for b in (-1, 0, 1)
    ]
    return np.max(sh, axis=0) - np.min(sh, axis=0) > cfg["map"]["edge_range"]


def targets(gt, cfg, start_xy):
    res, size = cfg["map"]["res"], cfg["map"]["size"]
    k = int(round(cfg["trav"]["block"] / res))
    L = traverse.layers(gt, res, k)
    passable = ~traverse.lethal(L, cfg)
    s = (
        int((start_xy[1] + size / 2) / (k * res)),
        int((start_xy[0] + size / 2) / (k * res)),
    )
    blk = traverse.reachable(passable, s)
    out = np.zeros(gt.shape, bool)
    nb = blk.shape[0]
    out[: nb * k, : nb * k] = np.kron(blk, np.ones((k, k), bool))
    return out


def coverage(m, tgt, pool):
    q = (m.n // pool) * pool
    t = tgt[:q, :q].reshape(q // pool, pool, q // pool, pool).any(axis=(1, 3))
    return float((m.mapped(pool) & t).sum() / t.sum())


def height_rms(m, gt, edges):
    seen = m.v < m.vmap
    e = m.h.astype(np.float64) - gt
    a, b = seen & ~edges, seen & edges
    return {
        "rms": float(np.sqrt((e[a] ** 2).mean())) if a.any() else math.nan,
        "rms_edges": float(np.sqrt((e[b] ** 2).mean())) if b.any() else math.nan,
        "n": int(a.sum()),
        "n_edges": int(b.sum()),
    }
