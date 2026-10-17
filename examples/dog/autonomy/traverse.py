"""Traversability of 0.5 m blocks from a 5 cm height grid: plane fit (slope, roughness), largest neighbour jump (step)."""

from collections import deque

import numpy as np

MOVES = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def blocks(a, k):
    nb = a.shape[0] // k
    return (
        a[: nb * k, : nb * k]
        .reshape(nb, k, nb, k)
        .transpose(0, 2, 1, 3)
        .reshape(nb, nb, k * k)
    )


def layers(h, res, k):
    h = np.asarray(h, dtype=np.float64)
    known = np.isfinite(h)
    hb, wb = blocks(np.where(known, h, 0.0), k), blocks(known.astype(np.float64), k)
    v, u = np.divmod(np.arange(k * k), k)
    U = np.stack(
        [u * res, v * res, np.ones(k * k)], axis=1
    )  # (k*k, 3), the same in every block
    A = np.einsum("...c,ci,cj->...ij", wb, U, U)
    b = np.einsum("...c,ci->...i", wb * hb, U)
    coef = np.einsum(
        "...ij,...j->...i", np.linalg.pinv(A), b
    )  # weighted LSQ plane per block (pinv: < 3 points is fine)
    resid = (hb - np.einsum("ci,...i->...c", U, coef)) * wb
    n = wb.sum(-1)
    with np.errstate(invalid="ignore", divide="ignore"):
        rough = np.sqrt((resid**2).sum(-1) / n)
    dx = np.full(h.shape, np.nan)
    dy = np.full(h.shape, np.nan)
    dx[:, :-1] = np.abs(
        h[:, 1:] - h[:, :-1]
    )  # pair (i, i+1) belongs to cell i's block: borders are covered
    dy[:-1, :] = np.abs(h[1:, :] - h[:-1, :])
    st = np.fmax(dx, dy)
    step = blocks(np.where(np.isfinite(st), st, 0.0), k).max(-1)
    return {
        "slope": np.arctan(np.hypot(coef[..., 0], coef[..., 1])),
        "step": step,
        "rough": np.where(n > 0, rough, 0.0),
        "known": n / (k * k),
    }


def lethal(L, cfg):
    T = cfg["trav"]
    judged = L["known"] >= T["min_known"]
    bad = (
        (L["slope"] > T["slope_max"])
        | (L["step"] > T["step_max"])
        | (L["rough"] > T["rough_max"])
    )
    return judged & bad


def dilate(m):
    p = np.pad(m, 1)
    out = m.copy()
    n = m.shape
    for di, dj in MOVES:
        out |= p[1 + di : 1 + di + n[0], 1 + dj : 1 + dj + n[1]]
    return out


def cost(L, cfg):
    T = cfg["trav"]
    leth = lethal(L, cfg)
    c = np.where(L["known"] >= T["min_known"], 1.0, T["unknown_cost"])
    c = c + np.where(dilate(leth) & ~leth, T["near_lethal"], 0.0)
    return np.where(leth, np.inf, c)


def reachable(passable, start):
    nb0, nb1 = passable.shape
    seen = np.zeros_like(passable, dtype=bool)
    if not passable[start]:
        return seen
    seen[start] = True
    q = deque([start])
    while q:
        i, j = q.popleft()
        for di, dj in MOVES:
            a, b = i + di, j + dj
            if not (0 <= a < nb0 and 0 <= b < nb1) or seen[a, b] or not passable[a, b]:
                continue
            if di and dj and not (passable[i + di, j] and passable[i, j + dj]):
                continue  # no corner cutting
            seen[a, b] = True
            q.append((a, b))
    return seen


class Traversability:
    def __init__(self, cfg, n):
        self.cfg, self.res = cfg, cfg["map"]["res"]
        self.k = int(round(cfg["trav"]["block"] / self.res))
        nb = n // self.k
        z = np.zeros((nb, nb))
        self.L = {
            "slope": z.copy(),
            "step": z.copy(),
            "rough": z.copy(),
            "known": z.copy(),
        }
        self.cost = np.full((nb, nb), cfg["trav"]["unknown_cost"])

    def update(self, h, window=None):
        k, nb = self.k, self.cost.shape[0]
        if window is None:
            b0, b1, c0, c1 = 0, nb, 0, nb
        else:  # blocks touched by the changed cells, +1 (step pairs at the border, near-lethal ring)
            i0, i1, j0, j1 = window
            b0, b1 = max(i0 // k - 1, 0), min((i1 - 1) // k + 2, nb)
            c0, c1 = max(j0 // k - 1, 0), min((j1 - 1) // k + 2, nb)
        sub = h[
            b0 * k : min(b1 * k + 1, h.shape[0]), c0 * k : min(c1 * k + 1, h.shape[1])
        ]
        Ls = layers(
            sub, self.res, k
        )  # the extra row / column gives the border step pairs, blocks() drops the partial block
        for key in self.L:
            self.L[key][b0:b1, c0:c1] = Ls[key][: b1 - b0, : c1 - c0]
        r0, r1, s0, s1 = (
            max(b0 - 1, 0),
            min(b1 + 1, nb),
            max(c0 - 1, 0),
            min(c1 + 1, nb),
        )
        e0, e1, f0, f1 = (
            max(r0 - 1, 0),
            min(r1 + 1, nb),
            max(s0 - 1, 0),
            min(s1 + 1, nb),
        )
        Lw = {key: v[e0:e1, f0:f1] for key, v in self.L.items()}
        self.cost[r0:r1, s0:s1] = cost(Lw, self.cfg)[
            r0 - e0 : r1 - e0, s0 - f0 : s1 - f0
        ]
        return self.cost
