"""Footprint-aware planning from the geometry itself (ruins): obstacle cells of the 5 cm elevation map -> distance transform ->
block cost from the clearance at the block centre -> waypoints pulled onto the medial axis, with a straight approach to every
doorway. Replaces block lethality (which was one-sided: a wall's lethal band grew on one side only, the free gap of a doorway was
narrower than the real one and off-centre, and the robot grazed a jamb or turned on the spot inside the door)."""

import math

import numpy as np

GROUND_BLOCK = 2.0  # m: the ground level of a cell is the lowest known cell of its 2 m block (flat ground; ruins)


def obstacles(hn, res, step_max):
    """Known cells higher than the ground of their 2 m block by more than the step the body climbs."""
    n = hn.shape[0]
    k = max(int(round(GROUND_BLOCK / res)), 1)
    nb = n // k
    core = hn[: nb * k, : nb * k].reshape(nb, k, nb, k)
    with np.errstate(all="ignore"):
        low = np.nanmin(np.where(np.isfinite(core), core, np.inf), axis=(1, 3))
    ground = np.repeat(np.repeat(low, k, 0), k, 1)
    out = np.zeros(hn.shape, bool)
    out[: nb * k, : nb * k] = np.isfinite(ground) & (
        hn[: nb * k, : nb * k] - ground > step_max
    )
    return out


def edt(mask, res):
    """Distance in m to the nearest True cell: 3-4 chamfer, vectorised per row (error < 9 %, over-estimates). Without obstacles: 1e3 m."""
    a, b = 1.0, math.sqrt(2.0)
    d = np.where(mask, 0.0, 1e6)
    n, m = d.shape
    ja = a * np.arange(m)

    def sweep(row):  # propagate along the row, both ways
        row = np.minimum.accumulate(row - ja) + ja
        return np.minimum.accumulate((row + ja)[::-1])[::-1] - ja

    for rng in (range(n), range(n - 1, -1, -1)):
        prev = None
        for i in rng:
            row = d[i]
            if prev is not None:
                row = np.minimum(row, prev + a)
                row[1:] = np.minimum(row[1:], prev[:-1] + b)
                row[:-1] = np.minimum(row[:-1], prev[1:] + b)
            d[i] = prev = sweep(row)
    return np.minimum(d * res, 1e3)


def block_clearance(D, k):
    """Clearance of each block: the smallest distance among the central 2 x 2 cells (k cells per block side)."""
    nb = D.shape[0] // k
    c = k // 2
    sub = D[: nb * k, : nb * k].reshape(nb, k, nb, k)[
        :, c - 1 : c + 1, :, c - 1 : c + 1
    ]
    return sub.min(axis=(1, 3))


def block_cost(known, D, cfg):
    """Cost grid: 1 known / unknown_cost unknown, plus a penalty rising to near_lethal as the clearance falls from
    clear_margin to clear_lethal; infinite below clear_lethal (the feet reach 0.29 m from the body centre)."""
    T = cfg["trav"]
    k = int(round(T["block"] / cfg["map"]["res"]))
    c = block_clearance(D, k)[: known.shape[0], : known.shape[1]]
    lo, hi = T["clear_lethal"], T["clear_margin"]
    base = np.where(known >= T["min_known"], 1.0, T["unknown_cost"])
    cost = base + T["near_lethal"] * np.clip((hi - c) / (hi - lo), 0.0, 1.0)
    return np.where(c < lo, np.inf, cost)


def sample(D, x, y, x0, res):
    n = D.shape[0]
    j = np.clip(np.floor((np.asarray(x, float) - x0) / res).astype(int), 0, n - 1)
    i = np.clip(np.floor((np.asarray(y, float) - x0) / res).astype(int), 0, n - 1)
    return D[i, j]


def refine(path, D, cfg):
    """Pull every interior waypoint sideways (across the path) to the best clearance within +-clear_shift, capped at clear_margin
    (open ground does not pull, a doorway centres); then put a straight approach waypoint before every narrow passage."""
    T = cfg["trav"]
    res, x0 = cfg["map"]["res"], -cfg["map"]["size"] / 2
    if len(path) < 3:
        return list(path)
    cap, shift = T["clear_margin"], T["clear_shift"]
    offs = np.arange(-shift, shift + res / 2, res)
    pts = [tuple(p) for p in path]
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        dx, dy = pts[i + 1][0] - pts[i - 1][0], pts[i + 1][1] - pts[i - 1][1]
        n = math.hypot(dx, dy)
        if n < 1e-9:
            out.append(pts[i])
            continue
        nx, ny = -dy / n, dx / n
        cx, cy = pts[i][0] + offs * nx, pts[i][1] + offs * ny
        score = np.minimum(sample(D, cx, cy, x0, res), cap) - 1e-3 * np.abs(offs)
        # a candidate must not be inside an obstacle's reach
        j = int(np.argmax(score))
        out.append((float(cx[j]), float(cy[j])))
    out.append(pts[-1])
    return approach(out, D, cfg)


def approach(pts, D, cfg):
    """A waypoint `clear_approach` m before every run of narrow waypoints (clearance < clear_margin), on the line of the run: the robot
    enters a doorway straight instead of turning inside it."""
    T = cfg["trav"]
    res, x0 = cfg["map"]["res"], -cfg["map"]["size"] / 2
    cl = [float(sample(D, p[0], p[1], x0, res)) for p in pts]
    out, i = [], 0
    while i < len(pts):
        if cl[i] < T["clear_margin"] and i > 0 and cl[i - 1] >= T["clear_margin"]:
            j = i
            while j + 1 < len(pts) and cl[j + 1] < T["clear_margin"]:
                j += 1
            dx, dy = pts[j][0] - pts[i][0], pts[j][1] - pts[i][1]
            n = math.hypot(dx, dy)
            if n > 1e-9:
                p = (
                    pts[i][0] - T["clear_approach"] * dx / n,
                    pts[i][1] - T["clear_approach"] * dy / n,
                )
                if float(sample(D, p[0], p[1], x0, res)) >= T["clear_lethal"]:
                    out.append(p)
        out.append(pts[i])
        i += 1
    return out
