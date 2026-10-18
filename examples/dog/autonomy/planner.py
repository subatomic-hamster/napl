"""Coverage planning on 0.5 m blocks: boustrophedon sweep lanes, then the nearest unmapped gap / frontier (Dijkstra once per
new goal, A* at 1-2 Hz to the current goal), pure pursuit to follow the path. Python heapq: ~40 000 blocks."""

import heapq
import math

import numpy as np

from .localize import wrap
from .traverse import MOVES

SQ2 = math.sqrt(2.0)


def _legal(cost, i, j, di, dj):
    a, b = i + di, j + dj
    if not (0 <= a < cost.shape[0] and 0 <= b < cost.shape[1]) or not math.isfinite(
        cost[a, b]
    ):
        return False
    return not (di and dj) or (
        math.isfinite(cost[i + di, j]) and math.isfinite(cost[i, j + dj])
    )


def _search(cost, start, goal):
    n1 = cost.shape[1]
    dist = np.full(cost.shape, np.inf)
    parent = np.full(cost.size, -1, dtype=np.int64)
    if not math.isfinite(cost[start]):
        return dist, parent
    dist[start] = 0.0
    h = (
        (
            lambda i, j: (
                max(abs(i - goal[0]), abs(j - goal[1]))
                + (SQ2 - 1) * min(abs(i - goal[0]), abs(j - goal[1]))
            )
        )
        if goal
        else (lambda i, j: 0.0)
    )
    pq = [(h(*start), 0.0, start)]
    done = np.zeros(cost.shape, bool)
    c = cost.tolist()
    while pq:
        _, d, (i, j) = heapq.heappop(pq)
        if done[i, j]:
            continue
        done[i, j] = True
        if goal is not None and (i, j) == goal:
            break
        for di, dj in MOVES:
            if not _legal(cost, i, j, di, dj):
                continue
            a, b = i + di, j + dj
            nd = d + c[a][b] * (SQ2 if di and dj else 1.0)
            if nd < dist[a, b]:
                dist[a, b] = nd
                parent[a * n1 + b] = i * n1 + j
                heapq.heappush(pq, (nd + h(a, b), nd, (a, b)))
    return dist, parent


def dijkstra(cost, start):
    return _search(cost, start, None)


def path_to(parent, start, goal, n1=None):
    n1 = n1 or math.isqrt(parent.size)
    out, k, s = [], goal[0] * n1 + goal[1], start[0] * n1 + start[1]
    while k != -1:
        out.append(divmod(int(k), n1))
        if k == s:
            return out[::-1]
        k = parent[k]
    return None


def astar(cost, start, goal):
    dist, parent = _search(cost, start, goal)
    if not math.isfinite(dist[goal]):
        return None
    return path_to(parent, start, goal, cost.shape[1])


def block_of(x, y, cfg):
    b, h = cfg["trav"]["block"], cfg["map"]["size"] / 2
    nb = int(round(cfg["map"]["size"] / b))
    return min(max(int((y + h) // b), 0), nb - 1), min(
        max(int((x + h) // b), 0), nb - 1
    )


def centre(ij, cfg):
    b, h = cfg["trav"]["block"], cfg["map"]["size"] / 2
    return -h + (ij[1] + 0.5) * b, -h + (ij[0] + 0.5) * b


def lanes(cfg, lane_m, start_xy):
    h = cfg["map"]["size"] / 2
    inset = (
        cfg["trav"]["block"] * 8
    )  # 4 m from the field edge: the lidar maps the border strip from inside
    n = math.ceil((2 * h) / lane_m)
    ys = [-h + lane_m / 2 + k * (2 * h - lane_m) / max(n - 1, 1) for k in range(n)]
    if start_xy[1] > 0:
        ys = ys[::-1]
    xs = (h - inset, -h + inset) if start_xy[0] > 0 else (-h + inset, h - inset)
    pts = []
    for k, y in enumerate(ys):
        a, b = xs if k % 2 == 0 else xs[::-1]
        pts += [(a, y), (b, y)]
    return pts


def gain(mapped, cfg, pool):
    k = int(
        round(cfg["trav"]["block"] / (cfg["map"]["res"] * pool))
    )  # pooled cells per block side
    nb = mapped.shape[0] // k
    un = ~mapped[: nb * k, : nb * k]
    return un.reshape(nb, k, nb, k).sum(axis=(1, 3)) / (k * k)


def box_sum(a, r):
    """Sum of `a` over the (2r+1)^2 window around every cell (integral image)."""
    n, m = a.shape
    s = np.zeros((n + 1, m + 1))
    s[1:, 1:] = a.cumsum(0).cumsum(1)
    i0, i1 = np.clip(np.arange(n) - r, 0, n), np.clip(np.arange(n) + r + 1, 0, n)
    j0, j1 = np.clip(np.arange(m) - r, 0, m), np.clip(np.arange(m) + r + 1, 0, m)
    return s[i1][:, j1] - s[i0][:, j1] - s[i1][:, j0] + s[i0][:, j0]


def choose_view(dist, g, clear, tried, cfg):
    """Room-tour step (clearance mode): the viewpoint with the most unmapped floor within the lidar reach `fill_radius`, per unit of
    path cost (greedy nearest-neighbour on value density). Floor = blocks with a free clearance (wall interiors never get mapped and
    would keep the value up for ever). tried: bool grid of viewpoints already visited."""
    P, T = cfg["plan"], cfg["trav"]
    r = int(round(P["fill_radius"] / T["block"]))
    v = box_sum(g * (clear >= T["clear_lethal"]) * T["block"] ** 2, r)
    cand = np.isfinite(dist) & (v >= P["fill_min"]) & ~tried
    if not cand.any():
        return None
    score = np.where(cand, v / (dist + P["fill_d0"]), -np.inf)
    return tuple(int(x) for x in np.unravel_index(int(np.argmax(score)), score.shape))


def choose(dist, g, tried, cfg):
    cand = np.isfinite(dist) & (g >= cfg["plan"]["min_gain"])
    for ij in tried:
        cand[ij] = False
    if not cand.any():
        return None
    key = np.where(cand, dist - 1e-6 * g, np.inf)
    return tuple(int(v) for v in np.unravel_index(int(np.argmin(key)), key.shape))


def view_goal(path, cfg):
    back = cfg["plan"]["view_back"] / cfg["trav"]["block"]
    acc = 0.0
    for k in range(len(path) - 1, 0, -1):
        a, b = path[k], path[k - 1]
        acc += math.hypot(a[0] - b[0], a[1] - b[1])
        if acc >= back - 1e-9:
            return b
    return path[0]


def pursue(pose, path_xy, cfg, lim, lookahead=None, back=0.0):
    """Pure pursuit. back > 0 (m/s): a target behind the robot is reached by reversing, never by turning on the spot (next to a wall
    the feet, 0.29 m from the centre, wedge against it: ruins training seed 2, 100-800 bumps in one place)."""
    P = cfg["plan"]
    L = P["lookahead"] if lookahead is None else lookahead
    x, y, yaw = pose
    pts = np.asarray(path_xy, float)
    d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
    k0 = int(np.argmin(d))
    far = np.flatnonzero(d[k0:] >= L)
    tx, ty = pts[k0 + far[0]] if len(far) else pts[-1]
    a = wrap(math.atan2(ty - y, tx - x) - yaw)
    if abs(a) > P["turn_in_place"]:
        if back > 0.0 and abs(a) > math.pi / 2:
            b = wrap(a - math.pi)  # heading error of the rear
            return -back * math.cos(b), max(-0.5, min(0.5, 1.5 * b))
        return 0.0, math.copysign(lim["wz_max"], a)
    v = lim["v_max"] * math.cos(a)
    wz = 2.0 * v * math.sin(a) / L
    return v, max(-lim["wz_max"], min(lim["wz_max"], wz))
