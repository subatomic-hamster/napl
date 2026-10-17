import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import numpy as np
import pytest
from autonomy import config, planner

CFG = config.load()


def bellman_ford(cost, start):
    n0, n1 = cost.shape
    d = np.full(cost.shape, np.inf)
    d[start] = 0.0
    for _ in range(n0 * n1):
        changed = False
        for i in range(n0):
            for j in range(n1):
                if not np.isfinite(cost[i, j]):
                    continue
                for di, dj in planner.MOVES:
                    a, b = i - di, j - dj  # predecessor
                    if not (0 <= a < n0 and 0 <= b < n1) or not np.isfinite(d[a, b]):
                        continue
                    if (
                        di
                        and dj
                        and not (
                            np.isfinite(cost[a + di, b])
                            and np.isfinite(cost[a, b + dj])
                        )
                    ):
                        continue
                    c = d[a, b] + cost[i, j] * math.hypot(di, dj)
                    if c < d[i, j] - 1e-12:
                        d[i, j], changed = c, True
        if not changed:
            return d
    return d


def test_astar_and_dijkstra_match_a_reference():
    rng = np.random.default_rng(0)
    cost = rng.choice([1.0, 2.0, 3.0, np.inf], size=(14, 14), p=[0.55, 0.2, 0.1, 0.15])
    cost[0, 0] = cost[13, 13] = 1.0
    ref = bellman_ford(cost, (0, 0))
    dist, parent = planner.dijkstra(cost, (0, 0))
    np.testing.assert_allclose(dist, ref)
    for goal in [(13, 13), (7, 3), (2, 12)]:
        p = planner.astar(cost, (0, 0), goal)
        if not np.isfinite(ref[goal]):
            assert p is None
            continue
        c = sum(
            cost[b] * math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:])
        )
        assert c == pytest.approx(ref[goal]) and p[0] == (0, 0) and p[-1] == goal
        assert planner.path_to(parent, (0, 0), goal)[-1] == goal


def test_door_and_no_corner_cutting():
    cost = np.ones((9, 9))
    cost[4, :] = np.inf
    cost[4, 6] = 1.0  # a one-block door
    p = planner.astar(cost, (0, 0), (8, 0))
    assert (4, 6) in p
    q = np.ones((3, 3))
    q[0, 1] = q[1, 0] = np.inf  # (0, 0) touches (1, 1) only at a corner
    q[1, 2] = q[2, 1] = np.inf
    assert planner.astar(q, (0, 0), (2, 2)) is None


def test_lanes_cover_the_field_from_the_nearest_corner():
    size, lane = CFG["map"]["size"], 8.0
    w = planner.lanes(CFG, lane, (40.0, -45.0))
    assert (
        math.hypot(w[0][0] - 46.0, w[0][1] + 46.0) < 1.0
    )  # starts at the south-east corner
    ys = sorted({round(p[1], 6) for p in w})
    assert (
        ys[0] - (-size / 2) <= lane / 2 + 1e-9 and size / 2 - ys[-1] <= lane / 2 + 1e-9
    )
    assert all(b - a <= lane + 1e-9 for a, b in zip(ys, ys[1:]))
    for a, b in zip(w, w[1:]):
        assert a[0] == b[0] or a[1] == b[1]  # axis-aligned legs


def test_gain_choose_and_view_goal():
    cfg = config.override(CFG, **{"map.size": 10.0})
    mapped = np.ones((100, 100), bool)  # pool 2 on a 200-cell map
    mapped[60:65, 80:85] = (
        False  # one block (12, 16) fully unmapped: 5 x 5 pooled cells
    )
    mapped[0:5, 0:5] = False  # one block (0, 0) fully unmapped
    mapped[20:22, 20:25] = False  # block (4, 4): 10 of 25 unmapped = 0.4
    mapped[40:41, 40:45] = False  # block (8, 8): 5 of 25 = 0.2, below min_gain
    g = planner.gain(mapped, cfg, 2)
    assert (
        g[12, 16] == 1.0
        and g[0, 0] == 1.0
        and g[4, 4] == pytest.approx(0.4)
        and g.sum() == pytest.approx(2.6)
    )
    cost = np.ones((20, 20))
    dist, _ = planner.dijkstra(cost, (10, 10))
    assert planner.choose(dist, g, set(), cfg) == (12, 16)
    assert planner.choose(dist, g, {(12, 16)}, cfg) == (
        4,
        4,
    )  # (8, 8) is nearer but below min_gain
    assert planner.choose(dist, g, {(12, 16), (4, 4)}, cfg) == (0, 0)
    path = [(10, j) for j in range(20)]
    assert planner.view_goal(path, cfg) == (10, 14)  # 2.5 m = 5 blocks before (10, 19)


def test_pure_pursuit():
    P = CFG["plan"]
    lim = {"v_max": 1.5, "wz_max": 1.0}
    ahead = [(0.5 * k, 0.0) for k in range(10)]
    assert planner.pursue((0.0, 0.0, 0.0), ahead, CFG, lim) == pytest.approx((1.5, 0.0))
    left = [(0.0, 0.5 * k) for k in range(10)]
    assert planner.pursue((0.0, 0.0, 0.0), left, CFG, lim) == pytest.approx((0.0, 1.0))
    a = math.radians(30)
    diag = [(0.25 * k * math.cos(a), 0.25 * k * math.sin(a)) for k in range(20)]
    vx, wz = planner.pursue((0.0, 0.0, 0.0), diag, CFG, lim)
    assert vx == pytest.approx(1.5 * math.cos(a)) and wz == pytest.approx(
        min(2 * vx * math.sin(a) / P["lookahead"], 1.0)
    )
