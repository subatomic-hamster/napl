import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import numpy as np
import pytest
from autonomy import clearance, config, elevmap, planner

CFG = config.override(
    config.load(),
    **{"map.size": 6.0, "trav.block": 0.2, "trav.clearance": 1, "map.support": 0.3},
)
STEP = CFG["trav"]["step_max"]


def noisy_wall_map(support, seed=0, sigma=0.04, n=40000):
    """A 0.3 m thick wall along y = 0 seen from both sides: face returns (z 0.2-1.8) with `sigma` lateral error, plus floor."""
    rng = np.random.default_rng(seed)
    m = elevmap.ElevationMap(config.override(CFG, **{"map.support": support}))
    for _ in range(10):  # ten frames
        x = rng.uniform(-2.5, 2.5, n)
        side = rng.choice([-0.15, 0.15], n)
        y = side + rng.normal(0, sigma, n)
        z = rng.uniform(0.2, 1.8, n)
        fx, fy = rng.uniform(-2.9, 2.9, 4 * n), rng.uniform(-2.9, 2.9, 4 * n)
        keep = np.abs(fy) > 0.17  # floor everywhere outside the wall footprint
        xs = np.r_[x, fx[keep]]
        ys = np.r_[y, fy[keep]]
        zs = np.r_[z, np.zeros(keep.sum())]
        m.fuse_z(xs, ys, zs, np.full(xs.size, 1e-4), 0.0)
    return m


def wall_extent(m):
    obs = clearance.obstacles(m.hn, 0.05, STEP, m.support() if m.sup else None)
    ys = np.flatnonzero(obs[:, 50:70].any(axis=1))
    return (ys.max() - ys.min() + 1) * 0.05 if ys.size else 0.0


def test_mapped_wall_thickness_is_within_0_1_m_of_the_truth():
    # regression (ruins 50 m): walls mapped 0.6-0.7 m thick against 0.3 m; the first return in the tail of the blur flagged the floor
    # beside the wall
    plain, supported = noisy_wall_map(0.0), noisy_wall_map(0.3)
    assert wall_extent(plain) > 0.45  # the problem is real in this scene
    assert abs(wall_extent(supported) - 0.3) <= 0.1 + 1e-6  # two 5 cm cells


def test_support_is_off_by_default_so_rubble_maps_are_unchanged():
    cfg = config.override(config.load(), **{"map.size": 6.0})
    assert cfg["map"]["support"] == 0
    assert not hasattr(elevmap.ElevationMap(cfg), "nhi")


def test_a_mapped_06m_door_reads_open_at_clearance_020():
    # the mapped doors of the ruins are about 0.6 m wide (0.8 m real): the centre line has 0.30 m, above the 0.20 m limit
    n, res = 120, 0.05
    xc = -3.0 + (np.arange(n) + 0.5) * res
    hn = np.zeros((n, n), np.float32)
    wall = (np.abs(xc)[:, None] <= 0.15) & ~(np.abs(xc[None, :]) <= 0.30)
    hn[wall] = 1.0
    D = clearance.edt(clearance.obstacles(hn, res, STEP), res)
    cfg = config.override(CFG, **{"map.size": 6.0})
    cost = clearance.block_cost(np.ones((30, 30)), D, cfg)
    assert np.isfinite(cost[14:16, 13:17]).any()


def test_pursue_turns_on_an_arc_only_where_there_is_room():
    lim = {"v_max": 1.5, "wz_max": 1.0}
    path = [(-1.0, 0.0), (-2.0, 0.0)]
    assert planner.pursue((0.0, 0.0, 0.0), path, CFG, lim) == (0.0, 1.0)
    v, wz = planner.pursue((0.0, 0.0, 0.0), path, CFG, lim, None, 0.0, 0.5)
    assert v == pytest.approx(0.5) and wz == pytest.approx(1.0)
    v, wz = planner.pursue(
        (0.0, 0.0, 0.0), [(2.0, 0.0)], CFG, lim, None, 0.0, 0.5
    )  # ahead: normal pursuit
    assert v > 1.0
