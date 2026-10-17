import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
import numpy as np
import pytest
from autonomy import carrier, config, elevmap, lidar, metrics
from napl.terrain import field

CFG = config.load()
SMALL = config.override(CFG, **{"map.size": 10.0})  # 200 x 200 cells


def test_gate_keeps_the_higher_surface_and_kalman_shrinks_variance():
    m = elevmap.ElevationMap(SMALL)
    x, y = np.array([0.01]), np.array([0.01])
    m.fuse_z(x, y, np.array([0.0]), np.array([1e-4]), 0.0)
    assert m.var(0.01, 0.01) == pytest.approx(1e-4, rel=1e-6)
    m.fuse_z(x, y, np.array([0.01]), np.array([1e-4]), 0.0)  # within the gate: Kalman
    assert m.var(0.01, 0.01) == pytest.approx(5e-5, rel=1e-6) and m.height(
        0.01, 0.01
    ) == pytest.approx(0.005, abs=1e-7)
    m.fuse_z(
        x, y, np.array([0.30]), np.array([1e-4]), 0.0
    )  # higher by > 3 sigma: a surface on top -> replace
    assert m.height(0.01, 0.01) == pytest.approx(0.30, abs=1e-7) and m.var(
        0.01, 0.01
    ) == pytest.approx(1e-4, rel=1e-6)
    m.fuse_z(
        x, y, np.array([0.0]), np.array([1e-4]), 0.0
    )  # lower by > 3 sigma: dropped (no smearing below a top)
    assert (
        m.height(0.01, 0.01) == pytest.approx(0.30, abs=1e-7)
        and int(m.hits[100, 100]) == 3
    )
    m2 = elevmap.ElevationMap(
        SMALL
    )  # four points of one frame in one cell: mean + the frame's common variance
    m2.fuse_z(
        np.full(4, 0.01),
        np.full(4, 0.01),
        np.array([0.0, 0.02, 0.0, 0.02]),
        np.full(4, 4e-4),
        1e-4,
    )
    assert m2.height(0.01, 0.01) == pytest.approx(0.01, abs=1e-7) and m2.var(
        0.01, 0.01
    ) == pytest.approx(2e-4, rel=1e-6)
    assert (
        math.isnan(m2.height(3.0, 3.0))
        and math.isinf(m2.var(3.0, 3.0))
        and math.isnan(m2.height(9.0, 0.0))
    )
    assert m2.dirty == (100, 101, 100, 101)


def test_point_variance_model():
    m = elevmap.ElevationMap(SMALL)
    L = CFG["lidar"]
    o = np.array([0.0, 0.0, 2.0])
    pts = np.array(
        [[0.01, 0.01, 0.0], [4.0, 0.01, 0.0], [20.0, 0.0, 0.0]]
    )  # down, 4 m out, beyond max_range
    m.fuse(pts, o, 1e-4, 0.005)
    assert m.var(0.01, 0.01) == pytest.approx(
        L["sigma0"] ** 2
        + 0.005**2 * 0.01**2 * 2
        + L["ang_sigma"] ** 2 * 0.01**2 * 2
        + 1e-4,
        rel=1e-3,
    )
    r2, rh2 = 4.0**2 + 0.01**2 + 4.0, 4.0**2 + 0.01**2
    exp = L["sigma0"] ** 2 * 4.0 / r2 + (L["ang_sigma"] ** 2 + 0.005**2) * rh2 + 1e-4
    assert m.var(4.0, 0.01) == pytest.approx(exp, rel=1e-3)
    assert int(m.hits.sum()) == 2


def test_truth_grid_is_ground_truth_at_cell_centres():
    f = field("rubble", size=20.0, patch=10.0, seed=1)
    g = metrics.truth_grid(f, 0.05)
    rng = np.random.default_rng(0)
    r, c = rng.integers(0, 400, 500), rng.integers(0, 400, 500)
    xc, yc = -10.0 + (c + 0.5) * 0.05, -10.0 + (r + 0.5) * 0.05
    np.testing.assert_array_equal(g[r, c], f.height(xc, yc))
    b = f.boxes[0]
    i, j = int((b.pos[0] + 10.0) / 0.05), int((b.pos[1] + 10.0) / 0.05)
    assert (
        g[j, i] >= b.pos[2] + b.size[2]
    )  # rubble always has 40 boulders; another one may overlap higher


def ring_world():
    n = 200
    gt = np.zeros((n, n))
    gt[60:70, 60:140] = gt[130:140, 60:140] = gt[60:140, 60:70] = gt[
        60:140, 130:140
    ] = 1.0  # walls: blocks 6 and 13
    x = np.arange(n) * 0.05
    gt[0:40, 160:200] = (x[160:200] - x[160]) * math.tan(
        math.radians(35)
    )  # a 35 deg ramp
    return gt


def test_coverage_denominator_is_ground_truth_reachable_traversable():
    gt = ring_world()
    tgt = metrics.targets(gt, SMALL, (-4.5, -4.5))
    assert tgt[20, 20] and not tgt[100, 100] and not tgt[65, 100] and not tgt[20, 180]
    m = elevmap.ElevationMap(SMALL)
    m.h[:], m.v[:] = gt, 1e-4
    assert metrics.coverage(m, tgt, 1) == 1.0
    m.h[10:30, :] += (
        1.0  # the robot's map believes in a wall: the denominator does not move
    )
    assert (
        metrics.coverage(m, tgt, 1) == 1.0
        and metrics.targets(gt, SMALL, (-4.5, -4.5)).sum() == tgt.sum()
    )
    m.v[:, 100:] = np.inf  # only the west half seen
    assert metrics.coverage(m, tgt, 1) == pytest.approx(tgt[:, :100].sum() / tgt.sum())
    assert metrics.coverage(m, tgt, 2) == pytest.approx(
        tgt[:, :100].sum() / tgt.sum(), abs=0.01
    )


def test_edge_cells_are_scored_separately():
    gt = np.zeros((200, 200))
    gt[80:120, 80:120] = 1.0
    m = elevmap.ElevationMap(SMALL)
    m.h[:], m.v[:] = gt, 1e-4
    edges = metrics.edge_mask(gt, SMALL)
    m.h[edges] = 0.5  # mixed returns on the box sides
    r = metrics.height_rms(m, gt, edges)
    assert (
        r["rms"] == 0.0
        and r["rms_edges"] > 0.3
        and r["n_edges"] == edges.sum()
        and 0 < r["n_edges"] < 400
    )


def test_edge_margin_widens_the_edge_band_by_whole_cells():
    gt = np.zeros((200, 200))
    gt[80:120, 80:120] = 1.0
    base = metrics.edge_mask(gt, SMALL)
    wide = metrics.edge_mask(gt, config.override(SMALL, **{"map.edge_margin": 0.30}))
    assert (wide & base).sum() == base.sum() and wide.sum() > base.sum()
    assert (
        wide[80 - 6, 100] and not wide[80 - 7, 100]
    )  # 0.30 m = 6 cells out from the wall (the 3x3 test covers the first)


def test_rubble_pass_at_20k_rays_with_truth_pose_maps_within_3cm(tmp_path):
    cfg = config.override(CFG, **{"map.size": 20.0})
    f = field(
        "rubble", size=20.0, patch=10.0, seed=4, rocks=100, boulders=2
    )  # the 100 x 100 m densities scaled to 400 m^2
    start = (-8.0, 0.0, 0.0)
    sim = napl.Sim.load(carrier.build(f, cfg, 20000, start, tmp_path))
    c = carrier.Carrier(sim, f, cfg, start)
    ros = lidar.Rosette(sim, "lidar", 20000, c.dt, cfg)
    ext = lidar.extrinsic((tmp_path / "carrier.xml").read_text(), "lidar", "carrier")
    m, rng = elevmap.ElevationMap(cfg), np.random.default_rng(0)
    c.command(1.5, 0.0, 0.0)
    for _ in range(110):
        c.advance(0.1)
        p, q = c.truth()
        s = ros.latest()
        o = p + lidar.qmat(q) @ ext[0]
        m.fuse(
            lidar.points(s, o, lidar.qmul(q, ext[1]), cfg["lidar"]["ang_sigma"], rng),
            o,
            0.0,
            0.0,
        )
    gt = metrics.truth_grid(f, cfg["map"]["res"])
    r = metrics.height_rms(m, gt, metrics.edge_mask(gt, cfg))
    assert r["rms"] <= 0.03 and r["n"] > 20000
