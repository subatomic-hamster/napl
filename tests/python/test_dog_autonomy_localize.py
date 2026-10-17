import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
import numpy as np
import pytest
from autonomy import carrier, config, icp, lidar, localize
from autonomy.iface import Odom
from napl.terrain import field

CFG = config.load()
LEVER = np.array([CFG["gnss"]["lever_x"], 0.0, CFG["gnss"]["lever_z"]])


def straight(
    yaw_map=0.7, off=0.3, t0=np.array([2.0, -1.0, 0.0]), pitch=0.1, v=1.5, n=11
):
    """Truth poses along yaw_map at v, and the same motion seen in an odometry frame rotated by -off (no drift)."""
    out = []
    for k in range(n):
        t = 0.1 * k
        p = t0 + np.array([v * t * math.cos(yaw_map), v * t * math.sin(yaw_map), 0.3])
        q = lidar.quat_zyx(yaw_map, pitch, 0.0)
        po = localize.rz(-off) @ (p - t0)
        out.append(
            (
                t,
                p,
                q,
                Odom(
                    t,
                    po,
                    lidar.quat_zyx(yaw_map - off, pitch, 0.0),
                    np.zeros(3),
                    np.zeros(3),
                    False,
                ),
            )
        )
    return out


def antenna(p, q):
    return p + lidar.qmat(q) @ LEVER


def test_late_rtk_fix_is_compared_with_the_pose_at_its_epoch():
    run = straight()
    t, p, q, _ = run[0]
    loc = localize.Localizer(CFG, np.r_[p, 0.7], LEVER)
    for _, _, _, od in run:
        loc.predict(od)
    x_before = loc.x.copy()
    _, p9, q9, _ = run[
        9
    ]  # the fix measured at t = 0.9 arrives at t = 1.0 (latency 0.1 s)
    assert loc.update_rtk(0.9, antenna(p9, q9))
    np.testing.assert_allclose(
        loc.x, x_before, atol=1e-9
    )  # exact fix at its own epoch: no correction
    loc2 = localize.Localizer(CFG, np.r_[p, 0.7], LEVER)
    for _, _, _, od in run:
        loc2.predict(od)
    loc2.update_rtk(
        1.0, antenna(p9, q9)
    )  # the same fix wrongly stamped "now": pulls the pose back by ~v * latency
    assert np.linalg.norm(loc2.x[:2] - x_before[:2]) > 0.01
    loc3 = localize.Localizer(CFG, np.r_[p, 0.7], LEVER)
    for _, _, _, od in run:
        loc3.predict(od)
    assert loc3.update_rtk(0.9, antenna(p9, q9) + np.array([0.05, 0.0, 0.0]))
    assert 0.0 < loc3.x[0] - x_before[0] < 0.05


def test_gate_rejects_a_2m_jump():
    run = straight()
    loc = localize.Localizer(CFG, np.r_[run[0][1], 0.7], LEVER)
    for _, _, _, od in run:
        loc.predict(od)
    x = loc.x.copy()
    assert not loc.update_rtk(
        0.9, antenna(run[9][1], run[9][2]) + np.array([2.0, 0.0, 0.0])
    )
    np.testing.assert_array_equal(loc.x, x)


class FakeGnss:
    def __init__(self):
        self.tick, self.vals = 0, {}

    def query(self, path, attr):
        return np.array([self.vals[attr]]), "deg"


def test_held_fix_is_an_outage_and_reconvergence_waits():
    G = CFG["gnss"]
    sim = FakeGnss()
    r = localize.GnssReader(sim, "gnss", CFG, 0.01)
    got = {}
    for s in range(121):
        sim.tick = s * 10 + 1  # sample s happened on tick 10 s
        k = 4 if 5 <= s <= 8 else s  # samples 5..8 hold sample 4's output (fix_lost)
        sim.vals = {"lat": G["lat0"] + 1e-7 * k, "lon": G["lon0"], "alt": 0.1 * k}
        f = r.poll()
        if f is not None:
            got[s] = f
        assert r.poll() is None  # no new sample until the tick moves on
    assert sorted(got)[:4] == [1, 2, 3, 4] and got[1][0] == pytest.approx(0.0)
    after = [s for s in got if s > 4]
    # the first fresh sample after the hold is s = 9 (t_meas 0.8 s); fixes are usable from 0.8 + reconverge_s = 10.8 s (s = 109)
    assert after[0] in (109, 110) and got[after[0]][0] >= 0.8 + G["reconverge_s"] - 1e-9
    m, _ = localize.radii(G["lat0"])
    assert got[2][1][1] == pytest.approx(
        math.radians(2e-7) * m, rel=1e-6
    )  # lat0 + 2e-7 loses ~4e-8 relative to double rounding


def room(n, rng, noise=0.005):
    """Floor z = 0.1, walls x / y = +-5.1, a pillar with x faces at 1.7 / 2.3: surfaces mid-voxel (0.2 m voxels), so the
    noise does not split a surface over two voxel layers."""
    u = rng.uniform(-1, 1, (n, 2))
    k = rng.integers(0, 6, n)
    pts = np.zeros((n, 3))
    pts[k == 0] = np.c_[5 * u[k == 0], np.full((k == 0).sum(), 0.1)]
    for j, (ax, val) in enumerate(((0, 5.1), (0, -5.1), (1, 5.1), (1, -5.1)), start=1):
        m = k == j
        pts[m, ax] = val
        pts[m, 1 - ax] = 5 * u[m, 0]
        pts[m, 2] = 1.1 + u[m, 1]
    m = k == 5
    pts[m] = np.c_[np.where(u[m, 0] > 0, 2.3, 1.7), 1 + 0.25 * u[m, 1], 1.1 + u[m, 0]]
    return pts + noise * rng.standard_normal(pts.shape)


def test_icp_recovers_an_offset_in_a_synthetic_room():
    rng = np.random.default_rng(0)
    truth = np.array([0.3, -0.2, 0.45, 0.2])
    sm = icp.ScanMatcher(CFG)
    assert sm.add(room(150000, rng), truth)  # ~10 floor points per 0.2 m voxel
    w = room(8000, rng)
    s = (w - truth[:3]) @ localize.rz(
        truth[3]
    )  # levelled base frame: Rz(yaw)^T (w - p)
    m = sm.match(s, truth + np.array([0.15, -0.10, 0.05, 0.03]))
    assert np.abs(m.x[:3] - truth[:3]).max() < 0.01 and abs(m.x[3] - truth[3]) < 0.003
    assert not m.degenerate.any() and m.inliers > 1000


def test_flat_floor_is_degenerate_in_x_y_and_yaw():
    rng = np.random.default_rng(1)
    floor = np.c_[
        rng.uniform(-5, 5, (40000, 2)), 0.1 + 0.003 * rng.standard_normal(40000)
    ]
    sm = icp.ScanMatcher(CFG)
    sm.add(floor, np.zeros(4))
    guess = np.array([0.1, 0.1, 0.4, 0.05])
    m = sm.match(
        floor[:5000] - np.array([0.0, 0.0, 0.35]), guess
    )  # true z = 0.1 + 0.35 = 0.45 is wrong: the floor is at 0.1, so z = 0.35 (plan arithmetic slip)
    np.testing.assert_array_equal(m.degenerate, [True, True, False, True])
    np.testing.assert_allclose(
        m.x[[0, 1, 3]], guess[[0, 1, 3]], atol=1e-3
    )  # normals are only nearly vertical
    assert m.x[2] == pytest.approx(0.35, abs=0.005)


def test_ruins_circle_with_forced_outage_icp_bounds_the_drift(tmp_path):
    cfg = config.override(
        CFG,
        **{
            "gnss.sky_block": 0.0,
            "odom.drift_xy": 0.05,
            "odom.drift_yaw": 0.02,
            "map.size": 20.0,
        },
    )
    f = field("ruins", size=20.0, patch=10.0, seed=3)
    start = (
        4.0,
        2.5,
        math.pi / 2,
    )  # radius 1.5 m circle around the cell centre (2.5, 2.5)
    sim = napl.Sim.load(carrier.build(f, cfg, 5000, start, tmp_path))
    c = carrier.Carrier(sim, f, cfg, start, seed=3)
    ros = lidar.Rosette(sim, "lidar", 5000, c.dt, cfg)
    ext = lidar.extrinsic((tmp_path / "carrier.xml").read_text(), "lidar", "carrier")
    p0, q0 = c.truth()
    x0 = np.r_[p0, lidar.yaw_of(q0)]
    with_icp, odo_only = (
        localize.Localizer(cfg, x0, LEVER),
        localize.Localizer(cfg, x0, LEVER),
    )
    sm = icp.ScanMatcher(cfg)
    c.command(1.0, 0.0, 1.0 / 1.5)
    for k in range(300):
        c.advance(0.1)
        od, s = c.odometry(), ros.latest()
        with_icp.predict(od)
        odo_only.predict(od)
        pts = icp.levelled(s, ext, od.quat)
        if k:
            m = sm.match(pts, with_icp.x.copy())
            if m is not None:
                with_icp.update_icp(m.x, m.cov)
        sm.add(pts @ localize.rz(with_icp.x[3]).T + with_icp.x[:3], with_icp.x)
    assert c.gnss_lost
    p, _ = c.truth()
    e_icp, e_odo = (float(np.hypot(*(L.x[:2] - p[:2]))) for L in (with_icp, odo_only))
    assert e_icp < 0.15 and e_icp < 0.5 * e_odo
