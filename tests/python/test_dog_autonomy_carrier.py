import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
import numpy as np
import pytest
from autonomy import carrier, config, lidar
from napl.terrain import field


def make(tmp_path, kind="scree", start=(0.0, 0.0, 0.0), rays=5000, seed=1, **ov):
    cfg = config.override(config.load(), **ov)
    f = field(kind, size=20.0, patch=10.0, seed=seed)
    p = carrier.build(f, cfg, rays, start, tmp_path)
    sim = napl.Sim.load(p)
    return carrier.Carrier(sim, f, cfg, start, seed=seed), f, cfg


def test_scree_points_match_ground_truth(tmp_path):
    c, f, cfg = make(tmp_path, **{"lidar.sigma0": 0.0})
    ros = lidar.Rosette(c.sim, "lidar", 5000, c.dt, cfg)
    ext = lidar.extrinsic((tmp_path / "carrier.xml").read_text(), "lidar", "carrier")
    c.command(1.0, 0.0, 0.2)
    errs = []
    for _ in range(10):
        c.advance(0.1)
        p_b, q_b = (
            c.truth()
        )  # the pose set at this chunk's start = the pose the frame was cast from
        s = ros.latest()
        assert s.t == pytest.approx(c.time_of_last_frame, abs=1e-9)
        R = lidar.qmat(q_b)
        w = lidar.points(s, p_b + R @ ext[0], lidar.qmul(q_b, ext[1]))
        keep = (np.abs(w[:, :2]) < 9.5).all(1) & (np.linalg.norm(w - p_b, axis=1) < 12)
        errs.append(w[keep, 2] - f.height(w[keep, 0], w[keep, 1]))
    e = np.abs(np.concatenate(errs))
    assert np.median(e) < 1e-3 and np.quantile(e, 0.99) < 2e-3


def test_commands_are_clamped_and_rate_limited(tmp_path):
    c, f, cfg = make(tmp_path)
    C = cfg["carrier"]
    c.command(5.0, 0.0, 3.0)
    c.advance(0.1)
    assert c.v[0] <= C["a_max"] * 0.1 + 1e-12 and c.v[2] <= C["alpha_max"] * 0.1 + 1e-12
    c.advance(3.0)
    assert c.v[0] == pytest.approx(C["v_max"]) and c.v[2] == pytest.approx(C["wz_max"])
    assert c.limits() == {"v_max": C["v_max"], "wz_max": C["wz_max"]}


def test_drift_statistics_follow_the_config():
    cfg = config.load()
    O = cfg["odom"]
    yaw_err, z_err = [], []
    for seed in range(400):
        d = carrier.DriftOdom(cfg, seed)
        for k in range(101):  # 100 m straight, 1 m per update
            o = d.update(
                0.1 * k, np.array([float(k), 0.0, 0.0]), np.array([1.0, 0, 0, 0])
            )
        yaw_err.append(lidar.yaw_of(o.quat))
        z_err.append(o.pos[2])
    assert np.std(yaw_err) == pytest.approx(O["drift_yaw"] * 10, rel=0.15)
    assert np.std(z_err) == pytest.approx(O["drift_z"] * 10, rel=0.15)


def test_bump_stops_at_a_wall(tmp_path):
    f = field("ruins", size=20.0, patch=10.0, seed=2)
    b = max(
        (b for b in f.boxes if abs(b.rpy[2]) < 1e-9 and b.size[0] > 1.0),
        key=lambda b: b.size[0],
    )
    start = (b.pos[0], b.pos[1] - b.size[1] - 1.5, math.pi / 2)
    c, f, cfg = make(tmp_path, kind="ruins", start=start, seed=2)
    c.command(1.0, 0.0, 0.0)
    c.advance(4.0)
    p, _ = c.truth()
    assert (
        c.bumps > 0 and p[1] + cfg["carrier"]["foot_a"] <= b.pos[1] - b.size[1] + 1e-9
    )


def test_sky_blocked_in_an_enclosure_not_in_the_open(tmp_path):
    cfg = config.load()
    walls = "".join(
        f'<geom type="box" size="{sx} {sy} 2" pos="{x} {y} 2"/>'
        for x, y, sx, sy in (
            (1.2, 0, 0.1, 1.3),
            (-1.2, 0, 0.1, 1.3),
            (0, 1.2, 1.3, 0.1),
            (0, -1.2, 1.3, 0.1),
        )
    )
    p = tmp_path / "e.xml"
    p.write_text(
        f'<mujoco><worldbody><geom type="box" size="30 30 0.1" pos="0 0 -0.1"/>{walls}</worldbody></mujoco>'
    )
    sim = napl.Sim.load(p)
    assert carrier.sky_blocked(sim, np.array([0.0, 0.0, 0.5]), cfg)
    assert not carrier.sky_blocked(sim, np.array([15.0, 0.0, 0.5]), cfg)


def test_same_seed_same_state_hash(tmp_path):
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    hs, odo = [], []
    for d in ("a", "b"):
        c, f, cfg = make(tmp_path / d)
        c.command(1.2, 0.0, 0.3)
        c.advance(1.0)
        hs.append(c.sim.state_hash())
        odo.append(c.odometry().pos.copy())
    assert hs[0] == hs[1]
    np.testing.assert_array_equal(odo[0], odo[1])
