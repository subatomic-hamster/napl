import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
import numpy as np
import pytest
from autonomy import config, lidar

DT = 0.01


def tilted_scene(tmp_path, rays=2000, sigma0=0.0):
    cfg = config.override(config.load(), **{"lidar.sigma0": sigma0})
    q = lidar.qmul(lidar.quat_zyx(0.3, cfg["lidar"]["tilt"], 0.0), lidar.SITE_Q0)
    dev = {"rangefinders": [lidar.device("lidar", "lidar", cfg, rays)]}
    qs = " ".join(
        repr(float(v)) for v in q
    )  # numpy 2 reprs np.float64 as "np.float64(...)"
    xml = (
        f'<mujoco><option timestep="{DT}" gravity="0 0 0"/><worldbody>'
        '<geom type="box" size="20 20 0.1" pos="0 0 -0.1"/>'
        '<geom type="box" size="0.1 3 2" pos="6.0 0 1"/>'
        '<body name="head" pos="0.2 -0.1 0.45">'
        f'<site name="lidar" quat="{qs}"/></body>'
        f"</worldbody><custom><text name=\"napl.devices\" data='{json.dumps(dev)}'/></custom></mujoco>"
    )
    p = tmp_path / "m.xml"
    p.write_text(xml)
    return napl.Sim.load(p), cfg, np.array([0.2, -0.1, 0.45]), q


def test_points_match_the_kernel_rays_on_a_tilted_site(tmp_path):
    sim, cfg, pos, q = tilted_scene(tmp_path)
    ros = lidar.Rosette(sim, "lidar", 2000, DT, cfg)
    sim.step(30)  # samples on ticks 0, 10, 20
    s = ros.latest()
    assert s.frame == 2 and s.t == pytest.approx(0.20, abs=1e-12)
    assert ros.latest() is None  # nothing new until the next frame
    R = lidar.qmat(q)
    d_world = s.dirs @ R.T
    again = sim.raycast(pos, d_world)
    np.testing.assert_allclose(again, s.r, atol=1e-9)
    w = lidar.points(s, pos, q)
    on_floor = np.abs(w[:, 2]) < 1e-6
    on_wall = np.abs(w[:, 0] - 5.9) < 1e-6
    assert (
        (on_floor | on_wall).all() and on_wall.sum() > 20 and on_floor.sum() > 300
    )  # 2000 rays: 362 floor + 169 wall returns (the plan's 500 miscounted the -7..52 deg span tilted 25 deg down)
    full = lidar.directions(
        2000,
        2,
        cfg["lidar"]["fov_v_min"],
        cfg["lidar"]["fov_v_max"],
        cfg["lidar"]["pattern_seed"],
    )
    el = np.arcsin(np.clip(full[:, 0], -1, 1))
    assert (
        el.min() >= cfg["lidar"]["fov_v_min"] - 1e-12
        and el.max() <= cfg["lidar"]["fov_v_max"] + 1e-12
    )
    frame0 = lidar.directions(
        2000,
        0,
        cfg["lidar"]["fov_v_min"],
        cfg["lidar"]["fov_v_max"],
        cfg["lidar"]["pattern_seed"],
    )
    assert not np.allclose(frame0, full)  # the sequence continues, frames do not repeat


def test_angular_noise_has_the_datasheet_sigma(tmp_path):
    sim, cfg, pos, q = tilted_scene(tmp_path, rays=20000)
    ros = lidar.Rosette(sim, "lidar", 20000, DT, cfg)
    sim.step(1)
    s = ros.latest()
    sig = cfg["lidar"]["ang_sigma"]
    clean = lidar.points(s, pos, q) - pos
    noisy = lidar.points(s, pos, q, sig, np.random.default_rng(0)) - pos
    cosang = (clean * noisy).sum(1) / (
        np.linalg.norm(clean, axis=1) * np.linalg.norm(noisy, axis=1)
    )
    ang = np.arccos(np.clip(cosang, -1, 1))
    assert math.sqrt((ang**2).mean()) == pytest.approx(sig * math.sqrt(2), rel=0.05)


def test_quaternion_helpers_and_the_extrinsic_chain():
    for yaw in (0.0, 1.0, math.pi, -math.pi / 2):
        q = lidar.quat_zyx(yaw, 0.2, -0.1)
        assert np.isfinite(q).all() and math.cos(
            lidar.yaw_of(q) - yaw
        ) == pytest.approx(1.0, abs=1e-12)
        R = lidar.qmat(q)
        cy, sy, cp, sp, cr, sr = (
            math.cos(yaw),
            math.sin(yaw),
            math.cos(0.2),
            math.sin(0.2),
            math.cos(-0.1),
            math.sin(-0.1),
        )
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        np.testing.assert_allclose(R, Rz @ Ry @ Rx, atol=1e-12)
    R0 = lidar.qmat(lidar.SITE_Q0)
    np.testing.assert_allclose(
        R0, [[0, 0, 1], [0, -1, 0], [1, 0, 0]], atol=1e-12
    )  # +x up, +z forward
    xml = (
        '<mujoco><worldbody><body name="base"><body name="head" pos="0.25 0 0.1" euler="0 0 90">'
        '<site name="s" pos="0.05 0 0.02" quat="0 0.7071067811865476 0 0.7071067811865476"/></body></body></worldbody></mujoco>'
    )
    p, qq = lidar.extrinsic(xml, "s", "base")
    np.testing.assert_allclose(p, [0.25, 0.05, 0.12], atol=1e-12)
    np.testing.assert_allclose(
        lidar.qmat(qq), lidar.qmat(lidar.quat_zyx(math.pi / 2, 0, 0)) @ R0, atol=1e-12
    )
    with pytest.raises(ValueError, match="xyaxes"):
        lidar.extrinsic(
            xml.replace('euler="0 0 90"', 'xyaxes="0 1 0 -1 0 0"'), "s", "base"
        )
