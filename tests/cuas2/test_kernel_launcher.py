"""Kernel launcher device with the cuas2 per-shot attrs (spec 7.4, acceptance MODEL 4): additive (attrs at 0 -> identical flight), Magnus lift equal
to the model, aim offsets in the documented directions, per-shot muzzle velocity, and the plant driving it end to end.

The lite replay hashes are the other half of 'additive' (a launcher without cl_k / spin_tau has no extra state): tests/lite/test_lockstep.py -k hash."""

import math

import numpy as np
import pytest

pytest.importorskip("napl._napl")
from napl.cuas2 import ballistics as B
from napl.cuas2 import calib, sim_mcu
from napl.cuas2.launcher import Shot
from napl.cuas2.sim_io import LauncherRig

C = calib.Calib()
BALL = (C.ball.d_mm / 1000, C.ball.m_g / 1000, C.ball.cd)
PLATE_X = 10.0 - 0.001  # front face of the 2 mm plate at x = 10
MUZZLE = np.array([0.05, 0.0, 10.0])


def shoot(tmp_path, name, shot, **scene):
    rig = LauncherRig(tmp_path / name, C, **scene)
    rig.fire(shot)
    rig.step(900)
    return rig, rig.hit()


def model_hit(v0, omega_y=0.0, el=0.0, az=0.0):
    u = np.array(
        [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
    )
    t, p, _ = B.fly(
        MUZZLE,
        v0 * u,
        B.omega_vec(u, omega_y),
        BALL,
        cl_k=C.launcher.cl_k,
        spin_tau=C.launcher.spin_tau_s,
        dt=0.0005,
        stop_x=PLATE_X,
    )
    i = np.searchsorted(p[:, 0], PLATE_X)
    f = (PLATE_X - p[i - 1, 0]) / (p[i, 0] - p[i - 1, 0])
    return p[i - 1] + f * (p[i] - p[i - 1])


def S(v0=30.0, omega=0.0, err=(0.0, 0.0)):
    return Shot(v0, err, omega, BALL[0], BALL[1], BALL[2], 0.0)


def test_attrs_at_zero_are_bit_identical_to_a_device_without_them(tmp_path):
    plain = LauncherRig(
        tmp_path / "plain", C, cl_k=0.0, spin_tau=0.0
    )  # no cl_k / spin_tau: the cuas-lite device, no extra state, no new attrs
    with pytest.raises((KeyError, ValueError)):
        plain.get("spin_rps")
    plain.sim.set(plain.path, "fire", 1.0)
    plain.step(900)
    rig = LauncherRig(
        tmp_path / "ext", C
    )  # cl_k 0.75: the per-shot block exists but is all zero
    assert rig.get("spin_rps")[0] == 0.0 and rig.get("muzzle_velocity")[0] == 0.0
    rig.sim.set(rig.path, "fire", 1.0)
    rig.step(900)
    np.testing.assert_array_equal(
        rig.hit(), plain.hit()
    )  # not just close: the same doubles


def test_no_spin_flight_matches_the_python_model_to_1cm(tmp_path):
    rig, hit = shoot(tmp_path, "a", S(30.0))
    assert hit[0] == pytest.approx(PLATE_X, abs=1e-6)
    assert np.linalg.norm(hit - model_hit(30.0)) < 0.01
    assert hit[2] < 10.0 - 0.3  # it dropped: drag and gravity are in


def test_backspin_height_difference_equals_the_model_within_1cm(tmp_path):
    omega = -1.0 / 0.0222 * 0.8  # 1 m/s wheel difference at k_spin 0.8
    _, h0 = shoot(tmp_path, "n", S(30.0))
    _, h1 = shoot(tmp_path, "s", S(30.0, omega))
    dz_kernel = h1[2] - h0[2]
    dz_model = model_hit(30.0, omega)[2] - model_hit(30.0)[2]
    assert dz_kernel > 0.05  # backspin lifts
    assert dz_kernel == pytest.approx(dz_model, abs=0.01)
    _, h2 = shoot(tmp_path, "t", S(30.0, -omega))  # topspin: the mirror image
    assert (
        h2[2] < h0[2] - 0.05 and abs(h2[1]) < 1e-6 and abs(h1[1]) < 1e-6
    )  # no sideways force for spin about the lateral axis


def test_aim_offsets_go_where_the_convention_says(tmp_path):
    _, h0 = shoot(tmp_path, "o0", S(30.0))
    _, hv = shoot(tmp_path, "ov", S(30.0, err=(0.0, 0.01)))  # +10 mrad vertical = up
    assert hv[2] - h0[2] == pytest.approx(0.10, abs=0.01) and abs(hv[1]) < 1e-6
    _, hh = shoot(
        tmp_path, "oh", S(30.0, err=(0.01, 0.0))
    )  # +10 mrad horizontal = left = +y (world)
    assert hh[1] == pytest.approx(0.10, abs=0.005) and hh[2] == pytest.approx(
        h0[2], abs=0.002
    )
    assert np.linalg.norm(hh - model_hit(30.0, az=0.01)) < 0.01


def test_muzzle_velocity_attr_sets_each_shot(tmp_path):
    rig = LauncherRig(tmp_path / "v", C)
    zs = []
    for v in (24.0, 34.0):
        rig.fire(S(v))
        rig.step(900)
        zs.append(rig.hit()[2])
        assert np.linalg.norm(rig.hit() - model_hit(v)) < 0.015
    assert zs[0] < zs[1] - 0.3  # the slower ball drops more
    assert rig.rounds == 2


def test_plant_drives_the_kernel_end_to_end(tmp_path):
    rig = LauncherRig(tmp_path / "plant", C)
    p = sim_mcu.Plant2(rig=rig, seed=11)
    for _ in range(2500):
        p.tick(speed_req_pm=1000)
        if p.t_ms > 600 and p.outs[-1].status_fields["ready"]:
            break
    p.tick(speed_req_pm=1000, cmd="FIRE")
    p.run(200, speed_req_pm=1000)
    rig.step(900)
    assert (
        rig.rounds == 1 == p.feeder.fire_calls == len(p.feeder.feed_edges)
    )  # one FIRE, one FEED_REQ, one pellet
    shot = p.feeder.shots[0][1]
    hit = rig.hit()
    assert hit is not None and abs(hit[0] - PLATE_X) < 1e-6
    assert (
        np.linalg.norm(
            hit - model_hit(shot.v0, shot.omega, el=shot.dir_err[1], az=shot.dir_err[0])
        )
        < 0.015
    )
