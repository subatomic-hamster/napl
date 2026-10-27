"""LauncherModel (spec 7.1-7.2, acceptance MODEL 2): speed, spin sign and size, energy dip, sampled scatter, velocity cap, wheel dynamics."""

import math
from dataclasses import replace

import numpy as np
import pytest
from napl.cuas2 import calib
from napl.cuas2.launcher import LauncherModel, dshot_to_u

C = calib.Calib()


def rpm_for_dv(model, rpm, dv):
    """(rpm_top, rpm_bot) whose surface speeds differ by dv (bottom faster by dv)."""
    d_rpm = dv / (math.pi * C.launcher.wheel_d_mm / 1000 / 60)
    return rpm - d_rpm / 2, rpm + d_rpm / 2


def test_a_speed_is_k_slip_times_mean_surface_speed():
    m = LauncherModel(C)
    assert m.mean_v0((17905, 17905)) == pytest.approx(
        30.0, abs=0.05
    )  # k 0.8 x pi x 0.040 x 17905 / 60
    assert m.mean_v0(17905, diff_pm=0) == pytest.approx(30.0, abs=0.05)
    assert (
        m.mean_v0((17905, 17905), with_dip=True) < 30.0
    )  # the 7.2 in-contact dip lowers the exit speed ...
    assert m.mean_v0((17905, 17905), with_dip=True) == pytest.approx(
        m.mean_v0((17905, 17905)) * (1 - m.dip_fraction(17905, 17905) / 2)
    )  # ... by half the rpm drop


def test_b_spin_sign_and_size():
    m = LauncherModel(C)
    kc = replace(C, launcher=replace(C.launcher, k_spin=1.0))
    m1 = LauncherModel(kc)
    top, bot = rpm_for_dv(m1, 17000, 1.0)  # bottom wheel 1 m/s faster
    om = m1.spin_rad_s(top, bot)
    assert om < 0  # backspin is negative about +y
    assert abs(om) * 60 / (2 * math.pi) == pytest.approx(
        430, abs=5
    )  # 1 m/s over a 22.2 mm ball
    assert m.spin_rad_s(*rpm_for_dv(m, 17000, -1.0)) > 0  # top faster: topspin
    assert m.spin_rad_s(17000, 17000) == 0
    # the default differential (15 permille, bottom faster) is backspin through split()
    assert m.spin_rad_s(*m.split(17500)) < 0


def test_c_rpm_dip_per_shot():
    rc = replace(
        C, launcher=replace(C.launcher, diff_pm=0.0)
    )  # symmetric wheels: one number
    m = LauncherModel(rc, np.random.default_rng(1))
    m.omega[:] = 17500 * 2 * math.pi / 60
    w0 = m.omega.copy()
    shot = m.sample_shot()
    drop = 1 - m.omega / w0
    assert drop[0] == pytest.approx(drop[1], rel=1e-9)
    assert drop[0] * 100 == pytest.approx(
        5.3, abs=0.5
    )  # J 4e-6, eta 0.6, 1.86 g at ~29 m/s (spec 4.2: ~930 rpm, 5.3 %)
    assert shot.dip == pytest.approx(
        drop[0], rel=0.15
    )  # the sampled ball's own mass varies 4 %: dip follows within that


def test_d_sampled_scatter_matches_calibration():
    lo = replace(
        C.launcher,
        sigma_v_rel=0.04,
        sigma_v_floor_mps=0.6,
        spread_mrad=(7.0, 12.0),
        bias_mrad=(1.5, -2.5),
    )
    m = LauncherModel(replace(C, launcher=lo))
    b = m.sample_shots(5000, np.random.default_rng(3), setpoint=17000)
    v_mean = b.v0.mean()
    sig_expect = math.hypot(
        lo.sigma_v_rel * m.mean_v0(17000, with_dip=True), lo.sigma_v_floor_mps
    )
    assert b.v0.std() == pytest.approx(sig_expect, rel=0.03)
    assert v_mean == pytest.approx(m.mean_v0(17000, with_dip=True), rel=0.01)
    assert b.dir_err[:, 0].std() * 1e3 == pytest.approx(7.0, rel=0.03)
    assert b.dir_err[:, 1].std() * 1e3 == pytest.approx(12.0, rel=0.03)
    assert b.dir_err[:, 0].mean() * 1e3 == pytest.approx(1.5, abs=0.3)
    assert b.dir_err[:, 1].mean() * 1e3 == pytest.approx(-2.5, abs=0.4)
    assert b.d.std() * 1e3 == pytest.approx(
        C.ball.d_sigma_mm, rel=0.05
    ) and b.cd.mean() == pytest.approx(C.ball.cd, rel=0.01)


def test_e_cap_holds_whatever_is_requested():
    m = LauncherModel(C, np.random.default_rng(4))
    assert m.rpm_cap == 18143
    for req in (17500, 20000, 25000, 1e6):
        rt, rb = m.split(
            req, 60
        )  # even a +-60 permille differential stays under the cap on both wheels
        assert max(rt, rb) <= m.rpm_cap
        b = m.sample_shots(2000, np.random.default_rng(5), setpoint=req, diff_pm=60)
        assert b.v0.max() <= 38.0
    # a hot calibration (k_slip 1, big scatter) fed the raw wheel speeds still cannot pass 38 m/s: the sampler clips at v_cap
    hot = replace(
        C,
        launcher=replace(
            C.launcher, k_slip=1.0, sigma_v_rel=0.2, sigma_v_floor_mps=3.0
        ),
    )
    b = LauncherModel(hot).sample_shots(
        5000, np.random.default_rng(6), setpoint=(18143, 18143)
    )
    assert b.v0.max() <= 38.0 and (b.v0 == 38.0).any()


def test_wheel_dynamics_spin_up_and_telemetry():
    m = LauncherModel(C, np.random.default_rng(7))
    assert dshot_to_u(0) == 0 and dshot_to_u(48) == 0 and dshot_to_u(2047) == 1.0
    for _ in range(1500):
        m.step_wheels(1200, 1200)  # u = 0.59: steady state kv V u = 16.3k rpm
    ss = 2300 * 12 * dshot_to_u(1200)
    assert m.rpm[0] == pytest.approx(ss, rel=0.005)
    for _ in range(300):
        m.step_wheels(0, 0)
    assert m.rpm[0] < 100  # braking is limited, not instantaneous
    t = LauncherModel(C, np.random.default_rng(8))
    t.omega[:] = 17000 * 2 * math.pi / 60
    tel = np.array([t.telemetry() for _ in range(2000)])
    assert tel[:, 0].std() / 17000 == pytest.approx(
        C.wheel.rpm_sigma_pm / 1000, rel=0.1
    )
