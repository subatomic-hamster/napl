"""cuas2 ballistics (spec 7.3, acceptance MODEL 3): reproduces the v1 table without spin, Magnus lift matches its closed form, fly_batch matches fly."""

import math

import numpy as np
import pytest
from napl.cuas2 import ballistics as B
from napl.cuas2 import calib
from napl.cuas2.launcher import LauncherModel

C = calib.Calib()
BALL = (0.0222, 0.00186, 0.47)
# docs/cuas-lite/spec.md 5.3 / examples/cuas-lite/ballistics_table.py: elevation (deg) for a flat target 3..15 m away at 30 m/s
V1_TABLE = {3: 1.06, 5: 1.93, 8: 3.54, 10: 4.88, 12: 6.49, 15: 9.61}


@pytest.mark.parametrize("rng_m", sorted(V1_TABLE))
def test_zero_spin_reproduces_v1_table(rng_m):
    M = np.array([0.0, 0.0, 4.0])
    s = B.solve_lead2(M, M + [rng_m, 0, 0], np.zeros(3), v0=30.0, cl_k=0.0, dt=0.001)
    assert math.degrees(s["el"]) == pytest.approx(V1_TABLE[rng_m], abs=0.01)


def test_zero_spin_trajectory_equals_v1_flight():
    from napl.cuas import ballistics as v1

    p = v1.Pellet(v0=30.0, mass=BALL[1], diameter=BALL[0], cd=BALL[2])
    u = np.array([math.cos(0.08), 0.0, math.sin(0.08)])
    t1, p1, _ = v1.flight(p, [0, 0, 4.0], 30 * u, [0, 0, 0], t_max=0.5, dt=0.001)
    t2, p2, _ = B.fly(
        [0, 0, 4.0], 30 * u, None, BALL, cl_k=0.75, t_max=0.5, dt=0.001
    )  # cl_k set but omega None: no Magnus
    n = min(len(p1), len(p2))
    assert np.abs(p1[:n] - p2[:n]).max() < 1e-9


def test_backspin_lift_matches_the_closed_form():
    """dv = 1 m/s -> omega = 45 rad/s. a_L = rho A cl_k omega r v / (2 m) is linear in v, so the extra height at x = X over a flight of T is
    int_0^T (T - s) a_L(s) ds. Evaluate that with the no-spin trajectory's v(s) (an independent integral from the model's RK4 with spin)."""
    omega = -1.0 / 0.0222  # k_spin 1: 1 m/s difference, backspin
    z0 = 10.0
    t, p, v = B.fly([0, 0, z0], [30.0, 0, 0], None, BALL, stop_x=10.0, dt=0.001)
    ts, ps, vs = B.fly(
        [0, 0, z0],
        [30.0, 0, 0],
        [0, omega, 0],
        BALL,
        cl_k=0.75,
        spin_tau=6.0,
        stop_x=10.0,
        dt=0.001,
    )
    i0, i1 = np.searchsorted(p[:, 0], 10.0), np.searchsorted(ps[:, 0], 10.0)
    f0 = (10 - p[i0 - 1, 0]) / (p[i0, 0] - p[i0 - 1, 0])
    f1 = (10 - ps[i1 - 1, 0]) / (ps[i1, 0] - ps[i1 - 1, 0])
    dz_model = (ps[i1 - 1, 2] + f1 * (ps[i1, 2] - ps[i1 - 1, 2])) - (
        p[i0 - 1, 2] + f0 * (p[i0, 2] - p[i0 - 1, 2])
    )
    d, m, _ = BALL
    rho = float(B.rho_at(z0))
    area = math.pi * d * d / 4
    K = rho * area * 0.75 * abs(omega) * (d / 2) / (2 * m)  # a_L / v
    kd = (
        rho * BALL[2] * area / (2 * m)
    )  # drag k: the extra vertical speed is damped at k |v|
    speed = np.linalg.norm(v[: i0 + 1], axis=1)
    s = t[: i0 + 1]
    # linearised perturbation of the no-spin flight: d(dvz)/ds = K v e^(-s/tau) - kd v dvz,  dz = int dvz   (trapezoid, 1 ms)
    dvz = np.zeros_like(s)
    for j in range(1, len(s)):
        h = s[j] - s[j - 1]
        f0 = (
            K * speed[j - 1] * math.exp(-s[j - 1] / 6.0)
            - kd * speed[j - 1] * dvz[j - 1]
        )
        pred = dvz[j - 1] + h * f0
        f1 = K * speed[j] * math.exp(-s[j] / 6.0) - kd * speed[j] * pred
        dvz[j] = dvz[j - 1] + 0.5 * h * (f0 + f1)
    dz_closed = float(np.trapezoid(dvz, s))
    assert (
        dz_model > 0.05
    )  # lift: a visible fraction of a ball diameter's worth at 10 m (~ 0.1 m)
    assert dz_model == pytest.approx(dz_closed, rel=0.15)
    # and the sign: topspin drops it by the same amount
    _, pt, _ = B.fly(
        [0, 0, z0], [30.0, 0, 0], [0, -omega, 0], BALL, stop_x=10.0, dt=0.001
    )
    assert pt[np.searchsorted(pt[:, 0], 10.0), 2] < p[i0, 2] - 0.05


def test_cl_saturates_at_0_25():
    # a huge spin: C_L = min(cl_k S, 0.25) -> lift acceleration capped at 0.25 * rho A v^2 / (2 m)
    d, m, _ = BALL
    a = B._acc(np.array([0, 0, 10.0]), np.array([30.0, 0, 0]), 0.0, np.zeros(3), BALL[2] * math.pi * d * d / (8 * m), math.pi * d * d / (8 * m), d / 2,
               np.array([0, -1e5, 0.0]), 0.75, 6.0, 0.0)  # fmt: skip
    cap = B.CL_MAX * float(B.rho_at(10.0)) * (math.pi * d * d / 4) * 900 / (2 * m)
    drag = float(B.rho_at(10.0)) * BALL[2] * (math.pi * d * d / 4) / (2 * m) * 900
    assert a[2] + B.G == pytest.approx(cap, rel=1e-6) and a[0] == pytest.approx(
        -drag, rel=1e-6
    )


def test_fly_batch_matches_fly_at_10m():
    rng = np.random.default_rng(1)
    n = 40
    el = rng.uniform(0.0, 0.12, n)
    v0 = rng.uniform(24, 34, n)
    om = rng.uniform(-60, 20, n)
    u = np.stack([np.cos(el), np.zeros(n), np.sin(el)], axis=1)
    worst = 0.0
    # time of flight to x = 10 m from the reference, then the batch integrator at that time
    ts = []
    ref = []
    for k in range(n):
        t, p, _ = B.fly(
            [0, 0, 10.0], v0[k] * u[k], [0, om[k], 0], BALL, stop_x=10.0, dt=0.001
        )
        i = np.searchsorted(p[:, 0], 10.0)
        f = (10 - p[i - 1, 0]) / (p[i, 0] - p[i - 1, 0])
        ts.append(t[i - 1] + f * 0.001)
        ref.append(p[i - 1] + f * (p[i] - p[i - 1]))
    pb = B.fly_batch(
        [0, 0, 10.0],
        v0[:, None] * u,
        np.stack([np.zeros(n), om, np.zeros(n)], axis=1),
        BALL[0],
        BALL[1],
        BALL[2],
        np.array(ts),
        dt=0.008,
    )
    worst = float(np.linalg.norm(pb - np.array(ref), axis=1).max())
    assert worst <= 0.02, worst
    assert (
        worst < 0.005
    )  # the midpoint at 8 ms is far inside the acceptance bound; a regression to Euler would not be


def test_solve_lead2_uses_the_model_spin_and_meets_the_target():
    m = LauncherModel(C)
    M = np.array([0.0, 0.0, 4.0])
    tgt = M + [10.0, 3.0, 1.0]
    s = B.solve_lead2(M, tgt, np.zeros(3), model=m, setpoint=17500, dt=0.001)
    assert s and s["ok"]
    rt, rb = m.split(17500)
    u = s["u"]
    _, p, _ = B.fly(
        M,
        m.mean_v0((rt, rb), with_dip=True) * u,
        B.omega_vec(u, m.spin_rad_s(rt, rb)),
        C,
        dt=0.001,
        t_max=s["t_flight"] + 0.05,
    )
    i = int(round(s["t_flight"] / 0.001))
    assert np.linalg.norm(p[i] - tgt) < 0.05
    # backspin lifts the ball, so the solved elevation is LOWER than the no-spin solution at the same speed
    s0 = B.solve_lead2(
        M, tgt, np.zeros(3), v0=m.mean_v0((rt, rb), with_dip=True), cl_k=0.0, dt=0.001
    )
    assert s["el"] < s0["el"]
