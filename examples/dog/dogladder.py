"""Diagnostic ladder of the dog's locomotion (SP4 Task 6): the controller with GROUND-TRUTH state (the estimator is out of the loop), so each
rung isolates one layer. Harness only (reads the truth), never imported by examples/dog/loco."""

import numpy as np
from loco import estimator as E
from loco import model as M

LEG_NAMES = ("fl", "fr", "rl", "rr")


def truth_state(c):
    """Replace c.ctl's estimator output by the truth (pose, velocity, attitude, feet from FK on the true pose); contact, f_ground and the
    rest stay the estimator's."""
    est, ctl = c.ctl.est, c.ctl
    orig = est.update

    def upd(q, dq, tau, gyro, acc, gate=None):
        st = orig(q, dq, tau, gyro, acc, gate)
        R = M.quat_R(c.tr.quat())
        st.R, st.p, st.v = R, c.tr.pos(), c.tr.vel()
        st.rpy = E.rpy_of(R)
        st.w = ctl.m.imu_R @ gyro
        feet = np.array([ctl.legs[i].fk_jac(q[3 * i : 3 * i + 3])[0] for i in range(4)])
        st.feet = st.p + feet @ R.T
        return st

    est.update = upd
    return c


def com_accel_force(c, dt_ticks=10):
    """-> callable() giving m (a_com + g) in the world frame, from the true CoM velocity over the last `dt_ticks` calls (call once per tick)."""
    m = c.ctl.m
    hist = []

    def v_com():
        w = np.array(
            c.sim.query("/world/deck_top/deck_top:free", "qvel")[0][3:6], float
        )
        return c.tr.vel() + M.quat_R(c.tr.quat()) @ np.cross(w, m.com)

    def f():
        hist.append(v_com())
        if len(hist) <= dt_ticks:
            return np.full(3, np.nan)
        a = (hist[-1] - hist[-1 - dt_ticks]) / (dt_ticks * 0.002)
        return m.mass * (a + np.array([0.0, 0.0, 9.81]))

    return f


def saturation(c):
    """Fraction-of-limit of the delivered joint torques: -> (12,) |tau_est| / tau_max."""
    q, dq, tau = c.hw.joints()
    lim = np.concatenate([L.tau_max for L in c.ctl.legs])
    return np.abs(tau) / lim
