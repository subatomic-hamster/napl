"""Diagnostic ladder of the dog's locomotion (SP4 Task 6): the controller with GROUND-TRUTH state (the estimator is out of the loop), so each
rung isolates one layer. Harness only (reads the truth), never imported by examples/dog/loco."""

import numpy as np
from loco import estimator as E
from loco import model as M

LEG_NAMES = ("fl", "fr", "rl", "rr")


ALL = ("R", "p", "v")


def truth_state(c, parts=ALL):
    """Replace c.ctl's estimator output by the truth, per part: "R" attitude (R, rpy), "p" position, "v" velocity; the feet are FK on the
    resulting pose. Contact, f_ground and the rest stay the estimator's (the contact estimate is NOT replaced). Default: all three."""
    est, ctl = c.ctl.est, c.ctl
    orig = est.update

    def upd(q, dq, tau, gyro, acc, gate=None, hold=False):
        st = orig(q, dq, tau, gyro, acc, gate, hold)
        if "R" in parts:
            st.R = M.quat_R(c.tr.quat())
            st.rpy = E.rpy_of(st.R)
        if "p" in parts:
            st.p = c.tr.pos()
        if "v" in parts:
            st.v = c.tr.vel()
        st.w = ctl.m.imu_R @ gyro
        if (
            "contact" in parts
        ):  # the virtual FootContact's truth (30 / 15 N, 4 ms debounce), not the torque-based estimate
            st.contact = np.asarray(c.tr.contacts(), float) > 0.5
        if "R" in parts or "p" in parts:
            feet = np.array(
                [ctl.legs[i].fk_jac(q[3 * i : 3 * i + 3])[0] for i in range(4)]
            )
            st.feet = st.p + feet @ st.R.T
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
