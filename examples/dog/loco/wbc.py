"""Whole-body controller of the dog (SP4), 500 Hz: WBIC (Kim, Jorgensen, Lee, Kim, "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse
Control and Model Predictive Control", arXiv:1909.06586, 2019; the MIT Mini Cheetah structure) on the controller's own model (loco.model).
1. Kinematic tasks, strict priorities by null-space projection: stance feet still -> body orientation -> body position -> swing feet. Gives
   joint position / velocity targets and the commanded generalized acceleration qdd (A^-1-weighted pseudo-inverses for qdd).
2. QP (OSQP, fixed dense pattern, warm start): z = [dqdd_fb (6), df (12)], min W_FB |dqdd_fb|^2 + W_F |df|^2, s.t.
     floating-base rows  A_fb (qdd + [dqdd_fb, 0]) + b_fb + g_fb = J_c,fb' (f_ref + df)
     f_ref + df inside the friction pyramid with fz in [F_MIN, F_MAX] on stance feet, 0 on swing feet (the rows of loco.mpc)
     joint torques       A_j (qdd + [dqdd_fb, 0]) + b_j + g_j - J_c,j' (f_ref + df) within +-tau_max
   f_ref = the MPC forces on the feet the WBC treats as stance. tau_ff = those joint torques.
Generalized velocity u = [v (trunk origin, world), w (world), dq (12)]; equations A u' + b + g = S' tau + J_c' f."""

import sys
from collections import namedtuple
from pathlib import Path

import numpy as np
import osqp
import scipy.sparse as sp

from . import mpc as Mp

BUDGET_S = 0.001  # median per 2 ms control tick: half the tick, the rest is I/O + estimator + swing references (Task 1 / 3 numbers)
KP_BODY, KD_BODY = (
    100.0,
    20.0,
)  # 1/s^2, 1/s: body position task (critically damped, ~1.6 Hz; Mini Cheetah WBIC class)
KP_ORI, KD_ORI = 100.0, 20.0  # same for orientation
KP_FOOT, KD_FOOT = 400.0, 40.0  # swing foot task (~3.2 Hz: well inside a 0.1 s swing)
W_FB, W_F = (
    10.0,
    1.0,
)  # QP weights: floating-base acceleration relaxation vs reaction-force change (Kim 2019 Q2 / Q1 ratio)
LAM, LAM_DYN = (
    1e-4,
    1e-6,
)  # damping of the kinematic (m^2/rad^2) and dynamically consistent (m^2 / (kg)) pseudo-inverses
V_MAX = 40.0  # rad/s: as loco.controller.V_MAX
STEP_MAX = 0.3  # rad: largest joint step toward the task solution per tick (as the J^T path's IK)
EPS = 1e-5  # s: finite-difference step for J-dot u and the Coriolis term
WbcOut = namedtuple("WbcOut", "p_des v_des tau f ok")


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _rot(w, th):
    n = np.linalg.norm(w) * th
    if n < 1e-12:
        return np.eye(3)
    K = skew(w / np.linalg.norm(w))
    return np.eye(3) + np.sin(n) * K + (1 - np.cos(n)) * K @ K


def _elements(m, R, q):
    """Per link: (mass, Jv (3,18) of its com, Jw (3,18), inertia) in the world frame; per foot: Jc (3,18) of the ball centre."""
    legs, els, Jc = list(m.legs.values()), [], np.zeros((4, 3, 18))
    for i, L in enumerate(legs):
        r, J = L.fk_jac(q[3 * i : 3 * i + 3])
        Jc[i, :, 0:3], Jc[i, :, 3:6], Jc[i, :, 6 + 3 * i : 9 + 3 * i] = (
            np.eye(3),
            -skew(R @ r),
            R @ J,
        )
    for lk in m.links:
        Jv, Jw = np.zeros((3, 18)), np.zeros((3, 18))
        Jv[:, 0:3], Jw[:, 3:6] = np.eye(3), np.eye(3)
        if lk.leg < 0:
            x, Rk = lk.com, np.eye(3)
        else:
            Rk, t, ws, ps = legs[lk.leg]._chain(q[3 * lk.leg : 3 * lk.leg + 3], lk.upto)
            x = Rk @ lk.com + t
            for j in range(lk.upto):
                Jv[:, 6 + 3 * lk.leg + j] = R @ np.cross(ws[j], x - ps[j])
                Jw[:, 6 + 3 * lk.leg + j] = R @ ws[j]
        Jv[:, 3:6] = -skew(R @ x)
        Rw = R @ Rk
        els.append((lk.mass, Jv, Jw, Rw @ lk.inertia @ Rw.T))
    return els, Jc


def _advance(R, q, u, eps):
    return _rot(u[3:6], eps) @ R, q + eps * u[6:]


def dynamics(m, R, q, u):
    els, _ = _elements(m, R, q)
    A, g = np.diag(np.r_[np.zeros(6), m.armature]), np.zeros(18)
    for mass, Jv, Jw, I in els:
        A += mass * Jv.T @ Jv + Jw.T @ I @ Jw
        g += mass * Mp.G * Jv[2]
    b = np.zeros(18)
    if np.any(
        u
    ):  # ponytail: J-dot u by finite difference (one extra kinematics pass); analytic RNEA if the profile shows it
        els2, _ = _elements(m, *_advance(R, q, u, EPS))
        for (mass, Jv, Jw, I), (_, Jv2, Jw2, _) in zip(els, els2):
            wk = Jw @ u
            b += mass * Jv.T @ ((Jv2 - Jv) @ u / EPS) + Jw.T @ (
                I @ ((Jw2 - Jw) @ u / EPS) + np.cross(wk, I @ wk)
            )
    return A, b, g


def _pinv(J):
    """Damped right pseudo-inverse: finite at the straight-leg singularity (an undamped one sends a swing joint to +-100 rad)."""
    return J.T @ np.linalg.inv(J @ J.T + LAM * np.eye(J.shape[0]))


def _pinv_dyn(J, Ainv):
    return Ainv @ J.T @ np.linalg.inv(J @ Ainv @ J.T + LAM_DYN * np.eye(J.shape[0]))


def _log(Rerr):
    c = np.clip((np.trace(Rerr) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    v = np.array(
        [Rerr[2, 1] - Rerr[1, 2], Rerr[0, 2] - Rerr[2, 0], Rerr[1, 0] - Rerr[0, 1]]
    )
    return v / 2 if th < 1e-6 else v * th / (2 * np.sin(th))


class Wbc:
    def __init__(self, model):
        self.m = model
        self.tau_max = np.concatenate([L.tau_max for L in model.legs.values()])
        self.nc = 6 + 20 + 12
        self.P = sp.diags(np.r_[np.full(6, W_FB), np.full(12, W_F)]).tocsc()
        self.A_qp = sp.csc_matrix(
            np.ones((self.nc, 18))
        )  # dense pattern, explicit entries everywhere (values set per solve)
        self.prob, self.n_fail, self.n_relaxed, self.status = None, 0, 0, ""

    def solve(
        self, p, R, w, v, q, dq, f_mpc, stance, mu, body_ref, swing, normals=None
    ):
        """normals (4, 3): unit ground normal per foot, the friction pyramids' axes (None = world up)."""
        u = np.r_[v, w, dq]
        A, b, g = dynamics(self.m, R, q, u)
        Ainv = np.linalg.inv(A)
        _, Jc = _elements(self.m, R, q)
        _, Jc2 = _elements(self.m, *_advance(R, q, u, EPS))
        Jcd_u = (Jc2 - Jc) @ u / EPS  # (4, 3)
        n18 = np.eye(18)
        # ---- 1. kinematic tasks, strict priorities
        st = np.flatnonzero(stance)
        if len(st):
            Jst = Jc[st].reshape(-1, 18)
            qdd = _pinv_dyn(Jst, Ainv) @ (-Jcd_u[st].ravel())
            N = n18 - _pinv(Jst) @ Jst
        else:
            qdd, N = np.zeros(18), n18.copy()
        p_ref, v_ref, R_ref = body_ref
        Jo, Jp = np.zeros((3, 18)), np.zeros((3, 18))
        Jo[:, 3:6], Jp[:, 0:3] = np.eye(3), np.eye(3)
        tasks = [(Jo, np.zeros(3), _log(R_ref @ R.T), np.zeros(3), KP_ORI, KD_ORI),
                 (Jp, np.zeros(3), p_ref - p, v_ref, KP_BODY, KD_BODY)]  # fmt: skip
        legs = list(self.m.legs.values())
        for i in np.flatnonzero(~stance):
            x = p + R @ legs[i].fk_jac(q[3 * i : 3 * i + 3])[0]
            tasks.append(
                (Jc[i], Jcd_u[i], swing[0][i] - x, swing[1][i], KP_FOOT, KD_FOOT)
            )
        dpos, dvel = np.zeros(18), np.zeros(18)
        for J, Jd_u, e, vd, kp, kd in tasks:
            JN = J @ N
            JN_pinv = _pinv(JN)
            dpos = dpos + JN_pinv @ (e - J @ dpos)
            dvel = dvel + JN_pinv @ (vd - J @ dvel)
            qdd = qdd + _pinv_dyn(JN, Ainv) @ (
                kp * e + kd * (vd - J @ u) - Jd_u - J @ qdd
            )
            N = N @ (n18 - JN_pinv @ JN)
        # ---- 2. dynamics QP
        f_ref = np.where(stance[:, None], f_mpc, 0.0).ravel()
        JcT = Jc.reshape(12, 18).T  # (18, 12)
        h = A @ qdd + b + g - JcT @ f_ref
        M = np.zeros((self.nc, 18))
        l, up = np.zeros(self.nc), np.zeros(self.nc)
        M[0:6, 0:6], M[0:6, 6:] = A[0:6, 0:6], -JcT[0:6]
        l[0:6] = up[0:6] = -h[0:6]
        for i in range(4):
            rows = Mp.pyramid(mu[i], None if normals is None else normals[i])
            r0 = 6 + 5 * i
            M[r0 : r0 + 5, 6 + 3 * i : 9 + 3 * i] = rows
            fr = f_ref[3 * i : 3 * i + 3]
            fn = rows[4] @ fr  # normal force of the reference
            l[r0 : r0 + 4], up[r0 : r0 + 4] = -np.inf, -(rows[:4] @ fr)
            l[r0 + 4], up[r0 + 4] = (
                Mp.F_MIN * stance[i] - fn,
                Mp.F_MAX * stance[i] - fn,
            )
        M[26:, 0:6], M[26:, 6:] = A[6:, 0:6], -JcT[6:]
        # torque limits bind the STANCE legs' joints (the ground forces can move them); a swing joint's torque is set by the swing task and no
        # force changes it much: a hard bound there makes the QP infeasible when the swing foot is asked for more than the drivers give (the
        # 25 deg ramp), so a swing joint is only clipped to its limit after the solve (the swing task then lags, as a saturated driver would)
        free = np.repeat(~stance, 3)
        l[26:] = np.where(free, -np.inf, -self.tau_max - h[6:])
        up[26:] = np.where(free, np.inf, self.tau_max - h[6:])
        Ax = M.ravel(order="F")  # CSC of a dense pattern = column-major values
        if self.prob is None:
            self.A_qp.data = Ax
            self.prob = osqp.OSQP()
            self.prob.setup(self.P, np.zeros(18), self.A_qp, l, up, warm_start=True, verbose=False, polish=True,
                            eps_abs=1e-5, eps_rel=1e-5, max_iter=2000)  # fmt: skip
        else:
            self.prob.update(Ax=Ax, l=l, u=up)
        res = self.prob.solve()
        self.status = res.info.status
        if res.info.status not in ("solved", "solved inaccurate"):
            # the friction pyramids and the drivers' torque limits cannot both hold (a stance leg that must carry its own weight while the MPC
            # leaves its foot nearly unloaded, a leg at full extension): keep the pyramids, drop the torque rows and let the clip below saturate
            # the torque like the driver does (the realized force is then below the command; counted in n_relaxed)
            l[26:], up[26:] = -np.inf, np.inf
            self.prob.update(l=l, u=up)
            res = self.prob.solve()
            self.status = res.info.status
            self.n_relaxed += 1
        if res.info.status not in ("solved", "solved inaccurate"):
            self.n_fail += 1
            return WbcOut(
                np.clip(q + dpos[6:], q - STEP_MAX, q + STEP_MAX),
                np.clip(dvel[6:], -V_MAX, V_MAX),
                np.zeros(12),
                f_mpc,
                False,
            )
        z = res.x
        f = f_ref + z[6:]
        self.tau_raw = h[6:] + A[6:, 0:6] @ z[0:6] - JcT[6:] @ z[6:]
        tau = np.clip(
            self.tau_raw, -self.tau_max, self.tau_max
        )  # stance joints: solver tolerance only; swing joints: the limit of the driver
        return WbcOut(
            np.clip(q + dpos[6:], q - STEP_MAX, q + STEP_MAX),
            np.clip(dvel[6:], -V_MAX, V_MAX),
            tau,
            f.reshape(4, 3),
            True,
        )


def make(model, cpp=True):
    """The C++ port (examples/dog/loco/cpp, cmake -DNAPL_LOCO_CPP=ON -> build/loco) when it was built, else the Python class."""
    if cpp:
        out = Path(__file__).resolve().parents[3] / "build" / "loco"
        if out.is_dir() and str(out) not in sys.path:
            sys.path.append(str(out))
        try:
            from loco_wbc_cpp import Wbc as CppWbc

            return CppWbc(model)
        except ImportError:
            pass
    return Wbc(model)
