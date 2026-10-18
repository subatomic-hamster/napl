"""Convex MPC on the single-rigid-body model (Di Carlo, Wensing, Katz, Bledt, Kim, "Dynamic Locomotion in the MIT Cheetah 3 Through Convex
Model-Predictive Control", IROS 2018), solved by OSQP (Apache-2.0) in condensed form (states eliminated; a dense 144 x 144 Hessian and a FIXED
constraint pattern), so every solve is an `update` of values with a warm start. World frame; decision variables = ground reaction forces on the
4 feet over the horizon."""

import numpy as np
import osqp
import scipy.sparse as sp
from scipy.linalg import expm

N, DT = (
    12,
    0.025,
)  # horizon 12 x 25 ms = 0.30 s ~ one trot period; re-solved at 50 Hz (every 10 leg-control ticks)
NX, NU, G = 13, 12, 9.81
# MIT cheetah-software convexMPC weights (Mini Cheetah, 9 kg): rpy, p, w, v, g. Roll / pitch x10 and the force weight /10 for this dog: with the
# MIT values (0.25, 4e-5) standing on four feet is UNSTABLE in the ideal plant (a 2 cm lateral offset of the CoM over the feet grows to a
# fall in 3 s: the force penalty makes redistributing the weight dearer than the small angle error it prevents; Task 6 ladder rung b).
# The roll / pitch RATE weights are 1.0 (MIT: 0): with a 30 ms force lag (legs, bus latency, the MPC's own 20 ms hold) a controller without
# rate damping is unstable at standstill for any roll weight above ~0.3; with 0.3-1.0 it is stable for every combination tried
# (lag 30 ms, 24 weight sets; docs/dog/locomotion.md). The xy velocity weights are 3.0 (MIT 0.2): a velocity error is cheap next to the pitch
# moment a forward force at a foot 0.43 m below the CoM makes, and the trot settled 7-10 % under the command (0.90 at 1.0 m/s); at 3.0 the
# truth-state trot holds 0.93-1.0 m/s.
Q_DIAG = np.array([2.5, 2.5, 10.0, 2.0, 2.0, 50.0, 1.0, 1.0, 0.3, 3.0, 3.0, 0.1, 0.0])
ALPHA = 4e-6
EPS = 1e-6  # OSQP absolute / relative tolerance (1e-4 let the 15 deg ramp fall: the closed loop is sensitive to the force accuracy; 1e-6 costs 6.0 ms vs 4.2 ms median)
F_MIN, F_MAX = (
    2.0,
    300.0,
)  # N per stance foot: keep it loaded; above the 257 N study peak (docs/dog/actuator-study.md candidate 1)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def rz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pyramid(mu, n=None):
    """(5, 3) rows of the friction pyramid about the unit ground normal n (default world up) on a world-frame foot force f: four rows
    `row . f <= 0` (|f . t1|, |f . t2| <= mu f . n) and the normal-force row `n . f` (bounded F_MIN..F_MAX by the callers)."""
    if n is None or not np.any(n[:2]):
        return np.array(
            [[1, 0, -mu], [-1, 0, -mu], [0, 1, -mu], [0, -1, -mu], [0, 0, 1]], float
        )
    t1 = np.array([1.0, 0.0, 0.0]) - n[0] * n
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    return np.array([t1 - mu * n, -t1 - mu * n, t2 - mu * n, -t2 - mu * n, n])


class Mpc:
    def __init__(self, mass, inertia_body):
        self.m, self.Ib = float(mass), np.asarray(inertia_body, float)
        # Condensed form: x = Sx x0 + Su u, decision variables = the forces u only (N*12 = 144), dense Hessian, 5 friction rows per
        # (step, foot). With the state equalities kept as constraints (the first version, 300 variables) ADMM needed 300-850 iterations on
        # the test problems and the stress-test median was 9 ms; condensed it needs about 75 and the forces agree with a 1e-7 solve to 0.03 N.
        self.nu, self.nc = N * NU, N * 20
        r, c = [], []
        for k in range(N):
            for i in range(4):
                rr, cc = np.meshgrid(np.arange(5), np.arange(3), indexing="ij")
                r.append((k * 20 + 5 * i + rr).ravel())
                c.append((k * NU + 3 * i + cc).ravel())
        A0 = sp.coo_matrix(
            (
                np.arange(1, sum(map(len, r)) + 1, dtype=float),
                (np.concatenate(r), np.concatenate(c)),
            ),
            shape=(self.nc, self.nu),
        ).tocsc()
        self.perm = (A0.data - 1).astype(int)  # CSC data slot -> pattern entry
        self.A = A0
        cols, rows = np.nonzero(
            np.triu(np.ones((self.nu, self.nu), bool)).T
        )  # upper triangle, column-major = CSC order
        self.p_idx = (rows, cols)
        self.P = sp.csc_matrix(
            (np.ones(len(rows)), (rows, cols)), shape=(self.nu, self.nu)
        )
        self.qb = np.tile(Q_DIAG, N)
        self.prob, self.n_fail = None, 0

    def _ab(self, yaw, r):
        Rz = rz(yaw)
        Iw_inv = np.linalg.inv(Rz @ self.Ib @ Rz.T)
        A = np.zeros((NX, NX))
        A[0:3, 6:9], A[3:6, 9:12], A[11, 12] = Rz.T, np.eye(3), 1.0
        B = np.zeros((NX, NU))
        for i in range(4):
            B[6:9, 3 * i : 3 * i + 3] = Iw_inv @ skew(r[i])
            B[9:12, 3 * i : 3 * i + 3] = np.eye(3) / self.m
        return A, B

    def solve(self, x0, xref, r_feet, contact, mu, normals=None):
        """normals (4, 3): unit ground normal per foot (the friction cones' axes); None = world up."""
        pyr = [
            pyramid(mu[i], None if normals is None else normals[i]) for i in range(4)
        ]
        Ads, Bds = [], []
        for k in range(N):
            A, B = self._ab(xref[k, 2], r_feet[k])
            Md = expm(
                np.block([[A, B], [np.zeros((NU, NX + NU))]]) * DT
            )  # zero-order hold
            Ads.append(Md[:NX, :NX])
            Bds.append(Md[:NX, NX:])
        # x_{k+1} = Ad_k x_k + Bd_k u_k  ->  X = Sx x0 + Su U (block lower triangular Su)
        Su, Sx, P = np.zeros((N * NX, N * NU)), np.zeros((N * NX, NX)), np.eye(NX)
        for i in range(N):
            P = Ads[i] @ P
            Sx[i * NX : (i + 1) * NX] = P
        for j in range(N):
            M = Bds[j]
            for i in range(j, N):
                Su[i * NX : (i + 1) * NX, j * NU : (j + 1) * NU] = M
                if i + 1 < N:
                    M = Ads[i + 1] @ M
        Ax = np.concatenate([pyr[i].ravel() for _ in range(N) for i in range(4)])[
            self.perm
        ]
        l, u = np.full(self.nc, -np.inf), np.zeros(self.nc)
        fr = np.arange(self.nc).reshape(N, 4, 5)
        l[fr[..., 4]], u[fr[..., 4]] = (
            F_MIN * contact,
            F_MAX * contact,
        )  # normal force n . f; the four pyramid rows are <= 0
        # force penalty about the static weight share (not about zero): the reference state is then an exact equilibrium, with a plain
        # alpha |u|^2 the optimum sags toward the end of the horizon and the first-step force at the reference is 11 % over the weight
        ns = np.maximum(contact.sum(axis=1), 1.0)
        u_ff = np.zeros((N, 4, 3))
        u_ff[:, :, 2] = contact * (self.m * G / ns)[:, None]
        H = Su.T @ (self.qb[:, None] * Su)
        H[np.diag_indices_from(H)] += ALPHA
        q = Su.T @ (self.qb * (Sx @ x0 - xref.ravel())) - ALPHA * u_ff.ravel()
        Px = H[self.p_idx]
        if self.prob is None:
            self.A.data, self.P.data = Ax, Px
            self.prob = osqp.OSQP()
            self.prob.setup(self.P, q, self.A, l, u, warm_start=True, verbose=False, polish=False,
                            eps_abs=EPS, eps_rel=EPS, max_iter=4000)  # fmt: skip
        else:
            self.prob.update(Px=Px, q=q, l=l, u=u, Ax=Ax)
        res = self.prob.solve()
        if res.info.status not in ("solved", "solved inaccurate"):
            self.n_fail += 1
            f = np.zeros((4, 3))
            st = contact[0] > 0
            f[st, 2] = (
                self.m * G / max(1, st.sum())
            )  # static share: keep standing through a failed solve
            return f, False
        f = res.x[:NU].reshape(4, 3)
        # ADMM stops at a residual of ~1e-4 x |f|: project the first step onto the exact bounds (normal force, friction pyramid); a swing foot pushes nothing
        for i in range(4):
            if contact[0, i] <= 0:
                f[i] = 0.0
                continue
            t1, t2, n = (
                (pyr[i][0] - pyr[i][1]) / 2,
                (pyr[i][2] - pyr[i][3]) / 2,
                pyr[i][4],
            )
            fn = float(np.clip(f[i] @ n, F_MIN, F_MAX))
            f[i] = (
                fn * n
                + np.clip(f[i] @ t1, -mu[i] * fn, mu[i] * fn) * t1
                + np.clip(f[i] @ t2, -mu[i] * fn, mu[i] * fn) * t2
            )
        return f, True
