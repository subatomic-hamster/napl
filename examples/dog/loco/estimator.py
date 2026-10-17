"""State estimator of the dog (SP4), 500 Hz. Inputs: the drivers and the IMU (loco.hw) and two SP5 stand-ins (height map, pose fix).
1. Attitude: Mahony complementary filter (Mahony, Hamel, Pflimlin 2008) on gyro + accelerometer; the gravity correction is used only while
   |a| is within 15 % of g (not in flight phases). Yaw is free (no magnetometer): only a pose fix corrects it.
2. Linear Kalman filter on x = [p, v, p_foot x 4] (18, world frame; Bledt, Powell, Katz, Di Carlo, Wensing, Kim, IROS 2018, MIT Cheetah 3):
   process v += (R a + g) dt, stance feet still, swing feet free; measurements per foot in contact: p_foot - p = R r_i(q) and
   v = -R (J_i dq + w x r_i), foot height = map height under it; pose fix p_xy (with its age, against the estimate of that time).
3. Contact: ground force from the drivers' own torque estimate f_i = -R (J_i^T)^-1 tau_i (leg dynamics neglected: ~1.5 kg legs swing at
   < 15 N equivalent); contact when f_z > F_ON, released below F_OFF; `gate` forces no-contact early in a scheduled swing."""

import math

import numpy as np

G = 9.81
# N: contact on above F_ON, off below F_OFF. The static per-foot load is 28.6 N (11.65 kg), so the virtual FootContact's 30 / 15 N would
# call a standing dog airborne; 20 / 10 N sits between the swing-leg inertial equivalent (< ~10 N) and the standing load, and below any
# trot stance force (> 50 N).
F_ON, F_OFF = 20.0, 10.0
KP_ACC = 1.0  # rad/s per rad: Mahony k_P, ~1 s tilt correction
SIG_A, SIG_FOOT_ST, SIG_FOOT_SW = (
    0.3,
    0.002,
    1.0,
)  # process: accel (m/s^2), stance / swing foot random walk (m/sqrt(s))
SIG_KIN, SIG_VEL, SIG_MAPZ = (
    0.005,
    0.05,
    0.02,
)  # measurement: leg position (m), leg velocity (m/s), map height under a stance foot (m)
HIST = 64  # estimate history for pose fixes up to 0.128 s old


def rpy_of(R):
    return np.array(
        [
            math.atan2(R[2, 1], R[2, 2]),
            -math.asin(max(-1.0, min(1.0, R[2, 0]))),
            math.atan2(R[1, 0], R[0, 0]),
        ]
    )


def rot(w, th):
    n = np.linalg.norm(w)
    if n * th < 1e-12:
        return np.eye(3)
    k = w / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    a = n * th
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


class State:
    __slots__ = ("R", "contact", "f_ground", "feet", "h_map", "p", "rpy", "v", "w")


class Estimator:
    def __init__(self, model, hmap, dt=0.002):
        self.m, self.map, self.dt = model, hmap, dt
        self.legs = list(model.legs.values())
        self.R, self.x, self.P = np.eye(3), np.zeros(18), np.eye(18) * 1e-4
        self.contact = np.ones(4, bool)
        self.hist, self.k = np.zeros((HIST, 3)), 0  # (x, y, yaw) per tick

    def reset(self, q):
        r = np.array(
            [L.fk_jac(q[3 * i : 3 * i + 3])[0] for i, L in enumerate(self.legs)]
        )
        rf = self.legs[0].r_foot
        z = (
            float(np.nanmean(self.map.height(r[:, 0], r[:, 1]))) + rf - r[:, 2].mean()
        )  # feet (ball centres) on the map
        self.x[:] = 0.0
        self.x[2] = z
        self.x[6:] = (r + self.x[:3]).ravel()
        self.P = np.eye(18) * 1e-4

    def update(self, q, dq, tau, gyro, acc, gate=None):
        dt, R = self.dt, self.R
        w, a = self.m.imu_R @ gyro, self.m.imu_R @ acc
        na = np.linalg.norm(a)
        err = (
            np.cross(a / na, R.T @ [0, 0, 1]) if abs(na - G) < 0.15 * G else np.zeros(3)
        )
        self.R = R = R @ rot(w + KP_ACC * err, dt)
        if self.k % 100 == 0:
            u, _, vt = np.linalg.svd(R)
            self.R = R = u @ vt
        # ---- KF predict
        F = np.eye(18)
        F[0:3, 3:6] = np.eye(3) * dt
        self.x = F @ self.x
        self.x[3:6] += (R @ a - [0, 0, G]) * dt
        Qd = np.zeros(18)
        Qd[3:6] = (SIG_A * dt) ** 2
        Qd[0:3] = (SIG_A * dt * dt) ** 2
        # ---- contact from the drivers' torque
        r, J = zip(*(L.fk_jac(q[3 * i : 3 * i + 3]) for i, L in enumerate(self.legs)))
        # f_leg = -(J^T)^+ tau, damped: exact away from singularities, finite at the straight (build) pose where J loses rank
        fg = np.array(
            [
                -R
                @ np.linalg.solve(
                    J[i] @ J[i].T + 1e-6 * np.eye(3), J[i] @ tau[3 * i : 3 * i + 3]
                )
                for i in range(4)
            ]
        )
        on = np.where(self.contact, fg[:, 2] > F_OFF, fg[:, 2] > F_ON)
        if gate is not None:
            on &= ~np.asarray(gate, bool)
        self.contact = on
        for i in range(4):
            Qd[6 + 3 * i : 9 + 3 * i] = (
                SIG_FOOT_ST if on[i] else SIG_FOOT_SW
            ) ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag(Qd)
        # ---- KF update: stacked rows of the feet in contact
        H, y, Rn = [], [], []
        for i in np.flatnonzero(on):
            s = slice(6 + 3 * i, 9 + 3 * i)
            h = np.zeros((3, 18))
            h[:, s], h[:, 0:3] = np.eye(3), -np.eye(3)
            H.append(h); y.append(R @ r[i] - (self.x[s] - self.x[0:3])); Rn += [SIG_KIN**2] * 3  # fmt: skip
            h = np.zeros((3, 18))
            h[:, 3:6] = np.eye(3)
            H.append(h); y.append(-R @ (J[i] @ dq[3 * i : 3 * i + 3] + np.cross(w, r[i])) - self.x[3:6]); Rn += [SIG_VEL**2] * 3  # fmt: skip
            zf = (
                float(self.map.height(self.x[s][0], self.x[s][1])) + self.legs[0].r_foot
            )
            if np.isfinite(zf):
                h = np.zeros((1, 18))
                h[0, s.start + 2] = 1.0
                H.append(h); y.append([zf - self.x[s.start + 2]]); Rn.append(SIG_MAPZ**2)  # fmt: skip
        if H:
            self._correct(np.vstack(H), np.concatenate(y), np.array(Rn))
        self.hist[self.k % HIST] = (self.x[0], self.x[1], rpy_of(self.R)[2])
        self.k += 1
        st = State()
        st.p, st.v, st.R, st.rpy, st.w = (
            self.x[0:3].copy(),
            self.x[3:6].copy(),
            self.R.copy(),
            rpy_of(self.R),
            w,
        )
        st.feet, st.contact, st.f_ground = (
            self.x[6:].reshape(4, 3).copy(),
            on.copy(),
            fg,
        )
        st.h_map = st.p[2] - float(
            np.nanmean(self.map.height(st.feet[:, 0], st.feet[:, 1]))
        )
        return st

    def _correct(self, H, y, rn):
        S = H @ self.P @ H.T + np.diag(rn)
        K = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + K @ y
        self.P = (np.eye(18) - K @ H) @ self.P

    def fix(self, p_xy, yaw, age_s, sig_p, sig_yaw):
        """Pose fix (SP5 stand-in, 10 Hz): compared with this estimator's own pose `age_s` ago; the innovation is applied now (xy through the
        KF, yaw as a rotation about world z weighted by the variances)."""
        lag = min(HIST - 1, int(round(age_s / self.dt)))
        x0, y0, yaw0 = self.hist[(self.k - 1 - lag) % HIST]
        h = np.zeros((2, 18))
        h[0, 0], h[1, 1] = 1.0, 1.0
        self._correct(h, np.array([p_xy[0] - x0, p_xy[1] - y0]), np.full(2, sig_p**2))
        dyaw = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi
        self.R = (
            rot(np.array([0.0, 0.0, 1.0]), 0.5 * dyaw) @ self.R
        )  # ponytail: fixed 0.5 blend, a yaw state in the KF if SP5 needs better
