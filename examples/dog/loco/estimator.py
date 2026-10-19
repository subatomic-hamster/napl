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
H_ON, H_OFF = (
    0.025,
    0.05,
)  # m: a foot higher than this above the map cannot have touched down / has lifted off
KP_ACC = 1.0  # rad/s per rad: Mahony k_P, ~1 s tilt correction
KI_ACC, B_MAX = (
    0.2,
    0.05,
)  # rad/s^2 per rad: Mahony k_I (critically damped with k_P 1); rad/s: bias clip (the MC draws N(0, 0.005))
V_GATE = 0.6  # m/s: a stance foot's velocity innovation above this is an outlier (clean trot: < 0.2)
TAU_G = 0.3  # s: averaging of the accelerometer / predicted gravity before the tilt correction (~ one trot period)
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
        self.a_lp, self.g_lp = np.array([0.0, 0.0, G]), np.array([0.0, 0.0, 1.0])
        self.contact = np.ones(4, bool)
        self.hist, self.k = np.zeros((HIST, 3)), 0  # (x, y, yaw) per tick
        self.dq_prev, self.qdd_f = np.zeros(12), np.zeros(12)
        self.bias = np.zeros(3)  # rad/s: estimated gyro bias

    def reset(self, q, p_xy=(0.0, 0.0), yaw=0.0):
        """Start on level feet at world (p_xy, yaw): the SP5 stand-in's first fix (the harness knows where the dog was put)."""
        self.R = rot(np.array([0.0, 0.0, 1.0]), yaw)
        self._settle(q, p_xy)

    def _settle(self, q, p_xy):
        """Trunk height (feet on the map under them at the current attitude), foot states and covariance for the current R."""
        r = np.array(
            [L.fk_jac(q[3 * i : 3 * i + 3])[0] for i, L in enumerate(self.legs)]
        )
        px, py = float(p_xy[0]), float(p_xy[1])
        fw = self.R @ r.T  # feet in the world frame, relative to the trunk origin
        zs = self.map.height(px + fw[0], py + fw[1])
        z = (
            float(np.nanmean(zs)) + self.legs[0].r_foot - float(fw[2].mean())
        )  # feet (ball centres) on the map
        self.x[:] = 0.0
        self.x[0:3] = (px, py, z)
        self.x[6:] = (self.x[:3] + r @ self.R.T).ravel()
        self.P = np.eye(18) * 1e-4

    def update(self, q, dq, tau, gyro, acc, gate=None, hold=False):
        """hold: the body is being accelerated on purpose (velocity command changing): the specific force is not g, so the tilt correction
        pauses and the gyro carries the attitude."""
        dt, R = self.dt, self.R
        w, a = self.m.imu_R @ gyro, self.m.imu_R @ acc
        # gravity correction on the accelerometer and the predicted gravity direction averaged over ~a gait period (the mean specific force
        # over a cycle is g up; the instantaneous one swings by +-0.5 g with the bounce)
        self.a_lp += (a - self.a_lp) * min(1.0, dt / TAU_G)
        self.g_lp += (R.T @ [0.0, 0.0, 1.0] - self.g_lp) * min(1.0, dt / TAU_G)
        na = np.linalg.norm(self.a_lp)
        err = (
            np.cross(self.a_lp / na, self.g_lp / np.linalg.norm(self.g_lp))
            if abs(na - G) < 0.15 * G and not hold
            else np.zeros(3)
        )
        # Mahony PI: the integral of the (valid) gravity error is the gyro bias (roll / pitch axes; yaw has no gravity reference)
        self.bias = np.clip(self.bias - KI_ACC * err * dt, -B_MAX, B_MAX)
        w = w - self.bias
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
        # The leg's own inertia and weight are subtracted (a stance leg commanded J^T f would otherwise read back exactly f): the part of the
        # joint torque that the leg's motion does not explain is the ground force. qdd = low-passed difference of the drivers' dq.
        qdd = (dq - self.dq_prev) / dt if self.k else np.zeros(12)
        self.qdd_f += 0.5 * (qdd - self.qdd_f)
        self.dq_prev = np.array(dq, float)
        fg = np.zeros((4, 3))
        for i in range(4):
            s = slice(3 * i, 3 * i + 3)
            res = (
                self.m.leg_required(i, q[s], self.qdd_f[s], a) - tau[s]
            )  # = J^T F (F: ground force on the foot, trunk frame)
            fg[i] = R @ np.linalg.solve(J[i] @ J[i].T + 1e-6 * np.eye(3), J[i] @ res)
        on = np.where(self.contact, fg[:, 2] > F_OFF, fg[:, 2] > F_ON)
        # a foot cannot touch the ground while it is clearly above it: the swing PD torques of a fast leg look like a ground force
        # (kinematic height of the ball bottom over the map under it; unknown map = no veto)
        feet_w = self.x[0:3] + np.array([R @ r[i] for i in range(4)])
        zg = np.asarray(self.map.height(feet_w[:, 0], feet_w[:, 1]), float)
        h_foot = feet_w[:, 2] - self.legs[0].r_foot - zg
        on &= ~(np.isfinite(h_foot) & (h_foot > np.where(self.contact, H_OFF, H_ON)))
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
            # the ball is rigid with the shank and rolls without slipping: its centre moves at r_foot (omega x z), omega = trunk rate + joint rates
            ws = self.legs[i]._chain(q[3 * i : 3 * i + 3])[2]
            omega = R @ (w + sum(ws[j] * dq[3 * i + j] for j in range(3)))
            roll_v = self.legs[0].r_foot * np.cross(omega, [0.0, 0.0, 1.0])
            yv = (
                -R @ (J[i] @ dq[3 * i : 3 * i + 3] + np.cross(w, r[i]))
                + roll_v
                - self.x[3:6]
            )
            if (
                np.linalg.norm(yv) < V_GATE
            ):  # outlier: a foot flagged in contact that is still moving (landing, lift-off) reads its own speed
                H.append(h); y.append(yv); Rn += [SIG_VEL**2] * 3  # fmt: skip
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
        # feet = forward kinematics on the estimated pose: the controller's lever arms (foot - CoM) must be the kinematic ones at every tick; the
        # KF foot states lag them and carry the swing feet's random walk
        st.feet, st.contact, st.f_ground = (
            st.p + np.array(r) @ self.R.T,
            on.copy(),
            fg,
        )
        # median: a foot whose estimated xy is over a wall top or a boulder (ruins: feet within 0.3 m of a 1.8 m wall) must not read as "the body is
        # on the floor" (mean of four: +0.45 m, h_map < 0.5 h_stand, a FALSE fall: the dog sat down on flat ground, 3 of 8 training ruins seeds)
        st.h_map = st.p[2] - float(
            np.nanmedian(self.map.height(st.feet[:, 0], st.feet[:, 1]))
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
