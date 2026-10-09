"""State estimation for the Skydio-style quadrotor: an error-state Kalman filter (ESKF) on IMU + visual odometry + barometer. numpy only.

    eskf = Eskf(p0, v0, R0, sigma_gyro, sigma_accel)
    eskf.predict(gyro, accel, dt)                    # every IMU sample (500 Hz): strapdown integration + covariance propagation
    eskf.update_vio(t_sample, v_body, R_meas, ...)   # every VIO frame (25 Hz, delivered late): body-frame velocity + attitude
    eskf.update_height(z)                            # barometer (50 Hz)
    eskf.p, eskf.v, eskf.R, eskf.sigmas()            # the estimate the controller, the map and the planner use

State: position p, velocity v (world), attitude R (body -> world), accelerometer bias ba, gyro bias bg (15 error states). The world frame is the
odometry frame: it starts at the known take-off pose, there is no GNSS and no absolute position fix, so p drifts with the visual odometry; the
covariance grows accordingly. Height is held by the barometer.

THE SENSORS (sensor models: what the filter is fed; the truth never reaches it)
  IMU: the `imu` device of assemblies/skydio.toml (ICM-42688-P noise densities and ADC/range from the part file, 500 Hz) plus `ImuBias`: a turn-on
       bias drawn within the part's initial zero-rate / zero-g offsets (+-8.7e-3 rad/s, +-0.196 m/s^2) and a bias random walk (estimate, below).
  VIO: `VioSim` is a STATISTICAL stand-in for a stereo/depth visual-inertial front end (no feature tracking is simulated): body-frame velocity
       from frame-to-frame increments with a scale error, a slowly varying (Gauss-Markov) bias and white noise, and a gravity-aligned attitude with
       white + Gauss-Markov roll/pitch error and a yaw that random-walks (heading drift). Calibrated so that the dead-reckoned path error is
       about 1 % of the distance travelled, the drift of Intel's T265 tracking camera ("under 1 % closed loop drift", Intel support article 000032899)
       and the order of the 0.05-0.3 m ATE of stereo-inertial OpenVINS / VINS-Fusion over 36-130 m EuRoC runs (table compiled in arXiv:2511.18756).
  Barometer: the `baro` device (MS5611, 50 Hz) through the ISA formula; the simulated air has no weather, no prop-wash pressure error and no drift.
Filter tuning is deliberately a little pessimistic (measurement sigmas above the simulated ones, process noise of the biases above the simulated walk).
Delayed measurements: a VIO frame is applied to the CURRENT state with the innovation computed against the state history at the sample time
(the usual innovation-lag approximation; no re-propagation).
"""

from collections import deque

import numpy as np

G = 9.80665


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp_so3(w):
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        return np.eye(3) + skew(w)
    K = skew(w / th)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * K @ K


def log_so3(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = float(np.arccos(c))
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-6:
        return v / 2.0
    return v * th / (2.0 * np.sin(th))


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class ImuBias:
    """Bias the IMU device does not model (it only has white noise, quantisation and range): turn-on offset + random walk, added to its output."""

    GYRO_ZRO, ACCEL_ZERO_G = (
        8.727e-3,
        0.19613,
    )  # rad/s, m/s^2: ICM-42688-P initial offsets (datasheet, parts/drone/tdk-icm-42688-p.toml)
    GYRO_RW, ACCEL_RW = (
        2e-5,
        2e-4,
    )  # rad/s/sqrt(s), m/s^2/sqrt(s): estimates (the part publishes no in-run stability here)

    def __init__(self, rng):
        self.rng = rng
        self.bg = rng.uniform(-self.GYRO_ZRO, self.GYRO_ZRO, 3)
        self.ba = rng.uniform(-self.ACCEL_ZERO_G, self.ACCEL_ZERO_G, 3)

    def apply(self, gyro, accel, dt):
        self.bg = self.bg + self.GYRO_RW * np.sqrt(dt) * self.rng.standard_normal(3)
        self.ba = self.ba + self.ACCEL_RW * np.sqrt(dt) * self.rng.standard_normal(3)
        return np.asarray(gyro) + self.bg, np.asarray(accel) + self.ba


class VioSim:
    """Statistical visual-odometry output at the camera frame rate. `sample` -> (body-frame velocity, attitude R in the odometry frame)."""

    def __init__(self, rng, scale_sigma=0.004, v_gm=(0.02, 8.0), v_white=(0.02, 0.01), rp_white_deg=0.2, rp_gm_deg=(0.15, 10.0),
                 yaw_white_deg=0.3, yaw_rw_deg=0.05):  # fmt: skip
        self.rng = rng
        self.scale = (
            scale_sigma * rng.standard_normal()
        )  # one scale error per run (baseline / intrinsics calibration)
        self.v_gm, self.v_white = (
            v_gm,
            v_white,
        )  # (sigma m/s, tau s); (sigma0 m/s, k per m/s)
        self.rp_white, self.rp_gm = (
            np.radians(rp_white_deg),
            (np.radians(rp_gm_deg[0]), rp_gm_deg[1]),
        )
        self.yaw_white, self.yaw_rw = np.radians(yaw_white_deg), np.radians(yaw_rw_deg)
        self.gv, self.ga, self.yaw_drift = np.zeros(3), np.zeros(2), 0.0

    def sample(self, dt, R_true, v_true):
        r = self.rng
        a = np.exp(-dt / self.v_gm[1])
        self.gv = a * self.gv + np.sqrt(1 - a * a) * self.v_gm[0] * r.standard_normal(3)
        a = np.exp(-dt / self.rp_gm[1])
        self.ga = a * self.ga + np.sqrt(1 - a * a) * self.rp_gm[0] * r.standard_normal(
            2
        )
        self.yaw_drift += self.yaw_rw * np.sqrt(dt) * r.standard_normal()
        vb = R_true.T @ v_true
        vb = (
            (1.0 + self.scale) * vb
            + self.gv
            + (self.v_white[0] + self.v_white[1] * np.linalg.norm(vb))
            * r.standard_normal(3)
        )
        err = np.array(
            [
                self.ga[0] + self.rp_white * r.standard_normal(),
                self.ga[1] + self.rp_white * r.standard_normal(),
                self.yaw_white * r.standard_normal(),
            ]
        )
        return vb, rot_z(self.yaw_drift) @ R_true @ exp_so3(err)


class Eskf:
    def __init__(
        self,
        p0,
        v0,
        R0,
        sigma_gyro,
        sigma_accel,
        bias_walk=(1e-4, 2e-3),
        sigma_vio_v=0.07,
        sigma_vio_att_deg=0.5,
        sigma_height=0.3,
        drift=0.05,
    ):
        """sigma_gyro / sigma_accel: per-sample white noise of the IMU (rad/s, m/s^2); bias_walk: filter's (gyro, accel) bias process noise per sqrt(s)."""
        self.p, self.v, self.R = (
            np.array(p0, float),
            np.array(v0, float),
            np.array(R0, float),
        )
        self.ba, self.bg = np.zeros(3), np.zeros(3)
        self.g = np.array([0.0, 0.0, -G])
        self.sg, self.sa = (
            2.0 * sigma_gyro,
            2.0 * sigma_accel,
        )  # x2: vibration / quantisation not in the white-noise figure
        self.bw = bias_walk
        self.drift = drift  # m/sqrt(s): position process noise standing for the VIO's correlated velocity / heading errors, which the white
        # measurement sigmas do not cover (a calibrated pessimism: without it the covariance reads 5x tighter than the dead-reckoned error)
        self.sv, self.sth, self.sh = (
            sigma_vio_v,
            np.radians(sigma_vio_att_deg),
            sigma_height,
        )
        self.P = np.diag(
            [0.01**2] * 3
            + [0.05**2] * 3
            + [np.radians(0.5) ** 2] * 3
            + [0.2**2] * 3
            + [0.01**2] * 3
        )
        self.hist = deque(maxlen=200)  # (t, p, v, R) after each predict
        self.t = 0.0
        self.w = np.zeros(3)  # last bias-corrected body rate

    # ---- propagation ------------------------------------------------------------------------------------------------------------------
    def predict(self, gyro, accel, dt):
        w, f = np.asarray(gyro) - self.bg, np.asarray(accel) - self.ba
        R = self.R
        a = R @ f + self.g
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R @ skew(f) * dt
        F[3:6, 9:12] = -R * dt
        F[6:9, 6:9] = np.eye(3) - skew(w) * dt
        F[6:9, 12:15] = -np.eye(3) * dt
        self.p = self.p + self.v * dt + 0.5 * a * dt * dt
        self.v = self.v + a * dt
        self.R = R @ exp_so3(w * dt)
        Q = np.zeros(15)
        Q[0:3] = self.drift**2 * dt
        Q[3:6], Q[6:9] = (self.sa * dt) ** 2, (self.sg * dt) ** 2
        Q[9:12], Q[12:15] = self.bw[1] ** 2 * dt, self.bw[0] ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag(Q)
        self.t += dt
        self.w = w
        self.hist.append((self.t, self.p.copy(), self.v.copy(), self.R.copy()))

    # ---- updates ----------------------------------------------------------------------------------------------------------------------
    def _inject(self, H, y, Rn):
        S = H @ self.P @ H.T + Rn
        K = np.linalg.solve(S, H @ self.P).T
        dx = K @ y
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rn @ K.T
        self.p, self.v = self.p + dx[0:3], self.v + dx[3:6]
        self.R = self.R @ exp_so3(dx[6:9])
        self.ba, self.bg = self.ba + dx[9:12], self.bg + dx[12:15]

    def _state_at(self, t):
        if not self.hist:
            return self.p, self.v, self.R
        h = min(self.hist, key=lambda e: abs(e[0] - t))
        return h[1], h[2], h[3]

    def update_vio(self, t_sample, v_body, R_meas):
        """Body-frame velocity and attitude of the VIO frame sampled at `t_sample` (filter time)."""
        _, v_h, R_h = self._state_at(t_sample)
        vb = R_h.T @ v_h
        y = np.concatenate([np.asarray(v_body) - vb, log_so3(R_h.T @ R_meas)])
        H = np.zeros((6, 15))
        H[0:3, 3:6], H[0:3, 6:9], H[3:6, 6:9] = R_h.T, skew(vb), np.eye(3)
        self._inject(H, y, np.diag([self.sv**2] * 3 + [self.sth**2] * 3))

    def update_height(self, z):
        H = np.zeros((1, 15))
        H[0, 2] = 1.0
        self._inject(H, np.array([z - self.p[2]]), np.array([[self.sh**2]]))

    # ---- outputs ----------------------------------------------------------------------------------------------------------------------
    def sigmas(self):
        """1-sigma of position (3), velocity (3), attitude (3, rad) from the covariance diagonal."""
        return np.sqrt(np.maximum(np.diag(self.P)[:9], 0.0))


def baro_height(pressure_pa, p0_pa=101325.0):
    """ISA height (m) of a static pressure above the p0 level."""
    return 44330.0 * (1.0 - (pressure_pa / p0_pa) ** (1.0 / 5.255))
