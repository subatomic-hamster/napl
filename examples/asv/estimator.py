"""State estimation of the ASV from its sensors only: a Mahony attitude filter (gyro + accelerometer + magnetometer) and a 6-state position / velocity /
accelerometer-bias Kalman filter (IMU accelerations propagate, GNSS position and velocity correct). Nothing here sees the true state.

Frames: world ENU (x East, y North, z up), body x forward, y port, z up. Sensor conventions: gyro rad/s about the body axes, accel the specific force (m/s^2, +g on z
at rest), mag the field in the body frame (T), GNSS lat/lon (deg) and velocity (north, east, down) (m/s).
Fidelity: the Mahony filter has no model of the wave-induced accelerations (they act as noise on the tilt and as horizontal acceleration noise on the
position filter); GNSS latency is compensated by extrapolating each fix with its own velocity (valid while the velocity is steady over the latency).
"""

import math

import numpy as np

from .geo import LAT0, LON0, enu_from_latlon, yaw_of_quat

G = 9.80665


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def rot_of_quat(q):
    """Body-to-world rotation matrix of a unit quaternion (w, x, y, z)."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_of_rot(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 2 * math.sqrt(1 + t)
        q = [
            s / 4,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [
            (R[2, 1] - R[1, 2]) / s,
            s / 4,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
        ]
    elif R[1, 1] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [
            (R[0, 2] - R[2, 0]) / s,
            (R[0, 1] + R[1, 0]) / s,
            s / 4,
            (R[1, 2] + R[2, 1]) / s,
        ]
    else:
        s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [
            (R[1, 0] - R[0, 1]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            s / 4,
        ]
    q = np.array(q)
    return q / np.linalg.norm(q)


class Attitude:
    """Mahony complementary filter on a quaternion, MARG form: the accelerometer corrects roll / pitch, the magnetometer yaw, the gyro integrates.

    kp_acc / kp_mag: proportional gains (1/s) of the two corrections; ki: integral gain on the gyro bias. `mag_ref` is the local field in the world frame
    (any scale), what a navigation computer takes from the World Magnetic Model for its position."""

    def __init__(self, mag_ref, kp_acc=0.8, kp_mag=0.5, ki=0.01):
        self.mref = np.asarray(mag_ref, float) / np.linalg.norm(mag_ref)
        self.kp_acc, self.kp_mag, self.ki = kp_acc, kp_mag, ki
        self.q = np.array([1.0, 0, 0, 0])
        self.bias = np.zeros(3)
        self.ready = False

    def init(self, acc, mag):
        """Initial attitude from one accelerometer and one magnetometer sample (TRIAD): body up from the specific force, the field fixes the heading."""
        b1 = np.asarray(acc, float) / np.linalg.norm(
            acc
        )  # body-frame direction of world +z at rest
        w1 = np.array([0.0, 0, 1])
        w2 = np.cross(w1, self.mref)
        w2 /= np.linalg.norm(w2)
        b2 = np.cross(b1, np.asarray(mag, float))
        b2 /= np.linalg.norm(b2)
        # TRIAD: the rotation that takes the body triad (up, up x field, ...) to the world one; R maps body vectors to world
        R = (
            np.outer(w1, b1)
            + np.outer(w2, b2)
            + np.outer(np.cross(w1, w2), np.cross(b1, b2))
        )
        self.q = quat_of_rot(R)
        self.ready = True

    def update(self, gyro, acc, mag, dt):
        if not self.ready:
            if (
                np.linalg.norm(acc) > 0.5 * G and np.linalg.norm(mag) > 0
            ):  # the devices hold zeros until their first sample
                self.init(acc, mag)
            return
        R = rot_of_quat(self.q)
        a = np.asarray(acc, float)
        na = np.linalg.norm(a)
        m = np.asarray(mag, float)
        nm = np.linalg.norm(m)
        e = np.zeros(3)
        if na > 0.3 * G:  # not in free fall
            e += self.kp_acc * np.cross(a / na, R.T @ np.array([0.0, 0, 1]))
        if nm > 0:
            e += self.kp_mag * np.cross(m / nm, R.T @ self.mref)
        self.bias -= self.ki * e * dt
        w = np.asarray(gyro, float) - self.bias + e
        dq = 0.5 * quat_mul(self.q, np.array([0.0, *w]))
        self.q = self.q + dq * dt
        self.q /= np.linalg.norm(self.q)

    @property
    def yaw(self):
        return yaw_of_quat(self.q)

    def roll_pitch(self):
        R = rot_of_quat(self.q)
        return math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0])))


class Navigator:
    """Attitude + position / velocity estimate. Call `imu(gyro, acc, mag, dt)` at the IMU rate and `gnss(lat, lon, vel_ned)` for every new fix."""

    def __init__(
        self,
        mag_ref,
        gnss_latency=0.2,
        pos_sigma=2.2,
        vel_sigma=0.1,
        acc_noise=0.3,
        bias_walk=0.002,
        lat0=LAT0,
        lon0=LON0,
    ):
        self.att = Attitude(mag_ref)
        self.lat0, self.lon0 = lat0, lon0
        self.latency = gnss_latency
        self.rp, self.rv = pos_sigma**2, vel_sigma**2
        self.qa, self.qb = acc_noise**2, bias_walk**2
        self.x = np.zeros(6)  # E, N, vE, vN, bias_aE, bias_aN
        self.P = np.diag([25.0, 25.0, 1.0, 1.0, 0.04, 0.04])
        self.gyro = np.zeros(3)
        self.have_fix = False
        self.t_fix = None

    # -- IMU: propagate -------------------------------------------------------------------------------------------------------------
    def imu(self, gyro, acc, mag, dt):
        self.att.update(gyro, acc, mag, dt)
        self.gyro = np.asarray(gyro, float) - self.att.bias
        if not self.have_fix:
            return
        a_w = rot_of_quat(self.att.q) @ np.asarray(acc, float) - np.array([0.0, 0, G])
        a = a_w[:2] - self.x[4:6]
        x = self.x
        x[0:2] += x[2:4] * dt + 0.5 * a * dt * dt
        x[2:4] += a * dt
        F = np.eye(6)
        F[0, 2] = F[1, 3] = dt
        F[2, 4] = F[3, 5] = -dt
        Q = np.diag([0, 0, self.qa * dt, self.qa * dt, self.qb * dt, self.qb * dt])
        self.P = F @ self.P @ F.T + Q

    # -- GNSS: correct --------------------------------------------------------------------------------------------------------------
    def gnss(self, lat, lon, vel_ned):
        """One new fix. The position is moved forward by `latency` along the fix's own velocity (the receiver reports a solution that old)."""
        e, n = enu_from_latlon(lat, lon, self.lat0, self.lon0)
        ve, vn = float(vel_ned[1]), float(vel_ned[0])
        z = np.array([e + ve * self.latency, n + vn * self.latency, ve, vn])
        if not self.have_fix:
            self.x[:4] = z
            self.have_fix = True
            return
        H = np.zeros((4, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = H[3, 3] = 1.0
        # the position error is time-correlated (Gauss-Markov, 600 s): use the measurement variance inflated so the filter does not trust single fixes
        R = np.diag([self.rp, self.rp, self.rv, self.rv])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    # -- outputs --------------------------------------------------------------------------------------------------------------------
    @property
    def yaw(self):
        return self.att.yaw

    @property
    def yaw_rate(self):
        """Rate about the world vertical (rad/s, + counter-clockwise): the body rate projected on the up axis."""
        R = rot_of_quat(self.att.q)
        return float(R[2, :] @ self.gyro)

    def state(self):
        """Estimate dict: pos (E, N), vel (vE, vN), psi, r, speed, cog, u (surge), v (sway, + port), roll, pitch."""
        psi = self.yaw
        vel = self.x[2:4].copy()
        sp = float(np.hypot(*vel))
        c, s = math.cos(psi), math.sin(psi)
        roll, pitch = self.att.roll_pitch()
        return dict(
            pos=self.x[0:2].copy(), vel=vel, psi=psi, r=self.yaw_rate, speed=sp, cog=math.atan2(vel[1], vel[0]) if sp > 0.2 else psi,
            u=c * vel[0] + s * vel[1], v=-s * vel[0] + c * vel[1], roll=roll, pitch=pitch,
        )  # fmt: skip
