"""State estimation of the lander flight computer: gyro-integrated attitude with pad calibration, and one 3-state Kalman filter per world axis
(position, velocity, accelerometer bias) fed by the barometer (z) and GNSS (x, y, z velocity). Pure numpy, no simulator imports."""

import math

import numpy as np

G = 9.80665


# ---------------------------------------------------------------------------------------------------------------- attitude
def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_exp(rv):
    """Unit quaternion of the rotation vector rv (rad)."""
    a = float(np.linalg.norm(rv))
    if a < 1e-12:
        return np.array([1.0, *(0.5 * np.asarray(rv))])
    s = math.sin(0.5 * a) / a
    return np.array([math.cos(0.5 * a), *(s * np.asarray(rv))])


def quat_to_mat(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_from_up(u):
    """Attitude (body -> world) with the body-frame up vector u (the third row of R) and zero heading: the minimal rotation taking u to world z."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    axis = np.cross(u, [0.0, 0.0, 1.0])
    s = float(np.linalg.norm(axis))
    if s < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quat_exp(axis / s * math.atan2(s, float(u[2])))


class Attitude:
    """Quaternion (body -> world) integrated from the gyro. Calibrated on the pad (mean gyro = bias, mean accel = up vector); in flight the accelerometer
    corrects tilt (Mahony complementary term) only while the specific force is within `gate` of 1 g, i.e. never under thrust or in free fall:
    the flight is gyro-only, and the unit test bounds the drift."""

    def __init__(self, kp=1.0, gate=0.15):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self.kp, self.gate = kp, gate
        self._g, self._a, self._n = np.zeros(3), np.zeros(3), 0

    def calibrate(self, gyro, accel):
        self._g += gyro
        self._a += accel
        self._n += 1

    def finish_calibration(self):
        self.bias = self._g / self._n
        self.q = quat_from_up(self._a / self._n)

    def update(self, gyro, accel, dt):
        w = np.asarray(gyro, float) - self.bias
        f = float(np.linalg.norm(accel))
        if abs(f - G) < self.gate * G:  # near 1 g: gravity is the specific force
            u_meas = np.asarray(accel, float) / f
            u_pred = quat_to_mat(self.q).T[:, 2]  # body-frame up (R^T z)
            w = w + self.kp * np.cross(u_meas, u_pred)
        self.q = quat_mul(self.q, quat_exp(w * dt))
        self.q /= np.linalg.norm(self.q)

    @property
    def R(self):
        return quat_to_mat(self.q)

    @property
    def tilt(self):
        """Angle between the body axis and the vertical (rad)."""
        return math.acos(max(-1.0, min(1.0, self.R[2, 2])))

    def euler(self):
        """(roll about x, pitch about y, yaw about z) in rad, ZYX; for the telemetry."""
        R = self.R
        return (
            math.atan2(R[2, 1], R[2, 2]),
            -math.asin(max(-1.0, min(1.0, R[2, 0]))),
            math.atan2(R[1, 0], R[0, 0]),
        )


# ------------------------------------------------------------------------------------------------------------ axis filter
class AxisFilter:
    """x = [position, velocity, accelerometer bias] of one world axis, propagated with the measured world acceleration, corrected by delayed position /
    velocity fixes. A measurement `lag` ticks old is compared with the state stored `lag` ticks ago, and the Kalman gain is applied to the current state
    (a cheap delayed-measurement form: it ignores the correlation between the stored and the current state)."""

    def __init__(
        self, dt, sigma_acc=0.3, sigma_bias_rw=0.02, p0=(1.0, 0.5, 0.3), history=80
    ):
        self.dt = dt
        self.x = np.zeros(3)
        self.P = np.diag(np.square(p0)).astype(float)
        self.sa2, self.sb2 = sigma_acc**2, sigma_bias_rw**2
        self.hist = []
        self.nh = history

    def predict(self, a):
        dt = self.dt
        F = np.array([[1.0, dt, -0.5 * dt * dt], [0.0, 1.0, -dt], [0.0, 0.0, 1.0]])
        g = np.array([0.5 * dt * dt, dt, 0.0])
        p, v, b = self.x
        ac = a - b
        self.x = np.array([p + v * dt + 0.5 * ac * dt * dt, v + ac * dt, b])
        Q = np.outer(g, g) * self.sa2 + np.diag([0.0, 0.0, self.sb2 * dt])
        self.P = F @ self.P @ F.T + Q
        self.hist.append(self.x.copy())
        if len(self.hist) > self.nh:
            self.hist.pop(0)

    def update(self, idx, z, sigma, lag=0):
        """Scalar measurement of state `idx` (0 position, 1 velocity) with std `sigma`, `lag` ticks old."""
        ref = self.hist[-1 - lag][idx] if lag < len(self.hist) else self.x[idx]
        H = np.zeros(3)
        H[idx] = 1.0
        S = float(H @ self.P @ H) + sigma**2
        K = self.P @ H / S
        self.x = self.x + K * (z - ref)
        self.P = (np.eye(3) - np.outer(K, H)) @ self.P

    @property
    def pos(self):
        return float(self.x[0])

    @property
    def vel(self):
        return float(self.x[1])


def baro_height(p_pa, p_ref_pa):
    """Height above the pad (m) from static pressure: ISA troposphere inverted about the pad pressure."""
    k = 1.0 / 5.255876
    return 288.15 / 0.0065 * ((p_ref_pa / 101325.0) ** k - (p_pa / 101325.0) ** k)
