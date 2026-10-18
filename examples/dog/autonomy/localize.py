"""4-DOF EKF over (x, y, z, yaw) in the map frame (ENU of the GNSS origin = world, alt0 = 0). Roll / pitch come from odometry
(gravity-referenced). Prediction: the odometry increment rotated by the current map-odometry yaw offset, Q = drift^2 x distance.
RTK fixes arrive latency_s late: the pose at the fix epoch is rebuilt as the current state minus the odometry motion since then
(not a stored state: stored states go stale after every update), with the antenna lever arm at that epoch's attitude."""

import math

import numpy as np

from . import lidar

A_WGS84, F_WGS84 = 6378137.0, 1 / 298.257223563
E2 = F_WGS84 * (2 - F_WGS84)


def radii(lat0):
    s = math.sin(math.radians(lat0))
    w = 1 - E2 * s * s
    return A_WGS84 * (1 - E2) / w**1.5, A_WGS84 / math.sqrt(w)


def enu(lat, lon, alt, G):
    """Inverse of the GNSS device's flat-earth map (docs/models.md, GNSS)."""
    m, n = radii(G["lat0"])
    return np.array(
        [
            math.radians(lon - G["lon0"]) * n * math.cos(math.radians(G["lat0"])),
            math.radians(lat - G["lat0"]) * m,
            alt - G["alt0"],
        ]
    )


def wrap(a):
    return math.remainder(a, 2 * math.pi)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _perp(v):  # d(Rz(a) u) / da = perp(Rz(a) u)
    return np.array([-v[1], v[0], 0.0])


def _qz(a):
    return np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)])


class GnssReader:
    def __init__(self, sim, name, cfg, dt):
        self.G, self.sim, self.path, self.dt = cfg["gnss"], sim, f"/devices/{name}", dt
        self.period = int(round(1.0 / (self.G["rate_hz"] * dt)))
        self.delay = int(round(self.G["latency_s"] * self.G["rate_hz"]))
        self.last_s, self.prev, self.held, self.ok_after = -1, None, False, -math.inf

    def poll(self):
        if self.sim.tick < 1:
            return None
        s = (
            self.sim.tick - 1
        ) // self.period  # latest sample index (sampled on tick s * period)
        if s == self.last_s:
            return None
        self.last_s = s
        if s < self.delay:
            return None  # output still at the origin
        v = tuple(
            float(self.sim.query(self.path, a)[0][0]) for a in ("lat", "lon", "alt")
        )
        t = (s - self.delay) * self.period * self.dt
        if v == self.prev:  # fix_lost: the output holds
            self.held = True
            return None
        self.prev = v
        if self.held:
            self.held, self.ok_after = False, t + self.G["reconverge_s"]
        if t < self.ok_after:
            return None
        return t, enu(*v, self.G)


class Localizer:
    def __init__(self, cfg, x0, lever):
        self.O, self.Lc, self.G = cfg["odom"], cfg["loc"], cfg["gnss"]
        self.x = np.array(x0, dtype=np.float64)
        self.P = np.diag([self.Lc["init_sigma"] ** 2] * 3 + [self.Lc["init_yaw"] ** 2])
        self.lever = np.asarray(lever, dtype=np.float64)
        self.key_dt = 1.0 / cfg["lidar"]["rate_hz"]
        self.hist, self.last, self.t_fix = {}, None, -math.inf
        self.rejected = 0  # consecutive rejected RTK fixes

    def predict(self, od):
        yaw_o = lidar.yaw_of(od.quat)
        q_rp = lidar.qmul(_qz(-yaw_o), od.quat)
        if self.last is not None:
            p0, y0 = self.last[0], self.last[1]
            dp = rz(self.x[3] - y0) @ (od.pos - p0)
            ds = math.hypot(dp[0], dp[1])
            Fm = np.eye(4)
            Fm[:3, 3] = _perp(dp)
            self.x[:3] += dp
            self.x[3] = wrap(self.x[3] + wrap(yaw_o - y0))
            Q = np.diag(
                [self.O["drift_xy"] ** 2 * ds] * 2
                + [self.O["drift_z"] ** 2 * ds, self.O["drift_yaw"] ** 2 * ds]
            )
            self.P = Fm @ self.P @ Fm.T + Q
        self.last = (np.array(od.pos, dtype=np.float64), yaw_o, q_rp, od.t)
        k = int(round(od.t / self.key_dt))
        self.hist[k] = self.last
        for old in [j for j in self.hist if j < k - 50]:  # 5 s of history
            del self.hist[old]

    def pose(self):
        return self.x[:3].copy(), lidar.qmul(_qz(self.x[3]), self.last[2])

    @property
    def var_z(self):
        return float(self.P[2, 2])

    def update(self, z, h, H, R, chi2, wrap_last=False):
        nu = z - h
        if wrap_last:
            nu[-1] = wrap(nu[-1])
        S = H @ self.P @ H.T + R
        if float(nu @ np.linalg.solve(S, nu)) > chi2:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.x[3] = wrap(self.x[3])
        IKH = np.eye(4) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T  # Joseph form
        return True

    def update_rtk(self, t_meas, z):
        rec = self.hist.get(int(round(t_meas / self.key_dt)))
        if rec is None or self.last is None:
            return False
        p_tm, yaw_tm_o, q_rp_tm, _ = rec
        p_now, yaw_now_o = self.last[0], self.last[1]
        dp = rz(self.x[3] - yaw_now_o) @ (
            p_now - p_tm
        )  # odometry motion since the epoch, in the map frame
        yaw_tm = self.x[3] - wrap(yaw_now_o - yaw_tm_o)
        lev = lidar.qmat(lidar.qmul(_qz(yaw_tm), q_rp_tm)) @ self.lever
        h = self.x[:3] - dp + lev
        H = np.zeros((3, 4))
        H[:, :3] = np.eye(3)
        H[:, 3] = -_perp(dp) + _perp(
            lev
        )  # H[0,3] = dp_y - lev_y, H[1,3] = -dp_x + lev_x
        R = np.diag([self.G["horiz_std"] ** 2] * 2 + [self.G["vert_std"] ** 2])
        z = np.asarray(z, float)
        ok = self.update(z, h, H, R, self.Lc["chi2"])
        self.rejected = 0 if ok else self.rejected + 1
        if (
            self.rejected >= self.Lc["rtk_reset_n"]
        ):  # lost in ICP / odometry: take the position from the fix (recovery)
            self.x[:3] += z - h
            self.P[:3, :] = 0.0
            self.P[:, :3] = 0.0
            self.P[:3, :3] = R + np.eye(3) * self.Lc["init_sigma"] ** 2
            self.rejected, ok = 0, True
        if ok:
            self.t_fix = max(self.t_fix, t_meas)
        return ok

    def update_icp(self, x_meas, cov):
        return self.update(
            np.asarray(x_meas, float),
            self.x.copy(),
            np.eye(4),
            cov,
            self.Lc["chi2_icp"],
            wrap_last=True,
        )

    def rtk_ok(self, t_now):
        return t_now - self.t_fix < self.Lc["rtk_age"]
