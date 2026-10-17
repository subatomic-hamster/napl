"""Kinematic carrier (harness): a pose-commanded body with the dog's sensors, gravity off, no contacts (geom group 4).
The pose is set (a logged command) at the start of each lidar frame, so every frame is cast from exactly the commanded pose;
the body follows the ground under four virtual feet. Also the harness' drifting odometry and GNSS sky-view outages."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from napl.terrain import add_field

from . import lidar
from .iface import Odom

FEET = ((1, 1), (1, -1), (-1, 1), (-1, -1))  # (fore +1 / hind -1, left +1 / right -1)


def _s(v):
    return " ".join(
        repr(float(x)) for x in v
    )  # numpy 2 reprs np.float64 as "np.float64(...)"


def terrain_pose(f, cfg, x, y, yaw):
    C = cfg["carrier"]
    a, b = C["foot_a"], C["foot_b"]
    c, s = math.cos(yaw), math.sin(yaw)
    h = {}
    for i, j in FEET:
        h[i, j] = float(f.height(x + c * i * a - s * j * b, y + s * i * a + c * j * b))
    front, rear = (h[1, 1] + h[1, -1]) / 2, (h[-1, 1] + h[-1, -1]) / 2
    left, right = (h[1, 1] + h[-1, 1]) / 2, (h[1, -1] + h[-1, -1]) / 2
    pitch = math.atan2(
        rear - front, 2 * a
    )  # front lower -> nose down -> positive pitch about +y
    roll = math.atan2(left - right, 2 * b)
    z = (front + rear) / 2 + C["base_h"]
    return np.array([x, y, z]), lidar.quat_zyx(yaw, pitch, roll)


def build(f, cfg, rays, start, out_dir):
    L, C, G = cfg["lidar"], cfg["carrier"], cfg["gnss"]
    root = ET.fromstring(
        f'<mujoco><option timestep="{C["dt"]!r}" gravity="0 0 0"/><worldbody><light pos="0 0 30"/></worldbody></mujoco>'
    )
    add_field(root, f)
    p, q = terrain_pose(f, cfg, *start)
    qs = lidar.qmul(lidar.quat_zyx(0.0, L["tilt"], 0.0), lidar.SITE_Q0)
    wb = root.find("worldbody")
    body = ET.SubElement(wb, "body", name="carrier", pos=_s(p), quat=_s(q))
    ET.SubElement(body, "freejoint", name="fj")
    ET.SubElement(
        body,
        "geom",
        type="box",
        size="0.3 0.12 0.06",
        mass="12",
        contype="0",
        conaffinity="0",
        group="4",
    )
    ET.SubElement(
        body,
        "site",
        name="lidar",
        pos=f"{L['mount_x']!r} 0 {L['mount_height'] - C['base_h']!r}",
        quat=_s(qs),
    )
    ET.SubElement(body, "site", name="gnss", pos=f"{G['lever_x']!r} 0 {G['lever_z']!r}")
    gnss = {
        "name": "gnss", "site": "gnss", "rate_hz": G["rate_hz"], "lat0": G["lat0"], "lon0": G["lon0"], "alt0": G["alt0"],
        "horiz_std": G["horiz_std"], "vert_std": G["vert_std"], "vel_std": 0.0,  # GNSS velocity unused by SP5
        "latency_s": G["latency_s"], "seed": 7, "corr_time_s": G["corr_time_s"], "corr_frac": G["corr_frac"],
    }  # fmt: skip
    devs = {
        "rangefinders": [lidar.device("lidar", "lidar", cfg, rays)],
        "gnss_receivers": [gnss],
    }
    custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "text", name="napl.devices", data=json.dumps(devs))
    out = Path(out_dir) / "carrier.xml"
    out.write_text(ET.tostring(root, encoding="unicode"))
    return out


class DriftOdom:
    """Odometry = truth increments with random-walk drift per metre travelled (variance drift^2 * distance) and white
    roll / pitch error. Yaw drift rotates later increments, as a real leg-inertial estimator does."""

    def __init__(self, cfg, seed):
        self.o = cfg["odom"]
        self.rng = np.random.default_rng([seed, 0x0D0])
        self.prev = None
        self.p = None
        self.dyaw = 0.0

    def update(self, t, pos, quat):
        yaw = lidar.yaw_of(quat)
        R = lidar.qmat(quat)
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        if self.prev is None:
            self.p = np.array(pos, float)
        else:
            dp = pos - self.prev[0]
            ds = float(np.hypot(dp[0], dp[1]))
            g = self.rng.standard_normal(4)
            c, s = math.cos(self.dyaw), math.sin(self.dyaw)
            self.p = self.p + np.array(
                [c * dp[0] - s * dp[1], s * dp[0] + c * dp[1], dp[2]]
            )
            self.p += math.sqrt(ds) * np.array(
                [
                    self.o["drift_xy"] * g[0],
                    self.o["drift_xy"] * g[1],
                    self.o["drift_z"] * g[2],
                ]
            )
            self.dyaw += self.o["drift_yaw"] * math.sqrt(ds) * g[3]
        self.prev = (np.array(pos, float), yaw)
        e = self.o["att_sigma"] * self.rng.standard_normal(2)
        q = lidar.quat_zyx(yaw + self.dyaw, pitch + e[0], roll + e[1])
        return Odom(t, self.p.copy(), q, np.zeros(3), np.zeros(3), False)


def sky_blocked(sim, p, cfg):
    G = cfg["gnss"]
    n, el = int(G["sky_az"]), G["sky_elev"]
    az = 2 * math.pi * np.arange(n) / n
    d = np.stack(
        [
            math.cos(el) * np.cos(az),
            math.cos(el) * np.sin(az),
            np.full(n, math.sin(el)),
        ],
        axis=1,
    )
    r = np.asarray(sim.raycast(p, d))
    return bool(((r >= 0) & (r <= G["sky_range"])).mean() >= G["sky_block"])


class Carrier:
    base, lidar, gnss = "/world/carrier", "lidar", "gnss"

    def __init__(self, sim, f, cfg, start, seed=0):
        self.sim, self.f, self.cfg, self.C = sim, f, cfg, cfg["carrier"]
        self.dt = self.C["dt"]
        self.chunk = int(
            round(1.0 / (cfg["lidar"]["rate_hz"] * self.dt))
        )  # physics steps per lidar frame
        self.x, self.y, self.yaw = (float(v) for v in start)
        self.v = np.zeros(3)  # vx, vy, wz (body)
        self.cmd = np.zeros(3)
        self.p, self.q = terrain_pose(f, cfg, self.x, self.y, self.yaw)
        self.odo = DriftOdom(cfg, seed)
        self.last = self.odo.update(0.0, self.p, self.q)
        self.bumps, self.gnss_lost, self.distance = 0, False, 0.0
        self.lever = np.array([cfg["gnss"]["lever_x"], 0.0, cfg["gnss"]["lever_z"]])
        self.time_of_last_frame = 0.0

    def command(self, vx, vy, wz):
        C = self.C
        self.cmd = np.array(
            [
                np.clip(vx, -C["v_max"], C["v_max"]),
                np.clip(vy, -C["v_max"] / 2, C["v_max"] / 2),
                np.clip(wz, -C["wz_max"], C["wz_max"]),
            ]
        )

    def limits(self):
        return {"v_max": self.C["v_max"], "wz_max": self.C["wz_max"]}

    def truth(self):
        return self.p.copy(), self.q.copy()

    def odometry(self):
        return self.last

    def fix(self, p_xy, yaw, t_pose, sig_p=None, sig_yaw=None):
        pass  # the carrier's pose is commanded, not estimated: nothing to correct

    def _blocked(self, x, y, yaw):
        C, f = self.C, self.f
        lim = f.size / 2 - C["foot_a"]
        if abs(x) > lim or abs(y) > lim:
            return True
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [
            (
                x + c * i * C["foot_a"] - s * j * C["foot_b"],
                y + s * i * C["foot_a"] + c * j * C["foot_b"],
            )
            for i, j in FEET
        ]
        pts.append((x, y))
        xs, ys = np.array(pts).T
        return bool((f.height(xs, ys) - f.hfield_height(xs, ys) > C["step_max"]).any())

    def _frame(self):
        C, T = self.C, self.chunk * self.dt
        lim = np.array([C["a_max"], C["a_max"], C["alpha_max"]]) * T
        self.v = self.v + np.clip(self.cmd - self.v, -lim, lim)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = self.x + (c * self.v[0] - s * self.v[1]) * T
        y = self.y + (s * self.v[0] + c * self.v[1]) * T
        yaw = math.remainder(self.yaw + self.v[2] * T, 2 * math.pi)
        if self._blocked(x, y, yaw):
            self.bumps += 1
            self.v[:2] = 0.0
            # a blocked step never leaves the body in contact (it used to keep the new yaw: feet inside a wall, every later step
            # blocked, a permanent trap: ruins mission, 7441 bumps); slide: the move alone, else the turn alone, else stay
            if self._blocked(x, y, self.yaw):
                x, y = self.x, self.y
                if self._blocked(x, y, yaw):
                    yaw = self.yaw
                    self.v[2] = 0.0
            else:
                yaw = self.yaw
                self.v[2] = 0.0
        self.distance += math.hypot(x - self.x, y - self.y)
        self.x, self.y, self.yaw = x, y, yaw
        self.p, self.q = terrain_pose(self.f, self.cfg, x, y, yaw)
        self.sim.set("/world/carrier/fj", "qpos", np.concatenate([self.p, self.q]))
        lost = sky_blocked(self.sim, self.p + lidar.qmat(self.q) @ self.lever, self.cfg)
        if lost != self.gnss_lost:
            self.sim.set("/devices/gnss", "fix_lost", 1.0 if lost else 0.0)
            self.gnss_lost = lost
        self.time_of_last_frame = self.sim.tick * self.dt
        self.sim.step(self.chunk)
        self.last = self.odo.update(self.time_of_last_frame, self.p, self.q)

    def advance(self, seconds):
        for _ in range(int(round(seconds / (self.chunk * self.dt)))):
            self._frame()
