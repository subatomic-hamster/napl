"""Rosette lidar points, mirroring the kernel (src/mujoco/devices/rangefinder.cpp::rosette_dir, offsets in devices.cpp).

Ray m = frame * n + k: u1 = frac(off1 + G1 m), u2 = frac(off2 + G2 m), az = fov_h (u1 - 1/2), el = fov_v_min + (fov_v_max -
fov_v_min) u2 (uniform in angle); site-frame direction (sin el, cos el sin az, cos el cos az): +x = sensor up, +z = forward.
A sample describes the state at the start of its tick; the device's `t` is that tick's end, so the pose time is t - dt."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

G1, G2 = (
    0.7548776662466927,
    0.5698402909980532,
)  # 1/g, 1/g^2, g = plastic number (kernel constants)
PHI = 0.6180339887498949
SITE_Q0 = np.array(
    [0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)]
)  # site +x = body up, +z = body forward, +y = body right


def offsets(pattern_seed):
    s = float(int(pattern_seed) % 1000003)
    return math.fmod(0.5 + s * PHI, 1.0), math.fmod(0.5 + s * PHI * PHI, 1.0)


def directions(n, frame, fov_v_min, fov_v_max, pattern_seed, fov_h=2 * math.pi):
    o1, o2 = offsets(pattern_seed)
    m = float(frame) * n + np.arange(n, dtype=np.float64)
    u1, u2 = o1 + G1 * m, o2 + G2 * m
    az = fov_h * (u1 - np.floor(u1) - 0.5)
    el = fov_v_min + (fov_v_max - fov_v_min) * (u2 - np.floor(u2))
    ce = np.cos(el)
    return np.stack([np.sin(el), ce * np.sin(az), ce * np.cos(az)], axis=1)


def device(name, site, cfg, rays):
    L = cfg["lidar"]
    return {
        "name": name, "site": site, "rate_hz": L["rate_hz"], "projection": "rosette", "fov_h": 2 * math.pi,
        "fov_v_min": L["fov_v_min"], "fov_v_max": L["fov_v_max"], "rays_per_frame": int(rays),
        "min_range": L["min_range"], "max_range": L["max_range"], "sigma0": L["sigma0"],
        "ray_groups": 1, "seed": int(L["seed"]), "pattern_seed": int(L["pattern_seed"]),
    }  # fmt: skip


@dataclass
class Scan:
    t: float  # time of the pose the rays were cast from
    frame: int
    dirs: np.ndarray  # (m, 3) unit, site frame, returns only
    r: np.ndarray  # (m,) range


class Rosette:
    def __init__(self, sim, name, rays, dt, cfg):
        self.sim, self.path, self.n, self.dt = sim, f"/devices/{name}", int(rays), dt
        L = cfg["lidar"]
        self.lims = (L["fov_v_min"], L["fov_v_max"], int(L["pattern_seed"]))
        self.seen = 0

    def latest(self):
        frames = int(self.sim.query(self.path, "frames")[0][0])
        if frames == self.seen:
            return None
        self.seen = frames
        r = np.asarray(self.sim.query(self.path, "range")[0], dtype=np.float64).ravel()
        t = float(self.sim.query(self.path, "t")[0][0]) - self.dt
        d = directions(self.n, frames - 1, *self.lims)
        ok = r > 0
        return Scan(t, frames - 1, d[ok], r[ok])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])  # fmt: skip


def _axis(i, a):
    q = np.zeros(4)
    q[0], q[1 + i] = math.cos(a / 2), math.sin(a / 2)
    return q


def quat_zyx(yaw, pitch, roll):
    return qmul(qmul(_axis(2, yaw), _axis(1, pitch)), _axis(0, roll))


def qmat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def points(scan, pos, quat, ang_sigma=0.0, rng=None):
    d = scan.dirs
    if (
        ang_sigma > 0
    ):  # the kernel's rays are exact; the datasheet angular precision goes on the reported direction
        g = rng.standard_normal(d.shape)
        g -= (g * d).sum(1, keepdims=True) * d
        d = d + ang_sigma * g
        d /= np.linalg.norm(d, axis=1, keepdims=True)
    return np.asarray(pos, float) + (scan.r[:, None] * d) @ qmat(quat).T


def _local(el):
    p = np.array([float(v) for v in el.get("pos", "0 0 0").split()])
    bad = [k for k in ("axisangle", "xyaxes", "zaxis") if k in el.attrib]
    if bad:
        raise ValueError(
            f"extrinsic: {el.get('name')} uses {bad[0]}; only pos / quat / euler are supported"
        )
    if "quat" in el.attrib:
        return p, np.array([float(v) for v in el.get("quat").split()])
    if "euler" in el.attrib:
        a, b, c = (math.radians(float(v)) for v in el.get("euler").split())
        return p, qmul(
            qmul(_axis(0, a), _axis(1, b)), _axis(2, c)
        )  # MuJoCo default eulerseq "xyz", degrees
    return p, np.array([1.0, 0, 0, 0])


def extrinsic(xml_text, site, body):
    root = ET.fromstring(xml_text)
    comp = root.find("compiler")
    if comp is not None and (
        comp.get("angle", "degree") != "degree" or comp.get("eulerseq", "xyz") != "xyz"
    ):
        raise ValueError(
            "extrinsic: compiler angle / eulerseq must be MuJoCo's defaults"
        )
    parent = {c: p for p in root.iter() for c in p}
    el = next(s for s in root.iter("site") if s.get("name") == site)
    chain = []
    while el.get("name") != body or el.tag != "body":
        chain.append(el)
        el = parent[el]
    pos, q = np.zeros(3), np.array([1.0, 0, 0, 0])
    for e in reversed(chain):
        p, qe = _local(e)
        pos, q = pos + qmat(q) @ p, qmul(q, qe)
    return pos, q
