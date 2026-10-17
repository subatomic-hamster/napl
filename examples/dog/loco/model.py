"""The controller's own model of the dog (SP4), from the DESIGN: the built MJCF's static body tree at q = 0 (body / joint / site poses, geom
masses, as the napl builder wrote them from the part files) and the napl.devices limits. No mjData, no runtime sim query.
Kinematics: product of exponentials (Lynch & Park, Modern Robotics ch. 4): one formula for all four legs, mirrored and turned legs included,
because every joint axis and point comes from the design. Inertia: geom masses as points plus each box geom's own inertia (an approximation:
mesh and cylinder geoms as points, ~10-20 % on the trunk inertia; the MPC tolerates it, the MC disperses the plant around it).
The link list (trunk + 3 per leg) and the joint-side armature (reflected rotor inertia) serve the whole-body controller (loco.wbc)."""

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")
ROOT = "deck_top"


def quat_R(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def rot(w, th):
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def vec(s, d):
    return np.array([float(v) for v in s.split()]) if s else np.array(d, float)


@dataclass
class Leg:
    name: str
    w: np.ndarray
    p: np.ndarray
    foot0: np.ndarray
    r_foot: float
    q_lo: np.ndarray
    q_hi: np.ndarray
    tau_max: np.ndarray
    kp_max: np.ndarray
    kd_max: np.ndarray
    bend: float = 1.0  # sign of the knee seed that bends this leg the designed way (set by from_build)

    def _chain(self, q, upto=3):
        R, t, ws, ps = np.eye(3), np.zeros(3), [], []
        for i in range(upto):
            ws.append(R @ self.w[i])
            ps.append(R @ self.p[i] + t)
            Ri = rot(self.w[i], q[i])
            t = (
                R @ (self.p[i] - Ri @ self.p[i]) + t
            )  # g <- g o (x -> p_i + R_i (x - p_i))
            R = R @ Ri
        return R, t, ws, ps

    def fk_jac(self, q):
        R, t, ws, ps = self._chain(q)
        x = R @ self.foot0 + t
        return x, np.column_stack([np.cross(w, x - p) for w, p in zip(ws, ps)])

    def knee(self, q):
        R, t, _, _ = self._chain(q, 2)
        return R @ self.p[2] + t

    def reach(self, x_des, frac=0.97):
        """x_des pulled toward the hip-pitch axis point until it is within `frac` of the straight leg (an unreachable swing target would make the
        IK flail the abduction joint)."""
        h = self.p[1]
        d = x_des - h
        lmax = frac * np.linalg.norm(self.foot0 - h)
        n = np.linalg.norm(d)
        return x_des if n <= lmax else h + d * lmax / n

    def ik(self, x_des, q0, iters=6, lam=1e-6, step=None):
        q = np.array(q0, float)
        for _ in range(iters):
            x, J = self.fk_jac(q)
            dq = np.linalg.solve(J.T @ J + lam * np.eye(3), J.T @ (x_des - x))
            q += dq if step is None else np.clip(dq, -step, step)
        return np.clip(q, self.q_lo, self.q_hi)


@dataclass
class Link:
    leg: int  # index into LEGS, -1 = trunk
    upto: int  # how many of its leg's joints lie between deck_top and it (abad body 1, thigh 2, shank / foot 3)
    mass: float
    com: np.ndarray  # at q = 0, trunk frame
    inertia: np.ndarray  # about com, trunk axes at q = 0 (geom points + box own inertia, as Model.inertia)


@dataclass
class Model:
    mass: float
    com: np.ndarray
    inertia: np.ndarray
    legs: dict
    imu_R: np.ndarray
    leg_mass: float
    links: list = field(default_factory=list)
    armature: np.ndarray = field(default_factory=lambda: np.zeros(12))
    q_prev: dict = field(default_factory=dict)

    def feet(self, q):
        return np.array(
            [self.legs[l].fk_jac(q[3 * i : 3 * i + 3])[0] for i, l in enumerate(LEGS)]
        )

    def stand_q(self, frac):
        """12 joint targets: each foot raised straight up from its build-pose spot to `frac` of the hip-pitch-to-foot distance."""
        out = []
        for l in LEGS:
            L = self.legs[l]
            d = np.linalg.norm(L.foot0 - L.p[1])
            seed = np.array([0.0, -0.5 * L.bend, 1.0 * L.bend])
            out.append(
                L.ik(L.foot0 + np.array([0.0, 0.0, (1 - frac) * d]), seed, iters=20)
            )
        return np.concatenate(out)

    def leg_required(self, i, q3, qdd3, f_s):
        """Joint torque the leg i would need to carry its own motion with NO ground force: M(q) qdd + sum_k m_k Jv_k^T f_s (the second term is the
        links' weight plus the trunk's acceleration, f_s = the IMU's specific force in the trunk frame). Coriolis terms are left out (the
        estimator vetoes contact for feet that are high above the ground, where the leg moves fast). Body rotation rates are not in it."""
        L = self.legs[LEGS[i]]
        M = np.diag(self.armature[3 * i : 3 * i + 3]).astype(float)
        g = np.zeros(3)
        for lk in self.links:
            if lk.leg != i:
                continue
            R, t, ws, ps = L._chain(q3, lk.upto)
            com = R @ lk.com + t
            Jv, Jw = np.zeros((3, 3)), np.zeros((3, 3))
            for j in range(lk.upto):
                Jv[:, j] = np.cross(ws[j], com - ps[j])
                Jw[:, j] = ws[j]
            Iw = R @ lk.inertia @ R.T
            M += lk.mass * Jv.T @ Jv + Jw.T @ Iw @ Jw
            g += lk.mass * Jv.T @ f_s
        return M @ qdd3 + g

    def props(self, q):
        """(mass, com (3,), inertia (3,3) about com) of the whole dog at joint angles q, trunk frame, from the links."""
        parts = []
        for lk in self.links:
            if lk.leg < 0:
                R, t = np.eye(3), np.zeros(3)
            else:
                R, t, _, _ = self.legs[LEGS[lk.leg]]._chain(
                    q[3 * lk.leg : 3 * lk.leg + 3], lk.upto
                )
            parts.append((lk.mass, R @ lk.com + t, R @ lk.inertia @ R.T))
        m = sum(p[0] for p in parts)
        com = sum(p[0] * p[1] for p in parts) / m
        I = np.zeros((3, 3))
        for mi, ci, Ii in parts:
            d = ci - com
            I += Ii + mi * (d @ d * np.eye(3) - np.outer(d, d))
        return m, com, I


def _poses(root_body):
    """Every body under deck_top -> (R, t) in the trunk frame; also its ancestors' leg joints (name -> joint element) and its geoms."""
    out = []

    def walk(b, R, t, joints):
        j = list(joints) + [(e.get("name"), e, R, t) for e in b.findall("joint")]
        out.append((b, R, t, j))
        for c in b.findall("body"):
            Rc = R @ quat_R(vec(c.get("quat"), [1, 0, 0, 0]))
            tc = R @ vec(c.get("pos"), [0, 0, 0]) + t
            walk(c, Rc, tc, j)

    walk(root_body, np.eye(3), np.zeros(3), [])
    return out


def from_build(xml):
    root = ET.parse(xml).getroot()
    dev = json.loads(root.find("custom/text[@name='napl.devices']").get("data"))
    qdd = {q["name"]: q for q in dev["qdds"]}
    trunk = next(
        b for b in root.find("worldbody").findall("body") if b.get("name") == ROOT
    )
    bodies = _poses(trunk)
    byname = {b.get("name"): (b, R, t) for b, R, t, _ in bodies}
    legs, arm = {}, np.zeros(12)
    for li, leg in enumerate(LEGS):
        w, p, lo, hi, tmax, kpm, kdm = [], [], [], [], [], [], []
        for ji, jn in enumerate(JOINTS):
            body = next(
                b
                for b, _, _, js in bodies
                if any(n == f"{leg}:{jn}_j" for n, *_ in js)
                and b.find(f"joint[@name='{leg}:{jn}_j']") is not None
            )
            _, R, t = byname[body.get("name")]
            e = body.find(f"joint[@name='{leg}:{jn}_j']")
            ax = R @ vec(e.get("axis"), [0, 0, 1])
            w.append(ax / np.linalg.norm(ax))
            p.append(R @ vec(e.get("pos"), [0, 0, 0]) + t)
            r = [float(v) for v in e.get("range").split()]
            lo.append(r[0]); hi.append(r[1])  # fmt: skip
            d = qdd[f"{leg}:{jn}"]
            tmax.append(d["tau_ff_max"]); kpm.append(d["kp_max"]); kdm.append(d["kd_max"])  # fmt: skip
            arm[3 * li + ji] = d["motor"]["rotor_inertia"] * d["gear_ratio"] ** 2
        fb, _, tf = byname[f"{leg}:foot_ball"]
        g = fb.find("geom[@type='sphere']")
        if np.linalg.norm(vec(g.get("pos"), [0, 0, 0])) > 1e-3:
            raise ValueError(
                f"{leg}:foot_ball geom is not centred on its body: the FK point must be the ball centre"
            )
        legs[leg] = Leg(leg, np.array(w), np.array(p), tf.copy(), float(g.get("size").split()[0]), np.array(lo), np.array(hi),
                        np.array(tmax), np.array(kpm), np.array(kdm))  # fmt: skip
    # mass points per link
    acc = {}
    for b, R, t, js in bodies:
        key = (-1, 0)
        lj = [n for n, *_ in js if n and n.split(":")[0] in LEGS and n.endswith("_j")]
        if lj:
            key = (LEGS.index(lj[0].split(":")[0]), len(lj))
        for g in b.findall("geom"):
            m = float(g.get("mass", 0))
            if m <= 0:
                continue
            Rg = R @ quat_R(vec(g.get("quat"), [1, 0, 0, 0]))
            c = R @ vec(g.get("pos"), [0, 0, 0]) + t
            Iown = np.zeros((3, 3))
            if g.get("type") == "box":
                a, bb, cc = (float(v) for v in g.get("size").split())
                Iown = (
                    Rg
                    @ np.diag(
                        [
                            m * (bb**2 + cc**2) / 3,
                            m * (a**2 + cc**2) / 3,
                            m * (a**2 + bb**2) / 3,
                        ]
                    )
                    @ Rg.T
                )
            acc.setdefault(key, []).append((m, c, Iown))
    links = []
    for (leg, upto), items in sorted(acc.items()):
        m = sum(i[0] for i in items)
        com = sum(i[0] * i[1] for i in items) / m
        I = sum(
            Io
            + mi * ((ci - com) @ (ci - com) * np.eye(3) - np.outer(ci - com, ci - com))
            for mi, ci, Io in items
        )
        links.append(Link(leg, upto, m, com, I))
    # IMU: the site of imus[0] (its frame rotation in the trunk frame)
    site = dev["imus"][0]["site"]
    imu_R = np.eye(3)
    for b, R, t, _ in bodies:
        s = b.find(f"site[@name='{site}']")
        if s is not None:
            imu_R = R @ quat_R(vec(s.get("quat"), [1, 0, 0, 0]))
    model = Model(0.0, np.zeros(3), np.eye(3), legs, imu_R, float(np.mean([sum(l.mass for l in links if l.leg == i) for i in range(4)])),
                  links, arm)  # fmt: skip
    # bend: the knee direction that puts the knee behind (fl, fr) / ahead of (rl, rr) the hip pitch point
    for leg, L in legs.items():
        d = np.linalg.norm(L.foot0 - L.p[1])
        good = None
        for s in (1.0, -1.0):
            Ls = Leg(**{**L.__dict__, "bend": s})
            q = Ls.ik(
                L.foot0 + np.array([0.0, 0.0, 0.18 * d]),
                np.array([0.0, -0.5 * s, s]),
                iters=20,
            )
            behind = Ls.knee(q)[0] < L.p[1][0]
            inside = np.all(
                (q > L.q_lo + 1e-9) & (q < L.q_hi - 1e-9)
            )  # clipped to a joint stop = the wrong side of the elbow
            if behind == leg.startswith("f") and inside:
                good = s
        if good is None:
            raise ValueError(f"{leg}: neither knee direction bends the designed way")
        L.bend = good
    model.mass, model.com, model.inertia = model.props(model.stand_q(0.92))
    return model
