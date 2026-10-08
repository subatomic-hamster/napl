"""Programmable drones: a position / velocity setpoint controller for any multirotor design (napl-native, no PX4).

Why not PX4 SITL multi-instance? `napl program`'s PX4 interface owns one World per bridge in wall-clock lockstep with one PX4 process
(`px4/bridge_*.toml`); five instances would need five Worlds or a multi-vehicle bridge, ~6 s of boot each and real-time pacing, and
nothing would make the run replayable as one World. The napl-native controller below (the one `Design.simulate("hover")` uses, grown
into a setpoint tracker) drives only the ESC throttle inputs through the command bus, so a whole swarm is ONE deterministic, replayable
World. The drone *program* (examples/cuas/swarm.py) sees a small API: `Drone.goto(pos, vel, yaw)`.

    cal = calibrate("x500v2", workdir)                 # rotor thrust curves, torque ratio, mass, inertia (JSON-able dict, cached)
    f = Flyer(sim, cal, paths, dt_ctl)                 # paths: prim paths of one vehicle in the composed scene
    f.target(pos, vel, yaw); f.update()                # every control step: reads state, sets 4 throttles

Control law (as simulate.hover with hold_xy, plus velocity feed-forward): PD(+I) on position -> desired thrust vector (tilted at most
`max_tilt` from vertical) -> geometric attitude target with fixed yaw -> PD(+I) attitude -> torque; a mixer built from the model's own rotor
positions / axes / spin directions allocates thrust + torques to rotors; per-rotor thrust-vs-throttle tables come from a short calibration on a
fork. Ground truth state (no estimator): this is a flight-control stand-in, not an avionics model. Inertia is approximated by point masses of
the instances (a controller scaling only: the loop is stable for +-50 % errors; the plant is MuJoCo).
"""

import json
import math
from pathlib import Path

import numpy as np

from .. import Sim, design
from ..design.simulate import _R, G, _model, _Rz, _vee

CAL_TICKS = 150  # ticks each throttle level is held in the calibration
WN_Z, ZETA_Z = 3.0, 1.0  # altitude loop (rad/s)
WN_XY, ZETA_XY = 1.8, 1.0  # horizontal position loop, slower than the attitude loop
WN_R, ZETA_R, KI_R = (
    12.0,
    1.0,
    10.0,
)  # attitude loop: slower than simulate.hover's 20 rad/s, which limit-cycles an x500 in turbulence (motor lag)
WN_YAW = 6.0


def calibrate(assembly, workdir, parts=None, altitude=60.0) -> dict:
    """Build `assembly` alone, lift it `altitude` m above the ground (no ground effect), hold each throttle level for CAL_TICKS on a fork and
    record thrust / torque / spin speed per rotor. Everything the controller needs, in the root body frame; cached in workdir/cal_<name>.json."""
    from ..cuas.scene import REPO, _asm_path

    work = Path(workdir)
    (work / "cal").mkdir(parents=True, exist_ok=True)
    cache = work / f"cal_{Path(str(assembly)).stem}.json"
    if cache.exists():
        return json.loads(cache.read_text())
    text, report = design.compute(
        _asm_path(assembly), parts or REPO / "parts", work / "cal"
    )
    xml = work / "cal" / "vehicle.xml"
    xml.write_text(text)
    dev, paths, joints, dt, root = _model(xml)
    rotors, escs = dev["rotors"], {e["joint"]: e for e in dev["escs"]}
    sim = Sim.load(xml)
    q = lambda p, a, fr="": sim.query(p, a, fr)[0]
    rp = paths[root]
    free = joints[f"{root}:free"][0]
    sim.set(
        free, "qpos", [0.0, 0.0, altitude + float(q(rp, "pos")[2]), 1.0, 0.0, 0.0, 0.0]
    )
    n = len(rotors)
    esc_p = [f"/devices/{escs[r['joint']]['name']}" for r in rotors]
    rot_p = [f"/devices/{r['name']}" for r in rotors]
    dirs = np.array([float(r.get("dir", 1)) for r in rotors])
    pos = np.array([q(paths[r["body"]], "pos", rp) for r in rotors])
    ax = np.array(
        [
            _R(q(paths[r["body"]], "quat", rp))
            @ np.array(joints[r["joint"]][1].split(), float)
            for r in rotors
        ]
    )
    items = [
        (i["mass"], np.array(q(paths[i["body"]], "pos", rp)))
        for i in report["instances"]
        if i.get("body") and i["mass"] > 0
    ]
    msum = sum(m for m, _ in items)
    com = sum(m * p for m, p in items) / msum
    mass = float(report["mass"]["total"])
    d2 = np.array(
        [
            sum(m * np.sum(np.delete(p - com, k) ** 2) for m, p in items)
            for k in range(3)
        ]
    )
    inertia = np.maximum(d2 * mass / msum, mass * 0.02**2)
    f = sim.fork()
    levels = np.concatenate([[0.0], np.linspace(0.1, 1.0, 10)])
    T, Q, W = (np.zeros((len(levels), n)) for _ in range(3))
    for li, u in enumerate(levels):
        for p in esc_p:
            f.set(p, "throttle", float(u))
        f.step(CAL_TICKS)
        T[li] = [
            q_ * d for q_, d in zip((f.query(p, "thrust")[0][0] for p in rot_p), dirs)
        ]
        Q[li] = [abs(f.query(p, "torque")[0][0]) for p in rot_p]
        W[li] = [f.query(p, "omega")[0][0] for p in rot_p]
    T = np.maximum.accumulate(np.maximum(T, 0.0), axis=0)
    ok = T[-1] > 1e-6
    kq = float(np.mean(Q[-4:, ok].sum(axis=1) / T[-4:, ok].sum(axis=1)))
    cal = dict(assembly=str(assembly), mass=mass, inertia=inertia.tolist(), com=com.tolist(), pos=pos.tolist(), ax=ax.tolist(),
               dirs=dirs.tolist(), levels=levels.tolist(), T=T.tolist(), omega=W.tolist(), kq=kq, dt=dt,
               rotor_joints=[r["joint"] for r in rotors], max_thrust=float(T[-1].sum()))  # fmt: skip
    cache.write_text(json.dumps(cal))
    return cal


class Flyer:
    """One drone's controller. `paths`: dict(root=<prim of the root body>, free=<prim of its free joint>, escs=[/devices/..], yaw0=rad)."""

    def __init__(
        self,
        sim,
        cal,
        paths,
        dt_ctl,
        max_tilt=0.5,
        vmax=8.0,
        wn_r=WN_R,
        zeta_r=ZETA_R,
        ki_r=KI_R,
    ):
        self.wn_r, self.zeta_r, self.ki_r = wn_r, zeta_r, ki_r
        self.sim, self.cal, self.p, self.dtc = sim, cal, paths, dt_ctl
        self.mass, self.inertia = cal["mass"], np.array(cal["inertia"])
        self.T, self.levels = np.array(cal["T"]), np.array(cal["levels"])
        pos, ax, dirs = np.array(cal["pos"]), np.array(cal["ax"]), np.array(cal["dirs"])
        t = dirs[:, None] * ax
        arm = pos - np.array(cal["com"])
        n = len(pos)
        B = np.zeros((4, n))
        for i in range(n):
            B[0, i] = t[i][2]
            B[1:, i] = np.cross(arm[i], t[i]) - cal["kq"] * ax[i]
        self.Bp = np.linalg.pinv(B)
        self.n = n
        self.max_tilt, self.vmax = max_tilt, vmax
        self.iz, self.ir, self.ixy = 0.0, np.zeros(3), np.zeros(2)
        self.yaw = paths.get("yaw0", 0.0)
        self.sp = (np.zeros(3), np.zeros(3))
        self.alive = True
        self.last = None  # (p, v, R, tilt, throttles) of the last update

    def target(self, pos, vel=(0.0, 0.0, 0.0), yaw=None):
        self.sp = (np.asarray(pos, float), np.asarray(vel, float))
        if yaw is not None:
            self.yaw = float(yaw)

    def hover_omega(self):
        """Spin speed per rotor (rad/s, along the joint axis) at the throttle that carries the weight: for starting a drone in the air."""
        w = self.mass * G / self.n
        return [
            float(np.interp(w, self.T[:, i], np.array(self.cal["omega"])[:, i]))
            for i in range(self.n)
        ]

    def state(self):
        q = lambda p, a: self.sim.query(p, a)[0]
        return (
            q(self.p["root"], "pos"),
            _R(q(self.p["root"], "quat")),
            q(self.p["free"], "qvel"),
        )

    def update(self):
        p, R, v6 = self.state()
        pd, vd = self.sp
        if (
            not self.alive
        ):  # cut: keep reporting the state (it is still falling), command nothing
            self.last = (
                p,
                v6[:3].copy(),
                R,
                math.acos(max(-1.0, min(1.0, R[2, 2]))),
                [0.0] * self.n,
            )
            return
        dtc, m = self.dtc, self.mass
        kz, kvz = WN_Z**2, 2 * ZETA_Z * WN_Z
        kxy, kvxy = WN_XY**2, 2 * ZETA_XY * WN_XY
        e = p - pd
        self.iz = float(np.clip(self.iz - e[2] * dtc * 1.0, -4.0, 4.0))
        self.ixy = np.clip(self.ixy - e[:2] * dtc * 0.3, -3.0, 3.0)
        ev = v6[:3] - vd
        a = np.array(
            [
                -kxy * e[0] - kvxy * ev[0] + self.ixy[0],
                -kxy * e[1] - kvxy * ev[1] + self.ixy[1],
                -kz * e[2] - kvz * ev[2] + self.iz,
            ]
        )
        fv = m * np.array([a[0], a[1], G + a[2]])
        fv[2] = max(fv[2], 0.2 * m * G)
        fh = np.linalg.norm(fv[:2])
        if fh > fv[2] * math.tan(self.max_tilt):
            fv[:2] *= fv[2] * math.tan(self.max_tilt) / fh
        Rd = _Rz(self.yaw)
        b3 = fv / np.linalg.norm(fv)
        b2 = np.cross(b3, Rd[:, 0])
        b2 /= np.linalg.norm(b2)
        Rt = np.column_stack([np.cross(b2, b3), b2, b3])
        fz = float(fv @ R[:, 2])
        eR = 0.5 * _vee(Rt.T @ R - R.T @ Rt)
        self.ir = np.clip(self.ir + eR * dtc, -0.2, 0.2)
        WN_R, ZETA_R, KI_R = self.wn_r, self.zeta_r, self.ki_r
        kr, kw = WN_R**2, 2 * ZETA_R * WN_R
        kr_v = np.array([kr, kr, WN_YAW**2])
        kw_v = np.array([kw, kw, 2 * ZETA_R * WN_YAW])
        ki_v = np.array([KI_R, KI_R, KI_R * (WN_YAW / WN_R) ** 2])
        tau = self.inertia * (-kr_v * eR - kw_v * v6[3:6] - ki_v * self.ir)
        Ti = np.clip(self.Bp @ np.array([fz, *tau]), 0.0, self.T[-1])
        u = [float(np.interp(Ti[i], self.T[:, i], self.levels)) for i in range(self.n)]
        for pth, ui in zip(self.p["escs"], u):
            self.sim.set(pth, "throttle", ui)
        tilt = math.acos(max(-1.0, min(1.0, R[2, 2])))
        self.last = (p, v6[:3].copy(), R, tilt, u)

    def cut(self):
        """The controller gives up (the vehicle is lost): throttles to zero."""
        self.alive = False
        for pth in self.p["escs"]:
            self.sim.set(pth, "throttle", 0.0)
