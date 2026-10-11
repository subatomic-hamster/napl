"""Sim-side I/O implementations (spec section 4): clocks, the napl plant behind SimRP2040, SimCamera, TruthDetector, the scene builder for the
placeholder turret + hanging targets, and the lockstep harness.

PLACEHOLDER SCENE. Turret kinematics come from the existing assemblies/turret.toml (2 x XL430 servos as pan / tilt, the pellet launcher device,
the Pi camera pose device) because the cuas-lite assembly (cad / parts workers) does not exist yet. build_scene() overrides the launcher's ballistic
constants with BALL (22.2 mm, 1.86 g, Cd 0.47, v0 30 m/s, sigma 1.5 m/s, spread 10 mrad) and adds targets as MJCF pendulums (ball joint, no code
moves them: deterministic kernel physics). When the real assembly exists, build_scene() keeps the same names: /devices/turret_pan, _tilt, _gun, _cam.

    sc = build_scene(out_dir, [Balloon("b1", (8, 0, 2.0))])
    run = Lockstep(sc, run_dir, targets=[...])    # World + SimRP2040 + SimCamera + TruthDetector + App
    run.run(40.0)
"""

import json
import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..cuas.camera import Camera as PoseCamera
from ..cuas.camera import Frame as PoseFrame
from ..cuas.scene import Environment, Vehicle, _bodies_dfs, _paths, compose
from . import aim as aimlib
from .io import Det, DetResult, Frame, Intrinsics

BRIO_FOV_V = 0.911  # rad (52.2 deg)
BRIO = Intrinsics.from_fov(1280, 720, BRIO_FOV_V)


# ---- clocks -----------------------------------------------------------------------------------------------------------------------------
class ManualClock:
    def __init__(self, t=0.0):
        self.t = t

    def now(self):
        return self.t

    def sleep_until(self, t):
        raise RuntimeError("lockstep clock: the harness drives time")


class SimClock:
    """now() = world.t (sim time)."""

    def __init__(self, world):
        self.w = world

    def now(self):
        return self.w.t

    def sleep_until(self, t):
        raise RuntimeError("lockstep clock: the harness drives time")


# ---- the napl plant (what SimRP2040 drives) -----------------------------------------------------------------------------------------------
class NaplPlant:
    """SimRP2040's plant over a napl World: joint goals are `goal_position` on the pan / tilt servo devices with goal_speed 0 (the twin's planner is the
    profile), FIRE pulses `fire` on the launcher. estop_open / key_on are the GPIO inputs the harness (or a scenario fault) sets."""

    def __init__(self, world, p_gain=1500):
        self.w, self.estop_open, self.key_on = world, False, True
        self._goal = None
        for s in (world.pan, world.tilt):
            world.sim.set(s, "goal_speed", 0.0)
            world.sim.set(s, "p_gain", p_gain)
            world.sim.set(s, "torque_enable", 0.0)
        self.duty = 0.0
        self.pulses = 0

    def joint_deg(self):
        pan, tilt = self.w.pan_tilt()
        return math.degrees(pan), math.degrees(tilt)

    def set_joint_goal(self, pan_deg, tilt_deg):
        g = (round(pan_deg, 6), round(tilt_deg, 6))
        if g != self._goal:
            self._goal = g
            self.w.aim(math.radians(g[0]), math.radians(g[1]))

    def set_torque(self, on):
        for s in (self.w.pan, self.w.tilt):
            self.w.sim.set(s, "torque_enable", 1.0 if on else 0.0)

    def set_flywheel(self, duty):
        self.duty = duty  # ponytail: the launcher has no runtime muzzle-velocity attr; v0 stays the device config (needs a kernel attr to follow the flywheel)

    def fire_pulse(self):
        self.pulses += 1
        self.w.sim.set(self.w.gun, "fire", 1.0)


# ---- scene ------------------------------------------------------------------------------------------------------------------------------
@dataclass
class Balloon:
    """A target hanging on a string from `anchor` (world), a 3 g sphere (size_m diameter) or a disc, swinging `swing_deg` toward azimuth `swing_az`."""

    id: str
    anchor: tuple
    length: float = 2.0
    size_m: float = 0.30
    swing_deg: float = 0.0
    swing_az: float = 90.0
    kind: str = "balloon"  # balloon (sphere) | target (disc facing the turret)
    cls: str = None  # detector class (default = kind)

    @property
    def klass(self):
        return self.cls or self.kind


def _pend_pose(t: Balloon):
    a, az = math.radians(t.swing_deg), math.radians(t.swing_az)
    u = np.array([math.cos(az), math.sin(az), 0.0])
    ax = np.array([math.sin(az), -math.cos(az), 0.0])
    q = [math.cos(a / 2), *(ax * math.sin(a / 2))]
    center = (
        np.array(t.anchor)
        + t.length * math.sin(a) * u
        + np.array([0, 0, -t.length * math.cos(a)])
    )
    return center, q


def build_scene(out, targets=(), mast=1.0, env=None, seed=1, v0=aimlib.V0_NOMINAL):
    """compose() the turret, override the launcher with BALL, add the pendulum targets. -> manifest dict (model, body_paths ...)."""
    out = Path(out)
    scene = compose(
        out,
        [Vehicle("turret", "turret", pos=(0.0, 0.0, mast), pedestal=mast)],
        env or Environment(),
        seed=seed,
    )
    model = Path(scene["model"])
    root = ET.parse(model).getroot()
    txt = root.find("custom/text[@name='napl.devices']")
    devs = json.loads(txt.get("data"))
    man = scene["manifest"]
    for g in devs["launchers"]:
        g.update(muzzle_velocity=v0, v_std=aimlib.V0_SIGMA, spread_std=aimlib.SPREAD_SIGMA, pellet_mass=aimlib.BALL["m"], diameter=aimlib.BALL["d"],
                 cd=aimlib.BALL["cd"], max_time=3.0)  # fmt: skip
        man["device_cfg"][g["name"]] = g
    txt.set("data", json.dumps(devs, sort_keys=True, separators=(",", ":")))
    wb = root.find("worldbody")
    r_ball = aimlib.BALL_RADIUS
    for t in targets:
        c, q = _pend_pose(t)
        b = ET.SubElement(
            wb,
            "body",
            name=t.id,
            pos=" ".join(f"{x:.9g}" for x in c),
            quat=" ".join(f"{x:.9g}" for x in q),
        )
        ET.SubElement(
            b,
            "joint",
            name=f"{t.id}_j",
            type="ball",
            pos=f"0 0 {t.length:.9g}",
            damping="0.0002",
        )
        r = (
            t.size_m / 2 + r_ball
        )  # the launcher casts a zero-radius pellet: inflate the target by the ball radius (spec 4.2)
        if t.kind == "balloon":
            ET.SubElement(b, "geom", name=f"{t.id}_g", type="sphere", size=f"{r:.9g}", mass="0.003", contype="0", conaffinity="0", group="2",
                          rgba="0.9 0.2 0.2 1")  # fmt: skip
        else:
            ET.SubElement(b, "geom", name=f"{t.id}_g", type="cylinder", size=f"{r:.9g} 0.005", euler="0 1.5707963 0", mass="0.05", contype="0",
                          conaffinity="0", group="2", rgba="0.95 0.8 0.1 1")  # fmt: skip
        ET.SubElement(b, "geom", name=f"{t.id}_string", type="capsule", fromto=f"0 0 0 0 0 {t.length:.9g}", size="0.001", mass="0", contype="0",
                      conaffinity="0", group="5", rgba="0.8 0.8 0.8 1")  # fmt: skip
    ET.indent(root)
    model.write_text(ET.tostring(root, encoding="unicode"))
    man["body_names"] = _bodies_dfs(root)
    man["body_paths"] = _paths(root)
    (out / "scene.json").write_text(json.dumps(man, indent=1))
    return man


def measure_geometry(world, plant, settle=120):
    """Head geometry of the placeholder turret by posing it: muzzle and camera at tilt 0 and 0.3 rad give the tilt pivot (spec 5.1 frame B: origin on
    the pan axis at the vehicle position, pan 0 = +x). Leaves the head at (0, 0)."""
    base = np.array(world.man["vehicles"]["turret"]["pos"], float)
    plant.set_torque(True)

    def pose_at(tilt):
        world.aim(0.0, tilt)
        world.step(settle)
        m, _ = world.muzzle()
        return m - base, np.array(world.sim.query(world.cam_path, "pos")[0]) - base

    m0, c0 = pose_at(0.0)
    m1, _ = pose_at(0.3)
    world.aim(0.0, 0.0)
    world.step(
        settle
    )  # torque stays on: the head holds level until the twin takes over at IDLE
    ph = 0.3
    rm = np.array(
        [[math.cos(ph), -math.sin(ph)], [math.sin(ph), math.cos(ph)]]
    )  # raises x toward z
    pv = np.linalg.solve(
        np.eye(2) - rm, np.array([m1[0], m1[2]]) - rm @ np.array([m0[0], m0[2]])
    )
    return aimlib.Geometry(
        h_t=float(pv[1]),
        l_m=float(m0[0]),
        d_b=float(m0[2] - pv[1]),
        l_c=float(c0[0]),
        d_c=float(c0[2] - pv[1]),
    )


# ---- camera + detector ------------------------------------------------------------------------------------------------------------------
class SimCamera:
    """Samples the World every 1/60 s of sim time (at the next control tick), delivers each frame `latency` s later with t_cap = the sample time.
    rgb is None; meta carries the ground-truth snapshot `truth_fn(t)` returns (camera pose + targets in B) for the TruthDetector."""

    def __init__(self, clock, truth_fn, intrinsics=BRIO, fps=60.0, latency=0.030):
        self.clock, self.truth_fn, self.intrinsics, self.period, self.latency = (
            clock,
            truth_fn,
            intrinsics,
            1.0 / fps,
            latency,
        )
        self.next_t = 0.0
        self.q = deque()
        self.seq = 0

    def read(self, timeout=0.1):
        now = self.clock.now()
        if now >= self.next_t - 1e-9:
            self.next_t += self.period
            self.seq += 1
            k = self.intrinsics
            self.q.append(
                (
                    now + self.latency,
                    Frame(self.seq, now, k.w, k.h, None, self.truth_fn(now)),
                )
            )
        out = None
        while self.q and self.q[0][0] <= now + 1e-9:
            out = self.q.popleft()[1]  # newest ready frame wins
        return out

    def close(self):
        pass


class TruthDetector:
    """In-process stand-in detector: projects the truth snapshot of a frame into pixel boxes with the spec 7.2 noise model (bbox centre jitter 1.5 px,
    size jitter 4 %, p_miss = 0.02 + 0.03 R/5 m, false positives 0.01/frame, person conf N(0.75, 0.1), latency 15 ms). Seeded. `drop` = reply loss
    probability, `outages` = [(t0, t1)] windows with no replies (scenario s8)."""

    def __init__(
        self,
        clock,
        intrinsics=BRIO,
        seed=0,
        latency=0.015,
        drop=0.0,
        outages=(),
        noise=True,
    ):
        self.clock, self.k, self.latency, self.drop, self.outages, self.noise = (
            clock,
            intrinsics,
            latency,
            drop,
            list(outages),
            noise,
        )
        self.rng = np.random.default_rng(seed)
        self.cam = PoseCamera(
            None,
            "",
            2 * math.atan(0.5 * intrinsics.h / intrinsics.fy),
            intrinsics.w,
            intrinsics.h,
        )
        self.q = deque()
        self.last_reply_t = None
        self.popped = set()  # ids no longer reported (a popped balloon is gone)
        self.mutate = None  # optional fn(dets) -> dets applied to every reply (tests inject persons / low confidence)

    @property
    def stale_s(self):
        return (
            math.inf
            if self.last_reply_t is None
            else self.clock.now() - self.last_reply_t
        )

    def submit(self, frame):
        now = self.clock.now()
        if any(a <= now < b for a, b in self.outages) or self.rng.random() < self.drop:
            return
        self.q.append(
            (
                now + self.latency,
                DetResult(
                    frame.seq,
                    self.latency * 1000,
                    (self.mutate or (lambda d: d))(self._dets(frame.meta)),
                    0.0,
                ),
            )
        )

    def poll(self):
        now, out = self.clock.now(), None
        while self.q and self.q[0][0] <= now + 1e-9:
            out = self.q.popleft()[1]
        if out is not None:
            out.t_rx, self.last_reply_t = now, now
        return out

    def _dets(self, meta):
        rng, k, n = self.rng, self.k, self.noise
        pf = PoseFrame(
            None,
            None,
            np.asarray(meta["cam_pos"]),
            np.asarray(meta["cam_fwd"]),
            np.asarray(meta["cam_up"]),
            0.0,
        )
        dets = []
        for t in meta["targets"]:
            if t["id"] in self.popped:
                continue
            pr = self.cam.project(t["pos"], pf)
            if pr is None:
                continue
            u, v, z = pr
            w_m, h_m = t.get("w", t["size"]), t.get("h", t["size"])
            w, h = k.fx * w_m / z, k.fy * h_m / z
            if not (0 <= u < k.w and 0 <= v < k.h):
                continue
            if n:
                if t["cls"] != "person" and rng.random() < 0.02 + 0.03 * (z / 5.0):
                    continue
                u, v = u + rng.normal(0, 1.5), v + rng.normal(0, 1.5)
                s = 1 + rng.normal(0, 0.04)
                w, h = w * s, h * s
                conf = float(
                    np.clip(
                        rng.normal(0.75, 0.1)
                        if t["cls"] == "person"
                        else rng.normal(0.85, 0.07),
                        0.3,
                        0.99,
                    )
                )
            else:
                conf = 0.9
            dets.append(
                Det(t["cls"], conf, (u - w / 2, v - h / 2, u + w / 2, v + h / 2))
            )
        if n and rng.random() < 0.01:
            u, v, s = (
                rng.uniform(50, k.w - 50),
                rng.uniform(50, k.h - 50),
                rng.uniform(8, 20),
            )
            dets.append(
                Det(
                    "drone",
                    float(rng.uniform(0.5, 0.9)),
                    (u - s / 2, v - s / 2, u + s / 2, v + s / 2),
                )
            )
        return dets


# ---- lockstep harness --------------------------------------------------------------------------------------------------------------------
class Lockstep:
    """One process, deterministic: World(ctl_ticks=5) + NaplPlant + SimRP2040(LoopbackLink) + SimCamera + TruthDetector + App. Per 5 ms control period:
    twin.step(now), app.tick(now), world.step(1)."""

    def __init__(
        self,
        manifest,
        run_dir,
        targets,
        app_cfg=None,
        sector=None,
        seed=0,
        detector_kw=None,
        drop_link=None,
    ):
        from ..cuas.world import World
        from .app import App, AppCfg
        from .gimbal import SerialGimbal
        from .sim_mcu import LoopbackLink, SimRP2040

        self.targets = {t.id: t for t in targets}
        self.world = w = World(manifest["model"], run_dir, seed=seed or 1, ctl_ticks=5)
        self.base = np.array(w.man["vehicles"]["turret"]["pos"], float)
        self.clock = SimClock(w)
        self.plant = NaplPlant(w)
        self.link = LoopbackLink(self.clock)
        self.twin = SimRP2040(self.plant, self.link)
        self.geom = measure_geometry(w, self.plant)
        self.twin.step(w.t)
        self.camera = SimCamera(self.clock, self._truth)
        self.camera.next_t = w.t
        self.detector = TruthDetector(self.clock, seed=seed, **(detector_kw or {}))
        self.gimbal = SerialGimbal(self.link, self.clock, session=0x5EED0000 + seed)
        cfg = app_cfg or AppCfg()
        cfg.auto_home = True
        self.app = App(
            self.camera,
            self.detector,
            self.gimbal,
            self.clock,
            cfg,
            geom=self.geom,
            sector=sector,
        )
        self.script = []  # [(t, fn(harness))] one-shot operator actions, sorted by t
        self.hold_deadman = False
        self._next_dm = 0.0
        self.popped = {}
        self.fire_log = []  # (t, n_pulses) from the plant

    def _truth(self, t):
        w, q = self.world, self.world.sim.query
        tg = []
        for tr in self.targets.values():
            p = q(w.body_path[w.man["body_names"].index(tr.id)], "pos")[0] - self.base
            tg.append(dict(id=tr.id, cls=tr.klass, pos=p, size=tr.size_m))
        return dict(
            cam_pos=q(w.cam_path, "pos")[0] - self.base,
            cam_fwd=q(w.cam_path, "fwd")[0],
            cam_up=q(w.cam_path, "up")[0],
            targets=tg,
        )

    def run(self, seconds, until=None):
        w, app = self.world, self.app
        n = int(round(seconds / w.dtc))
        seen = 0
        for _ in range(n):
            now = w.t
            while self.script and self.script[0][0] <= now:
                self.script.pop(0)[1](self)
            if self.hold_deadman and now >= self._next_dm:
                app.deadman()
                self._next_dm = now + 0.2
            self.twin.step(now)
            app.tick(now)
            w.step(1)
            for ev in w.events[seen:]:
                if ev.get("type") == "hit":
                    name = ev["body"].rsplit("/", 1)[-1]
                    tr = self.targets.get(name)
                    if (
                        tr and name not in self.popped and ev["speed"] >= 8.0
                    ):  # balloon: any hit at >= 8 m/s pops it (spec 7.3)
                        self.popped[name] = w.t
                        self.detector.popped.add(name)
            seen = len(w.events)
            if until and until(self):
                break
        return self

    def finish(self):
        self.world.finish()
        return json.loads((Path(self.world.run_dir) / "native.json").read_text())
