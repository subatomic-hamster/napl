"""Sim-side I/O implementations (spec section 4): clocks, the napl plant behind SimRP2040, SimCamera, TruthDetector, the scene builder for the
placeholder turret + hanging targets, and the lockstep harness.

SCENE. The turret is assemblies/cuas-lite.toml (CAD meshes, NEMA 17 + TMC2209 stepper-as-servo pan / tilt, the foam launcher at the muzzle, the Brio
camera pose device); names /devices/turret_pan, _tilt, _gun, _cam. build_scene() overrides the launcher's ballistic constants with BALL (22.2 mm,
1.86 g, Cd 0.47, v0 30 m/s, sigma 1.5 m/s, spread 10 mrad) and adds targets as MJCF pendulums (ball joint, no code moves them: deterministic kernel
physics). measure_geometry() re-measures the head geometry off the built scene (h_t 202 mm, muzzle 130 / -52.5 mm, Brio 122 / -11 mm).

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

    def __init__(self, world, p_gain=10000):
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


@dataclass
class Mover:
    """A kinematic target (balloon drifting with the wind, small quad, walking person): a free-joint body with gravcomp whose pose the harness sets
    from the closed form below every control period (SetAttr qpos / qvel: logged, replayed, deterministic).

    position(t) = route(t) + sum_k amp_k (sin(2 pi t' / T_k + phi_k) - sin phi_k),  t' = max(t - t_start, 0)
    route = the polyline `route` flown at `speed`, ping-pong at the ends (or once: `once`; one point = hover; before t_start it waits at route[0]).
    `kind` picks the geometry: balloon (sphere, size_m), quad (flat box size_m x size_m x 0.06), person (capsule 0.5 x 1.7 m, origin at its centre)."""

    id: str
    route: tuple
    speed: float = 0.0
    waves: tuple = ()  # ((ax, ay, az), period_s, phase_rad)
    t_start: float = 0.0
    size_m: float = 0.30
    kind: str = "balloon"  # balloon | quad | person
    once: bool = False  # fly the route once and stop at its end instead of ping-pong
    cls: str = (
        None  # detector class (balloon -> balloon, quad -> drone, person -> person)
    )

    @property
    def klass(self):
        return self.cls or {"quad": "drone"}.get(self.kind, self.kind)

    @property
    def box_m(self):  # (w, h) the stand-in detector projects
        return (0.5, 1.7) if self.kind == "person" else (self.size_m, self.size_m)

    def pv(self, t):
        """-> (position, velocity) at sim time t."""
        r = np.array(self.route, float).reshape(-1, 3)
        tt = max(t - self.t_start, 0.0)
        pos, vel = r[0].copy(), np.zeros(3)
        if len(r) > 1 and self.speed > 0 and t >= self.t_start:
            seg = np.diff(r, axis=0)
            ln = np.linalg.norm(seg, axis=1)
            tot = ln.sum()
            m = self.speed * tt if self.once else (self.speed * tt) % (2 * tot)
            fwd = m <= tot
            d = m if fwd else 2 * tot - m
            d = min(d, tot)
            i = min(int(np.searchsorted(np.cumsum(ln), d, side="right")), len(ln) - 1)
            u = seg[i] / ln[i]
            pos = r[i] + u * (d - (np.cumsum(ln)[i] - ln[i]))
            vel = u * self.speed * (1 if fwd else -1) * (not (self.once and m >= tot))
        for a, per, ph in self.waves:
            a, w = np.array(a, float), 2 * math.pi / per
            pos = pos + a * (math.sin(w * tt + ph) - math.sin(ph))
            vel = vel + a * w * math.cos(w * tt + ph) * (t >= self.t_start)
        return pos, vel


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


def _add_mover(wb, t: Mover, r_ball):
    p0, _ = t.pv(0.0)
    b = ET.SubElement(
        wb, "body", name=t.id, pos=" ".join(f"{x:.9g}" for x in p0), gravcomp="1"
    )
    ET.SubElement(b, "joint", name=f"{t.id}_j", type="free")
    g = dict(
        name=f"{t.id}_g", contype="0", conaffinity="0", group="2"
    )  # inflated by the ball radius, as the pendulum targets (spec 4.2)
    if t.kind == "person":
        ET.SubElement(
            b,
            "geom",
            type="capsule",
            fromto="0 0 -0.6 0 0 0.6",
            size=f"{0.25 + r_ball:.9g}",
            mass="70",
            rgba="0.2 0.4 0.9 1",
            **g,
        )
    elif t.kind == "quad":
        h = t.size_m / 2 + r_ball
        ET.SubElement(
            b,
            "geom",
            type="box",
            size=f"{h:.9g} {h:.9g} {0.03 + r_ball:.9g}",
            mass="0.5",
            rgba="0.2 0.2 0.2 1",
            **g,
        )
    else:
        ET.SubElement(
            b,
            "geom",
            type="sphere",
            size=f"{t.size_m / 2 + r_ball:.9g}",
            mass="0.003",
            rgba="0.9 0.2 0.2 1",
            **g,
        )


def compose_base(out, mast=1.0):
    """The composed turret alone (the slow part of build_scene: tens of seconds), to be reused by build_scene(..., base=out) for many scenes."""
    return compose(
        out,
        [Vehicle("turret", "cuas-lite", pos=(0.0, 0.0, mast), pedestal=mast)],
        Environment(),
    )


def build_scene(
    out,
    targets=(),
    mast=1.0,
    env=None,
    seed=1,
    v0=aimlib.V0_NOMINAL,
    launcher=None,
    base=None,
):
    """compose() the turret, override the launcher with BALL, add the pendulum / Mover targets. `launcher` overrides launcher config keys
    (muzzle_velocity, v_std, spread_std, cd ...: the Monte Carlo dispersion). `base` = a compose_base() directory: its scene is copied (meshes linked)
    and given this environment / seed instead of composing again. -> manifest dict (model, body_paths ...)."""
    out = Path(out)
    if base is None:
        scene = compose(
            out,
            [Vehicle("turret", "cuas-lite", pos=(0.0, 0.0, mast), pedestal=mast)],
            env or Environment(),
            seed=seed,
        )
    else:
        base = Path(base)
        out.mkdir(parents=True, exist_ok=True)
        man0 = json.loads((base / "scene.json").read_text())
        man0.update(model=str(out / "scene.xml"), seed=seed)
        scene = dict(model=man0["model"], manifest=man0)
    model = Path(scene["model"])
    root = ET.parse(model if base is None else base / "scene.xml").getroot()
    if (
        base is not None
    ):  # mesh paths are relative to the scene directory: make them absolute
        for m in root.iter("mesh"):
            m.set("file", str((base / m.get("file")).resolve()))
    txt = root.find("custom/text[@name='napl.devices']")
    devs = json.loads(txt.get("data"))
    man = scene["manifest"]
    if base is not None:  # what compose() puts in from env / seed
        e = env or Environment()
        devs["environments"][0].update(
            wind=list(e.wind), w20=e.w20, seed=e.seed, origin_alt=e.origin_alt
        )
        devs["seed"] = seed
    for g in devs["launchers"]:
        g.update(dict(muzzle_velocity=v0, v_std=aimlib.V0_SIGMA, spread_std=aimlib.SPREAD_SIGMA, pellet_mass=aimlib.BALL["m"], diameter=aimlib.BALL["d"],
                      cd=aimlib.BALL["cd"], max_time=3.0), **(launcher or {}))  # fmt: skip
        man["device_cfg"][g["name"]] = g
    txt.set("data", json.dumps(devs, sort_keys=True, separators=(",", ":")))
    wb = root.find("worldbody")
    r_ball = aimlib.BALL_RADIUS
    for t in targets:
        if isinstance(t, Mover):
            _add_mover(wb, t, r_ball)
            continue
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
    """Head geometry of the turret by posing it: muzzle and camera at tilt 0 and 0.3 rad give the tilt pivot (spec 5.1 frame B: origin on
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

    def __init__(
        self,
        clock,
        truth_fn,
        intrinsics=BRIO,
        fps=60.0,
        latency=0.030,
        jitter=0.0,
        seed=0,
    ):
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
        self.jitter, self.due = (
            jitter,
            0.0,
        )  # per-frame latency sigma (s); frames stay in order (a late frame holds the later ones)
        self.rng = np.random.default_rng(seed + 7919)

    def read(self, timeout=0.1):
        now = self.clock.now()
        if now >= self.next_t - 1e-9:
            self.next_t += self.period
            self.seq += 1
            k = self.intrinsics
            lat = self.latency + (
                self.rng.normal(0, self.jitter) if self.jitter else 0.0
            )
            self.due = max(self.due, now + max(lat, 0.005))
            self.q.append(
                (
                    self.due,
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
        jitter=0.0,
        px_sigma=1.5,
        miss_scale=1.0,
        person_miss=0.0,
    ):
        self.clock, self.k, self.latency, self.drop, self.outages, self.noise = (
            clock,
            intrinsics,
            latency,
            drop,
            list(outages),
            noise,
        )
        self.jitter, self.due = (
            jitter,
            0.0,
        )  # reply latency sigma (s), replies stay in order
        self.px_sigma, self.miss_scale, self.person_miss = (
            px_sigma,
            miss_scale,
            person_miss,
        )  # person_miss: stress only (spec default: persons are not missed)
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
        lat = self.latency + (self.rng.normal(0, self.jitter) if self.jitter else 0.0)
        self.due = max(self.due, now + max(lat, 0.002))
        self.q.append(
            (
                self.due,
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
            person = t["cls"] == "person"
            if person:  # a person is reported as soon as any part of the box is in the image (a real detector sees a half body)
                if not (
                    u + w / 2 > 0
                    and u - w / 2 < k.w
                    and v + h / 2 > 0
                    and v - h / 2 < k.h
                ):
                    continue
            elif not (0 <= u < k.w and 0 <= v < k.h):
                continue
            if n:
                if person and self.person_miss and rng.random() < self.person_miss:
                    continue
                if not person and rng.random() < self.miss_scale * (
                    0.02 + 0.03 * (z / 5.0)
                ):
                    continue
                u, v = (
                    u + rng.normal(0, self.px_sigma),
                    v + rng.normal(0, self.px_sigma),
                )
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
            x0, y0, x1, y1 = u - w / 2, v - h / 2, u + w / 2, v + h / 2
            if person:
                x0, y0, x1, y1 = max(x0, 0.0), max(y0, 0.0), min(x1, k.w), min(y1, k.h)
            dets.append(Det(t["cls"], conf, (x0, y0, x1, y1)))
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
        camera_kw=None,
    ):
        from ..cuas.world import World
        from .app import App, AppCfg
        from .gimbal import SerialGimbal
        from .sim_mcu import LoopbackLink, SimRP2040

        self.targets = {t.id: t for t in targets}
        self.movers = [t for t in targets if isinstance(t, Mover)]
        self.world = w = World(manifest["model"], run_dir, seed=seed or 1, ctl_ticks=5)
        self._drive(w.t)
        self.base = np.array(w.man["vehicles"]["turret"]["pos"], float)
        self.clock = SimClock(w)
        self.plant = NaplPlant(w)
        self.link = LoopbackLink(self.clock)
        # step_loss off: the kernel joint is a stepper-as-servo position loop (DC-equivalent motor, no pull-in torque curve, no step loss), so the
        # spec's lost-steps model (4 full steps, the default of SimRP2040) has no plant behind it here; pid_scale of the assembly is tuned so that a
        # 40 deg move at 100-240 deg/s is tracked within 0.05 deg with 0.03 deg overshoot
        self.twin = SimRP2040(
            self.plant,
            self.link,
            step_loss=False,
            status_from_joint=True,
            home_off_deg=(-1.0, -1.0),
        )
        self.geom = measure_geometry(w, self.plant)
        self.twin.step(w.t)
        self.camera = SimCamera(self.clock, self._truth, seed=seed, **(camera_kw or {}))
        self.camera.next_t = w.t
        self.detector = TruthDetector(self.clock, seed=seed, **(detector_kw or {}))
        self.gimbal = SerialGimbal(self.link, self.clock, session=0x5EED0000 + seed)
        cfg = app_cfg or AppCfg()
        if (
            app_cfg is None
        ):  # slow slews: kept from the XL430 placeholder (lag under ~1 deg); the stepper-servo tracks 240 deg/s within 0.05 deg but the scenarios were tuned on these
            cfg.fire.vmax, cfg.fire.amax = (100.0, 70.0), (600.0, 500.0)
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
        self.hits = {}  # target id -> balls that struck it
        self.hit_log = []  # (t, id, speed, round)
        self._ev_seen = 0

    def _drive(self, t):
        """Place every Mover at its closed-form pose for sim time t (free-joint qpos + qvel: logged SetAttr, replayed)."""
        w = self.world
        for m in self.movers:
            pos, vel = m.pv(t)
            j = f"{w.body_path[w.man['body_names'].index(m.id)]}/{m.id}_j"
            w.sim.set(j, "qpos", [*pos, 1.0, 0.0, 0.0, 0.0])
            w.sim.set(j, "qvel", [*vel, 0.0, 0.0, 0.0])

    def _truth(self, t):
        w, q = self.world, self.world.sim.query
        tg = []
        for tr in self.targets.values():
            p = q(w.body_path[w.man["body_names"].index(tr.id)], "pos")[0] - self.base
            d = dict(id=tr.id, cls=tr.klass, pos=p, size=tr.size_m)
            if isinstance(tr, Mover):
                d["w"], d["h"] = tr.box_m
            tg.append(d)
        return dict(
            cam_pos=q(w.cam_path, "pos")[0] - self.base,
            cam_fwd=q(w.cam_path, "fwd")[0],
            cam_up=q(w.cam_path, "up")[0],
            targets=tg,
        )

    def run(self, seconds, until=None):
        w, app = self.world, self.app
        n = int(round(seconds / w.dtc))
        for _ in range(n):
            now = w.t
            while self.script and self.script[0][0] <= now:
                self.script.pop(0)[1](self)
            if self.hold_deadman and now >= self._next_dm:
                app.deadman()
                self._next_dm = now + 0.2
            if self.movers:
                self._drive(now)
            self.twin.step(now)
            app.tick(now)
            w.step(1)
            for ev in w.events[self._ev_seen :]:
                if ev.get("type") == "hit":
                    name = ev["body"].rsplit("/", 1)[-1]
                    tr = self.targets.get(name)
                    if tr:
                        self.hits[name] = self.hits.get(name, 0) + 1
                        self.hit_log.append((ev["t"], name, ev["speed"], ev["round"]))
                    if (
                        tr
                        and tr.klass != "person"
                        and name not in self.popped
                        and ev["speed"] >= 8.0
                    ):  # balloon: any hit at >= 8 m/s pops it (spec 7.3); a quad / hanging target is taken as killed the same way
                        self.popped[name] = w.t
                        self.detector.popped.add(name)
            self._ev_seen = len(w.events)
            if until and until(self):
                break
        return self

    def finish(self):
        self.world.finish()
        return json.loads((Path(self.world.run_dir) / "native.json").read_text())
