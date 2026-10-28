"""cuas2 control app: napl.track tracker -> target selection -> cuas2 lead solution -> hit-probability fire control -> protocol 1.1 FIRE.

    app = App2(camera, detector, Gimbal2(link, clock), clock, AppCfg2(), geom=Geometry(), sector=Sector(-60, 60, 0, 60))
    app.tick(clock.now())                     # one run-to-completion step, same contract as napl.lite.app.App

It IS the lite App (HELLO, sector sync, ARM, heartbeat, dead-man, fail-closed, MCU events) with three parts replaced:

* perception: `TrackFront` feeds each detector reply to a napl.track tracker (default `imm2-gnn`, any name of napl.track.variants.VARIANTS) as pixel + size-range
  measurements of a pinhole camera at the head pose of the frame, and hands fire control `Tgt` objects with the track mean/covariance;
* aiming: `lead2` = lite's two-pass lead with `ballistics.solve_lead2` (drag + Magnus, v0 and spin from the LauncherModel set point);
* fire control: `FireControl2` = lite's FireControl (person block, geofence, dead-man, cap, timeout, key/E-stop: all inherited, default-deny) with the pointing-error
  rule replaced by **Ph**: a Monte Carlo over (track state covariance propagated to the intercept time) x (launcher dispersion from LauncherModel.sample_shots),
  FIRE only when Ph >= `ph_tau` and the MCU reports READY, and shoot-look-shoot: after a ball is fed the same target is not fired at again before the ball
  has exited, flown and been observed (`t_exit + t_flight + t_obs`).

Hard rules are unchanged and live in the inherited gates: a person above `person_conf` anywhere in the image (held 1 s), a stale detector or camera, a target box
overlapping a person box, a target or barrel outside the sector, no operator ARM, no fresh dead-man -> no FIRE, whatever Ph says. `ph_gate=False` swaps Ph for lite's
pointing tolerance (the ablation); it never removes a veto."""

import math
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from napl.lite import aim as aimlib
from napl.lite import protocol as p
from napl.lite.app import App, AppCfg
from napl.lite.fire import Decision, FireCfg, FireControl, View
from napl.lite.io import TARGET_CLASSES
from napl.track import variants as tv
from napl.track.models import CameraModel, PositionModel

from . import ballistics as bal
from .calib import Calib
from .launcher import LauncherModel

T_EXIT = 0.118  # s from the FEED start (EVENT 2 SHOT) to the muzzle beam: docs/cuas2/spec.md 5.4
R_TARGET = {
    "balloon": 0.15,
    "target": 0.15,
    "drone": 0.15,
}  # sphere radius by class (spec 8); the kernel inflates the target by the ball radius, so does Ph
SIZE_M = aimlib.CLASS_SIZE_M


@dataclass
class FireCfg2(FireCfg):
    ph_gate: bool = True
    ph_tau: float = 0.3  # FIRE only if Ph >= this (spec 8: sweep 0.1-0.5)
    n_ph: int = 500  # Monte Carlo samples per decision (spec 8; 300 before P13 PERF)
    ph_mixture: bool = True  # target states from the IMM mixture, not the moment-matched Gaussian (P13 PERF: kills/episode 0.50 -> 0.75 at 5-10 m)
    t_obs: float = 0.15  # s after the ball would have arrived before the same target is fired at again
    point_gate_deg: float = (
        3.0  # with the Ph gate: a coarse sanity bound, Ph decides inside it
    )
    flywheel_duty: float = (
        1.0  # fraction of rpm_max with a target (rpm_max 17500 -> ~30 m/s)
    )
    idle_duty: float = 0.257  # pre-rev while armed with a live dead-man (rpm_idle / rpm_max = 4500 / 17500)
    tracker: str = "imm2-gnn"
    confirm_age: int = (
        5  # scans before a confirmed track is engageable (lite: 5 consecutive replies)
    )
    seed: int = 0
    sigma_px: float = 2.0  # tracker measurement model: pixel noise ...
    range_rel: float = 0.15  # ... and size-range noise (real boxes: 10-25 %, spec 0; the sim detector jitters the box 4 %)


@dataclass
class AppCfg2(AppCfg):
    fire: FireCfg2 = field(default_factory=FireCfg2)


@dataclass
class Tgt:
    """What fire control sees of one track: mean/cov in B at `t` (the scan time), class and box from the associated detection."""

    id: int
    t: float
    mean: np.ndarray  # (6,)
    cov: np.ndarray  # (6, 6)
    track: object
    cls: str = "drone"
    conf: float = 0.0
    bbox: tuple = (0.0, 0.0, 0.0, 0.0)
    confirmed: bool = False
    misses: int = 0
    range_m: float = 0.0
    manual_range: float = None

    @property
    def pos(self):
        return self.mean[:3]

    @property
    def vel(self):
        return self.mean[3:6]


class TrackFront:
    """Detector replies -> napl.track tracker -> [Tgt]. Measurement = (u, v, r) pixels + size range r = f * size(cls) / box, noise sigma_px / range_rel; the
    `ab-nn` baseline gets the same measurement turned into a 3D position (it has no camera model)."""

    def __init__(
        self,
        k,
        variant="imm2-gnn",
        seed=0,
        sigma_px=2.0,
        range_rel=0.15,
        pd=0.93,
        clutter=0.5,
        confirm_age=5,
        sizes=None,
    ):
        self.k, self.variant, self.sigma_px, self.range_rel, self.confirm_age = (
            k,
            variant,
            sigma_px,
            range_rel,
            confirm_age,
        )
        self.sizes = dict(SIZE_M, **(sizes or {}))
        lam = clutter / (
            k.w * k.h * 30.0
        )  # expected false detections per scan over the (u, v, r) volume
        self.tracker = tv.make_tracker(variant, SimpleNamespace(pd=pd, lam=lam), seed)
        self.pos_meas = variant == "ab-nn"
        self.meta = {}  # track id -> (cls, conf, bbox)

    def update(self, dets, pose, t):
        k = self.k
        pan = math.atan2(pose.cam_fwd[1], pose.cam_fwd[0])
        tilt = math.asin(max(-1.0, min(1.0, pose.cam_fwd[2])))
        cam = CameraModel(
            pose.cam_pos,
            pan,
            tilt,
            f_px=k.fy,
            cx=k.w / 2,
            cy=k.h / 2,
            sigma_px=self.sigma_px,
            size_range=True,
            range_rel=self.range_rel,
        )
        zs, info = [], []
        for d in dets:
            x0, y0, x1, y1 = (float(v) for v in d.xyxy)
            size = max(x1 - x0, y1 - y0, 1.0)
            u, v = (x0 + x1) / 2, (y0 + y1) / 2
            r = k.fy * self.sizes.get(d.cls, 0.30) / size
            if self.pos_meas:
                o, ray = cam.ray(np.array([u, v]))
                zs.append(o + ray * r)
            else:
                zs.append([u, v, r])
            info.append((u, v, d.cls, d.conf, (x0, y0, x1, y1)))
        meas = PositionModel(0.4) if self.pos_meas else cam
        tr = self.tracker.step(t, np.array(zs, float).reshape(-1, meas.dim), meas)
        del tr
        out = []
        for tk in self.tracker.tracks:
            m, P = tk.mean, tk.cov
            if info and tk.misses == 0:  # attach the detection this track just took
                uv = cam.h(np.asarray(m[:3], float))
                j = int(
                    np.argmin([(i[0] - uv[0]) ** 2 + (i[1] - uv[1]) ** 2 for i in info])
                )
                if (info[j][0] - uv[0]) ** 2 + (info[j][1] - uv[1]) ** 2 < 40.0**2:
                    self.meta[tk.id] = (info[j][2], info[j][3], info[j][4])
            cls, conf, bbox = self.meta.get(
                tk.id, ("unknown", 0.0, (0.0, 0.0, 0.0, 0.0))
            )
            ok = (
                tk.confirmed
                and tk.age >= self.confirm_age
                and cls in TARGET_CLASSES
                and conf >= aimlib.MIN_CONF
            )
            out.append(Tgt(tk.id, t, np.asarray(m[:6], float), np.asarray(P[:6, :6], float), tk, cls, float(conf), bbox, bool(ok), tk.misses,
                           float(np.linalg.norm(np.asarray(m[:3]) - pose.cam_pos))))  # fmt: skip
        self.meta = {
            i: v
            for i, v in self.meta.items()
            if any(tk.id == i for tk in self.tracker.tracks)
        }
        return out


def lead2(
    geom, model, setpoint, tgt, dt_extra, pan_deg, tilt_deg, wind=(0.0, 0.0, 0.0)
):
    """lite's two-pass lead (muzzle at the head attitude, then at the solved attitude) with the cuas2 solver. -> dict(pan, tilt, u, t_flight, ok, speed, range) or None."""
    pos = np.asarray(tgt.pos, float) + np.asarray(tgt.vel, float) * dt_extra
    m, up, sol = geom.pose(pan_deg, tilt_deg).muzzle, np.zeros(3), None
    for _ in range(2):
        sol = bal.solve_lead2(
            m + up,
            pos + up,
            tgt.vel,
            model=model,
            setpoint=setpoint,
            wind=wind,
            dt=0.008,
            iters=8,
            tol=0.03,
        )
        if (
            sol is None and not up.any()
        ):  # the flat first guess of a long shot from a low mast falls to the ground: solve 15 m higher (same angles)
            up = np.array([0.0, 0.0, 15.0])
            sol = bal.solve_lead2(
                m + up,
                pos + up,
                tgt.vel,
                model=model,
                setpoint=setpoint,
                wind=wind,
                dt=0.008,
                iters=8,
                tol=0.03,
            )
        if sol is None:
            return None
        m = geom.pose(math.degrees(sol["az"]), math.degrees(sol["el"])).muzzle
    return dict(pan=math.degrees(sol["az"]), tilt=math.degrees(sol["el"]), u=sol["u"], t_flight=sol["t_flight"], ok=sol["ok"], speed=sol["speed"],
                range=float(np.linalg.norm(pos - m)), intercept=sol["intercept"] - up)  # fmt: skip


def hit_probability(
    model,
    setpoint,
    muzzle,
    u,
    tgt,
    t_imp,
    t_flight,
    radius,
    n,
    rng,
    wind=(0.0, 0.0, 0.0),
    mixture=False,
):
    """Ph of a ball fired along the unit vector `u` from `muzzle` against the track `tgt` (spec 8): n target states drawn from the track covariance propagated to
    t_imp x n shots from LauncherModel.sample_shots (speed, direction scatter, spin, ball d / m / cd), each flown t_flight; a hit = the closest approach of
    the ball to the target (relative motion, within +-0.2 s) is inside `radius`. mixture: draw the target states from the IMM mixture (mode ~ mu, then that
    mode's Gaussian) instead of the moment-matched single Gaussian. -> (Ph, n)."""
    tp = None
    est = getattr(tgt.track, "est", None) if mixture else None
    if (
        est is not None
    ):  # a track without an estimator (tests' fakes) keeps the Gaussian
        e = est.copy()
        if t_imp > tgt.track.t:
            e.predict(t_imp - tgt.track.t)
        modes = getattr(e, "modes", None)
        if modes is not None and len(modes) > 1:
            j = rng.choice(len(modes), n, p=e.mu / e.mu.sum())
            tp = np.empty((n, 6))
            for k, md in enumerate(modes):
                idx = j == k
                if idx.any():
                    Pk = np.asarray(md.P, float)[:6, :6]
                    Pk = 0.5 * (Pk + Pk.T) + 1e-9 * np.eye(6)
                    tp[idx] = rng.multivariate_normal(
                        np.asarray(md.x, float)[:6], Pk, int(idx.sum())
                    )
    if tp is None:
        m6, P6 = tgt.track.predict_to(t_imp)
        m6, P6 = np.asarray(m6, float)[:6], np.asarray(P6, float)[:6, :6]
        P6 = 0.5 * (P6 + P6.T) + 1e-9 * np.eye(6)
        tp = rng.multivariate_normal(m6, P6, n)
    sh = model.sample_shots(n, rng, setpoint)
    left = np.cross(bal.UP, u)
    left /= np.linalg.norm(left)
    upv = np.cross(u, left)
    dirs = u + sh.dir_err[:, :1] * left + sh.dir_err[:, 1:2] * upv
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    lo = model.c.launcher
    kw = dict(cl_k=lo.cl_k, spin_tau=lo.spin_tau_s, wind=wind)
    om = sh.omega[:, None] * bal.lateral_axis(u)
    p1 = bal.fly_batch(
        muzzle, sh.v0[:, None] * dirs, om, sh.d, sh.m, sh.cd, t_flight, **kw
    )
    p2 = bal.fly_batch(
        muzzle, sh.v0[:, None] * dirs, om, sh.d, sh.m, sh.cd, t_flight + 0.004, **kw
    )
    vb = (p2 - p1) / 0.004
    mrel, w = p1 - tp[:, :3], vb - tp[:, 3:6]
    ts = np.clip(-(mrel * w).sum(1) / np.maximum((w * w).sum(1), 1e-9), -0.2, 0.2)
    d = np.linalg.norm(mrel + w * ts[:, None], axis=1)
    return float(np.mean(d <= radius)), n


class FireControl2(FireControl):
    """lite FireControl with the Ph gate. `model` is the app's LauncherModel (nominal calibration), `setpoint` the rpm it asks for."""

    def __init__(self, cfg, geom, sector=None, f_px=736.0, model=None):
        super().__init__(cfg, geom, sector, f_px)
        self.model = model or LauncherModel(Calib())
        self.setpoint = self.model.c.launcher.rpm_max * cfg.flywheel_duty
        self.rng = np.random.default_rng(cfg.seed + 77)
        self.next_ok = {}  # track id -> earliest time of the next FIRE at it (shoot-look-shoot)
        self.last_tf = 0.4
        self.last_ph = self._ph_sent = None
        self.ph_shots = []  # Ph of the decision behind every ball fed (calibration against the referee's hits)

    def arm(self, now):
        super().arm(now)
        self.next_ok = {}

    def fire_sent(self, now, n=None):
        self._ph_sent = self.last_ph
        super().fire_sent(now, n)

    def on_shot(self, now):
        tid = self.pending[1] if self.pending else None
        self.ph_shots.append(self._ph_sent)
        super().on_shot(now)
        if tid is not None:
            self.next_ok[tid] = now + T_EXIT + self.last_tf + self.cfg.t_obs

    def _step(self, now, v: View) -> Decision:
        c, st = self.cfg, v.status
        d = Decision()
        if (
            self.pending
            and st is not None
            and st.state != p.FIRING
            and now - self.pending[0] > 0.1
        ):
            self.pending, self.cancel = (
                None,
                False,
            )  # the MCU is not firing: nothing in flight
        d.stage = (
            "SAFE"
            if (not v.link_ok or st is None or st.state in (p.SAFE, p.FAULT))
            else p.STATE_NAMES[st.state]
        )
        d.deadman = now - v.deadman_t < c.deadman_s
        veto = self.hard_veto(v)
        if self.armed and ("cap" in veto or "timeout" in veto) and not self.pending:
            d.disarm, d.reasons = True, veto
            self.disarm()
            return d
        if (
            self.armed
            and st is not None
            and v.link_ok
            and st.state in (p.IDLE, p.SAFE, p.FAULT)
            and now - self.arm_t > 0.5
        ):
            self.disarm()  # the MCU left the armed states: re-arm is the operator's call
            veto.append("mcu_not_armed")
        d.reasons = list(veto)
        live = self.pending_live(now)
        if self.pending is not None and not live:
            self.pending = None
        if self.pending is None:
            self.cancel = False
        hold = self.last_aim
        if veto and not (set(veto) <= {"cap"} and live):
            d.aim, d.flywheel, self.fw_on, d.inhibit = (
                (hold[0], hold[1], 0.0, 0.0),
                0.0,
                False,
                True,
            )
            return d
        cands = [t for t in v.tracks if not self.engageable(t, v)]
        tgt = min(cands, key=lambda t: t.range_m) if cands else None
        if self.target_id is not None:  # sticky while engageable
            tgt = next((t for t in cands if t.id == self.target_id), None) or tgt
        idle = c.idle_duty if d.deadman else 0.0
        if tgt is None:
            self.target_id = self.last_solution = None
            d.aim = (hold[0], hold[1], 0.0, 0.0)
            d.reasons.append("no_target")
            # lite's spin-down hold: a one-frame track dropout must not drop the wheels to idle (re-spin to the READY window took ~0.5 s: READY 23 % of the time on quad8)
            hot = self.fw_on and d.deadman and now - self.last_target_t <= c.spin_down_s
            d.flywheel, d.inhibit = (c.flywheel_duty if hot else idle), not live
            return d
        self.target_id, self.last_target_t = tgt.id, now
        pan, tilt = v.pan, v.tilt
        sol = lead2(
            self.geom,
            self.model,
            self.setpoint,
            tgt,
            (now - v.tracker_t) + c.t_actuator + c.t_fire,
            pan,
            tilt,
            v.wind,
        )
        d.target, d.solution = tgt, sol
        if sol is None:
            d.aim, d.inhibit = (hold[0], hold[1], 0.0, 0.0), not live
            d.reasons.append("no_solution")
            return d
        self.last_tf = sol["t_flight"]
        if not (
            self.sector.contains(sol["pan"], sol["tilt"])
            and self.sector.contains(pan, tilt)
        ):
            d.reasons.append("barrel_outside_sector")
            d.aim, d.inhibit = (hold[0], hold[1], 0.0, 0.0), True
            self.cancel |= live
            return d
        pt = self.last_solution
        if pt and pt[1] == tgt.id and now > pt[0]:
            self.vff = tuple(
                0.5 * o + 0.5 * float(np.clip((n - q) / (now - pt[0]), -200.0, 200.0))
                for o, n, q in zip(self.vff, (sol["pan"], sol["tilt"]), pt[2:])
            )
        else:
            self.vff = (0.0, 0.0)
        self.last_solution = (now, tgt.id, sol["pan"], sol["tilt"])
        d.aim = (sol["pan"], sol["tilt"], self.vff[0], self.vff[1])
        self.last_aim = (sol["pan"], sol["tilt"])
        if not d.deadman:
            d.reasons.append("deadman")
            d.flywheel, self.fw_on, d.inhibit = 0.0, False, True
            self.cancel |= live
            return d
        self.fw_on = True
        d.flywheel = c.flywheel_duty
        pose = self.geom.pose(pan, tilt)
        err = math.degrees(
            math.acos(max(-1.0, min(1.0, float(pose.muzzle_dir @ sol["u"]))))
        )
        d.err_deg = err
        if not sol["ok"]:
            d.reasons.append("solution_not_ok")
        if not (
            st is not None and st.state == p.ENGAGING and getattr(st, "ready", False)
        ):
            d.reasons.append(
                "not_ready"
            )  # wheels in window, ball staged, interval elapsed: the MCU's own READY
        if live:
            d.reasons.append("pending")
        if now < self.next_ok.get(tgt.id, -math.inf):
            d.reasons.append("shoot_look_shoot")
        if self.shots + c.burst > c.shot_cap:
            d.reasons.append("cap")
        if c.ph_gate:
            if err > c.point_gate_deg:
                d.reasons.append("pointing")
        else:
            size_px = max(tgt.bbox[2] - tgt.bbox[0], tgt.bbox[3] - tgt.bbox[1], 1)
            if err > min(
                c.aim_tol_deg,
                c.tol_frac * math.degrees(math.atan2(0.5 * size_px, self.f_px)),
            ):
                d.reasons.append("pointing")
        d.ph = None
        if not [
            x for x in d.reasons if x != "no_target"
        ]:  # the Monte Carlo only for a shot that is otherwise allowed (also computed, not gated, with ph_gate off: calibration)
            t_imp = now + c.t_actuator + c.t_fire + sol["t_flight"]
            rad = R_TARGET.get(tgt.cls, 0.15) + self.model.c.ball.d_mm / 2000.0
            d.ph, _ = hit_probability(
                self.model,
                self.setpoint,
                pose.muzzle,
                pose.muzzle_dir,
                tgt,
                t_imp,
                sol["t_flight"],
                rad,
                c.n_ph,
                self.rng,
                v.wind,
                mixture=c.ph_mixture,
            )
            self.last_ph = d.ph
            if c.ph_gate and d.ph < c.ph_tau:
                d.reasons.append("ph")
        blocking = [x for x in d.reasons if x != "no_target"]
        d.fire = 0 if blocking else c.burst
        d.inhibit = not (d.fire or live)
        return d


class App2(App):
    """The lite App with TrackFront + FireControl2 (see the module docstring)."""

    def __init__(
        self,
        camera,
        detector,
        gimbal,
        clock,
        cfg=None,
        geom=None,
        sector=None,
        model=None,
    ):
        cfg = cfg or AppCfg2()
        super().__init__(camera, detector, gimbal, clock, cfg, geom, sector)
        k = camera.intrinsics
        f = cfg.fire
        self.front = TrackFront(
            k, f.tracker, f.seed, f.sigma_px, f.range_rel, confirm_age=f.confirm_age
        )
        self.fc = FireControl2(cfg.fire, self.geom, sector, f_px=k.fy, model=model)

    def _perceive(self, now, f, res):
        g = self.gim
        pt = g.status_at(f.t_cap)
        dets = res.dets
        pc = self.fc.cfg.person_conf
        self.persons = [(d.conf, tuple(d.xyxy)) for d in dets if d.cls == "person"]
        if any(cf >= pc for cf, _ in self.persons):
            self.person_t = now
        if pt is None or g.status_t is None or now - g.status_t > self.cfg.pose_max_age:
            return False  # no head attitude for this frame (the person block above still holds)
        pose = self.geom.pose(*pt)
        keep = [d for d in dets if d.cls in TARGET_CLASSES and d.conf >= 0.3]
        self.tracks = self.front.update(keep, pose, f.t_cap)
        self.tracker_t = f.t_cap
        return True
