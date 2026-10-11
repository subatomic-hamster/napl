"""App-layer fire control: the rules of the "rule-by-layer" table (spec 3.2) that the app owns, as one state machine with default-deny.

    fc = FireControl(FireCfg(), geom, sector)
    fc.arm(now)                                              # operator pressed ARM (the MCU confirms through STATUS.state)
    d = fc.step(now, view)                                   # one decision per detector reply; the caller executes it
    d.inhibit, d.aim, d.flywheel, d.fire, d.reasons          # FIRE_INHIBIT for the heartbeat, AIM, FLYWHEEL duty, FIRE n, why not

Authorization (Q9, resolved): the default is SESSION ARM, once ARMED every confirmed in-sector track is engageable; with `cfg.per_track_auth` the
operator must `authorize(track_id)` first (valid 10 s, 3 rounds). Everything else holds in both modes: person block (a person at conf >= 0.15 anywhere
in the last 1.0 s, a stale detector or camera, a target box overlapping a person box), geofence (target AND commanded barrel inside the sector,
el_min >= 0), dead-man fresh (< 0.5 s), pointing error <= min(0.5 deg, 0.25 x the target's angular radius), per-arm shot cap and timeout, key on /
E-stop released / homed (from STATUS). Default-deny: FIRE_INHIBIT is in every heartbeat except while a FIRE this module decided is in flight.
`step` never raises on bad input of its own making; the App wraps it so that any exception disarms and stops the heartbeats (fail closed)."""

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import aim as aimlib
from . import protocol as p
from .io import TARGET_CLASSES

SECTOR_PATH = Path("~/.config/cuas-lite/sector.toml").expanduser()


@dataclass
class Sector:
    """Geofence in the home frame, degrees: azimuth (pan, + CCW) and elevation (tilt, + up) limits, optional range limit in metres."""

    az_min: float
    az_max: float
    el_min: float
    el_max: float
    r_max: float = None

    def __post_init__(self):
        if self.el_min < 0:
            raise ValueError("sector el_min must be >= 0 deg (never toward the ground)")
        if not (self.az_min < self.az_max and self.el_min < self.el_max):
            raise ValueError("sector limits must satisfy min < max")
        if not (-180 <= self.az_min and self.az_max <= 180 and self.el_max <= 90):
            raise ValueError("sector outside az -180..180 / el 0..90")

    def contains(self, az, el, r=None):
        return (
            self.az_min <= az <= self.az_max
            and self.el_min <= el <= self.el_max
            and (self.r_max is None or r is None or r <= self.r_max)
        )

    def asdict(self):
        return dict(
            az_min=self.az_min,
            az_max=self.az_max,
            el_min=self.el_min,
            el_max=self.el_max,
            r_max=self.r_max,
        )


def load_sector(path=SECTOR_PATH):
    import tomllib

    try:
        d = tomllib.loads(Path(path).read_text())["sector"]
        return Sector(
            float(d["az_min"]),
            float(d["az_max"]),
            float(d["el_min"]),
            float(d["el_max"]),
            d.get("r_max"),
        )
    except (FileNotFoundError, KeyError, ValueError, tomllib.TOMLDecodeError):
        return None  # no (valid) sector: the app fires at nothing until the operator sets one


def save_sector(sector, path=SECTOR_PATH):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    s = f"[sector]\naz_min = {sector.az_min}\naz_max = {sector.az_max}\nel_min = {sector.el_min}\nel_max = {sector.el_max}\n"
    path.write_text(
        s + (f"r_max = {sector.r_max}\n" if sector.r_max is not None else "")
    )


@dataclass
class FireCfg:
    shot_cap: int = 5  # UI cap per arm (the MCU clamps to its hard cap)
    arm_timeout_s: int = 60
    burst: int = 1
    min_conf: float = 0.5
    person_conf: float = 0.15  # = the detector's own person threshold (cuas-lite-detector README): no person det is ignored
    person_hold_s: float = 1.0
    det_stale_s: float = 0.25
    deadman_s: float = 0.5
    min_range_m: float = 2.5
    max_range_m: float = 20.0
    give_up_rounds: int = 5
    give_up_s: float = 6.0
    flywheel_duty: float = 0.8  # duty 800 permille = 30 m/s nominal (spec 4.2)
    aim_tol_deg: float = 0.5
    tol_frac: float = 0.25
    per_track_auth: bool = False
    auth_valid_s: float = 10.0
    auth_rounds: int = 3
    spin_down_s: float = 2.0
    pending_timeout_s: float = 0.8
    k_v: float = 1.0
    t_actuator: float = aimlib.T_ACTUATOR
    t_fire: float = aimlib.T_FIRE
    vmax: tuple = (240.0, 150.0)
    amax: tuple = (2000.0, 1500.0)


@dataclass
class View:
    """What the control loop knows at one decision. All times are on the app clock."""

    now: float
    status: object  # protocol.Status or None
    link_ok: bool
    det_stale_s: float
    cam_stale_s: float
    persons: list  # [(conf, xyxy)] from the latest detector reply
    person_t: float  # time of the latest person >= person_conf (-inf if never)
    tracks: list
    tracker_t: float  # t_cap of the tracker's last update
    pan: float  # current head attitude, deg (STATUS)
    tilt: float
    deadman_t: float  # time of the last dead-man post (-inf if never)
    wind: tuple = (0.0, 0.0, 0.0)
    calib_ok: bool = True


@dataclass
class Decision:
    stage: str = "SAFE"
    inhibit: bool = True
    deadman: bool = False
    aim: tuple = None  # (pan, tilt, pan_vff, tilt_vff) deg, deg/s
    flywheel: float = 0.0  # duty 0..1
    fire: int = 0
    disarm: bool = False
    target: object = None  # Track
    solution: dict = None
    err_deg: float = None
    reasons: list = field(default_factory=list)


def _overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


class FireControl:
    def __init__(
        self, cfg: FireCfg, geom: aimlib.Geometry, sector: Sector = None, f_px=736.0
    ):
        self.cfg, self.geom, self.sector, self.f_px = cfg, geom, sector, f_px
        self.armed = False
        self.arm_t = 0.0
        self.shots = 0
        self.auth = {}  # track id -> [expires, rounds]
        self.given_up = {}  # track id -> until
        self.shots_at = {}
        self.target_id = None
        self.pending = None  # (t_sent, target_id)
        self.last_refusal = None
        self.last_solution = None  # (t, id, pan, tilt)
        self.vff = (0.0, 0.0)
        self.fw_on = False
        self.last_target_t = -1e9
        self.fw_started_t = -1e9
        self.last_shot_t = -1e9
        self.last_aim = (0.0, 0.0)

    # ---- operator / MCU events ---------------------------------------------------------------------------------------------------------
    def arm(self, now):
        (
            self.armed,
            self.arm_t,
            self.shots,
            self.pending,
            self.given_up,
            self.shots_at,
        ) = True, now, 0, None, {}, {}
        self.fw_on = False

    def disarm(self):
        self.armed, self.pending, self.fw_on, self.target_id = False, None, False, None

    def authorize(self, track_id, now):
        self.auth[track_id] = [now + self.cfg.auth_valid_s, self.cfg.auth_rounds]

    def on_shot(self, now):
        """EVENT 2 from the MCU: one ball fed."""
        self.shots += 1
        self.last_shot_t = now
        if self.pending:
            tid = self.pending[1]
            self.pending = None
            self.shots_at[tid] = self.shots_at.get(tid, 0) + 1
            if tid in self.auth:
                self.auth[tid][1] -= 1
            if self.shots_at[tid] >= self.cfg.give_up_rounds:
                self.shots_at[tid] = 0
                self.given_up[tid] = now + self.cfg.give_up_s
                self.target_id = None

    def on_fire_ack(self, ack):
        if ack.result != p.OK:
            self.pending, self.last_refusal = None, p.RESULT_NAMES[ack.result]

    def fire_sent(self, now):
        self.pending = (now, self.target_id)

    # ---- gates -------------------------------------------------------------------------------------------------------------------------
    def hard_veto(self, v: View):
        """Reasons that forbid any shot regardless of target (these also set FIRE_INHIBIT)."""
        r, st, c = [], v.status, self.cfg
        if not self.armed:
            r.append("not_armed")
        if not v.link_ok or st is None:
            r.append("link")
        else:
            if st.flag1(p.F1_ESTOP):
                r.append("estop")
            if not st.flag1(p.F1_KEY):
                r.append("key_off")
            if st.state < p.ARMED or st.state > p.FIRING:
                r.append("mcu_not_armed")
        if v.now - v.person_t < c.person_hold_s:
            r.append("person")
        if v.det_stale_s > c.det_stale_s:
            r.append("detector_stale")
        if v.cam_stale_s > c.det_stale_s:
            r.append("camera_stale")
        if self.sector is None:
            r.append("no_sector")
        if not v.calib_ok:
            r.append("no_calib")
        if self.shots >= c.shot_cap:
            r.append("cap")
        if v.now - self.arm_t > c.arm_timeout_s:
            r.append("timeout")
        return r

    def engageable(self, tr, v: View):
        """-> list of reasons this track cannot be engaged (empty = engageable)."""
        c, r = self.cfg, []
        if not tr.confirmed:
            r.append("unconfirmed")
        if getattr(tr, "cls", "") not in TARGET_CLASSES:
            r.append("class")
        if getattr(tr, "conf", 0.0) < c.min_conf:
            r.append("conf")
        if tr.misses:
            r.append("coasting")
        if self.given_up.get(tr.id, 0) > v.now:
            r.append("given_up")
        if c.per_track_auth:
            a = self.auth.get(tr.id)
            if not a or a[0] < v.now or a[1] <= 0:
                r.append("unauthorized")
        az, el, rng = aimlib.bearing(tr.pos, (0.0, 0.0, self.geom.h_t))
        if not c.min_range_m <= rng <= c.max_range_m:
            r.append("range")
        if self.sector is None or not self.sector.contains(az, el, rng):
            r.append("target_outside_sector")
        if any(
            _overlap(tr.bbox, bx) for conf, bx in v.persons if conf >= c.person_conf
        ):
            r.append("person_overlap")
        return r

    # ---- one decision ------------------------------------------------------------------------------------------------------------------
    def step(self, now, v: View) -> Decision:
        c, st = self.cfg, v.status
        d = Decision()
        d.stage = (
            "SAFE"
            if (not v.link_ok or st is None or st.state in (p.SAFE, p.FAULT))
            else p.STATE_NAMES[st.state]
        )
        d.deadman = now - v.deadman_t < c.deadman_s
        veto = self.hard_veto(v)
        if self.armed and ("cap" in veto or "timeout" in veto) and not self.pending:
            d.disarm = True
            self.disarm()
            d.reasons = veto
            return d
        if (
            self.armed
            and st is not None
            and v.link_ok
            and st.state in (p.IDLE, p.SAFE, p.FAULT)
            and now - self.arm_t > 0.5
        ):
            self.disarm()  # the MCU left the armed states (cap, timeout, disarm, watchdog): "re-arm" is the operator's call
            veto.append("mcu_not_armed")
        d.reasons = list(veto)
        pending_live = (
            self.pending is not None and now - self.pending[0] < c.pending_timeout_s
        )
        if self.pending is not None and not pending_live:
            self.pending = None
        hold = self.last_aim
        if veto and not (set(veto) <= {"cap"} and pending_live):
            d.aim = (hold[0], hold[1], 0.0, 0.0)
            d.flywheel = 0.0
            self.fw_on = False
            d.inhibit = True
            return d
        # armed, nothing vetoing: pick a target
        cands = []
        for tr in v.tracks:
            why = self.engageable(tr, v)
            if not why:
                cands.append(tr)
        tgt = min(cands, key=lambda t: t.range_m) if cands else None
        if (
            self.target_id is not None
        ):  # stick with the current target while it stays engageable
            cur = next((t for t in cands if t.id == self.target_id), None)
            tgt = cur or tgt
        if tgt is None:
            self.target_id = None
            self.last_solution = None
            d.aim = (hold[0], hold[1], 0.0, 0.0)
            d.reasons.append("no_target")
            if self.fw_on and now - self.last_target_t > c.spin_down_s:
                self.fw_on = False
            d.flywheel = c.flywheel_duty if self.fw_on and d.deadman else 0.0
            d.inhibit = not pending_live
            return d
        self.target_id, self.last_target_t = tgt.id, now
        pan, tilt = v.pan, v.tilt
        sol = aimlib.lead(
            self.geom,
            tgt,
            (now - v.tracker_t) + c.t_actuator + c.t_fire,
            pan,
            tilt,
            v.wind,
            c.k_v,
        )
        d.target, d.solution = tgt, sol
        if sol is None:
            d.aim = (hold[0], hold[1], 0.0, 0.0)
            d.reasons.append("no_solution")
            d.inhibit = not pending_live
            return d
        # barrel direction (commanded and actual) inside the geofence
        _, _, rng = aimlib.bearing(tgt.pos, (0.0, 0.0, self.geom.h_t))
        if not (
            self.sector.contains(sol["pan"], sol["tilt"])
            and self.sector.contains(pan, tilt)
        ):
            d.reasons.append("barrel_outside_sector")
            d.aim = (hold[0], hold[1], 0.0, 0.0)
            d.inhibit = not pending_live
            return d
        # feed-forward from the last two solutions
        pt = self.last_solution
        if pt and pt[1] == tgt.id and now > pt[0]:
            k = 0.5
            self.vff = tuple(
                (1 - k) * o + k * float(np.clip((n - q) / (now - pt[0]), -200.0, 200.0))
                for o, n, q in zip(self.vff, (sol["pan"], sol["tilt"]), pt[2:])
            )
        else:
            self.vff = (0.0, 0.0)
        self.last_solution = (now, tgt.id, sol["pan"], sol["tilt"])
        d.aim = (sol["pan"], sol["tilt"], self.vff[0], self.vff[1])
        self.last_aim = (sol["pan"], sol["tilt"])
        if not d.deadman:
            d.reasons.append("deadman")
            d.flywheel = 0.0
            self.fw_on = False
            d.inhibit = True
            return d
        if not self.fw_on:
            self.fw_started_t = now
        self.fw_on = True
        d.flywheel = c.flywheel_duty
        # pointing error against the solution (angle between the actual muzzle direction and the solved one)
        cur_dir = self.geom.pose(pan, tilt).muzzle_dir
        err = math.degrees(math.acos(max(-1.0, min(1.0, float(cur_dir @ sol["u"])))))
        d.err_deg = err
        size_px = max(tgt.bbox[2] - tgt.bbox[0], tgt.bbox[3] - tgt.bbox[1], 1)
        tol = min(
            c.aim_tol_deg,
            c.tol_frac * math.degrees(math.atan2(0.5 * size_px, self.f_px)),
        )
        if err > tol:
            d.reasons.append("pointing")
        if not sol["ok"]:
            d.reasons.append("solution_not_ok")
        spun = (
            st is not None
            and st.state == p.ENGAGING
            and st.fw_duty >= 0.9 * c.flywheel_duty * 1000
            and now - self.fw_started_t >= 0.8
        )
        if not spun:
            d.reasons.append("spinup")
        if pending_live:
            d.reasons.append("pending")
        if now - self.last_shot_t < 0.3:
            d.reasons.append("shot_interval")
        if self.shots + c.burst > c.shot_cap:
            d.reasons.append("cap")
        blocking = [x for x in d.reasons if x not in ("no_target",)]
        d.fire = 0 if blocking else c.burst
        d.inhibit = not (d.fire or pending_live)
        return d
