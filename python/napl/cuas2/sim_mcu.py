"""Plant behind the cuas2 feeder FSM (docs/cuas2/spec.md 7.4): flywheels, limiter, gate and pusher servos, beams, the ball, and the launcher shot.

    plant = Plant2(rig=None, calib=calib.load())      # fsm defaults to StubFsm (below) until napl.cuas2.fsm (FW) is merged
    out = plant.tick(cmd="FIRE", speed_req_pm=1000)   # one 1 ms step: FeederIn from the plant -> fsm.step -> FeederOut applied to the plant
    plant.run(500, speed_req_pm=1000)

`fsm` is anything with `step(FeederIn) -> FeederOut`; FeederIn / FeederOut below carry the field names fixed by spec section 12 (FW's classes have the same
fields, so Plant2 only reads/writes attributes). `StubFsm` is a minimal feeder + wheel controller for the plant tests, NOT the safety FSM: it has none of
the fault latches, the sector abort timing of 5.4 beyond a commit check, or the config handling of the real one.

Plant model (7.4): wheels = LauncherModel.step_wheels behind the DShot values; the limiter powers the pusher servo 254 ms after a FEED_REQ rising edge and
ignores edges for 338 +- 15 % (I14, as built); gate = slew-limited servo (60 deg in 100 ms), blocked while the rod is inside the bore; rod = 47.2 mm stroke at 0.4 mm/ms,
only moves while powered, cannot extend unless the gate is at BORE (I9); the ball falls into the pocket after 30 ms at LOAD; the breech beam reads the staged
ball and clears at 0.6 of the stroke; once the rod passes the commit stroke (t_commit_ms x speed) the ball is in the nip and leaves t_exit_ms (+ N(0, sigma)) later: LauncherModel.sample_shot
(wheels dip), rig.fire(shot), and the muzzle beam pulses for d/v starting when the ball front reaches the beam 55 mm after the nip centre. The beam level the
FSM sees at each 1 ms tick is latched high if any pulse overlapped that tick (the RP2040 timestamps the edges with a 1 us IRQ).
"""

import dataclasses
import math
from collections import deque
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from napl.lite import sim_mcu as _lite

from . import fsm as _F
from . import protocol as p2
from .calib import Calib
from .launcher import LauncherModel, dshot_to_u  # noqa: F401


@dataclass
class FeederIn:
    t_ms: int
    rpm_top: float | None  # None = telemetry lost
    rpm_bot: float | None
    beam_breech: bool  # a ball is staged
    beam_muzzle: bool  # a ball is in the beam (latched per tick)
    gate_home: bool
    push_home: bool
    estop_open: bool = False
    key_on: bool = True
    hb_ok: bool = True
    deadman: bool = True
    inhibit: bool = False
    in_sector: bool = True
    cmd: str | None = None  # None | "FIRE"
    speed_req_pm: int = 0
    diff_pm: int = 15
    cfg: dict = field(default_factory=dict)


@dataclass
class FeederOut:
    dshot_top: int = 0
    dshot_bot: int = 0
    gate_us: int = 1150
    push_us: int = 1000
    feed_req: bool = False
    kick: bool = False
    status_fields: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    ack: str | None = None


# ---- the stub FSM ------------------------------------------------------------------------------------------------------------------------
GATE_LOAD_US, GATE_BORE_US, PUSH_RET_US, PUSH_EXT_US = 1150, 1700, 1000, 2000
COMMIT_MS = 105  # fw_core.h FDR_COMMIT_MS: the ball is in the nip and leaves regardless (spec 5.4)


class StubFsm:
    """Wheel PI + feeder cycle INIT -> HOMING -> LOADING -> TO_BORE -> STAGED -> PUSH -> EXIT_WAIT -> RETRACT -> TO_LOAD (spec 5.5), 1 ms calls."""

    def __init__(
        self,
        rpm_max=17500.0,
        window_pm=10,
        settle_ms=100,
        min_interval_ms=500,  # firmware cfg 15 default (clamp >= 470, FDR_MIN_INTERVAL_FLOOR)
        kp=8.0,
        ki=40.0,
        ramp_rpm_s=40000.0,
        kv=2300.0,
        v_bus=12.0,
    ):
        self.rpm_max, self.window, self.settle, self.min_interval = (
            rpm_max,
            window_pm,
            settle_ms,
            min_interval_ms,
        )
        self.kp, self.ki, self.ramp, self.kv_v = kp, ki, ramp_rpm_s, kv * v_bus
        self.st, self.t0 = "HOMING", 0
        self.ref = [0.0, 0.0]
        self.hist = [deque(maxlen=8), deque(maxlen=8)]
        self.integ = [0.0, 0.0]
        self.in_win_since = None
        self.last_feed = -(10**9)
        self.tries = 0
        self.fire_id = 0
        self.feed_edge_until = -1

    def _wheels(self, i, out):
        want = (
            i.speed_req_pm / 1000.0 * self.rpm_max
            if (i.key_on and not i.estop_open and i.hb_ok)
            else 0.0
        )
        d = i.diff_pm / 1000.0
        sets = (want * (1 - d / 2), want * (1 + d / 2))
        for k, raw in enumerate(
            (i.rpm_top, i.rpm_bot)
        ):  # 8 ms moving average of the (noisy) telemetry
            if raw is None:
                self.hist[k].clear()
            else:
                self.hist[k].append(raw)
        meas = tuple(sum(h) / len(h) if h else None for h in self.hist)
        ok = want > 0 and None not in meas
        for k in (0, 1):
            step = self.ramp * 0.001
            self.ref[k] += max(-step, min(step, sets[k] - self.ref[k]))
            if want == 0 or meas[k] is None:
                self.integ[k] = 0.0
                u = 0.0 if want == 0 else self.ref[k] / self.kv_v
            else:
                e = (self.ref[k] - meas[k]) / self.rpm_max
                self.integ[k] = max(-0.5, min(0.5, self.integ[k] + self.ki * e * 0.001))
                u = self.ref[k] / self.kv_v + self.kp * e + self.integ[k]
            ds = 0 if want == 0 else int(round(48 + 1952 * max(0.0, min(1.0, u))))
            if k == 0:
                out.dshot_top = ds
            else:
                out.dshot_bot = ds
            ok = (
                ok
                and meas[k] is not None
                and abs(meas[k] - sets[k]) <= self.window / 1000.0 * max(sets[k], 1.0)
            )
        if ok:
            self.in_win_since = (
                i.t_ms if self.in_win_since is None else self.in_win_since
            )
        else:
            self.in_win_since = None
        return (
            self.in_win_since is not None and i.t_ms - self.in_win_since >= self.settle
        )

    def step(self, i: FeederIn) -> FeederOut:
        o, t = FeederOut(), i.t_ms
        ready_w = self._wheels(i, o)
        o.kick = True
        if i.estop_open or not i.key_on:
            self.st, self.t0 = "HOMING", t
        s = self.st
        o.gate_us = (
            GATE_BORE_US
            if s in ("TO_BORE", "STAGED", "PUSH", "EXIT_WAIT", "RETRACT")
            else GATE_LOAD_US
        )
        o.push_us = PUSH_EXT_US if s in ("PUSH", "EXIT_WAIT") else PUSH_RET_US
        if s == "HOMING" and i.gate_home and i.push_home:
            self.st, self.t0 = "LOADING", t
        elif s == "LOADING" and i.gate_home and t - self.t0 >= 30:
            self.st, self.t0 = "TO_BORE", t
        elif s == "TO_BORE" and t - self.t0 >= 110:
            if i.beam_breech:
                self.st, self.tries = "STAGED", 0
            else:
                self.tries += 1
                self.st, self.t0 = ("EMPTY", t) if self.tries >= 3 else ("TO_LOAD", t)
        elif s == "TO_LOAD" and i.gate_home:
            self.st, self.t0 = "LOADING", t
        elif s == "STAGED":
            if i.cmd == "FIRE":
                why = (
                    "INHIBIT" if i.inhibit else "NOT_SPUN" if not ready_w else "NO_BALL" if not i.beam_breech else "NOT_CLEAR" if i.beam_muzzle
                    else "TOO_SOON" if t - self.last_feed < self.min_interval else "RANGE" if not (i.in_sector and i.deadman) else None
                )  # fmt: skip
                o.ack = why or "OK"
                if not why:
                    self.st, self.t0, self.last_feed = "PUSH", t, t
                    self.fire_id += 1
                    o.events.append(("SHOT", self.fire_id))
                    o.push_us, o.feed_req = PUSH_EXT_US, True
                    self.feed_edge_until = t + 5
        elif s == "PUSH":
            if t - self.t0 < COMMIT_MS and not (
                i.in_sector and i.deadman and not i.inhibit and ready_w
            ):  # before commit: retract, no ball leaves
                self.st, self.t0, o.push_us = "RETRACT", t, PUSH_RET_US
                o.events.append(("FEED_ABORT", self.fire_id))
            elif t - self.t0 >= COMMIT_MS:
                self.st = "EXIT_WAIT"
        elif s == "EXIT_WAIT":
            if i.beam_muzzle:
                o.events.append(("MUZZLE", self.fire_id))
                self.st, self.t0, o.push_us = "RETRACT", t, PUSH_RET_US
            elif t - self.t0 > 250:  # exit_timeout_ms (cfg 38) from the feed start
                o.events.append(("JAM", self.fire_id))
        elif s == "RETRACT" and i.push_home:
            self.st, self.t0 = "TO_LOAD", t
        if t <= self.feed_edge_until:
            o.feed_req = True
        o.status_fields = dict(
            state=self.st,
            ready=self.st == "STAGED"
            and ready_w
            and t - self.last_feed >= self.min_interval,
            wheels_ready=ready_w,
        )
        return o


# ---- the plant ---------------------------------------------------------------------------------------------------------------------------
class FeederPlant:
    GATE_RATE = 1.0 / 100.0  # fraction of the swing per ms (60 deg in 100 ms)
    # Constants follow MECH / firmware (spec 5.4, wiring.md, fw_core.h, tests/cuas2/fwharness.Plant): 47.2 mm stroke at 0.4 mm/ms (118 ms out), the breech beam clears at
    # 0.6 of the stroke (71 ms), the ball is in the nip at 0.86 (101 ms, calib t_commit_ms), limiter window 254 ms / lockout 338 ms +-15 %, pocket fill 30 ms.
    ROD_RATE, STROKE_MM, CLEAR_MM = (
        0.4,
        47.2,
        14.2,
    )  # mm/ms, stroke, rod tip outside the valve body below this (0.3 of the stroke)
    BREECH_CLEAR_MM = 28.3  # 0.6 of the stroke: the breech beam (9 mm behind the plug axis) reads clear
    WINDOW_S, HOLD_S = 0.254, 0.338  # I14 as built
    MUZ_BEAM_M = 0.055  # beam after the nip centre
    FILL_MS = (
        30  # the ball falls into the pocket after this long at LOAD (spec 5.3 fill_ms)
    )

    def __init__(self, model: LauncherModel, rig=None, rng=None, hopper=24):
        self.model, self.rig = model, rig
        self.rng = rng if rng is not None else np.random.default_rng(2)
        c = model.c
        self.commit_mm = self.ROD_RATE * c.feed.t_commit_ms
        self.hold_s = self.HOLD_S * (1 + self.rng.uniform(-0.15, 0.15))
        self.g, self.x, self.hopper, self.pocket = 0.0, 0.0, hopper, False
        self.at_load_ms = 0
        self.powered_until, self.hold_until, self.feed_prev = -1.0, -1.0, False
        (
            self.t_nip,
            self.in_nip,
            self.pulses,
            self.shots,
            self.feed_edges,
            self.fire_calls,
        ) = None, False, [], [], [], 0
        self.strokes = 0  # rod strokes past the commit point
        self.windows = 0  # FEED_REQ edges the limiter accepted (each powers the pusher servo for 254 ms)

    @property
    def breech(self):
        return self.pocket and self.g > 0.95 and self.x < self.BREECH_CLEAR_MM

    def beams(self, t, dt):
        mu = any(a < t + dt and b > t for a, b, _ in self.pulses[-3:])
        return dict(
            beam_breech=self.breech,
            beam_muzzle=mu,
            gate_home=self.g < 0.02,
            push_home=self.x < 0.5,
        )

    def step(self, t, dt, out):
        """Advance the feeder by dt seconds at plant time t with the FSM outputs `out`."""
        rising = out.feed_req and not self.feed_prev
        self.feed_prev = out.feed_req
        if rising:
            self.feed_edges.append(t)
            if (
                t >= self.hold_until
            ):  # I14: one stroke window per edge, further edges ignored while the one-shot holds
                self.powered_until, self.hold_until = t + self.WINDOW_S, t + self.hold_s
                self.windows += 1
        ms = dt * 1000.0
        g_t = min(
            max((out.gate_us - GATE_LOAD_US) / (GATE_BORE_US - GATE_LOAD_US), 0.0), 1.0
        )
        if self.x <= self.CLEAR_MM:  # the valve cannot turn with the rod in the bore
            self.g += max(-self.GATE_RATE * ms, min(self.GATE_RATE * ms, g_t - self.g))
        x_t = (
            min(
                max((out.push_us - PUSH_RET_US) / (PUSH_EXT_US - PUSH_RET_US), 0.0), 1.0
            )
            * self.STROKE_MM
        )
        if self.g < 0.95:
            x_t = 0.0  # the housing wall closes the rod aperture at LOAD (I9)
        x_prev = self.x
        if t < self.powered_until:
            self.x += max(-self.ROD_RATE * ms, min(self.ROD_RATE * ms, x_t - self.x))
        self.strokes += (
            x_prev < self.commit_mm <= self.x
        )  # rod passes the commit stroke (with or without a ball)
        if self.g < 0.05:
            self.at_load_ms += 1
            if self.at_load_ms >= self.FILL_MS and not self.pocket and self.hopper > 0:
                self.pocket, self.hopper = True, self.hopper - 1
        else:
            self.at_load_ms = 0
        if (
            self.pocket
            and self.g > 0.95
            and self.x >= self.commit_mm
            and self.t_nip is None
        ):  # in the nip: leaves regardless of later commands
            self.pocket = False
            f = self.model.c.feed
            self.t_nip = (
                t
                + max(f.t_exit_ms + self.rng.normal(0.0, f.t_exit_sigma_ms), 1.0)
                / 1000.0
            )
        if self.t_nip is not None and t >= self.t_nip:
            shot = self.model.sample_shot(self.rng)
            self.shots.append((t, shot))
            if self.rig is not None:
                self.rig.fire(shot)
                self.fire_calls += 1
            t0 = t + (self.MUZ_BEAM_M - shot.d / 2) / shot.v0
            self.pulses.append((t0, t0 + shot.d / shot.v0, shot.v0))
            self.t_nip = None
        return (
            self.t_nip is not None
        )  # True while the ball is in contact with the wheels


class Plant2:
    def __init__(self, fsm=None, calib=None, rig=None, seed=1, hopper=24, cfg=None):
        self.calib = calib or Calib()
        rng = np.random.default_rng(seed)
        self.model = LauncherModel(self.calib, rng)
        self.feeder = FeederPlant(self.model, rig, rng, hopper)
        self.fsm = fsm or StubFsm(
            rpm_max=self.calib.launcher.rpm_max,
            kv=self.calib.wheel.kv,
            v_bus=self.calib.wheel.v_bus,
        )
        self.cfg = cfg or {}
        self.t_ms = 0
        self.rpm_log = []  # (t_ms, rpm_top, rpm_bot) true speeds, every tick
        self.outs = []

    def tick(self, **inputs):
        t = self.t_ms
        rt, rb = (
            self.model.telemetry()
            if not inputs.pop("tel_lost", False)
            else (None, None)
        )
        fin = FeederIn(
            t_ms=t,
            rpm_top=rt,
            rpm_bot=rb,
            cfg=self.cfg,
            **self.feeder.beams(t / 1000.0, 0.001),
            **inputs,
        )
        out = self.fsm.step(fin)
        loaded = self.feeder.step(t / 1000.0, 0.001, out)
        self.model.step_wheels(
            out.dshot_top, out.dshot_bot, 0.001, load=1.0 if loaded else 0.0
        )
        self.rpm_log.append((t, *self.model.rpm))
        self.outs.append(out)
        self.t_ms += 1
        return out

    def run(self, ms, **inputs):
        return [self.tick(**inputs) for _ in range(int(ms))]


# ---- the real feeder FSM behind Plant2 -------------------------------------------------------------------------------------------------
class RealFsm:
    """napl.cuas2.fsm.FeederFsm (the python twin of the C feeder, checked against it vector by vector) behind Plant2's FeederIn / FeederOut: string acks, named
    events, the servo pulses mapped to the plant's 1000/2000 us and 1150/1700 us conventions. `Plant2(fsm=RealFsm())` replaces StubFsm."""

    EVENTS = {
        p2.EV_SHOT: "SHOT",
        p2.EV_FEED_ABORT: "FEED_ABORT",
        p2.EV_MUZZLE: "MUZZLE",
        p2.EV_JAM: "JAM",
    }

    def __init__(self, cfg=None):
        self.f = _F.FeederFsm()
        self.fid = 0
        self.cfg = dict(cfg or {})  # config overrides under whatever Plant2 passes

    def step(self, i: FeederIn) -> FeederOut:
        c = dict(p2.CFG_DEFAULTS2)
        c.update(self.cfg)
        c.update(i.cfg or {})
        if i.cmd == "FIRE":
            self.fid += 1
        rpm = lambda v: None if v is None else int(round(v))
        r = self.f.step(
            _F.FeederIn(
                t_ms=i.t_ms, rpm_top=rpm(i.rpm_top), rpm_bot=rpm(i.rpm_bot), beam_breech=i.beam_breech, beam_muzzle=i.beam_muzzle, gate_home=i.gate_home,
                push_home=i.push_home, estop_open=i.estop_open, key_on=i.key_on, hb_ok=i.hb_ok, deadman=i.deadman, inhibit=i.inhibit, in_sector=i.in_sector,
                cmd=i.cmd, speed_req_pm=i.speed_req_pm, diff_pm=i.diff_pm, cfg=c, fire_id=self.fid,
            )
        )  # fmt: skip
        sf = r.status_fields
        return FeederOut(
            dshot_top=r.dshot_top, dshot_bot=r.dshot_bot,
            gate_us=GATE_BORE_US if r.gate_us == c[p2.CFG_GATE_BORE_US] else GATE_LOAD_US,
            push_us=PUSH_EXT_US if r.push_us == c[p2.CFG_PUSHER_EXT_US] else PUSH_RET_US,
            feed_req=r.feed_req, kick=r.kick,
            status_fields={**sf, "state": sf["gate_state"], "ready": bool(sf["ready_flags"] & p2.RDY_READY), "wheels_ready": bool(sf["wheels_ready"])},
            events=[(self.EVENTS[code], self.fid) for code, _ in r.events if code in self.EVENTS],
            ack=None if r.ack is None else p2.RESULT_NAMES[r.ack],
        )  # fmt: skip


# ---- the whole-MCU twin: lite's SimRP2040 safety FSM + the cuas2 feeder ------------------------------------------------------------------
class _NoWeapon:
    def inputs(self, t):
        return dict(
            rpm_top=None,
            rpm_bot=None,
            beam_breech=False,
            beam_muzzle=False,
            gate_home=True,
            push_home=True,
            muzzle_dt_us=0,
        )

    def step(self, out, t, rail=True):
        pass


DEBOUNCE_MS = 10  # E-stop released / key on must be stable this long; open / off acts at once (as the firmware)
HWC_SKEW_MS = 5  # FDR_HWC_SKEW_MS: both telemetry streams lost within this of each other = weapon power lost


class Sim2RP2040(_lite.SimRP2040):
    """The simulated cuas2 Pico: lite's SimRP2040 (ARM, 300 ms heartbeat watchdog, fire_id ring, shot cap, fire sector, E-stop / key, axes, config, STATUS
    pacing) with its pusher and flywheel replaced by napl.cuas2.fsm.FeederFsm exactly as firmware/cuas2-rp2040/src/fsm.c wires the C feeder in (FIRE gates in
    the same order, provisional BUSY in the ring, FIRE answered by the feeder, 45-byte STATUS, FLYWHEEL tail, cuas2 config keys and ranges, hardware-counter
    label, KICK supervisor).

        twin = Sim2RP2040(KinematicPlant(), link, weapon=plant)    # plant: .inputs(t_ms) -> rpm/beam dict, .step(out, t_ms, rail) (tests' fwharness.Plant)
        twin.step(now)

    tests/cuas2/test_sim2.py replays scripted scenarios and a fuzz sample against the C core and compares the acks, events and STATUS words tick by tick.
    Known gaps (not modelled): the 16 KiB receive queue (USB overflow fault), flash write failures, the DEBUG boot note, the Pico-only LEDs / buzzer."""

    def __init__(self, plant, link, *args, weapon=None, **kw):
        self.last_hb_ms = self.arm_deadline_ms = self.deadman_low_ms = None
        self.hb_run_ms0 = self.fw_refresh_ms = self.next_status_ms = 0
        super().__init__(plant, link, *args, **kw)
        home = {k: self.cfg[k] for k in (p2.CFG_PAN_HOME_OFF, p2.CFG_TILT_HOME_OFF)}
        self.cfg = dict(p2.CFG_DEFAULTS2)
        self.cfg.update(home)
        self.flash = dict(self.cfg)
        self.weapon = weapon if weapon is not None else _NoWeapon()
        self.dec = p2.Decoder2()
        self.fdr = _F.FeederFsm()
        self.fo = None
        self.wi = None
        self.fire_pending = None  # (fire_id, seq, ring entry)
        self.clear_pending = False
        self.diff_ovr = None
        self.muz_key = 0
        self.muz_prev = 0
        self.conf_arm = 0
        self._in_init = False
        self.est_closed_ms = self.key_on_ms = 0
        self.kick_level = self.kick_ok = 0
        self.last_toggle = 0
        self._prev_kick = None
        self.rail_hw = True

    def reboot(self):
        w = self.weapon
        super().reboot()
        self.weapon = w

    # ---- config ------------------------------------------------------------------------------------------------------------------------
    def _image_crc(self):
        import struct
        import zlib

        body = struct.pack(
            "<II41i", _lite.CFG_IMAGE_MAGIC, 3, *(self.flash[k] for k in range(1, 42))
        )
        return zlib.crc32(body) & 0xFFFFFFFF

    def _hello(self, f, h):
        res = p2.hello_result(h)
        crc = self._image_crc()
        if res:
            self.version_bad = True
        else:
            self.version_bad = False
            if self.session != h.session:
                self.session = h.session
                self.shots_arm = 0
                self.hb_run = 0
                self.last_hb_ms = None
                if self.state != p2.SAFE:
                    self._to(p2.SAFE, p2.R_NEW_SESSION)
            self.hello_done = True
        self._send(p2.HelloAck(p2.MAJOR, p2.MINOR, res, 2, 0, 0, self.uid, crc))

    @staticmethod
    def _cfg_valid(key, v):
        if key in (p2.CFG_PAN_SPR, p2.CFG_TILT_SPR):
            return v > 0
        if key == p2.CFG_DIR_INVERT:
            return 0 <= v <= 3
        if key == p2.CFG_HARD_SHOT_CAP:
            return 0 <= v <= 255
        if key == p2.CFG_HARD_ARM_TIMEOUT:
            return 0 <= v <= 65535
        if key in (p2.CFG_MIN_SHOT_INTERVAL_MS, p2.CFG_SPINUP_MS):
            return 0 <= v <= 60000
        if key == p2.CFG_HOMING_SPEED:
            return 0 < v <= 3000
        lo, hi = p2.CFG_RANGES.get(key, (None, None))
        return lo is None or lo <= v <= hi

    def _cfg_set(self, f, m):
        if self.state not in (p2.SAFE, p2.IDLE):
            return self._ack(f, p2.BAD_STATE)
        if not 1 <= m.key < p2.CFG_COUNT:
            return self._ack(f, p2.BAD_ARG)
        if m.key in p2.MAINTENANCE_KEYS and self.state != p2.SAFE:
            return self._ack(f, p2.BAD_STATE)
        v = m.value
        if m.key == p2.CFG_WD_MS:
            v = max(1, min(300, v))
        if not self._cfg_valid(m.key, v):
            return self._ack(f, p2.BAD_ARG)
        if m.key == p2.CFG_MIN_SHOT_INTERVAL_MS:
            v = max(v, p2.MIN_SHOT_INTERVAL_FLOOR)
        self.cfg[m.key] = v
        if m.key in p2.SECTOR_KEYS:
            self.sector_set.add(m.key)
        if m.key in (p2.CFG_PAN_SPR, p2.CFG_TILT_SPR):
            a = self.ax[m.key - p2.CFG_PAN_SPR]
            a.spr, a.homed = (
                v,
                False,
            )  # the position scale changed: the old home no longer holds
        if m.key == p2.CFG_DIR_INVERT:
            for i, a in enumerate(self.ax):
                a.sign = -1.0 if v >> i & 1 else 1.0
        self._ack(f, p2.OK)

    def _cfg_get(self, f, m):
        if not 1 <= m.key < p2.CFG_COUNT:
            return self._ack(f, p2.BAD_ARG)
        self._send(p2.CfgVal(m.key, self.cfg[m.key]))

    def _clear_fault(self, f, m):
        if self.state not in (p2.SAFE, p2.FAULT):
            return self._ack(f, p2.BAD_STATE)
        if self.estop_open:
            return self._ack(f, p2.INTERLOCK)
        self.fault_bits = self.faults_reported = self.diag_ms = 0
        self.clear_pending = True  # also clears the jam / no_exit latches in the feeder (it re-homes the gate)
        if self.state == p2.FAULT:
            self._to(p2.SAFE, p2.R_DISARM)
        self._ack(f, p2.OK)

    # ---- FLYWHEEL / FIRE ---------------------------------------------------------------------------------------------------------------
    def _flywheel_cmd(self, f, m):
        if m.speed_permille > 1000:
            return self._ack(f, p2.BAD_ARG)
        self.diff_ovr = (
            None
            if m.diff_pm is None
            else max(-p2.DIFF_MAX_PM, min(p2.DIFF_MAX_PM, m.diff_pm))
        )
        if m.speed_permille == 0:
            self.fw_req = 0.0
            if self.state == p2.ENGAGING:
                self._to(p2.ARMED, p2.R_NONE)
            return
        if not self.rail_on:
            return self._ack(f, p2.INTERLOCK)
        if not self.armed_state:
            return self._ack(f, p2.BAD_STATE)
        self.fw_req, self.fw_refresh_ms = float(m.speed_permille), self.ms
        if self.state == p2.ARMED:
            self._to(p2.ENGAGING, p2.R_NONE)

    def _in_sector(self):
        """fsm.c in_sector(): the fire sector shrunk on both ends by |axis speed| * t_exit (v deg/s * 8 ms = v * 8 mdeg)."""
        c, (pa, ti) = self.cfg, self.ax
        pan, tilt = pa.pos * 1000.0, ti.pos * 1000.0
        mp, mt = abs(pa.v) * _F.SECTOR_T_EXIT_MS, abs(ti.v) * _F.SECTOR_T_EXIT_MS
        return (
            c[p2.CFG_FIRE_PAN_MIN] + mp <= pan <= c[p2.CFG_FIRE_PAN_MAX] - mp
            and c[p2.CFG_FIRE_TILT_MIN] + mt <= tilt <= c[p2.CFG_FIRE_TILT_MAX] - mt
        )

    def _precheck(self, m):
        """fire_precheck(): the gates owned by the safety FSM, in the firmware's order; the feeder's follow."""
        if self.state == p2.FIRING or self.fire_pending:
            return p2.BUSY
        if self.state != p2.ENGAGING:
            return p2.BAD_STATE
        if m.n != 1:
            return p2.BAD_ARG
        if self.fault_bits:
            return p2.JAMMED
        if not self.rail_on:
            return p2.INTERLOCK
        if not self.hb_ok or not self.deadman_ok or self.inhibit:
            return p2.INHIBIT
        if not all(a.homed for a in self.ax):
            return p2.NOT_HOMED
        if not self.sector_ok:
            return p2.NO_SECTOR
        if not self._in_sector():
            return p2.RANGE
        if self.shots_arm + 1 > self.shot_cap:
            return p2.CAP
        return p2.OK

    def _fire(self, f, m):
        for fid, res, _ in self.fire_ids:
            if fid == m.fire_id:  # retransmission: the old answer, nothing shot
                return self._send(p2.Ack(f.id, f.seq, res, 0))
        res = self._precheck(m)
        ent = [
            m.fire_id,
            res or p2.BUSY,
            0,
        ]  # provisional BUSY until the feeder has decided
        self.fire_ids.append(ent)
        if res:
            return self._ack(f, res)
        self.fire_pending = (m.fire_id, f.seq, ent)

    # ---- the tick ----------------------------------------------------------------------------------------------------------------------
    def _tick(self):
        data = self.link.mcu_read(self.t)
        if data:
            self._rx(data)
        if hasattr(self.plant, "advance"):
            self.plant.advance(_lite.SUB)
        self.wi = self.weapon.inputs(self.ms)
        self._inputs()
        self._hw_counter_note()
        self._feeder_stage()
        self._watchdog()
        self._timers()
        self._axes()
        if self.hello_done and self.ms >= self.next_status_ms:
            self.next_status_ms += 10
            if (
                self.ms >= self.next_status_ms
            ):  # late (the link was down): no catch-up burst
                self.next_status_ms = self.ms + 10
            self._send(self._status())
        self._outputs()

    def _inputs(self):
        e_raw, k_raw = bool(self.plant.estop_open), bool(self.plant.key_on)
        if not self._in_init:  # the first sample is the initial condition, not an edge
            self._in_init = True
            self.estop_open, self.key_on = e_raw, k_raw
            self.est_closed_ms = 0 if e_raw else DEBOUNCE_MS
            self.key_on_ms = DEBOUNCE_MS if k_raw else 0
        self.est_closed_ms = 0 if e_raw else min(self.est_closed_ms + 1, DEBOUNCE_MS)
        self.key_on_ms = 0 if not k_raw else min(self.key_on_ms + 1, DEBOUNCE_MS)
        e, k = self.est_closed_ms < DEBOUNCE_MS, self.key_on_ms >= DEBOUNCE_MS
        if e != self.estop_open:
            self.estop_open = e
            self._event(p2.EV_ESTOP_EDGE, int(e))
            if e:
                self.fw_req = self.fw = 0.0
                self._to(
                    p2.SAFE, p2.R_ESTOP
                ) if self.state != p2.SAFE else self._estop_while_safe()
        if k != self.key_on:
            self.key_on = k
            self._event(p2.EV_KEY_EDGE, int(k))
            if not k and self.state != p2.SAFE:
                self._to(p2.SAFE, p2.R_KEY_OFF)
        if not self.rail_on:
            self.fw = self.fw_req = 0.0
            self.pusher_ext = False

    def _hw_counter_note(self):
        """Per-key-session count of raw muzzle-beam rising edges (the 4017 counts them all); the key resets it."""
        mz = int(bool(self.wi["beam_muzzle"]))
        if not self.key_on:
            self.muz_key = 0
        elif mz and not self.muz_prev and self.muz_key < 0xFFFF:
            self.muz_key += 1
        self.muz_prev = mz

    def _feeder_stage(self):
        w, c = self.wi, self.cfg
        fp = self.fire_pending
        if (
            fp and self.fault_bits
        ):  # a fault latched after the precheck, in this tick: refuse here as well
            self.fire_pending = None
            fp[2][1] = p2.JAMMED
            self.refusals.append((self.t, p2.FIRE, p2.JAMMED))
            self._send(p2.Ack(p2.FIRE, fp[1], p2.JAMMED, 0))
            fp = None
        speed = (
            int(math.floor(self.fw_req + 0.5))
            if (self.rail_on and self.armed_state)
            else 0
        )
        r = self.fdr.step(
            _F.FeederIn(
                t_ms=self.ms, rpm_top=w["rpm_top"], rpm_bot=w["rpm_bot"], beam_breech=w["beam_breech"], beam_muzzle=w["beam_muzzle"], gate_home=w["gate_home"],
                push_home=w["push_home"], estop_open=self.estop_open, key_on=self.key_on, hb_ok=self.hb_ok, deadman=self.deadman_ok, inhibit=self.inhibit,
                in_sector=self._in_sector(), cmd="FIRE" if fp else ("CLEAR" if self.clear_pending else None), speed_req_pm=speed,
                diff_pm=self.diff_ovr if self.diff_ovr is not None else c[p2.CFG_DIFF_PM], cfg=c, fire_id=fp[0] if fp else self.cur_fire_id,
                muzzle_dt_us=w["muzzle_dt_us"],
            )
        )  # fmt: skip
        self.clear_pending = False
        self.fo = r
        sf = r.status_fields
        fb = sf["fault_bits"]
        a = self.fdr.age
        if ((fb & p2.FAULT_TELEMETRY_LOST) and not (self.fault_bits & p2.FAULT_TELEMETRY_LOST) and self.key_on and not self.estop_open
                and c[p2.CFG_HW_COUNTER_N] > 0 and self.muz_key >= c[p2.CFG_HW_COUNTER_N] and abs(a[0] - a[1]) <= HWC_SKEW_MS):  # fmt: skip
            self.fault_bits |= (
                p2.FAULT_HW_COUNTER
            )  # the counter's clamp, inferred from the power loss (fsm.c feeder_stage)
            self._event(p2.EV_HW_COUNTER, self.muz_key)
        self.fault_bits |= fb & p2.FDR_FAULT_MASK
        self.faults_reported |= fb & p2.FDR_FAULT_MASK
        if self.shots_arm == 0:
            self.conf_arm = 0
        if fp:  # answer the FIRE the feeder just decided
            res = r.ack if r.ack is not None else p2.BAD_STATE
            self.fire_pending = None
            fp[2][1] = res
            if res:
                self.refusals.append((self.t, p2.FIRE, res))
            self._send(p2.Ack(p2.FIRE, fp[1], res, 0))
            if res == p2.OK:
                self.last_fire_id = self.cur_fire_id = fp[0]
                self._to(p2.FIRING, p2.R_NONE)
        aborted = sector = False
        for code, arg in r.events:
            if code == p2.EV_SHOT:
                self.ball_idx = 0
                self.shots_arm += 1
                self.shots_total += 1
                self.last_ball_t = self.t
                self.shots.append((self.t, self.cur_fire_id, 0))
            elif code == p2.EV_MUZZLE:
                self.conf_arm += 1
            elif code == p2.EV_FEED_ABORT:
                aborted = True
            elif code == p2.EV_SECTOR_ABORT:
                sector = True
            self._event(code, arg)
        if aborted and self.state == p2.FIRING:
            if sector:
                self.fw_req = 0.0  # fault row 25: ARMED, no flywheel request
            self._burst_done()
            if sector and self.state == p2.ENGAGING:
                self._to(p2.ARMED, p2.R_NONE)
        elif self.state == p2.FIRING and not sf["feed_busy"]:
            self._burst_done()
        self.fw = sf["throttle_mean_pm"]
        self.pusher_ext = bool(
            sf["feed_busy"]
            and r.push_us == c[p2.CFG_PUSHER_EXT_US]
            and r.push_us != c[p2.CFG_PUSHER_RET_US]
        )

    # ---- time in integer milliseconds (lite compares float seconds: on a boundary tick that differs from the firmware's MS_SINCE by one) ----
    @property
    def hb_ok(self):
        return (
            self.hello_done
            and self.last_hb_ms is not None
            and self.ms - self.last_hb_ms <= self.cfg[p2.CFG_WD_MS]
        )

    def _heartbeat(self, f, m):
        if (
            self.last_hb_ms is not None
            and self.ms - self.last_hb_ms <= self.cfg[p2.CFG_WD_MS]
        ):
            if self.hb_run == 0:
                self.hb_run_ms0 = self.ms
            self.hb_run = min(self.hb_run + 1, 0xFFFF)
        else:
            self.hb_run, self.hb_run_ms0 = 1, self.ms
        self.last_hb_ms = self.ms
        self.hb_flags = m.flags

    def _arm(self, f, m):
        if m.mode == 0:
            if self.armed_state:
                self._to(p2.IDLE, p2.R_DISARM)
            return self._ack(f, p2.OK)
        if m.mode != 1 or m.shot_cap == 0 or m.timeout_s == 0:
            return self._ack(f, p2.BAD_ARG)
        if self.state != p2.IDLE:
            return self._ack(f, p2.BAD_STATE)
        if self.estop_open or not self.key_on:
            return self._ack(f, p2.INTERLOCK)
        if self.fault_bits:
            return self._ack(f, p2.BAD_STATE)
        if not self.hb_ok or not self.deadman_ok:
            return self._ack(f, p2.INHIBIT)
        if not all(a.homed for a in self.ax):
            return self._ack(f, p2.NOT_HOMED)
        if not self.sector_ok:
            return self._ack(f, p2.NO_SECTOR)
        hcap, hto = self.cfg[p2.CFG_HARD_SHOT_CAP], self.cfg[p2.CFG_HARD_ARM_TIMEOUT]
        self.shot_cap = min(m.shot_cap, max(hcap, 0))
        self.arm_deadline_ms = self.ms + 1000 * min(m.timeout_s, max(hto, 0))
        self.shots_arm = 0
        self.deadman_low_ms = None
        self._to(p2.ARMED, p2.R_OPERATOR_ARM)
        self._ack(f, p2.OK, self.shot_cap)

    def _watchdog(self):
        wd = self.cfg[p2.CFG_WD_MS]
        if (
            self.hello_done
            and self.last_hb_ms is not None
            and self.ms - self.last_hb_ms > wd
        ):
            self.hb_run = 0
            if self.state == p2.IDLE or self.armed_state:
                self.wd_tripped = True
                self._event(p2.EV_WATCHDOG, self.ms - self.last_hb_ms)
                self._to(p2.SAFE, p2.R_WATCHDOG)
        if self.armed_state and self.shots_arm > self.shot_cap:
            self.fault_bits |= p2.FAULT_SHOT_COUNTER  # internal assertion
        if self.diag_pin and self.steppers_enabled:
            self.diag_ms = min(self.diag_ms + 1, 255)
            if self.diag_ms > _lite.DIAG_FAULT_MS:
                self.fault_bits |= p2.FAULT_DRIVER_DIAG
        else:
            self.diag_ms = 0
        fresh = self.fault_bits & ~self.faults_reported
        for b in range(6):
            if fresh >> b & 1:
                self._event(p2.EV_FAULT, b)
        self.faults_reported |= fresh
        if self.fault_bits and self.state != p2.FAULT:
            self._to(p2.FAULT, p2.R_FAULT)
        if (
            self.state == p2.SAFE
            and self.hello_done
            and not self.version_bad
            and not self.fault_bits
            and not self.estop_open
            and self.hb_run >= 20
            and self.ms - self.hb_run_ms0 >= 1000
        ):
            self.wd_tripped = False
            self._to(p2.IDLE, p2.R_LINK_UP)

    def _timers(self):
        if self.armed_state and self.ms - self.arm_deadline_ms >= 0:
            self._to(p2.IDLE, p2.R_ARM_TIMEOUT)
        if (
            self.state in (p2.ENGAGING, p2.FIRING)
            and self.fw_req > 0
            and self.ms - self.fw_refresh_ms > 500
        ):
            self.fw_req = 0.0
            if self.state == p2.ENGAGING:
                self._to(p2.ARMED, p2.R_NONE)
        if self.armed_state:
            if self.deadman_ok:
                self.deadman_low_ms = None
            else:
                if self.deadman_low_ms is None:
                    self.deadman_low_ms = self.ms
                if self.ms - self.deadman_low_ms > 1000 and self.fw_req > 0:
                    self.fw_req = 0.0
                    if self.state == p2.ENGAGING:
                        self._to(p2.ARMED, p2.R_NONE)

    def _flywheel(self):  # replaced by _feeder_stage (the launcher drives the ESCs)
        pass

    def _pusher(self):
        pass

    def _status(self):
        s = super()._status()
        r, sf = self.fo, self.fo.status_fields
        rt, rb = sf["rpm_top"], sf["rpm_bot"]
        ready = sf["ready_flags"]
        if (ready & p2.RDY_READY) and not (
            self.state == p2.ENGAGING and self.shots_arm < self.shot_cap
        ):
            ready &= ~p2.RDY_READY
        kw = {f.name: getattr(s, f.name) for f in dataclasses.fields(s)}
        kw.update(fw_duty=int(round(self.fw)), fw_rpm=0xFFFF if 0xFFFF in (rt, rb) else (rt + rb) // 2, rpm_top=rt, rpm_bot=rb, rpm_set=sf["rpm_set"],
                  beams=sf["beams"], gate_state=sf["gate_state"], ready_flags=ready, last_v_dmps=sf["last_v_dmps"], last_dip_pm=sf["last_dip_pm"],
                  shots_confirmed=sf["shots_confirmed"] & 0xFFFF, feed_ms=sf["feed_ms"])  # fmt: skip
        left = 0
        if self.armed_state and self.arm_deadline_ms - self.ms > 0:
            left = (self.arm_deadline_ms - self.ms + 999) // 1000
        kw["arm_left_s"] = min(left, 0xFFFF)
        return p2.Status2(**kw)

    def _outputs(self):
        """KICK (I15) and the supervisor's relay model, then the weapon plant gets this tick's outputs."""
        r, ms = self.fo, self.ms
        self.kick_ok = int(bool(
            r.kick and not self.fault_bits and self.state <= p2.FAULT and (not self.armed_state or self.shots_arm <= self.shot_cap)
            and self.fdr.muz_all <= self.fdr.feeds))  # fmt: skip
        if self.kick_ok and ms % 5 == 0:
            self.kick_level ^= 1
        if self.kick_ok and self.kick_level != self._prev_kick:
            self.last_toggle = ms
        self._prev_kick = self.kick_level
        self.rail_hw = bool(
            self.plant.key_on
            and not self.plant.estop_open
            and ms - self.last_toggle < 150
        )
        self.weapon.step(
            SimpleNamespace(
                dshot_top=r.dshot_top,
                dshot_bot=r.dshot_bot,
                feed_req=r.feed_req,
                push_us=r.push_us,
                gate_us=r.gate_us,
            ),
            ms,
            rail=self.rail_hw,
        )
