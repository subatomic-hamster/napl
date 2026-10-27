"""Plant behind the cuas2 feeder FSM (docs/cuas2/spec.md 7.4): flywheels, limiter, gate and pusher servos, beams, the ball, and the launcher shot.

    plant = Plant2(rig=None, calib=calib.load())      # fsm defaults to StubFsm (below) until napl.cuas2.fsm (FW) is merged
    out = plant.tick(cmd="FIRE", speed_req_pm=1000)   # one 1 ms step: FeederIn from the plant -> fsm.step -> FeederOut applied to the plant
    plant.run(500, speed_req_pm=1000)

`fsm` is anything with `step(FeederIn) -> FeederOut`; FeederIn / FeederOut below carry the field names fixed by spec section 12 (FW's classes have the same
fields, so Plant2 only reads/writes attributes). `StubFsm` is a minimal feeder + wheel controller for the plant tests, NOT the safety FSM: it has none of
the fault latches, the sector abort timing of 5.4 beyond a commit check, or the config handling of the real one.

Plant model (7.4): wheels = LauncherModel.step_wheels behind the DShot values; the limiter powers the pusher servo 250 ms after a FEED_REQ rising edge and
ignores edges for 330 +- 15 % (I14); gate = slew-limited servo (60 deg in 100 ms), blocked while the rod is inside the bore; rod = 32 mm stroke at 0.4 mm/ms,
only moves while powered, cannot extend unless the gate is at BORE (I9); the ball falls into the pocket after 20 ms at LOAD; the breech beam reads the staged
ball; once the rod passes the commit stroke (t_commit_ms x speed) the ball is in the nip and leaves t_exit_ms (+ N(0, sigma)) later: LauncherModel.sample_shot
(wheels dip), rig.fire(shot), and the muzzle beam pulses for d/v starting when the ball front reaches the beam 55 mm after the nip centre. The beam level the
FSM sees at each 1 ms tick is latched high if any pulse overlapped that tick (the RP2040 timestamps the edges with a 1 us IRQ).
"""

from collections import deque
from dataclasses import dataclass, field

import numpy as np

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


class StubFsm:
    """Wheel PI + feeder cycle INIT -> HOMING -> LOADING -> TO_BORE -> STAGED -> PUSH -> EXIT_WAIT -> RETRACT -> TO_LOAD (spec 5.5), 1 ms calls."""

    def __init__(
        self,
        rpm_max=17500.0,
        window_pm=10,
        settle_ms=100,
        min_interval_ms=450,
        kp=8.0,
        ki=40.0,
        ramp_rpm_s=40000.0,
        kv=1800.0,
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
            if t - self.t0 < 45 and not (
                i.in_sector and i.deadman and not i.inhibit and ready_w
            ):  # before commit: retract, no ball leaves
                self.st, self.t0, o.push_us = "RETRACT", t, PUSH_RET_US
                o.events.append(("FEED_ABORT", self.fire_id))
            elif t - self.t0 >= 45:
                self.st = "EXIT_WAIT"
        elif s == "EXIT_WAIT":
            if i.beam_muzzle:
                o.events.append(("MUZZLE", self.fire_id))
                self.st, self.t0, o.push_us = "RETRACT", t, PUSH_RET_US
            elif t - self.t0 > 250 + 45:
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
    ROD_RATE, STROKE_MM, CLEAR_MM = (
        0.4,
        32.0,
        6.0,
    )  # mm/ms, stroke, rod tip outside the valve body below this
    MUZ_BEAM_M = 0.055  # beam after the nip centre
    FILL_MS = 20  # the ball falls into the pocket after this long at LOAD

    def __init__(self, model: LauncherModel, rig=None, rng=None, hopper=24):
        self.model, self.rig = model, rig
        self.rng = rng if rng is not None else np.random.default_rng(2)
        c = model.c
        self.commit_mm = self.ROD_RATE * c.feed.t_commit_ms
        self.hold_s = 0.33 * (1 + self.rng.uniform(-0.15, 0.15))
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
        self.windows = 0  # FEED_REQ edges the limiter accepted (each powers the pusher servo for 250 ms)

    @property
    def breech(self):
        return self.pocket and self.g > 0.95 and self.x < self.commit_mm

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
                self.powered_until, self.hold_until = t + 0.25, t + self.hold_s
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
