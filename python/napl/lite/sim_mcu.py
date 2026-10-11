"""SimRP2040: the simulated Pico. Implements the MCU side of docs/cuas-lite/spec.md sections 2-3 byte for byte (same protocol.py as the host) so the
sim measures the real gate logic: the six-state machine, the 300 ms heartbeat watchdog, ARM preconditions + hard shot cap / timeout, FIRE gating
(INHIBIT / DEADMAN / homed / tilt / spin-up / cap / interval, fire_id de-duplication), a 1 kHz velocity- and acceleration-limited position servo with
the 25 ksteps/s step-rate cap, hall homing, flywheel lag, pusher timing, E-stop and key inputs, config + flash image, faults.

    twin = SimRP2040(plant, link)            # plant: KinematicPlant (pure python) or sim_io.NaplPlant (drives the napl servos / launcher)
    twin.step(now)                           # advance the MCU to `now` (seconds) in 1 ms ticks; reads link.mcu_read, writes link.mcu_write

The link is the MCU's end: `mcu_read(t) -> bytes`, `mcu_write(data, t)` (LoopbackLink in-process, PtyMcu over a pty). Time is whatever clock the harness
passes to step(): sim time in lockstep (the 300 ms watchdog counts sim time), wall time in HIL.

Aligned with the firmware (firmware/cuas-lite-rp2040/src, divergence table in its README), which is what runs on the turret:
  - spun = applied duty >= requested for >= spinup_ms since the request; the applied duty ramps up 4 permille/ms and drops at once (no rotor lag);
  - a burst ended by a lost flywheel refresh / dead-man / inhibit goes to ARMED (ENGAGING needs a flywheel request);
  - the planner brakes its P term to sqrt(2*amax*err) (a trapezoid), so a fast move does not overshoot the target;
  - lost steps raise the TMC2209 DIAG pin (joint more than 4 full steps from the command); FAULT_DRIVER_DIAG follows after 50 ms of DIAG; no resync;
  - homing works from any start: SEEK_POS at 100 deg/s for the negative half-range + 10 deg, then SEEK_NEG for the full range + 20 deg, any hall hit
    goes through CLEAR (out the negative edge) to the slow APPROACH, so the edge that sets home is always the rising edge met moving positive;
  - STATUS every 10 ms from the first one (no catch-up burst); cfg_crc = CRC-32 of the 88-byte config image; home offsets default to 0
    (a hall window of +-1 deg is centred on q_hall: home_off_deg=(-1, -1) puts home-frame 0 at its centre, as a calibrated unit).
Deviations from a literal reading of the spec, for the firmware worker to know (all marked "twin" in the code):
  - key-off / E-stop / watchdog / new session cut the flywheel PWM to 0 in the same tick (a coasting rotor is not modelled);
  - ACK(ARM, ok).info carries the active shot cap, ACK(HOME, ok).info the axis mask;
  - heartbeats before the first HELLO are ignored; every command but HELLO is answered BAD_STATE before the first HELLO.
"""

import math
import os
import struct
import zlib
from collections import deque

from . import protocol as p
from .protocol import (
    ARMED,
    ENGAGING,
    FAULT,
    FIRING,
    IDLE,
    SAFE,
    Ack,
    Aim,
    Arm,
    CfgGet,
    CfgSet,
    CfgVal,
    Event,
    Fire,
    Flywheel,
    Heartbeat,
    Hello,
    HelloAck,  # fmt: skip
    Home,
    Status,
)

STEP_RATE_MAX = 25000.0  # steps/s: PIO / timer ISR limit (spec 2 rates)
KP = 20.0  # 1/s
SUB = 0.001
STATUS_PERIOD = 0.010
HOMING_TIMEOUT_S = 15.0
BACKOFF_DEG = 3.0
HALL_HALF_WIDTH_DEG = 1.0
FW_RAMP_PER_MS = 4.0  # permille per ms soft start (inrush limit), as the firmware
DIAG_FAULT_MS = 50
HOMING_SEARCH_DEG_S = 100.0
CFG_IMAGE_MAGIC, CFG_IMAGE_VERSION = 0x31434C43, 2
FW_REFRESH_S = 0.5
DEADMAN_LOW_S = 1.0
AIM_STALE_VFF_S, AIM_STALE_HOLD_S = 0.25, 1.0
FIRE_MEMORY = 8
FW_VERSION = (0, 1, 0)


class KinematicPlant:
    """Pure-python plant for tests and HIL smoke: joints follow the goal with a first-order lag, no physics. Fields the harness may set: estop_open,
    key_on. Records fire pulses."""

    def __init__(self, pan0=0.0, tilt0=0.0, tau=0.002):
        self.q = [pan0, tilt0]
        self.goal = [pan0, tilt0]
        self.torque = False
        self.estop_open = False
        self.key_on = True
        self.fire_pulses = []
        self.tau = tau
        self.t = 0.0

    def joint_deg(self):
        return self.q[0], self.q[1]

    def set_joint_goal(self, pan, tilt):
        self.goal = [pan, tilt]

    def set_torque(self, on):
        self.torque = bool(on)

    def set_flywheel(self, duty):
        self.duty = duty

    def fire_pulse(self):
        self.fire_pulses.append(self.t)

    def advance(self, dt):
        self.t += dt
        if self.torque:
            a = 1 - math.exp(-dt / self.tau)
            for k in (0, 1):
                self.q[k] += (self.goal[k] - self.q[k]) * a


class Axis:
    def __init__(self, name, spr):
        self.name, self.spr = name, spr
        self.pos = 0.0  # deg, home frame (float, quantised to microsteps on output)
        self.v = 0.0
        self.target = 0.0
        self.vff = 0.0
        self.vmax = 0.0
        self.amax = 0.0
        self.t_aim = None
        self.homed = False
        self.offset = 0.0  # joint = sign * pos + offset
        self.sign = 1.0
        self.homing = (
            None  # None | "seek_pos" | "seek_neg" | "clear" | "approach" | "backoff"
        )
        self.homing_t0 = 0.0
        self.marked = False
        self.mark = (
            0.0  # where the seek started / where the hall released while backing out
        )
        self.hall = False
        self.prev_hall = False
        self.cmd_joint = None  # last joint angle written to the plant
        self.stuck_t = 0.0


class SimRP2040:
    def __init__(
        self,
        plant,
        link,
        uid=bytes.fromhex("e66050a8d3156f21"),
        q_hall=(0.0, 0.0),
        step_loss=True,
        cfg_invalid=False,
        home_off_deg=(0.0, 0.0),
        status_from_joint=False,
    ):
        self.status_from_joint = status_from_joint
        self.plant, self.link, self.uid = plant, link, uid
        self.q_hall, self.step_loss = q_hall, step_loss
        self.diag_pin, self.diag_ms = False, 0
        self.faults_reported = 0
        self.cfg = dict(p.CFG_DEFAULTS)
        # twin: the hall window is 2 deg wide centred on q_hall; the edge the approach sees is half a window short of the centre, so a home offset of
        # -1 deg puts home-frame 0 at the window centre (cfg 7, 8 are calibrated that way on a real unit; the default is 0, as the firmware's)
        for k, d in (
            (p.CFG_PAN_HOME_OFF, home_off_deg[0]),
            (p.CFG_TILT_HOME_OFF, home_off_deg[1]),
        ):
            self.cfg[k] = round(
                d
                * self.cfg[p.CFG_PAN_SPR if k == p.CFG_PAN_HOME_OFF else p.CFG_TILT_SPR]
                / 360
            )
        self.flash = dict(self.cfg)
        self.ax = [
            Axis("pan", self.cfg[p.CFG_PAN_SPR]),
            Axis("tilt", self.cfg[p.CFG_TILT_SPR]),
        ]
        self.t = None
        self.t0 = None
        self.state = SAFE
        self.session = None
        self.hello_done = False
        self.version_bad = False
        self.last_hb_t = None
        self.hb_flags = 0
        self.hb_run, self.hb_run_t0 = 0, 0.0
        self.wd_tripped = False
        self.fault_bits = p.FAULT_CFG_INVALID if cfg_invalid else 0
        self.steppers_enabled = False
        self.estop_open = bool(getattr(plant, "estop_open", False))
        self.key_on = bool(getattr(plant, "key_on", True))
        plant.set_torque(False)
        self.fw_req = 0.0
        self.fw = 0.0
        self.fw_req_t = 0.0
        self.fw_refresh_t = 0.0
        self.shots_total = 0
        self.shots_arm = 0
        self.shot_cap = 0
        self.arm_deadline = 0.0
        self.deadman_low_since = None
        self.last_fire_id = 0
        self.fire_ids = deque(maxlen=FIRE_MEMORY)  # (fire_id, result, info)
        self.balls_left = 0
        self.ball_idx = 0
        self.cur_fire_id = 0
        self.phase = None  # None | "wait" | "ext" | "ret"
        self.phase_end = 0.0
        self.last_ball_t = -1e9
        self.pusher_ext = False
        self.tx_seq = 0
        self.tx_started = False
        self.dec = p.Decoder()
        self.bad_reported = 0
        self.bad_event_t = -1e9
        self.next_status = 0.0
        self.bootsel = False
        self.events = []  # every EVENT sent: (t, code, arg), for tests and the evaluator
        self.shots = []  # (t, fire_id, ball_index) at each extend-complete (ball fed)
        self.refusals = []  # (t, cmd_id, result)
        self.transitions = []  # (t, state, reason)
        if self.fault_bits:
            self.faults_reported = (
                self.fault_bits
            )  # the cfg-invalid fault is reported by the state, not by an EVENT
            self._to(FAULT, p.R_FAULT, quiet=True)

    # ---- helpers -----------------------------------------------------------------------------------------------------------------------
    @property
    def ms(self):
        return int(round((self.t - self.t0) * 1000)) & 0xFFFFFFFF

    def _send(self, m, t=None):
        if not self.tx_started:
            self.tx_started = True
            self.link.mcu_write(
                b"\0", self.t
            )  # resync byte: a receiver ignores everything up to the first 0x00
        self.link.mcu_write(p.encode(self.tx_seq, m), self.t if t is None else t)
        self.tx_seq = (self.tx_seq + 1) & 0xFF

    def _ack(self, f, result, info=0):
        if result:
            self.refusals.append((self.t, f.id, result))
        self._send(Ack(f.id, f.seq, result, info))

    def _event(self, code, arg):
        self.events.append((self.t, code, arg))
        if self.hello_done:
            self._send(Event(code, arg))

    @property
    def hb_ok(self):
        return (
            self.hello_done
            and self.last_hb_t is not None
            and (self.t - self.last_hb_t) * 1000 <= self.cfg[p.CFG_WD_MS]
        )

    @property
    def deadman_ok(self):
        return self.hb_ok and bool(self.hb_flags & p.HB_DEADMAN_OK)

    @property
    def inhibit(self):
        return bool(self.hb_flags & p.HB_FIRE_INHIBIT)

    @property
    def rail_on(self):
        return self.key_on and not self.estop_open

    @property
    def armed_state(self):
        return self.state in (ARMED, ENGAGING, FIRING)

    def spd(self, i):
        return (
            self.cfg[(p.CFG_PAN_SPR, p.CFG_TILT_SPR)[i]] / 360.0
        )  # microsteps per degree

    def _to(self, state, reason, quiet=False):
        """State change with the outputs of the new state: SAFE / FAULT / IDLE cut the flywheel PWM and pusher in the same tick."""
        old = self.state
        self.state = state
        if not self.armed_state:
            self._cut()
            self.shot_cap = 0
        if state in (SAFE, FAULT) and reason in (p.R_ESTOP, p.R_FAULT):
            self._steppers(False)
            for a in self.ax:
                a.homed, a.homing = False, None
                a.v = 0.0
        if state == SAFE and reason in (p.R_WATCHDOG, p.R_KEY_OFF, p.R_NEW_SESSION):
            self._hold()
        if state == IDLE:
            self._steppers(True)
        if state == SAFE:
            self.hb_run = 0  # back to IDLE only after a fresh second of heartbeats (the E-stop / key / session / watchdog cases alike)
        if state != old:
            self.transitions.append((self.t, state, reason))
            if not quiet:
                self._event(p.EV_STATE_CHANGE, state | (reason << 8))

    def _cut(self):
        self.fw_req = self.fw = 0.0
        self.plant.set_flywheel(0.0)
        self.balls_left, self.phase, self.pusher_ext = 0, None, False

    def _hold(self):
        for a in self.ax:
            a.target, a.vff, a.t_aim = a.pos, 0.0, None

    def _steppers(self, on):
        if on != self.steppers_enabled:
            self.steppers_enabled = on
            if on:  # twin: the steppers hold wherever the joint is; home-frame position 0 is that angle until homed
                for a, qi in zip(self.ax, self.plant.joint_deg()):
                    a.offset = qi - a.sign * a.pos
            self.plant.set_torque(on)
            if not on:
                for a in self.ax:
                    a.v = 0.0

    # ---- host -> MCU -------------------------------------------------------------------------------------------------------------------
    def _rx(self, data):
        for f in self.dec.feed(data):
            self._handle(f)
        bad = self.dec.bad_frames - self.bad_reported
        if bad and self.t - self.bad_event_t >= 1.0:
            self.bad_reported, self.bad_event_t = self.dec.bad_frames, self.t
            self._event(p.EV_BAD_FRAMES, bad)

    def _handle(self, f):
        if f.id not in p.HOST_IDS:
            return self._ack(f, p.UNKNOWN_CMD)
        if f.bad_arg:
            return self._ack(f, p.BAD_ARG)
        m = f.msg
        if f.id == p.HELLO:
            return self._hello(f, m)
        if not self.hello_done or self.version_bad:
            return self._ack(f, p.BAD_STATE)
        {p.HEARTBEAT: self._heartbeat, p.AIM: self._aim, p.HOME: self._home, p.FLYWHEEL: self._flywheel_cmd, p.FIRE: self._fire, p.ARM: self._arm,
         p.CFG_SET: self._cfg_set, p.CFG_GET: self._cfg_get, p.CFG_SAVE: self._cfg_save, p.CLEAR_FAULT: self._clear_fault, p.BOOTSEL: self._bootsel}[f.id](f, m)  # fmt: skip

    def _image_crc(self):
        """cfg_crc: CRC-32 of the stored config image (magic "CLC1", version, cfg 1..19 as i32 LE), as the firmware's flash sector."""
        body = struct.pack(
            "<II22i",
            CFG_IMAGE_MAGIC,
            CFG_IMAGE_VERSION,
            *(self.flash[k] for k in range(1, 23)),
        )
        return zlib.crc32(body) & 0xFFFFFFFF

    def _hello(self, f, h: Hello):
        res = p.hello_result(h)
        crc = self._image_crc()
        if res:
            self.version_bad = True
        else:
            self.version_bad = False
            if self.session != h.session:
                self.session = h.session
                self.shots_arm = 0
                self.hb_run = 0
                self.last_hb_t = None
                if self.state != SAFE:
                    self._to(SAFE, p.R_NEW_SESSION)
            self.hello_done = True
        self._send(HelloAck(p.MAJOR, p.MINOR, res, *FW_VERSION, self.uid, crc))

    def _heartbeat(self, f, m: Heartbeat):
        wd = self.cfg[p.CFG_WD_MS] / 1000.0
        if self.last_hb_t is not None and self.t - self.last_hb_t <= wd:
            if self.hb_run == 0:
                self.hb_run_t0 = self.t
            self.hb_run += 1
        else:
            self.hb_run, self.hb_run_t0 = 1, self.t
        self.last_hb_t = self.t
        self.hb_flags = m.flags

    def _aim(self, f, m: Aim):
        if self.state in (SAFE, FAULT):
            return self._ack(f, p.BAD_STATE)
        if min(m.pan_vmax, m.tilt_vmax, m.pan_amax, m.tilt_amax) == 0:
            return self._ack(f, p.BAD_ARG)
        lim = (
            (self.cfg[p.CFG_PAN_MIN], self.cfg[p.CFG_PAN_MAX]),
            (self.cfg[p.CFG_TILT_MIN], self.cfg[p.CFG_TILT_MAX]),
        )
        for a, tgt, vff, vmax, amax, (lo, hi) in ((self.ax[0], m.pan_mdeg, m.pan_vff, m.pan_vmax, m.pan_amax, lim[0]),
                                                  (self.ax[1], m.tilt_mdeg, m.tilt_vff, m.tilt_vmax, m.tilt_amax, lim[1])):  # fmt: skip
            a.target = max(lo, min(hi, tgt)) / 1000.0
            a.vff, a.vmax, a.amax, a.t_aim = (
                vff / 10.0,
                vmax / 10.0,
                float(amax),
                self.t,
            )

    def _home(self, f, m: Home):
        if self.estop_open:
            return self._ack(f, p.INTERLOCK)
        if self.state != IDLE:
            return self._ack(f, p.BAD_STATE)
        if not m.axis_mask & 3:
            return self._ack(f, p.BAD_ARG)
        for i, a in enumerate(self.ax):
            if m.axis_mask >> i & 1:
                a.homed, a.homing, a.homing_t0 = (
                    False,
                    "clear" if a.hall else "seek_pos",
                    self.t,
                )
                a.marked, a.mark = False, a.pos
        self._ack(f, p.OK, m.axis_mask & 3)

    def _flywheel_cmd(self, f, m: Flywheel):
        if m.duty_permille > 1000:
            return self._ack(f, p.BAD_ARG)
        if m.duty_permille == 0:
            self.fw_req = 0.0
            if self.state in (ENGAGING,):
                self._to(ARMED, p.R_NONE, quiet=False)
            return
        if not self.rail_on:
            return self._ack(f, p.INTERLOCK)
        if not self.armed_state:
            return self._ack(f, p.BAD_STATE)
        if m.duty_permille > self.fw_req:
            self.fw_req_t = self.t
        self.fw_req, self.fw_refresh_t = float(m.duty_permille), self.t
        if self.state == ARMED:
            self._to(ENGAGING, p.R_NONE)

    def _fire(self, f, m: Fire):
        for fid, res, info in self.fire_ids:
            if fid == m.fire_id:  # retransmission: the old answer, nothing shot
                return self._send(Ack(f.id, f.seq, res, info))
        res = self._fire_check(m)
        self.fire_ids.append((m.fire_id, res, 0))
        if res:
            return self._ack(f, res)
        self.last_fire_id = m.fire_id
        self.cur_fire_id, self.balls_left, self.ball_idx = m.fire_id, m.n, 0
        self.phase = "wait"
        self._to(FIRING, p.R_NONE, quiet=False)
        self._send(Ack(f.id, f.seq, p.OK, 0))

    def _fire_check(self, m):
        if self.state == FIRING:
            return p.BUSY
        if self.state != ENGAGING:
            return p.BAD_STATE
        if not 1 <= m.n <= 3:
            return p.BAD_ARG
        if not self.rail_on:
            return p.INTERLOCK
        if not self.hb_ok or not self.deadman_ok or self.inhibit:
            return p.INHIBIT
        if not all(a.homed for a in self.ax):
            return p.NOT_HOMED
        if not self._in_sector():
            return p.RANGE
        if not self._spun():
            return p.NOT_SPUN
        if self.shots_arm + m.n > self.shot_cap:
            return p.CAP
        if (self.t - self.last_ball_t) * 1000 < self.cfg[p.CFG_MIN_SHOT_INTERVAL_MS]:
            return p.CAP
        return p.OK

    def _spun(self):
        return (
            self.fw_req > 0
            and self.fw >= self.fw_req
            and (self.t - self.fw_req_t) * 1000 >= self.cfg[p.CFG_SPINUP_MS]
        )

    def _arm(self, f, m: Arm):
        if m.mode == 0:
            if self.armed_state:
                self._to(IDLE, p.R_DISARM)
            return self._ack(f, p.OK)
        if m.mode != 1 or m.shot_cap == 0 or m.timeout_s == 0:
            return self._ack(f, p.BAD_ARG)
        if self.state != IDLE:
            return self._ack(f, p.BAD_STATE)
        if self.estop_open or not self.key_on:
            return self._ack(f, p.INTERLOCK)
        if self.fault_bits:
            return self._ack(f, p.BAD_STATE)
        if not self.hb_ok or not self.deadman_ok:
            return self._ack(f, p.INHIBIT)
        if not all(a.homed for a in self.ax):
            return self._ack(f, p.NOT_HOMED)
        self.shot_cap = min(m.shot_cap, self.cfg[p.CFG_HARD_SHOT_CAP])
        self.arm_deadline = self.t + min(m.timeout_s, self.cfg[p.CFG_HARD_ARM_TIMEOUT])
        self.shots_arm = 0
        self.deadman_low_since = None
        self._to(ARMED, p.R_OPERATOR_ARM)  # (hb_run is only reset on entering SAFE)
        self._ack(f, p.OK, self.shot_cap)

    def _cfg_set(self, f, m: CfgSet):
        if self.state not in (SAFE, IDLE):
            return self._ack(f, p.BAD_STATE)
        if m.key not in self.cfg:
            return self._ack(f, p.BAD_ARG)
        if (
            m.key in (p.CFG_HARD_SHOT_CAP, p.CFG_HARD_ARM_TIMEOUT)
            and self.state != SAFE
        ):
            return self._ack(
                f, p.BAD_STATE
            )  # the hard limits change only by a deliberate maintenance act in SAFE
        v = m.value
        if m.key == p.CFG_WD_MS:
            v = max(1, min(300, v))
        if m.key in (p.CFG_PAN_SPR, p.CFG_TILT_SPR) and v <= 0:
            return self._ack(f, p.BAD_ARG)
        if m.key in (p.CFG_HARD_SHOT_CAP,) and not 0 <= v <= 255:
            return self._ack(f, p.BAD_ARG)
        self.cfg[m.key] = v
        if m.key in (p.CFG_PAN_SPR, p.CFG_TILT_SPR):
            self.ax[m.key - p.CFG_PAN_SPR].spr = v
        if m.key == p.CFG_DIR_INVERT:
            for i, a in enumerate(self.ax):
                a.sign = -1.0 if v >> i & 1 else 1.0
        self._ack(f, p.OK)

    def _cfg_get(self, f, m: CfgGet):
        if m.key not in self.cfg:
            return self._ack(f, p.BAD_ARG)
        self._send(CfgVal(m.key, self.cfg[m.key]))

    def _cfg_save(self, f, m):
        if self.state not in (SAFE, IDLE):
            return self._ack(f, p.BAD_STATE)
        self.flash = dict(self.cfg)
        self._ack(f, p.OK)

    def _clear_fault(self, f, m):
        if self.state not in (SAFE, FAULT):
            return self._ack(f, p.BAD_STATE)
        if self.estop_open:
            return self._ack(f, p.INTERLOCK)
        self.fault_bits = self.faults_reported = self.diag_ms = 0
        if self.state == FAULT:
            self._to(SAFE, p.R_DISARM)
        self._ack(f, p.OK)

    def _bootsel(self, f, m):
        if self.state == SAFE and m.magic == 0xB007B007:
            self.bootsel = True  # firmware: reset into the ROM USB bootloader
        else:
            self._ack(f, p.BAD_STATE if self.state != SAFE else p.BAD_ARG)

    # ---- time --------------------------------------------------------------------------------------------------------------------------
    def step(self, now):
        """Advance to `now` in 1 ms ticks (the first call only sets the clock). The plant's joint goals are written once per call."""
        if self.t is None:
            self.t = self.t0 = now
            self.next_status = now
            return
        while self.t + SUB <= now + 1e-9:
            self.t += SUB
            self._tick()
        self._write_plant()

    def _tick(self):
        data = self.link.mcu_read(self.t)
        if data:
            self._rx(data)
        pl = self.plant
        if hasattr(pl, "advance"):
            pl.advance(SUB)
        self._inputs()
        self._watchdog()
        self._timers()
        self._flywheel()
        self._pusher()
        self._axes()
        if self.hello_done and self.t >= self.next_status:
            self.next_status += STATUS_PERIOD
            if (
                self.t >= self.next_status
            ):  # late (the link was down): no catch-up burst
                self.next_status = self.t + STATUS_PERIOD
            self._send(self._status())

    def _inputs(self):
        e, k = bool(self.plant.estop_open), bool(self.plant.key_on)
        if e != self.estop_open:
            self.estop_open = e
            self._event(p.EV_ESTOP_EDGE, int(e))
            if e:
                self.fw_req = self.fw = 0.0
                self._to(
                    SAFE, p.R_ESTOP
                ) if self.state != SAFE else self._estop_while_safe()
        if k != self.key_on:
            self.key_on = k
            self._event(p.EV_KEY_EDGE, int(k))
            if not k and self.state != SAFE:
                self._to(SAFE, p.R_KEY_OFF)
        if not self.rail_on:  # hardware: the rail is open, nothing downstream of the key + E-stop has power
            self.fw = self.fw_req = 0.0
            self.pusher_ext, self.balls_left, self.phase = False, 0, None

    def _estop_while_safe(self):
        self._steppers(False)
        for a in self.ax:
            a.homed, a.homing = False, None

    def _watchdog(self):
        if (
            self.hello_done
            and self.last_hb_t is not None
            and (self.t - self.last_hb_t) * 1000 > self.cfg[p.CFG_WD_MS]
        ):
            self.hb_run = 0
            if self.state in (IDLE, ARMED, ENGAGING, FIRING):
                self.wd_tripped = True
                self._event(p.EV_WATCHDOG, int((self.t - self.last_hb_t) * 1000))
                self._to(SAFE, p.R_WATCHDOG)
        if self.diag_pin and self.steppers_enabled:
            self.diag_ms = min(self.diag_ms + 1, 255)
            if self.diag_ms > DIAG_FAULT_MS:
                self.fault_bits |= p.FAULT_DRIVER_DIAG
        else:
            self.diag_ms = 0
        fresh = self.fault_bits & ~self.faults_reported
        for b in range(6):
            if fresh >> b & 1:
                self._event(p.EV_FAULT, b)
        self.faults_reported |= fresh
        if self.fault_bits and self.state != FAULT:
            self._to(FAULT, p.R_FAULT)
        if (self.state == SAFE and self.hello_done and not self.version_bad and not self.fault_bits and not self.estop_open and self.hb_run >= 20
                and self.t - self.hb_run_t0 >= 1.0 - 1e-9):  # fmt: skip
            self.wd_tripped = False
            self._to(IDLE, p.R_LINK_UP)

    def _timers(self):
        if self.armed_state and self.t >= self.arm_deadline:
            self._to(IDLE, p.R_ARM_TIMEOUT)
        if (
            self.state in (ENGAGING, FIRING)
            and self.fw_req > 0
            and self.t - self.fw_refresh_t > FW_REFRESH_S
        ):
            self.fw_req = 0.0
            if self.state == ENGAGING:
                self._to(ARMED, p.R_NONE)
        if self.armed_state:
            if self.deadman_ok:
                self.deadman_low_since = None
            else:
                self.deadman_low_since = (
                    self.t if self.deadman_low_since is None else self.deadman_low_since
                )
                if self.t - self.deadman_low_since > DEADMAN_LOW_S and self.fw_req > 0:
                    self.fw_req = 0.0
                    if self.state == ENGAGING:
                        self._to(ARMED, p.R_NONE)

    def _flywheel(self):
        tgt = self.fw_req if self.rail_on and self.armed_state else 0.0
        if self.fw != tgt:
            if self.fw > tgt:  # spin-down is immediate: the gate is switched off
                self.fw = tgt
            else:
                self.fw = min(tgt, self.fw + FW_RAMP_PER_MS)
            self.plant.set_flywheel(self.fw)

    def _in_sector(self):
        """The measured barrel position inside the fire sector (cfg 20, 21 pan; 17, 22 tilt; mdeg): checked at FIRE and every tick until the feed point."""
        pan, tilt = self.ax[0].pos * 1000, self.ax[1].pos * 1000
        c = self.cfg
        return (
            c[p.CFG_FIRE_PAN_MIN] <= pan <= c[p.CFG_FIRE_PAN_MAX]
            and c[p.CFG_FIRE_TILT_MIN] <= tilt <= c[p.CFG_FIRE_TILT_MAX]
        )

    def _pusher(self):
        t = self.t
        if self.phase is None:
            return
        if self.phase in ("wait", "ext") and not self._in_sector():
            # the barrel left the sector before the ball was fed: retract, report, ARMED (no flywheel request left)
            self.pusher_ext, self.balls_left, self.phase = False, 0, None
            self.fw_req = 0.0
            self._event(
                p.EV_SECTOR_ABORT, (self.cur_fire_id << 8) | (self.ball_idx & 0xFF)
            )
            if self.state == FIRING:
                self._to(ARMED, p.R_NONE)
            return
        dwell = self.cfg[p.CFG_PUSHER_DWELL_MS] / 1000.0
        if (
            self.phase == "wait"
            and t - self.last_ball_t >= self.cfg[p.CFG_MIN_SHOT_INTERVAL_MS] / 1000.0
        ):
            self.phase, self.phase_end, self.pusher_ext = "ext", t + dwell, True
        elif self.phase == "ext" and t >= self.phase_end:
            # re-check the gates at the instant the ball is fed: a burst aborts when the app inhibits mid-burst
            if not (
                self.rail_on
                and self.deadman_ok
                and not self.inhibit
                and self.state == FIRING
                and self.fw_req > 0
                and self.shots_arm < self.shot_cap
            ):
                self.pusher_ext, self.balls_left, self.phase = False, 0, None
                return self._burst_done()
            self.shots_arm += 1
            self.shots_total += 1
            self.last_ball_t = t
            self.shots.append((t, self.cur_fire_id, self.ball_idx))
            self.plant.fire_pulse()
            self.phase, self.phase_end, self.pusher_ext = "ret", t + dwell, False
        elif self.phase == "ret" and t >= self.phase_end:
            self._event(p.EV_SHOT, (self.cur_fire_id << 8) | (self.ball_idx & 0xFF))
            self.ball_idx += 1
            self.balls_left -= 1
            if self.balls_left > 0 and self.shots_arm < self.shot_cap:
                self.phase = "wait"
            else:
                self.balls_left, self.phase = 0, None
                self._burst_done()

    def _burst_done(self):
        if self.state != FIRING:
            return
        if self.shots_arm >= self.shot_cap:
            self._to(IDLE, p.R_SHOT_CAP)
        else:  # ENGAGING needs a flywheel request (spec 3.1): a burst ended by a lost refresh / dead-man / inhibit goes to ARMED
            self._to(ENGAGING if self.fw_req > 0 else ARMED, p.R_NONE)

    # ---- axes --------------------------------------------------------------------------------------------------------------------------
    def _axes(self):
        q = self.plant.joint_deg()
        for i, a in enumerate(self.ax):
            a.prev_hall, a.hall = (
                a.hall,
                abs(q[i] - self.q_hall[i]) < HALL_HALF_WIDTH_DEG,
            )
            if not self.steppers_enabled:
                a.v = 0.0
                continue
            spd = self.spd(i)
            vcap = STEP_RATE_MAX / spd
            if a.homing:
                self._homing(i, a, q[i], vcap)
                continue
            lo, hi = (
                self.cfg[k] / 1000.0
                for k in (
                    (p.CFG_PAN_MIN, p.CFG_PAN_MAX),
                    (p.CFG_TILT_MIN, p.CFG_TILT_MAX),
                )[i]
            )
            age = None if a.t_aim is None else self.t - a.t_aim
            if age is None or age > AIM_STALE_HOLD_S:
                tgt, vff = a.pos, 0.0
                vmax, amax = max(a.vmax, 1.0), max(a.amax, 100.0)
            else:
                tgt, vff, vmax, amax = (
                    a.target,
                    (0.0 if age > AIM_STALE_VFF_S else a.vff),
                    a.vmax,
                    a.amax,
                )
            if self.state == SAFE:  # watchdog hold: targets frozen where they were
                tgt, vff = a.pos, 0.0
            vmax = min(vmax, vcap)
            err = (
                tgt - a.pos
            )  # P term braked to the stopping distance v^2/(2*amax) (a trapezoid: no overshoot on a fast move)
            brake = math.sqrt(2 * amax * abs(err))
            vp = max(-brake, min(brake, KP * err))
            v_cmd = max(-vmax, min(vmax, vff + vp))
            room_hi, room_lo = (
                max(0.0, hi - a.pos),
                max(0.0, a.pos - lo),
            )  # stopping distance fits before the soft limit
            v_cmd = (
                min(v_cmd, math.sqrt(2 * amax * room_hi))
                if v_cmd > 0
                else max(v_cmd, -math.sqrt(2 * amax * room_lo))
            )
            dv = amax * SUB
            a.v += max(-dv, min(dv, v_cmd - a.v))
            a.pos += a.v * SUB
            far = (
                a.homed
                and a.hall
                and abs(
                    a.pos
                    - self.cfg[(p.CFG_PAN_HOME_OFF, p.CFG_TILT_HOME_OFF)[i]] / spd
                    - HALL_HALF_WIDTH_DEG
                )
                > 5.0
            )
            a.stuck_t = (
                a.stuck_t + SUB if far else 0.0
            )  # the window centre; a joint sweeping through the window behind a lagging command is not stuck
            if a.stuck_t > 0.5:
                self.fault_bits |= p.FAULT_HALL_STUCK

    def _homing(self, i, a, q, vcap):
        spd = self.spd(i)
        v = min(self.cfg[p.CFG_HOMING_SPEED] / 10.0, vcap)
        vs = min(HOMING_SEARCH_DEG_S, vcap)
        lo, hi = (
            self.cfg[k] / 1000.0
            for k in ((p.CFG_PAN_MIN, p.CFG_PAN_MAX), (p.CFG_TILT_MIN, p.CFG_TILT_MAX))[
                i
            ]
        )
        timeout = self.t - a.homing_t0 > HOMING_TIMEOUT_S
        if (
            a.homing == "seek_pos"
        ):  # negative half-range + 10 deg: a start on the negative side must meet the sensor within it
            a.pos += vs * SUB
            a.v = vs
            if a.hall:
                a.homing, a.marked = "clear", False
            elif a.pos - a.mark > -lo + 10.0:
                a.homing, a.mark = "seek_neg", a.pos
        elif (
            a.homing == "seek_neg"
        ):  # the start was on the positive side: sweep down through the whole range
            a.pos -= vs * SUB
            a.v = -vs
            if a.hall:
                a.homing, a.marked = "clear", False
            elif a.mark - a.pos > (hi - lo) + 20.0:
                timeout = True  # nothing found: sensor or magnet missing
        elif a.homing == "clear":  # on the sensor: back out on the negative side first
            a.pos -= v * SUB
            a.v = -v
            if not a.hall and not a.marked:
                a.marked, a.mark = True, a.pos
            if (
                a.marked and a.pos <= a.mark - 2.0
            ):  # 2 deg clear of the window's negative edge
                a.homing = "approach"
        elif a.homing == "approach":
            a.pos += v * SUB
            a.v = v
            if a.hall and not a.prev_hall:
                a.pos = self.cfg[(p.CFG_PAN_HOME_OFF, p.CFG_TILT_HOME_OFF)[i]] / spd
                a.offset = q - a.sign * a.pos
                a.target = a.pos - BACKOFF_DEG
                a.homing = "backoff"
        elif a.homing == "backoff":
            a.pos -= v * SUB
            a.v = -v
            if a.pos <= a.target:
                a.pos, a.v, a.homing, a.homed = a.target, 0.0, None, True
                a.t_aim = None
                self._event(p.EV_HOMED, 1 << i)
        if timeout and a.homing:
            a.homing, a.v = None, 0.0
            self.fault_bits |= p.FAULT_HOMING_TIMEOUT

    def _write_plant(self):
        """Once per call: joint goals = sign * pos + offset, and the step-loss check against where the joint actually is."""
        if self.t is None:
            return
        q = self.plant.joint_deg()
        self.diag_pin = False
        for i, a in enumerate(self.ax):
            if not self.steppers_enabled:
                a.cmd_joint = None
                continue
            if self.step_loss and a.cmd_joint is not None and a.homed:
                full = (
                    4 * 16 * 360.0 / a.spr
                )  # 4 full motor steps in output degrees (pan 1.44, tilt 1.8)
                if (
                    abs(q[i] - a.cmd_joint) > full
                ):  # compared with the PREVIOUS call's goal (5 ms to follow); the TMC2209 raises DIAG
                    self.diag_pin = True
            a.cmd_joint = a.sign * a.pos + a.offset
        if self.steppers_enabled:
            self.plant.set_joint_goal(self.ax[0].cmd_joint, self.ax[1].cmd_joint)

    # ---- status ------------------------------------------------------------------------------------------------------------------------
    def _status(self) -> Status:
        a0, a1 = self.ax
        f1 = (self.estop_open * p.F1_ESTOP | self.key_on * p.F1_KEY | a0.homed * p.F1_PAN_HOMED | a1.homed * p.F1_TILT_HOMED | a0.hall * p.F1_PAN_HALL
              | a1.hall * p.F1_TILT_HALL | self.wd_tripped * p.F1_WD_TRIPPED | self.hb_ok * p.F1_HB_OK)  # fmt: skip
        lim = lambda a, k: (
            a.homed
            and abs(a.pos - self.cfg[k[0]] / 1000.0) < 0.01
            or abs(a.pos - self.cfg[k[1]] / 1000.0) < 0.01
        )
        f2 = ((abs(a0.v) > 0.05) * p.F2_PAN_MOVING | (abs(a1.v) > 0.05) * p.F2_TILT_MOVING | self.steppers_enabled * p.F2_STEPPERS_EN
              | self.pusher_ext * p.F2_PUSHER_EXT | self.inhibit * p.F2_INHIBIT | bool(self.hb_flags & p.HB_DEADMAN_OK) * p.F2_DEADMAN
              | lim(a0, (p.CFG_PAN_MIN, p.CFG_PAN_MAX)) * p.F2_PAN_LIMIT | lim(a1, (p.CFG_TILT_MIN, p.CFG_TILT_MAX)) * p.F2_TILT_LIMIT)  # fmt: skip
        left = max(0, math.ceil(self.arm_deadline - self.t)) if self.armed_state else 0
        pos = [a.pos for a in self.ax]
        if self.status_from_joint:  # twin: report where the (non-stepper) placeholder joint really is, as a perfect encoder would
            q = self.plant.joint_deg()
            pos = [
                a.sign * (q[i] - a.offset) if a.homed else a.pos
                for i, a in enumerate(self.ax)
            ]
        return Status(self.ms, self.state, f1, f2, round(pos[0] * self.spd(0)), round(pos[1] * self.spd(1)), int(round(self.fw)), 0xFFFF,
                      self.shots_total & 0xFFFF, self.shots_arm, self.shot_cap if self.armed_state else 0, self.fault_bits, min(left, 0xFFFF),
                      self.last_fire_id & 0xFFFF)  # fmt: skip


# ---- links (the MCU's end) -------------------------------------------------------------------------------------------------------------
class LoopbackLink:
    """In-process link for lockstep runs: the host end (write / read, the napl.lite.io.Link Protocol) and the MCU end (mcu_read / mcu_write) with
    1 ms USB latency each way on the shared clock. Bytes go through COBS / CRC like on a wire. `drop = True` discards both directions
    (the comms_drop fault, as FeetechPort.drop)."""

    def __init__(self, clock, latency=0.001):
        self.clock, self.latency, self.drop = clock, latency, False
        self.up, self.down = deque(), deque()  # (t_due, bytes)

    def write(self, data):
        if not self.drop:
            self.up.append((self.clock.now() + self.latency, bytes(data)))

    def read(self, timeout=0.0):
        now, out = self.clock.now(), b""
        while self.down and self.down[0][0] <= now + 1e-9:
            out += self.down.popleft()[1]
        return out

    def mcu_read(self, t):
        out = b""
        while self.up and self.up[0][0] <= t + 1e-9:
            out += self.up.popleft()[1]
        return out

    def mcu_write(self, data, t):
        if not self.drop:
            self.down.append((t + self.latency, bytes(data)))


class PtyMcu:
    """The MCU's end of a pty for HIL: the harness holds the master fd, the app opens `slave_path` (raw 8N1 like the Pico's CDC port). `drop = True`
    discards both directions."""

    def __init__(self):
        import tty

        self.master, slave = os.openpty()
        tty.setraw(slave)
        self.slave_path = os.ttyname(slave)
        self._slave = (
            slave  # keep it open: closing the last slave end makes master reads fail
        )
        os.set_blocking(self.master, False)
        self.drop = False

    def mcu_read(self, t):
        try:
            data = os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            return b""
        return b"" if self.drop else data

    def mcu_write(self, data, t):
        if self.drop:
            return
        try:
            os.write(self.master, data)
        except (BlockingIOError, OSError):
            pass

    def close(self):
        for fd in (self.master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass
