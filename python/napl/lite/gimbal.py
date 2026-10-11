"""SerialGimbal: the host side of the RP2040 protocol (spec section 2) over any Link. One implementation for real (TtyLink), HIL (TtyLink on a pty) and
lockstep sim (sim_mcu.LoopbackLink): the same bytes, the same code.

Not a thread: the control loop calls pump() to drain the link, and every send is a non-blocking link.write. Positions arrive in microsteps and are
converted with the per-axis steps_per_rev read back with CFG_GET after the HELLO (defaults 16000 / 12800)."""

import os
import random
import select
import termios
import time
import tty
from collections import deque

from . import protocol as p
from .protocol import sat

LINK_LATENCY_S = 0.001  # USB each way, subtracted from a STATUS arrival to date it


class TtyLink:
    """Raw 8N1 tty (the Pico's CDC port, or a pty slave): termios raw like python/napl/feetech.py, no pyserial. Baud is ignored by CDC."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except termios.error:
            pass  # a pty slave may refuse a baud; irrelevant there

    def write(self, data):
        try:
            os.write(self.fd, data)
        except (BlockingIOError, OSError):
            pass  # a full or vanished port is a link loss: the MCU watchdog handles it

    def read(self, timeout=0.0):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except (BlockingIOError, OSError):
            return b""

    def close(self):
        os.close(self.fd)


class SerialGimbal:
    def __init__(self, link, clock, session=None):
        self.link, self.clock = link, clock
        self.session = session if session is not None else random.getrandbits(32)
        self.dec = p.Decoder()
        self.seq = 0
        self.hb_seq = 0
        self.fire_id = 0
        self.status = p.Status()
        self.status_t = None  # app-clock time of the latest STATUS
        self.ring = deque(maxlen=200)  # (t, Status): 2 s at 100 Hz
        self.events = deque(maxlen=512)  # (t, Event)
        self.acks = deque(maxlen=128)  # (t, Ack)
        self.ack = None  # HelloAck
        self.cfg = {}
        self.spr = [p.CFG_DEFAULTS[p.CFG_PAN_SPR], p.CFG_DEFAULTS[p.CFG_TILT_SPR]]
        self.sent = 0
        self.link.write(
            b"\0"
        )  # resync byte: the MCU ignores everything up to the first 0x00

    # ---- host -> MCU -------------------------------------------------------------------------------------------------------------------
    def _send(self, m):
        self.link.write(p.encode(self.seq, m))
        self.seq = (self.seq + 1) & 0xFF
        self.sent += 1

    def hello(self):
        self.ack = None
        self._send(p.Hello(p.MAJOR, p.MINOR, self.session))

    def request_config(self):
        for k in (
            p.CFG_PAN_SPR,
            p.CFG_TILT_SPR,
            p.CFG_PAN_MIN,
            p.CFG_PAN_MAX,
            p.CFG_TILT_MIN,
            p.CFG_TILT_MAX,
            p.CFG_HARD_SHOT_CAP,
            p.CFG_HARD_ARM_TIMEOUT,
        ):
            self._send(p.CfgGet(k))

    def heartbeat(self, deadman, inhibit):
        self.hb_seq = (self.hb_seq + 1) & 0xFFFF
        self._send(
            p.Heartbeat(
                self.hb_seq,
                (p.HB_DEADMAN_OK if deadman else 0)
                | (p.HB_FIRE_INHIBIT if inhibit else 0),
            )
        )

    def aim(
        self,
        pan_deg,
        tilt_deg,
        pan_vff=0.0,
        tilt_vff=0.0,
        vmax=(240.0, 150.0),
        amax=(2000.0, 1500.0),
    ):
        self._send(p.Aim(sat(pan_deg * 1000, -(2**31), 2**31 - 1), sat(tilt_deg * 1000, -(2**31), 2**31 - 1), sat(pan_vff * 10, -32768, 32767),
                         sat(tilt_vff * 10, -32768, 32767), sat(vmax[0] * 10, 1, 65535), sat(vmax[1] * 10, 1, 65535), sat(amax[0], 1, 65535),
                         sat(amax[1], 1, 65535)))  # fmt: skip

    def home(self, pan=True, tilt=True):
        self._send(p.Home(int(pan) | int(tilt) << 1))

    def flywheel(self, duty):
        self._send(p.Flywheel(sat(duty * 1000, 0, 1000)))

    def fire(self, n=1):
        self.fire_id = (self.fire_id + 1) & 0xFFFF or 1
        self._send(p.Fire(self.fire_id, max(1, min(3, n))))
        return self.fire_id

    def arm(self, shot_cap, timeout_s):
        self._send(p.Arm(1, sat(shot_cap, 1, 255), sat(timeout_s, 1, 65535)))

    def disarm(self):
        self._send(p.Arm(0, 0, 0))

    def set_cfg(self, key, value):
        self._send(p.CfgSet(key, int(value)))

    def clear_fault(self):
        self._send(p.ClearFault())

    # ---- MCU -> host -------------------------------------------------------------------------------------------------------------------
    def pump(self):
        """Drain the link; update status / ring / events / acks. Returns the number of frames handled."""
        data = self.link.read(0.0)
        if not data:
            return 0
        now = self.clock.now()
        n = 0
        for f in self.dec.feed(data):
            n += 1
            m = f.msg
            if isinstance(m, p.Status):
                self.status, self.status_t = m, now - LINK_LATENCY_S
                self.ring.append((self.status_t, m))
            elif isinstance(m, p.Event):
                self.events.append((now, m))
            elif isinstance(m, p.Ack):
                self.acks.append((now, m))
            elif isinstance(m, p.HelloAck):
                self.ack = m
            elif isinstance(m, p.CfgVal):
                self.cfg[m.key] = m.value
                if m.key in (p.CFG_PAN_SPR, p.CFG_TILT_SPR) and m.value > 0:
                    self.spr[m.key - p.CFG_PAN_SPR] = m.value
        return n

    @property
    def link_ok(self):
        """A STATUS in the last 100 ms (the MCU sends one every 10 ms after the HELLO)."""
        return self.status_t is not None and self.clock.now() - self.status_t < 0.1

    @property
    def connected(self):
        return self.ack is not None and p.handshake_ok(self.ack) and self.link_ok

    def deg(self, st=None):
        """(pan, tilt) in degrees of a STATUS (default: the latest)."""
        st = st or self.status
        return st.pan_steps * 360.0 / self.spr[0], st.tilt_steps * 360.0 / self.spr[1]

    def status_at(self, t):
        """(pan_deg, tilt_deg) at app time t: linear interpolation in the STATUS ring, clamped to its ends; None before the first STATUS."""
        r = self.ring
        if not r:
            return None
        if t <= r[0][0]:
            return self.deg(r[0][1])
        if t >= r[-1][0]:
            return self.deg(r[-1][1])
        for (t0, s0), (t1, s1) in zip(
            reversed(list(r)[:-1]), reversed(list(r)[1:])
        ):  # walk back from the newest pair
            if t0 <= t <= t1:
                a = (t - t0) / max(t1 - t0, 1e-9)
                (p0, q0), (p1, q1) = self.deg(s0), self.deg(s1)
                return p0 + a * (p1 - p0), q0 + a * (q1 - q0)
        return self.deg(r[-1][1])

    def wall_wait(self, pred, timeout=1.0):
        """Real-time helper for HIL / hardware start-up only (never lockstep): pump until pred() or timeout."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.pump()
            if pred():
                return True
            time.sleep(0.002)
        return False
