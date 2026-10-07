"""Virtual Feetech SCS/STS serial bus: a pty speaking the STS3215 protocol, backed by simulated servo devices.

    port = FeetechPort(sim, servos={1: "/devices/s1", 2: "/devices/s2"})   # sim: napl.Sim.load(..., host=True)
    port.device_path        # e.g. /dev/pts/7: open it with unmodified Feetech host code (scservo_sdk, LeRobot, ...)

Protocol (Feetech SCS/STS, 8N1 half-duplex-style, no echo): `FF FF ID LEN INSTR PARAMS.. CHK`, `CHK = ~sum(ID..PARAMS) & 0xFF`,
`LEN = len(PARAMS) + 2`; status `FF FF ID LEN ERR PARAMS.. CHK` (`LEN = len(PARAMS) + 2`). Little-endian words.
Instructions: PING 1, READ 2, WRITE 3, REG_WRITE 4, ACTION 5, RESET 6, SYNC_READ 0x82, SYNC_WRITE 0x83. Broadcast ID 0xFE
never replies. A bad checksum / bad length is dropped (no action, no reply), as a real servo does.
Sources of the register map: LeRobot `motors/feetech/tables.py` (STS_SMS_SERIES_CONTROL_TABLE, sts3215 model number 777,
baud table, sign-magnitude bits: Present_Load 10, Present_Position / Present_Velocity / Goal_Position / Goal_Velocity 15),
the Feetech SDK `scservo_def.SCS_TOHOST(a, b)` (bit b = sign, value = -(a & ~(1 << b))), and parts/arm/feetech-sts3215-c001.toml.

Register map. S = simulated (reads come from / writes go to the sim), P = stored in the port (RAM / EEPROM image: the
port, not the sim, keeps it; it does not influence the physics), R = read-only.
  0 fw major, 1 fw minor, 3-4 model number 777 .................... P R   (fw version 0.0 is a placeholder)
  5 ID ............................................................ P     (re-keys the port's servo table)
  6 baud index, 7 return delay (x2 us), 8 status level ............ P     (7: wire delay; 8: 0 = no reply to writes; baud
                                                                           reg does not retune the pty: `baud=` does)
  9-12 min / max angle limit ...................................... P     (clamp the goal when max > 0)
  13-15 max temp / max V / min V, 16-17 max torque, 18-20, 24-39 ... P     (max torque, P/D/I: stored only, see Gaps)
  21 P, 22 D, 23 I ................................................ P
  40 torque enable ................................................ S     -> torque_enable (any nonzero = 1)
  41 acceleration, 44-45 goal time ................................ P
  42-43 goal position (counts, bit 15 = sign) ..................... S     -> goal_position [rad]
  46-47 goal speed (steps/s, bit 15 = sign, magnitude used) ....... S     -> goal_speed [rad/s]  (0 = no profile)
  48-49 torque limit, 55 lock (1 = EEPROM addr < 40 read-only) .... P
  56-57 present position (0..cpr-1, wrapped) ...................... S R   counts mod cpr (the sim encoder is unwrapped)
  58-59 present speed (steps/s, bit 15 sign) ...................... S R   finite difference of counts over >= 20 ticks
  60-61 present load (0.1 % of stall torque, bit 10 sign) ......... S R   `torque` / stall_torque
  62 present voltage (0.1 V) ...................................... S R   `voltage` attr
  63 present temperature (degC) ................................... S R   `temperature` attr (motor thermal state)
  65 status, 66 moving ............................................. S R   moving = torque on and |goal - position| > 2 counts
  69-70 present current (6.5 mA) .................................. S R   |`current`| (motor current)
Every SetAttr goes through the command bus with source "feetech", so a session with a Host log replays exactly (the log
holds the writes and the Steps; the pty traffic itself and its wall-clock timing are not part of it).
Timing: with `realtime=True` (the Sim running at Host rate 1) each byte of a request and of a reply takes 10 / baud s of
wall clock and the reply additionally waits `return delay x 2 us`; the register is read after the request has arrived.
"""

import os
import select
import termios
import threading
import time
import tty
from collections import deque
from math import pi

MODEL_NUMBER = 777  # sts3215 (LeRobot MODEL_NUMBER_TABLE)
BAUDS = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 57_600,
    6: 38_400,
    7: 19_200,
}
PING, READ, WRITE, REG_WRITE, ACTION, RESET, SYNC_READ, SYNC_WRITE = (
    1,
    2,
    3,
    4,
    5,
    6,
    0x82,
    0x83,
)
BROADCAST = 0xFE
_RO = {0, 1, 3, 4, *range(56, 71)}  # fw version, model, present values
_MEM = 128


def checksum(body: bytes) -> int:
    return ~sum(body) & 0xFF


def packet(sid: int, instr_or_err: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, instr_or_err]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])


def _sm(v: int, bit: int) -> int:  # sign-magnitude -> int (SCS_TOHOST)
    return -(v & ~(1 << bit)) if v & (1 << bit) else v


def _to_sm(v: int, bit: int) -> int:  # int -> sign-magnitude (SCS_TOSCS)
    return (-v | (1 << bit)) if v < 0 else v


class _Servo:
    def __init__(self, sid, path, baud_index):
        self.id, self.path = sid, path
        self.mem = bytearray(_MEM)
        self.pending = None  # REG_WRITE buffer: (addr, data)
        self.goal_dirty = False
        self.hist = deque(maxlen=64)  # (tick, counts) for the finite-difference speed
        self.speed = 0
        self.reset(baud_index)

    def reset(self, baud_index=0):
        m = self.mem
        m[:] = bytes(_MEM)
        m[3:5] = MODEL_NUMBER.to_bytes(2, "little")
        m[5], m[6], m[8] = self.id, baud_index, 1
        m[11:13] = (4095).to_bytes(2, "little")
        m[13], m[14], m[15] = (
            70,
            80,
            40,
        )  # max temp degC (part), max V (assumed 8.0 V), min V 4.0 V (part)
        m[16:18] = m[48:50] = (1000).to_bytes(
            2, "little"
        )  # max torque / torque limit 0.1 % (part)
        m[21], m[22], m[23] = 32, 32, 0  # P, D (part: D estimated), I
        m[26] = m[27] = 1  # dead zones (part)
        m[28:30] = (511).to_bytes(2, "little")
        m[30], m[34], m[35], m[36], m[38] = (
            1,
            20,
            200,
            80,
            200,
        )  # part: protection / overload defaults

    def w(self, a):  # little-endian word
        return self.mem[a] | self.mem[a + 1] << 8

    def put(self, a, v, n=2):
        self.mem[a : a + n] = (v & ((1 << 8 * n) - 1)).to_bytes(n, "little")


class FeetechPort:
    def __init__(
        self,
        sim,
        servos: dict,
        baud: int = 1_000_000,
        cpr: int = 4096,
        symlink=None,
        realtime: bool = True,
        stall_torque: float = 1.9123,
        dt: float = 0.001,
    ):
        """sim: a Sim loaded with host=True (thread-safe). servos: {bus id: "/devices/<servo name>"}. cpr: encoder counts
        per revolution (the servo's `encoder_cpr`). stall_torque: N*m that reads as 100 % load (part: 1.9123 at 7.4 V).
        dt: the model's timestep, only for the finite-difference speed. symlink: stable path to create for the pty."""
        self.sim, self.baud, self.cpr, self.realtime = sim, baud, cpr, realtime
        self.stall_torque, self.dt = stall_torque, dt
        bi = {v: k for k, v in BAUDS.items()}.get(baud, 0)
        self.servos = {i: _Servo(i, p, bi) for i, p in servos.items()}
        self.errors = []  # rejected SetAttr replies and dropped frames, for tests / diagnostics
        self._rx = bytearray()
        self._stop = threading.Event()
        self._master, self._slave = os.openpty()
        tty.setraw(self._slave, termios.TCSANOW)
        os.set_blocking(self._master, False)
        self.device_path = os.ttyname(self._slave)
        self.symlink = None
        if symlink is not None:
            self.symlink = os.fspath(symlink)
            if os.path.lexists(self.symlink):
                os.unlink(self.symlink)
            os.symlink(self.device_path, self.symlink)
        self._thread = threading.Thread(
            target=self._serve, name="feetech-port", daemon=True
        )
        self._thread.start()

    # ---- lifecycle -------------------------------------------------------------------------------------------
    def close(self):
        self._stop.set()
        self._thread.join()
        for fd in (self._master, self._slave):
            os.close(fd)
        if self.symlink and os.path.islink(self.symlink):
            os.unlink(self.symlink)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # ---- I/O thread ------------------------------------------------------------------------------------------
    def _wire(self, nbytes, extra=0.0):
        if self.realtime:
            time.sleep(nbytes * 10 / self.baud + extra)

    def _serve(self):
        while not self._stop.is_set():
            r, _, _ = select.select([self._master], [], [], 0.05)
            if not r:
                self._rx.clear()  # a stalled partial frame is dropped
                continue
            try:
                data = os.read(self._master, 4096)
            except (BlockingIOError, InterruptedError):
                continue
            for pkt in self.feed(data):
                self._wire(
                    len(pkt)
                )  # the request's time on the wire; state is sampled after it
                for reply in self.handle(pkt):
                    delay = self._delay(pkt)
                    self._wire(len(reply), delay)
                    try:
                        os.write(self._master, reply)
                    except BlockingIOError:
                        self.errors.append("tx buffer full: reply dropped")

    def _delay(self, pkt):
        s = self.servos.get(pkt[2])
        return (s.mem[7] * 2e-6) if s else 0.0

    # ---- protocol --------------------------------------------------------------------------------------------
    def feed(self, data: bytes) -> list:
        """Append raw bytes, return the complete, checksum-valid request packets (bad frames are dropped)."""
        rx, out = self._rx, []
        rx += data
        while True:
            i = rx.find(b"\xff\xff")
            if i < 0:
                del rx[: max(len(rx) - 1, 0)]  # keep a trailing lone 0xFF
                return out
            del rx[:i]
            if len(rx) < 4:
                return out
            if rx[2] == 0xFF or rx[3] < 2:  # not an ID / length: resync one byte on
                del rx[:1]
                continue
            total = rx[3] + 4
            if len(rx) < total:
                return out
            pkt = bytes(rx[:total])
            if checksum(pkt[2:-1]) != pkt[-1]:
                self.errors.append("bad checksum")
                del rx[:2]
                continue
            del rx[:total]
            out.append(pkt)

    def handle(self, pkt: bytes) -> list:
        """One valid request packet -> list of status packets to send (applies the writes)."""
        sid, instr, params = pkt[2], pkt[4], pkt[5:-1]
        if sid == BROADCAST:
            if instr == SYNC_WRITE and len(params) >= 2:
                addr, n = params[0], params[1]
                body = params[2:]
                for k in range(
                    0, len(body) - n, n + 1
                ):  # ponytail: trailing partial entry ignored
                    s = self.servos.get(body[k])
                    if s:
                        self._write(s, addr, body[k + 1 : k + 1 + n])
            elif instr == SYNC_READ and len(params) >= 2:
                addr, n = params[0], params[1]
                return [
                    r
                    for i in params[2:]
                    if i in self.servos
                    for r in [self._read_reply(self.servos[i], addr, n)]
                    if r
                ]
            elif instr == WRITE and len(params) >= 2:
                for s in list(self.servos.values()):
                    self._write(s, params[0], params[1:])
            elif instr == ACTION:
                for s in list(self.servos.values()):
                    self._action(s)
            return []  # broadcast: no status packet
        s = self.servos.get(sid)
        if s is None:
            return []
        if instr == PING:
            return [packet(sid, 0)]
        if instr == READ and len(params) == 2:
            r = self._read_reply(s, params[0], params[1])
            return [r] if r else []
        reply = [packet(sid, 0)] if s.mem[8] else []
        if instr == WRITE and len(params) >= 2:
            self._write(s, params[0], params[1:])
        elif instr == REG_WRITE and len(params) >= 2:
            s.pending = (params[0], bytes(params[1:]))
        elif instr == ACTION:
            self._action(s)
        elif instr == RESET:
            s.reset(s.mem[6])
        else:
            return []
        return reply

    def _read_reply(self, s, addr, n):
        if n == 0 or addr + n > _MEM:
            self.errors.append(f"read out of range {addr}+{n}")
            return None
        if addr < 71 and addr + n > 56:
            self._refresh(s)
        return packet(s.id, 0, bytes(s.mem[addr : addr + n]))

    def _action(self, s):
        if s.pending:
            addr, data = s.pending
            s.pending = None
            self._write(s, addr, data)

    # ---- register file <-> simulated device ------------------------------------------------------------------
    def _set(self, s, attr, value):
        r = self.sim.call(
            {
                "source": "feetech",
                "cmd_type": "SetAttr",
                "cmd": {
                    "path": s.path,
                    "attr": attr,
                    "frame": "",
                    "values": [float(value)],
                },
            }
        )
        if not r["ok"]:
            self.errors.append(f"{s.path}.{attr}: {r['message']}")

    def _q(self, s, attr):
        return float(self.sim.query(s.path, attr)[0][0])

    def _write(self, s, addr, data):
        lock = s.mem[55]
        for k, b in enumerate(data):
            a = addr + k
            if a >= _MEM or a in _RO or (lock and a < 40 and a != 55):
                continue
            s.mem[a] = b
        n = len(data)
        hit = lambda a, w=1: addr < a + w and a < addr + n
        if hit(40):
            on = s.mem[40] != 0
            if on and not s.goal_dirty:
                self._set(
                    s, "goal_position", self._q(s, "position")
                )  # enabling holds the present position
            if not on:
                s.goal_dirty = False
            self._set(s, "torque_enable", 1.0 if on else 0.0)
        if hit(
            46, 2
        ):  # speed before position: the profile applies from the first firmware step
            self._set(s, "goal_speed", abs(_sm(s.w(46), 15)) * 2 * pi / self.cpr)
        if hit(42, 2):
            g = _sm(s.w(42), 15)
            lo, hi = s.w(9), s.w(11)
            if hi > 0:
                g = min(max(g, lo), hi)
            if (
                0 <= g < self.cpr
            ):  # single-turn target, reached without wrapping: stay in the current turn
                g += int(self._q(s, "counts") // self.cpr) * self.cpr
            s.goal_dirty = True
            self._set(s, "goal_position", g * 2 * pi / self.cpr)
        if hit(5):
            new = s.mem[5]
            if new != s.id and new < BROADCAST:
                self.servos[new] = self.servos.pop(s.id)
                s.id = new

    def _refresh(self, s):
        counts = round(self._q(s, "counts"))
        tick = self.sim.tick
        s.hist.append((tick, counts))
        old = next(
            (h for h in s.hist if tick - h[0] >= 20), None
        )  # oldest sample >= 20 ticks back
        if old:
            s.speed = round((counts - old[1]) / ((tick - old[0]) * self.dt))
        torque = self._q(s, "torque")
        load = min(round(abs(torque) / self.stall_torque * 1000), 1000)
        goal = self._q(s, "goal_position") * self.cpr / (2 * pi)
        s.put(56, counts % self.cpr)
        s.put(58, _to_sm(s.speed, 15))
        s.put(60, _to_sm(-load if torque < 0 else load, 10))
        s.put(62, round(self._q(s, "voltage") * 10), 1)
        s.put(63, round(self._q(s, "temperature")), 1)
        s.put(66, int(s.mem[40] != 0 and abs(goal - counts) > 2), 1)
        s.put(69, round(abs(self._q(s, "current")) / 0.0065))
