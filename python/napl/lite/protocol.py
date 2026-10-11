"""RP2040 USB-CDC serial protocol v1.0 (docs/cuas-lite/spec.md section 2): COBS framing, CRC-16/CCITT-FALSE, every message, version handshake.

Shared by the app (napl.lite.gimbal) and the simulated MCU (napl.lite.sim_mcu); the firmware's C implementation tests against
tests/lite/protocol_vectors.json, which tests/lite/test_protocol.py checks against this module.

    wire = encode(HEARTBEAT, seq, Heartbeat(hb_seq=1, flags=1))        # COBS(id | seq | payload | crc16 LE) 0x00
    dec = Decoder(); for f in dec.feed(rx_bytes): f.id, f.seq, f.msg    # f.msg is None for an id nobody knows (f.payload kept)

A frame with bad CRC, bad COBS or fewer than 3 body bytes is dropped silently (Decoder.bad_frames counts them). Unpacking accepts a longer
payload than the table (minor versions append fields at the tail) and raises ProtocolError on a shorter one (the receiver answers ACK BAD_ARG).
"""

import struct
from dataclasses import astuple, dataclass, fields

MAJOR, MINOR = 1, 0
MAX_PAYLOAD = 60
CRC_CHECK = 0x29B1  # crc16(b"123456789")

# host -> MCU ids
HELLO, HEARTBEAT, AIM, HOME, FLYWHEEL, FIRE, ARM, CFG_SET, CFG_GET, CFG_SAVE, CLEAR_FAULT, BOOTSEL = (
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0F)  # fmt: skip
# MCU -> host ids
HELLO_ACK, STATUS, ACK, CFG_VAL, EVENT, DEBUG = 0x81, 0x82, 0x83, 0x84, 0x85, 0x86

# STATUS.state
SAFE, IDLE, ARMED, ENGAGING, FIRING, FAULT = range(6)
STATE_NAMES = ("SAFE", "IDLE", "ARMED", "ENGAGING", "FIRING", "FAULT")
# STATUS.flags1 bits
(
    F1_ESTOP,
    F1_KEY,
    F1_PAN_HOMED,
    F1_TILT_HOMED,
    F1_PAN_HALL,
    F1_TILT_HALL,
    F1_WD_TRIPPED,
    F1_HB_OK,
) = (1 << i for i in range(8))
# STATUS.flags2 bits
(
    F2_PAN_MOVING,
    F2_TILT_MOVING,
    F2_STEPPERS_EN,
    F2_PUSHER_EXT,
    F2_INHIBIT,
    F2_DEADMAN,
    F2_PAN_LIMIT,
    F2_TILT_LIMIT,
) = (1 << i for i in range(8))
# HEARTBEAT.flags
HB_DEADMAN_OK, HB_FIRE_INHIBIT = 1, 2
# fault_bits
(
    FAULT_HOMING_TIMEOUT,
    FAULT_DRIVER_DIAG,
    FAULT_CFG_INVALID,
    FAULT_SHOT_COUNTER,
    FAULT_HALL_STUCK,
    FAULT_USB_OVERFLOW,
) = (1 << i for i in range(6))
# EVENT codes
(
    EV_STATE_CHANGE,
    EV_SHOT,
    EV_HOMED,
    EV_FAULT,
    EV_WATCHDOG,
    EV_ESTOP_EDGE,
    EV_KEY_EDGE,
    EV_BAD_FRAMES,
) = range(1, 9)
# state-change reasons (EVENT 1 arg = new_state | reason << 8)
(
    R_BOOT,
    R_WATCHDOG,
    R_ESTOP,
    R_KEY_OFF,
    R_DISARM,
    R_ARM_TIMEOUT,
    R_SHOT_CAP,
    R_FAULT,
    R_OPERATOR_ARM,
    R_LINK_UP,
    R_NEW_SESSION,
) = range(1, 12)
# ACK.result
(
    OK,
    BAD_STATE,
    INTERLOCK,
    INHIBIT,
    CAP,
    RANGE,
    BUSY,
    UNKNOWN_CMD,
    BAD_ARG,
    NOT_HOMED,
    NOT_SPUN,
) = range(11)
RESULT_NAMES = (
    "OK",
    "BAD_STATE",
    "INTERLOCK",
    "INHIBIT",
    "CAP",
    "RANGE",
    "BUSY",
    "UNKNOWN_CMD",
    "BAD_ARG",
    "NOT_HOMED",
    "NOT_SPUN",
)
# config keys
(CFG_PAN_SPR, CFG_TILT_SPR, CFG_PAN_MIN, CFG_PAN_MAX, CFG_TILT_MIN, CFG_TILT_MAX, CFG_PAN_HOME_OFF, CFG_TILT_HOME_OFF, CFG_DIR_INVERT,
 CFG_HARD_SHOT_CAP, CFG_HARD_ARM_TIMEOUT, CFG_PUSHER_EXT_US, CFG_PUSHER_RET_US, CFG_PUSHER_DWELL_MS, CFG_MIN_SHOT_INTERVAL_MS,
 CFG_SPINUP_MS, CFG_FIRE_TILT_MIN, CFG_WD_MS, CFG_HOMING_SPEED) = range(1, 20)  # fmt: skip
CFG_DEFAULTS = {CFG_PAN_SPR: 16000, CFG_TILT_SPR: 12800, CFG_PAN_MIN: -170000, CFG_PAN_MAX: 170000, CFG_TILT_MIN: -15000, CFG_TILT_MAX: 70000,
                CFG_PAN_HOME_OFF: 0, CFG_TILT_HOME_OFF: 0, CFG_DIR_INVERT: 0, CFG_HARD_SHOT_CAP: 10, CFG_HARD_ARM_TIMEOUT: 120,
                CFG_PUSHER_EXT_US: 1900, CFG_PUSHER_RET_US: 1100, CFG_PUSHER_DWELL_MS: 120, CFG_MIN_SHOT_INTERVAL_MS: 300, CFG_SPINUP_MS: 700,
                CFG_FIRE_TILT_MIN: 0, CFG_WD_MS: 300, CFG_HOMING_SPEED: 200}  # fmt: skip


class ProtocolError(ValueError):
    pass


# ---- CRC-16/CCITT-FALSE and COBS ---------------------------------------------------------------------------------------------------
def _table():
    t = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        t.append(c)
    return t


_CRC = _table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """poly 0x1021, init 0xFFFF, no reflection, xorout 0: crc16(b"123456789") == 0x29B1."""
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC[(crc >> 8) ^ b]
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Standard COBS (no trailing delimiter): the output contains no 0x00."""
    out, block, code, full = bytearray(), bytearray(), 1, False
    for b in data:
        full = False
        if b == 0:
            out.append(code)
            out += block
            block, code = bytearray(), 1
        else:
            block.append(b)
            code += 1
            if code == 0xFF:
                out.append(code)
                out += block
                block, code, full = bytearray(), 1, True
    if not full:  # data ending exactly on a full 254-byte block gets no trailing empty block (the reference encoding)
        out.append(code)
        out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """-> decoded bytes; ProtocolError on a zero byte inside or a run that overshoots the input."""
    out, i, n = bytearray(), 0, len(data)
    if 0 in data:
        raise ProtocolError("bad COBS")
    while i < n:
        code = data[i]
        if i + code > n:
            raise ProtocolError("bad COBS")
        out += data[i + 1 : i + code]
        i += code
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---- messages --------------------------------------------------------------------------------------------------------------------------
class Msg:
    """A dataclass whose fields pack with struct format FMT (little-endian, no padding)."""

    ID = 0
    FMT = "<"

    def pack(self) -> bytes:
        return struct.pack(self.FMT, *astuple(self))

    @classmethod
    def unpack(cls, payload: bytes):
        n = struct.calcsize(cls.FMT)
        if len(payload) < n:
            raise ProtocolError(f"{cls.__name__}: payload {len(payload)} < {n}")
        return cls(
            *struct.unpack_from(cls.FMT, payload)
        )  # a longer payload: the tail is a later minor version's

    def asdict(self):
        return {
            f.name: (
                getattr(self, f.name).hex()
                if isinstance(getattr(self, f.name), bytes)
                else getattr(self, f.name)
            )
            for f in fields(self)
        }


def msg(id_, fmt):
    def wrap(cls):
        cls = dataclass(cls)
        cls.ID, cls.FMT = id_, "<" + fmt
        return cls

    return wrap


@msg(HELLO, "BBI")
class Hello(Msg):
    major: int = MAJOR
    minor: int = MINOR
    session: int = 0


@msg(HEARTBEAT, "HB")
class Heartbeat(Msg):
    hb_seq: int = 0
    flags: int = 0


@msg(AIM, "iihhHHHH")
class Aim(Msg):
    pan_mdeg: int = 0
    tilt_mdeg: int = 0
    pan_vff: int = 0  # 0.1 deg/s
    tilt_vff: int = 0
    pan_vmax: int = 2400  # 0.1 deg/s
    tilt_vmax: int = 1500
    pan_amax: int = 2000  # deg/s^2
    tilt_amax: int = 1500


@msg(HOME, "B")
class Home(Msg):
    axis_mask: int = 3


@msg(FLYWHEEL, "H")
class Flywheel(Msg):
    duty_permille: int = 0


@msg(FIRE, "HB")
class Fire(Msg):
    fire_id: int = 0
    n: int = 1


@msg(ARM, "BBH")
class Arm(Msg):
    mode: int = 1
    shot_cap: int = 5
    timeout_s: int = 60


@msg(CFG_SET, "Bi")
class CfgSet(Msg):
    key: int = 0
    value: int = 0


@msg(CFG_GET, "B")
class CfgGet(Msg):
    key: int = 0


@msg(CFG_SAVE, "")
class CfgSave(Msg):
    pass


@msg(CLEAR_FAULT, "")
class ClearFault(Msg):
    pass


@msg(BOOTSEL, "I")
class Bootsel(Msg):
    magic: int = 0xB007B007


@msg(HELLO_ACK, "BBBBBB8sI")
class HelloAck(Msg):
    major: int = MAJOR
    minor: int = MINOR
    result: int = 0  # 0 ok, 1 version mismatch
    fw_major: int = 0
    fw_minor: int = 0
    fw_patch: int = 0
    uid: bytes = b"\0" * 8
    cfg_crc: int = 0


@msg(STATUS, "IBBBiiHHHBBHHH")
class Status(Msg):
    t_ms: int = 0
    state: int = SAFE
    flags1: int = 0
    flags2: int = 0
    pan_steps: int = 0
    tilt_steps: int = 0
    fw_duty: int = 0
    fw_rpm: int = 0xFFFF
    shots_total: int = 0
    shots_arm: int = 0
    shot_cap: int = 0
    fault_bits: int = 0
    arm_left_s: int = 0
    last_fire_id: int = 0

    def flag1(self, bit):
        return bool(self.flags1 & bit)

    def flag2(self, bit):
        return bool(self.flags2 & bit)


@msg(ACK, "BBBB")
class Ack(Msg):
    cmd_id: int = 0
    cmd_seq: int = 0
    result: int = 0
    info: int = 0


@msg(CFG_VAL, "Bi")
class CfgVal(Msg):
    key: int = 0
    value: int = 0


@msg(EVENT, "Bi")
class Event(Msg):
    code: int = 0
    arg: int = 0


@dataclass
class Debug:
    text: str = ""
    ID = DEBUG

    def pack(self) -> bytes:
        return self.text.encode("ascii", "replace")[:48]

    @classmethod
    def unpack(cls, payload: bytes):
        return cls(payload.decode("ascii", "replace"))

    def asdict(self):
        return {"text": self.text}


MESSAGES = {c.ID: c for c in (Hello, Heartbeat, Aim, Home, Flywheel, Fire, Arm, CfgSet, CfgGet, CfgSave, ClearFault, Bootsel,
                              HelloAck, Status, Ack, CfgVal, Event, Debug)}  # fmt: skip
HOST_IDS = frozenset(i for i in MESSAGES if i < 0x80)
MCU_IDS = frozenset(i for i in MESSAGES if i >= 0x80)


# ---- frames ----------------------------------------------------------------------------------------------------------------------------
def encode_raw(id_: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("payload > 60 bytes")
    body = bytes([id_, seq & 0xFF]) + payload
    return cobs_encode(body + struct.pack("<H", crc16(body))) + b"\0"


def encode(seq: int, m) -> bytes:
    """One frame on the wire for a message object."""
    return encode_raw(m.ID, seq, m.pack())


@dataclass
class Frame:
    id: int
    seq: int
    payload: bytes
    msg: object = None  # decoded message, None for an unknown id or a payload shorter than its table row (bad_arg=True)
    bad_arg: bool = False


def decode_body(body: bytes) -> Frame:
    """body = id | seq | payload | crc16. ProtocolError on a short frame or a bad CRC."""
    if (
        len(body) < 4 or len(body) > 64
    ):  # id + seq + crc16 is 4 bytes; the spec's "length < 3" floor is subsumed (a 3-byte body cannot hold a CRC)
        raise ProtocolError("bad length")
    if crc16(body[:-2]) != struct.unpack("<H", body[-2:])[0]:
        raise ProtocolError("bad CRC")
    f = Frame(body[0], body[1], bytes(body[2:-2]))
    cls = MESSAGES.get(f.id)
    if cls:
        try:
            f.msg = cls.unpack(f.payload)
        except ProtocolError:
            f.bad_arg = True
    return f


def decode(wire: bytes) -> Frame:
    """One complete frame (with or without the trailing 0x00)."""
    return decode_body(cobs_decode(wire[:-1] if wire.endswith(b"\0") else wire))


class Decoder:
    """Streaming receiver: feed bytes, get frames. Bytes before the first 0x00 are ignored (resync); every later 0x00 ends a frame."""

    def __init__(self):
        self.buf = bytearray()
        self.synced = False
        self.bad_frames = 0
        self.good_frames = 0
        self.seq_gaps = 0
        self._seq = None

    def feed(self, data: bytes) -> list:
        out = []
        for b in data:
            if b != 0:
                if (
                    len(self.buf) <= 80
                ):  # longer than any frame: garbage, it fails to decode at the next delimiter
                    self.buf.append(b)
                continue
            chunk, self.buf = bytes(self.buf), bytearray()
            if not self.synced:
                self.synced = (
                    True  # first delimiter: what came before it is a partial frame
                )
                continue
            if not chunk:
                continue
            try:
                f = decode_body(cobs_decode(chunk))
            except ProtocolError:
                self.bad_frames += 1
                continue
            if self._seq is not None and f.seq != (self._seq + 1) & 0xFF:
                self.seq_gaps += (f.seq - self._seq - 1) & 0xFF
            self._seq = f.seq
            self.good_frames += 1
            out.append(f)
        return out


def hello_result(h: Hello) -> int:
    """MCU side of the version handshake: 0 ok (any minor), 1 on a major mismatch (the MCU then stays in SAFE rejecting all but HELLO)."""
    return 0 if h.major == MAJOR else 1


def handshake_ok(ack: HelloAck) -> bool:
    """Host side: the MCU accepted our major and speaks the same major."""
    return ack.result == 0 and ack.major == MAJOR


def sat(x, lo, hi):
    return max(lo, min(hi, int(round(x))))
