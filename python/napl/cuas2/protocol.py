"""cuas2 serial protocol 1.1 (docs/cuas2/spec.md section 6): lite's framing, CRC, COBS and unchanged messages re-exported, plus the new constants, Status2
(45 bytes, v1 offsets 0-28 unchanged), Flywheel with the optional diff tail, Fire (n fixed 1), the extended config keys and a Decoder2 that decodes with the
cuas2 message table. Nothing here forks lite: `from napl.lite import protocol` is the single framing implementation (spec 2.3).

    from napl.cuas2 import protocol as p2
    wire = p2.encode(seq, p2.Flywheel(800, diff_pm=20))
    for f in p2.Decoder2().feed(rx): f.msg      # Status2 / Event / Ack ... (lite's Status is replaced by Status2)
"""

import struct
from dataclasses import dataclass

from napl.lite import protocol as _p
from napl.lite.protocol import (  # noqa: F401  (re-exported unchanged; every other lite name resolves through __getattr__ below)
    ACK,
    CFG_HARD_ARM_TIMEOUT,
    CFG_HARD_SHOT_CAP,
    CFG_MIN_SHOT_INTERVAL_MS,
    CFG_PUSHER_DWELL_MS,
    CFG_PUSHER_EXT_US,
    CFG_PUSHER_RET_US,
    CFG_SPINUP_MS,
    Ack,
    Aim,
    Arm,
    Bootsel,
    CfgGet,
    CfgSave,
    CfgSet,
    CfgVal,
    ClearFault,
    Debug,
    Decoder,
    Event,
    Frame,
    Heartbeat,
    Hello,
    HelloAck,
    Home,
    Msg,
    ProtocolError,
    cobs_decode,
    cobs_encode,
    crc16,
    encode,
    encode_raw,
    msg,
    sat,
)  # fmt: skip


def __getattr__(
    name,
):  # PEP 562: napl.cuas2.protocol.SAFE, .HB_DEADMAN_OK, .Decoder ... are lite's
    try:
        return getattr(_p, name)
    except AttributeError:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from None


MAJOR, MINOR = 1, 1  # protocol 1.1: tail-append only
FW_MAJOR = 2  # HELLO_ACK.fw_major of the cuas2 firmware
CRC_CHECK = _p.CRC_CHECK

# ---- new codes (spec 6.4) ----------------------------------------------------------------------------------------------------------
(
    FAULT_TELEMETRY_LOST,
    FAULT_OVERSPEED,
    FAULT_WHEEL_NOT_READY,
    FAULT_BREECH_DOUBLE,
    FAULT_NO_EXIT,
    FAULT_UNEXPECTED_EXIT,
    FAULT_PUSHER_STUCK,
    FAULT_GATE_STUCK,
    FAULT_SHOT_OVERCOUNT,
) = (1 << i for i in range(6, 15))  # b6-b14; b15 reserved
FDR_FAULT_MASK = 0x7FC0  # b6-b14: each stops KICK
FDR_JAM_MASK = 0x7E00  # b9-b14: shown as gate_state JAM
(
    EV_MUZZLE,
    EV_FEED_ABORT,
    EV_EMPTY,
    EV_JAM,
    EV_WHEELS_READY,
    EV_WHEELS_LOST,
    EV_HW_COUNTER,
) = range(10, 17)
FA_SECTOR, FA_DEADMAN, FA_INHIBIT, FA_WHEELS, FA_BEAM, FA_RAIL = range(1, 7)
FEED_ABORT_NAMES = ("", "sector", "deadman", "inhibit", "wheels", "beam", "rail")
NO_BALL, JAMMED, NOT_CLEAR, TOO_SOON = 12, 13, 14, 15
RESULT_NAMES = _p.RESULT_NAMES + ("NO_BALL", "JAMMED", "NOT_CLEAR", "TOO_SOON")
# gate_state (STATUS offset 36)
(
    GS_UNKNOWN,
    GS_HOMING,
    GS_LOADING,
    GS_TO_BORE,
    GS_STAGED,
    GS_PUSH,
    GS_EXIT_WAIT,
    GS_RETRACT,
    GS_TO_LOAD,
    GS_EMPTY,
    GS_JAM,
) = range(11)
GATE_NAMES = (
    "UNKNOWN",
    "HOMING",
    "LOADING",
    "TO_BORE",
    "STAGED",
    "PUSH",
    "EXIT_WAIT",
    "RETRACT",
    "TO_LOAD",
    "EMPTY",
    "JAM",
)
# ready_flags (offset 37), beams (offset 35)
RDY_READY, RDY_EMPTY, RDY_JAM, RDY_HOPPER_LOW = 1, 2, 4, 8
BM_BREECH, BM_MUZZLE, BM_GATE_HOME, BM_PUSH_HOME, BM_WHEELS_READY, BM_FEED_BUSY = (
    1,
    2,
    4,
    8,
    16,
    32,
)

# ---- config keys 23-41 (spec 6.4 table; 41 is the hardware shot counter's DIP setting) -------------------------------------------------
(
    CFG_WHEEL_D_UM, CFG_V_CAP_DMPS, CFG_RPM_MAX, CFG_RPM_IDLE, CFG_RPM_WINDOW_PM, CFG_SETTLE_MS, CFG_DIFF_PM, CFG_POLE_PAIRS, CFG_RAMP_RPM_S,
    CFG_THROTTLE_MAX_PM, CFG_KP, CFG_KI, CFG_GATE_LOAD_US, CFG_GATE_BORE_US, CFG_FILL_MS, CFG_EXIT_TIMEOUT_MS, CFG_TELEMETRY_LOSS_MS, CFG_K_BEAM,
    CFG_HW_COUNTER_N,
) = range(23, 42)  # fmt: skip
CFG_COUNT = 42
CFG_DEFAULTS2 = dict(_p.CFG_DEFAULTS)
CFG_DEFAULTS2.update(
    {CFG_PUSHER_DWELL_MS: 80, CFG_MIN_SHOT_INTERVAL_MS: 450, CFG_SPINUP_MS: 100}
)
CFG_DEFAULTS2.update({CFG_WHEEL_D_UM: 40000, CFG_V_CAP_DMPS: 380, CFG_RPM_MAX: 17500, CFG_RPM_IDLE: 4500, CFG_RPM_WINDOW_PM: 10, CFG_SETTLE_MS: 100,
                      CFG_DIFF_PM: 15, CFG_POLE_PAIRS: 7, CFG_RAMP_RPM_S: 40000, CFG_THROTTLE_MAX_PM: 900, CFG_KP: 600, CFG_KI: 40, CFG_GATE_LOAD_US: 1150,
                      CFG_GATE_BORE_US: 1700, CFG_FILL_MS: 30, CFG_EXIT_TIMEOUT_MS: 250, CFG_TELEMETRY_LOSS_MS: 50, CFG_K_BEAM: 1000, CFG_HW_COUNTER_N: 10})  # fmt: skip
MIN_SHOT_INTERVAL_FLOOR = 400  # firmware clamp of cfg 15
V_CAP_MAX_DMPS = 380  # 38 m/s: the absolute cap
# inclusive (lo, hi) ranges the firmware enforces at CFG_SET (a value outside answers BAD_ARG)
CFG_RANGES = {
    CFG_WHEEL_D_UM: (10000, 100000), CFG_V_CAP_DMPS: (1, 380), CFG_RPM_MAX: (0, 60000), CFG_RPM_IDLE: (0, 60000), CFG_RPM_WINDOW_PM: (1, 100),
    CFG_SETTLE_MS: (0, 2000), CFG_DIFF_PM: (-100, 100), CFG_POLE_PAIRS: (1, 32), CFG_RAMP_RPM_S: (1000, 400000), CFG_THROTTLE_MAX_PM: (100, 1000),
    CFG_KP: (0, 100000), CFG_KI: (0, 100000), CFG_GATE_LOAD_US: (500, 2500), CFG_GATE_BORE_US: (500, 2500), CFG_FILL_MS: (0, 1000),
    CFG_EXIT_TIMEOUT_MS: (100, 1000), CFG_TELEMETRY_LOSS_MS: (10, 1000), CFG_K_BEAM: (100, 3000), CFG_HW_COUNTER_N: (0, 255),
    CFG_PUSHER_EXT_US: (500, 2500), CFG_PUSHER_RET_US: (500, 2500), CFG_PUSHER_DWELL_MS: (50, 300),
}  # fmt: skip
MAINTENANCE_KEYS = (
    CFG_HARD_SHOT_CAP,
    CFG_HARD_ARM_TIMEOUT,
    CFG_WHEEL_D_UM,
    CFG_V_CAP_DMPS,
    CFG_HW_COUNTER_N,
)
DIFF_MAX_PM = 60  # FLYWHEEL tail clamp


def rpm_cap(wheel_d_um=40000, v_cap_dmps=380):
    """I13: floor(v_cap * 60 / (pi * wheel_d)) with the firmware's fall-back for out-of-range values; same expression order as the C core."""
    import math

    v = v_cap_dmps if 1 <= v_cap_dmps <= 380 else 380
    d = wheel_d_um if 10000 <= wheel_d_um <= 100000 else 40000
    return math.floor(float(v) * 6000000.0 / (math.pi * float(d)))


# ---- messages with tails ----------------------------------------------------------------------------------------------------------------
@dataclass
class Flywheel(_p.Msg):
    """speed_permille: fraction of rpm_max (0 = off). diff_pm: optional tail, bottom-faster differential permille of the mean (None = use cfg 29)."""

    ID = _p.FLYWHEEL
    speed_permille: int = 0
    diff_pm: int | None = None

    def pack(self) -> bytes:
        b = struct.pack("<H", self.speed_permille)
        return b if self.diff_pm is None else b + struct.pack("<h", self.diff_pm)

    @classmethod
    def unpack(cls, payload: bytes):
        if len(payload) < 2:
            raise ProtocolError(f"Flywheel: payload {len(payload)} < 2")
        (s,) = struct.unpack_from("<H", payload)
        (d,) = struct.unpack_from("<h", payload, 2) if len(payload) >= 4 else (None,)
        return cls(s, d)


@msg(_p.FIRE, "HB")
class Fire(_p.Msg):
    """n is fixed 1 in cuas2: the field stays so a v1 app still encodes it, and the MCU answers BAD_ARG to any other value."""

    fire_id: int = 0
    n: int = 1


_STATUS_TAIL = struct.Struct("<HHHBBBHHHB")
STATUS_LEN = 45


@dataclass
class Status2(_p.Msg):
    """STATUS, 45 bytes: v1 offsets 0-28 unchanged (fw_duty = mean throttle permille, fw_rpm = mean wheel rpm, 0xFFFF = telemetry lost) + 16 tail bytes."""

    ID = _p.STATUS
    t_ms: int = 0
    state: int = _p.SAFE
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
    rpm_top: int = 0xFFFF
    rpm_bot: int = 0xFFFF
    rpm_set: int = 0
    beams: int = 0
    gate_state: int = GS_UNKNOWN
    ready_flags: int = 0
    last_v_dmps: int = 0xFFFF
    last_dip_pm: int = 0
    shots_confirmed: int = 0
    feed_ms: int = 0

    _V1 = struct.Struct("<IBBBiiHHHBBHHH")

    def pack(self) -> bytes:
        v1 = self._V1.pack(self.t_ms, self.state, self.flags1, self.flags2, self.pan_steps, self.tilt_steps, self.fw_duty, self.fw_rpm, self.shots_total,
                           self.shots_arm, self.shot_cap, self.fault_bits, self.arm_left_s, self.last_fire_id)  # fmt: skip
        return v1 + struct.pack("<HHHBBBHHHB", self.rpm_top, self.rpm_bot, self.rpm_set, self.beams, self.gate_state, self.ready_flags, self.last_v_dmps,
                                self.last_dip_pm, self.shots_confirmed, self.feed_ms)  # fmt: skip

    @classmethod
    def unpack(cls, payload: bytes):
        """A 29-byte (v1) payload is accepted: the tail takes its defaults (telemetry lost, no ball data)."""
        n1 = cls._V1.size
        if len(payload) < n1:
            raise ProtocolError(f"Status2: payload {len(payload)} < {n1}")
        v1 = cls._V1.unpack_from(payload)
        if len(payload) < STATUS_LEN:
            return cls(*v1)
        return cls(*v1, *struct.unpack_from("<HHHBBBHHHB", payload, n1))

    def flag1(self, bit):
        return bool(self.flags1 & bit)

    def flag2(self, bit):
        return bool(self.flags2 & bit)

    @property
    def ready(self):
        return bool(self.ready_flags & RDY_READY)


MESSAGES2 = dict(_p.MESSAGES)
MESSAGES2.update({Flywheel.ID: Flywheel, Fire.ID: Fire, Status2.ID: Status2})
HOST_IDS = frozenset(i for i in MESSAGES2 if i < 0x80)
MCU_IDS = frozenset(i for i in MESSAGES2 if i >= 0x80)


def decode_body2(body: bytes) -> Frame:
    """Like napl.lite.protocol.decode_body with the cuas2 message table."""
    f = _p.decode_body(body)
    cls = MESSAGES2.get(f.id)
    if cls and cls is not _p.MESSAGES.get(f.id):
        f.msg, f.bad_arg = None, False
        try:
            f.msg = cls.unpack(f.payload)
        except ProtocolError:
            f.bad_arg = True
    return f


def decode2(wire: bytes) -> Frame:
    return decode_body2(_p.cobs_decode(wire[:-1] if wire.endswith(b"\0") else wire))


class Decoder2(_p.Decoder):
    """Streaming receiver with the cuas2 table: STATUS frames come out as Status2 (a 29-byte one too), FLYWHEEL with its tail."""

    def feed(self, data: bytes) -> list:
        out = super().feed(data)
        for f in out:
            cls = MESSAGES2.get(f.id)
            if cls and cls is not _p.MESSAGES.get(f.id):
                f.msg, f.bad_arg = None, False
                try:
                    f.msg = cls.unpack(f.payload)
                except ProtocolError:
                    f.bad_arg = True
        return out


def handshake_ok(ack: HelloAck) -> bool:
    """Host side: the MCU accepted our major and speaks the same major (any minor, cuas2 firmware reports fw_major 2)."""
    return ack.result == 0 and ack.major == MAJOR
