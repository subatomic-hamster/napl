"""A3/S3: the DYNAMIXEL Protocol 2.0 pty (python/napl/dynamixel.py) backed by the simulated servos of
tests/data/dynamixel/arm3.xml (s1, s2 = XL430-W250-T at 12 V, s3 = XL330-M288-T at 5 V). The raw client below is written from
the protocol spec with its own CRC / byte stuffing (nothing imported from the port but the constants); the `sdk` tests drive the
same bus with the unmodified ROBOTIS `dynamixel_sdk` (PyPI dynamixel-sdk, the `arm` extra), skipped if it is not installed."""

import os
import re
import select
import subprocess
import sys
import time
import tty
from math import pi
from pathlib import Path

import napl
import pytest
from napl.dynamixel import DynamixelPort

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "tests" / "data" / "dynamixel"
MODEL = DATA / "arm3.xml"
REPLAY_EXE = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))
SERVOS = {1: "/devices/s1", 2: "/devices/s2", 3: "/devices/s3"}
MODELS = {3: "xl330-m288-t"}
JOINTS = {1: "/world/l1/j1", 2: "/world/l2/j2", 3: "/world/l3/j3"}
CPR = 4096
VEL_UNIT = 0.229 * 2 * pi / 60  # rad/s per velocity count
PING, READ, WRITE, REG_WRITE, ACTION, FACTORY_RESET, REBOOT, CLEAR = (
    1,
    2,
    3,
    4,
    5,
    6,
    8,
    0x10,
)
SYNC_READ, SYNC_WRITE, BULK_READ, BULK_WRITE = 0x82, 0x83, 0x92, 0x93
BC = 0xFE
HDR = b"\xff\xff\xfd\x00"
# registers
ID, BAUD, RDT, MODE, HOMING, TEMP_LIM, MAXV, MINV, PWM_LIM, SHUTDOWN = (
    7,
    8,
    9,
    11,
    20,
    31,
    32,
    34,
    36,
    63,
)
TORQUE, SRL, REGISTERED, HWERR, POS_D, POS_I, POS_P, GOAL_PWM, PROF_VEL, GOAL = (
    64,
    68,
    69,
    70,
    80,
    82,
    84,
    100,
    112,
    116,
)
MOVING, PRES_PWM, PRES_LOAD, PRES_VEL, PRES_POS, PRES_V, PRES_T = (
    122,
    124,
    126,
    128,
    132,
    144,
    146,
)


def goal_rad(counts):
    return counts * 2 * pi / CPR


# ---- the client: own CRC, own byte stuffing -----------------------------------------------------------------------------
def crc(data):  # CRC-16, poly 0x8005, init 0, MSB first, bit by bit
    c = 0
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x8005) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def stuff(b):
    out = bytearray()
    for x in b:
        out.append(x)
        if bytes(out[-3:]) == b"\xff\xff\xfd":
            out.append(0xFD)
    return bytes(out)


def unstuff(b):
    out, i = bytearray(), 0
    while i < len(b):
        out.append(b[i])
        if bytes(out[-3:]) == b"\xff\xff\xfd" and i + 1 < len(b) and b[i + 1] == 0xFD:
            i += 1
        i += 1
    return bytes(out)


def frame(sid, instr, params=b""):
    body = stuff(bytes([instr]) + bytes(params))
    p = HDR + bytes([sid]) + (len(body) + 2).to_bytes(2, "little") + body
    return p + crc(p).to_bytes(2, "little")


class RawBus:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        self.last_tx = self.last_rx = b""

    def close(self):
        os.close(self.fd)

    def send_raw(self, data):
        self.last_tx = data
        os.write(self.fd, data)

    def send(self, sid, instr, params=b""):
        self.send_raw(frame(sid, instr, params))

    def recv(self, n=1, timeout=2.0):
        """n status packets -> [(id, err, params)]; fewer if the bus goes quiet. Checks header, LEN, CRC, instruction 0x55."""
        buf, out, end, raw = b"", [], time.monotonic() + timeout, b""
        while len(out) < n and time.monotonic() < end:
            if select.select([self.fd], [], [], 0.02)[0]:
                buf += os.read(self.fd, 4096)
            while len(buf) >= 7 and len(buf) >= 7 + (buf[5] | buf[6] << 8):
                assert buf[:4] == HDR
                ln = buf[5] | buf[6] << 8
                p, buf = buf[: 7 + ln], buf[7 + ln :]
                assert crc(p[:-2]) == int.from_bytes(p[-2:], "little"), p.hex(" ")
                assert p[7] == 0x55
                raw += p
                body = unstuff(p[8:-2])
                out.append((p[4], body[0], body[1:]))
        self.last_rx = raw
        return out

    def txrx(self, sid, instr, params=b""):
        self.send(sid, instr, params)
        r = self.recv()
        assert len(r) == 1, "no status packet"
        return r[0]

    def quiet(self, timeout=0.2):
        assert self.recv(1, timeout) == []

    def read(self, sid, addr, n, signed=False):
        i, err, data = self.txrx(
            sid, READ, addr.to_bytes(2, "little") + n.to_bytes(2, "little")
        )
        assert (i, err & 0x7F, len(data)) == (sid, 0, n), (
            i,
            err,
            data,
        )  # the Alert bit (0x80) is allowed
        return int.from_bytes(data, "little", signed=signed)

    def write_err(self, sid, addr, data):  # error byte of the status of a WRITE
        i, err, d = self.txrx(sid, WRITE, addr.to_bytes(2, "little") + bytes(data))
        assert (i, d) == (sid, b"")
        return err

    def write(self, sid, addr, value, n=1):
        err = self.write_err(sid, addr, value.to_bytes(n, "little", signed=value < 0))
        assert err & 0x7F == 0, err  # the Alert bit (0x80) is allowed

    def enable(self, sid):
        self.write(sid, TORQUE, 1)

    def pos(self, sid):
        return self.read(sid, PRES_POS, 4, signed=True)


@pytest.fixture
def sim():
    return napl.Sim.load(MODEL, host=True)


@pytest.fixture
def port(sim):
    with DynamixelPort(sim, SERVOS, models=MODELS, realtime=False) as p:
        yield p


@pytest.fixture
def bus(port):
    b = RawBus(port.device_path)
    yield b
    b.close()


def attr(sim, i, name):
    return sim.query(f"/devices/s{i}", name)[0][0]


TOL = 3  # counts: the model's friction (tanh loss torque) leaves a few counts of creep at low P


def settle(sim, read_pos, goal, tol=TOL, max_steps=4000):
    """Step the sim until the reading is within tol counts of goal and every servo is at rest; returns the steps used."""
    for k in range(0, max_steps, 50):
        sim.step(50)
        if abs(read_pos() - goal) <= tol and all(
            abs(attr(sim, i, "velocity")) < 5e-3 for i in (1, 2, 3)
        ):
            return k + 50
    return None


# ---- 1. framing: CRC, stuffing, e-Manual example packets ---------------------------------------------------------------
EMANUAL = [  # (instruction packet or status, from the protocol page examples)
    "FF FF FD 00 01 03 00 01 19 4E",  # ping
    "FF FF FD 00 01 07 00 55 00 06 04 26 65 5D",  # ping status (XM430 model number 0x0406, fw 0x26)
    "FF FF FD 00 01 07 00 02 84 00 04 00 1D 15",  # read present position
    "FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0",  # its status: 166
    "FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89",  # write goal position 512
    "FF FF FD 00 01 09 00 04 68 00 C8 00 00 00 AE 8E",  # reg write
    "FF FF FD 00 01 03 00 05 02 CE",  # action
    "FF FF FD 00 01 04 00 06 01 A1 E6",  # factory reset, option 1
    "FF FF FD 00 01 08 00 10 01 44 58 4C 22 B1 DC",  # clear
    "FF FF FD 00 FE 09 00 82 84 00 04 00 01 02 CE FA",  # sync read
    "FF FF FD 00 FE 11 00 83 74 00 04 00 01 96 00 00 00 02 AA 00 00 00 82 87",  # sync write
    "FF FF FD 00 FE 0D 00 92 01 90 00 02 00 02 92 00 01 00 1A 05",  # bulk read
    "FF FF FD 00 FE 10 00 93 01 20 00 02 00 A0 00 02 1F 00 01 00 50 B7 68",  # bulk write
]


@pytest.mark.parametrize("hexs", EMANUAL)
def test_crc_matches_emanual_example_packets(hexs):
    from napl.dynamixel import crc16, packet

    p = bytes.fromhex(hexs)
    assert crc(p[:-2]) == int.from_bytes(p[-2:], "little")  # the client's CRC
    assert crc16(p[:-2]) == int.from_bytes(p[-2:], "little")  # the port's
    if p[7] != 0x55:  # the port builds the same instruction packet
        assert packet(p[4], p[7], p[8:-2]) == p
        assert frame(p[4], p[7], p[8:-2]) == p


def test_byte_stuffing_round_trip_and_length_counts_stuffed_bytes():
    from napl.dynamixel import stuff as pstuff
    from napl.dynamixel import unstuff as punstuff

    cases = [
        b"",
        b"\xff\xff\xfd",
        b"\xff\xff\xfd\xfd",
        b"\xff\xff\xff\xfd",
        b"\x01\xff\xff\xfd\x00\xff\xff\xfd",
        b"\xff\xff",
    ]
    for c in cases:
        assert (
            unstuff(stuff(c)) == c
            and punstuff(pstuff(c)) == c
            and stuff(c) == pstuff(c)
        )
    assert stuff(b"\xff\xff\xfd") == b"\xff\xff\xfd\xfd"
    # an instruction packet whose data holds FF FF FD: the stuffed body follows the instruction, LEN counts the extra FD
    p = frame(1, WRITE, b"\x74\x00" + b"\xff\xff\xfd\xff")
    assert p[7:15] == b"\x03\x74\x00\xff\xff\xfd\xfd\xff"
    assert (
        int.from_bytes(p[5:7], "little") == 1 + 2 + 5 + 2
    )  # INSTR + addr + stuffed data + CRC


def test_stuffed_payload_goes_through_the_bus_both_ways(bus, sim):
    bus.write(
        1, MODE, 4
    )  # extended position: Goal Position takes -1,048,575 .. 1,048,575
    g = -131073  # little endian FF FF FD FF
    assert g.to_bytes(4, "little", signed=True) == b"\xff\xff\xfd\xff"
    bus.write(1, GOAL, g, 4)
    assert (
        b"\xff\xff\xfd\xfd\xff" in bus.last_tx
    )  # stuffed on the way in, the port acted on the unstuffed value
    assert sim.query("/devices/s1", "goal_position")[0][0] == pytest.approx(goal_rad(g))
    assert bus.read(1, GOAL, 4, signed=True) == g
    assert b"\xff\xff\xfd\xfd\xff" in bus.last_rx  # stuffed on the way out
    assert (
        int.from_bytes(bus.last_rx[5:7], "little") == len(bus.last_rx) - 7 == 9
    )  # LEN counts the stuffed byte (8 + 1)
    bus.write(1, GOAL, 0, 4)


# ---- 2. instructions ------------------------------------------------------------------------------------------------------
def test_ping_model_numbers_and_broadcast_ping_in_id_order(bus):
    assert bus.txrx(1, PING) == (
        1,
        0,
        bytes([0x24, 0x04, 52]),
    )  # 1060 = 0x0424, fw placeholder 52
    assert bus.txrx(3, PING)[2][:2] == (1200).to_bytes(2, "little")
    assert bus.read(2, 0, 2) == 1060 and bus.read(3, 0, 2) == 1200
    bus.send(9, PING)
    bus.quiet()  # nobody home
    bus.send(BC, PING)
    r = bus.recv(3)
    assert [(i, e, int.from_bytes(d[:2], "little")) for i, e, d in r] == [
        (1, 0, 1060),
        (2, 0, 1060),
        (3, 0, 1200),
    ]


def test_control_table_defaults_per_model(bus):
    xl430 = [
        (ID, 1, 1),
        (BAUD, 1, 3),
        (RDT, 1, 250),
        (MODE, 1, 3),
        (TEMP_LIM, 1, 72),
        (MAXV, 2, 140),
        (MINV, 2, 60),
        (PWM_LIM, 2, 885),
        (SHUTDOWN, 1, 52),
        (SRL, 1, 2),
        (76, 2, 1000),
        (78, 2, 100),
        (POS_D, 2, 4000),
        (POS_I, 2, 0),
        (POS_P, 2, 640),
        (44, 4, 265),
        (48, 4, 4095),
        (52, 4, 0),
        (24, 4, 10),
        (12, 1, 255),
        (13, 1, 2),
    ]
    for a, n, v in xl430:
        assert bus.read(1, a, n) == v, a
    xl330 = [
        (ID, 1, 3),
        (TEMP_LIM, 1, 70),
        (MAXV, 2, 70),
        (MINV, 2, 35),
        (SHUTDOWN, 1, 53),
        (POS_P, 2, 400),
        (POS_D, 2, 0),
        (38, 2, 1750),
        (62, 1, 140),
        (44, 4, 445),
        (102, 2, 1750),
    ]
    for a, n, v in xl330:
        assert bus.read(3, a, n) == v, a
    assert (
        bus.write_err(1, 38, b"\x00\x00") == 7
        and bus.write_err(1, 102, b"\x00\x00") == 7
    )  # XL430 has no current registers


def test_read_write_registers_map_to_device_attrs_and_stay_per_servo(bus, sim):
    from napl.dynamixel import I_SCALE, P_SCALE

    writes = [  # register, size, value, device attr, expected attr value
        (POS_P, 2, 800, "p_gain", 800 * P_SCALE),
        (POS_D, 2, 3000, "d_gain", 3000),
        (POS_I, 2, 512, "i_gain", 512 * I_SCALE),
        (PWM_LIM, 2, 442, "max_torque", 442 * 1000 / 885),
        (GOAL_PWM, 2, 200, "torque_limit", 200 * 1000 / 885),
        (TEMP_LIM, 1, 60, "max_temp", 60),
        (MAXV, 2, 130, "max_voltage", 13.0),
        (MINV, 2, 80, "min_voltage", 8.0),
        (SHUTDOWN, 1, 49, "unload_mask", 49 & 0x25),
        (PROF_VEL, 4, 100, "goal_speed", 100 * VEL_UNIT),
    ]
    for a, n, v, name, want in writes:
        bus.write(1, a, v, n)
        assert bus.read(1, a, n) == v
        assert attr(sim, 1, name) == pytest.approx(want), name
        assert attr(sim, 2, name) != pytest.approx(want), name  # only servo 1
    bus.write(1, GOAL, 1500, 4)  # mode 3: goal counts from the present turn (0)
    assert attr(sim, 1, "goal_position") == pytest.approx(goal_rad(1500))
    bus.write(1, 24, 20, 4)  # stored only: moving threshold
    assert bus.read(1, 24, 4) == 20
    bus.write(1, 65, 1)  # LED
    assert bus.read(1, 65, 1) == 1


def test_present_values_follow_the_sim_and_goal_reached(bus, sim):
    bus.enable(1)
    bus.enable(3)
    assert (
        bus.read(1, GOAL, 4) == bus.pos(1) == 0
    )  # enabling holds the present position
    bus.write(1, GOAL, 1000, 4)
    bus.write(3, GOAL, 800, 4)
    sim.step(60)
    assert bus.read(1, MOVING, 1) == 1
    assert (
        bus.read(1, 123, 1) & 2 == 0
    )  # no profile velocity: a step profile, "profile ongoing" stays 0
    v = bus.read(1, PRES_VEL, 4, signed=True)
    assert (
        v > 100 and abs(v - round(attr(sim, 1, "velocity") / VEL_UNIT)) <= 1
    )  # 0.229 rev/min units, moving up
    load = bus.read(1, PRES_LOAD, 2, signed=True)  # XL430: 0.1 % of full duty
    assert load > 200 and load == round(attr(sim, 1, "load") * 1000)
    assert bus.read(1, PRES_PWM, 2, signed=True) == round(attr(sim, 1, "load") * 885)
    cur = bus.read(
        3, PRES_LOAD, 2, signed=True
    )  # XL330: the same address is the present current in mA
    assert cur > 10 and cur == round(attr(sim, 3, "current") * 1000)
    assert 100 < bus.read(1, PRES_V, 2) <= 120  # ~12 V supply (0.1 V units)
    assert 25 <= bus.read(3, PRES_V, 2) <= 50
    assert bus.read(1, PRES_T, 1) == round(attr(sim, 1, "temperature"))
    assert settle(sim, lambda: bus.pos(1), 1000) and settle(
        sim, lambda: bus.pos(3), 800
    )
    assert abs(bus.pos(1) - 1000) <= TOL and abs(bus.pos(3) - 800) <= TOL
    assert (
        bus.read(1, MOVING, 1) == 0 and bus.read(1, 123, 1) & 1
    )  # at rest, in position
    assert bus.read(1, 140, 4, signed=True) == pytest.approx(
        1000, abs=2
    )  # position trajectory = the held setpoint


def test_profile_velocity_limits_the_speed(bus, sim):
    bus.enable(1)
    bus.write(
        1, PROF_VEL, 150, 4
    )  # 150 x 0.229 rev/min = 3.6 rad/s: well under the 6.4 rad/s no-load speed
    bus.write(1, GOAL, 1500, 4)
    sim.step(100)
    assert bus.read(1, MOVING, 1) == 1 and bus.read(1, 123, 1) & 2  # profile ongoing
    assert 0 < attr(sim, 1, "velocity") <= 150 * VEL_UNIT * 1.15
    assert settle(sim, lambda: bus.pos(1), 1500)
    assert bus.read(1, 123, 1) & 2 == 0


def test_mode3_wraps_the_reading_mode4_is_continuous(bus, sim):
    sim.set(JOINTS[1], "qpos", -goal_rad(100))
    sim.step(1)
    assert bus.pos(1) == CPR - 100  # mode 3: 0..4095
    bus.write(1, MODE, 4)
    sim.step(1)
    assert bus.pos(1) == -100  # extended position: continuous
    bus.enable(1)
    bus.write(1, GOAL, -600, 4)
    assert settle(sim, lambda: bus.pos(1), -600)
    assert abs(attr(sim, 1, "counts") + 600) <= TOL


def test_homing_offset_adds_to_the_actual_position(bus, sim):
    """e-Manual: Present Position = Actual Position + Homing Offset (the opposite sign of Feetech); goals use the same frame."""
    sim.set(JOINTS[1], "qpos", -goal_rad(100))
    sim.step(1)
    bus.write(1, HOMING, 300, 4)
    sim.step(1)
    assert bus.pos(1) == 200  # (-100 + 300) mod 4096
    bus.enable(1)
    bus.write(1, GOAL, 350, 4)  # register frame: 150 counts above the present reading
    assert settle(sim, lambda: bus.pos(1), 350)
    assert abs(attr(sim, 1, "counts") - 50) <= TOL  # -100 + 150 encoder counts
    bus.write(1, TORQUE, 0)
    bus.write(1, HOMING, -300, 4)
    sim.step(1)
    assert bus.pos(1) == (round(attr(sim, 1, "counts")) - 300) % CPR
    assert (
        bus.write_err(1, HOMING, (1044480).to_bytes(4, "little")) == 4
    )  # out of range


def test_mode3_goal_stays_in_the_present_turn_and_checks_the_limits(bus, sim):
    assert (
        bus.write_err(1, GOAL, (4096).to_bytes(4, "little")) == 6
    )  # beyond Max Position Limit: Data Limit
    bus.write(1, 48, 3000, 4)  # max position limit
    assert bus.write_err(1, GOAL, (3001).to_bytes(4, "little")) == 6
    bus.write(1, GOAL, 3000, 4)
    assert attr(sim, 1, "goal_position") == pytest.approx(
        goal_rad(3000)
    )  # no wrap to -1096
    bus.write(1, MODE, 4)
    bus.write(1, GOAL, 3500, 4)  # extended: the position limits are not applied
    assert bus.write_err(1, GOAL, (1048576).to_bytes(4, "little")) == 4  # Data Range


def test_reg_write_and_action(bus, sim):
    assert bus.txrx(2, ACTION)[:2] == (
        2,
        2,
    )  # Action without Reg Write: Instruction Error
    assert bus.txrx(
        2, REG_WRITE, (GOAL).to_bytes(2, "little") + (500).to_bytes(4, "little")
    )[:2] == (2, 0)
    assert bus.read(2, REGISTERED, 1) == 1
    assert (
        attr(sim, 2, "goal_position") == 0.0 and bus.read(2, GOAL, 4) == 0
    )  # buffered, not applied
    bus.send(BC, ACTION)
    bus.quiet()
    assert bus.read(2, REGISTERED, 1) == 0
    assert (
        attr(sim, 2, "goal_position") == pytest.approx(goal_rad(500))
        and bus.read(2, GOAL, 4) == 500
    )
    assert bus.txrx(2, ACTION)[:2] == (2, 2)  # nothing registered any more


def test_sync_read_and_sync_write(bus, sim):
    bus.send(
        BC,
        SYNC_READ,
        (0).to_bytes(2, "little") + (2).to_bytes(2, "little") + bytes([3, 1, 2]),
    )
    r = bus.recv(3)
    assert [(i, e, int.from_bytes(d, "little")) for i, e, d in r] == [
        (3, 0, 1200),
        (1, 0, 1060),
        (2, 0, 1060),
    ]  # request order
    body = (GOAL).to_bytes(2, "little") + (4).to_bytes(2, "little")
    for sid, g in (
        (1, 150),
        (3, 170),
    ):  # sync write goal position to ids 1 and 3, not 2
        body += bytes([sid]) + g.to_bytes(4, "little")
    bus.send(BC, SYNC_WRITE, body)
    bus.quiet()
    assert attr(sim, 1, "goal_position") == pytest.approx(goal_rad(150))
    assert attr(sim, 3, "goal_position") == pytest.approx(goal_rad(170))
    assert attr(sim, 2, "goal_position") == 0.0


def test_bulk_read_and_bulk_write(bus, sim):
    ent = lambda sid, a, n: (
        bytes([sid]) + a.to_bytes(2, "little") + n.to_bytes(2, "little")
    )
    bus.send(BC, BULK_READ, ent(1, PRES_V, 2) + ent(3, PRES_T, 1) + ent(2, 0, 2))
    r = bus.recv(3)
    assert [i for i, _, _ in r] == [1, 3, 2] and [len(d) for _, _, d in r] == [2, 1, 2]
    assert (
        int.from_bytes(r[0][2], "little") == bus.read(1, PRES_V, 2)
        and int.from_bytes(r[2][2], "little") == 1060
    )
    body = (
        ent(1, POS_P, 2)
        + (700).to_bytes(2, "little")
        + ent(2, GOAL, 4)
        + (123).to_bytes(4, "little")
    )
    bus.send(BC, BULK_WRITE, body)
    bus.quiet()
    assert bus.read(1, POS_P, 2) == 700 and attr(sim, 1, "p_gain") == pytest.approx(700)
    assert attr(sim, 2, "goal_position") == pytest.approx(goal_rad(123))


def test_broadcast_write_is_applied_and_never_answered(bus, sim):
    bus.send(BC, WRITE, TORQUE.to_bytes(2, "little") + b"\x01")
    bus.quiet()
    assert [attr(sim, i, "torque_enable") for i in (1, 2, 3)] == [1, 1, 1]
    bus.send(BC, READ, (0).to_bytes(2, "little") + (2).to_bytes(2, "little"))
    bus.quiet()  # Read is not executed on the broadcast ID


def test_factory_reset_options_and_id_change(bus, sim):
    bus.write(
        1, ID, 10
    )  # servo 1 becomes 10; the status of the write still carries the old id
    assert bus.txrx(10, PING)[0] == 10
    bus.send(1, PING)
    bus.quiet()
    bus.write(10, BAUD, 4)
    bus.write(10, POS_P, 100, 2)
    bus.write(10, RDT, 0)
    assert attr(sim, 1, "p_gain") == pytest.approx(100)
    assert bus.txrx(10, FACTORY_RESET, b"\x02")[:2] == (10, 0)  # all but ID and baud
    assert (
        bus.read(10, ID, 1),
        bus.read(10, BAUD, 1),
        bus.read(10, RDT, 1),
        bus.read(10, POS_P, 2),
    ) == (10, 4, 250, 640)
    assert attr(sim, 1, "p_gain") == pytest.approx(
        640
    )  # the defaults are pushed to the device
    assert bus.txrx(10, FACTORY_RESET, b"\x01")[:2] == (10, 0)  # all but ID
    assert (bus.read(10, ID, 1), bus.read(10, BAUD, 1)) == (10, 1)
    bus.send(BC, FACTORY_RESET, b"\xff")  # broadcast Reset All is not executed
    bus.quiet()
    assert bus.read(10, ID, 1) == 10
    assert bus.txrx(10, FACTORY_RESET, b"\x07")[1] == 4  # unknown option
    assert bus.txrx(10, FACTORY_RESET, b"\xff")[:2] == (10, 0)  # all: ID back to 1
    assert bus.txrx(1, PING)[0] == 1
    bus.send(10, PING)
    bus.quiet()


def test_reboot_resets_ram_keeps_eeprom_and_clears_hardware_error(bus, sim):
    bus.write(1, POS_P, 100, 2)
    bus.write(1, RDT, 10)
    bus.enable(1)
    bus.write(1, GOAL, 100, 4)
    assert bus.txrx(1, REBOOT)[:2] == (1, 0)
    assert (
        bus.read(1, POS_P, 2) == 640
        and bus.read(1, RDT, 1) == 10
        and bus.read(1, TORQUE, 1) == 0
    )
    assert attr(sim, 1, "torque_enable") == 0 and attr(
        sim, 1, "p_gain"
    ) == pytest.approx(640)
    # overheating: the temperature limit below ambient trips the device alarm; Shutdown(63) = 52 includes bit 2
    bus.write(1, TEMP_LIM, 5)
    sim.step(3)
    assert bus.txrx(1, PING)[1] == 0x80  # Alert only, no error number
    assert bus.read(1, HWERR, 1) == 0x04
    assert (
        bus.write_err(1, TORQUE, b"\x01") == 0x81
    )  # shut down: torque cannot be enabled (Result Fail + Alert) ...
    bus.write_err(1, TEMP_LIM, bytes([72]))
    sim.step(3)
    assert (
        bus.read(1, HWERR, 1) == 0x04 and bus.txrx(1, PING)[1] == 0x80
    )  # ... and the error stays latched
    assert bus.txrx(1, REBOOT)[:2] == (
        1,
        0x80,
    )  # the status of the reboot still shows the alert
    sim.step(3)
    assert bus.txrx(1, PING)[1] == 0 and bus.read(1, HWERR, 1) == 0  # REBOOT clears it
    bus.enable(1)
    assert attr(sim, 1, "torque_enable") == 1


def test_shutdown_releases_a_running_servo(bus, sim):
    bus.enable(1)
    bus.write(1, GOAL, 1000, 4)
    sim.step(30)
    bus.write_err(1, TEMP_LIM, bytes([5]))  # EEPROM, torque on: Access Error
    assert bus.read(1, TEMP_LIM, 1) == 72
    bus.write(1, TORQUE, 0)
    bus.write(1, TEMP_LIM, 5)
    bus.enable(1)  # alarm not yet seen: the servo runs ...
    sim.step(3)
    assert (
        bus.txrx(1, PING)[1] == 0x80
    )  # ... until the port polls the alarm: shutdown clears the torque
    assert bus.read(1, TORQUE, 1) == 0 and attr(sim, 1, "torque_enable") == 0


def test_clear_resets_the_multi_turn_position(bus, sim):
    clear = b"\x01\x44\x58\x4c\x22"
    bus.write(1, MODE, 4)
    sim.set(JOINTS[1], "qpos", goal_rad(CPR * 2 + 100))  # two turns and 100 counts
    sim.step(1)
    assert bus.pos(1) == 2 * CPR + 100
    assert bus.txrx(1, CLEAR, clear)[:2] == (1, 0)
    assert bus.pos(1) == 100
    assert (
        bus.txrx(1, CLEAR, b"\x02\x45\x52\x43\x4c")[1] == 2
    )  # clear error: Y series only
    assert bus.txrx(1, CLEAR, b"\x01\x00\x00\x00\x00")[1] == 4  # wrong magic bytes
    bus.write(1, TORQUE, 1)
    bus.write(1, GOAL, 3100, 4)
    sim.step(60)
    assert bus.txrx(1, CLEAR, clear)[1] == 1  # only while stopped: Result Fail


# ---- 3. error paths ---------------------------------------------------------------------------------------------------------
def test_error_paths_and_raw_bytes(bus, sim):
    addr = lambda a: a.to_bytes(2, "little")
    # bad CRC: not executed, CRC Error status
    bad = bytearray(frame(1, WRITE, addr(TORQUE) + b"\x01"))
    bad[-1] ^= 0x55
    bus.send_raw(bytes(bad))
    r = bus.recv()
    print(f"[bad crc] request {bytes(bad).hex(' ')}  ->  status {bus.last_rx.hex(' ')}")
    assert r == [(1, 3, b"")] and attr(sim, 1, "torque_enable") == 0
    b2 = bytearray(frame(BC, WRITE, addr(TORQUE) + b"\x01"))
    b2[-1] ^= 0x55
    bus.send_raw(bytes(b2))
    bus.quiet()
    assert attr(sim, 2, "torque_enable") == 0
    # the parser resyncs: garbage + a frame split across two writes
    good = frame(1, READ, addr(7) + addr(1))
    os.write(bus.fd, b"\x00\x12\xff\xff" + good[:5])
    time.sleep(0.01)
    os.write(bus.fd, good[5:])
    assert bus.recv()[0] == (1, 0, b"\x01")
    # unknown instruction, undefined / read-only / past-the-table access, wrong data length, ranges, limits
    cases = [
        ("unknown instruction", 1, 0x77, b"", 2),
        ("control table backup (not implemented)", 1, 0x20, b"\x01\x43\x54\x52\x4c", 2),
        ("write 1 byte of a 4-byte register", 1, WRITE, addr(GOAL) + b"\x01", 5),
        (
            "write straddling the table end",
            1,
            WRITE,
            addr(146) + b"\x01\x02\x03\x04",
            7,
        ),
        ("write undefined address", 1, WRITE, addr(200) + b"\x01", 7),
        ("write read-only", 1, WRITE, addr(PRES_POS) + bytes(4), 7),
        ("write read-only model number", 1, WRITE, addr(0) + bytes(2), 7),
        ("write with no data", 1, WRITE, addr(TORQUE), 5),
        ("baud index 8 on XL430", 1, WRITE, addr(BAUD) + b"\x08", 4),
        ("baud index 7 on XL330", 3, WRITE, addr(BAUD) + b"\x07", 4),
        ("operating mode 5 on XL430", 1, WRITE, addr(MODE) + b"\x05", 4),
        ("operating mode 2", 3, WRITE, addr(MODE) + b"\x02", 4),
        ("torque enable 2", 1, WRITE, addr(TORQUE) + b"\x02", 4),
        (
            "goal past the max position limit",
            1,
            WRITE,
            addr(GOAL) + (5000).to_bytes(4, "little"),
            6,
        ),
        (
            "goal PWM past the PWM limit",
            1,
            WRITE,
            addr(GOAL_PWM) + (886).to_bytes(2, "little"),
            4,
        ),
        ("PWM limit 886", 1, WRITE, addr(PWM_LIM) + (886).to_bytes(2, "little"), 4),
        (
            "goal current past the current limit (XL330)",
            3,
            WRITE,
            addr(102) + (1751).to_bytes(2, "little", signed=True),
            4,
        ),
        (
            "read-only registers read fine, undefined read",
            1,
            READ,
            addr(150) + addr(1),
            7,
        ),
        ("read across the table end", 1, READ, addr(146) + addr(4), 7),
        ("read length 0", 1, READ, addr(0) + addr(0), 5),
        ("READ with a short parameter block", 1, READ, b"\x00", 2),
        ("duplicate id", 1, WRITE, addr(ID) + b"\x02", 1),
    ]
    for name, sid, instr, params, want in cases:
        got = bus.txrx(sid, instr, params)
        assert got == (sid, want, b""), (name, got)
    # EEPROM write while the torque is on, and Result Fail for a mode with no firmware
    bus.enable(1)
    bus.send(1, WRITE, addr(BAUD) + b"\x03")
    r = bus.recv()
    print(
        f"[eeprom while torque on] request {bus.last_tx.hex(' ')}  ->  status {bus.last_rx.hex(' ')}"
    )
    assert r == [(1, 7, b"")] and bus.read(1, BAUD, 1) == 3
    bus.write(1, TORQUE, 0)
    bus.write(
        1, MODE, 16
    )  # PWM mode: accepted (EEPROM) but the device has no PWM firmware: torque enable refused
    assert bus.txrx(1, WRITE, addr(TORQUE) + b"\x01")[1] == 1
    bus.write(1, MODE, 1)
    assert bus.txrx(1, WRITE, addr(TORQUE) + b"\x01")[1] == 1
    bus.write(1, MODE, 4)
    bus.write(
        3, MODE, 5
    )  # XL330 current-based position: position firmware, Goal Current stored only
    bus.write(3, TORQUE, 1)
    assert attr(sim, 3, "torque_enable") == 1
    bus.write(3, 102, 300, 2)
    assert bus.read(3, 102, 2) == 300


def test_status_return_level(bus, sim):
    bus.write(1, SRL, 1)  # the write itself still answers (level read before the write)
    bus.send(1, WRITE, TORQUE.to_bytes(2, "little") + b"\x01")
    bus.quiet()  # level 1: PING and READ only
    assert attr(sim, 1, "torque_enable") == 1  # but it was executed
    assert bus.read(1, TORQUE, 1) == 1 and bus.txrx(1, PING)[:2] == (1, 0)
    bus.send(1, REBOOT)  # not answered at level 1
    bus.quiet()
    assert bus.read(1, SRL, 1) == 2  # RAM: back to the default after the reboot
    bus.write(1, SRL, 0)
    bus.send(1, READ, (7).to_bytes(2, "little") + (1).to_bytes(2, "little"))
    bus.quiet()  # level 0: PING only
    assert bus.txrx(1, PING)[:2] == (1, 0)
    bus.send(
        BC,
        SYNC_READ,
        (0).to_bytes(2, "little") + (2).to_bytes(2, "little") + bytes([1, 2]),
    )
    assert [i for i, _, _ in bus.recv(2)] == [
        2
    ]  # servo 1 (level 0) stays silent, servo 2 answers
    bus.send(1, WRITE, SRL.to_bytes(2, "little") + b"\x02")
    bus.quiet()
    assert bus.read(1, SRL, 1) == 2


def test_voltage_limit_alarm_is_reported_as_alert_without_shutdown(bus, sim):
    bus.write(
        1, MINV, 130, 2
    )  # 13.0 V > the 12 V supply: Input Voltage Error (bit 0, not in the XL430 default Shutdown 52)
    sim.step(3)
    assert bus.txrx(1, PING)[1] == 0x80
    assert bus.read(1, HWERR, 1) == 0x01
    bus.enable(1)  # allowed: the error is only reported
    assert bus.read(1, TORQUE, 1) == 1
    assert bus.txrx(2, PING)[1] == 0  # alarms are per servo
    bus.write(1, TORQUE, 0)
    bus.write(1, MINV, 60, 2)
    sim.step(3)
    assert (
        bus.txrx(1, PING)[1] == 0 and bus.read(1, HWERR, 1) == 0
    )  # not latched without a Shutdown bit


# ---- 4. replay and misc ---------------------------------------------------------------------------------------------------------
def test_pty_commands_are_in_the_log_and_replay_to_the_same_hash(sim, tmp_path):
    assert REPLAY_EXE.exists(), f"build the C++ exe first: {REPLAY_EXE}"
    with DynamixelPort(sim, SERVOS, models=MODELS, realtime=False) as p:
        b = RawBus(p.device_path)
        assert sim.call({"cmd_type": "Run", "cmd": {"rate": 0}})[
            "ok"
        ]  # as fast as possible
        b.enable(1)
        b.write(1, PROF_VEL, 300, 4)
        b.write(1, GOAL, 700, 4)
        for _ in range(2000):
            if abs(b.pos(1) - 700) <= TOL:
                break
        sim.call({"cmd_type": "Pause", "cmd": {}})
        b.close()
    log = sim.log()
    assert (
        log.count(b"dynamixel") >= 4
    )  # source of every SetAttr the pty produced (enable, goal hold, speed, goal, ...)
    f = tmp_path / "pty.naplog"
    f.write_bytes(log)
    out = subprocess.run(
        [str(REPLAY_EXE), str(f), "--root", str(DATA)],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    m = re.fullmatch(r"tick=(\d+) state_hash=([0-9a-f]{16})\n", out)
    assert m, out
    assert (int(m[1]), m[2]) == (sim.tick, f"{sim.state_hash():016x}")


def test_symlink_is_created_and_removed_and_unknown_model_rejected(sim, tmp_path):
    link = tmp_path / "ttyNAPL0"
    with DynamixelPort(sim, SERVOS, symlink=link, realtime=False) as p:
        assert os.path.realpath(link) == os.path.realpath(p.device_path)
    assert not link.exists() and not link.is_symlink()
    with pytest.raises(ValueError, match="unknown Dynamixel model"):
        DynamixelPort(sim, SERVOS, models={1: "xl999"}, realtime=False)


# ---- 5. the unmodified ROBOTIS SDK ---------------------------------------------------------------------------------------------------
@pytest.fixture
def sdk(port):
    sdk = pytest.importorskip("dynamixel_sdk")
    pytest.importorskip("serial")
    ph = sdk.PortHandler(port.device_path)
    pk = sdk.PacketHandler(2.0)
    # Host PTY termios speed is independent of the simulated wire's 1 Mbaud.
    assert ph.setBaudRate(115200 if sys.platform == "darwin" else 1_000_000)
    yield sdk, ph, pk
    ph.closePort()


def ok(result, error, pk, sdk):
    assert (result, error) == (sdk.COMM_SUCCESS, 0), (
        pk.getTxRxResult(result),
        pk.getRxPacketError(error),
    )


def test_sdk_ping_read_write_and_stuffing(sdk, sim):
    sdkm, ph, pk = sdk
    model, r, e = pk.ping(ph, 1)
    ok(r, e, pk, sdkm)
    assert model == 1060
    assert pk.ping(ph, 3)[0] == 1200
    found, r = pk.broadcastPing(ph)
    assert r == sdkm.COMM_SUCCESS and {i: m[0] for i, m in found.items()} == {
        1: 1060,
        2: 1060,
        3: 1200,
    }
    ok(*pk.write2ByteTxRx(ph, 1, POS_P, 900), pk, sdkm)
    v, r, e = pk.read2ByteTxRx(ph, 1, POS_P)
    ok(r, e, pk, sdkm)
    assert v == 900 and attr(sim, 1, "p_gain") == pytest.approx(900)
    ok(*pk.write1ByteTxRx(ph, 1, MODE, 4), pk, sdkm)
    ok(
        *pk.write4ByteTxRx(ph, 1, GOAL, 0xFFFDFFFF), pk, sdkm
    )  # bytes FF FF FD FF: the SDK stuffs it, the port unstuffs it
    v, r, e = pk.read4ByteTxRx(
        ph, 1, GOAL
    )  # ... and the reply is stuffed by the port, unstuffed by the SDK
    ok(r, e, pk, sdkm)
    assert v == 0xFFFDFFFF and attr(sim, 1, "goal_position") == pytest.approx(
        goal_rad(-131073)
    )
    r, e = pk.write1ByteTxRx(
        ph, 1, 200, 1
    )  # undefined address: Access Error, as an SDK error byte
    assert (r, e) == (sdkm.COMM_SUCCESS, 7)
    ok(*pk.factoryReset(ph, 1, 0x02), pk, sdkm)
    assert pk.read1ByteTxRx(ph, 1, MODE)[0] == 3
    ok(*pk.reboot(ph, 2), pk, sdkm)


def test_sdk_group_sync_and_bulk(sdk, sim):
    sdkm, ph, pk = sdk
    for i in (1, 2, 3):
        ok(*pk.write1ByteTxRx(ph, i, TORQUE, 1), pk, sdkm)
    goals = {1: 1000, 2: 600, 3: 900}
    gw = sdkm.GroupSyncWrite(ph, pk, GOAL, 4)
    for i, g in goals.items():
        assert gw.addParam(i, list(g.to_bytes(4, "little")))
    assert gw.txPacket() == sdkm.COMM_SUCCESS
    gw.clearParam()
    for i, g in goals.items():
        assert settle(
            sim, lambda i=i: pk.read4ByteTxRx(ph, i, PRES_POS)[0], g, max_steps=5000
        ), i
    gr = sdkm.GroupSyncRead(ph, pk, PRES_POS, 4)
    for i in goals:
        assert gr.addParam(i)
    assert gr.txRxPacket() == sdkm.COMM_SUCCESS
    for i, g in goals.items():
        assert (
            gr.isAvailable(i, PRES_POS, 4)
            and abs(gr.getData(i, PRES_POS, 4) - g) <= TOL
        )
    br = sdkm.GroupBulkRead(ph, pk)
    assert (
        br.addParam(1, PRES_V, 2)
        and br.addParam(2, PRES_T, 1)
        and br.addParam(3, PRES_LOAD, 2)
    )
    assert br.txRxPacket() == sdkm.COMM_SUCCESS
    assert 100 < br.getData(1, PRES_V, 2) <= 120
    assert br.getData(2, PRES_T, 1) == round(attr(sim, 2, "temperature"))
    cur = br.getData(
        3, PRES_LOAD, 2
    )  # XL330 present current (mA, signed) at rest; the SDK returns it unsigned
    assert abs((cur + 32768) % 65536 - 32768) < 100
    bw = sdkm.GroupBulkWrite(ph, pk)
    assert bw.addParam(1, POS_P, 2, [0x20, 0x03]) and bw.addParam(
        2, GOAL, 4, list((10).to_bytes(4, "little"))
    )
    assert bw.txPacket() == sdkm.COMM_SUCCESS
    time.sleep(0.05)
    assert pk.read2ByteTxRx(ph, 1, POS_P)[0] == 0x320 and attr(
        sim, 2, "goal_position"
    ) == pytest.approx(goal_rad(10))


@pytest.fixture
def live(sim):
    """Sim free-running at real time (Host rate 1), port with realtime wire timing, SDK client."""
    sdk = pytest.importorskip("dynamixel_sdk")
    with DynamixelPort(sim, SERVOS, models=MODELS, realtime=True) as p:
        ph = sdk.PortHandler(p.device_path)
        assert ph.setBaudRate(115200 if sys.platform == "darwin" else 1_000_000)
        assert sim.call({"cmd_type": "Run", "cmd": {"rate": 1.0}})["ok"]
        yield sdk, ph, sdk.PacketHandler(2.0), sim, p
        sim.call({"cmd_type": "Pause", "cmd": {}})
        ph.closePort()


def test_sdk_drives_servos_to_goals_in_physical_time(live):
    sdk, ph, pk, sim, port = live
    t0 = sim.tick
    ok(*pk.write1ByteTxRx(ph, 1, TORQUE, 1), pk, sdk)
    ok(*pk.write1ByteTxRx(ph, 3, TORQUE, 1), pk, sdk)
    ok(*pk.write4ByteTxRx(ph, 1, GOAL, 1000), pk, sdk)
    ok(*pk.write4ByteTxRx(ph, 3, GOAL, 700), pk, sdk)
    t_end = time.monotonic() + 8.0
    while time.monotonic() < t_end:
        p1, p3 = (
            pk.read4ByteTxRx(ph, 1, PRES_POS)[0],
            pk.read4ByteTxRx(ph, 3, PRES_POS)[0],
        )
        v1 = pk.read4ByteTxRx(ph, 1, PRES_VEL)[0]
        if abs(p1 - 1000) <= TOL and abs(p3 - 700) <= TOL and abs(v1) <= 2:
            break
        time.sleep(0.01)
    else:
        pytest.fail(f"goals not reached: {p1}, {p3}")
    # cannot beat the 12 V no-load speed (6.39 rad/s, part robotis-xl430-w250-t): 1000 counts = 1.53 rad >= 0.24 s
    assert (sim.tick - t0) * 0.001 >= goal_rad(1000) / 6.3879
    assert port.errors == []
