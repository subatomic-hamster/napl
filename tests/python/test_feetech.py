"""N6a: the Feetech SCS/STS protocol over a pty, backed by the simulated STS3215 servos of tests/data/feetech/arm3.xml
(part feetech-sts3215-c001). The raw client below is written from the protocol spec; the `sdk` client is the unmodified
official Feetech Python SDK (module scservo_sdk, PyPI feetech-servo-sdk), used only if the environment has it installed (it
is NOT a project dependency: see docs in python/napl/feetech.py and the N6a report for the licence finding)."""

import os
import re
import select
import subprocess
import time
import tty
from math import pi
from pathlib import Path

import napl
import pytest
from napl.feetech import (
    ACTION,
    BROADCAST,
    PING,
    READ,
    REG_WRITE,
    SYNC_READ,
    SYNC_WRITE,
    WRITE,
    FeetechPort,
    checksum,
    packet,
)

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "tests" / "data" / "feetech"
MODEL = DATA / "arm3.xml"
REPLAY_EXE = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))
SERVOS = {1: "/devices/s1", 2: "/devices/s2", 3: "/devices/s3"}
JOINTS = {1: "/world/l1/j1", 2: "/world/l2/j2", 3: "/world/l3/j3"}
CPR = 4096
NO_LOAD_SPEED = 5.4542  # rad/s at 7.4 V, part feetech-sts3215-c001 no_load_speed_7v4
GOAL = {1: 1000, 2: 3000}  # counts


def goal_rad(counts):
    return counts * 2 * pi / CPR


class RawBus:
    """Protocol client written from the spec (FF FF ID LEN INSTR PARAMS CHK) on the pty slave."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)

    def close(self):
        os.close(self.fd)

    def send(self, sid, instr, params=b""):
        os.write(self.fd, packet(sid, instr, params))

    def recv(self, n=1, timeout=2.0):
        """n status packets -> [(id, error, params)]; fewer if the bus goes quiet (timeout)."""
        buf, out, end = b"", [], time.monotonic() + timeout
        while len(out) < n and time.monotonic() < end:
            if select.select([self.fd], [], [], 0.02)[0]:
                buf += os.read(self.fd, 256)
            while len(buf) >= 4 and len(buf) >= buf[3] + 4:
                p, buf = buf[: buf[3] + 4], buf[buf[3] + 4 :]
                assert p[:2] == b"\xff\xff" and checksum(p[2:-1]) == p[-1]
                out.append((p[2], p[4], p[5:-1]))
        return out

    def txrx(self, sid, instr, params=b""):
        self.send(sid, instr, params)
        r = self.recv()
        assert len(r) == 1, "no status packet"
        return r[0]

    def read(self, sid, addr, n):
        i, err, data = self.txrx(sid, READ, bytes([addr, n]))
        assert (i, err, len(data)) == (sid, 0, n)
        return int.from_bytes(data, "little")

    def write(self, sid, addr, value, n=1):
        assert self.txrx(sid, WRITE, bytes([addr]) + value.to_bytes(n, "little"))[
            :2
        ] == (sid, 0)

    def enable(self, sid):
        self.write(sid, 40, 1)

    def write_pos(
        self, sid, counts
    ):  # acc 0, position, time 0, speed 0 (the SDK's WritePosEx layout, addr 41)
        self.write_block(sid, 41, bytes([0]) + counts.to_bytes(2, "little") + bytes(4))

    def write_block(self, sid, addr, data):
        assert self.txrx(sid, WRITE, bytes([addr]) + data)[:2] == (sid, 0)

    def read_pos(self, sid):
        return self.read(sid, 56, 2)


class SdkBus:
    """The same operations through the unmodified Feetech SDK."""

    def __init__(self, path):
        sdk = pytest.importorskip("scservo_sdk")
        pytest.importorskip("serial")
        self.sdk, self.ph = sdk, sdk.PortHandler(path)
        self.pk = sdk.PacketHandler(0)
        assert self.ph.openPort() and self.ph.setBaudRate(1_000_000)

    def close(self):
        self.ph.closePort()

    def _ok(self, result, error):
        assert (result, error) == (self.sdk.COMM_SUCCESS, 0), self.pk.getTxRxResult(
            result
        )

    def enable(self, sid):
        self._ok(*self.pk.write1ByteTxRx(self.ph, sid, 40, 1))

    def write_pos(self, sid, counts):  # the SDK sample's WritePosEx
        data = [0, counts & 0xFF, counts >> 8, 0, 0, 0, 0]
        self._ok(*self.pk.writeTxRx(self.ph, sid, 41, 7, data))

    def read_pos(self, sid):
        v, r, e = self.pk.read2ByteTxRx(self.ph, sid, 56)
        self._ok(r, e)
        return v


@pytest.fixture
def sim():
    return napl.Sim.load(MODEL, host=True)


@pytest.fixture
def port(sim):
    with FeetechPort(sim, SERVOS, realtime=False) as p:
        yield p


@pytest.fixture
def bus(port):
    b = RawBus(port.device_path)
    yield b
    b.close()


# ---- 1. protocol unit tests -----------------------------------------------------------------------------------------
def test_checksum_and_packet_framing():
    assert packet(1, PING) == bytes(
        [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]
    )  # ~(1 + 2 + 1) & 0xFF
    assert checksum(bytes([0x01, 0x04, 0x02, 0x38, 0x02])) == 0xBE


def test_bad_checksum_and_garbage_are_dropped(port, bus, sim):
    bad = bytearray(packet(1, WRITE, bytes([40, 1])))
    bad[-1] ^= 0x55
    os.write(
        bus.fd, b"\x00\x12\xff" + bytes(bad)
    )  # leading garbage + frame with a wrong checksum
    assert bus.recv(1, 0.2) == []
    assert sim.query("/devices/s1", "torque_enable")[0][0] == 0  # no action
    assert port.errors == ["bad checksum"]
    # the parser has resynced: a good frame split across two writes is served
    good = packet(1, READ, bytes([5, 1]))
    os.write(bus.fd, good[:3])
    time.sleep(0.01)
    os.write(bus.fd, good[3:])
    assert bus.recv()[0] == (1, 0, bytes([1]))


def test_ping_and_model_number_and_unknown_id(bus):
    assert bus.txrx(2, PING) == (2, 0, b"")
    assert bus.read(2, 3, 2) == 777  # STS3215 model number (LeRobot MODEL_NUMBER_TABLE)
    assert bus.read(3, 5, 1) == 3
    bus.send(9, PING)
    assert bus.recv(1, 0.2) == []  # nobody home


def test_read_write_ram_register_persists_in_port_not_sim(port, bus, sim):
    assert (
        bus.read(1, 21, 1) == 32
        and bus.read(1, 22, 1) == 32
        and bus.read(1, 23, 1) == 0
    )  # part defaults P / D / I
    assert bus.read(1, 16, 2) == 1000  # max torque, 0.1 %
    bus.write(1, 21, 16)  # LeRobot's SO-101 P coefficient
    assert bus.read(1, 21, 1) == 16 and bus.read(2, 21, 1) == 32
    bus.write(1, 44, 0x1234, 2)  # goal time: little-endian word, stored
    assert bus.read(1, 44, 1) == 0x34 and bus.read(1, 45, 1) == 0x12
    bus.write(1, 56, 1234, 2)  # present position is read-only
    assert bus.read_pos(1) == 0
    bus.write(1, 55, 1)  # lock: EEPROM (addr < 40) becomes read-only
    bus.write(1, 21, 1)
    assert bus.read(1, 21, 1) == 16
    bus.write(1, 41, 7)  # RAM still writable
    assert bus.read(1, 41, 1) == 7
    assert port.errors == []


def test_sign_magnitude_registers():
    from napl.feetech import _sm, _to_sm

    assert _to_sm(-300, 15) == 0x8000 | 300 and _sm(0x8000 | 300, 15) == -300
    assert (
        _to_sm(-5, 10) == 0x400 | 5 and _sm(0x400 | 5, 10) == -5 and _sm(77, 10) == 77
    )


def test_sync_write_goal_positions_and_broadcast(bus, sim):
    bus.send(BROADCAST, WRITE, bytes([40, 1]))  # broadcast: applied, never answered
    assert bus.recv(1, 0.2) == []
    assert [sim.query(f"/devices/s{i}", "torque_enable")[0][0] for i in (1, 2, 3)] == [
        1,
        1,
        1,
    ]
    body = bytes(
        [42, 2]
    )  # sync write goal position (addr 42, 2 bytes) to ids 1 and 3, not 2
    for sid, g in ((1, 100), (3, 4000)):
        body += bytes([sid]) + g.to_bytes(2, "little")
    bus.send(BROADCAST, SYNC_WRITE, body)
    assert bus.recv(1, 0.2) == []
    time.sleep(0.1)
    q = lambda i: sim.query(f"/devices/s{i}", "goal_position")[0][0]
    assert q(1) == pytest.approx(goal_rad(100)) and q(3) == pytest.approx(
        goal_rad(4000)
    )
    assert q(2) == pytest.approx(0.0)


def test_sync_read_replies_in_order_and_reg_write_action(bus, sim):
    bus.send(
        BROADCAST, SYNC_READ, bytes([3, 2, 3, 1, 2])
    )  # model number of ids 3, 1, 2
    r = bus.recv(3)
    assert [(i, int.from_bytes(d, "little")) for i, _, d in r] == [
        (3, 777),
        (1, 777),
        (2, 777),
    ]
    assert bus.txrx(2, REG_WRITE, bytes([42]) + (500).to_bytes(2, "little"))[:2] == (
        2,
        0,
    )
    time.sleep(0.05)
    assert (
        sim.query("/devices/s2", "goal_position")[0][0] == 0.0
    )  # buffered, not applied
    bus.send(BROADCAST, ACTION)
    time.sleep(0.1)
    assert sim.query("/devices/s2", "goal_position")[0][0] == pytest.approx(
        goal_rad(500)
    )


def test_register_writes_map_to_device_attrs(bus, sim):
    bus.enable(1)
    bus.write_block(1, 46, (1000).to_bytes(2, "little"))  # goal speed 1000 steps/s
    assert sim.query("/devices/s1", "goal_speed")[0][0] == pytest.approx(goal_rad(1000))
    bus.write(1, 40, 0)
    assert sim.query("/devices/s1", "torque_enable")[0][0] == 0
    bus.write(1, 11, 2000, 2)  # max angle limit clamps the goal
    bus.write_pos(1, 3000)
    assert sim.query("/devices/s1", "goal_position")[0][0] == pytest.approx(
        goal_rad(2000)
    )


def test_present_values_follow_the_sim(bus, sim):
    sim.set("/devices/s1", "torque_enable", 0)  # power-up state: torque off
    bus.enable(1)
    bus.write_pos(1, 200)
    sim.step(600)
    assert abs(bus.read_pos(1) - 200) <= 2
    assert bus.read(1, 62, 1) == pytest.approx(
        round(sim.query("/devices/s1", "voltage")[0][0] * 10), abs=1
    )
    assert 60 < bus.read(1, 62, 1) <= 74  # ~7.4 V supply (0.1 V units)
    assert bus.read(1, 63, 1) == round(sim.query("/devices/s1", "temperature")[0][0])
    assert bus.read(1, 66, 1) == 0  # arrived
    bus.write_pos(1, 3000)
    sim.step(20)
    assert bus.read(1, 66, 1) == 1  # moving
    sim.step(40)
    spd = bus.read(1, 58, 2)
    assert 0 < spd < 0x8000  # moving up: positive, sign bit clear
    assert bus.read(1, 69, 2) > 0  # current while driving


def test_negative_encoder_reads_wrapped_single_turn(bus, sim):
    sim.set(
        JOINTS[1], "qpos", -goal_rad(100)
    )  # the sim encoder is unwrapped: -100 counts
    sim.step(1)
    assert bus.read_pos(1) == CPR - 100


# ---- 2/3. SDK / raw client against the running sim ---------------------------------------------------------------------
@pytest.fixture(params=["raw", "sdk"])
def live(request, sim):
    """Sim free-running at real time (Host rate 1), port with realtime wire timing, and a client."""
    with FeetechPort(sim, SERVOS, realtime=True) as p:
        c = (RawBus if request.param == "raw" else SdkBus)(p.device_path)
        assert sim.call({"cmd_type": "Run", "cmd": {"rate": 1.0}})["ok"]
        yield sim, p, c
        sim.call({"cmd_type": "Pause", "cmd": {}})
        c.close()


def wait_for(read, goal, tol=2, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if abs(read() - goal) <= tol:
            return True
        time.sleep(0.01)
    return False


def test_client_drives_sim_joint_to_goal_in_physical_time(live):
    sim, port, c = live
    t0 = sim.tick
    c.enable(1)
    c.write_pos(1, GOAL[1])
    assert wait_for(lambda: c.read_pos(1), GOAL[1], tol=2)
    sim_time = (sim.tick - t0) * 0.001
    # cannot beat the no-load speed (part file: 5.454 rad/s); a 1000-count move at P = 32 settles within a few seconds
    assert sim_time >= goal_rad(GOAL[1]) / NO_LOAD_SPEED
    assert sim_time < 3.0
    err = abs(c.read_pos(1) - GOAL[1])
    assert (
        err <= 2
    )  # documented resolution: 1 count = 2 pi / 4096 = 0.088 deg (+-1 count of settling)
    assert port.errors == []


def test_sim_sees_what_the_client_wrote(live):
    sim, port, c = live
    for sid in (1, 2):
        c.enable(sid)
        c.write_pos(sid, GOAL[sid])
    for sid in (1, 2):
        assert wait_for(lambda: c.read_pos(sid), GOAL[sid])
    sim.call(
        {"cmd_type": "Pause", "cmd": {}}
    )  # freeze: protocol and sim see the same tick
    for sid, g in GOAL.items():
        path = SERVOS[sid]
        assert sim.query(path, "goal_position")[0][0] == pytest.approx(goal_rad(g))
        assert sim.query(path, "torque_enable")[0][0] == 1
        counts = round(sim.query(path, "counts")[0][0])
        assert abs(sim.query(JOINTS[sid], "qpos")[0][0] - goal_rad(g)) < 3 * goal_rad(
            1
        )  # the joint moved
        assert c.read_pos(sid) == counts % CPR
    assert (
        sim.query("/devices/s3", "goal_position")[0][0] == 0.0 and c.read_pos(3) == 0
    )  # untouched servo stays


# ---- 4. replay ---------------------------------------------------------------------------------------------------------
def test_pty_commands_are_in_the_log_and_replay_to_the_same_hash(sim, tmp_path):
    assert REPLAY_EXE.exists(), f"build the C++ exe first: {REPLAY_EXE}"
    with FeetechPort(sim, SERVOS, realtime=False) as p:
        b = RawBus(p.device_path)
        assert sim.call({"cmd_type": "Run", "cmd": {"rate": 0}})[
            "ok"
        ]  # as fast as possible
        b.enable(1)
        b.write_pos(1, 700)
        b.write_block(1, 46, (500).to_bytes(2, "little"))
        assert wait_for(lambda: b.read_pos(1), 700, tol=2, timeout=10)
        sim.call({"cmd_type": "Pause", "cmd": {}})
        b.close()
    log = sim.log()
    assert (
        log.count(b"feetech") >= 4
    )  # source of every SetAttr the pty produced (enable, goal, speed, ...)
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


def test_symlink_is_created_and_removed(sim, tmp_path):
    link = tmp_path / "ttyNAPL0"
    with FeetechPort(sim, SERVOS, symlink=link, realtime=False) as p:
        assert os.path.realpath(link) == os.path.realpath(p.device_path)
    assert not link.exists() and not link.is_symlink()
