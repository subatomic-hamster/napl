"""A4: the RoboClaw Packet Serial pty (python/napl/roboclaw.py) backed by the two wheel servos of assemblies/rover.toml.

The raw client below is written from the manual (rev 5.5) with its own CRC16 (the manual's C function, transliterated; nothing imported from
the port): byte-exact vectors are asserted both against CRCs computed by it and against hex literals. The `basicmicro` tests drive the same
port with the unmodified official library (PyPI basicmicro, the `rover` extra), skipped if it is not installed."""

import os
import select
import struct
import time
import tty

import napl
import pytest
from napl import design
from napl.roboclaw import RoboclawPort, crc16, packet

ADDR = 0x80
MOTORS = {1: "/devices/wheel_l", 2: "/devices/wheel_r"}
M = 1200 / (2 * 3.141592653589793 * 0.045)  # counts per metre of wheel travel


def crc_manual(data):  # the manual's crc16(), "CRC16 Checksum Calculation"
    crc = 0
    for byte in data:
        crc = crc ^ (byte << 8)
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF


def frame(cmd, data=b""):
    p = bytes([ADDR, cmd]) + data
    return p + struct.pack(">H", crc_manual(p))


class Bus:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)

    def close(self):
        os.close(self.fd)

    def recv(self, n, timeout=2.0):
        buf, end = b"", time.monotonic() + timeout
        while len(buf) < n and time.monotonic() < end:
            if select.select([self.fd], [], [], 0.02)[0]:
                buf += os.read(self.fd, 4096)
        return buf

    def write(self, cmd, data=b""):  # a write command: the ack byte
        os.write(self.fd, frame(cmd, data))
        assert self.recv(1) == b"\xff", f"no ack for command {cmd}"

    def read(
        self, cmd, n
    ):  # a read command: [ADDR, CMD] only -> n data bytes + CRC16 over ADDR CMD DATA
        os.write(self.fd, bytes([ADDR, cmd]))
        r = self.recv(n + 2)
        assert len(r) == n + 2, f"short reply to command {cmd}: {r.hex()}"
        assert crc_manual(bytes([ADDR, cmd]) + r[:-2]) == struct.unpack(">H", r[-2:])[0]
        return r[:-2]

    def silent(self, raw, timeout=0.2):  # bytes that must get no reply at all
        os.write(self.fd, raw)
        assert self.recv(1, timeout) == b""

    def enc(self, m):
        c, st = struct.unpack(">IB", self.read(15 + m, 5))
        return c, st

    def speed(self, m):
        s, d = struct.unpack(">IB", self.read(17 + m, 5))
        return -s if d else s

    def pwm(self):
        return struct.unpack(">hh", self.read(48, 4))

    def amps(self):
        return [x / 100 for x in struct.unpack(">hh", self.read(49, 4))]


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    out = tmp_path_factory.mktemp("rover")
    xml, _ = design.compute("assemblies/rover.toml", "parts", out)
    (out / "rover.xml").write_text(xml)
    return out / "rover.xml"


@pytest.fixture
def sim(model):
    s = napl.Sim.load(model, host=True)
    s.step(500)  # settle on the floor
    return s


@pytest.fixture
def port(sim):
    with RoboclawPort(sim, MOTORS, address=ADDR, realtime=False) as p:
        yield p


@pytest.fixture
def bus(port):
    b = Bus(port.device_path)
    yield b
    b.close()


# ---- 1. wire format ----------------------------------------------------------------------------------------------------------
def test_crc16_is_xmodem_and_matches_the_manuals_function():
    assert (
        crc16(b"123456789") == crc_manual(b"123456789") == 0x31C3
    )  # CRC-16/XMODEM check value
    assert crc16(b"") == 0
    data = bytes(range(256)) * 3
    assert crc16(data) == crc_manual(data)


VECTORS = {  # request bytes with the CRC computed by the manual's algorithm (the manual prints no packet with a CRC)
    "drive M1 forward 64": (packet(ADDR, 0, bytes([64])), "800040739e"),
    "duty M1 / M2 +-16384": (
        packet(ADDR, 34, struct.pack(">hh", 16384, -16384)),
        "80224000c000e0df",
    ),
    "velocity PID M1 (D, P, I, QPPS = the manual defaults)": (
        packet(ADDR, 28, struct.pack(">4I", 0x4000, 0x10000, 0x8000, 44000)),
        "801c0000400000010000000080000000abe026fa",
    ),
    "reset encoders": (packet(ADDR, 20), "8014492d"),
    "speed M1 / M2 +-1000": (
        packet(ADDR, 37, struct.pack(">ii", 1000, -1000)),
        "8025000003e8fffffc188290",
    ),
}


@pytest.mark.parametrize("name", VECTORS)
def test_request_vectors_are_byte_exact(name):
    built, literal = VECTORS[name]
    assert built.hex() == literal
    cmd = built[1]
    assert (
        frame(cmd, built[2:-2]) == built
    )  # the independent client builds the same bytes


def test_vectors_are_acknowledged_with_ff(bus, port):
    for name, (_, literal) in VECTORS.items():
        os.write(bus.fd, bytes.fromhex(literal))
        assert bus.recv(1) == b"\xff", name
    assert port.errors == []


def test_reply_bytes_are_exact(bus, sim):
    sim.step(1)
    v = round(float(sim.query("/devices/wheel_l", "voltage")[0][0]) * 10)
    assert v == 123  # the full 3S pack, 12.31 V
    os.write(bus.fd, bytes([ADDR, 24]))
    assert bus.recv(4) == bytes.fromhex(
        "007bf806"
    )  # value 123 (0.1 V), CRC over 80 18 00 7b
    os.write(bus.fd, bytes([ADDR, 16]))
    assert bus.recv(7) == frame(16, bytes(5))[2:]  # count 0, status 0
    os.write(bus.fd, bytes([ADDR, 21]))
    r = bus.recv(60, 0.3)
    assert b"\n\x00" in r  # the string ends with LF, NUL (manual), then the CRC
    s = r.index(b"\n\x00") + 2
    assert crc_manual(bytes([ADDR, 21]) + r[:s]) == struct.unpack(">H", r[s : s + 2])[0]


def test_bad_packets_get_no_reply_and_the_port_recovers(bus, port):
    good = bytes.fromhex("800040739e")
    bus.silent(good[:-1] + b"\x00")  # bad CRC
    bus.silent(bytes([0x81]) + good[1:])  # another address (even with a stale CRC)
    bus.silent(bytes([ADDR, 0xAA, 1, 2, 3, 4]))  # unknown command
    bus.silent(good[:3])  # partial packet: dropped after 10 ms ...
    time.sleep(0.05)
    bus.write(0, bytes([64]))  # ... and the next valid packet is served
    assert len(port.errors) >= 3


def test_comms_drop_flag_discards_requests(bus, port):
    port.drop = True
    bus.silent(bytes.fromhex("800040739e"))
    port.drop = False
    bus.write(0, bytes([0]))


# ---- 2. the commands move the simulated rover -----------------------------------------------------------------------------------
def test_duty_commands_drive_both_wheels_and_encoders_count_up(bus, sim):
    bus.write(20)
    bus.write(34, struct.pack(">hh", 16384, 16384))
    sim.step(1500)
    (c1, s1), (c2, s2) = bus.enc(1), bus.enc(2)
    assert (
        c1 > 3000 and c2 > 3000 and abs(c1 - c2) < 0.05 * c1
    )  # both count up for a forward drive
    assert s1 & 2 == 0 and s2 & 2 == 0  # direction: forward
    p1, p2 = bus.pwm()
    assert p1 == p2 == pytest.approx(16384, abs=2)
    i1, i2 = bus.amps()
    assert 0.05 < i1 < 1.5 and 0.05 < i2 < 1.5
    assert bus.speed(1) > 1000
    bus.write(34, struct.pack(">hh", 0, 0))
    sim.step(2000)
    assert abs(bus.speed(1)) < 50 and bus.pwm() == (0, 0)


def test_seven_bit_commands(bus, sim):
    bus.write(0, bytes([127]))  # M1 full forward
    bus.write(5, bytes([127]))  # M2 full backwards
    sim.step(50)
    assert bus.pwm() == (32767, -32767)
    bus.write(6, bytes([64]))  # M1 stop
    bus.write(7, bytes([0]))  # M2 full reverse
    sim.step(50)
    assert bus.pwm() == (0, -32767)
    bus.write(1, bytes([64]))  # M1 backwards at 64 / 127
    sim.step(50)
    assert bus.pwm()[0] == pytest.approx(-0.504 * 32767, abs=40)


def test_reverse_wraps_the_unsigned_counter_and_sets_the_underflow_flag(bus, sim):
    bus.write(20)
    bus.write(32, struct.pack(">h", -16384))  # M1 duty -50 %
    sim.step(1000)
    c, st = bus.enc(1)
    assert c > 2**31 and 2**32 - c > 1000  # unsigned 32 bit: below 0 wraps
    assert st & 1 and st & 2  # underflow occurred, direction backwards
    c2, st2 = bus.enc(1)
    assert not st2 & 1 and st2 & 2  # the flag cleared after reading
    assert bus.speed(1) < -500  # the speed reply: magnitude + direction byte
    bus.write(22, struct.pack(">I", 1000))  # set encoder 1 = 1000
    assert abs(bus.enc(1)[0] - 1000) < 200  # (the wheel keeps turning)


def test_velocity_pid_and_speed_commands(bus, sim):
    bus.write(
        28, struct.pack(">4I", 0, 65536, 4 * 65536, 10600)
    )  # D 0, P 1, I 4, QPPS 10600
    bus.write(29, struct.pack(">4I", 0, 65536, 4 * 65536, 10600))
    assert struct.unpack(">4I", bus.read(55, 16)) == (
        65536,
        4 * 65536,
        0,
        10600,
    )  # reply order: P, I, D, QPPS
    bus.write(37, struct.pack(">ii", 2000, -2000))
    sim.step(2500)
    assert bus.speed(1) == pytest.approx(2000, rel=0.03)
    assert bus.speed(2) == pytest.approx(-2000, rel=0.03)
    # rolling at 2000 counts/s = 0.47 m/s: the wheel has turned about 5000 counts
    assert 3000 < bus.enc(1)[0] < 6000


def test_defaults_of_the_manual_are_pushed_at_start(bus):
    assert struct.unpack(">4I", bus.read(55, 16)) == (0x10000, 0x8000, 0x4000, 44000)
    assert struct.unpack(">4I", bus.read(56, 16)) == (0x10000, 0x8000, 0x4000, 44000)


def test_speed_with_acceleration_ramps(bus, sim):
    bus.write(28, struct.pack(">4I", 0, 65536, 4 * 65536, 10600))
    bus.write(
        40, struct.pack(">Iii", 4000, 2000, 2000)
    )  # 4000 counts/s^2 to 2000 counts/s: 0.5 s
    sim.step(250)
    assert 500 < bus.speed(1) < 1500  # half way, not there yet
    sim.step(1500)
    assert bus.speed(1) == pytest.approx(2000, rel=0.03)


def test_duty_with_acceleration_ramps(bus, sim):
    bus.write(
        54, struct.pack(">hIhI", 32767, 32767, 32767, 32767)
    )  # 100 % at 1 per second
    sim.step(500)
    assert bus.pwm()[0] == pytest.approx(16384, abs=3000)
    sim.step(900)
    assert bus.pwm()[0] == 32767


def test_status_flags_overcurrent_against_the_stored_limit(bus, sim):
    assert struct.unpack(">I", bus.read(90, 4)) == (0,)
    assert struct.unpack(">II", bus.read(135, 8)) == (0, 0)
    bus.write(133, struct.pack(">II", 1, 0))  # M1 limit 10 mA
    assert struct.unpack(">II", bus.read(135, 8)) == (1, 0)
    bus.write(34, struct.pack(">hh", 32767, 32767))
    sim.step(200)  # at full duty the start-up current is ~ 5 A
    assert struct.unpack(">I", bus.read(90, 4)) == (
        1,
    )  # M1 only (no limit stored for M2)


# ---- 3. the official basicmicro library -----------------------------------------------------------------------------------------
@pytest.fixture
def lib(sim, port):
    basicmicro = pytest.importorskip("basicmicro")
    pytest.importorskip("serial")
    assert sim.call({"cmd_type": "Run", "cmd": {"rate": 1.0}})["ok"]
    rc = basicmicro.Basicmicro(port.device_path, 38400)
    assert rc.Open()
    yield rc
    rc.close()
    sim.call({"cmd_type": "Pause", "cmd": {}})


def test_library_reads_and_pid_roundtrip(lib):
    ok, version = lib.ReadVersion(ADDR)
    assert ok and "Roboclaw" in version
    ok, v = lib.ReadMainBatteryVoltage(ADDR)
    assert ok and v == 123
    assert lib.SetM1VelocityPID(ADDR, 1.0, 4.0, 0.5, 10600)
    ok, p, i, d, qpps = lib.ReadM1VelocityPID(ADDR)
    assert (ok, p, i, d, qpps) == (True, 1.0, 4.0, 0.5, 10600)
    assert lib.ReadError(ADDR) == (True, 0)
    assert lib.ResetEncoders(ADDR)
    ok, c, st = lib.ReadEncM1(ADDR)
    assert ok and c == 0 and st == 0


def test_library_drives_the_rover(lib):
    assert lib.SetM1VelocityPID(ADDR, 1.0, 4.0, 0.0, 10600) and lib.SetM2VelocityPID(
        ADDR, 1.0, 4.0, 0.0, 10600
    )
    assert lib.ResetEncoders(ADDR)
    assert lib.SpeedAccelM1M2(ADDR, 6000, 2000, 2000)
    time.sleep(1.5)
    ok, s1, d1 = lib.ReadSpeedM1(ADDR)
    ok2, s2, d2 = lib.ReadSpeedM2(ADDR)
    assert (
        ok
        and ok2
        and (d1, d2) == (0, 0)
        and s1 == pytest.approx(2000, rel=0.05)
        and s2 == pytest.approx(2000, rel=0.05)
    )
    ok, c1, _ = lib.ReadEncM1(ADDR)
    assert ok and 1000 < c1 < 4000
    ok, a1, a2 = lib.ReadCurrents(ADDR)
    assert ok and 5 < a1 < 150 and 5 < a2 < 150  # 10 mA units: 0.05 .. 1.5 A
    assert lib.DutyM1M2(ADDR, 0, 0)
    time.sleep(0.5)
    ok, p1, p2 = lib.ReadPWMs(ADDR)
    assert ok and (p1, p2) == (0, 0)
