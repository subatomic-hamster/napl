"""Tiny Feetech STS client written from the protocol spec (`FF FF ID LEN INSTR PARAMS.. CHK`, CHK = ~sum(ID..PARAMS) & 0xFF,
little-endian words, goal/position words sign-magnitude with bit 15 = negative), plus the SO-101 bring-up (LeRobot's configure).
Plain os / select / tty: no napl import, it talks to any STS3215 bus (the real one or the napl pty)."""

import os
import select
import time
import tty
from math import pi

CPR = 4096  # encoder counts per revolution
P_REG, I_REG, D_REG = 16, 0, 32  # LeRobot SO-101 configure
GRIP_TORQUE, GRIP_PROT_CURRENT, GRIP_OVERLOAD = 500, 250, 25  # gripper only (LeRobot)
CURRENT_LSB = 0.0065  # A per count of the present-current register


class Bus:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        self.buf = bytearray()
        self.errs = []  # nonzero error bytes of status packets: (id, error)
        self.bad = 0

    def close(self):
        os.close(self.fd)

    def send(self, sid, instr, params=b""):
        body = bytes([sid, len(params) + 2, instr]) + bytes(params)
        os.write(self.fd, b"\xff\xff" + body + bytes([~sum(body) & 0xFF]))

    def recv(self, n, timeout=1.0):
        """n status packets -> [(id, error, params)] (fewer if the bus goes quiet)."""
        out, end = [], time.monotonic() + timeout
        while len(out) < n and time.monotonic() < end:
            if select.select([self.fd], [], [], 0.005)[0]:
                self.buf += os.read(self.fd, 512)
            while len(self.buf) >= 4:
                if self.buf[:2] != b"\xff\xff":
                    del self.buf[0]
                    continue
                total = self.buf[3] + 4
                if len(self.buf) < total:
                    break
                p = bytes(self.buf[:total])
                del self.buf[:total]
                if ~sum(p[2:-1]) & 0xFF != p[-1]:
                    self.bad += 1
                    continue
                out.append((p[2], p[4], p[5:-1]))
                if p[4]:
                    self.errs.append((p[2], p[4]))
        return out

    def write(self, sid, addr, value, n=1):
        self.send(sid, 3, bytes([addr]) + int(value).to_bytes(n, "little"))
        (r,) = self.recv(1)
        assert r[:2] == (sid, 0), r

    def sync_write(self, addr, items):
        """items {id: bytes}: one broadcast SYNC_WRITE, no reply."""
        n = len(next(iter(items.values())))
        self.send(
            0xFE,
            0x83,
            bytes([addr, n]) + b"".join(bytes([i]) + d for i, d in items.items()),
        )

    def sync_read(self, addr, n, ids):
        self.send(0xFE, 0x82, bytes([addr, n, *ids]))
        r = self.recv(len(ids))
        assert [x[0] for x in r] == list(ids), "sync read incomplete"
        return {i: d for i, _, d in r}


def goal_bytes(counts):
    """Goal position word: sign-magnitude, bit 15 = negative (the sim's zero pose is count 0, joints cross it)."""
    v = int(round(counts))
    return ((-v) | 0x8000 if v < 0 else v).to_bytes(2, "little")


def unwrap(pos):
    return (
        pos - CPR if pos >= CPR // 2 else pos
    )  # present position is wrapped to 0..4095


def configure(bus, ids=range(1, 7)):
    """RESET (factory defaults: overload 80 % / 2 s, unload mask, voltage and temperature limits; the so101 build itself leaves
    every protection off), then LeRobot's SO-101 configure step over the bus, plus open angle limits (multi-turn style:
    min = max = 0) so that goals on both sides of the zero pose are not clamped."""
    for i in ids:
        bus.send(i, 6)  # RESET
        assert bus.recv(1)[0][:2] == (i, 0)
        bus.write(i, 55, 0)  # unlock EEPROM
        bus.write(i, 7, 0)  # return delay
        bus.write(i, 41, 254)  # acceleration (stored)
        bus.write(i, 33, 0)  # operating mode: position
        bus.write(i, 9, 0, 2)
        bus.write(i, 11, 0, 2)
        bus.write(i, 21, P_REG)
        bus.write(i, 23, I_REG)
        bus.write(i, 22, D_REG)
        if i == 6:
            bus.write(i, 16, GRIP_TORQUE, 2)  # max torque limit
            bus.write(i, 28, GRIP_PROT_CURRENT, 2)  # protection current (stored)
            bus.write(i, 36, GRIP_OVERLOAD)  # overload torque, % of full duty
        bus.write(i, 40, 1)  # torque on: holds the present position


def decode(d):
    """The 15 bytes from address 56 -> dict of present values."""
    w = lambda k: d[k] | d[k + 1] << 8
    load = w(4)
    return {
        "q": unwrap(w(0)) * 2 * pi / CPR,
        "load": (-(load & ~0x400) if load & 0x400 else load) / 1000,
        "status": d[9],
        "current": w(13) * CURRENT_LSB,
    }
