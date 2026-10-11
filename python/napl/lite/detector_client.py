"""The detector IPC (briefing + spec section 1 clarifications), app side, and the CLF1 shm ring shared with the camera side.

Ring: header = five little-endian u32 (b"CLF1", slots, width, height, stride bytes) zero-padded to 64 bytes; slot k at 64 + k * stride * height; RGB24,
stride = 3 * width, 4 slots. Our additive use of the padding (the real detector ignores it): u32 `counter` at 20 (frames written), u32 `latest` at 24
(slot of the newest), f64 `t_cap` at 32, used by ShmCamera (HIL sim camera). The app sends a request only when the previous reply has arrived or 100 ms
passed; a slot is not rewritten for 3 further frames. A missing reply for 250 ms is stale (fail closed, spec 3.2 row 1).

    det = SocketDetector(clock)            # app side: writes the frame into the ring, one JSON line per request, never blocks
    ShmWriter / ShmCamera                  # the HIL sim camera ring (harness writes, app reads)"""

import json
import mmap
import os
import socket
import struct
from collections import deque

import numpy as np

from .io import Det, DetResult, Frame, Intrinsics

MAGIC = b"CLF1"
HEADER = 64
SLOTS = 4
FRAMES_PATH = "/dev/shm/cuas-lite-frames"
SIMCAM_PATH = "/dev/shm/cuas-lite-simcam"


def sock_path():
    return os.path.join(
        os.environ.get("XDG_RUNTIME_DIR") or "/tmp", "cuas-lite-det.sock"
    )


class ShmWriter:
    def __init__(self, path, w, h, slots=SLOTS):
        self.path, self.w, self.h, self.slots, self.stride = path, w, h, slots, 3 * w
        self.size = HEADER + slots * self.stride * h
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_TRUNC, 0o600)
        os.ftruncate(fd, self.size)
        self.mm = mmap.mmap(fd, self.size)
        os.close(fd)
        self.mm[:20] = MAGIC + struct.pack("<IIII", slots, w, h, self.stride)
        self.counter = 0

    def write(self, slot, rgb, t_cap=0.0):
        off = HEADER + slot * self.stride * self.h
        self.mm[off : off + self.stride * self.h] = np.ascontiguousarray(
            rgb, np.uint8
        ).tobytes()
        self.counter += 1
        self.mm[20:40] = struct.pack("<IIxxxxd", self.counter, slot, t_cap)

    def close(self):
        self.mm.close()

    def unlink(self):
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass


class ShmReader:
    def __init__(self, path):
        fd = os.open(path, os.O_RDONLY)
        self.mm = mmap.mmap(fd, 0, prot=mmap.PROT_READ)
        os.close(fd)
        if bytes(self.mm[:4]) != MAGIC:
            raise ValueError(f"{path}: not a CLF1 ring")
        self.slots, self.w, self.h, self.stride = struct.unpack("<IIII", self.mm[4:20])

    def header(self):
        counter, latest = struct.unpack("<II", self.mm[20:28])
        return counter, latest, struct.unpack("<d", self.mm[32:40])[0]

    def slot(self, k):
        off = HEADER + k * self.stride * self.h
        return (
            np.frombuffer(self.mm[off : off + self.stride * self.h], np.uint8)
            .reshape(self.h, self.w, 3)
            .copy()
        )

    def close(self):
        self.mm.close()


class ShmCamera:
    """HIL sim camera: the newest frame in a CLF1 file the harness writes (waits up to `timeout` for the file and for a new frame)."""

    def __init__(self, path=SIMCAM_PATH, fov_v=0.911):
        self.path, self.r, self.last, self.fov_v = path, None, 0, fov_v
        self.intrinsics = Intrinsics.from_fov(1280, 720, fov_v)

    def read(self, timeout=0.1):
        import time

        end = time.monotonic() + timeout
        while True:
            if self.r is None and os.path.exists(self.path):
                self.r = ShmReader(self.path)
                self.intrinsics = Intrinsics.from_fov(self.r.w, self.r.h, self.fov_v)
            if self.r is not None:
                n, k, t = self.r.header()
                if n != self.last:
                    self.last = n
                    return Frame(n, t, self.r.w, self.r.h, self.r.slot(k))
            if time.monotonic() >= end:
                return None
            time.sleep(0.002)

    def close(self):
        if self.r:
            self.r.close()


class SocketDetector:
    def __init__(self, clock, path=None, frames_path=FRAMES_PATH, retry_s=1.0):
        self.clock, self.path, self.frames_path, self.retry_s = (
            clock,
            path or sock_path(),
            frames_path,
            retry_s,
        )
        self.sock = None
        self.buf = b""
        self.ring = None
        self.slot = 0
        self.waiting = None  # (seq, t_sent)
        self.last_reply_t = None
        self.t_try = -1e9
        self.out = deque()

    @property
    def connected(self):
        return self.sock is not None

    @property
    def stale_s(self):
        if self.sock is None or self.last_reply_t is None:
            return float("inf")
        return self.clock.now() - self.last_reply_t

    def _connect(self, now):
        if now - self.t_try < self.retry_s:
            return
        self.t_try = now
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self.path)
            s.setblocking(False)
        except OSError:
            s.close()
            return
        self.sock, self.buf, self.waiting = s, b"", None
        self.last_reply_t = now  # the detector gets 250 ms from the connection to answer the first request

    def _drop(self):
        if self.sock:
            self.sock.close()
        self.sock, self.waiting = None, None

    def submit(self, frame):
        now = self.clock.now()
        if self.sock is None:
            self._connect(now)
        if self.sock is None or frame.rgb is None:
            return
        if self.waiting and now - self.waiting[1] < 0.1:
            return  # latest frame wins: the previous request is still out
        if self.ring is None or (self.ring.w, self.ring.h) != (frame.w, frame.h):
            if self.ring:
                self.ring.close()
            self.ring = ShmWriter(self.frames_path, frame.w, frame.h)
        self.slot = (self.slot + 1) % self.ring.slots
        self.ring.write(self.slot, frame.rgb, frame.t_cap)
        line = (
            json.dumps(
                {
                    "seq": frame.seq,
                    "slot": self.slot,
                    "t_cap": frame.t_cap,
                    "w": frame.w,
                    "h": frame.h,
                }
            )
            + "\n"
        )
        try:
            self.sock.sendall(line.encode())
            self.waiting = (frame.seq, now)
        except (BlockingIOError, OSError):
            self._drop()

    def poll(self):
        if self.sock is None:
            return None
        try:
            data = self.sock.recv(65536)
            if not data:
                self._drop()
                return None
            self.buf += data
        except BlockingIOError:
            pass
        except OSError:
            self._drop()
            return None
        best = None
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                m = json.loads(line)
                best = DetResult(int(m["seq"]), float(m.get("t_infer_ms", 0.0)),
                                 [Det(d["cls"], float(d["conf"]), tuple(float(x) for x in d["xyxy"])) for d in m.get("dets", [])], self.clock.now())  # fmt: skip
            except (ValueError, KeyError, TypeError):
                continue  # a malformed line is a dropped reply
            self.last_reply_t = self.clock.now()
            if self.waiting and m["seq"] >= self.waiting[0]:
                self.waiting = None
        return best
