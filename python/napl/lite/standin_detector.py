"""Stand-in detector process: the fixed detector IPC (shm ring CLF1 + Unix socket JSON lines) without Ultralytics, for sim and CI.

    python -m napl.lite.standin_detector --sock PATH --frames /dev/shm/cuas-lite-frames --mode classical
    python -m napl.lite.standin_detector --sock PATH --frames FILE --mode fixed --box person:0.9:100,100,200,300

Modes: `classical` runs napl.cuas.tracking.detect (dark blobs against the sky) and reports every blob as a "drone"; `fixed` replies the given boxes
for every request (IPC and person-block tests). StandinDetectorProcess spawns it as a child (context manager)."""

import argparse
import json
import os
import socket
import subprocess
import sys
import time

from .detector_client import ShmReader


def classical_infer(rgb):
    from ..cuas.tracking import detect

    h = rgb.shape[0]
    return [
        dict(
            cls="drone",
            conf=0.9,
            xyxy=[
                float(b.bbox[0]),
                float(b.bbox[1]),
                float(b.bbox[2] + 1),
                float(b.bbox[3] + 1),
            ],
        )
        for b in detect(rgb, h)
    ]


def serve(sock, frames, infer, stop_after=None):
    """Accept one client at a time; per request line read the slot and answer one JSON line. Returns when the client disconnects after `stop_after`
    requests (None = serve forever)."""
    if os.path.exists(sock):
        os.unlink(sock)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock)
    srv.listen(1)
    ring, n = None, 0
    while True:
        conn, _ = srv.accept()
        f = conn.makefile("rwb")
        for line in f:
            try:
                req = json.loads(line)
                if ring is None:
                    ring = ShmReader(frames)
                t0 = time.monotonic()
                dets = infer(ring.slot(req["slot"]))
                rep = {
                    "seq": req["seq"],
                    "t_infer_ms": round((time.monotonic() - t0) * 1000, 3),
                    "dets": dets,
                }
            except (ValueError, KeyError, OSError):
                continue
            f.write((json.dumps(rep) + "\n").encode())
            f.flush()
            n += 1
        if stop_after is not None and n >= stop_after:
            return


class StandinDetectorProcess:
    def __init__(self, sock, frames, mode="classical", boxes=()):
        self.cmd = [
            sys.executable,
            "-m",
            "napl.lite.standin_detector",
            "--sock",
            sock,
            "--frames",
            frames,
            "--mode",
            mode,
        ]
        for b in boxes:
            self.cmd += ["--box", b]
        self.sock, self.proc = sock, None

    def __enter__(self):
        if os.path.exists(self.sock):
            os.unlink(self.sock)
        self.proc = subprocess.Popen(self.cmd, start_new_session=True)
        for _ in range(500):  # wait for the socket to appear
            if os.path.exists(self.sock):
                return self
            time.sleep(0.01)
        raise RuntimeError("stand-in detector did not come up")

    def __exit__(self, *a):
        self.proc.terminate()
        try:
            self.proc.wait(3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        if os.path.exists(self.sock):
            os.unlink(self.sock)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--mode", choices=("classical", "fixed"), default="classical")
    ap.add_argument(
        "--box", action="append", default=[], help="cls:conf:x0,y0,x1,y1 (fixed mode)"
    )
    a = ap.parse_args(argv)
    if a.mode == "fixed":
        fixed = [
            dict(cls=c, conf=float(cf), xyxy=[float(x) for x in xy.split(",")])
            for c, cf, xy in (b.split(":") for b in a.box)
        ]
        infer = lambda rgb: fixed
    else:
        infer = classical_infer
    serve(a.sock, a.frames, infer)


if __name__ == "__main__":
    main()
