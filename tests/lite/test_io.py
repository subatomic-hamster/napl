"""I/O implementations: shm ring, detector socket + stand-in process, web UI, and a short HIL smoke over a real pty."""

import json
import struct
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest
from napl.lite import protocol as p
from napl.lite.detector_client import ShmCamera, ShmReader, ShmWriter, SocketDetector
from napl.lite.io import Frame
from napl.lite.standin_detector import StandinDetectorProcess


class WallClock:
    def now(self):
        return time.monotonic()

    def sleep_until(self, t):
        time.sleep(max(0, t - time.monotonic()))


def test_shm_ring_layout_and_roundtrip(tmp_path):
    path = str(tmp_path / "ring")
    w = ShmWriter(path, 64, 48)
    img = np.arange(64 * 48 * 3, dtype=np.uint8).reshape(48, 64, 3)
    w.write(2, img, t_cap=1.25)
    raw = open(path, "rb").read()
    assert (
        raw[:4] == b"CLF1"
        and struct.unpack("<IIII", raw[4:20]) == (4, 64, 48, 192)
        and len(raw) == 64 + 4 * 192 * 48
    )
    assert raw[64 + 2 * 192 * 48 : 64 + 3 * 192 * 48] == img.tobytes()
    r = ShmReader(path)
    assert (
        (r.slots, r.w, r.h, r.stride) == (4, 64, 48, 192)
        and np.array_equal(r.slot(2), img)
        and r.header()[0] == 1
        and r.header()[2] == 1.25
    )


def sky(w=1280, h=720, rect=(300, 100, 16, 12)):
    img = np.tile(np.array([25, 31, 41], np.uint8), (h, w, 1))
    x, y, rw, rh = rect
    img[y : y + rh, x : x + rw] = (5, 5, 6)
    return img


def wait_reply(det, seq, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = det.poll()
        if r:
            return r
        time.sleep(0.005)
    raise AssertionError("no detector reply")


def test_detector_ipc_with_the_standin_process(tmp_path):
    sock, ring = str(tmp_path / "d.sock"), str(tmp_path / "frames")
    with StandinDetectorProcess(sock, ring, "classical"):
        det = SocketDetector(WallClock(), sock, ring)
        assert det.stale_s == float("inf")
        for _ in range(100):  # connect (the process is up once its socket exists)
            det.submit(Frame(1, 0.5, 1280, 720, sky()))
            if det.waiting:
                break
            time.sleep(0.01)
        r = wait_reply(det, 1)
        assert r.seq == 1 and len(r.dets) == 1 and r.dets[0].cls == "drone"
        x0, y0, x1, y1 = r.dets[0].xyxy
        assert (x0, y0, x1, y1) == (300, 100, 316, 112) and det.stale_s < 1.0
    time.sleep(0.1)
    det.poll()
    assert (
        det.stale_s == float("inf") or det.stale_s > 0
    )  # the process is gone: the socket is dropped on the next poll / submit


def test_fixed_standin_reports_a_person(tmp_path):
    sock, ring = str(tmp_path / "d.sock"), str(tmp_path / "frames")
    with StandinDetectorProcess(
        sock,
        ring,
        "fixed",
        boxes=["person:0.9:100,100,200,300", "balloon:0.8:10,10,30,30"],
    ):
        det = SocketDetector(WallClock(), sock, ring)
        for _ in range(100):
            det.submit(Frame(7, 0.1, 64, 48, np.zeros((48, 64, 3), np.uint8)))
            if det.waiting:
                break
            time.sleep(0.01)
        r = wait_reply(det, 7)
        assert [(d.cls, d.conf) for d in r.dets] == [("person", 0.9), ("balloon", 0.8)]


def test_detector_error_reply_is_stale_not_empty():
    """cuas-lite-detector answers a request it cannot serve with `"error"` and no dets: that must read as a stale detector (fail closed), never
    as a valid "nothing seen" reply."""
    import socket

    class Clock:
        t = 0.0

        def now(self):
            return self.t

    clock = Clock()
    det = SocketDetector(clock, "/nonexistent")
    a, b = socket.socketpair()
    a.setblocking(False)
    det.sock, det.last_reply_t, det.waiting = a, 0.0, (5, 0.0)
    clock.t = 0.3
    b.sendall(
        b'{"seq": 5, "t_infer_ms": -1, "dets": [], "error": "slot size mismatch"}\n'
    )
    time.sleep(0.01)
    assert det.poll() is None and det.waiting is None
    assert det.stale_s == pytest.approx(0.3)  # not refreshed by the error reply
    b.sendall(b'{"seq": 6, "t_infer_ms": 9, "dets": []}\n')
    time.sleep(0.01)
    assert det.poll().seq == 6 and det.stale_s == 0.0
    a.close()
    b.close()


def test_web_ui_token_commands_sse_and_mjpeg():
    from apprig import AppRig
    from napl.lite.web import serve

    r = AppRig()

    class Jpg:
        def jpeg(self):
            return b"\xff\xd8fakejpeg\xff\xd9"

    srv = serve(r.app, "127.0.0.1", 0, "tok", Jpg())
    assert srv.server_address[0] == "127.0.0.1"
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + "/")
        assert e.value.code == 403
        assert b"HOLD TO FIRE" in urllib.request.urlopen(base + "/?token=tok").read()
        post = lambda c, b=None, tok="tok": urllib.request.urlopen(
            urllib.request.Request(
                f"{base}/api/{c}?token={tok}",
                json.dumps(b or {}).encode(),
                method="POST",
            )
        )
        with pytest.raises(urllib.error.HTTPError) as e:
            post("arm", tok="bad")
        assert e.value.code == 403
        with pytest.raises(urllib.error.HTTPError) as e:
            post("rm_rf")
        assert e.value.code == 404
        assert post("deadman").status == 200 and r.app.deadman_t == 0.0
        post("sector", dict(az_min=-10, az_max=10, el_min=0, el_max=30, r_max=12))
        post("authorize", dict(id="A"))
        post("auth_mode", dict(per_track=True))
        post("range", dict(id="A", metres=9.0))
        r.run(0.02)  # commands run inside the control loop
        assert (
            r.app.fc.sector.az_max == 10
            and r.app.fc.cfg.per_track_auth
            and r.app.tracker.manual_range == {"A": 9.0}
            and "A" in r.app.fc.auth
        )
        with urllib.request.urlopen(base + "/events?token=tok") as ev:
            line = b""
            while not line.startswith(b"data: "):
                line = ev.readline()
            s = json.loads(line[6:])
        assert (
            s["stage"] == "SAFE"
            and s["sector"]["az_min"] == -10
            and "tracks" in s
            and s["frame"] == [1280, 720]
        )
        with urllib.request.urlopen(base + "/video.mjpg?token=tok") as v:
            assert "multipart/x-mixed-replace" in v.headers["Content-Type"]
            head = v.read(120)
            assert b"--cuasframe" in head and b"image/jpeg" in head
    finally:
        srv.shutdown()


def test_hil_smoke_over_a_real_pty(tmp_path):
    """Twin on a pty (wall time, thread), the App against the pty slave with ShmCamera + SocketDetector + the stand-in process: hello, IDLE, homing,
    ARM with a person in the frame -> no ball. Bounded: ~10 s."""

    from napl.lite.app import App, AppCfg
    from napl.lite.fire import Sector
    from napl.lite.gimbal import SerialGimbal, TtyLink
    from napl.lite.sim_mcu import KinematicPlant, PtyMcu, SimRP2040

    mcu = PtyMcu()
    plant = KinematicPlant()
    twin = SimRP2040(plant, mcu)
    stop = threading.Event()

    def twin_loop():
        twin.step(time.monotonic())
        while not stop.is_set():
            twin.step(time.monotonic())
            time.sleep(0.002)

    cam_path, ring, sock = (
        str(tmp_path / "simcam"),
        str(tmp_path / "frames"),
        str(tmp_path / "d.sock"),
    )
    cam_w = ShmWriter(cam_path, 640, 360)
    frame = sky(640, 360, (300, 100, 16, 12))

    def cam_loop():
        while not stop.is_set():
            cam_w.write(0, frame, time.monotonic())
            time.sleep(1 / 30)

    threads = [
        threading.Thread(target=twin_loop, daemon=True),
        threading.Thread(target=cam_loop, daemon=True),
    ]
    for t in threads:
        t.start()
    clock = WallClock()
    try:
        with StandinDetectorProcess(
            sock, ring, "fixed", boxes=["person:0.9:20,20,60,200"]
        ):
            gim = SerialGimbal(TtyLink(mcu.slave_path), clock, session=5)
            app = App(
                ShmCamera(cam_path),
                SocketDetector(clock, sock, ring),
                gim,
                clock,
                AppCfg(auto_home=True),
                sector=Sector(-60, 60, 0, 60),
            )
            t_end, armed_at = time.monotonic() + 14.0, None
            while time.monotonic() < t_end:
                now = clock.now()
                app.deadman()
                app.tick(now)
                st = gim.status
                if (
                    armed_at is None
                    and gim.status_t
                    and st.state == p.IDLE
                    and st.flag1(p.F1_PAN_HOMED)
                    and st.flag1(p.F1_TILT_HOMED)
                    and app.person_t > 0
                ):
                    app.post("arm")
                    armed_at = now
                if armed_at and now - armed_at > 2.5:
                    break
                time.sleep(0.003)
            assert p.handshake_ok(gim.ack) and armed_at, (
                gim.ack,
                p.STATE_NAMES[gim.status.state],
                list(app.log)[-6:],
            )
            assert twin.shots == [] and plant.fire_pulses == []
            assert "person" in app.dec.reasons
            assert twin.state in (p.IDLE, p.ARMED)
            assert gim.link_ok and not app.failed
    finally:
        stop.set()
        for t in threads:
            t.join(2)
        mcu.close()
        cam_w.close()
