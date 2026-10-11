"""The cuas-lite control app: App.tick(now) is one run-to-completion step (poll camera, detector, gimbal; decide; send) shared by the real runner,
HIL and the lockstep sim. App code never touches time / os / sockets: those live in the I/O implementations passed in (spec section 4).

    app = App(camera, detector, gimbal, clock, AppCfg(), geom=Geometry(), sector=Sector(-60, 60, 0, 60))
    while True: app.tick(clock.now())              # real runner (main() below); the lockstep harness calls it once per 5 ms control period

Operator commands (web thread or harness) go through app.post(cmd, **kw) and run inside tick(): the control loop is the only writer of fire-control
state and the only sender of AIM / FIRE / HEARTBEAT. Any exception inside tick() disarms, stops the heartbeats and latches `app.failed` (fail closed)."""

import math
from collections import deque
from dataclasses import dataclass, field

from . import aim as aimlib
from . import protocol as p
from .fire import Decision, FireCfg, FireControl, Sector, View


@dataclass
class AppCfg:
    fire: FireCfg = field(default_factory=FireCfg)
    auto_home: bool = (
        False  # the real UI has a Home button; the sim harness turns this on
    )
    hello_period: float = 0.5
    hb_period: float = 0.05
    aim_period: float = 0.05
    fw_refresh: float = 0.1
    calib_ok: bool = True  # hardware: calib.toml exists and is newer than the camera mode (the runner sets it)
    wind: tuple = (0.0, 0.0, 0.0)
    pose_max_age: float = 0.2  # s: no STATUS this close to a frame's t_cap -> the frame is not used for tracking


class App:
    def __init__(
        self, camera, detector, gimbal, clock, cfg=None, geom=None, sector=None
    ):
        self.cfg = cfg or AppCfg()
        self.cam, self.det, self.gim, self.clock = camera, detector, gimbal, clock
        self.geom = geom or aimlib.Geometry()
        k = camera.intrinsics
        self.tracker = aimlib.ClassTracker(k)
        self.fc = FireControl(self.cfg.fire, self.geom, sector, f_px=k.fy)
        self.failed = None
        self.tracks, self.persons = [], []
        self.person_t = -math.inf
        self.deadman_t = -math.inf
        self.tracker_t = 0.0
        self.last_frame_t = -math.inf
        self.inflight = {}  # seq -> Frame (t_cap)
        self.dec = Decision()
        self.cmds = deque()
        self.t_hello = -math.inf
        self.t_hb = -math.inf
        self.t_aim = -math.inf
        self.t_fw = -math.inf
        self.last_aim = None
        self.fw_sent = 0.0
        self.cfg_requested = False
        self.t_sector = -math.inf
        self.home_sent = -math.inf
        self.estop_ui = False
        self.last_refusal = None
        self.log = deque(maxlen=200)  # (t, text): state changes, refusals, shots
        self.arm_pending_t = None
        self._dec_t = -math.inf
        self.n_ticks = 0
        self.mismatch = False

    # ---- operator interface (thread-safe: appends to a deque) ----------------------------------------------------------------------------
    def post(self, cmd, **kw):
        self.cmds.append((cmd, kw))

    def deadman(self):
        """The web UI posts this every 200 ms while 'hold to fire' is held."""
        self.deadman_t = self.clock.now()

    def _do(self, now, cmd, kw):
        fc, g = self.fc, self.gim
        if cmd == "arm":
            if (
                self.failed
                or not self.cfg.calib_ok
                or not g.link_ok
                or not self.sector_synced()
            ):
                self.last_refusal = "arm refused: " + (
                    self.failed
                    or (
                        "no calibration"
                        if not self.cfg.calib_ok
                        else "no link"
                        if not g.link_ok
                        else "fire sector not set on the MCU"
                    )
                )
                return self._note(now, self.last_refusal)
            self.estop_ui = False
            g.arm(fc.cfg.shot_cap, fc.cfg.arm_timeout_s)
            fc.arm(now)
            self.arm_pending_t = now
        elif cmd in ("disarm", "estop_ui"):
            self.estop_ui = cmd == "estop_ui"
            self._disarm()
        elif cmd == "authorize":
            fc.authorize(kw["id"], now)
        elif cmd == "auth_mode":
            fc.cfg.per_track_auth = bool(kw["per_track"])
        elif cmd == "sector":
            fc.sector = Sector(
                *(float(kw[k]) for k in ("az_min", "az_max", "el_min", "el_max")),
                kw.get("r_max"),
            )
        elif cmd == "home":
            g.home()
        elif cmd == "range":
            self.tracker.set_range(kw["id"], kw.get("metres"))
        elif cmd == "deadman":
            self.deadman_t = now

    def _sector_cfg(self):
        """The MCU's fire-sector config (spec 2.2, keys 17 and 20-22, mdeg) for the app's sector, or None without one."""
        s = self.fc.sector
        if s is None:
            return None
        return {
            p.CFG_FIRE_PAN_MIN: round(s.az_min * 1000),
            p.CFG_FIRE_PAN_MAX: round(s.az_max * 1000),
            p.CFG_FIRE_TILT_MIN: round(s.el_min * 1000),
            p.CFG_FIRE_TILT_MAX: round(s.el_max * 1000),
        }

    def sector_synced(self):
        """The MCU reports (CFG_VAL) exactly the app's sector: the second, independent layer of the geofence (the MCU refuses FIRE outside it and
        aborts a pusher stroke that leaves it). ARM needs it."""
        want = self._sector_cfg()
        return want is not None and all(
            self.gim.cfg.get(k) == v for k, v in want.items()
        )

    def _sync_sector(self, now):
        """In SAFE / IDLE (the only states that accept CFG_SET): write the sector to the MCU and read it back; retried every 0.5 s until it matches."""
        g, want = self.gim, self._sector_cfg()
        if (
            want is None
            or not g.link_ok
            or self.mismatch
            or g.status.state not in (p.SAFE, p.IDLE)
            or self.sector_synced()
            or now - self.t_sector < 0.5
        ):
            return
        self.t_sector = now
        for k, v in want.items():
            if g.cfg.get(k) != v:
                g.set_cfg(k, v)
            g._send(p.CfgGet(k))

    def _disarm(self):
        self.gim.disarm()
        self.gim.flywheel(0.0)
        self.fw_sent = 0.0
        self.fc.disarm()

    def _note(self, now, text):
        self.log.append((now, text))

    # ---- one step ------------------------------------------------------------------------------------------------------------------------
    def tick(self, now):
        if self.failed:
            return
        try:
            self._tick(now)
        except Exception as e:  # fail closed: disarm, stop the heartbeats (the MCU watchdog trips), stay failed
            self.failed = f"{type(e).__name__}: {e}"
            self._note(now, "FAILED " + self.failed)
            try:
                self._disarm()
            except Exception:  # noqa: BLE001
                pass

    def _tick(self, now):
        self.n_ticks += 1
        g, fc, c = self.gim, self.fc, self.cfg
        while self.cmds:
            self._do(now, *self.cmds.popleft())
        g.pump()
        self._mcu_events(now)
        # link bring-up: HELLO until acked, again whenever STATUS stops
        if (g.ack is None or not g.link_ok) and now - self.t_hello >= c.hello_period:
            self.t_hello = now
            g.hello()
        if g.ack is not None and not self.cfg_requested:
            self.cfg_requested = True
            self.mismatch = not p.handshake_ok(g.ack)
            g.request_config()
        # perception
        fr = self.cam.read(0.0)
        if fr is not None:
            self.last_frame_t = now
            self.inflight[fr.seq] = fr
            if len(self.inflight) > 16:
                self.inflight.pop(min(self.inflight))
            self.det.submit(fr)
        res = self.det.poll()
        new_dec = False
        if res is not None:
            f = self.inflight.pop(res.seq, None)
            if f is not None:
                new_dec = self._perceive(now, f, res)
        self._sync_sector(now)
        # home on request (auto) once the MCU is IDLE
        st = g.status
        if (
            c.auto_home
            and g.link_ok
            and st.state == p.IDLE
            and not (st.flag1(p.F1_PAN_HOMED) and st.flag1(p.F1_TILT_HOMED))
            and now - self.home_sent > 20.0
        ):
            self.home_sent = now
            g.home()
        # decide
        if new_dec or (now - self._dec_t >= c.aim_period):
            self._decide(now)
        # heartbeat (from this loop, never a helper thread)
        if not self.mismatch and now - self.t_hb >= c.hb_period:
            self.t_hb = now
            g.heartbeat(self._deadman_ok(now), self._inhibit(now))

    def _deadman_ok(self, now):
        return now - self.deadman_t < self.fc.cfg.deadman_s

    def _mcu_events(self, now):
        g, fc = self.gim, self.fc
        while g.events:
            t, ev = g.events.popleft()
            if ev.code == p.EV_SHOT:
                fc.on_shot(now)
                self._note(now, f"shot {fc.shots}/{fc.cfg.shot_cap}")
            elif ev.code == p.EV_SECTOR_ABORT:
                fc.on_sector_abort()
                self._note(
                    now, f"MCU aborted a ball: barrel left the sector ({ev.arg})"
                )
            elif ev.code == p.EV_STATE_CHANGE:
                self._note(
                    now, f"MCU -> {p.STATE_NAMES[ev.arg & 0xFF]} (reason {ev.arg >> 8})"
                )
            elif ev.code in (p.EV_WATCHDOG, p.EV_FAULT, p.EV_ESTOP_EDGE, p.EV_KEY_EDGE):
                self._note(now, f"MCU event {ev.code} arg {ev.arg}")
        while g.acks:
            t, a = g.acks.popleft()
            if a.result != p.OK:
                self.last_refusal = f"{p.MESSAGES[a.cmd_id].__name__} refused: {p.RESULT_NAMES[a.result]}"
                self._note(now, self.last_refusal)
                if a.cmd_id == p.FIRE:
                    fc.on_fire_ack(a)
                if a.cmd_id == p.ARM:
                    fc.disarm()

    def _perceive(self, now, f, res):
        """Detector reply -> tracker. Returns True when a fresh decision should be taken."""
        g = self.gim
        pt = g.status_at(f.t_cap)
        dets = res.dets
        pc = self.fc.cfg.person_conf
        self.persons = [(d.conf, tuple(d.xyxy)) for d in dets if d.cls == "person"]
        if any(cf >= pc for cf, _ in self.persons):
            self.person_t = now
        if pt is None or g.status_t is None or now - g.status_t > self.cfg.pose_max_age:
            return False  # no head attitude for this frame: cannot place the boxes in B (the person block above still holds)
        pose = self.geom.pose(*pt)
        blobs = [
            aimlib.DetBlob.from_det(d)
            for d in dets
            if d.cls in aimlib.TARGET_CLASSES and d.conf >= 0.3
        ]
        self.tracks = self.tracker.update(blobs, self.tracker.frame(pose, f.t_cap))
        self.tracker_t = f.t_cap
        return True

    def _view(self, now):
        g = self.gim
        pan, tilt = g.deg() if g.status_t is not None else (0.0, 0.0)
        return View(now, g.status if g.status_t is not None else None, g.link_ok, self.det.stale_s, now - self.last_frame_t, self.persons, self.person_t,
                    self.tracks, self.tracker_t, pan, tilt, self.deadman_t, self.cfg.wind, self.cfg.calib_ok)  # fmt: skip

    def _inhibit(self, now):
        fc = self.fc
        return (
            bool(fc.hard_veto(self._view(now))) or not fc.pending_live(now) or fc.cancel
        )

    def _decide(self, now):
        g, fc = self.gim, self.fc
        self._dec_t = now
        d = fc.step(now, self._view(now))
        self.dec = d
        if d.disarm:
            self._disarm()
            self._note(now, "disarmed: " + ",".join(d.reasons))
            return
        if d.aim is not None:
            pan, tilt, vp, vt = d.aim
            if (
                g.status_t is None
                or d.target is not None
                or self.last_aim is None
                or now - self.t_aim >= self.cfg.aim_period
            ):
                g.aim(pan, tilt, vp, vt, fc.cfg.vmax, fc.cfg.amax)
                self.last_aim, self.t_aim = d.aim, now
        # flywheel: on change, refreshed while non-zero
        if d.flywheel != self.fw_sent or (
            d.flywheel > 0 and now - self.t_fw >= self.cfg.fw_refresh
        ):
            g.flywheel(d.flywheel)
            self.fw_sent, self.t_fw = d.flywheel, now
        if d.fire:
            fc.fire_sent(now, d.fire)  # clears the inhibit for the in-flight shot ...
            g.heartbeat(
                True, False
            )  # ... and the MCU gets that flag immediately before the FIRE (ordered link)
            self.t_hb = now
            g.fire(d.fire)
        elif fc.cancel:  # a gate failed with a burst in flight: FIRE_INHIBIT now, not at the next heartbeat
            g.heartbeat(self._deadman_ok(now), True)
            self.t_hb = now

    # ---- state for the UI ----------------------------------------------------------------------------------------------------------------
    def snapshot(self):
        g, fc, d, now = self.gim, self.fc, self.dec, self.clock.now()
        st = g.status
        ok = g.status_t is not None
        tracks = []
        for t in self.tracks:
            if t.misses > 3:
                continue
            why = fc.engageable(t, self._view(now))
            tracks.append(dict(id=t.id, cls=getattr(t, "cls", "?"), conf=round(float(getattr(t, "conf", 0)), 2), bbox=[int(x) for x in t.bbox],
                               range=round(float(t.range_m), 2), manual_range=getattr(t, "manual_range", None), confirmed=bool(t.confirmed),
                               engageable=not why, why=why, authorized=bool(fc.auth.get(t.id) and fc.auth[t.id][0] > now),
                               selected=t.id == fc.target_id))  # fmt: skip
        return dict(
            frame=[self.cam.intrinsics.w, self.cam.intrinsics.h],
            t=round(now, 3), stage=d.stage, armed=fc.armed, failed=self.failed, link=g.link_ok, mismatch=self.mismatch,
            mcu_state=p.STATE_NAMES[st.state] if ok else "NO LINK", estop=bool(ok and st.flag1(p.F1_ESTOP)), key=bool(ok and st.flag1(p.F1_KEY)),
            homed=[bool(ok and st.flag1(p.F1_PAN_HOMED)), bool(ok and st.flag1(p.F1_TILT_HOMED))], pan_tilt=[round(x, 2) for x in g.deg()] if ok else None,
            shots=fc.shots, shot_cap=fc.cfg.shot_cap, arm_left_s=int(st.arm_left_s) if ok else 0, fault_bits=int(st.fault_bits) if ok else 0,
            deadman=self._deadman_ok(now), inhibit=self._inhibit(now), person_block=now - self.person_t < fc.cfg.person_hold_s,
            det_stale_s=None if math.isinf(self.det.stale_s) else round(self.det.stale_s, 3), reasons=d.reasons, err_deg=d.err_deg,
            per_track_auth=fc.cfg.per_track_auth, sector=fc.sector.asdict() if fc.sector else None, tracks=tracks, last_refusal=self.last_refusal,
            calib_ok=self.cfg.calib_ok, log=[(round(t, 2), s) for t, s in list(self.log)[-12:]],
        )  # fmt: skip


# ---- the real runner -------------------------------------------------------------------------------------------------------------------------
class MonotonicClock:
    def now(self):
        import time

        return time.monotonic()

    def sleep_until(self, t):
        import time

        time.sleep(max(0.0, t - time.monotonic()))


def build_camera(spec):
    """'v4l2:/dev/video0[:WxH@fps]' | 'shm:/dev/shm/cuas-lite-simcam'."""
    kind, _, rest = spec.partition(":")
    if kind == "v4l2":
        from .camera_v4l2 import V4L2Camera

        dev, _, mode = rest.partition(":")
        w, h, fps = (1280, 720, 60)
        if mode:
            wh, _, f = mode.partition("@")
            w, h = (int(x) for x in wh.split("x"))
            fps = int(f or 60)
        return V4L2Camera(dev, w, h, fps)
    if kind == "shm":
        from .detector_client import ShmCamera

        return ShmCamera(rest)
    raise SystemExit(f"unknown camera spec {spec!r}")


def main(argv=None):
    import argparse
    import secrets
    import threading
    import time

    from .detector_client import SocketDetector
    from .fire import load_sector
    from .gimbal import SerialGimbal, TtyLink

    ap = argparse.ArgumentParser(
        prog="python -m napl.lite.app",
        description="cuas-lite control app (spec docs/cuas-lite/spec.md)",
    )
    ap.add_argument(
        "--camera",
        required=True,
        help="v4l2:/dev/video0[:1280x720@60] or shm:/dev/shm/cuas-lite-simcam",
    )
    ap.add_argument(
        "--gimbal",
        required=True,
        help="serial device (CDC port, or a pty slave in HIL)",
    )
    ap.add_argument(
        "--web",
        default="127.0.0.1:8080",
        help="host:port of the operator UI (localhost by default)",
    )
    ap.add_argument(
        "--lan",
        action="store_true",
        help="bind the UI to 0.0.0.0 (token still required)",
    )
    ap.add_argument("--token", default=None, help="UI token (default: random, printed)")
    ap.add_argument("--auto-home", action="store_true")
    ap.add_argument(
        "--no-calib",
        action="store_true",
        help="mark calib.toml as missing (the app then refuses ARM)",
    )
    ap.add_argument(
        "--per-track-auth",
        action="store_true",
        help="stricter mode: the operator authorizes each track",
    )
    a = ap.parse_args(argv)
    from .web import serve

    clock = MonotonicClock()
    cam = build_camera(a.camera)
    cfg = AppCfg(
        auto_home=a.auto_home,
        calib_ok=getattr(cam, "calib_ok", True) and not a.no_calib,
    )
    cfg.fire.per_track_auth = a.per_track_auth
    app = App(
        cam,
        SocketDetector(clock),
        SerialGimbal(TtyLink(a.gimbal), clock),
        clock,
        cfg,
        sector=load_sector(),
    )
    host, _, port = a.web.rpartition(":")
    token = a.token or secrets.token_urlsafe(8)
    srv = serve(
        app, "0.0.0.0" if a.lan else (host or "127.0.0.1"), int(port), token, cam
    )
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(
        f"cuas-lite UI: http://{srv.server_address[0]}:{srv.server_address[1]}/?token={token}",
        flush=True,
    )  # ttsr-ignore: py/no-print (CLI)
    try:
        while True:
            app.tick(clock.now())
            time.sleep(0.002)
    finally:
        app.post("disarm")
        app.tick(clock.now())
        srv.shutdown()


if __name__ == "__main__":
    main()
