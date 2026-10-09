"""The Skydio autonomy (fly.py: depth -> map -> planner, detector, search belief, ESKF VIO estimator) as a COMPANION COMPUTER to a real PX4 autopilot.

    napl program examples/skydio/follow.px4.scenario.toml --runs-dir <dir>      (and goal.px4.scenario.toml)

What is different from fly.py (everything not listed here is the same code, inherited from `fly.Skydio`):
  - The World belongs to napl-px4-bridge, stepped in lockstep with PX4 SITL at wall = sim time. This program has no physics access except what the bridge's
    ctl socket gives it, which is what a companion computer has: the IMU samples (`imu <since>`: every sample PX4 got), the barometer, the depth camera,
    and, for evaluation only, the kernel's true state (never fed to the estimator or planner).
  - The VIO position / velocity / attitude estimate (the ESKF of estimator.py, on the IMU + the statistical VIO model + the barometer) goes to PX4 as
    MAVLink ODOMETRY at 31 Hz; PX4's EKF2 fuses it (GPS disabled: EKF2_GPS_CTRL 0, EKF2_EV_CTRL position + velocity + yaw). PX4's own estimate drives its
    controller; this program also reads it back (telemetry) and logs how far it is from the companion's.
  - The depth map becomes OBSTACLE_DISTANCE (72 x 5 deg, body frame) at 25 Hz.
  - The planner's primitive goes to PX4 as OFFBOARD position + velocity + acceleration set-points (62.5 Hz); PX4's position / velocity / attitude / rate
    controllers and mixer fly the airframe, not `cuas.flyer.Flyer`. Arming, offboard mode and take-off are MAVLink commands.
  - The loop runs on SIM time (one cycle per 16 ms of ticks), so a slow cycle (planner) shows up as extra delay, as on a real companion computer: the
    wall-clock cost of the Python planner adds to the simulated LATENCY_S / VIO_LATENCY_S, which are kept.
Artefacts: as fly.py (telemetry.jsonl, depth.npz, summary.json, forest.json), but the trace is companion.csv (trace.csv is the bridge's), plus
px4_est.csv (PX4's own local-position estimate against the companion's). The replay log is the bridge's (replay.naplog) and, because PX4 is not
lockstep-deterministic in its timing, every run replays to its own hash (docs/guide.md, "Replay hash")."""

import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import fly as F
from estimator import baro_height
from fly import LATENCY_S, MAP_S, TAKEOFF_V, VIO_LATENCY_S, Skydio, quat_R  # noqa: F401
from napl import companion
from napl.ctl import RemoteSim

LOOP_TICKS = (
    16  # one companion cycle per 16 ms of sim time (62.5 Hz: set-points, trace rows)
)
ODOM_EVERY = 2  # ODOMETRY every 2nd cycle: 31 Hz
READY_S = (
    6.0  # estimator settle before arming, after PX4 reports a valid local position (s)
)
TAIL_S = 1.5  # keep holding the last position after the mission ends so the replay does not end in a fall (s)


def R_to_q(R):
    """Rotation matrix -> unit quaternion (w, x, y, z), w >= 0."""
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        q = [
            s / 4,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i], q[1 + j], q[1 + k] = (
            s / 4,
            (R[j, i] + R[i, j]) / s,
            (R[k, i] + R[i, k]) / s,
        )
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


class Px4Flyer:
    """The part of fly.EstFlyer's interface the autonomy reads, with no control law: PX4 is the controller. `last` = (estimate p, v, R, tilt, None) of the
    last cycle, `truth` = the kernel's state of the last read (evaluation only), `sp` = what the loop sends to PX4."""

    est = truth = last = None

    def __init__(self, sim, paths):
        self.sim, self.p = sim, paths
        self.sp = (np.zeros(3), np.zeros(3), np.zeros(3))
        self.yaw = 0.0

    def read_truth(self):
        q = lambda p, a: self.sim.query(p, a)[0]
        self.truth = (
            q(self.p["root"], "pos"),
            quat_R(q(self.p["root"], "quat")),
            q(self.p["free"], "qvel"),
        )
        return self.truth

    def state(self):
        t = self.read_truth()
        if self.est is None:
            return t
        e = self.est
        return e.p.copy(), e.R.copy(), np.concatenate([e.v, e.w])

    def target(self, pos, vel=(0.0, 0.0, 0.0), yaw=None, acc=(0.0, 0.0, 0.0)):
        self.sp = (
            np.asarray(pos, float),
            np.asarray(vel, float),
            np.asarray(acc, float),
        )
        if yaw is not None:
            self.yaw = float(yaw)


class Px4Skydio(Skydio):
    TRACE = "companion.csv"

    def __init__(self, model, run_dir, mission, seed, ctl, url):
        self._ctl, self.link = ctl, companion.Link(url)
        self._tick, self.tick_ref = 0, 0
        super().__init__(model, run_dir, mission, seed)
        self.phase = "boot"
        rate = json.loads((Path(model).parent / "scene.json").read_text())[
            "device_cfg"
        ]["skydio_depth"]["rate_hz"]
        self.depth_period = round(
            1.0 / (float(rate) * self.dt)
        )  # ticks between depth frames (the device samples at tick % period == 0)
        self.imu_c0 = self.imu_c = (
            None  # tick counter of the filter clock's zero / of the last IMU sample applied
        )
        self.last_pa, self.last_cycle = None, 0
        self.px4_est, self.armed_at = [], None
        self.t_zero_wall = time.monotonic()
        self.cyc = 0

    # ---- hooks of fly.Skydio ----------------------------------------------------------------------------------------------------
    def _open(self, model):
        return RemoteSim(self._ctl)  # the bridge logs; there is no local World

    def _flyer(self, paths, cal_dir):
        return Px4Flyer(self.sim, paths)

    def _seal(self, r):
        st = self.sim.tick
        (r / "companion.json").write_text(
            json.dumps(dict(tick=st, dt=self.dt))
        )  # the replay log and its hash are the bridge's

    @property
    def t(self):
        return (self._tick - self.tick_ref) * self.dt

    def _command(self, pp, vv, aa):
        self.flyer.target(pp, vv, self.yaw_sp, acc=aa)

    # ---- one cycle ------------------------------------------------------------------------------------------------------------------
    def _fetch_imu(self):
        """The bridge's IMU samples since the last cycle -> ESKF predictions; the filter clock is (sample tick - first tick) x dt."""
        tick, rec = self.sim.imu(self.imu_c if self.imu_c is not None else 0)
        self._tick = tick
        for c, *ga in rec:
            c = int(c)
            if self.imu_c is None:
                self.imu_c0 = self.imu_c = (
                    c - 4
                )  # the first record is one HIL step (4 ticks) long
            if not np.any(ga[3:]):  # the device reads 0 before its first sample
                self.imu_c = c
                continue
            dt = (c - self.imu_c) * self.dt
            gyro, accel = self.imu_bias.apply(ga[:3], ga[3:], dt)
            self.eskf.predict(gyro, accel, dt)
            self.imu_c = c
        return tick

    def _wait_cycle(self):
        """Block until LOOP_TICKS ticks have passed since the last cycle began (a cycle that ran long starts the next one at once, it does not catch up)."""
        while self._fetch_imu() < self.last_cycle + LOOP_TICKS:
            time.sleep(0.001)
        self.last_cycle = self._tick

    def _vio_baro(self):
        t = self.eskf.t
        while self.vio_pending and self.vio_pending[0][0] <= t + 1e-9:
            _, t_s, vb, Rm = self.vio_pending.popleft()
            self.eskf.update_vio(t_s, vb, Rm)
        pa = float(self.sim.query(self.baro_dev, "pressure")[0][0])
        if pa > 1e4 and pa != self.last_pa:  # a new sample of the 50 Hz barometer
            self.last_pa = pa
            z = baro_height(pa)
            if self.baro_ref is None:
                self.baro_ref = z - self.p_start[2]
            self.eskf.update_height(z - self.baro_ref)

    def _sense(self):
        n = int(self.sim.query(self.depth_dev, "frames")[0][0])
        if (
            n != self.frames_seen
        ):  # a new depth frame, sampled at the last multiple of its period
            self.frames_seen = n
            c = (self._tick - 1) // self.depth_period * self.depth_period
            t_f = (c - self.imu_c0) * self.dt  # filter clock of the sampling instant
            t_s = (c - self.tick_ref) * self.dt  # mission clock
            p, _, R = self.eskf._state_at(
                t_f
            )  # ESTIMATED pose at the sampling instant: the map and the detector use only this
            img = F.marine.depth_image(
                self.sim, self.depth_dev, self.cam.n_h, self.cam.n_v
            )
            cam_pos, cam_R = p + R @ self.cam_o_root, R @ self.cam_R_root
            pt, Rt, vt = (
                self.flyer.truth[0],
                self.flyer.truth[1],
                self.flyer.truth[2][:3],
            )  # truth of ~ the same instant: VIO input + detector scoring
            tcam = (pt + Rt @ self.cam_o_root, Rt @ self.cam_R_root)
            vb, Rm = self.vio.sample(max(t_s - self.vio_t, 1e-3), Rt, vt)
            self.vio_t = t_s
            self.vio_pending.append((self.eskf.t + VIO_LATENCY_S, t_f, vb, Rm))
            self.pending.append((self.t + LATENCY_S, img, cam_pos, cam_R, t_s, tcam))
            self._obstacles(img)
        while self.pending and self.pending[0][0] <= self.t + 1e-9:
            _, img, cam_pos, cam_R, t_s, tcam = self.pending.popleft()
            self._process(img, cam_pos, cam_R, t_s, tcam)

    def _obstacles(self, img):
        """The depth frame as OBSTACLE_DISTANCE (body frame): the horizontal range of the nearest return in each 5 deg bin, within the height band of the airframe."""
        ok = np.isfinite(img)
        pc = (np.where(ok, img, 0.0)[..., None] * self.cam.tangents())[
            ok
        ]  # sensor frame (x up, y right, z along the axis)
        pb = self.cam_o_root + pc @ self.cam_R_root.T  # root plate frame (FLU)
        self.link.obstacle_distance(
            self._tick * self.dt, pb, min_m=self.cam.min_range, max_m=self.cam.max_range
        )

    def _odometry(self):
        e = self.eskf
        s = e.sigmas()
        self.link.odometry(
            self._tick * self.dt, e.p - self.p_start, R_to_q(e.R), e.v, w_flu=e.w,
            sigma=(float(np.mean(s[0:3])), float(np.mean(s[3:6])), float(np.mean(s[6:8])), float(s[8])),
        )  # fmt: skip

    def _send_setpoint(self):
        p, v, a = self.flyer.sp
        yaw = (
            90.0 - math.degrees(self.flyer.yaw) + 180.0
        ) % 360.0 - 180.0  # ENU heading (CCW from East) -> NED heading (CW from North)
        self.link.setpoint(
            companion.enu_to_ned(p - self.p_start),
            companion.enu_to_ned(v),
            companion.enu_to_ned(a),
            yaw,
        )

    def _cycle(self, flying):
        self.flyer.read_truth()
        p, v, R = self.eskf.p.copy(), self.eskf.v.copy(), self.eskf.R.copy()
        tilt = math.acos(max(-1.0, min(1.0, R[2, 2])))
        self.flyer.last = (p, v, R, tilt, None)
        if flying:
            self._setpoint()
            if self.subject:
                self.subject.update()
        self._vio_baro()
        self._sense()
        if self.cyc % ODOM_EVERY == 0:
            self._odometry()
        if flying:
            self._send_setpoint()
            self._trace(*self.flyer.truth[:2], self.flyer.truth[2][:3], tilt)
            if self.cyc % 32 == 0:  # PX4's own estimate every 0.5 s
                n = self.link.tel.position_velocity_ned()
                self.px4_est.append((self.t, n.position.north_m, n.position.east_m, n.position.down_m, n.velocity.north_m_s, n.velocity.east_m_s, n.velocity.down_m_s,
                                     *companion.enu_to_ned(self.eskf.p - self.p_start), *companion.enu_to_ned(self.eskf.v)))  # fmt: skip
        self.cyc += 1

    # ---- the flight ------------------------------------------------------------------------------------------------------------------
    def _log(self, msg):
        print(  # ttsr-ignore: py/no-print (program.log is the progress log of a scenario program)
            f"[{time.monotonic() - self.t_zero_wall:6.1f}s wall, sim {self._tick * self.dt:6.1f}s] {msg}",
            flush=True,
        )

    def _boot(self, timeout_s=240.0):
        """Feed ODOMETRY (and take the depth map) until PX4's EKF2 has a valid local position from it, then let the estimator settle."""
        end, ok_since = time.monotonic() + timeout_s, None
        self.last_cycle = self._fetch_imu()
        while True:
            self._wait_cycle()
            self._cycle(False)
            if self.cyc % 16 == 0:
                h = self.link.tel.health()
                # is_armable is judged in the current nav mode (Hold), which needs a global position a GPS-denied vehicle never has; arming is done in
                # Offboard, which only needs the local position (ulog of the first run: cs_ev_pos/yaw/hgt/vel fused, "Ready for takeoff!" at 3.4 s)
                ready = h.is_local_position_ok
                ok_since = (ok_since or self._tick) if ready else None
                if self.cyc % 160 == 0:
                    self._log(
                        f"waiting for PX4: local_position_ok={h.is_local_position_ok}"
                    )
                if (
                    ok_since is not None
                    and (self._tick - ok_since) * self.dt >= READY_S
                ):
                    return
            if time.monotonic() > end:
                raise RuntimeError(
                    "PX4 never reported a valid local position from the vision odometry (see px4.log)"
                )

    def _takeoff(self):
        """Offboard on, arm; the mission clock starts here."""
        self.flyer.target(self.p_start, np.zeros(3), 0.0)

        def stream(
            n,
        ):  # PX4 wants a set-point stream before it accepts offboard, and keeps it while arming
            for _ in range(n):
                self._wait_cycle()
                self._cycle(False)
                self._send_setpoint()

        stream(20)
        for what, call in (
            ("offboard", self.link.off.start),
            ("arm", self.link.act.arm),
        ):
            for attempt in range(8):
                try:
                    call()
                    break
                except Exception as e:  # MAVSDK raises on a denied command (PX4 not ready yet): stream and retry
                    self._log(f"{what} refused ({attempt + 1}/8): {e}")
                    stream(62)
            else:
                raise RuntimeError(f"PX4 refused {what} 8 times (see px4.log)")
        self._log("offboard + armed")
        self.tick_ref = (
            self._tick
        )  # t = 0: take-off (fly.py's phases and the monitors count from here)
        self.pending.clear()
        self.phase, self.t_phase = "takeoff", 0.0

    def _process(self, img, cam_pos, cam_R, t_s, tcam):
        if (
            self.phase != "boot"
        ):  # on the pad, before take-off, the VIO runs (the filter needs it) but nothing is mapped or logged
            super()._process(img, cam_pos, cam_R, t_s, tcam)

    def run(self, t_max):
        arrived_at = None
        try:
            self._boot()
            self._takeoff()
            while self.t < t_max:
                self._wait_cycle()
                self._cycle(True)
                if self.phase == "arrived":
                    arrived_at = arrived_at or self.t
                    if self.t - arrived_at > 2.0:
                        break
                if self.subject and self.subject.s >= self.subject.length - 0.5:
                    break
                if self.crashed():
                    self._log("crashed (tilt / ground / contacts)")
                    break
            self.flyer.target(
                self.eskf.p, np.zeros(3), self.yaw_sp
            )  # hold, so the replay does not end in a fall
            end = self._tick + int(TAIL_S / self.dt)
            while self._tick < end:
                self._wait_cycle()
                self._cycle(False)
                self._send_setpoint()
            summ = self.finish()
        finally:
            self.link.destroy()
        return summ

    def finish(self):
        summ = super().finish()
        e = np.array(self.px4_est)
        if len(e):
            pos, vel = (
                np.linalg.norm(e[:, 1:4] - e[:, 7:10], axis=1),
                np.linalg.norm(e[:, 4:7] - e[:, 10:13], axis=1),
            )
            summ["px4_vs_companion"] = dict(
                pos_rms_m=float(np.sqrt((pos**2).mean())),
                pos_max_m=float(pos.max()),
                vel_rms_mps=float(np.sqrt((vel**2).mean())),
                samples=len(e),
            )
            (self.run_dir / "summary.json").write_text(json.dumps(summ, indent=1))
            with open(self.run_dir / "px4_est.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(
                    [
                        "t",
                        "px4_n",
                        "px4_e",
                        "px4_d",
                        "px4_vn",
                        "px4_ve",
                        "px4_vd",
                        "cmp_n",
                        "cmp_e",
                        "cmp_d",
                        "cmp_vn",
                        "cmp_ve",
                        "cmp_vd",
                    ]
                )
                w.writerows(e)
        return summ


def main():
    import skydio_scene as scene

    mission = os.environ.get("SKYDIO_MISSION", "goal")
    run = Px4Skydio(
        os.environ["NAPL_MODEL"],
        os.environ["NAPL_RUN_DIR"],
        mission,
        scene.seed(mission),
        os.environ["NAPL_CTL"],
        os.environ["NAPL_MAVLINK"],
    )
    s = run.run(float(os.environ.get("SKYDIO_T_MAX", F.T_MAX[mission])))
    sys.stdout.write(json.dumps(s) + "\n")


if __name__ == "__main__":
    main()
