"""The Skydio-style autonomy program: depth image -> point cloud -> rolling voxel map -> sampled-primitive planner -> acceleration-feedforward setpoints
to the napl-native controller. Two missions (SKYDIO_MISSION): `goal` flies from the start to a goal through the forest / wall doorway / arch,
`follow` keeps a walking subject at ~4.5 m behind it through the trees. Run by `napl program examples/skydio/<mission>.scenario.toml`; `python fly.py`
with NAPL_MODEL / NAPL_RUN_DIR set also works (see run_mission for a one-call version that builds the scene itself).

WHAT THE PLANNER SEES. Inputs of every replan: the depth frame of the D435i rangefinder device (160 x 90, 25 Hz, delivered LATENCY_S after it was
sampled, with the camera pose of the estimate at that instant), the vehicle's pose / velocity ESTIMATE (estimator.py: an error-state Kalman filter on the
simulated IMU, a statistical VIO model and the simulated barometer; the kernel's rigid-body state is `Flyer.truth`, used by evaluation only), the goal
position (goal mission), the person detector's output on the depth image (detector.py, follow mission; no ground-truth subject position).
It never reads an obstacle position: `forest.py` (ground truth) is used by evaluation only (clearance, trace.csv) and by the scene builder.

CONTROL. The planner outputs one primitive (velocity ramp + hold + braking tail, planner.py). Every control period (4 ms) the primitive is sampled at the
time since it was planned: position / velocity / acceleration setpoints go to cuas.flyer.Flyer (PD + integral on position, geometric attitude loop,
mixer from the airframe's own rotor geometry) as pos = p_traj + a_traj / k_pos (which makes its PD law a PD law with acceleration feed-forward).

ARTEFACTS in the run dir: trace.csv (62.5 Hz: pose, speed, truth clearance, contact count, monitors' columns), telemetry.jsonl (25 Hz, one record per
replan, see README), depth.npz (every processed depth frame), forest.json (ground-truth scene), replay.naplog + native.json (the runner's replay
hash check), summary.json."""

import csv
import json
import math
import os
import sys
import time
import tomllib
from collections import Counter, deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import follow as F
from detector import PersonDetector, Tracker
from estimator import Eskf, ImuBias, VioSim, baro_height, log_so3
from forest import generate
from mapping import Camera, VoxelMap
from napl import Sim, marine
from napl.cuas import flyer as fl
from napl.cuas.scene import REPO
from planner import (
    GoalField,
    Params,
    Plan,
    Planner,
    follow_objective,
    goal_objective,
)

CTL_TICKS = 4  # control period: 4 ms
LATENCY_S = 0.06  # depth frame sampled -> map + plan available (USB transfer, stereo matching, voxelising, planning on the companion computer: estimate)
R_BOUND = 0.40  # m: the bounding sphere of the airframe about the root plate (prop tips at 0.38 m); `clearance_m` is measured against it
TAKEOFF_V, MAP_S = (
    0.7,
    1.0,
)  # climb rate (m/s) and the hover during which the first frames build the map
YAW_RATE = 1.6  # rad/s limit of the heading command
FOLLOW_DIST, FOLLOW_Z = 4.5, 1.9  # m behind the subject, flying height
SUBJECT_KEEP_OUT_LAST = 1.8  # m: static clearance from the last detected position while the subject is lost (it may have stopped there)
AIM_KI = 0.15  # 1/s: gain of that integral action
SUBJECT_KEEP_OUT_UNSEEN = 2.6  # m: the same while the detector has lost the subject (a person closer than ~3 m is below the camera's view: do not close in blind)
SUBJECT_KEEP_OUT = 1.1  # m: horizontal clearance from the subject's predicted track (0.35 person + 0.40 airframe sphere + 0.35 for track error)
FOLLOW_PULL = (
    0.6,
    1.2,
    2.5,
)  # 1/s: gains of the extra "subject velocity + pull toward the spot behind it" candidates
ARRIVE_M = 1.0  # m: the goal counts as reached inside this horizontal distance
MODES = ("track", "brake", "escape", "scan")
IMU_TICKS = 2  # the imu device samples every 2nd 1 ms tick (500 Hz): the loop advances in 2-tick blocks and reads each sample
BARO_EVERY = 5  # control periods between barometer fixes (20 ms = 50 Hz)
VIO_LATENCY_S = 0.05  # VIO frame sampled -> pose available to the filter (stereo processing + IMU pre-integration on the companion computer: estimate)


def imu_sigmas(c):
    """Per-sample white-noise sigma (gyro rad/s, accel m/s^2) of the imu device from its config: noise density x sqrt(rate / 2) (sensors.cpp)."""
    k = (float(c["rate_hz"]) / 2.0) ** 0.5
    return float(c["gyro"]["noise_density"]) * k, float(c["accel"]["noise_density"]) * k


def optical_offset():
    """The D435i optical frame origin in the part frame (the `optical` port) from the part file: the rangefinder's site."""
    port = next(
        p
        for p in tomllib.loads(
            (REPO / "parts/electronics/intel-realsense-d435i.toml").read_text()
        )["ports"]
        if p["name"] == "optical"
    )
    return np.array(port["pos"], float)


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class EstFlyer(fl.Flyer):
    """The napl-native controller fed with the ESTIMATOR's position / velocity / attitude / body rate instead of the kernel's state (as a flight
    controller runs on its EKF output). `truth` keeps the kernel's state of the last update, for evaluation only."""

    est = None
    truth = None

    def state(self):
        self.truth = super().state()
        if self.est is None:
            return self.truth
        e = self.est
        return e.p.copy(), e.R.copy(), np.concatenate([e.v, e.w])


class Skydio:
    def __init__(self, model, run_dir, mission, seed, cal_dir=None, params=None):
        self.mission, self.seed = mission, seed
        self.run_dir = Path(run_dir)
        self.sim = Sim.load(model)
        self.sim.start_log()  # before any command: the replay log starts at tick 0
        man = json.loads((Path(model).parent / "scene.json").read_text())
        self.dt = man["dt"]
        self.dtc = self.dt * CTL_TICKS
        self.forest = generate(seed, mission)
        cfg, dev = man["device_cfg"], man["devices"]
        self.cam = Camera.from_cfg(cfg["skydio_depth"])
        self.depth_dev = "/devices/skydio_depth"
        rotors = [cfg[n] for n in dev["rotors"] if n.startswith("skydio_")]
        escs = {cfg[n]["joint"]: cfg[n] for n in dev["escs"] if n.startswith("skydio_")}
        root = man["body_paths"]["skydio_bottom"]
        paths = dict(
            root=root,
            free=f"{root}/bottom:free",
            escs=[f"/devices/{escs[r['joint']]['name']}" for r in rotors],
            yaw0=0.0,
        )
        cal = fl.calibrate("skydio", Path(cal_dir or self.run_dir / "cal"))
        self.prm = params or Params()
        self.flyer = EstFlyer(self.sim, cal, paths, self.dtc, max_tilt=0.45)
        self.root = root
        self.free = paths["free"]
        cam_path = man["body_paths"]["skydio/cam"]
        q = self.sim.query
        self.cam_p_root = q(cam_path, "pos", root)[
            0
        ]  # calibration: camera extrinsics in the root plate's frame (a fixed mount)
        self.cam_R_root = quat_R(q(cam_path, "quat", root)[0])
        self.cam_o_root = self.cam_p_root + self.cam_R_root @ optical_offset()
        self.map = VoxelMap(hit_hold=int(os.environ.get("SKYDIO_HIT_HOLD", 12)))
        self.planner = Planner(self.prm)
        self.hist = deque(maxlen=8)
        self.pending = deque()  # (deliver_at, depth, pos, R, t_sample)
        self.frames_seen = 0
        self.plan, self.plan_t = None, 0.0
        self.k, self.phase, self.t_phase = 0, "takeoff", 0.0
        self.rng = np.random.default_rng(seed + 1000)
        self.goal = np.array(self.forest.goal, float)
        self.z_c = FOLLOW_Z if mission == "follow" else self.prm.z_cruise
        self.yaw_sp = 0.0
        self.scan_dir = 1.0 if seed % 2 == 0 else -1.0
        # follow mission
        self.subject = self.det = self.trk = None
        self.est = (
            None  # follow: (subject xy, vxy, seen, sigma_xy) in the ESTIMATE frame
        )
        if mission == "follow":
            self.subject = F.Subject(self.sim, self.forest.path)
            self.det = PersonDetector(self.cam)
            self.trk = Tracker(keep_s=float(os.environ.get("SKYDIO_KEEP_S", 5.0)))
        self.last_det, self.last_dets, self.last_flags, self.det_stats = (
            None,
            [],
            [],
            Counter(),
        )
        # records
        self.rows, self.tele, self.depth_log = (
            [],
            open(self.run_dir / "telemetry.jsonl", "w"),
            [],
        )
        self.n_contact = 0
        self.min_clear = 99.0
        self.path_len, self.prev_p, self.t_fly0, self.t_goal = 0.0, None, None, None
        self.plan_ms, self.modes = [], Counter()
        self.wall0 = time.monotonic()
        p0, R0, _ = (
            self.flyer.state()
        )  # the take-off pad is surveyed: the estimator starts at the true pose, at rest
        self.p_start = p0.copy()
        # estimator + its sensors (estimator.py): IMU device + python-side bias, VIO model, barometer device
        self.imu_dev, self.baro_dev = "/devices/skydio_imu", "/devices/skydio_baro"
        erng = np.random.default_rng(seed + 2000)
        self.imu_bias, self.vio = ImuBias(erng), VioSim(erng)
        rate = float(cfg["skydio_imu"]["rate_hz"])
        sg, sa = imu_sigmas(cfg["skydio_imu"])
        self.imu_dt = 1.0 / rate
        self.eskf = Eskf(p0, np.zeros(3), R0, sg, sa)
        self.flyer.est = self.eskf
        self.vio_pending = deque()  # (deliver_at, t_sample, v_body, R_meas)
        self.vio_t, self.baro_ref = 0.0, None
        self.err = []  # (t, |dp|, |dv|, attitude error deg) while flying
        self.t_stuck = None  # since when the follower has been crawling without the subject in view
        self.aim_off = (
            0.0  # m: integral correction of the follow distance (see _follow_target)
        )
        self.flyer.target(p0, np.zeros(3), 0.0)

    @property
    def t(self):
        return self.k * self.dtc

    # ---- one control period -----------------------------------------------------------------------------------------------------------
    def step(self):
        self._setpoint()
        self.flyer.update()
        p, v, R, tilt, _ = self.flyer.last  # the estimate
        self.hist.append((self.t, p.copy(), R.copy()))
        if self.subject:
            self.subject.update()
        for _ in range(CTL_TICKS // IMU_TICKS):
            self.sim.step(IMU_TICKS)
            self._imu()
        self.k += 1
        self._vio_baro()
        self._sense()
        if self.k % 4 == 0:
            self._trace(*self.flyer.truth[:2], self.flyer.truth[2][:3], tilt)

    # ---- estimator inputs -----------------------------------------------------------------------------------------------------------
    def _imu(self):
        """One IMU sample (500 Hz): the device's reading (noise, quantisation, range) plus the python-side bias -> strapdown + covariance prediction."""
        q = self.sim.query
        gyro, accel = q(self.imu_dev, "gyro")[0], q(self.imu_dev, "accel")[0]
        if not np.any(accel):  # the device reads 0 before its first sample
            return
        gyro, accel = self.imu_bias.apply(gyro, accel, self.imu_dt)
        self.eskf.predict(gyro, accel, self.imu_dt)

    def _truth(self):
        q = self.sim.query
        return (
            q(self.root, "pos")[0],
            quat_R(q(self.root, "quat")[0]),
            q(self.free, "qvel")[0][:3],
        )

    def _vio_baro(self):
        t = self.eskf.t
        while self.vio_pending and self.vio_pending[0][0] <= t + 1e-9:
            _, t_s, vb, Rm = self.vio_pending.popleft()
            self.eskf.update_vio(t_s, vb, Rm)
        pa = float(self.sim.query(self.baro_dev, "pressure")[0][0])
        if (
            self.k % BARO_EVERY == 0 and pa > 1e4
        ):  # (0 before the device's first sample)
            z = baro_height(pa)
            if self.baro_ref is None:
                self.baro_ref = z - self.p_start[2]
            self.eskf.update_height(z - self.baro_ref)

    def _setpoint(self):
        fl_, t = self.flyer, self.t
        if self.flyer.last is None:
            return
        p, v, R, _, _ = fl_.last
        if self.phase == "takeoff":
            z = min(self.z_c, self.p_start[2] + TAKEOFF_V * max(t - 0.4, 0.0))
            fl_.target(
                [self.p_start[0], self.p_start[1], z],
                [0, 0, TAKEOFF_V if z < self.z_c else 0.0],
                0.0,
            )
            if z >= self.z_c and p[2] > self.z_c - 0.12 and abs(v[2]) < 0.3:
                self._enter("map")
        elif self.phase == "map":
            fl_.target(
                [self.p_start[0], self.p_start[1], self.z_c], np.zeros(3), self.yaw_sp
            )
            if t - self.t_phase >= MAP_S:
                self._enter("fly")
                self.t_fly0 = t
                if self.subject:
                    self.subject.go = True
        else:
            if self.plan is None:
                fl_.target(p, np.zeros(3), self.yaw_sp)
                return
            tau = t - self.plan_t
            pp, vv, aa = self.plan.at(tau)
            kxy, kz = fl.WN_XY**2, fl.WN_Z**2
            sp = pp + np.array([aa[0] / kxy, aa[1] / kxy, aa[2] / kz])
            self._yaw_update()
            fl_.target(sp, vv, self.yaw_sp)

    def _enter(self, phase):
        self.phase, self.t_phase = phase, self.t

    def _yaw_update(self):
        """Heading command: toward the plan's motion, else the subject / goal; the camera looks where the vehicle is going."""
        plan, dt = self.plan, self.dtc
        if self.phase == "arrived":
            return
        v = plan.vdes[:2] if plan else np.zeros(2)
        lost = self.mission == "follow" and self.est is not None and not self.est[2]
        if lost and np.linalg.norm(self.flyer.last[1][:2]) < 0.6:
            self.t_stuck = self.t_stuck if self.t_stuck is not None else self.t
        else:
            self.t_stuck = None
        if (
            self.t_stuck is not None and self.t - self.t_stuck > 1.5
        ):  # lost the subject and crawling: turn to look for a way (the camera only sees ahead)
            want = self.yaw_sp + self.scan_dir * 2.0
        elif plan and plan.mode == "scan":
            want = self.yaw_sp + self.scan_dir * 2.0  # turn on the spot to look around
        elif (
            self.mission == "follow"
            and self.est is not None
            and (self.est[2] or np.linalg.norm(v) < 0.5)
        ):  # unseen and moving: look where we fly (a camera swept sideways sees no free path ahead: tried, the follower stalled more often)
            d = self.est[0] - self.flyer.last[0][:2]
            want = math.atan2(d[1], d[0])
        elif np.linalg.norm(v) > 0.7:
            want = math.atan2(v[1], v[0])
        else:
            d = self.goal - self.flyer.last[0][:2]
            want = math.atan2(d[1], d[0])
        self.yaw_sp = wrap(
            self.yaw_sp
            + float(np.clip(wrap(want - self.yaw_sp), -YAW_RATE * dt, YAW_RATE * dt))
        )

    # ---- sensing and planning ---------------------------------------------------------------------------------------------------------
    def _sense(self):
        n = int(self.sim.query(self.depth_dev, "frames")[0][0])
        if (
            n != self.frames_seen
        ):  # a new depth frame: the pose of the sampling instant is the last control step's
            self.frames_seen = n
            t_s, p, R = self.hist[
                -1
            ]  # ESTIMATED pose at the sampling instant: the map and the detector use only this
            img = marine.depth_image(
                self.sim, self.depth_dev, self.cam.n_h, self.cam.n_v
            )
            cam_pos = p + R @ self.cam_o_root
            cam_R = R @ self.cam_R_root
            pt, Rt, vt = (
                self.flyer.truth[0],
                self.flyer.truth[1],
                self.flyer.truth[2][:3],
            )  # truth of the same instant: VIO input + detector scoring
            tcam = (pt + Rt @ self.cam_o_root, Rt @ self.cam_R_root)
            vb, Rm = self.vio.sample(max(t_s - self.vio_t, 1e-3), Rt, vt)
            self.vio_t = t_s
            t_f = self.eskf.t - self.dtc  # filter clock of the sampling instant
            self.vio_pending.append((self.eskf.t + VIO_LATENCY_S, t_f, vb, Rm))
            self.pending.append((self.t + LATENCY_S, img, cam_pos, cam_R, t_s, tcam))
        while self.pending and self.pending[0][0] <= self.t + 1e-9:
            _, img, cam_pos, cam_R, t_s, tcam = self.pending.popleft()
            self._process(img, cam_pos, cam_R, t_s, tcam)

    def _score(self, dets, tcam, img):
        """Evaluation only: which detections sit on the true subject (its centre projected with the TRUE camera pose), and whether it was visible."""
        c = self.cam
        pc = (np.append(self.subject.pos()[:2], F.CENTRE_Z) - tcam[0]) @ tcam[1]
        i, j, z = (float(a[0]) for a in c.project(pc[None]))
        flags = [
            bool(
                np.isfinite(i)
                and abs(d["row"] - i) <= d["half_h"] + 2
                and abs(d["col"] - j) <= d["half_w"] + 2
            )
            for d in dets
        ]
        visible = bool(
            np.isfinite(i)
            and c.min_range + 0.2 < z < 8.0
            and 2.0 <= 0.5 * F.HEIGHT * 0.5 * c.n_v / np.tan(c.fov_v / 2) / z
        )
        if visible:  # not hidden behind something nearer
            r, q = int(round(i)), int(round(j))
            patch = img[max(r - 1, 0) : r + 2, max(q - 1, 0) : q + 2]
            d = np.nanmedian(patch) if np.isfinite(patch).any() else np.nan
            visible = bool(np.isfinite(d) and abs(d - z) < 0.8)
        if self.phase not in ("fly", "arrived"):
            return flags
        self.det_stats["frames"] += 1
        self.det_stats["visible"] += visible
        self.det_stats["detected_visible"] += visible and any(flags)
        self.det_stats["detections"] += len(dets)
        self.det_stats["false_positives"] += len(dets) - sum(flags)
        self.det_stats["fp_frames"] += len(dets) > sum(flags)
        return flags

    def _process(self, img, cam_pos, cam_R, t_s, tcam):
        w0 = time.perf_counter()
        ignore = None
        if self.mission == "follow":
            dets = self.det.detect(img, cam_pos, cam_R)
            r = self.trk.update(t_s, dets)
            self.last_dets, self.last_flags = dets, self._score(dets, tcam, img)
            self.last_det = r[4] if r else None
            box = self.last_det
            if box is None and r is not None and t_s - self.trk.t_seen < 1.0:
                box = F.track_box(
                    self.cam, r[0], cam_pos, cam_R
                )  # missed this frame: keep the tracked subject out of the map anyway
            if box is not None:
                ignore = F.subject_mask(self.cam, box)
            self.est = None if r is None else (r[0], r[1], r[2], r[3])
        self.map.integrate(img, self.cam, cam_pos, cam_R, ignore)
        self.depth_log.append((t_s, img.astype(np.float16)))
        if self.phase in ("fly", "arrived"):
            self._plan()
        self.plan_ms.append(1000 * (time.perf_counter() - w0))

    def _plan(self):
        p, v, R = self.flyer.last[0], self.flyer.last[1], self.flyer.last[2]
        yaw = yaw_of(R)
        win = self.map.window(p, half=10.0)
        moving = None
        if self.mission == "goal":
            field = GoalField(win, self.goal)
            bearing, obj = field.bearing(p), goal_objective(self.goal, self.z_c, field)
            extra_v = ()
        else:
            bearing, obj, extra_v, moving = self._follow_target(p, win)
        if (
            bearing is None
        ):  # follow: no subject track at all -> brake to a stop and turn on the spot to look for it
            prm = self.prm
            plan = Plan(
                p.copy(),
                v.copy(),
                np.zeros(3),
                prm.a_max,
                prm.a_brake,
                prm.horizon,
                "scan",
            )
            plan.telemetry = dict(n=0, n_safe=0, chosen=0, candidates=[])
        else:
            plan = self.planner.plan(p, v, yaw, win, bearing, obj, extra_v, moving)
        self.modes[plan.mode] += 1
        self.plan, self.plan_t = plan, self.t
        if (
            self.mission == "goal"
            and np.linalg.norm(self.goal - p[:2]) < ARRIVE_M
            and np.linalg.norm(v) < 1.0
            and self.phase == "fly"
        ):
            self._enter("arrived")
        self._telemetry(p, v, R, plan, win)

    def _follow_target(self, p, win):
        """-> (bearing, objective, extra candidate velocities): aim FOLLOW_DIST behind the subject's PREDICTED position (the track is as old as the
        depth frame, LATENCY_S), and add velocity candidates that match the subject's velocity plus a proportional pull toward that spot."""
        if self.est is None:
            return None, None, (), None
        vs = self.est[1]
        ps = self.est[0] + vs * LATENCY_S
        sp = np.linalg.norm(vs)
        back = (
            -vs / sp
            if sp > 0.6
            else (p[:2] - ps) / max(np.linalg.norm(p[:2] - ps), 1e-6)
        )
        gap = float(np.linalg.norm(p[:2] - ps)) - FOLLOW_DIST
        if (
            self.est[2] and sp > 0.8 and abs(gap) < 1.5
        ):  # integral action on the measured distance: removes the steady trailing of the open-loop aim
            self.aim_off = float(
                np.clip(self.aim_off - AIM_KI * gap * self.dtc * 10, -1.0, 0.5)
            )  # 10 control periods per depth frame
        des = ps + (FOLLOW_DIST + self.aim_off) * back
        field = GoalField(win, des)
        vz = float(np.clip(self.z_c - p[2], -1.2, 1.2))
        extra_v = []
        for gain in FOLLOW_PULL:
            vxy = vs + gain * (des - p[:2])
            n = np.linalg.norm(vxy)
            vxy = vxy * min(1.0, self.prm.v_cruise / max(n, 1e-6))
            extra_v.append(np.array([vxy[0], vxy[1], vz]))
        moving = [
            (ps, vs, SUBJECT_KEEP_OUT if self.est[2] else SUBJECT_KEEP_OUT_UNSEEN)
        ]  # the person is not in the map (masked): keep clear of its predicted track
        return field.bearing(p), follow_objective(des, vs, field=field), extra_v, moving

    # ---- records ----------------------------------------------------------------------------------------------------------------------
    def _telemetry(self, p, v, R, plan, win):
        pp, vv, aa = plan.at(0.0)
        taus = np.arange(0.0, plan.duration + 1e-9, 0.1)
        pt, Rt, vt = self.flyer.truth[0], self.flyer.truth[1], self.flyer.truth[2][:3]
        sg = self.eskf.sigmas()
        rec = dict(
            t=round(self.t, 3), tick=self.sim.tick, phase=self.phase, mode=plan.mode,
            pos=np.round(pt, 3).tolist(), vel=np.round(vt, 3).tolist(), yaw=round(yaw_of(Rt), 4), tilt_deg=round(float(np.degrees(self.flyer.last[3])), 2),
            est_pos=np.round(p, 3).tolist(), est_vel=np.round(v, 3).tolist(), est_yaw=round(yaw_of(R), 4),
            est_cov=dict(pos_sigma=np.round(sg[0:3], 4).tolist(), vel_sigma=np.round(sg[3:6], 4).tolist(), att_sigma_deg=np.round(np.degrees(sg[6:9]), 3).tolist()),
            est_bias=dict(gyro=np.round(self.eskf.bg, 5).tolist(), accel=np.round(self.eskf.ba, 4).tolist()),
            est_err=dict(pos=np.round(pt - p, 3).tolist(), pos_norm=round(float(np.linalg.norm(pt - p)), 3), vel_norm=round(float(np.linalg.norm(vt - v)), 3),
                         yaw_deg=round(math.degrees(wrap(yaw_of(Rt) - yaw_of(R))), 3)),
            setpoint=dict(pos=np.round(pp, 3).tolist(), vel=np.round(vv, 3).tolist(), yaw=round(self.yaw_sp, 4), vdes=np.round(plan.vdes, 3).tolist()),
            chosen=np.round([plan.at(x)[0] for x in taus], 3).tolist(), chosen_dt=0.1, chosen_cost=None if not np.isfinite(plan.cost) else round(plan.cost, 3),
            candidates=plan.telemetry["candidates"], n_candidates=plan.telemetry["n"], n_safe=plan.telemetry["n_safe"],
            map=dict(voxels=self.map.n_occupied, frames=self.map.frames, clearance_est=round(float(win.clearance(p[None])[0]), 3)),
            min_clearance=round(self.forest.clearance(pt) - R_BOUND, 3), goal=self.goal.tolist(), depth_index=len(self.depth_log) - 1,
        )  # fmt: skip
        if self.mission == "follow":
            rec["subject"] = dict(est=None if self.est is None else np.round(self.est[0], 3).tolist(), est_vel=None if self.est is None else np.round(self.est[1], 3).tolist(),
                                  seen=bool(self.est and self.est[2]), box=None if self.last_det is None else {k: round(float(self.last_det[k]), 2) for k in ("row", "col", "half_h", "half_w")},
                                  truth=np.round(self.subject.pos(), 3).tolist(), sigma=None if self.est is None else round(float(self.est[3]), 3),
                                  detections=[dict(row=round(d["row"], 1), col=round(d["col"], 1), half_h=round(d["half_h"], 1), half_w=round(d["half_w"], 1),
                                                   range=round(d["range"], 2), score=round(d["score"], 2), xy=np.round(d["xy"], 2).tolist(), z_top=round(d["z_top"], 2), width=round(d["width"], 2), on_subject=ok) for d, ok in zip(self.last_dets, self.last_flags)])  # fmt: skip
        self.tele.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def _trace(self, p, R_true, v, tilt_cmd):
        """One 16 ms row from the KERNEL's state (truth: evaluation and the scenario monitors) plus the estimate and its error."""
        tilt = math.acos(max(-1.0, min(1.0, R_true[2, 2])))
        pe, ve = self.eskf.p, self.eskf.v
        if self.phase in ("fly", "arrived"):
            self.err.append(
                (
                    self.t,
                    float(np.linalg.norm(p - pe)),
                    float(np.linalg.norm(v - ve)),
                    math.degrees(
                        float(np.linalg.norm(log_so3(R_true.T @ self.eskf.R)))
                    ),
                )
            )
        clear = self.forest.clearance(p) - R_BOUND
        if self.subject:
            s = self.subject.pos()  # the person is a vertical segment (feet to head) of radius 0.2, not a point at its feet
            nearest = np.array([s[0], s[1], min(max(p[2], 0.0), F.HEIGHT)])
            clear = min(clear, float(np.linalg.norm(p - nearest) - 0.2 - R_BOUND))
        self.min_clear = min(self.min_clear, clear)
        contact = int(
            self.sim.query("/sensors/hit_obstacles", "value")[0][0]
            + (
                self.sim.query("/sensors/hit_subject", "value")[0][0]
                if self.subject
                else 0
            )
        )
        self.n_contact += contact > 0
        speed = float(np.linalg.norm(v))
        if self.t_fly0 is not None and self.t_goal is None:
            self.path_len += (
                float(np.linalg.norm(p - self.prev_p))
                if self.prev_p is not None
                else 0.0
            )
        self.prev_p = p.copy()
        dg = float(np.linalg.norm(self.goal - p[:2]))
        if (
            self.mission == "goal"
            and dg < ARRIVE_M * 1.5
            and self.t_goal is None
            and self.t_fly0 is not None
        ):
            self.t_goal = self.t
        q = self.sim.query(self.root, "quat")[0]
        fd = (
            float(np.linalg.norm(p[:2] - self.subject.pos()[:2]))
            if self.subject
            else 0.0
        )
        self.rows.append([self.t, *p, *q, speed, clear, self.n_contact, dg, math.degrees(tilt), self.map.n_occupied, MODES.index(self.plan.mode) if self.plan else 0,
                          fd, float(bool(self.est and self.est[2])), 1.0 if self.phase in ("fly", "arrived") else 0.0, *pe, float(np.linalg.norm(p - pe)),
                          float(bool(self.subject and self.subject.go and self.subject.s < self.subject.length - 1.0))])  # fmt: skip

    COLS = [
        "t",
        "x",
        "y",
        "z",
        "qw",
        "qx",
        "qy",
        "qz",
        "speed",
        "clearance_m",
        "contact_steps",
        "dist_goal",
        "tilt_deg",
        "map_voxels",
        "planner_mode",
        "follow_dist",
        "subject_seen",
        "flying",
        "est_x",
        "est_y",
        "est_z",
        "est_err_m",
        "subject_walking",
    ]

    # ---- run + finish -----------------------------------------------------------------------------------------------------------------
    def crashed(self):
        p, tilt = (
            self.flyer.truth[0],
            math.acos(max(-1.0, min(1.0, self.flyer.truth[1][2, 2]))),
        )  # the kernel's state: a crash is physics, not belief
        return (
            tilt > 1.4 or (self.phase == "fly" and p[2] < 0.15) or self.n_contact > 40
        )

    def run(self, t_max):
        arrived_at = None
        subject_done = None
        while self.t < t_max:
            self.step()
            if self.phase == "arrived":
                arrived_at = arrived_at or self.t
                if self.t - arrived_at > 2.0:
                    break
            if self.subject and self.subject.s >= self.subject.length - 0.5:
                subject_done = subject_done or self.t
                break  # the mission ends when the walk does (a person standing inside the camera's blind zone is not tracked)
            if self.flyer.last is not None and self.crashed():
                break
        return self.finish()

    def finish(self):
        r = self.run_dir
        self.tele.close()
        with open(r / "trace.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            w.writerows(self.rows)
        tr = np.array(self.rows)
        col = {c: tr[:, i] for i, c in enumerate(self.COLS)}
        fly = col["flying"] > 0
        v = col["speed"][fly]
        d = (
            np.array([x[1] for x in self.depth_log])
            if self.depth_log
            else np.zeros((0, 1, 1))
        )
        np.savez_compressed(
            r / "depth.npz", t=np.array([x[0] for x in self.depth_log]), depth=d
        )
        (r / "replay.naplog").write_bytes(self.sim.log())
        summ = dict(mission=self.mission, seed=self.seed, sim_s=self.t, wall_s=time.monotonic() - self.wall0, min_clearance_m=self.min_clear,
                    contact_steps=int(self.n_contact), time_to_goal_s=None if self.t_goal is None else self.t_goal - self.t_fly0,
                    reached_goal=self.t_goal is not None, path_length_m=self.path_len, fly_s=float(fly.sum() * 4 * self.dtc),
                    mean_speed_mps=float(v.mean()) if len(v) else 0.0, max_speed_mps=float(v.max()) if len(v) else 0.0,
                    cruise_speed_p90_mps=float(np.percentile(v, 90)) if len(v) else 0.0, planner=dict(n=len(self.plan_ms), mean_ms=float(np.mean(self.plan_ms)) if self.plan_ms else 0.0,
                    max_ms=float(np.max(self.plan_ms)) if self.plan_ms else 0.0, modes=dict(self.modes)), max_tilt_deg=float(col["tilt_deg"].max()))  # fmt: skip
        if self.subject:
            walking = col["subject_walking"] > 0
            stopped = (
                fly & ~walking & (col["t"] > (self.t_fly0 or 0) + 8.0)
            )  # after the walk ends the subject stands still
            fd = col["follow_dist"][
                fly & walking & (col["t"] > (self.t_fly0 or 0) + 8.0)
            ]
            if stopped.any():
                summ["follow_dist_min_after_stop"] = float(
                    col["follow_dist"][stopped].min()
                )
            summ.update(follow_dist_median=float(np.median(fd)) if len(fd) else None, follow_dist_p5_p95=[float(np.percentile(fd, 5)), float(np.percentile(fd, 95))] if len(fd) else None,
                        subject_seen_frac=float(col["subject_seen"][fly].mean()) if fly.any() else 0.0)  # fmt: skip
        if self.err:
            e = np.array(self.err)
            summ["estimation"] = dict(pos_rms_m=float(np.sqrt((e[:, 1] ** 2).mean())), pos_max_m=float(e[:, 1].max()), pos_final_m=float(e[-1, 1]),
                                      drift_pct_of_path=100.0 * float(e[-1, 1]) / max(self.path_len, 1e-6), vel_rms_mps=float(np.sqrt((e[:, 2] ** 2).mean())),
                                      att_rms_deg=float(np.sqrt((e[:, 3] ** 2).mean())), att_max_deg=float(e[:, 3].max()),
                                      sigma_pos_final_m=float(np.linalg.norm(self.eskf.sigmas()[:3])), vio_scale_err=float(self.vio.scale))  # fmt: skip
        if self.subject:
            ds = self.det_stats
            summ["detection"] = dict(
                dict(ds),
                recall=ds["detected_visible"] / max(ds["visible"], 1),
                fp_per_frame=ds["false_positives"] / max(ds["frames"], 1),
            )
        (r / "summary.json").write_text(json.dumps(summ, indent=1))
        (r / "native.json").write_text(
            json.dumps(
                dict(
                    tick=self.sim.tick,
                    dt=self.dt,
                    state_hash=f"{self.sim.state_hash():016x}",
                    diverged=bool(self.sim.diverged),
                )
            )
        )
        return summ


T_MAX = {"goal": 70.0, "follow": 60.0}


def run_mission(mission, seed, out, cal_dir=None, t_max=None, params=None):
    """Build the scene for (mission, seed) under `out`/model and fly it, writing the artefacts into `out`. -> summary dict. (`napl program` does
    the same in two steps: the runner builds the model, this file's main() flies it.)"""
    import skydio_scene as scene

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    model = scene.build(out / "model", mission, seed)
    return Skydio(model, out, mission, seed, cal_dir, params).run(
        t_max or T_MAX[mission]
    )


def main():
    mission = os.environ.get("SKYDIO_MISSION", "goal")
    seed = int(os.environ.get("SKYDIO_SEED", 1))
    cal_dir = os.environ.get("SKYDIO_CAL")
    if "NAPL_MODEL" in os.environ:  # under `napl program`
        run = Skydio(
            os.environ["NAPL_MODEL"], os.environ["NAPL_RUN_DIR"], mission, seed, cal_dir
        )
        s = run.run(float(os.environ.get("SKYDIO_T_MAX", T_MAX[mission])))
    else:  # standalone: python fly.py <out dir> [mission] [seed]
        out = sys.argv[1]
        mission = sys.argv[2] if len(sys.argv) > 2 else mission
        seed = int(sys.argv[3]) if len(sys.argv) > 3 else seed
        s = run_mission(
            mission,
            seed,
            out,
            cal_dir,
            float(os.environ.get("SKYDIO_T_MAX", T_MAX[mission])),
        )
    sys.stdout.write(json.dumps(s) + "\n")


if __name__ == "__main__":
    main()
