"""The Skydio-style autonomy program: depth image -> point cloud -> rolling voxel map -> sampled-primitive planner -> acceleration-feedforward setpoints
to the napl-native controller. Two missions (SKYDIO_MISSION): `goal` flies from the start to a goal through the forest / wall doorway / arch,
`follow` keeps a walking subject at ~4.5 m behind it through the trees. Run by `napl program examples/skydio/<mission>.scenario.toml`; `python fly.py`
with NAPL_MODEL / NAPL_RUN_DIR set also works (see run_mission for a one-call version that builds the scene itself).

WHAT THE PLANNER SEES. Inputs of every replan: the depth frame of the D435i rangefinder device (160 x 90, 25 Hz, delivered LATENCY_S after it was
sampled, with the camera pose of the sampling instant), the vehicle's pose / velocity estimate (the simulator's rigid-body state: a stand-in for
visual-inertial odometry, no estimator error is added), the goal position (goal mission), the detector's pixel box of the subject (follow mission).
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
ARRIVE_M = 1.0  # m: the goal counts as reached inside this horizontal distance
MODES = ("track", "brake", "escape", "scan")


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
        self.flyer = fl.Flyer(self.sim, cal, paths, self.dtc, max_tilt=0.45)
        self.root = root
        cam_path = man["body_paths"]["skydio/cam"]
        q = self.sim.query
        self.cam_p_root = q(cam_path, "pos", root)[
            0
        ]  # calibration: camera extrinsics in the root plate's frame (a fixed mount)
        self.cam_R_root = quat_R(q(cam_path, "quat", root)[0])
        self.cam_o_root = self.cam_p_root + self.cam_R_root @ optical_offset()
        self.map = VoxelMap()
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
        self.est = None
        if mission == "follow":
            self.subject = F.Subject(self.sim, self.forest.path)
            self.det = F.SubjectDetector(self.cam, lambda: self.subject.pos(), self.rng)
            self.trk = F.SubjectTracker()
        self.last_det = None
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
        p0 = self.flyer.state()[0]
        self.p_start = p0.copy()
        self.flyer.target(p0, np.zeros(3), 0.0)

    @property
    def t(self):
        return self.k * self.dtc

    # ---- one control period -----------------------------------------------------------------------------------------------------------
    def step(self):
        self._setpoint()
        self.flyer.update()
        p, v, R, tilt, _ = self.flyer.last
        self.hist.append((self.t, p.copy(), R.copy()))
        if self.subject:
            self.subject.update()
        self.sim.step(CTL_TICKS)
        self.k += 1
        self._sense()
        if self.k % 4 == 0:
            self._trace(p, v, R, tilt)

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
        if plan and plan.mode == "scan":
            want = self.yaw_sp + self.scan_dir * 2.0  # turn on the spot to look around
        elif self.mission == "follow" and self.est is not None:
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
            t_s, p, R = self.hist[-1]
            img = marine.depth_image(
                self.sim, self.depth_dev, self.cam.n_h, self.cam.n_v
            )
            cam_pos = p + R @ self.cam_o_root
            cam_R = R @ self.cam_R_root
            self.pending.append((self.t + LATENCY_S, img, cam_pos, cam_R, t_s))
        while self.pending and self.pending[0][0] <= self.t + 1e-9:
            _, img, cam_pos, cam_R, t_s = self.pending.popleft()
            self._process(img, cam_pos, cam_R, t_s)

    def _process(self, img, cam_pos, cam_R, t_s):
        w0 = time.perf_counter()
        ignore, extra = None, []
        if self.mission == "follow":
            det = self.det.detect(img, cam_pos, cam_R)
            self.last_det = det
            if det is not None:
                ignore = F.subject_mask(self.cam, det)
            self.est = self.trk.update(t_s, self.cam, det, img, cam_pos, cam_R)
            if self.est is not None:
                extra = [(self.est[0][0], self.est[0][1], z, 0.45) for z in (0.9, 1.7)]
            self.est = (self.est[0], self.est[1], self.est[2]) if self.est else None
        self.map.integrate(img, self.cam, cam_pos, cam_R, ignore)
        self.depth_log.append((t_s, img.astype(np.float16)))
        if self.phase in ("fly", "arrived"):
            self._plan(extra)
        self.plan_ms.append(1000 * (time.perf_counter() - w0))

    def _plan(self, extra):
        p, v, R = self.flyer.last[0], self.flyer.last[1], self.flyer.last[2]
        yaw = yaw_of(R)
        win = self.map.window(p, half=10.0, extra=extra)
        if self.mission == "goal":
            field = GoalField(win, self.goal)
            bearing, obj = field.bearing(p), goal_objective(self.goal, self.z_c, field)
        else:
            bearing, obj = self._follow_target(p, win)
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
            plan = self.planner.plan(p, v, yaw, win, bearing, obj)
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
        if self.est is None:
            return None, None
        ps, vs = self.est[0], self.est[1]
        sp = np.linalg.norm(vs)
        back = (
            -vs / sp
            if sp > 0.6
            else (p[:2] - ps) / max(np.linalg.norm(p[:2] - ps), 1e-6)
        )
        des = ps + FOLLOW_DIST * back
        field = GoalField(win, des)
        return field.bearing(p), follow_objective(des, vs, field=field)

    # ---- records ----------------------------------------------------------------------------------------------------------------------
    def _telemetry(self, p, v, R, plan, win):
        pp, vv, aa = plan.at(0.0)
        taus = np.arange(0.0, plan.duration + 1e-9, 0.1)
        rec = dict(
            t=round(self.t, 3), tick=self.sim.tick, phase=self.phase, mode=plan.mode,
            pos=np.round(p, 3).tolist(), vel=np.round(v, 3).tolist(), yaw=round(yaw_of(R), 4), tilt_deg=round(float(np.degrees(self.flyer.last[3])), 2),
            setpoint=dict(pos=np.round(pp, 3).tolist(), vel=np.round(vv, 3).tolist(), yaw=round(self.yaw_sp, 4), vdes=np.round(plan.vdes, 3).tolist()),
            chosen=np.round([plan.at(x)[0] for x in taus], 3).tolist(), chosen_dt=0.1, chosen_cost=None if not np.isfinite(plan.cost) else round(plan.cost, 3),
            candidates=plan.telemetry["candidates"], n_candidates=plan.telemetry["n"], n_safe=plan.telemetry["n_safe"],
            map=dict(voxels=self.map.n_occupied, frames=self.map.frames, clearance_est=round(float(win.clearance(p[None])[0]), 3)),
            min_clearance=round(self.forest.clearance(p) - R_BOUND, 3), goal=self.goal.tolist(), depth_index=len(self.depth_log) - 1,
        )  # fmt: skip
        if self.mission == "follow":
            rec["subject"] = dict(est=None if self.est is None else np.round(self.est[0], 3).tolist(), est_vel=None if self.est is None else np.round(self.est[1], 3).tolist(),
                                  seen=bool(self.est and self.est[2]), box=None if self.last_det is None else {k: round(float(x), 2) for k, x in self.last_det.items()},
                                  truth=np.round(self.subject.pos(), 3).tolist())  # fmt: skip
        self.tele.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def _trace(self, p, v, R, tilt):
        clear = self.forest.clearance(p) - R_BOUND
        if self.subject:
            clear = min(
                clear, float(np.linalg.norm(p - self.subject.pos()) - 0.35 - R_BOUND)
            )
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
                          fd, float(bool(self.est and self.est[2])), 1.0 if self.phase in ("fly", "arrived") else 0.0])  # fmt: skip

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
    ]

    # ---- run + finish -----------------------------------------------------------------------------------------------------------------
    def crashed(self):
        p, tilt = self.flyer.last[0], self.flyer.last[3]
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
                if self.t - subject_done > 4.0:
                    break
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
            fd = col["follow_dist"][fly & (col["t"] > (self.t_fly0 or 0) + 8.0)]
            summ.update(follow_dist_median=float(np.median(fd)) if len(fd) else None, follow_dist_p5_p95=[float(np.percentile(fd, 5)), float(np.percentile(fd, 95))] if len(fd) else None,
                        subject_seen_frac=float(col["subject_seen"][fly].mean()) if fly.any() else 0.0)  # fmt: skip
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
    import scene

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
