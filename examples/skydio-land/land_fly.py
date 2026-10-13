"""The Skydio landing program: the original autonomy (examples/skydio/fly.py: depth -> voxel map -> sampled-primitive planner -> acceleration feed-forward to the
napl-native controller, ESKF on IMU + VIO + barometer) extended with a landing phase. Imported, not copied: `SkydioLand` subclasses `fly.Skydio`.

    python examples/skydio-land/land_fly.py <scenario.scenario.toml> <out dir>        # standalone (NAPL_PARAMS='{"seed": 7}' picks the seed)
    napl program examples/skydio-land/forest/clearing.scenario.toml                    # the runner builds the model (land_scene.py) and judges trace.csv

MISSION. The program is told the zones (land.json `mission`: designated landing zone, an alternate, home = the take-off pad: surveyed pad coordinates). Everything
else comes from sensors: it never reads the ground truth in land.json `truth` (evaluation: clearance, hazards, safe zones, touchdown monitors).

PHASES. takeoff -> map (1 s hover, the first depth frames build the map) -> fly (planner, obstacle avoidance, to the zone) -> scan (turn 360 deg at the survey
height: the depth map of the surroundings) -> survey (a lawn-mower of the downward lidar over the zone at `survey_agl`) -> assess (land.py: slope, flatness,
clear ring, no drop-off, no obstacle in the map; the best spot within 1.5 m of the zone centre) -> settle -> descend (lidar-governed sink, slower near the ground)
-> touchdown (lidar says on the ground: motors off) -> landed. A zone that fails is rejected with its reason and the next one is flown to; if every zone fails the
vehicle goes home and lands there without a survey (the take-off pad is known to hold it).

DENSITY. The controller's thrust tables are calibrated for the site's air (`calibrate_at`: the scene's origin_alt, ISA density from the mission's field elevation);
the integral term takes the rest. At 3200 m the air has 73 % of the sea-level density: more throttle for the same weight, less margin.

ARTEFACTS (run dir): trace.csv (62.5 Hz: truth pose + monitors' columns), telemetry.jsonl (planner replans), depth.npz (every 5th depth frame), summary.json,
replay.naplog + native.json (the runner's replay hash check), land.json copy (scene.json beside the model)."""

import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "skydio"))
import land as L
import land_scene
import land_world as W
from estimator import log_so3
from fly import (
    ARRIVE_M,
    MODES,
    R_BOUND,
    TAKEOFF_V,
    EstFlyer,
    Skydio,
    quat_R,
    wrap,
    yaw_of,
)
from mapping import VoxelMap
from napl.cuas import flyer as fl
from planner import GoalField, goal_objective

GEAR_H = 0.217  # m: the root plate's height above the gear bottom at rest (the X500's legs; tests/python/test_skydio_land.py checks it)
CRUISE_H = 2.0  # m: plate height above the ground in transit (the original's z_cruise)
PHASES = (
    "takeoff",
    "map",
    "fly",
    "scan",
    "survey",
    "settle",
    "descend",
    "touchdown",
    "landed",
)
T_MAX = 300.0
SCAN_RATE = 0.9  # rad/s of the 360 degree look-around over the zone
DEPTH_EVERY = 5  # every 5th depth frame goes into depth.npz
SETTLE_S = 1.0  # hold above the chosen spot before sinking
HOME_RADIUS = 2.0
DIST_ZONE_M = 5.0  # m: a landing counts as "at a zone" within this of a truth-safe zone's centre (VIO drift is 0.8 % of the path: 3-4 m after a 250 s flight)


def calibrate_at(assembly, workdir, origin_alt):
    """cuas.flyer.calibrate with the air of the site: the vehicle is built with the environment's origin_alt = `origin_alt` (thrust and torque follow the density)."""
    import xml.etree.ElementTree as ET

    orig = fl.design.compute

    def compute(*a, **k):
        text, report = orig(*a, **k)
        root = ET.fromstring(text)
        txt = root.find("custom/text")
        d = json.loads(txt.get("data"))
        for e in d.get("environments", []):
            e["origin_alt"] = float(origin_alt)
        txt.set("data", json.dumps(d, sort_keys=True, separators=(",", ":")))
        return ET.tostring(root, encoding="unicode"), report

    fl.design.compute = compute
    try:
        return fl.calibrate(assembly, workdir)
    finally:
        fl.design.compute = orig


class SkydioLand(Skydio):
    def __init__(self, model, run_dir, seed, cal_dir=None, params=None, lp=None):
        mdir = Path(model).parent
        self.land = json.loads((mdir / "land.json").read_text())
        self.lp = lp or L.LandParams()
        super().__init__(
            model, run_dir, "goal", seed, cal_dir, params
        )  # the original's scene objects; its forest and goal are replaced below
        self.mission = "land"
        man = json.loads((mdir / "scene.json").read_text())
        self.world = W.load_world(self.land["truth"])  # ground truth: evaluation only
        self.forest = self.world  # Skydio._telemetry reads forest.clearance
        self.zones = self.land["mission"]
        self.zi = 0
        self.goal = np.array([self.zones[0]["x"], self.zones[0]["y"]])
        self.use_rtk, self.follow_ground = (
            self.land["use_rtk"],
            self.land["terrain_following"],
        )
        q, root = self.sim.query, self.root
        lid = man["body_paths"]["skydio/lidar"]
        self.lidar_p_root = q(lid, "pos", root)[0]
        self.lidar_axis_root = quat_R(q(lid, "quat", root)[0]) @ np.array(
            [0.0, 0.0, 1.0]
        )  # the part's +z is the viewing direction
        self.lidar_dev, self.rtk_dev = "/devices/skydio_lidar", "/devices/skydio_rtk"
        self.rtk_cfg = man["device_cfg"]["skydio_rtk"]
        self.lidar_n, self.vr_rest, self._rest = 0, None, []
        self.agl, self._agl_buf = (
            None,
            [],
        )  # gear height above the ground below, from the lidar (median of 5)
        self.ground = (
            float(self.p_start[2]) - GEAR_H
        )  # ground height under the take-off pad (the pad is surveyed)
        cfg = self.land["cfg"]
        self.cruise = float(
            cfg.get("cruise_h", CRUISE_H)
        )  # m: plate height above the ground in transit
        self.band_hi = float(
            cfg.get("band_hi", 3.4)
        )  # m: top of the planner's altitude band above the ground reference
        self.z_c = self.ground + self.cruise
        self.gmap = L.GroundMap()
        self.surveying = False
        self.assessments, self.rejected = [], []
        self.spot = self.spot_z = None
        self.sp = self.p_start.copy()
        self.sp_v = np.zeros(3)
        self.wps, self.wi = [], 0
        self.scan_yaw0 = 0.0
        self.t_phase_ground = None
        self.force_home = False
        self.n_rtk, self.rtk_last = 0, None
        self.gusts = sorted(self.land.get("gusts", []), key=lambda g: g["t"])
        z_lo, z_hi = (
            float(cfg.get("map_z_lo", self.ground - 0.4)),
            float(cfg.get("map_z_hi", self.ground + 19.6)),
        )  # the mission area's height range (a priori, from the map; 20 m for flat sites)
        self.map = VoxelMap(
            z0=z_lo,
            n_z=int(math.ceil((z_hi - z_lo) / 0.2)),
            hit_hold=int(os.environ.get("SKYDIO_HIT_HOLD", 12)),
        )
        # truth bookkeeping (evaluation only)
        self.airborne = False
        self.n_obst = self.n_crash = self.n_ground = 0
        self.td = None
        self.vz_hist = []
        self.t_cut = self.t_landed = None
        self.t_stable = None

    # ---- the original's three hooks ------------------------------------------------------------------------------------------------------
    def _flyer(self, paths, cal_dir):
        alt = float(self.land["origin_alt"])
        work = Path(cal_dir or self.run_dir / "cal") / f"alt{int(round(alt, -2))}"
        return EstFlyer(
            self.sim,
            calibrate_at("skydio-land", work, alt),
            paths,
            self.dtc,
            max_tilt=0.45,
        )

    # ---- one control period ---------------------------------------------------------------------------------------------------------------
    def step(self):
        while self.gusts and self.gusts[0]["t"] <= self.t:
            g = self.gusts.pop(0)
            self.sim.set(
                "/devices/wind", "gust", [g["amp"], g["H"], *g["dir"], 0.0, g["speed"]]
            )  # amp m/s, length H m, direction, delay s, front speed m/s
        super().step()
        self._lidar()
        if self.use_rtk:
            self._rtk()
        self._truth_contacts()

    def _lidar(self):
        """The downward lidar: gear height above the ground (zeroed on the pad), ground points for the survey's map, the terrain-following ground reference."""
        q = self.sim.query
        n = int(q(self.lidar_dev, "frames")[0][0])
        if n == self.lidar_n or self.flyer.last is None:
            return
        self.lidar_n = n
        r = float(q(self.lidar_dev, "range")[0][0])
        p, _, R, _, _ = self.flyer.last
        o, d = p + R @ self.lidar_p_root, R @ self.lidar_axis_root
        vr = -d[2]  # cosine between the beam and the downward vertical
        if vr < 0.8:  # tilted more than 37 deg: no vertical reading
            return
        if r <= 0.0:
            self.agl = None
            self._agl_buf.clear()
            if self.surveying:
                self.gmap.add_void(o[0], o[1])
            return
        if self.vr_rest is None:
            if self.phase == "takeoff" and self.t < 0.4:
                self._rest.append(r * vr)
                if len(self._rest) >= 8:
                    self.vr_rest = float(np.median(self._rest))
            return
        self._agl_buf = (self._agl_buf + [r * vr - self.vr_rest])[-5:]
        self.agl = float(np.median(self._agl_buf))
        hit = o + r * d
        if self.surveying:
            self.gmap.add(hit[0], hit[1], hit[2])
        if self.follow_ground and self.phase in ("fly", "map", "scan"):
            self.ground += (
                0.02 * (float(hit[2]) - self.ground)
            )  # ~0.5 s at the lidar's 100 Hz: follows the terrain, ignores a boulder passed over

    def _rtk(self):
        """RTK rover fix (10 Hz, delayed): a position measurement for the filter, gated at 3 m (a jump is a fault)."""
        q, c = self.sim.query, self.rtk_cfg
        fix = tuple(float(q(self.rtk_dev, a)[0][0]) for a in ("lat", "lon", "alt"))
        if fix == self.rtk_last or not any(fix):
            return
        self.rtk_last = fix
        z = np.array(
            L.geodetic_to_enu(*fix, c["lat0"], c["lon0"], self.land["origin_alt"])
        )  # (the scene sets alt0 = origin_alt)
        if np.linalg.norm(z - self.eskf.p) < 3.0:
            L.update_position(self.eskf, self.eskf.t - c["latency_s"], z, 0.05)
            self.n_rtk += 1

    # ---- phases -----------------------------------------------------------------------------------------------------------------------------
    def _enter(self, phase):
        super()._enter(phase)
        self.surveying = phase in ("scan", "survey")
        if (
            phase == "scan"
        ):  # the virtual set-point starts where the vehicle is, at rest
            self.sp, self.sp_v = np.array(self.flyer.last[0], float), np.zeros(3)

    def _setpoint(self):
        if self.flyer.last is None:
            return
        getattr(self, "_ph_" + self.phase)(*self.flyer.last[:3])

    def _ph_takeoff(self, p, v, R):
        t = self.t
        z = min(self.z_c, self.p_start[2] + TAKEOFF_V * max(t - 0.4, 0.0))
        self.flyer.target(
            [self.p_start[0], self.p_start[1], z],
            [0, 0, TAKEOFF_V if z < self.z_c else 0.0],
            0.0,
        )
        if (
            z >= self.z_c
            and p[2] > self.z_c - 0.12
            and abs(v[2]) < 0.3
            and self.vr_rest is not None
        ):
            self._enter("map")

    def _ph_map(self, p, v, R):
        self.flyer.target(
            [self.p_start[0], self.p_start[1], self.z_c], np.zeros(3), self.yaw_sp
        )
        if self.t - self.t_phase >= 1.0:
            self._enter("fly")
            self.t_fly0 = self.t
            self.plan = None

    def _ph_fly(self, p, v, R):
        if self.plan is None:
            self.flyer.target(p, np.zeros(3), self.yaw_sp)
            return
        pp, vv, aa = self.plan.at(self.t - self.plan_t)
        self._yaw_update()
        self._command(pp, vv, aa)

    def _move_sp(self, wp, vmax, amax=1.5):
        """Virtual set-point toward wp with an acceleration-limited, stop-at-the-end speed profile -> flyer targets (position + velocity feed-forward)."""
        d = np.asarray(wp, float) - self.sp
        n = float(np.linalg.norm(d))
        vt = d / n * min(vmax, math.sqrt(2.0 * amax * n)) if n > 1e-6 else np.zeros(3)
        dv = vt - self.sp_v
        m, lim = float(np.linalg.norm(dv)), amax * self.dtc
        self.sp_v = self.sp_v + (dv if m <= lim else dv / m * lim)
        self.sp = self.sp + self.sp_v * self.dtc
        self.flyer.target(self.sp, self.sp_v, self.yaw_sp)
        return n

    def _z_survey(self):
        return self.ground + GEAR_H + self.lp.survey_agl

    def _turn_to(self, yaw, rate=1.0):
        self.yaw_sp = wrap(
            self.yaw_sp
            + float(np.clip(wrap(yaw - self.yaw_sp), -rate * self.dtc, rate * self.dtc))
        )

    def _ph_scan(self, p, v, R):
        z = self.zones[self.zi]
        c = np.array([z["x"], z["y"], self._z_survey()])
        self._move_sp(c, 1.5)
        if (
            np.linalg.norm(p[:2] - c[:2]) > 1.5 or abs(p[2] - c[2]) > 0.6
        ):  # not over the zone yet: no turning
            self.t_phase = self.t
            self.scan_yaw0 = self.yaw_sp
            return
        self.yaw_sp = wrap(self.yaw_sp + SCAN_RATE * self.dtc)
        if SCAN_RATE * (self.t - self.t_phase) >= 2 * math.pi:
            self._start_survey()

    def _start_survey(self):
        lp, z = self.lp, self.zones[self.zi]
        c = np.array([z["x"], z["y"]])
        u = c - self.p_start[:2]
        u = (
            u / max(np.linalg.norm(u), 1e-6)
            if np.linalg.norm(u) > 1.0
            else np.array([1.0, 0.0])
        )
        n = np.array([-u[1], u[0]])
        zs, wps = self._z_survey(), []
        win = self.map.window(
            np.array([c[0], c[1], zs]),
            half=9.0,
            z_lo=zs - 1.0,
            z_hi=zs + 1.0,
        )  # what the look-around mapped: legs stop short of anything in the way
        for i in range(lp.n_legs):
            off = (i - (lp.n_legs - 1) / 2) * lp.leg_gap
            a, b = c + n * off - u * lp.leg_half, c + n * off + u * lp.leg_half
            leg = L.trim_leg(win, a, b, zs, self.prm.r_safe)
            if leg is not None:
                for e in leg if i % 2 == 0 else leg[::-1]:
                    wps.append(np.array([e[0], e[1], zs]))
        if len(wps) < 6:  # fewer than three legs are clear: the zone is hemmed in
            self.assessments.append(
                dict(
                    t=round(self.t, 2),
                    zone=self.zones[self.zi]["name"],
                    ok=False,
                    reason="obstructed",
                    legs=len(wps) // 2,
                )
            )
            self.rejected.append(self.zones[self.zi]["name"])
            return self._next_zone()
        self.wps, self.wi, self.survey_yaw = wps, 0, math.atan2(u[1], u[0])
        self.gmap.clear()
        self._enter("survey")

    def _ph_survey(self, p, v, R):
        self._turn_to(self.survey_yaw)
        wp = self.wps[self.wi]
        n = self._move_sp(wp, self.lp.survey_speed if self.wi > 0 else 1.5, 2.0)
        if (
            n < 0.02
            and np.linalg.norm(self.sp_v) < 0.05
            and np.linalg.norm(p - wp) < 0.4
            and abs(wrap(self.yaw_sp - yaw_of(R))) < 0.3
        ):
            self.wi += 1
            if self.wi >= len(self.wps):
                self._assess()

    def _assess(self):
        z = self.zones[self.zi]
        self.surveying = False
        occ = self.map.occupied_points((z["x"], z["y"]), 9.0)
        a = self.gmap.assess((z["x"], z["y"]), self.lp, occ)
        rec = dict(
            t=round(self.t, 2),
            zone=z["name"],
            **a.brief(),
            n_points=len(self.gmap.pts),
            n_void=len(self.gmap.void),
        )
        self.assessments.append(rec)
        if a.ok:
            self.spot, self.spot_z = np.array([a.x, a.y]), a.z
            self.sp_v[:] = 0.0
            self._enter("settle")
        else:
            self.rejected.append(z["name"])
            self._next_zone()

    def _next_zone(self):
        self.zi += 1
        if self.zi >= len(self.zones):  # nothing left: home, and down without a survey
            home = next(
                (i for i, z in enumerate(self.zones) if z["name"] == "home"),
                len(self.zones) - 1,
            )
            self.zi, self.force_home = home, True
        z = self.zones[self.zi]
        self.goal = np.array([z["x"], z["y"]])
        self._enter("fly")
        self.plan = None
        self.t_fly0 = self.t

    def _ph_settle(self, p, v, R):
        self._turn_to(self.survey_yaw)
        tgt = np.array([self.spot[0], self.spot[1], self._z_survey()])
        n = self._move_sp(tgt, 1.0)
        if n < 0.03 and np.linalg.norm(p - tgt) < 0.3 and np.linalg.norm(v) < 0.25:
            if self.t_stable is None:
                self.t_stable = self.t
            if self.t - self.t_stable >= SETTLE_S:
                self.t_stable = None
                self.sp = np.array(p, float)
                self._enter("descend")
        else:
            self.t_stable = None

    def _height(self, p):
        """Gear height above the ground below: the lidar's, else dead-reckoned from the plane the survey found."""
        if self.agl is not None:
            return self.agl
        return (
            float(p[2])
            - GEAR_H
            - (self.spot_z if self.spot_z is not None else self.ground)
        )

    def _ph_descend(self, p, v, R):
        lp = self.lp
        h = self._height(p)
        vs = L.sink_speed(h, lp)
        self.sp = np.array(
            [self.spot[0], self.spot[1], max(self.sp[2] - vs * self.dtc, p[2] - 0.35)]
        )
        self.flyer.target(self.sp, [0.0, 0.0, -vs], self.survey_yaw)
        low = self._agl_buf and np.mean(self._agl_buf) < lp.touch_agl
        stalled = h < 0.25 and v[2] > -0.05 and self.t - self.t_phase > 1.0
        if low or stalled:
            if self.t_stable is None:
                self.t_stable = self.t
            if self.t - self.t_stable >= 0.1:
                self.flyer.cut()
                self.t_cut = self.t
                self._enter("touchdown")
        else:
            self.t_stable = None

    def _ph_touchdown(self, p, v, R):
        if np.linalg.norm(v) < 0.1 and self.t - self.t_cut > 0.5:
            self.t_stable = self.t_stable or self.t
            if self.t - self.t_stable >= 1.0:
                self._enter("landed")
                self.t_landed = self.t
        else:
            self.t_stable = None
        if self.t - self.t_cut > 6.0:
            self._enter("landed")
            self.t_landed = self.t

    def _ph_landed(self, p, v, R):
        pass

    # ---- sensing and planning ---------------------------------------------------------------------------------------------------------------
    def _process(self, img, cam_pos, cam_R, t_s, tcam):
        w0 = time.perf_counter()
        self.map.integrate(img, self.cam, cam_pos, cam_R)
        if self.map.frames % DEPTH_EVERY == 0:
            self.depth_log.append((t_s, img.astype(np.float16)))
        if self.phase == "fly":
            self._plan()
        self.plan_ms.append(1000 * (time.perf_counter() - w0))

    def _plan(self):
        p, v, R = self.flyer.last[0], self.flyer.last[1], self.flyer.last[2]
        g, prm = self.ground, self.prm
        hi = self.band_hi
        prm.z_min, prm.z_max, prm.z_cruise = (
            g + 0.9,
            g + hi,
            g + self.cruise,
        )  # the original's altitude band, above the ground reference
        win = self.map.window(p, half=10.0, z_lo=g + 0.4, z_hi=g + hi + 1.6)
        field = GoalField(win, self.goal, z_lo=g + 0.9, z_hi=g + hi)
        plan = self.planner.plan(
            p,
            v,
            yaw_of(R),
            win,
            field.bearing(p),
            goal_objective(self.goal, g + self.cruise, field),
        )
        self.modes[plan.mode] += 1
        self.plan, self.plan_t = plan, self.t
        self._telemetry(p, v, R, plan, win)
        if np.linalg.norm(self.goal - p[:2]) < ARRIVE_M and np.linalg.norm(v) < 1.0:
            if self.force_home:  # no zone passed: straight down at home
                z = self.zones[self.zi]
                self.spot, self.spot_z = np.array([z["x"], z["y"]]), self.ground
                self.survey_yaw = yaw_of(R)
                self.sp, self.sp_v = np.array(p, float), np.zeros(3)
                self._enter("descend")
            else:
                self._enter("scan")

    # ---- records (truth: evaluation only) ---------------------------------------------------------------------------------------------------
    def _truth_contacts(self):
        q = self.sim.query
        go, gg = (
            float(q(f"/sensors/{s}", "value")[0][0])
            for s in ("hit_obstacles", "hit_ground")
        )
        gear = sum(float(q(f"/sensors/hit_gear_{l}", "value")[0][0]) for l in "fr")
        pt, Rt, vt = self.flyer.truth[0], self.flyer.truth[1], self.flyer.truth[2][:3]
        if (
            not self.airborne
            and pt[2] - GEAR_H - self.world.ground_z(pt[0], pt[1]) > 0.3
        ):
            self.airborne = True
        self.vz_hist = (self.vz_hist + [float(vt[2])])[-3:]
        if go > 0:
            self.n_obst += 1
        if not self.airborne:
            return
        if gg > 0:
            self.n_ground += 1
        if max(gg - gear, 0.0) > 0:  # a part that is not a landing leg on the ground
            self.n_crash += 1
        if gear > 0 and self.td is None:
            z, d = self.world.nearest_safe_zone(pt[:2])
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, Rt[2, 2]))))
            on = min(
                self.world.zones, key=lambda zz: math.hypot(pt[0] - zz.x, pt[1] - zz.y)
            )
            self.td = dict(t=self.t, speed=-min(self.vz_hist), tilt_deg=tilt, dist_safe_m=d, zone=None if z is None else z.name,
                           nearest=on.name, on_safe=bool(on.safe and math.hypot(pt[0] - on.x, pt[1] - on.y) <= on.radius), safe=self.world.spot_safe(pt[0], pt[1]),
                           pos=np.round(pt, 3).tolist())  # fmt: skip

    COLS = ["t", "x", "y", "z", "qw", "qx", "qy", "qz", "speed", "clearance_m", "contact_steps", "dist_goal", "tilt_deg", "map_voxels", "planner_mode", "flying",
            "est_x", "est_y", "est_z", "est_err_m", "phase", "agl_m", "zone", "td_speed_ms", "td_tilt_deg", "td_dist_m", "landed", "crash", "ground_steps", "td_safe"]  # fmt: skip

    def _trace(self, p, R_true, v, tilt_cmd):
        tilt = math.acos(max(-1.0, min(1.0, R_true[2, 2])))
        pe, ve = self.eskf.p, self.eskf.v
        flying = self.phase not in ("takeoff", "landed") or self.t > 1.0
        if self.phase in ("fly", "scan", "survey", "settle", "descend"):
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
        agl = float(p[2]) - GEAR_H - self.world.ground_z(p[0], p[1])
        landing = self.phase in ("descend", "touchdown", "landed")
        clear = self.world.clearance(p) - R_BOUND
        if self.world.terrain is not None and not landing and self.airborne:
            clear = min(clear, self.world.terrain_clearance(p) - R_BOUND)
        self.min_clear = min(self.min_clear, clear)
        z = self.zones[self.zi]
        dg = float(np.linalg.norm(p[:2] - [z["x"], z["y"]]))
        if self.t_fly0 is not None and self.prev_p is not None:
            self.path_len += float(np.linalg.norm(p - self.prev_p))
        self.prev_p = p.copy()
        td = self.td
        self.rows.append([self.t, *p, *self.sim.query(self.root, "quat")[0], float(np.linalg.norm(v)), clear, self.n_obst, dg, math.degrees(tilt), self.map.n_occupied,
                          MODES.index(self.plan.mode) if self.plan else 0, 1.0 if self.airborne else 0.0, *pe, float(np.linalg.norm(p - pe)),
                          PHASES.index(self.phase), agl, self.zi, 0.0 if td is None else td["speed"], 0.0 if td is None else td["tilt_deg"],
                          99.0 if td is None else td["dist_safe_m"], 1.0 if self.phase == "landed" else 0.0, float(self.n_crash > 0), self.n_ground, 0.0 if td is None else float(td["safe"])])  # fmt: skip

    # ---- run + finish -----------------------------------------------------------------------------------------------------------------------
    def crashed(self):
        tilt = math.acos(max(-1.0, min(1.0, self.flyer.truth[1][2, 2])))
        return (
            self.n_crash > 40
            or self.n_obst > 40
            or (tilt > 1.4 and self.phase not in ("touchdown", "landed"))
        )

    def run(self, t_max):
        beat = 0
        while self.t < t_max:
            self.step()
            if self.t >= beat:  # a heartbeat on stderr every 20 simulated seconds
                beat += 20.0
                sys.stderr.write(
                    f"[land_fly] t={self.t:6.1f} s wall={time.monotonic() - self.wall0:6.1f} s phase={self.phase} zone={self.zi}\n"
                )
                sys.stderr.flush()
            if self.phase == "landed" and self.t - self.t_landed > 0.5:
                break
            if self.flyer.last is not None and self.crashed():
                break
        return self.finish()

    def finish(self):
        r = self.run_dir
        self.tele.close()
        with open(r / self.TRACE, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            w.writerows(self.rows)
        tr = np.array(self.rows)
        col = {c: tr[:, i] for i, c in enumerate(self.COLS)}
        d = (
            np.array([x[1] for x in self.depth_log])
            if self.depth_log
            else np.zeros((0, 1, 1))
        )
        np.savez_compressed(
            r / "depth.npz", t=np.array([x[0] for x in self.depth_log]), depth=d
        )
        td = self.td
        fin = tr[-1]
        z_fin, z_name = self.world.nearest_safe_zone(fin[1:3])
        ok = bool(self.phase == "landed" and td is not None and self.n_crash == 0 and self.n_obst == 0 and td["speed"] <= 1.0 and td["tilt_deg"] <= 12.0
                  and td["safe"] and td["dist_safe_m"] <= DIST_ZONE_M and col["tilt_deg"][-1] < 25.0)  # fmt: skip
        summ = dict(scenario=self.land["scenario"], family=self.land["family"], seed=self.seed, sim_s=self.t, wall_s=time.monotonic() - self.wall0,
                    landed=self.phase == "landed", success=ok, touchdown=td, crash_steps=int(self.n_crash), contact_steps=int(self.n_obst),
                    min_clearance_m=self.min_clear, path_length_m=self.path_len, zones_tried=[a["zone"] for a in self.assessments], rejected=self.rejected,
                    assessments=self.assessments, hazard=self.world.notes.get("hazard", ""), zones_truth=[z.__dict__ for z in self.world.zones],
                    final_xy=np.round(fin[1:3], 3).tolist(), final_tilt_deg=float(col["tilt_deg"][-1]), rtk_fixes=self.n_rtk, force_home=self.force_home,
                    planner=dict(n=len(self.plan_ms), mean_ms=float(np.mean(self.plan_ms)) if self.plan_ms else 0.0, max_ms=float(np.max(self.plan_ms)) if self.plan_ms else 0.0,
                                 modes=dict(self.modes)), max_tilt_deg=float(col["tilt_deg"].max()))  # fmt: skip
        if self.err:
            e = np.array(self.err)
            summ["estimation"] = dict(pos_rms_m=float(np.sqrt((e[:, 1] ** 2).mean())), pos_max_m=float(e[:, 1].max()), pos_final_m=float(e[-1, 1]),
                                      att_rms_deg=float(np.sqrt((e[:, 3] ** 2).mean())), att_max_deg=float(e[:, 3].max()))  # fmt: skip
        (r / "summary.json").write_text(json.dumps(summ, indent=1))
        self._seal(r)
        return summ


def main():
    scenario = Path(sys.argv[1]).resolve()
    cfg, prm, _ = land_scene.scenario_cfg(scenario)
    cal = os.environ.get("SKYDIO_CAL")
    if "NAPL_MODEL" in os.environ:  # under `napl program`
        model, out = os.environ["NAPL_MODEL"], Path(os.environ["NAPL_RUN_DIR"])
    else:  # standalone: land_fly.py <scenario> <out dir>
        out = Path(sys.argv[2])
        out.mkdir(parents=True, exist_ok=True)
        model = land_scene.build(out / "model", scenario)
    run = SkydioLand(model, out, prm["seed"], cal)
    s = run.run(float(os.environ.get("LAND_T_MAX", prm.get("t_max_s", T_MAX))))
    sys.stdout.write(json.dumps(s) + "\n")


if __name__ == "__main__":
    main()
