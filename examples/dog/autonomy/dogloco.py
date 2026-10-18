"""SP5 on the dog: the Locomotion door over SP4's controller. Builds the dog scene with its lidar kept (SP4's scenes strip it),
runs SP4's 500 Hz loop (Controller.step + sim.step(2)), hands SP5 the odometry of SP4's estimator with SP5's own fixes taken
back out (FixLedger), forwards SP5's pose fixes, and serves SP5's elevation map to SP4 as its HeightMap.

The carrier (carrier.py) stays the default body and the reference; this adapter only reads SP4's public names (Controller, Hw,
Truth, dogcourse), never edits them."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from napl.terrain import add_field

from . import carrier, config, lidar
from .iface import Odom
from .localize import rz, wrap

TRUNK = "deck_top"
SELF_M = 0.5  # m: sky-view rays that hit within this are the dog's own body (the antenna is inside the trunk's geometry)


def devices(xml_text):
    d = json.loads(
        ET.fromstring(xml_text).find("custom/text[@name='napl.devices']").get("data")
    )
    lid = next(r for r in d["rangefinders"] if r.get("projection") == "rosette")
    return lid, d["gnss_receivers"][0]


def cfg_from_devices(cfg, lid, gn):
    return config.override(cfg, **{
        "gnss.lat0": gn["lat0"], "gnss.lon0": gn["lon0"], "gnss.alt0": gn.get("alt0", 0.0), "gnss.rate_hz": gn["rate_hz"],
        "gnss.latency_s": gn.get("latency_s", 0.0), "gnss.horiz_std": gn["horiz_std"], "gnss.vert_std": gn["vert_std"],
        "lidar.fov_v_min": lid["fov_v_min"], "lidar.fov_v_max": lid["fov_v_max"], "lidar.rate_hz": lid["rate_hz"],
        "lidar.pattern_seed": lid.get("pattern_seed", 0),
    })  # fmt: skip


def build(f, auto, seed, start, out):
    import dogcourse  # SP4 harness (examples/dog on sys.path)

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    nominal = dogcourse.base_model()
    root = ET.parse(nominal).getroot()
    t = root.find("custom/text[@name='napl.devices']")
    d = json.loads(t.get("data"))
    for r in d["rangefinders"]:
        if r.get("projection") == "rosette":
            r["rays_per_frame"] = int(auto["rays"])
    t.set("data", json.dumps(d))
    wb = root.find("worldbody")
    for g in [g for g in wb.findall("geom") if g.get("type") == "plane"]:
        wb.remove(g)  # the field is the floor now (as dogcourse.scene)
    add_field(root, f)
    x, y, yaw = start
    u, v = np.meshgrid(
        np.linspace(-dogcourse.FOOTPRINT[0] / 2, dogcourse.FOOTPRINT[0] / 2, 7),
        np.linspace(-dogcourse.FOOTPRINT[1] / 2, dogcourse.FOOTPRINT[1] / 2, 5),
    )
    c, s = math.cos(yaw), math.sin(yaw)
    top = float(np.max(f.height(x + c * u - s * v, y + s * u + c * v)))
    z0 = float(
        wb.find(f"body[@name='{TRUNK}']").get("pos").split()[2]
    )  # nominal standing height above z = 0
    dogcourse._place(
        root, float(x), float(y), float(yaw), z0 + top + dogcourse.START_CLEAR
    )
    path = out / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    (out / "course.json").write_text(
        json.dumps(
            {
                "course": f"sp5-{auto['kind']}",
                "seed": int(seed),
                "nominal": str(nominal),
            }
        )
    )
    return path


def _yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


class FixLedger:
    """SP4's estimator absorbs SP5's fixes (position and yaw jumps). SP5's EKF must not see its own corrections as motion:
    odometry = SP4's increments with the jumps removed, rotated back by the accumulated yaw jump."""

    def __init__(self):
        self.p, self.prev, self.jp, self.jyaw = None, None, np.zeros(3), 0.0

    def odom(self, p_est, R_est):
        if self.p is None:
            self.p = np.array(p_est, float)
        else:
            self.p = self.p + rz(-self.jyaw) @ (p_est - self.prev - self.jp)
        self.prev, self.jp = np.array(p_est, float), np.zeros(3)
        return self.p.copy(), wrap(_yaw(R_est) - self.jyaw)

    def jumped(self, p0, R0, p1, R1):
        self.jp += np.asarray(p1) - np.asarray(p0)
        self.jyaw += wrap(_yaw(R1) - _yaw(R0))


class DogLoco:
    def __init__(self, sim, model, cfg, f, start, z_ground0, leg_ctl=None):
        import dogcourse
        import dogtruth
        from loco import controller as Ct
        from loco import hw as H
        from loco import model as M

        xml = Path(model).read_text()
        lid, gn = devices(xml)
        self.sim, self.cfg, self.ctl_dt = sim, cfg, Ct.DT
        self.dt = Ct.DT / 2  # physics step: two per control tick
        self.lidar, self.gnss, self.base = lid["name"], gn["name"], f"/world/{TRUNK}"
        self.lever = lidar.extrinsic(xml, gn["site"], TRUNK)[0]
        self.period = int(round(1.0 / (lid["rate_hz"] * self.dt)))
        self.emap, self.z0, self.cmd = None, float(z_ground0), (0.0, 0.0, 0.0)
        self.fill = self.z0  # ground height handed to SP4 where SP5's map has no cell (under and behind the feet at the start)
        self.wall_m = float(cfg["dog"]["wall_m"])
        self.n_nan = 0  # height queries the map could not answer (filled with `fill`)
        self.ctl = Ct.Controller(
            M.from_build(dogcourse.info(model)["nominal"]),
            H.Hw(sim),
            self,
            leg_ctl=leg_ctl or ("wbc" if cfg["dog"]["leg_ctl"] else "jt"),
        )
        self.ctl.set_pose(np.asarray(start[:2], float), float(start[2]))
        q = self.ctl.hw.joints()[
            0
        ]  # start the estimator now (what Controller.step does on its first tick), so the first odometry
        self.ctl.hw.enable()  # snapshot is already at the start pose and SP5 sees no jump
        self.ctl.est.reset(q, *self.ctl.pose0)
        self.ctl.q0, self.ctl.started = q.copy(), True
        self.tr = dogtruth.Truth(sim, f)
        self.ledger = FixLedger()
        self.snap = None
        self.gnss_lost, self.bumps, self.distance, self.time_of_last_frame, self._xy = (
            False,
            0,
            0.0,
            0.0,
            None,
        )

    def attach(self, emap):
        self.emap = emap

    def height(self, x, y):  # SP4's HeightMap, served from SP5's map
        h = (
            np.full(np.shape(x), np.nan)
            if self.emap is None
            else np.asarray(self.emap.height(x, y), float)
        )
        bad = np.isnan(h)
        if self.wall_m:  # a wall or boulder is an obstacle, not terrain to step on: SP5 plans around it, SP4's footholds never see it
            bad |= h > self.fill + self.wall_m
        if bad.any():
            self.n_nan += int(bad.sum())
            h = np.where(bad, self.fill, h)
        return h

    def _est(self):
        return np.array(self.ctl.est.x[:3], float), np.array(self.ctl.est.R, float)

    def command(self, vx, vy, wz):
        lim = self.limits()
        self.cmd = (
            float(np.clip(vx, -lim["v_max"], lim["v_max"])),
            float(np.clip(vy, -lim["v_max"] / 2, lim["v_max"] / 2)),
            float(np.clip(wz, -lim["wz_max"], lim["wz_max"])),
        )

    def limits(self):
        D = self.cfg[
            "dog"
        ]  # decision 8: what the legged robot is allowed, not the carrier's 1.5 m/s
        return {"v_max": D["v_max"], "wz_max": D["wz_max"]}

    def truth(self):
        return self.tr.pos(), self.tr.quat()

    @property
    def fallen(
        self,
    ):  # harness column (ground truth); the robot sees ctl.mode == "FALLEN" through Odom.fallen
        return bool(self.tr.fallen())

    def advance(self, seconds):
        for _ in range(int(round(seconds / self.ctl_dt))):
            if (
                self.sim.tick % self.period == 0
            ):  # the lidar / GNSS sampling tick: snapshot the estimate the frame belongs to
                self.time_of_last_frame = self.sim.tick * self.dt
                self.snap = self._est()
                lost = carrier.sky_blocked(
                    self.sim,
                    self.tr.pos() + lidar.qmat(self.tr.quat()) @ self.lever,
                    self.cfg,
                    SELF_M,
                )
                if lost != self.gnss_lost:
                    self.sim.set(
                        f"/devices/{self.gnss}", "fix_lost", 1.0 if lost else 0.0
                    )
                    self.gnss_lost = lost
                if (
                    self.emap is not None
                ):  # ground under the dog, for cells the map has not seen
                    g = float(np.asarray(self.emap.height(*self.snap[0][:2])))
                    if math.isfinite(g):
                        self.fill = g
            self.ctl.step(self.cmd)
            self.sim.step(2)
            if self.ctl.mode == "STAND" and self.tr.clear0 is None:
                self.tr.arm_fall()  # the harness's fall test needs the standing clearance (as dogwalk.tick)
        xy = self.tr.pos()[:2]
        if self._xy is not None:
            self.distance += float(np.hypot(*(xy - self._xy)))
        self._xy = xy

    def odometry(self):
        p, R = self.snap if self.snap is not None else self._est()
        po, yaw = self.ledger.odom(p, R)
        q = lidar.quat_zyx(
            yaw, -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[2, 1], R[2, 2])
        )
        return Odom(
            self.time_of_last_frame,
            po,
            q,
            np.zeros(3),
            np.zeros(3),
            self.ctl.mode == "FALLEN",
        )

    def fix(self, p_xy, yaw, t_pose, sig_p=None, sig_yaw=None):
        if self.ctl.mode in ("STANDUP", "FALLEN", "SIT"):
            return
        kw = (
            {}
            if sig_p is None
            else {
                "sig_p": max(float(sig_p), 0.02),
                "sig_yaw": max(float(sig_yaw), math.radians(1.0)),
            }
        )
        p0, R0 = self._est()
        self.ctl.fix(
            np.asarray(p_xy, float), float(yaw), self.sim.tick * self.dt - t_pose, **kw
        )
        p1, R1 = self._est()
        self.ledger.jumped(p0, R0, p1, R1)
