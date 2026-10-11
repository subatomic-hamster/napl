"""Geometry, per-class tracking, size ranging and the ballistic lead solution of the cuas-lite app (spec section 5).

    geom = Geometry()                                  # base frame B: x forward at pan 0, y left, z up; pan CCW, tilt up (spec 5.1)
    pose = geom.pose(pan_deg, tilt_deg)                # camera + muzzle in B for a head attitude
    trk = ClassTracker(intrinsics)                     # napl.cuas.tracking.Tracker with a size prior per detector class and 5-consecutive confirmation
    tracks = trk.update([DetBlob.from_det(d) for d in dets], trk.frame(pose, t_cap))
    sol = lead(geom, tr, now, tracker_t, pan_deg, tilt_deg, wind)    # {az, el (deg), u, t_flight, ok, range} or None

napl.cuas.tracking.Tracker is used as is (its single `size_m` is switched per detection by the subclass, so the existing cuas showcase and its hashes
are untouched). Range comes from the apparent box size: range = f_px * size_m(class) / box_px, good to 10-25 % (spec section 0, item 2).
"""

import math
from dataclasses import dataclass

import numpy as np

from ..cuas import ballistics
from ..cuas.camera import Camera as PoseCamera
from ..cuas.camera import Frame as TrackFrame
from ..cuas.tracking import Blob, Tracker
from .io import TARGET_CLASSES, Intrinsics, Pose

# One BALL block: every consumer (this module, the twin, the sim scene) reads these. Spec section 0 item 1 / section 5.3: the Rival round
# (d = 22.2 mm, m = 1.86 g, secondary sources) with the brief's 17.8 mm / 0.95 g kept as an alternative set.
BALL = dict(d=0.0222, m=0.00186, cd=0.47)
BALL_BRIEF = dict(d=0.0178, m=0.00095, cd=0.47)
V0_NOMINAL = 30.0  # m/s [est]
V0_SIGMA, SPREAD_SIGMA = 1.5, 0.010  # m/s, rad per axis [est]
BALL_RADIUS = BALL["d"] / 2
CLASS_SIZE_M = {
    "balloon": 0.30,
    "target": 0.30,
    "drone": 0.30,
}  # [est] spec 5.3; a 0.55 m sim quad is configured per scenario
T_ACTUATOR, T_FIRE = (
    0.060,
    0.150,
)  # s [est] planner + step lag; FIRE command -> ball leaves
CONFIRM = 5  # consecutive detector replies
MIN_CONF = 0.5


def pellet(k_v=1.0, ball=None):
    b = ball or BALL
    return ballistics.Pellet(
        v0=k_v * V0_NOMINAL, mass=b["m"], diameter=b["d"], cd=b["cd"]
    )


def rot_head(pan_deg, tilt_deg):
    """Head frame -> B: Rz(pan) Ry(-tilt) (a positive tilt raises the muzzle)."""
    c, s, ct, st = (
        math.cos(math.radians(pan_deg)),
        math.sin(math.radians(pan_deg)),
        math.cos(math.radians(tilt_deg)),
        math.sin(math.radians(tilt_deg)),
    )
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    ry = np.array([[ct, 0, -st], [0, 1, 0], [st, 0, ct]])  # Ry(-tilt)
    return rz @ ry


@dataclass
class Geometry:
    """Head geometry in B (spec 5.1). Defaults are the CAD estimates; the sim harness measures them off the scene (sim_io.measure_geometry); on
    hardware they come from calib.toml. cam_boresight = (d_az, d_el) deg: the camera-to-barrel angular offset from the boresight calibration."""

    h_t: float = 0.12  # tilt axis height above the base plate
    l_m: float = 0.15  # muzzle ahead of the tilt axis
    d_b: float = 0.0  # muzzle above the tilt axis
    l_c: float = 0.05  # camera ahead of the tilt axis [est]
    d_c: float = 0.06  # camera above the tilt axis (Brio on top of the barrel housing)
    cam_boresight: tuple = (0.0, 0.0)

    def pose(self, pan_deg, tilt_deg) -> Pose:
        r = rot_head(pan_deg, tilt_deg)
        o = np.array([0.0, 0.0, self.h_t])
        rc = (
            r @ rot_head(*(-x for x in self.cam_boresight))
            if any(self.cam_boresight)
            else r
        )
        return Pose(o + r @ np.array([self.l_c, 0.0, self.d_c]), rc @ np.array([1.0, 0, 0]), rc @ np.array([0, 0, 1.0]),
                    o + r @ np.array([self.l_m, 0.0, self.d_b]), r @ np.array([1.0, 0, 0]), pan_deg, tilt_deg)  # fmt: skip


def pose_camera(k: Intrinsics) -> PoseCamera:
    """napl.cuas.camera.Camera used only for its pixel <-> ray geometry (no sim): vertical FOV from fy."""
    return PoseCamera(None, "", 2 * math.atan(0.5 * k.h / k.fy), k.w, k.h)


@dataclass
class DetBlob(Blob):
    cls: str = "drone"
    conf: float = 1.0
    size_px: float = (
        0.0  # the box's larger side, subpixel (Blob.size counts whole pixels)
    )

    @property
    def size(self):
        return self.size_px or super().size

    @classmethod
    def from_det(cls, d):
        x0, y0, x1, y1 = (float(v) for v in d.xyxy)
        return cls(
            (int(x0), int(y0), int(x1), int(y1)),
            ((x0 + x1) / 2, (y0 + y1) / 2),
            int((x1 - x0) * (y1 - y0)),
            d.cls,
            d.conf,
            max(x1 - x0, y1 - y0),
        )


class ClassTracker(Tracker):
    """napl.cuas.tracking.Tracker with a size prior per detector class and confirmation = CONFIRM consecutive replies. Each Track gets .cls and .conf
    (from its latest detection) and .streak."""

    def __init__(self, k: Intrinsics, sizes=None, confirm=CONFIRM, **kw):
        super().__init__(pose_camera(k), confirm=confirm, **kw)
        self.sizes_m = dict(CLASS_SIZE_M, **(sizes or {}))
        self.manual_range = {}  # track id -> metres

    @staticmethod
    def frame(pose: Pose, t) -> TrackFrame:
        return TrackFrame(None, None, pose.cam_pos, pose.cam_fwd, pose.cam_up, t)

    def _measure(self, tr, b, fr):
        self.size_m = self.sizes_m.get(getattr(b, "cls", "drone"), self.size_m)
        out = super()._measure(tr, b, fr)
        tr.cls, tr.conf = getattr(b, "cls", "drone"), getattr(b, "conf", 1.0)
        m = self.manual_range.get(tr.id)
        tr.manual_range = m
        if m:  # operator-entered range (Q2): overrides size ranging for this track
            tr.range_m = m
            out = (fr.pos + out[1] * m, out[1])
        return out

    def set_range(self, track_id, metres):
        """Manual range for a track (None / 0 clears it)."""
        if metres and metres > 0:
            self.manual_range[track_id] = float(metres)
        else:
            self.manual_range.pop(track_id, None)

    def update(self, blobs, fr):
        out = super().update(blobs, fr)
        for t in out:
            t.streak = (getattr(t, "streak", 0) + 1) if t.misses == 0 else 0
            t.confirmed = (
                t.streak >= self.confirm
                and getattr(t, "cls", "") in TARGET_CLASSES
                and getattr(t, "conf", 0.0) >= MIN_CONF
            )
        return out


def bearing(pos, origin=(0.0, 0.0, 0.0)):
    """(az, el) deg and range of a point in B seen from `origin`."""
    d = np.asarray(pos, float) - np.asarray(origin, float)
    r = float(np.linalg.norm(d))
    return (
        math.degrees(math.atan2(d[1], d[0])),
        math.degrees(math.asin(max(-1.0, min(1.0, d[2] / max(r, 1e-9))))),
        r,
    )


def lead(
    geom: Geometry, tr, dt_extra, pan_deg, tilt_deg, wind=(0.0, 0.0, 0.0), k_v=1.0
):
    """Lead solution for track `tr` (world = B position / velocity from the tracker, valid at the tracker's last update) for a shot `dt_extra` s later
    (frame age + T_ACTUATOR + T_FIRE, caller's choice). The solver is run from the muzzle at the current head attitude, then once more from the
    muzzle at the solved attitude (the muzzle sits l_m ahead of the tilt axis, so it moves with pan / tilt).
    -> dict(pan, tilt (deg), u, t_flight, ok, range, speed) or None when the target is out of the ball's reach."""
    pel = pellet(k_v)
    pos = np.asarray(tr.pos, float) + np.asarray(tr.vel, float) * dt_extra
    m = geom.pose(pan_deg, tilt_deg).muzzle
    sol = None
    for _ in range(2):
        sol = ballistics.solve_lead(pel, m, pos, tr.vel, wind)
        if sol is None:
            return None
        m = geom.pose(math.degrees(sol["az"]), math.degrees(sol["el"])).muzzle
    return dict(pan=math.degrees(sol["az"]), tilt=math.degrees(sol["el"]), u=sol["u"], t_flight=sol["t_flight"], ok=sol["ok"], speed=sol["speed"],
                range=float(np.linalg.norm(pos - m)), intercept=sol["intercept"])  # fmt: skip
