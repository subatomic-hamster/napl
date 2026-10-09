"""A companion computer's MAVLink link to a PX4 autopilot (SITL through napl-px4-bridge, or a real vehicle): what a vision-navigation drone sends
and receives besides the pilot's commands.

    link = Link("udp://:14540")                       # MAVSDK (BSD-3), the same connection string a mission program uses (python/napl/px4.py)
    link.odometry(t_s, p_enu, q_enu_flu, v_enu, sig)  # ODOMETRY -> PX4 EKF2 external vision fusion (EKF2_EV_CTRL), GPS-denied flight
    link.obstacle_distance(t_s, pts_flu)              # OBSTACLE_DISTANCE -> PX4 collision prevention (CP_DIST > 0) / the obstacle_distance topic
    link.setpoint(p_enu, v_enu, a_enu, yaw_enu)       # offboard position + velocity + acceleration set-point (the planner's output)

The pure encoders (`odometry_fields`, `obstacle_distance_fields`) take napl's frames (world ENU z up, body FLU) and return the MAVLink field dict
(field names of common.xml) that `MavlinkDirect` sends; they have no dependency and are what tests/python/test_companion.py checks. Frames: PX4 is NED /
FRD, converted exactly as src/px4/frames.h does for the bridge (identity napl attitude = nose East = yaw +90 deg in NED).

What PX4 v1.17.0 does with them (source read in the PX4 tree the bridge uses, not guessed): mavlink_receiver handle_message_odometry -> vehicle_visual_odometry
(estimator types UNKNOWN / NAIVE / VISION / VIO) -> EKF2 external vision (position / velocity / yaw per EKF2_EV_CTRL); handle_message_obstacle_distance ->
obstacle_distance, consumed by lib/collision_prevention, which only the Position-mode stick task (FlightTaskManualPosition) calls: in OFFBOARD mode
the avoidance is the companion planner's job, the message is still delivered (px4.log shows the topic)."""

import math

import numpy as np

# MAV_FRAME / MAV_ESTIMATOR_TYPE values used (common.xml)
FRAME_LOCAL_NED, FRAME_BODY_FRD = 1, 12
ESTIMATOR_VIO = 3
SECTORS, SECTOR_DEG = (
    72,
    5,
)  # OBSTACLE_DISTANCE: 72 bins of 5 deg (the field is uint16[72])
UNKNOWN = 65535  # UINT16_MAX = "not used / unknown" for a bin; max_distance_cm + 1 = "no obstacle in range"


def enu_to_ned(v):
    v = np.asarray(v, float)
    return np.array([v[1], v[0], -v[2]])


def q_ned_frd(q):
    """Unit quaternion (w, x, y, z) of body FLU -> world ENU to body FRD -> world NED, w >= 0 (src/px4/frames.h q_ned_frd)."""
    s = math.sqrt(0.5)

    def mul(a, b):
        return np.array([a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                         a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]])  # fmt: skip

    out = mul(
        mul(np.array([0.0, s, s, 0.0]), np.asarray(q, float)),
        np.array([0.0, 1.0, 0.0, 0.0]),
    )
    return -out if out[0] < 0 else out


def _cov21(d):
    """Upper triangle (row-major, 21 entries) of a 6 x 6 covariance with diagonal `d`."""
    c = np.full(21, 0.0)
    for k, i in enumerate(
        (0, 6, 11, 15, 18, 20)
    ):  # the diagonal entries of the packed upper triangle
        c[i] = d[k]
    return c


def odometry_fields(
    t_s,
    p_enu,
    q_enu_flu,
    v_enu,
    w_flu=(0, 0, 0),
    sigma=(0.1, 0.1, 0.1, 0.05),
    quality=100,
    reset_counter=0,
):
    """ODOMETRY (VIO): pose and velocity in the local NED frame whose origin is the estimator's (`p_enu` = the position relative to it), orientation
    body FRD -> NED. sigma = (position, velocity, roll / pitch, yaw) 1-sigma, written to the covariance diagonals (EKF2_EV_NOISE_MD 0 reads them;
    1 uses the EKF2_EV*_NOISE parameters instead)."""
    p, v, q = enu_to_ned(p_enu), enu_to_ned(v_enu), q_ned_frd(q_enu_flu)
    sp, sv, srp, sy = sigma
    return dict(
        time_usec=int(round(t_s * 1e6)), frame_id=FRAME_LOCAL_NED, child_frame_id=FRAME_LOCAL_NED,  # the velocity is in the local NED axes too
        x=float(p[0]), y=float(p[1]), z=float(p[2]), q=[float(c) for c in q], vx=float(v[0]), vy=float(v[1]), vz=float(v[2]),
        rollspeed=float(w_flu[0]), pitchspeed=-float(w_flu[1]), yawspeed=-float(w_flu[2]),
        pose_covariance=_cov21([sp**2] * 3 + [srp**2, srp**2, sy**2]).tolist(), velocity_covariance=_cov21([sv**2] * 3 + [0, 0, 0]).tolist(),
        reset_counter=int(reset_counter), estimator_type=ESTIMATOR_VIO, quality=int(quality),
    )  # fmt: skip


def obstacle_distance_fields(t_s, pts_flu, band=(-0.6, 0.6), min_m=0.28, max_m=10.0):
    """OBSTACLE_DISTANCE from 3D points in the body FLU frame (a depth image back-projected with the camera's mounting): the horizontal range
    of the nearest point of each 5 deg bearing bin among those within `band` metres of the vehicle's own height (the plane PX4's collision prevention
    works in; the floor and the canopy are outside it). Body-frame message, bin 0 = straight ahead, bearings clockwise seen from above (FRD).
    Bins the sensor cannot see stay UNKNOWN (65535, i.e. not "clear"); a bin the sensor sees with nothing in range reads max_distance + 1 cm."""
    pts = np.asarray(pts_flu, float).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    rng = np.hypot(pts[:, 0], pts[:, 1])
    k = (np.degrees(np.arctan2(-pts[:, 1], pts[:, 0])) % 360.0 // SECTOR_DEG).astype(
        int
    ) % SECTORS  # FRD bearing: y right = -y FLU
    d = np.full(SECTORS, UNKNOWN, np.int64)
    d[np.unique(k)] = (
        int(max_m * 100) + 1
    )  # the sensor has a point in this bin (any height): it looks there, nothing in range yet
    sel = (
        (pts[:, 2] >= band[0])
        & (pts[:, 2] <= band[1])
        & (rng >= min_m)
        & (rng <= max_m)
    )
    for b, r in zip(k[sel], rng[sel]):
        d[b] = min(d[b], int(round(r * 100)))
    return dict(
        time_usec=int(round(t_s * 1e6)), sensor_type=0, distances=[int(x) for x in d], increment=SECTOR_DEG, min_distance=int(min_m * 100),
        max_distance=int(max_m * 100), increment_f=float(SECTOR_DEG), angle_offset=0.0, frame=FRAME_BODY_FRD,
    )  # fmt: skip


class Link:
    """MAVSDK connection to the autopilot: raw message sending (ODOMETRY, OBSTACLE_DISTANCE), offboard set-points, arming and the health /
    position telemetry the flight sequence waits on."""

    def __init__(self, url, timeout_s=90.0):
        import json

        from mavsdk import Configuration, Mavsdk
        from mavsdk.component_type import ComponentType
        from mavsdk.plugins.action import Action
        from mavsdk.plugins.mavlink_direct import MavlinkDirect
        from mavsdk.plugins.offboard import Offboard
        from mavsdk.plugins.telemetry import Telemetry

        self._json = json
        cfg = Configuration.create_with_component_type(ComponentType.COMPANION_COMPUTER)
        cfg.system_id = 245  # not the autopilot's own id (PX4 SITL instance n is system n + 1): MAVSDK would otherwise share system 1 with instance 0
        self.mv = Mavsdk(cfg)
        self.mv.add_any_connection(url)
        self.system = self.mv.first_autopilot(timeout_s)
        if self.system is None:
            raise RuntimeError(f"no autopilot on {url} after {timeout_s} s")
        self.act, self.off, self.tel = (
            Action(self.system),
            Offboard(self.system),
            Telemetry(self.system),
        )
        self.raw = MavlinkDirect(self.system)

    def send(self, name, fields):
        from mavsdk.plugins.mavlink_direct import MavlinkMessage

        self.raw.send_message(
            MavlinkMessage(name, 0, 0, 0, 0, self._json.dumps(fields))
        )  # ids 0: MAVSDK fills in our system / component

    def odometry(self, t_s, p_enu, q_enu_flu, v_enu, **kw):
        self.send("ODOMETRY", odometry_fields(t_s, p_enu, q_enu_flu, v_enu, **kw))

    def obstacle_distance(self, t_s, pts_flu, **kw):
        self.send("OBSTACLE_DISTANCE", obstacle_distance_fields(t_s, pts_flu, **kw))

    def setpoint(self, p_ned, v_ned, a_ned, yaw_deg):
        """Offboard position + velocity + acceleration (NED) and heading, one message."""
        from mavsdk.plugins.offboard import (
            AccelerationNed,
            PositionNedYaw,
            VelocityNedYaw,
        )

        self.off.set_position_velocity_acceleration_ned(
            PositionNedYaw(*(float(x) for x in p_ned), float(yaw_deg)),
            VelocityNedYaw(*(float(x) for x in v_ned), float(yaw_deg)),
            AccelerationNed(*(float(x) for x in a_ned)),
        )

    def destroy(self):
        self.mv.destroy()
