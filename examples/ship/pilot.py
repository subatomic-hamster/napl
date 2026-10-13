"""Heading / speed autopilot and a dynamic-positioning-lite hold for the 12.6 m support vessel (assemblies/osv.toml), built on the ASV's estimator and loops.

The two Deep Blue 50i pods are the only actuators (port = esc1 at y > 0, starboard = esc2): common throttle drives the surge, differential throttle the yaw. The bare hull is
directionally unstable (Munk moment, see parts/marine/generic-osv-hull-1260.toml), so the heading loop is not optional. "DP-lite" = station keeping WITHOUT a lateral thruster: it
points the bow at the target point and moves ahead / astern along the heading to close the distance, so a cross-wind or cross-current offset is worked off by weathervaning and
repositioning, with a limit circle of a few metres, not by sideways thrust. Every input is an estimate (asv.estimator.Navigator on the vessel's Pixhawk IMU, magnetometer and M8N GNSS).

    pilot = ShipPilot(sim)
    pilot.set_route([(0, 0), (200, 0)], speed=2.5)      # or pilot.set_hold((x, y)) / pilot.set_heading(psi, speed)
    while ...: pilot.advance(sim)                        # 40 ms of simulation, estimator, control; pilot.est has the estimate
"""

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1])
)  # examples/ : the package `asv`

from asv.control import Autopilot, Gains, Route
from asv.estimator import Navigator
from asv.geo import wrap

DT = 0.001  # s, the timestep of assemblies/osv.toml (shared with the 1 ms landers of ship_scene)
IMU_TICKS = 8  # 125 Hz estimator input, as the ASV
CTRL_TICKS = 40  # 40 ms control period
MAG_REF = np.array(
    [1.294e-6, 2.1756e-5, -4.3027e-5]
)  # as the ASV (assemblies/asv.toml): the local field at the scene origin
# common throttle that holds a speed in calm water, straight (measured under the heading loop at 1..4 m/s: 0.183, 0.346, 0.516, 0.805); the PI term trims the rest
SPEED_THR = (
    (0.0, 0.0),
    (1.0, 0.183),
    (2.0, 0.346),
    (3.0, 0.516),
    (4.0, 0.805),
    (4.4, 1.0),
)


@dataclass
class ShipGains(Gains):
    """Loops of the 10 t vessel (yaw inertia ~1.5e5 kg m2 with added mass; one unit of differential throttle is about 1e4 N m)."""

    kp_psi: float = 2.0
    kd_psi: float = 6.0
    d_max: float = 0.35
    psi_rate: float = 0.12  # rad/s
    kp_u: float = 0.15
    ki_u: float = 0.03
    lookahead: float = 25.0
    lookahead_k: float = 2.0
    accept: float = 20.0  # m, wheel-over distance of a waypoint turn (the 90 deg turn of the 10 t boat needs ~25 m)
    slow_radius: float = 40.0
    u_final: float = 0.3
    turn_slow: float = 0.9


class _Auto(Autopilot):
    def feed_forward(self, u):
        return math.copysign(float(np.interp(abs(u), *zip(*SPEED_THR))), u)


class ShipPilot:
    def __init__(self, sim, gains: ShipGains | None = None, prefix="osv"):
        self.g = gains or ShipGains()
        self.p = prefix
        self.nav = Navigator(MAG_REF)
        self.ap = _Auto(self.g)
        self.route = None
        self.mode = "idle"
        self.t_ctrl = 0.3
        self.thr = (0.0, 0.0)
        self.cmd = dict(psi=0.0, u=0.0)
        self._last = None

    # -- commands -------------------------------------------------------------------------------------------------------------------
    def set_route(self, waypoints, speed=2.5):
        self.route, self.mode = Route(waypoints, speed, gains=self.g), "route"

    def set_heading(self, psi, speed):
        self.mode, self.cmd = "heading", dict(psi=psi, u=speed)

    def set_hold(self, target, heading=None):
        """Station keeping on (E, N) `target` m; `heading` (rad) = bow direction when inside the circle (None = keep the heading)."""
        self.mode, self.target, self.hold_heading = (
            "hold",
            np.asarray(target, float),
            heading,
        )

    # -- one control period ---------------------------------------------------------------------------------------------------------
    def advance(self, sim):
        p = self.p
        for _ in range(CTRL_TICKS // IMU_TICKS):
            sim.step(IMU_TICKS)
            self.nav.imu(
                _q(sim, f"/devices/{p}_imu", "gyro"),
                _q(sim, f"/devices/{p}_imu", "accel"),
                _q(sim, f"/devices/{p}_mag", "field"),
                IMU_TICKS * DT,
            )
        fix = (
            float(_q(sim, f"/devices/{p}_gnss", "lat")[0]),
            float(_q(sim, f"/devices/{p}_gnss", "lon")[0]),
        )
        if fix != self._last:
            self._last = fix
            self.nav.gnss(fix[0], fix[1], _q(sim, f"/devices/{p}_gnss", "vel_ned"))
        self.est = est = self.nav.state()
        t = sim.tick * DT
        if t < self.t_ctrl or not self.nav.have_fix or self.mode == "idle":
            return self.thr
        psi, u = est["psi"], 0.0
        if self.mode == "route":
            rt = self.route.update(est["pos"])
            psi, u = rt["psi"], (0.0 if rt["done"] else rt["u"])
            if rt["done"]:
                psi = est["psi"]
        elif self.mode == "heading":
            psi, u = self.cmd["psi"], self.cmd["u"]
        elif self.mode == "hold":
            d = self.target - est["pos"]
            dist = float(np.hypot(*d))
            bearing = math.atan2(d[1], d[0])
            if (
                dist > 3.0
            ):  # bow to the target at all times, ground speed proportional to the distance: a cross current is worked off by weathervaning (the boat settles down-stream of the target, thrusting up-stream)
                psi, u = bearing, min(2.0, 0.1 * dist)
            else:  # inside 3 m: keep (or take) the wanted heading, move along the bow axis to cancel the offset (astern if it is behind)
                psi = est["psi"] if self.hold_heading is None else self.hold_heading
                along = d[0] * math.cos(est["psi"]) + d[1] * math.sin(est["psi"])
                u = float(np.clip(0.15 * along, -0.4, 0.4))
        if (
            u > 0
        ):  # a twin-screw boat turns on the spot: no way on until the bow is within ~70 deg of the wanted course
            u *= float(
                np.clip(
                    1.2 - abs(wrap(psi - est["psi"])) / math.radians(60.0), 0.0, 1.0
                )
            )
        self.cmd = dict(psi=psi, u=u)
        self.thr = self.ap.step(psi, u, est["psi"], est["r"], est["u"], CTRL_TICKS * DT)
        sim.set(f"/devices/{p}_esc1", "throttle", float(self.thr[0]))
        sim.set(f"/devices/{p}_esc2", "throttle", float(self.thr[1]))
        return self.thr


def _q(sim, path, what):
    return np.asarray(sim.query(path, what)[0], float)


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


__all__ = ["ShipGains", "ShipPilot", "wrap", "yaw_of"]
