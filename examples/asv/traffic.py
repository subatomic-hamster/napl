"""Scripted traffic vessels: a vessel of assemblies/asv-traffic.toml follows its route with the same heading / speed loops as the ASV (asv.control), steering on
its true state (the traffic is the scenario's script, not a system under test), and obeys its `Bot` behaviour (asv.encounters): keep course, or give way to the ASV
by turning to starboard when it comes within `r_act`, keeping the turn until the ASV is clear and then returning to its route.
"""

import math

import numpy as np

from .control import Autopilot, Gains, Route
from .geo import wrap, yaw_of_quat


class TrafficVessel:
    def __init__(self, spec, dt_ctrl):
        self.spec = spec
        self.dt = dt_ctrl
        g = Gains()
        self.route = Route(spec.route, speed=spec.speed, gains=g)
        self.ap = Autopilot(g)
        self.turning = self.done = False
        self.turn_psi = 0.0
        self.prev = None
        self.v = np.zeros(2)
        self.r = 0.0

    def step(self, pos, quat, asv_pos):
        """Truth in (position xy, quaternion), the ASV's true position (the scripted vessel 'sees' it) -> (throttle_port, throttle_stbd)."""
        psi = yaw_of_quat(quat)
        if self.prev is not None:
            self.v = (np.asarray(pos[:2]) - self.prev[0]) / self.dt
            self.r = wrap(psi - self.prev[1]) / self.dt
        self.prev = (np.asarray(pos[:2], float).copy(), psi)
        b = self.spec.bot
        rng = float(np.hypot(*(np.asarray(asv_pos[:2]) - np.asarray(pos[:2]))))
        rt = self.route.update(pos[:2])
        psi_cmd = rt["psi"]
        u_cmd = rt["u"]
        if b.kind in ("give_way", "head_on") and b.complies:
            if not self.turning and not self.done and rng < b.r_act:
                self.turning = True
                self.turn_psi = wrap(rt["alpha"] - math.radians(b.turn_deg))
            if self.turning:
                rel = np.asarray(asv_pos[:2]) - np.asarray(pos[:2])
                ahead = float(
                    rel @ np.array([math.cos(rt["alpha"]), math.sin(rt["alpha"])])
                )
                if rng > 1.15 * b.r_act or (
                    ahead < -5.0 and rng > 8.0
                ):  # the ASV is clear astern: return to the route
                    self.turning, self.done = False, True
                else:
                    psi_cmd = self.turn_psi
        speed = float(np.hypot(*self.v))
        u = float(self.v @ np.array([math.cos(psi), math.sin(psi)]))
        return self.ap.step(
            psi_cmd, u_cmd, psi, self.r, u if speed > 0 else 0.0, self.dt
        )
