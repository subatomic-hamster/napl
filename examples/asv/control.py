"""Guidance and control of the ASV: line-of-sight path following along a waypoint route, a heading PD loop and a speed PI loop, and the differential-thrust
mixer for the two bidirectional thrusters. All inputs are estimates (asv.estimator); nothing here reads the simulator.

Conventions: math yaw psi (counter-clockwise from East), positive turn rate to port (left). The thruster pair is port (+y) and starboard (-y): more thrust on the
starboard thruster turns the bow to port. Throttle -1..1 (negative = reverse on the bidirectional ESC).
"""

import math
from dataclasses import dataclass

import numpy as np

from .geo import wrap

T_FULL = 36.4  # N, one T200 at full forward throttle at 12 V (parts: 3.71 kgf)


@dataclass
class Gains:
    """Tuning of the loops. The defaults are the values the scenarios fly with (found in the simulator, see README)."""

    kp_psi: float = 1.5  # differential throttle per rad of heading error (heading std 2.3 deg in Hs 0.8 m waves; 0.55 gave 10 deg and loops, see README)
    kd_psi: float = 0.9  # differential throttle per rad/s of yaw rate
    d_max: float = 0.7  # largest differential throttle (each side)
    psi_rate: float = 0.7  # rad/s, largest rate of the heading reference
    kp_u: float = 0.25  # throttle per m/s of speed error
    ki_u: float = 0.08  # throttle per m/s per s
    u_ff_lin: float = 3.0  # N s/m, linear drag of the nominal model (both hulls, the sum used for the throttle feed-forward)
    u_ff_quad: float = 2.6  # N s2/m2, quadratic drag of the nominal model
    lookahead: float = 4.0  # m, LOS look-ahead distance
    lookahead_k: float = 1.7  # the look-ahead is at least this times the cross-track error (atan(1/1.7) = 30 degrees at most)
    accept: float = 3.0  # m, waypoint acceptance radius
    slow_radius: float = 8.0  # m, speed ramps down to `u_final` over this distance before the last waypoint
    u_final: float = 0.4  # m/s at the last waypoint
    turn_slow: float = 0.9  # speed is cut when the heading error is large: u *= max(0.3, cos(|err| x turn_slow))


class Route:
    """Waypoints (E, N) m, flown leg by leg with LOS guidance. `speed` m/s on every leg unless `speeds` (one per leg) is given."""

    def __init__(self, waypoints, speed=1.5, speeds=None, gains: Gains | None = None):
        self.wp = np.asarray(waypoints, float)
        self.speeds = (
            list(speeds) if speeds is not None else [speed] * (len(self.wp) - 1)
        )
        self.g = gains or Gains()
        self.k = 0  # leg index: from wp[k] to wp[k+1]
        self.done = False

    def update(self, pos):
        """-> dict(psi, xte, along, leg, dist, u, done): the LOS course, signed cross-track error (+ left of the leg), along-track distance on the leg,
        distance to the end waypoint, the nominal speed on this leg, and whether the route is finished."""
        g = self.g
        p = np.asarray(pos, float)
        while True:
            a, b = self.wp[self.k], self.wp[self.k + 1]
            d = b - a
            L = float(np.hypot(*d))
            alpha = math.atan2(d[1], d[0])
            c, s = math.cos(alpha), math.sin(alpha)
            r = p - a
            along = c * r[0] + s * r[1]
            xte = -s * r[0] + c * r[1]
            dist = float(np.hypot(*(b - p)))
            if (dist < g.accept or along > L) and self.k < len(self.wp) - 2:
                self.k += 1  # next leg
                continue
            last = self.k == len(self.wp) - 2
            if last and (dist < g.accept or (along > L and dist < 2 * g.accept)):
                self.done = True  # reached, or passed abeam of it within twice the radius (a position error of a few metres must not make it circle)
            break
        psi = alpha - math.atan2(
            xte, max(g.lookahead, g.lookahead_k * abs(xte))
        )  # the look-ahead grows with the error: approach angle below ~30 degrees
        u = self.speeds[self.k]
        if self.k == len(self.wp) - 2 and dist < g.slow_radius:
            u = min(u, g.u_final + (u - g.u_final) * dist / g.slow_radius)
        return dict(
            psi=psi,
            xte=xte,
            along=along,
            leg=self.k,
            dist=dist,
            u=u,
            done=self.done,
            alpha=alpha,
        )


class Autopilot:
    """Heading + speed loops with the thruster mixer. `step(psi_cmd, u_cmd, psi, r, u, dt)` -> (throttle_port, throttle_stbd)."""

    def __init__(self, gains: Gains | None = None):
        self.g = gains or Gains()
        self.psi_ref = None
        self.int_u = 0.0
        self.last = (0.0, 0.0)

    def reset(self, psi):
        self.psi_ref = psi
        self.int_u = 0.0

    def feed_forward(self, u):
        """Throttle that balances the nominal drag at speed u (both thrusters equal), from T = T_FULL thr^2."""
        g = self.g
        drag = g.u_ff_lin * abs(u) + g.u_ff_quad * u * abs(u)
        return math.copysign(math.sqrt(min(abs(drag) / (2 * T_FULL), 1.0)), drag)

    def step(self, psi_cmd, u_cmd, psi, r, u, dt):
        g = self.g
        if self.psi_ref is None:
            self.reset(psi)
        # heading reference: rate limited (a step in the wanted course becomes a ramp the boat can follow)
        err_ref = wrap(psi_cmd - self.psi_ref)
        self.psi_ref = wrap(
            self.psi_ref + max(-g.psi_rate * dt, min(g.psi_rate * dt, err_ref))
        )
        err = wrap(self.psi_ref - psi)
        d = max(-g.d_max, min(g.d_max, g.kp_psi * err - g.kd_psi * r))
        # speed: cut when the bow is far off the wanted course
        ue = u_cmd * max(
            0.3, math.cos(min(abs(wrap(psi_cmd - psi)) * g.turn_slow, 1.4))
        )
        e_u = ue - u
        self.int_u = max(-0.3, min(0.3, self.int_u + g.ki_u * e_u * dt))
        base = max(-1.0, min(1.0, self.feed_forward(ue) + g.kp_u * e_u + self.int_u))
        base = math.copysign(
            min(abs(base), 1.0 - abs(d)), base
        )  # keep the differential: shave the common part
        self.last = (base - d, base + d)
        return self.last
