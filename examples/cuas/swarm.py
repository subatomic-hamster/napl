"""A seeded boids-like swarm: the programmed targets of the counter-UAS demo.

Each agent is a point with a position and velocity (the *virtual leader* of one drone); the drone's own controller (napl.cuas.flyer) tracks it, so
the real vehicle lags and wobbles around the agent like a real quad around its setpoint. Rules per agent, summed to an acceleration (clamped):

  separation   push away from neighbours closer than `sep_m`
  cohesion     weak pull to the flock centre (flock stays loosely together)
  alignment    match the neighbours' mean velocity
  wander       a seeded random-walk acceleration (so the paths are not a perfect orbit)
  arena        steer back inside the arena: 15 - 60 m from the turret, 5 - 25 m altitude, within +-AZ of the turret's +x axis (the arena the
               scenario promises the turret; outside it a drone would leave the camera's sweep)

Deterministic: all randomness from one numpy Generator seeded by `seed`; step(dt) is a pure function of the state.
"""

import math

import numpy as np

DEFAULT_SEED = 7


class Swarm:
    def __init__(self, names, seed=DEFAULT_SEED, turret_xy=(0.0, 0.0), rmin=15.0, rmax=52.0, zmin=5.0, zmax=25.0, az_max=math.radians(55),
                 vmax=5.0, amax=3.0, sep_m=8.0):  # fmt: skip
        self.names, self.rng = list(names), np.random.default_rng(seed)
        self.c = np.array(turret_xy, float)
        self.rmin, self.rmax, self.zmin, self.zmax, self.az_max = (
            rmin,
            rmax,
            zmin,
            zmax,
            az_max,
        )
        self.vmax, self.amax, self.sep = vmax, amax, sep_m
        n = len(self.names)
        # start on spread-out bearings inside the arena
        az = np.linspace(-0.8, 0.8, n) * az_max + self.rng.uniform(-0.1, 0.1, n)
        r = self.rng.uniform(25.0, 50.0, n)
        z = self.rng.uniform(8.0, 20.0, n)
        self.p = np.stack(
            [self.c[0] + r * np.cos(az), self.c[1] + r * np.sin(az), z], axis=1
        )
        th = self.rng.uniform(0, 2 * math.pi, n)
        self.v = (
            0.05  # start (nearly) at rest: the drones are placed at rest in the air
            * self.vmax
            * np.stack(
                [np.cos(th), np.sin(th), 0.15 * self.rng.standard_normal(n)], axis=1
            )
        )
        self.wander = np.zeros((n, 3))

    def start(self):
        """{name: initial position} (the scene builder places the drones there, in the air)."""
        return {n: self.p[i].copy() for i, n in enumerate(self.names)}

    def _arena(self, p):
        a = np.zeros(3)
        d = p[:2] - self.c
        r = float(np.hypot(*d))
        k = 1.2
        if r > self.rmax - 8:
            a[:2] -= k * (r - (self.rmax - 8)) * d / r
        if r < self.rmin + 8:
            a[:2] += k * ((self.rmin + 8) - r) * d / max(r, 1e-6)
        az = math.atan2(d[1], d[0])
        if abs(az) > self.az_max - 0.15:  # steer tangentially back toward the +x axis
            tang = np.array([-d[1], d[0]]) / max(r, 1e-6)
            a[:2] -= 6.0 * math.copysign(abs(az) - (self.az_max - 0.15), az) * tang
        if p[2] > self.zmax - 4:
            a[2] -= k * (p[2] - (self.zmax - 4))
        if p[2] < self.zmin + 4:
            a[2] += k * ((self.zmin + 4) - p[2])
        return a

    def step(self, dt, alive=None):
        """Advance every agent by dt; `alive` = names still flying (others are left where they are and ignored by the rules).
        -> {name: (position, velocity)}"""
        n = len(self.names)
        live = [i for i, nm in enumerate(self.names) if alive is None or nm in alive]
        self.wander += -self.wander * 0.5 * dt + self.rng.standard_normal(
            (n, 3)
        ) * np.array([1.0, 1.0, 0.3]) * math.sqrt(dt)
        acc = np.zeros((n, 3))
        for i in live:
            others = [j for j in live if j != i]
            a = self._arena(self.p[i]) + 1.2 * self.wander[i]
            sp = float(np.hypot(*self.v[i][:2]))
            if (
                sp < 0.4 * self.vmax
            ):  # a swarm keeps moving: accelerate along the heading
                a[:2] += 1.5 * (
                    self.v[i][:2] / sp if sp > 1e-6 else np.array([1.0, 0.0])
                )
            if others:
                c = self.p[others].mean(axis=0)
                a += 0.04 * (c - self.p[i]) + 0.3 * (
                    self.v[others].mean(axis=0) - self.v[i]
                )
                for j in others:
                    d = self.p[i] - self.p[j]
                    dist = float(np.linalg.norm(d))
                    if dist < self.sep:
                        a += 1.5 * (self.sep - dist) * d / max(dist, 1e-6)
            na = float(np.linalg.norm(a))
            acc[i] = a * min(1.0, self.amax / na) if na > 0 else a
        for i in live:
            self.v[i] += acc[i] * dt
            sp = float(np.linalg.norm(self.v[i][:2]))
            if sp > self.vmax:
                self.v[i][:2] *= self.vmax / sp
            self.v[i][2] = float(np.clip(self.v[i][2], -1.5, 1.5))
            self.p[i] += self.v[i] * dt
        return {
            nm: (self.p[i].copy(), self.v[i].copy()) for i, nm in enumerate(self.names)
        }
