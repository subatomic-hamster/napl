"""Point-mass pellet ballistics for the turret's fire-control: the same model as the kernel's launcher device (src/mujoco/devices/launcher.cpp),
written independently in numpy.

    a = g - k |v - w| (v - w),   k = rho(z) cd A / (2 m),   rho = ISA troposphere

`solve_lead` returns the aim direction that puts the pellet on a constant-velocity target, with drag and a wind estimate. It is the *turret's
belief*: the real pellet gets muzzle-velocity scatter, spread and the true (turbulent) wind, which is why the hit rate is not 100 %.
"""

import math
from dataclasses import dataclass

import numpy as np

G = 9.80665
T0, P0, R_AIR, LAPSE, ISA_EXP = 288.15, 101325.0, 287.052874, 0.0065, 5.25588


def isa_rho(h):
    h = min(max(h, -5000.0), 11000.0)
    t = T0 - LAPSE * h
    return P0 * (t / T0) ** ISA_EXP / (R_AIR * t)


@dataclass
class Pellet:
    v0: float = 200.0  # nominal muzzle velocity, m/s
    mass: float = 5.598e-4  # kg
    diameter: float = 4.51e-3  # m
    cd: float = 0.38
    alt0: float = 0.0  # environment origin altitude, m

    @property
    def area(self):
        return math.pi * self.diameter**2 / 4

    def k(self, z):
        return isa_rho(self.alt0 + z) * self.cd * self.area / (2 * self.mass)


def accel(p, pos, vel, wind):
    vr = vel - wind
    return np.array([0.0, 0.0, -G]) - p.k(pos[2]) * np.linalg.norm(vr) * vr


def flight(p, pos, vel, wind, t_max=3.0, dt=0.004):
    """RK4 trajectory -> (t[n], pos[n,3], vel[n,3]), n = steps + 1; stops at the ground (z < 0)."""
    pos, vel, wind = np.array(pos, float), np.array(vel, float), np.asarray(wind, float)
    ts, ps, vs = [0.0], [pos.copy()], [vel.copy()]
    t = 0.0
    while t < t_max and pos[2] > 0.0:
        k1v = accel(p, pos, vel, wind); k1p = vel  # fmt: skip
        k2v = accel(p, pos + 0.5 * dt * k1p, vel + 0.5 * dt * k1v, wind); k2p = vel + 0.5 * dt * k1v  # fmt: skip
        k3v = accel(p, pos + 0.5 * dt * k2p, vel + 0.5 * dt * k2v, wind); k3p = vel + 0.5 * dt * k2v  # fmt: skip
        k4v = accel(p, pos + dt * k3p, vel + dt * k3v, wind); k4p = vel + dt * k3v  # fmt: skip
        pos = pos + dt / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        vel = vel + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        t += dt
        ts.append(t); ps.append(pos.copy()); vs.append(vel.copy())  # fmt: skip
    return np.array(ts), np.array(ps), np.array(vs)


def _reach(p, muzzle, u, wind, point, dt):
    """Fly along u until the pellet's distance along the line muzzle -> point reaches |point - muzzle|.
    -> (t, pellet position, speed) or None if the pellet never gets there (ground / out of time)."""
    line = point - muzzle
    r = float(np.linalg.norm(line))
    ts, ps, vs = flight(p, muzzle, p.v0 * u, wind, t_max=r / (0.3 * p.v0) + 0.5, dt=dt)
    s = (ps - muzzle) @ (line / r)
    i = np.searchsorted(
        s, r
    )  # s increases monotonically (the pellet outruns its own drag by far)
    if i == 0 or i >= len(s):
        return None
    f = (r - s[i - 1]) / (s[i] - s[i - 1])
    return (
        ts[i - 1] + f * dt,
        ps[i - 1] + f * (ps[i] - ps[i - 1]),
        float(np.linalg.norm(vs[i])),
    )


def solve_lead(p, muzzle, tgt_pos, tgt_vel, wind, iters=12, dt=0.004, tol=0.02):
    """Aim for a constant-velocity target. -> dict(u (aim unit vector), az, el (rad), t_flight, intercept (3,), speed (impact), err (final
    miss of the solver's own prediction, m), ok) or None if the target cannot be reached. The muzzle is assumed static."""
    muzzle, tgt_pos, tgt_vel = (
        np.asarray(x, float) for x in (muzzle, tgt_pos, tgt_vel)
    )
    wind = np.asarray(wind, float)
    tf = float(np.linalg.norm(tgt_pos - muzzle)) / p.v0
    aim = tgt_pos.copy()
    for _ in range(iters):
        icpt = tgt_pos + tgt_vel * tf
        d = aim - muzzle
        u = d / np.linalg.norm(d)
        hit = _reach(p, muzzle, u, wind, icpt, dt)
        if hit is None:
            return None
        tf, q, speed = hit
        e = tgt_pos + tgt_vel * tf - q
        aim = (
            aim + e
        )  # shift the aim point by the miss, retry with the new time of flight
        if np.linalg.norm(e) < tol:
            break
    d = aim - muzzle
    u = d / np.linalg.norm(d)
    return dict(u=u, az=math.atan2(u[1], u[0]), el=math.asin(u[2]), t_flight=tf, intercept=tgt_pos + tgt_vel * tf,
                speed=speed, err=float(np.linalg.norm(e)), ok=bool(np.linalg.norm(e) < 4 * tol))  # fmt: skip
