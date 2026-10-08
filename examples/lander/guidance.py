"""Landing-burn timing for a solid motor that cannot be throttled or shut down: when must it be lit?

Model (vertical axis, thrust along the vertical, tail-first quadratic drag): from the onset state (height z0 above the standing height, vertical speed v0)
at the instant the thrust curve starts (curve time tau = 0), integrate
    m dv/dt = F(tau) - m g - 0.5 rho CdA v |v|,     m = m0 - burned(tau)
until the feet touch (z = 0). `touchdown_speed` does it for a whole vector of onset heights at once (explicit Euler, dt = 2 ms), `ignition_table` finds for
each onset speed the lowest onset height that still touches down no faster than `v_target` (the knife edge: a few tenths of a metre higher the
vehicle is stopped above the ground and falls back, a few lower it hits hard), and `Planner` turns the table into the ignition rule used by flight.py.
`analytic_constant_thrust` is the closed-form solution the unit test compares against."""

import numpy as np
from vehicle import CDA_DOWN, RHO0, G, Motor

DT = 0.002


def touchdown_speed(
    motor: Motor,
    m0,
    z0,
    v0,
    cda=CDA_DOWN,
    scale=1.0,
    dt=DT,
    horizon=None,
    g=G,
    first_contact=False,
):
    """Vertical speed (m/s, negative = downwards) of the HARDEST ground contact for onset heights z0 (array), onset speed v0 (scalar or array): the
    floor is inelastic, so a vehicle that touches with thrust above its weight rises again and comes down after burnout; every touchdown is
    counted and the fastest one is what the legs have to take. `first_contact=True` returns the first contact instead (what the closed-form
    comparison in the tests needs). Vehicles that never reach the ground within `horizon` return nan."""
    z = np.atleast_1d(np.asarray(z0, float)).copy()
    v = np.broadcast_to(np.asarray(v0, float), z.shape).copy()
    out = np.where(z <= 0.0, v, np.nan)
    done = np.zeros(z.shape, bool)
    horizon = horizon or (motor.burn + 6.0)
    t = 0.0
    while t < horizon:
        F = scale * float(motor.thrust(t + 0.5 * dt))
        m = m0 - float(motor.burned(t + 0.5 * dt))
        a = F / m - g - 0.5 * RHO0 * cda * v * np.abs(v) / m
        vn = v + a * dt
        zn = z + 0.5 * (v + vn) * dt
        hit = (z > 0.0) & (zn <= 0.0) & (vn < 0.0) & ~done
        if hit.any():
            f = z[hit] / np.maximum(
                z[hit] - zn[hit], 1e-12
            )  # linear interpolation of the speed at z = 0
            vc = v[hit] + (vn[hit] - v[hit]) * f
            out[hit] = np.fmin(
                out[hit], vc
            )  # the hardest contact so far (most negative)
            if first_contact:
                done |= hit
        floor = zn <= 0.0
        zn = np.where(floor, 0.0, zn)
        vn = np.where(floor & (vn < 0.0), 0.0, vn)
        z, v = zn, vn
        t += dt
        if t > motor.burn and not (z > 0.0).any():
            break
    return out


def ignition_table(
    motor: Motor,
    m0,
    v_target=0.8,
    speeds=None,
    z_max=30.0,
    dz=0.02,
    slack=0.3,
    v_max=8.0,
    **kw,
):
    """-> (speeds (ascending, negative), onset heights): per onset speed, the middle of the lowest window of onset heights whose last-contact
    touchdown speed is within max(`v_target`, best + `slack`) m/s (the best the motor can do at that speed may be above the target). nan where
    even the best is faster than `v_max`: the motor cannot stop that speed."""
    speeds = np.arange(-30.0, 0.5, 1.0) if speeds is None else np.asarray(speeds)
    zs = np.arange(0.0, z_max, dz)
    heights = []
    for v0 in speeds:
        vt = np.nan_to_num(np.abs(touchdown_speed(motor, m0, zs, v0, **kw)), nan=1e9)
        best = float(vt.min())
        if best > v_max:
            heights.append(np.nan)
            continue
        ok = np.nonzero(vt <= max(v_target, best + slack))[0]
        gaps = np.nonzero(np.diff(ok) > 1)[0]
        run = ok[: gaps[0] + 1] if len(gaps) else ok  # the lowest contiguous window
        heights.append(0.5 * (zs[run[0]] + zs[run[-1]]))
    return speeds, np.array(heights)


def ballistic(z, v, t, cda=CDA_DOWN, m=1.0, g=G):
    """Free flight for t seconds with quadratic drag (explicit, 5 ms steps). -> z, v"""
    n = max(1, int(round(t / 0.005)))
    h = t / n
    for _ in range(n):
        a = -g - 0.5 * RHO0 * cda * v * abs(v) / m
        v2 = v + a * h
        z += 0.5 * (v + v2) * h
        v = v2
    return z, v


class Planner:
    """Ignition rule: fire when the height the vehicle will have when the thrust starts (`delay` s from now, ballistic) is at or below the table height for
    the speed it will have then, plus `aim_bias` (m; positive = earlier)."""

    def __init__(self, motor: Motor, m0, v_target=0.8, aim_bias=0.0, **kw):
        table_kw = {
            k: kw.pop(k) for k in ("speeds", "z_max", "dz", "slack", "v_max") if k in kw
        }
        self.motor, self.m0, self.kw = (
            motor,
            m0,
            kw,
        )  # kw: integrator options (dt, cda, g, horizon)
        self.speeds, self.heights = ignition_table(
            motor, m0, v_target, **table_kw, **kw
        )
        ok = ~np.isnan(self.heights)
        self.speeds, self.heights = self.speeds[ok], self.heights[ok]
        if not len(self.speeds):
            raise ValueError(
                f"{motor.id} at {m0:.3f} kg cannot land from any speed of the table"
            )
        self.aim_bias = aim_bias
        self.m_ball = m0

    def required_height(self, v_onset):
        return float(np.interp(v_onset, self.speeds, self.heights)) + self.aim_bias

    def onset(self, z, v, delay):
        return ballistic(z, v, delay, m=self.m_ball)

    def margin(self, z, v, delay):
        """Metres the predicted onset height is above the required one: <= 0 means light now."""
        zo, vo = self.onset(z, v, delay)
        return zo - self.required_height(vo)

    def predict_touchdown(self, z, v, delay, scale=1.0):
        """Touchdown speed (m/s) if the motor were lit right now."""
        zo, vo = self.onset(z, v, delay)
        if zo <= 0:
            return vo
        return float(
            touchdown_speed(self.motor, self.m0, [zo], vo, scale=scale, **self.kw)[0]
        )


def analytic_constant_thrust(v0, a_net):
    """Closed form for constant thrust T, mass m, no drag: net deceleration a = T/m - g > 0. Onset speed v0 < 0 (down) stops after
    t = |v0| / a at a height h = v0^2 / (2 a) above the point where the thrust started; returns (h, t)."""
    return v0 * v0 / (2.0 * a_net), abs(v0) / a_net
