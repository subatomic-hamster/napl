"""Foam-ball ballistics with drag and Magnus lift (docs/cuas2/spec.md 7.3): the same equations as the kernel launcher device
(src/mujoco/devices/launcher.cpp), written independently in numpy.

    a = g - k |v_rel| v_rel + a_M,     k = rho(z) cd A / (2 m),     v_rel = v - wind
    a_M = rho A C_L |v_rel|^2 / (2 m) * (w^ x v^_rel),    C_L = min(cl_k S, 0.25),  S = |w| r / |v_rel|,   w(t) = w0 exp(-t / spin_tau)

    t, pos, vel = fly(p0, v, omega)                       # RK4, 1 ms, one ball                    (the reference)
    pos = fly_batch(p0, v, omega, d, m, cd, t_eval)       # vectorised midpoint, 8 ms, n balls      (fire control)
    sol = solve_lead2(muzzle, tgt_pos, tgt_vel, v0=..., omega=...)    # v1's solve_lead fixed point plus the spin term

Spin is the vector `omega` (rad/s, world frame). For a launcher shooting along u with the head upright, `omega_y` about the lateral axis
up x u (the head's +y) is the scalar the launcher model returns: `omega_vec(u, omega_y)`. Negative = backspin = lift (ball moving +x, omega_y < 0).
"""

import math

import numpy as np

from ..cuas.ballistics import G
from .calib import Calib

CL_MAX = 0.25
UP = np.array([0.0, 0.0, 1.0])


def rho_at(z, alt0=0.0):
    """ISA density, vectorised (same troposphere as napl.cuas.ballistics.isa_rho)."""
    h = np.clip(alt0 + np.asarray(z, float), -5000.0, 11000.0)
    t = 288.15 - 0.0065 * h
    return 101325.0 * (t / 288.15) ** 5.25588 / (287.052874 * t)


def lateral_axis(u):
    """Unit lateral axis of the head for a barrel direction u: up x u (the head's +y when upright). Vertical shots: world +y."""
    a = np.cross(UP, np.asarray(u, float))
    n = np.linalg.norm(a)
    return a / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])


def omega_vec(u, omega_y):
    return omega_y * lateral_axis(u)


def _acc(pos, vel, t, wind, cdam, aam, r, w0, cl_k, tau, alt0):
    """pos, vel (..., 3); t scalar or (...,); cdam = cd A/(2m), aam = A/(2m) scalar or (...,); w0 (..., 3) spin at t = 0."""
    rho = rho_at(pos[..., 2], alt0)[..., None]
    vr = vel - wind
    vn = np.linalg.norm(vr, axis=-1, keepdims=True)
    a = np.array([0.0, 0.0, -G]) - rho * np.asarray(cdam)[..., None] * vn * vr
    if cl_k > 0:
        w = w0 * (np.exp(-np.asarray(t, float) / tau)[..., None] if tau > 0 else 1.0)
        wn = np.linalg.norm(w, axis=-1, keepdims=True)
        ok = (wn > 1e-12) & (vn > 1e-9)
        cl = np.minimum(
            cl_k * wn * np.asarray(r)[..., None] / np.where(vn > 1e-9, vn, 1.0), CL_MAX
        )
        c = np.where(
            ok,
            rho * np.asarray(aam)[..., None] * cl * vn / np.where(wn > 1e-12, wn, 1.0),
            0.0,
        )
        a = a + c * np.cross(w, vr)
    return a


def _ball(ball):
    if isinstance(ball, Calib):
        return ball.ball.d_mm / 1000.0, ball.ball.m_g / 1000.0, ball.ball.cd
    if isinstance(ball, dict):
        return ball["d"], ball["m"], ball["cd"]
    return ball


def fly(
    p0,
    v0,
    omega=None,
    ball=None,
    *,
    cl_k=0.75,
    spin_tau=6.0,
    wind=(0, 0, 0),
    alt0=0.0,
    dt=0.001,
    t_max=3.0,
    stop_x=None,
):
    """RK4 flight of one ball -> (t[n], pos[n,3], vel[n,3]); stops at the ground (z < 0), at t_max, or once x >= stop_x (last point is past it).
    `ball` = (d, m, cd) in SI, a dict(d, m, cd), or a Calib (default: the Calib defaults). omega: world spin vector rad/s (None = no spin)."""
    d, m, cd = _ball(ball if ball is not None else Calib())
    area = math.pi * d * d / 4
    cdam, aam, r = cd * area / (2 * m), area / (2 * m), d / 2
    wind = np.asarray(wind, float)
    w0 = np.zeros(3) if omega is None else np.asarray(omega, float)
    f = lambda t, p, v: _acc(p, v, t, wind, cdam, aam, r, w0, cl_k, spin_tau, alt0)
    p, v = np.array(p0, float), np.array(v0, float)
    ts, ps, vs = [0.0], [p.copy()], [v.copy()]
    t = 0.0
    while t < t_max and p[2] > 0.0 and (stop_x is None or p[0] < stop_x):
        k1v = f(t, p, v); k1p = v  # fmt: skip
        k2v = f(t + dt / 2, p + dt / 2 * k1p, v + dt / 2 * k1v); k2p = v + dt / 2 * k1v  # fmt: skip
        k3v = f(t + dt / 2, p + dt / 2 * k2p, v + dt / 2 * k2v); k3p = v + dt / 2 * k2v  # fmt: skip
        k4v = f(t + dt, p + dt * k3p, v + dt * k3v); k4p = v + dt * k3v  # fmt: skip
        p = p + dt / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        v = v + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        t += dt
        ts.append(t); ps.append(p.copy()); vs.append(v.copy())  # fmt: skip
    return np.array(ts), np.array(ps), np.array(vs)


def fly_batch(
    p0,
    v0,
    omega,
    d,
    m,
    cd,
    t_eval,
    *,
    cl_k=0.75,
    spin_tau=6.0,
    wind=(0, 0, 0),
    alt0=0.0,
    dt=0.008,
):
    """n balls at once, explicit midpoint with ~dt steps -> positions (n, 3) at t_eval (scalar or (n,)). p0 (3,) or (n,3), v0 (n,3), omega (n,3) or
    None, d, m, cd (n,) or scalars. Every ball gets the same number of steps (step = t_eval / steps), so balls with different t_eval stay vectorised."""
    v = np.array(v0, float)
    n = len(v)
    p = np.broadcast_to(np.asarray(p0, float), (n, 3)).copy()
    d, m, cd = (np.broadcast_to(np.asarray(x, float), (n,)) for x in (d, m, cd))
    area = math.pi * d * d / 4
    cdam, aam, r = cd * area / (2 * m), area / (2 * m), d / 2
    w0 = (
        np.zeros((n, 3))
        if omega is None
        else np.broadcast_to(np.asarray(omega, float), (n, 3))
    )
    te = np.broadcast_to(np.asarray(t_eval, float), (n,))
    wind = np.asarray(wind, float)
    steps = max(1, math.ceil(float(te.max()) / dt))
    h = te / steps
    t = np.zeros(n)
    f = lambda tt, pp, vv: _acc(
        pp, vv, tt, wind, cdam, aam, r, w0, cl_k, spin_tau, alt0
    )
    for _ in range(steps):
        a1 = f(t, p, v)
        pm, vm = p + 0.5 * h[:, None] * v, v + 0.5 * h[:, None] * a1
        a2 = f(t + 0.5 * h, pm, vm)
        p = p + h[:, None] * vm
        v = v + h[:, None] * a2
        t = t + h
    return p


def impact_height(calib, v0, omega_y, range_m, cd=None, cl_k=None, dt=0.002, ball=None):
    """z (m, relative to the muzzle) where a ball fired along +x from the origin crosses x = range_m; spin about +y (omega_y rad/s). Used by the
    bench fitter. `ball` = (d, m, cd) SI overrides the calibration's mean ball. NaN if it never gets there."""
    d, m, cd0 = _ball(ball if ball is not None else calib)
    ball = (d, m, cd0 if cd is None else cd)
    lo = calib.launcher
    z0 = 10.0  # high enough that the ball never meets the ground (fly stops at z < 0); the density at 10 m differs from the muzzle's by 0.1 %
    t, p, _ = fly([0, 0, z0], [v0, 0, 0], [0, omega_y, 0], ball, cl_k=lo.cl_k if cl_k is None else cl_k, spin_tau=lo.spin_tau_s, dt=dt, t_max=4.0, stop_x=range_m)  # fmt: skip
    if p[-1, 0] < range_m:
        return math.nan
    i = np.searchsorted(p[:, 0], range_m)
    f = (range_m - p[i - 1, 0]) / (p[i, 0] - p[i - 1, 0])
    return float(p[i - 1, 2] + f * (p[i, 2] - p[i - 1, 2])) - z0


def _reach(muzzle, u, v0, omega_y, ball, kw, wind, point, dt):
    line = point - muzzle
    r = float(np.linalg.norm(line))
    ts, ps, vs = fly(
        muzzle,
        v0 * u,
        omega_vec(u, omega_y),
        ball,
        wind=wind,
        dt=dt,
        t_max=r / (0.3 * v0) + 0.5,
        **kw,
    )
    s = (ps - muzzle) @ (line / r)
    i = np.searchsorted(s, r)
    if i == 0 or i >= len(s):
        return None
    f = (r - s[i - 1]) / (s[i] - s[i - 1])
    return (
        ts[i - 1] + f * dt,
        ps[i - 1] + f * (ps[i] - ps[i - 1]),
        float(np.linalg.norm(vs[i])),
    )


def solve_lead2(muzzle, tgt_pos, tgt_vel, *, v0=None, omega_y=0.0, model=None, setpoint=None, ball=None, cl_k=None, spin_tau=None, wind=(0, 0, 0),
                iters=12, dt=0.004, tol=0.02, alt0=0.0):  # fmt: skip
    """v1's solve_lead fixed point (aim shifted by the miss, retried with the new flight time) with the spin term. Either v0 / omega_y explicit
    (m/s, rad/s about the lateral axis) or `model` = a LauncherModel and `setpoint` (rpm mean or (top, bottom)): v0 = model.mean_v0(setpoint, with_dip=True),
    omega_y = model.spin_rad_s(...). -> dict(u, az, el, t_flight, intercept, speed, err, ok) or None if unreachable."""
    cal = model.c if model is not None else Calib()
    if model is not None:
        rt, rb = model._rpms(setpoint, None)
        v0 = model.mean_v0((rt, rb), with_dip=True) if v0 is None else v0
        omega_y = model.spin_rad_s(rt, rb)
    if v0 is None:
        raise ValueError("solve_lead2: give v0 or a model")
    kw = dict(
        cl_k=cal.launcher.cl_k if cl_k is None else cl_k,
        spin_tau=cal.launcher.spin_tau_s if spin_tau is None else spin_tau,
        alt0=alt0,
    )
    ball = _ball(ball if ball is not None else cal)
    muzzle, tgt_pos, tgt_vel, wind = (
        np.asarray(x, float) for x in (muzzle, tgt_pos, tgt_vel, wind)
    )
    tf = float(np.linalg.norm(tgt_pos - muzzle)) / v0
    aim = tgt_pos.copy()
    for _ in range(iters):
        icpt = tgt_pos + tgt_vel * tf
        dd = aim - muzzle
        u = dd / np.linalg.norm(dd)
        hit = _reach(muzzle, u, v0, omega_y, ball, kw, wind, icpt, dt)
        if hit is None:
            return None
        tf, q, speed = hit
        e = tgt_pos + tgt_vel * tf - q
        aim = aim + e
        if np.linalg.norm(e) < tol:
            break
    dd = aim - muzzle
    u = dd / np.linalg.norm(dd)
    return dict(u=u, az=math.atan2(u[1], u[0]), el=math.asin(u[2]), t_flight=tf, intercept=tgt_pos + tgt_vel * tf, speed=speed,
                err=float(np.linalg.norm(e)), ok=bool(np.linalg.norm(e) < 4 * tol))  # fmt: skip
