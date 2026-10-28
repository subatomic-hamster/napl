"""Parametric kinematic generators for generic test targets (multirotor routes, orbit, survey, weaving, swarms, balloons, hanging
loads, birds, debris). Each generator is `name(rng, T, **params) -> (P, V)` at the fine rate FINE (positions/velocities, shape
(N, 3)); `name_params(rng)` draws a randomised parameter set (the "random mode" used for ML training data). No generator steers
toward anything: routes follow given waypoints, weaving/birds only random-walk their turn rate (and are kept inside a box).
Sources and [est] marks: docs/cuas2/threat-scenarios.md. World frame x forward, y left, z up (m, s)."""

import numpy as np

FINE = 0.01  # truth integration step (s)
DT = 0.05  # sensor / scenario step (s)
SUB = 5  # FINE steps per DT
G = 9.81
BOX = np.array(
    [[8.0, -15.0, 2.0], [35.0, 15.0, 15.0]]
)  # backyard working volume (m), range ~8-35 m from the sensor


def _n(T):
    return int(round(T / FINE)) + 1


def _clipn(v, m):
    n = np.linalg.norm(v)
    return v if n <= m else v * (m / n)


def _smooth(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def ou(rng, n, tau, sigma, dim=None):
    """Stationary Ornstein-Uhlenbeck sequence at the fine rate (exact discretisation)."""
    a = np.exp(-FINE / tau)
    shape = (n,) if dim is None else (n, dim)
    e = rng.normal(size=shape) * sigma * np.sqrt(1 - a * a)
    x = np.empty(shape)
    x[0] = rng.normal(size=shape[1:]) * sigma
    for k in range(1, n):
        x[k] = a * x[k - 1] + e[k]
    return x


def sou(rng, n, tau, sigma, dim=None):
    """Smooth stationary noise for position-type perturbations (OU through two more first-order lags: continuous velocity),
    rescaled to std sigma."""
    x = ou(rng, n, tau, 1.0, dim)
    a = np.exp(-FINE / (tau / 3))
    for _ in range(2):
        for k in range(1, n):
            x[k] = a * x[k - 1] + (1 - a) * x[k]
    return x * (sigma / max(x.std(), 1e-9))


def _vel(P):
    return np.gradient(P, FINE, axis=0)


def _pt(rng, box=BOX):
    return rng.uniform(box[0], box[1])


def _keepin(p, psi, box, rate):
    """Turn-rate bias (rad/s) that rotates the heading away from a wall of the box while it points out (geofence)."""
    nv = np.zeros(2)
    for i in (0, 1):
        if p[i] < box[0][i]:
            nv[i] = 1.0
        elif p[i] > box[1][i]:
            nv[i] = -1.0
    if not nv.any():
        return 0.0
    h = np.array([np.cos(psi), np.sin(psi)])
    if h @ nv > 0.3 * np.linalg.norm(nv):
        return 0.0
    return rate * (1.0 if h[0] * nv[1] - h[1] * nv[0] > 0 else -1.0)


# ------------------------------------------------------------------------------------------------------------- wind
def wind(rng, T, w20, h=8.0, gain=1.0, mean_dir=None):
    """Mean wind (speed w20, random direction) plus Dryden low-altitude turbulence (MIL-STD-1797A scales/intensities, first-order
    shaping per axis for u, v, w) [est: first-order shaping for v, w; gain scales sigma]. Returns (N, 3)."""
    n = _n(T)
    hf = max(h * 3.281, 10.0)
    k = 0.177 + 0.000823 * hf
    sw = 0.1 * w20 * gain
    sig = np.array([sw / k**0.4, sw / k**0.4, sw])
    L = np.array([hf / k**1.2, hf / k**1.2 / 2, hf / 2]) * 0.3048
    ang = rng.uniform(0, 2 * np.pi) if mean_dir is None else mean_dir
    mean = w20 * np.array([np.cos(ang), np.sin(ang), 0.0])
    V = max(w20, 1.0)
    out = np.empty((n, 3))
    for i in range(3):
        out[:, i] = ou(rng, n, max(L[i] / V, 0.2), sig[i])
    return out + mean


def discrete_gusts(rng, T, n_gust, amp):
    """MIL-HDBK-1797 style "1-cosine" horizontal gusts: (N, 3) additive wind."""
    n = _n(T)
    t = np.arange(n) * FINE
    g = np.zeros((n, 3))
    for _ in range(n_gust):
        t0, d, a = (
            rng.uniform(1, max(T - 3, 2)),
            rng.uniform(1.5, 4.0),
            rng.uniform(0.5, 1.0) * amp,
        )
        th = rng.uniform(0, 2 * np.pi)
        s = np.where(
            (t > t0) & (t < t0 + d),
            0.5 * a * (1 - np.cos(2 * np.pi * (t - t0) / d)),
            0.0,
        )
        g += s[:, None] * np.array([np.cos(th), np.sin(th), 0.0])
    return g


# ------------------------------------------------------------------------------------------------------- multirotor
def route(
    rng,
    T,
    wps,
    vmax=5.0,
    amax=3.0,
    jmax=4.0,
    vzu=3.0,
    vzd=1.5,
    vpass=2.5,
    p0=None,
    v0=None,
    tau=0.6,
):
    """Point-mass multirotor flying a waypoint list [(xyz, dwell_s)] with velocity, acceleration and jerk limits (PX4 jerk-limited
    style). dwell > 0: stop (hover) at the waypoint for that long; dwell == 0: pass through at about vpass. Hovers after the last."""
    n = _n(T)
    P, V = np.zeros((n, 3)), np.zeros((n, 3))
    p = np.array(wps[0][0] if p0 is None else p0, float)
    v = np.zeros(3) if v0 is None else np.array(v0, float)
    a = np.zeros(3)
    i = 0 if p0 is not None else 1
    dwell = None
    for k in range(n):
        P[k], V[k] = p, v
        vd = np.zeros(3)
        if i < len(wps):
            tgt, dw = np.asarray(wps[i][0], float), wps[i][1]
            d = tgt - p
            dist = np.linalg.norm(d)
            vp = 0.0 if dw > 0 else vpass
            vdes = min(vmax, np.sqrt(vp**2 + 2 * 0.7 * amax * max(dist - 0.3, 0.0)))
            vd = d / max(dist, 1e-9) * vdes
            vd[2] = np.clip(vd[2], -vzd, vzu)
            vd[:2] = _clipn(vd[:2], vmax)
            if dw > 0:
                if dist < 0.5 and np.linalg.norm(v) < 0.8:
                    dwell = dw if dwell is None else dwell
                if dwell is not None:
                    dwell -= FINE
                    if dwell <= 0:
                        i, dwell = i + 1, None
            elif dist < 1.5:
                i += 1
        a = a + _clipn(_clipn((vd - v) / tau, amax) - a, jmax * FINE)
        v = v + a * FINE
        p = p + v * FINE
    return P, V


def route_params(rng, fast=False, hover=True, box=BOX):
    """fast=False: PX4 stock-ish limits (vmax 4-8 m/s, a 3 m/s^2, jerk 4); fast=True: tuned airframe up to ~2x stock, vmax up to 15."""
    s = rng.uniform(1.0, 2.0) if fast else rng.uniform(0.8, 1.2)
    vmax = rng.uniform(8, 15) if fast else rng.uniform(4, 8)
    nw = int(rng.integers(3, 7))
    wps = [
        (
            _pt(rng, box),
            float(rng.uniform(1.0, 4.0)) if hover and rng.random() < 0.4 else 0.0,
        )
        for _ in range(nw)
    ]
    return dict(
        wps=wps,
        vmax=vmax,
        amax=3.0 * s,
        jmax=4.0 * s,
        vzu=3.0 * s,
        vzd=1.5 * s,
        vpass=min(0.5 * vmax, 6.0),
    )


def survey(
    rng,
    T,
    x0,
    y0,
    width,
    height,
    lane,
    z,
    vmax=6.0,
    amax=3.0,
    jmax=4.0,
    vpass=2.0,
    flip=False,
):
    """Lawnmower area survey: lanes along x spaced by `lane`, corners flown through at ~vpass."""
    ys = np.arange(y0, y0 + height + 1e-9, lane)
    wps = []
    for j, y in enumerate(ys):
        xa, xb = (x0, x0 + width) if (j % 2 == 0) != flip else (x0 + width, x0)
        wps += [((xa, y, z), 0.0), ((xb, y, z), 0.0)]
    return route(rng, T, wps, vmax, amax, jmax, vpass=vpass)


def survey_params(rng):
    return dict(
        x0=rng.uniform(8, 18),
        y0=rng.uniform(-14, -6),
        width=rng.uniform(10, 18),
        height=rng.uniform(8, 18),
        lane=rng.uniform(3, 6),
        z=rng.uniform(5, 12),
        vmax=rng.uniform(3, 9),
        flip=bool(rng.random() < 0.5),
    )


def orbit(rng, T, c, R, v, direction=1, th0=0.0, bob=0.5, bob_f=0.1, wobble=0.2):
    """Loiter/orbit at speed v on a circle of radius R about a fixed point c (altitude bob, radius wobble)."""
    n = _n(T)
    t = np.arange(n) * FINE
    th = th0 + direction * v / R * t
    Rr = R + sou(rng, n, 3.0, wobble)
    z = c[2] + bob * np.sin(2 * np.pi * bob_f * t + rng.uniform(0, 6.28))
    P = np.stack([c[0] + Rr * np.cos(th), c[1] + Rr * np.sin(th), z], 1)
    return P, _vel(P)


def orbit_params(rng):
    R = rng.uniform(4, 12)
    v = rng.uniform(2, min(10.0, np.sqrt(0.5 * G * R)))  # centripetal accel <= 0.5 g
    c = _pt(rng, np.array([[16.0, -6.0, 4.0], [28.0, 6.0, 12.0]]))
    return dict(
        c=c,
        R=R,
        v=v,
        direction=int(rng.choice([-1, 1])),
        th0=rng.uniform(0, 6.28),
        bob=rng.uniform(0, 1.5),
        bob_f=rng.uniform(0.05, 0.2),
    )


def weave(
    rng,
    T,
    p0,
    psi0,
    v0=6.0,
    sig_w=0.6,
    tau_w=1.5,
    sig_s=1.0,
    tau_s=2.0,
    sig_z=1.0,
    vzmax=3.0,
    amax=8.0,
    vmax=15.0,
    box=BOX,
):
    """Random evasive weaving (Singer-like): turn rate and speed are Ornstein-Uhlenbeck, vertical acceleration likewise.
    Not pursuit of anything: only a keep-in box rotates the heading away from its walls."""
    n = _n(T)
    w, s, az = (
        sou(rng, n, tau_w, sig_w),
        v0 + sou(rng, n, tau_s, sig_s),
        sou(rng, n, 1.5, sig_z),
    )
    P = np.zeros((n, 3))
    p, psi, vz = np.array(p0, float), psi0, 0.0
    for k in range(n):
        P[k] = p
        sp = float(np.clip(s[k], 0.5, vmax))
        om = np.clip(
            w[k] + _keepin(p, psi, box, 0.8 * amax / sp), -amax / sp, amax / sp
        )
        psi += om * FINE
        vz = np.clip(
            vz
            + (az[k] - vz / 2.0 - 12.0 * (max(p[2] - 18, 0) - max(2.5 - p[2], 0)))
            * FINE,
            -vzmax,
            vzmax,
        )
        p = p + np.array([sp * np.cos(psi), sp * np.sin(psi), vz]) * FINE
    return P, _vel(P)


def weave_params(rng):
    return dict(
        p0=_pt(rng),
        psi0=rng.uniform(-np.pi, np.pi),
        v0=rng.uniform(3, 11),
        sig_w=rng.uniform(0.3, 1.2),
        tau_w=rng.uniform(1, 3),
        sig_s=rng.uniform(0.5, 1.5),
        sig_z=rng.uniform(0.3, 1.5),
    )


# ------------------------------------------------------------------------------------------------------------ swarm
def formation(kind, n, s):
    """Offsets (n, 2) = (forward, left) in the leader frame; leader is index 0."""
    i = np.arange(n)
    if kind == "line":
        return np.stack([np.zeros(n), (i - (n - 1) / 2) * s], 1)
    if kind == "v":
        k = (i + 1) // 2
        side = np.where(i % 2 == 1, 1.0, -1.0)
        return np.stack([-k * s * 0.8, side * k * s * 0.8], 1)
    cols = int(np.ceil(np.sqrt(n)))
    return np.stack([-(i // cols) * s, ((i % cols) - (cols - 1) / 2) * s], 1)


def swarm(
    rng,
    T,
    n,
    leader,
    kind0="v",
    kind1=None,
    t_change=8.0,
    spacing=3.0,
    jitter=0.25,
    split=None,
):
    """Formation of n agents behind a leader route (P, V). kind1/t_change: smooth change of formation over 4 s. split=(t_split,
    t_merge, dist, k): the last k agents peel off sideways by `dist` m, then rejoin. Returns list of (P, V) per agent."""
    LP, LV = leader
    N = len(LP)
    t = np.arange(N) * FINE
    fast = np.flatnonzero(np.linalg.norm(LV[:, :2], axis=1) > 0.5)
    psi = np.arctan2(LV[fast[0], 1], LV[fast[0], 0]) if len(fast) else 0.0
    psis = np.zeros(N)
    rate = 0.0
    for k in range(
        N
    ):  # formation heading = leader heading, slew-limited to 0.4 rad/s through a 1 s lag (smooth member velocities)
        want = 0.0
        if np.linalg.norm(LV[k, :2]) > 0.5:
            err = (np.arctan2(LV[k, 1], LV[k, 0]) - psi + np.pi) % (2 * np.pi) - np.pi
            want = float(np.clip(2.0 * err, -0.4, 0.4))
        rate += (want - rate) * FINE / 1.0
        psi += rate * FINE
        psis[k] = psi
    cs, sn = np.cos(psis), np.sin(psis)
    A = formation(kind0, n, spacing)
    B = formation(kind1 or kind0, n, spacing)
    b = _smooth((t - t_change) / 4.0) if kind1 else np.zeros(N)
    out = []
    for i in range(n):
        off = (1 - b)[:, None] * A[i] + b[:, None] * B[i]
        off = off + sou(rng, N, 1.5, jitter, 2)
        dz = np.zeros(N)
        if split and i >= n - split[3]:
            t0, t1, dist, _ = split
            off[:, 1] += (
                dist
                * (_smooth((t - t0) / 6.0) - _smooth((t - t1) / 6.0))
                * (1 if i % 2 else -1)
            )
            dz = 2.0 * (_smooth((t - t0) / 6.0) - _smooth((t - t1) / 6.0))
        P = LP.copy()
        P[:, 0] += cs * off[:, 0] - sn * off[:, 1]
        P[:, 1] += sn * off[:, 0] + cs * off[:, 1]
        P[:, 2] += dz + sou(rng, N, 2.0, jitter / 2)
        out.append((P, _vel(P)))
    return out


def swarm_params(rng, n=None):
    n = n or int(rng.integers(4, 10))
    kinds = ["line", "v", "grid"]
    k0, k1 = rng.choice(kinds, 2, replace=False)
    split = None
    if rng.random() < 0.5:
        t0 = rng.uniform(3, 8)
        split = (
            t0,
            t0 + rng.uniform(5, 8),
            rng.uniform(6, 12),
            int(rng.integers(1, max(2, n // 2) + 1)),
        )
    leader = route_params(rng)
    leader["vmax"] = min(leader["vmax"], 7.0)
    leader["vpass"] = min(leader["vpass"], 3.0)
    return dict(
        n=n,
        kind0=str(k0),
        kind1=str(k1) if rng.random() < 0.7 else None,
        t_change=rng.uniform(5, 12),
        spacing=rng.uniform(2.0, 5.0),
        jitter=rng.uniform(0.1, 0.4),
        split=split,
        leader_params=leader,
    )


# ---------------------------------------------------------------------------------------------- balloon / hanging
def balloon(
    rng, T, p0, w20=1.5, tau=2.0, bz=0.0, gain=2.0, n_gust=1, g_amp=2.5, mean_dir=None
):
    """Wind-drifting balloon: velocity relaxes to the local wind (Dryden turbulence + discrete gusts) with time constant tau
    [est], plus a net buoyancy vertical drift bz."""
    n = _n(T)
    W = wind(rng, T, w20, h=p0[2], gain=gain, mean_dir=mean_dir) + discrete_gusts(
        rng, T, n_gust, g_amp
    )
    W[:, 2] *= 0.5
    P = np.zeros((n, 3))
    p, v = np.array(p0, float), W[0].copy()
    for k in range(n):
        P[k] = p
        v = v + ((W[k] + np.array([0, 0, bz])) - v) / tau * FINE
        p = p + v * FINE
        if p[2] < 1.0:
            p[2], v[2] = 1.0, abs(v[2])
    return P, _vel(P)


def balloon_params(rng, T=20.0):
    w20 = rng.uniform(0.5, 3.0)
    ang = rng.uniform(0, 2 * np.pi)
    c = _pt(rng, np.array([[14.0, -8.0, 3.0], [30.0, 8.0, 12.0]]))
    drift = w20 * np.array([np.cos(ang), np.sin(ang), 0.0])
    return dict(
        p0=c - drift * T / 2,
        w20=w20,
        tau=rng.uniform(1.0, 4.0),
        bz=rng.uniform(-0.3, 0.3),
        gain=rng.uniform(1.0, 3.0),
        n_gust=int(rng.integers(0, 3)),
        g_amp=rng.uniform(1.5, 4.0),
        mean_dir=ang,
    )


def hanging(
    rng, T, anchor, L=1.5, th0=0.3, ph0=0.0, damp=0.08, w20=1.0, tau_d=4.0, gain=2.0
):
    """Hanging/tethered load: horizontal pendulum (small-angle spring g/L, damping, wind drag toward the local wind) on a fixed
    anchor; the bob height follows the string constraint."""
    n = _n(T)
    W = wind(rng, T, w20, h=anchor[2], gain=gain)[:, :2]
    om2 = G / L
    x = L * np.sin(th0) * np.array([np.cos(ph0), np.sin(ph0)])
    v = np.zeros(2)
    P = np.zeros((n, 3))
    for k in range(n):
        P[k, :2] = anchor[:2] + x
        P[k, 2] = anchor[2] - np.sqrt(max(L * L - x @ x, 1e-6))
        a = -om2 * x - damp * v + (W[k] - v) / tau_d
        v = v + a * FINE
        x = x + v * FINE
        r = np.linalg.norm(x)
        if r > 0.85 * L:
            x *= 0.85 * L / r
    return P, _vel(P)


def hanging_params(rng):
    return dict(
        anchor=_pt(rng, np.array([[10.0, -10.0, 4.0], [30.0, 10.0, 8.0]])),
        L=rng.uniform(0.5, 3.0),
        th0=rng.uniform(0.05, 0.5),
        ph0=rng.uniform(0, 6.28),
        damp=rng.uniform(0.03, 0.15),
        w20=rng.uniform(0.3, 2.5),
    )


# ------------------------------------------------------------------------------------------------------------- bird
# (m kg, wingspan b m, wing area S m^2, cruise speed range m/s, mean flap burst s, mean glide s) [est: textbook-scale values]
SPECIES = {
    "pigeon": (0.35, 0.65, 0.07, (8.0, 14.0), 3.0, 1.0),
    "corvid": (0.50, 0.90, 0.10, (7.0, 12.0), 2.5, 2.0),
    "gull": (0.80, 1.40, 0.20, (6.0, 12.0), 1.5, 3.0),
    "small": (
        0.03,
        0.25,
        0.012,
        (6.0, 11.0),
        0.5,
        0.3,
    ),  # flap-bounding: flaps at ~13 Hz, aliased at the 20 Hz sensor rate
}


def wingbeat_hz(m, b, S, rho=1.2):
    """Pennycuick 1996 fitted wingbeat frequency f = m^(3/8) g^(1/2) b^(-23/24) S^(-1/3) rho^(-3/8) (SI; equation recalled, not re-read)."""
    return m**0.375 * G**0.5 * b ** (-23 / 24) * S ** (-1 / 3) * rho**-0.375


def bird(
    rng,
    T,
    p0,
    psi0,
    species="pigeon",
    v0=10.0,
    f_hz=None,
    heave=0.04,
    rate_e=0.2,
    sig_e=1.5,
    vmax=15.0,
    box=BOX,
    z_ref=None,
):
    """Bird: alternating flap and glide phases (Markov durations by species), flapping adds body heave and +-8 % speed modulation
    at the wingbeat frequency, gliding decays to 0.75 v0 with a sink rate; erratic turns = OU turn rate plus Poisson shocks."""
    m, b, S, _, tf, tg = SPECIES[species]
    f = f_hz or wingbeat_hz(m, b, S) * rng.uniform(0.9, 1.1)
    n = _n(T)
    t = np.arange(n) * FINE
    flap = np.zeros(n)
    k = 0
    fl = rng.random() < 0.7
    while k < n:
        d = int(rng.exponential(tf if fl else tg) / FINE) + 20
        flap[k : k + d] = 1.0 if fl else 0.0
        k, fl = k + d, not fl
    env = np.empty(n)
    e = 0.0
    for k in range(n):
        e += (flap[k] - e) * FINE / 0.1
        env[k] = e
    w = sou(rng, n, 1.5, 0.15)
    shock = np.zeros(n)
    for _ in range(rng.poisson(rate_e * T)):
        k0 = int(rng.uniform(0, T - 1) / FINE)
        d = int(rng.uniform(0.3, 0.8) / FINE)
        shock[k0 : k0 + d] += rng.normal() * sig_e
    z_ref = z_ref if z_ref is not None else p0[2]
    phase = rng.uniform(0, 6.28)
    zbound = 0.15 * (species == "small")
    P = np.zeros((n, 3))
    p, psi, s, zc, vz = np.array(p0, float), psi0, v0, p0[2], 0.0
    sink = rng.uniform(0.3, 0.8)
    for k in range(n):
        P[k] = p
        vt = v0 if flap[k] else 0.75 * v0
        s += (vt - s) * FINE / 1.5
        om = np.clip(w[k] + shock[k] + _keepin(p, psi, box, 2.0), -3.0, 3.0)
        psi += om * FINE
        vzt = (-sink if not flap[k] else 0.0) + 0.3 * (z_ref - zc)
        vz += (vzt - vz) * FINE / 0.8
        zc += vz * FINE
        zc = float(np.clip(zc, 1.5, 22))
        ph = 2 * np.pi * f * t[k] + phase
        sp = min(s * (1 + 0.08 * env[k] * np.cos(ph)), vmax)
        p = p + np.array([sp * np.cos(psi), sp * np.sin(psi), 0.0]) * FINE
        p[2] = zc + env[k] * (
            heave * np.sin(ph) + zbound * np.sin(2 * np.pi * 2.5 * t[k])
        )
    return P, _vel(P)


def bird_params(rng):
    sp = str(rng.choice(list(SPECIES)))
    lo, hi = SPECIES[sp][3]
    return dict(
        p0=_pt(rng),
        psi0=rng.uniform(-np.pi, np.pi),
        species=sp,
        v0=rng.uniform(lo, hi),
        heave=rng.uniform(0.02, 0.07),
        rate_e=rng.uniform(0.1, 0.4),
        sig_e=rng.uniform(1.0, 2.0),
    )


# ----------------------------------------------------------------------------------------------------------- debris
def debris(
    rng,
    T,
    p0,
    kind="leaf",
    w20=1.5,
    tau=1.0,
    sink=0.8,
    flut_a=0.2,
    flut_f=2.0,
    gain=2.0,
    mean_dir=None,
):
    """Drifting debris. leaf: sinks at `sink`, follows the wind with tau, tumbles (vertical flutter flut_a at flut_f Hz, lateral
    OU 0.4 m/s). bag: near neutral buoyancy, vertical gusts at full strength, slow random vertical OU. [est]"""
    n = _n(T)
    t = np.arange(n) * FINE
    W = wind(rng, T, w20, h=max(p0[2], 3.0), gain=gain, mean_dir=mean_dir)
    if kind == "bag":
        W[:, 2] *= 2.0
        W[:, 2] += ou(rng, n, 1.0, 0.5)
    lat = ou(rng, n, 0.5, 0.4, 2)
    P = np.zeros((n, 3))
    p, v = np.array(p0, float), W[0].copy()
    ph = rng.uniform(0, 6.28)
    for k in range(n):
        tgt = (
            W[k]
            + np.r_[lat[k], 0.0]
            - np.array([0, 0, sink if kind == "leaf" else 0.3 * (p[2] - p0[2])])
        )
        v = v + (tgt - v) / tau * FINE
        p = p + v * FINE
        P[k] = p
        P[k, 2] += (
            (flut_a * np.sin(2 * np.pi * flut_f * t[k] + ph)) if kind == "leaf" else 0.0
        )
        if p[2] < 0.3:
            P[k:, :] = P[k]
            P[k:, 2] = 0.3
            break
    return P, _vel(P)


def debris_params(rng, T=20.0, kind=None):
    kind = kind or str(rng.choice(["leaf", "bag"]))
    w20 = rng.uniform(0.5, 3.0)
    ang = rng.uniform(0, 2 * np.pi)
    c = _pt(rng, np.array([[14.0, -8.0, 4.0], [30.0, 8.0, 14.0]]))
    drift = w20 * np.array([np.cos(ang), np.sin(ang), 0.0])
    z0 = rng.uniform(8, 18) if kind == "leaf" else c[2]
    p0 = c - drift * T / 2
    p0[2] = z0
    return dict(
        p0=p0,
        kind=kind,
        w20=w20,
        tau=rng.uniform(0.5, 1.5),
        sink=rng.uniform(0.4, 1.2),
        flut_a=rng.uniform(0.1, 0.3),
        flut_f=rng.uniform(1, 3),
        gain=rng.uniform(1.5, 3.0),
        mean_dir=ang,
    )
