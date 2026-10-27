"""Textbook tracking benchmarks (generic kinematic targets, no threat behaviours): single CV, coordinated-turn onset, constant
acceleration, two crossing targets, five closely spaced targets in clutter, and a pan/tilt camera with size-based range.
build(name, seed) is deterministic in (name, seed); the same seed gives the same truth and detections to every variant."""

from dataclasses import dataclass

import numpy as np

from .models import CT, CameraModel, PositionModel

NAMES = ("cv", "turn", "ca", "cross", "swarm5", "cam")


@dataclass
class Scenario:
    name: str
    dt: float
    ids: list
    truth: np.ndarray  # (T, n, 6)
    frames: list  # [(t, meas, zs (k, dim))]
    lam: float  # clutter density per unit measurement-space volume
    pd: float
    c: float  # GOSPA cut-off (m)
    n_clutter: float


def _path(p0, v0, segs, dt, rng=None, q_acc=0.0):
    """Piecewise path: segs = [(duration, 'cv') | (duration, 'ca', a3) | (duration, 'ct', omega)]; optional random-walk velocity."""
    ct = CT()
    x = np.r_[p0, v0].astype(float)
    out = [x[:6].copy()]
    for seg in segs:
        for _ in range(int(round(seg[0] / dt))):
            if seg[1] == "ct":
                y = ct.f(np.r_[x[:6], seg[2]], dt)
                x = y[:6]
            else:
                a = np.asarray(seg[2], float) if seg[1] == "ca" else np.zeros(3)
                if rng is not None and q_acc:
                    a = a + rng.normal(size=3) * q_acc
                x = np.r_[x[:3] + x[3:6] * dt + 0.5 * a * dt * dt, x[3:6] + a * dt]
            out.append(x[:6].copy())
    return np.array(out)


def build(name, seed):
    rng = np.random.default_rng(
        [sum(map(ord, name)), seed]
    )  # not hash(): that is salted per process
    dt, pd, c, ncl, sig = 0.1, 0.9, 2.0, 5.0, 0.5
    if name == "cv":
        tr = [_path([0, 0, 10], [10, 3, 0], [(20, "cv")], dt)]
        pd, ncl = 0.95, 3.0
    elif name == "turn":
        tr = [_path([0, 0, 10], [12, 0, 0], [(8, "cv"), (6, "ct", 0.5), (6, "cv")], dt)]
    elif name == "ca":
        tr = [_path([0, 0, 10], [5, 0, 0], [(16, "ca", [1.5, 0.5, 0])], dt)]
    elif name == "cross":
        v = 12 * np.array([np.cos(0.35), np.sin(0.35), 0])
        w = v * [1, -1, 1]
        tr = [
            _path(np.array([0, 0, 10]) - v * 10, v, [(20, "cv")], dt),
            _path(np.array([0, 0, 10]) - w * 10, w, [(20, "cv")], dt),
        ]
        ncl = 8.0
    elif name == "swarm5":
        y = [-6, -3, 0, 3, 6]
        z = [0, 2, -2, 1.5, -1.5]
        d = [0.6, -0.3, 0.0, 0.3, -0.6]
        tr = [
            _path([0, y[i], 10 + z[i]], [10, d[i], 0], [(15, "cv")], dt, rng, q_acc=0.5)
            for i in range(5)
        ]
        ncl = 15.0
    elif name == "cam":
        dt, c, ncl, sig = 0.05, 6.0, 1.0, 2.0
        tr = [_path([20, -12, 3], [0.5, 4, 0.2], [(12, "cv")], dt)]
    else:
        raise ValueError(name)
    truth = np.stack(tr, 1)  # (T, n, 6)
    T = len(truth)
    frames = []
    if name == "cam":
        vol = 1920 * 1080 * 35.0
        lam = ncl / vol
        prev = truth[0, 0, :3]
        for k in range(T):
            d0 = prev
            pan, tilt = (
                np.arctan2(d0[1], d0[0]),
                np.arctan2(d0[2], np.hypot(d0[0], d0[1])),
            )  # head follows with one frame of lag
            m = CameraModel((0, 0, 0), pan, tilt, size_range=True)
            zs = (
                [m.sample(rng, truth[k, 0, :3])]
                if rng.random() < pd and m.in_view(truth[k, 0, :3])
                else []
            )
            for _ in range(rng.poisson(ncl)):
                zs.append(
                    np.array(
                        [rng.uniform(0, 1920), rng.uniform(0, 1080), rng.uniform(5, 40)]
                    )
                )
            frames.append((k * dt, m, np.array(zs).reshape(-1, 3)))
            prev = truth[k, 0, :3]
    else:
        pm = PositionModel(sig)
        lo, hi = (
            truth.reshape(-1, 6)[:, :3].min(0) - [10, 10, 5],
            truth.reshape(-1, 6)[:, :3].max(0) + [10, 10, 5],
        )
        lam = ncl / float(np.prod(hi - lo))
        for k in range(T):
            zs = [
                pm.sample(rng, truth[k, i, :3])
                for i in range(truth.shape[1])
                if rng.random() < pd
            ]
            zs += list(rng.uniform(lo, hi, size=(rng.poisson(ncl), 3)))
            rng.shuffle(zs)
            frames.append((k * dt, pm, np.array(zs).reshape(-1, 3)))
    return Scenario(
        name, dt, list(range(truth.shape[1])), truth, frames, lam, pd, c, ncl
    )
