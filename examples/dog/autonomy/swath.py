"""Swath and density measurements on a straight carrier pass, mapped with the true pose (isolates the sensor from localization).
cd examples/dog && ../../.venv/bin/python -m autonomy.swath [--kind rubble] [--rays 5000] [--pool 2] [--seed 1]"""

import argparse
import json
import tempfile
from pathlib import Path

import napl
import numpy as np

from . import carrier, config, elevmap, lidar, scene

BLIND_START = 1.5  # m of the route not scored at its start (see drive_and_map)
LANE_OVERLAP = 0.9  # lane = 2 x w95 x 0.9: 10 % overlap between neighbouring swaths


def drive_and_map(kind, rays, seed, out_dir, t_s=10.0, size=20.0):
    out_dir = Path(out_dir)
    cfg = config.override(config.load(), **{"map.size": size})
    f = scene.make_field({"kind": kind, "size": size, "patch": 10.0}, seed)
    start = (-size / 2 + 2.0, 0.0, 0.0)
    sim = napl.Sim.load(carrier.build(f, cfg, rays, start, out_dir))
    c = carrier.Carrier(sim, f, cfg, start, seed=seed)
    ros = lidar.Rosette(sim, "lidar", rays, c.dt, cfg)
    ext = lidar.extrinsic((out_dir / "carrier.xml").read_text(), "lidar", "carrier")
    m, rng = elevmap.ElevationMap(cfg), np.random.default_rng([seed, 0x5A7])
    c.command(cfg["carrier"]["v_max"], 0.0, 0.0)
    xs = []
    for _ in range(int(round(t_s * cfg["lidar"]["rate_hz"]))):
        c.advance(1.0 / cfg["lidar"]["rate_hz"])
        p, q = c.truth()
        s = ros.latest()
        o = p + lidar.qmat(q) @ ext[0]
        m.fuse(
            lidar.points(s, o, lidar.qmul(q, ext[1]), cfg["lidar"]["ang_sigma"], rng),
            o,
            0.0,
            cfg["odom"]["att_sigma"],
        )
        xs.append(float(p[0]))
    # cells under and within ~1 m ahead of the lidar at the start pose are blind (ground is first seen 0.72 m ahead of the lidar, which
    # sits 0.25 m ahead of the base) and the rear view looks up, so they are never mapped by a pass that starts there: score from 1.5 m on
    return m, cfg, (min(xs) + BLIND_START, max(xs))


def _centres(m, pool):
    q = m.n // pool
    return m.x0 + (np.arange(q) + 0.5) * m.res * pool


def band_coverage(m, pool, x_range, y_half):
    c = _centres(m, pool)
    cols = (c >= x_range[0]) & (c <= x_range[1])
    rows = np.abs(c) < y_half
    return float(m.mapped(pool)[np.ix_(rows, cols)].mean())


def profile(m, pool, x_range, step=0.5, reach=12.0):
    c = _centres(m, pool)
    cols = (c >= x_range[0]) & (c <= x_range[1])
    mp = m.mapped(pool)[:, cols]
    offs = np.arange(-reach, reach, step) + step / 2
    frac = np.array(
        [
            mp[np.abs(c - o) < step / 2].mean()
            if (np.abs(c - o) < step / 2).any()
            else 0.0
            for o in offs
        ]
    )
    return offs, frac


def measure(kind, rays, pool, seed, out_dir):
    m, cfg, xr = drive_and_map(kind, rays, seed, out_dir)
    offs, frac = profile(m, pool, xr)
    w = {}
    for side, sel in (("w95_right", offs < 0), ("w95_left", offs > 0)):
        o, fr = np.abs(offs[sel]), frac[sel]
        order = np.argsort(o)
        ok = np.cumprod(fr[order] >= 0.95).astype(
            bool
        )  # contiguous from the robot outwards
        w[side] = float(o[order][ok].max() + 0.25) if ok.any() else 0.0
    w["lane_m"] = 2 * min(w["w95_left"], w["w95_right"]) * LANE_OVERLAP
    return w


if __name__ == "__main__":
    a = argparse.ArgumentParser()
    a.add_argument("--kind", default="rubble")
    a.add_argument("--rays", type=int, default=5000)
    a.add_argument("--pool", type=int, default=2)
    a.add_argument("--seed", type=int, default=1)
    a = a.parse_args()
    with tempfile.TemporaryDirectory() as d:
        print(json.dumps(measure(a.kind, a.rays, a.pool, a.seed, Path(d))))
