"""Stills of the vessel at sea (GPU, offscreen Vulkan): python examples/ship/render_shots.py --out <dir> [--hs 1.5] [--lander x500v2] [--speed 3.0] [--seed 3] [--run 60]

The vessel sails `--run` s at `--speed` m/s under the autopilot in a seeded JONSWAP sea of `--hs` m (waves from ahead-port), then three stills are drawn: `quarter` (stern
quarter, the helipad and the lander in view), `side` (abeam, the roll and pitch of the hull against the waves) and `top` (the pad from above). Files: <out>/<lander>_<view>.png.
Never committed; look at them."""

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "python"))
sys.path.insert(0, str(HERE.parent))
import napl
from napl import _napl
from napl.cuas import relnav
from napl.cuas.ship import PAD_CENTRE, body_to_world, ship_scene

from ship.pilot import ShipPilot


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--hs", type=float, default=1.5)
    ap.add_argument("--lander", default="x500v2")
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--run", type=float, default=60.0)
    ap.add_argument("--size", default="1280x720")
    a = ap.parse_args(argv)
    w, h = (int(v) for v in a.size.split("x"))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    napl.render_style("outdoor,shadows")
    sc = ship_scene(
        tempfile.mkdtemp(prefix="osvshots_"),
        lander=a.lander or None,
        hs=a.hs,
        wave_dir=150.0,
        seed=a.seed,
        visual=True,
    )
    sim = napl.Sim.load(sc["model"])
    pl = ShipPilot(sim)
    pl.set_heading(0.0, a.speed)
    for _ in range(int(a.run / 0.04)):
        pl.advance(sim)
    hp = np.asarray(sim.query("/world/osv_hull", "pos")[0], float)
    hq = np.asarray(sim.query("/world/osv_hull", "quat")[0], float)
    R = relnav.quat_to_mat(hq)
    pad = body_to_world(hp, hq, PAD_CENTRE)
    views = {
        "quarter": dict(
            pos=body_to_world(hp, hq, [-13.0, 7.0, 4.5]),
            lookat=pad + np.array([0.0, 0.0, 0.2]),
            fovy=50.0,
        ),
        "side": dict(
            pos=hp + R @ np.array([0.0, -17.0, 3.5]),
            lookat=hp + R @ np.array([0.0, 0.0, 1.5]),
            fovy=45.0,
        ),
        "top": dict(pos=pad + np.array([-0.01, 0.0, 14.0]), lookat=pad, fovy=45.0),
    }
    for name, cam in views.items():
        img, _ = sim.render(width=w, height=h, **cam)
        (out / f"{a.lander or 'ship'}_{name}.png").write_bytes(
            _napl.encode_png(np.ascontiguousarray(img))
        )
    sys.stdout.write(
        f"stills in {out}: hs {a.hs} m, {a.speed} m/s, t = {sim.tick * 0.001:.1f} s, pad z {pad[2]:.2f}\n"
    )


if __name__ == "__main__":
    main()
