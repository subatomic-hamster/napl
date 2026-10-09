"""Render frames of a finished ASV run (GPU, offscreen Vulkan): python examples/asv/render_frames.py <run dir> --out <dir> [options]

<run dir> is a `napl program` run dir built with visual = true (result.json, replay.naplog, trace.csv, model/ present). The run's replay log is re-applied to a fresh
World (tools/screenshots.py `Replay`), so the frames show exactly the simulated run; the sim is advanced to each requested time and `Sim.render` draws it.

    --times 20,50,80        sim seconds of the stills (default: 5 evenly spaced over the run)
    --fps 30 --from 0 --to 20   instead: every 1/fps s over [from, to] (a frame sequence for a video; f00000.png ...)
    --cam chase|side|top|bow|wide   chase = 6 m behind and 2.5 m above the ASV looking ahead of it, side = 9 m abeam, top = 45 m above, bow = 1.6 m above the
                                    deck looking along the course, wide = 22 m back and 9 m up (the encounter in view)
    --size 960x540
Writes <out>/f<NNNNN>.png and <out>/frames.json (sim time and tick of every frame, the replay's hash check). Frames are for looking at and for the video worker;
they are never committed."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "tools"))
sys.path.insert(0, str(HERE.parents[1] / "python"))
import napl
from napl import _napl
from screenshots import Replay

Z_DECK = 0.35  # m above the water, the camera's reference height


def camera(kind, p, h):
    """p: ASV (x, y, z); h: unit heading vector (east, north, 0). -> kwargs of Sim.render."""
    side = np.array([-h[1], h[0], 0.0])
    if kind == "bow":
        return dict(
            pos=p + 0.5 * h + np.array([0, 0, 1.6]), lookat=p + 25 * h, fovy=60.0
        )
    if kind == "side":
        return dict(
            pos=p + 9.0 * side + np.array([0, 0, 2.5]), lookat=p + 1.0 * h, fovy=45.0
        )
    if kind == "top":
        return dict(
            pos=p + np.array([0.0, 0.0, 45.0]) - 0.01 * h, lookat=p + 6.0 * h, fovy=60.0
        )
    if kind == "wide":
        return dict(
            pos=p - 22.0 * h + 5.0 * side + np.array([0, 0, 9.0]),
            lookat=p + 14.0 * h,
            fovy=55.0,
        )
    return dict(
        pos=p - 6.0 * h + np.array([0, 0, 2.5]) + 1.0 * side,
        lookat=p + 4.0 * h,
        fovy=50.0,
    )


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", required=True)
    ap.add_argument("--times", default="")
    ap.add_argument("--fps", type=float, default=0)
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=0.0)
    ap.add_argument(
        "--cam", default="chase", choices=["chase", "side", "top", "bow", "wide"]
    )
    ap.add_argument("--size", default="960x540")
    a = ap.parse_args(argv)
    w, h = (int(v) for v in a.size.split("x"))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    napl.render_style("outdoor,shadows")
    r = Replay(a.run)
    r.dt = 0.002  # the kernel step (assemblies/asv.toml); the trace's last row is not necessarily the last tick
    t_end = r.res["sim"]["tick"] * r.dt
    if a.fps:
        times = list(np.arange(a.t0, a.t1 or t_end, 1.0 / a.fps))
    elif a.times:
        times = [float(v) for v in a.times.split(",")]
    else:
        times = list(np.linspace(0.15 * t_end, 0.95 * t_end, 5))
    col = {k: r.col(k) for k in ("t", "x_m", "y_m", "heading_deg")}
    meta = []
    for k, t in enumerate(times):
        tick = r.to(t)
        j = min(int(np.searchsorted(col["t"], t)), len(col["t"]) - 1)
        p = np.array([col["x_m"][j], col["y_m"][j], Z_DECK])
        psi = math.radians(90.0 - col["heading_deg"][j])
        hv = np.array([math.cos(psi), math.sin(psi), 0.0])
        img, _ = r.sim.render(width=w, height=h, **camera(a.cam, p, hv))
        (out / f"f{k:05d}.png").write_bytes(_napl.encode_png(np.ascontiguousarray(img)))
        meta.append(
            dict(frame=k, t=round(float(t), 3), tick=int(tick), pos=p.round(2).tolist())
        )
    hsh, ok = r.finish()
    (out / "frames.json").write_text(
        json.dumps(
            dict(
                cam=a.cam,
                size=[w, h],
                replay_hash=hsh,
                replay_matches_run=ok,
                frames=meta,
            ),
            indent=1,
        )
    )
    sys.stdout.write(f"{len(meta)} frames in {out}, replay matches: {ok}\n")


if __name__ == "__main__":
    main()
