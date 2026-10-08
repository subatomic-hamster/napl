"""Render frames of a finished Skydio run (GPU, offscreen Vulkan): python examples/skydio/render_frames.py <run dir> --out <dir> [options]

<run dir> is a `napl program` run dir (result.json, replay.naplog, trace.csv, model/ present). The run's replay log is re-applied to a fresh World
(tools/screenshots.py `Replay`), so the frames show exactly the simulated flight; the sim is advanced to each requested time and `Sim.render` draws it.

    --times 6,12,18,24      sim seconds of the stills (default: 5 evenly spaced over the flight)
    --fps 30 --from 0 --to 20   instead: every 1/fps s over [from, to] (a frame sequence for a video; f00000.png ...)
    --cam chase|side|top|fpv    chase = 3.5 m behind and 1.2 m above the drone looking at it, side = 8 m abeam, top = 25 m above,
                                fpv = from the airframe looking along its nose (the camera's field of view, in colour)
    --size 960x540
Writes <out>/f<NNNNN>.png and <out>/frames.json (sim time and tick of every frame, the replay's hash check)."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "tools"))
sys.path.insert(0, str(HERE.parents[1] / "python"))
import napl
from napl import _napl
from screenshots import Replay


def quat_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def camera(kind, p, R, vel):
    fwd = R @ np.array([1.0, 0.0, 0.0])
    if kind == "fpv":
        return dict(
            pos=p + 0.1 * fwd + np.array([0, 0, 0.05]), lookat=p + 5 * fwd, fovy=58.0
        )
    h = (
        vel[:2] / np.linalg.norm(vel[:2])
        if np.linalg.norm(vel[:2]) > 0.5
        else fwd[:2] / max(np.linalg.norm(fwd[:2]), 1e-6)
    )
    h = np.array([h[0], h[1], 0.0])
    side = np.array([-h[1], h[0], 0.0])
    if kind == "chase":
        return dict(
            pos=p - 3.5 * h + np.array([0, 0, 1.2]) + 0.6 * side,
            lookat=p + 2.0 * h,
            fovy=45.0,
        )
    if kind == "side":
        return dict(
            pos=p + 8.0 * side + np.array([0, 0, 1.5]), lookat=p + 2.0 * h, fovy=50.0
        )
    return dict(
        pos=p + np.array([0.0, 0.0, 25.0]) - 0.01 * h,
        lookat=p + 3.0 * h,
        fovy=60.0,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", required=True)
    ap.add_argument("--times", default="")
    ap.add_argument("--fps", type=float, default=0)
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=0.0)
    ap.add_argument("--cam", default="chase", choices=["chase", "side", "top", "fpv"])
    ap.add_argument("--size", default="960x540")
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    napl.render_style("outdoor,shadows")
    r = Replay(a.run)
    r.dt = (
        0.001  # the kernel step; the trace's last row is not necessarily the last tick
    )
    t_end = r.res["sim"]["tick"] * r.dt
    if a.fps:
        t1 = a.t1 or t_end
        times = list(np.arange(a.t0, t1, 1.0 / a.fps))
    elif a.times:
        times = [float(v) for v in a.times.split(",")]
    else:
        times = list(np.linspace(0.2 * t_end, 0.95 * t_end, 5))
    col = {k: r.col(k) for k in ("t", "x", "y", "z", "qw", "qx", "qy", "qz")}
    meta = []
    for k, t in enumerate(times):
        tick = r.to(t)
        j = min(int(np.searchsorted(col["t"], t)), len(col["t"]) - 1)
        p = np.array([col["x"][j], col["y"][j], col["z"][j]])
        q = [col[c][j] for c in ("qw", "qx", "qy", "qz")]
        jp = max(j - 25, 0)
        vel = (p - np.array([col["x"][jp], col["y"][jp], col["z"][jp]])) / max(
            col["t"][j] - col["t"][jp], 1e-6
        )
        img, _ = r.sim.render(width=w, height=h, **camera(a.cam, p, quat_rot(q), vel))
        (out / f"f{k:05d}.png").write_bytes(_napl.encode_png(np.ascontiguousarray(img)))
        meta.append(
            dict(frame=k, t=round(float(t), 3), tick=int(tick), pos=p.round(3).tolist())
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
    sys.stdout.write(
        f"{len(meta)} frames -> {out} (replay hash {hsh}, {'MATCH' if ok else 'DIFFERENT'})\n"
    )


if __name__ == "__main__":
    main()
