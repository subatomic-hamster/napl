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


def camera(kind, p, R, vel, mount_deg=0.0):
    fwd = R @ np.array([1.0, 0.0, 0.0])
    if (
        kind == "fpv"
    ):  # the optical axis: the airframe's x axis pitched up by the mount angle (goal +10, follow -8, see skydio_scene.tilt_camera)
        axis = R @ np.array(
            [np.cos(np.radians(mount_deg)), 0.0, np.sin(np.radians(mount_deg))]
        )
        return dict(
            pos=p + 0.1 * fwd + np.array([0, 0, 0.05]), lookat=p + 5 * axis, fovy=58.0
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


def draw_detections(img, cam, dets):
    """Outline each detection (world box: feet to z_top, `width` wide) on an fpv frame; green = the subject, red = anything else."""
    h, w = img.shape[:2]
    d = cam["lookat"] - cam["pos"]
    d = d / np.linalg.norm(d)
    right = np.cross(d, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, d)
    f = (h / 2) / np.tan(np.radians(cam["fovy"]) / 2)
    out = img.copy()
    for det in dets:
        us, vs = [], []
        for dx in (-0.5, 0.5):
            for z in (0.0, det["z_top"]):
                c = np.array([*det["xy"], z]) + dx * det["width"] * right - cam["pos"]
                if c @ d < 0.3:
                    break
                us.append(w / 2 + f * (c @ right) / (c @ d))
                vs.append(h / 2 - f * (c @ up) / (c @ d))
        if len(us) < 4:
            continue
        u0, u1 = int(max(min(us), 0)), int(min(max(us), w - 1))
        v0, v1 = int(max(min(vs), 0)), int(min(max(vs), h - 1))
        col = ([0, 255, 0, 255] if det.get("on_subject") else [255, 0, 0, 255])[
            : img.shape[2]
        ]
        for t in range(2):
            out[v0 : v1 + 1, [u0 + t, u1 - t]] = col
            out[[v0 + t, v1 - t], u0 : u1 + 1] = col
    return out


def draw_belief(img, cam, pts, z=1.0):
    """Yellow dots at the subject-search particles (telemetry subject.search.particles, ground xy, drawn at chest height `z`) in any camera of `camera()`."""
    h, w = img.shape[:2]
    d = cam["lookat"] - cam["pos"]
    d = d / np.linalg.norm(d)
    right = np.cross(d, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, d)
    f = (h / 2) / np.tan(np.radians(cam["fovy"]) / 2)
    out = img.copy()
    for x, y in pts:
        c = np.array([x, y, z]) - cam["pos"]
        if c @ d < 0.3:
            continue
        u, v = (
            int(w / 2 + f * (c @ right) / (c @ d)),
            int(h / 2 - f * (c @ up) / (c @ d)),
        )
        if 2 <= u < w - 2 and 2 <= v < h - 2:
            out[v - 2 : v + 3, u - 2 : u + 3] = [255, 220, 0, 255][: img.shape[2]]
    return out


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
    ap.add_argument(
        "--detections",
        action="store_true",
        help="fpv only: outline telemetry.jsonl detections (follow runs)",
    )
    ap.add_argument(
        "--belief",
        action="store_true",
        help="follow runs: draw the subject-search particles (yellow) while the subject is unseen",
    )
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))
    mission = json.loads((Path(a.run) / "model" / "forest.json").read_text())["mission"]
    mount_deg = -8.0 if mission == "follow" else 10.0
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
    if a.detections or a.belief:
        tele = [json.loads(ln) for ln in open(Path(a.run) / "telemetry.jsonl")]
        tele = [x for x in tele if x.get("subject")]
        tel_t = np.array([x["t"] for x in tele])
    for k, t in enumerate(times):
        tick = r.to(t)
        j = min(int(np.searchsorted(col["t"], t)), len(col["t"]) - 1)
        p = np.array([col["x"][j], col["y"][j], col["z"][j]])
        q = [col[c][j] for c in ("qw", "qx", "qy", "qz")]
        jp = max(j - 25, 0)
        vel = (p - np.array([col["x"][jp], col["y"][jp], col["z"][jp]])) / max(
            col["t"][j] - col["t"][jp], 1e-6
        )
        cam = camera(a.cam, p, quat_rot(q), vel, mount_deg)
        img, _ = r.sim.render(width=w, height=h, **cam)
        if a.detections or a.belief:
            i = min(int(np.searchsorted(tel_t, t)), len(tel_t) - 1)
        if a.detections:
            img = draw_detections(
                np.asarray(img), cam, tele[i]["subject"]["detections"]
            )
        if a.belief and tele[i]["subject"].get("search"):
            img = draw_belief(
                np.asarray(img), cam, tele[i]["subject"]["search"]["particles"]
            )
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
