"""Show / record a counter-UAS (or any native-program) run in the viewer with the HUD. Media is never committed: frames and videos stay on the host.

The run (`napl program examples/cuas/cuas.scenario.toml`) is REPLAYED, not re-run: its replay.naplog is applied to a fresh World on a Host
(the same replay the state-hash check uses, so the picture is of exactly the run that was judged), the viewer attaches to that Host in
`clean` mode (no panes: the 3D view fills the window) and the HUD document is rebuilt from the run's hud.jsonl (stage, tracks, counters,
camera frames) + events.json + the replayed World (muzzle, pellets, drones) by napl.cuas.hudview. The final state hash is compared with
the run's native.json. The turret's pixels come from the run's own camera frames (frames/*.png), so no second render of them is needed.

    play    RUN_DIR [--camera SPEC] [--rate 1]      live window (needs a display and a GPU); rate = sim seconds per wall second at most
    frames  RUN_DIR OUT_DIR [--t0 0 --t1 60 --fps 20 --camera SPEC]
                                                    xvfb + the viewer; one PNG per video frame, stepped in lockstep with the window
                                                    (OUT_DIR/f00000.png ..., meta.json). Chunks: run it again with another --t0/--t1
                                                    and --start-index; the last chunk (--t1 >= the run) checks the replay hash.
    encode  FRAMES_DIR... OUT.webm [--fps 20] [--gpl]   needs `av` + Pillow (a throwaway venv, as record_build.py): OpenH264 / VP9 by default,
                                                    libx264 (GPL) only with --gpl
SPEC = a preset (overview | chase | target | top | swarm | close | orbit | swarmorbit | free) or a schedule "0:overview,20:chase,40:target".

hfbox (every command under an outer timeout, the GPU lock held for minutes only; see docs/guide.md):
    tools/remote.sh run --gpu 'timeout 600 .venv/bin/python tools/record_sim.py frames <run> /workspace/wt/napl-hud/out/f --t0 0 --t1 20'
"""

import argparse
import bisect
import json
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import videnc

W, H = 1280, 720
VID_FPS = 20


def parse_camera(spec):
    """'overview' or '0:overview,20:chase' -> [(t, preset)] sorted."""
    if ":" not in spec:
        return [(0.0, spec)]
    out = []
    for part in spec.split(","):
        t, name = part.split(":")
        out.append((float(t), name))
    return sorted(out)


class Player:
    """A run's replay.naplog applied, in order and tick-exact, to a fresh World on a Host; plus the run's telemetry at any sim time."""

    def __init__(self, run_dir, camera="overview"):
        from napl import Sim
        from napl.cuas import hudview
        from screenshots import setattrs

        self.run = Path(run_dir).resolve()
        self.hv = hudview
        self.man = json.loads((self.run / "model" / "scene.json").read_text())
        self.native = json.loads((self.run / "native.json").read_text())
        self.recs = setattrs((self.run / "replay.naplog").read_bytes())
        self.hud_recs = hudview.read_hud(self.run)
        self.hud_t = [r["t"] for r in self.hud_recs]
        self.events = json.loads((self.run / "events.json").read_text())
        self.sim = Sim.load(self.run / "model" / "scene.xml", host=True)
        self.dt, self.i = self.native["dt"], 0
        self.total_s = self.native["tick"] * self.dt
        turret = next(
            v
            for v in self.man["vehicles"]
            if any(
                n.startswith(v + "_") for n in self.man["devices"].get("launchers", [])
            )
        )
        self.gun = f"/devices/{turret}_gun"
        self.drones = {}
        for name in self.man["vehicles"]:
            if name != turret and any(
                n.startswith(name + "_") for n in self.man["devices"].get("rotors", [])
            ):
                self.drones[name] = next(
                    p
                    for p in self.man["body_paths"].values()
                    if p.startswith(f"/world/{name}_") and p.count("/") == 2
                )
        self.destroyed = {
            e["drone"]: e["t"] for e in self.events if e["type"] == "destroyed"
        }
        self.schedule = parse_camera(camera)
        self.cam = None

    @property
    def t(self):
        return self.sim.tick * self.dt

    def to(self, t):
        """Advance the replay to sim time t (never backwards)."""
        tick = min(round(t / self.dt), self.native["tick"])
        while self.i < len(self.recs) and self.recs[self.i][0] <= tick:
            self._adv(self.recs[self.i][0])
            self.sim.set(*self.recs[self.i][1:])
            self.i += 1
        self._adv(tick)

    def _adv(self, tick):
        if tick > self.sim.tick:
            self.sim.step(tick - self.sim.tick)

    def finish(self):
        """Replay the rest; -> (hash, matches the run's native.json)."""
        for tick, *r in self.recs[self.i :]:
            self._adv(tick)
            self.sim.set(*r)
        self.i = len(self.recs)
        self._adv(self.native["tick"])
        h = f"{self.sim.state_hash():016x}"
        return h, h == self.native["state_hash"]

    # ---- one HUD document ------------------------------------------------------------------------------------------------------
    def preset(self, t):
        k = bisect.bisect_right([s[0] for s in self.schedule], t) - 1
        return self.schedule[max(k, 0)][1]

    def document(self, seq, size=(W, H), dt=1 / VID_FPS):
        """Build the HUD document for the replay's current time (call after `to`)."""
        t, q = self.t, self.sim.query
        j = bisect.bisect_right(self.hud_t, t) - 1
        rec = self.hud_recs[j] if j >= 0 else None
        mz, md = q(self.gun, "muzzle_pos")[0], q(self.gun, "muzzle_dir")[0]
        pellets = []
        if int(q(self.gun, "rounds")[0][0]) > 0:
            act, pos = q(self.gun, "active")[0], q(self.gun, "pos")[0].reshape(-1, 3)
            pellets = [pos[i].tolist() for i in range(len(act)) if act[i] > 0.5]
        view = self.hv.View(
            t=t,
            rec=rec,
            run_dir=self.run,
            muzzle=(mz.tolist(), md.tolist()),
            pellets=pellets,
            events=self.events,
            vp=size,
        )
        camera = None
        preset = self.preset(t)
        if preset != "free":
            drones = [
                q(p, "pos")[0].tolist()
                for n, p in self.drones.items()
                if self.destroyed.get(n, 1e9) > t
            ]
            sel = (
                next((x for x in rec["tracks"] if x["id"] == rec.get("selected")), None)
                if rec
                else None
            )
            est = sel["world_xyz_est"] if sel else None
            name, _, fixed = preset.partition(
                "@"
            )  # "close@drone4": that drone, flying or falling
            if name == "close" and fixed in self.drones:
                est = q(self.drones[fixed], "pos")[0].tolist()
            elif (
                name == "close" and est and drones
            ):  # the drone itself (true position), not the track's estimate (metres off along the line of sight)
                est = min(
                    drones, key=lambda p: sum((p[i] - est[i]) ** 2 for i in range(3))
                )
            self.cam = self.hv.camera_for(
                name,
                mz.tolist(),
                drones or [mz.tolist()],
                md.tolist(),
                est,
                self.cam,
                dt,
                t,
            )
            camera = self.cam
        return self.hv.document(view, seq=seq, tick=self.sim.tick, camera=camera)


def feeder(player, hud, sim, t_end, pace, stop):
    """Live: wall-clock pacing (`pace` sim s per wall s, at most; the replay may be slower)."""
    t0, seq = time.monotonic(), 0
    try:
        while not stop.is_set():
            want = min((time.monotonic() - t0) * pace, t_end)
            player.to(want)
            seq += 1
            hud.set(player.document(seq, dt=1 / 30))
            if want >= t_end:
                time.sleep(3.0)
                break
            time.sleep(1 / 30)
    finally:
        sim.close_viewer()


def cmd_play(a):
    from design_shot import watchdog
    from napl import hud as hudmod

    p = Player(a.run_dir, a.camera)
    hud, stop = hudmod.Hud(), threading.Event()
    t_end = p.total_s if a.t1 is None else min(a.t1, p.total_s)
    threading.Thread(
        target=feeder, args=(p, hud, p.sim, t_end, a.rate, stop), daemon=True
    ).start()
    watchdog(p.sim, int(t_end / max(a.rate, 0.01)) + 600)
    try:
        p.sim.show(width=W, height=H, hud=hud, clean=True)
    finally:
        stop.set()
    if t_end >= p.total_s:
        h, ok = p.finish()
        print(
            f"replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} the run's {p.native['state_hash']}"
        )


# ---- frames: xvfb, lockstep ------------------------------------------------------------------------------------------------------------
def cmd_frames(a):
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="napl-xvfb-") as fb:
        argv = [
            "xvfb-run",
            "-a",
            "-s",
            f"-screen 0 {W}x{H}x24 -fbdir {fb}",
            sys.executable,
            __file__,
            "_frames",
            a.run_dir,
            str(out),
            fb,
        ]
        argv += [
            "--t0",
            str(a.t0),
            "--fps",
            str(a.fps),
            "--camera",
            a.camera,
            "--start-index",
            str(a.start_index),
        ]
        argv += ["--t1", str(a.t1)] if a.t1 is not None else []
        subprocess.run(argv, check=True, timeout=a.timeout)


def _frames(a):
    import design_shot
    from napl import hud as hudmod

    p = Player(a.run_dir, a.camera)
    out = Path(a.out_dir)
    t1 = p.total_s if a.t1 is None else min(a.t1, p.total_s)
    n = int(round((t1 - a.t0) * a.fps))
    hud, stop = hudmod.Hud(), threading.Event()
    log = {"frames": 0, "errors": [], "hash": None}

    def run():
        try:
            while (
                hud.progress()["frames"] < 3 and not stop.is_set()
            ):  # the window is up
                time.sleep(0.2)
            pre = min(
                a.t0, 3.0
            )  # chunks join seamlessly: the eased camera runs for 3 s before the first frame (e^-4 of the previous chunk's state is left)
            p.to(a.t0 - pre)
            for j in range(int(pre * 20)):
                p.to(a.t0 - pre + (j + 1) / 20)
                p.document(-1, dt=1 / 20)
            p.to(a.t0)
            for k in range(n):
                t = a.t0 + k / a.fps
                p.to(t)
                seq = a.start_index + k
                hud.set(p.document(seq, dt=1 / a.fps))
                t_wait = time.monotonic()
                while time.monotonic() - t_wait < 20:
                    pr = hud.progress()
                    if (
                        pr["seq"] == seq
                        and pr["tick"] == p.sim.tick
                        and pr["stable"] >= 3
                    ):
                        break
                    time.sleep(0.01)
                else:
                    raise RuntimeError(
                        f"frame {k}: the window never drew seq {seq} at tick {p.sim.tick}: {hud.progress()}"
                    )
                time.sleep(0.08)  # present + the X server's copy
                img = design_shot.grab(a.fb)
                videnc.save_frame(out, f"f{seq:05d}", img)
                log["frames"] += 1
                if hud.progress()["error"] and not log["errors"]:
                    log["errors"].append(hud.progress()["error"])
            if t1 >= p.total_s - 1e-9:
                h, ok = p.finish()
                log["hash"], log["hash_matches"] = h, ok
        except Exception as e:  # reported below; the window must close either way
            log["errors"].append(f"{type(e).__name__}: {e}")
        finally:
            stop.set()
            p.sim.close_viewer()

    threading.Thread(target=run, daemon=True).start()
    design_shot.watchdog(p.sim, int(a.timeout) - 40)
    p.sim.show(width=W, height=H, hud=hud, clean=True)
    stop.set()
    time.sleep(0.2)
    meta = dict(
        run=str(p.run),
        t0=a.t0,
        t1=t1,
        fps=a.fps,
        start_index=a.start_index,
        native_hash=p.native["state_hash"],
        **log,
    )
    (out / f"meta_{a.start_index:05d}.json").write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta))
    if log["errors"] or log.get("hash_matches") is False:
        raise SystemExit(
            "frame run failed: " + "; ".join(log["errors"])
            if log["errors"]
            else "replay hash differs from the run's"
        )


# ---- encode (throwaway venv: av + Pillow) ----------------------------------------------------------------------------------------------
def cmd_encode(a):
    names = [(d, s) for d in a.dirs for s in videnc.frame_stems(d)]
    out = Path(a.out)
    enc = videnc.Encoder(
        out, W, H, a.fps, gpl=a.gpl
    )  # GPU (VA) when this host has it, else PyAV software
    for d, s in names:
        enc.write(videnc.load_array(d, s))
    enc.close()
    print(
        f"wrote {out}: {len(names)} frames, {out.stat().st_size / 1e6:.1f} MB, encoder {enc.backend}"
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(s, frames=False):
        s.add_argument(
            "--keep-frames",
            action="store_true",
            help="also write a PNG per frame (default: raw .rgz only)",
        )
        s.add_argument(
            "--camera", default="overview", help="preset or schedule (see above)"
        )
        s.add_argument(
            "--t1",
            type=float,
            default=None,
            help="end time, sim s (default: the whole run)",
        )
        if frames:
            s.add_argument("--t0", type=float, default=0.0)
            s.add_argument("--fps", type=int, default=VID_FPS)
            s.add_argument(
                "--start-index",
                type=int,
                default=0,
                help="number of the first PNG (chunks continue the numbering)",
            )

    s = sub.add_parser("play")
    s.add_argument("run_dir")
    s.add_argument("--rate", type=float, default=1.0)
    common(s)
    s = sub.add_parser("frames")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument(
        "--timeout",
        type=float,
        default=580,
        help="wall seconds before the chunk is abandoned (outer limit: 600)",
    )
    common(s, True)
    s = sub.add_parser("_frames")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument("fb")
    s.add_argument("--timeout", type=float, default=580)
    common(s, True)
    s = sub.add_parser("encode")
    s.add_argument("dirs", nargs="+")
    s.add_argument("out")
    s.add_argument("--fps", type=int, default=VID_FPS)
    s.add_argument(
        "--gpl",
        action="store_true",
        help="ffmpeg VAAPI first and libx264 in the software fallback (GPL); default is GStreamer VA (LGPL), then openh264 / VP9",
    )
    a = ap.parse_args()
    videnc.keep_from(a)
    if a.cmd == "encode":
        cmd_encode(a)
    elif a.cmd == "_frames":
        _frames(a)
    else:
        globals()["cmd_" + a.cmd](a)


if __name__ == "__main__":
    main()
