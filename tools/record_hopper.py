"""Recording harness for the hopper showcase video: the Grasshopper-style EDF jet-vane VTVL hopper built from an EMPTY design in the viewer, wired at pin level,
programmed and flown. Media is never committed (frames and the webm stay on the host). The video is made in a THROWAWAY venv with `av` + Pillow.

Modelled on tools/record_turret.py (which it imports for the generic parts: the request runner, the design comparison) and tools/record_sim.py; nothing
of those two files is edited.

    explore OPLOG --expect ASM [--save OUT.toml]   no window: the op log through napl.design.viewer.Backend, every answer printed, the built design compared
                                                   with ASM (assemblies/hopper.toml): same instances / mates / wires / devices / poses / mass, DRC
    build   OPLOG OUT_DIR --expect ASM [--save OUT.toml]
                                                   xvfb + the viewer on an EMPTY design; every step sent through the Backend the window talks to; screen grabbed
                                                   (~12 fps) -> OUT_DIR/f*.png + timeline.json (the window is W x (H - BAR); the caption strip is added by `encode`)
    frames  RUN_DIR OUT_DIR --key divert25 [--t0 --t1 --fps --start-index]
                                                   xvfb + the viewer on the REPLAY of a `napl program` run (examples/hopper/*.scenario.toml), clean mode + the
                                                   hopper HUD (telemetry.jsonl) + landing-pad markings + the camera of docs/showcase/hopper/sim_shots.json;
                                                   one PNG per video frame, stepped in lockstep with the window; the last chunk checks the replay hash
    encode  OUT.webm --build DIR --sim KEY=FRAMES_DIR ...   title / chapter / end cards, the build with captions (per-step speed), the sim frames with captions;
                                                   VP9 only (libx264 is GPL: not used)

Op log (docs/showcase/hopper/oplog.json): record_turret's steps ({seg, title, why, speed?, dwell?, reqs | macro: "place"}) plus references into the reference
assembly, so the long provenance tables are not retyped: macro place `ref_mate` (mate name + joint), design_edit args `ref` {params, device_maps, devices,
offboard, place}, flatten.wire `ref_wire` (all fields of that wire). See docs/showcase/hopper/README.md.
"""

import argparse
import bisect
import copy
import json
import math
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
W, H, BAR = (
    1600,
    900,
    108,
)  # output; the build window is W x (H - BAR) with the caption strip under it
FPS1 = 12  # build screen grabs per second
FPS = 20  # output
PACE = {
    "LIBRARY": 2.5,
    "BUILD": 2.5,
    "WIRING": 1.5,
    "PROGRAM": 1.5,
}  # speed-up of the build segments on top of the op log's per-step speed (3-5 min video)
SHOWCASE = REPO / "docs/showcase/hopper"
BLANK = '<mujoco model="blank"><worldbody><geom name="floor" type="plane" size="3 3 0.1"/></worldbody></mujoco>'
NAME = "hopper"


# ---- op log -> plain requests ---------------------------------------------------------------------------------------------------------
def resolve(steps, ref):
    """Steps with the references into the reference Design `ref` replaced by the literal requests."""
    insts = {i["id"]: i for i in ref.doc["instances"]}
    mates = {m["name"]: m for m in ref.doc["mates"]}
    wires = {w["name"]: w for w in ref.doc["wires"]}
    devs = {d["name"]: d for d in ref.doc["devices"]}
    out = []
    for st in steps:
        st = copy.deepcopy(st)
        if st.get("ref_mate"):
            m = mates[st.pop("ref_mate")]
            st["mate"] = m["name"]
            st["k"] = m.get("rotation", 0)  # the snap's rotation alternative, pinned
            if m.get("joint"):
                st["joint"] = m["joint"]
        for rq in st.get("reqs", []) + st.get("post", []):
            a = rq["args"]
            r = a.pop("ref", None)
            if r:
                ops = []
                for iid in r.get("offboard", []):
                    ops.append(
                        {
                            "op": "add",
                            "part": insts[iid]["part"],
                            "id": iid,
                            "offboard": True,
                        }
                    )
                for iid in r.get("place", []):
                    ops.append(
                        {
                            "op": "place",
                            "id": iid,
                            "pos": insts[iid]["pos"],
                            "rpy": insts[iid].get("rpy", [0, 0, 0]),
                        }
                    )
                for iid in r.get("params", []):
                    for k, v in insts[iid]["params"].items():
                        ops.append({"op": "set", "id": iid, "param": k, "value": v})
                for n in (
                    ref.doc["device_maps"]
                    if r.get("device_maps") == "all"
                    else r.get("device_maps", [])
                ):
                    ops.append(
                        {
                            "op": "device_map",
                            "name": n,
                            "map": ref.doc["device_maps"][n],
                        }
                    )
                for n in devs if r.get("devices") == "all" else r.get("devices", []):
                    ops.append({"op": "device", "device": devs[n]})
                a["ops"] = ops
            if a.get("ref_wire"):
                w = wires[a.pop("ref_wire")]
                a.update(
                    {
                        k: w[k]
                        for k in (
                            "from",
                            "to",
                            "part",
                            "length",
                            "conductors",
                            "included_in",
                            "from_pins",
                            "to_pins",
                            "name",
                        )
                        if k in w
                    }
                )
        out.append(st)
    return out


def flat_steps(steps):
    """Macro steps expanded (record_turret.expand) so run_steps sees plain requests."""
    from record_turret import expand

    out = []
    for st in steps:
        st = dict(
            st, reqs=expand(st) + st.get("post", [])
        )  # post: requests after the placement
        st.pop("macro", None)
        out.append(st)
    return out


def load_steps(oplog, expect, sel=""):
    """-> (steps, reference Design, index of the first step). sel = "LO:HI": only steps[LO:HI] (a part of the build: recordings stay under 10 minutes)."""
    from napl.design import Design

    ref = Design.open(expect)
    steps = flat_steps(resolve(json.loads(Path(oplog).read_text())["steps"], ref))
    lo, _, hi = sel.partition(":")
    lo = int(lo or 0)
    return (steps[lo : int(hi)] if hi else steps[lo:]), ref, lo


def new_design(ref, load=None):
    from napl.design import Design

    if load:  # the design the previous part of the build saved
        return Design.open(load)
    return Design.new(
        NAME,
        REPO / "parts",
        description="EDF VTVL hopper built from an empty design in the viewer",
    )


def cmd_explore(a):
    from napl.design.viewer import Backend
    from record_turret import compare_designs, run_steps

    steps, ref, _ = load_steps(a.oplog, a.expect, a.steps)
    d = new_design(ref, a.load)
    b = Backend(lambda: d, Path(tempfile.mkdtemp(prefix="napl-explore-")))
    run_steps(b, d, steps, save_to=a.save or "hopper_built.toml")
    if a.save:
        d.save(a.save)
    diffs, info = compare_designs(d, a.expect)
    print(json.dumps(info))
    print(
        "DIFFERENCES:" if diffs else "final design == expected (parts, poses, mass)",
        *diffs,
        sep="\n  ",
    )
    raise SystemExit(1 if diffs else 0)


# ---- build under xvfb --------------------------------------------------------------------------------------------------------------------
def xvfb(argv_tail, w, h, timeout):
    with tempfile.TemporaryDirectory(prefix="napl-xvfb-") as fb:
        argv = ["xvfb-run", "-a", "-s", f"-screen 0 {w}x{h}x24 -fbdir {fb}"]
        subprocess.run(
            argv + [sys.executable, __file__] + argv_tail + ["--fb", fb],
            check=True,
            timeout=timeout,
        )


def cmd_build(a):
    Path(a.out_dir).mkdir(parents=True, exist_ok=True)
    tail = ["_build", a.oplog, a.out_dir, "--expect", a.expect, "--steps", a.steps]
    tail += ["--save", a.save] if a.save else []
    tail += ["--load", a.load] if a.load else []
    tail += ["--final"] if a.final else []
    tail += ["--timeout", str(a.timeout)]
    xvfb(tail, W, H - BAR, a.timeout + 30)


def _build(a):
    import design_shot
    from napl import Sim, _napl
    from napl.design.viewer import Backend
    from record_turret import compare_designs, run_steps

    out, fb = Path(a.out_dir), a.fb
    steps, ref, first = load_steps(a.oplog, a.expect, a.steps)
    d = new_design(ref, a.load)
    work = Path(tempfile.mkdtemp(prefix="napl-rec-"))
    b = Backend(lambda: d, work / "build")
    b.auto_thumbs = True
    b.thumbs.start()
    b.thumbs.wait(900)
    blank = work / "blank.xml"
    blank.write_text(BLANK)
    sim = Sim.load(blank, host=True)
    t0 = time.time()
    frames, timeline, stop = [], [], threading.Event()
    q = queue.Queue(maxsize=64)  # PNG encoding off the grabbing thread

    def writer():
        while True:
            item = q.get()
            if item is None:
                return
            k, img = item
            (out / f"f{k:05d}.png").write_bytes(_napl.encode_png(img))

    writers = [threading.Thread(target=writer, daemon=True) for _ in range(3)]
    for w_ in writers:
        w_.start()

    def grabber():
        k = 0
        while not stop.is_set():
            t = time.time() - t0
            try:
                img = design_shot.xwd_rgba(Path(fb) / "Xvfb_screen0")
            except Exception:
                time.sleep(0.05)
                continue
            q.put((k, img))
            frames.append(round(t, 3))
            k += 1
            time.sleep(max(0.0, 1 / FPS1 - (time.time() - t0 - t)))

    def states():
        return sum(r["tool"] == "state" for r, _ in b.log)

    def session():
        while not b.log:
            time.sleep(0.2)
        time.sleep(1.5)

        def on_step(rec):
            rec["t0"], rec["t1"] = round(rec["t0"] - t0, 3), round(rec["t1"] - t0, 3)
            timeline.append(rec)

        run_steps(
            b,
            d,
            steps,
            window={"states": states},
            on_step=on_step,
            save_to=a.save or str(work / "hopper_built.toml"),
        )
        time.sleep(1.0)
        stop.set()
        sim.close_viewer()

    def guarded():
        try:
            session()
        except BaseException as e:
            timeline.append(dict(error=f"{type(e).__name__}: {e}"))
            raise
        finally:
            stop.set()
            sim.close_viewer()

    threading.Thread(target=grabber, daemon=True).start()
    threading.Thread(target=guarded, daemon=True).start()
    design_shot.watchdog(sim, int(a.timeout) - 40)
    sim.show(width=W, height=H - BAR, design=b)
    stop.set()
    time.sleep(0.3)
    for _ in writers:
        q.put(None)
    for w_ in writers:
        w_.join()
    done = len(timeline) == len(steps) and not any("error" in t for t in timeline)
    diffs, info = compare_designs(d, a.expect) if done and a.final else ([], {})
    (out / "timeline.json").write_text(
        json.dumps(
            dict(
                first_step=first,
                frames=frames,
                steps=timeline,
                complete=done,
                differences=diffs,
                info=info,
                fps=FPS1,
            ),
            indent=1,
        )
    )
    if a.save and done:
        d.save(a.save)
    print(
        f"{len(frames)} frames, {len(timeline)}/{len(steps)} steps; {info}; differences: {diffs}"
    )
    if not done or diffs:
        raise SystemExit("incomplete, or the built design differs from --expect")
    if not a.final:
        print(
            "part saved; the comparison with --expect is made by the last part (--final)"
        )


# ---- the simulation: replay of a run + the hopper HUD ------------------------------------------------------------------------------------
PAD_R, TOL_R = (
    0.9,
    0.3,
)  # landing-pad radius and the mission's 0.3 m acceptance ring (examples/hopper/*.scenario.toml: lands_on_the_second_pad)


def pad_lines(H_, centres):
    """The landing-pad markings as HUD world lines (ground circles and a cross at each pad). They are drawn by the viewer's overlay, not put in the
    scene: pad geoms in the scene change the replay state hash (tools/_probe_hash.py: divert25 8fefff2e164b775e became d961148e2c822d3b, in geom group 0
    and in group 5 alike), the overlay leaves the world untouched."""
    out = []
    for x, y in centres:

        def circle(r, col, lw, n=72):
            out.append(
                H_.wline(
                    [
                        (
                            x + r * math.cos(2 * math.pi * i / n),
                            y + r * math.sin(2 * math.pi * i / n),
                            0.012,
                        )
                        for i in range(n + 1)
                    ],
                    col=col,
                    lw=lw,
                )
            )

        circle(PAD_R, (240, 240, 245, 255), 4.0)
        circle(PAD_R - 0.12, (200, 205, 215, 150), 1.5)
        circle(TOL_R, (64, 217, 115, 255), 3.0)  # the mission's 0.3 m acceptance ring
        for a_ in (0.0, math.pi / 2):
            c, s = math.cos(a_), math.sin(a_)
            out.append(
                H_.wline(
                    [
                        (x - (PAD_R - 0.2) * c, y - (PAD_R - 0.2) * s, 0.012),
                        (x + (PAD_R - 0.2) * c, y + (PAD_R - 0.2) * s, 0.012),
                    ],
                    col=(240, 240, 245, 210),
                    lw=2.0,
                )
            )
    return out


def smooth(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


def lerp_rec(a, b, f):
    if isinstance(a, dict):
        return {k: lerp_rec(a[k], b[k], f) if k in b else a[k] for k in a}
    if isinstance(a, (int, float)) and not isinstance(a, bool):
        return a + (b - a) * f
    return a


class Player:
    """A run's replay.naplog applied tick-exactly to a fresh World on a Host (as tools/record_sim.Player), the pad markings patched into a copy of the run's
    scene, and the run's telemetry at any sim time."""

    def __init__(self, run_dir, key):
        from napl import Sim
        from screenshots import setattrs

        self.run = Path(run_dir).resolve()
        self.key = key
        self.native = json.loads((self.run / "native.json").read_text())
        self.recs = setattrs((self.run / "replay.naplog").read_bytes())
        self.tel = [
            json.loads(ln)
            for ln in (self.run / "telemetry.jsonl").read_text().split("\n")
            if ln
        ]
        self.tt = [r["t"] for r in self.tel]
        res = json.loads((self.run / "result.json").read_text())
        import tomllib

        # result.json carries no parameter values: the pad offset is the scenario's [[parameters]] default (dx_m, dy_m), as flown
        sc = tomllib.loads(Path(res["scenario"]).read_text())
        pr = {p["name"]: p.get("default") for p in sc.get("parameters", [])}
        self.par = pr
        xml = (self.run / "model" / "scene.xml").read_text()
        m = re.search(r'<body name="frame" pos="([^"]*)"', xml)
        self.p0 = [float(v) for v in m.group(1).split()] if m else [0.0, 0.0, 0.0]
        dx, dy = float(pr.get("dx_m", 0.0)), float(pr.get("dy_m", 0.0))
        self.pads = [(self.p0[0], self.p0[1]), (self.p0[0] + dx, self.p0[1] + dy)]
        self.sim = Sim.load(self.run / "model" / "scene.xml", host=True)
        self.dt, self.i = self.native["dt"], 0
        self.total_s = self.native["tick"] * self.dt
        self.dz = self.p0[2] - self.tel[0]["truth"]["alt_m"]
        shots = json.loads((SHOWCASE / "sim_shots.json").read_text())[key]
        self.keys = shots["keys"]
        self.caps = json.loads((SHOWCASE / "sim_captions.json").read_text()).get(
            key, []
        )
        self.trail = [
            [
                self.pads[0][0] + r["truth"]["x_m"],
                self.pads[0][1] + r["truth"]["y_m"],
                r["truth"]["alt_m"] + self.dz,
            ]
            for r in self.tel
        ]

    @property
    def t(self):
        return self.sim.tick * self.dt

    def to(self, t):
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
        for tick, *r in self.recs[self.i :]:
            self._adv(tick)
            self.sim.set(*r)
        self.i = len(self.recs)
        self._adv(self.native["tick"])
        h = f"{self.sim.state_hash():016x}"
        return h, h == self.native["state_hash"]

    def rec(self, t):
        """Telemetry at sim time t: the two records around it, numbers interpolated."""
        j = max(0, min(len(self.tel) - 2, bisect.bisect_right(self.tt, t) - 1))
        a, b = self.tel[j], self.tel[j + 1]
        f = max(0.0, min(1.0, (t - a["t"]) / max(b["t"] - a["t"], 1e-9)))
        return lerp_rec(a, b, f)

    # ---- camera: keyframes of (target, yaw, pitch, dist), eased between neighbours
    def target(self, spec, pos):
        if spec == "veh":
            return pos
        if spec == "pad_a":
            return [self.pads[0][0], self.pads[0][1], 0.3]
        if spec == "pad_b":
            return [self.pads[1][0], self.pads[1][1], 0.3]
        if spec == "mid":
            return [
                (self.pads[0][0] + self.pads[1][0]) / 2,
                (self.pads[0][1] + self.pads[1][1]) / 2,
                6.0,
            ]
        return list(spec)

    def camera(self, t, pos):
        ks = self.keys
        j = max(0, bisect.bisect_right([k["t"] for k in ks], t) - 1)
        k0 = ks[j]
        k1 = ks[min(j + 1, len(ks) - 1)]
        u = 1.0 if k1["t"] == k0["t"] else smooth((t - k0["t"]) / (k1["t"] - k0["t"]))
        if j == len(ks) - 1:
            u = 0.0
        a0, a1 = self.target(k0["target"], pos), self.target(k1["target"], pos)
        tgt = [a0[i] + (a1[i] - a0[i]) * u for i in range(3)]
        # `spin`: extra degrees per sim second on top of the keyed yaw (a continuous orbit inside one key interval)
        yaw = math.radians(
            k0["yaw"]
            + (k1["yaw"] - k0["yaw"]) * u
            + k0.get("spin", 0.0) * (t - k0["t"])
        )
        dist = k0["dist"] + (k1["dist"] - k0["dist"]) * u
        tgt[2] -= (
            0.07 * dist
        )  # look a little below the subject: it sits above the caption strip
        return dict(
            target=tgt,
            yaw=yaw,
            pitch=math.radians(k0["pitch"] + (k1["pitch"] - k0["pitch"]) * u),
            dist=dist,
        )

    # ---- the HUD document
    def document(self, seq, speed=1.0):
        from napl import hud as H_

        t = self.t
        r = self.rec(t)
        pos = [float(v) for v in self.sim.query("/world/frame", "pos")[0]]
        cam = self.camera(t, pos)
        est, tru, va, st = r["est"], r["truth"], r["vanes"], r["state"]

        def f(
            v, spec, unit
        ):  # the estimator publishes null until the pad calibration is done
            return "    --" if v is None else format(v, spec) + unit

        gs = None if est["vx_ms"] is None else math.hypot(est["vx_ms"], est["vy_ms"])
        bx = [self.pads[1][0] - pos[0], self.pads[1][1] - pos[1]]
        rng = math.hypot(*bx)
        items = []
        stages = [
            "PAD",
            "READY",
            "SPOOL",
            "CLIMB",
            "TRANSIT",
            "HOVER",
            "DESCENT",
            "FINAL",
            "TOUCHDOWN",
            "LANDED",
        ]
        act = (
            stages.index(st) if st in stages else len(stages) - 1
        )  # ABORT_LAND / FAULT: shown on the status line
        items.append(
            H_.tabs(
                stages,
                act,
                at=(20, 20),
                size=15,
                gap=18,
                pad=12,
                status=f"flight state {st}   |   t = {t:5.1f} s"
                + (f"   |   slow motion 1/{1 / speed:g}" if speed < 0.99 else ""),
            )
        )
        items.append(
            H_.kv(
                [
                    ("ALTITUDE (est)", f(est["alt_m"], "6.2f", " m")),
                    ("VERTICAL SPEED", f(est["vz_ms"], "+6.2f", " m/s")),
                    ("GROUND SPEED", f(gs, "6.2f", " m/s")),
                    ("RANGE TO PAD B", f"{rng:6.2f} m"),
                    ("TILT", f(est["tilt_deg"], "6.1f", " deg")),
                    ("THROTTLE", f"{100 * r['throttle']:5.0f} %"),
                    ("THRUST", f"{r['thrust_N']:6.1f} N"),
                    ("FAN", f"{r['motor']['rpm']:6.0f} rpm"),
                    (
                        "BATTERY",
                        f"{r['battery']['v']:5.1f} V {r['battery']['i_a']:4.0f} A",
                    ),
                    ("CHARGE LEFT", f"{100 * r['battery']['soc']:5.0f} %"),
                ],
                at=(W - 20, 20),
                a="tr",
                size=17,
                gap=36,
                pad=14,
            )
        )
        # four vane gauges: commanded (goal) angle on a +-17 deg bar, the measured one as the fill
        gw, gh, lim = 300, 150, 17.0
        gi = [H_.rect((0, 0), (gw, gh), fill=(0, 0, 0, 228))]
        names = ["V1 +x", "V2 +y", "V3 -x", "V4 -y"]
        for k in range(4):
            y = 34 + k * 28
            ang = va[f"a{k + 1}_deg"]
            gi.append(H_.text(names[k], (12, y - 9), size=15, col=H_.DIM))
            x0, x1 = 80, 220
            xm = (x0 + x1) / 2
            half = (x1 - x0) / 2
            xa = xm + half * max(-1, min(1, ang / lim))
            gi.append(
                H_.rect(
                    (x0, y - 6),
                    (x1 - x0, 12),
                    fill=(40, 44, 52, 255),
                    line=(90, 95, 105, 255),
                )
            )
            gi.append(
                H_.rect(
                    (min(xm, xa), y - 5),
                    (max(abs(xa - xm), 1.5), 10),
                    fill=H_.CYAN if abs(ang) < 0.85 * lim else H_.AMBER,
                )
            )
            gi.append(
                H_.line([(xm, y - 9), (xm, y + 9)], col=(200, 205, 215, 255), lw=1.0)
            )
            gi.append(
                H_.text(
                    f"{ang:+5.1f}",
                    (gw - 12, y - 9),
                    a="tr",
                    size=15,
                    col=H_.WHITE,
                    font="mono",
                )
            )
        gi.insert(
            1,
            H_.text("JET VANE ANGLES   limit +-17 deg", (12, 8), size=13, col=H_.AMBER),
        )
        items.append(
            H_.group(gi, at=(20, H - BAR - 20), size=(gw, gh), space=(gw, gh), a="bl")
        )
        # altitude profile (whole flight, a cursor at t) and the ground track
        pw, ph = 330, 150
        tmax = self.tel[-1]["t"]
        amax = max(max(x["truth"]["alt_m"] for x in self.tel), 1.0) * 1.1
        prof = [H_.rect((0, 0), (pw, ph), fill=(0, 0, 0, 228))]
        prof.append(H_.text("ALTITUDE vs TIME", (10, 6), size=13, col=H_.AMBER))
        pts = [
            (
                12 + (pw - 24) * x["t"] / tmax,
                ph - 14 - (ph - 40) * x["truth"]["alt_m"] / amax,
            )
            for x in self.tel[:: max(1, len(self.tel) // 160)]
        ]
        prof.append(H_.line(pts, col=(120, 130, 150, 255), lw=1.5))
        done = [
            p
            for p, x in zip(pts, self.tel[:: max(1, len(self.tel) // 160)])
            if x["t"] <= t
        ]
        if len(done) > 1:
            prof.append(H_.line(done, col=H_.CYAN, lw=2.5))
        cx = 12 + (pw - 24) * min(t, tmax) / tmax
        prof.append(
            H_.line([(cx, 26), (cx, ph - 10)], col=(255, 255, 255, 120), lw=1.0)
        )
        gw2 = 170
        track = [H_.rect((0, 0), (gw2, ph), fill=(0, 0, 0, 228))]
        track.append(
            H_.text("GROUND TRACK  (top view)", (10, 6), size=13, col=H_.AMBER)
        )
        xs = [self.pads[0][0], self.pads[1][0]]
        ys = [self.pads[0][1], self.pads[1][1]]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 4.0) * 1.35
        cxm, cym = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2

        def tp(x, y):
            return (
                gw2 / 2 + (x - cxm) / span * (gw2 - 24),
                ph / 2 + 8 - (y - cym) / span * (ph - 44),
            )

        for k, p in enumerate(self.pads):
            c = tp(*p)
            track.append(
                H_.rect(
                    (c[0] - 6, c[1] - 6),
                    (12, 12),
                    fill=None,
                    line=H_.GREEN if k else H_.WHITE,
                    lw=2.0,
                    r=6,
                )
            )
            track.append(
                H_.text(
                    "AB"[k],
                    (c[0] + 9, c[1] - 8),
                    size=13,
                    col=H_.GREEN if k else H_.WHITE,
                )
            )
        tr_pts = [tp(p[0], p[1]) for p, x in zip(self.trail, self.tel) if x["t"] <= t]
        if len(tr_pts) > 1:
            track.append(
                H_.line(tr_pts[:: max(1, len(tr_pts) // 150)], col=H_.CYAN, lw=2.0)
            )
        c = tp(pos[0], pos[1])
        track.append(H_.rect((c[0] - 4, c[1] - 4), (8, 8), fill=H_.AMBER, r=4))
        items.append(
            H_.group(
                prof,
                at=(W - 20 - gw2 - 12, H - BAR - 20),
                size=(pw, ph),
                space=(pw, ph),
                a="br",
            )
        )
        items.append(
            H_.group(
                track,
                at=(W - 20, H - BAR - 20),
                size=(gw2, ph),
                space=(gw2, ph),
                a="br",
            )
        )
        # world marks: pads, the path so far, the altitude drop line
        world = pad_lines(
            H_, self.pads
        )  # drawn first, so the panels stay on top of the world lines
        for k, p in enumerate(self.pads):
            world.append(
                H_.wtext(
                    (  # on the rim nearest the camera, so the vehicle on the pad never hides it
                        p[0] + PAD_R * math.cos(cam["yaw"]),
                        p[1] + PAD_R * math.sin(cam["yaw"]),
                        0.05,
                    ),
                    "PAD A  (take-off)" if k == 0 else "PAD B  (target, 0.3 m ring)",
                    off=(0, -14),
                    size=15,
                    col=H_.GREEN if k else H_.WHITE,
                )
            )
        past = [p for p, x in zip(self.trail, self.tel) if x["t"] <= t]
        if len(past) > 1:
            world.append(
                H_.wline(
                    past[:: max(1, len(past) // 400)], col=(60, 220, 255, 190), lw=2.0
                )
            )
        if pos[2] > 0.6:
            world.append(
                H_.wline([pos, [pos[0], pos[1], 0.0]], col=(255, 255, 255, 90), lw=1.0)
            )
            world.append(
                H_.wtext(
                    [pos[0], pos[1], pos[2] + 0.35],
                    f(est["alt_m"], ".1f", " m")
                    + "  "
                    + f(est["vz_ms"], "+.1f", " m/s"),
                    off=(0, -6),
                    size=15,
                    col=H_.CYAN,
                )
            )
        return H_.doc(world + items, seq=seq, tick=self.sim.tick, camera=cam)


def cmd_frames(a):
    Path(a.out_dir).mkdir(parents=True, exist_ok=True)
    tail = [
        "_frames",
        a.run_dir,
        a.out_dir,
        "--key",
        a.key,
        "--t0",
        str(a.t0),
        "--fps",
        str(a.fps),
        "--start-index",
        str(a.start_index),
        "--timeout",
        str(a.timeout),
    ]
    tail += ["--t1", str(a.t1)] if a.t1 is not None else []
    xvfb(tail, W, H, a.timeout + 30)


def _frames(a):
    import design_shot
    from napl import _napl
    from napl import hud as hudmod

    p = Player(a.run_dir, a.key)
    out = Path(a.out_dir)
    t1 = p.total_s if a.t1 is None else min(a.t1, p.total_s)
    n = int(round((t1 - a.t0) * a.fps))
    speed = (
        20.0 / a.fps if a.fps >= 20 else 1.0
    )  # slow motion only (fps above 20 = 1 / (fps / 20))
    hud, stop = hudmod.Hud(), threading.Event()
    log = {"frames": 0, "errors": [], "hash": None}
    q = queue.Queue(maxsize=64)

    def writer():
        while True:
            item = q.get()
            if item is None:
                return
            (out / f"f{item[0]:05d}.png").write_bytes(_napl.encode_png(item[1]))

    writers = [threading.Thread(target=writer, daemon=True) for _ in range(3)]
    for w_ in writers:
        w_.start()

    def run():
        try:
            while hud.progress()["frames"] < 3 and not stop.is_set():
                time.sleep(0.2)
            p.to(a.t0)
            for k in range(n):
                p.to(a.t0 + k / a.fps)
                seq = a.start_index + k
                hud.set(p.document(seq, 1.0 / (a.fps / 20) if a.fps > 20 else 1.0))
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
                time.sleep(0.05)
                q.put((seq, design_shot.xwd_rgba(Path(a.fb) / "Xvfb_screen0")))
                log["frames"] += 1
                if hud.progress()["error"] and not log["errors"]:
                    log["errors"].append(hud.progress()["error"])
            if t1 >= p.total_s - 1e-9:
                h, ok = p.finish()
                log["hash"], log["hash_matches"] = h, ok
        except Exception as e:
            log["errors"].append(f"{type(e).__name__}: {e}")
        finally:
            stop.set()
            p.sim.close_viewer()

    threading.Thread(target=run, daemon=True).start()
    design_shot.watchdog(p.sim, int(a.timeout) - 40)
    p.sim.show(width=W, height=H, hud=hud, clean=True)
    stop.set()
    for _ in writers:
        q.put(None)
    for w_ in writers:
        w_.join()
    meta = dict(
        run=str(p.run),
        key=a.key,
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


# ---- encode (throwaway venv: av + Pillow) --------------------------------------------------------------------------------------------------
RESULTS = [  # examples/hopper/README.md "Monte Carlo (held-out seeds, flown once)", final v3, report seeds 13000+
    (
        "hop10  (10 m hop, hover, land)",
        "150",
        "99.3 %  [96.3, 99.9]",
        "0.27 / 0.30 / 0.34",
        "0.03 / 0.06 / 0.08",
    ),
    (
        "divert25  (25 m, 10 m divert, wind up to 3 m/s)",
        "300",
        "99.3 %  [97.6, 99.8]",
        "0.24 / 0.34 / 0.48",
        "0.05 / 0.09 / 0.15",
    ),
    (
        "gusts  (5 m/s + servo fault + 1.5 V step)",
        "200",
        "87.5 %  [82.2, 91.4]",
        "0.24 / 0.29 / 0.34",
        "0.11 / 0.33 / 10.4",
    ),
]


def cmd_encode(a):
    import av
    from PIL import Image, ImageDraw, ImageFont

    FONT = "/usr/share/fonts/truetype/freefont/FreeSans.ttf"
    BOLD = "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf"
    MONO = "/usr/share/fonts/truetype/freefont/FreeMono.ttf"
    f_s, f_b, f_t, f_h, f_m, f_sm, f_c = (
        ImageFont.truetype(p, s)
        for p, s in (
            (FONT, 19),
            (BOLD, 23),
            (BOLD, 46),
            (BOLD, 14),
            (MONO, 20),
            (FONT, 16),
            (FONT, 22),
        )
    )
    codecs = [c for c in ("libvpx-vp9",) if c in av.codecs_available]
    if not codecs:
        raise SystemExit("no VP9 encoder in this PyAV (libx264 is GPL: not used)")
    out = Path(a.out)
    cont = av.open(str(out), "w")
    st = cont.add_stream("libvpx-vp9", rate=FPS)
    st.width, st.height, st.pix_fmt = W, H, "yuv420p"
    st.options = {"crf": "31", "b": "0", "cpu-used": "2", "row-mt": "1"}
    marks, nframes = [], [0]

    def emit(img):
        for pk in st.encode(av.VideoFrame.from_image(img.convert("RGB"))):
            cont.mux(pk)
        nframes[0] += 1

    def wrap(text, font, width, dr):
        lines, cur = [], ""
        for w in text.split():
            if dr.textlength((cur + " " + w).strip(), font=font) > width and cur:
                lines.append(cur)
                cur = w
            else:
                cur = (cur + " " + w).strip()
        return lines + ([cur] if cur else [])

    def card(title, lines, secs, label=None, extra=None):
        marks.append((nframes[0] / FPS, label or title))
        img = Image.new("RGB", (W, H), (10, 11, 13))
        dr = ImageDraw.Draw(img)
        dr.line([(100, 230), (280, 230)], fill=(255, 196, 60), width=3)
        y = 258
        for sub in wrap(title.upper(), f_t, W - 200, dr):
            dr.text((100, y), sub, font=f_t, fill=(245, 245, 248))
            y += 60
        y += 14
        for ln in lines:
            for sub in wrap(ln, f_c, W - 260, dr):
                dr.text((100, y), sub, font=f_c, fill=(185, 190, 200))
                y += 31
            y += 12
        if extra:
            extra(dr, y + 10)
        for _ in range(int(secs * FPS)):
            emit(img)

    def bar(img, seg, title, why, right=None, n=None):
        """The caption strip: black, small-caps segment tag, bold title, the reason; BAR px tall at the bottom of the frame."""
        dr = ImageDraw.Draw(img, "RGBA")
        dr.rectangle([0, H - BAR, W, H], fill=(6, 7, 9, 255))
        dr.line([(0, H - BAR), (W, H - BAR)], fill=(255, 196, 60, 255), width=2)
        dr.text(
            (28, H - BAR + 9),
            seg.upper() + (f"   {n}" if n else ""),
            font=f_h,
            fill=(255, 196, 60),
        )
        dr.text((28, H - BAR + 26), title, font=f_b, fill=(255, 255, 255))
        wd, fnt = 1230 if right else W - 56, f_s
        lines = wrap(why, fnt, wd, dr)
        if len(lines) > 2:  # a long reason: the smaller type, still two lines
            fnt = f_sm
            lines = wrap(why, fnt, wd, dr)
        for i, sub in enumerate(lines[:2]):
            dr.text((28, H - BAR + 58 + 22 * i), sub, font=fnt, fill=(190, 196, 208))
        for i, ln in enumerate(right or []):
            dr.text(
                (W - 28 - dr.textlength(ln, font=f_m), H - BAR + 14 + 24 * i),
                ln,
                font=f_m,
                fill=(150, 235, 170)
                if i == len(right) - 1 and "0 errors" in ln
                else (200, 205, 215),
            )

    card(
        "A jet-vane hopper, built and flown by an agent",
        [
            "An LLM agent (Claude) builds a Grasshopper-style vertical take-off and landing hopper from an empty design in the napl viewer: a 33 N ducted fan, four jet vanes on smart servos, a Teensy flight computer with IMU, barometer, lidar and RTK GNSS.",
            "It wires it pin by pin, reads the flight software it will run, and flies a 25 m hop with a 10 m divert onto a second pad in wind, with the HUD on the replay of the very run the kernel judged.",
            "Everything in the picture is the napl code path (Design backend, DRC, build, MuJoCo, device models). Who decided what: docs/showcase/hopper/README.md",
        ],
        7.0,
        "title",
    )

    # ---- build: grabbed frames, per-step speed
    ops = json.loads((SHOWCASE / "oplog.json").read_text())["steps"]
    last_seg, cur = None, None
    chapters = {
        "LIBRARY": (
            "1  Library and build",
            "From an empty design: every part comes out of the 686-part library with datasheet numbers, snaps to a port, and the status bar keeps mass and design-rule checks live.",
        ),
        "WIRING": (
            "2  Wiring",
            "The 2D sheet with the real connector pins of the ESC, battery, regulators, servos and flight computer; the design-rule check reads the pins; back in 3D the conductors are tubes routed around the parts.",
        ),
        "PROGRAM": (
            "3  Programming",
            "The flight software that will fly it: estimator, trajectory, vane allocation, energy and spool logic, and the scenario that judges the flight.",
        ),
        "SIMULATE": ("4  Simulation", ""),
    }
    shown = set()
    for bdir in a.build:
        bd = Path(bdir)
        tl = json.loads((bd / "timeline.json").read_text())
        ts, steps, first = tl["frames"], tl["steps"], tl.get("first_step", 0)
        if cur is None:
            cur = steps[0]
        segs, prev = [], 0.0
        for s in steps:
            s["gn"] = first + s["n"]  # the step's number in the whole op log
            # an editing choice: the op log's current value wins, times the pace of the section; no step shorter than 2.5 s on screen
            s["speed"] = ops[s["gn"] - 1].get("speed", 1.0) * PACE.get(s["seg"], 1.0)
            s["speed"] = max(1.0, min(8.0, s["speed"], (s["t1"] - s["t0"]) / 2.5))
            if s["t0"] > prev:
                segs.append((prev, s["t0"], 1.0, None))
            segs.append((s["t0"], s["t1"], s["speed"], s))
            prev = s["t1"]
        if ts[-1] > prev:
            segs.append((prev, ts[-1], 1.0, None))
        cache = {}
        for t0_, t1_, sp, s in segs:
            if s is not None:
                cur = s
                if s["seg"] != last_seg:
                    key = "LIBRARY" if s["seg"] in ("LIBRARY", "BUILD") else s["seg"]
                    if key in chapters and key not in shown and chapters[key][1]:
                        shown.add(key)
                        card(chapters[key][0], [chapters[key][1]], 3.5)
                    marks.append((nframes[0] / FPS, s["seg"]))
                    last_seg = s["seg"]
            n_out = max(1, int(math.ceil((t1_ - t0_) / sp * FPS)))
            for k in range(n_out):
                j = max(
                    0, min(len(ts) - 1, bisect.bisect_right(ts, t0_ + k * sp / FPS) - 1)
                )
                if j not in cache:
                    cache = {j: Image.open(bd / f"f{j:05d}.png").convert("RGB")}
                img = Image.new("RGB", (W, H), (6, 7, 9))
                img.paste(cache[j], (0, 0))
                c = cur
                bar(
                    img,
                    c["seg"],
                    c["title"],
                    c["why"],
                    [
                        f"{c['instances']} parts   {c['mass']:.3f} kg",
                        f"DRC {c['errors']} errors, {c['warnings']} warnings",
                    ],
                    f"step {c['gn']}/{len(ops)}" + (f"   {sp:g}x" if sp != 1 else ""),
                )
                emit(img)

    # ---- simulate
    card(
        "4  Simulation",
        [
            "The built hopper flies `examples/hopper/flight.py` through `napl program`; the kernel judges the run on its monitors. What follows is that run's replay.naplog in the viewer with the HUD: the telemetry is the flight computer's own, the camera orbits and pans around the vehicle, and the landing pads are drawn on the ground (cosmetic: the replay hash still matches the run's)."
        ],
        5.0,
        "SIMULATE",
    )
    sims = dict(x.split("=", 1) for x in a.sim)
    tail_note = []
    for key, d in sims.items():
        sd = Path(d)
        caps = json.loads((SHOWCASE / "sim_captions.json").read_text()).get(key, [])
        metas = sorted(sd.glob("meta_*.json"))
        if key != next(iter(sims)):
            card(
                {
                    "gusts": "Robustness: 5 m/s wind, gusts, a servo goes soft, the pack steps down"
                }.get(key, key),
                [
                    {
                        "gusts": "The stress case of the Monte Carlo (mission c): the flight software knows about none of it. Same hopper, same code; a 15 m hop with a 6 m divert, a steady 5 m/s wind with Dryden gusts, one vane servo losing 65 % of its torque at 14 s and a 1.5 V step in the pack voltage 3 s later."
                    }.get(key, "")
                ],
                4.0,
                key.upper(),
            )
        for mp in metas:
            m = json.loads(mp.read_text())
            fps_c, t0c, i0 = m["fps"], m["t0"], m["start_index"]
            slow, fast = fps_c / FPS, FPS / fps_c
            for k in range(m["frames"]):
                tsim = t0c + k / fps_c
                img = Image.open(sd / f"f{i0 + k:05d}.png").convert("RGB")
                cap = next((c for c in caps if c["t0"] <= tsim < c["t1"]), None)
                rate = (
                    f"slow motion 1/{slow:g}"
                    if slow > 1.01
                    else (f"{fast:g}x" if fast > 1.01 else "real time")
                )
                bar(
                    img,
                    "simulate",
                    cap["title"] if cap else "",
                    cap["text"] if cap else "",
                    [f"sim t = {tsim:5.1f} s", rate],
                )
                emit(img)
        if metas:
            tail_note.append((key, json.loads(metas[-1].read_text())))

    # ---- end card
    def table(dr, y):
        cols = [100, 700, 800, 1040, 1290]
        heads = [
            "Mission (held-out seeds, flown once)",
            "Seeds",
            "Success  [95 % Wilson]",
            "Touchdown m/s  p50/90/99",
            "Pad distance m  p50/90/99",
        ]
        for x, h in zip(cols, heads):
            dr.text((x, y), h, font=f_sm, fill=(255, 196, 60))
        y += 30
        dr.line([(100, y), (W - 100, y)], fill=(90, 95, 105), width=1)
        for row in RESULTS:
            y += 14
            for x, v in zip(cols, row):
                dr.text((x, y), v, font=f_c, fill=(235, 238, 245))
            y += 36
        y += 20
        dr.text(
            (100, y),
            "Source: examples/hopper/README.md, Monte Carlo of the simulated vehicle (physics, not hardware); report seeds 13000+, each flown once. Nothing here has been on a test stand.",
            font=f_sm,
            fill=(150, 155, 165),
        )
        y += 34
        for key, m in tail_note:
            dr.text(
                (100, y),
                f"This video: run {key}, state hash {m['native_hash']}: the replay ends on {'EXACTLY this hash (MATCH)' if m.get('hash_matches') else 'a different hash (MISMATCH)'}.",
                font=f_sm,
                fill=(150, 235, 170) if m.get("hash_matches") else (255, 120, 110),
            )
            y += 24

    card(
        "Result",
        [
            "The numbers below are the project's Monte Carlo, not the two flights in this video."
        ],
        12.0,
        "RESULT",
        table,
    )
    for pk in st.encode():
        cont.mux(pk)
    cont.close()
    print(
        f"wrote {out}: {out.stat().st_size / 1e6:.1f} MB, {nframes[0]} frames = {nframes[0] / FPS:.1f} s, {W}x{H}"
    )
    print("timeline (output seconds):", ", ".join(f"{t:.0f}s {l}" for t, l in marks))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("explore")
    s.add_argument("oplog")
    s.add_argument("--expect", required=True)
    s.add_argument("--save")
    s.add_argument("--steps", default="", help="LO:HI, only steps[LO:HI]")
    s.add_argument("--load", help="start from this saved design (the previous part)")
    for name in ("build", "_build"):
        s = sub.add_parser(name)
        s.add_argument("oplog")
        s.add_argument("out_dir")
        s.add_argument("--fb")
        s.add_argument("--expect", required=True)
        s.add_argument("--save")
        s.add_argument("--steps", default="", help="LO:HI, only steps[LO:HI]")
        s.add_argument(
            "--load", help="start from this saved design (the previous part)"
        )
        s.add_argument(
            "--final",
            action="store_true",
            help="the last part: the saved design must equal --expect",
        )
        s.add_argument("--timeout", type=float, default=560)
    for name in ("frames", "_frames"):
        s = sub.add_parser(name)
        s.add_argument("run_dir")
        s.add_argument("out_dir")
        s.add_argument("--fb")
        s.add_argument("--key", required=True)
        s.add_argument("--t0", type=float, default=0.0)
        s.add_argument("--t1", type=float, default=None)
        s.add_argument("--fps", type=int, default=FPS)
        s.add_argument("--start-index", type=int, default=0)
        s.add_argument("--timeout", type=float, default=560)
    s = sub.add_parser("encode")
    s.add_argument("out")
    s.add_argument(
        "--build",
        nargs="+",
        required=True,
        help="the build parts' frame dirs, in order",
    )
    s.add_argument(
        "--sim",
        nargs="+",
        required=True,
        help="KEY=FRAMES_DIR, in video order (the keys of sim_shots.json)",
    )
    a = ap.parse_args()
    {
        "explore": cmd_explore,
        "build": cmd_build,
        "_build": _build,
        "frames": cmd_frames,
        "_frames": _frames,
        "encode": cmd_encode,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
