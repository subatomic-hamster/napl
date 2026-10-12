"""Recording harness of the Skydio-style quadcopter end-to-end video (media is never committed: frames and the webm stay on the host). Exact commands: docs/skydio/video.md.

Reuses the existing pipelines instead of copying them:
  build / wiring / program  tools/record_turret.py's xvfb viewer capture (`build ... --open assemblies/skydio.toml`); this tool only generates the op log (`oplog`)
  sim                       the run's replay.naplog applied tick-exact to a fresh World (tools/screenshots.py Replay, like examples/skydio/render_frames.py), rendered offscreen
                            with a directed camera (orbit / chase / high / tripod pan / pair); the HUD is drawn at encode time from the run's own trace.csv / telemetry.jsonl / depth.npz
  encode                    tools/videnc.py (GPU, GStreamer VA AV1 webm; nothing GPL unless --gpl) with the cards and caption strip of tools/record_cuas_lite.py

    oplog   OUT.json              build / wiring / program op log generated from assemblies/skydio.toml (needs the Design: run it on the host)
    sim     RUN_DIR OUT_DIR       frames f*.rgz (main view) + p*.rgz (the camera's own colour view) + sim.json (camera of every frame, the replay hash check). GPU
    encode  ROOT OUT.webm         ROOT holds build/ (record_turret.py build), goal/ and follow/ (run dirs, each with sim/ from `sim`)
    png     FRAMES_DIR OUT_DIR N...   some frames as PNG, to look at
"""

import argparse
import bisect
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples" / "skydio"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import videnc

W, H = 1280, 720
FPS = 20
FPV = (320, 180)
ASM = REPO / "assemblies" / "skydio.toml"
MOUNT_DEG = {
    "goal": 10.0,
    "follow": -8.0,
}  # camera pitch over the airframe x axis (skydio_scene.tilt_camera)
MODE_TEXT = {
    "track": "FOLLOW A SAFE PRIMITIVE",
    "brake": "BRAKE",
    "escape": "ESCAPE",
    "scan": "SCAN: yaw to look",
}


# ---- the build / wiring / program op log, generated from the assembly --------------------------------------------------------------------------------
BUILD_STAGES = [  # (title, why, instance ids)
    ("X500 V2 frame: plates, arm tubes and clamps",
     "Holybro X500 V2 (assemblies/x500v2.toml, from the parts library): bottom and top carbon plates, four carbon arm tubes held by the four arm connector clamps (M3 bolts, flange locknuts).",
     ["bottom", "top", "con1", "con2", "con3", "con4", "tube1", "tube2", "tube3", "tube4"]),
    ("Landing gear, rails, battery mount, platform",
     "Landing gear, the two payload rail rods with their eight hangers, the battery mount, the payload platform board, the GPS mast and the hardware kit.",
     ["leg_f", "leg_r", "rod_l", "rod_r", "hanger1", "hanger2", "hanger3", "hanger4", "hanger5", "hanger6", "hanger7", "hanger8", "platform", "batmount", "mast", "hw"]),
    ("Propulsion: 2216 KV920 motors, 1045 props, four 20 A ESCs",
     "Four Holybro 2216 KV920 motors with 10 x 4.5 in props, each with a BLHeli-S 20 A ESC. The motor and propeller models carry the thrust and torque curves the controller is calibrated from.",
     ["mot1", "mot2", "mot3", "mot4", "prop1", "prop2", "prop3", "prop4", "esc1", "esc2", "esc3", "esc4"]),
    ("Power: 4S 5200 mAh battery, PM02 V3, PDB",
     "Tattu 4S 5200 mAh 35C pack, the Holybro PM02 V3 power module (battery sense) and the 60 A power distribution board that feeds the four ESCs.",
     ["bat", "pm02", "pdb"]),
    ("Flight controller, GPS and telemetry radio",
     "Pixhawk 6C (PX4), M8N GPS and the SiK telemetry radio. The GPS stays on the airframe but is not used by the autonomy: the mission flies GPS-denied.",
     ["fc", "gps", "sik"]),
    ("Vision payload: RealSense D435i, Raspberry Pi 5, 5 V regulator",
     "The Skydio-style addition: an Intel RealSense D435i depth camera on the nose (tilted 10 deg up in this assembly), a Raspberry Pi 5 companion computer stacked over the Pixhawk, and a Pololu D24V50F5 regulator tapped from the PDB for the Pi and the camera.",
     ["cam", "pi", "reg"]),
]  # fmt: skip
SHEETS = [  # (title, why, instances)
    ("Power: battery, PM02, PDB, four ESCs, Pixhawk power port",
     "Sheet 1. Battery to the PM02 power module to the PDB, the PDB's four XT30 outputs to the four ESCs, the PM02's regulated 5 V to the Pixhawk POWER1 port. Every connector shows its real pins.",
     ["bat", "pm02", "pdb", "esc1", "esc2", "esc3", "esc4", "fc"]),
    ("Motors: Pixhawk PWM outputs, ESC signal, three-phase leads",
     "Sheet 2. FMU outputs MAIN 1-4 to the ESC signal inputs, and the three motor phase wires of each ESC to its motor.",
     ["fc", "esc1", "esc2", "esc3", "esc4", "mot1", "mot2", "mot3", "mot4"]),
    ("Companion: regulator, Raspberry Pi 5, D435i USB, UART to the Pixhawk",
     "Sheet 3. A tap on PDB output 1 into the 5 V regulator, the regulator to the Pi 5 USB-C power input, the Pi's USB 3 port to the D435i, and the Pi GPIO UART (GPIO14 / 15 / GND) crossed to the Pixhawk TELEM2 for MAVLink. GPS and the radio sit on their own Pixhawk ports.",
     ["pdb", "reg", "pi", "cam", "fc", "gps", "sik"]),
]  # fmt: skip
SRC_PAGES = [  # (title, why, file, [scroll fraction or regex])
    ("Mapping: depth image to a voxel map and a distance field", "examples/skydio/mapping.py. Each 160 x 90 depth frame is back-projected with the estimated camera pose, carved into a rolling log-odds voxel grid (0.2 m) and turned into a Euclidean distance field. The planner reads only this map, never an obstacle position.", "examples/skydio/mapping.py", [0.0, r"def distance|def clearance"]),
    ("Planner: sampled motion primitives, flown only if the braking stop is safe", "examples/skydio/planner.py. 400+ velocity-ramp primitives per replan; one is flown only if its whole path including the braking tail stays clear of occupied voxels and inside space the camera has already seen to be free.", "examples/skydio/planner.py", [0.0, r"def plan|def _safe"]),
    ("The flight loop and the two missions", "examples/skydio/fly.py. Takeoff, a 1 s mapping hover, then every replan: depth to map, estimate, detector (follow), planner, setpoints to the controller. The scenario files judge trace.csv and re-check the replay hash.", "examples/skydio/fly.py", [0.0, r"def step"]),
    ("A scenario file: the monitors the run is judged on", "examples/skydio/follow.scenario.toml: zero contacts, clearance, the strict 3-8 m follow band, subject tracked.", "examples/skydio/follow.scenario.toml", [0.0, 0.5]),
]  # fmt: skip


def _scroll(path, pat):
    import re

    if isinstance(pat, float):
        return pat
    lines = (REPO / path).read_text().splitlines()
    k = next((i for i, ln in enumerate(lines) if re.search(pat, ln)), 0)
    return round(max(0, k - 3) / max(1, len(lines) - 40), 3)


def cmd_oplog(a):
    import hashlib
    import tomllib

    from napl.design import Design
    from napl.design.viewer import Backend

    doc = tomllib.loads(ASM.read_text())
    ids = [i["id"] for i in doc["instances"]]
    on = [i["id"] for i in doc["instances"] if not i.get("offboard")]
    staged = [x for st in BUILD_STAGES for x in st[2]]
    assert len(staged) == len(set(staged)), "an instance is in two build stages"
    assert set(staged) <= set(ids), f"unknown ids {set(staged) - set(ids)}"
    stages = [list(s) for s in BUILD_STAGES]
    stages[-1][2] = stages[-1][2] + [
        x for x in on if x not in staged
    ]  # anything not named is shown with the payload
    R = lambda tool, wait=0.0, **args: {"tool": tool, "args": args, "wait": wait}
    steps = [
        dict(seg="BUILD", title="The finished design, every part hidden", speed=1.0,
             why=f"assemblies/skydio.toml, generated by examples/skydio/build_design.py with the design API: {len(on)} parts on the airframe, {len(doc['mates'])} mates, {len(doc['wires'])} wires. The BROWSER's eye hides parts in the 3D view; they come back in the order a builder fits them.",
             reqs=[R("ui.mode", 1.0, name="build"), R("select", 0.3, instances=[]), R("ui.visibility", 1.2, visible=False), R("ui.camera", 0.8, preset="iso")]),
    ]  # fmt: skip
    shown = []
    for n, (title, why, grp) in enumerate(stages, 1):
        grp = [x for x in grp if x in ids]
        shown += grp
        reqs = [
            R("ui.visibility", 0.6, instances=grp, visible=True),
            R("ui.frame", 1.2, instances=shown),
            R(
                "select", 0.2, instances=[]
            ),  # ui.frame selects what it frames: clear after it
            R(
                "camera.orbit",
                4.2,
                yaw_deg=75,
                pitch_deg=8 if n == 1 else 0,
                seconds=4.0,
                relative=True,
            ),
        ]
        steps.append(
            dict(
                seg="BUILD",
                title=f"{n}/{len(stages)}  {title}",
                why=why,
                speed=1.0,
                dwell=0.4,
                reqs=reqs,
            )
        )
    steps.append(dict(seg="BUILD", title="Mass and DRC", speed=1.0, dwell=1.0,
                      why="The Design reports mass and the design-rule check. The 6 DRC warnings are inherited unchanged from assemblies/x500v2.toml.",
                      reqs=[R("ui.visibility", 0.5, visible=True), R("ui.frame", 1.0, instances=[]), R("design_check", 2.5)]))  # fmt: skip
    # wiring sheets: the card geometry comes from the real flatten state (cached: a full design build per assembly text)
    cpath = Path(a.out).with_suffix(".cards.json")
    sha = hashlib.sha1(ASM.read_bytes() + repr(SHEETS).encode()).hexdigest()
    cached = json.loads(cpath.read_text()) if cpath.exists() else {}
    cached = cached if cached.get("sha") == sha else {"sha": sha}
    b = Backend(lambda d=Design.open(ASM): d, Path("/tmp/napl-oplog-skydio"))
    for n, (title, why, inst) in enumerate(SHEETS, 1):
        inst = [i for i in inst if i in set(ids)]
        ports = sorted(
            {
                w[e]
                for w in doc["wires"]
                for e in ("from", "to")
                if w["from"].partition(".")[0] in inst
                and w["to"].partition(".")[0] in inst
            }
        )
        if title not in cached:
            r = json.loads(
                b(
                    json.dumps(
                        {
                            "tool": "flatten.begin",
                            "args": {"instances": inst, "ports": ports},
                        }
                    )
                )
            )
            if not r["ok"]:
                raise SystemExit(f"sheet {n}: {r.get('error')}")
            cards = r["flatten"]["cards"]
            wired = lambda c: sum(1 for p in c["pins"] if p.get("wired"))
            key = sorted(
                sorted(cards, key=wired, reverse=True)[:3], key=lambda c: c["x"]
            )
            cached[title] = [
                [round(c["x"] + c["w"] / 2), round(c["y"] + c["h"] / 2)] for c in key
            ]
            b(json.dumps({"tool": "flatten.end", "args": {}}))
            cpath.write_text(json.dumps(cached))
        reqs = [
            R("ui.mode", 1.4, name="wiring"),
            R("select", 1.0, instances=inst),
            R("flatten.begin", 3.0, instances=inst, ports=ports),
            R("ui.wire_view", 3.0, fit=True),
        ]
        reqs += [
            R("ui.wire_view", 4.4, zoom=0.9, center=c, seconds=2.4)
            for c in cached[title]
        ]
        reqs += [R("ui.wire_view", 2.4, fit=True, seconds=1.6), R("flatten.end", 1.0)]
        steps.append(
            dict(seg="WIRING", title=title, why=why, speed=1.0, dwell=0.6, reqs=reqs)
        )
    steps.append(dict(seg="WIRING", title=f"Back to 3D: the {len(doc['wires'])} wires are tubes routed around the parts", speed=1.0, dwell=0.6,
                      why="In 3D each conductor is a tube with its gauge's diameter, leaving its connector along the connector axis and routed through free space (a distance field of the assembly), never through a part.",
                      reqs=[R("ui.mode", 1.2, name="build"), R("ui.view", 0.5, emphasise_wires=True), R("select", 0.5, instances=[]), R("ui.frame", 1.2, instances=[]),
                            R("camera.orbit", 8.5, yaw_deg=300, pitch_deg=12, seconds=8.0, relative=True)]))  # fmt: skip
    for title, why, f, pats in SRC_PAGES:
        steps.append(dict(seg="PROGRAM", title=title, why=why, speed=1.0, dwell=0.4,
                          reqs=[R("ui.mode", 1.0, name="program")] + [R("ui.program", 4.2, file=f, scroll=_scroll(f, p)) for p in pats]))  # fmt: skip
    Path(a.out).write_text(
        json.dumps(
            {
                "about": "generated by tools/record_skydio.py oplog from assemblies/skydio.toml",
                "steps": steps,
            },
            indent=1,
        )
    )
    print(f"{len(steps)} steps -> {a.out}")


# ---- one run: trace, telemetry, depth ---------------------------------------------------------------------------------------------------------------
def quat_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])  # fmt: skip


class Run:
    """The files a `napl program` skydio run leaves: trace.csv (62.5 Hz), telemetry.jsonl (25 Hz, from the flight start), depth.npz, model/forest.json."""

    def __init__(self, d):
        import csv

        self.dir = Path(d)
        rows = list(csv.DictReader(open(self.dir / "trace.csv")))
        self.tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        self.tele = [json.loads(ln) for ln in open(self.dir / "telemetry.jsonl")]
        self.tt = [r["t"] for r in self.tele]
        z = np.load(self.dir / "depth.npz")
        self.dt_t, self.depth = z["t"], z["depth"]
        self.forest = json.loads((self.dir / "model" / "forest.json").read_text())
        self.mission = self.forest["mission"]
        self.summary = json.loads((self.dir / "summary.json").read_text())
        self.native = json.loads((self.dir / "native.json").read_text())
        self.t_end = self.native["tick"] * self.native["dt"]
        fl = self.tr["flying"] > 0.5
        self.t_fly0 = float(self.tr["t"][np.argmax(fl)])

    def at(self, k, t):
        return float(np.interp(t, self.tr["t"], self.tr[k]))

    def pos(self, t):
        return np.array([self.at(k, t) for k in "xyz"])

    def R(self, t):
        j = min(int(np.searchsorted(self.tr["t"], t)), len(self.tr["t"]) - 1)
        return quat_rot([self.tr[k][j] for k in ("qw", "qx", "qy", "qz")])

    def rec(self, t):
        j = bisect.bisect_right(self.tt, t) - 1
        return self.tele[j] if j >= 0 else None

    def subject_truth(self, t):
        r = self.rec(t) or self.tele[0]
        return np.array(r["subject"]["truth"]) if "subject" in r else None

    def obstacle_range(self, t, R, p):
        """Nearest depth return that is above the ground (world z of the hit > 0.4 m), m; the camera ray angles are the pinhole model of mapping.Camera approximated as
        elevation = mount + row angle, azimuth = column angle (good to a few degrees: this is a HUD number, the planner does its own geometry)."""
        j = min(int(np.searchsorted(self.dt_t, t)), len(self.dt_t) - 1)
        d = self.depth[j].astype(float)
        h, w = d.shape
        fy, fx = (
            (h / 2) / math.tan(math.radians(29.0)),
            (w / 2) / math.tan(math.radians(43.5)),
        )
        rows, cols = np.mgrid[0:h, 0:w]
        el = math.radians(MOUNT_DEG[self.mission]) + np.arctan(
            (h / 2 - 0.5 - rows) / fy
        )
        az = np.arctan((cols - (w / 2 - 0.5)) / fx)
        dirb = np.stack(
            [np.cos(el) * np.cos(az), -np.cos(el) * np.sin(az), np.sin(el)], -1
        )
        zhit = p[2] + d * (dirb @ R.T)[..., 2]
        ok = (d > 0.3) & (d < 9.9) & (zhit > 0.4)
        return float(d[ok].min()) if ok.any() else None


# ---- the director: camera moves ----------------------------------------------------------------------------------------------------------------------
def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def shots_for(run):
    """[(t_start, kind, kwargs)]: orbit around the quad on the pad, chase, a high oblique, tripod pans at the hard places, then a closing orbit. Tripod = a fixed camera
    standing where the quad is at `ref` seconds plus an offset, panning to keep it in frame."""
    f = run.forest
    if run.mission == "goal":
        n = f["notes"]
        tw = float(run.tr["t"][np.argmax(run.tr["x"] > n["wall_x"])])
        ta = float(run.tr["t"][np.argmax(run.tr["x"] > n["arch_x"])])
        return [
            (0.0, "orbit", dict(r=3.0, up=1.3, w=0.55, a0=2.6)),
            (3.6, "chase", {}),
            (8.0, "side", {}),
            (
                tw - 3.0,
                "tripod",
                dict(ref=tw, off=(-3.0, 6.5, 1.8)),
            ),  # the wall with the doorway: a pan from the side
            (ta - 3.0, "tripod", dict(ref=ta, off=(-4.0, -6.5, 1.8))),  # the arch
            (
                min(ta + 2.2, run.t_end - 4.0),
                "orbit",
                dict(r=6.0, up=2.8, w=-0.45, a0=0.6),
            ),
        ]
    return [
        (0.0, "orbit", dict(r=3.0, up=1.3, w=0.55, a0=2.6)),
        (3.6, "chase", {}),
        (8.5, "pair", dict(side=1.0)),
        (14.0, "tripod", dict(ref=16.5, off=(-3.0, 6.5, 1.8))),
        (19.0, "side", dict(s=-1.0)),
        (22.5, "pair", dict(side=-1.0)),
    ]  # fmt: skip


class Director:
    """Sequential (frame k after frame k-1): eased camera, heading and position filters, a 1.6 s smoothstep blend of the whole pose when the shot changes."""

    def __init__(self, run):
        self.run, self.shots = run, shots_for(run)
        self.ps = self.h = None
        self.cur, self.prev, self.t_sw = -1, None, 0.0
        self.shot_t0 = 0.0

    def _pose(self, kind, kw, t, p, h, side, subj):
        ps = self.ps
        if kind == "orbit":
            a = kw["a0"] + kw["w"] * (t - self.shot_t0)
            return ps + [kw["r"] * math.cos(a), kw["r"] * math.sin(a), kw["up"]], p + [
                0,
                0,
                0.1,
            ]
        if kind == "chase":
            return ps - 3.4 * h + [0, 0, 1.2] + 0.7 * side, p + 2.5 * h
        if (
            kind == "side"
        ):  # abeam at flying height, between the trunks: the canopy spheres above 4 m hide the quad from any high camera
            return ps + 4.2 * side * kw.get("s", 1.0) - 0.5 * h + [
                0,
                0,
                0.3,
            ], p + 1.0 * h
        if kind == "tripod":
            return self.run.pos(kw["ref"]) + np.array(kw["off"]), p
        if kind == "pair":
            s = subj if subj is not None else p + 4.5 * h
            m = (p + s) / 2
            d = np.linalg.norm(s[:2] - p[:2])
            e = (s - p)[:2] / max(d, 1e-6)
            perp = np.array([-e[1], e[0], 0.0]) * kw["side"]
            return m + perp * (3.2 + 0.7 * d) + [0, 0, 2.6], m + [0, 0, 1.0]
        raise ValueError(kind)

    def step(self, t, dt=1 / FPS):
        run = self.run
        p, R, v = (
            run.pos(t),
            run.R(t),
            np.array([run.at(k, t) - run.at(k, t - 0.2) for k in "xyz"]) / 0.2,
        )
        fwd = R @ [1.0, 0, 0]
        hn = (
            v[:2] / np.linalg.norm(v[:2])
            if np.linalg.norm(v[:2]) > 0.6
            else fwd[:2] / max(np.linalg.norm(fwd[:2]), 1e-6)
        )
        hn = np.array([hn[0], hn[1], 0.0])
        if self.ps is None:
            self.ps, self.h = p.copy(), hn
        k = 1 - math.exp(-dt / 0.45)
        self.ps = self.ps + (p - self.ps) * k
        self.h = self.h + (hn - self.h) * (1 - math.exp(-dt / 0.8))
        self.h = self.h / max(np.linalg.norm(self.h), 1e-6)
        side = np.array([-self.h[1], self.h[0], 0.0])
        i = max(j for j, s in enumerate(self.shots) if s[0] <= t + 1e-9)
        if i != self.cur:
            self.prev, self.t_sw, self.cur, self.shot_t0 = (
                (self.last if hasattr(self, "last") else None),
                t,
                i,
                t,
            )
        _, kind, kw = self.shots[i]
        pos, look = self._pose(kind, kw, t, p, self.h, side, run.subject_truth(t))
        pos, look = np.asarray(pos, float), np.asarray(look, float)
        if self.prev is not None:
            a = smooth((t - self.t_sw) / 1.6)
            pos, look = (
                self.prev[0] * (1 - a) + pos * a,
                self.prev[1] * (1 - a) + look * a,
            )
        pos[2] = max(pos[2], 0.8)
        self.last = (pos, look)
        return dict(pos=pos, lookat=look, fovy=36.0 if kind == "tripod" else 50.0), kind


def fpv_cam(run, t):
    p, R = run.pos(t), run.R(t)
    m = math.radians(MOUNT_DEG[run.mission])
    axis = R @ [math.cos(m), 0.0, math.sin(m)]
    return dict(
        pos=p + 0.1 * (R @ [1.0, 0, 0]) + [0, 0, 0.05], lookat=p + 5 * axis, fovy=58.0
    )


# ---- sim: replay + offscreen render ------------------------------------------------------------------------------------------------------------------
def cmd_sim(a):
    import napl
    from napl import _napl  # noqa: F401  (renderer init)
    from screenshots import Replay

    run, out = Run(a.run_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    napl.render_style("outdoor,shadows")
    r = Replay(a.run_dir)
    r.dt = (
        0.001  # the kernel step; the trace's last row is not necessarily the last tick
    )
    d = Director(run)
    n = int(run.t_end * FPS)
    meta = []
    for k in range(n):
        t = k / FPS
        tick = r.to(t)
        cam, kind = d.step(t)
        img, _ = r.sim.render(width=W, height=H, **cam)
        videnc.save_frame(out, f"f{k:05d}", np.asarray(img))
        img2, _ = r.sim.render(width=FPV[0], height=FPV[1], **fpv_cam(run, t))
        videnc.save_frame(out, f"p{k:05d}", np.asarray(img2))
        meta.append(
            dict(
                k=k,
                t=round(t, 3),
                tick=int(tick),
                shot=kind,
                pos=cam["pos"].round(3).tolist(),
                lookat=cam["lookat"].round(3).tolist(),
                fovy=cam["fovy"],
            )
        )
    h, ok = r.finish()
    (out / "sim.json").write_text(
        json.dumps(
            dict(
                run=str(run.dir),
                mission=run.mission,
                fps=FPS,
                replay_hash=h,
                replay_matches_run=ok,
                native_hash=run.native["state_hash"],
                frames=meta,
            )
        )
    )
    print(
        f"{n} frames in {out}; replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} native.json"
    )
    sys.exit(0 if ok else 1)


# ---- encode: HUD, cards, captions -------------------------------------------------------------------------------------------------------------------
def project(cam, pt, size=(W, H)):
    """Pixel of a world point in a `Sim.render` camera (vertical fov), None behind the camera."""
    d = cam["lookat"] - cam["pos"]
    d = d / np.linalg.norm(d)
    right = np.cross(d, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, d)
    c = np.asarray(pt, float) - cam["pos"]
    z = c @ d
    if z < 0.3:
        return None
    f = (size[1] / 2) / math.tan(math.radians(cam["fovy"]) / 2)
    return size[0] / 2 + f * (c @ right) / z, size[1] / 2 - f * (c @ up) / z


PAL = np.array(
    [[255, 50, 40], [255, 175, 40], [130, 220, 80], [40, 170, 230], [40, 50, 130]],
    float,
)


def depth_image(d):
    """160 x 90 range image (m) -> 320 x 180 RGB: near red ... far blue, no return (max range) dark."""
    d = np.nan_to_num(
        d.astype(float), nan=10.0, posinf=10.0
    )  # no return = NaN / inf in the device's image
    x = np.clip((d - 0.3) / 9.7, 0, 1) ** 0.6 * (len(PAL) - 1)
    i = np.minimum(x.astype(int), len(PAL) - 2)
    f = (x - i)[..., None]
    rgb = PAL[i] * (1 - f) + PAL[i + 1] * f
    rgb[d >= 9.95] = (22, 24, 30)
    return np.repeat(np.repeat(rgb.astype(np.uint8), 2, 0), 2, 1)


def hud(w, run, m, base, fpv, sim_t):
    """The frame with the HUD: mode chip, kv panel, projected plan / trail / goal / subject, the camera's colour view and its depth image."""
    from PIL import Image, ImageDraw

    cam = {
        k: np.array(m[k], float) if k != "fovy" else m[k]
        for k in ("pos", "lookat", "fovy")
    }
    img = base.convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    F = w.F
    p, R = run.pos(sim_t), run.R(sim_t)
    tele = run.rec(sim_t)
    flying = run.at("flying", sim_t) > 0.5
    # the trail (last 5 s) and the planned primitive (green, 2 s ahead)
    ts = np.arange(max(0.0, sim_t - 5.0), sim_t + 1e-9, 0.2)
    pts = [project(cam, run.pos(t)) for t in ts]
    for i in range(len(pts) - 1):
        if pts[i] and pts[i + 1]:
            dr.line(
                [pts[i], pts[i + 1]],
                fill=(255, 255, 255, int(40 + 150 * i / len(pts))),
                width=2,
            )
    if flying and tele and tele.get("chosen"):
        pl = [project(cam, c) for c in tele["chosen"]]
        pl = [x for x in pl if x]
        if len(pl) > 1:
            dr.line(pl, fill=(90, 255, 130, 230), width=3)
    if run.mission == "goal":
        g = run.forest["goal"]
        gp = [project(cam, (g[0], g[1], z)) for z in (0.0, 4.0)]
        if gp[0] and gp[1]:
            dr.line(gp, fill=(255, 200, 60, 230), width=3)
            dr.text(
                (gp[1][0] + 8, gp[1][1] - 8),
                f"GOAL  {run.at('dist_goal', sim_t):.0f} m",
                font=F["m"],
                fill=(255, 210, 80, 255),
            )
    s = run.subject_truth(sim_t)
    if s is not None:
        top, foot = project(cam, (s[0], s[1], 2.0)), project(cam, (s[0], s[1], 0.0))
        if top and foot:
            seen = run.at("subject_seen", sim_t) > 0.5
            col = (90, 255, 130, 255) if seen else (255, 90, 80, 255)
            hh = max(foot[1] - top[1], 6)
            dr.rectangle(
                [top[0] - hh * 0.28, top[1], top[0] + hh * 0.28, foot[1]],
                outline=col,
                width=2,
            )
            dr.text(
                (top[0] - hh * 0.28, top[1] - 22),
                "SUBJECT " + ("TRACKED" if seen else "LOST"),
                font=F["m"],
                fill=col,
            )
    # mode chip + kv panel
    if not flying:
        mode, bad = "TAKEOFF + MAPPING HOVER", False
    elif tele and tele["phase"] == "arrived":
        mode, bad = "ARRIVED: HOLD", False
    else:
        mode = MODE_TEXT[tele["mode"]] if tele else "-"
        bad = bool(tele) and tele["mode"] in ("brake", "escape")
    dr.rounded_rectangle(
        [14, 14, 14 + 14 + dr.textlength(mode, font=F["b"]) + 14, 54],
        radius=8,
        fill=(0, 0, 0, 175),
    )
    dr.text(
        (28, 21),
        mode,
        font=F["b"],
        fill=(255, 120, 110, 255) if bad else (255, 255, 255, 255),
    )
    near = run.obstacle_range(sim_t, R, p)
    rows = [("SPEED", f"{run.at('speed', sim_t):4.1f} m/s"), ("ALTITUDE", f"{p[2]:4.1f} m"),
            ("NEAREST DEPTH RETURN", "-" if near is None else f"{near:4.1f} m"), ("CLEARANCE (truth)", f"{run.at('clearance_m', sim_t):4.1f} m")]  # fmt: skip
    if run.mission == "follow":
        fd = run.at("follow_dist", sim_t)
        rows.append(("FOLLOW DISTANCE", f"{fd:4.1f} m  (3-8 band)" if fd > 0 else "-"))
    if tele:
        rows.append(
            ("SAFE PRIMITIVES", f"{tele['n_safe']:3d} / {tele['n_candidates']}")
        )
        rows.append(("MAP VOXELS", f"{tele['map']['voxels']}"))
        rows.append(("ESTIMATE ERROR", f"{tele['est_err']['pos_norm']:.2f} m"))
    x1, y0 = W - 14, 14
    wid = 330
    dr.rounded_rectangle(
        [x1 - wid, y0, x1, y0 + 14 + 24 * len(rows)], radius=8, fill=(0, 0, 0, 175)
    )
    for i, (k, v) in enumerate(rows):
        dr.text(
            (x1 - wid + 12, y0 + 10 + 24 * i),
            k,
            font=F["sm"],
            fill=(190, 196, 208, 255),
        )
        dr.text(
            (x1 - 12 - dr.textlength(v, font=F["m"]), y0 + 8 + 24 * i),
            v,
            font=F["m"],
            fill=(255, 255, 255, 255),
        )
    img = Image.alpha_composite(img, ov)
    # insets: the camera's colour view (render) and the depth image the planner sees
    j = min(int(np.searchsorted(run.dt_t, sim_t)), len(run.dt_t) - 1)
    dep = Image.fromarray(depth_image(run.depth[j])).convert("RGBA")
    fp = fpv.convert("RGBA").resize(FPV)
    y = H - 14 - FPV[1]
    for x, im, lab in (
        (14, fp, "D435i VIEW (render)"),
        (
            14 + FPV[0] + 10,
            dep,
            "DEPTH 160x90: what the planner sees (red near, blue 10 m)",
        ),
    ):
        img.paste(im, (x, y))
        d2 = ImageDraw.Draw(img, "RGBA")
        d2.rectangle(
            [x - 1, y - 1, x + FPV[0], y + FPV[1]], outline=(255, 255, 255, 120)
        )
        d2.rectangle(
            [x, y - 22, x + d2.textlength(lab, font=F["sm"]) + 10, y - 2],
            fill=(0, 0, 0, 190),
        )
        d2.text((x + 5, y - 20), lab, font=F["sm"], fill=(255, 255, 255, 255))
    return img.convert("RGB"), mode, bad, near


SCN = [  # (dir, title, blurb)
    ("goal", "Mission A: fly 58 m through a forest, a wall doorway and an arch to the goal",
     "Seed 1. The quad takes off, hovers 1 s to map, then flies to the goal from its depth camera alone: depth image, voxel map, 400+ candidate primitives per replan at 25 Hz, only one whose braking stop is in seen-free space is flown. The state it flies on is an estimate (ESKF on IMU + VIO model + baro)."),
    ("follow", "Mission B: follow a walking person through the trees",
     "Seed 1. The person (scripted, ground truth only for scoring) is found by a classical detector on the depth image; the quad keeps about 4.5 m behind, avoids the trees, and turns its camera toward a search belief when the person is hidden. Judged on the strict 3-8 m distance band."),
]  # fmt: skip

CHAPTERS = {
    "build": (
        "1  Build",
        [
            "The Holybro X500 V2 airframe, propulsion, power and avionics, then the vision payload (RealSense D435i + Raspberry Pi 5) appearing in build order in the napl viewer's CAD-style build workspace on assemblies/skydio.toml, the camera orbiting."
        ],
    ),
    "wiring": (
        "2  Wiring",
        [
            "Pin-level wiring sheets of the real assembly: battery, power module, PDB, four ESCs and motors, the Pixhawk, and the companion computer (regulator, Pi 5, D435i USB, the crossed UART to TELEM2). Then the same wires as tubes in 3D."
        ],
    ),
    "program": (
        "3  Program",
        [
            "The autonomy in code: depth-to-map, the primitive planner, the flight loop, and the scenario file the run is judged on."
        ],
    ),
}


def _sim_frames(w, root, d, title, blurb, hold_s=1.5):
    run = Run(d)
    sim = json.loads((Path(d) / "sim" / "sim.json").read_text())
    frames = sim["frames"]
    w.card(title, [blurb], 4.5, title)
    last = None
    for m in frames:
        t = m["t"]
        base = videnc.load_frame(Path(d) / "sim", f"f{m['k']:05d}")
        fpv = videnc.load_frame(Path(d) / "sim", f"p{m['k']:05d}")
        pic, mode, bad, near = hud(w, run, m, base, fpv, t)
        last = pic
        tele = run.rec(t)
        right = [f"sim t = {t:5.1f} s", f"camera: {m['shot']}"]
        w.emit(
            w.bar(
                pic,
                "simulate",
                title,
                blurb,
                right,
                None,
                f"MODE {mode}" if bad else None,
            )
        )
    for _ in range(int(hold_s * FPS)):
        w.emit(w.bar(last, "simulate", title, blurb, ["final frame"], None, None))
    return sim


def cmd_encode(a):
    import record_cuas_lite as rc

    root = Path(a.root)
    enc = videnc.Encoder(Path(a.out), W, rc.HO, FPS, gpl=a.gpl)
    w = rc.Writer(enc)
    w.card("Skydio-style autonomous quadcopter: from the parts library to simulated flight through a forest", [
        "A Holybro X500 V2 with an Intel RealSense D435i depth camera and a Raspberry Pi 5, built from the napl parts library, wired pin by pin, and flown in simulation from its depth camera alone: no GPS, no obstacle positions. Everything on screen is the napl code path: the Design backend, the DRC, the MuJoCo-based kernel, the depth ray caster, the ESKF, the planner.",
        "Two missions, flown on the estimated state: goal seeking through trees, a wall with a doorway and an arch, and following a walking person. Details and limits: docs/skydio/video.md and examples/skydio/README.md",
    ], 7.0, "title")  # fmt: skip
    rc._build_frames(w, root, a.speedup, CHAPTERS)
    w.card("The autonomy pipeline", [
        "depth image (160 x 90, 25 Hz)  ->  point cloud  ->  rolling voxel map + distance field",
        "IMU + VIO model + baro  ->  ESKF  ->  pose estimate (the planner never sees ground truth)",
        "400+ velocity-ramp primitives / replan; flown only if the braking stop stays in seen-free space",
        "setpoints (position, velocity, acceleration)  ->  flight controller  ->  the kernel's physics",
        "follow: detector on the depth image + tracker + search belief when the person is hidden",
    ], 7.0, "PIPELINE", mono=5)  # fmt: skip
    w.card(
        "4  Simulate",
        [
            "Each run is the napl program scenario (examples/skydio/*.scenario.toml): the kernel steps the world, the autonomy flies, the runner judges the trace and re-runs the replay log to check the state hash. The picture is a replay of exactly that log, with a directed camera: orbit, chase, high oblique and tripod pans."
        ],
        4.0,
        "SIMULATE",
    )
    sims = []
    for sub, title, blurb in SCN:
        d = root / sub
        if (d / "sim" / "sim.json").exists():
            sims.append((sub, _sim_frames(w, root, d, title, blurb)))
    lines = []
    for sub, s in sims:
        r = Run(root / sub)
        sm = r.summary
        res = json.loads((root / sub / "result.json").read_text())
        verdict = "PASS" if res["pass"] else "FAIL"
        if not res["replay"]["match"]:
            verdict += " (scenario replay DIFFERS)"
        ev = (
            "reached goal in %.1f s" % sm["time_to_goal_s"]
            if sm["reached_goal"]
            else "follow %.2f m median, band %.2f-%.2f m, seen %.0f %%"
            % (
                sm["follow_dist_median"],
                *sm["follow_band_min_max"],
                100 * sm["subject_seen_frac"],
            )
        )
        lines.append(f"{sub:7s} seed {sm['seed']}  {ev}")
        lines.append(
            f"        contacts {sm['contact_steps']}  min clearance {sm['min_clearance_m']:.2f} m  mean speed {sm['mean_speed_mps']:.1f} m/s  est. drift {sm['estimation']['drift_pct_of_path']:.1f} % of path"
        )
        lines.append(
            f"        napl program {verdict}  state hash {s['native_hash']}  replay behind these frames {'MATCHES' if s['replay_matches_run'] else 'DIFFERS'}"
        )
    w.card("Result", lines + [
        "Monte Carlo over forests (examples/skydio/README.md, not rerun for this video): native flyer, 24 forests per set. Goal 24/24 (development seeds), 23/24 held out (the failure: VIO scale drift left a 1.4 m position error, the quad stopped outside the goal radius). Follow 24/24 and 23/24 held out, zero contacts in 72 follow runs, but the strict 3-8 m band is held for the whole walk in only 21-22 of 24 runs (up to 11.6 m).",
        "Through PX4 SITL without GPS: goal 12/12; follow only 7/12 on the strict band on the training seeds (22/24 held out), PX4 is not deterministic.",
    ], 16.0, "RESULT", mono=len(lines))  # fmt: skip
    w.card("What is real and what is a stand-in", [
        "Real: the parts, assembly, wiring and DRC; the physics, rotor and battery models; the depth ray cast against the scene geometry (2 % range noise, 0.06 s latency); the voxel map, planner, estimator and detector code that flies.",
        "Stand-ins: the VIO is a statistical model of a VIO output (no feature tracker, no tracking loss); the person detector is classical, runs on depth only and has no decoys to confuse it; the depth image is sub-sampled to 160 x 90 with no stereo artefacts; the person is a scripted walker; the forest is generated primitives (cylinders, spheres, boxes), not photographs; the colour view is a render, the planner never sees it.",
        "On screen: NEAREST DEPTH RETURN comes from the depth image (ground removed, approximate ray angles); CLEARANCE (truth) and the subject box are scoring information the autonomy does not have.",
    ], 14.0, "STAND-INS")  # fmt: skip
    enc.close()
    print(
        f"wrote {a.out}: {w.n} frames = {w.n / FPS:.1f} s, {Path(a.out).stat().st_size / 1e6:.1f} MB, encoder {enc.backend}"
    )
    print(
        "timeline (output seconds):", ", ".join(f"{t:.0f}s {lab}" for t, lab in w.marks)
    )


def cmd_png(a):
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    sj = Path(a.frames_dir) / "sim.json"
    if (
        sj.exists()
    ):  # a run's sim/ dir: the frame with its HUD, as the encode stage composes it
        import types

        import record_cuas_lite as rc

        w, run = types.SimpleNamespace(F=rc._fonts()), Run(Path(a.frames_dir).parent)
        fr = json.loads(sj.read_text())["frames"]
        for i in a.idx:
            pic, *_ = hud(
                w,
                run,
                fr[i],
                videnc.load_frame(a.frames_dir, f"f{i:05d}"),
                videnc.load_frame(a.frames_dir, f"p{i:05d}"),
                fr[i]["t"],
            )
            pic.save(out / f"{run.mission}_hud_{i:05d}.png")
        return
    for i in a.idx:
        videnc.load_frame(a.frames_dir, f"f{i:05d}").save(
            out / f"{Path(a.frames_dir).name}_{i:05d}.png"
        )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("oplog")
    s.add_argument("out")
    s = sub.add_parser("sim")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s = sub.add_parser("encode")
    s.add_argument("root")
    s.add_argument("out")
    s.add_argument("--speedup", type=float, default=1.0)
    s.add_argument(
        "--gpl",
        action="store_true",
        help="ffmpeg VAAPI / libx264 allowed (GPL); default is GStreamer VA (LGPL)",
    )
    s = sub.add_parser("png")
    s.add_argument("frames_dir")
    s.add_argument("out_dir")
    s.add_argument("idx", nargs="+", type=int)
    a = ap.parse_args()
    {"oplog": cmd_oplog, "sim": cmd_sim, "encode": cmd_encode, "png": cmd_png}[a.cmd](a)


if __name__ == "__main__":
    main()
