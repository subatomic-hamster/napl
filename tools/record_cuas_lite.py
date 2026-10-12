"""Recording harness of the cuas-lite end-to-end video (media is never committed: frames and the webm stay on the host). Exact commands: docs/cuas-lite/video.md.

The pipeline reuses the existing ones instead of copying them:
  build / wiring / program  tools/record_turret.py's xvfb viewer capture (`_build`), on the REAL assembly (assemblies/cuas-lite.toml opened, not built part by part);
                            this tool only generates the op log (`oplog`) from the assembly: parts revealed in build order, pin-level wiring sheets, source pages.
  sim                       tools/record_sim.py frames --lite: the run's replay.naplog played back in the viewer with the cuas-lite HUD (LitePlayer below)
  web                       the REAL operator page (python/napl/lite/web.py + static/ui.html) in headless Chromium (playwright, a throwaway venv), fed from the run's recorded
                            App.snapshot() stream, ARM / HOLD pressed by clicking the real buttons
  encode                    tools/videnc.py (GPU encode; libx264 only with --gpl): title cards, captions, the segments in order

    run    SCENARIO OUT_DIR [--seed 1]    the lockstep scenario (examples/cuas-lite/run.py run_episode: same harness, same replay hash as `napl program`) + OUT_DIR/lite_hud.jsonl (App.snapshot()
                                          every 50 ms) + lite_meta.json; OUT_DIR/model/ is the scene, replay.naplog / native.json as any napl run
    plan   RUN_DIR                        the auto-plan (sim windows, playback speed, camera) from the run's own events, printed as JSON
    sim    RUN_DIR OUT_DIR                the planned chunks through record_sim.py frames --lite (one xvfb viewer per chunk, GPU)
    oplog  OUT.json                       the build / wiring / program op log generated from the assembly
    web    RUN_DIR OUT_DIR                the operator page captured in Chromium
    encode ROOT OUT.webm                  ROOT = the work dir holding build/ web/ sim/<scenario>/ (see docs/cuas-lite/video.md)
"""

import argparse
import bisect
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples" / "cuas-lite"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import videnc

W, H = 1280, 720
SNAP_EVERY = 10  # control periods (5 ms) between logged HUD records: 20 per sim second
ASM = REPO / "assemblies" / "cuas-lite.toml"
SECTOR_R = 15.0  # m: the drawn radius of the geofence sector (the app's r_max default is unlimited; targets sit at 4-12 m)


# ---- run: the lockstep scenario with a HUD log -----------------------------------------------------------------------------------------------
class HudLog:
    """run_episode's `hook`: every SNAP_EVERY control periods one record = App.snapshot() (what the operator page shows) + the persons' boxes + which track is which target
    (nearest projected truth box centre) + the MCU twin's numbers. Observation only: nothing here touches the harness (the replay hash must not move)."""

    def __init__(self, path):
        self.f = open(path, "w")
        self.last = None
        self.n = 0

    def __call__(self, s):
        self.last = s
        if s.world.k % SNAP_EVERY:
            return
        from napl.cuas.camera import Frame as PoseFrame

        app, now = s.app, s.world.t
        snap = app.snapshot()
        tr = s._truth(now)
        pf = PoseFrame(None, None, tr["cam_pos"], tr["cam_fwd"], tr["cam_up"], 0.0)
        who = {}
        for t in app.tracks:
            if t.misses > 3:
                continue
            cx, cy = (t.bbox[0] + t.bbox[2]) / 2, (t.bbox[1] + t.bbox[3]) / 2
            best = None
            for g in tr["targets"]:
                pr = s.detector.cam.project(g["pos"], pf)
                if pr is None:
                    continue
                d = math.hypot(pr[0] - cx, pr[1] - cy)
                if best is None or d < best[0]:
                    best = (d, g["id"])
            if best and best[0] < 60:
                who[t.id] = best[1]
        rec = dict(
            t=round(now, 3), snap=snap, who=who, persons=[[round(float(x), 1) for x in b] for _, b in app.persons],
            popped=sorted(s.popped), shots=len(s.twin.shots), fw=round(float(s.twin.fw), 3), twin=int(s.twin.state),
        )  # fmt: skip
        self.f.write(json.dumps(rec, separators=(",", ":"), default=float) + "\n")
        self.n += 1

    def close(self, h):
        self.f.close()
        return h


def cmd_run(a):
    import run as litarun
    import scene as sc
    from napl.lite import protocol as p

    os.environ["CUAS_SCENARIO"], os.environ["CUAS_SEED"] = a.scenario, str(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sp = sc.make(a.scenario, a.seed)
    model = sc.build(out / "model")
    log = HudLog(out / "lite_hud.jsonl")
    t0 = time.time()
    res = litarun.run_episode(model, out, sp, a.seed, hook=log)
    s = log.last
    log.f.close()
    meta = dict(
        scenario=a.scenario, seed=a.seed, arm_at=res["arm_at"], shot_t=res["shot_t"], popped_t=res["popped_t"], hit_log=[list(x) for x in s.hit_log],
        transitions=[[t, p.STATE_NAMES[st], r] for t, st, r in s.twin.transitions], t_cut=res.get("t_cut"), fault=sp.fault,
        targets=[dict(id=t.id, cls=t.klass, size=t.size_m, kind=getattr(t, "kind", "")) for t in sp.targets], sector=sc.SECTOR.asdict(),
        sim_s=res["sim_s"], ok=res["ok"], checks=res["checks"], state_hash=res["state_hash"], rounds=res["rounds"], hits_on_targets=res["hits_on_targets"],
        violations=res["violations"], wall_s=round(time.time() - t0, 1),
    )  # fmt: skip
    (out / "lite_meta.json").write_text(json.dumps(meta, indent=1, default=float))
    print(
        json.dumps(
            {
                k: meta[k]
                for k in (
                    "scenario",
                    "ok",
                    "state_hash",
                    "rounds",
                    "hits_on_targets",
                    "sim_s",
                    "wall_s",
                )
            }
        )
    )
    sys.exit(0 if res["ok"] else 1)


# ---- the HUD and the replay player (record_sim.py frames --lite) ------------------------------------------------------------------------------
STAGES = ("SAFE", "IDLE", "ARMED", "ENGAGING", "FIRING")
CLS_COL = {
    "balloon": (255, 120, 110),
    "target": (255, 210, 90),
    "drone": (130, 200, 255),
    "person": (255, 90, 80),
}
REASON_TEXT = {
    "person": "PERSON IN VIEW", "estop": "E-STOP PRESSED", "key_off": "ARM KEY OFF", "link": "NO LINK TO THE MCU", "detector_stale": "DETECTOR SILENT",
    "camera_stale": "CAMERA SILENT", "cap": "SHOT CAP REACHED", "timeout": "ARM TIMEOUT", "mcu_not_armed": "MCU NOT ARMED", "no_sector": "NO SECTOR",
}  # fmt: skip
INHIBIT_KEYS = (
    "person",
    "estop",
    "key_off",
    "link",
    "detector_stale",
    "camera_stale",
    "cap",
    "timeout",
)


def status_text(rec):
    s = rec["snap"]
    r = s["reasons"]
    sel = next((x for x in s["tracks"] if x["selected"]), None)
    sel_id = rec["who"].get(sel["id"], sel["id"]) if sel else None
    for k in INHIBIT_KEYS:
        if k in r:
            return f"FIRING INHIBITED: {REASON_TEXT[k]}", True
    # app notes (App.log, last 12): EVENT 9 SECTOR_ABORT and the sector-not-set ARM refusal, shown for 3 s after the note
    for t, txt in reversed(s.get("log", [])):
        if s["t"] - t > 3.0:
            break
        if (
            "aborted a ball" in txt
        ):  # app.py: "MCU aborted a ball: barrel left the sector"
            return "SECTOR ABORT: barrel left the fire sector, ball not fed", True
        if "arm refused" in txt.lower() and "sector" in txt.lower():
            return "ARM refused: sector not set", True
    if not s["armed"]:
        return (
            ("Disarmed: operator must ARM", False)
            if s["mcu_state"] == "IDLE"
            else (f"MCU {s['mcu_state']}", False)
        )
    if sel_id:
        err = s.get("err_deg")
        return (
            f"Engaging {sel_id}"
            + (f"  pointing error {err:.2f} deg" if err is not None else ""),
            False,
        )
    n = sum(t["confirmed"] for t in s["tracks"])
    return (f"Armed, scanning: {len(s['tracks'])} track(s), {n} confirmed", False)


def sector_items(sec, base, H_):
    """The geofence sector on the ground and at the muzzle height: the two edge rays and the arc (green)."""
    az0, az1 = math.radians(sec["az_min"]), math.radians(sec["az_max"])
    col = (110, 235, 140, 150)
    out = []
    for z in (0.02,):  # ground fan; the elevation limits are in the web UI sector box
        arc = [
            [
                base[0] + SECTOR_R * math.cos(az0 + (az1 - az0) * k / 24),
                base[1] + SECTOR_R * math.sin(az0 + (az1 - az0) * k / 24),
                z,
            ]
            for k in range(25)
        ]
        out.append(
            H_.wline(
                [[base[0], base[1], z], arc[0], *arc, arc[-1], [base[0], base[1], z]],
                col=col,
                lw=1.2,
            )
        )
    return out


def lite_items(v, H_):
    """v: dict(t, rec, meta, vp, muzzle, pellets, targets {id: (pos, cls, popped)}, base, events). -> HUD items (generic napl.hud items)."""
    W_, Hh = v["vp"]
    ui = Hh / 720.0
    m = 16 * ui
    rec, t, meta = v["rec"], v["t"], v["meta"]
    out = []
    if rec is None:
        return [
            H_.tabs(
                STAGES,
                -1,
                (m, m),
                size=13 * ui,
                status="Starting up",
                pad=10 * ui,
                gap=22 * ui,
            )
        ]
    s = rec["snap"]
    sel = next((x for x in s["tracks"] if x["selected"]), None)
    sel_tgt = rec["who"].get(sel["id"]) if sel else None
    status, bad = status_text(rec)
    out.append(
        H_.tabs(
            STAGES,
            STAGES.index(s["stage"]) if s["stage"] in STAGES else 0,
            (m, m),
            size=13 * ui,
            status=status,
            pad=10 * ui,
            gap=22 * ui,
        )
    )
    ok = lambda b: "ok" if b else "NO"
    out.append(H_.kv([("BALLS FED", f"{s['shots']} / {s['shot_cap']}"), ("TARGETS HIT", len(rec["popped"])), ("MCU", s["mcu_state"]),
                      ("DEAD-MAN", ok(s["deadman"])), ("ARM KEY", "on" if s["key"] else "OFF"), ("E-STOP", "PRESSED" if s["estop"] else "released")],
                     (W_ - m, m), a="tr", size=13 * ui, pad=10 * ui, gap=26 * ui))  # fmt: skip
    if bad:
        out.append(
            H_.text(
                status,
                (W_ / 2, m + 70 * ui),
                a="t",
                size=17 * ui,
                col=H_.RED,
                bg=(0, 0, 0, 235),
                pad=11 * ui,
            )
        )
    # flashes: a ball fed, a hit, a pop
    for tf in meta["shot_t"]:
        if 0 <= t - tf < 1.0:
            out.append(
                H_.text(
                    f"BALL FED  {meta['shot_t'].index(tf) + 1}",
                    (W_ / 2, m),
                    a="t",
                    size=14 * ui,
                    col=H_.GREEN,
                    bg=(0, 0, 0, 220),
                    pad=9 * ui,
                )
            )
    for tid, tp in meta["popped_t"].items():
        if 0 <= t - tp < 2.5:
            out.append(
                H_.text(
                    f"HIT  {tid}",
                    (W_ - m, m + 160 * ui),
                    a="tr",
                    size=13 * ui,
                    col=H_.AMBER,
                    bg=(0, 0, 0, 220),
                    pad=7 * ui,
                )
            )
    out += sector_items(meta["sector"], v["base"], H_)
    # the 3D view: one label per target (class, range), the selected one ringed, the person marked
    for tid, (pos, cls, popped) in v["targets"].items():
        if popped:
            continue
        d = math.dist(pos, v["base"])
        tk = next((x for x in s["tracks"] if rec["who"].get(x["id"]) == tid), None)
        col = CLS_COL.get(cls, H_.WHITE)
        if cls == "person":
            out.append(
                H_.wtext(
                    pos,
                    f"PERSON  {d:.0f} m",
                    off=(0, -14 * ui),
                    size=12 * ui,
                    col=H_.RED,
                    bg=(0, 0, 0, 230),
                    pad=4 * ui,
                )
            )
            continue
        tag = f"{tid}  {cls}  {d:.0f} m" + (
            "" if tk is None else "  TRACK" if not tk["confirmed"] else "  CONFIRMED"
        )
        out.append(
            H_.wtext(
                pos,
                tag,
                off=(0, -14 * ui),
                size=11 * ui,
                col=H_.WHITE if tid == sel_tgt else col,
                bg=(0, 0, 0, 215),
                pad=3 * ui,
            )
        )
        if tid == sel_tgt:
            out.append(H_.wring(pos, r=16 * ui, col=H_.WHITE, lw=1.6))
    if v["muzzle"] is not None:
        mz = list(v["muzzle"][0])
        if sel_tgt in v["targets"] and s["stage"] in ("ENGAGING", "FIRING"):
            out.append(
                H_.wline(
                    [mz, list(v["targets"][sel_tgt][0])],
                    col=(255, 255, 255, 60),
                    lw=1.0,
                )
            )
        for p in v["pellets"]:
            out.append(H_.wline([mz, list(p)], col=(255, 255, 255, 140), lw=1.0))
            out.append(H_.wdot(p, r=3.0 * ui, col=H_.WHITE))
    # DETECTOR VIEW (picture-in-picture): the stand-in detector's boxes in the camera frame, no pixels (the sim camera is a truth projection)
    fw, fh = s["frame"]
    pw = round(0.24 * W_)
    ph = round(pw * fh / fw)
    x0, y0 = W_ - m - pw, Hh - m - ph
    inner = []
    for tk in s["tracks"]:
        b = tk["bbox"]
        c = (255, 255, 255) if tk["selected"] else CLS_COL.get(tk["cls"], H_.CYAN)
        inner.append(
            H_.rect(
                (b[0] - 6, b[1] - 6),
                (b[2] - b[0] + 12, b[3] - b[1] + 12),
                line=c,
                lw=1.6 if tk["selected"] else 1.0,
            )
        )
        inner.append(
            H_.text(
                rec["who"].get(tk["id"], tk["id"]),
                (b[0] - 6, b[1] - 8),
                a="bl",
                size=12,
                col=c,
            )
        )
    for b in rec["persons"]:
        inner.append(
            H_.rect(
                (b[0], b[1]),
                (b[2] - b[0], b[3] - b[1]),
                fill=(255, 90, 80, 50),
                line=H_.RED,
                lw=2.0,
            )
        )
        inner.append(H_.text("PERSON", (b[0], b[1] - 4), a="bl", size=16, col=H_.RED))
    cx, cy, k = fw / 2, fh / 2, 18
    inner += [H_.line([(cx - k, cy), (cx - 5, cy)], H_.WHITE, 1.0), H_.line([(cx + 5, cy), (cx + k, cy)], H_.WHITE, 1.0),
              H_.line([(cx, cy - k), (cx, cy - 5)], H_.WHITE, 1.0), H_.line([(cx, cy + 5), (cx, cy + k)], H_.WHITE, 1.0)]  # fmt: skip
    out.append(
        H_.group(
            inner,
            (x0, y0),
            (pw, ph),
            space=(fw, fh),
            fill=(8, 10, 14, 235),
            line=(255, 255, 255, 80),
            lw=1.0,
        )
    )
    out.append(
        H_.text(
            "DETECTOR VIEW (stand-in, ground truth)",
            (x0, y0 - 5 * ui),
            a="bl",
            size=11 * ui,
            col=H_.WHITE,
            bg=H_.BLACK,
            pad=4 * ui,
        )
    )
    return out


class LitePlayer:
    """Same interface as record_sim.Player (to / document / finish / sim / total_s / native / run) for a cuas-lite run: replay.naplog applied tick-exact to a fresh World on a
    Host, the HUD built from the run's lite_hud.jsonl + the replayed bodies (targets, muzzle, balls in flight)."""

    def __init__(self, run_dir, camera="orbit"):
        from napl import Sim
        from napl import hud as H_
        from napl.cuas import hudview
        from record_sim import parse_camera
        from screenshots import setattrs

        self.run, self.H, self.hv = Path(run_dir).resolve(), H_, hudview
        self.man = json.loads((self.run / "model" / "scene.json").read_text())
        self.native = json.loads((self.run / "native.json").read_text())
        self.meta = json.loads((self.run / "lite_meta.json").read_text())
        self.recs = setattrs((self.run / "replay.naplog").read_bytes())
        self.hud = [
            json.loads(x)
            for x in (self.run / "lite_hud.jsonl").read_text().splitlines()
            if x.strip()
        ]
        self.hud_t = [r["t"] for r in self.hud]
        self.sim = Sim.load(self.run / "model" / "scene.xml", host=True)
        self.dt, self.i = self.native["dt"], 0
        self.total_s = self.native["tick"] * self.dt
        tur = self.man["vehicles"]["turret"]
        self.gun, self.base = "/devices/turret_gun", list(tur["pos"])
        self.paths = {
            t["id"]: self.man["body_paths"][t["id"]] for t in self.meta["targets"]
        }
        self.schedule, self.cam = parse_camera(camera), None

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

    def preset(self, t):
        k = bisect.bisect_right([s[0] for s in self.schedule], t) - 1
        return self.schedule[max(k, 0)][1]

    def document(self, seq, size=(W, H), dt=1 / 20):
        t, q = self.t, self.sim.query
        j = bisect.bisect_right(self.hud_t, t) - 1
        rec = self.hud[j] if j >= 0 else None
        mz, md = q(self.gun, "muzzle_pos")[0], q(self.gun, "muzzle_dir")[0]
        pellets = []
        if int(q(self.gun, "rounds")[0][0]) > 0:
            act, pos = q(self.gun, "active")[0], q(self.gun, "pos")[0].reshape(-1, 3)
            pellets = [pos[i].tolist() for i in range(len(act)) if act[i] > 0.5]
        popped = {k for k, tp in self.meta["popped_t"].items() if tp <= t}
        tg = {
            tid: (q(p, "pos")[0].tolist(), c["cls"], tid in popped)
            for (tid, p), c in zip(self.paths.items(), self.meta["targets"])
        }
        v = dict(
            t=t,
            rec=rec,
            meta=self.meta,
            vp=size,
            muzzle=(mz.tolist(), md.tolist()),
            pellets=pellets,
            targets=tg,
            base=self.base,
        )
        camera = None
        preset = self.preset(t)
        if preset != "free":
            head = [self.base[0], self.base[1], self.base[2] + 0.2]
            live = [p for p, c, po in tg.values() if not po and c != "person"] or [head]
            sel = None
            if rec:
                s = next((x for x in rec["snap"]["tracks"] if x["selected"]), None)
                sel = (
                    tg[rec["who"][s["id"]]][0] if s and s["id"] in rec["who"] else None
                )
            goal = None
            if (
                preset == "orbit"
            ):  # slow circle of the pan-tilt stack (hudview's orbit is sized for the turret demo)
                goal = dict(
                    target=[head[0], head[1], head[2] - 0.05],
                    yaw=0.5 + 0.35 * t,
                    pitch=0.22,
                    dist=1.15,
                )
            elif (
                preset == "field"
            ):  # the turret and every target in frame, circling slowly
                pts = [head] + live
                c = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
                r = max(math.dist(c, p) for p in pts) + 1.5
                goal = dict(
                    target=c,
                    yaw=math.atan2(head[1] - c[1], head[0] - c[0]) + 0.5 + 0.12 * t,
                    pitch=0.25,
                    dist=r / math.tan(math.radians(22.5)) * 0.8,
                )
            if goal:  # eased like hudview.camera_for (tau 0.7 s)
                k = 1 - math.exp(-dt / 0.7) if self.cam else 1.0
                p0 = self.cam or goal
                self.cam = dict(
                    target=[
                        p + (g - p) * k for p, g in zip(p0["target"], goal["target"])
                    ],
                    yaw=p0["yaw"] + self.hv._wrap(goal["yaw"] - p0["yaw"]) * k,
                    pitch=p0["pitch"] + (goal["pitch"] - p0["pitch"]) * k,
                    dist=p0["dist"] + (goal["dist"] - p0["dist"]) * k,
                )
            else:
                self.cam = self.hv.camera_for(
                    preset, head, live, md.tolist(), sel, self.cam, dt, t, fovy=45.0
                )
            camera = self.cam
        return self.H.doc(
            lite_items(v, self.H), seq=seq, tick=self.sim.tick, camera=camera
        )


# ---- the auto-plan: which sim stretches, how fast, which camera ------------------------------------------------------------------------------------
def make_plan(run_dir, fps=20, slow=4, fast=4):
    """Windows [t0, t1, fps, camera] covering the run from the first moment of interest to its end, from the run's own events: dead time before ARM at `fast`x, real time
    around the ball fed / hit / person / fault, slow motion (1/`slow`) around a hit, long quiet stretches at 2x. fps in the frame count per sim second: fps/20 = playback speed."""
    run = Path(run_dir)
    meta = json.loads((run / "lite_meta.json").read_text())
    hud = [
        json.loads(x)
        for x in (run / "lite_hud.jsonl").read_text().splitlines()
        if x.strip()
    ]
    arm = meta["arm_at"]
    hits = sorted({round(h[0], 2) for h in meta["hit_log"]})
    cuts = [meta["t_cut"]] if meta.get("t_cut") else []
    pers = [r["t"] for r in hud if r["persons"]]
    marks = [
        (e - 0.4, e + 0.8, 2, "target") for e in meta["shot_t"]
    ]  # priority 2: real time
    marks += [
        (e - 0.6, e + 1.0, 3, "close") for e in hits + cuts
    ]  # priority 3: slow motion
    if pers:  # the person's whole walk through the image, real time
        marks.append((min(pers) - 1.0, max(pers) + 1.5, 2, "field"))
    last = max([b for a, b, p, c in marks] + [arm]) + 5.0
    end = min(meta["sim_s"], last)
    step = 0.05
    cells = []
    for k in range(int(end / step)):
        t = (k + 0.5) * step
        best = max((p for a, b, p, c in marks if a <= t < b), default=0)
        if t < arm - 0.5:
            mode = ("intro", 2 * fps // 3, "orbit" if t < 5.0 else "field")
        elif best == 3:
            mode = ("slow", fps * slow, "close")
        elif best == 2:
            c = next(c for a, b, p, c in marks if a <= t < b and p == 2)
            mode = ("real", fps, c)
        else:
            mode = ("quiet", fps // 2, "field")
        cells.append(mode)
    plan = []
    for k, (name, f, cam) in enumerate(cells):
        if plan and plan[-1][2] == f and plan[-1][3] == cam:
            plan[-1][1] = round((k + 1) * step, 3)
        else:
            plan.append([round(k * step, 3), round((k + 1) * step, 3), f, cam])
    return [w for w in plan if w[1] - w[0] >= 0.3 or w is plan[-1]]


def cmd_verify(a):
    """No window: the replay of the run's log ends on the run's state hash (the frames are of exactly the run that was judged)."""
    h, ok = LitePlayer(a.run_dir).finish()
    print(f"replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} native.json")
    sys.exit(0 if ok else 1)


def cmd_plan(a):
    plan = make_plan(a.run_dir)
    print(
        json.dumps(plan),
        f"-> {sum((b - a_) * f for a_, b, f, c in plan) / 20:.1f} s of video",
    )


def cmd_sim(a):
    """The plan's windows through record_sim.py frames --lite; one xvfb viewer per window (<= 580 s each), frames numbered on across windows."""
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    plan = make_plan(a.run_dir)
    (out / "plan.json").write_text(json.dumps(plan))
    idx = 0
    for t0, t1, fps, cam in plan:
        n = int(round((t1 - t0) * fps))
        for c0 in range(
            0, n, a.chunk
        ):  # long windows split so no single viewer runs over its budget
            c1 = min(n, c0 + a.chunk)
            argv = [sys.executable, str(REPO / "tools" / "record_sim.py"), "frames", str(a.run_dir), str(out), "--lite", "--t0", f"{t0 + c0 / fps:.4f}",
                    "--t1", f"{t0 + c1 / fps:.4f}", "--fps", str(fps), "--camera", f"0:{cam}", "--start-index", str(idx)]  # fmt: skip
            subprocess.run(argv, check=True, timeout=640)
            idx += json.loads((out / f"meta_{idx:05d}.json").read_text())["frames"]
    print(f"{idx} frames in {out}")


# ---- the build / wiring / program op log, generated from the assembly --------------------------------------------------------------------------------
BUILD_STAGES = [  # (title, why, instance ids): the reveal order of the finished design, as a builder would assemble it
    ("Printed frame: stator plate, legs, electronics tray",
     "Printed from cad/cuas-lite (build123d, one source for STL, STEP and the simulation meshes). The stator plate is the root: frame B, pan axis through its centre. Four legs, the electronics tray, the front panel and the pan motor / hall mounts. Every part fits a 220 mm bed.",
     ["stator", "leg1", "leg2", "leg3", "leg4", "tray", "front_panel", "pan_motor_mount", "pan_hall_mount"]),
    ("Pan drive: NEMA 17, 20T GT2 pulley, hall homing switch",
     "17HS4401S stepper on the pan mount with a 20-tooth GT2 pulley; the A3144 hall switch homes the axis. The belt (a closed GT2 loop, not modelled) reduces 5:1 to the 100-tooth pan pulley.",
     ["nema_pan", "pulley_pan", "hall_pan"]),
    ("Lazy-susan bearing and the pan rotor",
     "A 6-inch square lazy susan carries the whole head. On it the printed rotor plate, the 100T pan pulley, the cable collar, the hard-stop tab and the homing magnet.",
     ["lazy_susan", "rotor", "pulley_pan_100t", "collar", "pan_stop_tab", "mag_pan"]),
    ("Yokes, bearings and the tilt drive",
     "Two printed yokes carry the tilt axis on a 608ZZ and a 6807ZZ bearing. The second NEMA 17 with its 20T pulley drives the tilt axis through an 80T pulley (4:1); a second hall switch homes it.",
     ["yoke_l", "yoke_r", "tilt_motor_mount", "brg_l", "brg_r", "nema_tilt", "pulley_tilt", "hall_tilt"]),
    ("Gravity hopper and ball feed",
     "Rival-style foam balls (17.8 mm) are held in the hopper on the pan stage; the feed elbow leads them through the hollow right trunnion into the launcher, so the hopper does not tilt.",
     ["hopper", "hopper_lid", "feed_elbow"]),
    ("Launcher: chassis, barrel, flywheels, pusher",
     "The tilt group: launcher chassis and hub, the 80T tilt pulley, the two-piece barrel, two 180-size flywheel motors with 35 mm flywheels and their covers, the SG92R pusher servo with its arm and piston.",
     ["chassis", "tilt_hub", "pulley_tilt_80t", "barrel_a", "barrel_b", "beam", "piston", "servo_bracket", "servo", "pusher_arm", "wcover_pos", "wcover_neg",
      "rod_l", "mag_tilt", "fw_motor_pos", "fw_motor_neg", "fw_wheel_pos", "fw_wheel_neg", "launcher"]),
    ("Camera",
     "A Logitech Brio 4K on the printed camera mount, co-aligned with the barrel: the target centred in the image is on the barrel axis.",
     ["cam_mount", "brio"]),
    ("Electronics: Jetson, Pico, drivers, relay, MOSFETs, key and E-stop",
     "Jetson Orin Nano and the RP2040 Pico, two TMC2209 drivers, the 5 V buck, the weapon-rail relay, two flywheel MOSFETs with flyback diodes, the XT60 inlet and fuse, the arm key, the E-stop and the pull-up resistors.",
     ["jetson", "pico", "tmc_pan", "tmc_tilt", "buck", "relay", "q1", "q2", "d1", "d2", "fuse", "xt60", "estop", "key", "r_en_pan", "r_en_tilt", "r_ms_pan", "r_ms_tilt",
      "r_su_pan", "r_su_tilt", "r_hp", "r_ht", "r_kt", "r_kb", "r_et", "r_eb", "r_gate"]),
]  # fmt: skip
SHEETS = [  # (title, why, instances)
    ("Power: XT60 inlet, fuse, Jetson and the two motor drivers",
     "Sheet 1. 12 V from the bench supply or a LiPo through the XT60 and a 10 A blade fuse to the Jetson barrel jack and the VM pins of both TMC2209 drivers; returns to the inlet minus. Every connector shows its real pins.",
     ["psu", "xt60", "fuse", "jetson", "tmc_pan", "tmc_tilt"]),
    ("Weapon-rail interlock: key, E-stop, relay coil, relay contact, 5 V buck",
     "Sheet 2. Key (COM-NO) then the NC E-stop in series drive the relay coil (30 mA). The relay contact feeds the 5 V buck that powers both flywheel motors and the pusher servo: key off, E-stop pressed or a broken wire cuts the weapon rail in hardware, whatever the Pico does.",
     ["fuse", "key", "estop", "relay", "buck", "xt60", "servo", "fw_motor_pos", "fw_motor_neg", "q1", "q2", "d1", "d2"]),
    ("Pico to the TMC2209 drivers and the NEMA 17 motors",
     "Sheet 3. STEP / DIR / EN and the MS1 / MS2 straps from the Pico, the single-wire UART through 1 k resistors, the 3.3 V logic rail, and the four motor leads of each stepper (A1 A2 B1 B2).",
     ["pico", "tmc_pan", "tmc_tilt", "nema_pan", "nema_tilt", "r_en_pan", "r_en_tilt", "r_ms_pan", "r_ms_tilt", "r_su_pan", "r_su_tilt", "r_hp"]),
    ("Pico sensing: halls, key and E-stop dividers, MOSFET gates, USB to the Jetson",
     "Sheet 4. Hall outputs with pull-ups on GP8 / GP9, the 33k / 10k dividers that let the Pico read the key and E-stop nodes, the flywheel gate resistor to both MOSFETs, the pusher servo signal, and the USB links Pico-Jetson and Brio-Jetson.",
     ["pico", "jetson", "brio", "hall_pan", "hall_tilt", "r_hp", "r_ht", "r_kt", "r_kb", "r_et", "r_eb", "r_gate", "q1", "q2", "key", "estop", "servo"]),
]  # fmt: skip
SRC_PAGES = [  # (title, why, file, [(scroll fraction or "regex", seconds)])
    ("Firmware: the RP2040 state machine", "firmware/cuas-lite-rp2040/src/fsm.c (C on pico-sdk, BSD-3). Six states: SAFE, IDLE, ARMED, ENGAGING, FIRING, FAULT. to_state() is the one place a transition happens; each leaves SAFE only through valid heartbeats.", "firmware/cuas-lite-rp2040/src/fsm.c",
     [r"static void to_state", r"static void cmd_arm", r"static void watchdog", r"static void pusher"]),
    ("Jetson app: one tick of the control loop", "python/napl/lite/app.py. App.tick(now) polls the camera, the detector and the gimbal, decides, sends. The same code runs in the simulation and on the Jetson; only the camera, detector and serial implementations differ.", "python/napl/lite/app.py",
     [r"def _tick", r"def _decide", r"def snapshot"]),
    ("Fire control: the gates", "python/napl/lite/fire.py. hard_veto() is the list of reasons that forbid any shot (not armed, E-stop, key off, person, stale detector, cap, timeout); engageable() adds the per-track ones (class, confidence, sector, overlap with a person).", "python/napl/lite/fire.py",
     [r"def hard_veto", r"def engageable"]),
    ("Operator web UI", "python/napl/lite/static/ui.html, served from the Jetson: live feed with track boxes, ARM / DISARM, the sector, HOLD TO FIRE (the dead-man, posted every 200 ms), the track table and the status block.", "python/napl/lite/static/ui.html", [0.0]),
    ("A scenario file", "examples/cuas-lite/s6_person_walk.scenario.toml: the scene, the harness, the limits and the monitors the runner judges the trace on: no ball with a person in view or in the 1 s hold, no safety violation.", "examples/cuas-lite/s6_person_walk.scenario.toml", [0.0, 0.5]),
]  # fmt: skip


def _scroll(path, pat):
    """Fraction (0..1) of the file at which the first line matching the regex sits (the PROGRAM page's scroll argument)."""
    import re

    if isinstance(pat, float):
        return pat
    lines = (REPO / path).read_text().splitlines()
    k = next((i for i, ln in enumerate(lines) if re.search(pat, ln)), 0)
    return round(max(0, k - 3) / max(1, len(lines) - 40), 3)


def cmd_oplog(a):
    """Op log for tools/record_turret.py build --open assemblies/cuas-lite.toml: CAD source, parts revealed in build order, the wiring sheets (real pin-level connectors, the
    wires of the assembly, pans over the busiest cards), the wires as tubes in 3D, then the source pages. Needs the Design (run it on the host)."""
    import tomllib

    from napl.design import Design
    from napl.design.viewer import Backend

    doc = tomllib.loads(ASM.read_text())
    ids = [i["id"] for i in doc["instances"]]
    on = [i["id"] for i in doc["instances"] if not i.get("offboard")]
    staged = [x for st in BUILD_STAGES for x in st[2]]
    assert len(staged) == len(set(staged)), "an instance is in two build stages"
    assert set(staged) <= set(ids), f"unknown ids {set(staged) - set(ids)}"
    left = [x for x in on if x not in staged]
    stages = [list(s) for s in BUILD_STAGES]
    stages[-1][2] = (
        stages[-1][2] + left
    )  # anything the stages did not name is shown with the electronics
    R = lambda tool, wait=0.0, **args: {"tool": tool, "args": args, "wait": wait}
    steps = [
        dict(
            seg="CAD",
            title="The parts are code: build123d parameters",
            speed=1.0,
            why="cad/cuas-lite/params.py: every dimension (lazy-susan bolt circle, belt teeth, bearing seats, wall) is a named parameter; parts.py turns them into solids; build.py exports the STL, the STEP and the meshes and masses the simulation uses, from the same source.",
            reqs=[
                R("ui.mode", 1.0, name="program"),
                R("ui.program", 3.5, file="cad/cuas-lite/params.py", scroll=0.15),
                R("ui.program", 4.0, file="cad/cuas-lite/parts.py", scroll=0.3),
            ],
        ),
        dict(
            seg="BUILD",
            title="The finished design, every part hidden",
            speed=1.0,
            why="assemblies/cuas-lite.toml: 79 instances (74 on the turret), 44 mates, 72 wires. The BROWSER's eye hides parts in the 3D view; the parts now come back in the order they are built.",
            reqs=[
                R("ui.mode", 1.0, name="build"),
                R("select", 0.3, instances=[]),
                R("ui.visibility", 1.2, visible=False),
                R("ui.camera", 0.8, preset="iso"),
            ],
        ),
    ]
    shown = []
    for n, (title, why, grp) in enumerate(stages, 1):
        grp = [x for x in grp if x in ids]
        shown += grp
        reqs = [
            R("ui.visibility", 0.6, instances=grp, visible=True),
            R("ui.frame", 1.2, instances=shown),
            R(
                "select", 0.2, instances=[]
            ),  # ui.frame selects what it frames (viewer.cpp "frame"): clear after it
        ]
        reqs += [
            R(
                "camera.orbit",
                4.2,
                yaw_deg=75,
                pitch_deg=8 if n == 1 else 0,
                seconds=4.0,
                relative=True,
            )
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
    steps.append(
        dict(
            seg="BUILD",
            title="Mass and DRC",
            speed=1.0,
            dwell=1.0,
            why="The Design reports mass and the design-rule check: no wire to an unknown pin, no supply fed twice, no connector mismatch.",
            reqs=[
                R("ui.visibility", 0.5, visible=True),
                R("ui.frame", 1.0, instances=[]),
                R("design_check", 2.5),
            ],
        )
    )
    # ---- wiring sheets: the card geometry comes from the real flatten state
    import hashlib

    cpath = Path(
        a.out
    ).with_suffix(
        ".cards.json"
    )  # the card centres cost a full design build (about 6 min): cached per assembly text
    sha = hashlib.sha1(ASM.read_bytes() + repr(SHEETS).encode()).hexdigest()
    cached = json.loads(cpath.read_text()) if cpath.exists() else {}
    cached = cached if cached.get("sha") == sha else {"sha": sha}
    d = Design.open(ASM)
    b = Backend(lambda: d, Path("/tmp/napl-oplog"))
    by_inst = {i["id"] for i in doc["instances"]}
    for n, (title, why, inst) in enumerate(SHEETS, 1):
        inst = [i for i in inst if i in by_inst]
        ports = sorted(
            {
                f"{w[e].partition('.')[0]}.{w[e].partition('.')[2]}"
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
        for c in cached[title]:
            reqs.append(R("ui.wire_view", 4.4, zoom=0.9, center=c, seconds=2.4))
        reqs += [R("ui.wire_view", 2.4, fit=True, seconds=1.6), R("flatten.end", 1.0)]
        steps.append(
            dict(
                seg="WIRING", title=f"{title}", why=why, speed=1.0, dwell=0.6, reqs=reqs
            )
        )
    steps.append(
        dict(
            seg="WIRING",
            title="Back to 3D: the 72 wires are tubes routed around the parts",
            speed=1.0,
            dwell=0.6,
            why="The wires are already in the design. In 3D each conductor is a tube with its gauge's diameter, leaving its connector along the connector axis and routed through free space (a distance field of the assembly, A*), never through a part.",
            reqs=[
                R("ui.mode", 1.2, name="build"),
                R("ui.view", 0.5, emphasise_wires=True),
                R("select", 0.5, instances=[]),
                R("ui.frame", 1.2, instances=[]),
                R(
                    "camera.orbit",
                    8.5,
                    yaw_deg=300,
                    pitch_deg=12,
                    seconds=8.0,
                    relative=True,
                ),
            ],
        )
    )
    for title, why, f, pats in SRC_PAGES:
        steps.append(
            dict(
                seg="PROGRAM",
                title=title,
                why=why,
                speed=1.0,
                dwell=0.4,
                reqs=[R("ui.mode", 1.0, name="program")]
                + [R("ui.program", 4.2, file=f, scroll=_scroll(f, p)) for p in pats],
            )
        )
    Path(a.out).write_text(
        json.dumps(
            {
                "about": "generated by tools/record_cuas_lite.py oplog from assemblies/cuas-lite.toml",
                "steps": steps,
            },
            indent=1,
        )
    )
    print(f"{len(steps)} steps -> {a.out}")  # fmt: skip


# ---- the operator web page, captured in headless Chromium --------------------------------------------------------------------------------------------
CHROME = os.environ.get(
    "NAPL_CHROME",
    "/workspace/wt/napl-video/chrome/chrome-headless-shell-linux64/chrome-headless-shell",
)


class ReplayApp:
    """What web.serve() takes as `app`: snapshot() answers with the run's recorded App.snapshot() at the virtual time `t`; the posts of the clicked buttons are logged."""

    def __init__(self, hud):
        self.hud, self.t, self.posts, self.deadmans = hud, 0.0, [], 0
        self.ts = [r["t"] for r in hud]

    def rec(self):
        return self.hud[max(0, bisect.bisect_right(self.ts, self.t) - 1)]

    def snapshot(self):
        return self.rec()["snap"]

    def post(self, cmd, **kw):
        self.posts.append((self.t, cmd, kw))

    def deadman(self):
        self.deadmans += 1


class SchematicCamera:
    """The page's video feed: the sim has no pixels (the camera is a truth projection), so the feed is drawn from the same detector boxes (sky, ground, a balloon / disc / quad /
    person shape in every box). Labelled on the picture."""

    def __init__(self, app):
        self.app = app

    def jpeg(self):
        import io

        from PIL import Image, ImageDraw

        r = self.app.rec()
        w, h = r["snap"]["frame"]
        img = Image.new("RGB", (w, h))
        dr = ImageDraw.Draw(img)
        for y in range(h):
            k = y / h
            dr.line(
                [(0, y), (w, y)],
                fill=(int(70 + 90 * k), int(110 + 80 * k), int(170 + 50 * k)),
            )
        dr.rectangle([0, int(h * 0.82), w, h], fill=(70, 88, 62))
        for tk in r["snap"]["tracks"]:
            x0, y0, x1, y1 = tk["bbox"]
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if tk["cls"] == "balloon":
                dr.ellipse([x0, y0, x1, y1], fill=(215, 45, 45))
                dr.line(
                    [(cx, y1), (cx, min(h, y1 + 6 * (y1 - y0)))],
                    fill=(230, 230, 230),
                    width=1,
                )
            elif tk["cls"] == "target":
                dr.ellipse([x0, y0, x1, y1], fill=(240, 205, 30), outline=(40, 40, 40))
                dr.ellipse(
                    [
                        cx - (x1 - x0) / 4,
                        cy - (y1 - y0) / 4,
                        cx + (x1 - x0) / 4,
                        cy + (y1 - y0) / 4,
                    ],
                    outline=(40, 40, 40),
                )
            else:
                dr.rectangle(
                    [x0, cy - (y1 - y0) / 8, x1, cy + (y1 - y0) / 8], fill=(35, 35, 40)
                )
                dr.line([(x0, y0), (x1, y1)], fill=(60, 60, 66), width=2)
                dr.line([(x0, y1), (x1, y0)], fill=(60, 60, 66), width=2)
        for x0, y0, x1, y1 in r["persons"]:
            cx = (x0 + x1) / 2
            dr.ellipse(
                [cx - (x1 - x0) / 5, y0, cx + (x1 - x0) / 5, y0 + (y1 - y0) * 0.14],
                fill=(235, 190, 150),
            )
            dr.rounded_rectangle(
                [x0, y0 + (y1 - y0) * 0.14, x1, y1], radius=20, fill=(50, 90, 200)
            )
        dr.text(
            (10, h - 18),
            "SIMULATED CAMERA: schematic drawn from the stand-in detector's boxes (the sim has no pixels)",
            fill=(255, 255, 255),
        )
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=85)
        return buf.getvalue()


def cmd_web(a):
    """Page captured at 1280x720: sector set, ARM clicked, HOLD TO FIRE held (the real dead-man posts reach the server), while the page shows the run's recorded snapshot stream.
    The clicks do not steer the sim run (it is a recording); they are timed to the run's own script (ARM at arm_at, dead-man held from the start)."""
    import importlib.util
    import io
    import socket
    import threading

    from PIL import Image
    from playwright.sync_api import sync_playwright

    run, out = Path(a.run_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    meta = json.loads((run / "lite_meta.json").read_text())
    hud = [
        json.loads(x)
        for x in (run / "lite_hud.jsonl").read_text().splitlines()
        if x.strip()
    ]
    sp = importlib.util.spec_from_file_location(
        "lite_web", REPO / "python" / "napl" / "lite" / "web.py"
    )
    web = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(web)  # stdlib only: no kernel needed in the browser venv
    app = ReplayApp(hud)
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    srv = web.serve(app, "127.0.0.1", port, "video", SchematicCamera(app))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    pers = [r["t"] for r in hud if r["persons"]]
    t0 = a.t0 if a.t0 is not None else meta["arm_at"] - 3.0
    t1 = (
        a.t1
        if a.t1 is not None
        else (max(pers) + 2.0 if pers else meta["arm_at"] + 12.0)
    )
    n = int((t1 - t0) * a.fps)
    marks = {}
    with sync_playwright() as pw:
        br = pw.chromium.launch(executable_path=CHROME, args=["--no-sandbox"])
        pg = br.new_page(viewport=dict(width=W, height=H))
        pg.goto(f"http://127.0.0.1:{port}/?token=video")
        pg.evaluate(
            "document.body.style.zoom = '0.85'"
        )  # the page is laid out for about 1330 CSS px: browser zoom 85 % so it fits 1280 x 720
        app.t = t0
        time.sleep(1.0)
        did = set()
        for k in range(n):
            t = t0 + k / a.fps * a.speed
            app.t = t
            if t >= meta["arm_at"] - 2.4 and "sector" not in did:
                did.add("sector")
                pg.click("#setsec")
                marks["sector"] = k
            if t >= meta["arm_at"] - 0.1 and "arm" not in did:
                did.add("arm")
                pg.click("#arm")
                marks["arm"] = k
            if t >= meta["arm_at"] + 0.6 and "hold" not in did:
                did.add("hold")
                bx = pg.locator("#hold").bounding_box()
                pg.mouse.move(bx["x"] + bx["width"] / 2, bx["y"] + bx["height"] / 2)
                pg.mouse.down()
                marks["hold"] = k
            time.sleep(0.3)  # the page polls SSE at 10 Hz; the MJPEG feed at 15 fps
            pg.evaluate(
                "e => { e.scrollTop = e.scrollHeight }", pg.query_selector("#log")
            )  # newest log line in view
            png = pg.screenshot(type="png")
            img = Image.open(io.BytesIO(png)).convert("RGBA")
            videnc.save_frame(out, f"f{k:05d}", img)
        if "hold" in did:
            pg.mouse.up()
        br.close()
    srv.shutdown()
    (out / "meta_web.json").write_text(
        json.dumps(
            dict(
                run=str(run),
                t0=t0,
                t1=t1,
                fps=a.fps,
                speed=a.speed,
                frames=n,
                marks=marks,
                posts=len(app.posts),
                deadman_posts=app.deadmans,
                post_cmds=sorted({p[1] for p in app.posts}),
            )
        )
    )
    print(
        f"{n} frames in {out}; clicks at frames {marks}; server saw {len(app.posts)} posts, {app.deadmans} dead-man posts"
    )


# ---- encode: cards, captions, the segments in order ---------------------------------------------------------------------------------------------------
FPS = 20
BAR = 108  # caption strip below the picture
HO = H + BAR
REASON_NAMES = (
    "-",
    "boot",
    "heartbeat watchdog",
    "E-stop",
    "key off",
    "operator disarm",
    "arm timeout",
    "shot cap",
    "fault",
    "operator ARM",
    "link up",
    "new session",
)
SIM_SCENES = [  # (scenario, title, what it is)
    ("s2_balloon_drift", "S2  Balloons drifting in the wind", "Two balloons at 6-10 m carried back and forth by a 1 m/s wind. The app leads the targets and corrects for the wind its anemometer reports."),
    ("s3_hanging_targets", "S3  Hanging targets", "Three discs on 1 m strings swinging at 1.5 m/s peak, at 4, 8 and 12 m."),
    ("s4_quad_3ms", "S4  A small quad crossing at 3 m/s", "A 0.3 m quad crosses the field of view at 9 m. The weakest case: foam balls rarely hit a crossing quad (docs/cuas-lite/sim_results.md)."),
    ("s5_mix", "S5  A mix: hung balloon, drifting balloon, 4 m/s quad", "Nearest-first engagement under a shot cap of 5 balls."),
    ("s6_person_walk", "S6  Safety: a person walks through the view", "Five hung balloons, ARM at 8 s. A person (capsule, class person) crosses the whole image. No ball may be fed while a person is in the image or in the 1 s hold afterwards."),
    ("s7_fault_estop", "S7  Fault: the E-stop is pressed mid-engagement", "The E-stop opens while a pusher cycle is in progress. The weapon rail drops in hardware, the MCU goes SAFE with the steppers disabled, and it stays safe until the operator re-arms."),
]  # fmt: skip


def _fonts():
    from PIL import ImageFont

    pick = lambda *c: next((p for p in c if Path(p).exists()), c[-1])
    reg = pick(
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    bold = pick(
        "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    )
    mono = pick(
        "/usr/share/fonts/truetype/freefont/FreeMono.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    )
    sz = 1.0 if "Free" in reg else 0.88  # DejaVu is wider
    f = lambda p, s: ImageFont.truetype(p, int(s * sz))
    return dict(
        s=f(reg, 17),
        b=f(bold, 21),
        t=f(bold, 40),
        h=f(bold, 13),
        m=f(mono, 18),
        sm=f(reg, 14),
    )


class Writer:
    """Cards and the caption strip, shared by every segment of the video."""

    def __init__(self, enc):
        from PIL import Image, ImageDraw

        self.enc, self.n, self.marks, self.F = enc, 0, [], _fonts()
        self.Image, self.Draw = Image, ImageDraw

    def emit(self, img):
        self.enc.write(img)
        self.n += 1

    @staticmethod
    def wrap(text, font, width, dr):
        lines, cur = [], ""
        for w in text.split():
            if dr.textlength((cur + " " + w).strip(), font=font) > width and cur:
                lines.append(cur)
                cur = w
            else:
                cur = (cur + " " + w).strip()
        return lines + ([cur] if cur else [])

    def card(self, title, lines, secs, label=None, mono=0):
        """mono: the first `mono` lines are set in the monospace font (tables)."""
        self.marks.append((self.n / FPS, label or title))
        img = self.Image.new("RGB", (W, HO), (10, 11, 13))
        dr = self.Draw.Draw(img)
        dr.line([(80, 190), (230, 190)], fill=(255, 255, 255), width=2)
        y = 215
        for sub in self.wrap(title.upper(), self.F["t"], W - 160, dr):
            dr.text((80, y), sub, font=self.F["t"], fill=(245, 245, 248))
            y += 52
        y += 14
        for i, ln in enumerate(lines):
            fnt = self.F["m"] if i < mono else self.F["s"]
            for sub in self.wrap(ln, fnt, W - 160, dr):
                dr.text((80, y), sub, font=fnt, fill=(185, 190, 200))
                y += 26 if i >= mono else 22
            y += 10 if i >= mono else 2
        for _ in range(int(secs * FPS)):
            self.emit(img)

    def bar(self, pic, seg, title, why, right=None, n=None, red=None):
        """pic (W x H) with the caption strip below it: segment tag, bold title, the reason, optional red finding, right-aligned mono lines."""
        img = self.Image.new("RGB", (W, HO), (6, 7, 9))
        img.paste(pic, (0, 0))
        dr = self.Draw.Draw(img, "RGBA")
        F, y0 = self.F, H
        dr.line([(0, y0), (W, y0)], fill=(255, 255, 255, 60), width=1)
        dr.text(
            (24, y0 + 9),
            seg.upper() + (f"   {n}" if n else ""),
            font=F["h"],
            fill=(255, 196, 60),
        )
        dr.text((24, y0 + 26), title, font=F["b"], fill=(255, 255, 255))
        for i, sub in enumerate(
            self.wrap(why, F["s"], 900 if right else W - 48, dr)[: 2 if red else 3]
        ):
            dr.text((24, y0 + 54 + 19 * i), sub, font=F["s"], fill=(190, 196, 208))
        if red:
            dr.text((24, y0 + 54 + 38), red[:150], font=F["sm"], fill=(255, 120, 110))
        for i, ln in enumerate(right or []):
            dr.text(
                (W - 24 - dr.textlength(ln, font=F["m"]), y0 + 14 + 21 * i),
                ln,
                font=F["m"],
                fill=(200, 205, 215),
            )
        return img


def _build_frames(w, root, speedup, chapters_over=None):
    """The viewer capture (record_turret.py build --open): per-step playback speed, the router / rebuild dead time played fast, captions from the op log + the Design's replies."""
    bd = root / "build"
    if not (bd / "timeline.json").exists():
        return None
    tl = json.loads((bd / "timeline.json").read_text())
    ts, steps = tl["frames"], tl["steps"]
    segs, prev = [], 0.0
    for s in steps:
        if s["t0"] > prev:
            segs.append((prev, s["t0"], 1.0, None))
        a0 = s["t0"]
        for sa, z in sorted(s.get("slow", [])):
            sa, z = max(sa, a0), min(z, s["t1"])
            if z - sa < 3.0:
                continue
            if sa > a0:
                segs.append((a0, sa, s["speed"], s))
            segs.append((sa, z, max(s["speed"], (z - sa) / 1.5), s))
            a0 = z
        if s["t1"] > a0:
            segs.append((a0, s["t1"], s["speed"], s))
        prev = s["t1"]
    if ts[-1] > prev:
        segs.append((prev, ts[-1], 1.0, None))
    chap = {"CAD": "build", "BUILD": "build", "WIRING": "wiring", "PROGRAM": "program"}
    chapters = {
        "build": (
            "1  Build",
            [
                "The parametric CAD (build123d) and the parts of the assembly appearing in build order, the camera orbiting. The window is the napl viewer's CAD-style build workspace on assemblies/cuas-lite.toml."
            ],
        ),
        "wiring": (
            "2  Wiring",
            [
                "Pin-level wiring sheets of the real assembly: Pico, two TMC2209, Jetson, the key / E-stop / relay weapon-rail interlock, hall switches, MOSFETs. Then the same wires as tubes routed around the parts in 3D."
            ],
        ),
        "program": (
            "3  Program",
            [
                "The Jetson app and its fire-control gates, the operator web UI, the RP2040 firmware state machine."
            ],
        ),
    }
    chapters = (
        chapters_over or chapters
    )  # tools/record_skydio.py passes its own chapter cards
    cache, cur, last = {}, steps[0], None
    for t0_, t1_, sp, s in segs:
        if s is not None:
            cur = s
            c = chap[s["seg"]]
            if c != last:
                w.card(*chapters[c], 3.5, label=chapters[c][0])
                last = c
        sp = max(sp, min(sp * speedup, (t1_ - t0_) / 4.0))
        for k in range(max(1, int(math.ceil((t1_ - t0_) / sp * FPS)))):
            t = t0_ + k * sp / FPS
            j = max(0, min(len(ts) - 1, sum(1 for x in ts if x <= t) - 1))
            if j not in cache:
                cache = {j: videnc.load_frame(bd, f"f{j:05d}")}
            w.emit(
                w.bar(
                    cache[j],
                    cur["seg"],
                    cur["title"],
                    cur["why"],
                    [
                        f"{cur['instances']} parts   {cur['mass']:.3f} kg",
                        f"DRC {cur['errors']} errors, {cur['warnings']} warnings",
                    ],
                    f"step {cur['n']}/{len(steps)}"
                    + (f"   {sp:g}x" if abs(sp - 1) > 1e-6 else ""),
                    (cur.get("findings") or [None])[0],
                )
            )
    return last


def _web_frames(w, root):
    d = root / "web"
    if not (d / "meta_web.json").exists():
        return
    meta = json.loads((d / "meta_web.json").read_text())
    run = (
        Path(meta["run"])
        if Path(meta["run"]).exists()
        else root / "runs" / Path(meta["run"]).name
    )
    hud = [
        json.loads(x)
        for x in (run / "lite_hud.jsonl").read_text().splitlines()
        if x.strip()
    ]
    ts = [r["t"] for r in hud]
    mk = meta["marks"]
    for k in range(meta["frames"]):
        t = meta["t0"] + k / meta["fps"] * meta["speed"]
        rec = hud[max(0, bisect.bisect_right(ts, t) - 1)]
        s = rec["snap"]
        stage = (
            "hold"
            if "hold" in mk and k >= mk["hold"]
            else "arm"
            if "arm" in mk and k >= mk["arm"]
            else "sector"
            if "sector" in mk and k >= mk["sector"]
            else "start"
        )
        why = {
            "start": "The operator page the Jetson serves on the LAN (python/napl/lite/web.py, static/ui.html), opened in headless Chromium: live feed with track boxes, ARM / DISARM, sector, shot count, dead-man, status block, track table. The feed here is a schematic drawn from the stand-in detector's boxes (the sim has no pixels).",
            "sector": "Sector set: azimuth -60 to 60 deg, elevation 0 to 60 deg, radius 15 m. A ball is fed only if both the target and the barrel direction are inside it.",
            "arm": "ARM pressed: the operator arms this session; the MCU goes IDLE then ARMED only with key on, E-stop released, homed, link up.",
            "hold": "HOLD TO FIRE held: the page posts the dead-man every 200 ms; releasing it for 0.5 s stops all fire. The tracks, states and inhibit reasons are the run's recorded App.snapshot() stream (the clicks do not steer the recorded sim).",
        }[stage]
        status, bad = status_text(rec)
        pic = videnc.load_frame(d, f"f{k:05d}")
        w.emit(
            w.bar(
                pic,
                "program",
                "Operator web UI"
                + ("  -  " + status if stage in ("arm", "hold") else ""),
                why,
                [
                    f"sim t = {t:5.1f} s",
                    f"MCU {s['mcu_state']}  balls {s['shots']}/{s['shot_cap']}",
                ],
                None,
                status if bad else None,
            )
        )


def _fsm_frames(w, root):
    """The MCU state machine as a diagram, walked through the transitions the S7 E-stop run really made (lite_meta.json transitions, from the twin that mirrors the firmware)."""
    run = root / "runs" / "s7_fault_estop"
    if not (run / "lite_meta.json").exists():
        return
    tr = json.loads((run / "lite_meta.json").read_text())["transitions"]
    names = ("SAFE", "IDLE", "ARMED", "ENGAGING", "FIRING", "FAULT")
    pos = {
        "SAFE": (200, 170),
        "IDLE": (520, 170),
        "ARMED": (840, 170),
        "ENGAGING": (840, 350),
        "FIRING": (520, 350),
        "FAULT": (200, 350),
    }
    edges = [
        ("SAFE", "IDLE"),
        ("IDLE", "ARMED"),
        ("ARMED", "ENGAGING"),
        ("ENGAGING", "FIRING"),
        ("FIRING", "ENGAGING"),
        ("ARMED", "IDLE"),
        ("ENGAGING", "ARMED"),
    ]
    F = w.F
    shown = []
    prev = "SAFE"
    for t, st, rsn in [[0.0, "SAFE", 1]] + tr:
        if shown and shown[-1][1] == st:
            continue
        shown.append((t, st, rsn, prev))
        prev = st
    for i, (t, st, rsn, src) in enumerate(shown):
        for hold in range(int(1.4 * FPS)):
            img = w.Image.new("RGB", (W, HO), (10, 11, 13))
            dr = w.Draw.Draw(img, "RGBA")
            dr.text(
                (24, 20),
                "PROGRAM   RP2040 firmware: the six-state machine (firmware/cuas-lite-rp2040/src/fsm.c, mirrored by the sim twin)",
                font=F["h"],
                fill=(255, 196, 60),
            )
            for a_, b_ in edges:
                (x0, y0), (x1, y1) = pos[a_], pos[b_]
                off = 7 if (b_, a_) in edges and a_ > b_ else 0
                dr.line(
                    [(x0 + off, y0 + off), (x1 + off, y1 + off)],
                    fill=(120, 125, 135, 255),
                    width=2,
                )
            if i:
                (x0, y0), (x1, y1) = pos[src], pos[st]
                dr.line([(x0, y0), (x1, y1)], fill=(110, 235, 140, 255), width=5)
            for n_ in names:
                x, y = pos[n_]
                on = n_ == st
                dr.rounded_rectangle(
                    [x - 85, y - 32, x + 85, y + 32],
                    radius=12,
                    fill=(40, 120, 70, 255)
                    if on and n_ != "SAFE"
                    else (150, 50, 45, 255)
                    if on
                    else (28, 30, 36, 255),
                    outline=(255, 255, 255, 200 if on else 70),
                    width=3 if on else 1,
                )
                dr.text(
                    (x - dr.textlength(n_, font=F["b"]) / 2, y - 12),
                    n_,
                    font=F["b"],
                    fill=(255, 255, 255),
                )
            dr.text(
                (24, 450),
                f"sim t = {t:6.2f} s    {src} -> {st}    reason: {REASON_NAMES[rsn]}",
                font=F["m"],
                fill=(230, 235, 245),
            )
            for j, (t2, s2, r2, p2) in enumerate(shown[max(0, i - 5) : i + 1]):
                dr.text(
                    (24, 490 + 22 * j),
                    f"{t2:6.2f} s   {p2:9s} -> {s2:9s} {REASON_NAMES[r2]}",
                    font=F["m"],
                    fill=(150, 235, 170) if t2 == t else (150, 155, 165),
                )
            note = (
                "Any state goes to SAFE when the heartbeat is missing > 300 ms, the E-stop opens, the key goes off, or a new session starts. SAFE leaves only by valid heartbeats; ARMED needs a new operator ARM."
                if True
                else ""
            )
            for k2, sub in enumerate(w.wrap(note, F["s"], W - 48, dr)):
                dr.text((24, H + 14 + 19 * k2), sub, font=F["s"], fill=(190, 196, 208))
            w.emit(img)


def cmd_encode(a):
    root = Path(a.root)
    enc = videnc.Encoder(Path(a.out), W, HO, FPS, gpl=a.gpl)
    w = Writer(enc)
    w.card("cuas-lite: a printable counter-drone turret, from CAD to simulated engagement", [
        "A parametric 3D-printable pan-tilt foam-ball turret (Jetson Orin Nano, Logitech Brio, RP2040 + two TMC2209, relay weapon-rail interlock), its control software and its napl simulation. Everything on screen is the napl code path: the Design backend, the DRC, the build, the MuJoCo-based kernel, the real Jetson app and a simulated RP2040 speaking the real serial protocol.",
        "Targets: balloons, hanging targets and small quads in simulation; people are never targeted. Details and limits: docs/cuas-lite/video.md",
    ], 6.0, "title")  # fmt: skip
    _build_frames(w, root, a.speedup)
    _web_frames(w, root)
    _fsm_frames(w, root)
    w.card(
        "4  Simulate",
        [
            "Each scenario is a lockstep run of the real Jetson app against a simulated RP2040 and the napl kernel; the replay.naplog is played back here with the HUD, real time with slow motion around hits. Every run ends on the replay hash printed on the result card."
        ],
        3.5,
        "SIMULATE",
    )
    results = []
    for scn, title, blurb in SIM_SCENES:
        rd, fd = root / "runs" / scn, root / "sim" / scn
        if not (rd / "lite_meta.json").exists() or not fd.exists():
            continue
        meta = json.loads((rd / "lite_meta.json").read_text())
        hud = [
            json.loads(x)
            for x in (rd / "lite_hud.jsonl").read_text().splitlines()
            if x.strip()
        ]
        hts = [r["t"] for r in hud]
        w.card(title, [blurb], 3.0, title)
        for mp in sorted(fd.glob("meta_*.json")):
            m = json.loads(mp.read_text())
            slow = m["fps"] / FPS
            for k in range(m["frames"]):
                t = m["t0"] + k / m["fps"]
                rec = hud[max(0, bisect.bisect_right(hts, t) - 1)]
                status, bad = status_text(rec)
                s = rec["snap"]
                w.emit(
                    w.bar(
                        videnc.load_frame(fd, f"f{m['start_index'] + k:05d}"),
                        "simulate",
                        title,
                        blurb,
                        [
                            f"sim t = {t:5.1f} s"
                            + (f"   slow motion 1/{slow:g}" if slow > 1.01 else "")
                            + (f"   {1 / slow:g}x" if slow < 0.99 else ""),
                            f"balls {s['shots']}/{s['shot_cap']}  hit {len(rec['popped'])}  MCU {s['mcu_state']}",
                        ],
                        None,
                        status if bad else None,
                    )
                )
        results.append((scn, meta))
    lines = [
        f"{scn:20s} {'PASS' if m['ok'] else 'FAIL'}  balls {m['rounds']:2d}  hits {m['hits_on_targets']}  violations {sum(m['violations'].values())}  hash {m['state_hash']}"
        for scn, m in results
    ]
    w.card("Result", lines + [
        "Each hash is the state hash at the end of the run; the replay behind the frames above ends on exactly that hash (checked: tools/record_cuas_lite.py verify). Honest limits: the detector in the sim is a ground-truth projection with the spec's noise model (the real one is YOLO in the AGPL sibling repo); the sim camera has no pixels; hit probabilities against moving targets are low (docs/cuas-lite/sim_results.md).",
    ], 12.0, "RESULT", mono=len(lines))  # fmt: skip
    enc.close()
    print(
        f"wrote {a.out}: {w.n} frames = {w.n / FPS:.1f} s, {Path(a.out).stat().st_size / 1e6:.1f} MB, encoder {enc.backend}"
    )
    print(
        "timeline (output seconds):", ", ".join(f"{t:.0f}s {lab}" for t, lab in w.marks)
    )


def cmd_png(a):
    """A few frames of a frame dir as PNG (the store keeps raw .rgz), to look at: needs Pillow."""
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for i in a.idx:
        videnc.load_frame(a.frames_dir, f"f{i:05d}").save(
            out / f"{Path(a.frames_dir).name}_{i:05d}.png"
        )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("run")
    s.add_argument("scenario")
    s.add_argument("out")
    s.add_argument("--seed", type=int, default=1)
    s = sub.add_parser("plan")
    s.add_argument("run_dir")
    s = sub.add_parser("sim")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument("--chunk", type=int, default=300, help="frames per viewer process")
    s = sub.add_parser("verify")
    s.add_argument("run_dir")
    s = sub.add_parser("oplog")
    s.add_argument("out")
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
    s.add_argument(
        "idx",
        nargs="+",
        type=int,
        help="frame numbers to convert (for looking at them)",
    )
    s = sub.add_parser("web")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument("--fps", type=int, default=10, help="frames per sim second")
    s.add_argument(
        "--speed", type=float, default=1.0, help="sim s per frame step x fps"
    )
    s.add_argument("--t0", type=float)
    s.add_argument("--t1", type=float)
    a = ap.parse_args()
    {
        "encode": cmd_encode,
        "png": cmd_png,
        "web": cmd_web,
        "run": cmd_run,
        "plan": cmd_plan,
        "sim": cmd_sim,
        "verify": cmd_verify,
        "oplog": cmd_oplog,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
