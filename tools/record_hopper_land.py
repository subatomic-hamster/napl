"""Recording harness of the hopper-land end-to-end video (media is never committed: frames and the webm stay on the host). Exact commands: docs/hopper-land/video.md.

Same pipeline as tools/record_skydio.py / record_cuas_lite.py (cards, caption strip, build-frame composer, encoder are imported, not copied), for examples/hopper-land:
  oplog   OUT.json            the op log of a design built from an EMPTY design in the viewer: the build and the wiring steps of the hopper video (docs/showcase/hopper/oplog.json, the
                              references resolved against assemblies/hopper-land.toml: the two assemblies are the same vehicle), the WIRING page with the picker, the pin-level sheet, a
                              pin mistake caught by the DRC, the panel fold / hide-parts demo and zoom pans, then the PROGRAM IDE (find / replace / go to definition / rename / hover /
                              terminal) and the Run slot (BENCH on the lidar, the scenario combo, FULL SIM of mesa/down.scenario.toml). Needs the Design: run it on the host
  build   OPLOG OUT_DIR       record_turret's xvfb viewer capture with the PROGRAM page on examples/hopper-land and a scratch file for the typing / rename demo; the built design must equal
                              assemblies/hopper-land.toml (parts, poses, mass, wires, devices)
  explore OPLOG               no window: the build/wiring steps through the Backend, the built design compared with the assembly
  sim     RUN_DIR OUT_DIR     a `napl program` run of one scenario replayed tick-exact (replay.naplog on a fresh World) with a directed camera that stays close to the 1.3 kg vehicle
                              (chase / side / orbit / tripod; a wide shot only to establish the site) and a time schedule (sped-up calibration and transit, real time on the
                              approach, slow motion at first contact); the HUD is drawn at encode time from the run's trace.csv / telemetry.jsonl
  encode  ROOT OUT.webm       ROOT holds build/ (the `build` output) and one run dir per scenario (mesa_down, ... each with sim/ from `sim`)
  png     SIM_DIR OUT_DIR N...    some frames with their HUD as PNG, to look at
"""

import argparse
import bisect
import csv
import json
import math
import sys
import tomllib
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples" / "hopper"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_hopper as rh
import record_skydio as rs
import videnc

W, H, FPS = rs.W, rs.H, rs.FPS
ASM = REPO / "assemblies" / "hopper-land.toml"
HOP_OPLOG = REPO / "docs" / "showcase" / "hopper" / "oplog.json"
PROG = "examples/hopper-land"
SCRATCH = "_scratch.py"  # created in the program root by `_session` before the capture, removed after
SCRATCH_TEXT = (
    "def sink_cmd(agl_m, v_max=0.6):\n"
    "    # the final sink: slower when closer, never faster than v_max (landflight.py FINAL)\n"
    "    return min(v_max, 0.25 + 0.2 * agl_m)\n"
    "\n"
    "\n"
    "print(sink_cmd(1.5))\n"
)
SCENARIOS = [  # (dir under the run root, Run slot path under examples/hopper-land, title, blurb), in video order
    ("mesa_down", "mesa/down.scenario.toml", "Mesa, down: plateau pad to lowland pad 30 m lower at 3200 m",
     "Thin air (73 % of sea-level density, T/W 2.0 -> 1.64), a 4 m/s wind with the terrain wind field (log boundary layer, speed-up over the brink, lee downdraft) and Dryden turbulence. The planner climbs 12 m above the plateau, crosses the cliff and lands on a pad 30 m BELOW the take-off pad: the lidar is trusted only over a surveyed pad."),
    ("mesa_up", "mesa/up.scenario.toml", "Mesa, up: the same hop in reverse, from the lowland up onto the plateau",
     "Take off from the lowland, climb past the 30 m cliff face and land on the higher plateau pad. The MC row: 93.3 % over 60 held-out seeds, and the wind-x2 variant fails 35 % (not a pass-rate claim for this one flight)."),
    ("ridge_summit", "ridge/summit.scenario.toml", "Ridge summit at 3800 m: ridge wind and a gust front",
     "Hop along a 45 m high ridge crest at 3800 m (T/W 1.58). The wind blows across the ridge (updraft windward, lee turbulence) and a gust front sweeps through. A disturbance observer subtracts the wind's push (k_dob). The README: 75 % over 60 seeds, 45 % at wind x1.5: the 2:1 vehicle runs out of vane authority."),
    ("ledge_lee", "ledge/lee.scenario.toml", "Ledge in the lee: a pad cut into the steep lee face 13 m below the summit",
     "Lee turbulence and a downdraft over the carved ledge pad at 3400 m. The README: 80 % over 60 seeds, wind x1.5 30 %, k_dob off 50 %: the failures are aborts (off-plan, tilt) at peak winds of 9-18 m/s."),
    ("rain25", "rain/rain25.scenario.toml", "Rain 25 mm/h: a hop with lidar dropouts and false near returns",
     "The kernel's rain device: a little fan thrust loss, water film and drag, lidar dropouts and false near returns that grow with the rate, overcast and streaks in the renderer. Rain catch area 0.045 m2 and wet area 0.12 m2 are estimates. MC 100 % over 60 seeds."),
    ("rain100", "rain/rain100.scenario.toml", "Rain 100 mm/h: a violent downpour",
     "Four times the rain rate; about 64 % of the lidar-like returns survive and false near returns rise. MC 100 % over 60 seeds (150 mm/h and wind x2 also 100 %), simulated rain, not a measured one."),
    ("deck_scripted", "deck/scripted.scenario.toml", "Deck, scripted: a moving platform with heave, roll, pitch and drift",
     "Deck-relative landing on a scripted platform (invented motion and 10 Hz link noise): the vehicle follows the deck state (DeckState), averages the lidar height over 0.1 s before cutting the fan, and holds the last 0.45 m until the forecast deck rise is slow. MC 98.3 % over 60 seeds (drift 2 m/s: 70 %)."),
    ("osv_underway", "osv/underway.scenario.toml", "Offshore support vessel under way: landing on a heaving deck in a JONSWAP sea",
     "The 12 m OSV makes 2 m/s in head seas, Hs 0.5 m. Moving-base RTK relative to the vessel's antenna (RelGnss), the motion unit's deck link (DeckLink) and a forecast of the deck height (DeckPredictor); the hopper waits for a quiet window, then sinks on the forecast. Seed 1 is one draw from an 80 % family."),
]  # fmt: skip
BENCH_WAIT = 14.0  # s the bench run is left to finish on screen
FULL_WAIT = 190.0  # s the live mesa/down run is left to fly (wall; 81 sim s take about 110 s with the window rendering, then napl-replay checks the log)
FULL_SPEED = 6.0  # the step's playback speed in the video


def R(tool, wait=0.0, **args):
    return {"tool": tool, "args": args, "wait": wait}


def _line(path, pat):
    return rs._line(path, pat)


# ---- the op log ------------------------------------------------------------------------------------------------------------------------------
def cmd_oplog(a):
    from napl.design import Design
    from napl.design.viewer import Backend

    ref = Design.open(ASM)
    doc = tomllib.loads(ASM.read_text())
    src = json.loads(HOP_OPLOG.read_text())["steps"]
    steps = rh.resolve(
        src, ref
    )  # the references into the reference design replaced by their literal values
    for st in steps:
        if (
            st["seg"] == "LIBRARY"
        ):  # record_cuas_lite._build_frames knows CAD / BUILD / WIRING / PROGRAM
            st["seg"] = "BUILD"
        for rq in st.get("reqs", []):
            for op in rq["args"].get("ops", []):
                if op.get("op") == "assembly":
                    op["description"] = doc["assembly"]["description"]
    for st in steps:  # no camera below the ground: from underneath the plates are seen through the floor as bare outlines
        for rq in st.get("reqs", []):
            if (
                rq["tool"] == "camera.orbit"
                and rq["args"].get("relative") is False
                and rq["args"].get("pitch_deg", 0) < 0
                and st["title"].startswith("Four vanes")
            ):
                rq["args"]["pitch_deg"] = 12
        if st["title"] in ("Four skid legs", "Device bindings: the simulation side"):
            st.setdefault("pre", []).append(R("ui.camera", 1.0, preset="iso"))
            if st["title"].startswith("Device"):
                st["reqs"].insert(0, st["pre"].pop())
    head = steps[:32]  # steps 0-31: the build up to the DRC step
    wiring = steps[
        34:38
    ]  # power chain, the pin mistake, its fix, servo chains and logic feeds
    pick = steps[32]["reqs"][1]["args"]["instances"]
    fb = steps[33]["reqs"][0]["args"]
    # the cards' centres on the sheet (the zoom pans), computed on the finished design
    b = Backend(lambda d=Design.open(ASM): d, Path("/tmp/napl-oplog-hopper-land"))
    r = json.loads(b(json.dumps({"tool": "flatten.begin", "args": fb})))
    if not r["ok"]:
        raise SystemExit(f"flatten.begin: {r.get('error')}")
    cards = json.loads(b(json.dumps({"tool": "flatten.layout", "args": {}})))[
        "flatten"
    ]["cards"]
    wired = lambda c: sum(1 for p in c["pins"] if p.get("wired"))
    key = sorted(sorted(cards, key=wired, reverse=True)[:3], key=lambda c: c["x"])
    centres = [[round(c["x"] + c["w"] / 2), round(c["y"] + c["h"] / 2)] for c in key]
    b(json.dumps({"tool": "flatten.end", "args": {}}))
    out = list(head)
    out.append(dict(
        seg="WIRING", title="Wiring page: tick the components in the picker, flatten to the pin sheet", speed=1.2, dwell=0.6,
        why="The electrical side: battery, ESC, motor, the two regulators, the four servos, the Teensy and the lidar. The WIRING page lists every component with its real connectors in the Wire Components picker; ticking a box adds its terminals, and the sheet shows each connector with its pins as printed on the part (pack leads, the ESC's motor phases and signal, the Pololu pads, the servo JSTs, the Teensy header).",
        reqs=[R("ui.mode", 1.4, name="wiring"), R("ui.wire_pick", 3.4, instances=pick), R("flatten.begin", 3.0, **fb), R("ui.wire_view", 2.0, fit=True)]))  # fmt: skip
    out += wiring
    hid = ["batt", "esc", "reg1", "reg2"]
    out.append(dict(
        seg="WIRING", title="Panels, hidden parts and zoom: the sheet follows the 3D scene", speed=1.8, dwell=0.6,
        why="The right panel folds away for a bigger sheet. The browser's eye hides the battery, the ESC and the regulators: their cards and wires vanish in 2D and the tubes in 3D; showing them brings them back. Then the canvas zooms to 100 % so every pin label can be read.",
        reqs=[R("ui.wire_view", 2.0, panel="fold"), R("ui.wire_view", 2.0, panel="open"), R("ui.visibility", 3.0, instances=hid, visible=False), R("ui.mode", 1.0, name="build"),
              R("ui.view", 0.3, emphasise_wires=True), R("ui.camera", 4.0, preset="iso"), R("ui.visibility", 3.5, visible=True), R("ui.mode", 1.5, name="wiring"), R("ui.wire_view", 1.5, fit=True),
              *[R("ui.wire_view", 4.4, zoom=0.9, center=c, seconds=2.4) for c in centres], R("ui.wire_view", 2.4, fit=True, seconds=1.6)]))  # fmt: skip
    out += steps[38:40]  # back to 3D: the tubes, a closer look from below
    out.append(steps[40])  # device bindings (the simulation side)
    out.append(dict(seg="BUILD", title="Save the design", speed=1.0, dwell=0.4, why="The design is saved as assemblies/hopper-land.toml: the next page edits the code that flies it.",
                    reqs=[R("ui.mode", 1.2, name="build"), R("design_save", 2.0, path="__SAVE__")]))  # fmt: skip
    # ---- program: the IDE, the Run slot
    U = lambda wait, **kw: R("prog.ui", wait, **kw)
    C = lambda wait, name: U(wait, cmd="cmd", name=name)
    lf = (REPO / PROG / "landflight.py").read_text().splitlines()
    cls = next(i for i, ln in enumerate(lf) if ln.startswith("class Flight"))
    est = next(i for i, ln in enumerate(lf) if "LandEstimator(DT)" in ln)
    estc = lf[est].index("LandEstimator")
    kd = next(i for i, ln in enumerate(lf) if 'p["k_dob"]' in ln or "p['k_dob']" in ln)
    kdc = lf[kd].index("k_dob")
    lv = (REPO / PROG / "landveh.py").read_text().splitlines()
    tf = next(i for i, ln in enumerate(lv) if ln.startswith("def t_full"))
    lh = (REPO / PROG / "landhop.py").read_text().splitlines()
    hl = next(i for i, ln in enumerate(lh) if '"/devices/lidar"' in ln)
    hc = lh[hl].index("lidar") + 2
    P = lambda title, why, *reqs, speed=1.0: out.append(
        dict(
            seg="PROGRAM", title=title, why=why, speed=speed, dwell=0.5, reqs=list(reqs)
        )
    )
    P("PROGRAM is an IDE: the landing flight software", "examples/hopper-land/landflight.py: Flight is the 250 Hz state machine (PAD, SPOOL, CLIMB, TRANSIT, HOVER, DESCENT, FINAL, TOUCHDOWN) of the original hopper with the landing changes: pads at different heights, a density-aware thrust feed-forward, gust handling and the deck interface. File explorer, tabs, Python syntax colouring, a problems dock.",
      R("ui.mode", 1.5, name="program"), U(5.0, cmd="open", path="landflight.py", line=cls, col=6, len=6))  # fmt: skip
    P("Density-aware feed-forward", "landveh.py t_full: the fan's full thrust falls with the air density, T_full = 25.3 (rho / 1.225)^0.63 N (a fit to the kernel's EDF data, 0 to 5500 m, extrapolated above). The flight computer reads the density from its barometer; the firmware refuses to launch when the thrust margin is under 1.15.",
      U(1.0, cmd="goto", line=tf + 1), U(4.5, cmd="open", path="landveh.py", line=tf, col=4, len=6), speed=1.2)  # fmt: skip
    P("Go to definition: LandEstimator", "F12 on `LandEstimator` jumps to landest.py: the estimator fuses the lidar only over a surveyed pad (or the deck); elsewhere the lidar height above ground is reported but not used as altitude.",
      U(1.0, cmd="open", path="landflight.py", line=est, col=estc, len=13), U(1.5, cmd="define"), U(3.5, cmd="hover", line=-1))  # fmt: skip
    P("Find, replace and find in files", "Ctrl+F finds the selected symbol (here the gust gain `k_dob`: the wind's push subtracted from the position loop's force), Ctrl+H opens replace, Ctrl+Shift+F searches every file of the program.",
      U(1.0, cmd="open", path="landflight.py", line=kd, col=kdc, len=5), C(3.0, "find"), C(2.5, "replace"), C(3.0, "find_in_files"), U(0.5, cmd="dock", name="search"))  # fmt: skip
    P("Hover a device: the tooltip names the part and its pins", "`/devices/lidar` in landhop.py is the TF-Luna of the design: hovering shows the component, its part and the pins it is wired to.",
      U(1.5, cmd="open", path="landhop.py", line=hl, col=hc), U(4.5, cmd="hover", line=hl, col=hc), U(0.3, cmd="hover", line=-1))  # fmt: skip
    P("Edit, rename a symbol, and a real shell in the TERMINAL tab", "A scratch file in the program folder: type a line, rename `sink_cmd` with F2 (jedi renames every use), then the TERMINAL tab is a pty shell in the program folder.",
      U(1.0, cmd="open", path=SCRATCH, line=2, col=0, len=0), U(3.0, cmd="type", text="    # a demo line: the real sink law is in landflight.py\n"), U(1.0, cmd="open", path=SCRATCH, line=0, col=4, len=8),
      U(3.0, cmd="rename", new="sink_rate"), U(0.5, cmd="dock", name="terminal"), U(1.2, cmd="term", text="ls"), U(2.4, cmd="term", text="python _scratch.py"))  # fmt: skip
    P("Run slot, BENCH: the TF-Luna model on the bench", "BENCH runs the selected component's device model on a bench (a rangefinder: the noise readout at several ranges), then the pytest files tied to the open file (none for README.md); the OUTPUT tab streams the table and the chip shows PASS.",
      U(0.5, cmd="open", path="README.md", line=0, col=0), U(0.5, cmd="dock", name="output"), C(0.8, "bench"), C(1.0, "target:lidar"), C(BENCH_WAIT, "run"))  # fmt: skip
    P("Run slot, the scenarios: every folder of the combo", "The Run slot lists every *.scenario.toml under the program root; the folders group them: deck, ledge, mesa, osv, rain, ridge. Step through all eight.",
      C(0.8, "full"), *[C(1.2, f"target:{s[1]}") for s in sorted(SCENARIOS, key=lambda s: s[1])], speed=1.2)  # fmt: skip
    P("Run slot, FULL SIM: mesa/down flies live in SIMULATE", "FULL SIM runs the scenario in the window's own simulation, in real time: the page switches to SIMULATE with the LIVE label and the chase camera; the monitors judge the run and the replay hash is re-checked.",
      C(0.8, "full"), C(1.0, f"target:{SCENARIOS[0][1]}"), C(0.5, "run"), R("ui.mode", FULL_WAIT, name="simulate"), speed=FULL_SPEED)  # fmt: skip
    P("Run slot result: the chip, and the monitors' verdict in OUTPUT", "When the scenario ends the Run slot shows the verdict chip and OUTPUT lists the monitors, the state hash and whether the replay matched.",
      R("ui.mode", 1.0, name="program"), U(6.0, cmd="dock", name="output"))  # fmt: skip
    for n, st in enumerate(out, 1):
        st["n"] = n
    Path(a.out).write_text(
        json.dumps(
            {
                "about": "generated by tools/record_hopper_land.py oplog from assemblies/hopper-land.toml + docs/showcase/hopper/oplog.json",
                "steps": out,
            },
            indent=1,
        )
    )
    print(f"{len(out)} steps -> {a.out}; zoom centres {centres}")


# ---- the build capture: record_turret's _build with the program root and the scratch file ---------------------------------------------------
def cmd_build(a):
    import subprocess
    import tempfile

    videnc.keep_from(a)
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
            "_build",
            a.oplog,
            str(out),
            fb,
            "--expect",
            a.expect,
            "--timeout",
            str(a.timeout),
        ]
        argv += ["--save", a.save] if a.save else []
        argv += ["--steps", str(a.steps), "--quick-until", str(a.quick_until)]
        subprocess.run(argv, check=True, timeout=a.timeout)


def _session(fn, a):
    """record_turret's `fn` on an empty design, but the PROGRAM page edits examples/hopper-land (the design has no file yet) and the scratch file exists for the session."""
    import napl.design.viewer as V
    import record_turret as rt

    rt.NAME, rt.DESC = (
        "hopper-land",
        tomllib.loads(ASM.read_text())["assembly"]["description"],
    )

    class LandBackend(V.Backend):
        def __init__(self, *args, **kw):
            super().__init__(*args, program_root=REPO / PROG, **kw)

    V.Backend = LandBackend
    sc = REPO / PROG / SCRATCH
    sc.write_text(SCRATCH_TEXT)
    try:
        a.open = None
        fn(a)
    finally:
        sc.unlink(missing_ok=True)


def _build(a):
    import record_turret as rt

    _session(rt._build, a)


def cmd_explore(a):
    """No window: the build / wiring steps (not the PROGRAM page) through the Backend, the built design compared with --expect."""
    import tempfile

    import record_turret as rt

    steps = [
        s
        for s in json.loads(Path(a.oplog).read_text())["steps"]
        if s["seg"] != "PROGRAM"
    ]
    tmp = Path(tempfile.mkdtemp(prefix="hl-explore-")) / "oplog.json"
    tmp.write_text(json.dumps({"steps": steps}))
    a.oplog = str(tmp)
    _session(rt.cmd_explore, a)


# ---- one run: trace, telemetry, scene --------------------------------------------------------------------------------------------------------
PHASE_TEXT = {
    "PAD": "PAD: SENSOR CALIBRATION", "READY": "READY: SPOOL-UP CHECKS", "SPOOL": "SPOOL THE FAN", "CLIMB": "CLIMB", "TRANSIT": "TRANSIT TO THE LANDING PAD",
    "HOVER": "HOVER OVER THE LANDING PAD", "DESCENT": "DESCENT", "FINAL": "FINAL: LIDAR-GOVERNED SINK", "TOUCHDOWN": "TOUCHDOWN: FAN OFF", "LANDED": "LANDED",
    "ABORT_LAND": "ABORT: LAND NOW", "FAULT": "FAULT",
}  # fmt: skip
STAGES = [
    "PAD",
    "SPOOL",
    "CLIMB",
    "TRANSIT",
    "HOVER",
    "DESCENT",
    "FINAL",
    "TOUCHDOWN",
    "LANDED",
]
RATE = {
    "PAD": 4.0,
    "READY": 4.0,
    "TRANSIT": 2.0,
    "HOVER": 2.0,
}  # sim seconds per video second above real time (labelled on screen)
SLOW_TD = 4  # slow motion factor around the first contact


class Landing:
    """The files a `napl program` hopper-land run leaves: trace.csv, telemetry.jsonl, result.json, native.json."""

    def __init__(self, d):
        self.dir = Path(d)
        rows = list(csv.DictReader(open(self.dir / "trace.csv")))
        self.tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        self.tele = [json.loads(ln) for ln in open(self.dir / "telemetry.jsonl")]
        self.tt = [r["t"] for r in self.tele]
        self.result = json.loads((self.dir / "result.json").read_text())
        self.native = json.loads((self.dir / "native.json").read_text())
        self.t_end = self.native["tick"] * self.native["dt"]
        self.name = self.result.get("name", self.dir.name)
        td = self.tr["td_speed_ms"]
        self.program = self.result  # the runner's result.json: monitors, replay check
        # the trace holds 99 in the touchdown columns until the feet are down
        self.t_td = float(self.tr["t"][np.argmax(td < 90)]) if (td < 90).any() else None
        self.t_fan = next(
            (float(r["t"]) for r in self.tele if r["state"] == "SPOOL"), 0.0
        )

    def at(self, k, t):
        return float(np.interp(t, self.tr["t"], self.tr[k]))

    def rec(self, t):
        j = bisect.bisect_right(self.tt, t) - 1
        return self.tele[max(j, 0)]

    def state(self, t):
        return self.rec(t)["state"]

    def gust_at(self, t):
        """The wind at the reference height now vs 3 s ago (telemetry wind_ref_ms): a front shows as a jump of >= 2.5 m/s."""
        w = lambda s: np.linalg.norm(self.rec(s).get("wind_ref_ms", [0, 0, 0])[:2])
        return w(t) - w(max(t - 3.0, 0.0)) >= 2.5


def schedule(run):
    """[(sim time, rate)] one entry per output frame: rate = sim seconds per video second."""
    out, t = [], 0.0
    while t < run.t_end - 1e-9:
        st = run.state(t)
        r = RATE.get(st, 1.0)
        if st in ("SPOOL", "CLIMB") and run.rec(t)["truth"]["alt_m"] > 0:
            r = 1.0
        if run.t_td is not None and run.t_td - 0.6 <= t < run.t_td + 1.2:
            r = 1.0 / SLOW_TD
        out.append((t, r))
        t += r / FPS
    return out


# ---- the director: camera moves near the vehicle ---------------------------------------------------------------------------------------------
class Director:
    """Eased following point; camera positions from candidate rings, accepted when above the ground and with a clear ray to the vehicle (the Sim's own raycast: terrain, ship, rocks)."""

    def __init__(self, sim, body):
        self.sim, self.body = sim, body
        self.cur = self.ps = self.h = self.cpos = self.clook = None
        self.sel, self.bad_since, self.t_sw, self.t0, self.tri = 0, None, 0.0, 0.0, None
        self.p0 = None

    def ground(self, x, y, z):
        d = self.sim.raycast([x, y, z + 400.0], [[0, 0, -1.0]])[0]
        return z + 400.0 - d if d > 0 else -1e9

    def ok(self, c, q, gz):
        if c[2] < self.ground(c[0], c[1], c[2]) + 0.8 or np.linalg.norm(c - q) < 1.5:
            return False
        v = q - c
        d = float(np.linalg.norm(v))
        hit = self.sim.raycast(c, [v])[0]
        return not (0 < hit < d - 0.6)

    def shot(self, run, t, st):
        if t < max(run.t_fan - 0.5, 2.0) and run.at("z_m", t) < 1.0:
            return ("establish", {})
        if run.t_td is not None and run.t_td - 0.4 <= t < run.t_td + 3.5:
            return ("tripod", {})
        if st in ("SPOOL", "CLIMB"):
            return ("orbit", dict(r=2.8, up=0.8, w=0.45))
        if st == "TRANSIT":
            return ("chase", {}) if int(t / 8.0) % 2 == 0 else ("side", {})
        if st == "HOVER":
            return ("orbit", dict(r=3.4, up=1.4, w=0.3))
        if st in ("DESCENT", "FINAL"):
            return ("orbit", dict(r=2.6, up=1.0, w=-0.4))
        return ("orbit", dict(r=3.4, up=1.4, w=0.3))

    def _cands(self, kind, kw, t, p, h, side):
        ps, z = self.ps, np.array([0.0, 0.0, 1.0])
        if kind == "orbit":
            a0 = 2.6 + kw["w"] * (t - self.t0)
            return [
                ps
                + [kw["r"] * math.cos(a0 + da), kw["r"] * math.sin(a0 + da), kw["up"]]
                for da in (0, 0.8, -0.8, 1.6, -1.6, math.pi)
            ], p
        if kind == "chase":
            return [
                ps - 3.6 * h + 0.9 * side + 2.6 * z,
                ps - 3.6 * h - 0.9 * side + 2.6 * z,
                ps - 4.5 * h + 3.2 * z,
                ps - 7.0 * h + 4.5 * z,
            ], p + 1.5 * h
        if kind == "side":
            return [
                ps + lat * sg * side - 0.3 * h + up * z
                for lat, up in ((4.0, 2.4), (3.2, 1.6), (5.5, 3.2), (8.0, 4.0))
                for sg in (1.0, -1.0)
            ], p + 0.5 * h
        if (
            kind == "establish"
        ):  # the site from a distance, dollying in over the calibration
            u = min(1.0, max(0.0, t / 6.0))
            far = 13.0 - 6.0 * u
            return [
                self.p0 + [far * math.cos(a), far * math.sin(a), 2.0 + 2.5 * (1 - u)]
                for a in (2.4, 3.2, 1.6, 4.0, 0.8)
            ], self.p0 + [0, 0, 0.8]
        raise ValueError(kind)

    def step(self, run, t, dt):
        p = np.array(self.sim.query(self.body, "pos")[0], float)
        if self.p0 is None:
            self.p0, self.ps, self.h, self.hold = (
                p.copy(),
                p.copy(),
                np.array([1.0, 0, 0]),
                p.copy(),
            )
        v = (
            (p - self.prev) / max(dt, 1e-3)
            if getattr(self, "prev", None) is not None
            else np.zeros(3)
        )
        self.prev = p.copy()
        hn = np.array([v[0], v[1], 0.0]) if np.linalg.norm(v[:2]) > 0.5 else self.h
        hn = hn / max(np.linalg.norm(hn), 1e-6)
        self.ps = self.ps + (p - self.ps) * (1 - math.exp(-dt / 0.35))
        self.h = self.h + (hn - self.h) * (1 - math.exp(-dt / 0.9))
        self.h = self.h / max(np.linalg.norm(self.h), 1e-6)
        side = np.array([-self.h[1], self.h[0], 0.0])
        st = run.state(t)
        kind, kw = self.shot(run, t, st)
        key = (
            kind,
            tuple(sorted(kw.items())),
            int(t / 10.0) if kind in ("chase", "side") else 0,
        )
        if key != self.cur:
            self.cur, self.t_sw, self.t0, self.sel, self.bad_since = key, t, t, 0, None
            if kind == "tripod":
                self.tri = self._tripod(p)
        q = p + [0, 0, 0.1]
        if kind == "tripod":
            cands, look = [self.tri], p + [0, 0, 0.1]
        else:
            cands, look = self._cands(kind, kw, t, p, self.h, side)
            cands = [np.asarray(c, float) for c in cands]
            if self.ok(cands[self.sel], q, 0.0):
                self.bad_since = None
            else:
                self.bad_since = t if self.bad_since is None else self.bad_since
                if t - self.bad_since >= 0.12:
                    good = [j for j, c in enumerate(cands) if self.ok(c, q, 0.0)]
                    self.sel, self.bad_since = (good[0] if good else self.sel), None
        pos, look = np.asarray(cands[self.sel], float).copy(), np.asarray(look, float)
        if self.cpos is None:
            self.cpos, self.clook = pos.copy(), look.copy()
        tau = 0.9 if (t - self.t_sw < 1.8 or kind == "tripod") else 0.35
        self.cpos = self.cpos + (pos - self.cpos) * (1 - math.exp(-dt / tau))
        self.clook = self.clook + (look - self.clook) * (1 - math.exp(-dt / 0.25))
        cam = self.cpos.copy()
        cam[2] = max(cam[2], self.ground(cam[0], cam[1], cam[2]) + 0.6)
        return dict(
            pos=cam,
            lookat=self.clook.copy(),
            fovy={"tripod": 32.0, "side": 40.0, "establish": 55.0}.get(kind, 48.0),
        ), kind

    def _tripod(self, p):
        """A fixed camera low beside the touchdown point, looking at the feet."""
        for r in (2.2, 3.0, 4.0):
            for ang in np.arange(0, 2 * math.pi, math.radians(20)):
                c = p + [r * math.cos(ang), r * math.sin(ang), 0.0]
                c[2] = self.ground(c[0], c[1], p[2]) + 0.45
                if self.ok(c, p + [0, 0, 0.1], 0.0):
                    return c
        return p + [3.0, 0.0, 0.8]


# ---- sim: replay + offscreen render ----------------------------------------------------------------------------------------------------------
def cmd_sim(a):
    import napl
    from napl import _napl  # noqa: F401  (renderer init)
    from screenshots import Replay

    run, out = Landing(a.run_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    napl.render_style("outdoor,shadows")
    r = Replay(a.run_dir)
    r.dt = 0.001
    body = (
        "/world/hopper_frame"
        if "hopper_frame" in " ".join(r.sim.body_positions())
        else "/world/frame"
    )
    d = Director(r.sim, body)
    times = schedule(run)
    meta = []
    for k, (t, rate) in enumerate(times):
        tick = r.to(t)
        cam, kind = d.step(run, t, rate / FPS)
        img, _ = r.sim.render(width=W, height=H, **cam)
        videnc.save_frame(out, f"f{k:05d}", np.asarray(img))
        meta.append(dict(k=k, t=round(t, 3), tick=int(tick), shot=kind, rate=rate, pos=cam["pos"].round(3).tolist(), lookat=cam["lookat"].round(3).tolist(), fovy=cam["fovy"],
                         vpos=[round(float(v), 3) for v in r.sim.query(body, "pos")[0]]))  # fmt: skip
    h, ok = r.finish()
    (out / "sim.json").write_text(
        json.dumps(
            dict(
                run=str(run.dir),
                fps=FPS,
                replay_hash=h,
                replay_matches_run=ok,
                native_hash=run.native["state_hash"],
                frames=meta,
            )
        )
    )
    print(
        f"{len(times)} frames in {out}; replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} native.json; {len(times) / FPS:.1f} s of video"
    )
    sys.exit(0 if ok else 1)


# ---- encode: HUD, cards, captions ------------------------------------------------------------------------------------------------------------
def hud(w, run, m, base, sim_t):
    """The frame with the HUD: phase chip, rate badge, banners, the telemetry panel, the four jet-vane gauges, the vehicle's altitude drop line."""
    from PIL import Image, ImageDraw

    cam = {
        k: np.array(m[k], float) if k != "fovy" else m[k]
        for k in ("pos", "lookat", "fovy")
    }
    img = base.convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    F = w.F
    r = run.rec(sim_t)
    st = r["state"]
    est, tru, va = r["est"], r["truth"], r["vanes"]
    gp = rs.project(cam, m["vpos"])
    if gp and m["shot"] != "establish":
        dr.text(
            (gp[0] + 22, gp[1] - 34), "HOPPER", font=F["m"], fill=(255, 210, 80, 255)
        )
        dr.line(
            [(gp[0] + 6, gp[1] - 6), (gp[0] + 20, gp[1] - 24)],
            fill=(255, 210, 80, 220),
            width=2,
        )
    mode = PHASE_TEXT.get(st, st)
    dr.rounded_rectangle(
        [14, 14, 14 + 28 + dr.textlength(mode, font=F["b"]), 54],
        radius=8,
        fill=(0, 0, 0, 175),
    )
    dr.text((28, 21), mode, font=F["b"], fill=(255, 255, 255, 255))
    x = 14
    for i, s_ in enumerate(STAGES):  # the stage strip: where in the state machine
        c = (255, 210, 80, 255) if s_ == st else (150, 156, 168, 200)
        tw = dr.textlength(s_, font=F["h"])
        dr.text((x, 60), s_, font=F["h"], fill=c)
        x += tw + 11
    y = 82
    if m["rate"] != 1.0:
        lab = (
            f"SLOW MOTION 1/{round(1 / m['rate'])}: first contact"
            if m["rate"] < 1
            else f"{m['rate']:g}x REAL TIME"
        )
        dr.rounded_rectangle(
            [14, y, 14 + 28 + dr.textlength(lab, font=F["m"]), y + 30],
            radius=8,
            fill=(0, 0, 0, 175),
        )
        dr.text((28, y + 5), lab, font=F["m"], fill=(255, 210, 80, 255))
        y += 36
    banners = []
    if run.gust_at(sim_t):
        banners.append(
            (
                "GUST FRONT: the wind at 10 m jumps; the disturbance observer subtracts the push",
                True,
            )
        )
    if r.get("rain_mmh", 0) > 0:
        banners.append(
            (
                f"RAIN {r['rain_mmh']:.0f} mm/h: lidar dropouts and false near returns, thrust loss",
                False,
            )
        )
    if st in ("ABORT_LAND", "FAULT"):
        banners.append((f"{st}: {r['flags']['abort'] or 'fault'}", True))
    if run.t_td is not None and sim_t >= run.t_td and sim_t < run.t_td + 8.0:
        k_ = min(
            int(np.searchsorted(run.tr["t"], run.t_td + 0.5)), len(run.tr["t"]) - 1
        )
        banners.append(
            (
                f"TOUCHDOWN {run.tr['td_speed_ms'][k_]:.2f} m/s, tilt {run.tr['td_tilt_deg'][k_]:.1f} deg, {run.tr['td_dist_m'][k_]:.2f} m from the pad centre",
                run.tr["td_speed_ms"][k_] > 0.5,
            )
        )
    for txt, bad in banners:
        for sub in w.wrap(txt, F["m"], 640, dr):
            tw = dr.textlength(sub, font=F["m"])
            dr.rounded_rectangle(
                [14, y, 14 + 24 + tw, y + 28],
                radius=8,
                fill=(90, 0, 0, 200) if bad else (0, 70, 20, 200),
            )
            dr.text((26, y + 4), sub, font=F["m"], fill=(255, 255, 255, 255))
            y += 32
    f_ = lambda v, s, u: "   --" if v is None else format(v, s) + u
    gs = None if est.get("vx_ms") is None else math.hypot(est["vx_ms"], est["vy_ms"])
    air = r.get("air", {})
    rho = air.get("rho_true")
    wind = r.get("wind_ref_ms")
    rows = [("ALT vs TAKE-OFF (est)", f_(est.get("alt_m"), "6.1f", " m")), ("VERTICAL SPEED", f_(est.get("vz_ms"), "+6.2f", " m/s")), ("GROUND SPEED", f_(gs, "6.2f", " m/s")),
            ("LIDAR HEIGHT", f_(r.get("lidar_m") if 0.1 < r.get("lidar_m", 0) < 8.0 else None, "6.2f", " m").replace("   --", "out of range")), ("TILT", f_(est.get("tilt_deg"), "6.1f", " deg")), ("THROTTLE", f"{100 * r['throttle']:5.0f} %"),
            ("THRUST", f"{r['thrust_N']:6.1f} N"), ("CHARGE LEFT", f"{100 * r['battery']['soc']:5.0f} %")]  # fmt: skip
    if rho:
        rows.append(("AIR DENSITY", f"{rho:5.3f} kg/m3 ({100 * rho / 1.225:.0f} %)"))
    if air.get("margin"):
        rows.append(("T/W MARGIN", f"{air['margin']:5.2f}"))
    if wind:
        rows.append(("WIND 10 m", f"{math.hypot(wind[0], wind[1]):5.1f} m/s"))
    x1, y0, wid = W - 14, 14, 400
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
    gw, gh, lim = (
        270,
        112,
        17.0,
    )  # the four vane gauges: the measured angle as the fill on a +-17 deg bar
    gx, gy = 14, H - 14 - gh
    dr.rounded_rectangle([gx, gy, gx + gw, gy + gh], radius=8, fill=(0, 0, 0, 175))
    dr.text(
        (gx + 10, gy + 5),
        "JET VANE ANGLES  limit +-17 deg",
        font=F["h"],
        fill=(255, 196, 60, 255),
    )
    for k in range(4):
        yy = gy + 30 + 20 * k
        ang = va[f"a{k + 1}_deg"]
        xm = gx + 140
        xa = xm + 60 * max(-1, min(1, ang / lim))
        dr.text(
            (gx + 10, yy - 7),
            ["V1 +x", "V2 +y", "V3 -x", "V4 -y"][k],
            font=F["sm"],
            fill=(190, 196, 208, 255),
        )
        dr.rectangle(
            [xm - 60, yy - 5, xm + 60, yy + 5],
            fill=(40, 44, 52, 255),
            outline=(90, 95, 105, 255),
        )
        dr.rectangle(
            [min(xm, xa), yy - 4, max(xa, xm + 1.5), yy + 4],
            fill=(60, 220, 255, 255) if abs(ang) < 0.85 * lim else (255, 175, 40, 255),
        )
        dr.text(
            (gx + gw - 10 - dr.textlength(f"{ang:+5.1f}", font=F["sm"]), yy - 7),
            f"{ang:+5.1f}",
            font=F["sm"],
            fill=(255, 255, 255, 255),
        )
    return Image.alpha_composite(img, ov).convert("RGB"), mode


def verdict(w, run, pic):
    """The last frame with the monitors' verdict panel."""
    from PIL import ImageDraw

    img = pic.convert("RGBA")
    ov = w.Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    res = run.program
    mons = res["monitors"]
    ok = res["pass"] and res["replay"]["match"]
    hh = 70 + 24 * len(mons) + 30
    x0, y0 = W // 2 - 300, H // 2 - hh // 2
    dr.rounded_rectangle(
        [x0, y0, x0 + 600, y0 + hh],
        radius=12,
        fill=(0, 0, 0, 215),
        outline=(90, 255, 130, 255) if ok else (255, 90, 80, 255),
        width=3,
    )
    dr.text(
        (x0 + 20, y0 + 12),
        ("PASS" if ok else "FAIL") + f"  {res['name']}",
        font=w.F["b"],
        fill=(90, 255, 130, 255) if ok else (255, 110, 100, 255),
    )
    for i, mo in enumerate(mons):
        dr.text(
            (x0 + 24, y0 + 54 + 24 * i),
            ("ok   " if mo["pass"] else "FAIL ") + mo["name"],
            font=w.F["m"],
            fill=(230, 235, 240, 255) if mo["pass"] else (255, 110, 100, 255),
        )
    dr.text(
        (x0 + 24, y0 + hh - 30),
        f"state hash {run.native['state_hash']}  replay {'matches' if res['replay']['match'] else 'DIFFERS'}",
        font=w.F["sm"],
        fill=(190, 196, 208, 255),
    )
    return w.Image.alpha_composite(img, ov).convert("RGB")


CHAPTERS = {
    "build": ("1  Build", ["From an EMPTY design in the napl viewer: the 80 mm ducted fan, motor, ESC, battery, four jet-vane servos and vanes, four legs, the Teensy 4.0 flight computer, regulators, the ZED-F9P RTK GNSS and the downward TF-Luna lidar the landing law runs on. Library search, animated drop onto the mate ports, parameters with provenance, device bindings, live mass and design-rule checks."]),
    "wiring": ("2  Wiring", ["The WIRING page: the component picker, then the pin-level sheet with the real connector pins, the legend and controls top-left. A pin mistake (the 22 V pack onto the Teensy's VIN) is caught by the DRC; hiding parts takes their cards and wires with them; then the wires as tubes routed around the parts in 3D."]),
    "program": ("3  Program", ["The PROGRAM page is an IDE for the landing code: syntax colouring, go to definition, find / replace / find in files, rename, device tooltips, a pty terminal. The Run slot benches a component (BENCH: the TF-Luna model) and flies a scenario live in SIMULATE (FULL SIM) with the monitors' verdict."]),
}  # fmt: skip


def _sim_frames(w, d, title, blurb, hold_s=6.0):
    run = Landing(d)
    sim = json.loads((Path(d) / "sim" / "sim.json").read_text())
    w.card(title, [blurb], 5.5, title)
    last = None
    for m in sim["frames"]:
        t = m["t"]
        base = videnc.load_frame(Path(d) / "sim", f"f{m['k']:05d}")
        last, _ = hud(w, run, m, base, t)
        right = [
            f"sim t = {t:5.1f} s"
            + (
                f"   {m['rate']:g}x"
                if m["rate"] > 1
                else f"   slow 1/{round(1 / m['rate'])}"
                if m["rate"] < 1
                else ""
            ),
            f"camera: {m['shot']}",
        ]
        w.emit(w.bar(last, "simulate", title, blurb, right, None, None))
    vp = verdict(w, run, last)
    for _ in range(int(hold_s * FPS)):
        w.emit(
            w.bar(
                vp,
                "simulate",
                title,
                blurb,
                ["final frame", "monitors' verdict"],
                None,
                None,
            )
        )
    return run, sim


def cmd_encode(a):
    import record_cuas_lite as rc

    root = Path(a.root)
    enc = videnc.Encoder(Path(a.out), W, rc.HO, FPS, gpl=a.gpl)
    w = rc.Writer(enc)
    w.card("A jet-vane hopper that lands: from an empty design to eight landings on mountains, in rain and on a ship", [
        "The 1.3 kg Grasshopper-style EDF hopper (80 mm ducted fan, four jet vanes, Teensy 4.0, lidar, RTK GNSS), built from the napl parts library in the viewer, wired pin by pin, programmed in the PROGRAM IDE and flown in simulation from pad to pad: a mesa and a ridge at 3200-3800 m in thin air, a lee ledge, 25 and 100 mm/h rain, a scripted moving deck and a 12 m offshore support vessel under way in a JONSWAP sea.",
        "Every flight is a napl program run with monitors and a state hash; the picture is a replay of exactly that log. Details, limits and the Monte Carlo tables: examples/hopper-land/README.md",
    ], 8.0, "title")  # fmt: skip
    rc._build_frames(w, root, a.speedup, CHAPTERS)
    w.card("The landing pipeline", [
        "IMU + barometer + GNSS (RTK) + lidar + battery  ->  landest: lidar trusted only over a surveyed pad or the deck, density from the barometer  ->  landflight state machine",
        "feed-forward T_full = 25.3 (rho / 1.225)^0.63 N, firmware go / no-go on T/W >= 1.15, gust push subtracted (k_dob), cruise height from the terrain survey",
        "final: sink on the lidar height; touchdown when the 0.1 s mean height is under 3 cm (one lidar sample is not trusted); fan off",
        "decks: moving-base RTK + the vessel's motion unit + a forecast of the deck height; wait for a quiet window; hold the last 0.45 m while the deck rises",
    ], 8.0, "PIPELINE", mono=4)  # fmt: skip
    w.card("4  Simulate", [
        "Each scenario is a napl program (examples/hopper-land/*/*.scenario.toml): the kernel steps the world, the flight software flies, the runner judges the trace on the monitors and re-runs the replay log to check the state hash. The picture replays exactly that log with a camera that stays close to the vehicle; calibration and transit are sped up and labelled, the approach runs in real time, the first contact in slow motion."
    ], 5.0, "SIMULATE")  # fmt: skip
    runs = []
    for sub, _, title, blurb in SCENARIOS:
        d = root / sub
        if (d / "sim" / "sim.json").exists():
            runs.append((sub, *_sim_frames(w, d, title, blurb)))
    lines = []
    for sub, run, sim in runs:
        res = run.program
        ok = "PASS" if res["pass"] else "FAIL"
        if not res["replay"]["match"]:
            ok += " (replay DIFFERS)"
        k_ = min(
            int(np.searchsorted(run.tr["t"], (run.t_td or run.t_end) + 0.5)),
            len(run.tr["t"]) - 1,
        )
        lines.append(
            f"{sub:14s} {ok}  {run.t_end:5.1f} s  touchdown {run.tr['td_speed_ms'][k_]:.2f} m/s {run.tr['td_tilt_deg'][k_]:4.1f} deg {run.tr['td_dist_m'][k_]:.2f} m  {run.native['state_hash']}"
        )
    w.card("Result of the eight runs shown", lines + ["Each hash is the state hash at the end of the run; the replay behind the frames above ends on exactly that hash. Seed 1 each: one draw per family, not a pass rate."], 12.0, "RUNS", mono=len(lines))  # fmt: skip
    w.card("Monte Carlo over seeds (examples/hopper-land/README.md, not rerun for this video)", [
        "60 held-out report seeds from 21000 per family (osv: 40); success = touchdown speed < 0.5 m/s (relative on a deck), tilt < 5 deg, within 0.5 m of the pad, no abort, clearance >= 1 m; Wilson 95 % interval",
        "mesa down 100 % [94, 100]   mesa up 93.3 % [84, 97]   rain 25 mm/h 100 % [94, 100]   rain 100 mm/h 100 % [94, 100]   scripted deck 98.3 % [91, 100]",
        "ridge 75.0 % [63, 84] (wind x1.5: 45 %, x2: 0 %)   ledge 80.0 % [68, 88] (wind x1.5: 30 %, x2: 5 %): failures are aborts at peak winds of 9-18 m/s",
        "OSV under way (n = 40): 80.0 % [65, 90]. Envelope, n = 16 each: Hs 0.25 m 87.5 %, Hs 0.5 m 87.5 %, Hs 1.0 m 50 % [28, 72], Hs 1.5 m 37.5 % [19, 61]; vessel stopped 93.8 %, 3.5 m/s 25 %, beam seas 25 %, deck link 0.3 s late 37.5 %",
        "So: up to Hs 0.5 m, head seas, vessel stopped to 2 m/s the landing works 80-88 %; the >= 90 % target is NOT met on the ship. From Hs 1.0 m it lands 40-50 % (deck tilt at contact over 5 deg). Aborting over a ship lands in the sea: not handled.",
    ], 22.0, "MC")  # fmt: skip
    w.card("What is real and what is a stand-in", [
        "Real: the parts, assembly, wiring and DRC; the kernel's physics, EDF and battery models, the rain device, the terrain and wind field, the JONSWAP sea and the vessel's hull; the lidar ray casts; the estimator, state machine, deck predictor code that flies.",
        "Stand-ins and estimates: standard-day air only; the thrust fit T_full is extrapolated above 5500 m (firmware refuses at 9000 m); rain catch / wet areas are estimates; the scripted deck's motion and link noise are invented; the rover antenna sits at the hopper frame origin; the terrain is synthetic and seeded, not surveyed.",
        "Limits: no terrain-clearance guard without the survey map; the ship rows are small samples (n = 16-40); on screen the pad truth and the touchdown distance are scoring information the flight software does not have.",
    ], 16.0, "STAND-INS")  # fmt: skip
    enc.close()
    print(
        f"wrote {a.out}: {w.n} frames = {w.n / FPS:.1f} s, {Path(a.out).stat().st_size / 1e6:.1f} MB, encoder {enc.backend}"
    )
    print(
        "timeline (output seconds):", ", ".join(f"{t:.0f}s {lab}" for t, lab in w.marks)
    )


def cmd_png(a):
    import types

    import record_cuas_lite as rc

    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    sj = Path(a.frames_dir) / "sim.json"
    if sj.exists():
        w = types.SimpleNamespace(F=rc._fonts(), wrap=rc.Writer.wrap)
        run = Landing(Path(a.frames_dir).parent)
        fr = json.loads(sj.read_text())["frames"]
        for i in a.idx:
            pic, _ = hud(
                w, run, fr[i], videnc.load_frame(a.frames_dir, f"f{i:05d}"), fr[i]["t"]
            )
            pic.save(out / f"{run.dir.name}_hud_{i:05d}.png")
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
    for name in ("build", "_build"):
        s = sub.add_parser(name)
        s.add_argument("oplog")
        s.add_argument("out_dir")
        if name == "_build":
            s.add_argument("fb")
        s.add_argument("--expect", required=True)
        s.add_argument("--save")
        s.add_argument("--open")
        s.add_argument("--timeout", type=float, default=2800)
        s.add_argument("--quick-until", type=int, default=0)
        s.add_argument("--steps", type=int, default=0)
        s.add_argument("--keep-frames", action="store_true")
    s = sub.add_parser("explore")
    s.add_argument("oplog")
    s.add_argument("--expect", required=True)
    s.add_argument("--save")
    s.add_argument("--open")
    s.add_argument("--quick-until", type=int, default=0)
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
    {
        "oplog": cmd_oplog,
        "build": cmd_build,
        "_build": _build,
        "explore": cmd_explore,
        "sim": cmd_sim,
        "encode": cmd_encode,
        "png": cmd_png,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
