"""Recording harness of the recon-quadruped video (media is never committed: frames and the webm stay on the host). Part (a) construction, part (b) programming
overview (diagrams drawn from a tap of the run) and part (c) sim demo (tick-exact replay); the cards and the encode come with SP6 Task 9.

Reuses the existing pipelines instead of copying them:
  build / wiring / program  tools/record_turret.py's xvfb viewer capture (`build OPLOG OUT --open assemblies/dog/dog.toml`); this tool only generates the op log
  explode                   the built MJCF with each sub-assembly's topmost body displaced, rendered offscreen with an orbit camera (no viewer op for it exists)

    probe   [DIR]          read-only: MJCF group structure of dogsim.build + the wiring sheets' card preview (what explode_xml / cmd_oplog rely on)
    oplog   OUT.json       construction + wiring + program op log generated from assemblies/dog/dog.toml (needs the Design: run it on the host)
    explode OUT_DIR        frames x*.rgz + explode.json (factor and camera per frame). GPU
    diagrams RUN_DIR OUT_DIR   the seven programming diagrams d*.rgz + diagrams.json from RUN_DIR/tap (a run recorded with DOG_TAP_DIR); refuses a stale tap. CPU
    sim     RUN_DIR OUT_DIR    the run's replay.naplog applied tick-exact, orbit / map cameras: f*.rgz + sim.json; exit 1 if the replay hash differs from the run's. GPU
    encode  ROOT OUT.webm [--speedup 1.5] [--gpl] [--results docs/dog/missions.json]
                           ROOT holds build/ (record_turret.py build), explode/, diagrams/, showcase/sim/; cards + the three parts in order + the Monte Carlo card. GPU encode,
                           GStreamer VA (LGPL) by default, ffmpeg / libx264 only with --gpl. Run it with the video venv (Pillow, numpy)
    png     FRAMES_DIR OUT_DIR IDX...   some frames as PNG, to look at (a sim dir gets its HUD)
"""

import argparse
import json
import math
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples" / "dog"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

W, H, FPS = 1280, 720, 20
ASM = REPO / "assemblies" / "dog" / "dog.toml"
PART_A_MAX_S = 170.0  # seconds of op-log waits part (a) may take in the final video (Task 9 timeline)
LEG_ORDER = [  # build order of one leg (assemblies/dog/README.md sub-assembly tree), revealed one by one inside the leg's own document
    "abad_mount", "abad_motor", "hip_housing", "hip_motor", "knee_motor", "knee_crank", "thigh",
    "knee_rod", "rod_end_a", "rod_end_b", "shank", "foot_core", "foot_ball", "hub",
]  # fmt: skip
CHASSIS = ["deck_top", "bulk_f", "bulk_r", "side_l", "side_r"]
STAGES = [  # (title, why, groups): "chassis" is the five named instances, any other group is every instance whose id starts "<group>:"
    ("1/6  Chassis: carbon deck, bulkheads, side plates",
     "The top carbon deck is the root. Two printed bulkheads (front, rear) and two printed side plates bolt to it; the hips will bolt to the bulkheads.",
     ["chassis"]),
    ("2/6  One leg: abduction, hip and knee QDD actuators, crank + push rod, thigh, shank, foot",
     "The leg (assemblies/dog/leg.toml) opened as its own document: abduction mount and motor, the hip housing carrying the hip and knee GIM8108-8 drives, the knee crank and push rod (ratio 1.5), thigh, shank, foot core and rubber foot ball, and the CAN hub.",
     ["fl"]),
    ("3/6  The leg used four times: fr and rl mirrored, rl and rr turned 180 deg",
     "The same leg file serves all four corners: fr and rl are the leg mirrored about the XZ plane; the rear bulkhead is the front one turned 180 deg about z, so rl and rr hang from it. Front knees point backward, rear knees forward.",
     ["fr", "rl", "rr"]),
    ("4/6  Head: Livox Mid-360 and RealSense D435i",
     "The sensor head on the front of the top deck: Livox Mid-360 lidar (rosette scan) and Intel RealSense D435i depth camera.",
     ["head"]),
    ("5/6  Power: two 8S packs, BMS, contactor, PDB, DC-DCs, e-stop",
     "The power module under the front bulkhead: two 8S1P Li-ion packs with a BMS each, the 100 A contactor with its e-stop coil loop, the PDB feeding one XT60 per leg hub, and the Jetson and aux DC-DCs.",
     ["power"]),
    ("6/6  Compute: Jetson Orin Nano, four CANables, IMU, RTK board",
     "The compute tray on the top deck: Jetson Orin Nano, one CANable per leg bus, the IMU, the RTK GNSS board and its antenna.",
     ["compute"]),
]  # fmt: skip
SHEETS = [  # (title, why, instances): flattened ids; wiring uses collapse=False so the cards are the parts, not the sub-assembly blocks
    ("Power: packs, BMS, contactor, PDB, DC-DCs, e-stop, one leg hub",
     "Sheet 1. Two packs through their BMS to the 100 A contactor, the e-stop on its coil, the PDB and its XT60 feed to the front-left leg hub. Every connector shows its real pins.",
     ["power:pack_a", "power:pack_b", "power:bms_a", "power:bms_b", "power:contactor", "power:estop", "power:pdb", "fl:hub"]),
    ("CAN daisy chain of the front-left leg",
     "Sheet 2. The Jetson's USB to its CANable, the CANable's CAN pair to the leg hub, and the hub to the abduction, hip and knee drives.",
     ["compute:jetson", "compute:can_fl", "fl:hub", "fl:abad_motor", "fl:hip_motor", "fl:knee_motor"]),
]  # fmt: skip
# World displacement of each sub-assembly's topmost body at factor 1 (x forward, y left, z up); legs are placed by the side of their hip at run time (leg_vectors).
LEG_OUT, LEG_DOWN = 0.35, 0.10
VEC = {
    "head:": (0.35, 0.0, 0.15),
    "power:": (0.0, 0.0, -0.30),
    "compute:": (0.0, 0.0, 0.25),
}
LEGS = ("fl:", "fr:", "rl:", "rr:")


# ---- MJCF: body tree with accumulated frames ------------------------------------------------------------------------------------------------------
def _rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _frame(b):
    """(pos, R) of a <body> relative to its parent; only `pos` and `quat` frames exist in the built dog (anything else is refused, not guessed)."""
    for k in ("euler", "axisangle", "xyaxes", "zaxis"):
        if b.get(k) is not None:
            raise ValueError(f"body {b.get('name')}: '{k}' frames are not supported")
    pos = np.array([float(v) for v in (b.get("pos") or "0 0 0").split()])
    q = np.array([float(v) for v in (b.get("quat") or "1 0 0 0").split()])
    return pos, _rot(q)


def _walk(body, parent, p, R, out):
    """out[name] = (world pos, world R, parent body element or None, body element); only <body> children are followed."""
    pos, Rb = _frame(body)
    pw, Rw = p + R @ pos, R @ Rb
    out[body.get("name")] = (pw, Rw, parent, body)
    for c in body.findall("body"):
        _walk(c, body, pw, Rw, out)


def _bodies(root):
    out = {}
    for b in root.find("worldbody").findall("body"):
        _walk(b, None, np.zeros(3), np.eye(3), out)
    return out


def world_pos(xml_text):
    """{body name: world position (3,)} composed down the body tree (positions and quaternions)."""
    return {k: v[0] for k, v in _bodies(ET.fromstring(xml_text)).items()}


def explode_xml(xml_text, groups, factor):
    """The MJCF with every group's topmost body moved by `factor * vector` (vector in WORLD axes; `groups` maps a body-name prefix such as "fl:" to it).

    A group's topmost body is a body whose name starts with the prefix and whose parent's does not. Probe answer (`record_dog.py probe`, dogsim.build of
    assemblies/dog/dog.toml): fl:/fr: abad_mount are children of bulk_f, rl:/rr: abad_mount children of bulk_r, power:deck_bottom a child of bulk_f, head:head_mount and
    compute:tray children of deck_top, all in the chassis tree under the root body deck_top (a child of worldbody, free joint). bulk_r is turned 180 deg about z
    (quat 0 0 0 1), so the displacement is applied in the parent's frame as R_parent^T @ vector and the WORLD displacement is exactly the vector. Nothing is
    renamed or removed: the body set is unchanged. factor = 0 returns the input text untouched."""
    if factor == 0:
        return xml_text
    root = ET.fromstring(xml_text)
    for name, (_, _, parent, body) in _bodies(root).items():
        pre = next((g for g in groups if name.startswith(g)), None)
        if pre is None or parent is None or parent.get("name", "").startswith(pre):
            continue
        Rp = _bodies(
            root
        )[
            parent.get("name")
        ][
            1
        ]  # (recomputed: an earlier shift never changes a parent's frame, only positions)
        pos, _ = _frame(body)
        new = pos + Rp.T @ (factor * np.asarray(groups[pre], float))
        body.set("pos", " ".join(repr(float(v)) for v in new))
    return ET.tostring(root, encoding="unicode")


def leg_vectors(xml_text):
    """Per-leg displacement: outward along the side of the leg's hip (sign of its world y at factor 0) and down."""
    wp = world_pos(xml_text)
    return {
        g: (0.0, math.copysign(LEG_OUT, wp[g + "abad_mount"][1]), -LEG_DOWN)
        for g in LEGS
    }


# ---- the construction / wiring / program op log, generated from the assembly --------------------------------------------------------------------
def expand(group, ids):
    """Instance ids of a stage group: "chassis" is the five named ones, any other is every id starting "<group>:"."""
    got = (
        CHASSIS if group == "chassis" else [i for i in ids if i.startswith(group + ":")]
    )
    assert got and set(got) <= set(ids), f"group {group}: {got}"
    return got


def cmd_oplog(a):
    import dogsim
    from napl.design import Design
    from napl.design.viewer import Backend

    d = Design.open(ASM)
    ids = [i["id"] for i in d.doc["instances"]]
    staged = [x for s in STAGES for g in s[2] for x in expand(g, ids)]
    assert len(staged) == len(set(staged)), "an instance is in two build stages"
    missing = set(ids) - set(staged)
    assert not missing, f"instances in no stage: {sorted(missing)}"
    for _, _, inst in SHEETS:
        assert set(inst) <= set(ids), f"unknown sheet ids {set(inst) - set(ids)}"
    xml, rep = dogsim.build(tempfile.mkdtemp(prefix="napl-oplog-dog-"))
    n_wires = len(rep["wires"])
    R = lambda tool, wait=0.0, **args: {"tool": tool, "args": args, "wait": wait}
    steps = [
        dict(seg="BUILD", title="The finished design, every part hidden", speed=1.0,
             why=f"assemblies/dog/dog.toml: {len(ids)} parts in 4 legs, head, power and compute sub-assemblies on a carbon chassis. The BROWSER's eye hides parts in the 3D view; they come back in the order a builder fits them.",
             reqs=[R("ui.mode", 1.0, name="build"), R("ui.view", 0.1, tooltips="off"), R("select", 0.3, instances=[]), R("ui.visibility", 1.2, visible=False), R("ui.camera", 0.8, preset="iso")]),
    ]  # fmt: skip
    for n, (title, why, groups) in enumerate(STAGES, 1):
        grp = [x for g in groups for x in expand(g, ids)]
        reqs = []
        if (
            groups == ["fl"]
        ):  # the leg as its own document: parts revealed in order, then back to the dog with the leg in place
            reqs += [
                R("sub.open", 1.5, id="fl"),
                R("ui.visibility", 0.4, visible=False),
                R("ui.camera", 0.6, preset="iso"),
            ]
            reqs += [
                R("ui.visibility", 0.5, instances=[p], visible=True) for p in LEG_ORDER
            ]
            later = [x for s in STAGES[n:] for g in s[2] for x in expand(g, ids)]
            reqs += [  # closing the leg document brings the whole dog back (seen in the first capture): hide what is not built yet
                R("sub.close", 0.8, save=False),
                R("ui.visibility", 0.4, instances=later, visible=False),
            ]
        reqs += [
            R("ui.visibility", 0.6, instances=grp, visible=True),
            # no ui.frame (it selects what it frames, record_skydio finding): the dog fits the iso view of step 1, the camera only orbits
            R("camera.orbit", 4.2, yaw_deg=75, pitch_deg=6, seconds=4.0, relative=True),
        ]
        steps.append(
            dict(seg="BUILD", title=title, why=why, speed=1.0, dwell=0.4, reqs=reqs)
        )
    mass = rep["mass"]["total"]
    price = (
        rep["bom"].get("total_usd", rep["bom"].get("total"))
        if isinstance(rep["bom"], dict)
        else None
    )
    cost = f", BOM {price:,.0f} USD" if isinstance(price, (int, float)) else ""
    steps.append(dict(seg="BUILD", title="Mass and DRC", speed=1.0, dwell=1.0,
                      why=f"The Design reports mass and the design-rule check: {mass:.2f} kg{cost} (read from the build report).",
                      reqs=[R("ui.visibility", 0.5, visible=True), R("ui.camera", 1.0, preset="iso"), R("design_check", 2.5)]))  # fmt: skip
    for n, (title, why, inst) in enumerate(SHEETS, 1):
        # the card layout is checked now, not guessed: flatten.preview must accept the ids with the parts as cards
        b = Backend(lambda: d, Path(tempfile.mkdtemp(prefix="napl-oplog-")))
        r = json.loads(
            b(
                json.dumps(
                    {
                        "tool": "flatten.preview",
                        "args": {"instances": inst, "collapse": False},
                    }
                )
            )
        )
        assert r["ok"], f"sheet {n}: {r.get('error')}"
        cards = r["result"]["cards"] if "result" in r else r["flatten"]["cards"]
        assert {c["id"] for c in cards} == set(inst), (
            f"sheet {n}: cards {[c['id'] for c in cards]}"
        )
        key = sorted(
            sorted(
                cards,
                key=lambda c: sum(1 for p in c["pins"] if p.get("wired")),
                reverse=True,
            )[:3],
            key=lambda c: c["x"],
        )
        centres = [
            [round(c["x"] + c["w"] / 2), round(c["y"] + c["h"] / 2)] for c in key
        ]
        reqs = [
            R("ui.mode", 1.4, name="wiring"),
            R("ui.wire_pick", 3.0, instances=inst),
            R("flatten.begin", 3.0, instances=inst, collapse=False),
            R("ui.wire_view", 2.5, fit=True),
        ]
        if (
            n == 1
        ):  # hide the power parts: their cards and wires vanish in 2D and in 3D; show them again
            reqs += [
                R(
                    "ui.visibility",
                    2.5,
                    instances=[i for i in ids if i.startswith("power:")],
                    visible=False,
                ),
                R("ui.mode", 1.0, name="build"),
                R("ui.view", 0.3, emphasise_wires=True),
                R("ui.camera", 3.0, preset="iso"),
                R("ui.visibility", 2.5, visible=True),
                R("ui.mode", 1.5, name="wiring"),
                R("ui.wire_view", 1.5, fit=True),
            ]
        reqs += [
            R("ui.wire_view", 3.4, zoom=0.9, center=c, seconds=2.0) for c in centres[:2]
        ]
        reqs += [R("ui.wire_view", 2.0, fit=True, seconds=1.4), R("flatten.end", 1.0)]
        steps.append(
            dict(seg="WIRING", title=title, why=why, speed=1.0, dwell=0.6, reqs=reqs)
        )
    steps.append(dict(seg="WIRING", title=f"Back to 3D: {n_wires} wires routed as tubes, none through a part", speed=1.0, dwell=0.6,
                      why="In 3D each conductor is a tube with its gauge's diameter, leaving its connector along the connector axis and routed through free space (a distance field of the assembly), never through a part.",
                      reqs=[R("ui.mode", 1.2, name="build"), R("ui.view", 0.5, emphasise_wires=True), R("select", 0.5, instances=[]), R("ui.camera", 1.0, preset="iso"),
                            R("camera.orbit", 8.5, yaw_deg=300, pitch_deg=12, seconds=8.0, relative=True)]))  # fmt: skip
    U = lambda wait, **kw: R("prog.ui", wait, **kw)
    # The PROGRAM page edits `--program-root` (default: the design's folder); the programming story itself is part (b).
    # record_turret build --program-root examples/dog (201d547) points the PROGRAM page at the robot's software; paths below are relative to that root.
    prog = [
        ("loco/controller.py", r"^class Controller"),
        ("autonomy/planner.py", r"^def astar"),
    ]
    for k, (path, pat) in enumerate(prog):
        steps.append(dict(seg="PROGRAM", title="PROGRAM is an IDE: " + Path(path).name, speed=1.0, dwell=0.5,
                          why=f"examples/dog/{path} in the PROGRAM page: file explorer, tabs, syntax colouring. The programming story itself is part (b) of the video.",
                          reqs=([R("ui.mode", 1.5, name="program")] if k == 0 else []) + [U(8.0, cmd="open", path=path, line=_line("examples/dog/" + path, pat), col=0, len=0)]))  # fmt: skip
    total = sum(r["wait"] for s in steps for r in s["reqs"])
    assert total <= PART_A_MAX_S, (
        f"part (a) op log waits {total:.1f} s > {PART_A_MAX_S}"
    )
    Path(a.out).write_text(
        json.dumps(
            {
                "about": "generated by tools/record_dog.py oplog from assemblies/dog/dog.toml",
                "steps": steps,
            },
            indent=1,
        )
    )
    print(
        f"{len(steps)} steps, {total:.1f} s of waits -> {a.out}"
    )  # ttsr-ignore: py/no-print


def _line(path, pat):
    """0-based line of the first line of `path` matching the regex."""
    import re

    return next(
        i
        for i, ln in enumerate((REPO / path).read_text().splitlines())
        if re.search(pat, ln)
    )


# ---- exploded view -------------------------------------------------------------------------------------------------------------------------------
def cmd_explode(a):
    import dogsim
    import napl
    import videnc
    from napl import Sim

    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    xml, _ = dogsim.build(out / "model")
    text = xml.read_text()
    groups = {**VEC, **leg_vectors(text)}
    napl.render_style(
        "studio,nowires"
    )  # displaced parts would drag the straight wire chords through other parts (seen in the first render): no wires here
    base = world_pos(text)["deck_top"]
    n = 40
    meta = []
    for k in range(n):
        f = (
            math.sin(math.pi * k / (n - 1)) ** 2
        )  # 0 -> 1 -> 0: the parts fly apart, then join
        p = xml.with_name(
            f"exploded_{k:03d}.xml"
        )  # next to the built model: its mesh paths stay valid
        p.write_text(explode_xml(text, groups, f))
        sim = Sim.load(p)
        az = 40.0 + 3.0 * k
        dist = 1.4 + 0.9 * f
        img, _ = sim.render(
            width=W,
            height=H,
            camera="orbit",
            az=az,
            el=18.0,
            dist=dist,
            target=(float(base[0]), float(base[1]), float(base[2]) - 0.1),
        )
        videnc.save_frame(out, f"x{k:05d}", np.asarray(img))
        meta.append(dict(k=k, factor=round(f, 4), az=az, el=18.0, dist=round(dist, 3)))
        p.unlink()
    (out / "explode.json").write_text(
        json.dumps(
            dict(fps=FPS, vectors={k: list(v) for k, v in groups.items()}, frames=meta)
        )
    )
    print(f"{n} frames in {out}")  # ttsr-ignore: py/no-print


# ---- probe ---------------------------------------------------------------------------------------------------------------------------------------
def cmd_probe(a):
    import dogsim
    from napl.design import Design
    from napl.design.viewer import Backend

    xml, rep = dogsim.build(tempfile.mkdtemp(prefix="napl-probe-dog-"))
    text = xml.read_text()
    allb = _bodies(ET.fromstring(text))
    pre = ("fl:", "fr:", "rl:", "rr:", "head:", "power:", "compute:")
    for g in pre:
        for name, (pw, _, parent, _b) in allb.items():
            if name.startswith(g) and not parent.get("name", "").startswith(g):
                print(
                    f"group {g:9s} top={name:20s} parent={parent.get('name'):10s} world={np.round(pw, 3).tolist()}"
                )  # ttsr-ignore: py/no-print
    print("leg vectors", leg_vectors(text))  # ttsr-ignore: py/no-print
    print(
        "bodies",
        len(allb),
        "wires in report",
        len(rep["wires"]),
        "mass",
        rep["mass"]["total"],
        "bom keys",
        list(rep["bom"])[:6] if isinstance(rep["bom"], dict) else type(rep["bom"]),
    )  # ttsr-ignore: py/no-print
    d = Design.open(ASM)
    b = Backend(lambda: d, Path(tempfile.mkdtemp(prefix="napl-probe-")))
    for title, _, inst in SHEETS:
        r = json.loads(
            b(
                json.dumps(
                    {
                        "tool": "flatten.preview",
                        "args": {"instances": inst, "collapse": False},
                    }
                )
            )
        )
        print(
            title, "ok" if r["ok"] else r.get("error"), list(r.keys())
        )  # ttsr-ignore: py/no-print
        if r["ok"]:
            cards = r["result"]["cards"] if "result" in r else r["flatten"]["cards"]
            print(
                [(c["id"], sum(1 for p in c["pins"] if p.get("wired"))) for c in cards]
            )  # ttsr-ignore: py/no-print


# ---- part (b): the programming overview, block diagrams drawn from the run's tap --------------------------------------------------------------
PART_B_MAX_S = 110.0
DIAGRAMS = [  # (key, title, blurb, seconds): seven diagrams, 12-16 s each
    ("estimator", "State estimator", "IMU + leg kinematics + foot contact in a Kalman filter; the numbers are this run's estimate against ground truth.", 15.0),
    ("mpc_wbc", "Convex MPC and whole-body controller", "The MPC picks stance-foot forces at 50 Hz, the QP whole-body controller turns them into joint torques at 500 Hz.", 16.0),
    ("footholds", "Gait and footholds", "The gait scheduler picks stance and swing; each swing foot gets a touchdown target (Raibert + capture point, terrain-aware).", 15.0),
    ("elevmap", "Elevation map", "Each lidar frame is fused into a 5 cm height grid with a variance per cell; a cell is mapped once its variance is below (5 cm)^2.", 15.0),
    ("localizer", "Localization", "Leg odometry predicts; RTK GNSS and scan-to-map ICP correct. Outages of the sky view hand the pose to ICP.", 14.0),
    ("planner", "Coverage planner", "Lanes over the mapped, traversable area; the path avoids lethal cells and tells the controller where to go.", 15.0),
    ("executive", "Mission executive", "SWEEP the lanes, FILL the holes, RETURN home before the budget is spent, then DONE (or ABORT on a fall).", 14.0),
]  # fmt: skip
FEET = ("FL", "FR", "RL", "RR")  # loco.model.LEGS order
BG, LINE, ACC, GOOD, BAD, TXT, DIM = (
    (12, 14, 18),
    (90, 100, 115),
    (255, 196, 60),
    (110, 220, 120),
    (255, 100, 90),
    (225, 228, 235),
    (150, 156, 168),
)
_F = {}


def _fonts():
    if not _F:
        import record_cuas_lite as rc

        _F.update(rc._fonts())
    return _F


def phase_names():
    """{phase int: name} read from the executive (SP5), so a renamed phase follows."""
    from autonomy import mission

    return {
        v: k
        for k, v in vars(mission.Executive).items()
        if k.isupper() and isinstance(v, int)
    }


class TapData:
    """A mission run dir recorded with DOG_TAP_DIR=<run>/tap: the tap arrays plus the run's trace.csv (optional columns read as nan). A tap whose state hash differs from
    the run's result.json is refused: the diagrams must come from the run the sim frames replay."""

    def __init__(self, run_dir):
        d = Path(run_dir)
        meta = json.loads((d / "tap" / "tap.json").read_text())
        have = json.loads((d / "result.json").read_text())["sim"]["state_hash"]
        if str(meta["state_hash"]) != str(have):
            raise SystemExit(
                f"stale tap: tap hash {meta['state_hash']} != run hash {have} ({d})"
            )
        z = np.load(d / "tap" / "tap.npz")
        self.z, self.meta, self.dir = {k: z[k] for k in z.files}, meta, d
        self.n, self.t = len(self.z["t"]), self.z["t"]
        self.cum = np.r_[0, np.cumsum(self.z["path_len"])]
        self.tr = {}
        if (d / "trace.csv").exists():
            import csv

            rows = list(csv.DictReader(open(d / "trace.csv")))
            self.tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}

    def est_error(self, i):
        return float(np.hypot(*(self.z["est_xy"][i] - self.z["truth_xy"][i])))

    def force_sum(self, i):
        return float(np.nansum(self.z["force_z"][i]))

    def gait_name(self, i):
        from autonomy import videotap

        g = int(self.z["gait"][i])
        return videotap.GAITS[g] if 0 <= g < len(videotap.GAITS) else "?"

    def mode_name(self, i):
        from autonomy import videotap

        m = int(self.z["mode"][i])
        return videotap.MODES[m] if 0 <= m < len(videotap.MODES) else "?"

    def col(self, name, i):
        """A trace.csv column at tap sample i (interpolated in time); nan if the run has no trace."""
        return (
            float(np.interp(self.t[i], self.tr["t"], self.tr[name]))
            if name in self.tr
            else float("nan")
        )

    def mapped(self, i):
        """-> bool (n, n) mask of the newest map snapshot at or before sample i (rows = y, cols = x, origin at the field's south-west corner)."""
        j = max(int(np.searchsorted(self.z["map_t"], self.t[i], side="right")) - 1, 0)
        shape = tuple(int(v) for v in self.z["mapped_shape"])
        return (
            np.unpackbits(self.z["mapped_packed"][j])[: shape[0] * shape[1]]
            .reshape(shape)
            .astype(bool)
        )

    def path(self, i):
        return self.z["path_xy"][self.cum[i] : self.cum[i + 1]]

    def map_geometry(self):
        """(x0, cell) of the mapped mask: the field is centred on the origin; cell = map.res x plan.pool (autonomy.toml)."""
        from autonomy import config

        cell = config.load()["map"]["res"] * self.meta["pool"]
        return -0.5 * self.z["mapped_shape"][0] * cell, cell


def _canvas(size, title, blurb, files):
    from PIL import Image, ImageDraw

    img = Image.new("RGB", size, BG)
    dr = ImageDraw.Draw(img)
    F = _fonts()
    dr.text((40, 22), title.upper(), font=F["t"], fill=TXT)
    dr.text((40, 76), blurb, font=F["s"], fill=DIM)
    dr.text((40, size[1] - 34), "code: " + "   ".join(files), font=F["sm"], fill=ACC)
    return img, dr, F


def _box(dr, F, x, y, w, h, head, sub="", col=LINE):
    dr.rectangle([x, y, x + w, y + h], outline=col, width=2, fill=(20, 23, 29))
    dr.text((x + 10, y + 8), head, font=F["b"], fill=TXT)
    for k, ln in enumerate(sub.split("\n") if sub else []):
        dr.text((x + 10, y + 38 + 19 * k), ln, font=F["sm"], fill=DIM)


def _arrow(dr, a, b, col=LINE):
    dr.line([a, b], fill=col, width=3)
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    for s in (-0.45, 0.45):
        dr.line(
            [b, (b[0] - 14 * math.cos(ang + s), b[1] - 14 * math.sin(ang + s))],
            fill=col,
            width=3,
        )


def _lights(dr, F, x, y, vals, names=FEET):
    for k, v in enumerate(vals):
        c = DIM if not np.isfinite(v) else (GOOD if v > 0.5 else BAD)
        dr.ellipse([x + 62 * k, y, x + 62 * k + 22, y + 22], fill=c)
        dr.text((x + 62 * k + 28, y + 2), names[k], font=F["sm"], fill=TXT)


def _fmt(v, f, unit=""):
    return "n/a" if not np.isfinite(v) else f"{v:{f}}{unit}"


def _trail(dr, box, series, i, cols, win=200):
    """Polylines of the last `win` samples of each (n, 2) series, shared scale, drawn into box = (x, y, w, h)."""
    x, y, w, h = box
    dr.rectangle([x, y, x + w, y + h], outline=LINE, width=1)
    lo = max(0, i - win)
    pts = np.concatenate([s[lo : i + 1] for s in series])
    c, r = pts.mean(0), max(float(np.ptp(pts, axis=0).max()), 0.5) * 0.6
    for s, col in zip(series, cols):
        p = [
            (
                x + w / 2 + (q[0] - c[0]) / r * w / 2,
                y + h / 2 - (q[1] - c[1]) / r * h / 2,
            )
            for q in s[lo : i + 1]
        ]
        if len(p) > 1:
            dr.line(p, fill=col, width=2)
        dr.ellipse([p[-1][0] - 5, p[-1][1] - 5, p[-1][0] + 5, p[-1][1] + 5], fill=col)


def _mask_image(tap, i, side):
    from PIL import Image

    m = tap.mapped(i)[::-1]  # y up
    a = np.where(
        m[..., None], np.array(GOOD, np.uint8) // 2, np.array((30, 34, 42), np.uint8)
    ).astype(np.uint8)
    return Image.fromarray(a).resize((side, side), Image.NEAREST)


def _to_px(p, x0, cell, n, ox, oy, side):
    """World (x, y) -> pixel on a mask image of `side` px drawn at (ox, oy) (y up)."""
    return ox + (p[0] - x0) / (n * cell) * side, oy + side - (p[1] - x0) / (
        n * cell
    ) * side


def draw_estimator(tap, i, size):
    img, dr, F = _canvas(
        size,
        "State estimator",
        DIAGRAMS[0][2],
        ["examples/dog/loco/estimator.py", "examples/dog/loco/hw.py"],
    )
    _box(dr, F, 40, 140, 300, 90, "IMU", "gyro + accel, 500 Hz")
    _box(
        dr,
        F,
        40,
        270,
        300,
        90,
        "Leg kinematics",
        "joint q, dq from the QDD drives\nfoot contact: virtual f_z threshold",
    )
    _box(
        dr,
        F,
        40,
        400,
        300,
        90,
        "Pose fixes (SP5)",
        "RTK GNSS + scan ICP, loco Estimator.fix",
    )
    _box(
        dr,
        F,
        450,
        220,
        330,
        190,
        "Kalman estimator",
        "stance feet are still: p_foot = p + R r(q)\nswing feet free; process v += (R a + g) dt",
        ACC,
    )
    for y in (185, 315, 445):
        _arrow(dr, (340, y), (450, 315 if y != 445 else 380))
    _arrow(dr, (780, 315), (860, 315))
    _box(dr, F, 860, 260, 190, 110, "pose, velocity", "-> gait, MPC, WBC")
    dr.text((450, 440), "foot contact (from f_z)", font=F["sm"], fill=DIM)
    _lights(dr, F, 450, 465, tap.z["contact"][i])
    e = tap.est_error(i)
    dr.text(
        (450, 515),
        f"estimate vs ground truth: {100 * e:.1f} cm",
        font=F["b"],
        fill=GOOD if e < 0.3 else BAD,
    )
    dr.text(
        (450, 548),
        f"t = {tap.t[i]:6.1f} s   mode {tap.mode_name(i)}",
        font=F["m"],
        fill=TXT,
    )
    dr.text((860, 395), "truth (white) / estimate (amber)", font=F["sm"], fill=DIM)
    _trail(
        dr,
        (860, 420, 360, 240),
        [tap.z["truth_xy"], tap.z["est_xy"]],
        i,
        [(235, 235, 240), ACC],
    )
    return img


def draw_mpc_wbc(tap, i, size):
    from loco.controller import DT, MPC_EVERY

    img, dr, F = _canvas(
        size,
        "Convex MPC and whole-body controller",
        DIAGRAMS[1][2],
        [
            "examples/dog/loco/mpc.py",
            "examples/dog/loco/wbc.py",
            "examples/dog/loco/controller.py",
        ],
    )
    _box(
        dr,
        F,
        40,
        180,
        300,
        150,
        "Convex MPC",
        f"{1 / (DT * MPC_EVERY):.0f} Hz, condensed QP\nstate: pose, velocity\ncommand: foot forces over the horizon",
        ACC,
    )
    _arrow(dr, (340, 255), (470, 255))
    f = tap.z["force_z"][i]
    dr.text(
        (480, 150),
        f"stance force per foot (z), sum {_fmt(tap.force_sum(i), '.0f', ' N')}",
        font=F["s"],
        fill=TXT,
    )
    top = max(
        float(np.nanmax(tap.z["force_z"]))
        if np.isfinite(tap.z["force_z"]).any()
        else 1.0,
        1.0,
    )
    for k in range(4):
        v = 0.0 if not np.isfinite(f[k]) else float(f[k])
        x = 490 + 90 * k
        dr.rectangle(
            [x, 330 - 140 * v / top, x + 60, 330],
            fill=(70, 150, 230) if v > 0 else (40, 44, 52),
        )
        dr.text((x + 10, 336), FEET[k], font=F["s"], fill=TXT)
        dr.text((x, 358), _fmt(v, ".0f", " N"), font=F["sm"], fill=DIM)
    _arrow(dr, (860, 255), (930, 255))
    _box(
        dr,
        F,
        930,
        180,
        300,
        150,
        "Whole-body QP",
        f"{1 / DT:.0f} Hz\nforces + swing accelerations\n-> 12 joint torques, limits",
        ACC,
    )
    _arrow(dr, (1080, 330), (1080, 420))
    _box(dr, F, 930, 420, 300, 80, "12 QDD drives", "abduction, hip, knee x 4 (CAN)")
    dr.text(
        (40, 420),
        f"gait {tap.gait_name(i)}   mode {tap.mode_name(i)}",
        font=F["b"],
        fill=TXT,
    )
    dr.text((40, 456), "contact", font=F["sm"], fill=DIM)
    _lights(dr, F, 40, 480, tap.z["contact"][i])
    return img


def draw_footholds(tap, i, size):
    img, dr, F = _canvas(
        size,
        "Gait and footholds",
        DIAGRAMS[2][2],
        [
            "examples/dog/loco/gait.py",
            "examples/dog/loco/terrain.py",
            "examples/dog/loco/controller.py",
        ],
    )
    cx, cy, s = (
        430,
        410,
        220.0,
    )  # pixels per metre, dog at the centre, world axes (not rotated with the heading)
    dr.rectangle([60, 130, 800, 690], outline=LINE, width=1)
    c, tgt = tap.z["truth_xy"][i], tap.z["foot_xy"][i]
    dr.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], outline=TXT, width=3)
    stance = []
    for k in range(4):
        d = tgt[k] - c
        if not np.all(np.isfinite(d)) or np.hypot(*d) > 1.2 or not np.any(tgt[k]):
            continue
        p = (cx + s * d[0], cy - s * d[1])
        down = tap.z["contact"][i][k] > 0.5
        dr.ellipse(
            [p[0] - 14, p[1] - 14, p[0] + 14, p[1] + 14],
            fill=GOOD if down else None,
            outline=ACC,
            width=3,
        )
        dr.text((p[0] + 18, p[1] - 10), FEET[k], font=F["sm"], fill=TXT)
        if down:
            stance.append(p)
    if len(stance) >= 3:
        cen = np.mean(stance, 0)
        order = sorted(stance, key=lambda q: math.atan2(q[1] - cen[1], q[0] - cen[0]))
        dr.line(order + [order[0]], fill=GOOD, width=2)
    dr.text((840, 150), f"gait: {tap.gait_name(i)}", font=F["t"], fill=TXT)
    dr.text(
        (840, 210),
        "touchdown targets (rings), feet down (filled),",
        font=F["sm"],
        fill=DIM,
    )
    dr.text((840, 232), "support polygon (green).", font=F["sm"], fill=DIM)
    _lights(dr, F, 840, 280, tap.z["contact"][i])
    dr.text((840, 330), f"mode {tap.mode_name(i)}", font=F["m"], fill=TXT)
    dr.text(
        (840, 360),
        f"feet down: {int(np.nansum(tap.z['contact'][i]))} of 4",
        font=F["m"],
        fill=TXT,
    )
    return img


def draw_elevmap(tap, i, size):
    from autonomy import config

    img, dr, F = _canvas(
        size,
        "Elevation map",
        DIAGRAMS[3][2],
        ["examples/dog/autonomy/elevmap.py", "examples/dog/autonomy/lidar.py"],
    )
    cfg = config.load()
    side, m = 560, tap.mapped(i)
    img.paste(_mask_image(tap, i, side), (60, 120))
    x0, cell = tap.map_geometry()
    p = _to_px(tap.z["est_xy"][i], x0, cell, m.shape[0], 60, 120, side)
    dr.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill=ACC)
    dr.text(
        (680, 140),
        f"mapped: {100 * m.mean():.1f} % of the field's cells",
        font=F["b"],
        fill=TXT,
    )
    dr.text(
        (680, 180),
        f"cell {100 * cell:.0f} cm (map {100 * cfg['map']['res']:.0f} cm, pooled {tap.meta['pool']}x)",
        font=F["s"],
        fill=DIM,
    )
    dr.text(
        (680, 210),
        f"mapped = variance below ({100 * np.sqrt(cfg['map']['var_mapped']):.0f} cm)^2",
        font=F["s"],
        fill=DIM,
    )
    dr.text(
        (680, 240),
        f"lidar {cfg['lidar']['rate_hz']:.0f} Hz, {cfg['lidar']['rays_mission']} rays per frame",
        font=F["s"],
        fill=DIM,
    )
    dr.rectangle([680, 300, 704, 324], fill=tuple(int(v) // 2 for v in GOOD))
    dr.text((712, 300), "mapped", font=F["s"], fill=TXT)
    dr.rectangle([680, 336, 704, 360], fill=(30, 34, 42), outline=LINE)
    dr.text((712, 336), "not yet seen", font=F["s"], fill=TXT)
    dr.ellipse([680, 372, 704, 396], fill=ACC)
    dr.text((712, 372), "the dog (estimate)", font=F["s"], fill=TXT)
    dr.text((680, 440), f"t = {tap.t[i]:6.1f} s", font=F["m"], fill=TXT)
    return img


def draw_localizer(tap, i, size):
    img, dr, F = _canvas(
        size,
        "Localization",
        DIAGRAMS[4][2],
        ["examples/dog/autonomy/localize.py", "examples/dog/autonomy/icp.py"],
    )
    _box(dr, F, 40, 140, 300, 90, "Leg odometry", "predict (loco Estimator pose)")
    _box(dr, F, 40, 260, 300, 90, "RTK GNSS", "position fix, sigma from the receiver")
    _box(
        dr,
        F,
        40,
        380,
        300,
        90,
        "Scan-to-map ICP",
        "lidar scan against the elevation map",
    )
    _box(
        dr,
        F,
        450,
        220,
        300,
        190,
        "EKF (x, y, z, yaw)",
        "predict + gated updates\nfixes handed back to the Estimator",
        ACC,
    )
    for y, ty in ((185, 280), (305, 315), (425, 350)):
        _arrow(dr, (340, y), (450, ty))
    rtk, lost, err = tap.col("rtk_ok", i), tap.col("gnss_lost", i), tap.col("err_xy", i)
    yn = lambda v: "n/a" if not np.isfinite(v) else ("yes" if v > 0.5 else "no")
    dr.text(
        (450, 440),
        f"RTK fix available: {yn(rtk)}",
        font=F["b"],
        fill=GOOD if rtk > 0.5 else BAD,
    )
    dr.text((450, 474), f"sky view lost: {yn(lost)}", font=F["s"], fill=TXT)
    dr.text(
        (450, 504),
        f"position error: {_fmt(100 * err, '.1f', ' cm')} (limit 30 cm)",
        font=F["s"],
        fill=GOOD if err < 0.3 else BAD,
    )
    if "err_xy" in tap.tr:  # error history up to now, the 30 cm monitor line in red
        x, y, w, h = 800, 220, 420, 200
        dr.rectangle([x, y, x + w, y + h], outline=LINE, width=1)
        sel = tap.tr["t"] <= tap.t[i]
        t, e = tap.tr["t"][sel], tap.tr["err_xy"][sel]
        top = max(0.4, float(e.max()) if len(e) else 0.4)
        if len(t) > 1:
            dr.line(
                [
                    (x + w * (a - t[0]) / max(t[-1] - t[0], 1e-6), y + h - h * b / top)
                    for a, b in zip(t, e)
                ],
                fill=ACC,
                width=2,
            )
        dr.line(
            [(x, y + h - h * 0.3 / top), (x + w, y + h - h * 0.3 / top)],
            fill=BAD,
            width=1,
        )
        dr.text((x, y - 24), "position error over time", font=F["sm"], fill=DIM)
    return img


def draw_planner(tap, i, size):
    img, dr, F = _canvas(
        size,
        "Coverage planner",
        DIAGRAMS[5][2],
        ["examples/dog/autonomy/planner.py", "examples/dog/autonomy/traverse.py"],
    )
    side, m = 560, tap.mapped(i)
    img.paste(_mask_image(tap, i, side), (60, 120))
    x0, cell = tap.map_geometry()
    pa = tap.path(i)
    px = [_to_px(q, x0, cell, m.shape[0], 60, 120, side) for q in pa]
    if len(px) > 1:
        dr.line(px, fill=GOOD, width=3)
    trail = [
        _to_px(q, x0, cell, m.shape[0], 60, 120, side)
        for q in tap.z["truth_xy"][max(0, i - 400) : i + 1]
    ]
    if len(trail) > 1:
        dr.line(trail, fill=TXT, width=2)
    p = _to_px(tap.z["est_xy"][i], x0, cell, m.shape[0], 60, 120, side)
    dr.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill=ACC)
    dr.text((680, 140), f"planned path: {len(pa)} waypoints", font=F["b"], fill=TXT)
    dr.text(
        (680, 180),
        f"mapped: {100 * m.mean():.1f} % of the cells",
        font=F["s"],
        fill=DIM,
    )
    dr.line([(680, 250), (730, 250)], fill=GOOD, width=3)
    dr.text((740, 238), "planned lane path", font=F["s"], fill=TXT)
    dr.line([(680, 286), (730, 286)], fill=TXT, width=2)
    dr.text((740, 274), "where the dog has been", font=F["s"], fill=TXT)
    return img


def draw_executive(tap, i, size):
    img, dr, F = _canvas(
        size, "Mission executive", DIAGRAMS[6][2], ["examples/dog/autonomy/mission.py"]
    )
    names = phase_names()
    cur = int(tap.z["phase"][i])
    chips = [(v, n) for v, n in sorted(names.items()) if n != "INIT"]
    for k, (v, n) in enumerate(chips):
        x = 60 + 230 * k
        on = v == cur
        dr.rounded_rectangle(
            [x, 220, x + 200, 300],
            radius=14,
            outline=ACC if on else LINE,
            width=4 if on else 2,
            fill=(40, 34, 14) if on else (20, 23, 29),
        )
        dr.text((x + 20, 245), n, font=F["t"], fill=TXT if on else DIM)
        if k + 1 < len(chips):
            _arrow(dr, (x + 204, 260), (x + 228, 260))
    dr.text((60, 350), f"phase now: {names.get(cur, '?')}", font=F["b"], fill=TXT)
    dr.text(
        (60, 390),
        f"t = {tap.t[i]:6.1f} s   mode {tap.mode_name(i)}   gait {tap.gait_name(i)}",
        font=F["m"],
        fill=TXT,
    )
    dr.text(
        (60, 440),
        f"coverage of the reachable area: {_fmt(100 * tap.col('coverage', i), '.1f', ' %')}",
        font=F["b"],
        fill=TXT,
    )
    dr.text(
        (60, 480),
        f"map RMS: {_fmt(100 * tap.col('rms', i), '.1f', ' cm')}   walked: {_fmt(tap.col('distance', i), '.1f', ' m')}",
        font=F["s"],
        fill=DIM,
    )
    return img


def draw(key, tap, i, size):
    """One diagram frame (PIL RGB image of `size`) from tap sample i; every number on it is read from the tap / trace."""
    return globals()[f"draw_{key}"](tap, i, size)


def cmd_diagrams(a):
    import videnc

    tap, out = TapData(a.run_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    meta, n = [], 0
    for key, _, _, secs in DIAGRAMS:
        nf = int(secs * FPS)
        for f in range(
            nf
        ):  # the tap index advances from the run's start to its end during each diagram, so the numbers move
            i = int(round((tap.n - 1) * f / max(nf - 1, 1)))
            videnc.save_frame(out, f"d{n:05d}", np.asarray(draw(key, tap, i, (W, H))))
            meta.append(dict(k=n, key=key, tap=i))
            n += 1
    (out / "diagrams.json").write_text(
        json.dumps(dict(fps=FPS, frames=meta, state_hash=tap.meta["state_hash"]))
    )
    print(f"{n} frames = {n / FPS:.1f} s in {out}")  # ttsr-ignore: py/no-print


# ---- part (c): the sim demo, the run replayed tick-exact with an orbit camera, the map and the path overlaid ------------------------------------
PART_C_MAX_S = 135.0  # plan: 180; the three parts at their caps plus the cards and the 15 s rubble short must fit TOTAL_MAX_S = 480 (TIMELINE below)
SPEED = [
    (0.0, 25.0, 1.0),
    (25.0, None, 4.0),
]  # (video s from, to, sim s per video s): real time while the gait is readable, then 4x
ORBIT_R, ORBIT_EL, ORBIT_DEG_S, LOOK_TAU, MAP_H = 2.2, 22.0, 6.0, 0.25, 35.0


def sim_time(v):
    """Sim time shown at video time v (s) under SPEED."""
    t = 0.0
    for a, b, s in SPEED:
        hi = v if b is None else min(v, b)
        if hi > a:
            t += (hi - a) * s
    return t


def video_time(t):
    """Inverse of sim_time: the video second at which sim time t is shown."""
    for a, b, s in SPEED:
        if b is None or t <= sim_time(b):
            return a + (t - sim_time(a)) / s
    raise AssertionError("SPEED must end with an open segment")


def shot_at(v, total=PART_C_MAX_S):
    """'orbit' or 'map' at video second v: orbit for 0-25 s, then 20 s orbit / 12 s map alternating, the last 20 s of `total` all map."""
    if v < 25.0:
        return "orbit"
    if v >= total - 20.0:
        return "map"
    return "orbit" if (v - 25.0) % 32.0 < 20.0 else "map"


class Director:
    """Sequential camera (frame k after frame k-1): orbit around the dog at ORBIT_R m, azimuth advancing ORBIT_DEG_S deg per VIDEO second, the look-at point a low-pass
    (LOOK_TAU s) of the dog's ground-truth position so a trotting trunk cannot jitter the frame; `kind == "map"` is a high wide shot straight above the dog."""

    def __init__(self, trace):
        self.tr, self.az, self.look = trace, 0.0, None

    def dog(self, t, z=None):
        p = [float(np.interp(t, self.tr["t"], self.tr[k])) for k in ("x", "y")]
        zz = (
            z
            if z is not None
            else (
                float(np.interp(t, self.tr["t"], self.tr["z"]))
                if "z" in self.tr
                else 0.4
            )
        )
        return np.array([p[0], p[1], zz + 0.1])

    def step(self, t, dt, kind="orbit", z=None):
        """-> (camera dict for Sim.render: pos, lookat, fovy; kind). `dt` is the VIDEO time since the last call."""
        target = self.dog(t, z)
        a = 1.0 - math.exp(-dt / LOOK_TAU)
        self.look = (
            target if self.look is None else self.look + a * (target - self.look)
        )
        self.az += math.radians(ORBIT_DEG_S) * dt
        if kind == "map":
            c = self.look
            return dict(
                pos=np.array([c[0], c[1] - 0.5, MAP_H]),
                lookat=np.array([c[0], c[1], 0.0]),
                fovy=50.0,
            ), "map"
        el = math.radians(ORBIT_EL)
        off = ORBIT_R * np.array(
            [math.cos(self.az), math.sin(self.az), 0.0]
        ) + np.array([0.0, 0.0, ORBIT_R * math.tan(el)])
        return dict(pos=self.look + off, lookat=self.look.copy(), fovy=50.0), "orbit"


def project(cam, pt, size=(W, H)):
    """Pixel of a world point in a `Sim.render` camera (vertical fov), None behind the camera (the same pinhole as tools/record_skydio.project)."""
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


def hud(w, tap, i, m, base, speedup=1.0):
    """The frame with the HUD from tap sample i: phase chip and a key-value panel (gait, speed, mode, coverage, map RMS, localization error, elapsed); in `map` shots the
    mapped cells, the planned path (green), the dog's trail (white) and the estimated pose (amber) against the true one (white ring), projected with the frame's own camera `m`."""
    from PIL import Image, ImageDraw

    F = getattr(w, "F", None) or _fonts()
    cam = {
        k: np.array(m[k], float) if k != "fovy" else m[k]
        for k in ("pos", "lookat", "fovy")
    }
    img = base.convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    if m.get("shot") == "map":
        x0, cell = tap.map_geometry()
        mk = tap.mapped(i)
        n = mk.shape[0]
        b = max(
            1, int(round(max(0.5, n * cell / 60.0) / cell))
        )  # draw blocks of >= 0.5 m: at most ~3600 polygons per frame
        nb = n // b
        frac = mk[: nb * b, : nb * b].reshape(nb, b, nb, b).mean(axis=(1, 3))
        for r_, c_ in zip(*np.nonzero(frac > 0.5)):
            xs, ys = x0 + c_ * b * cell, x0 + r_ * b * cell
            q = [
                project(cam, (xs + dx * b * cell, ys + dy * b * cell, 0.02), img.size)
                for dx, dy in ((0, 0), (1, 0), (1, 1), (0, 1))
            ]
            if all(q):
                dr.polygon(q, fill=(110, 220, 120, 70))
        pa = [project(cam, (q[0], q[1], 0.05), img.size) for q in tap.path(i)]
        if sum(1 for q in pa if q) > 1:
            dr.line([q for q in pa if q], fill=(90, 255, 130, 230), width=3)
        tr = [
            project(cam, (q[0], q[1], 0.05), img.size)
            for q in tap.z["truth_xy"][max(0, i - 400) : i + 1]
        ]
        if sum(1 for q in tr if q) > 1:
            dr.line([q for q in tr if q], fill=(255, 255, 255, 200), width=2)
        for xy, col, ring in (
            (tap.z["truth_xy"][i], (255, 255, 255, 255), True),
            (tap.z["est_xy"][i], (255, 196, 60, 255), False),
        ):
            q = project(cam, (xy[0], xy[1], 0.1), img.size)
            if q:
                dr.ellipse(
                    [q[0] - 9, q[1] - 9, q[0] + 9, q[1] + 9], outline=col, width=3
                ) if ring else dr.rectangle(
                    [q[0] - 5, q[1] - 5, q[0] + 5, q[1] + 5], fill=col
                )
    names = phase_names()
    chip = names.get(int(tap.z["phase"][i]), "?")
    bad = chip == "ABORT" or tap.mode_name(i) == "FALLEN"
    dr.rounded_rectangle(
        [14, 14, 14 + 28 + dr.textlength(chip, font=F["b"]), 54],
        radius=8,
        fill=(0, 0, 0, 175),
    )
    dr.text(
        (28, 21),
        chip,
        font=F["b"],
        fill=(255, 100, 90, 255) if bad else (255, 196, 60, 255),
    )
    d_now, d_before = (
        tap.col("distance", i),
        float(np.interp(tap.t[i] - 1.0, tap.tr["t"], tap.tr["distance"]))
        if "distance" in tap.tr
        else float("nan"),
    )
    rows = [
        ("t", f"{tap.t[i]:7.1f} s" + (f"   {speedup:g}x" if speedup != 1 else "")),
        ("gait", f"{tap.gait_name(i)}  {tap.mode_name(i)}"),
        ("speed", _fmt(d_now - d_before, ".2f", " m/s")),
        ("coverage", _fmt(100 * tap.col("coverage", i), ".1f", " %")),
        ("map RMS", _fmt(100 * tap.col("rms", i), ".1f", " cm")),
        ("loc error", _fmt(100 * tap.est_error(i), ".1f", " cm")),
    ]
    x1 = img.size[0] - 14
    dr.rounded_rectangle(
        [x1 - 270, 14, x1, 14 + 30 * len(rows) + 14], radius=8, fill=(0, 0, 0, 175)
    )
    for k, (a_, b_) in enumerate(rows):
        dr.text((x1 - 258, 22 + 30 * k), a_, font=F["h"], fill=(255, 196, 60, 255))
        dr.text((x1 - 168, 18 + 30 * k), b_, font=F["m"], fill=(235, 238, 245, 255))
    return Image.alpha_composite(img, ov).convert("RGB")


def cmd_sim(a):
    import napl
    import videnc
    from screenshots import Replay

    run, out = Path(a.run_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    import csv

    rows = list(csv.DictReader(open(run / "trace.csv")))
    trace = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    native = json.loads((run / "native.json").read_text())
    t_end = native["tick"] * native["dt"]
    napl.render_style("outdoor,shadows")
    r = Replay(run)
    r.dt = native[
        "dt"
    ]  # the kernel step; the trace's last row (a lidar frame) is not necessarily the last tick
    trunk = next(
        (p for p in r.sim.body_positions() if p.endswith("deck_top")), None
    )  # the trunk's prim path, for its z
    d = Director(trace)
    total = min(PART_C_MAX_S, video_time(t_end))
    n = int(total * FPS)
    meta = []
    for k in range(n):
        v = k / FPS
        t = sim_time(v)
        r.to(t)
        z = float(r.sim.query(trunk, "pos")[0][2]) if trunk else None
        cam, kind = d.step(t, 1.0 / FPS, shot_at(v, total), z)
        img, _ = r.sim.render(width=W, height=H, **cam)
        videnc.save_frame(out, f"f{k:05d}", np.asarray(img))
        meta.append(
            dict(
                k=k,
                t=round(t, 3),
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
                run=str(run),
                fps=FPS,
                speed=SPEED,
                replay_hash=h,
                replay_matches_run=ok,
                native_hash=native["state_hash"],
                t_end=t_end,
                frames=meta,
            )
        )
    )
    print(
        f"{n} frames in {out}; replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} native.json"
    )  # ttsr-ignore: py/no-print
    sys.exit(0 if ok else 1)


# ---- encode: cards, the three parts in order, captions -------------------------------------------------------------------------------------------
TOTAL_MAX_S = 480.0
SHORT_MAX_S = 15.0  # the honest rubble clip after the main demo
SIM_CLIPS = [  # (dir under ROOT holding sim/, caption title, field)
    ("showcase", "Showcase mission in ruins (flat floor, 0.4 m/s)", "The 30 m ruins field, seed 7002"),
    ("short_rubble", "The same dog on rubble", "The 30 m rubble field, seed 7001"),
]  # fmt: skip
TIMELINE = [  # (part, label, seconds): the budget of each piece; the encode prints the real one
    ("card", "title", 5.0),
    ("A", "part 1 card + construction + wiring + program (viewer capture)", 2.5 + PART_A_MAX_S - 12.0),
    ("A", "exploded view", 12.0),
    ("B", "part 2 card + seven diagrams from the run's tap", 2.5 + sum(s for *_, s in DIAGRAMS)),
    ("C", "part 3 card + the sim demo + the rubble short", 2.5 + PART_C_MAX_S + SHORT_MAX_S),
    ("result", "Monte Carlo and criteria", 16.0),
    ("result", "what is real and what is a stand-in", 14.0),
]  # fmt: skip
STANDINS = [
    "Real: the parts, the assembly and its wiring, the design-rule check, the physics (MuJoCo-based kernel), the QDD actuator model, the rosette lidar ray cast, the controllers and the autonomy code that run (everything in the diagrams is a file of this repository).",
    "Stand-ins: the lidar scan pattern is the published field of view and rosette, not the proprietary pattern; the footfall contact signal is virtual (a force threshold); the terrain is generated; the exploded view is a render of the built MJCF with the wires not drawn; the programming diagrams are drawn from a tap of the run shown in part 3; the showcase seed was fixed before the run and not searched.",
    "Recon and mapping only: no weapon, launcher or payload release, nothing that targets people.",
]
PROG_CHAPTERS = {
    "build": ("1  Build", ["The recon quadruped in the napl viewer's build workspace (assemblies/dog/dog.toml): the chassis, then one leg as its own document, then the leg used four times, the head, the power module and the compute tray, the camera orbiting."]),
    "wiring": ("2  Wiring", ["Pin-level wiring sheets of the real assembly (power, then the CAN chain of one leg), then the same wires as tubes routed around the parts in 3D: none passes through a part."]),
    "program": ("3  Program", ["The programming workspace on the assembly's files. The programming story itself is part 2 of this video."]),
}  # fmt: skip


def _gtt():
    p = Path("/sys/class/drm/card1/device/mem_info_gtt_used")
    return int(p.read_text()) if p.exists() else -1


def mc_card_lines(missions_json):
    """The Monte Carlo card, every number read from docs/dog/missions.json (generated by Task 5 from the MC output): per-family no-fall k/n with its Wilson 95 % interval, the
    100 x 100 m missions, then each spec criterion as MET / NOT MET / not measured."""
    d = json.loads(Path(missions_json).read_text())
    L = [
        f"{fam:8s} no-fall {s['k_nofall']}/{s['n']}   Wilson 95 % [{s['wilson_nofall'][0]:.2f}, {s['wilson_nofall'][1]:.2f}]"
        + (
            f"   mission success {s['k_success']}/{s['n']}, aborted {s['aborted']} ({s['abort_nofall']} with no truth fall)"
            if "aborted" in s
            else ""
        )
        for fam, s in d["families"].items()
    ]
    for name, ms in d.get("missions", {}).items():
        L.append(
            f"{name:8s} 100 x 100 m: "
            + "; ".join(
                f"seed {m['seed']}: "
                + (
                    "fell"
                    if m.get("fell_ever")
                    else "aborted, no fall"
                    if m.get("phase") == 5
                    else "no fall"
                )
                + f", coverage {100 * m['coverage']:.0f} %"
                for m in ms
            )
        )
    return L + [
        f"{c['verdict']:8s} {c['text']}"
        + (f"  ({c['measured']})" if c["measured"] else "")
        for c in d.get("criteria", [])
    ]


def cmd_encode(a):
    import record_cuas_lite as rc
    import videnc

    root, out = Path(a.root), Path(a.out)
    gtt0 = _gtt()
    enc = videnc.Encoder(out, W, rc.HO, FPS, gpl=a.gpl)
    w = rc.Writer(enc)
    w.card("Recon quadruped: from the parts library to a mapping mission, in simulation", [
        "A 12 kg legged robot: 12 GIM8108-8 quasi-direct-drive actuators, a Livox Mid-360 lidar, a RealSense D435i, RTK GNSS, an IMU and a Jetson Orin Nano, assembled and wired from napl's parts library, walking under a convex MPC and whole-body controller, mapping a field and planning its own coverage path. Everything on screen is the napl code path.",
        "Recon and mapping only. Results, misses included: docs/dog/README.md",
    ], 5.0, "title")  # fmt: skip
    w.card(
        "Part 1  Construction",
        ["Sub-assemblies appear and join, then pin-level wiring."],
        2.5,
        "PART 1",
    )
    _bf = rc._build_frames(w, root, a.speedup, PROG_CHAPTERS)
    xj = root / "explode" / "explode.json"
    if xj.exists():
        ex = json.loads(xj.read_text())
        for fr in ex["frames"]:
            img = videnc.load_frame(root / "explode", f"x{fr['k']:05d}")
            for _ in range(6):  # 2 s of frames shown slowly: 12 s
                w.emit(
                    w.bar(
                        img,
                        "construction",
                        "Exploded view",
                        "The built MJCF with each sub-assembly's topmost body displaced; the parts fly apart and join again. Wires are not drawn in this render.",
                        [f"factor {fr['factor']:.2f}"],
                    )
                )
    w.card(
        "Part 2  Programming",
        [
            "Block diagrams of the software, drawn from the data of the run shown in part 3."
        ],
        2.5,
        "PART 2",
    )
    dj = root / "diagrams" / "diagrams.json"
    sj = root / "showcase" / "sim" / "sim.json"
    if dj.exists():
        dm = json.loads(dj.read_text())
        blurb = {k: (t, b) for k, t, b, _ in DIAGRAMS}
        tap = TapData(json.loads(sj.read_text())["run"]) if sj.exists() else None
        for fr in dm["frames"]:
            t, b = blurb[fr["key"]]
            w.emit(
                w.bar(
                    videnc.load_frame(root / "diagrams", f"d{fr['k']:05d}"),
                    "programming",
                    t,
                    b,
                    [f"run t = {tap.t[fr['tap']]:.1f} s"] if tap else None,
                )
            )
    w.card(
        "Part 3  Simulation",
        [
            "The mission run replayed tick-exact in the napl renderer: orbit camera, then the map with the coverage path. The overlay numbers are the run's own."
        ],
        2.5,
        "PART 3",
    )
    for (
        sub,
        title,
        field,
    ) in SIM_CLIPS:  # the main demo (walks), then the honest short (falls)
        sjc = root / sub / "sim" / "sim.json"
        if not sjc.exists():
            continue
        sm = json.loads(sjc.read_text())
        tap = TapData(sm["run"])
        summ = json.loads((Path(sm["run"]) / "summary.json").read_text())
        end = (
            f"fell at {summ['t']:.0f} s"
            if summ.get("fell_ever")
            else (
                f"aborted at {summ['t']:.0f} s by the controller's fall detector (no ground-truth fall)"
                if summ.get("phase") == 5
                else "no fall"
            )
        )
        what = end + f", {100 * summ['coverage']:.0f} % of the field mapped"
        for fr in sm["frames"]:
            i = int(np.argmin(abs(tap.t - fr["t"])))
            sp = sim_time((fr["k"] + 1) / FPS) - sim_time(fr["k"] / FPS)
            img = hud(
                w,
                tap,
                i,
                fr,
                videnc.load_frame(root / sub / "sim", f"f{fr['k']:05d}"),
                sp * FPS,
            )
            w.emit(
                w.bar(
                    img,
                    "simulation",
                    title,
                    f"{field}; seed fixed before the run, not searched. This run: {what}. Replay hash {sm['replay_hash']} (matches the run: {sm['replay_matches_run']}).",
                    [f"sim t = {fr['t']:.1f} s"],
                )
            )
    mj = Path(a.results)
    w.card(
        "Monte Carlo and the spec's criteria",
        mc_card_lines(mj)
        if mj.exists()
        else [f"{mj} not found: no Monte Carlo numbers in this video"],
        16.0,
        "RESULTS",
        mono=99,
    )
    w.card("What is real and what is a stand-in", STANDINS, 14.0, "STAND-INS")
    enc.close()
    print(
        f"wrote {out}: {w.n} frames = {w.n / FPS:.1f} s, {out.stat().st_size / 1e6:.1f} MB, encoder {enc.backend}; GTT used {gtt0} -> {_gtt()} bytes (limit 4294967296)"
    )  # ttsr-ignore: py/no-print
    print(
        "timeline (output seconds):", ", ".join(f"{t:.0f}s {lab}" for t, lab in w.marks)
    )  # ttsr-ignore: py/no-print


def cmd_png(a):
    """Some frames of a frame dir as PNG, to look at (never committed): a sim dir with a sim.json gets the HUD."""
    import videnc

    d, out = Path(a.frames_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    sj = d / "sim.json"
    sm, tap = (json.loads(sj.read_text()), None) if sj.exists() else (None, None)
    tap = TapData(sm["run"]) if sm else None
    stem = "f" if sm else a.stem
    for i in a.idx:
        img = videnc.load_frame(d, f"{stem}{i:05d}")
        if sm:
            fr = sm["frames"][i]
            img = hud(None, tap, int(np.argmin(abs(tap.t - fr["t"]))), fr, img)
        img.save(out / f"{d.name}_{i:05d}.png")


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("encode")
    s.add_argument(
        "root", help="work dir holding build/ explode/ diagrams/ showcase/sim/"
    )
    s.add_argument("out", help="output .webm (AV1, GStreamer VA) or .mp4")
    s.add_argument("--speedup", type=float, default=1.5)
    s.add_argument(
        "--gpl",
        action="store_true",
        help="allow ffmpeg VAAPI / libx264 (GPL builds possible); the default is the LGPL GStreamer path",
    )
    s.add_argument("--results", default=str(REPO / "docs" / "dog" / "missions.json"))
    s = sub.add_parser("png")
    s.add_argument("frames_dir")
    s.add_argument("out_dir")
    s.add_argument("idx", type=int, nargs="+")
    s.add_argument(
        "--stem",
        default="f",
        help="frame file prefix when the dir has no sim.json (x explode, d diagrams)",
    )
    sub.add_parser("probe")
    s = sub.add_parser("oplog")
    s.add_argument("out")
    s = sub.add_parser("explode")
    s.add_argument("out_dir")
    s = sub.add_parser("diagrams")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s = sub.add_parser("sim")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    {
        "probe": cmd_probe,
        "oplog": cmd_oplog,
        "explode": cmd_explode,
        "diagrams": cmd_diagrams,
        "sim": cmd_sim,
        "encode": cmd_encode,
        "png": cmd_png,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
