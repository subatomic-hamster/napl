"""Recording harness of the recon-quadruped video (media is never committed: frames and the webm stay on the host). Part (a), construction, is here;
parts (b) programming overview and (c) sim demo come with SP6 Tasks 7-9.

Reuses the existing pipelines instead of copying them:
  build / wiring / program  tools/record_turret.py's xvfb viewer capture (`build OPLOG OUT --open assemblies/dog/dog.toml`); this tool only generates the op log
  explode                   the built MJCF with each sub-assembly's topmost body displaced, rendered offscreen with an orbit camera (no viewer op for it exists)

    probe   [DIR]          read-only: MJCF group structure of dogsim.build + the wiring sheets' card preview (what explode_xml / cmd_oplog rely on)
    oplog   OUT.json       construction + wiring + program op log generated from assemblies/dog/dog.toml (needs the Design: run it on the host)
    explode OUT_DIR        frames x*.rgz + explode.json (factor and camera per frame). GPU
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
            reqs += [R("sub.close", 0.8, save=False)]
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
    # The PROGRAM page edits the design's folder: for assemblies/dog/dog.toml that is assemblies/dog (progops.Prog.root; record_turret has no --program-root, so
    # examples/dog/autonomy/planner.py cannot be opened). The IDE is shown on the files that folder holds; the programming story is part (b).
    prog = [
        ("assemblies/dog/README.md", r"^## Sub-assembly tree"),
        ("assemblies/dog/dog.toml", r"^\[\[subassemblies\]\]"),
    ]
    for k, (path, pat) in enumerate(prog):
        steps.append(dict(seg="PROGRAM", title="PROGRAM is an IDE: " + Path(path).name, speed=1.0, dwell=0.5,
                          why=f"{path} in the PROGRAM page: file explorer, tabs, syntax colouring. The programming story itself is part (b) of the video.",
                          reqs=([R("ui.mode", 1.5, name="program")] if k == 0 else []) + [U(8.0, cmd="open", path=Path(path).name, line=_line(path, pat), col=0, len=0)]))  # fmt: skip
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


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("probe")
    s = sub.add_parser("oplog")
    s.add_argument("out")
    s = sub.add_parser("explode")
    s.add_argument("out_dir")
    a = ap.parse_args(argv)
    {"probe": cmd_probe, "oplog": cmd_oplog, "explode": cmd_explode}[a.cmd](a)


if __name__ == "__main__":
    main()
