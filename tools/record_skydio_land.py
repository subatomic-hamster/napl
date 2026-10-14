"""Recording harness of the Skydio landing end-to-end video (media is never committed: frames and the webm stay on the host). Exact commands: docs/skydio/land_video.md.

Same pipeline as tools/record_skydio.py (which it imports for the HUD helpers), extended to the landing example (examples/skydio-land):
  oplog   OUT.json            the op log of a design built from an EMPTY design in the viewer: every part of assemblies/skydio-land.toml placed (library search, animated drag
                              and drop onto the mate ports, instant drops for the repeated parts), the params / devices applied, the wires drawn pin to pin on the WIRING canvas
                              (flatten.wire), the PROGRAM IDE (find / replace / go to definition / rename / hover / terminal) and the Run slot (BENCH, FULL SIM).
                              Captured by this tool's `build OPLOG OUT --expect assemblies/skydio-land.toml` (record_turret's xvfb viewer capture with the PROGRAM page on
                              examples/skydio-land and a scratch file for the typing / rename demo)
  sim     RUN_DIR OUT_DIR     a `napl program` run of one of the 8 scenarios replayed tick-exact (replay.naplog on a fresh World) with a directed camera, the HUD from the run's own
                              trace.csv / telemetry.jsonl / depth.npz and a time schedule (cruise sped up and labelled, descent and touchdown in real time / slow motion)
  encode  ROOT OUT.webm       ROOT holds build/ (record_turret.py build) and one run dir per scenario (forest, rain25 ... with sim/ from `sim`)
  png     SIM_DIR OUT_DIR N...    some frames with their HUD as PNG, to look at
"""

import argparse
import bisect
import json
import math
import sys
import tomllib
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples" / "skydio"))
sys.path.insert(0, str(REPO / "examples" / "skydio-land"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_skydio as rs
import videnc

W, H, FPS = rs.W, rs.H, rs.FPS
ASM = REPO / "assemblies" / "skydio-land.toml"
PROG = "examples/skydio-land"
SCRATCH = "_scratch.py"  # created in the program root by `record_turret.py build --scratch` before the session, removed after
SCRATCH_TEXT = (
    "def descent_rate(agl_m):\n"
    "    # lidar-governed sink: fast when high, slow near the ground (land_fly.py _ph_descend)\n"
    "    return 0.8 if agl_m > 3.0 else 0.45 if agl_m > 1.0 else 0.30\n"
    "\n"
    "\n"
    "print(descent_rate(2.0))\n"
)

# ---- the stages of the build (instance ids of assemblies/skydio-land.toml) ---------------------------------------------------------------------------
STAGES = [(t, y, ids) for t, y, ids in rs.BUILD_STAGES[:5]] + [
    ("Vision payload: RealSense D435i, Raspberry Pi 5, 5 V regulator",
     "The Skydio-style addition: an Intel RealSense D435i depth camera on the nose (tilted 10 deg up), a Raspberry Pi 5 companion computer stacked over the Pixhawk and a Pololu D24V50F5 regulator tapped from the PDB for the Pi and the camera. Placed at their poses on the plates.",
     ["cam", "pi", "reg"]),
    ("Landing sensors: downward TF-Luna lidar and a ZED-F9P RTK rover",
     "What this copy adds for landing: a Benewake TF-Luna on the underside looking straight down (height above ground and sink rate, 100 Hz, sigma 3 cm + 1 %), and a u-blox ZED-F9P RTK rover board with its patch antenna (centimetre-class position when a base is in range: the moving-baseline base on a ship, a fixed base on the ground pads).",
     ["lidar", "rtk_kit"]),
]  # fmt: skip
SHEETS = list(rs.SHEETS) + [
    ("Landing sensors: TF-Luna power and UART on the Pi 5 GPIO header",
     "Sheet 4, new in this copy. The 5 V regulator feeds the TF-Luna; its TXD / RXD go to the Pi 5 UART2 pins GPIO5 / GPIO4 of the full 40-pin J8 header (UART0 stays the MAVLink link to the Pixhawk). Hover a pin for its name.",
     ["reg", "lidar", "pi"]),
]  # fmt: skip
WIRE_KEYS = (
    "from",
    "to",
    "part",
    "length",
    "conductors",
    "included_in",
    "from_pins",
    "to_pins",
    "name",
    "connectors",
)


NO_AUTO_BOLTS = {
    "top-seat1"
}  # mates the animated drag would bolt ('auto') although the assembly lists no fasteners: placed with design_edit (fasteners None)


def R(tool, wait=0.0, **args):
    return {"tool": tool, "args": args, "wait": wait}


def wire_args(w):
    """flatten.wire arguments of a wire of the assembly. A one-conductor lead between two real pins is drawn pin to pin ('lidar.C.3' -> 'pi.J8.29', the form the
    design script used); the others are drawn port to port with their pin lists."""
    a = {k: w[k] for k in WIRE_KEYS if k in w}
    fp, tp = w.get("from_pins") or [], w.get("to_pins") or []
    if len(fp) == len(tp) == 1:
        a.pop("from_pins", None), a.pop("to_pins", None)
        a["from"], a["to"] = (
            f"{w['from'].partition('.')[0]}.{fp[0]}",
            f"{w['to'].partition('.')[0]}.{tp[0]}",
        )
    return a


def _line(path, pat):
    return rs._line(path, pat)


# ---- doc -> placement plan ---------------------------------------------------------------------------------------------------------------------
def placement_plan(doc):
    """[(stage index, instance, mate-or-None)]: for every on-board instance the mate that places it (the first one in the file whose other end is already placed),
    in stage order; the mates left over are closures (applied after the stage)."""
    insts = {i["id"]: i for i in doc["instances"]}
    stage_of = {x: n for n, st in enumerate(STAGES) for x in st[2]}
    missing = [i for i in insts if not insts[i].get("offboard") and i not in stage_of]
    assert not missing, f"instances in no stage: {missing}"
    placed, plan, used = {"bottom"}, [], set()
    mates = doc["mates"]
    for n, st in enumerate(STAGES):
        todo = [x for x in st[2] if x != "bottom"]
        while todo:
            progressed = False
            for x in list(todo):
                i = insts[x]
                if "pos" in i:  # free-placed at a pose on the plates
                    plan.append((n, x, None))
                    placed.add(x)
                    todo.remove(x)
                    progressed = True
                    continue
                cand = [
                    m
                    for m in mates
                    if m["name"] not in used
                    and not m.get("closure")
                    and m["b"].partition(".")[0] == x
                    and m["a"].partition(".")[0] in placed
                ]
                cand = cand or [
                    m
                    for m in mates
                    if m["name"] not in used
                    and m["b"].partition(".")[0] == x
                    and m["a"].partition(".")[0] in placed
                ]
                if cand:
                    plan.append((n, x, cand[0]))
                    used.add(cand[0]["name"])
                    placed.add(x)
                    todo.remove(x)
                    progressed = True
            assert progressed, (
                f"stage {n}: cannot place {todo} (mates run the other way?)"
            )
    return plan, used


def add_op(doc_inst, m):
    """The design_edit `add` op of one instance placed by mate `m` (a = placed side) or at its pose."""
    op = {"op": "add", "part": doc_inst["part"], "id": doc_inst["id"]}
    if doc_inst.get("group"):
        op["group"] = doc_inst["group"]
    if m is None:
        op["pos"], op["rpy"] = doc_inst["pos"], doc_inst.get("rpy", [0.0, 0.0, 0.0])
        return op
    op["to"], op["port"], op["mate"] = m["a"], m["b"].partition(".")[2], m["name"]
    op["k"] = m.get("rotation", 0)
    op["fasteners"] = m.get(
        "fasteners"
    )  # None: bolt nothing (the default 'auto' would add bolts to the top plate seats, which the assembly does not list)
    if m.get("joint") is not None:
        op["joint"] = m["joint"]
    return op


def cmd_oplog(a):
    import hashlib

    from napl.design import Design
    from napl.design.viewer import Backend

    doc = tomllib.loads(ASM.read_text())
    insts = {i["id"]: i for i in doc["instances"]}
    plan, used = placement_plan(doc)
    closures = [m for m in doc["mates"] if m["name"] not in used]
    steps = []
    pset = lambda x: [
        {"op": "set", "id": x, "param": k_, "value": v}
        for k_, v in (insts[x].get("params") or {}).items()
    ]  # a placed part's parameter overrides, with their provenance

    def S(seg, title, why, *reqs, speed=1.0, dwell=0.5, **kw):
        steps.append(
            dict(
                seg=seg,
                title=title,
                why=why,
                speed=speed,
                dwell=dwell,
                reqs=list(reqs),
                **kw,
            )
        )

    root = insts["bottom"]
    steps.append(dict(
        seg="LIBRARY", title="Start from an empty design: the X500 V2 bottom plate", speed=1.2, dwell=0.5,
        why="Nothing is placed yet. The LIBRARY page lists the real parts; the Holybro X500 V2 bottom carbon plate becomes the root, the frame every other part is mated to.",
        reqs=[R("ui.mode", 0.8, name="library"), R("library.search", 2.4, text="x500v2 bottom", type_ms=55),
              R("library.drop", 1.0, part=root["part"], id="bottom", group=root["group"]), R("ui.mode", 0.6, name="build"), R("ui.camera", 0.4, preset="iso"), R("ui.frame", 1.0, instances=[]),
              R("camera.orbit", 3.0, yaw_deg=120, pitch_deg=8, seconds=2.7, relative=True)]))  # fmt: skip
    steps.append(dict(
        seg="BUILD", title="A free-flying vehicle: 1 ms step, noise seed 3", speed=1.0, dwell=0.4,
        why="A new design is a free-floating vehicle (not bolted down, unlike a turret). The physics step is 1 ms and the noise seed is part of the design, so every flight replays.",
        reqs=[R("design_edit", 1.4, ops=[{"op": "assembly", "base": doc["assembly"]["base"], "timestep": doc["assembly"]["timestep"], "seed": doc["assembly"]["seed"], "description": doc["assembly"]["description"]}])]))  # fmt: skip
    for n, st in enumerate(STAGES):
        seen_part, firsts, rest = set(), [], []
        for sn, x, m in plan:
            if sn != n:
                continue
            (rest if insts[x]["part"] in seen_part else firsts).append((x, m))
            seen_part.add(insts[x]["part"])
        for j, (x, m) in enumerate(firsts):
            op = add_op(insts[x], m)
            title = (
                f"{n + 1}/{len(STAGES)}  {st[0]}"
                if j == 0
                else f"{n + 1}/{len(STAGES)}  {insts[x]['part']} as {x}"
            )
            why = (
                st[1]
                if j == 0
                else f"{insts[x]['part']} ({x}): "
                + (
                    f"snapped onto {m['a']} (mate {m['name']}"
                    + (", spinning joint" if m.get("joint") else "")
                    + ")."
                    if m
                    else "placed at its pose on the plates."
                )
            )
            if m is not None and (m.get("fasteners") or m["name"] not in NO_AUTO_BOLTS):
                steps.append(dict(seg="LIBRARY", macro="place", title=title, why=why, speed=1.3 if j else 1.0, dwell=0.3, search=insts[x]["part"].split("-", 1)[-1][:28], part=insts[x]["part"],
                                  to=m["a"], port=m["b"].partition(".")[2], id=x, group=insts[x].get("group", ""), mate=m["name"], k=m.get("rotation", 0),
                                  **({"fasteners": m["fasteners"]} if m.get("fasteners") else {}), **({"joint": m["joint"]} if m.get("joint") else {}),
                                  post=([R("design_edit", 0.3, ops=pset(x))] if pset(x) else []) + ([R("camera.orbit", 2.2, yaw_deg=40, pitch_deg=0, seconds=2.0, relative=True)] if j == 0 else [])))  # fmt: skip
            else:
                S("LIBRARY", title, why, R("ui.mode", 0.4, name="library"), R("library.search", 0.045 * 22 + 1.2, text=insts[x]["part"].split("-", 1)[-1][:24], type_ms=45),
                  R("design_edit", 1.2, ops=[op, *pset(x)]), R("ui.mode", 0.5, name="build"), R("select", 0.2, instances=[x]), R("ui.frame", 1.0, instances=[]),
                  R("camera.orbit", 2.6, yaw_deg=70, pitch_deg=0, seconds=2.3, relative=True), speed=1.3 if j else 1.0)  # fmt: skip
        if rest:
            S("BUILD", f"{n + 1}/{len(STAGES)}  The same parts again: " + ", ".join(x for x, _ in rest[:12]) + (" ..." if len(rest) > 12 else ""),
              "Repeated parts are dropped the same way (each snaps onto its own mate port with the same bolt pattern, rotation and fasteners as the first); the camera orbits the growing assembly.",
              R("select", 0.2, instances=[]), R("design_edit", 2.0, ops=[o for x, m in rest for o in (add_op(insts[x], m), *pset(x))]), R("ui.frame", 0.8, instances=[]),
              R("camera.orbit", 4.6, yaw_deg=110, pitch_deg=4, seconds=4.3, relative=True), speed=2.0, dwell=0.4)  # fmt: skip
        placed_now = {x for sn, x, _ in plan if sn <= n} | {"bottom"}
        cl = [
            m
            for m in closures
            if m["a"].partition(".")[0] in placed_now
            and m["b"].partition(".")[0] in placed_now
            and not m.get("_done")
        ]
        for m in cl:
            m["_done"] = True
        if cl:
            ops = [{"op": "mate", "a": m["a"], "b": m["b"], "name": m["name"], "k": m.get("rotation", 0), **({"closure": True} if m.get("closure") else {}),
                    "fasteners": m.get("fasteners"), **({"joint": m["joint"]} if m.get("joint") else {})} for m in cl]  # fmt: skip
            S("BUILD", f"{n + 1}/{len(STAGES)}  Close the loops: {len(cl)} more mates", "The arms are one rigid ring: the extra mates close the loop (seat of the connectors on the top plate, tube ends, battery rails) and the bolts are counted.",
              R("design_edit", 2.0, ops=ops), R("ui.frame", 0.6, instances=[]), speed=1.5)  # fmt: skip
    # the chips that have no body, the parameters with provenance, the device bindings
    off = [i for i in doc["instances"] if i.get("offboard")]
    S("BUILD", "The chips that have no body: IMU, barometer, magnetometer, GNSS, RTK, frame kit",
      "The sensor chips on the Pixhawk and the RTK board are data-only instances (no mass, no geometry): their datasheet numbers feed the simulated devices. The frame kit carries the published weight.",
      R("ui.mode", 0.4, name="library"), R("library.search", 2.0, text="icm-42688", type_ms=45), R("library.search", 2.0, text="zed-f9p", type_ms=45),
      R("design_edit", 1.4, ops=[{"op": "add", "part": i["part"], "id": i["id"], "offboard": True} for i in off]), R("ui.mode", 0.4, name="build"), speed=1.6)  # fmt: skip
    S("BUILD", "Where the datasheet is silent, the number says so",
      "Every parameter override carries its fidelity (datasheet with a quote, derived, estimated with the reason): the lidar's 5 g is the datasheet's maximum, the RTK allowance and the lead lengths are estimates. Select the lidar for its PROPERTIES, hover it for its card.",
      R("select", 0.4, instances=["lidar"]), R("ui.hover", 3.5, instance="lidar", seconds=3.0), R("select", 0.2, instances=[]), speed=1.2)  # fmt: skip
    dops = (
        [
            {"op": "device_map", "name": nme, "map": mp}
            for nme, mp in doc["device_maps"].items()
        ]
        + [{"op": "device", "device": d} for d in doc["devices"]]
        + [{"op": "target", **t} for t in doc.get("targets", [])]
    )
    S("BUILD", f"The simulation side: {len(doc['devices'])} device bindings, including the new `lidar` and `rtk`",
      "Each binding maps part parameters onto a kernel device (battery, ESC, rotor, IMU, baro, mag, GNSS, depth camera). New here: `lidar`, a 1 x 1 rangefinder of the TF-Luna's 2 degree field, and `rtk`, the ZED-F9P as the RTK rover input (10 Hz, 2-3 cm).",
      R("design_edit", 2.0, ops=dops), R("design_check", 3.0), speed=1.6)  # fmt: skip
    S("BUILD", "The assembled vehicle", "Mass and the design-rule check for the finished copy; the DRC warnings are inherited from assemblies/x500v2.toml.",
      R("ui.frame", 0.8, instances=[]), R("camera.orbit", 8.5, yaw_deg=300, pitch_deg=12, seconds=8.0, relative=True), R("design_check", 2.0), speed=1.4)  # fmt: skip
    # ---- wiring: draw each wire pin to pin on the canvas ------------------------------------------------------------------------------------------------
    left = list(doc["wires"])
    cpath = Path(a.out).with_suffix(".cards.json")
    sha = hashlib.sha1(ASM.read_bytes() + repr(SHEETS).encode()).hexdigest()
    cached = json.loads(cpath.read_text()) if cpath.exists() else {}
    cached = cached if cached.get("sha") == sha else {"sha": sha}
    b = Backend(lambda d=Design.open(ASM): d, Path("/tmp/napl-oplog-skydio-land"))
    for n, (title, why, inst) in enumerate(SHEETS, 1):
        inst = [i for i in inst if i in insts]
        mine = [
            w
            for w in left
            if w["from"].partition(".")[0] in inst and w["to"].partition(".")[0] in inst
        ]
        left = [w for w in left if w not in mine]
        ports = sorted({w[e] for w in mine for e in ("from", "to")})
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
            r = json.loads(b(json.dumps({"tool": "flatten.layout", "args": {}})))
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
            R("ui.wire_pick", 3.2, instances=inst),
            R("flatten.begin", 3.0, instances=inst, ports=ports),
            R("ui.wire_view", 2.0, fit=True),
        ]
        for w in mine:
            reqs.append(R("flatten.wire", 1.7, **wire_args(w)))
        reqs += [R("flatten.layout", 2.0), R("ui.wire_view", 2.4, fit=True)]
        if (
            n == 1
        ):  # fold / open the right panel; hide parts: their cards and wires vanish in 2D and 3D; show them again
            reqs += [R("ui.wire_view", 2.0, panel="fold"), R("ui.wire_view", 2.0, panel="open"), R("ui.visibility", 3.0, instances=["pm02", "pdb", "esc1", "esc2", "esc3", "esc4"], visible=False),
                     R("ui.mode", 1.0, name="build"), R("ui.view", 0.3, emphasise_wires=True), R("ui.camera", 4.0, preset="iso"), R("ui.visibility", 3.5, visible=True), R("ui.mode", 1.5, name="wiring"),
                     R("ui.wire_view", 1.5, fit=True)]  # fmt: skip
        reqs += [
            R("ui.wire_view", 4.4, zoom=0.9, center=c, seconds=2.4)
            for c in cached[title]
        ]
        reqs += [R("ui.wire_view", 2.4, fit=True, seconds=1.6), R("flatten.end", 1.0)]
        S("WIRING", title, why, *reqs, speed=1.8 if n == 1 else 1.5, dwell=0.6)
    assert not left, f"wires in no sheet: {[w['name'] for w in left]}"
    S("WIRING", f"Back to 3D: the {len(doc['wires'])} wires are tubes routed around the parts",
      "In 3D each conductor is a tube with its gauge's diameter, leaving its connector along the connector axis and routed through free space (a distance field of the assembly), never through a part. The lidar's 26 AWG leads run to the regulator and the Pi header.",
      R("ui.mode", 1.2, name="build"), R("ui.view", 0.5, emphasise_wires=True), R("select", 0.5, instances=[]), R("ui.camera", 1.2, preset="iso"), R("camera.orbit", 8.5, yaw_deg=300, pitch_deg=12, seconds=8.0, relative=True),
      R("camera.orbit", 5.5, target=[0, 0, 0], yaw_deg=60, pitch_deg=-60, seconds=5.0, relative=True), dwell=0.6)  # fmt: skip
    S("BUILD", "Save the design", "The design is saved as assemblies/skydio-land.toml: the next page edits the code that flies it.", R("design_save", 1.0, path="__SAVE__"), speed=1.0)  # fmt: skip
    # ---- program: the IDE, the Run slot --------------------------------------------------------------------------------------------------------------------
    U = lambda wait, **kw: R("prog.ui", wait, **kw)
    C = lambda wait, name: U(wait, cmd="cmd", name=name)
    fl = REPO / PROG / "land_fly.py"
    lf = fl.read_text().splitlines()
    cls = next(i for i, ln in enumerate(lf) if ln.startswith("class SkydioLand"))
    gm = next(i for i, ln in enumerate(lf) if "L.GroundMap()" in ln)
    gmc = lf[gm].index("GroundMap")
    dsc = next(i for i, ln in enumerate(lf) if ln.startswith("    def _ph_descend"))
    bdl = (REPO / PROG / "build_design.py").read_text().splitlines()
    hl = next(
        i for i, ln in enumerate(bdl) if 'd.wire("reg.vout", "lidar.power_in"' in ln
    )
    hc = bdl[hl].index("lidar.power_in") + 8
    P = lambda title, why, *reqs, speed=1.0: steps.append(
        dict(
            seg="PROGRAM", title=title, why=why, speed=speed, dwell=0.5, reqs=list(reqs)
        )
    )
    P("PROGRAM is an IDE: the landing program", "examples/skydio-land/land_fly.py: SkydioLand takes the original's planner and adds the landing phases (scan, survey, settle, descend, touchdown). File explorer, tabs, Python syntax colouring, a problems dock.",
      R("ui.mode", 1.5, name="program"), U(5.0, cmd="open", path="land_fly.py", line=cls, col=6, len=10))  # fmt: skip
    P("The lidar-governed descent", "_ph_descend: sink 0.8 m/s high, 0.45 then 0.30 m/s near the ground, from the lidar's height above the ground.", U(1.0, cmd="goto", line=dsc + 1), U(4.5, cmd="open", path="land_fly.py", line=dsc, col=8, len=11))  # fmt: skip
    P("Go to definition: GroundMap", "F12 on `GroundMap` jumps to land.py: a robust plane fit of the lidar's ground points gives slope and roughness; rises, drop-offs and the depth map's obstacles decide whether a zone is safe.",
      U(1.0, cmd="open", path="land_fly.py", line=gm, col=gmc, len=9), U(1.5, cmd="define"), U(3.5, cmd="hover", line=-1))  # fmt: skip
    P("Find, replace and find in files", "Ctrl+F finds the selected symbol in the open file; Ctrl+H opens replace; Ctrl+Shift+F searches every file of the program.",
      U(1.0, cmd="open", path="land_fly.py", line=dsc, col=8, len=11), C(3.0, "find"), C(2.5, "replace"), C(3.0, "find_in_files"), U(0.5, cmd="dock", name="search"))  # fmt: skip
    P("Hover a pin: the tooltip names the part and the pin", "A pin reference in build_design.py (here the TF-Luna's power input) shows the part's pin tooltip on hover.",
      U(1.5, cmd="open", path="build_design.py", line=hl, col=hc), U(4.5, cmd="hover", line=hl, col=hc), U(0.3, cmd="hover", line=-1))  # fmt: skip
    P("Edit, rename a symbol, and a real shell in the TERMINAL tab", "A scratch file in the program folder: type a line, rename `descent_rate` with F2 (jedi renames every use), then the TERMINAL tab is a pty shell in the program folder.",
      U(1.0, cmd="open", path=SCRATCH, line=2, col=0, len=0), U(3.0, cmd="type", text="    # slower than the real program: a demo\n"), U(1.0, cmd="open", path=SCRATCH, line=0, col=4, len=12), U(3.0, cmd="rename", new="sink_rate"),
      U(0.5, cmd="dock", name="terminal"), U(1.2, cmd="term", text="ls"), U(2.4, cmd="term", text="python _scratch.py"))  # fmt: skip
    P("Run slot, BENCH: the TF-Luna model on the bench", "BENCH runs the selected component's device model on a bench (a 1 x 1 rangefinder: the noise readout at several ranges) plus the pytest files tied to the open file (none for README.md); the OUTPUT tab streams the table and the chip shows PASS.",
      U(0.5, cmd="open", path="README.md", line=0, col=0), U(0.5, cmd="dock", name="output"), C(0.8, "bench"), C(1.0, "target:lidar"), C(BENCH_WAIT, "run"))  # fmt: skip
    P("Run slot, the scenarios: every folder of the combo", "The Run slot lists every *.scenario.toml under the program root; the folders group them: forest, rain, summit, ledge, mesa_brink, deck. Step through them.",
      C(0.8, "full"), *[C(1.2, f"target:{s}") for s in SCENARIOS], speed=1.2)  # fmt: skip
    P("Run slot, FULL SIM: forest/clearing flies live in SIMULATE", "FULL SIM runs the scenario in the window's own simulation, in real time: the page switches to SIMULATE with the LIVE label and a chase camera; the monitors judge the run and the replay hash is re-checked.",
      C(0.8, "full"), C(1.0, f"target:{SCENARIOS[0]}"), C(0.5, "run"), R("ui.mode", FULL_WAIT, name="simulate"), speed=FULL_SPEED)  # fmt: skip
    P("Run slot result: the chip, and the monitors' verdict in OUTPUT", "When the scenario ends the Run slot shows the verdict chip and OUTPUT lists the monitors, the state hash and whether the replay matched.",
      R("ui.mode", 1.0, name="program"), U(6.0, cmd="dock", name="output"))  # fmt: skip
    Path(a.out).write_text(
        json.dumps(
            {
                "about": "generated by tools/record_skydio_land.py oplog from assemblies/skydio-land.toml",
                "steps": steps,
            },
            indent=1,
        )
    )
    print(f"{len(steps)} steps -> {a.out}")


SCENARIOS = [  # (Run slot path under examples/skydio-land)
    "forest/clearing.scenario.toml", "rain/rain25.scenario.toml", "rain/rain100.scenario.toml", "summit/summit.scenario.toml",
    "ledge/ledge.scenario.toml", "mesa_brink/mesa_brink.scenario.toml", "deck/osv.scenario.toml", "deck/asv.scenario.toml",
]  # fmt: skip
BENCH_WAIT = 12.0  # s the bench run is left to finish on screen
FULL_WAIT = 380.0  # s the live forest run is left to fly (wall; 171 sim s take about 230 s with the window rendering, then napl-replay checks the 30 MB log); the step is played fast
FULL_SPEED = 6.0


# ---- the build capture: record_turret's _build with the program root and the scratch file ------------------------------------------------------------
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
    """record_turret's `fn` on an empty design, but the PROGRAM page edits examples/skydio-land (the design has no file yet) and the scratch file exists for the session."""
    import napl.design.viewer as V
    import record_turret as rt

    rt.NAME, rt.DESC = (
        "skydio-land",
        tomllib.loads(ASM.read_text())["assembly"]["description"],
    )  # the new design's name and description (the header the comparison checks)

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
    """No window: the op log through the Backend, the built design compared with --expect (parts, poses, mass, wires, devices)."""
    import record_turret as rt

    _session(rt.cmd_explore, a)


# ---- one run: trace, telemetry, depth, truth world ---------------------------------------------------------------------------------------------------
PHASE_TEXT = {
    "takeoff": "TAKEOFF", "map": "MAPPING HOVER", "fly": "FLY TO THE LANDING ZONE", "scan": "LOOK AROUND: 360 DEG TURN", "survey": "LIDAR SURVEY OF THE ZONE",
    "settle": "SETTLE OVER THE SPOT", "descend": "DESCEND: LIDAR-GOVERNED SINK", "touchdown": "TOUCHDOWN", "landed": "LANDED",
    "follow": "FOLLOW THE MOVING PAD", "wait": "WAIT FOR A QUIET WINDOW", "sink": "SINK ON THE FORECAST DECK HEIGHT",
}  # fmt: skip
RATE = {
    "fly": 3.0,
    "scan": 2.5,
    "survey": 3.0,
    "follow": 4.0,
    "wait": 3.0,
}  # sim seconds per video second above real time (labelled on screen)
SLOW_TD = 4  # slow motion factor around the first gear contact


class LandRun:
    """The files a `napl program` skydio-land run leaves."""

    def __init__(self, d):
        import csv

        import land_world as LW
        from land_fly import PHASES

        self.dir, self.PHASES = Path(d), PHASES
        rows = list(csv.DictReader(open(self.dir / "trace.csv")))
        self.tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        self.tele = [json.loads(ln) for ln in open(self.dir / "telemetry.jsonl")]
        self.tt = [r["t"] for r in self.tele]
        z = np.load(self.dir / "depth.npz")
        self.dt_t, self.depth = z["t"], z["depth"]
        self.land = json.loads((self.dir / "model" / "land.json").read_text())
        self.summary = json.loads((self.dir / "summary.json").read_text())
        self.native = json.loads((self.dir / "native.json").read_text())
        self.result = json.loads((self.dir / "result.json").read_text())
        self.t_end = self.native["tick"] * self.native["dt"]
        self.world = LW.load_world(self.land["truth"])
        self.family = self.land["family"]
        self.deck = self.family.startswith("deck")
        self.zones = self.land["mission"]
        self.events = self._events()

    def at(self, k, t):
        return float(np.interp(t, self.tr["t"], self.tr[k]))

    def pos(self, t):
        return np.array([self.at(k, t) for k in "xyz"])

    def R(self, t):
        j = min(int(np.searchsorted(self.tr["t"], t)), len(self.tr["t"]) - 1)
        return rs.quat_rot([self.tr[k][j] for k in ("qw", "qx", "qy", "qz")])

    def phase(self, t):
        j = min(int(np.searchsorted(self.tr["t"], t)), len(self.tr["t"]) - 1)
        return self.PHASES[int(round(self.tr["phase"][j]))]

    def zone(self, t):
        j = min(int(np.searchsorted(self.tr["t"], t)), len(self.tr["t"]) - 1)
        return int(self.tr["zone"][j])

    def rec(self, t):
        j = bisect.bisect_right(self.tt, t) - 1
        return self.tele[j] if j >= 0 else None

    def ground_z(self, t):
        """Height of the surface under the vehicle (terrain; on a deck the pad's true height from the trace)."""
        if self.deck:
            return self.at("pad_z", t)
        p = self.pos(t)
        return self.world.ground_z(p[0], p[1])

    def _events(self):
        """[(t0, t1, text, bad)]: the moments the picture should say something about, from the run's own records."""
        ev, tr = [], self.tr
        z = tr["zone"]
        s = self.summary
        for j in np.nonzero(np.diff(z) > 0)[0]:
            k = int(z[j])
            asm = next(
                (
                    a
                    for a in s["assessments"]
                    if a["zone"] == self.zones[k]["name"] and not a["ok"]
                ),
                {},
            )
            why = asm.get("reason") or "unsafe"
            if asm.get(
                "reasons"
            ):  # the onboard map's own cell counts, e.g. "rough (obstacle 17, rough 32)"
                why += (
                    " ("
                    + ", ".join(f"{n} {c}" for n, c in asm["reasons"].items())
                    + ")"
                )
            t = float(tr["t"][j + 1])
            ev.append(
                (
                    t,
                    t + 9.0,
                    f"ZONE {self.zones[k]['name'].upper()} REJECTED: {why}. Flying to {self.zones[min(k + 1, len(self.zones) - 1)]['name'].upper()}",
                    True,
                )
            )
        td = s.get("touchdown")
        if td and (tr["td_speed_ms"] > 0).any():
            ts = float(tr["t"][np.argmax(tr["td_speed_ms"] > 0)])
            ev.append(
                (
                    ts,
                    ts + 8.0,
                    f"TOUCHDOWN {td['speed']:.2f} m/s, tilt {td['tilt_deg']:.1f} deg, {td['dist_safe_m']:.2f} m from the zone centre",
                    td["speed"] > 1.0,
                )
            )
        for g in self.land.get("gusts", []):
            ev.append(
                (
                    g["t"],
                    g["t"] + 10.0,
                    f"GUST FRONT: +{g['amp']:.0f} m/s sweeps across the ridge",
                    True,
                )
            )
        self.t_td = next((e[0] for e in ev if e[2].startswith("TOUCHDOWN")), None)
        return ev

    def active(self, t):
        return [e for e in self.events if e[0] <= t < e[1]]


# ---- the time schedule: sped-up cruise (labelled), real time around events, slow motion at first contact ------------------------------------------------
def schedule(run):
    """[(sim time, rate)] one entry per output frame: rate = sim seconds per video second."""
    out, t = [], 0.0
    while t < run.t_end - 1e-9:
        r = RATE.get(run.phase(t), 1.0)
        if any(e[0] - 2.5 <= t < e[1] for e in run.events):
            r = min(r, 1.0)
        if run.t_td is not None and run.t_td - 0.75 <= t < run.t_td + 1.5:
            r = 1.0 / SLOW_TD
        out.append((t, r))
        t += r / FPS
    return out


# ---- the director: camera moves ------------------------------------------------------------------------------------------------------------------------
class LandDirector:
    """Sequential like record_skydio.Director: eased following point, positions chosen from candidate rings that are above the ground, outside the obstacles and
    with a clear line to the vehicle (scene geometry used only to place the camera)."""

    def __init__(self, run):
        self.run, self.cur, self.t_sw, self.t0 = run, None, 0.0, 0.0
        self.ps = self.h = self.cpos = self.clook = None
        self.sel, self.bad_since, self.tri, self.gz = 0, None, None, 0.0

    def shot(self, t):
        r, ph = self.run, self.run.phase(t)
        if any(e[2].startswith(("ZONE", "GUST")) for e in r.active(t)):
            return ("orbit", dict(r=7.0, up=4.0, w=0.3))
        if ph in ("takeoff", "map"):
            return (
                "orbit",
                dict(r=7.0 if r.deck else 3.5, up=2.5 if r.deck else 1.3, w=0.5),
            )
        if ph == "fly":
            return ("chase", {}) if int(t / 14.0) % 2 == 0 else ("side", {})
        if ph in ("scan", "survey"):
            return ("high", {})
        if ph in ("follow", "wait"):
            return ("orbit", dict(r=9.0, up=4.5, w=0.18))
        if ph in ("settle", "descend", "sink"):
            return ("orbit", dict(r=3.4 if not r.deck else 5.0, up=1.6, w=-0.45))
        if ph == "touchdown":
            return ("tripod", {})
        return ("orbit", dict(r=5.5, up=2.5, w=0.4))  # landed

    def _ok(self, pos, q):
        w, run = self.run.world, self.run
        if pos[2] < self.gz + 0.9 or (run.deck and pos[2] < 1.2):
            return False
        if np.linalg.norm(pos - q) < 1.6 or (
            not run.deck and w.clearance(pos, 3.0) < 0.6
        ):
            return False
        for s in np.linspace(0.15, 0.95, 8):
            p = pos + (q - pos) * s
            if not run.deck and (
                p[2] < w.ground_z(p[0], p[1]) + 0.15 or w.clearance(p, 1.0) < 0.05
            ):
                return False
        return True

    def _cands(self, kind, kw, t, p, h, side, shot_t0):
        ps, z = self.ps, np.array([0.0, 0.0, 1.0])
        if kind == "orbit":
            a0 = 2.6 + kw["w"] * (t - shot_t0)
            return [
                ps
                + [kw["r"] * math.cos(a0 + da), kw["r"] * math.sin(a0 + da), kw["up"]]
                for da in (0, 0.8, -0.8, 1.6, -1.6, math.pi)
            ], p + [0, 0, 0.1]
        if kind == "chase":
            return [
                ps - 3.4 * h + 0.7 * side + 1.2 * z,
                ps - 3.4 * h - 0.7 * side + 1.2 * z,
                ps - 4.6 * h + 2.2 * z,
                ps - 6.0 * h + 3.6 * z,
                ps - 9.0 * h + 6.0 * z,
            ], p + 2.5 * h
        if kind == "side":
            return [
                ps + lat * sg * side - 0.5 * h + up * z
                for lat, up in ((4.0, 0.8), (3.0, 0.6), (5.5, 2.0), (8.0, 3.5))
                for sg in (1.0, -1.0)
            ], p + 1.0 * h
        if kind == "high":
            return [
                ps - bk * h + up * z
                for bk, up in ((6.0, 5.0), (8.0, 7.0), (4.0, 7.0), (10.0, 10.0))
            ] + [ps + 7.0 * side + 6.0 * z], p
        raise ValueError(kind)

    def step(self, t, dt):
        run = self.run
        p, R = run.pos(t), run.R(t)
        v = np.array([run.at(k, t) - run.at(k, t - 0.2) for k in "xyz"]) / 0.2
        fwd = R @ [1.0, 0, 0]
        hn = (
            v[:2] / np.linalg.norm(v[:2])
            if np.linalg.norm(v[:2]) > 0.6
            else fwd[:2] / max(np.linalg.norm(fwd[:2]), 1e-6)
        )
        hn = np.array([hn[0], hn[1], 0.0])
        if self.ps is None:
            self.ps, self.h = p.copy(), hn
        self.ps = self.ps + (p - self.ps) * (1 - math.exp(-dt / 0.45))
        self.h = self.h + (hn - self.h) * (1 - math.exp(-dt / 0.8))
        self.h = self.h / max(np.linalg.norm(self.h), 1e-6)
        side = np.array([-self.h[1], self.h[0], 0.0])
        self.gz = run.ground_z(t)
        kind, kw = self.shot(t)
        key = (
            kind,
            tuple(sorted(kw.items())),
            int(t / 14.0) if kind in ("chase", "side") else 0,
        )
        if key != self.cur:
            self.cur, self.t_sw, self.t0, self.sel, self.bad_since = key, t, t, 0, None
            if kind == "tripod":
                self.tri = self._tripod(p)
        q = p + [0, 0, 0.05]
        if kind == "tripod":
            cands, look = [self.tri], p
        else:
            cands, look = self._cands(kind, kw, t, p, self.h, side, self.t0)
            cands = [np.asarray(c, float) for c in cands]
            if self._ok(cands[self.sel], q):
                self.bad_since = None
            else:
                self.bad_since = t if self.bad_since is None else self.bad_since
                if t - self.bad_since >= 0.12:
                    good = [j for j, c in enumerate(cands) if self._ok(c, q)]
                    self.sel, self.bad_since = (good[0] if good else self.sel), None
        pos, look = np.asarray(cands[self.sel], float).copy(), np.asarray(look, float)
        if self.cpos is None:
            self.cpos, self.clook = pos.copy(), look.copy()
        tau = 0.9 if (t - self.t_sw < 1.8 or kind == "tripod") else 0.35
        self.cpos = self.cpos + (pos - self.cpos) * (1 - math.exp(-dt / tau))
        self.clook = self.clook + (look - self.clook) * (1 - math.exp(-dt / 0.25))
        cam = self.cpos.copy()
        cam[2] = max(cam[2], self.gz + 0.7, 1.0 if run.deck else -1e9)
        return dict(
            pos=cam,
            lookat=self.clook.copy(),
            fovy={"tripod": 34.0, "side": 42.0, "high": 46.0}.get(kind, 50.0),
        ), kind

    def _tripod(self, p):
        """A fixed camera low beside the touchdown point, looking at the gear (on a deck: a little further and above the sea)."""
        run = self.run
        gz = run.ground_z(run.t_td if run.t_td is not None else 0.0)
        for r in (2.4, 3.2, 4.2) if not run.deck else (3.5, 4.5):
            for ang in np.arange(0, 2 * math.pi, math.radians(20)):
                c = p + [r * math.cos(ang), r * math.sin(ang), 0.0]
                c[2] = gz + (0.45 if not run.deck else 0.9)
                self.gz = gz
                if run.deck or self._ok(c, p + [0, 0, 0.1]):
                    return c
        return p + [3.0, 0.0, gz + 1.0 - p[2]]


def fpv_cam(run, t):
    p, R = run.pos(t), run.R(t)
    m = math.radians(
        10.0
    )  # the D435i is tilted 10 deg up in assemblies/skydio-land.toml
    axis = R @ [math.cos(m), 0.0, math.sin(m)]
    return dict(
        pos=p + 0.1 * (R @ [1.0, 0, 0]) + [0, 0, 0.05], lookat=p + 5 * axis, fovy=58.0
    )


# ---- sim: replay + offscreen render --------------------------------------------------------------------------------------------------------------------
def cmd_sim(a):
    import napl
    from napl import _napl  # noqa: F401  (renderer init)
    from screenshots import Replay

    run, out = LandRun(a.run_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    napl.render_style("outdoor,shadows")
    r = Replay(a.run_dir)
    r.dt = 0.001
    d = LandDirector(run)
    times = schedule(run)
    meta = []
    for k, (t, rate) in enumerate(times):
        tick = r.to(t)
        cam, kind = d.step(t, rate / FPS)
        img, _ = r.sim.render(width=W, height=H, **cam)
        videnc.save_frame(out, f"f{k:05d}", np.asarray(img))
        img2, _ = r.sim.render(width=rs.FPV[0], height=rs.FPV[1], **fpv_cam(run, t))
        videnc.save_frame(out, f"p{k:05d}", np.asarray(img2))
        meta.append(
            dict(
                k=k,
                t=round(t, 3),
                tick=int(tick),
                shot=kind,
                rate=rate,
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
                family=run.family,
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


# ---- encode: HUD, cards, captions -----------------------------------------------------------------------------------------------------------------------
def rho_ratio(h):
    return (1 - 2.25577e-5 * h) ** 4.25588


def env_lines(run):
    L = run.land
    out = []
    if run.deck:
        p = L["params"]
        out.append(
            f"SEA Hs {p['hs_m']:.2f} m, vessel {p.get('ship_speed_ms', 0):.0f} m/s"
            if run.family == "deck_osv"
            else f"SEA Hs {p['hs_m']:.2f} m, 1.2 m catamaran"
        )
    if L["origin_alt"] > 0:
        out.append(
            f"SITE {L['origin_alt']:.0f} m ASL, air {100 * rho_ratio(L['origin_alt']):.0f} % of sea level"
        )
    if L["rain_mm_h"] > 0:
        out.append(f"RAIN {L['rain_mm_h']:.0f} mm/h")
    out.append(
        f"WIND {math.hypot(*L['wind'][:2]):.1f} m/s (10 m), turbulence W20 {L['params'].get('turbulence_ms', 0):.1f} m/s"
    )
    return out


def hud(w, run, m, base, fpv, sim_t):
    """The frame with the HUD: phase chip, environment + kv panel, event banner, the plan / trail projected, the camera's colour view and depth image."""
    from PIL import Image, ImageDraw

    cam = {
        k: np.array(m[k], float) if k != "fovy" else m[k]
        for k in ("pos", "lookat", "fovy")
    }
    img = base.convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    F = w.F
    tele, ph = run.rec(sim_t), run.phase(sim_t)
    ts = np.arange(max(0.0, sim_t - 6.0), sim_t + 1e-9, 0.25)
    pts = [rs.project(cam, run.pos(t)) for t in ts]
    for i in range(len(pts) - 1):
        if pts[i] and pts[i + 1]:
            dr.line(
                [pts[i], pts[i + 1]],
                fill=(255, 255, 255, int(40 + 150 * i / len(pts))),
                width=2,
            )
    if ph == "fly" and tele and tele.get("chosen"):
        pl = [x for x in (rs.project(cam, c) for c in tele["chosen"]) if x]
        if len(pl) > 1:
            dr.line(pl, fill=(90, 255, 130, 230), width=3)
    zi = run.zone(sim_t)
    zn = run.zones[min(zi, len(run.zones) - 1)]
    if run.deck:
        pad = np.array([run.at(k, sim_t) for k in ("pad_x", "pad_y", "pad_z")])
        gp, name = [rs.project(cam, pad + [0, 0, z]) for z in (0.0, 3.0)], "PAD"
    else:
        gz = run.world.ground_z(zn["x"], zn["y"])
        gp, name = (
            [rs.project(cam, (zn["x"], zn["y"], gz + z)) for z in (0.0, 3.0)],
            f"ZONE {zn['name'].upper()}",
        )
    if gp[0] and gp[1]:
        dr.line(gp, fill=(255, 200, 60, 230), width=3)
        dr.text(
            (gp[1][0] + 8, gp[1][1] - 8), name, font=F["m"], fill=(255, 210, 80, 255)
        )
    mode = PHASE_TEXT.get(ph, ph.upper())
    dr.rounded_rectangle(
        [14, 14, 14 + 28 + dr.textlength(mode, font=F["b"]), 54],
        radius=8,
        fill=(0, 0, 0, 175),
    )
    dr.text((28, 21), mode, font=F["b"], fill=(255, 255, 255, 255))
    y = 62
    if m["rate"] != 1.0:
        lab = (
            f"SLOW MOTION 1/{round(1 / m['rate'])}: first gear contact"
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
    for e in run.active(sim_t):
        for sub in w.wrap(e[2], F["m"], 640, dr):
            tw = dr.textlength(sub, font=F["m"])
            dr.rounded_rectangle(
                [14, y, 14 + 24 + tw, y + 28],
                radius=8,
                fill=(90, 0, 0, 200) if e[3] else (0, 70, 20, 200),
            )
            dr.text((26, y + 4), sub, font=F["m"], fill=(255, 255, 255, 255))
            y += 32
    rows = [("SPEED", f"{run.at('speed', sim_t):5.1f} m/s"), ("HEIGHT ABOVE DECK" if run.deck else "HEIGHT ABOVE GROUND", f"{max(run.at('agl_m', sim_t), 0.0):5.1f} m"),
            ("TILT", f"{run.at('tilt_deg', sim_t):5.1f} deg"), ("ESTIMATE ERROR", f"{run.at('est_err_m', sim_t):5.2f} m"),
            ("PAD" if run.deck else "ZONE", "deck" if run.deck else f"{zi + 1} of {len(run.zones)}: {zn['name']}")]  # fmt: skip
    if run.deck:
        rows.append(
            (
                "PAD HEAVE (truth)",
                f"{run.at('pad_z', sim_t) - run.land['truth']['start'][2]:+5.2f} m",
            )
        )
    rows += [(s, "") for s in env_lines(run)]
    x1, y0, wid = W - 14, 14, 420
    dr.rounded_rectangle(
        [x1 - wid, y0, x1, y0 + 14 + 24 * len(rows)], radius=8, fill=(0, 0, 0, 175)
    )
    for i, (k, v) in enumerate(rows):
        if v == "":
            dr.text(
                (x1 - wid + 12, y0 + 8 + 24 * i),
                k,
                font=F["sm"],
                fill=(150, 205, 255, 255),
            )
            continue
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
    j = min(int(np.searchsorted(run.dt_t, sim_t)), len(run.dt_t) - 1)
    dep = Image.fromarray(rs.depth_image(run.depth[j])).convert("RGBA")
    fp = fpv.convert("RGBA").resize(rs.FPV)
    yy = H - 14 - rs.FPV[1]
    for x, im, lab in (
        (14, fp, "D435i VIEW (render)"),
        (14 + rs.FPV[0] + 10, dep, "DEPTH 160x90 (red near, blue 10 m)"),
    ):
        img.paste(im, (x, yy))
        d2 = ImageDraw.Draw(img, "RGBA")
        d2.rectangle(
            [x - 1, yy - 1, x + rs.FPV[0], yy + rs.FPV[1]], outline=(255, 255, 255, 120)
        )
        d2.rectangle(
            [x, yy - 22, x + d2.textlength(lab, font=F["sm"]) + 10, yy - 2],
            fill=(0, 0, 0, 190),
        )
        d2.text((x + 5, yy - 20), lab, font=F["sm"], fill=(255, 255, 255, 255))
    return img.convert("RGB"), mode


def verdict(w, run, pic):
    """The last frame with the monitors' verdict panel."""
    from PIL import ImageDraw

    img = pic.convert("RGBA")
    ov = w.Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    res = run.result
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


SCN = [  # (dir, title, blurb)
    ("forest", "Forest/clearing: fly 58 m through the forest, land in another clearing",
     "Seed 1. Take-off, 58 m through trees, a wall doorway and an arch on the depth camera alone, then a lidar survey of the designated clearing. When a seeded hazard (boulders, a log) is found the zone is rejected and the alternate is flown to. No RTK under the canopy: VIO + baro + IMU only."),
    ("rain25", "Rain 25 mm/h: the same forest landing in rain",
     "The kernel's rain device: a little rotor thrust loss, water film and drag, depth and lidar dropouts and false near returns that grow with the rain rate, overcast and streaks in the renderer. The depth image is despeckled before the planner and the lidar ground points are median-gated."),
    ("rain100", "Rain 100 mm/h: a violent downpour",
     "Four times the rain: about 64 % of the depth pixels are valid and 3-4 % are false near returns (kernel estimates). The README: one of the 12 held-out flights rejected the designated clearing and the alternate and landed at home (the roughness limit is marginal in this rain)."),
    ("summit", "Summit: a 3200 m ridge crest, thin air, ridge wind and a gust front",
     "Generated mountain terrain, air at 73 % of sea-level density (thrust margin 2.6 down to about 1.9), terrain-aware wind with ridge updraft and a 5 m/s gust front at t = 100 s. RTK is available. The designated pad holds a hazard in about 35 % of seeds."),
    ("ledge", "Ledge: a pad cut into the steep lee face at 3000 m",
     "Lee turbulence and a downdraft on the lee side of the ridge, the terrain-following altitude band, and an alternate ledge when the first holds boulders."),
    ("mesa_brink", "Mesa brink: a plateau pad within reach of the cliff edge",
     "The designated pad is within reach of the cliff in about 60 % of seeds. The lidar's missing returns beyond the edge (a drop-off) reject it and the alternate 40 m inland is flown to."),
    ("osv", "Offshore support vessel under way: landing on a heaving deck",
     "The 12.6 m OSV makes 3 m/s in head seas, Hs 0.5 m (JONSWAP). The lander follows the pad with moving-base RTK relative to the vessel's base antenna, the vessel's motion unit over a lossy radio, the downward lidar, and a forecast of the pad height; it waits for a quiet window, then sinks on the forecast and is pressed onto the deck."),
    ("asv", "The 1.2 m ASV catamaran in calm water: a 0.75 x 0.90 m plate",
     "A much smaller deck: Hs 0.15 m, a 0.10 m margin for the gear. The README: 6 of 8 held-out flights pass; the failures topple or slide off the plate after touching down."),
]  # fmt: skip

CHAPTERS = {
    "build": ("1  Build", ["From an EMPTY design in the napl viewer: the X500 V2 airframe, propulsion, power and avionics, the vision payload, then what this copy adds for landing: a downward TF-Luna lidar and a ZED-F9P RTK rover. Library search, animated drag and drop onto the mate ports, parameters with provenance, device bindings."]),
    "wiring": ("2  Wiring", ["The wire picker and the pin-level canvas: tick components, flatten, and draw each wire pin to pin on the full headers (the Pi 5's 40-pin J8, the Pixhawk ports, the XT30 leads). Then the same wires as tubes routed around the parts in 3D. New sheet 4: the TF-Luna's power and UART on the Pi's GPIO header."]),
    "program": ("3  Program", ["The PROGRAM page is an IDE for the landing code: syntax colouring, go to definition, find / replace, find in files, rename, pin tooltips, a pty terminal. The Run slot benches a component (BENCH: the TF-Luna model) and flies a scenario live in SIMULATE (FULL SIM) with the monitors' verdict."]),
}  # fmt: skip


def _sim_frames(w, d, title, blurb, hold_s=6.0):
    run = LandRun(d)
    sim = json.loads((Path(d) / "sim" / "sim.json").read_text())
    w.card(title, [blurb], 5.5, title)
    last = None
    for m in sim["frames"]:
        t = m["t"]
        base = videnc.load_frame(Path(d) / "sim", f"f{m['k']:05d}")
        fpv = videnc.load_frame(Path(d) / "sim", f"p{m['k']:05d}")
        last, _ = hud(w, run, m, base, fpv, t)
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
    w.card("Skydio-style quadcopter that lands: from an empty design to eight landings in forest, rain, mountains and on ships", [
        "The Holybro X500 V2 with an Intel RealSense D435i, a Raspberry Pi 5, a downward TF-Luna lidar and a ZED-F9P RTK rover, built from the napl parts library in the viewer, wired pin by pin, programmed in the PROGRAM IDE, and flown in simulation from take-off to a landing somewhere else.",
        "Eight scenarios: a forest clearing, 25 and 100 mm/h rain, a 3200 m summit with a gust front, a lee-side ledge, a mesa brink, a 12.6 m support vessel under way and a 1.2 m catamaran. Every flight is a napl program run with monitors and a replay hash; the picture is a replay of exactly that log. Details and limits: examples/skydio-land/README.md",
    ], 8.0, "title")  # fmt: skip
    rc._build_frames(w, root, a.speedup, CHAPTERS)
    w.card("The landing pipeline", [
        "depth image + IMU + VIO + baro  ->  ESKF  ->  voxel map  ->  sampled-primitive planner (the original Skydio's)",
        "arrive: 360 degree look-around, lidar survey of the zone at 3 m  ->  slope, roughness, rises, drop-offs, obstacles  ->  safe, or reject and go to the alternate",
        "descend on the lidar's height: 0.8, 0.45, 0.30 m/s; touchdown monitors: vertical speed, tilt, distance from the pad, only the gear touches",
        "decks: moving-base RTK + the vessel's motion unit + a forecast of the pad height; wait for a quiet window; sink on the forecast; press into the deck",
    ], 8.0, "PIPELINE", mono=4)  # fmt: skip
    w.card("4  Simulate", [
        "Each scenario is the napl program (examples/skydio-land/*/*.scenario.toml): the kernel steps the world, the autonomy flies, the runner judges the trace and re-runs the replay log to check the state hash. The picture replays exactly that log with a directed camera; cruise legs are sped up and labelled, descent and touchdown run in real time, the first gear contact in slow motion."
    ], 5.0, "SIMULATE")  # fmt: skip
    runs = []
    for sub, title, blurb in SCN:
        d = root / sub
        if (d / "sim" / "sim.json").exists():
            runs.append((sub, *_sim_frames(w, d, title, blurb)))
    lines = []
    for sub, run, sim in runs:
        sm, res = run.summary, run.result
        td = sm.get("touchdown") or {}
        ok = "PASS" if res["pass"] else "FAIL"
        if not res["replay"]["match"]:
            ok += " (replay DIFFERS)"
        lines.append(
            f"{sub:10s} seed {sm['seed']:<3d} {ok}  {sm['sim_s']:5.1f} s  touchdown {td.get('speed', 0):.2f} m/s {td.get('tilt_deg', 0):4.1f} deg  zones tried {len(sm['zones_tried'])}  {run.native['state_hash']}"
        )
    w.card("Result of the eight runs shown", lines + ["Each hash is the state hash at the end of the run; the replay behind the frames above ends on exactly that hash."], 12.0, "RUNS", mono=len(lines))  # fmt: skip
    w.card("Monte Carlo over seeds (examples/skydio-land/README.md, not rerun for this video)", [
        "held-out seeds, pass = landed, no obstacle contact, only the gear touches, touchdown <= 1.0 m/s, tilt <= 12 deg, on a truth-safe spot; Wilson 95 % interval",
        "forest/clearing 12/12 (0.76-1.00)   rain 25 mm/h 12/12   rain 100 mm/h 12/12   summit 12/12   ledge 12/12   mesa_brink 12/12",
        "deck/osv Hs 0.5 m, 3 m/s: 8/8 (0.68-1.00); Hs 1.0 m: 5 of 6, touchdown speed marginal; Hs 1.5 m: 4 of 6",
        "deck/asv Hs 0.15 m: 6/8 (0.41-0.93); Hs 0.3 m: 3 of 6; Hs 0.5 m: 0 of 6 (the lander topples or its arms strike the hull)",
        "A 100 % rate over 12 flights bounds the true rate only to >= 0.76: reliable in these environments at these seeds, not never failing. Rain 150 mm/h: every zone rejected as rough, the lander falls back to home (safe, 240 s).",
    ], 18.0, "MC")  # fmt: skip
    w.card("What is real and what is a stand-in", [
        "Real: the parts, assembly, wiring and DRC; the kernel's physics, rotor and battery models, the rain device, the terrain and wind field, the JONSWAP sea and the vessel's hull; the lidar and depth ray casts; the ESKF, voxel map, planner, landing-zone assessment and deck predictor code that flies.",
        "Stand-ins: the VIO is a statistical model; RTK, the vessel's radio link and the ASV's base antenna are modelled devices (the ASV has no RTK base or motion-unit radio in its design: assumed); the depth camera does not see the terrain (the lidar does); the forest is generated primitives; rain coefficients are the kernel's estimates; the colour inset is a render the autonomy never sees.",
        "Limits: the lander on the ASV plate needs a net or a clamp (not modelled); Hs above 1 m on the OSV is marginal; gear tolerances beyond the monitors are not characterised. On screen: the pad truth and the touchdown distance are scoring information the autonomy does not have.",
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
        run = LandRun(Path(a.frames_dir).parent)
        fr = json.loads(sj.read_text())["frames"]
        for i in a.idx:
            pic, _ = hud(
                w,
                run,
                fr[i],
                videnc.load_frame(a.frames_dir, f"f{i:05d}"),
                videnc.load_frame(a.frames_dir, f"p{i:05d}"),
                fr[i]["t"],
            )
            pic.save(out / f"{run.family}_hud_{i:05d}.png")
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
