"""Recording harness for docs/showcase/car_from_scratch.mp4: a design grown edit by edit in the viewer's Design window, then the finished car driven.
One entry point, four subcommands (the video itself is made in a THROWAWAY venv that has `av` + Pillow; neither is a project dependency):

    oplog   SESSION.jsonl OUT.json        turn an exploration session's request log into the committed op log (adds nothing, reorders nothing)
    build   OPLOG.json OUT_DIR            Part 1. xvfb + the viewer on an EMPTY design; every request of the op log is applied through
                                          napl.design.viewer.Backend (the object the window itself talks to), the window reloads, the screen is grabbed
                                          at ~6 fps -> OUT_DIR/f*.png, OUT_DIR/timeline.json (per request: wall times, op, key result facts, the agent's reason).
                                          Verifies that the replayed log reproduces the same accept / refuse answers and the same final design as
                                          --expect (assemblies/car4.toml). GPU + xvfb.
    drive   RUN_DIR OUT_DIR               Part 2. RUN_DIR = a `napl program` run dir (replay.naplog, model/, result.json); the log is replayed into a
                                          fresh World (tools/screenshots.py Replay: same state hash, checked) and rendered offscreen with a follow camera
                                          + an overhead inset -> OUT_DIR/f*.png, hud.json (sim time, speed, wheel currents, pose). GPU.
    encode  PART1_DIR PART2_DIR OUT.mp4   needs `av` + Pillow (run it with the throwaway venv's python): caption strip + DRC / mass banner on Part 1,
                                          HUD on Part 2, title cards, H.264 (else VP9) 1280x720.

Whose decisions are in the video: see docs/showcase/car/README.md. Typical run (hfbox):

    tools/remote.sh run --gpu '.venv/bin/python tools/record_build.py build docs/showcase/car/oplog.json /workspace/wt/napl/scratch/p1 --expect assemblies/car4.toml'
    tools/remote.sh run '.venv/bin/napl program examples/car4/square.scenario.toml --runs-dir /workspace/wt/napl/scratch/runs'
    tools/remote.sh run --gpu '.venv/bin/python tools/record_build.py drive <run dir> /workspace/wt/napl/scratch/p2'
    <vidvenv>/bin/python tools/record_build.py encode /workspace/wt/napl/scratch/p1 /workspace/wt/napl/scratch/p2 docs/showcase/car_from_scratch.mp4
"""

import argparse
import json
import math
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
W, H = 1280, 720
NAME, DESC = "car4", "4-wheel skid-steer car built from an empty design"
BLANK = '<mujoco model="blank"><worldbody><geom name="floor" type="plane" size="3 3 0.1"/></worldbody></mujoco>'
FPS1 = 6  # Part 1 screen grabs per second
INS = (
    630,
    360,
)  # Part 1 inset: render of the current build, pasted at (5, 32) over the viewer's scene-tree / inspector panels
DWELL = {
    "design_edit": 2.2,
    "design_undo": 2.2,
    "design_suggest": 1.8,
    "design_check": 1.8,
    "design_save": 2.5,
}


# ---- oplog ------------------------------------------------------------------------------------------------------------------------
def cmd_oplog(a):
    rows = [
        json.loads(line)
        for line in Path(a.session).read_text().splitlines()
        if line.strip()
    ]
    doc = {
        "about": "Request log of the exploration session in which an LLM agent (Claude) built assemblies/car4.toml from an empty design, one request at a "
        "time, reading each reply before choosing the next. `request` = a napl.design.viewer.Backend request (the same arguments as the MCP "
        "design_* tools); `why` = the reason the agent wrote when it sent it; `ok` = whether the Design accepted it (a refused request changed nothing). "
        "tools/record_build.py build replays these requests in order.",
        "requests": rows,
    }
    Path(a.out).write_text(json.dumps(doc, indent=1) + "\n")
    print(f"{a.out}: {len(rows)} requests")


# ---- one request -> caption lines ---------------------------------------------------------------------------------------------------
def _op_head(op):
    k = op.get("op")
    if k == "add":
        if op.get("root"):
            return f"add ROOT {op['id']} = {op['part']}"
        return f"add {op.get('id') or op['part']} = {op['part']} -> {op.get('to')}"
    if k == "wire":
        return f"wire {op['from']} -> {op['to']} ({op['part']})"
    if k == "autowire":
        return f"autowire {op['a']}" + (f" -> {op['b']}" if op.get("b") else "")
    if k == "device":
        return f"device {op['device'].get('type')} {op['device'].get('name')}"
    return k if k != "device_map" else f"device_map {op.get('name')}"


def head_of(req):
    tool, a = req["tool"], req.get("args") or {}
    if tool == "design_suggest":
        return f"suggest {a['target']}" + (
            f"  [{a['category']}]" if a.get("category") else ""
        )
    if tool == "design_check":
        return "check (DRC)"
    if tool == "design_undo":
        return "undo"
    if tool == "design_save":
        return "save (Design.save -> TOML)"
    ops = a.get("ops", [])
    heads = [_op_head(o) for o in ops]
    if len(heads) > 2:
        return f"{heads[0]}  (+{len(heads) - 1} more ops)"
    return "  |  ".join(heads)


def _snap_fact(c):
    s = c.get("snap") or {}
    if not s:
        return None
    k = s.get("k")
    cl = s.get("clearance_mm")
    alts = len(s.get("alternatives") or [])
    return (
        f"{c['id']}: placement k={k}, score {s['score']:.1f}, {s.get('evaluated', '?')} evaluated, {alts} alternatives"
        + (f", clearance {cl:.1f} mm" if cl is not None else "")
    )


def facts_of(req, reply):
    """-> (list of short fact lines drawn from the Design's own reply, drc dict or None)."""
    if not reply["ok"]:
        return [
            "REFUSED by the Design (nothing changed): " + reply["error"][:170]
        ], None
    tool, r = req["tool"], reply["result"]
    if tool == "design_suggest":
        rows = r["suggestions"]
        return [
            f"{len(rows)} candidates: "
            + ", ".join(f"{x['part']} ({x['score']:g})" for x in rows[:3])
        ], None
    if tool == "design_check":
        fs = r["findings"]
        return [
            f"{len(fs)} findings: " + ", ".join(sorted({f["code"] for f in fs}))
        ], None
    if tool == "design_save":
        return ["saved " + Path(r["saved"]).name], None
    out = []
    changes = r.get("changes", [])
    snaps = [x for x in (_snap_fact(c) for c in changes) if x]
    out += snaps[:2] + ([f"... +{len(snaps) - 2} more snaps"] if len(snaps) > 2 else [])
    for c in changes:
        o = c.get("op")
        if o == "wire":
            w = c["wire"]
            L = (
                w["length"]
                if isinstance(w["length"], str)
                else f"{w['length']['value']} m"
            )
            out.append(f"wire {w['from']} -> {w['to']}: {w['part']}, length {L}")
        elif o == "autowire":
            out.append("autowire: " + c["reasons"]["part"][:110])
        elif o == "autowire_all":
            out.append(
                f"autowire_all: wired {len(c['wired'])}, ambiguous {len(c['ambiguous'])}, unmatched {len(c['unmatched'])}"
            )
            out += [
                f"  wired {w['wire']['from']} -> {w['wire']['to']}"
                for w in c["wired"][:2]
            ]
        elif o == "auto_devices":
            out.append(
                f"auto_devices: created {[d['device'] for d in c['created']]}, unset {len(c['unset'])}"
            )
        elif o == "device":
            out.append(f"device {c['device']['name']} on {c['device']['instance']}")
    return out[:5], r.get("drc")


# ---- Part 1 -------------------------------------------------------------------------------------------------------------------------
def cmd_build(a):
    """Re-exec under xvfb-run (one screen, -fbdir to read it back), the way tools/screenshots.py does its UI shots."""
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
        ]
        argv += ["--expect", a.expect] if a.expect else []
        argv += ["--no-ui"] if a.no_ui else []
        subprocess.run(argv, check=True, timeout=600)


def ui_prelude(b, req):
    """What the window shows before an `add` request lands, driven through the same backend ops an agent calls: the LIBRARY tab
    opens, the part's name is typed into the search box (one character per 45 ms), then the BUILD tab returns and the ghost of
    the part appears at the port it is about to snap to (library.preview). The request itself is applied afterwards, unchanged."""
    ops = req["args"].get("ops") or []
    if req["tool"] != "design_edit" or not ops or ops[0].get("op") != "add":
        return
    op = ops[0]

    def call(tool, **args):
        r = json.loads(b(json.dumps({"tool": tool, "args": args})))
        if not r["ok"]:
            raise SystemExit(f"{tool}: {r['error']}")

    call("ui.mode", name="library")
    call("library.search", text=op["part"], type_ms=45)
    time.sleep(0.045 * len(op["part"]) + 0.7)
    call("ui.mode", name="build")
    if op.get("to") and not op.get("root"):
        try:
            call("library.preview", part=op["part"], to=op["to"])
        except SystemExit:  # a port the preview cannot show (wire / cable ports): the edit still happens
            return
        time.sleep(1.4)  # the ghost flight (1.2 s)


def _build(oplog, out, fb, expect, ui=True):
    import design_shot
    from napl import Sim, _napl
    from napl.design import Design
    from napl.design import assembly as A
    from napl.design.viewer import Backend

    out = Path(out)
    reqs = json.loads(Path(oplog).read_text())["requests"]
    d = Design.new(NAME, REPO / "parts", description=DESC)
    work = Path(tempfile.mkdtemp(prefix="napl-rec-"))
    b = Backend(lambda: d, work / "build")
    blank = work / "blank.xml"
    blank.write_text(BLANK)
    sim = Sim.load(
        blank, host=True
    )  # the window opens on an empty floor; the first successful edit loads the first real model
    t0 = time.time()
    frames, timeline, stop = [], [], threading.Event()
    n_png = [0]

    def grabber():
        while not stop.is_set():
            t = time.time() - t0
            try:
                img = design_shot.xwd_rgba(Path(fb) / "Xvfb_screen0")
            except Exception:  # the file is rewritten under us: skip this grab
                time.sleep(0.05)
                continue
            (out / f"f{n_png[0]:05d}.png").write_bytes(_napl.encode_png(img))
            frames.append(round(t, 3))
            n_png[0] += 1
            time.sleep(max(0.0, 1 / FPS1 - (time.time() - t0 - t)))

    def states():
        return sum(q["tool"] == "state" for q, _ in b.log)

    def session():
        while not b.log:  # the window's first poll: it is up
            time.sleep(0.2)
        time.sleep(2.5)  # empty design on screen
        inset = None
        for i, rq in enumerate(reqs, 1):
            req = dict(rq["request"])
            if req["tool"] == "design_save":
                req["args"] = {"path": str(work / "car4.toml")}
            s0, ts = states(), time.time() - t0
            if ui:
                ui_prelude(b, req)
            reply = json.loads(b(json.dumps(req)))
            if reply["ok"] != rq["ok"]:
                raise SystemExit(
                    f"request {rq['n']}: replay answered ok={reply['ok']}, the exploration got ok={rq['ok']}"
                )
            if reply["ok"] and req["tool"] in ("design_edit", "design_undo"):
                t1 = time.time()
                while (
                    states() == s0 and time.time() - t1 < 8
                ):  # the window's rev poll (1 s) -> state -> LoadModel
                    time.sleep(0.1)
            st = b.state(d) if d.doc["instances"] else None
            if (
                reply["ok"]
                and req["tool"] in ("design_edit", "design_undo")
                and st
                and st["xml"]
            ):
                # inset: an offscreen render of the SAME build the window just loaded (the window's own viewport is a small
                # slot between its panels); the encoder pastes it over the scene-tree / inspector panels
                # (a separate process: a second Vulkan context inside the viewer's process crashes it)
                inset = f"i{i:02d}.png"
                subprocess.run(
                    [sys.executable, __file__, "_render", st["xml"], str(out / inset)],
                    check=True,
                    capture_output=True,
                )
            t_inset = time.time() - t0
            time.sleep(DWELL[req["tool"]])
            facts, drc = facts_of(req, reply)
            timeline.append(
                dict(
                    i=i,
                    n=rq["n"],
                    inset=inset,
                    t_inset=round(t_inset, 3),
                    t0=round(ts, 3),
                    t1=round(time.time() - t0, 3),
                    head=head_of(req),
                    facts=facts,
                    why=rq.get("why", ""),
                    ok=reply["ok"],
                    drc_before=(drc or {}).get("before"),
                    drc_after=(drc or {}).get("after"),
                    mass=round(st["report"]["mass"]["total"], 4) if st else 0.0,
                    instances=len(d.doc["instances"]),
                    errors=len([f for f in d.check() if f.severity == "error"])
                    if d.doc["instances"]
                    else 0,
                    warnings=len([f for f in d.check() if f.severity == "warning"])
                    if d.doc["instances"]
                    else 0,
                )
            )
        time.sleep(4.0)  # the finished car
        stop.set()
        sim.close_viewer()

    threading.Thread(target=grabber, daemon=True).start()

    def guarded():
        try:
            session()
        finally:  # a failed request / replay mismatch used to end this thread silently and leave show() blocking forever
            stop.set()
            sim.close_viewer()

    threading.Thread(target=guarded, daemon=True).start()
    design_shot.watchdog(sim, 540)
    sim.show(width=W, height=H, design=b)
    stop.set()
    time.sleep(0.3)
    done = len(timeline) == len(reqs)
    same = None
    if expect:
        same = A.dumps(d.asm) == A.dumps(
            A.load(expect)
        )  # the replayed design == the saved assembly, byte for byte
    (out / "timeline.json").write_text(
        json.dumps(
            dict(
                frames=frames,
                requests=timeline,
                complete=done,
                final_matches_expect=same,
                fps=FPS1,
            ),
            indent=1,
        )
    )
    print(
        f"{len(frames)} frames, {len(timeline)}/{len(reqs)} requests, final design == {expect}: {same}"
    )
    if not done or same is False:
        raise SystemExit("incomplete or the replayed design differs from --expect")


# ---- Part 2 -------------------------------------------------------------------------------------------------------------------------
def decorated(xml_text):
    """The run's model plus VISUAL-ONLY floor markings (static worldbody boxes, contype = conaffinity = 0, no mass): a 0.25 m grid and the
    1 m square the program drives. Without them the plain grey floor gives a follow camera no sense of motion. The replay's final state hash
    is still checked against the run's (cmd_drive), which would catch any physical effect."""
    g = []
    for i in range(-8, 17):
        v = i * 0.25
        g.append(
            f'<geom type="box" pos="{v} 0.5 0.0003" size="0.0015 3 0.0003" rgba="0.58 0.6 0.64 1" contype="0" conaffinity="0"/>'
        )
        g.append(
            f'<geom type="box" pos="0.5 {v} 0.0003" size="3 0.0015 0.0003" rgba="0.58 0.6 0.64 1" contype="0" conaffinity="0"/>'
        )
    for pos, size in (
        ("0.5 0 0.0006", "0.506 0.006 0.0003"),
        ("0.5 1 0.0006", "0.506 0.006 0.0003"),
        ("0 0.5 0.0006", "0.006 0.506 0.0003"),
        ("1 0.5 0.0006", "0.006 0.506 0.0003"),
    ):
        g.append(
            f'<geom type="box" pos="{pos}" size="{size}" rgba="0.2 0.35 0.9 1" contype="0" conaffinity="0"/>'
        )
    return xml_text.replace("</worldbody>", "\n".join(g) + "\n</worldbody>", 1)


def cmd_drive(a):
    import numpy as np
    from napl import _napl, showcase
    from screenshots import Replay

    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    src = next((Path(a.run_dir) / "model").glob("car4.xml"))
    (out / "decorated.xml").write_text(decorated(src.read_text()))
    r = Replay(a.run_dir, out / "decorated.xml")
    total = r.res["sim"]["sim_s"]
    wheels = [f"/devices/wheel_{s}" for s in ("fl", "rl", "fr", "rr")]
    hud, pos = [], []
    n = int(total * a.fps)
    for k in range(n + 1):
        t = k / a.fps
        r.to(t)
        p = [float(x) for x in r.sim.query("/world/plate", "pos")[0]]
        qt = [float(x) for x in r.sim.query("/world/plate", "quat")[0]]
        pos.append(p)
        v = 0.0
        if k >= 4:
            q = pos[k - 4]
            v = ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5 / (4 / a.fps)
        hud.append(
            dict(
                t=round(t, 3),
                speed=round(v, 4),
                x=p[0],
                y=p[1],
                qw=qt[0],
                qz=qt[3],
                cur=[round(float(r.sim.query(w, "current")[0][0]), 3) for w in wheels],
            )
        )
        main = showcase.shot(
            r.sim,
            W,
            H,
            1,
            camera="orbit",
            az=a.az,
            el=a.el,
            dist=a.dist,
            target=[p[0], p[1], 0.02],
        )
        over = showcase.shot(
            r.sim,
            320,
            320,
            1,
            camera="orbit",
            az=-90,
            el=89,
            dist=3.0,
            target=[0.5, 0.5, 0.0],
        )
        main[16:336, W - 336 : W - 16] = (
            over  # overhead inset, top right (the encoder draws its frame)
        )
        (out / f"f{k:05d}.png").write_bytes(
            _napl.encode_png(np.ascontiguousarray(main))
        )
    h, ok = r.finish()
    (out / "hud.json").write_text(
        json.dumps(
            dict(
                fps=a.fps, frames=hud, state_hash=h, replay_matches_run=ok, sim_s=total
            ),
            indent=1,
        )
    )
    print(
        f"{n + 1} frames, replay hash {h} {'MATCHES' if ok else 'DIFFERS FROM'} the run's"
    )
    if not ok:
        raise SystemExit("replay hash mismatch")


# ---- encode (throwaway venv: av + Pillow) ----------------------------------------------------------------------------------------------
def cmd_encode(a):
    import av
    from PIL import Image, ImageDraw, ImageFont

    FONT = "/usr/share/fonts/truetype/freefont/FreeSans.ttf"
    MONO = "/usr/share/fonts/truetype/freefont/FreeMono.ttf"
    BOLD = "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf"
    f_s, f_m, f_b, f_t = (
        ImageFont.truetype(p, s)
        for p, s in ((FONT, 19), (MONO, 17), (BOLD, 21), (BOLD, 34))
    )
    FPS = 20
    p1, p2 = Path(a.part1), Path(a.part2)
    tl = json.loads((p1 / "timeline.json").read_text())
    hud = json.loads((p2 / "hud.json").read_text())
    codecs = [
        c
        for c in (
            "libx264",
            "libopenh264",
            "libvpx-vp9",
        )  # (the generic name "h264" resolves to libx264: not listed)
        if c in av.codecs_available
    ]
    print("encoders available among h264 / vp9:", codecs)
    out = Path(a.out)
    codec = next(c for c in codecs if c != "libx264" or not a.no_gpl)
    if codec.startswith("libvpx") and out.suffix != ".webm":
        out = out.with_suffix(".webm")
    cont = av.open(str(out), "w")
    st = cont.add_stream(codec, rate=FPS)
    st.width, st.height, st.pix_fmt = W, H, "yuv420p"
    st.options = (
        {"crf": "26", "preset": "slow"}
        if codec == "libx264"
        else {"crf": "34", "b": "0"}
        if codec.startswith("libvpx")
        else {}
    )

    def emit(img):
        for pk in st.encode(av.VideoFrame.from_image(img.convert("RGB"))):
            cont.mux(pk)

    def wrap(text, font, width, draw):
        lines, cur = [], ""
        for w in text.split():
            if draw.textlength(cur + " " + w, font=font) > width and cur:
                lines.append(cur)
                cur = w
            else:
                cur = (cur + " " + w).strip()
        return lines + ([cur] if cur else [])

    def card(title, lines, secs):
        img = Image.new("RGB", (W, H), (18, 20, 26))
        dr = ImageDraw.Draw(img)
        y = 120
        dr.text((60, y), title, font=f_t, fill=(240, 240, 245))
        y += 70
        for ln in lines:
            for sub in wrap(ln, f_s, W - 120, dr):
                dr.text((60, y), sub, font=f_s, fill=(200, 205, 215))
                y += 28
            y += 8
        for _ in range(int(secs * FPS)):
            emit(img)

    card(
        "A car, built from an empty design by an agent",
        [
            "Part 1: the napl viewer's Design window while an LLM agent (Claude) builds a 4-wheel skid-steer car, one design op at a time, reading each result before choosing the next op.",
            "Part 2: the finished car driving a 1 m square over two RoboClaw serial ports, with the unmodified basicmicro library.",
            "Disclosure: the decisions (which part, which port, which wire) were the agent's, made in an exploratory session; this take re-applies that logged session through the same Backend the window uses. The recording harness, the drive program and the custom plate part were written beforehand. Details: docs/showcase/car/README.md",
        ],
        7.0,
    )

    # ---- Part 1: the latest grabbed frame at each output time; inset (render of the current build) over the scene-tree / inspector panels,
    # caption panel over the (unused, empty) Library tab of the Design window; the DRC window and the instance list stay the real window
    ts = tl["frames"]
    reqs = tl["requests"]
    T = ts[-1]
    cache, ins_cache = {}, {}
    j = 0
    for k in range(int(T * FPS)):
        t = k / FPS
        while j + 1 < len(ts) and ts[j + 1] <= t:
            j += 1
        if j not in cache:
            cache = {j: Image.open(p1 / f"f{j:05d}.png").convert("RGB")}
        img = cache[j].copy()
        dr = ImageDraw.Draw(img)
        cur = None
        for r in reqs:
            if r["t0"] <= t < r["t1"] + 0.3:
                cur = r
        shown = [
            r for r in reqs if r["inset"] and r["t_inset"] <= t
        ]  # the last inset rendered by now
        if shown:
            name = shown[-1]["inset"]
            if name not in ins_cache:
                ins_cache = {name: Image.open(p1 / name).convert("RGB")}
            img.paste(ins_cache[name], (5, 32))
            dr.rectangle([4, 31, 5 + 630, 32 + 360], outline=(90, 110, 140))
            dr.text(
                (12, 36),
                "offscreen render of the build the window just loaded",
                font=f_s,
                fill=(235, 235, 235),
            )
        else:
            dr.rectangle([5, 32, 635, 392], fill=(18, 20, 26))
            dr.text(
                (200, 200), "empty design (no root yet)", font=f_b, fill=(200, 205, 215)
            )
        dr.rectangle([0, 0, W, 28], fill=(8, 10, 14))
        dr.text(
            (10, 3),
            "Part 1  build: every op was chosen by an LLM agent after reading the previous result (exploratory session; this take re-applies the log)",
            font=f_s,
            fill=(235, 235, 120),
        )
        # caption panel (right, over the empty Library tab)
        x0, y0, pw = 646, 262, 628
        dr.rectangle([x0, y0, x0 + pw, H - 4], fill=(8, 10, 14))
        lines = []
        if (
            cur is not None
        ):  # the request and the agent's reason from the moment it is sent; the Design's answer once the window shows it
            lines = [
                (
                    f"request {cur['i']}/{len(reqs)}   {cur['head']}",
                    f_b,
                    (255, 255, 255),
                )
            ]
            if cur["why"]:
                lines += [("agent's reason: " + cur["why"], f_s, (255, 215, 150))]
            if t >= cur["t_inset"] - 0.4:
                lines += [(x, f_m, (170, 230, 255)) for x in cur["facts"]]
        elif t >= reqs[-1]["t1"]:
            lines = [
                (
                    "finished: the replayed design is byte-identical to assemblies/car4.toml (Design.save)",
                    f_b,
                    (255, 255, 255),
                ),
                (
                    f"DRC: {reqs[-1]['errors']} errors, {reqs[-1]['warnings']} warnings (plus the info SERVO_ID_UNSET: the 4 wheel `servo` devices read as a shared serial bus, as on the rover)",
                    f_m,
                    (170, 230, 255),
                ),
            ]
        y = y0 + 8
        for txt, font, col in lines:
            for sub in wrap(txt, font, pw - 20, dr):
                if y > H - 56:
                    break
                dr.text((x0 + 10, y), sub, font=font, fill=col)
                y += 25 if font is not f_m else 21
            y += 4
        # status: the design as the window shows it (the previous request's until the window has loaded this one)
        shown_s = [r for r in reqs if r["t_inset"] <= t]
        s = shown_s[-1] if shown_s else None
        if s is not None:
            bad = s["errors"] > 0
            dr.text(
                (x0 + 10, H - 40),
                f"instances {s['instances']}   mass {s['mass']:.3f} kg   DRC errors {s['errors']}  warnings {s['warnings']}",
                font=f_b,
                fill=(255, 120, 120) if bad else (120, 255, 150),
            )
        emit(img)

    card(
        "Result of the build",
        [
            "assemblies/car4.toml, written by Design.save from the session just shown: 4 x Pololu 37D 4751 gearmotor on brackets, hubs and 90 mm wheels, 2 x RoboClaw 2x7A (one per side), 3S 2200 mAh pack, custom 4-bracket plate.",
            f"DRC: {reqs[-1]['errors']} errors. Mass {reqs[-1]['mass']:.3f} kg. Next: the drive, replayed from its replay log.",
        ],
        4.0,
    )

    # ---- Part 2
    fr = hud["frames"]
    xs = [(f["x"], f["y"]) for f in fr]
    for k, f in enumerate(fr):
        img = Image.open(p2 / f"f{k:05d}.png").convert("RGB")
        dr = ImageDraw.Draw(img)
        dr.rectangle([0, 0, W, 34], fill=(8, 10, 14))
        dr.text(
            (10, 6),
            "Part 2  simulate: replay of the napl program run (basicmicro library -> 2 RoboClaw ptys -> sim), follow camera",
            font=f_s,
            fill=(235, 235, 120),
        )
        cur = f["cur"]
        yaw = math.degrees(2 * math.atan2(f["qz"], f["qw"]))
        yaw = (yaw + 180) % 360 - 180 if abs(yaw) > 180 else yaw
        txt = [
            f"sim time   {f['t']:6.2f} s",
            f"speed      {f['speed']:6.2f} m/s    yaw {yaw:6.1f} deg",
            "wheel current, A   (fl  rl  fr  rr)",
            "           " + " ".join(f"{c:+5.2f}" for c in cur),
        ]
        dr.rectangle([0, 42, 470, 42 + 24 * len(txt) + 8], fill=(8, 10, 14))
        for i, s in enumerate(txt):
            dr.text((10, 46 + 24 * i), s, font=f_m, fill=(200, 255, 200))
        # overhead inset frame + trail: 320 px = 3.0 m * 2*tan(22.5 deg)... fixed camera dist 3.0, fovy 45 -> 2.49 m across, centre (0.5, 0.5)
        x0, y0, side = W - 336, 16, 320
        scale = side / (2 * 3.0 * 0.41421356)
        dr.rectangle([x0 - 1, y0 - 1, x0 + side, y0 + side], outline=(255, 255, 255))
        px = lambda x, y: (
            x0 + side / 2 + (x - 0.5) * scale,
            y0 + side / 2 - (y - 0.5) * scale,
        )  # top view: az -90 -> +x right, +y up
        dr.line(
            [px(*c) for c in ((0, 0), (1, 0), (1, 1), (0, 1), (0, 0))],
            fill=(120, 160, 255),
            width=1,
        )  # the 1 m square
        if k > 0:
            dr.line(
                [px(*xs[i]) for i in range(k + 1)], fill=(255, 220, 0), width=2
            )  # ground-truth trail
        emit(img)
    card(
        "What is real in this video",
        [
            "The design ops, the DRC, the build and the physics are the napl code paths (Design, DRC, MuJoCo); the drive is the unmodified basicmicro library over a pty.",
            "Not an autonomous run: the agent's decisions came from an exploratory session, replayed here. Skid steer scrubs: a commanded 90 degree turn at the geometric track gave about 50 degrees (docs/showcase/car/README.md).",
        ],
        5.0,
    )
    for pk in st.encode():
        cont.mux(pk)
    cont.close()
    print(f"wrote {out}: {out.stat().st_size / 1e6:.1f} MB, codec {codec}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("oplog")
    s.add_argument("session")
    s.add_argument("out")
    s = sub.add_parser("build")
    s.add_argument("oplog")
    s.add_argument("out_dir")
    s.add_argument(
        "--expect",
        help="assembly TOML the replayed design must equal (assemblies/car4.toml)",
    )
    s.add_argument(
        "--no-ui",
        action="store_true",
        help="skip the visible UI reactions (library tab, typed search, ghost preview); the edits and the final design are the same",
    )
    s = sub.add_parser("_build")
    s.add_argument("oplog")
    s.add_argument("out_dir")
    s.add_argument("fb")
    s.add_argument("--expect")
    s.add_argument("--no-ui", action="store_true")
    s = sub.add_parser("_render")
    s.add_argument("xml")
    s.add_argument("png")
    s = sub.add_parser("drive")
    s.add_argument("run_dir")
    s.add_argument("out_dir")
    s.add_argument("--fps", type=int, default=20)
    s.add_argument("--az", type=float, default=35.0)
    s.add_argument("--el", type=float, default=24.0)
    s.add_argument("--dist", type=float, default=0.7)
    s = sub.add_parser("encode")
    s.add_argument("part1")
    s.add_argument("part2")
    s.add_argument("out")
    s.add_argument(
        "--no-gpl",
        action="store_true",
        help="never use libx264 (GPL): fall back to openh264 / VP9",
    )
    a = ap.parse_args()
    if a.cmd == "_build":
        _build(a.oplog, a.out_dir, a.fb, a.expect, not a.no_ui)
    elif a.cmd == "_render":
        from napl import Sim, _napl, showcase

        img = showcase.shot(Sim.load(a.xml), INS[0], INS[1], 2, camera="iso")
        Path(a.png).write_bytes(_napl.encode_png(img))
    else:
        globals()["cmd_" + a.cmd](a)


if __name__ == "__main__":
    main()
