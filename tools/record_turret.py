"""Recording harness for the phase-4 end-to-end video: the counter-UAS turret built from an EMPTY design in the viewer, wired, programmed and
flown against a drone swarm. Media is never committed (frames and the webm stay on the host); the video is made in a THROWAWAY venv with `av` +
Pillow (neither is a project dependency).

    explore OPLOG [--expect ASM] [--save OUT.toml]      no window: apply the op log through napl.design.viewer.Backend, print every answer, compare
                                                        the result with ASM (assemblies/turret.toml): same parts / poses (1 um) / mass, DRC
    build   OPLOG OUT_DIR --expect ASM [--save OUT.toml]
                                                        xvfb + the viewer on an EMPTY design; every step of the op log is sent through the Backend the
                                                        window talks to (library search, animated drag and drop, gizmos, colours, the 2D wiring canvas,
                                                        the PROGRAM page); the screen is grabbed at ~8 fps -> OUT_DIR/f*.png + timeline.json. Same
                                                        comparison as `explore`; the built design is written to --save (the sim flies THAT file).
    encode  BUILD_DIR SIM_DIR OUT.webm --run RUN_DIR    needs `av` + Pillow: title card, the build with captions (per-step speed), chapter cards, the
                                                        record_sim.py frames (with slow-motion chunks) with captions, end card with the run's numbers.
                                                        VP9 / OpenH264 only (libx264 is GPL: not used).

Op log (docs/showcase/turret/oplog.json): {"steps": [{seg, title, why, speed?, dwell?, reqs: [{tool, args, wait?, until?, expect_fail?}] |
macro: "place" (+ part, to | point, id, group, search, ...)}]}. `why` is the reason the agent gave. See docs/showcase/turret/README.md.
"""

import argparse
import copy
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
import videnc

W, H = 1280, 720  # the window / the screen grabs
BAR = 108  # the caption strip below the picture (the window's own status bar stays visible)
HO = H + BAR  # output frame height
FPS1 = 15  # build screen grabs per second (camera orbits need the density)
FPS = 20  # output
BLANK = '<mujoco model="blank"><worldbody><geom name="floor" type="plane" size="3 3 0.1"/></worldbody></mujoco>'
NAME, DESC = (
    "turret",
    "Counter-UAS pan-tilt turret built from an empty design in the viewer",
)


# ---- op log -> requests --------------------------------------------------------------------------------------------------------------
def R(tool, wait=0.0, **args):
    return {"tool": tool, "args": args, "wait": wait}


QUICK_SKIP = {
    "library.search",
    "camera.orbit",
    "camera.fly",
    "ui.camera",
    "ui.hover",
    "ui.measure",
    "ui.visibility",
}


def place_drop(m):
    """The `place` macro as one instant library.drop (quick mode)."""
    args = {
        k: m[k]
        for k in ("id", "group", "joint", "mate", "k", "fasteners")
        if m.get(k) is not None
    }
    where = {"to": m["to"]} if m.get("to") else {"point": m.get("point", [0, 0, 0])}
    return R("library.drop", 0.3, part=m["part"], **where, **args)


def expand(step, quick=False):
    """A step -> its list of requests ({tool, args, wait, until?, expect_fail?}). quick (preview runs, never the take): no typing, no
    flying ghost, no camera moves, 0.2 s waits: the design gets to the interesting step in seconds."""
    if quick:
        out = []
        for rq in step.get("pre", []) + (
            [place_drop(step)] if step.get("macro") == "place" else step["reqs"]
        ):
            if rq["tool"] in QUICK_SKIP:
                continue
            rq = dict(rq, wait=min(rq.get("wait", 0.0), 0.2))
            if rq["tool"] == "library.drag":
                rq = dict(rq, args=dict(rq["args"], animate=False))
                rq.pop("until", None)
            out.append(rq)
        return out
    if step.get("macro") != "place":
        return step["reqs"]
    m = step
    out = list(m.get("pre", []))
    if m.get(
        "search"
    ):  # the LIBRARY page: the part's name typed into the search box, the card shows up
        out.append(R("ui.mode", 0.3, name="library"))
        out.append(
            R(
                "library.search",
                0.045 * len(m["search"]) + m.get("browse", 1.2),
                text=m["search"],
                category=m.get("category", ""),
                filters=m.get("filters") or {},
                type_ms=45,
            )
        )
    # the drop point must be inside the 3D viewport when the ghost arrives (the window drops only there): BUILD page, everything in frame
    out += [
        R("ui.mode", 0.5, name="build"),
        R("select", 0.1, instances=[]),
        R("ui.frame", 1.0, instances=[]),
    ]
    if m.get("to"):
        inst, _, port = m["to"].partition(".")
        tgt = {"instance": inst, "port": port}
    else:
        tgt = {"point": m.get("point", [0, 0, 0])}
    args = {
        k: m[k]
        for k in ("id", "group", "offboard", "joint", "mate", "k", "port", "fasteners")
        if m.get(k) is not None
    }
    d = R(
        "library.drag", m.get("fly", 1.0), part=m["part"], to=tgt, animate=True, **args
    )
    d["until"] = {"instance": m["id"]}
    out.append(d)
    out += m.get(
        "post", []
    )  # e.g. select the new part (PROPERTIES panel), orbit around the assembly
    return out


# ---- design comparison ---------------------------------------------------------------------------------------------------------------
def _round(x, n=6):
    if isinstance(x, float):
        return round(x, n)
    if isinstance(x, list):
        return [_round(v, n) for v in x]
    if isinstance(x, dict):
        return {k: _round(v, n) for k, v in x.items()}
    return x


def canon(doc):
    """The assembly document with list order and wire / mate names normalised away (the build order is the agent's, the content is what counts)."""
    doc = _round(copy.deepcopy(doc))
    for i in doc["instances"]:
        if i.get("rpy") == [0.0, 0.0, 0.0]:
            del i["rpy"]
        i.pop("color", None)  # the agent's own cosmetic colours
    for m in doc.get("mates", []):
        if m.get("rotation") == 0:
            del m[
                "rotation"
            ]  # k = 0 is the default; add records it, a hand-written file leaves it out
    key = {
        "instances": lambda x: x["id"],
        "mates": lambda x: x.get("name") or f"{x['a']}--{x['b']}",
        "wires": lambda x: f"{x['from']}->{x['to']}",
        "devices": lambda x: x["name"],
        "targets": lambda x: x["name"],
    }
    out = {}
    for k, v in doc.items():
        out[k] = {key[k](x): x for x in v} if k in key else v
    for w in out.get("wires", {}).values():
        w.pop("name", None)
    return out


def compare_designs(d, expect):
    """-> (list of differences, info). Differences: sections / items present on one side only, or different; the built poses (1 um), the mass."""
    from napl.design import Design
    from napl.design.viewer import Backend

    e = Design.open(expect)
    a, b = canon(d.doc), canon(e.doc)
    diffs = []
    for sec in sorted(set(a) | set(b)):
        x, y = a.get(sec), b.get(sec)
        if isinstance(x, dict) and isinstance(y, dict) and sec != "assembly":
            for k in sorted(set(x) | set(y)):
                if k not in x:
                    diffs.append(f"{sec}[{k}]: missing in the built design")
                elif k not in y:
                    diffs.append(f"{sec}[{k}]: extra in the built design")
                elif x[k] != y[k]:
                    ks = (
                        sorted(
                            f
                            for f in set(x[k]) | set(y[k])
                            if x[k].get(f) != y[k].get(f)
                        )
                        if isinstance(x[k], dict)
                        else []
                    )
                    diffs.append(
                        f"{sec}[{k}]: differs in {ks}: built {json.dumps({f: x[k].get(f) for f in ks})[:300]} vs expected {json.dumps({f: y[k].get(f) for f in ks})[:300]}"
                    )
        elif x != y:
            diffs.append(f"{sec}: differs")
    work = Path(tempfile.mkdtemp(prefix="napl-cmp-"))
    sa, sb = (
        Backend(lambda: d, work / "a").state(d),
        Backend(lambda: e, work / "b").state(e),
    )
    pose = 0.0
    for iid, g in sa["geo"].items():
        h = sb["geo"].get(iid)
        if h is None:
            continue
        dp = max(abs(p - q) for p, q in zip(g["pos"] + g["R"], h["pos"] + h["R"]))
        if dp > 1e-6:
            diffs.append(
                f"pose of {iid} differs by {dp:.2e}: built {g['pos']} vs expected {h['pos']}"
            )
        pose = max(pose, dp)
    ma, mb = sa["report"]["mass"]["total"], sb["report"]["mass"]["total"]
    info = dict(
        max_pose_diff=pose,
        mass=ma,
        mass_expect=mb,
        drc_errors=len([f for f in d.check() if f.severity == "error"]),
        drc_warnings=len([f for f in d.check() if f.severity == "warning"]),
        drc_expect_errors=len([f for f in e.check() if f.severity == "error"]),
    )
    if pose > 1e-6:
        diffs.append(f"poses differ by up to {pose:.2e}")
    if abs(ma - mb) > 1e-9:
        diffs.append(f"mass {ma:.6f} vs {mb:.6f}")
    return diffs, info


# ---- running the steps ---------------------------------------------------------------------------------------------------------------
def run_steps(
    b,
    d,
    steps,
    window=None,
    on_step=None,
    log=print,
    save_to="turret_built.toml",
    quick_until=0,
):
    """Send every step through the Backend `b`. window: None (headless) or a dict with `states()` (count of the window's state polls)."""
    for n, st in enumerate(steps, 1):
        t0 = time.time()
        facts, findings, slow = [], [], []
        for rq in expand(st, quick=n <= quick_until):
            tq = time.time()
            args = {
                k: (save_to if v == "__SAVE__" else v) for k, v in rq["args"].items()
            }
            reply = json.loads(b(json.dumps({"tool": rq["tool"], "args": args})))
            ok_want = not rq.get("expect_fail")
            if reply["ok"] != ok_want:
                raise SystemExit(
                    f"step {n} ({st.get('title')}): {rq['tool']} answered ok={reply['ok']}: {reply.get('error')}"
                )
            if not reply["ok"]:
                facts.append("REFUSED: " + reply["error"][:160])
            s0 = window["states"]() if window else 0
            if rq.get(
                "until"
            ):  # the window flies the ghost, then sends the drop itself; headless the drag has already committed
                t1 = time.time()
                while time.time() - t1 < 12 and not any(
                    i["id"] == rq["until"]["instance"] for i in d.doc["instances"]
                ):
                    time.sleep(0.1)
                if not any(
                    i["id"] == rq["until"]["instance"] for i in d.doc["instances"]
                ):
                    raise SystemExit(
                        f"step {n}: {rq['until']} never appeared (the window did not drop it)"
                    )
            if (
                window
                and reply["ok"]
                and (
                    rq["tool"]
                    in (
                        "design_edit",
                        "design_undo",
                        "gizmo.move",
                        "gizmo.rotate",
                        "set_color",
                        "align",
                        "distribute",
                        "duplicate",
                        "mirror",
                    )
                    or rq.get("until")
                    or rq["tool"].startswith("flatten.")
                    and "state" in reply
                )
            ):
                t1 = time.time()
                while window["states"]() == s0 and time.time() - t1 < 6:
                    time.sleep(0.1)
            if (
                reply["ok"]
                and reply.get("result")
                and isinstance(reply["result"], dict)
            ):
                for c in reply["result"].get("changes", [])[:1]:
                    sn = c.get("snap") or {}
                    if sn:
                        facts.append(
                            f"snap score {sn['score']:.0f}, k={sn.get('k')}, {len(sn.get('alternatives') or [])} alternatives"
                        )
            if (
                window and time.time() - tq > 3.0 and not rq.get("until")
            ):  # the wire router / model rebuild: dead time on screen, the encoder plays it fast
                slow.append((tq, time.time()))
            if (
                rq["tool"] == "design_check"
            ):  # what the DRC said at this point (the captions' red banner)
                findings = [
                    f"{f.code}: {f.message}"
                    for f in d.check()
                    if f.code.startswith("PIN_")
                ]
            if window:
                time.sleep(rq.get("wait", 0.0))
        if window and n > quick_until:
            time.sleep(st.get("dwell", 0.6))
        mass = 0.0
        if d.doc["instances"]:
            mass = (b.state(d)["report"] or {}).get("mass", {}).get("total", 0.0)
        fs = d.check() if d.doc["instances"] else []
        rec = dict(
            n=n,
            seg=st.get("seg", ""),
            title=st.get("title", ""),
            why=st.get("why", ""),
            speed=st.get("speed", 1.0),
            t0=t0,
            t1=time.time(),
            facts=facts,
            findings=findings,
            slow=slow,
            instances=len(d.doc["instances"]),
            mass=round(mass, 4),
            errors=len([f for f in fs if f.severity == "error"]),
            warnings=len([f for f in fs if f.severity == "warning"]),
        )
        log(
            f"step {n:3d} {rec['seg']:8s} {rec['title'][:60]:60s} inst {rec['instances']:2d} mass {rec['mass']:.4f} DRC {rec['errors']}e/{rec['warnings']}w {facts}"
        )
        if on_step:
            on_step(rec)


def cmd_explore(a):
    from napl.design import Design
    from napl.design.viewer import Backend

    steps = json.loads(Path(a.oplog).read_text())["steps"]
    d = (
        Design.open(a.open)
        if a.open
        else Design.new(NAME, REPO / "parts", description=DESC)
    )
    b = Backend(lambda: d, Path(tempfile.mkdtemp(prefix="napl-explore-")))
    run_steps(
        b, d, steps, save_to=a.save or "turret_built.toml", quick_until=a.quick_until
    )
    if a.save:
        d.save(a.save)
    if a.expect:
        diffs, info = compare_designs(d, a.expect)
        print(json.dumps(info))
        print(
            "DIFFERENCES:"
            if diffs
            else "final design == expected (parts, poses, mass)",
            *diffs,
            sep="\n  ",
        )
        raise SystemExit(1 if diffs else 0)


# ---- build under xvfb ----------------------------------------------------------------------------------------------------------------
def cmd_build(a):
    videnc.keep_from(a)  # --keep-frames reaches the xvfb child through the environment
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
        argv += ["--save", a.save] if a.save else []
        argv += ["--open", a.open] if a.open else []
        argv += ["--quick-until", str(a.quick_until), "--steps", str(a.steps)]
        subprocess.run(argv, check=True, timeout=a.timeout)


def _build(a):
    import design_shot
    from napl import Sim
    from napl.design import Design
    from napl.design.viewer import Backend

    out, fb = Path(a.out_dir), a.fb
    steps = json.loads(Path(a.oplog).read_text())["steps"]
    if a.steps:  # preview: only the first N steps (the comparison with --expect then reports the missing rest)
        steps = steps[: a.steps]
    d = (
        Design.open(a.open)
        if a.open
        else Design.new(NAME, REPO / "parts", description=DESC)
    )
    work = Path(tempfile.mkdtemp(prefix="napl-rec-"))
    b = Backend(lambda: d, work / "build")
    b.auto_thumbs = not a.open  # --open (cuas-lite video) never shows the library page
    if not a.open:
        b.thumbs.start()  # the library page needs its thumbnails from frame 1 (cached in build/thumbs after the first run)
        b.thumbs.wait(900)
    blank = work / "blank.xml"
    blank.write_text(BLANK)
    if a.open:  # an opened design: build it first (cuas-lite: routes 72 wires, about 6 min), the window opens on it
        st = b.state(d)
        if st["xml"] is None:
            raise SystemExit(f"the design does not build: {st['build_error']}")
        sim = Sim.load(st["xml"], host=True)
    else:
        sim = Sim.load(
            blank, host=True
        )  # the window opens on an empty floor; the first drop loads the first real model
    t0 = time.time()
    frames, timeline, stop = [], [], threading.Event()

    def grabber():
        k = 0
        while not stop.is_set():
            t = time.time() - t0
            try:
                img = design_shot.grab(fb)
            except Exception:
                time.sleep(0.05)
                continue
            videnc.save_frame(out, f"f{k:05d}", img)
            frames.append(round(t, 3))
            k += 1
            time.sleep(max(0.0, 1 / FPS1 - (time.time() - t0 - t)))

    def states():
        return sum(q["tool"] == "state" for q, _ in b.log)

    def session():
        while not b.log:  # the window's first poll: it is up
            time.sleep(0.2)
        time.sleep(1.5)

        def on_step(rec):
            rec["t0"], rec["t1"] = round(rec["t0"] - t0, 3), round(rec["t1"] - t0, 3)
            rec["slow"] = [[round(a - t0, 3), round(z - t0, 3)] for a, z in rec["slow"]]
            timeline.append(rec)

        run_steps(
            b,
            d,
            steps,
            window={"states": states},
            on_step=on_step,
            quick_until=a.quick_until,
            save_to=a.save or str(work / "turret_built.toml"),
        )
        time.sleep(1.0)
        stop.set()
        sim.close_viewer()

    def guarded():
        try:
            session()
        except (
            BaseException
        ) as e:  # a refused request / a drop that never came ends the recording, loudly
            timeline.append(dict(error=f"{type(e).__name__}: {e}"))
            raise
        finally:
            stop.set()
            sim.close_viewer()

    threading.Thread(target=grabber, daemon=True).start()
    threading.Thread(target=guarded, daemon=True).start()
    design_shot.watchdog(sim, int(a.timeout) - 40)
    sim.show(width=W, height=H, design=b)
    stop.set()
    time.sleep(0.3)
    done = len(timeline) == len(steps) and not any("error" in t for t in timeline)
    diffs, info = compare_designs(d, a.expect) if a.expect and done else ([], {})
    (out / "timeline.json").write_text(
        json.dumps(
            dict(
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
    print(
        f"{len(frames)} frames, {len(timeline)}/{len(steps)} steps; {info}; differences: {diffs}"
    )
    if not done or diffs:
        raise SystemExit("incomplete, or the built design differs from --expect")


# ---- encode (throwaway venv: av + Pillow) --------------------------------------------------------------------------------------------
def cmd_encode(a):
    from PIL import Image, ImageDraw, ImageFont

    FONT = "/usr/share/fonts/truetype/freefont/FreeSans.ttf"
    BOLD = "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf"
    MONO = "/usr/share/fonts/truetype/freefont/FreeMono.ttf"
    f_s, f_b, f_t, f_h, f_m, f_sm = (
        ImageFont.truetype(p, s)
        for p, s in (
            (FONT, 17),
            (BOLD, 21),
            (BOLD, 40),
            (BOLD, 13),
            (MONO, 18),
            (FONT, 14),
        )
    )
    out = Path(a.out)
    enc = videnc.Encoder(
        out, W, HO, FPS, gpl=a.gpl
    )  # GPU (VA) when this host has it, else PyAV software
    marks = []  # (output second, label): the timeline printed at the end
    nframes = [0]

    def emit(img):
        enc.write(img)
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

    def card(title, lines, secs, label=None):
        marks.append((nframes[0] / FPS, label or title))
        img = Image.new("RGB", (W, HO), (10, 11, 13))
        dr = ImageDraw.Draw(img)
        dr.line([(80, 214), (230, 214)], fill=(255, 255, 255), width=2)
        y = 240
        for sub in wrap(title.upper(), f_t, W - 160, dr):
            dr.text((80, y), sub, font=f_t, fill=(245, 245, 248))
            y += 52
        y += 14
        for ln in lines:
            for sub in wrap(ln, f_s, W - 200, dr):
                dr.text((80, y), sub, font=f_s, fill=(185, 190, 200))
                y += 26
            y += 10
        for _ in range(int(secs * FPS)):
            emit(img)

    def bar(pic, seg, title, why, right=None, n=None, finding=None):
        """The picture (W x H) with the caption strip below it: black, small caps segment tag, bold title, the agent's reason, and in red
        the DRC finding the Design reported (when there is one). -> the output frame (W x HO)."""
        img = Image.new("RGB", (W, HO), (6, 7, 9))
        img.paste(pic, (0, 0))
        dr = ImageDraw.Draw(img, "RGBA")
        y0 = H
        dr.line([(0, y0), (W, y0)], fill=(255, 255, 255, 60), width=1)
        tag = seg.upper() + (f"   {n}" if n else "")
        dr.text((24, y0 + 9), tag, font=f_h, fill=(255, 196, 60))
        dr.text((24, y0 + 26), title, font=f_b, fill=(255, 255, 255))
        lines = wrap(why, f_s, 960 if right else W - 48, dr)[: 2 if finding else 3]
        for i, sub in enumerate(lines):
            dr.text((24, y0 + 54 + 19 * i), sub, font=f_s, fill=(190, 196, 208))
        if finding:
            dr.text(
                (24, y0 + 54 + 19 * 2),
                "DRC  " + finding[:150],
                font=f_sm,
                fill=(255, 120, 110),
            )
        if right:
            for i, ln in enumerate(right):
                tw = dr.textlength(ln, font=f_m)
                dr.text(
                    (W - 24 - tw, y0 + 14 + 21 * i),
                    ln,
                    font=f_m,
                    fill=(150, 235, 170)
                    if i == len(right) - 1 and "0 errors" in ln
                    else (200, 205, 215),
                )
        return img

    # ---- title
    card(
        "A counter-UAS turret, built and flown by an agent",
        [
            "An LLM agent (Claude) builds a pan / tilt turret from an empty design in the napl viewer: library, CAD-style build workspace, pin-level wiring sheet (the Raspberry Pi 5 with its full 40-pin header), Python programs. It then flies the result against a swarm of programmed drones.",
            "Everything in the picture is the napl code path: the Design backend the window talks to, the DRC, the build, MuJoCo physics with the pellet's drag, gravity and wind. Details and who decided what: docs/showcase/turret/README.md",
        ],
        6.0,
        "title",
    )

    # ---- build: grabbed frames, per-step speed
    bd = Path(a.build_dir)
    tl = json.loads((bd / "timeline.json").read_text())
    ts, steps = tl["frames"], tl["steps"]
    T = ts[-1]
    segs = []  # (src t0, src t1, speed, step or None)
    prev = 0.0
    # playback speed is an editing choice: the op log's current "speed" wins over the one stored at recording time
    ops = json.loads((REPO / "docs/showcase/turret/oplog.json").read_text())["steps"]
    for s in steps:
        s["speed"] = ops[s["n"] - 1].get("speed", 1.0)
        if s["t0"] > prev:
            segs.append((prev, s["t0"], 1.0, None))
        a0 = s["t0"]
        for sa, z in sorted(
            s.get("slow", [])
        ):  # the router / rebuild's dead time plays fast
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
    if T > prev:
        segs.append((prev, T, 1.0, None))
    cache = {}
    last_seg = None
    cur_step = steps[0]
    for t0_, t1_, sp, s in segs:
        if s is not None:
            cur_step = s
            if s["seg"] != last_seg:
                marks.append((nframes[0] / FPS, s["seg"]))
                last_seg = s["seg"]
        # --speedup: tighten the whole build, but keep every segment at least 4 s on screen so the caption can be read
        sp = max(sp, min(sp * a.speedup, (t1_ - t0_) / 4.0))
        n_out = max(1, int(math.ceil((t1_ - t0_) / sp * FPS)))
        for k in range(n_out):
            t = t0_ + k * sp / FPS
            j = max(0, min(len(ts) - 1, sum(1 for x in ts if x <= t) - 1))
            if j not in cache:
                cache = {j: videnc.load_frame(bd, f"f{j:05d}")}
            img = cache[j].copy()
            c = cur_step
            right = [
                f"{c['instances']} parts   "
                + (f"{c['mass']:.3f} kg" if c["mass"] else "mass pending"),
                f"DRC {c['errors']} errors, {c['warnings']} warnings",
            ]
            emit(
                bar(
                    img,
                    c["seg"],
                    c["title"],
                    c["why"],
                    right,
                    f"step {c['n']}/{len(steps)}"
                    + (
                        f"   {sp:.0f}x (wire router running)"
                        if sp >= 6 * c["speed"]
                        else f"   {sp:g}x"
                        if sp != 1
                        else ""
                    ),
                    (c.get("findings") or [None])[0],
                )
            )

    # ---- simulate
    run = Path(a.run)
    native = json.loads((run / "native.json").read_text())
    summary = (
        json.loads((run / "summary.json").read_text())
        if (run / "summary.json").exists()
        else {}
    )
    caps = json.loads(Path(a.captions).read_text())["captions"]
    sd = Path(a.sim_dir)
    metas = sorted(sd.glob("meta_*.json"))
    card(
        "Simulate: the built turret against the swarm",
        [
            "`napl program examples/cuas/cuas.scenario.toml` flew the assembly built above (CUAS_TURRET). What follows is its replay.naplog played back in the viewer with the HUD: real time, with slow motion around hits."
        ],
        3.0,
        "SIMULATE",
    )
    for mp in metas:
        m = json.loads(mp.read_text())
        fps_c, t0c, i0 = m["fps"], m["t0"], m["start_index"]
        slow = fps_c / FPS
        n = m["frames"]
        # the orbit camera eases in from its start pose: the first second of the run looks at the mast foot, not played
        for k in range(20 if i0 == 0 else 0, n):
            tsim = t0c + k / fps_c
            img = videnc.load_frame(sd, f"f{i0 + k:05d}")
            cap = next((c for c in caps if c["t0"] <= tsim < c["t1"]), None)
            cap = cap or {"title": "The built turret flies the scenario", "text": ""}
            emit(
                bar(
                    img,
                    "simulate",
                    cap["title"],
                    cap["text"],
                    [
                        f"sim t = {tsim:5.1f} s"
                        + (f"   slow motion 1/{slow:g}" if slow > 1.01 else "")
                    ],
                )
            )

    # ---- end card
    rounds, hits = summary["rounds"], summary["hits_on_drones"]
    destroyed = sorted(
        n for n, d in summary["drones"].items() if d["status"] == "destroyed"
    )
    card(
        "Result",
        [
            f"rounds fired {rounds}  |  hits on a drone {hits}  |  drones destroyed {len(destroyed)} of 4 ({', '.join(destroyed) or 'none'})",
            f"state hash {native['state_hash']} at tick {native['tick']}: the replay behind the frames above ends on {'EXACTLY this hash (replay MATCH)' if json.loads(metas[-1].read_text()).get('hash_matches') else 'a DIFFERENT hash (MISMATCH)'}.",
            "The turret proposes, a scripted operator authorizes. Pellet: .177, point-mass ballistics with air drag, gravity and wind. A generic air-rifle-class launcher; no real weapon is modelled.",
        ],
        8.0,
        "RESULT",
    )
    enc.close()
    print(
        f"wrote {out}: {out.stat().st_size / 1e6:.1f} MB, {nframes[0]} frames = {nframes[0] / FPS:.1f} s, codec {codec}"
    )
    print("timeline (output seconds):", ", ".join(f"{t:.0f}s {l}" for t, l in marks))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("explore")
    s.add_argument("oplog")
    s.add_argument("--expect")
    s.add_argument("--save")
    s.add_argument(
        "--open",
        help="start from this assembly instead of an empty design (cuas-lite video: tools/record_cuas_lite.py oplog)",
    )
    s.add_argument("--quick-until", type=int, default=0)  # test the preview path
    for name in ("build", "_build"):
        s = sub.add_parser(name)
        s.add_argument("oplog")
        s.add_argument("out_dir")
        if name == "_build":
            s.add_argument("fb")
        s.add_argument("--expect")
        s.add_argument("--save")
        s.add_argument("--open")
        s.add_argument("--timeout", type=float, default=800)
        s.add_argument(
            "--quick-until",
            type=int,
            default=0,
            help="preview run: steps 1..N are applied instantly (no animation); never for the take",
        )
        s.add_argument(
            "--steps", type=int, default=0, help="preview run: only the first N steps"
        )
        s.add_argument(
            "--keep-frames",
            action="store_true",
            help="also write a PNG per grabbed frame (default: raw .rgz only, no PNG encode)",
        )
    s = sub.add_parser("encode")
    s.add_argument("build_dir")
    s.add_argument("sim_dir")
    s.add_argument("out")
    s.add_argument("--run", required=True)
    s.add_argument("--speedup", type=float, default=1.0)
    s.add_argument(
        "--gpl",
        action="store_true",
        help="ffmpeg VAAPI first (av1_vaapi / h264_vaapi) and libx264 in the software fallback; default is GStreamer VA (LGPL)",
    )
    s.add_argument(
        "--captions", default=str(REPO / "docs/showcase/turret/sim_captions.json")
    )
    a = ap.parse_args()
    {
        "explore": cmd_explore,
        "build": cmd_build,
        "_build": _build,
        "encode": cmd_encode,
    }[a.cmd](a)


if __name__ == "__main__":
    main()
