"""Screenshots of the BUILD / LIBRARY / WIRING UIs and single-assembly renders (phase 4, worker UI2). Never committed: images stay on
the host / in remote-out/. Every viewer run is under design_shot.watchdog (the viewer is asked to close after the budget; the process
is killed 15 s later), the grab thread closes the viewer in `finally`.

model ASSEMBLY [--out DIR] [--az 40 --el 25 --dist 0.6 --target x,y,z]     offscreen orbit render (GPU)
ui    SCENE [--out DIR] [--size WxH]    SCENE = wiring | wiring-flat | build-tooltip | library | all | wiring-turret
      | pages (the turret design: BUILD with a selection, LIBRARY, WIRING canvas, PROGRAM, SIMULATE, a camera.orbit pair)
      viewer under xvfb (GPU + xvfb); files are named ui-<tag>-<W>x<H>.png

tools/remote.sh run --gpu 'timeout 600 .venv/bin/python tools/ui_shots.py ui all --out build/ui'
tools/remote.sh run --gpu 'timeout 600 .venv/bin/python tools/ui_shots.py ui pages --size 1920x1080 --out build/ui'
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
W, H = (int(v) for v in os.environ.get("NAPL_SHOT_SIZE", "1280x720").split("x"))

# the x500v2 power chain, wired by the "agent" in the canvas (look_shots.py flatten shows the same session)
PICK = {
    "instances": ["bat", "pm02", "pdb", "esc1", "esc2", "mot1", "mot2"],
    "ports": [
        "bat.power_out",
        "pm02.power_in",
        "pm02.power_out",
        "pdb.power_in",
        "pdb.out1",
        "pdb.out2",
        "esc1.power_in",
        "esc1.motor_out",
        "esc2.power_in",
        "esc2.motor_out",
        "mot1.phase",
        "mot2.phase",
    ],
}
TURRET_PICK = {
    "instances": ["pi", "adapter", "camera", "psu5", "psu", "pan"],
    "ports": [
        "pi.gpio_uart",
        "pi.cam0",
        "pi.power_in",
        "adapter.bus",
        "adapter.power_in",
        "adapter.servo_bus",
        "camera.csi",
        "psu5.power_out",
        "psu.power_out",
        "pan.port_a",
    ],
}
WIRES = [  # (from, to, awg, length m)
    ("bat.power_out", "pm02.power_in", 10, 0.12),
    ("pm02.power_out", "pdb.power_in", 12, 0.10),
    ("pdb.out1", "esc1.power_in", 16, 0.14),
    ("pdb.out2", "esc2.power_in", 16, 0.14),
    ("esc1.motor_out", "mot1.phase", 18, 0.20),
    ("esc2.motor_out", "mot2.phase", 18, 0.20),
]
UNWIRE = (
    "bat-to-pm02",
    "pm02-to-pdb",
    "pdb-to-esc1",
    "pdb-to-esc2",
    "esc1-to-mot1",
    "esc2-to-mot2",
)


def cmd_model(a):
    import napl
    from napl import _napl, design

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="napl-ui-"))
    xml = design.build(
        REPO / "assemblies" / f"{a.assembly}.toml", REPO / "parts", work
    )["xml"]
    sim = napl.Sim.load(xml)
    napl.render_style("studio,noedges")
    target = [float(x) for x in a.target.split(",")] if a.target else None
    rgba, _ = sim.render(
        camera="orbit",
        az=a.az,
        el=a.el,
        dist=a.dist,
        target=target,
        fovy=40,
        width=W,
        height=H,
    )
    p = out / f"model-{a.assembly}.png"
    p.write_bytes(_napl.encode_png(rgba))
    print("wrote", p)


def cmd_ui(a):
    global W, H
    os.environ["NAPL_SHOT_SIZE"] = a.size  # the xvfb child reads it
    W, H = (int(v) for v in a.size.split("x"))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="napl-xvfb-") as fb:
        subprocess.run(
            [
                "xvfb-run",
                "-a",
                "-s",
                f"-screen 0 {W}x{H}x24 -fbdir {fb}",
                sys.executable,
                __file__,
                "_ui",
                a.scene,
                str(out),
                fb,
            ],
            check=True,
            timeout=400,
        )


def _ui(scene, out, fb):
    import design_shot
    from napl import Sim, _napl
    from napl.design import Design
    from napl.design.viewer import Backend

    d = Design.open(
        REPO
        / "assemblies"
        / ("turret.toml" if scene in ("wiring-turret", "pages") else "x500v2.toml")
    )
    scenes = (
        ["wiring", "build-tooltip", "library"]
        if scene == "all"
        else ["build", "library", "wiring-turret", "program", "simulate", "camera"]
        if scene == "pages"
        else [scene]
    )
    if "wiring" in scenes:
        for w in UNWIRE:
            d.unwire(w)
    b = Backend(lambda: d, tempfile.mkdtemp(prefix="napl-ui-"))
    b.auto_thumbs = True
    sim = Sim.load(b.state(d)["xml"], host=True)
    out = Path(out)

    def call(tool, args=None):
        r = json.loads(b(json.dumps({"tool": tool, "args": args or {}})))
        print(tool, "ok" if r["ok"] else r["error"], flush=True)
        return r

    def grab(tag):
        img = design_shot.grab(fb)
        (out / f"ui-{tag}-{W}x{H}.png").write_bytes(_napl.encode_png(img))
        print("wrote", tag, flush=True)

    def session():
        try:
            t0 = time.time()
            while (
                not b.log and time.time() - t0 < 60
            ):  # never wait on a viewer that does not come up
                time.sleep(0.2)
            time.sleep(3)
            for s in scenes:
                if s == "build":
                    call("ui.mode", {"name": "build"})
                    call("select", {"instances": []})
                    call("ui.camera", {"preset": "iso"})
                    call("ui.frame", {})  # nothing selected: frames everything
                    time.sleep(1.5)
                    call("select", {"instances": ["pan"]})
                    time.sleep(2.5)
                    grab("build")
                    call("ui.measure", {"a": "pi", "b": "pan"})
                    time.sleep(1.5)
                    grab("build-measure")
                    call("ui.measure", {"off": True})
                    call(
                        "ui.visibility",
                        {"instances": ["psu", "psu5", "adapter"], "visible": False},
                    )
                    time.sleep(1.2)
                    grab("build-hidden")
                    call("ui.visibility", {"visible": True})
                elif s == "program":
                    call("ui.mode", {"name": "program"})
                    time.sleep(2)
                    grab("program")
                elif s == "simulate":
                    call("ui.mode", {"name": "simulate"})
                    time.sleep(2)
                    grab("simulate")
                elif s == "camera":  # camera.orbit: two frames of one sweep
                    call("ui.mode", {"name": "build"})
                    call("camera.orbit", {"yaw_deg": 90, "seconds": 2.0})
                    time.sleep(0.3)
                    grab("camera-a")
                    time.sleep(2.2)
                    grab("camera-b")
                elif s == "build-tooltip":
                    call("ui.mode", {"name": "build"})
                    call("ui.hover", {"instance": "pdb", "seconds": 8})
                    time.sleep(2.5)
                    grab("build-tooltip")
                elif s == "library":
                    call("ui.mode", {"name": "library"})
                    print("thumbnails complete:", b.thumbs.wait(200), flush=True)
                    call("library.search", {"text": "imu"})
                    time.sleep(3)
                    grab("library-imu")
                    call("library.search", {"text": ""})
                    time.sleep(2.5)
                    grab("library")
                elif (
                    s == "wiring-turret"
                ):  # the Pi 5 with its full J8 header, wires on pins
                    call("ui.mode", {"name": "wiring"})
                    call("flatten.begin", TURRET_PICK)
                    time.sleep(3.5)
                    grab("wiring-turret")
                    call(
                        "ui.wire_view", {"zoom": 0.55}
                    )  # mid zoom: the wired pins keep their labels
                    time.sleep(0.8)
                    grab("wiring-turret-mid")
                    call("ui.wire_view", {"zoom": 1.0})  # zoomed in: every pin label
                    time.sleep(0.8)
                    grab("wiring-turret-zoom")
                    call("flatten.end")
                elif s == "wiring":
                    call("ui.mode", {"name": "wiring"})
                    call("flatten.begin", PICK)
                    time.sleep(2.5)
                    grab("wiring-cards")
                    for k, (f, t, awg, ln) in enumerate(WIRES):
                        call(
                            "flatten.wire",
                            {"from": f, "to": t, "awg": awg, "length": ln},
                        )
                        if k == 2:  # the third wire mid-drag
                            time.sleep(0.45)
                            grab("wiring-drawing")
                        time.sleep(1.2)
                    call(
                        "flatten.wire", {"from": "esc1.power_in", "to": "esc2.power_in"}
                    )  # refused: the reason is toasted
                    time.sleep(1.2)
                    grab("wiring-mid-edit")
                    call("flatten.end")
        finally:
            sim.close_viewer()

    design_shot.watchdog(sim, 300)
    threading.Thread(target=session, daemon=True).start()
    sim.show(width=W, height=H, design=b)


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("model")
    p.add_argument("assembly")
    p.add_argument("--out", default="build/ui")
    p.add_argument("--az", type=float, default=40)
    p.add_argument("--el", type=float, default=25)
    p.add_argument("--dist", type=float, default=0.6)
    p.add_argument("--target", default="")
    p = sub.add_parser("ui")
    p.add_argument("scene")
    p.add_argument("--out", default="build/ui")
    p.add_argument("--size", default="1280x720")
    p = sub.add_parser("_ui")
    p.add_argument("scene")
    p.add_argument("out")
    p.add_argument("fb")
    a = ap.parse_args(argv)
    {"model": cmd_model, "ui": cmd_ui, "_ui": lambda a: _ui(a.scene, a.out, a.fb)}[
        a.cmd
    ](a)


if __name__ == "__main__":
    main()
