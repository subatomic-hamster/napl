"""Screenshots of the renderer look and the 2D wiring canvas (phase 4). Never committed: images stay on the host / in remote-out/.

models  [--out DIR]    x500v2 / so101 / rover with their wire tubes, outdoor and studio styles (+ CAD edges): offscreen, GPU
flatten [--out DIR]    the viewer under xvfb: 3D -> flatten -> cards -> wires drawn (drag animation) -> back to 3D: GPU + xvfb

tools/remote.sh run --gpu '.venv/bin/python tools/look_shots.py models --out build/look'
tools/remote.sh run --gpu '.venv/bin/python tools/look_shots.py flatten --out build/look'
"""

import argparse
import json
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

# (assembly, camera az, el, dist, target) -- close enough that the 2 mm cables are a few pixels wide
MODELS = {
    "x500v2": (45, 28, 0.95, None),
    "so101": (50, 22, 0.75, None),
    "rover": (40, 25, 1.4, None),
}
STYLES = {
    "outdoor": "outdoor,noedges",
    "studio": "studio,noedges",
    "outdoor-edges": "outdoor,edges",
}


def cmd_models(a):
    import napl
    from napl import _napl, design

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="napl-look-"))
    for name, (az, el, dist, target) in MODELS.items():
        xml = design.build(
            REPO / "assemblies" / f"{name}.toml", REPO / "parts", work / name
        )["xml"]
        sim = napl.Sim.load(xml)
        for style, words in {**STYLES, "outdoor-low": "outdoor,noedges"}.items():
            napl.render_style(words)
            rgba, _ = sim.render(
                camera="orbit",
                az=az,
                el=5
                if style == "outdoor-low"
                else el,  # low: the horizon and sky are in frame
                dist=dist,
                target=target,
                fovy=45,
                width=W,
                height=H,
            )
            p = out / f"{name}-{style}.png"
            p.write_bytes(_napl.encode_png(rgba))
            print("wrote", p)


# ---- the 2D canvas, driven the way an agent would drive it -------------------------------------------------------------------
PICK = {
    "instances": ["bat", "pm02", "pdb", "esc1", "esc2", "mot1"],
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
        "mot1.phase",
    ],
}
WIRES = [  # (from, to, awg, length m)
    ("bat.power_out", "pm02.power_in", 10, 0.12),
    ("pm02.power_out", "pdb.power_in", 12, 0.10),
    ("pdb.out1", "esc1.power_in", 16, 0.14),
    ("esc1.motor_out", "mot1.phase", 18, 0.20),
]


def cmd_flatten(a):
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
                "_flatten",
                str(out),
                fb,
            ],
            check=True,
            timeout=300,
        )


def _flatten(out, fb):
    import design_shot
    from napl import Sim, _napl
    from napl.design import Design
    from napl.design.viewer import Backend

    d = Design.open(REPO / "assemblies" / "x500v2.toml")
    for w in ("bat-to-pm02", "pm02-to-pdb", "pdb-to-esc1", "esc1-to-mot1"):
        d.unwire(w)  # the agent wires them in the canvas
    b = Backend(lambda: d, tempfile.mkdtemp(prefix="napl-flat-"))
    sim = Sim.load(b.state(d)["xml"], host=True)
    out = Path(out)

    def call(tool, args=None):
        r = json.loads(b(json.dumps({"tool": tool, "args": args or {}})))
        print(tool, "ok" if r["ok"] else r["error"])
        return r

    def grab(tag):
        img = design_shot.grab(fb)
        (out / f"flatten-{tag}.png").write_bytes(_napl.encode_png(img))
        print("wrote", tag)

    def session():
        t0 = time.time()
        while (
            not b.log and time.time() - t0 < 60
        ):  # budget: never wait on a viewer that does not come up
            time.sleep(0.2)
        time.sleep(3)
        grab("0-3d")
        call("flatten.begin", PICK)
        time.sleep(1.1)
        grab("1-cards-popping")
        time.sleep(1.5)
        grab("2-cards")
        for i, (f, t, awg, ln) in enumerate(WIRES):
            call("flatten.wire", {"from": f, "to": t, "awg": awg, "length": ln})
            if i == 1:
                time.sleep(0.75)
                grab("3-wire-being-drawn")
                time.sleep(1.0)
            else:
                time.sleep(1.4)
        call(
            "flatten.wire", {"from": "esc1.power_in", "to": "esc2.power_in"}
        )  # a refusal: the reason is shown
        time.sleep(1.0)
        grab("4-mid-edit")
        call("flatten.end")
        time.sleep(2.5)
        grab("5-back-in-3d")
        sim.close_viewer()

    wd = threading.Timer(120, lambda: sim.close_viewer())  # hard time budget
    wd.daemon = True
    wd.start()
    threading.Thread(target=session, daemon=True).start()
    sim.show(width=W, height=H, design=b)


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    for n in ("models", "flatten"):
        p = sub.add_parser(n)
        p.add_argument("--out", default="build/look")
    p = sub.add_parser("_flatten")
    p.add_argument("out")
    p.add_argument("fb")
    a = ap.parse_args(argv)
    {
        "models": cmd_models,
        "flatten": cmd_flatten,
        "_flatten": lambda a: _flatten(a.out, a.fb),
    }[a.cmd](a)


if __name__ == "__main__":
    main()
