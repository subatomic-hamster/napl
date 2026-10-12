"""Screenshots of the PROGRAM page (the code editor / IDE) on the Skydio design, driven through the `prog.ui` backend op.

    xvfb-run -a -s "-screen 0 1600x900x24 -fbdir DIR" python tools/ide_shots.py OUT_DIR DIR

Writes ide-editor.png, ide-find.png, ide-hover.png, ide-terminal.png. Same capture path as tools/ui_shots.py (design_shot.grab).
"""

import json
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import design_shot
from napl import Sim, _napl
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[1]
W, H = 1600, 900


def main(out, fb):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    d = Design.open(REPO / "assemblies" / "skydio.toml")
    b = Backend(lambda: d, tempfile.mkdtemp(prefix="napl-ide-"))
    sim = Sim.load(b.state(d)["xml"], host=True)
    src = (REPO / "examples/skydio/build_design.py").read_text().split("\n")
    hl = next(i for i, ln in enumerate(src) if 'd.wire("pdb.out1"' in ln)
    hc = src[hl].index("pdb.out1") + 4  # on `out1`
    pl = next(
        i for i, ln in enumerate(
            (REPO / "examples/skydio/planner.py").read_text().split("\n")
        ) if ln.startswith("class Planner")
    )  # fmt: skip

    def ui(**a):
        r = json.loads(b(json.dumps({"tool": "prog.ui", "args": a})))
        assert r["ok"], r

    def grab(tag):
        (out / f"ide-{tag}.png").write_bytes(_napl.encode_png(design_shot.grab(fb)))
        print("wrote", tag, flush=True)

    def session():
        try:
            t0 = time.time()
            while not b.log and time.time() - t0 < 60:
                time.sleep(0.2)
            time.sleep(3)
            b(json.dumps({"tool": "ui.mode", "args": {"name": "program"}}))
            time.sleep(1.5)
            ui(cmd="open", path="planner.py", line=pl, col=6, len=7)
            time.sleep(2.5)
            grab("editor")
            ui(cmd="cmd", name="find")
            time.sleep(1.5)
            grab("find")
            ui(cmd="open", path="build_design.py", line=hl, col=hc)
            time.sleep(1.5)
            ui(cmd="hover", line=hl, col=hc)
            time.sleep(3)
            grab("hover")
            ui(cmd="hover", line=-1)
            ui(cmd="term", text="ls")
            time.sleep(0.8)
            ui(cmd="term", text='python -c "print(1)"')
            time.sleep(0.8)
            ui(cmd="term", text="ls --color=always .. | head -4")
            time.sleep(1.5)
            grab("terminal")
        finally:
            sim.close_viewer()

    design_shot.watchdog(sim, 200)
    threading.Thread(target=session, daemon=True).start()
    sim.show(width=W, height=H, design=b)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
