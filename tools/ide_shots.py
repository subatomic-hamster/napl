"""Screenshots of the PROGRAM page (the code editor / IDE) on the Skydio design, driven through the `prog.ui` backend op.

    xvfb-run -a -s "-screen 0 1600x900x24 -fbdir DIR" python tools/ide_shots.py OUT_DIR DIR

Works on a temporary copy of examples/skydio (the rename scene edits files). Writes ide-editor, ide-find (n of m + every match
highlighted), ide-replace, ide-search (find in files as typed), ide-define (F12), ide-rename (F2), ide-hover, ide-terminal.
Same capture path as tools/ui_shots.py (design_shot.grab).
"""

import json
import shutil
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
    root = Path(tempfile.mkdtemp(prefix="napl-ide-prog-")) / "skydio"
    shutil.copytree(
        REPO / "examples/skydio", root, ignore=shutil.ignore_patterns("__pycache__")
    )
    d = Design.open(REPO / "assemblies" / "skydio.toml")
    b = Backend(lambda: d, tempfile.mkdtemp(prefix="napl-ide-"), program_root=root)
    sim = Sim.load(b.state(d)["xml"], host=True)
    src = (root / "build_design.py").read_text().split("\n")
    hl = next(i for i, ln in enumerate(src) if 'd.wire("pdb.out1"' in ln)
    hc = src[hl].index("pdb.out1") + 4  # on `out1`
    pls = (root / "planner.py").read_text().split("\n")
    pl = next(i for i, ln in enumerate(pls) if ln.startswith("class Planner"))
    call = next(i for i, ln in enumerate(pls) if "= rollout(" in ln)

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
            grab("editor")  # explorer tree indentation
            ui(
                cmd="find", text="prm"
            )  # "n of m", every match highlighted, bar docked above the code
            time.sleep(1.5)
            grab("find")
            ui(cmd="find", text="prm", replace="params")  # replace row, buttons enabled
            time.sleep(1.5)
            grab("replace")
            ui(cmd="dock", name="search")
            ui(
                cmd="search", text="Planner", send=False
            )  # typed, never sent: must search as you type
            time.sleep(2)
            grab("search")
            ui(cmd="open", path="planner.py", line=call, col=pls[call].index("rollout"))
            time.sleep(1.5)
            ui(cmd="define")  # F12
            time.sleep(2.5)
            grab("define")
            ui(
                cmd="rename", new="simulate_rollout"
            )  # F2 on the definition's name, every file
            time.sleep(3)
            grab("rename")
            n = sum("simulate_rollout" in p.read_text() for p in root.glob("*.py"))
            print("rename on disk:", n, "file(s)", flush=True)
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

    design_shot.watchdog(sim, 260)
    threading.Thread(target=session, daemon=True).start()
    sim.show(width=W, height=H, design=b)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
