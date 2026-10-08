#!/usr/bin/env python3
"""Before / after pictures of assemblies: each assemblies/<name>.toml built with and without the visual layer, rendered
at a 3/4 view -> build/shots/<name>-plain.png, <name>-visual.png and one side-by-side sheet build/shots/sheet.png
(plain left, visual right, one row per assembly). Needs a Vulkan device:

    tools/remote.sh run --gpu '.venv/bin/python tools/visual_shots.py [--size 640] [--az 35 --el 25] [NAME ...]'
"""

import argparse
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "tools"))
from napl import Sim, _napl
from napl.design.build import compute
from napl.design.library import Library
from thumbs import finish


def shot(xml_path, size, az, el, pad=0.06):
    sim = Sim.load(xml_path)
    pts = [
        v for v in sim.body_positions().values() if np.shape(v) == (3,)
    ]  # devices report longer state vectors
    pos = np.array(pts or [[0, 0, 0]])
    ctr = (pos.min(0) + pos.max(0)) / 2
    rad = float(np.linalg.norm(pos - ctr, axis=1).max()) + pad
    fovy = 30.0
    d = rad / math.sin(math.radians(fovy / 2))
    a, e = math.radians(az), math.radians(el)
    cam = ctr + d * np.array(
        [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]
    )
    return finish(
        *sim.render(pos=cam, lookat=ctr, fovy=fovy, width=2 * size, height=2 * size),
        size,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--out", default="build/shots")
    ap.add_argument("--size", type=int, default=640)
    ap.add_argument("--az", type=float, default=35)
    ap.add_argument("--el", type=float, default=25)
    ap.add_argument(
        "--pad",
        type=float,
        default=0.06,
        help="metres added to the framing radius (long barrels, wings)",
    )
    ap.add_argument(
        "--visual-only", action="store_true", help="skip the plain-primitive twin"
    )
    ap.add_argument(
        "--suffix", default="", help="appended to the PNG names (several angles)"
    )
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    lib = Library(REPO / "parts")
    names = a.names or sorted(p.stem for p in (REPO / "assemblies").glob("*.toml"))
    rows = []
    for n in names:
        row = []
        for tag, vis in (("plain", False), ("visual", True)):
            if a.visual_only and not vis:
                continue
            d = Path(tempfile.mkdtemp(prefix=f"napl-shot-{n}-", dir=out.resolve()))
            xml, _ = compute(
                REPO / "assemblies" / f"{n}.toml", REPO / "parts", d, lib, visual=vis
            )
            (d / "model.xml").write_text(xml)
            img = shot(d / "model.xml", a.size, a.az, a.el, a.pad)
            (out / f"{n}-{tag}{a.suffix}.png").write_bytes(_napl.encode_png(img))
            row.append(img)
        rows.append(np.concatenate(row, axis=1))
    (out / "sheet.png").write_bytes(_napl.encode_png(np.concatenate(rows, axis=0)))
    print(f"wrote {len(names)} before/after pairs to {out}")


if __name__ == "__main__":
    main()
