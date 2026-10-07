"""Screenshot of the viewer's Design window after a scripted editing session on the X500 (docs/showcase/design_window.png).

    xvfb-run -a -s "-screen 0 1280x720x24 -fbdir DIR" python tools/design_shot.py OUT.png DIR

Xvfb's -fbdir keeps the screen as an XWD file; this reads it while the window is up (the viewer has no readback of its own).
"""

import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from napl import Sim, _napl
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[1]
# remove the PDB (new DRC findings), browse parts for its seat, snap one back (4 placements), preview the third
SCRIPT = "2 dremove pdb; 4 dsuggest bottom.pdb pcb; 6 dsnap holybro-pdb-60a bottom.pdb pdb; 8 dcycle 2; 12 dfinding 4"
EXPECT = (
    4  # design requests the script makes (the picture is taken after the last answer)
)


def xwd_rgba(path):
    b = Path(path).read_bytes()
    h = struct.unpack(">25I", b[:100])
    hdr, depth, w, hgt, bpl, ncol = h[0], h[3], h[4], h[5], h[12], h[19]
    assert h[11] == 32 and h[7] == 0, "expects 32 bpp little-endian Xvfb output"
    px = np.frombuffer(b, np.uint8, offset=hdr + ncol * 12, count=hgt * bpl).reshape(
        hgt, bpl
    )
    bgrx = px[:, : w * 4].reshape(hgt, w, 4)
    return np.ascontiguousarray(
        np.dstack(
            [
                bgrx[:, :, 2],
                bgrx[:, :, 1],
                bgrx[:, :, 0],
                np.full((hgt, w), 255, np.uint8),
            ]
        )
    )


def main(out, fbdir):
    d = Design.open(REPO / "assemblies" / "x500v2.toml")
    b = Backend(lambda: d, tempfile.mkdtemp(prefix="napl-shot-"))
    sim = Sim.load(b.state(d)["xml"], host=True)
    shot = []

    def grab():
        t0 = time.time()
        while (
            time.time() - t0 < 120
            and sum(q["tool"] not in ("rev", "state") for q, _ in b.log) < EXPECT
        ):
            time.sleep(0.2)
        time.sleep(2.5)  # reload + a few frames
        shot.append(xwd_rgba(Path(fbdir) / "Xvfb_screen0"))
        sim.close_viewer()

    threading.Thread(target=grab, daemon=True).start()
    sim.show(script=SCRIPT, width=1280, height=720, design=b)
    img = shot[0]
    Path(out).write_bytes(_napl.encode_png(img))
    print(
        f"wrote {out}: {img.shape[1]}x{img.shape[0]}, {Path(out).stat().st_size} bytes; requests {[q['tool'] for q, _ in b.log if q['tool'] != 'rev']}"
    )


if __name__ == "__main__":
    main(*sys.argv[1:3])
