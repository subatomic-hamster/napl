"""Scene of the boat float: assemblies/boat-cat.toml on a pool. The design builder puts a floor at z = 0 (where a land vehicle stands); here the
floor is moved down to the seabed (z = -2) and the still water stays at z = 0. FLOAT_HS (env, m, default 0 = calm) adds a seeded JONSWAP sea of
that significant wave height (peak period FLOAT_TP, default 2.5 s; seed FLOAT_SEED, default 1). `napl program` calls build(outdir) -> MJCF path."""

import json
import os
import re
from pathlib import Path

from napl import design

REPO = Path(__file__).resolve().parents[2]
SEABED = -2.0


def build(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    xml, _ = design.compute(REPO / "assemblies" / "boat-cat.toml", REPO / "parts", out)
    xml = xml.replace(
        '<geom name="floor" type="plane"',
        f'<geom name="floor" type="plane" pos="0 0 {SEABED}"',
        1,
    )
    hs = float(os.environ.get("FLOAT_HS", "0"))
    if hs > 0:
        m = re.search(r"(<text name=\"napl.devices\" data=')([^']*)(')", xml)
        d = json.loads(m.group(2))
        d["seas"][0]["jonswap"] = dict(
            hs=hs,
            tp=float(os.environ.get("FLOAT_TP", "2.5")),
            seed=int(os.environ.get("FLOAT_SEED", "1")),
        )
        xml = xml[: m.start(2)] + json.dumps(d, separators=(",", ":")) + xml[m.end(2) :]
    (out / "scene.xml").write_text(xml)
    return out / "scene.xml"
