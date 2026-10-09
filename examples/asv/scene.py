"""The harbour scene: the ASV (assemblies/asv.toml) and the scripted traffic vessels (assemblies/asv-traffic.toml, the same hull / thrusters / device models) composed
by napl.cuas.scene.compose into ONE MJCF / one World, plus the sea, the seabed, the static shoreline, pier, breakwater, dock and buoys (asv.encounters.SCENERY).

Post-processing of compose()'s output:
  - ONE sea device (the kernel allows one): the ASV's, with a seeded JONSWAP sea of the Spec's Hs / Tp / direction; the traffic vessels' own sea devices are dropped.
  - the seabed is the floor moved to z = -3 (the design builder puts it at 0); the water surface is a drawn translucent plane at z = 0 in geom group 2, which the
    lidar's ray mask excludes (examples: asv.toml `ray_groups`); there is no rendered wave surface (the renderer has no deforming mesh), only the physics sea.
  - static scenery as geoms with conaffinity 1 (vehicles collide with them: contype 1 / conaffinity 0 on the vehicle side), group 0 (visible to the lidar).
  - a cabin block on each traffic vessel's hull body (a box 0.6 x 0.28 x 0.9 m from deck to 1 m above the keel plane) standing in for the superstructure a real work
    boat carries at lidar height: the lidar plane is ~0.5 m above the water, over the bare catamaran deck.
Everything is deterministic in (Spec, seed). `build(out)` is what `napl program`'s [model] script calls; the programs rebuild with their own Spec (`build_spec`).
"""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from napl.cuas.scene import Vehicle, compose

from . import encounters as enc

REPO = Path(__file__).resolve().parents[2]
SEABED = -3.0
WATER_RGBA = "0.10 0.30 0.45 0.55"


def _fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


def _box(wb, name, c, half, rgba, group=0):
    ET.SubElement(
        wb,
        "geom",
        name=name,
        type="box",
        pos=_fmt(c),
        size=_fmt(half),
        rgba=_fmt(rgba),
        group=str(group),
        contype="1",
        conaffinity="1",
    )


def _cyl(wb, name, c, r, hh, rgba):
    ET.SubElement(
        wb,
        "geom",
        name=name,
        type="cylinder",
        pos=_fmt(c),
        size=_fmt((r, hh)),
        rgba=_fmt(rgba),
        group="0",
        contype="1",
        conaffinity="1",
    )


def build_spec(out, spec: enc.Spec, seed=None, visual=False):
    """Compose and post-process the scene of `spec` into `out`/scene.xml -> path. seed = device seed (default spec.seed)."""
    out = Path(out)
    seed = int(spec.seed if seed is None else seed)
    veh = [Vehicle("asv", "asv", pos=(0.0, 0.0, 0.0), yaw=0.0)]
    for t in spec.traffic:
        veh.append(
            Vehicle(
                t.name,
                "asv-traffic",
                pos=(t.pos[0], t.pos[1], 0.0),
                yaw=math.degrees(t.psi),
            )
        )
    res = compose(out, veh, seed=seed, visual=visual)
    path = Path(res["model"])
    root = ET.parse(path).getroot()
    wb = root.find("worldbody")
    txt = root.find("custom/text")
    d = json.loads(txt.get("data"))
    # one sea: the ASV's; waves from the Spec (direction = the way they travel, math angle)
    sea = next(s for s in d["seas"] if s["name"] == "asv_sea")
    sea["jonswap"] = (
        dict(hs=spec.hs, tp=spec.tp, direction=spec.wave_dir, seed=1000 + seed)
        if spec.hs > 0
        else None
    )
    if sea["jonswap"] is None:
        del sea["jonswap"]
    if spec.current != (0.0, 0.0):
        sea["current"] = [spec.current[0], spec.current[1], 0.0]
    d["seas"] = [sea]
    # lidar only on the ASV; traffic carries none in its assembly, but be explicit
    d["rangefinders"] = [
        r for r in d.get("rangefinders", []) if r["name"].startswith("asv_")
    ]
    txt.set("data", json.dumps(d, sort_keys=True, separators=(",", ":")))
    # seabed + water plane
    for g in wb.findall("geom"):
        if g.get("name") == "floor":
            g.set("pos", _fmt((0, 0, SEABED)))
            g.set("rgba", "0.30 0.27 0.20 1")
    ET.SubElement(
        wb,
        "geom",
        name="water",
        type="plane",
        size="400 400 0.1",
        pos="0 0 0",
        rgba=WATER_RGBA,
        group="2",
        contype="0",
        conaffinity="0",
    )
    # static scenery and buoys
    for name, kind, c, size, rgba in enc.SCENERY:
        _box(wb, name, c, size, rgba)
    for name, (x, y), rgba in enc.BUOYS:
        _cyl(wb, name, (x, y, 0.25), enc.BUOY_R, enc.BUOY_H + 0.25, rgba)
    for o in spec.obstacles:  # floating obstacles on the route: a drum (r 0.4 m) with a mast stub, 1 m above the water
        _cyl(wb, o.name, (o.pos[0], o.pos[1], 0.35), 0.4, 0.65, (0.9, 0.55, 0.1, 1))
    # cabin blocks on the traffic hulls
    for b in wb.findall("body"):
        n = b.get("name", "")
        if n.endswith("_hull") and not n.startswith("asv_"):
            ET.SubElement(
                b,
                "geom",
                name=f"{n}_cabin",
                type="box",
                pos="0 0 0.55",
                size="0.30 0.14 0.45",
                rgba="0.85 0.85 0.80 1",
                group="4",
                contype="1",
                conaffinity="0",
                mass="0.05",  # a sensing proxy: its own mass would be 150 kg at MuJoCo's default density
            )
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path


def build(out):
    """`napl program` [model] script entry: the default scene (head-on encounter, calm-ish sea)."""
    return build_spec(out, enc.make("head_on", seed=1, hs=0.4))
