"""Scene builder of the Skydio landing scenarios: assemblies/skydio-land.toml composed (python/napl/cuas/scene.py) with the family's ground truth (land_world.py).

`napl program` calls build(outdir) of the scenario's model script (a three-line file beside the scenario, `build_for(out, __file__)`); the scenario's
`[land]` table names the family and its fixed settings, its `[[parameters]]` (NAPL_PARAMS) the knobs. Families:

  forest   take-off at the origin of the original Skydio forest (wall doorway, arch, trees, poles, boulders), designated clearing 58 m away, alternate clearing
           11-15 m from it; steady wind from a seeded direction + Dryden turbulence; `hazard`: the designated clearing holds boulders or a log (drawn from the seed)
  rain     the same forest in rain of `rain_mm_h` (kernel rain device: rotor thrust loss, water film, depth dropouts and false near returns)

Obstacles are static geoms of one body `obstacles` (contype / conaffinity 1, geom group 0: visible to the depth camera and the lidar, they collide with the
airframe). Contact sensors: `hit_obstacles` (airframe vs obstacles), `hit_ground` (airframe vs the world body: floor, terrain, rocks), `hit_gear_f` / `hit_gear_r`
(the two landing-gear legs vs the world): a landing is gear contact only, anything else on the ground is a crash. Pads are painted markers (group 4: drawn, not
rayed, no contact).
"""

import json
import math
import sys
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "skydio"))
import land_world as W
from napl import params as nparams
from napl.cuas.scene import Environment, Vehicle, compose
from napl.terrain import Pad, WindField, _marker
from skydio_scene import (
    add_obstacles,
)  # the original's: forest.Obstacle -> static geoms of the body `obstacles`

VEHICLE = "skydio"
ASSEMBLY = "skydio-land"
TERRAIN_GROUP = 2
CATCH_AREA, WET_AREA = 0.09, 0.25  # m^2: X500 estimates (docs/models.md Rain)


def scenario_cfg(scenario):
    sc = tomllib.loads(Path(scenario).read_text())
    return sc.get("land", {}), nparams.values(scenario), sc


def make_world(fam, seed, prm, cfg):
    if fam in ("forest", "rain"):
        h = prm.get("hazard", "auto")
        return W.forest_world(seed, None if h == "auto" else h == "yes")
    if fam in W.TERRAIN_FAMILIES:
        h = prm.get("hazard", "auto")
        return W.terrain_world(fam, seed, None if h == "auto" else h == "yes")
    raise ValueError(f"unknown family {fam!r}")


def wind_vector(seed, speed, az_deg=None, jitter_deg=0.0):
    """Steady wind: a seeded azimuth (world x = East, y = North; the wind blows TOWARD az) unless `az_deg` is given (+- `jitter_deg`, seeded)."""
    rng = np.random.default_rng(seed + 11)
    az = float(rng.uniform(0, 2 * math.pi))
    if az_deg is not None:
        az = math.radians(az_deg + jitter_deg * float(rng.uniform(-1, 1)))
    return (speed * math.cos(az), speed * math.sin(az), 0.0), math.degrees(az)


def build(out, scenario):
    """Build the scenario's scene under `out` -> path of the MJCF. Also writes land.json (mission + truth + settings) beside it."""
    cfg, prm, sc = scenario_cfg(scenario)
    fam, seed = cfg["family"], prm["seed"]
    if fam.startswith("deck"):
        return build_deck(out, cfg, prm, sc)
    world = make_world(fam, seed, prm, cfg)
    wind, az = wind_vector(
        seed,
        prm["wind_ms"],
        cfg.get("wind_az_deg"),
        float(cfg.get("wind_jitter_deg", 0.0)),
    )
    rain = float(prm.get("rain_mm_h", cfg.get("rain_mm_h", 0.0)))
    origin_alt = float(cfg.get("origin_alt", 0.0))
    wf = None
    if world.terrain is not None:
        wfc = cfg.get("wind_field", {})
        wf = WindField(
            **{
                **dict(
                    z0=0.01, ridge_gain=1.0, ridge_len=80.0, lee_turb=2.0, lee_down=0.2
                ),
                **wfc,
            }
        )
    gusts = []
    if (
        float(prm.get("gust_ms", 0.0)) > 0
    ):  # a front: 1-cos gust of length H m moving with the mean wind (kernel wind device `gust`)
        d = [math.cos(math.radians(az)), math.sin(math.radians(az)), 0.0]
        gusts.append(
            dict(
                t=float(prm.get("gust_t_s", 60.0)),
                amp=float(prm["gust_ms"]),
                H=float(cfg.get("gust_len_m", 20.0)),
                dir=d,
                speed=max(prm["wind_ms"], 1.0),
            )
        )
    env = Environment(wind=wind, w20=prm["turbulence_ms"], origin_alt=origin_alt, seed=seed, rain=rain > 0 or bool(cfg.get("rain", False)), rain_rate=rain,
                      rain_vehicle=VEHICLE, rain_catch_area=CATCH_AREA, rain_wet_area=WET_AREA)  # fmt: skip
    x0, y0, z0 = world.start
    res = compose(
        out,
        [Vehicle(VEHICLE, ASSEMBLY, pos=(x0, y0, z0))],
        env,
        seed=seed,
        floor_size=(0, 0, 1),
        terrain=world.terrain,
        wind_field=wf,
    )
    path = Path(res["model"])
    tree = ET.parse(path)
    root = tree.getroot()
    wb = root.find("worldbody")
    if world.terrain is None:
        _ground_box(wb)
    add_obstacles(wb, world.forest)
    if (
        not world.forest.obstacles
    ):  # the contact sensor needs a geom on the body: a pebble 500 m down
        ET.SubElement(
            wb.find("body[@name='obstacles']"),
            "geom",
            name="obs_none",
            type="sphere",
            size="0.1",
            pos="0 0 -500",
            contype="1",
            conaffinity="1",
            group="4",
        )
    if world.terrain is None:  # (terrain pads are painted by compose)
        for z in world.zones:
            _marker(wb, Pad(z.name, z.x, z.y, z.z, z.radius))
    else:  # the heightfield moves to geom group 2, which only the downward lidar rays (see _devices)
        wb.find("geom[@name='terrain']").set("group", str(TERRAIN_GROUP))
    _sensors(root)
    _devices(root, origin_alt, world.terrain is not None)
    ET.indent(root)
    tree.write(path, encoding="unicode")
    land = dict(scenario=sc.get("name", ""), family=fam, seed=seed, params=prm, cfg=cfg, wind=list(wind), wind_az_deg=az, origin_alt=origin_alt, rain_mm_h=rain,
                mission=world.mission(), truth=world.to_json(), use_rtk=bool(cfg.get("rtk", False)), terrain_following=bool(cfg.get("terrain_following", False)),
                gusts=gusts)  # fmt: skip
    (Path(out) / "land.json").write_text(json.dumps(land))
    (Path(out) / "forest.json").write_text(json.dumps(world.forest.to_json()))
    return path


ASV_PAD_OFF = (
    -0.15,
    0.0,
    0.505,
)  # the ASV landing plate's top surface centre in the hull frame (examples/asv/scene.py add_landing_pad)
ASV_BASE_OFF = (
    -0.5,
    0.0,
    0.9,
)  # ESTIMATE: a ZED-F9P base antenna on a short mast aft; the ASV design has no RTK base (the OSV's is on its wheelhouse mast)


def _asv_scene(out, seed, hs, wave_dir_deg, wind, w20):
    """The 1.2 m ASV with its landing plate and the Skydio standing on it, in a seeded sea of significant height `hs` (calm: 0.1-0.3 m). -> (model path, pad top centre world)."""
    sys.path.insert(0, str(HERE.parents[0]))  # examples/ (package asv)
    import asv.encounters as enc
    import asv.scene as asv_scene

    # compose() wants one timestep: the ASV design runs at 2 ms, the Skydio at 1 ms. A copy of the ASV design at 1 ms (same parts, wires, devices) goes next to the scene.
    asv1 = Path(out) / "asv-1ms.toml"
    Path(out).mkdir(parents=True, exist_ok=True)
    src = (HERE.parents[1] / "assemblies" / "asv.toml").read_text()
    assert "timestep = 0.002" in src
    asv1.write_text(src.replace("timestep = 0.002", "timestep = 0.001"))
    res = compose(
        out,
        [
            Vehicle("asv", str(asv1), pos=(0.0, 0.0, 0.0)),
            Vehicle(VEHICLE, ASSEMBLY, pos=(0.0, 0.0, 0.0)),
        ],
        env=Environment(wind=wind, w20=w20, seed=seed),
        seed=seed,
        visual=True,
    )
    path = Path(res["model"])
    root = ET.parse(path).getroot()
    wb = root.find("worldbody")
    hull = next(b for b in wb.findall("body") if b.get("name") == "asv_hull")
    hz = float(hull.get("pos", "0 0 0").split()[2])
    pad = np.array(
        [ASV_PAD_OFF[0], ASV_PAD_OFF[1], hz + ASV_PAD_OFF[2]]
    )  # hull yaw 0, level at tick 0
    for b in wb.findall("body"):  # the lander stands on the plate (2 mm clearance)
        if b.get("name", "").startswith(VEHICLE + "_"):
            p = [float(x) for x in b.get("pos", "0 0 0").split()]
            b.set(
                "pos",
                " ".join(f"{x:.9g}" for x in (pad[0], pad[1], pad[2] + 0.002 + p[2])),
            )
    asv_scene.add_landing_pad(hull)
    txt = root.find("custom/text")
    d = json.loads(txt.get("data"))
    sea = next(s for s in d["seas"] if s["name"] == "asv_sea")
    sea.pop("jonswap", None)
    if hs > 0:
        sea["jonswap"] = dict(
            hs=hs,
            tp=enc.peak_period(hs),
            direction=math.radians(wave_dir_deg),
            seed=1000 + seed,
        )
    d["seas"] = [sea]
    txt.set("data", json.dumps(d, sort_keys=True, separators=(",", ":")))
    for g in wb.findall("geom"):
        if g.get("name") == "floor":
            g.set("pos", "0 0 -40")
            g.set("rgba", "0.30 0.27 0.20 1")
    ET.SubElement(
        wb,
        "geom",
        name="water",
        type="plane",
        size="400 400 0.1",
        pos="0 0 0",
        rgba=asv_scene.WATER_RGBA,
        group="2",
        contype="0",
        conaffinity="0",
    )
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path, pad


def build_deck(out, cfg, prm, sc):
    """Deck families `deck_osv` (the 12.6 m support vessel under way, Hs `hs_m`, `ship_speed_ms`) and `deck_asv` (the 1.2 m ASV in calm water): the scene of napl.cuas.ship.ship_scene /
    examples/asv with the landing program's contact sensors (gear vs the hull) and mission (one zone: the deck)."""
    import os
    import tempfile

    from napl.cuas.ship import PAD_CENTRE, POINTS, ship_scene

    # the first build of the vessel (or the ASV) routes its wires: minutes; the route cache makes every later build seconds
    os.environ.setdefault(
        "NAPL_ROUTE_CACHE",
        str(Path(tempfile.gettempdir()) / "napl-route-cache-skyland"),
    )
    fam, seed = cfg["family"], prm["seed"]
    kind = fam.split("_", 1)[1]
    wind, az = wind_vector(
        seed,
        prm["wind_ms"],
        cfg.get("wind_az_deg"),
        float(cfg.get("wind_jitter_deg", 0.0)),
    )
    if kind == "osv":
        r = ship_scene(
            out,
            lander=ASSEMBLY,
            lander_start="deck",
            hs=prm["hs_m"],
            wave_dir=prm["wave_dir_deg"],
            wind=wind,
            w20=prm["turbulence_ms"],
            seed=seed,
            ship_yaw=0.0,
        )
        path, pad0 = Path(r["model"]), np.asarray(r["pad"], float)
        deck = dict(
            hull="osv_hull",
            prefix="osv",
            pad_off=list(PAD_CENTRE),
            base_off=list(POINTS["rtk_base"]),
            half=[1.5, 1.5],
            margin=0.25,
            pad_radius=1.5,
        )
    else:
        path, pad0 = _asv_scene(
            out, seed, prm["hs_m"], prm["wave_dir_deg"], wind, prm["turbulence_ms"]
        )
        deck = dict(
            hull="asv_hull",
            prefix="asv",
            pad_off=list(ASV_PAD_OFF),
            base_off=list(ASV_BASE_OFF),
            half=[0.375, 0.45],
            margin=0.10,
            pad_radius=0.4,
        )
    tree = ET.parse(path)
    root = tree.getroot()
    wb = root.find("worldbody")
    add_obstacles(wb, W.F.Forest(seed, "goal"))
    ET.SubElement(
        wb.find("body[@name='obstacles']"),
        "geom",
        name="obs_none",
        type="sphere",
        size="0.1",
        pos="0 0 -500",
        contype="1",
        conaffinity="1",
        group="4",
    )
    _sensors(root, ground=deck["hull"])
    _devices(root, 0.0)
    ET.indent(root)
    tree.write(path, encoding="unicode")
    world = W.deck_world(
        fam, seed, (float(pad0[0]), float(pad0[1]), float(pad0[2])), deck["pad_radius"]
    )
    deck.update(kind=kind, t_go=float(cfg.get("t_go_s", 14.0)), ship_speed_ms=float(prm.get("ship_speed_ms", 0.0)), ship_heading_deg=float(cfg.get("ship_heading_deg", 0.0)),
                t_wait_s=float(cfg.get("t_wait_s", 45.0)), wait_max_s=float(cfg.get("wait_max_s", 60.0)))  # fmt: skip
    land = dict(scenario=sc.get("name", ""), family=fam, seed=seed, params=prm, cfg=cfg, wind=list(wind), wind_az_deg=az, origin_alt=0.0, rain_mm_h=0.0, mission=world.mission(),
                truth=world.to_json(), use_rtk=False, terrain_following=False, gusts=[], deck=deck)  # fmt: skip
    (Path(out) / "land.json").write_text(json.dumps(land))
    (Path(out) / "forest.json").write_text(json.dumps(world.forest.to_json()))
    return path


def build_for(out, script_file):
    """The model script of a scenario `<name>.scenario.toml` is `<name>.py` beside it: build(out) = build_for(out, __file__)."""
    return build(out, Path(script_file).with_suffix("").with_suffix(".scenario.toml"))


def _ground_box(wb, half=500.0):
    """The ground as a 1 km x 1 km slab instead of compose's infinite plane. mj_multiRay (the rangefinder's ray cast, with a cutoff = max_range) culls a geom by its
    bounding sphere, and an infinite plane has radius 0 at the world origin: a lidar or depth camera more than max_range (8 m) from the origin never sees the plane,
    however low it flies. (The original Skydio demo never noticed: its depth camera ignores the floor.)"""
    for g in [g for g in wb.findall("geom") if g.get("type") == "plane"]:
        wb.remove(g)
    wb.insert(
        0,
        ET.Element(
            "geom",
            name="floor",
            type="box",
            size=f"{half} {half} 1",
            pos="0 0 -1",
            contype="1",
            conaffinity="1",
            group="0",
            rgba="0.36 0.42 0.30 1",
        ),
    )


def _sensors(root, ground="world"):
    """`ground`: the body the lander lands on (the world body, or a hull on the decks: its whole subtree, the pad and the deck fittings are child bodies)."""
    g2 = {"body2": ground} if ground == "world" else {"subtree2": ground}
    s = root.find("sensor")
    if s is None:
        s = ET.SubElement(root, "sensor")
    ET.SubElement(
        s,
        "contact",
        name="hit_obstacles",
        subtree1="skydio_bottom",
        body2="obstacles",
        data="found",
        num="1",
    )
    ET.SubElement(
        s,
        "contact",
        name="hit_ground",
        subtree1="skydio_bottom",
        **g2,
        data="found",
        num="1",
    )
    for leg in ("f", "r"):
        ET.SubElement(
            s,
            "contact",
            name=f"hit_gear_{leg}",
            body1=f"skydio/leg_{leg}",
            **g2,
            data="found",
            num="1",
        )


def _devices(root, origin_alt, terrain=False):
    """Dryden turbulence at the height of the airframe (ref_body), the GNSS altitude origin at the site's altitude. On terrain the depth camera does not ray the heightfield
    (geom group TERRAIN_GROUP): MuJoCo's ray-heightfield test walks the grid cells under the ray, and 14 400 rays x 30 Hz against a 400 x 290 grid run ~250x slower than
    real time (measured: 8 s per 0.1 s of simulated time). The vehicle sees the ground below it with the lidar, and boulders (ordinary geoms) with the camera."""
    txt = root.find("custom/text[@name='napl.devices']")
    d = json.loads(txt.get("data"))
    if terrain:
        for r in d.get("rangefinders", []):
            if r["name"] == f"{VEHICLE}_depth":
                r["ray_groups"] = 47 & ~(
                    1 << TERRAIN_GROUP
                )  # design.devices.RAY_GROUPS without the terrain group
    for e in d.get("environments", []):
        e["ref_body"] = f"{VEHICLE}_bottom"
    for g in d.get("gnss_receivers", []):
        g["alt0"] = origin_alt
    txt.set("data", json.dumps(d, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    print(build(sys.argv[2] if len(sys.argv) > 2 else "scene_out", sys.argv[1]))
