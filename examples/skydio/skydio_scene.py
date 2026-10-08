"""Scene of the Skydio-style demo: assemblies/skydio.toml composed (python/napl/cuas/scene.py) with the seeded obstacle field of forest.py and, in the
`follow` mission, a walking subject. `napl program` calls build(outdir) -> path of the MJCF (scenario [model] script).

Environment: SKYDIO_MISSION = goal | follow (default goal); the seed (`seed()` below: NAPL_PARAMS "seed", else SKYDIO_SEED, else 1) seeds the forest,
the wind turbulence and the sensor noise.

Obstacles are static geoms of one body, `obstacles`: contype / conaffinity 1 (they collide with the airframe and the subject) and in geom group 0
(the depth camera's rays see group 0-4, so they are visible to the rangefinder). A `contact` sensor on the airframe's subtree counts its contacts with
that body, so "no collision" is measured by the physics, not by a clearance proxy.

The subject is a 1.75 m person-sized figure on a planar slide-slide-hinge carrier (three joints), driven by velocity actuators `subject_vx`,
`subject_vy`, `subject_wz` from a pure-pursuit controller (fly.py) along forest.path. It collides with the trees (the path keeps 1.1 m from them)."""

import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from forest import generate
from napl import params as nparams
from napl.cuas.scene import Environment, Vehicle, compose

WIND = (
    0.8,
    0.4,
    0.0,
)  # m/s steady, plus Dryden turbulence W20 below: a light breeze in a forest
W20 = 1.0
SUBJECT_HEIGHT = 1.75
FOLLOW_CAM_LOWER_DEG = (
    18.0  # follow mission: camera 10 deg up (assembly) -> 8 deg down, see tilt_camera
)


def mission():
    return os.environ.get("SKYDIO_MISSION", "goal")


def seed(mission_name=None):
    """The one seed of a run (forest, wind turbulence, sensor noise, detector misses), read the same way by this scene builder and by fly.py:
    NAPL_PARAMS {"seed": n} (napl.params; what a UI or `napl program` caller sets) > SKYDIO_SEED (standalone runs, montecarlo.py) > the `seed`
    parameter's default in <mission>.scenario.toml (1)."""
    sc = HERE / f"{mission_name or mission()}.scenario.toml"
    if "seed" in nparams.env_overrides():
        return nparams.values(sc)["seed"]
    if "SKYDIO_SEED" in os.environ:
        return int(os.environ["SKYDIO_SEED"])
    return nparams.resolve(sc)["seed"]


def _f(v):
    return " ".join(f"{x:.6g}" for x in v)


def add_obstacles(worldbody, forest):
    body = ET.SubElement(worldbody, "body", name="obstacles")
    for o in forest.obstacles:
        common = dict(
            name=f"obs_{o.name}",
            contype="1",
            conaffinity="1",
            group="0",
            rgba=_f(o.rgba),
        )
        if o.kind == "cyl":
            ET.SubElement(
                body, "geom", type="cylinder", size=_f(o.size), pos=_f(o.pos), **common
            )
        elif o.kind == "sphere":
            ET.SubElement(
                body, "geom", type="sphere", size=_f(o.size), pos=_f(o.pos), **common
            )
        else:
            c, s = (
                math.cos(o.yaw / 2),
                math.sin(o.yaw / 2),
            )
            ET.SubElement(
                body,
                "geom",
                type="box",
                size=_f(o.size),
                pos=_f(o.pos),
                quat=_f([c, 0, 0, s]),
                **common,
            )


def add_subject(root, start):
    wb = root.find("worldbody")
    b = ET.SubElement(wb, "body", name="subject", pos=_f([start[0], start[1], 0.02]))
    for n, ax in (("subject_x", "1 0 0"), ("subject_y", "0 1 0")):
        ET.SubElement(b, "joint", name=n, type="slide", axis=ax)
    ET.SubElement(b, "joint", name="subject_yaw", type="hinge", axis="0 0 1")
    ET.SubElement(b, "inertial", pos="0 0 0.9", mass="70", diaginertia="12 12 1.5")
    g = dict(contype="1", conaffinity="1", group="0")
    for s in (-1, 1):  # legs
        ET.SubElement(
            b,
            "geom",
            name=f"subject_leg{s}",
            type="capsule",
            fromto=_f([0, s * 0.1, 0.12, 0, s * 0.1, 0.85]),
            size="0.09",
            rgba="0.12 0.14 0.22 1",
            **g,
        )
    ET.SubElement(
        b,
        "geom",
        name="subject_torso",
        type="capsule",
        fromto="0 0 0.95 0 0 1.42",
        size="0.19",
        rgba="0.95 0.45 0.08 1",
        **g,
    )
    for s in (-1, 1):  # arms
        ET.SubElement(
            b,
            "geom",
            name=f"subject_arm{s}",
            type="capsule",
            fromto=_f([0, s * 0.27, 1.38, 0, s * 0.28, 0.95]),
            size="0.055",
            rgba="0.95 0.45 0.08 1",
            **g,
        )
    ET.SubElement(
        b,
        "geom",
        name="subject_head",
        type="sphere",
        pos="0 0 1.63",
        size="0.11",
        rgba="0.85 0.65 0.5 1",
        **g,
    )
    act = (
        root.find("actuator")
        if root.find("actuator") is not None
        else ET.SubElement(root, "actuator")
    )
    for n, j, kv, lim in (
        ("subject_vx", "subject_x", 3000, 6),
        ("subject_vy", "subject_y", 3000, 6),
        ("subject_wz", "subject_yaw", 300, 6),
    ):
        ET.SubElement(
            act,
            "velocity",
            name=n,
            joint=j,
            kv=str(kv),
            ctrllimited="true",
            ctrlrange=f"-{lim} {lim}",
        )


def tilt_camera(root, lower_deg):
    """Lower the D435i's optical axis by `lower_deg` (about the airframe's y axis) in the generated MJCF: the follow mission's mount angle. The assembly
    (assemblies/skydio.toml) has the camera 10 deg UP, which keeps the view level when the quad pitches nose down at 4-5 m/s on the goal mission; following
    a person at 2.4 m/s the quad pitches +-15 deg while accelerating and braking, and with the camera up a braking (nose-up) quad loses the person's
    legs below the 58 deg field of view (detection rate 0.67 at 2-12 deg nose up against 0.95 nose down, flights of seeds 1, 111, 302, 100, 117, 205).
    -8 deg (18 deg lower) keeps head and knees in view over the whole +-15 deg pitch range at 4-7 m."""
    body = next(b for b in root.iter("body") if b.get("name", "").endswith("/cam"))
    w, x, y, z = (float(v) for v in body.get("quat").split())
    c, s = (
        math.cos(math.radians(lower_deg) / 2),
        math.sin(math.radians(lower_deg) / 2),
    )  # left-multiply by Ry(+lower): nose-down rotation of the mount
    q = (c * w - s * y, c * x + s * z, c * y + s * w, c * z - s * x)
    body.set("quat", " ".join(f"{v:.9g}" for v in q))


def build(out, mission_name=None, seed_value=None):
    mission_name, sd = (
        mission_name or mission(),
        seed_value if seed_value is not None else seed(mission_name),
    )
    forest = generate(sd, mission_name)
    res = compose(
        out,
        [Vehicle("skydio", "skydio")],
        Environment(wind=WIND, w20=W20, seed=sd),
        seed=sd,
        floor_size=(0, 0, 1),
    )
    path = Path(res["model"])
    tree = ET.parse(path)
    root = tree.getroot()
    add_obstacles(root.find("worldbody"), forest)
    if mission_name == "follow":
        add_subject(root, forest.path[0])
        tilt_camera(root, FOLLOW_CAM_LOWER_DEG)
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(
        sensors,
        "contact",
        name="hit_obstacles",
        subtree1="skydio_bottom",
        body2="obstacles",
        data="found",
        num="1",
    )
    if mission_name == "follow":
        ET.SubElement(
            sensors,
            "contact",
            name="hit_subject",
            subtree1="skydio_bottom",
            body2="subject",
            data="found",
            num="1",
        )
    ET.indent(root)
    tree.write(path, encoding="unicode")
    (Path(out) / "forest.json").write_text(json.dumps(forest.to_json()))
    return path


if __name__ == "__main__":
    print(build(sys.argv[1] if len(sys.argv) > 1 else "scene_out"))
