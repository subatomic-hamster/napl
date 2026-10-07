"""The Koch v1.1 pick scene: the built assembly (assemblies/koch.toml, 2 x XL430 + 4 x XL330) plus a table, jaw contact pads and a PLA cube.

Composed from the built MJCF in a temp dir; the assembly and the parts are not touched. Table top = the base's bottom plane (the real arm
is clamped to a table). 20 mm solid PLA cube (density 1240 kg/m^3, friction 0.40, as examples/so101/scene.py).
Every mesh geom of the arm has its collision off in this scene (MuJoCo collides a mesh as its convex hull, which for the concave jaws fills
the gap the cube has to fit in); two box pads on the jaws (the pad faces are 20 mm apart and parallel at gripper angle koch_ik.G_REF) are what
touches the cube and the table. `build(outdir)` is the hook of examples/koch/pick.scenario.toml (`[model] script`)."""

import functools
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(Path(__file__).resolve().parent), str(REPO / "tools")]
import so101_lib  # tools/: STL reader
from koch_ik import G_REF, GRASP_L5, _rot, fk_l5
from napl import design

ASM, PARTS = REPO / "assemblies" / "koch.toml", REPO / "parts"
CUBE = 0.020  # m, edge
CUBE_DENSITY, MU = (
    1240.0,
    0.40,
)  # PLA; PLA on PLA sliding friction (an assumption, as the SO-101 scene)
FRICTION = f"{MU} 0.005 0.0001"  # sliding, torsional (m), rolling (m): condim 4 gives the cube a torsional grip
PAD_HALF = (
    0.006,
    0.002,
    0.005,
)  # x: along the finger, y: thickness (jaw opening direction), z: width
# pad centres in the link_5 frame (= the fixed jaw's frame at q = 0): faces at y5 = +0.0095 and -0.0105, the cube centre in between
FIXED_PAD_L5 = GRASP_L5 + [0.0, 0.010 + PAD_HALF[1], 0.0]
MOVING_PAD_L5 = GRASP_L5 - [0.0, 0.010 + PAD_HALF[1], 0.0]
CUBE_XY, TARGET_XY = (
    (-0.12, -0.05),
    (-0.12, 0.05),
)  # 10 cm apart; the arm faces -x (forearm along -x at the URDF zero)


@functools.cache
def built(out=None):
    """assemblies/koch.toml -> (dir, MJCF text). compute() = build() without the DRC step (as servo_bench)."""
    tmp = Path(tempfile.mkdtemp(prefix="koch_pick_")) if out is None else Path(out)
    tmp.mkdir(parents=True, exist_ok=True)
    text, _ = design.compute(ASM, PARTS, tmp)
    return tmp, text


def _R(q):
    w, x, y, z = np.array(q, float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _f(s):
    return np.array([float(x) for x in s.split()])


def mjcf_poses(root, q=None):
    """-> {body name: 4x4 world pose} of the MJCF at joint angles q ({joint: rad}, default 0)."""
    q, poses = q or {}, {}

    def walk(b, T):
        loc = np.eye(4)
        loc[:3, :3] = _R(_f(b.get("quat", "1 0 0 0")))
        loc[:3, 3] = _f(b.get("pos", "0 0 0"))
        T = T @ loc
        for j in b.findall("joint"):
            Rj, p = (
                _rot(_f(j.get("axis")), q.get(j.get("name"), 0.0)),
                _f(j.get("pos")),
            )
            Tj = np.eye(4)
            Tj[:3, :3], Tj[:3, 3] = Rj, p - Rj @ p
            T = T @ Tj
        poses[b.get("name")] = T
        for c in b.findall("body"):
            walk(c, T)

    for b in root.find("worldbody").findall("body"):
        walk(b, np.eye(4))
    return poses


def _inv(T):
    o = np.eye(4)
    o[:3, :3], o[:3, 3] = T[:3, :3].T, -T[:3, :3].T @ T[:3, 3]
    return o


def _tf(p):
    T = np.eye(4)
    T[:3, 3] = p
    return T


def _geom(body, **a):
    return ET.SubElement(body, "geom", {k: str(v) for k, v in a.items()})


def _vec(v):
    return " ".join(f"{x:.9g}" for x in v)


def scene(cube_xy=None, out=None):
    """-> (xml path, table top z). The built MJCF plus table, jaw pads and (optionally) the cube at cube_xy."""
    tmp, text = built(out)
    root = ET.fromstring(text)
    wb = root.find("worldbody")
    # table top = the base's bottom plane (the base body sits at the world origin: the URDF base_link frame)
    mesh = {m.get("name"): m.get("file") for m in root.iter("mesh")}
    g = next(g for g in root.iter("geom") if g.get("name") == "base")
    v = so101_lib.read_stl((tmp / mesh[g.get("mesh")]).resolve()).reshape(-1, 3)
    zt = float(v[:, 2].min())
    for g in root.iter("geom"):
        if g.get("name") == "floor":
            g.set("pos", "0 0 -1")  # a far safety floor: the table is the surface
        elif g.get("type") == "mesh":
            g.set("contype", "0")  # the pads below replace the jaw hulls in this scene
    _geom(
        wb,
        name="table",
        type="box",
        size="0.35 0.35 0.02",
        pos=_vec([-0.1, 0, zt - 0.02]),
        contype=1,
        conaffinity=1,
        friction=FRICTION,
        condim=4,
        rgba="0.6 0.5 0.4 1",
    )
    # pads: defined in the world at q = 0 (link_5 = fk_l5(0)); the moving jaw's at gripper angle G_REF, where its pad is parallel
    T5 = fk_l5(np.zeros(5))
    poses = mjcf_poses(root)
    ref = mjcf_poses(root, {"gripper": G_REF})
    bodies = {b.get("name"): b for b in root.iter("body")}
    grp = next(g for g in root.iter("geom") if g.get("name") == "gripper_static").get(
        "group", "0"
    )
    for name, c, P in (
        ("gripper_static", FIXED_PAD_L5, poses),
        ("gripper_moving", MOVING_PAD_L5, ref),
    ):
        T = _inv(P[name]) @ T5 @ _tf(c)
        _geom(
            bodies[name],
            name=f"{name}_pad",
            type="box",
            size=_vec(PAD_HALF),
            pos=_vec(T[:3, 3]),
            xyaxes=_vec(np.r_[T[:3, 0], T[:3, 1]]),
            mass="1e-4",
            contype=1,
            conaffinity=0,
            friction=FRICTION,
            condim=4,
            group=grp,
        )
    if cube_xy is not None:
        cb = ET.SubElement(
            wb, "body", name="cube", pos=_vec([*cube_xy, zt + CUBE / 2 + 1e-4])
        )
        ET.SubElement(cb, "freejoint", name="cube_free")
        _geom(
            cb,
            name="cube",
            type="box",
            size=_vec([CUBE / 2] * 3),
            density=CUBE_DENSITY,
            contype=1,
            conaffinity=1,
            friction=FRICTION,
            condim=4,
            rgba="0.9 0.2 0.2 1",
        )
    path = tmp / f"scene_{os.getpid()}_{int(time.time() * 1e6)}.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path, zt


def build(outdir):
    """Scenario hook: the pick scene (table, pads, cube at CUBE_XY) written under outdir -> MJCF path."""
    path, zt = scene(cube_xy=CUBE_XY, out=outdir)
    print(f"[scene] table top z = {zt:.6f} m", file=sys.stderr)
    return path


if __name__ == "__main__":  # print the workcell constant pick.py hardcodes
    _, zt = scene(cube_xy=CUBE_XY)
    print(f"TABLE_Z = {zt!r}")
