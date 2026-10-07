"""The SO-101 pick scene: the built assembly (assemblies/so101.toml, 6 x STS3215) plus a table, jaw contact pads and a PLA cube.

Composed from the built MJCF in a temp dir; the assembly and the parts are not touched. Table top = the base's bottom plane (the
real arm is clamped to a table). 20 mm solid PLA cube (density 1240 kg/m^3, friction 0.40, docs/research/sts3215-calibration.md 5).
Box contact pads on the two jaws (the jaw meshes' collision is off in this scene: MuJoCo collides a mesh as its convex hull, which
for these concave jaws fills the gap the cube has to fit in). Pad faces are 20 mm apart and parallel at gripper angle G_REF.
`build(outdir)` is the hook of examples/so101/pick.scenario.toml (`[model] script`)."""

import functools
import json
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ik
import so101_lib
from ik import G_REF, fk_tcp
from napl import design

REPO = Path(__file__).resolve().parents[2]
ASM, PARTS = REPO / "assemblies" / "so101.toml", REPO / "parts"
CUBE = 0.020  # m, edge
CUBE_DENSITY, MU = (
    1240.0,
    0.40,
)  # PLA; PLA on PLA sliding friction (research 5.2, an assumption)
FRICTION = f"{MU} 0.005 0.0001"  # sliding, torsional (m), rolling (m): condim 4 gives the cube a torsional grip
PAD_HALF = (0.002, 0.007, 0.008)
FIXED_PAD_TCP = (
    0.002,
    0.0,
    -0.008,
)  # pad centres in the TCP frame (x = jaw opening direction, z = out of the jaw tip)
MOVING_PAD_TCP = (-0.022, 0.0, -0.008)
CUBE_XY, TARGET_XY = (0.20, -0.05), (0.20, 0.04)  # 9 cm apart


@functools.cache
def built(out=None, asm=None, parts=None):
    """assemblies/so101.toml (or the saved design `asm`, same instance / joint names) -> (dir, MJCF text, devices dict).
    compute() = build() without the DRC step (as servo_bench)."""
    tmp = Path(tempfile.mkdtemp(prefix="so101_pick_")) if out is None else Path(out)
    tmp.mkdir(parents=True, exist_ok=True)
    text, _ = design.compute(asm or ASM, parts or PARTS, tmp)
    root = ET.fromstring(text)
    dev = json.loads(root.find("custom/text[@name='napl.devices']").get("data"))
    return tmp, text, dev


def _R(q):
    w, x, y, z = np.array(q, float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _f(s, n=None):
    v = np.array([float(x) for x in s.split()])
    assert n is None or len(v) == n
    return v


def mjcf_poses(root, q=None):
    """-> ({body name: 4x4 world pose}, {joint name: prim path}) of the MJCF at joint angles q ({joint: rad}, default 0)."""
    q, poses, paths = q or {}, {}, {}

    def walk(b, T, path):
        loc = np.eye(4)
        loc[:3, :3] = _R(_f(b.get("quat", "1 0 0 0"), 4))
        loc[:3, 3] = _f(b.get("pos", "0 0 0"), 3)
        T = T @ loc
        path = f"{path}/{b.get('name')}"
        for j in b.findall("joint"):
            paths[j.get("name")] = f"{path}/{j.get('name')}"
            Rj, p = (
                ik._rot(_f(j.get("axis"), 3), q.get(j.get("name"), 0.0)),
                _f(j.get("pos"), 3),
            )
            Tj = np.eye(4)
            Tj[:3, :3], Tj[:3, 3] = Rj, p - Rj @ p
            T = T @ Tj
        poses[b.get("name")] = T
        for c in b.findall("body"):
            walk(c, T, path)

    for b in root.find("worldbody").findall("body"):
        walk(b, np.eye(4), "/world")
    return poses, paths


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


def scene(cube_xy=None, payload_kg=None, out=None, asm=None, parts=None):
    """-> (xml path, table top z). The built MJCF plus table, jaw pads, and (optionally) the cube at cube_xy; or, with payload_kg
    given (0 = none), no table and a payload mass fixed at the TCP (weld = a collision-free box geom in the fixed jaw body)."""
    tmp, text, _ = built(out, asm, parts)
    root = ET.fromstring(text)
    poses, _ = mjcf_poses(root)
    wb = root.find("worldbody")
    # table top = the base's bottom plane
    base = poses["base"]
    mesh = {m.get("name"): m.get("file") for m in root.iter("mesh")}
    g = next(g for g in root.iter("geom") if g.get("name") == "base")
    v = so101_lib.read_stl((tmp / mesh[g.get("mesh")]).resolve()).reshape(-1, 3)
    zt = float((v @ base[:3, :3].T + base[:3, 3])[:, 2].min())
    arm_geom = next(g for g in root.iter("geom") if g.get("name") == "fixed_jaw")
    grp = arm_geom.get("group", "0")
    for g in root.iter("geom"):
        if g.get("name") == "floor":
            g.set("pos", "0 0 -1")  # a far safety floor: the table is the surface
        if g.get("name") in ("fixed_jaw", "moving_jaw"):
            g.set("contype", "0")  # the pads below replace the jaw hulls in this scene
    if (
        payload_kg is None
    ):  # the payload sweep holds the arm in free air (no table to rest on)
        _geom(
            wb,
            name="table",
            type="box",
            size="0.35 0.35 0.02",
            pos=_vec([0.1, 0, zt - 0.02]),
            contype=1,
            conaffinity=1,
            friction=FRICTION,
            condim=4,
            rgba="0.6 0.5 0.4 1",
        )
    # pads, defined in the TCP frame at the reference pose (arm angles 0, gripper G_REF)
    T_tcp0 = fk_tcp(np.zeros(5))
    ref, _ = mjcf_poses(root, {"gripper": G_REF})
    bodies = {b.get("name"): b for b in root.iter("body")}
    for name, c in (("fixed_jaw", FIXED_PAD_TCP), ("moving_jaw", MOVING_PAD_TCP)):
        T = _inv(ref[name]) @ T_tcp0 @ _tf(c)
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
    if payload_kg:
        pos = (_inv(ref["fixed_jaw"]) @ T_tcp0)[:3, 3]
        pb = ET.SubElement(bodies["fixed_jaw"], "body", name="payload", pos=_vec(pos))
        _geom(
            pb,
            name="payload",
            type="box",
            size="0.01 0.01 0.01",
            mass=repr(float(payload_kg)),
            contype=0,
            conaffinity=0,
        )
    path = tmp / f"scene_{os.getpid()}_{int(time.time() * 1e6)}.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path, zt


def build(outdir, design=None, parts=None):
    """Scenario hook: the pick scene (table, pads, cube at CUBE_XY) written under outdir -> MJCF path. With a scenario [design]
    the arm is that saved design (it must use so101.toml's instance and joint names) instead of assemblies/so101.toml."""
    path, zt = scene(cube_xy=CUBE_XY, out=outdir, asm=design, parts=parts)
    print(f"[scene] table top z = {zt:.6f} m", file=sys.stderr)
    return path


if __name__ == "__main__":  # print the workcell constants pick.py hardcodes
    _, zt = scene(cube_xy=CUBE_XY)
    print(f"TABLE_Z = {zt!r}  PID_SCALE = {built()[2]['servos'][0]['pid_scale']!r}")
