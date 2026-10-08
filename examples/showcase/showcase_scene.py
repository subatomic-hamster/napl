"""N7 showcase scene: the real-parts SO-101 arm on a fixed shelf beside the real-parts X500 V2 (both built by napl.design from
assemblies/so101.toml and assemblies/x500v2.toml), one MJCF, one World, a stainless payload cube on the shelf. `build(outdir)` is the
hook of arm_loads_drone.scenario.toml (`[model] script`); NAPL_SHOWCASE_PAYLOAD=0 builds the same scene without the payload (the
unloaded control flight of fly_unloaded.scenario.toml).

Geometry (all derived from the built models, nothing hand-placed except R_PICK / R_PLACE):
  - the drone stands at the origin on its landing gear, nose = +x. Its top plate (a 143.7 mm square, carbon fibre) carries the
    flight controller at the centre and the SiK radio on the +y side; the free strip is the -y side, between the plate edge and
    the flight controller. The payload goes to the middle of that strip.
  - the arm base is at x = 0 on the -y side, yawed 90 degrees so the arm's +x (forward) points at the drone: base-to-place distance
    R_PLACE = 0.25 m (the arm's top-down reach at this height, tools: ik.ik; 0.28 m is out of reach), pick R_PICK = 0.16 m on the
    same line. The base bottom plane is at the top-plate height, so pick and place are at the same level.
  - the arm stands on a shelf (top = base bottom plane = top-plate height) over a pillar at its back: the shelf front end stops 24 mm
    short of the pick point, clear of the landing gear and the plate. Everything stays inside |x| < 55 mm, the corridor between the
    two right-hand rotor discs (disc centres (+-0.177, -0.177) m, blade radius 0.127 m), and below the blade plane (10 mm gap), so
    the drone can lift off through its own footprint.
Collision flags in the built models: drone geoms contype 1 / conaffinity 0, arm links the same, so the arm and the drone do not collide
with each other (the drone lifts clear of the parked arm anyway: the arm parks over the shelf, 0.14 m away from the payload); the
payload and the shelf have conaffinity 1 and do collide with everything.
"""

import json
import os
import sys
import tempfile
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "so101"))
sys.path.insert(0, str(REPO / "tools"))
import scene as so101  # examples/so101/scene.py: the SO-101 + jaw pads scene
from napl import design

X500 = REPO / "assemblies" / "x500v2.toml"
PARTS = REPO / "parts"
YAW = np.pi / 2  # arm +x -> world +y (towards the drone)
R_PICK, R_PLACE = 0.16, 0.25  # m from the arm's pan axis along its forward line
SHELF_HALF_X, SHELF_T, PILLAR = (
    0.056,
    0.012,
    0.05,
)  # shelf half width / thickness, pillar half width under the base back
PAYLOAD_PART = "stainless-304"  # parts/common/stainless-304.toml, density 7889 kg/m3 (Proto Labs data sheet)
CUBE = so101.CUBE  # 20 mm: the jaw pads are 20 mm apart


def _q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])  # fmt: skip


def _g(root, name):
    return next(g for g in root.iter("geom") if g.get("name") == name)


def _v(s):
    return np.array([float(x) for x in s.split()])


def _vec(v):
    return " ".join(f"{x:.9g}" for x in v)


def payload_density():
    return float(
        tomllib.loads((PARTS / "common" / f"{PAYLOAD_PART}.toml").read_text())[
            "params"
        ]["density"]["value"]
    )


def layout(x500_root):
    """-> dict of the placement numbers (world frame, m) from the built X500 MJCF."""
    poses, _ = so101.mjcf_poses(x500_root)
    top, fc = poses["top"], poses["fc"]
    assert np.allclose(top[:3, :3], np.eye(3), atol=1e-6) and np.allclose(
        fc[:3, :3], np.eye(3), atol=1e-6
    ), "plate not axis-aligned"
    ts, fs = _v(_g(x500_root, "top").get("size")), _v(_g(x500_root, "fc").get("size"))
    z_plate = top[2, 3] + ts[2]  # top face of the top plate
    y_edge, y_fc = (
        top[1, 3] - ts[1],
        fc[1, 3] - fs[1],
    )  # -y plate edge, -y face of the flight controller
    y_place = (y_edge + y_fc) / 2  # middle of the free strip
    x_c = top[0, 3]
    return dict(z_plate=z_plate, x=x_c, y_place=y_place, y_base=y_place - R_PLACE, y_pick=y_place - R_PLACE + R_PICK,
                strip=(y_edge, y_fc), root_z=poses["bottom"][2, 3])  # fmt: skip


def merge_devices(a, b):
    """x500 device dict `a` + arm device dict `b`: every arm device name gets the prefix `arm_`, and its electrical nodes move above
    the drone's (node numbers are one shared namespace, ground 0 stays 0); wires of both lists concatenate."""
    top = max([d.get("node", 0) for k in a.values() if isinstance(k, list) for d in k]
              + [n for k in a.values() if isinstance(k, list) for d in k for n in (d.get("a", 0), d.get("b", 0))])  # fmt: skip
    out = {k: (list(v) if isinstance(v, list) else v) for k, v in a.items()}
    names = {d["name"] for k in a.values() if isinstance(k, list) for d in k}
    for k, v in b.items():
        if not isinstance(v, list):
            continue  # `seed`: the drone's
        for d in v:
            d = dict(d, name="arm_" + d["name"])
            assert d["name"] not in names, f"device name collision: {d['name']}"
            for f in ("node", "a", "b"):
                if d.get(f, 0) > 0:
                    d[f] += top
            out.setdefault(k, []).append(d)
    nodes = [
        d[f]
        for k in out.values()
        if isinstance(k, list)
        for d in k
        for f in ("node", "a", "b")
        if f in d
    ]
    assert max(nodes) <= 32, f"{max(nodes)} electrical nodes: the kernel supports 32"
    return out


def build(outdir, payload=None):
    """The composed scene -> MJCF path under outdir (the meshes of the arm are referenced relative to it)."""
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    if payload is None:
        payload = os.environ.get("NAPL_SHOWCASE_PAYLOAD", "1") != "0"
    text, _ = design.compute(X500, PARTS, out)
    x5 = ET.fromstring(text)
    L = layout(x5)
    arm_xml, zt = so101.scene(
        payload_kg=0, out=out
    )  # arm + jaw pads, no table / cube; mesh paths relative to `out`
    arm = ET.parse(arm_xml).getroot()
    wb, awb = x5.find("worldbody"), arm.find("worldbody")
    # arm placement: world = Rz(YAW) p + (x, y_base, z_plate - zt)   (zt = the arm base's bottom plane in the arm frame)
    base = awb.find("body")
    qz = np.array([np.cos(YAW / 2), 0, 0, np.sin(YAW / 2)])
    c, s = np.cos(YAW), np.sin(YAW)
    p = _v(base.get("pos"))
    pw = np.array(
        [
            L["x"] + c * p[0] - s * p[1],
            L["y_base"] + s * p[0] + c * p[1],
            L["z_plate"] - zt + p[2],
        ]
    )
    base.set("pos", _vec(pw))
    base.set("quat", _vec(_q_mul(qz, _v(base.get("quat", "1 0 0 0")))))
    # names must not collide (the arm's floor is dropped: the drone's is the ground)
    names = lambda r: {
        e.get("name")
        for t in ("body", "geom", "joint", "site")
        for e in r.iter(t)
        if e.get("name")
    }
    awb.remove(next(g for g in awb.findall("geom") if g.get("name") == "floor"))
    # cable tubes (visual-only tendons through "wire:" sites): the arm's get an arm_ prefix, then join the drone's <tendon>
    for e in arm.iter("site"):
        if e.get("name", "").startswith("wire:"):
            e.set("name", "arm_" + e.get("name"))
    if arm.find("tendon") is not None:
        for e in arm.find("tendon").iter():
            for a in ("name", "site"):
                if e.get(a, "").startswith("wire:"):
                    e.set(a, "arm_" + e.get(a))
        tn = x5.find("tendon")
        if tn is None:
            tn = ET.SubElement(x5, "tendon")
        tn.extend(list(arm.find("tendon")))
    clash = names(x5) & names(arm)
    assert not clash, f"MJCF name collision: {sorted(clash)}"
    wb.append(base)
    if arm.find("asset") is not None:
        # one <asset>: the same visual mesh / material (content-hashed / named) in both builds is shared, a different one is a clash
        a5 = x5.find("asset")
        if a5 is None:
            x5.insert(list(x5).index(wb), arm.find("asset"))
        else:
            have = {(e.tag, e.get("name")): e.attrib for e in a5}
            for e in arm.find("asset"):
                k = (e.tag, e.get("name"))
                assert k not in have or have[k] == e.attrib, f"MJCF asset clash: {k}"
                if k not in have:
                    a5.append(e)
    # shelf: top = base bottom plane = top-plate height; pillar under the back of the base
    zp, xc, yb, y_pick = L["z_plate"], L["x"], L["y_base"], L["y_pick"]
    y0, y1 = (
        yb - 0.095,
        y_pick + 0.0 + CUBE / 2 + 0.014,
    )  # back end / front end (24 mm past the pick point's centre)
    flags = dict(contype=1, conaffinity=1, friction=so101.FRICTION, condim=4)
    for name, hx, (lo, hi, zlo, zhi) in (
        ("shelf", SHELF_HALF_X, (y0, y1, zp - SHELF_T, zp)),
        ("pillar", PILLAR, (y0, y0 + 0.07, 0.0, zp - SHELF_T)),
    ):
        ET.SubElement(wb, "geom", {"name": name, "type": "box", "rgba": "0.55 0.55 0.58 1", **{k: str(v) for k, v in flags.items()},
                                   "size": _vec([hx, (hi - lo) / 2, (zhi - zlo) / 2]), "pos": _vec([xc, (lo + hi) / 2, (zlo + zhi) / 2])})  # fmt: skip
    # payload: a solid stainless 20 mm cube on the shelf at the pick point
    if payload:
        pb = ET.SubElement(
            wb, "body", name="payload", pos=_vec([xc, y_pick, zp + CUBE / 2 + 1e-4])
        )
        ET.SubElement(pb, "freejoint", name="payload_free")
        ET.SubElement(pb, "geom", {"name": "payload", "type": "box", "size": _vec([CUBE / 2] * 3), "density": str(payload_density()),
                                   "rgba": "0.58 0.60 0.62 1", **{k: str(v) for k, v in flags.items()}})  # fmt: skip
    # devices: one napl.devices text, arm devices prefixed
    tx = x5.find("custom/text[@name='napl.devices']")
    dev = merge_devices(
        json.loads(tx.get("data")),
        json.loads(arm.find("custom/text[@name='napl.devices']").get("data")),
    )
    tx.set("data", json.dumps(dev, sort_keys=True, separators=(",", ":")))
    x5.set("model", "x500v2_so101_showcase")
    path = out / "showcase.xml"
    path.write_text(ET.tostring(x5, encoding="unicode"))
    print(f"[scene] {L}", file=sys.stderr)
    return path


if __name__ == "__main__":  # print the placement numbers
    d = Path(tempfile.mkdtemp(prefix="showcase_"))
    p = build(d)
    root = ET.parse(p).getroot()
    print(p)
    print(
        json.dumps(
            layout(ET.fromstring(design.compute(X500, PARTS, d / "x")[0])),
            indent=1,
            default=float,
        )
    )
    print("payload kg", payload_density() * CUBE**3)
