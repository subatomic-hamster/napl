"""URDF + STL helpers for tools/so101_ports.py (stdlib + numpy)."""

import pathlib
import xml.etree.ElementTree as ET

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
URDF = ROOT / "meshes/so101/so101_new_calib.urdf"


def rot_rpy(r, p, y):
    """URDF fixed-axis rpy: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = (
        np.cos(r),
        np.sin(r),
        np.cos(p),
        np.sin(p),
        np.cos(y),
        np.sin(y),
    )
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def T_from(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rot_rpy(*rpy)
    t[:3, 3] = xyz
    return t


def inv(t):
    o = np.eye(4)
    o[:3, :3] = t[:3, :3].T
    o[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return o


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    t = np.eye(4)
    t[:2, :2] = [[c, -s], [s, c]]
    return t


RX_PI = np.diag([1.0, -1.0, -1.0, 1.0])


def parse_urdf(path=URDF):
    """-> links {name: [(mesh_stem, T_link_visual)]}, joints {name: dict}, in file order."""
    root = ET.parse(path).getroot()
    links = {}
    for ln in root.findall("link"):
        vis = []
        for v in ln.findall("visual"):
            o = v.find("origin")
            xyz = [float(s) for s in o.get("xyz").split()]
            rpy = [float(s) for s in o.get("rpy").split()]
            stem = pathlib.Path(v.find("geometry/mesh").get("filename")).stem
            vis.append((stem, T_from(xyz, rpy)))
        links[ln.get("name")] = vis
    joints = {}
    for jn in root.findall("joint"):
        o = jn.find("origin")
        xyz = [float(s) for s in o.get("xyz").split()]
        rpy = [float(s) for s in o.get("rpy").split()]
        lim = jn.find("limit")
        joints[jn.get("name")] = dict(
            type=jn.get("type"),
            parent=jn.find("parent").get("link"),
            child=jn.find("child").get("link"),
            T=T_from(xyz, rpy),
            axis=[float(s) for s in jn.find("axis").get("xyz").split()],
            lower=None if lim is None else float(lim.get("lower")),
            upper=None if lim is None else float(lim.get("upper")),
            limit_text=None
            if lim is None
            else ET.tostring(lim, encoding="unicode").strip(),
        )
    return links, joints


def read_stl(path):
    """Binary STL -> (n,3,3) float64 triangles."""
    b = pathlib.Path(path).read_bytes()
    n = int(np.frombuffer(b, "<u4", 1, 80)[0])
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(b, dt, n, 84)["v"].astype(np.float64)
