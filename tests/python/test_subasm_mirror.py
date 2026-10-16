"""Mirrored sub-assemblies: pose equality with the hand mirror, the unmirrorable-part DRC error, mirror_signs."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest
from napl import design
from napl.design import DesignError, Library
from napl.design import assembly as A
from napl.design.drc import check
from test_subasm import LEG, make_parts, write

M = np.diag([1.0, -1, 1])  # mirror across xz
S = np.diag([1.0, -1, 1])  # the test parts' local symmetry plane (xz)
LEGJ = LEG.replace(
    'b = "mot.in"', 'b = "mot.in"\n    joint = { type = "hinge" }'
)  # abduction-like hinge between bracket and motor


LEGA = LEGJ.replace('part = "brk"', 'part = "brk2"')  # bracket with an off-plane port


def quat_R(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def poses(xml):
    """{body: (R, p)} in the plate's frame."""
    out = {}

    def walk(el, R, p):
        for b in el.findall("body"):
            q = [float(v) for v in b.get("quat", "1 0 0 0").split()]
            Rw = R @ quat_R(q)
            pw = R @ np.array([float(v) for v in b.get("pos", "0 0 0").split()]) + p
            out[b.get("name")] = (Rw, pw)
            walk(b, Rw, pw)

    walk(ET.fromstring(xml).find("worldbody"), np.eye(3), np.zeros(3))
    R0, p0 = out["plate"]
    return {k: (R0.T @ R, R0.T @ (p - p0)) for k, (R, p) in out.items()}


def parent(tmp_path, leg=LEGJ, r_extra='mirror = "xz"'):
    write(tmp_path / "leg.toml", leg)
    return write(
        tmp_path / "dog.toml",
        f"""
        [assembly]
        name = "dog"
        [[instances]]
        id = "plate"
        part = "plate"
        root = true
        [[subassemblies]]
        id = "l"
        file = "leg.toml"
        [[subassemblies]]
        id = "r"
        file = "leg.toml"
        {r_extra}
        [[mates]]
        name = "pl"
        a = "plate.mount_l"
        b = "l:mount"
        [[mates]]
        name = "pr"
        a = "plate.mount_r"
        b = "r:mount"
        """,
    )


def test_mirrored_leg_poses_equal_hand_mirror(tmp_path):
    parts = make_parts(tmp_path)
    xml, rep = design.compute(A.load(parent(tmp_path)), parts, tmp_path / "out")
    P = poses(xml)
    for b in ("brk", "mot"):
        (Rl, pl), (Rr, pr) = P[f"l:{b}"], P[f"r:{b}"]
        assert np.allclose(pr, M @ pl, atol=1e-12)
        assert np.allclose(Rr, M @ Rl @ S, atol=1e-12)
        assert np.isclose(np.linalg.det(Rr), 1.0)


def test_unmirrored_twin_is_not_the_mirror(tmp_path):  # the pose comparison can fail
    parts = make_parts(tmp_path)
    xml, _ = design.compute(
        A.load(parent(tmp_path, leg=LEGA, r_extra="")), parts, tmp_path / "out"
    )
    P = poses(xml)
    assert not np.allclose(P["r:mot"][1], M @ P["l:mot"][1], atol=1e-6)


def test_mirror_unsupported_part_is_drc_error(tmp_path):
    parts = make_parts(tmp_path)
    leg = """
        [assembly]
        name = "leg"
        [exports]
        ports = { mount = "nm.base" }
        [[instances]]
        id = "nm"
        part = "nomirror"
        root = true
    """
    p = parent(tmp_path, leg=leg)
    asm = A.load(p)
    fs = [f for f in check(asm, Library(parts)) if f.code == "MIRROR_UNSUPPORTED"]
    assert len(fs) == 1 and "nomirror" in fs[0].message and "r:nm" in fs[0].message
    with pytest.raises(DesignError, match="no \\[mirror\\]"):
        design.compute(asm, parts, tmp_path / "out")


def test_mirror_signs(tmp_path):
    parts = make_parts(tmp_path)
    _, rep = design.compute(A.load(parent(tmp_path)), parts, tmp_path / "out")
    # the hinge axis lies in the mirror plane: the reflection reverses the sense of a turn, so +q on r mirrors -q on l
    assert rep["mirror_signs"] == {"r:brk--mot": -1}


def test_false_symmetry_claim_is_an_error(tmp_path):
    parts = make_parts(
        tmp_path
    )  # brk2 claims xz symmetry but its `out` port sits off the plane
    with pytest.raises(DesignError, match="not the mirror image"):
        design.compute(A.load(parent(tmp_path, leg=LEGA)), parts, tmp_path / "out")
    with pytest.raises(DesignError, match="mirror is one of"):
        A.load(parent(tmp_path, r_extra='mirror = "zz"'))
