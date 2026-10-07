"""napl.design core: snapping, symmetry, fastener choice, numbers rule, mass formula, MJCF build. Synthetic parts only."""

import json
import math
import struct
import xml.etree.ElementTree as ET

import napl
import numpy as np
import pytest
from napl import design
from napl.design import (
    DesignError,
    Library,
    MateSpec,
    dumps,
    loads,
    mate_transform,
    port_frame,
    solve_tree,
)
from napl.design.assembly import Ctx, _v
from napl.design.build import instance_mass, part_shapes
from napl.design.snap import RX_PI, select_fasteners


# ---- helpers ---------------------------------------------------------------------------------------------------------
def prm(v, unit="kg", fid="datasheet"):
    return {"value": v, "unit": unit, "fidelity": fid, "src": "S1"}


def P(
    name,
    ptype,
    gender="neutral",
    pos=(0, 0, 0),
    axis=(0, 0, 1),
    ref=(1, 0, 0),
    kind="mech",
    **kw,
):
    return {
        "name": name,
        "kind": kind,
        "type": ptype,
        "gender": gender,
        "pos": list(pos),
        "axis": list(axis),
        "ref": list(ref),
        **kw,
    }


def write_part(parts, pid, category="misc", params=None, ports=(), geometry=None):
    L = [
        f'id = "{pid}"',
        'name = "t"',
        f'category = "{category}"',
        'vendor = "t"',
        "",
        "[params]",
    ]
    L += [f"{k} = {_v(e)}" for k, e in (params or {}).items()]
    for p in ports:
        L += ["", "[[ports]]"] + [f"{k} = {_v(v)}" for k, v in p.items()]
    if geometry:
        L += ["", "[geometry]"] + [f"{k} = {_v(v)}" for k, v in geometry.items()]
    parts.mkdir(parents=True, exist_ok=True)
    (parts / f"{pid}.toml").write_text("\n".join(L) + "\n")


def blk(parts, pid, ports=(), mass=0.1, half=(0.05, 0.05, 0.05)):
    write_part(
        parts,
        pid,
        "misc",
        {"mass": prm(mass)},
        ports,
        {"shape": "box", "size": list(half)},
    )


def prov(v, unit, fid="estimated"):
    return {
        "value": v,
        "unit": unit,
        "fidelity": fid,
        **(
            {"rationale": "test"} if fid == "estimated" else {"src": "S1", "quote": "q"}
        ),
    }


def asm(text):
    return loads(text)


def fastener_lib(parts):
    for L in (6, 8, 10, 12, 16):
        write_part(
            parts,
            f"m3x{L}",
            "fastener",
            {
                "mass": prm(1e-3 * L / 10),
                "length": prm(L * 1e-3, "m"),
                "thread_pitch": prm(0.5e-3, "m"),
            },
            [
                P(
                    "thread",
                    "thread.m3",
                    "male",
                    axis=(0, 0, -1),
                    symmetry=360,
                    length=L * 1e-3,
                )
            ],
        )
    for pid, h in (("nut-m3", 2.4e-3), ("nut-m3-thick", 3e-3)):
        write_part(
            parts,
            pid,
            "fastener",
            {
                "mass": prm(2e-4),
                "height": prm(h, "m"),
                "thread_pitch": prm(0.5e-3, "m"),
            },
            [P("thread", "thread.m3", "female", symmetry=360)],
        )
    return Library(parts)


def rodrigues(v, axis, ang):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    return (
        v * math.cos(ang)
        + np.cross(k, v) * math.sin(ang)
        + k * (k @ v) * (1 - math.cos(ang))
    )


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# ---- pattern match ---------------------------------------------------------------------------------------------------
def test_find_compatible_pair_unique_ambiguous_none(tmp_path):
    parts = tmp_path / "parts"
    blk(
        parts,
        "pa",
        [
            P("p1", "bolt-pattern.16x16.m3"),
            P("p2", "bolt-pattern.30x30.m3"),
            P("p3", "shaft.4", "male"),
        ],
    )
    blk(
        parts,
        "pb",
        [P("q1", "bolt-pattern.30x30.m3"), P("q2", "bolt-pattern.20x20.m3")],
    )
    blk(
        parts,
        "pc",
        [P("r1", "bolt-pattern.30x30.m3"), P("r2", "bolt-pattern.30x30.m3")],
    )
    blk(parts, "pd", [P("s1", "tube.12")])
    lib = Library(parts)
    assert design.find_compatible_ports(lib.get("pa"), lib.get("pb")) == [("p2", "q1")]
    base = '[assembly]\nname="t"\n[[instances]]\nid="a"\npart="pa"\nroot=true\n[[instances]]\nid="b"\npart="{}"\n[[mates]]\na="a"\nb="b"\n'
    xml, rep = design.compute(
        asm(base.format("pb")), parts, tmp_path
    )  # unique pair found automatically
    assert rep["assembly"] == "t" and '<body name="b"' in xml
    with pytest.raises(
        DesignError, match=r"ambiguous.*a\.p2 <-> b\.r1.*a\.p2 <-> b\.r2"
    ):
        design.compute(asm(base.format("pc")), parts, tmp_path)
    with pytest.raises(DesignError, match="no compatible"):
        design.compute(asm(base.format("pd")), parts, tmp_path)


# ---- symmetry / gender / axis ----------------------------------------------------------------------------------------
def test_symmetry_rotations_and_gcd():
    a = P("a", "bolt-pattern.16x16.m3", axis=(0, 0, 1), ref=(1, 0, 0), symmetry=4)
    b = P("b", "bolt-pattern.16x16.m3", axis=(0, 0, 1), ref=(1, 0, 0), symmetry=4)
    Ta = np.eye(4)
    for k in range(4):
        Tb = mate_transform(Ta, a, b, k)
        ref_world = (Tb @ port_frame(b))[:3, 0]
        assert np.allclose(
            ref_world,
            rodrigues(np.array([1.0, 0, 0]), (0, 0, 1), k * math.pi / 2),
            atol=1e-12,
        )
    with pytest.raises(DesignError, match="rotation 4"):
        mate_transform(Ta, a, b, 4)
    c = dict(b, symmetry=2)
    assert mate_transform(Ta, a, c, 1) is not None  # gcd(4, 2) = 2 -> k in {0, 1}
    with pytest.raises(DesignError, match="rotation 2"):
        mate_transform(Ta, a, c, 2)


def test_gender_and_type_rules():
    m, f, n = P("m", "t", "male"), P("f", "t", "female"), P("n", "t", "neutral")
    assert mate_transform(np.eye(4), m, f) is not None
    assert mate_transform(np.eye(4), n, n) is not None
    assert mate_transform(np.eye(4), n, m) is not None
    with pytest.raises(DesignError, match="gender"):
        mate_transform(np.eye(4), m, dict(m))
    with pytest.raises(DesignError, match="type"):
        mate_transform(np.eye(4), m, dict(f, type="other"))
    with pytest.raises(DesignError, match="perpendicular"):
        port_frame(P("bad", "t", axis=(0, 0, 1), ref=(1, 0, 1e-3)))
    with pytest.raises(DesignError, match="unit"):
        port_frame(P("bad", "t", axis=(0, 0, 2)))


def test_paired_port_families_shaft_bore_tube_clamp(tmp_path):
    from napl.design.library import types_ok

    assert types_ok("shaft.3", "bore.3") and types_ok("bore.3", "shaft.3")
    assert types_ok("tube.16", "tube-clamp.16") and types_ok("tube-clamp.16", "tube.16")
    assert types_ok("tube.16", "tube.16")
    assert not types_ok("shaft.3", "bore.5")  # sizes must agree
    assert not types_ok("shaft.3", "tube-clamp.3")  # only the named pairs
    assert not types_ok("shaft", "bore")  # a size suffix is required
    sh, bo = P("s", "shaft.3", "male"), P("b", "bore.3", "female")
    assert mate_transform(np.eye(4), sh, bo) is not None
    with pytest.raises(DesignError, match="gender"):  # the gender rule still applies
        mate_transform(np.eye(4), sh, dict(bo, gender="male"))
    with pytest.raises(DesignError, match="type"):
        mate_transform(np.eye(4), sh, dict(bo, type="bore.4"))
    parts = tmp_path / "parts"
    blk(parts, "motor", [P("shaft", "shaft.3", "male")])
    blk(parts, "prop", [P("hub", "bore.3", "female")])
    lib = Library(parts)
    assert design.find_compatible_ports(lib.get("motor"), lib.get("prop")) == [
        ("shaft", "hub")
    ]
    assert [p.id for p, _ in lib.compatible("shaft.3", "male")] == ["prop"]
    assert [p.id for p, _ in lib.compatible("bore.3", "female")] == ["motor"]


def test_axes_opposed_and_positions_coincide():
    a = P(
        "a",
        "t",
        pos=(0.1, 0.2, 0.3),
        axis=unit((1, 2, 2)),
        ref=unit((2, 1, -2)),
        symmetry=3,
    )
    b = P(
        "b",
        "t",
        pos=(-0.05, 0.07, 0.02),
        axis=unit((2, 3, 6)),
        ref=unit((3, 2, -2)),
        symmetry=3,
    )
    Ta = np.eye(4)
    Ta[:3, :3] = np.column_stack([rodrigues(c, (1, 1, 0), 0.7) for c in np.eye(3)])
    Ta[:3, 3] = [1.0, -2.0, 0.5]
    for k in range(3):
        Tb = mate_transform(Ta, a, b, k)
        assert np.allclose(
            Tb[:3, :3] @ b["axis"], -(Ta[:3, :3] @ a["axis"]), atol=1e-12
        )
        assert np.allclose(
            Tb[:3, :3] @ b["pos"] + Tb[:3, 3],
            Ta[:3, :3] @ a["pos"] + Ta[:3, 3],
            atol=1e-12,
        )
        assert (
            np.allclose(Tb[:3, :3] @ Tb[:3, :3].T, np.eye(3), atol=1e-12)
            and np.linalg.det(Tb[:3, :3]) > 0
        )


# ---- tree ------------------------------------------------------------------------------------------------------------
def independent_child(Tp, pa, pb, k, n):
    Rp, tp = Tp[:3, :3], Tp[:3, 3]
    za, xa = Rp @ np.array(pa["axis"]), Rp @ np.array(pa["ref"])
    x_w = rodrigues(xa, za, 2 * math.pi * k / n)
    z_w = -za
    W = np.column_stack([x_w, np.cross(z_w, x_w), z_w])
    zb, xb = np.array(pb["axis"]), np.array(pb["ref"])
    Rc = W @ np.column_stack([xb, np.cross(zb, xb), zb]).T
    return Rc, Rp @ np.array(pa["pos"]) + tp - Rc @ np.array(pb["pos"])


def test_three_level_chain_matches_independent_formulation():
    ab_a = P(
        "a",
        "t",
        pos=(0.1, 0.0, 0.02),
        axis=unit((1, 2, 2)),
        ref=unit((2, 1, -2)),
        symmetry=4,
    )
    ab_b = P(
        "b",
        "t",
        pos=(0.0, 0.03, -0.01),
        axis=unit((2, 3, 6)),
        ref=unit((3, 2, -2)),
        symmetry=4,
    )
    bc_a = P(
        "a2", "u", pos=(-0.02, 0.05, 0.04), axis=(0, 0, 1), ref=(0, 1, 0), symmetry=6
    )
    bc_b = P(
        "b2",
        "u",
        pos=(0.01, 0.01, 0.0),
        axis=unit((-1, 2, 2)),
        ref=unit((2, 2, -1)),
        symmetry=3,
    )
    ca_a = P(
        "a3", "w", pos=(0.0, 0.0, 0.05), axis=unit((0, 3, 4)), ref=(1, 0, 0), symmetry=1
    )
    ca_b = P(
        "b3",
        "w",
        pos=(0.02, -0.01, 0.0),
        axis=(1, 0, 0),
        ref=unit((0, 4, -3)),
        symmetry=1,
    )
    mates = [
        MateSpec("ab", "root", "A", ab_a, ab_b, 1),
        MateSpec("bc", "A", "B", bc_a, bc_b, 2),
        MateSpec("cd", "B", "C", ca_a, ca_b, 0),
    ]
    # the middle mate uses n = gcd(6, 3) = 3, k = 2
    tree = solve_tree("root", ["root", "A", "B", "C"], mates)
    T, n_ = np.eye(4), [4, 3, 1]
    for m, nn, child in zip(mates, n_, ["A", "B", "C"]):
        R, t = independent_child(T, m.pa, m.pb, m.rotation, nn)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        assert np.allclose(tree.T[child], T, atol=1e-12)
    assert tree.order == ["root", "A", "B", "C"]


def _loop_parts():
    pa1 = P("a1", "t", pos=(0.1, 0, 0), axis=(0, 0, 1))
    pb1 = P("b1", "t", pos=(0, 0.2, 0), axis=(0, 1, 0), ref=(1, 0, 0))
    pb2 = P("b2", "u", pos=(0, 0, 0.3), axis=(1, 0, 0), ref=(0, 1, 0))
    pc1 = P("c1", "u", pos=(0.05, 0.05, 0), axis=(0, 0, -1), ref=(1, 0, 0))
    return pa1, pb1, pb2, pc1


def test_cycle_rejected_and_closure_checked():
    pa1, pb1, pb2, pc1 = _loop_parts()
    pa2 = P("a2", "v", pos=(0.01, 0.02, 0.03))
    base = [MateSpec("ab", "A", "B", pa1, pb1), MateSpec("bc", "B", "C", pb2, pc1)]
    # C -> A without closure: cycle, error names the loop
    pc2 = P("c2", "v")
    with pytest.raises(DesignError, match=r"cycle: C -> B -> A -> C.*closure"):
        solve_tree("A", ["A", "B", "C"], base + [MateSpec("ca", "C", "A", pc2, pa2)])
    # consistent closure: pose of c2 on C chosen so that mating with a2 lands exactly on A
    T_C = solve_tree("A", ["A", "B", "C"], base).T["C"]
    F = np.linalg.inv(T_C) @ port_frame(pa2) @ RX_PI
    good = P("c2", "v", pos=F[:3, 3], axis=F[:3, 2], ref=F[:3, 0])
    tree = solve_tree(
        "A", ["A", "B", "C"], base + [MateSpec("ca", "C", "A", good, pa2, closure=True)]
    )
    assert tree.closures[0][1] < 1e-9 and tree.closures[0][2] < 1e-9
    off = dict(good, pos=list(np.array(good["pos"]) + [1e-3, 0, 0]))
    with pytest.raises(DesignError, match="closure mate 'ca' is inconsistent"):
        solve_tree(
            "A",
            ["A", "B", "C"],
            base + [MateSpec("ca", "C", "A", off, pa2, closure=True)],
        )
    tiny = dict(
        good, pos=list(np.array(good["pos"]) + [1e-9, 0, 0])
    )  # within the 1e-6 m tolerance
    solve_tree(
        "A", ["A", "B", "C"], base + [MateSpec("ca", "C", "A", tiny, pa2, closure=True)]
    )
    with pytest.raises(DesignError, match="closure mate cannot have a joint"):
        solve_tree(
            "A",
            ["A", "B", "C"],
            base + [MateSpec("ca", "C", "A", good, pa2, closure=True, has_joint=True)],
        )


def test_mate_reached_from_b_side_is_flipped_and_unreachable_rejected():
    pa1, pb1, _, _ = _loop_parts()
    m = MateSpec("ab", "A", "B", pa1, pb1, 0)
    tree = solve_tree("B", ["A", "B"], [m])  # root is b: the mate is solved backwards
    assert np.allclose(tree.T["B"], tree.T["A"] @ m.matrix(), atol=1e-12)
    with pytest.raises(DesignError, match="not connected"):
        solve_tree("A", ["A", "B", "Z"], [MateSpec("ab", "A", "B", pa1, pb1)])
    with pytest.raises(DesignError, match="swap a and b"):
        solve_tree(
            "B", ["A", "B"], [MateSpec("ab", "A", "B", pa1, pb1, has_joint=True)]
        )


# ---- fasteners -------------------------------------------------------------------------------------------------------
def test_fastener_tapped_through_and_refusals(tmp_path):
    lib = fastener_lib(tmp_path / "parts")
    a = P("a", "bolt-pattern.16x16.m3", thickness=0.002)
    # tapped: stack = 0.002, engagement = 1.5 x 3 mm = 4.5 mm -> L_min 6.5 mm -> M3x8
    rec, screw, nut, pos, warns = select_fasteners(
        lib,
        "m",
        a,
        P("b", "bolt-pattern.16x16.m3", tapped=True, thickness=0.01),
        "auto",
    )
    assert (
        rec["L_min"],
        rec["part"],
        rec["nut"],
        rec["count"],
        rec["holes"],
        warns,
    ) == (pytest.approx(0.0065, abs=1e-15), "m3x8", None, 4, 4, [])
    assert (
        rec["stack"] == 0.002
        and rec["engagement"] == pytest.approx(0.0045, abs=1e-15)
        and len(pos) == 4
    )
    assert rec["mass"] == pytest.approx(4 * 0.8e-3, abs=1e-15)
    # thread_depth caps the engagement: 0.002 + 0.004 = 6 mm exactly -> M3x6 (length >= L_min is inclusive)
    rec = select_fasteners(
        lib,
        "m",
        a,
        P("b", "bolt-pattern.16x16.m3", tapped=True, thread_depth=0.004),
        "auto",
    )[0]
    assert rec["engagement"] == 0.004 and rec["part"] == "m3x6"
    # through bolt: stack 0.002 + 0.003, thinnest nut 2.4 mm, + 2 pitches (1 mm) -> L_min 8.4 mm -> M3x10 + nut-m3
    rec, screw, nut, _, _ = select_fasteners(
        lib, "m", a, P("b", "bolt-pattern.16x16.m3", thickness=0.003), "auto"
    )
    assert rec["stack"] == pytest.approx(0.005, abs=1e-15) and rec[
        "engagement"
    ] == pytest.approx(0.0034, abs=1e-15)
    assert (
        rec["L_min"] == pytest.approx(0.0084, abs=1e-15)
        and rec["part"] == "m3x10"
        and rec["nut"] == "nut-m3"
    )
    assert rec["mass"] == pytest.approx(4 * (1e-3 + 2e-4), abs=1e-15)
    explicit = {"part": "nut-m3-thick"}  # a nut is not a screw
    with pytest.raises(DesignError, match="not a m3 screw"):
        select_fasteners(
            lib, "m", a, P("b", "bolt-pattern.16x16.m3", thickness=0.003), explicit
        )
    # refuse: stack too thick (longest is 16 mm)
    with pytest.raises(
        DesignError,
        match=r"no m3 screw fits.*L_min = 0\.0234.*longest available = 0\.016",
    ):
        select_fasteners(
            lib, "m", a, P("b", "bolt-pattern.16x16.m3", thickness=0.018), "auto"
        )
    # refuse: no screw of that thread at all
    with pytest.raises(DesignError, match=r"no m5 screw fits.*none in library"):
        select_fasteners(
            lib,
            "m",
            P("a", "bolt-pattern.16x16.m5"),
            P("b", "bolt-pattern.16x16.m5", tapped=True),
            "auto",
        )
    with pytest.raises(DesignError, match="thread mismatch"):
        select_fasteners(
            lib, "m", P("a", "x", thread="m3"), P("b", "x", thread="m4"), "auto"
        )


def test_fastener_explicit_too_short_warns_and_thread_parsing(tmp_path):
    lib = fastener_lib(tmp_path / "parts")
    a, b = (
        P("a", "bolt-circle.25.4.m3", thickness=0.002),
        P("b", "bolt-circle.25.4.m3", tapped=True),
    )
    rec, _, _, pos, warns = select_fasteners(
        lib, "m", a, b, {"part": "m3x6", "count": 2}
    )
    assert (
        [w[0] for w in warns] == ["FASTENER_SHORT"]
        and rec["count"] == 2
        and rec["holes"] == 4
        and len(pos) == 2
    )
    rec, _, _, _, warns = select_fasteners(lib, "m", a, b, {"part": "m3x12"})
    assert warns == []
    assert (
        design.thread_of(P("a", "bolt-circle.25.4.m3")) == "m3"
        and design.thread_of(P("a", "thread.m2.5")) == "m2.5"
    )
    assert (
        design.thread_of(P("a", "bolt-pattern.x", thread="m4")) == "m4"
        and design.thread_of(P("a", "bolt-pattern.16x16")) is None
    )
    assert (
        design.hole_count(P("a", "bolt-circle.25.4.m3")) == 4
        and design.hole_count(P("a", "thread.m3")) == 1
    )
    pos = design.hole_positions(P("a", "bolt-pattern.30.5x30.5.m3"))
    assert sorted(map(tuple, pos)) == sorted(
        [
            (0.01525, 0.01525),
            (-0.01525, 0.01525),
            (-0.01525, -0.01525),
            (0.01525, -0.01525),
        ]
    )
    assert np.allclose(
        design.hole_positions(P("a", "bolt-circle.25.4.m3"))[1], [0, 0.0125], atol=1e-15
    )


# ---- numbers rule / save-load ----------------------------------------------------------------------------------------
HEAD = '[assembly]\nname = "t"\nbase = "fixed"\ntimestep = 0.002\n[[instances]]\nid = "a"\npart = "pa"\nroot = true\n[[instances]]\nid = "b"\npart = "pa"\n'


def test_numbers_rule():
    with pytest.raises(DesignError, match=r"bare number at 'wires\.0\.length'"):
        loads(HEAD + '[[wires]]\nfrom = "a.p"\nto = "b.p"\npart = "w"\nlength = 0.3\n')
    with pytest.raises(DesignError, match="bare number.*joint.range"):
        loads(
            HEAD
            + '[[mates]]\na = "a"\nb = "b"\njoint = { type = "hinge", range = [-1.0, 1.0] }\n'
        )
    with pytest.raises(
        DesignError, match="params.mass: an override must be a provenance table"
    ):
        loads(
            HEAD.replace('part = "pa"\n', 'part = "pa"\nparams = { mass = 3.0 }\n', 1)
        )
    with pytest.raises(DesignError, match="estimated provenance needs 'rationale'"):
        loads(
            HEAD
            + '[[wires]]\nfrom = "a.p"\nto = "b.p"\npart = "w"\nlength = { value = 1.0, unit = "m", fidelity = "estimated" }\n'
        )
    ok = loads(
        HEAD
        + '[[wires]]\nfrom = "a.p"\nto = "b.p"\npart = "w"\nconductors = 2\nlength = "a.cable"\n'
        '[[mates]]\na = "a"\nb = "b"\nrotation = 1\nfasteners = { part = "s", count = 2 }\n'
    )
    assert (
        ok.doc["wires"][0]["conductors"] == 2
    )  # counts / rotation / sim settings may be bare


def test_references_and_provenance_resolve(tmp_path):
    parts = tmp_path / "parts"
    write_part(parts, "pa", params={"cable": prm(0.25, "m"), "mass": prm(1.0)})
    a = loads(
        HEAD + '[[wires]]\nfrom = "a.p"\nto = "b.p"\npart = "w"\nlength = "a.cable"\n'
    )
    ctx = Ctx(a, Library(parts))
    assert ctx.num("a.cable", "t", {"m": 1.0})[0] == 0.25
    assert ctx.num(prov(2.0, "m"), "t", {"m": 1.0})[0] == 2.0
    assert (
        ctx.estimated and ctx.estimated[0]["rationale"] == "test"
    )  # estimated values used are listed
    with pytest.raises(DesignError, match="unit 'm', expected one of \\['kg'\\]"):
        ctx.num("a.cable", "t", {"kg": 1.0})
    with pytest.raises(DesignError, match="no param 'nope'"):
        ctx.num("a.nope", "t")
    with pytest.raises(DesignError, match="unknown instance|not an"):
        ctx.num("zz.cable", "t")


def test_save_load_round_trip():
    text = (
        HEAD.replace(
            'part = "pa"\n',
            'part = "pa"\ngroup = "g"\nparams = { x = { value = 1.5, unit = "kg", fidelity = "estimated", rationale = "say \\"hi\\"" } }\n',
            1,
        )
        + '[[mates]]\nname = "m1"\na = "a.p1"\nb = "b"\nrotation = 2\nclosure = false\nfasteners = "auto"\nnut = "n"\n'
        'joint = { type = "hinge", reverse = true, range = { value = [-1.0, 1e-05], unit = "rad", fidelity = "derived", derivation = "d" } }\n'
        '[[wires]]\nfrom = "a.p"\nto = "b.p"\npart = "w"\nlength = "a.x"\nconductors = 2\n'
        '[device_maps.m]\n"motor.kv" = "kv"\n"capacity_ah" = "motor:cap"\n'
        '[[devices]]\ntype = "esc"\nname = "e1"\ninstance = "a"\nmap = "m"\nmotor = "b"\ncomplementary = true\ndir = -1\n'
        'set = { "motor.i0" = { value = 0.5, unit = "A", fidelity = "datasheet", src = "S", quote = "q" } }\n'
        '[[targets]]\nname = "t1"\ngroups = ["g"]\nvalue = { value = 2.0, unit = "kg", fidelity = "derived", derivation = "d" }\n'
    )
    a = loads(text)
    b = loads(dumps(a))
    assert a == b and dumps(b) == dumps(a)


@pytest.mark.parametrize("name", ["so101", "x500v2"])
def test_real_assembly_save_load_round_trip_builds_identically(name, tmp_path):
    root = design.__file__.rsplit("/python/", 1)[0]
    src, parts = f"{root}/assemblies/{name}.toml", f"{root}/parts"
    a = design.load(src)
    design.save(a, tmp_path / f"{name}.toml")
    assert design.load(tmp_path / f"{name}.toml") == a
    r1 = design.build(src, parts, tmp_path / "o1")
    r2 = design.build(tmp_path / f"{name}.toml", parts, tmp_path / "o2")
    assert r1["xml"].read_bytes() == r2["xml"].read_bytes()


# ---- mass ------------------------------------------------------------------------------------------------------------
def write_cube_stl(path, s):
    faces = [
        ((s, 0, 0), (0, s, 0), (0, 0, s)),
        ((0, 0, 0), (0, 0, s), (0, s, 0)),
        ((0, s, 0), (0, 0, s), (s, 0, 0)),
        ((0, 0, 0), (s, 0, 0), (0, 0, s)),
        ((0, 0, s), (s, 0, 0), (0, s, 0)),
        ((0, 0, 0), (0, s, 0), (s, 0, 0)),
    ]
    tris = []
    for o, u, v in faces:
        o, u, v = map(np.array, (o, u, v))
        tris += [(o, o + u, o + u + v), (o, o + u + v, o + v)]
    out = bytearray(80) + struct.pack("<I", len(tris))
    for t in tris:
        out += struct.pack("<12fH", 0, 0, 0, *np.concatenate(t), 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(out))


def cube_part(parts, pid="cube", wall=2, infill=0.2):
    write_part(parts, "mat", "material", {"density": prm(1000.0, "kg/m3")})
    write_part(
        parts,
        pid,
        "frame.printed",
        {
            "infill_fraction": prm(infill, "1"),
            "nozzle_diameter": prm(0.0004, "m"),
            "wall_count": prm(wall, "1", "estimated") | {"rationale": "r"},
            "mass_x": prm(1.0),
        },
        [P("bot", "seat", axis=(0, 0, -1), pos=(0.03125, 0.03125, 0))],
        {"shape": "mesh", "mesh": "meshes/cube.stl", "material": "mat"},
    )


def test_printed_part_mass_formula(tmp_path):
    parts = tmp_path / "parts"
    write_cube_stl(
        tmp_path / "meshes" / "cube.stl", 0.0625
    )  # side 1/16 m: exact in float32
    cube_part(parts)
    lib = Library(parts)
    a = loads('[assembly]\nname="t"\n[[instances]]\nid="c"\npart="cube"\nroot=true\n')
    ctx = Ctx(a, lib)
    inst = ctx.insts["c"]
    mass, fid, tol, note, sm = instance_mass(ctx, inst, part_shapes(lib, inst.part))
    # V = s^3 = 2.44140625e-4, A = 6 s^2, t = 2 x 0.4 mm -> V_shell = A t = 1.875e-5; m = rho (V_shell + 0.2 (V - V_shell))
    assert (
        mass == pytest.approx(0.063828125, rel=1e-9)
        and fid == "derived"
        and sm == [pytest.approx(mass)]
    )
    assert tol == pytest.approx(
        0.0075, rel=1e-9
    )  # wall_count 1 and 3 both move the mass by 7.5 g
    assert any(
        e["key"] == "wall_count" for e in ctx.estimated
    )  # estimated input is listed


def test_library_load_errors_do_not_crash_until_used(tmp_path):
    parts = tmp_path / "parts"
    blk(parts, "ok")
    (parts / "broken.toml").write_text("id = [unterminated\n")
    (parts / "noid.toml").write_text('name = "x"\n')
    lib = Library(parts)
    assert len(lib.errors) == 2 and lib.get("ok").id == "ok"
    with pytest.raises(DesignError, match="failed to load"):
        lib.get("broken")
    with pytest.raises(DesignError, match="unknown part"):
        lib.get("nope")
    a = loads('[assembly]\nname="t"\n[[instances]]\nid="a"\npart="broken"\nroot=true\n')
    with pytest.raises(DesignError, match="failed to load"):
        design.compute(a, parts, tmp_path)
    assert [p.id for p in lib.by_category("misc")] == ["ok"] and lib.by_port_type(
        "x"
    ) == []


# ---- build -----------------------------------------------------------------------------------------------------------
def tiny_parts(tmp_path):
    parts = tmp_path / "parts"
    blk(
        parts,
        "plate",
        [P("top", "seat", axis=(0, 0, 1), pos=(0, 0, 0.05))],
        mass=0.5,
        half=(0.1, 0.1, 0.05),
    )
    write_cube_stl(tmp_path / "meshes" / "cube.stl", 0.0625)
    cube_part(parts)
    write_part(
        parts,
        "m3x6",
        "fastener",
        {"mass": prm(1e-3), "length": prm(6e-3, "m"), "thread_pitch": prm(0.5e-3, "m")},
        [P("thread", "thread.m3", "male", axis=(0, 0, -1), symmetry=360)],
    )
    return parts


TINY = """
[assembly]
name = "tiny"
base = "free"
timestep = 0.001
[[instances]]
id = "plate"
part = "plate"
root = true
group = "frame"
[[instances]]
id = "arm"
part = "cube"
group = "printed"
[[mates]]
a = "plate"
b = "arm"
joint = { type = "hinge", range = { value = [-1.0, 1.0], unit = "rad", fidelity = "estimated", rationale = "test" } }
[[targets]]
name = "all"
all = true
value = { value = 0.6, unit = "kg", fidelity = "derived", derivation = "test" }
"""


def test_tiny_build_is_byte_identical_loads_and_steps(tmp_path):
    parts = tiny_parts(tmp_path)
    (tmp_path / "tiny.toml").write_text(TINY)
    out = tmp_path / "out"
    r1 = design.build(tmp_path / "tiny.toml", parts, out)
    x1, j1 = r1["xml"].read_bytes(), r1["report_path"].read_bytes()
    r2 = design.build(tmp_path / "tiny.toml", parts, out)
    assert r2["xml"].read_bytes() == x1 and r2["report_path"].read_bytes() == j1
    assert (
        str(tmp_path).encode() not in x1 and str(tmp_path).encode() not in j1
    )  # no absolute paths
    rep = json.loads(j1)
    names = {g.get("name") for g in ET.fromstring(x1).iter("geom")} - {"floor"}
    assert names == set(rep["geoms"]) == {"plate", "arm"}
    assert rep["geoms"]["arm"] == "cube" and rep["mass"]["total"] == pytest.approx(
        0.5 + 0.063828125
    )
    t = rep["targets"][0]
    assert (
        t["gap"] == pytest.approx(0.563828125 - 0.6)
        and t["tolerance"] == pytest.approx(0.0075)
        and not t["within"]
    )
    root = ET.fromstring(x1)
    assert (
        root.find("compiler").get("angle") == "radian" and root.find("custom") is None
    )
    assert (
        root.find("asset/mesh").get("file") == "../meshes/cube.stl"
    )  # relative to the output file
    # free base: the plate (half-height 0.05) rests on z = 0
    assert float(root.find("worldbody/body").get("pos").split()[2]) == pytest.approx(
        0.05
    )
    if napl._napl is None:
        pytest.skip("kernel not built")
    sim = napl.Sim.load(r1["xml"])
    sim.step(100)
    assert not sim.diverged


def test_fasteners_on_mate_add_geoms_and_mass(tmp_path):
    parts = tiny_parts(tmp_path)
    text = TINY.replace('b = "arm"\n', 'b = "arm"\nfasteners = { part = "m3x6" }\n')
    # the seat type has no thread suffix, so the ports carry a `thread` key; the cube's port is tapped
    # (stack = plate port thickness 0, engagement 4.5 mm <= the 6 mm screw): one hole -> one screw
    for pid, extra in (
        ("plate", 'thread = "m3"'),
        ("cube", 'thread = "m3"\ntapped = true'),
    ):
        s = (
            (parts / f"{pid}.toml")
            .read_text()
            .replace('type = "seat"', 'type = "seat"\n' + extra)
        )
        (parts / f"{pid}.toml").write_text(s)
    (tmp_path / "f.toml").write_text(text)
    r = design.build(tmp_path / "f.toml", parts, tmp_path / "out")["report"]
    assert (
        r["fasteners"][0]["part"] == "m3x6"
        and r["fasteners"][0]["count"] == 1
        and r["warnings"] == []
    )
    assert any(k.endswith(":screw:0") for k in r["geoms"]) and r["mass"][
        "total"
    ] == pytest.approx(0.5 + 0.063828125 + 1e-3)


DEV = """
[assembly]
name = "dev"
base = "fixed"
seed = 3
[[instances]]
id = "plate"
part = "plate"
root = true
group = "frame"
[[instances]]
id = "pack"
part = "pack"
group = "power"
[[instances]]
id = "lamp"
part = "lamp"
group = "power"
[[mates]]
a = "plate.top"
b = "pack.bot"
[[mates]]
a = "plate.side"
b = "lamp.bot"
[[wires]]
from = "pack.out"
to = "lamp.in"
part = "wire24"
length = { value = 0.5, unit = "m", fidelity = "estimated", rationale = "test" }
conductors = 2
[device_maps.pack]
capacity_ah = "capacity_ah"
r0 = "r0"
[[devices]]
type = "battery"
name = "bat"
instance = "pack"
port = "out"
map = "pack"
ocv = "ocv"
[[devices]]
type = "load"
name = "lamp"
instance = "lamp"
port = "in"
set = { i0 = { value = -0.5, unit = "A", fidelity = "estimated", rationale = "test" } }
"""


def dev_parts(tmp_path, r0_unit="ohm"):
    parts = tmp_path / "parts"
    blk(
        parts,
        "plate",
        [
            P("top", "seat.a", pos=(0, 0, 0.05)),
            P("side", "seat.b", pos=(0.1, 0, 0), axis=(1, 0, 0), ref=(0, 1, 0)),
        ],
        mass=0.5,
    )
    write_part(
        parts,
        "pack",
        "battery.lipo",
        {
            "mass": prm(0.2),
            "capacity_ah": prm(5.2, "Ah"),
            "cells_series": prm(2, "1"),
            "r0": prm(0.02, r0_unit),
        },
        [
            P("bot", "seat.a", axis=(0, 0, -1)),
            P("out", "xt60", "male", kind="power", role="source"),
        ],
        {"shape": "box", "size": [0.02, 0.02, 0.02]},
    )
    write_part(
        parts,
        "lamp",
        "load",
        {"mass": prm(0.1)},
        [
            P("bot", "seat.b", axis=(-1, 0, 0), ref=(0, 1, 0)),
            P("in", "xt60", "female", kind="power", role="sink"),
        ],
        {"shape": "sphere", "size": [0.02]},
    )
    write_part(
        parts,
        "wire24",
        "wire",
        {
            "awg": prm(24, "AWG"),
            "mass_per_m": prm(0.002, "kg/m", "derived") | {"derivation": "d"},
        },
    )
    write_part(
        parts,
        "xt60-f",
        "connector",
        {"mass": prm(0.003)},
        [P("c", "xt60", "female", kind="power")],
    )
    write_part(
        parts,
        "xt60-m",
        "connector",
        {"mass": prm(0.004)},
        [P("c", "xt60", "male", kind="power")],
    )
    write_part(parts, "ocv", "battery.cell_curve", {}, [])
    (parts / "ocv.toml").write_text(
        (parts / "ocv.toml").read_text()
        + '\n[tables.ocv_vs_soc]\ncolumns = ["soc", "ocv_V"]\nrows = [[0.0, 3.0], [0.5, 3.7], [1.0, 4.2]]\nfidelity = "literature"\n'
    )
    return parts


def test_wires_devices_connectors_and_unit_check(tmp_path):
    parts = dev_parts(tmp_path)
    xml, rep = design.compute(loads(DEV), parts, tmp_path / "out")
    dev = json.loads(ET.fromstring(xml).find("custom/text").get("data"))
    assert dev["seed"] == 3
    assert dev["wires"] == [
        {
            "name": "wire0",
            "a": 1,
            "b": 2,
            "awg": 24,
            "length_m": 0.5,
            "conductors": 2,
            "parts": ["wire24"],
        }
    ]
    assert dev["batteries"][0]["node"] == 1 and dev["loads"][0] == {
        "name": "lamp",
        "node": 2,
        "i0": -0.5,
        "parts": ["lamp"],
    }
    assert dev["batteries"][0]["ocv_v"] == [6.0, 7.4, 8.4] and dev["batteries"][0][
        "parts"
    ] == ["ocv", "pack"]
    assert [c["part"] for c in rep["connectors"]] == ["xt60-f", "xt60-m"]
    # wire mass = 0.002 kg/m x 0.5 m x 2 conductors; connectors 3 g + 4 g
    assert rep["wires"][0]["mass"] == pytest.approx(0.002) and rep["mass"][
        "total"
    ] == pytest.approx(0.5 + 0.2 + 0.1 + 0.002 + 0.007)
    assert {"wire:0", "wire:0:conn:0", "wire:0:conn:1"} <= set(rep["geoms"]) and rep[
        "mass"
    ]["by_group"]["power"] == pytest.approx(0.309)
    assert any(
        e["where"].startswith("wire 0 (pack.out") and e["key"] == "length"
        for e in rep["estimated"]
    )
    assert [d["name"] for d in rep["devices"]] == ["wire0", "bat", "lamp"]
    # included_in: electrical only, no mass
    inc = DEV.replace("conductors = 2\n", 'conductors = 2\nincluded_in = "pack"\n')
    assert design.compute(loads(inc), parts, tmp_path / "out")[1]["mass"][
        "total"
    ] == pytest.approx(0.8)
    # wrong unit on a mapped param is an error
    dev_parts(tmp_path, r0_unit="mOhm")
    with pytest.raises(DesignError, match="unit 'mOhm'"):
        design.compute(loads(DEV), tmp_path / "parts", tmp_path / "out")
    if napl._napl is None:
        pytest.skip("kernel not built")
    dev_parts(tmp_path)
    r = design.build(_write(tmp_path / "dev.toml", DEV), parts, tmp_path / "out")
    sim = napl.Sim.load(r["xml"])  # strict device-JSON parse in the kernel
    sim.step(10)
    assert not sim.diverged


def _write(path, text):
    path.write_text(text)
    return path


DRIVE = (
    DEV.split("[[mates]]")[0]
    .replace('name = "dev"', 'name = "drv"')
    .replace('[[instances]]\nid = "lamp"\npart = "lamp"\ngroup = "power"\n', "")
    + """
[[instances]]
id = "mot"
part = "mot"
[[instances]]
id = "rot"
part = "rot"
[[instances]]
id = "esc"
part = "esc"
[[mates]]
a = "plate.top"
b = "pack.bot"
[[mates]]
a = "plate.m"
b = "mot.bot"
[[mates]]
name = "spin"
a = "mot.top"
b = "rot.bot"
joint = { type = "hinge", spin = true }
[[mates]]
a = "plate.e"
b = "esc.bot"
[[wires]]
from = "pack.out"
to = "esc.in"
part = "wire24"
length = { value = 0.2, unit = "m", fidelity = "estimated", rationale = "test" }
conductors = 2
[device_maps.pack]
capacity_ah = "capacity_ah"
r0 = "r0"
[device_maps.esc]
"motor.kv" = "motor:kv"
"motor.r20" = "motor:r_winding"
"motor.i0" = "motor:i0"
"motor.rotor_inertia" = "motor:rotor_inertia"
"motor.c_th" = "motor:c_th"
"motor.r_th" = "motor:r_th"
[device_maps.rotor]
diameter = "diameter"
[[devices]]
type = "battery"
name = "bat"
instance = "pack"
port = "out"
map = "pack"
ocv = "ocv"
[[devices]]
type = "esc"
name = "esc1"
instance = "esc"
port = "in"
motor = "mot"
joint = "spin"
map = "esc"
complementary = true
set = { i_limit = { value = 20.0, unit = "A", fidelity = "estimated", rationale = "test" } }
[[devices]]
type = "rotor"
name = "prop"
instance = "rot"
body = "rot"
joint = "spin"
map = "rotor"
dir = 1
ground_effect = true
set = { j = { value = [0.0, 0.5], unit = "1", fidelity = "estimated", rationale = "t" }, ct = { value = [0.1, 0.05], unit = "1", fidelity = "estimated", rationale = "t" }, cq = { value = [0.01, 0.005], unit = "1", fidelity = "estimated", rationale = "t" } }
[[devices]]
type = "imu"
name = "imu"
site = "plate"
set = { rate_hz = { value = 500.0, unit = "Hz", fidelity = "estimated", rationale = "t" }, "gyro.noise_density" = { value = 0.001, unit = "rad/s/sqrt(Hz)", fidelity = "estimated", rationale = "t" }, "accel.noise_density" = { value = 0.004, unit = "m/s^2/sqrt(Hz)", fidelity = "estimated", rationale = "t" } }
"""
)


def test_esc_rotor_imu_bindings_load_and_run_in_kernel(tmp_path):
    parts = dev_parts(tmp_path)
    blk(
        parts,
        "plate",
        [
            P("top", "seat.a", pos=(0, 0, 0.05)),
            P("m", "seat.m", pos=(0.1, 0, 0.05)),
            P("e", "seat.e", pos=(-0.1, 0, 0.05)),
        ],
        mass=0.5,
    )
    write_part(
        parts,
        "mot",
        "motor.bldc",
        {
            "mass": prm(0.06),
            "kv": prm(920, "rpm/V"),
            "r_winding": prm(0.115, "ohm"),
            "i0": prm(0.8, "A"),
            "rotor_inertia": prm(4.2e-6, "kg*m^2"),
            "c_th": prm(20.0, "J/K"),
            "r_th": prm(3.0, "K/W"),
        },
        [P("bot", "seat.m", axis=(0, 0, -1)), P("top", "seat.rot", pos=(0, 0, 0.02))],
        {"shape": "cylinder", "size": [0.014, 0.02]},
    )
    write_part(
        parts,
        "rot",
        "prop",
        {"mass": prm(0.01), "diameter": prm(0.254, "m")},
        [P("bot", "seat.rot", axis=(0, 0, -1))],
        {"shape": "cylinder", "size": [0.12, 0.002]},
    )
    write_part(
        parts,
        "esc",
        "esc",
        {"mass": prm(0.03)},
        [
            P("bot", "seat.e", axis=(0, 0, -1)),
            P("in", "xt60", "female", kind="power", role="sink"),
        ],
        {"shape": "box", "size": [0.02, 0.01, 0.005]},
    )
    xml, rep = design.compute(loads(DRIVE), parts, tmp_path / "out")
    dev = json.loads(ET.fromstring(xml).find("custom/text").get("data"))
    esc = dev["escs"][0]
    assert esc["joint"] == "spin" and esc["motor"] == {
        "kv": 920,
        "r20": 0.115,
        "i0": 0.8,
        "rotor_inertia": 4.2e-6,
        "c_th": 20.0,
        "r_th": 3.0,
    }
    assert (
        esc["parts"] == ["esc", "mot"]
        and esc["complementary"] is True
        and esc["i_limit"] == 20.0
    )
    rotor = dev["rotors"][0]
    assert (rotor["body"], rotor["ray_groups"], rotor["dir"], rotor["j"]) == (
        "rot",
        47,
        1,
        [0.0, 0.5],
    )  # group 4 (vehicle) excluded
    assert dev["imus"][0]["site"] == "site:plate" and 'name="site:plate"' in xml
    jl = [
        ln for ln in xml.splitlines() if "<joint" in ln
    ]  # unlimited rotor hinge, axis = a.axis in b's frame
    assert (
        len(jl) == 1 and "range" not in jl[0] and 'type="hinge" axis="0 0 1.0"' in jl[0]
    ), jl
    if napl._napl is None:
        pytest.skip("kernel not built")
    sim = napl.Sim.load(_write(tmp_path / "drv.xml", xml))
    sim.step(100)
    assert not sim.diverged


def test_cli_design_build_info_and_errors(tmp_path, capsys):
    from napl import cli

    parts = tiny_parts(tmp_path)
    _write(tmp_path / "tiny.toml", TINY)
    out = tmp_path / "o"
    assert (
        cli.main(
            [
                "design",
                "build",
                str(tmp_path / "tiny.toml"),
                "--parts",
                str(parts),
                "--out",
                str(out),
            ]
        )
        == 0
    )
    assert (
        "built tiny" in capsys.readouterr().out
        and (out / "tiny.xml").is_file()
        and (out / "tiny.report.json").is_file()
    )
    assert (
        cli.main(["design", "info", str(tmp_path / "tiny.toml"), "--parts", str(parts)])
        == 0
    )
    assert "target all" in capsys.readouterr().out
    bare = 'value = { value = 0.6, unit = "kg", fidelity = "derived", derivation = "test" }'
    assert bare in TINY
    _write(tmp_path / "bad.toml", TINY.replace(bare, "value = 0.6"))
    assert (
        cli.main(["design", "info", str(tmp_path / "bad.toml"), "--parts", str(parts)])
        == 2
    )
    assert "bare number" in capsys.readouterr().err
