"""assemblies/so101.toml (SO-101 follower, parts/arm): builds, loads in the kernel, holds the zero pose, mass vs the URDF,
geom provenance, determinism, and joint frames against the official URDF (meshes/so101/so101_new_calib.urdf)."""

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import so101_lib

ASM = REPO / "assemblies" / "so101.toml"
PARTS = REPO / "parts"
JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
SERVOS = [f"servo{i}" for i in range(1, 7)]
G = 9.81


# ---- helpers ---------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("so101")
    r = design.build(ASM, PARTS, out)
    return {"xml": r["xml"], "report": r["report"], "dir": out}


def quat_R(q):
    w, x, y, z = np.array(q, float) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def floats(s, n=None):
    v = [float(x) for x in s.split()]
    assert n is None or len(v) == n
    return np.array(v)


def mjcf_tree(xml_path):
    """-> (devices dict, {joint: (path, world pos, world axis, range)}, [geom elements]) from the built MJCF."""
    root = ET.parse(xml_path).getroot()
    data = root.find("custom/text[@name='napl.devices']").get("data")
    joints, geoms = {}, []

    def walk(body, T, path):
        pos = floats(body.get("pos", "0 0 0"), 3)
        R = quat_R(floats(body.get("quat", "1 0 0 0"), 4))
        Tw = np.eye(4)
        Tw[:3, :3], Tw[:3, 3] = T[:3, :3] @ R, T[:3, :3] @ pos + T[:3, 3]
        path = f"{path}/{body.get('name')}"
        for j in body.findall("joint"):
            p, a = floats(j.get("pos"), 3), floats(j.get("axis"), 3)
            rng = floats(j.get("range"), 2) if j.get("range") else None
            joints[j.get("name")] = (
                f"{path}/{j.get('name')}",
                Tw[:3, :3] @ p + Tw[:3, 3],
                Tw[:3, :3] @ a,
                rng,
            )
        geoms.extend(body.findall("geom"))
        for b in body.findall("body"):
            walk(b, Tw, path)

    wb = root.find("worldbody")
    geoms.extend(wb.findall("geom"))
    for b in wb.findall("body"):
        walk(b, np.eye(4), "/world")
    return json.loads(data), joints, geoms


# ---- 1. build + kernel load --------------------------------------------------------------------------------------------
def test_builds_and_loads_in_the_kernel(built):
    dev, joints, _ = mjcf_tree(built["xml"])
    assert [s["name"] for s in dev["servos"]] == SERVOS
    assert [s["joint"] for s in dev["servos"]] == JOINTS
    assert [p["name"] for p in dev["psus"]] == ["psu"]
    assert len(dev["wires"]) == 8  # PSU cord, adapter internal edge, 6 servo cables
    assert set(JOINTS) == set(joints)
    sim = napl.Sim.load(
        built["xml"]
    )  # strict flatbuffers JSON: unknown / mistyped field = load error
    for name in ["psu", *SERVOS]:
        sim.query(f"/devices/{name}", "voltage")  # prim exists (KeyError otherwise)
    for j in JOINTS:
        assert sim.query(joints[j][0], "qpos")[0][0] == 0.0
    assert not sim.diverged
    # joint damping / armature of the official MJCF are not in the model (the servo device owns them)
    text = Path(built["xml"]).read_text()
    assert "damping=" not in text and "armature=" not in text


# ---- 2. holds the zero pose --------------------------------------------------------------------------------------------
def test_holds_zero_pose_within_the_servo_law(built):
    dev, joints, _ = mjcf_tree(built["xml"])
    cfg = {s["name"]: s for s in dev["servos"]}
    sim = napl.Sim.load(built["xml"])
    for n in SERVOS:
        sim.set(
            f"/devices/{n}", "goal_position", 0.0
        )  # = the power-up goal, set explicitly
        sim.set(f"/devices/{n}", "torque_enable", 1)
    sim.step(1000)  # 1 s at 1 ms
    assert not sim.diverged

    q = lambda n, a: float(sim.query(f"/devices/{n}", a)[0][0])
    print(
        f"\n{'servo':8s} {'joint':14s} {'q rad':>9s} {'q deg':>8s} {'counts':>6s} {'tau N*m':>8s} {'duty':>7s} {'I A':>7s} {'V':>6s} {'omega':>8s} {'tol rad':>8s}"
    )
    for n, j in zip(SERVOS, JOINTS):
        c, m = cfg[n], cfg[n]["motor"]
        qj = float(sim.query(joints[j][0], "qpos")[0][0])
        Kt = 60 / (2 * math.pi * m["kv"])
        V, tau, duty = q(n, "voltage"), q(n, "torque"), q(n, "duty")
        count = 2 * math.pi / c["encoder_cpr"]
        # Tolerance from the model, not a chosen bound. At rest the servo must carry the gravity load: its static torque is
        # tau = eta N Kt (duty V / R) (back-EMF 0, friction 0 at zero speed), so the duty it needs is |tau| R / (eta N Kt V)
        # and the position loop produces that duty from an error e = duty pid_scale / p counts (proportional term only: the
        # LeRobot I gain is 0, D acts on velocity only). The load carried at the end of the second (|tau| measured, it equals
        # the gravity torque up to the small acceleration) therefore bounds the sag; plus the dead zone and one count of
        # encoder quantisation.
        duty_static = abs(tau) * m["r20"] / (c["efficiency"] * c["gear_ratio"] * Kt * V)
        e_counts = duty_static * c["pid_scale"] / c["p"] + c["deadband"] + 1
        tol = e_counts * count
        print(
            f"{n:8s} {j:14s} {qj:9.4f} {math.degrees(qj):8.2f} {q(n, 'counts'):6.0f} {tau:8.4f} {duty:7.4f} {q(n, 'current'):7.4f} {V:6.3f} {q(n, 'velocity'):8.4f} {tol:8.4f}"
        )
        assert abs(duty) < c["max_torque"] / 1000, (
            f"{n} saturated: the linear law does not apply"
        )
        assert abs(qj) <= tol, (
            f"{j}: |q| = {abs(qj):.4f} rad > model tolerance {tol:.4f}"
        )
        assert abs(q(n, "counts") * count - qj) <= count  # encoder reads the joint
        assert q(n, "temperature") < 70  # below the part's max_temperature register
    assert (
        4.0 < q("servo1", "voltage") <= 5.0
    )  # the 5 V supply, sagging under load, above min_input_voltage


def stl_centroid(path):
    """Volume centroid of a closed binary STL (signed tetrahedra against the origin)."""
    raw = Path(path).read_bytes()
    n = int(np.frombuffer(raw, "<u4", 1, 80)[0])
    rec = np.frombuffer(
        raw, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84
    )
    v = rec["v"].astype(float)
    sv = np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])) / 6
    return (sv[:, None] * v.sum(1) / 4).sum(0) / sv.sum()


def static_torques(xml_path, joint_names, q=None):
    """Gravity torque (N*m) about each named joint axis at joint angles q ({joint: rad}, default all 0), from the MJCF alone: every geom of the joint's subtree
    contributes its `mass` at its own centre (uniform density: primitive pos / capsule midpoint, mesh = STL volume centroid)."""
    root = ET.parse(xml_path).getroot()
    meshes = {m.get("name"): m.get("file") for m in root.iter("mesh")}
    out, q = {}, q or {}

    def geom_com(g):
        if g.get("type") == "mesh":
            c = stl_centroid(Path(xml_path).parent / meshes[g.get("mesh")])
            R = quat_R(floats(g.get("quat", "1 0 0 0"), 4))
            return R @ c + floats(g.get("pos", "0 0 0"), 3)
        if g.get("fromto"):
            return floats(g.get("fromto"), 6).reshape(2, 3).mean(0)
        return floats(g.get("pos", "0 0 0"), 3)

    def walk(body, T):
        pos, R = (
            floats(body.get("pos", "0 0 0"), 3),
            quat_R(floats(body.get("quat", "1 0 0 0"), 4)),
        )
        Tw = (T[0] @ R, T[0] @ pos + T[1])
        Tj = Tw  # frame the joint is defined in; the geoms and child bodies see it turned by q about the joint
        for j in body.findall("joint"):
            a = floats(j.get("axis"), 3)
            a, p = a / np.linalg.norm(a), floats(j.get("pos"), 3)
            c, s = (
                math.cos(q.get(j.get("name"), 0.0)),
                math.sin(q.get(j.get("name"), 0.0)),
            )
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            Rj = c * np.eye(3) + s * K + (1 - c) * np.outer(a, a)
            Tw = (Tj[0] @ Rj, Tj[0] @ (p - Rj @ p) + Tj[1])
        mine = [
            (float(g.get("mass")), Tw[0] @ geom_com(g) + Tw[1])
            for g in body.findall("geom")
        ]
        for b in body.findall("body"):
            mine += walk(b, Tw)
        for j in body.findall("joint"):
            if j.get("name") in joint_names:
                r0 = Tj[0] @ floats(j.get("pos"), 3) + Tj[1]
                axis = Tj[0] @ floats(j.get("axis"), 3)
                tau = sum(
                    np.dot(axis, np.cross(r - r0, [0, 0, -m * G])) for m, r in mine
                )
                out[j.get("name")] = tau
        return mine

    for b in root.find("worldbody").findall("body"):
        walk(b, (np.eye(3), np.zeros(3)))
    return out


def test_holding_torque_matches_independent_gravity_torque(built):
    """Independent of the servo law: the torque the servos hold equals the static gravity torque computed from the MJCF geom
    masses and centres at the pose the arm settles in (the servo sags a few degrees under load, so the q = 0 value is only
    printed). No simulated torque enters the expected value."""
    joints_ = ["shoulder_lift", "elbow_flex"]
    _, jmap, _ = mjcf_tree(built["xml"])
    q0 = static_torques(built["xml"], set(joints_))
    sim = napl.Sim.load(built["xml"])
    for n in SERVOS:
        sim.set(f"/devices/{n}", "goal_position", 0.0)
        sim.set(f"/devices/{n}", "torque_enable", 1)
    sim.step(1000)
    qs = {j: float(sim.query(jmap[j][0], "qpos")[0][0]) for j in JOINTS}
    expect = static_torques(built["xml"], set(joints_), qs)
    for n, j in zip(SERVOS, JOINTS):
        if j in joints_:
            tau = float(sim.query(f"/devices/{n}", "torque")[0][0])
            print(
                f"{j}: servo {tau:.5f} N*m, independent gravity at q={qs[j]:.4f}: {expect[j]:.5f} N*m, at q=0: {q0[j]:.5f} N*m"
            )
            assert abs(expect[j]) > 0.01  # a real load, not 0 == 0
            assert abs(abs(tau) - abs(expect[j])) <= 0.02 * abs(expect[j]), (
                j,
                tau,
                expect[j],
            )


# ---- 3. mass vs the URDF -------------------------------------------------------------------------------------------------
def test_mass_budget_and_urdf_target(built):
    rep = built["report"]
    _, _, geoms = mjcf_tree(built["xml"])
    total = rep["mass"]["total"]
    assert math.isclose(sum(rep["mass"]["by_instance"].values()), total, rel_tol=1e-12)
    gm = sum(float(g.get("mass")) for g in geoms if g.get("name") != "floor")
    assert math.isclose(
        gm, total, rel_tol=1e-9
    )  # every kg in the report is a geom in the MJCF
    (t,) = rep["targets"]
    assert t["name"] == "URDF total mass" and t["scope"] == ["arm"]
    urdf = sum(  # the 7 link masses of the official URDF (the 1e-9 one is the TCP frame link)
        float(m.get("value"))
        for m in ET.parse(so101_lib.URDF).getroot().iter("mass")
        if float(m.get("value")) > 1e-6
    )
    assert math.isclose(urdf, t["target"], abs_tol=1e-9)
    screws = sum(f["mass"] for f in rep["fasteners"] if f["group"] == "arm")
    cables = sum(
        w["mass"] for w in rep["wires"] if w.get("counted") and w["group"] == "arm"
    )
    print(
        f"\nmodel {t['model']:.4f} kg  target {t['target']:.6f} kg  gap {t['gap']:+.4f} kg ({t['gap_pct']:+.1f} %)  "
        f"tolerance {t['tolerance']:.4f} kg  estimated share {t['estimated_share']:.3f}  "
        f"(in scope: screws {screws * 1000:.1f} g, cables {cables * 1000:.1f} g)"
    )
    assert abs(t["gap"]) == pytest.approx(abs(t["model"] - t["target"]))
    assert t["within"], (
        f"gap {t['gap']:+.4f} kg exceeds the tolerance {t['tolerance']:.4f} kg"
    )


# ---- 4. every geom is a real part --------------------------------------------------------------------------------------------
def test_every_geom_maps_to_a_library_part(built):
    _, _, geoms = mjcf_tree(built["xml"])
    names = [g.get("name") for g in geoms if g.get("name") != "floor"]
    assert len(names) == len(set(names)) and len(names) > 50
    rep = built["report"]["geoms"]
    lib = design.Library(PARTS)
    assert set(names) == set(rep)
    for n in names:
        assert rep[n] in lib.parts, f"geom {n}: part {rep[n]} not in the library"
    assert [g.get("name") for g in geoms].count("floor") == 1


# ---- 5. deterministic ---------------------------------------------------------------------------------------------------------
def test_build_is_byte_identical(tmp_path):
    a = design.build(
        ASM, PARTS, tmp_path / "a"
    )  # sibling dirs: same relative mesh paths
    b = design.build(ASM, PARTS, tmp_path / "b")
    assert a["xml"].read_bytes() == b["xml"].read_bytes()
    assert a["report_path"].read_bytes() == b["report_path"].read_bytes()


# ---- 6. joint frames = URDF forward kinematics at q = 0 ---------------------------------------------------------------------------
def test_joint_frames_match_the_urdf(built):
    """World frame = URDF base_link. The builder's hinge sits at the child horn seat's port origin = the servo's `output`
    port, which tools/so101_ports.py derived as the mean over the 6 joints, so each joint carries that averaging error
    (the ports file documents origin spread 1.1e-7 m and axis spread 6.7e-6 rad, both URDF rounding). The point is
    compared as its distance from the URDF joint axis line: the builder puts it on the horn face, 36.4 mm along the axis
    from the URDF joint origin, which does not change the hinge."""
    _, joints, _ = mjcf_tree(built["xml"])
    links, ujoints = so101_lib.parse_urdf()
    T = {"base_link": np.eye(4)}
    while len(T) < len(ujoints) + 1:  # file order is not parents-first
        for u in ujoints.values():
            if u["parent"] in T and u["child"] not in T:
                T[u["child"]] = T[u["parent"]] @ u["T"]
    worst_p = worst_a = 0.0
    print(f"\n{'joint':14s} {'position m':>11s} {'axis rad':>10s}")
    for j in JOINTS:
        u = ujoints[j]
        assert u["type"] == "revolute"
        path, pos, axis, rng = joints[j]
        Tc = T[u["child"]]
        want_axis = Tc[:3, :3] @ np.array(u["axis"])
        d = pos - Tc[:3, 3]
        dp = float(np.linalg.norm(d - (d @ want_axis) * want_axis))  # off the axis line
        assert (
            abs(d @ want_axis) < 0.05
        )  # the builder's hinge point is on the horn face, along the axis
        da = math.acos(min(1.0, float(axis @ want_axis) / np.linalg.norm(axis)))
        worst_p, worst_a = max(worst_p, dp), max(worst_a, da)
        print(f"{j:14s} {dp:11.3e} {da:10.3e}")
        assert rng == pytest.approx(
            [u["lower"], u["upper"]], abs=1e-9
        )  # limits = URDF limits
    print(f"worst: {worst_p:.3e} m, {worst_a:.3e} rad")
    # 1e-9 is not reachable by construction: one shared servo `output` port serves all six joints. The residual is the
    # URDF's own rounding: its joint origins carry 6 significant digits (<= 5e-8 m on a 0.1 m offset, 1.6e-7 m seen) and its
    # rpy values are the rounded 1.5708 (pi/2 - 1.5708 = 3.67e-6 rad; 6.7e-6 rad seen). Bounds: 2e-7 m, 1e-5 rad.
    assert worst_p < 2e-7 and worst_a < 1e-5
