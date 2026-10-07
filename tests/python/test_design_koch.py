"""assemblies/koch.toml (Koch v1.1 follower, parts/arm): DRC, editing API, build, kernel load, joint frames against the URDF, mass,
holding pose. Mirrors test_design_so101.py / test_design_aos5.py."""

import math
import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design
from napl.design import Design, Library

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import so101_lib
from test_design_so101 import mjcf_tree, static_torques

ASM = REPO / "assemblies" / "koch.toml"
PARTS = REPO / "parts"
URDF = REPO / "meshes" / "koch" / "follower.urdf"
JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
URDF_JOINTS = [f"joint_{i}" for i in range(1, 7)]
SERVOS = [f"servo{i}" for i in range(1, 7)]
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("koch")
    r = design.build(ASM, PARTS, out)
    return {"xml": r["xml"], "report": r["report"], "dir": out}


# ---- 1. DRC through the editing API ----------------------------------------------------------------------------------------
def test_drc_has_no_errors_and_the_expected_warnings():
    d = Design.open(ASM)
    fs = d.check()
    assert [f for f in fs if f.severity == "error"] == [], [
        (f.code, f.message) for f in fs if f.severity == "error"
    ]
    # Every warning is a worst-case (all servos stalled) current above a connector / supply rating, justified in assemblies/koch.toml:
    #  CURRENT_PEAK_OVER_RATING x2: cable_servo3_servo4 (3 XL330 stalled = 4.4 A vs the JST EH 3 A figure), lead_reducer_servo3 (4 XL330
    #    stalled = 5.9 A vs the 22 AWG lead's 3 A); sustained holding current is 0.2 / 0.5 A.
    #  PSU_OVERLOAD: the reducer (3.5 A rated, 6 A peak per the product title) under 4 stalled XL330 (5.9 A); sustained 0.5 A.
    warn = sorted((f.code, tuple(f.where)) for f in fs if f.severity == "warning")
    assert [c for c, _ in warn] == [
        "CURRENT_PEAK_OVER_RATING",
        "CURRENT_PEAK_OVER_RATING",
        "PSU_OVERLOAD",
    ], warn
    assert {w for _, w in warn} == {
        ("cable_servo3_servo4", "wire part robotis-robot-cable-x3p-180mm"),
        ("lead_reducer_servo3", "wire part awg-22-silicone-wire"),
        ("reducer",),
    }
    # the torque rule ran: no servo is over its stall torque at any sampled pose; the tightest one has margin > 1
    st = next(f for f in fs if f.code == "SERVO_TORQUE")
    assert st.severity == "info" and "servo3" in st.message
    margin = float(st.message.split("margin ")[1].split(")")[0])
    assert margin > 3.0, (
        st.message
    )  # 0.127 N*m worst gravity torque vs 0.52 N*m stall at 5 V
    assert "DRC_INTERNAL_ERROR" not in {f.code for f in fs}
    assert "INTERFERENCE" not in {
        f.code for f in fs
    }  # the servo bays are cavities of the printed meshes


def test_the_interference_rule_would_fire_here():
    """Negative control: a second elbow_to_wrist link on the same horn overlaps the first one, so the clean DRC above is not vacuous."""
    d = Design.open(ASM)
    d.add("koch-elbow-to-wrist", id="dup")
    r = d.mate("servo4.output", "dup.horn_seat", name="dup_joint")
    assert any(
        f["code"] == "INTERFERENCE" and set(f["where"]) == {"dup", "elbow_to_wrist"}
        for f in r.new
    )


def test_editing_api_reproduces_and_swaps(tmp_path):
    d = Design.open(ASM)
    base = {(f.code, tuple(f.where)) for f in d.check()}
    p = tmp_path / "koch_saved.toml"
    d.save(p)  # deterministic TOML (comments are not kept)
    assert design.load(p).doc == design.load(ASM).doc
    assert {(f.code, tuple(f.where)) for f in Design.open(p).check()} == base
    # the XL330-M077 shares the XL330 body (same envelope, mass and horn / base ports): swapping the elbow servo keeps every mate
    r = d.swap("servo3", "robotis-xl330-m077-t")
    c = r.changes[0]
    assert c["mates"]["lost"] == [] and len(c["mates"]["kept"]) == 2
    # the only thing the M077 file lacks is the device-schema params of the M288 file (it is the leader servo, not bound here)
    assert c["dangling"] and all("device_" in x for x in c["dangling"]), c["dangling"]


# ---- 2. build + kernel load ------------------------------------------------------------------------------------------------
@needs_kernel
def test_builds_and_loads_in_the_kernel(built):
    dev, joints, _ = mjcf_tree(built["xml"])
    assert [s["name"] for s in dev["servos"]] == SERVOS
    assert [s["joint"] for s in dev["servos"]] == JOINTS
    assert [p["name"] for p in dev["psus"]] == ["psu", "reducer"]
    assert (
        len(dev["wires"]) == 9
    )  # PSU cord, adapter edge, 2 board leads... see assemblies/koch.toml
    assert set(JOINTS) == set(joints)
    s = {x["name"]: x for x in dev["servos"]}
    # one device map per servo model: gains and scales come from the part params
    for n in ("servo1", "servo2"):
        assert (
            s[n]["p"],
            s[n]["i"],
            s[n]["d"],
            s[n]["pid_scale"],
            s[n]["d_scale"],
        ) == (640, 0, 4000, 113280, 14160)
        assert (s[n]["max_temp"], s[n]["unload_mask"], s[n]["encoder_cpr"]) == (
            72,
            52,
            4096,
        )
    for n in ("servo3", "servo4", "servo5", "servo6"):
        assert (
            s[n]["p"],
            s[n]["i"],
            s[n]["d"],
            s[n]["pid_scale"],
            s[n]["d_scale"],
        ) == (400, 0, 0, 113280, 14160)
        assert (s[n]["max_temp"], s[n]["unload_mask"], s[n]["encoder_cpr"]) == (
            70,
            53,
            4096,
        )
    sim = napl.Sim.load(built["xml"])
    for name in ["psu", "reducer", *SERVOS]:
        sim.query(f"/devices/{name}", "voltage")
    for j in JOINTS:
        assert sim.query(joints[j][0], "qpos")[0][0] == 0.0
    assert not sim.diverged


def test_build_is_byte_identical(tmp_path):
    a = design.build(ASM, PARTS, tmp_path / "a")
    b = design.build(ASM, PARTS, tmp_path / "b")
    assert a["xml"].read_bytes() == b["xml"].read_bytes()
    assert a["report_path"].read_bytes() == b["report_path"].read_bytes()


# ---- 3. joint frames = URDF forward kinematics at q = 0 --------------------------------------------------------------------
def test_joint_frames_and_limits_match_the_urdf(built):
    """World = URDF base_link. The builder puts the hinge at the child horn seat's port origin (the horn outer face = the URDF joint
    origin by construction of tools/koch_ports.py): position compared as the distance from the URDF joint axis line, axis directly
    (reverse = true on every mate turns the servo output axis into the URDF joint axis). Ranges = the part `<joint>_range` params."""
    _, joints, _ = mjcf_tree(built["xml"])
    _, uj = so101_lib.parse_urdf(URDF)
    T = {"base_link": np.eye(4)}
    while len(T) < len(uj) + 1:
        for u in uj.values():
            if u["parent"] in T and u["child"] not in T:
                T[u["child"]] = T[u["parent"]] @ u["T"]
    lib = Library(PARTS)
    ranges = {  # mate -> (child part, param)
        "shoulder_pan": ("koch-shoulder-rotation", "shoulder_pan_range"),
        "shoulder_lift": ("koch-shoulder-to-elbow", "shoulder_lift_range"),
        "elbow_flex": ("koch-elbow-to-wrist-extension", "elbow_flex_range"),
        "wrist_flex": ("koch-elbow-to-wrist", "wrist_flex_range"),
        "wrist_roll": ("koch-gripper-static", "wrist_roll_range"),
        "gripper": ("koch-gripper-moving", "gripper_range"),
    }
    worst_p = worst_a = 0.0
    for j, uname in zip(JOINTS, URDF_JOINTS):
        u = uj[uname]
        _, pos, axis, rng = joints[j]
        Tc = T[u["child"]]
        want = Tc[:3, :3] @ np.array(u["axis"])
        d = pos - Tc[:3, 3]
        dp = float(np.linalg.norm(d - (d @ want) * want))
        da = math.acos(min(1.0, float(axis @ want) / np.linalg.norm(axis)))
        worst_p, worst_a = max(worst_p, dp), max(worst_a, da)
        assert abs(d @ want) < 0.05  # on the horn face, along the axis
        part, key = ranges[j]
        assert rng == pytest.approx(lib.get(part).params[key]["value"], abs=1e-9)
    print(f"\nworst joint frame error vs URDF: {worst_p:.3e} m, {worst_a:.3e} rad")
    assert (
        worst_p < 1e-6 and worst_a < 1e-6
    )  # ports are closed to 3e-17 m by tools/koch_ports.py --check


# ---- 4. mass -----------------------------------------------------------------------------------------------------------------
def test_mass_budget_and_bom_estimate(built):
    rep = built["report"]
    _, _, geoms = mjcf_tree(built["xml"])
    total = rep["mass"]["total"]
    assert math.isclose(sum(rep["mass"]["by_instance"].values()), total, rel_tol=1e-12)
    gm = sum(float(g.get("mass")) for g in geoms if g.get("name") != "floor")
    assert math.isclose(gm, total, rel_tol=1e-9)
    by = rep["mass"]["by_instance"]
    printed = sum(
        by[i]
        for i in (
            "base",
            "shoulder_rotation",
            "shoulder_to_elbow",
            "elbow_to_wrist_extension",
            "elbow_to_wrist",
            "gripper_static",
            "gripper_moving",
        )
    )
    servos = sum(by[s] for s in SERVOS)
    assert servos == pytest.approx(2 * 0.0572 + 4 * 0.018)  # e-Manual masses
    assert printed == pytest.approx(
        0.10056, abs=2e-4
    )  # docs/research/koch-v1.1.md table
    (t,) = rep["targets"]
    print(
        f"\nprinted {printed * 1000:.1f} g  servos {servos * 1000:.1f} g  model total {total * 1000:.1f} g "
        f"(arm group {t['model'] * 1000:.1f} g)  BOM estimate {t['target'] * 1000:.1f} g  gap {t['gap'] * 1000:+.1f} g ({t['gap_pct']:+.1f} %)"
    )
    assert t["scope"] == ["arm"] and t["target"] == pytest.approx(0.28696)
    assert 0.28 < total < 0.33
    # the model adds the four servo-to-servo cables (3.2 g each) on top of the estimate
    assert t["gap"] == pytest.approx(0.0167, abs=0.002) and t["within"]


# ---- 5. holding pose -------------------------------------------------------------------------------------------------------------
@needs_kernel
def test_holds_the_zero_pose_for_2_s_with_no_alarm(built):
    dev, joints, _ = mjcf_tree(built["xml"])
    cfg = {s["name"]: s for s in dev["servos"]}
    sim = napl.Sim.load(built["xml"])
    for n in SERVOS:
        sim.set(f"/devices/{n}", "goal_position", 0.0)
        sim.set(f"/devices/{n}", "torque_enable", 1)
    grav = static_torques(
        built["xml"], set(JOINTS)
    )  # independent of the servo law: MJCF masses and centres at q = 0
    sim.step(
        1800
    )  # 1.8 s at 1 ms, then 0.2 s averaged: the XL430 D term chatters, see below
    q = lambda n, a: float(sim.query(f"/devices/{n}", a)[0][0])
    acc = {n: [0.0, 0.0, 0.0] for n in SERVOS}  # sum |torque|, sum torque, max alarm
    for _ in range(200):
        sim.step(1)
        for n in SERVOS:
            a = acc[n]
            a[0] += abs(q(n, "torque"))
            a[1] += q(n, "torque")
            a[2] = max(a[2], q(n, "alarm"))
    assert not sim.diverged
    print(
        f"\n{'joint':14s} {'q deg':>8s} {'tol deg':>8s} {'mean tau':>9s} {'gravity':>8s} {'duty':>7s} {'V':>6s} {'T degC':>7s} alarm"
    )
    for n, j in zip(SERVOS, JOINTS):
        c, m = cfg[n], cfg[n]["motor"]
        qj = float(sim.query(joints[j][0], "qpos")[0][0])
        Kt = 60 / (2 * math.pi * m["kv"])
        V, duty = q(n, "voltage"), q(n, "duty")
        tau = acc[n][1] / 200
        # Tolerance from the model, not a chosen bound (as test_design_so101): holding the INDEPENDENT gravity torque needs the static duty
        # |tau| R / (eta N Kt V) = p e / pid_scale, i.e. e = duty pid_scale / p counts of the proportional term (I = 0, D acts on the
        # error change); x 1.25 because the arm sags to a pose whose torque is a little different from the q = 0 value, + the dead zone
        # + 2 counts (the +-1 count chatter of the D term and the encoder quantisation).
        e_counts = (
            1.25
            * abs(grav[j])
            * m["r20"]
            / (c["efficiency"] * c["gear_ratio"] * Kt * V)
            * c["pid_scale"]
            / c["p"]
            + c["deadband"]
            + 2
        )
        tol = e_counts * 2 * math.pi / c["encoder_cpr"]
        print(
            f"{j:14s} {math.degrees(qj):8.3f} {math.degrees(tol):8.3f} {tau:9.4f} {grav[j]:8.4f} {duty:7.4f} {V:6.3f} {q(n, 'temperature'):7.2f} {acc[n][2]:.0f}"
        )
        assert abs(qj) <= tol, f"{j}: |q| {abs(qj):.4f} rad > model tolerance {tol:.4f}"
        assert abs(qj) < math.radians(8.0), f"{j} sags {math.degrees(qj):.2f} deg"
        if (
            abs(grav[j]) > 0.02
        ):  # a real load: the servo carries the gravity torque on average (not instantaneously, see below)
            assert abs(tau) == pytest.approx(abs(grav[j]), rel=0.25), (j, tau, grav[j])
        assert acc[n][2] == 0, f"{n} alarm"
        assert q(n, "temperature") < c["max_temp"]
        assert q(n, "torque_enable") == 1
    assert 11.0 < q("servo1", "voltage") <= 12.0  # 12 V supply through board + lead
    assert 4.5 < q("servo3", "voltage") <= 5.3  # reducer output
