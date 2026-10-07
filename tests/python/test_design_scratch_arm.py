"""S5: the SO-101 follower built FROM AN EMPTY DESIGN through the editing API alone (the arm counterpart of test_design_scratch.py):
Design.new, add(root=True), parts from suggest(category=), mechanics by add-to-port snapping with joint specs (port names and the
mate rotation k, never poses), the servo bus daisy-chained by wire / autowire, servo IDs by `set`, devices from auto_devices.
assemblies/so101.toml is loaded only for the comparison. Also the arm design rules (servo bus, ARM_REACH), suggest() for a joint,
and `napl program` of examples/so101's pick scenario against the saved design.
Needs the kernel for the simulation tests (hfbox: tools/remote.sh run --nolock '.venv/bin/python -m pytest tests/python/test_design_scratch_arm.py -v -s')."""

import copy
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import pytest
from napl import program
from napl.design import Design, DesignError, drc

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
SO101 = REPO / "assemblies" / "so101.toml"
KOCH = REPO / "assemblies" / "koch.toml"
PICK = REPO / "examples" / "so101" / "pick.scenario.toml"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")

SERVO, CABLE, M2, M3 = (
    "feetech-sts3215-c001",
    "sts3215-servo-cable-3pin",
    {"part": "so101-screw-m2x6", "count": 4},
    {"part": "so101-screw-m3x6", "count": 1},
)
# (joint = mate name, child part, child instance): joint k is driven by servo k, whose horn carries the child
JOINTS = [
    ("shoulder_pan", "so101-rotation-pitch", "rotation_pitch"),
    ("shoulder_lift", "so101-upper-arm", "upper_arm"),
    ("elbow_flex", "so101-lower-arm", "lower_arm"),
    ("wrist_flex", "so101-wrist-roll-pitch", "wrist_roll_pitch"),
    ("wrist_roll", "so101-gripper-fixed-jaw", "fixed_jaw"),
    ("gripper", "so101-moving-jaw", "moving_jaw"),
]


def est(v, unit, why):
    return {"value": v, "unit": unit, "fidelity": "estimated", "rationale": why}


REF = "same value as assemblies/so101.toml (the reference this build is compared with)"
PID_SCALE = {
    "value": 4000,
    "unit": "1",
    "fidelity": "derived",
    "derivation": "Register units -> duty: Feetech memory table P (1/4) with the 0.1 % duty unit = 4 x 1000 "
    "(docs/research/sts3215-calibration.md); " + REF,
}
C_TH = est(
    10.0,
    "J/K",
    "no published winding thermal capacity; the arm3.xml test value. " + REF,
)
R_TH = est(
    15.0,
    "K/W",
    "no published winding-to-ambient resistance; the arm3.xml test value. " + REF,
)
R_BOARD = est(
    0.02,
    "ohm",
    "the part publishes no series resistance: DC jack contact pair + a short trace, two mated 5264 contacts. "
    + REF,
)


def ids(rows):
    return [r["part"] for r in rows]


def pick(rows, want, why):
    """Choose from a library query / suggest() result; the choice must be one of the candidates the API offered."""
    assert want in ids(rows), f"{want} not offered for {why}: {ids(rows)[:8]}"
    return want


def build_arm(parts=PARTS):
    """-> the SO-101 follower as a Design, from nothing."""
    d = Design.new(
        "scratch-so101", parts, base="fixed", root_frame="urdf_link",
        description="SO-101 follower built from an empty design",
    )  # fmt: skip
    # ---- 1. base: the root part, then everything snaps to its named ports ----------------------------------------------------------
    d.add("so101-base", id="base", root=True, group="arm")
    d.add(
        "so101-base-motor-holder",
        id="base_motor_holder",
        to="base.motor_holder_seat",
        group="arm",
    )
    d.add(
        "so101-waveshare-mounting-plate", id="plate", to="base.plate_seat", group="arm"
    )
    adapter = pick(
        d.suggest("plate.board_seat", category="pcb.servo_driver"),
        "waveshare-bus-servo-adapter-a",
        "plate.board_seat",
    )
    d.add(adapter, id="adapter", to="plate.board_seat", group="electronics")
    # ---- 2. six joints: servo on the parent's servo seat (k = 0), then the child on the servo's horn with the joint spec -----------------
    seat = "base.servo_seat"
    for k, (joint, child, cid) in enumerate(JOINTS, 1):
        servo = pick(d.suggest(seat, category="motor.servo"), SERVO, seat)
        d.add(servo, id=f"servo{k}", to=seat, port="base", k=0, group="arm",
              fasteners={**M2, "count": 2 if k == 5 else 4})  # fmt: skip
        pick(
            d.suggest(f"servo{k}.output", category="frame.printed"),
            child,
            f"servo{k}.output",
        )
        d.add(child, id=cid, to=f"servo{k}.output", port="horn_seat", k=0, group="arm", mate=joint,
              joint={"type": "hinge", "range": f"{cid}.{joint}_range"}, fasteners=M3)  # fmt: skip
        seat = f"{cid}.servo_seat"
        if joint == "shoulder_pan":
            d.add(
                "so101-motor-holder-shoulder",
                id="motor_holder_shoulder",
                to="rotation_pitch.holder_seat",
                group="arm",
            )
        if joint == "elbow_flex":
            d.add(
                "so101-motor-holder-wrist",
                id="motor_holder_wrist",
                to="lower_arm.holder_seat",
                group="arm",
            )
    # ---- 3. power: the PSU (offboard) and its cord, then the servo bus ---------------------------------------------------------------
    psu = pick(
        d.suggest("adapter.power_in", category="psu"),
        "facmogu-5v-4a-psu",
        "adapter.power_in",
    )
    d.add(psu, id="psu", offboard=True, group="bench")
    d.autowire(
        "psu.power_out", "adapter.power_in", included_in="psu",
        length=est(1.5, "m", "the supply's own fixed cord, length not published; 1.2-1.8 m is typical. " + REF),
    )  # fmt: skip
    d.add(
        CABLE, id="cable", offboard=True, group="bench"
    )  # holds the cable part's params (length_default)
    # the adapter's servo port declares v_max 12.6 V (its DC input range) above the servo's 7.4 V: autowire refuses that pair, so
    # the first hop is an explicit wire; the 5 hops servo(k).port_b -> servo(k+1).port_a are autowired (passthrough ports)
    d.wire(
        "adapter.servo_bus",
        "servo1.port_a",
        CABLE,
        "cable.length_default",
        conductors=2,
    )
    for k in range(1, 6):
        d.autowire(f"servo{k}.port_b", f"servo{k + 1}.port_a", part=CABLE)
    # ---- 4. bus IDs: the position along the chain (LeRobot: id = joint order) -------------------------------------------------------
    for k in range(1, 7):
        d.set(f"servo{k}", "servo_id", k, unit="1", fidelity="derived",
              derivation=f"servo {k} of the daisy chain adapter -> servo1 -> ... -> servo6; LeRobot SO-101 id = joint order")  # fmt: skip
    # ---- 5. devices from the parts + topology; the values no part carries are the `fill` ---------------------------------------------
    fill = {"adapter_board": {"set": {"resistance": R_BOARD}}}
    for k in range(1, 7):
        fill[f"servo{k}"] = {
            "set": {
                "pid_scale": PID_SCALE, "motor.c_th": C_TH, "motor.r_th": R_TH,
                "p": f"servo{k}.reg_p_lerobot_so101", "i": f"servo{k}.reg_i_lerobot_so101", "d": f"servo{k}.reg_d_lerobot_so101",
                **({"max_torque": "servo6.lerobot_gripper_max_torque_limit"} if k == 6 else {}),
            }
        }  # fmt: skip
    d.auto_devices(fill=fill)
    return d


@pytest.fixture(scope="module")
def arm():
    return build_arm()


def fork(d):
    """An independent copy of a design (the module-scoped one stays as built)."""
    return Design(type(d.asm)(copy.deepcopy(d.doc)), d.lib)


def _codes(fs, sev):
    return {f.code for f in fs if f.severity == sev}


def _one(fs, code):
    (f,) = [f for f in fs if f.code == code]
    return f


def test_built_from_empty_with_no_toml_and_drc_has_no_errors(arm):
    assert arm.path is None  # never opened from a file
    fs = arm.check()
    assert [f for f in fs if f.severity == "error"] == [], [
        (f.code, f.where, f.message[:100]) for f in fs if f.severity == "error"
    ]
    # the warnings of the reference arm (assemblies/so101.toml, tests/python/test_drc.py) plus the two worst-case bus rules
    assert _codes(fs, "warning") <= {
        "FASTENER_SHORT", "VOLTAGE_BELOW_MIN", "PSU_OVERLOAD", "SERVO_TORQUE", "SERVO_TORQUE_MARGIN",
        "CURRENT_PEAK_OVER_RATING", "SERVO_BUS_CURRENT", "SERVO_CHAIN_LENGTH",
    }, _codes(fs, "warning")  # fmt: skip
    assert (
        len(arm.doc["wires"]) == 1 + 1 + 5
    )  # PSU cord, adapter -> servo1, five servo-to-servo hops
    assert len([b for b in arm.doc["devices"] if b["type"] == "servo"]) == 6
    assert all(b.get("auto") for b in arm.doc["devices"])
    assert "SERVO_ID_UNSET" not in {f.code for f in fs}  # every servo has its ID
    print()
    for f in fs:
        if f.severity != "info":
            print(f"  {f.severity:7s} {f.code:22s} {f.where} {f.message[:110]}")


# ---- numbers against assemblies/so101.toml -------------------------------------------------------------------------------------
def _mjcf(xml):
    root = ET.parse(xml).getroot()
    dev = json.loads(root.find("custom/text[@name='napl.devices']").get("data"))
    rng = {
        j.get("name"): [float(x) for x in j.get("range").split()]
        for j in root.iter("joint")
        if j.get("range")
    }
    return dev, rng


@pytest.fixture(scope="module")
def numbers(arm, tmp_path_factory):
    out = {}
    for name, des in (("scratch", arm), ("so101", Design.open(SO101))):
        b = des.build(tmp_path_factory.mktemp(name))
        dev, rng = _mjcf(b["xml"])
        out[name] = {"mass": b["report"]["mass"]["total"], "dev": dev, "range": rng}
        if napl._napl is not None:
            out[name]["hold"] = des.simulate(
                "hold", out=tmp_path_factory.mktemp(name + "-h")
            )
    return out


def test_mass_limits_and_servo_devices_match_the_reference(numbers):
    s, r = numbers["scratch"], numbers["so101"]
    print(
        f"\nmass: scratch {s['mass']:.5f} kg  so101.toml {r['mass']:.5f} kg  ({100 * (s['mass'] / r['mass'] - 1):+.3f} %)"
    )
    assert abs(s["mass"] / r["mass"] - 1) < 0.005
    assert s["range"] == pytest.approx(
        r["range"], abs=1e-9
    )  # joint limits: the same params of the same printed parts
    assert set(s["range"]) == {j for j, _, _ in JOINTS}
    # the generated servo devices carry the reference's numbers (same params, mapped by auto_devices instead of by hand)
    for a, b in zip(s["dev"]["servos"], r["dev"]["servos"]):
        assert a.pop("motor") == pytest.approx(b.pop("motor"), rel=1e-9), b["name"]
        (
            a.pop("node"),
            b.pop("node"),
        )  # electrical node numbers depend on the order of the wires, not on the circuit
        assert a == pytest.approx(b, rel=1e-9), b["name"]
    # the circuit: the same resistances (cord, adapter board, six cables), the same supply
    res = lambda m: sorted(
        (
            w.get("awg", 0),
            w.get("resistance", 0),
            w.get("length_m", 0),
            w.get("conductors", 0),
        )
        for w in m["dev"]["wires"]
    )
    # (awg, resistance, length, conductors): equal except that autowire sizes the two longer hops to the port distance x 1.3
    # (0.165 m and 0.18 m) where the reference keeps the stocked 0.15 m "although the geometric hops are longer"
    assert len(res(s)) == len(res(r)) == 8
    for (ga, gr, gl, gc), (ha, hr, hl, hc) in zip(res(s), res(r)):
        assert (ga, gr, gc) == (ha, hr, hc) and 0 <= gl - hl <= 0.031, (res(s), res(r))
    assert s["dev"]["psus"][0]["i_limit"] == r["dev"]["psus"][0]["i_limit"]


@needs_kernel
def test_hold_passes_and_sag_and_current_match_the_reference(numbers):
    s, r = numbers["scratch"]["hold"], numbers["so101"]["hold"]
    assert s["pass"] and r["pass"], (s, r)
    print(
        f"\n{'joint':14s} {'sag scratch':>12s} {'sag so101':>10s} {'I scratch':>10s} {'I so101':>8s}"
    )
    for a, b in zip(s["servos"].values(), r["servos"].values()):
        print(
            f"{a['joint']:14s} {a['sag_max_rad']:12.4f} {b['sag_max_rad']:10.4f} {a['current_max_A']:10.4f} {b['current_max_A']:8.4f}"
        )
        assert abs(a["sag_max_rad"] - b["sag_max_rad"]) < 0.005  # 0.3 deg
        assert a["current_max_A"] == pytest.approx(
            b["current_max_A"], rel=0.02, abs=1e-3
        )
    assert s["sag_max_rad"] == pytest.approx(r["sag_max_rad"], abs=0.005)
    assert s["psu_current_A"] == pytest.approx(r["psu_current_A"], rel=0.02)


def test_save_and_reload_give_the_identical_drc(arm, tmp_path):
    d2 = Design.open(arm.save(tmp_path / "scratch-so101.toml"))
    key = lambda fs: sorted((f.code, f.severity, tuple(f.where), f.message) for f in fs)
    assert key(d2.check()) == key(arm.check())


# ---- arm rules 1: the servo bus ---------------------------------------------------------------------------------------------------
def new(r, code):
    return [f for f in r.new if f["code"] == code]


def sset(d, inst, param, v, unit):
    return d.set(
        inst,
        param,
        v,
        unit=unit,
        fidelity="estimated",
        rationale="seeded by tests/python/test_design_scratch_arm.py",
    )


def test_servo_id_conflict_and_range(arm):
    d = fork(arm)
    (f,) = new(
        sset(d, "servo3", "servo_id", 2, "1"), "SERVO_ID_CONFLICT"
    )  # servo2 already has 2
    assert f["severity"] == "error" and f["where"] == ["servo2", "servo3"]
    assert (
        "share bus ID 2" in f["message"] and "e.g. 3" in f["fix"]
    )  # 3 is the first free ID (servo3 gave it up)
    (g,) = new(
        sset(fork(arm), "servo3", "servo_id", 254, "1"), "SERVO_ID_CONFLICT"
    )  # 254 = broadcast
    assert g["severity"] == "error" and "outside 0..252" in g["message"]


def test_unset_ids_on_a_shared_bus_are_reported_not_invented(arm):
    d = fork(arm)
    for k in range(1, 7):
        d.set(f"servo{k}", "servo_id", unset=True)
    fs = d.check()
    f = _one(fs, "SERVO_ID_UNSET")
    assert (
        f.severity == "info"
        and f.where == [f"servo{k}" for k in range(1, 7)]
        and "factory ID" in f.message
    )
    assert "SERVO_ID_CONFLICT" not in {x.code for x in fs}
    # the stock reference arm declares no IDs either: reported, no error (its file is not touched)
    ref = Design.open(SO101).check()
    assert _one(ref, "SERVO_ID_UNSET").severity == "info" and not [
        f for f in ref if f.severity == "error"
    ]


def test_servo_baud_mismatch(arm):
    d = fork(arm)
    (f,) = new(sset(d, "servo4", "baud", 115200, "bit/s"), "SERVO_BAUD_MISMATCH")
    assert (
        f["severity"] == "error"
        and "115200 bit/s: servo4" in f["message"]
        and "1000000 bit/s: servo1" in f["message"]
    )


def test_bus_current_budget_vs_the_adapter_rating(arm):
    f = _one(
        arm.check(), "SERVO_BUS_CURRENT"
    )  # 6 x 1.67 A at 5 V (stall) against the adapter's max_continuous_current 5 A
    assert (
        f.severity == "warning"
        and "10.0 A" in f.message
        and "max_continuous_current 5 A" in f.message
    )
    assert "1.8 A holding" in f.message and "rated >= 10 A" in f.fix
    d = fork(arm)
    r = sset(
        d, "adapter", "max_continuous_current", 12.0, "A"
    )  # a board that carries all six at stall
    assert [x["code"] for x in r.resolved] == ["SERVO_BUS_CURRENT"]
    r = sset(
        fork(arm), "adapter", "max_continuous_current", 1.0, "A"
    )  # below even the holding current: error
    assert [
        (c["before"]["severity"], c["after"]["severity"])
        for c in r.changed
        if c["after"]["code"] == "SERVO_BUS_CURRENT"
    ] == [("warning", "error")]


def test_chain_length_drop_matches_an_independent_sum(arm, tmp_path):
    """All servos stalled: the drop to servo6 = sum over the wires on the path of (current through it) x (ohm/m x length x
    conductors), the PSU cord included. Computed here from the built device list (awg, length, conductors per wire)."""
    f = _one(arm.check(), "SERVO_CHAIN_LENGTH")
    assert (
        f.severity == "warning"
        and f.where == ["servo6"]
        and "7 cable hops" in f.message
        and "below the servo's minimum 4 V" in f.message
    )
    part = arm.lib.get(SERVO)
    i_s = drc._at_voltage(
        drc._servo_points(part), 5.0, 2
    )  # stall current of one servo at the 5 V supply
    dev, _ = _mjcf(arm.build(tmp_path)["xml"])
    r = lambda w: (
        arm.lib.get(w["parts"][0]).value("resistance_per_m")
        * w["length_m"]
        * w["conductors"]
    )
    by = {w["name"]: r(w) for w in dev["wires"] if "conductors" in w}
    hops = ["psu.power_out--adapter.power_in", "adapter.servo_bus--servo1.port_a"] + [
        f"servo{k}.port_b--servo{k + 1}.port_a" for k in range(1, 6)
    ]
    expect = sum(by[h] * i_s * n for h, n in zip(hops, [6, 6, 5, 4, 3, 2, 1]))
    got = float(re.search(r"cables drop ([\d.]+) V", f.message).group(1))
    print(
        f"\nchain drop to servo6: rule {got:.3f} V, independent {expect:.3f} V (stall {i_s:.3f} A per servo)"
    )
    assert got == pytest.approx(expect, abs=0.006)
    assert float(re.search(r"leaving ([\d.]+) V", f.message).group(1)) == pytest.approx(
        5.0 - expect, abs=0.01
    )


def test_chain_longer_than_the_board_drives_is_an_error(arm):
    r = sset(fork(arm), "adapter", "max_servos", 5, "1")
    (f,) = [x for x in new(r, "SERVO_CHAIN_LENGTH") if x["severity"] == "error"]
    assert "6 servos on a bus whose board (adapter) drives at most 5" in f["message"]


def test_the_koch_buses_raise_none_of_the_bus_rules():
    codes = {
        f.code for f in Design.open(KOCH).check()
    }  # 2 XL430 at 12 V + 4 XL330 at 5 V, no declared IDs
    assert not codes & {
        "SERVO_ID_CONFLICT",
        "SERVO_BAUD_MISMATCH",
        "SERVO_BUS_CURRENT",
        "SERVO_CHAIN_LENGTH",
    }


# ---- arm rules 2: ARM_REACH ---------------------------------------------------------------------------------------------------------
def pt(v):
    return est(v, "m", "seeded by tests/python/test_design_scratch_arm.py")


def reach(d, name, pts=None, box=None):
    r = d.target(
        name,
        reach=[pt(p) for p in pts] if pts else None,
        workspace=pt(box) if box else None,
    )
    return new(r, "ARM_REACH")


def test_reach_ok_for_the_pick_workcell(arm):
    (f,) = reach(
        fork(arm), "pick", [[0.20, -0.05, 0.0077], [0.20, 0.04, 0.0077]]
    )  # cube and target of examples/so101/pick.py
    assert f["severity"] == "info" and "all 2 point(s) reachable" in f["message"]


def test_unreachable_point_reports_closest_approach_and_the_fix(arm):
    (f,) = reach(fork(arm), "far", [[0.60, 0.0, 0.10]])
    assert f["severity"] == "warning" and f["where"] == ["far"]
    gap = float(re.search(r"closest approach (\d+) mm", f["message"]).group(1))
    print(f"\nfar: {f['message']}\nfix: {f['fix']}")
    assert (
        100 <= gap <= 120
        and f"lengthen a link by about {gap:.0f} mm or mount the base about {gap:.0f} mm closer"
        in f["fix"]
    )
    # the fix does what it says: a target `gap` closer (10 mm margin) is inside the workspace, this one is not
    (g,) = reach(fork(arm), "closer", [[0.60 - gap / 1000 - 0.01, 0.0, 0.10]])
    assert g["severity"] == "info"


def test_too_close_is_told_apart_from_too_far(arm):
    (f,) = reach(
        fork(arm), "box", box=[0.1, -0.1, 0.0, 0.3, 0.1, 0.2]
    )  # 3 x 3 x 3 grid; the corner next to the base cannot be folded to
    assert (
        f["severity"] == "warning" and "2 of 27 point(s) out of reach" in f["message"]
    )
    assert (
        "cannot fold that close" in f["fix"] and "farther from the target" in f["fix"]
    )


def test_reach_target_is_a_provenance_table_and_survives_save(arm, tmp_path):
    d = fork(arm)
    with pytest.raises(DesignError, match="bare number"):
        d.target(
            "bad", reach=[[0.2, 0.0, 0.1]]
        )  # numbers rule: a point is a provenance table, not a bare list
    reach(d, "far", [[0.60, 0.0, 0.10]])
    d2 = Design.open(d.save(tmp_path / "reach.toml"))
    assert d2.doc["targets"] == d.doc["targets"]
    assert [f.code for f in d2.check() if f.code == "ARM_REACH"] == ["ARM_REACH"]
    assert (
        d2.build(tmp_path / "b")["report"]["targets"] == []
    )  # a reach target is not a mass target


# ---- suggest() for an arm joint -------------------------------------------------------------------------------------------------------
def test_suggest_a_servo_for_a_joint_ranks_by_holding_margin(arm):
    rows = arm.suggest("shoulder_lift", limit=100)
    f = next(
        x
        for x in arm.check()
        if x.code == "SERVO_TORQUE_MARGIN" and "shoulder_lift" in x.where
    )
    now = next(r for r in rows if r["part"] == SERVO)
    print()
    for r in rows:
        print(
            f"  {r['part']:34s} margin {r['margin']:6.2f}  stall {r['stall_Nm']:.3f} N*m at {r['voltage_V']:g} V  fits {r['mates_fit']}  {r['why']}"
        )
    # the statics are the DRC's: same worst torque and margin as the SERVO_TORQUE_MARGIN finding of this arm
    assert now["tau_worst_Nm"] == pytest.approx(
        float(re.search(r"torque ([\d.]+) N\*m", f.message).group(1)), abs=1e-3
    )
    assert now["margin"] == pytest.approx(
        float(re.search(r"margin ([\d.]+)\)", f.message).group(1)), abs=0.005
    )
    assert (
        "the servo on this joint now" in now["why"][0]
        and "SERVO_TORQUE_MARGIN" in now["why"][0]
    )
    assert [r["margin"] for r in rows] == sorted(
        (r["margin"] for r in rows), reverse=True
    )
    assert all(
        r["voltage_V"] == 5.0 for r in rows
    )  # the bus voltage of this arm, for every candidate
    assert rows[0]["margin"] > now["margin"] >= 1  # a stronger servo is on offer ...
    assert any(
        r["margin"] >= 2 > now["margin"] for r in rows
    )  # ... one that clears the 2x rule
    weak = next(r for r in rows if r["part"] == "robotis-xl330-m288-t")
    assert weak["margin"] < 1 and "cannot hold" in weak["why"][0]
    assert {r["part"] for r in rows if r["mates_fit"]} >= {
        SERVO
    }  # the horn / seat of the servo now there
    assert not any(
        r["part"] == "robotis-xl430-w250-t" for r in rows
    )  # 6.5-12 V servo on a 5 V bus: not operable, not offered
    assert [r["part"] for r in arm.suggest("shoulder_lift", limit=3)] == [
        r["part"] for r in rows[:3]
    ]


def test_suggest_a_joint_that_does_not_exist_is_an_error(arm):
    with pytest.raises(DesignError, match="unknown instance|no instance"):
        arm.suggest("shoulder_roll")


# ---- save -> `napl program` the existing pick scenario against the saved design ---------------------------------------------------------
def pick_on(tmp_path, saved, servos=False):
    """examples/so101/pick.scenario.toml with [design] path = the saved design; the servo map and baud come from the design
    (the scenario's own `servos` / `baud` lines are dropped unless servos = True)."""
    ex = PICK.parent
    t = PICK.read_text().replace('"{dir}/pick.py"', f'"{ex / "pick.py"}"')
    t = t.replace(
        'script = "scene.py"',
        f'script = "{ex / "scene.py"}"\n[design]\npath = "{saved}"\n',
    )
    if not servos:
        t = re.sub(r"^(servos|baud) = .*\n", "", t, flags=re.MULTILINE)
    f = tmp_path / "pick-design.scenario.toml"
    f.write_text(t)
    return f


def test_scenario_takes_servo_ids_from_the_design(arm, tmp_path):
    saved = arm.save(tmp_path / "scratch-so101.toml")
    ifs = program.load_scenario(pick_on(tmp_path, saved))["interfaces"]["feetech"]
    assert (
        ifs["servos"] == {k: f"/devices/servo{k}" for k in range(1, 7)}
        and ifs["baud"] == 1_000_000
    )
    own = program.load_scenario(PICK)["interfaces"]["feetech"]
    assert {int(k): v for k, v in own["servos"].items()} == ifs[
        "servos"
    ]  # the hand-written map of the shipped scenario
    # a design without servo_id params (the stock reference) cannot supply the map
    with pytest.raises(program.ScenarioError, match="servo_id"):
        program.load_scenario(pick_on(tmp_path, SO101))
    # [design] without a feetech/px4 interface still has to be a saved design
    with pytest.raises(program.ScenarioError, match="path"):
        (tmp_path / "e.scenario.toml").write_text(
            '[design]\n[interfaces.feetech]\nservos = {1 = "/devices/servo1"}\n[program]\ncmd = ["python", "x.py"]\n'
        )
        program.load_scenario(tmp_path / "e.scenario.toml")


@needs_kernel
def test_pick_scenario_passes_on_the_scratch_built_arm_and_replays(arm, tmp_path):
    assert program.REPLAY.exists(), f"build the C++ exe first: {program.REPLAY}"
    saved = arm.save(tmp_path / "scratch-so101.toml")
    res, d = program.run(pick_on(tmp_path, saved), tmp_path)
    print(
        f"\n[pick on the scratch arm] {res['program']} sim {res['sim']['sim_s']:.1f} s"
    )
    for m in res["monitors"]:
        print(
            f"  {m['name']}: pass={m['pass']} min {m['min']:.4g} max {m['max']:.4g} final {m['final']:.4g}"
        )
    assert res["pass"], res["fail_reasons"]
    assert {
        m["name"] for m in res["monitors"] if not m["name"].startswith("no_servo_alarm")
    } == {
        "cube_within_1cm_of_target",
        "cube_on_table",
        "cube_was_lifted",
    }  # + one no_servo_alarm per servo (6)
    assert (
        len([m for m in res["monitors"] if m["name"].startswith("no_servo_alarm")]) == 6
    )
    assert (
        all(m["pass"] for m in res["monitors"])
        and not res["sim"]["port_errors"]
        and not res["sim"]["diverged"]
    )
    assert "done:" in (d / "program.log").read_text()
    tick, h = program.replay_hash(
        d / "replay.naplog", next((d / "model").glob("scene_*.xml")).parent
    )
    print(f"  replay tick {tick} hash {h} vs run {res['sim']['state_hash']}")
    assert (
        h == res["sim"]["state_hash"]
        and tick == res["sim"]["tick"]
        and res["replay"]["match"]
    )
