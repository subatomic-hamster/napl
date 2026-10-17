"""Dog parts library (SP3a): QDD constants reproduce the datasheet; every dog part has a dated USD price and a mirror table."""

import math
import pathlib
import tomllib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
QDD = [
    "actuators/steadywin-gim8108-8",
    "actuators/steadywin-gim6010-8",
    "actuators/cubemars-ak70-9-v3-kv60",
]


def load(rel):
    return tomllib.loads((ROOT / "parts" / f"{rel}.toml").read_text())


def v(p, k):
    return p["params"][k]["value"]


@pytest.mark.parametrize("rel", QDD)
def test_qdd_constants_reproduce_datasheet(rel):
    """Qdd builder inputs (Kt motor-side, N, eta, Ke) must give the published OUTPUT-side peak torque and no-load speed within 10%."""
    p = load(rel)
    kt, n, eta, ke = (
        v(p, "kt_motor"),
        v(p, "gear_ratio"),
        v(p, "efficiency"),
        v(p, "ke_si") if "ke_si" in p["params"] else v(p, "kt_motor"),
    )
    assert (
        0.6 <= eta <= 0.95
    )  # effective figure, bounded so it cannot absorb an arbitrary mismatch
    tau = kt * n * eta * v(p, "current_peak")
    assert tau == pytest.approx(v(p, "torque_peak"), rel=0.10)
    w0 = v(p, "v_rated") / (ke * n)  # rad/s at the output, no load (R*i drop neglected)
    assert w0 == pytest.approx(v(p, "speed_no_load"), rel=0.10)
    assert kt == pytest.approx(ke, rel=0.01)  # SI identity Kt = Ke
    assert v(p, "kv_output") * v(p, "v_rated") * 2 * math.pi / 60 == pytest.approx(
        v(p, "speed_no_load"), rel=0.10
    )
    assert p["params"]["price"]["unit"] == "USD" and v(p, "price") > 0
    assert p["mirror"]["symmetry"] in ("xy", "yz", "xz")


PRICED = [  # purchased parts of the dog: a dated USD price (param `price`, or `price_per_m` for tube)
    "electronics/livox-mid-360",
    "electronics/livox-mid-360-splitter-cable",
    "electronics/intel-realsense-d435i",
    "electronics/sparkfun-gps-rtk-sma-16481",
    "electronics/makerbase-mks-canable-v2-pro",
    "electronics/pololu-d24v22f12",
    "electronics/pololu-d24v22f5",
    "electronics/daly-k-series-smart-bms-8s-60a",
    "electronics/dog-power-distribution-board",
    "propulsion/auline-21700-s70-8s1p-4500mah",
    "connectors/cubemars-ak-v3-power-can-cable",
    "structure/mcmaster-natural-rubber-ball-1-3-8in",
    "structure/adafruit-1252-gt2-pulley-20t-8mm",
    "structure/adafruit-1254-gt2-pulley-36t-8mm",
    "structure/cf-tube-16x14",
    "structure/cf-tube-10x8",
    "electronics/altran-alev100-b-dc-contactor-12v",
    "electronics/dog-leg-hub",
    "structure/generic-phs5-rod-end-m5",
    "structure/dog-knee-rod-200",
    "structure/dog-knee-rod-220",
    "structure/dog-knee-rod-250",
    "structure/dog-deck-top-cf3",
    "structure/dog-deck-bottom-cf3",
    "electronics/pololu-minimu-9-v6",
    "electronics/ublox-ann-mb-00",
    "electronics/generic-estop-22mm-nc",
    *QDD,
]


@pytest.mark.parametrize("rel", PRICED)
def test_dog_part_has_usd_price(rel):
    p = load(rel)["params"]
    e = p.get("price") or p["price_per_m"]
    assert e["unit"] == "USD" and e["value"] > 0
    assert e["fidelity"] in (
        "datasheet",
        "literature",
        "measured",
        "derived",
        "estimated",
    )


HANDED = [
    "hip-housing",
    "thigh-bracket",
    "side-plate",
    "knee-crank",  # handed pair (the plate side flips with the leg side), a deviation from the plan's single dog-knee-crank
]


@pytest.mark.parametrize("base", HANDED)
def test_printed_pairs_are_mirrors(base):
    left, right = load(f"structure/dog-{base}-l"), load(f"structure/dog-{base}-r")
    assert (
        left["mirror"]["opposite"] == f"dog-{base}-r"
        and right["mirror"]["opposite"] == f"dog-{base}-l"
    )
    lg, rg = left["geometry"], right["geometry"]
    assert lg["bbox_min"][1] == pytest.approx(-rg["bbox_max"][1]) and lg["bbox_max"][
        1
    ] == pytest.approx(-rg["bbox_min"][1])
    assert left["params"]["mass"]["value"] == pytest.approx(
        right["params"]["mass"]["value"]
    )
    for g in (lg, rg):
        assert (ROOT / g["mesh"]).stat().st_size > 1000


@pytest.mark.parametrize(
    "name",
    [
        "shank",
        "foot-core",
        "head-mount",
        "battery-tray",
        "abad-mount",
        "bulkhead",
        "compute-tray",
    ],
)
def test_printed_singles_have_symmetry_and_mesh(name):
    p = load(f"structure/dog-{name}")
    assert p["mirror"]["symmetry"] in ("xy", "yz", "xz")
    assert (ROOT / p["geometry"]["mesh"]).stat().st_size > 1000


from napl.design import assembly as A
from napl.design import auto_devices as AD
from napl.design import subasm

STEADYWIN = QDD[:2]  # steadywin-gim8108-8, steadywin-gim6010-8


def test_subasm_prefixes_the_ground_devices():
    assert {"qdd", "foot_contact"} <= A.DEVICE_TYPES
    assert A.DEVICE_TYPES <= set(
        subasm.REF_FIELDS
    )  # a device type without reference fields cannot sit in a leg.toml


@pytest.mark.parametrize("rel", STEADYWIN)
def test_steadywin_part_feeds_every_qdd_field(rel):
    p = load(rel)
    for field, names in {**AD.QDD_MOTOR, **AD.QDD}.items():
        if field == "motor.i0":  # optional in the kernel (docs/models.md "Qdd")
            continue
        assert any(n in p["params"] for n in names), (rel, field, names)
    kt = 60 / (2 * math.pi * v(p, "kv"))  # motor-side Kt from KV: the Qdd's own law
    stall = kt * v(p, "gear_ratio") * v(p, "efficiency") * v(p, "i_limit_phase")
    assert stall == pytest.approx(v(p, "torque_peak"), rel=0.05)
    assert v(p, "mit_tau_ff_max") >= v(p, "torque_peak")
    assert 25.0 < v(p, "temp_derate_start") < v(p, "temp_fault")
    assert v(p, "can_latency") <= 14 / v(p, "can_cmd_hz")  # kernel bind-time rule


from napl.design import Library
from napl.design.snap import incompatible

PEAK_BUS_A = 87.0  # worst simultaneous stance peak, docs/dog/actuator-study.md finding 2 (GIM8108-8, r = 1.0)
CONTACTOR = "electronics/altran-alev100-b-dc-contactor-12v"  # replaces the plan's ZJW100A-class part (no datasheet, >1 A coil)


def test_knee_rod_does_not_buckle_at_peak_knee_torque():  # Review Focus 1
    act = load("actuators/steadywin-gim8108-8")
    r1 = v(load("structure/dog-knee-crank-l"), "crank_radius")
    f_rod = (
        v(act, "torque_peak") / r1
    )  # the crank turns the whole stall torque into rod force (~1.1 kN)
    assert (
        v(load("structure/generic-phs5-rod-end-m5"), "load_static_radial")
        >= 2.0 * f_rod
    )
    for L in ("200", "220", "250"):
        rod = load(f"structure/dog-knee-rod-{L}")
        od, idd = v(rod, "outer_diameter"), v(rod, "inner_diameter")
        i_area = math.pi / 64 * (od**4 - idd**4)
        p_cr = (
            math.pi**2
            * v(rod, "youngs_modulus_axial")
            * i_area
            / v(rod, "pin_length") ** 2
        )  # Euler, pinned-pinned
        assert p_cr >= 2.0 * f_rod, (L, p_cr, f_rod)
        assert v(rod, "buckling_load") == pytest.approx(p_cr, rel=1e-3)
        # the rod is the tube plus two rod-end centre-to-shoulder lengths
        assert v(rod, "pin_length") == pytest.approx(
            v(rod, "tube_length")
            + 2 * v(load("structure/generic-phs5-rod-end-m5"), "centre_to_shoulder"),
            abs=1e-9,
        )


def test_contactor_and_parallel_packs_cover_the_study_peak():
    c = load(CONTACTOR)
    assert v(c, "i_rated_continuous") >= PEAK_BUS_A and v(c, "v_rated_contact") >= 33.6
    assert v(c, "coil_voltage") == pytest.approx(12.0) and v(c, "coil_current") <= 1.0
    lib = Library(ROOT / "parts")
    assert (
        2 * v(load("propulsion/auline-21700-s70-8s1p-4500mah"), "i_max_continuous")
        >= PEAK_BUS_A
    )
    bms_port = next(
        p
        for p in lib.get("daly-k-series-smart-bms-8s-60a").ports
        if p["name"] == "pack"
    )
    assert 2 * bms_port["i_max"]["value"] >= PEAK_BUS_A
    pdb_in = next(
        p
        for p in lib.get("dog-power-distribution-board").ports
        if p["name"] == "bat_in"
    )
    assert pdb_in["i_max"]["value"] >= PEAK_BUS_A


DECK_SEATS = [  # (custom part, port, held part, port)
    ("dog-deck-bottom-cf3", "bms_a", "daly-k-series-smart-bms-8s-60a", "base"),
    ("dog-deck-bottom-cf3", "pdb", "dog-power-distribution-board", "base"),
    (
        "dog-deck-bottom-cf3",
        "contactor",
        "altran-alev100-b-dc-contactor-12v",
        "base",
    ),
    ("dog-deck-bottom-cf3", "dcdc_jetson", "pololu-d24v22f12", "base"),
    ("dog-deck-bottom-cf3", "dcdc_aux", "pololu-d24v22f12", "base"),
    ("dog-deck-bottom-cf3", "estop", "generic-estop-22mm-nc", "panel"),
    ("dog-compute-tray", "can_fl", "makerbase-mks-canable-v2-pro", "base"),
    ("dog-abad-mount", "hub", "dog-leg-hub", "base"),
    ("dog-battery-tray", "pocket", "auline-21700-s70-8s1p-4500mah", "base"),
    ("dog-knee-rod-220", "end_a", "generic-phs5-rod-end-m5", "thread"),
    ("dog-knee-rod-220", "end_b", "generic-phs5-rod-end-m5", "thread"),
]


@pytest.mark.parametrize("pp,pport,bp,bport", DECK_SEATS)
def test_custom_seat_mates_its_part(pp, pport, bp, bport):
    lib = Library(ROOT / "parts")
    port = lambda pid, n: next(p for p in lib.get(pid).ports if p["name"] == n)
    assert incompatible(port(pp, pport), port(bp, bport)) is None


def test_deck_trays_are_battery_tray_seats():
    lib = Library(ROOT / "parts")
    port = lambda pid, n: next(p for p in lib.get(pid).ports if p["name"] == n)
    for n in ("tray_a", "tray_b"):
        assert (
            incompatible(
                port("dog-deck-bottom-cf3", n), port("dog-battery-tray", "origin")
            )
            is None
        )
    assert (
        incompatible(
            port("dog-deck-top-cf3", "compute"), port("dog-compute-tray", "origin")
        )
        is None
    )
    assert (
        incompatible(port("dog-deck-top-cf3", "head"), port("dog-head-mount", "origin"))
        is None
    )
    assert (
        incompatible(
            port("dog-deck-top-cf3", "side_l"), port("dog-side-plate-l", "top")
        )
        is None
    )


def test_liion_ocv_table_is_monotone_and_spans_the_cell_window():
    t = load("common/liion-cell-ocv-chen2020-lgm50")["tables"]["ocv_vs_soc"]
    assert t["columns"] == ["soc", "ocv_V"]
    soc = [r[0] for r in t["rows"]]
    ocv = [r[1] for r in t["rows"]]
    assert soc[0] == 0.0 and soc[-1] == 1.0
    assert all(b > a for a, b in zip(ocv, ocv[1:]))
    assert 2.4 <= ocv[0] <= 2.6 and 4.1 <= ocv[-1] <= 4.2


def test_leg_hub_feeds_three_drives_with_a_can_input():
    h = load("electronics/dog-leg-hub")
    ports = {p["name"]: p for p in h["ports"]}
    assert {"base", "power_in", "can_in"} <= set(ports)
    for j in "ahk":
        assert ports[f"out_{j}"]["type"] == "xt30pw-2+2"
        assert (
            len(ports[f"out_{j}"]["pins"]) == 2 and len(ports[f"can_{j}"]["pins"]) == 2
        )
    assert v(h, "r_path") < 0.01 and v(h, "can_termination") == 120.0
    assert (
        ports["out_a"]["type"]
        == next(
            p
            for p in load("actuators/steadywin-gim8108-8")["ports"]
            if p["name"] == "power"
        )["type"]
    )
