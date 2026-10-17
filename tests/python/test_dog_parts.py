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


HANDED = ["hip-housing", "thigh-bracket", "knee-drive-housing", "side-plate"]


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


@pytest.mark.parametrize("name", ["shank", "foot-core", "head-mount", "battery-tray"])
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
