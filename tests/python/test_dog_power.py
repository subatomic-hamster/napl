"""Dog power module (assemblies/dog/power.toml): 2 x (8S1P pack + 60 A BMS) in parallel, a 100 A contactor switched by the e-stop
through its 12 V coil, compute / lidar DC-DCs fed before the contactor, the PDB after it."""

import pathlib

from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, POWER = ROOT / "parts", ROOT / "assemblies" / "dog" / "power.toml"
LIB = Library(PARTS)
PEAK_BUS_A = 87.0  # docs/dog/actuator-study.md finding 2
CONTACTOR = "altran-alev100-b-dc-contactor-12v"


def test_power_module_is_drc_clean():
    fs = check(A.load(POWER), LIB)
    assert [(f.code, f.message) for f in fs if f.severity == "error"] == []


def test_estop_switches_only_the_coil_and_ratings_cover_the_peak():  # Review Focus 4
    asm = A.load(POWER)
    ends = [
        (w["from"], w["to"])
        for w in asm.doc["wires"]
        if "estop." in w["from"] + w["to"]
    ]
    assert sorted(
        w["name"] for w in asm.doc["wires"] if "estop." in w["from"] + w["to"]
    ) == ["estop_in", "estop_out"]
    assert {e for pair in ends for e in pair if not e.startswith("estop.")} == {
        "dcdc_aux.vout",
        "contactor.coil",
    }
    diag = {}
    check(asm, LIB, diag=diag)
    coil = LIB.get(CONTACTOR).value("coil_current")
    for w in ("estop_in", "estop_out"):
        # a loop edge of the coil loop carries 0 in the DRC's spanning-tree flow (drc.py Net.flows), so it may be absent
        peak, _ = diag["currents"].get(w, (0.0, 0.0))
        assert peak <= 1.05 * coil, (w, peak)
    # the DRC does not model `load` devices, so also check the coil current against the ratings it flows through
    cp = lambda pid, n: next(p for p in LIB.get(pid).ports if p["name"] == n)
    assert cp("generic-estop-22mm-nc", "contact")["i_max"]["value"] >= coil
    assert cp(CONTACTOR, "coil")["i_max"]["value"] >= coil
    assert (
        cp("pololu-d24v22f12", "vout")["i_max"]["value"] >= coil + 6.5 / 9.0
    )  # coil + lidar at the 9 V minimum
    # every series element of the motor feed is rated for the study's simultaneous peak
    port = lambda pid, n: next(p for p in LIB.get(pid).ports if p["name"] == n)
    assert (
        2 * LIB.get("auline-21700-s70-8s1p-4500mah").value("i_max_continuous")
        >= PEAK_BUS_A
    )
    assert (
        2 * port("daly-k-series-smart-bms-8s-60a", "pack")["i_max"]["value"]
        >= PEAK_BUS_A
    )
    assert LIB.get(CONTACTOR).value("i_rated_continuous") >= PEAK_BUS_A
    assert (
        port("dog-power-distribution-board", "bat_in")["i_max"]["value"] >= PEAK_BUS_A
    )
    # the compute / lidar DC-DCs hang on the contactor's LINE side: the e-stop never drops the recon payload
    vin = {w["to"] for w in asm.doc["wires"] if w["from"] == "contactor.line"}
    assert {"dcdc_jetson.vin", "dcdc_aux.vin"} <= vin


def test_two_batteries_with_the_liion_curve():
    asm = A.load(POWER)
    bats = {d["name"]: d for d in asm.doc["devices"] if d["type"] == "battery"}
    assert sorted(bats) == ["pack_a", "pack_b"]
    assert {b["ocv"] for b in bats.values()} == {"liion-cell-ocv-chen2020-lgm50"}
