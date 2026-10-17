"""Dog leg sub-assembly (assemblies/dog/leg.toml): builds alone, DRC clean apart from being unpowered, three Qdds + a foot contact
with every physical field from the part files, the knee on the linkage ratio."""

import json
import pathlib
import xml.etree.ElementTree as ET

import pytest
from napl import design
from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, LEG = ROOT / "parts", ROOT / "assemblies" / "dog" / "leg.toml"
LIB = Library(PARTS)
STANDALONE_OK = {
    "NET_NO_SOURCE"
}  # a leg alone has no battery; anything else is a real error


def devices_json(xml_text):
    return json.loads(
        ET.fromstring(xml_text).find("custom/text[@name='napl.devices']").get("data")
    )


def test_leg_drc_has_no_errors_but_the_missing_source():
    fs = check(A.load(LEG), LIB)
    assert [
        f.code for f in fs if f.severity == "error" and f.code not in STANDALONE_OK
    ] == []


def test_leg_exports_and_mass(tmp_path):
    asm = A.load(LEG)
    ex = asm.raw["exports"]
    assert ex["ports"] == {"hip_mount": "abad_mount.chassis"}
    assert ex["connectors"] == {"power": "hub.power_in", "can": "hub.can_in"}
    assert ex["params"] == {"knee_drive_ratio": "knee_motor.drive_ratio"}
    _, rep = design.compute(asm, PARTS, tmp_path)
    assert (
        1.4 < rep["mass"]["total"] < 2.4
    )  # 3 x 0.396 kg actuators + printed parts + rod + foot + hub + cables


def test_leg_qdds_come_from_the_part_and_the_knee_from_the_linkage(tmp_path):
    xml, rep = design.compute(A.load(LEG), PARTS, tmp_path)
    dev = devices_json(xml)
    q = {d["name"]: d for d in dev["qdds"]}
    assert sorted(q) == ["abad", "hip", "knee"]
    act = LIB.get("steadywin-gim8108-8")
    for n in ("abad", "hip"):
        assert q[n]["gear_ratio"] == act.value("gear_ratio")
        assert q[n]["i_limit"] == act.value("i_limit_phase")
        assert q[n]["motor"]["kv"] == pytest.approx(act.value("kv"))
    r = LIB.get("dog-shank").value("lug_radius") / LIB.get("dog-knee-crank-l").value(
        "crank_radius"
    )
    assert q["knee"]["gear_ratio"] == pytest.approx(act.value("gear_ratio") * r)
    assert [j["joint"] for j in dev["qdds"]] == ["abad_j", "hip_j", "knee_j"]
    (fc,) = dev["foot_contacts"]
    assert fc["name"] == "foot" and fc["geom"] == "foot_ball"
