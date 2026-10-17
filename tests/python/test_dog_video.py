import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
sys.path.insert(0, str(ROOT / "tools"))

import napl
import record_dog as RD

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def test_stages_cover_every_instance_once_and_name_the_five_sub_assemblies():
    import tomllib

    doc = tomllib.loads((ROOT / "assemblies" / "dog" / "dog.toml").read_text())
    sub_ids = {s["id"] for s in doc["subassemblies"]}
    assert sub_ids == {"fl", "fr", "rl", "rr", "head", "power", "compute"}
    seen = [g for _, _, groups in RD.STAGES for g in groups]
    assert len(seen) == len(set(seen)) and set(seen) >= sub_ids and "chassis" in seen
    assert [t.split()[0] for t, _, _ in RD.STAGES][:2] == ["1/6", "2/6"]
    assert set(RD.CHASSIS) == {i["id"] for i in doc["instances"]}


@needs_kernel
def test_oplog_steps_are_timed_and_ordered(tmp_path):
    out = tmp_path / "oplog.json"
    RD.main(["oplog", str(out)])
    steps = json.loads(out.read_text())["steps"]
    segs = [s["seg"] for s in steps]
    assert segs.index("BUILD") < segs.index("WIRING") < segs.index("PROGRAM")
    assert (
        sum(r["wait"] for s in steps for r in s["reqs"]) <= RD.PART_A_MAX_S
    )  # part (a) stays inside its time budget
    assert any(
        r["tool"] == "sub.open" for s in steps for r in s["reqs"]
    )  # the leg is opened as its own document
    assert any(
        r["tool"] == "ui.wire_pick" for s in steps for r in s["reqs"]
    )  # pin-level wiring is shown
    assert all(
        r["tool"] != "ui.frame" for s in steps for r in s["reqs"]
    )  # ui.frame selects what it frames (record_skydio finding)
    assert steps == json.loads(json.dumps(steps))  # plain JSON, deterministic
    RD.main(["oplog", str(tmp_path / "again.json")])
    assert (tmp_path / "again.json").read_text() == out.read_text()


@needs_kernel
def test_explode_keeps_every_body_and_moves_only_groups(tmp_path):  # Review Focus 5
    import dogsim

    xml, _ = dogsim.build(tmp_path)
    text = xml.read_text()
    groups = {
        "fl:": (0.3, 0.3, 0.0),
        "rl:": (-0.2, 0.1, 0.05),
        "head:": (0.4, 0.0, 0.2),
    }
    out = RD.explode_xml(text, groups, 1.0)
    bodies = lambda t: sorted(b.get("name") for b in ET.fromstring(t).iter("body"))
    assert bodies(out) == bodies(text)
    assert RD.explode_xml(text, groups, 0.0) == text
    moved, base = RD.world_pos(out), RD.world_pos(text)
    assert set(moved) == set(base)
    for k in base:
        want = next(
            (np.array(v) for g, v in groups.items() if k.startswith(g)), np.zeros(3)
        )
        assert abs(moved[k] - base[k] - want).max() < 1e-9, (
            k
        )  # rl: hangs from the 180-deg-turned bulkhead: the WORLD shift is still the vector
    vec = RD.leg_vectors(text)
    assert (
        vec["fl:"][1] > 0 > vec["fr:"][1] and vec["rl:"][1] > 0 > vec["rr:"][1]
    )  # outward along each leg's own side
