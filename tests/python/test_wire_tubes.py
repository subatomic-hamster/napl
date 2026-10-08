"""Wires as 3D tubes: every assembly wire with both ends on bodies becomes one visual-only spatial tendon per conductor (so the
tube follows its bodies); the renderer draws them (Solver::tubes). They must not change the dynamics."""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design
from napl.design import wiretubes

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
NAMES = ("x500v2", "so101", "rover")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    return {
        n: design.build(
            REPO / "assemblies" / f"{n}.toml", PARTS, tmp_path_factory.mktemp(n)
        )
        for n in NAMES
    }


def _tendons(xml):
    return ET.parse(xml).getroot().findall("./tendon/spatial")


def _expected(asm_name, built_one):
    """(tube count, {tendon name: diameter}) from the assembly's wires: onboard ends only, <= MAX_TUBES conductors each."""
    lib = design.Library(PARTS)
    doc = design.load(REPO / "assemblies" / f"{asm_name}.toml").doc
    off = {i["id"] for i in doc["instances"] if i.get("offboard")}
    dia = {}
    for k, w in enumerate(doc.get("wires", [])):
        if w["from"].partition(".")[0] in off or w["to"].partition(".")[0] in off:
            continue
        p = lib.get(w["part"])
        d = wiretubes.od_m(p.value("awg"), p.value("conductor_diameter"))
        for c in range(min(w.get("conductors", 1), wiretubes.MAX_TUBES)):
            dia[f"wire:{k}:{c}"] = d
    return dia


@pytest.mark.parametrize("name", NAMES)
def test_every_onboard_wire_is_a_tube_with_its_awg_diameter(built, name):
    want = _expected(name, built[name])
    got = {
        t.get("name"): 2 * float(t.get("width")) for t in _tendons(built[name]["xml"])
    }
    assert want and set(got) == set(want)
    for k, d in want.items():
        assert got[k] == pytest.approx(d, rel=1e-4)
    assert wiretubes.od_m(18) == pytest.approx(
        2.0e-3
    )  # table sanity: 18 AWG silicone ~ 2 mm


@pytest.mark.parametrize("name", NAMES)
def test_tendons_are_physically_inert(built, name):
    for t in _tendons(built[name]["xml"]):
        assert t.get("stiffness") in (None, "0") and t.get("damping") in (
            None,
            "0",
        )  # MuJoCo default 0
        assert t.get("range") is None and t.get("limited") is None
    root = ET.parse(built[name]["xml"]).getroot()
    assert root.find("actuator") is None or all(
        a.get("tendon") is None for a in root.find("actuator")
    )


@needs_kernel
@pytest.mark.parametrize("name", NAMES)
def test_solver_reports_the_tubes(built, name):
    s = napl.Sim.load(built[name]["xml"])
    tubes = s.tubes()
    assert len(tubes) == len(_tendons(built[name]["xml"]))
    for radius, length, rgba, pts in tubes:
        assert radius > 0 and length > 0 and len(rgba) == 4
        assert pts.shape == (2, 3) and np.isfinite(pts).all()


@needs_kernel
@pytest.mark.parametrize("name", ("x500v2", "rover"))
def test_state_hash_is_unchanged_by_the_wire_visuals(built, name, tmp_path):
    """The same scenario on the model with its tendons and on a copy with them stripped: identical state hashes."""
    xml = built[name]["xml"]
    text = xml.read_text()
    bare = tmp_path / "bare.xml"
    stripped = re.sub(r"\s*<tendon>.*?</tendon>", "", text, flags=re.DOTALL)
    assert stripped != text
    bare.write_text(stripped)
    (tmp_path / "meshes").symlink_to(
        REPO / "meshes"
    )  # mesh paths are relative to the xml
    if (xml.parent / "visuals").exists():
        (tmp_path / "visuals").symlink_to(xml.parent / "visuals")
    hashes = []
    for path in (xml, bare):
        s = napl.Sim.load(path)
        for _ in range(5):
            s.step(100)
            hashes.append(s.state_hash())
    assert hashes[:5] == hashes[5:] and len(set(hashes[:5])) > 1


@needs_kernel
def test_tubes_follow_their_bodies(built):
    """Move a joint of the SO-101 arm: the cable anchors on the moved links move with it."""
    s = napl.Sim.load(built["so101"]["xml"])
    before = [p.copy() for *_, p in s.tubes()]
    prims = s.call({"cmd_type": "Query", "cmd": {"path": ""}}).get("prims", [])
    js = [p for p in prims if p.endswith("/shoulder_lift")]
    assert js, f"no arm joint among {prims[:10]}"
    s.set(js[0], "qpos", [0.6])
    after = [p for *_, p in s.tubes()]
    assert any(not np.allclose(a, b) for a, b in zip(after, before))
