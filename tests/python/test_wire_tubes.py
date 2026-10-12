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
    """{wire index: conductor diameter} of the assembly's wires with both ends onboard (one tube per pin pair, <= MAX_TUBES)."""
    lib = design.Library(PARTS)
    doc = design.load(REPO / "assemblies" / f"{asm_name}.toml").doc
    off = {i["id"] for i in doc["instances"] if i.get("offboard")}
    dia = {}
    for k, w in enumerate(doc.get("wires", [])):
        if w["from"].partition(".")[0] in off or w["to"].partition(".")[0] in off:
            continue
        p = lib.get(w["part"])
        dia[k] = wiretubes.od_m(p.value("awg"), p.value("conductor_diameter"))
    return dia


@pytest.mark.parametrize("name", NAMES)
def test_every_onboard_wire_is_a_tube_with_its_awg_diameter(built, name):
    want = _expected(name, built[name])
    got, ends = {}, {}
    for t in _tendons(built[name]["xml"]):
        nm, ia, ib = t.get("name").split(
            "|"
        )  # "wire:<i>:<c>|<end a>|<end b>": the viewer hides a cable by these ids
        _, k, c = nm.split(":")
        got.setdefault(int(k), {})[int(c)] = 2 * float(t.get("width"))
        ends[int(k)] = (ia, ib)
    assert want and set(got) == set(want)
    wires = design.load(REPO / "assemblies" / f"{name}.toml").doc["wires"]
    for k in want:
        assert ends[k] == tuple(wires[k][e].partition(".")[0] for e in ("from", "to"))
    for k, d in want.items():
        assert sorted(got[k]) == list(range(len(got[k])))  # conductors 0..n-1
        assert 1 <= len(got[k]) <= wiretubes.MAX_TUBES
        assert all(v == pytest.approx(d, rel=1e-4) for v in got[k].values())
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
        assert pts.shape[0] >= 2 and pts.shape[1] == 3 and np.isfinite(pts).all()


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
