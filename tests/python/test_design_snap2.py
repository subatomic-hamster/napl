"""S2: geometry-aware snapping (rotation / port pair chosen by interference, clearance, upright, CoM) and auto-wiring
(`Design.autowire` / `autowire_all`, MCP ops). Placement cases use small synthetic parts written to a tmp library; wiring cases
use the X500 assembly and the real library, edited in memory (nothing under parts/ or assemblies/ changes)."""

import asyncio
import copy
from pathlib import Path

import pytest
from mcp import Client
from napl import design
from napl.design import Assembly, Design, DesignError, Library
from napl.mcp_server import NaplServer
from test_design_mcp import accept, call, ok, root  # noqa: F401  (root is a fixture)

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
X500 = REPO / "assemblies" / "x500v2.toml"

PROV = 'fidelity = "estimated", rationale = "synthetic test part"'


def part_toml(pid, ports, box):
    hx, hy, hz, px, py, pz = box
    out = [
        f'id = "{pid}"',
        'name = "synthetic"',
        'category = "frame"',
        "[params]",
        f'mass = {{ value = 0.05, unit = "kg", {PROV} }}',
        "[geometry]",
        'shape = "box"',
        f"size = [{hx}, {hy}, {hz}]",
        f"pos = [{px}, {py}, {pz}]",
    ]
    for name, typ, pos, axis, sym in ports:
        out += [
            "[[ports]]",
            f'name = "{name}"',
            'kind = "mech"',
            f'type = "{typ}"',
            'gender = "neutral"',
            f"pos = {pos}",
            f"axis = {axis}",
            "ref = [1.0, 0.0, 0.0]",
            f"symmetry = {sym}",
        ]
    return "\n".join(out) + "\n"


@pytest.fixture
def pdir(tmp_path):
    """plate (4-fold seat `top`, one seat `seat_b` for a block), block (a cube on seat_b), arm (a 20 cm bar along its +x),
    blob (a 50 cm slab: covers the block whatever the rotation)."""
    d = tmp_path / "parts"
    d.mkdir()
    up, down = "[0.0, 0.0, 1.0]", "[0.0, 0.0, -1.0]"
    (d / "t-plate.toml").write_text(
        part_toml(
            "t-plate",
            [
                ("top", "pad", "[0.0, 0.0, 0.005]", up, 4),
                ("seat_b", "pad2", "[0.15, 0.0, 0.005]", up, 1),
            ],
            (0.1, 0.1, 0.005, 0, 0, 0),
        )
    )
    (d / "t-block.toml").write_text(
        part_toml(
            "t-block",
            [("base", "pad2", "[0.0, 0.0, 0.0]", down, 1)],
            (0.02, 0.02, 0.02, 0, 0, 0.02),
        )
    )
    (d / "t-arm.toml").write_text(
        part_toml(
            "t-arm",
            [("base", "pad", "[0.0, 0.0, 0.0]", down, 4)],
            (0.1, 0.01, 0.01, 0.1, 0, 0.01),
        )
    )
    (d / "t-blob.toml").write_text(
        part_toml(
            "t-blob",
            [("base", "pad", "[0.0, 0.0, 0.0]", down, 4)],
            (0.25, 0.25, 0.02, 0, 0, 0.02),
        )
    )
    return d


@pytest.fixture
def plate(pdir):
    doc = {
        "assembly": {"name": "t", "description": "synthetic", "base": "free"},
        "instances": [
            {"id": "plate", "part": "t-plate", "root": True},
            {"id": "block", "part": "t-block"},
        ],
        "mates": [{"a": "plate.seat_b", "b": "block.base", "rotation": 0}],
    }
    return Design(Assembly(doc), Library(pdir))


# ---- 1. geometry-aware snap ---------------------------------------------------------------------------------------------------
def test_snap_avoids_the_rotation_that_interferes(plate):
    r = plate.snap("t-arm", to="plate", id="arm")
    ch = r.changes[0]
    snap = ch["snap"]
    assert (
        snap["evaluated"] == 4
        and ch["mate"]["rotation"] != 0
        and snap["k"] == ch["mate"]["rotation"]
    )
    assert snap["interferes"] == []
    k0 = next(a for a in snap["alternatives"] if a["k"] == 0)
    assert k0["interferes"] == ["block"] and any(
        "interferes with block by" in w for w in k0["why"]
    )
    assert (
        k0["score"] < snap["score"] - 500
    )  # an interfering placement ranks below every clean one
    assert "INTERFERENCE" not in {f["code"] for f in r.new}
    assert not any(f.code == "INTERFERENCE" for f in plate.check())


def test_mate_searches_the_rotation_too(plate):
    plate.add("t-arm", id="arm")
    r = plate.mate("plate.top", "arm.base")  # k omitted: searched
    assert r.changes[0]["mate"]["rotation"] != 0
    assert any(
        a["k"] == 0 and a["interferes"] == ["block"]
        for a in r.changes[0]["snap"]["alternatives"]
    )
    # k given: honoured, and a single determined placement is made (the DRC reports it)
    plate.undo()
    r = plate.mate("plate.top", "arm.base", k=0)
    assert r.changes[0]["mate"]["rotation"] == 0
    assert any(f["code"] == "INTERFERENCE" for f in r.new)


def test_every_placement_interferes_is_refused_with_reasons(plate):
    before = copy.deepcopy(plate.doc)
    with pytest.raises(DesignError, match=r"refused, all 4 placements interfere") as e:
        plate.snap("t-blob", to="plate", id="blob")
    assert "interferes with block by" in str(e.value) and "force" in str(e.value)
    assert plate.doc == before  # nothing changed
    r = plate.snap("t-blob", to="plate", id="blob", force=True)  # the caller's call
    assert any(f["code"] == "INTERFERENCE" for f in r.new)


def test_snap_ranks_by_clearance_when_nothing_interferes(plate):
    plate.remove("block")
    r = plate.snap("t-arm", to="plate", id="arm")
    snap = r.changes[0]["snap"]
    assert (
        snap["evaluated"] == 4
        and snap["interferes"] == []
        and r.changes[0]["mate"]["rotation"] == 0
    )
    assert all(a["interferes"] == [] for a in snap["alternatives"])
    assert any("upright" in w for w in snap["why"])


# ---- 2. auto-wiring -----------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def lib():
    return Library(PARTS)


@pytest.fixture(scope="module")
def x500_doc():
    return design.load(X500).doc


@pytest.fixture
def x500(lib, x500_doc):
    return Design(Assembly(copy.deepcopy(x500_doc)), lib)


def wire(d, name):
    return next(w for w in d.doc["wires"] if w["name"] == name)


def ampacity(lib, wire_part):
    return min(q["i_max"]["value"] for q in lib.get(wire_part).ports[:2])


def test_autowire_battery_lead_is_heavier_than_a_signal_lead(x500, lib):
    x500.unwire("bat-to-pm02")
    x500.unwire("gps-to-fc")
    pw = x500.autowire("bat.power_out").changes[0]
    sg = x500.autowire("gps.gps_out").changes[0]
    assert {pw["wire"]["from"], pw["wire"]["to"]} == {"bat.power_out", "pm02.power_in"}
    assert {sg["wire"]["from"], sg["wire"]["to"]} == {"gps.gps_out", "fc.gps1"}
    amp_p, amp_s = ampacity(lib, pw["wire"]["part"]), ampacity(lib, sg["wire"]["part"])
    assert amp_p > amp_s and lib.get(pw["wire"]["part"]).value("mass_per_m") > (
        lib.get(sg["wire"]["part"]).value("mass_per_m")
        or lib.get(sg["wire"]["part"]).value("mass_per_m_copper")
    )
    need = pw["reasons"]["current_A"]["peak"]
    # D5: the pack now declares its `lead` (120 mm 10 AWG, inside its mass), so the pack's own wire is used, not the margin-sized one
    assert (
        pw["wire"]["part"] == "awg-10-silicone-wire"
        and pw["wire"]["included_in"] == "bat"
    )
    assert (
        amp_p < need * design.wiring.WIRE_MARGIN
    )  # reported by DRC (CURRENT_*) rather than silently up-sized
    assert pw["wire"]["conductors"] == 2 and "conductors" not in sg["wire"]
    print(
        "battery lead:",
        pw["wire"]["part"],
        need,
        "A peak;",
        "signal lead:",
        sg["wire"]["part"],
    )
    print(pw["reasons"]["part"])
    print(sg["reasons"]["part"])


def test_autowire_recreates_the_removed_pdb_esc_wire(x500, lib):
    orig = copy.deepcopy(wire(x500, "pdb-to-esc1"))
    x500.unwire("pdb-to-esc1")
    r = x500.autowire("esc1.power_in")
    ch = r.changes[0]
    w = ch["wire"]
    assert (w["from"], w["to"]) == (orig["from"], orig["to"])
    assert (
        w["connectors"] == orig["connectors"]
    )  # {from: amass-xt30apw-m, to: amass-xt30u-f}: siblings' pins reused
    assert w.get("conductors") == orig["conductors"] == 2
    assert ampacity(lib, w["part"]) >= ampacity(
        lib, orig["part"]
    )  # gauge >= the original (awg 16: 22 A)
    assert "FLOATING_PORT" in {
        f["code"] for f in r.resolved
    } or "FLOATING_PORT" not in {f["code"] for f in r.new}
    assert (
        w["length"]["fidelity"] == "derived"
        and "routing factor 1.3" in w["length"]["derivation"]
    )
    assert w["length"]["value"] > 0.03
    assert "same part" in ch["reasons"]["connector_from"]
    assert "same part" in ch["reasons"]["connector_to"]
    print("recreated:", w, "\n", ch["reasons"])


def test_autowire_length_is_distance_times_the_routing_factor(x500):
    x500.unwire("pdb-to-esc1")
    ch = x500.autowire("esc1.power_in").changes[0]
    import re

    d = float(
        re.search(
            r"straight line ([\d.]+) m", ch["wire"]["length"]["derivation"]
        ).group(1)
    )
    assert ch["wire"]["length"]["value"] >= d * design.wiring.ROUTING_FACTOR - 1e-9
    assert (
        ch["wire"]["length"]["value"]
        < d * design.wiring.ROUTING_FACTOR + design.wiring.LENGTH_QUANTUM + 1e-9
    )


def test_autowire_all_wires_the_unique_and_reports_the_ambiguous(x500):
    n = len(x500.doc["wires"])
    for nm in ("pdb-to-esc1", "fc-pwm1", "fc-pwm2"):
        x500.unwire(nm)
    r = x500.autowire_all()
    ch = r.changes[0]
    assert [x["wire"]["to"] for x in ch["wired"]] == [
        "esc1.power_in"
    ]  # the one with a single sensible target
    # a 2 x 2 ambiguity is ONE decision (not four reports seen from both sides): which PWM channel is the user's call
    assert len(ch["ambiguous"]) == 1
    (dec,) = ch["ambiguous"]
    assert dec["ports"] == ["esc1.signal", "esc2.signal"]
    assert {"fc.pwm1", "fc.pwm2"} <= set(
        dec["candidates"]
    )  # plus the 6C's other free MAIN / AUX channels
    assert len(x500.doc["wires"]) == n - 2  # nothing was guessed
    with pytest.raises(DesignError, match="equally good"):
        x500.autowire("esc1.signal")
    r = x500.autowire("esc1.signal", b="fc.pwm1")  # naming the target resolves it
    assert r.changes[0]["wire"]["from"] == "esc1.signal"
    print(ch["ambiguous"])


def test_autowire_errors_are_clean(x500):
    with pytest.raises(DesignError, match="already has a wire"):
        x500.autowire("esc1.power_in")
    with pytest.raises(DesignError, match="power port"):
        x500.autowire("esc1.power_in", kind="signal")
    before = copy.deepcopy(x500.doc)
    x500.unwire("fc-pwm1")
    with pytest.raises(DesignError, match="not an open compatible"):
        x500.autowire("esc1.signal", b="fc.gps1")
    assert x500.doc != before


def test_suggest_and_snap_say_what_comes_next(x500):
    rows = x500.suggest("con1.motor", limit=3)
    motor = next(r for r in rows if r["category"].startswith("motor"))
    text = " ".join(motor["needs"])
    assert "shaft" in text and "phase" in text, motor[
        "needs"
    ]  # a prop for the shaft, an ESC channel for the phase
    x500.remove("mot1")
    r = x500.snap("holybro-2216-kv920", to="con1", id="mot1")
    assert r.changes[0]["needs"] and "shaft" in " ".join(r.changes[0]["needs"])


# ---- 3. MCP -------------------------------------------------------------------------------------------------------------------
def test_mcp_autowire_round_trip(root):  # noqa: F811
    async def agent():
        async with Client(NaplServer(root).server, elicitation_callback=accept) as c:
            await ok(c, "design_open", path="assemblies/x500v2.toml")
            r = await ok(
                c, "design_edit", ops=[{"op": "unwire", "name": "pdb-to-esc2"}]
            )
            assert any(f["code"] == "FLOATING_PORT" for f in r["drc"]["new"])
            r = await ok(
                c, "design_edit", ops=[{"op": "autowire", "a": "esc2.power_in"}]
            )
            ch = r["changes"][0]
            assert ch["op"] == "autowire" and ch["wire"]["to"] == "esc2.power_in"
            assert ch["reasons"]["part"] and ch["reasons"]["connector_from"]
            assert any(f["code"] == "FLOATING_PORT" for f in r["drc"]["resolved"])
            r = await ok(
                c,
                "design_edit",
                ops=[{"op": "unwire", "name": "pdb-to-esc3"}, {"op": "autowire_all"}],
            )
            assert [x["wire"]["to"] for x in r["changes"][1]["wired"]] == [
                "esc3.power_in"
            ]
            assert r["changes"][1]["ambiguous"] == []
            err, text = await call(
                c, "design_edit", ops=[{"op": "autowire", "a": "esc1.power_in"}]
            )
            assert err and "already has a wire" in text
            err, text = await call(
                c,
                "design_edit",
                ops=[{"op": "autowire", "a": "esc1.power_in", "kind": "mech"}],
            )
            assert err  # the schema only allows power | signal
            r = await ok(c, "design_undo")
            assert r["changes"][0]["op"] == "undo"

    asyncio.run(agent())
