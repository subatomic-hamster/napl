"""The 2D flatten wiring mode as design_api ops: begin -> wire -> end must leave the same design as the direct `autowire` op;
refusals carry the reason; every action is a named op a scripted agent can call (tools/record_build.py replays them)."""

import json
from pathlib import Path

import pytest
from napl.design import Design
from napl.design.viewer import Backend

X500 = Path(__file__).resolve().parents[2] / "assemblies" / "x500v2.toml"
PAIR = ("pdb.out1", "esc1.power_in")


def ask(b, tool, args=None):
    return json.loads(b(json.dumps({"tool": tool, "args": args or {}})))


@pytest.fixture
def pair(tmp_path):
    d, ref = Design.open(X500), Design.open(X500)
    for x in (d, ref):
        x.unwire("pdb-to-esc1")
    return d, ref, Backend(lambda: d, tmp_path)


def begin(b):
    r = ask(
        b,
        "flatten.begin",
        {"instances": ["pdb", "esc1", "esc2"], "ports": list(PAIR) + ["esc2.power_in"]},
    )
    assert r["ok"], r
    return r["flatten"]


def test_begin_shows_cards_with_the_chosen_pins_only(pair):
    _, _, b = pair
    f = begin(b)
    assert f["active"] and {c["id"] for c in f["cards"]} == {"pdb", "esc1", "esc2"}
    pins = {c["id"]: [p["name"] for p in c["pins"]] for c in f["cards"]}
    assert pins == {"pdb": ["out1"], "esc1": ["power_in"], "esc2": ["power_in"]}
    p = f["cards"][0]["pins"][0]
    assert p["kind"] == "power" and p["type"] and p["side"] in ("left", "right")
    assert all(c["w"] > 0 and c["h"] > 0 for c in f["cards"])
    assert len({(c["x"], c["y"]) for c in f["cards"]}) == 3  # laid out, not stacked


def test_flatten_wire_equals_the_direct_wire_op(pair):
    d, ref, b = pair
    begin(b)
    r = ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})
    assert r["ok"], r
    assert r["flatten"]["wires"][0]["from"] == PAIR[0]
    assert ask(b, "flatten.end")["flatten"]["active"] is False
    ref.autowire(PAIR[0], PAIR[1])
    assert d.doc["wires"] == ref.doc["wires"]
    assert Backend.rev_of(d) == Backend.rev_of(ref)
    assert r["state"]["rev"] == Backend.rev_of(
        d
    )  # the viewer reloads this build: the wire's tubes appear


def test_awg_and_length_are_applied(pair):
    d, _, b = pair
    begin(b)
    r = ask(
        b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1], "awg": 12, "length": 0.25}
    )
    assert r["ok"], r
    w = d.doc["wires"][-1]
    assert w["part"] == "awg-12-silicone-wire" and w["length"]["value"] == 0.25
    name = r["flatten"]["wires"][0]["name"]
    r = ask(b, "flatten.set_wire", {"name": name, "awg": 14})
    assert r["ok"] and d.doc["wires"][-1]["part"] == "awg-14-silicone-wire"
    assert r["flatten"]["wires"][0]["awg"] == 14
    assert ask(b, "flatten.unwire", {"name": name})["flatten"]["wires"] == []
    assert not any(w.get("name") == name for w in d.doc.get("wires", []))


def test_refusals_carry_the_reason_and_change_nothing(pair):
    d, _, b = pair
    begin(b)
    before = Backend.rev_of(d)
    r = ask(
        b, "flatten.wire", {"from": "esc1.power_in", "to": "esc2.power_in"}
    )  # sink -> sink
    assert not r["ok"] and r["error"]
    r2 = ask(
        b, "flatten.wire", {"from": "pdb.out2", "to": "esc1.power_in"}
    )  # pin not on the canvas
    assert not r2["ok"] and "not a pin on the canvas" in r2["error"]
    r3 = ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1], "awg": 99})
    assert not r3["ok"] and "AWG" in r3["error"]
    assert Backend.rev_of(d) == before


def test_move_card_layout_state_and_rev(pair):
    _, _, b = pair
    begin(b)
    rev0 = ask(b, "rev")["result"]["flat_rev"]
    r = ask(b, "flatten.move_card", {"id": "pdb", "x": 321.0, "y": 123.0})
    card = next(c for c in r["flatten"]["cards"] if c["id"] == "pdb")
    assert (card["x"], card["y"]) == (321.0, 123.0)
    assert (
        ask(b, "rev")["result"]["flat_rev"] > rev0
    )  # the window polls this and re-reads flatten.state
    ask(b, "flatten.layout")
    card = next(
        c for c in ask(b, "flatten.state")["flatten"]["cards"] if c["id"] == "pdb"
    )
    assert (card["x"], card["y"]) != (321.0, 123.0)
    assert not ask(b, "flatten.nope")["ok"]
    ask(b, "flatten.end")
    assert not ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})[
        "ok"
    ]  # not active any more


def test_wire_direction_pushes_the_target_card_right(pair):
    _, _, b = pair
    begin(b)
    ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})
    f = ask(b, "flatten.layout")["flatten"]
    x = {c["id"]: c["x"] for c in f["cards"]}
    assert x["esc1"] > x["pdb"]
