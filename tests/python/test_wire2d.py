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


def test_wiring_relays_out_until_a_card_is_moved_by_hand(pair):
    _, _, b = pair
    begin(b)
    f = ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})["flatten"]
    x = {c["id"]: c["x"] for c in f["cards"]}
    assert x["esc1"] > x["pdb"]  # no layout call: the wire alone moved the cards
    ask(b, "flatten.move_card", {"id": "pdb", "x": 5.0, "y": 5.0})
    f = ask(b, "flatten.unwire", {"name": f["wires"][0]["name"]})["flatten"]
    assert next(c for c in f["cards"] if c["id"] == "pdb")["x"] == 5.0


# ---- layout and routing -----------------------------------------------------------------------------------------------------------
def _all_wired(tmp_path, asm="x500v2"):
    d = Design.open(X500.with_name(f"{asm}.toml"))
    b = Backend(lambda: d, tmp_path)
    ids = sorted(
        {w[e].partition(".")[0] for w in d.doc["wires"] for e in ("from", "to")}
    )
    ports = [w[e] for w in d.doc["wires"] for e in ("from", "to")]
    r = ask(b, "flatten.begin", {"instances": ids, "ports": sorted(set(ports))})
    assert r["ok"], r
    return r["flatten"]


def _inside(p, q, c):
    """Does the axis-parallel segment p-q enter the interior of card c (shrunk by 0.1: routes are rounded to 0.01)?"""
    (x0, y0), (x1, y1) = p, q
    lo_x, hi_x, lo_y, hi_y = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
    l, t, r, b = (
        c["x"] + 0.1,
        c["y"] + 0.1,
        c["x"] + c["w"] - 0.1,
        c["y"] + c["h"] - 0.1,
    )
    return (
        lo_x < r
        and hi_x > l
        and lo_y < b
        and hi_y > t
        and not (x0 == x1 and not l < x0 < r)
        and not (y0 == y1 and not t < y0 < b)
    )


ASMS = ["x500v2", "so101", "koch", "rover", "car4", "aos5", "turret"]


@pytest.mark.parametrize("asm", ASMS)
def test_layout_cards_do_not_overlap_and_flow_left_to_right(tmp_path, asm):
    f = _all_wired(tmp_path, asm)
    cards = f["cards"]
    assert len(cards) >= 3
    for i, a in enumerate(cards):
        for b in cards[i + 1 :]:
            assert not (
                a["x"] < b["x"] + b["w"]
                and b["x"] < a["x"] + a["w"]
                and a["y"] < b["y"] + b["h"]
                and b["y"] < a["y"] + a["h"]
            ), (a["id"], b["id"])
    if asm == "x500v2":
        x = {c["id"]: c["x"] for c in cards}
        assert (
            x["bat"] < x["pdb"] < x["esc1"] < x["mot1"]
        )  # sources -> distribution -> ESCs -> loads
        assert x["esc1"] == x["esc2"] and x["mot1"] == x["mot2"]  # one layer each


@pytest.mark.parametrize("asm", ASMS)
def test_pins_sit_on_the_side_facing_their_peer_and_wires_avoid_cards(tmp_path, asm):
    f = _all_wired(tmp_path, asm)
    cards = {c["id"]: c for c in f["cards"]}
    assert f["wires"]
    for w in f["wires"]:
        (ia, _, pa), (ib, _, pb) = w["from"].partition("."), w["to"].partition(".")
        ca, cb = cards[ia], cards[ib]
        sa = {p["name"]: p["side"] for p in ca["pins"]}[pa]
        sb = {p["name"]: p["side"] for p in cb["pins"]}[pb]
        if ca["band"] == cb["band"] and ca["layer"] != cb["layer"]:
            lo, hi = (sa, sb) if ca["layer"] < cb["layer"] else (sb, sa)
            assert (lo, hi) == ("right", "left"), (w["name"], sa, sb)
        r = w["route"]
        assert len(r) >= 2
        for p, q in zip(r, r[1:]):
            assert p[0] == q[0] or p[1] == q[1], (w["name"], "not orthogonal")
            for c in cards.values():
                assert not _inside(p, q, c), (w["name"], "passes through", c["id"])


@pytest.mark.parametrize("asm", ASMS)
def test_labels_and_card_widths_leave_no_overlap(tmp_path, asm):
    f = _all_wired(tmp_path, asm)
    boxes = []
    for w in f["wires"]:
        lab = w["label"]
        if lab is not None:
            boxes.append(
                (
                    lab["x"] - lab["w"] / 2,
                    lab["y"] - lab["h"] / 2,
                    lab["x"] + lab["w"] / 2,
                    lab["y"] + lab["h"] / 2,
                )
            )
    assert len(boxes) >= len(f["wires"]) * 0.8  # nearly every wire is labelled
    for i, a in enumerate(boxes):
        for b in boxes[i + 1 :]:
            assert not (a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3])
        for c in f["cards"]:
            assert not (
                a[0] < c["x"] + c["w"]
                and c["x"] < a[2]
                and a[1] < c["y"] + c["h"]
                and c["y"] < a[3]
            )
    for c in f[
        "cards"
    ]:  # wide enough for the widest left pin text plus the widest right pin text
        for side in ("left", "right"):
            need = max(
                (
                    len(p["name"]) * 7.6 + len(p["type"]) * 6.0
                    for p in c["pins"]
                    if p["side"] == side
                ),
                default=0,
            )
            assert c["w"] >= need
