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
    # every real pin of each card is drawn, grouped by connector (pdb out1 = the O1 solder pads, esc power_in = the XT30 plug)
    pins = {c["id"]: [p["name"] for p in c["pins"]] for c in f["cards"]}
    assert {"O1.1", "O1.2", "O2.1", "IN.1"} <= set(pins["pdb"])
    assert {"PWR.1", "PWR.2", "MOT.1", "SIG.1"} <= set(pins["esc1"])
    assert len(pins["pdb"]) == len(set(pins["pdb"]))
    pdb = next(c for c in f["cards"] if c["id"] == "pdb")
    assert {c["id"] for c in pdb["connectors"]} >= {"O1", "O2", "IN"}
    p = pdb["pins"][0]
    assert p["kind"] in ("power", "ground", "signal", "nc") and p["side"] in (
        "left",
        "right",
    )
    assert p["conn"] and p["tip"] and p["label"].startswith(p["name"].partition(".")[2])
    assert all(c["w"] > 0 and c["h"] > 0 for c in f["cards"])
    assert len({(c["x"], c["y"]) for c in f["cards"]}) == 3  # laid out, not stacked


def test_flatten_wire_equals_the_direct_wire_op(pair):
    d, ref, b = pair
    begin(b)
    r = ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})
    assert r["ok"], r
    assert [w["from"] for w in r["flatten"]["wires"]].count(PAIR[0]) == 1
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
    name = next(w["name"] for w in r["flatten"]["wires"] if w["from"] == PAIR[0])
    r = ask(b, "flatten.set_wire", {"name": name, "awg": 14})
    assert r["ok"] and d.doc["wires"][-1]["part"] == "awg-14-silicone-wire"
    assert next(w for w in r["flatten"]["wires"] if w["name"] == name)["awg"] == 14
    left = ask(b, "flatten.unwire", {"name": name})["flatten"]["wires"]
    assert name not in [w["name"] for w in left]
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
        b, "flatten.wire", {"from": "pdb.out3", "to": "esc3.power_in"}
    )  # esc3 is not on the canvas
    assert not r2["ok"] and "not a pin on the canvas" in r2["error"]
    r3 = ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1], "awg": 99})
    assert not r3["ok"] and "AWG" in r3["error"]
    assert Backend.rev_of(d) == before


def test_explicit_lead_bridges_what_autowire_refuses(tmp_path):
    d = Design.open(X500.with_name("turret.toml"))
    ref = [dict(w) for w in d.doc["wires"]]
    for w in ref:
        d.unwire(w["name"])
    b = Backend(lambda: d, tmp_path)
    ask(
        b,
        "flatten.begin",
        {"instances": ["adapter", "pan"], "ports": ["adapter.servo_bus", "pan.port_a"]},
    )
    r = ask(
        b,
        "flatten.wire",
        {"from": "adapter.servo_bus", "to": "pan.port_a", "awg": 22, "length": 0.15},
    )
    assert (
        not r["ok"] and "compatible" in r["error"]
    )  # autowire: unlike connector families
    w = next(x for x in ref if x["name"] == "lead_adapter_pan")
    r = ask(
        b,
        "flatten.wire",
        {
            k: w[k]
            for k in (
                "from",
                "to",
                "part",
                "length",
                "conductors",
                "name",
                "from_pins",
                "to_pins",
            )
        },
    )
    assert r["ok"] and d.doc["wires"] == [w]


def test_a_tap_on_the_pdb_solder_pads_can_be_drawn():
    """assemblies/skydio.toml taps the PDB rail on the TAP solder pads (TAP.1 / TAP.2, not pins of port out1): the TAP connector is joinable, so the editing op accepts them as the loader does."""
    d = Design.open(X500.with_name("skydio.toml"))
    w = dict(next(x for x in d.doc["wires"] if x["name"] == "pdb-to-reg"))
    assert w["from_pins"] == ["TAP.1", "TAP.2"]
    d.unwire("pdb-to-reg")
    keys = ("part", "length", "conductors", "name", "from_pins")
    d.wire(w["from"], w["to"], **{k: w[k] for k in keys if k in w})
    assert d.doc["wires"][-1] == w and not [
        f for f in d.check() if f.code == "PIN_ALREADY_USED"
    ]


def test_explicit_lead_keeps_its_pinned_connectors(tmp_path):
    """A lead drawn on the canvas with an explicit part carries every field of the wire, the pinned connector parts included (they were dropped)."""
    d = Design.open(X500)
    w = dict(next(x for x in d.doc["wires"] if x["name"] == "pdb-to-esc1"))
    assert w["connectors"]
    d.unwire("pdb-to-esc1")
    b = Backend(lambda: d, tmp_path)
    ask(b, "flatten.begin", {"instances": ["pdb", "esc1"], "ports": list(PAIR)})
    keys = (
        "from",
        "to",
        "part",
        "length",
        "conductors",
        "included_in",
        "name",
        "connectors",
        "from_pins",
        "to_pins",
    )
    r = ask(b, "flatten.wire", {k: w[k] for k in keys if k in w})
    assert r["ok"], r
    assert d.doc["wires"][-1] == w


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


ASMS = ["x500v2", "so101", "koch", "rover", "car4", "aos5", "turret", "skydio"]


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
        assert w["strands"] and w["route"] == w["strands"][0]["route"]
        for s in w["strands"]:
            (ia, _, pa), (ib, _, pb) = s["from"].partition("."), s["to"].partition(".")
            ca, cb = cards[ia], cards[ib]
            pina = {p["name"]: p for p in ca["pins"]}[pa]
            pinb = {p["name"]: p for p in cb["pins"]}[pb]
            two_row = any(
                k["side"] == "both" for c_ in (ca, cb) for k in c_["connectors"]
            )
            if ca["band"] == cb["band"] and ca["layer"] != cb["layer"] and not two_row:
                lo, hi = (
                    (pina["side"], pinb["side"])
                    if ca["layer"] < cb["layer"]
                    else (pinb["side"], pina["side"])
                )
                assert (lo, hi) == ("right", "left"), (w["name"], pa, pb)
            r = s["route"]
            assert len(r) >= 2
            for p, q in zip(r, r[1:]):
                assert p[0] == q[0] or p[1] == q[1], (w["name"], "not orthogonal")
                for c in cards.values():
                    assert not _inside(p, q, c), (w["name"], "passes through", c["id"])


def _assemblies_with_wires():
    import tomllib

    root = X500.parent
    return sorted(
        p.stem
        for p in root.glob("*.toml")
        if tomllib.loads(p.read_text(encoding="utf-8")).get("wires")
    )


@pytest.mark.parametrize("asm", _assemblies_with_wires())
def test_no_route_segment_enters_a_card_in_any_assembly(tmp_path, asm):
    """Every conductor of every wire of every shipped assembly: an orthogonal polyline that never enters the interior of a card."""
    f = _all_wired(tmp_path, asm)
    n = 0
    for w in f["wires"]:
        for s in w["strands"]:
            r = s["route"]
            assert len(r) >= 2, (asm, w["name"])
            for p, q in zip(r, r[1:]):
                assert p[0] == q[0] or p[1] == q[1], (asm, w["name"], "not orthogonal")
                for c in f["cards"]:
                    assert not _inside(p, q, c), (
                        asm,
                        w["name"],
                        "passes through",
                        c["id"],
                    )
            n += 1
    assert n >= len(f["wires"])


def test_turret_pi_shows_every_header_pin_and_wires_land_on_pins(tmp_path):
    f = _all_wired(tmp_path, "turret")
    pi = next(c for c in f["cards"] if c["id"] == "pi")
    j8 = [p for p in pi["pins"] if p["conn"] == "J8"]
    assert sorted(int(p["num"]) for p in j8) == list(
        range(1, 41)
    )  # the whole 2 x 20 header, not the three wired pins
    assert {p["side"] for p in j8} == {
        "left",
        "right",
    }  # a two-row header: odd pins one edge, even the other
    assert all(p["side"] == ("left" if int(p["num"]) % 2 else "right") for p in j8)
    assert {p["kind"] for p in j8} >= {"power", "ground", "signal"}
    gpio14 = next(p for p in j8 if p["num"] == "8")
    assert "UART0_TX" in gpio14["tip"] and gpio14["wired"]
    assert any(
        k["id"] == "J8" and k["side"] == "both" and k["nrows"] == 21
        for k in pi["connectors"]
    )
    uart = next(w for w in f["wires"] if w["name"] == "pi_uart_adapter")
    assert [(s["from"], s["to"]) for s in uart["strands"]] == [
        ("pi.J8.8", "adapter.UART.2"),
        ("pi.J8.10", "adapter.UART.1"),
        ("pi.J8.6", "adapter.UART.3"),
    ]
    ribbon = next(w for w in f["wires"] if w["name"] == "pi_camera_ribbon")
    assert len(ribbon["strands"]) == 15 and any(
        p["conn"] == "CAM0" for p in pi["pins"]
    )  # the 22-pin camera port is drawn in full as well
    assert not [
        w for w in f["wires"] if w["flag"]
    ]  # the shipped turret wiring is clean


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
                    len(p["label"]) * 7.6 + (0 if p["conn"] else len(p["type"]) * 6.0)
                    for p in c["pins"]
                    if p["side"] == side
                ),
                default=0,
            )
            assert c["w"] >= need


# ---- a part removed while the canvas is open ---------------------------------------------------------------------------------------
def test_removing_a_part_prunes_its_card_and_wires(pair):
    d, _, b = pair
    begin(b)
    ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})
    assert any(
        w["to"].startswith("esc2.") for w in ask(b, "flatten.state")["flatten"]["wires"]
    )
    rev0 = ask(b, "rev")["result"]["flat_rev"]
    d.edit([{"op": "remove", "id": "esc2"}])  # any edit path: the canvas is not asked
    assert ask(b, "rev")["result"]["flat_rev"] > rev0  # the window refetches the state
    f = ask(b, "flatten.state")["flatten"]
    assert {c["id"] for c in f["cards"]} == {"pdb", "esc1"}
    assert not any("esc2" in (w["from"] + w["to"]) for w in f["wires"])
    assert not any(
        "esc2" in c["id"] + str(p["connects"]) for c in f["cards"] for p in c["pins"]
    )
    for tool, args in (
        ("flatten.layout", {}),
        ("flatten.move_card", {"id": "pdb", "x": 9.0, "y": 9.0}),
        ("flatten.state", {}),
    ):
        assert ask(b, tool, args)["ok"], tool
    d.edit([{"op": "remove", "id": "esc1"}])
    d.edit([{"op": "remove", "id": "pdb"}])
    f = ask(b, "flatten.state")["flatten"]
    assert not f["cards"] and not f["active"]  # nothing left: back to the picker


def test_pin_tooltip_lists_the_far_ends(pair):
    d, _, b = pair
    begin(b)
    f = ask(b, "flatten.wire", {"from": PAIR[0], "to": PAIR[1]})["flatten"]
    pdb = next(c for c in f["cards"] if c["id"] == "pdb")
    hit = [
        p for p in pdb["pins"] if any(c["component"] == "esc1" for c in p["connects"])
    ]
    assert hit and all(p["port"] == "out1" for p in hit)
    far = hit[0]["connects"][0]
    assert far["component"] == "esc1" and far["port"] == "power_in"
    from napl.design.wire2d import pin_links

    links = pin_links(d, "pdb.out1")
    assert links and all(e["component"] == "pdb" and e["port"] == "out1" for e in links)
    assert any(c["component"] == "esc1" for e in links for c in e["connects"])
    assert pin_links(d, "esc1.nope") == [] and len(pin_links(d, "pdb")) >= len(links)


def test_preview_returns_the_cards_and_changes_nothing(pair):
    d, _, b = pair
    before, rev0 = Backend.rev_of(d), ask(b, "rev")["result"]["flat_rev"]
    r = ask(b, "flatten.preview", {"instances": ["pdb", "esc1"], "ports": list(PAIR)})
    assert r["ok"] and {c["id"] for c in r["result"]["cards"]} == {"pdb", "esc1"}
    assert all(c["pins"] and c["w"] > 0 for c in r["result"]["cards"])
    assert not r["flatten"]["active"] and not r["flatten"]["cards"]  # no canvas opened
    assert ask(b, "rev")["result"]["flat_rev"] == rev0 and Backend.rev_of(d) == before
    assert not ask(b, "flatten.preview", {"instances": ["nope"]})["ok"]


def test_state_flags_the_eye_hidden_parts_and_their_wires(tmp_path):
    d = Design.open(X500)
    d.unwire("pdb-to-esc1")
    b = Backend(lambda: d, tmp_path)
    ask(
        b,
        "flatten.begin",
        {"instances": ["pdb", "esc1", "esc2"], "ports": ["pdb.out1", "esc1.power_in"]},
    )
    ask(b, "flatten.wire", {"from": "pdb.out1", "to": "esc1.power_in"})
    f = ask(b, "flatten.state")["flatten"]
    assert f["wires"] and not any(c["hidden"] for c in f["cards"])
    assert not any(w["hidden"] for w in f["wires"])
    ask(b, "ui.visibility", {"instances": ["esc1"], "visible": False})
    f = ask(b, "flatten.state")["flatten"]
    assert {c["id"] for c in f["cards"] if c["hidden"]} == {"esc1"}  # still listed
    ends = lambda w: {w["from"].partition(".")[0], w["to"].partition(".")[0]}
    assert f["wires"] and all(w["hidden"] == ("esc1" in ends(w)) for w in f["wires"])
    ask(b, "ui.visibility", {"visible": True})
    f = ask(b, "flatten.state")["flatten"]
    assert not any(c["hidden"] for c in f["cards"] + f["wires"])


@pytest.mark.parametrize("asm", ASMS)
def test_a_connector_title_fits_its_half_of_the_card(tmp_path, asm):
    f = _all_wired(tmp_path, asm)
    for c in f["cards"]:
        for k in c["connectors"]:
            title = len(f"{k['id']}  {k['name']}") * 7.6 * 0.8 + 20
            assert c["w"] >= title * (1 if k["side"] == "both" else 2) - 1e-6
