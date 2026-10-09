"""Pin-level connectors (parts/README.md "Connectors and pins"): schema validation, pin resolution of ports and wires, the pin DRC rules."""

import copy
import tomllib
from pathlib import Path

import pytest
from napl.design import Design, assembly, drc, drc_pins, library, pins

ROOT = Path(__file__).resolve().parents[2]
LIB = library.Library(ROOT / "parts")
TURRET = ROOT / "assemblies" / "turret.toml"


def good():
    return {
        "id": "demo",
        "sources": [{"id": "S1", "url": "http://x", "title": "t", "quote": "q"}],
        "ports": [
            {"name": "p", "kind": "signal", "type": "x", "pins": ["J1.1", "J1.2"]}
        ],
        "connectors": [
            {
                "id": "J1",
                "type": "jst",
                "pos": [0, 0, 0],
                "axis": [0, 0, 1],
                "ref": [1, 0, 0],
                "cols": 2,
                "pitch": 0.001,
                "src": "S1",
                "fidelity": "datasheet",
                "pins": [
                    {"number": 1, "name": "GND", "kind": "ground"},
                    {
                        "number": 2,
                        "name": "TX",
                        "kind": "signal",
                        "dir": "out",
                        "v": 3.3,
                    },
                ],
            }
        ],
    }


# ---- schema validation ----------------------------------------------------------------------------------------------------------
def test_a_well_formed_connector_validates():
    assert pins.validate(good()) == []


@pytest.mark.parametrize(
    "mutate, needle",
    [
        (lambda d: d["connectors"][0].pop("src"), "src"),
        (lambda d: d["connectors"][0].update(fidelity="estimated"), "rationale"),
        (lambda d: d["connectors"][0].update(fidelity="derived"), "derivation"),
        (lambda d: d["connectors"][0]["pins"][0].update(kind="rail"), "kind"),
        (lambda d: d["connectors"][0]["pins"][1].update(number=1), "duplicate"),
        (lambda d: d["connectors"][0]["pins"][1].update(dir="sideways"), "dir"),
        (lambda d: d["connectors"][0].update(axis=[0, 0, 2]), "unit"),
        (lambda d: d["connectors"][0].update(pitch=-1), "pitch"),
        (lambda d: d["connectors"][0]["pins"][1].update(v="3V3"), "'v'"),
        (lambda d: d["ports"][0].update(pins=["J1.1", "J9.1"]), "J9.1"),
        (lambda d: d["connectors"][0]["pins"][0].update(colour="red"), "unknown key"),
    ],
)
def test_validation_names_the_problem(mutate, needle):
    d = good()
    mutate(d)
    msgs = pins.validate(d)
    assert msgs and any(needle in m for m in msgs), msgs


def test_every_shipped_connector_part_validates_and_cites_its_values():
    """All parts with [[connectors]]: no validation problem, and every pin value has a source, a quote and a fidelity."""
    n = 0
    for f in sorted((ROOT / "parts").glob("*/*.toml")):
        doc = tomllib.loads(f.read_text(encoding="utf-8"))
        if not doc.get("connectors"):
            continue
        n += 1
        assert pins.validate(doc) == [], f.name
        srcs = {s["id"]: s for s in doc.get("sources", [])}
        for c in doc["connectors"]:
            assert c.get("fidelity") in (
                "datasheet",
                "literature",
                "measured",
                "derived",
                "estimated",
            ), (f.name, c["id"])
            assert (
                c.get("src") in srcs
                and srcs[c["src"]].get("quote")
                or c["fidelity"] in ("derived", "estimated")
            ), (f.name, c["id"])
    assert n >= 15
    assert LIB.errors == []


# ---- resolution -------------------------------------------------------------------------------------------------------------------
def test_pi_header_geometry_and_pins():
    pi = LIB.get("raspberry-pi-5")
    j8 = pi.connectors["J8"]
    assert (j8["rows"], j8["cols"]) == (2, 20) and len(j8["pins"]) == 40
    p8, p10, p1, p2 = (j8["pins"][k] for k in ("8", "10", "1", "2"))
    assert p8["name"] == "GPIO14" and "a4:UART0_TX" in p8["alt"]
    assert p1["kind"] == "power" and p1["v"] == 3.3 and p2["v"] == 5.0
    # odd pins are one row, even pins the other; pitch 2.54 mm between neighbours
    d_col = abs(p8["pos"] - p10["pos"])
    assert d_col.max() == pytest.approx(0.00254, abs=1e-6)
    assert abs(p1["pos"] - p2["pos"]).max() == pytest.approx(0.00254, abs=1e-6)


def test_ports_resolve_to_pins_and_unclaimed_pins_get_a_synthetic_port():
    pi = LIB.get("raspberry-pi-5")
    assert pins.port_pins(pi, "gpio_uart") == ["J8.8", "J8.10", "J8.6"]
    assert pins.port_of_pin(pi, "J8.8") == "gpio_uart"
    assert (
        pins.port_of_pin(pi, "J8.2") == "j8"
    )  # the 5 V pin: no authored port claims it
    syn = pi.port("cam0")
    assert syn["synthetic"] and syn["type"] == "csi" and len(syn["pins"]) == 22
    assert "cam0" not in [p["name"] for p in pi.ports]  # not part of the authored ports


def test_wire_shorthand_folds_into_port_and_pins():
    asm = assembly.load(TURRET)
    doc = copy.deepcopy(asm.doc)
    w = {
        "name": "x",
        "from": "pi.J8.8",
        "to": "adapter.UART.2",
        "part": "awg-26-silicone-wire",
        "length": 0.2,
    }
    doc["wires"] = [w]
    assembly.Ctx(assembly.Assembly(doc), LIB)
    assert (w["from"], w["from_pins"]) == ("pi.gpio_uart", ["J8.8"])
    assert w["to_pins"] == ["UART.2"] and w["to"].startswith("adapter.")


def test_port_level_wire_resolves_to_concrete_pins():
    asm = assembly.load(TURRET)
    w = {
        "name": "x",
        "from": "psu.power_out",
        "to": "adapter.power_in",
        "part": "awg-22-silicone-wire",
        "length": 0.2,
    }
    asm.doc["wires"] = [w]
    ctx = assembly.Ctx(asm, LIB)
    assert pins.wire_pairs(ctx, w) == [("OUT.1", "DC.1"), ("OUT.2", "DC.2")]


def test_turret_wires_land_on_real_pins():
    ctx = assembly.Ctx(assembly.load(TURRET), LIB)
    got = {w["name"]: pins.wire_pairs(ctx, w) for w in ctx.doc["wires"]}
    assert got["pi_uart_adapter"] == [
        ("J8.8", "UART.2"),
        ("J8.10", "UART.1"),
        ("J8.6", "UART.3"),
    ]
    assert len(got["pi_camera_ribbon"]) == 15
    assert all(a and b for pairs in got.values() for a, b in pairs)


def test_pinwire_op_adds_one_conductor_between_two_pins():
    d = Design.open(TURRET)
    d.unwire("pi_uart_adapter")
    length = {"value": 0.2, "unit": "m", "fidelity": "estimated", "rationale": "test"}
    r = d.edit(  # the adapter is not mounted in the turret: the length is given
        [
            {
                "op": "pinwire",
                "a": "pi.J8.8",
                "b": "adapter.UART.2",
                "included_in": "pi",
                "length": length,
            }
        ]
    )
    w = d.doc["wires"][-1]
    assert (w["from"], w["from_pins"], w["to_pins"], w["conductors"]) == (
        "pi.gpio_uart",
        ["J8.8"],
        ["UART.2"],
        1,
    )
    assert w["name"] == "pi.J8.8--adapter.UART.2" and w["part"].startswith("awg-")
    assert r.changes
    with pytest.raises(Exception, match="pin"):
        d.edit(
            [
                {
                    "op": "pinwire",
                    "a": "pi.J8.99",
                    "b": "adapter.UART.2",
                    "length": length,
                }
            ]
        )


# ---- pin DRC rules ------------------------------------------------------------------------------------------------------------------
def findings(path, *wires, drop=()):
    asm = assembly.load(path)
    asm.doc["wires"] = [w for w in asm.doc["wires"] if w["name"] not in drop] + list(
        wires
    )
    F = []
    drc_pins.pin_rules(assembly.Ctx(asm, LIB), F, drc._add)
    return F


def wire(name, a, b, fa, fb, **kw):
    return {
        "name": name,
        "from": a,
        "to": b,
        "from_pins": fa,
        "to_pins": fb,
        "part": "awg-26-silicone-wire",
        "length": 0.2,
        **kw,
    }


def codes(F):
    return sorted({f.code for f in F})


def test_shipped_designs_with_pins_are_clean_except_the_known_so101_supply():
    for name in ("turret", "x500v2", "skydio", "koch"):
        assert findings(ROOT / "assemblies" / f"{name}.toml") == [], name
    (f,) = findings(ROOT / "assemblies" / "so101.toml")
    assert (
        f.code == "PIN_VOLTAGE_MISMATCH" and f.severity == "warning"
    )  # 5 V supply into the 9-12.6 V adapter


def test_pin_already_used():
    F = findings(
        TURRET, wire("dup", "pi.gpio_uart", "adapter.bus", ["J8.8"], ["UART.2"])
    )
    assert "PIN_ALREADY_USED" in codes(F)


def test_kind_mismatch_power_to_signal():
    F = findings(
        TURRET, wire("bad", "pi.j8", "adapter.bus", ["J8.2"], ["UART.1"])
    )  # 5 V pin onto TXD
    assert [f.severity for f in F if f.code == "PIN_KIND_MISMATCH"] == ["error"]


def test_voltage_mismatch_power():
    F = findings(
        TURRET,
        wire(
            "v",
            "psu5.power_out",
            "adapter.power_in",
            ["OUT.1", "OUT.2"],
            ["DC.1", "DC.2"],
        ),
    )
    assert "PIN_VOLTAGE_MISMATCH" in codes(F)


def test_two_outputs_on_a_net():
    F = findings(
        ROOT / "assemblies" / "x500v2.toml",
        wire("oo", "sik.telem", "gps.gps_out", ["TELEM.3"], ["GPS.3"]),
    )  # radio TX meets GPS TX
    assert "PIN_TWO_OUTPUTS" in codes(F)


def test_ground_missing():
    F = findings(
        TURRET,
        wire(
            "nog",
            "pi.gpio_uart",
            "adapter.bus",
            ["J8.8", "J8.10"],
            ["UART.2", "UART.1"],
        ),
        drop=("pi_uart_adapter",),
    )
    assert codes(F) == ["PIN_GND_MISSING"]


def test_alt_function_conflict_and_missing_function():
    split = wire(
        "split",
        "pi.j8",
        "adapter.bus",
        ["J8.8", "J8.21", "J8.6"],
        ["UART.2", "UART.1", "UART.3"],
    )  # TX on UART0, RX on UART3
    F = findings(TURRET, split, drop=("pi_uart_adapter",))
    assert "PIN_ALT_CONFLICT" in codes(F)
    nofn = wire(
        "nofn",
        "pi.j8",
        "adapter.bus",
        ["J8.3", "J8.10", "J8.6"],
        ["UART.2", "UART.1", "UART.3"],
    )  # GPIO2 has no TX
    F = findings(TURRET, nofn, drop=("pi_uart_adapter",))
    assert "PIN_FUNCTION_MISSING" in codes(F)


def test_findings_reach_the_main_drc():
    d = Design.open(TURRET)
    d.edit(
        [
            {
                "op": "wire",
                "name": "dup",
                "from": "pi.J8.8",
                "to": "adapter.UART.2",
                "part": "awg-26-silicone-wire",
                "length": {
                    "value": 0.2,
                    "unit": "m",
                    "fidelity": "estimated",
                    "rationale": "test",
                },
            }
        ]
    )
    assert "PIN_ALREADY_USED" in {f.code for f in d.findings()}
