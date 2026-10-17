"""SP1-ui tasks 3-6: the sub-assembly workspace in the viewer (breadcrumb, New part / exports, MY PARTS, tree, hierarchical WIRING
block, mass / cost). Backend-only tests run anywhere; the window tests (`gpu`) run under `xvfb-run -a pytest -m gpu` and read the
window's own `ui.report` (see test_viewer_controls.py)."""

import os
import threading
import time

import napl
import pytest
from napl.design import Design
from napl.design.assembly import _v
from napl.design.viewer import Backend
from test_subasm import LEG, make_parts, prm, write
from test_subasm_ops import PORTS, call, raw_call

needs_display = pytest.mark.skipif(
    not os.environ.get("DISPLAY"), reason="needs a display: run under xvfb-run"
)


def priced_chassis(tmp_path, mirrored=("fr", "br")):
    """The 4-leg chassis of test_subasm_ops with a price on every part, saved to dog.toml."""
    parts = make_parts(tmp_path)
    for f in parts.glob("*.toml"):
        t = f.read_text()
        f.write_text(
            t.replace(
                "[params]\n", "[params]\nprice = " + _v(prm(5.0, "USD")) + "\n", 1
            )
        )
    # (no [[devices]]: the window tests load the model into a Sim, which knows only the real device schema)
    write(
        tmp_path / "leg.toml",
        LEG.replace("[[devices]]", "[[x]]").split("[[x]]")[0]
        + "[[targets]]"
        + LEG.split("[[targets]]")[1],
    )
    d = Design.new("dog", parts)
    d.add("plate", id="plate", root=True)
    d.save(tmp_path / "dog.toml")
    b = Backend(lambda: d, tmp_path / "out")
    for sid, port in PORTS:
        call(
            b,
            "sub.insert",
            file="leg.toml",
            id=sid,
            mate={"a": f"plate.{port}", "export": "mount"},
            **({"mirror": "xz"} if sid in mirrored else {}),
        )
    d.save()
    return d, b


def reports(b):
    return [q["args"]["layout"] for q, _ in b.log if q["tool"] == "ui.report"]


def show_until(b, d, script, done, timeout=60, during=None, width=1280, height=720):
    """Run the window (host Sim) until `done(reports)` or the timeout; `during()` runs once on a thread after the first report."""
    sim = napl.Sim.load(b.state(d)["xml"], host=True)  # (close_viewer needs a host Sim)

    def closer():
        t0, ran = time.time(), False
        while time.time() - t0 < timeout:
            rs = reports(b)
            if rs and during and not ran:
                ran = True
                during()
            if rs and (not during or ran) and done(rs):
                break
            time.sleep(0.2)
        time.sleep(0.5)
        sim.close_viewer()

    threading.Thread(target=closer, daemon=True).start()
    sim.show(script=script, width=width, height=height, design=b)
    return reports(b)


# ---- backend-level ---------------------------------------------------------------------------------------------------------------
def test_new_part_crumb_and_dirty_marker_survive_a_refused_close(
    tmp_path,
):  # Review Focus 3
    d, b = priced_chassis(tmp_path)
    r = call(b, "sub.new", name="foot", base="free")
    assert r["result"]["crumb"] == ["dog", "foot"]
    assert [x["title"] for x in r["state"]["docs"]] == ["dog", "dog > foot"]
    call(
        b, "design_edit", ops=[{"op": "add", "part": "plate", "id": "p", "root": True}]
    )
    st = call(b, "state")["state"]
    assert [x["dirty"] for x in st["docs"]] == [False, True]
    out = raw_call(b, "sub.close")  # no file yet: refused, nothing dropped
    assert not out["ok"]
    st = call(b, "state")["state"]
    assert [x["dirty"] for x in st["docs"]] == [False, True]
    call(b, "sub.close", path="foot.toml")  # relative paths go next to the base design
    assert (tmp_path / "foot.toml").exists()
    assert [x["title"] for x in call(b, "state")["state"]["docs"]] == ["dog"]


def test_missing_child_file_is_reported_not_raised(tmp_path):  # Review Focus 5
    d, b = priced_chassis(tmp_path)
    assert call(b, "state")["state"]["sub_errors"] == {}
    (tmp_path / "leg.toml").unlink()
    st = call(b, "state")["state"]
    assert set(st["sub_errors"]) == {"fl", "fr", "bl", "br"}
    assert "leg.toml" in st["sub_errors"]["fl"]
    out = raw_call(
        b, "sub.reload"
    )  # the file is gone: an error reply, the old flat design stays
    assert not out["ok"] and "leg.toml" in out["error"]
    assert len(d.doc["instances"]) == 1 + 4 * 2


def test_my_parts_op_and_the_library_category(tmp_path):
    d, b = priced_chassis(tmp_path)
    rows = call(b, "library.my_parts")["result"]["parts"]
    assert [r["name"] for r in rows] == ["leg"] and rows[0]["path"].endswith("leg.toml")
    r = call(b, "library.search", category="@my")["result"]
    assert [p["name"] for p in r["parts"]] == ["leg"] and r["total"] == 1


def test_bom_has_per_sub_mass_and_cost(tmp_path):
    d, b = priced_chassis(tmp_path)
    bom = call(b, "state")["state"]["report"]["bom"]
    assert set(bom["by_sub"]) == {"fl", "fr", "bl", "br"}
    assert bom["by_sub"]["fl"]["cost"] == pytest.approx(10.0)  # brk + mot, $5 each
    assert bom["total"]["cost"] == pytest.approx(5.0 + 4 * 10.0)


def wired_parent(tmp_path):
    """A PDB feeding four ESC sub-assemblies (esc_unit.toml exports its power input as the connector `power`)."""
    write(
        tmp_path / "esc_unit.toml",
        """
        [assembly]
        name = "esc_unit"
        [exports]
        connectors = { power = "esc.power_in" }
        [[instances]]
        id = "esc"
        part = "holybro-x500v2-esc-blheli-s-20a"
        root = true
        """,
    )
    text = """
        [assembly]
        name = "pwr"
        [[instances]]
        id = "pdb"
        part = "holybro-pdb-60a"
        root = true
        params = { esc_lead_length = { value = 0.2, unit = "m", fidelity = "estimated", rationale = "test lead" } }
    """
    for k in range(1, 5):
        text += f"""
        [[subassemblies]]
        id = "u{k}"
        file = "esc_unit.toml"
        pos = [{0.1 * k}, 0, 0]
        [[wires]]
        name = "pdb-to-u{k}"
        from = "pdb.out{k}"
        to = "u{k}:power"
        from_pins = ["O{k}.1", "O{k}.2"]
        to_pins = ["PWR.1", "PWR.2"]
        part = "awg-16-silicone-wire"
        length = "pdb.esc_lead_length"
        conductors = 2
        """
    write(tmp_path / "pwr.toml", text)
    d = Design.open(tmp_path / "pwr.toml")
    return d, Backend(lambda: d, tmp_path / "out")


def test_parent_wiring_sheet_has_four_blocks_with_exported_connectors_only(tmp_path):
    d, b = wired_parent(tmp_path)
    assert len(d.doc["instances"]) == 5
    r = call(
        b,
        "flatten.begin",
        instances=["pdb", "u1:esc", "u2:esc", "u3:esc", "u4:esc"],
    )
    cards = {c["id"]: c for c in r["flatten"]["cards"]}
    assert set(cards) == {"pdb", "u1", "u2", "u3", "u4"}
    for k in range(1, 5):
        blk = cards[f"u{k}"]
        assert blk["sub"] and blk["part"] == "esc_unit.toml"
        assert [p["name"] for p in blk["pins"]] == ["power"]  # exported connectors only
    wires = r["flatten"]["wires"]
    assert len(wires) == 4
    for w in wires:
        assert {s["to"].split(".")[0] for s in w["strands"]} <= {"u1", "u2", "u3", "u4"}
        assert all(s["to"].endswith(".power") for s in w["strands"])
    # a wire dragged onto a block is refused with the way out
    bad = raw_call(b, "flatten.wire", **{"from": "pdb.out1", "to": "u1.power"})
    assert not bad["ok"] and "sub-assembly" in bad["error"]


# ---- the window ------------------------------------------------------------------------------------------------------------------
@needs_display
@pytest.mark.gpu
def test_window_breadcrumb_and_unsaved_child_asks_before_returning(tmp_path):
    d, b = priced_chassis(tmp_path)
    call(b, "sub.open", id="fl")
    call(  # an edit in the child: it is dirty
        b,
        "design_edit",
        ops=[{"op": "add", "part": "plate", "id": "extra", "pos": [0.5, 0, 0]}],
    )
    rs = show_until(
        b,
        d,
        "30 crumb 0",
        lambda rs: rs[-1]["confirm"] == "close_doc",
    )
    last = rs[-1]
    assert last["crumbs"] == ["dog", "fl (leg.toml) *"]  # the marker stays on the child
    assert last["confirm"] == "close_doc"  # offers to save; nothing was dropped
    assert len(call(b, "state")["state"]["docs"]) == 2
    assert (tmp_path / "leg.toml").read_text().count(
        "extra"
    ) == 0  # (and nothing written behind its back)


@needs_display
@pytest.mark.gpu
def test_window_tree_from_line_panel_cost_and_missing_file(tmp_path):
    d, b = priced_chassis(tmp_path)
    (tmp_path / "leg.toml").rename(
        tmp_path / "leg.keep"
    )  # one run with the child file gone (Review Focus 5) ...
    rs = show_until(
        b,
        d,
        "10 uisel fl:mot",
        lambda rs: any(r["from"] for r in rs) and any(r["panel"] for r in rs),
        timeout=40,
    )
    (tmp_path / "leg.keep").rename(tmp_path / "leg.toml")  # ... then with it present
    node = rs[-1]["tree"][0]
    assert node["id"] == "fl" and "leg.toml" in node["error"] and node["items"] == 2
    assert all(n["error"] for n in rs[-1]["tree"])
    rs = show_until(
        b,
        d,
        "10 uisel fl:mot",
        lambda rs: any(r["from"] for r in rs) and any(r["panel"] for r in rs),
    )
    last = rs[-1]
    assert [(n["id"], n["mirror"], n["items"]) for n in last["tree"]] == [
        ("fl", "", 2),
        ("fr", "xz", 2),
        ("bl", "", 2),
        ("br", "xz", 2),
    ]
    assert not any(n["error"] for n in last["tree"])
    assert any(r["from"] == "from leg.toml" for r in rs)  # the inspector of fl:mot
    bom = d.build()["report"]["bom"]
    assert any(r["cost"] == f"COST  ${bom['total']['cost']:.2f}" for r in rs)
    panel = next(r["panel"] for r in rs if r["panel"])
    assert panel[0] == "fl | %.1f g | $10.00" % (bom["by_sub"]["fl"]["mass"] * 1000)
    assert len(panel) == 4


@needs_display
@pytest.mark.gpu
def test_window_my_parts_insert_mirror_and_two_nodes(tmp_path):
    priced_chassis(
        tmp_path
    )  # only for its parts and leg.toml; the window opens a fresh, empty design
    d2 = Design.new("dog2", tmp_path / "parts")
    d2.add("plate", id="plate", root=True)
    d2.save(tmp_path / "dog2.toml")
    b2 = Backend(lambda: d2, tmp_path / "out2")
    call(
        b2, "library.search", category="@my"
    )  # the window opens on MY PARTS (an event)
    call(b2, "ui.mode", name="library")

    def drag_in():  # what the drag-in does (one sub.insert) and then the Mirror button (sub.mirror)
        call(
            b2,
            "sub.insert",
            file="leg.toml",
            id="leg",
            mate={"a": "plate.mount_l", "export": "mount"},
        )
        call(b2, "sub.mirror", id="leg", plane="xz", to="plate.mount_r")

    rs = show_until(
        b2,
        d2,
        "",
        lambda rs: len(rs[-1]["tree"]) == 2,
        during=drag_in,
    )
    assert any(r["category"] == "@my" and r["hits"] == ["leg"] for r in rs)
    assert [n["id"] for n in rs[-1]["tree"]] == ["leg", "leg_m"]
    assert rs[-1]["tree"][1]["mirror"] == "xz"
