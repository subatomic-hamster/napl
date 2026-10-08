"""P4-UI: the scriptable build UI. Library search ranking, colour persisted + applied at build, free-placed poses (gizmo ops)
through save / load, drag-and-drop == add + snap (one undo step), guides, align / distribute, UI events."""

import copy
import json
import re
from pathlib import Path

import numpy as np
import pytest
from napl.design import Design
from napl.design import assembly as A
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[2]
X500 = REPO / "assemblies" / "x500v2.toml"


def ask(b, tool, args=None, **kw):
    r = json.loads(b(json.dumps({"tool": tool, "args": args or {}, **kw})))
    return r


@pytest.fixture()
def be(tmp_path):
    d = Design.open(X500)
    return Backend(lambda: d, tmp_path), d


def body_rgba(xml, name):
    """rgba of what is drawn for an instance: its cosmetic visual geom ("<id>:v0") when the part has one (the plain geom is then
    hidden: alpha 0), else the plain geom."""
    txt = Path(xml).read_text()
    m = re.search(rf'<geom name="{name}:v0"[^>]*rgba="([^"]+)"', txt) or re.search(
        rf'<geom name="{name}"[^>]*rgba="([^"]+)"', txt
    )
    return [float(x) for x in m.group(1).split()]


def test_search_ranking_and_filters(be):
    b, _ = be
    r = ask(b, "library.search", {"text": "2216 kv920"})["result"]
    assert r["parts"][0]["id"] == "holybro-2216-kv920"
    # id prefix beats a substring of the name; all tokens must hit; a typo within a word still finds it
    assert ask(b, "library.search", {"text": "holybro motor"})["result"]["parts"][0][
        "category"
    ].startswith("motor")
    assert ask(b, "library.search", {"text": "holbyro 2216"})["result"]["parts"][0][
        "id"
    ].startswith("holybro-2216")
    assert ask(b, "library.search", {"text": "zzzzqq"})["result"]["total"] == 0
    # param filters: kv in [900, 1000] -> every hit has it, and the 920 kV motors are among them
    f = ask(
        b,
        "library.search",
        {
            "text": "",
            "category": "motor.bldc",
            "filters": {"kv": [900, 1000]},
            "limit": 500,
        },
    )["result"]
    ids = {p["id"] for p in f["parts"]}
    assert "holybro-2216-kv920" in ids and f["total"] < 64
    lib = be[1].lib
    assert all(900 <= lib.get(i).value("kv") <= 1000 for i in ids)
    v = ask(
        b,
        "library.search",
        {"category": "battery", "filters": {"voltage": [10, 12.5]}, "limit": 500},
    )["result"]
    assert v["total"] > 0 and all(
        10
        <= (
            lib.get(p["id"]).value("v_nominal")
            or lib.get(p["id"]).value("voltage_rated")
            or lib.get(p["id"]).value("v_full")
            or lib.get(p["id"]).value("v_operating_max")
        )
        <= 12.5
        for p in v["parts"]
    )
    cats = ask(b, "library.categories")["result"]["categories"]
    n_motor = sum(
        re.search(r'^category = "motor[."]', p.read_text(), re.MULTILINE) is not None
        for p in (REPO / "parts").rglob("*.toml")
    )
    assert n_motor >= 142 and {"path": "motor", "count": n_motor, "depth": 0} in cats
    info = ask(b, "library.info", {"part": "holybro-2216-kv920"})["result"]
    assert (
        info["ports"]
        and any(p["name"] == "kv" and p["fidelity"] for p in info["params"])
        and info["boxes"]
    )


def test_color_persists_and_builds(be, tmp_path):
    b, d = be
    r = ask(b, "set_color", {"instance": "gps", "rgba": [0.9, 0.1, 0.2, 0.5]})
    assert r["ok"], r
    assert body_rgba(r["state"]["xml"], "gps") == [0.9, 0.1, 0.2, 0.5]
    assert d.doc["instances"][[i["id"] for i in d.doc["instances"]].index("gps")][
        "color"
    ] == [0.9, 0.1, 0.2, 0.5]
    saved = d.save(tmp_path / "c.toml")
    d2 = Design.open(saved)
    assert A.dumps(d2.asm) == A.dumps(d.asm)  # round trip
    assert body_rgba(d2.build(tmp_path / "o")["xml"], "gps") == [0.9, 0.1, 0.2, 0.5]
    ask(b, "design_undo")
    assert "color" not in next(i for i in d.doc["instances"] if i["id"] == "gps")
    assert not ask(b, "set_color", {"instance": "gps", "rgba": [2, 0, 0]})[
        "ok"
    ]  # out of range


def test_gizmo_move_roundtrip_through_save_load(be, tmp_path):
    b, d = be
    st = b.state(d)
    p0 = np.array(st["geo"]["gps"]["pos"])
    # a mated part is not moved silently
    r = ask(b, "gizmo.move", {"instance": "gps", "delta": [0.05, 0, 0]})
    assert not r["ok"] and "unmate" in r["error"]
    n_mates = len(d.doc["mates"])
    r = ask(
        b,
        "gizmo.move",
        {"instance": "gps", "delta": [0.05, 0.01, 0.02], "unmate": True},
    )
    assert r["ok"], r
    assert len(d.doc["mates"]) == n_mates - 1 and not any(
        f.code == "FLOATING_INSTANCE" for f in d.check()
    )
    p1 = np.array(r["state"]["geo"]["gps"]["pos"])
    assert np.allclose(p1 - p0, [0.05, 0.01, 0.02], atol=1e-5)
    # second move: already free, no unmate needed, and equals the sum
    r = ask(b, "gizmo.move", {"instance": "gps", "delta": [-0.05, 0, 0]})
    assert np.allclose(
        np.array(r["state"]["geo"]["gps"]["pos"]) - p0, [0, 0.01, 0.02], atol=1e-5
    )
    # rotate 90 deg about world z: the origin stays, the frame turns
    R0 = np.array(r["state"]["geo"]["gps"]["R"]).reshape(3, 3)
    r = ask(b, "gizmo.rotate", {"instance": "gps", "axis": "z", "angle_deg": 90})
    R1 = np.array(r["state"]["geo"]["gps"]["R"]).reshape(3, 3)
    Rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert np.allclose(R1, Rz @ R0, atol=1e-5)
    # save -> load -> same document, same pose
    saved = d.save(tmp_path / "m.toml")
    assert "pos = " in saved.read_text() and "rpy = " in saved.read_text()
    d2 = Design.open(saved)
    b2 = Backend(lambda: d2, tmp_path / "b2")
    g2 = b2.state(d2)["geo"]["gps"]
    assert np.allclose(g2["pos"], r["state"]["geo"]["gps"]["pos"], atol=1e-5)
    assert np.allclose(g2["R"], r["state"]["geo"]["gps"]["R"], atol=1e-5)
    # undo steps back through rotate, move, move+unmate
    for _ in range(3):
        ask(b, "design_undo")
    assert len(d.doc["mates"]) == n_mates


def test_drop_equals_add_plus_snap_and_is_one_undo_step(be):
    b, d = be
    ask(b, "design_edit", {"ops": [{"op": "remove", "id": "gps"}]})
    d_ref = Design(A.Assembly(copy.deepcopy(d.doc)), d.lib)
    depth = len(d._undo)
    r = ask(
        b, "library.drop", {"part": "holybro-m8n-gps", "to": "mast.tray", "id": "gps2"}
    )
    assert r["ok"], r
    d_ref.add("holybro-m8n-gps", id="gps2", to="mast.tray")
    assert A.dumps(d.asm) == A.dumps(d_ref.asm)
    assert len(d._undo) == depth + 1
    ask(b, "design_undo")
    assert all(i["id"] != "gps2" for i in d.doc["instances"])
    # the ghost list offers the same port among its targets, with a pose
    t = ask(b, "library.targets", {"part": "holybro-m8n-gps"})["result"]
    assert any(x["to"] == "mast.tray" for x in t["targets"]) and t["boxes"]
    assert (
        ask(b, "library.preview", {"part": "holybro-m8n-gps", "to": "mast.tray"})[
            "result"
        ]["target"]["to"]
        == "mast.tray"
    )


def test_free_drop_point_and_floating_is_clean(be):
    b, d = be
    r = ask(
        b,
        "library.drop",
        {"part": "holybro-m8n-gps", "point": [0.3, 0.1, 0.05], "id": "gpsfree"},
    )
    assert r["ok"], r
    g = r["state"]["geo"]["gpsfree"]
    assert np.allclose(g["pos"], [0.3, 0.1, 0.05], atol=1e-5)
    assert not any(f.code == "FLOATING_INSTANCE" for f in d.check())
    assert "gpsfree" in r["state"]["free"]


def test_snap_guides_and_grid(be):
    b, d = be
    geo = b.state(d)["geo"]
    other = next(
        i for i in geo if i not in ("gps",) and geo[i]["hi"][0] - geo[i]["lo"][0] > 0.01
    )
    # a move that ends 1 mm short of aligning the max x edges: the guide pulls it in
    want = geo[other]["hi"][0] - geo["gps"]["hi"][0]
    r = ask(
        b,
        "gizmo.move",
        {
            "instance": "gps",
            "delta": [want - 0.001, 0, 0],
            "unmate": True,
            "snap": {"edges": True},
        },
    )
    assert r["ok"] and r["result"]["guides"], r
    assert any(g["axis"] == 0 for g in r["result"]["guides"])
    moved = r["state"]["geo"]["gps"]
    assert any(
        abs(moved[k][0] - geo[o][k2][0]) < 1e-5
        for o in geo
        if o != "gps"
        for k, k2 in (("lo", "lo"), ("hi", "hi"))
    )


def test_align_distribute_duplicate_mirror(be):
    b, d = be
    ids = []
    for k, x in enumerate((0.0, 0.07, 0.2)):
        r = ask(
            b,
            "library.drop",
            {
                "part": "holybro-m8n-gps",
                "point": [1.0 + x, 0.05 * k, 0.3],
                "id": f"t{k}",
            },
        )
        assert r["ok"]
        ids.append(f"t{k}")
    r = ask(b, "align", {"instances": ids, "axis": "y", "to": "min"})
    ys = [r["state"]["geo"][i]["lo"][1] for i in ids]
    assert np.allclose(ys, ys[0], atol=1e-6)
    r = ask(b, "distribute", {"instances": ids, "axis": "x"})
    g = r["state"]["geo"]
    order = sorted(ids, key=lambda i: g[i]["pos"][0])
    gaps = [g[order[k + 1]]["lo"][0] - g[order[k]]["hi"][0] for k in range(2)]
    assert abs(gaps[0] - gaps[1]) < 1e-5
    r = ask(b, "duplicate", {"instance": "t0", "delta": [0, 0, 0.1]})
    new = r["result"]["changes"][0]["created"][0]
    assert np.allclose(
        np.array(r["state"]["geo"][new]["pos"]) - r["state"]["geo"]["t0"]["pos"],
        [0, 0, 0.1],
        atol=1e-5,
    )
    r = ask(b, "mirror", {"instances": ["t2"], "axis": "y", "at": 0.0})
    m = r["state"]["geo"]["t2-m"]["pos"]
    assert np.isclose(m[1], -r["state"]["geo"]["t2"]["pos"][1], atol=1e-5)


def test_ui_events_and_scripted_state(be):
    b, d = be
    assert (
        ask(b, "ui_poll")["result"]["events"] == []
    )  # first poll: a window is now attached
    ask(b, "ui.mode", {"name": "library"})
    ask(b, "library.search", {"text": "m8n"})
    ask(b, "select", {"instances": ["gps"]})
    ask(b, "ui.camera", {"preset": "top"})
    assert not ask(b, "ui.mode", {"name": "nope"})["ok"]
    ev = ask(b, "ui_poll")["result"]["events"]
    assert [e["cmd"] for e in ev] == ["mode", "search", "select", "camera"]
    assert ev[1]["text"] == "m8n"
    # animate=true queues the drop for the window instead of committing
    n = len(d.doc["instances"])
    ask(b, "design_edit", {"ops": [{"op": "remove", "id": "gps"}]})
    r = ask(
        b,
        "library.drag",
        {
            "part": "holybro-m8n-gps",
            "to": {"instance": "mast", "port": "tray"},
            "animate": True,
        },
    )
    assert r["result"]["queued"] and len(d.doc["instances"]) == n - 1
    ev = ask(b, "ui_poll")["result"]["events"]
    assert (
        ev[-1]["cmd"] == "drag"
        and ev[-1]["then"] == "drop"
        and ev[-1]["to"] == "mast.tray"
    )
    assert ask(b, "ui.state")["result"]["mode"] == "library"


def test_group_gizmo_silent_preview_and_program_text(be):
    b, d = be
    ask(b, "ui_poll")
    geo = b.state(d)["geo"]
    ids = ["gps", "mast"]
    p0 = {i: np.array(geo[i]["pos"]) for i in ids}
    # a group move is ONE request = ONE undo step; silent=true (the window made the call) queues no animation events
    depth = len(d._undo)
    r = ask(
        b,
        "gizmo.move",
        {"instances": ids, "delta": [0, 0.03, 0], "unmate": True, "silent": True},
    )
    assert r["ok"], r
    assert len(d._undo) == depth + 1
    for i in ids:
        assert np.allclose(
            np.array(r["state"]["geo"][i]["pos"]) - p0[i], [0, 0.03, 0], atol=1e-5
        )
    assert ask(b, "ui_poll")["result"]["events"] == []
    # a group rotation turns about the group's centre: the centre of the two origins stays, the pair rotates 90 deg
    c0 = (
        np.array(r["state"]["geo"]["gps"]["pos"])
        + np.array(r["state"]["geo"]["mast"]["pos"])
    ) / 2
    r = ask(
        b,
        "gizmo.rotate",
        {"instances": ids, "axis": [0, 0, 1], "angle_deg": 90, "unmate": True},
    )
    assert r["ok"], r
    c1 = (
        np.array(r["state"]["geo"]["gps"]["pos"])
        + np.array(r["state"]["geo"]["mast"]["pos"])
    ) / 2
    assert np.allclose(c0, c1, atol=1e-5)
    # the preview flies a ghost to the port (an event) without touching the design
    n = len(d.doc["instances"])
    ask(b, "ui_poll")
    r = ask(b, "library.preview", {"part": "holybro-m8n-gps", "to": "mast.tray"})
    assert r["ok"] and len(d.doc["instances"]) == n
    if r["result"]["target"]:
        ev = ask(b, "ui_poll")["result"]["events"]
        assert ev[-1]["cmd"] == "drag" and ev[-1]["then"] == "none"
    # the PROGRAM tab: the assembly as TOML
    t = ask(b, "design_text")["result"]["text"]
    assert t.startswith("[assembly]") and 'part = "holybro-m8n-gps"' in t


def test_view_hover_ops_and_undo_history(be):
    b, d = be
    ask(b, "ui_poll")  # a window attaches
    r = ask(b, "ui.view", {"tooltips": "off", "emphasise_wires": False})
    assert r["ok"] and r["result"] == {"tooltips": "off", "emphasise_wires": False}
    assert not ask(b, "ui.view", {"tooltips": "sometimes"})["ok"]
    assert ask(b, "ui.state")["result"]["tooltips"] == "off"
    assert ask(b, "ui.hover", {"instance": "pdb", "seconds": 3})["ok"]
    assert not ask(b, "ui.hover", {"instance": "nope"})["ok"]
    ev = ask(b, "ui_poll")["result"]["events"]
    assert [e["cmd"] for e in ev] == ["view", "hover"] and ev[1]["instance"] == "pdb"
    # the undo history the HISTORY tab lists: one note per step, kept in step with undo / redo
    assert ask(b, "state")["state"]["history"] == {"undo": [], "redo": []}
    ask(b, "design_edit", {"ops": [{"op": "remove", "id": "gps"}]})
    ask(b, "design_edit", {"ops": [{"op": "unwire", "name": "pdb-to-esc1"}]})
    h = ask(b, "state")["state"]["history"]
    assert h["undo"] == ["remove gps", "unwire pdb-to-esc1"] and h["redo"] == []
    ask(b, "design_undo")
    h = ask(b, "state")["state"]["history"]
    assert h == {"undo": ["remove gps"], "redo": ["unwire pdb-to-esc1"]}
    ask(b, "design_undo", {"redo": True})
    assert ask(b, "state")["state"]["history"]["undo"][-1] == "unwire pdb-to-esc1"


def test_program_file_and_animated_drop_extras(tmp_path):
    d = Design.new("t", REPO / "parts")
    b = Backend(lambda: d, tmp_path)
    ask(b, "ui_poll")
    st = ask(
        b, "state"
    )  # an empty design: the window asks for its state before the first part
    assert (
        st["ok"]
        and st["state"]["xml"] is None
        and "no root" in st["state"]["build_error"]
    )
    r = ask(b, "ui.program", {"file": "examples/cuas/swarm.py", "scroll": 0.5})
    assert r["ok"] and r["result"]["lines"] > 50
    ev = ask(b, "ui_poll")["result"]["events"]
    assert (
        ev[0]["cmd"] == "program"
        and "class Swarm" in ev[0]["text"]
        and ev[0]["scroll"] == 0.5
    )
    assert not ask(b, "ui.program", {"file": "../../etc/passwd"})["ok"]
    assert ask(b, "ui.program", {})["ok"]  # back to the design TOML
    # an animated drag is committed by the window's own silent drop; the drag's id / group ride along
    assert ask(b, "library.drop", {"part": "cuas-turret-base-plate"})["ok"]
    so = "ettinger-05-12-143-m2.5-standoff"
    r = ask(
        b,
        "library.drag",
        {
            "part": so,
            "to": {"point": [0, 0, 0]},
            "animate": True,
            "id": "so1",
            "group": "g",
        },
    )
    assert r["result"]["queued"]
    assert ask(b, "library.drop", {"part": so, "point": [0, 0, 0], "silent": True})[
        "ok"
    ]
    assert next(i for i in d.doc["instances"] if i["id"] == "so1")["group"] == "g"


def test_assembly_header_op(tmp_path):
    d = Design.new("t", REPO / "parts")
    d.edit([{"op": "add", "part": "cuas-turret-base-plate", "root": True}])
    r = d.edit(
        [{"op": "assembly", "base": "fixed", "timestep": 0.001, "description": "x"}]
    )
    assert (
        d.doc["assembly"]["base"] == "fixed"
        and d.doc["assembly"]["timestep"] == 0.001
        and r.changes[0]["op"] == "assembly"
    )
    for bad in ({}, {"base": "floating"}, {"timestep": 1.0}):
        with pytest.raises(Exception):
            d.edit([{"op": "assembly", **bad}])
    assert d.doc["assembly"]["base"] == "fixed"


def test_pose_and_color_schema_checks():
    doc = A.load(X500).doc
    bad = copy.deepcopy(doc)
    bad["instances"][1]["pos"] = [1, 2]
    with pytest.raises(Exception, match="pos"):
        A.validate(bad)
    bad = copy.deepcopy(doc)
    root = next(i for i in bad["instances"] if i.get("root"))
    root["pos"] = [0, 0, 0]
    with pytest.raises(Exception, match="root"):
        A.validate(bad)
