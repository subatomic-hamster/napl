"""Backend sub.* ops (viewer.py), Design.reload_children, uilib.my_parts."""

import json

import pytest
from napl.design import Design, Library, uilib
from napl.design.viewer import Backend
from test_subasm import LEG, make_parts, write

CABLE = {"value": 0.32, "unit": "m", "fidelity": "derived", "derivation": "test"}
PORTS = (("fl", "mount_l"), ("fr", "mount_r"), ("bl", "mount_3"), ("br", "mount_4"))


def raw_call(b, tool, **args):
    return json.loads(b(json.dumps({"tool": tool, "args": args})))


def call(b, tool, **args):
    r = raw_call(b, tool, **args)
    assert r["ok"], r
    return r


def chassis(tmp_path):
    parts = make_parts(tmp_path)
    write(tmp_path / "leg.toml", LEG)
    d = Design.new("dog", parts)
    d.add("plate", id="plate", root=True)
    d.save(tmp_path / "dog.toml")
    return d, parts, Backend(lambda: d, tmp_path / "out")


def insert_legs(b, **extra):
    for sid, port in PORTS:
        call(
            b,
            "sub.insert",
            file="leg.toml",
            id=sid,
            mate={"a": f"plate.{port}", "export": "mount"},
            **extra,
        )


def test_insert_override_save_reload(tmp_path):  # Review Focus 5
    d, parts, b = chassis(tmp_path)
    insert_legs(b)
    r = call(b, "sub.override", id="fl", name="cable_len", value=CABLE)
    assert [s["id"] for s in r["state"]["subs"]] == ["fl", "fr", "bl", "br"]
    assert r["state"]["origins"]["fl:mot"] == {
        "sub": "fl",
        "file": "leg.toml",
        "id": "mot",
    }
    d.save()
    text = (tmp_path / "dog.toml").read_text()
    assert text.count("[[subassemblies]]") == 4 and text.count("[[instances]]") == 1
    assert "fl:mot" not in text and "[[devices]]" not in text
    d2 = Design.open(tmp_path / "dog.toml", parts)
    assert d2.doc == d.doc and len(d2.doc["instances"]) == 1 + 4 * 2
    inst = {i["id"]: i for i in d2.doc["instances"]}
    assert inst["fl:mot"]["params"]["lead"]["value"] == 0.32
    d.undo()  # the override
    assert "params" not in d.asm.raw["subassemblies"][0]


def test_four_legs_two_mirrored_and_mirror_op(tmp_path):
    d, parts, b = chassis(tmp_path)
    for sid, port in PORTS:
        call(
            b,
            "sub.insert",
            file="leg.toml",
            id=sid,
            mate={"a": f"plate.{port}", "export": "mount"},
            **({"mirror": "xz"} if sid in ("fr", "br") else {}),
        )
    d.save()
    assert d.asm.raw["subassemblies"][1]["mirror"] == "xz"
    assert d.build()["report"]["mirror_signs"] == {}  # no hinge in this leg
    # sub.mirror: a mirrored copy of fl hung on another mount; toggling clears / sets the flag
    d2 = Design.open(tmp_path / "dog.toml", parts)
    b2 = Backend(lambda: d2, tmp_path / "out2")
    call(b2, "sub.override", id="fl", name="cable_len", value=CABLE)
    r = call(b2, "sub.mirror", id="fr", plane="xz", toggle=True)
    assert "mirror" not in d2.asm.raw["subassemblies"][1] and r["ok"]
    r = call(b2, "sub.mirror", id="fl", plane="xz", new_id="fl_m")
    assert d2.asm.raw["subassemblies"][-1] == {
        "id": "fl_m",
        "file": "leg.toml",
        "mirror": "xz",
        "params": {"cable_len": CABLE},
    }


def test_bad_ops_change_nothing(tmp_path):
    d, parts, b = chassis(tmp_path)
    insert_legs(b)
    rev = b.rev_of(d)
    bad = raw_call(b, "sub.insert", file="leg.toml", id="fl")
    assert not bad["ok"] and "already exists" in bad["error"]
    bad = raw_call(b, "sub.insert", file="nope.toml", id="x")
    assert not bad["ok"] and "nope.toml" in bad["error"] and b.rev_of(d) == rev
    bad = raw_call(b, "sub.override", id="fl", name="nope", value=CABLE)
    assert not bad["ok"] and "exports: cable_len" in bad["error"]
    assert b.rev_of(d) == rev


def test_open_close_breadcrumb_and_separate_undo(tmp_path):
    d, parts, b = chassis(tmp_path)
    insert_legs(b)
    d.save()
    n = len(d._undo)
    r = call(b, "sub.open", id="fl")
    assert [x["crumb"] for x in r["state"]["docs"]] == [
        ["dog"],
        ["dog", "fl (leg.toml)"],
    ]
    r = call(b, "sub.export", kind="param", ref="mot.rate", name="rate")
    assert r["result"]["undo_depth"] == 1 and len(d._undo) == n  # separate stacks
    assert r["state"]["docs"][-1]["dirty"]
    assert r["state"]["exports"]["params"]["rate"] == "mot.rate"
    r = call(b, "sub.close")  # saves the dirty child, re-flattens the parent
    assert [x["crumb"] for x in r["state"]["docs"]] == [["dog"]]
    assert "rate = " in (tmp_path / "leg.toml").read_text()
    assert not raw_call(b, "sub.close")["ok"]  # nothing open any more


def test_new_part_crumb_and_unsaved_close(tmp_path):
    d, parts, b = chassis(tmp_path)
    r = call(b, "sub.new", name="foot", base="free")
    assert r["result"]["crumb"] == ["dog", "foot"]
    call(
        b, "design_edit", ops=[{"op": "add", "part": "plate", "id": "p", "root": True}]
    )
    out = raw_call(b, "sub.close")
    assert not out["ok"] and "no file yet" in out["error"]
    r = call(b, "sub.close", path=str(tmp_path / "foot.toml"))
    assert r["result"]["saved"].endswith("foot.toml")
    assert (tmp_path / "foot.toml").exists()


def test_reload_children_picks_up_disk_edit(tmp_path):  # Review Focus 3
    d, parts, b = chassis(tmp_path)
    insert_legs(b)
    m0 = d.build()["report"]["mass"]["total"]
    heavy = LEG.replace(
        'part = "mot"',
        'part = "mot"\n    params = { mass = { value = 0.9, unit = "kg", fidelity = "estimated", rationale = "t" } }',
    )
    write(tmp_path / "leg.toml", heavy)
    assert d.reload_children() is True
    assert d.build()["report"]["mass"]["total"] == pytest.approx(m0 + 4 * 0.8)
    assert d.reload_children() is False


def test_my_parts(tmp_path):
    d, parts, b = chassis(tmp_path)
    rows = uilib.my_parts(tmp_path)
    assert [r["name"] for r in rows] == ["leg"]
    assert rows[0]["exports"] == {"ports": ["mount"], "params": ["cable_len"]}
    assert rows[0]["mass"] is None
    assert uilib.my_parts(tmp_path, Library(parts))[0]["mass"] == pytest.approx(0.2)
