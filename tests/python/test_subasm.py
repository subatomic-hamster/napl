"""Sub-assemblies (python/napl/design/subasm.py): [[subassemblies]] + [exports], flatten on load, reference rewriting, overrides.
Synthetic parts only (written into tmp_path)."""

import textwrap
import tomllib

import pytest
from napl import design
from napl.design import DesignError, subasm
from napl.design import assembly as A
from napl.design.assembly import _v


def write(p, text):
    p.write_text(textwrap.dedent(text))
    return p


def prm(v, unit="kg", fid="datasheet"):
    return {"value": v, "unit": unit, "fidelity": fid, "src": "S1", "quote": "q"}


def port(name, gender, pos=(0, 0, 0)):
    return {
        "name": name,
        "kind": "mech",
        "type": "tube.12",
        "gender": gender,
        "pos": list(pos),
        "axis": [0, 0, 1],
        "ref": [1, 0, 0],
    }


def write_part(parts, pid, params, ports, mirror="xz"):
    L = [f'id = "{pid}"', 'name = "t"', 'category = "misc"', 'vendor = "t"', "[params]"]
    L += [f"{k} = {_v(e)}" for k, e in {"mass": prm(0.1), **params}.items()]
    for p in ports:
        L += ["", "[[ports]]"] + [f"{k} = {_v(v)}" for k, v in p.items()]
    L += ["", "[geometry]", 'shape = "box"', "size = [0.05, 0.05, 0.05]"]
    if mirror:
        L += ["", "[mirror]", f'symmetry = "{mirror}"']
    parts.mkdir(parents=True, exist_ok=True)
    (parts / f"{pid}.toml").write_text("\n".join(L) + "\n")


def make_parts(tmp_path):
    parts = tmp_path / "parts"
    write_part(
        parts,
        "plate",
        {},
        [
            port("mount_l", "male", (0.1, 0.1, 0)),
            port("mount_r", "male", (0.1, -0.1, 0)),
            port("mount_3", "male", (-0.1, 0.1, 0)),
            port("mount_4", "male", (-0.1, -0.1, 0)),
        ],
    )
    write_part(parts, "brk", {}, [port("base", "female"), port("out", "male")])
    out2 = port("out", "male", (0, 0.03, 0.05))
    write_part(parts, "brk2", {}, [port("base", "female"), out2])
    write_part(parts, "nomirror", {}, [port("base", "female")], mirror=None)
    write_part(
        parts,
        "mot",
        {"rate": prm(100.0, "Hz"), "lead": prm(0.2, "m")},
        [port("in", "female")],
    )
    return parts


LEG = """
    [assembly]
    name = "leg"
    [exports]
    ports = { mount = "brk.base" }
    params = { cable_len = "mot.lead" }
    [[instances]]
    id = "brk"
    part = "brk"
    root = true
    group = "leg"
    [[instances]]
    id = "mot"
    part = "mot"
    [[mates]]
    name = "brk--mot"
    a = "brk.out"
    b = "mot.in"
    [[devices]]
    type = "imu"
    name = "imu"
    instance = "mot"
    body = "mot"
    set = { rate_hz = "mot.rate" }
    [[targets]]
    name = "leg_mass"
    all = true
    value = { value = 1.0, unit = "kg", fidelity = "estimated", rationale = "test" }
"""
BOT = """
    [assembly]
    name = "bot"
    [[instances]]
    id = "plate"
    part = "plate"
    root = true
    [[subassemblies]]
    id = "l"
    file = "leg.toml"
    {l_extra}
    [[subassemblies]]
    id = "r"
    file = "leg.toml"
    [[mates]]
    name = "pl"
    a = "plate.mount_l"
    b = "l:mount"
    [[mates]]
    name = "pr"
    a = "plate.mount_r"
    b = "r:mount"
"""


def bot(tmp_path, l_extra=""):
    write(tmp_path / "leg.toml", LEG)
    return write(
        tmp_path / "bot.toml",
        BOT.replace("{l_extra}", textwrap.dedent(l_extra).strip()),
    )


# ---- Task 1 ----------------------------------------------------------------------------------------------------------------------
def test_expand_prefixes_and_origins(tmp_path):
    asm = A.load(bot(tmp_path))
    ids = [i["id"] for i in asm.doc["instances"]]
    assert ids.count("plate") == 1 and "l:brk" in ids and "r:mot" in ids
    assert asm.origins["l:mot"] == {"sub": "l", "file": "leg.toml", "id": "mot"}
    assert asm.origins["r:imu"] == {"sub": "r", "file": "leg.toml", "id": "imu"}
    assert not any(i.get("root") for i in asm.doc["instances"] if ":" in i["id"])
    assert "subassemblies" not in asm.doc and asm.raw["subassemblies"][0]["id"] == "l"
    assert {m["name"]: m["b"] for m in asm.doc["mates"]}["pl"] == "l:brk.base"


def test_loads_without_path_rejects_subassemblies(tmp_path):
    text = bot(tmp_path).read_text()
    with pytest.raises(DesignError, match="needs the parent file's path"):
        A.loads(text)
    assert A.loads(text, tmp_path / "bot.toml").doc["instances"][-1]["id"] == "r:mot"


def test_cycle(tmp_path):
    inc = '[assembly]\nname = "x"\n[[subassemblies]]\nid = "s"\nfile = "{}"\n'
    write(tmp_path / "a.toml", inc.format("b.toml"))
    write(tmp_path / "b.toml", inc.format("a.toml"))
    with pytest.raises(DesignError, match=r"cycle: a.toml -> b.toml -> a.toml"):
        A.load(tmp_path / "a.toml")


def test_reach_inside_is_error(tmp_path):
    p = bot(tmp_path)
    p.write_text(p.read_text().replace('b = "r:mount"', 'b = "r:brk.base"'))
    with pytest.raises(DesignError, match=r"export it in leg.toml \[exports\]"):
        A.load(p)


def test_unknown_export_and_bad_schema(tmp_path):
    p = bot(tmp_path)
    p.write_text(p.read_text().replace('b = "r:mount"', 'b = "r:nope"'))
    with pytest.raises(
        DesignError, match=r"not an export of leg.toml \(exports: mount\)"
    ):
        A.load(p)
    q = bot(tmp_path, "bogus = 1")
    with pytest.raises(DesignError, match="unknown keys"):
        A.load(q)


def test_save_writes_only_the_parent(tmp_path):
    asm = A.load(bot(tmp_path))
    text = A.dumps(asm)
    assert text.count("[[subassemblies]]") == 2
    assert "l:brk" not in text and "[[devices]]" not in text
    assert A.loads(text, tmp_path / "bot.toml") == asm


def test_no_subassemblies_unchanged(tmp_path):
    doc = tomllib.loads(LEG)
    flat, origins = subasm.expand(doc, None)
    assert flat is doc and origins == {}
    asm = A.loads(LEG)
    asm.doc = {
        **asm.doc,
        "assembly": {"name": "z"},
    }  # undo-style replacement stays in step with what is saved
    assert 'name = "z"' in A.dumps(asm)


# ---- Task 2 ----------------------------------------------------------------------------------------------------------------------
def test_every_device_type_has_ref_fields():
    assert set(subasm.REF_FIELDS) >= A.DEVICE_TYPES
    assert set(subasm.REF_FIELDS["servo"]) >= {"instance", "motor", "joint", "map"}
    assert set(subasm.FIELD_KIND) >= {
        f for fs in subasm.REF_FIELDS.values() for f in fs
    }


def _all_types_child(tmp_path, extra=None):
    devs = []
    for t in sorted(A.DEVICE_TYPES):
        d = {"type": t, "name": f"d_{t}", "port": "p", "a": "ground", "b": "p"}
        d |= {f: "x" for f in subasm.REF_FIELDS[t]} | (extra or {})
        d["set"] = {"k": "x.v", "j": ["x.v", "x.w"]}
        devs.append(d)
    lines = [
        '[assembly]\nname = "c"',
        '[[instances]]\nid = "x"\npart = "p"\nroot = true',
        '[device_maps.x]\n"f" = "p"',
    ]
    lines += [
        "[[devices]]\n" + "\n".join(f"{k} = {_v(v)}" for k, v in d.items())
        for d in devs
    ]
    write(tmp_path / "c.toml", "\n".join(lines) + "\n")
    parent = write(
        tmp_path / "p.toml",
        '[assembly]\nname = "p"\n[[subassemblies]]\nid = "l"\nfile = "c.toml"\n',
    )
    return subasm.expand(tomllib.loads(parent.read_text()), parent)


def test_device_refs_prefixed(tmp_path):
    flat, origins = _all_types_child(tmp_path)
    assert len(flat["devices"]) == len(A.DEVICE_TYPES)
    for d in flat["devices"]:
        assert d["name"] == "l:" + d["name"][2:] and d["name"].startswith("l:d_")
        assert origins[d["name"]]["id"] == d["name"][2:]
        for f in subasm.REF_FIELDS[d["type"]]:
            assert d[f] == "l:x", (d["type"], f)
        assert (d["port"], d["a"], d["b"]) == (
            "p",
            "ground",
            "p",
        )  # port names are not references
        assert d["set"] == {"k": "l:x.v", "j": ["l:x.v", "l:x.w"]}
    assert "l:x" in flat["device_maps"] and flat["device_maps"]["l:x"] == {"f": "p"}


def test_unknown_string_field_is_error_not_silent(tmp_path):
    with pytest.raises(DesignError, match=r"REF_FIELDS"):
        _all_types_child(tmp_path, {"mystery": "x"})


def test_device_binds_to_prefixed_bodies_on_build(tmp_path):
    parts = make_parts(tmp_path)
    asm = A.load(bot(tmp_path))
    xml, rep = design.compute(asm, parts, tmp_path / "out")
    assert sorted(d["name"] for d in rep["devices"]) == ["l:imu", "r:imu"]
    assert 'name="l:mot"' in xml and 'name="r:brk"' in xml
    scope = {t["name"]: t["scope"] for t in rep["targets"]}
    assert scope == {"l:leg_mass": ["l", "l:leg"], "r:leg_mass": ["r", "r:leg"]}


def test_param_refs_prefixed(tmp_path):
    inc = """
        [assembly]
        name = "c"
        [[instances]]
        id = "h"
        part = "mot"
        root = true
        [[wires]]
        from = "h.a"
        to = "h.b"
        part = "w"
        length = "h.lead"
        included_in = "h"
        [[targets]]
        name = "t"
        groups = ["g"]
        value = "h.lead"
    """
    write(tmp_path / "c.toml", inc)
    p = write(
        tmp_path / "p.toml",
        '[assembly]\nname = "p"\n[[subassemblies]]\nid = "l"\nfile = "c.toml"\ngroup = "legs"\n',
    )
    flat, _ = subasm.expand(tomllib.loads(p.read_text()), p)
    w = flat["wires"][0]
    assert (w["from"], w["to"], w["length"], w["included_in"]) == (
        "l:h.a",
        "l:h.b",
        "l:h.lead",
        "l:h",
    )
    assert flat["targets"][0]["value"] == "l:h.lead" and flat["targets"][0][
        "groups"
    ] == ["legs"]
    assert flat["instances"][0]["group"] == "legs"


# ---- Task 3 ----------------------------------------------------------------------------------------------------------------------
def test_override_applies_to_one_copy(tmp_path):
    val = '{ value = 0.32, unit = "m", fidelity = "derived", derivation = "test" }'
    asm = A.load(bot(tmp_path, f"params = {{ cable_len = {val} }}"))
    inst = {i["id"]: i for i in asm.doc["instances"]}
    assert inst["l:mot"]["params"]["lead"]["value"] == 0.32
    assert "lead" not in inst["r:mot"].get("params", {})
    inst["l:mot"]["params"]["lead"]["value"] = 9  # copies share nothing mutable
    inst["l:brk"]["group"] = "changed"
    assert inst["r:brk"]["group"] == "r:leg"
    assert asm.raw["subassemblies"][0]["params"]["cable_len"]["value"] == 0.32
    assert asm.origins["l:mot"]["sub"] == "l"


def test_override_of_unexported_param(tmp_path):
    val = '{ value = 0.32, unit = "m", fidelity = "derived", derivation = "test" }'
    with pytest.raises(DesignError, match="exports: cable_len"):
        A.load(bot(tmp_path, f"params = {{ lead = {val} }}"))


def test_override_needs_provenance(tmp_path):
    with pytest.raises(DesignError, match="provenance|bare number"):
        A.load(bot(tmp_path, "params = { cable_len = 0.32 }"))


def test_nested_subassemblies(tmp_path):
    write(tmp_path / "leg.toml", LEG)
    write(
        tmp_path / "mid.toml",
        """
        [assembly]
        name = "mid"
        [exports]
        ports = { up = "k:mount" }
        params = { cable = "k:cable_len" }
        [[instances]]
        id = "hub"
        part = "plate"
        root = true
        [[subassemblies]]
        id = "k"
        file = "leg.toml"
        [[mates]]
        name = "hk"
        a = "hub.mount_l"
        b = "k:mount"
        """,
    )
    top = write(
        tmp_path / "top.toml",
        """
        [assembly]
        name = "top"
        [[instances]]
        id = "body"
        part = "plate"
        root = true
        [[subassemblies]]
        id = "fl"
        file = "mid.toml"
        params = { cable = { value = 0.5, unit = "m", fidelity = "derived", derivation = "t" } }
        [[mates]]
        name = "bf"
        a = "body.mount_l"
        b = "fl:up"
        """,
    )
    asm = A.load(top)
    inst = {i["id"]: i for i in asm.doc["instances"]}
    assert "fl:k:mot" in inst and inst["fl:k:mot"]["params"]["lead"]["value"] == 0.5
    assert asm.origins["fl:k:mot"] == {"sub": "fl:k", "file": "leg.toml", "id": "mot"}
    assert {m["name"]: m["b"] for m in asm.doc["mates"]}["bf"] == "fl:k:brk.base"
