"""S4: Design.auto_devices derives the device bindings from parts + topology (no [devices] / [device_maps] written by hand).

* the four reference assemblies with their device sections stripped: the derived napl.devices JSON is compared with the
  reference's, field by field. A difference is listed in DIFFS with its reason or the test fails; a field no part carries is
  reported `unset` and supplied from the reference's own `set` (`fill`), nothing else is.
* the scratch quad (tests/python/test_design_scratch.py) with auto_devices instead of bind_devices hovers like the reference
  (that file's own tests); here: the `lead` port field makes autowire set included_in (and the lead's length / gauge).
* swapping the X500 motors: stale bindings are reported, the refresh is in the same edit, DRC + hover are run.
"""

import asyncio
import copy
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import pytest
from mcp import Client
from napl.design import Design, DesignError, Library
from napl.design.auto_devices import _key
from napl.mcp_server import NaplServer
from test_design_mcp import accept, ok, root  # noqa: F401  (root is a fixture)
from test_design_scratch import ARM, FILL, build_scratch, with_leads

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
ASMS = REPO / "assemblies"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def devices_json(d: Design, out) -> dict:
    """The napl.devices JSON of the built MJCF."""
    r = d.build(out)
    root_ = ET.parse(r["xml"]).getroot()
    return json.loads(
        next(t for t in root_.iter("text") if t.get("name") == "napl.devices").get(
            "data"
        )
    )


def flat(x, path=""):
    """{path: leaf} of a nested JSON value (lists are indexed)."""
    if isinstance(x, dict):
        return {
            k2: v2
            for k, v in x.items()
            for k2, v2 in flat(v, f"{path}.{k}" if path else k).items()
        }
    if isinstance(x, list):
        return {
            k2: v2
            for i, v in enumerate(x)
            for k2, v2 in flat(v, f"{path}[{i}]").items()
        }
    return {path: x}


def close(a, b, rel):
    if (
        isinstance(a, (int, float))
        and isinstance(b, (int, float))
        and not isinstance(a, bool)
    ):
        return math.isclose(a, b, rel_tol=rel, abs_tol=1e-300)
    return a == b


def dev_key(kind, d):
    """Identity of a device across the two builds (the names differ: rotor1 / rotor_prop1)."""
    if kind in ("escs", "rotors", "servos"):
        return (kind, d["joint"])
    if kind in ("imus", "baros", "mags", "gnss_receivers"):
        return (kind, d["site"])
    return (kind, d["name"])


def index(js):
    return {dev_key(k, d): d for k, v in js.items() if isinstance(v, list) for d in v}


def compare(ref_js, new_js):
    """-> ({key: {path: (ref, new)}} for the devices in both builds, keys only in the reference, keys only in the derived)."""
    r, n = index(ref_js), index(new_js)
    diffs, equal = {}, 0
    for k in r.keys() & n.keys():
        fr = flat({a: b for a, b in r[k].items() if a != "name"})
        fn = flat({a: b for a, b in n[k].items() if a != "name"})
        dd = {
            p: (fr.get(p), fn.get(p))
            for p in fr.keys() | fn.keys()
            if not close(fr.get(p), fn.get(p), 1e-9)
        }
        equal += len(fr.keys() | fn.keys()) - len(dd)
        if dd:
            diffs[k] = dd
    return diffs, equal, sorted(r.keys() - n.keys()), sorted(n.keys() - r.keys())


# Every difference between a derived and a reference binding, with its reason. (assembly, device kind, field regex, rel tol or
# None, reason). A numeric rule with a tolerance matches a rounded copy of the same number; None = any difference.
DIFFS = [
    (
        "x500v2",
        "rotors",
        r"cq\[\d+\]",
        None,
        "the reference rescales the whole cq list with cq_ref = 0.005794, the C_Q(J=0) of the napl thrust-stand bench fit "
        "(derived, not in any part); the derived cq is the prop part's own table (C_Q(J=0) = 0.00526, vendor torque rounded to 0.01 N m)",
    ),
    (
        "x500v2",
        "rotors",
        r"k_h",
        None,
        "the reference sets k_h = 0 explicitly; 0 is the schema default (H-force off)",
    ),
    ("aos5", "rotors", r"k_h", None, "as x500v2: k_h = 0 is the schema default"),
    (
        "x500v2",
        "loads",
        r"i0",
        2e-3,
        "the reference types the avionics current rounded to 3-4 digits (-0.1083 A); derived = 5.2 V x I / (0.85 x 14.8 V) unrounded",
    ),
    (
        "x500v2",
        "gnss_receivers",
        r"horiz_std|vert_std",
        1e-4,
        "the reference types 2.1233 / 3.185 m (rounded); derived = CEP / 1.17741 and its ratio to the vertical CEP, unrounded",
    ),
    (
        "x500v2",
        "gnss_receivers",
        r"latency_s|corr_time_s|corr_frac",
        None,
        "estimates the reference assembly makes (no part publishes the receiver latency or its error correlation): unset here, "
        "the schema defaults (0 = no lag, white noise) apply; not reported as unset because the device loads without them",
    ),
    (
        "x500v2",
        "environments",
        r"w20",
        None,
        "the reference sets w20 = 0 explicitly; 0 is the schema default (calm air)",
    ),
    (
        "so101",
        "servos",
        r"p",
        None,
        "the reference uses the LeRobot SO-101 preset (reg_p_lerobot_so101 = 16); derived uses the part's factory default "
        "(reg_p_coefficient_default = 32): nothing in the topology says this arm runs the LeRobot configuration",
    ),
    (
        "so101",
        "servos",
        r"max_torque",
        None,
        "the reference sets the gripper's Max_Torque_Limit to LeRobot's 500 (lerobot_gripper_max_torque_limit); derived = factory 1000",
    ),
]
# unset before the fill: what no part carries (device -> fields)
UNSET = {
    "x500v2": {
        **{f"esc{k}": ["motor.c_th", "motor.r_th"] for k in range(1, 5)},
        "imu": ["rate_hz"],
        "baro": ["rate_hz"],
        "mag": ["rate_hz", "earth_field"],
        "gnss": ["lat0", "lon0", "alt0"],
    },
    "aos5": {
        f"esc{k}": ["motor.rotor_inertia", "motor.c_th", "motor.r_th"]
        for k in range(1, 5)
    },
    "so101": {
        "adapter_board": ["resistance"],
        **{f"servo{k}": ["motor.c_th", "motor.r_th", "pid_scale"] for k in range(1, 7)},
    },
    "koch": {
        "adapter_board": ["resistance"],
        **{f"servo{k}": ["motor.c_th", "motor.r_th"] for k in range(1, 7)},
    },
}
NAMES = {  # reference name -> derived name, where the rule differs (rotor_<prop instance>)
    "x500v2": {f"rotor{k}": f"rotor_prop{k}" for k in range(1, 5)},
    "aos5": {f"rotor{k}": f"rotor_prop{k}" for k in range(1, 5)},
}


def strip_and_derive(name, tmp_path):
    """Open the reference, strip its devices, auto_devices, fill what is unset from the reference's own `set`."""
    d = Design.open(ASMS / f"{name}.toml")
    ref_doc = copy.deepcopy(d.doc)
    ref_js = devices_json(d, tmp_path / "ref")
    d.doc.pop("devices")
    d.doc.pop("device_maps")
    rec = d.auto_devices().changes[0]
    unset = {u["device"]: u["fields"] for u in rec["unset"]}
    refb = {_key(b): b for b in ref_doc["devices"]}
    newb = {b["name"]: b for b in d.doc["devices"]}
    fill = {}
    for dev, fields in unset.items():
        rb = refb[_key(newb[dev])]
        gap = [f for f in fields if f not in rb.get("set", {})]
        assert not gap, (dev, "unset, and not in the reference's `set` either", gap)
        fill[dev] = {"set": {f: rb["set"][f] for f in fields}}
    rec2 = d.auto_devices(fill=fill).changes[0]
    assert rec2["unset"] == [], rec2["unset"]
    return d, ref_doc, ref_js, unset, rec2


@pytest.mark.parametrize("name", ["x500v2", "aos5", "so101", "koch"])
def test_stripped_reference_is_reproduced_and_every_difference_is_explained(
    name, tmp_path, capsys
):
    d, ref_doc, ref_js, unset, rec = strip_and_derive(name, tmp_path)
    assert unset == UNSET[name]
    new_js = devices_json(d, tmp_path / "new")
    diffs, equal, only_ref, only_new = compare(ref_js, new_js)
    assert only_ref == [] and only_new == [], (
        only_ref,
        only_new,
    )  # the same devices on the same joints / sites / nodes
    # names: identical except where the naming rule differs
    ref_names = {x["name"] for k, v in ref_js.items() if isinstance(v, list) for x in v}
    new_names = {x["name"] for k, v in new_js.items() if isinstance(v, list) for x in v}
    rename = NAMES.get(name, {})
    assert {rename.get(n, n) for n in ref_names} == new_names
    hit, bad = {}, []
    for (kind, _), dd in diffs.items():
        for path, (a, b) in dd.items():
            rule = next(
                (
                    i
                    for i, (nm, kd, rx, tol, _) in enumerate(DIFFS)
                    if nm == name
                    and kd == kind
                    and re.fullmatch(rx, path)
                    and (tol is None or close(a, b, tol))
                ),
                None,
            )
            if rule is None:
                bad.append((kind, path, a, b))
            else:
                hit[rule] = hit.get(rule, 0) + 1
    assert bad == [], f"unexplained differences: {bad}"
    mine = {i for i, r in enumerate(DIFFS) if r[0] == name}
    assert set(hit) == mine, f"rules that matched nothing: {sorted(mine - set(hit))}"
    # cq_ref is a uniform scale of the cq list: the J shape is the part's
    if name == "x500v2":
        ratios = {
            round(diffs[k][f"cq[{i}]"][0] / diffs[k][f"cq[{i}]"][1], 6)
            for k in diffs
            if k[0] == "rotors"
            for i in range(18)
            if f"cq[{i}]" in diffs[k]
        }
        assert len(ratios) == 1, ratios
    with capsys.disabled():
        n_diff = sum(len(v) for v in diffs.values())
        print(
            f"\n{name}: {len(index(new_js))} devices derived; {equal} leaf fields identical, {n_diff} differ (all explained); "
            f"unset before fill: {sum(len(v) for v in unset.values())} fields on {len(unset)} devices"
        )


def test_provenance_every_bound_field_names_a_part_param_and_every_computed_value_says_how(
    tmp_path,
):
    d, *_ = strip_and_derive("x500v2", tmp_path)
    lib = d.lib
    pid = {i["id"]: i["part"] for i in d.doc["instances"]}
    for b in d.doc["devices"]:
        assert b["auto"] is True
        spec_inst = {
            "": b.get("instance"),
            **{k: v for k, v in b.items() if isinstance(v, str)},
        }
        for f, spec in d.doc.get("device_maps", {}).get(b.get("map"), {}).items():
            role, _, p = spec.rpartition(":")
            inst = spec_inst[role]
            assert p.startswith("@") or p in lib.get(pid[inst]).params, (
                b["name"],
                f,
                spec,
            )
        for f, e in b.get("set", {}).items():
            assert isinstance(e, dict) and (
                e.get("derivation") or e.get("rationale") or e.get("src")
            ), (b["name"], f)
    c1 = next(b for b in d.doc["devices"] if b["name"] == "battery")["set"]["c1"]
    assert (
        "thevenin_tau" in c1["rationale"]
        and "thevenin_r1" in c1["rationale"]
        and c1["fidelity"] == "estimated"
    )
    load = next(b for b in d.doc["devices"] if b["name"] == "load_fc")["set"]["i0"]
    assert (
        "supply_current_typ" in load["rationale"]
        and "bec_efficiency" in load["rationale"]
    )


def test_hand_written_bindings_win_and_overwrite_replaces_them(tmp_path):
    d = Design.open(ASMS / "so101.toml")
    before = copy.deepcopy(d.doc)
    rec = d.auto_devices().changes[0]
    assert d.doc == before  # every explicit binding kept, nothing written
    assert (
        rec["created"] == []
        and len(rec["kept"]) == len(before["devices"])
        and rec["summary"]["kept"] == 8
    )
    rec = d.auto_devices(overwrite=["servo1"]).changes[
        0
    ]  # a list of device names: only those
    assert [x["device"] for x in rec["created"] + rec["refreshed"]] == [
        "servo1"
    ] and len(rec["kept"]) == 7
    s1 = next(b for b in d.doc["devices"] if b["name"] == "servo1")
    assert (
        s1["auto"] is True and "set" not in s1
    )  # the reference's pid_scale / c_th / r_th are gone: now reported
    assert rec["unset"] == [
        {"device": "servo1", "fields": ["motor.c_th", "motor.r_th", "pid_scale"]}
    ]
    d.undo()
    rec = d.auto_devices(overwrite=True).changes[0]
    assert rec["kept"] == [] and all(b.get("auto") for b in d.doc["devices"])


def test_filled_values_survive_regeneration_and_explicit_fill_wins(tmp_path):
    d, *_ = strip_and_derive("koch", tmp_path)
    first = copy.deepcopy(d.doc["devices"])
    rec = d.auto_devices().changes[0]  # no fill this time
    assert d.doc["devices"] == first and rec["summary"]["unchanged"] == len(first)
    assert next(x for x in rec["unchanged"] if x["device"] == "servo1")["carried"] == [
        "motor.c_th",
        "motor.r_th",
    ]
    new = {"value": 20.0, "unit": "K/W", "fidelity": "estimated", "rationale": "test"}
    rec = d.auto_devices(
        fill={"servo1": {"set": {"motor.r_th": new}}, "nope*": {"set": {}}}
    ).changes[0]
    assert (
        next(b for b in d.doc["devices"] if b["name"] == "servo1")["set"]["motor.r_th"]
        == new
    )
    assert [x["device"] for x in rec["refreshed"]] == ["servo1"]
    assert any(
        "nope*" in n for n in rec["notes"]
    )  # a pattern that matched nothing is said so


def test_a_binding_the_design_no_longer_implies_is_removed(tmp_path):
    d, *_ = strip_and_derive("aos5", tmp_path)
    r = d.remove("mot4")
    gone = {
        x["name"] for x in r.changes[0]["devices_removed"]
    }  # remove drops the bindings naming it
    assert "esc4" in gone, gone
    rec = d.auto_devices().changes[0]
    names = {b["name"] for b in d.doc["devices"]}
    assert "esc4" not in names and "esc3" in names, (names, rec["notes"])


# ---- the `lead` port field ---------------------------------------------------------------------------------------------------
def test_lead_flag_makes_autowire_set_included_in_and_use_the_leads_own_length_and_gauge():
    explicit, _ = build_scratch(devices="hand")
    leads, _ = build_scratch(lib=with_leads(Library(PARTS)), leads=True)
    inc = lambda d: {w["name"]: w.get("included_in") for w in d.doc["wires"]}
    assert inc(leads) == inc(
        explicit
    )  # all 19 wires: the same lead-inside-the-datasheet-mass flags, none typed
    # ... but the receiver's two pigtails are the user's own cable (the R24 ships none): never inside a datasheet mass
    own = [n for n in inc(leads) if "rx." in n]
    assert len(own) == 2 and not any(inc(leads)[n] for n in own)
    assert all(v for n, v in inc(leads).items() if n not in own)
    bat = next(w for w in leads.doc["wires"] if w["from"] == "bat.power_out")
    assert (
        bat["length"] == "bat.discharge_wire_length"
        and bat["part"] == "awg-10-silicone-wire"
    )  # the pack's own lead
    mot = next(w for w in leads.doc["wires"] if w["to"] == "mot1.phase")
    assert mot["length"] == "mot1.cable_length"
    pdb = next(w for w in leads.doc["wires"] if w["to"] == "esc1.power_in")
    assert (
        isinstance(pdb["length"], dict) and pdb["length"]["fidelity"] == "derived"
    )  # flag only: routed length
    assert [f for f in leads.check() if f.severity == "error"] == []
    r = leads.build()["report"]
    assert (
        sum(
            w["mass"] or 0
            for w in r["wires"]
            if not w["included_in"] and w["name"] not in own
        )
        == 0
    )


def test_lead_with_a_param_the_part_lacks_is_refused():
    lib = Library(PARTS)
    for p in lib.get("tattu-4s-5200mah-35c").ports:
        if p["name"] == "power_out":
            p["lead"] = {"length": "no_such_param"}

    with pytest.raises(DesignError, match="no_such_param"):
        build_scratch(lib=lib, leads=True)


# ---- swap -----------------------------------------------------------------------------------------------------------------
def kde_as_x500_motor(lib):
    """The KDE 2315XF-965 (datasheet j_rotor_bell, thrust table; 4S): its 16x19 M3 pattern and M6 prop adapter seat on the X500 arm
    connector / prop as they are (D5: no in-memory port patching any more); the point of the tests is the device refresh."""
    return "kde-2315xf-965"


def test_swap_reports_stale_bindings_and_offers_the_refresh(tmp_path):
    d = Design.open(ASMS / "x500v2.toml")
    kde = kde_as_x500_motor(d.lib)
    ch = d.swap("mot1", kde).changes[0]
    assert [(x["device"], x["generated"]) for x in ch["stale_devices"]] == [
        ("esc1", False)
    ]  # hand-written: named, not touched
    assert "auto_devices_offer" in ch and "auto_devices" not in ch
    d.undo()
    ch = d.swap("mot1", kde, auto=True).changes[
        0
    ]  # same edit: the refresh runs, but the explicit binding wins
    assert ch["auto_devices"]["kept"] and ch["auto_devices"]["created"] == []
    d.undo()
    ch = d.swap("mot1", kde, auto=True, overwrite=True).changes[
        0
    ]  # explicit stale ones replaced too
    esc1 = next(b for b in d.doc["devices"] if b["name"] == "esc1")
    assert esc1["auto"] is True
    assert [
        x["device"]
        for x in ch["auto_devices"]["refreshed"] + ch["auto_devices"]["created"]
    ] == ["esc1"]
    assert ch["auto_devices"]["unset"] == [
        {"device": "esc1", "fields": ["motor.c_th", "motor.r_th"]}
    ]  # reported, not invented


@pytest.fixture(scope="module")
def swapped():
    """The auto-bound scratch quad, all four motors swapped for the KDE part with the refresh in the same edits."""
    lib = Library(PARTS)
    kde = kde_as_x500_motor(lib)
    d, _ = build_scratch(lib=lib)
    before = copy.deepcopy(d.doc["devices"])
    r = d.swap(
        [f"mot{k}" for k in ARM], kde, auto=True, rewire=True
    )  # the Holybro motor's `lead` made autowire name its cable_length; the KDE has none
    return d, before, r, kde


def test_swap_with_auto_refreshes_only_the_motor_dependent_bindings(swapped):
    d, before, r, kde = swapped
    assert len(r.changes) == 4
    for ch in r.changes[:-1]:
        assert [x["generated"] for x in ch["stale_devices"]] == [True]
    last = r.changes[-1]["auto_devices"]
    assert (
        last["summary"]["unset"] == 0
    )  # the fill (c_th / r_th) was carried: reported below, not silently kept
    esc = [x for x in last["unchanged"] + last["refreshed"] if x["type"] == "esc"]
    assert (
        all(set(x["carried"]) == {"motor.c_th", "motor.r_th"} for x in esc)
        and len(esc) == 4
    )
    # one refresh per swap, each touching only the ESC whose motor just changed
    assert [
        [x["device"] for x in c["auto_devices"]["refreshed"]] for c in r.changes
    ] == [[f"esc{k}"] for k in ARM]
    after = {b["name"]: b for b in d.doc["devices"]}
    for b in before:
        if b["type"] != "esc":
            assert after[b["name"]] == b, b["name"]
    esc1 = after["esc1"]
    assert d.doc["device_maps"][esc1["map"]]["motor.kv"] == "motor:kv"
    assert [m for m in d.doc["device_maps"] if m.startswith("auto_esc")] == [
        esc1["map"]
    ]  # the old map is gone
    assert "kde-2315xf-965" in esc1["map"]
    # the new motor's own numbers reached the device (kv 965, R 0.102 ohm), not the old ones
    js = devices_json(d, Path(d.build()["xml"]).parent)
    e = next(x for x in js["escs"] if x["name"] == "esc1")
    assert e["motor"]["kv"] == 965 and math.isclose(e["motor"]["r20"], 0.102)
    assert (
        e["motor"]["rotor_inertia"] == 4.1e-06
    )  # KDE datasheet j_rotor_bell (the X500 motor's was estimated)


def test_swapped_quad_drc_has_no_errors(swapped):
    d, *_ = swapped
    errors = [f for f in d.check() if f.severity == "error"]
    assert errors == [], [(f.code, f.where, f.message[:100]) for f in errors]


@needs_kernel
def test_swapped_quad_hovers_and_numbers_are_reported(
    swapped, tmp_path_factory, capsys
):
    d, *_ = swapped
    ref, _ = build_scratch()
    a = ref.simulate("hover", out=tmp_path_factory.mktemp("before"))
    b = d.simulate("hover", out=tmp_path_factory.mktemp("after"))
    codes = lambda x: sorted({f.code for f in x.check() if f.severity != "info"})
    with capsys.disabled():
        print(
            f"\nX500 motors -> KDE 2315XF-965 (+ auto refresh): "
            f"hover current {a['hover_current_A']:.3f} -> {b['hover_current_A']:.3f} A, "
            f"throttle {a['hover_throttle']:.3f} -> {b['hover_throttle']:.3f}, T/W {a['thrust_to_weight']:.2f} -> {b['thrust_to_weight']:.2f}; "
            f"DRC {codes(ref)} -> {codes(d)}"
        )
    assert a["pass"] and b["pass"], (a, b)
    assert (
        b["hover_current_A"] != a["hover_current_A"]
    )  # the new motor's numbers are in the loop, not the old ones


# ---- D5: interchangeable parts swap without patching ----------------------------------------------------------------------
@pytest.fixture(scope="module")
def x500_kde():
    """The stock X500 with its four motors swapped for the KDE 2315XF-965: unpatched library, rewire + auto devices in the same edits
    (the thermal pair is the X500 motor's, FILL: the KDE is also a 64 g motor and publishes no c_th / r_th)."""
    d = Design.open(ASMS / "x500v2.toml")
    r = d.swap(
        [f"mot{k}" for k in ARM],
        "kde-2315xf-965",
        auto=True,
        overwrite=True,
        fill=FILL,
        rewire=True,
    )
    return d, r


def test_x500_to_kde_2315_remates_and_rewires(x500_kde):
    d, r = x500_kde
    for k, ch in zip(ARM, r.changes):
        kept = {m["mate"]: m for m in ch["mates"]["kept"]}
        assert ch["mates"]["lost"] == [] and set(kept) == {f"motor{k}", f"spin{k}"}
        assert (
            kept[f"motor{k}"]["new_port"] == "base_16x19_m3"
        )  # not a name match: the 16x19 pattern
        assert (
            kept[f"spin{k}"]["new_port"] == "prop_adapter"
        )  # M6 adapter, not the 4 mm shaft
        assert [w["wire"] for w in ch["wires"]["rewired"]] == [f"esc{k}-to-mot{k}"]
        assert "need_length" not in ch["wires"] and ch["dangling"] == []
    m1 = next(m for m in d.doc["mates"] if m["name"] == "motor1")
    assert (
        m1["a"] == "con1.motor_16x19" and m1["b"] == "mot1.base_16x19_m3"
    )  # the connector's co-located 16x19 port
    w = next(w for w in d.doc["wires"] if w["name"] == "esc1-to-mot1")
    assert w["included_in"] == "mot1" and isinstance(
        w["length"], (int, float, dict)
    )  # the lead's mass is the motor's


def test_x500_to_kde_2315_without_rewire_names_the_wires():
    d = Design.open(ASMS / "x500v2.toml")
    ch = d.swap("mot1", "kde-2315xf-965").changes[0]
    assert [x["wire"] for x in ch["wires"]["need_length"]] == ["esc1-to-mot1"]
    assert ch["wires"]["need_length"][0]["missing_param"] == "cable_length"
    assert (
        ch["autowire_offer"]["rewire"] is True
        and "esc1-to-mot1" in ch["autowire_offer"]["note"]
    )


def test_x500_kde_2315_drc_has_no_errors(x500_kde):
    d, _ = x500_kde
    errors = [f for f in d.check() if f.severity == "error"]
    assert errors == [], [(f.code, f.where, f.message[:100]) for f in errors]


def test_swap_back_to_the_stock_motor_remates():
    d = Design.open(ASMS / "x500v2.toml")
    d.swap("mot1", "kde-2315xf-965", rewire=True)
    ch = d.swap("mot1", "holybro-2216-kv920").changes[0]
    assert ch["mates"]["lost"] == []
    kept = {m["mate"]: m["new_port"] for m in ch["mates"]["kept"]}
    assert kept == {"motor1": "base", "spin1": "prop_adapter"}
    m1 = next(m for m in d.doc["mates"] if m["name"] == "motor1")
    assert m1["a"] == "con1.motor"  # back on the 16x16 seat


@pytest.mark.parametrize(
    "motor, pattern",
    [
        ("kde-5215xf-330", "bolt-circle.30.4.m4"),  # 30 mm circle, M4
        ("tmotor-4214-kv380", "bolt-circle.30.4.m4"),
        (
            "tmotor-mn3510-kv700",
            "bolt-circle.25.4.m3",
        ),  # 25 mm: not an X500 seat either
    ],
)
def test_swap_to_another_mount_pattern_is_refused_with_the_reason(motor, pattern):
    d = Design.open(ASMS / "x500v2.toml")
    before = copy.deepcopy(d.doc)
    with pytest.raises(
        DesignError,
        match=r"refused.*cannot be mounted on con1\.motor.*bolt-pattern\.16x16\.m3",
    ) as e:
        d.swap("mot1", motor)
    assert pattern in str(e.value)
    assert d.doc == before  # nothing changed


def test_swap_to_a_motor_with_16x16_pattern_still_seats():
    d = Design.open(ASMS / "x500v2.toml")
    ch = d.swap("mot1", "iflight-xing2-2207-1855kv").changes[
        0
    ]  # 16x16 M3 (shaft 5 mm: the prop mate is the reported loss)
    assert [m["mate"] for m in ch["mates"]["kept"]] == ["motor1"]


def test_device_unset_fires_on_a_stripped_thermal_field():
    d = Design.open(ASMS / "x500v2.toml")
    assert [
        f for f in d.check() if f.code == "DEVICE_UNSET"
    ] == []  # the stock design is complete
    esc1 = next(b for b in d.doc["devices"] if b["name"] == "esc1")
    del esc1["set"]["motor.c_th"]
    fs = [f for f in d.check() if f.code == "DEVICE_UNSET"]
    assert [(f.severity, f.where) for f in fs] == [("error", ["esc1", "motor.c_th"])]
    assert (
        "provenance table" in fs[0].fix
        and "'esc1'" in fs[0].fix
        and "simulation run would fail" in fs[0].message
    )


def test_device_unset_after_a_swap_without_the_thermal_fill():
    d = Design.open(ASMS / "x500v2.toml")
    d.swap("mot1", "kde-2315xf-965", auto=True, overwrite=True, rewire=True)
    fs = [(f.where[0], f.where[1]) for f in d.check() if f.code == "DEVICE_UNSET"]
    assert fs == [
        ("esc1", "motor.c_th"),
        ("esc1", "motor.r_th"),
    ]  # reported before any run, with the fix


@needs_kernel
def test_x500_with_kde_2315_hovers(x500_kde, tmp_path_factory, capsys):
    d, _ = x500_kde
    ref = Design.open(ASMS / "x500v2.toml")
    a = ref.simulate("hover", out=tmp_path_factory.mktemp("stock"))
    b = d.simulate("hover", out=tmp_path_factory.mktemp("kde"))
    with capsys.disabled():
        print(
            f"\nX500 -> KDE 2315XF-965 (unpatched swap): hover current {a['hover_current_A']:.3f} -> {b['hover_current_A']:.3f} A, "
            f"throttle {a['hover_throttle']:.3f} -> {b['hover_throttle']:.3f}, T/W {a['thrust_to_weight']:.2f} -> {b['thrust_to_weight']:.2f}"
        )
    assert a["pass"] and b["pass"], (a, b)
    assert b["hover_current_A"] != a["hover_current_A"]


# ---- MCP -----------------------------------------------------------------------------------------------------------------
def test_mcp_auto_devices_op_and_swap_flags(root):
    async def agent():
        async with Client(NaplServer(root).server, elicitation_callback=accept) as c:
            await ok(c, "design_open", path="assemblies/so101.toml")
            r = await ok(c, "design_edit", ops=[{"op": "auto_devices"}])
            assert r["changes"][0]["summary"]["kept"] == 8  # hand-written bindings win
            r = await ok(
                c,
                "design_edit",
                ops=[
                    {
                        "op": "auto_devices",
                        "overwrite": ["servo1", "servo2"],
                        "fill": {},
                    }
                ],
            )
            assert (
                r["changes"][0]["summary"]["refreshed"]
                + r["changes"][0]["summary"]["created"]
                == 2
            )
            assert [u["device"] for u in r["changes"][0]["unset"]] == [
                "servo1",
                "servo2",
            ]
            await ok(c, "design_open", path="assemblies/x500v2.toml")
            r = await ok(
                c,
                "design_edit",
                ops=[
                    {
                        "op": "swap",
                        "id": "mot1",
                        "part": "holybro-2216-kv920",
                        "auto": True,
                    }
                ],
            )
            assert r["changes"][0]["stale_devices"][0]["device"] == "esc1"

    asyncio.run(agent())
