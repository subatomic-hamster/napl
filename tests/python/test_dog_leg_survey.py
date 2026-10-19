"""The bio-inspired '-al' dog leg (docs/dog/leg-survey.md): the variant files differ from the default only where intended, the variant builds DRC-clean,
and the hand-calc (tools/dog_leg_struct.py) keeps the printed leg failing and the '-al' leg at safety factor >= 2 on every load case."""

import pathlib
import sys

import pytest
from napl import design
from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, DOG = ROOT / "parts", ROOT / "assemblies" / "dog"
sys.path.insert(0, str(ROOT / "tools"))
import dog_leg_struct as S


def body(name, first):
    """File lines from the original's first line on (the variant adds header comments before it)."""
    lines = (DOG / name).read_text().splitlines()
    return lines[next(i for i, x in enumerate(lines) if x.startswith(first)) :]


def test_variant_files_differ_only_in_the_leg_parts():
    a, b = body("dog.toml", "# The recon"), body("dog-al.toml", "# The recon")
    assert [y for x, y in zip(a, b) if x != y] == ['file = "leg-al.toml"'] * 4
    a, b = body("leg.toml", "# One leg"), body("leg-al.toml", "# One leg")
    d = [(x, y) for x, y in zip(a, b) if x != y]
    assert {y for _, y in d if y.startswith("part =")} == {
        'part = "dog-thigh-al-l"',
        'part = "dog-shank-cf-l"',
        'part = "dog-foot-core-cf"',
    }
    assert len(d) == 4  # three parts + the description line


def test_al_leg_keeps_the_kinematics():
    lib = Library(PARTS)
    for pair in (
        ("dog-thigh-bracket-l", "dog-thigh-al-l"),
        ("dog-thigh-bracket-r", "dog-thigh-al-r"),
    ):
        old, new = (lib.get(p) for p in pair)
        assert new.value("length") == old.value("length")
        assert new.params["knee_range"]["value"] == old.params["knee_range"]["value"]
        assert [p for p in new.ports if p["name"] == "rod"][0]["pos"][1:] == [
            p for p in old.ports if p["name"] == "rod"
        ][0]["pos"][1:]
    assert lib.get("dog-shank-cf-l").value("lug_radius") == lib.get("dog-shank").value(
        "lug_radius"
    )


def test_al_parts_carry_material_colours_and_the_shell_knee_port():
    """Rev 2: parts draw in their real materials (plates / head aluminium, bands / foot core dark nylon, tube carbon), not the PLA default orange; the thigh is a
    36 mm deep-shell whose knee axis sits on the shank tube centre line (y = 18 mm)."""
    lib = Library(PARTS)
    want = {
        "dog-thigh-al-l": {"alu-brushed", "plastic-dark"},
        "dog-thigh-al-r": {"alu-brushed", "plastic-dark"},
        "dog-shank-cf-l": {"alu-brushed", "carbon-fibre"},
        "dog-shank-cf-r": {"alu-brushed", "carbon-fibre"},
        "dog-foot-core-cf": {"plastic-dark"},
    }
    for pid, cols in want.items():
        els = lib.get(pid).raw["visual"]["elements"]
        assert {e["colors"]["body"] for e in els} == cols
        assert all((ROOT / e["mesh"]).is_file() for e in els)
    for pid, sign in (("dog-thigh-al-l", 1), ("dog-thigh-al-r", -1)):
        knee = [p for p in lib.get(pid).ports if p["name"] == "knee"][0]
        assert knee["pos"][1] == pytest.approx(sign * 0.018)
        bb = lib.get(pid).raw["geometry"]
        assert (
            bb["bbox_max"][0] - bb["bbox_min"][0] >= 0.07
        )  # deep shell: 60-78 mm fore-aft against the printed thigh's 24 mm


def test_al_dog_drc_clean_and_builds(tmp_path):
    assert [
        f
        for f in check(A.load(DOG / "dog-al.toml"), Library(PARTS))
        if f.severity == "error"
    ] == []
    rep = design.build(DOG / "dog-al.toml", PARTS, tmp_path)["report"]
    assert 10.0 <= rep["mass"]["total"] <= 13.0
    assert rep["bom"]["missing"] == []


@pytest.mark.parametrize("case", list(S.CASES))
def test_hand_calc_printed_fails_al_survives(case):
    rows = {(r["leg"], r["case"]): r for r in S.run() if "Fb" not in r}
    assert rows[("printed PLA leg", case)]["vt"][3] < 1.0
    al = rows[("'-al' leg", case)]
    assert min(al["vt"][3], al["vs"][3]) >= 2.0


def test_hand_calc_knee_pin():
    pin = {r["leg"]: r for r in S.run() if "Fb" in r}
    assert pin["printed PLA leg"]["bear"] > pin["printed PLA leg"]["bear_allow"]
    al = pin["'-al' leg"]
    assert al["bear"] < al["bear_allow"] / 2 and al["sig_pin"] < al["pin_yield"] / 2
