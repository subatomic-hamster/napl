"""Fidelity report (napl.fidelity): counts match tools/check_parts.py, every assembly has a section, every estimated value
of a build report is listed, determinism test names exist."""

import re
import subprocess
import sys
from pathlib import Path

import pytest
from napl import fidelity as fid

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def data(tmp_path_factory):
    return fid.generate(out_dir=tmp_path_factory.mktemp("fid"), servo=True)


@pytest.fixture(scope="module")
def md(data):
    return fid.to_markdown(data)


def test_library_counts_match_check_parts(data):
    out = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "check_parts.py"), str(ROOT / "parts")],
        capture_output=True,
        text=True,
    ).stdout.splitlines()[-1]
    m = re.match(r"(\d+) parts, (\d+) values \((.*)\), 0 problem", out)
    assert m, out
    L = data["library"]
    assert (L["parts"], L["values"]) == (int(m[1]), int(m[2]))
    want = {k: int(v) for k, v in (x.split("=") for x in m[3].split(", "))}
    assert {k: v for k, v in L["by_fidelity"].items() if v} == want
    assert sum(r["parts"] for r in L["by_dir"].values()) == L["parts"]
    assert sum(r["values"] for r in L["by_category"].values()) == L["values"]


def test_every_assembly_has_a_section_and_consistent_mass(data, md):
    files = sorted(p.name for p in (ROOT / "assemblies").glob("*.toml"))
    assert files and [a["file"] for a in data["assemblies"]] == files
    for a in data["assemblies"]:
        assert f"`{a['file']}`" in md
        assert a["mass_items_total"] == pytest.approx(a["mass_total"], rel=1e-9)
        assert sum(a["groups"]["(all)"][k] for k in fid.FIDS) == pytest.approx(1.0)


def test_every_estimated_value_is_listed(data, md, tmp_path):
    from napl.design import build

    for a in data["assemblies"]:
        rep = build(ROOT / "assemblies" / a["file"], ROOT / "parts", tmp_path)["report"]
        assert rep["estimated"] and len(a["estimated"]) == len(rep["estimated"])
        for e in rep["estimated"]:
            assert f"| {e['where']} | {e['key']} |" in md, e


def test_models_validation_and_determinism_sections(data, md):
    assert [m["name"] for m in data["models"]] == [m[0] for m in fid.MODELS]
    assert all(
        m["rows"] for m in data["models"]
    )  # every model resolved to docs/models.md rows
    s = data["validation"]["stand"]
    assert len(s["rows"]) >= 17 and 0 < s["mean_thrust_mape"] < 15
    assert any("PX4" in k for k in data["validation"]["hover"]["rows"][0])
    for t, f, _ in fid.DETERMINISM:
        src = (ROOT / f).read_text()
        name = t.split("::")[-1].split(".")[-1]
        assert re.search(rf"\b{name}\b", src), t
    assert (
        "## 4. Determinism" in md
        and "STS3215" in md
        or not data["validation"]["servo"]["available"]
    )


def test_deterministic(data, tmp_path):
    again = fid.generate(out_dir=tmp_path, servo=False)
    data = dict(
        data, validation=dict(data["validation"], servo=again["validation"]["servo"])
    )
    assert fid.to_markdown(data) == fid.to_markdown(again)
