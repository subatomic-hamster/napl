"""assemblies/aos5.toml: iFlight AOS 5 EVO 5-inch quad built through the editing API (A2): builds, DRC clean, hover, mass."""

import math
from pathlib import Path

import napl
import pytest
from napl import design
from napl.design import Design, Library

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
ASM = REPO / "assemblies" / "aos5.toml"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    return design.build(ASM, PARTS, tmp_path_factory.mktemp("aos5"))["report"]


def test_builds_and_drc_has_no_errors():
    d = Design.open(ASM)
    errors = [f for f in d.check() if f.severity == "error"]
    assert errors == [], [(f.code, f.message) for f in errors]
    # the only warnings: peak current above the continuous wire rating (hover sustained is far below it)
    assert {f.code for f in d.check() if f.severity == "warning"} <= {
        "CURRENT_PEAK_OVER_RATING"
    }


def test_mass_matches_sum_of_part_masses_within_tolerance(built):
    """Total = sum of the library's own part masses (datasheet or estimated) + fasteners + wires, to the stated tolerances."""
    lib = Library(PARTS)
    doc = design.load(ASM).doc
    part_sum = sum(
        lib.get(i["part"]).value("mass") or 0.0
        for i in doc["instances"]
        if not i.get("offboard")
    )
    m = built["mass"]
    extra = sum(f["mass"] for f in built["fasteners"]) + sum(
        (w["mass"] or 0.0) for w in built["wires"] if not w["included_in"]
    )  # leads inside a part's datasheet mass are not added again
    assert math.isclose(m["total"], part_sum + extra, rel_tol=1e-9)
    tol = sum(r["mass_tol"] for r in built["instances"]) + sum(
        f["mass_tol"] for f in built["fasteners"]
    )
    assert abs(m["total"] - (part_sum + extra)) <= tol
    assert (
        0.45 < m["total"] < 0.55
    )  # kg: no camera / VTX / receiver, see the assembly header


def test_frame_group_gap_to_published_kit_is_reported_not_hidden(built):
    """iFlight: 'Approx. 120g with all hardware'. The model has plates + arms + stack screws only (no upper plate, TPU, shield
    tubes, screw bag, straps: no masses published, no seats): the gap is large and negative, and the target says so."""
    (t,) = built["targets"]
    assert t["target"] == 0.120 and t["scope"] == ["frame"]
    assert -0.45 < t["gap_pct"] / 100 < -0.2, t["gap_pct"]


@needs_kernel
def test_hover_passes_and_thrust_to_weight_from_table(tmp_path):
    r = Design.open(ASM).simulate("hover", out=tmp_path)
    assert r["pass"], r
    assert r["thrust_to_weight"] > 1.05
    # the vendor table at 24 V, 100 % throttle: 4 x 16.524 N against this weight (the sim number is pack-sag limited)
    lib = Library(PARTS)
    t = lib.get("iflight-xing2-2207-1855kv").raw["tables"]["thrust_test"]
    n = 4 * t["rows"][-1][t["columns"].index("thrust_N")]
    assert n / r["weight_N"] > 8
