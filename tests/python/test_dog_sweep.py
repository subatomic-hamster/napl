"""Speed sweep harness (tools/dog_sweep.py): variant files load for every default point; the report labels every column and claims no
gait speed it did not measure (Review Focus 5); two runs give the same numbers."""

import json
import subprocess
import sys
from pathlib import Path

import napl
import pytest
from napl.design import assembly as A

REPO = Path(__file__).resolve().parents[2]
TOOL = REPO / "tools" / "dog_sweep.py"
sys.path.insert(0, str(REPO / "tools"))
import dog_sweep as S

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.mark.parametrize("point", S.DEFAULT_POINTS)
def test_variant_files_load(tmp_path, point):
    act, ratio, L = point
    asm = A.load(S.write_variant(tmp_path, act, ratio, L))
    by = {i["id"]: i for i in asm.doc["instances"]}
    assert by["fl:knee_motor"]["params"]["drive_ratio"]["value"] == pytest.approx(
        8 * ratio
    )
    k = S.KSFX.get(ratio, "") if L == 0.22 else ""
    assert by["rr:thigh"]["part"] == "dog-thigh-bracket-l" + S.LSFX[L] + k
    probe = [d for d in asm.doc["devices"] if d["type"] == "qdd" and "map" not in d]
    assert bool(probe) == (
        act == "gim6010"
    )  # the probe carries the GIM6010-8 electrics as provenance-tagged `set`


@needs_kernel
def test_report_labels_every_column_and_is_deterministic(tmp_path):  # Review Focus 5
    outs = []
    for k in range(2):
        out = tmp_path / f"s{k}"
        r = subprocess.run([sys.executable, str(TOOL), "--quick", "--points", "gim8108:1.5:0.22", "--out", str(out), "--jobs", "1"],
                           capture_output=True, text=True, timeout=3000)  # fmt: skip
        assert r.returncode == 0, r.stderr[-3000:]
        outs.append(json.loads((out / "speed-sweep.json").read_text()))
    data = outs[0]
    assert set(data["columns"]) == set(S.COLUMNS) and set(S.COLUMNS.values()) <= {
        "measured",
        "derived",
        "bom",
        "sp4",
    }
    assert S.COLUMNS["v_top_trot"] == "sp4" and all(
        p["v_top_trot"] is None for p in data["points"]
    )
    assert all(
        S.COLUMNS[c] != "measured" for c in ("v_swing", "v_stance", "v_trot_bound")
    )
    md = (tmp_path / "s0" / "speed-sweep.md").read_text()
    assert (
        "not measured (SP4)" in md
        and "derived (study formulas, sim-measured inputs)" in md
    )
    assert outs[0]["points"] == outs[1]["points"]


# ---- leg study variants (docs/dog/leg-study.md): per-seat actuators, knee convention, oracle, BOM ----
def _by(asm, key):
    return {
        x["name" if key == "devices" else "id"]: x
        for x in asm.raw[key]
        if x.get("type", "qdd") == "qdd"
    }


def _val(x):
    return x["value"] if isinstance(x, dict) else x


def test_probe_actuator_carries_electrics_mass_and_the_scaled_knee_ranges(tmp_path):
    asm = A.load(S.write_variant(tmp_path, "ak70", 1.5, 0.22))
    leg = A.load(tmp_path / "leg.toml")
    dev, inst = _by(leg, "devices"), _by(leg, "instances")
    assert all("map" not in d for d in dev.values())
    assert (
        _val(dev["hip"]["set"]["i_limit"]) == pytest.approx(23.8)
        and _val(dev["hip"]["set"]["gear_ratio"]) == 9.0
    )
    assert _val(dev["knee"]["set"]["tau_ff_max"]) == pytest.approx(
        29.2 * 2.0
    )  # leg.toml's convention: the largest swept ratio
    assert _val(dev["knee"]["set"]["kp_max"]) == pytest.approx(500 * 4)
    assert all(
        _val(inst[f"{c}_motor"]["params"]["mass"]) == pytest.approx(0.54)
        for c in S.CLASSES
    )
    by = {i["id"]: i for i in asm.doc["instances"]}
    assert by["fl:knee_motor"]["params"]["drive_ratio"]["value"] == pytest.approx(
        9 * 1.5
    )


def test_mixed_seats_keep_the_baseline_map_on_the_baseline_joint(tmp_path):
    S.write_variant(tmp_path, "gim8108+ak70+ak70", 1.5, 0.22)
    leg = A.load(tmp_path / "leg.toml")
    dev = _by(leg, "devices")
    assert "map" in dev["abad"] and "map" not in dev["hip"] and "map" not in dev["knee"]
    assert "mass" not in (_by(leg, "instances")["abad_motor"].get("params") or {})


def test_exact_knee_range_is_the_real_joint_limit_and_the_baseline_stays_untouched(
    tmp_path,
):
    S.write_variant(tmp_path / "x", "gim8108", 1.5, 0.22, ktau="exact")
    knee = _by(A.load(tmp_path / "x" / "leg.toml"), "devices")["knee"]
    assert "map" in knee and _val(knee["set"]["tau_ff_max"]) == pytest.approx(
        22.0 * 1.5
    )  # 33 N m, not leg.toml's 44
    S.write_variant(tmp_path / "b", "gim8108", 1.5, 0.22)
    assert (tmp_path / "b" / "leg.toml").read_text() == (
        S.DOG / "leg.toml"
    ).read_text()  # the default point is the shipped leg, byte for byte


def test_oracle_scales_torque_and_speed_at_the_same_mass_and_price(tmp_path):
    S.write_variant(tmp_path, "orcT2.5S2", 1.5, 0.22)
    leg = A.load(tmp_path / "leg.toml")
    hip = _by(leg, "devices")["hip"]["set"]
    assert _val(hip["motor.kv"]) == pytest.approx(53.36 * 2) and _val(
        hip["i_limit"]
    ) == pytest.approx(22.0 * 2.5 * 2)
    kt, n, eta = (
        60 / (2 * 3.14159265 * _val(hip["motor.kv"])),
        _val(hip["gear_ratio"]),
        _val(hip["efficiency"]),
    )
    assert kt * n * eta * _val(hip["i_limit"]) == pytest.approx(
        22.0 * 2.5, rel=1e-3
    )  # stall torque x T
    assert "mass" not in (_by(leg, "instances")["hip_motor"].get("params") or {})
    rep = {"bom": {"total": {"cost": 5554.0}}}
    assert (
        S.variant_cost(rep, "orcT2.5S2") == 5554.0
        and S.variant_cost(rep, "gim8108") == 5554.0
    )


def test_variant_cost_swaps_only_the_probe_seats():
    rep = {"bom": {"total": {"cost": 5554.0}}}
    lib = S.Library(S.REPO / "parts")
    d = lambda k: (
        lib.get(S.ACTS[k]).value("price") - lib.get(S.ACTS["gim8108"]).value("price")
    )
    assert S.variant_cost(rep, "ak70") == pytest.approx(5554 + 12 * d("ak70"))
    assert S.variant_cost(rep, "gim8108+ak10+ak10") == pytest.approx(
        5554 + 8 * d("ak10")
    )


@pytest.mark.parametrize(
    "ratio,sfx,rng", [(1.0, "-k10", (-1.995, 0.005)), (2.0, "-k20", (-1.02, 0.0))]
)
def test_real_geometry_ratio_parts_load_with_their_own_knee_range(
    tmp_path, ratio, sfx, rng
):
    asm = A.load(S.write_variant(tmp_path, "gim8108", ratio, 0.22))
    by = {i["id"]: i for i in asm.doc["instances"]}
    assert by["fl:thigh"]["part"] == "dog-thigh-bracket-l" + sfx
    assert by["rr:shank"]["part"] == "dog-shank" + sfx
    assert by["fl:knee_rod"]["part"] == "dog-knee-rod-220" + sfx
    part = S.Library(REPO / "parts").get("dog-thigh-bracket-l" + sfx)
    assert tuple(part.params["knee_range"]["value"]) == rng


def test_point_strings_round_trip():
    assert S.parse_point("gim8108+ak70+ak70:2.0:0.25:exact") == (
        "gim8108+ak70+ak70",
        2.0,
        0.25,
        "exact",
    )
    assert S.point_name(S.parse_point("ak70:1.5:0.22")) == "ak70-r1.5-L220"
    assert S.point_name(S.parse_point("ak70:1.5:0.22:exact")) == "ak70-r1.5-L220-exact"
