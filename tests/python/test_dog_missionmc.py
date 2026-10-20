import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
sys.path.insert(0, str(ROOT / "tools"))

import missionmc as MC


def row(seed=1, **kw):
    r = dict(
        seed=seed,
        coverage=0.96,
        rms=0.03,
        phase=4,
        t=900.0,
        fell_ever=0,
        fallen=0,
        bumps=0,
        distance=800.0,
    )
    r.update(kw)
    return r


def test_summary_counts_every_run_and_uses_ever_fell():  # Review Focus 1
    rows = [
        row(1),
        row(2, fell_ever=1, fallen=0, phase=4),
        {"seed": 3, "error": "boom"},
        row(4, coverage=0.90),
        row(5, phase=3, t=1300.0),
    ]
    s = MC.summarize(rows, 1200.0)
    assert s["n"] == 5  # the crash is in the denominator
    assert (
        s["k_nofall"] == 3
    )  # seed 2 fell although its last row says standing; the crash is not counted as no-fall
    assert (
        s["k_success"] == 1
        and s["fails"]["fell"] == 1
        and s["fails"]["crashed"] == 1
        and s["fails"]["coverage"] == 1
    )
    assert s["fails"]["not_home"] == 1 and s["fails"]["late"] == 1
    lo, hi = s["wilson_nofall"]
    assert lo < 3 / 5 < hi
    with pytest.raises(ValueError):
        MC.summarize([], 1200.0)


def test_seed_blocks_are_disjoint_and_heldout_needs_final_rev():  # Review Focus 2
    tune, held = set(MC.seeds("tune", 30)), set(MC.seeds("held", 100))
    assert not tune & held
    assert not (
        (tune | held)
        & (
            set(range(1, 31))
            | set(range(1001, 1026))
            | set(range(21000, 21101))
            | set(range(1000, 1010))
            | {s for b in (31000, 41000, 51000, 61000) for s in range(b, b + 1000)}
        )
    )  # SP4's terrain MC / training blocks: 31000+, 41000+, 51000+, 61000+ (n = 40 each, 1000 reserved)
    with pytest.raises(ValueError, match="rev"):
        MC.check_heldout_rev([dict(rev="abc"), dict(rev="def")])
    MC.check_heldout_rev([dict(rev="abc"), dict(rev="abc")])


def test_results_table_marks_every_criterion():  # Review Focus 6
    by = {"rubble": MC.summarize([row(i) for i in range(36)], 1200.0)}
    md = MC.results_table(
        by, {"top_speed_flat": None, "rocky_speed": 1.2, "bom_usd": 5554.0}
    )
    for c in MC.CRITERIA:
        assert (
            c[1] in md
        )  # (the plan's `assert c in md` compares a tuple with a str: the criterion's text is what the table carries)
    assert (
        md.count("MET") >= 1 and "NOT MET" in md and "not measured" in md
    )  # rocky 1.2 < 1.5, top speed missing
    assert (
        "36/36" in md and "0.90" in md
    )  # n and the Wilson bound are printed next to the rate


def test_nofall_row_states_a_lower_bound_under_the_target():
    by = {
        "rubble": MC.summarize([row(i) for i in range(12)], 1200.0)
    }  # 12/12: rate 1.0 meets 0.90, the bound (0.76) does not
    line = next(
        ln for ln in MC.results_table(by, {}).splitlines() if ln.startswith("| >= 90 %")
    )
    assert "MET on rate, lower bound under 0.90 for rubble" in line and "0.76" in line
    bad = {
        "scree": MC.summarize([row(i, fell_ever=int(i < 3)) for i in range(10)], 1200.0)
    }  # 7/10 = 0.70
    assert "NOT MET" in next(
        ln
        for ln in MC.results_table(bad, {}).splitlines()
        if ln.startswith("| >= 90 %")
    )


def test_flat_top_speed_target_is_the_lowered_2_0():
    c = next(c for c in MC.CRITERIA if c[0] == "top_speed_flat")
    assert c[3] == 2.0 and "2.0 m/s" in c[1]
    assert "| flat top speed >= 2.0 m/s | 2.1 | MET |" in MC.results_table(
        {}, {"top_speed_flat": 2.1}
    )
    assert "| flat top speed >= 2.0 m/s | 1.9 | NOT MET |" in MC.results_table(
        {}, {"top_speed_flat": 1.9}
    )


def test_unscored_coverage_and_aborts_do_not_break_the_summary():
    nan = float("nan")
    rows = [
        row(1, coverage=nan, phase=5, fell_ever=1),
        row(2, coverage=nan, rms=nan, phase=5, fell_ever=0),
    ]
    s = MC.summarize(rows, 420.0)
    assert (
        s["aborted"] == 2 and s["k_nofall"] == 1 and s["fails"]["coverage"] == 2
    )  # an abort without a ground-truth fall is no-fall, still not a success
    assert s["abort_nofall"] == 1 and "| 2 (1) |" in MC.results_table({"x": s}, {})
    assert (
        s["medians"]["coverage"] != s["medians"]["coverage"] and s["covs"] == []
    )  # all nan -> nan median, no crash


def test_family_scenarios_exist_and_n_spec_parses():
    for fam in MC.FAMILIES:
        assert MC.scenario_path(fam).exists()
    assert MC.parse_n("5", ["rubble", "ruins"]) == {"rubble": 5, "ruins": 5}
    assert MC.parse_n("rubble=20,ruins=34", ["ruins"]) == {"ruins": 34}
    with pytest.raises(SystemExit):
        MC.parse_n("rubble=20", ["ruins"])


def test_heldout_refuses_without_rev_and_does_not_rerun(tmp_path):
    with pytest.raises(SystemExit, match="--rev"):
        MC.main(
            [
                "--family",
                "rubble",
                "--block",
                "held",
                "--n",
                "1",
                "--out",
                str(tmp_path),
            ]
        )
    (tmp_path / "rubble.jsonl").write_text("{}\n")
    with pytest.raises(SystemExit, match="once"):
        MC.main(
            [
                "--family",
                "rubble",
                "--block",
                "held",
                "--n",
                "1",
                "--rev",
                "abc",
                "--out",
                str(tmp_path),
            ]
        )


def test_readme_numbers_match_the_results_json():
    import json
    import re

    readme = (ROOT / "docs" / "dog" / "README.md").read_text()
    data = json.loads((ROOT / "docs" / "dog" / "missions.json").read_text())
    for fam, s in data["families"].items():
        assert f"{s['k_nofall']}/{s['n']}" in readme, fam  # the rate as k/n
        assert f"{s['wilson_nofall'][0]:.2f}" in readme, fam  # and its lower bound
        assert f"{s['k_success']}/{s['n']}" in readme, (
            fam
        )  # mission success is its own number
    for c in MC.CRITERIA:
        # (the plan compares c, a tuple, with a line; c[1] is the criterion's text)
        line = next(ln for ln in readme.splitlines() if c[1].split(" (")[0] in ln)
        assert re.search(r"\b(NOT MET|MET|not measured)\b", line), c[0]
    assert "tuning" in readme.lower() and "held-out" in readme.lower()
