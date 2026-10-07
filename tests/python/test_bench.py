"""N4 thrust-stand bench: exit check (2216 @ 16 V held-out thrust and current within 10 %) and the whole-suite run.

Regression thresholds are the held-out MAPE (percent) of the committed method, rounded up with ~25 % headroom: the bench is
deterministic, so the headroom only absorbs fit-subset / solver-tolerance changes. They are NOT accuracy claims; the likely
cause of each large one is in LIKELY_CAUSE (and in docs/validation/thrust-stands.md, which lists every held-out row).
"""

import pytest
from napl import bench

# (motor, table) -> (thrust MAPE %, current MAPE %) ceilings
# N4b (Reynolds factor + thermal steady state): re-measured, ceil(1.25 x MAPE); the N4 limits are in the comments. Tighter
# everywhere except tmotor-mn4110 current (N4 limit 2, N4 MAPE 0.9, now 2.2: the hot-winding model raised it; thrust
# improved 1.8 -> 1.5): that limit is LOOSENED to 3 on purpose, the other two stayed equal (1855 5140, mn3508).
LIMITS = {
    ("holybro-2216-kv920", "thrust_test"): (7, 3),  # N4 (8, 4)
    ("iflight-xing2-2207-1855kv", "thrust_test"): (12, 5),  # (13, 6)
    ("iflight-xing2-2207-1855kv", "thrust_test_5140"): (16, 8),  # (16, 8)
    ("iflight-xing2-2207-2755kv", "thrust_test"): (7, 9),  # (7, 10)
    ("iflight-xing2-2207-2755kv", "thrust_test_5140"): (16, 12),  # (17, 13)
    ("tmotor-2807-kv1250", "thrust_test"): (10, 10),  # (13, 13)
    ("tmotor-2814-kv900", "thrust_test"): (8, 10),  # (11, 12)
    ("tmotor-3110-kv1580", "thrust_test"): (13, 9),  # (17, 11)
    ("tmotor-4214-kv380", "thrust_test"): (5, 8),  # (9, 10)
    ("tmotor-air2213-kv1000", "thrust_test"): (4, 3),  # (6, 4)
    ("tmotor-mn2806-kv400", "thrust_test"): (4, 3),  # (5, 4)
    ("tmotor-mn3508-kv700", "thrust_test"): (13, 14),  # (13, 14)
    ("tmotor-mn4004-kv300", "thrust_test"): (5, 7),  # (5, 8)
    ("tmotor-mn4006-kv380", "thrust_test"): (4, 5),  # (5, 6)
    ("tmotor-mn4110-kv340", "thrust_test"): (
        2,
        3,
    ),  # (3, 2): current loosened, see above
    ("tmotor-mn5006-kv300", "thrust_test"): (4, 4),  # (4, 5)
    ("tmotor-mn5008-kv340", "thrust_test"): (6, 8),  # (7, 9)
}
# Motors that miss the +-10 % per-row bar on some held-out row, with the likely cause (not hidden: see the docs table).
LIKELY_CAUSE = {
    "xing2 2207 (4 tables)": "6-row tables, 3 held-out, no rpm: C_T0/duty slope not separable (prior used); 90 % row extrapolates past the 80 % fit row; "
    "static prop coefficients for 5140 are guessed from a label",
    "tmotor C-series 2807/2814/3110": "vendor list shaft power not V*I; low-throttle current is under-predicted and a "
    "speed-dependent I0 loss makes it worse (train rows reject it, docs/validation/thrust-stands.md); "
    "fitted C_T0 ~0.17 is high for a 7.5 in 3-blade prop (prop label/diameter or unrecorded sag)",
    "tmotor-mn3508": "only 5 rows (2 held-out); 12x4 prop coefficients drift with rpm",
}


@pytest.fixture(scope="module")
def suite():
    return {m: bench.bench_motor(m) for m in bench.tabled_motors()}


def test_2216_holdout_within_10_percent(suite):
    (b,) = suite["holybro-2216-kv920"]["tables"]
    held = [r for r in b["rows"] if r["split"] == "held_out"]
    assert len(held) >= 10 and set(b["fitted"]["rows"]).isdisjoint(
        r["idx"] for r in held
    )
    assert all(abs(r["voltage"] - 16.0) < 1e-9 for r in b["rows"])
    for r in held:
        assert abs(r["rel_err"]["thrust"]) <= 0.10, r
        assert abs(r["rel_err"]["current"]) <= 0.10, r


def test_pipeline_all_tabled_motors(suite):
    assert len(suite) >= 15
    seen = 0
    for m, res in suite.items():
        for b in res["tables"]:
            seen += 1
            assert not b["unconverged"], (m, b["unconverged"])
            # held-out data exists and fit never saw every row
            assert 0 < len(b["fitted"]["rows"]) < len(b["rows"])
            # the closed-form steady state used in the fit matches the simulated stand
            assert b["sim_vs_analytic_max_rel_diff"] < 1e-3, (m, b["table"])
            ho = b["summary"]["held_out"]
            lt, lc = LIMITS[(m, b["table"])]
            assert 100 * ho["thrust"]["mape"] <= lt, (m, b["table"], ho["thrust"])
            assert 100 * ho["current"]["mape"] <= lc, (m, b["table"], ho["current"])
    assert seen == len(LIMITS)


def test_reynolds_shape_from_uiuc_tables():
    s = bench.reynolds_shape()
    assert s["x"] == sorted(s["x"]) and len(s["x"]) >= 6
    assert min(s["n_props"]) >= 2
    k = s["x"].index(s["x_ref"])
    assert s["ft"][k] == pytest.approx(1.0) and s["fq"][k] == pytest.approx(1.0)
    assert (
        s["ft"][-1] > 1.02 > 0.98 > s["ft"][0]
    )  # C_T rises with n D^2 (UIUC static tables)
    assert all(
        0.9 < v < 1.1 for v in s["fq"][1:]
    )  # C_P is nearly flat above the smallest x


def test_baseline_model_is_the_n4_stand():
    """model="baseline" = constant coefficients, no Re table in the stand, isothermal 20 degC: the N4 numbers."""
    b = bench.bench("holybro-2216-kv920", sim_rows=False, model="baseline")
    h = b["summary"]["held_out"]
    assert round(100 * h["thrust"]["mape"], 1) == 6.0  # N4 doc: 6.0 / 2.0 / 1.8
    assert round(100 * h["current"]["mape"], 1) == 2.0
    case = bench.make_case("holybro-2216-kv920", model="baseline")
    assert bench.rotor_re(case) == {} and case["t_amb"] == 20.0


def test_hover_x500():
    h = bench.hover_bench(auw=1.65)
    s = h["start"]
    assert abs(s["thrust_N"] - 1.65 * 9.80665 / 4) < 0.02
    assert 0.1 < s["throttle_duty"] < 0.8
    assert 5 < h["endurance_min"] < 40, h[
        "endurance_min"
    ]  # sanity band, not a validation
    assert "terminal voltage" in h["cutoff"]
