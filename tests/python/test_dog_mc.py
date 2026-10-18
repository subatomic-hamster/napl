"""SP4 Task 11: terrain-family Monte Carlo. The summary counts falls from ground truth, gives Wilson 95 % intervals and refuses runs with the lidar
on (Review Focus 5); per-family pass-rate floors on held-out seeds (small n: a regression catch, not the claim; docs/dog/locomotion.md has the MC)."""

import sys
from pathlib import Path

import napl
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogmc as MC

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def row(**kw):
    r = dict(seed=1, fam="rubble", reached=True, fallen=False, ctl_fallen=False, fault=False, diverged=False, mean_speed=1.6, dist=17.0,
             n_slip=0, mpc_fail=0, sat_hip=0.1, rangefinders=0)  # fmt: skip
    r.update(kw)
    return r


def test_summary_uses_truth_falls_wilson_and_refuses_lidar():  # Review Focus 5
    rows = [
        row(),
        row(),
        row(fallen=True, reached=False),
        row(reached=False),
        row(ctl_fallen=True),
    ]
    s = MC.summarize(rows)
    assert (
        s["n"] == 5 and s["k_nofall"] == 4
    )  # the controller's own detector (last row) is not a fall; truth is
    assert s["k_success"] == 3 and s["failures"] == {"fall": 1, "timeout": 1}
    lo, hi = s["wilson_nofall"]
    assert lo < 4 / 5 < hi and s["wilson_success"][0] < 3 / 5
    assert s["speed_mean"] == pytest.approx(1.6)  # successful runs only
    with pytest.raises(ValueError, match="lidar"):
        MC.summarize([row(rangefinders=1)])
    with pytest.raises(ValueError):
        MC.summarize([])
    assert (
        min(MC.TEST_BASE, *(MC.TEST_BASE + i for i in range(100)))
        > MC.TRAIN_BASE + 10000
    )  # held-out seed blocks never overlap


def test_a_field_without_a_route_is_reported_not_counted():
    s = MC.summarize(
        [
            row(),
            dict(seed=2, fam="ruins", leg_ctl="wbc", skipped="no route"),
            row(fallen=True, reached=False),
        ]
    )
    assert s["n"] == 2 and s["n_no_route"] == 1 and s["k_nofall"] == 1


@needs_kernel
def test_one_run_returns_a_truth_row_without_the_lidar():
    r = MC.one(
        (MC.TEST_BASE, "scree", {"duration": 3.0})
    )  # 3 s: the stand-up and a few steps, not the route
    assert r["fam"] == "scree" and r["rangefinders"] == 0 and r["route_length"] > 5.0
    assert set(
        (
            "reached",
            "fallen",
            "ctl_fallen",
            "fault",
            "diverged",
            "mean_speed",
            "n_slip",
            "mpc_fail",
        )
    ) <= set(r)
    assert MC.summarize([r])["n"] == 1


# Floors on the first held-out seeds (21000...): (n flown, successes required). Rule of the plan: (3, 2) for families with MC success >= 75 %,
# (3, 1) for 40-75 %, a family below 40 % is dropped (a floor cannot pass on a failing family; docs/dog/locomotion.md "Monte Carlo" is the claim).
# The MC (n = 20): rubble 0 %, slopes 0 %, scree 10 %, ruins 0 %, criterion 0 % success, so every family is dropped.
FLOORS = {}


@needs_kernel
@pytest.mark.parametrize("fam", list(FLOORS))
def test_monte_carlo_floor_on_held_out_seeds(fam):
    n, need = FLOORS[fam]
    res = [MC.one((MC.TEST_BASE + i, fam, {})) for i in range(n)]
    assert all(r["rangefinders"] == 0 for r in res)
    k = sum(MC.ok(r) for r in res)
    assert k >= need, (fam, k, n, [MC.why(r) for r in res if not MC.ok(r)])
