"""Dog stand / squat sanity (examples/dog): the assembly loads and simulates on its 12 Qdds under a plain joint PD (no MPC yet), the
stance is symmetric (Review Focus 3), and the scenario replays to its golden state hash."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogsim as D

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = "be1847a2fba03440"  # stand.scenario.toml: res["sim"]["state_hash"] (hfbox, 5000 ticks)


def test_golden_is_recorded():
    assert GOLDEN, "record the stand scenario's state hash (Task 8 Step 5)"


def test_ik_round_trip():
    l1, l2 = 0.22, 0.245
    for d in (0.30, 0.38, l1 + l2):
        qh, qk = D.ik(d, l1, l2)
        a, phi = (
            -D.KNEE_BACK * qh,
            D.KNEE_BACK * qk,
        )  # thigh angle off vertical, knee flexion
        foot_x = l1 * np.sin(a) - l2 * np.sin(phi - a)
        foot_z = -l1 * np.cos(a) - l2 * np.cos(phi - a)
        # 1e-7, not 1e-9: at the straight leg d = l1 + l2 the acos argument is 1 within eps and the angle is only good to sqrt(eps) ~ 1.5e-8 rad
        assert abs(foot_x) < 1e-7 and foot_z == pytest.approx(-d)


@needs_kernel
def test_stance_is_symmetric(tmp_path):  # Review Focus 3
    xml, rep = D.build(tmp_path)
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    assert {k for k, s in sg.items() if s == -1} <= {
        f"{leg}:{j}" for leg in ("fr", "rl") for j in D.JOINTS
    }
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    rows = []
    D.ramp(sim, sg, l1, l2, 1.0, D.SQUAT, 1.5, rows)
    D.ramp(sim, sg, l1, l2, D.SQUAT, D.SQUAT, 1.0, rows)
    hip = {leg: D.body(sim, f"{leg}:thigh") for leg in D.LEGS}
    knee = {leg: D.body(sim, f"{leg}:shank") for leg in D.LEGS}
    foot = {leg: D.body(sim, f"{leg}:foot_ball") for leg in D.LEGS}
    ctr = np.mean(list(hip.values()), axis=0)
    rel = {leg: foot[leg] - ctr for leg in D.LEGS}
    fy, fxy = np.array([1, -1, 1]), np.array([-1, -1, 1])
    assert np.allclose(rel["fr"], rel["fl"] * fy, atol=3e-3)  # left / right mirror
    assert np.allclose(
        rel["rr"], rel["fl"] * fxy, atol=1e-2
    )  # front / rear: the rear pair is the front pair turned 180 deg
    assert np.allclose(rel["rl"], rel["fr"] * fxy, atol=1e-2)
    for leg in ("fl", "fr"):
        assert knee[leg][0] < hip[leg][0] - 0.02  # front knees point backward
    for leg in ("rl", "rr"):
        assert knee[leg][0] > hip[leg][0] + 0.02  # rear knees point forward
    for leg in D.LEGS:
        assert (
            abs(foot[leg][0] - hip[leg][0]) < 0.015
        )  # the IK keeps each foot under its hip
    assert max(r[D.TRACE_COLS.index("fault")] for r in rows) == 0


@needs_kernel
def test_stand_scenario_passes_replays_and_matches_golden(tmp_path):
    res, _ = program.run(EX / "stand.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    assert GOLDEN is None or res["sim"]["state_hash"] == GOLDEN, res["sim"][
        "state_hash"
    ]
