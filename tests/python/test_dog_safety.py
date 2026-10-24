"""SP4 Task 9: slip detection (synthetic stance-foot velocities; a mud strip), push rejection (golden scenario), fall -> safe-sit without driver
faults or stiff joints."""

import sys
from pathlib import Path
from types import SimpleNamespace

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C
import dogwalk as W
from loco import model as M
from loco import safety as S
from loco import wbc as Wb

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = {
    "push": "d63fd6b4e3399485"
}  # res["sim"]["state_hash"] of walk/push.scenario.toml with the C++ WBC (fix round 2 re-record: tilt correction no longer paused by command changes, gyro-bias integrator; c73b5e477107f5ba before; fix round 1: condensed MPC 628c2645672cfdcf, 2f3b3706cb51a5b5 before; then J^T stance torque)
c = W.COLS.index


class FlatLeg:
    def fk_jac(self, q):
        return np.array(q, float), np.eye(3)


def stance_state(v_foot):
    st = SimpleNamespace(
        R=np.eye(3),
        v=np.zeros(3),
        w=np.zeros(3),
        contact=np.ones(4, bool),
        rpy=np.zeros(3),
        h_map=0.35,
    )
    return st, np.tile([0.2, 0.1, -0.35], 4), np.tile(v_foot, 4)


def test_slip_lowers_mu_slows_and_recovers():
    s = S.Safety()
    phases = [SimpleNamespace(stance=True, late=False, early=False, s=0.0)] * 4
    legs = [FlatLeg()] * 4
    st, q, dq = stance_state([0.0, 0.0, 0.0])
    for _ in range(50):
        assert s.update(st, phases, q, dq, legs, 0.002) is None
    st, q, dq = stance_state([0.5, 0.0, 0.0])  # feet sliding at 0.5 m/s while in stance
    ev = [s.update(st, phases, q, dq, legs, 0.002) for _ in range(S.SLIP_TICKS + 1)]
    assert (
        "slip" in ev
        and s.mu.min() <= S.MU_NOM - S.MU_DROP + 1e-9
        and s.speed_scale < 1.0
    )
    st, q, dq = stance_state([0.0, 0.0, 0.0])
    for _ in range(int(10 / 0.002)):
        s.update(st, phases, q, dq, legs, 0.002)
    assert s.mu.min() == pytest.approx(S.MU_NOM) and s.speed_scale == 1.0


def test_fall_detector_thresholds():
    s = S.Safety()
    up = SimpleNamespace(rpy=np.array([0.1, 0.0, 0.0]), w=np.zeros(3), h_map=0.35)
    assert not s.fallen(up, 0.35)
    assert s.fallen(
        SimpleNamespace(
            rpy=np.array([np.radians(S.TILT_FALL + 1), 0, 0]),
            w=np.zeros(3),
            h_map=0.35,
        ),
        0.35,
    )
    assert s.fallen(
        SimpleNamespace(
            rpy=np.array([np.radians(S.TILT_FALL_FAST + 1), 0, 0]),
            w=np.array([S.W_FALL + 1, 0, 0]),
            h_map=0.35,
        ),
        0.35,
    )
    assert s.fallen(SimpleNamespace(rpy=np.zeros(3), w=np.zeros(3), h_map=0.1), 0.35)


@needs_kernel
def test_the_mud_strip_is_crossed_without_a_false_slip(tmp_path):
    """The plan asked for a slip DETECTED on this crossing; the dog does not slide on the mud strip at 1 m/s (the foot-velocity statistic only
    shows the 20 ms touchdown transients of 0.2-0.4 m/s), so there is nothing to detect and a detector that fired here (the plan's 3 ticks did,
    4 times, and cost 15 % of the speed) is a false alarm. The detection is tested on a real slide below."""
    r = W.run(
        C.scene(tmp_path, "slip", seed=1),
        dict(
            W.DEFAULTS, course="slip", v_cmd=0.8, duration=40.0
        ),  # 0.8: the controller's terrain speed cap (V_TERRAIN, fix round 3)
        log=False,
    )
    rows = np.array(r["rows"])
    assert rows[:, c("fallen")].max() == 0 and r["res"]["reached"]
    assert r["ctl"].stats["n_slip"] == 0 and rows[:, c("mu_min")].min() == S.MU_NOM


@needs_kernel
def test_a_kick_on_mud_is_a_real_slide_and_is_detected(tmp_path):
    p = dict(
        W.DEFAULTS, course="slip", v_cmd=1.0, duration=20.0, push_t=16.5, push_dv=1.0
    )  # at x = 1.7 m, on the mud: sideways velocity step 1 m/s
    r = W.run(C.scene(tmp_path, "slip", seed=1), p, log=False)
    rows = np.array(r["rows"])
    assert (
        r["ctl"].stats["n_slip"] >= 1
        and rows[:, c("mu_min")].min()
        <= S.MU_NOM - S.MU_DROP + S.MU_RECOVER * 0.02 + 1e-9
    )  # (one 50 Hz trace row of mu recovery after the drop: the trace may miss the exact tick)  # recovering from it is not asserted: docs/dog/locomotion.md "Slips, pushes, falls"


@needs_kernel
def test_a_fall_ends_in_safe_sit_without_faults(tmp_path):
    p = dict(
        W.DEFAULTS, v_cmd=1.0, duration=8.0, push_t=4.0, push_dv=3.0
    )  # 36 N s sideways: beyond any recovery
    r = W.run(C.scene(tmp_path, "flat", seed=1), p, log=False)
    rows = np.array(r["rows"])
    t = rows[:, c("t")]
    tilt_over = t[rows[:, c("tilt")] > 45.0]
    ctl_fell = t[rows[:, c("ctl_fallen")] > 0]
    assert len(ctl_fell) and (
        not len(tilt_over) or ctl_fell[0] <= tilt_over[0] + 0.3
    )  # detected within 0.3 s of the truth
    assert rows[:, c("fault")].max() == 0
    assert r["ctl"].mode in ("FALLEN", "SIT", "STANDUP", "STAND", "WALK")
    if r["ctl"].mode in ("FALLEN", "SIT"):
        assert np.all(r["ctl"].hw._kp <= S.KP_SIT)  # limp, not stiff, on the ground


@needs_kernel
def test_push_scenario_passes_replays_and_matches_golden(tmp_path):
    res, _ = program.run(EX / "walk" / "push.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    if sys.platform != "darwin":
        if isinstance(Wb.make(M.from_build(C.base_model())), Wb.Wbc):
            pytest.skip("hfbox golden was recorded with the C++ WBC")
        assert res["sim"]["state_hash"] == GOLDEN["push"], res["sim"]["state_hash"]


@needs_kernel
@pytest.mark.parametrize("t_push, dv", [(6.04, -0.4), (6.08, 0.4)])
def test_a_lateral_push_at_other_gait_phases_is_survived(t_push, dv):
    """Fix round 1: a 0.4 m/s push at a gait phase other than the Task 9 envelope's (t = 5.0 s) toppled the WBC dog in 19 of 24 phase x seed cases
    because its own stance torque (inertial terms of the body tasks, the QP's force change) fought the MPC forces; the stance joints now carry the MPC
    force through J^T (loco.controller WBC_STANCE_JT): 24 of 24. These two phases fell with the WBC torque."""
    import dogflat

    r = dogflat.one(21000, v=1.0, dur=9.0, off="bias,pay", pushes=[(t_push, dv)])
    assert not r["fallen"], r["fall"]
