"""SP4 Task 5: gait scheduler (pairs, duty, transitions at the phase wrap, early touchdown), swing curves, Raibert + capture-point footsteps."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "dog"))
from loco import gait as Gt

DT = 0.002


def run(s, n, contact=None):
    """Ticks; with contact=None the feet touch down exactly as scheduled (contact = the previous tick's scheduled stance)."""
    out, c = [], contact or (True,) * 4
    for _ in range(n):
        legs = s.advance(DT, c)
        out.append(legs)
        c = contact or tuple(lp.stance for lp in legs)
    return out


@pytest.mark.parametrize(
    "name,pairs", [("trot", [(0, 3), (1, 2)]), ("bound", [(0, 1), (2, 3)])]
)
def test_pairs_move_together_and_duty_holds(name, pairs):
    s = Gt.Scheduler(name)
    g = Gt.GAITS[name]
    hist = np.array(
        [[lp.stance for lp in legs] for legs in run(s, round(g.period / DT) * 4)]
    )
    for a, b in pairs:
        assert (hist[:, a] == hist[:, b]).all()
    assert hist.mean(axis=0) == pytest.approx(np.full(4, g.duty), abs=0.01)


def test_gait_change_waits_for_the_phase_wrap():
    s = Gt.Scheduler("trot")
    run(s, 10)
    s.request("bound")
    assert s.g.name == "trot"
    run(s, round(Gt.GAITS["trot"].period / DT))
    assert s.g.name == "bound"


def test_choose_by_speed_and_terrain():
    assert Gt.choose((0.0, 0.0, 0.0), False) == "stand"
    assert Gt.choose((1.0, 0.0, 0.0), False) == "trot"
    assert Gt.choose((2.6, 0.0, 0.0), False) == "flying_trot"
    assert Gt.choose((3.6, 0.0, 0.0), False) == "bound"
    assert Gt.choose((1.0, 0.0, 0.0), True) == "walk"


def test_early_touchdown_turns_the_leg_to_stance():
    s = Gt.Scheduler("trot")
    g = Gt.GAITS["trot"]
    while True:  # leg 0 just past mid-swing
        legs = s.advance(DT, (False, True, True, False))
        if not legs[0].stance and legs[0].s > 0.6:
            break
    for _ in range(
        Gt.TD_TICKS - 1
    ):  # a short blip of contact does not stand the leg up
        legs = s.advance(DT, (True, True, True, False))
        assert not legs[0].stance
    s.advance(
        DT, (False, True, True, False)
    )  # ... and a lost contact restarts the count
    for _ in range(Gt.TD_TICKS - 1):
        legs = s.advance(DT, (True, True, True, False))
    assert not legs[0].stance
    legs = s.advance(DT, (True, True, True, False))
    assert legs[0].stance and legs[0].early
    assert not s.gate()[0] and g.duty < 1.0


def test_swing_curve_ends_apex_and_rest():
    p0, p1, T = np.array([0.0, 0.0, 0.0]), np.array([0.2, 0.05, 0.15]), 0.16
    a, va = Gt.swing(p0, p1, 0.08, 0.0, T)
    b, vb = Gt.swing(p0, p1, 0.08, 1.0, T)
    m, _ = Gt.swing(p0, p1, 0.08, 0.5, T)
    assert (
        np.allclose(a, p0)
        and np.allclose(b, p1)
        and np.allclose(va, 0)
        and np.allclose(vb, 0)
    )
    assert m[2] == pytest.approx(0.15 + 0.08)
    s = np.linspace(0, 1, 401)
    z = np.array([Gt.swing(p0, p1, 0.08, x, T)[0][2] for x in s])
    assert np.abs(np.diff(z)).max() < 0.005  # continuous


def test_raibert_capture_point_footstep():
    hip, h, Ts = np.array([0.3, 0.1, 0.4]), 0.35, 0.16
    v = np.array([1.0, 0.0, 0.0])
    xy = Gt.nominal_foothold(hip, v, v, 0.0, Ts, h)
    assert xy == pytest.approx(hip[:2] + [v[0] * Ts / 2, 0.0])
    fast = Gt.nominal_foothold(hip, np.array([1.3, 0.0, 0.0]), v, 0.0, Ts, h)
    assert (
        fast[0] > xy[0] + Gt.K_CAP * np.sqrt(h / Gt.G) * 0.3 * 0.99
    )  # running ahead of the command: step further out
    far = Gt.nominal_foothold(hip, np.array([9.0, 0.0, 0.0]), v, 0.0, Ts, h)
    assert np.linalg.norm(far - hip[:2]) == pytest.approx(Gt.D_MAX)
