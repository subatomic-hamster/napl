"""SP4 Task 6 diagnostic ladder, rungs c and d: the trot with GROUND-TRUTH state (estimator out of the loop): in place, 0.5 m/s, 1.0 m/s.
The trot is forced (loco picks 'stand' for a zero command) so the in-place case exercises the swing / stance hand-over without travel."""

import sys
from pathlib import Path

import napl
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogcourse as C
import dogladder as Ld
import dogwalk as W
from loco import controller as Ct

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@needs_kernel
@pytest.mark.parametrize("vx", [0.0, 0.5, 1.0])
def test_truth_state_trot_holds_the_command(tmp_path, monkeypatch, vx):
    monkeypatch.setattr(Ct.Gt, "choose", lambda cmd, rough: "trot")
    c = Ld.truth_state(
        W.setup(
            C.scene(tmp_path, "flat", seed=1),
            dict(W.DEFAULTS, v_cmd=vx, duration=9.0),
            log=False,
        )
    )
    n_walk = n_sat = 0
    t_a = x_a = None
    for k in range(int(9.0 / Ct.DT)):
        t = k * Ct.DT
        W.tick(c, k, (vx if t > 2.5 else 0.0, 0.0, 0.0))
        if c.ctl.mode == "WALK":
            n_walk += 1
            n_sat += int(Ld.saturation(c).max() > 0.95)
        if t_a is None and t >= 4.0:
            t_a, x_a = t, c.tr.pos()[0]
        assert not c.fell, f"fell at {t:.2f} s"
    assert n_walk > 0
    assert c.tr.tilt_deg() < 10.0
    assert abs(c.tr.pos()[1]) < 0.3
    mean_v = (c.tr.pos()[0] - x_a) / (9.0 - t_a)
    assert abs(mean_v - vx) < 0.1 + 0.1 * vx, mean_v
    assert n_sat / n_walk < 0.01, n_sat / n_walk
