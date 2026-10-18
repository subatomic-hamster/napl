"""SP4 Task 6 diagnostic ladder, rung b: the MPC alone on four feet, ground-truth state (the estimator is out of the loop). Stand 12 s, a 5 cm
body-height step, a 7 deg roll step, a 0.3 m/s lateral shove: no fall, the feet stay where they were put, no joint saturates.

The slow failure this pins: leg weight not carried by the stance torque walked the feet inward at 1.3 cm/s and the stand fell at 13 s."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogcourse as C
import dogladder as Ld
import dogwalk as W
from loco import controller as Ct

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@needs_kernel
def test_four_foot_stance_holds_steps_and_a_shove(tmp_path):
    c = Ld.truth_state(
        W.setup(
            C.scene(tmp_path, "flat", seed=1),
            dict(W.DEFAULTS, v_cmd=0.0, duration=12.0),
            log=False,
        )
    )
    ctl = c.ctl
    events = {2.5: (0, -0.05), 4.0: (0, 0.0), 8.5: (1, 0.12), 10.0: (1, 0.0)}
    sat = np.zeros(12)
    w_a = w_b = None
    for k in range(int(12.5 / Ct.DT)):
        t = k * Ct.DT
        for te, (i, v) in events.items():
            if abs(t - te) < Ct.DT / 2:
                ctl.body_ref[i] = v
        if abs(t - 11.0) < Ct.DT / 2:
            c.tr.push(np.array([0.0, 0.3, 0.0]))
        W.tick(c, k, (0.0, 0.0, 0.0))
        if t > 2.0:
            sat = np.maximum(sat, Ld.saturation(c))
            if abs(t - 4.5) < Ct.DT / 2:
                w_a = np.ptp(ctl.st.feet[:, 1])
            if abs(t - 10.9) < Ct.DT / 2:
                w_b = np.ptp(ctl.st.feet[:, 1])
        assert not c.fell, f"fell at {t:.2f} s"
    assert c.tr.tilt_deg() < 5.0
    # Two checks instead of one cumulative width change (a steady creep plus a shove slide whose sign flips with the solver's numerics: the
    # total was -0.002 .. -0.043 m over the same scenario). The creep: the failure this pins walked each foot inward at 1.3 cm/s (2.6 cm/s of
    # width); the stand now creeps 2.4 mm/s per foot (4.8 mm/s of width; open, docs/dog/locomotion.md "Known gaps").
    assert (w_a - w_b) / 6.4 < 0.010, "feet walked"
    # the 0.3 m/s shove at 11 s slides the feet a little, in either direction
    assert abs(np.ptp(ctl.st.feet[:, 1]) - w_b) < 0.03, "feet slid in the shove"
    assert sat.max() < 0.95, sat.reshape(4, 3).round(2)
