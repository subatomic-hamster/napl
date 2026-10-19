"""SP4 Task 10: top speed. The report states the measured speed and the gap to the 3.5 m/s criterion honestly (pure); flying trot beats the floor on
flat ground with each leg controller at its best stand height; the SP3b sweep slot runs on a sweep model and refuses a report without the model path."""

import sys
from pathlib import Path

import napl
import pytest

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C
import dogsim as D
import dogtop as T

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
FLOOR = 1.9  # m/s: min(2.0, floor(0.85 x the flying-trot v_top of the search, 0.1)) = floor(0.85 x 2.25): below the plan's 2.0 (docs/dog/locomotion.md "Top speed")
H_BEST = {
    "wbc": 0.85,
    "jt": 0.95,
}  # stand height fraction of each leg controller's best flying trot (docs/dog/locomotion.md)


def res(v):
    return dict(gait="flying_trot", leg_ctl="wbc", v_top=v, params=dict(period=0.28, duty=0.38), by_leg=dict(wbc=v, jt=v - 0.3),
                validate=dict(n=10, median=v, min=v - 0.1),
                diag=dict(sat_hip=0.42, sat_knee=0.10, w_frac=0.93, n_slip=3, i_pack_peak=81.0, cot=0.9))  # fmt: skip


def test_report_states_measured_top_speed_and_gap():
    md = T.report_md(res(1.71))
    assert "flat top speed (measured, flying_trot, wbc): 1.71 m/s" in md
    assert "spec target 2.0 m/s: NOT met (gap 0.29 m/s)" in md
    assert (
        "leg controller: wbc 1.71 m/s, jt 1.41 m/s" in md
    )  # the WBC is measured against the J^T baseline, not assumed better
    assert (
        "hip torque at its limit" in md and "of no-load speed" in md
    )  # the limiting factors are named, not guessed
    assert "spec target 2.0 m/s: met" in T.report_md(res(2.1))
    assert "no held-out seeds run" in T.report_md({**res(2.0), "validate": None})


@needs_kernel
@pytest.mark.parametrize("leg_ctl", ["wbc", "jt"])
def test_flying_trot_beats_the_floor(tmp_path, leg_ctl):
    r = T.measure(
        C.scene(tmp_path, "flat", seed=1),
        "flying_trot",
        {"h_frac": H_BEST[leg_ctl]},
        FLOOR,
        leg_ctl=leg_ctl,
    )
    assert r["ok"], r["diag"]


@needs_kernel
def test_sweep_gait_slot(tmp_path, monkeypatch):
    monkeypatch.setattr(T, "SWEEP_RUNGS", (1.0,))  # one rung: the slot, not the sweep
    xml, rep = D.build(tmp_path)
    sim = napl.Sim.load(xml)
    with pytest.raises(ValueError, match="xml"):
        T.sweep_gait(sim, D.signs(rep), rep)
    assert (
        T.sweep_gait(napl.Sim.load(xml), D.signs(rep), {**rep, "xml": str(xml)}) > 0.9
    )
