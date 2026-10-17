"""SP4 Task 6: the controller on flat ground: stand-up from the build pose, a 1 m/s trot, estimator accuracy, the torque-based contact estimate
against the virtual FootContact (Review Focus 4), the controller's inputs (Review Focus 2), golden state hash of walk/trot.scenario.toml."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C
import dogwalk as W
from loco import hw as H

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = {
    "trot": "b3a9d5997271ab5a"
}  # res["sim"]["state_hash"] of walk/trot.scenario.toml (Task 6 Step 5; re-record if a later task changes the trot)
LEGS = ("fl", "fr", "rl", "rr")
c = W.COLS.index


def test_goldens_are_recorded():
    assert all(GOLDEN.values()), (
        "record walk/trot.scenario.toml's state hash (Task 6 Step 5)"
    )


def test_loco_never_touches_the_sim_or_the_truth():  # Review Focus 2, static half
    for f in (EX / "loco").glob("*.py"):
        if f.name != "hw.py":
            src = f.read_text()
            for bad in ("import napl", "from napl", "sim.", "dogtruth", "dogcourse"):
                assert bad not in src, (f.name, bad)


@pytest.fixture(scope="module")
def trot(tmp_path_factory):
    model = C.scene(tmp_path_factory.mktemp("trot"), "flat", seed=1)
    return W.run(
        model, dict(W.DEFAULTS, v_cmd=1.0, duration=8.0), log=False, audit=True
    )


@needs_kernel
def test_controller_reads_only_hardware_attrs(trot):  # Review Focus 2, dynamic half
    reads = trot["ctl"].hw.reads
    assert reads and all(p.startswith("/devices/") for p, _ in reads)
    assert {a for _, a in reads} <= H.HW_ATTRS
    assert not any(
        p.endswith(":foot") for p, _ in reads
    )  # the virtual FootContact is ground truth, not a sensor


@needs_kernel
def test_trot_tracks_one_metre_per_second(trot):
    rows = np.array(trot["rows"])
    late = rows[rows[:, c("t")] > 5.0]
    assert late[:, c("vx")].mean() == pytest.approx(1.0, abs=0.1)
    assert rows[:, c("fallen")].max() == 0 and rows[:, c("fault")].max() == 0
    assert np.abs(late[:, c("y")]).max() < 0.3
    assert np.sqrt(np.mean(late[:, c("est_v_err")] ** 2)) < 0.1


@needs_kernel
def test_contact_estimate_matches_foot_contact_truth(trot):  # Review Focus 4
    rows = np.array(trot["rows"])
    late = rows[:, c("t")] > 3.0
    est = rows[late][:, [c(f"c_est_{l}") for l in LEGS]]
    tru = rows[late][:, [c(f"c_true_{l}") for l in LEGS]]
    assert (est == tru).mean() >= 0.95


@needs_kernel
def test_stands_up_from_the_build_pose(tmp_path):
    r = W.run(
        C.scene(tmp_path, "flat", seed=1),
        dict(W.DEFAULTS, v_cmd=0.0, duration=3.0),
        log=False,
    )
    last = r["rows"][-1]
    assert (
        last[c("mode")] == W.MODE_CODE["STAND"]
        and last[c("tilt")] < 2.0
        and last[c("fault")] == 0
    )


@needs_kernel
@pytest.mark.parametrize("name", ["trot"])
def test_scenario_passes_replays_and_matches_golden(tmp_path, name):
    res, _ = program.run(EX / "walk" / f"{name}.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    assert GOLDEN[name] is None or res["sim"]["state_hash"] == GOLDEN[name], res["sim"][
        "state_hash"
    ]
