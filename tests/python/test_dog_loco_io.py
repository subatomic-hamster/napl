"""SP4 Task 1: the controller's hardware door (loco/hw.py), the harness ground truth (dogtruth.py), the flat scene with the lidar removed
(dogcourse.py). Needs the built kernel for the Sim parts."""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C
import dogtruth as T
from loco import hw as H

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def devices(xml):
    return json.loads(
        ET.parse(xml).getroot().find("custom/text[@name='napl.devices']").get("data")
    )


def test_osqp_is_importable():
    import osqp  # noqa: F401  (the loco extra is installed)


def test_flat_scene_has_no_lidar_and_records_its_nominal_model(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    assert (
        devices(model).get("rangefinders", []) == []
    )  # spec s9 risk 1: locomotion runs never cast rays
    assert (
        len(devices(C.base_model())["rangefinders"]) == 2
    )  # the nominal dog keeps head:lidar and head:depth
    inf = C.info(model)
    assert inf["course"] == "flat" and Path(inf["nominal"]) == C.base_model()


@needs_kernel
def test_hw_reads_only_hardware_attrs_and_sends_mit_frames(tmp_path):
    sim = napl.Sim.load(C.scene(tmp_path, "flat", seed=1))
    hw = H.Hw(sim, audit=True)
    hw.enable()
    q, dq, tau = hw.joints()
    gyro, acc = hw.imu()
    hw.health()
    assert q.shape == dq.shape == tau.shape == (12,) and gyro.shape == acc.shape == (3,)
    hw.send(q, np.zeros(12), np.full(12, 20.0), np.full(12, 1.0), np.full(12, 0.5))
    sim.step(4)
    assert (
        abs(float(sim.query(H.QDD[4], "tau_cmd")[0][0])) > 0.1
    )  # the frame reached the driver (a test-side read, not the Hw's)
    assert all(p.startswith("/devices/") for p, _ in hw.reads)
    assert {a for _, a in hw.reads} <= H.HW_ATTRS


@needs_kernel
def test_truth_push_changes_the_velocity_through_the_bus(tmp_path):
    sim = napl.Sim.load(C.scene(tmp_path, "flat", seed=1))
    sim.start_log()
    tr = T.Truth(sim)
    sim.step(2)
    v0 = tr.vel()
    tr.push([0.0, 0.5, 0.0])
    assert tr.vel()[1] == pytest.approx(v0[1] + 0.5, abs=1e-9)
    assert tr.contacts().shape == (4,)
    sim.step(10)
    assert sim.log() is not None  # the push is a logged SetAttr: replay reproduces it


@needs_kernel
def test_set_many_is_set_in_a_loop(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    a, b = napl.Sim.load(model), napl.Sim.load(model)
    paths = [p for p in H.QDD for _ in range(2)]
    attrs = ["enable", "tau_ff"] * 12
    vals = [1.0, 0.7] * 12
    a.set_many(paths, attrs, vals)
    for p, at, v in zip(paths, attrs, vals):
        b.set(p, at, v)
    a.step(20)
    b.step(20)
    assert a.state_hash() == b.state_hash()
    with pytest.raises(ValueError):
        a.set_many(["/devices/fl:hip"], ["kp"], [-1.0])
