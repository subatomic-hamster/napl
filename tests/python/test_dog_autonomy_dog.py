import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
import numpy as np
import pytest
from autonomy import config, dogloco, lidar, localize, scene

HAS_SP4 = (ROOT / "examples" / "dog" / "loco" / "controller.py").exists()
CPP_WBC = bool(list((ROOT / "build" / "loco").glob("loco_wbc_cpp*")))
GOLDEN_DOG_SMOKE = "9eb34232ef72640e"  # re-recorded (fix round 2: tilt correction no longer paused by command changes, gyro-bias integrator; was 27ffca55529022aa):LOCO fix 2/3 (WBC J^T stance torque, POSTURE 0.7, scheduled stand height); default seed, 15 s, C++ WBC, OMP_NUM_THREADS=1
needs_dog = pytest.mark.skipif(
    napl._napl is None or not HAS_SP4,
    reason="kernel or SP4 (examples/dog/loco) missing",
)


def test_odometry_takes_sp5_fixes_back_out():
    led = dogloco.FixLedger()
    R = lambda a: localize.rz(a)
    out = [led.odom(np.array([float(t), 0.0, 0.3]), R(0.0)) for t in range(6)]
    p5 = np.array([5.0, 0.0, 0.3])
    p5j, d = (
        p5 + np.array([0.3, -0.2, 0.0]),
        0.1,
    )  # SP4 absorbed a fix: position and yaw jump
    led.jumped(p5, R(0.0), p5j, R(d))
    for t in range(6, 10):
        p_est = p5j + R(d) @ np.array(
            [t - 5.0, 0.0, 0.0]
        )  # SP4 keeps integrating in its jumped frame
        out.append(led.odom(p_est, R(d)))
    for t, (p, yaw) in enumerate(out):
        np.testing.assert_allclose(p, [t, 0.0, 0.3], atol=1e-12)
        assert yaw == pytest.approx(0.0, abs=1e-12)


@needs_dog
def test_dog_scene_keeps_the_lidar_and_does_not_see_itself(tmp_path):
    auto = {"kind": "rubble", "size": 20.0, "patch": 10.0, "rays": 5000}
    f = scene.make_field(auto, 1)
    p = dogloco.build(f, auto, 1, (-8.0, -8.0, 0.0), tmp_path)
    xml = p.read_text()
    lid, gn = dogloco.devices(xml)
    assert (
        lid["projection"] == "rosette"
        and lid["rays_per_frame"] == 5000
        and gn["rate_hz"] > 0
    )
    ext = lidar.extrinsic(
        xml, lid["site"], dogloco.TRUNK
    )  # pos / quat / euler only, else ValueError
    sim = napl.Sim.load(p)
    cfg = dogloco.cfg_from_devices(
        config.override(config.load(), **{"map.size": 20.0}), lid, gn
    )
    dog = dogloco.DogLoco(
        sim, p, cfg, f, (-8.0, -8.0, 0.0), float(f.height(-8.0, -8.0))
    )
    ros = lidar.Rosette(sim, lid["name"], 5000, dog.dt, cfg)
    dog.advance(1.0)  # standing up
    s = ros.latest()
    assert (
        s is not None and s.r.min() > 0.6
    )  # nothing of the dog's own body in the returns
    assert np.isfinite(ext[0]).all()


@needs_dog
def test_dog_smoke_passes_and_replays(tmp_path):
    from napl import program

    res, _ = program.run(
        ROOT
        / "examples"
        / "dog"
        / "missions"
        / "dog_smoke"
        / "dog_smoke.scenario.toml",
        tmp_path / "runs",
    )
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["pass"], res["fail_reasons"]
    if (
        CPP_WBC
    ):  # the golden is the C++ WBC's (the Python class differs in the last bits)
        assert res["sim"]["state_hash"] == GOLDEN_DOG_SMOKE, res["sim"]["state_hash"]
