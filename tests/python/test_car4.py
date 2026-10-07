"""The 4-wheel skid-steer car (assemblies/car4.toml, built from an empty design: docs/showcase/car/README.md): DRC 0 errors, then driven
through two RoboClaw ptys by examples/car4/square.py (the unmodified `basicmicro` library): a 1 m square, and the skid-steer turn
measured against the commanded one (spin in place by encoder odometry, true yaw from the plate quaternion). Run with -s for the numbers."""

import csv
import math
import re
from pathlib import Path

import napl
import pytest
from napl import design, program
from test_rover import pose, q

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "car4"
WHEELS = {s: f"/devices/wheel_{s}" for s in ("fl", "rl", "fr", "rr")}
R_WHEEL = 0.045
TRACK_GEOMETRIC = 0.223  # wheel centre to wheel centre (the rover's, same brackets)


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    out = tmp_path_factory.mktemp("car4")
    xml, _ = design.compute("assemblies/car4.toml", "parts", out)
    (out / "car4.xml").write_text(xml)
    return out / "car4.xml"


def test_drc_has_no_errors_and_no_warnings():
    fs = design.check("assemblies/car4.toml")
    assert [f for f in fs if f.severity in ("error", "warning")] == [], [
        (f.code, f.where) for f in fs
    ]
    # the one info is the servo-bus rule seeing the 4 wheel `servo` devices as a shared serial bus (same as assemblies/rover.toml)
    assert {f.code for f in fs} <= {"SERVO_ID_UNSET"}


def test_mass_and_stance(model):
    _, rep = design.compute("assemblies/car4.toml", "parts", model.parent)
    assert rep["mass"]["total"] == pytest.approx(1.587, abs=0.01)
    sim = napl.Sim.load(model)
    sim.step(1000)
    assert pose(sim)[0] == pytest.approx(0.0, abs=1e-3)
    assert float(sim.query("/world/plate", "pos")[0][2]) == pytest.approx(
        0.0217, abs=0.001
    )  # four wheels on the floor, no sag


def test_straight_line_duty(model):
    sim = napl.Sim.load(model)
    sim.step(500)
    for w in WHEELS.values():
        sim.set(w, "torque_enable", 1)
        sim.set(w, "mode", 1)
        sim.set(w, "goal_duty", 0.3)
    sim.step(2000)
    x0, y0, _ = pose(sim)
    sim.step(1000)
    x1, y1, yaw = pose(sim)
    v = x1 - x0
    wheel = sum(q(sim, w, "velocity") for w in WHEELS.values()) / 4 * R_WHEEL
    print(
        f"\nstraight, duty 0.3: {v:.4f} m/s (wheel rate x radius {wheel:.4f}), drift {1000 * (y1 - y0):.3f} mm/s, yaw {math.degrees(yaw):.4f} deg"
    )
    assert v == pytest.approx(wheel, rel=0.01)
    assert abs(y1 - y0) < 0.002 and abs(yaw) < math.radians(0.2)
    assert (
        max(q(sim, w, "current") for w in WHEELS.values()) < 1.0
    )  # 4 motors on 2 controllers, nowhere near 7.5 A


def unwrapped_spin(model, duty):
    """Spin in place (left side -duty, right side +duty) for 2 s after 1.5 s: -> (true yaw rate, odometry yaw rate at the geometric track)."""
    sim = napl.Sim.load(model)
    sim.step(500)
    for s, w in WHEELS.items():
        sim.set(w, "torque_enable", 1)
        sim.set(w, "mode", 1)
        sim.set(w, "goal_duty", -duty if s[1] == "l" else duty)
    sim.step(1500)
    yaw = lambda: pose(sim)[2]
    unw, last, odo = 0.0, yaw(), 0.0
    for _ in range(20):
        sim.step(100)
        y = yaw()
        unw += (y - last + math.pi) % (2 * math.pi) - math.pi
        last = y
        vl = (
            (q(sim, WHEELS["fl"], "velocity") + q(sim, WHEELS["rl"], "velocity"))
            / 2
            * R_WHEEL
        )
        vr = (
            (q(sim, WHEELS["fr"], "velocity") + q(sim, WHEELS["rr"], "velocity"))
            / 2
            * R_WHEEL
        )
        odo += (vr - vl) * 0.1
    return unw / 2.0, odo / TRACK_GEOMETRIC / 2.0


def test_skid_steer_effective_track(model):
    for duty in (0.3, 0.5):
        true, odo = unwrapped_spin(model, duty)
        eff = TRACK_GEOMETRIC * odo / true
        print(
            f"\nspin in place, duty {duty}: true {math.degrees(true):.1f} deg/s, odometry at the geometric track {math.degrees(odo):.1f} deg/s "
            f"(true / odometry {true / odo:.2f}), effective track {eff:.3f} m (geometric {TRACK_GEOMETRIC})"
        )
        assert (
            0.38 < eff < 0.46
        )  # scrub: ~1.9 x the geometric track (examples/car4/square.py TRACK = 0.42)


def true_yaw_deg(run):
    with open(run / "trace.csv") as f:
        last = list(csv.DictReader(f))[
            -1
        ]  # the plate quaternion columns qw / qz the scenarios record
    yaw = math.degrees(2 * math.atan2(float(last["qz"]), float(last["qw"])))
    return (
        (yaw + 180) % 360 - 180 if abs(yaw) > 180 else yaw
    )  # q and -q are the same rotation


@pytest.mark.parametrize(
    "name,lo,hi", [("turn90", 85, 98), ("turn90_geometric", 40, 58)]
)
def test_commanded_90_degree_turn_vs_true(tmp_path, name, lo, hi):
    pytest.importorskip("basicmicro")
    res, d = program.run(EX / f"{name}.scenario.toml", tmp_path)
    odo = float(
        re.search(r"heading=(-?[\d.]+) deg", (d / "program.log").read_text())[1]
    )
    yaw = true_yaw_deg(d)
    print(
        f"\n{name}: commanded 90 deg, odometry says {odo:.1f}, true yaw {yaw:.1f} deg; replay {res['replay']}"
    )
    assert res["pass"], res["fail_reasons"]
    assert lo < yaw < hi
    assert res["replay"]["available"] and res["replay"]["match"]


def test_one_metre_square_through_two_roboclaws(tmp_path):
    pytest.importorskip("basicmicro")
    res, d = program.run(EX / "square.scenario.toml", tmp_path)
    mons = {m["name"]: m for m in res["monitors"]}
    log = (d / "program.log").read_text()
    ox, oy = map(
        float, re.search(r"odometry end pose x=(-?[\d.]+) y=(-?[\d.]+)", log).groups()
    )
    final = mons["back_at_start"]["final"]
    print(
        f"\n1 m square: {res['sim']['sim_s']:.1f} s sim, {res['program']['wall_s']:.1f} s wall; true end {1000 * final:.1f} mm from the start, "
        f"odometry end ({1000 * ox:.1f}, {1000 * oy:.1f}) mm, true end yaw {true_yaw_deg(d):.1f} deg; replay {res['replay']}"
    )
    assert res["pass"], res["fail_reasons"]
    assert (
        final < 0.12
    )  # measured 34 - 90 mm over ten runs (heading overshoot of the scrubbing turns, invisible to the encoders)
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["sim"]["diverged"] is False
    assert not res["sim"]["port_errors"]
