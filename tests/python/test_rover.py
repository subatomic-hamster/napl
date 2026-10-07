"""A4: the rover (assemblies/rover.toml: 2 x Pololu 37D 4751 gearmotor, RoboClaw 2x7A, 3S LiPo) against its datasheet and on the floor.

Motor points at exactly 12.000 V at the motor (the pack's `ocv_offset` fault input is trimmed until the wheel servo's terminal voltage reads
12.000 V, so the 3S pack's own sag and 12.31 V do not enter): free run (530 rpm / 0.2 A), stall (5.5 A / 8.5 kg cm), and the maximum
efficiency point (470 rpm at 1.0 kg cm, 0.76 A, 55 %; the load is the servo's `ext_torque` fault input on the wheel joint). Free-run, stall current and
stall torque are the numbers the motor model was derived from (R, Ke, eta: docs/research/rover.md), so they check the implementation (law,
gearbox, encoder units, thermal state), not the data; the maximum-efficiency point is NOT an input of the derivation, it is the one independent
check. Run with -s for the numbers.
Then the straight line on the floor and the 1 m square by encoder odometry through `napl program` (needs the `rover` extra and napl-replay)."""

import math
import re
from pathlib import Path

import napl
import pytest
from napl import design, program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "rover"
W = ("/devices/wheel_l", "/devices/wheel_r")
R_WHEEL, CPR = 0.045, 1200
RPM = 60 / (2 * math.pi)
KGCM = 0.0980665  # N m per kg cm


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    out = tmp_path_factory.mktemp("rover")
    xml, _ = design.compute("assemblies/rover.toml", "parts", out)
    (out / "rover.xml").write_text(xml)
    return out / "rover.xml"


def q(sim, path, attr, i=0):
    return float(sim.query(path, attr)[i][0])


def start(model, mode=1):
    sim = napl.Sim.load(model)
    sim.step(500)
    for w in W:
        sim.set(w, "torque_enable", 1)
        sim.set(w, "mode", mode)
    return sim


def trim_to(sim, volts, steps=300, n=4, tol=2e-3):
    """Trim the battery's ocv_offset until wheel_l's terminal voltage reads `volts` (n passes of `steps` ms)."""
    off = 0.0
    for _ in range(n):
        sim.step(steps)
        off += volts - q(sim, W[0], "voltage")
        sim.set("/devices/battery", "ocv_offset", off)
    sim.step(steps)
    assert q(sim, W[0], "voltage") == pytest.approx(volts, abs=tol)


def point(model, load_kgcm=0.0, stall=False, duty=1.0, volts=12.0):
    """-> (rpm, amps, N m output torque, volts) of wheel_l at full duty and `volts`, a load of `load_kgcm` opposing the motion on both wheels."""
    sim = start(model)
    for w in W:
        sim.set(w, "goal_duty", duty)
        sim.set(w, "ext_torque", -load_kgcm * KGCM)
    if stall:
        sim.set(W[0], "jam", 1)
        trim_to(
            sim, volts, steps=20, n=8, tol=0.01
        )  # the jam spring's creep moves the current a little
    else:
        sim.step(1500)
        trim_to(sim, volts)
    return (
        q(sim, W[0], "velocity") * RPM,
        q(sim, W[0], "current"),
        q(sim, W[0], "torque"),
        q(sim, W[0], "voltage"),
    )


def err(model_value, ref):
    return 100 * (model_value - ref) / ref


def test_free_run_at_12v_vs_datasheet(model):
    rpm, amps, _, v = point(model)
    print(
        f"\nfree run @ {v:.3f} V: {rpm:.1f} rpm (datasheet 530, {err(rpm, 530):+.2f} %), {amps:.3f} A (datasheet 0.2, {err(amps, 0.2):+.1f} %)"
    )
    assert rpm == pytest.approx(530, rel=0.03)
    assert amps == pytest.approx(
        0.2, rel=0.15
    )  # includes the wheel's rolling drag on the floor


def test_stall_at_12v_vs_datasheet(model):
    rpm, amps, tq, v = point(model, stall=True)
    kgcm = tq / KGCM
    print(
        f"\nstall @ {v:.3f} V: {amps:.3f} A (datasheet 5.5, {err(amps, 5.5):+.2f} %), {kgcm:.2f} kg cm (datasheet 8.5, {err(kgcm, 8.5):+.2f} %), {rpm:.2f} rpm"
    )
    assert abs(rpm) < 2
    assert amps == pytest.approx(5.5, rel=0.03)
    assert (
        kgcm == pytest.approx(8.5, rel=0.06)
    )  # eta is derived from this very torque: the residual is the thermal / friction detail of the law


def test_max_efficiency_point_vs_datasheet(model):
    rpm, amps, tq, v = point(model, load_kgcm=1.0)
    eff = 100 * (tq * rpm / RPM) / (v * amps)
    print(
        f"\nmax efficiency point @ {v:.3f} V, load 1.0 kg cm: {rpm:.1f} rpm (datasheet 470, {err(rpm, 470):+.2f} %), {amps:.3f} A (datasheet 0.76, {err(amps, 0.76):+.1f} %), "
        f"output torque {tq / KGCM:.2f} kg cm, efficiency {eff:.1f} % (datasheet 55), output power {tq * rpm / RPM:.2f} W (datasheet 5.0)"
    )
    assert rpm == pytest.approx(470, rel=0.04)
    assert amps == pytest.approx(0.76, rel=0.10)
    assert (
        eff == pytest.approx(55, rel=0.12)
    )  # the model's current is 7 % high here (docs/research/rover.md), so the efficiency is 10 % low


def pose(sim):
    p, qt = sim.query("/world/plate", "pos")[0], sim.query("/world/plate", "quat")[0]
    return float(p[0]), float(p[1]), 2 * math.atan2(float(qt[3]), float(qt[0]))


def test_straight_line_on_flat_ground(model):
    sim = start(model)
    for w in W:
        sim.set(w, "goal_duty", 0.5)
    sim.step(1500)  # past the start-up
    x0, y0, _ = pose(sim)
    t0 = sim.tick
    sim.step(3000)
    x1, y1, yaw = pose(sim)
    t = (sim.tick - t0) * 1e-3
    v, w_rad = (x1 - x0) / t, (q(sim, W[0], "velocity") + q(sim, W[1], "velocity")) / 2
    volts = q(sim, W[0], "voltage")
    # datasheet-scaled prediction: the motor sees duty x V; speed = 530 rpm x (duty V - i0 R) / (12 - i0 R), i0 = 0.2 A, R = 12 / 5.5 ohm
    drop = 0.2 * 12 / 5.5
    pred = 530 / RPM * (0.5 * volts - drop) / (12 - drop) * R_WHEEL
    print(
        f"\nstraight, duty 0.5 on {volts:.2f} V: {v:.4f} m/s (wheel rate x radius {w_rad * R_WHEEL:.4f}; slip {err(v, w_rad * R_WHEEL):+.2f} %), "
        f"datasheet-scaled prediction {pred:.4f} ({err(v, pred):+.2f} %); lateral drift {1000 * (y1 - y0):.2f} mm over {x1 - x0:.2f} m, yaw {math.degrees(yaw):.3f} deg"
    )
    assert v == pytest.approx(w_rad * R_WHEEL, rel=0.01)  # rolling without slip
    assert v == pytest.approx(pred, rel=0.03)
    assert abs(y1 - y0) < 0.01 and abs(yaw) < math.radians(0.5)


def test_velocity_mode_straight_line(model):
    sim = start(model, mode=2)
    for w in W:
        sim.set(w, "qpps", 10600)
        sim.set(w, "vel_p", 1.0)
        sim.set(w, "vel_i", 4.0)
        sim.set(w, "vel_d", 0.0)
        sim.set(w, "goal_velocity", 2000)
    sim.step(2500)
    x0, y0, _ = pose(sim)
    sim.step(3000)
    x1, y1, yaw = pose(sim)
    v = (x1 - x0) / 3.0
    want = 2000 / (CPR / (2 * math.pi * R_WHEEL))
    print(
        f"\nvelocity mode 2000 counts/s: {v:.4f} m/s (commanded {want:.4f}, {err(v, want):+.2f} %), yaw {math.degrees(yaw):.3f} deg, drift {1000 * (y1 - y0):.2f} mm"
    )
    assert v == pytest.approx(want, rel=0.02)
    assert abs(y1 - y0) < 0.01


def test_one_metre_square_by_odometry_through_napl_program(tmp_path):
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
        f"odometry end ({1000 * ox:.1f}, {1000 * oy:.1f}) mm; replay {res['replay']}"
    )
    assert res["pass"], res["fail_reasons"]
    assert final < 0.05
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["sim"]["diverged"] is False
