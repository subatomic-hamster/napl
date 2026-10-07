"""F3 fault injection without PX4: each fault kind is a device attr set through the bus (tests/data/devices/all.xml), checked against
the physics it must change; the faults stay in the replay log and the replayed hash matches; the scenario `[[faults]]` table is
validated; a Feetech scenario with a servo_overload fault and a `fault:<name>` monitor phase; the SO-101 comms-drop example.
PX4 scenarios: tests/python/test_faults_px4.py. Needs build/napl-replay for the hash checks (skipped without).
Run: tools/remote.sh run --nolock '.venv/bin/python -m pytest tests/python/test_faults.py -v -s'"""

from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
ALL = REPO / "tests" / "data" / "devices" / "all.xml"
ARM3 = REPO / "tests" / "data" / "feetech" / "arm3.xml"
FAULTS = REPO / "examples" / "faults"


def q(s, path, attr):
    return s.query(path, attr)[0]


def pair(setup, steps=60, then=1):
    """Two identical sims, esc1 at 0.45 throttle for `steps`; the second one gets `setup(sim)` and both advance `then` more steps."""
    out = []
    for fault in (False, True):
        s = napl.Sim.load(ALL)
        s.set("/devices/esc1", "throttle", 0.45)
        s.step(steps)
        if fault:
            setup(s)
        s.step(then)
        out.append(s)
    return out


def test_battery_cell_fault_sags_the_pack_by_i_times_r_and_by_lost_emf():
    base, weak = pair(lambda s: s.set("/devices/pack", "r_fault", 0.05))
    i = q(weak, "/devices/pack", "current")[0]
    sag = (
        q(base, "/devices/pack", "voltage")[0] - q(weak, "/devices/pack", "voltage")[0]
    )
    print(
        f"\n[battery] {i:.1f} A, extra 50 mOhm: sag {sag:.3f} V (I R = {i * 0.05:.3f})"
    )
    assert i > 3 and sag == pytest.approx(i * 0.05, rel=0.2)
    base, dead = pair(lambda s: s.set("/devices/pack", "ocv_offset", -1.0))
    sag = (
        q(base, "/devices/pack", "voltage")[0] - q(dead, "/devices/pack", "voltage")[0]
    )
    assert 0.8 < sag < 1.0  # the lost EMF, less what the lower voltage saves in current


def test_motor_out_stops_the_drive_and_the_rotor_spins_down():
    base, out = pair(lambda s: s.set("/devices/esc1", "fault_open", 1), then=400)
    w0, w1 = q(base, "/devices/esc1", "omega")[0], q(out, "/devices/esc1", "omega")[0]
    print(
        f"\n[motor_out] omega {w0:.0f} rad/s healthy, {w1:.0f} rad/s open circuit; phase current {q(out, '/devices/esc1', 'current')[0]} A"
    )
    assert (
        q(out, "/devices/esc1", "current")[0] == 0
        and q(out, "/devices/esc1", "torque")[0] <= 0
    )
    assert (
        w1 < 0.7 * w0 and w1 > 0
    )  # coasting down on the prop torque, not stopped dead
    assert (
        q(out, "/devices/prop1", "thrust")[0]
        < 0.6 * q(base, "/devices/prop1", "thrust")[0]
    )


def test_prop_damage_scales_thrust_and_torque():
    base, hurt = pair(lambda s: s.set("/devices/prop1", "damage", 0.4))
    for attr in ("thrust", "torque"):
        r = q(hurt, "/devices/prop1", attr)[0] / q(base, "/devices/prop1", attr)[0]
        assert r == pytest.approx(0.6, abs=0.02), (attr, r)


def test_gnss_outage_holds_the_output_and_jump_steps_the_position():
    def alts(setup):
        s = napl.Sim.load(ALL)
        s.step(1000)
        setup(s)
        s.step(1000)
        return s

    healthy = alts(lambda s: None)
    lost = alts(lambda s: s.set("/devices/gnss", "fix_lost", 1))
    held = q(lost, "/devices/gnss", "alt")[0]
    s = napl.Sim.load(ALL)
    s.step(1000)
    at_fault = q(s, "/devices/gnss", "alt")[0]
    print(
        f"\n[gnss_outage] alt at fault {at_fault:.2f} m, held {held:.2f}, healthy 1 s later {q(healthy, '/devices/gnss', 'alt')[0]:.2f}"
    )
    assert (
        held == pytest.approx(at_fault, abs=0.01)
        and abs(q(healthy, "/devices/gnss", "alt")[0] - at_fault) > 3.0
    )
    jumped = alts(
        lambda s: s.set("/devices/gnss", "jump", [0.0, 100.0, 0.0])
    )  # 100 m north
    dlat = q(jumped, "/devices/gnss", "lat")[0] - q(healthy, "/devices/gnss", "lat")[0]
    assert dlat * 111_200.0 == pytest.approx(
        100.0, abs=3.0
    )  # same noise stream: only the step differs


def test_baro_drift_ramps_the_pressure_and_imu_bias_step_offsets_the_gyro():
    base, drift = pair(
        lambda s: s.set("/devices/baro", "drift_rate", 50.0), then=1000
    )  # 1 s
    d = (
        q(drift, "/devices/baro", "pressure")[0]
        - q(base, "/devices/baro", "pressure")[0]
    )
    print(
        f"\n[baro_drift] 50 Pa/s for 1 s: {d:.1f} Pa; bias_extra {q(drift, '/devices/baro', 'bias_extra')[0]:.1f}"
    )
    assert d == pytest.approx(50.0, abs=3.0)
    base, step = pair(
        lambda s: s.set(
            "/devices/imu", "gyro_bias", q(s, "/devices/imu", "gyro_bias") + [0.1, 0, 0]
        ),
        then=50,
    )
    dg = q(step, "/devices/imu", "gyro") - q(base, "/devices/imu", "gyro")
    assert dg[0] == pytest.approx(0.1, abs=0.03) and np.abs(dg[1:]).max() < 0.03


def test_servo_overload_pushes_and_servo_stall_seizes_the_joint():
    def pan(setup):
        s = napl.Sim.load(ALL)
        s.set(
            "/devices/gimbal_pan", "torque_enable", 0
        )  # open phase: only the fault torque acts
        setup(s)
        s.set("/devices/gimbal_pan", "ext_torque", 0.1)
        s.step(200)
        return q(s, "/devices/gimbal_pan", "position")[0]

    free, jammed = (
        pan(lambda s: None),
        pan(lambda s: s.set("/devices/gimbal_pan", "jam", 1)),
    )
    print(
        f"\n[servo] 0.1 N m for 0.2 s: free joint {free:.3f} rad, jammed {jammed:.5f} rad"
    )
    assert free > 0.05 and abs(jammed) < 0.005


def test_faults_are_in_the_replay_log_and_the_hash_matches():
    s = napl.Sim.load(ALL, host=True)  # a host Sim keeps the replay log
    s.set("/devices/esc1", "throttle", 0.45)
    s.step(100)
    for path, attr, v in (("/devices/esc1", "fault_open", [1]), ("/devices/gnss", "fix_lost", [1]),
                          ("/devices/pack", "r_fault", [0.05]), ("/devices/prop2", "damage", [0.3])):  # fmt: skip
        s.set(path, attr, v)
        s.step(50)
    log = s.log()
    assert b"fault_open" in log and b"fix_lost" in log
    out = Path(pytest.importorskip("tempfile").mkdtemp()) / "f.naplog"
    out.write_bytes(log)
    rp = program.replay_hash(out, ALL.parent)
    if rp is None:
        pytest.skip("napl-replay missing")
    assert rp == (s.tick, f"{s.state_hash():016x}")


@pytest.mark.parametrize(
    "faults,msg",
    [
        ('kind = "nope"\nat = 1', "kind"),
        ('kind = "gnss_outage"\ntarget = "/devices/g"', "exactly one of at / after"),
        ('kind = "gnss_outage"\nat = 1', "target"),
        (
            'kind = "gnss_outage"\ntarget = "/devices/g"\nat = 1\nparams = { bogus = 1 }',
            "unknown params",
        ),
        ('kind = "comms_drop"\nafter = "program+x"', "after ="),
        ('kind = "motor_out"\ntarget = "/devices/e"\nat = 1\nduration = 0', "duration"),
        (
            'kind = "imu_bias_step"\ntarget = "/devices/imu"\nat = 1',
            "gyro and / or accel",
        ),
    ],
)
def test_fault_table_is_validated(tmp_path, faults, msg):
    f = tmp_path / "t.scenario.toml"
    f.write_text(
        f'[model]\nmjcf = "{ARM3}"\n[interfaces.px4]\nbridge = "b.toml"\n[program]\ncmd = ["x"]\n[[faults]]\n{faults}\n'
    )
    with pytest.raises(program.ScenarioError, match=msg):
        program.load_scenario(f)

    f.write_text(
        f.read_text().split("[[faults]]")[0]
        + '[[faults]]\nkind = "comms_drop"\nat = 1\n'
    )
    with pytest.raises(
        program.ScenarioError, match="needs a feetech / dynamixel"
    ):  # a px4-only scenario has no serial bus
        program.load_scenario(f)


def test_scenario_fault_with_a_fault_phase_monitor_and_replay(tmp_path):
    """arm3, servo s1 holds 0; at 1 s a 0.4 N m external torque pushes it for 1 s; the program just sleeps 3.5 s."""
    prog = tmp_path / "p.py"
    prog.write_text("import time\nprint('GO', flush=True)\ntime.sleep(3.5)\n")
    f = tmp_path / "t.scenario.toml"
    f.write_text(f"""name = "t"
[model]
mjcf = "{ARM3}"
[interfaces.feetech]
servos = {{ 1 = "/devices/s1", 2 = "/devices/s2", 3 = "/devices/s3" }}
[[setup]]
path = "/devices/s1"
attr = "torque_enable"
values = [1]
[program]
cmd = ["python", "{prog}"]
[limits]
wall_s = 30
[[faults]]
name = "push"
kind = "servo_overload"
target = "/devices/s1"
params = {{ torque_nm = 0.4 }}
after = "program+1.0"
log_marker = "GO"
duration = 1.0
[[faults]]
name = "never"
kind = "servo_stall"
target = "/devices/s2"
after = "program+60"
[[monitors]]
name = "pushed_aside"
path = "/devices/s1"
attr = "position"
phase = "fault:push"
to = 1.0
op = "eventually"
lo = 0.01
[[monitors]]
name = "holds_before"
path = "/devices/s1"
attr = "position"
phase = "program"
to = 0.9
op = "always"
lo = -0.005
hi = 0.005
[[monitors]]
name = "back_after"
path = "/devices/s1"
attr = "position"
phase = "fault:push"
from = 1.5
to = 2.2
op = "always"
lo = -0.01
hi = 0.01
""")
    res, d = program.run(f, tmp_path)
    by = {m["name"]: m for m in res["monitors"]}
    ft = res["faults"][0]
    print(f"\n[scenario] fault push t={ft['start_t']:.3f}..{ft['end_t']:.3f} s; pushed max {by['pushed_aside']['max']:.4f} rad; "
          f"after {by['back_after']['min']:.4f}..{by['back_after']['max']:.4f}")  # fmt: skip
    assert (
        by["pushed_aside"]["pass"]
        and by["holds_before"]["pass"]
        and by["back_after"]["pass"]
    ), by
    assert (
        ft["end_t"] - ft["start_t"] == pytest.approx(1.0, abs=0.05)
        and ft["error"] is None
    )
    # the stall on s2 was never reached (program ended first): reported as a failed monitor, so the run fails
    assert by["fault_never_applied"]["pass"] is False and not res["pass"]
    assert res["replay"].get("match", True)
    assert b"ext_torque" in (d / "replay.naplog").read_bytes()


def test_so101_comms_drop_example(tmp_path):
    res, d = program.run(FAULTS / "so101_comms_drop.scenario.toml", tmp_path)
    log = (d / "program.log").read_text()
    ft = res["faults"][0]
    print(
        f"\n[comms_drop] {log.strip().splitlines()[-1]}; fault {ft['start_t']:.2f}..{ft['end_t']:.2f} s"
    )
    for m in res["monitors"]:
        print(
            f"[comms_drop]   {m['name']}: pass={m['pass']} min {m.get('min', 0):.4g} max {m.get('max', 0):.4g} final {m.get('final', 0):.4g}"
        )
    assert res["pass"], res["fail_reasons"]
    n = int(log.split("done: ")[1].split(" read timeouts")[0])
    assert n >= 5, (
        log
    )  # the user program saw the timeouts (50 Hz cycles, 0.5 s down) and recovered (it exits 1 otherwise)
    assert ft["end_t"] - ft["start_t"] == pytest.approx(0.5, abs=0.05)
    assert res["replay"].get("match", True)
