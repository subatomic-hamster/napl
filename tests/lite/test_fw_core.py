"""The firmware's C core (host build) against the spec's firmware section, driven through the rig: the safety-gate matrix (every violated precondition must
get no ball), the 300 ms watchdog, homing, the planner limits measured on the emitted microstep stream (independent of the core's own bookkeeping), config
store, faults. These tests always use the C core, whatever --fw-core says. Build it with tools/fw_host_test.sh."""

import pytest
from fwmcu import available, lib_path
from napl.lite import protocol as p
from rig import DT, Rig

pytestmark = pytest.mark.skipif(
    not available(), reason="firmware host library not built (tools/fw_host_test.sh)"
)


@pytest.fixture(autouse=True)
def c_core(monkeypatch):
    monkeypatch.setenv("NAPL_FW_CORE", str(lib_path()))


def no_ball(r):
    assert r.twin.shots == [] and r.plant.fire_pulses == [], "a ball was fed"


def fire_result(r, n=1):
    fid = r.gim.fire(n)
    r.run(0.05)
    return [a for a in r.acks(p.FIRE)][-1].result


def test_positive_control_a_ball_is_fed_when_every_gate_is_open():
    r = Rig().up().engage()
    assert type(r.twin).__name__ == "FwMcu"  # the rig really runs the C core
    assert fire_result(r) == p.OK
    r.run(0.5)
    assert len(r.twin.shots) == 1 and len(r.plant.fire_pulses) == 1


def arm_engaged(r, **kw):
    r.up()
    r.arm(**kw)
    r.fw = 0.8
    r.run(1.2)
    assert r.st.state == p.ENGAGING
    return r


def gate_not_armed(r):  # IDLE, never ARMed
    r.up()
    return p.BAD_STATE


def gate_armed_no_flywheel(r):
    r.up()
    r.arm()
    return p.BAD_STATE


def gate_not_spun(r):
    r.up()
    r.arm()
    r.fw = 0.8
    r.run(0.3)
    return p.NOT_SPUN


def gate_watchdog(r):
    arm_engaged(r)
    r.hb = False
    r.run(0.4)
    assert r.st.flag1(p.F1_WD_TRIPPED)
    return p.BAD_STATE


def gate_shot_cap(r):
    arm_engaged(r, cap=1)
    assert fire_result(r) == p.OK
    r.run(0.6)
    assert len(r.twin.shots) == 1 and r.st.state == p.IDLE
    r.twin.shots.clear()
    r.plant.fire_pulses.clear()  # the one legitimate ball is accounted; from here on none
    return p.BAD_STATE


def gate_burst_over_cap(r):
    arm_engaged(r, cap=2)
    return p.CAP  # n = 3 > cap


def gate_timeout(r):
    r.up()
    r.arm(timeout=2)
    r.fw = 0.8
    r.run(2.5)
    assert r.st.state == p.IDLE
    return p.BAD_STATE


def gate_inhibit(r):
    arm_engaged(r)
    r.inhibit = True
    r.run(0.1)
    return p.INHIBIT


def gate_deadman(r):
    arm_engaged(r)
    r.deadman = False
    r.run(0.1)
    return p.INHIBIT


def gate_key(r):
    arm_engaged(r)
    r.plant.key_on = False
    r.run(0.05)
    assert r.st.state == p.SAFE
    return p.BAD_STATE


def gate_estop(r):
    arm_engaged(r)
    r.plant.estop_open = True
    r.run(0.02)
    assert r.st.state == p.SAFE
    return p.BAD_STATE


def gate_tilt_min(r):
    r.up()
    r.gim.aim(0, -5.0)
    r.run(1.5)
    r.arm()
    r.fw = 0.8
    r.run(1.2)
    return p.RANGE


def gate_not_homed(r):
    r.up()
    r.arm()
    r.fw = 0.8
    r.run(1.2)
    r.twin.ax[1].homed = False
    return p.NOT_HOMED


def gate_sector(key, value):
    def setup(
        r,
    ):  # the head sits at pan 0 / tilt 5 after up(); move the fire sector so that it is outside
        r.up()
        r.gim.set_cfg(key, value)
        r.run(0.05)
        r.arm()
        r.fw = 0.8
        r.run(1.2)
        assert r.st.state == p.ENGAGING
        return p.RANGE

    return setup


GATES = {
    "pan_outside_sector": (gate_sector(p.CFG_FIRE_PAN_MIN, 10000), 1),
    "tilt_above_sector": (gate_sector(p.CFG_FIRE_TILT_MAX, 3000), 1),
    "not_armed": (gate_not_armed, 1),
    "armed_no_flywheel": (gate_armed_no_flywheel, 1),
    "not_spun": (gate_not_spun, 1),
    "watchdog": (gate_watchdog, 1),
    "shot_cap": (gate_shot_cap, 1),
    "burst_over_cap": (gate_burst_over_cap, 3),
    "arm_timeout": (gate_timeout, 1),
    "fire_inhibit": (gate_inhibit, 1),
    "deadman": (gate_deadman, 1),
    "key_off": (gate_key, 1),
    "estop": (gate_estop, 1),
    "tilt_minimum": (gate_tilt_min, 1),
    "not_homed": (gate_not_homed, 1),
}


@pytest.mark.parametrize("name", GATES)
def test_fire_refused_under_each_violated_precondition(name):
    setup, n = GATES[name]
    r = Rig()
    want = setup(r)
    got = fire_result(r, n)
    assert got == want, (p.RESULT_NAMES[got], p.RESULT_NAMES[want])
    r.run(0.8)
    no_ball(r)


def test_watchdog_trips_at_300_ms_not_before():
    r = Rig().up().engage()
    r.fw = 0.8
    t_last = r.twin.last_hb_t
    r.hb = False
    r.run(
        t_last + 0.29 - r.clock.t
    )  # 290 ms after the last heartbeat the MCU is still up
    assert r.st.state == p.ENGAGING
    r.run(0.02)
    assert r.st.state == p.SAFE and r.st.fw_duty == 0 and r.st.flag1(p.F1_WD_TRIPPED)
    t_trip = next(
        t for t, s, why in r.twin.transitions if s == p.SAFE and why == p.R_WATCHDOG
    )
    assert 0.3 < t_trip - t_last <= 0.3011, t_trip - t_last
    r.fw = 0.0
    r.hb = True
    r.run(0.5)
    assert (
        r.st.state == p.SAFE
    )  # a second of heartbeats first, then IDLE and never ARMED by itself
    r.run(0.9)
    assert r.st.state == p.IDLE


def test_wd_ms_config_is_clamped_to_300():
    r = Rig()
    r.run(0.2)
    r.gim.set_cfg(p.CFG_WD_MS, 100000)
    r.run(0.05)
    r.gim._send(p.CfgGet(p.CFG_WD_MS))
    r.run(0.05)
    assert r.gim.cfg[p.CFG_WD_MS] == 300


def test_homing_from_the_sensor_and_from_the_negative_side():
    for start in (0.0, -40.0):  # on the hall window, and 40 deg before it
        r = Rig()
        r.plant.q[0] = r.plant.goal[0] = start
        r.up(homed=False)
        r.gim.home(pan=True, tilt=False)
        r.run(6.0)
        assert r.st.flag1(p.F1_PAN_HOMED) and not r.st.flag1(p.F1_TILT_HOMED), start
        assert abs(r.plant.q[0] - (-1.0 - 3.0)) < 0.3, r.plant.q[
            0
        ]  # 3 deg backed off the hall edge (-1 deg)
        assert r.acks(p.HOME)[-1].result == p.OK and r.acks(p.HOME)[-1].info == 1
        assert any(e.code == p.EV_HOMED and e.arg == 1 for _, e in r.gim.events)


def test_homing_timeout_when_the_sensor_never_comes():
    r = Rig(q_hall=(500.0, 500.0)).up(homed=False)  # a sensor that is nowhere near
    r.gim.home()
    r.run(16.5)
    assert r.st.state == p.FAULT and r.st.fault_bits & p.FAULT_HOMING_TIMEOUT
    assert any(e.code == p.EV_FAULT and e.arg == 0 for _, e in r.gim.events)
    r.gim.clear_fault()
    r.run(0.05)
    assert r.twin.state == p.SAFE and r.twin.fault_bits == 0


def pan_grid(r):
    """Pan position in degrees on a 1 ms grid, rebuilt from the emitted microstep stream (not from anything the core reports)."""
    spd = r.twin.cfg[p.CFG_PAN_SPR] / 360.0
    ms0, ms1 = r.twin.step_log[0][0], r.twin.step_log[-1][0]
    n = {ms: a for ms, a, _ in r.twin.step_log}
    pos, out = 0, []
    for ms in range(ms0, ms1 + 1):
        pos += n.get(ms, 0)
        out.append(pos / spd)
    return out


def test_planner_vmax_amax_and_step_rate_measured_on_the_step_stream():
    r = Rig().up()
    r.gim.aim(0, 0)
    r.run(1.0)
    r.twin.step_log = []
    for _ in range(300):
        r.gim.aim(100.0, 0, vmax=(240.0, 150.0), amax=(2000.0, 1500.0))
        r.run(DT)
    assert max(abs(a) for _, a, _ in r.twin.step_log) <= 25  # <= 25 ksteps/s
    pos, win = pan_grid(r), 50  # ms
    v = [
        (pos[i + win] - pos[i]) / (win / 1000.0) for i in range(0, len(pos) - win, win)
    ]
    assert max(v) <= 240.0 * 1.005 and max(v) > 225.0, max(
        v
    )  # reaches vmax, never beyond (one step per window = 0.45 deg/s)
    acc = [abs(b - a) / (win / 1000.0) for a, b in zip(v, v[1:])]
    assert max(acc) <= 2000.0 * 1.03, max(acc)
    assert abs(pos[-1] - 100.0) < 0.3


def test_planner_step_rate_cap_and_soft_limits():
    r = Rig().up()
    r.gim.aim(0, 0)
    r.run(1.0)
    r.twin.step_log = []
    r.gim.aim(
        150.0, 0, vmax=(6000.0, 6000.0)
    )  # 6000 deg/s asked: capped by 25 ksteps/s (562 deg/s on the pan)
    r.run(1.5)
    assert max(abs(a) for _, a, _ in r.twin.step_log) <= 25
    pos = pan_grid(r)
    assert max((b - a) * 1000.0 for a, b in zip(pos, pos[1:])) <= 562.6
    assert max(pos) <= 150.5, (
        max(pos)
    )  # brakes in time (a 1 ms controller lag overshoots by < 0.5 deg at 562 deg/s; before the brake term: 20 deg)
    r.gim.aim(250.0, 0)
    r.run(2.0)
    assert abs(r.gim.deg()[0] - 170.0) < 0.1
    assert max(pan_grid(r)) <= 170.05


def test_config_save_writes_the_flash_image_and_hard_limits_need_safe():
    r = Rig()
    r.run(0.2)
    r.gim.set_cfg(p.CFG_HARD_SHOT_CAP, 3)
    r.gim._send(p.CfgSave())
    r.run(0.05)
    assert (
        r.twin.flash[p.CFG_HARD_SHOT_CAP] == 3 and r.twin.cfg[p.CFG_HARD_SHOT_CAP] == 3
    )
    r2 = Rig().up()
    r2.gim.set_cfg(p.CFG_HARD_SHOT_CAP, 99)
    r2.run(0.05)
    assert r2.acks(p.CFG_SET)[-1].result == p.BAD_STATE
    r2.gim.set_cfg(p.CFG_PUSHER_EXT_US, 9000)  # beyond any servo
    r2.run(0.05)
    assert (
        r2.acks(p.CFG_SET)[-1].result == p.BAD_ARG
        and r2.twin.cfg[p.CFG_PUSHER_EXT_US] == 1900
    )


def test_corrupt_flash_boots_defaults_into_fault_and_blank_flash_does_not():
    r = Rig(cfg_invalid=True)
    r.run(0.2)
    assert r.twin.state == p.FAULT and r.twin.fault_bits & p.FAULT_CFG_INVALID
    assert r.twin.cfg[p.CFG_WD_MS] == 300 and r.twin.cfg[p.CFG_PAN_HOME_OFF] == 0
    r.gim.clear_fault()
    r.run(0.05)
    assert r.twin.state == p.SAFE and r.twin.fault_bits == 0
    r = Rig(flash_mode=0)
    r.run(0.2)
    assert r.twin.state == p.SAFE and r.twin.fault_bits == 0


def test_bootsel_only_in_safe_with_the_magic():
    r = Rig()
    r.run(0.2)
    r.gim._send(p.Bootsel(0x12345678))
    r.run(0.02)
    assert not r.twin.bootsel and r.acks(p.BOOTSEL)[-1].result == p.BAD_ARG
    r.gim._send(p.Bootsel())
    r.run(0.02)
    assert r.twin.bootsel
    r = Rig().up()
    r.gim._send(p.Bootsel())
    r.run(0.02)
    assert not r.twin.bootsel and r.acks(p.BOOTSEL)[-1].result == p.BAD_STATE


def test_hall_stuck_fault():
    r = Rig().up()
    r.gim.aim(60.0, 5.0)
    r.run(2.0)
    r.twin.q_hall = (60.0, 0.0)  # the sensor reads "magnet" 60 deg from home: stuck
    r.run(1.0)
    assert r.twin.fault_bits & p.FAULT_HALL_STUCK and r.twin.state == p.FAULT


def test_rx_overflow_raises_a_fault():
    r = Rig()
    r.run(0.1)
    r.link.write(b"\x01" * 4000)  # a flood without delimiters inside one millisecond
    r.run(0.05)
    assert r.twin.fault_bits & p.FAULT_USB_OVERFLOW
