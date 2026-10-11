"""SimRP2040 against the spec's firmware section (2-3): state machine, watchdog, ARM / FIRE gating, caps, planner limits, homing, inputs."""

import math

import pytest
from napl.lite import protocol as p
from rig import DT, Rig


def test_handshake_and_status_stream():
    r = Rig()
    r.run(0.1)
    assert p.handshake_ok(r.gim.ack) and r.gim.ack.uid == r.twin.uid
    assert r.st.state == p.SAFE and r.gim.link_ok
    r2 = Rig()
    r2.gim._send(p.Hello(2, 0, 1))  # a later major: mismatch
    r2.run(0.2)
    assert r2.gim.ack.result == 1 and r2.twin.version_bad
    r2.gim.heartbeat(True, False)
    r2.run(0.1)
    assert r2.twin.refusals[-1][2] == p.BAD_STATE or r2.twin.state == p.SAFE


def test_safe_to_idle_needs_a_second_of_heartbeats_and_estop_released():
    r = Rig()
    r.run(0.9)
    assert r.st.state == p.SAFE
    r.run(0.3)
    assert r.st.state == p.IDLE
    r = Rig()
    r.plant.estop_open = True
    r.run(1.5)
    assert r.st.state == p.SAFE and r.st.flag1(p.F1_ESTOP)
    r.plant.estop_open = False
    r.run(1.2)
    assert r.st.state == p.IDLE


def test_homing_sets_flags_and_zero():
    r = Rig().up()
    st = r.st
    assert st.flag1(p.F1_PAN_HOMED) and st.flag1(p.F1_TILT_HOMED)
    assert (
        abs(r.gim.deg()[0]) < 0.05 and abs(r.gim.deg()[1] - 5.0) < 0.05
    )  # up() aimed (0, 5) after homing
    assert (
        abs(r.plant.q[0]) < 0.1
    )  # home-frame 0 is the hall window centre (home_off calibrated)
    assert [t for t, s, why in r.twin.transitions if why == p.R_LINK_UP]


def test_arm_preconditions():
    r = Rig().up(homed=False)
    r.arm()
    assert r.acks(p.ARM)[-1].result == p.NOT_HOMED
    r = Rig().up()
    r.deadman = False
    r.run(0.1)
    r.arm()
    assert r.acks(p.ARM)[-1].result == p.INHIBIT
    r = Rig().up()
    r.plant.key_on = False
    r.run(0.05)
    r.arm()
    assert (
        r.acks(p.ARM)[-1].result == p.BAD_STATE and r.twin.state == p.SAFE
    )  # key off sent the MCU to SAFE (reason 4): no ARM
    r = Rig().up()
    r.arm(cap=200, timeout=60000)  # the host cannot raise the hard values
    assert r.st.state == p.ARMED and r.st.shot_cap == 10 and 0 < r.st.arm_left_s <= 120
    assert r.acks(p.ARM)[-1].result == p.OK


def test_watchdog_trips_within_301_ms_and_does_not_rearm():
    r = Rig().up().engage()
    r.fw = 0.8
    t_last = r.twin.last_hb_t
    r.hb = False
    r.run(0.25)
    assert r.st.state == p.ENGAGING
    r.run(0.1)
    assert r.st.state == p.SAFE and r.st.flag1(p.F1_WD_TRIPPED) and r.st.fw_duty == 0
    t_trip = next(
        t for t, s, why in r.twin.transitions if s == p.SAFE and why == p.R_WATCHDOG
    )
    assert t_trip - t_last <= 0.3011
    r.fw = 0.0
    r.hb = True
    r.run(1.3)
    assert r.st.state == p.IDLE and not r.st.flag1(
        p.F1_WD_TRIPPED
    )  # link restored: IDLE, not ARMED
    assert r.twin.shot_cap == 0


def test_comms_drop_is_a_watchdog_trip():
    r = Rig().up().engage()
    r.link.drop = True
    r.run(0.4)
    assert r.twin.state == p.SAFE


def fire_ok(r):
    fid, ack = r.fire()
    return ack.result


@pytest.mark.parametrize(
    "name",
    [
        "state_armed_only",
        "inhibit",
        "deadman",
        "not_spun",
        "tilt_below_min",
        "bad_n",
        "not_homed",
        "estop",
        "key_off",
        "hb_lost",
    ],
)
def test_fire_refused_under_each_violated_precondition(name):
    r = Rig().up()
    if name == "state_armed_only":
        r.arm()  # ARMED, flywheel never requested
        assert fire_ok(r) == p.BAD_STATE
    elif name == "not_spun":
        r.arm()
        r.fw = 0.8
        r.run(0.2)
        assert fire_ok(r) == p.NOT_SPUN
    elif name == "tilt_below_min":
        r.gim.aim(0, -5.0)
        r.run(1.5)
        r.engage()
        assert fire_ok(r) == p.RANGE
    elif name == "not_homed":
        r.engage()
        r.twin.ax[0].homed = False  # lost steps cleared it after arming
        assert fire_ok(r) == p.NOT_HOMED
    else:
        r.engage()
        if name == "inhibit":
            r.inhibit = True
            r.run(0.1)
            assert fire_ok(r) == p.INHIBIT
        elif name == "deadman":
            r.deadman = False
            r.run(0.1)
            assert fire_ok(r) == p.INHIBIT
        elif name == "bad_n":
            r.gim._send(p.Fire(77, 4))
            r.run(0.05)
            assert r.acks(p.FIRE)[-1].result == p.BAD_ARG
        elif name == "estop":
            r.plant.estop_open = True
            r.run(0.01)
            assert r.twin.state == p.SAFE
            assert fire_ok(r) == p.BAD_STATE
        elif name == "key_off":
            r.plant.key_on = False
            r.run(0.01)
            assert r.twin.state == p.SAFE
            assert fire_ok(r) == p.BAD_STATE
        elif name == "hb_lost":
            r.hb = False
            r.run(0.4)
            assert fire_ok(r) == p.BAD_STATE
    assert (
        r.twin.shots == [] and r.plant.fire_pulses == []
    )  # no ball was fed, whichever gate said no


def test_fire_accepted_when_everything_holds_and_dedups_retransmits():
    r = Rig().up().engage()
    fid, ack = r.fire()
    assert ack.result == p.OK
    r.gim._send(
        p.Fire(fid, 1)
    )  # retransmission of the same fire_id: the old answer, nothing shot
    r.run(0.6)
    assert len(r.twin.shots) == 1 and len(r.plant.fire_pulses) == 1
    assert [a.result for a in r.acks(p.FIRE)] == [p.OK, p.OK]
    ev = [e for _, e in r.gim.events if e.code == p.EV_SHOT]
    assert len(ev) == 1 and ev[0].arg >> 8 == fid
    assert r.st.shots_arm == 1 and r.st.state == p.ENGAGING


def test_shot_interval_cap_and_timeout():
    r = Rig().up()
    r.arm(cap=2)
    r.fw = 0.8
    r.run(1.2)
    fid, ack = r.fire(1, wait=0.02)
    assert ack.result == p.OK
    r.gim.fire(1)  # immediately again: busy (a pusher cycle is in progress)
    r.run(0.05)
    assert r.acks(p.FIRE)[-1].result == p.BUSY
    r.run(0.5)
    fid, ack = r.fire(1, wait=0.6)
    assert ack.result == p.OK
    assert (
        r.twin.state == p.IDLE and r.twin.transitions[-1][2] == p.R_SHOT_CAP
    )  # cap reached: disarmed
    assert len(r.twin.shots) == 2
    r.gim.fire(1)
    r.run(0.05)
    assert r.acks(p.FIRE)[-1].result == p.BAD_STATE and len(r.twin.shots) == 2
    r = Rig().up()
    r.arm(cap=5, timeout=2)
    r.run(2.1)
    assert r.twin.state == p.IDLE and r.twin.transitions[-1][2] == p.R_ARM_TIMEOUT


def test_cap_check_refuses_a_burst_that_would_exceed_it():
    r = Rig().up()
    r.arm(cap=2)
    r.fw = 0.8
    r.run(1.2)
    fid, ack = r.fire(3)
    assert ack.result == p.CAP and r.twin.shots == []


def test_burst_aborts_when_the_app_inhibits_mid_burst():
    r = Rig().up().engage()
    r.gim.fire(3)
    r.run(0.15)
    r.inhibit = True
    r.run(1.5)
    assert len(r.twin.shots) <= 1 and r.twin.state in (p.ENGAGING, p.ARMED)


def test_flywheel_lapses_without_refresh_and_with_deadman_low():
    r = Rig().up().engage()
    r.fw = 0.0
    r.gim.flywheel(0.8)
    r.run(0.7)  # one request, never refreshed: back to ARMED after 500 ms
    assert r.st.state == p.ARMED
    r = Rig().up().engage()
    r.deadman = False
    r.run(1.2)
    assert r.st.state == p.ARMED and r.twin.fw_req == 0


def test_new_session_disarms():
    r = Rig().up().engage()
    r.gim.session = 99
    r.gim.hello()
    r.run(0.05)
    assert (
        r.twin.state == p.SAFE
        and r.twin.transitions[-1][2] == p.R_NEW_SESSION
        and r.twin.shots_arm == 0
    )


def test_planner_respects_vmax_amax_soft_limits_and_step_rate():
    r = Rig().up()
    r.gim.aim(0, 0)
    r.run(1.0)
    r.gim.aim(100.0, 0, vmax=(240.0, 150.0), amax=(2000.0, 1500.0))
    v = []
    for _ in range(300):
        r.run(DT)
        r.gim.aim(100.0, 0, vmax=(240.0, 150.0), amax=(2000.0, 1500.0))
        v.append(r.twin.ax[0].v)
    assert max(v) <= 240.001 and max(v) > 235  # reaches vmax, never beyond
    assert (
        max(abs(b - a) / DT for a, b in zip(v, v[1:])) <= 2000 * 1.001
    )  # acceleration limit
    assert abs(r.gim.deg()[0] - 100.0) < 0.3
    r.gim.aim(250.0, 0)
    r.run(2.0)
    assert abs(r.gim.deg()[0] - 170.0) < 0.1  # clipped to the soft limit (cfg 4)
    r.gim.aim(
        0, 0, vmax=(5000.0, 5000.0)
    )  # a vmax above the 25 ksteps/s cap is cut: 25000 / 16000 * 360 = 562 deg/s
    r.run(0.05)
    assert abs(r.twin.ax[0].v) <= 562.6


def test_aim_refused_in_safe_and_stale_aim_holds():
    r = Rig()
    r.run(0.2)
    r.gim.aim(10, 0)
    r.run(0.05)
    assert r.acks(p.AIM)[-1].result == p.BAD_STATE
    r = Rig().up()
    r.gim.aim(30, 0, pan_vff=50.0)
    r.run(2.0)
    p1 = r.gim.deg()[0]
    r.run(1.0)  # no further AIM: after 1 s the axis holds
    assert abs(r.gim.deg()[0] - p1) < 0.2 and not math.isnan(p1)


def test_config_rules():
    r = Rig()
    r.run(0.2)
    r.gim.set_cfg(p.CFG_WD_MS, 5000)
    r.gim._send(p.CfgGet(p.CFG_WD_MS))
    r.run(0.05)
    assert r.gim.cfg[p.CFG_WD_MS] == 300  # clamped to <= 300
    r.gim.set_cfg(p.CFG_HARD_SHOT_CAP, 3)
    r.gim._send(p.CfgSave())
    r.run(0.05)
    assert r.twin.flash[p.CFG_HARD_SHOT_CAP] == 3
    r2 = Rig().up()
    r2.gim.set_cfg(p.CFG_HARD_SHOT_CAP, 99)
    r2.run(0.05)
    assert (
        r2.acks(p.CFG_SET)[-1].result == p.BAD_STATE
        and r2.twin.cfg[p.CFG_HARD_SHOT_CAP] == 10
    )  # in IDLE the hard limit is read-only
    r2.gim._send(p.Aim(1, 2, 3, 4, 0, 0, 0, 0))
    r2.run(0.05)
    assert r2.acks(p.AIM)[-1].result == p.BAD_ARG
    r2.gim._send(p.CfgGet(200))
    r2.run(0.05)
    assert r2.acks(p.CFG_GET)[-1].result == p.BAD_ARG


def test_unknown_command_and_bad_frames_are_counted():
    r = Rig()
    r.run(0.1)
    r.link.write(p.encode_raw(0x7E, 9, b""))
    r.link.write(b"\x05\x11\x22\x00")  # bad COBS
    r.run(1.2)
    assert r.acks(0x7E)[-1].result == p.UNKNOWN_CMD
    assert any(e.code == p.EV_BAD_FRAMES for _, e in r.gim.events)


def test_lost_steps_clear_homed_and_fault():
    r = Rig().up()
    r.plant.tau = 1e9  # the joint stops following the commanded position ...
    r.plant.q[0] += 5.0  # ... and is 5 deg off while the step count says otherwise
    r.run(
        0.1
    )  # the twin faults at once; the firmware (spec 2.2) after DIAG has been high for 50 ms
    assert (
        not r.twin.ax[0].homed
        and r.twin.fault_bits & p.FAULT_DRIVER_DIAG
        and r.twin.state == p.FAULT
    )
    r.gim.clear_fault()
    r.run(0.05)
    assert r.twin.state == p.SAFE and r.twin.fault_bits == 0
