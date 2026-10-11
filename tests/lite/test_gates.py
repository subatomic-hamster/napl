"""Every gate of the safety table (spec 3.2), exercised through the whole app against the SimRP2040 twin: each test violates ONE precondition and the twin
must not feed a ball (twin.shots, plant.fire_pulses empty). test_control_fires proves the rig can fire at all, so the "no FIRE" results are not vacuous."""

import pytest
from apprig import AppRig, balloon, person
from napl.lite import protocol as p
from napl.lite.aim import ClassTracker
from napl.lite.app import AppCfg
from napl.lite.fire import FireCfg, Sector
from napl.lite.io import Det
from napl.lite.sim_io import BRIO


def assert_no_fire(r):
    assert r.twin.shots == [] and r.plant.fire_pulses == [], r.app.dec.reasons


def armed_rig(**kw):
    r = AppRig(**kw).ready()
    r.arm()
    return r


def test_control_fires_in_session_arm_mode_with_no_per_track_click():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0) >= 1
    assert r.plant.fire_pulses and r.app.fc.cfg.per_track_auth is False
    assert not r.app.fc.auth  # nothing was authorized by the operator: session arm engaged the confirmed in-sector track by itself
    assert r.st.shots_arm <= 5


def test_not_armed_never_fires():
    r = AppRig().ready()
    r.run(8.0)
    assert_no_fire(r)
    assert r.st.state == p.IDLE and "not_armed" in r.app.dec.reasons


def test_person_in_frame_blocks_firing():
    r = AppRig(targets=[balloon(), person()]).ready()
    r.arm()
    r.run(8.0)
    assert_no_fire(r)
    assert "person" in r.app.dec.reasons and r.app._inhibit(r.clock.t)


def test_person_far_edge_of_frame_blocks_firing():
    r = AppRig(
        targets=[balloon(), person(pos=(6.0, -3.2, 0.9))]
    ).ready()  # near the image edge, nowhere near the balloon
    r.arm()
    r.run(8.0)
    assert_no_fire(r)


def test_person_overlapping_the_target_blocks_even_without_the_time_hold():
    r = AppRig(targets=[balloon(), person(pos=(8.0, 1.0, 0.85))]).ready()
    r.arm()
    r.run(6.0)
    assert_no_fire(r)


def test_person_seen_once_blocks_for_a_second_after():
    r = AppRig().ready()
    r.arm()
    r.run(2.0)
    shots0 = r.shots
    seen = {"n": 0}

    def inject(dets):
        if seen["n"] == 0:  # one single reply with a person, then clean again
            seen["n"] = 1
            dets = dets + [Det("person", 0.3, (10, 10, 40, 100))]
        return dets

    r.det.mutate = inject
    r.run(0.3)
    t0, n0 = r.clock.t, r.shots
    r.run(0.6)
    assert (
        r.shots == n0 or r.clock.t - t0 > 1.0
    )  # no ball while the 1.0 s hold from the single sighting runs
    assert r.app.person_t > 0 and shots0 <= r.shots


def test_stale_detector_inhibits_and_resumes():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0)
    r.det.outages = [(r.clock.t, r.clock.t + 3.0)]
    r.run(0.4)  # 250 ms of staleness + the in-flight burst
    n = r.shots
    r.run(2.5)
    assert r.shots == n, "a ball was fed during a detector outage"
    assert "detector_stale" in r.app.dec.reasons or r.app.dec.inhibit
    assert r.app._inhibit(r.clock.t)


def test_no_detector_replies_at_all_never_fires():
    r = AppRig(detector_kw=dict(outages=[(0.0, 1e9)])).ready()
    r.arm()
    r.run(6.0)
    assert_no_fire(r)


def test_socket_detector_down_is_stale_forever(tmp_path):
    from napl.lite.detector_client import SocketDetector
    from napl.lite.sim_io import ManualClock

    d = SocketDetector(
        ManualClock(),
        path=str(tmp_path / "nope.sock"),
        frames_path=str(tmp_path / "frames"),
    )
    assert d.stale_s == float("inf") and d.poll() is None


def test_unconfirmed_track_is_not_engaged():
    r = AppRig(targets=[balloon()]).ready()
    r.det.mutate = lambda dets: [
        Det(d.cls, 0.4, d.xyxy) for d in dets
    ]  # conf < 0.5: never confirmed
    r.arm()
    r.run(6.0)
    assert_no_fire(r)
    assert not any(t.confirmed for t in r.app.tracks)


def test_non_target_class_is_not_engaged():
    r = AppRig(targets=[balloon(cls="bird")]).ready()
    r.det.mutate = lambda dets: [Det("bird", 0.9, d.xyxy) for d in dets]
    r.arm()
    r.run(5.0)
    assert_no_fire(r)


def test_five_consecutive_replies_confirm_not_four():
    tr = ClassTracker(BRIO)
    from napl.lite.aim import DetBlob, Geometry

    geom = Geometry()
    pose = geom.pose(0, 5)
    for k in range(5):
        tracks = tr.update(
            [DetBlob.from_det(Det("balloon", 0.9, (600, 340, 620, 360)))],
            tr.frame(pose, k * 0.017),
        )
        assert tracks[0].confirmed == (k == 4)


@pytest.mark.parametrize(
    "sector",
    [Sector(10, 60, 0, 60), Sector(-60, 60, 20, 60), Sector(-60, 60, 0, 60, r_max=5.0)],
)
def test_geofence_outside_the_sector_never_fires(sector):
    r = AppRig(sector=sector).ready()  # the balloon is at az ~7 deg, el ~5 deg, 8 m
    r.arm()
    r.run(8.0)
    assert_no_fire(r)
    assert (
        "target_outside_sector" in r.app.dec.reasons or "no_target" in r.app.dec.reasons
    )


def test_no_sector_configured_never_fires():
    r = AppRig(sector=None).ready()
    r.arm()
    r.run(6.0)
    assert_no_fire(r)
    assert "no_sector" in r.app.dec.reasons


def test_sector_rejects_aiming_below_the_horizon():
    with pytest.raises(ValueError):
        Sector(-60, 60, -5, 60)


def test_barrel_outside_sector_never_fires():
    r = AppRig(
        sector=Sector(-60, 60, 4.0, 60)
    ).ready()  # target el ~5.0 inside, but the head sits at tilt ~0 -> commanded barrel must be inside too
    r.app.fc.sector = Sector(
        -60, 60, 0.0, 3.0
    )  # target (5 deg) outside -> no target; and any barrel solution is above 3 deg
    r.run(
        1.0
    )  # the app writes the new sector to the MCU (ARM is refused until it reads back)
    assert r.app.sector_synced()
    r.arm()
    r.run(6.0)
    assert_no_fire(r)
    assert r.app.fc.armed or "arm refused" not in (r.app.last_refusal or "")


def test_arm_is_refused_until_the_mcu_holds_the_sector():
    r = AppRig().ready()
    r.app.fc.sector = Sector(-30, 30, 0, 45)
    r.arm()  # the sector on the MCU is still the old one
    assert not r.app.fc.armed and "sector" in r.app.last_refusal
    r.run(1.0)
    assert r.app.sector_synced()
    assert [
        r.twin.cfg[k]
        for k in (
            p.CFG_FIRE_PAN_MIN,
            p.CFG_FIRE_PAN_MAX,
            p.CFG_FIRE_TILT_MIN,
            p.CFG_FIRE_TILT_MAX,
        )
    ] == [-30000, 30000, 0, 45000]
    r.arm()
    assert r.app.fc.armed


def test_burst_of_three_feeds_three_balls_of_one_fire():
    cfg = AppCfg()
    cfg.fire.burst = 3
    r = AppRig(cfg=cfg).ready()
    assert r.engage_until_shot(10.0) >= 1
    r.run(2.0)
    first = r.twin.shots[0][1]
    assert [(f, i) for _, f, i in r.twin.shots[:3]] == [
        (first, 0),
        (first, 1),
        (first, 2),
    ]


def test_burst_stops_at_the_next_ball_when_a_person_appears():
    cfg = AppCfg()
    cfg.fire.burst = 3
    r = AppRig(cfg=cfg).ready()
    r.arm()
    while r.clock.t < 30.0 and not r.shots:
        r.run(0.005)
    assert r.shots == 1
    r.targets.append(person(pos=(6.0, -0.5, 0.9)))  # in the frame, beside the balloon
    r.run(2.0)
    assert r.shots == 1, r.twin.shots


def test_fence_clamps_the_aim_inside_the_sector_with_a_braking_margin():
    from napl.lite.fire import FireControl, View

    fc = FireControl(FireCfg(), None, Sector(-60, 60, 0, 60))
    fc.armed = True
    fc.rate = (
        120.0,
        0.0,
    )  # slewing at 120 deg/s: braking 120^2 / 4000 = 3.6 deg (+ one period of acceleration) + 0.5 deg
    v = View(0.0, None, True, 0.0, 0.0, [], 0.0, [], 0.0, 0.0, 0.0, 0.0)
    pan, tilt, vp, vt = fc.fence((70.0, 30.0, 40.0, 5.0), v)
    assert 55.0 < pan < 56.4 and tilt == 30.0 and vp == 0.0 and vt == 5.0
    fc.armed = False
    assert fc.fence((70.0, 30.0, 40.0, 5.0), v) == (
        70.0,
        30.0,
        40.0,
        5.0,
    )  # the operator may aim anywhere while disarmed


def test_deadman_not_held_never_fires():
    r = AppRig().ready()
    r.hold = False
    r.run(1.0)
    r.arm()
    r.run(8.0)
    assert_no_fire(r)
    assert (
        r.st.fw_duty == 0 and r.st.state == p.IDLE
    )  # the MCU refused the ARM itself (no DEADMAN_OK): ACK INHIBIT
    assert "INHIBIT" in (r.app.last_refusal or "")


def test_deadman_released_mid_engagement_stops_the_shooting():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0)
    r.hold = False
    r.run(0.8)  # 0.5 s freshness + an in-flight ball
    n = r.shots
    r.run(4.0)
    assert r.shots == n


def test_shot_cap_stops_and_disarms():
    cfg_fire = FireCfg(shot_cap=2)
    from napl.lite.app import AppCfg

    r = AppRig(cfg=AppCfg(fire=cfg_fire)).ready()
    r.arm()
    r.run(15.0)
    assert r.shots == 2 and r.st.state == p.IDLE and not r.app.fc.armed
    assert r.twin.transitions[-1][2] in (p.R_SHOT_CAP, p.R_DISARM)


def test_arm_timeout_stops_and_disarms():
    from napl.lite.app import AppCfg

    r = AppRig(cfg=AppCfg(fire=FireCfg(arm_timeout_s=1))).ready()
    r.arm()
    r.run(1.5)
    n = r.shots
    r.run(6.0)
    assert r.shots == n and r.st.state == p.IDLE and not r.app.fc.armed


def test_pointing_error_beyond_tolerance_blocks():
    r = AppRig().ready()
    r.app.fc.cfg.aim_tol_deg = (
        1e-6  # the head can never be inside this tolerance of the solution
    )
    r.arm()
    r.run(6.0)
    assert_no_fire(r)
    assert "pointing" in r.app.dec.reasons


def test_per_track_authorization_mode_needs_the_click():
    r = AppRig(per_track=True).ready()
    r.arm()
    r.run(6.0)
    assert_no_fire(r)
    assert any("unauthorized" in t["why"] for t in r.app.snapshot()["tracks"])
    tid = next(t.id for t in r.app.tracks if t.confirmed)
    r.app.post("authorize", id=tid)
    r.run(8.0)
    assert r.shots >= 1 and r.shots <= 3  # 3 rounds per authorization


def test_estop_stops_everything_in_5ms_of_ticks():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0)
    r.plant.estop_open = True
    r.run(0.006)
    n = r.shots
    assert r.twin.state == p.SAFE
    r.run(5.0)
    assert r.shots == n and r.st.fw_duty == 0 and not r.app.fc.armed


def test_key_off_stops_everything():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0)
    r.plant.key_on = False
    r.run(0.01)
    n = r.shots
    r.run(5.0)
    assert r.shots == n and r.twin.state in (p.SAFE, p.IDLE)


def test_link_loss_trips_the_mcu_within_310_ms_and_nothing_fires_after():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0)
    r.link.drop = True
    t0 = r.clock.t
    r.run(0.31)
    assert r.twin.state == p.SAFE
    assert (
        r.twin.transitions[-1][2] == p.R_WATCHDOG
        and r.twin.transitions[-1][0] - t0 <= 0.311
    )
    n = r.shots
    r.link.drop = False
    r.run(6.0)
    assert r.shots == n, "no auto re-arm: the operator must ARM again"
    assert r.twin.state == p.IDLE and not r.app.fc.armed


def test_exception_in_the_control_loop_fails_closed():
    r = AppRig().ready()
    assert r.engage_until_shot(10.0)

    def boom(*a, **k):
        raise RuntimeError("synthetic fault")

    r.app.fc.step = boom
    r.run(0.05)
    assert r.app.failed and "synthetic fault" in r.app.failed
    n = r.shots
    r.run(0.4)
    assert (
        r.twin.state == p.SAFE and r.twin.transitions[-1][2] == p.R_WATCHDOG
    )  # heartbeats stopped: the MCU tripped on purpose
    r.run(3.0)
    assert r.shots == n and r.app.failed


def test_no_calibration_refuses_arm():
    from napl.lite.app import AppCfg

    r = AppRig(cfg=AppCfg(calib_ok=False)).ready()
    r.arm()
    assert not r.app.fc.armed and "arm refused" in (r.app.last_refusal or "")
    r.run(4.0)
    assert_no_fire(r)


def test_version_mismatch_sends_no_heartbeats():
    r = AppRig()
    r.twin.version_bad = False
    r.run(0.1)
    r.app.mismatch = True
    n = r.gim.sent
    r.run(0.5)
    assert r.gim.sent - n <= 12  # no heartbeat stream (only hello retries)


def test_manual_range_overrides_size_ranging():
    r = AppRig().ready()  # not armed: the head must stay put while the range is overridden (an armed head chases the 12 m solution out of view)
    r.run(2.0)
    t = next(t for t in r.app.tracks if t.confirmed)
    size_range = t.range_m
    r.app.post("range", id=t.id, metres=12.0)
    r.run(0.5)
    t = next(x for x in r.app.tracks if x.id == t.id)
    assert t.range_m == pytest.approx(12.0) and abs(size_range - 12.0) > 1.0
    assert r.app.snapshot()["tracks"][0]["manual_range"] == 12.0
    import numpy as np

    assert (
        11.0
        < np.linalg.norm(t.pos - r.app.geom.pose(*r.plant.joint_deg()).cam_pos)
        < 13.0
    )
    r.app.post("range", id=t.id, metres=None)
    r.run(0.5)
    assert next(x for x in r.app.tracks if x.id == t.id).range_m == pytest.approx(
        size_range, rel=0.15
    )
