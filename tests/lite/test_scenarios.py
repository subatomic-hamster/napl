"""s6 / s7 safety invariants of the cuas-lite scenario set, fast variants on the kernel-free rig (apprig.AppRig: the real App + SimRP2040 twin + TruthDetector, kinematic
head): a person WALKS into the frame mid-engagement (zero balls while visible, resume only after it left and the 1.0 s hold ran out), a person stands next to a balloon,
and the dead-man / detector / E-stop / key / link faults hit while a pusher cycle is in progress (no ball after the cut + the latency of that gate). The same
invariants on the napl kernel are the s6 / s7 scenarios (examples/cuas-lite/*.scenario.toml, Referee in python/napl/lite/sim_eval.py). Whole file < 60 s."""

import math

import numpy as np
import pytest
from apprig import AppRig, balloon, person
from napl.lite import protocol as p
from napl.lite.app import AppCfg
from napl.lite.fire import FireCfg
from napl.lite.sim_eval import CUT_LATENCY_S, GRACE_S, HOLD_S, person_in_image
from napl.lite.sim_io import Mover


def cfg(cap=10, burst=1):
    return AppCfg(fire=FireCfg(shot_cap=cap, burst=burst))


def first_shot(r, limit=25.0):
    r.arm()
    t_end = r.clock.t + limit
    while r.clock.t < t_end and not r.twin.shots:
        r.run(0.05)
    assert r.twin.shots, r.app.dec.reasons


def test_person_walks_in_mid_engagement_zero_balls_while_visible_then_resumes():
    pers = person(pos=(5.0, -9.0, 0.85))
    r = AppRig(targets=[balloon(), pers], cfg=cfg()).ready()
    first_shot(r)
    n_before = r.shots
    vis, spans = False, []  # truth visibility log: [start, end]
    t0 = r.clock.t
    while pers["pos"][1] < 9.0 and r.clock.t < t0 + 25.0:
        r.run(0.02)
        pers["pos"][1] += 1.5 * 0.02
        v = person_in_image(r._truth(r.clock.t), r.det.cam, r.det.k)
        if v and not vis:
            spans.append([r.clock.t, r.clock.t])
        if v:
            spans[-1][1] = r.clock.t
        vis = v
    r.run(6.0)  # gone: the 1.0 s hold runs out, then the engagement resumes
    assert spans, "the person never entered the image: the test would be vacuous"
    assert n_before >= 1
    for t, _, _ in r.twin.shots:
        for a, b in spans:
            assert not (a + GRACE_S < t <= b + 0.006), (
                f"ball at {t:.3f} s with the person in view since {a:.3f} s"
            )
    last = spans[-1][1]
    after = [t for t, _, _ in r.twin.shots if t > last]
    assert after, "no resume after the person left"
    assert min(after) >= last + HOLD_S - 0.03, (
        f"first ball {min(after) - last:.3f} s after the person left (hold {HOLD_S} s)"
    )
    assert r.app.failed is None


def test_person_standing_next_to_a_balloon_no_ball():
    r = AppRig(
        targets=[balloon(pos=(8.0, 1.0, 0.85)), person(pos=(8.0, 1.8, 0.85))], cfg=cfg()
    ).ready()
    r.arm()
    r.run(10.0)
    assert r.twin.shots == [] and r.plant.fire_pulses == []
    assert any(t.confirmed for t in r.app.tracks), (
        "balloon never confirmed: only the person block is under test when it is"
    )
    assert "person" in r.app.dec.reasons


def apply_fault(r, kind):
    if kind == "deadman":
        r.hold = False
    elif kind == "detector":
        r.det.outages.append((r.clock.t, r.clock.t + 2.0))
    elif kind == "estop":
        r.plant.estop_open = True
    elif kind == "key":
        r.plant.key_on = False
    elif kind == "link":
        r.link.drop = True


@pytest.mark.parametrize("burst", [1, 3])
@pytest.mark.parametrize("kind", ["deadman", "detector", "estop", "key", "link"])
def test_fault_while_a_pusher_cycle_runs_feeds_no_ball_after_the_gate_latency(
    kind, burst
):
    r = AppRig(cfg=cfg(burst=burst)).ready()
    r.arm()
    t_end = r.clock.t + 25.0
    while r.twin.state != p.FIRING and r.clock.t < t_end:
        r.run(0.005)
    assert r.twin.state == p.FIRING, (
        r.app.dec.reasons
    )  # a FIRE accepted: the pusher cycle is in progress
    t_cut = r.clock.t
    n0 = r.shots
    apply_fault(r, kind)
    if kind == "link":
        r.run(0.5)
        r.link.drop = False
    r.run(6.0)
    end = t_cut + 2.0 if kind == "detector" else math.inf
    late = [t for t, _, _ in r.twin.shots if t_cut + CUT_LATENCY_S[kind] < t < end]
    assert not late, f"{kind}: balls fed at {late} after the cut at {t_cut:.3f}"
    assert len([t for t, _, _ in r.twin.shots[n0:] if t < end]) <= burst
    if kind == "estop":  # the E-stop stays pressed: SAFE for good
        assert r.twin.state == p.SAFE and r.twin.fw == 0.0 and not r.app.fc.armed
    elif (
        kind == "key"
    ):  # SAFE at once (reason key_off); with valid heartbeats it returns to IDLE (key state is irrelevant there) but never to ARMED by itself
        assert any(
            x[2] == p.R_KEY_OFF for x in r.twin.transitions
        ) and r.twin.state in (p.SAFE, p.IDLE)
        assert r.twin.fw == 0.0 and not r.app.fc.armed
    elif kind == "link":
        assert (
            any(x[2] == p.R_WATCHDOG for x in r.twin.transitions) and not r.app.fc.armed
        )  # tripped, no auto re-arm
        assert r.shots - n0 <= 1
    elif kind == "deadman":
        assert r.twin.fw == 0.0 and r.twin.state != p.FIRING


def test_mover_path_is_continuous_and_its_velocity_is_the_derivative():
    m = Mover(
        "q",
        ((5.0, -3.0, 2.0), (5.0, 3.0, 2.0)),
        4.0,
        waves=(((0, 0, 0.25), 3.0, 0.7),),
        t_start=1.0,
        kind="quad",
    )
    assert (
        np.allclose(m.pv(0.0)[0], m.pv(1.0)[0]) and not m.pv(0.5)[1].any()
    )  # waits at route[0] until t_start
    h, prev = 1e-4, None
    for t in np.arange(1.0, 8.0, 0.37):
        p0, v0 = m.pv(t)
        p1, _ = m.pv(t + h)
        assert np.allclose((p1 - p0) / h, v0, atol=2e-2), (
            t
        )  # the ping-pong reversal at t = 2.5 s is a kink, not a jump of position
        if prev is not None:
            assert np.linalg.norm(p0 - prev) < 4.0 * 0.37 + 1.0
        prev = p0
        assert -3.0 - 1e-6 <= p0[1] <= 3.0 + 1e-6
    assert (
        m.klass == "drone" and Mover("p", ((0, 0, 1),), kind="person").klass == "person"
    )


def test_half_visible_person_is_reported_by_the_stand_in_detector():
    r = AppRig(
        targets=[person(pos=(5.0, -4.5, 0.85))]
    ).ready()  # az -42 deg: the box straddles the left image edge (centre outside)
    r.run(0.5)
    meta = r._truth(r.clock.t)
    assert person_in_image(meta, r.det.cam, r.det.k)
    assert any(d.cls == "person" for d in r.det._dets(meta)), r.app.persons
