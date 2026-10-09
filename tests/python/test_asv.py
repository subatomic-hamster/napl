"""Autonomous surface vessel showcase (examples/asv, assemblies/asv.toml): the COLREG classification and the judge against hand-worked geometries, the LOS / heading
controller, the GNSS + IMU estimator on synthetic data, the lidar scan convention, the AIS channel, the planner's choices in textbook encounters, the assemblies (DRC
clean, build, load), and the five scenarios through `napl program` (monitors pass, the replay log reproduces the state hash)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from napl import design
from napl.design import Design

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples"
sys.path.insert(0, str(EX))

from asv import ais as ais_mod
from asv import colreg as cr
from asv import encounters as enc
from asv import judge
from asv.control import Autopilot, Gains, Route
from asv.estimator import Navigator
from asv.geo import (
    LAT0,
    LON0,
    enu_from_latlon,
    heading_deg,
    latlon_from_enu,
    psi_from_heading_deg,
    rel_bearing,
    wrap,
)
from asv.perception import Tracker, scan_points
from asv.planner import Planner

PARTS = REPO / "parts"
G = 9.80665


# ----------------------------------------------------------------------------------------------------------------------------- geometry & COLREG
def test_geo_round_trips_and_compass_conventions():
    x, y = 123.4, -56.7
    assert enu_from_latlon(*latlon_from_enu(x, y)) == pytest.approx((x, y), abs=1e-6)
    assert heading_deg(0.0) == pytest.approx(90.0)  # East
    assert heading_deg(math.pi / 2) == pytest.approx(0.0)  # North
    assert psi_from_heading_deg(180.0) == pytest.approx(-math.pi / 2)
    # a target on the right of the bow is a positive (starboard) bearing
    assert rel_bearing(0.0, (0, 0), (10, -10)) == pytest.approx(math.radians(45))
    assert rel_bearing(0.0, (0, 0), (10, 10)) == pytest.approx(-math.radians(45))


def geom(own_pos, own_vel, tgt_pos, tgt_vel):
    psi = math.atan2(own_vel[1], own_vel[0])
    return cr.geometry(own_pos, own_vel, psi, tgt_pos, tgt_vel)


@pytest.mark.parametrize(
    "tgt_pos, tgt_vel, want",
    [
        ((100, 1), (-1.5, 0), cr.HEAD_ON),  # reciprocal courses, Rule 14
        (
            (75, 75),
            (0.0, -1.5),
            cr.STAND_ON,
        ),  # from the port side (north), heading south
        (
            (75, -75),
            (0.0, 1.5),
            cr.GIVE_WAY,
        ),  # from the starboard side (south), heading north
        (
            (40, 0),
            (0.7, 0),
            cr.OVERTAKING,
        ),  # ahead of us, same course: Rule 13, we overtake
        ((-40, 0), (3.0, 0), cr.OVERTAKEN),  # astern, faster: it overtakes us
    ],
)
def test_classify_the_five_situations(tgt_pos, tgt_vel, want):
    g = geom((0, 0), (1.5, 0), tgt_pos, tgt_vel)
    assert cr.classify(g, 1.5, math.hypot(*tgt_vel)) == want


def test_classify_rule_13_sector_edge_and_roles():
    # we come up on a target heading East: it is in the overtaking sector of ours when we are > 112.5 degrees from ITS bow (we are abaft its beam by 22.5 degrees)
    v = (1.5, 0.0)
    inside = geom(
        (0, 0), v, (-5.0, 0.0), (3.0, 0.0)
    )  # astern of us (bearing 180), faster: it overtakes us
    assert cr.classify(inside, 1.5, 3.0) == cr.OVERTAKEN
    # the same pair 100 degrees off our bow is a crossing, not an overtake (inside the beam sector)
    b = math.radians(100.0)
    g = geom((0, 0), v, (30 * math.cos(-b), 30 * math.sin(-b)), (1.5, 0.0))
    assert cr.classify(g, 1.5, 1.5) == cr.GIVE_WAY
    assert (
        cr.role(cr.HEAD_ON)
        == cr.role(cr.OVERTAKING)
        == cr.role(cr.GIVE_WAY)
        == "give_way"
    )
    assert cr.role(cr.STAND_ON) == cr.role(cr.OVERTAKEN) == "stand_on"
    assert cr.role(cr.NONE) == "none"


def test_cpa_tcpa_closed_form():
    cpa, t = cr.cpa_tcpa((0, 0), (1, 0), (100, 10), (-1, 0))
    assert cpa == pytest.approx(10.0) and t == pytest.approx(50.0)
    cpa, t = cr.cpa_tcpa(
        (0, 0), (1, 0), (-20, 0), (1, 0)
    )  # same velocity: never changes
    assert cpa == pytest.approx(20.0) and math.isinf(t)
    cpa, t = cr.cpa_tcpa((0, 0), (-1, 0), (100, 0), (1, 0))  # opening
    assert t < 0 and cpa == pytest.approx(100.0)


def test_passing_side_and_astern_helpers():
    # head-on, the other vessel 3 m to the north of our track: it passes on our port side
    assert cr.passes_port_to_port((0, 0), (1.5, 0), (100, 3), (-1.5, 0))
    assert not cr.passes_port_to_port((0, 0), (1.5, 0), (100, -3), (-1.5, 0))
    # a target heading North through x = 75: we reach x = 75 at t = 50 s; it reaches our track at t = 60 s (we cross ahead of it) or at t = 40 s (it crosses ahead of us)
    assert cr.passes_astern((0, 0), (1.5, 0), (75, -90), (0.0, 1.5)) is False
    assert cr.passes_astern((0, 0), (1.5, 0), (75, -60), (0.0, 1.5)) is True


# ----------------------------------------------------------------------------------------------------------------------------------------- judge
def tracks(own_y=0.0, tgt_y=3.0, n=200, v_t=-1.5):
    t = np.linspace(0, 100, n)
    own = np.stack([1.5 * t, np.full(n, own_y)], 1)
    tgt = np.stack([150 + v_t * t, np.full(n, tgt_y)], 1)
    return t, own, np.zeros(n), np.full(n, 1.5), tgt, np.tile([v_t, 0.0], (n, 1))


def test_judge_head_on_port_to_port_and_collision():
    t, own, psi, spd, tgt, tv = tracks(tgt_y=8.0)
    v = judge.verdict("head_on", t, own, psi, spd, tgt, tv, 0.0, 1.5)
    assert (
        v["side"] == "port_to_port"
        and v["cpa_ok"]
        and v["side_ok"]
        and v["colreg_ok"]
        and not v["collision"]
    )
    assert v["cpa_m"] == pytest.approx(8.0, abs=0.3)
    t, own, psi, spd, tgt, tv = tracks(tgt_y=-8.0)  # starboard to starboard: wrong
    v = judge.verdict("head_on", t, own, psi, spd, tgt, tv, 0.0, 1.5)
    assert (
        v["side"] == "starboard_to_starboard"
        and not v["side_ok"]
        and not v["colreg_ok"]
    )
    t, own, psi, spd, tgt, tv = tracks(tgt_y=0.5)  # centres 0.5 m apart
    v = judge.verdict("head_on", t, own, psi, spd, tgt, tv, 0.0, 1.5)
    assert v["collision"] and not v["colreg_ok"] and not v["cpa_ok"]


def test_judge_port_turn_and_stand_on_limits():
    t, own, psi, spd, tgt, tv = tracks(tgt_y=8.0)
    d = np.where(
        (np.hypot(*(tgt - own).T) < 60.0), math.radians(25.0), 0.0
    )  # 25 degrees to port while the other vessel is near
    v = judge.verdict("head_on", t, own, d, spd, tgt, tv, 0.0, 1.5)
    assert not v["side_ok"] and v["max_port_deg"] == pytest.approx(25.0, abs=0.5)
    # stand-on: a 25 degree turn or a stop breaks 17(a); keeping course and speed passes
    v = judge.verdict("crossing_stand_on", t, own, psi, spd, tgt, tv, 0.0, 1.5)
    assert v["side_ok"]
    v = judge.verdict("crossing_stand_on", t, own, d, spd, tgt, tv, 0.0, 1.5)
    assert not v["side_ok"]
    slow = np.where((np.hypot(*(tgt - own).T) < 60.0), 0.5, 1.5)
    v = judge.verdict("crossing_stand_on", t, own, psi, slow, tgt, tv, 0.0, 1.5)
    assert not v["side_ok"]
    v = judge.verdict(
        "crossing_stand_on", t, own, d, spd, tgt, tv, 0.0, 1.5, complied=False
    )  # the other vessel did not give way: not judged
    assert v["side_ok"]


def test_static_clearance_is_surface_distance():
    box = [("b", "box", (10.0, 0.0, 0.0), (2.0, 3.0, 1.0), None)]
    assert judge.static_clearance((0, 0), box, [], [], 0.35) == pytest.approx(8.0)
    assert judge.static_clearance((10, 0), box, [], [], 0.35) == pytest.approx(0.0)
    assert judge.static_clearance(
        (0, 0), [], [("buoy", (3.0, 4.0), None)], [], 0.35
    ) == pytest.approx(4.65)
    assert judge.static_clearance(
        (0, 0), [], [], [enc.Obstacle("f", (6.0, 8.0))], 0.35
    ) == pytest.approx(9.6)


# ------------------------------------------------------------------------------------------------------------------------------------ control
def test_route_los_cross_track_sign_and_completion():
    r = Route([(0, 0), (100, 0)], speed=1.5)
    left = r.update(
        (20.0, 5.0)
    )  # 5 m left of the leg (North of an eastbound leg): xte positive, steer to the right (smaller yaw)
    assert left["xte"] == pytest.approx(5.0) and left["psi"] < 0.0
    right = r.update((20.0, -5.0))
    assert right["xte"] == pytest.approx(-5.0) and right["psi"] > 0.0
    assert r.update((50.0, 0.0))["psi"] == pytest.approx(0.0)
    assert not r.done
    near = r.update((98.0, 1.0))  # inside the 3 m acceptance radius
    assert r.done and near["done"]
    # a position that passed abeam of the last waypoint within twice the radius (a navigation error) also completes
    r2 = Route([(0, 0), (100, 0)], speed=1.5)
    r2.update((101.0, 5.0))
    assert r2.done
    r3 = Route([(0, 0), (100, 0)], speed=1.5)
    r3.update((101.0, 9.0))
    assert not r3.done


def test_route_advances_legs_and_slows_at_the_end():
    r = Route([(0, 0), (50, 0), (50, 50)], speed=1.5)
    assert r.update((10, 0))["leg"] == 0
    s = r.update((49.0, 0.0))
    assert s["leg"] == 1 and s["alpha"] == pytest.approx(math.pi / 2)
    end = r.update((50.0, 46.0))
    assert (
        end["u"] < 1.5 and end["u"] >= Gains().u_final
    )  # ramped down towards the last waypoint


def test_autopilot_mixer_signs_and_saturation():
    ap = Autopilot()
    port, stbd = ap.step(
        psi_cmd=0.5, u_cmd=1.5, psi=0.0, r=0.0, u=1.5, dt=0.04
    )  # wants to turn left (positive yaw): more thrust on the starboard side
    assert stbd > port
    ap2 = Autopilot()
    port, stbd = ap2.step(psi_cmd=-0.5, u_cmd=1.5, psi=0.0, r=0.0, u=1.5, dt=0.04)
    assert port > stbd
    g = Gains()
    ap3 = Autopilot()
    for _ in range(100):
        port, stbd = ap3.step(psi_cmd=3.0, u_cmd=1.5, psi=0.0, r=0.0, u=0.0, dt=0.04)
    assert max(abs(port), abs(stbd)) <= 1.0 and abs(stbd - port) <= 2 * g.d_max + 1e-9
    # feed-forward: the throttle that balances the nominal drag
    ff = ap.feed_forward(1.5)
    assert 0.0 < ff < 1.0
    assert 2 * 36.4 * ff**2 == pytest.approx(
        g.u_ff_lin * 1.5 + g.u_ff_quad * 1.5**2, rel=1e-9
    )


def test_heading_reference_is_rate_limited():
    ap = Autopilot()
    ap.reset(0.0)
    ap.step(psi_cmd=math.pi / 2, u_cmd=0.0, psi=0.0, r=0.0, u=0.0, dt=0.04)
    assert ap.psi_ref == pytest.approx(Gains().psi_rate * 0.04)


# --------------------------------------------------------------------------------------------------------------------------------- estimator
MAG_REF = np.array([0.0, 20.0, -45.0])  # uT, local field: North and down (ENU world)


def body_mag(psi):
    c, s = math.cos(psi), math.sin(psi)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])  # body -> world
    return R.T @ MAG_REF


def test_attitude_initialises_to_the_true_heading_from_accel_and_mag():
    for psi in (0.0, 0.7, -2.5, 3.0):
        nav = Navigator(MAG_REF)
        nav.imu(
            np.zeros(3), np.zeros(3), np.zeros(3), 0.008
        )  # the devices hold zeros before their first sample: no initialisation
        assert not nav.att.ready
        nav.imu(np.zeros(3), np.array([0, 0, G]), body_mag(psi), 0.008)
        assert nav.att.ready and wrap(nav.yaw - psi) == pytest.approx(0.0, abs=1e-6)


def test_navigator_converges_on_a_constant_velocity_track():
    psi, v = 0.4, np.array([1.2, 0.5])
    nav = Navigator(MAG_REF)
    rng = np.random.default_rng(3)
    t, dt = 0.0, 0.008
    next_fix = 0.0
    for _ in range(int(60 / dt)):
        nav.imu(
            rng.normal(0, 0.005, 3),
            np.array([0, 0, G]) + rng.normal(0, 0.05, 3),
            body_mag(psi),
            dt,
        )
        t += dt
        if t >= next_fix:
            next_fix += 0.2
            p = v * (t - 0.2)  # the fix is 0.2 s old
            lat, lon = latlon_from_enu(*(p + rng.normal(0, 1.0, 2)))
            nav.gnss(
                lat, lon, (v[1] + rng.normal(0, 0.05), v[0] + rng.normal(0, 0.05), 0.0)
            )  # NED
    est = nav.state()
    assert np.hypot(*(est["pos"] - v * t)) < 1.5
    assert np.hypot(*(est["vel"] - v)) < 0.1
    assert wrap(est["psi"] - psi) == pytest.approx(0.0, abs=math.radians(2))
    assert est["speed"] == pytest.approx(np.hypot(*v), abs=0.1)


# --------------------------------------------------------------------------------------------------------------------------------- perception
def test_scan_azimuth_is_positive_to_port():
    n = 1600
    r = np.full(n, np.inf)
    j = int(0.75 * n)  # a_j = 2 pi ((j + 1/2) / n - 1/2) = +90 degrees: to port
    r[j - 1 : j + 1] = 10.0
    w, b, rng, az, idx = scan_points(r, psi=0.0, pos=(0.0, 0.0))
    assert np.all(w[:, 1] > 9.9)  # North of an East-facing vessel is its port side
    assert np.all(az > 0) and np.allclose(b[:, 0], 0.45, atol=0.1)
    w, *_ = scan_points(
        r, psi=math.pi / 2, pos=(5.0, 0.0)
    )  # facing North: port is West
    assert np.all(w[:, 0] < -4.9)


def test_tracker_estimates_the_velocity_of_a_moving_contact_and_keeps_static_ones_still():
    tr = Tracker()
    truth = np.array([60.0, 10.0])
    v = np.array([-1.0, 0.2])
    for k in range(40):
        t = 0.1 * k * 5
        tr.update_lidar(
            t,
            [
                dict(
                    c=truth + v * t,
                    ext=1.2,
                    n=10,
                    rng=60.0,
                    brg=0.0,
                    pts=np.zeros((2, 2)),
                )
            ],
        )
    (m,) = tr.moving(t)
    assert np.hypot(*(m.vel - v)) < 0.15 and np.hypot(*(m.pos - (truth + v * t))) < 1.0
    still = Tracker()
    for k in range(40):
        still.update_lidar(
            0.5 * k,
            [
                dict(
                    c=np.array([30.0, 5.0]),
                    ext=0.7,
                    n=6,
                    rng=30.0,
                    brg=0.0,
                    pts=np.zeros((2, 2)),
                )
            ],
        )
    assert len(still.confirmed(20.0)) == 1 and not still.moving(20.0)
    still.update_lidar(40.0, [])  # nothing returned for 4 s: dropped
    still.update_lidar(46.0, [])
    assert not still.confirmed(46.0)


def test_tracker_ais_report_creates_a_confirmed_moving_track_at_the_reported_time():
    tr = Tracker()
    tr.update_ais(10.0, [dict(id=7, t_tx=9.0, x=100.0, y=0.0, vx=-1.5, vy=0.0)])
    (m,) = tr.moving(10.0)
    assert m.mmsi == 7 and m.pos[0] == pytest.approx(
        98.5, abs=1e-6
    )  # moved on by the report's age


# ------------------------------------------------------------------------------------------------------------------------------------------ AIS
def test_ais_rate_latency_and_loss():
    tp = ais_mod.Transponder(5, LAT0, LON0)
    ch = ais_mod.Channel(11, LAT0, LON0, latency=1.0, loss=0.0)
    sent = []
    for k in range(600):  # 60 s at 10 Hz, steady course
        t = 0.1 * k
        tp.fix(t, LAT0, LON0, (0.0, 1.5, 0.0))
        rep = tp.poll(t)
        if rep:
            sent.append(rep.t_tx)
            ch.send(rep)
    assert np.diff(sent) == pytest.approx(10.0, abs=0.11)  # Class A steady: 10 s
    assert ch.receive(sent[0] + 0.5) == []  # not there yet
    (m,) = ch.receive(sent[0] + 1.05)
    assert (
        m["id"] == 5
        and m["t_tx"] == pytest.approx(sent[0])
        and m["vx"] == pytest.approx(1.5, abs=0.05)
    )
    lossy = ais_mod.Channel(1, LAT0, LON0, loss=0.5)
    for i in range(400):
        lossy.send(ais_mod.Report(1, float(i), LAT0, LON0, 1.0, 0.0))
    assert 140 < len(lossy.receive(1e9)) < 260


def test_ais_turning_vessel_reports_faster():
    tp = ais_mod.Transponder(5, LAT0, LON0)
    times = []
    for k in range(300):
        t = 0.1 * k
        cog = 0.2 * t  # 11 degrees per second: well above the turning threshold
        tp.fix(t, LAT0, LON0, (1.5 * math.sin(cog), 1.5 * math.cos(cog), 0.0))
        if tp.poll(t):
            times.append(t)
    assert np.diff(times)[2:] == pytest.approx(10.0 / 3.0, abs=0.11)


# ------------------------------------------------------------------------------------------------------------------------------------ encounters
def test_encounters_are_seeded_and_on_collision_courses():
    a = enc.make("crossing_give_way", seed=7, hs=0.5, jitter=1.0)
    b = enc.make("crossing_give_way", seed=7, hs=0.5, jitter=1.0)
    c = enc.make("crossing_give_way", seed=8, hs=0.5, jitter=1.0)
    assert a == b and a != c
    assert (
        enc.make("head_on", seed=7).traffic[0].pos
        == enc.make("head_on", seed=99).traffic[0].pos
    )  # jitter 0: the textbook geometry, seed-free
    for kind in ("head_on", "crossing_give_way", "crossing_stand_on", "overtaking"):
        for seed in range(1, 8):
            s = enc.make(kind, seed=seed, hs=0.4, jitter=1.0)
            (t,) = s.traffic
            cpa, tcpa = cr.cpa_tcpa(
                (0, 0),
                (enc.ASV_SPEED, 0),
                t.pos,
                t.speed * np.array([math.cos(t.psi), math.sin(t.psi)]),
            )
            assert cpa <= 3.5, (kind, seed, cpa)  # "if nobody acted, they would meet"
    assert enc.peak_period(0.2) >= 2.2 and enc.peak_period(0.8) == pytest.approx(
        3.5 * math.sqrt(0.8) + 0.6
    )
    with pytest.raises(ValueError):
        enc.make("nonsense")


# ------------------------------------------------------------------------------------------------------------------------------------ planner
def route_state(pos=(0.0, 0.0), psi=0.0):
    rt = Route([(0, 0), (160, 0)], speed=1.5).update(pos)
    return dict(
        rt, leg_a=np.array([0.0, 0.0]), leg_dir=np.array([1.0, 0.0]), leg_len=160.0
    )


def plan_for(tgt_pos, tgt_vel, own_psi=0.0):
    """Plan at t = 10 s with a contact that has been tracked since t = 0 (AIS reports 5 s apart)."""
    p0 = np.asarray(tgt_pos, float) - 10.0 * np.asarray(tgt_vel, float)
    tr = Tracker()
    for t in (0.0, 5.0, 10.0):
        tr.update_ais(
            t,
            [
                dict(
                    id=3,
                    t_tx=t,
                    x=p0[0] + tgt_vel[0] * t,
                    y=p0[1] + tgt_vel[1] * t,
                    vx=tgt_vel[0],
                    vy=tgt_vel[1],
                )
            ],
        )
    own = dict(
        pos=np.array([0.0, 0.0]),
        vel=1.5 * np.array([math.cos(own_psi), math.sin(own_psi)]),
        psi=own_psi,
        speed=1.5,
    )
    pl = Planner()
    return pl.plan(10.0, own, route_state(), tr.moving(10.0), np.zeros((0, 2)))


def test_planner_head_on_turns_to_starboard():
    p = plan_for((110, 1.0), (-1.5, 0.0))
    assert p.rule == "head_on" and p.delta < 0.0 and p.hold > 0.0
    assert p.contacts[0]["role"] == "give_way"


def test_planner_crossing_give_way_never_turns_to_port():
    p = plan_for((75, -75), (0.0, 1.5))  # from the starboard side, arrives with us
    assert p.rule == "crossing_give_way" and p.contacts[0]["role"] == "give_way"
    assert p.delta <= 0.0 and (p.delta < 0.0 or p.frac < 1.0)


def test_planner_crossing_stand_on_keeps_course_and_speed_while_time_remains():
    p = plan_for((75, 75), (0.0, -1.5))  # from the port side, 40 s to go
    assert p.rule == "crossing_stand_on" and p.contacts[0]["role"] == "stand_on"
    assert p.delta == 0.0 and p.frac == 1.0 and not p.emergency


def test_planner_overtaking_gives_way_to_starboard_with_a_wider_pass():
    p = plan_for((45, 0.5), (0.7, 0.0))
    assert p.rule == "overtaking" and p.contacts[0]["role"] == "give_way"
    assert p.delta < 0.0


def test_planner_without_traffic_or_obstacles_follows_the_route():
    pl = Planner()
    own = dict(pos=np.array([0.0, 0.0]), vel=np.array([1.5, 0.0]), psi=0.0, speed=1.5)
    p = pl.plan(10.0, own, route_state(), [], np.zeros((0, 2)))
    assert p.delta == 0.0 and p.frac == 1.0 and p.rule == "none"


def test_planner_steers_around_a_fixed_object_on_the_route():
    pts = np.array(
        [
            [30.0 + 0.3 * np.cos(a), 0.0 + 0.3 * np.sin(a)]
            for a in np.linspace(0, 2 * np.pi, 12)
        ]
    )
    pl = Planner()
    own = dict(pos=np.array([5.0, 0.0]), vel=np.array([1.5, 0.0]), psi=0.0, speed=1.5)
    p = pl.plan(10.0, own, route_state((5.0, 0.0)), [], pts)
    assert p.hold > 0.0 and (p.delta != 0.0 or p.frac < 1.0)
    d = np.hypot(*(p.traj - np.array([30.0, 0.0])).T)
    assert d.min() > 3.0  # the chosen rollout does not pass through the object


# ---------------------------------------------------------------------------------------------------------------------------------- assemblies
@pytest.mark.parametrize("name", ["asv", "asv-traffic"])
def test_assemblies_are_drc_clean(name):
    d = Design.open(REPO / "assemblies" / f"{name}.toml")
    errors = [f for f in d.check() if f.severity == "error"]
    assert errors == [], [(f.code, f.message) for f in errors]


def test_asv_assembly_builds_with_bidirectional_escs_and_the_sensor_suite(tmp_path):
    rep = design.build(REPO / "assemblies" / "asv.toml", PARTS, tmp_path)["report"]
    m = rep["mass"]["total"]
    assert 8.0 < m < 20.0, (
        m
    )  # kg: BlueBoat class (the real BlueBoat weighs about 14 kg)
    import json

    xml = next(tmp_path.rglob("*.xml")).read_text()
    devs = json.loads(
        xml.split("napl.devices")[1].split("data='", 1)[1].split("'/>", 1)[0]
    )
    assert len(devs["escs"]) == 2 and all(e.get("bidirectional") for e in devs["escs"])
    assert len(devs["rotors"]) == 2 and all(
        0.7 < r["rev_thrust"] < 0.85 for r in devs["rotors"]
    )
    assert devs["vessels"][0].get(
        "fins"
    )  # the skegs that fix the directional instability
    kinds = {k for k in devs if devs[k]}
    assert {
        "imus",
        "mags",
        "gnss_receivers",
        "rangefinders",
        "batteries",
        "seas",
        "vessels",
    } <= kinds


# ------------------------------------------------------------------------------------------------------------------------------------ scenarios
# A whole scenario takes about 2 minutes of wall time (more on a busy host): the full run is tested for the busiest one (three vessels, every Rule), the others run
# 45 s of their scenario (`t_stop`: through the approach, before the mission ends) and are judged on the monitors that do not need the whole mission. The full
# runs of every scenario are `napl program` and mc.py (examples/asv/README.md).
FULL_SCENARIO = "multi_traffic"
PARTIAL_MONITORS = {
    "no_collision",
    "estimate_position",
    "estimate_heading",
    "clear_of_fixed_objects",
}


@pytest.mark.parametrize("kind", enc.KINDS)
def test_scenario_passes_its_monitors_and_replays(kind, tmp_path, monkeypatch):
    import json

    from napl import program

    full = kind == FULL_SCENARIO
    if not full:
        monkeypatch.setenv("NAPL_PARAMS", json.dumps({"t_stop": 45.0}))
    res, run_dir = program.run(
        EX / "asv" / f"asv-{kind.replace('_', '-')}.scenario.toml", tmp_path / "run"
    )
    bad = [
        m["name"]
        for m in res["monitors"]
        if not m["pass"] and (full or m["name"] in PARTIAL_MONITORS)
    ]
    assert not bad, bad
    assert res["replay"]["available"] and res["replay"]["match"], res["replay"]
    assert any(Path(run_dir).rglob("telemetry.jsonl"))


# ------------------------------------------------------------------------------------------------------------------ ASV2: multi-vessel, planner lines, drift
def test_multi_traffic_has_three_vessels_with_mixed_roles_on_collision_courses():
    for seed in range(1, 6):
        s = enc.make("multi_traffic", seed=seed, hs=0.4, jitter=1.0)
        assert [t.kind for t in s.traffic] == [
            "crossing_give_way",
            "head_on",
            "crossing_stand_on",
        ]
        for t in s.traffic:
            v = t.speed * np.array([math.cos(t.psi), math.sin(t.psi)])
            cpa, _ = cr.cpa_tcpa((0, 0), (enc.ASV_SPEED, 0), t.pos, v)
            assert cpa <= 6.0, (seed, t.name, cpa)


def test_verdict_all_judges_the_stand_on_vessel_only_where_the_asv_is_not_giving_way():
    t, own, psi, spd, tgt, tv = tracks(tgt_y=8.0)
    near = np.hypot(*(tgt - own).T) < 60.0
    turn = np.where(near, math.radians(25.0), 0.0)  # 25 degrees to port near both
    v = judge.verdict_all(
        ["head_on", "crossing_stand_on"],
        t,
        own,
        turn,
        spd,
        [tgt, tgt],
        [tv, tv],
        0.0,
        1.5,
    )
    assert not v["vessels"][0]["side_ok"]  # a port turn while giving way
    assert v["vessels"][1][
        "side_ok"
    ]  # excluded: the ASV was acting for the head-on vessel
    assert not v["side_ok"] and not v["colreg_ok"]
    v = judge.verdict_all(
        ["head_on", "crossing_stand_on"],
        t,
        own,
        psi,
        spd,
        [tgt, tgt],
        [tv, tv],
        0.0,
        1.5,
    )
    assert v["colreg_ok"] and v["cpa_m"] == pytest.approx(8.0, abs=0.3)


def test_route_steers_at_the_end_waypoint_when_the_look_ahead_would_overshoot_it():
    r = Route([(0, 0), (100, 0)], speed=1.5)
    st = r.update((92.0, 12.0))  # 8 m short of the end, 12 m off the leg
    assert st["psi"] == pytest.approx(math.atan2(-12.0, 8.0), abs=1e-6)
    far = Route([(0, 0), (100, 0)], speed=1.5).update((20.0, 12.0))
    assert far["psi"] == pytest.approx(
        0.0 - math.atan2(12.0, max(Gains().lookahead, Gains().lookahead_k * 12.0))
    )


def test_planner_follows_a_parallel_line_around_a_slow_vessel_and_steer_returns_its_course():
    from asv.planner import steer

    p = plan_for((45, 0.5), (0.7, 0.0))  # overtaking a slow vessel dead ahead
    assert p.rule == "overtaking"
    rt = route_state()
    c = steer(p, rt, np.array([0.0, 0.0]))
    assert c < rt["psi"] or p.hold <= 0  # to starboard of the leg course
    assert steer(None, rt, np.array([0.0, 0.0])) == rt["psi"]
    assert p.traj[:, 1].min() > -40.0  # the detour stays bounded


def test_asv_assemblies_use_the_heading_frame_damping(tmp_path):
    import json

    design.build(REPO / "assemblies" / "asv.toml", PARTS, tmp_path)
    xml = next(tmp_path.rglob("*.xml")).read_text()
    devs = json.loads(
        xml.split("napl.devices")[1].split("data='", 1)[1].split("'/>", 1)[0]
    )
    assert devs["vessels"][0]["heading_frame"] is True


def _passive_drift(tmp_path, heading_frame, hs=0.5, seed=1, t_end=110.0):
    """Mean horizontal drift speed (m/s) of the unpowered ASV hull in waves between 20 s and t_end (no current)."""
    import json
    import xml.etree.ElementTree as ET

    import napl
    from asv.scene import build_spec

    tmp_path.mkdir(parents=True, exist_ok=True)
    spec = enc.make("transit", seed=seed, hs=hs, jitter=0.0)
    m = build_spec(tmp_path, spec, seed=seed)
    tree = ET.parse(m)
    txt = tree.getroot().find("custom/text")
    dd = json.loads(txt.get("data"))
    for v in dd["vessels"]:
        v["heading_frame"] = heading_frame
    dd[
        "rangefinders"
    ] = []  # the 1600-ray lidar is most of the cost of a run and not needed here
    txt.set("data", json.dumps(dd))
    tree.write(m)
    sim = napl.Sim.load(str(m))
    p0 = None
    while sim.tick * 0.002 < t_end:
        sim.step(50)
        t, pos = sim.tick * 0.002, np.array(sim.query("/world/asv_hull", "pos")[0][:2])
        if t > 20.0 and p0 is None:
            p0 = (t, pos)
        last = (t, pos)
    return float(np.hypot(*(last[1] - p0[1])) / (last[0] - p0[0]))


def test_passive_asv_hull_in_waves_drifts_only_with_body_frame_heave_damping(tmp_path):
    old = _passive_drift(tmp_path / "a", False)
    new = _passive_drift(tmp_path / "b", True)
    assert new < 0.02, (old, new)  # the wave drift of a real hull is a few cm/s at most
    assert old > 2.0 * new, (old, new)  # the body-frame damping propels it
