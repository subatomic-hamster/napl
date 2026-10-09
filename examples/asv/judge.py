"""Judging an encounter from the TRUE tracks (never from the ASV's estimates): minimum distance, collision, the side the vessels passed on, and whether the ASV's
manoeuvre was the one the Rules ask for. Pure functions of arrays, so the same code scores a run, a replay and the unit tests.

Thresholds (the Rules give none; these are the ones the scenarios and the README state):
  collision        centre distance below 1.2 m (two 1.2 m hulls touching) to a traffic vessel, or the hull centre within 0.75 m of a static object's surface;
  CPA ok           at least 5 m (four hull lengths), the monitor of every encounter;
  port-to-port     head-on: the other vessel is on our port side at the closest point (Rule 14);
  astern           crossing give-way: at the closest point we are in the stern half of the other vessel (Rule 15: avoid crossing ahead of it);
  no port turn     the ASV never steers more than 10 degrees to port of its route course while a give-way obligation is open (Rule 16: early, substantial,
                   to starboard; a port turn for a vessel on the starboard bow is what the Rules discourage);
  stand-on         Rule 17(a): while the other vessel is within 60 m ahead of the closest point we keep within 15 degrees of the route course and 70 % of the speed it made before (5 s smoothed).
"""

import math

import numpy as np

from .geo import wrap

COLLISION_VESSEL = 1.2
COLLISION_STATIC = 0.75
CPA_OK = 5.0
PORT_TURN_MAX = math.radians(10.0)
STAND_ON_HDG = math.radians(15.0)
STAND_ON_SPEED = 0.7  # of the speed made before the encounter (smoothed), see verdict()
WINDOW_SPEED_S = 5.0  # s, smoothing of the speed trace
WINDOW = 60.0


def box_dist(p, c, half):
    """Distance from point p (xy) to the axis-aligned box centre c, half extents `half` (xy); 0 inside."""
    d = np.maximum(
        np.abs(np.asarray(p, float) - np.asarray(c, float)[:2])
        - np.asarray(half, float)[:2],
        0.0,
    )
    return float(np.hypot(*d))


def static_clearance(p, scenery, buoys, obstacles, buoy_r, obstacle_r=0.4):
    """Smallest distance from the point p to the surface of any static object (scenery boxes, buoy and obstacle cylinders)."""
    best = math.inf
    for _, kind, c, size, _ in scenery:
        best = min(best, box_dist(p, c, size))
    for _, (x, y), _ in buoys:
        best = min(best, math.hypot(p[0] - x, p[1] - y) - buoy_r)
    for o in obstacles:
        best = min(best, math.hypot(p[0] - o.pos[0], p[1] - o.pos[1]) - obstacle_r)
    return best


def pass_geometry(own, tgt, own_psi, tgt_vel):
    """At the closest approach of two tracks sampled together (N, 2 arrays): -> dict(cpa, k (sample), t_index side (+1 target on our port), astern (bool))."""
    d = np.hypot(*(tgt - own).T)
    k = int(np.argmin(d))
    rel = tgt[k] - own[k]
    h = np.array([math.cos(own_psi[k]), math.sin(own_psi[k])])
    side = (
        1 if h[0] * rel[1] - h[1] * rel[0] > 0 else -1
    )  # target to the left of our heading
    tv = tgt_vel[k]
    astern = bool((own[k] - tgt[k]) @ tv < 0) if np.hypot(*tv) > 1e-6 else True
    return dict(cpa=float(d[k]), k=k, side=side, astern=astern)


def verdict(
    kind,
    t,
    own,
    own_psi,
    own_speed,
    tgt,
    tgt_vel,
    route_psi,
    speed_nom,
    complied=True,
    exclude=None,
):
    """The judgement of an encounter. own / tgt (N, 2) true positions sampled together; own_psi (N,) true heading; own_speed (N,); tgt_vel (N, 2) the other vessel's
    true velocity; route_psi the course of the mission leg the encounter is on; speed_nom the mission speed. -> dict of metrics and the booleans
    cpa_ok, collision, side_ok (the Rule-specific manoeuvre), colreg_ok (= cpa_ok and no collision and side_ok).
    exclude (N, bool): samples that do not count towards the Rule-specific manoeuvre (the ASV is giving way to another vessel then: it cannot also keep its course for this one)."""
    g = pass_geometry(own, tgt, own_psi, tgt_vel)
    collision = g["cpa"] < COLLISION_VESSEL
    cpa_ok = g["cpa"] >= CPA_OK
    dev = np.array(
        [wrap(a - route_psi) for a in own_psi]
    )  # + = to port of the route course
    d = np.hypot(*(tgt - own).T)
    k = g["k"]
    risk = (np.arange(len(d)) <= k) & (
        d < WINDOW
    )  # the approach to the closest point inside 60 m
    if exclude is not None:
        risk &= ~np.asarray(exclude, bool)
    max_port = float(dev[risk].max()) if risk.any() else 0.0
    max_stbd = float(-dev[risk].min()) if risk.any() else 0.0
    no_port_turn = max_port <= PORT_TURN_MAX
    # speed: smoothed over 5 s (a pitching 1.2 m hull in 0.8 m waves surges instantaneously to near zero and back), against what the boat made on its approach
    # before the encounter window (waves take up to 30 % off the nominal speed): "no deliberate slow-down" = the smoothed minimum stays above 70 % of that
    n_s = max(1, int(round(WINDOW_SPEED_S / max(float(np.median(np.diff(t))), 1e-6))))
    spd = np.convolve(np.asarray(own_speed, float), np.ones(n_s) / n_s, mode="same")
    pre = (np.arange(len(d)) < k) & (d >= WINDOW) & (spd > 0.3 * speed_nom)
    spd_ref = float(np.median(spd[pre])) if pre.any() else speed_nom
    spd_min = float(spd[risk].min()) if risk.any() else spd_ref
    if kind == "head_on":
        side_ok = g["side"] > 0 and no_port_turn
    elif kind == "crossing_give_way":
        side_ok = g["astern"] and no_port_turn
    elif kind == "crossing_stand_on":
        side_ok = (
            (
                max(max_port, max_stbd) <= STAND_ON_HDG
                and spd_min >= STAND_ON_SPEED * spd_ref
            )
            if complied
            else True
        )
    elif kind == "overtaking":
        side_ok = (
            bool((own[-1] - tgt[-1]) @ tgt_vel[-1] > 0)
            if np.hypot(*tgt_vel[-1]) > 1e-6
            else True
        )  # ahead of it at the end
    else:
        side_ok = True
    return dict(
        cpa_m=g["cpa"], t_cpa=float(t[k]), side="port_to_port" if g["side"] > 0 else "starboard_to_starboard", astern=g["astern"],
        max_port_deg=math.degrees(max_port), max_stbd_deg=math.degrees(max_stbd), min_speed_ms=spd_min,
        collision=bool(collision), cpa_ok=bool(cpa_ok), side_ok=bool(side_ok), colreg_ok=bool(cpa_ok and not collision and side_ok),
    )  # fmt: skip


GIVE_WAY_KINDS = ("head_on", "crossing_give_way", "overtaking")
GIVE_WAY_RANGE = 120.0  # m: a give-way vessel acts early (Rule 16), long before the 60 m approach window of the per-vessel judgement


def verdict_all(
    kinds,
    t,
    own,
    own_psi,
    own_speed,
    tgts,
    tgt_vels,
    route_psi,
    speed_nom,
    complied=True,
):
    """Several traffic vessels: the verdict of each (`kinds[i]` = the situation it is judged as; tgts[i], tgt_vels[i] its true track), and the totals. A vessel the ASV
    stands on for is judged only on the samples where the ASV is not giving way to another vessel (inside the 60 m approach window of a give-way contact): the Rules
    do not ask a vessel that must give way to one vessel to keep its course for another (Rule 2(b): departure to avoid immediate danger; Rule 8). Everything else is the
    single-vessel judgement. -> dict(vessels=[verdict, ...], cpa_m (the smallest), collision (any), cpa_ok / side_ok / colreg_ok (all))."""
    gw = np.zeros(len(t), bool)
    for kd, tg, tv in zip(kinds, tgts, tgt_vels):
        if kd in GIVE_WAY_KINDS:
            g = pass_geometry(own, tg, own_psi, tv)
            gw |= (np.arange(len(t)) <= g["k"]) & (
                np.hypot(*(tg - own).T) < GIVE_WAY_RANGE
            )
    vs = [
        verdict(
            kd,
            t,
            own,
            own_psi,
            own_speed,
            tg,
            tv,
            route_psi,
            speed_nom,
            complied,
            exclude=gw if kd == "crossing_stand_on" else None,
        )
        for kd, tg, tv in zip(kinds, tgts, tgt_vels)
    ]
    tot = dict(
        vessels=vs,
        cpa_m=min(v["cpa_m"] for v in vs),
        t_cpa=min(vs, key=lambda v: v["cpa_m"])["t_cpa"],
    )
    for key in ("collision",):
        tot[key] = any(v[key] for v in vs)
    for key in ("cpa_ok", "side_ok", "colreg_ok"):
        tot[key] = all(v[key] for v in vs)
    tot["max_port_deg"] = max(v["max_port_deg"] for v in vs)
    tot["max_stbd_deg"] = max(v["max_stbd_deg"] for v in vs)
    tot["min_speed_ms"] = min(v["min_speed_ms"] for v in vs)
    tot["side"] = vs[0]["side"]
    tot["astern"] = vs[0]["astern"]
    return tot
