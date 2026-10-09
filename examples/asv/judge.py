"""Judging an encounter from the TRUE tracks (never from the ASV's estimates): minimum distance, collision, the side the vessels passed on, and whether the ASV's
manoeuvre was the one the Rules ask for. Pure functions of arrays, so the same code scores a run, a replay and the unit tests.

Thresholds (the Rules give none; these are the ones the scenarios and the README state):
  collision        centre distance below 1.2 m (two 1.2 m hulls touching) to a traffic vessel, or the hull centre within 0.75 m of a static object's surface;
  CPA ok           at least 5 m (four hull lengths), the monitor of every encounter;
  port-to-port     head-on: the other vessel is on our port side at the closest point (Rule 14);
  astern           crossing give-way: at the closest point we are in the stern half of the other vessel (Rule 15: avoid crossing ahead of it);
  no port turn     the ASV never steers more than 10 degrees to port of its route course while a give-way obligation is open (Rule 16: early, substantial,
                   to starboard; a port turn for a vessel on the starboard bow is what the Rules discourage);
  stand-on         Rule 17(a): while the other vessel is within 60 m ahead of the closest point we keep within 15 degrees of the route course and 70 % of its speed.
"""

import math

import numpy as np

from .geo import wrap

COLLISION_VESSEL = 1.2
COLLISION_STATIC = 0.75
CPA_OK = 5.0
PORT_TURN_MAX = math.radians(10.0)
STAND_ON_HDG = math.radians(15.0)
STAND_ON_SPEED = 0.7
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
    kind, t, own, own_psi, own_speed, tgt, tgt_vel, route_psi, speed_nom, complied=True
):
    """The judgement of an encounter. own / tgt (N, 2) true positions sampled together; own_psi (N,) true heading; own_speed (N,); tgt_vel (N, 2) the other vessel's
    true velocity; route_psi the course of the mission leg the encounter is on; speed_nom the mission speed. -> dict of metrics and the booleans
    cpa_ok, collision, side_ok (the Rule-specific manoeuvre), colreg_ok (= cpa_ok and no collision and side_ok)."""
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
    max_port = float(dev[risk].max()) if risk.any() else 0.0
    max_stbd = float(-dev[risk].min()) if risk.any() else 0.0
    no_port_turn = max_port <= PORT_TURN_MAX
    spd_min = float(own_speed[risk].min()) if risk.any() else speed_nom
    if kind == "head_on":
        side_ok = g["side"] > 0 and no_port_turn
    elif kind == "crossing_give_way":
        side_ok = g["astern"] and no_port_turn
    elif kind == "crossing_stand_on":
        side_ok = (
            (
                max(max_port, max_stbd) <= STAND_ON_HDG
                and spd_min >= STAND_ON_SPEED * speed_nom
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
