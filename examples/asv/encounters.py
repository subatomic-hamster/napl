"""The harbour and its encounters: the static scenery, the sea state and the scripted traffic of every scenario, as plain data (`Spec`).

All coordinates are the scene's ENU frame (x East, y North, m), yaw is the math angle (counter-clockwise from East). The ASV always starts at the origin heading East
towards the end of a 160 m mission leg; the traffic is placed so that, if nobody acted, the two vessels would collide near the middle of the leg (a collision course:
the closest point of approach of the constant-velocity tracks is `cpa0` below). `make(kind, seed, hs, jitter)` returns the Spec; jitter = 0 is the textbook geometry,
jitter = 1 draws the angles, speeds, offsets and sea direction from numpy's PCG64 seeded [seed, 4040] in a fixed order, so a (kind, seed, hs, jitter) replays exactly.
"""

import math
from dataclasses import dataclass, field

import numpy as np

KINDS = (
    "patrol",
    "head_on",
    "crossing_give_way",
    "crossing_stand_on",
    "overtaking",
    "multi_traffic",
)
TUNING_KINDS = (
    "transit",
)  # a plain 160 m leg without traffic, for tuning and tests; not a scenario
ASV_SPEED = 1.5  # m/s, the mission speed
LEG = 160.0  # m, length of the mission leg of the encounters
T_START = 8.0  # s, the vessel floats for this long before the mission starts (the boat settles at its draft, the estimator converges)

# Static scenery: (name, kind, centre xyz, size, rgba). kind box: size = half extents; cyl: size = (radius, half height). Water is z = 0; the seabed is at z = -3.
SCENERY = [
    (
        "shore_north",
        "box",
        (60.0, 140.0, 0.5),
        (140.0, 30.0, 1.0),
        (0.30, 0.34, 0.26, 1),
    ),  # y > 110
    (
        "shore_west",
        "box",
        (-90.0, 20.0, 0.5),
        (12.0, 130.0, 1.0),
        (0.30, 0.34, 0.26, 1),
    ),  # x < -78
    (
        "pier",
        "box",
        (125.0, 92.0, 0.4),
        (1.5, 18.0, 0.6),
        (0.50, 0.45, 0.38, 1),
    ),  # from the north shore down to y = 74
    ("pier_head", "box", (125.0, 72.0, 0.4), (3.5, 1.5, 0.6), (0.50, 0.45, 0.38, 1)),
    (
        "breakwater",
        "box",
        (70.0, -113.0, 0.3),
        (70.0, 2.0, 0.8),
        (0.42, 0.42, 0.44, 1),
    ),  # y = -115 ... -111
    (
        "dock",
        "box",
        (-30.0, 34.0, 0.2),
        (9.0, 1.5, 0.4),
        (0.55, 0.50, 0.42, 1),
    ),  # a floating dock off the start
]
BUOYS = [  # channel markers along the mission leg, 20 m off the centreline, and the two buoys the patrol has to pass
    ("buoy_r1", (40.0, -20.0), (0.75, 0.10, 0.08, 1)),
    ("buoy_g1", (40.0, 20.0), (0.10, 0.60, 0.20, 1)),
    ("buoy_r2", (120.0, -20.0), (0.75, 0.10, 0.08, 1)),
    ("buoy_g2", (120.0, 20.0), (0.10, 0.60, 0.20, 1)),
]
BUOY_R, BUOY_H = 0.35, 0.5  # m, radius and half height (z centre 0.25 above the water)


@dataclass
class Bot:
    """Scripted behaviour of a traffic vessel (examples/asv/traffic.py). kind: keep (course and speed), give_way (turn to starboard `turn_deg` when the ASV comes
    within `r_act` and keep it up until clear; `complies = False` makes it keep its course instead), head_on (turn to starboard when within `r_act`)."""

    kind: str = "keep"
    r_act: float = 45.0
    turn_deg: float = 30.0
    complies: bool = True


@dataclass
class Traffic:
    name: str
    pos: tuple
    psi: float
    speed: float
    route: list  # waypoints (E, N) the vessel follows (its own autopilot, which sees the true state)
    bot: Bot = field(default_factory=Bot)
    ais: bool = True  # carries an AIS transponder (False: only the lidar can see it)
    kind: str = ""  # the situation this vessel is judged as (head_on, crossing_give_way, ...); empty: the Spec's kind


@dataclass
class Obstacle:
    name: str
    pos: tuple


@dataclass
class Spec:
    kind: str
    seed: int
    hs: float
    tp: float
    wave_dir: float  # rad, direction the waves travel
    route: list
    speed: float
    traffic: list = field(default_factory=list)
    obstacles: list = field(
        default_factory=list
    )  # extra floating objects on the route (lidar-only)
    t_end: float = 120.0
    current: tuple = (0.0, 0.0)
    cpa0: float = (
        0.0  # closest approach of the first traffic vessel if nobody acted (m)
    )


def peak_period(hs):
    """Peak period (s) of a wind sea of significant height hs: 3.5 sqrt(hs) + 0.6, floored at 2.2 s. At hs = 0.8 m this is 3.7 s, a wavelength of 21 m, steepness
    hs / L = 1/26: under the 1/7 breaking limit and in the range of fetch-limited seas (fully developed seas are steeper in period, not in height)."""
    return max(2.2, 3.5 * math.sqrt(hs) + 0.6)


def _line(p, psi, length):
    return [tuple(p), (p[0] + length * math.cos(psi), p[1] + length * math.sin(psi))]


def make(kind, seed=1, hs=0.4, jitter=0.0, ais=True, traffic_complies=True) -> Spec:
    """The Spec of encounter `kind` (see KINDS)."""
    if kind not in KINDS + TUNING_KINDS:
        raise ValueError(f"encounter {kind!r}: one of {KINDS}")
    r = np.random.default_rng([int(seed), 4040])
    u = lambda lo, hi: (
        float(lo + (hi - lo) * r.random()) if jitter else 0.5 * (lo + hi)
    )  # fixed draw order: always consumes one number
    j = lambda lo, hi: float(0.5 * (lo + hi) + jitter * (u(lo, hi) - 0.5 * (lo + hi)))
    wave_dir = j(-math.radians(60), math.radians(60)) + (
        0.0 if jitter else math.radians(15)
    )  # waves from roughly ahead / the bow quarter
    spec = Spec(
        kind,
        int(seed),
        float(hs),
        peak_period(hs),
        wave_dir,
        [(0.0, 0.0), (LEG, 0.0)],
        ASV_SPEED,
    )
    xc = j(70.0, 90.0)  # where along the leg the tracks meet
    t_c = xc / ASV_SPEED
    if kind == "patrol":
        spec.route = [(0.0, 0.0), (100.0, 0.0), (100.0, 18.0), (0.0, 18.0), (0.0, 2.0)]
        spec.obstacles = [
            Obstacle("float1", (50.0, 0.0)),
            Obstacle("float2", (50.0, 18.0)),
        ]  # on the two legs
        spec.t_end = T_START + 1.3 * (2 * 100 + 2 * 18) / ASV_SPEED + 30
        return spec
    spec.t_end = T_START + 1.4 * LEG / ASV_SPEED + 30
    if kind == "transit":
        return spec
    if kind == "head_on":
        y_off = j(-3.0, 3.0)
        v = j(1.2, 1.8)
        x0 = xc + v * t_c
        spec.traffic = [
            Traffic(
                "t1",
                (x0, y_off),
                math.pi,
                v,
                _line((x0, y_off), math.pi, 220.0),
                Bot("head_on", 45.0, 30.0, traffic_complies),
                ais,
            )
        ]
        spec.cpa0 = abs(y_off)
    elif (
        kind == "crossing_give_way"
    ):  # the target comes from our starboard (south) side, heading North-ish: we give way
        ang = j(math.radians(70), math.radians(110))  # its course
        v = j(0.9, 1.4)
        d = v * t_c
        p0 = (xc - d * math.cos(ang), -d * math.sin(ang))
        spec.traffic = [
            Traffic("t1", p0, ang, v, _line(p0, ang, 260.0), Bot("keep"), ais)
        ]
        spec.cpa0 = 0.0
    elif (
        kind == "crossing_stand_on"
    ):  # the target comes from our port (north) side heading South-ish: it gives way, we stand on
        ang = -j(math.radians(70), math.radians(110))
        v = j(0.9, 1.4)
        d = v * t_c
        p0 = (xc - d * math.cos(ang), -d * math.sin(ang))
        spec.traffic = [
            Traffic(
                "t1",
                p0,
                ang,
                v,
                _line(p0, ang, 260.0),
                Bot("give_way", 50.0, 45.0, traffic_complies),
                ais,
            )
        ]
        spec.cpa0 = 0.0
    elif (
        kind == "overtaking"
    ):  # a slower vessel ahead on our course: we overtake and keep clear
        v = j(0.6, 0.9)
        y_off = j(-2.0, 2.0)
        x0 = j(35.0, 50.0)
        spec.traffic = [
            Traffic(
                "t1",
                (x0, y_off),
                0.0,
                v,
                _line((x0, y_off), 0.0, 300.0),
                Bot("keep"),
                ais,
            )
        ]
        spec.cpa0 = abs(y_off)
        spec.t_end = T_START + 1.4 * LEG / ASV_SPEED + 50
    elif kind == "multi_traffic":
        # a crowded harbour: three vessels with mixed roles, one after the other along the leg (the ASV passes x = 35-45 m at 30 s, 60-80 m at 50 s, 110-135 m at 80 s):
        #   t3 from the port side, heading South-ish, gives way (45 degrees to starboard at 50 m): the ASV stands on (Rule 15 / 17),
        #   t1 from the starboard side, heading North-ish, keeps its course: the ASV gives way and passes astern (Rule 15 / 16),
        #   t2 on the reciprocal course, turns to starboard at 45 m: the ASV turns to starboard and passes port-to-port (Rule 14).
        # Their action windows overlap: t3 is still near when t1 is, and t1 when t2 is.
        spec.traffic = [
            _crossing(
                "t1",
                "crossing_give_way",
                j(70.0, 90.0) - 10.0,
                j(math.radians(75), math.radians(105)),
                j(0.9, 1.3),
                ais,
                traffic_complies,
            ),
            _head_on(
                "t2", j(115.0, 135.0), j(-3.0, 3.0), j(1.2, 1.6), ais, traffic_complies
            ),
            _crossing(
                "t3",
                "crossing_stand_on",
                j(32.0, 46.0),
                -j(math.radians(75), math.radians(105)),
                j(0.9, 1.3),
                ais,
                traffic_complies,
            ),
        ]
        spec.cpa0 = 0.0
        spec.t_end = T_START + 1.4 * LEG / ASV_SPEED + 40
    for t_ in spec.traffic:
        t_.kind = t_.kind or kind
    return spec


def _crossing(name, kind, xc, ang, v, ais, complies):
    """A vessel that crosses the ASV's leg at x = xc (if nobody acts, the ASV is there at the same time): from the starboard side (ang > 0, `keep`) or the port side (ang < 0,
    gives way)."""
    d = v * xc / ASV_SPEED
    p0 = (xc - d * math.cos(ang), -d * math.sin(ang))
    bot = (
        Bot("keep")
        if kind == "crossing_give_way"
        else Bot("give_way", 50.0, 45.0, complies)
    )
    return Traffic(name, p0, ang, v, _line(p0, ang, 260.0), bot, ais, kind)


def _head_on(name, xc, y_off, v, ais, complies):
    x0 = xc + v * xc / ASV_SPEED
    return Traffic(
        name,
        (x0, y_off),
        math.pi,
        v,
        _line((x0, y_off), math.pi, 220.0),
        Bot("head_on", 45.0, 30.0, complies),
        ais,
        "head_on",
    )
