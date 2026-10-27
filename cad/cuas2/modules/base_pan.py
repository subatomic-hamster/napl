"""M1 BASE / PAN (spec 3): base plate 200 x 200, four legs and a lower deck (Jetson), pan motor under the plate with a 20T/100T GT2
drive (v1 ratio 5:1), 6 in lazy susan on four posts, rotor plate with the hung 100T pulley, pan hall + hard stop. Frame B (origin on
the pan axis at the top face of the stator plate). Yoke seats Y1 (4 x M4, 50 x 50) on the rotor top.
"""

import math

import interfaces as I
import params as P
from build123d import Rot
from prims import bxr, hole, tube

Y1 = I.DOCKS["Y1"]
LEG_C = 90.0
HALL_X = -82.0
PAN_STOP_R = 89.75


def base_plate():
    s = bxr(-100, 100, -100, 100, -8, 0)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = P.LS_BOLT_R * math.cos(a), P.LS_BOLT_R * math.sin(a)
        s = s + hole("z", 12.0, x, y, 0, P.POST_H)
        s = s - hole("z", P.M4_INSERT_D, x, y, P.POST_H - 8.0, P.POST_H + 1)
    # pan motor under the plate: shaft hole + 31 mm M3 pattern (screws from above)
    s = s - hole("z", 23.0, P.C_PAN, 0, -9, 1)
    for dx in (-1, 1):
        for dy in (-1, 1):
            s = s - hole("z", P.M3_CLR, P.C_PAN + dx * 15.5, dy * 15.5, -9, 1)
    # legs (M3 inserts from below), power box lugs (inserts from below), cable hole
    for sx in (-1, 1):
        for sy in (-1, 1):
            s = s - hole("z", P.M3_INSERT_D, sx * LEG_C, sy * LEG_C, -8 - 1, -8 + 6)
    for bx in (-45.0, 45.0):
        for by in (-35.0, 35.0):
            s = s - hole("z", P.M3_INSERT_D, -47.0 + bx, by, -8 - 1, -8 + 6)
    s = s - hole("z", 28.0, 0, 0, -9, 1)
    # pan hall post (outside the lazy susan, r > 76.2) and the M3 hard-stop stud
    s = s + bxr(HALL_X - 2.5, HALL_X + 2.5, -4, 4, 0, 24.0)
    s = s + hole("z", 3.0, 0, -PAN_STOP_R, 0, 14.0)
    return s


def base_leg():
    # the leg stands on the deck top (deck plate 4 mm thick); the deck screws come up from below into a heat-set insert in the leg foot
    return (
        bxr(-10, 10, -10, 10, P.DECK_Z + 4.0, -8)
        - hole("z", P.M3_CLR, 0, 0, P.DECK_Z + 4.0 + 6.0, -8 + 1)
        - hole("z", P.M3_INSERT_D, 0, 0, P.DECK_Z + 4.0 - 1, P.DECK_Z + 4.0 + 6.0)
    )


def lower_deck():
    s = bxr(-100, 100, -100, 100, P.DECK_Z, P.DECK_Z + 4.0)
    for sx in (-1, 1):
        for sy in (-1, 1):
            s = s - hole(
                "z", P.M3_CLR, sx * LEG_C, sy * LEG_C, P.DECK_Z - 1, P.DECK_Z + 5
            )
    # Jetson standoffs, 86 x 58 pattern, 6 mm, centred at x = -44
    for dx in (-43.0, 43.0):
        for dy in (-29.0, 29.0):
            s = s + hole("z", 7.0, -44.0 + dx, dy, P.DECK_Z + 4.0, P.DECK_Z + 10.0)
            s = s - hole(
                "z", P.M3_INSERT_D, -44.0 + dx, dy, P.DECK_Z + 4.0, P.DECK_Z + 11.0
            )
    return s


def rotor_plate():
    z0, z1 = P.Z_ROTOR0, P.Z_ROTOR0 + P.ROTOR_T
    s = bxr(-90, 90, -90, 90, z0, z1)
    s = s - hole(
        "z", 40.0, 0, 0, z0 - 1, z1 + 1
    )  # cable pass-through (the lazy susan opening below is larger)
    for k in range(4):
        a = math.radians(90 * k)
        s = s - hole(
            "z",
            P.M4_CLR,
            P.LS_BOLT_R * math.cos(a),
            P.LS_BOLT_R * math.sin(a),
            z0 - 1,
            z1 + 1,
        )
    # yoke seats: dowel + M4 pilots, Y1 male on both sides
    for sy in (1, -1):
        loc = I.place((0.0, sy * 60.0, z1), (1, 0, 0), (0, 0, 1))
        s = s + loc * I.male_feature(Y1)
        s = s - loc * I.bolt_holes(Y1, "male")
    # hung 100T pulley (teeth not modelled): ring + 3 spokes + 3 posts through the lazy susan aperture
    zb = 7.0
    ring = tube("z", P.PAN_PULLEY_OD / 2 + 1.5, 24.0, 0, 0, zb - 5.0, zb + 5.0)
    s = s + ring
    for k in range(3):
        a = math.radians(120 * k + 30)
        x, y = 46.0 * math.cos(a), 46.0 * math.sin(a)
        s = s + hole("z", 8.0, x, y, zb + 5.0, z0 + 0.5)
        s = s + Rot(0, 0, 120 * k + 30) * bxr(23.0, 46.0, -3.0, 3.0, zb + 2.0, zb + 5.0)
    # pan hall magnet pocket under the rim, hard-stop tab at +90 deg
    s = s - hole("z", 6.2, HALL_X, 0, z0 - 1, z0 + 2.2)
    s = s + bxr(-1.8, 1.8, PAN_STOP_R - 3.0, PAN_STOP_R + 3.0, 8.0, z0 + 0.5)
    return s


PARTS = {
    "base_plate": dict(
        fn=base_plate, group="static", count=1, material="PETG", module="M1"
    ),
    "base_leg": dict(
        fn=base_leg, group="static", count=4, material="PETG", module="M1"
    ),
    "lower_deck": dict(
        fn=lower_deck, group="static", count=1, material="PETG", module="M1"
    ),
    "rotor_plate": dict(
        fn=rotor_plate, group="pan", count=1, material="PETG", module="M1"
    ),
}


def bought():
    out = []
    z0 = P.Z_ROTOR0
    out.append(
        dict(
            name="lazy_susan",
            lib="generic-lazy-susan-6in-square",
            mass=P.LS_MASS,
            module="M1",
            group="pan",
            shape=hole("z", P.LS_OD, 0, 0, P.POST_H, z0)
            - hole("z", P.LS_APERTURE, 0, 0, P.POST_H - 1, z0 + 1),
        )
    )
    out.append(
        dict(
            name="nema_pan",
            lib="generic-17hs4401s-nema17-42x40",
            mass=P.N17_MASS,
            module="M1",
            group="static",
            shape=bxr(
                P.C_PAN - P.N17_FACE / 2,
                P.C_PAN + P.N17_FACE / 2,
                -P.N17_FACE / 2,
                P.N17_FACE / 2,
                -8 - P.N17_LEN,
                -8,
            ),
        )
    )
    out.append(
        dict(
            name="pulley_pan",
            lib="adafruit-1251-gt2-pulley-20t-5mm",
            mass=8.0,
            module="M1",
            group="static",
            shape=hole("z", 16.0, P.C_PAN, 0, 1.0, 15.0),
        )
    )
    out.append(
        dict(
            name="hall_pan",
            lib="allegro-a3144-hall-switch",
            mass=P.HALL_MASS,
            module="M1",
            group="static",
            shape=bxr(HALL_X - 2, HALL_X + 2, -1.5, 1.5, 24.0, 25.0),
        )
    )
    out.append(
        dict(
            name="magnet_pan",
            lib="cuas-lite-magnet-disc-6x2",
            mass=P.MAGNET_MASS,
            module="M1",
            group="pan",
            shape=hole("z", 6.0, HALL_X, 0, z0, z0 + 2.0),
        )
    )
    return out
