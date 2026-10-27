"""M2 TILT HEAD (spec 3): cradle (tilt group, frame L) with the D1 dovetail slots, D2 sensor dock and two printed trunnion stubs in two
608 bearings; two yoke cheeks (pan group, frame B) with the tilt NEMA 17 on the left cheek. No hollow trunnion, no elbow.
Cheeks are on the rotor plate through dock Y1 (4 x M4 on 50 x 50). Tilt axis in B: (0, 0, H_T).
"""

import interfaces as I
import params as P
from build123d import Pos
from prims import bxr, hole, prism_xz, tube

D1, D2, Y1, H1 = (I.DOCKS[k] for k in ("D1", "D2", "Y1", "H1"))
Z_TIP = D1.loc["base_to_face"] + D1.loc["head_h"]
ARM_X0, ARM_X1 = P.X_F1 - 4.0, 40.0
BRIDGE_X0, BRIDGE_X1, BRIDGE_Z0, BRIDGE_Z1 = 24.0, 38.0, 17.0, 22.0
SENSOR_ORIGIN = (30.0, 0.0, BRIDGE_Z1)


def d1_loc(sy):
    return I.place((P.X_F1, sy * P.CORE_HALF, 0.0), (1, 0, 0), (0, sy, 0))


def d2_loc():
    return I.place(SENSOR_ORIGIN, (1, 0, 0), (0, 0, 1))


def cradle():
    ya, yb = P.ARM_IN_Y, P.ARM_IN_Y + P.ARM_T
    z_lo, z_hi = -14.0, P.Z_A + 12.0
    s = None
    for sy in (1, -1):
        arm = bxr(
            ARM_X0, ARM_X1, min(sy * ya, sy * yb), max(sy * ya, sy * yb), z_lo, z_hi
        )
        # trunnion stub through the boss and bearing (dia 7.8 printed stub, 608 ID 8 slip fit) + pulley ring on +y
        y0, y1 = sy * yb, sy * (yb + P.BOSS_L + P.B608_W)
        arm = arm + hole("y", 7.8, P.X_A, P.Z_A, min(y0, y1), max(y0, y1))
        if sy == 1:
            ring = tube(
                "y", P.TILT_PULLEY_OD / 2 + 1.6, 4.0, P.X_A, P.Z_A, yb, yb + P.BOSS_L
            )  # belt pulley 80T, teeth not modelled
            arm = arm + ring
        # D1 slot: dovetail void open to the rear, plus clamp-screw clearance through the arm
        loc = d1_loc(sy)
        m = I.male_feature(D1, D1.clear())
        arm = arm - loc * (m + Pos(-12, 0, 0) * m)
        for u, v in D1.bolts:
            arm = arm - hole(
                "y",
                P.M3_CLR,
                P.X_F1 + u,
                v,
                min(sy * (P.CORE_HALF + Z_TIP - 0.5), sy * (yb + 1)),
                max(sy * (P.CORE_HALF + Z_TIP - 0.5), sy * (yb + 1)),
            )
        s = arm if s is None else s + arm
    # front bridge (D2) over the barrel
    s = s + bxr(BRIDGE_X0, BRIDGE_X1, -ya - 0.5, ya + 0.5, BRIDGE_Z0, BRIDGE_Z1)
    s = s + d2_loc() * I.male_feature(D2)
    s = s - d2_loc() * I.bolt_holes(D2, "male")
    return s


def h1_loc():
    return I.place(
        (20.0, P.CHEEK_IN_Y + P.CHEEK_T + 6.0, P.Z_ROTOR0 + P.ROTOR_T + 24.0),
        (1, 0, 0),
        (0, 1, 0),
    )


def y1_loc(sy):
    return I.place((0.0, sy * 60.0, P.Z_ROTOR0 + P.ROTOR_T), (1, 0, 0), (0, 0, 1))


def _yoke(side):
    sy = 1 if side == "left" else -1
    y0, y1 = P.CHEEK_IN_Y, P.CHEEK_IN_Y + P.CHEEK_T
    zt = P.H_T + 14.0
    zf = P.Z_ROTOR0 + P.ROTOR_T + P.FOOT_T
    pts = [
        (-31.0, zf),
        (31.0, zf),
        (16.0, zt - 6),
        (10.0, zt),
        (-10.0, zt),
        (-16.0, zt - 6),
    ]
    if (
        side == "left"
    ):  # motor ear to the rear: NEMA face (42.3) centred on the tilt motor axis at x = -C_TILT
        pts = [
            (-31.0, zf),
            (31.0, zf),
            (16.0, zt - 6),
            (10.0, zt),
            (-P.C_TILT + 24.0, zt),
            (-P.C_TILT - 24.0, P.H_T + 24.0),
            (-P.C_TILT - 24.0, P.H_T - 24.0),
            (-31.0, P.H_T - 30.0),
        ]
    s = prism_xz(pts, min(sy * y0, sy * y1), max(sy * y0, sy * y1))
    # foot flange on the rotor (Y1 pattern centred on y = +-60) and bearing seat for the 608
    zr = P.Z_ROTOR0 + P.ROTOR_T
    fy0, fy1 = 30.0, 90.0
    s = s + bxr(
        -31, 31, min(sy * fy0, sy * fy1), max(sy * fy0, sy * fy1), zr, zr + P.FOOT_T
    )
    seat = hole(
        "y",
        P.B608_OD + 0.2,
        0,
        P.H_T,
        min(sy * y0, sy * (y0 + P.B608_W)),
        max(sy * y0, sy * (y0 + P.B608_W)),
    )
    s = s - seat
    s = s - hole(
        "y", 10.0, 0, P.H_T, min(sy * y0, sy * y1) - 1, max(sy * y0, sy * y1) + 1
    )
    loc = I.place(
        (0.0, sy * 60.0, zr), (1, 0, 0), (0, 0, 1)
    )  # Y1: yoke on the rotor top (rotor = male, yoke = female)
    s = s - loc * I.female_void(Y1)
    s = s - loc * I.bolt_holes(Y1, "female")
    if side == "left":
        # tilt motor: 31 mm pattern + 22 pilot on the outer face
        mx = -P.C_TILT
        for dx in (-1, 1):
            for dz in (-1, 1):
                s = s - hole(
                    "y", P.M3_CLR, mx + dx * 15.5, P.H_T + dz * 15.5, y0 - 1, y1 + 1
                )
        s = s - hole("y", 23.0, mx, P.H_T, y0 - 1, y1 + 1)
        # H1 head-side harness panel on the cheek outer face (connector bulkhead, same cutout as the power box)
        hl = h1_loc()
        s = s + hl * bxr(-18, 18, -8, 8, -6, 0)
        s = s - hl * I.female_void(H1)
        s = s - hl * I.bolt_holes(H1, "male")
    return s


def yoke_left():
    return _yoke("left")


def yoke_right():
    return _yoke("right")


def ballast_cup():
    """Lead-shot pocket under the valve housing (<= 60 g allowed by spec 3). Clipped on with 2 x M3; frame L."""
    return bxr(-80.0, -50.0, -15.0, 15.0, -38.0, -20.0) - bxr(
        -77.0, -53.0, -12.0, 12.0, -35.0, -19.0
    )


PARTS = {
    "cradle": dict(fn=cradle, group="tilt", count=1, material="PETG", module="M2"),
    "ballast_cup": dict(
        fn=ballast_cup, group="tilt", count=1, material="PETG", module="M2"
    ),
    "yoke_left": dict(fn=yoke_left, group="pan", count=1, material="PETG", module="M2"),
    "yoke_right": dict(
        fn=yoke_right, group="pan", count=1, material="PETG", module="M2"
    ),
}


def bought():
    out = []
    yb = P.ARM_IN_Y + P.ARM_T + P.BOSS_L
    for sy, nm in ((1, "left"), (-1, "right")):
        out.append(
            dict(
                name=f"brg_{nm}",
                lib="generic-608zz-bearing",
                mass=P.B608_MASS,
                module="M2",
                group="pan",
                shape=tube(
                    "y",
                    P.B608_OD / 2,
                    P.B608_ID / 2,
                    0,
                    P.H_T,
                    min(sy * yb, sy * (yb + P.B608_W)),
                    max(sy * yb, sy * (yb + P.B608_W)),
                ),
            )
        )
    out.append(
        dict(
            name="nema_tilt",
            lib="generic-17hs4401s-nema17-42x40",
            mass=P.N17_MASS,
            module="M2",
            group="pan",
            shape=bxr(
                -P.C_TILT - P.N17_FACE / 2,
                -P.C_TILT + P.N17_FACE / 2,
                P.CHEEK_IN_Y + P.CHEEK_T,
                P.CHEEK_IN_Y + P.CHEEK_T + P.N17_LEN,
                P.H_T - P.N17_FACE / 2,
                P.H_T + P.N17_FACE / 2,
            ),
        )
    )
    out.append(
        dict(
            name="pulley_tilt",
            lib="adafruit-1251-gt2-pulley-20t-5mm",
            mass=8.0,
            module="M2",
            group="pan",
            shape=hole("y", 16.0, -P.C_TILT, P.H_T, P.ARM_IN_Y + P.ARM_T, P.CHEEK_IN_Y),
        )
    )
    out.append(
        dict(
            name="hall_tilt",
            lib="allegro-a3144-hall-switch",
            mass=P.HALL_MASS,
            module="M2",
            group="pan",
            shape=bxr(-2, 2, P.CHEEK_IN_Y - 1.5, P.CHEEK_IN_Y, P.H_T + 28, P.H_T + 31),
        )
    )
    out.append(
        dict(
            name="magnet_tilt",
            lib="cuas-lite-magnet-disc-6x2",
            mass=P.MAGNET_MASS,
            module="M2",
            group="tilt",
            shape=hole(
                "y",
                6.0,
                P.X_A + 1.0,
                P.Z_A + 27.0,
                P.ARM_IN_Y + P.ARM_T,
                P.ARM_IN_Y + P.ARM_T + 2.0,
            ),
        )
    )
    return out
