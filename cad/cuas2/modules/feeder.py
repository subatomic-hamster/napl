"""M4 FEEDER (spec 5): head-mounted 24-ball hopper, one-ball-wide throat, single-pocket plug-valve gate (SG92R), pusher rod on a
scotch yoke (SG92R), breech beam, two hall homes. Frame L. F1 flange on the cartridge rear face.

Plug axis along y at (XV, 0); the pocket is a diametral through-bore of length POCKET_LEN = plug diameter. BORE pose: bore along x
(coaxial with the barrel). LOAD pose: plug turned GATE_SWING_DEG so that the FRONT end is up (bore descends toward the rear): the
mouth sits under the throat and the ball is held at the rear end by gravity at every tilt (see checks.C-FEED).
"""

import math

import interfaces as I
import params as P
from build123d import Rot
from prims import about, bxr, hole, prism_xz, tube

F1 = I.DOCKS["F1"]
ROD_LEN = 60.0
YOKE_X_RET = P.ROD_TIP_RET - ROD_LEN  # rear end of the rod when retracted
SERVO_P_X = YOKE_X_RET + P.PUSH_STROKE / 2 + 0  # pusher servo spline x (crank centre)
SERVO_P_Z = -34.0
GATE_SERVO_Y0 = P.PLUG_L / 2 + P.HOUSING_WALL + P.PLUG_CLR + 0.8
Y_HOUSE = P.PLUG_L / 2 + P.HOUSING_WALL + P.PLUG_CLR  # housing half width (y)
# Breech beam: along y through the staged ball but 9 mm behind the plug axis, because the gate servo spline sits ON the plug axis (+y side).
# At BORE the beam hole in the plug lines up with the housing holes; the gate-home magnet pocket sits on the -y end face at +9 mm (LOAD turns it to the hall).
BEAM_DX = -9.0
MAG_R = 9.0
Z_HOP_TOP = P.Z_HOP0 + (
    (P.HOPPER_BALLS * math.pi / 6 * P.BALL_D**3 / P.PACKING) / P.HOPPER_W
    + 0.5
    * (P.X_HOP_FRONT - (P.X_T + P.THROAT_D / 2)) ** 2
    * math.tan(math.radians(P.HOPPER_ALPHA))
) / (P.X_HOP_FRONT - P.X_HOP_REAR)


def poly_centroid(pts):
    a = cx = cz = 0.0
    for (x0, z0), (x1, z1) in zip(pts, pts[1:] + pts[:1]):
        c = x0 * z1 - x1 * z0
        a += c
        cx += (x0 + x1) * c
        cz += (z0 + z1) * c
    return cx / (3 * a), cz / (3 * a)


def f1_loc():
    return I.place((P.X_F1, 0.0, 0.0), (0, 1, 0), (-1, 0, 0))


def hopper_inner():
    """Interior polygon (x, z) of the hopper: rear-bottom outlet, floor facet rising forward at HOPPER_ALPHA."""
    xt_f = P.X_T + P.THROAT_D / 2
    zf = P.Z_HOP0 + (P.X_HOP_FRONT - xt_f) * math.tan(math.radians(P.HOPPER_ALPHA))
    return [
        (P.X_HOP_REAR, P.Z_HOP0),
        (xt_f, P.Z_HOP0),
        (P.X_HOP_FRONT, zf),
        (P.X_HOP_FRONT, Z_HOP_TOP),
        (P.X_HOP_REAR, Z_HOP_TOP),
    ]


def valve_housing():
    x0 = P.XV - P.PLUG_R - P.HOUSING_WALL
    s = bxr(x0, P.X_F1 - 6.0, -Y_HOUSE, Y_HOUSE, -P.HOUSING_TOP, P.HOUSING_TOP)
    s = s + bxr(P.X_F1 - 6.0, P.X_F1, -32, 32, -26, 26)  # F1 flange plate
    # throat tube on top
    s = s + tube(
        "z",
        P.THROAT_D / 2 + P.THROAT_WALL,
        P.THROAT_D / 2,
        P.X_T,
        0,
        P.HOUSING_TOP - 0.01,
        P.Z_HOP0,
    )
    # gate servo bracket on +y (spline on the plug axis; servo body outside the housing)
    s = s + bxr(P.XV - 10, P.XV + 19, Y_HOUSE - 1, GATE_SERVO_Y0, -9, 9)
    # rear guide tube for the rod + pusher servo bracket
    s = s + tube("x", 13.0, 10.5, 0, 0, YOKE_X_RET + 22, x0 + 0.01)
    s = s + bxr(
        SERVO_P_X - 10, SERVO_P_X + 19, 12.5, 14.5, SERVO_P_Z - 9, 2
    )  # bracket plate hanging off the guide tube (+y side)
    # cavity, front channel, rear aperture, throat bore, beam holes
    s = s - hole(
        "y",
        2 * (P.PLUG_R + P.PLUG_CLR),
        P.XV,
        0,
        -P.PLUG_L / 2 - 0.3,
        P.PLUG_L / 2 + 0.3,
    )
    s = s - hole("x", P.BORE_ID, 0, 0, P.XV, P.X_F1 + 1)
    s = s - hole("x", P.APERTURE_D, 0, 0, YOKE_X_RET + 21, P.XV)
    s = s - hole("z", P.THROAT_D, P.X_T, 0, 0, P.Z_HOP0 + 1)
    s = s - hole(
        "y", 5.2, P.XV + BEAM_DX, 0, -Y_HOUSE - 1, GATE_SERVO_Y0 + 1
    )  # breech beam LED/PT seats (the LED passes the servo bracket)
    # axle clearance for the plug stub toward the servo
    s = s - hole("y", 6.0, P.XV, 0, 0, GATE_SERVO_Y0 + 1)
    # F1: counterbore for the ring lip + clearance holes
    f = f1_loc()
    s = s - f * I.female_void(F1)
    s = s - f * I.bolt_holes(F1, "female")
    s = s - hole("x", P.BORE_ID, 0, 0, P.X_F1 - 7, P.X_F1 + 1)
    return s


def valve_plug(deg=0.0):
    """Plug at swing `deg` from BORE (0) toward LOAD (GATE_SWING_DEG)."""
    p = hole("y", 2 * P.PLUG_R, P.XV, 0, -P.PLUG_L / 2, P.PLUG_L / 2)
    p = p + hole(
        "y", 5.0, P.XV, 0, P.PLUG_L / 2 - 0.5, P.PLUG_L / 2 + 9.0
    )  # axle stub to the servo horn
    p = p - hole("x", P.POCKET_BORE_D, 0, 0, P.XV - P.PLUG_R - 1, P.XV + P.PLUG_R + 1)
    p = p - hole(
        "y", 3.2, P.XV + BEAM_DX, 0, -P.PLUG_L / 2 - 1, P.PLUG_L / 2 + 1
    )  # breech beam cross hole (aligned at BORE)
    p = p - hole(
        "y", 6.1, P.XV + MAG_R, 0, -P.PLUG_L / 2 - 1, -P.PLUG_L / 2 + 2.0
    )  # gate-home magnet pocket in the -y end face
    if deg:
        p = about((P.XV, 0, 0), Rot(0, -deg, 0)) * p
    return p


def pusher_rod(ext=False):
    """Rod: dia ROD_D, soft pad at the tip, scotch-yoke plate with a vertical slot at the rear."""
    tip = P.ROD_TIP_EXT if ext else P.ROD_TIP_RET
    r = hole("x", P.ROD_D, 0, 0, tip - ROD_LEN, tip)
    xy = tip - ROD_LEN
    yoke = bxr(xy - 4, xy, -6, 6, SERVO_P_Z - 28, 6)
    yoke = yoke - bxr(
        xy - 5, xy + 1, -2.5, 2.5, SERVO_P_Z - 22, SERVO_P_Z + 22
    )  # crank-pin slot (pin 4 mm, slot 5 mm, 44 mm)
    return r + yoke


def hopper():
    pts = hopper_inner()
    ti = math.tan(math.radians(P.HOPPER_ALPHA))
    xt_f = pts[1][0]
    w = P.HOPPER_T
    drop = w / math.cos(math.radians(P.HOPPER_ALPHA))
    outer = [
        (P.X_HOP_REAR - w, P.Z_HOP0 - drop),
        (xt_f, P.Z_HOP0 - drop),
        (P.X_HOP_FRONT + w, pts[2][1] - drop + w * ti),
        (P.X_HOP_FRONT + w, Z_HOP_TOP + w),
        (P.X_HOP_REAR - w, Z_HOP_TOP + w),
    ]
    o = prism_xz(outer, -P.HOPPER_W / 2 - w, P.HOPPER_W / 2 + w)
    i = prism_xz(pts, -P.HOPPER_W / 2, P.HOPPER_W / 2)
    s = o - i
    s = s - hole(
        "z", P.THROAT_D, P.X_T, 0, P.Z_HOP0 - 5, P.Z_HOP0 + 1
    )  # outlet through the floor (rear-bottom corner)
    return s


def hopper_lid():
    return bxr(
        P.X_HOP_REAR - 2.0,
        P.X_HOP_FRONT + 2.0,
        -P.HOPPER_W / 2 - 2.0,
        P.HOPPER_W / 2 + 2.0,
        Z_HOP_TOP + 2.0,
        Z_HOP_TOP + 4.0,
    ) + bxr(
        P.X_HOP_REAR + 3,
        P.X_HOP_FRONT - 3,
        -P.HOPPER_W / 2 + 1,
        P.HOPPER_W / 2 - 1,
        Z_HOP_TOP,
        Z_HOP_TOP + 2.0,
    )


PARTS = {
    "valve_housing": dict(
        fn=valve_housing, group="tilt", count=1, material="PETG", module="M4"
    ),
    "valve_plug": dict(
        fn=valve_plug, group="tilt", count=1, material="PETG", module="M4"
    ),
    "pusher_rod": dict(
        fn=pusher_rod, group="tilt", count=1, material="PETG", module="M4"
    ),
    "hopper": dict(fn=hopper, group="tilt", count=1, material="PETG", module="M4"),
    "hopper_lid": dict(
        fn=hopper_lid, group="tilt", count=1, material="PLA", module="M4"
    ),
}


def _servo_box(cx, y0, y1, cz):
    return bxr(cx - 6.0, cx + 16.7, y0, y1, cz - 6.1, cz + 6.1)


def bought():
    out = []
    out.append(
        dict(
            name="servo_gate",
            lib="towerpro-sg92r",
            mass=P.SERVO_MASS,
            module="M4",
            group="tilt",
            shape=_servo_box(P.XV, GATE_SERVO_Y0, GATE_SERVO_Y0 + P.SERVO_BODY[1], 0.0),
        )
    )
    out.append(
        dict(
            name="servo_push",
            lib="towerpro-sg92r",
            mass=P.SERVO_MASS,
            module="M4",
            group="tilt",
            shape=bxr(
                SERVO_P_X - 6.0,
                SERVO_P_X + 16.7,
                14.5,
                14.5 + P.SERVO_BODY[1],
                SERVO_P_Z - 6.1,
                SERVO_P_Z + 6.1,
            ),
        )
    )
    # breech beam LED (+y) and phototransistor (-y)
    for sy, lib, nm in (
        (1, "ir-led-940nm-5mm", "breech_led"),
        (-1, "ir-phototransistor-940nm-5mm", "breech_pt"),
    ):
        out.append(
            dict(
                name=nm,
                lib=lib,
                mass=P.LED_MASS,
                module="M4",
                group="tilt",
                shape=hole(
                    "y",
                    5.0,
                    P.XV + BEAM_DX,
                    0,
                    min(sy * (Y_HOUSE - 3.5), sy * (Y_HOUSE + 4.0)),
                    max(sy * (Y_HOUSE - 3.5), sy * (Y_HOUSE + 4.0)),
                ),
            )
        )
    # halls + magnets: gate home (plug stub magnet at LOAD), push home (magnet on the yoke)
    ang = math.radians(P.GATE_SWING_DEG)
    mx, mz = P.XV + MAG_R * math.cos(ang), MAG_R * math.sin(ang)
    out.append(
        dict(
            name="hall_gate",
            lib="allegro-a3144-hall-switch",
            mass=P.HALL_MASS,
            module="M4",
            group="tilt",
            shape=bxr(
                mx - 2, mx + 2, -(Y_HOUSE + 4), -(Y_HOUSE + 1), mz - 1.5, mz + 1.5
            ),
        )
    )
    out.append(
        dict(
            name="magnet_gate",
            lib="cuas-lite-magnet-disc-6x2",
            mass=P.MAGNET_MASS,
            module="M4",
            group="tilt",
            shape=hole("y", 6.0, mx, mz, -P.PLUG_L / 2, -P.PLUG_L / 2 + 2.0),
        )
    )
    xy = YOKE_X_RET
    out.append(
        dict(
            name="hall_push",
            lib="allegro-a3144-hall-switch",
            mass=P.HALL_MASS,
            module="M4",
            group="tilt",
            shape=bxr(xy - 6, xy - 2, 8.0, 11.0, SERVO_P_Z + 9.5, SERVO_P_Z + 12.5),
        )
    )
    out.append(
        dict(
            name="magnet_push",
            lib="cuas-lite-magnet-disc-6x2",
            mass=P.MAGNET_MASS,
            module="M4",
            group="tilt",
            shape=hole("y", 6.0, xy - 2, SERVO_P_Z + 11.0, 6.0, 8.0),
        )
    )
    cx, cz = poly_centroid(
        hopper_inner()
    )  # full hopper = 24 balls fill the interior (229 cm3 at packing 0.6)
    out.append(
        dict(
            name="balls",
            lib=None,
            mass=P.BALLS_G,
            module="M4",
            group="tilt",
            shape=bxr(cx - 15, cx + 15, -20, 20, cz - 15, cz + 15),
        )
    )
    return out
