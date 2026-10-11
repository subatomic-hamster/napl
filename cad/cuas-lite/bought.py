"""Envelopes of the bought (non-printed) items, for the CG balance, the swept-clearance test and the preview.

Each entry: name, group (static | pan | tilt), mass_g, shape in the group's frame (tilt: head frame H), est = estimated dims.
Hardware that sits on the axes (rods, bearings) is included; fasteners/wiring are one `hardware` allowance per group.
"""

import math

import params as P
from build123d import Box, Cylinder, Pos, Rot

import parts as K

HARDWARE_TILT_G = 25.0  # screws, inserts, zip ties, cable on the head [est], placed at the head CG (no moment)


def _cyl_z(d, z0, z1, x, y):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


def _cyl_y(d, y0, y1, x, z):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(d / 2, y1 - y0)


def head_items():
    """Bought items on the tilting head, in H."""
    yw = P.FW_GAP / 2 + P.WHEEL_D / 2
    items = []
    for s in (-1, 1):
        items.append(
            dict(
                name=f"fw_motor_{'p' if s > 0 else 'n'}y",
                mass=P.M180_MASS,
                shape=K.L2H(_cyl_z(P.M180_D, 4.5, 4.5 + P.M180_LEN, 0, s * yw)),
            )
        )
        items.append(
            dict(
                name=f"fw_wheel_{'p' if s > 0 else 'n'}y",
                mass=P.WHEEL_MASS,
                shape=K.L2H(
                    _cyl_z(P.WHEEL_D, -P.WHEEL_W / 2, P.WHEEL_W / 2, 0, s * yw)
                ),
            )
        )
    bx, by, bz = P.SERVO["body"]
    yc = K.Y_S - (by / 2 - P.SERVO["spline_from_end"])
    zt = K.Z_SERVO_PLATE + 4.8
    items.append(
        dict(
            name="servo_sg90",
            mass=P.SERVO["mass"],
            shape=K.L2H(Pos(K.X_S, yc, zt - bz / 2) * Box(bx, by, bz)),
        )
    )
    bd, bw, bh = P.BRIO
    zc = K.CAM_Z0 + P.CAM_PLATE_T + bh / 2
    items.append(
        dict(
            name="brio",
            mass=P.BRIO_MASS,
            shape=K.L2H(Pos(K.MUZZLE_X - bd / 2, 0, zc) * Box(bd, bw, bh)),
        )
    )
    y0, y1 = (
        YO_ROD,
        K.Y_P0,
    )  # left rod only: the right trunnion is the hollow beam tube in a 6807 bearing
    items.append(
        dict(name="rod_left", mass=0.39 * (y1 - y0), shape=_cyl_y(8.0, y0, y1, 0, 0))
    )
    return items


YO_ROD = K.YO - 11.0


def balls_cg_B():
    """Centroid (B, pan group) of the ball load in the hopper (cone + cylinder, packing 0.6), by slicing."""
    h = K.HOPPER
    rn, Hs, _, _ = K._hopper_dims()
    z0 = K.elbow_dims()[2] + h["Hn"]
    num = den = 0.0
    n = 400
    for i in range(n):
        t = (i + 0.5) / n
        z = z0 + t * h["Hc"]
        r = rn + (h["R_top"] - rn) * t
        v = math.pi * r * r * h["Hc"] / n
        num += v * z
        den += v
    for i in range(n):
        z = z0 + h["Hc"] + (i + 0.5) / n * Hs
        v = math.pi * h["R_top"] ** 2 * Hs / n
        num += v * z
        den += v
    return (0.0, -P.ELBOW_YH, num / den)


def pan_items():
    """Bought items that rotate with the pan axis (frame B)."""
    ha, zc = P.H_T, P.H_T - P.C_TILT
    y_mp1 = K.Y_MP0 + K.MP_T
    items = [
        dict(
            name="lazy_susan",
            mass=P.LS_MASS,
            shape=(
                _cyl_z(P.LS_OD, P.Z_LS0, P.Z_ROTOR0, 0, 0)
                - _cyl_z(P.LS_APERTURE, P.Z_LS0 - 1, P.Z_ROTOR0 + 1, 0, 0)
            ),
        ),
        dict(
            name="tilt_motor",
            mass=P.N17_MASS,
            shape=Pos(0, y_mp1 + P.N17_LEN / 2, zc)
            * Box(P.N17_FACE, P.N17_LEN, P.N17_FACE),
        ),
        dict(
            name="tilt_motor_pulley",
            mass=8.0,
            shape=_cyl_y(16.0, y_mp1 - 20.0, K.Y_MP0 - 1.0, 0, zc),
        ),
        dict(
            name="bearing_left",
            mass=P.B608_MASS,
            shape=_cyl_y(P.B608_OD, K.Y_C1 - P.B608_W, K.Y_C1, 0, ha)
            - _cyl_y(8.2, K.Y_C1 - P.B608_W - 1, K.Y_C1 + 1, 0, ha),
        ),
        dict(
            name="bearing_right",
            mass=P.B6807_MASS,
            shape=_cyl_y(P.B6807_OD, -K.Y_C1, -(K.Y_C1 - P.B6807_W), 0, ha)
            - _cyl_y(P.B6807_ID, -K.Y_C1 - 1, -(K.Y_C1 - P.B6807_W) + 1, 0, ha),
        ),
    ]
    return items


def static_items():
    c = P.C_PAN
    return [
        dict(
            name="pan_motor",
            mass=P.N17_MASS,
            shape=Pos(c, 0, -P.STATOR_T - P.MOUNT_T - P.N17_LEN / 2)
            * Box(P.N17_FACE, P.N17_FACE, P.N17_LEN),
        ),
        dict(name="pan_motor_pulley", mass=8.0, shape=_cyl_z(16.0, 1.0, 15.0, c, 0)),
        dict(  # M3 threaded stud, the pan hard stop
            name="pan_stop_stud",
            mass=1.0,
            shape=_cyl_z(P.PAN_STOP_STUD_D, 0.0, P.PAN_STOP_STUD_H, 0.0, -P.PAN_STOP_R),
        ),
    ]
