"""build123d solids of the dog's printed parts (mm). Handed parts return the LEFT (+y outboard) version; build.py mirrors them about
the XZ plane for the right-hand twin. REGISTRY: name -> (builder, count per dog, material, handed).

Frames: leg parts are in the LEG FRAME (params.py header) unless noted; thigh / crank / shank are in their own frames (origin on the
joint axis, same orientation as the leg frame at the build pose q = 0); body parts in the body frame (x forward, y left, z up).
"""

import math

import params as P
from build123d import Axis, Box, Cylinder, Pos, Rot, Sphere, fillet


def bx(dx, dy, dz, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Box(dx, dy, dz)


def cyl_x(d, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, h)


def cyl_y(d, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Rot(90, 0, 0) * Cylinder(d / 2, h)


def cyl_z(d, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Cylinder(d / 2, h)


def arm_y(length, width, t, ang_deg, y=0.0):
    """Plate in the xz plane from the origin along the direction ang_deg (from +x toward +z), t thick in y centred on y."""
    return (
        Pos(0, y, 0)
        * Rot(0, -ang_deg, 0)
        * Pos(length / 2, 0, 0)
        * Box(length, t, width)
    )


def rounded(s, r=2.0):
    try:
        return fillet(s.edges().filter_by(Axis.Z), r)
    except Exception:  # ponytail: skip the cosmetic fillet when OCC cannot build it on boolean-cut edges
        return s


def bolt_circle_y(s, y, depth, r=None, x=0.0, z=0.0):
    r = P.BOLT_R if r is None else r
    for k in range(6):
        a = math.radians(60 * k)
        s -= cyl_y(P.M3_CLR, depth, x=x + r * math.cos(a), y=y, z=z + r * math.sin(a))
    return s


def abad_mount():
    """Abduction actuator cup. LF origin on the abad axis at the OUTER face of the flange (x in [-6, 0]); the actuator body sits in the
    cup INSIDE the body (x in [-58, 0]), its output face flush with x = 0. Open at the back; a pad behind the cup carries the leg hub.
    Symmetric about XZ."""
    f = P.HIP_FLANGE
    s = bx(6, 2 * f, 2 * f, x=-3)  # flange on the bulkhead's outer face
    s += cyl_x(P.AB_D + 8, 52, x=-32)  # cup x [-58, -6] through the bulkhead window
    s += bx(
        5, 32, 13, x=-59.5, z=-43.5
    )  # web from the cup ring down to the hub pad (x [-62, -57])
    s += bx(39, 56, 4, x=-75.5, z=-39)  # hub pad x [-95, -56], y +-28, top face z = -37
    s -= cyl_x(P.AB_D, 59, x=-28.5)  # actuator bore x [-58, 1]
    for yy in (-P.HIP_BOLT_Y, P.HIP_BOLT_Y):
        for zz in (-P.HIP_BOLT_Z, P.HIP_BOLT_Z):
            s -= cyl_x(P.M3_CLR, 8, x=-3, y=yy, z=zz)
    return rounded(s, 2.0)


def hip_housing():
    """Abduction link, left hand. Origin on the abad axis at the abad actuator OUTPUT face (x = 0): a flange disc, a web to the hip
    pitch axis (x = PITCH_X) and two sleeves on that axis: the hip actuator (output outboard, y = PITCH_Y) and the knee actuator
    (output inboard, y = KNEE_FACE) face to face, joined by a bridge over the top. The thigh, rod and crank live in the slab between."""
    hy0, hy1 = P.PITCH_Y - 55.0, P.PITCH_Y - 1.0  # hip sleeve y range
    ky0, ky1 = P.KNEE_FACE + 1.0, P.KNEE_FACE + 55.0  # knee sleeve y range
    od = P.AB_D + 8
    s = cyl_x(60, 8, x=4)  # flange disc on the abad output (6 x M3 on a 50 mm circle)
    s += bx(13, 50, 40, x=6.5, y=25)  # web to the hip sleeve
    s += cyl_y(od, hy1 - hy0, x=P.PITCH_X, y=(hy0 + hy1) / 2)
    s += cyl_y(od, ky1 - ky0, x=P.PITCH_X, y=(ky0 + ky1) / 2)
    s += bx(
        40, ky1 - hy0, 10, x=P.PITCH_X, y=(hy0 + ky1) / 2, z=49
    )  # bridge z [44, 54]
    s -= cyl_y(P.AB_D, hy1 - hy0 + 2, x=P.PITCH_X, y=(hy0 + hy1) / 2)
    s -= cyl_y(P.AB_D, ky1 - ky0 + 2, x=P.PITCH_X, y=(ky0 + ky1) / 2)
    for k in range(6):
        a = math.radians(60 * k)
        s -= cyl_x(
            P.M3_CLR, 10, x=4, y=P.BOLT_R * math.cos(a), z=P.BOLT_R * math.sin(a)
        )
    return rounded(s, 2.0)


def thigh_bracket():
    """Thigh link, left hand. Origin on the hip pitch axis at the hip motor output face; the plate lies outboard (y in [0, THIGH_T]) and
    hangs along -z for L_THIGH to the knee axis. The shank hinges at y = KNEE_Y on the outboard side; ribs stiffen the plate."""
    L, T = P.L_THIGH, P.THIGH_T
    s = cyl_y(60, T, y=T / 2)  # hip-side disc with the actuator bolt circle
    s += bx(24, T, L, y=T / 2, z=-L / 2)
    s += cyl_y(40, T, y=T / 2, z=-L)
    s += bx(6, 7, L - 100, y=T + 3.5, z=-30 - (L - 100) / 2)  # outboard rib
    s = bolt_circle_y(s, T / 2, T + 2)
    s -= cyl_y(P.M5_CLR, T + 2, y=T / 2, z=-L)  # knee pin
    return rounded(s, 2.0)


def knee_crank():
    """Crank on the knee actuator output, left hand. Origin on the axis at the output flange face; the plate lies inboard (y in
    [-CRANK_T, 0]); the arm reaches the rod-end pin at KNEE_R1 / ALPHA0_DEG."""
    t = P.CRANK_T
    s = cyl_y(60, t, y=-t / 2)
    s += arm_y(P.KNEE_R1, 14, t, P.ALPHA0_DEG, y=-t / 2)
    cx, cz = (
        P.KNEE_R1 * math.cos(math.radians(P.ALPHA0_DEG)),
        P.KNEE_R1 * math.sin(math.radians(P.ALPHA0_DEG)),
    )
    s += cyl_y(14, t, x=cx, y=-t / 2, z=cz)
    s = bolt_circle_y(s, -t / 2, t + 2)
    s -= cyl_y(P.M5_CLR, t + 2, x=cx, y=-t / 2, z=cz)  # M5 rod-end pin
    return s


def shank():
    """Shank: origin on the knee axis (y centre of the beam), foot socket at z = -L_SHANK, lug block above the knee on BOTH faces
    (so it is its own mirror) with the rod-end pin at KNEE_R2 / LUG_DEG. Symmetric about the XZ plane."""
    L, w = P.L_SHANK, P.SHANK_W
    b = math.radians(P.LUG_DEG)
    px, pz = P.KNEE_R2 * math.cos(b), P.KNEE_R2 * math.sin(b)
    s = bx(22, w, L, z=-L / 2)
    s += cyl_y(30, w, z=0)
    s += bx(
        14, w + 2 * P.LUG_T, pz + 7, x=px, z=(pz + 7) / 2
    )  # lug block (z up to the pin + 7)
    s -= cyl_y(P.M5_CLR, w + 8, z=0)  # knee pin
    s -= cyl_y(P.M5_CLR, w + 8, x=px, z=pz)  # rod-end pin
    s -= bx(12, 6, L - 60, z=-L / 2)  # lightening slot
    s += cyl_z(34, 20, z=-L + 6)  # foot socket stub
    s -= cyl_z(8.4, 12, z=-L + 12)  # M8-class stud bore for the foot core
    return rounded(s, 2.0)


def foot_core():
    """Foot core: origin at the ball centre, shank stub enters from +z. Symmetric."""
    s = cyl_z(46, 30, z=10)
    s -= Pos(0, 0, 0) * Sphere(P.BALL_D / 2)
    s -= cyl_z(8.4, 24, z=18)
    s -= cyl_z(P.BALL_D * 0.8, 6, z=-17)  # ball retention lip clearance
    return s


def head_mount():
    """Lidar + depth-camera head plate: origin at the bottom-centre of the base plate, +x forward. Mid-360 seat tilted nose-down by
    TILT_DEG; a vertical plate at the nose carries the D435i (looks forward). Symmetric about XZ."""
    s = bx(130, 100, 4, z=2)
    seat = Pos(-10, 0, 40) * Rot(0, P.TILT_DEG, 0) * bx(80, 80, 6, z=3)
    s += seat
    for sx in (-30, 30):
        s += bx(6, 70, 38, x=sx - 10, z=21)  # two ribs under the seat
    s += bx(4, 100, 40, x=67, z=20)  # camera plate x [65, 69]
    for xx in (-1, 1):
        for yy in (-1, 1):
            s -= cyl_z(P.M3_CLR, 8, x=xx * 55 + 0, y=yy * 40, z=2)
    return s


def battery_tray():
    """Battery tray for the 86 x 79 x 43 mm 8S pack: origin at the tray floor centre. Symmetric."""
    s = bx(100, 93, 50, z=25)
    s -= bx(92, 85, 50, z=29)  # 3 mm floor, 4 mm walls
    s -= bx(30, 50, 8, x=50, z=4)  # lead exit
    return rounded(s, 3.0)


def side_plate():
    """Chassis side plate (4 mm), 420 x 120, left hand: origin at the plate centre, +x forward, the plate normal along y; the
    flange along the top edge points INBOARD (-y for the left plate) and carries the plate on the top deck. A 60 mm service hole
    for the e-stop head."""
    s = bx(P.BODY_L, 4, P.BULK_H)
    s += bx(P.BODY_L, 12, 4, y=-8, z=P.BULK_H / 2 - 2)
    s -= cyl_y(60, 6, x=-140, z=-18)
    return rounded(s, 3.0)


def bulkhead():
    """Front / rear bulkhead in the YZ plane: origin at its centre; two windows for the abad cups at y = +-HIP_Y with the 4 x M3
    pattern; deck flanges toward the inside (-x) at z = +-(BULK_H / 2 - 2). Symmetric about XZ (one part, front and rear)."""
    s = bx(P.BULK_T, P.BULK_W, P.BULK_H)
    for sz in (-1, 1):
        s += bx(14, 184, 4, x=-10, z=sz * (P.BULK_H / 2 - 2))
    for sy in (-1, 1):
        s -= cyl_x(P.AB_D + 9, P.BULK_T + 2, y=sy * P.HIP_Y)
        for yy in (-P.HIP_BOLT_Y, P.HIP_BOLT_Y):
            for zz in (-P.HIP_BOLT_Z, P.HIP_BOLT_Z):
                s -= cyl_x(P.M3_CLR, P.BULK_T + 2, y=sy * P.HIP_Y + yy, z=zz)
    return rounded(s, 2.0)


def compute_tray():
    """Electronics tray on the top deck: Jetson devkit, 4 CANables, MinIMU, RTK board, antenna pad. Origin at the floor underside
    centre, +x forward. Symmetric."""
    s = bx(250, 160, 3, z=1.5)
    for sy in (-1, 1):
        s += bx(250, 3, 12, y=sy * 78.5, z=6)  # side walls
    return s


# name -> (builder, count per dog, material, handed)
REGISTRY = {
    "dog-abad-mount": (abad_mount, 4, "pla", False),
    "dog-hip-housing": (hip_housing, 4, "pla", True),
    "dog-thigh-bracket": (thigh_bracket, 4, "pla", True),
    "dog-knee-crank": (knee_crank, 4, "pla", True),
    "dog-shank": (shank, 4, "pla", False),
    "dog-foot-core": (foot_core, 4, "pla", False),
    "dog-head-mount": (head_mount, 1, "pla", False),
    "dog-battery-tray": (battery_tray, 2, "pla", False),
    "dog-side-plate": (side_plate, 2, "pla", True),
    "dog-bulkhead": (bulkhead, 2, "pla", False),
    "dog-compute-tray": (compute_tray, 1, "pla", False),
}
