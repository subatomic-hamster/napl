"""build123d solids of the dog's printed parts (mm). Handed parts return the LEFT (+y outboard) version; build.py mirrors them about
the XZ plane for the right-hand twin. REGISTRY: name -> (builder, count per dog, material, handed)."""

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


def rounded(s, r=2.0):
    try:
        return fillet(s.edges().filter_by(Axis.Z), r)
    except Exception:  # ponytail: skip the cosmetic fillet when OCC cannot build it on boolean-cut edges
        return s


def hip_housing():
    """Abduction actuator clamp: origin on the abduction axis at the chassis-side face (x = 0), body toward +x. Outboard boss (+y) carries the hip pitch actuator."""
    s = bx(70, 100, 110, x=35)
    s -= cyl_x(P.AB_D, 56, x=28)  # actuator bore, 56 mm deep (92 x 55 body)
    s -= cyl_x(60, 20, x=60)  # cable / output opening
    s += cyl_y(
        50, 30, x=35, y=65
    )  # outboard boss for the pitch actuator seat (breaks mirror symmetry)
    for yy in (-40, 40):
        for zz in (-45, 45):
            s -= cyl_x(P.M3_CLR, 70, x=35, y=yy, z=zz)  # chassis bolts
    return rounded(s, 3.0)


def thigh_bracket():
    """Thigh link: origin on the hip pitch axis, hangs along -z for L_THIGH to the knee axis; knee drive housing bolts on the outboard face."""
    L = P.L_THIGH
    s = bx(24, 12, L + 40, y=8, z=-L / 2 + 0)
    s += cyl_y(60, 12, y=8)  # hip-side disc with the actuator bolt circle
    for k in range(6):
        a = math.radians(60 * k)
        s -= cyl_y(P.M3_CLR, 14, x=25 * math.cos(a), y=8, z=25 * math.sin(a))
    s -= cyl_y(
        30.4, 8, y=11, z=-L
    )  # knee bearing seat (6903 class 30 mm OD), on the outboard side
    s += bx(30, 6, 40, x=0, y=14, z=-L)  # knee-side pad (breaks mirror symmetry)
    return rounded(s, 2.0)


def knee_drive_housing():
    """Knee actuator pocket + belt cavity at the thigh's lower end: origin on the knee axis."""
    s = bx(90, 50, 60, x=-10, y=0, z=0)
    s -= cyl_y(P.KN_D, 31, x=-25, y=-6)  # GIM6010 pocket, 80 x 40 body
    s -= bx(40, 20, 16, x=30, y=-10, z=0)  # belt window
    s += cyl_y(40, 8, x=20, y=29)  # outboard belt-cover boss (breaks mirror symmetry)
    for yy in (-15, 15):
        for xx in (-35, 15):
            s -= cyl_z(P.M3_CLR, 70, x=xx, y=yy)
    return rounded(s, 2.0)


def shank():
    """Shank beam: origin on the knee axis, foot socket at z = -L_SHANK. Symmetric about the XZ plane."""
    L = P.L_SHANK
    s = bx(22, 14, L, z=-L / 2)
    s += cyl_y(30, 14, z=0)
    s -= cyl_y(P.M3_CLR, 16, z=0)
    s -= bx(12, 8, L - 60, z=-L / 2)  # lightening slot
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
    """Lidar + depth-camera head plate: origin at the bottom-centre of the base plate, +x forward. Mid-360 seat tilted nose-down by TILT_DEG. Symmetric."""
    s = bx(130, 100, 4, z=2)
    seat = Rot(0, P.TILT_DEG, 0) * (bx(80, 80, 6, z=3))
    seat = Pos(-10, 0, 40) * seat
    s += seat
    for sx in (-30, 30):
        s += bx(6, 70, 38, x=sx - 10, z=21)  # two ribs under the seat
    s += bx(30, 60, 6, x=55, z=33)  # D435i shelf at the nose (camera 90 x 25 x 25 mm)
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
    """Chassis side plate (4 mm), 290 x 120: origin at the plate centre; the inboard flange (+y) is the handed feature."""
    s = bx(290, 4, 120)
    s += bx(290, 12, 4, y=8, z=58)  # top flange toward the inside
    for xx in (-100, 100):
        s -= cyl_y(P.AB_D - 8, 6, x=xx, z=0)  # hip window
        for a in range(4):
            s -= cyl_y(
                P.M3_CLR,
                6,
                x=xx + 40 * (1 if a % 2 else -1),
                z=40 * (1 if a // 2 else -1),
            )
    return rounded(s, 3.0)


REGISTRY = {
    "dog-hip-housing": (hip_housing, 4, "pla", True),
    "dog-thigh-bracket": (thigh_bracket, 4, "pla", True),
    "dog-knee-drive-housing": (knee_drive_housing, 4, "pla", True),
    "dog-shank": (shank, 4, "pla", False),
    "dog-foot-core": (foot_core, 4, "pla", False),
    "dog-head-mount": (head_mount, 1, "pla", False),
    "dog-battery-tray": (battery_tray, 1, "pla", False),
    "dog-side-plate": (side_plate, 2, "pla", True),
}
