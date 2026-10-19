"""Revision-2 "-al" leg parts (docs/dog/leg-survey.md): a Go2-style hollow thigh shell, a carbon-tube shank, a PA12-CF foot core (mm).

Thigh (left): two 0.063 in 6061 side plates (inboard y [0, AL_PLATE], outboard y [AL_Y - AL_PLATE, AL_Y]) with the outline
of a convex hull of circles (AL_HULL), spaced by two printed PA12-CF bands; the knee push rod and the shank tube run in the cavity between
the plates. A 3/16 in doubler ring carries the hip moment, knee pin pads sit on the outer plate faces. Shank (left): 6061 head (knee
boss, tube socket, rod-pin ear) + 25 x 23 carbon tube. Frames as parts.py: thigh origin on the hip axis at the hip output face; shank
origin on the knee axis, y = 0 on the tube centre line (y = SH_Y in the thigh frame).
"""

import math

import params as P
from build123d import Box, Cylinder, Plane, Polygon, Pos, Rot, Sphere, extrude


def bx(dx, dy, dz, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Box(dx, dy, dz)


def cyl_y(d, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Rot(90, 0, 0) * Cylinder(d / 2, h)


def cyl_z(d, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Cylinder(d / 2, h)


def hull2d(circles, n=96):
    """Convex hull (CCW list of (x, z)) of circles given as (x, z, r)."""
    pts = sorted(
        {
            (
                round(cx + r * math.cos(2 * math.pi * k / n), 4),
                round(cz + r * math.sin(2 * math.pi * k / n), 4),
            )
            for cx, cz, r in circles
            for k in range(n)
        }
    )

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def prism(pts, y0, y1):
    """Polygon pts = [(x, z)] extruded over y in [y0, y1] (Plane.XZ-style sketch, normal -y, so it starts at y1 and runs toward y0)."""
    pl = Plane(origin=(0, y1, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    return extrude(pl * Polygon(*pts, align=None), y1 - y0)


def _outline(inset=0.0):
    return hull2d([(x, z, r - inset) for x, z, r in P.AL_HULL])


def _thigh_split():
    """(aluminium solid, printed-band solid) of the thigh, left hand."""
    L, Y, PL = P.L_THIGH, P.AL_Y, P.AL_PLATE
    out, inn = _outline(), _outline(P.AL_BAND)
    pin = P.KNEE_PIN_D + 0.1
    inboard = prism(out, 0, PL)
    dbl = prism(hull2d([(0, 0, 30), (0, -40, 16)]), PL, PL + P.AL_HIP_PAD)
    dbl -= cyl_y(28, 30, y=PL)  # ring: the neck below stays solid
    outboard = prism(out, Y - PL, Y) & bx(300, 200, 300, y=Y / 2, z=-P.AL_TOP - 150)
    al = inboard + dbl + outboard
    al += cyl_y(32, P.AL_PAD_T, y=-P.AL_PAD_T / 2, z=-L)
    al += cyl_y(32, P.AL_PAD_T, y=Y + P.AL_PAD_T / 2, z=-L)
    for k in range(6):  # actuator bolt circle through the plate and the doubler
        a = math.radians(60 * k)
        al -= cyl_y(
            P.M3_CLR, 20, x=P.BOLT_R * math.cos(a), y=PL, z=P.BOLT_R * math.sin(a)
        )
    al -= cyl_y(pin, 80, y=Y / 2, z=-L)
    ring = prism(out, PL, Y - PL) - prism(inn, PL - 1, Y - PL + 1)
    rear = ring & bx(
        100 + P.AL_BAND_END_X,
        200,
        400,
        x=(-100 + P.AL_BAND_END_X) / 2,
        y=Y / 2,
        z=-P.AL_TOP - 200,
    )
    zf = (
        L - 24.0
    )  # front band ends 24 mm above the knee axis (the flexed shank tube sweeps below it)
    front = ring & bx(100, 200, zf - P.AL_TOP, x=50, y=Y / 2, z=-(P.AL_TOP + zf) / 2)
    return al, rear + front


def thigh_al():
    al, pa = _thigh_split()
    return al + pa


def thigh_al_aluminium():
    return _thigh_split()[0]


def thigh_al_bands():
    return _thigh_split()[1]


def thigh_al_mass_g():
    """Plates + doubler + pads solid 6061, bands at the printed effective fraction BAND_FILL."""
    al, pa = _thigh_split()
    return (
        al.volume * P.DENSITY["al6061"] + pa.volume * P.DENSITY["pa12cf"] * P.BAND_FILL
    )


def _shank_head():
    """6061 head, left hand, shank frame."""
    b = math.radians(P.LUG_DEG)
    px, pz = P.KNEE_R2 * math.cos(b), P.KNEE_R2 * math.sin(b)
    s = cyl_y(P.SH_BOSS_D, P.SH_BOSS_W)
    s += cyl_z(P.SH_SOCKET_D, P.SH_HEAD_LEN, z=-P.SH_HEAD_LEN / 2)
    s += bx(
        14, 6.5, pz + 7, x=px, y=-1.75, z=(pz + 7) / 2
    )  # rod-pin ear, y [-5, 1.5]: the rod (y >= 2) and its end (y >= 3) clear it
    s -= cyl_y(P.KNEE_PIN_D + 0.1, 40)
    s -= cyl_y(P.M5_CLR, 40, x=px, z=pz)
    s -= cyl_z(P.SH_OD, P.SH_HEAD_LEN - 8, z=-8 - (P.SH_HEAD_LEN - 8) / 2)
    return s


def _shank_tube():
    n = P.L_SHANK - 8
    return cyl_z(P.SH_OD, n, z=-8 - n / 2) - cyl_z(P.SH_ID, n + 2, z=-8 - n / 2)


def shank_cf():
    return _shank_head() + _shank_tube()


def shank_cf_head():
    return _shank_head()


def shank_cf_tube():
    return _shank_tube()


def shank_cf_mass_g():
    """Head (6061 solid) + tube at CF_G_PER_MM + bond."""
    return (
        _shank_head().volume * P.DENSITY["al6061"]
        + (P.L_SHANK - 8) * P.CF_G_PER_MM
        + P.BOND_G
    )


def foot_core_cf():
    """PA12-CF cup on the 35 mm ball with a spigot into the tube bore; origin at the ball centre, cup top face z = 25 mates the foot port."""
    s = cyl_z(40, 30, z=10)
    s += cyl_z(P.SH_ID - 0.2, 20, z=35)
    s -= Sphere(P.BALL_D / 2)
    s -= cyl_z(
        8.4, 18, z=15
    )  # blind vent / glue pocket over the ball's pole (keeps the mesh watertight)
    return s


# visual-only split meshes (build.py exports them as <variant>-vis-<key>.stl): key -> builder
VISUALS = {
    "dog-thigh-al": {"al": thigh_al_aluminium, "pa": thigh_al_bands},
    "dog-shank-cf": {"al": shank_cf_head, "cf": shank_cf_tube},
}
