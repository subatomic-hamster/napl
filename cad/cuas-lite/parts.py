"""build123d solids for every printed part of the cuas-lite turret (mm). All dimensions come from params.py.

Groups: "static" (base frame B), "pan" (rotates about B z; modelled in B at pan = 0), "tilt" (modelled in the head frame H:
origin on the tilt axis, x along the barrel, y left, z up). Each builder returns a build123d shape in its group's frame.
Launcher-frame (L) helpers: barrel axis z = 0, flywheel centre x = 0; H = L - (X_A, 0, Z_A).
"""

import math

import params as P
from build123d import (
    Axis,
    Box,
    Cone,
    Cylinder,
    Face,
    Plane,
    Pos,
    RegularPolygon,
    Rot,
    Solid,
    Sphere,
    Vector,
    Wire,
    extrude,
    fillet,
)

S3 = math.sqrt(3)
EPS = 0.1
MIRROR_Y = Plane.XZ


# ---- primitives -----------------------------------------------------------------------------------------------------------------
def bxr(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        x1 - x0, y1 - y0, z1 - z0
    )


def rbox(x0, x1, y0, y1, z0, z1, r):
    return fillet(bxr(x0, x1, y0, y1, z0, z1).edges().filter_by(Axis.Z), r)


def hole(ax, d, u, v, c0, c1):
    """Cylinder of diameter d along axis ax from c0 to c1; (u, v) = the other two coordinates in x, y, z order."""
    h, m = c1 - c0, (c0 + c1) / 2
    cyl = Cylinder(d / 2, h)
    if ax == "z":
        return Pos(u, v, m) * cyl
    if ax == "x":
        return Pos(m, u, v) * Rot(0, 90, 0) * cyl
    return Pos(u, m, v) * Rot(90, 0, 0) * cyl


def hexp(ax, af, u, v, c0, c1):
    """Hex prism (across flats af) along ax."""
    h, m = c1 - c0, (c0 + c1) / 2
    s = Pos(0, 0, -h / 2) * extrude(RegularPolygon(af / S3, 6), amount=h)
    if ax == "z":
        return Pos(u, v, m) * s
    if ax == "x":
        return Pos(m, u, v) * Rot(0, 90, 0) * s
    return Pos(u, m, v) * Rot(90, 0, 0) * s


def slot(ax, d, u, v, c0, c1, along, travel):
    """Stadium slot: hole d swept +-travel along the transverse axis `along` ('u' or 'v')."""
    du, dv = (travel, 0) if along == "u" else (0, travel)
    bu, bv = (2 * travel, d) if along == "u" else (d, 2 * travel)
    if ax == "z":
        b = bxr(u - bu / 2, u + bu / 2, v - bv / 2, v + bv / 2, c0, c1)
    elif ax == "x":
        b = bxr(c0, c1, u - bu / 2, u + bu / 2, v - bv / 2, v + bv / 2)
    else:
        b = bxr(u - bu / 2, u + bu / 2, c0, c1, v - bv / 2, v + bv / 2)
    return b.fuse(
        hole(ax, d, u - du, v - dv, c0, c1), hole(ax, d, u + du, v + dv, c0, c1)
    )


def csk_z(x, y, ztop, d=P.M3_CLR, head=P.M3_CSK_D, hh=1.7):
    """Countersunk through hole, head flush at ztop (cone opens upward)."""
    return hole("z", d, x, y, ztop - 40, ztop + EPS).fuse(
        Pos(x, y, ztop - hh / 2 + EPS / 2) * Cone(d / 2, head / 2, hh + EPS)
    )


def prism(pts, d):
    """Extrude the polygon through 3D points `pts` by the vector d."""
    return Solid.extrude(
        Face(Wire.make_polygon([Vector(*p) for p in pts], close=True)), Vector(*d)
    )


def sector_xz(cx, cz, r0, r1, a0, a1, y0, y1, step=3.0):
    """Annular sector in the xz plane (angle from +x toward +z), extruded along y from y0 to y1."""
    n = max(2, int(abs(a1 - a0) / step))
    ang = [math.radians(a0 + (a1 - a0) * i / n) for i in range(n + 1)]
    pts = [(cx + r1 * math.cos(a), y0, cz + r1 * math.sin(a)) for a in ang]
    pts += [(cx + r0 * math.cos(a), y0, cz + r0 * math.sin(a)) for a in reversed(ang)]
    return prism(pts, (0, y1 - y0, 0))


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def srt(a, b):
    return min(a, b), max(a, b)


def L2H(p):
    return Pos(-P.X_A, 0, -P.Z_A) * p


def seg(p, q, r):
    """Cylinder of radius r along the segment p-q."""
    a, b = Vector(*p), Vector(*q)
    d = b - a
    return Plane(origin=(a + b) / 2, z_dir=d) * Cylinder(r, d.length)


def ballsolid(p, r):
    return Pos(*p) * Sphere(r)


def _halfspace(v, n, big=600.0):
    """Box filling the half space {(x - v) . n >= 0}."""
    return Plane(origin=v, z_dir=n) * (Pos(0, 0, big / 2) * Box(big, big, big))


def mitre_tube(pts, r):
    """Round tube of radius r along the polyline pts with mitred joints (no spheres: equal-radius sphere/cylinder faces are
    tangent and break the booleans)."""
    v = [Vector(*p) for p in pts]
    d = [(v[i + 1] - v[i]).normalized() for i in range(len(v) - 1)]
    out = None
    for i in range(len(d)):
        a, b = v[i], v[i + 1]
        ea = eb = 0.0
        cuts = []
        if i > 0:  # joint with the previous leg at a
            n = (d[i - 1] + d[i]).normalized()
            ea = (
                r * math.tan(math.acos(max(-1.0, min(1.0, d[i - 1].dot(d[i])))) / 2)
                + 0.5
            )
            cuts.append(_halfspace(a, -n))
        if i < len(d) - 1:  # joint with the next leg at b
            n = (d[i] + d[i + 1]).normalized()
            eb = (
                r * math.tan(math.acos(max(-1.0, min(1.0, d[i].dot(d[i + 1])))) / 2)
                + 0.5
            )
            cuts.append(_halfspace(b, n))
        leg = seg(a - d[i] * ea, b + d[i] * eb, r)
        leg = leg.cut(*cuts) if cuts else leg
        out = leg if out is None else out.fuse(leg)
    return out


def gt2_ring(teeth, z0, z1):
    """Cylinder at the tooth OD with GT2-ish grooves cut around it, between z0 and z1 (axis z)."""
    od = P.pulley_od(teeth)
    zm, h = (z0 + z1) / 2, z1 - z0
    ring = Pos(0, 0, zm) * Cylinder(od / 2, h)
    gs = [
        Pos(
            od / 2 * math.cos(2 * math.pi * i / teeth),
            od / 2 * math.sin(2 * math.pi * i / teeth),
            zm,
        )
        * Cylinder(P.TOOTH_R, h + 0.02)
        for i in range(teeth)
    ]
    return ring.cut(*gs), od / 2 + P.FLANGE_OVER


# ---- shared layout numbers ----------------------------------------------------------------------------------------------------
YO = P.CAGE_W / 2  # cage half width (trunnion boss starts here)
Y_C0 = P.CHEEK_Y0  # cheek inner face (58)
Y_C1 = P.CHEEK_Y0 + P.CHEEK_T  # cheek outer face
YOKE_SCREW_Y = (
    Y_C0 - 3.5,
    Y_C1 + 3.5,
)  # vertical screws through the cheek foot into the rotor plate
YOKE_SCREW_X = (-30.0, -12.0, 12.0, 30.0)
Y_HUB0, HUB_W = Y_C1 + 0.5, 8.0
Y_P0 = Y_HUB0 + HUB_W  # tilt pulley web inboard face
Y_BELT = Y_P0 + 5.5 + 4.0  # belt centre plane of the tilt drive
Y_MP0, MP_T = Y_BELT + 7.0, 4.0  # tilt motor plate inboard face / thickness
X_S, Y_S = -65.5, -18.0  # servo spline (lever pivot) in L
Z_SERVO_PLATE = P.CHASSIS_Z0 - 12.0  # top of the servo bracket plate (tabs rest here)
ZEAR = -64.0  # tray ear / leg screw height
ELBOW_BOLTS = tuple(
    (sx, dz) for sx in (-22.0, 22.0) for dz in (-22.0, 22.0)
)  # feed elbow -> right cheek (x, z - H_T)
CHUTE_C = (P.X_A, 0.0, P.Z_A)  # ball turns 90 degrees here (on the tilt axis), L frame
CHUTE_F = (
    P.X_F,
    0.0,
    0.0,
)  # ball rests here in the bore in front of the piston, L frame
BEAM_BOLTS = tuple(
    (sx, sy) for sx in (-26.0, 26.0) for sy in (-42.0, 42.0)
)  # trunnion beam -> cage top (L x, y)


# ======================================================================================================================================
# static group (frame B)
# ======================================================================================================================================
def base_stator_plate():
    T = P.STATOR_T
    p = rbox(-100, 100, -100, 100, -T, 0, 10)
    posts, tools = [], [hole("z", 40, 0, 0, -T - EPS, EPS)]
    for k in range(4):
        x, y = polar(P.LS_BOLT_R, 45 + 90 * k)
        posts.append(Pos(x, y, P.POST_H / 2) * Cylinder(8, P.POST_H))
        tools.append(
            hole("z", P.M4_INSERT_D, x, y, P.POST_H - P.M4_INSERT_DEPTH, P.POST_H + EPS)
        )
    p = p.fuse(*posts)
    c = P.C_PAN
    tools.append(
        bxr(c - 14, c + 14, -10, 10, -T - EPS, EPS)
    )  # shaft / pulley-hub slot of the pan motor
    for sx in (-14, 14):
        for sy in (-25, 25):  # motor-mount tension screws: holes + nut traps (top)
            tools += [
                hole("z", P.M3_CLR, c + sx, sy, -T - EPS, EPS),
                hexp("z", P.M3_NUT_AF, c + sx, sy, -P.M3_NUT_H, EPS),
            ]
    for sx in (-88, 88):
        for sy in (-75, 75):
            tools.append(csk_z(sx, sy, 0))  # leg screws
    for sy in (-8, 8):  # pan hall bracket: screw from top, nut under
        tools += [
            hole("z", P.M3_CLR, -80, sy, -T - EPS, EPS),
            hexp("z", P.M3_NUT_AF, -80, sy, -T - EPS, -T + P.M3_NUT_H),
        ]
    for sy in (-40, 40):  # front panel ears
        tools.append(hole("z", P.M3_INSERT_D, 96, sy, -T - EPS, -T + P.M3_INSERT_DEPTH))
    return p.cut(*tools)


def base_leg():
    """One of four. Modelled at (+88, +75): tray-ear insert on the -y face. Others: translate, +-180 deg about z."""
    z0, z1 = -88.0, -P.STATOR_T
    p = bxr(78, 98, 65, 85, z0, z1)
    tools = [
        hole("z", P.M3_INSERT_D, 88, 75, z1 - P.M3_INSERT_DEPTH, z1 + EPS),
        hole("y", P.M3_INSERT_D, 88, ZEAR, 65 - EPS, 65 + P.M3_INSERT_DEPTH),
    ]
    return p.cut(*tools)


def electronics_tray():
    """Open box under the stator plate: Jetson standoffs, Pico posts, zip-tie pads, power-inlet notch, ears to the legs."""
    z0, zr = -85.0, -60.0
    p = rbox(-90, 90, -55, 55, z0, zr, 4).cut(bxr(-87, 87, -52, 52, z0 + 3, zr + EPS))
    add, tools = [], []
    cx, (jx, jy) = -34.0, P.JETSON_HOLES
    zf = z0 + 3
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = cx + sx * jx / 2, sy * jy / 2
            add.append(Pos(x, y, zf + P.STANDOFF_H / 2) * Cylinder(4.5, P.STANDOFF_H))
            tools.append(
                hole(
                    "z",
                    P.M3_INSERT_D,
                    x,
                    y,
                    zf + P.STANDOFF_H - P.M3_INSERT_DEPTH,
                    zf + P.STANDOFF_H + EPS,
                )
            )
    _, _, ps, pL = P.PICO  # Pico holes 11.4 x 47, board long axis along x
    px, py = 52.0, 33.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = px + sy * pL / 2, py + sx * ps / 2
            add.append(Pos(x, y, zf + 2) * Cylinder(3.0, 4))
            tools.append(hole("z", 2.2, x, y, zf, zf + 4 + EPS))
    for x, y in (
        (30, 5),
        (55, 5),
        (30, -30),
        (55, -30),
        (78, -12),
    ):  # pads: zip-tie slot pairs (drivers, bucks, MOSFET)
        for dy in (-12, 12):
            tools.append(
                bxr(x - 8, x + 8, y + dy - 1.3, y + dy + 1.3, z0 - EPS, zf + EPS)
            )
    tools.append(
        bxr(-90 - EPS, -86, -9, 9, z0 + 8, z0 + 22)
    )  # power inlet (XT60) notch, rear wall
    for sy in (-1, 1):
        for i in range(5):
            tools.append(
                bxr(
                    -75 + i * 12 - 3,
                    -75 + i * 12 + 3,
                    sy * 45 - 1.5,
                    sy * 45 + 1.5,
                    z0 - EPS,
                    zf + EPS,
                )
            )
    p = p.fuse(*add)
    for sx in (-88, 88):
        for ya, yb in (
            (52.0, 64.5),
            (-64.5, -52.0),
        ):  # ears reach the legs' inner faces (y = +-65)
            p = p.fuse(bxr(sx - 10, sx + 10, ya, yb, zr - 8, zr))
            tools.append(
                hole("y", P.M3_CLR, sx, ZEAR, ya, yb + EPS if yb > 0 else yb + EPS)
            )
    return p.cut(*tools)


def pan_motor_mount():
    c = P.C_PAN
    z1, z0 = -P.STATOR_T, -P.STATOR_T - P.MOUNT_T
    p = rbox(c - 28, c + 28, -28, 28, z0, z1, 4)
    tools = [hole("z", P.N17_PILOT + 0.4, c, 0, z0 - EPS, z1 + EPS)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            tools.append(csk_z(c + sx * P.N17_BOLT / 2, sy * P.N17_BOLT / 2, z1))
            tools.append(
                slot(
                    "z",
                    P.M3_CLR,
                    c + sx * 14,
                    sy * 25,
                    z0 - EPS,
                    z1 + EPS,
                    "u",
                    P.SLOT_TRAVEL,
                )
            )
    return p.cut(*tools)


def pan_hall_mount():
    """Holds the pan hall sensor 3 mm under the rotor-plate magnet; bolted on the stator plate at (-80, 0)."""
    zs = P.Z_ROTOR0 - 3.0
    p = bxr(-86, -74, -12, 12, 0, 4).fuse(bxr(-85, -75, -5, 5, 0, zs))
    tools = [hole("z", P.M3_CLR, -80, sy, -EPS, 4 + EPS) for sy in (-8, 8)]
    tools += [
        bxr(-82.3, -77.7, -1.7, 1.7, zs - 2.6, zs + EPS),
        bxr(-80.8, -79.2, -1.7, 5.1, zs - 2.6, zs - 0.8),
    ]
    return p.cut(*tools)


def front_panel():
    """E-stop (22 mm), key switch (19 mm), 2 LEDs, buzzer; two ears bolt under the stator plate front edge."""
    x0, x1 = 104.0, 108.0
    zt, zb = -P.STATOR_T, -78.0
    p = bxr(x0, x1, -60, 60, zb, zt)
    for ya, yb in ((30, 50), (-50, -30)):
        p = p.fuse(bxr(92, x1, ya, yb, zt - 4, zt))
    tools = [
        hole("x", 22.3, -28, -43, x0 - EPS, x1 + EPS),
        hole("x", 19.2, 28, -43, x0 - EPS, x1 + EPS),
        hole("x", 5.2, -6, -68, x0 - EPS, x1 + EPS),
        hole("x", 5.2, 6, -68, x0 - EPS, x1 + EPS),
        hole("x", 12.2, 40, -68, x0 - EPS, x1 + EPS),
    ]
    tools += [hole("z", P.M3_CLR, 96, sy, zt - 4 - EPS, zt + EPS) for sy in (-40, 40)]
    return p.cut(*tools)


# ======================================================================================================================================
# pan group (frame B)
# ======================================================================================================================================
def pan_rotor_plate():
    z0, z1 = P.Z_ROTOR0, P.Z_ROTOR1
    p = rbox(-90, 90, -90, 90, z0, z1, 12)
    tools = [
        hole("z", P.CABLE_HOLE_D, 0, 0, z0 - EPS, z1 + EPS),
        hole("z", 6.2, -80, 0, z0 - EPS, z0 + 3.2),
    ]
    for k in range(4):
        x, y = polar(P.LS_BOLT_R, 90 * k)  # ring-2 bolts, counterbored
        tools += [
            hole("z", P.M4_CLR, x, y, z0 - EPS, z1 + EPS),
            hole("z", 8.4, x, y, z1 - 4, z1 + EPS),
        ]
        x, y = polar(P.LS_BOLT_R, 45 + 90 * k)  # access to the ring-1 (stator) bolts
        tools.append(hole("z", 9.0, x, y, z0 - EPS, z1 + EPS))
    for k in range(6):
        x, y = polar(23.0, 30 + 60 * k)  # pulley screws, flat heads
        tools.append(csk_z(x, y, z1))
    for sy in (-1, 1):
        for sx in YOKE_SCREW_X:
            for yy in YOKE_SCREW_Y:
                tools.append(hole("z", P.M3_INSERT_D, sx, sy * yy, z1 - 6.0, z1 + EPS))
    return p.cut(*tools)


def pan_pulley_100t():
    """Printed GT2 100T; the neck's top face bolts to the rotor-plate underside, the teeth hang inside the bearing aperture."""
    zb = P.Z_BELT
    zt0, zt1 = zb - 4.0, zb + 4.0
    ring, rf = gt2_ring(P.T_PAN, zt0, zt1)
    rneck = 28.0
    h_cone = rf - rneck
    zc0 = zt1 + P.FLANGE_T
    body = ring.fuse(
        Pos(0, 0, zt0 - P.FLANGE_T / 2) * Cylinder(rf, P.FLANGE_T),
        Pos(0, 0, zt1 + P.FLANGE_T / 2) * Cylinder(rf, P.FLANGE_T),
        Pos(0, 0, zc0 + h_cone / 2) * Cone(rf, rneck, h_cone),
        Pos(0, 0, (zc0 + h_cone + P.Z_ROTOR0) / 2)
        * Cylinder(rneck, P.Z_ROTOR0 - zc0 - h_cone),
    )
    tools = [hole("z", 32.0, 0, 0, zt0 - 3, P.Z_ROTOR0 + EPS)]
    for k in range(6):
        x, y = polar(23.0, 30 + 60 * k)
        tools.append(
            hole(
                "z",
                P.M3_INSERT_D,
                x,
                y,
                P.Z_ROTOR0 - P.M3_INSERT_DEPTH,
                P.Z_ROTOR0 + EPS,
            )
        )
    return body.cut(*tools)


def cable_collar():
    """Press-in grommet for the rotor-plate cable hole (top flange clears the pulley screw heads)."""
    z1 = P.Z_ROTOR1
    p = (Pos(0, 0, z1 + 1.0) * Cylinder(19, 2)).fuse(
        Pos(0, 0, z1 - 3) * Cylinder(P.CABLE_HOLE_D / 2 - 0.2, 8)
    )
    tools = [hole("z", 20, 0, 0, z1 - 8, z1 + 3)]
    return p.cut(*tools)


def _yoke(side):
    """side +1 left (tilt drive + hall pocket), -1 right (idler + tilt stops); built at +y and mirrored for the right."""
    y0, y1 = Y_C0, Y_C1
    zb, zt, ha = P.Z_ROTOR1, P.H_T + 32, P.H_T
    p = prism(
        [(-42, y0, zb), (42, y0, zb), (38, y0, zt), (-38, y0, zt)], (0, P.CHEEK_T, 0)
    ).fuse(bxr(-42, 42, y0 - 7, y1 + 7, zb, zb + 8))
    if side > 0:
        tools = [
            hole("y", 14.0, 0, ha, y0 - EPS, y1 + EPS),
            hole("y", P.B608_OD + 0.2, 0, ha, y1 - P.B608_W - 0.2, y1 + EPS),
        ]
    else:  # hollow trunnion (OD 35) passes through; 6807 bearing seat on the outer face; 4 inserts for the feed elbow
        tools = [
            hole("y", P.TUBE_OD + 5.0, 0, ha, y0 - EPS, y1 + EPS),
            hole("y", P.B6807_OD + 0.2, 0, ha, y1 - P.B6807_W - 0.2, y1 + EPS),
        ] + [
            hole("y", P.M3_INSERT_D, sx, ha + dz, y1 - P.M3_INSERT_DEPTH, y1 + EPS)
            for sx, dz in ELBOW_BOLTS
        ]
    for sx in YOKE_SCREW_X:
        for yy in YOKE_SCREW_Y:
            tools += [
                hole("z", P.M3_CLR, sx, yy, zb - EPS, zb + 8 + EPS),
                hole("z", P.M3_HEAD_D, sx, yy, zb + 3, zb + 8 + EPS),
            ]
    extra = []
    if side > 0:
        for sx in (-33, 33):
            for z in (ha, ha - P.C_TILT):
                tools.append(
                    hole("y", P.M3_INSERT_D, sx, z, y1 - P.M3_INSERT_DEPTH, y1 + EPS)
                )
        tools += [
            bxr(-2.3, 2.3, y1 - 3.0, y1 + EPS, ha + 13 - 1.7, ha + 13 + 1.7),
            bxr(-0.8, 0.8, y1 - 3.0, y1 + EPS, ha + 13, ha + 22),
        ]
    else:
        half = math.degrees(math.asin(4.0 / 22.0))
        lo, hi = P.TILT_RANGE[0] - 3.0 - half, P.TILT_RANGE[1] + 3.0 + half
        extra = [
            sector_xz(0, ha, 22, 32, lo - 25, lo, y0 - 5.0, y0),
            sector_xz(0, ha, 22, 32, hi, hi + 25, y0 - 5.0, y0),
        ]
    p = p.cut(*tools)
    if extra:
        p = p.fuse(*extra)
    return p if side > 0 else p.mirror(MIRROR_Y)


def yoke_left():
    return _yoke(1)


def yoke_right():
    return _yoke(-1)


def tilt_motor_mount():
    """Plate on 4 posts off the left cheek; NEMA 17 on its outboard face (body outboard), tensioned vertically +-4 mm."""
    ha, zc = P.H_T, P.H_T - P.C_TILT
    ya, yb = Y_MP0, Y_MP0 + MP_T
    p = bxr(-38, 38, ya, yb, zc - 32, ha + 12)
    p = p.fuse(
        *[
            Pos(sx, (Y_C1 + yb) / 2, z) * Rot(90, 0, 0) * Cylinder(5, yb - Y_C1)
            for sx in (-33, 33)
            for z in (ha, zc)
        ]
    )
    tools = [
        slot("y", P.N17_PILOT + 0.4, 0, zc, ya - EPS, yb + EPS, "v", P.SLOT_TRAVEL)
    ]
    for sx in (-1, 1):
        for sz in (-1, 1):
            u, v = sx * P.N17_BOLT / 2, zc + sz * P.N17_BOLT / 2
            tools += [
                slot("y", P.M3_CLR, u, v, ya - EPS, yb + EPS, "v", P.SLOT_TRAVEL),
                slot("y", P.M3_HEAD_D, u, v, ya - EPS, ya + 2.0, "v", P.SLOT_TRAVEL),
            ]
    for sx in (-33, 33):
        for z in (ha, zc):
            tools += [
                hole("y", P.M3_CLR, sx, z, Y_C1 - EPS, yb + EPS),
                hole("y", P.M3_HEAD_D, sx, z, yb - 3.2, yb + EPS),
            ]
    return p.cut(*tools)


# ======================================================================================================================================
# tilt group (frame H, built in L)
# ======================================================================================================================================
def bore_x(x0, x1, R):
    """Ball bore along x: circle R plus a flat-topped 45 degree roof (prints without supports while lying on x)."""
    t = R / math.sqrt(2)
    zf = R * 1.25
    xf = R * math.sqrt(2) - zf
    tear = prism(
        [(x0, -t, t), (x0, -xf, zf), (x0, xf, zf), (x0, t, t), (x0, 0, 0)],
        (x1 - x0, 0, 0),
    )
    return hole("x", 2 * R, 0, 0, x0, x1).fuse(tear)


PIN_X_HOME = P.X_F - 13.5 - P.PISTON_LEN / 2  # piston pin x with the piston retracted


def launcher_chassis():
    """Breech + feed neck port + flywheel cage + trunnion bosses, one print (spec lists chassis and cage; merged: no bridged joint)."""
    R = P.BARREL_ID / 2
    xc = P.CAGE_X / 2
    p = bxr(
        P.CHASSIS_X0,
        -xc + 1,
        -P.CHASSIS_W / 2,
        P.CHASSIS_W / 2,
        P.CHASSIS_Z0,
        P.CHASSIS_Z1,
    ).fuse(bxr(-xc, xc, -YO, YO, P.CAGE_Z0, P.CAGE_Z1))
    rs = (
        P.JOUR_ID / 2 + P.CHUTE_WALL
    )  # chute sleeve behind the cage rear face is part of the chassis (the beam's ends at x = -xc)
    p = p.fuse(
        seg(CHUTE_C, CHUTE_F, rs)
        & bxr(P.CHASSIS_X0, -xc, -rs - 1, rs + 1, P.CHASSIS_Z0, 90)
    )
    tools = [
        bore_x(P.CHASSIS_X0 + 4, xc + EPS, R),
        seg(
            CHUTE_C, CHUTE_F, P.CHUTE_R
        ),  # ball chute from the trunnion beam down to the breech
    ]
    for sx, sy in BEAM_BOLTS:  # trunnion beam inserts (cage top)
        tools.append(
            hole(
                "z",
                P.M3_INSERT_D,
                sx,
                sy,
                P.CAGE_Z1 - P.M3_INSERT_DEPTH,
                P.CAGE_Z1 + EPS,
            )
        )
    for sx in (-1, 1):
        for sy in (-1, 1):  # barrel inserts (front)
            tools.append(
                hole(
                    "x",
                    P.M3_INSERT_D,
                    sy * 12.7,
                    sx * 12.7,
                    xc - P.M3_INSERT_DEPTH,
                    xc + EPS,
                )
            )
    tools.append(
        bxr(
            PIN_X_HOME - 2.1,
            PIN_X_HOME + P.PISTON_STROKE + 2.1,
            -2.1,
            2.1,
            P.CHASSIS_Z0 - EPS,
            P.CHASSIS_Z0 + 9,
        )
    )  # pusher pin slot
    for sx in (-30, 30):  # servo bracket inserts (bottom)
        tools.append(
            hole(
                "z",
                P.M3_INSERT_D,
                X_S + sx,
                Y_S,
                P.CHASSIS_Z0 - EPS,
                P.CHASSIS_Z0 + P.M3_INSERT_DEPTH,
            )
        )
    yw = P.FW_GAP / 2 + P.WHEEL_D / 2  # wheel centre |y|
    zb = P.WHEEL_W / 2 + 1.0  # wheel cavity half height
    rc = P.WHEEL_D / 2 + 1.5
    dm = P.M180_D + 0.4
    for s in (-1, 1):
        yc = s * yw
        tools.append(hole("z", 2 * rc, 0, yc, -zb, zb))  # wheel cavity
        ya, yb = srt(yc, s * (YO + EPS))
        tools.append(
            bxr(-rc, rc, ya, yb, -zb, zb)
        )  # ... open to the side (closed by wheel_cover)
        ya, yb = srt(s * (YO - 3.0), s * (YO + EPS))
        tools.append(bxr(-27, 27, ya, yb, -8.5, 8.5))  # cover recess
        ya, yb = srt(s * (YO - 3.0), s * (YO - 9.0))
        tools += [
            hole("y", P.M3_INSERT_D, sx, sz, ya, yb)
            for sx in (-23.5, 23.5)
            for sz in (-5.5, 5.5)
        ]
        tools.append(hole("z", dm, 0, yc, zb - 1.0, P.CAGE_Z1 + EPS))  # motor pocket
        for z in (11.0, 21.0):  # radial clamp screws from the outer face
            ya, yb = srt(
                yc + s * (dm / 2 - 2.0), s * (YO + EPS)
            )  # starts inside the pocket (no tangent faces)
            tools.append(hole("y", P.M3_CLR, 0, z, ya, yb))
            ya, yb = srt(s * (YO - 6.0), s * (YO + EPS))
            tools.append(hole("y", P.M3_INSERT_D, 0, z, ya, yb))
    return L2H(p.cut(*tools))


def trunnion_beam():
    """Tilt axis member on top of the cage: hollow right half (ball passes along the axis, 90 degree turn at C into the chute),
    solid left half with the rod seat; carries the chute sleeve above the cage and the right tilt-stop lug. Bolted to the cage."""
    xc = P.CAGE_X / 2
    rj = P.JOUR_ID / 2
    ro = rj + P.CHUTE_WALL
    yr, yl = -(Y_C1 - 0.5), YO + P.BOSS_L
    p = (
        Pos(P.X_A, (yr + yl) / 2, P.Z_A)
        * Rot(90, 0, 0)
        * Cylinder(P.TUBE_OD / 2, yl - yr)
    )
    p = p.fuse(
        bxr(
            -30,
            max(30.0, P.X_A + 17.0),
            -49,
            49,
            P.CAGE_Z1,
            P.CAGE_Z1 + P.BEAM_FOOT_T + 1.0,
        ),  # foot
        seg(
            CHUTE_C, CHUTE_F, ro
        ),  # chute sleeve (the part inside the cage/chassis is removed below)
        bxr(
            P.X_A + 6, P.X_A + 27, -YO - 4.0, -YO - 1.0, P.Z_A - 4, P.Z_A + 4
        ),  # stop lug
    )
    blocks = bxr(
        P.CHASSIS_X0,
        -xc + 1,
        -P.CHASSIS_W / 2,
        P.CHASSIS_W / 2,
        P.CHASSIS_Z0,
        P.CHASSIS_Z1,
    ).fuse(
        bxr(-xc, xc, -YO, YO, P.CAGE_Z0, P.CAGE_Z1),
        bxr(
            P.CHASSIS_X0 - 10, -xc, -60, 60, -40, 120
        ),  # chassis owns everything behind the cage rear face
    )
    p = p.cut(blocks)
    tools = [
        hole(
            "y", P.JOUR_ID, P.X_A, P.Z_A, yr - EPS, rj
        ),  # axial ball bore, runs on past C so the ball can sit at the turn
        seg(CHUTE_C, CHUTE_F, P.CHUTE_R),
        hole("y", 8.2, P.X_A, P.Z_A, YO - 11.0, yl + EPS),  # left rod seat
        hole(
            "x", P.M3_CLR, YO + P.BOSS_L / 2, P.Z_A, P.X_A, P.X_A + 20
        ),  # rod set screw
        bxr(  # nut slot, open upward
            P.X_A + 5.0,
            P.X_A + 7.6,
            YO + P.BOSS_L / 2 - 2.9,
            YO + P.BOSS_L / 2 + 2.9,
            P.Z_A - 3.2,
            P.Z_A + P.TUBE_OD / 2 + 1.0,
        ),
    ]
    for sx, sy in BEAM_BOLTS:
        tools += [
            hole(
                "z", P.M3_CLR, sx, sy, P.CAGE_Z1 - EPS, P.CAGE_Z1 + P.BEAM_FOOT_T + 2.0
            ),
            hole(
                "z",
                P.M3_HEAD_D,
                sx,
                sy,
                P.CAGE_Z1 + 1.8,
                P.CAGE_Z1 + P.BEAM_FOOT_T + 2.0,
            ),
        ]
    return L2H(p.cut(*tools))


def barrel_half(side):
    """Clamshell half. side +1 (y >= 0): through holes. side -1: same plus M3 nut pockets on the split face."""
    R, Ri = P.BARREL_R, P.BARREL_ID / 2
    x0 = P.CAGE_X / 2
    x1 = x0 + P.BARREL_LEN
    keep = bxr(x0 - 1, x1 + 1, 0, 40, -40, 40)
    p = (Pos((x0 + x1) / 2, 0, 0) * Rot(0, 90, 0) * Cylinder(R, x1 - x0)) & keep
    p = p.fuse(
        bxr(x0 + 4, x1, 0, 8, R - 5, R + 9),
        bxr(x0 + 4, x1, 0, 8, -R - 9, -R + 5),
        (Pos(x0 + 2, 0, 0) * Rot(0, 90, 0) * Cylinder(25, 4)) & keep,
    )
    tools = [bore_x(x0 - EPS, x1 + EPS, Ri)]
    for z in (R + 4, -R - 4):
        for x in (48.0, 76.0, 98.0):
            tools.append(hole("y", P.M3_CLR, x, z, -EPS, 8 + EPS))
            if side < 0:
                tools.append(hexp("y", P.M3_NUT_AF, x, z, -EPS, P.M3_NUT_H))
    for sz in (-1, 1):
        tools.append(hole("x", P.M3_CLR, 12.7, sz * 12.7, x0 - EPS, x0 + 4 + EPS))
    for x in (110.0, 124.0):
        tools.append(hole("z", P.M3_INSERT_D, x, 4.0, R + 9 - 6, R + 9 + EPS))
    p = p.cut(*tools)
    return L2H(p if side > 0 else p.mirror(MIRROR_Y))


def barrel_half_a():
    return barrel_half(1)


def barrel_half_b():
    return barrel_half(-1)


HOPPER = dict(R_top=42.0, wall=2.4, Hn=22.0, Hc=90.0)


def _hopper_dims():
    h = HOPPER
    rn = (P.BARREL_ID + 2.4) / 2
    v_ball = math.pi / 6 * P.BALL_D**3
    v_cone = math.pi * h["Hc"] / 3 * (h["R_top"] ** 2 + h["R_top"] * rn + rn**2)
    Hs = max(
        10.0,
        (P.HOPPER_BALLS * v_ball / P.PACKING - v_cone) / (math.pi * h["R_top"] ** 2),
    )
    return rn, Hs, v_ball, v_cone


def hopper_capacity():
    rn, Hs, v_ball, v_cone = _hopper_dims()
    return (v_cone + math.pi * HOPPER["R_top"] ** 2 * Hs) * P.PACKING / v_ball


def hopper():
    """Funnel (cone 18 deg off vertical, prints neck-down without supports) + 22 mm neck + flange; height set by ball count."""
    h = HOPPER
    rn, Hs, _, _ = _hopper_dims()
    Hn, Hc, Rt, w = h["Hn"], h["Hc"], h["R_top"], h["wall"]
    outer = Pos(0, 0, Hn / 2) * Cylinder(rn + w, Hn)
    outer = outer.fuse(
        Pos(0, 0, 2) * Cylinder(26, 4),
        Pos(0, 0, Hn + Hc / 2) * Cone(rn + w, Rt + w, Hc),
        Pos(0, 0, Hn + Hc + Hs / 2) * Cylinder(Rt + w, Hs),
    )
    inner = hole("z", 2 * rn, 0, 0, -EPS, Hn + EPS).fuse(
        Pos(0, 0, Hn + Hc / 2) * Cone(rn, Rt, Hc),
        hole("z", 2 * Rt, 0, 0, Hn + Hc - EPS, Hn + Hc + Hs + EPS),
    )
    tools = [inner] + [
        hole("z", P.M3_CLR, *polar(18, 45 + 90 * k), -EPS, 4 + EPS) for k in range(4)
    ]
    return Pos(0, -P.ELBOW_YH, elbow_dims()[2]) * outer.cut(*tools)


def elbow_dims():
    """Pan-side feed path key points (B frame): slope end s0 on the tilt axis side, slope start s1 under the hopper, flange plane zf."""
    ha = P.H_T
    s0 = (0.0, -P.ELBOW_YS, ha)
    s1 = (
        0.0,
        -P.ELBOW_YH,
        ha + (P.ELBOW_YH - P.ELBOW_YS) * math.tan(math.radians(P.ELBOW_SLOPE)),
    )
    return s0, s1, s1[2] + 20.0


def feed_elbow():
    """Pan-group feed elbow (B frame): bolts to the right cheek outer face (4 x M3), holds the hopper flange, sends balls down a
    sloping duct and along the tilt axis through a spigot that sits inside the rotating hollow trunnion."""
    ha = P.H_T
    s0, s1, zf = elbow_dims()
    ri, ro = P.DUCT_ID / 2, P.DUCT_ID / 2 + 2.0
    yp = -(Y_C1 + 4.0)  # plate outer face
    top = (0.0, s1[1], zf)
    pt = (0.0, yp, ha)
    tip = (0.0, -P.SPIG_TIP_Y, ha)
    p = mitre_tube([top, s1, s0, pt], ro).fuse(
        Pos(0, s1[1], zf - 4.0) * Cylinder(26.0, 8.0),  # hopper flange pad
        bxr(-36, 36, yp, -Y_C1, ha - 34, ha + 30),  # plate on the cheek
        seg((0.0, yp + 2.0, ha), tip, P.SPIG_OD / 2),  # spigot
    )
    tools = [
        mitre_tube([(0.0, s1[1], zf + EPS), s1, s0, (0.0, yp + 2.0, ha)], ri),
        seg((0.0, yp + 1.0, ha), (0.0, -P.SPIG_TIP_Y + EPS, ha), P.SPIG_ID / 2),
    ]
    for k in range(4):
        x, y = polar(18, 45 + 90 * k)
        tools.append(hole("z", P.M3_INSERT_D, x, s1[1] + y, zf - 6.0, zf + EPS))
    for sx, dz in ELBOW_BOLTS:
        tools.append(hole("y", P.M3_CLR, sx, ha + dz, yp - EPS, -Y_C1 + EPS))
    return p.cut(*tools)


def hopper_lid():
    Rt, w = HOPPER["R_top"], HOPPER["wall"]
    rn, Hs, _, _ = _hopper_dims()
    top = elbow_dims()[2] + HOPPER["Hn"] + HOPPER["Hc"] + Hs  # hopper rim (B)
    p = Pos(0, 0, 1.5) * Cylinder(Rt + w, 3)
    ring = (Pos(0, 0, -2.0) * Cylinder(Rt - 0.5, 4)).cut(
        Pos(0, 0, -2.0) * Cylinder(Rt - 2.9, 4.2)
    )
    tab = bxr(
        Rt, Rt + w + 12, -8, 8, 0, 3
    )  # finger tab, in plane (prints flat, plug ring up)
    return Pos(0, -P.ELBOW_YH, top) * p.fuse(ring, tab)


def piston():
    """Pusher piston at its retracted position (face at x_f - 13.5): ball dimple on the face, M3 pin insert in the bottom."""
    D, L = P.BALL_D, P.PISTON_LEN
    xf = P.X_F - 13.5
    body = Pos(xf - L / 2, 0, 0) * Rot(0, 90, 0) * Cylinder(D / 2, L)
    tools = [
        Pos(xf + D / 2 - 3.0, 0, 0) * Sphere(D / 2),
        hole(
            "z", P.M3_INSERT_D, xf - L / 2, 0, -D / 2 - EPS, -D / 2 + P.M3_INSERT_DEPTH
        ),
    ]
    return L2H(body.cut(*tools))


def servo_bracket():
    """Holds the SG90 under the chassis (spline up, tabs resting on the plate top)."""
    zp = Z_SERVO_PLATE
    p = bxr(X_S - 36, X_S + 36, Y_S - 24, Y_S + 28, zp - 3, zp)
    p = p.fuse(
        *[
            Pos(X_S + sx, Y_S, (zp + P.CHASSIS_Z0) / 2) * Cylinder(5, P.CHASSIS_Z0 - zp)
            for sx in (-30, 30)
        ]
    )
    bx, by = P.SERVO["body"][0], P.SERVO["body"][1]
    yc = Y_S - (by / 2 - P.SERVO["spline_from_end"])
    tools = [
        bxr(
            X_S - (bx + 0.4) / 2,
            X_S + (bx + 0.4) / 2,
            yc - (by + 0.4) / 2,
            yc + (by + 0.4) / 2,
            zp - 3 - EPS,
            zp + EPS,
        )
    ]
    tools += [
        hole(
            "z",
            P.M2_CLR,
            X_S,
            yc + dy * P.SERVO["tab_hole_dy"] / 2,
            zp - 3 - EPS,
            zp + EPS,
        )
        for dy in (-1, 1)
    ]
    for sx in (-30, 30):
        tools += [
            hole("z", P.M3_CLR, X_S + sx, Y_S, zp - 3 - EPS, P.CHASSIS_Z0 + EPS),
            hole("z", P.M3_HEAD_D, X_S + sx, Y_S, zp - 3 - EPS, zp - 3 + 3.2),
        ]
    return L2H(p.cut(*tools))


ARM_T = 3.0
Z_ARM = (
    Z_SERVO_PLATE + 5.4 + ARM_T / 2
)  # arm mid-plane: on the spline, 1 mm under its tip


def pusher_arm():
    """Scotch-yoke lever drawn at the servo (X_S, Y_S) pointing +y (servo at 0 deg); slot takes the M3 pin on the piston."""
    d = abs(Y_S)
    r_a, r_b = d, math.hypot(d, P.PISTON_STROKE / 2)
    arm = (Pos(X_S, Y_S, Z_ARM) * Cylinder(7.5, ARM_T)).fuse(
        Pos(X_S, Y_S + (r_b + 5) / 2, Z_ARM) * Box(10, r_b + 5, ARM_T)
    )
    ym = Y_S + (r_a + r_b) / 2
    slot_cut = bxr(
        X_S - P.M3_CLR / 2,
        X_S + P.M3_CLR / 2,
        Y_S + r_a,
        Y_S + r_b,
        Z_ARM - 3,
        Z_ARM + 3,
    ).fuse(
        hole("z", P.M3_CLR, X_S, Y_S + r_a, Z_ARM - 3, Z_ARM + 3),
        hole("z", P.M3_CLR, X_S, Y_S + r_b, Z_ARM - 3, Z_ARM + 3),
    )
    tools = [slot_cut, hole("z", 4.8, X_S, Y_S, Z_ARM - 3, Z_ARM + 3)]
    for k in range(4):
        x, y = polar(5.5, 45 + 90 * k)
        tools.append(hole("z", P.M2_CLR, X_S + x, Y_S + y, Z_ARM - 3, Z_ARM + 3))
    return L2H(arm.cut(*tools))


MUZZLE_X = P.CAGE_X / 2 + P.BARREL_LEN
CAM_Z0 = P.BARREL_R + 9  # top of the barrel ears = camera-plate underside (L)


def camera_mount():
    xm, z0 = MUZZLE_X, CAM_Z0
    bd, bw, bh = P.BRIO
    p = bxr(104.0, xm + 3, -55, 55, z0, z0 + P.CAM_PLATE_T)
    p = p.fuse(
        bxr(xm - bd - 4.0, xm - bd - 0.5, -55, 55, z0, z0 + 12),
        bxr(xm - bd - 0.5, xm, 51.6, 55, z0, z0 + 15),
        bxr(xm - bd - 0.5, xm, -55, -51.6, z0, z0 + 15),
        bxr(xm, xm + 3, -55, 55, z0, z0 + 5),
    )
    tools = [
        hole("z", P.UNC_QTR_CLR, xm - bd / 2, 0, z0 - EPS, z0 + P.CAM_PLATE_T + EPS),
        bxr(xm - bd - 5, xm - bd + 1, -8, 8, z0 + 4, z0 + 13),
    ]
    tools += [
        csk_z(x, sy, z0 + P.CAM_PLATE_T) for x in (110.0, 124.0) for sy in (-4.0, 4.0)
    ]
    return L2H(p.cut(*tools))


def camera_shim():
    """+-3 degree boresight wedge under the Brio (rotate to choose pitch or yaw correction); prints flat."""
    bd, bw = P.BRIO[0], P.BRIO[1]
    t = bd * math.tan(math.radians(3.0))
    return prism(
        [(0, 0, 0), (bd, 0, 0), (bd, 0, t + 0.8), (0, 0, 0.8)], (0, bw - 10, 0)
    )


def wheel_cover_pos_y():
    """Closes the +y wheel window of the cage. Drawn in place (M3x8 into the cage inserts)."""
    y0, y1 = YO - 2.9, YO - 0.1
    p = bxr(-26.8, 26.8, y0, y1, -8.3, 8.3)
    return L2H(
        p.cut(
            *[
                hole("y", P.M3_CLR, sx, sz, y0 - EPS, y1 + EPS)
                for sx in (-23.5, 23.5)
                for sz in (-5.5, 5.5)
            ]
        )
    )


def wheel_cover_neg_y():
    return wheel_cover_pos_y().mirror(MIRROR_Y)


def tilt_hub():
    """Pins the 80T pulley to the 8 mm rod: set screw + side-loaded nut, tilt-home magnet pocket, 4 pulley inserts."""
    y0, y1 = Y_HUB0, Y_HUB0 + HUB_W
    ym = (y0 + y1) / 2
    tools = [
        hole("y", 8.2, 0, 0, y0 - EPS, y1 + EPS),
        hole("y", 6.2, 0, 13.0, y0 - EPS, y0 + 3.2),
        hole("x", P.M3_CLR, ym, 0, 0, 18.0),
        bxr(6.0, 8.6, ym - 3.2, y1 + EPS, -2.9, 2.9),
    ]
    for k in range(4):
        x, z = polar(11.0, 45 + 90 * k)
        tools.append(hole("y", P.M3_INSERT_D, x, z, y1 - 6.2, y1 + EPS))
    return hole("y", 34.0, 0, 0, y0, y1).cut(*tools)


def tilt_pulley_80t():
    """Printed GT2 80T (axis y): web bolts to the hub (screws through the open centre), toothed rim, outer flange."""
    T = P.T_TILT
    yt0, yt1 = Y_BELT - 4.0, Y_BELT + 4.0
    ring, rf = gt2_ring(T, yt0, yt1)
    body = ring.fuse(
        Pos(0, 0, (Y_P0 + yt0) / 2) * Cylinder(rf, yt0 - Y_P0),
        Pos(0, 0, yt1 + P.FLANGE_T / 2) * Cylinder(rf, P.FLANGE_T),
    )
    tools = [
        hole("z", 40.0, 0, 0, yt0, yt1 + P.FLANGE_T + EPS),
        hole("z", 9.0, 0, 0, Y_P0 - EPS, yt0 + EPS),
    ]
    tools += [
        hole("z", P.M3_CLR, *polar(11.0, 45 + 90 * k), Y_P0 - EPS, yt0 + EPS)
        for k in range(4)
    ]
    return Rot(-90, 0, 0) * body.cut(*tools)  # z -> y


def cable_clip():
    """Screw-down cable clip for 6 mm cable (use several)."""
    p = bxr(-10, 10, -5, 5, 0, 3).fuse(
        Pos(0, 0, 7.5) * Rot(90, 0, 0) * Cylinder(6.5, 10)
    )
    tools = [
        Pos(0, 0, 7.5) * Rot(90, 0, 0) * Cylinder(4.2, 10 + 2 * EPS),
        bxr(-1.5, 1.5, -6, 6, 7.5, 16),
        hole("z", P.M3_CLR, -8.5, 0, -EPS, 3 + EPS),
    ]
    return p.cut(*tools)


# name -> (builder, group, count, material)
REGISTRY = {
    "base_stator_plate": (base_stator_plate, "static", 1, "PETG"),
    "base_leg": (base_leg, "static", 4, "PETG"),
    "electronics_tray": (electronics_tray, "static", 1, "PLA"),
    "pan_motor_mount": (pan_motor_mount, "static", 1, "PETG"),
    "pan_hall_mount": (pan_hall_mount, "static", 1, "PLA"),
    "front_panel": (front_panel, "static", 1, "PLA"),
    "pan_rotor_plate": (pan_rotor_plate, "pan", 1, "PETG"),
    "pan_pulley_100t": (pan_pulley_100t, "pan", 1, "PETG"),
    "cable_collar": (cable_collar, "pan", 1, "PLA"),
    "yoke_left": (yoke_left, "pan", 1, "PETG"),
    "yoke_right": (yoke_right, "pan", 1, "PETG"),
    "tilt_motor_mount": (tilt_motor_mount, "pan", 1, "PETG"),
    "tilt_hub": (tilt_hub, "tilt", 1, "PETG"),
    "tilt_pulley_80t": (tilt_pulley_80t, "tilt", 1, "PETG"),
    "launcher_chassis": (launcher_chassis, "tilt", 1, "PETG"),
    "barrel_half_a": (barrel_half_a, "tilt", 1, "PETG"),
    "barrel_half_b": (barrel_half_b, "tilt", 1, "PETG"),
    "trunnion_beam": (trunnion_beam, "tilt", 1, "PETG"),
    "feed_elbow": (feed_elbow, "pan", 1, "PETG"),
    "hopper": (hopper, "pan", 1, "PLA"),
    "hopper_lid": (hopper_lid, "pan", 1, "PLA"),
    "piston": (piston, "tilt", 1, "PETG"),
    "servo_bracket": (servo_bracket, "tilt", 1, "PETG"),
    "pusher_arm": (pusher_arm, "tilt", 1, "PETG"),
    "wheel_cover_pos_y": (wheel_cover_pos_y, "tilt", 1, "PETG"),
    "wheel_cover_neg_y": (wheel_cover_neg_y, "tilt", 1, "PETG"),
    "camera_mount": (camera_mount, "tilt", 1, "PLA"),
    "camera_shim": (camera_shim, "tilt", 2, "PLA"),
    "cable_clip": (cable_clip, "static", 6, "PLA"),
}
