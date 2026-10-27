"""build123d helpers shared by the cuas2 modules (mm). Same conventions as cad/cuas-lite/parts.py."""

import math

from build123d import Box, Cylinder, Plane, Polygon, Pos, Rot, extrude


def bxr(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        x1 - x0, y1 - y0, z1 - z0
    )


def hole(ax, d, u, v, c0, c1):
    """Cylinder dia d along axis ax from c0 to c1; (u, v) = the other two coordinates in x, y, z order."""
    h, m = c1 - c0, (c0 + c1) / 2
    cyl = Cylinder(d / 2, h)
    if ax == "z":
        return Pos(u, v, m) * cyl
    if ax == "x":
        return Pos(m, u, v) * Rot(0, 90, 0) * cyl
    return Pos(u, m, v) * Rot(90, 0, 0) * cyl


def tube(ax, r_out, r_in, u, v, c0, c1):
    return hole(ax, 2 * r_out, u, v, c0, c1) - hole(ax, 2 * r_in, u, v, c0 - 1, c1 + 1)


def prism_xz(pts, y0, y1):
    """Polygon of (x, z) points extruded across y0..y1."""
    f = extrude(
        Plane.XZ * Polygon(*pts, align=None), amount=y1 - y0
    )  # Plane.XZ normal is -y
    bb = f.bounding_box()
    return Pos(0, y0 - bb.min.Y, 0) * f


def slot_z(ax, d, u, v, c0, c1, travel):
    """Stadium slot of width d, long axis along z, +-travel, drilled along ax ('y' or 'x')."""
    bu, bv = d, 2 * travel
    if ax == "y":
        b = bxr(u - bu / 2, u + bu / 2, c0, c1, v - bv / 2, v + bv / 2)
    else:
        b = bxr(c0, c1, u - bu / 2, u + bu / 2, v - bv / 2, v + bv / 2)
    return b + hole(ax, d, u, v - travel, c0, c1) + hole(ax, d, u, v + travel, c0, c1)


def vol(a, b):
    try:
        i = a & b
        return float(i.volume) if i is not None else 0.0
    except Exception:
        return 0.0


def void(s, x, y, z):
    return not s.is_inside((x, y, z))


def rot_y(a_deg):
    """Rotation about +y by a_deg (build123d Rot(0, a, 0))."""
    return Rot(0, a_deg, 0)


def about(pt, rot):
    """Apply `rot` about the point pt (x, y, z)."""
    return Pos(*pt) * rot * Pos(-pt[0], -pt[1], -pt[2])


S3 = math.sqrt(3)
