"""Module docks as DATA (spec 3): D1 launcher dock, D2 sensor dock, F1 breech flange, H1 harness panel, Y1 yoke seat.

Both mating modules import the same Dock record and build their features from it (`male_feature`, `female_void`, `bolt_holes`),
so a bolt-pattern edit here changes both sides. checks.C-DOCK does NOT trust this file: it probes the finished part solids.

Dock frame: origin = datum A point on the mating plane, +x = datum B direction, +z = out of the male side, y = z cross x.
`place(origin, xdir, zdir)` returns the build123d Location of a dock frame inside a module frame.
"""

from dataclasses import dataclass

import params as P
from build123d import Box, Cylinder, Plane, Polygon, Pos, extrude


@dataclass(frozen=True)
class Dock:
    name: str
    kind: str  # dovetail | dowel | ring | panel
    bolts: tuple  # (u, v) hole centres in the dock frame
    bolt_axis_len: float  # depth of the insert hole on the male/tapped side
    male_hole_d: float  # insert pilot (male side) diameter
    female_hole_d: float  # clearance (female side) diameter
    loc: dict  # kind-specific locator dimensions
    clearance: float = P.DOVE_CLEAR  # per side

    def clear(self):
        return self.clearance


DOCKS = {
    # 60 deg dovetail, 12 mm head, rails on both cartridge sides slide into the cradle arms from the rear; 2 x M3 clamp screws
    "D1": Dock(
        "D1",
        "dovetail",
        bolts=((8.0, 0.0), (36.0, 0.0)),
        bolt_axis_len=6.0,
        male_hole_d=P.M3_INSERT_D,
        female_hole_d=P.M3_CLR,
        loc=dict(
            length=P.CORE_X1 - P.X_F1,
            neck=P.RAIL_NECK,
            head=P.RAIL_HEAD_W,
            head_h=P.RAIL_HEAD_H,
            base_to_face=P.ARM_IN_Y
            - P.RAIL_Y0,  # neck height up to the arm face = head start
        ),
    ),
    # sensor bar on the cradle bridge: 2 x M4 on 40 mm + 6 mm dowel
    "D2": Dock(
        "D2",
        "dowel",
        bolts=((0.0, -P.BRIO_HOLE_PITCH / 2), (0.0, P.BRIO_HOLE_PITCH / 2)),
        bolt_axis_len=8.0,
        male_hole_d=P.M4_INSERT_D,
        female_hole_d=P.M4_CLR,
        loc=dict(dowel_d=P.DOWEL_D, dowel_h=4.0, dowel_uv=(12.0, 0.0)),
    ),
    # breech flange: feeder housing on the cartridge rear face, 4 x M3 on 50 x 40, ring lip aligns the bores
    "F1": Dock(
        "F1",
        "ring",
        bolts=((-25.0, -20.0), (25.0, -20.0), (-25.0, 20.0), (25.0, 20.0)),
        bolt_axis_len=6.0,
        male_hole_d=P.M3_INSERT_D,
        female_hole_d=P.M3_CLR,
        loc=dict(lip_od=P.BORE_ID + 4.5, lip_h=P.F1_LIP_H),
    ),
    # head harness panel (connector bulkhead) present on the power box AND on the yoke: same cutout, same 2 x M3
    "H1": Dock(
        "H1",
        "panel",
        bolts=((-12.0, 0.0), (12.0, 0.0)),
        bolt_axis_len=6.0,
        male_hole_d=P.M3_INSERT_D,
        female_hole_d=P.M3_CLR,
        loc=dict(
            cut_w=15.6 + 2 * P.DOVE_CLEAR,
            cut_h=8.2 + 2 * P.DOVE_CLEAR,
            conn_w=15.6,
            conn_h=8.2,
        ),
    ),
    # yoke foot on the rotor plate: 4 x M4 on 50 x 50 (spec 3 M2 -> M1)
    "Y1": Dock(
        "Y1",
        "dowel",
        bolts=((-25.0, -25.0), (25.0, -25.0), (-25.0, 25.0), (25.0, 25.0)),
        bolt_axis_len=8.0,
        male_hole_d=P.M4_INSERT_D,
        female_hole_d=P.M4_CLR,
        loc=dict(dowel_d=P.DOWEL_D, dowel_h=3.0, dowel_uv=(0.0, 0.0)),
    ),
}


def place(origin, xdir, zdir):
    """Location of a dock frame whose origin/x/z axes are given in the module frame."""
    return Plane(origin=origin, x_dir=xdir, z_dir=zdir).location


def _cylz(d, z0, z1, u=0.0, v=0.0):
    return Pos(u, v, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


def _prism_x(pts, x0, x1):
    """Polygon given as (y, z) points, extruded along +x from x0 to x1 (dock-frame helper)."""
    f = extrude(Plane.YZ * Polygon(*pts, align=None), amount=x1 - x0)
    return Pos(x0, 0, 0) * f


def male_feature(d, grow=0.0):
    """Locator of the male side in the dock frame (grow > 0 = offset outward: the female void is male_feature(d, clearance))."""
    L = d.loc
    if d.kind == "dovetail":
        hn, hh = L["base_to_face"], L["head_h"]
        n, h = L["neck"] / 2 + grow, L["head"] / 2 + grow
        pts = [(-n, 0.0), (n, 0.0), (n, hn), (h, hn + hh), (-h, hn + hh), (-n, hn)]
        return _prism_x(
            pts, -grow if grow else 0.0, L["length"] + (grow if grow else 0.0)
        )
    if d.kind in ("dowel",):
        u, v = L["dowel_uv"]
        return _cylz(L["dowel_d"] + 2 * grow, 0.0, L["dowel_h"] + grow, u, v)
    if d.kind == "ring":
        return _cylz(L["lip_od"] + 2 * grow, 0.0, L["lip_h"] + grow) - _cylz(
            P.BORE_ID, -1, L["lip_h"] + 2
        )
    if d.kind == "panel":
        return Pos(0, 0, 1.5) * Box(
            L["conn_w"] + 2 * grow, L["conn_h"] + 2 * grow, 3.0 + grow
        )
    raise ValueError(d.kind)


def female_void(d):
    """Void cut into the female part, in the dock frame (z < 0 goes into the female part for non-dovetail kinds)."""
    c = d.clear()
    if d.kind == "dovetail":
        L = d.loc
        hn = L["base_to_face"]
        # opening through the arm face (neck + 2 c) is the region below the face plane; the undercut head pocket sits above it
        v = male_feature(d, c)
        return v
    if d.kind == "dowel":
        u, v = d.loc["dowel_uv"]
        return _cylz(d.loc["dowel_d"] + 2 * c, -0.05, d.loc["dowel_h"] + c, u, v)
    if d.kind == "ring":
        return _cylz(d.loc["lip_od"] + 2 * c, -0.05, d.loc["lip_h"] + c)
    if d.kind == "panel":
        return Pos(0, 0, 0) * Box(d.loc["cut_w"], d.loc["cut_h"], 40.0)
    raise ValueError(d.kind)


def bolt_holes(d, side):
    """Bolt holes in the dock frame. side 'male' = insert pilots going INTO the male part (-z), 'female' = clearance going through
    the female part. Both extend across the mating plane so a probe finds the void on either side."""
    out = None
    for u, v in d.bolts:
        if side == "male":
            h = _cylz(d.male_hole_d, -d.bolt_axis_len, 0.5, u, v)
        else:
            h = _cylz(d.female_hole_d, -0.5, 12.0, u, v)
        out = h if out is None else out + h
    return out


def probe_points(d):
    """Expected hole centres (u, v) in the dock frame: what C-DOCK looks for on each side."""
    return [tuple(b) for b in d.bolts]
