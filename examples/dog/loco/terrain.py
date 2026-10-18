"""Terrain side of the dog's controller (SP4): the height-map interface SP5's elevation map will implement, the ground-truth stand-in, and
(Task 8) footholds, posture plane, step height."""

from typing import Protocol

import numpy as np


class HeightMap(Protocol):
    def height(
        self, x, y
    ) -> np.ndarray: ...  # world z, numpy-broadcast; nan where the map has no cell


class FlatMap:
    def __init__(self, z=0.0):
        self.z = float(z)

    def height(self, x, y):
        return np.full(np.broadcast(np.asarray(x), np.asarray(y)).shape, self.z)


class TruthMap:
    """SP5 stand-in: the terrain's exact ground truth (napl.terrain Field.height: MuJoCo's hfield triangulation, box tops)."""

    def __init__(self, field):
        self.f = field

    def height(self, x, y):
        return np.asarray(
            self.f.height(np.asarray(x, float), np.asarray(y, float)), float
        )


# ---- footholds, posture plane, swing height ---------------------------------------------------------------------------------------
# On a FlatMap every function returns the nominal exactly (the flat trot's bits do not depend on the terrain code).
EDGE_OK = 0.015  # m: height spread within 4 cm that the 35 mm rubber ball sits on without rolling off
W_EDGE, W_SLOPE = (
    4.0,
    2.0,
)  # cost per cm of spread above EDGE_OK, per rad of local slope; distance term = |d|^2 / r^2 (1 at the rim)
POSTURE_PATCH = (
    0.6,
    0.4,
)  # m: footprint the posture plane is fitted over (about the stance rectangle)
R_BALL = 0.0175  # m: the foot ball radius (apex_over works on ball centres)
APEX_MAX = 0.12  # m: highest swing apex the leg can make in a swing (knee stop: ~7 cm under the hip; asking for more folds the leg under the body)
RISER = 0.08  # m per 5 cm ahead that makes the terrain "rough" (a 25 deg ramp is 2.3 cm per 5 cm)


def foothold(hmap, xy, r_foot, r=0.12, step=0.02, edge_r=0.04):
    """Ball centre (3,) for a touchdown near xy: the cheapest of a disc of candidates, cost = distance from the nominal + height spread within
    edge_r (a step edge, a hole) + local slope. nan z where the map knows nothing around the nominal."""
    if isinstance(hmap, FlatMap):
        return np.array([xy[0], xy[1], hmap.z + r_foot])
    g = np.arange(-r, r + 1e-9, step)
    X, Y = np.meshgrid(g, g)
    keep = X**2 + Y**2 <= r * r + 1e-12
    cx, cy = xy[0] + X[keep], xy[1] + Y[keep]
    h0 = hmap.height(cx, cy)
    ang = np.arange(8) * np.pi / 4
    hn = np.stack(
        [hmap.height(cx + edge_r * np.cos(a), cy + edge_r * np.sin(a)) for a in ang]
    )
    spread = np.max(np.abs(hn - h0), axis=0)
    cost = (
        (X[keep] ** 2 + Y[keep] ** 2) / r**2
        + W_EDGE * np.clip(spread - EDGE_OK, 0, None) / 0.01
        + W_SLOPE * np.arctan(spread / edge_r)
    )
    cost[~np.isfinite(h0) | ~np.isfinite(spread)] = np.inf
    k = int(np.argmin(cost))
    if not np.isfinite(cost[k]):
        return np.array(
            [xy[0], xy[1], np.nan]
        )  # the controller keeps the nominal at the lift-off height
    return np.array([cx[k], cy[k], h0[k] + r_foot])


TILT_MAX = np.radians(
    40.0
)  # the friction cones are never tilted further than this off the vertical


def normal(hmap, xy, d=0.05):
    """Unit ground normal at xy from central differences of the map over +-d (the direction the friction cone of a foot there is about);
    (0, 0, 1) on a FlatMap, where the map knows nothing, and for slopes beyond TILT_MAX (a riser: the foothold is elsewhere)."""
    up = np.array([0.0, 0.0, 1.0])
    if isinstance(hmap, FlatMap):
        return up
    zx = (hmap.height(xy[0] + d, xy[1]) - hmap.height(xy[0] - d, xy[1])) / (2 * d)
    zy = (hmap.height(xy[0], xy[1] + d) - hmap.height(xy[0], xy[1] - d)) / (2 * d)
    if not (np.isfinite(zx) and np.isfinite(zy)):
        return up
    n = np.array([-zx, -zy, 1.0])
    n /= np.linalg.norm(n)
    return n if n[2] >= np.cos(TILT_MAX) else up


def apex_over(hmap, p0, p1, apex, clear=0.05):
    """Swing apex (above the higher end, ball centres p0 -> p1) that clears the highest ground on the straight path by `clear`."""
    if isinstance(hmap, FlatMap):
        return apex
    s = np.linspace(0, 1, 12)
    pts = p0[None, :2] + (p1[:2] - p0[:2])[None, :] * s[:, None]
    top = np.nanmax(hmap.height(pts[:, 0], pts[:, 1]))
    return min(max(apex, top + clear - (max(p0[2], p1[2]) - R_BALL)), APEX_MAX)


def plane(hmap, xy, yaw):
    """Least-squares plane z = a x' + b y' + c over the body footprint in the yaw frame -> (roll, pitch, c)."""
    if isinstance(hmap, FlatMap):
        return 0.0, 0.0, hmap.z
    lx, ly = POSTURE_PATCH
    u, v = np.meshgrid(np.linspace(-lx / 2, lx / 2, 5), np.linspace(-ly / 2, ly / 2, 5))
    c, s = np.cos(yaw), np.sin(yaw)
    z = hmap.height(xy[0] + c * u - s * v, xy[1] + s * u + c * v)
    ok = np.isfinite(z)
    if ok.sum() < 6:
        return 0.0, 0.0, float(np.nanmean(z)) if ok.any() else 0.0
    A = np.column_stack([u[ok], v[ok], np.ones(ok.sum())])
    a, b, c0 = np.linalg.lstsq(A, z[ok], rcond=None)[0]
    return float(np.arctan(b)), float(-np.arctan(a)), float(c0)


SLOPE_WALK = np.radians(
    12.0
)  # ground steeper than this ahead: the walk gait (three feet down; the trot's two-foot support tips on a 25 deg ramp)


def steep_ahead(hmap, xy, yaw, dist):
    """True if the ground along the heading climbs or falls faster than SLOPE_WALK over any 0.3 m of the next `dist` m."""
    if isinstance(hmap, FlatMap):
        return False
    s = np.arange(0.0, dist + 1e-9, 0.1)
    z = hmap.height(xy[0] + np.cos(yaw) * s, xy[1] + np.sin(yaw) * s)
    d = np.abs(z[3:] - z[:-3]) / 0.3
    return bool(d.size and np.isfinite(d).any() and np.nanmax(d) > np.tan(SLOPE_WALK))


def rough_ahead(hmap, xy, yaw, dist):
    if isinstance(hmap, FlatMap):
        return False
    s = np.arange(0.0, dist, 0.05)
    z = hmap.height(xy[0] + np.cos(yaw) * s, xy[1] + np.sin(yaw) * s)
    d = np.abs(np.diff(z))
    return bool(d.size and np.isfinite(d).any() and np.nanmax(d) > RISER)
