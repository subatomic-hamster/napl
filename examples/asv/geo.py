"""Frames and angles of the ASV code.

World / ENU: x East, y North, z up, origin at (LAT0, LON0) -- the GNSS device's flat-earth frame (docs/models.md, GNSS). Yaw psi is the math angle:
counter-clockwise from East, radians. A compass heading (clockwise from North) is `heading_deg(psi)`. Relative bearings (COLREG) are positive to starboard.
"""

import math

import numpy as np

A_WGS84 = 6378137.0
F_WGS84 = 1 / 298.257223563
E2 = F_WGS84 * (2 - F_WGS84)
LAT0, LON0 = (
    47.3769,
    8.5417,
)  # origin of the scene; the same numbers as the gnss binding of assemblies/asv.toml


def wrap(a):
    """Angle in (-pi, pi]."""
    return math.remainder(a, 2 * math.pi)


def radii(lat0_deg):
    """Meridian and prime-vertical radii of curvature at the origin latitude (the GNSS device's M and N)."""
    s = math.sin(math.radians(lat0_deg))
    w = 1 - E2 * s * s
    return A_WGS84 * (1 - E2) / w**1.5, A_WGS84 / math.sqrt(w)


def enu_from_latlon(lat, lon, lat0=LAT0, lon0=LON0):
    """(East, North) metres of a lat/lon in degrees, the inverse of the GNSS device's flat-earth map."""
    m, n = radii(lat0)
    return math.radians(lon - lon0) * n * math.cos(math.radians(lat0)), math.radians(
        lat - lat0
    ) * m


def latlon_from_enu(x, y, lat0=LAT0, lon0=LON0):
    m, n = radii(lat0)
    return lat0 + math.degrees(y / m), lon0 + math.degrees(
        x / (n * math.cos(math.radians(lat0)))
    )


def heading_deg(psi):
    """Compass heading (degrees clockwise from North, [0, 360)) of a math yaw."""
    return math.degrees(math.pi / 2 - psi) % 360.0


def psi_from_heading_deg(h):
    return wrap(math.pi / 2 - math.radians(h))


def vec(psi):
    return np.array([math.cos(psi), math.sin(psi)])


def yaw_of_quat(q):
    """Yaw (math, CCW from East) of a unit quaternion (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rel_bearing(psi, own, tgt):
    """Bearing of `tgt` from `own` relative to the bow `psi`, positive to starboard (clockwise), in (-pi, pi]."""
    return wrap(-(math.atan2(tgt[1] - own[1], tgt[0] - own[0]) - psi))


KN = 1852.0 / 3600.0  # m/s per knot
