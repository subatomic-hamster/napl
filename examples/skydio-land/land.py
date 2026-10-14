"""Landing components of the Skydio-style quadrotor (numpy only, no kernel): landing-zone detection from the downward lidar and the depth map, the descent
profile and the RTK position update of the estimator.

LANDING-ZONE DETECTION (`GroundMap.assess`). A survey pass at `survey_agl` over the designated zone collects the downward lidar's ground points (x, y, z) in the
ESTIMATE frame (the pose of the estimator, so the map is as good as the odometry: relative geometry within the 40 s of a survey is accurate to a few cm
even when the absolute position has drifted) and the places where it saw nothing (a drop-off: range beyond 8 m). For every candidate spot within `search` of the
designated centre:
  footprint  the points inside `r_foot` (the airframe's bounding disc + margin) must be dense enough (coverage) and fit a plane (trimmed least squares) with a
             slope <= `slope_max_deg` and a robust roughness <= `rms_max` (flatness);
  clear ring the points between `r_foot` and `r_clear` must not rise more than `step_max` above that plane (boulder, log, bush), fall more than `drop_max` below
             it (a step or a cliff edge) nor be missing (more than `void_max` of the pulses returned nothing);
  obstacles  no voxel of the depth map (examples/skydio/mapping.py) stands higher than `obs_h` above the plane within `obs_r` of the spot (a tree, a pole, a wall).
The spot with the best score (flat, smooth, close to the designated centre) wins; no spot -> the zone is rejected with the dominant reason and the next one is tried.

DECKS are in land_deck.py (RelGnss + DeckLink + DeckPredictor of napl.cuas.relnav).
"""

import math
from collections import Counter
from dataclasses import dataclass, field

import numpy as np


@dataclass
class LandParams:
    survey_agl: float = 3.0  # m: gear height above the ground during the survey (the lidar's footprint at 3 m is ~0.1 m)
    survey_speed: float = 2.0  # m/s along a leg
    leg_half: float = (
        6.0  # m: half length of a survey leg (along the approach direction)
    )
    leg_gap: float = 1.25  # m between legs (a 0.5 m boulder is crossed by at least one)
    n_legs: int = 6
    search: float = 1.5  # m: candidate spots within this of the designated centre
    step: float = 0.5  # m: candidate grid
    r_foot: float = 1.5  # m: footprint disc: X500 prop tips at 0.38 m from the centre, legs ~0.3 m, + margin for the position error
    r_rise: float = 4.0  # m: ring in which nothing may stand taller than step_max
    r_clear: float = 6.0  # m: ring in which there may be no drop-off deeper than drop_max and no missing return
    slope_max_deg: float = 8.0
    rms_max: float = 0.09  # m (robust roughness of the footprint; the lidar's own noise is 0.03 m + 1 % of range = 0.06 m at the 3 m survey height)
    step_max: float = 0.30  # m: tallest thing allowed in the ring
    drop_max: float = 0.45  # m: deepest hole / step down allowed in the ring
    void_max: float = 0.04  # share of pulses with no return allowed in the ring
    n_min: int = 120  # lidar points needed in the footprint
    obs_r: float = 2.5  # m: horizontal reach of the depth-map obstacle test
    obs_h: float = 0.35  # m above the plane: voxels higher than this count as obstacles
    v_fast: float = 0.8  # m/s descent from the survey height to 1 m
    v_slow: float = 0.45  # m/s from 1 m down
    v_touch: float = 0.30  # m/s final sink, held through contact
    h_final: float = 1.0  # m: gear height where the final sink begins
    touch_agl: float = 0.04  # m: gear height at which the lidar says "on the ground"


@dataclass
class Assessment:
    ok: bool
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0  # plane height at the spot (estimate frame)
    slope_deg: float = 0.0
    rms: float = 0.0
    n: int = 0
    score: float = math.inf
    reason: str = ""
    reasons: dict = field(default_factory=dict)  # candidates rejected, by reason

    def brief(self):
        return dict(ok=self.ok, x=round(self.x, 2), y=round(self.y, 2), z=round(self.z, 2), slope_deg=round(self.slope_deg, 2), rms=round(self.rms, 3), n=self.n,
                    reason=self.reason, reasons=self.reasons)  # fmt: skip


def robust_plane(x, y, z, iters=3, trim=2.5):
    """z = a x + b y + c by least squares with trimming (points beyond `trim` robust sigmas are dropped) -> (a, b, c, robust sigma, kept mask)."""
    A = np.column_stack([x, y, np.ones_like(x)])
    keep = np.ones(len(x), bool)
    for _ in range(iters):
        coef, *_ = np.linalg.lstsq(A[keep], z[keep], rcond=None)
        res = z - A @ coef
        sig = 1.4826 * float(np.median(np.abs(res[keep] - np.median(res[keep])))) + 1e-3
        keep = np.abs(res) < max(trim * sig, 0.03)
        if keep.sum() < 20:
            break
    return coef[0], coef[1], coef[2], sig, keep


class GroundMap:
    """Ground points seen by the downward lidar, in the estimate frame; `void` = nadir positions of pulses that returned nothing."""

    def __init__(self):
        self.pts, self.void = [], []

    def add(self, x, y, z):
        self.pts.append((x, y, z))

    def add_void(self, x, y):
        self.void.append((x, y))

    def clear(self):
        self.pts, self.void = [], []

    def assess(self, c0, prm: LandParams, occ=None):
        """-> Assessment of the best spot within prm.search of c0 = (x, y). `occ`: (n, 3) centres of occupied depth-map voxels near the zone, or None."""
        S = np.asarray(self.pts, float).reshape(-1, 3)
        V = np.asarray(self.void, float).reshape(-1, 2)
        why = Counter()
        best = None
        if len(S) < prm.n_min:
            return Assessment(False, reason="no ground data", n=len(S))
        g = np.arange(-prm.search, prm.search + 1e-9, prm.step)
        for dx in g:
            for dy in g:
                c = np.array([c0[0] + dx, c0[1] + dy])
                d = np.hypot(S[:, 0] - c[0], S[:, 1] - c[1])
                A = S[d <= prm.r_foot]
                if len(A) < prm.n_min:
                    why["coverage"] += 1
                    continue
                a, b, c0z, sig, keep = robust_plane(
                    A[:, 0] - c[0], A[:, 1] - c[1], A[:, 2]
                )
                slope = math.degrees(math.atan(math.hypot(a, b)))
                if slope > prm.slope_max_deg:
                    why["slope"] += 1
                    continue
                if sig > prm.rms_max or keep.mean() < 0.9:
                    why["rough"] += 1
                    continue
                ring = S[(d > prm.r_foot) & (d <= prm.r_clear)]
                if len(ring):
                    dev = ring[:, 2] - (
                        a * (ring[:, 0] - c[0]) + b * (ring[:, 1] - c[1]) + c0z
                    )
                    near_ring = (
                        np.hypot(ring[:, 0] - c[0], ring[:, 1] - c[1]) <= prm.r_rise
                    )
                    if near_ring.any() and dev[near_ring].max() > prm.step_max:
                        why["obstacle"] += 1
                        continue
                    if dev.min() < -prm.drop_max:
                        why["drop-off"] += 1
                        continue
                if len(V):
                    nv = int(
                        (np.hypot(V[:, 0] - c[0], V[:, 1] - c[1]) <= prm.r_clear).sum()
                    )
                    if nv > prm.void_max * (nv + len(ring) + len(A)):
                        why["no return"] += 1
                        continue
                if occ is not None and len(occ):
                    h = occ[:, 2] - (
                        a * (occ[:, 0] - c[0]) + b * (occ[:, 1] - c[1]) + c0z
                    )
                    near = np.hypot(occ[:, 0] - c[0], occ[:, 1] - c[1]) <= prm.obs_r
                    if bool(((h > prm.obs_h) & near).any()):
                        why["map obstacle"] += 1
                        continue
                score = (
                    slope / prm.slope_max_deg
                    + sig / prm.rms_max
                    + 0.2 * math.hypot(dx, dy)
                )
                if best is None or score < best.score:
                    best = Assessment(
                        True,
                        float(c[0]),
                        float(c[1]),
                        float(c0z),
                        slope,
                        float(sig),
                        len(A),
                        score,
                    )
        if best is None:
            return Assessment(
                False, reason=why.most_common(1)[0][0], reasons=dict(why), n=len(S)
            )
        best.reasons = dict(why)
        return best


def trim_leg(win, a, b, z, r_safe, step=0.25, margin=0.3, min_len=2.5):
    """A survey leg a -> b (xy) at height z, reduced to its longest stretch that is clear in the depth map (a `mapping.Window`: r_safe + margin from every
    occupied voxel). -> (p, q) in the direction a -> b, or None when no stretch is min_len long."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(int(np.linalg.norm(b - a) / step), 1)
    pts = np.array([[*(a + (b - a) * k / n), z] for k in range(n + 1)])
    free = win.clearance(pts) >= r_safe + margin
    best, i = (0, 0), 0
    while i <= n:
        if free[i]:
            j = i
            while j < n and free[j + 1]:
                j += 1
            if j - i > best[1] - best[0]:
                best = (i, j)
            i = j + 1
        else:
            i += 1
    if (best[1] - best[0]) * step < min_len:
        return None
    return a + (b - a) * best[0] / n, a + (b - a) * best[1] / n


# ---- depth in rain ------------------------------------------------------------------------------------------------------------------
def despeckle(depth, unknown=1.0e3, tol=0.15, k_hit=3, k_miss=6):
    """Depth image cleaned of the speckle that rain leaves in it (a raindrop in the beam: a false NEAR return on an otherwise coherent surface; an attenuated return: a dropout in
    a valid neighbourhood). A pixel is judged by its 8 neighbours: a valid pixel with fewer than `k_hit` neighbours at a similar depth (within `tol` of it + 0.1 m) is a speckle; a
    no-return (inf) pixel with at least `k_miss` valid neighbours is a dropout, not free space. Both become `unknown` (a finite depth beyond the mapper's hit range: neither a hit nor
    a carved ray, mapping.integrate). Thin things one or two pixels wide (wires, twigs) are lost with the speckle: the price of the filter (README.md)."""
    h, w = depth.shape
    pad = np.pad(depth, 1, constant_values=np.inf)
    nb = [
        pad[i : i + h, j : j + w]
        for i in range(3)
        for j in range(3)
        if (i, j) != (1, 1)
    ]
    fin = np.isfinite(depth)
    with np.errstate(invalid="ignore"):
        similar = sum(
            np.isfinite(n) & (np.abs(n - depth) <= tol * depth + 0.1) for n in nb
        )
    valid_nb = sum(np.isfinite(n) for n in nb)
    bad = (fin & (similar < k_hit)) | (~fin & (valid_nb >= k_miss))
    out = depth.copy()
    out[bad] = unknown
    return out


# ---- estimator: RTK position fix ----------------------------------------------------------------------------------------------------
def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Flat-earth east / north / up of a fix w.r.t. (lat0, lon0, alt0): the inverse of the kernel's GNSS (sensors.cpp, WGS84 meridian and prime-vertical radii at lat0)."""
    a, e2 = 6378137.0, 6.69437999014e-3
    s = math.sin(math.radians(lat0))
    w = 1.0 - e2 * s * s
    r_pv, r_mer = a / math.sqrt(w), a * (1.0 - e2) / w**1.5
    return ((lon - lon0) * math.radians(1) * r_pv * math.cos(math.radians(lat0)),
            (lat - lat0) * math.radians(1) * r_mer, alt - alt0)  # fmt: skip


def update_position(eskf, t_sample, z, sigma):
    """Position measurement (world, m) sampled at filter time t_sample, delayed: the innovation is against the state history of that instant (the VIO's
    lag treatment, estimator.Eskf.update_vio)."""
    p_h, _, _ = eskf._state_at(t_sample)
    H = np.zeros((3, 15))
    H[:, 0:3] = np.eye(3)
    eskf._inject(H, np.asarray(z, float) - p_h, np.eye(3) * sigma**2)


# ---- descent profile ---------------------------------------------------------------------------------------------------------------
def sink_speed(agl, prm: LandParams):
    """Commanded descent speed (m/s, positive down) at gear height `agl`: fast above 1 m, then slower, the final sink held through contact."""
    if agl > prm.h_final + 0.5:
        return prm.v_fast
    if agl > prm.h_final:
        return prm.v_slow + (prm.v_fast - prm.v_slow) * (agl - prm.h_final) / 0.5
    return prm.v_slow + (prm.v_touch - prm.v_slow) * (1.0 - agl / prm.h_final)


# The deck-relative interface (DeckLink, RelGnss, DeckPredictor) is napl.cuas.relnav; land_deck.py wires the lander to it.
