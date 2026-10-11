"""Obstacle and traffic detection from the onboard sensors: a 360 degree 2-D lidar scan (RPLIDAR S2 model) is segmented into clusters, the clusters feed a
constant-velocity Kalman tracker, and AIS reports (asv.ais) feed the same tracker. Nothing here reads the simulator's truth: inputs are a scan, the estimated
own pose, and received AIS messages.

Frames: world ENU; the scan is in the lidar's frame: azimuth a_j = 2 pi ((j + 1/2) / n - 1/2) from the bow, + to port (the device's convention for the S2 optical port,
checked against a known object in tests/python/test_asv.py).
Fidelity: the lidar plane tilts with roll / pitch in waves and the scan is idealised as instantaneous (no motion distortion over the 0.1 s revolution);
a centroid of the visible face of a cluster is biased towards the observer by up to half the object's extent, which the measurement noise accounts for.
"""

import math
from dataclasses import dataclass, field

import numpy as np

LIDAR_X = 0.45  # m, lidar ahead of the GNSS puck (assemblies/asv.toml: masts at x = +0.30 and -0.15)
SELF_MASK = 1.0  # m, nearer returns are the vessel's own structure
TRACK_EXT_MAX = 6.0  # m, a cluster longer than this is shoreline / pier, not a vessel or buoy: it stays a static point set


def scan_points(ranges, psi, pos, n=None, lidar_x=LIDAR_X):
    """Valid returns of one scan -> (xy_world (m, 2), xy_body (m, 2), range (m,), bearing_body (rad, + port), index (m,))."""
    r = np.asarray(ranges, float)
    n = len(r)
    j = np.arange(n)
    a = 2 * math.pi * ((j + 0.5) / n - 0.5)
    ok = (r > SELF_MASK) & np.isfinite(r)
    r, a, j = r[ok], a[ok], j[ok]
    bx = lidar_x + r * np.cos(a)
    by = r * np.sin(a)
    c, s = math.cos(psi), math.sin(psi)
    wx = pos[0] + c * bx - s * by
    wy = pos[1] + s * bx + c * by
    return np.stack([wx, wy], 1), np.stack([bx, by], 1), r, a, j


def clusters(ranges, psi, pos, gap0=0.6, gap1=0.06):
    """Segment a scan into clusters of neighbouring returns whose ranges differ by less than gap0 + gap1 r (and at most one missing ray between).

    -> list of dict(c=centroid world xy, ext=extent m, n=points, rng=range of the nearest point, brg=its bearing (+ port), pts=world points)."""
    w, b, r, a, j = scan_points(ranges, psi, pos)
    if len(r) == 0:
        return []
    n = len(ranges)
    breaks = (
        (np.diff(j) > 4)
        | (  # up to three dropped rays inside a cluster
            np.abs(np.diff(r)) > gap0 + gap1 * 0.5 * (r[1:] + r[:-1])
        )
    )
    idx = np.split(np.arange(len(r)), np.nonzero(breaks)[0] + 1)
    # wrap-around: the first and last segments touch across the scan seam
    if (
        len(idx) > 1
        and (j[0] + n - j[-1]) <= 4
        and abs(r[0] - r[-1]) <= gap0 + gap1 * 0.5 * (r[0] + r[-1])
    ):
        idx[0] = np.concatenate([idx[-1], idx[0]])
        idx.pop()
    out = []
    for ii in idx:
        if (
            len(ii) < 2
        ):  # a single return is noise or a spar; two rays at 30 m are already 0.26 m apart
            continue
        p = w[ii]
        ext = float(np.hypot(*(p.max(0) - p.min(0))))
        k = int(np.argmin(r[ii]))
        out.append(
            dict(
                c=p.mean(0),
                ext=ext,
                n=len(ii),
                rng=float(r[ii][k]),
                brg=float(a[ii][k]),
                pts=p,
            )
        )
    return out


@dataclass
class Track:
    id: int
    x: np.ndarray  # E, N, vE, vN
    P: np.ndarray
    t: float
    hits: int = 0
    lid_t: float = -1e9  # time of the last lidar update
    ais_t: float = -1e9  # time of the last AIS update
    mmsi: int = 0
    ext: float = 0.0
    pts: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    t0: float = 0.0  # when the track was started

    @property
    def pos(self):
        return self.x[:2]

    @property
    def vel(self):
        return self.x[2:]

    @property
    def speed(self):
        return float(np.hypot(*self.x[2:]))

    def source(self, t):
        lid, ais = t - self.lid_t < 1.0, t - self.ais_t < 25.0
        return "fused" if lid and ais else "lidar" if lid else "ais" if ais else "coast"


class Tracker:
    """Global-nearest-neighbour constant-velocity tracker. `update_lidar(t, clusters)` and `update_ais(t, reports)` in any order; `moving(t)` the confirmed
    contacts that are vessels under way (speed above `v_min`), whose clusters are then not static obstacles."""

    def __init__(
        self, v_min=0.35, gate=4.0, q_acc=0.2, confirm=4, drop_s=4.0, ais_gate=14.0
    ):
        self.tr: list[Track] = []
        self.v_min, self.gate, self.qa, self.confirm = v_min, gate, q_acc, confirm
        self.drop_s, self.ais_gate = drop_s, ais_gate
        self._id = 0

    # -- filter -------------------------------------------------------------------------------------------------------------------------
    def _predict(self, tr: Track, t):
        dt = t - tr.t
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.qa * np.array([[dt**3 / 3, dt**2 / 2], [dt**2 / 2, dt]])
        Q = np.zeros((4, 4))
        for k in (0, 1):
            Q[np.ix_([k, k + 2], [k, k + 2])] = q
        tr.x = F @ tr.x
        tr.P = F @ tr.P @ F.T + Q
        tr.t = t

    @staticmethod
    def _correct(tr: Track, H, z, R):
        S = H @ tr.P @ H.T + R
        K = tr.P @ H.T @ np.linalg.inv(S)
        tr.x = tr.x + K @ (z - H @ tr.x)
        tr.P = (np.eye(4) - K @ H) @ tr.P

    def _new(self, t, pos, vel, sig_pos, sig_vel):
        self._id += 1
        tr = Track(
            self._id,
            np.array([*pos, *vel], float),
            np.diag([sig_pos**2] * 2 + [sig_vel**2] * 2),
            t,
            t0=t,
        )
        self.tr.append(tr)
        return tr

    # -- lidar --------------------------------------------------------------------------------------------------------------------------
    def update_lidar(self, t, cl):
        for tr in self.tr:
            self._predict(tr, t)
        cand = [c for c in cl if c["ext"] <= TRACK_EXT_MAX]
        pairs = []
        for i, tr in enumerate(self.tr):
            for k, c in enumerate(cand):
                d = float(np.hypot(*(tr.pos - c["c"])))
                # an AIS track the lidar does not see yet sits in the AIS frame (our GNSS error plus the other vessel's): wide gate, then snap to the lidar
                blind = tr.mmsi != 0 and t - tr.lid_t > 2.0
                if d < (self.ais_gate if blind else self.gate) + c["ext"]:
                    pairs.append((d, i, k))
        used_t, used_c = set(), set()
        H = np.hstack([np.eye(2), np.zeros((2, 2))])
        for d, i, k in sorted(pairs):
            if i in used_t or k in used_c:
                continue
            used_t.add(i)
            used_c.add(k)
            tr, c = self.tr[i], cand[k]
            sig = 0.35 + 0.01 * c["rng"] + 0.3 * c["ext"]
            if (
                tr.mmsi != 0 and t - tr.lid_t > 2.0
            ):  # first lidar sight of an AIS contact: take the lidar position (our frame), keep the AIS velocity
                tr.x[:2] = c["c"]
                tr.P[:2, :2] = np.eye(2) * sig**2
                tr.P[:2, 2:] = tr.P[2:, :2] = (
                    0.0  # the position is no longer correlated with the velocity: a stale cross term made the next scan's correction fling the velocity to 28 m/s
                )
            else:
                self._correct(tr, H, c["c"], np.eye(2) * sig**2)
            tr.hits += 1
            tr.lid_t = t
            tr.ext = c["ext"]
            tr.pts = c["pts"]
        for k, c in enumerate(cand):
            if k not in used_c:
                # a fragment of a contact that is already tracked (a dropped ray splits one hull into two clusters) is not a new contact
                if any(
                    float(np.hypot(*(q.pos - c["c"]))) < 3.0 + 0.5 * (c["ext"] + q.ext)
                    for q in self.tr
                ):
                    continue
                tr = self._new(t, c["c"], (0.0, 0.0), 1.0, 2.0)
                tr.hits, tr.lid_t, tr.ext, tr.pts = 1, t, c["ext"], c["pts"]
        self.tr = [tr for tr in self.tr if self.alive(tr, t)]

    def alive(self, tr, t):
        """A lidar contact is dropped after `drop_s` without a return; an AIS contact after 2.5 reporting intervals of the slowest class (25 s)."""
        return t - tr.lid_t < self.drop_s or t - tr.ais_t < 25.0

    # -- AIS ----------------------------------------------------------------------------------------------------------------------------
    def update_ais(self, t, reports):
        """Reports from asv.ais.Channel.receive: id, t_tx, x, y, vx, vy. Position of a report is `t - t_tx` old: moved on by its own velocity."""
        H4 = np.eye(4)
        for tr in self.tr:
            self._predict(tr, t)
        for m in sorted(reports, key=lambda m: m["t_tx"]):
            pos = np.array([m["x"], m["y"]]) + np.array([m["vx"], m["vy"]]) * (
                t - m["t_tx"]
            )
            vel = np.array([m["vx"], m["vy"]])
            tr = next((q for q in self.tr if q.mmsi == m["id"]), None)
            if (
                tr is None
            ):  # a track the lidar has already started, nearest to the report within the gate
                best = min(
                    self.tr, key=lambda q: float(np.hypot(*(q.pos - pos))), default=None
                )
                if (
                    best is not None
                    and best.mmsi == 0
                    and float(np.hypot(*(best.pos - pos))) < self.ais_gate
                ):
                    tr = best
            if tr is None:
                tr = self._new(t, pos, vel, 3.0, 0.3)
                tr.hits = self.confirm
            tr.mmsi = m["id"]
            tr.ais_t = t
            tr.hits = max(tr.hits, self.confirm)
            if (
                t - tr.lid_t > 3.0
            ):  # nobody sees it with the lidar: the (own-frame-biased) AIS position is all there is
                self._correct(
                    tr, H4, np.array([*pos, *vel]), np.diag([3.0**2] * 2 + [0.2**2] * 2)
                )
            else:  # the lidar fixes the position in OUR frame; AIS adds the velocity (white 0.1 m/s) which the CV filter would need many scans to find
                self._correct(
                    tr,
                    np.hstack([np.zeros((2, 2)), np.eye(2)]),
                    vel,
                    np.eye(2) * 0.2**2,
                )

    # -- queries ------------------------------------------------------------------------------------------------------------------------
    def confirmed(self, t):
        return [tr for tr in self.tr if tr.hits >= self.confirm and self.alive(tr, t)]

    def moving(self, t):
        return [tr for tr in self.confirmed(t) if tr.speed > self.v_min]
