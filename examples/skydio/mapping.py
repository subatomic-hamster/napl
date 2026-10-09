"""Depth image -> world point cloud -> rolling 3D occupancy grid -> clearance field. numpy only.

    cam = Camera.from_cfg(device_cfg)                    # intrinsics of the rangefinder device (the D435i part's field of view and range)
    m = VoxelMap(res=0.2)
    m.integrate(depth, cam_pos, cam_R)                   # one depth frame (n_v, n_h) float32, NaN = no return; cam_R: sensor axes in the world
    w = m.window(centre)                                 # Window: occupied / known-free / squared clearance field around `centre`
    w.clearance(points)                                  # distance to the nearest occupied voxel (m, conservative), per point

Sensor frame (the device's): +z = viewing direction, +x = image up, +y = image right; pixel (i, j) = row i from the top, column j from the left sees
the direction (tan(fov_v/2) (1 - 2 (i + .5) / n_v), tan(fov_h/2) (2 (j + .5) / n_h - 1), 1) and its depth is the z coordinate (docs/models.md).

The grid is a torus in x and y (a rolling window of `n_xy` cells around the vehicle: cells that fall out of the window on one side are cleared as
they re-enter on the other) and a plain slab in z. Each cell holds a clamped log-odds count: a depth hit adds HIT, a ray passing through the cell
subtracts FREE. `occupied` = log-odds >= OCC; `known free` = log-odds <= -1: only space a ray has crossed counts as flyable.
The ground plane is not mapped (hits below FLOOR are skipped): the planner keeps its own altitude band instead."""

from dataclasses import dataclass

import numpy as np

HIT, FREE, LO_MIN, LO_MAX, OCC, KNOWN_FREE = 3, 1, -8, 12, 3, -1
FLOOR = 0.25  # m: depth hits below this height are the ground, not mapped
K_CELLS = (
    8  # clearance field radius in cells (exact below this distance, saturated above)
)


@dataclass
class Camera:
    n_h: int
    n_v: int
    fov_h: float
    fov_v: float
    min_range: float
    max_range: float

    @classmethod
    def from_cfg(cls, c):
        return cls(
            int(c["n_h"]),
            int(c["n_v"]),
            float(c["fov_h"]),
            float(c["fov_v"]),
            float(c["min_range"]),
            float(c["max_range"]),
        )

    def tangents(self):
        """(n_v, n_h, 3) un-normalised ray directions (x up, y right, 1): point = depth * tangent."""
        i = np.arange(self.n_v)[:, None] + 0.5
        j = np.arange(self.n_h)[None, :] + 0.5
        x = np.tan(self.fov_v / 2) * (1 - 2 * i / self.n_v) + 0 * j
        y = np.tan(self.fov_h / 2) * (2 * j / self.n_h - 1) + 0 * i
        return np.stack([x, y, np.ones_like(x)], axis=-1)

    def project(self, p_cam):
        """Pixel (row, col) and depth of points in the sensor frame (n, 3); depth <= 0 or outside the image -> NaN rows."""
        z = p_cam[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            x, y = p_cam[:, 0] / z, p_cam[:, 1] / z
        i = (1 - x / np.tan(self.fov_v / 2)) * self.n_v / 2 - 0.5
        j = (y / np.tan(self.fov_h / 2) + 1) * self.n_h / 2 - 0.5
        bad = (
            (z <= 0)
            | (i < -0.5)
            | (i > self.n_v - 0.5)
            | (j < -0.5)
            | (j > self.n_h - 0.5)
        )
        return np.where(bad, np.nan, i), np.where(bad, np.nan, j), z


class VoxelMap:
    def __init__(
        self,
        res=0.2,
        n_xy=160,
        n_z=60,
        z0=-0.4,
        max_hit_range=9.5,
        carve_stride=2,
        hit_hold=12,
    ):
        self.res, self.n, self.z0 = res, (n_xy, n_xy, n_z), z0
        self.L = np.zeros(self.n, np.int8)
        self.hit_hold = hit_hold  # frames
        self.hit_frame = np.full(
            self.n, -(10**9), np.int32
        )  # frame index of the last depth hit per voxel
        self.o = np.array(
            [-n_xy // 2, -n_xy // 2]
        )  # absolute cell index of the window's lowest x / y cell
        self.max_hit_range, self.stride = max_hit_range, carve_stride
        self._steps = None
        self.frames = 0

    # ---- window bookkeeping ---------------------------------------------------------------------------------------------------------
    def recentre(self, p):
        """Slide the window so the vehicle stays near its middle (moves in steps of 8 cells; cells entering the window start unknown)."""
        c = np.floor(np.asarray(p[:2]) / self.res).astype(int)
        for ax in (0, 1):
            new = self.o[ax] + 8 * int(
                np.round((c[ax] - self.o[ax] - self.n[ax] // 2) / 8)
            )
            if new == self.o[ax]:
                continue
            old, n = self.o[ax], self.n[ax]
            if abs(new - old) >= n:
                self.L[:] = 0
                self.hit_frame[:] = -(10**9)
            else:
                enter = (
                    np.arange(old + n, new + n) if new > old else np.arange(new, old)
                )
                if ax == 0:
                    self.L[enter % n, :, :] = 0
                    self.hit_frame[enter % n, :, :] = -(10**9)
                else:
                    self.L[:, enter % n, :] = 0
                    self.hit_frame[:, enter % n, :] = -(10**9)
            self.o[ax] = new

    def _cells(self, pts):
        """World points (n, 3) -> (ix, iy, iz) storage indices and a validity mask (inside the window and the z slab)."""
        c = np.floor(pts / self.res).astype(np.int64)
        iz = np.floor((pts[:, 2] - self.z0) / self.res).astype(np.int64)
        ok = (
            (c[:, 0] >= self.o[0])
            & (c[:, 0] < self.o[0] + self.n[0])
            & (c[:, 1] >= self.o[1])
            & (c[:, 1] < self.o[1] + self.n[1])
            & (iz >= 0)
            & (iz < self.n[2])
        )
        return c[:, 0] % self.n[0], c[:, 1] % self.n[1], iz, ok

    # ---- integration ------------------------------------------------------------------------------------------------------------------
    def integrate(self, depth, cam: Camera, pos, R, ignore=None):
        """One depth frame. `pos`, `R`: the sensor origin and axes in the world; `ignore`: bool (n_v, n_h) pixels to leave out (e.g. the tracked subject)."""
        self.recentre(pos)
        self.frames += 1
        tang = cam.tangents()
        valid = (
            np.isfinite(depth)
            & (depth >= cam.min_range)
            & (depth <= self.max_hit_range)
        )
        miss = ~np.isfinite(
            depth
        )  # no return within max_range (or inside min_range, which the airframe's own margin keeps us from)
        hit = valid
        if (
            ignore is not None
        ):  # the tracked subject: no hits, and its rays prove free only the space 1.3 m short of it (a person is not a wall, and
            hit = (
                valid & ~ignore
            )  # the corridor towards it must not stay "unknown", or the follower could never close in on the subject)
            miss &= ~ignore
        # free space: every `stride`-th pixel, sampled every 0.6 cell along the ray from 0.5 m to the hit (minus a margin) / to 7 m for a miss
        s = self.stride
        margin = 0.4 if ignore is None else np.where(ignore, 1.3, 0.4)
        d = np.where(miss, self.max_hit_range - 0.5, depth - margin)[::s, ::s]
        use = (valid | miss)[::s, ::s] & (d > 0.6)
        if use.any():
            dirs = (
                tang[::s, ::s][use] @ R.T
            )  # (n, 3) world directions scaled so that |z-component along the optical axis| = 1
            dd = d[use]
            ts = np.arange(0.5, dd.max(), self.res * 0.6)
            pts = (
                pos + dirs[:, None, :] * ts[None, :, None]
            )  # (n, k, 3): the sample at ray parameter t has optical-axis depth t
            keep = ts[None, :] <= dd[:, None]
            ix, iy, iz, ok = self._cells(pts[keep])
            ok &= pts[keep][:, 2] > FLOOR * 0.5
            # a voxel hit within the last `hit_hold` frames is not carved: a thin pole (narrower than a 0.2 m voxel) is hit by one or two pixels while
            # the neighbouring rays, which hit the background, cross the same voxel and would otherwise erase it again
            inside = np.nonzero(ok)[0]
            ok[inside] &= (
                self.frames - self.hit_frame[ix[inside], iy[inside], iz[inside]]
            ) > self.hit_hold
            self.L[ix[ok], iy[ok], iz[ok]] = np.maximum(
                self.L[ix[ok], iy[ok], iz[ok]] - FREE, LO_MIN
            )
        # hits
        pw = pos + (tang[hit] * depth[hit][:, None]) @ R.T
        pw = pw[pw[:, 2] > FLOOR]
        ix, iy, iz, ok = self._cells(pw)
        self.L[ix[ok], iy[ok], iz[ok]] = np.minimum(
            self.L[ix[ok], iy[ok], iz[ok]] + HIT, LO_MAX
        )
        self.hit_frame[ix[ok], iy[ok], iz[ok]] = self.frames

    @property
    def n_occupied(self):
        return int((self.L >= OCC).sum())

    def occupied_points(self, centre, radius, limit=None):
        """World centres of occupied cells within `radius` m (horizontal) of `centre` (telemetry / visualisation)."""
        ix, iy, iz = np.nonzero(self.L >= OCC)
        ax = self.o[0] + (ix - self.o[0]) % self.n[0]
        ay = self.o[1] + (iy - self.o[1]) % self.n[1]
        pts = np.stack(
            [
                (ax + 0.5) * self.res,
                (ay + 0.5) * self.res,
                self.z0 + (iz + 0.5) * self.res,
            ],
            1,
        )
        pts = pts[np.hypot(pts[:, 0] - centre[0], pts[:, 1] - centre[1]) <= radius]
        return (
            pts
            if limit is None or len(pts) <= limit
            else pts[:: int(np.ceil(len(pts) / limit))]
        )

    # ---- the planner's view ----------------------------------------------------------------------------------------------------------
    def window(self, centre, half=8.0, z_lo=0.4, z_hi=5.0, extra=()):
        """Occupancy / known-free / clearance field over a box of +-`half` m around `centre` (horizontally) and [z_lo, z_hi]. `extra`: world points
        to count as occupied on top of the map (the tracked subject)."""
        r = self.res
        w = int(round(2 * half / r))
        lo = np.floor(np.asarray(centre[:2]) / r).astype(int) - w // 2
        lo = np.clip(
            lo, self.o, self.o + np.array(self.n[:2]) - w
        )  # stay inside the rolling window
        iz0 = int(np.floor((z_lo - self.z0) / r))
        nz = int(round((z_hi - z_lo) / r))
        ix = (lo[0] + np.arange(w)) % self.n[0]
        iy = (lo[1] + np.arange(w)) % self.n[1]
        sub = self.L[np.ix_(ix, iy, np.arange(iz0, iz0 + nz))]
        occ = sub >= OCC
        origin = np.array([lo[0] * r, lo[1] * r, self.z0 + iz0 * r])
        win = Window(origin, r, occ, sub <= KNOWN_FREE)
        for p in extra:
            win.add_sphere(p[:3], p[3])
        win.finish()
        return win


class Window:
    """A box of cells: `occ`, `known` (bool), `g` = squared distance in cells to the nearest occupied cell (saturated at K_CELLS^2 + 1)."""

    def __init__(self, origin, res, occ, known):
        self.origin, self.res, self.occ, self.known = origin, res, occ, known
        self.shape = np.array(occ.shape)
        self.g = None

    def add_sphere(self, c, radius):
        k = int(np.ceil(radius / self.res)) + 1
        ci = np.floor((np.asarray(c) - self.origin) / self.res).astype(int)
        lo, hi = np.maximum(ci - k, 0), np.minimum(ci + k + 1, self.shape)
        if (hi <= lo).any():
            return
        g = np.stack(
            np.meshgrid(*[np.arange(a, b) for a, b in zip(lo, hi)], indexing="ij"), -1
        )
        inside = (
            np.linalg.norm((g + 0.5) * self.res + self.origin - c, axis=-1) <= radius
        )
        self.occ[lo[0] : hi[0], lo[1] : hi[1], lo[2] : hi[2]] |= inside

    def finish(self):
        inf = 3 * K_CELLS**2 + 10
        g = np.where(self.occ, 0, inf).astype(np.int16)
        for ax in range(
            3
        ):  # separable squared Euclidean distance, exact up to K_CELLS cells
            out = g.copy()
            for k in range(1, K_CELLS + 1):
                a = [slice(None)] * 3
                b = [slice(None)] * 3
                a[ax], b[ax] = slice(k, None), slice(None, -k)
                out[tuple(a)] = np.minimum(out[tuple(a)], g[tuple(b)] + k * k)
                out[tuple(b)] = np.minimum(out[tuple(b)], g[tuple(a)] + k * k)
            g = out
        self.g = g

    def index(self, pts):
        """(n, 3) world points -> integer cell index and inside-the-box mask."""
        i = np.floor((pts - self.origin) / self.res).astype(int)
        ok = ((i >= 0) & (i < self.shape)).all(axis=1)
        return np.clip(i, 0, self.shape - 1), ok

    def clearance(self, pts):
        """Conservative distance (m) from points to the nearest occupied cell's surface: centre distance minus 0.9 cell (a cell holds its surface
        anywhere inside it); saturates at K_CELLS cells. Points outside the box read as occupied (-1)."""
        i, ok = self.index(pts)
        d = np.sqrt(self.g[i[:, 0], i[:, 1], i[:, 2]].astype(float))
        return np.where(
            ok, np.minimum(d, K_CELLS + 1) * self.res - 0.9 * self.res, -1.0
        )

    def is_known_free(self, pts):
        i, ok = self.index(pts)
        return ok & self.known[i[:, 0], i[:, 1], i[:, 2]]
