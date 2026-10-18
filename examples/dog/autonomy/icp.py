"""Scan-to-submap point-to-plane ICP, 4 DOF (x, y, z, yaw; roll / pitch from odometry). Submap: the last `keep` keyframes
voxelised at `voxel` m, a mean and normal per planar voxel; association = the nearest voxel mean among the point's voxel and its
6 face neighbours (int64 keys, searchsorted: no KD-tree, no scipy). Gauss-Newton with Huber weights; directions whose Hessian
eigenvalue is below degen x the largest are not updated and get a huge covariance (a flat floor fixes only z)."""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import lidar
from .localize import _qz, rz, wrap

BIG = 1e4  # m^2 / rad^2: "no information" along a degenerate direction
NEIGH = np.array(
    [[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]],
    dtype=np.int64,
)
MAX_PTS = 2000  # scan points used per match


def levelled(scan, ext, odom_quat, ang_sigma=0.0, rng=None):
    """Scan points in the yaw-free base frame (base frame rotated by the odometry's roll / pitch); world = Rz(yaw) p + x."""
    q_rp = lidar.qmul(_qz(-lidar.yaw_of(odom_quat)), odom_quat)
    return lidar.points(scan, ext[0], ext[1], ang_sigma, rng) @ lidar.qmat(q_rp).T


def _keys(ijk):
    ijk = ijk + (1 << 20)
    return (ijk[..., 0] << 42) | (ijk[..., 1] << 21) | ijk[..., 2]


@dataclass
class Match:
    x: np.ndarray
    cov: np.ndarray
    inliers: int
    degenerate: np.ndarray


class ScanMatcher:
    def __init__(self, cfg):
        self.I = cfg["icp"]
        self.vox = self.I["voxel"]
        self.kf = deque(maxlen=int(self.I["keep"]))
        self.last_kf = None
        self.keys = np.empty(0, np.int64)

    def add(self, world_pts, x):
        if self.last_kf is not None:
            d = math.hypot(x[0] - self.last_kf[0], x[1] - self.last_kf[1])
            if (
                d < self.I["kf_dist"]
                and abs(wrap(x[3] - self.last_kf[3])) < self.I["kf_yaw"]
            ):
                return False
        self.kf.append(np.asarray(world_pts, dtype=np.float64))
        self.last_kf = np.array(x, dtype=np.float64)
        P = np.concatenate(self.kf)
        k = _keys(np.floor(P / self.vox).astype(np.int64))
        o = np.argsort(k, kind="stable")
        k, P = k[o], P[o]
        st = np.flatnonzero(np.r_[True, k[1:] != k[:-1]])
        cnt = np.diff(np.r_[st, len(k)]).astype(np.float64)
        mu = np.add.reduceat(P, st, axis=0) / cnt[:, None]
        C = (
            np.add.reduceat(P[:, :, None] * P[:, None, :], st, axis=0)
            / cnt[:, None, None]
            - mu[:, :, None] * mu[:, None, :]
        )
        w, V = np.linalg.eigh(C)  # ascending
        ok = (cnt >= self.I["min_pts"]) & (w[:, 0] < self.I["planarity"] * w[:, 1])
        self.keys, self.mu, self.nrm = k[st][ok], mu[ok], V[ok][:, :, 0]
        return True

    def _assoc(self, w):
        base = np.floor(w / self.vox).astype(np.int64)
        best_i = np.full(len(w), -1)
        best_d = np.full(len(w), np.inf)
        for off in NEIGH:
            k = _keys(base + off)
            i = np.minimum(np.searchsorted(self.keys, k), len(self.keys) - 1)
            hit = self.keys[i] == k
            d = np.where(hit, ((w - self.mu[i]) ** 2).sum(1), np.inf)
            better = d < best_d
            best_i, best_d = np.where(better, i, best_i), np.where(better, d, best_d)
        return best_i

    def match(self, s, x_guess):
        if len(self.keys) == 0:
            return None
        s = s[:: max(1, len(s) // MAX_PTS)]
        x = np.array(x_guess, dtype=np.float64)
        hub, sig2 = self.I["huber"], self.I["sigma"] ** 2
        for _ in range(int(self.I["iters"])):
            ws = s @ rz(x[3]).T
            i = self._assoc(ws + x[:3])
            hit = i >= 0
            if hit.sum() < 50:
                return None
            n, mu, wsh = self.nrm[i[hit]], self.mu[i[hit]], ws[hit]
            e = ((wsh + x[:3] - mu) * n).sum(1)
            J = np.column_stack([n, -n[:, 0] * wsh[:, 1] + n[:, 1] * wsh[:, 0]])
            a = np.abs(e)
            wt = np.where(a <= hub, 1.0, hub / np.maximum(a, 1e-12))
            H = (J * wt[:, None]).T @ J
            g = (J * wt[:, None]).T @ e
            lam, U = np.linalg.eigh(H)
            good = lam > self.I["degen"] * lam.max()
            dx = -U[:, good] @ ((U[:, good].T @ g) / lam[good])
            x[:4] += dx
            x[3] = wrap(x[3])
            if np.linalg.norm(dx) < 1e-5:
                break
        cov = (
            U[:, good] @ np.diag(sig2 / lam[good]) @ U[:, good].T
            + U[:, ~good] @ np.diag(np.full((~good).sum(), BIG)) @ U[:, ~good].T
        )
        cov = cov + np.diag(
            [self.I["floor_xyz"] ** 2] * 3 + [self.I["floor_yaw"] ** 2]
        )  # formal sigma is far too tight on rubble
        return Match(x, cov, int(hit.sum()), np.diag(cov) > 1.0)
