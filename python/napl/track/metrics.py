"""Metrics: GOSPA (alpha=2) with its decomposition, OSPA, NEES / NIS, and a per-run Evaluator (track breaks, ID switches,
time to confirm, ANEES) that matches truth to confirmed tracks frame by frame."""

import numpy as np

from .assign import lsa


def _dist(X, Y):
    X, Y = np.atleast_2d(np.asarray(X, float)), np.atleast_2d(np.asarray(Y, float))
    return np.linalg.norm(X[:, None, :] - Y[None, :, :], axis=-1)


def gospa(X, Y, c=2.0, p=2):
    """GOSPA with alpha = 2 (Rahmathullah et al. 2017) between truth X (n, d) and estimates Y (m, d).
    gospa^p = sum_assigned d^p + (c^p / 2) (#missed + #false). Returns dict: total, loc, missed, false, each as a p-th root so that
    total^p = loc^p + missed^p + false^p; n_match counts assigned pairs with d < c."""
    n, m = len(X), len(Y)
    if n == 0 or m == 0:
        return dict(
            total=(0.5 * c**p * (n + m)) ** (1 / p),
            loc=0.0,
            missed=(0.5 * c**p * n) ** (1 / p),
            false=(0.5 * c**p * m) ** (1 / p),
            n_match=0,
        )
    D = _dist(X, Y)
    r, k = lsa(np.minimum(D, c) ** p)
    ok = D[r, k] < c
    loc = float((D[r, k][ok] ** p).sum())
    nm = int(ok.sum())
    miss, false = 0.5 * c**p * (n - nm), 0.5 * c**p * (m - nm)
    return dict(
        total=(loc + miss + false) ** (1 / p),
        loc=loc ** (1 / p),
        missed=miss ** (1 / p),
        false=false ** (1 / p),
        n_match=nm,
    )


def ospa(X, Y, c=2.0, p=2):
    n, k = len(X), len(Y)
    m = max(n, k)
    if m == 0:
        return 0.0
    if min(n, k) == 0:
        return c
    D = np.minimum(_dist(X, Y), c)
    r, q = lsa(D**p)
    return float(
        ((D[r, q] ** p).sum() + c**p * (m - min(n, k))) ** (1 / p) / m ** (1 / p)
    )


def nees(x_true, x_est, P):
    e = np.asarray(x_true, float) - np.asarray(x_est, float)
    return float(e @ np.linalg.solve(P, e))


def nis(nu, S):
    nu = np.asarray(nu, float)
    return float(nu @ np.linalg.solve(S, nu))


def chi2_band(dof, n_samples, z=3.0):
    """Normal approximation of the acceptance interval of the mean of n_samples chi-square(dof) variables (+-z sigma)."""
    s = z * np.sqrt(2.0 * dof / n_samples)
    return dof - s, dof + s


class Evaluator:
    """add(t, truth_ids, truth6 (n, 6), tracks) per frame, tracks = confirmed Track list; summary() at the end."""

    def __init__(self, c=2.0, p=2, nees_burn=10):
        self.c, self.p, self.burn = c, p, nees_burn
        self.rows, self.nees, self.dt_conf = [], [], {}
        self.last_id, self.was, self.t0 = {}, {}, None
        self.sw = self.brk = 0
        self.run = {}

    def add(self, t, ids, truth6, tracks):
        truth6 = np.asarray(truth6, float).reshape(len(ids), 6)
        est = np.array([tr.mean for tr in tracks]).reshape(len(tracks), 6)
        if self.t0 is None:
            self.t0 = t
        g = gospa(truth6[:, :3], est[:, :3], self.c, self.p)
        match = {}
        if len(ids) and len(tracks):
            D = _dist(truth6[:, :3], est[:, :3])
            for i, j in zip(*lsa(np.minimum(D, self.c))):
                if D[i, j] < self.c:
                    match[ids[i]] = j
        for i, tid in enumerate(ids):
            j = match.get(tid)
            if j is not None:
                trid = tracks[j].id
                if tid in self.last_id and self.last_id[tid] != trid:
                    self.sw += 1
                self.last_id[tid] = trid
                self.dt_conf.setdefault(tid, t - self.t0)
                self.run[(tid, trid)] = self.run.get((tid, trid), 0) + 1
                if self.run[(tid, trid)] > self.burn:
                    self.nees.append(nees(truth6[i], tracks[j].mean, tracks[j].cov))
            elif self.was.get(tid):
                self.brk += 1
            self.was[tid] = j is not None
        self.rows.append(
            (
                g["total"] ** self.p,
                g["loc"] ** self.p,
                g["missed"] ** self.p,
                g["false"] ** self.p,
                ospa(truth6[:, :3], est[:, :3], self.c, self.p),
                len(tracks),
            )
        )

    def summary(self, truth_ids):
        a = np.array(self.rows)
        ttc = [self.dt_conf.get(i, np.nan) for i in truth_ids]
        return dict(
            gospa=float(np.mean(a[:, 0] ** (1 / self.p))),
            loc=float(np.sqrt(a[:, 1].mean())),
            missed=float(np.sqrt(a[:, 2].mean())),
            false=float(np.sqrt(a[:, 3].mean())),
            ospa=float(a[:, 4].mean()),
            nees=float(np.mean(self.nees)) if self.nees else float("nan"),
            n_nees=len(self.nees),
            breaks=self.brk,
            switches=self.sw,
            ttc=float(np.nanmean(ttc)) if not np.all(np.isnan(ttc)) else float("nan"),
            lost=int(np.sum(np.isnan(ttc))),
        )
