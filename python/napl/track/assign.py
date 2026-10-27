"""Linear assignment: scipy.optimize.linear_sum_assignment when scipy is importable (it is not a napl dependency), else a
pure-numpy shortest-augmenting-path Hungarian (O(n^2 m), exact, rectangular)."""

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment as _scipy_lsa
except ImportError:  # scipy is optional
    _scipy_lsa = None


def _hungarian(cost):
    """Min-cost assignment of every row of an n x m matrix, n <= m (Jonker-Volgenant style potentials)."""
    n, m = cost.shape
    u, v = np.zeros(n + 1), np.zeros(m + 1)
    p = np.zeros(m + 1, dtype=int)  # p[j] = row matched to column j (1-based, 0 = free)
    way = np.zeros(m + 1, dtype=int)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(m + 1, np.inf)
        used = np.zeros(m + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            cur = cost[i0 - 1] - u[i0] - v[1:]
            free = ~used[1:]
            upd = free & (cur < minv[1:])
            minv[1:][upd] = cur[upd]
            way[1:][upd] = j0
            cand = np.where(free, minv[1:], np.inf)
            j1 = int(np.argmin(cand)) + 1
            delta = cand[j1 - 1]
            u[p[used]] += delta
            v[used] -= delta
            minv[1:][free] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    rows = np.empty(n, dtype=int)
    for j in range(1, m + 1):
        if p[j]:
            rows[p[j] - 1] = j - 1
    return np.arange(n), rows


def lsa(cost, force_numpy=False):
    """(row_ind, col_ind) of the min-cost assignment of min(n, m) pairs; cost must be finite (use a big number for forbidden)."""
    cost = np.asarray(cost, float)
    if cost.size == 0:
        return np.zeros(0, int), np.zeros(0, int)
    if _scipy_lsa is not None and not force_numpy:
        return _scipy_lsa(cost)
    if cost.shape[0] <= cost.shape[1]:
        return _hungarian(cost)
    c, r = _hungarian(cost.T)
    o = np.argsort(r)
    return r[o], c[o]
