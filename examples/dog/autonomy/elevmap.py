"""Global elevation map, 5 cm cells, height + variance (Fankhauser et al. 2014, "Robot-centric elevation mapping with
uncertainty estimates", fusion rule; kept in the fixed map frame here because RTK bounds the drift). Error budget at 4 m on the
carrier (docs/dog/autonomy.md): RTK vertical ~2 cm (correlated part), attitude 0.3 deg ~2.1 cm, angular precision ~1.0 cm,
range ~0.5 cm, discretisation ~0.8 cm -> ~3.2 cm in quadrature against the 5 cm target."""

import numpy as np


class ElevationMap:
    def __init__(self, cfg):
        M, L = cfg["map"], cfg["lidar"]
        self.res, self.x0 = M["res"], -M["size"] / 2
        self.n = int(round(M["size"] / self.res))
        self.h = np.zeros((self.n, self.n), np.float32)
        self.v = np.full((self.n, self.n), np.inf, np.float32)
        self.hits = np.zeros((self.n, self.n), np.uint32)
        self.hn = np.full(
            (self.n, self.n), np.nan, np.float32
        )  # h with nan where unseen (what traverse reads)
        self.gate2, self.floor, self.vmap, self.maxr = (
            M["gate"] ** 2,
            M["var_floor"],
            M["var_mapped"],
            M["max_range"],
        )
        self.sr2, self.sa2 = L["sigma0"] ** 2, L["ang_sigma"] ** 2
        self.version, self.dirty = 0, None

    def cells(self, x, y):
        c = np.floor((np.asarray(x, float) - self.x0) / self.res).astype(np.int64)
        r = np.floor((np.asarray(y, float) - self.x0) / self.res).astype(np.int64)
        inside = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        return np.clip(r, 0, self.n - 1), np.clip(c, 0, self.n - 1), inside

    def fuse(self, pts, origin, var_pz, att_sigma):
        d = pts - origin
        rh2 = d[:, 0] ** 2 + d[:, 1] ** 2
        r2 = rh2 + d[:, 2] ** 2
        ok = (r2 > 0) & (r2 <= self.maxr**2)
        vp = (
            self.sr2 * d[:, 2] ** 2 / np.where(ok, r2, 1.0)
            + (self.sa2 + att_sigma**2) * rh2
        )
        self.fuse_z(pts[ok, 0], pts[ok, 1], pts[ok, 2], vp[ok], var_pz)

    def fuse_z(self, x, y, z, var_point, var_common):
        r, c, inside = self.cells(x, y)
        if not inside.any():
            self.dirty = None
            return
        idx = (r * self.n + c)[inside]
        u, inv = np.unique(idx, return_inverse=True)
        w = 1.0 / np.maximum(var_point[inside], self.floor)
        W = np.bincount(inv, w)
        Z = np.bincount(inv, w * z[inside]) / W
        V = 1.0 / W + var_common
        hf, vf, nf = self.h.reshape(-1), self.v.reshape(-1), self.hits.reshape(-1)
        h0, v0 = hf[u].astype(np.float64), vf[u].astype(np.float64)
        new = ~np.isfinite(v0)
        s = v0 + V
        dz = Z - h0
        far = ~new & (dz * dz > self.gate2 * s)
        take = new | (far & (dz > 0))
        keep = ~(far & (dz < 0))
        with np.errstate(invalid="ignore"):
            hn = np.where(take, Z, (V * h0 + v0 * Z) / s)
            vn = np.where(take, V, v0 * V / s)
        k = u[keep]
        hf[k], vf[k] = hn[keep], np.maximum(vn[keep], self.floor)
        self.hn.reshape(-1)[k] = hn[keep]
        nf[k] += 1
        rr, cc = np.divmod(u, self.n)
        self.dirty = (
            int(rr.min()),
            int(rr.max()) + 1,
            int(cc.min()),
            int(cc.max()) + 1,
        )
        self.version += 1

    def height(self, x, y):
        r, c, inside = self.cells(x, y)
        v = self.v[r, c]
        out = np.where(inside & np.isfinite(v), self.h[r, c].astype(np.float64), np.nan)
        return float(out) if np.ndim(out) == 0 else out

    def var(self, x, y):
        r, c, inside = self.cells(x, y)
        out = np.where(inside, self.v[r, c].astype(np.float64), np.inf)
        return float(out) if np.ndim(out) == 0 else out

    def mapped(self, pool):
        m = (self.n // pool) * pool
        info = np.where(
            np.isfinite(self.v[:m, :m]), 1.0 / self.v[:m, :m].astype(np.float64), 0.0
        )
        info = info.reshape(m // pool, pool, m // pool, pool).sum(axis=(1, 3))
        return info > 1.0 / self.vmap
