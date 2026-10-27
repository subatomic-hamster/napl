"""Estimators with one interface: predict(dt), gate_d2(zs, meas), lik_each(zs, meas), update(z, meas), update_beta(...),
pv() -> (x6, P6), copy(). EKF (= KF for linear models), UKF, IMM over mixed-dimension modes, bootstrap particle filter.
Covariance updates use the Joseph form (EKF) or P - K S K' symmetrised (UKF)."""

import numpy as np

from .models import CHI2_GATE  # noqa: F401  (re-export for callers)


def _sym(P):
    return 0.5 * (P + P.T)


def _logn(nu, S):
    """log N(nu; 0, S) via Cholesky; also returns the squared Mahalanobis distance."""
    L = np.linalg.cholesky(S)
    y = np.linalg.solve(L, nu.T if nu.ndim > 1 else nu)
    d2 = (y * y).sum(0)
    return -0.5 * (d2 + len(S) * np.log(2 * np.pi)) - np.log(np.diag(L)).sum(), d2


class EKF:
    """Extended Kalman filter (exactly the KF when model and measurement are linear)."""

    def __init__(self, model, x, P):
        self.model, self.x, self.P = (
            model,
            np.asarray(x, float).copy(),
            np.asarray(P, float).copy(),
        )
        self._c = None
        self.last_nis = np.nan

    def copy(self):
        c = object.__new__(type(self))
        c.__dict__.update(self.__dict__)
        c.x, c.P = (
            self.x.copy(),
            self.P.copy(),
        )  # the cached prep tuple is never mutated in place, so it is shared
        return c

    def predict(self, dt):
        F = self.model.F(self.x, dt)
        self.x = self.model.f(self.x, dt)
        self.P = _sym(F @ self.P @ F.T + self.model.Q(self.x, dt))
        self._c = None

    def _prep(self, meas):
        if self._c is not None and self._c[0] is meas:
            return self._c
        H = meas.H(self.x)
        R = meas.R(self.x)
        S = _sym(H @ self.P @ H.T + R)
        K = np.linalg.solve(S, H @ self.P).T
        A = np.eye(len(self.x)) - K @ H
        Pu = _sym(A @ self.P @ A.T + K @ R @ K.T)  # Joseph form
        self._c = (meas, meas.h(self.x), S, K, Pu)
        return self._c

    def nu(self, zs, meas):
        c = self._prep(meas)
        return meas.resid(np.atleast_2d(zs), c[1])

    def gate_d2(self, zs, meas):
        return _logn(self.nu(zs, meas), self._prep(meas)[2])[1]

    def lik_each(self, zs, meas):
        return np.exp(_logn(self.nu(zs, meas), self._prep(meas)[2])[0])

    def update(self, z, meas):
        """Update with one measurement; returns its likelihood N(z; zhat, S)."""
        _, _, S, K, Pu = c = self._prep(meas)
        nu = self.nu(z, meas)[0]
        ll, d2 = _logn(nu, S)
        self.last_nis = float(d2)
        self.x = self.x + K @ nu
        self.P = Pu
        self._c = None
        return float(np.exp(ll))

    def update_beta(self, zs, beta0, betas, meas, lam=1.0):
        """PDA update with association probabilities (beta0 = none, betas per measurement). Returns the mixture likelihood."""
        _, _, S, K, Pu = self._prep(meas)
        nus = self.nu(zs, meas)
        L = np.exp(_logn(nus, S)[0])
        nu = betas @ nus
        spread = (nus * betas[:, None]).T @ nus - np.outer(nu, nu)
        self.x = self.x + K @ nu
        self.P = _sym(beta0 * self.P + (1 - beta0) * Pu + K @ spread @ K.T)
        self._c = None
        return beta0 * lam + float(betas @ L)

    def condition(self, z, meas):
        """Posterior mean for z (covariance is self._prep(meas)[4]); does not change the filter."""
        c = self._prep(meas)
        return self.x + c[3] @ self.nu(z, meas)[0]

    def post_cov(self, meas):
        return self._prep(meas)[4]

    def pv(self):
        return self.x[:6], self.P[:6, :6]


KF = EKF


class UKF(EKF):
    """Unscented Kalman filter (alpha=1, kappa=0, beta=2) through f and h; the same measurement interface."""

    def __init__(self, model, x, P, alpha=1.0, kappa=0.0, beta=2.0):
        super().__init__(model, x, P)
        n = len(self.x)
        lam = alpha**2 * (n + kappa) - n
        self.lam, self.n = lam, n
        self.Wm = np.full(2 * n + 1, 0.5 / (n + lam))
        self.Wc = self.Wm.copy()
        self.Wm[0] = lam / (n + lam)
        self.Wc[0] = lam / (n + lam) + 1 - alpha**2 + beta

    def _sigma(self, x, P):
        n = self.n
        try:
            L = np.linalg.cholesky((n + self.lam) * P)
        except np.linalg.LinAlgError:
            L = np.linalg.cholesky(
                (n + self.lam) * (P + 1e-9 * np.eye(n) * (1 + np.trace(P)))
            )
        return np.vstack([x, x + L.T, x - L.T])

    def predict(self, dt):
        X = self.model.f(self._sigma(self.x, self.P), dt)
        x = self.Wm @ X
        D = X - x
        self.P = _sym((D * self.Wc[:, None]).T @ D + self.model.Q(x, dt))
        self.x = x
        self._c = None

    def _prep(self, meas):
        if self._c is not None and self._c[0] is meas:
            return self._c
        X = self._sigma(self.x, self.P)
        Z = meas.h(X)
        dZ = meas.resid(Z, Z[0])
        off = self.Wm @ dZ
        zhat = Z[0] + off
        dZ = dZ - off
        R = meas.R(self.x)
        S = _sym((dZ * self.Wc[:, None]).T @ dZ + R)
        Pxz = ((X - self.x) * self.Wc[:, None]).T @ dZ
        K = np.linalg.solve(S, Pxz.T).T
        Pu = _sym(self.P - K @ S @ K.T)
        self._c = (meas, zhat, S, K, Pu)
        return self._c


class IMM:
    """Interacting multiple model over modes that may have different state dimensions (CV 6, CA/Singer 9, CT 7).
    Mixing lifts/truncates through the shared [p, v] block; added components get the target model's pad_P variance."""

    def __init__(self, modes, trans, mu0=None):
        self.modes = list(modes)
        self.trans = np.asarray(trans, float)
        m = len(self.modes)
        self.mu = np.full(m, 1.0 / m) if mu0 is None else np.asarray(mu0, float)
        self.c = self.mu @ self.trans
        self._lk = None

    def copy(self):
        o = object.__new__(type(self))
        o.__dict__.update(self.__dict__)
        o.modes = [m.copy() for m in self.modes]
        o.mu, o.c, o._lk = self.mu.copy(), self.c.copy(), None
        return o

    @staticmethod
    def _lift(x, P, src, dst):
        if src.layout == dst.layout:
            return x, P
        xn = np.zeros(dst.dim)
        xn[:6] = x[:6]
        Pn = np.zeros((dst.dim, dst.dim))
        Pn[:6, :6] = P[:6, :6]
        Pn[6:, 6:] = np.diag(dst.pad_P)
        return xn, Pn

    def predict(self, dt):
        M = len(self.modes)
        c = self.mu @ self.trans
        mix = (
            self.trans * self.mu[:, None] / np.maximum(c, 1e-300)[None, :]
        )  # mix[i, j] = P(mode i | next mode j)
        new = []
        for j, mj in enumerate(self.modes):
            lifted = [self._lift(mi.x, mi.P, mi.model, mj.model) for mi in self.modes]
            x0 = sum(mix[i, j] * lifted[i][0] for i in range(M))
            P0 = sum(
                mix[i, j]
                * (lifted[i][1] + np.outer(lifted[i][0] - x0, lifted[i][0] - x0))
                for i in range(M)
            )
            new.append((x0, _sym(P0)))
        for mj, (x0, P0) in zip(self.modes, new):
            mj.x, mj.P = x0, P0
            mj.predict(dt)
        self.c = c
        self._lk = None

    def _each(self, zs, meas):
        if self._lk is None or self._lk[0] is not meas:
            Ls = np.array([m.lik_each(zs, meas) for m in self.modes])  # (modes, nz)
            self._lk = (meas, Ls)
        return self._lk[1]

    def gate_d2(self, zs, meas):
        return np.min([m.gate_d2(zs, meas) for m in self.modes], 0)

    def lik_each(self, zs, meas):
        return self.c @ self._each(zs, meas)

    def _mu(self, Lam):
        mu = self.c * np.maximum(Lam, 1e-300)
        self.mu = mu / mu.sum()

    def update(self, z, meas):
        Lam = np.array([m.update(z, meas) for m in self.modes])
        self._mu(Lam)
        self._lk = None
        return float(self.c @ Lam)

    def update_beta(self, zs, beta0, betas, meas, lam=1.0):
        Lam = np.array([m.update_beta(zs, beta0, betas, meas, lam) for m in self.modes])
        self._mu(Lam)
        self._lk = None
        return float(self.c @ Lam)

    def condition(self, z, meas):
        return self.pv_of(
            [m.condition(z, meas) for m in self.modes],
            [m.post_cov(meas) for m in self.modes],
        )[0]

    def post_cov(self, meas):
        return self.pv_of(
            [m.x for m in self.modes], [m.post_cov(meas) for m in self.modes]
        )[1]

    def pv_of(self, xs, Ps):
        x6 = sum(w * x[:6] for w, x in zip(self.mu, xs))
        P6 = sum(
            w * (P[:6, :6] + np.outer(x[:6] - x6, x[:6] - x6))
            for w, x, P in zip(self.mu, xs, Ps)
        )
        return x6, _sym(P6)

    def pv(self):
        return self.pv_of([m.x for m in self.modes], [m.P for m in self.modes])

    @property
    def x(self):
        return self.pv()[0]

    @property
    def P(self):
        return self.pv()[1]

    @property
    def last_nis(self):
        return self.modes[int(np.argmax(self.mu))].last_nis


class ParticleFilter:
    """Bootstrap particle filter on a CV state [p, v]. Meant for bearings-only / young tracks: init_ray draws a range-
    parameterised cone of particles; hand off to an EKF/IMM with to_gauss() once the cloud is compact."""

    def __init__(self, model, X, rng, q_scale=1.0):
        self.model, self.X, self.rng = model, np.asarray(X, float).copy(), rng
        self.N = len(self.X)
        self.w = np.full(self.N, 1.0 / self.N)
        self._c = None
        self.q_scale = q_scale

    @classmethod
    def init_gauss(cls, model, p0, P0, vsig, N, rng):
        X = np.zeros((N, 6))
        X[:, :3] = rng.multivariate_normal(p0, P0, N)
        X[:, 3:] = rng.normal(size=(N, 3)) * vsig
        return cls(model, X, rng)

    @classmethod
    def init_ray(cls, model, o, d, rmin, rmax, ang_sig, vsig, N, rng):
        """Particles along the ray (o, d), range log-uniform in [rmin, rmax], lateral spread ang_sig * range."""
        r = np.exp(rng.uniform(np.log(rmin), np.log(rmax), N))
        X = np.zeros((N, 6))
        X[:, :3] = o + r[:, None] * d + rng.normal(size=(N, 3)) * (ang_sig * r)[:, None]
        X[:, 3:] = rng.normal(size=(N, 3)) * vsig
        return cls(model, X, rng)

    def copy(self):
        c = object.__new__(type(self))
        c.__dict__.update(self.__dict__)
        c.X, c.w, c._c = self.X.copy(), self.w.copy(), None
        return c

    def predict(self, dt):
        Q = self.model.Q(self.X[0], dt) * self.q_scale
        L = np.linalg.cholesky(Q + 1e-12 * np.eye(6))
        self.X = self.model.f(self.X, dt) + self.rng.normal(size=self.X.shape) @ L.T
        self._c = None

    def pv(self):
        x = self.w @ self.X
        D = self.X - x
        return x, _sym((D * self.w[:, None]).T @ D)

    @property
    def x(self):
        return self.pv()[0]

    @property
    def P(self):
        return self.pv()[1]

    def _prep(self, meas):
        if self._c is not None and self._c[0] is meas:
            return self._c
        Z = meas.h(self.X)
        z0 = Z[np.argmax(self.w)]
        dZ = meas.resid(Z, z0)
        zb = self.w @ dZ
        S = _sym(((dZ - zb) * self.w[:, None]).T @ (dZ - zb) + meas.R(self.pv()[0]))
        self._c = (meas, Z, S)
        return self._c

    def gate_d2(self, zs, meas):
        _, Z, S = self._prep(meas)
        z0 = Z[np.argmax(self.w)]
        zb = z0 + self.w @ meas.resid(Z, z0)
        return _logn(meas.resid(np.atleast_2d(zs), zb), S)[1]

    def lik_each(self, zs, meas):
        _, Z, _ = self._prep(meas)
        R = meas.R(self.pv()[0])
        out = []
        for z in np.atleast_2d(zs):
            ll, _ = _logn(meas.resid(z, Z), R)
            out.append(float(self.w @ np.exp(ll)))
        return np.array(out)

    def update(self, z, meas):
        _, Z, _ = self._prep(meas)
        ll, _ = _logn(meas.resid(z, Z), meas.R(self.pv()[0]))
        lik = self.w @ np.exp(ll)
        w = self.w * np.exp(ll - ll.max())
        self.w = w / w.sum()
        if 1.0 / (self.w**2).sum() < self.N / 2:  # systematic resampling
            u = (self.rng.random() + np.arange(self.N)) / self.N
            idx = np.minimum(np.searchsorted(np.cumsum(self.w), u), self.N - 1)
            self.X, self.w = self.X[idx], np.full(self.N, 1.0 / self.N)
        self._c = None
        return float(lik)

    def spread(self):
        return float(np.sqrt(np.trace(self.pv()[1][:3, :3])))
