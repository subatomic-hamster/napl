"""Motion and measurement models (plain numpy, vectorised over a leading batch axis).

State layouts, world frame x forward, y left, z up (metres, seconds):
  CV      [p(3), v(3)]            Q from white-noise acceleration, PSD `q` (m^2/s^3)
  CA      [p(3), v(3), a(3)]      Q from white-noise jerk, PSD `q`
  Singer  [p(3), v(3), a(3)]      a decays with time constant `tau`, stationary accel std `sigma_a`
  CT      [p(3), v(3), w]         nearly-constant turn in the horizontal plane, w = turn rate (rad/s), random-walk PSD `q_w`
All layouts start with [p, v], so any track can be reduced to a common 6-d state (Track.mean).
"""

import numpy as np

CHI2_GATE = {
    1: 10.83,
    2: 13.82,
    3: 16.27,
    4: 18.47,
    5: 20.52,
    6: 22.46,
}  # chi-square 0.999 quantiles by dof

_GL_X, _GL_W = np.polynomial.legendre.leggauss(6)


def num_jac(f, x, eps=1e-6):
    """Central-difference Jacobian of a batch-vectorised f: (n, d) -> (n, k) at one point x (d,). Returns (k, d)."""
    d = len(x)
    h = eps * (1.0 + np.abs(x))
    E = np.eye(d) * h
    fp, fm = f(x + E), f(x - E)
    return ((fp - fm) / (2 * h)[:, None]).T


class MotionModel:
    dim = 6
    layout = "cv"
    name = "model"
    pad_P = np.zeros(
        0
    )  # prior variance of components added when a lower-dim mode state is lifted into this model (IMM)

    def f(self, X, dt):
        raise NotImplementedError

    def F(self, x, dt):
        return num_jac(lambda X: self.f(X, dt), np.asarray(x, float))

    def Q(self, x, dt):
        raise NotImplementedError


class _Linear(MotionModel):
    """kron(A(dt), I3) dynamics driven by white noise of PSD q entering through b on the highest derivative."""

    def __init__(self, q):
        self.q = float(q)
        self._cache = {}

    def A(self, dt):
        raise NotImplementedError

    def _get(self, dt):
        key = round(float(dt), 9)
        c = self._cache.get(key)
        if c is None:
            Phi = np.kron(self.A(dt), np.eye(3))
            Qk = (
                sum(
                    w
                    * dt
                    / 2
                    * np.outer(
                        self.A(dt / 2 * (x + 1)) @ self.b,
                        self.A(dt / 2 * (x + 1)) @ self.b,
                    )
                    for x, w in zip(_GL_X, _GL_W)
                )
                * self.q
            )  # int_0^dt A(s) b q b' A(s)' ds
            c = self._cache[key] = (Phi, np.kron(Qk, np.eye(3)))
        return c

    def f(self, X, dt):
        return np.asarray(X) @ self._get(dt)[0].T

    def F(self, x, dt):
        return self._get(dt)[0]

    def Q(self, x, dt):
        return self._get(dt)[1]


class CV(_Linear):
    dim, layout, name = 6, "cv", "cv"
    b = np.array([0.0, 1.0])

    def __init__(self, q=0.5):
        super().__init__(q)

    def A(self, dt):
        return np.array([[1, dt], [0, 1.0]])


class CA(_Linear):
    dim, layout, name = 9, "ca", "ca"
    b = np.array([0.0, 0.0, 1.0])

    def __init__(self, q=2.0, pad_var=1.0):
        super().__init__(q)
        self.pad_P = np.full(3, pad_var)

    def A(self, dt):
        return np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1.0]])


class Singer(_Linear):
    """Singer: acceleration is a zero-mean first-order Markov process, stationary std sigma_a, correlation time tau."""

    dim, layout, name = 9, "ca", "singer"
    b = np.array([0.0, 0.0, 1.0])

    def __init__(self, sigma_a=3.0, tau=3.0):
        al = 1.0 / tau
        super().__init__(2 * al * sigma_a**2)
        self.al = al
        self.pad_P = np.full(3, sigma_a**2)

    def A(self, dt):
        al = self.al
        e = np.exp(-al * dt)
        return np.array(
            [[1, dt, (al * dt - 1 + e) / al**2], [0, 1, (1 - e) / al], [0, 0, e]]
        )


class CT(MotionModel):
    """Coordinated turn: horizontal velocity rotates at rate w (state 6), vertical motion is constant velocity."""

    dim, layout, name = 7, "ct", "ct"

    def __init__(self, q=0.5, q_w=0.05, pad_var=0.2**2):
        self.cv = CV(q)
        self.q_w = q_w
        self.pad_P = np.array([pad_var])

    def f(self, X, dt):
        X = np.asarray(X, float)
        w = X[..., 6]
        wt = w * dt
        small = np.abs(wt) < 1e-6
        ws = np.where(small, 1.0, w)
        s = np.where(small, dt * (1 - wt**2 / 6), np.sin(wt) / ws)
        c = np.where(small, 0.5 * w * dt * dt, (1 - np.cos(wt)) / ws)
        co, si = np.cos(wt), np.sin(wt)
        Y = np.empty_like(X)
        vx, vy = X[..., 3], X[..., 4]
        Y[..., 0] = X[..., 0] + s * vx - c * vy
        Y[..., 1] = X[..., 1] + c * vx + s * vy
        Y[..., 2] = X[..., 2] + X[..., 5] * dt
        Y[..., 3] = co * vx - si * vy
        Y[..., 4] = si * vx + co * vy
        Y[..., 5] = X[..., 5]
        Y[..., 6] = w
        return Y

    def Q(self, x, dt):
        Q = np.zeros((7, 7))
        Q[:6, :6] = self.cv.Q(x, dt)
        Q[6, 6] = self.q_w * dt
        return Q


# ---------------------------------------------------------------------------------------------- measurement models


class Meas:
    """Measurement model. h works on the position part X[..., :3] of any state. R may depend on the (predicted) state."""

    dim = 3
    linear = False

    def h(self, X):
        raise NotImplementedError

    def R(self, x):
        raise NotImplementedError

    def resid(self, z, zhat):
        return z - zhat

    def H(self, x):
        x = np.asarray(x, float)
        Hp = num_jac(lambda P: self.h(P), x[:3])
        out = np.zeros((self.dim, len(x)))
        out[:, :3] = Hp
        return out

    def init(self, z):
        """(p0 (3,), P0 (3,3)): position prior from one measurement."""
        raise NotImplementedError


class PositionModel(Meas):
    """Direct (x, y, z) measurement with std sigma (scalar or length 3)."""

    linear = True

    def __init__(self, sigma):
        self.sig = np.broadcast_to(np.asarray(sigma, float), (3,)).copy()
        self._R = np.diag(self.sig**2)

    def h(self, X):
        return np.asarray(X)[..., :3]

    def H(self, x):
        out = np.zeros((3, len(x)))
        out[:, :3] = np.eye(3)
        return out

    def R(self, x):
        return self._R

    def init(self, z):
        return np.asarray(z, float), self._R.copy()

    def sample(self, rng, p):
        return p + rng.normal(size=3) * self.sig


class RangeModel(Meas):
    """Scalar range from a sensor at `origin` (boresight rangefinder / radar range); sigma = sigma0 + rel * r."""

    dim = 1

    def __init__(self, origin=(0, 0, 0), sigma=0.1, rel=0.0):
        self.o, self.s0, self.rel = np.asarray(origin, float), sigma, rel

    def h(self, X):
        return np.linalg.norm(np.asarray(X)[..., :3] - self.o, axis=-1)[..., None]

    def R(self, x):
        r = np.linalg.norm(x[:3] - self.o)
        return np.array([[(self.s0 + self.rel * r) ** 2]])

    def sample(self, rng, p):
        r = np.linalg.norm(p - self.o)
        return np.array([r + rng.normal() * (self.s0 + self.rel * r)])


class BearingModel(Meas):
    """(azimuth, elevation) from a fixed sensor at `origin`; two of them at different origins are a stereo pair."""

    dim = 2

    def __init__(self, origin=(0, 0, 0), sig_az=1e-3, sig_el=1e-3):
        self.o = np.asarray(origin, float)
        self._R = np.diag([sig_az**2, sig_el**2])
        self.sig = np.array([sig_az, sig_el])

    def h(self, X):
        d = np.asarray(X)[..., :3] - self.o
        return np.stack(
            [
                np.arctan2(d[..., 1], d[..., 0]),
                np.arctan2(d[..., 2], np.hypot(d[..., 0], d[..., 1])),
            ],
            -1,
        )

    def R(self, x):
        return self._R

    def resid(self, z, zhat):
        e = z - zhat
        e[..., 0] = (e[..., 0] + np.pi) % (2 * np.pi) - np.pi
        return e

    def ray(self, z):
        az, el = z[0], z[1]
        return self.o, np.array(
            [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]
        )

    def init(self, z, r0=20.0, sig_r=10.0):
        o, d = self.ray(z)
        return _ray_prior(o, d, r0, sig_r, self.sig.max())

    def sample(self, rng, p):
        return self.h(p) + rng.normal(size=2) * self.sig


def _ray_prior(o, d, r0, sig_r, sig_ang):
    a = np.cross(d, [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(d, [1.0, 0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(d, a)
    Rm = np.stack([d, a, b], 1)
    lat = r0 * sig_ang
    return o + r0 * d, Rm @ np.diag([sig_r**2, lat**2, lat**2]) @ Rm.T


class CameraModel(Meas):
    """Pinhole camera on a pan/tilt head. Pose (pan about +z from +x, tilt up) is known at measurement time.
    z = [u, v] pixels, or [u, v, r] with a size-based range r = f*D/s whose noise is proportional to range (std = range_rel * r)."""

    def __init__(
        self,
        origin=(0, 0, 0),
        pan=0.0,
        tilt=0.0,
        f_px=1101.0,
        cx=960.0,
        cy=540.0,
        sigma_px=2.0,
        size_range=False,
        range_rel=0.15,
    ):
        self.o = np.asarray(origin, float)
        self.f, self.cx, self.cy, self.sp = f_px, cx, cy, sigma_px
        self.size_range, self.range_rel = size_range, range_rel
        self.dim = 3 if size_range else 2
        cp, sp, ct, st = np.cos(pan), np.sin(pan), np.cos(tilt), np.sin(tilt)
        self.fwd = np.array([ct * cp, ct * sp, st])
        self.right = np.array([sp, -cp, 0.0])
        self.down = -np.array([-st * cp, -st * sp, ct])
        self.Rcw = np.stack(
            [self.right, self.down, self.fwd]
        )  # world -> camera (x right, y down, z forward)

    def h(self, X):
        d = np.asarray(X)[..., :3] - self.o
        c = d @ self.Rcw.T
        zc = np.maximum(c[..., 2], 1e-6)
        out = [self.cx + self.f * c[..., 0] / zc, self.cy + self.f * c[..., 1] / zc]
        if self.size_range:
            out.append(np.linalg.norm(d, axis=-1))
        return np.stack(out, -1)

    def R(self, x):
        if not self.size_range:
            return np.eye(2) * self.sp**2
        r = np.linalg.norm(x[:3] - self.o)
        return np.diag([self.sp**2, self.sp**2, (self.range_rel * r) ** 2])

    def ray(self, z):
        d = (
            self.fwd
            + (z[0] - self.cx) / self.f * self.right
            + (z[1] - self.cy) / self.f * self.down
        )
        return self.o, d / np.linalg.norm(d)

    def init(self, z, r0=20.0, sig_r=10.0):
        o, d = self.ray(z)
        if self.size_range:
            r0, sig_r = z[2], self.range_rel * z[2]
        return _ray_prior(o, d, r0, sig_r, self.sp / self.f)

    def sample(self, rng, p):
        zt = self.h(p)
        return zt + np.sqrt(np.diag(self.R(p))) * rng.normal(size=self.dim)

    def in_view(self, p, w=1920, hgt=1080):
        u = self.h(p)
        return 0 <= u[0] < w and 0 <= u[1] < hgt and (p - self.o) @ self.fwd > 0


def triangulate(models, zs):
    """Least-squares intersection of the bearing rays of two or more BearingModel/CameraModel measurements -> (3,) point."""
    A, b = np.zeros((3, 3)), np.zeros(3)
    for m, z in zip(models, zs):
        o, d = m.ray(z)
        M = np.eye(3) - np.outer(d, d)
        A += M
        b += M @ o
    return np.linalg.solve(A, b)
