"""Discrimination baseline: hand-made features of a noisy position history (oracle-associated, so the classifier is isolated from the
tracker) and a numpy multinomial logistic regression. Classes: drone, balloon, hanging, bird, debris."""

import numpy as np

from .gen import DT
from .scenes import LABELS, build

FEATURES = (
    "speed_mean", "speed_std", "speed_max", "vz_std", "vh_p95", "acc_mean", "acc_std", "turn_std", "turn_absmean",
    "z_hp_rms", "z_band_0.5-2", "z_band_2-5", "z_band_5-10", "z_peak_hz", "xy_hp_rms", "straight", "z_mean", "z_std",
    "ln_diam", "ln_range", "ln_angsize",
)  # fmt: skip
LAG = 10  # 0.5 s differencing lag


def features(P, diam, rng, sigma=0.1):
    """P (N, 3) true positions at DT over the object's lifetime -> feature vector. Position noise sigma (m) is added here."""
    Z = P + rng.normal(size=P.shape) * sigma
    v = (Z[LAG:] - Z[:-LAG]) / (LAG * DT)
    sp, vh = np.linalg.norm(v, axis=1), np.linalg.norm(v[:, :2], axis=1)
    a = np.linalg.norm(v[LAG:] - v[:-LAG], axis=1) / (LAG * DT)
    ok = vh > 0.5
    psi = np.arctan2(v[:, 1], v[:, 0])
    dpsi = (psi[LAG:] - psi[:-LAG] + np.pi) % (2 * np.pi) - np.pi
    om = np.abs(dpsi[ok[LAG:] & ok[:-LAG]]) / (LAG * DT)
    w = int(1.0 / DT)

    def hp(x):  # remove a 1 s moving average
        return x - np.convolve(x, np.ones(w) / w, mode="same")

    zr = hp(Z[:, 2])[w:-w]
    xr = np.hypot(hp(Z[:, 0])[w:-w], hp(Z[:, 1])[w:-w])
    S = np.abs(np.fft.rfft((zr - zr.mean()) * np.hanning(len(zr)))) ** 2
    fr = np.fft.rfftfreq(len(zr), DT)
    tot = S.sum() + 1e-12
    band = [
        S[(fr >= lo) & (fr < hi)].sum() / tot
        for lo, hi in ((0.5, 2), (2, 5), (5, 10.01))
    ]
    path = np.linalg.norm(np.diff(Z, axis=0), axis=1).sum()
    d_hat = float(
        np.mean(diam * (1 + 0.15 * rng.normal(size=len(P))))
    )  # per-frame apparent-size estimate, averaged
    rng_m = float(np.linalg.norm(Z, axis=1).mean())
    return np.array(
        [sp.mean(), sp.std(), sp.max(), v[:, 2].std(), np.percentile(vh, 95), a.mean(), a.std(), om.std() if len(om) > 3 else 0.0,
         om.mean() if len(om) > 3 else 0.0, zr.std(), *band, fr[np.argmax(S)], xr.std(), np.linalg.norm(Z[-1] - Z[0]) / max(path, 1e-9),
         Z[:, 2].mean(), Z[:, 2].std(), np.log(d_hat), np.log(rng_m), np.log(d_hat / rng_m)]
    )  # fmt: skip


def dataset(seeds, sigma=0.1, min_s=3.0):
    """Features and labels of every object (alive >= min_s) in build('random', seed) for the given seeds."""
    X, y = [], []
    for sd in seeds:
        s = build("random", sd)
        rng = np.random.default_rng([99, sd])
        for i, lab in enumerate(s.labels):
            idx = np.flatnonzero(s.alive[:, i])
            if len(idx) * DT >= min_s:
                X.append(features(s.truth[idx, i, :3], s.diam[i], rng, sigma))
                y.append(LABELS.index(lab))
    return np.array(X), np.array(y)


class Softmax:
    """Multinomial logistic regression (standardised features, L2, class-balanced weights, Adam full batch)."""

    def __init__(self, n_class=5, l2=1e-2, iters=1500, lr=0.05):
        self.nc, self.l2, self.iters, self.lr = n_class, l2, iters, lr

    def fit(self, X, y):
        self.mu, self.sd = X.mean(0), X.std(0) + 1e-9
        Xs = np.c_[(X - self.mu) / self.sd, np.ones(len(X))]
        Y = np.eye(self.nc)[y]
        cw = len(y) / (self.nc * np.maximum(np.bincount(y, minlength=self.nc), 1))
        sw = cw[y][:, None]
        W = np.zeros((Xs.shape[1], self.nc))
        m, v = np.zeros_like(W), np.zeros_like(W)
        for t in range(1, self.iters + 1):
            P = self._p(Xs, W)
            g = (
                Xs.T @ ((P - Y) * sw) / len(y)
                + self.l2 * np.r_[W[:-1], np.zeros((1, self.nc))]
            )
            m, v = 0.9 * m + 0.1 * g, 0.999 * v + 0.001 * g * g
            W -= self.lr * (m / (1 - 0.9**t)) / (np.sqrt(v / (1 - 0.999**t)) + 1e-8)
        self.W = W
        return self

    @staticmethod
    def _p(Xs, W):
        z = Xs @ W
        z -= z.max(1, keepdims=True)
        e = np.exp(z)
        return e / e.sum(1, keepdims=True)

    def predict(self, X):
        return self._p(np.c_[(X - self.mu) / self.sd, np.ones(len(X))], self.W).argmax(
            1
        )


def confusion(y, yh, nc=5):
    C = np.zeros((nc, nc), int)
    np.add.at(C, (y, yh), 1)
    return C


ABLATIONS = {
    "all": (),
    "no size": ("ln_diam", "ln_angsize"),
    "no oscillation spectrum": (
        "z_hp_rms",
        "z_band_0.5-2",
        "z_band_2-5",
        "z_band_5-10",
        "z_peak_hz",
        "xy_hp_rms",
    ),
    "no size, no spectrum, no height/range": (
        "ln_diam",
        "ln_angsize",
        "z_hp_rms",
        "z_band_0.5-2",
        "z_band_2-5",
        "z_band_5-10",
        "z_peak_hz",
        "xy_hp_rms",
        "z_mean",
        "z_std",
        "ln_range",
    ),
}


def report(train_seeds, test_seeds, sigma, drop=(), data=None):
    """data = (Xa, ya, Xb, yb) to reuse features across ablations."""
    Xa, ya, Xb, yb = data or (*dataset(train_seeds, sigma), *dataset(test_seeds, sigma))
    keep = [i for i, f in enumerate(FEATURES) if f not in drop]
    clf = Softmax().fit(Xa[:, keep], ya)
    C = confusion(yb, clf.predict(Xb[:, keep]))
    return dict(sigma=sigma, n_train=len(ya), n_test=len(yb), C=C, recall=np.diag(C) / np.maximum(C.sum(1), 1), acc=float(np.trace(C) / C.sum()),
                bird_as_drone=int(C[LABELS.index("bird"), LABELS.index("drone")]), n_bird=int(C[LABELS.index("bird")].sum()),
                drone_as_bird=int(C[LABELS.index("drone"), LABELS.index("bird")]), n_drone=int(C[LABELS.index("drone")].sum()))  # fmt: skip
