"""Prediction metrics for a Gaussian predictive distribution (mean (S, H, 3), cov (S, H, 3, 3)) against truth (S, H, 3):
RMSE (3-D, per horizon), Gaussian NLL, and calibration = fraction of truth inside the central 50 / 90 % ellipsoids
(3-D Mahalanobis d2 <= chi-square(3) quantile). A calibrated model gives 0.50 / 0.90; fraction above = covariance too large
(pessimistic Ph), below = too small."""

import numpy as np

CHI2_3 = {0.5: 2.3660, 0.9: 6.2514}  # chi-square quantiles, 3 dof
EPS = 1e-9


def tri_to_cov(t):
    """(..., 6) as xx yy zz xy xz yz -> (..., 3, 3)."""
    C = np.zeros(t.shape[:-1] + (3, 3), t.dtype)
    for k, (i, j) in enumerate(((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))):
        C[..., i, j] = C[..., j, i] = t[..., k]
    return C


def mahal2(mean, cov, y):
    e = (y - mean).astype(np.float64)
    C = cov.astype(np.float64) + EPS * np.eye(3)
    return np.einsum("...i,...i->...", e, np.linalg.solve(C, e[..., None])[..., 0]), C


def evaluate(mean, cov, y):
    """-> dict of (H,) arrays: rmse, nll, c50, c90, d2 (mean Mahalanobis d2 / 3, calibrated = 1)."""
    d2, C = mahal2(mean, cov, y)
    nll = 0.5 * (d2 + np.linalg.slogdet(C)[1] + 3 * np.log(2 * np.pi))
    e2 = ((y - mean).astype(np.float64) ** 2).sum(-1)
    return dict(
        rmse=np.sqrt(e2.mean(0)),
        nll=nll.mean(0),
        c50=(d2 <= CHI2_3[0.5]).mean(0),
        c90=(d2 <= CHI2_3[0.9]).mean(0),
        d2=d2.mean(0) / 3,
    )


def fit_inflation(mean, cov, y, per_horizon=True):
    """Maximum-likelihood scalar s with cov' = s cov: s = mean(d2)/3 (per horizon, or one value for all horizons)."""
    d2 = mahal2(mean, cov, y)[0] / 3
    return d2.mean(0) if per_horizon else np.full(d2.shape[1], d2.mean())


def fit_quantile(mean, cov, y, q=0.9, per_horizon=True):
    """Scalar s that puts exactly a fraction q of the fit set inside the q ellipsoid of s cov (robust to heavy tails)."""
    d2 = mahal2(mean, cov, y)[0]
    s = np.quantile(d2, q, axis=0 if per_horizon else None) / CHI2_3[q]
    return s if per_horizon else np.full(d2.shape[1], s)


def inflate(cov, s):
    return cov * np.asarray(s)[None, :, None, None]
