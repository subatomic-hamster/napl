"""Mode- and innovation-conditioned calibration of an IMM's predicted covariance (docs/cuas2/ml-prediction.md, ML-2).

EXPERIMENTAL, NOT EXPORTED, DEFAULT_W is identity: this did NOT reach the +-0.05 cover90 target on every kind (worst kind +-0.21,
also leave-one-kind-out); see docs/cuas2/ml-prediction.md "ML-2". Kept as the reproducible fit behind `ml_predict.py calib`.

The IMM extrapolation covariance is pessimistic on benign targets (needed variance factor ~0.3) and about right on manoeuvring
ones (~0.7), so one global factor over-confidents the latter. This scales the 3x3 position covariance at horizon h by
    s = exp(w . phi),   phi = [1, h, h^2, q, q h, n, n h, a, a h]
with q = probability of the manoeuvre mode (IMM mode index 1) and n = ln of the mean NIS of the last k updates divided by its
expected value (measurement dimension). w is fitted by maximum likelihood (fit()); DEFAULT_W comes from tools/ml_predict.py calib."""

import numpy as np

# Fitted on the val split (seeds 2001-2040, six scenario kinds, IMM2 = CV + Singer), k = 5, dim 3. Regenerate with `ml_predict.py calib`.
DEFAULT_W = np.zeros(9)  # replaced below after the fit; identity (s = 1) until then
K_NIS = 5
NIS_CLIP = (0.05, 20.0)


def nis_feature(nis, dim=3):
    """ln(mean of the finite recent NIS values / dim), clipped; 0 when there are none. nis: (..., k) array-like."""
    x = np.asarray(nis, float)
    ok = np.isfinite(x)
    m = np.where(ok, x, 0.0).sum(-1) / np.maximum(ok.sum(-1), 1)
    r = np.where(ok.any(-1), m / dim, 1.0)
    return np.log(np.clip(r, *NIS_CLIP))


def accel_feature(v_now, v_prev, span, floor=0.2):
    """ln(|mean acceleration over the last `span` s| + floor) from the track's own velocity estimates (m/s), (..., 3) each."""
    a = (
        np.linalg.norm(np.asarray(v_now, float) - np.asarray(v_prev, float), axis=-1)
        / span
    )
    return np.log(a + floor)


def features(q, n, h, a=0.0):
    """phi (..., 9) for broadcastable manoeuvre-mode probability q, NIS feature n, horizon h (s), acceleration feature a."""
    q, n, h, a = np.broadcast_arrays(*(np.asarray(x, float) for x in (q, n, h, a)))
    return np.stack([np.ones_like(h), h, h * h, q, q * h, n, n * h, a, a * h], -1)


def fit(phi, d2, iters=30, ridge=1e-6):
    """Maximum-likelihood w for cov' = exp(phi w) cov given squared 3-D Mahalanobis distances d2 of the truth under the raw cov.
    Convex Newton iteration on sum 0.5 (d2 exp(-phi w) + 3 phi w)."""
    P, d2 = phi.reshape(-1, phi.shape[-1]), np.asarray(d2, float).reshape(-1)
    w = np.zeros(P.shape[1])
    w[0] = np.log(max(d2.mean() / 3, 1e-3))
    for _ in range(iters):
        e = d2 * np.exp(-P @ w)
        g = 0.5 * P.T @ (3 - e)
        Hs = 0.5 * (P * e[:, None]).T @ P + ridge * np.eye(len(w))
        step = np.linalg.solve(Hs, g)
        w -= step
        if np.abs(step).max() < 1e-8:
            break
    return w


def scale(mu, nis, h, w=None, dim=3, acc=0.0):
    """Variance factor for horizon h given IMM mode probabilities mu (manoeuvre mode = index 1), recent NIS values and acc =
    accel_feature of the track's velocity history."""
    w = DEFAULT_W if w is None else w
    return float(
        np.exp(features(np.asarray(mu)[..., 1], nis_feature(nis, dim), h, acc) @ w)
    )


def calibrated_cov(cov, mu, nis, h, w=None, dim=3, acc=0.0):
    """Calibrated predicted position covariance: cov (3x3 at horizon h) * scale(...). nis = last k NIS values of the track."""
    return np.asarray(cov) * scale(mu, nis, h, w, dim, acc)
