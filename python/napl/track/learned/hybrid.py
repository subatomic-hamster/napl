"""ML-4 hybrids: the learned MLP/GRU mean (accurate) with a covariance that cannot fall below what the IMM2 tracker knows.

mu_l, C_l = learned mean / covariance; mu2, C2 = IMM2 extrapolation; d = mu_l - mu2 (the learned model's disagreement).
  imm2cov : (mu_l, k C2 + g d d^T)                     learned mean, IMM2 covariance shifted to it, scaled by k
  floor   : (mu_l, C_l + (k C2 - C_l)_+ + g d d^T)     (.)_+ = positive eigen-part, so C >= C_l and C >= k C2 (Loewner)
  mix     : (mu_l, (1-k) C_l + k C2 + g d d^T)         convex mix, k in [0, 1]
The mean is never changed, so RMSE equals the learned model's. (k, g) are fitted on a validation split of the TRAINING groups only
(fit()); the held-out group is never seen. numpy only."""

import numpy as np

from .metrics import CHI2_3, mahal2

MODES = ("imm2cov", "floor", "mix")
GRID_K = {
    "imm2cov": (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0),
    "floor": (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0),
    "mix": tuple(np.round(np.linspace(0, 1, 11), 2)),
}
GRID_G = (0.0, 0.5, 1.0, 2.0)
GUARD = 0.85  # fit "guard": every training group's val 90 % cover (mean over horizons) >= this, if reachable


def _pos(X):
    w, V = np.linalg.eigh(X)
    return (V * np.clip(w, 0, None)[..., None, :]) @ np.swapaxes(V, -1, -2)


def hybrid(mode, mu_l, C_l, mu2, C2, k=1.0, g=0.0):
    """-> (mean, cov), same shapes as the inputs ((S, H, 3), (S, H, 3, 3))."""
    d = mu_l - mu2
    D = g * d[..., :, None] * d[..., None, :]
    if mode == "imm2cov":
        C = k * C2 + D
    elif mode == "floor":
        C = C_l + _pos(k * C2 - C_l) + D
    elif mode == "mix":
        C = (1 - k) * C_l + k * C2 + D
    else:
        raise ValueError(mode)
    return mu_l, C


def fit(mode, mu_l, C_l, mu2, C2, y, group, objective="nll", with_g=True):
    """Grid fit of (k, g) on validation samples of the training groups. group (S,) ints = generator group per sample.
    objective "nll": minimum pooled NLL (mean over horizons); "guard": minimum pooled NLL among the grid points whose worst
    training-group 90 % cover is >= GUARD (if none reaches it, the point with the best worst-group cover). -> (k, g, info)."""
    gs = np.unique(group)
    best = None
    for k in GRID_K[mode]:
        for g in GRID_G if with_g else (0.0,):
            m, C = hybrid(mode, mu_l, C_l, mu2, C2, k, g)
            d2, Cm = mahal2(m, C, y)
            nll = float((0.5 * (d2 + np.linalg.slogdet(Cm)[1])).mean())
            cov = (d2 <= CHI2_3[0.9]).mean(1)
            worst = min(float(cov[group == q].mean()) for q in gs)
            ok = objective == "nll" or worst >= GUARD
            key = (0, nll) if ok else (1, -worst)
            if best is None or key < best[0]:
                best = (key, k, g, dict(nll=nll, worst=worst, cover=float(cov.mean())))
    return best[1], best[2], best[3]
