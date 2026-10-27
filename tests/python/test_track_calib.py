"""napl.track.calib: ML fit recovers a known mode/NIS/horizon-dependent variance factor; NaN-safe NIS feature; calibrated_cov scaling."""

import numpy as np
from napl.track import calib as C


def test_fit_recovers_scale_and_calibrates():
    rng = np.random.default_rng(1)
    S = 40000
    q = rng.uniform(0, 1, (S, 1))
    n = rng.normal(0, 0.5, (S, 1))
    h = np.array([0.2, 0.6, 1.0])[None]
    a = rng.normal(0, 0.7, (S, 1))
    w_true = np.array([-1.0, 0.5, 0.0, 0.8, 0.3, 0.4, 0.0, 0.5, 0.2])
    phi = C.features(q, n, h, a)
    s = np.exp(phi @ w_true)
    d2 = s * rng.chisquare(
        3, size=s.shape
    )  # truth error is chi-square(3) with variance factor s under the raw cov
    w = C.fit(phi, d2)
    assert np.abs(phi @ (w - w_true)).max() < 0.1
    # after dividing by the fitted factor the fraction inside the 90 % ellipsoid is 0.90
    assert abs((d2 / np.exp(phi @ w) <= 6.2514).mean() - 0.9) < 0.01


def test_nis_feature_and_calibrated_cov():
    assert C.nis_feature([np.nan, np.nan]) == 0.0  # no update yet: neutral
    assert np.isclose(
        C.nis_feature([3.0, np.nan, 3.0]), 0.0
    )  # mean NIS == dim: neutral
    assert np.isclose(C.nis_feature([6.0]), np.log(2.0))
    w = np.zeros(9)
    assert np.isclose(
        C.accel_feature([2, 0, 0], [0, 0, 0], 1.0, floor=0.0), np.log(2.0)
    )
    w[0] = np.log(0.5)
    cov = np.diag([1.0, 2.0, 3.0])
    assert np.allclose(C.calibrated_cov(cov, [0.9, 0.1], [3.0], 0.6, w), 0.5 * cov)
    assert np.allclose(
        C.calibrated_cov(cov, [0.9, 0.1], [3.0], 0.6, None),
        cov * C.scale([0.9, 0.1], [3.0], 0.6),
    )
