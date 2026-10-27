"""napl.track.learned: calibration metrics on a known Gaussian, dataset shapes/horizons, zero-init net == IMM2, tiny torch fit."""

import numpy as np
import pytest
from napl.track.learned import data as D
from napl.track.learned import metrics as M
from napl.track.learned import models as ML


def test_metrics_calibrated_gaussian_and_inflation():
    rng = np.random.default_rng(0)
    S, H = 20000, 2
    A = rng.normal(size=(3, 3))
    C = A @ A.T + np.eye(3)
    cov = np.broadcast_to(C, (S, H, 3, 3))
    y = rng.multivariate_normal(np.zeros(3), C, size=(S, H))
    r = M.evaluate(np.zeros((S, H, 3)), cov, y)
    assert (
        abs(r["c50"] - 0.5).max() < 0.02
        and abs(r["c90"] - 0.9).max() < 0.015
        and abs(r["d2"] - 1).max() < 0.03
    )
    wrong = M.inflate(
        cov, [4.0, 4.0]
    )  # 4x too large: covers more, ML scalar recovers 1/4
    assert M.evaluate(np.zeros((S, H, 3)), wrong, y)["c90"].min() > 0.97
    assert np.allclose(M.fit_inflation(np.zeros((S, H, 3)), wrong, y), 0.25, atol=0.02)
    assert np.allclose(
        M.fit_quantile(np.zeros((S, H, 3)), wrong, y, 0.9), 0.25, atol=0.03
    )


@pytest.fixture(scope="module")
def tiny():
    parts = [D.collect("turn", s) for s in (1001, 1002, 1003)] + [
        D.collect("cam", 1001)
    ]
    return {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}


def test_collect_shapes_and_horizons(tiny):
    d = tiny
    n = len(d["y"])
    assert (
        n > 300
        and d["hm"].shape == (n, D.NHIST, 6)
        and d["pm2"].shape == (n, 5, 3)
        and d["pc3"].shape == (n, 5, 6)
    )
    assert np.isfinite(d["y"]).all() and np.isfinite(d["pm2"]).all()
    # 0.2 s ahead is a short extrapolation: far inside the 2 m GOSPA match radius of the truth for most samples
    assert np.median(np.linalg.norm(d["pm2"][:, 0] - d["y"][:, 0], axis=-1)) < 1.0
    # truth at 1.0 s is 5 steps of 0.2 s further: |y_h - y_0| grows with the horizon
    assert np.median(np.linalg.norm(d["y"][:, 4] - d["y"][:, 0], axis=-1)) > 3.0
    assert (d["hn"] >= 1).all() and (d["hn"] <= D.NHIST).all()


def test_zero_head_equals_imm2_and_torch_fit(tiny):
    rng = np.random.default_rng(0)
    nin = D.NHIST * 15 + 21 + 15 * 3 + 1
    W = dict(
        kind="mlp",
        s_mu=np.zeros(15, np.float32),
        s_sd=np.ones(15, np.float32),
        f_mu=np.zeros(nin - D.NHIST * 15, np.float32),
        f_sd=np.ones(nin - D.NHIST * 15, np.float32),
        rs=np.ones(5),
        w1=rng.normal(size=(16, nin)).astype(np.float32),
        b1=np.zeros(16, np.float32),
        w2=rng.normal(size=(16, 16)).astype(np.float32),
        b2=np.zeros(16, np.float32),
        w3=np.zeros((45, 16), np.float32),
        b3=np.zeros(45, np.float32),
    )
    m, c = ML.predict_np(W, tiny)
    assert np.allclose(m, tiny["pm2"], atol=1e-5)
    ref = M.tri_to_cov(tiny["pc2"].astype(np.float64)) + ML.BASE_FLOOR * np.eye(3)
    assert np.allclose(c, ref, rtol=1e-3, atol=1e-6)
    torch = pytest.importorskip("torch")  # noqa: F841
    for kind in ("mlp", "gru"):
        Wt = ML.train(
            tiny,
            tiny,
            kind,
            epochs=2,
            bs=256,
            hidden=16,
            device="cpu",
            log=lambda *_: None,
        )
        m, c = ML.predict_np(Wt, tiny)
        assert np.isfinite(m).all() and np.linalg.eigvalsh(c).min() > 0
