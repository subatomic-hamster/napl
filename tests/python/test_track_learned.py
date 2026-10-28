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


@pytest.fixture(scope="module")
def thr():
    return D.collect("route_hover", 1001, suite="threats", stride=4)


def test_threats_collect_and_group_meta(thr):
    d = thr
    n = len(d["y"])
    assert n > 50 and d["meta"].shape == (n, 7) and d["hm"].shape == (n, D.NHIST, 6)
    assert set(d["meta"][:, 5]) == {
        D.GROUPS.index("route")
    }  # route_hover = route generator only
    assert (d["meta"][:, 2] % 4 == 0).all() and np.isfinite(d["y"]).all()


def test_augment_is_a_consistent_rotation_and_scale(thr):
    d = thr
    a = D.augment(d, np.random.default_rng(0))
    cur, cur_a = d["hm"][:, -1, :3], a["hm"][:, -1, :3]
    v0 = np.linalg.norm(d["hm"][:, -1, 3:], axis=-1)
    ok = v0 > 0.1
    s = np.linalg.norm(a["hm"][:, -1, 3:], axis=-1)[ok] / v0[ok]
    assert s.min() > 0.69 and s.max() < 1.41
    # displacement to the truth / to the IMM2 prediction scale by the same s as the speed (rigid yaw + uniform scale)
    for k in ("y", "pm2"):
        r0 = np.linalg.norm(d[k] - cur[:, None], axis=-1)[ok]
        r1 = np.linalg.norm(a[k] - cur_a[:, None], axis=-1)[ok]
        big = r0 > 0.05
        assert np.allclose(r1[big], (r0 * s[:, None])[big], rtol=1e-3)
    # covariances: eigenvalues scale by s^2 (a rotation leaves them unchanged), so they stay positive definite
    C0 = M.tri_to_cov(d["pc2"].astype(np.float64))[ok]
    C1 = M.tri_to_cov(a["pc2"].astype(np.float64))[ok]
    assert np.allclose(
        np.linalg.eigvalsh(C1),
        np.linalg.eigvalsh(C0) * (s**2)[:, None, None],
        rtol=1e-2,
        atol=1e-6,
    )


def test_hybrid_floor_mix_and_guard_fit():
    from napl.track.learned import hybrid as HY

    rng = np.random.default_rng(1)
    S, H = 3000, 2
    A = rng.normal(size=(S, H, 3, 3)) * 0.3
    C2 = A @ np.swapaxes(A, -1, -2) + np.eye(3)  # IMM2 covariance
    Cl = 0.1 * np.broadcast_to(
        np.diag([4.0, 0.2, 0.2]), C2.shape
    )  # learned: too tight, other shape
    mu2 = rng.normal(size=(S, H, 3))
    mul = mu2 + 0.1
    # floor: Loewner-above both inputs; mean untouched; g adds the disagreement outer product
    m, C = HY.hybrid("floor", mul, Cl, mu2, C2, k=1.0, g=0.0)
    assert m is mul
    assert np.linalg.eigvalsh(C - Cl).min() > -1e-9
    assert np.linalg.eigvalsh(C - C2).min() > -1e-9
    d = mul - mu2
    _, Cg = HY.hybrid("floor", mul, Cl, mu2, C2, k=1.0, g=2.0)
    assert np.allclose(Cg - C, 2.0 * d[..., :, None] * d[..., None, :])
    # mix endpoints and imm2cov
    assert np.allclose(HY.hybrid("mix", mul, Cl, mu2, C2, k=0.0)[1], Cl)
    assert np.allclose(HY.hybrid("mix", mul, Cl, mu2, C2, k=1.0)[1], C2)
    assert np.allclose(HY.hybrid("imm2cov", mul, Cl, mu2, C2, k=2.0)[1], 2 * C2)
    # truth ~ N(mu_l, C2): the guard fit must pick a covariance near C2 (cover ~0.9), not the over-confident Cl
    e = np.einsum("shij,shj->shi", np.linalg.cholesky(C2), rng.normal(size=(S, H, 3)))
    grp = np.arange(S) % 3
    k, g, info = HY.fit(
        "floor", mul, Cl, mu2, C2, mul + e, grp, objective="guard", with_g=False
    )
    assert 0.85 < info["cover"] < 0.95 and 0.5 <= k <= 1.5
