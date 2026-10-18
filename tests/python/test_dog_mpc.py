"""SP4 Task 4: convex MPC. Weight carried evenly at a stand, physical forces with feet in swing and a braking velocity (the WBC test of Review Focus 3 builds on it),
the solve-time budget of a 50 Hz loop."""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "dog"))
from loco import mpc

M, IB = (
    12.0,
    np.diag([0.08, 0.25, 0.28]),
)  # test numbers of a 12 kg dog trunk (kg, kg m^2)
FEET = np.array(
    [
        [0.25, 0.15, -0.35],
        [0.25, -0.15, -0.35],
        [-0.25, 0.15, -0.35],
        [-0.25, -0.15, -0.35],
    ]
)


def problem(contact=(1, 1, 1, 1), v0=(0.0, 0.0, 0.0)):
    ref = np.array([0, 0, 0, 0, 0, 0.35, 0, 0, 0, 0, 0, 0, -mpc.G], float)
    x0 = ref.copy()
    x0[9:12] = v0
    return (
        x0,
        np.tile(ref, (mpc.N, 1)),
        np.tile(FEET, (mpc.N, 1, 1)),
        np.tile(np.array(contact, float), (mpc.N, 1)),
    )


def test_standing_forces_carry_the_weight_evenly():
    f, ok = mpc.Mpc(M, IB).solve(*problem(), mu=np.full(4, 0.5))
    assert ok
    assert f[:, 2].sum() == pytest.approx(M * mpc.G, rel=0.02)
    assert (
        np.allclose(f[:, 2], M * mpc.G / 4, rtol=0.1) and np.abs(f[:, :2]).max() < 1.0
    )


def test_forces_inside_friction_pyramid_and_zero_in_swing():
    x0, xref, r, c = problem(contact=(1, 0, 0, 1), v0=(0.0, 0.6, 0.0))
    c[6:] = [0, 1, 1, 0]  # the other diagonal takes over mid-horizon
    mu = np.full(4, 0.4)
    f, ok = mpc.Mpc(M, IB).solve(x0, xref, r, c, mu)
    assert ok
    assert np.abs(f[[1, 2]]).max() < 1e-3  # swing feet push nothing
    for i in (0, 3):
        assert np.abs(f[i, :2]).max() <= mu[i] * f[i, 2] + 1e-3
        assert mpc.F_MIN - 1e-3 <= f[i, 2] <= mpc.F_MAX + 1e-3
    assert f[:, 1].sum() < 0.0  # the lateral velocity is braked


def test_solve_time_budget():
    m, rng, ts = mpc.Mpc(M, IB), np.random.default_rng(1), []
    for k in range(100):
        x0, xref, r, c = problem(
            contact=(1, 0, 0, 1) if k % 2 else (0, 1, 1, 0),
            v0=rng.uniform(-1, 1, 3) * [2, 0.5, 0.2],
        )
        t0 = time.perf_counter()
        m.solve(x0, xref, r, c, np.full(4, 0.6))
        ts.append(time.perf_counter() - t0)
    print(f"MPC solve: median {np.median(ts) * 1e3:.2f} ms, max {max(ts) * 1e3:.2f} ms")
    assert np.median(ts) < 0.008  # 40 % of the 20 ms MPC period (50 Hz)
