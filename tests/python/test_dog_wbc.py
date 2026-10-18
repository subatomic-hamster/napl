"""SP4 Task 7: whole-body controller. Structure of the controller's floating-base dynamics, the QP keeps the MPC forces inside the friction pyramid
and the drivers' torque limits (Review Focus 3), the 500 Hz solve budget, and a WBC trot on the simulated dog measured against the J^T baseline."""

import sys
import time
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C
import dogwalk as W
from loco import model as M
from loco import mpc as Mp
from loco import wbc as Wb

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
c = W.COLS.index


@pytest.fixture(scope="module")
def m():
    return M.from_build(C.base_model())


def trot_problem(m, v=1.0):
    """Stand pose, moving at v along x, fl + rr in stance, fr + rl in swing: a real MPC solve gives f_mpc."""
    q = m.stand_q(0.82)
    feet = m.feet(q)
    stance = np.array([True, False, False, True])
    x0 = np.r_[0.0, 0.0, 0.0, m.com, 0.0, 0.0, 0.0, v, 0.0, 0.0, -Mp.G]
    xref = np.tile(x0, (Mp.N, 1))
    xref[:, 3] += v * Mp.DT * np.arange(1, Mp.N + 1)
    mu = np.full(4, 0.4)
    f_mpc, ok = Mp.Mpc(m.mass, m.inertia).solve(
        x0,
        xref,
        np.tile(feet - m.com, (Mp.N, 1, 1)),
        np.tile(stance, (Mp.N, 1)).astype(float),
        mu,
    )
    assert ok
    swing = (
        feet + [0.0, 0.0, 0.05],
        np.tile([v, 0.0, 0.0], (4, 1)),
    )  # trunk at the origin: body frame = world frame
    body_ref = (np.zeros(3), np.array([v, 0.0, 0.0]), np.eye(3))
    args = (
        np.zeros(3),
        np.eye(3),
        np.zeros(3),
        np.array([v, 0.0, 0.0]),
        q,
        np.zeros(12),
        f_mpc,
        stance,
        mu,
        body_ref,
        swing,
    )
    return args, f_mpc, stance, mu


def test_dynamics_structure(m):
    assert sum(lk.mass for lk in m.links) == pytest.approx(m.mass, rel=1e-9)
    q = m.stand_q(0.82)
    A, b, g = Wb.dynamics(m, np.eye(3), q, np.zeros(18))
    assert np.allclose(A, A.T, atol=1e-9) and np.linalg.eigvalsh(A).min() > 0
    assert np.allclose(
        A[0:3, 0:3], m.mass * np.eye(3), rtol=1e-9
    )  # linear block = total mass
    assert np.allclose(
        g[0:3], [0.0, 0.0, m.mass * Mp.G], rtol=1e-9
    )  # gravity on the trunk = weight
    assert np.allclose(b, 0.0, atol=1e-9)  # at rest: no Coriolis / centrifugal terms
    assert np.all(
        np.diag(A)[6:] >= m.armature
    )  # reflected rotor inertia is in the joint diagonal
    u = np.random.default_rng(1).normal(0, 1, 18)
    assert np.isfinite(Wb.dynamics(m, np.eye(3), q, u)[1]).all()


def test_wbc_keeps_mpc_forces_inside_friction_and_torque_limits(m):  # Review Focus 3
    args, f_mpc, stance, mu = trot_problem(m)
    wbc = Wb.Wbc(m)
    out = wbc.solve(*args)
    assert out.ok
    assert np.abs(out.f[~stance]).max() < 1e-6  # swing feet push nothing
    for i in np.flatnonzero(stance):
        assert np.abs(out.f[i, :2]).max() <= mu[i] * out.f[i, 2] + 1e-3
        assert Mp.F_MIN - 1e-3 <= out.f[i, 2] <= Mp.F_MAX + 1e-3
    assert np.all(np.abs(out.tau) <= wbc.tau_max + 1e-6)
    assert (
        np.abs(out.f - f_mpc).max() < 2.0
    )  # no limit binds: the MPC forces pass through (N)
    st12 = np.repeat(
        stance, 3
    )  # now the stance legs' torque rows bind (this pose needs only ~7 N m at the worst joint, far from 22)
    wbc.tau_max = np.where(st12, np.maximum(0.5 * np.abs(out.tau), 0.5), wbc.tau_max)
    tight = wbc.solve(*args)
    assert tight.ok and np.all(np.abs(tight.tau) <= wbc.tau_max + 1e-6)
    assert (
        np.abs(tight.f - f_mpc).max() > 2.0
    )  # the QP moved the forces instead of clipping torques after the fact


def test_cpp_dynamics_match_python(m):
    cpp = Wb.make(m)
    if isinstance(cpp, Wb.Wbc):
        pytest.skip("C++ WBC not built (cmake -DNAPL_LOCO_CPP=ON)")
    rng = np.random.default_rng(4)
    q, u = m.stand_q(0.82) + rng.normal(0, 0.1, 12), rng.normal(0, 1.0, 18)
    R = Wb._rot(rng.normal(0, 1, 3), 0.3)
    A, b, g, Jc, Jd = cpp.debug(R, q, u)
    A0, b0, g0 = Wb.dynamics(m, R, q, u)
    _, Jc0 = Wb._elements(m, R, q)
    _, Jc1 = Wb._elements(m, *Wb._advance(R, q, u, Wb.EPS))
    for x, y in (
        (A, A0),
        (b, b0),
        (g, g0),
        (Jc, Jc0.reshape(12, 18)),
        (Jd, ((Jc1 - Jc0) @ u / Wb.EPS).ravel()),
    ):
        assert np.allclose(x, y, rtol=1e-6, atol=1e-6), np.abs(x - y).max()


def test_cpp_wbc_matches_python(m):
    """The C++ port solves the same problem: same torques / forces / joint targets as the Python class (different QP builds: 1e-4 relative)."""
    cpp = Wb.make(m)
    if isinstance(cpp, Wb.Wbc):
        pytest.skip("C++ WBC not built (cmake -DNAPL_LOCO_CPP=ON)")
    py, rng = Wb.Wbc(m), np.random.default_rng(3)
    args, *_ = trot_problem(m)
    for _ in range(
        5
    ):  # warm-started sequence of different joint speeds, like consecutive ticks
        a = list(args)
        a[5] = rng.normal(0, 3.0, 12)
        a[4] = a[4] + rng.normal(0, 0.02, 12)
        o1, o2 = py.solve(*a), cpp.solve(*a)
        assert o1.ok and o2.ok
        for x, y in (
            (o1.p_des, o2.p_des),
            (o1.v_des, o2.v_des),
            (o1.tau, o2.tau),
            (o1.f, o2.f),
        ):
            assert np.allclose(x, y, rtol=1e-3, atol=2e-3), np.abs(x - y).max()


def test_solve_time_budget(m):
    args, *_ = trot_problem(m)
    wbc, rng, ts = Wb.make(m), np.random.default_rng(2), []
    assert not isinstance(wbc, Wb.Wbc), (
        "C++ WBC not built (the 1 ms budget is the C++ port's; Python measures 9 ms): "
        "cmake --preset dev -DNAPL_LOCO_CPP=ON && cmake --build --preset dev --target loco_wbc_cpp "
        "(add -DPython_EXECUTABLE=$PWD/.venv/bin/python if cmake cannot find the interpreter); output build/loco/"
    )
    for _ in range(200):
        a = list(args)
        a[5] = rng.normal(0, 3.0, 12)  # joint speeds of a fast trot
        t0 = time.perf_counter()
        wbc.solve(*a)
        ts.append(time.perf_counter() - t0)
    print(
        f"WBC solve ({type(wbc).__module__}): median {np.median(ts) * 1e3:.3f} ms, p99 {np.percentile(ts, 99) * 1e3:.3f} ms"
    )
    assert (
        np.median(ts) < Wb.BUDGET_S
    )  # over budget: Task 7 Step 6 (C++ port), never a looser budget


@needs_kernel
def test_wbc_trot_against_the_jt_baseline(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    runs = {
        k: W.run(model, dict(W.DEFAULTS, v_cmd=1.0, duration=8.0, leg_ctl=k), log=False)
        for k in ("wbc", "jt")
    }
    rows = np.array(runs["wbc"]["rows"])
    late = rows[rows[:, c("t")] > 5.0]
    assert late[:, c("vx")].mean() == pytest.approx(1.0, abs=0.1)
    assert rows[:, c("fallen")].max() == 0 and rows[:, c("fault")].max() == 0
    assert (
        runs["wbc"]["res"]["wbc_fail"] < 0.01 * len(rows) * W.ROW_EVERY
    )  # < 1 % of control ticks fall back to J^T
    ft = {k: r["res"]["f_track_rms"] for k, r in runs.items()}
    print(f"force tracking RMS (x mg): wbc {ft['wbc']:.3f}, jt {ft['jt']:.3f}")
    assert (
        ft["wbc"] <= 1.25 * ft["jt"]
    )  # a WBC tracking the MPC forces clearly worse than plain J^T is a bug, not a result
