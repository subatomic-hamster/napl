"""SP4 Task 2: the controller's model from the design. FK / Jacobian of all four legs (two mirrored, two turned 180 deg) against the simulated
dog (Review Focus 1), mass against the build report, IK round trip, stand pose with the designed knee directions."""

import json
import sys
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogcourse as C
from loco import model as M

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")


@pytest.fixture(scope="module")
def m():
    return M.from_build(C.base_model())


def prim(sim, name):
    prims = sim.call({"cmd_type": "Query", "cmd": {"path": ""}})["prims"]
    return next(p for p in prims if p.rsplit("/", 1)[-1] == name)


def test_mass_matches_the_build_report(m):
    rep = json.loads((C.base_model().parent / "dog.report.json").read_text())
    assert m.mass == pytest.approx(rep["mass"]["total"], rel=0.01)
    assert sum(lk.mass for lk in m.links) == pytest.approx(m.mass, rel=1e-9)


def test_jacobian_is_the_derivative_of_fk(m):
    rng = np.random.default_rng(2)
    for leg in LEGS:
        L = m.legs[leg]
        q = rng.uniform(0.6 * L.q_lo, 0.6 * L.q_hi)
        x, J = L.fk_jac(q)
        for j in range(3):
            dq = np.zeros(3)
            dq[j] = 1e-6
            assert np.allclose((L.fk_jac(q + dq)[0] - x) / 1e-6, J[:, j], atol=1e-5)


def test_ik_round_trip_and_designed_knee_directions(m):
    q = m.stand_q(0.82)
    feet = m.feet(q)
    for i, leg in enumerate(LEGS):
        L = m.legs[leg]
        assert (
            np.linalg.norm(feet[i, :2] - L.foot0[:2]) < 2e-3
        )  # foot under its build-pose spot (under the hip)
        knee_ahead = L.knee(q[3 * i : 3 * i + 3])[0] > L.p[1][0]
        assert knee_ahead == leg.startswith(
            "r"
        )  # spec / SP3b: front knees point backward, rear knees forward
        assert np.allclose(
            L.fk_jac(L.ik(feet[i], q[3 * i : 3 * i + 3] + 0.05))[0], feet[i], atol=1e-6
        )


@needs_kernel
def test_fk_matches_the_sim_for_every_leg_in_random_poses(m):  # Review Focus 1
    sim = napl.Sim.load(C.base_model())
    trunk = prim(sim, "deck_top")
    rng = np.random.default_rng(3)
    for _ in range(5):
        s = sim.fork()
        q = np.concatenate(
            [rng.uniform(0.7 * m.legs[l].q_lo, 0.7 * m.legs[l].q_hi) for l in LEGS]
        )
        for i, leg in enumerate(LEGS):
            for j, jn in enumerate(JOINTS):
                s.set(prim(s, f"{leg}:{jn}_j"), "qpos", [q[3 * i + j]])
        for i, leg in enumerate(LEGS):
            got = np.array(s.query(prim(s, f"{leg}:foot_ball"), "pos", trunk)[0])
            assert np.allclose(m.feet(q)[i], got, atol=1e-6), (leg, m.feet(q)[i], got)
