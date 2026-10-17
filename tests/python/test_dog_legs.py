"""SP4 Task 6 diagnostic ladder, rung a: the per-leg Jacobian and force map, ground truth only. All four legs (fr is mirrored, rl / rr are the
front pair turned 180 deg): model.py's foot Jacobian against finite differences of the SIMULATED foot position, and the sign / frame of
tau = J^T (-R^T f) against the body's response when one foot's ground force is changed."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogcourse as C
import dogsim as D
import dogtruth as T
from loco import hw as H
from loco import model as M

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")


def prim(sim, name):
    prims = sim.call({"cmd_type": "Query", "cmd": {"path": ""}})["prims"]
    return next(p for p in prims if p.rsplit("/", 1)[-1] == name)


@needs_kernel
def test_jacobian_matches_finite_differences_of_the_sim_foot():
    m = M.from_build(C.base_model())
    sim = napl.Sim.load(C.base_model())
    trunk = prim(sim, "deck_top")
    rng = np.random.default_rng(5)
    for leg_i, leg in enumerate(LEGS):
        L = m.legs[leg]
        q = rng.uniform(0.6 * L.q_lo, 0.6 * L.q_hi)
        s = sim.fork()

        def foot(qq):
            for j, jn in enumerate(JOINTS):
                s.set(prim(s, f"{leg}:{jn}_j"), "qpos", [float(qq[j])])
            return np.array(s.query(prim(s, f"{leg}:foot_ball"), "pos", trunk)[0])

        x0, J = foot(q), L.fk_jac(q)[1]
        for j in range(3):
            dq = np.zeros(3)
            dq[j] = 1e-5
            assert np.allclose((foot(q + dq) - x0) / 1e-5, J[:, j], atol=2e-4), (leg, j)


# Measured over the first 16 ms (docs/dog/locomotion.md): the sign and frame are right for every leg; the GAIN is not 1. x 1.0; y 1.4-1.8;
# z 0.5: each leg's operational mass at the foot is 0.1 / 0.27 / 1.6 kg (x / y / z, reflected knee rotor inertia 0.012 kg m^2 over a 0.096 m
# lever), 6.4 kg for four legs against a 11.65 kg body: a vertical ground force needs the knee rotors to accelerate first.
GAIN = {0: (0.8, 1.25), 1: (0.8, 2.2), 2: (0.3, 1.2)}


def _stand(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    sim = napl.Sim.load(model)
    m = M.from_build(C.info(model)["nominal"])
    hw = H.Hw(sim)
    hw.enable()
    q0 = hw.joints()[0]
    q_st = m.stand_q(D.STAND)
    for k in range(1500):  # 1 s ramp, 2 s hold
        a = min(1.0, k * 0.002)
        hw.send(
            q0 + (q_st - q0) * a,
            np.zeros(12),
            np.full(12, D.KP),
            np.full(12, D.KD),
            np.zeros(12),
        )
        sim.step(2)
    return sim, m


def _push(sim, m, leg, df, ticks=10):
    """20 ms of force-mode legs: every foot carries its weight share, `leg` carries df more (world frame). Returns the trunk's dv."""
    s = sim.fork()
    hw, tr = H.Hw(s), T.Truth(s)

    def v_com():  # the CoM, not the trunk origin: a pushed foot rolls the body, the origin (7 cm above the CoM) moves with the roll
        w = np.array(
            s.query(T.FREE, "qvel")[0][3:6], float
        )  # free joint: body-frame angular rate
        return tr.vel() + M.quat_R(tr.quat()) @ np.cross(w, m.com)

    v0 = v_com()
    share = np.array([0.0, 0.0, m.mass * 9.81 / 4])
    for _ in range(ticks):
        q, _, _ = hw.joints()
        R = M.quat_R(tr.quat())
        tau = np.zeros(12)
        for i, l in enumerate(LEGS):
            f = share + (df if leg is None or i == leg else 0.0)
            tau[3 * i : 3 * i + 3] = m.legs[l].fk_jac(q[3 * i : 3 * i + 3])[1].T @ (
                -R.T @ f
            )
        hw.send(
            q, np.zeros(12), np.zeros(12), np.zeros(12), tau
        )  # no joint damping: the pushed body rolls, damping would fight the legs following it
        s.step(2)
    return v_com() - v0


@needs_kernel
def test_force_map_sign_and_frame_for_every_leg(tmp_path):
    sim, m = _stand(tmp_path)
    n = 8  # 16 ms: the bus latency eats the first tick, the open-loop legs (a pushed body rolls) start to drift after ~20 ms
    base = _push(sim, m, 0, np.zeros(3), n)
    df = 15.0
    for leg in range(4):
        for ax in range(3):
            d = np.zeros(3)
            d[ax] = df
            dv = _push(sim, m, leg, d, n) - base
            ratio = dv / (
                df / m.mass * 0.002 * n
            )  # 1 = the commanded force reached the CoM
            lo, hi = GAIN[ax]
            assert lo < ratio[ax] < hi, (LEGS[leg], "xyz"[ax], ratio.round(2))
            if (
                ax < 2
            ):  # a lone foot pushing up rolls the body; its reaction on the other open-loop legs is not a force-map error
                assert np.abs(np.delete(ratio, ax)).max() < 0.5 * ratio[ax], (
                    LEGS[leg],
                    "xyz"[ax],
                    ratio.round(2),
                )


@needs_kernel
def test_all_four_feet_pushing_together_move_the_com_along_the_command(tmp_path):
    sim, m = _stand(tmp_path)
    n, df = 8, 15.0
    base = _push(sim, m, None, np.zeros(3), n)
    for ax in range(3):
        d = np.zeros(3)
        d[ax] = df
        ratio = (_push(sim, m, None, d, n) - base) / (4 * df / m.mass * 0.002 * n)
        assert GAIN[ax][0] < ratio[ax] < GAIN[ax][1], ("xyz"[ax], ratio.round(2))
        assert np.abs(np.delete(ratio, ax)).max() < 0.4, ("xyz"[ax], ratio.round(2))
