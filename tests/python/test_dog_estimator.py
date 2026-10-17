"""SP4 Task 3: estimator. Pure-math checks on synthetic signals (attitude convergence, the KF on a fake one-joint-per-axis leg) and on the
simulated dog under a PD squat / stand (no gait yet: Task 6 checks the trot and the contact estimate)."""

import math
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
from loco import estimator as E
from loco import hw as H
from loco import model as M
from loco import terrain as TR

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


class PointLeg:  # foot = q (trunk frame), J = I: lets the KF be checked on exact kinematics
    r_foot = 0.0

    def fk_jac(self, q):
        return np.array(q, float), np.eye(3)


class FakeModel:
    imu_R = np.eye(3)
    legs = {l: PointLeg() for l in ("fl", "fr", "rl", "rr")}


FEET = np.array(
    [
        [0.25, 0.15, -0.35],
        [0.25, -0.15, -0.35],
        [-0.25, 0.15, -0.35],
        [-0.25, -0.15, -0.35],
    ]
)


def test_attitude_converges_to_a_static_tilt():
    est = E.Estimator(FakeModel(), TR.FlatMap())
    est.reset(FEET.ravel())
    th = math.radians(10)
    g_body = (
        np.array([0.0, math.sin(th), math.cos(th)]) * 9.81
    )  # specific force of a trunk rolled 10 deg, at rest
    for _ in range(2500):  # 5 s at 500 Hz
        st = est.update(FEET.ravel(), np.zeros(12), np.zeros(12), np.zeros(3), g_body)
    assert math.degrees(st.rpy[0]) == pytest.approx(10.0, abs=0.5)


def test_kf_tracks_a_body_moving_over_fixed_feet():
    est = E.Estimator(FakeModel(), TR.FlatMap(-0.35))
    est.reset(FEET.ravel())
    v = np.array([0.5, 0.1, 0.0])
    dt = 0.002
    for k in range(1000):
        p = v * k * dt
        r = (FEET - p).ravel()  # feet fixed in the world, body moving
        st = est.update(
            r,
            np.tile(-v, 4),
            np.tile([0, 0, -30.0], 4),
            np.zeros(3),
            np.array([0, 0, 9.81]),
        )
    assert np.allclose(st.v, v, atol=0.01) and np.allclose(st.p, p, atol=0.01)


@needs_kernel
def test_estimator_tracks_a_squat_and_stand(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    sim = napl.Sim.load(model)
    m = M.from_build(C.info(model)["nominal"])
    hw, tr = H.Hw(sim), T.Truth(sim)
    est = E.Estimator(m, TR.FlatMap())
    hw.enable()
    q0 = hw.joints()[0]
    est.reset(q0)
    q_st, q_sq = m.stand_q(D.STAND), m.stand_q(D.SQUAT)
    errs, tilts, agree = [], [], []
    for k in range(2000):  # 4 s: 1 s to stand, 1.5 s squat, 1.5 s stand
        t = k * 0.002
        a = (
            min(1.0, t)
            if t < 1.0
            else (
                min(1.0, (t - 1.0) / 1.0) if t < 2.5 else 1 - min(1.0, (t - 2.5) / 1.0)
            )
        )
        tgt = q0 + (q_st - q0) * a if t < 1.0 else q_st + (q_sq - q_st) * a
        hw.send(tgt, np.zeros(12), np.full(12, D.KP), np.full(12, D.KD), np.zeros(12))
        sim.step(2)
        q, dq, tau = hw.joints()
        st = est.update(q, dq, tau, *hw.imu())
        if t > 1.5:
            errs.append(np.linalg.norm(st.v - tr.vel()))
            tilts.append(
                abs(math.degrees(math.acos(min(1.0, st.R[2, 2]))) - tr.tilt_deg())
            )
            # all four feet are on the floor by construction (the build-pose stance): the virtual FootContact (30 / 15 N, noise 5 N) is no
            # reference at a 28.6 N static load, so the trot test (Task 6) checks the estimate against the device instead
            agree.append(np.all(st.contact))
    assert np.sqrt(np.mean(np.square(errs))) < 0.03
    assert max(tilts) < 1.0
    assert np.mean(agree) > 0.99
