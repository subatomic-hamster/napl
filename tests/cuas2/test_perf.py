"""P13 PERF: the flywheel hold after the last target, and the IMM-mixture Ph. Kernel-free (FireControl2 + a fake IMM)."""

import numpy as np
import pytest
from napl.cuas2.app import hit_probability
from napl.cuas2.calib import Calib
from napl.cuas2.launcher import LauncherModel

from cuas2.test_app import NOW, FakeTrack, aligned, make_fc, tgt, view


def test_wheels_stay_up_for_spin_down_s_after_the_last_target_then_idle():
    fc = make_fc(spin_down_s=2.0)
    t = tgt()
    d = fc.step(NOW, aligned(fc, t))
    assert d.flywheel == fc.cfg.flywheel_duty and d.target is not None
    d = fc.step(NOW + 0.5, view([], now=NOW + 0.5, deadman_t=NOW + 0.4))  # one dropout
    assert "no_target" in d.reasons and d.flywheel == fc.cfg.flywheel_duty
    d = fc.step(NOW + 2.5, view([], now=NOW + 2.5, deadman_t=NOW + 2.4))
    assert d.flywheel == fc.cfg.idle_duty  # held long enough: back to the pre-rev


def test_wheel_hold_needs_a_live_deadman_and_is_off_with_spin_down_zero():
    fc = make_fc(spin_down_s=2.0)
    fc.step(NOW, aligned(fc, tgt()))
    d = fc.step(
        NOW + 0.5, view([], now=NOW + 0.5, deadman_t=NOW - 5.0)
    )  # stale dead-man
    assert d.flywheel == 0.0
    fc = make_fc(spin_down_s=0.0)
    fc.step(NOW, aligned(fc, tgt()))
    d = fc.step(NOW + 0.5, view([], now=NOW + 0.5, deadman_t=NOW + 0.4))
    assert d.flywheel == fc.cfg.idle_duty


class Mode:
    def __init__(self, x, sig):
        self.x, self.P = np.asarray(x, float), np.diag(np.full(len(x), sig**2))


class FakeIMM(FakeTrack):
    """Mixture: 90 % a tight CV mode, 10 % a wide manoeuvre mode; predict_to returns the moment-matched single Gaussian (wide)."""

    def __init__(self, mean):
        tight, wide = Mode(mean, 0.02), Mode(mean, 1.0)
        mu = np.array([0.9, 0.1])
        cov = 0.9 * tight.P + 0.1 * wide.P
        super().__init__(np.asarray(mean, float), cov)
        self.t = NOW
        self.est = type("E", (), {})()
        self.est.mu, self.est.modes = mu, [tight, wide]
        self.est.copy = lambda: self.est
        self.est.predict = lambda dt: None

    def predict_to(self, t):
        return self.mean, self.cov


def _ph(mixture):
    mean = [8.0, 0.0, 3.2, 0, 0, 0]
    t = tgt()
    t.track = FakeIMM(mean)
    m = LauncherModel(Calib())
    fc = make_fc()
    d = fc.step(NOW, aligned(fc, tgt()))
    pose = fc.geom.pose(*d.aim[:2])
    rng = np.random.default_rng(1)
    return hit_probability(
        m,
        fc.setpoint,
        pose.muzzle,
        d.solution["u"],
        t,
        NOW + d.solution["t_flight"],
        d.solution["t_flight"],
        0.16,
        2000,
        rng,
        mixture=mixture,
    )[0]


def test_mixture_ph_is_higher_than_the_moment_matched_gaussian_when_the_wide_mode_is_unlikely():
    assert _ph(True) > _ph(False) + 0.1


def test_mixture_falls_back_to_the_gaussian_for_a_track_without_modes():
    t = tgt()  # FakeTrack has no .est: the plain path must still work with mixture=True
    t.track.t = NOW
    t.track.est = type("E", (), {"copy": lambda s: s, "predict": lambda s, dt: None})()
    m = LauncherModel(Calib())
    fc = make_fc()
    d = fc.step(NOW, aligned(fc, t))
    pose = fc.geom.pose(*d.aim[:2])
    a = hit_probability(
        m,
        fc.setpoint,
        pose.muzzle,
        d.solution["u"],
        t,
        NOW + 0.4,
        0.4,
        0.16,
        500,
        np.random.default_rng(2),
        mixture=True,
    )[0]
    b = hit_probability(
        m,
        fc.setpoint,
        pose.muzzle,
        d.solution["u"],
        t,
        NOW + 0.4,
        0.4,
        0.16,
        500,
        np.random.default_rng(2),
        mixture=False,
    )[0]
    assert a == pytest.approx(b)
