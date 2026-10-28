"""napl.threats: determinism, labels/shapes, in-scope speed bounds, flap frequency, discrimination smoke, and an unchanged-baseline pin for
the existing napl.track scenarios. Pure numpy."""

import numpy as np
import pytest
from napl.threats import LABELS, NAMES, build, gen
from napl.threats.scenes import LABEL_OF, spawn
from napl.track.scenarios import NAMES as OLD_NAMES
from napl.track.scenarios import build as old_build

# (truth.sum(), sum of all measurement values, frames) of napl.track.scenarios.build(name, 1) on main 0655e83
OLD_PINS = {
    "cv": (30752.999999999993, 113772.31336945247, 201),
    "turn": (21836.604285850903, 122640.88408461244, 201),
    "ca": (25212.59999999997, 165910.8256555246, 161),
    "cross": (8551.533966775874, 20243.56867181072, 201),
    "swarm5": (70303.82837139835, 251596.2279916157, 151),
    "cam": (10579.899999999945, 718586.741017949, 241),
}


def test_existing_scenarios_unchanged():
    assert tuple(OLD_NAMES) == tuple(OLD_PINS)
    for n, (ts, zs, nf) in OLD_PINS.items():
        s = old_build(n, 1)
        assert len(s.frames) == nf
        assert s.truth.sum() == pytest.approx(ts, rel=1e-9)
        assert sum(f[2].sum() for f in s.frames) == pytest.approx(zs, rel=1e-9)


@pytest.mark.parametrize(
    "name", ["route_hover", "swarm_split", "clutter_mix", "random", "close_cam_lidar"]
)
def test_deterministic_in_name_seed(name):
    a, b, c = build(name, 3), build(name, 3), build(name, 4)
    assert np.array_equal(a.truth, b.truth) and np.array_equal(a.alive, b.alive)
    assert all(np.array_equal(f[2], g[2]) for f, g in zip(a.frames, b.frames))
    assert a.truth.shape != c.truth.shape or not np.array_equal(a.truth, c.truth)


def test_lidar_variant_shares_truth():
    a, b = build("close_cam", 2), build("close_cam_lidar", 2)
    assert np.array_equal(a.truth, b.truth) and b.lidar and not a.lidar


def test_labels_and_shapes_all_scenes():
    for name in NAMES:
        s = build(name, 1)
        n, T = len(s.ids), len(s.frames)
        assert (
            s.truth.shape == (T, n, 6)
            and s.alive.shape == (T, n)
            and s.diam.shape == (n,)
        )
        assert len(s.labels) == len(s.kinds) == n and set(s.labels) <= set(LABELS)
        assert s.alive.any(0).all(), "every object exists at some time"
        assert all(f[2].shape[1] == f[1].dim for f in s.frames)
        assert np.isfinite(s.truth).all()


def test_label_classes_covered():
    assert set(LABEL_OF.values()) == set(LABELS)
    assert {"drone", "balloon", "hanging", "bird", "debris"} <= {
        l for n in NAMES for l in build(n, 1).labels
    }


def test_in_scope_speed_and_range_bounds():
    """No generator, in its random mode, exceeds 16 m/s or leaves the backyard-scale volume; accelerations of multirotors stay bounded."""
    lim = {"route": 3.5 * 1.2, "route_fast": 3.5 * 2.0, "survey": 3.6}
    for kind in LABEL_OF:
        for seed in range(8):
            rng = np.random.default_rng(seed)
            for o in spawn(kind, rng, 20.0, {"n": 6} if kind == "swarm" else None):
                sp = np.linalg.norm(o["V"], axis=1)
                assert sp.max() <= 16.0, (kind, seed, sp.max())
                assert np.linalg.norm(o["P"], axis=1).max() < 120.0
                assert o["P"][:, 2].min() > -0.5
                if (
                    kind in lim
                ):  # point-mass multirotor acceleration limit (PX4-like, <= 2x stock)
                    a = np.linalg.norm(np.diff(o["V"][::5], axis=0), axis=1) / 0.05
                    assert a.max() <= lim[kind] * 2.0, (kind, seed, a.max())


def test_pennycuick_wingbeat_scale():
    m, b, S = gen.SPECIES["pigeon"][:3]
    assert 4.0 < gen.wingbeat_hz(m, b, S) < 9.0  # pigeon-size birds beat at several Hz
    m, b, S = gen.SPECIES["gull"][:3]
    assert gen.wingbeat_hz(m, b, S) < gen.wingbeat_hz(
        *gen.SPECIES["pigeon"][:3]
    )  # bigger bird, slower


def test_birth_death_and_gaps_in_mixed_scene():
    s = build("busy", 1)
    assert (~s.alive).any() and s.alive.any(0).all()


def test_discrimination_smoke():
    from napl.threats.discrim import report

    r = report(range(1, 41), range(1001, 1021), 0.1)
    assert r["C"].sum() == r["n_test"] > 20
    assert r["acc"] > 0.5  # chance is 0.2
