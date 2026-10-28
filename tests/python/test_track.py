"""napl.track: metrics known values, assignment, filter consistency, IMM on a turn, JPDA vs GNN, determinism. Pure numpy, small runs."""

import itertools

import numpy as np
import pytest
from napl.track import (
    CV,
    EKF,
    UKF,
    BearingModel,
    CameraModel,
    ParticleFilter,
    PositionModel,
    Track,
    gospa,
    nees,
    ospa,
    triangulate,
)
from napl.track.assign import lsa
from napl.track.filters import IMM
from napl.track.metrics import chi2_band
from napl.track.models import Singer
from napl.track.scenarios import _path, build
from napl.track.trackers import GNN, JPDA, MHT, _Leaf
from napl.track.variants import ekf, imm, make_tracker, prior, run


def test_gospa_known_values():
    c = 2.0
    z = np.zeros((1, 3))
    assert gospa(z, z, c)["total"] == pytest.approx(0.0)
    g = gospa(z, np.zeros((0, 3)), c)  # one missed target costs c^p / 2
    assert (
        g["total"] == pytest.approx(c / np.sqrt(2))
        and g["missed"] == g["total"]
        and g["false"] == 0
    )
    g = gospa(z, np.array([[1.0, 0, 0]]), c)  # assigned, d = 1 < c
    assert (
        g["total"] == pytest.approx(1.0)
        and g["loc"] == pytest.approx(1.0)
        and g["n_match"] == 1
    )
    g = gospa(z, np.array([[3.0, 0, 0]]), c)  # d >= c: one missed + one false = c^p
    assert (
        g["total"] == pytest.approx(c)
        and g["missed"] == pytest.approx(g["false"])
        and g["loc"] == 0
    )
    g = gospa(
        np.array([[0.0, 0, 0], [10, 0, 0]]), np.array([[0.5, 0, 0]]), c
    )  # 1 loc 0.5, 1 missed
    assert g["total"] ** 2 == pytest.approx(0.25 + 2.0)
    assert g["total"] ** 2 == pytest.approx(
        g["loc"] ** 2 + g["missed"] ** 2 + g["false"] ** 2
    )


def test_ospa_known_values():
    z = np.zeros((1, 3))
    assert ospa(z, np.zeros((0, 3)), 2.0) == 2.0
    assert ospa(np.zeros((2, 3)), z, 2.0) == pytest.approx(np.sqrt(4.0 / 2))
    assert ospa(z, np.array([[1.0, 0, 0]]), 2.0) == pytest.approx(1.0)


def test_lsa_numpy_matches_brute_force():
    rng = np.random.default_rng(0)
    for shape in [(4, 4), (3, 5), (5, 3), (1, 4)]:
        C = rng.random(shape)
        r, k = lsa(C, force_numpy=True)
        n = min(shape)
        best = (
            min(
                sum(C[i, p[i]] for i in range(shape[0]))
                for p in itertools.permutations(range(shape[1]), shape[0])
            )
            if shape[0] <= shape[1]
            else min(
                sum(C[p[j], j] for j in range(shape[1]))
                for p in itertools.permutations(range(shape[0]), shape[1])
            )
        )
        assert len(r) == n and C[r, k].sum() == pytest.approx(best)


def test_kf_consistency_nees_nis_joseph():
    """Linear-Gaussian CV truth: average NEES at a fixed step and average NIS over runs/steps sit in the chi-square band."""
    q, sig, dt, R, K = 0.5, 0.5, 0.1, 400, 40
    rng = np.random.default_rng(1)
    pm, model = PositionModel(sig), CV(q)
    ne, ni = [], []
    F, Lq = model.F(None, dt), np.linalg.cholesky(model.Q(None, dt))
    P0 = np.diag([4.0] * 3 + [25.0] * 3)
    for _ in range(R):
        x = np.sqrt(np.diag(P0)) * rng.normal(
            size=6
        )  # truth drawn from the filter's own prior and process noise
        f = EKF(model, np.zeros(6), P0)
        for _ in range(K):
            x = F @ x + Lq @ rng.normal(size=6)
            f.predict(dt)
            f.update(x[:3] + sig * rng.normal(size=3), pm)
            ni.append(f.last_nis)
            assert np.allclose(f.P, f.P.T) and np.linalg.eigvalsh(f.P).min() > 0
        ne.append(nees(x, f.x, f.P))
    lo, hi = chi2_band(6, R)
    assert lo < np.mean(ne) < hi, (np.mean(ne), lo, hi)
    lo, hi = chi2_band(3, len(ni), z=4)
    assert lo < np.mean(ni) < hi, (np.mean(ni), lo, hi)


def test_ukf_equals_ekf_on_linear_and_camera_nis_reasonable():
    model, pm = CV(0.5), PositionModel(0.4)
    x0, P0 = prior(model, np.array([1.0, 2, 3]), np.eye(3) * 0.16)
    a, b = EKF(model, x0, P0), UKF(model, x0, P0)
    rng = np.random.default_rng(2)
    for _ in range(10):
        z = rng.normal(size=3)
        a.predict(0.1)
        b.predict(0.1)
        a.update(z, pm)
        b.update(z, pm)
    assert np.allclose(a.x, b.x, atol=1e-6) and np.allclose(a.P, b.P, atol=1e-6)
    # camera with range-proportional size noise: EKF NIS (3 dof) near 3 on straight flight
    rng = np.random.default_rng(3)
    ni = []
    for _ in range(60):
        p, v = np.array([20.0, -8, 3]), np.array([0.0, 3, 0])
        cam = lambda p: CameraModel(
            (0, 0, 0),
            np.arctan2(p[1], p[0]),
            np.arctan2(p[2], np.hypot(p[0], p[1])),
            size_range=True,
        )
        m0 = cam(p)
        e = EKF(model, *prior(model, *m0.init(m0.sample(rng, p)), vsig=5.0))
        for k in range(30):
            p = p + v * 0.1
            e.predict(0.1)
            m = cam(p)
            e.update(m.sample(rng, p), m)
            if k > 5:
                ni.append(e.last_nis)
    assert 2.4 < np.mean(ni) < 3.6, np.mean(ni)


def test_imm_beats_cv_on_turn_onset():
    path = _path([0, 0, 10], [12, 0, 0], [(8, "cv"), (6, "ct", 0.5), (6, "cv")], 0.1)
    pm, errs = PositionModel(0.5), {"cv": [], "imm2": [], "imm3": []}
    cv_mk = ekf(CV(0.3))
    for seed in range(12):
        rng = np.random.default_rng(seed)
        ests = {
            "cv": cv_mk(path[0, :3], np.eye(3) * 0.25),
            "imm2": imm("2")(path[0, :3], np.eye(3) * 0.25),
            "imm3": imm("3")(path[0, :3], np.eye(3) * 0.25),
        }
        for e in ests.values():
            for m in getattr(e, "modes", [e]):
                m.x[3:6], m.P[3:6, 3:6] = path[0, 3:6], np.eye(3) * 0.25
        for k in range(1, 140):
            z = path[k, :3] + 0.5 * rng.normal(size=3)
            for n, e in ests.items():
                e.predict(0.1)
                e.update(z, pm)
                if 85 <= k < 140:  # from just after onset through the turn
                    errs[n].append(np.sum((e.pv()[0][:3] - path[k, :3]) ** 2))
    rm = {n: np.sqrt(np.mean(v)) for n, v in errs.items()}
    assert rm["imm2"] < 0.8 * rm["cv"] and rm["imm3"] < 0.8 * rm["cv"], rm


def test_jpda_splits_shared_measurement_gnn_does_not():
    """Two tracks, one measurement equidistant from both: JPDA shares it, GNN gives it to exactly one track."""
    pm = PositionModel(0.5)

    def mk(p):
        return EKF(
            CV(0.5), *prior(CV(0.5), np.array(p, float), np.eye(3) * 0.25, vsig=1.0)
        )

    z = np.array([[0.0, 0.0, 0.0]])
    mkr = lambda p0, P0: mk([0, 0, 0])
    j = JPDA(mkr, pd=0.9, lam=1e-3)
    j.tracks = [Track(1, mk([-0.4, 0, 0]), 0.0), Track(2, mk([0.4, 0, 0]), 0.0)]
    for tr in j.tracks:
        tr.confirmed = True
    G, L = j._gates(z, pm, 16.27)
    beta, beta0 = j._marginals(np.where(G, 0.9 * L / 1e-3, 0.0), G)
    assert (
        beta[0, 0] == pytest.approx(beta[1, 0], rel=1e-6) and beta[0, 0] < 0.6
    )  # shared, not winner-takes-all
    assert beta[0, 0] + beta[1, 0] <= 1.0 + 1e-9
    g = GNN(mkr, pd=0.9, lam=1e-3)
    g.tracks = [Track(1, mk([-0.4, 0, 0]), 0.0), Track(2, mk([0.4, 0, 0]), 0.0)]
    for tr in g.tracks:
        tr.confirmed = True
    g.step(0.0, z, pm)
    assert sorted(tr.misses for tr in g.tracks) == [
        0,
        1,
    ]  # one got the measurement, the other missed


def test_crossing_scenario_all_associators_hold_both_targets():
    for v in ("imm2-gnn", "imm2-jpda", "imm2-mht", "ekf-cv-gnn"):
        r = [run("cross", v, s) for s in (1, 2, 3)]
        assert (
            np.mean([x["lost"] for x in r]) == 0
            and np.mean([x["gospa"] for x in r]) < 1.3
        ), (v, r)


def test_mht_and_phd_track_single_target():
    for v in ("imm2-mht", "phd-singer"):
        r = run("cv", v, 1)
        assert r["gospa"] < 0.8 and r["lost"] == 0, (v, r)


OLD = {  # the options of JPDA / MHT / PHD before the tuning (weak hits count, duplicate deletion, one big label, ...)
    "imm2-jpda": dict(dedupe=9.0, hit_beta0=0.5),
    "imm2-mht": dict(kmax=4, nscan=3, decay=1.0, hit_llr=-1e9, joint=0, stick=0.0),
    "phd-singer": dict(xmerge=0, split_w=0.0, keep=0, merge_d2=4.0),
}


@pytest.mark.parametrize("v", sorted(OLD))
def test_tuned_variants_beat_their_old_options_on_the_clutter_swarm(v):
    """Failable: swarm5, seeds 1-3, paired. The tuned options must cut GOSPA by >= 15 % against the pre-tuning ones."""
    new = np.mean([run("swarm5", v, s)["gospa"] for s in (1, 2, 3)])
    old = np.mean([run("swarm5", v, s, **OLD[v])["gospa"] for s in (1, 2, 3)])
    assert new < 0.85 * old, (v, new, old)


def _mht_with_two_trees(**kw):
    """Two trees, two measurements (j = 0, 1) of scan 1: tree 1 scores 10 on z0 and 9 on z1, tree 2 scores 9.9 on z0 and 0 on z1."""
    m = MHT(None, **kw)

    def leaf(s, j, prev):
        return _Leaf(None, s, (True,), 0, ((0, prev), (1, j)), 3, True)

    m.trees = {
        1: [leaf(10, 0, 0), leaf(9, 1, 0)],
        2: [leaf(9.9, 0, 1), leaf(0.0, 1, 1)],
    }
    return m


def test_mht_joint_selection_is_the_best_assignment_greedy_is_not():
    """Failable: the greedy rule lets the best-scoring tree take the measurement the other tree needs (total 10), the joint
    assignment gives tree 1 z1 and tree 2 z0 (total 18.9)."""

    def pick(m):
        return {i: lf.path[-1][1] for i, lf in m._select(2).items()}

    assert pick(_mht_with_two_trees()) == {1: 0, 2: 1}
    assert pick(_mht_with_two_trees(joint=1)) == {1: 1, 2: 0}


def _switches(scene, seed, **opts):
    from napl.threats import bench

    scn = bench.build(scene, seed)
    tr = make_tracker("imm2-mht", scn, seed, **opts)
    ev, ids = bench.Evaluator(scn.c), np.array(scn.ids)
    for k, (t, meas, zs) in enumerate(scn.frames):
        a = scn.alive[k]
        ev.add(t, list(ids[a]), scn.truth[k][a], tr.step(t, zs, meas))
    return ev.summary(scn.ids)["switches"]


def test_mht_joint_hysteresis_cuts_id_switches_in_the_close_formation():
    """Failable: swarm_v seeds 2-4 (6 drones 0.6-3 m apart, 0.5 m noise): the greedy MHT hops between neighbours' measurements
    (82 vs 147 switches in the three runs when written)."""
    new = np.mean([_switches("swarm_v", s) for s in (2, 3, 4)])
    old = np.mean([_switches("swarm_v", s, joint=0, stick=0.0) for s in (2, 3, 4)])
    assert new < 0.75 * old, (new, old)


def test_jpda_hit_rule_needs_real_evidence():
    """A lone clutter point at the edge of a tentative track's gate must not count as a hit (beta0 ~ 0.4) but a close one must."""
    pm = PositionModel(0.5)
    cv = CV(0.5)
    mk = lambda: EKF(cv, *prior(cv, np.zeros(3), np.eye(3) * 0.25, vsig=1.0))
    for z, hit in (([0.1, 0.0, 0.0], True), ([2.0, 0.0, 0.0], False)):
        j = JPDA(lambda p0, P0: mk(), pd=0.9, lam=2e-3, hit_beta0=0.01)
        j.tracks = [Track(1, mk(), 0.0)]
        j.step(0.1, np.array([z]), pm)
        assert j.tracks[0].hits[-1] == hit, (z, list(j.tracks[0].hits))


def test_determinism_and_seed_sensitivity():
    a, b = run("swarm5", "imm2-gnn", 4), run("swarm5", "imm2-gnn", 4)
    assert {k: v for k, v in a.items() if k != "cpu_ms"} == {
        k: v for k, v in b.items() if k != "cpu_ms"
    }
    c = run("swarm5", "imm2-gnn", 5)
    assert c["gospa"] != a["gospa"]
    s1, s2 = build("cross", 7), build("cross", 7)
    assert all(np.array_equal(x[2], y[2]) for x, y in zip(s1.frames, s2.frames))


def test_pf_range_parameterised_init_and_triangulation():
    o1, o2, p = np.array([0.0, 0, 0]), np.array([10.0, 0, 0]), np.array([15.0, 8, 4])
    b1, b2 = BearingModel(o1, 1e-3, 1e-3), BearingModel(o2, 1e-3, 1e-3)
    z1, z2 = b1.h(p), b2.h(p)
    assert np.allclose(triangulate([b1, b2], [z1, z2]), p, atol=1e-9)
    rng = np.random.default_rng(0)
    o, d = b1.ray(z1)
    pf = ParticleFilter.init_ray(CV(0.3), o, d, 3.0, 60.0, 1e-3, 1.0, 3000, rng)
    pf.update(z2, b2)  # the second bearing collapses the range ambiguity
    assert np.linalg.norm(pf.pv()[0][:3] - p) < 1.5 and pf.spread() < 3.0


def test_track_predict_to_and_class_prob():
    e = EKF(CV(0.5), *prior(CV(0.5), np.zeros(3), np.eye(3)))
    e.x[3:6] = [2.0, 0, 0]
    tr = Track(1, e, 0.0)
    m, P = tr.predict_to(1.5)
    assert (
        np.allclose(m[:3], [3.0, 0, 0])
        and np.trace(P) > np.trace(tr.cov)
        and tr.t == 0.0
    )
    tr.update_class([0.9, 0.05, 0.05])
    assert tr.class_prob.argmax() == 0 and tr.class_prob.sum() == pytest.approx(1.0)


def test_singer_q_matches_cv_limit_and_imm_mixing_keeps_dims():
    # Singer with tiny correlation time -> velocity-level noise only through a; just check Q is symmetric PSD and IMM dims mix
    s = Singer(3.0, 2.0)
    Q = s.Q(np.zeros(9), 0.1)
    assert np.allclose(Q, Q.T) and np.linalg.eigvalsh(Q).min() > -1e-12
    m = imm("3")(np.zeros(3), np.eye(3))
    assert isinstance(m, IMM) and [x.model.dim for x in m.modes] == [6, 9, 7]
    m.predict(0.1)
    assert (
        m.pv()[0].shape == (6,)
        and np.isfinite(m.pv()[1]).all()
        and m.mu.sum() == pytest.approx(1.0)
    )
