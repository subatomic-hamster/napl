"""Named tracker variants (filter x motion model(s) x association) used by the benchmark and by integration code, and run()
which feeds one scenario to one variant and returns the metric summary plus CPU time per update."""

import time

import numpy as np

from .filters import EKF, IMM, UKF, ParticleFilter
from .metrics import Evaluator
from .models import CA, CT, CV, Singer
from .scenarios import build
from .trackers import GNN, JPDA, MHT, PHD, Tracker  # noqa: F401

VSIG = 10.0  # prior velocity std (m/s) of a track born from one measurement


def prior(model, p0, P0, vsig=VSIG):
    """Lift a position prior to the model's state (zero velocity/accel/turn rate, prior variances)."""
    x = np.zeros(model.dim)
    x[:3] = p0
    P = np.zeros((model.dim, model.dim))
    P[:3, :3] = P0
    P[3:6, 3:6] = np.eye(3) * vsig**2
    P[6:, 6:] = np.diag(model.pad_P)
    return x, P


def _modes():
    return CV(0.3), Singer(sigma_a=4.0, tau=3.0), CT(q=0.3, q_w=0.05)


def ekf(model, cls=EKF):
    return lambda p0, P0: cls(model, *prior(model, p0, P0))


def imm(kinds):
    T2 = np.array([[0.95, 0.05], [0.10, 0.90]])
    T3 = np.array([[0.94, 0.03, 0.03], [0.10, 0.90, 0.0], [0.10, 0.0, 0.90]])
    cv, sg, ct = _modes()
    ms = {"2": [cv, sg], "3": [cv, sg, ct]}[kinds]
    return lambda p0, P0: IMM(
        [EKF(m, *prior(m, p0, P0)) for m in ms], T2 if kinds == "2" else T3
    )


class AlphaBeta:
    """Baseline in the spirit of the repo's cuas Tracker: alpha-beta position/velocity, fixed Euclidean gate, no covariance."""

    sig = 1.5

    def __init__(self, p0, a=0.6, b=0.12):
        self.x, self.a, self.b, self.dt = np.r_[p0, np.zeros(3)], a, b, 0.1

    def copy(self):
        o = object.__new__(type(self))
        o.__dict__.update(self.__dict__)
        o.x = self.x.copy()
        return o

    def predict(self, dt):
        self.dt = dt
        self.x[:3] += self.x[3:] * dt

    def gate_d2(self, zs, meas):
        return ((np.atleast_2d(zs) - self.x[:3]) ** 2).sum(1) / self.sig**2

    def lik_each(self, zs, meas):
        return np.exp(-0.5 * self.gate_d2(zs, meas)) / (2 * np.pi * self.sig**2) ** 1.5

    def update(self, z, meas):
        nu = z - self.x[:3]
        self.x[:3] += self.a * nu
        self.x[3:] += self.b / self.dt * nu

    def pv(self):
        return self.x, np.diag([1.0] * 3 + [4.0] * 3)


class PFThenIMM:
    """Young-track estimator: a bootstrap particle filter for the first `hand` updates, then an IMM-2 seeded from its moments."""

    def __init__(self, pf, mk_imm, hand=3):
        self.cur, self.mk, self.hand, self.n = pf, mk_imm, hand, 0

    def copy(self):
        o = object.__new__(type(self))
        o.__dict__.update(self.__dict__)
        o.cur = self.cur.copy()
        return o

    def predict(self, dt):
        self.cur.predict(dt)

    def gate_d2(self, zs, meas):
        return self.cur.gate_d2(zs, meas)

    def lik_each(self, zs, meas):
        return self.cur.lik_each(zs, meas)

    def pv(self):
        return self.cur.pv()

    def update(self, z, meas):
        lik = self.cur.update(z, meas)
        self.n += 1
        if self.n == self.hand and isinstance(self.cur, ParticleFilter):
            x, P = self.cur.pv()
            self.cur = self.mk(
                x[:3], P[:3, :3]
            )  # fresh IMM at the PF mean, velocity cov from the cloud below
            for m in self.cur.modes:
                m.x[3:6], m.P[3:6, 3:6] = x[3:6], P[3:6, 3:6] + np.eye(3) * 0.01
                m.P[:3, :3] = P[:3, :3]
        return lik


def factory(variant, seed):
    rng = np.random.default_rng(seed + 7919)
    cv, sg, ct = _modes()
    if variant == "ab-nn":
        return GNN, (lambda p0, P0: AlphaBeta(p0))
    if variant == "ekf-cv-gnn":
        return GNN, ekf(cv)
    if variant == "ekf-ca-gnn":
        return GNN, ekf(CA(2.0))
    if variant == "ukf-ct-gnn":
        return GNN, ekf(ct, UKF)
    if variant == "imm2-gnn":
        return GNN, imm("2")
    if variant == "imm3-gnn":
        return GNN, imm("3")
    if (
        variant == "imm2-gnn-s"
    ):  # imm2-gnn with the strict hit rule the tuned JPDA/MHT use
        return GNN, imm("2")
    if variant == "imm2-pf-gnn":
        mk = imm("2")

        def f(p0, P0):
            pf = ParticleFilter.init_gauss(CV(0.3), p0, P0, VSIG, 200, rng)
            pf.q_scale = 4.0
            return PFThenIMM(pf, mk)

        return GNN, f
    if variant == "imm2-jpda":
        return JPDA, imm("2")
    if variant == "imm2-mht":
        return MHT, imm("2")
    if variant == "phd-singer":
        return PHD, ekf(sg)
    raise ValueError(variant)


# Tracker options of the tuned variants (docs/cuas2/tracking-bench.md, "Why JPDA / MHT / PHD underperformed"). Everything not listed
# keeps the Tracker defaults, so the old behaviour of a variant is its KW entry replaced by the defaults of its class.
KW = {
    "imm2-gnn-s": dict(
        min_llr=3.5
    ),  # ~ W > 33 x missed-detection weight, the same strictness as the hit rules below
    "imm2-jpda": dict(dedupe=0, hit_beta0=0.01),
    "imm2-mht": dict(kmax=8, nscan=1, decay=0.8, hit_llr=3.5, joint=1, stick=4.0),
    "phd-singer": dict(xmerge=1, split_w=0.7, keep=1, merge_d2=9.0),
}

VARIANTS = (
    "ab-nn",
    "ekf-cv-gnn",
    "ekf-ca-gnn",
    "ukf-ct-gnn",
    "imm2-gnn",
    "imm2-gnn-s",
    "imm3-gnn",
    "imm2-pf-gnn",
    "imm2-jpda",
    "imm2-mht",
    "phd-singer",
)


def make_tracker(variant, scn, seed=0, **opts):
    """Tracker of a named variant for scenario scn; opts override the variant's KW entry (e.g. m=4, hit_beta0=0.5)."""
    cls, mk = factory(variant, seed)
    return cls(mk, pd=scn.pd, lam=scn.lam, **{**KW.get(variant, {}), **opts})


def run(scn_name, variant, seed, scn=None, **opts):
    """One Monte Carlo run -> dict of metrics (see Evaluator.summary) + cpu_ms (mean wall ms per update). opts: tracker options."""
    scn = scn or build(scn_name, seed)
    if variant == "ab-nn" and scn_name == "cam":
        return None  # baseline works on position measurements only
    tr = make_tracker(variant, scn, seed, **opts)
    ev, dts = Evaluator(scn.c), []
    for k, (t, meas, zs) in enumerate(scn.frames):
        t0 = time.perf_counter()
        tracks = tr.step(t, zs, meas)
        dts.append(time.perf_counter() - t0)
        ev.add(t, scn.ids, scn.truth[k], tracks)
    s = ev.summary(scn.ids)
    s["cpu_ms"] = 1e3 * float(np.mean(dts))
    return s
