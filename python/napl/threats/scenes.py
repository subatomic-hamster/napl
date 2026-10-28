"""Scenario suite of generic test targets: build(name, seed) -> ThreatScenario (Scenario shape of napl.track plus labels, alive mask,
object sizes). Deterministic in (name, seed): truth and measurements use separate generators, so `<name>_lidar` has the same truth
as `<name>`. See docs/cuas2/threat-scenarios.md."""

from dataclasses import dataclass, field

import numpy as np

from ..track.models import CameraModel, PositionModel
from ..track.scenarios import Scenario
from . import gen
from .gen import DT, SUB, _pt

LABELS = ("drone", "balloon", "hanging", "bird", "debris")
DIAM = {
    "drone": (0.25, 0.6),
    "balloon": (0.3, 0.9),
    "hanging": (0.1, 0.4),
    "bird": (0.2, 0.5),
    "debris": (0.05, 0.4),
}  # m [est]
LABEL_OF = {
    "route": "drone",
    "route_fast": "drone",
    "orbit": "drone",
    "survey": "drone",
    "weave": "drone",
    "swarm": "drone",
    "balloon": "balloon",
    "hanging": "hanging",
    "bird": "bird",
    "leaf": "debris",
    "bag": "debris",
}
BOX_CLOSE = np.array([[3.5, -2.5, 2.0], [6.5, 2.5, 4.0]])  # within TF-Luna range (8 m)
LIDAR_MAX, LIDAR_SIG, LIDAR_BEAM = (
    8.0,
    0.015,
    np.radians(2.0),
)  # TF-Luna-like: 0.2-8 m, 1-2 cm noise [est], ~2 deg beam


@dataclass
class ThreatScenario(Scenario):
    labels: list = field(default_factory=list)  # per truth object: one of LABELS
    kinds: list = field(default_factory=list)  # generator kind per object
    alive: np.ndarray = None  # (T, n) bool: object exists (birth/death); truth rows outside are not scored
    diam: np.ndarray = None  # (n,) physical size (m)
    sensor: str = "pos"
    lidar: bool = False


class CameraLidar(CameraModel):
    """Size-range camera whose range channel is a boresight TF-Luna-like rangefinder while the target is inside its beam and
    within LIDAR_MAX (std LIDAR_SIG); elsewhere the size-based range (std range_rel * r)."""

    def R(self, x):
        d = x[:3] - self.o
        r = np.linalg.norm(d)
        sr = self.range_rel * r
        if r <= LIDAR_MAX:
            # smooth hand-over (log-linear in the off-axis angle between 0.5 and 1.5 beam widths): partial beam overlap, and it keeps
            # the EKF's predicted-state noise consistent with the truth-state noise a hard switch would break
            th = np.arccos(np.clip(d @ self.fwd / max(r, 1e-9), -1, 1)) / LIDAR_BEAM
            w = float(np.clip(th - 0.5, 0.0, 1.0))
            sr = float(np.exp((1 - w) * np.log(LIDAR_SIG) + w * np.log(sr)))
        return np.diag([self.sp**2, self.sp**2, sr**2])


def spawn(kind, rng, T, over=None):
    """One generator kind with a randomised parameter set (override with `over`; over['box'] relocates it). -> list of objects."""
    over = dict(over or {})
    box = over.pop("box", None)
    if kind in ("route", "route_fast"):
        p = gen.route_params(
            rng,
            fast=kind == "route_fast",
            hover=over.pop("hover", True),
            **({"box": box} if box is not None else {}),
        )
        p.update(over)
        out = [gen.route(rng, T, **p)]
    elif kind == "survey":
        p = gen.survey_params(rng)
        p.update(over)
        out = [gen.survey(rng, T, **p)]
    elif kind == "swarm":
        p = gen.swarm_params(rng, over.pop("n", None))
        p.update(over)
        lp = p.pop("leader_params")
        out = gen.swarm(rng, T, leader=gen.route(rng, T, **lp), **p)
    else:
        f, pf = {
            "orbit": (gen.orbit, gen.orbit_params),
            "weave": (gen.weave, gen.weave_params),
            "balloon": (gen.balloon, lambda r: gen.balloon_params(r, T)),
            "hanging": (gen.hanging, gen.hanging_params),
            "bird": (gen.bird, gen.bird_params),
            "leaf": (gen.debris, lambda r: gen.debris_params(r, T, "leaf")),
            "bag": (gen.debris, lambda r: gen.debris_params(r, T, "bag")),
        }[kind]
        p = pf(rng)
        if box is not None:
            key = "anchor" if kind == "hanging" else "c" if kind == "orbit" else "p0"
            p[key] = _pt(rng, box)
        p.update(over)
        out = [f(rng, T, **p)]
    lab = LABEL_OF[kind]
    return [
        dict(
            label=lab,
            kind=kind,
            P=P,
            V=V,
            diam=float(rng.uniform(*DIAM[lab])),
            t0=0.0,
            t1=T,
            gaps=[],
        )
        for P, V in out
    ]


def _window(rng, o, T, win):
    if win == "rand":
        o["t0"] = float(rng.uniform(0, 0.35 * T)) if rng.random() < 0.6 else 0.0
        o["t1"] = float(rng.uniform(0.65 * T, T)) if rng.random() < 0.6 else T
    elif win:
        o["t0"], o["t1"] = win


def _gaps(rng, o, T, rate):
    for _ in range(rng.poisson(rate)):
        a = rng.uniform(o["t0"], max(o["t1"] - 3, o["t0"] + 0.1))
        o["gaps"].append((a, a + rng.uniform(1.0, 3.0)))


# name -> (objects [(kind, over, win)], sensor options)
POS = dict(sensor="pos", pd=0.9, ncl=5.0, c=2.0, occl=0.0)
CAM = dict(sensor="cam", pd=0.9, ncl=1.0, c=4.0, occl=0.0)
SUITE = {
    "route_hover": ([("route", {}, None)], POS),
    "route_fast": ([("route_fast", {}, None)], POS),
    "orbit": ([("orbit", {}, None)], POS),
    "survey": ([("survey", {}, None)], POS),
    "weave": ([("weave", {}, None)], POS),
    "swarm_v": (
        [("swarm", {"n": 6, "kind0": "v", "kind1": "line", "split": None}, None)],
        dict(POS, ncl=10.0),
    ),
    "swarm_split": (
        [
            (
                "swarm",
                {"n": 8, "kind0": "grid", "kind1": None, "split": (5.0, 13.0, 9.0, 3)},
                None,
            )
        ],
        dict(POS, ncl=10.0),
    ),
    "balloons": ([("balloon", {}, None)] * 3, POS),
    "hanging": ([("hanging", {}, None)] * 2, POS),
    "birds": ([("bird", {}, None)] * 3 + [("route", {}, None)], POS),
    "clutter_mix": (
        [("route", {}, "rand"), ("balloon", {}, "rand"), ("bird", {}, "rand")]
        + [("leaf", {}, "rand"), ("bag", {}, "rand"), ("bag", {}, "rand")],
        dict(POS, ncl=8.0, occl=0.8, pd=0.85),
    ),
    "busy": (
        [
            ("weave", {}, "rand"),
            ("weave", {}, "rand"),
            ("swarm", {"n": 4}, "rand"),
            ("bird", {}, "rand"),
            ("bird", {}, "rand"),
            ("balloon", {}, "rand"),
            ("hanging", {}, None),
        ],
        dict(POS, ncl=10.0, occl=0.6, pd=0.85),
    ),
    "cam_mix": (
        [("route", {}, "rand"), ("bird", {}, "rand"), ("balloon", {}, "rand")],
        dict(CAM, occl=0.5),
    ),
    "close_cam": (
        [
            ("hanging", {"box": BOX_CLOSE}, None),
            ("route", {"box": BOX_CLOSE, "vmax": 2.0}, None),
        ],
        dict(CAM, c=2.0),
    ),
}
NAMES = tuple(SUITE) + ("random", "random_cam")
FUSION = ("close_cam",)  # also available as "<name>_lidar"


def random_objs(rng, T, max_obj=5):
    """Random family: 1..max_obj objects, label uniform over LABELS, kind and every parameter randomised, random windows/gaps."""
    by = {
        "drone": ["route", "route_fast", "orbit", "survey", "weave", "swarm"],
        "balloon": ["balloon"],
        "hanging": ["hanging"],
        "bird": ["bird"],
        "debris": ["leaf", "bag"],
    }
    objs = []
    for _ in range(int(rng.integers(1, max_obj + 1))):
        lab = LABELS[int(rng.integers(5))]
        kind = by[lab][int(rng.integers(len(by[lab])))]
        for o in spawn(
            kind, rng, T, {"n": int(rng.integers(3, 6))} if kind == "swarm" else None
        ):
            _window(rng, o, T, "rand" if rng.random() < 0.4 else None)
            objs.append(o)
    return objs


def build(name, seed, T=20.0):
    lidar = name.endswith("_lidar")
    base = name[:-6] if lidar else name
    rng = np.random.default_rng([sum(map(ord, base)), seed])  # truth
    mrng = np.random.default_rng([sum(map(ord, base)) + 1, seed])  # measurements
    if base in ("random", "random_cam"):
        o = POS if base == "random" else CAM
        o = dict(
            o,
            pd=float(rng.uniform(0.8, 0.95)),
            ncl=float(rng.uniform(2, 10)),
            occl=float(rng.uniform(0, 0.8)),
        )
        objs = random_objs(rng, T)
    else:
        spec, o = SUITE[base]
        objs = []
        for kind, over, win in spec:
            for ob in spawn(kind, rng, T, over):
                _window(rng, ob, T, win)
                objs.append(ob)
    for ob in objs:
        _gaps(rng, ob, T, o["occl"])
    return assemble(name, objs, T, mrng, o, lidar)


def assemble(name, objs, T, mrng, o, lidar=False):
    n = len(objs)
    P = np.stack([ob["P"][::SUB] for ob in objs], 1)
    V = np.stack([ob["V"][::SUB] for ob in objs], 1)
    truth = np.concatenate([P, V], 2)
    Tn = len(P)
    t = np.arange(Tn) * DT
    alive = np.stack([(t >= ob["t0"]) & (t <= ob["t1"]) for ob in objs], 1)
    vis = alive.copy()
    for i, ob in enumerate(objs):
        for a, b in ob["gaps"]:
            vis[(t >= a) & (t <= b), i] = False
    pd, ncl, cam = o["pd"], o["ncl"], o["sensor"] == "cam"
    frames = []
    if cam:
        lam = ncl / (1920 * 1080 * 35.0)
        cls = CameraLidar if lidar else CameraModel
        pan = tilt = 0.0
        prev = None
        for k in range(Tn):
            kp = max(
                k - 1, 0
            )  # head follows the alive objects' centroid with one frame of lag
            a = alive[kp] if alive[kp].any() else alive[k]
            if a.any():
                d0 = P[kp][a].mean(0)
                pan, tilt = (
                    np.arctan2(d0[1], d0[0]),
                    np.arctan2(d0[2], np.hypot(d0[0], d0[1])),
                )
            m = cls((0, 0, 0), pan, tilt, size_range=True)
            zs = [
                m.sample(mrng, P[k, i])
                for i in range(n)
                if vis[k, i] and mrng.random() < pd and m.in_view(P[k, i])
            ]
            for _ in range(mrng.poisson(ncl)):
                zs.append(
                    np.array(
                        [
                            mrng.uniform(0, 1920),
                            mrng.uniform(0, 1080),
                            mrng.uniform(5, 40),
                        ]
                    )
                )
            frames.append((k * DT, m, np.array(zs).reshape(-1, 3)))
    else:
        pm = PositionModel(0.5)
        lo, hi = (
            P.reshape(-1, 3).min(0) - [10, 10, 5],
            P.reshape(-1, 3).max(0) + [10, 10, 5],
        )
        lam = ncl / float(np.prod(hi - lo))
        for k in range(Tn):
            zs = [
                pm.sample(mrng, P[k, i])
                for i in range(n)
                if vis[k, i] and mrng.random() < pd
            ]
            zs += list(mrng.uniform(lo, hi, size=(mrng.poisson(ncl), 3)))
            mrng.shuffle(zs)
            frames.append((k * DT, pm, np.array(zs).reshape(-1, 3)))
    return ThreatScenario(
        name,
        DT,
        list(range(n)),
        truth,
        frames,
        lam,
        pd,
        o["c"],
        ncl,
        [ob["label"] for ob in objs],
        [ob["kind"] for ob in objs],
        alive,
        np.array([ob["diam"] for ob in objs]),
        o["sensor"],
        lidar,
    )
