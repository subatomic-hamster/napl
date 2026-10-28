"""Dataset of tracker-output histories for learned motion prediction (napl.track.scenarios only, no new threat profiles).

One sample = one confirmed IMM2 track matched (GOSPA cut-off, one-to-one) to a truth target at scan k:
  hm (N, 6)   last N track means [p, v] (oldest first; short histories are padded with the oldest entry), hn = valid count
  hs (N, 6)   their per-component std (sqrt diag of the 6x6 covariance)
  hu (N, 2)   IMM2 mode probabilities per scan;  hq (N,) NIS of the update at each scan (nan when none yet)
  cc (21,)    upper triangle of the current 6x6 covariance
  pm2 (H, 3), pc2 (H, 6)   IMM2 extrapolation (position mean, 3x3 covariance as xx yy zz xy xz yz) at horizons HORIZONS
  pm3, pc3    the same from IMM3 (a second tracker on the same detections; sample kept only if it also has a matched track)
  y (H, 3)    truth position at those horizons;  meta = [scenario index, seed, k, truth id, dt]
The truth at the longest horizon must exist, so the last 1 s of each scenario yields no samples."""

import numpy as np

from ..assign import lsa
from ..scenarios import NAMES, build
from ..variants import make_tracker

HORIZONS = (0.2, 0.4, 0.6, 0.8, 1.0)
NHIST = 10
IU = np.triu_indices(6)
P3 = ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))


def _tri(P):
    return np.array([P[i, j] for i, j in P3])


def _match(truth3, tracks, c):
    """{truth index: track index}, one-to-one nearest within c."""
    if not len(tracks) or not len(truth3):
        return {}
    est = np.array([tr.mean[:3] for tr in tracks])
    D = np.linalg.norm(truth3[:, None] - est[None], axis=-1)
    return {int(i): int(j) for i, j in zip(*lsa(np.minimum(D, c))) if D[i, j] < c}


def _extrap(tr, t, steps, dt):
    """IMM extrapolation of a track from its time t to t + s dt: (H, 3) means, (H, 6) covariances."""
    ms, cs = [], []
    for s in steps:
        m, P = tr.predict_to(t + s * dt)
        ms.append(m[:3])
        cs.append(_tri(P[:3, :3]))
    return np.array(ms), np.array(cs)


# Generator groups of the threat suite (leave-one-generator-out unit); KIND_GROUP maps napl.threats generator kind -> group.
GROUPS = ("route", "pattern", "weave", "swarm", "balloon", "hanging", "bird", "debris")
KIND_GROUP = {
    "route": "route",
    "route_fast": "route",
    "orbit": "pattern",
    "survey": "pattern",
    "weave": "weave",
    "swarm": "swarm",
    "balloon": "balloon",
    "hanging": "hanging",
    "bird": "bird",
    "leaf": "debris",
    "bag": "debris",
}


def collect(name, seed, n_hist=NHIST, horizons=HORIZONS, suite="track", stride=1):
    """All samples of one (scenario, seed) run -> dict of arrays (empty arrays if none).
    suite="threats": napl.threats scene (alive mask honoured: only existing objects are matched, truth must exist at the longest
    horizon); meta gets two extra columns [GROUPS index of the object's generator, LABELS index]. stride keeps every stride-th scan."""
    if suite == "threats":
        from ...threats import LABELS
        from ...threats import NAMES as TN
        from ...threats import build as tbuild

        scn, names = tbuild(name, seed), TN
    else:
        scn, names = build(name, seed), NAMES
    steps = [int(round(h / scn.dt)) for h in horizons]
    t2, t3 = make_tracker("imm2-gnn", scn, seed), make_tracker("imm3-gnn", scn, seed)
    buf, out = (
        {},
        {
            k: []
            for k in (
                "hm",
                "hs",
                "hu",
                "hq",
                "hn",
                "cc",
                "pm2",
                "pc2",
                "pm3",
                "pc3",
                "y",
                "meta",
            )
        },
    )
    T = len(scn.frames)
    for k, (t, meas, zs) in enumerate(scn.frames):
        c2, c3 = t2.step(t, zs, meas), t3.step(t, zs, meas)
        for tr in c2:
            m, P = tr.est.pv()
            b = buf.setdefault(tr.id, [])
            b.append((m.copy(), np.sqrt(np.diag(P)), tr.est.mu.copy(), tr.est.last_nis))
            del b[:-n_hist]
        if k + max(steps) >= T:
            continue
        if k % stride:
            continue
        truth3 = scn.truth[k, :, :3]
        if suite == "threats":  # dead objects can never match (distance > cut-off)
            truth3 = np.where(scn.alive[k][:, None], truth3, 1e9)
        m2, m3 = _match(truth3, c2, scn.c), _match(truth3, c3, scn.c)
        for i, j in m2.items():
            if i not in m3 or (suite == "threats" and not scn.alive[k + max(steps), i]):
                continue
            b = buf[c2[j].id]
            pad = [b[0]] * (n_hist - len(b)) + b
            pm2, pc2 = _extrap(c2[j], t, steps, scn.dt)
            pm3, pc3 = _extrap(c3[m3[i]], t, steps, scn.dt)
            out["hm"].append([x[0] for x in pad])
            out["hs"].append([x[1] for x in pad])
            out["hu"].append([x[2] for x in pad])
            out["hq"].append([x[3] for x in pad])
            out["hn"].append(len(b))
            out["cc"].append(c2[j].cov[IU])
            out["pm2"].append(pm2), out["pc2"].append(pc2)
            out["pm3"].append(pm3), out["pc3"].append(pc3)
            out["y"].append(np.array([scn.truth[k + s, i, :3] for s in steps]))
            meta = [names.index(name), seed, k, scn.ids[i], scn.dt]
            if suite == "threats":
                meta += [
                    GROUPS.index(KIND_GROUP[scn.kinds[i]]),
                    LABELS.index(scn.labels[i]),
                ]
            out["meta"].append(meta)
    dt = {"meta": np.float64, "hn": np.int16}
    return {k: np.array(v, dt.get(k, np.float32)) for k, v in out.items()}


def _job(a):
    return collect(*a)


def build_dataset(splits, names=NAMES, jobs=2, suite="track", stride=1, extra=None):
    """splits {split: seeds} -> {split: dict of concatenated arrays}. jobs = worker processes.
    extra {name: k}: scene names run with k times as many seeds (the randomised families): seed + j * 10000, j < k, which stays
    inside the split's range band (x1001-x1160 / x2001-x2040 / x3001-x3060 are disjoint across splits)."""
    from multiprocessing import Pool

    extra = extra or {}
    todo = [
        (s, n, sd + j * 10_000)
        for s, seeds in splits.items()
        for n in names
        for sd in seeds
        for j in range(extra.get(n, 1))
    ]
    with Pool(jobs) as p:
        res = p.map(
            _job,
            [(n, sd, NHIST, HORIZONS, suite, stride) for _, n, sd in todo],
            chunksize=1,
        )
    ds = {}
    for s in splits:
        parts = [r for (s2, _, _), r in zip(todo, res) if s2 == s and len(r["y"])]
        ds[s] = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    return ds


AUG_KEYS = (
    "hm",
    "hs",
    "hu",
    "hq",
    "hn",
    "cc",
    "pm2",
    "pc2",
    "y",
    "meta",
)  # what training reads


def _tri_of(C):
    return np.stack([C[..., i, j] for i, j in P3], -1)


def augment(d, rng, scale=(0.7, 1.4)):
    """One augmented copy of a split (keys AUG_KEYS only): every sample gets a random yaw (rotation about the vertical axis; the
    positions, velocities, 6x6 and 3x3 covariances and truth rotate together) and a random uniform world scale s (log-uniform in
    `scale`: positions and velocities x s, covariances x s^2, i.e. a faster or slower target on a larger or smaller range at the
    same sensor rate). Approximations: the per-scan stds hs of x and y are rotated as if x and y were uncorrelated, and the
    scaled IMM covariance is not what the tracker would output on a scaled scene."""
    S = len(d["y"])
    a = rng.uniform(0, 2 * np.pi, S)
    s = np.exp(rng.uniform(np.log(scale[0]), np.log(scale[1]), S))
    c, sn = np.cos(a), np.sin(a)
    R = np.zeros((S, 3, 3))
    R[:, 0, 0], R[:, 0, 1], R[:, 1, 0], R[:, 1, 1], R[:, 2, 2] = c, -sn, sn, c, 1.0
    R6 = np.zeros((S, 6, 6))
    R6[:, :3, :3] = R6[:, 3:, 3:] = R
    sc = lambda x, p: x * (s**p).reshape((-1,) + (1,) * (x.ndim - 1))
    vec = lambda x: sc(np.einsum("sij,s...j->s...i", R, x.astype(np.float64)), 1)
    out = {k: d[k] for k in ("hu", "hq", "hn", "meta")}
    hm = d["hm"].astype(np.float64)
    out["hm"] = np.concatenate([vec(hm[..., :3]), vec(hm[..., 3:])], -1)
    hs = d["hs"].astype(np.float64)
    hv = np.empty_like(hs)
    for o in (0, 3):  # rotated x/y variances of the diagonal stds
        vx, vy = hs[..., o] ** 2, hs[..., o + 1] ** 2
        cc_, ss_ = c[:, None] ** 2, sn[:, None] ** 2
        hv[..., o], hv[..., o + 1] = cc_ * vx + ss_ * vy, ss_ * vx + cc_ * vy
        hv[..., o + 2] = hs[..., o + 2] ** 2
    out["hs"] = sc(np.sqrt(hv), 1)
    iu = np.triu_indices(6)
    P = np.zeros((S, 6, 6))
    P[:, iu[0], iu[1]] = d["cc"]
    P = P + np.swapaxes(P, 1, 2) - P * np.eye(6)[None]
    P = sc(R6 @ P @ np.swapaxes(R6, 1, 2), 2)
    out["cc"] = P[:, iu[0], iu[1]]
    out["pm2"] = vec(d["pm2"])
    out["y"] = vec(d["y"])
    C3 = np.zeros(d["pc2"].shape[:-1] + (3, 3))
    for k, (i, j) in enumerate(P3):
        C3[..., i, j] = C3[..., j, i] = d["pc2"][..., k]
    C3 = sc(R[:, None] @ C3 @ np.swapaxes(R, 1, 2)[:, None], 2)
    out["pc2"] = _tri_of(C3)
    return {k: v.astype(d[k].dtype) for k, v in out.items()}


def save(path, ds):
    np.savez_compressed(
        path, **{f"{s}/{k}": v for s, d in ds.items() for k, v in d.items()}
    )


def load(path):
    """path may be several files joined by commas (one per split, see ml_predict.py gen --splits)."""
    ds = {}
    for p in str(path).split(","):
        z = np.load(p)
        for key in z.files:
            s, k = key.split("/")
            ds.setdefault(s, {})[k] = z[key]
    return ds
