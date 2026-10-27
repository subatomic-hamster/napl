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


def collect(name, seed, n_hist=NHIST, horizons=HORIZONS):
    """All samples of one (scenario, seed) run -> dict of arrays (empty arrays if none)."""
    scn = build(name, seed)
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
        truth3 = scn.truth[k, :, :3]
        m2, m3 = _match(truth3, c2, scn.c), _match(truth3, c3, scn.c)
        for i, j in m2.items():
            if i not in m3:
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
            out["meta"].append([NAMES.index(name), seed, k, scn.ids[i], scn.dt])
    dt = {"meta": np.float64, "hn": np.int16}
    return {k: np.array(v, dt.get(k, np.float32)) for k, v in out.items()}


def _job(a):
    return collect(*a)


def build_dataset(splits, names=NAMES, jobs=2):
    """splits {split: seeds} -> {split: dict of concatenated arrays}. jobs = worker processes (the box rule is at most 2)."""
    from multiprocessing import Pool

    todo = [(s, n, sd) for s, seeds in splits.items() for sd in seeds for n in names]
    with Pool(jobs) as p:
        res = p.map(_job, [(n, sd) for _, n, sd in todo], chunksize=2)
    ds = {}
    for s in splits:
        parts = [r for (s2, _, _), r in zip(todo, res) if s2 == s and len(r["y"])]
        ds[s] = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    return ds


def save(path, ds):
    np.savez_compressed(
        path, **{f"{s}/{k}": v for s, d in ds.items() for k, v in d.items()}
    )


def load(path):
    z, ds = np.load(path), {}
    for key in z.files:
        s, k = key.split("/")
        ds.setdefault(s, {})[k] = z[key]
    return ds
