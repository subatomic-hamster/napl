"""Multi-target trackers over the estimator interface of filters.py: GNN, JPDA, track-oriented MHT (N-scan), labelled GM-PHD.

Common call: tracker.step(t, zs, meas) -> confirmed Track list. zs is an (n, meas.dim) array of one scan, all produced by
the measurement model `meas` (it carries the sensor pose of that scan). make_est(p0, P0) builds a new estimator from the
position prior of one measurement (meas.init).
Track management is M-of-N confirmation (m hits within the last n scans), deletion after max_miss consecutive misses, birth from
measurements not explained by an existing track.
"""

from collections import deque

import numpy as np

from .assign import lsa
from .models import CHI2_GATE

BIG = 1e9


class Track:
    """What fire control reads: id, t, mean (6: p, v), cov (6x6), class_prob, confirmed, history; predict_to(t)."""

    def __init__(self, tid, est, t, n_classes=3, hist=300):
        self.id, self.est, self.t = tid, est, t
        self.t_birth, self.t_confirm, self.confirmed = t, None, False
        self.hits = deque(maxlen=16)
        self.misses, self.age = 0, 1
        self.class_prob = np.full(n_classes, 1.0 / n_classes)
        self.history = deque(maxlen=hist)  # (t, mean6)

    @property
    def mean(self):
        return self.est.pv()[0]

    @property
    def cov(self):
        return self.est.pv()[1]

    def predict_to(self, t):
        """(mean6, cov6) extrapolated to time t without changing the track."""
        e = self.est.copy()
        if t > self.t:
            e.predict(t - self.t)
        return e.pv()

    def update_class(self, lik):
        p = self.class_prob * np.asarray(lik, float)
        self.class_prob = p / p.sum() if p.sum() > 0 else np.full(len(p), 1.0 / len(p))


class Tracker:
    def __init__(self, make_est, pd=0.9, lam=1e-4, m=3, n=5, max_miss=8, n_classes=3):
        self.make_est, self.pd, self.lam = make_est, pd, lam
        self.m, self.n, self.max_miss, self.n_classes = m, n, max_miss, n_classes
        self.tracks, self._id = [], 0
        self.t = None

    # ---- shared management
    def _new_track(self, est, t):
        self._id += 1
        tr = Track(self._id, est, t, self.n_classes)
        tr.hits.append(True)
        return tr

    def _spawn(self, t, z, meas):
        p0, P0 = meas.init(z)
        tr = self._new_track(self.make_est(p0, P0), t)
        self.tracks.append(tr)
        return tr

    def _mark(self, tr, hit, t):
        tr.age += 1
        tr.hits.append(bool(hit))
        tr.misses = 0 if hit else tr.misses + 1
        recent = list(tr.hits)[-self.n :]
        if not tr.confirmed and sum(recent) >= self.m:
            tr.confirmed, tr.t_confirm = True, t

    def _alive(self, tr):
        if tr.confirmed:
            # also drop confirmed tracks fed only by sparse clutter hits: >= 3 hits in the last 10 scans once 10 old
            return tr.misses <= self.max_miss and (
                tr.age < 10 or sum(list(tr.hits)[-10:]) >= 3
            )
        # tentative: needs a second consecutive hit (two-point initiation) and must still be able to reach m of n
        return (
            tr.age < self.n
            and (len(tr.hits) < 2 or tr.hits[1])
            and sum(tr.hits) + (self.n - tr.age) >= self.m
        )

    def _finish(self, t):
        self.tracks = [tr for tr in self.tracks if self._alive(tr)]
        for tr in self.tracks:
            tr.history.append((t, tr.mean.copy()))

    def _dedupe(self, d2_max=9.0):
        """Delete tracks (aged >= m scans) that sit on a better track: JPDA coalescence / duplicate births."""
        order = sorted(
            self.tracks, key=lambda a: (-a.confirmed, -sum(a.hits), a.t_birth)
        )
        keep = []
        for a in order:
            ma, Pa = a.est.pv()
            dup = False
            for b in keep:
                mb, Pb = b.est.pv()
                d = ma - mb
                if (
                    a.age >= self.m
                    and np.linalg.norm(d[:3]) < 8
                    and d @ np.linalg.solve(Pa + Pb, d) < d2_max
                ):
                    dup = True
                    break
            if not dup:
                keep.append(a)
        ids = {a.id for a in keep}
        self.tracks = [a for a in self.tracks if a.id in ids]

    def _predict(self, t):
        for tr in self.tracks:
            if t > tr.t:
                tr.est.predict(t - tr.t)
            tr.t = t

    def _scan(self, zs, meas):
        zs = np.asarray(zs, float).reshape(-1, meas.dim)
        return zs, CHI2_GATE[meas.dim]

    def _gates(self, zs, meas, th):
        """(gate mask, likelihood) matrices, tracks x measurements."""
        T, M = len(self.tracks), len(zs)
        G, L = np.zeros((T, M), bool), np.zeros((T, M))
        if M:
            for i, tr in enumerate(self.tracks):
                G[i] = tr.est.gate_d2(zs, meas) < th
                L[i] = tr.est.lik_each(zs, meas)
        return G, L

    def _births(self, t, zs, meas, free, G):
        conf = [i for i, tr in enumerate(self.tracks[: G.shape[0]]) if tr.confirmed]
        for j in np.flatnonzero(free):
            if not (len(conf) and G[conf, j].any()):
                self._spawn(t, zs[j], meas)

    def step(self, t, zs, meas):
        zs, th = self._scan(zs, meas)
        self._tick(t, zs, meas, th)
        self.t = t
        return self.confirmed()

    def confirmed(self):
        return [tr for tr in self.tracks if tr.confirmed]


class GNN(Tracker):
    """Global nearest neighbour: one optimal one-to-one assignment (likelihood-ratio cost), chi-square gate."""

    def _tick(self, t, zs, meas, th):
        self._predict(t)
        G, L = self._gates(zs, meas, th)
        T, M = G.shape
        llr = np.log(np.maximum(self.pd * L / (self.lam * (1 - self.pd)), 1e-300))
        cost = np.where(G & (llr > 0), -llr, BIG)
        got, free = np.full(T, -1), np.ones(M, bool)
        if T and M:
            for i, j in zip(*lsa(cost)):
                if cost[i, j] < BIG / 2:
                    got[i], free[j] = j, False
        for i, tr in enumerate(self.tracks):
            if got[i] >= 0:
                tr.est.update(zs[got[i]], meas)
            self._mark(tr, got[i] >= 0, t)
        self._births(t, zs, meas, free, G)
        self._finish(t)


class JPDA(Tracker):
    """Joint probabilistic data association: each track is updated with all gated measurements weighted by joint-event
    marginals (clusters of tracks sharing measurements are enumerated exactly up to max_nodes)."""

    def __init__(self, *a, max_nodes=100000, **k):
        super().__init__(*a, **k)
        self.max_nodes = max_nodes

    def _marginals(self, W, G):
        T, M = W.shape
        pm = 1 - self.pd  # weight of "track i has no measurement"
        beta, beta0 = np.zeros((T, M)), np.ones(T)
        seen = np.zeros(T, bool)
        for s in range(T):  # clusters: tracks connected through shared measurements
            if seen[s]:
                continue
            cl, stack = [], [s]
            seen[s] = True
            while stack:
                i = stack.pop()
                cl.append(i)
                for k in range(T):
                    if not seen[k] and (G[i] & G[k]).any():
                        seen[k] = True
                        stack.append(k)
            self._cluster(W, G, pm, cl, beta, beta0)
        return beta, beta0

    def _cluster(self, W, G, pm, cl, beta, beta0):
        opts = [np.flatnonzero(G[i]) for i in cl]
        nc = len(cl)
        acc, accb, Z = np.zeros((nc, W.shape[1])), np.zeros(nc), [0.0]
        used, ch, nodes = set(), [-1] * nc, [0]

        def rec(a, w):
            nodes[0] += 1
            if nodes[0] > self.max_nodes:
                raise OverflowError
            if a == nc:
                Z[0] += w
                for q in range(nc):
                    if ch[q] < 0:
                        accb[q] += w
                    else:
                        acc[q, ch[q]] += w
                return
            ch[a] = -1
            rec(a + 1, w * pm)
            for j in opts[a]:
                if j not in used:
                    used.add(j)
                    ch[a] = j
                    rec(a + 1, w * W[cl[a], j])
                    used.discard(j)
            ch[a] = -1

        try:
            rec(0, 1.0)
            for q, i in enumerate(cl):
                beta[i], beta0[i] = acc[q] / Z[0], accb[q] / Z[0]
        except OverflowError:  # independent PDA
            for i in cl:
                w = np.where(G[i], W[i], 0.0)
                tot = pm + w.sum()
                beta[i], beta0[i] = w / tot, pm / tot

    def _tick(self, t, zs, meas, th):
        self._predict(t)
        G, L = self._gates(zs, meas, th)
        T, M = G.shape
        W = np.where(G, self.pd * L / self.lam, 0.0)
        beta, beta0 = self._marginals(W, G) if T else (np.zeros((0, M)), np.zeros(0))
        for i, tr in enumerate(self.tracks):
            idx = np.flatnonzero(G[i])
            if len(idx):
                tr.est.update_beta(zs[idx], beta0[i], beta[i, idx], meas, self.lam)
            self._mark(tr, len(idx) and beta0[i] < 0.5, t)
        free = (beta.sum(0) < 0.2) if T else np.ones(M, bool)
        self._births(t, zs, meas, free, G)
        self._finish(t)
        self._dedupe()


class _Leaf:
    __slots__ = ("age", "conf", "est", "hits", "miss", "path", "score")

    def __init__(self, est, score, hits, miss, path, age, conf):
        self.est, self.score, self.hits, self.miss, self.path, self.age, self.conf = (
            est,
            score,
            hits,
            miss,
            path,
            age,
            conf,
        )


class MHT(Tracker):
    """Track-oriented MHT. Each tree (= one track id) holds up to kmax leaf hypotheses scored by log-likelihood ratio.
    The global hypothesis is chosen greedily (trees by best score, first leaf whose measurements are unused), then
    N-scan pruning commits the choice made nscan scans ago. ponytail: greedy global hypothesis, not the exact ILP/Murty."""

    def __init__(self, *a, kmax=4, nscan=3, prune_gap=12.0, score_birth=2.0, **k):
        super().__init__(*a, **k)
        self.kmax, self.nscan, self.gap, self.s0 = kmax, nscan, prune_gap, score_birth
        self.trees, self.k = {}, 0
        self._sel = set()

    def confirmed(self):
        return [tr for tr in self.tracks if tr.confirmed and tr.id in self._sel]

    def _tick(self, t, zs, meas, th):
        k, M = self.k, len(zs)
        self.k += 1
        pd, lam, N = self.pd, self.lam, self.nscan
        lmiss = np.log(1 - pd)
        by_id = {tr.id: tr for tr in self.tracks}
        for tid, leaves in self.trees.items():
            tr = by_id[tid]
            dt = t - tr.t
            kids = []
            for lf in leaves:
                if dt > 0:
                    lf.est.predict(dt)
                if M:
                    d2, L = lf.est.gate_d2(zs, meas), lf.est.lik_each(zs, meas)
                    for j in np.flatnonzero(d2 < th):
                        e = lf.est.copy()
                        e.update(zs[j], meas)
                        sc = lf.score + np.log(max(pd * L[j] / lam, 1e-300))
                        kids.append(
                            _Leaf(
                                e,
                                sc,
                                (lf.hits + (True,))[-self.n :],
                                0,
                                (lf.path + ((k, int(j)),))[-(N + 1) :],
                                lf.age + 1,
                                lf.conf
                                or sum((lf.hits + (True,))[-self.n :]) >= self.m,
                            )
                        )
                lf.score += lmiss
                lf.hits, lf.miss, lf.age = (
                    (lf.hits + (False,))[-self.n :],
                    lf.miss + 1,
                    lf.age + 1,
                )
                lf.path = (lf.path + ((k, -1),))[-(N + 1) :]
                kids.append(lf)
            tr.t = t
            self.trees[tid] = kids
        # global hypothesis: greedy by best leaf score
        order = sorted(self.trees, key=lambda i: -max(lf.score for lf in self.trees[i]))
        used, sel = set(), {}
        for tid in order:
            for lf in sorted(self.trees[tid], key=lambda l: -l.score):
                if not (lf.conf or lf.score > 0):
                    break
                keys = {p for p in lf.path if p[1] >= 0}
                if keys & used:
                    continue
                used |= keys
                sel[tid] = lf
                break
        # N-scan pruning: commit the ancestor at scan k-N of every selected leaf
        commit = {tid: lf.path[0] for tid, lf in sel.items() if len(lf.path) == N + 1}
        ckeys = {p: tid for tid, p in commit.items() if p[1] >= 0}
        for tid, leaves in self.trees.items():
            keep = []
            for lf in leaves:
                if tid in commit and lf.path[0] != commit[tid]:
                    continue
                if any(p in ckeys and ckeys[p] != tid for p in lf.path):
                    continue
                keep.append(lf)
            self.trees[tid] = keep
        # limits: score gap, kmax, misses, tentative failure
        for tid in list(self.trees):
            ls = [
                lf
                for lf in self.trees[tid]
                if (lf.conf and lf.miss <= self.max_miss)
                or (
                    not lf.conf
                    and lf.age < self.n
                    and lf.score > -1.0
                    and sum(lf.hits) + self.n - lf.age >= self.m
                )
            ]
            if ls:
                best = max(lf.score for lf in ls)
                ls = sorted(
                    (lf for lf in ls if lf.score > best - self.gap),
                    key=lambda l: -l.score,
                )[: self.kmax]
            if ls:
                self.trees[tid] = ls
            else:
                del self.trees[tid]
                sel.pop(tid, None)
        self.tracks = [tr for tr in self.tracks if tr.id in self.trees]
        if sel:  # drop selections whose leaf was pruned
            sel = {
                i: lf
                for i, lf in sel.items()
                if any(lf is x for x in self.trees.get(i, []))
            }
        # expose the selected leaf
        for tr in self.tracks:
            lf = sel.get(tr.id)
            if lf is None:
                continue
            tr.est, tr.hits, tr.misses = lf.est, deque(lf.hits, maxlen=16), lf.miss
            tr.age = lf.age
            if lf.conf and not tr.confirmed:
                tr.confirmed, tr.t_confirm = True, t
            tr.history.append((t, tr.mean.copy()))
        self._sel = set(sel)
        # births from measurements nobody claimed (and not gated by a selected confirmed leaf)
        claimed = {
            p[1] for lf in sel.values() for p in lf.path if p[0] == k and p[1] >= 0
        }
        gated = np.zeros(M, bool)
        for lf in sel.values():
            if lf.conf and M:
                gated |= lf.est.gate_d2(zs, meas) < th
        for j in range(M):
            if j in claimed or gated[j]:
                continue
            p0, P0 = meas.init(zs[j])
            tr = self._new_track(self.make_est(p0, P0), t)
            self.tracks.append(tr)
            self.trees[tr.id] = [
                _Leaf(tr.est, self.s0, (True,), 0, ((k, int(j)),), 1, False)
            ]


class PHD(Tracker):
    """Gaussian-mixture PHD filter (constant Pd, Ps) with labels carried by the components (a label = one track): births from
    unexplained measurements, prune/merge within a label, extraction of labels whose summed weight exceeds 0.5."""

    def __init__(
        self, *a, ps=0.99, w_birth=0.1, w_prune=1e-4, merge_d2=4.0, jmax=80, **k
    ):
        super().__init__(*a, **k)
        self.ps, self.wb, self.wp, self.mg, self.jmax = (
            ps,
            w_birth,
            w_prune,
            merge_d2,
            jmax,
        )
        self.comps = []  # [w, est, label]
        self.card = 0.0

    def _tick(self, t, zs, meas, th):
        dt = 0.0 if self.t is None else t - self.t
        pd, lam, M = self.pd, self.lam, len(zs)
        for c in self.comps:
            if dt > 0:
                c[1].predict(dt)
            c[0] *= self.ps
        new, upd = [], []
        for w, e, lab in self.comps:
            new.append([w * (1 - pd), e, lab])
            if M:
                d2, L = e.gate_d2(zs, meas), e.lik_each(zs, meas)
                for j in np.flatnonzero(d2 < th):
                    upd.append((j, pd * w * L[j], e, lab))
        den = np.full(M, lam)
        for j, wl, _, _ in upd:
            den[j] += wl
        explained = np.zeros(M)
        for j, wl, e, lab in upd:
            x, P = e.condition(zs[j], meas), e.post_cov(meas)
            c = e.copy()
            c.x, c.P, c._c = x, P, None
            new.append([wl / den[j], c, lab])
            explained[j] += wl / den[j]
        self.comps = self._reduce([c for c in new if c[0] > self.wp])
        # births from measurements the mixture does not explain (tracks start next scan)
        near = np.zeros(
            M, bool
        )  # gated by a component of real weight: a missed target, not a new one
        for j, wl, e, lab in upd:
            near[j] |= wl > 0.02 * den[j]
        for j in range(M):
            if explained[j] < 0.9 and not near[j]:
                p0, P0 = meas.init(zs[j])
                self._id += 1
                self.comps.append(
                    [self.wb * (1 - explained[j]), self.make_est(p0, P0), self._id]
                )
        self.card = sum(c[0] for c in self.comps)
        self._extract(t)

    def _reduce(self, comps):
        out, by = [], {}
        for c in comps:
            by.setdefault(c[2], []).append(c)
        for lab, cs in by.items():
            cs = sorted(cs, key=lambda c: -c[0])
            while cs:
                h = cs.pop(0)
                grp, rest = [h], []
                Pi = np.linalg.inv(h[1].P[:6, :6] + 1e-12 * np.eye(6))
                for c in cs:
                    d = c[1].x[:6] - h[1].x[:6]
                    (grp if d @ Pi @ d < self.mg else rest).append(c)
                cs = rest
                if len(grp) == 1:
                    out.append(h)
                    continue
                W = sum(g[0] for g in grp)
                x = sum(g[0] * g[1].x for g in grp) / W
                P = (
                    sum(g[0] * (g[1].P + np.outer(g[1].x - x, g[1].x - x)) for g in grp)
                    / W
                )
                e = h[1].copy()
                e.x, e.P, e._c = x, 0.5 * (P + P.T), None
                out.append([W, e, lab])
        out.sort(key=lambda c: -c[0])
        return out[: self.jmax]

    def _extract(self, t):
        W, best = {}, {}
        for w, e, lab in self.comps:
            W[lab] = W.get(lab, 0.0) + w
            if lab not in best or w > best[lab][0]:
                best[lab] = (w, e)
        by_id = {tr.id: tr for tr in self.tracks}
        out = []
        for lab, wl in W.items():
            tr = by_id.get(lab)
            if tr is None:
                if wl < 0.3:
                    continue
                tr = Track(lab, best[lab][1].copy(), t, self.n_classes)
                tr.hits.clear()
                tr.age = 0
            else:
                tr.est = best[lab][1].copy()
            tr.t = t
            self._mark(tr, wl > 0.5, t)
            tr.misses = 0 if wl > 0.2 else tr.misses + 1
            if wl > 0.05 and (tr.confirmed or self._alive(tr)):
                out.append(tr)
        self.tracks = out
        for tr in out:
            tr.history.append((t, tr.mean.copy()))
