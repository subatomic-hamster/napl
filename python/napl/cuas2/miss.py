"""Closest approach of a ball to a target from samples taken every control period (5 ms, ~14 cm of flight at 28 m/s).

The lite referee keeps the smallest sampled distance. For a ball that HITS the (ball-radius inflated) target sphere the kernel removes it at the surface, so the last
sample lies short of the sphere on the incoming side: the recorded 'miss vector' points back along the flight path by about the target radius. A ball climbing at
elevation e therefore reads low by about r sin(e) (r = 0.16 m: -2 to -4 cm at 10-20 deg), which is what showed up as a "static-target aim bias" in the Monte Carlo
(P13 BIAS, docs/cuas2/aim-bias.md). Here consecutive samples are joined by a segment (relative motion is linear within 5 ms), and a ball that vanishes is
extrapolated four sample intervals along its last relative velocity, so the minimum is the pass of the ball through the target centre's neighbourhood and not the
sampling phase.

    ca = ClosestApproach()
    ca.add(rnd, target_id, t, rel)          # every period a ball is in flight; rel = ball - target (m)
    ca.end(rnd)                             # the ball is gone (hit, ground, expired)
    ca.best[rnd] -> dict(d, vec, id, t)
"""

import numpy as np

EXTRAPOLATE = 4.0  # sample intervals a vanished ball is continued along its last relative velocity


def _min_on_segment(r0, dr, s_max):
    """Point r0 + s dr, s in [0, s_max], closest to the origin -> (s, r)."""
    s = float(np.clip(-(r0 @ dr) / max(float(dr @ dr), 1e-12), 0.0, s_max))
    return s, r0 + s * dr


class ClosestApproach:
    def __init__(self):
        self.best = {}  # round -> dict(d, vec, id, t)
        self._last = {}  # (round, target id) -> [(t, rel) older, (t, rel) newest]

    def _offer(self, rnd, tid, t, rel):
        d = float(np.linalg.norm(rel))
        if d < self.best.get(rnd, {"d": np.inf})["d"]:
            self.best[rnd] = dict(d=d, vec=np.asarray(rel, float).copy(), id=tid, t=t)

    def add(self, rnd, tid, t, rel):
        rel = np.asarray(rel, float)
        last = self._last.setdefault((rnd, tid), [])
        if last:
            (t0, r0) = last[-1]
            s, r = _min_on_segment(r0, rel - r0, 1.0)
            self._offer(rnd, tid, t0 + s * (t - t0), r)
        self._offer(rnd, tid, t, rel)
        last.append((t, rel))
        del last[:-2]

    def end(self, rnd):
        """The ball vanished after its last sample: continue it along its last relative velocity."""
        for (r_, tid), last in list(self._last.items()):
            if r_ != rnd:
                continue
            if len(last) == 2:
                (t0, r0), (t1, r1) = last
                s, r = _min_on_segment(r1, r1 - r0, EXTRAPOLATE)
                self._offer(rnd, tid, t1 + s * (t1 - t0), r)
            del self._last[(r_, tid)]

    def rounds_in_flight(self):
        return {r for r, _ in self._last}
