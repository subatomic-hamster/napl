"""Tracker benchmark on the threat suite: napl.track variants fed a ThreatScenario with an alive mask (births/deaths: only objects that
exist in a frame are scored), paired seeds across variants."""

import time

import numpy as np

from ..track.metrics import Evaluator
from ..track.variants import make_tracker
from .scenes import build

KEYS = ("gospa", "loc", "missed", "false", "breaks", "switches", "cpu_ms")


def run(scn, variant, seed):
    """One run on a built ThreatScenario -> metric dict (Evaluator.summary + cpu_ms); None if the variant cannot use the sensor."""
    if variant == "ab-nn" and scn.sensor == "cam":
        return None  # baseline works on position measurements only
    tr = make_tracker(variant, scn, seed)
    ev, dts = Evaluator(scn.c), []
    ids = np.array(scn.ids)
    for k, (t, meas, zs) in enumerate(scn.frames):
        t0 = time.perf_counter()
        tracks = tr.step(t, zs, meas)
        dts.append(time.perf_counter() - t0)
        a = scn.alive[k]
        ev.add(t, list(ids[a]), scn.truth[k][a], tracks)
    s = ev.summary(scn.ids)
    s["cpu_ms"] = 1e3 * float(np.mean(dts))
    return s


def job(a):
    name, variant, seed = a
    return name, variant, seed, run(build(name, seed), variant, seed)


def aggregate(res):
    """[(name, variant, seed, metrics|None)] -> {(name, variant): {key: mean, gospa_ci95, n}}."""
    g = {}
    for name, v, _, r in res:
        if r is not None:
            g.setdefault((name, v), []).append(r)
    out = {}
    for k, rs in g.items():
        m = {q: float(np.nanmean([r[q] for r in rs])) for q in KEYS}
        m["gospa_ci95"] = (
            1.96 * float(np.std([r["gospa"] for r in rs], ddof=1)) / np.sqrt(len(rs))
        )
        m["n"] = len(rs)
        out[k] = m
    return out


def table(agg, scenarios, variants, header=True):
    lines = []
    for sc in scenarios:
        s = build(sc, 1)
        if header:
            lines += [
                f"### {sc}  ({s.sensor}{'+lidar' if s.lidar else ''}, {len(s.ids)} objects [{','.join(sorted(set(s.labels)))}], dt {s.dt} s, {len(s.frames)} frames, Pd {s.pd}, clutter {s.n_clutter}/scan, GOSPA c = {s.c} m)",
                "",
                "| variant | GOSPA | +-95% | loc | missed | false | breaks | ID sw | ms/update |",
                "|---|---|---|---|---|---|---|---|---|",
            ]
        for v in variants:
            m = agg.get((sc, v))
            if m is None:
                lines.append(f"| {v} | n/a | | | | | | | |")
                continue
            lines.append(
                f"| {v} | {m['gospa']:.3f} | {m['gospa_ci95']:.3f} | {m['loc']:.3f} | {m['missed']:.3f} | {m['false']:.3f} | {m['breaks']:.2f} | {m['switches']:.2f} | {m['cpu_ms']:.2f} |"
            )
        lines.append("")
    return lines
