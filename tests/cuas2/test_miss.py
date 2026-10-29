"""napl.cuas2.miss.ClosestApproach against the lite referee's 5 ms-sampled closest approach (P13 BIAS, docs/cuas2/aim-bias.md).

A ball that hits is removed at the target surface, so the lite rule (smallest SAMPLED distance) records a point short of the sphere on the incoming side:
for a ball climbing at 14 deg that is about -r sin(14 deg) = -4 cm in z for a centre shot. The refined rule recovers the true offset to the millimetre."""

import math

import numpy as np
import pytest
from napl.cuas2.miss import ClosestApproach

R_HIT = 0.15 + 0.011  # target radius + ball radius (the kernel's inflated sphere)
DT = 0.005
V = 28.5
EL = math.radians(14.0)
D = np.array([math.cos(EL), 0.0, math.sin(EL)])
PERP = np.array(
    [-math.sin(EL), 0.0, math.cos(EL)]
)  # in the vertical plane, normal to the flight


def flight(offset, phase, removed_on_hit=True):
    """Straight-line ball past a static target at the origin; rel = ball - target sampled every DT starting `phase` into the pass. -> list of (t, rel)."""
    o = offset * PERP
    out, k = [], 0
    while True:
        s = (
            -0.5 + phase * V + k * DT * V
        )  # path coordinate (m) relative to the closest point
        rel = o + D * s
        if removed_on_hit and np.linalg.norm(rel) <= R_HIT:
            return out  # the kernel removed the ball between the previous sample and this one
        if s > 0.5:
            return out
        out.append((phase + k * DT, rel))
        k += 1


def refined(samples):
    ca = ClosestApproach()
    for t, rel in samples:
        ca.add(1, "t", t, rel)
    ca.end(1)
    return ca.best[1]["vec"]


def naive(samples):
    return min((rel for _, rel in samples), key=np.linalg.norm)


OFFSETS = [-0.08, -0.04, 0.0, 0.04, 0.08]
PHASES = np.linspace(0.0, DT, 6, endpoint=False)


@pytest.mark.parametrize("removed", [True, False])
def test_refined_recovers_the_true_offset(removed):
    for off in OFFSETS:
        for ph in PHASES:
            s = flight(off, ph, removed)
            assert np.linalg.norm(refined(s) - off * PERP) < 2e-3, (off, ph, removed)


def test_lite_rule_reads_low_on_hits_and_the_refined_one_does_not():
    s_all = [flight(off, ph) for off in OFFSETS for ph in PHASES]
    z_naive = np.mean([naive(s)[2] for s in s_all])
    z_ref = np.mean([refined(s)[2] for s in s_all])
    assert z_naive < -0.02, (
        z_naive
    )  # the artefact: about -r sin(el) of the sphere radius, less the sampling phase
    assert (
        abs(z_ref - np.mean([off * PERP[2] for off in OFFSETS for _ in PHASES])) < 2e-3
    ), z_ref


def test_moving_target_uses_relative_motion():
    """rel is ball minus target: a target crossing the line of fire is handled the same way (only rel enters)."""
    ca = ClosestApproach()
    for k in range(40):
        t = k * DT
        ca.add(
            7, "m", t, np.array([-1.0 + 28.0 * t, 0.12 - 1.5 * t, 0.0])
        )  # closest in y at t = 0.04, x = 0.12 .. relative path is a line
    ca.end(7)
    b = ca.best[7]
    r0, dr = np.array([-1.0, 0.12, 0.0]), np.array([28.0, -1.5, 0.0])
    s = -(r0 @ dr) / (dr @ dr)
    assert np.linalg.norm(b["vec"] - (r0 + s * dr)) < 1e-6
