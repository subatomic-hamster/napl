"""The window filter on the wheel telemetry (launcher.c, FDR_RPM_FILT_DIV = 8: first-order IIR, tau 8 ms at 1 kHz [est, B-TIME]).
Each test runs the C core and the python twin on the same inputs and demands identical wheels_ready sequences.
Latency bound: a step of k x the window crosses the window edge in n samples with (7/8)^n <= 1 - 1/k: k = 2 -> 6 ms, k = 3 -> 4 ms."""

import numpy as np
import pytest
from napl.cuas2 import protocol as p

from cuas2 import fwharness as H

SET = 17500  # speed_req 1000 on the default cfg (rpm_max 17500, below the cap)
WIN = 10  # permille, the firmware default
SETTLE = 100
AGE_MAX = 20  # FDR_WHEEL_AGE_MAX_MS


@pytest.fixture(scope="module", autouse=True)
def need_lib():
    H.require_lib()


def fin(t, top, bot):
    # gate_home: the homing timeout (1 s) would otherwise latch a fault and drop the target
    return H.FeederIn(
        t, rpm_top=top, rpm_bot=bot, speed_req_pm=1000, diff_pm=0, gate_home=True
    )


class Rig:
    """Both implementations side by side; feed(top, bot) returns the shared wheels_ready bit."""

    def __init__(self):
        self.s = [H.FeederFsm(), H.CFeeder()]
        self.t = 0

    def feed(self, top, bot):
        self.t += 1
        r = []
        for s in self.s:
            o = s.step(fin(self.t, top, bot))
            r.append(o.status_fields["wheels_ready"])
        assert r[0] == r[1], f"C and python diverge at t={self.t}"
        return r[0]

    def run(self, n, top=SET, bot=SET):
        return [self.feed(top, bot) for _ in range(n)]


def settled():
    g = Rig()
    ready = g.run(1500)  # ramp (17.5 krpm at the default ramp rate) + settle
    assert ready[-1]
    return g


def ticks_until_not_ready(g, top, bot, limit=100):
    for n in range(1, limit + 1):
        if not g.feed(top, bot):
            return n
    return None


@pytest.mark.parametrize("k,bound_ms", [(2.0, 6), (3.0, 4), (5.0, 2)])
def test_real_slowdown_drops_ready_within_the_stated_bound(k, bound_ms):
    g = settled()
    low = round(SET * (1 - k * WIN / 1000))
    n = ticks_until_not_ready(g, low, low)
    assert n is not None and n <= bound_ms, (k, n)


def test_one_wheel_slowing_drops_ready_too():
    g = settled()
    n = ticks_until_not_ready(g, round(SET * (1 - 2.5 * WIN / 1000)), SET)
    assert n is not None and n <= 6


def test_wheel_difference_drift_drops_ready():
    """Both wheels individually inside the window, the difference outside it (+0.8 % / -0.8 % -> 1.6 % apart)."""
    g = settled()
    n = ticks_until_not_ready(g, round(SET * 0.992), round(SET * 1.008))
    assert n is not None and n <= 12


def test_step_response_of_the_filter_is_bounded_and_monotone():
    """A step from the setpoint to 0.9x never overshoots and ends in the window of the new level: no hidden gain, no windup."""
    f = H.FeederFsm()
    seen = []
    for t in range(1, 1500):
        f.step(fin(t, SET, SET))
    for t in range(1500, 1600):
        f.step(fin(t, SET * 9 // 10, SET))
        seen.append(f.frpm16[0] / 16)
    assert all(a >= b - 1 for a, b in zip(seen, seen[1:]))  # monotone down
    assert min(seen) >= SET * 0.9 - 1  # no undershoot
    assert abs(seen[-1] - SET * 0.9) < 1


def test_noise_at_the_plant_level_is_ready_at_the_default_window():
    rng = np.random.default_rng(1)
    g = Rig()
    g.run(700)  # spin-up
    n = 5000
    ripple = 0.0016 * np.sin(2 * np.pi * np.arange(n) / 37.0)
    top = SET * (1 + rng.normal(0, 0.003, n) + ripple)
    bot = SET * (1 + rng.normal(0, 0.003, n) - ripple)
    ready = [g.feed(round(a), round(b)) for a, b in zip(top, bot)]
    assert np.mean(ready[200:]) >= 0.95, np.mean(ready[200:])


def test_stale_telemetry_is_not_ready_and_never_the_last_filtered_value_forever():
    g = settled()
    # one wheel's frames stop: ready is gone once the age passes FDR_WHEEL_AGE_MAX_MS (20 ms) and stays gone
    ready = [g.feed(None, SET) for _ in range(AGE_MAX + 30)]
    assert all(ready[:AGE_MAX])  # tolerance: a few lost frames do not blink READY
    assert not any(ready[AGE_MAX:]), "stale telemetry still READY"
    # frames return: the filter is re-seeded from the first raw sample (not blended with a 50 ms old state), READY needs the settle time again
    back = [g.feed(SET, SET) for _ in range(SETTLE + 20)]
    assert not any(back[: SETTLE - 1]) and back[-1]


def test_stale_state_does_not_leak_into_the_first_new_sample():
    """Wheel really slowed to 0.97x while the frames were missing: the first frame back must not be averaged with the old in-window state."""
    g = settled()
    for _ in range(AGE_MAX + 5):
        g.feed(None, None)
    low = round(SET * 0.97)
    for t in range(SETTLE + 50):
        assert not g.feed(low, low)
    f = g.s[0]
    assert abs(f.frpm16[0] / 16 - low) < 1


def test_both_wheels_lost_for_the_telemetry_loss_time_faults():
    g = settled()
    for _ in range(80):
        g.feed(None, None)
    o = g.s[1].step(fin(g.t + 1, None, None))
    assert o.status_fields["fault_bits"] & p.FAULT_TELEMETRY_LOST
