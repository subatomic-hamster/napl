"""Lockstep sim smoke: the whole stack on the napl kernel (placeholder turret, pendulum balloon, SimRP2040 behind a LoopbackLink, TruthDetector, App).
Deterministic: two runs give the same state hash, and napl-replay reproduces it from the replay log."""

from pathlib import Path

import pytest
from napl import program
from napl.lite import protocol as p
from napl.lite.fire import Sector
from napl.lite.sim_io import Balloon, Lockstep, build_scene

BALLOONS = [Balloon("b1", (6.0, 0.5, 3.3), length=1.5)]


def run_once(tmp_path, name, seconds=30.0, targets=BALLOONS, hold=True, **kw):
    out = tmp_path / name
    out.mkdir()
    man = build_scene(out / "scene", targets, seed=1)
    h = Lockstep(man, out, targets, sector=Sector(-60, 60, 0, 60), seed=3, **kw)
    h.hold_deadman = hold
    h.run(8.0)
    assert (
        h.gimbal.status.state == p.IDLE
        and h.gimbal.status.flag1(p.F1_PAN_HOMED)
        and h.gimbal.status.flag1(p.F1_TILT_HOMED)
    ), h.app.log
    h.app.post("arm")
    h.run(seconds, until=lambda s: len(s.popped) == len(targets))
    return h, out, man


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("lockstep")
    return [run_once(tmp, "a"), run_once(tmp, "b")]


def test_balloon_engaged_and_hit(runs):
    h, out, man = runs[0]
    assert h.popped, (
        f"no balloon popped; shots={len(h.twin.shots)} log={list(h.app.log)[-8:]} reasons={h.app.dec.reasons}"
    )
    assert 1 <= len(h.twin.shots) <= 5
    assert h.world.rounds == len(h.twin.shots) == h.plant.pulses
    assert h.world.fires_unauthorized == 0
    assert h.app.failed is None


def test_geometry_was_measured_off_the_scene(runs):
    g = runs[0][0].geom
    assert (
        0.05 < g.h_t < 0.6
        and 0.05 < g.l_m < 0.6
        and abs(g.d_b) < 0.2
        and abs(g.d_c)
        < 0.3  # cuas-lite: the Brio sits 10 mm below the tilt axis (the placeholder camera was above it)
    ), g


def test_deterministic_and_replay_matches(runs):
    (h1, o1, m1), (h2, o2, m2) = runs
    n1, n2 = h1.finish(), h2.finish()
    assert (
        n1["state_hash"] == n2["state_hash"]
        and n1["tick"] == n2["tick"]
        and not n1["diverged"]
    )
    assert h1.popped == h2.popped and h1.twin.shots == h2.twin.shots
    rp = program.replay_hash(o1 / "replay.naplog", Path(m1["model"]).parent)
    if rp is None:
        pytest.skip("build/napl-replay missing")
    assert rp == (n1["tick"], n1["state_hash"])


def test_kernel_value_is_pinned(runs):
    """The lite kernel must not move: an additive change (the cuas2 launcher attrs, ...) keeps this run's tick and state hash. Update only on a deliberate kernel change."""
    n = runs[0][0].finish()
    assert (n["tick"], n["state_hash"]) == (11015, PINNED_HASH), n


PINNED_HASH = "35634cd3ed9cf0f0"


def test_person_scenario_fires_nothing(tmp_path):
    # a person capsule is not in the kernel scene: the detector reports one at the far edge of the image from the start
    from napl.lite.io import Det

    out = tmp_path / "p"
    out.mkdir()
    man = build_scene(out / "scene", BALLOONS, seed=1)
    h = Lockstep(man, out, BALLOONS, sector=Sector(-60, 60, 0, 60), seed=3)
    h.detector.mutate = lambda d: d + [Det("person", 0.8, (1180, 300, 1260, 600))]
    h.hold_deadman = True
    h.run(8.0)
    h.app.post("arm")
    h.run(12.0)
    assert h.twin.shots == [] and h.world.rounds == 0
