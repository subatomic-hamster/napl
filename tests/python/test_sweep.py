"""M7 exit checks: Sweep (grid / seeded random), thread-count independence, divergence robustness, session limits."""

import threading
import time
import tomllib
from pathlib import Path

import napl
import numpy as np
import pytest

DATA = Path(__file__).resolve().parents[2] / "tests" / "data"
PENDULUM = DATA / "pendulum.xml"
SERVO = "/actuators/servo"
KP = f"{SERVO}.kp"
HINGE = "/world/pole/hinge"


def spec(**over) -> dict:
    """servo.toml as a TestSpec (what the CLI sends); its own kp=50 setting is overridden by the sweep params."""
    s = tomllib.loads((DATA / "specs" / "servo.toml").read_text())
    s.pop("model")
    s.update(over)
    return s


def kp_param(values) -> list:
    return [{"path": SERVO, "attr": "kp", "values": [float(v) for v in values]}]


def sim(**kw) -> napl.Sim:
    return napl.Sim.load(PENDULUM, **kw)


def test_grid_50_rows_ordered_with_known_verdicts():
    kps = sorted([0.5, 50.0] + [float(x) for x in np.linspace(1.0, 49.0, 48)])
    rows = sim().sweep(spec(), kp_param(kps), threads=4)
    assert len(rows) == 50
    assert [r["index"] for r in rows] == list(range(50))
    # echoed, in grid order (the JSON edge prints doubles with 12 decimals, hence approx)
    assert [r["params"] for r in rows] == [
        pytest.approx({KP: kp}, abs=1e-11) for kp in kps
    ]
    assert all(
        r["error"] == "" and not r["diverged"] and len(r["monitors"]) == 2 for r in rows
    )
    verdict = {r["params"][KP]: r["pass"] for r in rows}
    assert verdict[0.5] is False and verdict[50.0] is True
    passes = [r["pass"] for r in rows]
    assert not any(passes[:10]) and all(
        passes[-10:]
    )  # low gain fails, high gain passes
    # a row is what a standalone RunTest of the same point says
    solo = sim().run_test(spec())  # servo.toml itself has kp = 50
    hold = next(m for m in rows[-1]["monitors"] if m["name"] == "holds_target")
    assert (
        hold["min_value"]
        == next(m for m in solo["monitors"] if m["name"] == "holds_target")["min_value"]
    )


def test_cartesian_product_order():
    params = kp_param([1, 2, 3]) + [{"path": SERVO, "attr": "kv", "values": [4, 5]}]
    rows = sim().sweep(spec(duration=0.1), params)
    assert [(r["params"][KP], r["params"][f"{SERVO}.kv"]) for r in rows] == [
        (kp, kv) for kp in (1, 2, 3) for kv in (4, 5)
    ]


def test_threads_6_equal_threads_1_exactly():
    params = kp_param(np.linspace(0.5, 80, 24)) + [
        {"path": SERVO, "attr": "kv", "values": [1.0, 2.0, 4.0]}
    ]
    one = sim().sweep(spec(), params, threads=1)
    six = sim().sweep(spec(), params, threads=6)
    assert len(one) == 72
    assert (
        one == six
    )  # full rows: params, verdicts, every monitor stat, exact float equality
    assert one == sim().sweep(spec(), params, threads=16)
    assert one == sim().sweep(
        spec(), params, threads=64
    )  # capped at the core count, same result


def test_kp_1e9_diverges_and_others_unaffected():
    clean = sim().sweep(spec(), kp_param([0.5, 20, 50]), threads=3)
    s = sim()
    rows = s.sweep(
        spec(), kp_param([0.5, 1e9, 20, 50]), threads=3
    )  # the process must survive this
    assert [r["params"][KP] for r in rows] == [0.5, 1e9, 20, 50]
    assert rows[1]["diverged"] is True and rows[1]["pass"] is False
    assert [rows[i] | {"index": 0} for i in (0, 2, 3)] == [
        r | {"index": 0} for r in clean
    ]
    assert not s.diverged and s.tick == 0
    # still alive afterwards
    s.step(10)
    assert s.tick == 10
    # MuJoCo auto-resets on a bad qacc (qpos back to 0, only a warning counter remains); the sticky flag still catches it,
    # and restoring a healthy snapshot clears both
    snap = s.save()
    r = s.run_test(
        spec(
            settings=[{"path": SERVO, "attr": "kp", "values": [1e9]}]
            + spec()["settings"][1:]
        )
    )
    assert r["diverged"] and s.diverged
    s.restore(snap)
    assert not s.diverged
    s.step(10)
    assert s.tick == 20 and not s.diverged


def test_random_sweep_is_seeded_by_run_index_only():
    p = [
        {"path": SERVO, "attr": "kp", "lo": 1.0, "hi": 100.0},
        {"path": SERVO, "attr": "kv", "lo": 0.5, "hi": 5.0},
    ]
    s, sp = sim(), spec(duration=0.2)
    a = s.sweep(sp, p, mode="random", n=12, seed=7, threads=1)
    assert a == s.sweep(
        sp, p, mode="random", n=12, seed=7, threads=6
    )  # repeatable, thread-independent
    b = s.sweep(sp, p, mode="random", n=12, seed=8)
    assert [r["params"] for r in a] != [r["params"] for r in b]
    assert len({r["params"][KP] for r in a}) == 12
    assert all(
        1.0 <= r["params"][KP] <= 100.0 and 0.5 <= r["params"][f"{SERVO}.kv"] <= 5.0
        for r in a
    )
    # run i depends on (seed + i) only: run 3 of seed 7 == run 0 of seed 10
    (c,) = s.sweep(sp, p, mode="random", n=1, seed=10)
    assert c["params"] == a[3]["params"]


def snapshot_id(s: napl.Sim) -> int:
    r = s.call({"cmd_type": "Snapshot", "cmd": {}})
    assert r["ok"]
    return r["snapshot_id"]


def test_caller_untouched_and_snapshot_base():
    s = sim()
    snap = snapshot_id(s)  # state at tick 0, hinge at 0
    s.step(100)
    s.set(HINGE, "qpos", 0.3)
    h, tick = s.state_hash(), s.tick
    now = s.sweep(spec(), kp_param([0.5, 50]))
    from_snap = s.sweep(spec(), kp_param([0.5, 50]), base=snap)
    assert (s.state_hash(), s.tick) == (h, tick)  # sweeps never mutate the caller
    assert from_snap == sim().sweep(
        spec(), kp_param([0.5, 50])
    )  # = a fresh Sim, not the stepped one
    assert from_snap != now
    assert (
        now[0]["monitors"][1]["hit_tick"] != from_snap[0]["monitors"][1]["hit_tick"]
    )  # world ticks of the base
    with pytest.raises(ValueError, match="not_found"):
        s.sweep(spec(), kp_param([1]), base=snap + 99)


def test_error_paths():
    s = sim(max_sweep_runs=10)
    h = s.state_hash()
    with pytest.raises(ValueError, match="limit_exceeded"):
        s.sweep(spec(), kp_param(range(11)))
    with pytest.raises(ValueError, match="limit_exceeded"):
        s.sweep(
            spec(),
            [{"path": SERVO, "attr": "kp", "lo": 0, "hi": 1}],
            mode="random",
            n=11,
        )
    assert (
        len(s.sweep(spec(), kp_param(range(10)))) == 10
    )  # exactly at the limit is fine
    with pytest.raises(ValueError, match="unknown prim '/actuators/nope'"):
        s.sweep(spec(), [{"path": "/actuators/nope", "attr": "kp", "values": [1]}])
    with pytest.raises(ValueError, match="not a position servo"):
        s.sweep(spec(), [{"path": "/actuators/torque", "attr": "kp", "values": [1]}])
    with pytest.raises(ValueError):
        s.sweep(spec(), kp_param([1]), mode="bogus")
    with pytest.raises(ValueError, match="invalid_argument"):
        s.sweep(
            spec(),
            [{"path": SERVO, "attr": "kp", "lo": 5, "hi": 1}],
            mode="random",
            n=2,
        )
    with pytest.raises(ValueError, match="invalid_argument"):
        s.sweep(spec(), kp_param([]))
    # a bad value is that run's problem, not the sweep's
    rows = s.sweep(spec(duration=0.1), kp_param([-1.0, 5.0]))
    assert rows[0]["error"] and not rows[0]["pass"] and rows[1]["error"] == ""
    assert s.state_hash() == h


def test_wall_time_limit():
    long_spec = spec(duration=2000.0)  # 1e6 ticks per run: far beyond the budget
    s = sim(max_wall_s=0.2)
    h = s.state_hash()
    t0 = time.monotonic()
    with pytest.raises(ValueError, match="limit_exceeded"):
        s.sweep(long_spec, kp_param([1, 2, 3, 4]), threads=4)
    assert time.monotonic() - t0 < 5
    t0 = time.monotonic()
    with pytest.raises(ValueError, match="max_wall_s"):
        s.run_test(long_spec)
    assert time.monotonic() - t0 < 5
    assert (s.state_hash(), s.tick) == (
        h,
        0,
    )  # the timed-out RunTest rolled back (settings included)
    # within budget the limit changes nothing
    assert sim(max_wall_s=60).sweep(spec(), kp_param([0.5, 50])) == sim().sweep(
        spec(), kp_param([0.5, 50])
    )


def test_sweep_from_background_thread_releases_gil():
    s, other = sim(), sim()
    out = {}

    def bg():
        out["rows"] = s.sweep(spec(), kp_param(np.linspace(1, 100, 120)), threads=4)

    t = threading.Thread(target=bg)
    t0 = time.monotonic()
    t.start()
    steps = 0
    while (
        t.is_alive()
    ):  # the main thread keeps working (and stays responsive) meanwhile
        other.step(1)
        steps += 1
    t.join()
    assert len(out["rows"]) == 120
    assert steps > 50, (
        f"main thread starved: only {steps} steps in {time.monotonic() - t0:.2f}s"
    )
