"""M4 exit checks for napl.Sim (the nanobind layer). Needs the C++ `napl-replay` exe from the dev build."""

import os
import random
import re
import subprocess
import threading
import time
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "tests" / "data"
PENDULUM = DATA / "pendulum.xml"
HINGE = "/world/pole/hinge"
REPLAY_EXE = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))


def pendulum() -> napl.Sim:
    return napl.Sim.load(PENDULUM)


def test_hash_matches_cpp_replay(tmp_path):
    rng = random.Random(4242)
    s = pendulum()
    s.start_log()
    for _ in range(200):
        k = rng.randrange(4)
        if k == 0:
            s.set(HINGE, "qpos", rng.uniform(-1.5, 1.5))
        elif k == 1:
            s.set("/actuators/torque", "ctrl", rng.uniform(-5, 5))
        elif k == 2:
            s.query("/world/pole", "pos")  # reads must not perturb the run
        else:
            s.step(rng.randrange(1, 40))
    log = tmp_path / "py.naplog"
    log.write_bytes(s.log())

    assert REPLAY_EXE.exists(), f"build the C++ exe first: {REPLAY_EXE}"
    out = subprocess.run(
        [str(REPLAY_EXE), str(log)], capture_output=True, text=True, check=True
    ).stdout
    m = re.fullmatch(r"tick=(\d+) state_hash=([0-9a-f]{16})\n", out)
    assert m, out
    assert int(m[1]) == s.tick
    assert m[2] == f"{s.state_hash():016x}"
    assert s.tick > 0


def test_eight_forks_diverge():
    parent = pendulum()
    parent.step(10)
    h0 = parent.state_hash()
    hashes = set()
    for i in range(8):
        f = parent.fork()
        assert f.state_hash() == h0 and f.tick == parent.tick  # a fork starts identical
        f.set("/actuators/torque", "ctrl", 0.3 * (i + 1))
        f.step(50)
        hashes.add(f.state_hash())
    assert len(hashes) == 8
    assert parent.state_hash() == h0 and parent.tick == 10  # parent untouched


def test_restore_gives_original_hash():
    s = pendulum()
    s.step(25)
    h, tick, blob = s.state_hash(), s.tick, s.save()
    s.set("/actuators/torque", "ctrl", 2.0)
    s.step(100)
    assert s.state_hash() != h
    s.restore(blob)
    assert s.state_hash() == h and s.tick == tick
    with pytest.raises(Exception):
        s.restore(b"junk")


def test_query_numpy_and_unit():
    s = pendulum()
    pos, unit = s.query("/world/pole", "pos")
    assert isinstance(pos, np.ndarray) and pos.dtype == np.float64 and pos.shape == (3,)
    assert unit == "m"
    np.testing.assert_allclose(pos, [0, 0, 1])
    with pytest.raises(KeyError):
        s.query("/world/nope", "pos")


def test_error_paths():
    s = pendulum()
    r = s.call({"id": 1, "source": "t", "cmd_type": "Query", "cmd": {"bogus_field": 1}})
    assert r["ok"] is False and "bogus_field" in r["message"]
    r = s.call("{not json")
    assert r["ok"] is False and r["message"]
    r = s.call({"cmd_type": "Step", "cmd": {"n": 3}})
    assert r["ok"] is True and r["tick"] == 3
    with pytest.raises(ValueError):
        s.set(HINGE, "qpos", [1.0, 2.0])  # wrong arity
    with pytest.raises(RuntimeError, match="no_such_model"):
        napl.Sim.load(DATA / "no_such_model.xml")


def _steps_for(seconds: float) -> int:
    probe = pendulum()
    t = time.perf_counter()
    probe.step(20000)
    return max(20000, int(20000 * seconds / (time.perf_counter() - t)))


def test_gil_released_threads_run_concurrently():
    n = _steps_for(0.5)
    ref = pendulum()
    ref.step(n)
    sims = [pendulum() for _ in range(2)]
    ts = [threading.Thread(target=s.step, args=(n,)) for s in sims]
    for t in ts:
        t.start()
    for t in ts:
        t.join(timeout=120)
        assert not t.is_alive()
    assert [s.state_hash() for s in sims] == [
        ref.state_hash()
    ] * 2  # concurrency does not change results


def test_gil_released_python_thread_progresses_during_step():
    n = _steps_for(0.5)
    s, done = pendulum(), threading.Event()

    def work():
        s.step(n)
        done.set()

    t = threading.Thread(target=work)
    t.start()
    ticks = 0
    while not done.wait(
        0.001
    ):  # if step held the GIL this loop would be starved until it finished
        ticks += 1
    t.join()
    assert ticks >= 5, f"main thread made only {ticks} iterations during a ~0.5 s step"
