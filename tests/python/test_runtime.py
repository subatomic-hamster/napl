"""Execution scheduling must not alter scientific state or report partial batches."""
import os
from pathlib import Path
import subprocess
import sys
import threading

import napl
import pytest
from napl import runtime


@pytest.mark.parametrize("profile", ["background", "balanced", "maximum"])
def test_policy_preserves_snapshot_continuation(profile):
    model = Path(__file__).resolve().parents[1] / "data" / "pendulum.xml"
    previous = runtime.status()["requested_profile"]
    try:
        sim = napl.Sim.load(model)
        runtime.configure("maximum")
        sim.step(37)
        saved = sim.save()
        sim.step(263)
        expected = sim.state_hash()
        sim.restore(saved)
        runtime.configure(profile)
        sim.step(263)
        assert sim.tick == 300
        assert sim.state_hash() == expected
    finally:
        runtime.configure(previous)


def test_impossible_memory_budget_rejects_worker():
    with pytest.raises(runtime.ResourcePressureError):
        runtime.worker_limit(gb_per_worker=1e12)


def test_batch_does_not_submit_after_critical_pressure(monkeypatch):
    started = []
    lock = threading.Lock()

    def snapshot(*args):
        with lock:
            return {"workers": 0 if started else 1, "requested_profile": "background", "reason": "injected pressure"}

    def job(seed):
        with lock:
            started.append(seed)
        return seed

    monkeypatch.setattr(runtime, "status", snapshot)
    with pytest.raises(runtime.ResourcePressureError):
        runtime.bounded_map(job, range(5), workers=1, threads=True)
    assert started == [0]


def test_batch_results_retain_input_order(monkeypatch):
    first_waiting = threading.Event()
    second_done = threading.Event()

    def job(seed):
        if seed == 0:
            first_waiting.set()
            assert second_done.wait(5)
        else:
            assert first_waiting.wait(5)
            second_done.set()
        return seed * 7

    monkeypatch.setattr(runtime, "status", lambda *args: {
        "workers": 2, "requested_profile": "maximum", "reason": "test budget"})
    assert runtime.bounded_map(job, [0, 1], workers=2, threads=True) == [0, 7]


def test_process_batch_recovers_main_dataclass_for_imported_target(tmp_path):
    """Fresh children must rebuild __main__ classes even when the target is imported (dataclasses.asdict)."""
    script = tmp_path / "driver.py"
    script.write_text(
        "from dataclasses import asdict, dataclass\n"
        "from napl.runtime import bounded_map\n"
        "@dataclass\nclass Job:\n    value: int\n    label: str\n"
        "if __name__ == '__main__':\n"
        "    print(bounded_map(asdict, [Job(7, 'a'), Job(9, 'b')], workers=2))\n"
    )
    run = subprocess.run([sys.executable, str(script)], capture_output=True, text=True,
                         timeout=120, env={**os.environ, "NAPL_PROFILE": "maximum"})
    assert run.returncode == 0, run.stderr
    assert run.stdout.strip() == "[{'value': 7, 'label': 'a'}, {'value': 9, 'label': 'b'}]"
