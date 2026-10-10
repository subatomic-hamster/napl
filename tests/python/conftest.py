"""`gpu`-marked tests need a Vulkan device: skipped when the first render says there is none.
`px4`-marked tests claim a free PX4 instance (so `pytest -n N` workers never share ports 4560+n / 14540+n / 14550+n / 18570+n)."""

import fcntl
import os
import time
from pathlib import Path

import napl
import pytest

_SCENE = Path(__file__).resolve().parents[1] / "data" / "render_scene.xml"
_probe = []  # [reason or None], filled once per session


def _no_device_reason():
    if not _probe:
        try:
            napl.Sim.load(_SCENE).render([-3, 0, 1], [0, 0, 0.25], 45.0, 16, 16)
            _probe.append(None)
        except RuntimeError as e:
            if "Vulkan" not in str(e):
                raise
            _probe.append(str(e))
    return _probe[0]


@pytest.fixture(autouse=True)
def _gpu_skip(request):
    if request.node.get_closest_marker("gpu"):
        reason = _no_device_reason()
        if reason:
            pytest.skip(f"no usable Vulkan device: {reason}")


# PX4 instances 0..9 (px4-rc.mavlink routes every instance above 9 to the same offboard port 14549). The preferred one is
# NAPL_PX4_INSTANCE (default 0), as before; a flock per instance keeps concurrent xdist workers apart, and ports already
# bound by anything else (another worker's MC on instance 5-6, ...) are skipped.
@pytest.fixture(autouse=True)
def _px4_instance(request, monkeypatch):
    if not request.node.get_closest_marker("px4"):
        yield
        return
    from napl import px4

    first = int(os.environ.get("NAPL_PX4_INSTANCE", "0"))
    deadline = time.time() + 1800
    while True:
        for k in range(10):
            n = (first + k) % 10
            lock = open(f"/tmp/napl-px4-instance-{n}.lock", "w")
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                lock.close()
                continue
            st = px4.Stack("cfg.toml", "/nonexistent", instance=n)
            if px4._busy("tcp", st.tcp) or px4._busy("udp", st.udp):
                lock.close()
                continue
            monkeypatch.setenv("NAPL_PX4_INSTANCE", str(n))
            try:
                yield
            finally:
                lock.close()
            return
        if time.time() > deadline:
            pytest.fail("no free PX4 instance 0-9 within 30 min")
        time.sleep(5)
