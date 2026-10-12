"""napl.attach: a Sim whose World lives in another process (here: another thread with its own socket), paced to the wall clock."""

import time
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import attach

ARM3 = Path(__file__).resolve().parents[2] / "tests" / "data" / "feetech" / "arm3.xml"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@needs_kernel
def test_remote_sim_drives_the_served_sim_and_keeps_exception_types(tmp_path):
    host = napl.Sim.load(ARM3, host=True)
    srv = attach.Server(host, tmp_path / "s", 0.001, 0.0)
    try:
        sim = napl.Sim(attach.Remote(srv.path))
        sim.start_log()  # no-op: the served Sim logs from tick 0 already
        assert sim.step(50) == 50 == host.tick == sim.tick
        np.testing.assert_allclose(
            sim.query("/devices/s1", "position")[0],
            host.query("/devices/s1", "position")[0],
        )
        sim.set("/devices/s1", "torque_enable", 1)
        assert (
            host.query("/devices/s1", "torque_enable")[0][0] == 1
        )  # the served World changed
        assert sim.state_hash() == host.state_hash() and sim.log() == host.log()
        with pytest.raises(KeyError):
            sim.query("/devices/nope", "position")
        with pytest.raises(ValueError):
            sim.set("/devices/s1", "no_such_attr", 1)
        assert sim.step(1) == 51  # still usable after errors
    finally:
        srv.close()


@needs_kernel
def test_step_is_paced_to_real_time(tmp_path):
    host = napl.Sim.load(ARM3, host=True)
    srv = attach.Server(host, tmp_path / "s", 0.001, 1.0)
    try:
        sim = napl.Sim(attach.Remote(srv.path))
        t0 = time.monotonic()
        for _ in range(20):
            sim.step(10)  # 0.2 s of sim time
        assert 0.19 < time.monotonic() - t0 < 0.5
    finally:
        srv.close()
