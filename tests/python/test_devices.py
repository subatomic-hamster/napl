"""N2 devices (tests/data/devices/all.xml: quad + battery + ESCs + rotors + servo + environment + IMU/baro/mag/GNSS)
through the public Python API and through the MCP tool path of test_mcp.py."""

import asyncio
import json
import shutil
from pathlib import Path

import napl
import numpy as np
import pytest
from mcp import Client
from napl.mcp_server import NaplServer

ALL = Path(__file__).resolve().parents[2] / "tests" / "data" / "devices" / "all.xml"
SENSORS = {
    ("/devices/imu", "accel"): ("m/s^2", 3),
    ("/devices/imu", "gyro"): ("rad/s", 3),
    ("/devices/baro", "pressure"): ("Pa", 1),
    ("/devices/mag", "field"): ("T", 3),
    ("/devices/gnss", "lat"): ("deg", 1),
    ("/devices/gnss", "vel_ned"): ("m/s", 3),
    ("/devices/esc1", "omega"): ("rad/s", 1),
    ("/devices/pack", "voltage"): ("V", 1),
}


def run_throttle(throttle: float, steps: int = 60) -> napl.Sim:
    s = napl.Sim.load(ALL)
    s.set("/devices/esc1", "throttle", throttle)
    s.step(steps)
    return s


def test_sim_query_reads_every_device_family():
    s = run_throttle(0.45)
    for (path, attr), (unit, n) in SENSORS.items():
        values, got_unit = s.query(path, attr)
        assert got_unit == unit, (path, attr)
        assert values.dtype == np.float64 and values.shape == (n,), (path, attr)
        assert np.all(np.isfinite(values)), (path, attr)
    accel, _ = s.query("/devices/imu", "accel")
    # free fall (reads ~0) plus the thrust of the one spinning rotor on 1.27 kg (~2 m/s^2) and the sensor biases
    assert 0.5 < np.linalg.norm(accel) < 15.0
    pressure, _ = s.query("/devices/baro", "pressure")
    assert 90000 < pressure[0] < 100000  # ~ ISA at 320 m
    with pytest.raises(KeyError):
        s.query("/devices/imu", "no_such_attr")
    with pytest.raises(KeyError):
        s.query("/devices/no_such_device", "accel")


def test_set_esc_throttle_changes_device_state():
    spun, idle = run_throttle(0.45), run_throttle(0.0)
    assert abs(idle.query("/devices/esc1", "omega")[0][0]) < 1e-9
    assert spun.query("/devices/esc1", "omega")[0][0] > 50.0
    assert (
        spun.query("/devices/esc1", "current")[0][0]
        > idle.query("/devices/esc1", "current")[0][0]
    )
    assert (
        spun.query("/devices/pack", "soc")[0][0]
        < idle.query("/devices/pack", "soc")[0][0]
    )
    assert spun.state_hash() != idle.state_hash()
    with pytest.raises(ValueError):
        spun.set(
            "/devices/esc1", "throttle", 1.5
        )  # out of range: rejected, nothing written
    with pytest.raises(ValueError):
        spun.set("/devices/imu", "accel", [0, 0, 0])  # sensors are read-only
    assert spun.query("/devices/esc1", "throttle")[0][0] == 0.45


def test_device_state_is_part_of_save_and_fork():
    s = run_throttle(0.45)
    h, blob = s.state_hash(), s.save()
    f = s.fork()
    assert f.state_hash() == h
    s.step(37)
    assert s.state_hash() != h
    s.restore(blob)
    assert s.state_hash() == h
    s.step(37)
    f.step(37)
    assert s.state_hash() == f.state_hash()


async def _ok(c, name, **args):
    r = await c.call_tool(name, args)
    assert not r.is_error, r.content[0].text
    return json.loads(r.content[0].text)


def test_mcp_get_and_set_device_state(tmp_path):
    shutil.copy(ALL, tmp_path / "all.xml")

    async def main():
        async with Client(NaplServer(tmp_path, allow_destructive=True).server) as c:
            await _ok(c, "load_model", path="all.xml")
            prims = (await _ok(c, "list_prims"))["prims"]
            await _ok(c, "step_simulation", n=40)
            # the listing carries the device prims; values and unit come back with full float64 precision
            got = {}
            for path, attr in [
                ("/devices/imu", "accel"),
                ("/devices/baro", "pressure"),
                ("/devices/esc1", "omega"),
            ]:
                assert path in prims, path
                got[path] = await _ok(c, "get_entity_state", path=path, attr=attr)
            assert (
                got["/devices/imu"]["unit"] == "m/s^2"
                and len(got["/devices/imu"]["values"]) == 3
            )
            assert (
                got["/devices/baro"]["unit"] == "Pa"
                and 90000 < got["/devices/baro"]["values"][0] < 100000
            )
            assert got["/devices/esc1"]["unit"] == "rad/s"
            assert (
                abs(got["/devices/esc1"]["values"][0]) < 1e-9
            )  # throttle 0: not spinning
            # an esc throttle set through the tool changes the device state
            await _ok(
                c,
                "set_entity_state",
                path="/devices/esc1",
                attr="throttle",
                values=[0.5],
            )
            assert (
                await _ok(c, "get_entity_state", path="/devices/esc1", attr="throttle")
            )["values"] == [0.5]
            await _ok(c, "step_simulation", n=60)
            omega = await _ok(c, "get_entity_state", path="/devices/esc1", attr="omega")
            assert omega["values"][0] > 50.0
            # a bad value is an error and writes nothing
            bad = await c.call_tool(
                "set_entity_state",
                {"path": "/devices/esc1", "attr": "throttle", "values": [2.0]},
            )
            assert bad.is_error
            assert (
                await _ok(c, "get_entity_state", path="/devices/esc1", attr="throttle")
            )["values"] == [0.5]

    asyncio.run(main())
