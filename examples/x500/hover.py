"""User program: arm, take off to 5 m, hold HOLD_S seconds, land (MAVSDK-Python on $NAPL_MAVLINK). The flight of
examples/x500/hover.design.scenario.toml and of tests/python/test_px4_design.py (designs built / edited with napl.design)."""

import asyncio
import os
import time

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.telemetry import FlightMode, LandedState, TelemetryAsync

ALT, HOLD_S = 5.0, float(os.environ.get("HOVER_HOLD_S", "20"))
T0 = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - T0:6.1f}s] {msg}", flush=True)


async def main():
    st = {"ned": None, "landed": None, "armed": None, "health": None, "mode": None}
    mv = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    tasks = []

    async def sub(gen, key):
        async for x in gen:
            st[key] = x

    async def until(cond, timeout, what):
        end = time.monotonic() + timeout
        while not cond():
            assert time.monotonic() < end, (
                f"timeout {timeout}s waiting for {what}: {st}"
            )
            await asyncio.sleep(0.1)

    try:
        await mv.add_any_connection(os.environ["NAPL_MAVLINK"])
        system = await mv.first_autopilot(timeout_s=60.0)
        assert system is not None, "no autopilot on the MAVLink link"
        tel, act = TelemetryAsync(system), ActionAsync(system)
        for gen, key in ((tel.subscribe_position_velocity_ned(), "ned"), (tel.subscribe_landed_state(), "landed"),
                         (tel.subscribe_armed(), "armed"), (tel.subscribe_health(), "health"),
                         (tel.subscribe_flight_mode(), "mode")):  # fmt: skip
            tasks.append(asyncio.create_task(sub(gen, key)))
        await until(
            lambda: (
                st["health"]
                and st["health"].is_global_position_ok
                and st["health"].is_home_position_ok
            ),
            150,
            "GPS + home",
        )
        await until(lambda: st["health"].is_armable and st["ned"], 60, "armable")
        await asyncio.sleep(5)  # let the estimator settle
        ground = st["ned"].position.down_m
        await act.set_takeoff_altitude(ALT)
        await act.arm()
        await act.takeoff()
        await until(
            lambda: ground - st["ned"].position.down_m > ALT - 0.5,
            60,
            f"climb of {ALT} m",
        )
        await until(lambda: st["mode"] == FlightMode.HOLD, 60, "HOLD")
        log(f"at {ground - st['ned'].position.down_m:.2f} m, holding {HOLD_S:.0f} s")
        await asyncio.sleep(HOLD_S)
        await act.land()
        await until(
            lambda: st["landed"] == LandedState.ON_GROUND and st["armed"] is False,
            90,
            "landed + disarmed",
        )
        log("landed and disarmed")
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        mv.destroy()


if __name__ == "__main__":
    asyncio.run(main())
