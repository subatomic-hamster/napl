"""User program: X500 mission over MAVLink with MAVSDK-Python ($NAPL_MAVLINK = "udp://:14540", PX4's offboard stream):
arm, take off to 5 m, fly a 10 m square at 3 m/s (offboard position setpoints moving at 3 m/s: north, east, south, west back to
home), return to launch, land. Local NED frame throughout (origin = home), so nothing depends on PX4's AMSL altitude estimate.

    napl program examples/x500/mission.scenario.toml
    NAPL_MAVLINK=udp://:14540 python examples/x500/mission.py       # same program against any PX4 SITL / vehicle

Needs `pip install mavsdk` (BSD-3). Exits 1 (AssertionError) if a step times out."""

import asyncio
import math
import os
import time

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.offboard import OffboardAsync, PositionNedYaw
from mavsdk.asyncio.plugins.telemetry import LandedState, TelemetryAsync

ALT, SIDE, SPEED, DT = 5.0, 10.0, 3.0, 0.1  # m, m, m/s, setpoint period s
T0 = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - T0:6.1f}s] {msg}", flush=True)


async def main():
    st = {"ned": None, "landed": None, "armed": None, "health": None}
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
        tel, act, off = (
            TelemetryAsync(system),
            ActionAsync(system),
            OffboardAsync(system),
        )
        for gen, key in ((tel.subscribe_position_velocity_ned(), "ned"), (tel.subscribe_landed_state(), "landed"),
                         (tel.subscribe_armed(), "armed"), (tel.subscribe_health(), "health")):  # fmt: skip
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
        pos = lambda: st["ned"].position  # north_m, east_m, down_m
        log(
            f"ready at NED ({pos().north_m:.2f}, {pos().east_m:.2f}, {pos().down_m:.2f})"
        )
        await act.set_takeoff_altitude(ALT)
        await act.set_return_to_launch_altitude(
            ALT
        )  # default 60 m would climb away from the geofence
        ground = pos().down_m  # the estimator's down value on the ground is not 0 (it has a vertical offset): climb is relative
        await act.arm()
        await act.takeoff()
        await until(lambda: ground - pos().down_m > ALT - 0.5, 60, f"climb of {ALT} m")
        await asyncio.sleep(3)  # PX4 hands over to HOLD near the target; let it settle
        log(f"climbed {ground - pos().down_m:.1f} m")

        n0, e0, d0 = (
            pos().north_m,
            pos().east_m,
            pos().down_m,
        )  # the square is flown at this height, anchored where we are
        await off.set_position_ned(PositionNedYaw(n0, e0, d0, 0.0))
        await off.start()
        n, e = n0, e0
        for dn, de in (
            (SIDE, 0),
            (SIDE, SIDE),
            (0, SIDE),
            (0, 0),
        ):  # corners relative to the start: N, NE, E, back
            tn, te = n0 + dn, e0 + de
            dist = math.hypot(tn - n, te - e)
            for k in range(
                1, int(dist / (SPEED * DT)) + 1
            ):  # the setpoint moves at SPEED
                f = k * SPEED * DT / dist
                await off.set_position_ned(
                    PositionNedYaw(n + (tn - n) * f, e + (te - e) * f, d0, 0.0)
                )
                await asyncio.sleep(DT)
            n, e = tn, te
            await off.set_position_ned(PositionNedYaw(n, e, d0, 0.0))
            await until(
                lambda: math.hypot(pos().north_m - n, pos().east_m - e) < 0.7,
                30,
                f"corner ({dn}, {de}) m",
            )
            log(f"corner ({dn}, {de}) m reached")
        await off.stop()

        await act.return_to_launch()
        await until(
            lambda: st["landed"] == LandedState.ON_GROUND and st["armed"] is False,
            120,
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
