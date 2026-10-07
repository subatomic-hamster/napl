"""User program 2 of the N7 showcase: the loaded X500 flies over MAVLink with MAVSDK-Python ($NAPL_MAVLINK = "udp://:14540", PX4's
offboard stream): arm, take off to 4 m, hover, hop 4 m north and back (offboard position setpoints at 1.5 m/s), then land 0.8 m north of
the start (away from the arm's shelf, which is on the south side) and disarm. Local NED frame, origin = home.

    napl program examples/showcase/arm_loads_drone.scenario.toml     # runs arm_program.py first, then this
    NAPL_MAVLINK=udp://:14540 python examples/showcase/fly.py        # the same program against any PX4 SITL / vehicle

Needs `pip install mavsdk` (BSD-3). Exits 1 (AssertionError) if a step times out."""

import asyncio
import math
import os
import time

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.offboard import OffboardAsync, PositionNedYaw
from mavsdk.asyncio.plugins.telemetry import LandedState, TelemetryAsync

ALT, HOP, BACK, SPEED, DT, HOVER_S = (
    4.0,
    4.0,
    0.8,
    1.5,
    0.1,
    8.0,
)  # m, m north, m north of home to land, m/s, s, s
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
        ground = (
            pos().down_m
        )  # the estimator's down value on the ground is not 0: climb is relative
        await act.arm()
        await act.takeoff()
        await until(lambda: ground - pos().down_m > ALT - 0.5, 60, f"climb of {ALT} m")
        await asyncio.sleep(
            HOVER_S
        )  # hover (PX4 hands over to HOLD near the target): the run's hover-current window
        log(f"climbed {ground - pos().down_m:.1f} m")

        n0, e0, d0 = pos().north_m, pos().east_m, pos().down_m
        await off.set_position_ned(PositionNedYaw(n0, e0, d0, 0.0))
        await off.start()
        n = n0
        for tn in (
            n0 + HOP,
            n0 + BACK,
        ):  # out 4 m north, then back to 0.8 m north of home
            for k in range(1, int(abs(tn - n) / (SPEED * DT)) + 1):
                await off.set_position_ned(
                    PositionNedYaw(
                        n + math.copysign(k * SPEED * DT, tn - n), e0, d0, 0.0
                    )
                )
                await asyncio.sleep(DT)
            n = tn
            await off.set_position_ned(PositionNedYaw(n, e0, d0, 0.0))
            await until(
                lambda: math.hypot(pos().north_m - n, pos().east_m - e0) < 0.7,
                30,
                f"north {n - n0:.1f} m",
            )
            log(f"at north {n - n0:.1f} m")
        await off.stop()
        await act.land()
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
