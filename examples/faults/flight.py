"""User program for the X500 fault scenarios (examples/faults/*.scenario.toml): arm, take off to 5 m, settle in HOLD (position
control), print HOLDING (the scenarios' `log_marker`: the fault is injected relative to that line), then watch PX4 for
$FAULT_HOLD_S seconds (default 30) and log what it does: every flight-mode change, every PX4 status text, the position estimate's
health, the battery voltage and the position. Afterwards it lands (if still flying) and waits for disarm. It never fails because of
the fault - the scenario's monitors and the test judge the outcome - only if the setup (GPS, arm, takeoff) does not work.

Lines (wall seconds ~ sim seconds: the bridge runs lockstep at speed 1.0):  MODE <name> | STATUS <text> | GPS_OK <bool> |
BATT <volts> | POS n e d | HOLDING | TOUCHDOWN | LANDED | DONE.

    napl program examples/faults/gnss_outage.scenario.toml"""

import asyncio
import os
import time

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.telemetry import LandedState, TelemetryAsync

ALT, HOLD_S = 5.0, float(os.environ.get("FAULT_HOLD_S", "30"))
T0 = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - T0:6.1f}s] {msg}", flush=True)


async def main():
    st = {k: None for k in ("ned", "landed", "armed", "health", "mode", "batt")}
    mv = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
    tasks = []

    async def sub(gen, key, show=None):
        async for x in gen:
            old, st[key] = st[key], x
            if show and show(old) != show(x):
                log(show(x))

    async def texts(gen):
        async for x in gen:
            log(f"STATUS {x.text}")

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
        for gen, key, show in (
            (tel.subscribe_position_velocity_ned(), "ned", None),
            (tel.subscribe_landed_state(), "landed", None),
            (tel.subscribe_armed(), "armed", None),
            (tel.subscribe_health(), "health", lambda h: f"GPS_OK {bool(h and h.is_global_position_ok)}"),
            (tel.subscribe_flight_mode(), "mode", lambda m: f"MODE {m.name if m else None}"),
            (tel.subscribe_battery(), "batt", None),
        ):  # fmt: skip
            tasks.append(asyncio.create_task(sub(gen, key, show)))
        tasks.append(asyncio.create_task(texts(tel.subscribe_status_text())))
        await until(
            lambda: (
                st["health"]
                and st["health"].is_global_position_ok
                and st["health"].is_home_position_ok
            ),
            150,
            "GPS + home",
        )
        await until(
            lambda: st["health"].is_armable and st["ned"] and st["batt"], 60, "armable"
        )
        await asyncio.sleep(5)
        pos = lambda: st["ned"].position
        await act.set_takeoff_altitude(ALT)
        ground = pos().down_m
        await act.arm()
        await act.takeoff()
        await until(lambda: ground - pos().down_m > ALT - 0.5, 60, f"climb of {ALT} m")
        await asyncio.sleep(5)  # PX4 hands over to HOLD near the target; let it settle
        log(
            f"HOLDING climb {ground - pos().down_m:.1f} m, battery {st['batt'].voltage_v:.2f} V"
        )
        t_hold, last = time.monotonic(), 0.0
        while time.monotonic() - t_hold < HOLD_S:
            await asyncio.sleep(0.25)
            if time.monotonic() - last >= 1.0:
                last = time.monotonic()
                p = pos()
                log(f"BATT {st['batt'].voltage_v:.2f}")
                log(f"POS {p.north_m:.2f} {p.east_m:.2f} {p.down_m:.2f}")
            if (
                st["landed"] == LandedState.ON_GROUND
                and time.monotonic() - t_hold > 1.0
            ):
                log("TOUCHDOWN")
                break
        if st["armed"] and st["landed"] != LandedState.ON_GROUND:
            await act.land()
            end = time.monotonic() + 40
            while time.monotonic() < end and st["armed"]:
                await asyncio.sleep(0.2)
        log(
            "LANDED"
            if st["landed"] == LandedState.ON_GROUND
            else f"NOT_LANDED {st['landed']}"
        )
        log("DONE")
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        mv.destroy()


if __name__ == "__main__":
    asyncio.run(main())
