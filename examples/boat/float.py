"""Boat float: settle at the draft, a straight leg (both thrusters at 50 %), then a 60 degree turn to starboard, a heading-hold autopilot
steering by differential thrust throughout. Native program: owns the Sim, writes trace.csv / replay.naplog / native.json (napl.marine.write_native). Env: NAPL_MODEL, NAPL_RUN_DIR."""

import math
import os

import napl
from napl.marine import write_native

DT, EVERY, CTRL = 0.002, 8, 10  # step, trace every 16 ms, autopilot every 20 ms
T_SETTLE, T_STRAIGHT, T_TURN = 6.0, 20.0, 14.0
CRUISE, KP, KD = 0.5, 0.4, 0.8  # throttle, P per rad, D per rad/s
TURN_DEG, TURN_RAMP = (
    -60.0,
    8.0,
)  # heading change to starboard, reached at a constant rate over this many seconds
COLS = [
    "t",
    "x_m",
    "y_m",
    "z_m",
    "speed_ms",
    "yaw_deg",
    "roll_deg",
    "pitch_deg",
    "thrust_N",
    "soc",
    "submerged_m3",
    "imm",
]


def q(sim, path, attr, i=0):
    return float(sim.query(path, attr)[0][i])


def main():
    sim = napl.Sim.load(os.environ["NAPL_MODEL"])
    sim.start_log()  # before any command: the replay log starts at tick 0
    rows, prev = [], None
    n = int((T_SETTLE + T_STRAIGHT + T_TURN) / DT)
    yaw_prev = 0.0
    for k in range(n):
        t = k * DT
        w, x, y, z = sim.query("/world/hull", "quat")[0]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if (
            k % CTRL == 0 and t >= T_SETTLE
        ):  # heading autopilot: differential thrust (the bare hull is directionally unstable, Munk moment)
            target = math.radians(TURN_DEG) * max(
                0.0, min(1.0, (t - T_SETTLE - T_STRAIGHT) / TURN_RAMP)
            )
            err = math.remainder(target - yaw, 2 * math.pi)
            rate = math.remainder(yaw - yaw_prev, 2 * math.pi) / (CTRL * DT)
            diff = max(-0.3, min(0.3, KP * err - KD * rate))
            sim.set(
                "/devices/esc1", "throttle", CRUISE - diff
            )  # port (+y) thruster pushes the bow to starboard
            sim.set("/devices/esc2", "throttle", CRUISE + diff)
        if k % CTRL == 0:
            yaw_prev = yaw
        sim.step(1)
        if k % EVERY == 0:
            w, x, y, z = sim.query("/world/hull", "quat")[0]
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            p = sim.query("/world/hull", "pos")[0]
            v = (
                0.0
                if prev is None
                else math.hypot(p[0] - prev[0], p[1] - prev[1]) / (EVERY * DT)
            )
            prev = p
            rows.append(
                [
                    sim.tick * DT, p[0], p[1], p[2], v,
                    math.degrees(yaw), math.degrees(roll), math.degrees(pitch),
                    q(sim, "/devices/rotor1", "thrust") + q(sim, "/devices/rotor2", "thrust"),
                    q(sim, "/devices/battery1", "soc"),
                    q(sim, "/devices/vessel", "submerged"),
                    q(sim, "/devices/rotor1", "ge_ratio"),
                ]
            )  # fmt: skip
    write_native(sim, DT, COLS, rows)


if __name__ == "__main__":
    main()
