"""Rocket hop: stand on the legs, ignite the E16, climb, burn out, coast, fall back onto the legs. Native program: owns the Sim, writes
trace.csv / replay.naplog / native.json (napl.marine.write_native). Env: NAPL_MODEL, NAPL_RUN_DIR; HOP_WIND (m/s, +x, default 0), HOP_SIM_S."""

import math
import os

import napl
from napl.marine import write_native

DT, EVERY = 0.001, 16
SIM_S = float(os.environ.get("HOP_SIM_S", "14"))
COLS = [
    "t",
    "x_m",
    "y_m",
    "z_m",
    "vz_ms",
    "tilt_deg",
    "alpha_deg",
    "thrust_N",
    "mass_kg",
    "phase",
    "impulse_Ns",
]


def q(sim, path, attr, i=0):
    return float(sim.query(path, attr)[0][i])


def main():
    sim = napl.Sim.load(os.environ["NAPL_MODEL"])
    sim.start_log()  # before any command: the replay log starts at tick 0
    sim.set("/devices/air", "wind", [float(os.environ.get("HOP_WIND", "0")), 0.0, 0.0])
    rows, z0 = [], None
    for k in range(int(SIM_S / DT)):
        if k == 500:  # half a second on the pad first
            sim.set("/devices/motor", "ignite", 1)
        sim.step(1)
        if k % EVERY == 0:
            qw, qx, qy, qz = sim.query("/world/tube", "quat")[0]
            up_z = 1 - 2 * (qx * qx + qy * qy)  # world z of the body +z axis
            z0 = z0 if z0 is not None else q(sim, "/world/tube", "pos", 2)
            rows.append(
                [
                    sim.tick * DT,
                    q(sim, "/world/tube", "pos", 0),
                    q(sim, "/world/tube", "pos", 1),
                    q(sim, "/world/tube", "pos", 2) - z0,
                    0.0,  # vz: filled in below by differencing
                    math.degrees(math.acos(max(-1.0, min(1.0, up_z)))),
                    q(sim, "/devices/aero", "alpha") * 57.29577951308232,
                    q(sim, "/devices/motor", "thrust"),
                    q(sim, "/devices/motor", "mass"),
                    q(sim, "/devices/motor", "phase"),
                    q(sim, "/devices/motor", "impulse"),
                ]
            )
    for a, b in zip(rows, rows[1:]):  # vertical speed by differencing the trace
        a[4] = (b[3] - a[3]) / (b[0] - a[0])
    write_native(sim, DT, COLS, rows)


if __name__ == "__main__":
    main()
