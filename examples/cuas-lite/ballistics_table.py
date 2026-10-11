"""Regenerates the reference ballistics table of docs/cuas-lite/spec.md 5.3 from the code's model (napl.cuas.ballistics.solve_lead, RK4 dt = 1 ms, the Rival ball
BALL of napl.lite.aim, no wind, flat target at muzzle height) and prints it as markdown rows:

    .venv/bin/python examples/cuas-lite/ballistics_table.py

tests/lite/test_aim.py::test_spec_ballistics_table checks the code against the same numbers."""

import math

import numpy as np
from napl.cuas import ballistics
from napl.lite import aim

RANGES = (3, 5, 8, 10, 12, 15)
M = np.array(
    [0.0, 0.0, 4.0]
)  # high enough that solve_lead's flat first guess does not reach the ground


def row(rng):
    tgt = M + [rng, 0.0, 0.0]
    solve = lambda k_v, v=(0.0, 0.0, 0.0): ballistics.solve_lead(
        aim.pellet(k_v), M, tgt, np.array(v), np.zeros(3), dt=0.001
    )
    s, s35, sc = solve(1.0), solve(35 / 30), solve(1.0, (0.0, 2.0, 0.0))
    return (
        math.degrees(s["el"]),
        s["t_flight"],
        s["speed"],
        rng * math.tan(abs(sc["az"])),
        math.degrees(s35["el"]),
    )


if __name__ == "__main__":
    print(
        "| range m | elev deg (30 m/s) | flight s | impact m/s | lead m at 2 m/s crossing | elev deg at 35 m/s |"
    )  # ttsr-ignore: py/no-print (CLI)
    print("|---|---|---|---|---|---|")  # ttsr-ignore: py/no-print (CLI)
    for r in RANGES:
        e, tf, v, lead, e35 = row(r)
        print(
            f"| {r} | {e:.2f} | {tf:.2f} | {v:.1f} | {lead:.2f} | {e35:.2f} |"
        )  # ttsr-ignore: py/no-print (CLI)
