"""User program: SO-101 pick-and-place over the Feetech STS serial port ($NAPL_SERIAL: a pty from `napl program`, or the real
/dev/ttyACM0 - the code does not know the difference). Moves to a hover pose, grips the 20 mm cube, carries it 9 cm and releases.

    napl program examples/so101/pick.scenario.toml
    NAPL_SERIAL=/dev/ttyACM0 python examples/so101/pick.py          # same program on hardware (calibrate the constants first)

Workcell constants (measured from the built scene: `python examples/so101/scene.py`): table height and the P-only servo's
pid_scale, from which the gripper squeeze is derived (duty = P e / pid_scale, below the 25 % overload threshold)."""

import os
import sys
import time
from math import pi
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ik import BUS_HZ, G_OPEN, G_REF, HI, LO, plan
from sts import CPR, P_REG, Bus, configure, decode, goal_bytes

TABLE_Z, PID_SCALE = (
    -0.0024002597322628667,
    4000.0,
)  # m (table top = base bottom plane); servo pid_scale
CUBE, CUBE_XY, TARGET_XY = 0.020, (0.20, -0.05), (0.20, 0.04)
SQUEEZE_DUTY = 0.15  # gripper duty while holding the cube
HOME_S = 3.0  # joint-space move from wherever the arm is to the hover pose


def main():
    bus = Bus(os.environ["NAPL_SERIAL"])
    c = np.array([*CUBE_XY, TABLE_Z + CUBE / 2])
    t = np.array([*TARGET_XY, TABLE_Z + CUBE / 2])
    e_sq = SQUEEZE_DUTY * PID_SCALE / P_REG  # counts of goal beyond the contact angle
    rows, q_home = plan(c, t, G_REF - e_sq * 2 * pi / CPR)
    configure(bus)
    dt = 1.0 / BUS_HZ
    st = {i: decode(d) for i, d in bus.sync_read(56, 15, range(1, 7)).items()}
    q0 = np.array([st[i]["q"] for i in range(1, 7)])
    goal = np.array([*q_home, G_OPEN])
    n = round(
        HOME_S * BUS_HZ
    )  # smoothstep from the present pose to the hover pose, then the planned trajectory
    s = lambda k: (k / n) ** 3 * (10 - 15 * k / n + 6 * (k / n) ** 2)
    path = [(q0 + (goal - q0) * s(k), "go home") for k in range(1, n + 1)]
    path += [(np.array([*qa, g]), label) for qa, g, label in rows]
    off = np.zeros(5)  # integrates the P-only servo's gravity sag away (rate 3 /s)
    t_next = time.monotonic()
    for qd, label in path:
        time.sleep(max(0.0, t_next - time.monotonic()))
        t_next += dt
        qc = np.clip(qd[:5] + off, LO[:5], HI[:5])
        bus.sync_write(
            42,
            {i + 1: goal_bytes(v * CPR / (2 * pi)) for i, v in enumerate([*qc, qd[5]])},
        )
        st = {i: decode(d) for i, d in bus.sync_read(56, 15, range(1, 7)).items()}
        qm = np.array([st[i]["q"] for i in range(1, 6)])
        off = np.clip(off + 3.0 * dt * (qd[:5] - qm), -0.4, 0.4)
    bus.close()
    print(
        f"done: {len(path)} cycles, status errors {bus.errs[:3]}, bad frames {bus.bad}"
    )


if __name__ == "__main__":
    main()
