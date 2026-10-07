"""User program 1 of the N7 showcase: the SO-101 over the Feetech STS serial port ($NAPL_SERIAL: a pty from `napl program`, or the real
/dev/ttyACM0) picks the 20 mm stainless payload off its shelf, places it on the X500's top plate, then parks over the shelf, clear of the
payload, so the drone can lift off. Same protocol client and IK as examples/so101/pick.py (the 5 arm joints by damped least squares
on the URDF forward kinematics); only the workcell differs: pick 0.16 m and place 0.25 m in front of the arm's pan axis (arm frame:
x forward, z up, origin on the pan axis; the base's bottom plane is z = TABLE_Z, which is also the top-plate height, see
showcase_scene.py), and a last move back to the pick hover pose.

    napl program examples/showcase/arm_loads_drone.scenario.toml

Constants measured from the built scene (python examples/so101/scene.py): TABLE_Z, and the P-only servo's pid_scale, from which the
gripper squeeze is derived (duty = P e / pid_scale, below the 25 % overload threshold). The payload (63 g) needs a normal force of
about 1.2 N at friction 0.4 with margin; the squeeze gives several N."""

import os
import sys
import time
from math import pi
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "so101"))
import ik
from ik import BUS_HZ, G_OPEN, G_REF, HI, HOVER, LO, plan
from sts import CPR, P_REG, Bus, configure, decode, goal_bytes

TABLE_Z, PID_SCALE = (
    -0.0024002597322628667,
    4000.0,
)  # m (base bottom plane), servo pid_scale
CUBE, R_PICK, R_PLACE = 0.020, 0.16, 0.25
SQUEEZE_DUTY = 0.15
HOME_S = 3.0  # joint-space move from the powered-up pose to the hover pose
PARK_S = 1.5


def main():
    bus = Bus(os.environ["NAPL_SERIAL"])
    c = np.array([R_PICK, 0.0, TABLE_Z + CUBE / 2])
    t = np.array([R_PLACE, 0.0, TABLE_Z + CUBE / 2])
    e_sq = SQUEEZE_DUTY * PID_SCALE / P_REG  # counts of goal beyond the contact angle
    rows, q_home = plan(c, t, G_REF - e_sq * 2 * pi / CPR)
    q, g = (
        rows[-1][0].copy(),
        rows[-1][1],
    )  # park: back over the shelf, above the pick point
    up, xd, a, b = (
        np.array([0, 0, HOVER]),
        np.array([0.0, 1.0, 0.0]),
        t + [0, 0, HOVER],
        c + [0, 0, HOVER],
    )
    for k in range(1, round(PARK_S * BUS_HZ) + 1):
        q, r = ik.ik(a + (b - a) * ik._smooth(k / (PARK_S * BUS_HZ)), xd, q)
        assert r < 1e-4, f"park: IK error {r} m"
        rows.append((q.copy(), g, "park"))
    rows += [(q.copy(), g, "parked")] * round(1.0 * BUS_HZ)
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
    path += [(np.array([*qa, gg]), label) for qa, gg, label in rows]
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
