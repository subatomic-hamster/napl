"""User program: Koch v1.1 pick-and-place over a Dynamixel Protocol 2.0 serial port ($NAPL_SERIAL: a pty from `napl program`, or the
real /dev/ttyUSB0 - the code does not know the difference), written with the unmodified ROBOTIS `dynamixel_sdk` (extra `arm`) the way
LeRobot's koch_follower drives the bus: torque off, Operating Mode 4 (extended position) on joints 1-5 and 5 (current-based position) on
the gripper, elbow_flex P / I / D = 1500 / 0 / 600, torque on, then Goal Position by GroupSyncWrite and Present Position by
GroupSyncRead at 50 Hz. Moves to a hover pose, grips the 20 mm cube, carries it 10 cm and releases.

    napl program examples/koch/pick.scenario.toml
    NAPL_SERIAL=/dev/ttyUSB0 python examples/koch/pick.py          # same program on hardware (calibrate the constants first)

Workcell constants (measured from the built scene: `python examples/koch/scene.py`): table height; the gripper servo's P-only
pid_scale (XL330 device map, assemblies/koch.toml), from which the squeeze is derived (duty = P e / pid_scale)."""

import os
import sys
import time
from math import pi
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from koch_ik import BUS_HZ, G_OPEN, G_REF, HI, LO, plan

try:
    from dynamixel_sdk import (
        COMM_SUCCESS,
        GroupSyncRead,
        GroupSyncWrite,
        PacketHandler,
        PortHandler,
    )
except ImportError:
    sys.exit(
        "dynamixel_sdk is not installed: pip install 'dynamixel-sdk==4.1.0' (the napl extra `arm`)"
    )

# XL430-W250-T / XL330-M288-T control table addresses (e-Manual)
RETURN_DELAY, OPERATING_MODE, TORQUE = 9, 11, 64
POS_D, POS_I, POS_P, GOAL_POS, PRESENT_POS = 80, 82, 84, 116, 132
EXTENDED_POSITION, CURRENT_BASED_POSITION = 4, 5
CPR = 4096
IDS = range(1, 7)
TABLE_Z, PID_SCALE, GRIPPER_P = (
    1.6024095383926351e-09,
    113280.0,
    400,
)  # m; XL330 pid_scale; its default Position P Gain
CUBE, CUBE_XY, TARGET_XY = 0.020, (-0.12, -0.05), (-0.12, 0.05)
SQUEEZE_DUTY = 0.12  # gripper duty while holding the cube
HOME_S = 3.0  # joint-space move from wherever the arm is to the hover pose
YDIR = np.array(
    [0.0, 1.0, 0.0]
)  # jaw opening direction (world y), fingers pointing down
Q_SEED = [0.0, -0.3, 0.3, -1.0, 0.0]  # IK start: elbow up, wrist flexed down


class Bus:
    """The LeRobot DynamixelMotorsBus calls this program needs, on dynamixel_sdk."""

    def __init__(self, path):
        self.ph, self.pk = PortHandler(path), PacketHandler(2.0)
        if not (self.ph.openPort() and self.ph.setBaudRate(1_000_000)):
            sys.exit(f"cannot open {path} at 1 Mbaud")
        self.gw = GroupSyncWrite(self.ph, self.pk, GOAL_POS, 4)
        self.gr = GroupSyncRead(self.ph, self.pk, PRESENT_POS, 4)
        for i in IDS:
            self.gr.addParam(i)

    def write(self, i, addr, value, n):
        fn = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx}[n]
        res, err = fn(self.ph, i, addr, value)
        if res != COMM_SUCCESS or err:
            raise RuntimeError(
                f"write id {i} addr {addr}: {self.pk.getTxRxResult(res)} / {self.pk.getRxPacketError(err)}"
            )

    def configure(self):
        """koch_follower.configure(): inside torque_disabled(), return delay 0, operating modes, elbow_flex gains; then torque on."""
        for i in IDS:
            self.write(i, TORQUE, 0, 1)
        for i in IDS:
            self.write(i, RETURN_DELAY, 0, 1)
            self.write(
                i,
                OPERATING_MODE,
                CURRENT_BASED_POSITION if i == 6 else EXTENDED_POSITION,
                1,
            )
        self.write(3, POS_P, 1500, 2)
        self.write(3, POS_I, 0, 2)
        self.write(3, POS_D, 600, 2)
        for i in IDS:
            self.write(i, TORQUE, 1, 1)

    def read_q(self):
        """Present Position of all six servos -> radians (the zero pose is count 0, no homing offset)."""
        if self.gr.txRxPacket() != COMM_SUCCESS:
            raise RuntimeError("sync read failed")
        return (
            np.array([self.s32(self.gr.getData(i, PRESENT_POS, 4)) for i in IDS])
            * 2
            * pi
            / CPR
        )

    def write_goals(self, q):
        for i, v in zip(IDS, q):
            self.gw.addParam(
                i,
                [
                    int(b)
                    for b in int(round(v * CPR / (2 * pi))).to_bytes(
                        4, "little", signed=True
                    )
                ],
            )
        res = self.gw.txPacket()
        self.gw.clearParam()
        if res != COMM_SUCCESS:
            raise RuntimeError("sync write failed")

    @staticmethod
    def s32(x):
        return x - (1 << 32) if x >= 1 << 31 else x


def main():
    bus = Bus(os.environ["NAPL_SERIAL"])
    c = np.array([*CUBE_XY, TABLE_Z + CUBE / 2])
    t = np.array([*TARGET_XY, TABLE_Z + CUBE / 2])
    e_sq = (
        SQUEEZE_DUTY * PID_SCALE / GRIPPER_P
    )  # counts of goal beyond the contact angle
    rows, q_home = plan(c, t, G_REF + e_sq * 2 * pi / CPR, Q_SEED, YDIR)
    bus.configure()
    dt = 1.0 / BUS_HZ
    q0 = bus.read_q()
    goal = np.array([*q_home, G_OPEN])
    n = round(
        HOME_S * BUS_HZ
    )  # smoothstep from the present pose to the hover pose, then the planned trajectory
    s = lambda k: (k / n) ** 3 * (10 - 15 * k / n + 6 * (k / n) ** 2)
    path = [(q0 + (goal - q0) * s(k), "go home") for k in range(1, n + 1)]
    path += [(np.array([*qa, g]), label) for qa, g, label in rows]
    off = np.zeros(5)  # integrates the P-only servos' gravity sag away (rate 3 /s)
    t_next = time.monotonic()
    for qd, label in path:
        time.sleep(max(0.0, t_next - time.monotonic()))
        t_next += dt
        qc = np.clip(qd[:5] + off, LO[:5], HI[:5])
        bus.write_goals([*qc, qd[5]])
        qm = bus.read_q()
        off = np.clip(off + 3.0 * dt * (qd[:5] - qm[:5]), -0.4, 0.4)
    print(f"done: {len(path)} cycles")


if __name__ == "__main__":
    main()
