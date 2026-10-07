"""Drive the 4-wheel skid-steer car (assemblies/car4.toml) round a 1 m square by encoder odometry, through TWO RoboClaw ptys (left
controller 0x80 = M1 front-left / M2 rear-left, right controller 0x81 = M1 front-right / M2 rear-right), with the unmodified
official `basicmicro` library (PyPI basicmicro==2.0.10, the `rover` extra). Run by `napl program examples/car4/square.scenario.toml`.
Ported from examples/rover/square.py (same control loops); what changes for four wheels:

* Odometry from the four encoder counts (1200 counts / wheel turn, 90 mm wheels): a side's travel is the mean of its two wheels.
* TRACK, the distance the odometry assumes between the left and right wheel lines. A skid-steer car turns by scrubbing its wheels
  sideways, so its yaw per wheel travel is not what the geometric track (0.223 m, wheel-centre to wheel-centre) predicts:
  the sim measures an EFFECTIVE track of about 0.42 m (spin in place at 0.3 - 0.5 duty: true yaw rate / odometry yaw rate = 0.53;
  docs/showcase/car/README.md). `CAR4_TRACK` overrides it (examples/car4/turn90_geometric.scenario.toml sets the geometric 0.223 m to show the
  uncalibrated error). The calibration is a constant measured once in the sim, not an estimate made by this program.
* `CAR4_MODE=turn`: only one 90 degree spin in place (the commanded-versus-true turn measurement), no square.
"""

import math
import os
import sys
import time

from basicmicro import Basicmicro

ADDR_L, ADDR_R = 0x80, 0x81
CPR, R_WHEEL = 1200, 0.045
TRACK = float(
    os.environ.get("CAR4_TRACK", 0.42)
)  # effective track, m (measured in the sim, see above)
M = CPR / (2 * math.pi * R_WHEEL)  # counts per metre of wheel travel
QPPS = 10600  # counts / s at 100 % duty = the datasheet's 530 rpm free speed at 12 V x 1200 counts / turn / 60 (docs/research/rover.md)
V_MAX, V_MIN, SIDE = 0.6, 0.06, 1.0  # m/s, m/s, m
HOLD = float(
    os.environ.get("CAR4_HOLD", 2.0)
)  # heading-hold gain on the straights, 1/s (turn rate per rad of heading error)
W_MAX, W_MIN = (
    2.0,
    0.8,
)  # rad/s (turn rate of the heading loop; below ~0.5 rad/s the scrubbing wheels stick)


def main(port_l=None, port_r=None, side=SIDE):
    ctl = []
    for addr, port, var in (
        (ADDR_L, port_l, "NAPL_SERIAL"),
        (ADDR_R, port_r, "NAPL_SERIAL_2"),
    ):
        rc = Basicmicro(port or os.environ[var], 38400)
        if not rc.Open():
            sys.exit(f"cannot open the RoboClaw port of controller {addr:#x}")
        for set_pid in (rc.SetM1VelocityPID, rc.SetM2VelocityPID):
            assert set_pid(
                addr, 1.0, 4.0, 0.0, QPPS
            )  # P, I, D, QPPS (the rover's retuned gains: docs/research/rover.md)
        rc.ResetEncoders(addr)
        ctl.append((rc, addr))
    signed = lambda v: (
        v - 2**32 if v >= 2**31 else v
    )  # counts are unsigned 32 bit, reversing wraps

    def enc():  # (left front, left rear, right front, right rear) counts
        out = []
        for rc, addr in ctl:
            out += [
                signed(rc.ReadEncM1(addr)[1] % 2**32),
                signed(rc.ReadEncM2(addr)[1] % 2**32),
            ]
        return tuple(out)

    last = enc()
    pose = [0.0, 0.0, 0.0]  # x, y, heading (rad), start frame

    def poll():
        nonlocal last
        now = enc()
        d = [(a - b) / M for a, b in zip(now, last)]
        last = now
        dl, dr = (d[0] + d[1]) / 2, (d[2] + d[3]) / 2
        th = pose[2] + (dr - dl) / TRACK / 2
        pose[0] += (dl + dr) / 2 * math.cos(th)
        pose[1] += (dl + dr) / 2 * math.sin(th)
        pose[2] += (dr - dl) / TRACK

    def drive(vl, vr):
        for (rc, addr), v in zip(ctl, (vl, vr)):
            rc.SpeedM1M2(
                addr, round(v * M), round(v * M)
            )  # both wheels of a side at the same speed

    def run(err, command, tol, vmin, vmax):
        """command(speed) every cycle while |err()| > tol; then stop and let the wheels settle."""
        while abs(err()) > tol:
            command(math.copysign(min(vmax, max(vmin, 3 * abs(err()))), err()))
            time.sleep(0.01)
            poll()
        drive(0, 0)
        time.sleep(0.4)
        poll()

    def corner(goal):
        run(
            lambda: goal - pose[2],
            lambda w: drive(-w * TRACK / 2, w * TRACK / 2),
            0.01,
            W_MIN,
            W_MAX,
        )

    if os.environ.get("CAR4_MODE") == "turn":
        corner(math.pi / 2)
        print(
            f"odometry end pose x={pose[0]:.4f} y={pose[1]:.4f} heading={math.degrees(pose[2]):.2f} deg"
        )
        return
    for k in range(4):
        x0, y0, hold = pose[0], pose[1], k * math.pi / 2

        def straight(
            v,
        ):  # heading hold: turn rate 2 /s x heading error, as a wheel speed difference
            w = HOLD * (hold - pose[2]) * TRACK / 2
            drive(v - w, v + w)

        run(
            lambda: side - math.hypot(pose[0] - x0, pose[1] - y0),
            straight,
            0.003,
            V_MIN,
            V_MAX,
        )
        print(
            f"side {k + 1}: x={pose[0]:.3f} y={pose[1]:.3f} heading={math.degrees(pose[2]):.1f} deg",
            flush=True,
        )
        corner((k + 1) * math.pi / 2)
        print(
            f"corner {k + 1}: x={pose[0]:.3f} y={pose[1]:.3f} heading={math.degrees(pose[2]):.1f} deg",
            flush=True,
        )
    print(
        f"odometry end pose x={pose[0]:.4f} y={pose[1]:.4f} heading={math.degrees(pose[2]):.2f} deg"
    )


if __name__ == "__main__":
    main()
