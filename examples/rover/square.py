"""Drive the rover (assemblies/rover.toml) round a 1 m square by encoder odometry, through the RoboClaw pty, with the unmodified official
`basicmicro` library (PyPI basicmicro==2.0.10, the `rover` extra). Run by `napl program examples/rover/square.scenario.toml`.

Odometry from the two encoder counts (1200 counts / wheel turn, 90 mm wheels, 223 mm between the tyre centres = assemblies/rover.toml):
x, y, heading are integrated from the count increments at ~50 Hz. Each side is a distance-to-go loop (speed = 3 /s x remaining distance,
capped at V_MAX, floor V_MIN) on the RoboClaw's velocity PID (command 37 = SpeedM1M2); each corner is the same loop on the heading with
the wheels turning opposite ways. Both loops end when the error is small and the wheels have stopped.
"""

import math
import os
import sys
import time

from basicmicro import Basicmicro

ADDR = 0x80
CPR, R_WHEEL, TRACK = 1200, 0.045, 0.223
M = CPR / (2 * math.pi * R_WHEEL)  # counts per metre of wheel travel
QPPS = 10600  # counts / s at 100 % duty = the datasheet's 530 rpm free speed at 12 V x 1200 counts / turn / 60 (docs/research/rover.md)
V_MAX, V_MIN, SIDE = 0.6, 0.06, 1.0  # m/s, m/s, m
W_MAX, W_MIN = 2.5, 0.3  # rad/s (turn rate of the heading loop)


def main(port=None, side=SIDE):
    rc = Basicmicro(port or os.environ["NAPL_SERIAL"], 38400)
    if not rc.Open():
        sys.exit("cannot open the RoboClaw port")
    for set_pid in (rc.SetM1VelocityPID, rc.SetM2VelocityPID):
        assert set_pid(
            ADDR, 1.0, 4.0, 0.0, QPPS
        )  # P, I, D, QPPS of this motor (the manual's defaults 1, 0.5, 0.25 are retuned: docs/research/rover.md)
    rc.ResetEncoders(ADDR)
    enc = lambda: (rc.ReadEncM1(ADDR)[1] % 2**32, rc.ReadEncM2(ADDR)[1] % 2**32)
    signed = lambda v: (
        v - 2**32 if v >= 2**31 else v
    )  # counts are unsigned 32 bit, reversing wraps
    last = tuple(map(signed, enc()))
    pose = [0.0, 0.0, 0.0]  # x, y, heading (rad), start frame

    def poll():
        nonlocal last
        now = tuple(map(signed, enc()))
        dl, dr = ((a - b) / M for a, b in zip(now, last))
        last = now
        th = pose[2] + (dr - dl) / TRACK / 2
        pose[0] += (dl + dr) / 2 * math.cos(th)
        pose[1] += (dl + dr) / 2 * math.sin(th)
        pose[2] += (dr - dl) / TRACK

    def drive(vl, vr):
        rc.SpeedM1M2(ADDR, round(vl * M), round(vr * M))

    def run(err, command, tol, vmin, vmax):
        """command(speed) every 20 ms while |err()| > tol; then stop and let the wheels settle."""
        while abs(err()) > tol:
            command(math.copysign(min(vmax, max(vmin, 3 * abs(err()))), err()))
            time.sleep(0.02)
            poll()
        drive(0, 0)
        time.sleep(0.3)
        poll()

    for k in range(4):
        x0, y0, hold = pose[0], pose[1], k * math.pi / 2

        def straight(
            v,
        ):  # heading hold: turn rate 2 /s x heading error, as a wheel speed difference
            w = 2.0 * (hold - pose[2]) * TRACK / 2
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
        goal = (k + 1) * math.pi / 2
        run(
            lambda: goal - pose[2],
            lambda w: drive(-w * TRACK / 2, w * TRACK / 2),
            0.01,
            W_MIN,
            W_MAX,
        )
        print(
            f"corner {k + 1}: x={pose[0]:.3f} y={pose[1]:.3f} heading={math.degrees(pose[2]):.1f} deg",
            flush=True,
        )
    print(
        f"odometry end pose x={pose[0]:.4f} y={pose[1]:.4f} heading={math.degrees(pose[2]):.2f} deg"
    )


if __name__ == "__main__":
    main()
