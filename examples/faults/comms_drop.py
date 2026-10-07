"""User program for examples/faults/so101_comms_drop.scenario.toml: moves the SO-101 shoulder pan (servo 1) 0.5 rad in 2 s over the
Feetech STS port ($NAPL_SERIAL), 50 position goals per second, reading the present position back every cycle. When the bus goes
silent (the scenario drops the link for 0.5 s mid-move) the read times out; the program counts the timeouts, keeps sending goals, and
resumes the moment the servo answers again. Exits 1 if the arm does not end up at the target.

    napl program examples/faults/so101_comms_drop.scenario.toml"""

import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "so101"))
from sts import CPR, Bus, configure, decode, goal_bytes

TARGET, MOVE_S, HZ = 0.5, 2.0, 50  # rad, s, Hz


def read_q(bus):
    """Present position of servo 1 in rad, or None on a timeout (no / bad reply)."""
    bus.send(1, 2, bytes([56, 15]))  # READ 15 bytes from the present-position register
    r = bus.recv(1, timeout=0.04)
    return decode(r[0][2])["q"] if r else None


def main():
    bus = Bus(os.environ["NAPL_SERIAL"])
    configure(bus)
    print("MOVE", flush=True)
    t_next, timeouts, longest, run, last_q = time.monotonic(), 0, 0, 0, None
    n = round(MOVE_S * HZ)
    for k in range(
        1, n + 1 + HZ
    ):  # the move, then 1 s of holding the target (time to catch up after the drop)
        time.sleep(max(0.0, t_next - time.monotonic()))
        t_next += 1.0 / HZ
        s = min(k / n, 1.0)
        bus.sync_write(
            42, {1: goal_bytes(TARGET * (3 * s**2 - 2 * s**3) * CPR / (2 * math.pi))}
        )
        q = read_q(bus)
        run = run + 1 if q is None else 0
        timeouts, longest = timeouts + (q is None), max(longest, run)
        last_q = q if q is not None else last_q
    bus.close()
    print(
        f"done: {timeouts} read timeouts (longest run {longest} cycles), last position {last_q} rad, bad frames {bus.bad}",
        flush=True,
    )
    assert last_q is not None and abs(last_q - TARGET) < 0.03, (
        f"did not recover: {last_q}"
    )


if __name__ == "__main__":
    main()
