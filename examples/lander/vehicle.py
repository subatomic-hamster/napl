"""What the flight computer of the TVC lander is built with: constants of assemblies/tvc-lander.toml and the two motors' certified thrust curves.

Firmware tables, not sensors: the numbers below are known to the flight computer at build time (from the design report and the motor data
sheets). tests/python/test_lander.py checks them against `napl design` so they cannot drift from the assembly. The motor curves are read from
parts/rocket/*.toml (thrustcurve.org certification data) the way a firmware build would embed them.
"""

import math
import tomllib
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]

G = 9.80665  # m/s2
RHO0 = 1.225  # kg/m3 (ISA, sea level)

# From `napl design check assemblies/tvc-lander.toml` (mass report; CG and inertia from the generated MJCF). Both motors loaded.
M_LOADED = 1.4505  # kg
L_NOZ = 0.3639  # m, CG to the nozzle exit plane (the thrust force acts there)
I_TRANS = 0.0879  # kg m2, pitch / yaw moment of inertia about the CG (loaded)
MOTOR_OFFSET = 0.01078  # m, each motor axis from the gimbal axis, in x and y (diagonal seats, tvc-gimbal-2x-cradle)
CDA_UP = 0.83 * 0.015328  # m2, drag area nose-first (airframe cd0 x s_ref)
CDA_DOWN = 1.33 * 0.015328  # m2, tail-first (airframe cd_rev x s_ref)
GIMBAL_LIMIT = math.radians(
    7.0
)  # rad, the software stops 1 deg inside the +-8 deg mechanical range

ASCENT_MOTOR = "estes-e16"  # motor_a, seat (+x, +y)
LANDING_MOTOR = "estes-f15"  # motor_b, seat (-x, -y)
SEAT = {"a": (+1.0, +1.0), "b": (-1.0, -1.0)}


class Motor:
    """A solid motor's firmware table: thrust curve F(tau), tau = time after the end of the igniter delay (the device's definition)."""

    def __init__(self, part_id):
        p = tomllib.loads((REPO / "parts" / "rocket" / f"{part_id}.toml").read_text())
        rows = np.array(p["tables"]["thrust_curve"]["rows"], float)
        if rows[0, 0] > 0:
            rows = np.vstack([[0.0, 0.0], rows])
        self.id = part_id
        self.t, self.f = rows[:, 0], rows[:, 1]
        self.cum = np.concatenate(
            [[0.0], np.cumsum(0.5 * (self.f[1:] + self.f[:-1]) * np.diff(self.t))]
        )
        v = {k: x["value"] for k, x in p["params"].items()}
        self.mass, self.prop, self.delay = (
            v["mass"],
            v["propellant_mass"],
            v["ignition_delay"],
        )
        self.burn = float(self.t[-1])
        self.impulse = float(self.cum[-1])

    def thrust(self, tau):
        return np.interp(tau, self.t, self.f, left=0.0, right=0.0)

    def burned(self, tau):
        """Propellant mass burned by tau (kg)."""
        return self.prop * np.interp(tau, self.t, self.cum) / self.impulse

    def tau_at_thrust(self, n):
        """First curve time at which the thrust reaches n newtons."""
        i = int(np.argmax(self.f >= n))
        if i == 0:
            return 0.0
        t0, t1, f0, f1 = self.t[i - 1], self.t[i], self.f[i - 1], self.f[i]
        return float(t0 + (t1 - t0) * (n - f0) / (f1 - f0))


def trim_angles(seat, length=L_NOZ):
    """Gimbal angles (pitch servo, yaw servo; rad) that put the thrust line of the motor in `seat` ('a' / 'b') through the CG.
    A motor at (sx d, sy d) pushes with a torque (sy d T, -sx d T) about (x, y); the gimbal gives (L T y, L T p): p = sx d / L, y = -sy d / L."""
    sx, sy = SEAT[seat]
    return sx * MOTOR_OFFSET / length, -sy * MOTOR_OFFSET / length
