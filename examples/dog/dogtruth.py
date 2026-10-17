"""Ground truth of a dog run, for the HARNESS only (monitors, MC verdicts, the route follower, pushes, the SP5 pose-fix stand-in). Never
imported by examples/dog/loco (Review Focus 2)."""

import math

import numpy as np

TRUNK, FREE = "/world/deck_top", "/world/deck_top/deck_top:free"
FEET = [f"/devices/{leg}:foot" for leg in ("fl", "fr", "rl", "rr")]
TILT_FALL_DEG = 60.0  # trunk beyond 60 deg from upright: on its side
CLEAR_FRAC = 0.6  # trunk-origin clearance below 60 % of its standing value (armed after stand-up): belly on the ground


class Truth:
    def __init__(self, sim, field=None):
        self.sim, self.field, self.clear0 = sim, field, None

    def pos(self):
        return np.array(self.sim.query(TRUNK, "pos")[0], float)

    def quat(self):
        return np.array(self.sim.query(TRUNK, "quat")[0], float)  # w x y z

    def yaw(self):
        w, x, y, z = self.quat()
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def tilt_deg(self):
        w, x, y, z = self.quat()
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def vel(self):
        return np.array(
            self.sim.query(FREE, "qvel")[0][:3], float
        )  # MuJoCo free joint: world linear velocity of the body origin

    def ground(self, x, y):
        return 0.0 if self.field is None else float(self.field.height(x, y))

    def clearance(self):
        p = self.pos()
        return p[2] - self.ground(p[0], p[1])

    def contacts(self):
        return np.array([self.sim.query(f, "contact")[0][0] for f in FEET], float)

    def push(self, dv_world):
        qv = np.array(self.sim.query(FREE, "qvel")[0], float)
        qv[:3] += dv_world
        self.sim.set(FREE, "qvel", qv)

    def arm_fall(self):
        self.clear0 = self.clearance()

    def fallen(self):
        low = self.clear0 is not None and self.clearance() < CLEAR_FRAC * self.clear0
        return self.tilt_deg() > TILT_FALL_DEG or low
