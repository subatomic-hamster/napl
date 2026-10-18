"""The controller's only door to the robot (SP4): reads what the real dog's sensors report (12 Qdd drivers over CAN, the IMU) and writes
MIT-mode frames. Nothing else in examples/dog/loco touches the Sim. `reads` records every (path, attr) for the audit test (Review Focus 2)."""

import numpy as np

LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")
QDD = [
    f"/devices/{leg}:{j}" for leg in LEGS for j in JOINTS
]  # leg-major: the order of every 12-vector in loco
IMU = "/devices/compute:imu"
HW_ATTRS = {
    "q",
    "dq",
    "tau_est",
    "temp",
    "fault",
    "voltage",
    "timeout",
    "gyro",
    "accel",
}  # a real bus reports these


class Hw:
    def __init__(self, sim, audit=False):
        self.sim, self.reads = sim, (set() if audit else None)
        self._kp = self._kd = None
        self._many = getattr(
            sim, "set_many", None
        )  # Task 1 Step 6 (only if the bench asked for it)

    def _get(self, path, attr):
        if self.reads is not None:
            self.reads.add((path, attr))
        return self.sim.query(path, attr)[0]

    def joints(self):
        g = self._get
        return (np.array([g(p, "q")[0] for p in QDD]), np.array([g(p, "dq")[0] for p in QDD]),
                np.array([g(p, "tau_est")[0] for p in QDD]))  # fmt: skip

    def imu(self):
        return np.array(self._get(IMU, "gyro"), float), np.array(
            self._get(IMU, "accel"), float
        )

    def health(self):
        g = self._get
        return (
            max(g(p, "fault")[0] for p in QDD),
            min(g(p, "voltage")[0] for p in QDD),
            max(g(p, "temp")[0] for p in QDD),
        )

    def enable(self):
        for p in QDD:
            self.sim.set(p, "enable", 1.0)

    def send(self, p_des, v_des, kp, kd, tau_ff):
        """One MIT frame per drive. kp / kd are written only when they change (the driver holds them; a real host repeats them in every
        frame, the bus here just logs fewer commands); p_des / v_des / tau_ff every call, which also feeds the drivers' CAN timeout."""
        cmds = [(p, "p_des", p_des[i]) for i, p in enumerate(QDD)] + [
            (p, "v_des", v_des[i]) for i, p in enumerate(QDD)
        ]
        cmds += [(p, "tau_ff", tau_ff[i]) for i, p in enumerate(QDD)]
        if self._kp is None or np.any(kp != self._kp):
            cmds += [(p, "kp", kp[i]) for i, p in enumerate(QDD)]
            self._kp = np.array(kp, float)
        if self._kd is None or np.any(kd != self._kd):
            cmds += [(p, "kd", kd[i]) for i, p in enumerate(QDD)]
            self._kd = np.array(kd, float)
        if self._many:
            self._many(
                [c[0] for c in cmds], [c[1] for c in cmds], [float(c[2]) for c in cmds]
            )
        else:
            for path, attr, val in cmds:
                self.sim.set(path, attr, float(val))
