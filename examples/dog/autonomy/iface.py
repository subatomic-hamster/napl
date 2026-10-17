"""The autonomy's only door to locomotion. The carrier (carrier.py) and the dog adapter (dogloco.py) implement it."""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class Odom:
    t: float  # sim time of the pose (s)
    pos: np.ndarray  # (3,) odometry frame, drifts
    quat: np.ndarray  # (4,) w x y z, odometry frame (roll / pitch gravity-referenced)
    vel: np.ndarray  # (3,) body frame
    omega: np.ndarray  # (3,) body frame
    fallen: bool  # SP4 Controller.mode == "FALLEN"


@runtime_checkable
class Locomotion(Protocol):
    sim: object
    dt: float
    lidar: str
    gnss: str
    base: str

    def advance(self, seconds: float) -> None: ...
    def command(self, vx: float, vy: float, wz: float) -> None: ...
    def odometry(self) -> Odom: ...
    # SP5's pose back to locomotion; the dog adapter forwards it to SP4 as Controller.fix(p_xy, yaw, age_s, sig_p, sig_yaw)
    # (sigmas optional, SP4 defaults 0.02 m / 1 deg); the carrier ignores it.
    def fix(
        self,
        p_xy,
        yaw: float,
        t_pose: float,
        sig_p: float | None = None,
        sig_yaw: float | None = None,
    ) -> None: ...
    def limits(self) -> dict: ...
    def truth(self) -> tuple: ...  # harness / metrics only
