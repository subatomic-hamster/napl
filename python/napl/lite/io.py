"""App I/O interfaces (docs/cuas-lite/spec.md section 4): Protocols and dataclasses only. Firmware-independent code (fire control, App) imports this
module and never an implementation; the real, HIL and lockstep-sim implementations live in camera_v4l2 / detector_client / gimbal / sim_io."""

from collections import deque
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from .protocol import Event, HelloAck, Status

CLASSES = ("drone", "balloon", "person", "target")
TARGET_CLASSES = ("drone", "balloon", "target")  # never "person"


class Clock(Protocol):
    def now(self) -> float: ...  # seconds, monotonic; sim: sim time

    def sleep_until(
        self, t: float
    ) -> None: ...  # a lockstep SimClock raises: the harness drives time


@dataclass(frozen=True)
class Intrinsics:
    w: int
    h: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple = (0, 0, 0, 0, 0)  # OpenCV k1 k2 p1 p2 k3

    @classmethod
    def from_fov(cls, w, h, fov_v_rad):
        """Pinhole prior from the vertical field of view (square pixels): the Brio at 52.2 deg is fx = fy = 735.6 at 1280x720."""
        import math

        f = 0.5 * h / math.tan(fov_v_rad / 2)
        return cls(w, h, f, f, w / 2, h / 2)


@dataclass
class Frame:
    seq: int
    t_cap: float
    w: int
    h: int
    rgb: np.ndarray | None = None  # uint8 (h, w, 3); None in no-pixel sim
    meta: dict | None = (
        None  # sim only: ground-truth snapshot for the TruthDetector. Real cameras never set it, the app never reads it
    )


class Camera(Protocol):
    intrinsics: Intrinsics

    def read(
        self, timeout: float = 0.1
    ) -> Frame | None: ...  # newest frame not yet returned, or None

    def close(self) -> None: ...


class Link(Protocol):  # raw bytes both ways: tty fd, pty slave, or in-process loopback
    def write(self, data: bytes) -> None: ...

    def read(self, timeout: float = 0.0) -> bytes: ...


@dataclass
class Det:
    cls: str
    conf: float
    xyxy: tuple


@dataclass
class DetResult:
    seq: int
    t_infer_ms: float
    dets: list
    t_rx: float = 0.0  # app clock when the reply arrived


class DetectorClient(Protocol):
    stale_s: (
        float  # seconds since the last reply (fail-closed input); inf before the first
    )

    def submit(
        self, frame: Frame
    ) -> None: ...  # writes the shm slot + request line; never blocks

    def poll(self) -> DetResult | None: ...  # newest reply not yet returned, or None


class Gimbal(
    Protocol
):  # speaks spec section 2 over a Link; ONE implementation (napl.lite.gimbal.SerialGimbal)
    status: Status
    events: "deque[Event]"
    ack: HelloAck | None

    def hello(self) -> None: ...

    def heartbeat(self, deadman: bool, inhibit: bool) -> None: ...

    def aim(
        self,
        pan_deg,
        tilt_deg,
        pan_vff=0.0,
        tilt_vff=0.0,
        vmax=(240.0, 150.0),
        amax=(2000.0, 1500.0),
    ) -> None: ...

    def home(self, pan=True, tilt=True) -> None: ...

    def flywheel(self, duty: float) -> None: ...

    def fire(self, n: int = 1) -> int: ...  # -> fire_id

    def arm(self, shot_cap: int, timeout_s: int) -> None: ...

    def disarm(self) -> None: ...

    def pump(self) -> None: ...  # drain the link, update .status and .events


@dataclass
class Pose:
    """Camera and muzzle in the base frame B at one instant (spec 5.1): from the STATUS ring and the head geometry."""

    cam_pos: np.ndarray
    cam_fwd: np.ndarray
    cam_up: np.ndarray
    muzzle: np.ndarray
    muzzle_dir: np.ndarray
    pan: float = 0.0  # deg
    tilt: float = 0.0


@dataclass
class Trace:
    """Per-run record of what the app decided (the harness evaluator reads these): list of dicts, append-only."""

    rows: list = field(default_factory=list)
