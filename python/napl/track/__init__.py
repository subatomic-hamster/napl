"""napl.track: multi-target tracking library for fire control (plain numpy; scipy used for assignment only if installed).

models (CV / CA / Singer / CT motion; position / range / bearing / camera measurements), filters (EKF=KF, UKF, IMM, particle),
trackers (GNN, JPDA, MHT, GM-PHD with M-of-N management, Track objects), metrics (GOSPA, OSPA, NEES/NIS, Evaluator),
scenarios (textbook benchmarks), variants (named configurations + run()). Benchmark: tools/track_bench.py."""

from .filters import EKF, IMM, KF, UKF, ParticleFilter
from .metrics import Evaluator, gospa, nees, nis, ospa
from .models import (
    CA,
    CT,
    CV,
    BearingModel,
    CameraModel,
    PositionModel,
    RangeModel,
    Singer,
    triangulate,
)
from .trackers import GNN, JPDA, MHT, PHD, Track

__all__ = [
    "CA",
    "CT",
    "CV",
    "EKF",
    "GNN",
    "IMM",
    "JPDA",
    "KF",
    "MHT",
    "PHD",
    "UKF",
    "BearingModel",
    "CameraModel",
    "Evaluator",
    "ParticleFilter",
    "PositionModel",
    "RangeModel",
    "Singer",
    "Track",
    "gospa",
    "nees",
    "nis",
    "ospa",
    "triangulate",
]
