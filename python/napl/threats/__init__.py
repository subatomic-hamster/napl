"""napl.threats: generic kinematic test targets (multirotor routes/orbit/survey/weaving, swarms, balloons, hanging loads, birds,
debris) with class labels, the scenario suite `build(name, seed)`, the tracker benchmark runner and a discrimination baseline.
Scope: tracker test data only; no homing/pursuit/dive profiles (see docs/cuas2/threat-scenarios.md)."""

from .scenes import LABELS, NAMES, ThreatScenario, build

__all__ = ["LABELS", "NAMES", "ThreatScenario", "build"]
