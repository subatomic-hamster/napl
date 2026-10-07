"""`gpu`-marked tests need a Vulkan device: skipped when the first render says there is none."""

from pathlib import Path

import napl
import pytest

_SCENE = Path(__file__).resolve().parents[1] / "data" / "render_scene.xml"
_probe = []  # [reason or None], filled once per session


def _no_device_reason():
    if not _probe:
        try:
            napl.Sim.load(_SCENE).render([-3, 0, 1], [0, 0, 0.25], 45.0, 16, 16)
            _probe.append(None)
        except RuntimeError as e:
            if "Vulkan" not in str(e):
                raise
            _probe.append(str(e))
    return _probe[0]


@pytest.fixture(autouse=True)
def _gpu_skip(request):
    if request.node.get_closest_marker("gpu"):
        reason = _no_device_reason()
        if reason:
            pytest.skip(f"no usable Vulkan device: {reason}")
