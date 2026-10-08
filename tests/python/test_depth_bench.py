"""The D435i depth bench (assemblies/depth-bench.toml): the rangefinder device built from the part's datasheet numbers, looking at a wall."""

from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design, marine

REPO = Path(__file__).resolve().parents[2]
WALL_X = 2.0
LOOK_X = "0 0.7071067811865476 0 0.7071067811865476"  # body z -> world +x, body x (image up) -> world +z


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    out = tmp_path_factory.mktemp("bench")
    xml, _ = design.compute(
        REPO / "assemblies" / "depth-bench.toml", REPO / "parts", out
    )
    body = xml.split('<body name="cam"')[1].split(">")[0]
    xml = xml.replace(
        f'<body name="cam"{body}>',
        f'<body name="cam" pos="0 0 0.5" quat="{LOOK_X}">',
        1,
    ).replace(
        '<geom name="floor"',
        f'<geom name="wall" type="box" pos="{WALL_X + 0.05} 0 1" size="0.05 3 3"/>\n    <geom name="floor"',
        1,
    )
    (out / "scene.xml").write_text(xml)
    return out / "scene.xml"


def frame(scene):
    sim = napl.Sim.load(scene)
    sim.step(50)
    return marine.depth_image(sim, "/devices/depth", 160, 90), sim


def test_depth_image_sees_the_wall_and_the_floor(scene):
    img, _ = frame(scene)
    assert img.shape == (90, 160) and img.dtype == np.float32
    assert not np.isnan(img).any()
    # the optical frame sits 25 mm ahead of the part origin; the wall is a plane: constant depth along the optical axis (noise sigma = 2 % of range)
    want = WALL_X - 0.025
    top = img[:30, 60:100]
    assert top.mean() == pytest.approx(want, abs=0.01)
    assert top.std() == pytest.approx(0.02 * want, rel=0.25)
    assert (
        img[-1, 80] < 1.2
    )  # the bottom row looks down 29 deg at the floor 0.5 m below: depth ~0.9 m
    assert img[-1, 80] > 0.28  # inside Min-Z


def test_depth_frames_are_deterministic(scene):
    a, sa = frame(scene)
    b, sb = frame(scene)
    np.testing.assert_array_equal(a, b)
    assert sa.state_hash() == sb.state_hash()
