"""Renderer: heightfield terrain (GeomType::hfield): drawn, depth equals the geometry, pad markers visible, the far plane reaches terrain kilometres away
(scenes without terrain keep the 1000 m far plane: tests/python/test_render.py goldens). Needs a Vulkan device (`gpu` marker)."""

import numpy as np
import pytest
from napl import Sim
from napl.terrain import PadSpec, add_terrain_xml, generate

BASE = """<mujoco model="t"><option timestep="0.002"/><worldbody>
<geom name="floor" type="plane" size="0 0 1"/>
<body name="b" pos="{x} {y} {z}"><freejoint/><geom type="box" size="0.2 0.2 0.1" mass="1" rgba="0.9 0.1 0.1 1"/></body>
</worldbody></mujoco>"""


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    t = generate(
        "ridge",
        size=(1600, 1200),
        n=161,
        relief=250,
        roughness=2.0,
        seed=3,
        pads=[PadSpec("summit", at="summit", radius=4)],
    )
    p = t.pad("summit")
    path = tmp_path_factory.mktemp("terr") / "s.xml"
    path.write_text(add_terrain_xml(BASE.format(x=p.x, y=p.y, z=p.z + 0.1), t))
    return Sim.load(path), t, p


@pytest.mark.gpu
def test_terrain_is_drawn_with_the_right_depth_and_the_pad_marker(scene):
    s, t, p = scene
    rgba, depth = s.render(
        pos=[p.x, p.y - 0.001, p.z + 30.0],
        lookat=[p.x, p.y, p.z],
        fovy=40,
        width=160,
        height=120,
    )
    assert np.isfinite(depth).all()  # ground everywhere below
    assert (
        abs(float(depth[60, 80]) - 29.9) < 0.15
    )  # the box top is at +0.2 m; the surrounding pad is at 30 m
    assert (
        abs(float(depth[5, 5]) - 30.0) < 1.5 or float(depth[5, 5]) > 28.0
    )  # corners: the flat pad (and the slope beyond it for a wide angle)
    px = (
        rgba[10:20, 70:90, :3].reshape(-1, 3).astype(float)
    )  # the painted disc (grey) in the upper middle
    assert 0.25 * 255 < px.mean() < 0.75 * 255 and px.std(axis=0).max() < 40


@pytest.mark.gpu
def test_terrain_is_visible_from_kilometres_and_shaded_by_slope_and_height(scene):
    s, t, p = scene
    rgba, depth = s.render(
        pos=[-1300, 0, 500.0], lookat=[0, 0, 0.0], fovy=50, width=320, height=180
    )
    d = depth[np.isfinite(depth)]
    assert (
        d.size > 0.2 * depth.size and d.max() > 1500
    )  # terrain beyond the usual 1000 m far plane
    # the sky fills the top, terrain colours (not the sky gradient) the bottom rows, and the rows differ (relief shading)
    bottom = rgba[-20:, :, :3].astype(float)
    assert bottom.std() > 3.0
    assert np.isfinite(depth[-1, 100:220]).all()


@pytest.mark.gpu
def test_rendering_a_flat_scene_is_unchanged_by_the_terrain_far_plane(tmp_path):
    """A 10 m box 1200 m away: clipped (+inf) by the old 1000 m far plane without terrain, seen (1195 m) when the scene has a heightfield."""
    wall = BASE.replace('size="0.2 0.2 0.1"', 'size="5 5 5"').format(x=1200, y=0, z=5.0)
    flat = tmp_path / "f.xml"
    flat.write_text(wall)
    cam = dict(pos=[0, 0, 5.0], lookat=[1200, 0, 5.0], fovy=10, width=64, height=48)
    _, d0 = Sim.load(flat).render(**cam)
    assert np.isinf(d0[24, 32])
    t = generate(
        "moorland",
        size=(400, 400),
        n=33,
        relief=10,
        roughness=0.5,
        seed=1,
        pads=[PadSpec("p", at="center", radius=3)],
    )
    terr = tmp_path / "t.xml"
    terr.write_text(add_terrain_xml(wall, t, markers=False))
    _, d1 = Sim.load(terr).render(**{**cam, "pos": [0, 0, 5.0]})
    assert abs(float(d1[24, 32]) - 1195.0) < 1.0
