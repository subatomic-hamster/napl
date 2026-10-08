"""The renderer's look: procedural sky (outdoor / studio), sun shadow, CAD edges, cables as tubes, sRGB output."""

from pathlib import Path

import napl
import numpy as np
import pytest

SCENE = Path(__file__).resolve().parents[2] / "tests" / "data" / "render_scene.xml"
CAM = dict(pos=[2.5, -2.2, 1.0], lookat=[0, 0, 0.5], fovy=60, width=200, height=150)


@pytest.fixture(autouse=True)
def _reset_style():
    yield
    napl.render_style("outdoor,shadows,noedges,wires")


def shot(style):
    napl.render_style(style)
    rgba, depth = napl.Sim.load(SCENE).render(**CAM)
    return rgba[..., :3].astype(int), depth


@pytest.mark.gpu
def test_outdoor_sky_is_a_blue_gradient_with_a_warm_bright_horizon():
    rgb, depth = shot("outdoor")
    sky = np.isinf(depth)
    assert sky[:3].all()  # the top rows see only sky
    top, near_horizon = rgb[1, 100], rgb[sky[:, 100].nonzero()[0].max(), 100]
    assert top[2] > top[0] + 30  # blue at the zenith
    assert (
        near_horizon[0] > near_horizon[2] and near_horizon.sum() > top.sum()
    )  # warm and brighter at the horizon
    ground = rgb[-2, 5]
    assert ground.max() < 130 and abs(ground[0] - ground[2]) < 25  # dark grey floor


@pytest.mark.gpu
def test_studio_is_neutral_and_differs_from_outdoor():
    a, _ = shot("outdoor")
    b, depth = shot("studio")
    top = b[1, 100]
    assert abs(top[0] - top[2]) < 40 and np.abs(a - b).mean() > 5


@pytest.mark.gpu
def test_shadow_map_darkens_the_floor_beside_the_box():
    lit, _ = shot("outdoor,shadows")
    unlit, _ = shot("outdoor,noshadows")
    diff = unlit.sum(2) - lit.sum(2)
    assert (diff > 60).sum() > 50  # a shadow patch on the floor
    assert diff.min() > -3  # shadows never brighten anything


@pytest.mark.gpu
def test_edges_toggle_adds_dark_outline_pixels():
    plain, _ = shot("outdoor,noedges")
    edged, _ = shot("outdoor,edges")
    dark = lambda im: (im.sum(2) < 40).sum()
    assert dark(edged) > dark(plain) + 20


@pytest.mark.gpu
def test_output_is_deterministic_and_depth_stays_exact():
    a, da = shot("outdoor")
    b, db = shot("outdoor")
    assert (a == b).all() and np.array_equal(da, db, equal_nan=False)
    assert np.isfinite(da).any() and np.isinf(da).any()
