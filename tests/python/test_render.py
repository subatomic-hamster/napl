"""M9 exit checks, Python side: Sim.render / Sim.raycast and the MCP `render` / `raycast` tools."""

import asyncio
import base64
import json
import shutil
import struct
import zlib
from pathlib import Path

import napl
import numpy as np
import pytest
from mcp import Client
from napl.mcp_server import NaplServer

DATA = Path(__file__).resolve().parents[2] / "tests" / "data"
SCENE = DATA / "render_scene.xml"
CAM = {
    "pos": [-3, 0, 0.25],
    "lookat": [0, 0, 0.25],
}  # 2.75 m from the box face x = -0.25


def png_size(data: bytes):
    """-> (width, height) of a PNG after checking signature, IHDR and that the IDAT stream inflates."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    n, kind = struct.unpack(">I4s", data[8:16])
    assert kind == b"IHDR" and n == 13
    w, h = struct.unpack(">II", data[16:24])
    pos, idat = 8, b""
    while pos < len(data):
        n, kind = struct.unpack(">I4s", data[pos : pos + 8])
        if kind == b"IDAT":
            idat += data[pos + 8 : pos + 8 + n]
        pos += 12 + n
    assert len(zlib.decompress(idat)) >= w * h  # at least one byte per pixel
    return w, h


@pytest.mark.gpu
def test_sim_render_shape_depth_and_lossless():
    s = napl.Sim.load(SCENE)
    rgba, depth = s.render(**CAM, fovy=45, width=161, height=121)
    assert rgba.shape == (121, 161, 4) and rgba.dtype == np.uint8
    assert depth.shape == (121, 161) and depth.dtype == np.float32
    assert abs(float(depth[60, 80]) - 2.75) < 0.01
    assert (rgba[..., 3] == 255).all()
    assert np.isinf(depth).any() and np.isfinite(depth).any()
    rgba2, depth2 = s.render(
        **CAM, fovy=45, width=161, height=121
    )  # same driver, same inputs
    assert np.array_equal(rgba, rgba2) and np.array_equal(depth, depth2)
    with pytest.raises(ValueError):
        s.render(**CAM, width=0, height=10)
    with pytest.raises(ValueError):
        s.render([0, 0, 0], [0, 0, 0])


def test_sim_raycast_matches_analytic():  # CPU only
    s = napl.Sim.load(SCENE)
    d = s.raycast([-2, 0, 0.25], [[1, 0, 0], [3, 0, 0], [1, 0.1, 0], [0, 0, 1]])
    assert d.dtype == np.float64
    np.testing.assert_allclose(
        d, [1.75, 1.75, 1.75 * np.sqrt(1.01), -1.0], rtol=0, atol=1e-9
    )
    with pytest.raises(ValueError):
        s.raycast([0, 0, 0], [[0, 0, 0]])


def test_read_only_session_can_render_cap_checked():
    s = napl.Sim.load(
        SCENE, caps=("control",)
    )  # no "read": the bus refuses before any GPU work
    with pytest.raises(RuntimeError, match="permission_denied"):
        s.raycast([0, 0, 0], [[1, 0, 0]])


@pytest.fixture
def root(tmp_path):
    shutil.copy(SCENE, tmp_path / "scene.xml")
    return tmp_path


async def _load(c):
    r = await c.call_tool("load_model", {"path": "scene.xml"})
    assert not r.is_error, r


@pytest.mark.gpu
def test_mcp_render_returns_png_image(root):
    async def main():
        async with Client(NaplServer(root, allow_destructive=True).server) as c:
            await _load(c)
            return await c.call_tool("render", {**CAM, "width": 64, "height": 48})

    r = asyncio.run(main())
    assert not r.is_error, r
    img = next(b for b in r.content if b.type == "image")
    assert img.mime_type == "image/png"
    assert png_size(base64.b64decode(img.data)) == (64, 48)
    info = json.loads(next(b for b in r.content if b.type == "text").text)
    assert info["width"] == 64 and abs(info["center_depth_m"] - 2.75) < 0.05


def test_mcp_raycast_tool_and_render_errors(root):
    async def main():
        async with Client(NaplServer(root, allow_destructive=True).server) as c:
            err = await c.call_tool("render", CAM)  # no model loaded yet
            await _load(c)
            ray = await c.call_tool(
                "raycast", {"origin": [-2, 0, 0.25], "directions": [1, 0, 0, 0, 0, 1]}
            )
            bad = await c.call_tool("render", {**CAM, "width": 0})
            return err, ray, bad

    err, ray, bad = asyncio.run(main())
    assert err.is_error and "no model loaded" in err.content[0].text
    assert not ray.is_error
    assert json.loads(ray.content[0].text)["distances"] == pytest.approx(
        [1.75, -1.0], abs=1e-9
    )
    assert (
        bad.is_error and "invalid_argument" in bad.content[0].text
    )  # validated before any GPU work
