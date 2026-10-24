"""M10, Python side: host-backed Sims, MCP play/pause tools, Sim.show (needs a display: run under `xvfb-run -a`) and
`napl mcp --view`."""

import asyncio
import json
import os
import shutil
import sys
import time
from pathlib import Path

import napl
import pytest
from mcp import Client
from mcp.client.stdio import StdioServerParameters
from napl.mcp_server import NaplServer

DATA = Path(__file__).resolve().parents[2] / "tests" / "data"
PENDULUM = DATA / "pendulum.xml"
HINGE = "/world/pole/hinge"
needs_display = pytest.mark.skipif(
    sys.platform != "darwin" and not os.environ.get("DISPLAY"), reason="needs a display: run under xvfb-run"
)


def test_host_backed_sim_matches_plain_sim_and_logs():
    plain, hosted = napl.Sim.load(PENDULUM), napl.Sim.load(PENDULUM, host=True)
    plain.start_log()
    for s in (plain, hosted):
        assert s.step(10) == 10
        s.set(HINGE, "qpos", [0.3])
        s.step(5)
    assert hosted.tick == 15 and not hosted.diverged
    assert hosted.state_hash() == plain.state_hash()
    assert hosted.query(HINGE, "qpos")[0][0] == pytest.approx(
        plain.query(HINGE, "qpos")[0][0]
    )
    assert hosted.log() is not None  # a Host always records
    assert hosted.call({"cmd_type": "Run", "cmd": {"rate": 0}})[
        "ok"
    ]  # works on a Host (a no-op on a bare World)
    time.sleep(0.1)
    assert hosted.call({"cmd_type": "Pause", "cmd": {}})["ok"]
    t = hosted.tick
    assert t > 15
    time.sleep(0.05)
    assert hosted.tick == t  # paused: frozen
    fork = hosted.fork()  # a plain Sim cloned on the sim thread
    assert fork.tick == t and fork.state_hash() == hosted.state_hash()


def test_mcp_play_pause_tools_only_when_host_backed(tmp_path):
    shutil.copy(PENDULUM, tmp_path / "pendulum.xml")

    async def main():
        async with Client(NaplServer(tmp_path, allow_destructive=True).server) as c:
            plain = {t.name for t in (await c.list_tools()).tools}
        async with Client(
            NaplServer(tmp_path, allow_destructive=True, view=True).server
        ) as c:
            tools = {t.name: t for t in (await c.list_tools()).tools}
            assert not (
                await c.call_tool("load_model", {"path": "pendulum.xml"})
            ).is_error
            r = await c.call_tool("play_simulation", {"rate": 0})
            assert not r.is_error, r
            await asyncio.sleep(0.1)
            paused = json.loads(
                (await c.call_tool("pause_simulation", {})).content[0].text
            )["tick"]
            await asyncio.sleep(0.05)
            after = json.loads(
                (await c.call_tool("step_simulation", {"n": 3})).content[0].text
            )["tick"]
            return plain, tools, paused, after

    plain, tools, paused, after = asyncio.run(main())
    assert "play_simulation" not in plain and "pause_simulation" not in plain
    assert set(tools) == plain | {"play_simulation", "pause_simulation"}
    for t in (
        "play_simulation",
        "pause_simulation",
    ):  # control: neither read-only nor destructive
        assert not tools[t].annotations.read_only_hint
        assert not tools[t].annotations.destructive_hint
    assert paused > 0 and after == paused + 3  # frozen while paused, exact step


@pytest.mark.skipif(sys.platform == "darwin", reason="Cocoa does not use DISPLAY")
def test_show_without_display_is_a_runtime_error_and_the_sim_survives(monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    s = napl.Sim.load(PENDULUM)
    s.step(4)
    with pytest.raises(RuntimeError, match="DISPLAY"):
        s.show(frames=1)
    assert s.tick == 4  # the World was handed back
    assert s.step(1) == 5


@pytest.mark.skipif(sys.platform != "darwin", reason="Cocoa main-thread contract")
def test_off_main_thread_show_rejects_and_returns_the_world():
    from concurrent.futures import ThreadPoolExecutor

    s = napl.Sim.load(PENDULUM)
    s.step(4)
    with ThreadPoolExecutor(max_workers=1) as pool:
        with pytest.raises(RuntimeError, match="main thread"):
            pool.submit(s.show, frames=1).result()
    assert s.tick == 4
    assert s.step(1) == 5


@pytest.mark.gpu
def test_show_offscreen_needs_no_display_and_returns_frames(monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.setenv("NAPL_OFFSCREEN", "1")
    s = napl.Sim.load(PENDULUM)
    s.show(
        frames=30,
        script=f"3 step 5; 10 setattr {HINGE} qpos 0.4",
        width=320,
        height=200,
    )
    seq, img = napl._napl.viewer_frame()
    assert seq >= 30 and img.shape == (200, 320, 4) and img.dtype.name == "uint8"
    assert (img[..., 3] == 255).all() and len(
        set(map(bytes, img.reshape(-1, 4)[::97]))
    ) > 8
    ref = napl.Sim.load(PENDULUM)
    ref.step(5)
    ref.set(HINGE, "qpos", [0.4])
    assert s.state_hash() == ref.state_hash()


@needs_display
@pytest.mark.gpu
def test_sim_show_frames_scripted_and_world_returned():
    script = f"2 select {HINGE}; 3 step 5; 10 setattr {HINGE} qpos 0.4; 20 step 20"
    s = napl.Sim.load(PENDULUM)
    s.show(frames=40, script=script, width=320, height=240)
    ref = napl.Sim.load(PENDULUM)
    ref.step(5)
    ref.set(HINGE, "qpos", [0.4])
    ref.step(20)
    assert s.tick == 25
    assert s.state_hash() == ref.state_hash()  # the viewer only read
    assert s.log() is not None  # the viewer's commands were recorded
    assert s.step(1) == 26  # and the Sim carries on
    with pytest.raises(
        RuntimeError, match="unknown action"
    ):  # rejected before any window opens
        s.show(frames=1, script="1 bogus")


@needs_display
@pytest.mark.gpu
def test_mcp_view_opens_viewer_on_main_thread_and_exits_with_the_client(tmp_path):
    shutil.copy(PENDULUM, tmp_path / "pendulum.xml")
    params = StdioServerParameters(
        command=sys.executable,
        args=[
            "-m",
            "napl.cli",
            "mcp",
            "--view",
            "--root",
            str(tmp_path),
            "--allow-destructive",
        ],
        cwd=tmp_path,
        env=dict(
            os.environ
        ),  # DISPLAY, VK_DRIVER_FILES, LP_NUM_THREADS reach the server
    )

    async def main():
        async with Client(params) as c:
            names = {t.name for t in (await c.list_tools()).tools}
            assert {"play_simulation", "pause_simulation"} <= names
            assert not (
                await c.call_tool("load_model", {"path": "pendulum.xml"})
            ).is_error
            await asyncio.sleep(
                1.0
            )  # the viewer window is up now, served from the main thread
            assert not (await c.call_tool("play_simulation", {"rate": 0})).is_error
            await asyncio.sleep(0.3)
            tick = json.loads(
                (await c.call_tool("pause_simulation", {})).content[0].text
            )["tick"]
            r = await c.call_tool("step_simulation", {"n": 2})
            assert json.loads(r.content[0].text)["tick"] == tick + 2
            return tick

    assert (
        asyncio.run(main()) > 0
    )  # leaving the client closes stdin: the server ends and closes its viewer
