"""M8 exit checks: the MCP server (tools from the schema, permissions, path sandbox, run_id polling, HTTP)."""

import asyncio
import json
import shutil
import socket
import subprocess
import sys
import time
import tomllib
from pathlib import Path

import napl
import pytest
from mcp import Client
from mcp.client.stdio import StdioServerParameters
from mcp.shared.exceptions import MCPError
from mcp_types import ElicitResult
from napl.mcp_server import NaplServer

DATA = Path(__file__).resolve().parents[2] / "tests" / "data"
HINGE = "/world/pole/hinge"
EXPECTED = {
    "get_entity_state", "set_entity_state", "step_simulation", "snapshot", "restore", "load_model", "run_test",
    "sweep", "list_prims", "get_run", "delete_run", "render", "raycast", "spawn_entity", "delete_entity",
}  # fmt: skip
DESIGN = {"design_open", "design_edit", "design_suggest", "design_check", "design_build", "design_simulate", "design_save", "design_undo"}  # fmt: skip
EXPECTED |= DESIGN
DESTRUCTIVE = {"load_model", "delete_run", "delete_entity", "design_save"}
READ_ONLY = {"get_entity_state", "list_prims", "get_run", "render", "raycast", "design_suggest", "design_check"}  # fmt: skip


@pytest.fixture
def root(tmp_path):
    shutil.copy(DATA / "pendulum.xml", tmp_path / "pendulum.xml")
    return tmp_path


def servo_spec() -> dict:
    s = tomllib.loads((DATA / "specs" / "servo.toml").read_text())
    s.pop("model")
    return s


async def accept(context, params):
    return ElicitResult(action="accept", content={"confirm": True})


async def decline(context, params):
    return ElicitResult(action="decline")


async def call(c, name, **args):
    """-> (is_error, parsed JSON result | error text)"""
    try:
        r = await c.call_tool(name, args)
    except (
        MCPError
    ) as e:  # protocol-level refusal (e.g. the client cannot be asked for approval)
        return True, str(e)
    text = r.content[0].text
    return r.is_error, (text if r.is_error else json.loads(text))


async def ok(c, name, **args):
    err, out = await call(c, name, **args)
    assert not err, out
    return out


async def poll(c, run_id, timeout=60):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = await ok(c, "get_run", run_id=run_id)
        if r["status"] in ("done", "error"):
            return r
        await asyncio.sleep(0.05)
    raise AssertionError("run did not finish")


def test_tool_list_complete_with_annotations(root):
    async def main():
        async with Client(NaplServer(root, allow_destructive=True).server) as c:
            return (await c.list_tools()).tools

    tools = {t.name: t for t in asyncio.run(main())}
    assert (
        set(tools) == EXPECTED
    )  # Run / Pause are deliberately absent (they need the Host thread)
    for name, t in tools.items():
        assert t.input_schema["type"] == "object", name
        assert t.annotations.destructive_hint == (name in DESTRUCTIVE), name
        assert t.annotations.read_only_hint == (name in READ_ONLY), name
    # input schemas are the flatc output: command fields and the definitions they reference
    assert "path" in tools["load_model"].input_schema["properties"]
    assert "napl_TestSpec" in tools["run_test"].input_schema["definitions"]
    assert (
        tools["sweep"].input_schema["properties"]["mode"]["$ref"]
        == "#/definitions/napl_SweepMode"
    )


def test_e2e_load_step_query_set(root):
    async def main():
        async with Client(NaplServer(root, allow_destructive=True).server) as c:
            assert (await call(c, "get_entity_state", path="/world/pole", attr="pos"))[
                0
            ]  # no model yet: error
            await ok(c, "load_model", path="pendulum.xml")
            assert HINGE in (await ok(c, "list_prims"))["prims"]
            assert (await ok(c, "step_simulation", n=10))["tick"] == 10
            pos = await ok(c, "get_entity_state", path="/world/pole", attr="pos")
            assert len(pos["values"]) == 3 and pos["unit"] == "m"
            await ok(c, "set_entity_state", path=HINGE, attr="qpos", values=[0.3])
            before = (await ok(c, "get_entity_state", path=HINGE, attr="qpos"))[
                "values"
            ][0]
            assert before == 0.3
            await ok(c, "step_simulation", n=50)
            after = (await ok(c, "get_entity_state", path=HINGE, attr="qpos"))[
                "values"
            ][0]
            assert after != before
            snap = (await ok(c, "snapshot"))["snapshot_id"]
            await ok(c, "step_simulation", n=5)
            await ok(c, "restore", id=snap)
            assert (await ok(c, "get_entity_state", path=HINGE, attr="qpos"))["values"][
                0
            ] == after
            # bad arguments are rejected by the schema (additionalProperties: false), not silently dropped
            assert (await call(c, "step_simulation", count=3))[0]

    asyncio.run(main())


def test_precision_is_lossless_for_reads(root):
    async def main():
        async with Client(NaplServer(root, allow_destructive=True).server) as c:
            await ok(c, "load_model", path="pendulum.xml")
            await ok(
                c,
                "set_entity_state",
                path=HINGE,
                attr="qpos",
                values=[1.2345678901234e-9],
            )
            return (await ok(c, "get_entity_state", path=HINGE, attr="qpos"))["values"][
                0
            ]

    assert (
        asyncio.run(main()) == 1.2345678901234e-9
    )  # the JSON edge would print this as 0.000000001235


def test_delete_run_needs_approval(root):
    async def prepare(c):
        await ok(c, "load_model", path="pendulum.xml")
        rid = (await ok(c, "run_test", spec=servo_spec()))["run_id"]
        assert (await poll(c, rid))["status"] == "done"
        return rid

    async def main():
        srv = NaplServer(root)  # no --allow-destructive
        async with Client(
            srv.server, elicitation_callback=accept
        ) as c:  # approve the load, then use the others
            rid = await prepare(c)
        run_dir = root / "runs" / rid
        assert run_dir.is_dir()

        async with Client(srv.server) as c:  # client without elicitation support
            err, text = await call(c, "delete_run", run_id=rid)
            assert err and "elicitation" in text, text
        async with Client(srv.server, elicitation_callback=decline) as c:
            err, text = await call(c, "delete_run", run_id=rid)
            assert err and "refused" in text, text
        assert run_dir.is_dir()
        async with Client(srv.server, elicitation_callback=accept) as c:
            assert (await ok(c, "delete_run", run_id=rid)) == {"deleted": rid}
        assert not run_dir.exists()
        # --allow-destructive skips the prompt
        srv2 = NaplServer(root, allow_destructive=True)
        async with Client(srv2.server) as c:
            await ok(c, "load_model", path="pendulum.xml")
            rid = (await ok(c, "run_test", spec=servo_spec()))["run_id"]
            await poll(c, rid)
            await ok(c, "delete_run", run_id=rid)
        assert not (root / "runs" / rid).exists()

    asyncio.run(main())


def test_load_model_refused_without_approval(root):
    async def main():
        srv = NaplServer(root)
        async with Client(srv.server) as c:
            err, text = await call(c, "load_model", path="pendulum.xml")
            assert err and "elicitation" in text, text
        async with Client(srv.server, elicitation_callback=decline) as c:
            err, text = await call(c, "load_model", path="pendulum.xml")
            assert err and "refused" in text, text
        assert srv.sim is None  # nothing changed

    asyncio.run(main())


def test_path_sandbox(root, tmp_path_factory):
    outside = tmp_path_factory.mktemp("outside")
    shutil.copy(DATA / "pendulum.xml", outside / "evil.xml")
    (root / "escape").symlink_to(
        outside, target_is_directory=True
    )  # symlink inside root pointing out

    async def main():
        srv = NaplServer(root, allow_destructive=True)
        async with Client(srv.server) as c:
            for bad in [
                "../../etc/passwd",
                "/etc/passwd",
                str(outside / "evil.xml"),
                "escape/evil.xml",
                "a/../../x",
            ]:
                err, text = await call(c, "load_model", path=bad)
                assert err and "outside the server root" in text, (bad, text)
            assert srv.sim is None
            await ok(c, "load_model", path="pendulum.xml")  # in-root still works
            for bad in ["../x", "a/b", "..", ".", "/etc", "runs/../../x", ""]:
                for tool in ("get_run", "delete_run"):
                    err, text = await call(c, tool, run_id=bad)
                    assert err and "invalid run_id" in text, (tool, bad, text)
            # the C++ bus applies its own root check to LoadModel as well
            err, text = await call(c, "get_run", run_id="nope")
            assert err and "not_found" in text
            r = srv.sim.call(
                {"cmd_type": "LoadModel", "cmd": {"path": str(outside / "evil.xml")}},
                elevate=True,
            )
            assert not r["ok"] and r["error"] == "permission_denied"

    asyncio.run(main())


def test_run_test_and_sweep_poll_to_done_and_match_direct(root):
    spec = servo_spec()
    params = [{"path": "/actuators/servo", "attr": "kp", "values": [0.5, 50.0]}]

    async def main():
        srv = NaplServer(root, allow_destructive=True)
        async with Client(srv.server) as c:
            await ok(c, "load_model", path="pendulum.xml")
            await ok(c, "step_simulation", n=3)
            tick = srv.sim.tick
            started = await ok(c, "run_test", spec=spec)
            assert (
                started["status"] in ("pending", "running", "done")
                and started["run_id"]
            )
            test = await poll(c, started["run_id"])
            sweep = await poll(
                c, (await ok(c, "sweep", spec=spec, params=params))["run_id"]
            )
            assert srv.sim.tick == tick  # jobs ran on forks
            job = await ok(
                c, "run_test", spec={"name": "bad", "duration": 1.0}
            )  # no monitors
            bad = await poll(c, job["run_id"])
            assert bad["status"] == "error" and "monitors" in bad["error"]
            return test, sweep

    test, sweep = asyncio.run(main())
    assert test["status"] == sweep["status"] == "done"
    s = napl.Sim.load(DATA / "pendulum.xml")
    s.step(3)  # same starting state as the MCP session
    direct = s.run_test(spec)
    assert test["result"]["monitors"] == direct["monitors"]
    assert (
        test["result"]["pass"] == direct["pass"]
        and test["result"]["end_tick"] == direct["end_tick"]
    )
    s2 = napl.Sim.load(DATA / "pendulum.xml")
    s2.step(3)
    rows = s2.sweep(spec, params)
    assert sweep["result"]["rows"] == rows
    assert [r["pass"] for r in rows] == [False, True]
    for r in (test, sweep):  # evidence on disk
        assert (root / "runs" / r["run_id"] / "result.json").is_file()
    assert (root / "runs" / test["run_id"] / "trace.csv").is_file()


def test_cpp_gate_holds_in_mcp_session(root):
    async def main():
        srv = NaplServer(
            root, allow_destructive=True
        )  # even with approval pre-granted, the session lacks the cap
        async with Client(srv.server) as c:
            await ok(c, "load_model", path="pendulum.xml")
        return srv.sim

    sim = asyncio.run(main())
    assert sim.caps == ["read", "control"]
    sim.step(5)
    load = {"cmd_type": "LoadModel", "cmd": {"path": str(root / "pendulum.xml")}}
    r = sim.call(load)  # raw call path, no elevation
    assert (
        not r["ok"]
        and r["error"] == "permission_denied"
        and "destructive" in r["message"]
    )
    assert sim.tick == 5  # refused before it did anything
    assert sim.fork().caps == [
        "read",
        "control",
    ]  # forks (background jobs) inherit the restriction
    assert sim.call(load, elevate=True)["ok"]  # one elevated call works...
    assert sim.tick == 0 and sim.caps == [
        "read",
        "control",
    ]  # ...and does not widen the session
    assert not sim.call(load)["ok"]


def test_step_limit_enforced(root):
    async def main():
        srv = NaplServer(root, allow_destructive=True, max_steps_per_call=100)
        async with Client(srv.server) as c:
            await ok(c, "load_model", path="pendulum.xml")
            assert (await ok(c, "step_simulation", n=100))[
                "tick"
            ] == 100  # at the limit: fine
            err, text = await call(c, "step_simulation", n=101)
            assert err and "limit_exceeded" in text
            assert srv.sim.tick == 100  # refused before any work
            # a forked job and an elevated call carry the same limit
            assert (
                srv.sim.fork().call({"cmd_type": "Step", "cmd": {"n": 101}})["error"]
                == "limit_exceeded"
            )
            r = srv.sim.call({"cmd_type": "Step", "cmd": {"n": 101}}, elevate=True)
            assert not r["ok"] and r["error"] == "limit_exceeded"

    asyncio.run(main())


def test_wall_limit_ends_run_and_sweep_with_limit_exceeded(root):
    spec = servo_spec()
    params = [{"path": "/actuators/servo", "attr": "kp", "values": [0.5, 50.0]}]

    async def main():
        srv = NaplServer(
            root, allow_destructive=True, max_wall_s=1e-6
        )  # forks inherit the session limits
        async with Client(srv.server) as c:
            await ok(c, "load_model", path="pendulum.xml")
            tick = srv.sim.tick
            test = await poll(c, (await ok(c, "run_test", spec=spec))["run_id"])
            sweep = await poll(
                c, (await ok(c, "sweep", spec=spec, params=params))["run_id"]
            )
            assert test["status"] == "error" and "max_wall_s" in test["error"]
            assert sweep["status"] == "error" and "limit_exceeded" in sweep["error"]
            assert srv.sim.tick == tick

    asyncio.run(main())


def test_cli_limit_flags_wired(root):
    params = StdioServerParameters(
        command=sys.executable,
        args=[
            "-m",
            "napl.cli",
            "mcp",
            "--root",
            str(root),
            "--allow-destructive",
            "--max-steps",
            "7",
            "--max-wall-s",
            "5",
        ],
        cwd=root,
    )

    async def main():
        async with Client(params) as c:
            await ok(c, "load_model", path="pendulum.xml")
            assert (await ok(c, "step_simulation", n=7))["tick"] == 7
            err, text = await call(c, "step_simulation", n=8)
            assert err and "limit_exceeded" in text and "max_steps_per_call=7" in text

    asyncio.run(main())


def test_stdio_smoke(root):
    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "napl.cli", "mcp", "--root", str(root), "--allow-destructive"],
        cwd=root,
    )

    async def main():
        async with Client(params) as c:
            assert {t.name for t in (await c.list_tools()).tools} == EXPECTED
            await ok(c, "load_model", path="pendulum.xml")
            return (await ok(c, "get_entity_state", path="/world/pole", attr="pos"))[
                "unit"
            ]

    assert asyncio.run(main()) == "m"


def test_http_smoke(root):
    with (
        socket.socket() as s
    ):  # OS-assigned free port (ephemeral range, never Replay's 5177-5194/8000/8010/8093)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    assert port not in (*range(5177, 5195), 8000, 8010, 8093)
    proc = subprocess.Popen(
        [sys.executable, "-m", "napl.cli", "mcp", "--http", "--host", "127.0.0.1", "--port", str(port),
         "--root", str(root), "--allow-destructive"],
        cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )  # fmt: skip

    async def main():
        end = time.monotonic() + 60
        while True:
            assert proc.poll() is None, proc.stderr.read().decode()
            try:
                async with Client(f"http://127.0.0.1:{port}/mcp") as c:
                    assert {t.name for t in (await c.list_tools()).tools} == EXPECTED
                    await ok(c, "load_model", path="pendulum.xml")
                    return (
                        await ok(c, "get_entity_state", path="/world/pole", attr="pos")
                    )["unit"]
            except Exception:
                if time.monotonic() > end:
                    raise
                await asyncio.sleep(0.3)

    try:
        assert asyncio.run(main()) == "m"
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    assert proc.poll() is not None
    with (
        pytest.raises(OSError),
        socket.create_connection(("127.0.0.1", port), timeout=1),
    ):
        pass  # server gone: connection refused
