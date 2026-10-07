"""M12 exit checks: Spawn / Delete (MuJoCo mjSpec attach + recompile): state kept, prims, replay, errors, permissions."""

import asyncio
import os
import re
import shutil
import subprocess
from pathlib import Path

import napl
import numpy as np
import pytest
from mcp import Client
from napl.mcp_server import NaplServer
from test_mcp import accept, call, decline, ok

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "tests" / "data"
PENDULUM = DATA / "pendulum.xml"
ARM = DATA / "two_link.urdf"
REPLAY_EXE = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))
HINGE = "/world/pole/hinge"
SERVO = "/actuators/servo"
FAR = [3.0, 0.0, 0.0]  # well clear of the pendulum: the arm cannot touch it


def spawn(s, at="/world/arm2", path=ARM, pos=FAR, **kw):
    cmd = {"path": str(path), "at": at, "pos": pos, **kw}
    return s.call({"cmd_type": "Spawn", "cmd": cmd})


def delete(s, path="/world/arm2"):
    return s.call({"cmd_type": "Delete", "cmd": {"path": path}})


def prims(s):
    return s.call({"cmd_type": "Query", "cmd": {"path": ""}})["prims"]


def arm_joints(s, root="/world/arm2"):
    return [
        p
        for p in prims(s)
        if p.startswith(root + "/") and p.rsplit("/", 1)[1] in ("shoulder", "elbow")
    ]


def pend(s):
    return (
        s.query(HINGE, "qpos")[0],
        s.query(HINGE, "qvel")[0],
        s.query(SERVO, "kp")[0],
        s.query("/actuators/torque", "ctrl")[0],
    )


def running_pendulum(log=True) -> napl.Sim:
    s = napl.Sim.load(PENDULUM)
    if log:
        s.start_log()
    s.set(HINGE, "qpos", 0.7)
    s.set(HINGE, "qvel", 0.3)
    s.set("/actuators/torque", "ctrl", 1.5)
    s.set(SERVO, "kp", 7.0)  # a model parameter: lives in mjModel, not in the spec
    s.step(25)
    return s


def replay(s, tmp_path, name="log"):
    """-> (tick, hash) of napl-replay on s's log; also checks that re-recording the replay is byte-identical."""
    assert REPLAY_EXE.exists(), f"build the C++ exe first: {REPLAY_EXE}"
    log = tmp_path / f"{name}.naplog"
    log.write_bytes(s.log())
    out = subprocess.run(
        [str(REPLAY_EXE), str(log), "--root", str(DATA)],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    m = re.fullmatch(r"tick=(\d+) state_hash=([0-9a-f]{16})\n", out)
    assert m, out
    rec = tmp_path / f"{name}.rec"
    subprocess.run(
        [str(REPLAY_EXE), str(log), "--root", str(DATA), "--record", str(rec)],
        capture_output=True,
        check=True,
    )
    assert rec.read_bytes() == log.read_bytes()
    return int(m[1]), m[2]


def test_spawn_keeps_state_adds_prims_then_delete(tmp_path):
    live, ref = running_pendulum(), running_pendulum(log=False)
    assert live.state_hash() == ref.state_hash()
    before, tick0 = pend(live), live.tick
    n_prims = len(prims(live))

    r = spawn(live)
    assert r["ok"], r
    assert live.tick == tick0
    for a, b in zip(
        pend(live), before
    ):  # bitwise: qpos, qvel, kp (model parameter) and ctrl all survive
        assert np.array_equal(a, b)
    assert "/world/arm2" in prims(live)
    assert len(arm_joints(live)) == 2, [p for p in prims(live) if "arm2" in p]
    assert len(prims(live)) > n_prims

    # the spawned arm is live: bent, it falls; the pendulum's trajectory is exactly the arm-free reference
    shoulder, elbow = arm_joints(live)
    live.set(elbow, "qpos", 0.5)
    for _ in range(40):
        live.step(5)
        ref.step(5)
        for a, b in zip(pend(live), pend(ref)):
            assert np.array_equal(a, b)
    assert abs(live.query(shoulder, "qvel")[0][0]) > 1e-3
    assert replay(live, tmp_path, "spawned") == (live.tick, f"{live.state_hash():016x}")

    # Delete: arm gone, pendulum state untouched, still the reference trajectory
    held = pend(live)
    assert delete(live)["ok"]
    assert not [p for p in prims(live) if "arm2" in p] and len(prims(live)) == n_prims
    for a, b in zip(pend(live), held):
        assert np.array_equal(a, b)
    live.step(30)
    ref.step(30)
    for a, b in zip(pend(live), pend(ref)):
        assert np.array_equal(a, b)
    assert replay(live, tmp_path, "deleted") == (live.tick, f"{live.state_hash():016x}")


def test_replay_when_log_starts_after_spawn_and_nested_spawn(tmp_path):
    s = running_pendulum(log=False)
    assert spawn(s)["ok"]
    s.start_log()  # the header now needs the spec of the already-edited model
    s.step(10)
    assert spawn(s, at="/world/arm2/base/arm3", pos=[0, 0, 1.0])["ok"], prims(s)
    assert len(arm_joints(s, "/world/arm2/base/arm3")) == 2
    s.step(10)
    assert delete(s, "/world/arm2")["ok"]  # removes the nested one too
    s.step(5)
    assert [p for p in prims(s) if "arm" in p] == []
    assert replay(s, tmp_path) == (s.tick, f"{s.state_hash():016x}")


def test_forks_survive_spawn_and_do_not_share_edits():
    s = running_pendulum(log=False)
    f = s.fork()
    assert spawn(s)["ok"]
    assert (
        not arm_joints(f) and len(arm_joints(s)) == 2
    )  # the fork keeps the old structure
    assert spawn(f, at="/world/other")["ok"]  # ... and can edit it independently
    assert [p for p in prims(f) if "arm2" in p] == [] and arm_joints(f, "/world/other")


def test_error_paths(tmp_path):
    s = running_pendulum()
    h = s.state_hash()
    outside = tmp_path / "evil.urdf"
    shutil.copy(ARM, outside)
    r = spawn(s, path=outside)
    assert not r["ok"] and r["error"] == "permission_denied"
    r = spawn(s, path="../../etc/passwd")
    assert not r["ok"] and r["error"] == "permission_denied"
    assert delete(s, "/world/nothing")["error"] == "not_found"
    assert delete(s, "/world")["error"] == "invalid_argument"
    assert delete(s, HINGE)["error"] == "invalid_argument"  # only bodies
    assert spawn(s, at="/world/nope/arm")["error"] == "not_found"  # unknown parent
    assert spawn(s, at="/elsewhere/arm")["error"] == "invalid_argument"
    assert spawn(s, at="/world/pole")["error"] == "invalid_argument"  # exists
    assert spawn(s, pos=[1, 2])["error"] == "invalid_argument"
    assert spawn(s, path=DATA / "missing.urdf")["error"] == "invalid_argument"
    assert s.state_hash() == h  # every failure left the world untouched
    assert spawn(s)["ok"] and spawn(s)["error"] == "invalid_argument"  # same name twice
    # session without the destructive cap: Delete is refused at the gate, Spawn (control) is not
    ro = napl.Sim.load(PENDULUM, caps=("read", "control"))
    assert spawn(ro)["ok"]
    r = delete(ro)
    assert (
        not r["ok"]
        and r["error"] == "permission_denied"
        and "destructive" in r["message"]
    )
    assert arm_joints(ro)


def test_stale_snapshot_rejected_cleanly():
    s = running_pendulum(log=False)
    snap = s.call({"cmd_type": "Snapshot", "cmd": {}})["snapshot_id"]
    blob = s.save()
    assert spawn(s)["ok"]
    h = s.state_hash()
    r = s.call({"cmd_type": "Restore", "cmd": {"id": snap}})
    assert not r["ok"] and "structure" in r["message"], r
    with pytest.raises(RuntimeError, match="structure"):
        s.restore(blob)
    assert s.state_hash() == h  # nothing was applied
    snap2 = s.call({"cmd_type": "Snapshot", "cmd": {}})["snapshot_id"]
    assert s.call({"cmd_type": "Restore", "cmd": {"id": snap2}})[
        "ok"
    ]  # same structure restores fine
    assert delete(s)["ok"]
    assert not s.call({"cmd_type": "Restore", "cmd": {"id": snap2}})[
        "ok"
    ]  # and the arm-era one no longer fits


@pytest.fixture
def root(tmp_path):
    shutil.copy(PENDULUM, tmp_path / "pendulum.xml")
    shutil.copy(ARM, tmp_path / "arm.urdf")
    return tmp_path


def test_mcp_spawn_free_delete_needs_approval(root):
    async def main():
        srv = NaplServer(
            root
        )  # default session: read + control, no --allow-destructive
        async with Client(srv.server, elicitation_callback=accept) as c:
            await ok(c, "load_model", path="pendulum.xml")
        async with Client(
            srv.server
        ) as c:  # no elicitation support: Spawn needs no approval, Delete does
            await ok(c, "spawn_entity", path="arm.urdf", at="/world/arm2", pos=FAR)
            assert len(arm_joints(srv.sim)) == 2
            err, text = await call(c, "delete_entity", path="/world/arm2")
            assert err and "elicitation" in text, text
            err, text = await call(c, "spawn_entity", path="../arm.urdf", at="/world/x")
            assert err and "outside the server root" in text, text
        async with Client(srv.server, elicitation_callback=decline) as c:
            err, text = await call(c, "delete_entity", path="/world/arm2")
            assert err and "refused" in text, text
        assert len(arm_joints(srv.sim)) == 2  # nothing changed
        direct = srv.sim.call(
            {"cmd_type": "Delete", "cmd": {"path": "/world/arm2"}}
        )  # bypassing the tool: the bus gates
        assert direct["error"] == "permission_denied"
        async with Client(srv.server, elicitation_callback=accept) as c:
            await ok(c, "delete_entity", path="/world/arm2")
        assert not arm_joints(srv.sim)

    asyncio.run(main())
