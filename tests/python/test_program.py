"""`napl program`: user programs against the simulated robots, run as separate processes, with pass/fail evidence.

Arm: examples/so101/pick.scenario.toml (the SO-101 pick through the Feetech pty, ~40 s real time each run). Broken programs fail
with evidence; a wall timeout kills the whole process group (children included, checked in /proc); rlimits are enforced; the replay
log reproduces the final state hash. PX4 mission (marker px4): examples/x500/mission.scenario.toml.
Needs the built kernel + build/napl-replay (hfbox: tools/remote.sh run --nolock '.venv/bin/python -m pytest tests/python/test_program.py -v -s').
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
from napl import program, px4

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples"
ARM3 = REPO / "tests" / "data" / "feetech" / "arm3.xml"
PICK = EX / "so101" / "pick.scenario.toml"


def scen(tmp_path, body, limits="", monitors=""):
    """A tiny scenario on the 3-servo test arm (fast): `body` is the program source."""
    prog = tmp_path / "prog.py"
    prog.write_text(body)
    f = tmp_path / "t.scenario.toml"
    f.write_text(
        f'name = "t"\n[model]\nmjcf = "{ARM3}"\n'
        '[interfaces.feetech]\nservos = { 1 = "/devices/s1", 2 = "/devices/s2", 3 = "/devices/s3" }\n'
        f'[program]\ncmd = ["python", "{prog}"]\n[limits]\n{limits}\n{monitors}'
    )
    return f


def pick_variant(tmp_path, src):
    """The pick scenario with its program replaced by `src` (scene.py and sibling modules by absolute path)."""
    prog = tmp_path / "variant.py"
    prog.write_text(f"import sys\nsys.path.insert(0, {str(EX / 'so101')!r})\n{src}")
    t = (
        PICK.read_text()
        .replace('"{dir}/pick.py"', f'"{prog}"')
        .replace('script = "scene.py"', f'script = "{EX / "so101" / "scene.py"}"')
    )
    f = tmp_path / "pick.scenario.toml"
    f.write_text(t)
    return f


def failed(res):
    return {m["name"] for m in res["monitors"] if not m["pass"]}


def test_judge_ops():
    s = [(0.0, 1.0), (1.0, 5.0), (2.0, 2.0)]
    j = lambda **m: program.judge({"name": "m", **m}, s)
    assert not j(op="always", hi=3)["pass"] and j(op="always", hi=3)["hit_t"] == 1.0
    assert j(op="always", hi=5)["pass"]
    assert j(op="eventually", lo=5)["pass"] and not j(op="eventually", lo=6)["pass"]
    assert j(op="final", hi=2)["pass"] and not j(op="final", hi=1.5)["pass"]
    assert j(op="always", hi=3, **{"from": 2.0})[
        "pass"
    ]  # the violation at t=1 is outside the window


def test_pick_scenario_passes_and_replays(tmp_path):
    res, d = program.run(PICK, tmp_path)
    print(f"\n[pick] {res['program']} sim {res['sim']['sim_s']:.1f} s")
    for m in res["monitors"]:
        print(
            f"[pick]   {m['name']}: pass={m['pass']} min {m['min']:.4g} max {m['max']:.4g} final {m['final']:.4g}"
        )
    assert res["pass"], res["fail_reasons"]
    assert res["program"]["returncode"] == 0 and not res["program"]["timeout"]
    assert not res["sim"]["port_errors"] and not res["sim"]["diverged"]
    assert {p.name for p in d.iterdir()} >= {
        "result.json",
        "program.log",
        "replay.naplog",
        "trace.csv",
    }
    assert "done:" in (d / "program.log").read_text()
    # replay: the log of the run reproduces the final state hash (napl-replay, the C++ replayer, over the saved log)
    assert program.REPLAY.exists(), f"build the C++ exe first: {program.REPLAY}"
    tick, h = program.replay_hash(
        d / "replay.naplog", next((d / "model").glob("scene_*.xml")).parent
    )
    print(f"[pick] replay tick {tick} hash {h} vs run {res['sim']['state_hash']}")
    assert (
        h == res["sim"]["state_hash"]
        and tick == res["sim"]["tick"]
        and res["replay"]["match"]
    )


def test_wrong_servo_id_fails_with_evidence(tmp_path):
    res, d = program.run(
        pick_variant(
            tmp_path,
            "import os, sts\nb = sts.Bus(os.environ['NAPL_SERIAL'])\nb.send(7, 3, bytes([40, 1]))\nassert b.recv(1), 'servo 7 never answered'\n",
        ),
        tmp_path,
    )
    log = (d / "program.log").read_text()
    print(f"\n[wrong id] fail_reasons {res['fail_reasons']}")
    assert not res["pass"] and res["program"]["returncode"] != 0
    assert "servo 7 never answered" in log  # the program's own evidence
    assert "cube_within_1cm_of_target" in failed(
        res
    )  # the cube never moved: 9 cm from the target
    assert "cube_was_lifted" in failed(res)


def test_never_releasing_the_cube_fails_with_evidence(tmp_path):
    src = """import pick
orig = pick.plan
def plan(c, t, gs):  # the same trajectory, but the gripper never opens again
    rows, q = orig(c, t, gs)
    return [(qa, gs if lab in ("open", "released", "retreat", "cube settles") else g, lab) for qa, g, lab in rows], q
pick.plan = plan
pick.main()
"""
    res, d = program.run(pick_variant(tmp_path, src), tmp_path)
    print(f"\n[no release] fail_reasons {res['fail_reasons']}")
    assert (
        not res["pass"] and res["program"]["returncode"] == 0
    )  # the program is happy: only the monitors see it
    assert "cube_on_table" in failed(res)  # the cube left the table with the arm
    assert next(m for m in res["monitors"] if m["name"] == "cube_was_lifted")["pass"]


def _marker_pids(marker):
    out = []
    for p in Path("/proc").iterdir():
        if p.name.isdigit():
            try:
                if marker in (p / "cmdline").read_bytes().decode(errors="replace"):
                    out.append(int(p.name))
            except OSError:
                pass
    return out


def test_wall_timeout_kills_the_whole_process_group(tmp_path):
    marker = f"napl-orphan-{os.getpid()}"
    body = f"import subprocess, time\nsubprocess.Popen(['bash', '-c', 'exec -a {marker} sleep 300'])\nsubprocess.Popen(['bash', '-c', 'sleep 300 & wait'])\ntime.sleep(300)\n"
    t0 = time.monotonic()
    res, d = program.run(scen(tmp_path, body, "wall_s = 3"), tmp_path)
    took = time.monotonic() - t0
    print(f"\n[timeout] {res['program']} total {took:.1f} s")
    assert (
        res["program"]["timeout"] == "wall"
        and res["program"]["signal"] in ("SIGTERM", "SIGKILL")
        and not res["pass"]
    )
    assert (
        took < 8
    )  # 3 s wall limit + SIGTERM: no 2 s + 5 s grace waits on a zombie leader
    time.sleep(0.2)
    assert _marker_pids(marker) == [], "orphaned child survived the group kill"
    try:  # the group itself is gone
        os.killpg(res["program"]["pgid"], 0)
        raise AssertionError("process group still alive")
    except ProcessLookupError:
        pass


def test_program_that_exits_leaves_no_background_children(tmp_path):
    marker = f"napl-bg-{os.getpid()}"
    body = f"import subprocess\nsubprocess.Popen(['bash', '-c', 'exec -a {marker} sleep 300'])\n"
    res, _ = program.run(scen(tmp_path, body), tmp_path)
    assert res["pass"] and not res["program"]["timeout"]
    time.sleep(0.2)
    assert _marker_pids(marker) == []


@pytest.mark.parametrize(
    "limits, body, sig, needle",
    [
        (
            "address_space_mb = 400",
            "x = bytearray(1500 * 2**20)\n",
            None,
            "MemoryError",
        ),
        ("cpu_s = 1", "while True:\n    pass\n", "SIGXCPU", ""),
        (
            "open_files = 16",
            "fs = [open('/dev/null') for _ in range(64)]\n",
            None,
            "Too many open files",
        ),
    ],
    ids=["address_space", "cpu", "open_files"],
)
def test_resource_limits_are_enforced(tmp_path, limits, body, sig, needle):
    res, d = program.run(scen(tmp_path, body, limits + "\nwall_s = 60"), tmp_path)
    print(f"\n[rlimit {limits}] {res['program']}")
    assert (
        not res["pass"]
        and res["program"]["returncode"] != 0
        and not res["program"]["timeout"]
    )
    assert res["program"]["signal"] == sig
    assert needle in (d / "program.log").read_text()


def test_environment_and_scenario_dir(tmp_path):
    body = "import os, stat\nassert stat.S_ISCHR(os.stat(os.environ['NAPL_SERIAL']).st_mode)\nassert os.path.isdir(os.environ['NAPL_SCENARIO_DIR'])\nprint('cwd', os.getcwd())\n"
    res, d = program.run(scen(tmp_path, body), tmp_path)
    assert res["pass"], (res["fail_reasons"], (d / "program.log").read_text())
    assert f"cwd {d / 'cwd'}" in (d / "program.log").read_text()


def test_cli_exit_codes(tmp_path):
    f = scen(tmp_path, "raise SystemExit(3)\n")
    r = subprocess.run(
        [
            sys.executable,
            "-m",
            "napl.cli",
            "program",
            str(f),
            "--runs-dir",
            str(tmp_path / "r"),
        ],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 1 and "FAIL t" in r.stdout, (r.stdout, r.stderr)
    r = subprocess.run(
        [sys.executable, "-m", "napl.cli", "program", str(tmp_path / "missing.toml")],
        capture_output=True,
        text=True,
    )
    assert r.returncode == 2


@pytest.mark.px4
@pytest.mark.skipif(px4.skip_reason() is not None, reason=px4.skip_reason() or "")
def test_x500_mission_scenario(tmp_path):
    res, d = program.run(EX / "x500" / "mission.scenario.toml", tmp_path)
    print(f"\n[mission] program {res['program']} sim {res['sim']}")
    for m in res["monitors"]:
        print(
            f"[mission]   {m['name']}: pass={m['pass']} min {m['min']:.4g} max {m['max']:.4g} final {m['final']:.4g}"
        )
    print(f"[mission] replay {res['replay']}")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["match"]
    assert {p.name for p in d.iterdir()} >= {
        "result.json",
        "program.log",
        "replay.naplog",
        "trace.csv",
        "bridge.log",
        "px4.log",
    }
