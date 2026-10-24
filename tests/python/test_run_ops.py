"""The viewer's Test / Run button: run.* ops on a Backend (no window needed). Bench: the component's device model on a short
bench (Skydio motor / ESC, SO-101 servo, battery, IMU) + the tests tied to the open file. Full: the Skydio scenario runs LIVE in a
host=True Sim (the one a window would show) at rate 0 for speed: pass, replay hash = `napl program`'s (examples/skydio/README.md);
stopping mid-run kills the program's process group. Needs the built kernel + build/napl-replay (hfbox, tools/remote.sh run --nolock)."""

import json
import os
import sys
import time
from pathlib import Path

import napl
import pytest
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "skydio"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
HASH = {
    "goal": "efb57e183637ae9d",
    "follow": "164b5cc71536dfec",
}  # examples/skydio/README.md


def ask(b, tool, args=None):
    r = json.loads(b(json.dumps({"tool": tool, "args": args or {}})))
    assert r["ok"], r
    return r["result"]


def finish(b, rid, timeout=120):
    out, t0 = "", time.monotonic()
    while time.monotonic() - t0 < timeout:
        r = ask(b, "run.poll", {"run_id": rid})
        out += r["out"]
        if r["state"] != "running":
            return r, out
        time.sleep(0.2)
    raise AssertionError(f"run still going after {timeout} s:\n{out[-2000:]}")


@pytest.fixture
def skydio(tmp_path):
    return Backend(
        lambda d=Design.open(REPO / "assemblies" / "skydio.toml"): d, tmp_path / "out"
    )


@needs_kernel
def test_targets_list_scenarios_and_components_with_models_first(skydio):
    t = ask(skydio, "run.targets")
    assert (
        "follow.scenario.toml" in t["scenarios"]
    )  # relative to the IDE's root, examples/skydio
    kinds = [c["kind"] for c in t["components"]]
    assert kinds[0] and kinds == sorted(
        kinds, key=lambda k: not k
    )  # modelled components first
    assert next(c["kind"] for c in t["components"] if c["id"] == "esc1") == "esc"


@needs_kernel
@pytest.mark.parametrize("comp,keys", [
    ("esc1", ["esc1.max_thrust_N", "esc1.max_bus_current_A", "esc1.supply_sag_V"]),
    ("mot1", ["esc1.max_rpm"]),
    ("bat", ["battery.sag_V", "battery.v_idle"]),
    ("icm", ["imu.gyro[x].noise", "imu.accel[z].noise"]),
])  # fmt: skip
def test_bench_skydio_components(skydio, comp, keys):
    rid = ask(skydio, "run.start", {"mode": "bench", "component": comp})["run_id"]
    r, out = finish(skydio, rid)
    assert r["state"] == "done" and r["result"]["pass"], out
    m = r["result"]["metrics"]
    assert all(k in m for k in keys), (keys, list(m))
    if (
        comp == "esc1"
    ):  # 2216 / 10 inch: ~1 kgf at full throttle, ~15-25 A from the pack, which sags a little but stays up
        assert 5 < m["esc1.max_thrust_N"] < 30 and 5 < m["esc1.max_bus_current_A"] < 40
        assert 0 < m["esc1.supply_sag_V"] < 4
        assert (
            0.3 * m["esc1.max_thrust_N"]
            < m["esc1.thrust_50_N"]
            < 0.7 * m["esc1.max_thrust_N"]
        )
        assert (
            "thrust N" in out and "PASS" in out
        )  # the table reached the terminal stream
    if comp == "icm":
        assert 0 < m["imu.gyro[x].noise"] < 0.1 and 0 < m["imu.accel[z].noise"] < 1


@needs_kernel
def test_bench_servo_step_response():
    b = Backend(
        lambda d=Design.open(REPO / "assemblies" / "so101.toml"): d,
        Path(os.environ.get("TMPDIR", "/tmp")) / "napl-run-servo",
    )
    rid = ask(b, "run.start", {"mode": "bench", "component": "servo5"})[
        "run_id"
    ]  # wrist: nearly unloaded
    r, out = finish(b, rid)
    m = r["result"]["metrics"]
    assert r["state"] == "done" and r["result"]["pass"], out
    assert (
        0 < m["servo5.rise_s"] < 0.5
        and 0 <= m["servo5.overshoot_pct"] < 50
        and m["servo5.peak_current_A"] > 0.05
    ), m
    assert abs(m["servo5.steady_error_rad"]) < 0.02 and "rise" in out


@needs_kernel
def test_component_without_device_model_runs_only_the_tests(tmp_path):
    prog = (
        tmp_path / "prog"
    )  # the program root is the IDE's (Prog.root), here an explicit one
    prog.mkdir()
    (prog / "foo.py").write_text("X = 1\n")
    (prog / "test_foo.py").write_text("def test_x():\n    assert 1\n")
    (prog / "test_bar.py").write_text(
        "import foo\n\n\ndef test_y():\n    assert foo.X == 1\n"
    )
    d = Design.open(REPO / "assemblies" / "skydio.toml")
    b = Backend(lambda: d, tmp_path / "out", program_root=prog)
    inst = next(c["id"] for c in ask(b, "run.targets")["components"] if not c["kind"])
    rid = ask(
        b, "run.start", {"mode": "bench", "component": inst, "files": ["foo.py"]}
    )["run_id"]
    r, out = finish(b, rid, timeout=600)
    assert "no device model" in out and r["result"]["metrics"] == {}
    assert r["result"]["tests"] == [
        "test_foo.py"
    ]  # test_<name>.py targets foo.py; test_bar.py merely imports it
    assert r["result"]["tests_pass"] and r["result"]["pass"] and "1 passed" in out, out[
        -800:
    ]
    assert "budget" in out  # the terminal says what runs and how long it may take

    # no test_<stem>.py: the importers run narrowed with -k <stem> (a big suite that merely imports the module is not the bench)
    (prog / "baz.py").write_text("Y = 2\n")
    (prog / "test_suite.py").write_text(
        "import baz\n\n\ndef test_baz_y():\n    assert baz.Y == 2\n\n\ndef test_unrelated_slow():\n    raise SystemExit('must not run')\n"
    )
    rid = ask(
        b, "run.start", {"mode": "bench", "component": inst, "files": ["baz.py"]}
    )["run_id"]
    r, out = finish(b, rid, timeout=600)
    assert (
        r["result"]["tests"] == ["test_suite.py"]
        and "-k baz" in out
        and "1 passed" in out
        and "1 deselected" in out
    ), out[-800:]


def test_chase_camera_steps_around_a_tree_and_eases():
    import numpy as np
    from napl.design.runops import Runs

    class Sim:  # a tree standing south-west of the vehicle: rays towards -x,-y hit it 2 m out
        def raycast(self, origin, dirs):
            u = np.asarray(dirs[0])
            return np.array([2.0 if u[0] < 0 and u[1] < 0 else -1.0])

    p, st = np.array([0.0, 0.0, 2.0]), {}
    e1 = Runs._chase_eye(Sim(), p, st)
    assert (
        e1[0] > -0.1 or e1[1] > -0.1
    )  # not behind the tree (the old fixed offset was (-4, -4, +3.5))
    az = st["az"]
    e2 = Runs._chase_eye(
        Sim(), p + [1, 0, 0], st
    )  # the vehicle moves on: same clear azimuth, camera eases (does not jump)
    assert st["az"] == az and 0 < np.linalg.norm(e2 - e1) < 1.2

    class Walled:  # boxed in on every side: a spring arm, the camera right next to the vehicle (never behind the wall)
        def raycast(self, origin, dirs):
            return np.array([0.5])

    assert np.linalg.norm(Runs._chase_eye(Walled(), p, {}) - p) == pytest.approx(0.8)


def test_root_is_the_ides_program_root(tmp_path):
    prog = tmp_path / "prog"
    prog.mkdir()
    (prog / "a.scenario.toml").write_text("name='a'\n")
    (prog / "sub").mkdir()
    (prog / "sub" / "b.scenario.toml").write_text("name='b'\n")
    d = Design.open(REPO / "assemblies" / "skydio.toml")
    b = Backend(lambda: d, tmp_path / "out", program_root=prog)
    assert ask(b, "run.targets")["scenarios"] == [
        "a.scenario.toml",
        "sub/b.scenario.toml",
    ]
    skydio = Backend(
        lambda: d, tmp_path / "o2"
    )  # default root: examples/skydio beside assemblies/skydio.toml
    assert "follow.scenario.toml" in ask(skydio, "run.targets")["scenarios"]
    r = json.loads(
        b(
            json.dumps(
                {
                    "tool": "run.start",
                    "args": {
                        "mode": "full",
                        "scenario": "../../examples/skydio/follow.scenario.toml",
                    },
                }
            )
        )
    )
    assert not r["ok"] and "outside the program root" in r["error"]


def test_paths_are_confined(skydio):
    for bad in ("../x.scenario.toml", "/etc/passwd"):
        r = json.loads(
            skydio(
                json.dumps(
                    {"tool": "run.start", "args": {"mode": "full", "scenario": bad}}
                )
            )
        )
        assert not r["ok"] and "outside the program root" in r["error"]
    r = json.loads(
        skydio(
            json.dumps(
                {
                    "tool": "run.start",
                    "args": {
                        "mode": "bench",
                        "component": "esc1",
                        "files": ["../../etc/passwd"],
                    },
                }
            )
        )
    )
    assert not r["ok"] and "outside" in r["error"]


def live_backend(tmp_path, monkeypatch, scenario):
    monkeypatch.setenv("SKYDIO_CAL", str(tmp_path / "cal"))
    d = Design.open(REPO / "assemblies" / "skydio.toml")
    b = Backend(lambda: d, tmp_path / "out", program_root=Path(scenario).parent)
    b.sim = napl.Sim.load(
        b.state(d)["xml"], root=b.out, host=True
    )  # what napl.design.viewer.view() hands to Sim.show
    b.run_rate = 0.0  # as fast as possible (the viewer demo runs at rate 1)
    return b


@needs_kernel
@pytest.mark.parametrize("mission", ["goal", "follow"])
def test_full_run_live_matches_napl_program_hash(tmp_path, monkeypatch, mission, unsandboxed_scenario):
    scenario = unsandboxed_scenario(EX / f"{mission}.scenario.toml")
    b = live_backend(tmp_path, monkeypatch, scenario)
    rid = ask(
        b,
        "run.start",
        {"mode": "full", "scenario": f"{mission}.scenario.toml"},
    )["run_id"]
    r, out = finish(b, rid, timeout=900)
    res = r["result"]
    from napl import program
    reference, _ = program.run(scenario, tmp_path / "standalone")
    assert res["sim"]["state_hash"] == reference["sim"]["state_hash"]
    assert res["sim"]["tick"] == reference["sim"]["tick"]
    assert r["state"] == "done" and res["pass"], (res.get("fail_reasons"), out[-1500:])
    assert res["replay"]["match"]  # independently reproduces this live session
    if sys.platform != "darwin":  # hfbox Linux reference; Darwin builds produce different but locally-reproducible bits
        assert res["sim"]["state_hash"] == HASH[mission]  # same bits as `napl program` on hfbox
    assert "PASS" in out and (Path(res["run_dir"]) / "replay.naplog").exists()
    assert (
        b.sim.tick > 0 and b.runs.held
    )  # the run's final scene stays in the window (the verdict is visible there)
    assert (
        ask(b, "run.restore")["restored"] and b.sim.tick == 0
    )  # the window left SIMULATE: the design's own model is back
    assert not ask(b, "run.restore")["restored"]


@needs_kernel
def test_live_replay_check_covers_the_windows_own_design_load(tmp_path, monkeypatch, unsandboxed_scenario):
    """The viewer's World logs the design's own LoadModel (the window reloading the design) before the run's: the replay
    check must replay those under the Sim's root, not the run's model folder (it said 'replay diverged ... permission_denied')."""
    b = live_backend(tmp_path, monkeypatch, unsandboxed_scenario(EX / "goal.scenario.toml"))
    xml = b.state(b.get_design())["xml"]
    assert b.sim.call({"cmd_type": "LoadModel", "cmd": {"path": str(xml)}})["ok"]
    rid = ask(b, "run.start", {"mode": "full", "scenario": "goal.scenario.toml"})[
        "run_id"
    ]
    r, out = finish(b, rid, timeout=900)
    res = r["result"]
    assert r["state"] == "done" and res["replay"]["match"], out[-1500:]


@needs_kernel
def test_stop_mid_run_kills_the_program_group(tmp_path, monkeypatch, unsandboxed_scenario):
    b = live_backend(tmp_path, monkeypatch, unsandboxed_scenario(EX / "follow.scenario.toml"))
    rid = ask(
        b,
        "run.start",
        {"mode": "full", "scenario": "follow.scenario.toml"},
    )["run_id"]
    t0, tick = time.monotonic(), 0
    while (
        time.monotonic() - t0 < 120 and tick < 2000
    ):  # the program is flying (attached: the viewer's clock advances)
        time.sleep(0.5)
        tick = b.sim.tick
    assert tick >= 2000
    ask(b, "run.stop", {"run_id": rid})
    r, out = finish(b, rid, timeout=60)
    assert r["state"] == "stopped" and out.rstrip().endswith("stopped"), out[-500:]
    pg = r["result"]["program"]["pgid"]
    with pytest.raises(ProcessLookupError):
        os.killpg(pg, 0)
