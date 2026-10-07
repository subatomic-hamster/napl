"""M6 exit checks: declarative tests (TestSpec / RunTest), `napl test`, and the runs/<id>/ evidence."""

import copy
import json
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

import napl
import pytest
from napl import report

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "tests" / "data"
PENDULUM = DATA / "pendulum.xml"
SERVO_TOML = DATA / "specs" / "servo.toml"
SERVO = "/actuators/servo"
REPLAY_EXE = Path(os.environ.get("NAPL_REPLAY_EXE", REPO / "build" / "napl-replay"))
NAPL = [sys.executable, "-m", "napl.cli"]  # same code as the `napl` console script


def servo_spec(kp: float) -> dict:
    """servo.toml as a TestSpec dict (what the CLI sends), with the kp setting replaced."""
    spec = tomllib.loads(SERVO_TOML.read_text())
    spec.pop("model")
    (kp_setting,) = [s for s in spec["settings"] if s["attr"] == "kp"]
    kp_setting["values"] = [kp]
    return spec


def run_servo(kp: float) -> dict:
    return napl.Sim.load(PENDULUM).run_test(servo_spec(kp))


def mon(result: dict, name: str) -> dict:
    (m,) = [m for m in result["monitors"] if m["name"] == name]
    return m


def cli(*args, runs_dir):
    return subprocess.run(
        [*NAPL, "test", *map(str, args), "--runs-dir", str(runs_dir)],
        capture_output=True,
        text=True,
    )


def test_kp50_passes_with_margin():
    r = run_servo(50.0)
    assert r["pass"] is True and not r["diverged"] and r["fail_monitor"] == ""
    assert r["fail_snapshot"] == b""
    reach, hold = mon(r, "reaches_target"), mon(r, "holds_target")
    assert (
        reach["hit"] and reach["hit_time"] < 0.5
    )  # satisfied-at, well inside the 1 s deadline
    # band is [0.4, 0.6] around the 0.5 rad target; steady state sags to ~0.477 under gravity (kp=50)
    assert (
        0.45 < hold["min_value"] and hold["max_value"] < 0.55
    )  # >= 0.05 rad margin to either edge
    assert abs(hold["final_value"] - 0.477) < 0.01
    assert r["end_tick"] - r["start_tick"] == 1500 and r["dt"] == 0.002


def test_kp_half_fails_and_reports_first_violation():
    r = run_servo(0.5)
    assert r["pass"] is False
    assert r["fail_monitor"] == "holds_target"
    dt = r["dt"]
    # the always window is [1.0, 3.0] s: first violation is in it, tick and time agree, value is out of band
    assert 1.0 <= r["fail_time"] <= 3.0
    assert r["fail_tick"] == round(r["fail_time"] / dt) + r["start_tick"]
    hold = mon(r, "holds_target")
    assert hold["pass"] is False and hold["hit"]
    assert hold["hit_tick"] == r["fail_tick"] and hold["hit_time"] == pytest.approx(
        r["fail_time"]
    )
    assert not 0.4 <= hold["hit_value"] <= 0.6
    assert (
        hold["max_value"] < 0.2
    )  # gravity beats a 0.5 N*m/rad spring: nowhere near the 0.5 rad target
    assert (
        mon(r, "reaches_target")["pass"] is False
        and not mon(r, "reaches_target")["hit"]
    )


def test_report_artifacts(tmp_path):
    spec = servo_spec(0.5)
    r = napl.Sim.load(PENDULUM).run_test(spec)
    d = report.write_report(r, spec, PENDULUM, tmp_path)
    assert d.parent == tmp_path and d.name.startswith("servo_step-")
    res = json.loads((d / "result.json").read_text())
    for key in (
        "run_id",
        "model",
        "spec",
        "pass",
        "monitors",
        "fail_monitor",
        "fail_tick",
        "fail_time",
        "fail_hash",
        "dt",
    ):
        assert key in res
    assert res["pass"] is False and res["fail_hash"] == r["fail_hash"]
    assert "trace" not in res and "fail_snapshot" not in res
    lines = (d / "trace.csv").read_text().splitlines()
    assert (
        lines[0] == "tick,t,/world/pole/hinge.qpos[0]"
    )  # both monitors share one signal column
    assert (
        len(lines) == 1 + 1500 // 10 + 1
    )  # tick 0, every 10th, and the last (1500 is aligned: no extra row)
    assert (d / "fail_snapshot.bin").read_bytes() == r["fail_snapshot"]
    # a passing re-run in the same dir id would drop the stale snapshot; different kp => different id here
    ok = napl.Sim.load(PENDULUM).run_test(servo_spec(50.0))
    d2 = report.write_report(ok, servo_spec(50.0), PENDULUM, tmp_path)
    assert (
        d2 != d
        and not (d2 / "fail_snapshot.bin").exists()
        and (d2 / "result.json").exists()
    )


def test_fail_snapshot_restores_state_and_gains():
    r = run_servo(0.5)
    fresh = napl.Sim.load(PENDULUM)
    assert fresh.query(SERVO, "kp")[0][0] == 0.0
    fresh.restore(r["fail_snapshot"])
    assert fresh.state_hash() == r["fail_hash"]
    assert fresh.tick == r["fail_tick"]
    kp, unit = fresh.query(SERVO, "kp")
    assert (
        kp[0] == 0.5 and unit == "N*m/rad"
    )  # the model attribute came back with the state
    assert fresh.query(SERVO, "kv")[0][0] == 2.0
    q = fresh.query("/world/pole/hinge", "qpos")[0][0]
    assert q == pytest.approx(r["monitors"][1]["hit_value"])


def test_snapshots_carry_model_attributes():
    s = napl.Sim.load(PENDULUM)
    s.set(SERVO, "kp", 7.0)
    blob, h7 = s.save(), s.state_hash()
    s.set(SERVO, "kp", 9.0)
    assert s.state_hash() != h7  # the hash sees gains
    s.restore(blob)
    assert s.query(SERVO, "kp")[0][0] == 7.0 and s.state_hash() == h7
    f = s.fork()
    assert f.query(SERVO, "kp")[0][0] == 7.0
    with pytest.raises(ValueError, match="not a position servo"):
        s.set("/actuators/torque", "kp", 1.0)
    with pytest.raises(ValueError):
        s.set(SERVO, "kp", -1.0)


def test_run_test_is_logged_and_replays(tmp_path):
    s = napl.Sim.load(PENDULUM)
    s.start_log()
    s.run_test(servo_spec(0.5))
    log = tmp_path / "t.naplog"
    log.write_bytes(s.log())
    out = subprocess.run(
        [str(REPLAY_EXE), str(log)], capture_output=True, text=True, check=True
    ).stdout
    m = re.fullmatch(r"tick=(\d+) state_hash=([0-9a-f]{16})\n", out)
    assert m and int(m[1]) == s.tick == 1500 and m[2] == f"{s.state_hash():016x}"


def test_cli_pass_and_fail_exit_codes(tmp_path):
    ok = cli(SERVO_TOML, runs_dir=tmp_path / "ok")
    assert ok.returncode == 0, ok.stdout + ok.stderr
    assert ok.stdout.startswith("PASS servo_step")
    bad = cli(SERVO_TOML, "--set", f"{SERVO}.kp=0.5", runs_dir=tmp_path / "bad")
    assert bad.returncode == 1, bad.stdout + bad.stderr
    assert "FAIL" in bad.stdout and "first violation: holds_target" in bad.stdout
    (rd,) = (tmp_path / "bad").iterdir()
    assert (rd / "fail_snapshot.bin").exists() and not any(
        (tmp_path / "ok").glob("*/fail_snapshot.bin")
    )
    # the real console script is installed too
    script = Path(sys.executable).parent / "napl"
    assert script.exists()
    assert (
        subprocess.run(
            [str(script), "test", str(SERVO_TOML), "--runs-dir", str(tmp_path / "s")]
        ).returncode
        == 0
    )


def test_toml_and_json_specs_give_identical_results(tmp_path):
    spec = servo_spec(0.5)
    js = tmp_path / "servo.json"
    js.write_text(json.dumps({**spec, "model": str(PENDULUM)}))
    a = cli(SERVO_TOML, "--set", f"{SERVO}.kp=0.5", runs_dir=tmp_path / "a")
    b = cli(js, runs_dir=tmp_path / "b")
    assert a.returncode == b.returncode == 1
    (ra,), (rb,) = (tmp_path / "a").iterdir(), (tmp_path / "b").iterdir()
    assert (ra / "trace.csv").read_bytes() == (rb / "trace.csv").read_bytes()
    assert (ra / "fail_snapshot.bin").read_bytes() == (
        rb / "fail_snapshot.bin"
    ).read_bytes()
    ja, jb = (
        json.loads((ra / "result.json").read_text()),
        json.loads((rb / "result.json").read_text()),
    )
    for j in (
        ja,
        jb,
    ):  # only the origin of the spec differs: model path spelling, set-vs-edited settings
        j.pop("model"), j.pop("run_id"), j.pop("spec")
    assert ja == jb
    # raw JSON text through the bus equals the dict path
    r_text = napl.Sim.load(PENDULUM).call(
        json.dumps({"cmd_type": "RunTest", "cmd": {"spec": spec}})
    )["result"]
    assert r_text["fail_hash"] == run_servo(0.5)["fail_hash"]


@pytest.mark.parametrize(
    "mutate, needle",
    [
        (lambda s: s.update(bogus_field=1), "bogus_field"),
        (lambda s: s["monitors"][0].update(op="sometimes"), "sometimes"),
        (
            lambda s: s["monitors"][0]["signal"].update(path="/world/nope"),
            "unknown prim",
        ),
        (lambda s: s["monitors"][0]["signal"].update(index=3), "out of range"),
        (lambda s: s.update(monitors=[]), "no monitors"),
        (lambda s: s.update(ticks=10), "exactly one"),
        (lambda s: s["monitors"][0].update(lo=1.0, hi=0.0), "lo <= hi"),
    ],
)
def test_malformed_spec_is_a_clear_error_exit_2(tmp_path, mutate, needle):
    spec = servo_spec(50.0)
    mutate(spec)
    with pytest.raises(ValueError, match=needle):
        napl.Sim.load(PENDULUM).run_test(copy.deepcopy(spec))
    js = tmp_path / "bad.json"
    js.write_text(json.dumps({**spec, "model": str(PENDULUM)}))
    r = cli(js, runs_dir=tmp_path / "runs")
    assert r.returncode == 2, r.stdout + r.stderr
    assert r.stderr.startswith("napl: error:") and needle in r.stderr
    assert not (tmp_path / "runs").exists()  # no evidence for a test that never ran


def test_cli_usage_errors_exit_2(tmp_path):
    assert cli(tmp_path / "missing.toml", runs_dir=tmp_path).returncode == 2
    assert cli(SERVO_TOML, "--set", "nonsense", runs_dir=tmp_path).returncode == 2
    assert (
        cli(SERVO_TOML, "--model", tmp_path / "no.xml", runs_dir=tmp_path).returncode
        == 2
    )
