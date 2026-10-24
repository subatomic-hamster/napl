"""Skydio landing demo, flights (kernel): every scenario of examples/skydio-land passes its monitors, replays to the same state hash, and ends on the golden hash recorded
below; the PROGRAM page's Run slot lists them and runs one. Slow (3 to 12 minutes per scenario on the CPU box): `-k forest` for the quick one.

The golden hashes belong to this code, this kernel build and the scenario's default seed. A change of the autonomy, the worlds, the design or the kernel moves them: re-record with
`napl program <scenario> --runs-dir <dir>` (result.json: sim.state_hash) after reading why they moved."""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import napl
import pytest
from napl import program
from napl.design import Design
from napl.design.viewer import Backend

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "skydio-land"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")

# scenario (path under examples/skydio-land) -> golden state hash (seed = the scenario's default)
GOLDEN = {
    "forest/clearing": "038fa2d363a24d5d",
    "rain/rain25": "71a66a806d2cd638",
    "rain/rain100": "71ce2471592209e5",
    "summit/summit": "79317ded795275e8",
    "ledge/ledge": "777bb343889a8d75",
    "mesa_brink/mesa_brink": "6871181af1b16932",
    "deck/osv": "56be8fda55e8b952",
    "deck/asv": "e05595b215d5c2e5",
}


def ask(b, tool, args=None):
    r = json.loads(b(json.dumps({"tool": tool, "args": args or {}})))
    assert r["ok"], r
    return r["result"]


def test_every_scenario_is_listed_and_has_the_standard_monitors():
    """No kernel needed: the files exist, parse, and carry the landing monitors."""
    import tomllib

    found = sorted(
        str(p.relative_to(EX)).removesuffix(".scenario.toml")
        for p in EX.rglob("*.scenario.toml")
    )
    assert found == sorted(GOLDEN)
    for name in found:
        sc = tomllib.loads((EX / f"{name}.scenario.toml").read_text())
        mon = {m["name"] for m in sc["monitors"]}
        assert {
            "no_collision",
            "no_crash",
            "touchdown_vertical_speed",
            "stays_upright",
        } <= mon
        assert any(m.startswith("lands_within") for m in mon)
        assert any(m.startswith("on_a_safe") for m in mon)


def test_run_targets_list_the_landing_scenarios_recursively(tmp_path):
    d = Design.open(REPO / "assemblies" / "skydio-land.toml")
    b = Backend(lambda: d, tmp_path / "out", program_root=EX)
    listed = ask(b, "run.targets")["scenarios"]
    assert sorted(listed) == sorted(
        f"{n}.scenario.toml" for n in GOLDEN
    )  # 'forest/clearing.scenario.toml', ...


@needs_kernel
def test_short_flight_replays_bit_identically(tmp_path):
    """Two 8 s forest flights (LAND_T_MAX) with the same seed end on the same state hash."""
    hashes = []
    for k in range(2):
        out = tmp_path / f"r{k}"
        r = subprocess.run(
            [sys.executable, str(EX / "land_fly.py"), str(EX / "forest" / "clearing.scenario.toml"), str(out)],
            capture_output=True, text=True,
            env={**os.environ, "SKYDIO_CAL": str(tmp_path / "cal"), "LAND_T_MAX": "8"},
        )  # fmt: skip
        assert r.returncode == 0, r.stderr[-2000:]
        s = json.loads((out / "summary.json").read_text())
        assert s["contact_steps"] == 0 and s["crash_steps"] == 0
        hashes.append(json.loads((out / "native.json").read_text())["state_hash"])
    assert hashes[0] == hashes[1]


@needs_kernel
@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_scenario_passes_replays_and_matches_its_golden_hash(
    tmp_path, monkeypatch, name, unsandboxed_scenario
):
    monkeypatch.setenv("SKYDIO_CAL", str(tmp_path / "cal"))
    monkeypatch.setenv(
        "NAPL_ROUTE_CACHE", str(tmp_path / "route-cache")
    )  # (the ship scenes route hundreds of wires)
    res, _ = program.run(unsandboxed_scenario(EX / f"{name}.scenario.toml"), tmp_path / "runs")
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["pass"], res["fail_reasons"]
    if sys.platform != "darwin":
        assert res["sim"]["state_hash"] == GOLDEN[name], res["sim"]["state_hash"]


@needs_kernel
def test_run_slot_runs_a_landing_scenario_live(tmp_path, monkeypatch, unsandboxed_scenario):
    """The viewer's Run button on examples/skydio-land: the forest landing runs LIVE, passes, and ends on the same bits as `napl program`."""
    monkeypatch.setenv("SKYDIO_CAL", str(tmp_path / "cal"))
    d = Design.open(REPO / "assemblies" / "skydio-land.toml")
    scenario = unsandboxed_scenario(EX / "forest/clearing.scenario.toml")
    b = Backend(lambda: d, tmp_path / "out", program_root=scenario.parents[1])
    b.sim = napl.Sim.load(b.state(d)["xml"], root=b.out, host=True)
    b.run_rate = 0.0
    rid = ask(
        b, "run.start", {"mode": "full", "scenario": "forest/clearing.scenario.toml"}
    )["run_id"]
    out, t0 = "", time.monotonic()
    while True:
        r = ask(b, "run.poll", {"run_id": rid})
        out += r["out"]
        if r["state"] != "running":
            break
        assert time.monotonic() - t0 < 1500, out[-2000:]
        time.sleep(0.5)
    res = r["result"]
    assert r["state"] == "done" and res["pass"], (res.get("fail_reasons"), out[-1500:])
    assert res["replay"]["match"]
    if sys.platform != "darwin":
        assert res["sim"]["state_hash"] == GOLDEN["forest/clearing"], res["sim"]["state_hash"]
