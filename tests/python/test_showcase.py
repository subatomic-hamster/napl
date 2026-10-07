"""N7 flagship: in ONE physics world the real-parts SO-101 arm (Feetech STS pty) picks a payload and places it on the real-parts
X500 V2's top plate, then real PX4 SITL (MAVLink via napl-px4-bridge) flies the loaded drone. examples/showcase/arm_loads_drone.scenario.toml
runs it through `napl program`: both robots live in the bridge's World, the arm's pty is served over the bridge's --ctl socket (see
python/napl/program.py `_combo`), the monitors and the replay-hash check are the scenario's. The control flight
(fly_unloaded.scenario.toml: same scene and program, no payload) shows the physics consequence: the loaded hover draws more current.

Skipped unless the PX4 build, the bridge exe, napl-replay and mavsdk are present (python/napl/px4.py). ~90 s of wall clock per flight.
Run: .venv/bin/python -m pytest tests/python/test_showcase.py -m px4 -v -s
Writes build/showcase/showcase_result.json (the numbers quoted in docs)."""

import csv
import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest
from napl import program, px4

pytestmark = pytest.mark.px4
REPO = px4.REPO
SC = REPO / "examples" / "showcase"
SKIP = px4.skip_reason()
OUT = REPO / "build" / "showcase"


def hover_current(run_dir, alt_min=3.6, v_max=0.2):
    """Mean battery current (A) over the samples hovering above alt_min (m above the start) slower than v_max (m/s)."""
    with open(Path(run_dir) / "trace.csv") as f:
        rows = list(csv.DictReader(f))
    c = {
        k: np.array([float(r[k]) for r in rows])
        for k in ("z", "vx", "vy", "vz", "batt_i", "batt_v")
    }
    m = (c["z"] - c["z"][0] > alt_min) & (
        np.sqrt(c["vx"] ** 2 + c["vy"] ** 2 + c["vz"] ** 2) < v_max
    )
    assert m.sum() > 1000, f"only {m.sum()} hover samples (4 ms each)"
    return float(c["batt_i"][m].mean()), float(c["batt_v"][m].mean()), int(m.sum())


@pytest.mark.skipif(SKIP is not None, reason=SKIP or "")
def test_arm_loads_drone(tmp_path):
    sys.path.insert(0, str(SC))
    import showcase_scene as ss

    mass = ss.payload_density() * ss.CUBE**3
    assert 0.050 <= mass <= 0.137, (
        f"payload {mass * 1e3:.0f} g: outside the arm's 50..137 g (overload limit at this reach)"
    )
    runs = {}
    for name, scen in (("loaded", "arm_loads_drone"), ("unloaded", "fly_unloaded")):
        res, d = program.run(SC / f"{scen}.scenario.toml", tmp_path / name)
        try:
            bad = [m["name"] for m in res["monitors"] if not m["pass"]]
            assert res["pass"], f"{name}: {res['fail_reasons']} {bad}"
            assert res["replay"]["available"] and res["replay"]["match"], (
                f"{name}: replay hash {res['replay']}"
            )
            assert [s["returncode"] for s in res["programs"]] == [0] * len(
                res["programs"]
            )
            assert res["sim"]["port_errors"] == []
            runs[name] = (res, d, hover_current(d))
        finally:  # ~90 MB per run (replay log, ulog): keep only with NAPL_KEEP_RUNS=1
            if not os.environ.get("NAPL_KEEP_RUNS"):
                shutil.rmtree(d, ignore_errors=True)
    (res, d, (i_l, v_l, n_l)), (_, _, (i_u, v_u, n_u)) = (
        runs["loaded"],
        runs["unloaded"],
    )
    assert [s["name"] for s in res["programs"]] == ["arm", "fly"]
    assert (
        res["programs"][0]["end_t"] <= res["programs"][1]["start_t"] + 1e-9
    )  # the arm program ran first, to completion
    print(f"\nhover current: loaded {i_l:.2f} A ({n_l} samples) vs unloaded {i_u:.2f} A ({n_u}): {100 * (i_l / i_u - 1):+.1f} %; "
          f"payload {mass * 1e3:.1f} g; loaded replay hash {res['replay']['state_hash']} tick {res['replay']['tick']}")  # fmt: skip
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "showcase_result.json").write_text(json.dumps(dict(
        payload_kg=mass, hover_current_loaded_a=i_l, hover_current_unloaded_a=i_u, hover_batt_v=[v_l, v_u],
        monitors={m["name"]: {k: m.get(k) for k in ("pass", "min", "max", "final")} for m in res["monitors"]},
        programs=res["programs"], replay=res["replay"], run_dir=str(d)), indent=1, default=str))  # fmt: skip
    assert i_l > i_u, "the loaded hover must draw more current than the unloaded one"
