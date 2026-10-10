"""The Skydio-style autonomy as a companion computer to real PX4 SITL (marker px4): examples/skydio/goal.px4.scenario.toml cut to a 14 s flight
(SKYDIO_T_MAX). GPS is disabled in the bridge config and in EKF2; the only position source PX4 has is the ODOMETRY the companion sends. Checks what
that path must deliver, not the mission (the full scenarios and the Monte Carlo are examples/skydio/px4_montecarlo.py): PX4 reached a valid local
position from vision alone, armed, flew offboard take-off and the first metres of the planned trajectory with no contact, and its own estimate stays close
to the companion's. Skipped unless the PX4 build, the bridge exe, napl-replay and mavsdk are present. ~3 min of wall clock (lockstep at speed 1.0).
Run on hfbox: NAPL_PX4_INSTANCE=5 .venv/bin/python -m pytest tests/python/test_px4_skydio.py -m px4 -v -s"""

import csv
import json
import os
import shutil
import tomllib

import numpy as np
import pytest
from napl import program, px4

pytestmark = pytest.mark.px4
SKIP = px4.skip_reason()


@pytest.mark.skipif(SKIP is not None, reason=SKIP or "")
def test_px4_flies_the_skydio_airframe_on_vision_odometry_alone(tmp_path, monkeypatch):
    monkeypatch.setenv("SKYDIO_T_MAX", "14")
    res, d = program.run(
        px4.REPO / "examples" / "skydio" / "goal.px4.scenario.toml", tmp_path / "run"
    )
    try:
        log = (d / "program.log").read_text()
        assert res["program"]["returncode"] == 0 and not res["program"]["timeout"], log[
            -3000:
        ]
        assert res["replay"]["available"] and res["replay"]["match"], res["replay"]
        s = json.loads((d / "summary.json").read_text())
        with open(d / "companion.csv") as f:
            rows = list(csv.DictReader(f))
        z = np.array([float(r["z"]) for r in rows])
        assert z.max() - z[0] > 1.5, (
            f"never climbed: z {z[0]:.2f} .. {z.max():.2f}"
        )  # offboard take-off to the 1.9 m cruise height happened
        assert s["contact_steps"] == 0 and s["min_clearance_m"] > 0.05
        e = s[
            "px4_vs_companion"
        ]  # PX4's EKF2 against the ESKF it is fed: it follows it (EKF2_EV_NOISE_MD 1, 0.1 m position noise)
        assert e["samples"] >= 10 and e["pos_rms_m"] < 0.6 and e["vel_rms_mps"] < 0.6, e
        assert "gnss" not in tomllib.loads(
            (px4.REPO / "px4" / "bridge_skydio.toml").read_text()
        )  # GPS-denied: the bridge sends no HIL_GPS
    finally:
        if not os.environ.get("NAPL_KEEP_RUNS"):
            shutil.rmtree(d, ignore_errors=True)
