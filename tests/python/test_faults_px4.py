"""F3 fault injection against real PX4 SITL (marker px4): examples/faults/x500_*.scenario.toml, each flies flight.py (arm, takeoff 5 m,
HOLD) and injects one fault through the bus 3 s into the hold. What PX4 does is read from program.log (flight mode changes, PX4
status texts, battery volts) and trace.csv (ground truth); the numbers are printed and asserted below. Skipped unless the PX4 build,
the bridge exe, napl-replay and mavsdk are present. ~1 min of wall clock each (the stack runs lockstep at speed 1.0).
Run on hfbox: NAPL_PX4_INSTANCE=4 .venv/bin/python -m pytest tests/python/test_faults_px4.py -m px4 -v -s

Observed behaviour (PX4 v1.17 SITL, tests/data/px4/x500_fixture.xml, params px4/params/napl_sitl.params), seconds after the fault:
  gnss_outage 10 s  : hovers ~5 s on baro / IMU, then "Failsafe activated" + LAND (position loss); it lands in place, drift < 1 m.
  motor_out         : NO failsafe, NO mode change: the status text "Preflight Fail: Attitude failure (roll)" is all PX4 says; the quad
                      flips and hits the ground within ~2 s (PX4 SITL gets no ESC feedback, so it cannot know a motor is gone).
  battery +60 mOhm  : PX4's BATTERY_STATUS voltage drops by I x R (~0.65 V at ~10 A) within a second."""

import csv
import os
import re
import shutil

import numpy as np
import pytest
from napl import program, px4

pytestmark = pytest.mark.px4
FAULTS = px4.REPO / "examples" / "faults"
SKIP = px4.skip_reason()


def run(name, tmp_path):
    """-> (result, program.log text, trace dict, program-log events). Times of the events are World sim s (trace.csv's t)."""
    res, d = program.run(FAULTS / f"{name}.scenario.toml", tmp_path / name)
    try:
        text = (d / "program.log").read_text()
        with open(d / "trace.csv") as f:
            rows = list(csv.DictReader(f))
        tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        tr["tilt"] = np.degrees(
            np.arccos(np.clip(1 - 2 * (tr["qx"] ** 2 + tr["qy"] ** 2), -1, 1))
        )
        tr["alt"] = tr["z"] - tr["z"][0]
        tr["dist_home"] = np.hypot(tr["x"] - tr["x"][0], tr["y"] - tr["y"][0])
    finally:  # ~90 MB per run (replay log, ulog): keep only with NAPL_KEEP_RUNS=1
        if not os.environ.get("NAPL_KEEP_RUNS"):
            shutil.rmtree(d, ignore_errors=True)
    p0 = res["program"]["start_t"]
    ev = [
        (float(m[1]) + p0, m[2], m[3])
        for m in re.finditer(r"^\[\s*([\d.]+)s\] ([A-Z_]+) ?(.*)$", text, re.MULTILINE)
    ]
    assert res["replay"]["available"] and res["replay"]["match"], (
        f"{name}: replay hash {res['replay']}"
    )
    print(f"\n[{name}] fault {res['faults'][0]}")
    for m in res["monitors"]:
        print(
            f"[{name}]   monitor {m['name']}: pass={m['pass']} min {m.get('min', 0):.4g} max {m.get('max', 0):.4g} final {m.get('final', 0):.4g}"
        )
    return res, tr, ev


def after(ev, kind, pred, t0):
    """Times of the log events of `kind` after t0 whose payload satisfies pred."""
    return [t for t, k, s in ev if k == kind and t > t0 and pred(s)]


def at(tr, key, t):
    return float(np.interp(t, tr["t"], tr[key]))


@pytest.mark.skipif(SKIP is not None, reason=SKIP or "")
def test_x500_gnss_outage_in_position_hold(tmp_path):
    res, tr, ev = run("x500_gnss_outage", tmp_path)
    f = res["faults"][0]
    t0, t1 = f["start_t"], f["end_t"]
    assert t1 - t0 == pytest.approx(10.0, abs=0.1)
    fs = after(ev, "STATUS", lambda s: "Failsafe activated" in s, t0)
    land = after(ev, "MODE", lambda s: s == "LAND", t0)
    down = after(ev, "GPS_OK", lambda s: s == "False", t0)
    up = after(ev, "GPS_OK", lambda s: s == "True", t1)
    touch = tr["t"][(tr["t"] > t0) & (tr["alt"] < 0.3)]
    drift = tr["dist_home"][(tr["t"] >= t0) & (tr["t"] <= touch[0])].max()
    print(f"[gnss] failsafe +{fs[0] - t0:.1f} s, LAND +{land[0] - t0:.1f} s, GPS health false +{down[0] - t0:.1f} s, "
          f"true again +{up[0] - t1:.1f} s after the fault ended; alt at +4 s {at(tr, 'alt', t0 + 4):.2f} m, "
          f"on the ground +{touch[0] - t0:.1f} s; max horizontal drift {drift:.2f} m")  # fmt: skip
    assert fs and 3.0 < fs[0] - t0 < 8.0, (
        "PX4 should declare position loss a few seconds into the outage"
    )
    assert land and abs(land[0] - fs[0]) < 2.0, "the failsafe action is LAND"
    assert down and up, (
        "the position-health flag goes false in the outage and true again after it"
    )
    assert at(tr, "alt", t0 + 4.0) > 4.5, "until the failsafe it keeps hovering"
    assert touch.size and touch[0] - t0 < 15.0, (
        "it lands during / right after the outage"
    )
    assert drift < 2.0, "no GNSS but attitude and baro: no flyaway"
    assert res["pass"], res["fail_reasons"]


@pytest.mark.skipif(SKIP is not None, reason=SKIP or "")
def test_x500_motor_out_in_hover(tmp_path):
    res, tr, ev = run("x500_motor_out", tmp_path)
    t0 = res["faults"][0]["start_t"]
    w = (tr["t"] >= t0) & (tr["t"] <= t0 + 10.0)
    crash = tr["t"][w & (tr["alt"] < 0.5)]
    tilt = tr["tilt"][w].max()
    modes = after(ev, "MODE", lambda s: True, t0)
    att = after(ev, "STATUS", lambda s: "Attitude failure" in s, t0)
    print(f"[motor_out] below 0.5 m +{crash[0] - t0:.1f} s, max tilt {tilt:.0f} deg, min alt {tr['alt'][w].min():.2f} m; "
          f"PX4: attitude-failure text +{(att or [float('nan')])[0] - t0:.1f} s; mode changes in the first 10 s: "
          f"{[(round(t - t0, 1), s) for t, k, s in ev if k == 'MODE' and t0 < t < t0 + 10]}")  # fmt: skip
    assert crash.size and crash[0] - t0 < 10.0, (
        "monitors detect the crash: below 0.5 m within 10 s"
    )
    assert tilt > 90.0, "a quad with one motor out tumbles"
    assert att and att[0] - t0 < 5.0, (
        "the only thing PX4 reports is the attitude-failure status text"
    )
    assert not [t for t in modes if t < t0 + 10.0], (
        "PX4 does not change mode: no failsafe for a lost motor in this SITL set-up"
    )
    assert res["pass"], res["fail_reasons"]


@pytest.mark.skipif(SKIP is not None, reason=SKIP or "")
def test_x500_battery_cell_fault_sag_reaches_px4(tmp_path):
    res, tr, ev = run("x500_battery_cell", tmp_path)
    t0 = res["faults"][0]["start_t"]
    volts = [
        (t, float(s)) for t, k, s in ev if k == "BATT"
    ]  # what PX4 reports (MAVSDK telemetry battery)
    before = np.mean([v for t, v in volts if t0 - 3 < t < t0 - 0.5])
    later = np.mean([v for t, v in volts if t0 + 2 < t < t0 + 10])
    amps = tr["batt_i"][(tr["t"] > t0 + 2) & (tr["t"] < t0 + 10)].mean()
    print(f"[battery] PX4 battery voltage {before:.2f} V -> {later:.2f} V ({before - later:.2f} V sag); hover current {amps:.1f} A; "
          f"I x 60 mOhm = {amps * 0.06:.2f} V")  # fmt: skip
    assert before - later > 0.4, "the sag is visible in PX4's battery voltage"
    assert before - later == pytest.approx(amps * 0.06, rel=0.3), (
        "and it is I x the extra cell resistance"
    )
    assert res["pass"], res["fail_reasons"]
