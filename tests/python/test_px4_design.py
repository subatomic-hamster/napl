"""E3: a design built / edited through the editing API is saved and flown with `napl program` against real PX4 SITL, no hand-written
bridge config (`[design] path = ...`, napl.design.px4cfg). Also the pre-flight check: a PWM order / prop direction that disagrees with
the PX4 quad-X airframe is refused with its fix before PX4 starts (non-px4 tests, need no PX4).

px4 tests (marker px4; NAPL_PX4_INSTANCE picks the ports): the scratch-built quad (test_design_scratch.build_scratch) takes off to 5 m,
holds 20 s, lands; the X500 with its four motors swapped for the KDE 2315XF-965 (swap(auto=True)) flies the same mission as the stock
X500 and the hover current / throttle / PX4's hover-thrust estimate are compared.
Run: .venv/bin/python -m pytest tests/python/test_px4_design.py -m px4 -v -s
"""

import csv
import json
import os
import re
import shutil
import subprocess
import threading
import time
import tomllib
from pathlib import Path

import numpy as np
import pytest
from napl import program, px4
from napl.design import Design, DesignError, build
from napl.design.px4cfg import bridge_config, check_px4
from test_design_scratch import ARM, FILL, build_scratch

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
X500 = REPO / "assemblies" / "x500v2.toml"
HOVER = REPO / "examples" / "x500" / "hover.py"
N = int(os.environ.get("NAPL_PX4_INSTANCE", "0"))
SKIP = px4.skip_reason()
needs_px4 = pytest.mark.skipif(SKIP is not None, reason=SKIP or "")

MONITORS = """
[[monitors]]
name = "max_altitude"
column = "alt"
op = "always"
hi = 6.5
[[monitors]]
name = "reached_5m"
column = "alt"
op = "eventually"
lo = 4.5
[[monitors]]
name = "stays_upright"
column = "tilt"
op = "always"
hi = 30.0
[[monitors]]
name = "stays_near_home"
column = "dist_home"
op = "always"
hi = 6.0
[[monitors]]
name = "landed_on_ground"
column = "alt"
op = "final"
lo = -0.2
hi = 0.2
"""


def scenario(tmp, design, parts=None, name="t"):
    f = tmp / f"{name}.scenario.toml"
    f.write_text(
        f'name = "{name}"\n[design]\npath = "{design}"\n'
        + (f'parts = "{parts}"\n' if parts else "")
        + f'[interfaces.px4]\n[program]\ncmd = ["python", "{HOVER}"]\n[limits]\nwall_s = 240\ncpu_s = 400\n{MONITORS}'
    )
    return f


def swap_pwm(d, a, b):
    """Exchange the PWM outputs fc.pwm<a> and fc.pwm<b> between their ESCs (two leads plugged in the wrong order)."""
    ends = {}
    for w in d.doc["wires"]:
        for x, y in ((w["from"], w["to"]), (w["to"], w["from"])):
            if x in (f"fc.pwm{a}", f"fc.pwm{b}"):
                ends[x] = (w["name"], y)
    for name, _ in ends.values():
        d.unwire(name)
    for out, other in ((a, b), (b, a)):
        esc = ends[f"fc.pwm{other}"][1]
        d.autowire(f"fc.pwm{out}", esc, included_in=esc.split(".")[0])


@pytest.fixture(scope="module")
def good(tmp_path_factory):
    """The scratch quad, saved to a TOML (what `napl program` reads), and its build."""
    d, _ = build_scratch()
    tmp = tmp_path_factory.mktemp("good")
    return d, d.save(tmp / "scratch-quad.toml"), tmp


# ---- the pre-flight check (no PX4) -------------------------------------------------------------------------------------------
def test_generated_bridge_config_matches_the_hand_written_one(good):
    d, path, tmp = good
    r = d.build(tmp / "b")
    assert check_px4(d.asm, r["xml"]) == []
    cfg = tomllib.loads(
        bridge_config(d.asm, r["xml"], tmp / "bridge.toml", PARTS).read_text()
    )
    ref = tomllib.loads((REPO / "px4" / "bridge_x500v2_asm.toml").read_text())
    ref.pop("model")
    assert {
        k: v for k, v in cfg.items() if k != "model"
    } == ref  # same esc order / sensors / home / battery as the typed file


def test_stock_x500_passes_the_check(tmp_path):
    r = build(X500, PARTS, tmp_path)
    assert check_px4(X500, r["xml"]) == []


@pytest.mark.parametrize(
    "a, b, codes",
    [
        (1, 2, {"PX4_MOTOR_ORDER"}),
        (1, 3, {"PX4_MOTOR_ORDER"}),
        (2, 4, {"PX4_MOTOR_ORDER"}),
    ],
)
def test_swapped_pwm_leads_are_refused_with_the_fix(a, b, codes, tmp_path):
    d, _ = build_scratch()
    swap_pwm(d, a, b)
    path = d.save(tmp_path / "swapped.toml")
    xml = d.build(tmp_path / "b")["xml"]
    fs = check_px4(d.asm, xml)
    assert (
        {f.code for f in fs} == codes
        and len(fs) == 2
        and all(f.severity == "error" for f in fs)
    )
    # the fix names the ESC the lead belongs on: pwm<a> goes back on ESC <a>'s instance (esc<a> is motor <a>)
    assert f"fc.pwm{a} -> esc{a}" in " ".join(f.fix for f in fs)
    with pytest.raises(
        DesignError, match=r"PX4_MOTOR_ORDER.*\n\s+fix: wire fc\.pwm\d -> esc\d\.signal"
    ):
        bridge_config(d.asm, xml, tmp_path / "bridge.toml")
    assert not (tmp_path / "bridge.toml").exists()  # nothing written
    # through `napl program`: refused before PX4 starts (so this runs without PX4)
    with pytest.raises(program.ScenarioError, match="PX4_MOTOR_ORDER") as e:
        program.run(scenario(tmp_path, path), tmp_path / "runs")
    print(
        f"\n[refused] {str(e.value).splitlines()[1]}\n[refused] {str(e.value).splitlines()[2]}"
    )


def test_reversed_prop_direction_is_refused(tmp_path, monkeypatch):
    import test_design_scratch as S

    monkeypatch.setitem(
        S.REVERSE, 1, True
    )  # motor 1 (front-right, CCW in PX4) built with a CW prop: reversed axis + dir -1
    d, _ = build_scratch()
    xml = d.build(tmp_path / "b")["xml"]
    fs = check_px4(d.asm, xml)
    assert [f.code for f in fs] == ["PX4_PROP_DIRECTION"], [
        (f.code, f.message) for f in fs
    ]
    assert (
        "motor 1" in fs[0].message and "CCW" in fs[0].message and "reverse" in fs[0].fix
    )
    print(f"\n[refused] {fs[0].message}\n[refused] fix: {fs[0].fix}")


def test_missing_pwm_wire_is_refused(tmp_path):
    d, _ = build_scratch()
    w = next(w for w in d.doc["wires"] if "fc.pwm3" in (w["from"], w["to"]))
    d.unwire(w["name"])
    fs = check_px4(d.asm, d.build(tmp_path)["xml"])
    assert [f.code for f in fs] == ["PX4_PWM_UNWIRED"] and "esc" in fs[0].fix


def test_design_scenario_rules(tmp_path):
    f = scenario(tmp_path, X500)
    f.write_text(
        f.read_text().replace("[interfaces.px4]", '[interfaces.px4]\nbridge = "x.toml"')
    )
    with pytest.raises(program.ScenarioError, match="without `bridge`"):
        program.load_scenario(f)
    f.write_text(f.read_text().replace('bridge = "x.toml"', ""))
    assert program.load_scenario(f)["model"]["assembly"] == str(X500)


# ---- real PX4 flights ----------------------------------------------------------------------------------------------------------
class HoverThrust(threading.Thread):
    """Polls PX4's hover_thrust_estimate topic (px4-listener, the module's own client) during the run -> [(wall s, hover_thrust, valid)]."""

    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.stop_ = [], threading.Event()

    def run(self):
        exe = px4.BUILD / "bin" / "px4-listener"
        while not self.stop_.wait(2.0):
            try:
                o = subprocess.run(
                    [str(exe), "--instance", str(N), "hover_thrust_estimate"],
                    capture_output=True,
                    text=True,
                    timeout=10,
                ).stdout
            except (OSError, subprocess.TimeoutExpired):
                continue
            h, v = (
                re.search(r"hover_thrust: ([-\d.e]+)", o),
                re.search(r"valid: (\w+)", o),
            )
            if h:
                self.rows.append((time.time(), float(h[1]), v and v[1] == "True"))


def fly(tmp, path, name, parts=None):
    """`napl program` on a saved design -> (result, numbers of the hold window)."""
    hte = HoverThrust()
    hte.start()
    try:
        res, run = program.run(scenario(tmp, path, parts, name), tmp / "runs")
    finally:
        hte.stop_.set()
    log = (run / "program.log").read_text()
    with open(run / "trace.csv") as f:
        rows = list(csv.DictReader(f))
    tr = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    alt = tr["z"] - tr["z"][0]
    hold = alt > 4.5
    thr = float(np.mean([tr[f"throttle{k}"][hold].mean() for k in range(4)]))
    est = [h for _, h, v in hte.rows if v]
    num = dict(
        hover_throttle=thr,
        per_motor=[float(tr[f"throttle{k}"][hold].mean()) for k in range(4)],
        hover_current_A=float(tr["batt_i"][hold].mean()),
        battery_v=float(tr["batt_v"][hold].mean()),
        hte_last_valid=est[-1] if est else None,
        hte_n=len(hte.rows),
        alt_max=float(alt.max()),
        final_alt=float(alt[-1]),
        sim_s=float(tr["t"][-1]),
    )
    shutil.rmtree(run / "model", ignore_errors=True)
    print(
        f"\n[{name}] pass={res['pass']} {res['fail_reasons']} replay={res['replay']} sim {num['sim_s']:.0f} s\n[{name}] {json.dumps(num)}\n{log[-400:]}"
    )
    shutil.rmtree(run, ignore_errors=True)  # hfbox /workspace has ~1 GB
    return res, num


def check_flight(res, num):
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"], res["replay"]
    assert 4.5 < num["alt_max"] <= 6.5 and abs(num["final_alt"]) < 0.2


@pytest.mark.px4
@needs_px4
def test_scratch_built_quad_flies_takeoff_hold_land(good):
    d, path, tmp = good
    res, num = fly(tmp, path, "scratch-quad")
    check_flight(res, num)
    assert num["hover_current_A"] > 5


@pytest.mark.px4
@needs_px4
def test_x500_with_kde_motors_vs_stock(tmp_path):
    # D5: the unpatched library: the KDE seats on the connector's 16x19 port + M6 adapter, `rewire` re-sizes the phase leads
    # (the KDE datasheet has no cable_length)
    d = Design.open(X500, PARTS)
    r = d.swap(
        [f"mot{k}" for k in ARM],
        "kde-2315xf-965",
        auto=True,
        overwrite=True,
        fill=FILL,
        rewire=True,
    )
    assert r.changes[-1]["auto_devices"]["summary"]["unset"] == 0, r.changes[-1][
        "auto_devices"
    ]
    kde_path = d.save(tmp_path / "x500-kde.toml")
    assert [f.code for f in check_px4(d.asm, d.build(tmp_path / "b")["xml"])] == []
    out = {}
    for name, path in (("x500-stock", X500), ("x500-kde", kde_path)):
        out[name] = fly(tmp_path, path, name)
        check_flight(*out[name])
    s, k = out["x500-stock"][1], out["x500-kde"][1]
    print(
        "\n[compare] stock X500 -> KDE 2315XF-965: "
        f"hover current {s['hover_current_A']:.2f} -> {k['hover_current_A']:.2f} A, "
        f"hover throttle (PX4 output) {s['hover_throttle']:.3f} -> {k['hover_throttle']:.3f}, "
        f"PX4 hover-thrust estimate {s['hte_last_valid']} -> {k['hte_last_valid']} (MPC_THR_HOVER default 0.5)"
    )
    assert s["hover_current_A"] != k["hover_current_A"]
