"""Qdd + FootContact (SP2, docs/models.md "Qdd", "FootContact") through the Python API, and the design side: a [[devices]] type = "qdd" /
"foot_contact" binding built from a synthetic actuator part, with the part's params converted by the numbers rule. Needs the built kernel
(hfbox: tools/remote.sh run 'pytest tests/python/test_qdd.py')."""

import json
import math
import xml.etree.ElementTree as ET

import napl
import numpy as np
import pytest
from napl import design
from napl.design import DesignError
from test_design_core import P, blk, prm, prov, write_part

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
KE = 60 / (2 * math.pi * 60)  # AK70-9 numbers: kv 60 rpm/V
N, ETA, R20 = 9, 0.9, 0.475

QDD_JSON = """{
 "psus": [{"name": "bench", "node": 1, "voltage": %(v)s, "r_series": 0.01}],
 "qdds": [{"name": "q1", "node": 1, "joint": "out",
  "motor": {"kv": 60, "r20": 0.475, "i0": 0.05, "rotor_inertia": 8.66307e-5, "w_s": 5, "c_th": 150, "r_th": 1.5},
  "gear_ratio": 9, "efficiency": 0.9, "encoder_cpr": 16384, "cmd_hz": 1000, "latency_s": 0.0, "i_limit": 20,
  "kp_max": 500, "kd_max": 10, "tau_ff_max": 100, "timeout_s": 0.05, "timeout_mode": 0, "kd_timeout": 2,
  "t_max": 100, "derate_start": 60}]}"""


def bench(tmp_path, volts=24):
    p = tmp_path / "qdd.xml"
    p.write_text(
        '<mujoco><option timestep="0.001" gravity="0 0 0"/><worldbody>'
        '<body name="link" pos="0 0 0.1"><joint name="out" type="hinge" axis="0 0 1"/>'
        '<geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.01" mass="0.05" contype="0" conaffinity="0"/></body></worldbody>'
        f"<custom><text name=\"napl.devices\" data='{QDD_JSON % {'v': volts}}'/></custom></mujoco>"
    )
    return p


def val(s, attr, dev="q1"):
    return float(s.query(f"/devices/{dev}", attr)[0][0])


@needs_kernel
def test_stall_torque_and_units(tmp_path):
    s = napl.Sim.load(bench(tmp_path))
    s.set("/devices/q1", "tau_ff", 100.0)
    s.set("/devices/q1", "enable", 1.0)
    s.step(1)
    r = R20 * (1 + 0.00393 * 5)
    i = min(20.0, val(s, "voltage") / r)
    assert val(s, "i_q") == pytest.approx(i, rel=1e-9)
    assert val(s, "tau_est") == pytest.approx(i * KE * N * ETA, rel=0.01)
    assert s.query("/devices/q1", "tau_est")[1] == "N*m"
    assert s.query("/devices/q1", "voltage")[1] == "V"
    with pytest.raises(Exception):
        s.set("/devices/q1", "tau_est", 1.0)  # read-only
    with pytest.raises(Exception):
        s.set("/devices/q1", "enable", 0.5)  # binary


@needs_kernel
def test_speed_limits_the_torque_and_command_loss_damps(tmp_path):
    s = napl.Sim.load(bench(tmp_path))
    s.set("/devices/q1", "tau_ff", 100.0)
    s.set("/devices/q1", "enable", 1.0)
    w0 = 24 / (KE * N)
    taus = []
    for f in (0.0, 0.5, 0.9, 1.0):
        s.set("/world/link/out", "qvel", [f * w0])
        s.step(1)
        taus.append(val(s, "tau_est"))
    assert taus[0] > taus[1] > taus[2] > 0 and abs(taus[3]) < 0.02 * taus[0]
    s2 = napl.Sim.load(bench(tmp_path))
    s2.set("/devices/q1", "enable", 1.0)
    s2.set("/world/link/out", "qvel", [5.0])
    s2.step(80)  # past the 50 ms CAN timeout with no new frame
    assert val(s2, "timeout") == 1.0
    assert val(s2, "tau_cmd") == pytest.approx(-2.0 * val(s2, "dq"), abs=1e-12)


# ---- design side ---------------------------------------------------------------------------------------------------------
def qdd_parts(tmp_path, kv_unit="rpm/V"):
    parts = tmp_path / "parts"
    blk(
        parts,
        "base",
        [P("top", "seat.a", pos=(0, 0, 0.05))],
        mass=1.0,
        half=(0.1, 0.1, 0.05),
    )
    prm_ = lambda v, u: prm(v, u, "datasheet")
    write_part(
        parts,
        "qdd-test",
        "actuator.bldc.qdd",
        {
            "mass": prm_(0.5, "kg"),
            "kv": prm_(60.0, kv_unit),
            "r_phase_to_phase": prm_(0.475, "ohm"),
            "rotor_inertia": prm_(8.66307e-5, "kg*m^2"),
            "i0": prm_(0.05, "A"),
            "c_th": prov(150.0, "J/K"),
            "r_th": prov(1.5, "K/W"),
            "gear_ratio": prm_(9.0, "1"),
            "gear_efficiency": prov(0.9, "1"),
            "encoder_cpr": prm_(16384, "1"),
            "can_cmd_hz": prm_(1000.0, "Hz"),
            "can_latency": prov(0.001, "s"),
            "i_limit_phase": prov(20.0, "A"),
            "mit_kp_max": prm_(500.0, "N*m/rad"),
            "mit_kd_max": prm_(10.0, "N*m*s/rad"),
            "mit_tau_ff_max": prm_(100.0, "N*m"),
            "can_timeout": prov(0.05, "s"),
            "kd_timeout": prov(2.0, "N*m*s/rad"),
            "temp_fault": prov(100.0, "degC"),
            "temp_derate_start": prov(60.0, "degC"),
        },
        [
            P("mount", "seat.a", axis=(0, 0, -1)),
            P("out", "seat.b", axis=(0, 0, 1), pos=(0, 0, 0.05)),
            P("pwr", "xt30", "male", kind="power", role="sink"),
        ],
        {"shape": "box", "size": [0.04, 0.04, 0.025]},
    )
    write_part(
        parts,
        "ball",
        "misc",
        {"mass": prm(0.2)},
        [P("bot", "seat.b", axis=(0, 0, -1))],
        {"shape": "sphere", "size": [0.03]},
    )
    return parts


ASM = """
[assembly]
name = "qdd_bench"
base = "fixed"
timestep = 0.001
[[instances]]
id = "base"
part = "base"
root = true
group = "frame"
[[instances]]
id = "act"
part = "qdd-test"
group = "frame"
[[instances]]
id = "foot"
part = "ball"
group = "frame"
[[mates]]
a = "base"
b = "act"
[[mates]]
name = "knee"
a = "act.out"
b = "foot"
joint = { type = "hinge" }
[device_maps.q]
"motor.kv" = "kv"
"motor.r20" = "r_phase_to_phase"
"motor.rotor_inertia" = "rotor_inertia"
"motor.i0" = "i0"
"motor.c_th" = "c_th"
"motor.r_th" = "r_th"
gear_ratio = "gear_ratio"
efficiency = "gear_efficiency"
encoder_cpr = "encoder_cpr"
cmd_hz = "can_cmd_hz"
latency_s = "can_latency"
i_limit = "i_limit_phase"
kp_max = "mit_kp_max"
kd_max = "mit_kd_max"
tau_ff_max = "mit_tau_ff_max"
timeout_s = "can_timeout"
kd_timeout = "kd_timeout"
t_max = "temp_fault"
derate_start = "temp_derate_start"
[[devices]]
type = "psu"
name = "bench"
instance = "act"
port = "pwr"
set = { voltage = { value = 24.0, unit = "V", fidelity = "estimated", rationale = "t" }, r_series = { value = 0.01, unit = "ohm", fidelity = "estimated", rationale = "t" } }
[[devices]]
type = "qdd"
name = "act"
instance = "act"
port = "pwr"
joint = "knee"
map = "q"
timeout_mode = 0
set = { "motor.w_s" = { value = 5.0, unit = "rad/s", fidelity = "estimated", rationale = "t" } }
[[devices]]
type = "foot_contact"
name = "foot"
instance = "foot"
mode = 0
set = { f_max = { value = 300.0, unit = "N", fidelity = "estimated", rationale = "t" }, hysteresis_on = { value = 2.0, unit = "N", fidelity = "estimated", rationale = "t" }, hysteresis_off = { value = 1.0, unit = "N", fidelity = "estimated", rationale = "t" }, debounce_s = { value = 0.005, unit = "s", fidelity = "estimated", rationale = "t" }, rate_hz = { value = 1000.0, unit = "Hz", fidelity = "estimated", rationale = "t" } }
"""


def build(tmp_path, text=ASM, **kw):
    parts = qdd_parts(tmp_path, **kw)
    (tmp_path / "a.toml").write_text(text)
    return design.build(tmp_path / "a.toml", parts, tmp_path / "out")


def devices_of(r):
    root = ET.parse(r["xml"]).getroot()
    return json.loads(
        next(t for t in root.iter("text") if t.get("name") == "napl.devices").get(
            "data"
        )
    )


def test_builder_emits_qdd_and_foot_contact(tmp_path):
    d = devices_of(build(tmp_path))
    q = d["qdds"][0]
    assert q["name"] == "act" and q["joint"] and q["node"] == d["psus"][0]["node"]
    assert (
        q["gear_ratio"] == 9.0 and q["efficiency"] == 0.9 and q["encoder_cpr"] == 16384
    )  # int field
    assert isinstance(q["encoder_cpr"], int) and q["timeout_mode"] == 0
    assert (
        q["motor"]["r20"] == 0.475
        and q["motor"]["kv"] == 60.0
        and q["motor"]["w_s"] == 5.0
    )
    assert q["parts"] == ["qdd-test"]
    f = d["foot_contacts"][0]
    assert (
        f["geom"] == "foot"
        and f["f_max"] == 300.0
        and f["mode"] == 0
        and f["parts"] == ["ball"]
    )


def test_builder_checks_units(tmp_path):
    with pytest.raises(DesignError, match="rpm/V"):
        build(tmp_path, kv_unit="rad/s/V")


@needs_kernel
def test_built_assembly_loads_and_the_actuator_drives_its_joint(tmp_path):
    r = build(tmp_path)
    s = napl.Sim.load(r["xml"])
    s.set("/devices/act", "enable", 1.0)
    s.set("/devices/act", "kp", 10.0)
    s.set("/devices/act", "kd", 0.5)
    s.set("/devices/act", "p_des", 0.2)
    s.step(1200)
    q = float(s.query("/devices/act", "q")[0][0])
    assert q == pytest.approx(0.2, abs=0.02)
    assert np.isfinite(float(s.query("/devices/foot", "force")[0][0]))
