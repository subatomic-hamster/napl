"""STS3215 (C001) servo bench: the servo model against the Feetech datasheet points, plus the SO-101 holding sag.

    python -m napl.servo_bench

Fixture: ONE servo whose config is the one the SO-101 assembly builds (assemblies/so101.toml, device map `sts3215`: kv, r20,
i0, rotor inertia, N, eta, cpr, registers, ... all produced by napl.design.build, nothing hand-copied here), on a regulated
PSU at V volts with R_SERIES in series (a bench supply; 1 mohm, so the motor sees V to 0.1 % at the 2.5 A stall). Gravity
off, output joint driving a 50 g link, no backlash joint. Two variants:
  stalled: the output joint has a +-0.05 rad range limit, the goal is 1 rad (saturated duty, max_torque 1000 = full duty).
           Torque = the device `torque` attr (output torque after gear efficiency); the C++ test
           servo_stall_torque_vs_constraint_force checks that attr against MuJoCo's constraint force, so it is read here
           rather than re-measured through the constraint.
  free:    no limit, no load but the servo's own friction / armature, goal 100 rad (saturated), steady speed + current.
Everything is one fixed-step run at the model timestep (1 ms), nothing fitted here.

Datasheet comparisons carry a `kind`: "circular" = the part's derived param was fitted to that very point (eta to the stall
torques, r_effective to the stall V/I, Ke to the no-load speed, i0 IS the 7.4 V no-load current), so agreement only shows
the fixture reproduces the derivation; "independent" = not an input of the model (the 6 V no-load current, whose model value
is the constant i0, and the step time, which depends on rotor inertia, an estimate).
12 V is a different part (STS3215 C018/C047, 30 kg.cm, 12 V coil) and is not benched: C001 is a 4-7.4 V servo.
"""

import functools
import json
import math
import re
import tempfile
from pathlib import Path

import napl
from napl import design

ROOT = Path(__file__).resolve().parents[2]
ASM = ROOT / "assemblies" / "so101.toml"
PART_ID = "feetech-sts3215-c001"
R_SERIES = 0.001  # ohm, PSU series resistance of the regulated bench supply
TOL = 10.0  # % tolerance of the datasheet comparison
JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
SERVOS = [f"servo{i}" for i in range(1, 7)]
STEP_DEG = 60.0
STEP_KIND = "not comparable: datasheet figure is the no-load speed restated"


@functools.cache
def _built():
    """Build assemblies/so101.toml once into a temp dir (kept for the process) -> (xml path, devices dict)."""
    tmp = tempfile.TemporaryDirectory(prefix="servo_bench_")
    # compute() = build() without the report / DRC step (the bench needs only the MJCF); mesh paths are relative to tmp
    xml_text, _ = design.compute(ASM, ROOT / "parts", tmp.name)
    xml = Path(tmp.name) / "so101.xml"
    xml.write_text(xml_text, encoding="utf-8")
    m = re.search(r"name=\"napl\.devices\" data='(.*?)'/>", xml.read_text(), re.DOTALL)
    return tmp, xml, json.loads(m.group(1))


def servo_config(**over) -> dict:
    """The so101 servo1 device config (from the part via the builder), minus node / joint binding, with overrides (p, d, ...)."""
    cfg = {
        k: v
        for k, v in _built()[2]["servos"][0].items()
        if k not in ("name", "node", "joint", "parts")
    }
    cfg.update(over)
    return cfg


def fixture(voltage: float, stalled: bool, **over) -> str:
    joint = '<joint name="out" type="hinge" axis="0 0 1"' + (
        ' limited="true" range="-0.05 0.05"/>' if stalled else "/>"
    )
    dev = {
        "psus": [
            {"name": "bench", "node": 1, "voltage": voltage, "r_series": R_SERIES}
        ],
        "servos": [{"name": "s1", "node": 1, "joint": "out", **servo_config(**over)}],
    }
    return f"""<mujoco model="servo_bench"><compiler angle="radian"/><option timestep="0.001" gravity="0 0 0"/>
<worldbody><body name="link" pos="0 0 0.1">{joint}
<geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.01" mass="0.05" contype="0" conaffinity="0"/></body></worldbody>
<custom><text name="napl.devices" data='{json.dumps(dev)}'/></custom></mujoco>"""


def _sim(voltage, stalled, **over):
    tmp = Path(tempfile.mkdtemp(prefix="servo_fx_"))
    (tmp / "fx.xml").write_text(fixture(voltage, stalled, **over))
    sim = napl.Sim.load(tmp / "fx.xml")
    return sim, lambda a: float(sim.query("/devices/s1", a)[0][0])


def _hold(sim, q, attrs, n):
    """Mean of the attrs over the next n ticks."""
    acc = dict.fromkeys(attrs, 0.0)
    for _ in range(n):
        sim.step(1)
        for a in attrs:
            acc[a] += q(a) / n
    return acc


def stall(voltage: float) -> dict:
    """Stalled variant at full duty: output torque (N*m) and supply current (A, `bus_current` is the node current, `current` the motor's)."""
    sim, q = _sim(voltage, True)
    sim.set("/devices/s1", "goal_position", 1.0)
    sim.set("/devices/s1", "torque_enable", 1)
    sim.step(1500)
    return _hold(sim, q, ["torque", "current", "duty", "voltage"], 200)


def no_load(voltage: float) -> dict:
    """Free variant, saturated duty: steady output speed (rad/s) and motor current (A)."""
    sim, q = _sim(voltage, False)
    sim.set("/devices/s1", "goal_position", 100.0)
    sim.set("/devices/s1", "torque_enable", 1)
    sim.step(1500)
    return _hold(sim, q, ["velocity", "current", "duty"], 200)


def step_response(
    voltage: float, p: float, pid_scale: float | None = None, d: float = 32.0
) -> dict:
    """60 deg step from rest, no load (pid_scale None = the assembly's). t_cover = first time the output is within 2 counts of
    60 deg; rise = 10-90 %; overshoot in %; settle = last time outside +-2 % of the step; peak_speed = max output speed rad/s."""
    over = {} if pid_scale is None else {"pid_scale": pid_scale}
    sim, q = _sim(voltage, False, p=p, d=d, **over)
    goal, cpr = math.radians(STEP_DEG), servo_config()["encoder_cpr"]
    sim.set("/devices/s1", "goal_position", goal)
    sim.set("/devices/s1", "torque_enable", 1)
    t, pos, peak = [], [], 0.0
    for k in range(3000):
        sim.step(1)
        t.append((k + 1) * 1e-3), pos.append(q("position"))
        peak = max(peak, abs(q("velocity")))
    cross = lambda f: next((ti for ti, x in zip(t, pos) if x >= f * goal), math.nan)
    t_cover = next(
        (ti for ti, x in zip(t, pos) if x >= goal - 2 * 2 * math.pi / cpr), math.nan
    )
    out = [ti for ti, x in zip(t, pos) if abs(x - goal) > 0.02 * goal]
    return {
        "t_cover": t_cover,
        "rise": cross(0.9) - cross(0.1),
        "overshoot": max(0.0, (max(pos) - goal) / goal * 100),
        "settle": out[-1] if out else 0.0,
        "peak_speed": peak,
    }


def full_duty_step(voltage: float) -> dict:
    """The same step with the loop saturated for the whole traverse (P = 1e4, D = 0): a full-duty bang-bang step."""
    return step_response(voltage, p=1e4, d=0.0)


def spin_up_tau() -> float:
    """J / b of the servo model (s): J = rotor inertia reflected to the output (I_rotor N^2), b = the damping split coefficient
    of devices.cpp drive_motor, b = eta Ke^2 N^2 / R (driving side, k = eta; R at 20 C, the winding barely warms in a step)."""
    cfg = servo_config()
    m, n = cfg["motor"], cfg["gear_ratio"]
    ke = 60.0 / (2 * math.pi * m["kv"])
    return m["rotor_inertia"] * n * n / (cfg["efficiency"] * ke * ke * n * n / m["r20"])


def step_checks(voltage: float) -> dict:
    """Full-duty 60 deg step against the datasheet no-load speed: t, t_min (= 60 deg / w_ds, the 'datasheet time'), excess
    t - 60 deg / w_sim vs tau = J / b, and peak output speed / w_ds."""
    w_ds = (
        design.Library(ROOT / "parts")
        .get(PART_ID)
        .value(f"no_load_speed_{'6v' if voltage == 6.0 else '7v4'}")
    )
    w_sim = abs(no_load(voltage)["velocity"])
    sr = full_duty_step(voltage)
    return {
        "V": voltage,
        "t": sr["t_cover"],
        "t_ds": math.radians(STEP_DEG) / w_ds,
        "excess": sr["t_cover"] - math.radians(STEP_DEG) / w_sim,
        "tau": spin_up_tau(),
        "peak_ratio": sr["peak_speed"] / w_ds,
    }


def _row(quantity, v, sim, ref, kind):
    err = (sim - ref) / ref * 100
    return {
        "quantity": quantity,
        "V": v,
        "sim": sim,
        "datasheet": ref,
        "err_pct": err,
        "pass": abs(err) <= TOL,
        "kind": kind,
    }


def datasheet_rows() -> list:
    """[{quantity, V, sim, datasheet, err_pct, pass, kind}] at 6.0 and 7.4 V, ref = the part's datasheet params."""
    part = design.Library(ROOT / "parts").get(PART_ID)
    rows = []
    for v, s in ((6.0, "6v"), (7.4, "7v4")):
        st, nl = stall(v), no_load(v)
        rows += [
            _row(
                "stall torque N*m",
                v,
                abs(st["torque"]),
                part.value(f"stall_torque_{s}"),
                "circular (eta fitted to both stall torques)",
            ),
            _row(
                "stall current A",
                v,
                abs(st["current"]),
                part.value(f"stall_current_{s}"),
                "circular (r_effective = mean V/I_stall; the 6 V vs 7.4 V ratio is the check)",
            ),
            _row(
                "no-load speed rad/s",
                v,
                abs(nl["velocity"]),
                part.value(f"no_load_speed_{s}"),
                "circular (Ke fitted to the no-load points)",
            ),
            _row(
                "no-load current A",
                v,
                abs(nl["current"]),
                part.value(f"no_load_current_{s}"),
                "circular (i0 = this value)"
                if v == 7.4
                else "independent; FAIL cause: the model i0 is a constant (the 7.4 V value) and the datasheet current falls with V",
            ),
        ]
        ref = part.value(f"no_load_speed_{s}")
        sr = full_duty_step(v)
        row = _row(
            "step 60deg s (full duty)",
            v,
            sr["t_cover"],
            math.radians(STEP_DEG) / ref,
            STEP_KIND,
        )
        row["pass"] = (
            None  # not a comparison, see STEP_KIND (the step checks are in step_checks)
        )
        rows.append(row)
    return rows


def step_table() -> list:
    """Step records for the registers of the part: LeRobot P=16 / factory P=32 at the arm's pid_scale, plus full duty."""
    pid = servo_config()["pid_scale"]
    out = []
    for v in (6.0, 7.4):
        for name, r in (
            (f"P=16 D=32 pid_scale={pid:g}", step_response(v, 16, pid)),
            (f"P=32 D=32 pid_scale={pid:g}", step_response(v, 32, pid)),
            ("full duty (P=1e4 D=0)", full_duty_step(v)),
        ):
            out.append({"V": v, "registers": name, **r})
    return out


def arm_sim(pid_scale=None, p=None):
    """SO-101 (assemblies/so101.toml via design.compute) as a Sim, torque off; optional pid_scale / p override on all six
    servos (config edit of the built MJCF: pid_scale is a config, not a runtime attr)."""
    _, xml, _ = _built()
    text = xml.read_text()

    def patch(m):
        dev = json.loads(m.group(1))
        for s in dev["servos"]:
            s["pid_scale"] = (
                float(pid_scale) if pid_scale is not None else s["pid_scale"]
            )
            s["p"] = float(p) if p is not None else s["p"]
        return f"name=\"napl.devices\" data='{json.dumps(dev)}'/>"

    patched = xml.with_name("so101_sag.xml")
    patched.write_text(
        re.sub(r"name=\"napl\.devices\" data='(.*?)'/>", patch, text, flags=re.DOTALL)
    )
    sim = napl.Sim.load(patched)
    return sim, lambda n, a: float(sim.query(f"/devices/{n}", a)[0][0])


def holding_sag(pid_scale=None, p=None, seconds=1.0) -> dict:
    """The arm holding goal 0, torque on, for `seconds` (pid_scale / p overrides as arm_sim).
    -> {joint: {sag_deg, counts, duty, torque}}"""
    sim, q = arm_sim(pid_scale, p)
    for n in SERVOS:
        sim.set(f"/devices/{n}", "goal_position", 0.0)
        sim.set(f"/devices/{n}", "torque_enable", 1)
    sim.step(round(seconds * 1000))
    cpr = servo_config()["encoder_cpr"]
    return {
        j: {
            "sag_deg": q(n, "counts") * 360 / cpr,
            "counts": q(n, "counts"),
            "duty": q(n, "duty"),
            "torque": q(n, "torque"),
        }
        for n, j in zip(SERVOS, JOINTS)
    }


def pan_settle_error(
    p: float, pid_scale: float = 4000.0, goal=0.5, seconds=1.5
) -> dict:
    """Unloaded shoulder_pan settling error in counts (|counts - goal counts|), the goal approached from below (0 -> goal) and
    from above (2 goal -> goal), other joints holding 0 (LeRobot issue 3400's test)."""
    cpr, out = servo_config()["encoder_cpr"], {}
    for side, start in (("below", 0.0), ("above", 2 * goal)):
        sim, q = arm_sim(pid_scale, p)
        for n in SERVOS:
            sim.set(f"/devices/{n}", "goal_position", 0.0)
            sim.set(f"/devices/{n}", "torque_enable", 1)
        sim.set("/devices/servo1", "goal_position", start)
        sim.step(round(seconds * 1000))
        sim.set("/devices/servo1", "goal_position", goal)
        sim.step(round(seconds * 1000))
        out[side] = abs(q("servo1", "counts") - round(goal * cpr / (2 * math.pi)))
    return out


def main():
    rows = datasheet_rows()
    print(
        f"| quantity | V | sim | datasheet | err % | pass (+-{TOL:g} %) | kind |\n|---|---|---|---|---|---|---|"
    )
    for r in rows:
        verdict = "n/a" if r["pass"] is None else "PASS" if r["pass"] else "FAIL"
        print(
            f"| {r['quantity']} | {r['V']:g} | {r['sim']:.4g} | {r['datasheet']:.4g} | {r['err_pct']:+.1f} | {verdict} | {r['kind']} |"
        )
    print(
        "\n12 V is a different part (STS3215 C018/C047, 12 V coil, 30 kg.cm) and is not benched: the C001 is a 4-7.4 V servo."
    )
    print(
        "Stall torque at 6 V: the datasheet source is ambiguous (Feetech web table '19.5kg.cm@6V' vs the PDF 16.5 kg.cm at 6 V,"
        " docs/research/sts3215-calibration.md concern 1, +-18 %); the bench uses the PDF value."
    )
    print(
        "Max duty: BAM measured 0.97 on an STS3215 (oscilloscope) vs 1.0 here (max_torque 1000); not modelled.\n"
    )
    print(
        "full-duty 60 deg step vs the datasheet no-load speed (J / b = reflected armature / damping split coefficient):"
    )
    print(
        "| V | t s | 60deg / w_datasheet s | t - 60deg / w_sim s | J / b s | peak speed / w_datasheet |\n|---|---|---|---|---|---|"
    )
    for v in (6.0, 7.4):
        c = step_checks(v)
        print(
            f"| {v:g} | {c['t']:.3f} | {c['t_ds']:.3f} | {c['excess']:.3f} | {c['tau']:.3f} | {c['peak_ratio']:.3f} |"
        )
    print()
    print(
        "| V | registers | t to 60deg (2 counts) s | rise 10-90 % s | overshoot % | settle (2 %) s |\n|---|---|---|---|---|---|"
    )
    for r in step_table():
        print(
            f"| {r['V']:g} | {r['registers']} | {r['t_cover']:.3f} | {r['rise']:.3f} | {r['overshoot']:.1f} | {r['settle']:.3f} |"
        )
    print(
        "\nholding sag at goal 0, 1 s, before (8192) / after (4000 = as built) the calibration:"
    )
    print(
        "| pid_scale | P | joint | sag deg | counts | duty |\n|---|---|---|---|---|---|"
    )
    for ps in (8192, 4000):
        for p in (16, 32):
            for j, s in holding_sag(ps, p).items():
                if j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
                    print(
                        f"| {ps} | {p} | {j} | {s['sag_deg']:.2f} | {s['counts']:.1f} | {s['duty']:+.3f} |"
                    )
    print(
        "\nunloaded shoulder_pan settling error |counts| (pid_scale 4000), goal approached from below / above;"
        " LeRobot issue 3400 (hardware): 63 ticks at P=16, 26.7 at P=32:"
    )
    print("| P | from below | from above |\n|---|---|---|")
    for p in (16, 32):
        e = pan_settle_error(p)
        print(f"| {p} | {e['below']:.0f} | {e['above']:.0f} |")
    print(
        "The sim shows less than the hardware because its friction is tanh-smoothed (no true stiction dead-band)."
    )


if __name__ == "__main__":
    main()
