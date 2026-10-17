#!/usr/bin/env python3
"""SP3b speed design sweep on the FULL dog model (spec s5): actuator x knee drive ratio x leg length.

Measured on the built dog, per point (no gait controller needed):
  stall   saturating command at dq = 0 -> hip / knee stall torque as the Qdd delivers it from this bus (N m)
  swing   (base fixed) fl hip minimum-time move over the study's 25 deg stride sweep, bang-bang at the Qdd's own envelope,
          rest of the leg held at the stand pose -> t_move (s), dq_peak (rad/s)
  push    standing at STAND, every hip / knee commanded fully extended with kp = kp_max for 0.3 s -> peak sum of foot forces / (m g)
  jump    from SQUAT the same command -> apex rise of the deck (m), peak / mean pack current (A), min pack voltage (V)
  drop    the standing dog lifted 0.25 m and released under the stand gains -> peak |tau_est| (N m), any fault, min deck height (m)
  hold    30 s standing -> max winding temperature rise (K), mean pack power (W)
Derived (tools/dog_actuator_study.py formulas, sim-measured inputs; h = 2 L cos(PHI) is the study's trot stance height): v_swing = h sin(THETA) / t_move; v_stance = h cos(THETA) w0 (1 -
tau_hip / stall_hip) with w0 = V_pack / (ke_si N), tau_hip = DYN PEAK_FACTOR m g / 2 h sin(THETA); v_trot_bound = min; pack_margin
= 2 x 70 A / peak pack current. Not measured: a gait's top speed and cost of transport (SP4: --gait module:function).
GIM6010-8 is a probe: its electrics and mass in the GIM8108-8 seats (no GIM6010 printed parts); its cost swaps the 12 actuator prices.
The knee ratio only changes the Qdd's gear_ratio (8 x ratio): the crank / lug geometry stays at 1.5, so a ratio 2.0 point is an
electrical what-if until a 40 mm lug is drawn.

    python tools/dog_sweep.py [--points act:ratio:L ...] [--quick] [--gait module:function] [--out docs/dog] [--jobs N]
"""

import argparse
import importlib
import json
import math
import multiprocessing as mp
import os
import shutil
import sys
import tempfile
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "examples" / "dog"), str(REPO / "tools")]
import dog_actuator_study as STUDY
import dogsim as D
import napl
from napl import design
from napl.design import Library
from napl.design import assembly as A

DOG = REPO / "assemblies" / "dog"
ACTS = {"gim8108": "steadywin-gim8108-8", "gim6010": "steadywin-gim6010-8"}
PACK = "auline-21700-s70-8s1p-4500mah"
LSFX = {0.20: "-200", 0.22: "", 0.25: "-250"}
DEFAULT_POINTS = [
    (a, r, L) for a in ACTS for r in (1.5, 2.0) for L in (0.20, 0.22, 0.25)
]  # study candidates 1-3 + L neighbourhood
COLUMNS = {
    "actuator": "bom", "knee_ratio": "bom", "leg_length": "bom", "mass": "bom", "cost": "bom", "cost_estimated_share": "bom",
    "stall_hip": "measured", "stall_knee": "measured", "t_move": "measured", "dq_peak": "measured",
    "push_force_ratio": "measured", "jump_rise": "measured", "i_pack_peak": "measured", "i_pack_mean_jump": "measured",
    "v_pack_min": "measured", "drop_tau_peak": "measured", "drop_fault": "measured", "drop_clearance": "measured",
    "hold_temp_rise": "measured", "hold_power": "measured",
    "w0": "derived", "pack_margin": "derived", "v_swing": "derived", "v_stance": "derived", "v_trot_bound": "derived",
    "v_top_trot": "sp4", "cot": "sp4",
}  # fmt: skip
G = STUDY.G
DROP_HEIGHT = 0.25  # m
PUSH_S, JUMP_S, DROP_S, HOLD_S = 0.3, 1.0, 1.0, 30.0
HOLD_HZ = 100.0  # command rate while standing still (the Qdd command timeout is 50 ms)


def _replace_once(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new)


def write_variant(tmp, act, ratio, L, base="free"):
    """dog.toml + its sub files into tmp for one point; -> the dog.toml path. base "fixed" welds the deck at the origin (swing runs)."""
    tmp = Path(tmp)
    tmp.mkdir(parents=True, exist_ok=True)
    s = LSFX[L]
    leg = (DOG / "leg.toml").read_text()
    leg = _replace_once(
        leg, 'part = "dog-thigh-bracket-l"', f'part = "dog-thigh-bracket-l{s}"'
    )
    leg = _replace_once(leg, 'part = "dog-shank"', f'part = "dog-shank{s}"')
    leg = _replace_once(
        leg, 'part = "dog-knee-rod-220"', f'part = "dog-knee-rod-{round(L * 1000)}"'
    )
    (tmp / "leg.toml").write_text(leg)
    if act != "gim8108":
        lasm = A.load(tmp / "leg.toml")
        P = Library(REPO / "parts").get(ACTS[act])
        assert P.value("gear_ratio") == 8  # drive_ratio = 8 x ratio stays right
        fields = lasm.raw["device_maps"]["qdd"]
        st = {f: {"value": P.value(pp), "unit": P.params[pp]["unit"], "fidelity": "derived",
                  "derivation": f"{P.id}.{pp}: sweep probe, {P.id} electrics in the GIM8108-8 seat (docs/dog/speed-sweep.md)"}
              for f, pp in fields.items()}  # fmt: skip
        for d in lasm.raw["devices"]:
            if d["type"] == "qdd":
                d.pop("map")
                d["set"] = {**st, **d.get("set", {})}
        for i in lasm.raw["instances"]:
            if i["part"] == "steadywin-gim8108-8":
                i.setdefault("params", {})["mass"] = {
                    "value": P.value("mass"),
                    "unit": "kg",
                    "fidelity": "derived",
                    "derivation": f"{P.id}.mass (sweep probe)",
                }
        (tmp / "leg.toml").write_text(A.dumps(lasm))
    for f in ("head.toml", "power.toml", "compute.toml", "dog.toml"):
        (tmp / f).write_text((DOG / f).read_text())
    dasm = A.load(tmp / "dog.toml")
    if base != "free":
        dasm.raw["assembly"]["base"] = base
    for sub in dasm.raw["subassemblies"]:
        if sub["file"] == "leg.toml":
            sub.setdefault("params", {})["knee_drive_ratio"] = {
                "value": 8 * ratio,
                "unit": "1",
                "fidelity": "derived",
                "derivation": f"8 x knee linkage ratio {ratio} (sweep point)",
            }
    (tmp / "dog.toml").write_text(A.dumps(dasm))
    return tmp / "dog.toml"


# ---- the sim side ------------------------------------------------------------------------------------------------------------------
def _model(tmp, act, ratio, L, base):
    """-> (xml path, report). The sweep needs neither the wire tubes nor the visual meshes: compute() without them is ~100 x faster."""
    src = write_variant(Path(tmp) / f"{act}-{ratio}-{L}-{base}", act, ratio, L, base)
    out = Path(tmp) / f"out-{act}-{ratio}-{L}-{base}"
    out.mkdir(parents=True, exist_ok=True)
    xml, rep = design.compute(src, D.PARTS, out, visual=False, route=False)[:2]
    if base == "fixed":  # no floor under a welded dog: the legs hang free
        xml = _replace_once(
            xml,
            '<geom name="floor" type="plane"',
            '<geom name="floor" type="plane" pos="0 0 -5"',
        )
    p = out / "dog.xml"
    p.write_text(xml)
    return p, rep


def val(sim, dev, attr):
    return float(sim.query(f"/devices/{dev}", attr)[0][0])


def _packs(sim):
    return [
        (val(sim, f"power:{p}", "voltage"), abs(val(sim, f"power:{p}", "current")))
        for p in ("pack_a", "pack_b")
    ]


def _stand(sim, sg, l1, l2, frac, rows):
    """Straight legs -> frac of the leg length over 1.5 s, hold 1 s; -> the deck z reference."""
    z0 = D.ramp(sim, sg, l1, l2, 1.0, frac, 1.5, rows)
    return D.ramp(sim, sg, l1, l2, frac, frac, 1.0, rows, z0)


def _extend(sim, sg, skip=()):
    """Every hip / knee fully extended (q = 0) at the Qdd's own gain limit; the abduction joints keep their stand gains."""
    for dev, s in sg.items():
        if dev in skip:
            continue
        j = dev.split(":")[1]
        big = j in ("hip", "knee")
        for attr, v in (
            ("p_des", 0.0),
            ("v_des", 0.0),
            ("kp", 1e4 if big else D.KP),
            ("kd", 5.0 if big else D.KD),
            ("tau_ff", 0.0),
        ):
            sim.set(f"/devices/{dev}", attr, v)


def measure_stall(sim, sg, leg="fl"):
    """Saturating torque command from rest, one tick: what the Qdd delivers from this bus."""
    out = {}
    for j in ("hip", "knee"):
        dev = f"{leg}:{j}"
        sim.set(f"/devices/{dev}", "kp", 0.0)
        sim.set(f"/devices/{dev}", "kd", 0.0)
        sim.set(f"/devices/{dev}", "tau_ff", 1e3)
        peak = 0.0
        for _ in range(
            6
        ):  # the Qdd's CAN latency and command period delay the torque by a few ticks; the joint has not moved yet
            sim.step(1)
            peak = max(peak, abs(val(sim, dev, "tau_est")))
        out[j] = peak
        sim.set(f"/devices/{dev}", "tau_ff", 0.0)
    return out["hip"], out["knee"]


def move_time(sim, sg, hold_q, dev, a, b, dt=0.001):
    """Bang-bang at the torque limit from joint angle a to b (rest to rest): full torque to the mid-point, then reversed until dq changes
    sign. -> (t, dq_peak). The other joints hold hold_q (re-commanded every 10 ticks, inside the 50 ms command timeout)."""
    up = 1.0 if b > a else -1.0
    t, peak, phase = 0.0, 0.0, 0
    path = f"/devices/{dev}"
    sim.set(path, "kp", 0.0)
    sim.set(path, "kd", 0.0)
    while True:
        if round(t / dt) % 10 == 0:
            D.command(sim, {d: s for d, s in sg.items() if d != dev}, hold_q)
        sim.set(path, "tau_ff", up * 1e3 * (1 if phase == 0 else -1))
        sim.step(1)
        t += dt
        q, dq = val(sim, dev, "q"), val(sim, dev, "dq")
        peak = max(peak, abs(dq))
        if phase == 0 and (q - (a + b) / 2) * up >= 0:
            phase = 1
        elif phase == 1 and dq * up <= 0:
            return t, peak
        if t > 2.0:
            raise RuntimeError(f"{dev}: no rest after 2 s")


def measure_swing(xml, rep):
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    qh, qk = D.ik(D.STAND * (l1 + l2), l1, l2)
    hold_q = {"abad": 0.0, "hip": qh, "knee": qk}
    for _ in range(50):  # 0.5 s at 100 Hz: the legs hang at the stand pose
        D.command(sim, sg, hold_q)
        sim.step(10)
    stall_hip, stall_knee = measure_stall(sim, sg)
    for _ in range(25):  # settle again after the stall pulses
        D.command(sim, sg, hold_q)
        sim.step(10)
    t, dq = move_time(
        sim,
        sg,
        hold_q,
        "fl:hip",
        val(sim, "fl:hip", "q"),
        val(sim, "fl:hip", "q") + STUDY.THETA,
    )
    return dict(stall_hip=stall_hip, stall_knee=stall_knee, t_move=t, dq_peak=dq)


def measure_push(xml, rep, quick):
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    _stand(sim, sg, l1, l2, D.STAND, [])
    m = rep["mass"]["total"]
    peak = 0.0
    _extend(sim, sg)
    for _ in range(round(PUSH_S * D.CMD_HZ)):
        _extend(sim, sg)
        sim.step(round(1 / D.CMD_HZ / 0.001))
        peak = max(peak, sum(val(sim, f"{leg}:foot", "force") for leg in D.LEGS))
    return dict(push_force_ratio=peak / (m * G))


def measure_jump(xml, rep, quick):
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    _stand(sim, sg, l1, l2, D.SQUAT, [])
    z_start = float(sim.query("/world/deck_top", "pos")[0][2])
    n = round(JUMP_S * D.CMD_HZ)
    apex, i_tot, v_min = 0.0, [], 1e9
    for _ in range(n):
        _extend(sim, sg)
        sim.step(round(1 / D.CMD_HZ / 0.001))
        apex = max(apex, float(sim.query("/world/deck_top", "pos")[0][2]) - z_start)
        pk = _packs(sim)
        i_tot.append(sum(i for _, i in pk))
        v_min = min(v_min, *(v for v, _ in pk))
    return dict(
        jump_rise=apex,
        i_pack_peak=max(i_tot),
        i_pack_mean_jump=sum(i_tot) / len(i_tot),
        v_pack_min=v_min,
    )


def measure_drop(xml, rep, quick):
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    _stand(sim, sg, l1, l2, D.STAND, [])
    free = "/world/deck_top/deck_top:free"
    qpos = [float(x) for x in sim.query(free, "qpos")[0]]
    qpos[2] += DROP_HEIGHT
    sim.set(free, "qpos", qpos)
    tau_peak, fault, clearance = 0.0, 0.0, 1e9
    for _ in range(round(DROP_S * D.CMD_HZ)):
        D.command(
            sim,
            sg,
            dict(
                zip(("abad", "hip", "knee"), (0.0, *D.ik(D.STAND * (l1 + l2), l1, l2)))
            ),
        )
        sim.step(round(1 / D.CMD_HZ / 0.001))
        for dev in sg:
            tau_peak = max(tau_peak, abs(val(sim, dev, "tau_est")))
            fault = max(fault, val(sim, dev, "fault"))
        clearance = min(clearance, float(sim.query("/world/deck_top", "pos")[0][2]))
    return dict(drop_tau_peak=tau_peak, drop_fault=fault, drop_clearance=clearance)


def measure_hold(xml, rep, quick):
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    _stand(sim, sg, l1, l2, D.STAND, [])
    qh, qk = D.ik(D.STAND * (l1 + l2), l1, l2)
    q = {"abad": 0.0, "hip": qh, "knee": qk}
    t0 = {d: val(sim, d, "temp") for d in sg}
    secs = HOLD_S / 10 if quick else HOLD_S
    n, k = round(secs * HOLD_HZ), round(1 / HOLD_HZ / 0.001)
    power, vsum = 0.0, 0.0
    for _ in range(n):
        D.command(sim, sg, q)
        sim.step(k)
        pk = _packs(sim)
        power += sum(v * i for v, i in pk)
        vsum += sum(v for v, _ in pk) / len(pk)
    return dict(
        hold_temp_rise=max(val(sim, d, "temp") - t0[d] for d in sg),
        hold_power=power / n,
        v_hold=vsum / n,
    )


def derive(m, act, mass, h):
    P = Library(REPO / "parts").get(ACTS[act])
    w0 = m["v_hold"] / (P.value("ke_si") * 8.0)
    pack_i = Library(REPO / "parts").get(PACK).value("i_max_continuous")
    tau_hip = STUDY.DYN * STUDY.PEAK_FACTOR * mass * G / 2 * h * math.sin(STUDY.THETA)
    v_swing = h * math.sin(STUDY.THETA) / m["t_move"]
    v_stance = h * math.cos(STUDY.THETA) * w0 * max(0.0, 1 - tau_hip / m["stall_hip"])
    return dict(
        w0=w0,
        pack_margin=2 * pack_i / m["i_pack_peak"],
        v_swing=v_swing,
        v_stance=v_stance,
        v_trot_bound=min(v_swing, v_stance),
    )


def run_point(point, quick=False, gait=None):
    """One sweep point -> {column: value} for every COLUMNS key."""
    act, ratio, L = point
    tmp = tempfile.mkdtemp(prefix="dogsweep-")
    try:
        free, rep = _model(tmp, act, ratio, L, "free")
        fixed, rep_f = _model(tmp, act, ratio, L, "fixed")
        m = {}
        m.update(measure_swing(fixed, rep_f))
        m.update(measure_push(free, rep, quick))
        m.update(measure_jump(free, rep, quick))
        m.update(measure_drop(free, rep, quick))
        m.update(measure_hold(free, rep, quick))
        mass = rep["bom"]["total"]["mass"]
        lib = Library(REPO / "parts")
        cost = rep["bom"]["total"]["cost"]
        if act != "gim8108":  # probe: the BOM priced 12 GIM8108-8
            cost -= 12 * (
                lib.get(ACTS["gim8108"]).value("price")
                - lib.get(ACTS[act]).value("price")
            )
        out = dict(actuator=ACTS[act], knee_ratio=ratio, leg_length=L, mass=mass, cost=round(cost, 2),
                   cost_estimated_share=rep["bom"]["total"]["estimated_share"])  # fmt: skip
        out.update({c: m[c] for c, k in COLUMNS.items() if k == "measured"})
        out.update(
            derive(m, act, mass, 2 * L * math.cos(STUDY.PHI))
        )  # the study's trot stance height h = 2 L cos(PHI), PHI = 40 deg
        out["v_top_trot"] = out["cot"] = None
        if gait:
            mod, fn = gait.split(":")
            sim = napl.Sim.load(free)
            sg = D.signs(rep)
            D.enable(sim, sg)
            _stand(sim, sg, *D.leg_lengths(sim), D.STAND, [])
            out["v_top_trot"] = float(
                getattr(importlib.import_module(mod), fn)(sim, sg, rep)
            )
        return {c: out[c] for c in COLUMNS}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---- selection and the report --------------------------------------------------------------------------------------------------------
def eligible(p):
    return (
        p["pack_margin"] >= 1.0 and not p["drop_fault"] and p["push_force_ratio"] >= 2.0
    )


def pareto(points):
    """Front over (v_trot_bound max, cost min, mass min) among eligible points."""
    ok = [p for p in points if eligible(p)]

    def dom(a, b):  # a dominates b
        ge = (
            a["v_trot_bound"] >= b["v_trot_bound"]
            and a["cost"] <= b["cost"]
            and a["mass"] <= b["mass"]
        )
        gt = (
            a["v_trot_bound"] > b["v_trot_bound"]
            or a["cost"] < b["cost"]
            or a["mass"] < b["mass"]
        )
        return ge and gt

    return [p for p in ok if not any(dom(q, p) for q in ok)]


def choose(points):
    front = pareto(points)
    return max(front, key=lambda p: (p["v_trot_bound"], -p["cost"])) if front else None


def _name(p):
    return f"{p['actuator'].replace('steadywin-', '')} r{p['knee_ratio']} L{p['leg_length'] * 1000:.0f}"


def write_md(out, data, cmd):
    pts, ch = data["points"], data["chosen"]
    f = lambda x, n=2: "-" if x is None else f"{x:.{n}f}"
    L = [
        "# Dog speed sweep (SP3b)",
        "",
        f"Generated {data['generated']} by `{cmd}` on the full dog model (assemblies/dog/dog.toml, 12 Qdd, flat floor).",
        "",
        "## How to read",
        "",
        "Every column is labelled by how it was obtained:",
        "",
        "* **measured**: on the full model in this run (stall, swing, push, jump, drop, hold; see `tools/dog_sweep.py`).",
        "* **derived (study formulas, sim-measured inputs)**: `tools/dog_actuator_study.py` formulas fed with measured t_move, stall torque and pack voltage: `w0`, `pack_margin`, `v_swing`, `v_stance`, `v_trot_bound`. These are bounds of a trot, NOT a gait speed.",
        "* **bom**: from the build report (`design.build` BOM / mass).",
        "* **not measured (SP4)**: `v_top_trot`, `cot` need a gait controller (`--gait module:function`); they stay empty here.",
        "",
        "GIM6010-8 points are a probe: GIM6010-8 electrics and mass in the GIM8108-8 seats (there are no GIM6010 printed parts). Knee ratio only sets the Qdd `gear_ratio` = 8 x ratio; the crank / lug geometry stays 1.5, so ratio 2.0 is an electrical what-if.",
        "",
        "## Points",
        "",
        "| point | mass kg | cost USD | stall hip/knee N m | t_move s | push F/mg | jump rise m | I peak A | margin | drop tau N m | drop fault | hold dT K | hold W | v_swing | v_stance | v_trot_bound m/s |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for p in pts:
        L.append(
            f"| {_name(p)} | {f(p['mass'], 3)} | {f(p['cost'], 0)} | {f(p['stall_hip'], 1)} / {f(p['stall_knee'], 1)} | {f(p['t_move'], 3)} | {f(p['push_force_ratio'])} | {f(p['jump_rise'], 3)} | "
            f"{f(p['i_pack_peak'], 1)} | {f(p['pack_margin'])} | {f(p['drop_tau_peak'], 1)} | {int(p['drop_fault'])} | {f(p['hold_temp_rise'], 1)} | {f(p['hold_power'], 0)} | "
            f"{f(p['v_swing'])} | {f(p['v_stance'])} | {f(p['v_trot_bound'])} |"
        )
    L += [
        "",
        "Eligible: pack_margin >= 1.0, no drop fault, push F/mg >= 2.0.",
        "",
        "## Pareto front (v_trot_bound max, cost min, mass min)",
        "",
    ]
    L += ["| point | v_trot_bound m/s | cost USD | mass kg |", "|---|---|---|---|"]
    L += [
        f"| {_name(p)} | {f(p['v_trot_bound'])} | {f(p['cost'], 0)} | {f(p['mass'], 3)} |"
        for p in pareto(pts)
    ]
    L += ["", "## Chosen point", ""]
    if ch:
        L += [
            f"**{_name(ch)}**: v_trot_bound {f(ch['v_trot_bound'])} m/s (derived, not a measured gait), cost {f(ch['cost'], 0)} USD, mass {f(ch['mass'], 3)} kg.",
            "",
            f"Gap to the 3.5 m/s criterion (spec s7): {f(3.5 - ch['v_trot_bound'])} m/s {'short' if ch['v_trot_bound'] < 3.5 else '(met on paper)'}; "
            f"to the 4.5 k USD target: {f(ch['cost'] - 4500, 0)} USD {'over' if ch['cost'] > 4500 else 'under'}. "
            "The real top speed needs SP4's gait (flying trot / bound); this table claims none.",
        ]
    else:
        L.append("No point is eligible.")
    (Path(out) / "speed-sweep.md").write_text("\n".join(L) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--points",
        nargs="*",
        help="act:ratio:L, e.g. gim8108:1.5:0.22 (default: the 12 study points)",
    )
    ap.add_argument("--quick", action="store_true", help="hold 3 s instead of 30 s")
    ap.add_argument(
        "--gait",
        help="module:function(sim, sg, report) -> mean forward speed over 10 s (the SP4 slot)",
    )
    ap.add_argument("--out", default=str(REPO / "docs" / "dog"))
    ap.add_argument("--jobs", type=int, default=0)
    a = ap.parse_args(argv)
    pts = (
        sorted(
            (x.split(":")[0], float(x.split(":")[1]), float(x.split(":")[2]))
            for x in a.points
        )
        if a.points
        else sorted(DEFAULT_POINTS)
    )
    jobs = a.jobs or int(os.environ.get("NAPL_THREADS", "1"))
    args = [(p, a.quick, a.gait) for p in pts]
    if jobs > 1:
        with mp.Pool(jobs) as pool:
            res = pool.starmap(run_point, args)
    else:
        res = [run_point(*x) for x in args]
    data = dict(
        columns=COLUMNS, points=res, chosen=None, generated=date.today().isoformat()
    )
    data["chosen"] = choose(res)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "speed-sweep.json").write_text(json.dumps(data, indent=1))
    write_md(
        out,
        data,
        "python tools/dog_sweep.py " + " ".join(sys.argv[1:] if argv is None else argv),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
