"""Virtual benches (milestone N4): a thrust stand that reproduces vendor thrust-test tables from device physics, and an
open-loop hover / endurance estimate for the X500 V2.

    napl bench thrust <motor-id> [--prop PART] [--table NAME]
    napl bench all                      # every motor with a [tables.thrust_test*]; writes build/bench + docs/validation
    napl bench hover [--auw KG]         # X500 V2 hover throttle / current / power / endurance

Method (docs/validation/thrust-stands.md has the generated results):
  * MJCF stand: welded stator, rotor hinge body (prop inertia) whose dof armature is the motor bell inertia, one PSU
    (vendors test on a regulated supply, small series R, voltage set PER ROW from the table), one ESC+BLDC device, one
    rotor (J = 0 static C_T / C_Q), no ground effect, no environment device (ISA sea level, still air), gravity off.
  * Every row is a fresh stand run to steady state (omega converged); thrust = rotor attr, current = supply current,
    rpm, electrical power = V I (or shaft power Q w when the vendor table's power column is mechanical).
  * Datasheet values (kv, R, I0) are FIXED. Unknown quantities are fitted on a small subset of rows ({0, n/2, n-1}) by
    Levenberg-Marquardt on the closed-form steady state of the same device equations (the bench checks that this matches
    the simulated stand). Errors are reported on the remaining HELD-OUT rows. Fitted values go to build/bench, never parts/.
"""

import functools
import json
import math
import re
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PARTS = ROOT / "parts"
OUT = ROOT / "build" / "bench"
DOCS = ROOT / "docs" / "validation" / "thrust-stands.md"
RHO0 = 1.2250  # ISA sea level, matches the C++ ISA model
R_SERIES = 0.001  # PSU series resistance (ohm): regulated bench supply
WS = 5.0  # friction smoothing speed, rad/s (device default)
IN = 0.0254
T_AMB = 25.0  # degC: ambient of the vendor stands (thermal model); R20 is referenced to 20 degC
T_ISO = 20.0  # degC: winding temperature of the isothermal baseline stand
ALPHA_CU = 0.00393  # copper temperature coefficient, 1/K (device default)

# ---- N4b model switches. "baseline" = the N4 model (constant static C_T/C_Q, constant-friction I0, isothermal 20 degC).
# "n4b" = Reynolds factor + speed-dependent no-load loss + thermal steady state. LOSS_VISC / LOSS_QUAD are the shares of the
# datasheet I0 torque (at the speed it was measured) that scale as w and w|w| (docs/models.md): ONE global pair, chosen by
# the pooled TRAIN-row cost of all tables (scan_loss_split), never per motor and never from held-out rows. The scan
# (in the generated doc) has its minimum at (0, 0): the train rows reject a speed-dependent split of the datasheet I0, so
# the adopted value is the backwards-compatible constant friction. The device supports the split (tested).
LOSS_VISC, LOSS_QUAD = 0.0, 0.0
LOSS_GRID = [(0.0, 0.0), (0.1, 0.0), (0.25, 0.0), (0.5, 0.0), (1.0, 0.0), (0.25, 0.25)]
MODELS = {
    "baseline": dict(re=False, visc=0.0, quad=0.0, thermal=False),
    "n4b": dict(re=True, visc=LOSS_VISC, quad=LOSS_QUAD, thermal=True, esc=True),
}
# `esc` switch (default off when the key is absent): model the ESC losses (r_on, sw_loss, i_q of the ESC part named here) in
# series with the winding, so the fitted C_Q0 no longer absorbs them. Only the 2216 table has an ESC to model: the Holybro
# vendor test does not name its ESC, the kit's own BLHeli_S 20A part is the stand-in (docs/validation/thrust-stands.md,
# "ESC loss"); every other table keeps its ideal stage, so their numbers are unchanged.
ESC_FOR = {"holybro-2216-kv920": "holybro-x500v2-esc-blheli-s-20a"}

# Vendor-printed motor temperature (degC, "Operating temperature ... at 100 % throttle" / "Temperature in full-throttle
# load"; the 2807 / 2814 / 3110 / 4214 / mn* tables say "Temperature N C"), transcribed from the comments above each table
# in parts/ (the parts DB has no structured field for it). Surface temperature of the motor, taken here as the winding
# temperature of the last (100 %) row; mn2806 is after 10 min at 80 %, not 100 %. Used only to calibrate r_th (below).
VENDOR_TEMP_C = {
    ("holybro-2216-kv920", "thrust_test"): 80.0,
    ("iflight-xing2-2207-1855kv", "thrust_test"): 55.0,
    ("iflight-xing2-2207-1855kv", "thrust_test_5140"): 56.0,
    ("iflight-xing2-2207-2755kv", "thrust_test"): 60.0,
    ("iflight-xing2-2207-2755kv", "thrust_test_5140"): 63.0,
    ("tmotor-2807-kv1250", "thrust_test"): 49.0,
    ("tmotor-2814-kv900", "thrust_test"): 55.0,
    ("tmotor-3110-kv1580", "thrust_test"): 55.0,
    ("tmotor-4214-kv380", "thrust_test"): 35.0,
    ("tmotor-air2213-kv1000", "thrust_test"): 47.0,
    ("tmotor-mn2806-kv400", "thrust_test"): 45.0,
    ("tmotor-mn3508-kv700", "thrust_test"): 48.0,
    ("tmotor-mn4004-kv300", "thrust_test"): 56.0,
    ("tmotor-mn4006-kv380", "thrust_test"): 46.0,
    ("tmotor-mn4110-kv340", "thrust_test"): 102.0,
    ("tmotor-mn5006-kv300", "thrust_test"): 67.0,
    ("tmotor-mn5008-kv340", "thrust_test"): 88.0,
}

from .design.reynolds import RE_X_REF, UIUC_STATIC
from .design.reynolds import reynolds_shape as _reynolds_shape


# ---------------------------------------------------------------- parts reader (tiny, private)
@functools.cache
def _library():
    from .design.library import Library

    return Library(PARTS)


def load_part(pid: str) -> dict:
    """Raw part document via napl.design.library (read-only use)."""
    try:
        return _library().get(pid).raw
    except Exception as e:  # DesignError: unknown / broken part
        raise KeyError(str(e)) from None


def pv(part: dict, key: str, default=None):
    p = part["params"].get(key)
    return default if p is None else p["value"]


def thrust_tables(part: dict) -> dict:
    return {
        k: v for k, v in part.get("tables", {}).items() if k.startswith("thrust_test")
    }


def tabled_motors() -> list:
    return [pid for pid, p in sorted(_library().parts.items()) if thrust_tables(p.raw)]


@functools.cache
def reynolds_shape() -> dict:
    """Pooled UIUC Reynolds factor shape; the code is napl.design.reynolds.reynolds_shape (shared with the builder)."""
    return _reynolds_shape(_library())


def rotor_re(case: dict) -> dict:
    """Rotor-device fields for the Reynolds factor (empty = off)."""
    if not case["model"]["re"]:
        return {}
    s = reynolds_shape()
    return {"re_x": s["x"], "re_ft": s["ft"], "re_fq": s["fq"]}


# ---------------------------------------------------------------- case = motor + prop + table
_LABEL_D = {
    "51466": 5.1 * IN,
    "5140": 5.1 * IN,
}  # Gemfan 5.1 in 3-blade (label digits: 51 = 5.1 in)


def prop_for(table: dict, prop_id: str | None) -> dict:
    """Prop geometry for a table. Flags say which values are datasheet and which are estimates."""
    pid = prop_id or table.get("prop")
    flags = {}
    if pid:
        p = load_part(pid)
        d = pv(p, "diameter")
        flags["diameter"] = p["params"]["diameter"]["fidelity"]
        m = pv(p, "mass")
        j = pv(p, "j_prop")
        if j is None:
            j = m * d * d / 12
            flags["j_prop"] = "estimated: slender rod m D^2/12 from part mass"
        else:
            flags["j_prop"] = p["params"]["j_prop"]["fidelity"]
        return {
            "id": pid,
            "label": table.get("prop_label", pid),
            "diameter": d,
            "mass": m,
            "j": j,
            "flags": flags,
        }
    label = table["prop_label"]
    if label in _LABEL_D:
        d = _LABEL_D[label]
        flags["diameter"] = "estimated: 5.1 in from the Gemfan label digits"
    else:
        mt = re.search(r"(?:^|[\s-])[A-Za-z]?(\d+(?:\.\d+)?)\*", label)
        if not mt:
            raise ValueError(f"cannot read a diameter from prop label '{label}'")
        d = float(mt.group(1)) * IN
        flags["diameter"] = "estimated: nominal inches from the T-MOTOR label"
    m = 0.013 * (d / 0.381) ** 1.5  # CF-prop mass trend of the tmotor-ns* parts
    flags["j_prop"] = (
        "estimated: mass trend of tmotor-ns* props, rod m D^2/12 (no effect on steady state)"
    )
    return {
        "id": None,
        "label": label,
        "diameter": d,
        "mass": m,
        "j": m * d * d / 12,
        "flags": flags,
    }


def make_case(
    motor_id: str,
    table_name: str = "thrust_test",
    prop_id: str | None = None,
    model: str | dict = "n4b",
) -> dict:
    mp = load_part(motor_id)
    model = dict(MODELS[model] if isinstance(model, str) else model)
    tab = thrust_tables(mp)[table_name]
    cols = tab["columns"]
    arr = np.array(tab["rows"], float)
    col = {c: arr[:, i] for i, c in enumerate(cols)}
    m = {
        k: pv(mp, k)
        for k in ("kv", "r_winding", "i0", "mass", "diameter", "j_rotor_bell")
    }
    fl = {
        k: mp["params"][k]["fidelity"]
        for k in ("kv", "r_winding", "i0")
        if k in mp["params"]
    }
    if m["j_rotor_bell"] is None:
        m["j_rotor_bell"] = 0.4 * m["mass"] * (0.45 * m["diameter"]) ** 2
        fl["j_rotor_bell"] = (
            "estimated: 40% of motor mass as a thin shell at 0.45 x outer diameter"
        )
    else:
        fl["j_rotor_bell"] = mp["params"]["j_rotor_bell"]["fidelity"]
    n = len(arr)
    case = {
        "motor": motor_id,
        "table": table_name,
        "model": model,
        "motor_params": m,
        "motor_flags": fl,
        "prop": prop_for(tab, prop_id),
        "cols": cols,
        "thr": col["throttle_pct"] / 100.0,
        "V": col["voltage_V"],
        "obs": {"thrust": col["thrust_N"], "current": col["current_A"]},
        "n": n,
        "train": sorted({0, n // 2, n - 1}),
    }
    if "rpm" in col:
        case["obs"]["rpm"] = col["rpm"]
    # Speed at which the datasheet I0 was measured: no-load speed at the stated test voltage, w0 = (V0 - R I0) / Ke.
    # (No part gives I0 at two voltages, so the one-point split by LOSS_VISC / LOSS_QUAD is the only case.)
    v0 = pv(mp, "i0_voltage", 10.0)
    ke = 60 / (2 * math.pi * m["kv"])
    case["i0_w"] = (v0 - m["r_winding"] * m["i0"]) / ke
    case["i0_voltage"] = v0
    # ESC losses of the stand (zeros = ideal stage): the ESC part's r_on_total / switching_loss_fraction / quiescent_current
    case["esc"] = {"r_on": 0.0, "sw_loss": 0.0, "i_q": 0.0}
    if model.get("esc") and motor_id in ESC_FOR:
        ep = load_part(ESC_FOR[motor_id])
        case["esc"] = {
            "r_on": pv(ep, "r_on_total"),
            "sw_loss": pv(ep, "switching_loss_fraction"),
            "i_q": pv(ep, "quiescent_current"),
        }
    # thermal: ambient and (calibrated per case by calibrate_r_th) winding-to-ambient resistance
    case["t_vendor"] = VENDOR_TEMP_C.get((motor_id, table_name))
    case["r_th"] = 1.0  # K/W placeholder until calibrate_r_th
    case["t_amb"] = T_AMB if model["thermal"] else T_ISO
    # vendor power column: electrical (V I) or shaft (Q w)? pick whichever matches the table
    p = col["power_W"]
    cand = {"electrical": col["voltage_V"] * col["current_A"]}
    if "rpm" in col and "torque_Nm" in col:
        cand["mechanical"] = col["torque_Nm"] * col["rpm"] * 2 * math.pi / 60
    basis = min(cand, key=lambda k: float(np.median(np.abs(cand[k] - p) / p)))
    case["power_basis"] = basis
    case["obs"]["power"] = p
    return case


# ---------------------------------------------------------------- closed-form steady state of the device equations
def steady(theta: dict, case: dict, V, thr, tmp_temp_ok=True) -> dict:
    """Same equations as the ESC + BLDC + rotor devices at J=0, rotor in equilibrium (see docs/models.md)."""
    mo, D, md = case["motor_params"], case["prop"]["diameter"], case["model"]
    kv, R20, i0 = mo["kv"], mo["r_winding"], mo["i0"]
    ke = 60 / (2 * math.pi * kv)
    d = min(max(theta["a"] * thr + theta["b"], 0.0), 1.0)
    t_amb = case["t_amb"]
    ro, sw, iq = case["esc"]["r_on"], case["esc"]["sw_loss"], case["esc"]["i_q"]
    if d <= 0:
        return {
            "omega": 0.0,
            "thrust": 0.0,
            "current": iq,
            "rpm": 0.0,
            "i_m": 0.0,
            "torque": 0.0,
            "duty": 0.0,
            "temp": t_amb,
        }
    cq = theta["cq0"] * RHO0 * D**5 / (4 * math.pi**2)  # Q = cq fQ(x) w^2
    sh = reynolds_shape() if md["re"] else None
    w0, vis, qd = case["i0_w"], md["visc"], md["quad"]

    def fre(w, key):  # Reynolds factor at x = |n| D^2
        return (
            1.0
            if sh is None
            else float(np.interp(abs(w) / (2 * math.pi) * D * D, sh["x"], sh[key]))
        )

    def loss(w):  # motor-shaft loss torque, same law as devices.h MotorCfg::loss
        return (
            ke
            * i0
            * (
                (1 - vis - qd) * math.tanh(w / WS)
                + vis * w / w0
                + qd * w * abs(w) / (w0 * w0)
            )
        )

    def solve(R):  # steady shaft speed at winding resistance R
        def i_m(
            w,
        ):  # PSU series drop on the bus current (d + sw) I_m + i_q; FET r_on in series with the winding
            return (d * V - d * R_SERIES * iq - ke * w) / (
                R + ro + R_SERIES * d * (d + sw)
            )

        def g(w):
            return ke * i_m(w) - loss(w) - cq * fre(w, "fq") * w * w

        lo, hi = 0.0, d * V / ke
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if g(mid) > 0 else (lo, mid)
        w = 0.5 * (lo + hi)
        return w, i_m(w)

    # Winding temperature: fixed point of T = t_amb + r_th I_m^2 R(T) (copper loss only) when the thermal model is on;
    # otherwise the isothermal t_amb. Same equation as the device thermal state at rest (docs/models.md).
    temp = t_amb
    for _ in range(60 if md["thermal"] else 1):
        R = R20 * (1 + ALPHA_CU * (temp - 20.0))
        w, im = solve(R)
        if not md["thermal"]:
            break
        tn = t_amb + case["r_th"] * im * im * R
        if abs(tn - temp) < 1e-9:
            temp = tn
            break
        temp = tn
    R = R20 * (1 + ALPHA_CU * (temp - 20.0))
    w, im = solve(R)
    n = w / (2 * math.pi)
    return {
        "omega": w,
        "rpm": w * 60 / (2 * math.pi),
        "thrust": theta["ct0"] * fre(w, "ft") * RHO0 * n * n * D**4,
        "current": (d + sw) * im + iq,
        "i_m": im,
        "torque": cq * fre(w, "fq") * w * w,
        "duty": d,
        "temp": temp,
    }


def calibrate_r_th(case: dict, th: dict) -> float:
    """r_th so that the model winding at the (training) 100 % row sits at the vendor-printed temperature:
    r_th = (T_vendor - T_amb) / (I_m^2 R(T_vendor)), I_m from the closed form with the fitted theta at T_vendor.
    One number per table from a datasheet temperature (not a free per-row parameter)."""
    tv = case["t_vendor"]
    i = case["n"] - 1
    save = (case["model"], case["r_th"])
    case["model"] = {**case["model"], "thermal": False}
    old_amb = case["t_amb"]
    case["t_amb"] = tv  # isothermal at T_vendor: R(T_vendor), no fixed point
    im = steady(th, case, case["V"][i], case["thr"][i])["i_m"]
    case["model"], case["t_amb"] = save[0], old_amb
    r = case["motor_params"]["r_winding"] * (1 + ALPHA_CU * (tv - 20.0))
    return (tv - case["t_amb"]) / max(im * im * r, 1e-9)


def _resid(x, case, rows):
    th = unpack(x, case)
    out = []
    for i in rows:
        s = steady(th, case, case["V"][i], case["thr"][i])
        for k in ("thrust", "current", "rpm"):
            if k in case["obs"]:
                out.append((max(s[k], 1e-9) - case["obs"][k][i]) / case["obs"][k][i])
    if not has_rpm(case):
        # Without rpm, C_T0 and the duty slope trade off (thrust ~ C_T a^2) and the fit runs away. Weak log-normal prior
        # on the static coefficients of small multirotor props (C_T ~ 0.10, C_Q ~ 0.0065, factor ~1.65 1-sigma).
        out += [(x[0] - math.log(0.10)) / 0.5, (x[1] - math.log(0.0065)) / 0.5]
    return np.array(out)


def has_rpm(case):
    return "rpm" in case["obs"]


def unpack(x, case) -> dict:
    return {
        "ct0": math.exp(x[0]),
        "cq0": math.exp(x[1]),
        "a": x[2],
        "b": x[3] if has_rpm(case) else 0.0,
    }


def fit(case: dict, rows: list) -> dict:
    """Levenberg-Marquardt on relative residuals of thrust, current (and rpm when the table has it) of the TRAIN rows.
    Parameters: ln C_T0, ln C_Q0, duty slope a (+ offset b only when rpm is observed: otherwise a and b are not separable)."""
    npar = 4 if has_rpm(case) else 3
    best = None
    for a0 in (
        0.6,
        0.9,
    ):  # two starts, keep the better: the steady state is mildly non-convex in (a, b)
        x = np.array([math.log(0.1), math.log(0.006), a0, 0.0])[:npar]
        x = np.concatenate([x, np.zeros(4 - npar)])
        lam = 1e-2
        r = _resid(x, case, rows)
        for _ in range(200):
            J = np.empty((len(r), npar))
            for j in range(npar):
                dx = np.zeros(4)
                dx[j] = 1e-6
                J[:, j] = (_resid(x + dx, case, rows) - r) / 1e-6
            A, g = J.T @ J, J.T @ r
            step = np.zeros(4)
            step[:npar] = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
            xn = x + step
            rn = _resid(xn, case, rows)
            if rn @ rn < r @ r:
                done = abs(r @ r - rn @ rn) < 1e-16
                x, r, lam = xn, rn, lam * 0.3
                if done:
                    break
            else:
                lam *= 5
                if lam > 1e12:
                    break
        cost = float(r @ r)
        if best is None or cost < best[0]:
            best = (cost, x)
    th = unpack(best[1], case)
    th["cost"] = best[0]
    return th


# ---------------------------------------------------------------- MJCF stand + run
def motor_cfg(case: dict, t_amb: float, c_th: float, r_th: float) -> dict:
    """MotorParams of the ESC device for this case (loss split from the model switches)."""
    mo, md = case["motor_params"], case["model"]
    m = {
        "kv": mo["kv"],
        "r20": mo["r_winding"],
        "i0": mo["i0"],
        "rotor_inertia": mo["j_rotor_bell"],
        "w_s": WS,
        "c_th": c_th,
        "r_th": r_th,
        "t_amb": t_amb,  # = initial winding temperature
    }
    if md["visc"] or md["quad"]:
        m.update(i0_w=case["i0_w"], i0_visc=md["visc"], i0_quad=md["quad"])
    return m


def esc_cfg(case: dict) -> dict:
    """Esc-table loss fields (schema devices.fbs) of the case: only the nonzero ones, none = the ideal stage."""
    return {k: v for k, v in case["esc"].items() if v}


def stand_mjcf(
    case: dict, theta: dict, volts: float, dt: float = 0.001, temp: float = T_ISO
) -> str:
    """`temp`: winding temperature held for the row (c_th / r_th huge: the stand does not heat up in the 6 s of a row; the
    steady-state temperature comes from the thermal fixed point of steady(), see docs/validation/thrust-stands.md)."""
    mo, pr = case["motor_params"], case["prop"]
    cfg = {
        "psus": [{"name": "psu", "node": 1, "voltage": volts, "r_series": R_SERIES}],
        "escs": [
            {
                "name": "esc",
                "node": 1,
                "joint": "shaft",
                "parts": [case["motor"]],
                "motor": motor_cfg(case, temp, 1e6, 1e6),
                **esc_cfg(case),
            }
        ],
        "rotors": [
            {
                "name": "prop",
                "body": "rotor",
                "joint": "shaft",
                "diameter": pr["diameter"],
                "parts": [pr["id"]] if pr["id"] else [],
                "j": [0.0],
                "ct": [theta["ct0"]],
                "cq": [theta["cq0"]],
                **rotor_re(case),
            }
        ],
    }
    jp = pr["j"]
    return f"""<mujoco model="thrust_stand">
  <option timestep="{dt}" gravity="0 0 0"/>
  <worldbody>
    <body name="stator" pos="0 0 1">
      <geom type="cylinder" size="0.02 0.02" mass="{mo["mass"]}" group="4" contype="0" conaffinity="0"/>
      <body name="rotor">
        <joint name="shaft" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="{pr["mass"]}" diaginertia="{jp} {jp} {jp}"/>
      </body>
    </body>
  </worldbody>
  <custom><text name="napl.devices" data='{json.dumps(cfg)}'/></custom>
</mujoco>
"""


def run_row(
    case: dict, theta: dict, volts: float, thr: float, tmax: float = 6.0
) -> dict:
    """Start at dt = 1 ms (or the friction-stability bound dt < 2 J w_s / (Kt I0), whichever is smaller) and shrink dt
    by 4x on solver divergence (small light props on high-I0 motors)."""
    mo = case["motor_params"]
    kt = 60 / (2 * math.pi * mo["kv"])
    j = mo["j_rotor_bell"] + case["prop"]["j"]
    dt = min(0.001, 0.25 * 2 * j * WS / max(kt * mo["i0"], 1e-9))
    for _ in range(4):
        try:
            return _run_row(case, theta, volts, thr, tmax, dt)
        except RuntimeError as e:
            if "diverged" not in str(e):
                raise
            dt /= 4
    raise RuntimeError(f"stand diverged for {case['motor']} {case['table']} thr={thr}")


def _run_row(case, theta, volts, thr, tmax, dt) -> dict:
    from . import Sim

    d = min(max(theta["a"] * thr + theta["b"], 0.0), 1.0)
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "stand.xml"
        temp = steady(theta, case, volts, thr)["temp"]
        path.write_text(stand_mjcf(case, theta, volts, dt, temp))
        sim = Sim.load(path)
        sim.set("/devices/esc", "throttle", d)
        prev, still, t = 0.0, 0, 0.0
        while t < tmax:
            nst = max(1, round(0.05 / dt))  # 50 ms chunks whatever dt is
            sim.step(nst)
            t += nst * dt
            w = float(sim.query("/devices/prop", "omega")[0][0])
            still = (
                still + 1
                if abs(w - prev) <= 1e-6 * max(abs(w), 1.0) * 5 and t > 0.3
                else 0
            )
            prev = w
            if still >= 4:  # 0.2 s with omega steady to 5e-6 relative
                break
        q = lambda p, a: float(sim.query(p, a)[0][0])
        w = q("/devices/prop", "omega")
        return {
            "converged": bool(still >= 4),
            "t_settle": t,
            "omega": w,
            "rpm": w * 60 / (2 * math.pi),
            "thrust": q("/devices/prop", "thrust"),
            "torque": q("/devices/prop", "torque"),
            "current": q("/devices/psu", "current"),
            "v_psu": q("/devices/psu", "voltage"),
            "duty": q("/devices/esc", "duty"),
            "p_elec": q("/devices/psu", "voltage") * q("/devices/psu", "current"),
            "p_mech": abs(q("/devices/prop", "torque")) * abs(w),
        }


# ---------------------------------------------------------------- bench driver
def bench(
    motor_id: str,
    table: str = "thrust_test",
    prop_id: str | None = None,
    sim_rows: bool = True,
    model: str | dict = "n4b",
) -> dict:
    case = make_case(motor_id, table, prop_id, model)
    th = fit(case, case["train"])
    if case["model"]["thermal"]:
        # r_th from the vendor temperature at the 100 % (training) row, two passes: fit -> r_th -> refit -> r_th.
        for _ in range(2):
            case["r_th"] = calibrate_r_th(case, th)
            th = fit(case, case["train"])
        case["r_th"] = calibrate_r_th(case, th)
    rows, errs = [], {"train": {}, "held_out": {}}
    for i in range(case["n"]):
        split = "train" if i in case["train"] else "held_out"
        ana = steady(th, case, case["V"][i], case["thr"][i])
        obs = {k: float(v[i]) for k, v in case["obs"].items()}
        r = {
            "idx": i,
            "throttle": float(case["thr"][i]),
            "voltage": float(case["V"][i]),
            "split": split,
            "obs": obs,
        }
        if sim_rows:
            s = run_row(case, th, float(case["V"][i]), float(case["thr"][i]))
            r["sim"] = s
            pw = s["p_mech"] if case["power_basis"] == "mechanical" else s["p_elec"]
            pred = {
                "thrust": s["thrust"],
                "current": s["current"],
                "rpm": s["rpm"],
                "power": pw,
            }
        else:
            pred = {
                "thrust": ana["thrust"],
                "current": ana["current"],
                "rpm": ana["rpm"],
                "power": case["V"][i] * ana["current"],
            }
        r["pred"] = {k: pred[k] for k in obs}
        r["rel_err"] = {k: (pred[k] - obs[k]) / obs[k] for k in obs}
        r["analytic"] = {k: ana[k] for k in ("thrust", "current", "rpm", "temp")}
        rows.append(r)
        for k, e in r["rel_err"].items():
            errs[split].setdefault(k, []).append(abs(e))
    summ = {}
    for split in errs:
        summ[split] = {
            k: {"mape": float(np.mean(v)), "max": float(np.max(v)), "n": len(v)}
            for k, v in errs[split].items()
        }
    mo, fl = case["motor_params"], case["motor_flags"]
    return {
        "motor": motor_id,
        "table": table,
        "prop": case["prop"],
        "power_basis": case["power_basis"],
        "datasheet_fixed": {
            k: {"value": mo[k], "fidelity": fl[k]} for k in ("kv", "r_winding", "i0")
        },
        "estimated_fixed": {
            "j_rotor_bell": {
                "value": mo["j_rotor_bell"],
                "fidelity": fl["j_rotor_bell"],
            },
            "j_prop": {
                "value": case["prop"]["j"],
                "fidelity": case["prop"]["flags"]["j_prop"],
            },
        },
        "model": case["model"],
        "thermal": {
            "r_th_K_per_W": case["r_th"] if case["model"]["thermal"] else None,
            "t_vendor_C": case["t_vendor"],
            "t_amb_C": case["t_amb"],
            "temp_C_by_row": [r["analytic"]["temp"] for r in rows],
        },
        "i0_w_rad_s": case["i0_w"],
        "fitted": {
            "params": {k: th[k] for k in ("ct0", "cq0", "a", "b")},
            "fidelity": "fitted",
            "rows": case["train"],
            "rows_throttle_pct": [float(case["thr"][i] * 100) for i in case["train"]],
            "free": ["ct0", "cq0", "a", "b"] if has_rpm(case) else ["ct0", "cq0", "a"],
            "cost": th["cost"],
        },
        "rows": rows,
        "summary": summ,
        "sim_vs_analytic_max_rel_diff": float(
            max(
                max(
                    abs(r["pred"][k] - r["analytic"][k]) / r["analytic"][k]
                    for k in ("thrust", "current", "rpm")
                    if k in r["obs"]
                )
                for r in rows
                if sim_rows
            )
        )
        if sim_rows
        else None,
        "unconverged": [
            r["idx"] for r in rows if sim_rows and not r["sim"]["converged"]
        ],
    }


def _ok(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1, default=float))


def bench_motor(
    motor_id: str, table: str | None = None, prop_id: str | None = None
) -> dict:
    mp = load_part(motor_id)
    tabs = [table] if table else list(thrust_tables(mp))
    bl = [bench(motor_id, t, prop_id) for t in tabs]
    for b in bl:  # same fit protocol, closed-form steady state (the stand matches it to < 1e-3), other model switches
        b["compare"] = {
            name: _held(bench(motor_id, b["table"], prop_id, sim_rows=False, model=md))
            for name, md in COMPARE.items()
        }
    res = {"motor": motor_id, "tables": bl}
    _ok(OUT / f"{motor_id}.json", res)
    return res


# held-out-MAPE comparison variants (all with the same protocol: fit on rows {0, n/2, n-1} only)
COMPARE = {
    "baseline": MODELS["baseline"],
    "re": dict(re=True, visc=0.0, quad=0.0, thermal=False),
    "thermal": dict(re=False, visc=0.0, quad=0.0, thermal=True),
    "n4b": MODELS["n4b"],
}


def _held(b: dict) -> dict:
    return {k: 100 * v["mape"] for k, v in b["summary"]["held_out"].items()}


def esc_compare() -> list:
    """The N4b fit of every ESC_FOR table with and without the ESC loss model (same 3-row protocol; closed-form rows)."""
    out = []
    for m in ESC_FOR:
        for t in thrust_tables(load_part(m)):
            row = {"motor": m, "table": t}
            for name, md in (
                ("ideal", dict(MODELS["n4b"], esc=False)),
                ("esc", MODELS["n4b"]),
            ):
                b = bench(m, t, sim_rows=False, model=md)
                row[name] = {
                    **b["fitted"]["params"],
                    "cost": b["fitted"]["cost"],
                    "r_th": b["thermal"]["r_th_K_per_W"],
                    "held": _held(b),
                }
            row["esc_params"] = make_case(m, t)["esc"]
            row["esc_part"] = ESC_FOR[m]
            out.append(row)
    return out


def esc_markdown(rows: list) -> list:
    out = [
        "",
        "## ESC loss (2216 only): separate from the prop torque coefficient",
        "",
        "The vendor test ESC is not named; the kit's own BLHeli_S 20A part stands in. Its losses are the device fields `r_on` "
        "(FET conduction, in series with the winding), `sw_loss` (switching + dead-time fraction of the motor-side power) and "
        "`i_q` (quiescent bus current), read from the ESC part (all estimated, rationale in the part file; BLHeli_S is "
        "complementary damped-light PWM, so no body-diode freewheel loss). They are fixed, not fitted. Same 3-row fit protocol; "
        "`ideal` = N4b without the ESC loss (the previous adopted model), `with ESC` = adopted. Without the separate loss the "
        "fitted C_Q0 also pays for the ESC; the other tables keep the ideal stage.",
        "",
        "| table | ESC part | r_on ohm | sw_loss | i_q A | variant | C_T0 | C_Q0 | duty a | duty b | fit cost | r_th K/W | held-out thrust / current / rpm MAPE % |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        e = r["esc_params"]
        for name, label in (("ideal", "ideal ESC"), ("esc", "with ESC")):
            v, h = r[name], r[name]["held"]
            ep = e if name == "esc" else {"r_on": 0, "sw_loss": 0, "i_q": 0}
            out.append(
                f"| {r['motor']} {r['table']} | {r['esc_part']} | {ep['r_on']} | {ep['sw_loss']} | {ep['i_q']} | {label} | "
                f"{v['ct0']:.4f} | {v['cq0']:.5f} | {v['a']:.3f} | {v['b']:.3f} | {v['cost']:.2e} | {v['r_th']:.3f} | "
                f"{h['thrust']:.1f} / {h['current']:.1f} / {h.get('rpm', float('nan')):.1f} |"
            )
    return out


def scan_loss_split(grid=LOSS_GRID) -> list:
    """Pooled TRAIN-row cost (sum of the LM cost of every table) of the n4b model for each (visc, quad) share of the datasheet
    I0 torque, with the held-out means alongside for information only (the choice uses the train cost)."""
    out = []
    for v, q in grid:
        md = {**MODELS["n4b"], "visc": v, "quad": q}
        cost, th, cu = 0.0, [], []
        for m in tabled_motors():
            for t in thrust_tables(load_part(m)):
                b = bench(m, t, sim_rows=False, model=md)
                cost += b["fitted"]["cost"]
                h = _held(b)
                th.append(h["thrust"])
                cu.append(h["current"])
        out.append(
            {
                "visc": v,
                "quad": q,
                "train_cost": cost,
                "held_thrust_mean": float(np.mean(th)),
                "held_current_mean": float(np.mean(cu)),
            }
        )
    return out


# ---------------------------------------------------------------- reporting
def fmt_row(b: dict) -> str:
    h, tr = b["summary"]["held_out"], b["summary"]["train"]
    f = lambda d, k: f"{100 * d[k]['mape']:.1f}" if k in d else "-"
    mx = max((v["max"] for v in h.values()), default=float("nan"))
    ho = [r["idx"] for r in b["rows"] if r["split"] == "held_out"]
    prop = b["prop"]["id"] or b["prop"]["label"]
    return (
        f"| {b['motor']} | {b['table'].replace('thrust_test', 'main') or 'main'} | {prop} | {len(b['rows'])} | {len(ho)} | "
        f"{f(h, 'thrust')} | {f(h, 'current')} | {f(h, 'rpm')} | {f(h, 'power')} ({b['power_basis'][:4]}) | {100 * mx:.1f} | "
        f"{', '.join(f'{p}' for p in b['fitted']['rows_throttle_pct'])} |"
    )


HEADER = (
    "| motor | table | prop | rows | held-out | thrust MAPE % | current MAPE % | rpm MAPE % | power MAPE % | max err % | "
    "fit rows (throttle %) |\n|---|---|---|---|---|---|---|---|---|---|---|"
)


def compare_markdown(bl: list, scan: list | None) -> list:
    out = [
        "",
        "## Before / after (held-out MAPE %, same fit protocol: rows first / middle / last only)",
        "",
        "`baseline` = the N4 model (constant static C_T / C_Q, constant-friction I0, isothermal 20 degC). `+Re` = Reynolds "
        "factor only. `+thermal` = thermal steady state only. `n4b` = adopted model (Re + thermal, constant-friction I0). "
        "Baseline / variant columns are the closed-form steady state of the same equations (it matches the simulated stand "
        "to < 1e-3, see `sim_vs_analytic_max_rel_diff` in the JSON); `n4b` in the main table is the simulated stand. "
        "Cells: thrust / current / rpm.",
        "",
        "| motor | table | baseline | +Re | +thermal | n4b |\n|---|---|---|---|---|---|",
    ]
    f = lambda d: "/".join(
        f"{d[k]:.1f}" if k in d else "-" for k in ("thrust", "current", "rpm")
    )
    mean = {n: [] for n in COMPARE}
    for b in bl:
        c = b["compare"]
        for n in COMPARE:
            mean[n].append((c[n]["thrust"], c[n]["current"]))
        out.append(
            f"| {b['motor']} | {b['table'].replace('thrust_test', 'main') or 'main'} | "
            + " | ".join(f(c[n]) for n in COMPARE)
            + " |"
        )
    out.append(
        "| **mean over tables (thrust / current)** | | "
        + " | ".join(
            f"**{np.mean([a for a, _ in mean[n]]):.2f} / {np.mean([c for _, c in mean[n]]):.2f}**"
            for n in COMPARE
        )
        + " |"
    )
    sh = reynolds_shape()
    out += [
        "",
        "## Reynolds / rotor-speed factor (literature shape)",
        "",
        "`C_T(J, n) = C_T(J) f_T(x)`, `C_Q(J, n) = C_Q(J) f_Q(x)`, `x = |n| D^2` (rev/s m^2), piecewise linear, clamped at the "
        "ends (rotor device fields `re_x`, `re_ft`, `re_fq`; absent = off). Re at 0.75 R is `rho (2 pi n) (0.75 R) c / mu`, "
        "so for blades of equal chord ratio c / D in the same air Re is proportional to `n D^2`: a smaller prop needs a "
        "proportionally higher rpm to reach the same Re (the diameter scaling). Assumptions: APC-like section chord ratio and "
        "blade-section Re behaviour for every prop (the T-MOTOR / Gemfan / Holybro props are different blades, so the "
        "table transfers only the generic trend); static (J = 0) data only, so no J-dependence of the Re effect; the "
        "effect is clamped beyond the measured x range; no Mach or flexibility effect. Source: "
        + ", ".join(f"`{p}`" for p in sh["props"])
        + " `[tables.static_ct_cp]` (UIUC Propeller Database vol. 1). Per prop f_i(x) = C(x) / C(x_ref), x_ref = "
        f"{sh['x_ref']} m^2/s; pooled by the median of ln f_i over the props covering x (>= 2), re-normalised to f(x_ref) = 1. "
        "Fitted parameters added: none.",
        "",
        "| x = n D^2 (m^2/s) | f_T | f_Q | props pooled |\n|---|---|---|---|",
    ]
    out += [
        f"| {x:.3f} | {a:.4f} | {c:.4f} | {n} |"
        for x, a, c, n in zip(sh["x"], sh["ft"], sh["fq"], sh["n_props"])
    ]
    out += [
        "",
        "## Thermal (stand at steady winding temperature)",
        "",
        "Used: the device thermal equation at rest, `T = T_amb + r_th I_m^2 R(T)` (copper loss; fixed point with "
        f"`R(T) = R20 (1 + {ALPHA_CU} (T - 20))`, T_amb = {T_AMB} degC), evaluated per row by the closed form; the stand "
        "device is initialised at that T with its own thermal dynamics frozen (c_th = r_th = 1e6), so the stand's R(T) is "
        "the device's. Running each row for minutes instead was not done (6 s rows; the steady state is the same fixed "
        "point). r_th is one number per table: (T_vendor - T_amb) / (I_m^2 R(T_vendor)) with I_m of the fitted model at "
        "the 100 % training row and T_vendor the vendor-printed motor (surface) temperature, transcribed from the part "
        "files into `VENDOR_TEMP_C` (surface temperature taken as winding temperature: the winding is hotter, so r_th "
        "is a lower bound; mn2806's is after 10 min at 80 %). No per-row parameter. The fit is re-run twice with the "
        "calibrated r_th (fit -> r_th -> refit).",
        "",
        "| motor | table | T_vendor degC | r_th K/W | winding T at first / last row degC |\n|---|---|---|---|---|",
    ]
    for b in bl:
        t = b["thermal"]
        ts = t["temp_C_by_row"]
        out.append(
            f"| {b['motor']} | {b['table']} | {t['t_vendor_C']} | {t['r_th_K_per_W']:.3f} | {ts[0]:.0f} / {ts[-1]:.0f} |"
        )
    out += [
        "",
        "## Motor loss law (speed-dependent I0): implemented, rejected by the training rows",
        "",
        "Device: `tau_loss(w) = Kt I0 [(1 - v - q) tanh(w / w_s) + v w / w0 + q w |w| / w0^2]`, `w0 = (V_test - R I0) / Ke` "
        "(shaft speed at the datasheet I0 test voltage), `v` = viscous share (bearing grease, hysteresis iron loss ~ w), `q` = "
        "quadratic share (windage, eddy current ~ w^2); `v = q = 0` is the original Coulomb friction. A one-point datasheet I0 "
        "fixes only the sum, so the split `(v, q)` is a global choice, not a datasheet value; no part gives I0 at a second "
        "voltage, so there is no two-point case. The split was chosen by the pooled TRAIN-row cost of all tables "
        "(re + thermal on), never from held-out rows:",
        "",
    ]
    if scan:
        out += [
            "| viscous share v | quadratic share q | pooled train cost | (info) held-out mean thrust % | (info) held-out mean current % |\n|---|---|---|---|---|"
        ]
        out += [
            f"| {s['visc']} | {s['quad']} | {s['train_cost']:.3f} | {s['held_thrust_mean']:.2f} | {s['held_current_mean']:.2f} |"
            for s in scan
        ]
    best = min(scan, key=lambda s: s["train_cost"]) if scan else None
    out += [
        "",
        f"Adopted: v = {LOSS_VISC}, q = {LOSS_QUAD}."
        + (
            f" Pooled train cost is minimal at v = {best['visc']}, q = {best['quad']}."
            if best
            else ""
        )
        + " Inference, not tested: a speed-growing loss lowers the predicted current at low speed, where the C-series "
        "tables are already under-predicted, so the missing low-throttle current is more likely the ESC / wiring quiescent "
        "draw and the throttle-to-duty offset of the vendor rig (not recorded) than motor iron / bearing loss.",
    ]
    return out


def markdown(
    results: list,
    hover: dict | None = None,
    scan: list | None = None,
    esc: list | None = None,
) -> str:
    bl = [b for r in results for b in r["tables"]]
    out = [
        "# Thrust-stand validation (N4)",
        "",
        "Generated by `napl bench all` (python/napl/bench.py). Do not edit by hand.",
        "",
        "## Method and what is a prediction",
        "",
        "- Stand: welded stator, rotor hinge with prop inertia (+ motor bell as dof armature), one regulated PSU "
        f"(series R {R_SERIES} ohm; vendors test on a regulated supply; voltage set PER ROW from the table so recorded "
        "sag is honoured), ESC + BLDC + rotor devices, still air, ISA sea level (rho 1.225), ground effect off, gravity off, "
        "winding held at the steady-state temperature of each row (thermal section below).",
        "- **Fixed from datasheet (not fitted):** kv, winding R (at 20 degC), I0 (constant friction torque Kt I0, see the "
        "loss-split scan), prop diameter. "
        "Inertias only affect settling time. Estimated inertias are flagged in `build/bench/<motor>.json`.",
        "- **Fixed from literature (not fitted):** the Reynolds / rotor-speed factors f_T(x), f_Q(x), x = n D^2, pooled "
        f"from the static C_T / C_P vs rpm tables of {len(UIUC_STATIC)} UIUC APC props (table and equations below). "
        "C_T0 / C_Q0 are the coefficients at x = " + f"{RE_X_REF} m^2/s.",
        "- **From a datasheet temperature (one number per table, not per row):** the winding-to-ambient resistance "
        "r_th, calibrated so the model winding at the 100 % row equals the vendor-printed motor temperature (below).",
        "- **Fitted** on 3 rows (first, middle, last throttle) by Levenberg-Marquardt: static prop C_T0 and C_Q0 (no "
        "measured coefficients exist for these exact props; `holybro-1045-prop`'s ct_static/cq_static were derived from "
        "the very same table so they are NOT used), and the ESC throttle-to-duty map `duty = a*throttle + b` "
        "(b only when the table has rpm). Fitted values are written to `build/bench/*.json`, never to parts/.",
        "- **Predicted (held-out):** thrust, current, rpm, power at every other row. Training rows are in the JSON "
        "but excluded from the table below. Errors are simulated stand vs vendor table, |sim-vendor|/vendor.",
        "- Power column: electrical V*I where the vendor column equals V*I; the T-MOTOR C-series tables "
        "(2807, 2814, 3110, 4214) list shaft power (matches torque*rpm, not V*I), so those are compared on shaft power.",
        "",
        "## Per-table results (held-out rows only)",
        "",
        HEADER,
    ]
    out += [fmt_row(b) for b in bl]
    out += [
        "",
        "## Fitted parameters (fidelity = fitted, from the train rows listed)",
        "",
        "| motor | table | C_T0 | C_Q0 | duty a | duty b | fit cost |\n|---|---|---|---|---|---|---|",
    ]
    for b in bl:
        p = b["fitted"]["params"]
        out.append(
            f"| {b['motor']} | {b['table']} | {p['ct0']:.4f} | {p['cq0']:.5f} | {p['a']:.3f} | "
            f"{p['b']:.3f}{'' if 'b' in b['fitted']['free'] else ' (fixed 0)'} | {b['fitted']['cost']:.2e} |"
        )
    out += [
        "",
        "## Worst held-out rows",
        "",
        "| motor | table | throttle % | quantity | vendor | sim | error % |\n|---|---|---|---|---|---|---|",
    ]
    for b in bl:
        worst = max(
            (
                (abs(e), r, k)
                for r in b["rows"]
                if r["split"] == "held_out"
                for k, e in r["rel_err"].items()
            ),
            key=lambda t: t[0],
        )
        e, r, k = worst
        out.append(
            f"| {b['motor']} | {b['table']} | {100 * r['throttle']:.0f} | {k} | {r['obs'][k]:.4g} | {r['pred'][k]:.4g} | {100 * r['rel_err'][k]:+.1f} |"
        )
    out += compare_markdown(bl, scan)
    if esc:
        out += esc_markdown(esc)
    out += [
        "",
        "## Fidelity limits and likely causes of error",
        "",
        "- I0 is a constant friction torque (the speed-dependent split is implemented but not supported by the train rows, "
        "see above).",
        "- The Reynolds factor is a generic APC-prop trend (about +5 % C_T from x = 4.5 to 7.6; the 2216 stand covers "
        "x = 4.3 to 10.6), clamped beyond the covered range; "
        "the 2216's own table rises 0.089 to 0.096 (+8 %, mostly below ~7000 rpm), so part of its rpm drift remains in the "
        "held-out residuals (props differ: the nylon / glass-fibre Holybro blade also flexes).",
        "- Winding temperature comes from a vendor SURFACE temperature (a lower bound for the winding) and a copper-loss-only "
        "heat balance; iron loss heating and the ~10 min soak times of the vendor tests are not modelled.",
        "- Tables without an rpm column (xing2 2207 pairs) cannot separate C_T0 from the duty slope; the fit there uses a "
        "weak log-normal prior (C_T ~ 0.10, C_Q ~ 0.0065, factor 1.65) and b fixed 0, so the fitted C_T0/C_Q0/a are not "
        "individually meaningful; only thrust/current predictions are checked.",
        "- ESC losses of the vendor rigs are not recorded. The 2216 table models them separately (estimated BLHeli_S 20A "
        "numbers, section above); the other tables keep an ideal ESC and absorb them, with wire / connector resistance, into "
        "the duty map and C_Q0. Even for the 2216 the vendor's electrical input at hover is not fully explained by shaft torque "
        "+ copper + I0 friction + modelled ESC loss (the unmodelled remainder, PWM-ripple copper / iron loss and the rounding "
        "of the vendor torque column are the candidates; docs/validation/x500-hover.md).",
        "- Vendor table voltages are as printed; where the printed voltage is a no-load figure, sag is unrecorded.",
    ]
    if hover:
        out += ["", hover_markdown(hover)]
    return "\n".join(out) + "\n"


def run_all(motors=None, hover: bool = True) -> list:
    results = [bench_motor(m) for m in (motors or tabled_motors())]
    hv = hover_bench() if hover else None
    if hv:
        _ok(OUT / "hover.json", hv)
    if motors is None:
        scan = scan_loss_split()
        _ok(OUT / "loss_scan.json", scan)
        esc = esc_compare()
        _ok(OUT / "esc_loss.json", esc)
        # summary.md under build/ is the artifact to pull (tools/remote.sh pull src/build/bench/summary.md); DOCS is
        # also written for local runs.
        DOCS.parent.mkdir(parents=True, exist_ok=True)
        DOCS.write_text(markdown(results, hv, scan, esc))
        (OUT / "summary.md").write_text(markdown(results, hv, scan, esc))
    return results


# ---------------------------------------------------------------- X500 V2 hover / endurance
# BOM mass (kg) of the X500 V2 ARF + battery, from the parts DB when the assembly report is absent.
_X500_BOM = {  # part id -> qty
    "holybro-x500v2-frame-kit": 1,
    "holybro-2216-kv920": 4,
    "holybro-x500v2-esc-blheli-s-20a": 4,
    "holybro-1045-prop": 4,
    "tattu-4s-5200mah-35c": 1,
    "holybro-pixhawk-6c": 1,
    "holybro-m8n-gps": 1,
    "holybro-pm02-v3": 1,
    "holybro-pdb-60a": 1,
    "holybro-sik-telemetry-radio-v3": 1,
}


ASM = ROOT / "assemblies" / "x500v2.toml"
ASM_OUT = ROOT / "build" / "assemblies"


def built_assembly() -> tuple[dict, dict]:
    """Build assemblies/x500v2.toml (deterministic, no kernel) -> (report, napl.devices dict). The hover reads AUW,
    battery (OCV table, r0 / r1 / c1, discharge lead) and avionics loads from it, so the parts are the single source."""
    from .design import build

    r = build(ASM, PARTS, ASM_OUT)
    xml = r["xml"].read_text()
    m = re.search(r"<text name=\"napl\.devices\" data='(.*?)'\s*/>", xml, re.DOTALL)
    return r["report"], json.loads(m.group(1))


def auw_estimate(report: dict | None = None) -> tuple[float, str]:
    """All-up mass (kg): the assembly report's `mass.total` (every instance, wire and fastener incl. the avionics; the
    report's `mass` is a dict {total, by_group, by_instance}), else the parts-DB sum below (N4 fallback)."""
    if report is not None:
        return float(
            report["mass"]["total"]
        ), f"{ASM.relative_to(ROOT)} build report mass.total"
    tot, miss = 0.0, []
    for pid, q in _X500_BOM.items():
        try:
            part = load_part(pid)
            # frame kit: Holybro's "Weight 610 g" (frame only; ambiguous, see the part file)
            m = pv(part, "mass", pv(part, "published_weight"))
        except KeyError:
            m = None
        if m is None:
            miss.append(pid)
        else:
            tot += q * m
    tot += 0.060  # wiring, XT60/XT30 connectors, straps, velcro: 60 g, estimated
    return (
        tot,
        f"sum of parts-DB masses ({', '.join(_X500_BOM)}) + 60 g estimated wiring/hardware; no mass for: {miss or 'none'}",
    )


def table_current(thrust_n: float) -> float:
    """Vendor (holybro-2216-kv920 [tables.thrust_test], 16 V) motor current at a thrust, linear interpolation."""
    tab = thrust_tables(load_part("holybro-2216-kv920"))["thrust_test"]
    c = tab["columns"]
    r = np.array(tab["rows"], float)
    return float(
        np.interp(thrust_n, r[:, c.index("thrust_N")], r[:, c.index("current_A")])
    )


def hover_bench(
    auw: float | None = None,
    cutoff_cell_v: float = 3.5,
    dt: float = 0.002,
    tmax_s: float = 3600.0,
) -> dict:
    """Hover of the X500 V2 with the 2216 fit, discharging the assembly's battery model to `cutoff_cell_v` per cell.
    AUW, battery (lipo-cell-ocv-chen2006 x 4, r0 / r1 / c1), the battery discharge lead and the avionics loads come from the
    built assemblies/x500v2.toml (`built_assembly`); motor + prop numbers are the 2216 stand fit. A governor (stand-in for
    the flight controller) holds per-motor thrust = AUW g / 4 by adjusting duty every 0.1 s."""
    from . import Sim

    b2216 = bench("holybro-2216-kv920", sim_rows=False)
    th = b2216["fitted"]["params"]
    case = make_case("holybro-2216-kv920")
    th = {"ct0": th["ct0"], "cq0": th["cq0"], "a": th["a"], "b": th["b"]}
    report, adev = built_assembly()
    if auw is None:
        auw, auw_src = auw_estimate(report)
    else:
        auw_src = "--auw"
    pack = dict(
        adev["batteries"][0], node=1
    )  # OCV table, capacity, r0 / r1 / c1 as the assembly builds them
    cap = pack["capacity_ah"]
    lead = next(w for w in adev["wires"] if w["name"] == "bat-to-pm02")
    wire = dict(lead, name="lead", a=1, b=2)
    # avionics: the assembly's constant-current loads sit on the pm02.power_in node = after the battery lead = bench node 2
    loads = [dict(ld, node=2) for ld in adev["loads"]]
    avionics_a = -sum(ld["i0"] for ld in loads)
    mo, pr = case["motor_params"], case["prop"]
    bodies, escs, rotors = [], [], []
    for k in range(4):
        bodies.append(
            f'<body name="s{k}" pos="{k} 0 1"><geom type="sphere" size="0.01" mass="{mo["mass"]}" contype="0" conaffinity="0"/>'
            f'<body name="r{k}"><joint name="j{k}" type="hinge" axis="0 0 1"/>'
            f'<inertial pos="0 0 0" mass="{pr["mass"]}" diaginertia="{pr["j"]} {pr["j"]} {pr["j"]}"/></body></body>'
        )
        escs.append(
            {
                "name": f"e{k}",
                "node": 2,
                "joint": f"j{k}",
                "parts": [case["motor"]],
                # winding heats with the calibrated r_th; c_th ~ 60 % of the motor mass at 420 J/(kg K) (estimated)
                "motor": motor_cfg(
                    case,
                    T_AMB,
                    0.6 * 420.0 * mo["mass"],
                    b2216["thermal"]["r_th_K_per_W"],
                ),
                **esc_cfg(case),
            }
        )
        rotors.append(
            {
                "name": f"p{k}",
                "body": f"r{k}",
                "joint": f"j{k}",
                "diameter": pr["diameter"],
                "j": [0.0],
                "ct": [th["ct0"]],
                "cq": [th["cq0"]],
                **rotor_re(case),
            }
        )
    cfg = {
        "batteries": [dict(pack, name="pack")],
        "wires": [wire],
        "loads": loads,
        "escs": escs,
        "rotors": rotors,
    }
    xml = (
        f'<mujoco><option timestep="{dt}" gravity="0 0 0"/><worldbody>{"".join(bodies)}</worldbody>'
        f"<custom><text name=\"napl.devices\" data='{json.dumps(cfg)}'/></custom></mujoco>"
    )
    target = auw * 9.80665 / 4
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "hover.xml"
        path.write_text(xml)
        sim = Sim.load(path)
        q = lambda p, a: float(sim.query(p, a)[0][0])
        d, t, trace = 0.5, 0.0, []
        stop = "tmax"
        n_ctl = int(round(0.1 / dt))
        first = None
        t_cut = t_80 = mah_cut = None
        while t < tmax_s:
            for k in range(4):
                sim.set(f"/devices/e{k}", "throttle", d)
            sim.step(n_ctl)
            t += n_ctl * dt
            T = q("/devices/p0", "thrust")
            d = min(max(d * (1 + 0.25 * (target / max(T, 1e-3) - 1)), 0.0), 1.0)
            v = q("/devices/pack", "voltage")
            if t > 2.0 and first is None:
                first = {
                    "throttle_duty": d,
                    "thrust_N": T,
                    "pack_v": v,
                    "pack_i": q("/devices/pack", "current"),
                    "motor_i_bus_A": q("/devices/e0", "bus_current"),
                    "rpm": q("/devices/p0", "omega") * 60 / (2 * math.pi),
                    "motor_p_elec_W": q("/devices/e0", "voltage")
                    * q("/devices/e0", "bus_current"),
                }
            if int(t * 10) % 300 == 0:
                trace.append(
                    {
                        "t_s": round(t, 1),
                        "pack_v": v,
                        "soc": q("/devices/pack", "soc"),
                        "duty": d,
                        "thrust": T,
                    }
                )
            soc_now = q("/devices/pack", "soc")
            if t_cut is None and v < cutoff_cell_v * 4:
                t_cut, mah_cut = t, (1 - soc_now) * cap * 1000
                stop = f"pack terminal voltage < {cutoff_cell_v * 4:.1f} V ({cutoff_cell_v} V/cell, under load)"
            if t_80 is None and soc_now <= 0.2:
                t_80 = t
            if (t_cut is not None and t_80 is not None) or soc_now <= 0 or d >= 1.0:
                break
        soc_end = q("/devices/pack", "soc")
    a, b = th["a"], th["b"]
    return {
        "auw_kg": auw,
        "auw_source": auw_src,
        "target_thrust_per_motor_N": target,
        "avionics_battery_A": avionics_a,
        "avionics_loads": {ld["name"]: -ld["i0"] for ld in loads},
        "ocv_part": "lipo-cell-ocv-chen2006",
        "table_current_A": table_current(first["thrust_N"]) if first else None,
        "cutoff": stop,
        "endurance_min": (t_cut if t_cut is not None else t) / 60,
        "endurance_80pct_dod_min": None if t_80 is None else t_80 / 60,
        "mah_at_cutoff": mah_cut,
        "soc_end": soc_end,
        "mah_used": (1 - soc_end) * cap * 1000,
        "start": first,
        "vendor_equiv_throttle_pct_start": (first["throttle_duty"] - b) / a * 100
        if first
        else None,
        "fit_used": {
            "params": th,
            "fidelity": "fitted (holybro-2216-kv920 thrust_test, rows "
            + str(b2216["fitted"]["rows"])
            + ")",
        },
        "claim_min": 18.0,
        "trace_every_30s": trace,
    }


def hover_markdown(h: dict) -> str:
    s = h["start"]
    return "\n".join(
        [
            "## X500 V2 hover estimate (open loop, 2216 fit)",
            "",
            f"- AUW {h['auw_kg']:.3f} kg ({h['auw_source']}); per-motor thrust {h['target_thrust_per_motor_N']:.2f} N; "
            f"avionics {h['avionics_battery_A']:.3f} A from the battery (the assembly's constant-current loads {h['avionics_loads']}); "
            f"battery OCV {h['ocv_part']} x 4 cells.",
            f"- Start of hover: duty {s['throttle_duty']:.3f} (vendor-equivalent throttle {h['vendor_equiv_throttle_pct_start']:.0f} %), "
            f"{s['rpm']:.0f} rpm, {s['motor_i_bus_A']:.2f} A and {s['motor_p_elec_W']:.1f} W per motor, pack {s['pack_v']:.2f} V, "
            f"pack current {s['pack_i']:.2f} A.",
            f"- Endurance to cutoff: **{h['endurance_min']:.1f} min** ({h['mah_at_cutoff'] or 0:.0f} mAh used; stop: {h['cutoff']}); "
            f"to 80 % depth of discharge (SoC 0.2): **{h['endurance_80pct_dod_min'] or float('nan'):.1f} min**. "
            f'Holybro: "Flight time: ~{h["claim_min"]:.0f} minutes hover with no additional payload. Tested with a 5000mAh Battery." '
            "(docs.holybro.com PX4 Development Kit X500v2, Mechanical Specification; AUW, cutoff voltage and battery model not stated). "
            f"Vendor-table motor current at this thrust {h['table_current_A']:.2f} A (16 V).",
            "- Uses a thrust governor in place of the flight controller; the prediction rests on fitted C_T/C_Q/duty map "
            "(2216 + T1045 table at 16 V), estimated battery R / RC (the assembly's, in the parts DB); it is not validated against a flight log. "
            "The measured comparison with a PX4 flight of the assembly itself is docs/validation/x500-hover.md.",
        ]
    )


# ---------------------------------------------------------------- CLI glue
def add_parser(sub) -> None:
    b = sub.add_parser("bench", help="virtual benches: thrust stand, hover estimate")
    bs = b.add_subparsers(dest="bench_cmd", required=True)
    t = bs.add_parser("thrust", help="reproduce one motor's vendor thrust table")
    t.add_argument("motor")
    t.add_argument("--prop", help="prop part id (default: the table's prop)")
    t.add_argument(
        "--table", help="table name, e.g. thrust_test_5140 (default: all thrust tables)"
    )
    bs.add_parser(
        "all",
        help="every tabled motor + hover; writes build/bench and docs/validation/thrust-stands.md",
    )
    h = bs.add_parser(
        "hover", help="X500 V2 hover throttle/current/power and endurance"
    )
    h.add_argument(
        "--auw",
        type=float,
        help="all-up weight, kg (default: assembly report, else parts-DB sum)",
    )
    h.add_argument("--cutoff-cell-v", type=float, default=3.5)


def main_bench(args) -> int:
    if args.bench_cmd == "thrust":
        res = bench_motor(args.motor, args.table, args.prop)
        print(HEADER)
        print("\n".join(fmt_row(b) for b in res["tables"]))
        print(f"-> {OUT / (args.motor + '.json')}")
    elif args.bench_cmd == "all":
        res = run_all()
        print(HEADER)
        print("\n".join(fmt_row(b) for r in res for b in r["tables"]))
        print(f"-> {DOCS}")
    else:
        h = hover_bench(args.auw, args.cutoff_cell_v)
        _ok(OUT / "hover.json", h)
        print(hover_markdown(h))
    return 0
