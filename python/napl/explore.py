"""`napl explore`: what a physics-based, real-parts design can answer that a toy sim cannot.

sweep  swap one instance (or a group, e.g. mot1,mot2,mot3,mot4) for every library candidate that FITS, run the DRC on each
       and the built-in hover (+ endurance) on each, table + JSON + scatter plot.
wind   hover with horizontal position hold under steady wind and Dryden turbulence (seeded), and the wind speed at which
       hover fails.
mc     endurance distribution under part tolerances (N runs, seeded).

Everything runs through `napl.design.Design` on copies of the assembly (the .toml is never written) and the built-in
hover controller of `napl.design.simulate`. Needs the C++ kernel. Parallel runs use policy-admitted spawned workers.
"""

import copy
import json
import os
import re
import struct
import tempfile
import zlib
from pathlib import Path

import numpy as np

from .design import Design, DesignError

HOLYBRO_FLIGHT_MIN = 18.0  # Holybro X500 V2 page: "~18 minutes hover with no additional payload" (5000 mAh battery)
KT = 0.514444  # m/s per knot
# MIL-F-8785C (3.7.3.1) / MIL-HDBK-1797 low-altitude Dryden model: sigma_w = 0.1 W20, W20 = wind speed at 20 ft. The
# turbulence severities usually quoted for it (MathWorks Aerospace Blockset docs, "Dryden Wind Turbulence Model"):
# light 15 kt, moderate 30 kt, severe 45 kt.
TURBULENCE_W20 = {"light": 15 * KT, "moderate": 30 * KT}
CDA_DEFAULT = (
    1.0,
    0.06,
)  # body drag Cd, A (m^2): ESTIMATE, bluff body, ~0.5 m wheelbase x ~0.12 m effective height
XY_TOL, ALT_TOL, SAT_TOL = (
    1.0,
    0.5,
    5.0,
)  # failure thresholds: m excursion, m altitude, % of hold ticks saturated
BATTERY_ASSUMED_CAP_TOL = (
    0.05  # no published capacity tolerance in the library: ASSUMED +-5 %
)


def _prov(v, unit, why):
    return {"value": v, "unit": unit, "fidelity": "estimated", "rationale": why}


def _map(fn, tasks, workers):
    from .runtime import bounded_map
    d = _CTX["d"]
    context = {key: value for key, value in _CTX.items() if key != "d"}
    jobs = ((fn.__name__, str(d.path), str(d.lib.dir), context, task) for task in tasks)
    return bounded_map(_explore_job, jobs, workers=workers)


def _explore_job(job):
    name, assembly, parts, context, task = job
    _CTX.clear()
    _CTX.update(context, d=Design.open(assembly, parts))
    return globals()[name](task)


_CTX = {}  # Per-job scientific context, serialized explicitly into spawned workers.


def _reset(d, doc):
    d.asm.doc = copy.deepcopy(doc)
    d._findings = None
    d._undo.clear()
    d._redo.clear()


# ---------------------------------------------------------------------------------------------------------------- sweep
# Fillers for part params a candidate lacks and the design references (the editing API never invents values).
# param -> (value from (old part, candidate part) | None, rationale)
def _ratio_r1(old, cand):
    r0, r0o, r1o = (
        cand.value("r_internal_pack"),
        old.value("r_internal_pack"),
        old.value("thevenin_r1"),
    )
    return r0 * r1o / r0o if r0 and r0o and r1o else None


FILLERS = {
    "discharge_wire_length": (
        lambda old, cand: old.value("discharge_wire_length"),
        "same lead length as the replaced pack (the candidate's own lead length is not in the library)",
    ),
    "thevenin_r1": (
        _ratio_r1,
        "candidate r_internal_pack x the replaced pack's thevenin_r1 / r_internal_pack ratio (no polarization data for the candidate)",
    ),
    "thevenin_tau": (
        lambda old, cand: old.value("thevenin_tau"),
        "same RC time constant as the replaced pack",
    ),
}


def _plan(d, ids, row):
    """-> (ops, adapter, fixes, None) for a candidate that fits, or (None, None, None, reason).

    Fit filter (the reason a candidate is dropped is kept in the JSON):
      * wires lost by the swap -> dropped (a lost electrical connection changes the circuit).
      * mates lost -> kept only when the lost mate ports can be re-created on the candidate by the box-mount adapter: both
        parts are boxes and the old port is re-seated at the same offset from the top face (the pack hangs on the same rails;
        z shifted by the half-height difference). The adapter is an ASSUMPTION about the mount, reported per row; the DRC
        (INTERFERENCE, CG_OFFSET ...) then judges the geometry.
      * dangling references -> kept only when every one is a param in FILLERS (estimated values, provenance recorded);
        anything else (a lost port, an unknown param) -> dropped.
    """
    old = d._part(d._inst(d.doc, ids[0])["part"])
    cand = d.lib.get(row["part"])
    if row["wires_lost"]:
        return None, None, None, f"wires lost: {row['wires_lost']}"
    adapter = None
    if row["mates_lost"]:
        if old.geometry.get("shape") != "box" or cand.geometry.get("shape") != "box":
            return None, None, None, f"mates lost, no box adapter: {row['mates_lost']}"
        ports = []
        for m in d.doc["mates"]:
            if m["name"] in row["mates_lost"]:
                for end in ("a", "b"):
                    i, _, p = m[end].partition(".")
                    if i == ids[0] and p:
                        ports.append(p)
        dz = cand.geometry["size"][2] - old.geometry["size"][2]
        adapter = [(p, dz) for p in ports]
    ops, fixes = [{"op": "swap", "id": list(ids), "part": cand.id}], []
    for msg in row["dangling"]:
        m = re.search(r"has no param '(\w+)'|'\w+\.(\w+)' no longer resolves", msg)
        name = (m.group(1) or m.group(2)) if m else None
        if name not in FILLERS:
            return None, None, None, f"unresolved: {msg[:160]}"
        fn, why = FILLERS[name]
        v = fn(old, cand)
        if v is None:
            return None, None, None, f"no value for {name}"
        unit = (old.params.get(name) or {}).get("unit", "1")
        for i in ids:
            ops.append(
                {
                    "op": "set",
                    "id": i,
                    "param": name,
                    "value": _prov(v, unit, why),
                }
            )
        fixes.append({"param": name, "value": v, "unit": unit, "why": why})
    return ops, adapter, fixes, None


def _with_adapter(d, ids, cand, adapter):
    """add the re-seated ports to the candidate Part (in this process); returns the restore function."""
    old = d._part(d._inst(d.doc, ids[0])["part"])
    orig = list(cand.ports)
    for p, dz in adapter or []:
        q = copy.deepcopy(old.port(p))
        x, y, z = q["pos"]
        q["pos"] = [x, y, z + dz]
        q["fidelity"] = "estimated"
        q["derivation"] = (
            "explore box-mount adapter: same offset from the top face as the replaced part's port"
        )
        cand.ports.append(q)
    return lambda: cand.ports.__setitem__(slice(None), orig)


def _flight(d, out, **kw):
    r = d.simulate("hover", out=out, **kw)
    return {
        "hover_pass": bool(r["pass"]),
        "auw_kg": r["mass_kg"],
        "thrust_to_weight": r["thrust_to_weight"],
        "hover_throttle": r.get("hover_throttle"),
        "hover_current_A": r.get("hover_current_A"),
        "flight_min": r["endurance_s"] / 60 if "endurance_s" in r else None,
        "reason": r.get("reason"),
    }


def _eval(task):
    row, ops, adapter, fixes, hov = task
    d, ids = _CTX["d"], _CTX["ids"]
    _reset(d, _CTX["base"])
    cand = d.lib.get(row["part"])
    out = {
        "part": cand.id,
        "name": cand.raw.get("name", ""),
        "capacity_ah": cand.value("capacity_ah"),
        "cells": cand.value("cells_series"),
        "mass_delta_kg": row["mass_delta"],
        "adapter": bool(adapter),
        "estimated_fills": fixes,
        "drc_errors": [],
        "drc_warnings": 0,
        "status": "ok",
    }
    undo = _with_adapter(d, ids, cand, adapter)
    try:
        d.edit(ops)
        fs = d.findings()
        out["drc_errors"] = sorted(
            {
                f"{f.code}@{','.join(map(str, f.where))}"
                for f in fs
                if f.severity == "error"
            }
        )
        out["drc_warnings"] = sum(f.severity == "warning" for f in fs)
        with tempfile.TemporaryDirectory(prefix="napl-explore-") as t:
            out.update(_flight(d, t, **hov))
    except DesignError as e:
        out["status"] = f"failed: {str(e)[:200]}"
    finally:
        undo()
    return out


def sweep(
    asm="assemblies/x500v2.toml",
    ids=("bat",),
    parts=None,
    only=None,
    limit=500,
    workers=None,
    hover=None,
):
    """-> {"baseline", "rows", "rejected"}. `only` = candidate part ids to try (default: every suggest() candidate)."""
    d = Design.open(asm, parts)
    ids = list(ids)
    hov = {"hold_s": 10.0, **(hover or {})}
    base_doc = copy.deepcopy(d.doc)
    _CTX.update(d=d, base=base_doc, ids=ids)
    suggestions = d.suggest(ids[0], limit=limit)
    old = d._part(d._inst(d.doc, ids[0])["part"])
    tasks, rejected = [], []
    for row in suggestions:
        if only and row["part"] not in only:
            continue
        ops, adapter, fixes, why = _plan(d, ids, row)
        if why:
            rejected.append({"part": row["part"], "reason": why})
        else:
            tasks.append((row, ops, adapter, fixes, hov))
    tasks.sort(key=lambda t: t[0]["part"])
    with tempfile.TemporaryDirectory(prefix="napl-explore-") as t:
        base = {"part": old.id, **_flight(d, t, **hov)}
    base["drc_errors"] = sorted(f.code for f in d.findings() if f.severity == "error")
    rows = _map(_eval, tasks, workers)
    return {
        "kind": "sweep",
        "asm": str(asm),
        "instances": ids,
        "hover": hov,
        "baseline": base,
        "rows": rows,
        "rejected": sorted(rejected, key=lambda r: r["part"]),
    }


def format_sweep(r) -> str:
    L = [
        f"sweep of {','.join(r['instances'])} in {r['asm']}  (baseline {r['baseline']['part']}: "
        f"AUW {r['baseline']['auw_kg']:.3f} kg, flight {r['baseline']['flight_min'] or float('nan'):.1f} min)",
        "| part | Ah | cells | AUW kg | hover A | flight min | T/W | hover | DRC errors |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    f = lambda v, p=2: "-" if v is None else f"{v:.{p}f}"
    for x in sorted(r["rows"], key=lambda x: -(x.get("flight_min") or -1)):
        err = "; ".join(x["drc_errors"]) or (
            "none" if x["status"] == "ok" else x["status"]
        )
        L.append(
            f"| {x['part']} | {f(x['capacity_ah'])} | {x['cells']} | {f(x.get('auw_kg'), 3)} | {f(x.get('hover_current_A'), 1)} "
            f"| {f(x.get('flight_min'), 1)} | {f(x.get('thrust_to_weight'))} | {'pass' if x.get('hover_pass') else 'FAIL'} | {err} |"
        )
    return "\n".join(L)


def plot_sweep(r, path):
    ok, bad, fail = [], [], []
    for x in r["rows"]:
        if x.get("flight_min") is None:
            continue
        p = (x["auw_kg"], x["flight_min"], x["part"])
        (bad if x["drc_errors"] else fail if not x.get("hover_pass") else ok).append(p)
    series = [
        {"label": "no DRC error", "pts": ok, "color": (30, 140, 60)},
        {"label": "DRC error(s)", "pts": bad, "color": (200, 40, 40)},
        {"label": "hover failed", "pts": fail, "color": (130, 130, 130)},
        {
            "label": f"baseline {r['baseline']['part']}",
            "pts": [(r["baseline"]["auw_kg"], r["baseline"]["flight_min"], "")],
            "color": (20, 60, 200),
        },
    ]
    return _plot(
        path,
        series,
        "all-up weight (kg)",
        "hover flight time to 20 % reserve (min)",
        f"{','.join(r['instances'])} swap sweep",
        hline=(HOLYBRO_FLIGHT_MIN, "Holybro ~18 min"),
        annotate=True,
    )


# ----------------------------------------------------------------------------------------------------------- wind / mc
def _root_id(doc):
    return next(i["id"] for i in doc["instances"] if i.get("root"))


def environment(d, w20=0.0, seed=1, cda=CDA_DEFAULT):
    """Mutate d's doc (a copy): environment device gets Dryden `w20` (m/s), `seed` and ref_body = the root (so the
    turbulence height is the hover height); a body_drag device (cd, area = `cda`, ESTIMATED) is added when the design has none."""
    devs = d.doc.setdefault("devices", [])
    env = next((x for x in devs if x["type"] == "environment"), None)
    if env is None:
        raise DesignError("explore: the design has no environment device")
    root = _root_id(d.doc)
    env.setdefault("set", {})["w20"] = _prov(
        w20, "m/s", "explore: MIL-F-8785C W20 of the scenario (0 = no turbulence)"
    )
    env["seed"] = int(seed)
    env["ref_body"] = root
    if not any(x["type"] == "body_drag" for x in devs):
        devs.append(
            {
                "type": "body_drag",
                "name": "body_drag",
                "body": root,
                "set": {
                    "cd": _prov(
                        cda[0], "1", "explore: bluff-body Cd, no wind-tunnel data"
                    ),
                    "area": _prov(
                        cda[1],
                        "m^2",
                        "explore: effective projected area of plates + arms + battery + legs, estimated from the geometry, no data",
                    ),
                },
            }
        )


def _scenario(task):
    sc, hov = task
    d = _CTX["d"]
    _reset(d, _CTX["base"])
    environment(d, sc.get("w20", 0.0), sc.get("seed", 1), _CTX["cda"])
    with tempfile.TemporaryDirectory(prefix="napl-explore-") as t:
        r = d.simulate(
            "hover",
            out=t,
            wind=[sc.get("wind", 0.0), 0.0, 0.0],
            hold_xy=True,
            **hov,
        )
    keys = (
        "xy_error_max_m",
        "xy_error_rms_m",
        "altitude_error_max_m",
        "tilt_max_deg",
        "saturation_pct",
        "motor_current_max_A",
        "battery_current_max_A",
        "hover_current_A",
        "hover_throttle",
    )
    o = {**sc, "pass": bool(r["pass"]), "reason": r.get("reason")}
    o.update({k: r.get(k) for k in keys})
    o["failed"] = failed(o)
    return o


def failed(o, xy_tol=XY_TOL, alt_tol=ALT_TOL, sat_tol=SAT_TOL):
    """hover failure: lost / diverged / no hold window, or excursion > xy_tol, altitude error > alt_tol, saturation > sat_tol %."""
    if o.get("xy_error_max_m") is None:
        return True
    return bool(
        o["xy_error_max_m"] > xy_tol
        or o["altitude_error_max_m"] > alt_tol
        or o["saturation_pct"] > sat_tol
    )


def wind(
    asm="assemblies/x500v2.toml",
    parts=None,
    speeds=(0.0, 5.0, 10.0),
    seeds=5,
    workers=None,
    hold_s=20.0,
    cda=CDA_DEFAULT,
    vmax=40.0,
    hover=None,
):
    """Scenario table + the steady wind speed at which hover fails (grid every 2.5 m/s then 2 rounds of 5-point refinement)."""
    d = Design.open(asm, parts)
    _CTX.update(d=d, base=copy.deepcopy(d.doc), cda=cda)
    hov = {"hold_s": hold_s, "tol": ALT_TOL, **(hover or {})}
    sc = [{"name": f"steady {s:g} m/s", "wind": float(s), "w20": 0.0} for s in speeds]
    for lvl, w20 in TURBULENCE_W20.items():
        for ws in (0.0, 5.0):
            for sd in range(1, seeds + 1):
                sc.append(
                    {
                        "name": f"{lvl} turbulence" + (f" + {ws:g} m/s" if ws else ""),
                        "wind": ws,
                        "w20": w20,
                        "seed": sd,
                    }
                )
    table = _map(_scenario, [(s, hov) for s in sc], workers)
    grid = [float(v) for v in np.arange(0.0, vmax + 1e-9, 2.5)]
    scan = _map(_scenario, [({"name": "scan", "wind": v}, hov) for v in grid], workers)
    lo, hi = None, None
    for a, b in zip(scan, scan[1:]):
        if not a["failed"] and b["failed"]:
            lo, hi = a["wind"], b["wind"]
            break
    if scan[0]["failed"]:
        lo = hi = 0.0
    for _ in range(2 if lo is not None and hi != lo else 0):
        pts = [lo + (hi - lo) * k / 6 for k in range(1, 6)]
        res = _map(
            _scenario, [({"name": "refine", "wind": v}, hov) for v in pts], workers
        )
        scan += res
        prev = lo
        for v, o in zip(pts, res):
            if o["failed"]:
                hi = v
                break
            prev = v
        lo = prev
    scan.sort(key=lambda o: o["wind"])
    return {
        "kind": "wind",
        "asm": str(asm),
        "hover": hov,
        "cda": {"cd": cda[0], "area_m2": cda[1], "fidelity": "estimated"},
        "thresholds": {
            "xy_m": XY_TOL,
            "altitude_m": ALT_TOL,
            "saturation_pct": SAT_TOL,
        },
        "turbulence_w20_m_s": TURBULENCE_W20,
        "scenarios": table,
        "scan": scan,
        "failure_wind_m_s": None if lo is None else {"last_ok": lo, "first_fail": hi},
    }


def format_wind(r) -> str:
    L = [
        "| scenario | seed | xy err max m | alt err max m | tilt max deg | sat % | motor I max A | pack I max A | fail |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    f = lambda v, p=2: "-" if v is None else f"{v:.{p}f}"
    for o in r["scenarios"]:
        L.append(
            f"| {o['name']} | {o.get('seed', '-')} | {f(o['xy_error_max_m'])} | {f(o['altitude_error_max_m'])} | {f(o['tilt_max_deg'], 1)} "
            f"| {f(o['saturation_pct'], 1)} | {f(o['motor_current_max_A'], 1)} | {f(o['battery_current_max_A'], 1)} | {'FAIL' if o['failed'] else 'ok'} |"
        )
    fw = r["failure_wind_m_s"]
    L.append(
        "\nhover failure wind: "
        + (
            "none up to the scan limit"
            if fw is None
            else f"last ok {fw['last_ok']:.2f} m/s, first fail {fw['first_fail']:.2f} m/s"
        )
    )
    return "\n".join(L)


def plot_wind(r, out_dir):
    s = [o for o in r["scan"] if o.get("xy_error_max_m") is not None]
    paths = []
    for key, yl, name, hl in (
        (
            "xy_error_max_m",
            "position hold error, max (m)",
            "wind-xy-error",
            (XY_TOL, "fail threshold"),
        ),
        ("tilt_max_deg", "max tilt (deg)", "wind-tilt", None),
        (
            "saturation_pct",
            "motor saturation (% of hold ticks)",
            "wind-saturation",
            (SAT_TOL, "fail threshold"),
        ),
    ):
        pts = [(o["wind"], o[key], "") for o in s]
        paths.append(
            _plot(
                Path(out_dir) / f"{name}.png",
                [
                    {
                        "label": "steady wind",
                        "pts": pts,
                        "color": (30, 80, 200),
                        "line": True,
                    }
                ],
                "steady wind (m/s)",
                yl,
                "hover under wind",
                hline=hl,
            )
        )
    return paths


def _mc_run(task):
    i, seed, n_cap_tol, hov = task
    d = _CTX["d"]
    _reset(d, _CTX["base"])
    rng = np.random.default_rng([seed, i])
    log = {}
    for it in _CTX["report"]["instances"]:
        tol = it.get("mass_tol") or 0.0
        if tol > 0 and it.get("body") and it["mass"] > 0:
            m = it["mass"] + rng.uniform(-tol, tol)
            d._inst(d.doc, it["id"]).setdefault("params", {})["mass"] = _prov(
                m, "kg", "Monte Carlo sample within the stated mass tolerance (uniform)"
            )
    bat = next(x for x in d.doc["devices"] if x["type"] == "battery")["instance"]
    p = d._part(d._inst(d.doc, bat)["part"])
    cap = p.value("capacity_ah") * (1 + rng.uniform(-n_cap_tol, n_cap_tol))
    d._inst(d.doc, bat).setdefault("params", {})["capacity_ah"] = _prov(
        cap, "Ah", "Monte Carlo sample, assumed tolerance (no published one)"
    )
    for e in d.doc[
        "instances"
    ]:  # published datasheet tolerances: <p>_tol next to <p> (kv, r_winding)
        pt = d._part(e["part"])
        for k in ("kv", "r_winding"):
            tol = pt.value(f"{k}_tol")
            if tol and pt.value(k):
                e.setdefault("params", {})[k] = _prov(
                    pt.value(k) + rng.uniform(-tol, tol),
                    pt.params[k]["unit"],
                    f"Monte Carlo sample within the datasheet {k}_tol",
                )
    with tempfile.TemporaryDirectory(prefix="napl-explore-") as t:
        r = d.simulate("hover", out=t, **hov)
    return {
        "run": i,
        "capacity_ah": cap,
        "auw_kg": r["mass_kg"],
        "hover_current_A": r.get("hover_current_A"),
        "flight_min": r["endurance_s"] / 60 if "endurance_s" in r else None,
        "pass": bool(r["pass"]),
    }


def montecarlo(
    asm="assemblies/x500v2.toml",
    parts=None,
    n=50,
    seed=1,
    workers=None,
    cap_tol=BATTERY_ASSUMED_CAP_TOL,
    hover=None,
):
    """Endurance distribution (time to the DRC's 20 % reserve at the hover SoC slope) with sampled part tolerances:
    every instance mass uniform within its report `mass_tol` (datasheet value, or the 0.5 x mass default for estimates),
    battery capacity +-`cap_tol` (ASSUMED, none published), motor kv / r_winding +-`<p>_tol` where the part publishes it.
    Run i uses rng [seed, i]: same seed, same numbers, any worker count."""
    d = Design.open(asm, parts)
    with tempfile.TemporaryDirectory(prefix="napl-explore-") as t:
        rep = d.build(t)["report"]
    _CTX.update(d=d, base=copy.deepcopy(d.doc), report=rep)
    hov = {"hold_s": 10.0, **(hover or {})}
    runs = _map(_mc_run, [(i, seed, cap_tol, hov) for i in range(n)], workers)
    ft = np.array([r["flight_min"] for r in runs if r["flight_min"] is not None])
    pc = lambda a: {f"p{q}": float(np.percentile(a, q)) for q in (5, 50, 95)}
    return {
        "kind": "mc",
        "asm": str(asm),
        "n": n,
        "seed": seed,
        "capacity_tol": cap_tol,
        "holybro_flight_min": HOLYBRO_FLIGHT_MIN,
        "flight_min": pc(ft) | {"mean": float(ft.mean()), "std": float(ft.std())},
        "auw_kg": pc([r["auw_kg"] for r in runs]),
        "hover_current_A": pc([r["hover_current_A"] for r in runs]),
        "n_flight_values": len(ft),
        "runs": runs,
    }


def plot_mc(r, path):
    v = np.sort([x["flight_min"] for x in r["runs"] if x["flight_min"] is not None])
    pts = [(float(x), (k + 1) / len(v), "") for k, x in enumerate(v)]
    return _plot(
        path,
        [{"label": "runs", "pts": pts, "color": (30, 80, 200), "line": True}],
        "hover flight time to 20 % reserve (min)",
        "cumulative fraction",
        f"Monte Carlo, N = {r['n']}",
        vline=(HOLYBRO_FLIGHT_MIN, "Holybro ~18 min"),
    )


# -------------------------------------------------------------------------------------------------------------- plotting
def _plot(path, series, xl, yl, title, hline=None, vline=None, annotate=False):
    """series: [{label, pts [(x, y, text)], color (r, g, b), line?}]. matplotlib when importable, else a tiny numpy PNG
    writer (points / polylines, axes box and ticks, NO text: the labels are in the table / JSON)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return _png_plot(path, series, hline, vline)
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    for s in series:
        if not s["pts"]:
            continue
        xs, ys = zip(*[(p[0], p[1]) for p in s["pts"]])
        c = tuple(v / 255 for v in s["color"])
        ax.plot(xs, ys, "-o" if s.get("line") else "o", color=c, label=s["label"], ms=5)
        if annotate:
            for p in s["pts"]:
                ax.annotate(
                    p[2].replace("tattu-", "t-").replace("-xt60", ""),
                    p[:2],
                    fontsize=5,
                    xytext=(2, 2),
                    textcoords="offset points",
                )
    if hline:
        ax.axhline(hline[0], color="k", ls="--", lw=0.8, label=hline[1])
    if vline:
        ax.axvline(vline[0], color="k", ls="--", lw=0.8, label=vline[1])
    ax.set(xlabel=xl, ylabel=yl, title=title)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def _png_plot(path, series, hline, vline, W=640, H=420, M=50):
    img = np.full((H, W, 3), 255, np.uint8)
    pts = [p for s in series for p in s["pts"]]
    if not pts:
        pts = [(0, 0, "")]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    if hline:
        ys.append(hline[0])
    if vline:
        xs.append(vline[0])
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    x1, y1 = x1 if x1 > x0 else x0 + 1, y1 if y1 > y0 else y0 + 1
    px = lambda x: int(M + (x - x0) / (x1 - x0) * (W - 2 * M))
    py = lambda y: int(H - M - (y - y0) / (y1 - y0) * (H - 2 * M))
    img[M, M : W - M] = img[H - M, M : W - M] = 0
    img[M : H - M + 1, M] = img[M : H - M + 1, W - M] = 0
    for k in range(6):  # ticks
        img[H - M : H - M + 5, px(x0 + (x1 - x0) * k / 5)] = 0
        img[py(y0 + (y1 - y0) * k / 5), M - 5 : M] = 0
    if hline:
        img[py(hline[0]), M : W - M : 6] = (90, 90, 90)
    if vline:
        img[M : H - M : 6, px(vline[0])] = (90, 90, 90)
    for s in series:
        c = s["color"]
        P = [(px(p[0]), py(p[1])) for p in s["pts"]]
        if s.get("line"):
            for (a, b), (e, f) in zip(P, P[1:]):
                for t in np.linspace(0, 1, 200):
                    img[int(b + (f - b) * t), int(a + (e - a) * t)] = c
        for a, b in P:
            img[max(b - 3, 0) : b + 4, max(a - 3, 0) : a + 4] = c
    raw = b"".join(b"\x00" + img[r].tobytes() for r in range(H))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )
    return path


# ------------------------------------------------------------------------------------------------------------------- CLI
def add_parser(sub):
    p = sub.add_parser(
        "explore",
        help="design-space sweeps over real parts, wind scenarios, Monte Carlo (needs the kernel)",
    )
    s = p.add_subparsers(dest="explore_cmd", required=True)
    for name, h in (
        (
            "sweep",
            "swap candidates for an instance / group; DRC + hover + endurance each",
        ),
        ("wind", "hover under steady wind and Dryden turbulence; failure wind speed"),
        ("mc", "endurance Monte Carlo with part tolerances"),
    ):
        q = s.add_parser(name, help=h)
        q.add_argument("asm", nargs="?", default="assemblies/x500v2.toml")
        q.add_argument("--parts", default=None)
        q.add_argument("--out", default="build/explore")
        q.add_argument("--workers", type=int, default=None, help="upper bound; default follows the execution policy")
        if name == "sweep":
            q.add_argument(
                "--inst",
                default="bat",
                help="instance id, or a,b,c for a group (default bat)",
            )
            q.add_argument(
                "--only", default=None, help="comma list of candidate part ids"
            )
        if name == "wind":
            q.add_argument("--seeds", type=int, default=5)
            q.add_argument("--speeds", default="0,5,10")
        if name == "mc":
            q.add_argument("--n", type=int, default=50)
            q.add_argument("--seed", type=int, default=1)
    q = s.add_parser(
        "plot",
        help="re-draw the PNG(s) of a result JSON (e.g. with matplotlib on another machine)",
    )
    q.add_argument("json")
    q.add_argument("--out", default=None, help="directory (default: next to the JSON)")


def main_explore(args) -> int:
    out = Path(getattr(args, "out", None) or ".")
    out.mkdir(parents=True, exist_ok=True)
    c = args.explore_cmd
    if c == "plot":
        r = json.loads(Path(args.json).read_text())
        out = Path(args.out or Path(args.json).parent)
        _draw(r, out, Path(args.json).stem)
        return 0
    try:
        if c == "sweep":
            ids = args.inst.split(",")
            r = sweep(
                args.asm,
                ids,
                args.parts,
                args.only.split(",") if args.only else None,
                workers=args.workers,
            )
            stem = "sweep-" + "-".join(ids)
            text = format_sweep(r)
        elif c == "wind":
            r = wind(
                args.asm,
                args.parts,
                [float(x) for x in args.speeds.split(",")],
                args.seeds,
                args.workers,
            )
            stem, text = "wind", format_wind(r)
        else:
            r = montecarlo(args.asm, args.parts, args.n, args.seed, args.workers)
            stem = "mc"
            f = r["flight_min"]
            text = (
                f"Monte Carlo N={r['n']} seed={r['seed']}: flight p5/p50/p95 = {f['p5']:.1f} / {f['p50']:.1f} / {f['p95']:.1f} min "
                f"(Holybro ~{HOLYBRO_FLIGHT_MIN:g}); AUW p5/p50/p95 = {r['auw_kg']['p5']:.3f} / {r['auw_kg']['p50']:.3f} / {r['auw_kg']['p95']:.3f} kg"
            )
    except DesignError as e:
        from .cli import SpecError

        raise SpecError(str(e)) from None
    (out / f"{stem}.json").write_text(json.dumps(r, indent=1))
    (out / f"{stem}.md").write_text(text + "\n")
    print(text)
    _draw(r, out, stem)
    return 0


def _draw(r, out, stem):
    k = r["kind"]
    if k == "sweep":
        plot_sweep(r, out / f"{stem}.png")
    elif k == "wind":
        plot_wind(r, out)
    else:
        plot_mc(r, out / f"{stem}.png")
