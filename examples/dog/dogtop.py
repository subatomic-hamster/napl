"""Top speed of the dog on flat ground (SP4, spec s7 'flat top speed >= 3.5 m/s'; docs/dog/actuator-study.md: torque-limited trot bound 2.4 m/s).
measure(): stand up, ramp the command at 1 m/s^2 to v_try, hold 4 s; ok when the TRUE mean forward speed over the last 3 s >= 0.95 v_try, no
fall, no driver fault, MPC failures < 5 % of solves. ladder(): v_try = v0, v0 + dv, ... until two consecutive failures; v_top = the last ok.
Diagnostics of every run (the hold window): hip / knee fraction of ticks with |tau_est| >= 0.95 x the driver's tau_ff_max, 99th-percentile |dq| (worst joint) as a
fraction of the no-load speed at the pack voltage, slips, pack peak current (both packs summed), cost of transport (mean pack power / (m g v)).
--search: GRID per gait on TRAIN seed 1001, best by v_top then CoT; --validate N: the best params on N held-out dispersion seeds (21000...:
payload 0-1 kg, IMU biases) -> median / min v_top. Nothing here is called a top speed unless it was measured this way."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse
import json
import math
import multiprocessing as mp
import sys
import tempfile
import xml.etree.ElementTree as ET
from itertools import product
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dogcourse
import dogwalk as W
from loco import controller as Ct

V_TARGET = 2.0  # user decision 2026-10-18 (was 3.5)
ACC, HOLD_S, TRAIN_SEED, TEST_BASE = 1.0, 4.0, 1001, 21000
GRID = {
    "trot": dict(period=(0.26, 0.30, 0.34), duty=(0.5,), apex=(0.06, 0.08), h_frac=(0.78, 0.82)),
    "flying_trot": dict(period=(0.24, 0.28, 0.32), duty=(0.30, 0.35, 0.40), apex=(0.06, 0.08), h_frac=(0.78, 0.82)),
    "bound": dict(period=(0.22, 0.26, 0.30), duty=(0.35, 0.40, 0.45), apex=(0.05, 0.07), h_frac=(0.78, 0.82)),
}  # fmt: skip
PACKS = ("power:pack_a", "power:pack_b")
SWEEP_GAIT, SWEEP_H_FRAC = (
    "flying_trot",
    0.85,
)  # tools/dog_sweep.py --gait dogtop:sweep_gait (WBC; docs/dog/locomotion.md "Top speed")
SWEEP_RUNGS = (1.0, 1.5, 2.0, 2.5)


def _w0(model):
    """(12,) no-load joint speed at 1 V: 1 / (ke_si N), ke_si = 60 / (2 pi kv) (kv in rpm/V as the part file has it)."""
    dev = json.loads(
        ET.parse(model).getroot().find("custom/text[@name='napl.devices']").get("data")
    )
    q = {d["name"]: d for d in dev["qdds"]}
    return np.array([2 * math.pi * q[f"{leg}:{j}"]["motor"]["kv"] / 60.0 / q[f"{leg}:{j}"]["gear_ratio"]
                     for leg in W.LEGS for j in ("abad", "hip", "knee")])  # fmt: skip


def _pack(sim):
    q = [
        sim.query(f"/devices/{p}", a)[0][0]
        for p in PACKS
        for a in ("current", "voltage")
    ]
    return (
        abs(q[0]) + abs(q[2]),
        0.5 * (q[1] + q[3]),
        q[1] * abs(q[0]) + q[3] * abs(q[2]),
    )


def _fly(c, w0, v_try):
    """Drive a set-up Ctx through the ramp-and-hold run; -> result dict (ok, v_mean, diag)."""
    t0 = Ct.T_STANDUP + 0.5
    dur = t0 + Ct.T_WARM + 0.1 + v_try / ACC + HOLD_S
    n, t_hold = int(round(dur / Ct.DT)), dur - HOLD_S
    lim = np.concatenate([L.tau_max for L in c.ctl.legs])
    i_pk, w_pk, vbat, p_sum, n_p, sat = 0.0, [], 0.0, 0.0, 0, np.zeros(12)
    n_s = 0
    for k in range(n):
        t = k * Ct.DT
        W.tick(c, k, (min(v_try, max(0.0, t - t0) * ACC), 0.0, 0.0))
        if c.ctl_fell or c.fell:
            break
        i_tot, v_bus, p_el = _pack(c.sim)
        i_pk = max(i_pk, i_tot)
        if t >= t_hold:
            vbat, p_sum, n_p = vbat + v_bus, p_sum + p_el, n_p + 1
            if k % 5 == 0:
                _, dq, tau = c.hw.joints()
                sat += np.abs(tau) >= 0.95 * lim
                w_pk.append(np.abs(dq))
                n_s += 1
    rows = np.array(c.rows)
    col = W.COLS.index
    late = rows[:, col("t")] > dur - 3.0
    v_mean = float(rows[late, col("vx")].mean()) if late.any() and not c.fell else 0.0
    res = W.result(c, n, 0.0)
    solves = max(1.0, dur / (Ct.DT * Ct.MPC_EVERY))
    ok = (not c.fell and not res["fault"] and not res["diverged"] and v_mean >= 0.95 * v_try
          and res["mpc_fail"] < 0.05 * solves and n_p > 0)  # fmt: skip
    v_bus = vbat / max(n_p, 1)
    sat = sat / max(n_s, 1)
    diag = dict(fell=bool(c.fell), t_end=float(rows[-1, 0]), sat_hip=float(sat[1::3].mean()), sat_knee=float(sat[2::3].mean()),
                w_frac=float((np.percentile(w_pk, 99, axis=0) / (w0 * v_bus)).max()) if w_pk else 0.0, n_slip=int(res["n_slip"]), i_pack_peak=float(i_pk),
                cot=float(p_sum / max(n_p, 1) / (c.ctl.m.mass * 9.81 * max(v_mean, 0.1))), f_track_rms=res["f_track_rms"],
                mpc_fail=int(res["mpc_fail"]), wbc_fail=res["wbc_fail"], fault=res["fault"])  # fmt: skip
    return dict(ok=bool(ok), v_mean=v_mean, diag=diag)


def measure(model, gait, params, v_try, seed=1, leg_ctl="wbc", imu_bias=None, **kw):
    """One flat run of `gait` (forced) with `params` (period / duty / apex of the gait, h_frac of the stand) at v_try."""
    params = dict(params)
    p = dict(W.DEFAULTS, seed=seed, course="flat", gait=gait, leg_ctl=leg_ctl, v_cmd=v_try, h_frac=params.pop("h_frac", None),
             gait_params=params or None, imu_bias=imu_bias)  # fmt: skip
    return _fly(W.setup(model, p, log=False), _w0(model), v_try)


def ladder(model, gait, params, seed=1, leg_ctl="wbc", v0=0.5, dv=0.25, vmax=4.0, **kw):
    runs, best, fails = [], 0.0, 0
    v = v0
    while v <= vmax + 1e-9 and fails < 2:
        r = measure(model, gait, params, v, seed, leg_ctl, **kw)
        runs.append(dict(v_try=v, **r))
        fails = 0 if r["ok"] else fails + 1
        best = v if r["ok"] else best
        v += dv
    return dict(v_top=best, runs=runs)


def _disp(seed):
    rng = np.random.default_rng([int(seed), 4242])
    return dict(payload=(float(rng.uniform(0, 1.0)), float(rng.uniform(-0.03, 0.03)), float(rng.uniform(-0.02, 0.02))),
                imu_bias=(rng.normal(0, 0.005, 3).tolist(), rng.normal(0, 0.05, 3).tolist()))  # fmt: skip


def _job(a):
    """(leg_ctl, gait, params, seed, kind) -> ladder result (kind "train": nominal scene; "test": dispersion of the seed)."""
    leg, gait, params, seed, kind = a
    d = _disp(seed) if kind == "test" else dict(payload=None, imu_bias=None)
    model = dogcourse.scene(
        tempfile.mkdtemp(prefix="dogtop-"), "flat", seed, payload=d["payload"]
    )
    return dict(
        leg_ctl=leg,
        gait=gait,
        params=params,
        seed=seed,
        **ladder(model, gait, params, seed, leg, imu_bias=d["imu_bias"]),
    )


def _grid(gait):
    g = GRID[gait]
    return [dict(zip(g, vals)) for vals in product(*g.values())]


def _cands(s):
    """--params: a JSON dict (one candidate) or a list of dicts (the best of them wins)."""
    x = json.loads(s)
    return x if isinstance(x, list) else [x]


def _best(rows):
    return max(
        rows,
        key=lambda r: (r["v_top"], -r["runs"][-1]["diag"]["cot"] if r["runs"] else 0.0),
    )


def report_md(res):
    v, gap = res["v_top"], V_TARGET - res["v_top"]
    d = res["diag"]
    verdict = "met" if gap <= 0 else f"NOT met (gap {gap:.2f} m/s)"
    val = res.get("validate")
    lines = [f"- flat top speed (measured, {res['gait']}, {res['leg_ctl']}): {v:.2f} m/s"
             + (f"; held-out seeds n = {val['n']}: median {val['median']:.2f}, min {val['min']:.2f} m/s" if val else "; no held-out seeds run"),
             f"- spec target {V_TARGET} m/s: {verdict}",
             "- leg controller: " + ", ".join(f"{k} {x:.2f} m/s" for k, x in res["by_leg"].items()),
             f"- at the first failing speed: hip torque at its limit {100 * d['sat_hip']:.0f} % of ticks, knee {100 * d['sat_knee']:.0f} %; "
             f"joint speed peak {100 * d['w_frac']:.0f} % of no-load speed; {d['n_slip']} slips; pack peak {d['i_pack_peak']:.0f} A; "
             f"cost of transport {d['cot']:.2f}"]  # fmt: skip
    return "\n".join(lines) + "\n"


def sweep_gait(sim, sg, report):
    """SP3b tools/dog_sweep.py --gait slot: SWEEP_GAIT on flat ground on THIS sweep variant (its own model, report["xml"]): a short ladder over
    SWEEP_RUNGS (each rung one fresh run of the ramp-and-hold of measure()), -> the highest rung that was ok (0.0 if none), the TRUE speed."""
    if "xml" not in report:
        raise ValueError(
            "sweep_gait needs report['xml'] (the variant's MJCF path; tools/dog_sweep.py passes it since SP4 Task 10)"
        )
    xml, best = str(report["xml"]), 0.0
    w0 = _w0(xml)
    inf = dict(
        course="flat",
        seed=1,
        nominal=xml,
        start=[0.0, 0.0, 0.0],
        route=None,
        field=None,
    )
    for i, v in enumerate(SWEEP_RUNGS):
        p = dict(W.DEFAULTS, gait=SWEEP_GAIT, v_cmd=v, h_frac=SWEEP_H_FRAC)
        r = _fly(
            W.setup(xml, p, log=False, sim=sim if i == 0 else None, inf=inf), w0, v
        )
        if not r["ok"]:
            break
        best = r["v_mean"]
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--gaits", default="trot,flying_trot,bound")
    ap.add_argument("--leg", default="wbc,jt")
    ap.add_argument(
        "--search",
        action="store_true",
        help="GRID per gait (default: the gait's built-in parameters only)",
    )
    ap.add_argument(
        "--params",
        default="{}",
        help="JSON: params of every non-search run, e.g. '{\"h_frac\": 0.85}'",
    )
    ap.add_argument("--validate", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", default="build/dog-top")
    ap.add_argument(
        "--set",
        default="",
        help="module constants for experiments, e.g. gait.D_MAX:0.22 (inherited by the pool workers)",
    )
    a = ap.parse_args()
    for s in filter(None, a.set.split(",")):
        name, val = s.split(":")
        mod, attr = name.split(".")
        setattr(__import__(f"loco.{mod}", fromlist=[attr]), attr, float(val))
    gaits, legs = a.gaits.split(","), a.leg.split(",")
    todo = [
        (leg, g, prm, TRAIN_SEED, "train")
        for leg in legs
        for g in gaits
        for prm in (_grid(g) if a.search else _cands(a.params))
    ]
    with mp.Pool(a.jobs) as pool:
        rows = pool.map(_job, todo, chunksize=1)
        best = {
            (leg, g): _best([r for r in rows if r["leg_ctl"] == leg and r["gait"] == g])
            for leg in legs
            for g in gaits
        }
        val = {}
        if a.validate:
            vt = [
                (leg, g, b["params"], TEST_BASE + i, "test")
                for (leg, g), b in best.items()
                for i in range(a.validate)
            ]
            for r in pool.map(_job, vt, chunksize=1):
                val.setdefault((r["leg_ctl"], r["gait"]), []).append(r["v_top"])
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    leg0 = "wbc" if "wbc" in legs else legs[0]
    head = max(
        (b for (leg, g), b in best.items() if leg == leg0), key=lambda b: b["v_top"]
    )
    run = head["runs"][-1]
    v = val.get((head["leg_ctl"], head["gait"]))
    res = dict(gait=head["gait"], leg_ctl=head["leg_ctl"], v_top=head["v_top"], params=head["params"],
               by_leg={leg: max(b["v_top"] for (l2, _), b in best.items() if l2 == leg) for leg in legs},
               validate=dict(n=len(v), median=float(np.median(v)), min=float(min(v))) if v else None, diag=run["diag"],
               table=[dict(leg_ctl=leg, gait=g, v_top=b["v_top"], params=b["params"], runs=b["runs"],
                           validate=val.get((leg, g))) for (leg, g), b in best.items()])  # fmt: skip
    (out / "topspeed.json").write_text(json.dumps(res, indent=1))
    lines = [
        report_md(res),
        "| leg | gait | v_top m/s | params | last ok: pack peak A / CoT / f_track | first fail |",
        "|---|---|---|---|---|---|",
    ]
    for t in res["table"]:
        ok = [r for r in t["runs"] if r["ok"]]
        bad = [r for r in t["runs"] if not r["ok"] and r["v_try"] > t["v_top"]]
        o = ok[-1]["diag"] if ok else None
        lines.append(f"| {t['leg_ctl']} | {t['gait']} | {t['v_top']:.2f} | {t['params'] or 'default'} | "
                     + (f"{o['i_pack_peak']:.0f} / {o['cot']:.2f} / {o['f_track_rms']:.3f}" if o else "-") + " | "
                     + (f"{bad[0]['v_try']:.2f} m/s (v {bad[0]['v_mean']:.2f}, fell {bad[0]['diag']['fell']})" if bad else "-") + " |")  # fmt: skip
    (out / "topspeed.md").write_text("\n".join(lines) + "\n")
    sys.stdout.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
