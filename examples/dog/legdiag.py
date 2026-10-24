"""Leg study runner (docs/dog/leg-study.md): the SAME evaluation for one dog variant, with the actuator diagnosis logged per control tick.

    python legdiag.py --name base [--asm path/to/variant/dog.toml] --train 10 --flat 16 --top --jobs 4 --out ../logs/leg-base

Tasks (one pool, 4 jobs): terrain Monte Carlo runs (dogmc.draw dispersion, dogmc.FAMILIES, seeds BASE..BASE+n-1 per family), flat push dispersion
(dogflat.one: dogmc.draw on flat ground, 1.0 m/s, 14 s, seeds 21000...), and the flat top-speed ladder (dogtop.ladder: flying trot, WBC, stand height 0.85,
seed 1). The variant is selected by NAPL_DOG_ASM (dogcourse hook); the controller is the napl-v1 controller as it is, so every variant flies the same code.
The run loop is dogwalk.run's with two differences that do not change a verdict: it stops at the first truth fall (fallen is sticky), and it records per
tick the joint torque / speed / angle (the controller's own Hw.joints() reads, wrapped), the pack current and voltage, the trunk tilt and the controller's
counters. Verdicts are dogmc's (ground truth).

Per run, for terrain / flat runs that fell: three windows ending at the truth fall time t_f and at the collapse onset t_on (the last tick with tilt < 15 deg
before t_f, at most 3 s before it): W_on = the 0.5 s before t_on (what the legs did BEFORE the trunk started to go: cause), W_1 = the 0.5 s before t_f (what the
legs did while going: cause or consequence), and the run's baseline (t > 3 s up to t_on - 0.5). Per window: fraction of control ticks with any joint at 95 % of
the torque the driver can deliver at that tick's joint speed and bus voltage (avail(): the Qdd law, so torque limit and speed limit in one number; per joint
class abad / hip / knee), with any joint above its corner speed (where the voltage starts to cut the torque), with any joint within 0.03 rad of a range stop;
peak |tau|; pack current peak and fraction of ticks above the 120 A BMS limit; slip, WBC relaxation, WBC failure and MPC failure counts. Classification of a
fall (thresholds are ours, stated here): actuator-limited when W_on has torque saturation on >= 10 % of ticks or pack current > 120 A on >= 5 %; else
kinematic when W_on has a range stop on >= 10 %; else control / planning."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse
import json
import math
from napl.runtime import BatchPool
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dogcourse
import dogmc
import dogtop
import dogwalk as W
from loco import controller as Ct

BMS_A, TILT_ON, SAT, VEL, RNG = 120.0, 15.0, 0.95, 0.80, 0.03
FLAT_BASE, FLAT_V, FLAT_DUR = 21000, 1.0, 14.0
CLASSES = ("abad", "hip", "knee")


def limits(model):
    """Per-joint (12,) actuator constants of the built dog: tau_ctl = the controller's tau_ff_max clip; ke = Kt (V s/rad, motor side); n = gear ratio
    (joint side); eta; i_lim (A); r (ohm, line-to-line r20 as the Qdd model uses it)."""
    dev = json.loads(
        ET.parse(model).getroot().find("custom/text[@name='napl.devices']").get("data")
    )
    q = {d["name"]: d for d in dev["qdds"]}
    names = [f"{leg}:{j}" for leg in W.LEGS for j in CLASSES]
    g = lambda f: np.array([f(q[n]) for n in names], float)
    return dict(tau_ctl=g(lambda d: d["tau_ff_max"]), ke=g(lambda d: 60.0 / (2 * math.pi * d["motor"]["kv"])), n=g(lambda d: d["gear_ratio"]),
                eta=g(lambda d: d["efficiency"]), i_lim=g(lambda d: d["i_limit"]), r=g(lambda d: d["motor"]["r20"]))  # fmt: skip


def avail(L, dq, v, tau):
    """Torque the driver can deliver at joint speed dq (rad/s), bus v (V), in the direction of tau (the Qdd law of src/mujoco/devices/qdd.cpp, cold winding):
    Kt N k min(i_limit, (v - Ke |w_motor|) / R) while the motor drives (k = eta), Kt N i_limit while it is back-driven (k = 1, the voltage helps)."""
    drive = tau * dq >= 0.0
    i = np.where(
        drive,
        np.minimum(
            L["i_lim"],
            np.maximum(v[:, None] - L["ke"] * L["n"] * np.abs(dq), 0.0) / L["r"],
        ),
        L["i_lim"],
    )
    return L["ke"] * L["n"] * np.where(drive, L["eta"], 1.0) * i


def fly(model, p):
    """dogwalk.run's loop with per-tick records; stops at the first truth fall. -> (c, k_end, rec) with rec = dict of arrays over ticks."""
    c = W.setup(model, p, log=False)
    n = int(round(p["duration"] / Ct.DT))
    last = {}
    orig = c.hw.joints

    def joints():
        last["j"] = orig()
        return last["j"]

    c.hw.joints = joints
    q, dq, tau = (np.zeros((n, 12), np.float32) for _ in range(3))
    t, i_tot, v_bus, tilt, slip, relax, wbc_f, mpc_f = (np.zeros(n) for _ in range(8))
    k_end, t0 = n, time.perf_counter()
    for k in range(n):
        if c.fol:
            cmd = c.fol.cmd(c.tr.pos(), c.tr.yaw())
            c.reached = c.reached or c.fol.done
        else:
            cmd = (
                (p["v_cmd"], 0.0, 0.0)
                if k * Ct.DT > Ct.T_STANDUP + 0.5
                else (0.0, 0.0, 0.0)
            )
        W.tick(c, k, cmd)
        q[k], dq[k], tau[k] = last["j"]
        i_tot[k], v_bus[k], _ = dogtop._pack(c.sim)
        t[k], tilt[k] = k * Ct.DT, c.tr.tilt_deg()
        st = c.ctl.stats
        slip[k], wbc_f[k], mpc_f[k] = st["n_slip"], st["wbc_fail"], st["mpc_fail"]
        relax[k] = getattr(c.ctl.wbc, "n_relaxed", 0) if c.ctl.wbc else 0
        if c.fell or c.sim.diverged:
            k_end = k + 1
            break
    sl = slice(0, k_end)
    rec = dict(t=t[sl], q=q[sl], dq=dq[sl], tau=tau[sl], i=i_tot[sl], v=v_bus[sl], tilt=tilt[sl], slip=slip[sl], relax=relax[sl], wbc_fail=wbc_f[sl],
               mpc_fail=mpc_f[sl], wall=time.perf_counter() - t0)  # fmt: skip
    return c, k_end, rec


def window(rec, L, qlo, qhi, a, b):
    """Statistics of the control ticks with a <= t < b (s); None for an empty window. sat = |tau_est| >= 95 % of avail() at the present speed and bus voltage
    (torque and speed limit in one: above the corner speed (v - R i_lim) / (Ke N) the voltage starts to cut the torque); vel = joint speed above that corner;
    rng = within RNG rad of a range stop. The _any fields are per control tick (any of the 12 joints); sat / vel / rng also per joint class abad, hip, knee."""
    m = (rec["t"] >= a) & (rec["t"] < b)
    if m.sum() < 5:
        return None
    tau, dq, q, v = rec["tau"][m], rec["dq"][m], rec["q"][m], rec["v"][m]
    sat = np.abs(tau) >= SAT * avail(L, dq, v, tau)
    vel = np.abs(dq) >= (v[:, None] - L["r"] * L["i_lim"]) / (L["ke"] * L["n"])
    rng = (q <= qlo + RNG) | (q >= qhi - RNG)
    cls = lambda x: [float(x[:, k::3].mean()) for k in range(3)]
    i = rec["i"][m]
    d = lambda key: float(rec[key][m][-1] - rec[key][m][0])
    cap = L["ke"] * L["n"] * L["i_lim"]  # the hard torque cap (back-driven, k = 1)
    return dict(n=int(m.sum()), sat_any=float(sat.any(1).mean()), sat=cls(sat), vel_any=float(vel.any(1).mean()), vel=cls(vel), rng_any=float(rng.any(1).mean()),
                rng=cls(rng), tau_peak=[float(np.abs(tau)[:, k::3].max()) for k in range(3)], tau_peak_frac_cap=[float((np.abs(tau) / cap)[:, k::3].max()) for k in range(3)],
                i_peak=float(i.max()), i_over_bms=float((i > BMS_A).mean()), slips=int(d("slip")), relaxed=int(d("relax")), wbc_fail=int(d("wbc_fail")),
                mpc_fail=int(d("mpc_fail")), tilt_max=float(rec["tilt"][m].max()))  # fmt: skip


def diagnose(c, k_end, rec, L):
    """Windows and the fall classification (module docstring). Runs that did not fall: the baseline over t > 3 s only."""
    qlo = np.concatenate([L.q_lo for L in c.ctl.legs])
    qhi = np.concatenate([L.q_hi for L in c.ctl.legs])
    t = rec["t"]
    out = dict(
        cap=[float(x) for x in (L["ke"] * L["n"] * L["i_lim"])[:3]],
        stall_drive=[float(x) for x in (L["ke"] * L["n"] * L["eta"] * L["i_lim"])[:3]],
        lim_ctl=[float(x) for x in L["tau_ctl"][:3]],
        i_peak_run=float(rec["i"].max()),
    )
    if not c.fell:
        out["base"] = window(rec, L, qlo, qhi, 3.0, 1e9)
        return out
    t_f = float(t[k_end - 1])
    lo = np.flatnonzero(rec["tilt"][: k_end - 1] < TILT_ON)
    t_on = max(float(t[lo[-1]]) if lo.size else 0.0, t_f - 3.0)
    wn = lambda a, b: window(rec, L, qlo, qhi, a, b)
    out.update(
        t_fall=t_f,
        t_on=t_on,
        w_on=wn(t_on - 0.5, t_on),
        w_1=wn(t_f - 0.5, t_f),
        base=wn(3.0, t_on - 0.5),
    )
    w = out["w_on"]
    out["cls"] = "undetermined" if w is None else (
        "actuator" if (w["sat_any"] >= 0.10 or w["i_over_bms"] >= 0.05) else "kinematic" if w["rng_any"] >= 0.10 else "control")  # fmt: skip
    return out


def terrain(seed, fam, h_frac=None):
    """dogmc.one with the diagnosis; -> dogmc-shaped row + diag."""
    course, v = dogmc.FAMILIES[fam]
    h = dogmc.draw(seed, fam)
    out = Path(
        tempfile.mkdtemp(
            dir=dogmc.SCRATCH if dogmc.SCRATCH.exists() else None,
            prefix=f"w{os.getpid()}-",
        )
    )
    try:
        model = dogcourse.scene(out, course, seed, payload=h["payload"])
    except RuntimeError as e:
        return dict(seed=int(seed), fam=fam, leg_ctl="wbc", skipped=str(e))
    length = dogmc.route_length(model)
    p = dict(W.DEFAULTS, seed=seed, course=course, v_cmd=v, duration=3.0 + 2.5 * length / v, pushes=h["pushes"], imu_bias=(h["gyro_bias"], h["accel_bias"]),
             leg_ctl="wbc", h_frac=h_frac)  # fmt: skip
    return _row(model, p, seed, fam, length)


def flat(seed, h_frac=None):
    """dogflat.one's scene and dispersion (1.0 m/s, 14 s), with the diagnosis."""
    h = dogmc.draw(seed, "flat")
    model = dogcourse.scene(tempfile.mkdtemp(), "flat", seed, payload=h["payload"])
    p = dict(W.DEFAULTS, seed=seed, course="flat", v_cmd=FLAT_V, duration=FLAT_DUR, leg_ctl="wbc", h_frac=h_frac, pushes=h["pushes"],
             imu_bias=(h["gyro_bias"], h["accel_bias"]))  # fmt: skip
    return _row(model, p, seed, "flat", 0.0)


def _row(model, p, seed, fam, length):
    c, k_end, rec = fly(model, p)
    res = W.result(c, k_end, rec["wall"])
    L = limits(model)
    dg = [r[W.COLS.index("dist_goal")] for r in c.rows]
    return dict(
        seed=int(seed),
        fam=fam,
        leg_ctl="wbc",
        route_length=length,
        **res,
        t_stop=float(rec["t"][-1]),
        prog=float(1 - dg[-1] / dg[0])
        if dg and dg[0] > 0
        else 0.0,  # fraction of the route covered when the run stopped (terrain runs)
        diag=diagnose(c, k_end, rec, L),
    )


def top(h_frac=dogtop.SWEEP_H_FRAC):
    """dogtop's training-seed ladder of the forced flying trot (the SWEEP_GAIT of tools/dog_sweep.py), WBC, stand height 0.85."""
    r = dogtop._job(
        ("wbc", dogtop.SWEEP_GAIT, {"h_frac": h_frac}, dogtop.TRAIN_SEED, "train", [])
    )
    return dict(fam="top", seed=dogtop.TRAIN_SEED, v_top=r["v_top"], runs=r["runs"])


def task(a):
    kind, *rest = a
    try:
        return {"terrain": terrain, "flat": flat, "top": top}[kind](*rest)
    except Exception as e:  # a variant that cannot stand etc.: recorded, not fatal
        return dict(
            fam=rest[1] if kind == "terrain" else kind,
            seed=rest[0] if rest else 0,
            error=f"{type(e).__name__}: {e}",
        )


def ident(a):
    """(fam, seed) of a task tuple, the same as the row it produces."""
    return (
        (a[2], a[1])
        if a[0] == "terrain"
        else ("top", dogtop.TRAIN_SEED)
        if a[0] == "top"
        else ("flat", a[1])
    )


def run_all(name, asm, tasks, jobs, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if asm:
        os.environ[dogcourse.ASM_ENV] = str(Path(asm).resolve())
    part = out / f"{name}.jsonl"
    rows = (
        [json.loads(x) for x in part.read_text().splitlines()] if part.exists() else []
    )
    have = {(r["fam"], r["seed"]) for r in rows if "error" not in r}
    todo = [a for a in tasks if ident(a) not in have]
    with BatchPool(jobs) as pool, part.open("a") as fh:
        for r in pool.imap_unordered(task, todo, chunksize=1):
            rows.append(r)
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            sys.stdout.write(
                f"{name} {r.get('fam')} {r.get('seed')} "
                + (
                    "ERR " + r["error"]
                    if "error" in r
                    else "fallen=%s reached=%s" % (r.get("fallen"), r.get("reached"))
                    if "fallen" in r
                    else "v_top=%s" % r.get("v_top")
                )
                + "\n"
            )
            sys.stdout.flush()
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--name", required=True)
    ap.add_argument(
        "--asm", default="", help="variant dog.toml (default: assemblies/dog)"
    )
    ap.add_argument("--families", default="rubble,slopes,scree,ruins")
    ap.add_argument(
        "--base", type=int, default=dogmc.TRAIN_BASE, help="first terrain seed"
    )
    ap.add_argument("--train", type=int, default=0, help="terrain runs per family")
    ap.add_argument(
        "--flat", type=int, default=0, help="flat push runs, seeds 21000..."
    )
    ap.add_argument("--top", action="store_true")
    ap.add_argument("--jobs", type=int, default=None, help="upper bound; default follows the execution policy")
    ap.add_argument("--out", default="build/leg-study")
    a = ap.parse_args()
    tasks = [("top",)] if a.top else []
    tasks += [
        ("terrain", a.base + i, f)
        for f in a.families.split(",")
        for i in range(a.train)
    ]
    tasks += [("flat", FLAT_BASE + i) for i in range(a.flat)]
    run_all(a.name, a.asm, tasks, a.jobs, a.out)


if __name__ == "__main__":
    main()
