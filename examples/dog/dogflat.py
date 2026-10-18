"""SP4 fix round 1, step 2: the Monte Carlo dispersion of dogmc.draw (payload 0-1 kg, IMU bias, two lateral pushes 0.2-0.5 m/s at 4-12 s) on FLAT
ground, and a per-fall cause record.

    python dogflat.py seed=21000 [v=1.0 dur=14 off=push,bias,pay hfrac=0.85 leg_ctl=wbc]

`off` switches parts of the dispersion off (ablation). Prints one `FLATMC` line: the verdict and, for a fall, the ladder-style evidence over the
last second before it (truth vs estimator velocity error, contact estimate vs truth, slips, MPC / WBC failures, the push that preceded it)."""

import dataclasses
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dogcourse as C
import dogmc as MC
import dogwalk as W

c = W.COLS.index


def fall_record(rows, pushes, win=1.0):
    """Evidence for the first fall in `rows` (None if it never fell): what the estimator, the contact estimate and the solvers did in the `win`
    seconds before the truth fall flag, and the nearest earlier push."""
    a = np.asarray(rows, float)
    idx = np.flatnonzero(a[:, c("fallen")] > 0.5)
    if not idx.size:
        return None
    k = int(idx[0])
    t = a[k, c("t")]
    w = a[(a[:, c("t")] > t - win) & (a[:, c("t")] <= t)]
    ce = w[:, [c(f"c_est_{n}") for n in W.LEGS]]
    ct = w[:, [c(f"c_true_{n}") for n in W.LEGS]]
    prior = [(t - tp, np.linalg.norm(d)) for tp, d in pushes if tp <= t]
    dt_push, mag = min(prior) if prior else (np.inf, 0.0)
    return dict(
        t_fall=round(float(t), 2),
        since_push=round(float(dt_push), 2),
        push_mag=round(float(mag), 2),
        est_v_err_max=round(float(w[:, c("est_v_err")].max()), 2),
        contact_mismatch=round(float((ce != ct).mean()), 2),
        slips=int(a[k, c("n_slip")] - a[max(k - len(w), 0), c("n_slip")]),
        mpc_fail=int(a[k, c("mpc_fail")]),
        wbc_fail=int(a[k, c("wbc_fail")]),
        mu_min=round(float(w[:, c("mu_min")].min()), 2),
        tilt_start=round(float(w[0, c("tilt")]), 1),
        tilt_max=round(float(w[:, c("tilt")].max()), 1),
        vx=round(float(a[max(k - len(w), 0), c("vx")]), 2),
    )


def run_truth(model, p, parts):
    """dogwalk.run with the estimator's attitude "R", position "p" and / or velocity "v" replaced by the truth (the diagnostic ladder)."""
    import dogladder as Ld
    from loco import controller as Ct

    cc = Ld.truth_state(W.setup(model, p, False, False), parts)
    n = int(round(p["duration"] / Ct.DT))
    for k in range(n):
        cmd = (
            (p["v_cmd"], 0.0, 0.0)
            if k * Ct.DT > Ct.T_STANDUP + 0.5
            else (0.0, 0.0, 0.0)
        )
        W.tick(cc, k, cmd)
    return dict(rows=cc.rows, res=W.result(cc, n, 0.0))


def flight_frac(rows, t0=3.0):
    """Fraction of trace rows after t0 (and before the first fall) with no foot on the ground (truth): a trot with duty 0.5 has no double support,
    so a late touchdown is a flight."""
    a = np.asarray(rows, float)
    a = a[a[:, c("t")] > t0]
    f = np.flatnonzero(a[:, c("fallen")] > 0.5)
    a = a[: f[0]] if f.size else a
    ct = a[:, [c(f"c_true_{n}") for n in W.LEGS]]
    return round(float((ct.sum(axis=1) == 0).mean()), 3) if len(a) else 0.0


def one(
    seed,
    v=1.0,
    dur=14.0,
    off="",
    hfrac=None,
    leg_ctl="wbc",
    win=None,
    pushes=None,
    truth=(),
):
    """win = (t0, t1): also print the 50 Hz trace rows (state, estimator error, contact estimate vs truth) of that window. pushes = [(t, dv_y)]
    replaces the drawn pushes (a phase sweep)."""
    h = MC.draw(seed, "flat")
    if pushes is not None:
        h["pushes"] = [(t, (0.0, dv, 0.0)) for t, dv in pushes]
    model = C.scene(
        tempfile.mkdtemp(), "flat", seed, payload=None if "pay" in off else h["payload"]
    )
    zero = [0.0] * 3
    p = dict(
        W.DEFAULTS,
        seed=seed,
        course="flat",
        v_cmd=v,
        duration=dur,
        leg_ctl=leg_ctl,
        h_frac=hfrac,
        pushes=[] if "push" in off else h["pushes"],
        imu_bias=(
            zero if {"bias", "gbias"} & set(off.split(",")) else h["gyro_bias"],
            zero if {"bias", "abias"} & set(off.split(",")) else h["accel_bias"],
        ),
    )
    r = W.run(model, p, log=False) if not truth else run_truth(model, p, truth)
    pushes = [] if "push" in off else h["pushes"]
    for row in r["rows"] if win else []:
        if win[0] <= row[c("t")] <= win[1]:
            sys.stdout.write(
                "FLATMC w t %5.2f vx %5.2f vy %5.2f tilt %5.1f clear %.3f est_err %.2f ce %s ct %s gait %d wbc_fail %d\n"
                % (
                    row[c("t")],
                    row[c("vx")],
                    row[c("vy")],
                    row[c("tilt")],
                    row[c("clear")],
                    row[c("est_v_err")],
                    [int(row[c(f"c_est_{n}")]) for n in W.LEGS],
                    [int(row[c(f"c_true_{n}")]) for n in W.LEGS],
                    row[c("gait")],
                    row[c("wbc_fail")],
                )  # fmt: skip
            )
    return dict(
        seed=seed,
        payload=round(h["payload"][0], 2),
        pushes=[(round(t, 1), round(d[1], 2)) for t, d in pushes],
        fallen=bool(r["res"]["fallen"]),
        vx_end=round(float(r["rows"][-1][c("v_mean_2s")]), 2),
        tilt_max=round(max(x[c("tilt")] for x in r["rows"]), 1),
        flight=flight_frac(r["rows"]),
        fall=fall_record(r["rows"], pushes),
    )


if __name__ == "__main__":
    kv = dict(a.split("=", 1) for a in sys.argv[1:])
    for s in filter(None, kv.get("set", "").split(",")):
        # set=gait.K_CAP_BIG:0.8,controller.POSTURE:0.5,gait.trot.duty:0.58 (experiments: module constants, gait fields)
        name, val = s.split(":")
        mod, attr, *fld = name.split(".")
        m = __import__(f"loco.{mod}", fromlist=[attr])
        if fld:
            m.GAITS[attr] = dataclasses.replace(m.GAITS[attr], **{fld[0]: float(val)})
        else:
            setattr(m, attr, float(val))
    out = one(
        int(kv.get("seed", 21000)),
        float(kv.get("v", 1.0)),
        float(kv.get("dur", 14.0)),
        kv.get("off", ""),
        float(kv["hfrac"]) if "hfrac" in kv else None,
        kv.get("leg_ctl", "wbc"),
        tuple(float(x) for x in kv["win"].split(",")) if "win" in kv else None,
        [tuple(float(x) for x in s.split(":")) for s in kv["push"].split(",")]
        if "push" in kv
        else None,
        tuple(kv["truth"].split(",")) if "truth" in kv else (),
    )
    sys.stdout.write("FLATMC [%s] %s\n" % (" ".join(sys.argv[1:]), out))
