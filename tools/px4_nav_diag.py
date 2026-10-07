#!/usr/bin/env python3
"""PX4 navigation-estimate error vs the bridge's true sim state (N5c).

    NAPL_PX4_INSTANCE=3 .venv/bin/python tools/px4_nav_diag.py OUT_DIR [--cfg px4/bridge_x500v2_asm.toml]
        [--model build/assemblies/x500v2.xml] [--mission examples/x500/mission.py] [--param NAME=VALUE ...]

Runs python/napl/px4.Stack, flies the mission program, then compares PX4's ULog (vehicle_global_position = the EKF output, AMSL
altitude; home_position) with the true base-body state in trace.csv (same clock: PX4 lockstep time = bridge sim time). Prints per-axis
error components: offset (mean), slow (std of the 5 s moving average) and fast (residual std), plus max |e|, and the landing
distance from the start point. ULog is converted with `ulog2csv` (pyulog: PX4's own venv, NAPL_PX4_DIR/../venv/bin).
"""

import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))
from napl import px4

R = 6378137.0


def read_csv(p):
    with open(p) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def stats(name, t, e):
    k = max(1, int(5.0 / max(np.median(np.diff(t)), 1e-3)))
    slow = np.convolve(e, np.ones(k) / k, mode="same")
    print(
        f"  {name:<10} offset {e.mean():+7.3f}  slow {slow.std():6.3f}  fast {(e - slow).std():6.3f}  "
        f"rms {np.sqrt((e**2).mean()):6.3f}  max|e| {np.abs(e).max():6.3f}  [m]  n={len(e)}"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--cfg", default=str(REPO / "px4/bridge_x500v2_asm.toml"))
    ap.add_argument("--model", default=str(REPO / "build/assemblies/x500v2.xml"))
    ap.add_argument("--mission", default=str(REPO / "examples/x500/mission.py"))
    ap.add_argument("--param", action="append", default=[])
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    params = dict(p.split("=", 1) for p in a.param)
    params.setdefault("SDLOG_MODE", "1")  # log from boot, not from arm
    st = px4.Stack(a.cfg, out, model=a.model, params=params)
    st.start()
    try:
        env = dict(os.environ, NAPL_MAVLINK=st.mavlink)
        rc = subprocess.run(
            [sys.executable, a.mission], env=env, timeout=600
        ).returncode
    finally:
        st.stop()
    print(f"mission rc={rc}")
    ulg = max((out / "px4run").rglob("*.ulg"))
    csvdir = out / "ulog"
    csvdir.mkdir(exist_ok=True)
    ulog2csv = (
        Path(os.environ.get("NAPL_PX4_DIR", str(px4.PX4_DIR))).parent
        / "venv/bin/ulog2csv"
    )
    subprocess.run(
        [
            str(ulog2csv),
            "-o",
            str(csvdir),
            str(ulg),
        ],
        check=True,
        capture_output=True,
    )
    gp = read_csv(next(csvdir.glob("*vehicle_global_position_0.csv")))
    lp = read_csv(next(csvdir.glob("*vehicle_local_position_0.csv")))
    hp = read_csv(next(csvdir.glob("*home_position_0.csv")))
    tr = read_csv(st.trace)
    cfgt = __import__("tomllib").loads(Path(a.cfg).read_text())["home"]
    lat0, lon0, alt0 = cfgt["lat"], cfgt["lon"], cfgt["alt"]
    # truth at each ULog sample: interpolate the trace on sim time
    ts = tr["t"]

    def truth(t_us):
        t = t_us * 1e-6
        return [np.interp(t, ts, tr[k]) for k in ("x", "y", "z")]

    t = gp["timestamp"] * 1e-6
    x, y, z = truth(
        gp["timestamp"]
    )  # ENU of the base body (z = 0 at the start; the start is z0 above the ground, ignored)
    z0 = tr["z"][0]
    e_n = (gp["lat"] - lat0) * math.pi / 180 * R - y
    e_e = (gp["lon"] - lon0) * math.pi / 180 * R * math.cos(math.radians(lat0)) - x
    e_u = (gp["alt"] - alt0) - (z - z0)
    print(
        f"ULog {ulg.name}: {len(gp['timestamp'])} global-position rows, t {t[0]:.1f}..{t[-1]:.1f} s (trace t {ts[0]:.1f}..{ts[-1]:.1f})"
    )
    print("EKF global position error (estimate - truth):")
    for n, e in (("north", e_n), ("east", e_e), ("up (AMSL)", e_u)):
        stats(n, t, e)
    print(
        f"  horizontal {np.hypot(e_n, e_e).mean():.3f} m mean, {np.percentile(np.hypot(e_n, e_e), 95):.3f} m p95"
    )
    # home position: the last value before takeoff (truth z leaves the ground) vs the true start
    t_up = ts[np.argmax(tr["z"] > z0 + 0.05)]
    k = max(0, int(np.searchsorted(hp["timestamp"] * 1e-6, t_up)) - 1)
    he_n = (hp["lat"][k] - lat0) * math.pi / 180 * R
    he_e = (hp["lon"][k] - lon0) * math.pi / 180 * R * math.cos(math.radians(lat0))
    print(
        f"home_position at takeoff (t={t_up:.1f} s): {he_n:+.2f} m N, {he_e:+.2f} m E, {hp['alt'][k] - alt0:+.2f} m alt"
    )
    print(
        f"local origin: ref_alt - alt0 = {lp['ref_alt'][-1] - alt0:+.3f} m (EKF origin altitude error)"
    )
    print("error vs time (5 s bins: mean N, E, U / max horizontal):")
    for t0 in np.arange(5, t[-1], 5.0):
        m = (t >= t0) & (t < t0 + 5)
        if m.any():
            print(
                f"  t {t0:4.0f}  N {e_n[m].mean():+6.2f} E {e_e[m].mean():+6.2f} U {e_u[m].mean():+6.2f}  "
                f"hmax {np.hypot(e_n[m], e_e[m]).max():5.2f}  truth alt {(z[m].mean() - z0):5.2f}"
            )
    ad = read_csv(next(csvdir.glob("*vehicle_air_data_0.csv")))
    eb = (
        ad["baro_alt_meter"]
        - alt0
        - (np.interp(ad["timestamp"] * 1e-6, ts, tr["z"]) - z0)
    )
    print(
        f"PX4 baro altitude error: first {eb[0]:+.2f} m, last {eb[-1]:+.2f} m, mean {eb.mean():+.2f}, std {eb.std():.2f}, max step {np.abs(np.diff(eb)).max():.2f}"
    )
    # attitude (yaw = heading, incl. the mag declination assumption) vs HIL_STATE_QUATERNION truth
    qa = read_csv(next(csvdir.glob("*vehicle_attitude_0.csv")))
    qg = read_csv(next(csvdir.glob("*vehicle_attitude_groundtruth_0.csv")))

    def yaw(q):
        w, x_, y_, z_ = (q[f"q[{i}]"] for i in range(4))
        return np.arctan2(2 * (w * z_ + x_ * y_), 1 - 2 * (y_**2 + z_**2))

    yg = np.interp(qa["timestamp"], qg["timestamp"], np.unwrap(yaw(qg)))
    ey = np.degrees((yaw(qa) - yg + np.pi) % (2 * np.pi) - np.pi)
    ey = ey[qa["timestamp"] > 20e6]  # after the initial heading alignment
    print(
        f"yaw error after 20 s (est - truth): mean {ey.mean():+.2f} deg, std {ey.std():.2f}, max|e| {np.abs(ey).max():.2f}"
    )
    d = math.hypot(tr["x"][-1] - tr["x"][0], tr["y"][-1] - tr["y"][0])
    print(
        f"truth: max alt {tr['z'].max() - z0:.2f} m, max dist {np.hypot(tr['x'] - tr['x'][0], tr['y'] - tr['y'][0]).max():.2f} m, final dist from start {d:.2f} m"
    )


if __name__ == "__main__":
    main()
