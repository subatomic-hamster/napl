"""Harness of the dog's walk scenarios (examples/dog/walk/*.scenario.toml, `napl program`): owns the Sim, plays the world (ground truth, pushes,
the SP5 stand-ins: pose fix = truth + noise, height map = ground truth), steps loco.controller at 500 Hz (sim.step(2) per tick) and writes
trace.csv / replay.naplog / native.json. The route follower (dogcourse.Follower) steers on the TRUE pose: route planning and localization
are SP5. Env: NAPL_MODEL, NAPL_RUN_DIR, NAPL_PARAMS; argument: the scenario file name."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault(
    "OPENBLAS_NUM_THREADS", "1"
)  # bitwise-repeatable BLAS (golden hashes)

import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dogcourse
import dogtruth
import napl
from loco import controller as Ct
from loco import hw as H
from loco import model as M
from loco import terrain as Tr
from napl import params as nparams
from napl.marine import write_native

SIM_DT, ROW_EVERY = (
    0.001,
    10,
)  # physics 1 kHz (assemblies/dog/dog.toml timestep); one trace row per 10 control ticks (50 Hz)
LEGS = H.LEGS
MODE_CODE = {m: i for i, m in enumerate(Ct.MODES)}
COLS = (["t", "x", "y", "vx", "vy", "v_mean_2s", "est_v_err", "tilt", "clear", "fallen", "ctl_fallen", "fault", "v_bat", "mode",
         "gait", "mu_min", "n_slip", "mpc_fail", "dist_goal"] + [f"c_est_{l}" for l in LEGS] + [f"c_true_{l}" for l in LEGS]
        + ["f_track", "wbc_fail"])  # fmt: skip
GAIT_CODE = {
    g: i for i, g in enumerate(["stand", "walk", "trot", "flying_trot", "bound"])
}
DEFAULTS = dict(
    seed=1,
    course="flat",
    v_cmd=1.0,
    duration=8.0,
    push_t=0.0,
    push_dv=0.0,
    gait="auto",
    pushes=(),
    leg_ctl="wbc",  # "wbc" (the SP4 design, C++ port when built) | "jt" (Jacobian-transpose baseline)
)
IMU = "/devices/compute:imu"


class Ctx:
    """One run in progress: the Sim, the harness-side truth, the controller and the stand-ins."""


def setup(model, p, log=True, audit=False):
    c = Ctx()
    c.p, c.model = p, model
    c.inf = dogcourse.info(model)
    c.field = dogcourse.field_of(c.inf)
    c.sim = napl.Sim.load(str(model))
    if log:
        c.sim.start_log()
    if p.get(
        "imu_bias"
    ):  # sensor dispersion of the MC (harness side: the controller never sees it)
        gb, ab = p["imu_bias"]
        c.sim.set(IMU, "gyro_bias", list(gb))
        c.sim.set(IMU, "accel_bias", list(ab))
    c.tr, c.hw = dogtruth.Truth(c.sim, c.field), H.Hw(c.sim, audit=audit)
    hmap = Tr.FlatMap() if c.field is None else Tr.TruthMap(c.field)
    c.ctl = Ct.Controller(
        M.from_build(c.inf["nominal"]), c.hw, hmap, leg_ctl=p["leg_ctl"]
    )
    st = c.inf.get("start", [0.0, 0.0, 0.0])
    c.ctl.set_pose(st[:2], c.tr.yaw())
    c.fix = dogtruth.PoseFix(c.tr, p["seed"])
    c.fol = None
    c.pushes = list(p.get("pushes") or ()) + (
        [(p["push_t"], (0.0, p["push_dv"], 0.0))] if p.get("push_dv") else []
    )
    c.rows, c.vx_hist = [], deque(maxlen=100)
    c.reached = c.fell = c.ctl_fell = False
    return c


def tick(c, k, cmd):
    """One 500 Hz control tick: stand-ins, pushes, controller step, two physics steps, truth bookkeeping, one trace row per ROW_EVERY."""
    t = k * Ct.DT
    for tp, dv in c.pushes:
        if abs(t - tp) < Ct.DT / 2:
            c.tr.push(np.asarray(dv, float))
    if c.fix.due(k):
        c.ctl.fix(*c.fix.sample())
    c.ctl.step(cmd)
    c.sim.step(2)
    if c.ctl.mode == "STAND" and c.tr.clear0 is None:
        c.tr.arm_fall()
    c.fell = c.fell or c.tr.fallen()
    c.ctl_fell = c.ctl_fell or c.ctl.mode == "FALLEN"
    if k % ROW_EVERY == 0:
        tr, ctl = c.tr, c.ctl
        v, yaw, pos = tr.vel(), tr.yaw(), tr.pos()
        vb = np.array(
            [
                math.cos(yaw) * v[0] + math.sin(yaw) * v[1],
                -math.sin(yaw) * v[0] + math.cos(yaw) * v[1],
            ]
        )
        c.vx_hist.append(vb[0])
        st = ctl.st
        fault, vbat, _ = c.hw.health()
        c.rows.append([t, pos[0], pos[1], vb[0], vb[1], float(np.mean(c.vx_hist)),
                       float(np.linalg.norm(st.v[:2] - v[:2])) if t > 2.0 else 0.0, tr.tilt_deg(), tr.clearance(), float(c.fell),
                       float(c.ctl_fell), fault, vbat, MODE_CODE[ctl.mode], GAIT_CODE[ctl.sched.g.name], float(ctl.mu.min()),
                       ctl.stats["n_slip"], ctl.stats["mpc_fail"], c.fol.dist if c.fol else 0.0]
                      + [float(x) for x in st.contact] + list(tr.contacts()) + [_f_track(c), ctl.stats["wbc_fail"]])  # fmt: skip


def _f_track(c):
    """RMS over the feet that are in true contact AND in the controller's MPC stance of |F_true| - |f_mpc|, per m g (0 if none qualify)."""
    ctl = c.ctl
    ok = (c.tr.contacts() > 0.5) & np.array(ctl._sset)
    if not ok.any():
        return 0.0
    d = c.tr.forces()[ok] - np.linalg.norm(ctl.f, axis=1)[ok]
    return float(np.sqrt(np.mean(d**2)) / (ctl.m.mass * 9.81))


def result(c, n, wall):
    ctl, fol = c.ctl, c.fol
    ft = np.array([r[COLS.index("f_track")] for r in c.rows if r[0] > 3.0])
    rf = len(
        json.loads(
            ET.parse(c.model)
            .getroot()
            .find("custom/text[@name='napl.devices']")
            .get("data")
        ).get("rangefinders", [])
    )
    return dict(reached=bool(c.reached), fallen=bool(c.fell), ctl_fallen=bool(c.ctl_fell),
                fault=bool(max(r[COLS.index("fault")] for r in c.rows) > 0), diverged=bool(c.sim.diverged), t_end=n * Ct.DT,
                dist=fol.length if fol else 0.0, mean_speed=(fol.length / fol.t_done) if fol and fol.done else 0.0,
                n_slip=ctl.stats["n_slip"], mpc_fail=ctl.stats["mpc_fail"], sat_hip=ctl.stats["sat_hip"], rangefinders=rf,
                wbc_fail=int(ctl.stats["wbc_fail"]), f_track_rms=float(np.sqrt(np.mean(ft**2))) if len(ft) else 0.0,
                wall_s=wall, rt_factor=n * Ct.DT / max(wall, 1e-9))  # fmt: skip


def run(model, p, run_dir=None, log=True, audit=False):
    c = setup(model, p, log, audit)
    n = int(round(p["duration"] / Ct.DT))
    t0 = time.perf_counter()
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
        tick(c, k, cmd)
    res = result(c, n, time.perf_counter() - t0)
    if run_dir:
        write_native(c.sim, SIM_DT, COLS, c.rows, run_dir)
    return dict(rows=c.rows, sim=c.sim, ctl=c.ctl, res=res)


def main():
    scen = Path(os.environ.get("NAPL_SCENARIO_DIR", HERE / "walk")) / sys.argv[1]
    p = dict(DEFAULTS, **nparams.values(scen))
    model, run_dir = Path(os.environ["NAPL_MODEL"]), Path(os.environ["NAPL_RUN_DIR"])
    inf = dogcourse.info(model)
    if (inf["course"], inf["seed"]) != (
        p["course"],
        p["seed"],
    ):  # a UI changed the course / seed: build that scene
        model = dogcourse.scene(run_dir / "scene", p["course"], p["seed"])
    r = run(model, p, run_dir)
    (run_dir / "result_native.json").write_text(json.dumps(r["res"]))


if __name__ == "__main__":
    main()
