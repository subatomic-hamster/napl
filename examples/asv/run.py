"""ASV scenario program: the harness of the asv-*.scenario.toml files. Owns the Sim, plays the world around the autopilot (samples the IMU / magnetometer / GNSS / lidar
devices, runs the scripted traffic and its AIS), steps the autonomy stack and writes trace.csv / replay.naplog / native.json (napl.marine.write_native), telemetry.jsonl
(the HUD / C2 stream, napl.params contract, schema in examples/asv/README.md) and result.json.

The autonomy (asv.estimator -> asv.perception -> asv.planner -> asv.control) sees ONLY sensor outputs: gyro, accelerometer, magnetometer, GNSS lat / lon / velocity, the
lidar scan and received AIS reports. The true state is read here for the trace, the telemetry `truth` block and the scripted traffic only.

usage: python run.py <kind>     kind: patrol | head_on | crossing_give_way | crossing_stand_on | overtaking
Env: NAPL_MODEL, NAPL_RUN_DIR, NAPL_SCENARIO_DIR (set by `napl program`), NAPL_PARAMS (JSON overrides of the scenario's [[parameters]]).
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1])
)  # examples/ : the package `asv`

import napl
from napl import params as nparams
from napl.marine import write_native

from asv import ais as ais_mod
from asv import colreg as cr
from asv import encounters as enc
from asv import judge
from asv.control import Autopilot, Gains, Route
from asv.estimator import Navigator
from asv.geo import (
    LAT0,
    LON0,
    heading_deg,
    latlon_from_enu,
    wrap,
    yaw_of_quat,
)
from asv.perception import Tracker, clusters, scan_points
from asv.planner import Planner, steer
from asv.scene import build_spec
from asv.traffic import TrafficVessel

DT = 0.002  # s, simulation step
IMU_TICKS = 4  # the estimator consumes IMU / magnetometer samples at 125 Hz (the devices run at 250 / 50 Hz)
CTRL_TICKS = 20  # 40 ms control period (25 Hz)
STATIC_MEM_S = 3.0  # s, how long a lidar return of a fixed object is remembered (in the world frame, from the estimated pose)
PLAN_EVERY = 12  # control periods between plans (0.48 s)
ROW_EVERY = 3  # trace row every 120 ms
TLM_EVERY = 5  # telemetry record every 200 ms
T_CTRL = 0.3  # s: the autopilot holds the heading from here (the first IMU / magnetometer / GNSS samples exist)
MAG_REF = np.array(
    [1.294e-6, 2.1756e-5, -4.3027e-5]
)  # the local field (World Magnetic Model value PX4 uses at the origin; assemblies/asv.toml)
KN_COLS = ["t", "x_m", "y_m", "heading_deg", "speed_ms", "est_pos_err_m", "est_hdg_err_deg", "xte_m", "thr_port", "thr_stbd", "range_t1_m", "cpa_min_m",
           "collision", "static_clear_m", "mission_done", "n_moving", "rule", "delta_deg", "colreg_ok", "side_ok", "cpa_ok", "pass_cpa_m", "soc"]  # fmt: skip
RULE_CODE = {
    "none": 0,
    "head_on": 1,
    "overtaking": 2,
    "crossing_give_way": 3,
    "crossing_stand_on": 4,
    "overtaken": 5,
}


def _q(sim, path, attr):
    return sim.query(path, attr)[0]


def fly(kind, p, run_dir, model=None, telemetry=True, gains=None):
    """One run of encounter `kind` with resolved parameters p (seed, hs, jitter, ais, traffic_complies, visual, speed_scale). -> dict(result, rows, sim)."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    spec = enc.make(
        kind,
        p["seed"],
        p["hs"],
        p["jitter"],
        bool(p["ais"]),
        bool(p["traffic_complies"]),
    )
    if p.get("t_stop", 0.0) > 0.0:
        spec.t_end = min(spec.t_end, float(p["t_stop"]))  # a shortened run (tests)
    if model is None:
        model = str(
            build_spec(
                run_dir / "model", spec, seed=p["seed"], visual=bool(p["visual"])
            )
        )
    sim = napl.Sim.load(str(model))
    sim.start_log()  # before any command: the replay log starts at tick 0
    tw = nparams.TelemetryWriter(run_dir / "telemetry.jsonl") if telemetry else None
    gains = gains or Gains()
    nav = Navigator(MAG_REF)
    route = Route(spec.route, spec.speed, gains=gains)
    ap = Autopilot(gains)
    tracker = Tracker()
    planner = Planner()
    bots = {t.name: TrafficVessel(t, CTRL_TICKS * DT) for t in spec.traffic}
    xp = {
        t.name: ais_mod.Transponder(1000 + i, LAT0, LON0, t_first=3.0)
        for i, t in enumerate(spec.traffic)
        if t.ais
    }
    chan = ais_mod.Channel(p["seed"], LAT0, LON0)
    last_gnss, last_gnss_t, last_frames, last_scan_t = None, {}, -1, -1.0
    tgt_names = [t.name for t in spec.traffic]
    static_pts = np.zeros((0, 2))
    scan_mem = []  # (t, world points) of the last STATIC_MEM_S: a 1 m drum is seen by a pitching one-row lidar on only some revolutions
    rows = []
    hist = dict(
        t=[],
        own=[],
        psi=[],
        speed=[],
        tgt=[[] for _ in tgt_names],
        tvel=[[] for _ in tgt_names],
    )  # tgt / tvel: one list per traffic vessel
    cpa_min, collision, clear_min, done_t = math.inf, 0, math.inf, None
    thr = (0.0, 0.0)
    delta, frac, hold, cur_plan = 0.0, 1.0, 0.0, None
    last_truth = None
    n_ctrl = int(spec.t_end / (CTRL_TICKS * DT))
    for kc in range(n_ctrl):
        for _ in range(CTRL_TICKS // IMU_TICKS):
            sim.step(IMU_TICKS)
            nav.imu(
                _q(sim, "/devices/asv_imu", "gyro"),
                _q(sim, "/devices/asv_imu", "accel"),
                _q(sim, "/devices/asv_mag", "field"),
                IMU_TICKS * DT,
            )
        t = sim.tick * DT
        # ---- sensors -> estimate ----------------------------------------------------------------------------------------------------
        fix = (
            float(_q(sim, "/devices/asv_gnss", "lat")[0]),
            float(_q(sim, "/devices/asv_gnss", "lon")[0]),
        )
        if fix != last_gnss:
            last_gnss = fix
            nav.gnss(fix[0], fix[1], _q(sim, "/devices/asv_gnss", "vel_ned"))
        est = nav.state()
        for name, tp in xp.items():
            f = (
                float(_q(sim, f"/devices/{name}_gnss", "lat")[0]),
                float(_q(sim, f"/devices/{name}_gnss", "lon")[0]),
            )
            if f != last_gnss_t.get(name):
                last_gnss_t[name] = f
                tp.fix(t, f[0], f[1], _q(sim, f"/devices/{name}_gnss", "vel_ned"))
            chan.send(tp.poll(t))
        msgs = chan.receive(t)
        if msgs:
            tracker.update_ais(t, msgs)
        frames = int(_q(sim, "/devices/asv_lidar", "frames")[0])
        if frames != last_frames and nav.have_fix:
            last_frames = frames
            rg = _q(sim, "/devices/asv_lidar", "range")
            tracker.update_lidar(t, clusters(rg, est["psi"], est["pos"]))
            w, _, _, _, _ = scan_points(rg, est["psi"], est["pos"])
            mv = tracker.moving(t)
            if mv and len(w):
                keep = np.ones(len(w), bool)
                for tr in mv:
                    keep &= np.hypot(*(w - tr.pos).T) > 3.0 + 0.5 * tr.ext
                w = w[keep]
            scan_mem = [(s, m) for s, m in scan_mem if t - s <= STATIC_MEM_S] + [(t, w)]
            allp = np.concatenate([m for _, m in scan_mem])
            if len(allp):
                allp = (
                    np.unique(np.round(allp / 0.5), axis=0) * 0.5
                )  # one point per 0.5 m cell
                for tr in mv or []:  # no ghost of a vessel in the remembered cells
                    allp = allp[np.hypot(*(allp - tr.pos).T) > 3.0 + 0.5 * tr.ext]
            static_pts, last_scan_t = allp, t
        moving = tracker.moving(t)
        # ---- guidance, planning, control --------------------------------------------------------------------------------------------
        active = t >= enc.T_START and not route.done
        rt = (
            route.update(est["pos"])
            if nav.have_fix
            else dict(
                psi=est["psi"],
                u=0.0,
                xte=0.0,
                along=0.0,
                leg=0,
                dist=0.0,
                done=False,
                alpha=est["psi"],
            )
        )
        if active and kc % PLAN_EVERY == 0:
            wp = route.wp[route.k + 1]
            a = route.wp[route.k]
            L = float(np.hypot(*(wp - a)))
            cur_plan = planner.plan(
                t,
                dict(
                    pos=est["pos"], vel=est["vel"], psi=est["psi"], speed=est["speed"]
                ),
                dict(rt, leg_a=a, leg_dir=(wp - a) / max(L, 1e-9), leg_len=L),
                moving,
                static_pts,
            )
            delta, frac, hold = cur_plan.delta, cur_plan.frac, cur_plan.hold
        if t >= T_CTRL and nav.have_fix:
            if active:
                # a manoeuvre steers the leg direction plus the offset (no cross-track term) or the LOS course of a parallel line, no manoeuvre follows the LOS course
                a_, b_ = route.wp[route.k], route.wp[route.k + 1]
                rtl = dict(
                    rt,
                    leg_a=a_,
                    leg_dir=(b_ - a_) / max(float(np.hypot(*(b_ - a_))), 1e-9),
                )
                psi_cmd, u_cmd = steer(cur_plan, rtl, est["pos"]), rt["u"] * frac
                rt = dict(rt, psi_sp=psi_cmd)
            else:  # settling or finished: hold the heading, no way on
                psi_cmd, u_cmd = (rt["psi"] if not route.done else est["psi"]), 0.0
            thr = ap.step(
                psi_cmd, u_cmd, est["psi"], est["r"], est["u"], CTRL_TICKS * DT
            )
            sim.set("/devices/asv_esc1", "throttle", float(thr[0]))
            sim.set("/devices/asv_esc2", "throttle", float(thr[1]))
        # ---- truth, traffic, judging ---------------------------------------------------------------------------------------------------
        pos = _q(sim, "/world/asv_hull", "pos")
        quat = _q(sim, "/world/asv_hull", "quat")
        psi_t = yaw_of_quat(quat)
        speed_t = (
            0.0
            if last_truth is None
            else float(np.hypot(*(pos[:2] - last_truth)) / (CTRL_TICKS * DT))
        )
        last_truth = pos[:2].copy()
        tpos, tvel = [], []
        for name in tgt_names:
            tp_ = _q(sim, f"/world/{name}_hull", "pos")
            tq_ = _q(sim, f"/world/{name}_hull", "quat")
            if t >= enc.T_START:
                th = bots[name].step(tp_, tq_, pos)
                sim.set(f"/devices/{name}_esc1", "throttle", float(th[0]))
                sim.set(f"/devices/{name}_esc2", "throttle", float(th[1]))
            tpos.append(tp_[:2].copy())
            tvel.append(bots[name].v.copy())
        rng_t1 = min(
            (float(np.hypot(*(q - pos[:2]))) for q in tpos), default=999.0
        )  # the nearest traffic vessel
        if tpos:
            cpa_min = min(cpa_min, rng_t1)
            if rng_t1 < judge.COLLISION_VESSEL:
                collision = 1
        clr = judge.static_clearance(
            pos[:2], enc.SCENERY, enc.BUOYS, spec.obstacles, enc.BUOY_R
        )
        clear_min = min(clear_min, clr)
        if clr < judge.COLLISION_STATIC:
            collision = 1
        if t >= enc.T_START:
            hist["t"].append(t)
            hist["own"].append(pos[:2].copy())
            hist["psi"].append(psi_t)
            hist["speed"].append(speed_t)
            for i_ in range(len(tpos)):
                hist["tgt"][i_].append(tpos[i_])
                hist["tvel"][i_].append(tvel[i_])
        if route.done and done_t is None:
            done_t = t
        # ---- log ------------------------------------------------------------------------------------------------------------------------
        a0, b0 = (
            route.wp[min(route.k, len(route.wp) - 2)],
            route.wp[min(route.k, len(route.wp) - 2) + 1],
        )
        leg = (b0 - a0) / max(float(np.hypot(*(b0 - a0))), 1e-9)
        xte_t = float(-leg[1] * (pos[0] - a0[0]) + leg[0] * (pos[1] - a0[1]))
        rule = cur_plan.rule if cur_plan is not None and active else "none"
        if kc % ROW_EVERY == 0:
            rows.append([
                t, pos[0], pos[1], heading_deg(psi_t), speed_t, float(np.hypot(*(est["pos"] - pos[:2]))), math.degrees(wrap(est["psi"] - psi_t)), xte_t,
                float(thr[0]), float(thr[1]), min(rng_t1, 999.0), min(cpa_min, 999.0), collision, min(clear_min, 999.0), int(route.done), len(moving),
                RULE_CODE.get(rule, 0), math.degrees(delta) if active else 0.0, 1, 1, 1, 0.0, float(_q(sim, "/devices/asv_battery1", "soc")[0]),
            ])  # fmt: skip
        if tw is not None and kc % TLM_EVERY == 0:
            tw.emit(
                telemetry_record(
                    sim,
                    t,
                    spec,
                    pos,
                    psi_t,
                    speed_t,
                    est,
                    thr,
                    rt,
                    route,
                    delta,
                    frac,
                    cur_plan,
                    moving,
                    tracker,
                    tpos,
                    tvel,
                    tgt_names,
                    active,
                    rule,
                    static_pts,
                )
            )
        if done_t is not None and t > done_t + 3.0:
            break
    # ---- verdict ------------------------------------------------------------------------------------------------------------------------
    res = dict(
        kind=kind,
        seed=p["seed"],
        hs=p["hs"],
        t_end=sim.tick * DT,
        collision=bool(collision),
        min_static_clear_m=min(clear_min, 999.0),
        cpa_min_m=min(cpa_min, 999.0),
    )
    own = np.array(hist["own"])
    xt = [r[7] for r in rows if r[0] >= enc.T_START]
    res.update(
        xte_rms_m=float(np.sqrt(np.mean(np.square(xt)))) if xt else 0.0,
        xte_max_m=float(np.max(np.abs(xt))) if xt else 0.0,
    )
    end = np.array(spec.route[-1])
    res["mission_complete"] = (
        bool(route.done and np.hypot(*(own[-1] - end)) < 8.0) if len(own) else False
    )
    res["end_dist_m"] = float(np.hypot(*(own[-1] - end))) if len(own) else 999.0
    res["est_pos_err_rms_m"] = float(np.sqrt(np.mean([r[5] ** 2 for r in rows])))
    res["est_hdg_err_rms_deg"] = float(
        np.sqrt(np.mean([r[6] ** 2 for r in rows if r[0] > 5.0]))
    )
    if spec.traffic:
        v = judge.verdict_all([t_.kind or kind for t_ in spec.traffic], np.array(hist["t"]), own, np.array(hist["psi"]), np.array(hist["speed"]),
                              [np.array(q) for q in hist["tgt"]], [np.array(q) for q in hist["tvel"]],
                              math.atan2(*(np.array(spec.route[1]) - np.array(spec.route[0]))[::-1]), spec.speed, complied=bool(p["traffic_complies"]))  # fmt: skip
        v["collision"] = bool(
            v["collision"] or collision
        )  # the judge sees the traffic only; the run also saw a fixed object touched
        res.update(v)
        res["cpa_m"] = v["cpa_m"]
        last = rows[-1]
        last[-5], last[-4], last[-3], last[-2] = (
            int(v["colreg_ok"]),
            int(v["side_ok"]),
            int(v["cpa_ok"]),
            v["cpa_m"],
        )
    else:
        res.update(
            colreg_ok=not collision,
            side_ok=True,
            cpa_ok=True,
            cpa_m=min(clear_min, 999.0),
        )
        rows[-1][-5] = int(res["colreg_ok"])
    res["ok"] = bool(
        res["mission_complete"] and res["colreg_ok"] and not res["collision"]
    )
    if tw is not None:
        tw.close()
    return dict(res=res, rows=rows, sim=sim)


def telemetry_record(
    sim,
    t,
    spec,
    pos,
    psi_t,
    speed_t,
    est,
    thr,
    rt,
    route,
    delta,
    frac,
    plan,
    moving,
    tracker,
    tpos,
    tvel,
    names,
    active,
    rule,
    static_pts,
):
    """One telemetry.jsonl line. Georeferenced (lat / lon of the scene origin, WGS84 flat-earth), SI units, headings and courses in degrees clockwise from North."""
    ll = lambda x, y: [round(v, 8) for v in latlon_from_enu(x, y)]
    cog = lambda v: round(math.degrees(math.pi / 2 - math.atan2(v[1], v[0])) % 360.0, 2)
    q = _q(sim, "/world/asv_hull", "quat")
    w, x, y, z = q
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    e = est
    contacts = []
    by_id = {c["id"]: c for c in (plan.contacts if plan is not None else [])}

    for tr in tracker.confirmed(t):
        gm = cr.geometry(e["pos"], e["vel"], e["psi"], tr.pos, tr.vel)
        c = by_id.get(tr.id, {})
        contacts.append(dict(
            id=tr.id, mmsi=tr.mmsi or None, src=tr.source(t), moving=tr.speed > tracker.v_min,
            lat_lon=ll(*tr.pos), x_m=round(float(tr.pos[0]), 2), y_m=round(float(tr.pos[1]), 2),
            sog_ms=round(tr.speed, 2), cog_deg=cog(tr.vel) if tr.speed > 0.1 else None,
            range_m=round(gm.range, 1), bearing_deg=round(math.degrees(gm.bearing), 1),
            cpa_m=round(gm.cpa, 1), tcpa_s=round(gm.tcpa, 1) if math.isfinite(gm.tcpa) else None,
            rule=c.get("cls", "none"), role=c.get("role", "none"), risk=c.get("risk", False), ext_m=round(tr.ext, 2),
        ))  # fmt: skip
    return dict(
        t=round(t, 3),
        pose=dict(lat_lon=ll(pos[0], pos[1]), x_m=round(float(pos[0]), 3), y_m=round(float(pos[1]), 3), heading_deg=round(heading_deg(psi_t), 2),
                  roll_deg=round(roll, 2), pitch_deg=round(pitch, 2), sog_ms=round(speed_t, 3)),
        est=dict(lat_lon=ll(*e["pos"]), x_m=round(float(e["pos"][0]), 3), y_m=round(float(e["pos"][1]), 3), heading_deg=round(heading_deg(e["psi"]), 2),
                 sog_ms=round(e["speed"], 3), cog_deg=cog(e["vel"]) if e["speed"] > 0.2 else None, yaw_rate_dps=round(math.degrees(-e["r"]), 2),
                 surge_ms=round(e["u"], 3), sway_ms=round(e["v"], 3)),
        sp=dict(heading_deg=round(heading_deg(rt.get("psi_sp", rt["psi"]) if active else rt["psi"]), 2), speed_ms=round(rt["u"] * frac if active else 0.0, 3),
                thr_port=round(float(thr[0]), 3), thr_stbd=round(float(thr[1]), 3), course_offset_deg=round(math.degrees(delta) if active else 0.0, 1), speed_frac=round(frac, 2)),
        mission=dict(leg=int(route.k), xte_m=round(float(rt["xte"]), 2), dist_to_wp_m=round(float(rt["dist"]), 1), done=bool(route.done)),
        tracks=contacts,
        traj=[ll(*pt) for pt in plan.traj[::4]] if plan is not None and active else [],
        rule=rule, emergency=bool(plan.emergency) if plan is not None else False,
        scan=dict(n_static_pts=len(static_pts)),
        truth=dict(traffic=[dict(name=n, lat_lon=ll(*tp_), sog_ms=round(float(np.hypot(*tv)), 2), cog_deg=cog(tv) if np.hypot(*tv) > 0.1 else None)
                            for n, tp_, tv in zip(names, tpos, tvel)]),
    )  # fmt: skip


def main():
    kind = sys.argv[1]
    sc = (
        Path(os.environ.get("NAPL_SCENARIO_DIR", Path(__file__).parent))
        / f"asv-{kind.replace('_', '-')}.scenario.toml"
    )
    p = nparams.values(sc)
    run_dir = Path(os.environ["NAPL_RUN_DIR"])
    r = fly(
        kind, p, run_dir
    )  # always rebuilds the scene of (kind, seed, sea state) into <run dir>/model: the replay uses it
    write_native(r["sim"], DT, KN_COLS, r["rows"])
    (run_dir / "result.json").write_text(json.dumps(r["res"], indent=1, default=float))
    sys.stdout.write(json.dumps(r["res"], default=float) + "\n")


if __name__ == "__main__":
    main()
