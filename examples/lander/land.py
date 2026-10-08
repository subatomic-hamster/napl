"""TVC lander flight harness: the program of land.scenario.toml. Owns the Sim, plays the world around the flight computer (sensor sampling at the part
rates, servo set-up, pyro channels with their delays), injects the seeded physical scatter, steps the flight software of flight.py at 250 Hz, writes
trace.csv / replay.naplog / native.json (napl.marine.write_native) and telemetry.jsonl (the HUD stream, napl.params contract).

Everything that is random is drawn from numpy's PCG64 seeded [seed, 5150] in a fixed order, or lives inside the sim (sensor noise, Dryden turbulence:
seeded by the model build, scene.build_seeded), so a (seed, parameters) pair replays to the same state hash.

Env: NAPL_MODEL, NAPL_RUN_DIR, NAPL_SCENARIO_DIR (set by `napl program`), NAPL_PARAMS (JSON overrides of the scenario's [[parameters]])."""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

import napl
from flight import DT as CTRL_DT
from flight import Flight
from napl import params as nparams
from napl.marine import write_native
from vehicle import GIMBAL_LIMIT

SIM_DT = 0.001
STRIDE = round(CTRL_DT / SIM_DT)  # sim ticks per control tick
ROW_EVERY = 16  # trace row every 16 ms
TLM_EVERY = (
    48  # telemetry record every 48 ms (about 21 Hz, a multiple of the control period)
)
SERVO_P, SERVO_D = (
    2000,
    6000,
)  # runtime servo gains (default P 400 sags 11 deg under the off-axis motor load; docs/lander.md)
SCENARIO = Path(__file__).parent / "land.scenario.toml"
STATE_CODE = dict(
    PAD=0,
    PAD_READY=0,
    BOOST=1,
    COAST=2,
    APOGEE=3,
    DESCENT=4,
    LANDING_BURN=5,
    TOUCHDOWN=6,
    ABORT=7,
)
COLS = [
    "t", "x_m", "y_m", "z_m", "vz_ms", "tilt_deg", "tilt_flight_deg", "state", "thrust_N", "pitch_deg", "yaw_deg", "sat_frac",
    "est_alt_m", "est_vz_ms", "alt_err_m", "xy_err_m", "td_speed_ms", "td_tilt_deg", "td_dist_m", "landed",
]  # fmt: skip
TOUCH_Z = 0.01  # m above the standing height: feet down


def draw_hidden(seed, p):
    """The physical scatter of one flight (the flight computer does not know any of it). Fixed draw order."""
    r = np.random.default_rng([int(seed), 5150])
    n = lambda s: float(np.clip(r.standard_normal(), -3, 3) * s)
    h = {}
    h["scale_a"] = 1 + n(p["impulse_sigma"])
    h["scale_b"] = 1 + n(p["impulse_sigma"])
    h["extra_a"] = max(0.0, p["delay_extra_s"] + n(p["delay_jitter_s"]))
    h["extra_b"] = max(0.0, p["delay_extra_s"] + n(p["delay_jitter_s"]))
    h["mis_pitch"] = math.radians(n(p["misalign_deg"]))
    h["mis_yaw"] = math.radians(n(p["misalign_deg"]))
    h["gyro_bias"] = [math.radians(n(p["gyro_bias_dps"])) for _ in range(3)]
    h["accel_bias"] = [n(p["accel_bias_ms2"]) for _ in range(3)]
    w = math.radians(p["wind_dir_deg"])
    h["wind"] = [p["wind_ms"] * math.cos(w), p["wind_ms"] * math.sin(w), 0.0]
    return h


def _play(y, x, b):
    """Backlash (play) operator, half width b: the output y does not move until the input x has taken up the play, then follows it at a distance b."""
    if x - y > b:
        return x - b
    if y - x > b:
        return x + b
    return y


def q(sim, path, attr, i=0):
    return float(sim.query(path, attr)[0][i])


def tilt_deg(sim):
    qw, qx, qy, qz = sim.query("/world/tube", "quat")[0]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))


def fly(seed, p, model, run_dir=None, log=True, telemetry=True, max_s=45.0):
    """One flight. -> dict(result metrics, rows, sim, events). `p` = resolved parameters (napl.params.resolve)."""
    sim = napl.Sim.load(str(model))
    if log:
        sim.start_log()  # before the first command
    h = draw_hidden(seed, p)
    tw = (
        nparams.TelemetryWriter(Path(run_dir) / "telemetry.jsonl")
        if telemetry and run_dir
        else None
    )
    for s in ("pitch_servo", "yaw_servo"):
        sim.set(f"/devices/{s}", "torque_enable", [1])
        sim.set(f"/devices/{s}", "p_gain", [SERVO_P])
        sim.set(f"/devices/{s}", "d_gain", [SERVO_D])
        sim.set(f"/devices/{s}", "goal_position", [0.0])
    sim.set("/devices/imu", "gyro_bias", h["gyro_bias"])
    sim.set("/devices/imu", "accel_bias", h["accel_bias"])
    sim.set("/devices/air", "wind", h["wind"])
    sim.set("/devices/motor_a", "thrust_scale", [h["scale_a"]])
    sim.set("/devices/motor_b", "thrust_scale", [h["scale_b"]])
    fl = Flight(
        t_launch=p["t_launch"], aim_bias=p["aim_bias_m"], v_target=p["v_target"], wn_att=p["wn_att"], kp_pos=p["kp_pos"], kd_vel=p["kd_vel"],
        kd_land=p["kd_land"], z_slow=p["z_slow"], td_gimbal_deg=p["td_gimbal_deg"], tilt_cmd_max_deg=p["tilt_cmd_max_deg"],
    )  # fmt: skip
    z0 = q(sim, "/world/tube", "pos", 2)
    x0, y0 = q(sim, "/world/tube", "pos", 0), q(sim, "/world/tube", "pos", 1)
    pending = []  # (tick, motor): pyro commands in flight (igniter delay beyond the motor's own)
    backlash = math.radians(
        p["backlash_deg"]
    )  # gearbox play of each gimbal servo (half width)
    play = [
        0.0,
        0.0,
    ]  # output shaft angle each servo holds inside its play, pitch / yaw
    last_baro, last_gnss = None, None
    rows, res = [], dict(td_speed=99.0, td_tilt=99.0, td_dist=99.0, landed=0)
    z_prev, vz, sat_n, pow_n, touch_tick, z_max = 0.0, 0.0, 0, 0, None, 0.0
    max_tilt_ascent = 0.0
    for k in range(int(max_s / SIM_DT)):
        if k % STRIDE == 0:
            t = sim.tick * SIM_DT
            gyro = sim.query("/devices/imu", "gyro")[0]
            accel = sim.query("/devices/imu", "accel")[0]
            pr = q(sim, "/devices/baro", "pressure")
            baro = pr if pr != last_baro and pr > 0 else None  # 0 = no sample yet
            last_baro = pr
            gn = tuple(float(x) for x in np.concatenate([sim.query("/devices/gnss", "lat")[0], sim.query("/devices/gnss", "lon")[0], sim.query("/devices/gnss", "alt")[0], sim.query("/devices/gnss", "vel_ned")[0]]))  # fmt: skip
            fix = gn if gn != last_gnss and any(gn) else None
            last_gnss = gn
            c = fl.step(t, gyro, accel, baro, fix)
            lim = 0.139  # mechanical range of the hinges
            for i, (name, goal, mis) in enumerate(
                (("pitch_servo", c.pitch_goal, h["mis_pitch"]), ("yaw_servo", c.yaw_goal, h["mis_yaw"]))
            ):  # fmt: skip
                play[i] = _play(
                    play[i], float(np.clip(goal + mis, -lim, lim)), backlash
                )
                sim.set(f"/devices/{name}", "goal_position", [play[i]])
            for m, flag in (("a", c.ignite_a), ("b", c.ignite_b)):
                if flag:
                    pending.append((k + round(h["extra_" + m] / SIM_DT), m))
            # truth for the trace and the touchdown logic
            zt = q(sim, "/world/tube", "pos", 2) - z0
            vz = (zt - z_prev) / CTRL_DT
            z_prev = zt
            thrust = q(sim, "/devices/motor_a", "thrust") + q(
                sim, "/devices/motor_b", "thrust"
            )
            tl = tilt_deg(sim)
            if thrust > 1.0:
                pow_n += 1
                sat_n += (
                    abs(c.pitch_goal) >= GIMBAL_LIMIT - 1e-6
                    or abs(c.yaw_goal) >= GIMBAL_LIMIT - 1e-6
                )
            z_max = max(z_max, zt)
            if (
                z_max > 0.5
                and zt <= TOUCH_Z
                and vz < 0
                and (touch_tick is None or -vz > res["td_speed"])
            ):  # the HARDEST ground contact counts: a vehicle that lifts off again and drops is judged on the drop
                touch_tick = k if touch_tick is None else touch_tick
                res.update(
                    td_speed=-vz,
                    td_tilt=tl,
                    td_dist=math.hypot(
                        q(sim, "/world/tube", "pos", 0) - x0,
                        q(sim, "/world/tube", "pos", 1) - y0,
                    ),
                    landed=1,
                )
            if fl.state in ("BOOST", "COAST", "APOGEE"):
                max_tilt_ascent = max(max_tilt_ascent, tl)
            if k % ROW_EVERY == 0:  # trace row every 16 ms
                px, py = (
                    q(sim, "/world/tube", "pos", 0) - x0,
                    q(sim, "/world/tube", "pos", 1) - y0,
                )
                est = fl.telemetry()
                ea = est.get("alt_m", 0.0)
                rows.append([
                    t, px, py, zt, vz, tl, tl if not res["landed"] else 0.0, STATE_CODE[fl.state], thrust,
                    math.degrees(q(sim, "/devices/pitch_servo", "position")), math.degrees(q(sim, "/devices/yaw_servo", "position")),
                    sat_n / max(pow_n, 1), ea, est.get("vz_ms", 0.0), ea - zt,
                    math.hypot(est.get("x_m", 0.0) - px, est.get("y_m", 0.0) - py),
                    res["td_speed"], res["td_tilt"], res["td_dist"], res["landed"],
                ])  # fmt: skip
            if tw and k % TLM_EVERY == 0:
                tw.emit(_record(fl, sim, t, zt, vz, tl, thrust, x0, y0, h))
            if touch_tick is not None and k - touch_tick > 2000:
                break
            if sim.diverged:
                break
        for item in [x for x in pending if x[0] == k]:
            sim.set(f"/devices/motor_{item[1]}", "ignite", 1)
            pending.remove(item)
        sim.step(1)
    if tw:
        tw.close()
    res.update(
        seed=seed, state=fl.state, apogee_m=fl.z_apogee, max_ascent_tilt=max_tilt_ascent, sat_frac=sat_n / max(pow_n, 1),
        t_end=sim.tick * SIM_DT, diverged=bool(sim.diverged), events=fl.events, hidden=h, d_learned=fl.d_est,
    )  # fmt: skip
    return dict(res=res, rows=rows, sim=sim)


def _record(fl, sim, t, zt, vz, tl, thrust, x0, y0, h):
    e = fl.telemetry()
    mots = {}
    for k in ("a", "b"):
        d = f"/devices/motor_{k}"
        mots[k] = dict(
            phase=["idle", "delay", "burning", "spent"][int(q(sim, d, "phase"))],
            thrust_N=round(q(sim, d, "thrust"), 3),
        )
    return dict(
        t=round(t, 3),
        state=e["state"],
        est={
            k: e.get(k)
            for k in (
                "alt_m",
                "vz_ms",
                "x_m",
                "y_m",
                "vx_ms",
                "vy_ms",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "tilt_deg",
            )
        },
        gimbal=dict(
            pitch_deg=round(
                math.degrees(q(sim, "/devices/pitch_servo", "position")), 3
            ),
            yaw_deg=round(math.degrees(q(sim, "/devices/yaw_servo", "position")), 3),
            pitch_goal_deg=e.get("pitch_goal_deg"),
            yaw_goal_deg=e.get("yaw_goal_deg"),
        ),
        thrust_N=round(thrust, 3),
        thrust_est_N=e.get("thrust_N"),
        mass_kg=e.get("mass_kg"),
        motors=mots,
        landing=dict(
            ign_margin_m=e.get("ign_margin_m"),
            pred_touchdown_ms=e.get("pred_touchdown_ms"),
        ),
        truth=dict(
            alt_m=round(zt, 3),
            vz_ms=round(vz, 3),
            x_m=round(q(sim, "/world/tube", "pos", 0) - x0, 3),
            y_m=round(q(sim, "/world/tube", "pos", 1) - y0, 3),
            tilt_deg=round(tl, 2),
        ),
        wind_ms=[round(w, 2) for w in h["wind"]],
    )


def main():
    sc = (
        Path(os.environ.get("NAPL_SCENARIO_DIR", Path(__file__).parent))
        / "land.scenario.toml"
    )
    p = nparams.values(sc)
    run_dir = Path(os.environ["NAPL_RUN_DIR"])
    model = os.environ["NAPL_MODEL"]
    if (
        p["seed"] != 1 or p["w20_ms"] >= 0
    ):  # the build-time constants (device seeds, Dryden intensity) follow the parameters
        from scene import build_seeded

        model = build_seeded(
            run_dir / "model",
            p["seed"],
            p["w20_ms"] if p["w20_ms"] >= 0 else p["wind_ms"],
        )
    r = fly(p["seed"], p, model, run_dir)
    write_native(r["sim"], SIM_DT, COLS, r["rows"])
    (run_dir / "result.json").write_text(
        json.dumps(
            {k: v for k, v in r["res"].items() if k != "hidden"}, default=str, indent=1
        )
    )
    sys.stdout.write(
        json.dumps(
            {k: v for k, v in r["res"].items() if k not in ("hidden", "events")},
            default=str,
        )
        + "\n"
    )
    for ev in r["res"]["events"]:
        sys.stdout.write(f"  {ev[0]:7.3f}s {ev[1]}\n")


if __name__ == "__main__":
    main()
