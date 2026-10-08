"""Flight software of the TVC lander. Reads ONLY simulated sensors (IMU gyro + accel, barometer pressure, GNSS fixes, the servo position readouts) and
commands the two gimbal servos and the two pyro channels. No truth state, no simulator import: `Flight.step` takes numbers and returns numbers, so the
same code runs in tests on synthetic data and in land.py against the napl sim at 250 Hz.

State machine (Flight.state):
  PAD          2 s of calibration (gyro bias, up vector, barometer reference, GNSS pad average), then the ascent motor is lit at t_launch
  BOOST        ascent motor lit: igniter delay, thrust detected on the accelerometer, TVC attitude control with lateral (wind) guidance, until burnout
  COAST        no thrust: gimbal centred, filters coast on the accelerometer
  APOGEE       vertical velocity estimate crosses zero (one tick)
  DESCENT      tail-first fall; every 0.1 s the ignition rule (energy.Lander.j_onset) asks how soft the best planned landing would be if the motor were lit at the next look
  LANDING_BURN landing motor lit (igniter delay, then thrust): attitude control + lateral guidance to the pad until burnout; every 0.1 s energy.Lander.plan picks the
               tilt that throws the excess of an unthrottleable motor's thrust sideways (alternating direction), levelled out for the last t_lev seconds
  TOUCHDOWN    contact (accelerometer spike, or low and slow after burnout): gimbal centred
  ABORT        tilt beyond ABORT_TILT while powered: gimbal centred, landing motor not lit if the vehicle is more than 60 deg off vertical

Attitude loop (per axis, small angles, body frame): e = tilt error of the body axis to the commanded thrust axis, alpha = wn^2 e - 2 zeta wn w, torque
I alpha, gimbal angle = torque / (L T) with T = m x measured specific force (gain scheduling: the loop gain is constant while thrust varies from 0 to 26 N
and the mass falls), plus the feed-forward trim of the lit motor's off-axis seat and a slow integrator (removes servo sag, thrust-line misalignment, CG
offset). Gimbal sign conventions (verified against the sim): pitch servo + gives torque about +y, yaw servo + gives torque about +x."""

import math
from dataclasses import dataclass

import numpy as np
from energy import Lander, TiltFamily, ascent_dump
from estimator import Attitude, AxisFilter, baro_height
from guidance import ballistic
from vehicle import (
    ASCENT_MOTOR,
    GIMBAL_LIMIT,
    I_TRANS,
    L_NOZ,
    LANDING_MOTOR,
    M_LOADED,
    G,
    Motor,
    trim_angles,
)

DT = 0.004  # s, control period (250 Hz: every second sample of the 500 Hz IMU)
T_NOM = 17.0  # N, a typical thrust of the lit motor, to which the integrator gain is referred
ABORT_TILT = math.radians(
    35.0
)  # the dump tilts the vehicle up to dump_max_deg plus a few degrees of guidance lean on purpose
EARTH_R = 6371000.0
ASC_T_LEV = 0.3  # s: the ascent burn is flown level for its last 0.3 s (energy.ascent_dump plans with the same)
PLAN_TICKS = 25  # the predictors run at 10 Hz

DEFAULTS = dict(
    t_cal=2.0,  # s of pad calibration
    t_launch=3.0,  # s: the ascent motor is lit
    wn_att=8.0,  # rad/s attitude loop bandwidth (13 saturated the servo goals on every dump step; 6 lagged)
    zeta=0.8,
    ki=1.2,  # rad of gimbal per rad of tilt error per s
    kp_pos=0.8,  # 1/s2 lateral position -> acceleration (0.35 let a 4 m/s wind push 3/4 of the flights past the 3 m circle)
    kd_vel=1.3,  # 1/s lateral velocity -> acceleration
    a_lat_max=2.2,  # m/s2
    tilt_cmd_max_deg=10.0,  # deg: lateral guidance limit when no thrust is being dumped
    s_plan=0.99,  # thrust scale the ignition rule plans for (the weak side of the scatter: a strong motor is dumped, a weak one cannot be helped)
    j_fire=1.0,  # m/s: the motor is lit at the latest moment at which the planned landing (scale s_plan) is still predicted this soft
    dump_max_deg=12.0,  # deg: largest tilt the landing burn may use to throw thrust away sideways (20 and above: lateral drift and tilt at touchdown cost more than the speed gains)
    asc_dump_max_deg=30.0,  # deg: same for the ascent burn
    apogee_cap_m=15.0,  # m: the ascent dumps thrust to keep the predicted apogee under this; the E16's natural 6.7-9 m never reaches 15, i.e. off (caps of 6-8 m lost, docs/lander.md)
    t_lev=0.8,  # s: the last part of the landing burn is flown level (the tilt is brought back before the thrust ends)
    w_lat=0.5,  # m/s per unit mean sin(tilt): price of lateral force in the landing plan
    alt_period=3.0,  # s: the sideways part of the dumped thrust alternates in direction with this period
    delay_prior=0.3,  # s: ignition_delay of the motor data sheet (estimated; the ascent ignition measures the real one)
    z_slow=3.0,  # m: below this height the landing burn fades position hold out and sideways-speed damping in
    td_gimbal_deg=7.0,  # deg: attitude correction allowed on the ground while the motor still burns
    kd_land=1.5,  # 1/s: extra sideways-speed damping at the ground
    v_arm=3.0,  # m/s: the ignition rule is armed below this vertical speed (coast, so before apogee)
)


@dataclass
class Commands:
    pitch_goal: float = 0.0  # rad, servo goal_position
    yaw_goal: float = 0.0
    ignite_a: bool = False  # one-tick pulses on the pyro channels
    ignite_b: bool = False


@dataclass
class _Motor:
    tab: Motor
    cmd_t: float = None  # time the pyro channel was fired
    tau0: float = (
        None  # time at which the curve time is 0 (estimated from the thrust onset)
    )
    spent: bool = False
    hits: int = 0


class Flight:
    def __init__(self, **params):
        self.p = {**DEFAULTS, **params}
        self.state = "PAD"
        self.t = 0.0
        self.att = Attitude()
        self.kf = None  # AxisFilter x3
        self.m = {"a": _Motor(Motor(ASCENT_MOTOR)), "b": _Motor(Motor(LANDING_MOTOR))}
        self.mass = M_LOADED
        self.f_z = G  # filtered specific force along body z
        self.thrust = 0.0
        self.i_p = self.i_y = 0.0
        self.goal = np.zeros(2)  # pitch, yaw servo goals
        self.cal = dict(n=0, p=[], gnss=[], f=[])
        self.p_ref = None
        self.gnss_ref = None
        self.d_est = self.p["delay_prior"]
        self.lander = None  # energy.Lander, built at the ascent burnout
        self.margin = float(
            "nan"
        )  # ignition yardstick: j_fire - predicted outcome at the next look (m/s), > 0 = not yet
        self.pred_td = float(
            "nan"
        )  # predicted hardest touchdown speed of the current landing plan (m/s)
        self.looks = []  # (t, z, vz, z_onset, v_onset, cost) of every ignition look
        self.j_min = float("inf")
        self.j_armed = False
        self.s_hat = {
            "a": 1.0,
            "b": 1.0,
        }  # thrust scale of each motor, measured on the accelerometer against its certified curve
        self.cos_f = 1.0  # cos of the tilt being dumped now (filtered)
        self.cos_tgt = 1.0
        self.cos_plan = 1.0  # the planner's choice, before the level-out ramp
        self.apo_pred = float("nan")
        self.phi = 0.0  # phase of the alternating sideways dump
        self.abort_reason = ""
        self.touch_t = None
        self.events = []  # (t, text) for the log
        self._burn_low = 0
        self._tick = 0
        self.z_apogee = 0.0
        self.active = None  # 'a' / 'b': the motor currently thrusting

    # ------------------------------------------------------------------------------------------------------- helpers
    def _log(self, text):
        self.events.append((round(self.t, 3), text))

    def _update_mass(self):
        m = M_LOADED
        for k in ("a", "b"):
            mm = self.m[k]
            if mm.tau0 is not None:
                m -= float(mm.tab.burned(self.t - mm.tau0))
        self.mass = m

    def _thrust_detect(self, k, thr_g, dt):
        """Thrust onset on the accelerometer: f_z above thr_g x g for 3 ticks -> the motor's curve time zero."""
        mm = self.m[k]
        if mm.tau0 is not None or mm.cmd_t is None:
            return
        mm.hits = mm.hits + 1 if self.f_z > thr_g * G else 0
        if mm.hits >= 3:
            tau_det = mm.tab.tau_at_thrust(thr_g * G * self.mass)
            mm.tau0 = self.t - tau_det - 2 * dt  # 3-tick confirmation
            self.active = k
            if k == "a":
                self.d_est = (
                    mm.tau0 - mm.cmd_t
                )  # the igniter delay measured on the ascent
            self._log(
                f"motor {k} thrust detected, ignition delay {mm.tau0 - mm.cmd_t:.3f} s"
            )

    @staticmethod
    def _levelled(c_plan, tau, burn, t_lev, ramp=0.3):
        """cos of the tilt to fly: the plan's, brought back to level over `ramp` s that end `t_lev` s before the burn does (a slew that is still going at the
        burnout leaves the unpowered vehicle rotating)."""
        b = min(1.0, max(0.0, (tau - (burn - t_lev - ramp)) / ramp))
        return 1.0 - (1.0 - c_plan) * (1.0 - b)

    def _scale_update(self):
        """Thrust scale of the burning motor: the thrust on the accelerometer over the certified curve at the time since its onset. Not on the steep parts of
        the curve, where the error of the onset time would pass as a scale error."""
        k = self.active
        if k is None:
            return
        mm = self.m[k]
        tau = self.t - mm.tau0
        tc = float(mm.tab.thrust(tau))
        if tc < 8.0 or tau > mm.tab.burn - 0.3:
            return
        slope = (
            abs(float(mm.tab.thrust(tau + 0.03)) - float(mm.tab.thrust(tau - 0.03)))
            / 0.06
        )
        if slope * 0.015 > 0.02 * tc:
            return
        self.s_hat[k] += 0.02 * (self.thrust / tc - self.s_hat[k])

    def _burnout_detect(self, k):
        mm = self.m[k]
        if mm.tau0 is None or mm.spent:
            return False
        tau = self.t - mm.tau0
        if tau > 0.5 * mm.tab.burn and self.f_z < 0.8 * G:
            self._burn_low += 1
        else:
            self._burn_low = 0
        if self._burn_low >= 3 or tau > mm.tab.burn + 0.3:
            mm.spent = True
            self._burn_low = 0
            self.active = None
            self._log(f"motor {k} burnout")
            return True
        return False

    # ---------------------------------------------------------------------------------------------------- calibration
    def _calibrate(self, gyro, accel, baro, gnss):
        c = self.cal
        self.att.calibrate(gyro, accel)
        c["f"].append(float(np.linalg.norm(accel)))
        if baro is not None:
            c["p"].append(baro)
        if gnss is not None:
            c["gnss"].append(gnss[:2])
        if self.t >= self.p["t_cal"] and len(c["p"]) >= 5:
            self.att.finish_calibration()
            self.p_ref = float(np.mean(c["p"]))
            g = np.array(c["gnss"]) if c["gnss"] else np.zeros((1, 2))
            self.gnss_ref = g.mean(axis=0)
            self.kf = [
                AxisFilter(DT),
                AxisFilter(DT),
                AxisFilter(DT, p0=(0.3, 0.2, 0.3)),
            ]
            self.kf[2].x[2] = (
                float(np.mean(c["f"])) - G
            )  # accelerometer z bias from |f| at rest
            self.state = "PAD_READY"
            self._log("calibrated")

    # ------------------------------------------------------------------------------------------------------- the tick
    def step(self, t, gyro, accel, baro=None, gnss=None):
        """One control tick. gyro rad/s, accel m/s2 (body), baro Pa or None (no new sample), gnss (lat, lon, alt, vn, ve, vd) or None. -> Commands"""
        self.t = t
        self._tick += 1
        gyro, accel = np.asarray(gyro, float), np.asarray(accel, float)
        out = Commands()
        if self.state == "PAD":
            self._calibrate(gyro, accel, baro, gnss)
            return out
        # ---- estimation
        self.att.update(gyro, accel, DT, aid=self.active is None)
        aw = self.att.R @ accel
        for i, a in enumerate((aw[0], aw[1], aw[2] - G)):
            self.kf[i].predict(a)
        if baro is not None:
            self.kf[2].update(0, baro_height(baro, self.p_ref), 0.15)
        if gnss is not None:
            lat, lon, _, vn, ve, vd = gnss
            y = math.radians(lat - self.gnss_ref[0]) * EARTH_R
            x = (
                math.radians(lon - self.gnss_ref[1])
                * EARTH_R
                * math.cos(math.radians(self.gnss_ref[0]))
            )
            lag = 50  # GNSS latency 0.2 s in ticks (the receiver's documented output delay)
            self.kf[0].update(0, x, 0.8, lag)
            self.kf[1].update(0, y, 0.8, lag)
            self.kf[0].update(1, ve, 0.08, lag)
            self.kf[1].update(1, vn, 0.08, lag)
            self.kf[2].update(1, -vd, 0.08, lag)
        z, vz = self.kf[2].pos, self.kf[2].vel
        self.f_z += 0.35 * (
            accel[2] - self.f_z
        )  # ~10 ms low pass of the specific force along the axis
        self._update_mass()
        self.thrust = self.mass * self.f_z if self.active else 0.0
        self._scale_update()

        st = self.state
        if st == "PAD_READY":
            if self.t >= self.p["t_launch"]:
                self.m["a"].cmd_t = self.t
                out.ignite_a = True
                self._log("ascent motor fired")
                self.state = "BOOST"
                self.goal[:] = trim_angles("a")
        elif st == "BOOST":
            self._thrust_detect("a", 1.25, DT)
            mm = self.m["a"]
            if (
                mm.tau0 is not None
                and not mm.spent
                and self._tick % PLAN_TICKS == 0
                and self.t - mm.tau0 > 0.3
            ):  # ascent energy management: dump what the predicted apogee exceeds the cap by
                self.cos_plan, self.apo_pred, _ = ascent_dump(
                    mm.tab,
                    self.t - mm.tau0,
                    z,
                    vz,
                    self.mass,
                    self.s_hat["a"],
                    self.p["apogee_cap_m"],
                    self.p["asc_dump_max_deg"],
                    t_lev=ASC_T_LEV,
                )
            if (
                mm.tau0 is not None
            ):  # upright again before the thrust ends: the coast is unpowered and tail-first unstable
                self.cos_tgt = self._levelled(
                    self.cos_plan, self.t - mm.tau0, mm.tab.burn, ASC_T_LEV
                )
            if self._burnout_detect("a"):
                self.state = "COAST"
                self.i_p = self.i_y = 0.0
                self.cos_tgt = 1.0
                self.lander = Lander(
                    Motor(LANDING_MOTOR),
                    TiltFamily(self.p["dump_max_deg"]),
                    self.p["t_lev"],
                    self.p["w_lat"],
                )
        elif st == "COAST":
            # the landing motor needs its igniter delay, so the rule may fire just before apogee
            self._descent(z, vz, out)
            if self.state == "COAST" and vz <= 0.0:
                self.state = "APOGEE"
                self.z_apogee = z
        elif st == "APOGEE":
            self._log(f"apogee {z:.1f} m")
            self.state = "DESCENT"
        elif st == "DESCENT":
            self._descent(z, vz, out)
        elif st in ("LANDING_BURN", "TOUCHDOWN"):
            self._thrust_detect("b", 0.8, DT)
            self._burnout_detect("b")
            mm = self.m["b"]
            if st == "LANDING_BURN" and not mm.spent:
                tau = 0.0 if mm.tau0 is None else self.t - mm.tau0
                if tau < mm.tab.burn - self.p["t_lev"] and self._tick % PLAN_TICKS == 0:
                    self.cos_plan, self.pred_td, _ = self.lander.plan(
                        tau, z, vz, self.mass, self.s_hat["b"]
                    )
                self.cos_tgt = self._levelled(
                    self.cos_plan, tau, mm.tab.burn, self.p["t_lev"]
                )  # level out before the thrust ends
        if st in ("DESCENT", "LANDING_BURN") and self._contact(z, vz, accel):
            self.state = "TOUCHDOWN"
            self.touch_t = self.t
            self.goal[:] = 0.0
            self._log("touchdown")
        if self.state not in ("BOOST", "LANDING_BURN"):
            self.cos_tgt = 1.0
        self.cos_f += 0.04 * (self.cos_tgt - self.cos_f)  # ~0.1 s low pass
        # ---- attitude control while a motor is lit
        if self.state in ("BOOST", "LANDING_BURN", "TOUCHDOWN") and self.active:
            if self.state != "TOUCHDOWN" and self.att.tilt > ABORT_TILT:
                self.abort_reason = f"tilt {math.degrees(self.att.tilt):.0f} deg"
                self._log(f"abort: {self.abort_reason}")
                self.state = "ABORT"
                self.goal[:] = 0.0
            else:
                self._attitude(gyro, z, vz)
        elif self.state in ("COAST", "APOGEE", "DESCENT", "TOUCHDOWN") or (
            self.state == "LANDING_BURN" and self.m["b"].spent
        ):
            self.goal[:] = 0.0
            for k in (
                "a",
                "b",
            ):  # the thrust tail after burnout detection still pushes off-axis: hold the seat trim until the curve ends
                mm = self.m[k]
                if (
                    mm.tau0 is not None
                    and mm.spent
                    and self.t < mm.tau0 + mm.tab.burn + 0.06
                ):
                    self.goal[:] = trim_angles(k)
        out.pitch_goal, out.yaw_goal = float(self.goal[0]), float(self.goal[1])
        return out

    # --------------------------------------------------------------------------------------------------- landing rule
    def _descent(self, z, vz, out):
        """Ignition rule. Every 0.1 s: where and how fast would the vehicle be when the thrust starts if the motor were commanded at the NEXT look
        (igniter delay + 0.1 s, ballistic)? `Lander.j_onset` plays the landing out from there with the thrust scale `s_plan` (the weak side of the
        scatter) and the best tilt schedule, and returns its cost (m/s, about the hardest touchdown speed). Lighting later only makes that worse, so the
        motor is commanded at the last look at which it was still within `j_fire`. A stronger motor than planned is dumped by tilting; a weaker one is
        what `s_plan` leaves room for."""
        ln = self.lander
        if ln is None or vz > self.p["v_arm"] or self.m["b"].cmd_t is not None:
            return
        fire = z < 1.0 and vz < -3.0  # last resort: a late burn beats none
        if self._tick % PLAN_TICKS == 0:
            zo, vo = ballistic(z, vz, self.d_est + PLAN_TICKS * DT, m=self.mass)
            j2 = ln.j_onset(zo, vo, self.mass, self.p["s_plan"]) if zo > 0.0 else 99.0
            self.margin = self.p["j_fire"] - j2
            self.looks.append(
                (
                    round(self.t, 2),
                    round(z, 2),
                    round(vz, 2),
                    round(zo, 2),
                    round(vo, 2),
                    round(j2, 2),
                )
            )
            if j2 <= self.p["j_fire"]:
                self.j_armed = True
            elif self.j_armed:
                fire = True  # was feasible at the last look, is not at the next: now
            elif j2 > self.j_min + 0.5:
                fire = True  # never got within j_fire (too heavy / too fast for this motor): best effort at the minimum
            self.j_min = min(self.j_min, j2)
        if fire:
            if self.att.tilt > math.radians(60.0):
                self._log(
                    "landing burn withheld: vehicle more than 60 deg off vertical"
                )
                self.state = "ABORT"
                self.abort_reason = "tilt before landing burn"
                return
            self.m["b"].cmd_t = self.t
            out.ignite_b = True
            self.state = "LANDING_BURN"
            self.goal[:] = trim_angles("b")
            self.i_p = self.i_y = 0.0
            self._log(
                f"landing motor fired at z={z:.2f} m v={vz:.2f} m/s margin {self.margin:.2f} m/s"
            )

    def _contact(self, z, vz, accel):
        if np.linalg.norm(accel) > 3.5 * G and z < 1.5:
            return True
        b = self.m["b"]
        return b.spent and z < 0.15 and abs(vz) < 0.4

    # ------------------------------------------------------------------------------------------------ attitude control
    def _attitude(self, gyro, z, vz):
        p = self.p
        k = self.active
        w = gyro - self.att.bias
        pos = np.array([self.kf[0].pos, self.kf[1].pos])
        vel = np.array([self.kf[0].vel, self.kf[1].vel])
        if self.state == "LANDING_BURN":
            # near the ground, null the sideways speed instead of chasing the (GNSS-biased) pad position: a foot that slides tips the vehicle
            low = min(1.0, max(0.0, z / p["z_slow"]))
            a_cmd = (
                -p["kp_pos"] * low * pos
                - (p["kd_vel"] + (1 - low) * p["kd_land"]) * vel
            )
        elif self.state == "TOUCHDOWN":
            a_cmd = np.zeros(2)  # on the legs: just stand up straight, no sideways push
        else:
            a_cmd = -p["kp_pos"] * pos - p["kd_vel"] * vel
        n = float(np.linalg.norm(a_cmd))
        if n > p["a_lat_max"]:
            a_cmd *= p["a_lat_max"] / n
        fz = max(self.f_z, 6.0)
        lean = a_cmd / fz  # commanded lean of the thrust axis (world x, y), rad
        ln = float(np.linalg.norm(lean))
        lmax = math.radians(p["tilt_cmd_max_deg"])
        if ln > lmax:
            lean *= lmax / ln
            ln = lmax
        l_dump = math.tan(
            math.acos(min(self.cos_f, 1.0))
        )  # the lean the energy management wants (thrust to throw away sideways)
        if l_dump > 0.02:
            # The magnitude of the lean is set by the vertical axis (cos tilt = vertical thrust share); the guidance only chooses its direction. What the
            # position / sideways-speed law asks for comes first, the rest of the magnitude goes sideways perpendicular to it, alternating in sign so
            # that it averages out (a circle of radius a / w^2 ~ 0.4 m, not a drift).
            self.phi += 2.0 * math.pi * DT / p["alt_period"]
            if ln > l_dump:
                lean *= l_dump / ln
                ln = l_dump
            e = lean / ln if ln > 1e-6 else np.array([1.0, 0.0])
            q = math.sqrt(max(l_dump**2 - ln**2, 0.0)) * float(
                np.clip(2.5 * math.sin(self.phi), -1.0, 1.0)
            )
            lean = lean + q * np.array([-e[1], e[0]])
        zc = np.array([lean[0], lean[1], 1.0])
        vb = self.att.R.T @ (
            zc / np.linalg.norm(zc)
        )  # commanded axis in the body frame
        ex, ey = (
            -vb[1],
            vb[0],
        )  # rotation (about body x, y) that takes the body axis onto it
        wn, ze = p["wn_att"], p["zeta"]
        ax = wn * wn * ex - 2 * ze * wn * w[0]
        ay = wn * wn * ey - 2 * ze * wn * w[1]
        T = max(
            self.thrust, 4.0
        )  # gain scheduling: torque -> gimbal angle with the thrust the accelerometer says we have
        gy = I_TRANS * ax / (L_NOZ * T)  # yaw servo: torque about +x
        gp = I_TRANS * ay / (L_NOZ * T)  # pitch servo: torque about +y
        tp, ty = trim_angles(k)
        lim = GIMBAL_LIMIT
        gp_t, gy_t = tp + gp + self.i_p, ty + gy + self.i_y
        ki = (
            p["ki"] * T_NOM / T
        )  # the integrator works on torque, so its gimbal-angle gain falls with thrust
        if abs(gp_t) < lim:  # anti-windup: freeze while the servo goal is saturated
            self.i_p += ki * ey * DT
        if abs(gy_t) < lim:
            self.i_y += ki * ex * DT
        dp, dy = gp + self.i_p, gy + self.i_y
        if self.state == "TOUCHDOWN":
            # a thrust of about the vehicle's weight leaves the feet nearly unloaded: a big gimbal angle pushes them sideways along the ground
            # (a sliding foot trips the vehicle when the motor ends), so the correction is held to a couple of degrees
            c = math.radians(p["td_gimbal_deg"])
            dp, dy = float(np.clip(dp, -c, c)), float(np.clip(dy, -c, c))
        self.goal[0] = float(np.clip(tp + dp, -lim, lim))
        self.goal[1] = float(np.clip(ty + dy, -lim, lim))

    # ------------------------------------------------------------------------------------------------------ telemetry
    def telemetry(self):
        """The flight computer's own view, for the HUD (estimates only)."""
        if self.kf is None:
            return dict(state=self.state, t=round(self.t, 3))
        roll, pitch, yaw = self.att.euler()
        ph = {}
        for k in ("a", "b"):
            mm = self.m[k]
            ph[k] = (
                "idle"
                if mm.cmd_t is None
                else "delay"
                if mm.tau0 is None
                else "spent"
                if mm.spent
                else "burning"
            )
        return dict(
            t=round(self.t, 3),
            state=self.state,
            alt_m=round(self.kf[2].pos, 3),
            vz_ms=round(self.kf[2].vel, 3),
            x_m=round(self.kf[0].pos, 3),
            y_m=round(self.kf[1].pos, 3),
            vx_ms=round(self.kf[0].vel, 3),
            vy_ms=round(self.kf[1].vel, 3),
            roll_deg=round(math.degrees(roll), 2),
            pitch_deg=round(math.degrees(pitch), 2),
            yaw_deg=round(math.degrees(yaw), 2),
            tilt_deg=round(math.degrees(self.att.tilt), 2),
            pitch_goal_deg=round(math.degrees(self.goal[0]), 3),
            yaw_goal_deg=round(math.degrees(self.goal[1]), 3),
            thrust_N=round(self.thrust, 2),
            mass_kg=round(self.mass, 4),
            motor_a=ph["a"],
            motor_b=ph["b"],
            ign_margin_m=None if math.isnan(self.margin) else round(self.margin, 3),
            pred_touchdown_ms=None
            if math.isnan(self.pred_td)
            else round(self.pred_td, 2),
            dump_deg=round(math.degrees(math.acos(min(self.cos_f, 1.0))), 2),
            scale_a=round(self.s_hat["a"], 3),
            scale_b=round(self.s_hat["b"], 3),
            apogee_pred_m=None
            if math.isnan(self.apo_pred)
            else round(self.apo_pred, 2),
        )
