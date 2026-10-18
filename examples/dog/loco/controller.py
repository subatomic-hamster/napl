"""The dog's locomotion controller (SP4): what runs on the Jetson. 500 Hz: estimator -> gait -> footholds / swing -> leg control (MIT frames);
50 Hz: convex MPC. Leg control (Di Carlo 2018 "leg controller"): stance tau_ff = J^T (-R^T f_mpc), kp 0, joint damping; swing joint PD on the IK
of the swing curve, v_des = J^-1 v_foot. Modes: STANDUP -> STAND <-> WALK; any -> FALLEN (safe-sit, Task 9) -> STANDUP when upright.
Inputs: loco.hw (drivers + IMU), the command, a height map, pose fixes (SP5 stand-ins). Never the Sim, never ground truth."""

import numpy as np

from . import gait as Gt
from . import mpc as Mp
from . import safety as Sf
from . import terrain as Tr
from . import wbc as Wb
from .estimator import Estimator

DT, MPC_EVERY = 0.002, 10  # 500 Hz leg control (spec s6); MPC every 10 ticks = 50 Hz
KP_SWING, KD_SWING, KD_STANCE = (
    40.0,
    1.2,
    0.8,
)  # joint side (N m/rad, N m s/rad): ~25 Hz swing bandwidth on a ~0.05 kg m^2 leg
KP_STAND, KD_STAND = (
    100.0,
    2.0,
)  # = KP / KD of examples/dog/dogsim (the SP3b stand gains; 60 / 1.5 sagged 13 mm)
# Hip-pitch-to-foot distance as a fraction of the straight leg while walking. The SP3b stand is 0.92; walking needs foot clearance and the knee
# stops at 1.32 rad (= 0.79 of the straight leg), so a swing foot passing under the hip can rise only (H_FRAC - 0.79) x 0.465 m: 0.92 -> 6 cm, 0.95 -> 7.4 cm.
H_FRAC = 0.95
WBC_STANCE_JT = (
    True  # stance torque of the WBC legs from the MPC force through J^T, see _legs_wbc
)
H_SCHED, H_FRAC_FAST, H_V0, H_V1, H_SLEW = (
    False,
    0.90,
    0.8,
    1.6,
    0.05,
)  # speed schedule: H_FRAC up to H_V0 m/s, H_FRAC_FAST from H_V1, linear between; H_SLEW per second
LEG_CTLS = (
    "wbc",
    "jt",
)  # whole-body QP (Task 7), Jacobian-transpose leg controller (the measured baseline)
T_WARM = 0.64  # s: two trot periods of stepping in place before the velocity command is released
T_STANDUP = 1.2  # s
TD_PEN = 0.02  # m: swing curves end this far below the ground
A_CMD = 1.0  # m/s^2: slew limit of the velocity command inside the controller (2.0 fell in 1 of 8 launches at 1 m/s, 1.0 in 0 of 8)
# A scheduled-stance foot without contact yet: True = leave the MPC's stance set and reach down on the swing PD (leaves the diagonal partner as
# the only support for ~0.1 s, the trot rolls); False = stay in the stance set, the commanded force pushes the foot down to the ground.
LATE_REACH = False
LEAD = 0.0  # s: the swing reference runs this far ahead of the clock (joint PD lag)
S_LAND = 1.0  # fraction of the swing at which the swing curve reaches its target
POSTURE = 0.7  # lean 70 % into a slope (15 deg ramp, seed 1: 0.3 and 0.5 fall at the ramp foot with the J^T stance torque, 0.7 and 0.9 climb)
Z_RATE, ANG_RATE = (
    0.25,
    0.5,
)  # m/s, rad/s: fastest the terrain-derived height and posture references may move
WALK_ROUGH = False  # the walk gait for steps / steep ground: it falls on FLAT ground with both leg controllers (docs/dog/locomotion.md), so rough ground trots
REACH_LO, REACH_HI = (
    0.6,
    0.95,
)  # hip pitch point to foot distance, fraction of the straight leg: the support height keeps every foot inside


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


YAW_LEAD = 0.3  # rad: the heading reference stays this close to the actual yaw
MODES = ("STANDUP", "STAND", "WALK", "FALLEN", "SIT")


V_MAX = 40.0  # rad/s: joint velocity references stay inside what the drivers accept (v_des limit 1e4) and the motors can follow


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _jinv(J, x):
    """J^-1 x, damped near the straight-leg singularity, clipped."""
    return np.clip(np.linalg.solve(J.T @ J + 1e-4 * np.eye(3), J.T @ x), -V_MAX, V_MAX)


class Controller:
    def __init__(self, model, hw, hmap, leg_ctl="jt"):
        assert leg_ctl in LEG_CTLS, leg_ctl
        self.leg_ctl = leg_ctl
        self.wbc = Wb.make(model) if leg_ctl == "wbc" else None
        self.wbc_ref = None  # (p, v, R) of the trunk origin: the MPC's reference one step ahead, for the WBC's body tasks
        self.m, self.hw, self.map = model, hw, hmap
        self.est, self.mpc = (
            Estimator(model, hmap, DT),
            Mp.Mpc(model.mass, model.inertia),
        )
        self.sched = Gt.Scheduler("stand")
        self.legs = list(model.legs.values())
        self.mode, self.k, self.t_mode = "STANDUP", 0, 0.0
        self.f = np.zeros((4, 3))
        self.safety = Sf.Safety()  # slip -> per-foot MPC friction (Sf.MU_NOM = 0.5: below rock 0.8 / grass 0.6 / gravel 0.55, napl.terrain MATERIALS)
        self.mu = self.safety.mu
        self.q_fall0 = None
        self.h_frac = H_FRAC
        self.h_sched, self._h_cont = (
            H_SCHED,
            H_FRAC,
        )  # speed-scheduled stand height (dogwalk's explicit h_frac turns it off)
        self.forced_gait = None  # dogtop: a gait name bypasses Gt.choose (a moving command still leaves "stand")
        self._sf = None
        self.rough = False  # steps or a steep slope ahead (refreshed at 50 Hz)
        self._ref, self._k_ref = (
            None,
            0,
        )  # rate-limited terrain references (roll, pitch, support z)
        self.normals = np.tile(
            [0.0, 0.0, 1.0], (4, 1)
        )  # ground normal under each foot (friction cone axes), refreshed at 50 Hz
        self.lift = np.zeros((4, 3))  # world lift-off point of each swing
        self.target = np.zeros((4, 3))  # world touchdown target of each swing
        self.was_stance = np.ones(4, bool)
        self.t_late = np.zeros(4)
        self.cmd_f = np.zeros(3)
        self.t_gait = 0.0  # s since the gait left "stand"
        self.t_acc = 0.0  # s left of the estimator's tilt-correction pause after the velocity command last changed
        self.stats = dict(n_slip=0, mpc_fail=0, wbc_fail=0, sat_hip=0.0, sat_knee=0.0)
        self.started = False
        self._sset = (True,) * 4
        self.pose0 = (np.zeros(2), 0.0)
        self.st = None
        self.yaw_ref = None
        # (dz m, droll rad, dpitch rad) added to the MPC's body reference: posture commands, and the steps of the diagnostic ladder
        self.body_ref = np.zeros(3)

    # ---- model-derived constants ------------------------------------------------------------------------------------------------
    def stand_feet(self):
        """(4, 3) nominal feet (ball centres) in the trunk frame at the stand pose."""
        if (
            self._sf is None or self._sf[0] != self.h_frac
        ):  # 20-iteration IK on four legs: 80 % of the run time when redone every tick
            q = self.m.stand_q(self.h_frac)
            self._sf = (self.h_frac, self.m.feet(q), q)
        return self._sf[1].copy()

    def stand_q(self):
        self.stand_feet()
        return self._sf[2].copy()

    def h_stand(self):
        """Trunk origin above the ball centres at the stand pose."""
        return float(-self.stand_feet()[:, 2].mean())

    def set_pose(self, p_xy, yaw):
        """Initial world pose (the SP5 stand-in's first fix): applied at the first step."""
        self.pose0 = (np.asarray(p_xy, float), float(yaw))

    def fix(
        self, p_xy, yaw, age_s, sig_p=0.02, sig_yaw=np.radians(1.0)
    ):  # SP5 passes its own sigmas (ICP-only 0.05-0.15 m)
        self.est.fix(p_xy, yaw, age_s, sig_p, sig_yaw)

    # ---- the 500 Hz tick --------------------------------------------------------------------------------------------------------
    def step(self, cmd):
        q, dq, tau = self.hw.joints()
        gyro, acc = self.hw.imu()
        if not self.started:
            self.hw.enable()
            self.est.reset(q, *self.pose0)
            self.q0, self.started = q.copy(), True
        self.t_acc = max(0.0, self.t_acc - DT)
        st = self.st = self.est.update(
            q, dq, tau, gyro, acc, self.sched.gate(), hold=self.t_acc > 0.0
        )
        self.k += 1
        self.t_mode += DT
        if self.mode == "STANDUP":
            a = min(1.0, self.t_mode / T_STANDUP)
            tgt = self.q0 + (self.stand_q() - self.q0) * a
            self.hw.send(
                tgt,
                np.zeros(12),
                np.full(12, KP_STAND),
                np.full(12, KD_STAND),
                np.zeros(12),
            )
            if a >= 1.0:
                self._enter("STAND")
            return
        if self.mode in ("FALLEN", "SIT"):
            self._down(st, q)
            return
        if self.safety.fallen(st, self.h_stand()):
            self.q_fall0 = q.copy()
            self._enter("FALLEN")
            self._down(st, q)
            return
        raw = np.asarray(cmd, float)
        raw = np.r_[
            raw[:2] * self.safety.speed_scale, raw[2]
        ]  # a slip slows the command for T_SLOW s
        if self.k % MPC_EVERY == 0 and self.mode in ("STAND", "WALK"):
            self.rough = Tr.rough_ahead(
                self.map, st.p[:2], st.rpy[2], 0.8
            ) or Tr.steep_ahead(self.map, st.p[:2], st.rpy[2], 0.8)
        if (
            self.rough
        ):  # steps or a steep slope ahead: the walk gait, at the speed it covers
            s, vmax = np.linalg.norm(raw[:2]), Gt.GAITS["walk"].v_max
            if s > vmax:
                raw = np.r_[raw[:2] * vmax / s, raw[2]]
        # launch: the gait steps in place for T_WARM before the velocity command is released (a trot started while accelerating at 2 m/s^2 flies
        # and falls in 2 of 4 seeds; stepping in place first, 0 of 4)
        cmd = self._slew(raw if self.t_gait >= T_WARM else np.zeros(3))
        if self.h_sched:  # lower stand at speed: a straight leg at 0.95 pegs the knee torque in a fast stance; 0.9+ keeps the swing clearance
            f = np.clip((np.hypot(cmd[0], cmd[1]) - H_V0) / (H_V1 - H_V0), 0.0, 1.0)
            tgt = H_FRAC - (H_FRAC - H_FRAC_FAST) * f
            self._h_cont += float(
                np.clip(tgt - self._h_cont, -H_SLEW * DT, H_SLEW * DT)
            )
            self.h_frac = round(
                self._h_cont, 2
            )  # the IK of the stand pose is cached per value
        if self.yaw_ref is None:
            self.yaw_ref = st.rpy[2]
        d = (self.yaw_ref + cmd[2] * DT - st.rpy[2] + np.pi) % (2 * np.pi) - np.pi
        self.yaw_ref = (
            st.rpy[2] + float(np.clip(d, -YAW_LEAD, YAW_LEAD))
        )  # heading hold: the reference integrates the yaw command and never runs more than YAW_LEAD ahead of the dog
        aim = (
            cmd if Gt.choose(cmd, False) != "stand" else raw
        )  # still moving, or asked to
        moving = Gt.choose(aim, False) != "stand"
        if self.mode == "STAND" and moving:
            self._enter("WALK")
        if self.mode == "WALK":
            self.sched.request(
                (self.forced_gait or Gt.choose(aim, self.rough and WALK_ROUGH))
                if moving
                else "stand"
            )
        phases = self.sched.advance(DT, st.contact)
        if self.safety.update(st, phases, q, dq, self.legs, DT) == "slip":
            self.stats["n_slip"] += 1
        self.mu = self.safety.mu
        self.t_gait = 0.0 if self.sched.g.name == "stand" else self.t_gait + DT
        if (
            self.sched.g.name == "stand" or not LATE_REACH
        ):  # standing: every foot is a stance foot, a noisy contact estimate must not unload it
            for lp in phases:
                lp.late = False
        if self.mode == "WALK" and self.sched.g.name == "stand" and not moving:
            self._enter("STAND")
        sset = tuple(lp.stance and not lp.late for lp in phases)
        if (
            self.k % MPC_EVERY == 0 or sset != self._sset
        ):  # 50 Hz, and at once when the stance set changes (a new stance foot has no force yet)
            self._mpc(st, cmd, phases)
        self._sset = sset
        self.hw.send(
            *(self._legs_wbc if self.wbc else self._legs)(st, cmd, phases, q, dq)
        )

    def _down(self, st, q):
        """FALLEN: fold the legs to a sit over a third of T_FALL_WAIT (soft gains), then go limp (damping only); after T_FALL_WAIT stand up
        when the estimated tilt is below TILT_STANDUP, else stay down (SIT, limp). No self-righting."""
        T = Sf.T_FALL_WAIT
        if self.mode == "SIT" or self.t_mode >= T / 3:
            self.hw.send(
                q, np.zeros(12), np.zeros(12), np.full(12, Sf.KD_SIT), np.zeros(12)
            )
        else:
            a = self.t_mode / (T / 3)
            tgt = self.q_fall0 + (self.m.stand_q(0.55) - self.q_fall0) * a
            self.hw.send(
                tgt,
                np.zeros(12),
                np.full(12, Sf.KP_SIT),
                np.full(12, Sf.KD_SIT),
                np.zeros(12),
            )
        if self.mode == "FALLEN" and self.t_mode >= T:
            if np.degrees(np.hypot(st.rpy[0], st.rpy[1])) < Sf.TILT_STANDUP:
                self._restart(q, st)
            else:
                self._enter("SIT")

    def _restart(self, q, st):
        """Stand up from where it lies: a fresh estimator, scheduler, references and safety state, then STANDUP from the current joints."""
        self.est.reset(q, st.p[:2], st.rpy[2])
        self.q0 = q.copy()
        self.sched = Gt.Scheduler("stand")
        self.safety = Sf.Safety()
        self.mu = self.safety.mu
        self.wbc_ref, self._ref, self.yaw_ref = None, None, None
        self.cmd_f, self.t_gait = np.zeros(3), 0.0
        self.was_stance[:] = True
        self._sset = (True,) * 4
        self._enter("STANDUP")

    def _enter(self, mode):
        if (
            self.mode == "STANDUP"
        ):  # until the first MPC solve (<= 20 ms): carry the weight evenly
            self.f = np.tile([0.0, 0.0, self.m.mass * Mp.G / 4], (4, 1))
        self.mode, self.t_mode = mode, 0.0

    def _slew(self, cmd):
        d = cmd - self.cmd_f
        n = np.linalg.norm(d[:2])
        if (
            n > 1e-9
        ):  # accelerating: the accelerometer low-pass takes ~2 time constants to forget it
            self.t_acc = 0.8
        lim = A_CMD * DT
        if n > lim:
            d[:2] *= lim / n
        d[2] = np.clip(d[2], -4.0 * DT, 4.0 * DT)
        self.cmd_f = self.cmd_f + d
        return self.cmd_f

    def _terrain_ref(self, roll, pitch, z):
        """The terrain-derived body references (posture plane, support height) move at most Z_RATE m/s and ANG_RATE rad/s: a front foot landing on
        a ramp or a step makes them jump, and a stiff MPC answers a 10 cm height jump with a thrust that launches the dog."""
        want = np.array([roll, pitch, z])
        if self._ref is None:
            self._ref = want.copy()
        dt = max(self.k - self._k_ref, 1) * DT
        self._k_ref = self.k
        lim = np.array([ANG_RATE, ANG_RATE, Z_RATE]) * dt
        self._ref = self._ref + np.clip(want - self._ref, -lim, lim)
        return float(self._ref[0]), float(self._ref[1]), float(self._ref[2])

    def _reach_clip(self, st, yaw, pts, roll, pitch, z_foot):
        """Support height limited so that every foot (stance where it is, swing where it lands) stays within [REACH_LO, REACH_HI] of its leg's
        straight length from its hip pitch point at the posture the MPC is asked to hold. The height and the posture planes are rate-limited
        separately, so at the foot of a ramp the height reference outruns the nose-up reference and the hind legs, still on the flat, run out
        of leg (knee at 0 rad: no torque margin, the dog pitches over)."""
        Rr = Mp.rz(yaw) @ rot_y(POSTURE * pitch) @ rot_x(POSTURE * roll)
        h0 = self.h_stand()
        lo, hi = -np.inf, np.inf
        for i, L in enumerate(self.legs):
            lmax = np.linalg.norm(L.foot0 - L.p[1])
            hip = Rr @ L.p[1]
            dxy2 = np.sum((st.p[:2] + hip[:2] - pts[i, :2]) ** 2)
            up = np.sqrt(max((REACH_HI * lmax) ** 2 - dxy2, 0.0))
            dn = np.sqrt(max((REACH_LO * lmax) ** 2 - dxy2, 0.0))
            base = (
                pts[i, 2] - hip[2]
            )  # trunk origin height for the hip level with the foot
            lo, hi = max(lo, base + dn), min(hi, base + up)
        z_t = z_foot + h0
        z_t = 0.5 * (lo + hi) if lo > hi else min(max(z_t, lo), hi)
        return z_t - h0

    # ---- MPC (50 Hz) ------------------------------------------------------------------------------------------------------------
    def _mpc(self, st, cmd, phases):
        R, m = st.R, self.m
        yaw = st.rpy[2]
        c = st.p + R @ m.com
        v_c = st.v + R @ np.cross(st.w, m.com)
        x0 = np.concatenate([st.rpy, c, R @ st.w, v_c, [-Mp.G]])
        dyaw = self.yaw_ref - yaw  # |dyaw| <= YAW_LEAD (step() keeps it)
        roll_p, pitch_p, zg = Tr.plane(self.map, c[:2], yaw)
        v_w = Mp.rz(yaw) @ np.array([cmd[0], cmd[1], 0.0])
        z_foot = (
            zg + self.m.legs["fl"].r_foot
        )  # ball centres under the trunk: flat ground
        if not isinstance(self.map, Tr.FlatMap):
            # the body rides at the stand height over the feet (stance feet where they are, swing feet where they will land), not over the terrain
            # under the CoM: at the foot of a ramp the plane under the CoM is already rising while the legs are still on the flat (the legs
            # straighten, the knee torque limit binds). The posture plane goes through the same four points: with the body plane a ramp lifts the
            # nose by 15 deg while the hind feet are still on the flat and unloads them
            pts = np.array(
                [
                    st.feet[i]
                    if (lp.stance and not lp.late) or not np.any(self.target[i])
                    else self.target[i]
                    for i, lp in enumerate(phases)
                ]
            )
            ry = Mp.rz(yaw)[:2, :2].T @ (pts[:, :2] - pts[:, :2].mean(0)).T
            a, b, _ = np.linalg.lstsq(
                np.column_stack([ry.T, np.ones(4)]), pts[:, 2], rcond=None
            )[0]
            roll_p, pitch_p = float(np.arctan(b)), float(-np.arctan(a))
            z_foot = float(pts[:, 2].mean())
        roll_p, pitch_p, z_foot = self._terrain_ref(roll_p, pitch_p, z_foot)
        if not isinstance(self.map, Tr.FlatMap):
            z_foot = self._reach_clip(st, yaw, pts, roll_p, pitch_p, z_foot)
        z_ref = z_foot + self.h_stand() + m.com[2] + self.body_ref[0]
        xref = np.zeros((Mp.N, Mp.NX))
        r = np.zeros((Mp.N, 4, 3))
        # standing: the CoM is held over the feet (a reference that follows the CoM leaves the position loop open and the body drifts out of the
        # support polygon after a push); walking: the position reference follows the CoM, only the velocity is tracked
        base_xy = st.feet[:, :2].mean(0) if self.sched.g.name == "stand" else c[:2]
        # on a slope the trunk climbs with the ground: z rate = ground slope (yaw frame) x commanded velocity (0.0 on flat ground)
        vz = float(np.tan(-pitch_p) * cmd[0] + np.tan(roll_p) * cmd[1]) + 0.0
        for k in range(Mp.N):
            t = (k + 1) * Mp.DT
            xref[k] = [POSTURE * roll_p + self.body_ref[1], POSTURE * pitch_p + self.body_ref[2], yaw + dyaw + cmd[2] * t, base_xy[0] + v_w[0] * t, base_xy[1] + v_w[1] * t, z_ref + vz * t,
                       0.0, 0.0, cmd[2], v_w[0], v_w[1], vz, -Mp.G]  # fmt: skip
        # lever arms about where the CoM will actually be (the current CoM carried along at the commanded velocity), not about the position
        # REFERENCE: standing with the CoM 1 cm off the reference, the reference-based arms miss the m g x offset torque and the MPC balances a
        # body that is not there (the slow lateral divergence of rung b)
        ts = (np.arange(Mp.N) + 1) * Mp.DT
        com_path = np.column_stack([c[0] + v_w[0] * ts, c[1] + v_w[1] * ts, xref[:, 5]])
        for i, lp in enumerate(phases):
            foot = st.feet[i] if lp.stance else self.target[i]
            r[:, i] = foot - com_path
            self.normals[i] = Tr.normal(
                self.map, foot[:2]
            )  # the friction cone of a foot is about the ground normal under it
        f, ok = self.mpc.solve(
            x0, xref, r, self.sched.table(Mp.N, Mp.DT), self.mu, self.normals
        )
        self.f = f
        self.stats["mpc_fail"] = self.mpc.n_fail
        rr, rp, ry = xref[0, 0], xref[0, 1], xref[0, 2]
        R_ref = Mp.rz(ry) @ _ry(rp) @ _rx(rr)
        self.wbc_ref = (xref[0, 3:6] - R_ref @ m.com, xref[0, 9:12].copy(), R_ref)

    # ---- leg control (500 Hz) ---------------------------------------------------------------------------------------------------
    def _swing_ref(self, i, L, lp, st, cmd, r):
        """World foot position / velocity reference of a swing (or late-touchdown) leg."""
        R, p, v, w = st.R, st.p, st.v, st.w
        yaw = st.rpy[2]
        v_cmd_w = Mp.rz(yaw) @ [cmd[0], cmd[1], 0.0]
        if self.was_stance[i] or not np.any(self.target[i]):
            self.lift[i] = p + R @ r
        nom = self.stand_feet()[i].copy()
        nom[1] += (
            np.sign(nom[1]) * self.safety.widen
        )  # after a slip: a wider stance (outward of the body centre line)
        hip_td = p + R @ np.array([nom[0], nom[1], 0.0]) + v_cmd_w * lp.t_to_td
        xy = Gt.nominal_foothold(
            hip_td,
            st.v,
            v_cmd_w,
            cmd[2],
            self.sched.g.duty * self.sched.g.period,
            self.h_stand(),
        )
        tgt = Tr.foothold(self.map, xy, L.r_foot)
        if not np.isfinite(tgt[2]):
            tgt[2] = self.lift[i][2]  # unknown map: keep the lift-off height
        self.target[i] = tgt
        apex = Tr.apex_over(self.map, self.lift[i], self.target[i], self.sched.g.apex)
        aim = (
            self.target[i] - [0.0, 0.0, TD_PEN]
        )  # aim through the ground: the foot arrives with speed, touchdown comes before s = 1
        x_w, v_w = Gt.swing(
            self.lift[i],
            aim,
            apex,
            min((lp.s + LEAD / lp.t_swing) / S_LAND, 1.0),
            lp.t_swing * S_LAND,
        )  # the curve ends at S_LAND of the swing: the joint PD lags the reference, the foot must be down when the stance is scheduled
        if lp.s > S_LAND:
            v_w = np.zeros(3)
        if lp.late:  # scheduled stance, no contact yet: keep reaching down, at most 6 cm below the plan
            x_w = aim - [0.0, 0.0, min(0.06, 0.4 * self.t_late[i])]
            v_w = np.array([0.0, 0.0, -0.4])
        return x_w, v_w

    def _legs(self, st, cmd, phases, q, dq):
        R, p, v, w = st.R, st.p, st.v, st.w
        p_des, v_des, kp, kd, tau = (
            q.copy(),
            np.zeros(12),
            np.zeros(12),
            np.full(12, KD_STANCE),
            np.zeros(12),
        )
        for i, (L, lp) in enumerate(zip(self.legs, phases)):
            s = slice(3 * i, 3 * i + 3)
            r, J = L.fk_jac(q[s])
            self.t_late[i] = self.t_late[i] + DT if lp.late else 0.0
            if lp.stance and not lp.late:
                # ground force f at the foot: -J^T R^T f, plus the torque that carries the leg's own weight (without it the realized ground
                # force is off by the leg weight, the feet walked inward at 1.3 cm/s and the stand fell at 13 s: rung b)
                tau[s] = J.T @ (-R.T @ self.f[i]) + self.m.leg_required(
                    i, q[s], np.zeros(3), R.T @ np.array([0.0, 0.0, Mp.G])
                )
                v_des[s] = _jinv(
                    J, -(R.T @ v + np.cross(w, r))
                )  # foot still in the world
                self.was_stance[i] = True
                continue
            x_w, v_w = self._swing_ref(i, L, lp, st, cmd, r)
            self.was_stance[i] = False
            x_b = L.reach(R.T @ (x_w - p))
            p_des[s] = L.ik(x_b, q[s], iters=4, lam=1e-3, step=0.3)
            v_des[s] = _jinv(J, R.T @ (v_w - v) - np.cross(w, r))
            kp[s], kd[s] = KP_SWING, KD_SWING
        lim = np.concatenate([L.tau_max for L in self.legs])
        sat = np.abs(tau) >= 0.95 * lim
        self.stats["sat_hip"] = (
            0.99 * self.stats["sat_hip"] + 0.01 * sat[1::3].mean()
        )  # ~0.2 s moving average
        self.stats["sat_knee"] = 0.99 * self.stats["sat_knee"] + 0.01 * sat[2::3].mean()
        kpm = np.concatenate([L.kp_max for L in self.legs])
        return p_des, v_des, np.minimum(kp, kpm), kd, np.clip(tau, -lim, lim)

    def _legs_wbc(self, st, cmd, phases, q, dq):
        stance = np.array([lp.stance and not lp.late for lp in phases])
        sx, sv = np.zeros((4, 3)), np.zeros((4, 3))
        for i, (L, lp) in enumerate(zip(self.legs, phases)):
            r = L.fk_jac(q[3 * i : 3 * i + 3])[0]
            self.t_late[i] = self.t_late[i] + DT if lp.late else 0.0
            if stance[i]:
                self.lift[i] = st.p + st.R @ r
                self.was_stance[i] = True
            else:
                x_w, sv[i] = self._swing_ref(i, L, lp, st, cmd, r)
                sx[i] = st.p + st.R @ L.reach(
                    st.R.T @ (x_w - st.p)
                )  # an unreachable target would flail the abduction joint
                self.was_stance[i] = False
        if self.wbc_ref is None:
            self.wbc_ref = (st.p.copy(), np.zeros(3), Mp.rz(st.rpy[2]))
        out = self.wbc.solve(
            st.p,
            st.R,
            st.R @ st.w,
            st.v,
            q,
            dq,
            self.f,
            stance,
            self.mu,
            self.wbc_ref,
            (sx, sv),
            self.normals,
        )
        self.stats["wbc_relaxed"] = (
            self.wbc.n_relaxed
        )  # ticks that needed the torque rows dropped
        if not out.ok:
            self.stats["wbc_fail"] += 1
            self.stats["wbc_status"] = (
                self.wbc.status
            )  # OSQP status of the last failure (diagnosis)
            return self._legs(st, cmd, phases, q, dq)  # this tick on J^T
        sw = np.repeat(~stance, 3)
        kpm = np.concatenate([L.kp_max for L in self.legs])
        kp = np.where(sw, np.minimum(KP_SWING, kpm), 0.0)
        kd = np.where(sw, KD_SWING, KD_STANCE)
        tau = out.tau.copy()
        if WBC_STANCE_JT:
            # Stance joints carry the MPC force through J^T (plus the leg's own weight), as in the J^T leg controller; the WBC supplies the swing
            # legs' torques and every joint's velocity target. With the WBC's own stance torque (dynamically consistent, with the leg
            # accelerations of the body tasks and the QP's force change) a 0.4 m/s lateral push at 24 phases / seeds leaves 5 dogs standing, with
            # this one 24 of 24 (J^T alone: 21 of 24; docs/dog/locomotion.md "Flat ground robustness").
            R = st.R
            for i, L in enumerate(self.legs):
                if stance[i]:
                    s = slice(3 * i, 3 * i + 3)
                    J = L.fk_jac(q[s])[1]
                    tau[s] = J.T @ (-R.T @ self.f[i]) + self.m.leg_required(
                        i, q[s], np.zeros(3), R.T @ np.array([0.0, 0.0, Mp.G])
                    )
        return np.where(sw, out.p_des, q), out.v_des, kp, kd, tau
