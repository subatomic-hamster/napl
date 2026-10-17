"""The dog's locomotion controller (SP4): what runs on the Jetson. 500 Hz: estimator -> gait -> footholds / swing -> leg control (MIT frames);
50 Hz: convex MPC. Leg control (Di Carlo 2018 "leg controller"): stance tau_ff = J^T (-R^T f_mpc), kp 0, joint damping; swing joint PD on the IK
of the swing curve, v_des = J^-1 v_foot. Modes: STANDUP -> STAND <-> WALK; any -> FALLEN (safe-sit, Task 9) -> STANDUP when upright.
Inputs: loco.hw (drivers + IMU), the command, a height map, pose fixes (SP5 stand-ins). Never the Sim, never ground truth."""

import numpy as np

from . import gait as Gt
from . import mpc as Mp
from . import terrain as Tr
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
T_WARM = 0.64  # s: two trot periods of stepping in place before the velocity command is released
T_STANDUP = 1.2  # s
MU_NOM = 0.5  # MPC friction: below rock 0.8 / grass 0.6 / gravel 0.55 (napl.terrain MATERIALS); slip handling lowers it (Task 9)
TD_PEN = 0.02  # m: swing curves end this far below the ground
A_CMD = 1.0  # m/s^2: slew limit of the velocity command inside the controller (2.0 fell in 1 of 8 launches at 1 m/s, 1.0 in 0 of 8)
# A scheduled-stance foot without contact yet: True = leave the MPC's stance set and reach down on the swing PD (leaves the diagonal partner as
# the only support for ~0.1 s, the trot rolls); False = stay in the stance set, the commanded force pushes the foot down to the ground.
LATE_REACH = False
LEAD = 0.0  # s: the swing reference runs this far ahead of the clock (joint PD lag)
S_LAND = 1.0  # fraction of the swing at which the swing curve reaches its target
POSTURE = 0.7  # lean 70 % into a slope
YAW_LEAD = 0.3  # rad: the heading reference stays this close to the actual yaw
MODES = ("STANDUP", "STAND", "WALK", "FALLEN", "SIT")


V_MAX = 40.0  # rad/s: joint velocity references stay inside what the drivers accept (v_des limit 1e4) and the motors can follow


def _jinv(J, x):
    """J^-1 x, damped near the straight-leg singularity, clipped."""
    return np.clip(np.linalg.solve(J.T @ J + 1e-4 * np.eye(3), J.T @ x), -V_MAX, V_MAX)


class Controller:
    def __init__(self, model, hw, hmap):
        self.m, self.hw, self.map = model, hw, hmap
        self.est, self.mpc = (
            Estimator(model, hmap, DT),
            Mp.Mpc(model.mass, model.inertia),
        )
        self.sched = Gt.Scheduler("stand")
        self.legs = list(model.legs.values())
        self.mode, self.k, self.t_mode = "STANDUP", 0, 0.0
        self.f = np.zeros((4, 3))
        self.mu = np.full(4, MU_NOM)
        self.h_frac = H_FRAC
        self.lift = np.zeros((4, 3))  # world lift-off point of each swing
        self.target = np.zeros((4, 3))  # world touchdown target of each swing
        self.was_stance = np.ones(4, bool)
        self.t_late = np.zeros(4)
        self.cmd_f = np.zeros(3)
        self.t_gait = 0.0  # s since the gait left "stand"
        self.t_acc = 0.0  # s left of the estimator's tilt-correction pause after the velocity command last changed
        self.stats = dict(n_slip=0, mpc_fail=0, sat_hip=0.0, sat_knee=0.0)
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
        return self.m.feet(self.m.stand_q(self.h_frac))

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
            tgt = self.q0 + (self.m.stand_q(self.h_frac) - self.q0) * a
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
        raw = np.asarray(cmd, float)
        # launch: the gait steps in place for T_WARM before the velocity command is released (a trot started while accelerating at 2 m/s^2 flies
        # and falls in 2 of 4 seeds; stepping in place first, 0 of 4)
        cmd = self._slew(raw if self.t_gait >= T_WARM else np.zeros(3))
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
            rough = Tr.rough_ahead(self.map, st.p[:2], st.rpy[2], 0.8)
            self.sched.request(Gt.choose(aim, rough) if moving else "stand")
        phases = self.sched.advance(DT, st.contact)
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
        self.hw.send(*self._legs(st, cmd, phases, q, dq))

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
        z_ref = (
            zg + self.m.legs["fl"].r_foot + self.h_stand() + m.com[2] + self.body_ref[0]
        )
        xref = np.zeros((Mp.N, Mp.NX))
        r = np.zeros((Mp.N, 4, 3))
        # standing: the CoM is held over the feet (a reference that follows the CoM leaves the position loop open and the body drifts out of the
        # support polygon after a push); walking: the position reference follows the CoM, only the velocity is tracked
        base_xy = st.feet[:, :2].mean(0) if self.sched.g.name == "stand" else c[:2]
        for k in range(Mp.N):
            t = (k + 1) * Mp.DT
            xref[k] = [POSTURE * roll_p + self.body_ref[1], POSTURE * pitch_p + self.body_ref[2], yaw + dyaw + cmd[2] * t, base_xy[0] + v_w[0] * t, base_xy[1] + v_w[1] * t, z_ref,
                       0.0, 0.0, cmd[2], v_w[0], v_w[1], 0.0, -Mp.G]  # fmt: skip
        # lever arms about where the CoM will actually be (the current CoM carried along at the commanded velocity), not about the position
        # REFERENCE: standing with the CoM 1 cm off the reference, the reference-based arms miss the m g x offset torque and the MPC balances a
        # body that is not there (the slow lateral divergence of rung b)
        ts = (np.arange(Mp.N) + 1) * Mp.DT
        com_path = np.column_stack([c[0] + v_w[0] * ts, c[1] + v_w[1] * ts, xref[:, 5]])
        for i, lp in enumerate(phases):
            foot = st.feet[i] if lp.stance else self.target[i]
            r[:, i] = foot - com_path
        f, ok = self.mpc.solve(x0, xref, r, self.sched.table(Mp.N, Mp.DT), self.mu)
        self.f = f
        self.stats["mpc_fail"] = self.mpc.n_fail

    # ---- leg control (500 Hz) ---------------------------------------------------------------------------------------------------
    def _swing_ref(self, i, L, lp, st, cmd, r):
        """World foot position / velocity reference of a swing (or late-touchdown) leg."""
        R, p, v, w = st.R, st.p, st.v, st.w
        yaw = st.rpy[2]
        v_cmd_w = Mp.rz(yaw) @ [cmd[0], cmd[1], 0.0]
        if self.was_stance[i] or not np.any(self.target[i]):
            self.lift[i] = p + R @ r
        nom = self.stand_feet()[i]
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
