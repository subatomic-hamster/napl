"""Slip and fall handling of the dog (SP4).
Slip: a stance foot (estimated contact, not the first 20 ms after touchdown) whose world velocity v + R (J dq + w x r) exceeds SLIP_V for
SLIP_TICKS ticks: lower that foot's MPC friction by MU_DROP (floor MU_MIN), widen the stance by WIDEN, scale the speed command to SPEED_SLIP
for T_SLOW s; mu recovers at MU_RECOVER per s (Focchi et al. 2018 slip detection / friction re-estimation, simplified to a per-foot step).
Fall: estimated tilt > TILT_FALL, or tilt > TILT_FALL_FAST while rolling faster than W_FALL, or the body below 50 % of the stand height over
the map -> FALLEN: legs fold to a sit (KP_SIT / KD_SIT) then go limp (damping only); after T_FALL_WAIT, stand up if tilt < TILT_STANDUP.
No self-righting (a fall is a fall)."""

import numpy as np

SLIP_V, SLIP_TICKS = (
    0.25,
    40,
)  # m/s, ticks (80 ms): above the estimator's ~0.05 m/s foot-velocity noise AND above its lag after a push. A sideways kick of 0.5 m/s leaves the
# body velocity estimate 0.2-0.4 m/s short for ~50 ms, so every planted foot "slides" by that much for up to 26 ticks (the plan's 3 ticks read the kick
# as a slip on three feet, dropped mu to 0.35 / 0.21 and slowed the command to 0.6: the dog fell; docs/dog/locomotion.md)
MU_NOM, MU_MIN, MU_DROP, MU_RECOVER = (
    0.5,
    0.2,
    0.15,
    0.05,
)  # mud is 0.25 (napl.terrain MATERIALS)
SPEED_SLIP, T_SLOW, WIDEN = 0.6, 2.0, 0.03
TILT_FALL, TILT_FALL_FAST, W_FALL = 40.0, 25.0, 4.0  # deg, deg, rad/s
TILT_STANDUP, T_FALL_WAIT = 30.0, 1.5
KP_SIT, KD_SIT = (
    20.0,
    1.0,
)  # soft: a falling leg must not slam (kd_timeout of the drivers is 1.0, SP3b)


class Safety:
    def __init__(self):
        self.mu, self.speed_scale, self.widen = np.full(4, MU_NOM), 1.0, 0.0
        self.count, self.t_slow, self.t_stance = np.zeros(4, int), 0.0, np.zeros(4)

    def update(self, st, phases, q, dq, legs, dt):
        ev = None
        for i, (L, lp) in enumerate(zip(legs, phases)):
            stance = lp.stance and st.contact[i]
            self.t_stance[i] = self.t_stance[i] + dt if stance else 0.0
            if stance and self.t_stance[i] > 0.02:
                r, J = L.fk_jac(q[3 * i : 3 * i + 3])
                vf = st.v + st.R @ (J @ dq[3 * i : 3 * i + 3] + np.cross(st.w, r))
                self.count[i] = (
                    self.count[i] + 1 if np.linalg.norm(vf[:2]) > SLIP_V else 0
                )
                if self.count[i] == SLIP_TICKS:
                    self.mu[i] = max(MU_MIN, self.mu[i] - MU_DROP)
                    self.t_slow, self.widen, ev = T_SLOW, WIDEN, "slip"
            elif not stance:
                self.count[i] = (
                    0  # a count does not carry over a swing (it would block the recovery of mu)
                )
        self.mu = np.minimum(MU_NOM, self.mu + MU_RECOVER * dt * (self.count == 0))
        self.t_slow = max(0.0, self.t_slow - dt)
        self.speed_scale = SPEED_SLIP if self.t_slow > 0 else 1.0
        self.widen = self.widen if self.t_slow > 0 else 0.0
        return ev

    def fallen(self, st, h_stand):
        tilt = np.degrees(np.hypot(st.rpy[0], st.rpy[1]))
        return bool(
            tilt > TILT_FALL
            or (tilt > TILT_FALL_FAST and np.linalg.norm(st.w[:2]) > W_FALL)
            or st.h_map < 0.5 * h_stand
        )
