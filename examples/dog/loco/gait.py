"""Gait scheduler, swing curves and nominal footsteps of the dog (SP4).
Phase clock: phase += dt / period; leg i is in scheduled stance while (phase + offset_i) mod 1 < duty. A gait change waits for the phase wrap.
Early touchdown (contact in the second half of a swing) turns the leg to stance until its scheduled stance; late touchdown (scheduled stance,
no contact) keeps it out of the MPC's stance set and lets the swing curve keep descending (loco.controller). Footsteps: Raibert (1986)
v T_stance / 2 plus the capture-point feedback sqrt(h / g) (v - v_cmd) and the turning term, as in the MIT Cheetah software."""

from dataclasses import dataclass, replace

import numpy as np

G = 9.81
K_CAP = 0.15  # x sqrt(h/g): capture-point feedback gain (1.0 moved the touchdown target 5-20 cm within one swing on the velocity noise and the trot fell; MIT's 0.03 s is 0.14 here)
TD_TICKS = 3  # control ticks (6 ms) of continuous contact that make an early touchdown
D_MAX = 0.18  # m: farthest touchdown from the hip (leg workspace at the stand height, ~0.4 of the leg length)


@dataclass(frozen=True)
class Gait:
    name: str
    period: float  # s
    duty: float  # stance fraction of the period
    offsets: tuple  # phase offset per leg (fl, fr, rl, rr)
    apex: float  # m above the higher end of the swing
    v_max: float  # m/s: choose() takes the slowest gait whose v_max covers the command


# Starting points (Mini Cheetah / Cheetah 3 class values scaled to 12 kg, 0.22 m links); Task 10 tunes trot / flying_trot / bound on TRAIN seeds and
# writes the chosen numbers back here with the run that chose them.
GAITS = {
    "stand": Gait("stand", 0.5, 1.0, (0.0, 0.0, 0.0, 0.0), 0.0, 0.0),
    "walk": Gait(
        "walk", 0.6, 0.75, (0.0, 0.5, 0.75, 0.25), 0.10, 0.5
    ),  # three feet down most of the time: steps, ledges
    "trot": Gait(
        "trot", 0.32, 0.5, (0.0, 0.5, 0.5, 0.0), 0.05, 2.2
    ),  # apex: the knee stop limits the lift under the hip to ~7 cm. Duty 0.58 (double support 26 ms at each swap) took a 0.4 m/s lateral push at
    # 24 phases x seeds from 5 to 17 of 24 stay up WITHOUT IMU bias / payload, but 1 of 8 vs 5 of 8 on the full dispersion: not adopted (docs/dog/locomotion.md)
    "flying_trot": Gait(
        "flying_trot", 0.28, 0.38, (0.0, 0.5, 0.5, 0.0), 0.08, 3.2
    ),  # flight phases: shorter, harder stance per step
    "bound": Gait(
        "bound", 0.26, 0.40, (0.0, 0.0, 0.5, 0.5), 0.07, 9.9
    ),  # front pair / rear pair
}
ORDER = ("trot", "flying_trot", "bound")


def choose(v_cmd, rough):
    s = float(np.hypot(v_cmd[0], v_cmd[1]))
    if s < 0.05 and abs(v_cmd[2]) < 0.05:
        return "stand"
    if rough:
        return "walk"
    return next(g for g in ORDER if s <= GAITS[g].v_max or g == ORDER[-1])


@dataclass
class LegPhase:
    stance: bool
    s: float  # swing progress 0..1 (0 in stance)
    t_swing: float  # s: swing duration of the current gait
    t_to_td: float  # s: until the next scheduled touchdown (0 in stance)
    early: bool
    late: bool


class Scheduler:
    def __init__(self, gait="stand"):
        self.gaits = dict(GAITS)
        self.g, self.next, self.phase = self.gaits[gait], None, 0.0
        self.early, self._legs, self.run = [False] * 4, [], [0] * 4

    def set_params(self, name, **kw):
        self.gaits[name] = replace(self.gaits[name], **kw)
        if self.g.name == name:
            self.g = self.gaits[name]

    def request(self, name):
        if name != self.g.name:
            self.next = self.gaits[name]

    def advance(self, dt, contact):
        p = self.phase + dt / self.g.period
        if p >= 1.0 or (self.next is not None and self.g.name == "stand"):
            p %= 1.0
            if self.next is not None:
                self.g, self.next = self.next, None
        self.phase = p
        g, legs = self.g, []
        for i in range(4):
            ph = (p + g.offsets[i]) % 1.0
            sched = ph < g.duty
            s = 0.0 if sched else (ph - g.duty) / (1.0 - g.duty)
            self.run[i] = self.run[i] + 1 if contact[i] else 0
            if sched:
                self.early[i] = False
            else:  # early touchdown: contact held for TD_TICKS ticks in the second half of a swing (one noisy estimate must not stand a leg up)
                self.early[i] = s > 0.5 and self.run[i] >= TD_TICKS
            t_sw = (1.0 - g.duty) * g.period
            legs.append(
                LegPhase(
                    sched or self.early[i],
                    s,
                    t_sw,
                    0.0 if sched else (1.0 - s) * t_sw,
                    self.early[i],
                    sched and not contact[i],
                )
            )
        self._legs = legs
        return legs

    def gate(self):
        return np.array(
            [not lp.stance and lp.s < 0.5 for lp in self._legs] or [False] * 4
        )

    def table(self, n, dt):
        """(n, 4) scheduled stance (1) / swing (0) over the MPC horizon; step 0 uses the legs' current state (early / late touchdown)."""
        g, out = self.g, np.zeros((n, 4))
        for k in range(n):
            ph = (self.phase + k * dt / g.period + np.array(g.offsets)) % 1.0
            out[k] = ph < g.duty
        if self._legs:
            out[0] = [lp.stance and not lp.late for lp in self._legs]
        return out


def _mj(a):
    return a**3 * (10 - 15 * a + 6 * a * a), 30 * a * a * (
        1 - a
    ) ** 2  # min-jerk value and d/da


def swing(p0, p1, apex, s, T):
    """Foot position / velocity at swing progress s over T s: xy min-jerk p0 -> p1; z min-jerk up to max(p0z, p1z) + apex at s = 0.5, then down."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    T = max(T, 1e-3)
    b, db = _mj(min(max(s, 0.0), 1.0))
    xy, vxy = p0[:2] + (p1[:2] - p0[:2]) * b, (p1[:2] - p0[:2]) * db / T
    top = max(p0[2], p1[2]) + apex
    a, z0, z1 = (2 * s, p0[2], top) if s < 0.5 else (2 * s - 1, top, p1[2])
    bz, dbz = _mj(min(max(a, 0.0), 1.0))
    return np.array([*xy, z0 + (z1 - z0) * bz]), np.array(
        [*vxy, (z1 - z0) * dbz * 2 / T]
    )


def nominal_foothold(hip_w, v, v_cmd_w, wz, t_stance, h):
    k = np.sqrt(h / G)
    d = np.asarray(v[:2]) * t_stance / 2 + K_CAP * k * (
        np.asarray(v[:2]) - np.asarray(v_cmd_w[:2])
    )
    d = d + 0.5 * k * np.cross(v, [0.0, 0.0, wz])[:2]
    n = np.linalg.norm(d)
    return np.asarray(hip_w[:2]) + (d if n <= D_MAX else d * D_MAX / n)
