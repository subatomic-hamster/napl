"""Local planner: sampled motion primitives scored against the depth-built map. numpy only.

Every replan (at the depth rate) the planner builds a few hundred candidate trajectories from the CURRENT state estimate (position p, velocity v): each
ramps the velocity to a target `vdes` at the acceleration limit, holds it until the horizon, then brakes to a stop at `a_brake`. A candidate is flown
only if its WHOLE path, braking tail included, (1) keeps `r_safe` from every occupied voxel, (2) lies in space the camera has already seen to be free
(a voxel a ray crossed; unknown space is never entered, except within `exempt` of the vehicle where the camera is blind), (3) stays in the altitude
band. So a stop that avoids everything is always available while the vehicle follows the plan, and a replan that finds nothing falls back to the
braking primitive, then to turning on the spot to look. Among the safe candidates an objective picks the cheapest (goal seeking or subject following).

    pl = Planner(Params())
    plan = pl.plan(p, v, yaw, window, objective)      # Plan | None; plan.at(tau) -> (p, v, a) along the chosen primitive
"""

from dataclasses import dataclass, field

import numpy as np
from mapping import Window


@dataclass
class Params:
    v_cruise: float = 5.0  # m/s: fastest candidate
    a_max: float = 3.2  # m/s^2: acceleration asked of the controller (tilt atan(3.2 / 9.81) = 18 deg)
    a_brake: float = 3.0  # m/s^2: braking in the safety tail
    a_emerg: float = 4.6  # m/s^2: braking when no safe primitive exists (tilt atan(4.6 / 9.81) = 25 deg, inside the Flyer's 26 deg limit)
    horizon: float = 1.4  # s of constant-velocity plan before the braking tail
    dt: float = 0.1  # s between trajectory samples
    r_safe: float = 0.62  # m: x500 prop-tip radius 0.38 + 0.24 for pose error, tracking error and map quantisation
    z_min: float = 0.9
    z_max: float = 3.4
    z_cruise: float = 2.0
    exempt: float = 1.0  # m around the vehicle where unknown cells are accepted (below the camera's minimum range / outside its view)
    clear_pref: float = 1.6  # m: clearance above which a wider berth is not rewarded
    w_clear: float = 1.5
    w_heading: float = (
        0.8  # per rad between the candidate's direction and the camera heading
    )
    w_smooth: float = (
        0.35  # per m/s of change of the commanded velocity between replans
    )
    w_alt: float = 0.5  # per m of altitude error at the end of the plan
    speeds: tuple = (1.0, 2.5, 4.0, 5.0)
    az_goal: tuple = tuple(
        np.radians([0, 6, 12, 19, 27, 36, 46, 58, 72, 90, 115, 150, 180])
    )
    az_cam: tuple = tuple(np.radians([0, 8, 16, 24, 32, 40]))


@dataclass
class Plan:
    p0: np.ndarray
    v0: np.ndarray
    vdes: np.ndarray
    a_max: float
    a_brake: float
    horizon: float
    mode: str = "track"  # track | brake | escape | scan
    cost: float = 0.0
    min_clear: float = 0.0
    telemetry: dict = field(default_factory=dict)

    def __post_init__(self):
        dv = self.vdes - self.v0
        n = float(np.linalg.norm(dv))
        self.ta = n / self.a_max
        self.dir = dv / n if n > 1e-9 else np.zeros(3)
        self.p_h, self.v_h, _ = self._ramp(self.horizon)
        sp = float(np.linalg.norm(self.v_h))
        self.tb = sp / self.a_brake
        self.u = self.v_h / sp if sp > 1e-9 else np.zeros(3)

    def _ramp(self, t):
        tc = min(t, self.ta)
        p = (
            self.p0
            + self.v0 * tc
            + 0.5 * self.a_max * self.dir * tc**2
            + self.vdes * (t - tc)
        )
        v = self.v0 + self.a_max * self.dir * tc
        return p, v, (self.a_max * self.dir if t < self.ta else np.zeros(3))

    def at(self, tau):
        """(position, velocity, acceleration) of the primitive tau seconds after it was planned; after the stop it stays put."""
        if tau <= self.horizon:
            return self._ramp(tau)
        s = min(tau - self.horizon, self.tb)
        a = -self.a_brake * self.u if tau - self.horizon < self.tb else np.zeros(3)
        return (
            self.p_h + self.v_h * s - 0.5 * self.a_brake * self.u * s * s,
            self.v_h - self.a_brake * self.u * s,
            a,
        )

    @property
    def duration(self):
        return self.horizon + self.tb


def rollout(p0, v0, vdes, prm: Params):
    """Batch of primitives: vdes (C, 3) -> points (C, K, 3) at times (K,) (horizon samples, then brake samples), accel-limited ramp + brake."""
    C = len(vdes)
    t_h = np.arange(1, int(round(prm.horizon / prm.dt)) + 1) * prm.dt
    dv = vdes - v0
    n = np.linalg.norm(dv, axis=1)
    ta = n / prm.a_max
    d = np.where(n[:, None] > 1e-9, dv / np.maximum(n, 1e-9)[:, None], 0.0)
    tc = np.minimum(t_h[None, :], ta[:, None])  # (C, Kh)
    P = (
        p0
        + v0 * tc[..., None]
        + 0.5 * prm.a_max * d[:, None, :] * tc[..., None] ** 2
        + vdes[:, None, :] * (t_h[None, :, None] - tc[..., None])
    )
    tcH = np.minimum(prm.horizon, ta)
    v_h = v0 + prm.a_max * d * tcH[:, None]
    p_h = P[:, -1]
    sp = np.linalg.norm(v_h, axis=1)
    tb = sp / prm.a_brake
    u = np.where(sp[:, None] > 1e-9, v_h / np.maximum(sp, 1e-9)[:, None], 0.0)
    nb = int(np.ceil((prm.v_cruise * 1.5) / prm.a_brake / prm.dt)) + 1
    s = np.minimum(np.arange(1, nb + 1)[None, :] * prm.dt, tb[:, None])
    B = (
        p_h[:, None, :]
        + v_h[:, None, :] * s[..., None]
        - 0.5 * prm.a_brake * u[:, None, :] * s[..., None] ** 2
    )
    times = np.concatenate([t_h, prm.horizon + np.arange(1, nb + 1) * prm.dt])
    return np.concatenate([P, B], axis=1), times


class Planner:
    def __init__(self, prm: Params = None):
        self.prm = prm or Params()
        self.prev_vdes = None

    def candidates(self, p, v, yaw, bearing, extra=()):
        """Target velocities (C, 3): directions fanned around the bearing to the goal and around the camera heading, four speeds, three climb rates."""
        prm = self.prm
        offs = np.array(prm.az_goal)
        az_g = bearing + np.concatenate([offs, -offs[1:]])
        offc = np.array(prm.az_cam)
        az_c = yaw + np.concatenate([offc, -offc[1:]])
        az = np.concatenate([az_g, az_c])
        vz0 = float(np.clip(1.0 * (prm.z_cruise - p[2]), -1.2, 1.2))
        out = [np.zeros(3), np.array([0.0, 0.0, vz0])]
        for s in prm.speeds:
            for vz in (vz0, vz0 - 1.0, vz0 + 1.0):
                out += [np.array([s * np.cos(a), s * np.sin(a), vz]) for a in az]
        out += [np.asarray(e, float) for e in extra]
        return np.array(out)

    def plan(self, p, v, yaw, win: Window, bearing, objective, extra=()):
        """-> Plan. `objective(P, times, vdes)` -> cost (C,) over the safe candidates' paths (lower is better)."""
        prm = self.prm
        vdes = self.candidates(p, v, yaw, bearing, extra)
        P, times = rollout(p, v, vdes, prm)
        C, K, _ = P.shape
        flat = P.reshape(-1, 3)
        clear = win.clearance(flat).reshape(C, K)
        known = win.is_known_free(flat).reshape(C, K)
        near = (np.linalg.norm(P - p, axis=2) < prm.exempt) | (
            np.linalg.norm(P[:, :, :2] - p[:2], axis=2) < prm.exempt
        )
        near_clear = np.where(
            near, clear, 99.0
        )  # inside the exempt radius only the occupied test applies
        alt_ok = ((P[:, :, 2] >= prm.z_min) & (P[:, :, 2] <= prm.z_max)).all(axis=1)
        collide_free = (clear >= prm.r_safe).all(axis=1) & alt_ok
        seen = (known | near).all(axis=1)
        min_clear = clear.min(axis=1)
        safe = collide_free & seen
        mode = "track"
        if not safe.any():  # nothing safe through known space: allow unknown beyond 1.5 s at crawl speed, then brake, then look around
            slow = np.linalg.norm(vdes, axis=1) <= 1.0
            safe2 = (
                collide_free
                & slow
                & ((known | near)[:, : int(1.5 / prm.dt)]).all(axis=1)
            )
            if safe2.any():
                safe, mode = safe2, "brake"
            else:
                mode = "escape" if (win.clearance(p[None])[0] < prm.r_safe) else "scan"
        cost = np.full(C, np.inf)
        idx = np.nonzero(safe)[0]
        if len(idx):
            c = objective(P[idx], times, vdes[idx])
            ang = np.arctan2(vdes[idx, 1], vdes[idx, 0]) - yaw
            ang = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) * (
                np.linalg.norm(vdes[idx, :2], axis=1) > 0.3
            )
            c = (
                c
                + prm.w_clear * np.maximum(0.0, prm.clear_pref - min_clear[idx])
                + prm.w_heading * ang
            )
            c = c + prm.w_alt * np.abs(P[idx, -1, 2] - prm.z_cruise)
            if self.prev_vdes is not None:
                c = c + prm.w_smooth * np.linalg.norm(
                    vdes[idx] - self.prev_vdes, axis=1
                )
            cost[idx] = c
            best = int(idx[np.argmin(c)])
        else:  # no safe primitive at all: the one with the most clearance that slows down (escape), or hold still (scan)
            score = min_clear - 0.3 * np.linalg.norm(vdes, axis=1) - 5.0 * (~alt_ok)
            best = int(np.argmax(score)) if mode == "escape" else 0
        self.prev_vdes = vdes[best].copy()
        plan = Plan(
            p.copy(),
            v.copy(),
            vdes[best].copy(),
            prm.a_max if mode in ("track", "brake") else prm.a_emerg,
            prm.a_brake,
            prm.horizon,
            mode,
            float(cost[best]),
            float(min_clear[best]),
        )
        plan.telemetry = self._telemetry(P, times, cost, safe, min_clear, best, vdes)
        return plan

    @staticmethod
    def _telemetry(
        P, times, cost, safe, min_clear, best, vdes, n_good=10, n_bad=6, pts=10
    ):
        """A few candidates for the HUD: the cheapest safe ones and some rejected ones, each as `pts` points of its path (m, rounded to cm)."""
        order = np.argsort(cost)
        good = [int(i) for i in order if np.isfinite(cost[i])][:n_good]
        bad_all = np.nonzero(~safe)[0]
        bad = [int(i) for i in bad_all[:: max(1, len(bad_all) // n_bad)]][:n_bad]
        sel = np.linspace(0, P.shape[1] - 1, pts).astype(int)
        row = lambda i, ok: dict(ok=ok, cost=None if not np.isfinite(cost[i]) else round(float(cost[i]), 2), min_clear=round(float(min_clear[i]), 2),
                                 v=np.round(vdes[i], 2).tolist(), path=np.round(P[i, sel], 2).tolist())  # fmt: skip
        return dict(
            n=len(P),
            n_safe=int(safe.sum()),
            chosen=int(best),
            candidates=[row(i, True) for i in good] + [row(i, False) for i in bad],
        )


# ---- objectives --------------------------------------------------------------------------------------------------------------------------
def goal_objective(goal, z_goal):
    """Distance from the stopping point of each primitive to the goal (m)."""
    g = np.array([goal[0], goal[1], z_goal])

    def cost(P, times, vdes):
        return np.linalg.norm(P[:, -1, :2] - g[:2], axis=1) + 0.0 * vdes[:, 0]

    return cost


def follow_objective(des_xy, subject_v_xy, t_c=1.5):
    """Distance, t_c seconds ahead, between the primitive and the desired spot (behind the subject), which moves with the subject's velocity; plus
    the mismatch to the subject's velocity so the drone settles instead of overshooting."""
    des = np.asarray(des_xy, float)
    vs = np.asarray(subject_v_xy, float)

    def cost(P, times, vdes):
        k = int(np.argmin(np.abs(times - t_c)))
        return np.linalg.norm(
            P[:, k, :2] - (des + vs * times[k]), axis=1
        ) + 0.4 * np.linalg.norm(vdes[:, :2] - vs, axis=1)

    return cost
