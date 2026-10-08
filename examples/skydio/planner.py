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
    w_commit: float = 3.0  # cost of switching the dodge side (see plan); m-equivalent like the objectives
    commit_t: float = 1.0  # s: the point of a primitive whose lateral offset from the goal line defines its side
    commit_lat: float = 0.7  # m: offsets below this are "straight on", they neither set nor break a commitment
    commit_steps: int = (
        25  # replans (1 s at 25 Hz) a commitment lasts without being re-confirmed
    )
    v_lo: float = 5.0  # m/s: fastest primitive in the most cluttered surroundings (openness <= free_lo)
    free_lo: float = 0.7  # m: openness (mean saturated clearance ahead, Planner.openness) at / below which the speed limit is v_lo
    free_hi: float = 1.3  # m: openness at / above which the full v_cruise is allowed
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
        self.side, self.side_step, self.step = (
            0.0,
            0,
            0,
        )  # committed dodge side (+1 left / -1 right of the bearing), step it was last confirmed, step counter
        self.free = 1.6

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

    def openness(self, p, v, bearing, win: Window):
        """Mean clearance (m, saturated at 1.6) of the map at 28 points in a +-60 deg sector, 1.5-6 m ahead, about the direction of travel (the bearing
        when slow): a clutter measure that falls as trunks crowd the way. Also kept for the telemetry."""
        sp = float(np.linalg.norm(v[:2]))
        h = float(np.arctan2(v[1], v[0])) if sp > 1.0 else bearing
        az = h + np.radians([-60, -40, -20, 0, 20, 40, 60])
        r = np.array([1.5, 3.0, 4.5, 6.0])
        xy = p[:2] + r[:, None, None] * np.stack([np.cos(az), np.sin(az)], -1)[None]
        pts = np.concatenate([xy.reshape(-1, 2), np.full((28, 1), p[2])], 1)
        self.free = float(np.minimum(win.clearance(pts), 1.6).mean())
        return self.free

    def plan(self, p, v, yaw, win: Window, bearing, objective, extra=(), moving=None):
        """-> Plan. `objective(P, times, vdes)` -> cost (C,) over the safe candidates' paths (lower is better). `extra`: more candidate velocities.
        `moving`: [(position xy, velocity xy, radius m)] of moving obstacles (the followed person): every sample of a primitive, braking tail
        included, must stay `radius` from its constant-velocity prediction at the same time (a static sphere at its current position would force the
        follower to hang back by the horizon's travel distance)."""
        prm = self.prm
        self.step += 1
        vdes = self.candidates(p, v, yaw, bearing, extra)
        free = self.openness(p, v, bearing, win)
        v_cap = (
            prm.v_lo
            + (prm.v_cruise - prm.v_lo)
            * float(
                np.clip((free - prm.free_lo) / (prm.free_hi - prm.free_lo), 0.0, 1.0)
            )
        )  # clutter-scaled speed limit: the less open space ahead, the lower the fastest primitive
        vdes = vdes[np.linalg.norm(vdes[:, :2], axis=1) <= max(v_cap, 1.0) + 1e-6]
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
        for pm, vm, rm in moving or ():
            gap = np.linalg.norm(P[:, :, :2] - (pm + vm * times[:, None]), axis=2)
            collide_free &= (gap >= rm).all(axis=1)
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
            # decision hysteresis: a dodge to one side of the line to the goal is a homotopy class; flipping to the other side between replans (as a
            # late-resolved trunk shifts the costs by a hair) swings the vehicle across the gap at speed. Switching costs w_commit, scaled with speed.
            kc = min(int(round(prm.commit_t / prm.dt)), K - 1)
            u = np.array([np.cos(bearing), np.sin(bearing)])
            lat = u[0] * (P[:, kc, 1] - p[1]) - u[1] * (
                P[:, kc, 0] - p[0]
            )  # signed lateral offset of each candidate's point at commit_t
            if self.side != 0.0 and self.step - self.side_step <= prm.commit_steps:
                flip = lat[idx] * self.side < -prm.commit_lat
                c = c + prm.w_commit * flip * np.minimum(
                    np.linalg.norm(vdes[idx, :2], axis=1) / 2.5, 1.5
                )
            cost[idx] = c
            best = int(idx[np.argmin(c)])
            if abs(lat[best]) > prm.commit_lat:
                self.side, self.side_step = float(np.sign(lat[best])), self.step
        else:  # no safe primitive at all: the one with the most clearance that slows down (escape), or hold still (scan)
            # escape: clearance 0.8 s in (must grow) + the worst clearance from 0.2 s on (must not shrink on the way). The plain path minimum is the
            # start point inside the margin, identical for every candidate, so it left the vehicle hovering next to the obstacle for 20 s.
            k8 = min(7, K - 1)
            score = (
                clear[:, k8]
                + clear[:, 2 : k8 + 2].min(axis=1)
                - 0.2 * np.linalg.norm(vdes, axis=1)
                - 5.0 * (~alt_ok)
            )
            best = int(np.argmax(score)) if mode == "escape" else 0
            if mode == "scan" and np.linalg.norm(v[:2]) > 1.5:
                # nothing is safe and the vehicle is FAST: holding still means braking in a straight line, wherever that leads. Take the primitive
                # (dodges included) whose path keeps the most clearance instead.
                best = int(np.argmax(min_clear - 5.0 * (~alt_ok)))
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
class GoalField:
    """Cost-to-go to the goal over the map window, so that a wall or a dead end does not trap a greedy planner (D*-style, optimistic).

    A 0.4 m grid over the window's horizontal extent. A cell is blocked if its column holds an occupied voxel inside the flight band; cells within
    `r_infl` of a blocked cell cost 8x and blocked ones 40x per metre (so the field prefers gaps wide enough to fly, but is defined everywhere). Unknown
    space counts as free. The window's border cells are seeded with their straight-line distance to the goal, the rest follows by relaxation, so a goal
    outside the window (the usual case) still pulls the field. Where the camera shows a wall, the field routes around its end or through its door."""

    def __init__(self, win: Window, goal, z_lo=0.9, z_hi=3.4, r_infl=0.62, res=0.4):
        k = max(1, int(round(res / win.res)))
        self.res, self.goal = k * win.res, np.asarray(goal, float)[:2]
        n = win.occ.shape[0] // k
        self.origin = win.origin[:2]
        z0 = int((z_lo - win.origin[2]) / win.res)
        z1 = int((z_hi - win.origin[2]) / win.res)
        col = (
            win.occ[: n * k, : n * k, z0:z1]
            .any(axis=2)
            .reshape(n, k, n, k)
            .any(axis=(1, 3))
        )
        r = int(np.ceil(r_infl / self.res))
        infl = col.copy()
        pad = np.pad(col, r)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    infl |= pad[r + dx : r + dx + n, r + dy : r + dy + n]
        m = 1.0 + 7.0 * infl + 32.0 * col  # per-metre cost multiplier of each cell
        ij = np.stack(np.meshgrid(np.arange(n), np.arange(n), indexing="ij"), -1)
        xy = self.origin + (ij + 0.5) * self.res
        seed = np.linalg.norm(xy - self.goal, axis=-1)
        V = np.full((n, n), np.inf)
        edge = np.zeros((n, n), bool)
        edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
        V[edge] = (seed + (m - 1.0) * self.res)[edge]
        g = np.floor((self.goal - self.origin) / self.res).astype(int)
        if (
            (g >= 0) & (g < n)
        ).all():  # the goal itself is in the window: it is the only true source
            V[tuple(g)] = 0.0
        for _ in range(
            4 * n
        ):  # Bellman-Ford sweeps over the 8 neighbours until nothing improves
            P = np.pad(V, 1, constant_values=np.inf)
            best = V
            for dx, dy, w in (
                (-1, 0, 1),
                (1, 0, 1),
                (0, -1, 1),
                (0, 1, 1),
                (-1, -1, 1.414),
                (-1, 1, 1.414),
                (1, -1, 1.414),
                (1, 1, 1.414),
            ):
                best = np.minimum(
                    best, P[1 + dx : 1 + dx + n, 1 + dy : 1 + dy + n] + w * self.res * m
                )
            if np.array_equal(best, V):
                break
            V = best
        self.V, self.n = V, n

    def cost(self, xy):
        """(n, 2) world points -> cost-to-go (m-equivalent); straight-line distance outside the window."""
        i = np.floor((xy - self.origin) / self.res).astype(int)
        inside = ((i >= 0) & (i < self.n)).all(axis=1)
        ic = np.clip(i, 0, self.n - 1)
        out = np.linalg.norm(xy - self.goal, axis=1)
        v = self.V[ic[:, 0], ic[:, 1]]
        return np.where(inside & np.isfinite(v), v, out)

    def bearing(self, p, radius=2.5):
        """Direction (rad) of steepest descent of the field around p: the way to head to get to the goal through the known map."""
        a = np.radians(np.arange(0, 360, 15))
        pts = p[:2] + radius * np.stack([np.cos(a), np.sin(a)], 1)
        return float(a[np.argmin(self.cost(pts))])


def goal_objective(goal, z_goal, field: GoalField = None):
    """Cost-to-go of the stopping point of each primitive: the map-aware field if given, else the straight-line distance to the goal (m)."""
    g = np.array([goal[0], goal[1], z_goal])

    def cost(P, times, vdes):
        if field is not None:
            return field.cost(P[:, -1, :2])
        return np.linalg.norm(P[:, -1, :2] - g[:2], axis=1)

    return cost


def follow_objective(
    des_xy, subject_v_xy, t_c=1.5, field: GoalField = None, extra=None
):
    """Distance, t_c seconds ahead, between the primitive and the desired spot (behind the subject), which moves with the subject's velocity; plus
    the mismatch to the subject's velocity so the drone settles instead of overshooting. With a `field` (GoalField to des_xy) the distance is the
    map-aware cost-to-go, so a tree line between drone and subject is routed around instead of nosed into."""
    des = np.asarray(des_xy, float)
    vs = np.asarray(subject_v_xy, float)

    def cost(P, times, vdes):
        k = int(np.argmin(np.abs(times - t_c)))
        shift = (
            vs * times[k]
        )  # the desired spot has moved by then: look the primitive's point up relative to it
        d = (
            field.cost(P[:, k, :2] - shift)
            if field is not None
            else np.linalg.norm(P[:, k, :2] - (des + shift), axis=1)
        )
        c = d + 0.4 * np.linalg.norm(vdes[:, :2] - vs, axis=1)
        return c if extra is None else c + extra(P, times, vdes)

    return cost
