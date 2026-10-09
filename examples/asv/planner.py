"""Local trajectory planner with COLREG costs (Rules 13-17), run at 2 Hz on the ESTIMATED own state and the tracks of asv.perception.

A set of candidate manoeuvres is rolled out with a kinematic model of the boat (first-order heading with a rate limit, first-order speed): hold a course offset
`delta` from the mission's LOS course (negative = to starboard) at a speed fraction for `hold` seconds, then return to the mission course. Every candidate is scored
on the same horizon:
  goal       distance of its end point to the point 50 m ahead on the mission leg                           (progress and return to the route)
  static     clearance to the lidar points that do not belong to a moving contact: 0 above `D_STATIC`, quadratic below, and a wall at `D_COLLIDE`
  traffic    closest point of approach to every moving contact under constant-velocity prediction: quadratic below `D_SAFE` (a bit more for a Rule-13 overtake)
  Rules      contact roles come from asv.colreg.classify and are latched while the situation is open:
               give-way (head-on, crossing with the contact on our starboard side, overtaking): port turns are penalised (Rules 14, 15, 16: "early and substantial
               action", to starboard), a head-on pass starboard-to-starboard and a crossing that passes AHEAD of the other vessel are penalised (Rules 14, 15);
               stand-on (crossing with the contact on our port side, being overtaken): keeping course and speed is rewarded (Rule 17(a)); the penalty is lifted when
               the contact is not giving way (its closest point of approach stays below `D_ACT` with `T_ACT` seconds to go, Rule 17(a)(ii) / (b)); a turn to port
               for a vessel on our port side stays penalised (Rule 17(c)).
  smoothness a cost for changing the manoeuvre between plans (no chattering)
The thresholds are tuning choices, listed in the README; the Rules give none.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from . import colreg as cr

D_SAFE = 12.0  # m, wanted closest point of approach
D_SAFE_OVERTAKE = 9.0
D_STATIC = 6.0  # m, wanted clearance to fixed obstacles
D_COLLIDE = 2.0  # m, nearer than this is a collision for the planner (hull half length 0.6 m + margins)
D_RISK = 20.0  # m: a contact whose CPA at the present course is below this is a risk of collision (Rule 7)
T_RISK = 60.0  # s: ... and whose closest point is within this time
D_ACT = 7.0  # m, the contact is not keeping clear if its CPA at our present course stays below this
T_ACT = 13.0  # s, ... with this much time to go (Rule 17(a)(ii)); below T_ACT / 2 the stand-on vessel MUST act (17(b))
HORIZON, STEP = (
    75.0,
    0.5,
)  # 75 s sees a meeting 225 m away at a closing speed of 3 m/s, and most of an overtaking pass (relative speed 0.8 m/s)
DELTAS = np.radians(
    [-80, -60, -45, -32, -22, -14, -7, 0, 7, 14, 22, 32, 45, 60, 80]
).astype(float)  # + = to port
FRACS = np.array([1.0, 0.6, 0.3, 0.0])
HOLDS = np.array(
    [0.0, 8.0, 16.0, 28.0]
)  # s; 0 = follow the LOS course of the leg (no manoeuvre)
# Second family of manoeuvres: follow a line parallel to the leg at a lateral offset (m, + to port) for `hold` s, then rejoin the leg. A pass of a slow vessel
# takes longer than any heading hold, and a heading held long enough to clear it ends tens of metres off the route; a parallel line costs exactly its offset.
OFFSETS = np.array([-24.0, -18.0, -13.0, -9.0, -6.0, -3.0, 4.0, 9.0, 15.0])
HOLDS_LINE = np.array([20.0, 40.0, 75.0])
OFFSET_REACH = 30.0  # m: a line at offset o counts as a course offset of atan(o / OFFSET_REACH) in the Rule costs
R_MAX, K_PSI = (
    0.30,
    0.55,
)  # kinematic model of the boat: largest turn rate (rad/s) and heading time constant gain (1/s)
TAU_U = 3.0  # s, speed time constant
D_PAST = 25.0  # m: a contact whose closest point is behind us is finished with once it is this far
T_NEAR, T_FAR, W_FAR_MIN = (
    35.0,
    65.0,
    0.25,
)  # s, s, 1: the weight of a contact's clearance cost falls from 1 (closest point within T_NEAR) to W_FAR_MIN (T_FAR and beyond)
ACT_HOLD, ACT_KEEP = (
    1.0,
    6.0,
)  # s: the stand-on vessel acts when the trigger has held for ACT_HOLD s, and then for ACT_KEEP s
PORT_W, PORT_FREE = (
    60.0,
    3.0,
)  # cost per 30 degrees of heading to port of the leg beyond PORT_FREE degrees while a head-on / crossing give-way contact is not yet passed
XTE_W = 0.08  # cost per metre of the widest cross-track excursion of a candidate
COMMIT = 0.3  # cost margin by which another manoeuvre must beat the one in force
CLASS_NAME = {
    cr.NONE: "none",
    cr.HEAD_ON: "head_on",
    cr.OVERTAKING: "overtaking",
    cr.OVERTAKEN: "overtaken",
    cr.GIVE_WAY: "crossing_give_way",
    cr.STAND_ON: "crossing_stand_on",
}


@dataclass
class Plan:
    delta: float = 0.0  # rad, course offset from the mission course (+ port); of a parallel line: atan(offset / OFFSET_REACH)
    frac: float = 1.0  # speed fraction of the mission speed
    hold: float = 0.0
    line: bool = False  # the manoeuvre follows a line parallel to the leg at `off` m (+ port), not a course offset
    off: float = 0.0
    traj: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    contacts: list = field(
        default_factory=list
    )  # per moving contact: dict(id, cls, role, range, bearing, cpa, tcpa, risk)
    rule: str = "none"
    cost: float = 0.0
    emergency: bool = False


def los_course(alpha, a, e, x, la_min, la_k):
    """LOS course (as asv.control.Route) of points x (C, 2) on the leg through a (2, or C x 2) with direction e: the leg direction turned towards the leg by
    atan(xte / lookahead), the look-ahead growing with the cross-track error so that the approach angle stays below ~30 degrees."""
    rel = x - a
    xte = e[0] * rel[:, 1] - e[1] * rel[:, 0]
    return alpha - np.arctan2(xte, np.maximum(la_min, la_k * np.abs(xte)))


def rollout(
    pos, psi, u, leg, u_nom, delta, frac, hold, n, la_min=4.0, la_k=1.7, off=None
):
    """Positions (C, n, 2) and velocities (C, n, 2) of C candidates. Arrays delta, frac, hold of shape (C,). leg = (alpha, a, e): the mission leg's direction angle,
    start and unit vector. A candidate steers alpha + delta for `hold` seconds (hold 0: never) and the LOS course of the leg afterwards.
    The offset is relative to the leg direction, not to the LOS course (whose cross-track term would otherwise cancel a deliberate avoidance turn).
    `off` (C,) given: the candidates with a non-NaN offset steer the LOS course of the line parallel to the leg at that lateral offset (+ port) for `hold` seconds."""
    alpha, a, e = leg
    C = len(delta)
    x = np.tile(np.asarray(pos, float), (C, 1))
    ps = np.full(C, psi, float)
    uu = np.full(C, max(u, 0.0), float)
    P = np.zeros((C, n, 2))
    V = np.zeros((C, n, 2))
    line = np.zeros(C, bool) if off is None else ~np.isnan(off)
    a_line = a + np.where(line, off if off is not None else 0.0, 0.0)[
        :, None
    ] * np.array([-e[1], e[0]])
    for i in range(n):
        t = i * STEP
        cmd = np.where(
            t < hold,
            np.where(
                line, los_course(alpha, a_line, e, x, la_min, la_k), alpha + delta
            ),
            los_course(alpha, a, e, x, la_min, la_k),
        )
        r = np.clip(K_PSI * ((cmd - ps + np.pi) % (2 * np.pi) - np.pi), -R_MAX, R_MAX)
        ps = ps + r * STEP
        uu = uu + (frac * u_nom - uu) * (1 - math.exp(-STEP / TAU_U))
        v = np.stack([uu * np.cos(ps), uu * np.sin(ps)], 1)
        x = x + v * STEP
        P[:, i], V[:, i] = x, v
    return P, V


def steer(plan, rt, pos, la_min=4.0, la_k=1.7):
    """The course the autopilot steers now: the mission's LOS course `rt["psi"]` without a manoeuvre, the leg direction plus the plan's course offset, or the LOS
    course of the parallel line. rt: the Route.update() result plus leg_a, leg_dir (as passed to Planner.plan)."""
    if plan is None or plan.hold <= 0:
        return rt["psi"]
    if not plan.line:
        return rt["alpha"] + plan.delta
    e = np.asarray(rt["leg_dir"], float)
    a = np.asarray(rt["leg_a"], float) + plan.off * np.array([-e[1], e[0]])
    return float(
        los_course(rt["alpha"], a, e, np.asarray(pos, float)[None, :], la_min, la_k)[0]
    )


class Planner:
    def __init__(self):
        self.prev = (0.0, 1.0)
        self.act_since: dict[
            int, float
        ] = {}  # Rule 17(a)(ii) trigger: since when per contact
        self.act_until: dict[int, float] = {}
        self.cls: dict[int, str] = {}
        self.born: dict[int, float] = {}  # when each contact was first classified
        self.past: dict[
            int, float | None
        ] = {}  # since when the closest point has been behind us

    def assess(self, t, own_pos, own_vel, own_psi, own_speed, tr, route=None):
        """Geometry and (latched) class of one moving contact."""
        if (
            route is not None and abs(self.prev[0]) < 1e-9 and self.prev[1] >= 1.0
        ):  # no manoeuvre in force: the situation is judged on the mission course, not on a transient heading (a boat that is still settling or being pulled back to the leg)
            own_psi, own_vel = route
        g = cr.geometry(own_pos, own_vel, own_psi, tr.pos, tr.vel)
        risk = g.cpa < D_RISK and 0.0 < g.tcpa < T_RISK and g.range < 250.0
        prev = self.cls.get(tr.id, cr.NONE)
        if prev == cr.NONE:
            # classified once the contact's velocity can be trusted (a vessel just leaving its berth has none): 4 s of track and a speed above the noise
            known = t - tr.t0 >= 4.0 and tr.speed > 0.25
            cls = (
                cr.classify(g, own_speed, tr.speed, cr.NONE)
                if risk and known
                else cr.NONE
            )
        else:
            # The situation was classified when the risk first appeared and keeps that class (our own manoeuvre moves the bearing out of the head-on cone, and
            # a vessel that was ahead is not "crossing" half a minute later) until the closest point is behind us for 3 s, or the contact is far and diverging.
            cls = prev
            if (
                abs(self.prev[0]) < 1e-9
                and self.prev[1] >= 1.0
                and risk
                and t - tr.t0 >= 4.0
                and tr.speed > 0.25
                and t - self.born.get(tr.id, -1e9) < 12.0
            ):  # only in the first 12 s of the encounter: later the other vessel's own manoeuvre (a give-way turn) must not change the class
                # no manoeuvre in force, so our own course has not moved the bearing: the class is re-read from the (improving) track instead of kept from the
                # first look, when a young track's velocity can still be wrong (a lidar track's course while the ASV is still settling on its heading)
                new = cr.classify(g, own_speed, tr.speed, prev)
                # a give-way duty is not shed by the re-read: a head-on vessel that has turned to starboard at 50 m is on our port bow, i.e. "crossing, we stand on", but
                # the situation was head-on and stays so until we are past (Rule 8(d)/16: keep well clear until finally past and clear)
                if not (cr.role(prev) == "give_way" and cr.role(new) != "give_way"):
                    cls = new
            self.past[tr.id] = self.past.get(tr.id, t) if g.tcpa < 0 else None
            gone = (
                g.cpa >= 1.5 * D_RISK and g.range > 40.0 and g.tcpa < 0.0
            )  # diverging and far: an approaching contact stays in force even when our own turn moved its predicted closest point away (Rule 16: keep clear until past)
            past = self.past.get(tr.id) is not None and t - self.past[tr.id] > 3.0
            if (
                gone or (past and g.range > D_PAST)
            ):  # Rule 13(d): the overtaking vessel stays the give-way vessel until FINALLY past and clear
                cls = cr.NONE
                self.past.pop(tr.id, None)
        if prev == cr.NONE and cls != cr.NONE:
            self.born[tr.id] = t
        self.cls[tr.id] = cls
        return g, cls, risk

    def plan(self, t, own, rt, moving, pts) -> Plan:
        """own: dict(pos, vel, psi, speed); rt: asv.control.Route.update() result plus `goal` (xy); moving: Tracks; pts: (m, 2) static lidar points."""
        pos, vel, psi = (
            np.asarray(own["pos"], float),
            np.asarray(own["vel"], float),
            own["psi"],
        )
        u_nom = rt["u"]
        n = int(HORIZON / STEP)
        dd, ff, hh = np.meshgrid(DELTAS, FRACS, HOLDS, indexing="ij")
        dd, ff, hh = dd.ravel(), ff.ravel(), hh.ravel()
        dd = np.where(
            hh == 0.0, 0.0, dd
        )  # hold 0 = pure LOS: the offset is meaningless
        oo = np.full(
            len(dd), np.nan
        )  # lateral offset of a parallel-line candidate (NaN: a course-offset candidate)
        lo_, lf_, lh_ = np.meshgrid(OFFSETS, FRACS, HOLDS_LINE, indexing="ij")
        oo = np.concatenate([oo, lo_.ravel()])
        dd = np.concatenate([dd, np.arctan2(lo_.ravel(), OFFSET_REACH)])
        ff = np.concatenate([ff, lf_.ravel()])
        hh = np.concatenate([hh, lh_.ravel()])
        leg = (
            rt["alpha"],
            np.asarray(rt["leg_a"], float),
            np.asarray(rt["leg_dir"], float),
        )
        P, V = rollout(pos, psi, own["speed"], leg, u_nom, dd, ff, hh, n, off=oo)
        ts = np.arange(n) * STEP
        cost = np.zeros(len(dd))
        # goal
        # progress along the mission leg that the boat would have made at the mission speed (overshooting the goal is not penalised: the leg ends there)
        a, e = np.asarray(rt["leg_a"], float), np.asarray(rt["leg_dir"], float)
        rel = P[:, -1] - a
        along_e, xte_e = rel @ e, e[0] * rel[:, 1] - e[1] * rel[:, 0]
        goal_along = min(rt["leg_len"], rt["along"] + u_nom * HORIZON)
        cost += (
            1.2 * (np.clip(goal_along - along_e, 0, None) + 0.5 * np.abs(xte_e)) / 10.0
        )
        # the detour itself: the widest excursion from the leg along the whole horizon (a wide pass that has returned by the end is not free)
        rel_all = P - a
        cost += XTE_W * np.abs(e[0] * rel_all[:, :, 1] - e[1] * rel_all[:, :, 0]).max(
            axis=1
        )
        cost += 0.15 * (np.abs(dd) / math.radians(30)) ** 2 + 0.4 * (
            1.0 - ff
        )  # an unneeded turn or slow-down costs a little
        # static obstacles
        hazard = False  # something to commit to: a fixed object within 25 m (below), or an engaged traffic contact
        if len(pts):
            near = pts[np.hypot(*(pts - pos).T) < 45.0]
            if len(near):
                hazard = bool(np.min(np.hypot(*(near - pos).T)) < 25.0)
                if len(near) > 160:
                    near = near[:: math.ceil(len(near) / 160)]
                dmin = np.min(
                    np.hypot(
                        P[:, :, None, 0] - near[None, None, :, 0],
                        P[:, :, None, 1] - near[None, None, :, 1],
                    ),
                    axis=(1, 2),
                )
                cost += 8.0 * np.clip(
                    (D_STATIC - dmin) / D_STATIC, 0, None
                ) ** 2 + 400.0 * (dmin < D_COLLIDE)
        # traffic
        info, emergency, rules = [], False, []
        closing = hazard  # commit to a manoeuvre while a fixed object is near or a contact is still to be passed (its closest point is not yet 3 s behind)
        for tr in moving:
            g, cls, risk = self.assess(
                t,
                pos,
                vel,
                psi,
                own["speed"],
                tr,
                route=(
                    rt["alpha"],
                    np.asarray(rt["leg_dir"], float) * max(own["speed"], 0.5 * u_nom),
                ),
            )
            role = cr.role(cls)
            closing = closing or (risk and g.tcpa > 0.0)
            hazard = (
                hazard or risk
            )  # a contact with risk of collision, classified yet or not
            tgt = tr.pos[None, None, :] + tr.vel[None, None, :] * (
                ts[None, :, None] + 0.0
            )
            d = np.hypot(*(P - tgt).transpose(2, 0, 1))
            k = np.argmin(d, axis=1)
            cpa = d[np.arange(len(dd)), k]
            ds = D_SAFE_OVERTAKE if cls == cr.OVERTAKING else D_SAFE
            w_cpa = 1.0
            # Rule 17(a)(ii): the other vessel is not keeping clear -- its closest point at our present course stays low with time running out, for ACT_HOLD s on
            # end (one noisy track update of a vessel that is turning away must not make the stand-on vessel turn); once acting it goes on for ACT_KEEP s
            if (
                role == "stand_on"
                and cls != cr.NONE
                and g.cpa < D_ACT
                and 0.0 < g.tcpa < T_ACT
            ):
                since = self.act_since.setdefault(tr.id, t)
                if t - since >= ACT_HOLD:
                    self.act_until[tr.id] = t + ACT_KEEP
            else:
                self.act_since.pop(tr.id, None)
            acting = t < self.act_until.get(tr.id, -1.0)
            if role == "stand_on" and cls != cr.NONE and not acting:
                w_cpa = 0.0  # Rule 17(a)(i): the stand-on vessel keeps course and speed; the predicted closest point is the other vessel's to remove, until 17(a)(ii)
            # a closest point far in the future is predicted with a constant-velocity model of a vessel that will still act: its clearance counts less than a near one's
            w_far = np.clip((T_FAR - k * STEP) / (T_FAR - T_NEAR), W_FAR_MIN, 1.0)
            cost += w_cpa * (
                14.0 * w_far * np.clip((ds - cpa) / ds, 0, None) ** 2
                + 400.0 * (cpa < D_COLLIDE)
            )
            rel_k = (
                tgt[0, k] - P[np.arange(len(dd)), k]
            )  # target relative to own at the closest point
            v_k = V[np.arange(len(dd)), k]
            side_port = (
                v_k[:, 0] * rel_k[:, 1] - v_k[:, 1] * rel_k[:, 0]
            ) > 0  # target on our port at the CPA
            ahead = (
                np.einsum("ij,j->i", P[np.arange(len(dd)), k] - tgt[0, k], tr.vel) > 0
            )  # we are in front of the target at the CPA
            engaged = (
                cls != cr.NONE
            )  # latched: the obligation outlives the moment the risk is no longer predicted (our own manoeuvre removes it)
            act_now = False
            if role == "stand_on" and engaged:
                # Rule 17(a)(ii) / (b): the contact is not keeping clear: its CPA at our present (nominal) course stays low with time running out
                act_now = acting
                emergency = emergency or (act_now and 0.0 < g.tcpa < T_ACT / 2)
            if role == "give_way" and engaged:
                w = 1.0
                cost += (
                    w * 10.0 * np.clip(dd, 0, None) / math.radians(30)
                )  # no turn to port: a side chosen once (the committed side of Rule 16's "substantial action") is not given up for a few metres of detour
                # nor a return towards the route that heads more than 6 degrees to port of the leg while the other vessel is still within 60 m and not yet passed
                # (the LOS course of a boat that is south of the leg points to port of it; Rule 16: the give-way vessel keeps well clear until it is past)
                if (
                    g.tcpa > -6.0
                    and cls
                    in (
                        cr.HEAD_ON,
                        cr.GIVE_WAY,
                    )
                ):  # (the judge's closest point is the TRUE one: a few seconds after the estimated one, the turn back must still wait)
                    h = np.arctan2(V[:, :, 1], V[:, :, 0]) - rt["alpha"]
                    port = (h + np.pi) % (2 * np.pi) - np.pi
                    live = (np.arange(n)[None, :] <= np.maximum(k, 12)[:, None]) & (
                        d < 60.0
                    )
                    worst = np.max(np.where(live, port, 0.0), axis=1)
                    cost += (
                        w
                        * PORT_W
                        * np.clip(worst - math.radians(PORT_FREE), 0, None)
                        / math.radians(30)
                    )
                if cls == cr.OVERTAKING:
                    # an overtaking vessel passes: waiting behind the slower one (a first Monte Carlo: 21 of 30 runs idled at 0.1 m/s behind it until the time was up)
                    # keeps clear but is no passing
                    cost += w * 8.0 * (1.0 - ff)
                if cls == cr.HEAD_ON:
                    cost += w * 6.0 * (~side_port)  # starboard-to-starboard
                if cls == cr.GIVE_WAY:
                    cost += (
                        w * 6.0 * (ahead & (cpa < 2.5 * D_SAFE))
                    )  # crossing ahead of the other vessel
            elif role == "stand_on" and engaged:
                keep = (dd / math.radians(30)) ** 2 + (1.0 - ff) ** 2
                cost += (0.0 if act_now else 40.0) * keep
                if (
                    g.bearing < 0
                ):  # contact on our port side: Rule 17(c), no turn to port in any case
                    cost += 3.0 * np.clip(dd, 0, None) / math.radians(30)
            info.append(
                dict(
                    id=tr.id,
                    cls=CLASS_NAME[cls],
                    role=role,
                    range=g.range,
                    bearing=g.bearing,
                    cpa=g.cpa,
                    tcpa=g.tcpa,
                    risk=bool(engaged),
                    act=bool(act_now),
                )
            )
            if engaged:
                rules.append((g.cpa, CLASS_NAME[cls]))
                hazard = True
                closing = closing or g.tcpa > -3.0
        cost += (
            0.02 * (dd > 0) - 0.05 * ((hh == 0.0) & (ff == 1.0))
        )  # a faint starboard preference (the sea convention) and one for the null manoeuvre
        i = int(np.argmin(cost))
        self.last = (cost, dd, ff, hh, oo)  # for the tuning scripts
        # commitment: keep the manoeuvre of the last plan unless another one is clearly better (no chattering between two near-equal choices) -- only while
        # there is something to avoid: with nothing near, a held offset would be kept for a few metres of cross-track cost (the route is the goal)
        key = np.where(
            np.isnan(oo), dd, 1000.0 + oo
        )  # a manoeuvre = its course offset or its line, and its speed fraction
        same = np.nonzero(
            (np.abs(key - self.prev[0]) < 1e-9) & (np.abs(ff - self.prev[1]) < 1e-9)
        )[0]
        if len(same) and closing:
            j = int(same[np.argmin(cost[same])])
            if cost[j] <= cost[i] + COMMIT:
                i = j
        self.prev = (float(key[i]), float(ff[i]))
        rule = min(rules)[1] if rules else "none"
        line = bool(not np.isnan(oo[i]) and hh[i] > 0)
        return Plan(
            delta=float(dd[i]),
            frac=float(ff[i]),
            hold=float(hh[i]),
            line=line,
            off=float(oo[i]) if line else 0.0,
            traj=P[i],
            contacts=info,
            rule=rule,
            cost=float(cost[i]),
            emergency=emergency,
        )
