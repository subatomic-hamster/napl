"""A fast stand-in for the kernel in the ASV Monte Carlo: the planner, tracker, AIS channel, route, autopilot and judge are the real ones (asv.planner, asv.perception,
asv.ais, asv.control, asv.judge); the boats are a 3-DOF Fossen model (surge, sway, yaw) with the T200 / hull / skeg numbers of the parts, and the sensors are
the measured statistics of the real ones (GNSS bias, lidar centroid, 30 m lidar range). One run takes seconds instead of two minutes, so the planner can be
tuned on hundreds of geometries; the kernel run (`run.py`, `mc.py`) stays the judge of what is claimed.

Differences from the kernel: no waves except a heading disturbance and a speed loss, no roll / pitch, the lidar sees a vessel within 28 m without occlusion, the
estimate is the truth plus a slowly varying GNSS bias and small noise (not the filter), static scenery is a set of points on the box / cylinder surfaces.

    python examples/asv/fastsim.py <kind> [seed] [hs]   one run, prints the result dict
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from asv import ais as ais_mod
from asv import encounters as enc
from asv import judge
from asv.control import T_FULL, Autopilot, Gains, Route
from asv.geo import LAT0, LON0, latlon_from_enu, wrap
from asv.perception import Tracker
from asv.planner import Planner, steer
from asv.traffic import TrafficVessel

DT = 0.04  # control period
IMU_RHO = 998.2
LIDAR_RANGE = 28.0


class Boat:
    """3-DOF (surge u, sway v, yaw rate r) rigid body with added mass, linear + quadratic damping and the two skegs."""

    M, A11, A22, IZ, A66 = 17.0, 1.7, 11.0, 4.5, 1.6
    ARM = 0.38

    def __init__(self, x, y, psi):
        self.p = np.array([x, y], float)
        self.psi, self.u, self.v, self.r = psi, 0.0, 0.0, 0.0

    @staticmethod
    def thrust(thr):
        return T_FULL * thr * abs(thr) * (1.0 if thr >= 0 else 0.787)

    def step(self, thr_port, thr_stbd, dt, n_torque=0.0, loss=1.0):
        tp, ts = self.thrust(thr_port), self.thrust(thr_stbd)
        u, v, r = self.u, self.v, self.r
        fin_w = v - 0.45 * r
        fy = (
            -2
            * 0.5
            * IMU_RHO
            * 0.03
            * (1.02 * abs(u) * fin_w + 1.1 * abs(fin_w) * fin_w)
        )
        x = (
            (tp + ts)
            - loss * (3.0 * u + 1.95 * abs(u) * u)
            + (self.M + self.A22) * v * r
        )
        y = -(15.0 * v + 49.4 * abs(v) * v) + fy - (self.M + self.A11) * u * r
        n = (
            self.ARM * (ts - tp)
            + (-0.45) * fy
            - (2.0 * r + 2.7 * abs(r) * r)
            + (self.A11 - self.A22) * u * v
            + n_torque
        )
        self.u += dt * x / (self.M + self.A11)
        self.v += dt * y / (self.M + self.A22)
        self.r += dt * n / (self.IZ + self.A66)
        self.psi = wrap(self.psi + dt * self.r)
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.p += dt * np.array([self.u * c - self.v * s, self.u * s + self.v * c])

    @property
    def vel(self):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return np.array([self.u * c - self.v * s, self.u * s + self.v * c])

    @property
    def quat(self):
        return (math.cos(0.5 * self.psi), 0.0, 0.0, math.sin(0.5 * self.psi))


class Ou:
    """Slowly varying 2-D bias (Ornstein-Uhlenbeck), the GNSS error."""

    def __init__(self, rng, sigma=2.6, tau=600.0):
        self.rng, self.s, self.tau = rng, sigma, tau
        self.x = rng.normal(0, sigma, 2)

    def step(self, dt):
        a = math.exp(-dt / self.tau)
        self.x = a * self.x + math.sqrt(1 - a * a) * self.s * self.rng.normal(0, 1, 2)
        return self.x


def static_points(spec, step=0.6):
    """World points on the surface of the scenery (boxes: top outline, cylinders: circle), the lidar's view of fixed objects at the plane of the scan."""
    pts = []
    for _, kind, c, size, _ in enc.SCENERY:
        cx, cy, hx, hy = c[0], c[1], size[0], size[1]
        for t in np.arange(-hx, hx + 1e-9, step):
            pts += [(cx + t, cy - hy), (cx + t, cy + hy)]
        for t in np.arange(-hy, hy + 1e-9, step):
            pts += [(cx - hx, cy + t), (cx + hx, cy + t)]
    for _, (x, y), _ in enc.BUOYS:
        pts += [
            (x + enc.BUOY_R * math.cos(a), y + enc.BUOY_R * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)
        ]
    for o in spec.obstacles:
        pts += [
            (o.pos[0] + 0.4 * math.cos(a), o.pos[1] + 0.4 * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, 10, endpoint=False)
        ]
    return np.array(pts)


def fly_fast(kind, p, gains=None, spec=None):
    """One run of encounter `kind` with parameters p (seed, hs, jitter, ais, traffic_complies) -> result dict (same keys as run.fly's `res`)."""
    spec = spec or enc.make(
        kind,
        p["seed"],
        p["hs"],
        p["jitter"],
        bool(p["ais"]),
        bool(p["traffic_complies"]),
    )
    rng = np.random.default_rng([int(p["seed"]), 7777])
    gains = gains or Gains()
    asv = Boat(0.0, 0.0, 0.0)
    route = Route(spec.route, spec.speed, gains=gains)
    ap = Autopilot(gains)
    tracker, planner = Tracker(), Planner()
    bots = {t.name: TrafficVessel(t, DT) for t in spec.traffic}
    boats = {t.name: Boat(t.pos[0], t.pos[1], t.psi) for t in spec.traffic}
    xp = {
        t.name: ais_mod.Transponder(1000 + i, LAT0, LON0, t_first=3.0)
        for i, t in enumerate(spec.traffic)
        if t.ais
    }
    chan = ais_mod.Channel(p["seed"], LAT0, LON0)
    bias_a = Ou(rng)
    bias_t = {t.name: Ou(rng) for t in spec.traffic}
    pts_all = static_points(spec)
    loss = 1.0 + 0.25 * p["hs"] / 0.8  # added resistance in waves
    sig_n = 0.9 * p["hs"] / 0.8  # N m, wave yaw disturbance
    n_dist = 0.0
    names = [t.name for t in spec.traffic]
    hist = dict(
        t=[],
        own=[],
        psi=[],
        speed=[],
        tgt=[[] for _ in names],
        tvel=[[] for _ in names],
    )
    rows_xte = []
    cpa_min, collision, clear_min, done_t = math.inf, 0, math.inf, None
    frac, plan = 1.0, None
    thr = (0.0, 0.0)
    static_pts, last_lidar, last_plan = np.zeros((0, 2)), -1.0, -1.0
    n_ctrl = int(spec.t_end / DT)
    last_pos = asv.p.copy()
    est_v = np.zeros(2)
    for kc in range(n_ctrl):
        t = kc * DT
        bias = bias_a.step(DT)
        noise = rng.normal(0, 0.25, 2)
        epos = asv.p + bias + noise
        est_v = (
            0.9 * est_v + 0.1 * (asv.vel + rng.normal(0, 0.08, 2))
            if t > 0
            else asv.vel.copy()
        )
        epsi = asv.psi + rng.normal(0, 0.012)
        c, s = math.cos(epsi), math.sin(epsi)
        est = dict(
            pos=epos,
            vel=est_v.copy(),
            psi=epsi,
            r=asv.r + rng.normal(0, 0.01),
            speed=float(np.hypot(*est_v)),
            u=c * est_v[0] + s * est_v[1],
            v=-s * est_v[0] + c * est_v[1],
        )
        # AIS: each traffic vessel's own GNSS (with its own bias) feeds the transponder
        for i, name in enumerate(names):
            if name in xp:
                b = boats[name]
                g = b.p + bias_t[name].step(DT) + rng.normal(0, 0.25, 2)
                if kc % 5 == 0:
                    lat, lon = latlon_from_enu(*g)
                    v = b.vel + rng.normal(0, 0.05, 2)
                    xp[name].fix(t, lat, lon, [v[1], v[0], 0.0])
                chan.send(xp[name].poll(t))
        msgs = chan.receive(t)
        if msgs:
            tracker.update_ais(t, msgs)
        if (
            t - last_lidar >= 0.1 - 1e-9
        ):  # 10 Hz scan: the vessels inside the range are clusters in OUR (biased) frame
            last_lidar = t
            cl = []
            for name in names:
                rel = boats[name].p - asv.p
                d = float(np.hypot(*rel))
                if d < LIDAR_RANGE:
                    rot = wrap(epsi - asv.psi)
                    rc, rs = math.cos(rot), math.sin(rot)
                    pc = epos + np.array(
                        [rc * rel[0] - rs * rel[1], rs * rel[0] + rc * rel[1]]
                    )
                    u_ = (pc - epos) / max(d, 1e-6)
                    cen = pc - 0.2 * u_ + rng.normal(0, 0.1, 2)
                    cl.append(
                        dict(
                            c=cen, ext=0.65, n=6, rng=d - 0.4, brg=0.0, pts=cen[None, :]
                        )
                    )
            tracker.update_lidar(t, cl)
            near = np.hypot(*(pts_all - asv.p).T) < LIDAR_RANGE
            static_pts = (
                pts_all[near] + (epos - asv.p) if near.any() else np.zeros((0, 2))
            )
        moving = tracker.moving(t)
        active = t >= enc.T_START and not route.done
        rt = (
            route.update(est["pos"])
            if t > 0.3
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
        if active and t - last_plan >= 12 * 20 * 0.002 - 1e-9:
            last_plan = t
            wp, a = route.wp[route.k + 1], route.wp[route.k]
            L = float(np.hypot(*(wp - a)))
            plan = planner.plan(
                t,
                dict(
                    pos=est["pos"], vel=est["vel"], psi=est["psi"], speed=est["speed"]
                ),
                dict(rt, leg_a=a, leg_dir=(wp - a) / max(L, 1e-9), leg_len=L),
                moving,
                static_pts,
            )
            frac = plan.frac
        if t >= 0.3:
            if active:
                a_, b_ = route.wp[route.k], route.wp[route.k + 1]
                rtl = dict(
                    rt,
                    leg_a=a_,
                    leg_dir=(b_ - a_) / max(float(np.hypot(*(b_ - a_))), 1e-9),
                )
                psi_cmd, u_cmd = steer(plan, rtl, est["pos"]), rt["u"] * frac
            else:
                psi_cmd, u_cmd = (rt["psi"] if not route.done else est["psi"]), 0.0
            thr = ap.step(psi_cmd, u_cmd, est["psi"], est["r"], est["u"], DT)
        # truth step
        n_dist = 0.97 * n_dist + math.sqrt(1 - 0.97**2) * sig_n * rng.normal()
        asv.step(thr[0], thr[1], DT, n_dist, loss)
        for name in names:
            th = (0.0, 0.0)
            if t >= enc.T_START:
                th = bots[name].step(boats[name].p, boats[name].quat, asv.p)
            boats[name].step(th[0], th[1], DT, 0.0, loss)
        speed_t = float(np.hypot(*(asv.p - last_pos)) / DT)
        last_pos = asv.p.copy()
        tpos = [boats[n].p.copy() for n in names]
        if tpos:
            r1 = min(float(np.hypot(*(q - asv.p))) for q in tpos)
            cpa_min = min(cpa_min, r1)
            if r1 < judge.COLLISION_VESSEL:
                collision = 1
        clr = judge.static_clearance(
            asv.p, enc.SCENERY, enc.BUOYS, spec.obstacles, enc.BUOY_R
        )
        clear_min = min(clear_min, clr)
        if clr < judge.COLLISION_STATIC:
            collision = 1
        if t >= enc.T_START:
            hist["t"].append(t)
            hist["own"].append(asv.p.copy())
            hist["psi"].append(asv.psi)
            hist["speed"].append(speed_t)
            for i_, n_ in enumerate(names):
                hist["tgt"][i_].append(tpos[i_])
                hist["tvel"][i_].append(boats[n_].vel.copy())
            a0, b0 = (
                route.wp[min(route.k, len(route.wp) - 2)],
                route.wp[min(route.k, len(route.wp) - 2) + 1],
            )
            leg = (b0 - a0) / max(float(np.hypot(*(b0 - a0))), 1e-9)
            rows_xte.append(
                float(-leg[1] * (asv.p[0] - a0[0]) + leg[0] * (asv.p[1] - a0[1]))
            )
        if route.done and done_t is None:
            done_t = t
        if done_t is not None and t > done_t + 3.0:
            break
    own = np.array(hist["own"])
    end = np.array(spec.route[-1])
    res = dict(
        kind=kind,
        seed=p["seed"],
        hs=p["hs"],
        t_end=t,
        collision=bool(collision),
        min_static_clear_m=min(clear_min, 999.0),
        cpa_min_m=min(cpa_min, 999.0),
    )
    res.update(
        xte_rms_m=float(np.sqrt(np.mean(np.square(rows_xte)))) if rows_xte else 0.0,
        xte_max_m=float(np.max(np.abs(rows_xte))) if rows_xte else 0.0,
    )
    res["mission_complete"] = (
        bool(route.done and np.hypot(*(own[-1] - end)) < 8.0) if len(own) else False
    )
    res["end_dist_m"] = float(np.hypot(*(own[-1] - end))) if len(own) else 999.0
    if spec.traffic:
        v = judge.verdict_all([t_.kind or kind for t_ in spec.traffic], np.array(hist["t"]), own, np.array(hist["psi"]), np.array(hist["speed"]),
                              [np.array(q) for q in hist["tgt"]], [np.array(q) for q in hist["tvel"]],
                              math.atan2(*(np.array(spec.route[1]) - np.array(spec.route[0]))[::-1]), spec.speed, complied=bool(p["traffic_complies"]))  # fmt: skip
        v["collision"] = bool(v["collision"] or collision)
        res.update(v)
        res["cpa_m"] = v["cpa_m"]
    else:
        res.update(
            colreg_ok=not collision,
            side_ok=True,
            cpa_ok=True,
            cpa_m=min(clear_min, 999.0),
        )
    res["ok"] = bool(
        res["mission_complete"] and res["colreg_ok"] and not res["collision"]
    )
    return res


if __name__ == "__main__":
    kind = sys.argv[1]
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
    hs = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    r = fly_fast(
        kind, dict(seed=seed, hs=hs, jitter=1.0, ais=True, traffic_complies=True)
    )
    sys.stdout.write(
        str({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})
        + "\n"
    )
