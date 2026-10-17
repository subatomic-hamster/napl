"""Mission executive and the robot's 10 Hz loop: localize -> fuse the scan -> traversability -> (1 Hz) plan -> pure pursuit.
Robot side only: it sees the Locomotion door, device attributes and its own estimates, never ground truth."""

import math
import time

import numpy as np

from . import elevmap, icp, lidar, localize, planner, traverse


class Executive:
    INIT, SWEEP, FILL, RETURN, DONE, ABORT = range(6)

    def __init__(self, cfg, home_xy):
        self.cfg, self.P, self.T = cfg, cfg["plan"], cfg["trav"]
        self.home = tuple(float(v) for v in home_xy)
        self.wps = planner.lanes(cfg, self.P["lane_m"], self.home)
        self.wi, self.phase, self.goal, self.goal_block = 0, self.INIT, None, None
        self.tried, self.best = set(), (math.inf, 0.0)
        self.unreachable = (
            False  # the last A* found no path to the goal: skip it on the next update
        )

    def _set(self, goal, t, block=None):
        self.goal, self.goal_block, self.best = goal, block, (math.inf, t)

    def update(self, t, pose, cost, mapped, fallen, unknown=None):
        P = self.P
        if fallen:
            self.phase = self.ABORT
        if self.phase in (self.DONE, self.ABORT):
            return []
        if self.phase == self.INIT:
            if t < P["init_s"]:
                return []
            self.phase = self.SWEEP
        xy = (pose[0], pose[1])
        v = self.cfg["carrier"]["v_max"]
        if (
            self.phase != self.RETURN
            and t
            >= P["t_budget"]
            - P["detour"] * math.dist(xy, self.home) / v
            - P["return_margin"]
        ):
            self.phase = self.RETURN
            self._set(self.home, t)
        skip, self.unreachable = self.unreachable, False
        if self.goal is not None:
            d = math.dist(xy, self.goal)
            if d < self.best[0] - 0.5:
                self.best = (d, t)
            elif t - self.best[1] > P["stuck_s"]:
                skip = True
        reached = self.goal is not None and math.dist(xy, self.goal) < P["goal_tol"]
        if self.phase == self.SWEEP and (self.goal is None or reached or skip):
            if self.goal is not None:
                self.wi += 1
            if self.wi < len(self.wps):
                self._set(self.wps[self.wi], t)
            else:
                self.phase = self.FILL
                self.goal = None
        me = planner.block_of(xy[0], xy[1], self.cfg)
        c = cost.copy()
        if unknown is not None and self.phase == self.SWEEP:
            # a sweep lane is a straight line: unknown ground costs the same as known, else A* walks back over the mapped strip
            c[unknown & np.isfinite(c)] = 1.0
        start = me
        if not math.isfinite(c[me]):
            # standing in a lethal block (a wall face seen from 0.3 m): plan from the nearest free block, never through the wall
            # (patching the start block finite lets A* tunnel into the unseen far half of the wall: ruins mission, bumps 12552)
            nb = c.shape[0]
            near = [
                (math.dist(xy, planner.centre((i, j), self.cfg)), (i, j))
                for i in range(max(me[0] - 2, 0), min(me[0] + 3, nb))
                for j in range(max(me[1] - 2, 0), min(me[1] + 3, nb))
                if math.isfinite(c[i, j])
            ]
            if near:
                start = min(near)[1]
            else:
                c[me] = self.T["unknown_cost"] + self.T["near_lethal"]
        if self.phase == self.FILL and (self.goal is None or reached or skip):
            if self.goal_block is not None:
                self.tried.add(self.goal_block)
            dist, parent = planner.dijkstra(c, start)
            ij = planner.choose(
                dist,
                planner.gain(mapped, self.cfg, int(P["pool"])),
                self.tried,
                self.cfg,
            )
            if ij is None:
                self.phase = self.RETURN
                self._set(self.home, t)
            else:
                vb = planner.view_goal(
                    planner.path_to(parent, start, ij, c.shape[1]), self.cfg
                )
                self._set(planner.centre(vb, self.cfg), t, ij)
        if self.phase == self.RETURN and math.dist(xy, self.home) < P["goal_tol"]:
            self.phase = self.DONE
            return []
        path = planner.astar(c, start, planner.block_of(*self.goal, self.cfg))
        if path is None:
            self.unreachable = True  # skipped on the next update (plan bug fixed: best=(inf, -inf) was reset by the progress test)
            return []
        return [planner.centre(b, self.cfg) for b in path[:-1]] + [self.goal]


class Robot:
    def __init__(self, loco, cfg, rays, ext, x0, lever, seed):
        self.loco, self.cfg, self.ext = loco, cfg, ext
        L, I, P = cfg["lidar"], cfg["icp"], cfg["plan"]
        self.ros = lidar.Rosette(loco.sim, loco.lidar, rays, loco.dt, cfg)
        self.gnss = localize.GnssReader(loco.sim, loco.gnss, cfg, loco.dt)
        self.loc = localize.Localizer(cfg, x0, lever)
        self.sm = icp.ScanMatcher(cfg)
        self.map = elevmap.ElevationMap(cfg)
        self.trav = traverse.Traversability(cfg, self.map.n)
        self.ex = Executive(cfg, x0[:2])
        self.rng = np.random.default_rng([seed, 0x11DA])
        self.icp_every = {
            True: int(round(L["rate_hz"] / I["rate_good"])),
            False: int(round(L["rate_hz"] / I["rate_bad"])),
        }
        self.plan_every = int(round(L["rate_hz"] / P["rate_hz"]))
        self.k, self.path = 0, []
        self.ms = dict.fromkeys(("points", "icp", "fuse", "trav", "plan"), 0.0)

    def _tick(self, key, t0):
        t1 = time.perf_counter()
        self.ms[key] += 1e3 * (t1 - t0)
        return t1

    def frame(self, t):
        cfg, t0 = self.cfg, time.perf_counter()
        od = self.loco.odometry()
        self.loc.predict(od)
        fix = self.gnss.poll()
        if fix is not None:
            self.loc.update_rtk(*fix)
        scan = self.ros.latest()
        q_rp = lidar.qmul(localize._qz(-lidar.yaw_of(od.quat)), od.quat)
        pts = icp.levelled(scan, self.ext, od.quat, cfg["lidar"]["ang_sigma"], self.rng)
        t0 = self._tick("points", t0)
        if self.k and self.k % self.icp_every[self.loc.rtk_ok(od.t)] == 0:
            m = self.sm.match(pts, self.loc.x.copy())
            if m is not None:
                self.loc.update_icp(m.x, m.cov)
        R = localize.rz(self.loc.x[3])
        world = pts @ R.T + self.loc.x[:3]
        self.sm.add(world, self.loc.x)
        t0 = self._tick("icp", t0)
        origin = R @ (lidar.qmat(q_rp) @ self.ext[0]) + self.loc.x[:3]
        self.map.fuse(world, origin, self.loc.var_z, cfg["odom"]["att_sigma"])
        t0 = self._tick("fuse", t0)
        if self.map.dirty is not None:
            self.trav.update(self.map.hn, self.map.dirty)
        t0 = self._tick("trav", t0)
        pose = (self.loc.x[0], self.loc.x[1], self.loc.x[3])
        if self.k % self.plan_every == 0:
            self.path = self.ex.update(
                t,
                pose,
                self.trav.cost,
                self.map.mapped(int(cfg["plan"]["pool"])),
                od.fallen,
                self.trav.L["known"] < cfg["trav"]["min_known"],
            )
        lim = self.loco.limits()
        vx, wz = planner.pursue(pose, self.path, cfg, lim) if self.path else (0.0, 0.0)
        self.loco.command(vx, 0.0, wz)
        P = self.loc.P  # SP5 -> SP4 pose fix, 10 Hz, with the localizer's own 1-sigma (SP4 Controller.fix sig_p / sig_yaw)
        self.loco.fix(
            self.loc.x[:2].copy(),
            float(self.loc.x[3]),
            od.t,
            float(math.sqrt(max(P[0, 0], P[1, 1]))),
            float(math.sqrt(P[3, 3])),
        )
        self._tick("plan", t0)
        self.k += 1
