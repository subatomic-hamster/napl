"""Deck landings of the Skydio landing program: the 12.6 m support vessel under way in a seeded sea (napl.cuas.ship.ship_scene, assemblies/osv.toml) or the 1.2 m ASV in calm
water (examples/asv/scene.py add_landing_pad). `SkydioDeck` is `SkydioLand` with the ground replaced by a moving pad and the navigation by the ship stage's aids (napl.cuas.relnav):

    RelGnss      moving-base RTK: the rover's position relative to the vessel's base antenna (8 Hz, 0.125 s latency, cm in FIX, decimetres in FLOAT, a metre in SINGLE)
    DeckLink     the vessel's motion unit over the deck radio: attitude, rates, heave and position of the hull (25 Hz, 0.08 s, bursts of loss)
    DeckPredictor the pad's height and the deck's roll / pitch a few seconds ahead (autoregressive fit of the last 60 s): quiescent windows
    lidar        the downward TF-Luna reads the height over the pad surface (the pad is a ray-visible geom)

Fusion: base antenna (DeckLink pose at the fix epoch) + RelGnss relative position -> the lander's antenna in the world -> an ESKF position update (land.update_position), so the
estimate stays within centimetres of the true position of the sea-fixed frame while the deck moves. The pad's pose comes from the DeckLink message extrapolated with its rates
(relnav.deck_point_at); the lander tracks the pad point with the pad's velocity as feed-forward.

PHASES. takeoff (from the pad, the vessel at rest) -> map -> follow (hover `hover_h` m over the pad; the vessel gets under way at `t_go`) -> wait (1.5 m over the pad, until
the predictor says a quiescent window will arrive when the final sink would end, or `wait_max_s` has passed) -> sink (constant `v_sink` relative to the pad, with the pad's
vertical velocity as feed-forward) -> touchdown (lidar says on the deck: motors off; 3 s on the deck) -> landed.

EVALUATION (truth): relative vertical speed at the first gear contact, tilt against the deck's normal, offset from the pad centre in the deck frame (must be inside the pad less a
0.25 m margin), and the lander still on the pad 3 s later. The autonomy reads only relnav's outputs, the lidar and its own estimator."""

import math
import sys
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))  # examples/: ship.pilot
import land as L
from land_fly import GEAR_H, SkydioLand
from napl.cuas import relnav as RN
from napl.cuas.flyer import WN_Z

ANT_ROOT = np.array(
    [-0.062, 0.0, 0.074]
)  # m: the RTK board's antenna in the lander's root frame (assemblies/skydio-land.toml rtk_kit; position estimated)
T_PRESS = 4.0  # s the lander is pressed onto the deck after the first contact, motors running at their floor
T_STAY = 4.0  # s on the deck after the first contact (motors off for the last T_STAY - T_PRESS)
V_SINK = 0.35  # m/s relative to the pad
H_WAIT = 1.5  # m: gear height over the pad while waiting for the window
HOVER_H = 4.0  # m: gear height over the pad while following
TAU_LEVEL = (
    3.0  # s: time constant of the pad level the lander rides (about a wave period)
)
T_BLEND = 1.5  # s: the sink's height reference moves from the pad's level to its forecast height over this time


def sink_start(pred, ts, t_sink=H_WAIT / V_SINK):
    """When to begin the final sink so that the contact (t_sink later) falls at the start of the next quiescent window (pad vertical speed < 0.2 m/s, roll and pitch < 3 deg
    for 1.5 s, predicted by `pred` up to 14 s ahead). -> a time (<= ts: now) or None (no window)."""
    w = pred.next_window(
        horizon=14.0,
        duration=1.5,
        vz_max=0.2,
        tilt_max=math.radians(3.0),
        t_now=ts + t_sink,
    )
    return None if w is None else w - t_sink + 0.2


class _NoStep:
    """A sim proxy whose step() does nothing: ShipPilot.advance() then runs its estimator + control law on the state the lander's own loop has reached."""

    def __init__(self, sim):
        self._s = sim

    def step(self, n):
        pass

    def __getattr__(self, k):
        return getattr(self._s, k)


class SkydioDeck(SkydioLand):
    def __init__(self, model, run_dir, seed, cal_dir=None, params=None, lp=None):
        super().__init__(model, run_dir, seed, cal_dir, params, lp)
        d = self.land["deck"]
        self.hull, self.prefix = d["hull"], d["prefix"]
        self.pad_off, self.base_off = (
            np.array(d["pad_off"], float),
            np.array(d["base_off"], float),
        )
        self.half, self.margin = (
            d["half"],
            d["margin"],
        )  # margin: the gear must touch down this far inside the pad edge (m)
        self.t_go, self.ship_speed, self.ship_psi = (
            d["t_go"],
            d["ship_speed_ms"],
            math.radians(d["ship_heading_deg"]),
        )
        self.t_wait, self.wait_max = d["t_wait_s"], d["wait_max_s"]
        self.relgnss, self.link = (
            RN.RelGnss(seed=seed + 31),
            RN.DeckLink(seed=seed + 32),
        )
        self.pred = RN.DeckPredictor()
        self.msgs = deque(maxlen=80)
        self.tdeck, self.fix, self.fix_t = None, None, None
        self.pilot, self._level, self._hp = None, None, None
        osv = d["kind"] == "osv"
        self.LIM = (
            1.0,
            12.0,
            1.25 if osv else 0.3,
        )  # the monitors of deck/osv and deck/asv: the gear's limits as on the ground (1.0 m/s, 12 deg), the pad's size
        if d["ship_speed_ms"] > 0 or d["kind"] == "osv":
            from ship.pilot import ShipPilot

            self.pilot = ShipPilot(self.sim, prefix=self.prefix)
        self._pt, self._went = self.sim.tick, False
        self.t_sink0 = self.h_sink0 = self.t_wait0 = None
        self.forced = False
        self.sigma_rtk = []
        self.window_log = []

    # ---- the vessel, the aids ---------------------------------------------------------------------------------------------------------
    def step(self):
        ts = self.sim.tick * self.dt
        if self.pilot is not None:
            if ts >= self.t_go and not self._went:
                self.pilot.set_heading(self.ship_psi, self.ship_speed)
                self._went = True
            if self.sim.tick - self._pt >= 40:
                self.pilot.advance(_NoStep(self.sim))
                self._pt = self.sim.tick
        self._aids(ts)
        if (
            pad := self._pad_now()
        ) is not None:  # the pad's smoothed level (see _ref_z)
            z = float(pad[0][2])
            self._level = (
                z
                if self._level is None
                else self._level + (z - self._level) * self.dtc / TAU_LEVEL
            )
        super().step()

    def _aids(self, ts):
        q = self.sim.query
        hp, hq = q(f"/world/{self.hull}", "pos")[0], q(f"/world/{self.hull}", "quat")[0]
        self.tdeck = RN.deck_truth(ts, hp, hq, self.tdeck)
        msg = self.link.step(ts, self.tdeck)
        if msg is not None and (not self.msgs or msg.seq != self.msgs[-1].seq):
            self.msgs.append(msg)
            self.pred.add(msg, self.pad_off)
        if self.flyer.truth is None or not self.msgs:
            return
        pt, Rt = self.flyer.truth[0], self.flyer.truth[1]
        base = hp + RN.quat_to_mat(hq) @ self.base_off
        fix = self.relgnss.step(ts, base, pt + Rt @ ANT_ROOT)
        if fix is not None and fix.t_meas != self.fix_t:
            self.fix_t = fix.t_meas
            self._fuse(fix, ts)
        self.fix = fix

    def _fuse(self, fix, ts):
        """Base antenna at the fix epoch (DeckLink message at or before it, extrapolated) + the relative position -> the lander's position -> ESKF update."""
        if fix.state == RN.SINGLE:
            return
        msg = next(
            (m for m in reversed(self.msgs) if m.t_meas <= fix.t_meas), self.msgs[0]
        )
        base_w, _ = RN.deck_point_at(msg, self.base_off, fix.t_meas)
        z = base_w + fix.rel - self.eskf.R @ ANT_ROOT
        if np.linalg.norm(z - self.eskf.p) > 3.0:
            return
        L.update_position(
            self.eskf, self.eskf.t - (ts - fix.t_meas), z, math.hypot(fix.sigma_h, 0.05)
        )
        self.n_rtk += 1

    def _pad_now(self):
        """(pad centre, pad velocity) in the world from the newest DeckLink message extrapolated to now; None before the first."""
        if not self.msgs:
            return None
        return RN.deck_point_at(self.msgs[-1], self.pad_off, self.sim.tick * self.dt)

    # ---- phases -----------------------------------------------------------------------------------------------------------------------
    def _ph_map(self, p, v, R):
        self.flyer.target(
            [self.p_start[0], self.p_start[1], self.z_c], np.zeros(3), self.yaw_sp
        )
        if self.t - self.t_phase >= 1.0:
            self._enter("follow")

    def _heave(self):
        """(z, vz, az) of the pad point now from the DeckPredictor's autoregressive forecast of its height (smooth: no sensor jitter, no extrapolation lag), the forecast refitted
        every 0.25 s; the DeckLink extrapolation (z, vz, 0) before the predictor is ready."""
        ts = self.sim.tick * self.dt
        if self._hp is None or ts - self._hp[0] >= 0.25:
            self._hp = (ts, self.pred.predict(6.0, keys=("h",)))
        pr = self._hp[1]
        if pr is not None and pr[0][0] <= ts <= pr[0][-2]:
            t_f, h = pr[0], pr[1]["h"]
            v = np.gradient(h, self.pred.dt)
            a = np.gradient(v, self.pred.dt)
            return tuple(float(np.interp(ts, t_f, x)) for x in (h, v, a))
        pad = self._pad_now()
        return None if pad is None else (float(pad[0][2]), float(pad[1][2]), 0.0)

    def _ref_z(self, w):
        """Height reference of the pad as (z, vz, az): the pad's smoothed level (tau TAU_LEVEL) blended by `w` (0..1) toward its forecast height. A pad that heaves at 3 m/s^2 in
        a head sea is not followed raw (measured: 35-45 deg tilt excursions chasing its sensor jitter and its extrapolation lag): the lander rides the level while it waits and
        goes over to the forecast for the sink, with the acceleration fed forward (a position offset az / kz: the altitude loop then has nothing left to track)."""
        hv = self._heave()
        if hv is None:
            return None
        if self._level is None:
            self._level = hv[0]
        return (self._level + w * (hv[0] - self._level), w * hv[1], w * hv[2])

    def _track(self, h, vz=0.0, w=0.0):
        """Command: gear height `h` over the pad point (horizontally moving with it; vertically see `_ref_z`); yaw with the vessel."""
        pad, ref = self._pad_now(), self._ref_z(w)
        if pad is None or ref is None:
            return None
        pp, pv = pad
        self.yaw_sp = self.msgs[-1].yaw
        self.flyer.target(
            [pp[0], pp[1], ref[0] + ref[2] / WN_Z**2 + GEAR_H + h],
            [pv[0], pv[1], ref[1] + vz],
            self.yaw_sp,
        )
        return pp, pv

    def _rel_h(self, p, w=0.0):
        ref = self._ref_z(w)
        return None if ref is None else float(p[2] - ref[0] - GEAR_H)

    def _ph_follow(self, p, v, R):
        self._track(HOVER_H)
        ts = self.sim.tick * self.dt
        if (
            ts >= self.t_wait
            and self.pred.ready()
            and (h := self._rel_h(p)) is not None
            and abs(h - HOVER_H) < 0.5
        ):
            self.t_wait0 = ts
            self._enter("wait")

    def _ph_wait(self, p, v, R):
        ts = self.sim.tick * self.dt
        h = H_WAIT + (HOVER_H - H_WAIT) * max(
            0.0, 1.0 - (ts - self.t_wait0) / 4.0
        )  # eased down to the waiting height in 4 s
        self._track(h)
        if ts - self.t_wait0 < 4.0:
            return
        t0 = sink_start(self.pred, ts)
        self.window_log.append((round(ts, 2), None if t0 is None else round(t0, 2)))
        timeout = ts - self.t_wait0 > self.wait_max
        if timeout or (t0 is not None and ts >= t0):
            self.forced = timeout and t0 is None
            self.t_sink0, self.h_sink0 = ts, self._rel_h(p)
            self._enter("sink")

    def _ph_sink(self, p, v, R):
        ts = self.sim.tick * self.dt
        h = self.h_sink0 - V_SINK * (ts - self.t_sink0)
        w = min((ts - self.t_sink0) / T_BLEND, 1.0)
        self._track(max(h, -0.3), -V_SINK, w)
        low = self._agl_buf and np.mean(self._agl_buf) < self.lp.touch_agl
        if low or h < -0.25:
            self.t_cut = ts  # (the contact; the motors go off after T_PRESS)
            self._enter("touchdown")

    def _ph_touchdown(self, p, v, R):
        """On a moving, tilting deck the lander is first PRESSED onto it with the attitude loop still running (the height command 0.35 m under the pad: the thrust drops to its floor,
        0.2 of the weight, the airframe stays level), then the motors go off. Cutting at the first contact tipped the lander over on the ASV's plate in 4 of 8 calm-water flights."""
        ts = self.sim.tick * self.dt
        pad, ref = self._pad_now(), self._ref_z(1.0)
        if ts - self.t_cut < T_PRESS and pad is not None and ref is not None:
            # horizontally: no position error to correct (the gear is on the deck, a tilted thrust would drag it), only the pad's velocity
            self.flyer.target(
                [p[0], p[1], ref[0] + ref[2] / WN_Z**2 + GEAR_H - 0.35],
                [pad[1][0], pad[1][1], ref[1] - V_SINK],
                self.yaw_sp,
            )
        elif self.flyer.alive:
            self.flyer.cut()
        if ts - self.t_cut > T_STAY:
            self.t_landed = self.t
            self._enter("landed")

    COLS = SkydioLand.COLS + [
        "pad_x",
        "pad_y",
        "pad_z",
        "sp_x",
        "sp_y",
        "sp_z",
    ]  # the true pad centre and the command

    def _trace(self, p, R_true, v, tilt_cmd):
        super()._trace(p, R_true, v, tilt_cmd)
        pad = self._deck_truth()[0] if self.tdeck is not None else np.zeros(3)
        sp = self.flyer.sp[0]
        self.rows[-1] += [*map(float, pad), *map(float, sp)]

    # ---- truth records ----------------------------------------------------------------------------------------------------------------
    def _deck_truth(self):
        """(pad centre, pad velocity, deck rotation matrix) of the true hull."""
        t = self.tdeck
        Rd = RN.quat_to_mat(t["quat"])
        return (
            t["pos"] + Rd @ self.pad_off,
            t["vel"] + Rd @ np.cross(t["omega"], self.pad_off),
            Rd,
        )

    def _offset_in_deck(self):
        pad, _, Rd = self._deck_truth()
        rel = Rd.T @ (self.flyer.truth[0] - pad)
        return rel

    def _truth_contacts(self):
        q = self.sim.query
        go, gg = (
            float(q(f"/sensors/{s}", "value")[0][0])
            for s in ("hit_obstacles", "hit_ground")
        )
        gear = sum(float(q(f"/sensors/hit_gear_{l}", "value")[0][0]) for l in "fr")
        pt, Rt, vt = self.flyer.truth[0], self.flyer.truth[1], self.flyer.truth[2][:3]
        pad, padv, Rd = self._deck_truth()
        self.world.pad_z = float(pad[2])
        if not self.airborne and pt[2] - GEAR_H - pad[2] > 0.3:
            self.airborne = True
        self.vz_hist = (self.vz_hist + [float((vt - padv) @ Rd[:, 2])])[-3:]
        if go > 0:
            self.n_obst += 1
        if not self.airborne:
            return
        if gg > 0:
            self.n_ground += 1
        if max(gg - gear, 0.0) > 0:
            self.n_crash += 1
        if gear > 0 and self.td is None:
            rel = Rd.T @ (pt - pad)
            tilt = math.degrees(
                math.acos(max(-1.0, min(1.0, float(Rt[:, 2] @ Rd[:, 2]))))
            )
            safe = bool(
                abs(rel[0]) <= self.half[0] - self.margin
                and abs(rel[1]) <= self.half[1] - self.margin
            )
            self.td = dict(t=self.t, speed=-min(self.vz_hist), tilt_deg=tilt, dist_safe_m=float(math.hypot(rel[0], rel[1])), zone="deck", nearest="deck", on_safe=safe,
                           safe=safe, pos=np.round(pt, 3).tolist(), offset_deck=np.round(rel, 3).tolist())  # fmt: skip

    def _final_ok(self):
        if self.tdeck is None or self.flyer.truth is None:
            return False
        rel = self._offset_in_deck()
        Rt, Rd = self.flyer.truth[1], self._deck_truth()[2]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Rt[:, 2] @ Rd[:, 2])))))
        return bool(
            abs(rel[0]) <= self.half[0] - 0.1
            and abs(rel[1]) <= self.half[1] - 0.1
            and abs(rel[2] - GEAR_H) < 0.1
            and tilt < 25.0
        )

    def _extra(self):
        return dict(deck=dict(kind=self.land["deck"]["kind"], forced_landing=self.forced, rtk_updates=self.n_rtk, rtk_states=dict(self.relgnss.n), link_sent=self.link.sent,
                         link_lost=self.link.lost, pred_rms_h=self.pred.rms("h"), windows=self.window_log[-3:], on_deck_at_end=self._final_ok()))  # fmt: skip
