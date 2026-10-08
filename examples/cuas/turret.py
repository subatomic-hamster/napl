"""The turret program (user code): a pan / tilt turret with a camera that tracks several flying drones and fires a generic 4.5 mm (.177)
pneumatic pellet launcher at them, ONLY after a human operator authorizes.

What the program may use (and nothing else): camera frames (rgb pixels), the camera pose, the pan / tilt servo readouts, a mast anemometer,
the launcher's counters, and the `authorize` callback. It never reads the drones' true positions or the camera's depth image.

Stages, exposed as `Turret.stage` and in every HUD record:
  SENSE  no tracks: sweep the camera over the arena (pan sweep) and look for things that are not sky
  TRACK  something is seen but not yet confirmed (< 3 consistent frames): hold and watch
  PICK   several confirmed tracks: choose a target (nearest by estimated range, not recently given up on)
  AIM    solve the drag-aware lead with the wind estimate, slew pan / tilt to the solution
  FIRE   aimed (pointing error below tolerance) AND authorized: fire (the launcher is refused any shot without a valid authorization)

Pipeline per frame: sky detection -> blobs -> multi-target tracker (stable IDs A, B, C..) -> range from apparent size -> bearing from the
pixel + pan / tilt (the camera pose) -> world position estimate -> alpha-beta velocity -> lead solution (napl.cuas.ballistics) -> servo goals.

Run through `napl program examples/cuas/cuas.scenario.toml` (the runner builds the scene and judges the run); `python turret.py` with
NAPL_MODEL / NAPL_RUN_DIR set also works.
"""

import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from napl.cuas import ballistics
from napl.cuas.camera import Camera, write_png
from napl.cuas.tracking import Tracker, detect, horizon_row
from napl.cuas.world import World
from swarm import DEFAULT_SEED, Swarm

FRAME_STRIDE = (
    17  # control periods (4 ms) per camera frame: 14.7 Hz, the camera's nominal 15 Hz
)
SIM_S = float(os.environ.get("CUAS_SIM_S", 60.0))
SERVO_SPEED = 4.0  # rad/s profile velocity of pan and tilt
SERVO_P_GAIN = 1500  # runtime P gain of pan / tilt (default 640): a 0.2 rad/s ramp lags its goal by 65 ms at 640, 15 ms at 1500
SERVO_LAG = 0.05  # s: 15 ms servo lag + half a frame interval (the goal is held between frames); the goal is set this far ahead
AIM_TOL = 0.003  # rad (0.17 deg: 0.12 m at 40 m): pointing error under which the turret may fire
SHOT_INTERVAL = 0.5  # s (the launcher part's min_shot_interval)
GIVE_UP_ROUNDS = 5  # rounds at one target before the turret moves on
GIVE_UP_S = 6.0  # ... and how long that target stays off the pick list
MAX_RANGE, MIN_ELEV = 55.0, math.radians(2.0)
SCAN_PAN, SCAN_RATE, SCAN_TILT = math.radians(60), 0.45, math.radians(20)


@dataclass
class Request:
    """What the operator is shown when the turret asks for authorization."""

    target: str
    range_m: float
    az_deg: float
    el_deg: float
    world_xyz_est: tuple
    rounds: int
    t: float


def operator(req: Request):
    """The scripted human operator of the demo: weapons free on a target inside the engagement envelope (range <= 55 m, shot elevation >= 2 deg,
    never toward the ground), 3 rounds per authorization. -> (granted, reason). A real operator would be a person at a console."""
    if req.range_m > MAX_RANGE:
        return False, "outside the engagement envelope"
    if req.el_deg < math.degrees(MIN_ELEV):
        return False, "shot too close to the ground"
    return True, "target inside the envelope"


class Turret:
    def __init__(self, world: World, authorize=operator, seed=1, frames_dir=None):
        self.w, self.authorize = world, authorize
        self.cam = Camera(
            world.sim,
            world.cam_path,
            world.cam_cfg["fov_v"],
            world.cam_cfg["width"],
            world.cam_cfg["height"],
        )
        self.tracker = Tracker(self.cam)
        self.pellet = ballistics.Pellet(**self._pellet_cfg())
        self.rng = np.random.default_rng(seed + 1000)
        self.wind = np.zeros(3)
        self.stage, self.target, self.tracks = "SENSE", None, []
        self.cmd = (0.0, SCAN_TILT)
        self.scan_dir = 1.0
        self.last_shot, self.last_req, self.shots_at, self.given_up = -9.0, -9.0, {}, {}
        self.auth = None
        self.frames_dir = Path(frames_dir) if frames_dir else None
        if self.frames_dir:
            self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.solution, self.pointing_err = None, 0.0
        for s in (world.pan, world.tilt):
            world.sim.set(s, "torque_enable", 1)
            world.sim.set(s, "goal_speed", SERVO_SPEED)
            world.sim.set(s, "p_gain", SERVO_P_GAIN)
        world.aim(*self.cmd)
        self.log = []

    def _pellet_cfg(self):
        g = self.w.man["device_cfg"][self.w.gun.rsplit("/", 1)[1]]
        return dict(
            v0=g["muzzle_velocity"],
            mass=g["pellet_mass"],
            diameter=g["diameter"],
            cd=g["cd"],
        )

    # ---- one camera frame ------------------------------------------------------------------------------------------
    def on_frame(self):
        w = self.w
        fr = self.cam.grab(t=w.t)
        blobs = detect(fr.rgb, horizon_row(self.cam, fr))
        self.tracks = self.tracker.update(blobs, fr)
        a = 0.08
        self.wind = (
            (1 - a) * self.wind + a * w.wind_sensor(self.rng)
            if self.wind.any()
            else w.wind_sensor(self.rng)
        )
        conf = [t for t in self.tracks if t.confirmed and t.misses == 0]
        for k in [k for k, v in self.given_up.items() if v < w.t]:
            del self.given_up[k]
        pick = [t for t in conf if t.id not in self.given_up and 8.0 < t.range_m < 90.0]
        self.solution, fired = None, False
        if not self.tracks:
            self.stage, self.target = "SENSE", None
            self._scan()
        elif not conf:
            self.stage, self.target = "TRACK", None
        elif not pick:
            self.stage, self.target = "TRACK", None
            self._scan()
        else:
            cur = next((t for t in pick if t.id == self.target), None)
            picked = cur is None
            if (
                picked
            ):  # a new target: this frame is the PICK stage (aim starts, no shot yet)
                cur = min(pick, key=lambda t: t.range_m)
                self.target = cur.id
            self.stage = "PICK" if picked else "AIM"
            fired = self._aim_and_fire(cur, allow_fire=not picked)
        self._hud(fr, fired)
        return fr

    def _scan(self):
        pan, _ = self.cmd
        pan += self.scan_dir * SCAN_RATE * FRAME_STRIDE * self.w.dtc
        if abs(pan) > SCAN_PAN:
            self.scan_dir *= -1
            pan = max(-SCAN_PAN, min(SCAN_PAN, pan))
        self.cmd = (pan, SCAN_TILT)
        self.w.aim(*self.cmd)

    def _aim_and_fire(self, tr, allow_fire=True):
        w = self.w
        m, d = w.muzzle()
        lat = 0.0  # s: the frame is rendered at the current sim time, the fire command acts on the next tick
        sol = ballistics.solve_lead(
            self.pellet, m, tr.pos + tr.vel * lat, tr.vel, self.wind
        )
        self.solution = sol
        if sol is None:
            return False
        # the servos lag their goal by ~SERVO_LAG s: command where the solution will be then, judge the pointing against the solution NOW
        ahead = (
            ballistics.solve_lead(
                self.pellet, m, tr.pos + tr.vel * (lat + SERVO_LAG), tr.vel, self.wind
            )
            or sol
        )
        self.cmd = (ahead["az"], ahead["el"])
        w.aim(*self.cmd)
        err = math.acos(max(-1.0, min(1.0, float(d @ sol["u"]))))
        self.pointing_err = err
        if err > AIM_TOL or not sol["ok"] or not allow_fire:
            return False
        # aimed: the gate
        a = w.auth
        if a is None or a.target != tr.id or a.rounds <= 0 or w.t > a.expires:
            if w.t - self.last_req < SHOT_INTERVAL:
                return False
            self.last_req = w.t
            req = Request(
                tr.id,
                tr.range_m,
                math.degrees(sol["az"]),
                math.degrees(sol["el"]),
                tuple(float(x) for x in tr.pos),
                3,
                w.t,
            )
            ok, why = self.authorize(req)
            w.events.append(
                dict(
                    type="authorization_request",
                    t=w.t,
                    target=tr.id,
                    range_m=tr.range_m,
                    granted=bool(ok),
                    reason=why,
                )
            )
            if not ok:
                self.given_up[tr.id] = w.t + 2.0
                return False
            a = w.authorize(tr.id, rounds=req.rounds)
        self.stage = "FIRE"
        if w.t - self.last_shot < SHOT_INTERVAL:
            return False
        if w.fire(a, tr.id):
            self.last_shot = w.t
            self.shots_at[tr.id] = self.shots_at.get(tr.id, 0) + 1
            if self.shots_at[tr.id] >= GIVE_UP_ROUNDS:
                self.shots_at[tr.id] = 0
                self.given_up[tr.id] = w.t + GIVE_UP_S
                self.target = None
            return True
        return False

    # ---- telemetry --------------------------------------------------------------------------------------------------
    def _hud(self, fr, fired):
        w = self.w
        path = None
        if self.frames_dir:
            path = f"frames/f{fr.index:05d}.png"
            write_png(self.frames_dir / f"f{fr.index:05d}.png", fr.rgb)
        pan, tilt = w.pan_tilt()
        m, d = w.muzzle()
        s = w.summary()
        rec = dict(
            t=round(w.t, 4), frame=fr.index, stage=self.stage, selected=self.target,
            tracks=[dict(id=t.id, bbox_px=list(t.bbox), range_m=round(t.range_m, 2), world_xyz_est=[round(float(x), 2) for x in t.pos],
                         vel_est=[round(float(x), 2) for x in t.vel], confirmed=t.confirmed, coasting=t.misses > 0) for t in self.tracks],
            rounds=w.rounds, hits=s["hits_on_drones"], destroyed=s["drones_destroyed"], damaged=s["drones_damaged"], fired=fired,
            projectiles=w.pellets_active(), tracers=w.tracers_recent(0.5),
            muzzle=dict(pos=[round(float(x), 3) for x in m], dir=[round(float(x), 4) for x in d], pan=round(pan, 4), tilt=round(tilt, 4)),
            wind_est=[round(float(x), 2) for x in self.wind],
            lead=None if self.solution is None else dict(az=round(self.solution["az"], 4), el=round(self.solution["el"], 4),
                                                           t_flight=round(float(self.solution["t_flight"]), 3), ok=self.solution["ok"], aim_err=round(self.pointing_err, 5)),
            camera_frame=path,
        )  # fmt: skip
        w.hud.emit(rec)


def console(rec):
    """Live HUD callback: one line per second of sim time in program.log (a GUI would draw `rec` instead: same dict as hud.jsonl)."""
    if rec["frame"] % 15 == 0:
        trk = ",".join(
            f"{t['id']}@{t['range_m']:.0f}m" for t in rec["tracks"] if t["confirmed"]
        )
        print(  # ttsr-ignore: py/no-print (the console HUD of a CLI program: stdout is program.log)
            f"[{rec['t']:6.2f} s] {rec['stage']:5s} sel={rec['selected']} tracks=[{trk}] rounds={rec['rounds']} hits={rec['hits']} destroyed={rec['destroyed']}",
            flush=True,
        )


def main():
    w = World.from_env(hud_callbacks=[console])
    seed = int(os.environ.get("CUAS_SEED", DEFAULT_SEED))
    swarm = Swarm(list(w.drones), seed)
    turret = Turret(
        w, operator, seed, frames_dir=Path(os.environ["NAPL_RUN_DIR"]) / "frames"
    )
    n_total = int(SIM_S / w.dtc)
    for k in range(n_total):
        alive = {n for n, d in w.drones.items() if d.status == "flying"}
        sp = swarm.step(w.dtc, alive)
        for n in alive:
            w.drones[n].goto(*sp[n])
        w.step()
        if k % FRAME_STRIDE == FRAME_STRIDE - 1:
            turret.on_frame()
    w.finish()
    print(  # ttsr-ignore: py/no-print (the final summary line of a CLI program)
        json.dumps(
            {k: v for k, v in w.summary().items() if k != "drones"}, default=float
        )
    )


if __name__ == "__main__":
    main()
