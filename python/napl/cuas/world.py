"""The counter-UAS World: a composed scene (cuas.scene.compose) stepped by ONE program, with the programmable drones, the turret's pan / tilt /
fire interface, hit -> damage handling, HUD telemetry and the run artefacts `napl program` (scenario `[interfaces.native]`) judges.

    w = World.from_env()                       # NAPL_MODEL (the scene.xml) + NAPL_RUN_DIR set by `napl program`; World(model, run_dir) otherwise
    w.drones["drone1"].goto((30, 5, 12))       # position / velocity setpoints, flown by cuas.flyer.Flyer (napl-native controller)
    w.step(n)                                  # n control periods (4 ticks = 4 ms each by default): drones update, kernel steps, hits are polled
    w.aim(az, el); tok = w.authorize(...); w.fire(tok)   # the turret: pan / tilt servo goals, an authorization token, the FIRE command
    w.finish()                                 # trace.csv, replay.naplog, native.json (tick, state hash), hits.json, summary.json

Determinism. The sim is the kernel's: every command (throttles, servo goals, fire) goes through the bus and into the replay log, so
`napl-replay` reproduces the final state hash. The hash does not depend on the camera (frames are only read), but the program's DECISIONS do
(they come from pixels): see cuas/camera.py for what that means across GPUs.

Hit -> damage (a scenario assumption, not a lethality claim): the kernel reports the body each pellet struck; `DAMAGE` maps the body name to
a fault attr of the existing device layer: a prop -> rotor `damage` 1 (blade lost); an arm / motor / ESC tube -> that rotor's ESC `fault_open`
(power to the motor cut); the flight-controller / power board -> every ESC `fault_open`; the battery pack -> `r_fault` (a failing cell,
voltage sags); anything else (frame plates, legs, mast, GPS ...) -> no functional damage. A drone is "destroyed" when it has functional damage
and then reaches the ground (z < 1 m) or tumbles (tilt > 80 deg).
"""

import csv
import json
import os
import re
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .. import Sim
from . import flyer as _flyer

RE_PROP, RE_ARM = (
    re.compile(r"^prop(\d+)$"),
    re.compile(r"^(mot|tube|arm|con|esc)(\d+)$"),
)
BOARD = {
    "fc",
    "pdb",
    "pm02",
    "esc",
}  # the flight controller / power board bodies (aos5 calls its board "esc")
PACK = {"bat", "batmount", "rod_l", "rod_r", "hw"}
LOST_Z, LOST_TILT = 1.0, np.radians(80)
ENGAGE_M = 3.0  # a pellet whose closest approach to a drone is under this was shot AT that drone (evaluation only)


@dataclass
class Authorization:
    target: str
    t: float
    rounds: int
    expires: float
    by: str = "operator"


@dataclass
class Drone:
    name: str
    flyer: object
    root: str
    rotors: list  # device paths
    escs: list
    battery: str
    rotor_bodies: list  # prim paths of the rotor bodies (for 'nearest rotor' damage)
    status: str = "flying"  # flying | damaged | destroyed
    damage: list = field(default_factory=list)  # [{t, body, effect}]
    t_hit: float = None
    t_destroyed: float = None
    hits: int = 0

    offset: np.ndarray = None  # the root body's position minus the vehicle's placement position (goto takes vehicle-origin positions)

    def goto(self, pos, vel=(0.0, 0.0, 0.0), yaw=None):
        self.flyer.target(np.asarray(pos, float) + self.offset, vel, yaw)

    @property
    def pos(self):
        return self.flyer.last[0] if self.flyer.last else None


class Hud:
    """JSON lines per frame in <run dir>/hud.jsonl plus any number of live callbacks."""

    def __init__(self, run_dir, callbacks=()):
        self.f = open(Path(run_dir) / "hud.jsonl", "w")
        self.callbacks = list(callbacks)
        self.n = 0

    def emit(self, rec):
        self.f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.f.flush()
        self.n += 1
        for cb in self.callbacks:
            cb(rec)

    def close(self):
        self.f.close()


class World:
    def __init__(
        self, model, run_dir, seed=1, ctl_ticks=4, hud_callbacks=(), cal_dir=None
    ):
        self.model, self.run_dir = Path(model), Path(run_dir)
        self.man = json.loads((self.model.parent / "scene.json").read_text())
        self.sim = Sim.load(self.model)
        self.sim.start_log()  # before any command: the replay log must start at tick 0
        self.dt, self.nctl = self.man["dt"], ctl_ticks
        self.dtc = self.dt * ctl_ticks
        self.seed = seed
        self.hud = Hud(self.run_dir, hud_callbacks)
        names = list(self.sim.body_positions())  # body prims in MuJoCo body-id order
        want = ["/world"] + [
            self.man["body_paths"][n] for n in self.man["body_names"][1:]
        ]
        if names[: len(want)] != want:
            raise RuntimeError(
                "scene manifest body order differs from the kernel's body order"
            )
        self.body_path = want
        cfg, dev = self.man["device_cfg"], self.man["devices"]
        self.turret = next(
            v
            for v in self.man["vehicles"]
            if any(n.startswith(v + "_") for n in dev.get("launchers", []))
        )
        t = self.turret
        self.gun, self.cam_path = f"/devices/{t}_gun", f"/devices/{t}_cam"
        self.pan, self.tilt = f"/devices/{t}_pan", f"/devices/{t}_tilt"
        self.cam_cfg = cfg[f"{t}_cam"]
        self.air = "/devices/air"
        self.drones: dict = {}
        cal_dir = Path(cal_dir or self.run_dir / "cal")
        cals = {}
        for name, v in self.man["vehicles"].items():
            rotors = [cfg[n] for n in dev.get("rotors", []) if n.startswith(name + "_")]
            if not rotors:
                continue
            escs = {
                cfg[n]["joint"]: cfg[n]
                for n in dev.get("escs", [])
                if n.startswith(name + "_")
            }
            cal = cals.setdefault(
                v["assembly"], _flyer.calibrate(v["assembly"], cal_dir)
            )
            root = next(
                p
                for p in self.man["body_paths"].values()
                if p.startswith(f"/world/{name}_") and p.count("/") == 2
            )
            orig = root.rsplit("/", 1)[1][len(name) + 1 :]
            paths = dict(root=root, free=f"{root}/{orig}:free", escs=[f"/devices/{escs[r['joint']]['name']}" for r in rotors],
                         yaw0=float(np.radians(v["yaw"])))  # fmt: skip
            self.drones[name] = Drone(
                name, _flyer.Flyer(self.sim, cal, paths, self.dtc, max_tilt=0.35), root, [f"/devices/{r['name']}" for r in rotors], paths["escs"],
                f"/devices/{name}_battery", [self.body_path[self.man["body_names"].index(r["body"])] for r in rotors])  # fmt: skip
        for (
            name,
            d,
        ) in self.drones.items():  # hold the placement until the program says otherwise
            p0 = self._q(d.root, "pos")
            d.offset = p0 - np.array(self.man["vehicles"][name]["pos"])
            R0 = _flyer._R(self._q(d.root, "quat"))
            d.flyer.yaw = float(
                np.arctan2(R0[1, 0], R0[0, 0])
            )  # hold the heading the model starts with
            d.flyer.target(p0)
        self.k = 0  # control periods done
        self.rows, self.events, self.truth = [], [], []
        self.hit_seen, self.in_flight = 0, 0
        self.tracers = deque(maxlen=48)  # (round_no, p0, p1, t)
        self.prev_pellets: dict = {}
        self.miss: dict = {}  # round -> dict(dist, drone, t, vec): closest approach to any drone's root (evaluation only, the turret never sees it)
        self.rounds = self.hits = self.ground = 0
        self.fires_unauthorized = 0
        self.engaged: set = set()
        self.pellet_range = 0.0
        self.auth: Authorization = None
        self.round_target: dict = {}  # round_no -> target id the turret said it fired at
        self.cur_target = None
        self.wall0 = time.monotonic()

    @classmethod
    def from_env(cls, **kw):
        return cls(os.environ["NAPL_MODEL"], os.environ["NAPL_RUN_DIR"], **kw)

    @property
    def t(self):
        return self.k * self.dtc

    # ---- stepping -------------------------------------------------------------------------------------------------
    def step(self, n=1):
        for _ in range(n):
            for d in self.drones.values():
                d.flyer.update()
            self.sim.step(self.nctl)
            self.k += 1
            self._poll()
            self._status()
            self._trace()

    def _q(self, path, attr):
        return self.sim.query(path, attr)[0]

    def _poll(self):
        c = self._q
        rounds, hits, ground, expired = (
            int(c(self.gun, a)[0]) for a in ("rounds", "hits", "ground", "expired")
        )
        self.rounds, self.hits, self.ground = rounds, hits, ground
        self.in_flight = rounds - hits - ground - expired
        cur = {}
        if self.in_flight > 0 or self.prev_pellets:
            act, rno, pos = (
                c(self.gun, "active"),
                c(self.gun, "round_no"),
                c(self.gun, "pos").reshape(-1, 3),
            )
            for i in np.nonzero(act > 0.5)[0]:
                cur[int(rno[i])] = pos[i].copy()
            m = np.array(c(self.gun, "muzzle_pos"))
            self.pellet_range = max(
                (float(np.hypot(*(p - m)[:2])) for p in cur.values()), default=0.0
            )
            for r, p in cur.items():
                if r in self.prev_pellets:
                    self.tracers.append((r, self.prev_pellets[r], p, self.t))
                    self._miss(r, self.prev_pellets[r], p)
        else:
            self.pellet_range = 0.0
        self.prev_pellets = cur
        n = int(c(self.gun, "hit_n")[0])
        if n - self.hit_seen > 16:
            raise RuntimeError("hit ring overflow: poll more often")
        for k in range(self.hit_seen, n):
            self._hit(k % 16)
        self.hit_seen = n

    def _hit(self, i):
        c = self._q
        body, rnd = int(c(self.gun, "hit_body")[i]), int(c(self.gun, "hit_round")[i])
        p, v = (
            c(self.gun, "hit_pos").reshape(-1, 3)[i],
            float(c(self.gun, "hit_speed")[i]),
        )
        path = self.body_path[body]
        ev = dict(
            t=self.t,
            round=rnd,
            body=path,
            pos=p.tolist(),
            speed=v,
            target=self.round_target.get(rnd),
        )
        top = path.split("/")[2] if body else ""
        d = (
            next(
                (d for d in self.drones.values() if top.startswith(d.name + "_")), None
            )
            if body
            else None
        )
        if d is None:
            ev.update(kind="ground" if body == 0 else "other")
            self.events.append(dict(ev, type="hit"))
            return
        d.hits += 1
        ev.update(
            kind="drone",
            drone=d.name,
            effect=self._damage(d, path.rsplit("/", 1)[1], p),
        )
        self.events.append(dict(ev, type="hit"))

    def _damage(self, d, leaf, p):
        near = lambda: int(
            np.argmin([np.linalg.norm(self._q(b, "pos") - p) for b in d.rotor_bodies])
        )
        eff = "none"
        if RE_PROP.match(leaf):
            k = near()
            self.sim.set(d.rotors[k], "damage", 1.0)
            eff = f"rotor{k + 1}.damage=1"
        elif RE_ARM.match(leaf):
            k = near()
            self.sim.set(d.escs[k], "fault_open", 1.0)
            eff = f"esc{k + 1}.fault_open=1"
        elif leaf in BOARD:
            for e in d.escs:
                self.sim.set(e, "fault_open", 1.0)
            eff = "all escs fault_open=1"
        elif leaf in PACK:
            self.sim.set(d.battery, "r_fault", 0.5)
            eff = "battery.r_fault=0.5"
        if eff != "none":
            d.damage.append(dict(t=self.t, body=leaf, effect=eff))
            if d.status == "flying":
                d.status, d.t_hit = "damaged", self.t
        return eff

    def _status(self):
        for d in self.drones.values():
            if d.status == "damaged" and d.flyer.last:
                p, _, _, tilt, _ = d.flyer.last
                if p[2] < LOST_Z or tilt > LOST_TILT:
                    d.status, d.t_destroyed = "destroyed", self.t
                    d.flyer.cut()
                    self.events.append(dict(type="destroyed", t=self.t, drone=d.name))

    def _trace(self):
        if (
            self.k % 25 == 0
        ):  # ground truth, 10 Hz, for plots / evaluation only (the turret program never reads it)
            self.truth.append(
                [self.t]
                + [
                    x
                    for d in self.drones.values()
                    if d.flyer.last
                    for x in (
                        *d.flyer.last[0],
                        ["flying", "damaged", "destroyed"].index(d.status),
                    )
                ]
            )
        if self.k % 4:  # 62.5 Hz
            return
        fl = [d for d in self.drones.values() if d.status == "flying" and d.flyer.last]
        m = np.array(self._q(self.gun, "muzzle_pos"))
        rng = [
            float(np.linalg.norm(d.flyer.last[0] - m))
            for d in self.drones.values()
            if d.flyer.last
        ]
        self.rows.append([self.t, self.rounds, self.hits, self.ground, sum(d.status == "destroyed" for d in self.drones.values()),
                          sum(d.status != "flying" for d in self.drones.values()), len(self.engaged),
                          len({m["drone"] for m in self.miss.values() if m["dist"] < ENGAGE_M}), self.fires_unauthorized,
                          self.pellet_range, min((d.flyer.last[0][2] for d in fl), default=99.0),
                          max((d.flyer.last[3] for d in fl), default=0.0), min(rng, default=0.0)])  # fmt: skip

    TRACE_COLS = ["t", "rounds", "hits", "ground", "destroyed", "damaged", "engaged_tracks", "engaged_drones", "fires_unauthorized", "pellet_range_m", "flying_min_alt_m",
                  "flying_max_tilt_rad", "nearest_drone_m"]  # fmt: skip

    # ---- the turret -----------------------------------------------------------------------------------------------
    def aim(self, az, el, speed=None):
        """Pan / tilt servo goals (rad): pan = azimuth about +z from +x (CCW), tilt = elevation up."""
        self.sim.set(self.pan, "goal_position", float(az))
        self.sim.set(self.tilt, "goal_position", float(el))
        if speed:
            for s in (self.pan, self.tilt):
                self.sim.set(s, "goal_speed", float(speed))

    def pan_tilt(self):
        return float(self._q(self.pan, "position")[0]), float(
            self._q(self.tilt, "position")[0]
        )

    def muzzle(self):
        return np.array(self._q(self.gun, "muzzle_pos")), np.array(
            self._q(self.gun, "muzzle_dir")
        )

    def wind_sensor(self, rng):
        """The turret's mast anemometer: the environment's wind plus Gaussian noise (std 0.3 m/s per axis), the only wind knowledge the
        fire control gets (the true wind, incl. turbulence, stays inside the kernel)."""
        return np.array(self._q(self.air, "wind")) + rng.normal(0.0, 0.3, 3)

    def authorize(self, target, rounds=3, valid_s=1.5, by="operator"):
        """The human gate. Called ONLY by the operator callback; logged. -> Authorization token."""
        a = Authorization(target, self.t, rounds, self.t + valid_s, by)
        self.events.append(
            dict(
                type="authorization",
                t=self.t,
                target=target,
                rounds=rounds,
                by=by,
                text=f"operator authorized {target}",
            )
        )
        self.auth = a
        return a

    def fire(self, auth, target=None):
        """FIRE. Refused (and counted in `fires_unauthorized`) without a valid, unexpired authorization with rounds left. -> bool fired."""
        if (
            auth is None
            or auth is not self.auth
            or auth.rounds <= 0
            or self.t > auth.expires
            or (target and target != auth.target)
        ):
            self.fires_unauthorized += 1
            return False
        auth.rounds -= 1
        self.sim.set(self.gun, "fire", 1.0)
        self.round_target[self.rounds + 1] = (
            auth.target
        )  # round numbers count from 1 in the kernel
        self.engaged.add(auth.target)
        self.events.append(dict(type="fire", t=self.t, target=auth.target))
        return True

    # ---- results --------------------------------------------------------------------------------------------------
    def tracers_recent(self, horizon=0.5):
        return [
            dict(round=r, p0=a.tolist(), p1=b.tolist(), t=t)
            for r, a, b, t in self.tracers
            if self.t - t <= horizon
        ]

    def pellets_active(self):
        return [dict(round=r, pos=p.tolist()) for r, p in self.prev_pellets.items()]

    def _miss(self, r, a, b):
        ab = b - a
        for d in self.drones.values():
            if not d.flyer.last:
                continue
            q = d.flyer.last[0]
            s = float(np.clip((q - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0))
            vec = a + s * ab - q
            dist = float(np.linalg.norm(vec))
            if r not in self.miss or dist < self.miss[r]["dist"]:
                self.miss[r] = dict(
                    dist=dist,
                    drone=d.name,
                    t=self.t,
                    vec=[round(float(x), 3) for x in vec],
                )

    def summary(self):
        ds = {
            n: dict(
                status=d.status,
                hits=d.hits,
                t_hit=d.t_hit,
                t_destroyed=d.t_destroyed,
                damage=d.damage,
            )
            for n, d in self.drones.items()
        }
        h = sum(d.hits for d in self.drones.values())
        return dict(rounds=self.rounds, hits_on_drones=h, hits_other=self.hits - h, ground_impacts=self.ground,
                    hit_rate=(h / self.rounds if self.rounds else None), drones_damaged=sum(d.status != "flying" for d in self.drones.values()),
                    drones_destroyed=sum(d.status == "destroyed" for d in self.drones.values()), engaged_tracks=sorted(self.engaged), drones=ds,
                    engaged_drones=sorted({m["drone"] for m in self.miss.values() if m["dist"] < ENGAGE_M}),
                    fires_unauthorized=self.fires_unauthorized, sim_s=self.t, wall_s=time.monotonic() - self.wall0,
                    closest_approach={str(k): v for k, v in sorted(self.miss.items())})  # fmt: skip

    def finish(self):
        """Write the artefacts the runner judges. Idempotent per run."""
        r = self.run_dir
        self.hud.close()
        with open(r / "trace.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.TRACE_COLS)
            w.writerows(self.rows)
        with (
            open(r / "truth.csv", "w", newline="") as f
        ):  # drone positions (root bodies) + status 0 flying / 1 damaged / 2 destroyed
            w = csv.writer(f)
            w.writerow(["t"] + [f"{n}_{c}" for n in self.drones for c in "xyzs"])
            w.writerows(self.truth)
        (r / "replay.naplog").write_bytes(self.sim.log())
        (r / "events.json").write_text(json.dumps(self.events, indent=1))
        (r / "summary.json").write_text(
            json.dumps(self.summary(), indent=1, default=float)
        )
        (r / "native.json").write_text(json.dumps(dict(tick=self.sim.tick, dt=self.dt, state_hash=f"{self.sim.state_hash():016x}",
                                                       diverged=bool(self.sim.diverged))))  # fmt: skip
