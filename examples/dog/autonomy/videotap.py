"""Opt-in per-frame tap for the video (SP6): reads the autonomy and locomotion state a diagram needs, never writes to the sim.
Enabled by scene.run when DOG_TAP_DIR is set. Missing attributes (a carrier has no legs) are stored as nan."""

import json
from pathlib import Path

import numpy as np

MODES = ["STANDUP", "STAND", "WALK", "FALLEN", "SIT"]
GAITS = [
    "stand",
    "walk",
    "trot",
    "flying_trot",
    "bound",
]  # SP4 gait names (loco.gait.GAITS); an unknown name is code -1


def _g(o, *names):
    for n in names:
        o = getattr(o, n, None)
        if o is None:
            return None
    return o


class Tap:
    def __init__(self, out, every=5, map_every_s=10.0, pool=2):
        self.dir, self.every, self.map_every, self.pool = (
            Path(out),
            int(every),
            float(map_every_s),
            int(pool),
        )
        self.dir.mkdir(parents=True, exist_ok=True)
        self.r, self.path_xy, self.maps, self.map_t, self.last_map, self.shape = (
            [],
            [],
            [],
            [],
            -1e9,
            (0, 0),
        )

    def __call__(self, k, car, rob):
        if k % self.every:
            return
        p, _ = car.truth()
        t, x = float(car.time_of_last_frame), rob.loc.x
        ctl = getattr(car, "ctl", None)
        contact = _g(ctl, "st", "contact")
        force = _g(ctl, "f")
        tgt = _g(ctl, "target")
        mode, gait = _g(ctl, "mode"), _g(ctl, "sched", "g", "name")
        nan4 = np.full(4, np.nan)
        self.r.append(
            dict(
                t=t,
                truth_xy=np.array(p[:2], float),
                est_xy=np.array(x[:2], float),
                est_yaw=float(x[3]),
                phase=int(rob.ex.phase),
                mode=MODES.index(mode) if mode in MODES else -1,
                gait=GAITS.index(gait) if gait in GAITS else -1,
                contact=np.array(contact, float) if contact is not None else nan4,
                force_z=np.array(force, float)[:, 2] if force is not None else nan4,
                foot_xy=np.array(tgt, float)[:, :2]
                if tgt is not None
                else np.full((4, 2), np.nan),
            )
        )  # fmt: skip  (np.array copies: the controller reuses its buffers)
        self.path_xy.append(np.array(rob.path, np.float32).reshape(-1, 2))
        if t - self.last_map >= self.map_every:
            self.last_map = t
            self.map_t.append(t)
            m = np.asarray(rob.map.mapped(self.pool), bool)
            self.maps.append(np.packbits(m))
            self.shape = m.shape

    def close(self, state_hash, dt):
        if not self.r:
            raise ValueError("empty tap")
        cols = {k: np.array([r[k] for r in self.r]) for k in self.r[0]}
        np.savez_compressed(
            self.dir / "tap.npz", **cols, path_len=np.array([len(p) for p in self.path_xy]),
            path_xy=np.concatenate(self.path_xy),
            map_t=np.array(self.map_t), mapped_packed=np.stack(self.maps), mapped_shape=np.array(self.shape),
        )  # fmt: skip
        (self.dir / "tap.json").write_text(json.dumps(dict(
            n=len(self.r), dt=dt, state_hash=state_hash, every=self.every, pool=self.pool, cols=sorted(cols))))  # fmt: skip
