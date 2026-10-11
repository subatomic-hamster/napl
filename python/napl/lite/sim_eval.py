"""Harness-side evaluator of a cuas-lite lockstep run (ground truth the app never sees): every ball the SimRP2040 feeds is checked against the safety
table (spec 3.2 / 7.3) using the truth. Call it after every control period (Lockstep.run(until=ref)).

    ref = Referee(h)                     # h: Lockstep; persons = targets of class "person"
    h.run(60.0, until=ref)
    ref.violations                       # dict of counters, every one must be 0; ref.ok

Latency allowances (the chain between a truth event and the MCU's gate, measured against the sim time of the ball): a person that has been in the image for
GRACE_S or less may still be unseen by the app (camera 30 ms + frame period 17 ms + detector 15 ms + heartbeat 50 ms + link, and the MCU re-checks the inhibit when
the ball is fed); the dead-man is fresh for 0.5 s after its last post; the detector is stale 0.25 s after its last reply (+ one heartbeat). The raw count of balls
fed while a person is in the image at all (`person_raw`) is reported next to the strict one."""

import math

import numpy as np

from ..cuas.camera import Frame as PoseFrame
from . import protocol as p

GRACE_S = 0.25  # person in image at most this long before a ball: inside the latency chain, not a violation
HOLD_S = 1.0  # the app's person hold after the last sighting
DEADMAN_S = 0.5 + 0.1
STALE_S = 0.25 + 0.1
# s7: a ball may still be fed this long after the cut (spec 3.1 / 3.2): hardware rail and key within a control tick; the dead-man is fresh for 0.5 s (+ heartbeat and link); a stale
# detector inhibits after 0.25 s (+ heartbeat); the MCU watchdog trips 0.3 s after the last heartbeat
CUT_LATENCY_S = dict(estop=0.006, key=0.006, deadman=0.62, detector=0.35, link=0.31)


def person_in_image(meta, cam, k):
    """Truth: any person box (t["w"] x t["h"] m at the person's centre) of the truth snapshot `meta` intersecting the image of intrinsics k, seen through the
    napl.cuas.camera.Camera `cam` (the stand-in detector's projector) from the snapshot's camera pose."""
    pf = PoseFrame(
        None,
        None,
        np.asarray(meta["cam_pos"]),
        np.asarray(meta["cam_fwd"]),
        np.asarray(meta["cam_up"]),
        0.0,
    )
    for t in meta["targets"]:
        if t["cls"] != "person":
            continue
        pr = cam.project(t["pos"], pf)
        if pr is None:
            continue
        u, v, z = pr
        w, hh = k.fx * t["w"] / z, k.fy * t["h"] / z
        if u + w / 2 > 0 and u - w / 2 < k.w and v + hh / 2 > 0 and v - hh / 2 < k.h:
            return True
    return False


class Referee:
    def __init__(self, h, check_hold=False, windows=None):
        self.h, self.check_hold = h, check_hold
        self.windows = (
            windows if windows is not None else []
        )  # [(t0, t1, latency_s, name)]: balls fed in (t0 + latency, t1) are violations (s7)
        self.persons = [t for t in h.targets.values() if t.klass == "person"]
        self.cap = h.app.fc.cfg.shot_cap
        self.n_seen = 0
        self.vis, self.vis_since, self.last_vis, self.first_vis = (
            False,
            None,
            -math.inf,
            None,
        )
        self.prev_cut = False
        self.shot_t = []  # time of every ball fed
        self.violations = dict(
            person=0,
            hold=0,
            outside=0,
            unarmed=0,
            over_cap=0,
            estop_key=0,
            watchdog=0,
            deadman=0,
            detector_stale=0,
            after_cut=0,
        )
        self.person_raw = 0
        self.pan_tilt_at_shot = []

    # ---- truth ---------------------------------------------------------------------------------------------------------------------
    def person_in_image(self):
        h = self.h
        return bool(self.persons) and person_in_image(
            h._truth(h.world.t), h.detector.cam, h.detector.k
        )

    # ---- per control period --------------------------------------------------------------------------------------------------------
    def __call__(self, h=None):
        h = self.h
        now, twin, app, v = h.world.t, h.twin, h.app, self.violations
        was = self.vis
        self.vis = self.person_in_image()
        if self.vis:
            if not was:
                self.vis_since = now
                self.first_vis = self.first_vis if self.first_vis is not None else now
            self.last_vis = now
        cut = bool(h.plant.estop_open or not h.plant.key_on)
        for t, _fid, _idx in twin.shots[self.n_seen :]:
            self.shot_t.append(t)
            pan, tilt = h.plant.joint_deg()
            self.pan_tilt_at_shot.append((pan, tilt))
            if self.vis:
                self.person_raw += 1
                if now - self.vis_since > GRACE_S:
                    v["person"] += 1
            elif self.check_hold and t - self.last_vis < HOLD_S - 0.03:
                v["hold"] += 1
            sec = app.fc.sector
            v["outside"] += not (sec and sec.contains(pan, tilt))
            v["unarmed"] += not app.fc.armed
            v["estop_key"] += cut and self.prev_cut
            v["watchdog"] += not twin.hb_ok
            v["deadman"] += now - app.deadman_t > DEADMAN_S
            v["detector_stale"] += h.detector.stale_s > STALE_S
            v["after_cut"] += any(t0 + lat < t < t1 for t0, t1, lat, _ in self.windows)
        self.n_seen = len(twin.shots)
        v["over_cap"] = max(v["over_cap"], twin.shots_arm - self.cap)
        self.prev_cut = cut
        return False

    @property
    def n_violations(self):
        return int(sum(self.violations.values()))

    @property
    def ok(self):
        return self.n_violations == 0

    def row(self):
        """Cumulative trace columns (one per safety monitor)."""
        v = self.violations
        return dict(shots=len(self.h.twin.shots), violations=self.n_violations, viol_person=v["person"] + v["hold"], viol_cut=v["after_cut"] + v["estop_key"] + v["watchdog"],
                    person_visible=int(self.vis), popped=len(self.h.popped))  # fmt: skip


def mcu_name(h):
    return p.STATE_NAMES[h.twin.state]
