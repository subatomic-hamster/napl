"""App + SimRP2040 rig without the kernel: KinematicPlant joints, a synthetic truth (static targets in B), SimCamera + TruthDetector, one ManualClock."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
from napl.lite import protocol as p
from napl.lite.aim import Geometry
from napl.lite.app import App, AppCfg
from napl.lite.fire import Sector
from napl.lite.gimbal import SerialGimbal
from napl.lite.sim_io import ManualClock, SimCamera, TruthDetector
from napl.lite.sim_mcu import KinematicPlant, LoopbackLink, SimRP2040

DT = 0.005


def balloon(i="b1", pos=(8.0, 1.0, 0.85), cls="balloon", size=0.30):
    return dict(id=i, cls=cls, pos=np.array(pos, float), size=size)


def person(i="p1", pos=(6.0, -2.0, 0.9)):
    return dict(id=i, cls="person", pos=np.array(pos, float), size=0.5, w=0.5, h=1.7)


class AppRig:
    def __init__(
        self,
        targets=None,
        sector=Sector(-60, 60, 0, 60),
        cfg=None,
        detector_kw=None,
        per_track=False,
    ):
        self.clock = ManualClock()
        self.plant = KinematicPlant()
        self.link = LoopbackLink(self.clock)
        self.twin = SimRP2040(self.plant, self.link)
        self.geom = Geometry()
        self.targets = list(targets if targets is not None else [balloon()])
        self.cam = SimCamera(self.clock, self._truth)
        self.det = TruthDetector(self.clock, noise=False, **(detector_kw or {}))
        self.gim = SerialGimbal(self.link, self.clock, session=11)
        cfg = cfg or AppCfg()
        cfg.auto_home = True
        cfg.fire.per_track_auth = per_track
        self.app = App(
            self.cam, self.det, self.gim, self.clock, cfg, geom=self.geom, sector=sector
        )
        self.hold = False
        self._dm = 0.0
        self.twin.step(0.0)

    def _truth(self, t):
        pose = self.geom.pose(*self.plant.joint_deg())
        return dict(
            cam_pos=pose.cam_pos,
            cam_fwd=pose.cam_fwd,
            cam_up=pose.cam_up,
            targets=list(self.targets),
        )

    @property
    def st(self):
        return self.gim.status

    @property
    def shots(self):
        return len(self.twin.shots)

    def run(self, secs):
        for _ in range(int(round(secs / DT))):
            c = self.clock
            c.t += DT
            if self.hold and c.t >= self._dm:
                self.app.deadman()
                self._dm = c.t + 0.2
            self.twin.step(c.t)
            self.app.tick(c.t)

    def ready(self):
        """Link up, IDLE, homed, aimed at the level, dead-man held."""
        self.hold = True
        self.run(5.0)
        assert (
            self.st.state == p.IDLE
            and self.st.flag1(p.F1_PAN_HOMED)
            and self.st.flag1(p.F1_TILT_HOMED)
        ), (p.STATE_NAMES[self.st.state], self.app.log)
        return self

    def arm(self):
        self.app.post("arm")
        self.run(0.05)

    def engage_until_shot(self, limit=10.0):
        self.arm()
        t_end = self.clock.t + limit
        while self.clock.t < t_end and not self.shots:
            self.run(0.1)
        return self.shots
