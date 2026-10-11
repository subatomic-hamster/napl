"""Test rig: SimRP2040 + KinematicPlant + LoopbackLink + SerialGimbal on a ManualClock (no kernel, no sleeping)."""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from napl.lite import protocol as p
from napl.lite.gimbal import SerialGimbal
from napl.lite.sim_io import ManualClock
from napl.lite.sim_mcu import KinematicPlant, LoopbackLink, SimRP2040

DT = 0.005


def make_mcu(plant, link, **kw):
    """SimRP2040, or the firmware's C core (fwmcu.FwMcu) when `pytest --fw-core=PATH` / NAPL_FW_CORE selects it."""
    if os.environ.get("NAPL_FW_CORE"):
        from fwmcu import FwMcu

        return FwMcu(plant, link, **kw)
    kw.setdefault(
        "home_off_deg", (-1.0, -1.0)
    )  # a calibrated unit: home-frame 0 at the hall window centre (the twin's own default is 0, the firmware's)
    return SimRP2040(plant, link, **kw)


class Rig:
    def __init__(self, start=(0.0, 0.0), **twin_kw):
        self.clock = ManualClock()
        self.plant = KinematicPlant(
            *start
        )  # joint angles at power-up (the steppers know nothing about them until homed)
        self.link = LoopbackLink(self.clock)
        self.twin = make_mcu(self.plant, self.link, **twin_kw)
        self.gim = SerialGimbal(self.link, self.clock, session=7)
        self.deadman, self.inhibit, self.hb = True, False, True
        self.fw = 0.0
        self._t_hb = self._t_fw = -1.0
        self.twin.step(0.0)
        self.gim.hello()

    @property
    def st(self):
        return self.gim.status

    def run(self, secs):
        for _ in range(int(round(secs / DT))):
            c = self.clock
            c.t += DT
            self.twin.step(c.t)
            self.gim.pump()
            if self.hb and c.t - self._t_hb >= 0.05 - 1e-9:
                self._t_hb = c.t
                self.gim.heartbeat(self.deadman, self.inhibit)
            if self.fw and c.t - self._t_fw >= 0.1 - 1e-9:
                self._t_fw = c.t
                self.gim.flywheel(self.fw)

    def acks(self, cmd_id):
        return [a for _, a in self.gim.acks if a.cmd_id == cmd_id]

    def up(self, homed=True):
        """HELLO, 1.2 s of heartbeats -> IDLE, then home both axes."""
        self.run(1.3)
        assert self.st.state == p.IDLE, p.STATE_NAMES[self.st.state]
        if homed:
            self.gim.home()
            self.run(3.0)
            assert self.st.flag1(p.F1_PAN_HOMED) and self.st.flag1(p.F1_TILT_HOMED)
            self.gim.aim(
                0, 5.0
            )  # above fire_tilt_min (homing leaves the tilt 3 deg below level)
            self.run(1.0)
        return self

    def arm(self, cap=5, timeout=60):
        self.gim.arm(cap, timeout)
        self.run(0.05)

    def engage(self, duty=0.8, spin=1.2):
        """ARM, FLYWHEEL, wait for spin-up: the MCU is ENGAGING and may accept a FIRE."""
        self.arm()
        self.fw = duty
        self.run(spin)
        assert self.st.state == p.ENGAGING, p.STATE_NAMES[self.st.state]
        return self

    def fire(self, n=1, wait=0.05):
        fid = self.gim.fire(n)
        self.run(wait)
        return fid, [a for a in self.acks(p.FIRE) if a.cmd_seq is not None][-1]
