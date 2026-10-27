"""Host side of the cuas2 serial link: lite's SerialGimbal with the cuas2 message table (45-byte STATUS, FLYWHEEL with the differential tail, FIRE with n fixed
at 1) and a view of the launcher / feeder part of STATUS.

    g = Gimbal2(link, clock); g.hello(); g.flywheel(0.8, diff_pm=20); g.fire(); g.launcher_status
"""

from napl.lite.gimbal import LINK_LATENCY_S, SerialGimbal

from . import protocol as p
from .protocol import sat


class Gimbal2(SerialGimbal):
    def __init__(self, link, clock, session=None):
        super().__init__(link, clock, session)
        self.dec = p.Decoder2()
        self.status = p.Status2()

    def flywheel(self, speed, diff_pm=None):
        """speed 0..1 of rpm_max (the firmware clamps it to rpm_cap); diff_pm: bottom-faster differential in permille of the mean (None: the stored cfg 29)."""
        self._send(
            p.Flywheel(
                sat(round(speed * 1000), 0, 1000),
                None
                if diff_pm is None
                else sat(diff_pm, -p.DIFF_MAX_PM, p.DIFF_MAX_PM),
            )
        )

    def fire(self, n=1):
        """One ball per FIRE: n is kept for call compatibility and must be 1."""
        if n != 1:
            raise ValueError("cuas2 feeds one ball per FIRE")
        self.fire_id = (self.fire_id + 1) & 0xFFFF or 1
        self._send(p.Fire(self.fire_id, 1))
        return self.fire_id

    def pump(self):
        """As the base class, with STATUS decoded as Status2."""
        data = self.link.read(0.0)
        if not data:
            return 0
        now = self.clock.now()
        n = 0
        for f in self.dec.feed(data):
            n += 1
            m = f.msg
            if isinstance(m, p.Status2):
                self.status, self.status_t = m, now - LINK_LATENCY_S
                self.ring.append((self.status_t, m))
            elif isinstance(m, p.Event):
                self.events.append((now, m))
            elif isinstance(m, p.Ack):
                self.acks.append((now, m))
            elif isinstance(m, p.HelloAck):
                self.ack = m
            elif isinstance(m, p.CfgVal):
                self.cfg[m.key] = m.value
                if m.key in (p.CFG_PAN_SPR, p.CFG_TILT_SPR) and m.value > 0:
                    self.spr[m.key - p.CFG_PAN_SPR] = m.value
        return n

    @property
    def launcher_status(self):
        s = self.status
        return dict(rpm_top=s.rpm_top, rpm_bot=s.rpm_bot, rpm_set=s.rpm_set, beams=s.beams, gate_state=s.gate_state, ready=s.ready, ready_flags=s.ready_flags,
                    last_v_dmps=s.last_v_dmps, last_dip_pm=s.last_dip_pm, shots_confirmed=s.shots_confirmed, feed_ms=s.feed_ms)  # fmt: skip
