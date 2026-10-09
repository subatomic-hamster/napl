"""Simulated AIS (Automatic Identification System) between the traffic vessels and the ASV.

Each traffic vessel is a Class A transponder fed by its OWN GNSS device (the same noisy, latent, correlated fixes the ASV's receiver has; nothing from the
simulator's truth): it transmits a position report every `interval` seconds, which the ASV receives `latency` seconds later, or not at all (`loss`).
Class A reporting interval (ITU-R M.1371): 10 s for a vessel under way at up to 14 kn on a steady course, 3 1/3 s while it is changing course. The report
quantisation follows the message: position 1e-4 arc-minute (not binding here), speed over ground 0.1 kn, course over ground 0.1 degree.
Not modelled: slot collisions and the reception range (the traffic is always within a few hundred metres); the ~1 s the self-organising TDMA adds is part of `latency`.
Scale caveat: a Class A transponder is for ships; fitting one to a 1.2 m craft is an idealisation of "the traffic broadcasts its own GNSS track".
"""

import math
from dataclasses import dataclass

import numpy as np

from .geo import KN, enu_from_latlon, wrap

INTERVAL_STEADY = 10.0  # s
INTERVAL_TURNING = 10.0 / 3.0  # s, Class A while changing course
TURN_RATE = math.radians(
    2.0
)  # rad/s of COG change counted as "changing course" (Class A: any rate of turn > 0.5 deg per 6 s; this is a bolder figure for a small craft)


@dataclass
class Report:
    mmsi: int
    t_tx: float  # when it was sent (sim s)
    lat: float
    lon: float
    sog: float  # m/s, quantised to 0.1 kn
    cog: float  # rad, math (counter-clockwise from East), quantised to 0.1 degree


class Transponder:
    """One vessel's AIS transmitter: feed it the GNSS fixes with `fix(t, lat, lon, vel_ned)`; it decides when to send."""

    def __init__(self, mmsi, lat0, lon0, t_first=1.0):
        self.mmsi = mmsi
        self.lat0, self.lon0 = lat0, lon0
        self.last_tx = t_first - INTERVAL_STEADY  # the first report goes out at t_first
        self.hist = []  # (t, cog) of the last ~2.5 s of fixes: the rate of turn is a change over about 2 s (single 5 Hz fixes are too noisy: 0.05 m/s at 1.5 m/s is 2 degrees)
        self.rot = 0.0
        self.fixes = None

    def fix(self, t, lat, lon, vel_ned):
        vn, ve = float(vel_ned[0]), float(vel_ned[1])
        cog = math.atan2(vn, ve)
        if not self.hist or t > self.hist[-1][0]:
            self.hist.append((t, cog))
        while len(self.hist) > 1 and t - self.hist[1][0] >= 1.9:
            self.hist.pop(0)
        t0, c0 = self.hist[0]
        self.rot = abs(wrap(cog - c0)) / (t - t0) if t - t0 >= 1.5 else 0.0
        self.fixes = (lat, lon, vn, ve)

    def poll(self, t):
        """-> Report or None: called every tick or every control step. The interval in force is re-read at every call, so a vessel that starts to turn reports
        3 1/3 s after its last report, not 10 s."""
        if (
            self.fixes is None
            or t
            < self.last_tx
            + (INTERVAL_TURNING if self.rot > TURN_RATE else INTERVAL_STEADY)
            - 1e-9
        ):
            return None
        lat, lon, vn, ve = self.fixes
        sog = round(math.hypot(vn, ve) / KN, 1) * KN
        cog = math.radians(round(math.degrees(math.atan2(vn, ve)) % 360.0 * 10) / 10)
        self.last_tx = t
        return Report(self.mmsi, t, lat, lon, sog, wrap(cog))


class Channel:
    """The VHF link to the ASV: delays every report by `latency` and loses a fraction `loss` of them (seeded)."""

    def __init__(self, seed, lat0, lon0, latency=1.0, loss=0.02):
        self.lat0, self.lon0 = lat0, lon0
        self.rng = np.random.default_rng([int(seed), 6161])
        self.latency, self.loss = latency, loss
        self.q = []

    def send(self, rep: Report):
        if rep is not None and self.rng.random() >= self.loss:
            self.q.append((rep.t_tx + self.latency, rep))

    def receive(self, t):
        """Reports that have arrived by time t, as dicts in the ASV's local frame: id, t_tx, x, y (E, N m), vx, vy (m/s)."""
        out = [r for ta, r in self.q if ta <= t]
        self.q = [(ta, r) for ta, r in self.q if ta > t]
        res = []
        for r in out:
            x, y = enu_from_latlon(r.lat, r.lon, self.lat0, self.lon0)
            res.append(
                dict(
                    id=r.mmsi,
                    t_tx=r.t_tx,
                    x=x,
                    y=y,
                    vx=r.sog * math.cos(r.cog),
                    vy=r.sog * math.sin(r.cog),
                )
            )
        return res
