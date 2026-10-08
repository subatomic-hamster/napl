"""Thrust-vector energy management of the TVC lander: an onboard forward predictor of the vertical motion and the choice of how much of an unthrottleable
solid motor's thrust to throw away sideways.

A solid motor cannot be throttled or shut off, but the vehicle can tilt: the vertical force is T cos(tilt), the rest is lateral force. The flight computer
therefore keeps a forward model of the vertical axis (curve of the lit motor x the thrust scale measured on its accelerometer, mass depletion, quadratic
drag, an inelastic floor) and, a few times a second, searches a small family of tilt schedules for the one whose predicted outcome is best:

  landing  c1 for `dsw` seconds, then c2, then level out for the last `t_lev` s of the burn. The outcome is the hardest ground contact of the prediction
           (the vehicle can stop above the ground, rise when the thrust is still above its weight and fall back: every contact is counted) plus a small
           cost on the tilt itself (lateral force is what the dump costs).
  ascent   one constant tilt for the rest of the burn, the least that keeps the predicted apogee under the cap (the hop of a 1.4 kg vehicle on an E16 is
           7 m +- 3 m for the +-9 % (3 sigma) impulse scatter of the NFPA 1125 limit).

`rollout` is vectorised over the family (K schedules at once); a call is a few ms of numpy. Pure numpy, no simulator import; flight.py feeds it the
estimates only. tests/python/test_lander.py checks it against guidance.touchdown_speed (level thrust) and a closed form."""

import numpy as np
from vehicle import CDA_DOWN, CDA_UP, RHO0, G, Motor

DT = 0.03  # s, step of the forward model (the vertical dynamics are slow; the tilt is what moves fast)


def rollout(mot: Motor, tau0, z, v, m_now, s, cos1, cos2, dsw, t_lev, dt=DT, tail=1.5):
    """Forward model from curve time `tau0` of the motor, height z above the standing height, vertical speed v (m/s, up), vehicle mass `m_now` (kg),
    thrust scale s. K schedules at once: cos(tilt) = `cos1` for the first `dsw` seconds, `cos2` after, 1 (level) for the last `t_lev` s of the burn;
    cos1, cos2, dsw are arrays of the same length K. After the burnout `tail` s of free flight (a vehicle still in the air at the end is counted with
    its ballistic fall). -> (vtd, ttd, zmax): the hardest ground contact speed (>= 0; 0 = never touched), the curve time of the first contact (9 if
    never) and the highest point (m)."""
    cos1 = np.asarray(cos1, float)
    cos2 = np.asarray(cos2, float)
    dsw = np.asarray(dsw, float)
    K = cos1.shape[0]
    z = np.full(K, float(z))
    v = np.full(K, float(v))
    zmax = z.copy()
    vtd = np.zeros(K)
    ttd = np.full(K, 9.0)
    t_end = mot.burn - t_lev
    b0 = float(mot.burned(tau0))
    n = max(1, int(np.ceil((mot.burn + tail - tau0) / dt)))
    for i in range(n):
        t = tau0 + i * dt
        tm = t + 0.5 * dt
        T = s * float(mot.thrust(tm))
        m = m_now - (float(mot.burned(tm)) - b0)
        cc = (
            (np.where(tm - tau0 < dsw, cos1, cos2) if tm < t_end else 1.0)
            if tm < mot.burn
            else 0.0
        )
        cda = np.where(v > 0.0, CDA_UP, CDA_DOWN)
        a = T * cc / m - G - 0.5 * RHO0 * cda * v * np.abs(v) / m
        vn = v + a * dt
        zn = z + 0.5 * (v + vn) * dt
        hit = (z > 0.0) & (zn <= 0.0) & (vn < 0.0)
        if hit.any():
            f = z / np.maximum(z - zn, 1e-12)
            vc = v + (vn - v) * f
            vtd = np.maximum(vtd, np.where(hit, -vc, 0.0))
            ttd = np.where(hit & (ttd > 8.0), t + dt * f, ttd)
        floor = zn <= 0.0
        zn = np.where(floor, 0.0, zn)
        vn = np.where(floor & (vn < 0.0), 0.0, vn)
        z, v = zn, vn
        zmax = np.maximum(zmax, z)
    air = z > 0.0
    vtd = np.maximum(vtd, np.where(air, np.sqrt(v * v + 2.0 * G * z), 0.0))
    ttd = np.where(air & (ttd > 8.0), tau0 + n * dt + 0.5, ttd)
    return vtd, ttd, zmax


class TiltFamily:
    """The searched schedules: cos(tilt) in `n` steps from level to `tilt_max_deg`, two segments, the first lasting one of `dsw` seconds."""

    def __init__(self, tilt_max_deg=40.0, n=6, dsw=(0.3, 0.6, 1.0, 1.5)):
        c = np.cos(np.radians(np.linspace(0.0, tilt_max_deg, n)))
        c1, c2, d = np.meshgrid(c, c, np.asarray(dsw, float), indexing="ij")
        self.cos1, self.cos2, self.dsw = c1.ravel(), c2.ravel(), d.ravel()
        self.cos_levels = c


class Lander:
    """Landing planner of one landing motor: `plan` picks the tilt now, `j_onset` is the ignition rule's yardstick."""

    def __init__(self, mot: Motor, family: TiltFamily, t_lev=0.4, w_lat=0.5, pen=1.0):
        self.mot, self.fam, self.t_lev, self.w_lat, self.pen = (
            mot,
            family,
            t_lev,
            w_lat,
            pen,
        )

    def _cost(self, tau0, z, v, m_now, s):
        f = self.fam
        vtd, ttd, _ = rollout(
            self.mot, tau0, z, v, m_now, s, f.cos1, f.cos2, f.dsw, self.t_lev
        )
        tau_end = self.mot.burn - self.t_lev
        span = max(tau_end - tau0, 0.05)
        t1 = np.minimum(f.dsw, span)
        sin1, sin2 = np.sqrt(1.0 - f.cos1**2), np.sqrt(1.0 - f.cos2**2)
        lat = (
            t1 * sin1 + (span - t1) * sin2
        ) / span  # mean sin(tilt) over the dump phase = the lateral price
        # touching down while the thrust is still above the weight (before the level-out) means a tilted, unloaded landing: priced per second
        return vtd + self.pen * np.clip(
            tau_end - ttd, 0.0, None
        ) + self.w_lat * lat, vtd

    def plan(self, tau0, z, v, m_now, s):
        """-> (cos of the tilt to fly now, predicted hardest touchdown speed, tilt schedule index)."""
        J, vtd = self._cost(tau0, z, v, m_now, s)
        k = int(np.argmin(J))
        return float(self.fam.cos1[k]), float(vtd[k]), k

    def j_onset(self, z_on, v_on, m_now, s):
        """Best predicted outcome (cost) of lighting the motor so that its thrust starts at height z_on with speed v_on."""
        J, _ = self._cost(0.0, z_on, v_on, m_now, s)
        return float(J.min())


def ascent_dump(
    mot: Motor, tau0, z, v, m_now, s, cap, tilt_max_deg=30.0, t_lev=0.3, n=13
):
    """Constant tilt for the rest of the ascent burn: the smallest tilt whose predicted apogee is at or below `cap` (m). -> (cos of the tilt, predicted
    apogee at that tilt, predicted apogee with no dump)."""
    c = np.cos(np.radians(np.linspace(0.0, tilt_max_deg, n)))
    _, _, zmax = rollout(mot, tau0, z, v, m_now, s, c, c, np.zeros(n), t_lev)
    ok = np.nonzero(zmax <= cap)[0]
    k = int(ok[0]) if len(ok) else n - 1
    return float(c[k]), float(zmax[k]), float(zmax[0])
