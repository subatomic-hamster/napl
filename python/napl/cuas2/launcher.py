"""LauncherModel: the dual-flywheel launcher physics of docs/cuas2/spec.md 7.1-7.2, parameterised from calib.toml.

    m = LauncherModel(calib.load(), rng=np.random.default_rng(1))
    m.step_wheels(dshot_top, dshot_bot)          # 1 ms plant step, called by the twin behind its DShot outputs
    m.telemetry()                                # (rpm_top, rpm_bot) with noise / dropout
    shot = m.sample_shot()                       # at the nip crossing: draws the shot AND applies the energy dip to the wheels
    b = m.sample_shots(500, rng, setpoint=17500) # fire control: the same sampler, no wheel state change -> ShotBatch of arrays

Conventions (head frame: x forward, y left, z up). Spin `omega` is about +y (rad/s): bottom wheel faster -> v_t - v_b < 0 -> omega < 0 =
backspin (a ball moving +x with omega_y < 0 gets lift +z). `dir_err` = (horizontal left+, vertical up+) rad. The kernel launcher's site has
x = up, z = barrel, so its dir_offset_x is the vertical error and dir_offset_y the NEGATIVE of the horizontal one (Shot.attrs()).
"""

import math
from typing import NamedTuple

import numpy as np

from .calib import Calib


class ShotBatch(NamedTuple):
    v0: np.ndarray  # (n,) m/s
    dir_err: np.ndarray  # (n, 2) rad (horizontal left+, vertical up+)
    omega: np.ndarray  # (n,) rad/s about +y
    d: np.ndarray  # (n,) m
    m: np.ndarray  # (n,) kg
    cd: np.ndarray  # (n,)
    dip: np.ndarray  # (n,) mean fractional rpm drop of the wheels this shot causes


class Shot(NamedTuple):
    v0: float
    dir_err: tuple
    omega: float
    d: float
    m: float
    cd: float
    dip: float

    def attrs(self):
        """The kernel launcher's per-shot inputs (muzzle_velocity, spin_rps, dir_offset_x, dir_offset_y); the device must have cl_k / spin_tau."""
        return dict(
            muzzle_velocity=self.v0,
            spin_rps=self.omega / (2 * math.pi),
            dir_offset_x=self.dir_err[1],
            dir_offset_y=-self.dir_err[0],
        )


def dshot_to_u(dshot):
    """DShot 0..2047 (0 = disarmed, 48..2047 = throttle) -> [0, 1]."""
    return 0.0 if dshot < 48 else min((dshot - 48) / 1952.0, 1.0)


class LauncherModel:
    def __init__(self, calib=None, rng=None):
        self.c = calib or Calib()
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.omega = np.zeros(2)  # rad/s (top, bottom)
        self._tel = (0.0, 0.0)

    # ---- geometry / set points ---------------------------------------------------------------------------------------------------------
    @property
    def r_wheel(self):
        return self.c.launcher.wheel_d_mm / 2000.0

    @property
    def rpm_cap(self):
        return self.c.rpm_cap

    def surface_speeds(self, rpm_top, rpm_bot):
        k = math.pi * self.c.launcher.wheel_d_mm / 1000.0 / 60.0
        return k * np.asarray(rpm_top, float), k * np.asarray(rpm_bot, float)

    def split(self, rpm_mean, diff_pm=None):
        """Set point -> (rpm_top, rpm_bot): top = mean (1 - diff/2), bottom = mean (1 + diff/2) (spec 4.4). Both clamped to rpm_cap (firmware)."""
        d = (self.c.launcher.diff_pm if diff_pm is None else diff_pm) / 1000.0
        cap = self.rpm_cap
        return min(rpm_mean * (1 - d / 2), cap), min(rpm_mean * (1 + d / 2), cap)

    def _rpms(self, setpoint, diff_pm):
        if setpoint is None:
            setpoint = self.c.launcher.rpm_max
        if isinstance(setpoint, (tuple, list)):
            return float(setpoint[0]), float(setpoint[1])
        return self.split(float(setpoint), diff_pm)

    def mean_v0(self, setpoint=None, diff_pm=None, with_dip=False):
        """k_slip * (v_t + v_b) / 2 for a set point (rpm mean, or (rpm_top, rpm_bot)); with_dip multiplies the mean in-contact dip of 7.2."""
        rt, rb = self._rpms(setpoint, diff_pm)
        v = self._v_mean(rt, rb) * self.c.k_slip_at((rt + rb) / 2)
        return v * (1 - self.dip_fraction(rt, rb) / 2) if with_dip else v

    def _v_mean(self, rt, rb):
        vt, vb = self.surface_speeds(rt, rb)
        return float((vt + vb) / 2)

    def nominal_v(self, rpm_top, rpm_bot):
        """Mean surface speed times (1 - dip/2): the speed at k_slip = 1 including the model's own in-contact dip (the bench fit divides it out)."""
        v_mean = self._v_mean(rpm_top, rpm_bot)
        return v_mean * (1 - self.dip_fraction(rpm_top, rpm_bot) / 2)

    def spin_rad_s(self, rpm_top, rpm_bot, k_spin=None):
        vt, vb = self.surface_speeds(rpm_top, rpm_bot)
        ks = self.c.launcher.k_spin if k_spin is None else k_spin
        return float(ks * (vt - vb) / (2 * self.c.ball.d_mm / 2000.0))

    # ---- energy dip --------------------------------------------------------------------------------------------------------------------
    def _dips(self, rt, rb, v_ball, m):
        """Per-wheel fractional rpm drop when the wheels give (0.5 m v^2 / eta) split by surface speed (7.2). v_ball is the ball speed BEFORE the dip."""
        w = self.c.wheel
        vt, vb = self.surface_speeds(rt, rb)
        e_ball = 0.5 * m * np.asarray(v_ball) ** 2 / w.eta_transfer
        s = np.maximum(vt + vb, 1e-9)
        out = []
        for v_i, rpm in ((vt, rt), (vb, rb)):
            e_w = 0.5 * w.J_kgm2 * (np.asarray(rpm) * 2 * math.pi / 60) ** 2
            frac = np.clip(e_ball * v_i / s / np.maximum(e_w, 1e-12), 0.0, 1.0)
            out.append(1 - np.sqrt(1 - frac))
        return out

    def dip_fraction(self, rpm_top, rpm_bot):
        """Mean fractional rpm drop of the two wheels for a mean ball (k_slip nominal speed). For the fitter / reports."""
        v = self._v_mean(rpm_top, rpm_bot) * self.c.k_slip_at((rpm_top + rpm_bot) / 2)
        dt, db = self._dips(rpm_top, rpm_bot, v, self.c.ball.m_g / 1000.0)
        return float((dt + db) / 2)

    # ---- sampling ----------------------------------------------------------------------------------------------------------------------
    def sample_shots(self, n, rng=None, setpoint=None, diff_pm=None):
        """n shots at a steady set point (rpm mean or (top, bottom)); does not touch the wheel state. Same draws as sample_shot."""
        rng = rng if rng is not None else self.rng
        c, lo = self.c, self.c.launcher
        rt, rb = self._rpms(setpoint, diff_pm)
        d = np.maximum(rng.normal(c.ball.d_mm, c.ball.d_sigma_mm, n), 1.0) / 1000.0
        m = np.maximum(rng.normal(c.ball.m_g, c.ball.m_sigma_g, n), 0.1) / 1000.0
        cd = np.maximum(rng.normal(c.ball.cd, c.ball.cd_sigma, n), 0.05)
        v_nom = self._v_mean(rt, rb) * c.k_slip_at((rt + rb) / 2)
        dt, db = self._dips(rt, rb, np.full(n, v_nom), m)
        dip = (dt + db) / 2
        v0 = v_nom * (1 - dip / 2) * (
            1 + rng.normal(0.0, lo.sigma_v_rel, n)
        ) + rng.normal(0.0, lo.sigma_v_floor_mps, n)
        v0 = np.clip(v0, 0.0, lo.v_cap_mps)
        err = np.empty((n, 2))
        for a in (0, 1):
            err[:, a] = (lo.bias_mrad[a] + rng.normal(0.0, lo.spread_mrad[a], n)) * 1e-3
        om = np.full(n, self.spin_rad_s(rt, rb))
        return ShotBatch(v0, err, om, d, m, cd, dip)

    def sample_shot(self, rng=None):
        """One shot from the CURRENT wheel speeds at the nip crossing; the wheels lose the ball's energy (omega' = sqrt(omega^2 - 2 dE / J))."""
        rng = rng if rng is not None else self.rng
        rt, rb = (float(w) * 60 / (2 * math.pi) for w in self.omega)
        b = self.sample_shots(1, rng, (rt, rb))
        w = self.c.wheel
        v_nom = self._v_mean(rt, rb) * self.c.k_slip_at((rt + rb) / 2)
        e_ball = 0.5 * float(b.m[0]) * v_nom**2 / w.eta_transfer
        vt, vb = self.surface_speeds(rt, rb)
        for i, v_i in enumerate((vt, vb)):
            e_i = e_ball * float(v_i) / max(float(vt + vb), 1e-9)
            self.omega[i] = math.sqrt(max(self.omega[i] ** 2 - 2 * e_i / w.J_kgm2, 0.0))
        return Shot(
            float(b.v0[0]),
            (float(b.dir_err[0, 0]), float(b.dir_err[0, 1])),
            float(b.omega[0]),
            float(b.d[0]),
            float(b.m[0]),
            float(b.cd[0]),
            float(b.dip[0]),
        )

    # ---- wheel dynamics (7.1) ----------------------------------------------------------------------------------------------------------
    def step_wheels(self, dshot_top, dshot_bot, dt=0.001, load=0.0):
        """d omega/dt = clamp((omega_t - omega) / tau, -a_brake, P_max / (J omega + eps)), omega_t = 2 pi/60 kv V u (1 - droop load)."""
        w = self.c.wheel
        for i, ds in enumerate((dshot_top, dshot_bot)):
            u = dshot_to_u(ds)
            target = 2 * math.pi / 60 * w.kv * w.v_bus * u * (1 - w.droop * load)
            acc = (target - self.omega[i]) / w.tau_motor_s
            acc = min(
                max(acc, -w.a_brake_rads2),
                w.p_max_w / (w.J_kgm2 * self.omega[i] + 1e-9),
            )
            self.omega[i] = max(self.omega[i] + acc * dt, 0.0)

    @property
    def rpm(self):
        return tuple(float(w) * 60 / (2 * math.pi) for w in self.omega)

    def telemetry(self):
        """(rpm_top, rpm_bot) with N(0, rpm_sigma_pm/1000) noise; a 0.1 % dropout returns the previous sample (stale)."""
        if self.rng.random() < 0.001:
            return self._tel
        s = self.c.wheel.rpm_sigma_pm / 1000.0
        self._tel = tuple(r * (1 + self.rng.normal(0.0, s)) for r in self.rpm)
        return self._tel
