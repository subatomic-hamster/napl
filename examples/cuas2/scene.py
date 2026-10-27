"""Scenes of the cuas2 lockstep scenarios: the SAME scenario kinds as examples/cuas-lite/scene.py (static / drifting balloons, hanging targets, quad crossing at 3 and
8 m/s, mix, person walk, person beside a balloon, the s7 fault scenarios, the mc_* single-target episodes); no new threat profiles. This module re-exports the
cuas-lite scene module and replaces make(): the dispersion knobs of the Monte Carlo (v0_err, spread, cd) become a perturbed *plant* Calib instead of kernel-side
scatter, because the cuas2 launcher draws every shot in Python (LauncherModel.sample_shot, spin and wheel dip included). The app keeps the nominal Calib.

    sp = make("mc_quad", seed, R=9.0, speed=5.0, wind=(0.4, -0.2, 0), v0_err=-1.0)     # Spec; sp.true_calib is the plant's Calib, sp.launcher the device config
"""

import dataclasses
import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))


def load(name, rel):
    """Import examples/<rel> under a private module name (the cuas-lite and cuas2 directories both have scene.py / run.py / mc.py)."""
    spec = importlib.util.spec_from_file_location(name, REPO / "examples" / rel)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


lite = load("cuas_lite_scene", "cuas-lite/scene.py")
globals().update({k: v for k, v in vars(lite).items() if not k.startswith("__")})

from napl.cuas2.calib import Calib
from napl.cuas2.sim_lock import launcher_cfg


def true_calib(v0_err=0.0, spread=0.010, cd=0.47, base=None):
    """The plant's Calib for a Monte Carlo episode: mean muzzle speed shifted by v0_err m/s (scaled k_slip), direction scatter `spread` rad per axis, ball Cd."""
    c = base or Calib()
    lo = dataclasses.replace(
        c.launcher,
        k_slip=c.launcher.k_slip * (1.0 + v0_err / 30.0),
        spread_mrad=(spread * 1e3, spread * 1e3),
    )
    return dataclasses.replace(c, launcher=lo, ball=dataclasses.replace(c.ball, cd=cd))


def make(name, seed=1, **kw):
    """lite make() plus .true_calib / .launcher for the cuas2 plant. Keywords as lite's (R, speed, t_walk, wind, v0_err, spread, cd, miss_scale, px_sigma, ...)."""
    v0_err, spread, cd = (
        kw.get("v0_err", 0.0),
        kw.get("spread", 0.010),
        kw.get("cd", 0.47),
    )
    sp = lite.make(name, seed, **kw)
    sp.true_calib = true_calib(v0_err, spread, cd)
    sp.launcher = launcher_cfg(sp.true_calib)
    return sp
