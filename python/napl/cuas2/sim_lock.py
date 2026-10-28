"""Lockstep harness of the cuas2 scenarios: the cuas-lite lockstep (napl.lite.sim_io.Lockstep: World, NaplPlant, SimCamera, TruthDetector, Referee-compatible)
with the cuas2 weapon behind it.

    sc = build_scene(out, targets, launcher=launcher_cfg(calib))      # lite scene builder + a launcher device that takes the per-shot attrs
    run = Lockstep2(sc, run_dir, targets, sector=SECTOR, true_calib=Calib(...), app_cfg=AppCfg2())
    run.run(40.0)

Per 5 ms control period: twin.step (Sim2RP2040 = lite's safety FSM + the cuas2 feeder FSM, 1 ms ticks), app.tick, world.step. `Weapon2` is the board around the feeder
FSM: wheels (LauncherModel.step_wheels behind the DShot outputs), the hardware limiter, gate, pusher, beams (FeederPlant, constants of MECH / firmware) and the
launcher shot: at the nip crossing LauncherModel.sample_shot() draws the ball (speed from the actual wheel rpm, scatter, spin, d / m / cd, wheel dip) and writes it into
the kernel launcher's per-shot attrs before `fire` (docs/cuas2/spec.md 7.4). The app plans with its own nominal Calib; the plant may use a perturbed one (`true_calib`).

SCENE. The turret is the cuas-lite assembly (CAD, steppers-as-servos, camera pose device): assemblies/cuas2.toml carries no launcher / servo / camera devices yet
(its header: left to integration), and the gimbal numbers of cuas2 are v1's (spec R5). Only the launcher device is configured as the cuas2 one."""

from types import SimpleNamespace

import numpy as np

from napl.lite import sim_io as _io
from napl.lite.sim_mcu import LoopbackLink

from . import protocol as p2
from .app import App2, AppCfg2
from .calib import Calib
from .gimbal import Gimbal2
from .launcher import LauncherModel
from .sim_mcu import (
    GATE_BORE_US,
    GATE_LOAD_US,
    PUSH_EXT_US,
    PUSH_RET_US,
    FeederPlant,
    Sim2RP2040,
)


def launcher_cfg(calib):
    """Launcher-device overrides for sim_io.build_scene: no kernel-side scatter (the Python plant draws every shot), Magnus on, the plant's ball."""
    b, lo = calib.ball, calib.launcher
    return dict(
        v_std=0.0,
        spread_std=0.0,
        muzzle_velocity=30.0,
        pellet_mass=b.m_g / 1000,
        diameter=b.d_mm / 1000,
        cd=b.cd,
        cl_k=lo.cl_k,
        spin_tau=lo.spin_tau_s,
    )


class GunRig:
    """FeederPlant's `rig`: writes the sampled Shot into the kernel launcher and fires it."""

    def __init__(self, world):
        self.w = world

    def fire(self, shot):
        for k, v in shot.attrs().items():
            self.w.sim.set(self.w.gun, k, v)
        self.w.sim.set(self.w.gun, "fire", 1.0)


class Weapon2:
    """The `weapon=` of Sim2RP2040: inputs(ms) -> sampled sensors, step(out, ms, rail) -> advance one ms."""

    def __init__(self, calib, rig=None, seed=1):
        rng = np.random.default_rng(seed)
        self.model = LauncherModel(calib, rng)
        self.feeder = FeederPlant(self.model, rig, rng)
        self.cfg = p2.CFG_DEFAULTS2
        self.rail = False

    def inputs(self, ms):
        b = self.feeder.beams(ms / 1000.0, 0.001)
        rt, rb = self.model.telemetry() if self.rail else (None, None)
        pulse = (
            self.feeder.pulses[-1]
            if (b["beam_muzzle"] and self.feeder.pulses)
            else None
        )
        return dict(
            rpm_top=None if rt is None else int(round(rt)),
            rpm_bot=None if rb is None else int(round(rb)),
            muzzle_dt_us=int((pulse[1] - pulse[0]) * 1e6) if pulse else 0,
            **b,
        )

    def step(self, out, ms, rail=True):
        self.rail = rail
        dt, db, fr = (
            (out.dshot_top, out.dshot_bot, out.feed_req) if rail else (0, 0, False)
        )
        c = self.cfg
        o = SimpleNamespace(dshot_top=dt, dshot_bot=db, feed_req=fr,
                            gate_us=GATE_BORE_US if out.gate_us == c[p2.CFG_GATE_BORE_US] else GATE_LOAD_US,
                            push_us=PUSH_EXT_US if out.push_us == c[p2.CFG_PUSHER_EXT_US] else PUSH_RET_US)  # fmt: skip
        if (
            not rail
        ):  # the rail is cut: the limiter's supply to the pusher servo dies with it
            self.feeder.powered_until = min(self.feeder.powered_until, ms / 1000.0)
        loaded = self.feeder.step(ms / 1000.0, 0.001, o)
        self.model.step_wheels(dt, db, 0.001, load=1.0 if loaded else 0.0)


class Lockstep2(_io.Lockstep):
    """napl.lite.sim_io.Lockstep with Sim2RP2040 + Weapon2 + Gimbal2 + App2. `true_calib` is what the plant does, `app_calib` what the app believes (default: the same)."""

    def __init__(
        self,
        manifest,
        run_dir,
        targets,
        app_cfg=None,
        sector=None,
        seed=0,
        detector_kw=None,
        drop_link=None,
        camera_kw=None,
        true_calib=None,
        app_calib=None,
        mcu_cfg=None,
    ):
        from napl.cuas.world import World

        self.targets = {t.id: t for t in targets}
        self.movers = [t for t in targets if isinstance(t, _io.Mover)]
        self.world = w = World(manifest["model"], run_dir, seed=seed or 1, ctl_ticks=5)
        self._drive(w.t)
        self.base = np.array(w.man["vehicles"]["turret"]["pos"], float)
        self.clock = _io.SimClock(w)
        self.plant = _io.NaplPlant(w)
        self.link = LoopbackLink(self.clock)
        self.weapon = Weapon2(true_calib or Calib(), GunRig(w), seed=seed + 1)
        self.twin = Sim2RP2040(
            self.plant,
            self.link,
            weapon=self.weapon,
            step_loss=False,
            status_from_joint=True,
            home_off_deg=(-1.0, -1.0),
        )
        # rpm window 2 % (the CFG_SET maximum) instead of the spec default 1 %: with the plant's 3 permille telemetry noise and the PI wobble the 1 % window is held for
        # settle_ms in about 1 of 40 samples, so READY blinks (tests/cuas2/test_plant.py uses 20 for the same reason). Bench item: measure the noise, then set the default.
        for k, v in {p2.CFG_RPM_WINDOW_PM: 20, **(mcu_cfg or {})}.items():
            self.twin.cfg[k] = self.twin.flash[k] = v
        self.geom = _io.measure_geometry(w, self.plant)
        self.twin.step(w.t)
        self.camera = _io.SimCamera(
            self.clock, self._truth, seed=seed, **(camera_kw or {})
        )
        self.camera.next_t = w.t
        self.detector = _io.TruthDetector(self.clock, seed=seed, **(detector_kw or {}))
        self.gimbal = Gimbal2(self.link, self.clock, session=0x5EED0000 + seed)
        cfg = app_cfg or AppCfg2()
        if app_cfg is None:
            cfg.fire.vmax, cfg.fire.amax = (
                (100.0, 70.0),
                (600.0, 500.0),
            )  # as the cuas-lite harness: slews the scenarios were tuned on
        cfg.auto_home = True
        cfg.fire.seed = seed
        self.app = App2(
            self.camera,
            self.detector,
            self.gimbal,
            self.clock,
            cfg,
            geom=self.geom,
            sector=sector,
            model=LauncherModel(app_calib or Calib()),
        )
        # exact (sim time) arm sessions and the session of every FIRE sent: the Referee judges a ball against the session of its FIRE
        self.arm_iv = []  # [[t_arm, t_disarm or inf]]
        self.fire_sess = {}  # fire_id -> index of the arm session the FIRE was sent in
        fc, w_, gim = self.app.fc, self.world, self.gimbal
        arm0, disarm0, fire0 = fc.arm, fc.disarm, gim.fire

        def arm(now):
            if fc.armed and self.arm_iv:
                self.arm_iv[-1][1] = min(self.arm_iv[-1][1], w_.t)
            arm0(now)
            self.arm_iv.append([w_.t, float("inf")])

        def disarm():
            if fc.armed and self.arm_iv:
                self.arm_iv[-1][1] = min(self.arm_iv[-1][1], w_.t)
            disarm0()

        def fire(n=1):
            fid = fire0(n)
            self.fire_sess.setdefault(fid, len(self.arm_iv) - 1)
            return fid

        fc.arm, fc.disarm, gim.fire = arm, disarm, fire
        self.script, self.hold_deadman, self._next_dm = [], False, 0.0
        self.popped, self.fire_log, self.hits, self.hit_log, self._ev_seen = (
            {},
            [],
            {},
            [],
            0,
        )
