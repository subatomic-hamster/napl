"""cuas2 fire control (napl.cuas2.app.FireControl2) without the kernel: person block, the Ph gate, shoot-look-shoot spacing, ARM / dead-man / READY. Every
test that shows a shot is blocked also shows the same scene fires once the single blocking condition is removed (negative control)."""

import numpy as np
import pytest
from napl.cuas2 import protocol as p2
from napl.cuas2.app import T_EXIT, FireCfg2, FireControl2, Tgt, hit_probability, lead2
from napl.cuas2.calib import Calib
from napl.cuas2.launcher import LauncherModel
from napl.lite import aim as aimlib
from napl.lite import protocol as p
from napl.lite.fire import Sector, View

NOW = 20.0


class FakeTrack:
    """Stands in for napl.track.Track: the position block of the mean/cov propagated linearly (constant velocity), sigma_v 0.3 m/s."""

    def __init__(self, mean, cov):
        self.mean, self.cov = mean, cov

    def predict_to(self, t):
        dt = max(t - NOW, 0.0)
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        return F @ self.mean, F @ self.cov @ F.T


def tgt(pos=(8.0, 0.0, 3.2), vel=(0, 0, 0), sig=0.05, cls="balloon", tid=1):
    mean = np.array([*pos, *vel], float)
    cov = np.diag(
        [sig**2] * 3 + [0.03**2] * 3
    )  # a settled track: velocity known to 3 cm/s
    return Tgt(
        tid,
        NOW,
        mean,
        cov,
        FakeTrack(mean, cov),
        cls,
        0.9,
        (600, 300, 640, 340),
        True,
        0,
        float(np.linalg.norm(mean[:3])),
    )


def status(state=p.ENGAGING, ready=True, key=True, estop=False):
    return p2.Status2(
        state=state,
        flags1=(p.F1_KEY if key else 0) | (p.F1_ESTOP if estop else 0),
        ready_flags=p2.RDY_READY if ready else 0,
    )


def view(
    tracks,
    pan=0.0,
    tilt=0.0,
    now=NOW,
    persons=(),
    person_t=-1e9,
    deadman_t=None,
    st=None,
    wind=(0, 0, 0),
):
    return View(
        now,
        st or status(),
        True,
        0.02,
        0.02,
        list(persons),
        person_t,
        tracks,
        now,
        pan,
        tilt,
        now - 0.1 if deadman_t is None else deadman_t,
        wind,
        True,
    )


def make_fc(**cfg):
    fc = FireControl2(
        FireCfg2(**cfg),
        aimlib.Geometry(),
        Sector(-60, 60, 0, 60),
        model=LauncherModel(Calib()),
    )
    fc.arm(NOW - 5.0)
    return fc


def aligned(fc, t):
    """A view whose head already points at the solution of target `t` (one step with the head at 0/0 gives the aim)."""
    d = fc.step(NOW, view([t]))
    assert d.aim is not None and d.solution is not None
    return view([t], pan=d.aim[0], tilt=d.aim[1])


def test_fires_when_every_gate_passes():
    fc = make_fc()
    t = tgt()
    d = fc.step(NOW, aligned(fc, t))
    assert d.fire == 1 and d.ph is not None and d.ph >= fc.cfg.ph_tau, (d.reasons, d.ph)
    assert (
        0.5 < d.ph <= 1.0
    )  # 8 m, 10 mrad scatter (8 cm), 5 cm track error, 16 cm radius


def test_person_blocks_fire_always():
    # a person detection within the hold: no FIRE however good Ph is ...
    fc = make_fc()
    t = tgt()
    v = aligned(fc, t)
    for dt in (0.0, 0.5, 0.95):
        v2 = view(
            [t],
            v.pan,
            v.tilt,
            now=NOW + dt,
            person_t=NOW + dt - 0.01,
            persons=[(0.8, (100, 100, 200, 400))],
        )
        d = fc.step(NOW + dt, v2)
        assert d.fire == 0 and "person" in d.reasons and d.inhibit
    # ... a person box over the target blocks that target even after the hold ran out
    far = view(
        [t],
        v.pan,
        v.tilt,
        now=NOW + 5,
        persons=[(0.8, (590, 290, 650, 500))],
        person_t=NOW + 5 - 2.0,
    )
    d = fc.step(NOW + 5, far)
    assert d.fire == 0 and "no_target" in d.reasons
    # negative control: the same scene without the person fires
    fc2 = make_fc()
    assert fc2.step(NOW, aligned(fc2, t)).fire == 1


def test_ph_gate_blocks_a_poor_track_and_tau_is_the_threshold():
    t = tgt(
        sig=1.5
    )  # 1.5 m position uncertainty at 8 m: the 16 cm ball path misses most samples
    fc = make_fc()
    d = fc.step(NOW, aligned(fc, t))
    assert d.fire == 0 and "ph" in d.reasons and d.ph < 0.3, (d.reasons, d.ph)
    fc0 = make_fc(ph_tau=0.0)  # negative control: gate open -> the same scene fires
    assert fc0.step(NOW, aligned(fc0, t)).fire == 1
    # Ph rises as the track tightens
    phs = []
    for s in (1.5, 0.3, 0.05):
        f = make_fc()
        phs.append(f.step(NOW, aligned(f, tgt(sig=s))).ph)
    assert phs[0] < phs[1] < phs[2], phs


def test_ph_matches_the_geometry():
    """No dispersion at all and the nominal aim: Ph = 1; the target 1 m off the line: Ph = 0."""
    import dataclasses

    cal = Calib()
    cal = dataclasses.replace(cal, launcher=dataclasses.replace(cal.launcher, sigma_v_rel=0.0, sigma_v_floor_mps=0.0, spread_mrad=(0.0, 0.0)),
                              ball=dataclasses.replace(cal.ball, d_sigma_mm=0.0, m_sigma_g=0.0, cd_sigma=0.0))  # fmt: skip
    model = LauncherModel(cal)
    geom = aimlib.Geometry()
    t = tgt(sig=1e-3)
    sp = model.c.launcher.rpm_max
    sol = lead2(geom, model, sp, t, 0.0, 0.0, 0.0)
    pose = geom.pose(sol["pan"], sol["tilt"])
    args = (
        model,
        sp,
        pose.muzzle,
        pose.muzzle_dir,
        t,
        NOW + sol["t_flight"],
        sol["t_flight"],
        0.16,
        200,
        np.random.default_rng(1),
    )
    assert hit_probability(*args)[0] == 1.0
    off = tgt(pos=(8.0, 1.0, 3.2), sig=1e-3)
    assert (
        hit_probability(
            model,
            sp,
            pose.muzzle,
            pose.muzzle_dir,
            off,
            NOW + sol["t_flight"],
            sol["t_flight"],
            0.16,
            200,
            np.random.default_rng(1),
        )[0]
        == 0.0
    )


def test_shoot_look_shoot_spacing():
    fc = make_fc()
    t = tgt()
    v = aligned(fc, t)
    d = fc.step(NOW, v)
    assert d.fire == 1
    fc.fire_sent(NOW, 1)
    fc.on_shot(NOW + 0.01)  # EVENT 2: the ball is fed
    tf = fc.last_tf
    assert tf > 0.1
    t_ok = NOW + 0.01 + T_EXIT + tf + fc.cfg.t_obs
    assert fc.next_ok[t.id] == pytest.approx(t_ok)
    # the MCU is READY again after 0.5 s, but the first ball has not been observed yet
    mid = NOW + 0.55
    assert mid < t_ok
    d = fc.step(mid, view([t], v.pan, v.tilt, now=mid, deadman_t=mid - 0.1))
    assert d.fire == 0 and "shoot_look_shoot" in d.reasons
    # after the outcome window the next FIRE is allowed (negative control)
    late = t_ok + 0.02
    d = fc.step(late, view([t], v.pan, v.tilt, now=late, deadman_t=late - 0.1))
    assert d.fire == 1, d.reasons


def test_arm_deadman_ready_and_cap():
    t = tgt()
    fc = make_fc()
    v = aligned(fc, t)
    assert fc.step(NOW, v).fire == 1
    # no operator ARM
    fc_off = make_fc()
    v_off = aligned(fc_off, t)
    fc_off.disarm()
    d = fc_off.step(NOW, v_off)
    assert d.fire == 0 and "not_armed" in d.reasons and d.flywheel == 0.0
    # stale dead-man: the flywheel is released too
    fc_dm = make_fc()
    d = fc_dm.step(NOW, view([t], v.pan, v.tilt, deadman_t=NOW - 2.0))
    assert d.fire == 0 and "deadman" in d.reasons and d.flywheel == 0.0
    # the MCU is not READY (wheels out of window, no ball staged, interval): no FIRE, wheels still requested
    fc_nr = make_fc()
    d = fc_nr.step(NOW, view([t], v.pan, v.tilt, st=status(ready=False)))
    assert (
        d.fire == 0
        and "not_ready" in d.reasons
        and d.flywheel == fc_nr.cfg.flywheel_duty
    )
    # E-stop pressed / key off: vetoed before any target logic
    for st in (status(estop=True), status(key=False), status(state=p.SAFE)):
        f = make_fc()
        d = f.step(NOW, view([t], v.pan, v.tilt, st=st))
        assert d.fire == 0 and d.inhibit
    # shot cap
    fc_cap = make_fc(shot_cap=2)
    v2 = aligned(fc_cap, t)
    fc_cap.shots = 2
    d = fc_cap.step(NOW, v2)
    assert d.fire == 0 and d.disarm


def test_ablation_without_the_ph_gate_still_obeys_the_vetoes():
    t = tgt(sig=1.5)
    fc = make_fc(ph_gate=False)
    v = aligned(fc, t)
    assert (
        fc.step(NOW, v).fire == 1
    )  # lite's pointing tolerance does not look at the covariance
    v_p = view(
        [t], v.pan, v.tilt, person_t=NOW - 0.1, persons=[(0.8, (100, 100, 200, 400))]
    )
    d = make_fc(ph_gate=False).step(NOW, v_p)
    assert d.fire == 0 and "person" in d.reasons
