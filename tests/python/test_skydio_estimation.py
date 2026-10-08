"""Skydio demo: the ESKF (IMU + VIO + barometer), the depth-image person detector / tracker and the thin-pole map persistence (examples/skydio)."""

import sys
from pathlib import Path

import numpy as np

EX = Path(__file__).resolve().parents[2] / "examples" / "skydio"
sys.path.insert(0, str(EX))

import detector
import estimator as E
import mapping
from forest import generate

CAM = mapping.Camera(160, 90, np.radians(87), np.radians(58), 0.28, 10.0)
# camera looking along world +x: sensor z (depth) -> +x, sensor x (image up) -> +z, sensor y (image right) -> -y
R_LOOK_X = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])


def test_so3_exp_log_roundtrip():
    w = np.array([0.3, -0.2, 0.5])
    assert np.allclose(E.log_so3(E.exp_so3(w)), w, atol=1e-9)
    assert np.allclose(E.exp_so3(w) @ E.exp_so3(w).T, np.eye(3), atol=1e-12)


def fly_synthetic(seconds=40.0, vio=True, seed=0):
    """A 3-D figure-of-eight at ~3 m/s, truth -> IMU (device-like noise + ImuBias) + VioSim + height; -> (truth p, estimate p, filter)."""
    rng = np.random.default_rng(seed)
    dt, rate = 0.002, 500.0
    sg, sa = 4.887e-5 * np.sqrt(rate / 2), 6.374e-4 * np.sqrt(rate / 2)
    bias = E.ImuBias(rng)
    vio_sim = E.VioSim(rng)
    p, v, R = np.array([0.0, 0.0, 2.0]), np.zeros(3), np.eye(3)
    f = E.Eskf(p, v, R, sg, sa)
    t, nxt = 0.0, 0.04
    errs, truth = [], []
    for k in range(int(seconds / dt)):
        t += dt
        a = np.array(
            [-1.5 * np.sin(0.5 * t), -1.0 * np.sin(1.0 * t), 0.3 * np.cos(0.7 * t)]
        )  # world acceleration
        w = np.array([0.15 * np.sin(0.8 * t), 0.1 * np.cos(0.6 * t), 0.2])  # body rate
        spec = R.T @ (a + np.array([0.0, 0.0, E.G]))  # specific force
        g_m, a_m = bias.apply(
            w + sg * rng.standard_normal(3), spec + sa * rng.standard_normal(3), dt
        )
        f.predict(g_m, a_m, dt)
        p, v, R = p + v * dt + 0.5 * a * dt * dt, v + a * dt, R @ E.exp_so3(w * dt)
        if vio and t >= nxt:
            nxt += 0.04
            vb, Rm = vio_sim.sample(0.04, R, v)
            f.update_vio(
                f.t, vb, Rm
            )  # no delay here (the delay path is exercised in the flight)
        if k % 10 == 0:
            f.update_height(p[2] + 0.1 * rng.standard_normal())
        errs.append(np.linalg.norm(f.p - p))
        truth.append(p.copy())
    path = np.sum(np.linalg.norm(np.diff(np.array(truth), axis=0), axis=1))
    return np.array(errs), path, f, p, v, R


def test_eskf_with_vio_tracks_a_manoeuvring_vehicle_with_bounded_drift():
    errs, path, f, p, v, R = fly_synthetic()
    assert path > 80.0
    assert np.sqrt((errs**2).mean()) < 0.5  # about 1 % of the path
    assert errs[-1] < 0.015 * path
    assert np.linalg.norm(f.v - v) < 0.2
    assert np.degrees(np.linalg.norm(E.log_so3(R.T @ f.R))) < 2.0
    # the reported 1-sigma covers the error (not wildly over- or under-confident)
    assert 0.2 < np.linalg.norm(f.sigmas()[:3]) / max(errs[-1], 1e-3) < 20


def test_without_vio_the_imu_alone_diverges():
    errs_imu, *_ = fly_synthetic(vio=False)
    errs_vio, *_ = fly_synthetic(vio=True)
    assert np.sqrt((errs_imu**2).mean()) > 10 * np.sqrt((errs_vio**2).mean())


def test_filter_learns_the_gyro_bias():
    _, _, f, *_ = fly_synthetic(seconds=40.0, seed=3)
    bias = E.ImuBias(
        np.random.default_rng(3)
    ).bg  # the first draw of the same stream is the filter run's turn-on bias
    assert np.linalg.norm(f.bg - bias) < 0.5 * np.linalg.norm(bias) + 1e-3


# ---- detector --------------------------------------------------------------------------------------------------------------
def render(cam_pos, cyls, R=R_LOOK_X):
    """Analytic depth image (sensor z) of vertical cylinders [(x, y, r, h)] over the ground plane z = 0."""
    t = CAM.tangents()
    d = t.reshape(-1, 3) @ R.T  # world ray per unit depth
    out = np.full(len(d), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        tg = -cam_pos[2] / d[:, 2]  # ground
        out = np.where((tg > 0), tg, out)
        for cx, cy, r, h in cyls:
            o = cam_pos[:2] - np.array([cx, cy])
            a = (d[:, :2] ** 2).sum(1)
            b = 2 * (d[:, :2] @ o)
            c = o @ o - r * r
            disc = b * b - 4 * a * c
            s = (-b - np.sqrt(disc)) / (2 * a)
            z = cam_pos[2] + s * d[:, 2]
            hit = (disc > 0) & (s > 0) & (z >= 0) & (z <= h)
            out = np.where(hit, np.minimum(out, s), out)
    out = out.reshape(CAM.n_v, CAM.n_h).astype(np.float32)
    out[(out > CAM.max_range) | ~np.isfinite(out)] = np.nan
    return out


def test_detector_finds_the_person_and_rejects_trunks_poles_and_boulders():
    cam = np.array([0.0, 0.0, 1.9])
    # person (r 0.25 m, 1.75 m) at 5 m slightly left; trunk (r .3, 7 m) at 7 m right; pole (r .07, 4 m); a 1 m rock-like stump (r .6, h 1.0)
    img = render(
        cam,
        [
            (5.0, 0.8, 0.25, 1.75),
            (7.0, -1.5, 0.3, 7.0),
            (6.0, -0.2, 0.07, 4.0),
            (4.0, -1.8, 0.6, 1.0),
        ],
    )
    dets = detector.PersonDetector(CAM).detect(img, cam, R_LOOK_X)
    assert len(dets) == 1, dets
    d = dets[0]
    assert np.hypot(d["xy"][0] - 5.0, d["xy"][1] - 0.8) < 0.5
    assert abs(d["z_top"] - 1.75) < 0.25


def test_detector_misses_a_person_hidden_behind_a_trunk():
    cam = np.array([0.0, 0.0, 1.9])
    img = render(cam, [(8.0, 0.0, 0.25, 1.75), (4.0, 0.0, 0.3, 7.0)])
    assert detector.PersonDetector(CAM).detect(img, cam, R_LOOK_X) == []


def test_tracker_follows_a_walker_and_coasts_through_a_miss():
    trk = detector.Tracker()
    for k in range(60):
        t = 0.04 * k
        xy = np.array([5.0 + 2.4 * t, 0.5])
        dets = (
            []
            if 30 <= k < 40
            else [dict(xy=xy + 0.05 * np.sin(k), range=5.0, score=0.5)]
        )
        r = trk.update(t, dets)
    assert r[2] and abs(r[1][0] - 2.4) < 0.3 and np.linalg.norm(r[0] - xy) < 0.2
    # a far-away second detection is gated out
    r = trk.update(
        0.04 * 60, [dict(xy=xy + np.array([6.0, 4.0]), range=9.0, score=0.9)]
    )
    assert not r[2]


# ---- thin poles stay in the map --------------------------------------------------------------------------------------------
def test_thin_pole_voxel_survives_neighbouring_background_rays_only_with_hit_hold():
    wall = np.full((CAM.n_v, CAM.n_h), 9.0, np.float32)  # a far wall
    d = wall.copy()
    d[:, 80] = (
        5.0  # a 1-pixel wide pole at 5 m, resolved by a pixel in only one frame of three (sub-pixel position, noise)
    )
    pos = np.array([0.0, 0.0, 2.0])
    n = {}
    for hold in (0, 12):
        m = mapping.VoxelMap(hit_hold=hold)
        for k in range(21):
            m.integrate(d if k % 3 == 0 else wall, CAM, pos, R_LOOK_X)
        pts = m.occupied_points((0, 0), 8.0)
        n[hold] = int(((np.abs(pts[:, 0] - 5.0) < 0.3) & (pts[:, 2] > 1.0)).sum())
    assert n[12] > 0 and n[12] > n[0]


def test_thin_pole_forest_variant_adds_poles_and_leaves_default_seeds_unchanged():
    base = generate(7, "goal", thin=False)
    thin = generate(7, "goal", thin=True)
    extra = [o for o in thin.obstacles if o.name.startswith("thinpole")]
    assert 10 <= len(extra) <= 14 and all(0.025 <= o.size[0] <= 0.05 for o in extra)
    assert [o.name for o in base.obstacles] == [
        o.name for o in thin.obstacles if not o.name.startswith("thinpole")
    ]
