"""Skydio landing demo (examples/skydio-land, assemblies/skydio-land.toml): assembly, worlds, landing-zone detection, deck interface, scenarios.
Flights (the golden hashes, the Run slot) are in test_skydio_land_flights.py."""

import math
import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "skydio-land"
sys.path.insert(0, str(EX))
sys.path.insert(0, str(REPO / "examples" / "skydio"))

import land as L
import land_world as W

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
LP = L.LandParams()


# ---- assembly ----------------------------------------------------------------------------------------------------------------------
@needs_kernel
def test_assembly_is_drc_clean_with_a_pin_level_lidar_and_an_rtk_rover(tmp_path):
    asm = REPO / "assemblies" / "skydio-land.toml"
    fs = design.check(asm)
    assert [f for f in fs if f.severity == "error"] == [], [
        (f.code, f.where) for f in fs if f.severity == "error"
    ]
    r = design.build(asm, REPO / "parts", tmp_path)
    devs = {d["name"] for d in r["report"]["devices"]}
    assert {"depth", "lidar", "rtk", "imu", "battery"} <= devs
    import tomllib

    wires = {w["name"]: w for w in tomllib.loads(asm.read_text())["wires"]}
    assert {"lidar-tx-to-pi", "lidar-rx-from-pi"} <= wires.keys()
    # the TF-Luna's UART on the Pi 5's UART2 pins (GPIO4 = J8.7, GPIO5 = J8.29), pin by pin, not a port shortcut
    assert wires["lidar-tx-to-pi"]["to_pins"] == ["J8.29"] and wires[
        "lidar-rx-from-pi"
    ]["to_pins"] == ["J8.7"]
    assert (
        1.7 < r["report"]["mass"]["total"] < 1.9
    )  # the Skydio's 1.73 kg + lidar 5 g + RTK board 40 g


@needs_kernel
def test_the_original_assembly_is_untouched():
    """examples/skydio and assemblies/skydio.toml are imported, not edited: their git blobs equal the branch point's."""
    import subprocess

    base = subprocess.run(
        ["git", "merge-base", "HEAD", "napl-v1"],
        cwd=REPO,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if not base:
        pytest.skip("no napl-v1 to compare with")
    out = subprocess.run(
        [
            "git",
            "diff",
            "--stat",
            base,
            "--",
            "assemblies/skydio.toml",
            "examples/skydio",
            "examples/hopper",
            "assemblies/hopper.toml",
        ],
        cwd=REPO,
        capture_output=True,
        text=True,
    )
    assert out.stdout.strip() == "", out.stdout


# ---- worlds ------------------------------------------------------------------------------------------------------------------------
def test_forest_world_is_seeded_and_carves_three_clearings():
    a, b = W.forest_world(5, hazard=False), W.forest_world(5, hazard=False)
    assert a.to_json() == b.to_json()
    assert W.forest_world(6, hazard=False).to_json() != a.to_json()
    for z in a.zones:
        assert (
            a.forest.clearance((z.x, z.y, 1.0)) > 4.0
        )  # no tree, pole or rock inside the clearing
    des, alt = a.zones[0], a.zones[1]
    assert 10.0 < math.hypot(des.x - alt.x, des.y - alt.y) < 16.0
    assert (
        W.forest_world(5, hazard=True).zones[0].safe is False
        and W.forest_world(5, hazard=False).zones[0].safe is True
    )


def test_forest_hazard_draw_is_near_its_probability():
    n = 200
    bad = sum(not W.forest_world(s).zones[0].safe for s in range(1, n + 1))
    assert (
        0.25 * n < bad < 0.55 * n
    )  # HAZARD_P = 0.4: the alternate-zone logic is exercised in a good share of the seeds


@pytest.mark.parametrize("fam", ["summit", "ledge", "mesa_brink"])
def test_terrain_world_is_seeded_and_roundtrips_through_json(fam):
    a, b = W.terrain_world(fam, 3), W.terrain_world(fam, 3)
    assert (
        np.array_equal(a.terrain.heights, b.terrain.heights)
        and a.to_json()["zones"] == b.to_json()["zones"]
    )
    j = a.to_json()
    w = W.load_world(__import__("json").loads(__import__("json").dumps(j)))
    assert np.array_equal(w.terrain.heights, a.terrain.heights) and len(
        w.terrain.rocks
    ) == len(a.terrain.rocks)
    home = a.zones[2]
    assert (home.name, home.z) == ("home", 0.0) and a.start == (
        home.x,
        home.y,
        0.0,
    )  # the take-off pad is the datum
    for z in a.zones:
        assert abs(a.ground_z(z.x, z.y) - z.z) < 1e-3  # a pad is exactly flat


@pytest.mark.parametrize("fam", ["summit", "ledge"])
def test_a_hazard_puts_boulders_on_the_designated_pad_and_the_truth_test_sees_them(fam):
    bad, ok = (
        W.terrain_world(fam, 4, hazard=True),
        W.terrain_world(fam, 4, hazard=False),
    )
    assert ok.zones[0].safe and not bad.zones[0].safe and bad.zones[0].why == "rough"
    assert bad.zones[1].safe  # the alternate is clean
    assert len(bad.terrain.rocks) == len(ok.terrain.rocks) + 3


def test_mesa_brink_has_unsafe_and_safe_designated_pads_by_seed():
    flags = {W.terrain_world("mesa_brink", s).zones[0].safe for s in range(1, 9)}
    assert flags == {True, False}
    unsafe = next(
        w for w in (W.terrain_world("mesa_brink", s, hazard=True) for s in range(1, 9))
    )
    assert (
        unsafe.zones[0].why in ("drop-off", "slope")
        and unsafe.zones[1].safe
        and unsafe.zones[2].safe
    )


def test_truth_test_flat_ok_slope_step_and_cliff_fail():
    import napl.terrain as T

    flat = T.generate(
        "moorland",
        size=(200.0, 200.0),
        n=161,
        relief=10.0,
        roughness=0.0,
        seed=1,
        pads=[T.PadSpec("p", at=(0, 0), radius=3.5, blend=8.0)],
    )
    assert W.pad_truth(flat, 0.0, 0.0)[0]
    t = T.generate(
        "moorland", size=(200.0, 200.0), n=161, relief=10.0, roughness=0.0, seed=1
    )
    t.heights[:] = 0.0
    assert W.pad_truth(t, 0.0, 0.0)[0]
    x = np.linspace(-100.0, 100.0, t.nx)[None, :]
    t.heights[:] = math.tan(math.radians(5.0)) * x  # 5 deg: ok
    assert W.pad_truth(t, 0.0, 0.0)[0]
    t.heights[:] = math.tan(math.radians(12.0)) * x  # 12 deg: slope fails
    assert W.pad_truth(t, 0.0, 0.0)[1] == "slope"
    t.heights[:] = 0.0
    t.heights[:, t.nx // 2 + 3 :] -= 1.0  # a 1 m step down 4 m from the centre
    assert W.pad_truth(t, 0.0, 0.0)[1:2] == ("drop-off",)


# ---- landing-zone detection ---------------------------------------------------------------------------------------------------------
def lidar_pass(f, c=(0.0, 0.0), half=6.0, gap=0.2, step=0.1, noise=0.01, seed=0):
    """A lawn-mower of ground points z = f(x, y) + noise over the square around c (what a downward lidar sees); returns a GroundMap."""
    rng = np.random.default_rng(seed)
    g = L.GroundMap()
    for y in np.arange(-half, half + 1e-9, gap):
        for x in np.arange(-half, half + 1e-9, step):
            if (z := f(c[0] + x, c[1] + y)) is None:
                g.add_void(c[0] + x, c[1] + y)
            else:
                g.add(c[0] + x, c[1] + y, z + float(rng.normal(0, noise)))
    return g


def test_assess_accepts_a_flat_and_a_gently_sloped_pad():
    for slope in (0.0, math.tan(math.radians(5))):
        a = lidar_pass(lambda x, y: slope * x).assess((0.0, 0.0), LP)
        assert (
            a.ok
            and abs(a.slope_deg - math.degrees(math.atan(slope))) < 0.6
            and a.rms < 0.03
        )


@pytest.mark.parametrize("name,f,reason", [
    ("steep", lambda x, y: 0.2 * x, "slope"),
    ("rough", lambda x, y: 0.3 * np.sin(3 * x) * np.sin(3 * y), "rough"),
    ("boulder in the ring", lambda x, y: 0.5 if math.hypot(x - 2.5, y) < 0.6 else 0.0, "obstacle"),
    ("boulder on the pad", lambda x, y: 0.5 if math.hypot(x - 0.5, y - 0.2) < 0.6 else 0.0, "rough"),
    ("drop-off", lambda x, y: -3.0 if x > 3.0 else 0.0, "drop-off"),
    ("void beyond a cliff", lambda x, y: None if x > 3.0 else 0.0, "no return"),
])  # fmt: skip
def test_assess_rejects_unsafe_zones_with_the_right_reason(name, f, reason):
    a = lidar_pass(f).assess((0.0, 0.0), LP)
    assert not a.ok, name
    assert (
        a.reason in (reason, "rough", "obstacle")
        if name == "boulder on the pad"
        else a.reason == reason
    ), (name, a.reason, a.reasons)


def test_assess_moves_the_spot_off_a_boulder_when_there_is_room():
    f = lambda x, y: (
        0.4 if math.hypot(x - 1.0, y) < 0.5 else 0.0
    )  # a boulder 1 m from the centre
    assert (
        not lidar_pass(f, half=9.0).assess((0.0, 0.0), L.LandParams(search=1.5)).ok
    )  # every spot within 1.5 m has it in its 4 m ring
    a = lidar_pass(f, half=10.0).assess((0.0, 0.0), L.LandParams(search=4.0))
    assert (
        a.ok and math.hypot(a.x - 1.0, a.y) > 4.5
    )  # a spot beyond the ring of the boulder


def test_assess_needs_data_and_a_depth_map_obstacle_rejects():
    assert L.GroundMap().assess((0, 0), LP).reason == "no ground data"
    g = lidar_pass(lambda x, y: 0.0)
    tree = np.array([[1.0, 1.0, z] for z in np.arange(0.0, 3.0, 0.2)])
    here = L.LandParams(search=0.0)  # the centre only
    assert not g.assess(
        (0.0, 0.0), here, tree
    ).ok  # a tree 1.4 m from the spot: within obs_r
    assert g.assess((0.0, 0.0), here, np.zeros((0, 3))).ok
    assert g.assess(
        (0.0, 0.0), LP, tree
    ).ok  # with the 1.5 m search it moves away from the tree instead


def test_trim_leg_stops_short_of_a_wall():
    class Win:
        def clearance(self, pts):
            return 3.0 - pts[:, 0]  # a wall at x = 3 (distance to it)

    a, b = L.trim_leg(Win(), (-6.0, 0.0), (6.0, 0.0), 2.0, 0.4)
    assert (
        a[0] == pytest.approx(-6.0) and 2.0 < b[0] <= 3.0 - 0.4 - 0.3 + 1e-6
    )  # r_safe 0.4 + the 0.3 m margin
    assert L.trim_leg(Win(), (3.0, 0.0), (6.0, 0.0), 2.0, 0.4) is None


def test_sink_speed_slows_toward_the_ground():
    v = [L.sink_speed(h, LP) for h in (3.0, 1.4, 1.0, 0.5, 0.0)]
    assert (
        v[0] == LP.v_fast
        and v == sorted(v, reverse=True)
        and v[-1] == pytest.approx(LP.v_touch)
    )


# ---- RTK fix ------------------------------------------------------------------------------------------------------------------------
def test_geodetic_to_enu_matches_the_kernels_wgs84_flat_earth():
    lat0, lon0 = 46.5, 7.9
    e, n, u = L.geodetic_to_enu(lat0 + 1e-3, lon0 + 1e-3, 3250.0, lat0, lon0, 3200.0)
    assert (
        n == pytest.approx(111.2, abs=0.3) and u == 50.0
    )  # 1 mdeg of latitude ~ 111.2 m
    assert e == pytest.approx(111.3 * math.cos(math.radians(lat0)), abs=0.3)


# ---- deck-relative interface (scripted moving platform) ----------------------------------------------------------------------------------
def sea(t, f=(0.11, 0.17), a=(0.5, 0.3), ph=(0.4, 2.0)):
    """Scripted heave (m): two sinusoids (the peak of a 0.7 m significant wave height sea, simplified)."""
    return sum(ai * np.sin(2 * np.pi * fi * t + p) for fi, ai, p in zip(f, a, ph))


def impact_speed(start_policy, t0, v=0.3, h=1.0, pred=None):
    """The vehicle holds h above the deck and sinks at v from `start` (the policy's choice): relative closing speed at contact."""
    tc = start_policy(t0) + h / v
    eps = 1e-3
    vz = (sea(tc + eps) - sea(tc - eps)) / (2 * eps)
    return v + max(float(vz), 0.0)


def test_heave_predictor_follows_a_scripted_deck_and_touchdown_timing_beats_starting_at_once():
    pred = L.DeckPredictor()
    for t in np.arange(0.0, 40.0, 0.1):
        pred.add(t, float(sea(t)))
    assert pred.fit() and pred.rms_residual < 0.03
    z, vz = pred.predict([42.0, 45.0])
    assert np.allclose(z, sea(np.array([42.0, 45.0])), atol=0.05)
    td = L.Touchdown(pred)
    t_now = 40.0
    timed = impact_speed(td.start_time, t_now)
    # starting at once vs the timed start, over many decision instants: the timed one closes slower on average, and never above the sink rate + the deck's peak upward speed
    now, tim = [], []
    for k in range(40):
        t0 = 40.0 + 1.37 * k
        now.append(impact_speed(lambda t: t, t0))
        tim.append(impact_speed(td.start_time, t0))
    assert np.mean(tim) < np.mean(now) - 0.05 and max(tim) < max(now)
    assert timed < 0.3 + 0.08
    assert td.contact_speed(td.start_time(t_now)) < 0.38


def test_touchdown_starts_at_once_without_a_fit_and_a_fixed_pad_never_moves():
    p = L.DeckPredictor()
    assert L.Touchdown(p).start_time(12.0) == 12.0
    s = L.FixedPad(3.0, 4.0, 5.0).state(9.0)
    assert np.array_equal(s.p, [3.0, 4.0, 5.0]) and not s.v.any()
