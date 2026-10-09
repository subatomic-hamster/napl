"""The Skydio-style autonomy demo (examples/skydio, assemblies/skydio.toml): assembly, forest, depth -> map, planner, flights, scenarios."""

import json
import os
import subprocess
import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design, program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "skydio"
sys.path.insert(0, str(EX))

import forest
import mapping
import planner

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")

# a camera looking along world +x: sensor z (depth) -> +x, sensor x (image up) -> +z, sensor y (image right) -> -y
R_LOOK_X = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
CAM = mapping.Camera(160, 90, np.radians(87), np.radians(58), 0.28, 10.0)


def wall_depth(x_wall, cam_pos):
    """Depth image of a plane x = x_wall seen from cam_pos looking along +x (NaN beyond the camera's range)."""
    d = np.full((CAM.n_v, CAM.n_h), float(x_wall - cam_pos[0]), np.float32)
    d[d > CAM.max_range] = np.nan
    return d


def mapped(depth, pos=(0.0, 0.0, 2.0), frames=3):
    m = mapping.VoxelMap()
    for _ in range(frames):
        m.integrate(depth, CAM, np.array(pos), R_LOOK_X)
    return m


# ---- assembly --------------------------------------------------------------------------------------------------------
@needs_kernel
def test_assembly_builds_drc_clean_and_depth_camera_is_wired(tmp_path):
    asm = REPO / "assemblies" / "skydio.toml"
    fs = design.check(asm)
    assert [f for f in fs if f.severity == "error"] == [], [
        (f.code, f.where) for f in fs if f.severity == "error"
    ]
    r = design.build(asm, REPO / "parts", tmp_path)
    devs = {d["name"] for d in r["report"]["devices"]}
    assert {"depth", "load_compute", "battery", "imu"} <= devs
    assert (
        1.6 < r["report"]["mass"]["total"] < 1.8
    )  # x500v2 1.52 kg + camera, Pi 5, regulator
    sim = napl.Sim.load(r["xml"])
    sim.step(50)
    assert not sim.diverged


# ---- forest ----------------------------------------------------------------------------------------------------------
def test_forest_is_deterministic_and_seeded():
    a, b, c = (
        forest.generate(5, "goal"),
        forest.generate(5, "goal"),
        forest.generate(6, "goal"),
    )
    assert json.dumps(a.to_json()) == json.dumps(b.to_json())
    assert json.dumps(a.to_json()) != json.dumps(c.to_json())
    assert (
        50
        <= sum(
            o.name.startswith("tree") and o.name.endswith("trunk") for o in a.obstacles
        )
        <= 65
    )


def test_forest_clearance_is_exact_surface_distance_and_start_is_free():
    f = forest.generate(3, "goal")
    o = next(o for o in f.obstacles if o.name.endswith("trunk"))
    p = np.array([o.pos[0] + o.size[0] + 1.5, o.pos[1], 2.0])
    assert f.clearance(p) <= 1.5 + 1e-9  # the nearest thing is at most this trunk
    assert f.clearance(np.array([*f.start[:2], 2.0])) > 2.0


def test_follow_forest_has_a_walkable_path_clear_of_obstacles():
    for seed in (1, 2, 3):
        f = forest.generate(seed, "follow")
        pts = np.array(f.path)
        assert len(pts) >= 2 and pts[0][0] < 8 and pts[-1][0] > 50
        for a, b in zip(pts[:-1], pts[1:]):
            for t in np.linspace(0, 1, 20):
                q = a + t * (b - a)
                assert f.clearance(np.array([q[0], q[1], 1.0])) > 0.5, (seed, q)


# ---- depth -> map ----------------------------------------------------------------------------------------------------
def test_wall_in_the_depth_image_becomes_occupied_voxels_at_the_right_place():
    m = mapped(wall_depth(6.0, (0, 0, 2)))
    pts = m.occupied_points((0, 0), 12.0)
    assert len(pts) > 100
    assert abs(float(np.median(pts[:, 0])) - 6.0) < 0.2  # within one voxel of the wall
    w = m.window(np.array([0.0, 0.0, 2.0]))
    assert w.clearance(np.array([[6.0 - 1.0, 0.0, 2.0]]))[0] == pytest.approx(
        1.0, abs=0.35
    )
    assert w.is_known_free(np.array([[3.0, 0.0, 2.0]]))[0]  # a ray crossed it
    assert not w.is_known_free(np.array([[7.5, 0.0, 2.0]]))[
        0
    ]  # behind the wall: never seen


def test_empty_depth_marks_space_free_but_maps_nothing():
    m = mapped(np.full((CAM.n_v, CAM.n_h), np.nan, np.float32))
    assert m.n_occupied == 0
    w = m.window(np.array([0.0, 0.0, 2.0]))
    assert w.is_known_free(np.array([[5.0, 0.0, 2.0]]))[0]


# ---- planner ---------------------------------------------------------------------------------------------------------
def plan_in(depth, goal=(40.0, 0.0), p=(0.0, 0.0, 2.0), v=(0.0, 0.0, 0.0)):
    m = mapped(depth, p)
    pl = planner.Planner(planner.Params())
    p, v = np.array(p), np.array(v)
    w = m.window(p, half=10.0)
    bearing = float(np.arctan2(goal[1] - p[1], goal[0] - p[0]))
    return pl.plan(p, v, 0.0, w, bearing, planner.goal_objective(goal, 2.0)), w


def test_open_space_plans_fast_toward_the_goal():
    plan, _ = plan_in(np.full((CAM.n_v, CAM.n_h), np.nan, np.float32))
    assert plan.mode == "track"
    assert np.linalg.norm(plan.vdes[:2]) >= 4.0
    assert plan.vdes[0] > 3.5  # toward +x


def test_wall_ahead_never_plans_a_path_through_it_and_stops_short():
    plan, w = plan_in(wall_depth(5.0, (0, 0, 2)), v=(4.0, 0.0, 0.0))
    ts = np.arange(0.0, plan.duration, 0.05)
    path = np.array([plan.at(t)[0] for t in ts])
    assert (
        path[:, 0].max() < 5.0 - planner.Params().r_safe + 0.3
    )  # stops before the wall (map quantisation 0.2 m)
    assert w.clearance(path[ts > 0.3]).min() >= planner.Params().r_safe - 0.05


def test_pole_in_the_middle_is_passed_with_clearance():
    d = np.full((CAM.n_v, CAM.n_h), np.nan, np.float32)
    d[:, 74:86] = 5.0  # a 1.2 m wide, full-height column straight ahead
    plan, w = plan_in(d)
    ts = np.arange(0.0, plan.duration, 0.05)
    path = np.array([plan.at(t)[0] for t in ts])
    assert w.clearance(path[ts > 0.3]).min() >= planner.Params().r_safe - 0.05
    assert (
        abs(plan.vdes[1]) > 0.5 or plan.vdes[0] < 4.0
    )  # goes around it or slows, does not drive straight into it


def test_blind_spot_is_never_entered():
    """A camera that has seen nothing may not fly: every non-hover primitive leaves known-free space, so the planner brakes or scans."""
    m = mapping.VoxelMap()
    pl = planner.Planner(planner.Params())
    p = np.array([0.0, 0.0, 2.0])
    plan = pl.plan(
        p,
        np.zeros(3),
        0.0,
        m.window(p, half=10.0),
        0.0,
        planner.goal_objective((40.0, 0.0), 2.0),
    )
    assert np.linalg.norm(plan.vdes) <= 1.0 + 1e-9


# ---- search belief (subject lost) ------------------------------------------------------------------------------------
def test_visibility_needs_range_band_field_of_view_and_a_clear_line_of_sight():
    import search

    cam = np.array([[0.0, 0.0, 1.9]])
    face = np.array([[1.0, 0.0]])
    tgt = np.array(
        [[5.0, 0.0], [1.5, 0.0], [9.0, 0.0], [0.0, 5.0], [5.0, 5.0]]
    )  # ahead / too close / too far / abeam / 45 deg off
    open_win = mapped(np.full((CAM.n_v, CAM.n_h), np.nan, np.float32), frames=1).window(
        cam[0], half=10.0
    )
    assert search.visible(open_win, cam, tgt, face)[0].tolist() == [
        True,
        False,
        False,
        False,
        False,
    ]
    wall_win = mapped(wall_depth(3.0, cam[0])).window(
        cam[0], half=10.0
    )  # a wall at x = 3 hides the person at x = 5
    assert not search.visible(wall_win, cam, tgt, face)[0, 0]


def test_belief_keeps_the_walkers_pace_and_loses_the_mass_the_camera_could_see():
    import search

    rng = np.random.default_rng(0)
    win = mapped(np.full((CAM.n_v, CAM.n_h), np.nan, np.float32), frames=1).window(
        np.zeros(3), half=10.0
    )
    b = search.Belief(rng)
    b.start(
        np.array([4.0, 0.0, 0.0, 2.4]), np.diag([0.05, 0.05, 0.1, 0.1])
    )  # walking along +y at 2.4 m/s
    for _ in range(50):  # 2 s at 25 Hz
        b.predict(0.04, win)
    m, v, sg = b.estimate()
    assert (
        0.6 * 2.4 <= np.hypot(*v) <= 1.4 * 2.4 and abs(m[1] - 4.8) < 1.5 and sg < 3.5
    )  # 2.9 m measured: the turn noise (0.8 rad/s) spreads the belief
    # a camera at the origin looking along +x sees the part of the belief in its sector, finds nothing, and the surviving mass is outside it
    cam_R = R_LOOK_X
    face = cam_R[:2, 2] / np.linalg.norm(cam_R[:2, 2])
    half = (
        b.n // 2
    )  # half the belief in the camera's sector (x = 4, y = 0), half beside it (x = 4, y = 4: 45 deg off the axis)
    b.xy = np.vstack(
        [np.tile([4.0, 0.0], (half, 1)), np.tile([4.0, 4.0], (b.n - half, 1))]
    )
    b.w = np.full(b.n, 1.0 / b.n)
    cam = np.array([[0.0, 0.0, 1.9]])
    before = search.visible(win, cam, b.xy, face[None])[0].mean()
    assert 0.4 < before < 0.6
    for _ in range(5):
        b.observe_unseen(win, cam[0], cam_R)
    after = (search.visible(win, cam, b.xy, face[None])[0] * b.w).sum()
    assert after < 0.3 * before


# ---- flights (kernel) ------------------------------------------------------------------------------------------------
@needs_kernel
def test_short_flight_is_collision_free_and_replays_bit_identically(tmp_path):
    cal = tmp_path / "cal"
    runs = []
    for k in range(2):
        out = tmp_path / f"r{k}"
        r = subprocess.run([sys.executable, str(EX / "fly.py"), str(out), "goal", "4"], capture_output=True, text=True,
                           env={**os.environ, "SKYDIO_CAL": str(cal), "SKYDIO_T_MAX": "9"})  # fmt: skip
        assert r.returncode == 0, r.stderr[-2000:]
        runs.append(
            (
                json.loads(r.stdout.strip().splitlines()[-1]),
                json.loads((out / "native.json").read_text()),
            )
        )
    for s, _ in runs:
        assert s["contact_steps"] == 0 and s["min_clearance_m"] > 0.3
        assert s["fly_s"] > 3 and s["max_speed_mps"] > 1.5
    assert runs[0][1]["state_hash"] == runs[1][1]["state_hash"]  # same seed, same bits


@needs_kernel
@pytest.mark.parametrize("mission", ["goal", "follow"])
def test_scenario_passes_with_replay_match(tmp_path, monkeypatch, mission):
    monkeypatch.setenv("SKYDIO_CAL", str(tmp_path / "cal"))
    res, _ = program.run(EX / f"{mission}.scenario.toml", tmp_path / "runs")
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["pass"], res[
        "fail_reasons"
    ]  # every monitor, including the strict 3-8 m follow_distance_band


def test_one_seed_for_scene_and_controller(monkeypatch):
    """The scenario's `seed` parameter (NAPL_PARAMS) is what both the model script and fly.py read; SKYDIO_SEED is the standalone fallback."""
    import skydio_scene

    for mission in ("goal", "follow"):
        monkeypatch.delenv("NAPL_PARAMS", raising=False)
        monkeypatch.delenv("SKYDIO_SEED", raising=False)
        assert skydio_scene.seed(mission) == 1  # the declared default
        monkeypatch.setenv("SKYDIO_SEED", "5")
        assert skydio_scene.seed(mission) == 5
        monkeypatch.setenv("NAPL_PARAMS", '{"seed": 111}')
        assert skydio_scene.seed(mission) == 111  # a caller's parameter wins
