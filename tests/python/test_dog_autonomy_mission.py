import math
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl
import numpy as np
import pytest
from autonomy import config, mission, planner
from napl import program

EX = ROOT / "examples" / "dog" / "missions"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
# scenario -> golden state hash (default seed); recorded in Step 5
GOLDEN = {
    "map_quality/map_quality": "2094372fd1eeb1aa",
    "coverage_smoke/coverage_smoke": "45570453d43e9c7a",
}
CFG = config.override(
    config.load(),
    **{"map.size": 20.0, "plan.t_budget": 100.0, "plan.return_margin": 5.0},
)


def test_executive_sweeps_skips_fills_returns():
    ex = mission.Executive(CFG, (-8.0, -8.0))
    cost = np.ones((40, 40))
    mapped = np.zeros((200, 200), bool)  # pool 2 on 400 cells
    assert (
        ex.update(0.5, (-8.0, -8.0, 0.0), cost, mapped, False) == []
        and ex.phase == ex.INIT
    )
    p = ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.SWEEP and p[-1] == pytest.approx((-6.0, -6.0))
    p = ex.update(2.0, (-6.0, -6.0, 0.0), cost, mapped, False)  # reached: next lane end
    assert p[-1] == pytest.approx((6.0, -6.0))
    ex.update(3.0, (0.0, -6.0, 0.0), cost, mapped, False)
    p = ex.update(
        3.0 + CFG["plan"]["stuck_s"] + 0.1, (0.0, -6.0, 0.0), cost, mapped, False
    )  # no progress: skip
    assert p[-1] == pytest.approx((6.0, 0.0))
    for wp in planner.lanes(CFG, CFG["plan"]["lane_m"], (-8.0, -8.0))[2:]:
        ex.update(30.0, (*wp, 0.0), cost, mapped, False)
    assert (
        ex.phase == ex.FILL
    )  # lanes done; with nothing mapped the first gap is the robot's own block
    mapped[:] = True
    mapped[25:30, 25:30] = False  # one gap block (5, 5), far from the robot
    p = ex.update(
        31.0, (6.0, 6.0, 0.0), cost, mapped, False
    )  # own block reached -> tried; next gap, viewed 2.5 m short of it
    assert ex.goal_block == (5, 5) and math.dist(
        p[-1], planner.centre((5, 5), CFG)
    ) == pytest.approx(2.5, abs=0.75)
    ex.update(
        32.0, (*p[-1], 0.0), cost, mapped, False
    )  # reached the view point: tried; no gap left -> home
    assert ex.phase == ex.RETURN
    ex.update(33.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.DONE


def test_sweep_goes_straight_over_unknown_ground_instead_of_back_over_the_mapped_strip():
    ex = mission.Executive(CFG, (-8.0, -8.0))
    unknown = np.ones((40, 40), bool)
    unknown[:4, :] = False  # a mapped strip along the south edge
    cost = np.where(unknown, 2.0, 1.0)
    mapped = np.zeros((200, 200), bool)
    ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False, unknown)
    ex.wi, ex.goal = 1, (6.0, -6.0)  # lane end reached next
    p = ex.update(
        2.0, (6.0, -6.0, 0.0), cost, mapped, False, unknown
    )  # next goal (6, 0): 12 blocks north
    assert p[-1] == pytest.approx((6.0, 0.0))
    assert all(x > 5.0 for x, _ in p)  # no detour west over the strip
    lane = ex.update(
        3.0, (6.0, 0.0, 0.0), cost, mapped, False, unknown
    )  # goal (-6, 0): a straight run west on row y = 0
    ys = {round(y, 6) for _, y in lane}
    assert max(ys) - min(ys) < 1.0  # stays on the lane row, not down at the strip


def test_standing_in_a_wall_block_plans_around_the_wall_not_through_it():
    ex = mission.Executive(CFG, (-8.0, -8.0))
    cost = np.ones((40, 40))
    cost[:, 10] = np.inf  # a wall one block thick ...
    cost[3, 10] = 1.0  # ... with a door far to the south
    mapped = np.zeros((200, 200), bool)
    ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    ex.goal, ex.best = (6.0, 0.25), (math.inf, 1.0)
    x, y = planner.centre(
        (20, 10), CFG
    )  # the robot is IN a wall block (it pushed up against a face seen from 0.3 m)
    p = ex.update(2.0, (x, y, 0.0), cost, mapped, False)
    assert p and p[-1] == pytest.approx((6.0, 0.25))
    blocks = {planner.block_of(*q, CFG) for q in p}
    assert (20, 11) not in blocks  # no tunnelling through the wall
    assert any(b[0] <= 4 and b[1] == 10 for b in blocks)  # it goes through the door


def test_return_is_triggered_by_the_real_way_home_not_the_straight_line():
    mapped = np.zeros((200, 200), bool)
    d = math.dist((8.0, -8.0), (-8.0, -8.0))
    t_straight = 100.0 - CFG["plan"]["detour"] * d / CFG["carrier"]["v_max"] - 5.0
    t = t_straight - 10.0  # still sweeping by the straight-line rule

    def phase(cost):
        ex = mission.Executive(CFG, (-8.0, -8.0))
        ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False)
        for k in range(5):  # the way home is re-planned every 5th update
            ex.update(t + 0.1 * k, (8.0, -8.0, 0.0), cost, mapped, False)
        return ex.phase

    free = np.ones((40, 40))
    wall = free.copy()
    wall[:, 20] = (
        np.inf
    )  # a wall between the robot and home, open only at the far (north) end
    wall[39, 20] = 1.0
    assert (
        phase(free) == mission.Executive.SWEEP
        and phase(wall) == mission.Executive.RETURN
    )


def test_unstick_backs_off_when_commanded_but_not_moving_then_goes_the_other_way():
    r = mission.Robot.__new__(mission.Robot)
    r.cfg, r._ref, r._back_until, r._dir = CFG, None, -1.0, 1.0
    P, pose = CFG["plan"], (1.0, 2.0, 0.3)
    assert r._unstick(0.0, pose, 1.0, 0.0) == (1.0, 0.0)  # reference taken
    assert r._unstick(P["unstick_s"] - 0.1, pose, 1.0, 0.0) == (1.0, 0.0)
    t = P["unstick_s"] + 0.1
    assert r._unstick(t, pose, 1.0, 0.0) == (-0.4, 0.0)  # back off
    assert r._unstick(t + 1.0, pose, 1.0, 0.0) == (-0.4, 0.0)  # still backing
    t2 = t + P["unstick_back_s"] + 0.1
    assert r._unstick(t2, pose, 1.0, 0.0) == (1.0, 0.0)  # done, new reference
    assert r._unstick(t2 + P["unstick_s"] + 0.1, pose, 1.0, 0.0) == (
        0.4,
        0.0,
    )  # next time forward
    assert r._unstick(99.0, (1.0, 2.0, 0.6), 0.0, 1.0) == (
        0.0,
        1.0,
    )  # turning in place is progress


def test_pursuit_lookahead_can_be_shortened_near_walls():
    lim = {"v_max": 1.5, "wz_max": 1.0}
    path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]
    _, wz_short = planner.pursue((0.0, -0.1, 0.0), path, CFG, lim, 0.45)
    _, wz_def = planner.pursue((0.0, -0.1, 0.0), path, CFG, lim)
    assert (
        wz_short > wz_def > 0.0
    )  # a shorter lookahead steers harder back onto the path


def test_return_trigger_and_abort():
    ex = mission.Executive(CFG, (-8.0, -8.0))
    cost, mapped = np.ones((40, 40)), np.zeros((200, 200), bool)
    ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    d = math.dist((8.0, 8.0), (-8.0, -8.0))
    t_ret = 100.0 - CFG["plan"]["detour"] * d / CFG["carrier"]["v_max"] - 5.0
    ex.update(t_ret - 0.5, (8.0, 8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.SWEEP
    p = ex.update(t_ret + 0.5, (8.0, 8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.RETURN and p[-1] == pytest.approx((-8.0, -8.0))
    assert (
        ex.update(t_ret + 1.0, (8.0, 8.0, 0.0), cost, mapped, True) == []
        and ex.phase == ex.ABORT
    )
    assert ex.update(t_ret + 2.0, (8.0, 8.0, 0.0), cost, mapped, False) == []


def test_every_mission_is_listed_with_its_monitors():
    found = sorted(
        str(p.relative_to(EX)).removesuffix(".scenario.toml")
        for p in EX.rglob("*.scenario.toml")
    )
    assert set(GOLDEN) <= set(found)
    for name in found:
        sc = tomllib.loads((EX / f"{name}.scenario.toml").read_text())
        assert {"map_rms_within_5cm", "localized_within_30cm"} <= {
            m["name"] for m in sc["monitors"]
        }
        config.leaves(sc["auto"], "auto")  # numbers rule


@needs_kernel
@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_mission_passes_replays_and_matches_its_golden_hash(tmp_path, name):
    res, _ = program.run(EX / f"{name}.scenario.toml", tmp_path / "runs")
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["pass"], res["fail_reasons"]
    assert GOLDEN[name] is None or res["sim"]["state_hash"] == GOLDEN[name], res["sim"][
        "state_hash"
    ]


LONG = [
    "coverage_rubble/coverage_rubble",
    "coverage_rubble_20k/coverage_rubble_20k",
    "coverage_ruins/coverage_ruins",
]  # bg only, never run by pytest


def test_long_missions_are_listed_with_the_mission_monitors():
    for name in LONG:
        sc = tomllib.loads((EX / f"{name}.scenario.toml").read_text())
        mon = {m["name"] for m in sc["monitors"]}
        assert {
            "map_rms_within_5cm",
            "localized_within_30cm",
            "covers_95_percent",
            "returns_home",
        } <= mon
        auto = config.leaves(sc["auto"], "auto")
        assert auto["size"] == 100.0 and auto["t_budget"] == 1200.0
