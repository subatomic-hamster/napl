import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import numpy as np
import pytest
from autonomy import config, mission, planner

CFG = config.override(
    config.load(),
    **{
        "map.size": 10.0,
        "trav.block": 0.2,
        "trav.clearance": 1,
        "plan.tour": 1,
        "plan.sweep": 0,
    },
)
NB = 50  # blocks per side
NP = 100  # pooled scoring cells per side (10 cm)


def unmapped(*patches):
    """Scoring grid, all mapped except square patches (x0, y0, side) in m."""
    m = np.ones((NP, NP), bool)
    for x0, y0, s in patches:
        i, j = int((y0 + 5) / 0.1), int((x0 + 5) / 0.1)
        m[i : i + int(s / 0.1), j : j + int(s / 0.1)] = False
    return m


def test_box_sum_matches_the_brute_force_window_sum():
    a = np.random.default_rng(0).random((12, 9))
    r = 2
    b = planner.box_sum(a, r)
    for i, j in ((0, 0), (5, 4), (11, 8), (3, 8)):
        w = a[max(i - r, 0) : i + r + 1, max(j - r, 0) : j + r + 1]
        assert b[i, j] == pytest.approx(w.sum())


def test_the_tour_visits_both_unmapped_rooms_nearest_first_then_returns():
    ex = mission.Executive(CFG, (-4.0, -4.0))
    ex.phase = ex.FILL
    cost, clear = np.ones((NB, NB)), np.full((NB, NB), 5.0)
    near, far = (-3.0, -3.0, 1.5), (3.0, 3.0, 1.5)
    pose = (-4.0, -4.0, 0.0)
    ex.update(10.0, pose, cost, unmapped(near, far), False, None, clear)
    assert math.dist(ex.goal, (-2.25, -2.25)) < 3.5  # the near patch first
    pose = (*ex.goal, 0.0)
    ex.update(
        20.0, pose, cost, unmapped(far), False, None, clear
    )  # near one now mapped
    assert math.dist(ex.goal, (3.75, 3.75)) < 3.5
    pose = (*ex.goal, 0.0)
    ex.update(30.0, pose, cost, unmapped(), False, None, clear)  # all mapped
    assert ex.phase == ex.RETURN and ex.goal == ex.home


def test_the_tour_does_not_choose_floor_inside_walls():
    # unmapped cells with no clearance (wall interiors) never count as value
    ex = mission.Executive(CFG, (-4.0, -4.0))
    ex.phase = ex.FILL
    cost, clear = np.ones((NB, NB)), np.full((NB, NB), 5.0)
    clear[30:40, 30:40] = 0.0
    ex.update(
        10.0, (-2.0, -2.0, 0.0), cost, unmapped((1.0, 1.0, 1.5)), False, None, clear
    )
    assert ex.phase == ex.RETURN


def test_rubble_never_calls_the_room_planner(monkeypatch):
    cfg = config.override(config.load(), **{"map.size": 10.0, "trav.block": 0.2})
    assert (
        cfg["plan"]["tour"] == 0
        and cfg["trav"]["clearance"] == 0
        and cfg["plan"]["sweep"] == 1
    )

    def boom(*a, **k):
        raise AssertionError("choose_view called in rubble mode")

    monkeypatch.setattr(planner, "choose_view", boom)
    ex = mission.Executive(cfg, (-4.0, -4.0))
    ex.phase = ex.FILL
    ex.update(
        10.0,
        (-4.0, -4.0, 0.0),
        np.ones((NB, NB)),
        unmapped((1.0, 1.0, 1.5)),
        False,
        None,
    )
    assert ex.goal is not None
