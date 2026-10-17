import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import numpy as np
import pytest
from autonomy import clearance, config, mission

RES = 0.05
CFG = config.override(
    config.load(),
    **{
        "map.size": 10.0,
        "trav.block": 0.2,
        "trav.clearance": 1,
        "trav.near_lethal": 6.0,
    },
)
N = 200
XC = -5.0 + (np.arange(N) + 0.5) * RES


def wall_map(gap_lo, gap_hi):
    """Flat floor, a 0.3 m thick wall along y = 0 (1 m high) with a gap [gap_lo, gap_hi] in x."""
    hn = np.zeros((N, N), np.float32)
    wall = (np.abs(XC)[:, None] <= 0.15) & ~(
        (XC[None, :] >= gap_lo) & (XC[None, :] <= gap_hi)
    )
    hn[wall] = 1.0
    return hn


def test_edt_is_within_8_percent_of_the_euclidean_distance():
    m = np.zeros((120, 120), bool)
    m[40, 70] = True
    d = clearance.edt(m, RES)
    i, j = np.mgrid[0:120, 0:120]
    true = np.hypot(i - 40, j - 70) * RES
    far = true > 0.2
    assert np.abs(d[far] / true[far] - 1).max() < 0.09 and d[40, 70] == 0.0
    assert clearance.edt(np.zeros((8, 8), bool), RES).min() >= 1e3  # no obstacle: far


def test_obstacles_are_cells_higher_than_the_ground_by_more_than_a_step():
    hn = np.zeros((N, N), np.float32)
    hn[50:60, 50:60] = 1.0  # a wall block
    hn[100:110, 100:110] = 0.10  # a 10 cm rock: climbable (step_max 0.18)
    hn[:20] = np.nan  # unknown
    o = clearance.obstacles(hn, RES, CFG["trav"]["step_max"])
    assert o[50:60, 50:60].all() and o.sum() == 100


def test_obstacles_ignore_a_low_outlier_cell_and_single_noise_cells():
    # regression (ruins 50 m training seed 3): one -0.2 m cell lowered the ground of its 2 m block, so floor cells of 0.05 m read as walls
    hn = np.full((N, N), 0.05, np.float32)
    hn[60, 60] = -0.20
    hn[120, 120] = 0.60  # a single spike
    assert not clearance.obstacles(hn, RES, CFG["trav"]["step_max"]).any()


def test_a_08m_door_is_passable_a_03m_door_is_not():
    for gap, ok in ((0.8, True), (0.3, False)):
        hn = wall_map(0.15 - gap / 2, 0.15 + gap / 2)
        D = clearance.edt(clearance.obstacles(hn, RES, CFG["trav"]["step_max"]), RES)
        cost = clearance.block_cost(np.ones((50, 50)), D, CFG)
        assert (
            np.isfinite(cost[24:26]).any() == ok
        )  # the block rows at y = 0 (the wall's thickness)


def test_a_relaxed_lethal_opens_a_door_that_reads_closed():
    # regression (ruins 50 m training seeds 2, 6, 7): no way home at the strict clearance, the robot stood still to the end
    hn = wall_map(0.15 - 0.15, 0.15 + 0.15)  # 0.3 m gap
    D = clearance.edt(clearance.obstacles(hn, RES, CFG["trav"]["step_max"]), RES)
    known = np.ones((50, 50))
    assert not np.isfinite(clearance.block_cost(known, D, CFG)[24:26]).any()
    assert np.isfinite(
        clearance.block_cost(known, D, CFG, CFG["trav"]["clear_relaxed"])[24:26]
    ).any()


def test_a_centred_08m_door_gives_a_centred_straight_path():
    # regression (ruins 50 m, 0/16): block lethality made the doorway narrow and off-centre, the path hugged a jamb
    hn = wall_map(
        -0.25, 0.55
    )  # the door centre line is x = 0.15, not on a block centre (block centres: ..., 0.1, 0.3, ...)
    D = clearance.edt(clearance.obstacles(hn, RES, CFG["trav"]["step_max"]), RES)
    cost = clearance.block_cost(np.ones((50, 50)), D, CFG)
    ex = mission.Executive(CFG, (0.15, -3.0))
    mapped = np.zeros((100, 100), bool)
    ex.update(1.0, (0.15, -3.0, 0.0), cost, mapped, False)
    ex.goal = (0.15, 3.0)
    path = ex.update(2.0, (0.15, -3.0, 0.0), cost, mapped, False)
    assert path
    path = clearance.refine(path, D, CFG)
    through = [p for p in path if abs(p[1]) < 0.2]  # inside the wall's thickness
    assert through and all(abs(p[0] - 0.15) < 0.06 for p in through)
    # and a straight run-up on the door axis before it (no turning inside the doorway)
    run_up = [p for p in path if -0.95 < p[1] < -0.25]
    assert run_up and all(abs(p[0] - 0.15) < 0.08 for p in run_up)


def test_pursue_reverses_instead_of_turning_on_the_spot_when_asked():
    # regression (ruins 50 m training seed 2): a target behind the robot next to a wall wedged a foot while it turned on the spot
    from autonomy import planner

    lim = {"v_max": 1.5, "wz_max": 1.5}
    path = [(-1.0, 0.0), (-2.0, 0.0)]
    assert planner.pursue((0.0, 0.0, 0.0), path, CFG, lim) == (0.0, 1.5)
    v, wz = planner.pursue((0.0, 0.0, 0.0), path, CFG, lim, 0.45, 0.3)
    assert v == pytest.approx(-0.3) and abs(wz) < 1e-9
    v, wz = planner.pursue(
        (0.0, 0.0, 0.0), [(1.0, 0.0)], CFG, lim, 0.45, 0.3
    )  # ahead: unchanged
    assert v > 0


def test_refine_does_not_move_waypoints_in_open_ground():
    D = np.full((N, N), 5.0)
    path = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.6, 0.0)]
    assert np.allclose(clearance.refine(path, D, CFG), path, atol=1e-9)
