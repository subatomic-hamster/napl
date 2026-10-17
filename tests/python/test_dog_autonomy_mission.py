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
    "map_quality/map_quality": "93045ecb08d4d08f",
    "coverage_smoke/coverage_smoke": "8bc000837600ce26",
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
