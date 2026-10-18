"""SP4 Task 8: terrain. Footholds off step edges and holes, swing apex over a step, posture plane on a 25 deg ramp (pure math on synthetic maps);
the criterion course (25 deg ramp + 15 cm steps, spec s7) and truth routes; the stairs and rubble scenarios with golden hashes."""

import math
import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C
from loco import model as M
from loco import mpc as Mp
from loco import terrain as Tr
from loco import wbc as Wb

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = {
    "stairs": None,
    "rubble": None,
}  # a scenario that does not pass has no golden: see the xfail below


class StepMap:  # 15 cm step up at x = 0.30, a hole at x in [0.6, 0.7]
    def height(self, x, y):
        x = np.asarray(x, float)
        h = np.where(x > 0.30, 0.15, 0.0)
        return np.where((x > 0.6) & (x < 0.7), np.nan, h) + 0.0 * np.asarray(y, float)


class Ramp:
    def height(self, x, y):
        return np.tan(math.radians(25)) * np.asarray(x, float) + 0.0 * np.asarray(
            y, float
        )


def test_foothold_keeps_off_step_edges_and_holes():
    p = Tr.foothold(StepMap(), np.array([0.31, 0.0]), 0.0175)
    assert abs(p[0] - 0.30) > 0.04 and p[2] - 0.0175 == pytest.approx(
        StepMap().height(p[0], p[1])
    )
    assert not 0.6 <= Tr.foothold(StepMap(), np.array([0.65, 0.0]), 0.0175)[0] <= 0.7
    flat = Tr.foothold(Tr.FlatMap(), np.array([1.0, 2.0]), 0.0175)
    assert flat == pytest.approx([1.0, 2.0, 0.0175])


def test_swing_clears_a_step_and_posture_follows_a_ramp():
    assert (
        Tr.apex_over(
            StepMap(), np.array([0.1, 0, 0.0175]), np.array([0.5, 0, 0.1675]), 0.08
        )
        >= 0.05 - 1e-9
    )
    assert Tr.apex_over(
        StepMap(), np.array([0.1, 0, 0.0175]), np.array([0.25, 0, 0.0175]), 0.08
    ) == pytest.approx(0.08)
    roll, pitch, _ = Tr.plane(Ramp(), np.array([0.0, 0.0]), 0.0)
    assert (
        math.degrees(-pitch) == pytest.approx(25.0, abs=0.5) and abs(roll) < 1e-6
    )  # nose-up pitch is negative about +y
    assert Tr.rough_ahead(
        StepMap(), np.array([0.0, 0.0]), 0.0, 0.8
    ) and not Tr.rough_ahead(Ramp(), np.array([0.0, 0.0]), 0.0, 0.8)


def test_ground_normal_tilts_the_friction_cone_and_the_swing_apex_is_capped():
    n = Tr.normal(Ramp(), np.array([0.0, 0.0]))
    assert n == pytest.approx(
        [-math.sin(math.radians(25)), 0.0, math.cos(math.radians(25))], abs=1e-6
    )
    assert Tr.normal(Tr.FlatMap(), np.array([0.0, 0.0])) == pytest.approx(
        [0, 0, 1]
    ) and Tr.normal(StepMap(), np.array([0.30, 0.0])) == pytest.approx(
        [0, 0, 1]
    )  # a riser: up
    rows = Mp.pyramid(0.5, n)
    assert rows[4] == pytest.approx(n) and np.all(
        rows[:4] @ n == pytest.approx(-0.5 * np.ones(4))
    )  # every edge of the pyramid leans mu off the NORMAL
    assert Mp.pyramid(0.5) == pytest.approx(Mp.pyramid(0.5, np.array([0.0, 0.0, 1.0])))
    wall = type("Wall", (), {"height": lambda s, x, y: np.full_like(x, 0.5, float)})()
    assert Tr.apex_over(
        wall, np.array([0.0, 0.0, 0.0175]), np.array([0.3, 0.0, 0.0175]), 0.05
    ) == pytest.approx(Tr.APEX_MAX)


def test_steep_ahead_flags_a_ramp_and_not_flat_ground():
    here = np.array([0.0, 0.0])
    assert Tr.steep_ahead(Ramp(), here, 0.0, 0.8) and not Tr.steep_ahead(
        Tr.FlatMap(), here, 0.0, 0.8
    )
    assert not Tr.steep_ahead(Ramp(), here, math.pi / 2, 0.8)  # across the slope: level


def test_criterion_course_has_the_spec_ramp_and_steps():
    f = C.criterion_course()
    x = np.arange(-14.0, 14.0, 0.05)
    z = f.height(x, np.zeros_like(x))
    slopes = np.degrees(np.arctan(np.diff(z) / 0.05))
    ramp = slopes[(slopes > 5) & (slopes < 40)]
    assert ramp.max() == pytest.approx(25.0, abs=0.5)
    risers = np.diff(z)[np.abs(np.diff(z)) > 0.1]
    assert sorted(np.round(np.abs(risers), 3)) == [0.15] * 4


@pytest.mark.parametrize("kind", ["rubble", "slopes", "scree", "ruins"])
def test_truth_routes_stay_on_traversable_ground(kind):
    f = C._field(kind, 7)
    r = C.route(f, np.random.default_rng([7, 31]))
    seg = np.linalg.norm(np.diff(r, axis=0), axis=1)
    assert 15.0 <= seg.sum() <= 40.0 and seg.max() <= 0.75
    xs = np.concatenate([np.linspace(a, b, 6) for a, b in zip(r[:-1], r[1:])])
    h = f.height(xs[:, 0], xs[:, 1])
    assert np.abs(np.diff(h)).max() <= 0.20  # never through a boulder / wall


def test_follower_walks_a_route_and_stops():
    r = np.column_stack([np.arange(0.0, 5.01, 0.5), np.zeros(11)])
    fol = C.Follower(r, 1.0)
    pos = np.array([0.0, 0.0])
    for _ in range(3000):
        vx, vy, wz = fol.cmd(pos, 0.0)
        pos = pos + [vx * 0.002, 0.0]
    assert fol.done and fol.length == pytest.approx(5.0) and fol.dist < 0.5
    assert (
        4.0 < fol.t_done < 6.0
    )  # 4.5 m at 1 m/s (slower at the start of the slew-free pursuit)
    assert fol.cmd(pos, 0.0) == (0.0, 0.0, 0.0)


@needs_kernel
def test_trot_climbs_a_15_degree_ramp_and_stands_on_the_plateau(tmp_path):
    """The capability that holds (docs/dog/locomotion.md "Terrain"): a 15 deg ramp climbed at 0.5 m/s to the plateau, no fall, upright."""
    import dogwalk as W

    C._field.cache_clear()
    C.RAMP_DEG = 15.0
    try:
        model = C.scene(tmp_path, "criterion", seed=1)
        r = W.run(
            model,
            dict(W.DEFAULTS, seed=1, course="criterion", v_cmd=0.5, duration=17.0),
            log=False,
        )
    finally:
        C.RAMP_DEG = 25.0
        C._field.cache_clear()
    x = np.array([row[W.COLS.index("x")] for row in r["rows"]])
    tilt = np.array([row[W.COLS.index("tilt")] for row in r["rows"]])
    assert not r["res"]["fallen"] and x[-1] > -5.5 and tilt.max() < 20.0, (
        x[-1],
        tilt.max(),
    )


@needs_kernel
@pytest.mark.xfail(
    strict=True,
    reason="OPEN (docs/dog/locomotion.md 'Terrain'): the dog does not climb the 25 deg ramp (it climbs 15 deg) and does not cross the rubble field",
)
@pytest.mark.parametrize("name", ["stairs", "rubble"])
def test_scenario_passes_replays_and_matches_golden(tmp_path, name):
    res, _ = program.run(EX / "walk" / f"{name}.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    if isinstance(Wb.make(M.from_build(C.base_model())), Wb.Wbc):
        pytest.skip("goldens were recorded with the C++ WBC")
    assert GOLDEN[name] is None or res["sim"]["state_hash"] == GOLDEN[name], res["sim"][
        "state_hash"
    ]
