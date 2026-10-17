import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import numpy as np
import pytest
from autonomy import config, traverse

CFG = config.load()
RES, K = 0.05, 10  # 0.5 m blocks of 5 cm cells


def grid(n=40):
    i = np.arange(n) * RES
    return np.meshgrid(i, i)  # x (columns), y (rows)


def test_slope_threshold():
    x, y = grid()
    for deg, lethal in ((20, False), (35, True)):
        h = x * math.tan(math.radians(deg))
        L = traverse.layers(h, RES, K)
        assert np.degrees(L["slope"]) == pytest.approx(np.full((4, 4), deg), abs=0.01)
        assert traverse.lethal(L, CFG).all() == lethal


def test_step_threshold_also_across_a_block_border():
    x, _ = grid()
    for dz, lethal in ((0.15, False), (0.20, True)):
        h = np.where(
            x >= 1.0, dz, 0.0
        )  # step between columns 19 and 20: the border of blocks 1 and 2
        L = traverse.layers(h, RES, K)
        assert (
            L["step"][:, 1] == pytest.approx(dz)
            and traverse.lethal(L, CFG)[:, 1].all() == lethal
        )


def test_roughness_threshold():
    x, y = grid()
    for amp, lethal in ((0.03, False), (0.20, True)):
        h = (
            amp * np.cos(2 * math.pi * x / 0.5) * np.cos(2 * math.pi * y / 0.5)
        )  # egg-crate: no plane component in a block
        L = traverse.layers(h, RES, K)
        assert L["rough"] == pytest.approx(np.full((4, 4), amp / 2), rel=1e-6)
        assert (
            np.abs(L["slope"]).max() < 1e-9
            and L["step"].max() < CFG["trav"]["step_max"]
        )
        assert traverse.lethal(L, CFG).all() == lethal


def test_unknown_block_and_near_lethal_cost():
    h = np.zeros((60, 60))
    h[:10, :10] = np.nan  # block (0, 0) unseen
    h[20:30, 20:30] = 1.0  # a 1 m box filling block (2, 2)
    c = traverse.cost(traverse.layers(h, RES, K), CFG)
    T = CFG["trav"]
    assert c[0, 0] == T["unknown_cost"]
    # the jumps at the 19|20 borders belong to the lower blocks (2, 1) and (1, 2); the 29|30 ones to (2, 2) itself
    assert math.isinf(c[2, 2]) and math.isinf(c[2, 1]) and math.isinf(c[1, 2])
    assert c[2, 3] == 1.0 + T["near_lethal"] and c[3, 3] == 1.0 + T["near_lethal"]
    assert c[5, 5] == 1.0


def test_window_update_equals_full_recompute():
    rng = np.random.default_rng(0)
    h = 0.02 * rng.standard_normal((200, 200))
    t = traverse.Traversability(CFG, 200)
    t.update(h)
    h2 = h.copy()
    h2[57:83, 101:140] += 0.5  # a new boulder
    win = t.update(h2, window=(57, 83, 101, 140))
    full = traverse.Traversability(CFG, 200).update(h2)
    assert np.isinf(full).any()
    np.testing.assert_allclose(
        win, full, rtol=1e-12
    )  # same blocks, same arithmetic; inf where full has inf


def test_reachable_respects_walls_and_corners():
    p = np.ones((7, 7), bool)
    p[2, 2:5] = p[4, 2:5] = p[2:5, 2] = p[2:5, 4] = False  # a closed ring around (3, 3)
    r = traverse.reachable(p, (0, 0))
    assert not r[3, 3] and r[6, 6]
    q = np.ones((3, 3), bool)
    q[0, 1] = q[1, 0] = False  # diagonal (0, 0) -> (1, 1) would cut two corners
    q[1, 2] = q[2, 1] = False
    assert not traverse.reachable(q, (0, 0))[1, 1]
