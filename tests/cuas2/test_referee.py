"""Referee2's per-ball arm-session verdict (examples/cuas2/run.py ball_unarmed): a ball is judged by the arm session of ITS OWN FIRE id.
Cases A-G; F is the one the old 'last FIRE sent before the feed' rule got wrong."""

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "cuas2"))
_spec = importlib.util.spec_from_file_location(
    "cuas2_run_ref", ROOT / "examples" / "cuas2" / "run.py"
)
run = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(run)
sys.path.remove(str(ROOT / "examples" / "cuas2"))

INF = float("inf")

# name: (arm sessions [[t_arm, t_disarm]], {fire_id: session}, [(feed t, fire_id, expected unarmed)])
CASES = {
    "A_normal": ([[0, INF]], {1: 0}, [(5.0, 1, False)]),
    "B_no_fire": ([[0, INF]], {}, [(5.0, 1, True)]),
    "C_after_disarm": ([[0, 4.0]], {1: 0}, [(5.0, 1, True), (3.9, 1, False)]),
    # the recorded disarm moved 0.5 s early: the same ball (fed 4.2, disarm really at 4.5) is now outside the session
    "D_disarm_0.5s_early": ([[0, 4.0]], {1: 0}, [(4.2, 1, True)]),
    "D_control_disarm_on_time": ([[0, 4.5]], {1: 0}, [(4.2, 1, False)]),
    # session-0 FIRE 1, quick re-arm at 4.1, ball fed at 4.2 inside session 1 but sent in session 0
    "E_quick_rearm": ([[0, 4.0], [4.1, INF]], {1: 0}, [(4.2, 1, True)]),
    # F: as E, and session 1 already sent its own FIRE 2 before the feed of ball 1: ball 1 is still unarmed, ball 2 is fine
    "F_old_fire_ball_after_new_fire": (
        [[0, 4.0], [4.1, INF]],
        {1: 0, 2: 1},
        [(4.2, 1, True), (4.3, 2, False)],
    ),
    # G: ball 1 fed in session 0 before session 1's FIRE 2 is sent (and before the re-arm): armed; ball 2 later in session 1: armed
    "G_feed_before_second_fire": (
        [[0, 4.0], [4.1, INF]],
        {1: 0, 2: 1},
        [(3.95, 1, False), (4.3, 2, False)],
    ),
}


@pytest.mark.parametrize("name", CASES)
def test_ball_unarmed_cases(name):
    iv, sess, balls = CASES[name]
    for t, fid, want in balls:
        assert run.ball_unarmed(iv, sess, t, fid) is want, (name, t, fid)
