"""cuas2 and cuas-lite must agree on the ball (spec 0 / 12 MECH acceptance 5): both CAD param files are read by path."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load(rel, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def clean_env(monkeypatch):
    for k in (
        "CUAS_BALL_D",
        "CUAS_BALL_M",
        "CUAS2_BALL_D",
        "CUAS2_WHEEL_D",
        "CUAS2_GATE_SWING_DEG",
        "CUAS2_POCKET_D_BALLS",
    ):
        monkeypatch.delenv(k, raising=False)


def test_ball_matches_lite(clean_env):
    c2 = _load("cad/cuas2/params.py", "cuas2_params")
    lite = _load("cad/cuas-lite/params.py", "cuas_lite_params")
    assert c2.BALL_D == lite.BALL_D == 22.2
    assert c2.BALL_M == lite.BALL_M == 1.86


def test_env_override_changes_cuas2(clean_env, monkeypatch):
    # the negative-control hook works: the override moves BALL_D (so the equality test above can fail)
    monkeypatch.setenv("CUAS2_BALL_D", "23.0")
    assert _load("cad/cuas2/params.py", "cuas2_params_neg").BALL_D == 23.0
