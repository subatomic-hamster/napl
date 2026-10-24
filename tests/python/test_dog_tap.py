import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
sys.path.insert(0, str(ROOT / "tools"))

import napl
from autonomy import scene, videotap

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
SMOKE = (
    ROOT
    / "examples"
    / "dog"
    / "missions"
    / "coverage_smoke"
    / "coverage_smoke.scenario.toml"
)  # carrier: no SP4 needed


class FakeMap:
    n = 8

    def mapped(self, pool):
        return np.eye(4, dtype=bool)

    def height(self, x, y):
        return np.zeros(np.shape(x))


class FakeRob:
    map, path = FakeMap(), [(0.0, 0.0), (1.0, 0.0)]
    loc = type("L", (), {"x": np.array([1.0, 2.0, 0.3, 0.4])})()
    ex = type("E", (), {"phase": 1})()


class FakeCar:  # a carrier: no .ctl, the tap must store nan for the gait columns
    def truth(self):
        return np.array([1.01, 2.02, 0.3]), np.array([1.0, 0, 0, 0])

    time_of_last_frame = 0.1


def test_tap_stores_nan_for_missing_gait_and_keeps_the_path(tmp_path):
    tap = videotap.Tap(tmp_path, every=1, map_every_s=0.0)
    tap(0, FakeCar(), FakeRob())
    tap.close(state_hash="abc", dt=0.1)
    z = np.load(tmp_path / "tap.npz")
    assert (
        z["est_xy"].shape == (1, 2)
        and np.isnan(z["contact"]).all()
        and z["path_len"].tolist() == [2]
    )
    assert json.loads((tmp_path / "tap.json").read_text())["state_hash"] == "abc"


@needs_kernel
def test_tap_does_not_change_the_run(tmp_path, monkeypatch, unsandboxed_scenario):  # Review Focus 3
    from napl import program

    scenario = unsandboxed_scenario(SMOKE)
    base, _ = program.run(scenario, tmp_path / "a")
    monkeypatch.setenv("DOG_TAP_DIR", str(tmp_path / "tap"))
    tapped, _ = program.run(scenario, tmp_path / "b")
    assert base["sim"]["state_hash"] == tapped["sim"]["state_hash"]
    assert (tmp_path / "tap" / "tap.npz").exists()
    assert (
        json.loads((tmp_path / "tap" / "tap.json").read_text())["state_hash"]
        == tapped["sim"]["state_hash"]
    )


def test_summary_has_fell_ever_even_if_the_last_row_is_standing():
    rows = [[0.0] * len(scene.COLS) for _ in range(3)]
    rows[1][scene.COLS.index("fallen")] = 1.0
    assert scene.fell_ever(rows) == 1 and scene.fell_ever(rows[:1]) == 0
