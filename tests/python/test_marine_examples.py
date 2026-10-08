"""Phase 5 examples through `napl program`: the rocket hop and the boat float pass their monitors, the replay log reproduces the state hash, and a
second run ends on the same hash (needs napl-replay on PATH, as the other native-interface tests)."""

from pathlib import Path

import pytest
from napl import program

EX = Path(__file__).resolve().parents[2] / "examples"


@pytest.mark.parametrize("scenario", ["rocket/hop", "boat/float"])
def test_example_passes_and_repeats(scenario, tmp_path):
    runs = []
    for i in range(2):
        res, _ = program.run(EX / f"{scenario}.scenario.toml", tmp_path / str(i))
        bad = [m["name"] for m in res["monitors"] if not m["pass"]]
        assert not bad, bad
        assert res["replay"]["available"] and res["replay"]["match"], res["replay"]
        runs.append(res["sim"]["state_hash"])
    assert runs[0] == runs[1]
