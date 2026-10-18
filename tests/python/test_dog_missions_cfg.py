import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
sys.path.insert(0, str(ROOT / "tools"))

from autonomy import scene

MIS = ROOT / "examples" / "dog" / "missions"
FAMS = {
    "fam_rubble": "rubble",
    "fam_slopes": "slopes",
    "fam_scree": "scree",
    "fam_ruins": "ruins",
}


@pytest.mark.parametrize("name,kind", FAMS.items())
def test_family_scenarios_are_30m_dog_runs_with_numbers_rule(name, kind):
    auto, prm, sc = scene.scenario(
        MIS / name / f"{name}.scenario.toml"
    )  # config.leaves refuses a bare number
    assert auto["kind"] == kind and auto["body"] == "dog" and auto["size"] == 30.0
    assert auto["rays"] <= 5000 and auto["patch"] in (
        5.0,
        10.0,
    )  # sim budget: lidar dominates (autonomy.md)
    assert prm["seed"] >= 1 and sc["name"] == f"dog-{name.replace('_', '-')}"
    assert {m["name"] for m in sc["monitors"]} >= {"never_falls", "map_rms_within_5cm"}


def test_dog_ruins_headline_is_100m_with_fine_blocks():
    auto, _, sc = scene.scenario(MIS / "dog_ruins" / "dog_ruins.scenario.toml")
    assert auto["size"] == 100.0 and auto["block"] == 0.3 and auto["t_budget"] == 1200.0
    assert auto["body"] == "dog"
    assert any(m["name"] == "covers_95_percent" for m in sc["monitors"])


@pytest.mark.parametrize("name", [*FAMS, "dog_ruins"])
def test_scenario_parses_and_names_its_script(name):
    sc = tomllib.loads((MIS / name / f"{name}.scenario.toml").read_text())
    assert (MIS / name / sc["model"]["script"]).exists()
