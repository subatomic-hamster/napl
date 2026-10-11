"""Monte Carlo smoke on the napl kernel (examples/cuas-lite/mc.py): a few seeds through the worker pool, the episodes are deterministic per seed, no crashed episode and no
safety violation, a moving quad (Mover) and the walking-person stress episode included."""

import json
import sys
from pathlib import Path

import pytest

EX = Path(__file__).resolve().parents[2] / "examples" / "cuas-lite"
sys.path.insert(0, str(EX))

mc = pytest.importorskip("mc")


def test_mc_smoke_few_seeds(tmp_path):
    md = tmp_path / "r.md"
    with pytest.raises(SystemExit) as e:
        mc.main(
            [
                "--runs",
                "1",
                "--kinds",
                "balloon",
                "quad8",
                "--bands",
                "5-10",
                "--person-runs",
                "1",
                "--workers",
                "2",
                "--out",
                str(tmp_path / "mc"),
                "--md",
                str(md),
            ]
        )
    assert e.value.code == 0, md.read_text() if md.exists() else "no table"
    res = json.loads((tmp_path / "mc" / "mc.json").read_text())
    assert [r["cell"] for r in res] == ["balloon/5-10", "quad8/5-10", "person"]
    assert all("error" not in r for r in res), res
    assert sum(r["n_violations"] for r in res) == 0
    assert any(r["shots"] for r in res), (
        "nothing was ever fired: the smoke would be vacuous"
    )
    txt = md.read_text()
    assert (
        "safety violations over all episodes: 0" in txt and "hit/ball [95% CI]" in txt
    )


def test_episode_is_deterministic(tmp_path):
    name, kw, seed = mc.draw("balloon", "5-10", 0)
    job = lambda d: ("balloon/5-10", name, kw, seed, str(d))
    for d in ("a", "b"):
        (tmp_path / d).mkdir()
    a, b = mc.episode(job(tmp_path / "a")), mc.episode(job(tmp_path / "b"))
    assert "error" not in a, a
    assert a["state_hash"] == b["state_hash"] and a["shot_t"] == b["shot_t"]


def test_wilson_interval():
    lo, hi = mc.wilson(0, 10)
    assert (
        lo == 0.0 and 0.25 < hi < 0.32
    )  # Wilson upper bound for 0 / 10 at 95 %: 0.2775
    lo, hi = mc.wilson(50, 100)
    assert 0.40 < lo < 0.41 and 0.59 < hi < 0.60
