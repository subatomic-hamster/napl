"""Monte Carlo smoke on the napl kernel (examples/cuas-lite/mc.py): a few seeds through the worker pool, the episodes are deterministic per seed, no crashed episode and no
safety violation, a moving quad (Mover) and the walking-person stress episode included."""

import json
import sys
from pathlib import Path

import pytest

EX = Path(__file__).resolve().parents[2] / "examples" / "cuas-lite"
CLASH = ("mc", "scene", "run")  # example-module names other examples also use
mc = scene = None


@pytest.fixture(scope="module", autouse=True)
def _lite_modules():
    """Other examples have their own mc.py / scene.py / run.py, and an earlier test in the same xdist worker may have imported them: resolve these
    names to examples/cuas-lite for this module's tests only (the worker pool forks with them)."""
    global mc, scene
    saved = {n: sys.modules.pop(n) for n in CLASH if n in sys.modules}
    sys.path.insert(0, str(EX))
    mc = pytest.importorskip("mc")
    import scene as lite_scene

    scene = lite_scene
    yield
    sys.path.remove(str(EX))
    for n in CLASH:
        sys.modules.pop(n, None)
    sys.modules.update(saved)


@pytest.fixture(scope="module")
def root(tmp_path_factory):
    scene.cached_base()  # the turret is composed once (minutes the first time, cached under build/ after); the episodes copy it
    return tmp_path_factory.mktemp("mc")


def test_mc_smoke_few_seeds(root):
    md = root / "r.md"
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
                str(root),
                "--md",
                str(md),
            ]
        )
    assert e.value.code == 0, md.read_text() if md.exists() else "no table"
    res = json.loads((root / "mc.json").read_text())
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


def test_episode_is_deterministic(root):
    name, kw, seed = mc.draw("balloon", "5-10", 0)
    a, b = (
        mc.episode(
            ("balloon/5-10", name, kw, seed, str(root), str(scene.cached_base()))
        )
        for _ in range(2)
    )
    assert "error" not in a, a
    assert a["state_hash"] == b["state_hash"] and a["shot_t"] == b["shot_t"]


def test_wilson_interval():
    lo, hi = mc.wilson(0, 10)
    assert (
        lo == 0.0 and 0.25 < hi < 0.32
    )  # Wilson upper bound for 0 / 10 at 95 %: 0.2775
    lo, hi = mc.wilson(50, 100)
    assert 0.40 < lo < 0.41 and 0.59 < hi < 0.60
