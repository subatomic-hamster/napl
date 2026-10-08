"""Library thumbnails on demand (python/napl/design/thumbjob.py): the Backend starts tools/thumbs.py in the background when the
cache lacks parts, `rev` changes as PNGs land, the library then finds them; the tile brightening lifts dark parts. A stand-in
script plays tools/thumbs.py (no GPU needed here); the real one runs in the screenshot tools on the build host."""

import importlib.util
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from napl.design import Design, uilib
from napl.design.library import Library
from napl.design.thumbjob import VERSION, ThumbJob

REPO = Path(__file__).resolve().parents[2]
FAKE = """
import argparse, json, sys
from pathlib import Path
ap = argparse.ArgumentParser()
ap.add_argument("--out"); ap.add_argument("--parts"); ap.add_argument("--priority", default="")
ap.add_argument("--missing", action="store_true"); ap.add_argument("--quiet", action="store_true")
a = ap.parse_args()
out = Path(a.out)
ids = sorted(p.stem for p in Path(a.parts).rglob("*.toml"))
first = a.priority.split(",") if a.priority else []
ids = first + [i for i in ids if i not in first]
idx = {}
for i in ids:
    (out / f"{i}.png").write_bytes(b"\\x89PNG fake")
    idx[i] = {"thumb": f"{i}.png"}
idx["_v"] = %d
(out / "index.json").write_text(json.dumps(idx))
"""


def mini_repo(tmp_path):
    parts = tmp_path / "parts"
    shutil.copytree(REPO / "parts" / "common", parts / "common")
    (tmp_path / "tools").mkdir()
    (tmp_path / "tools" / "thumbs.py").write_text(FAKE % VERSION)
    return Library(parts)


def test_missing_thumbnails_are_rendered_in_the_background_and_found(tmp_path):
    lib = mini_repo(tmp_path)
    some = sorted(lib.parts)[0]
    assert uilib.thumb_path(lib, some) is None  # empty cache: placeholders
    job = ThumbJob(lambda: lib, lambda: [some])
    assert job.missing(lib) == sorted(lib.parts) or set(job.missing(lib)) == set(
        lib.parts
    )
    assert job.start()
    assert job.wait(60)
    assert job.missing(lib) == []
    assert job.rev > 0
    assert uilib.thumb_path(lib, some)  # the library's search now returns the file
    assert not job.start() and job.settled  # nothing missing: no second job


def test_stale_cache_version_is_regenerated(tmp_path):
    lib = mini_repo(tmp_path)
    d = tmp_path / "build" / "thumbs"
    d.mkdir(parents=True)
    (d / "index.json").write_text(
        json.dumps({sorted(lib.parts)[0]: {"thumb": None}, "_v": VERSION - 1})
    )
    assert set(ThumbJob.missing(lib)) == set(lib.parts)


def test_backend_polls_start_the_job_only_when_enabled(tmp_path, monkeypatch):
    from napl.design.viewer import Backend

    d = Design.open(REPO / "assemblies" / "x500v2.toml")
    b = Backend(lambda: d, tmp_path)
    started = []
    monkeypatch.setattr(b.thumbs, "start", lambda: started.append(1) or False)
    b(json.dumps({"tool": "ui_poll"}))
    assert not started  # tests and headless agents never spawn the GPU job
    b.auto_thumbs = True
    b(json.dumps({"tool": "ui_poll"}))
    assert started
    assert "thumb_rev" in json.loads(b(json.dumps({"tool": "rev"})))["result"]


def test_dark_parts_are_brightened_but_light_ones_are_not():
    spec = importlib.util.spec_from_file_location(
        "thumbs_tool", REPO / "tools" / "thumbs.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules["thumbs_tool"] = mod
    spec.loader.exec_module(mod)

    def tile(v):
        rgba = np.full((8, 8, 4), 255, np.uint8)
        rgba[..., :3] = v
        return mod.finish(rgba, np.zeros((8, 8)), 4)[..., :3].mean() / 255.0

    assert tile(20) > 0.25  # carbon-black render (mean 0.08) lifted
    assert abs(tile(200) - 200 / 255) < 0.02  # bright parts untouched
