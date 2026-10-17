"""Scenes of the dog's locomotion runs (SP4, HARNESS side): the nominal dog built once and cached, then per run a scene = the nominal MJCF
with the lidar and depth camera removed (sim budget, docs/models.md "Ground sim budget"), a course (flat floor / terrain field), the start
pose and an optional dispersion payload. course.json beside scene.xml records what the harness needs (the nominal model is what the
CONTROLLER's model is built from: it never sees the payload or the terrain)."""

import hashlib
import json
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import dogsim as D

REPO = Path(__file__).resolve().parents[2]
CACHE = Path(
    os.environ.get("NAPL_DOG_CACHE", Path(tempfile.gettempdir()) / "napl-dog-build")
)


def _key():
    h = hashlib.sha256()
    for f in sorted(
        [
            *(REPO / "assemblies" / "dog").glob("*.toml"),
            *(REPO / "parts").rglob("*.toml"),
        ]
    ):
        h.update(f.read_bytes())
    return h.hexdigest()[:16]


def base_model():
    """Nominal dog MJCF (dogsim.build), cached by the content of assemblies/dog and parts; built in a temp dir and renamed into place, so
    concurrent MC workers never read a half-written build."""
    d = CACHE / _key()
    xml = d / "dog.xml"
    if not xml.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(dir=CACHE))
        x, rep = D.build(tmp)
        (tmp / "dog.report.json").write_text(json.dumps(rep))
        tree = ET.parse(x)
        for e in tree.getroot().iter():  # the builder writes mesh paths relative to its own xml: absolute, so scenes anywhere can use them
            if e.get("file"):
                f = (Path(x).parent / e.get("file")).resolve()
                if f.is_relative_to(
                    tmp.resolve()
                ):  # the builder's own visuals/ dir moves with the rename below
                    f = CACHE.resolve() / d.name / f.relative_to(tmp.resolve())
                e.set("file", str(f))
        tree.write(tmp / "dog.xml")
        if Path(x) != tmp / "dog.xml":
            Path(x).unlink()
        try:
            tmp.rename(d)
        except OSError:  # another worker won the race: same content
            shutil.rmtree(tmp, ignore_errors=True)
    return xml


def strip_lidar(root):
    t = root.find("custom/text[@name='napl.devices']")
    d = json.loads(t.get("data"))
    d["rangefinders"] = []
    t.set("data", json.dumps(d))


def scene(out, course, seed=1, payload=None):
    """-> scene.xml path in `out`. course: "flat" (Task 8 adds criterion, slip, rubble, slopes, scree, ruins). payload = (kg, dx, dy) or None."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    nominal = base_model()
    root = ET.parse(nominal).getroot()
    strip_lidar(root)
    inf = dict(
        course=course,
        seed=int(seed),
        nominal=str(nominal),
        start=[0.0, 0.0, 0.0],
        route=None,
        field=None,
    )
    if course != "flat":
        raise ValueError(f"course {course!r}: Task 8")
    if payload:
        _add_payload(root, *payload)
    path = out / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    (out / "course.json").write_text(json.dumps(inf))
    return path


def _add_payload(root, kg, dx, dy):
    trunk = next(b for b in root.iter("body") if b.get("name") == "deck_top")
    b = ET.SubElement(trunk, "body", name="payload", pos=f"{dx:.6g} {dy:.6g} 0.03")
    ET.SubElement(
        b,
        "geom",
        type="box",
        size="0.03 0.03 0.02",
        mass=f"{kg:.6g}",
        contype="0",
        conaffinity="0",
        group="1",
    )


def info(model):
    return json.loads((Path(model).parent / "course.json").read_text())


def field_of(inf):
    """The napl.terrain Field of a scene's course (None on the flat floor); Task 8 adds the terrain courses."""
    return


def build_for(scen):
    """-> build(out) of a walk scenario's [model] script: that scenario's scene at its parameter defaults."""
    from napl import params as nparams

    def build(out):
        p = nparams.resolve(scen)
        return str(scene(out, p["course"], p["seed"]))

    return build
