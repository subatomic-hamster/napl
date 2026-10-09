"""Cosmetic visual layer (python/napl/design/visuals, parts/README.md "Visual layer"): every generator yields closed
meshes within the triangle budget, and a build WITH visuals has the same physics as one without (report masses, replay
state hashes), for every assembly."""

import re
from pathlib import Path

import napl
import pytest
from napl.design import Library, visuals
from napl.design.build import compute
from napl.design.visuals.mesh import check_closed

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
ASSEMBLIES = sorted((REPO / "assemblies").glob("*.toml"))
BUDGET = 60000  # triangles per part: the renderer copies mesh vertices per geom
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def lib():
    return Library(PARTS)


def test_every_visual_is_closed_and_in_budget(lib):
    bad, total = [], 0
    for part in lib.parts.values():
        for e in visuals.elements(part):
            try:
                meshes = visuals.build_meshes(part, e, lib.root)
            except Exception as ex:
                bad.append(f"{part.id}: {type(ex).__name__}: {ex}")
                continue
            tris = sum(m.tri_count() for m in meshes.values())
            total += tris
            if tris > BUDGET:
                bad.append(f"{part.id}: {tris} triangles")
            g = part.geometry
            if "mesh" in e or any(
                s.get("shape") == "mesh" for s in g.get("shapes") or [g]
            ):
                continue  # vendor STLs are not ours to repair
            for slot, m in meshes.items():
                if m.tri_count() and (msg := check_closed(m)):
                    bad.append(f"{part.id}/{slot}: {msg}")
    assert not bad, "\n".join(bad[:1000])
    assert total > 0


@pytest.mark.parametrize("path", ASSEMBLIES, ids=lambda p: p.stem)
def test_visuals_do_not_change_physics(path, tmp_path, lib):
    xml_v, rep_v = compute(path, PARTS, tmp_path / "v", lib, visual=True)
    xml_p, rep_p = compute(path, PARTS, tmp_path / "p", lib, visual=False)
    assert rep_v["mass"] == rep_p["mass"]
    assert rep_v["instances"] == rep_p["instances"]
    gv = re.findall(r"<geom [^>]*/>", xml_v)
    # every plain geom is untouched but for its colour alpha; every extra geom is a zero-mass, non-colliding mesh
    plain = [g for g in gv if ":v" not in g.split()[1]]
    extra = [g for g in gv if ":v" in g.split()[1]]
    assert len(plain) == len(re.findall(r"<geom [^>]*/>", xml_p))
    assert all(
        'mass="0"' in g and 'contype="0"' in g and 'conaffinity="0"' in g for g in extra
    )
    assert extra, "assembly has no visual geoms"

    # stronger: with the visual layer stripped (extra geoms, assets, colours) the two models are the same text
    def physics_only(x):
        x = re.sub(r"<asset>.*?</asset>\s*", "", x, flags=re.DOTALL)
        x = re.sub(r"\s*<geom name=\"[^\"]*:v\d+\"[^>]*/>", "", x)
        # the cables are visual-only too (inert tendons + massless sites); their route avoids the visual meshes, so it differs
        x = re.sub(r"\s*<tendon>.*?</tendon>", "", x, flags=re.DOTALL)
        x = re.sub(r"\s*<site name=\"wire:[^>]*/>", "", x)
        return re.sub(r' rgba="[^"]*"', "", x)

    assert physics_only(xml_v) == physics_only(xml_p)


@pytest.mark.parametrize("path", ASSEMBLIES, ids=lambda p: p.stem)
def test_every_instance_in_every_assembly_is_drawn(path, tmp_path, lib):
    """Each onboard instance whose part has a visual (also a part without [geometry], e.g. the turret's Raspberry Pi 5) gets
    visual geoms; the only undrawn onboard instances are those of parts with nothing to draw."""
    xml, rep = compute(path, PARTS, tmp_path, lib, visual=True)
    undrawn = [
        f"{r['id']} ({r['part']})"
        for r in rep["instances"]
        if r.get("body")
        and f'name="{r["id"]}:v0"' not in xml
        and visuals.applies(lib.get(r["part"]))
    ]
    assert not undrawn, undrawn
    nothing = [
        r["part"]
        for r in rep["instances"]
        if r.get("body") and not visuals.applies(lib.get(r["part"]))
    ]
    assert not nothing, f"onboard instances of parts with no visual: {nothing}"


def test_geometry_less_part_is_drawn_in_the_build(tmp_path, lib):
    xml, _ = compute(ASSEMBLIES[0].parent / "turret.toml", PARTS, tmp_path, lib)
    assert 'name="pi:v0"' in xml and lib.get("raspberry-pi-5").geometry == {}


@needs_kernel
@pytest.mark.parametrize("path", ASSEMBLIES, ids=lambda p: p.stem)
def test_visual_build_replays_identically(path, tmp_path, lib):
    hashes = []
    for vis in (True, False):
        out = tmp_path / str(vis)
        out.mkdir()
        xml, rep = compute(path, PARTS, out, lib, visual=vis)
        mp = out / "model.xml"  # mesh paths in the MJCF are relative to its directory
        mp.write_text(xml)
        s = napl.Sim.load(mp)
        hs = []
        for _ in range(5):
            s.step(100)
            hs.append(s.state_hash())
        hashes.append(hs)
    assert hashes[0] == hashes[1]


@needs_kernel
def test_cuas_scene_visuals_do_not_change_physics(tmp_path):
    """The composed counter-UAS scene (turret + x500v2) replays with identical state hashes with and without the visual layer."""
    from napl.cuas.scene import Environment, Vehicle, compose

    hashes = []
    for vis in (True, False):
        vs = [
            Vehicle("turret", "turret", pos=(0, 0, 1.0), pedestal=1.0),
            Vehicle("drone1", "x500v2", pos=(30, 0, 12), yaw=0),
        ]
        sc = compose(
            tmp_path / str(vis),
            vs,
            Environment(wind=(2.0, -1.0, 0.0)),
            seed=3,
            visual=vis,
        )
        s = napl.Sim.load(sc["model"])
        hs = []
        for _ in range(5):
            s.step(100)
            hs.append(s.state_hash())
        hashes.append(hs)
    assert hashes[0] == hashes[1]


def test_parts_are_95_percent_covered(lib):
    """Every part that is an object (all but raw materials, OCV curves, wires, frame-kit data records) draws a purpose-built
    visual in a build: explicit [visual], family default, or a vendor mesh redrawn; also parts without [geometry]. The `shapes`
    fallback on bare primitives counts as uncovered."""
    import sys

    sys.path.insert(0, str(REPO / "tools"))
    from visual_coverage import classify

    kinds = [classify(p) for p in lib.parts.values()]
    obj = [k for k in kinds if k != "exempt"]
    cov = sum(k in ("explicit", "family") for k in obj) / len(obj)
    assert cov >= 0.95, f"{cov:.1%} of parts have a visual"
    assert "missing" not in obj, "a part without [geometry] has nothing to draw"
