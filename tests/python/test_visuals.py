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
