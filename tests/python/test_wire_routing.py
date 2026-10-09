"""3D wire routing: the tubes keep clear of every part (no wire through a component), leave their connectors straight, are
deterministic, and cost nothing physically. Contract (python/napl/design/wireroute.py): clearance is promised outside the
lead-out (the first and last straight run along the connector axis, which starts inside the part that owns the connector)."""

import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design
from napl.design import drc, wireroute

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
WIRED = sorted(
    p.stem for p in (REPO / "assemblies").glob("*.toml") if "[[wires]]" in p.read_text()
)


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    class Lazy(dict):  # builds an assembly the first time a test asks for it
        def __missing__(self, n):
            wireroute._cache.clear()
            self[n] = design.build(
                REPO / "assemblies" / f"{n}.toml", PARTS, tmp_path_factory.mktemp(n)
            )
            return self[n]

    return Lazy()


def _dense(pts, step=0.0005):
    dense, sid = [pts[0]], [0]
    for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        n = max(1, int(np.linalg.norm(b - a) / step))
        dense += [a + (b - a) * t for t in np.linspace(0, 1, n + 1)[1:]]
        sid += [k] * n
    return np.array(dense), np.array(sid)


def test_there_are_wired_assemblies():
    assert "turret" in WIRED and "x500v2" in WIRED


@needs_kernel
@pytest.mark.parametrize("name", WIRED)
def test_no_tube_point_is_inside_a_part(built, name):
    xml = built[name]["xml"]
    geoms = wireroute.geoms_from_world(drc._World(xml.read_text(), xml.parent))
    names = [t.get("name") for t in ET.parse(xml).getroot().findall("./tendon/spatial")]
    tubes = napl.Sim.load(xml).tubes()
    assert len(names) == len(tubes) > 0
    worst = 1e9
    for nm, (radius, _, _, pts) in zip(names, tubes):
        dense, sid = _dense(pts)
        keep = (
            (np.linalg.norm(dense - pts[0], axis=1) > wireroute.END_ZONE)
            & (np.linalg.norm(dense - pts[-1], axis=1) > wireroute.END_ZONE)
            & (sid >= 2)
            & (sid < len(pts) - 3)
        )
        if keep.any():
            m = float((wireroute.clearance(dense[keep], geoms) - radius).min())
            assert m > 0, f"{name} {nm}: tube penetrates a part by {-m * 1000:.2f} mm"
            worst = min(worst, m)


@needs_kernel
@pytest.mark.parametrize("name", ("turret", "x500v2"))
def test_routed_tubes_leave_their_connectors_straight(built, name):
    xml = built[name]["xml"]
    names = [t.get("name") for t in ET.parse(xml).getroot().findall("./tendon/spatial")]
    stubs, n_checked = {}, 0
    for nm, (radius, _, _, pts) in zip(names, napl.Sim.load(xml).tubes()):
        if len(pts) < 7:
            continue  # the straight chord of a wire the router could not place
        for end, (a, b) in enumerate(((pts[0], pts[1]), (pts[-1], pts[-2]))):
            u = (b - a) / np.linalg.norm(b - a)
            k = nm.split(":")[1]
            # every conductor of a wire leaves its connector along the same axis
            assert (u @ stubs.setdefault((k, end), u)) > 0.99
        n_checked += 1
    assert n_checked > 0


@needs_kernel
def test_routing_is_deterministic(tmp_path):
    outs = []
    for k in range(2):
        wireroute._cache.clear()
        r = design.build(REPO / "assemblies" / "turret.toml", PARTS, tmp_path / str(k))
        outs.append([t for *_, t in napl.Sim.load(r["xml"]).tubes()])
    assert len(outs[0]) == len(outs[1]) > 0
    assert all(np.array_equal(a, b) for a, b in zip(*outs))


def test_fillet_rounds_a_corner_with_the_minimum_radius():
    class Free:
        p = 0.003

        def at(self, q):
            return 1.0

    R = 0.01
    pts = [np.zeros(3), np.array([0.1, 0.0, 0.0]), np.array([0.1, 0.1, 0.0])]
    out, bends = wireroute._fillet(Free(), pts, R, 0.001)
    assert bends == [R] and len(out) > 5
    assert np.allclose(out[0], pts[0]) and np.allclose(out[-1], pts[-1])
    arc = out[1:-1]  # a quarter circle of radius R around (0.09, 0.01)
    assert np.allclose(np.linalg.norm(arc - np.array([0.09, 0.01, 0.0]), axis=1), R)


def test_a_wall_between_the_ends_is_routed_around():
    wall = {
        "kind": "box",
        "size": [0.002, 0.04, 0.04],
        "T": np.eye(4),
        "owner": "wall",
        "name": "wall",
        "lo": np.array([-0.002, -0.04, -0.04]),
        "hi": np.array([0.002, 0.04, 0.04]),
    }
    item = {
        "key": 0,
        "a": np.array([[-0.05, 0.0, 0.0]]),
        "b": np.array([[0.05, 0.0, 0.0]]),
        "na": np.array([-1.0, 0.0, 0.0]),
        "nb": np.array([1.0, 0.0, 0.0]),
        "ua": np.array([0.0, 0.0, 1.0]),
        "ub": np.array([0.0, 0.0, 1.0]),
        "d": 0.002,
    }
    wireroute._cache.clear()
    r = wireroute.route_wires([wall], [item])[0]
    assert r["ok"]
    dense, _ = _dense(r["centre"])
    assert (wireroute.clearance(dense, [wall]) > 0.001).all()
