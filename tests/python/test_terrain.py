"""Terrain (python/napl/terrain.py, src/mujoco hfield + windfield.cpp): the seeded generator, pads, hfield contact and rays in the kernel, the
terrain-aware wind felt by a hovering multirotor, compose(terrain=...). CPU only; the rendering of terrain is in tests/cpp/test_render.cpp /
tests/python/test_render_terrain.py (needs a Vulkan device)."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design
from napl.cuas.scene import Environment, Vehicle, compose
from napl.design import simulate
from napl.terrain import (
    KINDS,
    PadSpec,
    WindField,
    add_terrain,
    add_terrain_xml,
    generate,
)

REPO = Path(__file__).resolve().parents[2]
BASE = """<mujoco model="t"><option timestep="0.002"/><worldbody>
<geom name="floor" type="plane" size="0 0 1"/>
<body name="b" pos="{x} {y} {z}"><freejoint/><geom type="box" size="0.1 0.1 0.05" mass="1" contype="1" conaffinity="1"/></body>
</worldbody></mujoco>"""
ANCHOR = {"ridge": "summit", "mesa": "brink", "ledge": "ledge", "moorland": "hilltop"}


def small(kind, **kw):
    kw.setdefault("pads", [PadSpec("p", at=ANCHOR[kind], radius=5)])
    return generate(
        kind, size=(1200, 900), n=193, relief=200, roughness=1.5, seed=7, **kw
    )


# ---- generator ----------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("kind", KINDS)
def test_generator_is_seeded_and_pads_are_flat_and_exact(kind):
    a, b = small(kind, rocks=12), small(kind, rocks=12)
    assert np.array_equal(a.heights, b.heights) and a.rocks == b.rocks
    c = generate(
        kind,
        size=(1200, 900),
        n=193,
        relief=200,
        roughness=1.5,
        seed=8,
        pads=[PadSpec("p", at=ANCHOR[kind])],
    )
    assert not np.array_equal(a.heights, c.heights)
    p = a.pad("p")
    assert p.z == 0.0  # first pad = the datum
    dx = a.size[0] / (a.nx - 1)
    X, Y = np.meshgrid(
        -a.size[0] / 2 + dx * np.arange(a.nx),
        -a.size[1] / 2 + a.size[1] / (a.ny - 1) * np.arange(a.ny),
    )
    inside = np.hypot(X - p.x, Y - p.y) <= p.flat_radius
    assert inside.sum() >= 9 and np.abs(a.heights[inside] - p.z).max() < 1e-9
    assert p.flat_radius >= p.radius + 1.4 * dx
    for r in a.rocks:  # boulders stay off the pads and sit on the surface
        assert np.hypot(r.x - p.x, r.y - p.y) > p.flat_radius
        assert abs(r.z - a.height_at(r.x, r.y)) < 1.0
    assert a.zmax - a.zmin > 0.25 * 200


def test_kind_shapes():
    r = small("ridge")
    assert (
        abs(r.anchors["summit"][2] - r.heights.max()) < 40
    )  # the summit anchor is on the high ground
    assert r.anchors["windward"][2] < r.anchors["summit"][2] - 100
    m = small("mesa")
    plateau, low = m.anchors["plateau"][2], m.anchors["lowland"][2]
    assert plateau - low > 0.7 * 200  # the plateau stands a relief above the lowland
    assert (
        float(
            m.slope_at(
                *np.meshgrid(np.linspace(-500, 500, 80), np.linspace(-400, 400, 40))
            ).max()
        )
        > 0.8
    )  # a cliff
    led = small("ledge")
    assert (
        led.anchors["summit"][2] - led.anchors["ledge"][2] > 30
    )  # the ledge is below the crest
    moor = small("moorland")
    assert (
        float(
            moor.slope_at(
                *np.meshgrid(np.linspace(-500, 500, 20), np.linspace(-400, 400, 20))
            ).max()
        )
        < 1.3
    )


def test_two_pads_and_explicit_xy():
    t = generate("ridge", size=(1200, 900), n=193, relief=200, seed=3,
                 pads=[PadSpec("summit", at="summit", radius=4), PadSpec("slope", at=(-200.0, 100.0), radius=6, blend=30)])  # fmt: skip
    assert t.pad("summit").z == 0.0 and t.pad("slope").z < -50
    assert (
        abs(t.height_at(t.pad("slope").x, t.pad("slope").y) - t.pad("slope").z) < 1e-9
    )


# ---- the kernel sees it -------------------------------------------------------------------------------------------------------
def load(xml, tmp_path, name="s.xml"):
    p = tmp_path / name
    p.write_text(xml)
    return napl.Sim.load(p)


def test_kernel_surface_matches_the_generator(tmp_path):
    """mj_ray from above lands within one cell's relief of the bilinear surface everywhere, and the y axis is not mirrored."""
    t = small("ridge")
    s = load(add_terrain_xml(BASE.format(x=0, y=0, z=5000), t, markers=False), tmp_path)
    rng = np.random.default_rng(0)
    err, mirror = [], []
    for x, y in zip(rng.uniform(-550, 550, 12), rng.uniform(-400, 400, 12)):
        d = s.raycast([x, y, 1000.0], [[0, 0, -1]])[0]
        cell = t.heights[max(0, int((y + 450) / 900 * (t.ny - 1)) - 1):int((y + 450) / 900 * (t.ny - 1)) + 2,
                         max(0, int((x + 600) / 1200 * (t.nx - 1)) - 1):int((x + 600) / 1200 * (t.nx - 1)) + 2]  # fmt: skip
        assert (
            abs((1000.0 - d) - t.height_at(x, y))
            <= 0.5 * (cell.max() - cell.min()) + 1e-3
        )
        err.append(abs((1000.0 - d) - t.height_at(x, y)))
        mirror.append(abs((1000.0 - d) - t.height_at(x, -y)))
    assert np.mean(mirror) > 10 * np.mean(
        err
    )  # the y axis is not mirrored (MuJoCo's data row 0 is +y)


@pytest.mark.parametrize("kind", KINDS)
def test_box_dropped_on_a_pad_rests_at_the_right_height(kind, tmp_path):
    t = small(kind)
    p = t.pad("p")
    s = load(add_terrain_xml(BASE.format(x=p.x, y=p.y, z=p.z + 1.0), t), tmp_path)
    s.step(1500)
    x = s.query("/world/b", "pos")[0]
    assert abs(x[2] - (p.z + 0.05)) < 0.01, (
        x
    )  # MuJoCo's soft hfield contact jitters by a few mm
    assert np.hypot(x[0] - p.x, x[1] - p.y) < 0.2


RF = """<mujoco model="rf"><option timestep="0.002" gravity="0 0 0"/><worldbody>
<body name="s" pos="{x} {y} {z}"><freejoint/><geom type="sphere" size="0.02" mass="1" contype="0" conaffinity="0" group="4"/>
<site name="eye" xyaxes="1 0 0 0 -1 0"/></body></worldbody><custom><text name="napl.devices" data='{{"environments":[{{"name":"air","origin_alt":3200}}],
"rangefinders":[{{"name":"rf","site":"eye","rate_hz":50,"projection":"spherical","fov_h":0.1,"fov_v":0.1,"n_h":1,"n_v":1,"min_range":0.1,"max_range":30,
"ray_groups":{groups}}}]}}'/></custom></mujoco>"""


@pytest.mark.parametrize("kind", KINDS)
def test_rangefinder_distance_to_the_pad_is_analytic(kind, tmp_path):
    """A downward rangefinder 5 m over the pad reads 5.000 m within 1 cm; the painted marker (group 4) is outside the vehicle ray mask, so it
    does not add its 13 mm, but a ray mask that includes group 4 sees it."""
    t = small(kind)
    p = t.pad("p")
    for groups, expect in ((47, 5.0), (63, 5.0 - 0.013)):
        xml = add_terrain_xml(RF.format(x=p.x, y=p.y, z=p.z + 5.0, groups=groups), t)
        s = load(xml, tmp_path, f"rf{groups}.xml")
        s.step(40)
        r = float(s.query("/devices/rf", "range")[0][0])
        assert abs(r - expect) < 0.01, (kind, groups, r)
        if groups == 47:
            assert abs(r - 5.0) < 0.002
    # the ground-height query of the wind field agrees: the pad is the datum
    assert t.height_at(p.x, p.y) == pytest.approx(p.z, abs=1e-9)


# ---- wind in the kernel -------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def x500():
    return design.compute(
        REPO / "assemblies" / "x500v2.toml",
        REPO / "parts",
        Path("/tmp/p11_test_x500"),
        visual=False,
    )


def hover_throttle(x500, pad, gain, tmp_path):
    text, report = x500
    t = generate("ridge", size=(1600, 1200), n=193, relief=250, roughness=1.0, seed=3,
                 pads=[PadSpec("summit", at="summit", radius=4), PadSpec("windward", at=(-200.0, 0.0), radius=6, blend=30),
                       PadSpec("lee", at=(200.0, 0.0), radius=6, blend=30)])  # fmt: skip
    root = ET.fromstring(text)
    b = root.find("worldbody").find("body")
    p0 = [float(v) for v in b.get("pos", "0 0 0").split()]
    P = t.pad(pad)
    b.set("pos", f"{p0[0] + P.x} {p0[1] + P.y} {p0[2] + P.z}")
    add_terrain(root, t, wind=WindField(ridge_gain=gain, z0=0.0), origin_alt=3200.0)
    path = tmp_path / f"{pad}_{gain}.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    r = simulate.hover(
        str(path),
        report,
        altitude=3.0,
        climb_s=3.0,
        hold_s=5.0,
        tol=5.0,
        hold_xy=True,
        wind=(10.0, 0, 0),
    )
    assert r["pass"], r.get("reason")
    return r["hover_throttle"], t


def test_skydio_hover_feels_the_lee_downdraft_only_with_the_terrain_wind(
    x500, tmp_path
):
    """x500v2 (the Skydio airframe) hovering on a pad on the lee face of a 250 m ridge in a 10 m/s wind: without the terrain wind the hover
    throttle is that of the flat uniform wind; with it the downdraft (+ lee separation) costs a clearly higher throttle (rotor thrust drops with
    the axial inflow of a downdraft); on the summit pad the vertical wind is small and the throttle barely moves."""
    lee0, t = hover_throttle(x500, "lee", 0.0, tmp_path)
    lee1, _ = hover_throttle(x500, "lee", 1.0, tmp_path)
    sum0, _ = hover_throttle(x500, "summit", 0.0, tmp_path)
    sum1, _ = hover_throttle(x500, "summit", 1.0, tmp_path)
    assert lee1 > lee0 * 1.10, (lee0, lee1)
    assert abs(sum1 - sum0) < 0.02 * sum0, (sum0, sum1)


def test_windfield_attributes_and_validation(tmp_path):
    t = small("ridge")
    xml = BASE.format(x=0, y=0, z=40)
    root = ET.fromstring(xml)
    ET.SubElement(root, "custom")
    ET.SubElement(root.find("custom"), "text", name="napl.devices",
                  data=json.dumps({"environments": [{"name": "air", "wind": [8, 0, 0], "ref_body": "b"}]}))  # fmt: skip
    ET.SubElement(
        root.find("worldbody").find("body"),
        "geom",
        type="sphere",
        size="0.01",
        contype="0",
        conaffinity="0",
        mass="0.1",
    )
    add_terrain(root, t, wind=WindField(), origin_alt=3000.0)
    path = tmp_path / "w.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    s = napl.Sim.load(path)
    s.step(2)
    assert s.query("/devices/wind", "height_agl")[0][0] > 0
    assert s.query("/devices/wind", "wind_ref")[1] == "m/s"
    s.set(
        "/devices/wind", "gust", [4.0, 20.0, 0, 0, 1, 0.0, 0.0]
    )  # a 4 m/s updraft gust, uniform in space
    s.step(10)
    assert s.query("/devices/wind", "n_gusts")[0][0] == 1
    with pytest.raises(ValueError):
        s.set("/devices/wind", "gust", [1.0, 2.0])
    with pytest.raises(ValueError):
        s.set("/devices/wind", "height_agl", 3.0)  # read-only
    # air density follows origin_alt
    assert s.query("/devices/air", "density")[0][0] == pytest.approx(
        0.9093, abs=0.01
    )  # ISA at 3000 m
    # a wind_field without an environment is rejected at load
    bad = tmp_path / "bad.xml"
    broot = ET.parse(path).getroot()
    txt = broot.find("custom/text")
    dev = json.loads(txt.get("data"))
    del dev["environments"]
    txt.set("data", json.dumps(dev))
    bad.write_text(ET.tostring(broot, encoding="unicode"))
    with pytest.raises(Exception):
        napl.Sim.load(bad)


def test_compose_places_the_terrain_and_pads(tmp_path):
    t = small("moorland")
    sc = compose(tmp_path, [Vehicle("d", "x500v2", pos=(t.pad("p").x, t.pad("p").y, t.pad("p").z))], env=Environment(wind=(6.0, 0, 0), origin_alt=3300.0),
                 terrain=t, wind_field=WindField(ridge_gain=0.5))  # fmt: skip
    man = sc["manifest"]
    assert (
        man["pads"]["p"]["z"] == t.pad("p").z and man["terrain"]["kind"] == "moorland"
    )
    assert (
        man["device_cfg"]["wind"]["hfield_geom"] == "terrain"
        and man["device_cfg"]["air"]["origin_alt"] == 3300.0
    )
    root = ET.parse(sc["model"]).getroot()
    assert [
        g.get("name")
        for g in root.find("worldbody").findall("geom")
        if g.get("type") == "plane"
    ] == []
    s = napl.Sim.load(sc["model"])
    s.step(200)
    z = s.query("/world/d_bottom", "pos")[0][2]
    assert abs(z - t.pad("p").z) < 0.3  # the airframe sits on the pad, not through it
