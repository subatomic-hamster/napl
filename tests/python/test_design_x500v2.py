"""assemblies/x500v2.toml: the Holybro X500 V2 quad built from the parts library, loaded in the kernel, resting on its landing gear."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import design
from napl.design import Library

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
ASM = REPO / "assemblies" / "x500v2.toml"

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("x500v2")
    r = design.build(ASM, PARTS, out)
    return r["xml"], r["report"]


def _bodies(xml_path):
    """{body name: (parent name or None, ET element)} and the prim path of every body."""
    root = ET.parse(xml_path).getroot()
    parent, elem = {}, {}

    def walk(el, par):
        for b in el.findall("body"):
            parent[b.get("name")], elem[b.get("name")] = par, b
            walk(b, b.get("name"))

    walk(root.find("worldbody"), None)

    def path(n):
        return "/world/" + "/".join(reversed(_chain(parent, n)))

    return elem, {n: path(n) for n in elem}


def _chain(parent, n):
    out = [n]
    while parent[out[-1]]:
        out.append(parent[out[-1]])
    return out


def _quat_to_R(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# ---- 1. builds, loads, devices present -------------------------------------------------------------------------------
@needs_kernel
def test_builds_and_loads_with_every_device(built):
    xml_path, report = built
    assert report["warnings"] == []
    s = napl.Sim.load(
        xml_path
    )  # the kernel parses the devices JSON strictly (unknown field = load error)
    s.step(5)
    assert not s.diverged
    probes = {
        "battery": "voltage",
        "imu": "gyro",
        "baro": "pressure",
        "mag": "field",
        "gnss": "lat",
        **{f"esc{k}": "omega" for k in range(1, 5)},
        **{f"rotor{k}": "omega" for k in range(1, 5)},
    }
    for name, attr in probes.items():
        values, _ = s.query(f"/devices/{name}", attr)
        assert np.all(np.isfinite(values)), name
    devs = {d["name"]: d for d in report["devices"]}
    assert {"battery", "imu", "baro", "mag", "gnss"} <= set(devs)
    assert all(
        d["parts"] for d in devs.values()
    )  # every device carries part provenance
    assert devs["esc1"]["parts"] == [
        "holybro-2216-kv920",
        "holybro-x500v2-esc-blheli-s-20a",
    ]


# ---- 2. rest on the landing gear --------------------------------------------------------------------------------------
@needs_kernel
def test_rests_on_landing_gear_at_zero_throttle(built):
    xml_path, _ = built
    s = napl.Sim.load(xml_path)
    for k in range(1, 5):
        s.set(f"/devices/esc{k}", "throttle", 0.0)
    z0 = s.query("/world/bottom", "pos")[0][2]
    s.step(1000)  # 1 s
    assert not s.diverged
    p1 = s.query("/world/bottom", "pos")[0]
    s.step(10)
    p2 = s.query("/world/bottom", "pos")[0]
    speed = np.linalg.norm(p2 - p1) / 0.010
    # The builder places the lowest collision point exactly on the floor, so the only motion is the sinking into the soft
    # contact (MuJoCo default solref 0.02 s: settled within ~0.1 s) under 1.4 kg. Measured on hfbox: 0.23 mm of sinking and
    # a speed of ~1e-16 m/s after 1 s. Bounds: 1 mm (4x the measured sinking, 4900x below the 4.9 m of a free fall) and 1 mm/s.
    assert abs(p1[2] - z0) < 1e-3, (z0, p1[2])
    assert np.linalg.norm(p1[:2]) < 1e-3
    assert speed < 1e-3, speed
    for k in range(1, 5):  # unpowered, no airflow: props at rest
        assert abs(s.query(f"/devices/rotor{k}", "omega")[0][0]) < 1e-3
        assert abs(s.query(f"/devices/esc{k}", "omega")[0][0]) < 1e-3
    # no load on the pack (ESCs at zero throttle are open phase; no avionics Load is modelled: the parts publish no
    # Pixhawk current draw): terminal voltage = OCV(soc0 = 1) x 4 cells
    dev = json.loads(ET.parse(xml_path).getroot().find("custom/text").get("data"))
    ocv_full = dev["batteries"][0]["ocv_v"][-1]
    assert dev["batteries"][0]["ocv_soc"][-1] == 1.0
    v = s.query("/devices/battery", "voltage")[0][0]
    assert abs(v - ocv_full) < 1e-3, (v, ocv_full)
    # expected from the referenced OCV table (Chen 2006 cells charged to 4.1 V: 4 x 4.103 = 16.412 V), not a hard-coded 16.8
    lib = Library(PARTS)
    doc = design.load(ASM).doc
    ocv_id = next(d["ocv"] for d in doc["devices"] if d["type"] == "battery")
    assert ocv_id == "lipo-cell-ocv-chen2006"
    rows = lib.get(ocv_id).raw["tables"]["ocv_vs_soc"]["rows"]
    bat_part = lib.get(next(i["part"] for i in doc["instances"] if i["id"] == "bat"))
    cells = int(bat_part.value("cells_series"))
    assert abs(ocv_full - cells * rows[-1][1]) < 1e-9
    assert abs(ocv_full - 16.412) < 1e-9


@needs_kernel
def test_all_four_rotors_push_up_and_yaw_cancels(built):
    """Checks the spin-direction conventions end to end: CCW props have joint axis +z / dir +1, CW props -z / dir -1."""
    xml_path, _ = built
    elem, _ = _bodies(xml_path)
    s = napl.Sim.load(xml_path)
    for k in range(1, 5):
        s.set(f"/devices/esc{k}", "throttle", 0.5)
    z0 = s.query("/world/bottom", "pos")[0][2]
    s.step(300)
    assert not s.diverged
    ups = []
    for k in range(1, 5):
        axis_z = float(
            elem[f"prop{k}"].find(f"joint[@name='spin{k}']").get("axis").split()[2]
        )
        ups.append(
            s.query(f"/devices/rotor{k}", "thrust")[0][0] * axis_z
        )  # thrust is signed along the joint axis
    assert min(ups) > 4.0, ups  # N each: vehicle weight is ~13.5 N, so 4 x 5 N lifts it
    assert max(ups) - min(ups) < 0.05 * np.mean(ups), ups
    assert s.query("/world/bottom", "pos")[0][2] > z0 + 1e-3  # it lifts off the gear
    qw, qx, qy, qz = s.query("/world/bottom", "quat")[0]
    assert abs(qz) < 1e-3  # equal-and-opposite reaction torques: no yaw


# ---- 3. masses ---------------------------------------------------------------------------------------------------------
def _frame_target(report):
    (t,) = report["targets"]
    return t


def test_mass_budget_and_published_weight_comparison(built):
    _, report = built
    m = report["mass"]
    assert math.isclose(sum(m["by_instance"].values()), m["total"], rel_tol=1e-12)
    assert math.isclose(sum(m["by_group"].values()), m["total"], rel_tol=1e-12)
    assert (
        1.2 < m["total"] < 1.6
    )  # all-up mass incl. battery, kg (sanity: 4S 5200 mAh X500 V2 builds weigh ~1.3-1.5 kg)
    t = _frame_target(report)
    assert (
        t["scope"] == ["frame"] and t["target"] == 0.61
    )  # frame-kit param published_weight
    assert math.isclose(t["model"], m["by_group"]["frame"], rel_tol=1e-12)
    assert math.isclose(t["gap"], t["model"] - t["target"], abs_tol=1e-12)
    assert math.isclose(t["gap_pct"], 100 * t["gap"] / t["target"], rel_tol=1e-9)
    # tolerance = sum of the in-scope item tolerances (frame instances + frame fasteners)
    tol = sum(r["mass_tol"] for r in report["instances"] if r["group"] == "frame")
    tol += sum(f["mass_tol"] for f in report["fasteners"] if f["group"] == "frame")
    assert math.isclose(t["tolerance"], tol, rel_tol=1e-9)
    assert t["within"] == (abs(t["gap"]) <= t["tolerance"])


@pytest.mark.xfail(
    strict=True,
    reason="real modelling gap: the modelled frame group (plates, arm connectors, arm tubes, landing gear, rail rods, arm bolts) "
    "is 0.379 kg vs Holybro's 0.610 kg; the frame kit's platform board, battery mounting board + 2 straps, hangers, tee "
    "connectors, foam grips and remaining hardware have no part files yet. Not tuned to fit.",
)
def test_frame_group_within_published_weight_tolerance(built):
    _, report = built
    assert _frame_target(report)["within"]


# ---- 4. every geom maps to a real part -------------------------------------------------------------------------------------
def test_every_geom_maps_to_a_part(built):
    xml_path, report = built
    names = {
        g.get("name")
        for g in ET.parse(xml_path).getroot().iter("geom")
        if g.get("name") != "floor"
    }
    assert names and names == set(report["geoms"])
    lib = Library(PARTS)
    for g, pid in report["geoms"].items():
        assert pid in lib.parts, (g, pid)


# ---- 5. deterministic -----------------------------------------------------------------------------------------------------
def test_two_builds_are_byte_identical(tmp_path):
    a = design.build(ASM, PARTS, tmp_path / "a")
    b = design.build(ASM, PARTS, tmp_path / "b")
    assert a["xml"].read_bytes() == b["xml"].read_bytes()
    assert a["report_path"].read_bytes() == b["report_path"].read_bytes()


def test_connectors_are_connector_parts_and_environment_matches_gnss(built):
    _, report = built
    lib = Library(PARTS)
    for c in report["connectors"]:
        if c["part"] is not None:
            assert lib.get(c["part"]).category == "connector", c
    dev = json.loads(ET.parse(built[0]).getroot().find("custom/text").get("data"))
    assert (
        dev["environments"][0]["origin_alt"]
        == dev["gnss_receivers"][0]["alt0"]
        == 408.0
    )


# ---- 6. geometry: motor positions and spin axes ---------------------------------------------------------------------------
@needs_kernel
def test_motors_at_the_published_diagonal_and_spin_axes_vertical(built):
    xml_path, _ = built
    elem, path = _bodies(xml_path)
    s = napl.Sim.load(xml_path)
    # PX4 quad-X numbering seen from above with x forward, y left: m1 front-right, m2 rear-left, m3 front-left, m4 rear-right
    expect = {1: (1, -1), 2: (-1, 1), 3: (1, 1), 4: (-1, -1)}
    # STEP motor solids at +-177.1 mm (airframe doc), quoted to 0.1 mm: +-0.05 mm, plus the 5-digit rounding of the port
    # position 0.17266 m and plate port 0.05501 m (a few 1e-6 m): 1e-4 m covers both.
    tol = 1e-4
    bottom = s.query("/world/bottom", "pos")[0]
    for k, (sx, sy) in expect.items():
        p = s.query(path[f"mot{k}"], "pos")[0] - bottom
        assert abs(p[0] - sx * 0.1771) < tol and abs(p[1] - sy * 0.1771) < tol, (k, p)
        # published wheelbase 500 mm: the motor is 0.25 m from the centre (derived 0.17678 per axis): within 0.5 mm
        assert abs(math.hypot(p[0], p[1]) - 0.25) < 5e-4
        # prop sits above its motor on the same vertical axis; its spin axis is vertical
        pp = s.query(path[f"prop{k}"], "pos")[0] - bottom
        assert abs(pp[0] - p[0]) < 1e-9 and abs(pp[1] - p[1]) < 1e-9 and pp[2] > p[2]
        Rw = _quat_to_R(s.query(path[f"prop{k}"], "quat")[0])
        jt = elem[f"prop{k}"].find(f"joint[@name='spin{k}']")
        axis = Rw @ np.array([float(x) for x in jt.get("axis").split()])
        # CCW (m1, m2): +z; CW (m3, m4): -z (the rotor device then uses dir = -1)
        assert np.allclose(axis, [0, 0, 1 if k in (1, 2) else -1], atol=1e-9), (k, axis)
