"""X500 V2 PX4-bridge fixture (tests/data/px4/x500_fixture.xml): mass, ground rest, yaw sign, thrust margin, sensors."""

import json
import math
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np

REPO = Path(__file__).resolve().parents[2]
FIXTURE = REPO / "tests" / "data" / "px4" / "x500_fixture.xml"
ROOT = "/world/base/root"
G = 9.81  # MuJoCo default gravity of the fixture
AXIS_Z = np.array([1.0, 1.0, -1.0, -1.0])  # hinge axis z of motor0..3 (CCW +z, CW -z)

# part id -> (param, quantity): everything the fixture mounts (frame kit 'published_weight' = CAD parts + lumped hardware)
PARTS = {
    "holybro-x500v2-frame-kit": ("published_weight", 1),
    "holybro-2216-kv920": ("mass", 4),
    "holybro-x500v2-esc-blheli-s-20a": ("mass", 4),
    "holybro-1045-prop": ("mass", 4),
    "tattu-4s-5200mah-35c": ("mass", 1),
    "holybro-pm02-v3": ("mass", 1),
    "holybro-pdb-60a": ("mass", 1),
    "holybro-pixhawk-6c": ("mass_aluminium_case", 1),
    "holybro-m8n-gps": ("mass", 1),
}


def part_mass():
    total = 0.0
    for pid, (param, qty) in PARTS.items():
        with open(REPO / "parts" / "drone" / f"{pid}.toml", "rb") as f:
            total += qty * tomllib.load(f)["params"][param]["value"]
    return total


def fixture_mass():  # explicit geom masses (MuJoCo sums them; no settotalmass / density geoms in the file)
    return sum(
        float(g.get("mass")) for g in ET.parse(FIXTURE).iter("geom") if g.get("mass")
    )


def load(lift=False):
    s = napl.Sim.load(FIXTURE)
    if lift:
        s.set(ROOT, "qpos", [0, 0, 10, 1, 0, 0, 0])
    return s


def rot(s):  # body->world rotation from the base quaternion
    w, x, y, z = s.query("/world/base", "quat")[0]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def throttle(s, t):
    for i in range(4):
        s.set(f"/devices/esc{i}", "throttle", t[i] if hasattr(t, "__len__") else t)


def thrust_up(
    s,
):  # prop `thrust` is signed along the hinge axis: world-up thrust = thrust * axis_z
    return sum(
        s.query(f"/devices/prop{i}", "thrust")[0][0] * AXIS_Z[i] for i in range(4)
    )


def yaw_rate(s):  # free-joint angular velocity is in the body frame
    return (rot(s) @ s.query(ROOT, "qvel")[0][3:6])[2]


def test_mass_matches_parts():
    m = part_mass()
    print(f"\ntotal mass {fixture_mass():.4f} kg (parts {m:.4f} kg)")
    assert abs(fixture_mass() - m) < 1e-3
    # dynamic cross-check from the model itself: specific force = thrust / mass in free flight
    s = load(lift=True)
    throttle(s, 0.5)
    s.step(600)
    a = s.query("/devices/imu", "accel")[0][2]
    assert abs(thrust_up(s) / a - m) < 0.02 * m


def test_ground_rest():
    s = load()
    s.step(2000)
    z0 = s.query(ROOT, "qpos")[0][2]
    s.step(1000)
    z1 = s.query(ROOT, "qpos")[0][2]
    vz = s.query(ROOT, "qvel")[0][2]
    tilt = math.degrees(math.acos(min(1.0, rot(s)[2, 2])))
    print(
        f"\nrest z {z1:.4f} m (drift {1e3 * (z1 - z0):.3f} mm) vz {vz:.2e} tilt {tilt:.3f} deg"
    )
    assert abs(vz) < 1e-3 and abs(z1 - z0) < 2e-3 and tilt < 0.5
    assert abs(z1 - 0.231) < 5e-3  # 0.215 gear height + 0.016 plate mid-gap


def test_yaw_sign_matches_px4():
    for ccw, cw, sign in ((0.6, 0.4, -1), (0.4, 0.6, 1)):
        s = load(lift=True)
        throttle(s, [ccw, ccw, cw, cw])  # esc0/1 are the CCW motors
        s.step(500)
        r = yaw_rate(s)
        print(f"\nCCW {ccw} CW {cw}: yaw rate {r:.3f} rad/s")
        assert r * sign > 0.05


def hover_and_margin():
    s = load(lift=True)
    blob, weight = s.save(), part_mass() * G

    def total(t):
        s.restore(blob)
        throttle(s, t)
        s.step(500)
        return thrust_up(s)

    full = total(1.0)
    lo, hi = 0.0, 1.0
    for _ in range(12):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if total(mid) < weight else (lo, mid)
    return full, weight, (lo + hi) / 2


def test_thrust_margin_and_hover_throttle():
    full, weight, hover = hover_and_margin()
    print(
        f"\nfull-throttle thrust {full:.1f} N, weight {weight:.2f} N, margin {full / weight:.2f}, hover throttle {hover:.3f}"
    )
    assert full > 1.5 * weight
    assert 0.2 < hover < 0.8


def isa(h):
    return 101325.0 * (1 - 0.0065 * h / 288.15) ** 5.25588


def test_sensors_at_rest():
    s = load()
    s.step(1000)
    accel = s.query("/devices/imu", "accel")[0]
    assert np.allclose(accel, [0, 0, 9.8], atol=0.3), accel
    p = s.query("/devices/baro", "pressure")[0][0]
    assert abs(p - isa(488.0)) < 30, (p, isa(488.0))
    cfg = json.loads(
        ET.parse(FIXTURE).find(".//text[@name='napl.devices']").get("data")
    )
    b_world = np.array(cfg["mags"][0]["earth_field"])
    b = s.query("/devices/mag", "field")[0]
    assert np.linalg.norm(b - rot(s).T @ b_world) < 0.05 * np.linalg.norm(b_world), (
        b,
        b_world,
    )
    lat, lon = (
        s.query("/devices/gnss", "lat")[0][0],
        s.query("/devices/gnss", "lon")[0][0],
    )
    assert abs(lat - 47.397742) < 1e-4 and abs(lon - 8.545594) < 1e-4, (lat, lon)
    # IMU: exactly one new sample per 4 ticks
    seen, last = 0, s.query("/devices/imu", "accel")[0]
    for _ in range(40):
        s.step(1)
        a = s.query("/devices/imu", "accel")[0]
        seen += not np.array_equal(a, last)
        last = a
    assert seen == 10
