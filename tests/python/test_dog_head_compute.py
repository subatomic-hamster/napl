"""Dog head (Mid-360 tilted 25 deg nose-down + D435i) and compute module (Jetson, 4 CANables, IMU on I2C, RTK on UART)."""

import json
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from napl import design
from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, DOG = ROOT / "parts", ROOT / "assemblies" / "dog"
LIB = Library(PARTS)
STANDALONE_OK = {"NET_NO_SOURCE"}


def blocking(asm):
    """DRC errors a module cannot have even alone. Allowed: NET_NO_SOURCE (no battery here) and FLOATING_PORT on an EXPORTED connector
    (dog.toml wires it: the lidar / Jetson power input is the module's only power port)."""
    exported = set(asm.raw["exports"].get("connectors", {}).values())
    return [
        (f.code, f.message)
        for f in check(asm, LIB)
        if f.severity == "error"
        and f.code not in STANDALONE_OK
        and not (f.code == "FLOATING_PORT" and set(f.where) <= exported)
    ]


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def site_frames(xml_text):
    """{site name: world rotation} at qpos0 from the MJCF body tree."""
    out = {}

    def walk(el, R):
        for b in el.findall("body"):
            Rb = R @ quat_R([float(v) for v in b.get("quat", "1 0 0 0").split()])
            for s in b.findall("site"):
                out[s.get("name")] = Rb @ quat_R(
                    [float(v) for v in s.get("quat", "1 0 0 0").split()]
                )
            walk(b, Rb)

    walk(ET.fromstring(xml_text).find("worldbody"), np.eye(3))
    return out


def test_head_lidar_scans_the_mid360_envelope_tilted_25_deg(tmp_path):
    asm = A.load(DOG / "head.toml")
    assert blocking(asm) == []
    xml, _ = design.compute(asm, PARTS, tmp_path)
    dev = json.loads(
        ET.fromstring(xml).find("custom/text[@name='napl.devices']").get("data")
    )
    lid = {r["name"]: r for r in dev["rangefinders"]}["lidar"]
    assert lid["projection"] == "rosette"
    assert lid["fov_v_min"] == pytest.approx(math.radians(-7), abs=1e-4)
    assert lid["fov_v_max"] == pytest.approx(math.radians(52), abs=1e-4)
    assert lid["rays_per_frame"] == 5000
    up = site_frames(xml)[lid["site"]][
        :, 0
    ]  # rosette: site +x = the sensor's up axis (docs/models.md "Rosette scan pattern")
    assert up[0] > 0  # nose-down tilt leans the up axis forward
    assert math.degrees(math.acos(up[2])) == pytest.approx(25.0, abs=0.5)


def test_compute_wires_four_can_buses_and_the_sensors():
    asm = A.load(DOG / "compute.toml")
    assert blocking(asm) == []
    cans = sorted(
        i["id"]
        for i in asm.doc["instances"]
        if i["part"] == "makerbase-mks-canable-v2-pro"
    )
    assert cans == ["can_fl", "can_fr", "can_rl", "can_rr"]
    usb = {
        w["from"].split(".")[0]: w
        for w in asm.doc["wires"]
        if w["from"].split(".")[0] in cans
    }
    assert sorted(usb) == cans and {w["to"] for w in usb.values()} == {
        "jetson.usb_j6",
        "jetson.usb_j7",
    }
    pins = [p for w in usb.values() for p in w["to_pins"]]
    assert len(pins) == len(set(pins))  # four distinct USB-A sockets
    wires = {w["name"]: w for w in asm.doc["wires"]}
    assert (
        wires["rtk_uart"]["to"] == "jetson.uart1"
        and wires["imu_i2c"]["to"] == "jetson.i2c1"
    )
    ex = asm.raw["exports"]["connectors"]
    assert {
        "power_in",
        "eth",
        "usb_cam",
        "can_fl",
        "can_fr",
        "can_rl",
        "can_rr",
    } <= set(ex)
