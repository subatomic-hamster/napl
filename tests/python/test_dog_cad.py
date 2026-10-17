"""Dog printed parts after the SP3b leg rework (docs/superpowers/plans/2026-10-16-sp3b-dog-assembly.md Task 2): every printed seat
takes the port type of the bought part it holds, handed pairs reference each other, length variants put the knee / foot at L."""

import math
import pathlib

import pytest
from napl.design import Library
from napl.design.snap import incompatible

ROOT = pathlib.Path(__file__).resolve().parents[2]
LIB = Library(ROOT / "parts")


def port(pid, name):
    return next(p for p in LIB.get(pid).ports if p["name"] == name)


SEATS = [  # (printed part, port, bought part, port)
    ("dog-abad-mount", "motor", "steadywin-gim8108-8", "mount"),
    ("dog-hip-housing-l", "abad", "steadywin-gim8108-8", "output"),
    ("dog-hip-housing-l", "pitch_seat", "steadywin-gim8108-8", "mount"),
    ("dog-hip-housing-l", "knee_seat", "steadywin-gim8108-8", "mount"),
    ("dog-hip-housing-r", "pitch_seat", "steadywin-gim8108-8", "mount"),
    ("dog-thigh-bracket-l", "origin", "steadywin-gim8108-8", "output"),
    ("dog-thigh-bracket-r-250", "origin", "steadywin-gim8108-8", "output"),
    ("dog-knee-crank-l", "motor", "steadywin-gim8108-8", "output"),
    ("dog-knee-crank-r", "motor", "steadywin-gim8108-8", "output"),
    ("dog-foot-core", "ball", "mcmaster-natural-rubber-ball-1-3-8in", "seat"),
    ("dog-head-mount", "lidar", "livox-mid-360", "base"),
    ("dog-head-mount", "camera", "intel-realsense-d435i", "mount"),
    ("dog-compute-tray", "jetson", "nvidia-jetson-orin-nano-devkit-8gb", "base"),
    ("dog-compute-tray", "rtk", "sparkfun-gps-rtk-sma-16481", "base"),
    ("dog-compute-tray", "antenna", "ublox-ann-mb-00", "base"),
    ("dog-compute-tray", "imu", "pololu-minimu-9-v6", "base"),
]


@pytest.mark.parametrize("pp,pport,bp,bport", SEATS)
def test_printed_seat_mates_its_bought_part(pp, pport, bp, bport):
    assert incompatible(port(pp, pport), port(bp, bport)) is None


@pytest.mark.parametrize("sfx,L", [("", 0.22), ("-200", 0.20), ("-250", 0.25)])
def test_length_variants_put_knee_and_foot_at_L(sfx, L):
    for side, other in (("l", "r"), ("r", "l")):
        th = LIB.get(f"dog-thigh-bracket-{side}{sfx}")
        assert th.value("length") == pytest.approx(L)
        assert port(th.id, "knee")["pos"][2] == pytest.approx(-L)
        assert th.mirror["opposite"] == f"dog-thigh-bracket-{other}{sfx}"
    sh = LIB.get(f"dog-shank{sfx}")
    assert sh.value("length") == pytest.approx(L)
    assert port(sh.id, "foot")["pos"][2] == pytest.approx(-L)
    assert sh.mirror["symmetry"] == "xz"


def test_knee_linkage_parts_replace_the_knee_housing():
    assert not any(pid.startswith("dog-knee-drive-housing") for pid in LIB.parts)
    r1 = LIB.get("dog-knee-crank-l").value("crank_radius")
    for sfx in ("", "-200", "-250"):
        assert LIB.get(f"dog-shank{sfx}").value("lug_radius") / r1 == pytest.approx(1.5)


@pytest.mark.parametrize("sfx", ["", "-200", "-250"])
def test_knee_range_contains_the_build_pose_and_the_rod_port_points_at_the_lug(sfx):
    """q = 0 (the build pose) lies in knee_range; the thigh rod port axis tilts like the rod: the rod from the lug pin (R2 at 115 deg
    about the knee) to the crank pin (R1 at 51.65 deg about the hip) has the port's axis and the port's centre is its mid-point."""
    th = LIB.get(f"dog-thigh-bracket-l{sfx}")
    lo, hi = th.params["knee_range"]["value"]
    assert lo < -1.0 and 0.0 <= hi < 0.1
    L = th.value("length")
    pt = port(th.id, "rod")
    lug = (
        0.030 * math.cos(math.radians(115)),
        -L + 0.030 * math.sin(math.radians(115)),
    )
    crank = (
        0.020 * math.cos(math.radians(51.65)),
        0.020 * math.sin(math.radians(51.65)),
    )
    d = (crank[0] - lug[0], crank[1] - lug[1])
    n = math.hypot(*d)
    assert pt["axis"][0] == pytest.approx(d[0] / n, abs=1e-4)
    assert pt["axis"][2] == pytest.approx(d[1] / n, abs=1e-4)
    assert pt["pos"][0] == pytest.approx((lug[0] + crank[0]) / 2, abs=2e-4)
    assert pt["pos"][2] == pytest.approx((lug[1] + crank[1]) / 2, abs=2e-4)


def test_bulkhead_ports_sit_on_the_abad_axes_and_the_deck_flanges():
    b = LIB.get("dog-bulkhead")
    hl, hr = port(b.id, "hip_l"), port(b.id, "hip_r")
    assert hl["pos"][1] == pytest.approx(-hr["pos"][1]) == pytest.approx(0.06)
    assert hl["pos"][0] == pytest.approx(
        0.009
    )  # plate outer face 3 mm + 6 mm abad flange
    assert port(b.id, "deck_top")["pos"][2] == pytest.approx(0.06)
    assert port(b.id, "deck_bottom")["pos"][2] == pytest.approx(-0.056)
