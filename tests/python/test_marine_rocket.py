"""Phase 5 data and helpers: rocket motor parts against their certification figures, Barrowman against the classic closed form,
and the numpy views of the sea / range-sensor devices. (The device physics is in tests/cpp/test_{rocket,vessel,rangefinder}.cpp.)"""

import math
import tomllib
from pathlib import Path

import numpy as np
import pytest
from napl import marine
from napl.design import barrowman

PARTS = Path(__file__).resolve().parents[2] / "parts"
MOTORS = ["estes-c6", "estes-e16", "estes-f15", "estes-g40"]


def part(rel):
    return tomllib.loads((PARTS / rel).read_text())


@pytest.mark.parametrize("name", MOTORS)
def test_motor_curve_impulse_and_mass(name):
    p = part(f"rocket/{name}.toml")
    rows = np.array(p["tables"]["thrust_curve"]["rows"])
    t, f = (
        np.r_[0.0, rows[:, 0]] if rows[0, 0] > 0 else rows[:, 0],
        np.r_[0.0, rows[:, 1]] if rows[0, 0] > 0 else rows[:, 1],
    )
    impulse = float(np.sum(0.5 * (f[1:] + f[:-1]) * np.diff(t)))
    v = {k: x["value"] for k, x in p["params"].items()}
    assert impulse == pytest.approx(v["total_impulse_cert"], rel=0.025)
    assert t[-1] == pytest.approx(v["burn_time_cert"], rel=0.05)
    # the published peak can be a short spike the file's samples miss (G40: 74 N sampled against 84.5 N certified)
    assert 0.85 * v["max_thrust_cert"] <= f.max() <= 1.02 * v["max_thrust_cert"]
    assert 0.2 < v["propellant_mass"] / v["mass"] < 0.7 and v["ignition_delay"] > 0


def test_barrowman_nose_and_fins_against_closed_forms():
    assert barrowman.nose("cone", 0.3, 0.1) == (2.0, pytest.approx(0.2))
    c, x = barrowman.nose("ogive", 0.3, 0.1016)
    assert c == 2.0 and x / 0.3 == pytest.approx(0.462, abs=0.005)
    n, cr, ct, s, sw, d = 4, 0.16, 0.08, 0.12, 0.09, 0.1016
    lm = math.hypot(s, sw + ct / 2 - cr / 2)
    k = 1 + d / 2 / (s + d / 2)
    classic = k * 4 * n * (s / d) ** 2 / (1 + math.sqrt(1 + (2 * lm / (cr + ct)) ** 2))
    cn, xf = barrowman.fins(n, cr, ct, s, sw, d)
    assert cn == pytest.approx(classic, rel=1e-9) and 0 < xf < cr
    tot, xcp = barrowman.cp([(c, x), (cn, 0.59 + xf)])
    assert tot == pytest.approx(c + cn) and x < xcp < 0.59 + xf


class FakeSim:
    def __init__(self, waves, t=3.0, scale=0.5):
        self.v = {
            "waves": np.array(waves, float).ravel(),
            "time": np.array([t]),
            "wave_scale": np.array([scale]),
        }

    def query(self, path, attr):
        return self.v[attr], ""


def test_surface_elevation_matches_the_sum_of_components():
    a, k, w, th, ph = 0.1, 2.0, 1.5, 0.3, 0.7
    sim = FakeSim([[a, k, w, th, ph], [0.05, 4.0, 2.1, -0.2, 0.1]])
    x, y = np.array([0.0, 1.0, 2.0]), np.array([0.5, -1.0, 0.0])
    eta = marine.surface_elevation(sim, "/devices/sea", x, y, level=0.2)
    want = 0.2 + 0.5 * (
        a * np.cos(k * (x * math.cos(th) + y * math.sin(th)) - w * 3 + ph)
        + 0.05 * np.cos(4.0 * (x * math.cos(-0.2) + y * math.sin(-0.2)) - 2.1 * 3 + 0.1)
    )
    np.testing.assert_allclose(eta, want, atol=1e-12)
    assert marine.surface_elevation(sim, "/devices/sea", 0.0, 0.0, t=0.0).shape == ()


def test_depth_image_shape_nan_and_dtype():
    sim = type(
        "S",
        (),
        {"query": lambda self, p, a: (np.array([1.0, 0, 2.0, 3.0, 0, 5.0]), "m")},
    )()
    img = marine.depth_image(sim, "/devices/cam", 3, 2)
    assert img.shape == (2, 3) and img.dtype == np.float32
    assert np.isnan(img[0, 1]) and np.isnan(img[1, 1]) and img[1, 2] == 5.0
