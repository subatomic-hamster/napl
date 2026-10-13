"""Rain in the kernel (docs/models.md "Rain"): a Skydio X500 hovering at R = 0 / 25 / 100 mm/h (thrust margin, hover power, water film), the
depth camera's dropout / false-return statistics, the design-side schema, and that a scene without rain keeps its hash. Needs the built kernel
(hfbox: tools/remote.sh run 'pytest tests/python/test_rain.py -s')."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest
from napl.cuas import flyer as fl
from napl.cuas.scene import Environment, Vehicle, compose

REPO = Path(__file__).resolve().parents[2]
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
RATES = (0.0, 25.0, 100.0)
HOVER_Z = 3.0
WALL_X = 8.0


def scene(tmp_path, rain=True, wall=False):
    env = Environment(
        wind=(0.8, 0.4, 0.0),
        w20=1.0,
        rain=rain,
        rain_vehicle="skydio",
        rain_catch_area=0.09,  # ESTIMATES: X500 body + arms seen from above (props are counted by the rotor model)
        rain_wet_area=0.25,
    )
    res = compose(tmp_path, [Vehicle("skydio", "skydio")], env, seed=7)
    if wall:  # a wall in the depth camera's view (group 0 is ray-visible)
        p = Path(res["model"])
        tree = ET.parse(p)
        ET.SubElement(tree.getroot().find("worldbody"), "geom", name="wall", type="box", pos=f"{WALL_X + 0.5} 0 {HOVER_Z}",
                      size="0.5 20 20", contype="0", conaffinity="0", group="0")  # fmt: skip
        tree.write(p)
    return res


def hover(res, cal_dir, rate, seconds=10.0, collect_depth=False):
    """Hover at HOVER_Z for `seconds` with the napl-native controller; -> dict of means over the last 3 s."""
    man = res["manifest"]
    sim = napl.Sim.load(res["model"])
    cfg, dev = man["device_cfg"], man["devices"]
    rotors = [cfg[n] for n in dev["rotors"] if n.startswith("skydio_")]
    escs = {cfg[n]["joint"]: cfg[n] for n in dev["escs"] if n.startswith("skydio_")}
    root = man["body_paths"]["skydio_bottom"]
    paths = dict(
        root=root,
        free=f"{root}/bottom:free",
        escs=[f"/devices/{escs[r['joint']]['name']}" for r in rotors],
        yaw0=0.0,
    )
    dtc = man["dt"] * 4
    f = fl.Flyer(sim, fl.calibrate("skydio", cal_dir), paths, dtc, max_tilt=0.35)
    p0 = sim.query(root, "pos")[0]
    sim.set(paths["free"], "qpos", [p0[0], p0[1], HOVER_Z, 1.0, 0.0, 0.0, 0.0])
    f.target((p0[0], p0[1], HOVER_Z))
    if "rains" in dev:
        sim.set("/devices/rain", "rate", rate)
    n = int(seconds / dtc)
    acc = {
        "thr": [],
        "i": [],
        "v": [],
        "z": [],
        "lwc": [],
        "film": [],
        "loss": [],
        "tilt": [],
        "depth": [],
    }
    for k in range(n):
        f.update()
        sim.step(4)
        if k >= n - int(3.0 / dtc):
            acc["thr"].append(np.mean(f.last[4]))
            acc["i"].append(sim.query("/devices/skydio_battery", "current")[0][0])
            acc["v"].append(sim.query("/devices/skydio_battery", "voltage")[0][0])
            acc["z"].append(f.last[0][2])
            acc["tilt"].append(f.last[3])
            if "rains" in dev:
                acc["film"].append(sim.query("/devices/rain", "film_mass")[0][0])
                acc["loss"].append(sim.query("/devices/rain", "thrust_loss")[0][0])
        if collect_depth and k % 10 == 0 and k * dtc > 4.0:
            acc["depth"].append(
                np.array(sim.query("/devices/skydio_depth", "range")[0], float)
            )
    out = {k: float(np.mean(v)) for k, v in acc.items() if v and k != "depth"}
    out["power_w"] = out["i"] * out["v"]
    out["diverged"] = bool(sim.diverged)
    out["cal"] = f.cal
    out["depth"] = acc["depth"]
    out["hash"] = sim.state_hash()
    return out


@needs_kernel
def test_skydio_hover_in_rain_thrust_margin_and_power(tmp_path):
    res = scene(tmp_path)
    runs = {r: hover(res, tmp_path / "cal", r) for r in RATES}
    cal = runs[0.0]["cal"]
    t_max = float(
        np.sum(np.array(cal["T"])[-1])
    )  # N, all rotors at full throttle (dry)
    weight = cal["mass"] * 9.80665
    print(
        "\n   R mm/h   thrust_loss   film_g   hover_thr   power_W    margin (Tmax(1-loss)/(W+film))"
    )
    margin = {}
    for r in RATES:
        o = runs[r]
        margin[r] = (
            t_max * (1 - o.get("loss", 0.0)) / (weight + o.get("film", 0.0) * 9.80665)
        )
        print(
            f"   {r:6.0f}   {o.get('loss', 0.0):10.4f}   {1e3 * o.get('film', 0.0):6.1f}   {o['thr']:9.4f}   {o['power_w']:7.1f}    {margin[r]:.3f}"
        )
        assert not o["diverged"]
        assert abs(o["z"] - HOVER_Z) < 0.15, f"R={r}: hover altitude {o['z']}"
    assert runs[0.0].get("loss", 0.0) == 0.0 and runs[0.0].get("film", 0.0) == 0.0
    assert 0.0 < runs[25.0]["loss"] < runs[100.0]["loss"] < 0.03
    assert runs[100.0]["loss"] > 0.01
    assert (
        runs[0.0]["film"] == 0.0 < runs[25.0]["film"] < runs[100.0]["film"] < 0.09
    )  # kg, below the film cap
    assert (
        margin[0.0] > margin[25.0] > margin[100.0] > 1.5
    )  # a lighter touch than the weather: the drone still has thrust to spare
    assert runs[0.0]["power_w"] < runs[25.0]["power_w"] < runs[100.0]["power_w"]
    assert (
        runs[100.0]["power_w"] / runs[0.0]["power_w"] < 1.15
    )  # honest: a few percent, not a different aircraft


@needs_kernel
def test_depth_camera_dropout_and_false_returns_in_rain(tmp_path):
    res = scene(tmp_path, wall=True)
    stats = {}
    for r in RATES:
        o = hover(res, tmp_path / "cal", r, seconds=7.0, collect_depth=True)
        d = np.concatenate([x.ravel() for x in o["depth"]])
        wall = d[(d > 5.0) & (d < 11.0)]
        stats[r] = dict(
            valid=float(np.mean(d > 0)),
            near=float(np.mean((d > 0) & (d < 3.0))),
            sigma=float(np.std(wall)),
            n=len(d),
        )
    print("\n   R mm/h   valid_frac   near(<3m)_frac   wall sigma (m)")
    for r in RATES:
        s = stats[r]
        print(f"   {r:6.0f}   {s['valid']:9.4f}   {s['near']:13.4f}   {s['sigma']:.4f}")
    assert (
        stats[25.0]["near"] > stats[0.0]["near"]
        and stats[100.0]["near"] > stats[25.0]["near"]
    )  # drops return before the wall
    assert stats[100.0]["sigma"] > stats[0.0]["sigma"]
    assert stats[100.0]["valid"] > 0.5


@needs_kernel
def test_scene_without_rain_has_no_rain_device_and_the_same_hash(tmp_path):
    res = compose(
        tmp_path / "a",
        [Vehicle("skydio", "skydio")],
        Environment(wind=(0.8, 0.4, 0.0), w20=1.0),
        seed=7,
    )
    assert "rains" not in res["manifest"]["devices"]
    a = napl.Sim.load(res["model"])
    a.step(200)
    res2 = scene(tmp_path / "b")
    b = napl.Sim.load(res2["model"])
    b.step(200)
    assert "rains" in res2["manifest"]["devices"]
    # a dry rain device is the same physics: the dry state of the shared devices is identical (the rain group appends its own bytes)
    assert not a.diverged and not b.diverged
    pa, pb = (
        a.query("/world/skydio_bottom", "pos")[0],
        b.query("/world/skydio_bottom", "pos")[0],
    )
    assert np.allclose(pa, pb, atol=0, rtol=0)


def test_design_schema_knows_the_rain_fields():
    from napl.design import devices

    assert devices.TYPE_KEY["rain"] == "rains"
    for k in (
        "rate",
        "catch_area",
        "wet_area",
        "rain_kt",
        "rain_kq",
        "rain_a",
        "rain_margin_db",
        "rain_det",
        "rain_window_db",
    ):
        assert k in devices.FIELD_UNITS
    assert math.isclose(devices.FIELD_UNITS["rain_a"]["1/km"], 1e-3)
