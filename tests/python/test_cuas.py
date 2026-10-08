"""Counter-UAS demo core (python/napl/cuas, assemblies/turret.toml, scenario [interfaces.native]).

Pure-numpy parts (ballistics, camera geometry, detector / tracker) and the kernel-backed parts (scene composition, World: drones flown by
Flyer, authorization gate, hit -> damage, artefacts, `napl program` native scenario). No GPU needed: camera FRAMES are rendered only by the
demo itself (examples/cuas/turret.py, run on hfbox with --gpu); the tests use the camera device's pose and synthetic images.
"""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from napl import Sim, program
from napl.cuas import ballistics
from napl.cuas.camera import Camera, Frame
from napl.cuas.scene import Environment, Vehicle, compose
from napl.cuas.tracking import BARREL_MASK, Tracker, detect
from napl.cuas.world import World

REPO = Path(__file__).resolve().parents[2]


# ---- ballistics ---------------------------------------------------------------------------------------------------------------
def test_lead_solution_meets_the_target_and_wind_pushes_the_aim_upwind():
    p = ballistics.Pellet()
    m, tp, tv = (
        np.array([0.0, 0.0, 1.1]),
        np.array([40.0, 10.0, 15.0]),
        np.array([-3.0, 2.0, 0.0]),
    )
    s0 = ballistics.solve_lead(p, m, tp, tv, [0, 0, 0])
    assert s0 and s0["ok"]
    t, ps, vs = ballistics.flight(p, m, p.v0 * s0["u"], [0, 0, 0], dt=0.001)
    at = lambda a: np.array([np.interp(s0["t_flight"], t, a[:, k]) for k in range(3)])
    assert (
        np.linalg.norm(at(ps) - (tp + tv * s0["t_flight"])) < 0.1
    )  # the pellet is where the target will be
    assert 140 < np.linalg.norm(at(vs)) < 160  # ~ 200 m/s * exp(-k d): drag is in
    sw = ballistics.solve_lead(
        p, m, tp, tv, [0, 8.0, 0]
    )  # wind blowing toward +y pushes the pellet +y: aim to -y of the no-wind aim
    assert sw["az"] < s0["az"] - 0.002
    # a static target straight ahead: elevation above the line of sight = gravity drop + drag
    s = ballistics.solve_lead(p, m, np.array([50.0, 0.0, 1.1]), np.zeros(3), [0, 0, 0])
    assert 0.0005 < s["el"] < 0.02 and abs(s["az"]) < 1e-6


# ---- camera geometry ----------------------------------------------------------------------------------------------------------
def frame_at(pos, fwd, t=0.0):
    fwd = np.array(fwd, float) / np.linalg.norm(fwd)
    up = np.array([0, 0, 1.0]) - fwd * fwd[2]
    return Frame(
        np.zeros((720, 1280, 3), np.uint8),
        None,
        np.array(pos, float),
        fwd,
        up / np.linalg.norm(up),
        t,
    )


def test_pixel_ray_projection_round_trip_and_fov():
    cam = Camera(None, "/devices/cam", 0.7156, 1280, 720)
    assert math.degrees(cam.fov_h) == pytest.approx(
        67.2, abs=0.3
    )  # Pi Camera 3: 66 x 41 deg (16:9 crop)
    fr = frame_at([1, 2, 3], [1, 0.4, 0.3])
    assert np.allclose(cam.ray(640, 360, fr), fr.fwd)
    for u, v in [(100, 80), (1100, 600), (640, 10)]:
        x = fr.pos + cam.ray(u, v, fr) * 25.0
        pu, pv, z = cam.project(x, fr)
        assert (pu, pv) == pytest.approx((u, v), abs=1e-6) and z > 0
    assert cam.project(fr.pos - fr.fwd * 5, fr) is None  # behind the camera


# ---- detector + tracker -------------------------------------------------------------------------------------------------------
SKY = np.array([25, 31, 41], np.uint8)


def sky_frame(rects, tiny=()):
    img = np.tile(SKY, (720, 1280, 1))
    for x, y, w, h in rects:
        img[y : y + h, x : x + w] = (5, 5, 6)
    for x, y in tiny:  # a 2 x 2 px speck: a tracer / bird / noise, below min_area
        img[y : y + 2, x : x + 2] = (200, 120, 30)
    return img


def test_detector_finds_dark_blobs_ignores_specks_ground_and_own_barrel():
    rects = [(300, 100, 16, 12), (900, 200, 12, 10)]
    bx = int(1280 * (BARREL_MASK[0] + BARREL_MASK[2]) / 2)
    img = sky_frame(
        rects + [(bx, 500, 20, 30)], tiny=[(600, 150)]
    )  # the last rect sits in the muzzle mask
    blobs = detect(img, horizon=600)
    assert len(blobs) == 2
    got = sorted(b.bbox for b in blobs)
    assert got[0] == (300, 100, 315, 111) and got[1] == (900, 200, 911, 209)
    assert (
        detect(sky_frame([(300, 700, 20, 10)]), horizon=600) == []
    )  # below the horizon row: ground, not sky


def test_tracker_stable_ids_confirm_and_never_reuse():
    cam = Camera(None, "/devices/cam", 0.7156, 1280, 720)
    tr = Tracker(cam)
    fr0 = frame_at([0, 0, 1], [1, 0, 0.2])
    ids = []
    for k in range(8):
        fr = Frame(fr0.rgb, None, fr0.pos, fr0.fwd, fr0.up, k * 0.068)
        det = detect(
            sky_frame([(300 + 4 * k, 100, 16, 12), (900 - 3 * k, 200, 12, 10)]), 600
        )
        ids.append(sorted(t.id for t in tr.update(det, fr)))
    assert all(i == ["A", "B"] for i in ids)
    assert all(t.confirmed for t in tr.tracks)
    far = next(t for t in tr.tracks if t.id == "A")
    assert far.range_m == pytest.approx(
        cam.f_px * 0.55 / 16, rel=0.05
    )  # range from the apparent size
    for k in range(8, 22):  # A disappears for longer than max_miss: dropped
        fr = Frame(fr0.rgb, None, fr0.pos, fr0.fwd, fr0.up, k * 0.068)
        tr.update(detect(sky_frame([(900 - 3 * k, 200, 12, 10)]), 600), fr)
    assert [t.id for t in tr.tracks] == ["B"]
    fr = Frame(fr0.rgb, None, fr0.pos, fr0.fwd, fr0.up, 22 * 0.068)
    out = tr.update(
        detect(sky_frame([(300, 100, 16, 12), (900 - 66, 200, 12, 10)]), 600), fr
    )
    assert sorted(t.id for t in out) == [
        "B",
        "C",
    ]  # a reappearing target is a NEW id: ids are never reused


def test_tracker_follows_a_target_through_a_camera_pan():
    """Association projects the world estimate into the new frame: a still target stays one track while the camera turns by 10 deg."""
    cam = Camera(None, "/devices/cam", 0.7156, 1280, 720)
    tr = Tracker(cam)
    target = np.array([30.0, 5.0, 8.0])
    ids = set()
    for k in range(12):
        yaw = math.radians(0.8 * k)
        fr = frame_at([0, 0, 1], [math.cos(yaw), math.sin(yaw), 0.2], k * 0.068)
        u, v, _ = cam.project(target, fr)
        det = detect(sky_frame([(int(u) - 8, int(v) - 6, 16, 12)]), 700)
        ids |= {t.id for t in tr.update(det, fr)}
    assert ids == {"A"}


# ---- scene composition --------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    out = tmp_path_factory.mktemp("scene")
    vs = [
        Vehicle("turret", "turret", pos=(0, 0, 1.0), pedestal=1.0),
        Vehicle("drone1", "x500v2", pos=(30, 0, 12), yaw=0),
    ]
    return compose(out, vs, Environment(wind=(2.0, -1.0, 0.0)), seed=3)


def test_compose_namespaces_places_and_loads(scene):
    man = scene["manifest"]
    assert set(man["vehicles"]) == {"turret", "drone1"}
    names = man["devices"]
    assert (
        "turret_gun" in names["launchers"]
        and "turret_cam" in names["cameras"]
        and "air" in names["environments"]
    )
    assert len(names["rotors"]) == 4 and all(
        n.startswith("drone1_") for n in names["rotors"]
    )
    root = ET.parse(scene["model"]).getroot()
    wb = root.find("worldbody")
    tops = [b.get("name") for b in wb.findall("body")]
    assert "drone1_bottom" in tops and "turret_base" in tops
    assert len(set(man["body_paths"].values())) == len(
        man["body_paths"]
    )  # prim paths are unique
    assert (
        wb.find("body[@name='drone1_bottom']/freejoint") is not None
    )  # a free joint only works on a top-level body
    sim = Sim.load(scene["model"])
    sim.step(5)
    pos, fwd, up = (
        sim.query("/devices/turret_cam", a)[0] for a in ("pos", "fwd", "up")
    )
    assert (
        pos[2] == pytest.approx(1.17, abs=0.05)
        and np.allclose(fwd, [1, 0, 0], atol=0.01)
        and np.allclose(up, [0, 0, 1], atol=0.01)
    )
    d = sim.query("/devices/turret_gun", "muzzle_dir")[0]
    assert np.allclose(d, fwd, atol=0.01)
    ids = list(sim.body_positions())
    want = ["/world"] + [man["body_paths"][n] for n in man["body_names"][1:]]
    assert (
        ids[: len(want)] == want
    )  # manifest order == kernel body order (device prims follow the bodies)
    assert sim.query(man["body_paths"]["drone1_bottom"], "pos")[0][0] == pytest.approx(
        30.0, abs=0.01
    )


# ---- World: drones, gate, damage, artefacts -----------------------------------------------------------------------------------
@pytest.fixture
def world(scene, tmp_path):
    return World(scene["model"], tmp_path, seed=3)


def test_flyer_tracks_setpoints(world):
    d = world.drones["drone1"]
    p0 = np.array(d.pos)
    d.goto((25.0, 3.0, 14.0), (0.0, 0.0, 0.0))
    world.step(int(12.0 / world.dtc))
    err = np.linalg.norm(d.pos - (np.array([25.0, 3.0, 14.0]) + d.offset))
    assert err < 0.6 and d.flyer.last[3] < 0.15, (
        p0,
        d.pos,
        err,
    )  # settled within 0.6 m, level
    assert d.status == "flying" and not world.sim.diverged


def test_fire_needs_an_authorization_and_pellets_hit_the_ground(world):
    assert (
        world.fire(None) is False
        and world.fires_unauthorized == 1
        and world.rounds == 0
    )
    world.aim(0.0, 0.05)
    world.step(int(1.0 / world.dtc))
    tok = world.authorize("A", rounds=1, valid_s=2.0)
    assert world.fire(tok, "B") is False  # an authorization is for ONE target
    assert (
        world.fire(tok, "A") is True and world.fire(tok, "A") is False
    )  # one round authorized
    world.step(int(3.0 / world.dtc))
    assert (
        world.rounds == 1 and world.ground == 1
    )  # 3 deg up from 1.2 m: lands within the second
    ev = [e["type"] for e in world.events]
    assert ev.count("authorization") == 1 and ev.count("fire") == 1 and "hit" in ev
    assert world.events[0]["text"] == "operator authorized A"


def test_hit_to_damage_mapping(world):
    d = world.drones["drone1"]
    p = np.array(world._q(d.rotor_bodies[1], "pos"))  # a hit at rotor 2
    assert world._damage(d, "prop2", p) == "rotor2.damage=1"
    assert world._q(d.rotors[1], "damage")[0] == 1.0 and d.status == "damaged"
    assert (
        world._damage(d, "tube2", p) == "esc2.fault_open=1"
        and world._q(d.escs[1], "fault_open")[0] == 1.0
    )
    assert world._damage(d, "hanger3", p) == "none"  # frame: no functional damage
    assert world._damage(d, "fc", p) == "all escs fault_open=1"


def test_finish_writes_artefacts_and_hash(world, tmp_path):
    world.step(250)
    world.finish()
    nat = json.loads((tmp_path / "native.json").read_text())
    assert (
        nat["tick"] == world.sim.tick == 1000
        and nat["state_hash"] == f"{world.sim.state_hash():016x}"
    )
    head = (tmp_path / "trace.csv").read_text().splitlines()[0].split(",")
    assert (
        head[:2] == ["t", "rounds"]
        and "engaged_drones" in head
        and "pellet_range_m" in head
    )
    for f in ("replay.naplog", "events.json", "summary.json", "truth.csv", "hud.jsonl"):
        assert (tmp_path / f).exists()


# ---- `napl program` with [interfaces.native] ----------------------------------------------------------------------------------
def test_native_scenario_judges_trace_monitors_and_replay(tmp_path):
    (tmp_path / "scene.py").write_text(
        "from napl.cuas.scene import Environment, Vehicle, compose\n"
        "def build(out):\n"
        "    vs = [Vehicle('turret', 'turret', pos=(0, 0, 1.0), pedestal=1.0), Vehicle('drone1', 'x500v2', pos=(30, 0, 12))]\n"
        "    return compose(out, vs, Environment())['model']\n"
    )
    (tmp_path / "prog.py").write_text(
        "from napl.cuas.world import World\n"
        "w = World.from_env()\n"
        "w.step(1000)\n"
        "tok = w.authorize('A', rounds=1)\n"
        "w.fire(tok, 'A')\n"
        "w.step(500)\n"
        "w.finish()\n"
    )
    mon = (
        '[[monitors]]\nname = "airborne"\ncolumn = "flying_min_alt_m"\nop = "always"\nlo = 3.0\n'
        '[[monitors]]\nname = "fired"\ncolumn = "rounds"\nop = "final"\nlo = 1\n'
        '[[monitors]]\nname = "impossible"\ncolumn = "rounds"\nop = "final"\nlo = 2\n'
    )
    sc = tmp_path / "n.scenario.toml"
    sc.write_text(
        'name = "n"\n[model]\nscript = "scene.py"\n[interfaces.native]\n[program]\ncmd = ["python", "{dir}/prog.py"]\n'
        f"[limits]\nwall_s = 300\ncpu_s = 300\naddress_space_mb = 65536\n{mon}"
    )
    res, _ = program.run(sc, tmp_path / "runs")
    assert res["kind"] == "native" and res["program"]["returncode"] == 0
    ok = {m["name"]: m["pass"] for m in res["monitors"]}
    assert ok == {
        "airborne": True,
        "fired": True,
        "impossible": False,
    }  # judged from trace.csv; the failing one fails the run
    assert not res["pass"] and "monitor impossible" in res["fail_reasons"]
    assert res["sim"]["tick"] == 6000  # 1500 control periods of 4 ticks
    if res["replay"]["available"]:  # build/napl-replay present
        assert (
            res["replay"]["match"]
            and res["replay"]["state_hash"] == res["sim"]["state_hash"]
        )


def test_turret_pi_sits_on_the_base_plate_on_four_standoffs():
    """The Pi 5 is seated on the plate (inside its 200 x 200 mm footprint, clear of the pan servo), raised by four M2.5 standoffs
    that stand at its four holes; the design checks clean."""
    from napl.design import Design

    d = Design.open(REPO / "assemblies" / "turret.toml")
    T = d.poses()
    pi = d._part("raspberry-pi-5")
    L, W = pi.value("length"), pi.value("width")
    R, o = T["pi"][:3, :3], T["pi"][:3, 3]
    world = (
        np.array([[sx * L / 2, sy * W / 2, 0] for sx in (-1, 1) for sy in (-1, 1)])
        @ R.T
        + o
    )
    assert np.abs(world[:, :2]).max() <= 0.100 - 0.005  # 5 mm in from every plate edge
    assert world[:, 0].max() < -0.020  # clear of the pan servo (body from x = -0.011)
    holes = (
        np.array([[sx * 0.029, sy * 0.0245, 0] for sx in (-1, 1) for sy in (-1, 1)])
        @ R.T
        + o
    )
    assert abs(o[2] - 0.014) < 1e-9  # the board rests on the 14 mm standoff bodies
    by_id = {i["id"]: i for i in d.doc["instances"]}
    posts = np.array([by_id[f"pi_so{k}"]["pos"][:2] for k in range(1, 5)])
    for h in holes[:, :2]:
        assert (
            np.linalg.norm(posts - h, axis=1).min() < 1e-3
        )  # a standoff under every hole
    assert not [f for f in d.check() if f.severity in ("error", "warning")]


def test_native_scenario_validation(tmp_path):
    f = tmp_path / "bad.scenario.toml"
    f.write_text(
        'name = "b"\n[model]\nmjcf = "x.xml"\n[interfaces.native]\n[program]\ncmd = ["python", "x.py"]\n'
    )
    with pytest.raises(program.ScenarioError, match="native"):
        program.load_scenario(f)  # native needs a [model] script
    f.write_text(
        'name = "b"\n[model]\nscript = "s.py"\n[interfaces.native]\n[interfaces.px4]\nbridge = "b.toml"\n[program]\ncmd = ["python", "x.py"]\n'
    )
    with pytest.raises(program.ScenarioError, match="native"):
        program.load_scenario(f)  # and stands alone
