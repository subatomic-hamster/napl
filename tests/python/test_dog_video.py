import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
sys.path.insert(0, str(ROOT / "tools"))

import napl
import record_dog as RD

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def test_stages_cover_every_instance_once_and_name_the_five_sub_assemblies():
    import tomllib

    doc = tomllib.loads((ROOT / "assemblies" / "dog" / "dog.toml").read_text())
    sub_ids = {s["id"] for s in doc["subassemblies"]}
    assert sub_ids == {"fl", "fr", "rl", "rr", "head", "power", "compute"}
    seen = [g for _, _, groups in RD.STAGES for g in groups]
    assert len(seen) == len(set(seen)) and set(seen) >= sub_ids and "chassis" in seen
    assert [t.split()[0] for t, _, _ in RD.STAGES][:2] == ["1/6", "2/6"]
    assert set(RD.CHASSIS) == {i["id"] for i in doc["instances"]}


@needs_kernel
def test_oplog_steps_are_timed_and_ordered(tmp_path):
    out = tmp_path / "oplog.json"
    RD.main(["oplog", str(out)])
    steps = json.loads(out.read_text())["steps"]
    segs = [s["seg"] for s in steps]
    assert segs.index("BUILD") < segs.index("WIRING") < segs.index("PROGRAM")
    assert (
        sum(r["wait"] for s in steps for r in s["reqs"]) <= RD.PART_A_MAX_S
    )  # part (a) stays inside its time budget
    assert any(
        r["tool"] == "sub.open" for s in steps for r in s["reqs"]
    )  # the leg is opened as its own document
    assert any(
        r["tool"] == "ui.wire_pick" for s in steps for r in s["reqs"]
    )  # pin-level wiring is shown
    assert all(
        r["tool"] != "ui.frame" for s in steps for r in s["reqs"]
    )  # ui.frame selects what it frames (record_skydio finding)
    assert steps == json.loads(json.dumps(steps))  # plain JSON, deterministic
    RD.main(["oplog", str(tmp_path / "again.json")])
    assert (tmp_path / "again.json").read_text() == out.read_text()


@needs_kernel
def test_explode_keeps_every_body_and_moves_only_groups(tmp_path):  # Review Focus 5
    import dogsim

    xml, _ = dogsim.build(tmp_path)
    text = xml.read_text()
    groups = {
        "fl:": (0.3, 0.3, 0.0),
        "rl:": (-0.2, 0.1, 0.05),
        "head:": (0.4, 0.0, 0.2),
    }
    out = RD.explode_xml(text, groups, 1.0)
    bodies = lambda t: sorted(b.get("name") for b in ET.fromstring(t).iter("body"))
    assert bodies(out) == bodies(text)
    assert RD.explode_xml(text, groups, 0.0) == text
    moved, base = RD.world_pos(out), RD.world_pos(text)
    assert set(moved) == set(base)
    for k in base:
        want = next(
            (np.array(v) for g, v in groups.items() if k.startswith(g)), np.zeros(3)
        )
        assert abs(moved[k] - base[k] - want).max() < 1e-9, (
            k
        )  # rl: hangs from the 180-deg-turned bulkhead: the WORLD shift is still the vector
    vec = RD.leg_vectors(text)
    assert (
        vec["fl:"][1] > 0 > vec["fr:"][1] and vec["rl:"][1] > 0 > vec["rr:"][1]
    )  # outward along each leg's own side


def fake_run(tmp_path, h="h1", th="h1"):
    d = tmp_path / "run"
    (d / "tap").mkdir(parents=True)
    n = 12
    np.savez_compressed(
        d / "tap" / "tap.npz", t=np.arange(n) * 0.5, truth_xy=np.zeros((n, 2)), est_xy=np.full((n, 2), 0.04), est_yaw=np.zeros(n), phase=np.ones(n),
        mode=np.full(n, 2), gait=np.full(n, 2), contact=np.tile([1, 0, 1, 0], (n, 1)).astype(float), force_z=np.full((n, 4), 30.0),
        foot_xy=np.zeros((n, 4, 2)), path_len=np.full(n, 2), path_xy=np.tile(np.array([[0, 0], [5, 0]], np.float32), (n, 1)),
        map_t=np.array([0.0, 5.0]), mapped_packed=np.stack([np.packbits(np.eye(8, dtype=bool))] * 2), mapped_shape=np.array([8, 8]),
    )  # fmt: skip
    (d / "tap" / "tap.json").write_text(
        json.dumps(dict(n=n, dt=0.1, state_hash=th, every=5, pool=2, cols=[]))
    )
    (d / "result.json").write_text(json.dumps(dict(sim=dict(state_hash=h))))
    return d


def test_overlay_numbers_come_from_the_tap_and_stale_tap_is_refused(
    tmp_path,
):  # Review Focus 4
    d = fake_run(tmp_path)
    tap = RD.TapData(d)
    assert tap.est_error(3) == pytest.approx(
        np.hypot(0.04, 0.04)
    )  # the number printed on the estimator diagram
    assert tap.force_sum(3) == pytest.approx(120.0) and tap.gait_name(3) == "trot"
    with pytest.raises(SystemExit, match="stale"):
        RD.TapData(fake_run(tmp_path / "x", h="h1", th="h2"))
    pytest.importorskip(
        "PIL"
    )  # the drawing needs Pillow (the video venv; hfbox .venv has it installed for this test)
    for key, *_ in RD.DIAGRAMS:
        img = RD.draw(key, tap, 3, (RD.W, RD.H))
        assert img.size == (RD.W, RD.H)


def test_diagram_numbers_change_with_the_tap_sample(tmp_path):
    """The picture is a function of the tap: a different force on the tap gives a different estimator / MPC frame."""
    pytest.importorskip("PIL")
    d = fake_run(tmp_path)
    tap = RD.TapData(d)
    a = RD.draw("mpc_wbc", tap, 3, (RD.W, RD.H)).tobytes()
    tap.z["force_z"][3] = 60.0
    assert RD.draw("mpc_wbc", tap, 3, (RD.W, RD.H)).tobytes() != a


def test_diagram_layout_is_deterministic(tmp_path):
    import hashlib

    pytest.importorskip("PIL")
    tap = RD.TapData(fake_run(tmp_path))
    h = lambda: [
        hashlib.sha1(RD.draw(k, tap, 5, (RD.W, RD.H)).tobytes()).hexdigest()
        for k, *_ in RD.DIAGRAMS
    ]  # checked in memory, never written
    assert h() == h()
    assert sum(s for *_, s in RD.DIAGRAMS) <= RD.PART_B_MAX_S


def test_director_orbits_at_constant_radius_and_is_deterministic():
    tr = {
        "t": np.arange(0, 60, 0.1),
        "x": np.linspace(0, 30, 600),
        "y": np.zeros(600),
        "z": np.full(600, 0.4),
    }
    d1, d2 = RD.Director(tr), RD.Director(tr)
    ts = np.arange(0, 20, 0.05)
    cams = [d1.step(t, 0.05) for t in ts]
    again = [d2.step(t, 0.05) for t in ts]
    assert all(
        np.array_equal(c["pos"], c2["pos"])
        and np.array_equal(c["lookat"], c2["lookat"])
        for (c, _), (c2, _) in zip(cams, again)
    )
    r = [np.hypot(*(c["pos"][:2] - c["lookat"][:2])) for c, k in cams if k == "orbit"]
    assert (
        max(r) - min(r) < 0.35
    )  # smoothing lags the dog a little, the orbit radius stays ORBIT_R
    assert {k for _, k in cams} == {"orbit"}
    az = [np.arctan2(*(c["pos"][:2] - c["lookat"][:2])[::-1]) for c, _ in cams]
    assert abs(np.unwrap(az)[-1] - np.unwrap(az)[0]) == pytest.approx(
        np.radians(RD.ORBIT_DEG_S) * 19.95, rel=1e-6
    )  # 6 deg per video second
    c, k = RD.Director(tr).step(3.0, 0.05, "map")
    assert (
        k == "map" and c["pos"][2] == RD.MAP_H and c["lookat"][2] == 0.0
    )  # straight above the dog


def test_shot_schedule_ends_on_the_map_and_fits_part_c_budget():
    kinds = [RD.shot_at(t) for t in range(int(RD.PART_C_MAX_S))]
    assert (
        kinds[0] == "orbit"
        and kinds[24] == "orbit"
        and kinds[-1] == "map"
        and set(kinds) == {"orbit", "map"}
    )
    assert all(k == "map" for k in kinds[int(RD.PART_C_MAX_S) - 20 :])
    assert RD.PART_C_MAX_S <= 190.0


def test_speed_schedule_maps_video_time_to_sim_time_and_back():
    assert (
        RD.sim_time(10.0) == 10.0
        and RD.sim_time(25.0) == 25.0
        and RD.sim_time(35.0) == 25.0 + 4 * 10.0
    )
    for t in (0.0, 12.0, 25.0, 100.0, 640.0):
        assert RD.sim_time(RD.video_time(t)) == pytest.approx(t)


def test_hud_draws_from_the_tap_and_marks_the_map_shot(tmp_path):
    Image = pytest.importorskip("PIL.Image")
    tap = RD.TapData(fake_run(tmp_path))
    base = Image.new("RGB", (RD.W, RD.H), (60, 60, 60))
    cam = dict(pos=[0.0, -0.5, 35.0], lookat=[0.0, 0.0, 0.0], fovy=50.0)
    orbit = RD.hud(None, tap, 3, dict(cam, shot="orbit"), base)
    flat = RD.hud(None, tap, 3, dict(cam, shot="map"), base)
    assert (
        orbit.size == flat.size == (RD.W, RD.H) and orbit.tobytes() != flat.tobytes()
    )  # the map shot adds the mask, path and trail overlay


def test_timeline_fits_the_budget_and_orders_the_three_parts():
    parts = [p for p, _, _ in RD.TIMELINE]
    assert parts == sorted(parts, key=["card", "A", "B", "C", "result"].index)
    assert sum(s for _, _, s in RD.TIMELINE) <= RD.TOTAL_MAX_S
    assert RD.PART_A_MAX_S + RD.PART_B_MAX_S + RD.PART_C_MAX_S <= RD.TOTAL_MAX_S


def test_encoder_is_non_gpl_by_default():
    import shutil

    import videnc

    ids = [c[0] for c in videnc._candidates(".webm", gpl=False)] + [
        c[0] for c in videnc._candidates(".mp4", gpl=False)
    ]
    assert "ffmpeg" not in ids  # ffmpeg VAAPI / libx264 are the --gpl opt-in
    if shutil.which("ffmpeg"):
        assert "ffmpeg" in [c[0] for c in videnc._candidates(".webm", gpl=True)]
    args = RD.build_parser().parse_args(["encode", "r", "o.webm"])
    assert (
        args.gpl is False
        and RD.build_parser().parse_args(["encode", "r", "o.webm", "--gpl"]).gpl is True
    )


def test_cards_take_numbers_from_the_results_json(tmp_path):
    mj = tmp_path / "missions.json"
    mj.write_text(
        json.dumps(
            {
                "families": {
                    "rubble": {"n": 36, "k_nofall": 30, "wilson_nofall": [0.71, 0.93]}
                },
                "missions": {
                    "rubble": [{"seed": 31001, "fell_ever": 1, "coverage": 0.12}]
                },
                "criteria": [
                    {
                        "text": "BOM about 3-4.5 k USD",
                        "measured": "5554",
                        "verdict": "NOT MET",
                    }
                ],
            }
        )
    )
    lines = RD.mc_card_lines(mj)
    assert any("30/36" in ln and "0.71" in ln for ln in lines)
    assert any("seed 31001: fell" in ln and "12 %" in ln for ln in lines)
    assert any(
        ln.startswith("NOT MET") and "BOM" in ln and "5554" in ln for ln in lines
    )


def test_results_json_round_trips_into_the_card(tmp_path):
    import missionmc as MC

    row = dict(
        seed=1,
        coverage=0.5,
        rms=0.03,
        phase=3,
        t=300.0,
        fell_ever=1,
        fallen=0,
        bumps=0,
        distance=40.0,
    )
    by = {"rubble": MC.summarize([row, dict(row, seed=2, fell_ever=0)], 420.0)}
    mj = tmp_path / "missions.json"
    mj.write_text(json.dumps(MC.results_json(by, {"bom_usd": 5554.0}, "abc")))
    lines = RD.mc_card_lines(mj)
    assert any("1/2" in ln and "rubble" in ln for ln in lines) and any(
        ln.startswith("NOT MET") and "BOM" in ln for ln in lines
    )
