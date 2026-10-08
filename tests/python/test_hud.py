"""The viewer HUD without a window: documents built by napl.cuas.hudview from a recorded hud.jsonl (tests/data/cuas_hud/) are drawn by the
real C++ overlay into a throwaway ImGui context (`Hud.dry_run`: no GPU, no display), and where a world-anchored label lands is compared with
the analytic projection (napl.hud.project). The live replay + hash check is a GPU job (tools/record_sim.py)."""

import json
from pathlib import Path

import pytest
from napl import hud as H
from napl.cuas import hudview

DATA = Path(__file__).resolve().parents[1] / "data" / "cuas_hud"
needs_hud = pytest.mark.skipif(
    H.Hud is None, reason="this build of napl has no viewer / HUD"
)
CAM = dict(
    cam_pos=[-14.0, 9.0, 7.0],
    cam_look=[22.0, 2.0, 12.0],
    fovy=45.0,
    width=1280,
    height=720,
)


def records():
    return hudview.read_hud(DATA), json.loads((DATA / "events.json").read_text())


def view(rec, events, size=(1280, 720)):
    mz = rec["muzzle"]
    return hudview.View(t=rec["t"], rec=rec, run_dir=DATA, muzzle=(mz["pos"], mz["dir"]), pellets=[p["pos"] for p in rec["projectiles"]], events=events, vp=size)  # fmt: skip


@needs_hud
def test_recorded_telemetry_draws_and_labels_sit_where_the_camera_puts_them():
    recs, events = records()
    assert len(recs) >= 10
    h, seen, labels_checked = H.Hud(), set(), 0
    for n, rec in enumerate(recs):
        h.set(hudview.document(view(rec, events), seq=n))
        r = h.dry_run(**CAM)
        assert r["error"] == "", r["error"]
        assert r["vertices"] > 100  # tabs, counters, PiP frame ... at least
        want = [
            (f"{t['id']}  {t['range_m']:.0f} m", t["world_xyz_est"])
            for t in rec["tracks"]
            if t["confirmed"] and (t["id"] == rec["selected"] or t["range_m"] < 60)
        ]
        assert [x[0] for x in r["labels"]] == [w[0] for w in want]
        for (text, x, y, vis), (_, p) in zip(r["labels"], want):
            ref = H.project(
                CAM["cam_pos"],
                CAM["cam_look"],
                CAM["fovy"],
                CAM["width"],
                CAM["height"],
                p,
            )
            assert (ref is not None) or not vis
            if ref is not None:
                assert abs(x - ref[0]) < 0.05 and abs(y - ref[1]) < 0.05, (
                    text,
                    (x, y),
                    ref,
                )
                labels_checked += 1
        seen.add(rec["stage"])
    assert (
        labels_checked > 20
    )  # the recording has tracked targets for most of its length
    assert len(seen) >= 2  # the sample covers more than one stage


@needs_hud
def test_generic_items_and_a_bad_item_does_not_take_the_hud_down():
    h = H.Hud()
    items = [
        H.tabs(["BOOST", "COAST", "APOGEE"], 1, (16, 16), status="coasting"),
        H.nudge(
            H.kv([("ALT", "412 m"), ("SPEED", "88 m/s")], (1, 0), a="tr", f=True),
            -16,
            16,
        ),
        H.text("hello", (0.5, 0.5), a="c", f=True),
        H.group(
            [H.rect((0, 0), (100, 50), line=H.CYAN)],
            (10, 600),
            (200, 100),
            space=(100, 50),
        ),
        H.wline([(0, 0, 0), (5, 0, 5)]),
        H.wtext((10, 0, 5), "ROCKET"),
        H.image("/nonexistent.png", (0, 0), (100, 50)),  # a missing file draws nothing
        {"k": "wring", "w": "bad"},  # malformed: reported, the rest still draws
    ]
    h.set(H.doc(items, seq=3))
    r = h.dry_run([0, -20, 5], [5, 0, 5], 45.0, 1280, 720)
    assert r["vertices"] > 100 and "'w'" in r["error"]
    [(text, x, y, vis)] = r["labels"]
    ref = H.project([0, -20, 5], [5, 0, 5], 45.0, 1280, 720, (10, 0, 5))
    assert (
        text == "ROCKET" and vis and abs(x - ref[0]) < 0.05 and abs(y - ref[1]) < 0.05
    )
    with pytest.raises(RuntimeError):
        h.set("{not json")


def test_camera_presets_keep_turret_and_drones_in_frame():
    turret, drones = [0, 0, 1.1], [[30, 8, 14], [45, -12, 9], [38, 0, 22]]
    for preset in hudview.PRESETS:
        c = hudview.camera_for(preset, turret, drones, [1, 0, 0], target=[30, 8, 14])
        pos, look = H.orbit_camera(c["target"], c["yaw"], c["pitch"], c["dist"])
        keep = {
            "overview": [turret] + drones,
            "top": [turret] + drones,
            "chase": [turret],
            "target": [turret, drones[0]],
        }
        for p in keep[preset]:
            xy = H.project(pos, look, 45.0, 1280, 720, p)
            assert xy is not None and 0 < xy[0] < 1280 and 0 < xy[1] < 720, (
                preset,
                p,
                xy,
            )
