# SP6: dog integration missions, per-family Monte Carlo, the end-to-end video, the results page — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the whole dog (SP3b hardware, SP4 locomotion, SP5 autonomy through the Task 11 adapter `dogloco.py`) is measured against the spec's
section 7 on full-stack missions, and shown in one video. (1) Full-stack Monte Carlo per terrain family (rocky / boulders = `rubble`,
slopes + stairs / ledges = `slopes`, loose / slippery = `scree`, ruins = `ruins`): >= 90 % no-fall with the Wilson 95 % lower bound next to
it; mission criteria (coverage <= 20 min, map height RMS <= 5 cm, >= 95 % of the reachable traversable area mapped) reported per family;
two 100 x 100 m headline mission sets on the dog (rubble, ruins). (2) `tools/record_dog.py`: one video, three parts: (a) construction
breakdown (sub-assemblies leg / power / head / compute / chassis appearing and joining, an exploded view, pin-level wiring), (b) a brief
overview of the programming (estimator, convex MPC + WBC, footholds, elevation map, EKF / RTK / ICP, coverage planner, executive) as
on-screen diagrams driven by real sim data, (c) the sim demo (the dog on rocky terrain, the map and the coverage path growing).
(3) `docs/dog/README.md`: final numbers against the targets, misses stated. Recon and mapping only: no weapon, no targeting of people.

**Honesty is the product.** A miss is reported as a miss, with its number. Tests are never loosened to hide a miss; a tuning change is
made on the tuning seeds only, logged with its reason, and the held-out seeds are flown once with the final code.

**Architecture:** SP6 adds no robot software. New harness and tool code only:
`examples/dog/missionmc.py` (full-stack MC per family: one `automission.py` process per seed like `automc.py`, ground-truth fall verdicts,
Wilson, tuning / held-out seed blocks, Markdown + JSON writer), `examples/dog/autonomy/videotap.py` (an opt-in per-frame tap that records the
autonomy and locomotion state the video overlays are drawn from; off by default, never changes the sim), four family scenario sets under
`examples/dog/missions/fam_*`, `tools/record_dog.py` (op log for the viewer capture, exploded-view render, tap-driven diagram frames, sim
replay with an orbit camera, encode). It reuses `tools/record_turret.py` (`build --open`, xvfb viewer capture), `tools/record_cuas_lite.py`
(`Writer` cards and caption strip, `_build_frames`, `_fonts`), `tools/record_skydio.py` (`cmd_oplog` / `cmd_sim` / `cmd_encode` pattern,
`Run`-style trace access) and `tools/videnc.py` (GPU GStreamer VA AV1 `.webm`; ffmpeg VAAPI / libx264 only behind `--gpl`).

**Tech Stack:** Python 3.12, numpy, Pillow (video venv), napl (`Sim`, `napl.terrain`, `napl program`, Design `Backend`, `render`), pytest + xdist,
hfbox via `tools/remote.sh` (CPU slots; one `--gpu` job at a time). No new dependency.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` section 7 (success criteria), section 8 (video), section 9 (risks 1-3).
Inputs: `docs/superpowers/plans/2026-10-16-sp4-locomotion.md`, `docs/superpowers/plans/2026-10-16-sp5-autonomy.md`, `docs/dog/{speed-sweep,actuator-study,autonomy}.md`,
`docs/skydio/video.md` (the recorder pattern). Worktree `../napl-wt/sp6`, branch `p12-sp6` from `napl-v1`; remote dir `/workspace/wt/napl-skydio`.

## State of the inputs when this plan was written (2026-10-17) and what SP6 consumes by exact name

SP3b and SP5 Tasks 1-10 are merged on `napl-v1` (head 559c2e0). **SP4 (`p12-loco`) has only its SP3b ancestry committed (no `examples/dog/loco/`,
no `dogtruth.py`, `dogcourse.py`, `dogwalk.py`, `dogmc.py`, no `docs/dog/locomotion.md`) and SP5 Task 11 (`dogloco.py`, `body = "dog"`, `dog_smoke`,
`coverage_rubble_dog`) is not written.** SP6 Task 1 gates on both. Names SP6 uses, all from those plans (Task 1 Step 1 re-greps each in the merged
code and a renamed one is fixed in SP6 files, never in SP4 / SP5):

| From        | Name                                                                                                                                                                                                                                                                                                                                                       | Used for                             |
| ----------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| SP5 merged  | `autonomy.scene.scenario(path) -> (auto, prm, sc)`, `scene.run(model, scenario_path, out, t_max=None) -> summary dict`, `scene.COLS` (incl. `fallen`, `coverage`, `rms`, `phase`, `bumps`, `distance`), `summary.json`, `examples/dog/automission.py`, `automc.py` (`one`, `reasons`, subprocess + `NAPL_PARAMS`), `land_mc.wilson` (examples/skydio-land) | MC runner, summaries                 |
| SP5 merged  | `mission.Robot` (`.map` ElevationMap with `.height/.var/.mapped(pool)`, `.loc.x`, `.path`, `.ex.phase`, `.trav.cost`, `.sm`), `[plan]` keys `lane_m`, `return_margin`, `t_budget`, `pool`, `trav.block`                                                                                                                                                    | tap, scenarios                       |
| SP5 Task 11 | `scene.build` / `_loco` `body = "dog"` branch, `dogloco.DogLoco` (`.ctl`, `.tr`, `.fallen`, `.distance`, `.sim`), `missions/dog_smoke`, `missions/coverage_rubble_dog`                                                                                                                                                                                     | every dog mission                    |
| SP4 plan    | `loco.controller.Controller` attributes `.mode` (`STANDUP/STAND/WALK/FALLEN/SIT`), `.st` (`.p .R .v .w .feet .contact`), `.f` (4 x 3 MPC forces), `.target` (4 x 3 touchdown targets), `.lift`, `.sched` (gait name via `.sched.g.name`), `.stats` (`n_slip mpc_fail sat_hip sat_knee`), `.est.x`, `.mu`                                                   | tap + programming diagrams           |
| SP4 plan    | `dogcourse.base_model()`, `dogtruth.Truth.fallen()`, `docs/dog/locomotion.md` (top speed, rocky mean speed, criterion course 25 deg + 15 cm, SP4 terrain MC)                                                                                                                                                                                               | results page, README                 |
| SP3b merged | `assemblies/dog/{dog,leg,head,power,compute}.toml`, `examples/dog/dogsim.py` `build(out) -> (xml, report)`, report BOM / mass, `docs/dog/speed-sweep.md`                                                                                                                                                                                                   | video part (a), results (cost, mass) |

## Interface gaps found (flagged for the orchestrator; Task 1 Step 1 confirms each)

1. **No per-frame hook in `scene.run`** and `summary.json` is only the LAST row: a fall that ends in safe-sit and a later `fallen = 0` would be lost, and the video
   cannot get the map, path or gait state. Task 2 adds an optional `tap` hook (env `DOG_TAP_DIR`, off by default) and one additive summary key `fell_ever`
   (max of the `fallen` column). Neither changes the sim, a trace column, or any golden hash.
2. **SP4's `dogmc.py` is not the full-stack MC**: it runs 30 x 30 m fields with the lidar OFF along ground-truth routes (locomotion only). SP6's MC is the whole stack
   (lidar on, SP5 planner, SP5 map as SP4's `HeightMap`). The two answer different questions and both go in the README.
3. **SP5 mission criteria currently fail** (autonomy.md): rubble held-out coverage 0/16 (median 93.7 %, lane 8 m), 20k headline 89.0 % at pool 1, ruins 26 % mapped with 7441
   bumps (blocks 0.3 m). SP6 depends on SP5's "fix round"; if it has not landed, Task 5 reports these as measured misses on the dog and does not wait.
4. **Speed**: `docs/dog/speed-sweep.md` bounds a trot at 1.36 m/s (derived, chosen point gim8108-8 r2.0 L200) and the BOM is 5,554 USD against 3-4.5 k; SP5's missions assume 1.5 m/s
   (`carrier.v_max`) and 20 min. Whether the dog can finish 100 x 100 m in 20 min depends on SP4 Task 10's measured rocky speed. SP6 reports, it does not retune the hardware.
5. **Slopes are 10x costlier to simulate** (autonomy.md: slopes 5k = 0.31 sim-s per wall-s on the carrier) and there is no 100 m slopes mission: slopes MC runs on 30 m fields (Task 1 sizes N).
6. **No exploded-view op in the viewer** (grep of `python/napl/design/uiops.py`, `viewer.py`, `viewer.cpp` for `explode`: none). Part (a) shows sub-assemblies joining with
   `ui.visibility` + `sub.open` in the real viewer, and renders the exploded view offscreen from a displaced copy of the built MJCF (Task 6). Wires are not in that render.
7. **Held-out seeds already used**: SP5 flew rubble 1001-1016, SP4 reserves 21000+. SP6 uses a fresh pair of blocks (tuning 5001-5030, held-out 31001+).

## Global Constraints

- **Ruins headline is 50 x 50 m (user decision 2026-10-17, spec section 7):** wherever this plan says `dog_ruins` is 100 x 100 m
  (Task 1 files and test, Task 5 Step 3), use 50 x 50 m (~100 rooms) with the same 20 min / RMS 5 cm / 95 % criteria; rename
  the test `test_dog_ruins_headline_is_50m_with_fine_blocks`. Base it on SP5's 50 m ruins scenario if one exists in the merged tree.
- **All compute on hfbox:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '<cmd>'` (`NAPL_REMOTE_DIR` always set; SP6's remote dir is
  `/workspace/wt/napl-skydio`, worktree `../napl-wt/sp6`, branch `p12-sp6` from `napl-v1`). `--slots 2` for Monte Carlo, `bg` for runs over ~50 min (`run` times out at 3600 s),
  `--gpu` only for render / encode; ONE GPU job at a time; GTT < 4 GB (read `/sys/class/drm/card1/device/mem_info_gtt_used` before and after, the recorder prints it).
  Remote logs go to `../logs`. Locally: edits, git and reading only.
- **Build once per task that needs a fresh tree** (inside the remote command): `cmake --preset dev && cmake --build --preset dev`, then
  `PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh`.
- **USER RULE: targeted tests only.** Each task runs only the test files it names, `-n 8` at most. Never the full Python suite.
- **Edit project files with Edit / Write only** (no `sed -i`, heredocs, scripts that write files). Results pulled from hfbox (`tools/remote.sh pull`) are read with Read and written into the
  tree with Write.
- **No GPL dependency** (LGPL dynamically linked only). The video encoder is the GStreamer VA path by default; ffmpeg VAAPI / libx264 only behind `--gpl` (the `record_build` /
  `videnc.Encoder(gpl=)` pattern). `.venv/bin/python tools/check_licenses.py` reports 0 problems at the end (Task 10).
- **Never commit media:** no frames, no `.rgz` / `.png` / `.webm` / `.mp4`, no screenshots, no `.npz` taps. Frames, taps and the video stay under `$NAPL_REMOTE_DIR/out/dog/`. Tests check
  structure, timing and determinism (timelines, op logs, hashes, layout numbers), never pixels. `git status` shows no media before each commit.
- **Numbers rule** for any part / assembly / scenario-`[auto]` number this plan adds: `{ value, unit, fidelity, rationale }` (`fidelity` one of `datasheet measured literature estimated design`);
  `autonomy.config.leaves` refuses a bare leaf. SP6 adds no part or assembly TOML. The `napl program` runner keys (`[limits]`, monitor `lo` / `hi`, `[[parameters]]`) stay bare with a `#` source comment,
  as in every existing scenario.
- **Determinism:** same seed, same state hash; every harness sets `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1` before numpy; random draws from `np.random.default_rng([seed, <stream>])`.
- **Seeds:** TUNING block `5001-5030` (any change to a parameter is made against these only, each change logged in `docs/dog/missions.md` "Tuning log" with its reason); HELD-OUT block
  `31001+`, flown once per family with the final code; a held-out seed is never rerun after a change (a changed code means a new block `32001+`, stated). SP5's `1001-1025` and SP4's `21000+` are
  not reused.
- **No test is loosened to pass a miss.** A failed criterion appears in the README as NOT MET with its measured value and Wilson bound; a floor test pins only what the final code did.
- Every commit message ends with `Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5`. The orchestrator commits this plan file; implementers commit their own tasks.
- Recon only: no weapon, launcher or payload-release feature, nothing that targets people; the video's captions say so once.
- Every existing test and golden hash stays green; SP6 changes no kernel code and no robot module.

## Review Focus

1. The MC verdict is ground truth and no run disappears: crashed, timed-out and not-home runs stay in the denominator; a fall is `fell_ever` (max of the ground-truth `fallen` column), not the last
   row, not the controller's own detector. `tests/python/test_dog_missionmc.py::test_summary_counts_every_run_and_uses_ever_fell` (Task 3).
2. Seed discipline: tuning and held-out blocks are disjoint from each other and from SP4 / SP5's blocks, and a results file refuses a held-out row whose code revision differs from the final one.
   `tests/python/test_dog_missionmc.py::test_seed_blocks_are_disjoint_and_heldout_needs_final_rev` (Task 3).
3. The tap is read-only: a run with `DOG_TAP_DIR` set has the same state hash and trace as one without. `tests/python/test_dog_tap.py::test_tap_does_not_change_the_run` (Task 2).
4. Video overlays come from real run data: every diagram number in the programming part is read from the tap / trace, and a stale tap (hash differs from the run's) is refused.
   `tests/python/test_dog_video.py::test_overlay_numbers_come_from_the_tap_and_stale_tap_is_refused` (Task 7).
5. The replay behind the sim frames is the run: the recorder ends with a replay-hash check and exits non-zero on a mismatch; the exploded-view MJCF keeps every body of the built model.
   `tests/python/test_dog_video.py::test_explode_keeps_every_body_and_moves_only_groups` (Task 6).
6. Results page numbers are generated from the MC / mission JSON, and every spec criterion is MET or NOT MET with its measured value; a criterion with no data says "not measured".
   `tests/python/test_dog_missionmc.py::test_results_table_marks_every_criterion` (Task 3).

## Tasks

Every test file starts with:

```python
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
sys.path.insert(0, str(ROOT / "tools"))
```

### Task 1: gate, family scenarios, and the sizing of the Monte Carlo from measured speed

**Files:**

- Create: `examples/dog/missions/fam_rubble/fam_rubble.scenario.toml` + `fam_rubble.py`, and the same for `fam_slopes`, `fam_scree`, `fam_ruins`;
  `examples/dog/missions/dog_ruins/dog_ruins.scenario.toml` + `dog_ruins.py` (100 x 100 m ruins on the dog, `block = 0.3`); `docs/dog/missions.md` (skeleton: "Budget" section)
- Test: `tests/python/test_dog_missions_cfg.py`

**Interfaces:**

- Consumes: `autonomy.scene.scenario`, `config.leaves`, `missions/coverage_rubble_dog` (SP5 Task 11) as the template for `[auto]`, `[program]`, `[limits]`, monitors.
- Produces: scenario names `fam_rubble`, `fam_slopes`, `fam_scree`, `fam_ruins` (30 x 30 m, `body = "dog"`, `rays = 5000`, `patch = 5.0` m for rubble / slopes / scree, `10.0` for ruins per
  autonomy.md, `pool = 2`), `dog_ruins` (100 x 100 m, `block = 0.3`), and the budget table N per family in `docs/dog/missions.md`.

- [ ] **Step 1: gate and name check.** Run locally:
      `git log --oneline napl-v1 | grep -m1 "Merge p12-loco"; git log --oneline napl-v1 | grep -i -m1 "SP5.*dog\|dogloco"; ls examples/dog/loco/controller.py examples/dog/dogtruth.py examples/dog/dogcourse.py examples/dog/autonomy/dogloco.py examples/dog/missions/dog_smoke examples/dog/missions/coverage_rubble_dog`.
      Any missing: STOP and report which (SP6 Tasks 1-10 all need a walking, mapping dog; nothing here can be faked). Then
      `grep -n "self.mode\|self.f =\|self.target\|self.lift\|self.sched\|self.stats\|self.st =" examples/dog/loco/controller.py` and
      `grep -n "contact\|feet" examples/dog/loco/estimator.py | head`, and confirm the names of the interface table above; write any rename into this plan's table and use it in Tasks 2 and 7.
- [ ] **Step 2: failing test** `tests/python/test_dog_missions_cfg.py`:

```python
import tomllib

import pytest
from autonomy import scene

MIS = ROOT / "examples" / "dog" / "missions"
FAMS = {"fam_rubble": "rubble", "fam_slopes": "slopes", "fam_scree": "scree", "fam_ruins": "ruins"}


@pytest.mark.parametrize("name,kind", FAMS.items())
def test_family_scenarios_are_30m_dog_runs_with_numbers_rule(name, kind):
    auto, prm, sc = scene.scenario(MIS / name / f"{name}.scenario.toml")  # config.leaves refuses a bare number
    assert auto["kind"] == kind and auto["body"] == "dog" and auto["size"] == 30.0
    assert auto["rays"] <= 5000 and auto["patch"] in (5.0, 10.0)  # sim budget: lidar dominates (autonomy.md)
    assert prm["seed"] >= 1 and sc["name"] == f"dog-{name.replace('_', '-')}"
    assert {m["name"] for m in sc["monitors"]} >= {"never_falls", "map_rms_within_5cm"}


def test_dog_ruins_headline_is_100m_with_fine_blocks():
    auto, _, sc = scene.scenario(MIS / "dog_ruins" / "dog_ruins.scenario.toml")
    assert auto["size"] == 100.0 and auto["block"] == 0.3 and auto["t_budget"] == 1200.0
    assert any(m["name"] == "covers_95_percent" for m in sc["monitors"])
```

- [ ] **Step 3: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_missions_cfg.py -n 8'`
- [ ] **Step 4: write the five scenarios.** Each is `coverage_rubble_dog.scenario.toml` with: `name` as the test says; `[auto] kind`, `size` (`{ value = 30.0, unit = "m", fidelity = "design", rationale = "SP6 family MC: ~11x fewer lidar-hit cells than 100 m (docs/dog/missions.md budget)." }`), `body = "dog"`, `patch`,
      `rays = 5000`, `pool = 2`, `t_budget` (`{ value = 300.0, unit = "s", fidelity = "estimated", rationale = "30 m field: ~4 lanes of 30 m at the dog's measured rocky speed (SP4) plus fill and return; Task 1 Step 6 replaces it by the measured value." }`),
      start `-12.0 / -12.0 m, 0.0 rad` (3 m inside the south-west corner of the 30 m field, the same rule as the 100 m files' 4 m), `return_margin` as `coverage_rubble_dog`; `fam_ruins` and `dog_ruins` set `block = { value = 0.3, unit = "m", fidelity = "measured", rationale = "autonomy.md: 0.5 m blocks read 0.8-1.6 m doors closed." }`.
      Monitors: `never_falls` (`column = "fallen"`, `op = "always"`, `hi = 0`), `map_rms_within_5cm`, `localized_within_30cm`, `covers_95_percent`, `returns_home` (copied from `coverage_rubble_dog`; each bound keeps its `#` source comment).
      `[limits]` `wall_s = 3600` for the 30 m files, `7200` for `dog_ruins` (`# ruins 5k ~ 11-13 min carrier x 3-4 for the dog, autonomy.md`). Each `.py` is the 3-line model script of `coverage_rubble_dog.py`.
- [ ] **Step 5: run, expect PASS** (same command as Step 3).
- [ ] **Step 6: measure the budget and write it.** One seed per family, tuning block (`5001`), via `automission.py` (`AUTO_T_MAX=60` caps the run to a 60 sim-s probe):
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run --slots 2 'for f in rubble slopes scree ruins; do /usr/bin/time -f "$f %e s wall" env AUTO_T_MAX=60 NAPL_PARAMS={\"seed\":5001} .venv/bin/python examples/dog/automission.py examples/dog/missions/fam_$f/fam_$f.scenario.toml ../logs/probe_$f; done 2>&1 | tail -20'`.
      Wall per sim-s `w_f = wall_f / 60` per family. In `docs/dog/missions.md` ("Budget") write the table and **N per family = min(36, floor(6 h x 2 slots / (T_budget_f x w_f)))**, `T_budget_f` the scenario budget (replace the 300 s estimate by 1.3 x the probe's measured time-to-home if the probe got home, edit the scenarios accordingly). 36 is the smallest N at which a clean 36/36 has a Wilson lower bound
  > = 0.90 (n / (n + 3.84) = 0.904). A family with N < 36 states in the README that its lower bound cannot reach 90 % even with no falls. The 100 m missions are not run here (Task 5).
- [ ] **Step 7: commit** `git add examples/dog/missions/fam_* examples/dog/missions/dog_ruins docs/dog/missions.md tests/python/test_dog_missions_cfg.py && git commit -m "dog missions: four 30 m family scenarios and the 100 m ruins mission on the dog, MC sizing from measured sim speed" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 2: read-only tap and ever-fell summary (`videotap.py`, `scene.run` hook)

**Files:**

- Create: `examples/dog/autonomy/videotap.py`
- Modify: `examples/dog/autonomy/scene.py` (`run`: tap call, `fell_ever` in the summary)
- Test: `tests/python/test_dog_tap.py`

**Interfaces:**

- Consumes: `scene.run` locals `k, car, rob`; SP5 `rob.map.mapped(pool)`, `rob.loc.x`, `rob.path`, `rob.ex.phase`; SP4 `car.ctl` attributes of the table above (read with `getattr(..., None)`: a missing attribute is stored as `nan`, never raises).
- Produces: `videotap.Tap(dir, every=5, map_every_s=10.0, pool=2)` callable `(k, car, rob)`; `Tap.close()` writes `<dir>/tap.npz` + `tap.json` (`n`, `dt`, `state_hash`, `cols`); `scene.run` calls `Tap` when `DOG_TAP_DIR` is set; `summary["fell_ever"]` (0 / 1).
  Tap arrays: `t, truth_xy, est_xy, est_yaw, phase, mode (int code), gait (int code), contact (n,4), force_z (n,4), foot_xy (n,4,2 world touchdown targets), cov, path` (list of (m,2) float32 in an object-free ragged form: `path_len` + concatenated `path_xy`), `map_t`, `mapped_packed` (np.packbits of the pooled mapped mask), `height_q` (map height at 25 cm, last snapshot only).

- [ ] **Step 1: failing test** `tests/python/test_dog_tap.py`:

```python
import json
import os

import numpy as np
import pytest

import napl
from autonomy import scene, videotap

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
SMOKE = ROOT / "examples" / "dog" / "missions" / "coverage_smoke" / "coverage_smoke.scenario.toml"  # carrier: no SP4 needed


class FakeMap:
    n = 8

    def mapped(self, pool):
        return np.eye(4, dtype=bool)

    def height(self, x, y):
        return np.zeros(np.shape(x))


class FakeRob:
    map, path = FakeMap(), [(0.0, 0.0), (1.0, 0.0)]
    loc = type("L", (), {"x": np.array([1.0, 2.0, 0.3, 0.4])})()
    ex = type("E", (), {"phase": 1})()


class FakeCar:  # a carrier: no .ctl, the tap must store nan for the gait columns
    def truth(self):
        return np.array([1.01, 2.02, 0.3]), np.array([1.0, 0, 0, 0])

    time_of_last_frame = 0.1


def test_tap_stores_nan_for_missing_gait_and_keeps_the_path(tmp_path):
    tap = videotap.Tap(tmp_path, every=1, map_every_s=0.0)
    tap(0, FakeCar(), FakeRob())
    tap.close(state_hash="abc", dt=0.1)
    z = np.load(tmp_path / "tap.npz")
    assert z["est_xy"].shape == (1, 2) and np.isnan(z["contact"]).all() and z["path_len"].tolist() == [2]
    assert json.loads((tmp_path / "tap.json").read_text())["state_hash"] == "abc"


@needs_kernel
def test_tap_does_not_change_the_run(tmp_path, monkeypatch):  # Review Focus 3
    from napl import program

    base, _ = program.run(SMOKE, tmp_path / "a")
    monkeypatch.setenv("DOG_TAP_DIR", str(tmp_path / "tap"))
    tapped, _ = program.run(SMOKE, tmp_path / "b")
    assert base["sim"]["state_hash"] == tapped["sim"]["state_hash"]
    assert (tmp_path / "tap" / "tap.npz").exists()
    monkeypatch.delenv("DOG_TAP_DIR")


def test_summary_has_fell_ever_even_if_the_last_row_is_standing():
    rows = [[0.0] * len(scene.COLS) for _ in range(3)]
    rows[1][scene.COLS.index("fallen")] = 1.0
    assert scene.fell_ever(rows) == 1 and scene.fell_ever(rows[:1]) == 0
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_tap.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/videotap.py`:**

```python
"""Opt-in per-frame tap for the video (SP6): reads the autonomy and locomotion state a diagram needs, never writes to the sim.
Enabled by scene.run when DOG_TAP_DIR is set. Missing attributes (a carrier has no legs) are stored as nan."""

import json
from pathlib import Path

import numpy as np

MODES = ["STANDUP", "STAND", "WALK", "FALLEN", "SIT"]
GAITS = ["stand", "walk", "trot", "flying_trot", "bound"]  # SP4 gait names (loco.gait); an unknown name is code -1


def _g(o, *names):
    for n in names:
        o = getattr(o, n, None)
        if o is None:
            return None
    return o


class Tap:
    def __init__(self, out, every=5, map_every_s=10.0, pool=2):
        self.dir, self.every, self.map_every, self.pool = Path(out), int(every), float(map_every_s), int(pool)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.r, self.path_xy, self.maps, self.map_t, self.last_map = [], [], [], [], -1e9

    def __call__(self, k, car, rob):
        if k % self.every:
            return
        p, _ = car.truth()
        t, x = float(car.time_of_last_frame), rob.loc.x
        ctl = getattr(car, "ctl", None)
        contact = _g(ctl, "st", "contact")
        force = _g(ctl, "f")
        tgt = _g(ctl, "target")
        mode, gait = _g(ctl, "mode"), _g(ctl, "sched", "g", "name")
        nan4 = np.full(4, np.nan)
        self.r.append(dict(
            t=t, truth_xy=p[:2], est_xy=x[:2], est_yaw=float(x[3]), phase=int(rob.ex.phase),
            mode=MODES.index(mode) if mode in MODES else -1, gait=GAITS.index(gait) if gait in GAITS else -1,
            contact=np.asarray(contact, float) if contact is not None else nan4,
            force_z=np.asarray(force, float)[:, 2] if force is not None else nan4,
            foot_xy=np.asarray(tgt, float)[:, :2] if tgt is not None else np.full((4, 2), np.nan),
        ))  # fmt: skip
        self.path_xy.append(np.asarray(rob.path, np.float32).reshape(-1, 2))
        if t - self.last_map >= self.map_every:
            self.last_map = t
            self.map_t.append(t)
            self.maps.append(np.packbits(np.asarray(rob.map.mapped(self.pool), bool)))
            self.shape = np.asarray(rob.map.mapped(self.pool)).shape
        self.rob = rob

    def close(self, state_hash, dt):
        cols = {k: np.array([r[k] for r in self.r]) for k in self.r[0]}
        n = len(self.r)
        np.savez_compressed(
            self.dir / "tap.npz", **cols, path_len=np.array([len(p) for p in self.path_xy]),
            path_xy=np.concatenate(self.path_xy) if self.path_xy else np.zeros((0, 2), np.float32),
            map_t=np.array(self.map_t), mapped_packed=np.stack(self.maps),
            mapped_shape=np.array(self.shape),
        )  # fmt: skip
        (self.dir / "tap.json").write_text(json.dumps(dict(n=n, dt=dt, state_hash=state_hash, every=self.every, pool=self.pool,
                                                           cols=sorted(cols))))
```

Every map snapshot has the same shape, so `np.stack` is valid. No height snapshot is stored (the video draws the map's mapped-mask and the truth hfield, not heights; Task 7 documents it).
In `scene.py` add (module level) `def fell_ever(rows): return int(max(r[COLS.index("fallen")] for r in rows) > 0)`; in `run`, after `rob = mission.Robot(...)`:
`tap = videotap.Tap(os.environ["DOG_TAP_DIR"], pool=pool) if os.environ.get("DOG_TAP_DIR") else None` (import `os`, `from . import videotap`; `pool` is defined two lines later in the file: move the tap line below
`pool = int(cfg["plan"]["pool"])`), inside the loop right after `rob.frame(car.time_of_last_frame)`: `if tap is not None: tap(k, car, rob)`, and in the summary dict `"fell_ever": fell_ever(rows)`; after `write_native(...)`:
`if tap is not None: tap.close(state_hash=sim.state_hash(), dt=fdt)` (`sim.state_hash()` is what `write_native` records; if the Sim spells it differently, Step 1's grep of `scene.py` / `napl/marine.py` decides).

- [ ] **Step 4: run, expect PASS:** same command as Step 2 plus `.venv/bin/pytest tests/python/test_dog_autonomy_mission.py -n 8` (the goldens are unchanged).
- [ ] **Step 5: commit** `git add examples/dog/autonomy/videotap.py examples/dog/autonomy/scene.py tests/python/test_dog_tap.py && git commit -m "dog autonomy: opt-in read-only tap for the video (map, path, gait state per frame) and fell_ever in the mission summary" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 3: full-stack Monte Carlo per family (`missionmc.py`) with honest verdicts

**Files:**

- Create: `examples/dog/missionmc.py`
- Test: `tests/python/test_dog_missionmc.py`

**Interfaces:**

- Consumes: `automc.one(scenario, seed, out) -> dict` (subprocess `automission.py`, returns `summary.json` fields or `{"seed", "error"}`), `land_mc.wilson`, `summary["fell_ever"]` (Task 2), the scenario names of Task 1.
- Produces: `missionmc.FAMILIES = {"rubble": "fam_rubble", "slopes": "fam_slopes", "scree": "fam_scree", "ruins": "fam_ruins"}`, `TUNE_BASE = 5001`, `HELD_BASE = 31001`, `seeds(block, n)`, `verdict(row, t_budget) -> list[str]` (empty = mission success; reasons from `crashed fell coverage rms not_home late`), `summarize(rows, t_budget) -> dict` (`n k_nofall k_success wilson_nofall wilson_success fails medians`),
  `CRITERIA` (the spec section 7 rows), `results_table(by_family, extra) -> str`, CLI `python examples/dog/missionmc.py --family all|<name> --block tune|held --n N --jobs J --rev REV --out DIR` writing `<out>/<family>.jsonl`, `<out>/mc.json`, `<out>/mc.md`.

- [ ] **Step 1: failing test** `tests/python/test_dog_missionmc.py`:

```python
import pytest

import missionmc as MC


def row(seed=1, **kw):
    r = dict(seed=seed, coverage=0.96, rms=0.03, phase=4, t=900.0, fell_ever=0, fallen=0, bumps=0, distance=800.0)
    r.update(kw)
    return r


def test_summary_counts_every_run_and_uses_ever_fell():  # Review Focus 1
    rows = [row(1), row(2, fell_ever=1, fallen=0, phase=4), {"seed": 3, "error": "boom"}, row(4, coverage=0.90), row(5, phase=3, t=1300.0)]
    s = MC.summarize(rows, 1200.0)
    assert s["n"] == 5  # the crash is in the denominator
    assert s["k_nofall"] == 3  # seed 2 fell although its last row says standing; the crash is not counted as no-fall
    assert s["k_success"] == 1 and s["fails"]["fell"] == 1 and s["fails"]["crashed"] == 1 and s["fails"]["coverage"] == 1
    assert s["fails"]["not_home"] == 1 and s["fails"]["late"] == 1
    lo, hi = s["wilson_nofall"]
    assert lo < 3 / 5 < hi
    with pytest.raises(ValueError):
        MC.summarize([], 1200.0)


def test_seed_blocks_are_disjoint_and_heldout_needs_final_rev():  # Review Focus 2
    tune, held = set(MC.seeds("tune", 30)), set(MC.seeds("held", 100))
    assert not tune & held
    assert not ((tune | held) & (set(range(1, 31)) | set(range(1001, 1026)) | set(range(21000, 21101))))
    with pytest.raises(ValueError, match="rev"):
        MC.check_heldout_rev([dict(rev="abc"), dict(rev="def")])
    MC.check_heldout_rev([dict(rev="abc"), dict(rev="abc")])


def test_results_table_marks_every_criterion():  # Review Focus 6
    by = {"rubble": MC.summarize([row(i) for i in range(36)], 1200.0)}
    md = MC.results_table(by, {"top_speed_flat": None, "rocky_speed": 1.2, "bom_usd": 5554.0})
    for c in MC.CRITERIA:
        assert c in md
    assert md.count("MET") >= 1 and "NOT MET" in md and "not measured" in md  # rocky 1.2 < 1.5, top speed missing
    assert "36/36" in md and "0.90" in md  # n and the Wilson bound are printed next to the rate
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_missionmc.py -n 8'`
- [ ] **Step 3: implement `examples/dog/missionmc.py`** (key code):

```python
"""Full-stack Monte Carlo of the dog per terrain family (SP6; spec s7: '>= 90 % no-fall per terrain family (MC n per family stated, with a 95 %
lower bound)', 'full autonomous coverage ... <= 20 min, map height RMS <= 5 cm, >= 95 % mapped'). One automission.py process per seed (automc.one).
Verdicts are ground truth: a fall is summary['fell_ever'] (max of the ground-truth `fallen` column), a crash or timeout stays in the denominator.
Seeds: TUNE 5001+ (any tuning, each change logged), HELD 31001+ (flown once with the final code; every row records the code revision)."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse  # noqa: E402
import json  # noqa: E402
import statistics  # noqa: E402
import sys  # noqa: E402
from concurrent.futures import ThreadPoolExecutor  # noqa: E402
from pathlib import Path  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "skydio-land"))
from land_mc import wilson  # noqa: E402

FAMILIES = {"rubble": "fam_rubble", "slopes": "fam_slopes", "scree": "fam_scree", "ruins": "fam_ruins"}
TUNE_BASE, HELD_BASE = 5001, 31001
CRITERIA = [  # (key, text, op, target): the spec section 7 rows SP6 reports
    ("top_speed_flat", "flat top speed >= 3.5 m/s", ">=", 3.5),
    ("rocky_speed", "rocky field mean speed >= 1.5 m/s", ">=", 1.5),
    ("slope_step", "25 deg slopes and 15 cm steps traversed", "bool", True),
    ("nofall", ">= 90 % no-fall per terrain family (Wilson 95 % lower bound shown)", ">=", 0.90),
    ("coverage_mission", "100 x 100 m rocky coverage <= 20 min, RMS <= 5 cm, >= 95 % mapped", "bool", True),
    ("bom_usd", "BOM about 3-4.5 k USD", "<=", 4500.0),
]  # fmt: skip


def seeds(block, n):
    base = {"tune": TUNE_BASE, "held": HELD_BASE}[block]
    return list(range(base, base + n))


def check_heldout_rev(rows):
    revs = {r["rev"] for r in rows}
    if len(revs) != 1:
        raise ValueError(f"held-out rows were flown with different code revisions {sorted(revs)}: rerun on a new seed block")


def verdict(r, t_budget):
    if "error" in r:
        return ["crashed"]
    out = []
    if r.get("fell_ever", r.get("fallen", 0)):
        out.append("fell")
    if not r["coverage"] >= 0.95:
        out.append("coverage")
    if not r["rms"] <= 0.05:
        out.append("rms")
    if r["phase"] != 4:
        out.append("not_home")
    if r["t"] > t_budget:
        out.append("late")
    return out


def summarize(rows, t_budget):
    if not rows:
        raise ValueError("no runs")
    why = [verdict(r, t_budget) for r in rows]
    n = len(rows)
    k_nf = sum("error" not in r and "fell" not in w for r, w in zip(rows, why))
    k_ok = sum(not w for w in why)
    keys = ("crashed", "fell", "coverage", "rms", "not_home", "late")
    ok = [r for r in rows if "error" not in r]
    med = {c: statistics.median(r[c] for r in ok) for c in ("coverage", "rms", "t", "distance")} if ok else {}
    return dict(n=n, k_nofall=k_nf, k_success=k_ok, wilson_nofall=wilson(k_nf, n), wilson_success=wilson(k_ok, n),
                fails={k: sum(k in w for w in why) for k in keys}, medians=med, covs=sorted(r["coverage"] for r in ok))  # fmt: skip
```

`one(family, seed, out, rev)`: `automc.one(HERE/"missions"/name/f"{name}.scenario.toml", seed, out)` plus `rev`. `results_table(by_family, extra)` prints, per family, `k/n`, rate, `Wilson [lo, hi]`, then one line per `CRITERIA`
entry: `MET` / `NOT MET` with the measured value, or `not measured` when `extra[key]` is `None`/absent; the `nofall` row is MET only if the **point rate** is >= 0.90 for every family AND the line also prints the
lower bound (and `lower bound < 0.90` is stated as such, e.g. `MET on rate, lower bound 0.86`); `bool` rows read `extra[key]`. `main()` mirrors `automc.main` (ThreadPoolExecutor over seeds, `runs.jsonl`, `mc.json`, `mc.md`, `--rev` required for `--block held`, a
`--block held` run refuses to start if `<out>/<family>.jsonl` already exists: held-out seeds fly once).

- [ ] **Step 4: run, expect PASS:** same command as Step 2.
- [ ] **Step 5: commit** `git add examples/dog/missionmc.py tests/python/test_dog_missionmc.py && git commit -m "dog missions: full-stack Monte Carlo per terrain family (ground-truth fall verdicts, crashes counted, Wilson bounds, tuning / held-out seed blocks)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 4: tuning-seed runs (diagnosis only) and the tuning log

**Files:**

- Modify: `docs/dog/missions.md` ("Tuning log"); parameters only where a tuning-seed failure names one (never a test)
- Test: `tests/python/test_dog_autonomy_mission.py` (goldens re-recorded ONLY if a tuned parameter changes a golden scenario, each change named in the commit)

**Interfaces:**

- Consumes: `missionmc` CLI, `fam_*` scenarios, `docs/dog/autonomy.md` open issues (lane / fill / return margin, ruins blocks), SP4's `docs/dog/locomotion.md` known gaps.

- [ ] **Step 1: tuning MC, `--block tune`, N = 12 per family** (small: the aim is to find causes, not a rate), in the background:
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh bg --slots 2 dogtune ".venv/bin/python examples/dog/missionmc.py --family all --block tune --n 12 --jobs 8 --rev \$(git rev-parse --short HEAD) --out out/dog/tune0 > ../logs/dogtune.out 2>&1"`;
      `tools/remote.sh log dogtune 40` until finished; `tools/remote.sh pull out/dog/tune0`; Read `mc.md` / `runs.jsonl`.
- [ ] **Step 2: classify every failure** from `runs.jsonl` into `crashed / fell / coverage / rms / not_home / late` per family and look at the cause in the run dirs (`summary.json`, the mission `trace.csv`: `bumps`, `err_xy`, `phase`, `mode`).
      Write a "Tuning log" table in `docs/dog/missions.md`: round, family, symptom (counts), cause (one line), change (file, key, old, new, source of the reasoning), result on the same tuning seeds.
- [ ] **Step 3: at most three tuning rounds**, only on `autonomy.toml` keys (`plan.lane_m`, `plan.return_margin`, fill time, `trav` limits, `loc` gates) or on the SP4 controller's documented gains if the cause is locomotion
      (a change to SP4 code is a separate commit named for SP4 and re-runs SP4's targeted tests `tests/python/test_dog_walk.py tests/python/test_dog_mc.py -n 8`). Each round re-runs `--block tune` with a new `--out out/dog/tuneN`. A change that helps one family and costs another is
      reported, not hidden. Stop when a round gives no new information or after round 3; the remaining misses are the finding.
- [ ] **Step 4: if a tuned value changes a golden** (`test_dog_autonomy_mission.py::GOLDEN`), re-record that hash on hfbox with the program runner, and name it in the commit; any other test change is forbidden here.
- [ ] **Step 5: targeted regression:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_mission.py tests/python/test_dog_autonomy_config.py tests/python/test_dog_missionmc.py -n 8'`
- [ ] **Step 6: commit** `git add docs/dog/missions.md examples/dog/autonomy/autonomy.toml tests/python/test_dog_autonomy_mission.py && git commit -m "dog missions: tuning-seed rounds and the tuning log (causes, changes, what stayed open)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`
      (list only the files actually changed; if nothing was tuned the commit is the log alone.)

### Task 5: held-out Monte Carlo and the 100 x 100 m headline missions

**Files:**

- Modify: `docs/dog/missions.md` ("Held-out Monte Carlo", "100 m missions")
- Create: `docs/dog/missions.json` (generated numbers: MC rows summary, mission summaries, code revision)
- Test: `tests/python/test_dog_missionmc.py::test_results_table_marks_every_criterion` (already green; the doc is checked against it in Task 10)

**Interfaces:**

- Consumes: final code revision `REV=$(git rev-parse --short HEAD)` (recorded in every row), `N_f` from Task 1's budget table.

- [ ] **Step 1: freeze.** `git status` is clean, `git rev-parse --short HEAD` is `REV`; no later code change is allowed without a new block (`32001+`, stated in the doc).
- [ ] **Step 2: held-out family MC, once:** per family in turn so a crash does not lose the others (the runner refuses to rerun an existing family file):
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh bg --slots 2 doghold ".venv/bin/python examples/dog/missionmc.py --family all --block held --n <N_f per family as the Task 1 table, one --n per --family call> --jobs 8 --rev REV --out out/dog/held > ../logs/doghold.out 2>&1"`
      (the runner takes `--n` per family as `--n rubble=36,slopes=24,...`; write the call with the Task 1 values filled in). `tools/remote.sh log doghold 40` until done; `pull out/dog/held`; Read `mc.md`.
- [ ] **Step 3: 100 x 100 m headline missions on the dog** (the spec's mission criterion; unlike the family MC these are the full 20-minute missions): rubble `coverage_rubble_dog` seeds `31001-31006`, ruins `dog_ruins` seeds `31001-31004`
      (6 + 4 runs; 35-45 min wall each at 5k, SP5 Task 11 estimate; two at a time with `--slots 2` is about 4 h, state the real figure):
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh bg --slots 2 doghead ".venv/bin/python examples/dog/automc.py examples/dog/missions/coverage_rubble_dog/coverage_rubble_dog.scenario.toml --seeds 31001:31007 --jobs 2 --out out/dog/head_rubble && .venv/bin/python examples/dog/automc.py examples/dog/missions/dog_ruins/dog_ruins.scenario.toml --seeds 31001:31005 --jobs 2 --out out/dog/head_ruins > ../logs/doghead.out 2>&1"`.
      One of the rubble runs (seed 31001) is rerun at 20k rays / pool 1 only if the box is idle afterwards (SP5's 20k headline is the density-equivalence check, not a criterion); say so if skipped.
- [ ] **Step 4: write the results** into `docs/dog/missions.md` verbatim from `mc.md` / `report.md`: per family k/n, no-fall rate and Wilson 95 % interval, failure causes, medians of coverage / RMS / time / distance; the headline missions with all run values and the best / median / worst; wall times measured. Write `docs/dog/missions.json` with the same numbers
      (Write tool, from the pulled JSON). If any criterion misses, the section's first sentence says which and by how much; no sentence calls a miss "close" without the number.
- [ ] **Step 5: commit** `git add docs/dog/missions.md docs/dog/missions.json && git commit -m "dog missions: held-out per-family Monte Carlo and the 100 m rubble / ruins missions on the dog, results as measured" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 6: video part (a), construction: op log, explored build, exploded view (`tools/record_dog.py` oplog / explode)

**Files:**

- Create: `tools/record_dog.py` (module docstring, constants, `STAGES`, `cmd_oplog`, `explode_xml`, `cmd_explode`, `main`)
- Test: `tests/python/test_dog_video.py`

**Interfaces:**

- Consumes: `assemblies/dog/dog.toml` instances (flattened ids `fl:...`, `fr:...`, `rl:...`, `rr:...`, `head:...`, `power:...`, `compute:...`, chassis `deck_top bulk_f bulk_r side_l side_r`), Backend tools `ui.mode`, `ui.visibility`, `ui.camera`, `camera.orbit`, `ui.wire_pick`, `flatten.begin/end`, `ui.wire_view`, `sub.open`, `sub.close`, `design_check`
  (as used in `record_skydio.cmd_oplog`), `dogsim.build(out) -> (xml, report)`, `record_turret.py build --open assemblies/dog/dog.toml` (writes `build/f*.rgz` + `timeline.json`).
- Produces: `record_dog.STAGES` (list of `(title, why, group-prefixes)` in build order: chassis, leg (fl) then the mirrored fr/rl/rr, head, power, compute), `cmd_oplog(a)` writing `{"about","steps"}` JSON, `explode_xml(xml_text, groups, factor) -> str`, `cmd_explode(a)` rendering `OUT/x*.rgz` (offscreen sim.render, orbit camera) + `explode.json`.

- [ ] **Step 1: probe the viewer ops and the MJCF structure** (read-only, remote CPU): `grep -n "explode" python/napl/design/uiops.py python/napl/design/viewer.py src/viewer/viewer.cpp` (expected: none) and
      no inline scripts (no heredocs): the probe is `tools/record_dog.py probe` (a subcommand that builds `dogsim.build(tmp)` and prints, for each body name prefix group, the parent body name and whether the group's top body is a child of the root body or of `worldbody`).
      Record the printed answer in the docstring of `explode_xml` (the displacement is applied to each group's topmost body `pos`; if the groups nest, offsets are computed relative to the parent so the world displacement is the intended vector).
- [ ] **Step 2: failing test** `tests/python/test_dog_video.py`:

```python
import json
import xml.etree.ElementTree as ET

import pytest

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


def test_oplog_steps_are_timed_and_ordered(tmp_path):
    out = tmp_path / "oplog.json"
    RD.main(["oplog", str(out)])
    steps = json.loads(out.read_text())["steps"]
    segs = [s["seg"] for s in steps]
    assert segs.index("BUILD") < segs.index("WIRING") < segs.index("PROGRAM")
    assert sum(r["wait"] for s in steps for r in s["reqs"]) <= RD.PART_A_MAX_S  # part (a) stays inside its time budget
    assert any(r["tool"] == "sub.open" for s in steps for r in s["reqs"])  # the leg is opened as its own document
    assert all(r["tool"] != "ui.frame" for s in steps for r in s["reqs"])  # ui.frame selects what it frames (record_skydio finding)
    assert steps == json.loads(json.dumps(steps))  # plain JSON, deterministic
    RD.main(["oplog", str(tmp_path / "again.json")])
    assert (tmp_path / "again.json").read_text() == out.read_text()


@needs_kernel
def test_explode_keeps_every_body_and_moves_only_groups(tmp_path):  # Review Focus 5
    import dogsim

    xml, _ = dogsim.build(tmp_path)
    text = xml.read_text()
    groups = {"fl:": (0.3, 0.3, 0.0), "head:": (0.4, 0.0, 0.2)}
    out = RD.explode_xml(text, groups, 1.0)
    bodies = lambda t: sorted(b.get("name") for b in ET.fromstring(t).iter("body"))
    assert bodies(out) == bodies(text)
    assert RD.explode_xml(text, groups, 0.0) == text or bodies(RD.explode_xml(text, groups, 0.0)) == bodies(text)
    moved = RD.world_pos(out) ; base = RD.world_pos(text)
    assert all(abs(moved[k] - base[k]).max() < 1e-9 for k in base if not k.startswith(("fl:", "head:")))
    assert abs(moved["fl:thigh"] - base["fl:thigh"] - (0.3, 0.3, 0.0)).max() < 1e-9
```

(`world_pos(xml_text) -> {body name: np.array(3)}` is a test-support function in `record_dog.py` that composes `pos` down the body tree; frames are not rotated, only translations are compared.)

- [ ] **Step 3: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_video.py -n 8'`
- [ ] **Step 4: implement the op log** (mirror `record_skydio.cmd_oplog`). Constants: `W, H, FPS = 1280, 720, 20`, `ASM = REPO / "assemblies" / "dog" / "dog.toml"`, `PART_A_MAX_S = 170.0`. `STAGES` (title `n/6 ...`, why, groups):
  1. `1/6  Chassis: carbon deck, bulkheads, side plates` (`["chassis"]`, instances `deck_top bulk_f bulk_r side_l side_r`);
  2. `2/6  One leg: abduction, hip and knee QDD actuators, crank + push rod, thigh, shank, foot` (`["fl"]`, opened with `sub.open {"id": "fl"}` so the leg is shown as its own document, parts revealed in the order `abad_mount, abad_motor, hip_housing, hip_motor, knee_motor, knee_crank, thigh, knee_rod, shank, foot_core, foot_ball, hub`, then `sub.close {"save": false}`);
  3. `3/6  The leg used four times: fr and rl mirrored, rl and rr turned 180 deg` (`["fr","rl","rr"]`, the mirror and the rear turn named in `why` from `assemblies/dog/README.md` layout rules);
  4. `4/6  Head: Livox Mid-360 and RealSense D435i` (`["head"]`); 5. `5/6  Power: two 8S packs, BMS, contactor, PDB, DC-DCs, e-stop` (`["power"]`); 6. `6/6  Compute: Jetson Orin Nano, four CANables, IMU, RTK board` (`["compute"]`).
     A `group -> instance ids` helper expands `"fl"` to every id starting `fl:` from the TOML-flattened design (`Design.open(ASM)` instance ids; `chassis` is the explicit five). Each stage: `ui.visibility {instances, visible:true}` (wait 0.6) + `camera.orbit {yaw_deg:75, pitch_deg:6, seconds:4.0, relative:true}` (wait 4.2), plus the leg step's `sub.open` / `sub.close`.
     Then `Mass and DRC` (`design_check`; why quotes the build report: 11.65 kg, BOM 5,554 USD, read from `dogsim.build` report, not typed), then WIRING: sheet 1 `power` (`power:*` -> one `leg` hub: `ui.wire_pick`, `flatten.begin`), sheet 2 `CAN daisy chain of the front-left leg` (instances `compute:jetson compute:can_fl fl:hub fl:abad_motor fl:hip_motor fl:knee_motor`, instance ids resolved from the design, not guessed: `cmd_oplog` asserts they exist),
     the hide-parts demo (`ui.visibility` hides `power:*`: cards and wires vanish in 2D and 3D, then shown), and a closing 3D orbit with `ui.view emphasise_wires`: "153 routed tubes, none through a part". The PROGRAM segment is the short one: `planner.py` and `loco/controller.py` open in the IDE for 10 s each (`prog.ui open`), no terminal demo (part (b) carries the programming story).
     The sum of waits must be <= `PART_A_MAX_S` (test); durations are the budget of part (a) in the final video (see Task 9 timeline).
- [ ] **Step 5: implement `explode_xml` and `cmd_explode`:** `explode_xml(xml_text, groups, factor)`: parse MJCF, for each body whose `name` starts with a group prefix and whose parent body does not (the group's topmost body) add `factor * vector` to its `pos` in the parent's frame (vector is given in world axes; if the parent frame is rotated, rotate the vector by the transpose of the parent's accumulated rotation, `world_pos` and the Step 1 probe define this); `factor = 0` returns the input unchanged.
      `cmd_explode(a)`: `Sim.load` on the exploded MJCF for factors `0 -> 1 -> 0` in 40 steps (parts "fly" apart then join), `napl.render_style("studio")` (as `tools/design_shot.py` does; Step 1 of this step: read `design_shot.grab`/`render_style` usage and copy its call), orbit camera `yaw = 40 + 3 * k` degrees at 1.4 m, frames `OUT/x00000.rgz` via `videnc.save_frame`, `explode.json` listing per-frame factor and camera. GPU command: `tools/remote.sh run --gpu`.
      The vectors: `fl:`/`fr:`/`rl:`/`rr:` outward 0.35 m along the leg's side and 0.1 m down (signs from the leg's world x/y at factor 0), `head:` +0.35 m x +0.15 m z, `power:` -0.3 m z, `compute:` +0.25 m z.
- [ ] **Step 6: run, expect PASS** (Step 3 command), then capture-check on the GPU lane (viewer through xvfb, ~3 min):
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/python tools/record_dog.py oplog out/dog/oplog.json'` then
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run --gpu '.venv/bin/python tools/record_turret.py build out/dog/oplog.json out/dog/build --open assemblies/dog/dog.toml --timeout 2800 && cat /sys/class/drm/card1/device/mem_info_gtt_used'` (GTT < 4 GB = 4294967296; write the printed value in `docs/dog/video.md` in Task 9) and
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py explode out/dog/explode'`. If `record_turret build --open` fails on a `sub.open` / `sub.close` step (that pair is new in a recorder), fix the step order in `cmd_oplog`, not the viewer.
- [ ] **Step 7: commit** `git add tools/record_dog.py tests/python/test_dog_video.py && git commit -m "record_dog: construction part (sub-assembly build stages, leg as its own document, pin-level wiring sheets, exploded-view render)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 7: video part (b), the programming overview as diagrams driven by the tap (`record_dog.py` diagram frames)

**Files:**

- Modify: `tools/record_dog.py` (`Tap` reader, `DIAGRAMS`, `draw_*`, `cmd_diagrams`)
- Test: `tests/python/test_dog_video.py` (extend)

**Interfaces:**

- Consumes: a mission run dir from Task 5 recorded with `DOG_TAP_DIR=<run>/tap` (`tap.npz`, `tap.json` with the run's `state_hash`), the run's `trace.csv` / `result.json`, SP4's gait and force names (table above), SP5 `autonomy.toml` numbers for the labels (rates read from `config.load()`, never typed: lidar rate, MPC rate 50 Hz and WBC rate 500 Hz come from SP4's `loco.controller` constants `DT`, `MPC_EVERY`).
- Produces: `record_dog.TapData(run_dir)` (loads and checks `tap.json["state_hash"] == result.json["sim"]["state_hash"]`, else `SystemExit("stale tap")`), `DIAGRAMS` (ordered list of `(key, title, blurb, seconds)`: `estimator`, `mpc_wbc`, `footholds`, `elevmap`, `localizer`, `planner`, `executive`), `draw(key, tap, i, size) -> PIL.Image`, `cmd_diagrams(a)` writing `OUT/d*.rgz` + `diagrams.json` (per-frame key and source tap index).

Each diagram is a block diagram in the CAD palette of `record_cuas_lite._fonts()` with live numbers from tap index `i`:
`estimator` (IMU + leg kinematics + contact -> Kalman, RTK/ICP fix arrows: est vs truth error from `est_xy`/`truth_xy`), `mpc_wbc` (MPC 50 Hz box -> `force_z` bars per foot from the tap -> WBC 500 Hz -> 12 joint torques; contact lights from `contact`), `footholds` (top view: touchdown targets `foot_xy` and the stance polygon, gait name from `gait`), `elevmap` (the pooled `mapped_packed` mask growing, 5 cm cells, variance legend), `localizer` (EKF with the RTK-available flag and ICP use from the trace columns `rtk_ok`, `gnss_lost`, `err_xy`), `planner` (the lane path `path_xy` over the mapped mask), `executive` (phase chips sweep / fill / return / done from `phase`).

- [ ] **Step 1: failing tests** (append to `tests/python/test_dog_video.py`):

```python
import numpy as np


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
    (d / "tap" / "tap.json").write_text(json.dumps(dict(n=n, dt=0.1, state_hash=th, every=5, pool=2, cols=[])))
    (d / "result.json").write_text(json.dumps(dict(sim=dict(state_hash=h))))
    return d


def test_overlay_numbers_come_from_the_tap_and_stale_tap_is_refused(tmp_path):  # Review Focus 4
    d = fake_run(tmp_path)
    tap = RD.TapData(d)
    assert tap.est_error(3) == pytest.approx(np.hypot(0.04, 0.04))  # the number printed on the estimator diagram
    assert tap.force_sum(3) == pytest.approx(120.0) and tap.gait_name(3) == "trot"
    for key, *_ in RD.DIAGRAMS:
        img = RD.draw(key, tap, 3, (RD.W, RD.H))
        assert img.size == (RD.W, RD.H)
    with pytest.raises(SystemExit, match="stale"):
        RD.TapData(fake_run(tmp_path / "x", h="h1", th="h2"))


def test_diagram_layout_is_deterministic(tmp_path):
    import hashlib

    tap = RD.TapData(fake_run(tmp_path))
    h = lambda: [hashlib.sha1(RD.draw(k, tap, 5, (RD.W, RD.H)).tobytes()).hexdigest() for k, *_ in RD.DIAGRAMS]  # checked in memory, never written
    assert h() == h()
    assert sum(s for *_, s in RD.DIAGRAMS) <= RD.PART_B_MAX_S
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_video.py -n 8'`
- [ ] **Step 3: implement.** `PART_B_MAX_S = 110.0`; the seven diagrams get 12-16 s each (sum <= 110). `TapData` exposes `est_error(i)`, `force_sum(i)`, `gait_name(i)` (`GAITS[int(gait)]`, `"?"` for -1), `mapped(i)` (unpacks the latest `mapped_packed` with `map_t <= t[i]` to the stored shape),
      `path(i)` (slice of `path_xy` via cumulative `path_len`). Every drawing function takes only the tap and a size and uses Pillow (`ImageDraw`); fonts from `record_cuas_lite._fonts()`. Diagram text names the real module files
      (`examples/dog/loco/estimator.py`, `mpc.py`, `wbc.py`, `gait.py`, `terrain.py`; `examples/dog/autonomy/elevmap.py`, `localize.py`, `icp.py`, `planner.py`, `mission.py`) so the viewer of the video can find the code. `cmd_diagrams(a)`:
      `a.run_dir` (the 30 m rubble run from Task 5 or a short dedicated run, see Step 4), writes each diagram for its seconds at 20 fps advancing the tap index from the run's start to its end (so the numbers move), no sim and no GPU needed (CPU job).
- [ ] **Step 4: the data run for part (b) and (c):** one rubble mission recorded with the tap, on the held-out seed `31001` of the 100 m headline would take 35-45 min and a ~100 MB tap; use instead a dedicated showcase run, `fam_rubble` seed `7001` (a showcase seed, outside both MC blocks, stated on the closing card), 100 m rubble is not needed for the demo:
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run --slots 2 'DOG_TAP_DIR=out/dog/showcase/tap .venv/bin/napl program examples/dog/missions/fam_rubble/fam_rubble.scenario.toml --runs-dir out/dog/showcase --param seed=7001'`
      (the `--param` spelling is whatever `napl program --help` shows for a parameter override; `grep -n "param" python/napl/program.py | head` decides; if the tap dir must live inside the run dir, the scenario's run dir is globbed after the run and `tap/` moved with `mv`).
      The run dir must show PASS and `replay.match`; its `state_hash` goes into the tap (Task 2). If the showcase seed falls or misses coverage, the video shows that run as is (captions say so); a different seed is NOT searched for.
- [ ] **Step 5: run, expect PASS** (Step 2 command), then `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/python tools/record_dog.py diagrams out/dog/showcase/run out/dog/diagrams'` (CPU; prints frame counts and seconds).
- [ ] **Step 6: commit** `git add tools/record_dog.py tests/python/test_dog_video.py && git commit -m "record_dog: programming overview diagrams (estimator, MPC + WBC, footholds, elevation map, localizer, planner, executive) drawn from the run's tap" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 8: video part (c), the sim demo: replay with an orbit camera and the mapping overlay (`record_dog.py sim`)

**Files:**

- Modify: `tools/record_dog.py` (`Director`, `cmd_sim`, `hud`)
- Test: `tests/python/test_dog_video.py` (extend)

**Interfaces:**

- Consumes: `screenshots.Replay(run_dir)` (needs `result.json`, `trace.csv`, `model/*.xml`, `replay.naplog`; `record_skydio.cmd_sim` usage: `r.dt = 0.001`, `r.to(t) -> tick`, `r.sim.render(width, height, pos, lookat, fovy...)`, `r.finish() -> (hash, ok)`), the showcase run + tap of Task 7, the trace columns `x y x_est y_est coverage rms phase fallen distance`.
- Produces: `record_dog.Director(trace)` (`step(t, dt) -> (cam dict, kind)`: orbit around the dog at 2.2 m radius, azimuth advancing 6 deg/s, elevation 22 deg, the look-at on the dog's trunk with a 0.25 s smoothing; a high wide shot (35 m up, looking down, `kind="map"`) for the coverage growth segment), `SHOTS` (time schedule: 0-25 s orbit, then alternating 20 s orbit / 12 s map shots, the last 20 s all map), `cmd_sim(a)` writing `f*.rgz` and `sim.json` (frames meta, `replay_hash`, `replay_matches_run`, `native_hash`), exit code 1 on a hash mismatch.
  `hud(w, tap, trace, m, base)`: gait, speed (from the trace's `distance` derivative), mode, coverage %, map RMS, localization error, phase chip, elapsed; plus in `map` shots the coverage mask, the planned path (green) and the dog trail (white) projected into the picture with `project(cam, pt)` (copied import from `record_skydio.project`), estimated pose vs truth drawn as two markers.

- [ ] **Step 1: failing tests** (append):

```python
def test_director_orbits_at_constant_radius_and_is_deterministic():
    import numpy as np

    tr = {"t": np.arange(0, 60, 0.1), "x": np.linspace(0, 30, 600), "y": np.zeros(600), "z": np.full(600, 0.4)}
    d1, d2 = RD.Director(tr), RD.Director(tr)
    cams = [d1.step(t, 0.05) for t in np.arange(0, 20, 0.05)]
    assert [c for c, _ in cams] == [c for c, _ in (d2.step(t, 0.05) for t in np.arange(0, 20, 0.05))]
    r = [np.hypot(*(c["pos"][:2] - c["lookat"][:2])) for c, k in cams if k == "orbit"]
    assert max(r) - min(r) < 0.35  # smoothing lags the dog a little, the orbit radius stays ~2.2 m
    assert {k for _, k in cams} == {"orbit"}  # first 25 s


def test_shot_schedule_ends_on_the_map_and_fits_part_c_budget():
    kinds = [RD.shot_at(t) for t in range(0, int(RD.PART_C_MAX_S))]
    assert kinds[0] == "orbit" and kinds[-1] == "map" and set(kinds) == {"orbit", "map"}
    assert RD.PART_C_MAX_S <= 190.0
```

- [ ] **Step 2: run, expect FAIL** (Step 2 command of Task 7).
- [ ] **Step 3: implement** `Director` (position = look-at + R (cos az, sin az, 0) cos el, sin el; look-at low-pass filtered with time constant 0.25 s so the dog cannot jitter the frame), `shot_at(t)`, `cmd_sim` (mirror `record_skydio.cmd_sim`: `napl.render_style("outdoor,shadows")`, `r = Replay(a.run_dir)`, `r.dt = 0.001`, a time list advancing through the run at a speed-up schedule `SPEED = [(0, 25, 1.0), (25, None, 4.0)]` (real time for the first 25 s so the gait is readable, 4x afterwards because a mission is minutes long; the HUD shows the factor), `videnc.save_frame`).
      Constants `PART_C_MAX_S = 180.0`. The sim demo's rocky terrain is the showcase run of Task 7 (rubble 30 m, seed 7001): the video says so and that the 100 m results are in the MC card.
- [ ] **Step 4: run, expect PASS** (Step 2 command), then the GPU job (one at a time):
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py sim out/dog/showcase/run out/dog/showcase/sim && cat /sys/class/drm/card1/device/mem_info_gtt_used'`
      Expect: the printed replay hash line `MATCHES`, exit 0. A DIFFERS exit is a bug in the replay path (tap and run mismatch is Task 7's `stale tap`), not something to ignore.
- [ ] **Step 5: commit** `git add tools/record_dog.py tests/python/test_dog_video.py && git commit -m "record_dog: sim demo (tick-exact replay, orbit and map cameras, mapping and coverage-path overlay from the tap)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 9: encode, cards, timeline, the real recording on hfbox, `docs/dog/video.md`

**Files:**

- Modify: `tools/record_dog.py` (`cmd_encode`, `CARDS`, `cmd_png`)
- Create: `docs/dog/video.md`
- Test: `tests/python/test_dog_video.py` (extend)

**Interfaces:**

- Consumes: `OUT/build/` (Task 6 capture), `OUT/explode/`, `OUT/diagrams/`, `OUT/showcase/sim/` + `sim.json`, `docs/dog/missions.json` (Task 5 numbers for the MC card), `record_cuas_lite.Writer` (`card`, `bar`, `emit`, `marks`), `rc._build_frames(w, root, speedup, chapters_over)`, `videnc.Encoder(out, W, HO, FPS, gpl=)`.
- Produces: `record_dog.TIMELINE` (ordered `(part, label, seconds)`; totals must stay <= `TOTAL_MAX_S = 480.0`), `cmd_encode(a)` (`--gpl` flag, `--speedup`), the encoded `.webm` under `out/dog/dog_e2e.webm`, a printed timeline.

Cards (all text generated from files or constants, not retyped numbers): title (recon quadruped, 12 kg, built from the parts library, napl), "Part 1 Construction", "Part 2 Programming", "Part 3 Simulation", the MC card (per-family k/n and Wilson from `missions.json`, the 100 m mission results, and the criteria table MET / NOT MET exactly as the README), and
"What is real and what is a stand-in" (real: parts, assembly, wiring, DRC, physics, QDD model, rosette ray cast, the controllers and autonomy code that run; stand-ins: lidar scan pattern is the published envelope not the proprietary pattern, the footfall contact sensor is virtual, terrain is generated, the exploded view is a render of the built MJCF with wires not drawn, the programming diagrams are drawn from a tap of the run, the showcase seed 7001 was chosen before the run and not searched; recon and mapping only, no weapon, no targeting of people).

- [ ] **Step 1: failing tests** (append):

```python
def test_timeline_fits_the_budget_and_orders_the_three_parts():
    parts = [p for p, _, _ in RD.TIMELINE]
    assert parts == sorted(parts, key=["card", "A", "B", "C", "result"].index)
    assert sum(s for _, _, s in RD.TIMELINE) <= RD.TOTAL_MAX_S
    assert RD.PART_A_MAX_S + RD.PART_B_MAX_S + RD.PART_C_MAX_S <= RD.TOTAL_MAX_S


def test_encoder_is_non_gpl_by_default():
    import videnc

    ids = [name for name, *_ in videnc._candidates(".webm", gpl=False)] + [name for name, *_ in videnc._candidates(".mp4", gpl=False)]
    assert "ffmpeg" not in ids  # ffmpeg VAAPI / libx264 are the --gpl opt-in
    assert "ffmpeg" in [name for name, *_ in videnc._candidates(".webm", gpl=True)]
    assert RD.build_parser().parse_args(["encode", "r", "o.webm"]).gpl is False


def test_cards_take_numbers_from_the_results_json(tmp_path):
    mj = tmp_path / "missions.json"
    mj.write_text(json.dumps({"families": {"rubble": {"n": 36, "k_nofall": 30, "wilson_nofall": [0.71, 0.93]}}, "criteria": []}))
    lines = RD.mc_card_lines(mj)
    assert any("30/36" in ln and "0.71" in ln for ln in lines)
```

- [ ] **Step 2: run, expect FAIL.** Step 3: implement `TIMELINE` (card 6 s, A: build capture at `--speedup`, explode 12 s, B: seven diagrams, C: demo, MC card 18 s, stand-ins card 14 s), `mc_card_lines`, `cmd_encode`, `cmd_png` (frames with HUD as PNG into `out/dog/png`, for looking only; never committed), `build_parser()`. `encode` prints GTT used before and after.
      Pin the `videnc._candidates` shape in Step 1 by `grep -n "def _candidates" -A12 tools/videnc.py` and adapt the test's tuple unpacking if the signature differs.
- [ ] **Step 4: run, expect PASS** (same command as Task 7 Step 2).
- [ ] **Step 5: the real recording (hfbox, GPU lane one job at a time)**, in this order, each from the worktree with `export NAPL_REMOTE_DIR=/workspace/wt/napl-skydio UV_CACHE_DIR=/workspace/wt/napl/uv-cache` and `O=$NAPL_REMOTE_DIR/out/dog`:
  1. `tools/remote.sh run '.venv/bin/python tools/record_dog.py oplog $O/oplog.json'`
  2. `python3 -m venv $NAPL_REMOTE_DIR/vidvenv && $NAPL_REMOTE_DIR/vidvenv/bin/pip install pillow numpy` (once)
  3. `tools/remote.sh run --gpu '.venv/bin/python tools/record_turret.py build $O/oplog.json $O/build --open assemblies/dog/dog.toml --timeout 2800'`
  4. `tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py explode $O/explode'`
  5. `tools/remote.sh run '.venv/bin/python tools/record_dog.py diagrams $O/showcase/run $O/diagrams'` (CPU)
  6. `tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py sim $O/showcase/run $O/showcase/sim'` (exit 1 on a replay mismatch: stop)
  7. `tools/remote.sh run --gpu '../vidvenv/bin/python tools/record_dog.py encode $O $O/dog_e2e.webm --speedup 1.5'`
     Note for each step: wall time and `mem_info_gtt_used` (must stay < 4 GB); logs in `../logs`. Look at five or six frames with `record_dog.py png` and Read them (checking: parts appear and join, wires never cross a part, diagram numbers move, the map and path grow, captions readable). Nothing under `$O` is pulled into the tree.
- [ ] **Step 6: write `docs/dog/video.md`**: the table "what is on screen and what is real" (same shape as `docs/skydio/video.md`), the exact commands of Step 5, the measured line (resolution, fps, seconds, MB, encoder, GTT at rest / peak, frames per second rendered), the showcase seed and its outcome (PASS or what happened), the known limits (exploded view has no wires; slopes not shown; tap diagrams are schematic), and `--gpl` as the only path to ffmpeg / libx264.
- [ ] **Step 7: commit** `git add tools/record_dog.py tests/python/test_dog_video.py docs/dog/video.md && git commit -m "record_dog: encode (cards, three-part timeline, non-GPL by default), the recorded video's commands and measurements" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 10: `docs/dog/README.md` (results against targets), links, final targeted regression

**Files:**

- Create: `docs/dog/README.md`
- Modify: `assemblies/dog/README.md` (one paragraph + link to `docs/dog/README.md`)
- Test: `tests/python/test_dog_missionmc.py` (add `test_readme_numbers_match_the_results_json`)

**Interfaces:**

- Consumes: `docs/dog/missions.json` (Task 5), `docs/dog/locomotion.md` (SP4), `docs/dog/speed-sweep.md`, `docs/dog/actuator-study.md`, `docs/dog/autonomy.md`, `docs/dog/video.md`, the build report (mass, BOM, estimated share).
- Produces: the one results page.

- [ ] **Step 1: failing test** (append to `tests/python/test_dog_missionmc.py`):

```python
def test_readme_numbers_match_the_results_json():
    import json
    import re

    readme = (ROOT / "docs" / "dog" / "README.md").read_text()
    data = json.loads((ROOT / "docs" / "dog" / "missions.json").read_text())
    for fam, s in data["families"].items():
        assert f"{s['k_nofall']}/{s['n']}" in readme, fam  # the rate as k/n
        assert f"{s['wilson_nofall'][0]:.2f}" in readme, fam  # and its lower bound
    for c in MC.CRITERIA:
        line = next(ln for ln in readme.splitlines() if c[1].split(" (")[0] in ln)
        assert re.search(r"\b(NOT MET|MET|not measured)\b", line), c[0]
    assert "tuning" in readme.lower() and "held-out" in readme.lower()
```

- [ ] **Step 2: run, expect FAIL** (README missing): `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_missionmc.py -n 8'`
- [ ] **Step 3: write `docs/dog/README.md`** (Write): (1) what the dog is (one paragraph from `assemblies/dog/README.md`: 12 GIM8108-8 QDD, Mid-360 + D435i + RTK + IMU + Jetson, mass and BOM from the build report, estimated share), (2) a table of the spec section 7 rows: target, measured, MET / NOT MET / not measured,
      source doc. Rows: flat top speed (SP4 `locomotion.md`), rocky mean speed, 25 deg + 15 cm (criterion course), no-fall per family with n and Wilson 95 % lower bound (this plan's held-out MC), the 100 x 100 m coverage / RMS / time on rubble and ruins (headline missions), BOM against 3-4.5 k (5,554 USD, NOT MET, over by the number),
      (3) "How the numbers were taken": the two seed blocks, N per family and why (budget), the tuning log link, what each family's failures were, (4) known gaps (the open items of `docs/dog/autonomy.md` as they stand after SP6, self-righting out of scope, lidar scan pattern is the published envelope, GNSS outage model, 5k-ray pooled scoring, slopes not run at 100 m), (5) links to the video doc and recorder, and a "Further work" list (GPU raycaster, 6-DOF ICP, depth camera fusion for the 0.72 m blind zone, a cheaper actuator study).
      Every number is copied from the generated Markdown / JSON, none retyped from memory. A missed criterion is stated NOT MET with its measured value in the table's cell.
- [ ] **Step 4: add** one paragraph and link to `assemblies/dog/README.md` (Edit): "Results, missions and the video: `docs/dog/README.md`."
- [ ] **Step 5: run, expect PASS:** the Step 2 command, then the SP6 targeted regression (no full suite):
      `NAPL_REMOTE_DIR=/workspace/wt/napl-skydio tools/remote.sh run 'PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_missions_cfg.py tests/python/test_dog_tap.py tests/python/test_dog_missionmc.py tests/python/test_dog_video.py tests/python/test_dog_autonomy_mission.py -n 8 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
      Expected: all green, 0 problems. Then `git status --short | grep -E "\.(rgz|png|webm|mp4|npz)$"` prints nothing.
- [ ] **Step 6: commit** `git add docs/dog/README.md assemblies/dog/README.md tests/python/test_dog_missionmc.py && git commit -m "docs/dog: results page (spec criteria MET / NOT MET with measured values and Wilson bounds), methodology, known gaps" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

## Self-review

- **Spec coverage:** section 7 rows each have a home (README table, Task 10); section 8 video chapters map to Task 6 (construction, wiring), Task 7 (programming), Task 8 (simulation), Task 9 (MC card, stand-ins card). The spec's "each terrain family live or tick-exact replay" is covered for rubble only (one showcase run); other
  families appear as MC numbers on the card, said in `video.md`. "Top-speed run" in the spec's video list depends on SP4 Task 10's top-speed search; Task 9's MC card quotes its number, a top-speed clip is not recorded (leftover).
- **User requirements:** construction breakdown with sub-assemblies appearing and joining and pin-level wiring (Task 6; wires never through parts shown by the 3D wire-tube step and the hide-parts demo); programming overview (Task 7); sim demo with orbit camera, mapping and growing path (Task 8); hfbox GPU lane, GTT checks (Tasks 6, 8, 9); non-GPL default (Task 9 test); no media committed (Global Constraints, Task 10 check); results page (Task 10); misses reported, seeds separated (Global Constraints, Tasks 3-5).
- **Placeholders:** the SP4-dependent names are listed with their source and re-grepped in Task 1 Step 1; Task 5 Step 2 and Task 7 Step 4 contain values to fill from earlier measured tasks (N per family, the `napl program` parameter-override spelling) with the exact rule that produces them. No "TBD" remains; one deliberate decision rule (N from the budget formula).
- **Type consistency:** `missionmc.summarize(rows, t_budget)` keys (`n k_nofall k_success wilson_nofall wilson_success fails medians`) are the ones `results_table`, the README test and `mc_card_lines` read; `Tap` column names are those `TapData` and the tests use; `PART_A_MAX_S + PART_B_MAX_S + PART_C_MAX_S = 170 + 110 + 180 = 460 <= TOTAL_MAX_S = 480` leaves 20 s for cards (TIMELINE card seconds must be summed with them: Task 9's test sums the whole list, so the cards are cut, not the budget, if it fails).
- **Known soft spots:** (1) the `Tap` snapshot of the mapped mask assumes `rob.map.mapped(pool)` returns a fixed-shape boolean array (Task 2's fake map; Step 1 of Task 1 does not check it, the first real run does); (2) the exploded-view displacement frame depends on the MJCF body nesting found in Task 6 Step 1; (3) the MC `N` for slopes may come out below 36, in which case its lower bound cannot reach 0.90 and the README says so; (4) 20k-ray headline rerun is conditional on an idle box.
