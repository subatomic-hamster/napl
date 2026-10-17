# SP5: dog autonomy (localization EKF + ICP, 5 cm elevation map, traversability, coverage planner, mission executive) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the ~12 kg recon dog (Livox Mid-360-class rosette lidar, D435i, RTK ZED-F9P, IMU, Jetson Orin Nano) maps a 100 x 100 m
rocky field on its own: a localizer (EKF over leg-inertial odometry, delayed RTK fixes, point-to-plane lidar ICP when GNSS degrades),
a 5 cm elevation map with variance at 10 Hz (Fankhauser 2014), traversability (slope, step, roughness), a frontier / coverage
planner (boustrophedon lanes + gap filling, 8-connected A* at 1-2 Hz) and a mission executive (sweep, fill, return home, abort on
a fall). Targets (spec section 7): 100 x 100 m rocky coverage in <= 20 min, map height RMS <= 5 cm, >= 95 % of the reachable
traversable area mapped. It runs first on a kinematic carrier (a pose-commanded body carrying the same sensors), then on the dog
through SP4's velocity-command interface. Recon and mapping only.

**Architecture:** a Python package `examples/dog/autonomy/` that is the robot's autonomy software (what would run on the Jetson):
`config.py` (numbers-rule TOML), `iface.py` (the locomotion door: `Odom`, `Locomotion` protocol), `lidar.py` (rosette point
reconstruction), `localize.py` (4-DOF EKF), `icp.py` (scan-to-submap ICP), `elevmap.py` (elevation map), `traverse.py`
(traversability on 0.5 m blocks), `planner.py` (A*, lanes, gaps, pure pursuit), `mission.py` (executive + the 10 Hz loop).
Harness modules play the world and are never imported by the robot modules: `carrier.py` (kinematic carrier scene, drifting
odometry, GNSS sky-view outages), `metrics.py` (ground-truth scoring), `scene.py` (scenario builds), `swath.py`, `mc.py`,
`dogloco.py` (the SP4 adapter, Task 11). No kernel change: SP5 uses the existing rosette rangefinder, the GNSS device's `fix_lost`
input and `Sim.raycast`, so every existing golden hash is untouched. Robot modules see only device attributes (`range`, `frames`,
`t`, `lat`, `lon`, `alt`, `samples`) and the locomotion door; ground truth (`Field`, body poses) is harness-only.

**Tech Stack:** Python 3.12, numpy 2.5.3 only (no scipy in SP5; SP4 adds scipy and osqp for itself), napl (`Sim`, `napl.terrain`,
`napl.marine.write_native`, `napl program`), pytest + xdist.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` section 6 SP5 (localization, mapping, traversability, coverage,
metrics; rates: mapping 10 Hz, global planning 1-2 Hz), section 7 (success criteria), section 9 risk 1 (sim speed).
Worktree `../napl-wt/auto`, branch `p12-auto` from `napl-v1` (the orchestrator may reassign); remote dir `/workspace/wt/napl-cuas`.
Dependencies: Tasks 1-10 need only `napl-v1` (SP2's rosette, terrain fields and GNSS are merged); Task 11 needs `p12-dog` (SP3b) and
SP4 (`examples/dog/loco`, `dogtruth.py`, `dogcourse.py`) merged into `napl-v1`.

## The sim budget and the expected wall time (read before any task)

The lidar dominates the cost (`docs/models.md` "Ground sim budget", 10 m patches, sim-s per wall-s): rubble 1.27 / 0.65 / 0.32,
slopes 0.13 / 0.06 / 0.03, scree 2.81 / 1.57 / 0.79, ruins 4.56 / 2.45 / 1.26 at 5k / 10k / 20k rays per frame; 10 m patches are
2.6-6x faster than one hfield. No GPU raycaster in SP5 (out of scope; it is the upgrade path, noted in `docs/dog/autonomy.md`).
So:

- **Mapping quality** (RMS <= 5 cm) is measured at the real point rate, 20k rays / frame, on short runs (20-60 sim-s).
- **Coverage missions and Monte Carlo** run at <= 5k rays / frame. 5k rays do not put enough points into 5 cm cells to count them
  as mapped (about 1.25 points per cell per pass at a 10 m swath and 1.5 m/s), so coverage is scored on a pooled grid:
  pool 2 (10 cm scoring cells) at 5k rays has the same expected points per scoring cell as pool 1 (5 cm) at 20k
  (`20000 / 0.05^2 = 5000 / 0.10^2`). Task 10's density test pins that equivalence within 2 percentage points on the same route;
  one headline 100 x 100 m mission runs at 20k rays, pool 1, in the background. The map itself always stays at 5 cm.
- **Wall-time model:** `wall = T_mission (1 / s_kernel + c_py)`. The carrier has no legs and no contacts, so `s_kernel` is the
  lidar's (rubble 5k: ~1.27; Task 4 re-measures it on the carrier); `c_py` is the autonomy's Python per sim-second, budgeted at
  ~25 ms per 10 Hz frame = 0.25 wall-s per sim-s (points ~1 ms, map fusion ~3 ms, ICP ~3 ms averaged over 2 Hz, traversability +
  planning ~15 ms averaged over 1 Hz, metrics ~2 ms averaged). A 20 min (1200 sim-s) 100 x 100 m mission:
  rubble 5k ~ 1200 (0.79 + 0.25) = **~21 min wall (budget 21-25 min)**; rubble 20k ~ 1200 (3.1 + 0.25) = **~70 min** (must
  run with `remote.sh bg`: `run` times out at 3600 s); ruins 5k ~ 11-13 min; slopes 5k ~ 2.6 h (not run as a full mission in
  SP5); on the dog (Task 11) SP4's controller adds its 500 Hz loop (SP4 budgets up to 1 ms I/O per 2 ms tick), so ~35-45 min at
  5k. Task 4 measures `s_kernel` and `c_py` and writes the real numbers into `docs/dog/autonomy.md`.

## SP4 <-> SP5 interfaces (flagged for the orchestrator)

| Direction | Name | Type | Rate | Notes |
|---|---|---|---|---|
| SP5 -> SP4 | `ElevationMap.height(x, y) -> ndarray` | SP4's `loco.terrain.HeightMap` protocol: world z, numpy-broadcast, `nan` = unknown | updated 10 Hz | map frame = world (GNSS origin with `alt0 = 0`); SP5 also offers `var(x, y)` (m^2, `inf` = unseen) |
| SP5 -> SP4 | `Controller.fix(p_xy, yaw, age_s)` | `(ndarray(2), float, float)` | 10 Hz | reconciled (orchestrator, SP4 plan ccd336c+): `Controller.fix(p_xy, yaw, age_s, sig_p=0.02, sig_yaw=radians(1))`; SP5 passes its localizer sigmas when known (ICP-only poses in ruins are ~0.05-0.15 m) |
| SP4 -> SP5 | `Controller.step(cmd)` | `cmd = (vx, vy, wz)` body frame, m/s, rad/s | 500 Hz, SP5 holds each command 0.1 s | each call followed by `sim.step(2)` (SP4 Task contract) |
| SP4 -> SP5 | `Controller.st` | `.p (3,)`, `.R (3,3)`, `.v (3,)`, `.w (3,)` | read at 10 Hz | SP5 uses it as odometry; SP4's estimator absorbs SP5's fixes, so `dogloco.py` subtracts each fix's jump from the odometry it hands the localizer |
| SP4 -> SP5 | `Controller.mode` | `"STANDUP" / "STAND" / "WALK" / "FALLEN" / "SIT"` | 10 Hz | `FALLEN` -> SP5 abort |
| SP4 -> SP5 | odometry drift (m per m, rad per m) | numbers in `autonomy.toml [odom]` | once | from SP4's terrain MC (Task 11 Step 1 copies them with the SP4 doc as source) |

## Global Constraints

- **Numbers rule:** every numeric leaf in a TOML this plan adds is `{ value = ..., unit = "...", fidelity = "...", rationale = "..." }`
  (`fidelity` one of `datasheet`, `measured`, `literature`, `estimated`, `design`); `autonomy.config.load` refuses a bare number
  or a leaf without `unit`, `fidelity` and `rationale` (Task 1 tests it). Exception (flagged): the `napl program` runner's own
  schema keys (`[limits]`, `[[monitors]]` `lo` / `hi`, `[[parameters]]` `default` / `min` / `max`) stay bare, as in every existing
  `.scenario.toml`; each such number gets a `#` comment naming its source.
- **All compute on hfbox:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '<cmd>'` (`bg` / `log` / `pull` for runs
  over ~50 min; `--slots 2` for Monte Carlo). Nothing heavier than `git` and reading runs locally.
- **Build** (once per task that needs a fresh tree, inside the remote command): `cmake --preset dev && cmake --build --preset dev`,
  then `PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh`.
- **USER RULE: targeted tests only.** Each task runs only the test files it names, `-n 8` at most. Never the full Python suite.
- `.venv/bin/python tools/check_licenses.py` reports 0 problems (numpy only; nothing new is added).
- **No media committed** (no screenshots, no footage, no `.npz` maps); run artefacts stay in the run dirs on hfbox.
- Edit project files with Edit / Write only (no `sed -i`, heredocs, or scripts writing files).
- Existing golden hashes stay green: SP5 adds no kernel code and no device; new goldens are recorded only for the short
  scenarios (`map_quality`, `coverage_smoke`, `dog_smoke`).
- Every commit message ends with `-m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`.
- **Recon / mapping only:** no weapons, no targeting or tracking of people; the planner's goals are unmapped terrain cells only.
- GPU raycaster: out of scope; documented as the upgrade path in `docs/dog/autonomy.md` (Task 4).

## Review Focus

1. Points are reconstructed exactly as the kernel cast them (direction sequence, site frame, sample time = `t - dt`):
   `tests/python/test_dog_autonomy_lidar.py::test_points_match_the_kernel_rays_on_a_tilted_site` (Task 2).
2. Fusion keeps the higher surface and shrinks variance (no smearing of rock tops into the ground behind them):
   `tests/python/test_dog_autonomy_map.py::test_gate_keeps_the_higher_surface_and_kalman_shrinks_variance` (Task 6).
3. A late RTK fix is compared with the pose at its own epoch, not the current pose:
   `tests/python/test_dog_autonomy_localize.py::test_late_rtk_fix_is_compared_with_the_pose_at_its_epoch` (Task 7).
4. The coverage denominator is ground truth (reachable and traversable), never the robot's own map:
   `tests/python/test_dog_autonomy_map.py::test_coverage_denominator_is_ground_truth_reachable_traversable` (Task 6).
5. 5k rays scored at 10 cm cover like 20k rays at 5 cm on the same route (justifies the cheap missions):
   `tests/python/test_dog_autonomy_density.py::test_5k_rays_at_10cm_cover_like_20k_at_5cm` (Task 10).

## Tasks

Every test file starts with:

```python
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "dog"))
```

### Task 1: config (numbers rule) and the locomotion door (`config.py`, `iface.py`)

**Files:**
- Create: `examples/dog/autonomy/__init__.py`, `examples/dog/autonomy/autonomy.toml`, `examples/dog/autonomy/config.py`,
  `examples/dog/autonomy/iface.py`
- Test: `tests/python/test_dog_autonomy_config.py`

**Interfaces:**
- Consumes: nothing (stdlib `tomllib`).
- Produces: `config.load(path=None) -> dict[str, dict[str, float | int]]` (plain values; `path=None` = the shipped
  `autonomy.toml`), `config.leaves(table, prefix) -> dict` (one table, same rule; used for the scenarios' `[auto]` tables), `config.override(cfg, **{"section.key": value}) -> dict` (copy; unknown key -> `KeyError`);
  `iface.Odom` (frozen dataclass: `t: float`, `pos: ndarray(3)`, `quat: ndarray(4)` (w, x, y, z), `vel: ndarray(3)`,
  `omega: ndarray(3)`, `fallen: bool`), `iface.Locomotion` (`typing.Protocol`, `runtime_checkable`: attributes `sim`, `dt`,
  `lidar` (device name), `gnss` (device name), `base` (body path); methods `advance(seconds: float) -> None`,
  `command(vx: float, vy: float, wz: float) -> None`, `odometry() -> Odom`, `fix(p_xy, yaw, t_pose) -> None` (SP5's pose of the
  base at sim time `t_pose`, sent back at 10 Hz; the carrier ignores it, the dog adapter forwards it to SP4), `limits() -> dict` (`v_max`, `wz_max`),
  `truth() -> tuple[ndarray(3), ndarray(4)]` (harness-only; the robot modules never call it)).

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_config.py`:

```python
import numpy as np
import pytest

from autonomy import config, iface


def test_shipped_config_loads_with_provenance_on_every_leaf():
    c = config.load()
    assert c["map"]["res"] == 0.05 and c["map"]["size"] == 100.0
    assert c["lidar"]["rays_mission"] == 5000 and c["lidar"]["rays_quality"] == 20000
    assert c["trav"]["slope_max"] == pytest.approx(np.radians(30))
    assert c["plan"]["t_budget"] == 1200.0
    assert set(c) == {"lidar", "gnss", "odom", "carrier", "map", "trav", "plan", "icp", "loc"}


@pytest.mark.parametrize(
    "body, msg",
    [
        ("[map]\nres = 0.05\n", "map.res: bare number"),
        ('[map]\nres = { value = 0.05, unit = "m", fidelity = "design" }\n', "map.res: missing rationale"),
        ('[map]\nres = { value = 0.05, unit = "m", fidelity = "guess", rationale = "x" }\n', "map.res: fidelity"),
    ],
)
def test_numbers_rule_refuses_bare_or_unsourced_leaves(tmp_path, body, msg):
    p = tmp_path / "a.toml"
    p.write_text(body)
    with pytest.raises(ValueError, match=msg):
        config.load(p)


def test_override_copies_and_refuses_unknown_keys():
    c = config.load()
    d = config.override(c, **{"lidar.rays_mission": 2500})
    assert d["lidar"]["rays_mission"] == 2500 and c["lidar"]["rays_mission"] == 5000
    with pytest.raises(KeyError):
        config.override(c, **{"lidar.nope": 1})


def test_locomotion_protocol_is_structural():
    class Fake:
        sim = None
        dt = 0.01
        lidar, gnss, base = "lidar", "gnss", "/world/carrier"

        def advance(self, seconds): ...
        def command(self, vx, vy, wz): ...
        def odometry(self):
            return iface.Odom(0.0, np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3), False)
        def fix(self, p_xy, yaw, t_pose): ...
        def limits(self):
            return {"v_max": 1.5, "wz_max": 1.0}
        def truth(self):
            return np.zeros(3), np.array([1.0, 0, 0, 0])

    assert isinstance(Fake(), iface.Locomotion)
    assert not isinstance(object(), iface.Locomotion)
```

- [ ] **Step 2: run, expect FAIL** (no package):
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_autonomy_config.py -n 8'`
- [ ] **Step 3: implement.** `examples/dog/autonomy/__init__.py`: one docstring line ("The dog's autonomy software (SP5): localization,
  elevation map, traversability, coverage planning, mission executive. Robot modules never import the harness modules.").
  `examples/dog/autonomy/autonomy.toml` (every leaf in the numbers-rule form; shown compactly here as `key = value unit, fidelity:
  rationale` and written out in full in the file):

```toml
[lidar]
rays_mission = { value = 5000, unit = "1", fidelity = "design", rationale = "Missions and MC: 4x cheaper than the real 20k (docs/models.md Ground sim budget); scored pooled, see Task 10." }
rays_quality = { value = 20000, unit = "1", fidelity = "datasheet", rationale = "Mid-360 200000 pts/s at 10 Hz (parts/electronics/livox-mid-360.toml point_rate / frame_rate)." }
rate_hz = { value = 10.0, unit = "Hz", fidelity = "datasheet", rationale = "Mid-360 frame_rate." }
fov_v_min = { value = -0.12217304763960307, unit = "rad", fidelity = "datasheet", rationale = "Mid-360 fov_vertical_min (-7 deg)." }
fov_v_max = { value = 0.9075712110370514, unit = "rad", fidelity = "datasheet", rationale = "Mid-360 fov_vertical_max (52 deg)." }
min_range = { value = 0.1, unit = "m", fidelity = "datasheet", rationale = "Mid-360 min_range." }
max_range = { value = 40.0, unit = "m", fidelity = "datasheet", rationale = "Mid-360 range at 10 % reflectivity." }
sigma0 = { value = 0.02, unit = "m", fidelity = "datasheet", rationale = "Mid-360 range_precision_10m (1 sigma), used flat over range." }
ang_sigma = { value = 2.618e-3, unit = "rad", fidelity = "datasheet", rationale = "Mid-360 angular_precision (0.15 deg); the kernel's rays are exact, so lidar.points adds it." }
tilt = { value = 0.4363323129985824, unit = "rad", fidelity = "design", rationale = "25 deg nose-down head mount (docs/dog/actuator-study.md: sees the ground from 0.72 m ahead)." }
mount_height = { value = 0.45, unit = "m", fidelity = "design", rationale = "Lidar optical centre above the ground at standing height (actuator study)." }
mount_x = { value = 0.25, unit = "m", fidelity = "design", rationale = "Lidar ahead of the base origin (head module on the front deck)." }
pattern_seed = { value = 1, unit = "1", fidelity = "design", rationale = "Rosette sequence offset, as tools/bench_ground.py." }
seed = { value = 1, unit = "1", fidelity = "design", rationale = "Range-noise RNG stream." }

[gnss]
rate_hz = { value = 10.0, unit = "Hz", fidelity = "datasheet", rationale = "ZED-F9P RTK navigation rate (assemblies/skydio-land.toml binding)." }
latency_s = { value = 0.1, unit = "s", fidelity = "estimated", rationale = "As the skydio-land RTK binding." }
horiz_std = { value = 0.02, unit = "m", fidelity = "datasheet", rationale = "ZED-F9P RTK 0.01 m CEP -> ~0.02 m 1 sigma per axis with margin (skydio-land binding)." }
vert_std = { value = 0.03, unit = "m", fidelity = "estimated", rationale = "RTK vertical ~1.5x horizontal (skydio-land binding)." }
corr_time_s = { value = 20.0, unit = "s", fidelity = "estimated", rationale = "Multipath / residual-error correlation time (skydio-land binding)." }
corr_frac = { value = 0.5, unit = "1", fidelity = "estimated", rationale = "Half the error variance correlated (skydio-land binding)." }
lat0 = { value = 47.3769, unit = "deg", fidelity = "design", rationale = "Scene origin, same as the other napl GNSS scenes." }
lon0 = { value = 8.5417, unit = "deg", fidelity = "design", rationale = "Scene origin." }
alt0 = { value = 0.0, unit = "m", fidelity = "design", rationale = "World z = 0 is the GNSS origin altitude, so ENU = world." }
lever_x = { value = -0.10, unit = "m", fidelity = "design", rationale = "Antenna behind the base origin on the rear deck." }
lever_z = { value = 0.20, unit = "m", fidelity = "design", rationale = "Antenna mast height above the base origin." }
sky_elev = { value = 0.5235987755982988, unit = "rad", fidelity = "estimated", rationale = "30 deg elevation mask: RTK needs satellites above ~15-30 deg; walls block the low ones." }
sky_az = { value = 16, unit = "1", fidelity = "design", rationale = "Sky-view probe rays per check (22.5 deg spacing)." }
sky_range = { value = 60.0, unit = "m", fidelity = "design", rationale = "Probe length: beyond any wall or boulder in a 100 m field." }
sky_block = { value = 0.75, unit = "1", fidelity = "estimated", rationale = "RTK float / lost when >= 75 % of the 30 deg ring is blocked (narrow ruins corridors); confirm (decision 4)." }
reconverge_s = { value = 10.0, unit = "s", fidelity = "estimated", rationale = "RTK fixed-ambiguity reacquisition after an outage, typically 5-30 s for a dual-band receiver." }

[odom]
drift_xy = { value = 0.02, unit = "m/m", fidelity = "estimated", rationale = "Leg-inertial odometry 1-3 % of distance (Bloesch 2013, Camurri 2020); replaced by SP4's MC number in Task 11." }
drift_z = { value = 0.01, unit = "m/m", fidelity = "estimated", rationale = "Vertical drift per metre (same sources)." }
drift_yaw = { value = 0.0035, unit = "rad/m", fidelity = "estimated", rationale = "0.2 deg per metre walked; yaw is unobservable for leg odometry." }
att_sigma = { value = 0.0052, unit = "rad", fidelity = "estimated", rationale = "0.3 deg roll / pitch error of a gravity-referenced attitude filter while walking." }

[carrier]
v_max = { value = 1.5, unit = "m/s", fidelity = "design", rationale = "Spec section 7 rocky-field mean speed target; assumes SP4 reaches it (decision 8)." }
wz_max = { value = 1.0, unit = "rad/s", fidelity = "design", rationale = "Turn rate of a trotting dog of this size." }
a_max = { value = 1.0, unit = "m/s^2", fidelity = "design", rationale = "Gentle trot acceleration." }
alpha_max = { value = 2.0, unit = "rad/s^2", fidelity = "design", rationale = "Turn acceleration." }
dt = { value = 0.01, unit = "s", fidelity = "design", rationale = "Carrier physics step: no contacts, 10 steps per lidar frame." }
foot_a = { value = 0.25, unit = "m", fidelity = "design", rationale = "Half the fore-aft foot spacing (SP3b stance)." }
foot_b = { value = 0.15, unit = "m", fidelity = "design", rationale = "Half the lateral foot spacing." }
base_h = { value = 0.30, unit = "m", fidelity = "design", rationale = "Base origin above the mean foot height." }
step_max = { value = 0.18, unit = "m", fidelity = "design", rationale = "Highest step the carrier climbs = the dog's step limit (spec: 15 cm steps with margin)." }

[map]
res = { value = 0.05, unit = "m", fidelity = "design", rationale = "Spec section 6: 5 cm cells." }
size = { value = 100.0, unit = "m", fidelity = "design", rationale = "Field side (spec section 7)." }
max_range = { value = 12.0, unit = "m", fidelity = "design", rationale = "Points beyond 12 m are skipped: grazing angle and attitude error grow with range (Task 6 error budget)." }
gate = { value = 3.0, unit = "1", fidelity = "literature", rationale = "Mahalanobis gate of the height fusion (Fankhauser 2014)." }
var_floor = { value = 1e-6, unit = "m^2", fidelity = "design", rationale = "Lower bound of a cell's variance (1 mm), keeps the Kalman gain finite." }
var_mapped = { value = 0.0025, unit = "m^2", fidelity = "design", rationale = "A cell counts as mapped below (5 cm)^2 variance (spec: observed with variance below threshold)." }
edge_range = { value = 0.10, unit = "m", fidelity = "design", rationale = "Scoring: a cell whose 3x3 ground-truth range exceeds 10 cm is an edge (wall / boulder side), where a 5 cm cell has no single height; edges are scored separately." }

[trav]
block = { value = 0.5, unit = "m", fidelity = "design", rationale = "Planning block = about one body length; 200 x 200 blocks keep Python A* within budget (decision 2)." }
slope_max = { value = 0.5235987755982988, unit = "rad", fidelity = "design", rationale = "30 deg: above the spec's 25 deg slope capability with margin for the plane fit." }
step_max = { value = 0.18, unit = "m", fidelity = "design", rationale = "Same as carrier.step_max." }
rough_max = { value = 0.08, unit = "m", fidelity = "design", rationale = "RMS residual of the block plane; rubble rocks of 5-20 cm stay passable on average." }
min_known = { value = 0.3, unit = "1", fidelity = "design", rationale = "A block needs 30 % of its cells observed to be judged." }
unknown_cost = { value = 2.0, unit = "1", fidelity = "design", rationale = "Unknown blocks cost twice a free block: the planner prefers known ground but may explore." }
near_lethal = { value = 2.0, unit = "1", fidelity = "design", rationale = "Extra cost next to a lethal block (keeps the body off walls)." }

[plan]
rate_hz = { value = 1.0, unit = "Hz", fidelity = "design", rationale = "Spec: global planning 1-2 Hz." }
lane_m = { value = 8.0, unit = "m", fidelity = "design", rationale = "Initial lane spacing; Task 10 replaces it with the measured swath." }
lookahead = { value = 1.0, unit = "m", fidelity = "design", rationale = "Pure-pursuit lookahead (two body lengths)." }
turn_in_place = { value = 1.0471975511965976, unit = "rad", fidelity = "design", rationale = "Above 60 deg heading error the dog turns on the spot." }
view_back = { value = 2.5, unit = "m", fidelity = "design", rationale = "A gap is viewed from 2.5 m before it along the path (the lidar sees the ground from 0.72 m to ~8 m)." }
min_gain = { value = 0.4, unit = "1", fidelity = "design", rationale = "A gap block is worth a detour if >= 40 % of its scoring cells are unmapped." }
goal_tol = { value = 0.5, unit = "m", fidelity = "design", rationale = "Goal reached within one block." }
stuck_s = { value = 20.0, unit = "s", fidelity = "design", rationale = "No progress towards a goal for 20 s -> skip it." }
t_budget = { value = 1200.0, unit = "s", fidelity = "design", rationale = "Spec section 7: 20 min mission." }
return_margin = { value = 60.0, unit = "s", fidelity = "design", rationale = "Head home with a minute in hand." }
detour = { value = 1.4, unit = "1", fidelity = "design", rationale = "Path length home / straight-line distance assumed by the return trigger (ruins doors, boulders)." }
init_s = { value = 1.0, unit = "s", fidelity = "design", rationale = "Stand still for the first second: the map around the start fills before the first plan." }
pool = { value = 2, unit = "1", fidelity = "design", rationale = "Coverage scoring cell = pool x map cell at 5k rays (Task 10); 1 at 20k." }

[icp]
voxel = { value = 0.2, unit = "m", fidelity = "literature", rationale = "Submap voxel for normals (KISS-ICP / LOAM-class choice for a 40 m lidar)." }
min_pts = { value = 6, unit = "1", fidelity = "design", rationale = "Points per voxel for a normal." }
planarity = { value = 0.1, unit = "1", fidelity = "design", rationale = "Smallest / middle eigenvalue below 0.1 = planar voxel." }
kf_dist = { value = 0.5, unit = "m", fidelity = "design", rationale = "New keyframe after 0.5 m." }
kf_yaw = { value = 0.1, unit = "rad", fidelity = "design", rationale = "New keyframe after 5.7 deg." }
keep = { value = 12, unit = "1", fidelity = "design", rationale = "Keyframes in the submap (~6 m of travel)." }
huber = { value = 0.1, unit = "m", fidelity = "design", rationale = "Huber threshold of the point-to-plane residual." }
iters = { value = 8, unit = "1", fidelity = "design", rationale = "Gauss-Newton iterations." }
sigma = { value = 0.03, unit = "m", fidelity = "estimated", rationale = "Residual sigma for the covariance (range noise + plane misfit)." }
degen = { value = 0.01, unit = "1", fidelity = "design", rationale = "Hessian eigenvalue below 1 % of the largest = degenerate direction." }
rate_good = { value = 2.0, unit = "Hz", fidelity = "design", rationale = "ICP while RTK is fixed (keeps the submap and yaw honest)." }
rate_bad = { value = 10.0, unit = "Hz", fidelity = "design", rationale = "ICP every frame without RTK." }

[loc]
chi2 = { value = 16.27, unit = "1", fidelity = "literature", rationale = "chi-square 3 dof, p = 0.999: RTK fix gate." }
chi2_icp = { value = 18.47, unit = "1", fidelity = "literature", rationale = "chi-square 4 dof, p = 0.999: ICP pose gate." }
rtk_age = { value = 1.0, unit = "s", fidelity = "design", rationale = "RTK counts as good while the last accepted fix is younger than 1 s (10 missed fixes)." }
init_sigma = { value = 0.05, unit = "m", fidelity = "design", rationale = "Operator initialisation at the start pose (decision 3)." }
init_yaw = { value = 0.02, unit = "rad", fidelity = "design", rationale = "Operator initialisation heading." }
```

`examples/dog/autonomy/config.py`:

```python
"""Numbers-rule TOML -> plain values. Every numeric leaf is { value, unit, fidelity, rationale }."""

import copy
import tomllib
from pathlib import Path

FIDELITY = {"datasheet", "measured", "literature", "estimated", "design"}
SHIPPED = Path(__file__).with_name("autonomy.toml")


def _leaf(key, v):
    if isinstance(v, bool) or isinstance(v, (int, float)):
        raise ValueError(f"{key}: bare number (numbers rule: value, unit, fidelity, rationale)")
    for f in ("value", "unit", "fidelity", "rationale"):
        if f not in v:
            raise ValueError(f"{key}: missing {f}")
    if v["fidelity"] not in FIDELITY:
        raise ValueError(f"{key}: fidelity {v['fidelity']!r} not in {sorted(FIDELITY)}")
    if not str(v["rationale"]).strip():
        raise ValueError(f"{key}: missing rationale")
    return v["value"]


def leaves(sec, prefix):
    """One TOML table -> plain values; strings and booleans pass through, every number must carry its provenance."""
    return {k: v if isinstance(v, (str, bool)) else _leaf(f"{prefix}.{k}", v) for k, v in sec.items()}


def load(path=None):
    raw = tomllib.loads(Path(path or SHIPPED).read_text())
    return {s: leaves(sec, s) for s, sec in raw.items()}


def override(cfg, **kv):
    out = copy.deepcopy(cfg)
    for dotted, v in kv.items():
        s, k = dotted.split(".")
        if k not in out[s]:
            raise KeyError(dotted)
        out[s][k] = v
    return out
```

`examples/dog/autonomy/iface.py`:

```python
"""The autonomy's only door to locomotion. The carrier (carrier.py) and the dog adapter (dogloco.py) implement it."""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class Odom:
    t: float  # sim time of the pose (s)
    pos: np.ndarray  # (3,) odometry frame, drifts
    quat: np.ndarray  # (4,) w x y z, odometry frame (roll / pitch gravity-referenced)
    vel: np.ndarray  # (3,) body frame
    omega: np.ndarray  # (3,) body frame
    fallen: bool


@runtime_checkable
class Locomotion(Protocol):
    sim: object
    dt: float
    lidar: str
    gnss: str
    base: str

    def advance(self, seconds: float) -> None: ...
    def command(self, vx: float, vy: float, wz: float) -> None: ...
    def odometry(self) -> Odom: ...
    def fix(self, p_xy, yaw: float, t_pose: float) -> None: ...  # SP5's pose back to locomotion (the carrier ignores it)
    def limits(self) -> dict: ...
    def truth(self) -> tuple: ...  # harness / metrics only
```

- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_config.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/autonomy tests/python/test_dog_autonomy_config.py && git commit -m "dog autonomy: numbers-rule config and the locomotion door (Odom, Locomotion protocol)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 2: rosette point reconstruction (`lidar.py`) + the `docs/models.md` wording fix

**Files:**
- Create: `examples/dog/autonomy/lidar.py`
- Modify: `docs/models.md` (the rosette paragraph says "elevation uniform in sin"; the kernel draws it uniform in angle:
  `src/mujoco/devices/rangefinder.cpp::rosette_dir`, `el = fov_v_min + (fov_v_max - fov_v_min) frac(u2)`)
- Test: `tests/python/test_dog_autonomy_lidar.py`

**Interfaces:**
- Consumes: `Sim.query(path, attr)` of the rosette rangefinder (`frames`, `t`, `range`), `Sim.raycast`, `config.load`.
- Produces: `lidar.G1, lidar.G2, lidar.PHI` (the kernel's constants), `lidar.offsets(pattern_seed) -> (float, float)`,
  `lidar.directions(n, frame, fov_v_min, fov_v_max, pattern_seed, fov_h=2 pi) -> ndarray (n, 3)` (unit, site frame),
  `lidar.device(name, site, cfg, rays) -> dict` (rangefinder JSON), `lidar.Scan` (dataclass `t` = time of the pose the rays
  were cast from, `frame`, `dirs (m, 3)`, `r (m,)`, returns only), `lidar.Rosette(sim, name, rays, dt, cfg)` with `.latest() ->
  Scan | None`, `lidar.points(scan, pos, quat, ang_sigma=0.0, rng=None) -> ndarray (m, 3)` (world points of a site pose),
  `lidar.quat_zyx(yaw, pitch, roll)`, `lidar.qmul(a, b)`, `lidar.qmat(q)`, `lidar.yaw_of(q)` (quaternions w x y z),
  `lidar.extrinsic(xml_text, site, body) -> (pos (3,), quat (4,))` (site pose in the body frame through the static chain; `pos`,
  `quat`, `euler` (degrees, intrinsic xyz = MuJoCo's defaults) only; anything else -> `ValueError`), `lidar.SITE_Q0`.

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_lidar.py`:

```python
import json
import math

import numpy as np
import pytest

import napl
from autonomy import config, lidar

DT = 0.01


def tilted_scene(tmp_path, rays=2000, sigma0=0.0):
    cfg = config.override(config.load(), **{"lidar.sigma0": sigma0})
    q = lidar.qmul(lidar.quat_zyx(0.3, cfg["lidar"]["tilt"], 0.0), lidar.SITE_Q0)
    dev = {"rangefinders": [lidar.device("lidar", "lidar", cfg, rays)]}
    qs = " ".join(repr(float(v)) for v in q)  # numpy 2 reprs np.float64 as "np.float64(...)"
    xml = (
        f'<mujoco><option timestep="{DT}" gravity="0 0 0"/><worldbody>'
        '<geom type="box" size="20 20 0.1" pos="0 0 -0.1"/>'
        '<geom type="box" size="0.1 3 2" pos="6.0 0 1"/>'
        '<body name="head" pos="0.2 -0.1 0.45">'
        f'<site name="lidar" quat="{qs}"/></body>'
        f"</worldbody><custom><text name=\"napl.devices\" data='{json.dumps(dev)}'/></custom></mujoco>"
    )
    p = tmp_path / "m.xml"
    p.write_text(xml)
    return napl.Sim.load(p), cfg, np.array([0.2, -0.1, 0.45]), q


def test_points_match_the_kernel_rays_on_a_tilted_site(tmp_path):
    sim, cfg, pos, q = tilted_scene(tmp_path)
    ros = lidar.Rosette(sim, "lidar", 2000, DT, cfg)
    sim.step(30)  # samples on ticks 0, 10, 20
    s = ros.latest()
    assert s.frame == 2 and s.t == pytest.approx(0.20, abs=1e-12)
    assert ros.latest() is None  # nothing new until the next frame
    R = lidar.qmat(q)
    d_world = s.dirs @ R.T
    again = sim.raycast(pos, d_world)
    np.testing.assert_allclose(again, s.r, atol=1e-9)
    w = lidar.points(s, pos, q)
    on_floor = np.abs(w[:, 2]) < 1e-6
    on_wall = np.abs(w[:, 0] - 5.9) < 1e-6
    assert (on_floor | on_wall).all() and on_wall.sum() > 20 and on_floor.sum() > 500
    full = lidar.directions(2000, 2, cfg["lidar"]["fov_v_min"], cfg["lidar"]["fov_v_max"], cfg["lidar"]["pattern_seed"])
    el = np.arcsin(np.clip(full[:, 0], -1, 1))
    assert el.min() >= cfg["lidar"]["fov_v_min"] - 1e-12 and el.max() <= cfg["lidar"]["fov_v_max"] + 1e-12
    frame0 = lidar.directions(2000, 0, cfg["lidar"]["fov_v_min"], cfg["lidar"]["fov_v_max"], cfg["lidar"]["pattern_seed"])
    assert not np.allclose(frame0, full)  # the sequence continues, frames do not repeat


def test_angular_noise_has_the_datasheet_sigma(tmp_path):
    sim, cfg, pos, q = tilted_scene(tmp_path, rays=20000)
    ros = lidar.Rosette(sim, "lidar", 20000, DT, cfg)
    sim.step(1)
    s = ros.latest()
    sig = cfg["lidar"]["ang_sigma"]
    clean = lidar.points(s, pos, q) - pos
    noisy = lidar.points(s, pos, q, sig, np.random.default_rng(0)) - pos
    cosang = (clean * noisy).sum(1) / (np.linalg.norm(clean, axis=1) * np.linalg.norm(noisy, axis=1))
    ang = np.arccos(np.clip(cosang, -1, 1))
    assert math.sqrt((ang**2).mean()) == pytest.approx(sig * math.sqrt(2), rel=0.05)


def test_quaternion_helpers_and_the_extrinsic_chain():
    for yaw in (0.0, 1.0, math.pi, -math.pi / 2):
        q = lidar.quat_zyx(yaw, 0.2, -0.1)
        assert np.isfinite(q).all() and math.cos(lidar.yaw_of(q) - yaw) == pytest.approx(1.0, abs=1e-12)
        R = lidar.qmat(q)
        cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(0.2), math.sin(0.2), math.cos(-0.1), math.sin(-0.1)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        np.testing.assert_allclose(R, Rz @ Ry @ Rx, atol=1e-12)
    R0 = lidar.qmat(lidar.SITE_Q0)
    np.testing.assert_allclose(R0, [[0, 0, 1], [0, -1, 0], [1, 0, 0]], atol=1e-12)  # +x up, +z forward
    xml = (
        '<mujoco><worldbody><body name="base"><body name="head" pos="0.25 0 0.1" euler="0 0 90">'
        '<site name="s" pos="0.05 0 0.02" quat="0 0.7071067811865476 0 0.7071067811865476"/></body></body></worldbody></mujoco>'
    )
    p, qq = lidar.extrinsic(xml, "s", "base")
    np.testing.assert_allclose(p, [0.25, 0.05, 0.12], atol=1e-12)
    np.testing.assert_allclose(lidar.qmat(qq), lidar.qmat(lidar.quat_zyx(math.pi / 2, 0, 0)) @ R0, atol=1e-12)
    with pytest.raises(ValueError, match="xyaxes"):
        lidar.extrinsic(xml.replace('euler="0 0 90"', 'xyaxes="0 1 0 -1 0 0"'), "s", "base")
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_lidar.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/lidar.py`:**

```python
"""Rosette lidar points, mirroring the kernel (src/mujoco/devices/rangefinder.cpp::rosette_dir, offsets in devices.cpp).

Ray m = frame * n + k: u1 = frac(off1 + G1 m), u2 = frac(off2 + G2 m), az = fov_h (u1 - 1/2), el = fov_v_min + (fov_v_max -
fov_v_min) u2 (uniform in angle); site-frame direction (sin el, cos el sin az, cos el cos az): +x = sensor up, +z = forward.
A sample describes the state at the start of its tick; the device's `t` is that tick's end, so the pose time is t - dt."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

G1, G2 = 0.7548776662466927, 0.5698402909980532  # 1/g, 1/g^2, g = plastic number (kernel constants)
PHI = 0.6180339887498949
SITE_Q0 = np.array([0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)])  # site +x = body up, +z = body forward, +y = body right


def offsets(pattern_seed):
    s = float(int(pattern_seed) % 1000003)
    return math.fmod(0.5 + s * PHI, 1.0), math.fmod(0.5 + s * PHI * PHI, 1.0)


def directions(n, frame, fov_v_min, fov_v_max, pattern_seed, fov_h=2 * math.pi):
    o1, o2 = offsets(pattern_seed)
    m = float(frame) * n + np.arange(n, dtype=np.float64)
    u1, u2 = o1 + G1 * m, o2 + G2 * m
    az = fov_h * (u1 - np.floor(u1) - 0.5)
    el = fov_v_min + (fov_v_max - fov_v_min) * (u2 - np.floor(u2))
    ce = np.cos(el)
    return np.stack([np.sin(el), ce * np.sin(az), ce * np.cos(az)], axis=1)


def device(name, site, cfg, rays):
    L = cfg["lidar"]
    return {
        "name": name, "site": site, "rate_hz": L["rate_hz"], "projection": "rosette", "fov_h": 2 * math.pi,
        "fov_v_min": L["fov_v_min"], "fov_v_max": L["fov_v_max"], "rays_per_frame": int(rays),
        "min_range": L["min_range"], "max_range": L["max_range"], "sigma0": L["sigma0"],
        "ray_groups": 1, "seed": int(L["seed"]), "pattern_seed": int(L["pattern_seed"]),
    }  # fmt: skip


@dataclass
class Scan:
    t: float  # time of the pose the rays were cast from
    frame: int
    dirs: np.ndarray  # (m, 3) unit, site frame, returns only
    r: np.ndarray  # (m,) range


class Rosette:
    def __init__(self, sim, name, rays, dt, cfg):
        self.sim, self.path, self.n, self.dt = sim, f"/devices/{name}", int(rays), dt
        L = cfg["lidar"]
        self.lims = (L["fov_v_min"], L["fov_v_max"], int(L["pattern_seed"]))
        self.seen = 0

    def latest(self):
        frames = int(self.sim.query(self.path, "frames")[0][0])
        if frames == self.seen:
            return None
        self.seen = frames
        r = np.asarray(self.sim.query(self.path, "range")[0], dtype=np.float64).ravel()
        t = float(self.sim.query(self.path, "t")[0][0]) - self.dt
        d = directions(self.n, frames - 1, *self.lims)
        ok = r > 0
        return Scan(t, frames - 1, d[ok], r[ok])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])  # fmt: skip


def _axis(i, a):
    q = np.zeros(4)
    q[0], q[1 + i] = math.cos(a / 2), math.sin(a / 2)
    return q


def quat_zyx(yaw, pitch, roll):
    return qmul(qmul(_axis(2, yaw), _axis(1, pitch)), _axis(0, roll))


def qmat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def points(scan, pos, quat, ang_sigma=0.0, rng=None):
    d = scan.dirs
    if ang_sigma > 0:  # the kernel's rays are exact; the datasheet angular precision goes on the reported direction
        g = rng.standard_normal(d.shape)
        g -= (g * d).sum(1, keepdims=True) * d
        d = d + ang_sigma * g
        d /= np.linalg.norm(d, axis=1, keepdims=True)
    return np.asarray(pos, float) + (scan.r[:, None] * d) @ qmat(quat).T


def _local(el):
    p = np.array([float(v) for v in el.get("pos", "0 0 0").split()])
    bad = [k for k in ("axisangle", "xyaxes", "zaxis") if k in el.attrib]
    if bad:
        raise ValueError(f"extrinsic: {el.get('name')} uses {bad[0]}; only pos / quat / euler are supported")
    if "quat" in el.attrib:
        return p, np.array([float(v) for v in el.get("quat").split()])
    if "euler" in el.attrib:
        a, b, c = (math.radians(float(v)) for v in el.get("euler").split())
        return p, qmul(qmul(_axis(0, a), _axis(1, b)), _axis(2, c))  # MuJoCo default eulerseq "xyz", degrees
    return p, np.array([1.0, 0, 0, 0])


def extrinsic(xml_text, site, body):
    root = ET.fromstring(xml_text)
    comp = root.find("compiler")
    if comp is not None and (comp.get("angle", "degree") != "degree" or comp.get("eulerseq", "xyz") != "xyz"):
        raise ValueError("extrinsic: compiler angle / eulerseq must be MuJoCo's defaults")
    parent = {c: p for p in root.iter() for c in p}
    el = next(s for s in root.iter("site") if s.get("name") == site)
    chain = []
    while el.get("name") != body or el.tag != "body":
        chain.append(el)
        el = parent[el]
    pos, q = np.zeros(3), np.array([1.0, 0, 0, 0])
    for e in reversed(chain):
        p, qe = _local(e)
        pos, q = pos + qmat(q) @ p, qmul(q, qe)
    return pos, q
```

  `docs/models.md`, rosette paragraph: replace "elevation uniform in sin between `fov_v_min` and `fov_v_max`" with "elevation
  uniform in angle between `fov_v_min` and `fov_v_max` (so the ring density per solid angle rises towards the top of the span;
  `examples/dog/autonomy/lidar.py` mirrors the sequence)".
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_lidar.py -n 8'`.
  The raycast comparison is `atol = 1e-9`, not bitwise: the kernel may contract `off + G m` into an FMA.
- [ ] **Step 5: commit** `git add examples/dog/autonomy/lidar.py docs/models.md tests/python/test_dog_autonomy_lidar.py && git commit -m "dog autonomy: rosette point reconstruction mirroring the kernel (sample time t - dt, datasheet angular noise, static extrinsic chain); models.md: rosette elevation is uniform in angle" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 3: kinematic carrier, drifting odometry, GNSS sky-view outages (`carrier.py`)

**Files:**
- Create: `examples/dog/autonomy/carrier.py`
- Test: `tests/python/test_dog_autonomy_carrier.py`

**Interfaces:**
- Consumes: `napl.terrain.field`, `napl.terrain.add_field`, `Field.height`, `Field.hfield_height`, `Sim.set("/world/carrier/fj",
  "qpos", ...)`, `Sim.set("/devices/gnss", "fix_lost", 0/1)`, `Sim.raycast`, `lidar.device`, `lidar.SITE_Q0`, `lidar.quat_zyx`,
  `lidar.qmul`, `lidar.qmat`, `iface.Odom`.
- Produces: `carrier.build(f, cfg, rays, start, out_dir) -> Path` (writes `carrier.xml`: field patches, the carrier body
  `carrier` with freejoint `fj`, sites `lidar` / `gnss`, devices `lidar` (rosette) and `gnss`), `carrier.terrain_pose(f, cfg, x, y,
  yaw) -> (pos (3,), quat (4,))`, `carrier.DriftOdom(cfg, seed)` with `.update(t, pos, quat) -> Odom`,
  `carrier.sky_blocked(sim, p, cfg) -> bool`, `carrier.Carrier(sim, f, cfg, start, seed=0)` implementing `iface.Locomotion`
  (`base = "/world/carrier"`, `lidar = "lidar"`, `gnss = "gnss"`, extra attributes `bumps: int`, `gnss_lost: bool`,
  `distance: float`).

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_carrier.py`:

```python
import math

import numpy as np
import pytest

import napl
from autonomy import carrier, config, lidar
from napl.terrain import field


def make(tmp_path, kind="scree", start=(0.0, 0.0, 0.0), rays=5000, seed=1, **ov):
    cfg = config.override(config.load(), **ov)
    f = field(kind, size=20.0, patch=10.0, seed=seed)
    p = carrier.build(f, cfg, rays, start, tmp_path)
    sim = napl.Sim.load(p)
    return carrier.Carrier(sim, f, cfg, start, seed=seed), f, cfg


def test_scree_points_match_ground_truth(tmp_path):
    c, f, cfg = make(tmp_path, **{"lidar.sigma0": 0.0})
    ros = lidar.Rosette(c.sim, "lidar", 5000, c.dt, cfg)
    ext = lidar.extrinsic((tmp_path / "carrier.xml").read_text(), "lidar", "carrier")
    c.command(1.0, 0.0, 0.2)
    errs = []
    for _ in range(10):
        c.advance(0.1)
        p_b, q_b = c.truth()  # the pose set at this chunk's start = the pose the frame was cast from
        s = ros.latest()
        assert s.t == pytest.approx(c.time_of_last_frame, abs=1e-9)
        R = lidar.qmat(q_b)
        w = lidar.points(s, p_b + R @ ext[0], lidar.qmul(q_b, ext[1]))
        keep = (np.abs(w[:, :2]) < 9.5).all(1) & (np.linalg.norm(w - p_b, axis=1) < 12)
        errs.append(w[keep, 2] - f.height(w[keep, 0], w[keep, 1]))
    e = np.abs(np.concatenate(errs))
    assert np.median(e) < 1e-3 and np.quantile(e, 0.99) < 2e-3


def test_commands_are_clamped_and_rate_limited(tmp_path):
    c, f, cfg = make(tmp_path)
    C = cfg["carrier"]
    c.command(5.0, 0.0, 3.0)
    c.advance(0.1)
    assert c.v[0] <= C["a_max"] * 0.1 + 1e-12 and c.v[2] <= C["alpha_max"] * 0.1 + 1e-12
    c.advance(3.0)
    assert c.v[0] == pytest.approx(C["v_max"]) and c.v[2] == pytest.approx(C["wz_max"])
    assert c.limits() == {"v_max": C["v_max"], "wz_max": C["wz_max"]}


def test_drift_statistics_follow_the_config():
    cfg = config.load()
    O = cfg["odom"]
    yaw_err, z_err = [], []
    for seed in range(400):
        d = carrier.DriftOdom(cfg, seed)
        for k in range(101):  # 100 m straight, 1 m per update
            o = d.update(0.1 * k, np.array([float(k), 0.0, 0.0]), np.array([1.0, 0, 0, 0]))
        yaw_err.append(lidar.yaw_of(o.quat))
        z_err.append(o.pos[2])
    assert np.std(yaw_err) == pytest.approx(O["drift_yaw"] * 10, rel=0.15)
    assert np.std(z_err) == pytest.approx(O["drift_z"] * 10, rel=0.15)


def test_bump_stops_at_a_wall(tmp_path):
    f = field("ruins", size=20.0, patch=10.0, seed=2)
    b = max((b for b in f.boxes if abs(b.rpy[2]) < 1e-9 and b.size[0] > 1.0), key=lambda b: b.size[0])
    start = (b.pos[0], b.pos[1] - b.size[1] - 1.5, math.pi / 2)
    c, f, cfg = make(tmp_path, kind="ruins", start=start, seed=2)
    c.command(1.0, 0.0, 0.0)
    c.advance(4.0)
    p, _ = c.truth()
    assert c.bumps > 0 and p[1] + cfg["carrier"]["foot_a"] <= b.pos[1] - b.size[1] + 1e-9


def test_sky_blocked_in_an_enclosure_not_in_the_open(tmp_path):
    cfg = config.load()
    walls = "".join(
        f'<geom type="box" size="{sx} {sy} 2" pos="{x} {y} 2"/>'
        for x, y, sx, sy in ((1.2, 0, 0.1, 1.3), (-1.2, 0, 0.1, 1.3), (0, 1.2, 1.3, 0.1), (0, -1.2, 1.3, 0.1))
    )
    p = tmp_path / "e.xml"
    p.write_text(f'<mujoco><worldbody><geom type="box" size="30 30 0.1" pos="0 0 -0.1"/>{walls}</worldbody></mujoco>')
    sim = napl.Sim.load(p)
    assert carrier.sky_blocked(sim, np.array([0.0, 0.0, 0.5]), cfg)
    assert not carrier.sky_blocked(sim, np.array([15.0, 0.0, 0.5]), cfg)


def test_same_seed_same_state_hash(tmp_path):
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    hs, odo = [], []
    for d in ("a", "b"):
        c, f, cfg = make(tmp_path / d)
        c.command(1.2, 0.0, 0.3)
        c.advance(1.0)
        hs.append(c.sim.state_hash())
        odo.append(c.odometry().pos.copy())
    assert hs[0] == hs[1]
    np.testing.assert_array_equal(odo[0], odo[1])
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_carrier.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/carrier.py`:**

```python
"""Kinematic carrier (harness): a pose-commanded body with the dog's sensors, gravity off, no contacts (geom group 4).
The pose is set (a logged command) at the start of each lidar frame, so every frame is cast from exactly the commanded pose;
the body follows the ground under four virtual feet. Also the harness' drifting odometry and GNSS sky-view outages."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from napl.terrain import add_field

from . import lidar
from .iface import Odom

FEET = ((1, 1), (1, -1), (-1, 1), (-1, -1))  # (fore +1 / hind -1, left +1 / right -1)


def _s(v):
    return " ".join(repr(float(x)) for x in v)  # numpy 2 reprs np.float64 as "np.float64(...)"


def terrain_pose(f, cfg, x, y, yaw):
    C = cfg["carrier"]
    a, b = C["foot_a"], C["foot_b"]
    c, s = math.cos(yaw), math.sin(yaw)
    h = {}
    for i, j in FEET:
        h[i, j] = float(f.height(x + c * i * a - s * j * b, y + s * i * a + c * j * b))
    front, rear = (h[1, 1] + h[1, -1]) / 2, (h[-1, 1] + h[-1, -1]) / 2
    left, right = (h[1, 1] + h[-1, 1]) / 2, (h[1, -1] + h[-1, -1]) / 2
    pitch = math.atan2(rear - front, 2 * a)  # front lower -> nose down -> positive pitch about +y
    roll = math.atan2(left - right, 2 * b)
    z = (front + rear) / 2 + C["base_h"]
    return np.array([x, y, z]), lidar.quat_zyx(yaw, pitch, roll)


def build(f, cfg, rays, start, out_dir):
    L, C, G = cfg["lidar"], cfg["carrier"], cfg["gnss"]
    root = ET.fromstring(f'<mujoco><option timestep="{C["dt"]!r}" gravity="0 0 0"/><worldbody><light pos="0 0 30"/></worldbody></mujoco>')
    add_field(root, f)
    p, q = terrain_pose(f, cfg, *start)
    qs = lidar.qmul(lidar.quat_zyx(0.0, L["tilt"], 0.0), lidar.SITE_Q0)
    wb = root.find("worldbody")
    body = ET.SubElement(wb, "body", name="carrier", pos=_s(p), quat=_s(q))
    ET.SubElement(body, "freejoint", name="fj")
    ET.SubElement(body, "geom", type="box", size="0.3 0.12 0.06", mass="12", contype="0", conaffinity="0", group="4")
    ET.SubElement(body, "site", name="lidar", pos=f"{L['mount_x']!r} 0 {L['mount_height'] - C['base_h']!r}", quat=_s(qs))
    ET.SubElement(body, "site", name="gnss", pos=f"{G['lever_x']!r} 0 {G['lever_z']!r}")
    gnss = {
        "name": "gnss", "site": "gnss", "rate_hz": G["rate_hz"], "lat0": G["lat0"], "lon0": G["lon0"], "alt0": G["alt0"],
        "horiz_std": G["horiz_std"], "vert_std": G["vert_std"], "vel_std": 0.0,  # GNSS velocity unused by SP5
        "latency_s": G["latency_s"], "seed": 7, "corr_time_s": G["corr_time_s"], "corr_frac": G["corr_frac"],
    }  # fmt: skip
    devs = {"rangefinders": [lidar.device("lidar", "lidar", cfg, rays)], "gnss_receivers": [gnss]}
    custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "text", name="napl.devices", data=json.dumps(devs))
    out = Path(out_dir) / "carrier.xml"
    out.write_text(ET.tostring(root, encoding="unicode"))
    return out


class DriftOdom:
    """Odometry = truth increments with random-walk drift per metre travelled (variance drift^2 * distance) and white
    roll / pitch error. Yaw drift rotates later increments, as a real leg-inertial estimator does."""

    def __init__(self, cfg, seed):
        self.o = cfg["odom"]
        self.rng = np.random.default_rng([seed, 0x0D0])
        self.prev = None
        self.p = None
        self.dyaw = 0.0

    def update(self, t, pos, quat):
        yaw = lidar.yaw_of(quat)
        R = lidar.qmat(quat)
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        if self.prev is None:
            self.p = np.array(pos, float)
        else:
            dp = pos - self.prev[0]
            ds = float(np.hypot(dp[0], dp[1]))
            g = self.rng.standard_normal(4)
            c, s = math.cos(self.dyaw), math.sin(self.dyaw)
            self.p = self.p + np.array([c * dp[0] - s * dp[1], s * dp[0] + c * dp[1], dp[2]])
            self.p += math.sqrt(ds) * np.array([self.o["drift_xy"] * g[0], self.o["drift_xy"] * g[1], self.o["drift_z"] * g[2]])
            self.dyaw += self.o["drift_yaw"] * math.sqrt(ds) * g[3]
        self.prev = (np.array(pos, float), yaw)
        e = self.o["att_sigma"] * self.rng.standard_normal(2)
        q = lidar.quat_zyx(yaw + self.dyaw, pitch + e[0], roll + e[1])
        return Odom(t, self.p.copy(), q, np.zeros(3), np.zeros(3), False)


def sky_blocked(sim, p, cfg):
    G = cfg["gnss"]
    n, el = int(G["sky_az"]), G["sky_elev"]
    az = 2 * math.pi * np.arange(n) / n
    d = np.stack([math.cos(el) * np.cos(az), math.cos(el) * np.sin(az), np.full(n, math.sin(el))], axis=1)
    r = np.asarray(sim.raycast(p, d))
    return bool(((r >= 0) & (r <= G["sky_range"])).mean() >= G["sky_block"])


class Carrier:
    base, lidar, gnss = "/world/carrier", "lidar", "gnss"

    def __init__(self, sim, f, cfg, start, seed=0):
        self.sim, self.f, self.cfg, self.C = sim, f, cfg, cfg["carrier"]
        self.dt = self.C["dt"]
        self.chunk = int(round(1.0 / (cfg["lidar"]["rate_hz"] * self.dt)))  # physics steps per lidar frame
        self.x, self.y, self.yaw = (float(v) for v in start)
        self.v = np.zeros(3)  # vx, vy, wz (body)
        self.cmd = np.zeros(3)
        self.p, self.q = terrain_pose(f, cfg, self.x, self.y, self.yaw)
        self.odo = DriftOdom(cfg, seed)
        self.last = self.odo.update(0.0, self.p, self.q)
        self.bumps, self.gnss_lost, self.distance = 0, False, 0.0
        self.lever = np.array([cfg["gnss"]["lever_x"], 0.0, cfg["gnss"]["lever_z"]])
        self.time_of_last_frame = 0.0

    def command(self, vx, vy, wz):
        C = self.C
        self.cmd = np.array([np.clip(vx, -C["v_max"], C["v_max"]), np.clip(vy, -C["v_max"] / 2, C["v_max"] / 2), np.clip(wz, -C["wz_max"], C["wz_max"])])

    def limits(self):
        return {"v_max": self.C["v_max"], "wz_max": self.C["wz_max"]}

    def truth(self):
        return self.p.copy(), self.q.copy()

    def odometry(self):
        return self.last

    def fix(self, p_xy, yaw, t_pose):
        pass  # the carrier's pose is commanded, not estimated: nothing to correct

    def _blocked(self, x, y, yaw):
        C, f = self.C, self.f
        lim = f.size / 2 - C["foot_a"]
        if abs(x) > lim or abs(y) > lim:
            return True
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(x + c * i * C["foot_a"] - s * j * C["foot_b"], y + s * i * C["foot_a"] + c * j * C["foot_b"]) for i, j in FEET]
        pts.append((x, y))
        xs, ys = np.array(pts).T
        return bool((f.height(xs, ys) - f.hfield_height(xs, ys) > C["step_max"]).any())

    def _frame(self):
        C, T = self.C, self.chunk * self.dt
        lim = np.array([C["a_max"], C["a_max"], C["alpha_max"]]) * T
        self.v = self.v + np.clip(self.cmd - self.v, -lim, lim)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = self.x + (c * self.v[0] - s * self.v[1]) * T
        y = self.y + (s * self.v[0] + c * self.v[1]) * T
        yaw = math.remainder(self.yaw + self.v[2] * T, 2 * math.pi)
        if self._blocked(x, y, yaw):
            self.bumps += 1
            self.v[:2] = 0.0
            x, y = self.x, self.y
        self.distance += math.hypot(x - self.x, y - self.y)
        self.x, self.y, self.yaw = x, y, yaw
        self.p, self.q = terrain_pose(self.f, self.cfg, x, y, yaw)
        self.sim.set("/world/carrier/fj", "qpos", np.concatenate([self.p, self.q]))
        lost = sky_blocked(self.sim, self.p + lidar.qmat(self.q) @ self.lever, self.cfg)
        if lost != self.gnss_lost:
            self.sim.set("/devices/gnss", "fix_lost", 1.0 if lost else 0.0)
            self.gnss_lost = lost
        self.time_of_last_frame = self.sim.tick * self.dt
        self.sim.step(self.chunk)
        self.last = self.odo.update(self.time_of_last_frame, self.p, self.q)

    def advance(self, seconds):
        for _ in range(int(round(seconds / (self.chunk * self.dt)))):
            self._frame()
```

  The `Odom.t` of a frame is its pose time (chunk start), the same instant the lidar and GNSS sample. `vy` is clamped to half of
  `v_max` (a trotting dog's lateral speed); the planner sends `vy = 0` anyway.
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_carrier.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/autonomy/carrier.py tests/python/test_dog_autonomy_carrier.py && git commit -m "dog autonomy: kinematic carrier on terrain fields (pose per lidar frame, four-foot ground following, bumps), drifting odometry, GNSS sky-view outages via fix_lost" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 4: carrier sim budget (`tools/bench_autonomy.py`) and the rays / patch decision (`docs/dog/autonomy.md`)

**Files:**
- Create: `tools/bench_autonomy.py`, `docs/dog/autonomy.md`
- Test: `tests/python/test_dog_autonomy_bench.py`

**Interfaces:**
- Consumes: `carrier.build`, `carrier.Carrier`, `config.load`, `napl.terrain.field`.
- Produces: `tools/bench_autonomy.py [--kinds rubble,slopes,scree,ruins] [--rays 2500,5000,10000,20000] [--patch 10,5]
  [--sim 5.0]` printing a markdown table `| kind | patch | 2.5k | 5k | 10k | 20k |` of sim-s per wall-s of the carrier
  (kernel only); `docs/dog/autonomy.md` sections "Sim budget", "Wall-time model", "Decision: rays and patches",
  "Upgrade path: GPU raycaster" (later tasks append "Mapping quality", "Coverage", "Localization", "Monte Carlo", "On the dog").

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_bench.py`:

```python
import re
import subprocess


def test_bench_prints_one_row_per_kind_and_patch():
    out = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "bench_autonomy.py"), "--kinds", "ruins", "--rays", "2500", "--patch", "10,5", "--sim", "0.5"],
        capture_output=True, text=True, check=True, cwd=ROOT,
    ).stdout
    rows = [l for l in out.splitlines() if l.startswith("| ruins |")]
    assert len(rows) == 2
    for r in rows:
        assert float(re.split(r"\s*\|\s*", r.strip("| "))[-1]) > 0
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_bench.py -n 8'`
- [ ] **Step 3: implement `tools/bench_autonomy.py`:**

```python
"""SP5 sim budget: sim-seconds per wall-second of the kinematic carrier (rosette lidar + RTK GNSS, no legs) driving 1.5 m/s with a
slow turn over each 100 x 100 m terrain kind, per rays/frame and patch size. Kernel only (the autonomy's Python is measured by
the missions, Task 9). Run on hfbox, one job at a time:

    .venv/bin/python tools/bench_autonomy.py [--kinds ...] [--rays 2500,5000,10000,20000] [--patch 10,5] [--sim 5.0]"""

import argparse
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples" / "dog"))

import napl  # noqa: E402
from autonomy import carrier, config  # noqa: E402
from napl.terrain import field  # noqa: E402

START = {"ruins": (2.5, 2.5, 0.0)}  # a ruins cell centre (walls on the 5 m grid)


def run(kind, rays, patch, sim_s, tmp):
    cfg = config.load()
    f = field(kind, seed=3, patch=patch)
    start = START.get(kind, (0.0, 0.0, 0.0))
    s = napl.Sim.load(carrier.build(f, cfg, rays, start, tmp))
    c = carrier.Carrier(s, f, cfg, start)
    c.command(1.5, 0.0, 0.1)
    c.advance(0.5)  # first frames (lazy setup)
    t0 = time.perf_counter()
    c.advance(sim_s)
    return sim_s / (time.perf_counter() - t0)


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--kinds", default="rubble,slopes,scree,ruins")
    a.add_argument("--rays", default="2500,5000,10000,20000")
    a.add_argument("--patch", default="10,5")
    a.add_argument("--sim", type=float, default=5.0)
    a = a.parse_args()
    rays = [int(r) for r in a.rays.split(",")]
    print("| kind | patch | " + " | ".join(f"{r / 1000:g}k" for r in rays) + " |")
    print("|---|---|" + "---|" * len(rays))
    with tempfile.TemporaryDirectory() as tmp:
        for kind in a.kinds.split(","):
            for patch in (float(p) for p in a.patch.split(",")):
                row = [f"{run(kind, r, patch, a.sim, tmp):.2f}" for r in rays]
                print(f"| {kind} | {patch:g} m | " + " | ".join(row) + " |", flush=True)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_bench.py -n 8'`
- [ ] **Step 5: measure** (one job, no other load; ~25 min):
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/python tools/bench_autonomy.py --sim 5.0'`.
  Expected order (from `docs/models.md` "Ground sim budget", whose dog had physics at dt 2 ms on top): rubble 5k ~1.3-1.6,
  ruins 5k ~4.5-6, slopes 5k ~0.13-0.15, 20k about 4x slower than 5k.
- [ ] **Step 6: write `docs/dog/autonomy.md`** with: the measured table; the wall-time model `wall = T (1 / s_kernel + c_py)`
  with `c_py = 0.25` (budget, replaced by Task 9's measurement) and the resulting 20 min mission times per kind and ray count
  (a row per kind at 5k and 20k); **Decision: rays and patches**: patch size = the faster of 10 m / 5 m per kind (patch is a
  `field()` argument, the terrain is the same heights), missions and MC at `lidar.rays_mission` (5k) scored at `plan.pool` 2,
  mapping quality at 20k on short runs, one 20k headline mission in `bg`; rule: if rubble 5k measures below 1.0 sim-s per wall-s,
  set `rays_mission` to 2500 and `pool` to 3 (`20000 / 0.05^2 = 2500 / 0.1414^2`, rounded to the 0.15 m pool; Task 10's density
  test then pins pool 3 instead) and say so in the doc; **Upgrade path: GPU raycaster**: a batched ray caster (e.g. MuJoCo
  Warp's ray kernels or a CUDA/HIP BVH over the hfield triangles) would make the 20k missions cheap; out of SP5's scope.
- [ ] **Step 7: commit** `git add tools/bench_autonomy.py docs/dog/autonomy.md tests/python/test_dog_autonomy_bench.py && git commit -m "dog autonomy: carrier sim budget per kind / rays / patch, wall-time model, rays and patch decision" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 5: traversability on 0.5 m blocks (`traverse.py`)

**Files:**
- Create: `examples/dog/autonomy/traverse.py`
- Test: `tests/python/test_dog_autonomy_traverse.py`

**Interfaces:**
- Consumes: a height grid `h (n, n)` float (row = y, column = x, `nan` = unknown), `config.load()["trav"]`, `map.res`.
- Produces: `traverse.blocks(a, k) -> ndarray (nb, nb, k*k)`, `traverse.layers(h, res, k) -> dict(slope, step, rough, known)`
  (each `(nb, nb)`; `slope` rad, `step` / `rough` m, `known` fraction), `traverse.cost(L, cfg) -> ndarray (nb, nb)` (`inf` =
  lethal, 1 = free, `unknown_cost` when `known < min_known`, `+ near_lethal` next to a lethal block),
  `traverse.lethal(L, cfg) -> bool (nb, nb)`, `traverse.dilate(mask) -> bool` (8-neighbour), `traverse.reachable(passable,
  start) -> bool (nb, nb)` (8-connected, no corner cutting: the same moves as `planner.astar`), `traverse.Traversability(cfg, n)`
  with `.update(h, window=None) -> cost` (`window = (i0, i1, j0, j1)` map cells, half-open; `None` = everything) and
  attributes `L`, `cost`, `k`.

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_traverse.py`:

```python
import math

import numpy as np
import pytest

from autonomy import config, traverse

CFG = config.load()
RES, K = 0.05, 10  # 0.5 m blocks of 5 cm cells


def grid(n=40):
    i = np.arange(n) * RES
    return np.meshgrid(i, i)  # x (columns), y (rows)


def test_slope_threshold():
    x, y = grid()
    for deg, lethal in ((20, False), (35, True)):
        h = x * math.tan(math.radians(deg))
        L = traverse.layers(h, RES, K)
        assert np.degrees(L["slope"]) == pytest.approx(np.full((4, 4), deg), abs=0.01)
        assert traverse.lethal(L, CFG).all() == lethal


def test_step_threshold_also_across_a_block_border():
    x, _ = grid()
    for dz, lethal in ((0.15, False), (0.20, True)):
        h = np.where(x >= 1.0, dz, 0.0)  # step between columns 19 and 20: the border of blocks 1 and 2
        L = traverse.layers(h, RES, K)
        assert L["step"][:, 1] == pytest.approx(dz) and traverse.lethal(L, CFG)[:, 1].all() == lethal


def test_roughness_threshold():
    x, y = grid()
    for amp, lethal in ((0.03, False), (0.20, True)):
        h = amp * np.cos(2 * math.pi * x / 0.5) * np.cos(2 * math.pi * y / 0.5)  # egg-crate: no plane component in a block
        L = traverse.layers(h, RES, K)
        assert L["rough"] == pytest.approx(np.full((4, 4), amp / 2), rel=1e-6)
        assert np.abs(L["slope"]).max() < 1e-9 and L["step"].max() < CFG["trav"]["step_max"]
        assert traverse.lethal(L, CFG).all() == lethal


def test_unknown_block_and_near_lethal_cost():
    h = np.zeros((60, 60))
    h[:10, :10] = np.nan  # block (0, 0) unseen
    h[20:30, 20:30] = 1.0  # a 1 m box filling block (2, 2)
    c = traverse.cost(traverse.layers(h, RES, K), CFG)
    T = CFG["trav"]
    assert c[0, 0] == T["unknown_cost"]
    # the jumps at the 19|20 borders belong to the lower blocks (2, 1) and (1, 2); the 29|30 ones to (2, 2) itself
    assert math.isinf(c[2, 2]) and math.isinf(c[2, 1]) and math.isinf(c[1, 2])
    assert c[2, 3] == 1.0 + T["near_lethal"] and c[3, 3] == 1.0 + T["near_lethal"]
    assert c[5, 5] == 1.0


def test_window_update_equals_full_recompute():
    rng = np.random.default_rng(0)
    h = 0.02 * rng.standard_normal((200, 200))
    t = traverse.Traversability(CFG, 200)
    t.update(h)
    h2 = h.copy()
    h2[57:83, 101:140] += 0.5  # a new boulder
    win = t.update(h2, window=(57, 83, 101, 140))
    full = traverse.Traversability(CFG, 200).update(h2)
    assert np.isinf(full).any()
    np.testing.assert_allclose(win, full, rtol=1e-12)  # same blocks, same arithmetic; inf where full has inf


def test_reachable_respects_walls_and_corners():
    p = np.ones((7, 7), bool)
    p[2, 2:5] = p[4, 2:5] = p[2:5, 2] = p[2:5, 4] = False  # a closed ring around (3, 3)
    r = traverse.reachable(p, (0, 0))
    assert not r[3, 3] and r[6, 6]
    q = np.ones((3, 3), bool)
    q[0, 1] = q[1, 0] = False  # diagonal (0, 0) -> (1, 1) would cut two corners
    q[1, 2] = q[2, 1] = False
    assert not traverse.reachable(q, (0, 0))[1, 1]
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_traverse.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/traverse.py`:**

```python
"""Traversability of 0.5 m blocks from a 5 cm height grid: plane fit (slope, roughness), largest neighbour jump (step)."""

from collections import deque

import numpy as np

MOVES = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def blocks(a, k):
    nb = a.shape[0] // k
    return a[: nb * k, : nb * k].reshape(nb, k, nb, k).transpose(0, 2, 1, 3).reshape(nb, nb, k * k)


def layers(h, res, k):
    h = np.asarray(h, dtype=np.float64)
    known = np.isfinite(h)
    hb, wb = blocks(np.where(known, h, 0.0), k), blocks(known.astype(np.float64), k)
    v, u = np.divmod(np.arange(k * k), k)
    U = np.stack([u * res, v * res, np.ones(k * k)], axis=1)  # (k*k, 3), the same in every block
    A = np.einsum("...c,ci,cj->...ij", wb, U, U)
    b = np.einsum("...c,ci->...i", wb * hb, U)
    coef = np.einsum("...ij,...j->...i", np.linalg.pinv(A), b)  # weighted LSQ plane per block (pinv: < 3 points is fine)
    resid = (hb - np.einsum("ci,...i->...c", U, coef)) * wb
    n = wb.sum(-1)
    with np.errstate(invalid="ignore", divide="ignore"):
        rough = np.sqrt((resid**2).sum(-1) / n)
    dx = np.full(h.shape, np.nan)
    dy = np.full(h.shape, np.nan)
    dx[:, :-1] = np.abs(h[:, 1:] - h[:, :-1])  # pair (i, i+1) belongs to cell i's block: borders are covered
    dy[:-1, :] = np.abs(h[1:, :] - h[:-1, :])
    st = np.fmax(dx, dy)
    step = blocks(np.where(np.isfinite(st), st, 0.0), k).max(-1)
    return {
        "slope": np.arctan(np.hypot(coef[..., 0], coef[..., 1])),
        "step": step,
        "rough": np.where(n > 0, rough, 0.0),
        "known": n / (k * k),
    }


def lethal(L, cfg):
    T = cfg["trav"]
    judged = L["known"] >= T["min_known"]
    bad = (L["slope"] > T["slope_max"]) | (L["step"] > T["step_max"]) | (L["rough"] > T["rough_max"])
    return judged & bad


def dilate(m):
    p = np.pad(m, 1)
    out = m.copy()
    n = m.shape
    for di, dj in MOVES:
        out |= p[1 + di : 1 + di + n[0], 1 + dj : 1 + dj + n[1]]
    return out


def cost(L, cfg):
    T = cfg["trav"]
    leth = lethal(L, cfg)
    c = np.where(L["known"] >= T["min_known"], 1.0, T["unknown_cost"])
    c = c + np.where(dilate(leth) & ~leth, T["near_lethal"], 0.0)
    return np.where(leth, np.inf, c)


def reachable(passable, start):
    nb0, nb1 = passable.shape
    seen = np.zeros_like(passable, dtype=bool)
    if not passable[start]:
        return seen
    seen[start] = True
    q = deque([start])
    while q:
        i, j = q.popleft()
        for di, dj in MOVES:
            a, b = i + di, j + dj
            if not (0 <= a < nb0 and 0 <= b < nb1) or seen[a, b] or not passable[a, b]:
                continue
            if di and dj and not (passable[i + di, j] and passable[i, j + dj]):
                continue  # no corner cutting
            seen[a, b] = True
            q.append((a, b))
    return seen


class Traversability:
    def __init__(self, cfg, n):
        self.cfg, self.res = cfg, cfg["map"]["res"]
        self.k = int(round(cfg["trav"]["block"] / self.res))
        nb = n // self.k
        z = np.zeros((nb, nb))
        self.L = {"slope": z.copy(), "step": z.copy(), "rough": z.copy(), "known": z.copy()}
        self.cost = np.full((nb, nb), cfg["trav"]["unknown_cost"])

    def update(self, h, window=None):
        k, nb = self.k, self.cost.shape[0]
        if window is None:
            b0, b1, c0, c1 = 0, nb, 0, nb
        else:  # blocks touched by the changed cells, +1 (step pairs at the border, near-lethal ring)
            i0, i1, j0, j1 = window
            b0, b1 = max(i0 // k - 1, 0), min((i1 - 1) // k + 2, nb)
            c0, c1 = max(j0 // k - 1, 0), min((j1 - 1) // k + 2, nb)
        sub = h[b0 * k : min(b1 * k + 1, h.shape[0]), c0 * k : min(c1 * k + 1, h.shape[1])]
        Ls = layers(sub, self.res, k)  # the extra row / column gives the border step pairs, blocks() drops the partial block
        for key in self.L:
            self.L[key][b0:b1, c0:c1] = Ls[key][: b1 - b0, : c1 - c0]
        r0, r1, s0, s1 = max(b0 - 1, 0), min(b1 + 1, nb), max(c0 - 1, 0), min(c1 + 1, nb)
        e0, e1, f0, f1 = max(r0 - 1, 0), min(r1 + 1, nb), max(s0 - 1, 0), min(s1 + 1, nb)
        Lw = {key: v[e0:e1, f0:f1] for key, v in self.L.items()}
        self.cost[r0:r1, s0:s1] = cost(Lw, self.cfg)[r0 - e0 : r1 - e0, s0 - f0 : s1 - f0]
        return self.cost
```

  `layers` on a sub-window: the window's last row / column of blocks needs the cell after it for its border step pair, hence the
  `+ 1` cell; `blocks()` truncates the partial block. On a 2000 x 2000 map a full `update` is ~0.3 s (einsum + pinv of 40 000 3x3);
  the 10 Hz map changes ~12 m x 12 m around the robot, so the windowed update is ~10 ms (measured in Task 9's timing columns).
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_traverse.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/autonomy/traverse.py tests/python/test_dog_autonomy_traverse.py && git commit -m "dog autonomy: traversability on 0.5 m blocks (weighted plane slope / roughness, border-aware step, near-lethal cost, windowed update, corner-safe reachability)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 6: elevation map (`elevmap.py`) and ground-truth scoring (`metrics.py`)

**Files:**
- Create: `examples/dog/autonomy/elevmap.py`, `examples/dog/autonomy/metrics.py`
- Test: `tests/python/test_dog_autonomy_map.py`

**Interfaces:**
- Consumes: `config.load()["map"]`, `["lidar"]`, `traverse.layers`, `traverse.lethal`, `traverse.reachable`, `Field.height`,
  `Field.hfield_height`, `Field.boxes`; for the kernel test `carrier.*`, `lidar.*`.
- Produces: `elevmap.ElevationMap(cfg)`: arrays `h` (float32 n x n, row = y, column = x, cell centre `x0 + (i + 0.5) res`),
  `v` (float32, `inf` = unseen), `hn` (float32, `h` with `nan` where unseen: the input of `traverse`), `hits` (uint32); `x0 = -size / 2`, `n`, `res`, `version: int`, `dirty: (i0, i1, j0, j1) | None`
  (rows, columns, half-open, cells changed by the last fuse); `.cells(x, y) -> (row, col, inside)`;
  `.fuse(pts (m, 3), origin (3,), var_pz, att_sigma)` (point variance `sigma0^2 dz^2 + (ang_sigma^2 + att_sigma^2) rh^2`,
  `dz` = vertical cosine of the ray, `rh` = horizontal range; points beyond `map.max_range` dropped); `.fuse_z(x, y, z, var_point,
  var_common)` (per frame: inverse-variance merge per cell, frame variance `1 / sum(w) + var_common`; then per cell a 1-D Kalman
  update with a `gate`-sigma test: higher -> replace, lower -> drop); `.height(x, y) -> ndarray | float` (world z, `nan` = unseen:
  SP4's `HeightMap` protocol); `.var(x, y)`; `.mapped(pool) -> bool (n // pool, n // pool)` (pooled inverse-variance sum below
  `var_mapped`). `metrics.truth_grid(f, res) -> float64 (n, n)` (ground truth at cell centres), `metrics.edge_mask(gt, cfg) -> bool`,
  `metrics.targets(gt, cfg, start_xy) -> bool (n, n)` (cells of ground-truth reachable, traversable blocks; the coverage
  denominator), `metrics.coverage(m, tgt, pool) -> float`, `metrics.height_rms(m, gt, edges) -> dict(rms, rms_edges, n, n_edges)`.

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_map.py`:

```python
import math

import numpy as np
import pytest

import napl
from autonomy import carrier, config, elevmap, lidar, metrics
from napl.terrain import field

CFG = config.load()
SMALL = config.override(CFG, **{"map.size": 10.0})  # 200 x 200 cells


def test_gate_keeps_the_higher_surface_and_kalman_shrinks_variance():
    m = elevmap.ElevationMap(SMALL)
    x, y = np.array([0.01]), np.array([0.01])
    m.fuse_z(x, y, np.array([0.0]), np.array([1e-4]), 0.0)
    assert m.var(0.01, 0.01) == pytest.approx(1e-4, rel=1e-6)
    m.fuse_z(x, y, np.array([0.01]), np.array([1e-4]), 0.0)  # within the gate: Kalman
    assert m.var(0.01, 0.01) == pytest.approx(5e-5, rel=1e-6) and m.height(0.01, 0.01) == pytest.approx(0.005, abs=1e-7)
    m.fuse_z(x, y, np.array([0.30]), np.array([1e-4]), 0.0)  # higher by > 3 sigma: a surface on top -> replace
    assert m.height(0.01, 0.01) == pytest.approx(0.30, abs=1e-7) and m.var(0.01, 0.01) == pytest.approx(1e-4, rel=1e-6)
    m.fuse_z(x, y, np.array([0.0]), np.array([1e-4]), 0.0)  # lower by > 3 sigma: dropped (no smearing below a top)
    assert m.height(0.01, 0.01) == pytest.approx(0.30, abs=1e-7) and int(m.hits[100, 100]) == 3
    m2 = elevmap.ElevationMap(SMALL)  # four points of one frame in one cell: mean + the frame's common variance
    m2.fuse_z(np.full(4, 0.01), np.full(4, 0.01), np.array([0.0, 0.02, 0.0, 0.02]), np.full(4, 4e-4), 1e-4)
    assert m2.height(0.01, 0.01) == pytest.approx(0.01, abs=1e-7) and m2.var(0.01, 0.01) == pytest.approx(2e-4, rel=1e-6)
    assert math.isnan(m2.height(3.0, 3.0)) and math.isinf(m2.var(3.0, 3.0)) and math.isnan(m2.height(9.0, 0.0))
    assert m2.dirty == (100, 101, 100, 101)


def test_point_variance_model():
    m = elevmap.ElevationMap(SMALL)
    L = CFG["lidar"]
    o = np.array([0.0, 0.0, 2.0])
    pts = np.array([[0.01, 0.01, 0.0], [4.0, 0.01, 0.0], [20.0, 0.0, 0.0]])  # down, 4 m out, beyond max_range
    m.fuse(pts, o, 1e-4, 0.005)
    assert m.var(0.01, 0.01) == pytest.approx(L["sigma0"] ** 2 + 0.005**2 * 0.01**2 * 2 + L["ang_sigma"] ** 2 * 0.01**2 * 2 + 1e-4, rel=1e-3)
    r2, rh2 = 4.0**2 + 0.01**2 + 4.0, 4.0**2 + 0.01**2
    exp = L["sigma0"] ** 2 * 4.0 / r2 + (L["ang_sigma"] ** 2 + 0.005**2) * rh2 + 1e-4
    assert m.var(4.0, 0.01) == pytest.approx(exp, rel=1e-3)
    assert int(m.hits.sum()) == 2


def test_truth_grid_is_ground_truth_at_cell_centres():
    f = field("rubble", size=20.0, patch=10.0, seed=1)
    g = metrics.truth_grid(f, 0.05)
    rng = np.random.default_rng(0)
    r, c = rng.integers(0, 400, 500), rng.integers(0, 400, 500)
    xc, yc = -10.0 + (c + 0.5) * 0.05, -10.0 + (r + 0.5) * 0.05
    np.testing.assert_array_equal(g[r, c], f.height(xc, yc))
    b = f.boxes[0]
    i, j = int((b.pos[0] + 10.0) / 0.05), int((b.pos[1] + 10.0) / 0.05)
    assert g[j, i] >= b.pos[2] + b.size[2]  # rubble always has 40 boulders; another one may overlap higher


def ring_world():
    n = 200
    gt = np.zeros((n, n))
    gt[60:70, 60:140] = gt[130:140, 60:140] = gt[60:140, 60:70] = gt[60:140, 130:140] = 1.0  # walls: blocks 6 and 13
    x = np.arange(n) * 0.05
    gt[0:40, 160:200] = (x[160:200] - x[160]) * math.tan(math.radians(35))  # a 35 deg ramp
    return gt


def test_coverage_denominator_is_ground_truth_reachable_traversable():
    gt = ring_world()
    tgt = metrics.targets(gt, SMALL, (-4.5, -4.5))
    assert tgt[20, 20] and not tgt[100, 100] and not tgt[65, 100] and not tgt[20, 180]
    m = elevmap.ElevationMap(SMALL)
    m.h[:], m.v[:] = gt, 1e-4
    assert metrics.coverage(m, tgt, 1) == 1.0
    m.h[10:30, :] += 1.0  # the robot's map believes in a wall: the denominator does not move
    assert metrics.coverage(m, tgt, 1) == 1.0 and metrics.targets(gt, SMALL, (-4.5, -4.5)).sum() == tgt.sum()
    m.v[:, 100:] = np.inf  # only the west half seen
    assert metrics.coverage(m, tgt, 1) == pytest.approx(tgt[:, :100].sum() / tgt.sum())
    assert metrics.coverage(m, tgt, 2) == pytest.approx(tgt[:, :100].sum() / tgt.sum(), abs=0.01)


def test_edge_cells_are_scored_separately():
    gt = np.zeros((200, 200))
    gt[80:120, 80:120] = 1.0
    m = elevmap.ElevationMap(SMALL)
    m.h[:], m.v[:] = gt, 1e-4
    edges = metrics.edge_mask(gt, SMALL)
    m.h[edges] = 0.5  # mixed returns on the box sides
    r = metrics.height_rms(m, gt, edges)
    assert r["rms"] == 0.0 and r["rms_edges"] > 0.3 and r["n_edges"] == edges.sum() and 0 < r["n_edges"] < 400


def test_rubble_pass_at_20k_rays_with_truth_pose_maps_within_3cm(tmp_path):
    cfg = config.override(CFG, **{"map.size": 20.0})
    f = field("rubble", size=20.0, patch=10.0, seed=4, rocks=100, boulders=2)  # the 100 x 100 m densities scaled to 400 m^2
    start = (-8.0, 0.0, 0.0)
    sim = napl.Sim.load(carrier.build(f, cfg, 20000, start, tmp_path))
    c = carrier.Carrier(sim, f, cfg, start)
    ros = lidar.Rosette(sim, "lidar", 20000, c.dt, cfg)
    ext = lidar.extrinsic((tmp_path / "carrier.xml").read_text(), "lidar", "carrier")
    m, rng = elevmap.ElevationMap(cfg), np.random.default_rng(0)
    c.command(1.5, 0.0, 0.0)
    for _ in range(110):
        c.advance(0.1)
        p, q = c.truth()
        s = ros.latest()
        o = p + lidar.qmat(q) @ ext[0]
        m.fuse(lidar.points(s, o, lidar.qmul(q, ext[1]), cfg["lidar"]["ang_sigma"], rng), o, 0.0, 0.0)
    gt = metrics.truth_grid(f, cfg["map"]["res"])
    r = metrics.height_rms(m, gt, metrics.edge_mask(gt, cfg))
    assert r["rms"] <= 0.03 and r["n"] > 20000
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_map.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/elevmap.py`:**

```python
"""Global elevation map, 5 cm cells, height + variance (Fankhauser et al. 2014, "Robot-centric elevation mapping with
uncertainty estimates", fusion rule; kept in the fixed map frame here because RTK bounds the drift). Error budget at 4 m on the
carrier (docs/dog/autonomy.md): RTK vertical ~2 cm (correlated part), attitude 0.3 deg ~2.1 cm, angular precision ~1.0 cm,
range ~0.5 cm, discretisation ~0.8 cm -> ~3.2 cm in quadrature against the 5 cm target."""

import numpy as np


class ElevationMap:
    def __init__(self, cfg):
        M, L = cfg["map"], cfg["lidar"]
        self.res, self.x0 = M["res"], -M["size"] / 2
        self.n = int(round(M["size"] / self.res))
        self.h = np.zeros((self.n, self.n), np.float32)
        self.v = np.full((self.n, self.n), np.inf, np.float32)
        self.hits = np.zeros((self.n, self.n), np.uint32)
        self.hn = np.full((self.n, self.n), np.nan, np.float32)  # h with nan where unseen (what traverse reads)
        self.gate2, self.floor, self.vmap, self.maxr = M["gate"] ** 2, M["var_floor"], M["var_mapped"], M["max_range"]
        self.sr2, self.sa2 = L["sigma0"] ** 2, L["ang_sigma"] ** 2
        self.version, self.dirty = 0, None

    def cells(self, x, y):
        c = np.floor((np.asarray(x, float) - self.x0) / self.res).astype(np.int64)
        r = np.floor((np.asarray(y, float) - self.x0) / self.res).astype(np.int64)
        inside = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
        return np.clip(r, 0, self.n - 1), np.clip(c, 0, self.n - 1), inside

    def fuse(self, pts, origin, var_pz, att_sigma):
        d = pts - origin
        rh2 = d[:, 0] ** 2 + d[:, 1] ** 2
        r2 = rh2 + d[:, 2] ** 2
        ok = (r2 > 0) & (r2 <= self.maxr**2)
        vp = self.sr2 * d[:, 2] ** 2 / np.where(ok, r2, 1.0) + (self.sa2 + att_sigma**2) * rh2
        self.fuse_z(pts[ok, 0], pts[ok, 1], pts[ok, 2], vp[ok], var_pz)

    def fuse_z(self, x, y, z, var_point, var_common):
        r, c, inside = self.cells(x, y)
        if not inside.any():
            self.dirty = None
            return
        idx = (r * self.n + c)[inside]
        u, inv = np.unique(idx, return_inverse=True)
        w = 1.0 / np.maximum(var_point[inside], self.floor)
        W = np.bincount(inv, w)
        Z = np.bincount(inv, w * z[inside]) / W
        V = 1.0 / W + var_common
        hf, vf, nf = self.h.reshape(-1), self.v.reshape(-1), self.hits.reshape(-1)
        h0, v0 = hf[u].astype(np.float64), vf[u].astype(np.float64)
        new = ~np.isfinite(v0)
        s = v0 + V
        dz = Z - h0
        far = ~new & (dz * dz > self.gate2 * s)
        take = new | (far & (dz > 0))
        keep = ~(far & (dz < 0))
        with np.errstate(invalid="ignore"):
            hn = np.where(take, Z, (V * h0 + v0 * Z) / s)
            vn = np.where(take, V, v0 * V / s)
        k = u[keep]
        hf[k], vf[k] = hn[keep], np.maximum(vn[keep], self.floor)
        self.hn.reshape(-1)[k] = hn[keep]
        nf[k] += 1
        rr, cc = np.divmod(u, self.n)
        self.dirty = (int(rr.min()), int(rr.max()) + 1, int(cc.min()), int(cc.max()) + 1)
        self.version += 1

    def height(self, x, y):
        r, c, inside = self.cells(x, y)
        v = self.v[r, c]
        out = np.where(inside & np.isfinite(v), self.h[r, c].astype(np.float64), np.nan)
        return float(out) if np.ndim(out) == 0 else out

    def var(self, x, y):
        r, c, inside = self.cells(x, y)
        out = np.where(inside, self.v[r, c].astype(np.float64), np.inf)
        return float(out) if np.ndim(out) == 0 else out

    def mapped(self, pool):
        m = (self.n // pool) * pool
        info = np.where(np.isfinite(self.v[:m, :m]), 1.0 / self.v[:m, :m].astype(np.float64), 0.0)
        info = info.reshape(m // pool, pool, m // pool, pool).sum(axis=(1, 3))
        return info > 1.0 / self.vmap
```

  `examples/dog/autonomy/metrics.py` (harness: reads ground truth):

```python
"""Ground-truth scoring of a map. Harness only: the robot modules never import this."""

import math

import numpy as np

from . import traverse

ROWS = 200  # truth_grid works in row bands of 200 to bound memory


def truth_grid(f, res):
    n = int(round(f.size / res))
    xc = -f.size / 2 + (np.arange(n) + 0.5) * res
    g = np.empty((n, n))
    for r0 in range(0, n, ROWS):
        X, Y = np.meshgrid(xc, xc[r0 : r0 + ROWS])
        g[r0 : r0 + ROWS] = f.hfield_height(X, Y)
    for b in f.boxes:  # rasterise each box inside its own bounding box only
        ext = abs(b.size[0] * math.cos(b.rpy[2])) + abs(b.size[1] * math.sin(b.rpy[2])), abs(b.size[0] * math.sin(b.rpy[2])) + abs(b.size[1] * math.cos(b.rpy[2]))
        c0, c1 = (max(int((b.pos[0] - ext[0] + f.size / 2) / res) - 1, 0), min(int((b.pos[0] + ext[0] + f.size / 2) / res) + 2, n))
        r0, r1 = (max(int((b.pos[1] - ext[1] + f.size / 2) / res) - 1, 0), min(int((b.pos[1] + ext[1] + f.size / 2) / res) + 2, n))
        X, Y = np.meshgrid(xc[c0:c1], xc[r0:r1])
        c, s = math.cos(b.rpy[2]), math.sin(b.rpy[2])
        dx, dy = X - b.pos[0], Y - b.pos[1]
        inside = (np.abs(c * dx + s * dy) <= b.size[0]) & (np.abs(-s * dx + c * dy) <= b.size[1])
        g[r0:r1, c0:c1] = np.where(inside, np.maximum(g[r0:r1, c0:c1], b.pos[2] + b.size[2]), g[r0:r1, c0:c1])
    return g


def edge_mask(gt, cfg):
    p = np.pad(gt, 1, mode="edge")
    n = gt.shape
    sh = [p[1 + a : 1 + a + n[0], 1 + b : 1 + b + n[1]] for a in (-1, 0, 1) for b in (-1, 0, 1)]
    return np.max(sh, axis=0) - np.min(sh, axis=0) > cfg["map"]["edge_range"]


def targets(gt, cfg, start_xy):
    res, size = cfg["map"]["res"], cfg["map"]["size"]
    k = int(round(cfg["trav"]["block"] / res))
    L = traverse.layers(gt, res, k)
    passable = ~traverse.lethal(L, cfg)
    s = (int((start_xy[1] + size / 2) / (k * res)), int((start_xy[0] + size / 2) / (k * res)))
    blk = traverse.reachable(passable, s)
    out = np.zeros(gt.shape, bool)
    nb = blk.shape[0]
    out[: nb * k, : nb * k] = np.kron(blk, np.ones((k, k), bool))
    return out


def coverage(m, tgt, pool):
    q = (m.n // pool) * pool
    t = tgt[:q, :q].reshape(q // pool, pool, q // pool, pool).any(axis=(1, 3))
    return float((m.mapped(pool) & t).sum() / t.sum())


def height_rms(m, gt, edges):
    seen = m.v < m.vmap
    e = m.h.astype(np.float64) - gt
    a, b = seen & ~edges, seen & edges
    return {
        "rms": float(np.sqrt((e[a] ** 2).mean())) if a.any() else math.nan,
        "rms_edges": float(np.sqrt((e[b] ** 2).mean())) if b.any() else math.nan,
        "n": int(a.sum()),
        "n_edges": int(b.sum()),
    }
```

  (`targets` uses the same 0.5 m blocks and corner rule as the planner, so a door narrower than about two blocks can read closed
  in both the robot's plan and the denominator: decision 2.)
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_map.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/autonomy/elevmap.py examples/dog/autonomy/metrics.py tests/python/test_dog_autonomy_map.py && git commit -m "dog autonomy: 5 cm elevation map (per-frame inverse-variance merge, gated Kalman: higher replaces, lower dropped) and ground-truth scoring (truth grid, edge cells, reachable traversable denominator)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 7: localization (`localize.py`: 4-DOF EKF, delayed RTK, GNSS reader) and lidar ICP (`icp.py`)

**Files:**
- Create: `examples/dog/autonomy/localize.py`, `examples/dog/autonomy/icp.py`
- Test: `tests/python/test_dog_autonomy_localize.py`

**Interfaces:**
- Consumes: `iface.Odom` (10 Hz, `t` = pose time), GNSS device `lat`, `lon`, `alt` + `Sim.tick` (the device samples on ticks
  `k % period == 0`; output = the sample `delay = round(latency_s rate_hz)` samples older; `fix_lost` holds the output),
  `lidar.Scan`, `lidar.qmat`, `lidar.qmul`, `lidar.yaw_of`, `lidar.quat_zyx`, `config.load()["loc"]`, `["odom"]`, `["gnss"]`, `["icp"]`.
- Produces: `localize.enu(lat, lon, alt, G) -> ndarray (3,)`, `localize.wrap(a)`, `localize.rz(a) -> (3, 3)`,
  `localize.GnssReader(sim, name, cfg, dt)` with `.poll() -> (t_meas, enu (3,)) | None` (`None`: no new sample, warm-up, a held
  output = outage, or within `reconverge_s` after an outage), `localize.Localizer(cfg, x0 (4,), lever (3,))` with `.x (4,)`,
  `.P (4, 4)`, `.predict(odom)`, `.pose() -> (pos (3,), quat (4,))`, `.update(z, h, H, R, chi2, wrap_last=False) -> bool`,
  `.update_rtk(t_meas, enu) -> bool`, `.update_icp(x_meas (4,), cov (4, 4)) -> bool`, `.rtk_ok(t_now) -> bool`, `.var_z -> float`;
  `icp.levelled(scan, ext, odom_quat, ang_sigma=0.0, rng=None) -> ndarray (m, 3)` (points in the yaw-free base frame, with the
  datasheet angular noise when `ang_sigma > 0`), `icp.Match` (`x (4,)`, `cov (4, 4)`,
  `inliers: int`, `degenerate: bool (4,)`), `icp.ScanMatcher(cfg)` with `.add(world_pts, x) -> bool` (keyframe gate) and
  `.match(levelled_pts, x_guess) -> Match | None`.

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_localize.py`:

```python
import math

import numpy as np
import pytest

import napl
from autonomy import carrier, config, icp, lidar, localize
from autonomy.iface import Odom
from napl.terrain import field

CFG = config.load()
LEVER = np.array([CFG["gnss"]["lever_x"], 0.0, CFG["gnss"]["lever_z"]])


def straight(yaw_map=0.7, off=0.3, t0=np.array([2.0, -1.0, 0.0]), pitch=0.1, v=1.5, n=11):
    """Truth poses along yaw_map at v, and the same motion seen in an odometry frame rotated by -off (no drift)."""
    out = []
    for k in range(n):
        t = 0.1 * k
        p = t0 + np.array([v * t * math.cos(yaw_map), v * t * math.sin(yaw_map), 0.3])
        q = lidar.quat_zyx(yaw_map, pitch, 0.0)
        po = localize.rz(-off) @ (p - t0)
        out.append((t, p, q, Odom(t, po, lidar.quat_zyx(yaw_map - off, pitch, 0.0), np.zeros(3), np.zeros(3), False)))
    return out


def antenna(p, q):
    return p + lidar.qmat(q) @ LEVER


def test_late_rtk_fix_is_compared_with_the_pose_at_its_epoch():
    run = straight()
    t, p, q, _ = run[0]
    loc = localize.Localizer(CFG, np.r_[p, 0.7], LEVER)
    for _, _, _, od in run:
        loc.predict(od)
    x_before = loc.x.copy()
    _, p9, q9, _ = run[9]  # the fix measured at t = 0.9 arrives at t = 1.0 (latency 0.1 s)
    assert loc.update_rtk(0.9, antenna(p9, q9))
    np.testing.assert_allclose(loc.x, x_before, atol=1e-9)  # exact fix at its own epoch: no correction
    loc2 = localize.Localizer(CFG, np.r_[p, 0.7], LEVER)
    for _, _, _, od in run:
        loc2.predict(od)
    loc2.update_rtk(1.0, antenna(p9, q9))  # the same fix wrongly stamped "now": pulls the pose back by ~v * latency
    assert np.linalg.norm(loc2.x[:2] - x_before[:2]) > 0.01
    loc3 = localize.Localizer(CFG, np.r_[p, 0.7], LEVER)
    for _, _, _, od in run:
        loc3.predict(od)
    assert loc3.update_rtk(0.9, antenna(p9, q9) + np.array([0.05, 0.0, 0.0]))
    assert 0.0 < loc3.x[0] - x_before[0] < 0.05


def test_gate_rejects_a_2m_jump():
    run = straight()
    loc = localize.Localizer(CFG, np.r_[run[0][1], 0.7], LEVER)
    for _, _, _, od in run:
        loc.predict(od)
    x = loc.x.copy()
    assert not loc.update_rtk(0.9, antenna(run[9][1], run[9][2]) + np.array([2.0, 0.0, 0.0]))
    np.testing.assert_array_equal(loc.x, x)


class FakeGnss:
    def __init__(self):
        self.tick, self.vals = 0, {}

    def query(self, path, attr):
        return np.array([self.vals[attr]]), "deg"


def test_held_fix_is_an_outage_and_reconvergence_waits():
    G = CFG["gnss"]
    sim = FakeGnss()
    r = localize.GnssReader(sim, "gnss", CFG, 0.01)
    got = {}
    for s in range(121):
        sim.tick = s * 10 + 1  # sample s happened on tick 10 s
        k = 4 if 5 <= s <= 8 else s  # samples 5..8 hold sample 4's output (fix_lost)
        sim.vals = {"lat": G["lat0"] + 1e-7 * k, "lon": G["lon0"], "alt": 0.1 * k}
        f = r.poll()
        if f is not None:
            got[s] = f
        assert r.poll() is None  # no new sample until the tick moves on
    assert sorted(got)[:4] == [1, 2, 3, 4] and got[1][0] == pytest.approx(0.0)
    after = [s for s in got if s > 4]
    # the first fresh sample after the hold is s = 9 (t_meas 0.8 s); fixes are usable from 0.8 + reconverge_s = 10.8 s (s = 109)
    assert after[0] in (109, 110) and got[after[0]][0] >= 0.8 + G["reconverge_s"] - 1e-9
    m, _ = localize.radii(G["lat0"])
    assert got[2][1][1] == pytest.approx(math.radians(2e-7) * m, rel=1e-9)


def room(n, rng, noise=0.005):
    """Floor z = 0.1, walls x / y = +-5.1, a pillar with x faces at 1.7 / 2.3: surfaces mid-voxel (0.2 m voxels), so the
    noise does not split a surface over two voxel layers."""
    u = rng.uniform(-1, 1, (n, 2))
    k = rng.integers(0, 6, n)
    pts = np.zeros((n, 3))
    pts[k == 0] = np.c_[5 * u[k == 0], np.full((k == 0).sum(), 0.1)]
    for j, (ax, val) in enumerate(((0, 5.1), (0, -5.1), (1, 5.1), (1, -5.1)), start=1):
        m = k == j
        pts[m, ax] = val
        pts[m, 1 - ax] = 5 * u[m, 0]
        pts[m, 2] = 1.1 + u[m, 1]
    m = k == 5
    pts[m] = np.c_[np.where(u[m, 0] > 0, 2.3, 1.7), 1 + 0.25 * u[m, 1], 1.1 + u[m, 0]]
    return pts + noise * rng.standard_normal(pts.shape)


def test_icp_recovers_an_offset_in_a_synthetic_room():
    rng = np.random.default_rng(0)
    truth = np.array([0.3, -0.2, 0.45, 0.2])
    sm = icp.ScanMatcher(CFG)
    assert sm.add(room(150000, rng), truth)  # ~10 floor points per 0.2 m voxel
    w = room(8000, rng)
    s = (w - truth[:3]) @ localize.rz(truth[3])  # levelled base frame: Rz(yaw)^T (w - p)
    m = sm.match(s, truth + np.array([0.15, -0.10, 0.05, 0.03]))
    assert np.abs(m.x[:3] - truth[:3]).max() < 0.01 and abs(m.x[3] - truth[3]) < 0.003
    assert not m.degenerate.any() and m.inliers > 1000


def test_flat_floor_is_degenerate_in_x_y_and_yaw():
    rng = np.random.default_rng(1)
    floor = np.c_[rng.uniform(-5, 5, (40000, 2)), 0.1 + 0.003 * rng.standard_normal(40000)]
    sm = icp.ScanMatcher(CFG)
    sm.add(floor, np.zeros(4))
    guess = np.array([0.1, 0.1, 0.4, 0.05])
    m = sm.match(floor[:5000] - np.array([0.0, 0.0, 0.35]), guess)  # true z = 0.45
    np.testing.assert_array_equal(m.degenerate, [True, True, False, True])
    np.testing.assert_allclose(m.x[[0, 1, 3]], guess[[0, 1, 3]], atol=1e-3)  # normals are only nearly vertical
    assert m.x[2] == pytest.approx(0.45, abs=0.005)


def test_ruins_circle_with_forced_outage_icp_bounds_the_drift(tmp_path):
    cfg = config.override(CFG, **{"gnss.sky_block": 0.0, "odom.drift_xy": 0.05, "odom.drift_yaw": 0.02, "map.size": 20.0})
    f = field("ruins", size=20.0, patch=10.0, seed=3)
    start = (4.0, 2.5, math.pi / 2)  # radius 1.5 m circle around the cell centre (2.5, 2.5)
    sim = napl.Sim.load(carrier.build(f, cfg, 5000, start, tmp_path))
    c = carrier.Carrier(sim, f, cfg, start, seed=3)
    ros = lidar.Rosette(sim, "lidar", 5000, c.dt, cfg)
    ext = lidar.extrinsic((tmp_path / "carrier.xml").read_text(), "lidar", "carrier")
    p0, q0 = c.truth()
    x0 = np.r_[p0, lidar.yaw_of(q0)]
    with_icp, odo_only = localize.Localizer(cfg, x0, LEVER), localize.Localizer(cfg, x0, LEVER)
    sm = icp.ScanMatcher(cfg)
    c.command(1.0, 0.0, 1.0 / 1.5)
    for k in range(300):
        c.advance(0.1)
        od, s = c.odometry(), ros.latest()
        with_icp.predict(od)
        odo_only.predict(od)
        pts = icp.levelled(s, ext, od.quat)
        if k:
            m = sm.match(pts, with_icp.x.copy())
            if m is not None:
                with_icp.update_icp(m.x, m.cov)
        sm.add(pts @ localize.rz(with_icp.x[3]).T + with_icp.x[:3], with_icp.x)
    assert c.gnss_lost
    p, _ = c.truth()
    e_icp, e_odo = (float(np.hypot(*(L.x[:2] - p[:2]))) for L in (with_icp, odo_only))
    assert e_icp < 0.15 and e_icp < 0.5 * e_odo
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_localize.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/localize.py`:**

```python
"""4-DOF EKF over (x, y, z, yaw) in the map frame (ENU of the GNSS origin = world, alt0 = 0). Roll / pitch come from odometry
(gravity-referenced). Prediction: the odometry increment rotated by the current map-odometry yaw offset, Q = drift^2 x distance.
RTK fixes arrive latency_s late: the pose at the fix epoch is rebuilt as the current state minus the odometry motion since then
(not a stored state: stored states go stale after every update), with the antenna lever arm at that epoch's attitude."""

import math

import numpy as np

from . import lidar

A_WGS84, F_WGS84 = 6378137.0, 1 / 298.257223563
E2 = F_WGS84 * (2 - F_WGS84)


def radii(lat0):
    s = math.sin(math.radians(lat0))
    w = 1 - E2 * s * s
    return A_WGS84 * (1 - E2) / w**1.5, A_WGS84 / math.sqrt(w)


def enu(lat, lon, alt, G):
    """Inverse of the GNSS device's flat-earth map (docs/models.md, GNSS)."""
    m, n = radii(G["lat0"])
    return np.array([math.radians(lon - G["lon0"]) * n * math.cos(math.radians(G["lat0"])), math.radians(lat - G["lat0"]) * m, alt - G["alt0"]])


def wrap(a):
    return math.remainder(a, 2 * math.pi)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _perp(v):  # d(Rz(a) u) / da = perp(Rz(a) u)
    return np.array([-v[1], v[0], 0.0])


def _qz(a):
    return np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)])


class GnssReader:
    def __init__(self, sim, name, cfg, dt):
        self.G, self.sim, self.path, self.dt = cfg["gnss"], sim, f"/devices/{name}", dt
        self.period = int(round(1.0 / (self.G["rate_hz"] * dt)))
        self.delay = int(round(self.G["latency_s"] * self.G["rate_hz"]))
        self.last_s, self.prev, self.held, self.ok_after = -1, None, False, -math.inf

    def poll(self):
        if self.sim.tick < 1:
            return None
        s = (self.sim.tick - 1) // self.period  # latest sample index (sampled on tick s * period)
        if s == self.last_s:
            return None
        self.last_s = s
        if s < self.delay:
            return None  # output still at the origin
        v = tuple(float(self.sim.query(self.path, a)[0][0]) for a in ("lat", "lon", "alt"))
        t = (s - self.delay) * self.period * self.dt
        if v == self.prev:  # fix_lost: the output holds
            self.held = True
            return None
        self.prev = v
        if self.held:
            self.held, self.ok_after = False, t + self.G["reconverge_s"]
        if t < self.ok_after:
            return None
        return t, enu(*v, self.G)


class Localizer:
    def __init__(self, cfg, x0, lever):
        self.O, self.Lc, self.G = cfg["odom"], cfg["loc"], cfg["gnss"]
        self.x = np.array(x0, dtype=np.float64)
        self.P = np.diag([self.Lc["init_sigma"] ** 2] * 3 + [self.Lc["init_yaw"] ** 2])
        self.lever = np.asarray(lever, dtype=np.float64)
        self.key_dt = 1.0 / cfg["lidar"]["rate_hz"]
        self.hist, self.last, self.t_fix = {}, None, -math.inf

    def predict(self, od):
        yaw_o = lidar.yaw_of(od.quat)
        q_rp = lidar.qmul(_qz(-yaw_o), od.quat)
        if self.last is not None:
            p0, y0 = self.last[0], self.last[1]
            dp = rz(self.x[3] - y0) @ (od.pos - p0)
            ds = math.hypot(dp[0], dp[1])
            Fm = np.eye(4)
            Fm[:3, 3] = _perp(dp)
            self.x[:3] += dp
            self.x[3] = wrap(self.x[3] + wrap(yaw_o - y0))
            Q = np.diag([self.O["drift_xy"] ** 2 * ds] * 2 + [self.O["drift_z"] ** 2 * ds, self.O["drift_yaw"] ** 2 * ds])
            self.P = Fm @ self.P @ Fm.T + Q
        self.last = (np.array(od.pos, dtype=np.float64), yaw_o, q_rp, od.t)
        k = int(round(od.t / self.key_dt))
        self.hist[k] = self.last
        for old in [j for j in self.hist if j < k - 50]:  # 5 s of history
            del self.hist[old]

    def pose(self):
        return self.x[:3].copy(), lidar.qmul(_qz(self.x[3]), self.last[2])

    @property
    def var_z(self):
        return float(self.P[2, 2])

    def update(self, z, h, H, R, chi2, wrap_last=False):
        nu = z - h
        if wrap_last:
            nu[-1] = wrap(nu[-1])
        S = H @ self.P @ H.T + R
        if float(nu @ np.linalg.solve(S, nu)) > chi2:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        self.x[3] = wrap(self.x[3])
        IKH = np.eye(4) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T  # Joseph form
        return True

    def update_rtk(self, t_meas, z):
        rec = self.hist.get(int(round(t_meas / self.key_dt)))
        if rec is None or self.last is None:
            return False
        p_tm, yaw_tm_o, q_rp_tm, _ = rec
        p_now, yaw_now_o = self.last[0], self.last[1]
        dp = rz(self.x[3] - yaw_now_o) @ (p_now - p_tm)  # odometry motion since the epoch, in the map frame
        yaw_tm = self.x[3] - wrap(yaw_now_o - yaw_tm_o)
        lev = lidar.qmat(lidar.qmul(_qz(yaw_tm), q_rp_tm)) @ self.lever
        h = self.x[:3] - dp + lev
        H = np.zeros((3, 4))
        H[:, :3] = np.eye(3)
        H[:, 3] = -_perp(dp) + _perp(lev)  # H[0,3] = dp_y - lev_y, H[1,3] = -dp_x + lev_x
        R = np.diag([self.G["horiz_std"] ** 2] * 2 + [self.G["vert_std"] ** 2])
        ok = self.update(np.asarray(z, float), h, H, R, self.Lc["chi2"])
        if ok:
            self.t_fix = max(self.t_fix, t_meas)
        return ok

    def update_icp(self, x_meas, cov):
        return self.update(np.asarray(x_meas, float), self.x.copy(), np.eye(4), cov, self.Lc["chi2_icp"], wrap_last=True)

    def rtk_ok(self, t_now):
        return t_now - self.t_fix < self.Lc["rtk_age"]
```

  `examples/dog/autonomy/icp.py`:

```python
"""Scan-to-submap point-to-plane ICP, 4 DOF (x, y, z, yaw; roll / pitch from odometry). Submap: the last `keep` keyframes
voxelised at `voxel` m, a mean and normal per planar voxel; association = the nearest voxel mean among the point's voxel and its
6 face neighbours (int64 keys, searchsorted: no KD-tree, no scipy). Gauss-Newton with Huber weights; directions whose Hessian
eigenvalue is below degen x the largest are not updated and get a huge covariance (a flat floor fixes only z)."""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import lidar
from .localize import _qz, rz, wrap

BIG = 1e4  # m^2 / rad^2: "no information" along a degenerate direction
NEIGH = np.array([[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], dtype=np.int64)
MAX_PTS = 2000  # scan points used per match


def levelled(scan, ext, odom_quat, ang_sigma=0.0, rng=None):
    """Scan points in the yaw-free base frame (base frame rotated by the odometry's roll / pitch); world = Rz(yaw) p + x."""
    q_rp = lidar.qmul(_qz(-lidar.yaw_of(odom_quat)), odom_quat)
    return lidar.points(scan, ext[0], ext[1], ang_sigma, rng) @ lidar.qmat(q_rp).T


def _keys(ijk):
    ijk = ijk + (1 << 20)
    return (ijk[..., 0] << 42) | (ijk[..., 1] << 21) | ijk[..., 2]


@dataclass
class Match:
    x: np.ndarray
    cov: np.ndarray
    inliers: int
    degenerate: np.ndarray


class ScanMatcher:
    def __init__(self, cfg):
        self.I = cfg["icp"]
        self.vox = self.I["voxel"]
        self.kf = deque(maxlen=int(self.I["keep"]))
        self.last_kf = None
        self.keys = np.empty(0, np.int64)

    def add(self, world_pts, x):
        if self.last_kf is not None:
            d = math.hypot(x[0] - self.last_kf[0], x[1] - self.last_kf[1])
            if d < self.I["kf_dist"] and abs(wrap(x[3] - self.last_kf[3])) < self.I["kf_yaw"]:
                return False
        self.kf.append(np.asarray(world_pts, dtype=np.float64))
        self.last_kf = np.array(x, dtype=np.float64)
        P = np.concatenate(self.kf)
        k = _keys(np.floor(P / self.vox).astype(np.int64))
        o = np.argsort(k, kind="stable")
        k, P = k[o], P[o]
        st = np.flatnonzero(np.r_[True, k[1:] != k[:-1]])
        cnt = np.diff(np.r_[st, len(k)]).astype(np.float64)
        mu = np.add.reduceat(P, st, axis=0) / cnt[:, None]
        C = np.add.reduceat(P[:, :, None] * P[:, None, :], st, axis=0) / cnt[:, None, None] - mu[:, :, None] * mu[:, None, :]
        w, V = np.linalg.eigh(C)  # ascending
        ok = (cnt >= self.I["min_pts"]) & (w[:, 0] < self.I["planarity"] * w[:, 1])
        self.keys, self.mu, self.nrm = k[st][ok], mu[ok], V[ok][:, :, 0]
        return True

    def _assoc(self, w):
        base = np.floor(w / self.vox).astype(np.int64)
        best_i = np.full(len(w), -1)
        best_d = np.full(len(w), np.inf)
        for off in NEIGH:
            k = _keys(base + off)
            i = np.minimum(np.searchsorted(self.keys, k), len(self.keys) - 1)
            hit = self.keys[i] == k
            d = np.where(hit, ((w - self.mu[i]) ** 2).sum(1), np.inf)
            better = d < best_d
            best_i, best_d = np.where(better, i, best_i), np.where(better, d, best_d)
        return best_i

    def match(self, s, x_guess):
        if len(self.keys) == 0:
            return None
        s = s[:: max(1, len(s) // MAX_PTS)]
        x = np.array(x_guess, dtype=np.float64)
        hub, sig2 = self.I["huber"], self.I["sigma"] ** 2
        for _ in range(int(self.I["iters"])):
            ws = s @ rz(x[3]).T
            i = self._assoc(ws + x[:3])
            hit = i >= 0
            if hit.sum() < 50:
                return None
            n, mu, wsh = self.nrm[i[hit]], self.mu[i[hit]], ws[hit]
            e = ((wsh + x[:3] - mu) * n).sum(1)
            J = np.column_stack([n, -n[:, 0] * wsh[:, 1] + n[:, 1] * wsh[:, 0]])
            a = np.abs(e)
            wt = np.where(a <= hub, 1.0, hub / np.maximum(a, 1e-12))
            H = (J * wt[:, None]).T @ J
            g = (J * wt[:, None]).T @ e
            lam, U = np.linalg.eigh(H)
            good = lam > self.I["degen"] * lam.max()
            dx = -U[:, good] @ ((U[:, good].T @ g) / lam[good])
            x[:4] += dx
            x[3] = wrap(x[3])
            if np.linalg.norm(dx) < 1e-5:
                break
        cov = U[:, good] @ np.diag(sig2 / lam[good]) @ U[:, good].T + U[:, ~good] @ np.diag(np.full((~good).sum(), BIG)) @ U[:, ~good].T
        return Match(x, cov, int(hit.sum()), np.diag(cov) > 1.0)
```

- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_localize.py -n 8'`.
  If the ruins test's odometry-only error at seed 3 is below 0.3 m (the comparison is then not informative), raise
  `odom.drift_yaw` in the test override to 0.04, never loosen the 0.15 m bound.
- [ ] **Step 5: commit** `git add examples/dog/autonomy/localize.py examples/dog/autonomy/icp.py tests/python/test_dog_autonomy_localize.py && git commit -m "dog autonomy: 4-DOF EKF (odometry prediction, delayed RTK with lever arm at the fix epoch, chi2 gate, Joseph), GNSS reader (held output = outage, reconvergence), voxel point-to-plane ICP with degeneracy handling" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 8: coverage planner (`planner.py`: A*, boustrophedon lanes, gap / frontier goals, pure pursuit)

**Files:**
- Create: `examples/dog/autonomy/planner.py`
- Test: `tests/python/test_dog_autonomy_planner.py`

**Interfaces:**
- Consumes: the block cost grid of `traverse.Traversability.cost` (`inf` = lethal), `ElevationMap.mapped(pool)`,
  `config.load()["plan"]`, `["trav"]`, `["map"]`.
- Produces: `planner.MOVES` (= `traverse.MOVES`), `planner.astar(cost, start, goal) -> list[(i, j)] | None` (8-connected, move
  cost = destination cost x step length, no corner cutting, octile heuristic), `planner.dijkstra(cost, start) -> (dist, parent)`
  (`parent` = flat index, -1 = none), `planner.path_to(parent, start, goal) -> list[(i, j)]`, `planner.block_of(x, y, cfg)`,
  `planner.centre(ij, cfg) -> (x, y)`, `planner.lanes(cfg, lane_m, start_xy) -> list[(x, y)]` (boustrophedon along x, from the
  corner nearest `start_xy`), `planner.gain(mapped, cfg, pool) -> float (nb, nb)` (unmapped fraction of each block's scoring cells),
  `planner.choose(dist, gain, tried, cfg) -> (i, j) | None` (nearest block with `gain >= min_gain`, not tried, finite distance;
  ties -> larger gain), `planner.view_goal(path, cfg) -> (i, j)` (the block `view_back` m before the end of the path),
  `planner.pursue(pose_xy_yaw, path_xy, cfg, lim) -> (vx, wz)`.

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_planner.py`:

```python
import math

import numpy as np
import pytest

from autonomy import config, planner

CFG = config.load()


def bellman_ford(cost, start):
    n0, n1 = cost.shape
    d = np.full(cost.shape, np.inf)
    d[start] = 0.0
    for _ in range(n0 * n1):
        changed = False
        for i in range(n0):
            for j in range(n1):
                if not np.isfinite(cost[i, j]):
                    continue
                for di, dj in planner.MOVES:
                    a, b = i - di, j - dj  # predecessor
                    if not (0 <= a < n0 and 0 <= b < n1) or not np.isfinite(d[a, b]):
                        continue
                    if di and dj and not (np.isfinite(cost[a + di, b]) and np.isfinite(cost[a, b + dj])):
                        continue
                    c = d[a, b] + cost[i, j] * math.hypot(di, dj)
                    if c < d[i, j] - 1e-12:
                        d[i, j], changed = c, True
        if not changed:
            return d
    return d


def test_astar_and_dijkstra_match_a_reference():
    rng = np.random.default_rng(0)
    cost = rng.choice([1.0, 2.0, 3.0, np.inf], size=(14, 14), p=[0.55, 0.2, 0.1, 0.15])
    cost[0, 0] = cost[13, 13] = 1.0
    ref = bellman_ford(cost, (0, 0))
    dist, parent = planner.dijkstra(cost, (0, 0))
    np.testing.assert_allclose(dist, ref)
    for goal in [(13, 13), (7, 3), (2, 12)]:
        p = planner.astar(cost, (0, 0), goal)
        if not np.isfinite(ref[goal]):
            assert p is None
            continue
        c = sum(cost[b] * math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))
        assert c == pytest.approx(ref[goal]) and p[0] == (0, 0) and p[-1] == goal
        assert planner.path_to(parent, (0, 0), goal)[-1] == goal


def test_door_and_no_corner_cutting():
    cost = np.ones((9, 9))
    cost[4, :] = np.inf
    cost[4, 6] = 1.0  # a one-block door
    p = planner.astar(cost, (0, 0), (8, 0))
    assert (4, 6) in p
    q = np.ones((3, 3))
    q[0, 1] = q[1, 0] = np.inf  # (0, 0) touches (1, 1) only at a corner
    q[1, 2] = q[2, 1] = np.inf
    assert planner.astar(q, (0, 0), (2, 2)) is None


def test_lanes_cover_the_field_from_the_nearest_corner():
    size, lane = CFG["map"]["size"], 8.0
    w = planner.lanes(CFG, lane, (40.0, -45.0))
    assert math.hypot(w[0][0] - 46.0, w[0][1] + 46.0) < 1.0  # starts at the south-east corner
    ys = sorted({round(p[1], 6) for p in w})
    assert ys[0] - (-size / 2) <= lane / 2 + 1e-9 and size / 2 - ys[-1] <= lane / 2 + 1e-9
    assert all(b - a <= lane + 1e-9 for a, b in zip(ys, ys[1:]))
    for a, b in zip(w, w[1:]):
        assert a[0] == b[0] or a[1] == b[1]  # axis-aligned legs


def test_gain_choose_and_view_goal():
    cfg = config.override(CFG, **{"map.size": 10.0})
    mapped = np.ones((100, 100), bool)  # pool 2 on a 200-cell map
    mapped[60:65, 80:85] = False  # one block (12, 16) fully unmapped: 5 x 5 pooled cells
    mapped[0:5, 0:5] = False  # one block (0, 0) fully unmapped
    mapped[20:22, 20:25] = False  # block (4, 4): 10 of 25 unmapped = 0.4
    mapped[40:41, 40:45] = False  # block (8, 8): 5 of 25 = 0.2, below min_gain
    g = planner.gain(mapped, cfg, 2)
    assert g[12, 16] == 1.0 and g[0, 0] == 1.0 and g[4, 4] == pytest.approx(0.4) and g.sum() == pytest.approx(2.6)
    cost = np.ones((20, 20))
    dist, _ = planner.dijkstra(cost, (10, 10))
    assert planner.choose(dist, g, set(), cfg) == (12, 16)
    assert planner.choose(dist, g, {(12, 16)}, cfg) == (4, 4)  # (8, 8) is nearer but below min_gain
    assert planner.choose(dist, g, {(12, 16), (4, 4)}, cfg) == (0, 0)
    path = [(10, j) for j in range(0, 20)]
    assert planner.view_goal(path, cfg) == (10, 14)  # 2.5 m = 5 blocks before (10, 19)


def test_pure_pursuit():
    P = CFG["plan"]
    lim = {"v_max": 1.5, "wz_max": 1.0}
    ahead = [(0.5 * k, 0.0) for k in range(10)]
    assert planner.pursue((0.0, 0.0, 0.0), ahead, CFG, lim) == pytest.approx((1.5, 0.0))
    left = [(0.0, 0.5 * k) for k in range(10)]
    assert planner.pursue((0.0, 0.0, 0.0), left, CFG, lim) == pytest.approx((0.0, 1.0))
    a = math.radians(30)
    diag = [(0.25 * k * math.cos(a), 0.25 * k * math.sin(a)) for k in range(20)]
    vx, wz = planner.pursue((0.0, 0.0, 0.0), diag, CFG, lim)
    assert vx == pytest.approx(1.5 * math.cos(a)) and wz == pytest.approx(min(2 * vx * math.sin(a) / P["lookahead"], 1.0))
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_planner.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/planner.py`:**

```python
"""Coverage planning on 0.5 m blocks: boustrophedon sweep lanes, then the nearest unmapped gap / frontier (Dijkstra once per
new goal, A* at 1-2 Hz to the current goal), pure pursuit to follow the path. Python heapq: ~40 000 blocks."""

import heapq
import math

import numpy as np

from .localize import wrap
from .traverse import MOVES

SQ2 = math.sqrt(2.0)


def _legal(cost, i, j, di, dj):
    a, b = i + di, j + dj
    if not (0 <= a < cost.shape[0] and 0 <= b < cost.shape[1]) or not math.isfinite(cost[a, b]):
        return False
    return not (di and dj) or (math.isfinite(cost[i + di, j]) and math.isfinite(cost[i, j + dj]))


def _search(cost, start, goal):
    n1 = cost.shape[1]
    dist = np.full(cost.shape, np.inf)
    parent = np.full(cost.size, -1, dtype=np.int64)
    if not math.isfinite(cost[start]):
        return dist, parent
    dist[start] = 0.0
    h = (lambda i, j: max(abs(i - goal[0]), abs(j - goal[1])) + (SQ2 - 1) * min(abs(i - goal[0]), abs(j - goal[1]))) if goal else (lambda i, j: 0.0)
    pq = [(h(*start), 0.0, start)]
    done = np.zeros(cost.shape, bool)
    c = cost.tolist()
    while pq:
        _, d, (i, j) = heapq.heappop(pq)
        if done[i, j]:
            continue
        done[i, j] = True
        if goal is not None and (i, j) == goal:
            break
        for di, dj in MOVES:
            if not _legal(cost, i, j, di, dj):
                continue
            a, b = i + di, j + dj
            nd = d + c[a][b] * (SQ2 if di and dj else 1.0)
            if nd < dist[a, b]:
                dist[a, b] = nd
                parent[a * n1 + b] = i * n1 + j
                heapq.heappush(pq, (nd + h(a, b), nd, (a, b)))
    return dist, parent


def dijkstra(cost, start):
    return _search(cost, start, None)


def path_to(parent, start, goal, n1=None):
    n1 = n1 or int(math.isqrt(parent.size))
    out, k, s = [], goal[0] * n1 + goal[1], start[0] * n1 + start[1]
    while k != -1:
        out.append(divmod(int(k), n1))
        if k == s:
            return out[::-1]
        k = parent[k]
    return None


def astar(cost, start, goal):
    dist, parent = _search(cost, start, goal)
    if not math.isfinite(dist[goal]):
        return None
    return path_to(parent, start, goal, cost.shape[1])


def block_of(x, y, cfg):
    b, h = cfg["trav"]["block"], cfg["map"]["size"] / 2
    nb = int(round(cfg["map"]["size"] / b))
    return min(max(int((y + h) // b), 0), nb - 1), min(max(int((x + h) // b), 0), nb - 1)


def centre(ij, cfg):
    b, h = cfg["trav"]["block"], cfg["map"]["size"] / 2
    return -h + (ij[1] + 0.5) * b, -h + (ij[0] + 0.5) * b


def lanes(cfg, lane_m, start_xy):
    h = cfg["map"]["size"] / 2
    inset = cfg["trav"]["block"] * 8  # 4 m from the field edge: the lidar maps the border strip from inside
    n = math.ceil((2 * h) / lane_m)
    ys = [-h + lane_m / 2 + k * (2 * h - lane_m) / max(n - 1, 1) for k in range(n)]
    if start_xy[1] > 0:
        ys = ys[::-1]
    xs = (h - inset, -h + inset) if start_xy[0] > 0 else (-h + inset, h - inset)
    pts = []
    for k, y in enumerate(ys):
        a, b = xs if k % 2 == 0 else xs[::-1]
        pts += [(a, y), (b, y)]
    return pts


def gain(mapped, cfg, pool):
    k = int(round(cfg["trav"]["block"] / (cfg["map"]["res"] * pool)))  # pooled cells per block side
    nb = mapped.shape[0] // k
    un = ~mapped[: nb * k, : nb * k]
    return un.reshape(nb, k, nb, k).sum(axis=(1, 3)) / (k * k)


def choose(dist, g, tried, cfg):
    cand = np.isfinite(dist) & (g >= cfg["plan"]["min_gain"])
    for ij in tried:
        cand[ij] = False
    if not cand.any():
        return None
    key = np.where(cand, dist - 1e-6 * g, np.inf)
    return tuple(int(v) for v in np.unravel_index(int(np.argmin(key)), key.shape))


def view_goal(path, cfg):
    back = cfg["plan"]["view_back"] / cfg["trav"]["block"]
    acc = 0.0
    for k in range(len(path) - 1, 0, -1):
        a, b = path[k], path[k - 1]
        acc += math.hypot(a[0] - b[0], a[1] - b[1])
        if acc >= back - 1e-9:
            return b
    return path[0]


def pursue(pose, path_xy, cfg, lim):
    P = cfg["plan"]
    x, y, yaw = pose
    pts = np.asarray(path_xy, float)
    d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
    k0 = int(np.argmin(d))
    far = np.flatnonzero(d[k0:] >= P["lookahead"])
    tx, ty = pts[k0 + far[0]] if len(far) else pts[-1]
    a = wrap(math.atan2(ty - y, tx - x) - yaw)
    if abs(a) > P["turn_in_place"]:
        return 0.0, math.copysign(lim["wz_max"], a)
    v = lim["v_max"] * math.cos(a)
    wz = 2.0 * v * math.sin(a) / P["lookahead"]
    return v, max(-lim["wz_max"], min(lim["wz_max"], wz))
```

  `lanes`: `n` lanes spread evenly so the outer lanes sit `lane_m / 2` from the edges and neighbours are at most `lane_m` apart.
  The inset of 4 m along x is a design number taken from the block size (8 blocks); the field's x-edges are mapped by the
  lidar's forward reach at the lane ends and by the fill phase.
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_planner.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/autonomy/planner.py tests/python/test_dog_autonomy_planner.py && git commit -m "dog autonomy: coverage planner (corner-safe 8-connected A* / Dijkstra on blocks, boustrophedon lanes, nearest-gap choice with view point, pure pursuit)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 9: mission executive + the 10 Hz robot loop (`mission.py`), scenario harness (`scene.py`, `automission.py`), short scenarios with goldens

**Files:**
- Create: `examples/dog/autonomy/mission.py`, `examples/dog/autonomy/scene.py`, `examples/dog/automission.py`,
  `examples/dog/missions/map_quality/map_quality.scenario.toml`, `examples/dog/missions/map_quality/map_quality.py`,
  `examples/dog/missions/coverage_smoke/coverage_smoke.scenario.toml`, `examples/dog/missions/coverage_smoke/coverage_smoke.py`
- Modify: `docs/dog/autonomy.md` (measured `c_py` per module; the wall-time table recomputed)
- Test: `tests/python/test_dog_autonomy_mission.py`

**Interfaces:**
- Consumes: everything above; `napl.params.values`, `napl.marine.write_native`, `napl.program.run`.
- Produces: `mission.Executive(cfg, home_xy)` (`INIT, SWEEP, FILL, RETURN, DONE, ABORT = 0..5`, `.phase`, `.goal`,
  `.update(t, pose (x, y, yaw), cost, mapped, fallen) -> list[(x, y)]` (empty = stand still)); `mission.Robot(loco, cfg, rays,
  ext, x0, lever, seed)` (`.frame(t)` once per lidar frame after `loco.advance`; attributes `loc`, `map`, `trav`, `ex`, `ms` (dict
  of accumulated milliseconds: `points`, `icp`, `fuse`, `trav`, `plan`)); `scene.scenario(path) -> (auto, params, sc)`,
  `scene.cfg_for(auto) -> cfg`, `scene.make_field(auto, seed)`, `scene.build(out, path) -> Path`, `scene.build_for(out,
  script_file)`, `scene.COLS`, `scene.run(model, scenario_path, out, t_max=None) -> dict` (trace.csv, replay.naplog, native.json,
  summary.json); `automission.py <scenario> [<out dir>]`; scenario `[auto]` keys (numbers-rule leaves): `kind` (str), `size`,
  `patch`, `rays`, `pool`, `t_budget`, `start_x`, `start_y`, `start_yaw`, optional `lane_m`, `return_margin`.
  trace columns (`scene.COLS`): `t, x, y, x_est, y_est, err_xy, err_z, err_yaw, phase, coverage, rms, rms_edges, rtk_ok, gnss_lost,
  bumps, distance, fallen, ms_points, ms_icp, ms_fuse, ms_trav, ms_plan, ms_metrics`.

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_mission.py`:

```python
import math
import tomllib

import numpy as np
import pytest

import napl
from autonomy import config, mission, planner
from napl import program

EX = ROOT / "examples" / "dog" / "missions"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
# scenario -> golden state hash (default seed); recorded in Step 5
GOLDEN = {"map_quality/map_quality": None, "coverage_smoke/coverage_smoke": None}
CFG = config.override(config.load(), **{"map.size": 20.0, "plan.t_budget": 100.0, "plan.return_margin": 5.0})


def test_executive_sweeps_skips_fills_returns():
    ex = mission.Executive(CFG, (-8.0, -8.0))
    cost = np.ones((40, 40))
    mapped = np.zeros((200, 200), bool)  # pool 2 on 400 cells
    assert ex.update(0.5, (-8.0, -8.0, 0.0), cost, mapped, False) == [] and ex.phase == ex.INIT
    p = ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.SWEEP and p[-1] == pytest.approx((-6.0, -6.0))
    p = ex.update(2.0, (-6.0, -6.0, 0.0), cost, mapped, False)  # reached: next lane end
    assert p[-1] == pytest.approx((6.0, -6.0))
    ex.update(3.0, (0.0, -6.0, 0.0), cost, mapped, False)
    p = ex.update(3.0 + CFG["plan"]["stuck_s"] + 0.1, (0.0, -6.0, 0.0), cost, mapped, False)  # no progress: skip
    assert p[-1] == pytest.approx((6.0, 0.0))
    for wp in planner.lanes(CFG, CFG["plan"]["lane_m"], (-8.0, -8.0))[2:]:
        ex.update(30.0, (*wp, 0.0), cost, mapped, False)
    assert ex.phase == ex.FILL  # lanes done; with nothing mapped the first gap is the robot's own block
    mapped[:] = True
    mapped[25:30, 25:30] = False  # one gap block (5, 5), far from the robot
    p = ex.update(31.0, (6.0, 6.0, 0.0), cost, mapped, False)  # own block reached -> tried; next gap, viewed 2.5 m short of it
    assert ex.goal_block == (5, 5) and math.dist(p[-1], planner.centre((5, 5), CFG)) == pytest.approx(2.5, abs=0.75)
    ex.update(32.0, (*p[-1], 0.0), cost, mapped, False)  # reached the view point: tried; no gap left -> home
    assert ex.phase == ex.RETURN
    ex.update(33.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.DONE


def test_return_trigger_and_abort():
    ex = mission.Executive(CFG, (-8.0, -8.0))
    cost, mapped = np.ones((40, 40)), np.zeros((200, 200), bool)
    ex.update(1.0, (-8.0, -8.0, 0.0), cost, mapped, False)
    d = math.dist((8.0, 8.0), (-8.0, -8.0))
    t_ret = 100.0 - CFG["plan"]["detour"] * d / CFG["carrier"]["v_max"] - 5.0
    ex.update(t_ret - 0.5, (8.0, 8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.SWEEP
    p = ex.update(t_ret + 0.5, (8.0, 8.0, 0.0), cost, mapped, False)
    assert ex.phase == ex.RETURN and p[-1] == pytest.approx((-8.0, -8.0))
    assert ex.update(t_ret + 1.0, (8.0, 8.0, 0.0), cost, mapped, True) == [] and ex.phase == ex.ABORT
    assert ex.update(t_ret + 2.0, (8.0, 8.0, 0.0), cost, mapped, False) == []


def test_every_mission_is_listed_with_its_monitors():
    found = sorted(str(p.relative_to(EX)).removesuffix(".scenario.toml") for p in EX.rglob("*.scenario.toml"))
    assert set(GOLDEN) <= set(found)
    for name in found:
        sc = tomllib.loads((EX / f"{name}.scenario.toml").read_text())
        assert {"map_rms_within_5cm", "localized_within_30cm"} <= {m["name"] for m in sc["monitors"]}
        config.leaves(sc["auto"], "auto")  # numbers rule


@needs_kernel
@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_mission_passes_replays_and_matches_its_golden_hash(tmp_path, name):
    res, _ = program.run(EX / f"{name}.scenario.toml", tmp_path / "runs")
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["pass"], res["fail_reasons"]
    assert GOLDEN[name] is None or res["sim"]["state_hash"] == GOLDEN[name], res["sim"]["state_hash"]
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_mission.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/mission.py`:**

```python
"""Mission executive and the robot's 10 Hz loop: localize -> fuse the scan -> traversability -> (1 Hz) plan -> pure pursuit.
Robot side only: it sees the Locomotion door, device attributes and its own estimates, never ground truth."""

import math
import time

import numpy as np

from . import elevmap, icp, lidar, localize, planner, traverse


class Executive:
    INIT, SWEEP, FILL, RETURN, DONE, ABORT = range(6)

    def __init__(self, cfg, home_xy):
        self.cfg, self.P, self.T = cfg, cfg["plan"], cfg["trav"]
        self.home = tuple(float(v) for v in home_xy)
        self.wps = planner.lanes(cfg, self.P["lane_m"], self.home)
        self.wi, self.phase, self.goal, self.goal_block = 0, self.INIT, None, None
        self.tried, self.best = set(), (math.inf, 0.0)

    def _set(self, goal, t, block=None):
        self.goal, self.goal_block, self.best = goal, block, (math.inf, t)

    def update(self, t, pose, cost, mapped, fallen):
        P = self.P
        if fallen:
            self.phase = self.ABORT
        if self.phase in (self.DONE, self.ABORT):
            return []
        if self.phase == self.INIT:
            if t < P["init_s"]:
                return []
            self.phase = self.SWEEP
        xy = (pose[0], pose[1])
        v = self.cfg["carrier"]["v_max"]
        if self.phase != self.RETURN and t >= P["t_budget"] - P["detour"] * math.dist(xy, self.home) / v - P["return_margin"]:
            self.phase = self.RETURN
            self._set(self.home, t)
        skip = False
        if self.goal is not None:
            d = math.dist(xy, self.goal)
            if d < self.best[0] - 0.5:
                self.best = (d, t)
            elif t - self.best[1] > P["stuck_s"]:
                skip = True
        reached = self.goal is not None and math.dist(xy, self.goal) < P["goal_tol"]
        if self.phase == self.SWEEP and (self.goal is None or reached or skip):
            if self.goal is not None:
                self.wi += 1
            if self.wi < len(self.wps):
                self._set(self.wps[self.wi], t)
            else:
                self.phase = self.FILL
                self.goal = None
        me = planner.block_of(xy[0], xy[1], self.cfg)
        c = cost.copy()
        c[me] = min(c[me], self.T["unknown_cost"] + self.T["near_lethal"])  # never plan from a blocked start
        if self.phase == self.FILL and (self.goal is None or reached or skip):
            if self.goal_block is not None:
                self.tried.add(self.goal_block)
            dist, parent = planner.dijkstra(c, me)
            ij = planner.choose(dist, planner.gain(mapped, self.cfg, int(P["pool"])), self.tried, self.cfg)
            if ij is None:
                self.phase = self.RETURN
                self._set(self.home, t)
            else:
                vb = planner.view_goal(planner.path_to(parent, me, ij, c.shape[1]), self.cfg)
                self._set(planner.centre(vb, self.cfg), t, ij)
        if self.phase == self.RETURN and math.dist(xy, self.home) < P["goal_tol"]:
            self.phase = self.DONE
            return []
        path = planner.astar(c, me, planner.block_of(*self.goal, self.cfg))
        if path is None:
            self.best = (math.inf, -math.inf)  # unreachable: skipped on the next update
            return []
        return [planner.centre(b, self.cfg) for b in path[:-1]] + [self.goal]


class Robot:
    def __init__(self, loco, cfg, rays, ext, x0, lever, seed):
        self.loco, self.cfg, self.ext = loco, cfg, ext
        L, I, P = cfg["lidar"], cfg["icp"], cfg["plan"]
        self.ros = lidar.Rosette(loco.sim, loco.lidar, rays, loco.dt, cfg)
        self.gnss = localize.GnssReader(loco.sim, loco.gnss, cfg, loco.dt)
        self.loc = localize.Localizer(cfg, x0, lever)
        self.sm = icp.ScanMatcher(cfg)
        self.map = elevmap.ElevationMap(cfg)
        self.trav = traverse.Traversability(cfg, self.map.n)
        self.ex = Executive(cfg, x0[:2])
        self.rng = np.random.default_rng([seed, 0x11DA])
        self.icp_every = {True: int(round(L["rate_hz"] / I["rate_good"])), False: int(round(L["rate_hz"] / I["rate_bad"]))}
        self.plan_every = int(round(L["rate_hz"] / P["rate_hz"]))
        self.k, self.path = 0, []
        self.ms = dict.fromkeys(("points", "icp", "fuse", "trav", "plan"), 0.0)

    def _tick(self, key, t0):
        t1 = time.perf_counter()
        self.ms[key] += 1e3 * (t1 - t0)
        return t1

    def frame(self, t):
        cfg, t0 = self.cfg, time.perf_counter()
        od = self.loco.odometry()
        self.loc.predict(od)
        fix = self.gnss.poll()
        if fix is not None:
            self.loc.update_rtk(*fix)
        scan = self.ros.latest()
        q_rp = lidar.qmul(localize._qz(-lidar.yaw_of(od.quat)), od.quat)
        pts = icp.levelled(scan, self.ext, od.quat, cfg["lidar"]["ang_sigma"], self.rng)
        t0 = self._tick("points", t0)
        if self.k and self.k % self.icp_every[self.loc.rtk_ok(od.t)] == 0:
            m = self.sm.match(pts, self.loc.x.copy())
            if m is not None:
                self.loc.update_icp(m.x, m.cov)
        R = localize.rz(self.loc.x[3])
        world = pts @ R.T + self.loc.x[:3]
        self.sm.add(world, self.loc.x)
        t0 = self._tick("icp", t0)
        origin = R @ (lidar.qmat(q_rp) @ self.ext[0]) + self.loc.x[:3]
        self.map.fuse(world, origin, self.loc.var_z, cfg["odom"]["att_sigma"])
        t0 = self._tick("fuse", t0)
        if self.map.dirty is not None:
            self.trav.update(self.map.hn, self.map.dirty)
        t0 = self._tick("trav", t0)
        pose = (self.loc.x[0], self.loc.x[1], self.loc.x[3])
        if self.k % self.plan_every == 0:
            self.path = self.ex.update(t, pose, self.trav.cost, self.map.mapped(int(cfg["plan"]["pool"])), od.fallen)
        lim = self.loco.limits()
        vx, wz = planner.pursue(pose, self.path, cfg, lim) if self.path else (0.0, 0.0)
        self.loco.command(vx, 0.0, wz)
        self.loco.fix(self.loc.x[:2].copy(), float(self.loc.x[3]), od.t)  # SP5 -> SP4 pose fix, 10 Hz
        self._tick("plan", t0)
        self.k += 1
```

  `examples/dog/autonomy/scene.py` (harness):

```python
"""Scenario harness of the dog autonomy missions: builds the carrier scene from a `.scenario.toml`, runs the robot on it and
scores it against ground truth (trace.csv for the napl program monitors, summary.json)."""

import json
import time
import tomllib
from pathlib import Path

import numpy as np

import napl
from napl import params as nparams
from napl.marine import write_native
from napl.terrain import field

from . import carrier, config, lidar, metrics, mission

COLS = ["t", "x", "y", "x_est", "y_est", "err_xy", "err_z", "err_yaw", "phase", "coverage", "rms", "rms_edges", "rtk_ok",
        "gnss_lost", "bumps", "distance", "fallen", "ms_points", "ms_icp", "ms_fuse", "ms_trav", "ms_plan", "ms_metrics"]  # fmt: skip
METRIC_EVERY_S = 10.0  # scoring cadence (harness cost, not the robot's)
DENSITY = {"rubble": {"rocks": 2500, "boulders": 40}, "scree": {"rocks": 6000}}  # napl.terrain.field defaults per 100 x 100 m


def scenario(path):
    sc = tomllib.loads(Path(path).read_text())
    return config.leaves(sc["auto"], "auto"), nparams.values(path), sc


def cfg_for(auto):
    ov = {"map.size": auto["size"], "plan.t_budget": auto["t_budget"], "plan.pool": auto["pool"]}
    for k in ("lane_m", "return_margin"):  # optional per-scenario overrides of [plan]
        if k in auto:
            ov[f"plan.{k}"] = auto[k]
    return config.override(config.load(), **ov)


def make_field(auto, seed):
    a = (auto["size"] / 100.0) ** 2
    kw = {k: max(1, round(v * a)) for k, v in DENSITY.get(auto["kind"], {}).items()}
    return field(auto["kind"], size=auto["size"], patch=auto["patch"], seed=int(seed), **kw)


def _start(auto):
    return (auto["start_x"], auto["start_y"], auto["start_yaw"])


def build(out, path):
    auto, prm, _ = scenario(path)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    return carrier.build(make_field(auto, prm["seed"]), cfg_for(auto), int(auto["rays"]), _start(auto), out)


def build_for(out, script_file):
    return build(out, Path(script_file).with_suffix("").with_suffix(".scenario.toml"))


def _loco(sim, model, auto, cfg, f, seed, start):
    """-> (locomotion, lidar extrinsic, GNSS lever arm, cfg). Task 11 adds body = "dog"."""
    car = carrier.Carrier(sim, f, cfg, start, seed=seed)
    ext = lidar.extrinsic(Path(model).read_text(), "lidar", "carrier")
    return car, ext, np.array([cfg["gnss"]["lever_x"], 0.0, cfg["gnss"]["lever_z"]]), cfg


def run(model, scenario_path, out, t_max=None):
    auto, prm, _ = scenario(scenario_path)
    cfg, seed, start = cfg_for(auto), int(prm["seed"]), _start(auto)
    f = make_field(auto, seed)
    sim = napl.Sim.load(model)
    sim.start_log()
    car, ext, lever, cfg = _loco(sim, model, auto, cfg, f, seed, start)
    p0, q0 = car.truth()
    L = cfg["loc"]
    g = np.random.default_rng([seed, 0x5EED]).standard_normal(4)  # operator initialisation error (decision 3)
    x0 = np.r_[p0 + L["init_sigma"] * g[:3], lidar.yaw_of(q0) + L["init_yaw"] * g[3]]
    rob = mission.Robot(car, cfg, int(auto["rays"]), ext, x0, lever, seed)
    gt = metrics.truth_grid(f, cfg["map"]["res"])
    edges, tgt = metrics.edge_mask(gt, cfg), metrics.targets(gt, cfg, start[:2])
    pool = int(cfg["plan"]["pool"])
    fdt = 1.0 / cfg["lidar"]["rate_hz"]
    t_max = t_max or cfg["plan"]["t_budget"] + cfg["plan"]["return_margin"]
    every = int(round(METRIC_EVERY_S / fdt))
    rows, cov, r, ms_m = [], float("nan"), {"rms": float("nan"), "rms_edges": float("nan")}, 0.0

    def score():
        t0 = time.perf_counter()
        c, rr = metrics.coverage(rob.map, tgt, pool), metrics.height_rms(rob.map, gt, edges)
        return c, rr, 1e3 * (time.perf_counter() - t0)

    for k in range(int(round(t_max / fdt))):
        car.advance(fdt)
        rob.frame(car.time_of_last_frame)
        done = rob.ex.phase in (rob.ex.DONE, rob.ex.ABORT)
        if k % every == 0 or done or k == int(round(t_max / fdt)) - 1:
            cov, r, dm = score()
            ms_m += dm
        p, q = car.truth()
        x = rob.loc.x
        rows.append([
            car.time_of_last_frame, p[0], p[1], x[0], x[1], float(np.hypot(x[0] - p[0], x[1] - p[1])), x[2] - p[2],
            abs(((x[3] - lidar.yaw_of(q)) + np.pi) % (2 * np.pi) - np.pi), rob.ex.phase, cov, r["rms"], r["rms_edges"],
            int(rob.loc.rtk_ok(car.time_of_last_frame)), int(car.gnss_lost), car.bumps, car.distance, int(getattr(car, "fallen", False)),
            rob.ms["points"], rob.ms["icp"], rob.ms["fuse"], rob.ms["trav"], rob.ms["plan"], ms_m,
        ])  # fmt: skip
        if done:
            break
    write_native(sim, car.dt, COLS, rows, out)
    last = dict(zip(COLS, rows[-1]))
    nfr = len(rows)
    summ = {**last, "frames": nfr, "ms_per_frame": {k: rob.ms[k] / nfr for k in rob.ms}, "metrics_ms_per_frame": ms_m / nfr}
    (Path(out) / "summary.json").write_text(json.dumps(summ, indent=1))
    return summ
```

  `examples/dog/automission.py`:

```python
"""Runner of the dog autonomy missions (examples/dog/missions/*/*.scenario.toml).

    napl program examples/dog/missions/coverage_smoke/coverage_smoke.scenario.toml --runs-dir <dir>
    python examples/dog/automission.py <scenario> <out dir>            # standalone; NAPL_PARAMS='{"seed": 7}' picks the seed
AUTO_T_MAX (s) caps the run (tests)."""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from autonomy import scene  # noqa: E402


def main():
    path = Path(sys.argv[1]).resolve()
    if "NAPL_MODEL" in os.environ:  # under `napl program`
        model, out = Path(os.environ["NAPL_MODEL"]), Path(os.environ["NAPL_RUN_DIR"])
    else:
        out = Path(sys.argv[2])
        out.mkdir(parents=True, exist_ok=True)
        model = scene.build(out / "model", path)
    t = os.environ.get("AUTO_T_MAX")
    s = scene.run(model, path, out, float(t) if t else None)
    sys.stdout.write(json.dumps(s) + "\n")


if __name__ == "__main__":
    main()
```

  Model scripts (`map_quality.py`; `coverage_smoke.py` identical but for its docstring):

```python
"""Model script of map_quality.scenario.toml: the carrier scene is built by autonomy/scene.py."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from autonomy import scene  # noqa: E402


def build(out):
    return scene.build_for(out, __file__)
```

  `examples/dog/missions/map_quality/map_quality.scenario.toml`:

```toml
# napl program examples/dog/missions/map_quality/map_quality.scenario.toml --runs-dir <dir>
# Mapping quality at the real point rate (20k rays / frame): the kinematic carrier sweeps a 20 x 20 m rubble field for 45 s with
# RTK + odometry + ICP and the 5 cm elevation map; the map's height RMS against ground truth (edge cells excluded) is the spec's 5 cm.
name = "dog-auto-map-quality"

[auto]
kind = "rubble"
size = { value = 20.0, unit = "m", fidelity = "design", rationale = "Short run: a 20 m field (100 x 100 m densities scaled) keeps 20k rays at a few minutes wall." }
patch = { value = 10.0, unit = "m", fidelity = "design", rationale = "Terrain patch size (Task 4 decision)." }
rays = { value = 20000, unit = "1", fidelity = "datasheet", rationale = "Mid-360 point rate / frame rate." }
pool = { value = 1, unit = "1", fidelity = "design", rationale = "Full density: score 5 cm cells." }
t_budget = { value = 45.0, unit = "s", fidelity = "design", rationale = "Long enough for two lanes and a turn on 20 m." }
return_margin = { value = 0.0, unit = "s", fidelity = "design", rationale = "A mapping run, not a mission: no reserve for the way home." }
start_x = { value = -8.0, unit = "m", fidelity = "design", rationale = "South-west corner, 2 m inside." }
start_y = { value = -8.0, unit = "m", fidelity = "design", rationale = "South-west corner, 2 m inside." }
start_yaw = { value = 0.0, unit = "rad", fidelity = "design", rationale = "Facing east, along the first lane." }

[model]
script = "map_quality.py"

[interfaces.native]

[program]
cmd = ["python", "{dir}/../../automission.py", "{dir}/map_quality.scenario.toml"]

[[parameters]]
name = "seed"
type = "int"
default = 1
min = 1
max = 1000000
group = "run"
label = "Seed"
doc = "Seeds the terrain, the lidar / GNSS noise, the odometry drift and the initial pose error: one seed, one run."

[limits]
wall_s = 1200            # 45 + 60 sim-s at 20k rays on rubble ~ 6 min (Task 4 budget) x 3
cpu_s = 1200
address_space_mb = 16384

[[monitors]]
name = "map_rms_within_5cm"   # spec section 7
column = "rms"
op = "final"
hi = 0.05

[[monitors]]
name = "localized_within_30cm"  # design: 6x the RTK 1 sigma; the map budget assumes ~2 cm
column = "err_xy"
op = "always"
hi = 0.3
```

  `coverage_smoke.scenario.toml`: the same with `name = "dog-auto-coverage-smoke"`, comment "the whole mission loop on a 20 x 20 m
  rubble field at the mission ray count (5k, pooled scoring): sweep, fill, return home", `rays = { value = 5000, ... rationale
  = "Mission ray count (autonomy.toml lidar.rays_mission)." }`, `pool = { value = 2, ... rationale = "5k rays: 10 cm scoring
  cells (Task 10 density equivalence)." }`, `t_budget = { value = 150.0, ... rationale = "20 x 20 m at 1.5 m/s with 8 m lanes is
  ~60 s of sweep; the rest is fill and return." }`, `return_margin = { value = 10.0, unit = "s", fidelity = "design",
  rationale = "Home is at most ~25 m away on a 20 m field: 10 s in hand." }`, `script = "coverage_smoke.py"`, the `cmd`
  pointing at its own toml, `[limits] wall_s = 1200` (`# 160 sim-s at 5k on rubble ~ 3 min x 4`), the two monitors above plus:

```toml
[[monitors]]
name = "covers_95_percent"      # spec section 7
column = "coverage"
op = "final"
lo = 0.95

[[monitors]]
name = "returns_home"           # phase 4 = DONE
column = "phase"
op = "final"
lo = 4
hi = 4
```

- [ ] **Step 4: run, expect PASS (golden `None`):**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_mission.py -n 8'`
- [ ] **Step 5: record goldens.** Run each once with the runner,
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/napl program examples/dog/missions/coverage_smoke/coverage_smoke.scenario.toml --runs-dir /workspace/wt/napl-cuas/runs/auto && .venv/bin/napl program examples/dog/missions/map_quality/map_quality.scenario.toml --runs-dir /workspace/wt/napl-cuas/runs/auto'`,
  read `sim.state_hash` from each `result.json`, write both into `GOLDEN` (Edit), re-run Step 4 (PASS with the hashes).
- [ ] **Step 6: measure `c_py`.** From the coverage_smoke `summary.json` (`ms_per_frame`, `metrics_ms_per_frame`), write the
  per-module milliseconds and `c_py = sum(ms_per_frame) / 1000 / 0.1` into `docs/dog/autonomy.md` "Wall-time model" and recompute
  the 20 min mission table. If `c_py > 0.5` (Python doubles the 5k rubble mission), the doc names the largest module and the
  cheaper setting used (traversability window, ICP rate, `mapped` cadence) before Task 10 runs.
- [ ] **Step 7: commit** `git add examples/dog/autonomy/mission.py examples/dog/autonomy/scene.py examples/dog/automission.py examples/dog/missions docs/dog/autonomy.md tests/python/test_dog_autonomy_mission.py && git commit -m "dog autonomy: mission executive (sweep / fill / return / abort, stuck skip, budgeted return), 10 Hz robot loop, scenario harness with ground-truth scoring, map_quality and coverage_smoke missions with goldens" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 10: density equivalence, measured swath -> lane spacing, the 100 x 100 m missions and Monte Carlo

**Files:**
- Create: `examples/dog/autonomy/swath.py`, `examples/dog/automc.py`,
  `examples/dog/missions/coverage_rubble/coverage_rubble.scenario.toml` + `coverage_rubble.py`,
  `examples/dog/missions/coverage_rubble_20k/coverage_rubble_20k.scenario.toml` + `coverage_rubble_20k.py`,
  `examples/dog/missions/coverage_ruins/coverage_ruins.scenario.toml` + `coverage_ruins.py`
- Modify: `examples/dog/autonomy/autonomy.toml` (`plan.lane_m` = the measured value, `fidelity = "measured"`),
  `docs/dog/autonomy.md` (sections "Swath", "Coverage", "Monte Carlo"), `tests/python/test_dog_autonomy_mission.py`
  (`LONG` set: listed and monitor-checked, never run by pytest)
- Test: `tests/python/test_dog_autonomy_density.py`

**Interfaces:**
- Consumes: `scene.make_field`, `scene.scenario`, `carrier.*`, `lidar.*`, `elevmap.ElevationMap`, `automission.py`,
  `land_mc.wilson` (`examples/skydio-land/land_mc.py`, stdlib-only module).
- Produces: `swath.drive_and_map(kind, rays, seed, out_dir, t_s=10.0, size=20.0) -> (ElevationMap, cfg, (x_min, x_max))`
  (straight carrier pass along y = 0 at `v_max`, mapped with the true pose, datasheet angular noise and attitude sigma),
  `swath.band_coverage(m, pool, x_range, y_half) -> float`, `swath.profile(m, pool, x_range, step=0.5, reach=12.0) -> (offsets,
  fraction)`, `swath.measure(kind, rays, pool, seed, out_dir) -> dict(w95_left, w95_right, lane_m)`;
  `automc.py <scenario> --seeds a:b --jobs N --out DIR` (one `automission.py` process per seed, `NAPL_PARAMS={"seed": s}`;
  success = `coverage >= 0.95 and rms <= 0.05 and phase == 4 and t <= t_budget`; writes `runs.jsonl` and `report.md`: n, k,
  Wilson 95 % interval, medians of coverage / rms / time / distance, failure counts by reason).

- [ ] **Step 1: failing test** `tests/python/test_dog_autonomy_density.py`:

```python
import pytest

import napl
from autonomy import swath

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@needs_kernel
def test_5k_rays_at_10cm_cover_like_20k_at_5cm(tmp_path):
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    m20, cfg, xr = swath.drive_and_map("rubble", 20000, 1, tmp_path / "a")
    m5, _, xr5 = swath.drive_and_map("rubble", 5000, 1, tmp_path / "b")
    assert xr == xr5  # the carrier's route does not depend on the ray count
    c20, c5 = swath.band_coverage(m20, 1, xr, 3.0), swath.band_coverage(m5, 2, xr, 3.0)
    assert c20 > 0.8 and abs(c5 - c20) <= 0.02
    assert swath.band_coverage(m5, 1, xr, 3.0) < c20 - 0.1  # unpooled, 5k rays fall well short: why missions score at pool 2


@needs_kernel
def test_swath_profile_is_symmetric_enough_and_gives_a_lane(tmp_path):
    r = swath.measure("rubble", 5000, 2, 1, tmp_path)
    assert 2.0 < r["w95_left"] < 12.0 and 2.0 < r["w95_right"] < 12.0
    assert r["lane_m"] == pytest.approx(2 * min(r["w95_left"], r["w95_right"]) * 0.9)
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_density.py -n 8'`
- [ ] **Step 3: implement `examples/dog/autonomy/swath.py`** (harness):

```python
"""Swath and density measurements on a straight carrier pass, mapped with the true pose (isolates the sensor from localization).
    cd examples/dog && ../../.venv/bin/python -m autonomy.swath [--kind rubble] [--rays 5000] [--pool 2] [--seed 1]"""

import argparse
import json
import tempfile
from pathlib import Path

import numpy as np

import napl

from . import carrier, config, elevmap, lidar, scene

LANE_OVERLAP = 0.9  # lane = 2 x w95 x 0.9: 10 % overlap between neighbouring swaths


def drive_and_map(kind, rays, seed, out_dir, t_s=10.0, size=20.0):
    out_dir = Path(out_dir)
    cfg = config.override(config.load(), **{"map.size": size})
    f = scene.make_field({"kind": kind, "size": size, "patch": 10.0}, seed)
    start = (-size / 2 + 2.0, 0.0, 0.0)
    sim = napl.Sim.load(carrier.build(f, cfg, rays, start, out_dir))
    c = carrier.Carrier(sim, f, cfg, start, seed=seed)
    ros = lidar.Rosette(sim, "lidar", rays, c.dt, cfg)
    ext = lidar.extrinsic((out_dir / "carrier.xml").read_text(), "lidar", "carrier")
    m, rng = elevmap.ElevationMap(cfg), np.random.default_rng([seed, 0x5A7])
    c.command(cfg["carrier"]["v_max"], 0.0, 0.0)
    xs = []
    for _ in range(int(round(t_s * cfg["lidar"]["rate_hz"]))):
        c.advance(1.0 / cfg["lidar"]["rate_hz"])
        p, q = c.truth()
        s = ros.latest()
        o = p + lidar.qmat(q) @ ext[0]
        m.fuse(lidar.points(s, o, lidar.qmul(q, ext[1]), cfg["lidar"]["ang_sigma"], rng), o, 0.0, cfg["odom"]["att_sigma"])
        xs.append(float(p[0]))
    return m, cfg, (min(xs), max(xs))


def _centres(m, pool):
    q = m.n // pool
    return m.x0 + (np.arange(q) + 0.5) * m.res * pool


def band_coverage(m, pool, x_range, y_half):
    c = _centres(m, pool)
    cols = (c >= x_range[0]) & (c <= x_range[1])
    rows = np.abs(c) < y_half
    return float(m.mapped(pool)[np.ix_(rows, cols)].mean())


def profile(m, pool, x_range, step=0.5, reach=12.0):
    c = _centres(m, pool)
    cols = (c >= x_range[0]) & (c <= x_range[1])
    mp = m.mapped(pool)[:, cols]
    offs = np.arange(-reach, reach, step) + step / 2
    frac = np.array([mp[np.abs(c - o) < step / 2].mean() if (np.abs(c - o) < step / 2).any() else 0.0 for o in offs])
    return offs, frac


def measure(kind, rays, pool, seed, out_dir):
    m, cfg, xr = drive_and_map(kind, rays, seed, out_dir)
    offs, frac = profile(m, pool, xr)
    w = {}
    for side, sel in (("w95_right", offs < 0), ("w95_left", offs > 0)):
        o, fr = np.abs(offs[sel]), frac[sel]
        order = np.argsort(o)
        ok = np.cumprod(fr[order] >= 0.95).astype(bool)  # contiguous from the robot outwards
        w[side] = float(o[order][ok].max() + 0.25) if ok.any() else 0.0
    w["lane_m"] = 2 * min(w["w95_left"], w["w95_right"]) * LANE_OVERLAP
    return w


if __name__ == "__main__":
    a = argparse.ArgumentParser()
    a.add_argument("--kind", default="rubble")
    a.add_argument("--rays", type=int, default=5000)
    a.add_argument("--pool", type=int, default=2)
    a.add_argument("--seed", type=int, default=1)
    a = a.parse_args()
    with tempfile.TemporaryDirectory() as d:
        print(json.dumps(measure(a.kind, a.rays, a.pool, a.seed, Path(d))))
```

  (`measure` reports the outer edge of the last 0.5 m bin that is still >= 95 % mapped, hence `+ 0.25`; the test pins
  `lane_m = 2 min(w) 0.9` exactly.)
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_density.py -n 8'`.
  If the density test fails (|c5 - c20| > 0.02), the fallback is in the doc and in the config, not in the bound: score the
  missions at 20k rays (`rays_mission = 20000`, `pool = 1`), the rubble mission then costs ~70 min (`bg` only) and the MC below
  shrinks to n = 8.
- [ ] **Step 5: measure the swath** on rubble and ruins (seeds 1-3, 5k rays, pool 2):
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run 'cd examples/dog && for k in rubble ruins; do for s in 1 2 3; do ../../.venv/bin/python -m autonomy.swath --kind $k --seed $s; done; done'`.
  Set `plan.lane_m` in `autonomy.toml` to the smallest `lane_m` of the six (rounded down to 0.5 m) with `fidelity = "measured"`
  and a rationale naming this command and the six numbers; write the table into `docs/dog/autonomy.md` "Swath". Re-run Task 9's
  goldens (`test_dog_autonomy_mission.py`); the lane change moves `coverage_smoke`'s hash: re-record it as in Task 9 Step 5.
- [ ] **Step 6: the three 100 x 100 m scenarios.** Same layout as `coverage_smoke` (`[model]`, `[program]` with the scenario's own
  toml, seed parameter, the four monitors `map_rms_within_5cm`, `localized_within_30cm`, `covers_95_percent`, `returns_home`);
  `[limits] wall_s = 3600` (`# ~21-25 min expected at 5k, 3600 = remote.sh's cap`) for the 5k ones and `wall_s = 7200`
  (`# ~70 min expected at 20k`) for the 20k one; `[auto]`:
  - `coverage_rubble`: `kind = "rubble"`, `size = 100.0 m` (spec section 7), `patch` = Task 4's choice for rubble, `rays = 5000`,
    `pool = 2`, `t_budget = 1200.0 s` (spec: 20 min), `start_x = start_y = -46.0 m` (south-west corner, 4 m inside, on the first
    lane's start), `start_yaw = 0.0 rad`.
  - `coverage_rubble_20k`: the same with `rays = 20000` (datasheet), `pool = 1`.
  - `coverage_ruins`: `kind = "ruins"`, `patch` = Task 4's choice for ruins, `start_x = start_y = -47.5 m` (a ruins cell centre:
    walls lie on the 5 m grid from -50), the rest as `coverage_rubble`.
  Each `.py` model script is the Task 9 one with its docstring. Add to `tests/python/test_dog_autonomy_mission.py`:

```python
LONG = ["coverage_rubble/coverage_rubble", "coverage_rubble_20k/coverage_rubble_20k", "coverage_ruins/coverage_ruins"]  # bg only


def test_long_missions_are_listed_with_the_mission_monitors():
    for name in LONG:
        sc = tomllib.loads((EX / f"{name}.scenario.toml").read_text())
        mon = {m["name"] for m in sc["monitors"]}
        assert {"map_rms_within_5cm", "localized_within_30cm", "covers_95_percent", "returns_home"} <= mon
        auto = config.leaves(sc["auto"], "auto")
        assert auto["size"] == 100.0 and auto["t_budget"] == 1200.0
```

- [ ] **Step 7: implement `examples/dog/automc.py`:**

```python
"""Monte Carlo of a dog autonomy mission: one run per seed (automission.py in its own process), success = the spec's mission
criteria (coverage >= 95 % of the reachable traversable area, map RMS <= 5 cm, home by the 20 min budget), a Wilson interval.

    python examples/dog/automc.py <scenario.scenario.toml> --seeds 1001:1025 --jobs 8 --out <dir>

DEV seeds (tuning): 1-30. HELD-OUT seeds (never run before the numbers in docs/dog/autonomy.md were taken): 1001 and up."""

import argparse
import json
import os
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "skydio-land"))
from autonomy import scene  # noqa: E402
from land_mc import wilson  # noqa: E402  (stdlib-only module)


def one(scenario, seed, out):
    d = out / f"seed{seed}"
    d.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "NAPL_PARAMS": json.dumps({"seed": seed})}
    r = subprocess.run([sys.executable, str(HERE / "automission.py"), str(scenario), str(d)], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        return {"seed": seed, "error": r.stderr[-1500:]}
    return {"seed": seed, **json.loads((d / "summary.json").read_text())}


def reasons(s, t_budget):
    if "error" in s:
        return ["crashed"]
    out = []
    if not s["coverage"] >= 0.95:
        out.append("coverage")
    if not s["rms"] <= 0.05:
        out.append("rms")
    if s["phase"] != 4:
        out.append("not_home")
    if s["t"] > t_budget:
        out.append("late")
    return out


def main():
    a = argparse.ArgumentParser()
    a.add_argument("scenario")
    a.add_argument("--seeds", default="1001:1025")
    a.add_argument("--jobs", type=int, default=8)
    a.add_argument("--out", required=True)
    a = a.parse_args()
    scenario, out = Path(a.scenario).resolve(), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    lo, hi = (int(v) for v in a.seeds.split(":"))
    auto, _, _ = scene.scenario(scenario)
    with ThreadPoolExecutor(a.jobs) as ex:
        runs = list(ex.map(lambda s: one(scenario, s, out), range(lo, hi)))
    with open(out / "runs.jsonl", "w") as f:
        for s in runs:
            f.write(json.dumps(s) + "\n")
    why = {s["seed"]: reasons(s, auto["t_budget"]) for s in runs}
    k, n = sum(not w for w in why.values()), len(runs)
    wl, wh = wilson(k, n)
    ok = [s for s in runs if "error" not in s]
    med = {c: statistics.median(s[c] for s in ok) for c in ("coverage", "rms", "t", "distance")} if ok else {}
    fails = {r: sum(r in w for w in why.values()) for r in ("coverage", "rms", "not_home", "late", "crashed")}
    rep = [f"# MC {scenario.name}, seeds {a.seeds}", "", f"success {k}/{n}, Wilson 95 % [{wl:.3f}, {wh:.3f}]", "",
           "| median | value |", "|---|---|", *[f"| {c} | {v:.4g} |" for c, v in med.items()], "",
           "| failure | count |", "|---|---|", *[f"| {r} | {c} |" for r, c in fails.items()]]  # fmt: skip
    (out / "report.md").write_text("\n".join(rep) + "\n")
    print("\n".join(rep))


if __name__ == "__main__":
    main()
```

- [ ] **Step 8: run the missions and the MC on hfbox** (expected wall times from the model, Task 9's measured `c_py` replaces them
  in the doc):
  1. Headline 20k rubble mission (~70 min):
     `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh bg --slots 1 auto-rubble20k '.venv/bin/napl program examples/dog/missions/coverage_rubble_20k/coverage_rubble_20k.scenario.toml --runs-dir /workspace/wt/napl-cuas/runs/auto'`
  2. Ruins MC alongside it, n = 8 (one batch of 4 jobs x 2, ~25 min):
     `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh bg --slots 1 auto-mc-ruins '.venv/bin/python examples/dog/automc.py examples/dog/missions/coverage_ruins/coverage_ruins.scenario.toml --seeds 1001:1009 --jobs 4 --out /workspace/wt/napl-cuas/runs/auto/mc-ruins'`
  3. After both: rubble MC, n = 24 (3 batches of 8, ~75 min):
     `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh bg --slots 2 auto-mc-rubble '.venv/bin/python examples/dog/automc.py examples/dog/missions/coverage_rubble/coverage_rubble.scenario.toml --seeds 1001:1025 --jobs 8 --out /workspace/wt/napl-cuas/runs/auto/mc-rubble'`
  Follow with `tools/remote.sh log <name>`; total ~2.5 h wall. Copy only `report.md` / `summary.json` text into the doc (no media,
  no maps).
- [ ] **Step 9: write `docs/dog/autonomy.md`** "Coverage" (the three missions: coverage, RMS (edge cells separately), time,
  distance, bumps, wall time against the model) and "Monte Carlo" (both reports, Wilson intervals, failure reasons, what the
  failures are). Then `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_mission.py tests/python/test_dog_autonomy_density.py -n 8 && .venv/bin/python tools/check_licenses.py'` (PASS, 0 problems).
- [ ] **Step 10: commit** `git add examples/dog/autonomy/swath.py examples/dog/autonomy/autonomy.toml examples/dog/automc.py examples/dog/missions docs/dog/autonomy.md tests/python/test_dog_autonomy_density.py tests/python/test_dog_autonomy_mission.py && git commit -m "dog autonomy: density equivalence (5k pooled = 20k), measured swath sets the lanes, 100 x 100 m rubble / ruins missions, Monte Carlo with Wilson bounds" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 11: SP5 on the dog through SP4's velocity-command interface (`dogloco.py`) + `dog_smoke`

Gate: `git log --oneline napl-v1 | grep -m1 "Merge p12-dog"` and the SP4 merge (`examples/dog/loco/controller.py`,
`examples/dog/dogtruth.py`, `examples/dog/dogcourse.py` present on `napl-v1`). Without both, stop and report; Tasks 1-10 stand alone.

**Files:**
- Create: `examples/dog/autonomy/dogloco.py`, `examples/dog/missions/dog_smoke/dog_smoke.scenario.toml`,
  `examples/dog/missions/dog_smoke/dog_smoke.py`
- Modify: `examples/dog/autonomy/scene.py` (`body = "dog"` branch in `build` and `_loco`), `examples/dog/autonomy/autonomy.toml`
  (`[odom]` drift numbers from SP4's terrain MC, `fidelity = "measured"`), `docs/dog/autonomy.md` (section "On the dog"),
  `tests/python/test_dog_autonomy_mission.py` (`GOLDEN["dog_smoke/dog_smoke"]`)
- Test: `tests/python/test_dog_autonomy_dog.py`

**Interfaces:**
- Consumes (SP4, exact names): `loco.controller.Controller(model, hw, hmap)`, `.step((vx, vy, wz))` (500 Hz, each followed by
  `sim.step(2)`), `.fix(p_xy, yaw, age_s)`, `.mode`, `.est.x` (18; `[:3]` = trunk position), `.est.R` (3 x 3), `loco.controller.DT`
  (0.002), `loco.model.from_build(xml)`, `loco.hw.Hw(sim)`, `dogtruth.Truth(sim, field)` (`.pos()`, `.quat()`, `.fallen()`),
  `dogcourse.base_model()`, `dogcourse.info(model)` (reads `course.json` beside the model: `nominal`); SP4's `HeightMap`
  protocol (`height(x, y)`, world z, `nan` = unknown) implemented by the adapter over SP5's `ElevationMap`.
- Produces: `dogloco.TRUNK = "deck_top"`, `dogloco.devices(xml_text) -> (rosette rangefinder dict, gnss dict)` (found by type, not
  by name), `dogloco.cfg_from_devices(cfg, lid, gn) -> cfg` (GNSS origin / rate / latency / sigmas and rosette limits from the dog's
  own devices), `dogloco.build(f, auto, seed, start, out) -> Path` (the dog with its lidar kept, on the field, at the start pose,
  `rays_per_frame` set to the scenario's `rays`, `course.json` for `dogcourse.info`), `dogloco.FixLedger` (`.odom(p_est, R_est) ->
  (p_odom, yaw_odom)`, `.jumped(p0, R0, p1, R1)`), `dogloco.DogLoco(sim, model, cfg, f, start, z_ground0)` implementing
  `iface.Locomotion` + `height(x, y)` (the controller's map; `attach(emap)`), attributes `time_of_last_frame`, `gnss_lost`, `bumps`
  (always 0), `distance`, `fallen` (ground truth, harness column). `odometry()` advances the ledger: only the robot calls it, once
  per frame.

- [ ] **Step 1: verify SP4's names** against the merged code: `grep -n "def fix\|def step\|self.mode\|class Estimator\|self.R\b\|^DT" examples/dog/loco/controller.py examples/dog/loco/estimator.py` and `grep -n "def base_model\|def info\|def fallen" examples/dog/dogcourse.py examples/dog/dogtruth.py`.
  A renamed attribute is fixed in this task's code, never by editing SP4. Copy SP4's measured odometry drift (its
  `docs/dog/locomotion.md` estimator section: position drift per metre, yaw drift per metre) into `autonomy.toml [odom]` with
  `fidelity = "measured"` and the doc as rationale.
- [ ] **Step 2: failing test** `tests/python/test_dog_autonomy_dog.py`:

```python
import math

import numpy as np
import pytest

import napl
from autonomy import config, dogloco, lidar, localize, scene

HAS_SP4 = (ROOT / "examples" / "dog" / "loco" / "controller.py").exists()
needs_dog = pytest.mark.skipif(napl._napl is None or not HAS_SP4, reason="kernel or SP4 (examples/dog/loco) missing")


def test_odometry_takes_sp5_fixes_back_out():
    led = dogloco.FixLedger()
    R = lambda a: localize.rz(a)
    out = [led.odom(np.array([float(t), 0.0, 0.3]), R(0.0)) for t in range(6)]
    p5 = np.array([5.0, 0.0, 0.3])
    p5j, d = p5 + np.array([0.3, -0.2, 0.0]), 0.1  # SP4 absorbed a fix: position and yaw jump
    led.jumped(p5, R(0.0), p5j, R(d))
    for t in range(6, 10):
        p_est = p5j + R(d) @ np.array([t - 5.0, 0.0, 0.0])  # SP4 keeps integrating in its jumped frame
        out.append(led.odom(p_est, R(d)))
    for t, (p, yaw) in enumerate(out):
        np.testing.assert_allclose(p, [t, 0.0, 0.3], atol=1e-12)
        assert yaw == pytest.approx(0.0, abs=1e-12)


@needs_dog
def test_dog_scene_keeps_the_lidar_and_does_not_see_itself(tmp_path):
    auto = {"kind": "rubble", "size": 20.0, "patch": 10.0, "rays": 5000}
    f = scene.make_field(auto, 1)
    p = dogloco.build(f, auto, 1, (-8.0, -8.0, 0.0), tmp_path)
    xml = p.read_text()
    lid, gn = dogloco.devices(xml)
    assert lid["projection"] == "rosette" and lid["rays_per_frame"] == 5000 and gn["rate_hz"] > 0
    ext = lidar.extrinsic(xml, lid["site"], dogloco.TRUNK)  # pos / quat / euler only, else ValueError
    sim = napl.Sim.load(p)
    cfg = dogloco.cfg_from_devices(config.override(config.load(), **{"map.size": 20.0}), lid, gn)
    dog = dogloco.DogLoco(sim, p, cfg, f, (-8.0, -8.0, 0.0), float(f.height(-8.0, -8.0)))
    ros = lidar.Rosette(sim, lid["name"], 5000, dog.dt, cfg)
    dog.advance(1.0)  # standing up
    s = ros.latest()
    assert s is not None and s.r.min() > 0.6  # nothing of the dog's own body in the returns
    assert np.isfinite(ext[0]).all()


@needs_dog
def test_dog_smoke_passes_and_replays(tmp_path):
    from napl import program

    res, _ = program.run(ROOT / "examples" / "dog" / "missions" / "dog_smoke" / "dog_smoke.scenario.toml", tmp_path / "runs")
    assert res["replay"]["available"] and res["replay"]["match"]
    assert res["pass"], res["fail_reasons"]
```

- [ ] **Step 3: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_autonomy_dog.py -n 8'`
  (SP4 adds osqp and scipy through `install_dev.sh`; the rebuild picks up the merged SP3b / SP4 tree.)
- [ ] **Step 4: implement `examples/dog/autonomy/dogloco.py`:**

```python
"""SP5 on the dog: the Locomotion door over SP4's controller. Builds the dog scene with its lidar kept (SP4's scenes strip it),
runs SP4's 500 Hz loop (Controller.step + sim.step(2)), hands SP5 the odometry of SP4's estimator with SP5's own fixes taken
back out (FixLedger), forwards SP5's pose fixes, and serves SP5's elevation map to SP4 as its HeightMap."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from napl.terrain import add_field

from . import carrier, config, lidar
from .iface import Odom
from .localize import rz, wrap

TRUNK = "deck_top"


def devices(xml_text):
    d = json.loads(ET.fromstring(xml_text).find("custom/text[@name='napl.devices']").get("data"))
    lid = next(r for r in d["rangefinders"] if r.get("projection") == "rosette")
    return lid, d["gnss_receivers"][0]


def cfg_from_devices(cfg, lid, gn):
    return config.override(cfg, **{
        "gnss.lat0": gn["lat0"], "gnss.lon0": gn["lon0"], "gnss.alt0": gn.get("alt0", 0.0), "gnss.rate_hz": gn["rate_hz"],
        "gnss.latency_s": gn.get("latency_s", 0.0), "gnss.horiz_std": gn["horiz_std"], "gnss.vert_std": gn["vert_std"],
        "lidar.fov_v_min": lid["fov_v_min"], "lidar.fov_v_max": lid["fov_v_max"], "lidar.rate_hz": lid["rate_hz"],
        "lidar.pattern_seed": lid.get("pattern_seed", 0),
    })  # fmt: skip


def build(f, auto, seed, start, out):
    import dogcourse  # SP4 harness (examples/dog on sys.path)

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    nominal = dogcourse.base_model()
    root = ET.parse(nominal).getroot()
    t = root.find("custom/text[@name='napl.devices']")
    d = json.loads(t.get("data"))
    for r in d["rangefinders"]:
        if r.get("projection") == "rosette":
            r["rays_per_frame"] = int(auto["rays"])
    t.set("data", json.dumps(d))
    add_field(root, f)
    trunk = next(b for b in root.iter("body") if b.get("name") == TRUNK)
    z = float(trunk.get("pos", "0 0 0").split()[2])  # nominal standing height above z = 0
    x, y, yaw = start
    cfg = config.load()["carrier"]
    c, s = math.cos(yaw), math.sin(yaw)
    ground = max(float(f.height(x + c * i * cfg["foot_a"] - s * j * cfg["foot_b"], y + s * i * cfg["foot_a"] + c * j * cfg["foot_b"]))
                 for i in (-1, 1) for j in (-1, 1))  # fmt: skip
    trunk.set("pos", f"{x!r} {y!r} {z + ground!r}")
    trunk.set("quat", f"{math.cos(yaw / 2)!r} 0 0 {math.sin(yaw / 2)!r}")
    path = out / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    (out / "course.json").write_text(json.dumps({"course": f"sp5-{auto['kind']}", "seed": int(seed), "nominal": str(nominal)}))
    return path


def _yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


class FixLedger:
    """SP4's estimator absorbs SP5's fixes (position and yaw jumps). SP5's EKF must not see its own corrections as motion:
    odometry = SP4's increments with the jumps removed, rotated back by the accumulated yaw jump."""

    def __init__(self):
        self.p, self.prev, self.jp, self.jyaw = None, None, np.zeros(3), 0.0

    def odom(self, p_est, R_est):
        if self.p is None:
            self.p = np.array(p_est, float)
        else:
            self.p = self.p + rz(-self.jyaw) @ (p_est - self.prev - self.jp)
        self.prev, self.jp = np.array(p_est, float), np.zeros(3)
        return self.p.copy(), wrap(_yaw(R_est) - self.jyaw)

    def jumped(self, p0, R0, p1, R1):
        self.jp += np.asarray(p1) - np.asarray(p0)
        self.jyaw += wrap(_yaw(R1) - _yaw(R0))


class DogLoco:
    def __init__(self, sim, model, cfg, f, start, z_ground0):
        import dogcourse
        import dogtruth
        from loco import controller as Ct
        from loco import hw as H
        from loco import model as M

        xml = Path(model).read_text()
        lid, gn = devices(xml)
        self.sim, self.cfg, self.ctl_dt = sim, cfg, Ct.DT
        self.dt = Ct.DT / 2  # physics step: two per control tick
        self.lidar, self.gnss, self.base = lid["name"], gn["name"], f"/world/{TRUNK}"
        self.lever = lidar.extrinsic(xml, gn["site"], TRUNK)[0]
        self.period = int(round(1.0 / (lid["rate_hz"] * self.dt)))
        self.emap, self.z0, self.cmd = None, float(z_ground0), (0.0, 0.0, 0.0)
        self.ctl = Ct.Controller(M.from_build(dogcourse.info(model)["nominal"]), H.Hw(sim), self)
        self.tr = dogtruth.Truth(sim, f)
        self.ledger = FixLedger()
        self.snap = None
        self.gnss_lost, self.bumps, self.distance, self.time_of_last_frame, self._xy = False, 0, 0.0, 0.0, None

    def attach(self, emap):
        self.emap = emap

    def height(self, x, y):  # SP4's HeightMap, served from SP5's map
        h = np.full(np.shape(x), np.nan) if self.emap is None else np.asarray(self.emap.height(x, y), float)
        if self.sim.tick * self.dt < self.cfg["plan"]["init_s"]:  # stand-up: the operator-given ground under the start
            h = np.where(np.isnan(h), self.z0, h)
        return h

    def _est(self):
        return np.array(self.ctl.est.x[:3], float), np.array(self.ctl.est.R, float)

    def command(self, vx, vy, wz):
        self.cmd = (float(vx), float(vy), float(wz))

    def limits(self):
        C = self.cfg["carrier"]
        return {"v_max": C["v_max"], "wz_max": C["wz_max"]}  # decision 8: SP4's measured speeds replace these

    def truth(self):
        return self.tr.pos(), self.tr.quat()

    @property
    def fallen(self):  # harness column (ground truth); the robot sees ctl.mode == "FALLEN" through Odom.fallen
        return bool(self.tr.fallen())

    def advance(self, seconds):
        for _ in range(int(round(seconds / self.ctl_dt))):
            if self.sim.tick % self.period == 0:  # the lidar / GNSS sampling tick: snapshot the estimate the frame belongs to
                self.time_of_last_frame = self.sim.tick * self.dt
                self.snap = self._est()
                lost = carrier.sky_blocked(self.sim, self.tr.pos() + lidar.qmat(self.tr.quat()) @ self.lever, self.cfg)
                if lost != self.gnss_lost:
                    self.sim.set(f"/devices/{self.gnss}", "fix_lost", 1.0 if lost else 0.0)
                    self.gnss_lost = lost
            self.ctl.step(self.cmd)
            self.sim.step(2)
        xy = self.tr.pos()[:2]
        if self._xy is not None:
            self.distance += float(np.hypot(*(xy - self._xy)))
        self._xy = xy

    def odometry(self):
        p, R = self.snap if self.snap is not None else self._est()
        po, yaw = self.ledger.odom(p, R)
        q = lidar.quat_zyx(yaw, -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[2, 1], R[2, 2]))
        return Odom(self.time_of_last_frame, po, q, np.zeros(3), np.zeros(3), self.ctl.mode == "FALLEN")

    def fix(self, p_xy, yaw, t_pose):
        if self.ctl.mode in ("STANDUP", "FALLEN", "SIT"):
            return
        p0, R0 = self._est()
        self.ctl.fix(np.asarray(p_xy, float), float(yaw), self.sim.tick * self.dt - t_pose)
        p1, R1 = self._est()
        self.ledger.jumped(p0, R0, p1, R1)
```

  `scene.py`: in `build`, before the carrier line:

```python
    if auto.get("body", "carrier") == "dog":
        from . import dogloco

        return dogloco.build(make_field(auto, prm["seed"]), auto, prm["seed"], _start(auto), out)
```

  and at the top of `_loco`:

```python
    if auto.get("body", "carrier") == "dog":
        from . import dogloco

        xml = Path(model).read_text()
        lid, gn = dogloco.devices(xml)
        cfg = dogloco.cfg_from_devices(cfg, lid, gn)
        dog = dogloco.DogLoco(sim, model, cfg, f, start, float(f.height(start[0], start[1])))
        return dog, lidar.extrinsic(xml, lid["site"], dogloco.TRUNK), dog.lever, cfg
```

  and in `run`, after `rob = mission.Robot(...)`: `if hasattr(car, "attach"): car.attach(rob.map)` (SP4 walks on SP5's map).
  `mission.Robot` reads the lidar / GNSS device names from `loco.lidar` / `loco.gnss`, so nothing else changes.
  `examples/dog/missions/dog_smoke/dog_smoke.scenario.toml`: as `map_quality` with `name = "dog-auto-dog-smoke"`, comment "SP5 on
  the dog: the whole autonomy stack drives SP4's controller on a 20 x 20 m rubble field for 40 s (sweep start), 5k rays",
  `[auto] body = "dog"`, `kind = "rubble"`, `size = 20.0 m`, `patch = 10.0 m`, `rays = 5000` (the dog's instance param),
  `pool = 2`, `t_budget = 40.0 s` ("stand-up plus ~30 s of walking"), `return_margin = 0.0 s` ("a smoke run"), start
  `-8.0 / -8.0 m, 0.0 rad`, `script = "dog_smoke.py"`, its own `cmd`, `[limits] wall_s = 1800` (`# ~35-45 min per 20 min mission
  at 5k on the dog -> ~2 min for 40 s, x 15 for SP4's controller uncertainty`), monitors `map_rms_within_5cm`,
  `localized_within_30cm` and:

```toml
[[monitors]]
name = "never_falls"
column = "fallen"
op = "always"
hi = 0
```

- [ ] **Step 5: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_autonomy_dog.py -n 8'`.
  Then record the golden: add `"dog_smoke/dog_smoke": None` to `GOLDEN` in `test_dog_autonomy_mission.py`, run
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/napl program examples/dog/missions/dog_smoke/dog_smoke.scenario.toml --runs-dir /workspace/wt/napl-cuas/runs/auto'`,
  write its `sim.state_hash` into `GOLDEN`, and run `test_dog_autonomy_mission.py` (`-n 8`) green.
- [ ] **Step 6: one 100 x 100 m rubble mission on the dog** (`examples/dog/missions/coverage_rubble_dog/` = `coverage_rubble` with
  `body = "dog"`; `[limits] wall_s = 7200`; listed in `LONG`), in the background, ~35-45 min expected:
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh bg --slots 1 auto-dog '.venv/bin/napl program examples/dog/missions/coverage_rubble_dog/coverage_rubble_dog.scenario.toml --runs-dir /workspace/wt/napl-cuas/runs/auto'`.
  Write "On the dog" in `docs/dog/autonomy.md`: coverage / RMS / time / falls / wall time against the carrier run, what SP4 limited
  (speed, turn rate, footholds in the blind zone 0.72 m ahead of the lidar), and the fix-sigma request to SP4. Then
  `NAPL_REMOTE_DIR=/workspace/wt/napl-cuas tools/remote.sh run '.venv/bin/python tools/check_licenses.py'` (0 problems).
- [ ] **Step 7: commit** `git add examples/dog/autonomy/dogloco.py examples/dog/autonomy/scene.py examples/dog/autonomy/autonomy.toml examples/dog/missions/dog_smoke examples/dog/missions/coverage_rubble_dog docs/dog/autonomy.md tests/python/test_dog_autonomy_dog.py tests/python/test_dog_autonomy_mission.py && git commit -m "dog autonomy: SP5 on the dog through SP4's velocity commands (lidar kept, fix ledger, pose fixes back to SP4, SP5 map as SP4's HeightMap), dog_smoke mission" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

## Decisions to confirm (the hand-back lists them)

1. Missions and MC at 5k rays scored on 10 cm pooled cells (pinned by the density test), plus one 20k headline mission; the map
   itself stays at 5 cm.
2. 0.5 m planning blocks: doors narrower than ~1 m can read closed, in the plan and in the ground-truth denominator alike.
3. The initial pose is the truth plus `init_sigma` / `init_yaw` noise (operator initialisation at the start point).
4. GNSS outage = >= 75 % of a 30 deg sky ring blocked within 60 m (`gnss.sky_block`), via the existing `fix_lost` input.
5. The D435i is not fused into the map (the lidar's 0.72 m forward blind zone is left to the accumulated map, as the actuator study
   assumed); fusing depth is the first upgrade if SP4's footholds need it.
6. 4-DOF ICP with roll / pitch from odometry; 6-DOF is the upgrade path.
7. Numbers rule: the `napl program` runner keys (`[limits]`, monitor bounds, parameter ranges) stay bare with a source comment.
8. The 1.5 m/s carrier speed assumes SP4 reaches it on rubble; Task 11 reports what the dog actually does.
