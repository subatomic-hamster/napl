# SP4: dog locomotion (estimator, gait scheduler, convex MPC, leg control, terrain footholds, slip / fall handling, top speed, terrain MC) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the SP3b dog (`assemblies/dog/dog.toml`, 12 Qdd actuators, ~12 kg) walks on its own controller: a contact-aided estimator,
a gait scheduler (stand, walk, trot, flying trot, bound), convex MPC on ground reaction forces at 50 Hz, a QP whole-body
controller (WBIC-style; a Jacobian-transpose leg controller as the switchable baseline) at 500 Hz in MIT mode, terrain-aware footholds and posture from a height-map interface, slip and fall
handling, push rejection; it is measured honestly against the spec: flat top speed (target 3.5 m/s, study bound 2.4 m/s for a trot),
rocky-field mean speed (>= 1.5 m/s), 25 deg slopes + 15 cm steps, and >= 90 % no-fall per terrain family with a Wilson 95 % lower
bound, all written to `docs/dog/locomotion.md`; the SP3b speed sweep's `sp4` column is filled.

**Architecture:** a Python package `examples/dog/loco/` that is the robot's software (what would run on the Jetson): `hw.py` is
its only door to the robot (Qdd and IMU device attrs; nothing else), `model.py` builds its own kinematic / inertial model from the
design (the built MJCF's static body tree at q = 0 and the part masses, never mjData), `estimator.py` (Mahony attitude + linear
Kalman filter on body and feet, Bledt 2018; torque-based contact), `mpc.py` (Di Carlo 2018 convex MPC, OSQP sparse with a fixed
pattern), `wbc.py` (floating-base rigid-body dynamics of the controller's model, prioritized tasks, OSQP force / torque QP),
`gait.py` (phase scheduler, early / late touchdown, swing curves, Raibert + capture-point footsteps), `terrain.py`
(height-map protocol, ground-truth map until SP5, footholds, posture plane, step height), `safety.py` (slip, fall, safe-sit,
stand-up), `controller.py` (mode machine; 500 Hz loop, MPC every 10 ticks). Harness modules beside it play the world and are never
imported by `loco`: `dogtruth.py` (ground truth, pushes, the SP5 pose-fix stand-in), `dogcourse.py` (scenes: flat, criterion
course, the four terrain families with the lidar removed, truth routes, route follower), `dogwalk.py` (the `napl program`
harness), `dogtop.py` (top-speed search + the SP3b sweep slot), `dogmc.py` (Monte Carlo). Physics 1 kHz (the dog's
`timestep = 0.001`), leg control 500 Hz (`sim.step(2)` per control tick), MPC 50 Hz.

**Decisions this plan takes (the hand-back lists them for confirmation):**
1. **Language / location: Python** (numpy + OSQP's Python wheel) in `examples/dog/loco/`, measured first (Task 1). The 500 Hz loop is
   ~40 direct reads (`Sim.query` is a direct solver read, no JSON) and ~36-48 writes (`Sim.set` goes through the JSON command bus).
   Rule: if Task 1 measures more than 1.0 ms of I/O per 2 ms control tick, Task 1 Step 6 adds a C++ batch write `Sim.set_many`
   (bindings only: same Envelopes through the same bus, so logs, replay and every existing hash are unchanged). The MPC stays in
   Python either way (OSQP's C solver does the work; Task 4 pins < 8 ms median, 40 % of the 20 ms period). A C++ controller device
   is not planned: the sim is not real time-bound, only the MC budget is (Task 1 table), and the real robot's controller would be a
   separate port anyway.
2. **Convex MPC + QP whole-body control (the user's choice), reached in two steps.** Task 6 brings the robot up on the simple MIT
   Cheetah "leg controller" (Di Carlo 2018: stance `tau_ff = J^T (-R^T f_mpc)`, kp = 0, joint damping; swing joint PD on the IK of the
   swing curve). Task 7 adds the final leg controller, a WBIC-style whole-body controller at 500 Hz (Kim et al. 2019, the Mini Cheetah
   structure): strict-priority kinematic tasks (stance feet still, body orientation, body position, swing feet), then an OSQP QP on
   the floating-base dynamics that keeps the MPC reaction forces as the reference, with friction-pyramid and joint-torque limits.
   The WBC is the default from Task 7 on. The J^T controller stays behind a switch (`leg_ctl = "wbc" | "jt"`, in the scenarios,
   `dogtop --leg`, `dogmc --leg`) as the measured baseline: Task 7 compares force tracking, Task 10 top speed, Task 11 the MC. WBC
   budget: median <= 1.0 ms per 2 ms tick in Python. Over budget, Task 7 Step 6 ports the WBC (dynamics + QP) to C++ as an opt-in
   nanobind module (`NAPL_LOCO_CPP`, Eigen MPL-2.0 + OSQP C Apache-2.0) with the same `solve()` signature, checked against the
   Python one on fixed inputs.
3. **Contact = torque-based estimate** (`tau_est` of the drivers through `J^T`), because the SP3b `FootContact` is a virtual sensor
   with no hardware; the FootContact device is ground truth for tests only.
4. **SP5 stand-ins, declared as controller inputs:** a height map (`loco.terrain.TruthMap` over `napl.terrain.Field.height`) and a
   10 Hz pose fix (truth + noise sigma 2 cm / 1 deg, 50 ms old: RTK + lidar class), both swapped for SP5's outputs later with the
   same interfaces. Without the pose fix, odometry drift of 2-5 % would misplace footholds by decimetres over 20 m on a truth map.
5. **Terrain MC on 30 x 30 m fields** (`napl.terrain.field(kind, size=30.0, seed)`, same generator and statistics as the 100 m
   fields, ~9x fewer hfield cells to load) along **ground-truth A* routes** followed by pure pursuit on the true pose: route planning
   and localization are SP5; SP4's claim is locomotion along a traversable route. Lidar and depth camera are removed from every
   locomotion scene (`dogcourse.strip_lidar`), per the sim budget (docs/models.md "Ground sim budget": the dog alone is 40-100x
   real time, the lidar is the cost).
6. **Criterion course** (spec s7 "25 deg slopes and 15 cm steps") is a hand-made `Field` (one 25 deg ramp up and down, two 15 cm
   steps up and two down), because `field("slopes")` draws 15-30 deg and 10-18 cm at random.
7. **Pushes** are instantaneous velocity changes of the free joint (`qvel` is writable through the bus: logged, replayed); there is
   no external-force attribute. A 0.5 m/s lateral step on 12 kg is a 6 N s impulse (a 60 N shove for 0.1 s).
8. **Self-righting is out of scope:** a fall ends in safe-sit (legs folded, damping); stand-up is attempted only when the estimated
   tilt is below 30 deg. A fall is a fall in the MC whatever happens next.
9. **Gains and gait tables live in Python constants** with a one-line source / rationale each; no new assembly or part numbers.

**Tech Stack:** Python 3.12, numpy, OSQP (pip `osqp`, Apache-2.0; its scipy dependency BSD-3), napl (`napl.Sim`,
`napl.terrain`, `napl.params`, `napl.program`, `napl.marine.write_native`), pytest (+xdist), hfbox via `tools/remote.sh`; C++20 /
nanobind only if Task 1 Step 6 triggers.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` sections 6 (SP4, rates), 7 (success criteria), 9 (risks 1-3);
inputs `docs/superpowers/plans/2026-10-16-sp3b-dog-assembly.md` (the dog, `examples/dog/dogsim.py`, `tools/dog_sweep.py` and its
`--gait` slot), `docs/superpowers/plans/2026-10-16-sp2-ground-kernel.md`, `docs/models.md` "Qdd", "FootContact", "Small-scale
terrain", "Ground sim budget", `docs/dog/actuator-study.md`.

## Global Constraints

- **Prerequisite:** branch `p12-dog` (SP3b) is merged into `napl-v1` before Task 1 (Task 1 Step 1 checks it). Tasks 4 and 5 are
  pure Python and may start before the merge. Work in worktree `../napl-wt/loco`, branch `p12-loco` from `napl-v1`.
- **SP3b names this plan depends on** (from the SP3b plan; Task 1 Step 1 re-checks them in the merged code, and a renamed one is
  fixed in this plan's files, never by editing SP3b code except the one `tools/dog_sweep.py` call of Task 10): `examples/dog/dogsim.py`
  `LEGS, JOINTS, KP, KD, SQUAT, STAND, CMD_HZ, build(out) -> (xml Path, report)`, `timestep(xml)`, `signs(report)`; device paths
  `/devices/<leg>:abad|hip|knee` (qdd), `/devices/<leg>:foot` (foot_contact), `/devices/compute:imu`, `/devices/power:pack_a|pack_b`;
  bodies `deck_top` (root, free joint `deck_top:free`), `<leg>:thigh`, `<leg>:shank`, `<leg>:foot_ball`; joints
  `<leg>:abad_j|hip_j|knee_j`; `assemblies/dog/head.toml` rangefinders `head:lidar`, `head:depth`; `tools/dog_sweep.py --gait
  module:function`, `function(sim, sg, report) -> mean forward speed (m/s)`; `tests/python/test_dog_stand.py::GOLDEN`.
- **Numbers rule:** every numeric leaf in assembly / part TOML is `"<inst>.<param>"` or a provenance table `{value, unit, fidelity,
  src | derivation | rationale}` (bare only for `BARE_OK` keys). SP4 adds no assembly or part TOML. Scenario files follow the
  `napl.params` / `napl.program` schema (`[[parameters]] default/min/max`, `[[monitors]] lo/hi` are bare numbers there, as in every
  existing scenario); each monitor bound carries a comment naming its source.
- **All compute on hfbox**, laptop = edits + git only:
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '<cmd>'` (add `--slots 2` for MC / search runs; `bg` for long ones).
  Build after any C++ change or merge: `cmake --preset dev && cmake --build --preset dev` then
  `PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh`.
- **USER RULE: targeted tests only, never the full Python suite;** `pytest ... -n 8` at most.
- `.venv/bin/python tools/check_parts.py` and `.venv/bin/python tools/check_licenses.py`: 0 problems (Task 1 after the new dependency,
  Task 11 at the end).
- No new dependency except `osqp` (exact pin, license recorded in `pyproject.toml`), and, only if Task 7 Step 6 triggers, Eigen
  (MPL-2.0, header-only) and OSQP's C library (Apache-2.0) fetched behind the opt-in CMake option `NAPL_LOCO_CPP`. No GPL; LGPL only
  dynamically linked.
- No media committed (no screenshots, video, plots). Generated reports are Markdown / JSON text.
- Files are edited only with the Edit / Write tools (no sed / heredoc / shell redirection into the tree). Results that come back
  from hfbox (`tools/remote.sh pull`) are read with Read and written into the tree with Write.
- Every existing test and golden state hash stays green; SP4 changes no kernel state code (Task 1 Step 6, if triggered, touches
  bindings only; Task 7 Step 6, if triggered, adds a separate opt-in module outside the kernel).
- Determinism: same seed -> same state hash; every harness / tool sets `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1` before numpy
  is imported (bitwise-repeatable BLAS); random draws from `np.random.default_rng([seed, <stream>])`.
- The controller (`examples/dog/loco/*`) never imports `napl`, `dogtruth` or `dogcourse` and never calls the Sim except through
  `loco/hw.py` (Review Focus 2).
- Every commit message ends with `Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5`.
- Recon robot only: no weapon, launcher or payload-release feature; the MC "payload" is a dispersion mass for robustness, nothing
  more.

## Review Focus

1. Leg kinematics across mirrored and 180 deg-turned legs: the controller's own FK / Jacobian (from the design) must equal the simulated dog for all four legs, or every foothold and force is silently wrong on two legs. Pinned by `tests/python/test_dog_model.py::test_fk_matches_the_sim_for_every_leg_in_random_poses` (Task 2).
2. Controller honesty: the controller reads only what the real dog has (Qdd `q dq tau_est temp fault voltage timeout`, IMU `gyro accel`) plus the two declared SP5 stand-ins; never MuJoCo truth, never the virtual FootContact. Pinned by `tests/python/test_dog_walk.py::test_controller_reads_only_hardware_attrs` (Task 6).
3. Forces and torques are physical end to end: the WBC, fed a real MPC solve, returns contact forces inside the friction pyramid (zero on swing feet, fz in [F_MIN, F_MAX] on stance feet), joint torques within the drivers' limits, and the MPC forces themselves whenever no limit binds (the MPC's own pyramid is checked in Task 4's `test_forces_inside_friction_pyramid_and_zero_in_swing`). Pinned by `tests/python/test_dog_wbc.py::test_wbc_keeps_mpc_forces_inside_friction_and_torque_limits` (Task 7).
4. Torque-based contact estimate agrees with the FootContact ground truth on >= 95 % of samples in a trot (the MPC, the estimator and the early / late touchdown logic all trust it). Pinned by `tests/python/test_dog_walk.py::test_contact_estimate_matches_foot_contact_truth` (Task 6).
5. MC honesty: verdicts from ground truth (not the controller's own fall detector), Wilson 95 % interval per family, lidar off enforced, held-out seeds. Pinned by `tests/python/test_dog_mc.py::test_summary_uses_truth_falls_wilson_and_refuses_lidar` (Task 11).

---

### Task 1: prerequisites, OSQP, the hardware door (`loco/hw.py`), ground truth, flat scene, compute budget

**Files:**
- Modify: `pyproject.toml` (extra `loco`), `tools/install_dev.sh` (`.[dev,px4,loco]`)
- Create: `examples/dog/loco/__init__.py`, `examples/dog/loco/hw.py`, `examples/dog/dogtruth.py`, `examples/dog/dogcourse.py`
  (base model cache, `strip_lidar`, flat scene), `tools/dog_loco_bench.py`, `docs/dog/locomotion.md` (section "Compute budget")
- Conditional (Step 6): `src/python/bindings.cpp`, `python/napl/__init__.py`
- Test: `tests/python/test_dog_loco_io.py`

**Interfaces:**
- Consumes: `dogsim.build`, `dogsim.LEGS`, `dogsim.JOINTS`; Qdd attrs; IMU attrs `gyro`, `accel`, `gyro_bias`, `accel_bias`;
  FootContact `contact`; free joint `qvel`; `napl.marine.write_native`.
- Produces:
  - `loco.hw`: `LEGS`, `JOINTS`, `QDD` (12 paths, leg-major), `IMU`, `HW_ATTRS`, `Hw(sim, audit=False)` with `.joints() -> (q, dq,
    tau) (12,)`, `.imu() -> (gyro, accel) (3,)`, `.health() -> (fault, v_bus_min, temp_max)`, `.enable()`, `.send(p_des, v_des, kp,
    kd, tau_ff)` (12-vectors), `.reads` (set of `(path, attr)` when audited).
  - `dogtruth`: `TRUNK = "/world/deck_top"`, `FREE = "/world/deck_top/deck_top:free"`, `Truth(sim, field=None)` with `.pos()`,
    `.quat()`, `.yaw()`, `.tilt_deg()`, `.vel()` (world linear), `.clearance()`, `.contacts() -> (4,) 0/1`, `.push(dv_world)`,
    `.arm_fall()`, `.fallen()`; `CLEAR_FRAC`, `TILT_FALL_DEG`.
  - `dogcourse`: `base_model() -> Path` (cached nominal dog MJCF; its report beside it as `dog.report.json`), `strip_lidar(root)`,
    `scene(out, course, seed=1, payload=None) -> Path` (course `"flat"` in this task; Task 8 adds the others), `info(model) -> dict`
    (the `course.json` beside the scene: `course`, `seed`, `nominal`, `start`, `route`, `field`).

- [ ] **Step 1: check the prerequisite.** `git log --oneline napl-v1 | grep -m1 "Merge p12-dog"` and
  `grep -n "^def \|^LEGS\|^JOINTS\|^STAND\|^SQUAT" examples/dog/dogsim.py`. No merge or a missing name: stop and report to the
  orchestrator. Re-read the SP3b names of Global Constraints in the merged `assemblies/dog/*.toml` (device and body names after
  flattening) and note any difference in the task report.

- [ ] **Step 2: OSQP dependency.** On hfbox, `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run 'PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache uv pip install --dry-run osqp 2>&1 | tail -20'`:
  take the osqp version uv resolves and pin it exactly. Read the `License` line of its METADATA and of each new transitive package
  (`.venv/lib/python3.12/site-packages/<pkg>-<ver>.dist-info/METADATA` after the install) and record them in the comment. Add to
  `[project.optional-dependencies]` of `pyproject.toml`:

```toml
# osqp (OSQP, Stellato et al. 2020; PyPI METADATA "License: Apache-2.0"): the convex MPC of the dog's locomotion (examples/dog/loco/mpc.py).
# Transitive: scipy (BSD-3-Clause) and the other packages uv resolved, each with its METADATA license line here. Python only, never linked into napl.
loco = ["osqp==<the version uv resolved, e.g. 1.0.5>"]
```

  (the `<...>` text is replaced by the resolved number before the commit; nothing else in this file changes) and in
  `tools/install_dev.sh` change `-e ".[dev,px4]"` to `-e ".[dev,px4,loco]"`.

- [ ] **Step 3: failing test** `tests/python/test_dog_loco_io.py`:

```python
"""SP4 Task 1: the controller's hardware door (loco/hw.py), the harness ground truth (dogtruth.py), the flat scene with the lidar removed
(dogcourse.py). Needs the built kernel for the Sim parts."""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C  # noqa: E402
import dogtruth as T  # noqa: E402
from loco import hw as H  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def devices(xml):
    return json.loads(ET.parse(xml).getroot().find("custom/text[@name='napl.devices']").get("data"))


def test_osqp_is_importable():
    import osqp  # noqa: F401  (the loco extra is installed)


def test_flat_scene_has_no_lidar_and_records_its_nominal_model(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    assert devices(model).get("rangefinders", []) == []  # spec s9 risk 1: locomotion runs never cast rays
    assert len(devices(C.base_model())["rangefinders"]) == 2  # the nominal dog keeps head:lidar and head:depth
    inf = C.info(model)
    assert inf["course"] == "flat" and Path(inf["nominal"]) == C.base_model()


@needs_kernel
def test_hw_reads_only_hardware_attrs_and_sends_mit_frames(tmp_path):
    sim = napl.Sim.load(C.scene(tmp_path, "flat", seed=1))
    hw = H.Hw(sim, audit=True)
    hw.enable()
    q, dq, tau = hw.joints()
    gyro, acc = hw.imu()
    hw.health()
    assert q.shape == dq.shape == tau.shape == (12,) and gyro.shape == acc.shape == (3,)
    hw.send(q, np.zeros(12), np.full(12, 20.0), np.full(12, 1.0), np.full(12, 0.5))
    sim.step(4)
    assert abs(float(sim.query(H.QDD[4], "tau_cmd")[0][0])) > 0.1  # the frame reached the driver (a test-side read, not the Hw's)
    assert all(p.startswith("/devices/") for p, _ in hw.reads)
    assert {a for _, a in hw.reads} <= H.HW_ATTRS


@needs_kernel
def test_truth_push_changes_the_velocity_through_the_bus(tmp_path):
    sim = napl.Sim.load(C.scene(tmp_path, "flat", seed=1))
    sim.start_log()
    tr = T.Truth(sim)
    sim.step(2)
    v0 = tr.vel()
    tr.push([0.0, 0.5, 0.0])
    assert tr.vel()[1] == pytest.approx(v0[1] + 0.5, abs=1e-9)
    assert tr.contacts().shape == (4,)
    sim.step(10)
    assert sim.log() is not None  # the push is a logged SetAttr: replay reproduces it
```

- [ ] **Step 4: run, expect FAIL** (no `loco`, no `dogcourse`):
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_loco_io.py -n 8'`

- [ ] **Step 5: implement.** `examples/dog/loco/__init__.py`: one docstring line ("The dog's locomotion software (SP4); imports
  nothing from napl: loco/hw.py is the only module that touches the robot."). `examples/dog/loco/hw.py`:

```python
"""The controller's only door to the robot (SP4): reads what the real dog's sensors report (12 Qdd drivers over CAN, the IMU) and writes
MIT-mode frames. Nothing else in examples/dog/loco touches the Sim. `reads` records every (path, attr) for the audit test (Review Focus 2)."""

import numpy as np

LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")
QDD = [f"/devices/{leg}:{j}" for leg in LEGS for j in JOINTS]  # leg-major: the order of every 12-vector in loco
IMU = "/devices/compute:imu"
HW_ATTRS = {"q", "dq", "tau_est", "temp", "fault", "voltage", "timeout", "gyro", "accel"}  # a real bus reports these


class Hw:
    def __init__(self, sim, audit=False):
        self.sim, self.reads = sim, (set() if audit else None)
        self._kp = self._kd = None
        self._many = getattr(sim, "set_many", None)  # Task 1 Step 6 (only if the bench asked for it)

    def _get(self, path, attr):
        if self.reads is not None:
            self.reads.add((path, attr))
        return self.sim.query(path, attr)[0]

    def joints(self):
        g = self._get
        return (np.array([g(p, "q")[0] for p in QDD]), np.array([g(p, "dq")[0] for p in QDD]),
                np.array([g(p, "tau_est")[0] for p in QDD]))  # fmt: skip

    def imu(self):
        return np.array(self._get(IMU, "gyro"), float), np.array(self._get(IMU, "accel"), float)

    def health(self):
        g = self._get
        return (max(g(p, "fault")[0] for p in QDD), min(g(p, "voltage")[0] for p in QDD), max(g(p, "temp")[0] for p in QDD))

    def enable(self):
        for p in QDD:
            self.sim.set(p, "enable", 1.0)

    def send(self, p_des, v_des, kp, kd, tau_ff):
        """One MIT frame per drive. kp / kd are written only when they change (the driver holds them; a real host repeats them in every
        frame, the bus here just logs fewer commands); p_des / v_des / tau_ff every call, which also feeds the drivers' CAN timeout."""
        cmds = [(p, "p_des", p_des[i]) for i, p in enumerate(QDD)] + [(p, "v_des", v_des[i]) for i, p in enumerate(QDD)]
        cmds += [(p, "tau_ff", tau_ff[i]) for i, p in enumerate(QDD)]
        if self._kp is None or np.any(kp != self._kp):
            cmds += [(p, "kp", kp[i]) for i, p in enumerate(QDD)]
            self._kp = np.array(kp, float)
        if self._kd is None or np.any(kd != self._kd):
            cmds += [(p, "kd", kd[i]) for i, p in enumerate(QDD)]
            self._kd = np.array(kd, float)
        if self._many:
            self._many([c[0] for c in cmds], [c[1] for c in cmds], [float(c[2]) for c in cmds])
        else:
            for path, attr, val in cmds:
                self.sim.set(path, attr, float(val))
```

  `examples/dog/dogtruth.py`:

```python
"""Ground truth of a dog run, for the HARNESS only (monitors, MC verdicts, the route follower, pushes, the SP5 pose-fix stand-in). Never
imported by examples/dog/loco (Review Focus 2)."""

import math

import numpy as np

TRUNK, FREE = "/world/deck_top", "/world/deck_top/deck_top:free"
FEET = [f"/devices/{leg}:foot" for leg in ("fl", "fr", "rl", "rr")]
TILT_FALL_DEG = 60.0  # trunk beyond 60 deg from upright: on its side
CLEAR_FRAC = 0.6  # trunk-origin clearance below 60 % of its standing value (armed after stand-up): belly on the ground


class Truth:
    def __init__(self, sim, field=None):
        self.sim, self.field, self.clear0 = sim, field, None

    def pos(self):
        return np.array(self.sim.query(TRUNK, "pos")[0], float)

    def quat(self):
        return np.array(self.sim.query(TRUNK, "quat")[0], float)  # w x y z

    def yaw(self):
        w, x, y, z = self.quat()
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def tilt_deg(self):
        w, x, y, z = self.quat()
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def vel(self):
        return np.array(self.sim.query(FREE, "qvel")[0][:3], float)  # MuJoCo free joint: world linear velocity of the body origin

    def ground(self, x, y):
        return 0.0 if self.field is None else float(self.field.height(x, y))

    def clearance(self):
        p = self.pos()
        return p[2] - self.ground(p[0], p[1])

    def contacts(self):
        return np.array([self.sim.query(f, "contact")[0][0] for f in FEET], float)

    def push(self, dv_world):
        qv = np.array(self.sim.query(FREE, "qvel")[0], float)
        qv[:3] += dv_world
        self.sim.set(FREE, "qvel", qv)

    def arm_fall(self):
        self.clear0 = self.clearance()

    def fallen(self):
        low = self.clear0 is not None and self.clearance() < CLEAR_FRAC * self.clear0
        return self.tilt_deg() > TILT_FALL_DEG or low
```

  `examples/dog/dogcourse.py` (this task: cache, lidar strip, flat; Task 8 adds fields, the criterion course, routes and the
  follower):

```python
"""Scenes of the dog's locomotion runs (SP4, HARNESS side): the nominal dog built once and cached, then per run a scene = the nominal MJCF
with the lidar and depth camera removed (sim budget, docs/models.md "Ground sim budget"), a course (flat floor / terrain field), the start
pose and an optional dispersion payload. course.json beside scene.xml records what the harness needs (the nominal model is what the
CONTROLLER's model is built from: it never sees the payload or the terrain)."""

import hashlib
import json
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import dogsim as D

REPO = Path(__file__).resolve().parents[2]
CACHE = Path(os.environ.get("NAPL_DOG_CACHE", Path(tempfile.gettempdir()) / "napl-dog-build"))


def _key():
    h = hashlib.sha256()
    for f in sorted([*(REPO / "assemblies" / "dog").glob("*.toml"), *(REPO / "parts").rglob("*.toml")]):
        h.update(f.read_bytes())
    return h.hexdigest()[:16]


def base_model():
    """Nominal dog MJCF (dogsim.build), cached by the content of assemblies/dog and parts; built in a temp dir and renamed into place, so
    concurrent MC workers never read a half-written build."""
    d = CACHE / _key()
    xml = d / "dog.xml"
    if not xml.exists():
        tmp = Path(tempfile.mkdtemp(dir=CACHE.mkdir(parents=True, exist_ok=True) or CACHE))
        x, rep = D.build(tmp)
        (tmp / "dog.report.json").write_text(json.dumps(rep))
        Path(x).rename(tmp / "dog.xml")
        try:
            tmp.rename(d)
        except OSError:  # another worker won the race: same content
            pass
    return xml


def strip_lidar(root):
    t = root.find("custom/text[@name='napl.devices']")
    d = json.loads(t.get("data"))
    d["rangefinders"] = []
    t.set("data", json.dumps(d))


def scene(out, course, seed=1, payload=None):
    """-> scene.xml path in `out`. course: "flat" (Task 8 adds criterion, slip, rubble, slopes, scree, ruins). payload = (kg, dx, dy) or None."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    nominal = base_model()
    root = ET.parse(nominal).getroot()
    strip_lidar(root)
    inf = dict(course=course, seed=int(seed), nominal=str(nominal), start=[0.0, 0.0, 0.0], route=None, field=None)
    if course != "flat":
        raise ValueError(f"course {course!r}: Task 8")
    if payload:
        _add_payload(root, *payload)
    path = out / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    (out / "course.json").write_text(json.dumps(inf))
    return path


def _add_payload(root, kg, dx, dy):
    trunk = next(b for b in root.iter("body") if b.get("name") == "deck_top")
    b = ET.SubElement(trunk, "body", name="payload", pos=f"{dx:.6g} {dy:.6g} 0.03")
    ET.SubElement(b, "geom", type="box", size="0.03 0.03 0.02", mass=f"{kg:.6g}", contype="0", conaffinity="0", group="1")


def info(model):
    return json.loads((Path(model).parent / "course.json").read_text())
```

  `tools/dog_loco_bench.py` (prints a Markdown table; `--sim 2.0` sim seconds): flat scene; rows: (a) physics only under a held
  stand PD (dogsim's `KP`, `KD`, no calls), (b) + `Hw.joints()` + `Hw.imu()` every 2 ticks, (c) + `Hw.send` every 2 ticks with
  p_des / v_des / tau_ff changing each call, kp / kd constant, (d) the same with kp / kd changing every call; columns: sim-s per
  wall-s, and per control tick: read us, write us. Then the decision line `I/O per control tick: X ms (rule: > 1.0 ms -> Step 6)`.

- [ ] **Step 6: measure, decide, and (only if the rule fires) batch writes.**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/python tools/dog_loco_bench.py --sim 2.0'`.
  Write the table and the decision into `docs/dog/locomotion.md` section "Compute budget" (Write; the file starts with a one-paragraph
  summary of the SP4 architecture and the rates). **If the I/O per control tick (row c, read + write) exceeds 1.0 ms**, add to
  `bind_common` in `src/python/bindings.cpp` (next to `raycast`):

```cpp
      // Batch of single-value SetAttrs (SP4 dog leg control, 36-60 per 2 ms tick): one Envelope per entry through the same command bus as
      // call(), so each is logged and replayed exactly like a JSON SetAttr; only the JSON round trip is skipped. Stops at the first failure.
      .def("set_many",
           [](T& s, const std::vector<std::string>& paths, const std::vector<std::string>& attrs, const std::vector<double>& values) {
             if (paths.size() != attrs.size() || paths.size() != values.size()) throw nb::value_error("set_many: lengths differ");
             for (std::size_t i = 0; i < paths.size(); ++i) {
               flatbuffers::FlatBufferBuilder fbb;
               const double v = values[i];
               fbb.Finish(napl::CreateEnvelope(fbb, 0, fbb.CreateString("python"), napl::Command::SetAttr,
                                               napl::CreateSetAttr(fbb, fbb.CreateString(paths[i]), fbb.CreateString(attrs[i]),
                                                                   fbb.CreateString(""), fbb.CreateVector(&v, 1)).Union()));
               read_command(s, fbb, [](const napl::Reply&) { return 0; });
             }
           },
           "paths"_a, "attrs"_a, "values"_a)
```

  and to `napl.Sim` in `python/napl/__init__.py` (after `set`):

```python
    def set_many(self, paths, attrs, values) -> None:
        """Batch of single-value SetAttrs through the bus (logged and replayed exactly like set()); raises on the first failure."""
        self._c.set_many(list(paths), list(attrs), [float(v) for v in values])
```

  with this test appended to `tests/python/test_dog_loco_io.py`:

```python
@needs_kernel
def test_set_many_is_set_in_a_loop(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    a, b = napl.Sim.load(model), napl.Sim.load(model)
    paths = [p for p in H.QDD for _ in range(2)]
    attrs = ["enable", "tau_ff"] * 12
    vals = [1.0, 0.7] * 12
    a.set_many(paths, attrs, vals)
    for p, at, v in zip(paths, attrs, vals):
        b.set(p, at, v)
    a.step(20)
    b.step(20)
    assert a.state_hash() == b.state_hash()
    with pytest.raises(ValueError):
        a.set_many(["/devices/fl:hip"], ["kp"], [-1.0])
```

  then rebuild (Global Constraints build line), rerun the bench, and record both rows in "Compute budget". Under the rule, skip
  this paragraph and say "not needed (X ms)" in the doc.
- [ ] **Step 7: run, expect PASS + 0 problems:**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_loco_io.py tests/python/test_dog_stand.py -n 8 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
  (`test_dog_stand.py`: the SP3b golden stays green). If Step 6 changed bindings, add `tests/python/test_program.py` to the run.
- [ ] **Step 8: commit** `git add pyproject.toml tools/install_dev.sh tools/dog_loco_bench.py examples/dog/loco examples/dog/dogtruth.py examples/dog/dogcourse.py docs/dog/locomotion.md tests/python/test_dog_loco_io.py` (plus `src/python/bindings.cpp python/napl/__init__.py` if Step 6 ran) `&& git commit -m "dog loco: hardware door (Qdd/IMU only), ground truth, flat scene without lidar, osqp extra, compute budget" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 2: the controller's own model (`loco/model.py`) from the design

**Files:**
- Create: `examples/dog/loco/model.py`
- Test: `tests/python/test_dog_model.py`

**Interfaces:**
- Consumes: `dogcourse.base_model()` (MJCF with `napl.devices` JSON: `qdds[*].name, joint, tau_ff_max, kp_max, kd_max`, `imus[*].name,
  site`); the report beside it (`mass.total`).
- Produces: `loco.model`: `Leg` (fields `name, w (3,3), p (3,3), foot0 (3,), r_foot, q_lo (3,), q_hi (3,), tau_max (3,), kp_max (3,),
  kd_max (3,), bend`; methods `fk_jac(q3) -> (x (3,), J (3,3))`, `knee(q3) -> (3,)`, `ik(x, q0) -> q3`), `Model` (fields `mass,
  com (3,), inertia (3,3) about the CoM, legs: dict, imu_R (3,3) trunk<-imu, leg_mass`; methods `stand_q(frac) -> (12,)`,
  `feet(q12) -> (4,3)`), `from_build(xml) -> Model`. All in the trunk (`deck_top` body) frame.

- [ ] **Step 1: failing test** `tests/python/test_dog_model.py`:

```python
"""SP4 Task 2: the controller's model from the design. FK / Jacobian of all four legs (two mirrored, two turned 180 deg) against the simulated
dog (Review Focus 1), mass against the build report, IK round trip, stand pose with the designed knee directions."""

import json
import sys
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogcourse as C  # noqa: E402
from loco import model as M  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")


@pytest.fixture(scope="module")
def m():
    return M.from_build(C.base_model())


def prim(sim, name):
    prims = sim.call({"cmd_type": "Query", "cmd": {"path": ""}})["prims"]
    return next(p for p in prims if p.rsplit("/", 1)[-1] == name)


def test_mass_matches_the_build_report(m):
    rep = json.loads((C.base_model().parent / "dog.report.json").read_text())
    assert m.mass == pytest.approx(rep["mass"]["total"], rel=0.01)


def test_jacobian_is_the_derivative_of_fk(m):
    rng = np.random.default_rng(2)
    for leg in LEGS:
        L = m.legs[leg]
        q = rng.uniform(0.6 * L.q_lo, 0.6 * L.q_hi)
        x, J = L.fk_jac(q)
        for j in range(3):
            dq = np.zeros(3)
            dq[j] = 1e-6
            assert np.allclose((L.fk_jac(q + dq)[0] - x) / 1e-6, J[:, j], atol=1e-5)


def test_ik_round_trip_and_designed_knee_directions(m):
    q = m.stand_q(0.82)
    feet = m.feet(q)
    for i, leg in enumerate(LEGS):
        L = m.legs[leg]
        assert np.linalg.norm(feet[i, :2] - L.foot0[:2]) < 2e-3  # foot under its build-pose spot (under the hip)
        knee_ahead = L.knee(q[3 * i : 3 * i + 3])[0] > L.p[1][0]
        assert knee_ahead == leg.startswith("r")  # spec / SP3b: front knees point backward, rear knees forward
        assert np.allclose(L.fk_jac(L.ik(feet[i], q[3 * i : 3 * i + 3] + 0.05))[0], feet[i], atol=1e-6)


@needs_kernel
def test_fk_matches_the_sim_for_every_leg_in_random_poses(m):  # Review Focus 1
    sim = napl.Sim.load(C.base_model())
    trunk = prim(sim, "deck_top")
    rng = np.random.default_rng(3)
    for _ in range(5):
        s = sim.fork()
        q = np.concatenate([rng.uniform(0.7 * m.legs[l].q_lo, 0.7 * m.legs[l].q_hi) for l in LEGS])
        for i, leg in enumerate(LEGS):
            for j, jn in enumerate(JOINTS):
                s.set(prim(s, f"{leg}:{jn}_j"), "qpos", [q[3 * i + j]])
        for i, leg in enumerate(LEGS):
            got = np.array(s.query(prim(s, f"{leg}:foot_ball"), "pos", trunk)[0])
            assert np.allclose(m.feet(q)[i], got, atol=1e-6), (leg, m.feet(q)[i], got)
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_model.py -n 8'`
- [ ] **Step 3: implement `examples/dog/loco/model.py`:**

```python
"""The controller's own model of the dog (SP4), from the DESIGN: the built MJCF's static body tree at q = 0 (body / joint / site poses, geom
masses, as the napl builder wrote them from the part files) and the napl.devices limits. No mjData, no runtime sim query.
Kinematics: product of exponentials (Lynch & Park, Modern Robotics ch. 4): one formula for all four legs, mirrored and turned legs included,
because every joint axis and point comes from the design. Inertia: geom masses as points plus each box geom's own inertia (an approximation:
mesh and cylinder geoms as points, ~10-20 % on the trunk inertia; the MPC tolerates it, the MC disperses the plant around it)."""

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")
ROOT = "deck_top"


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])  # fmt: skip


def rot(w, th):
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def vec(s, d):
    return np.array([float(v) for v in s.split()]) if s else np.array(d, float)


@dataclass
class Leg:
    name: str
    w: np.ndarray
    p: np.ndarray
    foot0: np.ndarray
    r_foot: float
    q_lo: np.ndarray
    q_hi: np.ndarray
    tau_max: np.ndarray
    kp_max: np.ndarray
    kd_max: np.ndarray
    bend: float = 1.0  # sign of the knee seed that bends this leg the designed way (set by from_build)

    def _chain(self, q, upto=3):
        R, t, ws, ps = np.eye(3), np.zeros(3), [], []
        for i in range(upto):
            ws.append(R @ self.w[i])
            ps.append(R @ self.p[i] + t)
            Ri = rot(self.w[i], q[i])
            t = R @ (self.p[i] - Ri @ self.p[i]) + t  # g <- g o (x -> p_i + R_i (x - p_i))
            R = R @ Ri
        return R, t, ws, ps

    def fk_jac(self, q):
        R, t, ws, ps = self._chain(q)
        x = R @ self.foot0 + t
        return x, np.column_stack([np.cross(w, x - p) for w, p in zip(ws, ps)])

    def knee(self, q):
        R, t, _, _ = self._chain(q, 2)
        return R @ self.p[2] + t

    def ik(self, x_des, q0, iters=6):
        q = np.array(q0, float)
        for _ in range(iters):
            x, J = self.fk_jac(q)
            q += np.linalg.solve(J.T @ J + 1e-6 * np.eye(3), J.T @ (x_des - x))
        return np.clip(q, self.q_lo, self.q_hi)


@dataclass
class Model:
    mass: float
    com: np.ndarray
    inertia: np.ndarray
    legs: dict
    imu_R: np.ndarray
    leg_mass: float
    q_prev: dict = field(default_factory=dict)

    def feet(self, q):
        return np.array([self.legs[l].fk_jac(q[3 * i : 3 * i + 3])[0] for i, l in enumerate(LEGS)])

    def stand_q(self, frac):
        """12 joint targets: each foot raised straight up from its build-pose spot to `frac` of the hip-pitch-to-foot distance."""
        out = []
        for l in LEGS:
            L = self.legs[l]
            d = np.linalg.norm(L.foot0 - L.p[1])
            seed = np.array([0.0, -0.5 * L.bend, 1.0 * L.bend])
            out.append(L.ik(L.foot0 + np.array([0.0, 0.0, (1 - frac) * d]), seed, iters=20))
        return np.concatenate(out)
```

  `from_build(xml)`: parse; walk `worldbody` composing each body's `pos` / `quat` from the `deck_top` body (its own pose is the
  identity of the trunk frame); per leg take the `joint` named `<leg>:<j>_j` in its body (`pos` default 0, `axis` default `0 0 1`,
  `range`) into `w`, `p`, `q_lo`, `q_hi`; `foot0` = the `<leg>:foot_ball` body origin, `r_foot` = its sphere geom radius (raise if
  that geom's centre is more than 1 mm from the body origin: the FK point must be the ball centre); masses = every geom `mass`
  inside `deck_top` at its geom centre, box geoms add `m (b^2 + c^2) / 3` per axis (half sizes); `mass`, `com`, `inertia` (parallel
  axis about the CoM) over the whole dog at q = 0; `leg_mass` = mass of the bodies below `<leg>:abad_j` (mean of the four); limits
  per joint from `napl.devices` `qdds` (`<leg>:<j>`: `tau_ff_max`, `kp_max`, `kd_max`); `imu_R` = the rotation of the IMU's site
  (`imus[0]["site"]`) in the trunk frame. `bend`: for s in (+1, -1) solve `ik(foot0 + (0, 0, 0.18 d), (0, -0.5 s, s))` and keep the s
  whose knee point lies behind the hip pitch point (`knee[0] < p[1][0]`) for `fl` / `fr` and ahead of it for `rl` / `rr`; raise if
  neither does.
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_model.py -n 8'`.
  A failure of the sim comparison on two legs only is a mirror / turn bug in `from_build`'s frame composition: fix it there, never
  loosen the 1e-6 tolerance.
- [ ] **Step 5: commit** `git add examples/dog/loco/model.py tests/python/test_dog_model.py && git commit -m "dog loco: controller model from the design (PoE kinematics for mirrored/turned legs, mass and inertia, device limits)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 3: estimator (`loco/estimator.py`) + height-map protocol (`loco/terrain.py`, maps only)

**Files:**
- Create: `examples/dog/loco/estimator.py`, `examples/dog/loco/terrain.py` (this task: `HeightMap`, `FlatMap`, `TruthMap`)
- Test: `tests/python/test_dog_estimator.py`

**Interfaces:**
- Consumes: `loco.model.Model` (`legs[*].fk_jac`, `imu_R`), a `HeightMap`.
- Produces:
  - `loco.terrain`: `HeightMap` (Protocol: `height(x, y) -> ndarray`, nan = unknown), `FlatMap(z=0.0)`, `TruthMap(field)` (wraps
    `napl.terrain.Field.height`; the field object is passed in by the harness, `loco` never imports napl).
  - `loco.estimator`: `F_ON`, `F_OFF`, `KP_ACC`, `State` (fields `p, v, R, rpy, w, feet (4,3), contact (4,) bool, f_ground (4,3),
    h_map`), `Estimator(model, hmap, dt=0.002)` with `.reset(q)`, `.update(q, dq, tau, gyro, acc, gate=None) -> State`
    (`gate` (4,) bool: legs in the first half of a scheduled swing, never in contact), `.fix(p_xy, yaw, age_s, sig_p, sig_yaw)`.

- [ ] **Step 1: failing test** `tests/python/test_dog_estimator.py`:

```python
"""SP4 Task 3: estimator. Pure-math checks on synthetic signals (attitude convergence, the KF on a fake one-joint-per-axis leg) and on the
simulated dog under a PD squat / stand (no gait yet: Task 6 checks the trot and the contact estimate)."""

import math
import sys
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogcourse as C  # noqa: E402
import dogtruth as T  # noqa: E402
from loco import estimator as E  # noqa: E402
from loco import hw as H  # noqa: E402
from loco import model as M  # noqa: E402
from loco import terrain as TR  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


class PointLeg:  # foot = q (trunk frame), J = I: lets the KF be checked on exact kinematics
    def fk_jac(self, q):
        return np.array(q, float), np.eye(3)


class FakeModel:
    imu_R = np.eye(3)
    legs = {l: PointLeg() for l in ("fl", "fr", "rl", "rr")}


FEET = np.array([[0.25, 0.15, -0.35], [0.25, -0.15, -0.35], [-0.25, 0.15, -0.35], [-0.25, -0.15, -0.35]])


def test_attitude_converges_to_a_static_tilt():
    est = E.Estimator(FakeModel(), TR.FlatMap())
    est.reset(FEET.ravel())
    th = math.radians(10)
    g_body = np.array([0.0, math.sin(th), math.cos(th)]) * 9.81  # specific force of a trunk rolled 10 deg, at rest
    for _ in range(2500):  # 5 s at 500 Hz
        st = est.update(FEET.ravel(), np.zeros(12), np.zeros(12), np.zeros(3), g_body)
    assert math.degrees(st.rpy[0]) == pytest.approx(10.0, abs=0.5)


def test_kf_tracks_a_body_moving_over_fixed_feet():
    est = E.Estimator(FakeModel(), TR.FlatMap(-0.35))
    est.reset(FEET.ravel())
    v = np.array([0.5, 0.1, 0.0])
    dt = 0.002
    for k in range(1000):
        p = v * k * dt
        r = (FEET - p).ravel()  # feet fixed in the world, body moving
        st = est.update(r, np.tile(-v, 4), np.tile([0, 0, -30.0], 4), np.zeros(3), np.array([0, 0, 9.81]))
    assert np.allclose(st.v, v, atol=0.01) and np.allclose(st.p, p, atol=0.01)


@needs_kernel
def test_estimator_tracks_a_squat_and_stand(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    sim = napl.Sim.load(model)
    m = M.from_build(C.info(model)["nominal"])
    hw, tr = H.Hw(sim), T.Truth(sim)
    est = E.Estimator(m, TR.FlatMap())
    hw.enable()
    q0 = hw.joints()[0]
    est.reset(q0)
    q_st, q_sq = m.stand_q(0.82), m.stand_q(0.65)
    errs, tilts, agree = [], [], []
    for k in range(2000):  # 4 s: 1 s to stand, 1.5 s squat, 1.5 s stand
        t = k * 0.002
        a = min(1.0, t) if t < 1.0 else (min(1.0, (t - 1.0) / 1.0) if t < 2.5 else 1 - min(1.0, (t - 2.5) / 1.0))
        tgt = q0 + (q_st - q0) * a if t < 1.0 else q_st + (q_sq - q_st) * a
        hw.send(tgt, np.zeros(12), np.full(12, 60.0), np.full(12, 1.5), np.zeros(12))
        sim.step(2)
        q, dq, tau = hw.joints()
        st = est.update(q, dq, tau, *hw.imu())
        if t > 1.5:
            errs.append(np.linalg.norm(st.v - tr.vel()))
            tilts.append(abs(math.degrees(math.acos(min(1.0, st.R[2, 2]))) - tr.tilt_deg()))
            agree.append(np.all(st.contact == tr.contacts().astype(bool)))
    assert np.sqrt(np.mean(np.square(errs))) < 0.03
    assert max(tilts) < 1.0
    assert np.mean(agree) > 0.99
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_estimator.py -n 8'`
- [ ] **Step 3: implement.** `examples/dog/loco/terrain.py` (maps part):

```python
"""Terrain side of the dog's controller (SP4): the height-map interface SP5's elevation map will implement, the ground-truth stand-in, and
(Task 8) footholds, posture plane, step height."""

from typing import Protocol

import numpy as np


class HeightMap(Protocol):
    def height(self, x, y) -> np.ndarray: ...  # world z, numpy-broadcast; nan where the map has no cell


class FlatMap:
    def __init__(self, z=0.0):
        self.z = float(z)

    def height(self, x, y):
        return np.full(np.broadcast(np.asarray(x), np.asarray(y)).shape, self.z)


class TruthMap:
    """SP5 stand-in: the terrain's exact ground truth (napl.terrain Field.height: MuJoCo's hfield triangulation, box tops)."""

    def __init__(self, field):
        self.f = field

    def height(self, x, y):
        return np.asarray(self.f.height(np.asarray(x, float), np.asarray(y, float)), float)
```

  `examples/dog/loco/estimator.py`:

```python
"""State estimator of the dog (SP4), 500 Hz. Inputs: the drivers and the IMU (loco.hw) and two SP5 stand-ins (height map, pose fix).
1. Attitude: Mahony complementary filter (Mahony, Hamel, Pflimlin 2008) on gyro + accelerometer; the gravity correction is used only while
   |a| is within 15 % of g (not in flight phases). Yaw is free (no magnetometer): only a pose fix corrects it.
2. Linear Kalman filter on x = [p, v, p_foot x 4] (18, world frame; Bledt, Powell, Katz, Di Carlo, Wensing, Kim, IROS 2018, MIT Cheetah 3):
   process v += (R a + g) dt, stance feet still, swing feet free; measurements per foot in contact: p_foot - p = R r_i(q) and
   v = -R (J_i dq + w x r_i), foot height = map height under it; pose fix p_xy (with its age, against the estimate of that time).
3. Contact: ground force from the drivers' own torque estimate f_i = -R (J_i^T)^-1 tau_i (leg dynamics neglected: ~1.5 kg legs swing at
   < 15 N equivalent); contact when f_z > F_ON, released below F_OFF; `gate` forces no-contact early in a scheduled swing."""

import math

import numpy as np

G = 9.81
F_ON, F_OFF = 30.0, 15.0  # N: the thresholds of the virtual FootContact of leg.toml (25 % / 12 % of the 29 N static per-foot load)
KP_ACC = 1.0  # rad/s per rad: Mahony k_P, ~1 s tilt correction
SIG_A, SIG_FOOT_ST, SIG_FOOT_SW = 0.3, 0.002, 1.0  # process: accel (m/s^2), stance / swing foot random walk (m/sqrt(s))
SIG_KIN, SIG_VEL, SIG_MAPZ = 0.005, 0.05, 0.02  # measurement: leg position (m), leg velocity (m/s), map height under a stance foot (m)
HIST = 64  # estimate history for pose fixes up to 0.128 s old


def rpy_of(R):
    return np.array([math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])])


def rot(w, th):
    n = np.linalg.norm(w)
    if n * th < 1e-12:
        return np.eye(3)
    k = w / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    a = n * th
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


class State:
    __slots__ = ("p", "v", "R", "rpy", "w", "feet", "contact", "f_ground", "h_map")


class Estimator:
    def __init__(self, model, hmap, dt=0.002):
        self.m, self.map, self.dt = model, hmap, dt
        self.legs = list(model.legs.values())
        self.R, self.x, self.P = np.eye(3), np.zeros(18), np.eye(18) * 1e-4
        self.contact = np.ones(4, bool)
        self.hist, self.k = np.zeros((HIST, 3)), 0  # (x, y, yaw) per tick

    def reset(self, q):
        r = np.array([L.fk_jac(q[3 * i : 3 * i + 3])[0] for i, L in enumerate(self.legs)])
        z = float(np.nanmean(self.map.height(r[:, 0], r[:, 1]))) - (r[:, 2] - self.m.legs["fl"].r_foot).mean()  # feet on the map
        self.x[:] = 0.0
        self.x[2] = z
        self.x[6:] = (r + self.x[:3]).ravel()
        self.P = np.eye(18) * 1e-4

    def update(self, q, dq, tau, gyro, acc, gate=None):
        dt, R = self.dt, self.R
        w, a = self.m.imu_R @ gyro, self.m.imu_R @ acc
        na = np.linalg.norm(a)
        err = np.cross(a / na, R.T @ [0, 0, 1]) if abs(na - G) < 0.15 * G else np.zeros(3)
        self.R = R = R @ rot(w + KP_ACC * err, dt)
        if self.k % 100 == 0:
            u, _, vt = np.linalg.svd(R)
            self.R = R = u @ vt
        # ---- KF predict
        F = np.eye(18)
        F[0:3, 3:6] = np.eye(3) * dt
        self.x = F @ self.x
        self.x[3:6] += (R @ a - [0, 0, G]) * dt
        Qd = np.zeros(18)
        Qd[3:6] = (SIG_A * dt) ** 2
        Qd[0:3] = (SIG_A * dt * dt) ** 2
        # ---- contact from the drivers' torque
        r, J = zip(*(L.fk_jac(q[3 * i : 3 * i + 3]) for i, L in enumerate(self.legs)))
        fg = np.array([-R @ np.linalg.solve(J[i].T, tau[3 * i : 3 * i + 3]) for i in range(4)])
        on = np.where(self.contact, fg[:, 2] > F_OFF, fg[:, 2] > F_ON)
        if gate is not None:
            on &= ~np.asarray(gate, bool)
        self.contact = on
        for i in range(4):
            Qd[6 + 3 * i : 9 + 3 * i] = (SIG_FOOT_ST if on[i] else SIG_FOOT_SW) ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag(Qd)
        # ---- KF update: stacked rows of the feet in contact
        H, y, Rn = [], [], []
        for i in np.flatnonzero(on):
            s = slice(6 + 3 * i, 9 + 3 * i)
            h = np.zeros((3, 18))
            h[:, s], h[:, 0:3] = np.eye(3), -np.eye(3)
            H.append(h); y.append(R @ r[i] - (self.x[s] - self.x[0:3])); Rn += [SIG_KIN**2] * 3  # fmt: skip
            h = np.zeros((3, 18))
            h[:, 3:6] = np.eye(3)
            H.append(h); y.append(-R @ (J[i] @ dq[3 * i : 3 * i + 3] + np.cross(w, r[i])) - self.x[3:6]); Rn += [SIG_VEL**2] * 3  # fmt: skip
            zf = float(self.map.height(self.x[s][0], self.x[s][1])) + self.m.legs["fl"].r_foot
            if np.isfinite(zf):
                h = np.zeros((1, 18))
                h[0, s.start + 2] = 1.0
                H.append(h); y.append([zf - self.x[s.start + 2]]); Rn.append(SIG_MAPZ**2)  # fmt: skip
        if H:
            self._correct(np.vstack(H), np.concatenate(y), np.array(Rn))
        self.hist[self.k % HIST] = (self.x[0], self.x[1], rpy_of(self.R)[2])
        self.k += 1
        st = State()
        st.p, st.v, st.R, st.rpy, st.w = self.x[0:3].copy(), self.x[3:6].copy(), self.R.copy(), rpy_of(self.R), w
        st.feet, st.contact, st.f_ground = self.x[6:].reshape(4, 3).copy(), on.copy(), fg
        st.h_map = st.p[2] - float(np.nanmean(self.map.height(st.feet[:, 0], st.feet[:, 1])))
        return st

    def _correct(self, H, y, rn):
        S = H @ self.P @ H.T + np.diag(rn)
        K = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + K @ y
        self.P = (np.eye(18) - K @ H) @ self.P

    def fix(self, p_xy, yaw, age_s, sig_p, sig_yaw):
        """Pose fix (SP5 stand-in, 10 Hz): compared with this estimator's own pose `age_s` ago; the innovation is applied now (xy through the
        KF, yaw as a rotation about world z weighted by the variances)."""
        lag = min(HIST - 1, int(round(age_s / self.dt)))
        x0, y0, yaw0 = self.hist[(self.k - 1 - lag) % HIST]
        h = np.zeros((2, 18))
        h[0, 0], h[1, 1] = 1.0, 1.0
        self._correct(h, np.array([p_xy[0] - x0, p_xy[1] - y0]), np.full(2, sig_p**2))
        dyaw = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi
        self.R = rot(np.array([0.0, 0.0, 1.0]), 0.5 * dyaw) @ self.R  # ponytail: fixed 0.5 blend, a yaw state in the KF if SP5 needs better
```

- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_estimator.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/loco/estimator.py examples/dog/loco/terrain.py tests/python/test_dog_estimator.py && git commit -m "dog loco: estimator (Mahony attitude + leg-inertial KF, torque-based contact, pose-fix input) and height-map interface" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 4: convex MPC (`loco/mpc.py`)

**Files:**
- Create: `examples/dog/loco/mpc.py`
- Test: `tests/python/test_dog_mpc.py`

**Interfaces:**
- Consumes: `osqp` (Task 1), scipy (its dependency: `scipy.sparse`, `scipy.linalg.expm`).
- Produces: `loco.mpc`: `N = 12`, `DT = 0.025`, `NX = 13`, `NU = 12`, `G`, `Q_DIAG`, `ALPHA`, `F_MIN`, `F_MAX`, `rz(yaw)`,
  `Mpc(mass, inertia_body)` with `.solve(x0, xref, r_feet, contact, mu) -> (f (4,3) world, ok: bool)` and `.n_fail`;
  `x = [roll, pitch, yaw, p (CoM, world), w (world), v, -G]`.

- [ ] **Step 1: failing test** `tests/python/test_dog_mpc.py` (pure Python):

```python
"""SP4 Task 4: convex MPC. Weight carried evenly at a stand, physical forces with feet in swing and a braking velocity (the WBC test of Review Focus 3 builds on it),
the solve-time budget of a 50 Hz loop."""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "dog"))
from loco import mpc  # noqa: E402

M, IB = 12.0, np.diag([0.08, 0.25, 0.28])  # test numbers of a 12 kg dog trunk (kg, kg m^2)
FEET = np.array([[0.25, 0.15, -0.35], [0.25, -0.15, -0.35], [-0.25, 0.15, -0.35], [-0.25, -0.15, -0.35]])


def problem(contact=(1, 1, 1, 1), v0=(0.0, 0.0, 0.0)):
    ref = np.array([0, 0, 0, 0, 0, 0.35, 0, 0, 0, 0, 0, 0, -mpc.G], float)
    x0 = ref.copy()
    x0[9:12] = v0
    return x0, np.tile(ref, (mpc.N, 1)), np.tile(FEET, (mpc.N, 1, 1)), np.tile(np.array(contact, float), (mpc.N, 1))


def test_standing_forces_carry_the_weight_evenly():
    f, ok = mpc.Mpc(M, IB).solve(*problem(), mu=np.full(4, 0.5))
    assert ok
    assert f[:, 2].sum() == pytest.approx(M * mpc.G, rel=0.02)
    assert np.allclose(f[:, 2], M * mpc.G / 4, rtol=0.1) and np.abs(f[:, :2]).max() < 1.0


def test_forces_inside_friction_pyramid_and_zero_in_swing():
    x0, xref, r, c = problem(contact=(1, 0, 0, 1), v0=(0.0, 0.6, 0.0))
    c[6:] = [0, 1, 1, 0]  # the other diagonal takes over mid-horizon
    mu = np.full(4, 0.4)
    f, ok = mpc.Mpc(M, IB).solve(x0, xref, r, c, mu)
    assert ok
    assert np.abs(f[[1, 2]]).max() < 1e-3  # swing feet push nothing
    for i in (0, 3):
        assert np.abs(f[i, :2]).max() <= mu[i] * f[i, 2] + 1e-3
        assert mpc.F_MIN - 1e-3 <= f[i, 2] <= mpc.F_MAX + 1e-3
    assert f[:, 1].sum() < 0.0  # the lateral velocity is braked


def test_solve_time_budget():
    m, rng, ts = mpc.Mpc(M, IB), np.random.default_rng(1), []
    for k in range(100):
        x0, xref, r, c = problem(contact=(1, 0, 0, 1) if k % 2 else (0, 1, 1, 0), v0=rng.uniform(-1, 1, 3) * [2, 0.5, 0.2])
        t0 = time.perf_counter()
        m.solve(x0, xref, r, c, np.full(4, 0.6))
        ts.append(time.perf_counter() - t0)
    print(f"MPC solve: median {np.median(ts) * 1e3:.2f} ms, max {max(ts) * 1e3:.2f} ms")
    assert np.median(ts) < 0.008  # 40 % of the 20 ms MPC period (50 Hz)
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_mpc.py -n 8 -s'`
- [ ] **Step 3: implement `examples/dog/loco/mpc.py`:**

```python
"""Convex MPC on the single-rigid-body model (Di Carlo, Wensing, Katz, Bledt, Kim, "Dynamic Locomotion in the MIT Cheetah 3 Through Convex
Model-Predictive Control", IROS 2018), solved by OSQP (Apache-2.0) in sparse form with a FIXED sparsity pattern (explicit zeros kept), so every
solve is an `update` of values with a warm start. World frame; decision variables = ground reaction forces on the 4 feet over the horizon."""

import numpy as np
import osqp
import scipy.sparse as sp
from scipy.linalg import expm

N, DT = 12, 0.025  # horizon 12 x 25 ms = 0.30 s ~ one trot period; re-solved at 50 Hz (every 10 leg-control ticks)
NX, NU, G = 13, 12, 9.81
# MIT cheetah-software convexMPC weights (Mini Cheetah, 9 kg: same class): rpy, p, w, v, g; retuned only on TRAIN seeds (Task 11) if needed
Q_DIAG = np.array([0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.3, 0.2, 0.2, 0.1, 0.0])
ALPHA = 4e-5
F_MIN, F_MAX = 2.0, 300.0  # N per stance foot: keep it loaded; above the 257 N study peak (docs/dog/actuator-study.md candidate 1)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def rz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Mpc:
    def __init__(self, mass, inertia_body):
        self.m, self.Ib = float(mass), np.asarray(inertia_body, float)
        self.nz = N * (NX + NU)
        rows, cols = self._pattern()
        A0 = sp.coo_matrix((np.arange(1, len(rows) + 1, dtype=float), (rows, cols)), shape=(self.nc, self.nz)).tocsc()
        self.perm = (A0.data - 1).astype(int)  # CSC data slot -> pattern entry
        self.A = A0
        self.P = sp.diags(np.concatenate([np.tile(Q_DIAG, N), np.full(N * NU, ALPHA)])).tocsc()
        self.prob, self.n_fail = None, 0

    def _pattern(self):
        r, c = [], []

        def block(r0, c0, nr, nc):
            rr, cc = np.meshgrid(np.arange(nr), np.arange(nc), indexing="ij")
            r.append((r0 + rr).ravel())
            c.append((c0 + cc).ravel())

        for k in range(N):  # x_{k+1} at columns k*NX, u_k at N*NX + k*NU
            block(k * NX, k * NX, NX, NX)
            if k:
                block(k * NX, (k - 1) * NX, NX, NX)
            block(k * NX, N * NX + k * NU, NX, NU)
        r0 = N * NX
        for k in range(N):
            for i in range(4):
                block(r0, N * NX + k * NU + 3 * i, 5, 3)  # 4 friction-pyramid rows + the fz row
                r0 += 5
        self.nc = r0
        return np.concatenate(r), np.concatenate(c)

    def _ab(self, yaw, r):
        Rz = rz(yaw)
        Iw_inv = np.linalg.inv(Rz @ self.Ib @ Rz.T)
        A = np.zeros((NX, NX))
        A[0:3, 6:9], A[3:6, 9:12], A[11, 12] = Rz.T, np.eye(3), 1.0
        B = np.zeros((NX, NU))
        for i in range(4):
            B[6:9, 3 * i : 3 * i + 3] = Iw_inv @ skew(r[i])
            B[9:12, 3 * i : 3 * i + 3] = np.eye(3) / self.m
        return A, B

    def solve(self, x0, xref, r_feet, contact, mu):
        Ads, Bds = [], []
        for k in range(N):
            A, B = self._ab(xref[k, 2], r_feet[k])
            Md = expm(np.block([[A, B], [np.zeros((NU, NX + NU))]]) * DT)  # zero-order hold
            Ads.append(Md[:NX, :NX])
            Bds.append(Md[:NX, NX:])
        vals = []
        for k in range(N):
            vals += [np.eye(NX).ravel()] + ([(-Ads[k]).ravel()] if k else []) + [(-Bds[k]).ravel()]
        for k in range(N):
            for i in range(4):
                vals.append(np.array([[1, 0, -mu[i]], [-1, 0, -mu[i]], [0, 1, -mu[i]], [0, -1, -mu[i]], [0, 0, 1]]).ravel())
        Ax = np.concatenate(vals)[self.perm]
        l, u = np.zeros(self.nc), np.zeros(self.nc)
        l[:NX] = u[:NX] = Ads[0] @ x0
        fr = np.arange(N * NX, self.nc).reshape(N, 4, 5)
        l[fr[..., :4]] = -np.inf
        l[fr[..., 4]], u[fr[..., 4]] = F_MIN * contact, F_MAX * contact
        q = np.concatenate([-(xref * Q_DIAG).ravel(), np.zeros(N * NU)])
        if self.prob is None:
            self.A.data = Ax
            self.prob = osqp.OSQP()
            self.prob.setup(sp.triu(self.P, format="csc"), q, self.A, l, u, warm_start=True, verbose=False, polish=False,
                            eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)  # fmt: skip
        else:
            self.prob.update(q=q, l=l, u=u, Ax=Ax)
        res = self.prob.solve()
        if res.info.status not in ("solved", "solved inaccurate"):
            self.n_fail += 1
            f = np.zeros((4, 3))
            st = contact[0] > 0
            f[st, 2] = self.m * G / max(1, st.sum())  # static share: keep standing through a failed solve
            return f, False
        return res.x[N * NX : N * NX + NU].reshape(4, 3), True
```

  (If the pinned osqp release spells the status differently, use its documented solved-status test; the test above pins the
  behaviour, not the spelling.)
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_mpc.py -n 8 -s'`;
  copy the printed median / max solve time into `docs/dog/locomotion.md` "Compute budget" (Edit). Over 8 ms: drop `N` to 10 and
  rerun before anything else (and say so in the doc).
- [ ] **Step 5: commit** `git add examples/dog/loco/mpc.py tests/python/test_dog_mpc.py docs/dog/locomotion.md && git commit -m "dog loco: convex MPC (single rigid body, friction pyramid, OSQP sparse fixed pattern)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 5: gait scheduler, swing curves, nominal footsteps (`loco/gait.py`)

**Files:**
- Create: `examples/dog/loco/gait.py`
- Test: `tests/python/test_dog_gait.py`

**Interfaces:**
- Produces: `loco.gait`: `G`, `K_CAP`, `D_MAX`, `Gait(name, period, duty, offsets, apex, v_max)`, `GAITS` (`stand`, `walk`,
  `trot`, `flying_trot`, `bound`), `choose(v_cmd, rough) -> str`, `LegPhase(stance, s, t_swing, t_to_td, early, late)`,
  `Scheduler(gait="stand")` with `.g`, `.request(name)`, `.set_params(name, **overrides)`, `.advance(dt, contact) -> list[LegPhase]`,
  `.table(n, dt) -> (n, 4)` and `.gate() -> (4,) bool`; `swing(p0, p1, apex, s, T) -> (pos, vel)`;
  `nominal_foothold(hip_w, v, v_cmd_w, wz, t_stance, h) -> (2,)`.

- [ ] **Step 1: failing test** `tests/python/test_dog_gait.py` (pure Python):

```python
"""SP4 Task 5: gait scheduler (pairs, duty, transitions at the phase wrap, early touchdown), swing curves, Raibert + capture-point footsteps."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "dog"))
from loco import gait as Gt  # noqa: E402

DT = 0.002


def run(s, n, contact=(True,) * 4):
    return [s.advance(DT, contact) for _ in range(n)]


@pytest.mark.parametrize("name,pairs", [("trot", [(0, 3), (1, 2)]), ("bound", [(0, 1), (2, 3)])])
def test_pairs_move_together_and_duty_holds(name, pairs):
    s = Gt.Scheduler(name)
    g = Gt.GAITS[name]
    hist = np.array([[lp.stance for lp in legs] for legs in run(s, round(g.period / DT) * 4)])
    for a, b in pairs:
        assert (hist[:, a] == hist[:, b]).all()
    assert hist.mean(axis=0) == pytest.approx(np.full(4, g.duty), abs=0.01)


def test_gait_change_waits_for_the_phase_wrap():
    s = Gt.Scheduler("trot")
    run(s, 10)
    s.request("bound")
    assert s.g.name == "trot"
    run(s, round(Gt.GAITS["trot"].period / DT))
    assert s.g.name == "bound"


def test_choose_by_speed_and_terrain():
    assert Gt.choose((0.0, 0.0, 0.0), False) == "stand"
    assert Gt.choose((1.0, 0.0, 0.0), False) == "trot"
    assert Gt.choose((2.6, 0.0, 0.0), False) == "flying_trot"
    assert Gt.choose((3.6, 0.0, 0.0), False) == "bound"
    assert Gt.choose((1.0, 0.0, 0.0), True) == "walk"


def test_early_touchdown_turns_the_leg_to_stance():
    s = Gt.Scheduler("trot")
    g = Gt.GAITS["trot"]
    while True:  # leg 0 just past mid-swing
        legs = s.advance(DT, (False, True, True, False))
        if not legs[0].stance and legs[0].s > 0.6:
            break
    legs = s.advance(DT, (True, True, True, False))
    assert legs[0].stance and legs[0].early
    assert not s.gate()[0] and g.duty < 1.0


def test_swing_curve_ends_apex_and_rest():
    p0, p1, T = np.array([0.0, 0.0, 0.0]), np.array([0.2, 0.05, 0.15]), 0.16
    a, va = Gt.swing(p0, p1, 0.08, 0.0, T)
    b, vb = Gt.swing(p0, p1, 0.08, 1.0, T)
    m, _ = Gt.swing(p0, p1, 0.08, 0.5, T)
    assert np.allclose(a, p0) and np.allclose(b, p1) and np.allclose(va, 0) and np.allclose(vb, 0)
    assert m[2] == pytest.approx(0.15 + 0.08)
    s = np.linspace(0, 1, 401)
    z = np.array([Gt.swing(p0, p1, 0.08, x, T)[0][2] for x in s])
    assert np.abs(np.diff(z)).max() < 0.005  # continuous


def test_raibert_capture_point_footstep():
    hip, h, Ts = np.array([0.3, 0.1, 0.4]), 0.35, 0.16
    v = np.array([1.0, 0.0, 0.0])
    xy = Gt.nominal_foothold(hip, v, v, 0.0, Ts, h)
    assert xy == pytest.approx(hip[:2] + [v[0] * Ts / 2, 0.0])
    fast = Gt.nominal_foothold(hip, np.array([1.3, 0.0, 0.0]), v, 0.0, Ts, h)
    assert fast[0] > xy[0] + Gt.K_CAP * np.sqrt(h / Gt.G) * 0.3 * 0.99  # running ahead of the command: step further out
    far = Gt.nominal_foothold(hip, np.array([9.0, 0.0, 0.0]), v, 0.0, Ts, h)
    assert np.linalg.norm(far - hip[:2]) == pytest.approx(Gt.D_MAX)
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_gait.py -n 8'`
- [ ] **Step 3: implement `examples/dog/loco/gait.py`:**

```python
"""Gait scheduler, swing curves and nominal footsteps of the dog (SP4).
Phase clock: phase += dt / period; leg i is in scheduled stance while (phase + offset_i) mod 1 < duty. A gait change waits for the phase wrap.
Early touchdown (contact in the second half of a swing) turns the leg to stance until its scheduled stance; late touchdown (scheduled stance,
no contact) keeps it out of the MPC's stance set and lets the swing curve keep descending (loco.controller). Footsteps: Raibert (1986)
v T_stance / 2 plus the capture-point feedback sqrt(h / g) (v - v_cmd) and the turning term, as in the MIT Cheetah software."""

from dataclasses import dataclass, replace

import numpy as np

G = 9.81
K_CAP = 1.0  # x sqrt(h/g): capture-point feedback gain
D_MAX = 0.18  # m: farthest touchdown from the hip (leg workspace at the stand height, ~0.4 of the leg length)


@dataclass(frozen=True)
class Gait:
    name: str
    period: float  # s
    duty: float  # stance fraction of the period
    offsets: tuple  # phase offset per leg (fl, fr, rl, rr)
    apex: float  # m above the higher end of the swing
    v_max: float  # m/s: choose() takes the slowest gait whose v_max covers the command


# Starting points (Mini Cheetah / Cheetah 3 class values scaled to 12 kg, 0.22 m links); Task 10 tunes trot / flying_trot / bound on TRAIN seeds and
# writes the chosen numbers back here with the run that chose them.
GAITS = {
    "stand": Gait("stand", 0.5, 1.0, (0.0, 0.0, 0.0, 0.0), 0.0, 0.0),
    "walk": Gait("walk", 0.6, 0.75, (0.0, 0.5, 0.75, 0.25), 0.10, 0.5),  # three feet down most of the time: steps, ledges
    "trot": Gait("trot", 0.32, 0.5, (0.0, 0.5, 0.5, 0.0), 0.08, 2.2),
    "flying_trot": Gait("flying_trot", 0.28, 0.38, (0.0, 0.5, 0.5, 0.0), 0.08, 3.2),  # flight phases: shorter, harder stance per step
    "bound": Gait("bound", 0.26, 0.40, (0.0, 0.0, 0.5, 0.5), 0.07, 9.9),  # front pair / rear pair
}
ORDER = ("trot", "flying_trot", "bound")


def choose(v_cmd, rough):
    if rough:
        return "walk"
    s = float(np.hypot(v_cmd[0], v_cmd[1]))
    if s < 0.05 and abs(v_cmd[2]) < 0.05:
        return "stand"
    return next(g for g in ORDER if s <= GAITS[g].v_max or g == ORDER[-1])


@dataclass
class LegPhase:
    stance: bool
    s: float  # swing progress 0..1 (0 in stance)
    t_swing: float  # s: swing duration of the current gait
    t_to_td: float  # s: until the next scheduled touchdown (0 in stance)
    early: bool
    late: bool


class Scheduler:
    def __init__(self, gait="stand"):
        self.gaits = dict(GAITS)
        self.g, self.next, self.phase = self.gaits[gait], None, 0.0
        self.early, self._legs = [False] * 4, []

    def set_params(self, name, **kw):
        self.gaits[name] = replace(self.gaits[name], **kw)

    def request(self, name):
        if name != self.g.name:
            self.next = self.gaits[name]

    def advance(self, dt, contact):
        p = self.phase + dt / self.g.period
        if p >= 1.0 or (self.next is not None and self.g.name == "stand"):
            p %= 1.0
            if self.next is not None:
                self.g, self.next = self.next, None
        self.phase = p
        g, legs = self.g, []
        for i in range(4):
            ph = (p + g.offsets[i]) % 1.0
            sched = ph < g.duty
            s = 0.0 if sched else (ph - g.duty) / (1.0 - g.duty)
            if sched:
                self.early[i] = False
            elif s > 0.5 and contact[i]:
                self.early[i] = True
            t_sw = (1.0 - g.duty) * g.period
            legs.append(LegPhase(sched or self.early[i], s, t_sw, 0.0 if sched else (1.0 - s) * t_sw, self.early[i], sched and not contact[i]))
        self._legs = legs
        return legs

    def gate(self):
        return np.array([not lp.stance and lp.s < 0.5 for lp in self._legs] or [False] * 4)

    def table(self, n, dt):
        """(n, 4) scheduled stance (1) / swing (0) over the MPC horizon; step 0 uses the legs' current state (early / late touchdown)."""
        g, out = self.g, np.zeros((n, 4))
        for k in range(n):
            ph = (self.phase + k * dt / g.period + np.array(g.offsets)) % 1.0
            out[k] = ph < g.duty
        if self._legs:
            out[0] = [lp.stance and not lp.late for lp in self._legs]
        return out


def _mj(a):
    return a**3 * (10 - 15 * a + 6 * a * a), 30 * a * a * (1 - a) ** 2  # min-jerk value and d/da


def swing(p0, p1, apex, s, T):
    """Foot position / velocity at swing progress s over T s: xy min-jerk p0 -> p1; z min-jerk up to max(p0z, p1z) + apex at s = 0.5, then down."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    b, db = _mj(min(max(s, 0.0), 1.0))
    xy, vxy = p0[:2] + (p1[:2] - p0[:2]) * b, (p1[:2] - p0[:2]) * db / T
    top = max(p0[2], p1[2]) + apex
    a, z0, z1 = (2 * s, p0[2], top) if s < 0.5 else (2 * s - 1, top, p1[2])
    bz, dbz = _mj(min(max(a, 0.0), 1.0))
    return np.array([*xy, z0 + (z1 - z0) * bz]), np.array([*vxy, (z1 - z0) * dbz * 2 / T])


def nominal_foothold(hip_w, v, v_cmd_w, wz, t_stance, h):
    k = np.sqrt(h / G)
    d = np.asarray(v[:2]) * t_stance / 2 + K_CAP * k * (np.asarray(v[:2]) - np.asarray(v_cmd_w[:2]))
    d = d + 0.5 * k * np.cross(v, [0.0, 0.0, wz])[:2]
    n = np.linalg.norm(d)
    return np.asarray(hip_w[:2]) + (d if n <= D_MAX else d * D_MAX / n)
```

- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_gait.py -n 8'`
- [ ] **Step 5: commit** `git add examples/dog/loco/gait.py tests/python/test_dog_gait.py && git commit -m "dog loco: gait scheduler (stand/walk/trot/flying trot/bound, early/late touchdown), swing curves, Raibert + capture-point footsteps" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 6: leg control + controller + flat-ground harness, trot, golden hash

**Files:**
- Create: `examples/dog/loco/controller.py`, `examples/dog/dogwalk.py`, `examples/dog/walk/trot.scenario.toml`, `examples/dog/walk/trot.py`
- Modify: `examples/dog/dogtruth.py` (`PoseFix`), `examples/dog/dogcourse.py` (`build_for`), `docs/dog/locomotion.md` (section
  "Flat ground: tracking and estimator")
- Test: `tests/python/test_dog_walk.py`

**Interfaces:**
- Consumes: Tasks 1-5; `napl.params.resolve`; `napl.marine.write_native`.
- Produces:
  - `loco.controller`: `DT = 0.002`, `MPC_EVERY = 10`, `KP_SWING`, `KD_SWING`, `KD_STANCE`, `KP_STAND`, `KD_STAND`, `H_FRAC`,
    `T_STANDUP`, `MODES = ("STANDUP", "STAND", "WALK", "FALLEN", "SIT")`, `Controller(model, hw, hmap)` with `.step(cmd)` (cmd =
    (vx, vy, wz) body frame), `.fix(p_xy, yaw, age_s)`, `.mode`, `.sched`, `.est`, `.st`, `.mpc`, `.stats` (dict `n_slip`, `mpc_fail`,
    `sat_hip`, `sat_knee`), `.h_frac`.
  - `dogtruth.PoseFix(truth, seed, every=20, sig_p=0.02, sig_yaw_deg=1.0, age_s=0.05)` with `.due(k) -> bool`, `.sample() -> (p_xy, yaw,
    age_s)`.
  - `dogcourse.build_for(scenario) -> build(out)`.
  - `dogwalk`: `SIM_DT`, `COLS`, `MODE_CODE`, `DEFAULTS`, `run(model, p, run_dir=None, log=True, audit=False) -> dict(rows, sim, ctl,
    res)`, `main()`; `res` = `dict(reached, fallen, ctl_fallen, fault, diverged, t_end, dist, mean_speed, n_slip, mpc_fail, sat_hip,
    rangefinders)`.

- [ ] **Step 1: failing test** `tests/python/test_dog_walk.py`:

```python
"""SP4 Task 6: the controller on flat ground: stand-up from the build pose, a 1 m/s trot, estimator accuracy, the torque-based contact estimate
against the virtual FootContact (Review Focus 4), the controller's inputs (Review Focus 2), golden state hash of walk/trot.scenario.toml."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C  # noqa: E402
import dogwalk as W  # noqa: E402
from loco import hw as H  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = {"trot": None}  # recorded in Step 5 from res["sim"]["state_hash"]
LEGS = ("fl", "fr", "rl", "rr")
c = W.COLS.index


def test_goldens_are_recorded():
    assert all(GOLDEN.values()), "record walk/trot.scenario.toml's state hash (Task 6 Step 5)"


def test_loco_never_touches_the_sim_or_the_truth():  # Review Focus 2, static half
    for f in (EX / "loco").glob("*.py"):
        if f.name != "hw.py":
            src = f.read_text()
            for bad in ("import napl", "from napl", "sim.", "dogtruth", "dogcourse"):
                assert bad not in src, (f.name, bad)


@pytest.fixture(scope="module")
def trot(tmp_path_factory):
    model = C.scene(tmp_path_factory.mktemp("trot"), "flat", seed=1)
    return W.run(model, dict(W.DEFAULTS, v_cmd=1.0, duration=8.0), log=False, audit=True)


@needs_kernel
def test_controller_reads_only_hardware_attrs(trot):  # Review Focus 2, dynamic half
    reads = trot["ctl"].hw.reads
    assert reads and all(p.startswith("/devices/") for p, _ in reads)
    assert {a for _, a in reads} <= H.HW_ATTRS
    assert not any(p.endswith(":foot") for p, _ in reads)  # the virtual FootContact is ground truth, not a sensor


@needs_kernel
def test_trot_tracks_one_metre_per_second(trot):
    rows = np.array(trot["rows"])
    late = rows[rows[:, c("t")] > 5.0]
    assert late[:, c("vx")].mean() == pytest.approx(1.0, abs=0.1)
    assert rows[:, c("fallen")].max() == 0 and rows[:, c("fault")].max() == 0
    assert np.abs(late[:, c("y")]).max() < 0.3
    assert np.sqrt(np.mean(late[:, c("est_v_err")] ** 2)) < 0.1


@needs_kernel
def test_contact_estimate_matches_foot_contact_truth(trot):  # Review Focus 4
    rows = np.array(trot["rows"])
    late = rows[:, c("t")] > 3.0
    est = rows[late][:, [c(f"c_est_{l}") for l in LEGS]]
    tru = rows[late][:, [c(f"c_true_{l}") for l in LEGS]]
    assert (est == tru).mean() >= 0.95


@needs_kernel
def test_stands_up_from_the_build_pose(tmp_path):
    r = W.run(C.scene(tmp_path, "flat", seed=1), dict(W.DEFAULTS, v_cmd=0.0, duration=3.0), log=False)
    last = r["rows"][-1]
    assert last[c("mode")] == W.MODE_CODE["STAND"] and last[c("tilt")] < 2.0 and last[c("fault")] == 0


@needs_kernel
@pytest.mark.parametrize("name", ["trot"])
def test_scenario_passes_replays_and_matches_golden(tmp_path, name):
    res, _ = program.run(EX / "walk" / f"{name}.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    assert GOLDEN[name] is None or res["sim"]["state_hash"] == GOLDEN[name], res["sim"]["state_hash"]
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_walk.py -n 8'`
- [ ] **Step 3: implement `examples/dog/loco/controller.py`** (key code; terrain hooks call `loco.terrain` functions that Task 8
  adds, here stubbed by flat defaults inside `terrain.py`: `plane()` returns `(0, 0, mean map height)`, `foothold()` returns the
  nominal point at the map height, `apex_over()` returns the gait apex, `rough_ahead()` returns False; Task 8 replaces their bodies):

```python
"""The dog's locomotion controller (SP4): what runs on the Jetson. 500 Hz: estimator -> gait -> footholds / swing -> leg control (MIT frames);
50 Hz: convex MPC. Leg control (Di Carlo 2018 "leg controller"): stance tau_ff = J^T (-R^T f_mpc), kp 0, joint damping; swing joint PD on the IK
of the swing curve, v_des = J^-1 v_foot. Modes: STANDUP -> STAND <-> WALK; any -> FALLEN (safe-sit, Task 9) -> STANDUP when upright.
Inputs: loco.hw (drivers + IMU), the command, a height map, pose fixes (SP5 stand-ins). Never the Sim, never ground truth."""

import numpy as np

from . import gait as Gt
from . import mpc as Mp
from . import terrain as Tr
from .estimator import Estimator

DT, MPC_EVERY = 0.002, 10  # 500 Hz leg control (spec s6); MPC every 10 ticks = 50 Hz
KP_SWING, KD_SWING, KD_STANCE = 40.0, 1.2, 0.8  # joint side (N m/rad, N m s/rad): ~25 Hz swing bandwidth on a ~0.05 kg m^2 leg
KP_STAND, KD_STAND = 60.0, 1.5  # = dogsim.KP / KD (the SP3b stand gains)
H_FRAC = 0.82  # = dogsim.STAND: hip-pitch-to-foot distance as a fraction of the straight leg
T_STANDUP = 1.2  # s
MU_NOM = 0.5  # MPC friction: below rock 0.8 / grass 0.6 / gravel 0.55 (napl.terrain MATERIALS); slip handling lowers it (Task 9)
MODES = ("STANDUP", "STAND", "WALK", "FALLEN", "SIT")


class Controller:
    def __init__(self, model, hw, hmap):
        self.m, self.hw, self.map = model, hw, hmap
        self.est, self.mpc = Estimator(model, hmap, DT), Mp.Mpc(model.mass, model.inertia)
        self.sched = Gt.Scheduler("stand")
        self.legs = list(model.legs.values())
        self.mode, self.k, self.t_mode = "STANDUP", 0, 0.0
        self.f = np.zeros((4, 3))
        self.mu = np.full(4, MU_NOM)
        self.h_frac = H_FRAC
        self.lift = np.zeros((4, 3))  # world lift-off point of each swing
        self.target = np.zeros((4, 3))  # world touchdown target of each swing
        self.stats = dict(n_slip=0, mpc_fail=0, sat_hip=0.0, sat_knee=0.0)
        self.q_prev = None
        self.started = False

    def h_stand(self):
        L = self.m.legs["fl"]
        return self.h_frac * np.linalg.norm(L.foot0 - L.p[1]) - L.p[1][2]  # trunk origin above the ball centres

    def fix(self, p_xy, yaw, age_s):
        self.est.fix(p_xy, yaw, age_s, 0.02, np.radians(1.0))

    def step(self, cmd):
        q, dq, tau = self.hw.joints()
        gyro, acc = self.hw.imu()
        if not self.started:
            self.hw.enable()
            self.est.reset(q)
            self.q0, self.started = q.copy(), True
        st = self.st = self.est.update(q, dq, tau, gyro, acc, self.sched.gate())
        self.k += 1
        self.t_mode += DT
        if self.mode == "STANDUP":
            a = min(1.0, self.t_mode / T_STANDUP)
            tgt = self.q0 + (self.m.stand_q(self.h_frac) - self.q0) * a
            self.hw.send(tgt, np.zeros(12), np.full(12, KP_STAND), np.full(12, KD_STAND), np.zeros(12))
            if a >= 1.0:
                self._enter("STAND")
            return
        # (Task 9 inserts the safety checks here: slip -> mu / speed scale, fall -> FALLEN)
        moving = Gt.choose(cmd, False) != "stand"
        if self.mode == "STAND" and moving:
            self._enter("WALK")
        if self.mode == "WALK":
            self.sched.request(Gt.choose(cmd, Tr.rough_ahead(self.map, st.p[:2], st.rpy[2], 0.8)) if moving else "stand")
        phases = self.sched.advance(DT, st.contact)
        if self.mode == "WALK" and self.sched.g.name == "stand" and not moving:
            self._enter("STAND")
        if self.k % MPC_EVERY == 0:
            self._mpc(st, cmd, phases)
        self.hw.send(*self._legs(st, cmd, phases, q, dq))

    def _enter(self, mode):
        self.mode, self.t_mode = mode, 0.0
```

  `_mpc(st, cmd, phases)`: `R = st.R`, CoM `c = st.p + R @ m.com`, `x0 = [rpy, c, R @ st.w, st.v + R @ cross(w, com), -G]`,
  reference `xref[k]` (k = 1..N, t = k DT): `rpy = (POSTURE roll_t, POSTURE pitch_t, yaw + wz t)` from `Tr.plane(map, c_xy, yaw)`
  (`POSTURE = 0.7`: lean 70 % into the slope), CoM xy `c_xy + Rz(yaw) [vx, vy] t`, z = plane height + `h_stand()` + `m.com[2]`,
  `w = (0, 0, wz)`, `v = Rz(yaw) [vx, vy, 0]`; `r_feet[k, i]` = the foot's world point (stance: `st.feet[i]`; a swing landing within
  the horizon: `self.target[i]`) minus the reference CoM of step k; `contact = self.sched.table(N, DT)`; then `self.f, ok =
  self.mpc.solve(...)`, `stats["mpc_fail"] = self.mpc.n_fail`. `_legs(...)` returns the 5 MIT 12-vectors:

```python
    def _legs(self, st, cmd, phases, q, dq):
        R, p, v, w = st.R, st.p, st.v, st.w
        p_des, v_des, kp, kd, tau = q.copy(), np.zeros(12), np.zeros(12), np.full(12, KD_STANCE), np.zeros(12)
        yaw = st.rpy[2]
        v_cmd_w = Mp.rz(yaw) @ [cmd[0], cmd[1], 0.0]
        for i, (L, lp) in enumerate(zip(self.legs, phases)):
            s = slice(3 * i, 3 * i + 3)
            r, J = L.fk_jac(q[s])
            if lp.stance and not lp.late:
                tau[s] = J.T @ (-R.T @ self.f[i])
                v_des[s] = np.linalg.solve(J, -(R.T @ v + np.cross(R.T @ w, r)))  # foot still in the world
                self.lift[i] = p + R @ r
                continue
            if lp.s < 1e-9 or not np.any(self.target[i]):
                self.lift[i] = p + R @ r
            hip_td = p + R @ L.p[0] + v_cmd_w * lp.t_to_td
            xy = Gt.nominal_foothold(hip_td, v, v_cmd_w, cmd[2], self.sched.g.duty * self.sched.g.period, self.h_stand())
            self.target[i] = Tr.foothold(self.map, xy, L.r_foot)
            apex = Tr.apex_over(self.map, self.lift[i], self.target[i], self.sched.g.apex)
            x_w, v_w = Gt.swing(self.lift[i], self.target[i], apex, lp.s, lp.t_swing)
            if lp.late:  # scheduled stance, no contact yet: keep reaching down, at most 6 cm below the plan
                x_w = self.target[i] - [0.0, 0.0, min(0.06, 0.3 * (self.sched.g.duty * self.sched.g.period) * lp.s)]
                v_w = np.array([0.0, 0.0, -0.3])
            x_b = R.T @ (x_w - p)
            p_des[s] = L.ik(x_b, q[s])
            v_des[s] = np.linalg.solve(J, R.T @ (v_w - v) - np.cross(R.T @ w, r))
            kp[s], kd[s] = KP_SWING, KD_SWING
        lim = np.concatenate([L.tau_max for L in self.legs])
        sat = np.abs(tau) >= 0.95 * lim
        self.stats["sat_hip"] = 0.99 * self.stats["sat_hip"] + 0.01 * sat[1::3].mean()  # ~0.2 s moving average
        self.stats["sat_knee"] = 0.99 * self.stats["sat_knee"] + 0.01 * sat[2::3].mean()
        return p_des, v_des, np.minimum(kp, np.concatenate([L.kp_max for L in self.legs])), kd, np.clip(tau, -lim, lim)
```

  `dogtruth.PoseFix`: rng `[seed, 5151]`; `.due(k)` every 20 control ticks (10 Hz); `.sample()` = truth xy + N(0, 0.02 m), yaw +
  N(0, 1 deg) taken 50 ms earlier (a 25-entry deque of truth poses filled by `.due`), returned with `age_s = 0.05` ("RTK fix +
  lidar yaw class; SP5's own numbers replace these").
  `dogcourse.build_for(scen)`: `def build(out): p = napl.params.resolve(scen); return str(scene(out, p["course"], p["seed"]))`.
  `examples/dog/dogwalk.py`:

```python
"""Harness of the dog's walk scenarios (examples/dog/walk/*.scenario.toml, `napl program`): owns the Sim, plays the world (ground truth, pushes,
the SP5 stand-ins: pose fix = truth + noise, height map = ground truth), steps loco.controller at 500 Hz (sim.step(2) per tick) and writes
trace.csv / replay.naplog / native.json. The route follower (dogcourse.Follower) steers on the TRUE pose: route planning and localization
are SP5. Env: NAPL_MODEL, NAPL_RUN_DIR, NAPL_PARAMS; argument: the scenario file name."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")  # bitwise-repeatable BLAS (golden hashes)

import json  # noqa: E402
import math  # noqa: E402
import sys  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from collections import deque  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dogcourse  # noqa: E402
import dogtruth  # noqa: E402
import napl  # noqa: E402
from loco import controller as Ct  # noqa: E402
from loco import hw as H  # noqa: E402
from loco import model as M  # noqa: E402
from loco import terrain as Tr  # noqa: E402
from napl import params as nparams  # noqa: E402
from napl.marine import write_native  # noqa: E402

SIM_DT, ROW_EVERY = 0.001, 10  # physics 1 kHz (assemblies/dog/dog.toml timestep); one trace row per 10 control ticks (50 Hz)
LEGS = H.LEGS
MODE_CODE = {m: i for i, m in enumerate(Ct.MODES)}
COLS = (["t", "x", "y", "vx", "vy", "v_mean_2s", "est_v_err", "tilt", "clear", "fallen", "ctl_fallen", "fault", "v_bat", "mode",
         "gait", "mu_min", "n_slip", "mpc_fail", "dist_goal"] + [f"c_est_{l}" for l in LEGS] + [f"c_true_{l}" for l in LEGS])  # fmt: skip
GAIT_CODE = {g: i for i, g in enumerate(["stand", "walk", "trot", "flying_trot", "bound"])}
DEFAULTS = dict(seed=1, course="flat", v_cmd=1.0, duration=8.0, push_t=0.0, push_dv=0.0, gait="auto", pushes=())


def run(model, p, run_dir=None, log=True, audit=False):
    inf = dogcourse.info(model)
    field = dogcourse.field_of(inf)
    sim = napl.Sim.load(str(model))
    if log:
        sim.start_log()
    tr, hw = dogtruth.Truth(sim, field), H.Hw(sim, audit=audit)
    hmap = Tr.FlatMap() if field is None else Tr.TruthMap(field)
    ctl = Ct.Controller(M.from_build(inf["nominal"]), hw, hmap)
    fix = dogtruth.PoseFix(tr, p["seed"])
    route = inf.get("route")
    fol = dogcourse.Follower(route, p["v_cmd"]) if route else None
    pushes = list(p.get("pushes") or ()) + ([(p["push_t"], (0.0, p["push_dv"], 0.0))] if p.get("push_dv") else [])
    rows, vx_hist, reached, fell, ctl_fell = [], deque(maxlen=100), False, False, False
    n = int(round(p["duration"] / Ct.DT))
    for k in range(n):
        t = k * Ct.DT
        for tp, dv in pushes:
            if abs(t - tp) < Ct.DT / 2:
                tr.push(np.asarray(dv, float))
        if fix.due(k):
            ctl.fix(*fix.sample())
        if fol:
            cmd = fol.cmd(tr.pos(), tr.yaw())
            reached = reached or fol.done
        else:
            cmd = (p["v_cmd"], 0.0, 0.0) if t > Ct.T_STANDUP + 0.5 else (0.0, 0.0, 0.0)
        ctl.step(cmd)
        sim.step(2)
        if ctl.mode == "STAND" and tr.clear0 is None:
            tr.arm_fall()
        fell = fell or tr.fallen()
        ctl_fell = ctl_fell or ctl.mode == "FALLEN"
        if k % ROW_EVERY == 0:
            v = tr.vel()
            vb = np.array([math.cos(tr.yaw()) * v[0] + math.sin(tr.yaw()) * v[1], -math.sin(tr.yaw()) * v[0] + math.cos(tr.yaw()) * v[1]])
            vx_hist.append(vb[0])
            st = ctl.st
            pos = tr.pos()
            fault, vbat, _ = hw.health()
            rows.append([t, pos[0], pos[1], vb[0], vb[1], float(np.mean(vx_hist)),
                         float(np.linalg.norm(st.v[:2] - v[:2])) if t > 2.0 else 0.0, tr.tilt_deg(), tr.clearance(), float(fell),
                         float(ctl_fell), fault, vbat, MODE_CODE[ctl.mode], GAIT_CODE[ctl.sched.g.name], float(ctl.mu.min()),
                         ctl.stats["n_slip"], ctl.stats["mpc_fail"], fol.dist if fol else 0.0]
                        + [float(x) for x in st.contact] + list(tr.contacts()))  # fmt: skip
    res = dict(reached=bool(reached), fallen=fell, ctl_fallen=ctl_fell, fault=max(r[COLS.index("fault")] for r in rows) > 0,
               diverged=bool(sim.diverged), t_end=n * Ct.DT, dist=fol.length if fol else 0.0,
               mean_speed=(fol.length / fol.t_done) if fol and fol.done else 0.0, n_slip=ctl.stats["n_slip"],
               mpc_fail=ctl.stats["mpc_fail"], sat_hip=ctl.stats["sat_hip"],
               rangefinders=len(json.loads(ET.parse(model).getroot().find("custom/text[@name='napl.devices']").get("data"))
                                .get("rangefinders", [])))  # fmt: skip
    if run_dir:
        write_native(sim, SIM_DT, COLS, rows, run_dir)
    return dict(rows=rows, sim=sim, ctl=ctl, res=res)


def main():
    scen = Path(os.environ.get("NAPL_SCENARIO_DIR", HERE / "walk")) / sys.argv[1]
    p = dict(DEFAULTS, **nparams.values(scen))
    model, run_dir = Path(os.environ["NAPL_MODEL"]), Path(os.environ["NAPL_RUN_DIR"])
    inf = dogcourse.info(model)
    if (inf["course"], inf["seed"]) != (p["course"], p["seed"]):  # a UI changed the course / seed: build that scene
        model = dogcourse.scene(run_dir / "scene", p["course"], p["seed"])
    r = run(model, p, run_dir)
    (run_dir / "result_native.json").write_text(json.dumps(r["res"]))


if __name__ == "__main__":
    main()
```

  `Follower` is Task 8; here `fol` is None for `route = None`.
  `examples/dog/walk/trot.py`:

```python
"""[model] script of trot.scenario.toml: that scenario's scene at its default parameters (dogcourse.build_for)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import dogcourse  # noqa: E402

build = dogcourse.build_for(Path(__file__).with_name("trot.scenario.toml"))
```

  `examples/dog/walk/trot.scenario.toml`:

```toml
# napl program examples/dog/walk/trot.scenario.toml --runs-dir <dir>
# SP4: the dog stands up from its build pose (straight legs) and trots straight at 1 m/s on flat ground (convex MPC 50 Hz + leg control
# 500 Hz on the 12 Qdds; lidar and depth camera removed). examples/dog/dogwalk.py OWNS the Sim, steps examples/dog/loco and writes trace.csv /
# replay.naplog / native.json; the runner judges the monitors and replays for the state hash.
name = "dog-trot"

[model]
script = "trot.py"

[interfaces.native]

[program]
cmd = ["python", "{dir}/../dogwalk.py", "trot.scenario.toml"]

[limits]
wall_s = 900
cpu_s = 900

[[parameters]]
name = "seed"
type = "int"
default = 1
min = 1
max = 1000000
group = "run"
label = "Seed"
doc = "Seeds the pose-fix noise and (course != flat) the terrain: the same seed and parameters replay to the same state hash."

[[parameters]]
name = "course"
type = "choice"
choices = ["flat", "criterion", "slip", "rubble", "slopes", "scree", "ruins"]
default = "flat"
group = "course"
label = "Course"

[[parameters]]
name = "v_cmd"
type = "float"
default = 1.0
min = 0.0
max = 4.5
unit = "m/s"
group = "run"
label = "Commanded speed"

[[parameters]]
name = "duration"
type = "float"
default = 8.0
min = 2.0
max = 120.0
unit = "s"
group = "run"
label = "Run length"

[[monitors]]
name = "tracks_the_commanded_speed"   # mean forward speed over the last 2 s within 10 % of the 1 m/s command
column = "v_mean_2s"
op = "final"
lo = 0.9
hi = 1.1

[[monitors]]
name = "never_falls"                  # ground truth: tilt > 60 deg or trunk clearance < 60 % of standing (dogtruth.py)
column = "fallen"
op = "always"
hi = 0.0

[[monitors]]
name = "no_qdd_fault"
column = "fault"
op = "always"
hi = 0.0

[[monitors]]
name = "estimator_velocity"           # |v_est - v_true| (xy) after 2 s; 0.3 m/s = 30 % of the command, a broken-estimator alarm
column = "est_v_err"
op = "always"
hi = 0.3
```

- [ ] **Step 4: run, expect PASS except `test_goldens_are_recorded`:**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_walk.py -n 8'`.
  Tune only `KP_SWING`, `KD_SWING`, `KD_STANCE`, `MU_NOM`, `GAITS["trot"]` (period, apex) if needed, each change with its reason in
  the constant's comment; never loosen a test bound.
- [ ] **Step 5: record the golden hash.** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/napl program examples/dog/walk/trot.scenario.toml --runs-dir runs/dog-trot && cat runs/dog-trot/*/result.json'`;
  copy `sim.state_hash` into `GOLDEN["trot"]` (Edit), rerun Step 4: all PASS. Write `docs/dog/locomotion.md` "Flat ground" (Edit):
  speed tracking, estimator velocity RMS, contact agreement, MPC failures, real-time factor of the run (`result.json` wall vs sim).
- [ ] **Step 6: commit** `git add examples/dog/loco/controller.py examples/dog/loco/terrain.py examples/dog/dogwalk.py examples/dog/dogtruth.py examples/dog/dogcourse.py examples/dog/walk docs/dog/locomotion.md tests/python/test_dog_walk.py && git commit -m "dog loco: controller (MPC 50 Hz + leg control 500 Hz, stand-up), walk harness, flat trot scenario with golden hash" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 7: QP whole-body controller (WBIC-style, 500 Hz), measured against the J^T baseline; the `leg_ctl` switch

**Files:**
- Create: `examples/dog/loco/wbc.py`
- Modify: `examples/dog/loco/model.py` (`Link`, `links`, `armature`), `examples/dog/loco/controller.py` (`leg_ctl` switch, `_swing_ref`
  factored out of `_legs`, `_legs_wbc`, `body_ref`), `examples/dog/dogtruth.py` (`Truth.forces`), `examples/dog/dogwalk.py`
  (`leg_ctl`, columns `f_track`, `wbc_fail`), `examples/dog/walk/trot.scenario.toml` (parameter `leg_ctl`), `tests/python/test_dog_walk.py`
  (`GOLDEN["trot"]` re-recorded: the default leg controller changes), `docs/dog/locomotion.md` (section "Whole-body control")
- Conditional (Step 6): `CMakeLists.txt`, `cmake/deps.cmake`, `examples/dog/loco/cpp/CMakeLists.txt`, `examples/dog/loco/cpp/wbc.cpp`
- Test: `tests/python/test_dog_wbc.py`

**Interfaces:**
- Consumes: `loco.model.Model` / `Leg._chain`, `loco.mpc` (`F_MIN`, `F_MAX`, `G`, `Mpc`), osqp; FootContact `force` (harness only).
- Produces:
  - `loco.model`: `Link(leg, upto, mass, com, inertia)` (`leg` -1 = trunk; `upto` = joints of its leg above it; com / inertia at q = 0
    in the trunk frame), `Model.links`, `Model.armature (12,)`.
  - `loco.wbc`: `BUDGET_S`, `KP_BODY`, `KD_BODY`, `KP_ORI`, `KD_ORI`, `KP_FOOT`, `KD_FOOT`, `W_FB`, `W_F`,
    `dynamics(model, R, q, u) -> (A (18,18), b (18,), g (18,))` (u = [v world, w world, dq]), `WbcOut(p_des, v_des, tau, f, ok)`,
    `Wbc(model)` with `.tau_max (12,)`, `.n_fail`, `.solve(p, R, w, v, q, dq, f_mpc, stance, mu, body_ref, swing) -> WbcOut`
    (`body_ref` = (p_des, v_des, R_des) of the trunk origin; `swing` = (x (4,3), v (4,3)) world foot references, ignored on stance legs),
    `make(model) -> Wbc` (the C++ one if Step 6 built it, else Python).
  - `loco.controller`: `LEG_CTLS = ("wbc", "jt")`, `Controller(model, hw, hmap, leg_ctl="wbc")`, `.leg_ctl`, `.body_ref`,
    `stats["wbc_fail"]`.
  - `dogtruth.Truth.forces() -> (4,)` (FootContact force magnitude per foot); `dogwalk.DEFAULTS["leg_ctl"] = "wbc"`; `COLS` + `f_track`,
    `wbc_fail`; `res["f_track_rms"]`.

- [ ] **Step 1: failing test** `tests/python/test_dog_wbc.py`:

```python
"""SP4 Task 7: whole-body controller. Structure of the controller's floating-base dynamics, the QP keeps the MPC forces inside the friction pyramid
and the drivers' torque limits (Review Focus 3), the 500 Hz solve budget, and a WBC trot on the simulated dog measured against the J^T baseline."""

import sys
import time
from pathlib import Path

import napl
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C  # noqa: E402
import dogwalk as W  # noqa: E402
from loco import model as M  # noqa: E402
from loco import mpc as Mp  # noqa: E402
from loco import wbc as Wb  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
c = W.COLS.index


@pytest.fixture(scope="module")
def m():
    return M.from_build(C.base_model())


def trot_problem(m, v=1.0):
    """Stand pose, moving at v along x, fl + rr in stance, fr + rl in swing: a real MPC solve gives f_mpc."""
    q = m.stand_q(0.82)
    feet = m.feet(q)
    stance = np.array([True, False, False, True])
    x0 = np.r_[0.0, 0.0, 0.0, m.com, 0.0, 0.0, 0.0, v, 0.0, 0.0, -Mp.G]
    xref = np.tile(x0, (Mp.N, 1))
    xref[:, 3] += v * Mp.DT * np.arange(1, Mp.N + 1)
    mu = np.full(4, 0.4)
    f_mpc, ok = Mp.Mpc(m.mass, m.inertia).solve(x0, xref, np.tile(feet - m.com, (Mp.N, 1, 1)), np.tile(stance, (Mp.N, 1)).astype(float), mu)
    assert ok
    swing = (feet + [0.0, 0.0, 0.05], np.tile([v, 0.0, 0.0], (4, 1)))  # trunk at the origin: body frame = world frame
    body_ref = (np.zeros(3), np.array([v, 0.0, 0.0]), np.eye(3))
    args = (np.zeros(3), np.eye(3), np.zeros(3), np.array([v, 0.0, 0.0]), q, np.zeros(12), f_mpc, stance, mu, body_ref, swing)
    return args, f_mpc, stance, mu


def test_dynamics_structure(m):
    assert sum(lk.mass for lk in m.links) == pytest.approx(m.mass, rel=1e-9)
    q = m.stand_q(0.82)
    A, b, g = Wb.dynamics(m, np.eye(3), q, np.zeros(18))
    assert np.allclose(A, A.T, atol=1e-9) and np.linalg.eigvalsh(A).min() > 0
    assert np.allclose(A[0:3, 0:3], m.mass * np.eye(3), rtol=1e-9)  # linear block = total mass
    assert np.allclose(g[0:3], [0.0, 0.0, m.mass * Mp.G], rtol=1e-9)  # gravity on the trunk = weight
    assert np.allclose(b, 0.0, atol=1e-9)  # at rest: no Coriolis / centrifugal terms
    assert np.all(np.diag(A)[6:] >= m.armature)  # reflected rotor inertia is in the joint diagonal
    u = np.random.default_rng(1).normal(0, 1, 18)
    assert np.isfinite(Wb.dynamics(m, np.eye(3), q, u)[1]).all()


def test_wbc_keeps_mpc_forces_inside_friction_and_torque_limits(m):  # Review Focus 3
    args, f_mpc, stance, mu = trot_problem(m)
    wbc = Wb.Wbc(m)
    out = wbc.solve(*args)
    assert out.ok
    assert np.abs(out.f[~stance]).max() < 1e-6  # swing feet push nothing
    for i in np.flatnonzero(stance):
        assert np.abs(out.f[i, :2]).max() <= mu[i] * out.f[i, 2] + 1e-3
        assert Mp.F_MIN - 1e-3 <= out.f[i, 2] <= Mp.F_MAX + 1e-3
    assert np.all(np.abs(out.tau) <= wbc.tau_max + 1e-6)
    assert np.abs(out.f - f_mpc).max() < 2.0  # no limit binds: the MPC forces pass through (N)
    wbc.tau_max = 0.3 * wbc.tau_max  # now the torque rows bind
    tight = wbc.solve(*args)
    assert tight.ok and np.all(np.abs(tight.tau) <= wbc.tau_max + 1e-6)
    assert np.abs(tight.f - f_mpc).max() > 2.0  # the QP moved the forces instead of clipping torques after the fact


def test_solve_time_budget(m):
    args, *_ = trot_problem(m)
    wbc, rng, ts = Wb.make(m), np.random.default_rng(2), []
    for _ in range(200):
        a = list(args)
        a[5] = rng.normal(0, 3.0, 12)  # joint speeds of a fast trot
        t0 = time.perf_counter()
        wbc.solve(*a)
        ts.append(time.perf_counter() - t0)
    print(f"WBC solve ({type(wbc).__module__}): median {np.median(ts) * 1e3:.3f} ms, p99 {np.percentile(ts, 99) * 1e3:.3f} ms")
    assert np.median(ts) < Wb.BUDGET_S  # over budget: Task 7 Step 6 (C++ port), never a looser budget


@needs_kernel
def test_wbc_trot_against_the_jt_baseline(tmp_path):
    model = C.scene(tmp_path, "flat", seed=1)
    runs = {k: W.run(model, dict(W.DEFAULTS, v_cmd=1.0, duration=8.0, leg_ctl=k), log=False) for k in ("wbc", "jt")}
    rows = np.array(runs["wbc"]["rows"])
    late = rows[rows[:, c("t")] > 5.0]
    assert late[:, c("vx")].mean() == pytest.approx(1.0, abs=0.1)
    assert rows[:, c("fallen")].max() == 0 and rows[:, c("fault")].max() == 0
    assert runs["wbc"]["res"]["wbc_fail"] < 0.01 * len(rows) * W.ROW_EVERY  # < 1 % of control ticks fall back to J^T
    ft = {k: r["res"]["f_track_rms"] for k, r in runs.items()}
    print(f"force tracking RMS (x mg): wbc {ft['wbc']:.3f}, jt {ft['jt']:.3f}")
    assert ft["wbc"] <= 1.25 * ft["jt"]  # a WBC tracking the MPC forces clearly worse than plain J^T is a bug, not a result
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_wbc.py -n 8 -s'`
- [ ] **Step 3: model additions and `examples/dog/loco/wbc.py`.** In `loco/model.py`:

```python
@dataclass
class Link:
    leg: int  # index into LEGS, -1 = trunk
    upto: int  # how many of its leg's joints lie between deck_top and it (abad body 1, thigh 2, shank / foot 3)
    mass: float
    com: np.ndarray  # at q = 0, trunk frame
    inertia: np.ndarray  # about com, trunk axes at q = 0 (geom points + box own inertia, as Model.inertia)
```

  `from_build` groups the geoms it already reads by their body, gives each body its `leg` / `upto` from the joints on its path from
  `deck_top`, merges bodies with the same (`leg`, `upto`) into one `Link` (13 links: trunk + 3 per leg), and sets `armature[3 i + j] =
  motor.rotor_inertia * gear_ratio**2` of Qdd `<leg>:<joint>` from napl.devices (joint-side reflected rotor inertia). `Model` gains
  `links: list` and `armature: np.ndarray` (defaults `field(default_factory=list)` / `np.zeros(12)` so Task 2's tests are unchanged).
  `examples/dog/loco/wbc.py`:

```python
"""Whole-body controller of the dog (SP4), 500 Hz: WBIC (Kim, Jorgensen, Lee, Kim, "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse
Control and Model Predictive Control", arXiv:1909.06586, 2019; the MIT Mini Cheetah structure) on the controller's own model (loco.model).
1. Kinematic tasks, strict priorities by null-space projection: stance feet still -> body orientation -> body position -> swing feet. Gives
   joint position / velocity targets and the commanded generalized acceleration qdd (A^-1-weighted pseudo-inverses for qdd).
2. QP (OSQP, fixed dense pattern, warm start): z = [dqdd_fb (6), df (12)], min W_FB |dqdd_fb|^2 + W_F |df|^2, s.t.
     floating-base rows  A_fb (qdd + [dqdd_fb, 0]) + b_fb + g_fb = J_c,fb' (f_ref + df)
     f_ref + df inside the friction pyramid with fz in [F_MIN, F_MAX] on stance feet, 0 on swing feet (the rows of loco.mpc)
     joint torques       A_j (qdd + [dqdd_fb, 0]) + b_j + g_j - J_c,j' (f_ref + df) within +-tau_max
   f_ref = the MPC forces on the feet the WBC treats as stance. tau_ff = those joint torques.
Generalized velocity u = [v (trunk origin, world), w (world), dq (12)]; equations A u' + b + g = S' tau + J_c' f."""

from collections import namedtuple

import numpy as np
import osqp
import scipy.sparse as sp

from . import mpc as Mp

BUDGET_S = 0.001  # median per 2 ms control tick: half the tick, the rest is I/O + estimator + swing references (Task 1 / 3 numbers)
KP_BODY, KD_BODY = 100.0, 20.0  # 1/s^2, 1/s: body position task (critically damped, ~1.6 Hz; Mini Cheetah WBIC class)
KP_ORI, KD_ORI = 100.0, 20.0  # same for orientation
KP_FOOT, KD_FOOT = 400.0, 40.0  # swing foot task (~3.2 Hz: well inside a 0.1 s swing)
W_FB, W_F = 10.0, 1.0  # QP weights: floating-base acceleration relaxation vs reaction-force change (Kim 2019 Q2 / Q1 ratio)
EPS = 1e-5  # s: finite-difference step for J-dot u and the Coriolis term
WbcOut = namedtuple("WbcOut", "p_des v_des tau f ok")


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _rot(w, th):
    n = np.linalg.norm(w) * th
    if n < 1e-12:
        return np.eye(3)
    K = skew(w / np.linalg.norm(w))
    return np.eye(3) + np.sin(n) * K + (1 - np.cos(n)) * K @ K


def _elements(m, R, q):
    """Per link: (mass, Jv (3,18) of its com, Jw (3,18), inertia) in the world frame; per foot: Jc (3,18) of the ball centre."""
    legs, els, Jc = list(m.legs.values()), [], np.zeros((4, 3, 18))
    for i, L in enumerate(legs):
        r, J = L.fk_jac(q[3 * i : 3 * i + 3])
        Jc[i, :, 0:3], Jc[i, :, 3:6], Jc[i, :, 6 + 3 * i : 9 + 3 * i] = np.eye(3), -skew(R @ r), R @ J
    for lk in m.links:
        Jv, Jw = np.zeros((3, 18)), np.zeros((3, 18))
        Jv[:, 0:3], Jw[:, 3:6] = np.eye(3), np.eye(3)
        if lk.leg < 0:
            x, Rk = lk.com, np.eye(3)
        else:
            Rk, t, ws, ps = legs[lk.leg]._chain(q[3 * lk.leg : 3 * lk.leg + 3], lk.upto)
            x = Rk @ lk.com + t
            for j in range(lk.upto):
                Jv[:, 6 + 3 * lk.leg + j] = R @ np.cross(ws[j], x - ps[j])
                Jw[:, 6 + 3 * lk.leg + j] = R @ ws[j]
        Jv[:, 3:6] = -skew(R @ x)
        Rw = R @ Rk
        els.append((lk.mass, Jv, Jw, Rw @ lk.inertia @ Rw.T))
    return els, Jc


def _advance(R, q, u, eps):
    return _rot(u[3:6], eps) @ R, q + eps * u[6:]


def dynamics(m, R, q, u):
    els, _ = _elements(m, R, q)
    A, g = np.diag(np.r_[np.zeros(6), m.armature]), np.zeros(18)
    for mass, Jv, Jw, I in els:
        A += mass * Jv.T @ Jv + Jw.T @ I @ Jw
        g += mass * Mp.G * Jv[2]
    b = np.zeros(18)
    if np.any(u):  # ponytail: J-dot u by finite difference (one extra kinematics pass); analytic RNEA if the profile shows it
        els2, _ = _elements(m, *_advance(R, q, u, EPS))
        for (mass, Jv, Jw, I), (_, Jv2, Jw2, _) in zip(els, els2):
            wk = Jw @ u
            b += mass * Jv.T @ ((Jv2 - Jv) @ u / EPS) + Jw.T @ (I @ ((Jw2 - Jw) @ u / EPS) + np.cross(wk, I @ wk))
    return A, b, g


def _pinv_dyn(J, Ainv):
    return Ainv @ J.T @ np.linalg.pinv(J @ Ainv @ J.T, rcond=1e-6)


def _log(Rerr):
    c = np.clip((np.trace(Rerr) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    v = np.array([Rerr[2, 1] - Rerr[1, 2], Rerr[0, 2] - Rerr[2, 0], Rerr[1, 0] - Rerr[0, 1]])
    return v / 2 if th < 1e-6 else v * th / (2 * np.sin(th))


class Wbc:
    def __init__(self, model):
        self.m = model
        self.tau_max = np.concatenate([L.tau_max for L in model.legs.values()])
        self.nc = 6 + 20 + 12
        self.P = sp.diags(np.r_[np.full(6, W_FB), np.full(12, W_F)]).tocsc()
        self.A_qp = sp.csc_matrix(np.ones((self.nc, 18)))  # dense pattern, explicit entries everywhere (values set per solve)
        self.prob, self.n_fail = None, 0

    def solve(self, p, R, w, v, q, dq, f_mpc, stance, mu, body_ref, swing):
        u = np.r_[v, w, dq]
        A, b, g = dynamics(self.m, R, q, u)
        Ainv = np.linalg.inv(A)
        _, Jc = _elements(self.m, R, q)
        _, Jc2 = _elements(self.m, *_advance(R, q, u, EPS))
        Jcd_u = (Jc2 - Jc) @ u / EPS  # (4, 3)
        n18 = np.eye(18)
        # ---- 1. kinematic tasks, strict priorities
        st = np.flatnonzero(stance)
        if len(st):
            Jst = Jc[st].reshape(-1, 18)
            qdd = _pinv_dyn(Jst, Ainv) @ (-Jcd_u[st].ravel())
            N = n18 - np.linalg.pinv(Jst, rcond=1e-6) @ Jst
        else:
            qdd, N = np.zeros(18), n18.copy()
        p_ref, v_ref, R_ref = body_ref
        Jo, Jp = np.zeros((3, 18)), np.zeros((3, 18))
        Jo[:, 3:6], Jp[:, 0:3] = np.eye(3), np.eye(3)
        tasks = [(Jo, np.zeros(3), _log(R_ref @ R.T), np.zeros(3), KP_ORI, KD_ORI),
                 (Jp, np.zeros(3), p_ref - p, v_ref, KP_BODY, KD_BODY)]  # fmt: skip
        legs = list(self.m.legs.values())
        for i in np.flatnonzero(~stance):
            x = p + R @ legs[i].fk_jac(q[3 * i : 3 * i + 3])[0]
            tasks.append((Jc[i], Jcd_u[i], swing[0][i] - x, swing[1][i], KP_FOOT, KD_FOOT))
        dpos, dvel = np.zeros(18), np.zeros(18)
        for J, Jd_u, e, vd, kp, kd in tasks:
            JN = J @ N
            JN_pinv = np.linalg.pinv(JN, rcond=1e-6)
            dpos = dpos + JN_pinv @ (e - J @ dpos)
            dvel = dvel + JN_pinv @ (vd - J @ dvel)
            qdd = qdd + _pinv_dyn(JN, Ainv) @ (kp * e + kd * (vd - J @ u) - Jd_u - J @ qdd)
            N = N @ (n18 - JN_pinv @ JN)
        # ---- 2. dynamics QP
        f_ref = np.where(stance[:, None], f_mpc, 0.0).ravel()
        JcT = Jc.reshape(12, 18).T  # (18, 12)
        h = A @ qdd + b + g - JcT @ f_ref
        M = np.zeros((self.nc, 18))
        l, up = np.zeros(self.nc), np.zeros(self.nc)
        M[0:6, 0:6], M[0:6, 6:] = A[0:6, 0:6], -JcT[0:6]
        l[0:6] = up[0:6] = -h[0:6]
        for i in range(4):
            rows = np.array([[1, 0, -mu[i]], [-1, 0, -mu[i]], [0, 1, -mu[i]], [0, -1, -mu[i]], [0, 0, 1]])
            r0 = 6 + 5 * i
            M[r0 : r0 + 5, 6 + 3 * i : 9 + 3 * i] = rows
            fr = f_ref[3 * i : 3 * i + 3]
            l[r0 : r0 + 4], up[r0 : r0 + 4] = -np.inf, -(rows[:4] @ fr)
            l[r0 + 4], up[r0 + 4] = Mp.F_MIN * stance[i] - fr[2], Mp.F_MAX * stance[i] - fr[2]
        M[26:, 0:6], M[26:, 6:] = A[6:, 0:6], -JcT[6:]
        l[26:], up[26:] = -self.tau_max - h[6:], self.tau_max - h[6:]
        Ax = M.ravel(order="F")  # CSC of a dense pattern = column-major values
        if self.prob is None:
            self.A_qp.data = Ax
            self.prob = osqp.OSQP()
            self.prob.setup(self.P, np.zeros(18), self.A_qp, l, up, warm_start=True, verbose=False, polish=True,
                            eps_abs=1e-5, eps_rel=1e-5, max_iter=2000)  # fmt: skip
        else:
            self.prob.update(Ax=Ax, l=l, u=up)
        res = self.prob.solve()
        if res.info.status not in ("solved", "solved inaccurate"):
            self.n_fail += 1
            return WbcOut(q + dpos[6:], dvel[6:], np.zeros(12), f_mpc, False)
        z = res.x
        f = f_ref + z[6:]
        tau = np.clip(h[6:] + A[6:, 0:6] @ z[0:6] - JcT[6:] @ z[6:], -self.tau_max, self.tau_max)  # clip = solver tolerance only
        return WbcOut(q + dpos[6:], dvel[6:], tau, f.reshape(4, 3), True)


def make(model):
    try:
        from loco_wbc_cpp import Wbc as CppWbc  # Step 6, only if it was built

        return CppWbc(model)
    except ImportError:
        return Wbc(model)
```

  (`P` must be upper-triangular for OSQP: it is diagonal. Use the pinned osqp release's status test as in `loco/mpc.py`.)
- [ ] **Step 4: wire it in.** `loco/controller.py`: `LEG_CTLS = ("wbc", "jt")`; `__init__(self, model, hw, hmap, leg_ctl="wbc")` sets
  `self.leg_ctl`, `self.wbc = Wb.make(model) if leg_ctl == "wbc" else None` (`from . import wbc as Wb`), `self.body_ref = None`, `stats["wbc_fail"] = 0`; the swing
  block of `_legs` (from `if lp.s < 1e-9 ...` to the `lp.late` branch) moves unchanged into `_swing_ref(i, L, lp, st, cmd, r) -> (x_w,
  v_w)` which `_legs` calls (a pure refactor: the jt trot golden of Task 6 must reproduce before the default changes); `_mpc` sets
  `self.body_ref = (c_ref1 - R_ref1 @ m.com, v_ref1, R_ref1)` from step 1 of its reference (`R_ref1` from that step's rpy); and

```python
    def _legs_wbc(self, st, cmd, phases, q, dq):
        stance = np.array([lp.stance and not lp.late for lp in phases])
        sx, sv = np.zeros((4, 3)), np.zeros((4, 3))
        for i, (L, lp) in enumerate(zip(self.legs, phases)):
            r = L.fk_jac(q[3 * i : 3 * i + 3])[0]
            if stance[i]:
                self.lift[i] = st.p + st.R @ r
            else:
                sx[i], sv[i] = self._swing_ref(i, L, lp, st, cmd, r)
        if self.body_ref is None:
            self.body_ref = (st.p.copy(), np.zeros(3), Mp.rz(st.rpy[2]))
        out = self.wbc.solve(st.p, st.R, st.R @ st.w, st.v, q, dq, self.f, stance, self.mu, self.body_ref, (sx, sv))
        if not out.ok:
            self.stats["wbc_fail"] += 1
            return self._legs(st, cmd, phases, q, dq)  # this tick on J^T
        sw = np.repeat(~stance, 3)
        kp = np.where(sw, KP_SWING, 0.0)
        kd = np.where(sw, KD_SWING, KD_STANCE)
        return np.where(sw, out.p_des, q), out.v_des, kp, kd, out.tau
```

  and `step` sends `(self._legs_wbc if self.wbc else self._legs)(st, cmd, phases, q, dq)`. (`st.w` is the trunk-frame rate of the IMU;
  the WBC takes it in the world frame.) `dogtruth.Truth.forces()`: `np.array([np.linalg.norm(self.sim.query(f, "force")[0]) for f in
  FEET])`. `dogwalk`: `DEFAULTS["leg_ctl"] = "wbc"`, `Ct.Controller(..., leg_ctl=p["leg_ctl"])`, new columns `f_track` (per row: RMS
  over feet that are in true contact and in the controller's MPC stance of `|F_true_i| - |f_mpc_i|`, divided by `m g` of the
  controller's model; 0 when no foot qualifies) and `wbc_fail`; `res["f_track_rms"]` = RMS of `f_track` over rows with t > 3 s,
  `res["wbc_fail"] = ctl.stats["wbc_fail"]`. `trot.scenario.toml` gains

```toml
[[parameters]]
name = "leg_ctl"
type = "choice"
choices = ["wbc", "jt"]
default = "wbc"
group = "controller"
label = "Leg controller"
doc = "wbc: QP whole-body controller (WBIC-style, the SP4 design). jt: Jacobian-transpose leg controller, the measured baseline."
```

  (Tasks 8 and 9 create their scenarios "as trot", so they carry it too.)
- [ ] **Step 5: run, measure, re-record.** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_wbc.py tests/python/test_dog_model.py tests/python/test_dog_walk.py -n 8 -s'`.
  First with `DEFAULTS["leg_ctl"]` and the scenario default still `"jt"`: `test_dog_walk.py` must pass with the Task 6 golden (the refactor
  changed nothing). Then switch both defaults to `"wbc"`, re-record `GOLDEN["trot"]` (as Task 6 Step 5) and rerun: all PASS. Tune only
  the `KP_*` / `KD_*` / `W_*` constants of `wbc.py`, each with its reason; never a test bound. Write `docs/dog/locomotion.md` "Whole-body
  control" (Edit): WBC solve time median / p99 (and the C++ one if Step 6 ran), force-tracking RMS wbc vs jt, 1 m/s trot tracking for
  both, WBC fallback count.
- [ ] **Step 6 (only if `test_solve_time_budget` fails in Python): the C++ path.** Port `dynamics`, `_elements` and `Wbc.solve` one to
  one to `examples/dog/loco/cpp/wbc.cpp` (Eigen 3.4 for the small dense algebra, OSQP's C API `osqp_setup` / `osqp_update_data_mat` /
  `osqp_update_data_vec` / `osqp_solve` with the same dense pattern), as a nanobind module `loco_wbc_cpp` whose `Wbc(model)` reads the
  same Python `Model` fields (`legs[*].w, p, foot0, tau_max`, `links`, `armature`) once and exposes `solve(...)` with the Python
  signature, returning the same `WbcOut` fields. Build: `option(NAPL_LOCO_CPP "dog whole-body controller in C++ (SP4)" OFF)` in
  `CMakeLists.txt` with `add_subdirectory(examples/dog/loco/cpp)` under it; in `cmake/deps.cmake` under `if(NAPL_LOCO_CPP)`
  FetchContent Eigen (tag 3.4.0, MPL-2.0) and osqp (the C release matching the pinned Python wheel, Apache-2.0); the module's output
  directory is `${CMAKE_BINARY_DIR}/loco`, which `dogwalk.py`, `dogtop.py`, `dogmc.py` and the tests put on `sys.path` when it exists.
  Append to `tests/python/test_dog_wbc.py`:

```python
def test_cpp_wbc_matches_python(m):
    cpp = pytest.importorskip("loco_wbc_cpp")
    args, *_ = trot_problem(m)
    rng = np.random.default_rng(3)
    for _ in range(20):
        a = list(args)
        a[5] = rng.normal(0, 3.0, 12)
        py, cc = Wb.Wbc(m).solve(*a), cpp.Wbc(m).solve(*a)
        assert py.ok == cc.ok and all(np.allclose(x, y, atol=1e-6) for x, y in zip(py[:4], cc[:4]))
```

  Build and run: `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run 'cmake --preset dev -DNAPL_LOCO_CPP=ON && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_wbc.py -n 8 -s && .venv/bin/python tools/check_licenses.py'`
  (budget test now times the C++ one through `make`; check_licenses 0 problems with the two new `_deps`). Still over budget in C++:
  stop and report to the orchestrator (the WBC would then run at 250 Hz with joint PD between solves, a spec deviation for the user).
- [ ] **Step 7: commit** `git add examples/dog/loco/wbc.py examples/dog/loco/model.py examples/dog/loco/controller.py examples/dog/dogtruth.py examples/dog/dogwalk.py examples/dog/walk/trot.scenario.toml tests/python/test_dog_wbc.py tests/python/test_dog_walk.py docs/dog/locomotion.md`
  (plus `CMakeLists.txt cmake/deps.cmake examples/dog/loco/cpp` if Step 6 ran) `&& git commit -m "dog loco: QP whole-body controller (WBIC-style, OSQP, torque and friction limits) as the default leg controller; J^T kept as the switchable baseline" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 8: terrain-aware footholds, posture, step height; courses (criterion, terrain families, truth routes); stairs + rubble scenarios

**Files:**
- Modify: `examples/dog/loco/terrain.py` (`foothold`, `apex_over`, `plane`, `rough_ahead` real bodies), `examples/dog/dogcourse.py`
  (courses `criterion`, `slip`, `rubble`, `slopes`, `scree`, `ruins`; `field_of`, `criterion_course`, `route`, `Follower`)
- Create: `examples/dog/walk/{stairs,rubble}.scenario.toml`, `examples/dog/walk/{stairs,rubble}.py` (as `trot.py`)
- Modify: `docs/dog/locomotion.md` (section "Terrain")
- Test: `tests/python/test_dog_terrain_loco.py`

**Interfaces:**
- Consumes: `napl.terrain.field`, `napl.terrain.add_field`, `napl.terrain.Field`, `napl.terrain.Patch`.
- Produces:
  - `loco.terrain`: `EDGE_OK`, `W_EDGE`, `W_SLOPE`, `foothold(hmap, xy, r_foot, r=0.12, step=0.02, edge_r=0.04) -> (3,)` (ball
    centre: ground + r_foot), `apex_over(hmap, p0, p1, apex, clear=0.05)`, `plane(hmap, xy, yaw) -> (roll, pitch, z)`,
    `rough_ahead(hmap, xy, yaw, dist) -> bool`.
  - `dogcourse`: `FIELD_SIZE = 30.0`, `criterion_course() -> Field`, `field_of(info) -> Field | None`, `route(f, rng, ...) -> (n, 2)`,
    `Follower(route, v)` with `.cmd(pos, yaw) -> (vx, vy, wz)`, `.done`, `.dist`, `.length`, `.t_done`; `scene()` accepts every
    course of the scenario choice list.

- [ ] **Step 1: failing test** `tests/python/test_dog_terrain_loco.py`:

```python
"""SP4 Task 8: terrain. Footholds off step edges and holes, swing apex over a step, posture plane on a 25 deg ramp (pure math on synthetic maps);
the criterion course (25 deg ramp + 15 cm steps, spec s7) and truth routes; the stairs and rubble scenarios with golden hashes."""

import math
import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C  # noqa: E402
from loco import terrain as Tr  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = {"stairs": None, "rubble": None}  # recorded in Step 5


class StepMap:  # 15 cm step up at x = 0.30, a hole at x in [0.6, 0.7]
    def height(self, x, y):
        x = np.asarray(x, float)
        h = np.where(x > 0.30, 0.15, 0.0)
        return np.where((x > 0.6) & (x < 0.7), np.nan, h) + 0.0 * np.asarray(y, float)


class Ramp:
    def height(self, x, y):
        return np.tan(math.radians(25)) * np.asarray(x, float) + 0.0 * np.asarray(y, float)


def test_goldens_are_recorded():
    assert all(GOLDEN.values())


def test_foothold_keeps_off_step_edges_and_holes():
    p = Tr.foothold(StepMap(), np.array([0.31, 0.0]), 0.0175)
    assert abs(p[0] - 0.30) > 0.04 and p[2] - 0.0175 == pytest.approx(StepMap().height(p[0], p[1]))
    assert not 0.6 <= Tr.foothold(StepMap(), np.array([0.65, 0.0]), 0.0175)[0] <= 0.7
    flat = Tr.foothold(Tr.FlatMap(), np.array([1.0, 2.0]), 0.0175)
    assert flat == pytest.approx([1.0, 2.0, 0.0175])


def test_swing_clears_a_step_and_posture_follows_a_ramp():
    assert Tr.apex_over(StepMap(), np.array([0.1, 0, 0.0175]), np.array([0.5, 0, 0.1675]), 0.08) >= 0.05 - 1e-9
    assert Tr.apex_over(StepMap(), np.array([0.1, 0, 0.0175]), np.array([0.25, 0, 0.0175]), 0.08) == pytest.approx(0.08)
    roll, pitch, _ = Tr.plane(Ramp(), np.array([0.0, 0.0]), 0.0)
    assert math.degrees(-pitch) == pytest.approx(25.0, abs=0.5) and abs(roll) < 1e-6  # nose-up pitch is negative about +y
    assert Tr.rough_ahead(StepMap(), np.array([0.0, 0.0]), 0.0, 0.8) and not Tr.rough_ahead(Ramp(), np.array([0.0, 0.0]), 0.0, 0.8)


def test_criterion_course_has_the_spec_ramp_and_steps():
    f = C.criterion_course()
    x = np.arange(-14.0, 14.0, 0.05)
    z = f.height(x, np.zeros_like(x))
    slopes = np.degrees(np.arctan(np.diff(z) / 0.05))
    ramp = slopes[(slopes > 5) & (slopes < 40)]
    assert ramp.max() == pytest.approx(25.0, abs=0.5)
    risers = np.diff(z)[np.abs(np.diff(z)) > 0.1]
    assert sorted(np.round(np.abs(risers), 3)) == [0.15] * 4


@pytest.mark.parametrize("kind", ["rubble", "slopes", "scree", "ruins"])
def test_truth_routes_stay_on_traversable_ground(kind):
    from napl.terrain import field

    f = field(kind, size=C.FIELD_SIZE, seed=7)
    r = C.route(f, np.random.default_rng([7, 31]))
    seg = np.linalg.norm(np.diff(r, axis=0), axis=1)
    assert 15.0 <= seg.sum() <= 40.0 and seg.max() <= 0.75
    xs = np.concatenate([np.linspace(a, b, 6) for a, b in zip(r[:-1], r[1:])])
    h = f.height(xs[:, 0], xs[:, 1])
    assert np.abs(np.diff(h)).max() <= 0.20  # never through a boulder / wall


@needs_kernel
@pytest.mark.parametrize("name", ["stairs", "rubble"])
def test_scenario_passes_replays_and_matches_golden(tmp_path, name):
    res, _ = program.run(EX / "walk" / f"{name}.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    assert GOLDEN[name] is None or res["sim"]["state_hash"] == GOLDEN[name], res["sim"]["state_hash"]
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_terrain_loco.py -n 8'`
- [ ] **Step 3: implement `loco/terrain.py` functions:**

```python
EDGE_OK = 0.015  # m: height spread within 4 cm that the 35 mm rubber ball sits on without rolling off
W_EDGE, W_SLOPE = 4.0, 2.0  # cost per cm of spread above EDGE_OK, per rad of local slope; distance term = |d|^2 / r^2 (1 at the rim)
POSTURE_PATCH = (0.6, 0.4)  # m: footprint the posture plane is fitted over (about the stance rectangle)


def foothold(hmap, xy, r_foot, r=0.12, step=0.02, edge_r=0.04):
    g = np.arange(-r, r + 1e-9, step)
    X, Y = np.meshgrid(g, g)
    keep = X**2 + Y**2 <= r * r + 1e-12
    cx, cy = xy[0] + X[keep], xy[1] + Y[keep]
    h0 = hmap.height(cx, cy)
    ang = np.arange(8) * np.pi / 4
    hn = np.stack([hmap.height(cx + edge_r * np.cos(a), cy + edge_r * np.sin(a)) for a in ang])
    spread = np.max(np.abs(hn - h0), axis=0)
    cost = (X[keep] ** 2 + Y[keep] ** 2) / r**2 + W_EDGE * np.clip(spread - EDGE_OK, 0, None) / 0.01 + W_SLOPE * np.arctan(spread / edge_r)
    cost[~np.isfinite(h0) | ~np.isfinite(spread)] = np.inf
    k = int(np.argmin(cost))
    if not np.isfinite(cost[k]):
        return np.array([xy[0], xy[1], np.nan])  # nothing known around the nominal: the controller keeps the nominal at the last height
    return np.array([cx[k], cy[k], h0[k] + r_foot])


def apex_over(hmap, p0, p1, apex, clear=0.05):
    s = np.linspace(0, 1, 12)
    pts = p0[None, :2] + (p1[:2] - p0[:2])[None, :] * s[:, None]
    top = np.nanmax(hmap.height(pts[:, 0], pts[:, 1]))
    return max(apex, top + clear - (max(p0[2], p1[2]) - 0.0175))  # 0.0175 = ball radius: p0 / p1 are ball centres


def plane(hmap, xy, yaw):
    """Least-squares plane z = a x' + b y' + c over the body footprint in the yaw frame -> (roll, pitch, c)."""
    lx, ly = POSTURE_PATCH
    u, v = np.meshgrid(np.linspace(-lx / 2, lx / 2, 5), np.linspace(-ly / 2, ly / 2, 5))
    c, s = np.cos(yaw), np.sin(yaw)
    z = hmap.height(xy[0] + c * u - s * v, xy[1] + s * u + c * v)
    ok = np.isfinite(z)
    if ok.sum() < 6:
        return 0.0, 0.0, float(np.nanmean(z)) if ok.any() else 0.0
    A = np.column_stack([u[ok], v[ok], np.ones(ok.sum())])
    a, b, c0 = np.linalg.lstsq(A, z[ok], rcond=None)[0]
    return float(np.arctan(b)), float(-np.arctan(a)), float(c0)


def rough_ahead(hmap, xy, yaw, dist):
    s = np.arange(0.0, dist, 0.05)
    z = hmap.height(xy[0] + np.cos(yaw) * s, xy[1] + np.sin(yaw) * s)
    return bool(np.nanmax(np.abs(np.diff(z))) > 0.08)  # a riser above 8 cm per 5 cm ahead: walk (a 25 deg ramp is 2.3 cm per 5 cm)
```

  In `controller._legs`, a nan `target[i][2]` keeps the nominal xy at the lift-off height (unknown map). `dogcourse` additions:

```python
FIELD_SIZE = 30.0  # m: 3 x 3 patches of 10 m (napl.terrain.field), the 100 m statistics on a ~9x smaller load
COURSES = ("flat", "criterion", "slip", "rubble", "slopes", "scree", "ruins")


def criterion_course():
    """Spec s7 '25-degree slopes and 15 cm steps traversed', along +x at y in [-15, 15]: flat to x = -9, a 25 deg ramp up 3 m (1.40 m), 2 m
    plateau, two 15 cm steps up and two down 2 m apart (one-cell risers, as napl.terrain's own steps), 2 m, a 25 deg ramp down, flat. Rock."""
    from napl.terrain import Field, Patch

    res, n = 0.05, int(FIELD_SIZE / 0.05) + 1
    x = -FIELD_SIZE / 2 + res * np.arange(n)
    t25 = np.tan(np.radians(25))
    top = 3.0 * t25
    z = np.select(
        [x < -9, x < -6, x < -4, x < -2, x < 0, x < 2, x < 4, x < 6, x < 9],
        [0 * x, (x + 9) * t25, top + 0 * x, top + 0.15 + 0 * x, top + 0.30 + 0 * x, top + 0.15 + 0 * x, top + 0 * x, top + 0 * x,
         top - (x - 6) * t25],
        0 * x,
    )  # fmt: skip
    h = np.tile(z, (n, 1))
    pc = int(10.0 / res)
    patches = [Patch(-15 + a * 10, -15 + b * 10, -5 + a * 10, -5 + b * 10, "rock", a * pc, (a + 1) * pc, b * pc, (b + 1) * pc)
               for b in range(3) for a in range(3)]  # fmt: skip
    f = Field("slopes", FIELD_SIZE, res, 0, h, patches, [], None, [], 10.0, {"criterion": True})
    f.grid = np.full((3, 3), "rock", dtype=object)
    return f
```

  `field_of(info)`: `None` for flat; `criterion_course()`; `slip` = a flat field (`heights` zero) whose middle patch column is
  `"mud"` and the rest `"rock"`; the four families `napl.terrain.field(kind, size=FIELD_SIZE, seed=info["seed"])`. `scene()` for a
  field course: remove the nominal MJCF's floor (`worldbody` geoms of `type="plane"`), `add_field(root, f)`, place the root body:
  `start` = route start (families) or `(-12, 0)` (criterion, slip; route = straight line to `(12, 0)` every 0.5 m), yaw toward the
  second waypoint, z = the nominal root z (straight legs on a z = 0 floor) + the max ground height under a 0.7 x 0.5 m footprint +
  0.01; write route and start into course.json. `route(f, rng, length=(15.0, 20.0), res=0.1, max_step=0.20, inflate=0.3, tries=50)`:
  docstring "Truth-traversable route for the HARNESS (SP5 plans on its own map)": grid of `f.height` at `res` with a 1 m margin;
  blocked where any 8-neighbour differs by more than `max_step` (boulders, ruin walls; the 10-18 cm steps of `slopes` and rubble rocks
  stay open), dilated by `inflate` (numpy rolls); start / goal on free cells drawn until their distance is within `length`; A*
  (heapq, 8-connected, cost = distance + 2 |dz|); waypoints every 0.5 m; `RuntimeError` after `tries`. `Follower(route, v,
  lookahead=0.8)`: pure pursuit on the true pose; `wz = clip(2.0 e, -1.5, 1.5)` with e the heading error to the look-ahead point,
  `vx = v cos(e)^2` (slows in turns), `vy = 0`; `done` within 0.5 m of the last waypoint (then the command is zero); `dist` = remaining
  route length; `length`, `t_done` (time when done, from its own call count x `0.002`).
  Scenarios: `stairs.scenario.toml` (as trot: `name = "dog-stairs"`, `course = "criterion"` default, `v_cmd` 0.8, `duration` 40;
  monitors `reaches_the_end` (column `dist_goal`, `op = "final"`, `hi = 0.5`), `never_falls`, `no_qdd_fault`, `stays_upright` (`tilt`
  always `hi = 35.0`: the 25 deg ramp plus 10 deg of gait motion)); `rubble.scenario.toml` (`name = "dog-rubble"`, `course =
  "rubble"`, `seed` 1, `v_cmd` 1.8, `duration` 25; monitors `reaches_the_end`, `never_falls`, `no_qdd_fault`, `rocky_mean_speed`
  (column `v_mean_2s`, `op = "eventually"`, `lo = 1.5`: spec s7, the per-run peak, the MC reports the mean)).
- [ ] **Step 4: run, expect PASS except the goldens:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_terrain_loco.py tests/python/test_dog_walk.py -n 8'`
  (the trot golden must stay: flat-map footholds return the nominal exactly). Tune only `EDGE_OK`, `W_EDGE`, `W_SLOPE`, the `walk`
  gait and the 0.08 m riser threshold, each with its reason.
- [ ] **Step 5: record the goldens** (as Task 6 Step 5, for `stairs` and `rubble`) and write `docs/dog/locomotion.md` "Terrain"
  (criterion course: time, mean speed, max tilt; rubble seed 1: mean speed).
- [ ] **Step 6: commit** `git add examples/dog/loco/terrain.py examples/dog/loco/controller.py examples/dog/dogcourse.py examples/dog/walk docs/dog/locomotion.md tests/python/test_dog_terrain_loco.py && git commit -m "dog loco: terrain-aware footholds, posture plane, step height; criterion course, terrain-family scenes, truth routes; stairs and rubble scenarios" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 9: slip detection and recovery, fall detection, safe-sit and stand-up, push rejection

**Files:**
- Create: `examples/dog/loco/safety.py`, `examples/dog/walk/push.scenario.toml`, `examples/dog/walk/push.py`
- Modify: `examples/dog/loco/controller.py` (safety hooks), `docs/dog/locomotion.md` (section "Slips, pushes, falls")
- Test: `tests/python/test_dog_safety.py`

**Interfaces:**
- Consumes: `loco.estimator.State`, `loco.gait.LegPhase`, `loco.controller.Controller`.
- Produces: `loco.safety`: `SLIP_V`, `SLIP_TICKS`, `MU_NOM`, `MU_MIN`, `MU_DROP`, `MU_RECOVER`, `SPEED_SLIP`, `T_SLOW`, `WIDEN`,
  `TILT_FALL`, `TILT_FALL_FAST`, `W_FALL`, `TILT_STANDUP`, `KP_SIT`, `KD_SIT`, `T_FALL_WAIT`, `Safety()` with `.update(st, phases, q, dq,
  legs, dt) -> "slip" | None`, `.fallen(st, h_stand) -> bool`, `.mu (4,)`, `.speed_scale`, `.widen`; controller modes `FALLEN` (safe-sit)
  and `SIT`.

- [ ] **Step 1: failing test** `tests/python/test_dog_safety.py`:

```python
"""SP4 Task 9: slip detection (synthetic stance-foot velocities; a mud strip), push rejection (golden scenario), fall -> safe-sit without driver
faults or stiff joints."""

import sys
from pathlib import Path
from types import SimpleNamespace

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C  # noqa: E402
import dogwalk as W  # noqa: E402
from loco import controller as Ct  # noqa: E402
from loco import safety as S  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = {"push": None}  # recorded in Step 5
c = W.COLS.index


class FlatLeg:
    def fk_jac(self, q):
        return np.array(q, float), np.eye(3)


def stance_state(v_foot):
    st = SimpleNamespace(R=np.eye(3), v=np.zeros(3), w=np.zeros(3), contact=np.ones(4, bool), rpy=np.zeros(3), h_map=0.35)
    return st, np.tile([0.2, 0.1, -0.35], 4), np.tile(v_foot, 4)


def test_goldens_are_recorded():
    assert all(GOLDEN.values())


def test_slip_lowers_mu_slows_and_recovers():
    s = S.Safety()
    phases = [SimpleNamespace(stance=True, late=False, early=False, s=0.0)] * 4
    legs = [FlatLeg()] * 4
    st, q, dq = stance_state([0.0, 0.0, 0.0])
    for _ in range(50):
        assert s.update(st, phases, q, dq, legs, 0.002) is None
    st, q, dq = stance_state([0.5, 0.0, 0.0])  # feet sliding at 0.5 m/s while in stance
    ev = [s.update(st, phases, q, dq, legs, 0.002) for _ in range(S.SLIP_TICKS + 1)]
    assert "slip" in ev and s.mu.min() <= S.MU_NOM - S.MU_DROP + 1e-9 and s.speed_scale < 1.0
    st, q, dq = stance_state([0.0, 0.0, 0.0])
    for _ in range(int(10 / 0.002)):
        s.update(st, phases, q, dq, legs, 0.002)
    assert s.mu.min() == pytest.approx(S.MU_NOM) and s.speed_scale == 1.0


def test_fall_detector_thresholds():
    s = S.Safety()
    up = SimpleNamespace(rpy=np.array([0.1, 0.0, 0.0]), w=np.zeros(3), h_map=0.35)
    assert not s.fallen(up, 0.35)
    assert s.fallen(SimpleNamespace(rpy=np.array([np.radians(S.TILT_FALL + 1), 0, 0]), w=np.zeros(3), h_map=0.35), 0.35)
    assert s.fallen(SimpleNamespace(rpy=np.array([np.radians(S.TILT_FALL_FAST + 1), 0, 0]), w=np.array([S.W_FALL + 1, 0, 0]), h_map=0.35), 0.35)
    assert s.fallen(SimpleNamespace(rpy=np.zeros(3), w=np.zeros(3), h_map=0.1), 0.35)


@needs_kernel
def test_slip_on_mud_is_detected_and_survived(tmp_path):
    r = W.run(C.scene(tmp_path, "slip", seed=1), dict(W.DEFAULTS, course="slip", v_cmd=1.0, duration=30.0), log=False)
    rows = np.array(r["rows"])
    assert r["ctl"].stats["n_slip"] >= 1 and rows[:, c("mu_min")].min() <= S.MU_NOM - S.MU_DROP + 1e-9
    assert rows[:, c("fallen")].max() == 0 and r["res"]["reached"]


@needs_kernel
def test_a_fall_ends_in_safe_sit_without_faults(tmp_path):
    p = dict(W.DEFAULTS, v_cmd=1.0, duration=8.0, push_t=4.0, push_dv=3.0)  # 36 N s sideways: beyond any recovery
    r = W.run(C.scene(tmp_path, "flat", seed=1), p, log=False)
    rows = np.array(r["rows"])
    t = rows[:, c("t")]
    tilt_over = t[rows[:, c("tilt")] > 45.0]
    ctl_fell = t[rows[:, c("ctl_fallen")] > 0]
    assert len(ctl_fell) and (not len(tilt_over) or ctl_fell[0] <= tilt_over[0] + 0.3)  # detected within 0.3 s of the truth
    assert rows[:, c("fault")].max() == 0
    assert r["ctl"].mode in ("FALLEN", "SIT", "STANDUP", "STAND", "WALK")
    if r["ctl"].mode in ("FALLEN", "SIT"):
        assert np.all(r["ctl"].hw._kp <= S.KP_SIT)  # limp, not stiff, on the ground


@needs_kernel
def test_push_scenario_passes_replays_and_matches_golden(tmp_path):
    res, _ = program.run(EX / "walk" / "push.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    assert GOLDEN["push"] is None or res["sim"]["state_hash"] == GOLDEN["push"], res["sim"]["state_hash"]
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_safety.py -n 8'`
- [ ] **Step 3: implement `examples/dog/loco/safety.py`:**

```python
"""Slip and fall handling of the dog (SP4).
Slip: a stance foot (estimated contact, not the first 20 ms after touchdown) whose world velocity v + R (J dq + w x r) exceeds SLIP_V for
SLIP_TICKS ticks: lower that foot's MPC friction by MU_DROP (floor MU_MIN), widen the stance by WIDEN, scale the speed command to SPEED_SLIP
for T_SLOW s; mu recovers at MU_RECOVER per s (Focchi et al. 2018 slip detection / friction re-estimation, simplified to a per-foot step).
Fall: estimated tilt > TILT_FALL, or tilt > TILT_FALL_FAST while rolling faster than W_FALL, or the body below 50 % of the stand height over
the map -> FALLEN: legs fold to a sit (KP_SIT / KD_SIT) then go limp (damping only); after T_FALL_WAIT, stand up if tilt < TILT_STANDUP.
No self-righting (a fall is a fall)."""

import numpy as np

SLIP_V, SLIP_TICKS = 0.25, 3  # m/s, ticks (6 ms): well above the estimator's ~0.05 m/s foot-velocity noise
MU_NOM, MU_MIN, MU_DROP, MU_RECOVER = 0.5, 0.2, 0.15, 0.05  # mud is 0.25 (napl.terrain MATERIALS)
SPEED_SLIP, T_SLOW, WIDEN = 0.6, 2.0, 0.03
TILT_FALL, TILT_FALL_FAST, W_FALL = 40.0, 25.0, 4.0  # deg, deg, rad/s
TILT_STANDUP, T_FALL_WAIT = 30.0, 1.5
KP_SIT, KD_SIT = 20.0, 1.0  # soft: a falling leg must not slam (kd_timeout of the drivers is 1.0, SP3b)


class Safety:
    def __init__(self):
        self.mu, self.speed_scale, self.widen = np.full(4, MU_NOM), 1.0, 0.0
        self.count, self.t_slow, self.t_stance = np.zeros(4, int), 0.0, np.zeros(4)

    def update(self, st, phases, q, dq, legs, dt):
        ev = None
        for i, (L, lp) in enumerate(zip(legs, phases)):
            stance = lp.stance and st.contact[i]
            self.t_stance[i] = self.t_stance[i] + dt if stance else 0.0
            if stance and self.t_stance[i] > 0.02:
                r, J = L.fk_jac(q[3 * i : 3 * i + 3])
                vf = st.v + st.R @ (J @ dq[3 * i : 3 * i + 3] + np.cross(st.w, r))
                self.count[i] = self.count[i] + 1 if np.linalg.norm(vf[:2]) > SLIP_V else 0
                if self.count[i] == SLIP_TICKS:
                    self.mu[i] = max(MU_MIN, self.mu[i] - MU_DROP)
                    self.t_slow, self.widen, ev = T_SLOW, WIDEN, "slip"
        self.mu = np.minimum(MU_NOM, self.mu + MU_RECOVER * dt * (self.count == 0))
        self.t_slow = max(0.0, self.t_slow - dt)
        self.speed_scale = SPEED_SLIP if self.t_slow > 0 else 1.0
        self.widen = self.widen if self.t_slow > 0 else 0.0
        return ev

    def fallen(self, st, h_stand):
        tilt = np.degrees(np.hypot(st.rpy[0], st.rpy[1]))
        return tilt > TILT_FALL or (tilt > TILT_FALL_FAST and np.linalg.norm(st.w[:2]) > W_FALL) or st.h_map < 0.5 * h_stand
```

  Controller hooks (replace the Task 6 placeholder comment in `step`): in STAND / WALK, `if self.safety.update(st, phases_prev, q, dq,
  self.legs, DT) == "slip": self.stats["n_slip"] += 1`; `self.mu = self.safety.mu`; the command is scaled by `safety.speed_scale`
  before `choose` / the reference; `widen` moves each nominal foothold outward (away from the body centre line) in `_legs`; `if
  self.safety.fallen(st, self.h_stand()): self._enter("FALLEN")`. FALLEN: for `T_FALL_WAIT / 3` ramp the joints to
  `m.stand_q(0.55)` with `KP_SIT`, `KD_SIT`, then `kp = 0`, `kd = KD_SIT`; after `T_FALL_WAIT`, if the estimated tilt is below
  `TILT_STANDUP`, `est.reset(q)` and enter STANDUP (from the current q), else enter SIT (limp, stays). `push.scenario.toml`: as trot
  (`name = "dog-push"`, `v_cmd` 1.0, `duration` 10, parameters `push_t` 5.0 s and `push_dv` 0.5 m/s, declared `[[parameters]]`),
  monitors `never_falls`, `no_qdd_fault`, `recovers_the_speed` (`v_mean_2s` final in [0.85, 1.15]: 3 s after the push), and
  `stays_upright` (`tilt` always `hi = 20.0`).
- [ ] **Step 4: run, expect PASS except the golden:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_safety.py tests/python/test_dog_walk.py tests/python/test_dog_terrain_loco.py -n 8'`.
  The trot / stairs / rubble goldens may move only if the safety hooks changed those runs (a slip detected on rubble, say): then
  re-record them in the same commit and write why in the commit message.
- [ ] **Step 5: record the push golden** (as Task 6 Step 5) and write `docs/dog/locomotion.md` "Slips, pushes, falls": the push
  envelope from a ladder `push_dv` in {0.25, 0.5, 0.75, 1.0, 1.25, 1.5} m/s, 5 seeds each (`dogwalk.run` in a short loop on hfbox,
  printed table), the mud-strip result, and the "no self-righting" gap.
- [ ] **Step 6: commit** `git add examples/dog/loco/safety.py examples/dog/loco/controller.py examples/dog/walk tests/python/test_dog_safety.py tests/python/test_dog_walk.py tests/python/test_dog_terrain_loco.py docs/dog/locomotion.md && git commit -m "dog loco: slip detection + friction re-estimate, fall detection, safe-sit and stand-up, push scenario with golden hash" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 10: speed: flying trot and bound, top-speed search for both leg controllers, the SP3b sweep slot, honest report

**Files:**
- Create: `examples/dog/dogtop.py`
- Modify: `examples/dog/loco/gait.py` (tuned `GAITS` values, with the run that chose them), `tools/dog_sweep.py` (one call:
  the gait slot gets the variant's MJCF path), `docs/dog/locomotion.md` (section "Top speed"), `docs/dog/speed-sweep.md`,
  `docs/dog/speed-sweep.json`, `assemblies/dog/README.md` (speed line + sweep table from the rerun)
- Test: `tests/python/test_dog_topspeed.py`

**Interfaces:**
- Consumes: `dogwalk.run`, `dogcourse.scene`, `loco.controller.Controller` (`sched.set_params`, `h_frac`, `stats`), Qdd attrs `tau_cmd`,
  `tau_est`, `dq` (harness-side diagnostics only), battery `current`, `voltage`; `tools/dog_sweep.py --gait`.
- Produces: `dogtop`: `V_TARGET = 3.5`, `GRID`, `measure(model, gait, params, v_try, seed=1, leg_ctl="wbc") -> dict(ok, v_mean, diag)`,
  `ladder(model, gait, params, seed=1, leg_ctl="wbc") -> dict(v_top, runs)`, `report_md(res) -> str` (`res["by_leg"]` = v_top per leg
  controller with the best params of each), `sweep_gait(sim, sg, report) -> float` (WBC), CLI
  `python examples/dog/dogtop.py [--gaits trot,flying_trot,bound] [--leg wbc,jt] [--search] [--validate N] [--jobs N] [--out DIR]` writing
  `<out>/topspeed.json` and `<out>/topspeed.md`.

- [ ] **Step 1: failing test** `tests/python/test_dog_topspeed.py`:

```python
"""SP4 Task 10: top speed. The report states the measured speed and the gap to the 3.5 m/s criterion honestly (pure); flying trot beats the floor on
flat ground; the SP3b sweep slot runs on a sweep model and refuses a report without the model path."""

import sys
from pathlib import Path

import napl
import pytest

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogcourse as C  # noqa: E402
import dogsim as D  # noqa: E402
import dogtop as T  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
FLOOR = 2.0  # m/s: regression floor below the measured top speed (Step 5 rule); the study's torque bound for a trot is 2.4 m/s


def res(v):
    return dict(gait="flying_trot", leg_ctl="wbc", v_top=v, params=dict(period=0.28, duty=0.38), by_leg=dict(wbc=v, jt=v - 0.3),
                validate=dict(n=10, median=v, min=v - 0.1),
                diag=dict(sat_hip=0.42, sat_knee=0.10, w_frac=0.93, n_slip=3, i_pack_peak=81.0, cot=0.9))  # fmt: skip


def test_report_states_measured_top_speed_and_gap():
    md = T.report_md(res(2.71))
    assert "flat top speed (measured, flying_trot, wbc): 2.71 m/s" in md
    assert "spec target 3.5 m/s: NOT met (gap 0.79 m/s)" in md
    assert "leg controller: wbc 2.71 m/s, jt 2.41 m/s" in md  # the WBC is measured against the J^T baseline, not assumed better
    assert "hip torque at its limit" in md and "of no-load speed" in md  # the limiting factors are named, not guessed
    assert "spec target 3.5 m/s: met" in T.report_md(res(3.6))


@needs_kernel
@pytest.mark.parametrize("leg_ctl", ["wbc", "jt"])
def test_flying_trot_beats_the_floor(tmp_path, leg_ctl):
    r = T.measure(C.scene(tmp_path, "flat", seed=1), "flying_trot", {}, FLOOR, leg_ctl=leg_ctl)
    assert r["ok"], r["diag"]


@needs_kernel
def test_sweep_gait_slot(tmp_path):
    xml, rep = D.build(tmp_path)
    sim = napl.Sim.load(xml)
    with pytest.raises(ValueError, match="xml"):
        T.sweep_gait(sim, D.signs(rep), rep)
    assert T.sweep_gait(napl.Sim.load(xml), D.signs(rep), {**rep, "xml": str(xml)}) > 1.0
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_topspeed.py -n 8'`
- [ ] **Step 3: implement `examples/dog/dogtop.py`** (key code; harness side, may read truth):

```python
"""Top speed of the dog on flat ground (SP4, spec s7 'flat top speed >= 3.5 m/s'; docs/dog/actuator-study.md: torque-limited trot bound 2.4 m/s).
measure(): stand up, ramp the command at 1 m/s^2 to v_try, hold 4 s; ok when the TRUE mean forward speed over the last 3 s >= 0.95 v_try, no
fall, no driver fault, MPC failures < 5 % of solves. ladder(): v_try = 1.5, 1.75, ... until two consecutive failures; v_top = the last ok.
Diagnostics of the best and the first failing run: hip / knee fraction of stance ticks with |tau_est| >= 0.95 x the torque the driver could
give at that speed (the Qdd envelope, docs/models.md "Qdd"), peak |dq| as a fraction of the no-load speed at the bus voltage, slips, pack
peak current, cost of transport (mean pack power / (m g v)). --search: GRID per gait on TRAIN seed 1001, best by v_top then CoT; --validate N:
the best params on N held-out dispersion seeds (21000...: payload 0-1 kg, IMU biases) -> median / min v_top. Nothing here is called a top
speed unless it was measured this way."""

V_TARGET = 3.5
GRID = {
    "trot": dict(period=(0.26, 0.30, 0.34), duty=(0.5,), apex=(0.06, 0.08), h_frac=(0.78, 0.82)),
    "flying_trot": dict(period=(0.24, 0.28, 0.32), duty=(0.30, 0.35, 0.40), apex=(0.06, 0.08), h_frac=(0.78, 0.82)),
    "bound": dict(period=(0.22, 0.26, 0.30), duty=(0.35, 0.40, 0.45), apex=(0.05, 0.07), h_frac=(0.78, 0.82)),
}


def report_md(res):
    v, gap = res["v_top"], V_TARGET - res["v_top"]
    d = res["diag"]
    verdict = "met" if gap <= 0 else f"NOT met (gap {gap:.2f} m/s)"
    lines = [
        f"- flat top speed (measured, {res['gait']}, {res['leg_ctl']}): {v:.2f} m/s; held-out seeds n = {res['validate']['n']}: median "
        f"{res['validate']['median']:.2f}, min {res['validate']['min']:.2f} m/s",
        f"- spec target {V_TARGET} m/s: {verdict}",
        "- leg controller: " + ", ".join(f"{k} {x:.2f} m/s" for k, x in res["by_leg"].items()),
        f"- at the first failing speed: hip torque at its limit {100 * d['sat_hip']:.0f} % of stance ticks, knee {100 * d['sat_knee']:.0f} %; "
        f"joint speed peak {100 * d['w_frac']:.0f} % of no-load speed; {d['n_slip']} slips; pack peak {d['i_pack_peak']:.0f} A; "
        f"cost of transport {d['cot']:.2f}",
    ]
    return "\n".join(lines) + "\n"


def sweep_gait(sim, sg, report):
    """SP3b tools/dog_sweep.py --gait slot: 10 s on flat ground with the tuned fastest gait on THIS sweep variant (its own model from
    report["xml"]): stand-up 1.2 s, ramp at 1 m/s^2 toward V_TARGET + 0.5, -> TRUE mean forward speed over the last 3 s."""
    if "xml" not in report:
        raise ValueError("sweep_gait needs report['xml'] (the variant's MJCF path; tools/dog_sweep.py passes it since SP4 Task 10)")
    ...
```

  `measure` and `sweep_gait` drive `loco.controller.Controller` through the same loop as `dogwalk.run` (factor the per-tick body of
  `dogwalk.run` into `dogwalk.tick(ctx, k)` so both use one loop; the trot / stairs / rubble / push goldens must not move: a pure
  refactor) with `ctl.sched.set_params(gait, period=..., duty=..., apex=...)`, `ctl.h_frac`, and the gait forced
  (`choose` bypassed: `p["gait"] != "auto"` makes the harness call `ctl.sched.request(p["gait"])` each tick and the controller skip
  `choose` when `ctl.forced_gait` is set; add that attribute in `controller.py`). Diagnostics read `tau_cmd`, `tau_est`, `dq` of the
  Qdds and `current`, `voltage` of `/devices/power:pack_a|pack_b` from the Sim in the harness (truth-side, never in `loco`). CLI:
  `mp.Pool(args.jobs)` over (leg_ctl, gait, params) for `--search`, one ladder each; the headline (`gait`, `leg_ctl`, `v_top`) is the
  WBC's best, `by_leg` holds each controller's best (`leg_ctl` reaches `Controller` through `p["leg_ctl"]` of the shared loop).
  In `tools/dog_sweep.py`, where `run_point` calls the `--gait` function, pass `{**report, "xml": str(<the free-base variant's MJCF
  path that sim was loaded from>)}` instead of `report` (one call; nothing else in the tool changes).
- [ ] **Step 4: run the tests, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_topspeed.py tests/python/test_dog_sweep.py tests/python/test_dog_walk.py -n 8'`
- [ ] **Step 5: search and validate on hfbox (background, whole box):**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh bg --slots 2 dogtop '.venv/bin/python examples/dog/dogtop.py --gaits trot,flying_trot,bound --leg wbc,jt --search --validate 10 --jobs 16 --out build/dog-top'`;
  follow with `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh log dogtop 40`; `tools/remote.sh pull build/dog-top`, Read
  `remote-out/build/dog-top/topspeed.md` / `.json`. Write the chosen params of each gait into `GAITS` (Edit, comment = "dogtop
  --search <date>: v_top X m/s on seed 1001") and `v_max` of trot / flying_trot to the speed where the next gait became faster. Set
  `FLOOR` in the test to `min(2.0, floor(0.85 x the flying-trot v_top, 0.1))` and say so in the doc if it is below 2.0. Rerun Step 4;
  the four goldens are re-recorded if (only if) the tuned `GAITS` changed them, in this commit.
- [ ] **Step 6: the SP3b sweep with the gait slot:**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh bg --slots 2 dogsweep '.venv/bin/python tools/dog_sweep.py --gait dogtop:sweep_gait --out build/dog-sweep-sp4 --jobs 16'`,
  pull, Write `docs/dog/speed-sweep.md` / `.json` from it, update the speed line and sweep table of `assemblies/dog/README.md`. If the
  sweep's chosen point moves away from the built design, do NOT change the hardware here: report it to the orchestrator (an SP3
  decision for the user).
- [ ] **Step 7: write `docs/dog/locomotion.md` "Top speed"** (Edit): the `report_md` block, the per-gait table of the search (best
  params, v_top, CoT, for wbc and jt), the force-tracking RMS of both at their top speeds, the sweep's `v_top_trot` column, and, **if
  the target is not met**, the honest path: the measured number in the spec criteria table as NOT met, the limiting factor the
  diagnostics name (hip stance torque, no-load speed, slips, MPC or WBC force tracking), and the options with their evidence and cost,
  none adopted here: the sweep's other points (knee ratio 2.0, leg 0.25 m), the AK70-9 class (study: 2.8 m/s trot bound, +2.2 k USD),
  a higher bus voltage within the drivers' rating, WBC retuning (task gains / QP weights, only if its force tracking is what fails).
  If jt beats wbc at speed, say so with the numbers; the WBC stays the design's leg controller and the gap is a WBC finding for the
  orchestrator, not a silent switch.
- [ ] **Step 8: commit** `git add examples/dog/dogtop.py examples/dog/dogwalk.py examples/dog/loco/gait.py examples/dog/loco/controller.py tools/dog_sweep.py tests/python/test_dog_topspeed.py tests/python/test_dog_*.py docs/dog/locomotion.md docs/dog/speed-sweep.md docs/dog/speed-sweep.json assemblies/dog/README.md && git commit -m "dog loco: flying trot / bound tuned by a top-speed search, measured top speed vs 3.5 m/s with diagnostics, SP3b sweep gait column filled" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 11: terrain-family Monte Carlo, `docs/dog/locomotion.md`, targeted regression

**Files:**
- Create: `examples/dog/dogmc.py`
- Modify: `docs/dog/locomotion.md` (sections "Monte Carlo", "Spec criteria", "Known gaps"), `assemblies/dog/README.md` (one
  paragraph + link)
- Test: `tests/python/test_dog_mc.py`

**Interfaces:**
- Consumes: `dogwalk.run`, `dogcourse.scene` (payload), `dogtruth`, IMU `gyro_bias` / `accel_bias`.
- Produces: `dogmc`: `FAMILIES`, `TRAIN_BASE = 1000`, `TEST_BASE = 21000`, `draw(seed, fam) -> dict`, `one((seed, fam, over)) -> row`,
  `ok(row)`, `why(row)`, `wilson(k, n)`, `summarize(rows) -> dict`, `write_md(out, data)`, CLI `python examples/dog/dogmc.py --family
  all|<name> [--leg wbc|jt] [--train N] [--test N] [--jobs N] [--over JSON] [--out DIR]` writing `<out>/locomotion-mc.json` and `.md`
  (`--leg` sets `over["leg_ctl"]`, default `"wbc"`; every row records its `leg_ctl`).

- [ ] **Step 1: failing test** `tests/python/test_dog_mc.py`:

```python
"""SP4 Task 11: terrain-family Monte Carlo. The summary counts falls from ground truth, gives Wilson 95 % intervals and refuses runs with the lidar
on (Review Focus 5); per-family pass-rate floors on held-out seeds (small n: a regression catch, not the claim; docs/dog/locomotion.md has the MC)."""

import sys
from pathlib import Path

import napl
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "examples" / "dog"))
import dogmc as MC  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


def row(**kw):
    r = dict(seed=1, fam="rubble", reached=True, fallen=False, ctl_fallen=False, fault=False, diverged=False, mean_speed=1.6, dist=17.0,
             n_slip=0, mpc_fail=0, sat_hip=0.1, rangefinders=0)  # fmt: skip
    r.update(kw)
    return r


def test_summary_uses_truth_falls_wilson_and_refuses_lidar():  # Review Focus 5
    rows = [row(), row(), row(fallen=True, reached=False), row(reached=False), row(ctl_fallen=True)]
    s = MC.summarize(rows)
    assert s["n"] == 5 and s["k_nofall"] == 4  # the controller's own detector (last row) is not a fall; truth is
    assert s["k_success"] == 3 and s["failures"] == {"fall": 1, "timeout": 1}
    lo, hi = s["wilson_nofall"]
    assert lo < 4 / 5 < hi and s["wilson_success"][0] < 3 / 5
    assert s["speed_mean"] == pytest.approx(1.6)  # successful runs only
    with pytest.raises(ValueError, match="lidar"):
        MC.summarize([row(rangefinders=1)])
    with pytest.raises(ValueError):
        MC.summarize([])
    assert min(MC.TEST_BASE, *(MC.TEST_BASE + i for i in range(100))) > MC.TRAIN_BASE + 10000  # held-out seed blocks never overlap


# Floors on the first held-out seeds (21000...): (n flown, successes required). Below the MC rate of docs/dog/locomotion.md (set in Step 6):
# they catch a regression, they do not restate the rate.
FLOORS = {"rubble": (3, 2), "slopes": (3, 2), "scree": (3, 2), "ruins": (3, 2), "criterion": (3, 2)}


@needs_kernel
@pytest.mark.parametrize("fam", list(FLOORS))
def test_monte_carlo_floor_on_held_out_seeds(fam):
    n, need = FLOORS[fam]
    res = [MC.one((MC.TEST_BASE + i, fam, {})) for i in range(n)]
    assert all(r["rangefinders"] == 0 for r in res)
    k = sum(MC.ok(r) for r in res)
    assert k >= need, (fam, k, n, [MC.why(r) for r in res if not MC.ok(r)])
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_mc.py -n 8'`
- [ ] **Step 3: implement `examples/dog/dogmc.py`** (key code):

```python
"""Monte Carlo of the dog's locomotion per terrain family (SP4, spec s7: '>= 90 % no-fall per terrain family (MC n per family stated, with a
95 % lower bound)', 'rocky field mean speed >= 1.5 m/s', '25-degree slopes and 15 cm steps traversed').
Per run, from its seed: a new 30 x 30 m field of the family (terrain seed = seed; criterion: the fixed course), a truth route of 15-20 m, plant
dispersion the controller does not know (payload 0-1 kg at +-3 / +-2 cm on the trunk, IMU gyro bias N(0, 0.005 rad/s) and accel bias N(0, 0.05
m/s^2) per axis, two lateral pushes of 0.2-0.5 m/s at random times), pose-fix noise. Lidar and depth camera are OFF (removed from the scene;
summarize() refuses a row that had them). Verdicts from GROUND TRUTH (dogtruth.Truth.fallen), never from the controller's own fall detector.
Seeds: TRAIN 1000... (any tuning), TEST 21000... (held out, flown once with the final code)."""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse  # noqa: E402
import json  # noqa: E402
import multiprocessing as mp  # noqa: E402
import sys  # noqa: E402
import tempfile  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dogcourse  # noqa: E402
import dogwalk  # noqa: E402

FAMILIES = {  # family: (course, commanded speed m/s)
    "rubble": ("rubble", 1.8),  # spec s7 rocky mean >= 1.5: commanded above it, the follower slows in turns
    "slopes": ("slopes", 1.0),
    "scree": ("scree", 1.2),
    "ruins": ("ruins", 1.2),
    "criterion": ("criterion", 0.8),
}
TRAIN_BASE, TEST_BASE = 1000, 21000
SCRATCH = Path(os.environ.get("NAPL_MC_TMP", Path(tempfile.gettempdir()) / "napl-dog-mc"))


def wilson(k, n, z=1.96):  # = examples/hopper/mc.py wilson (copied: importing it pulls the hopper's `hop` / `mc` module names into a shared worker)
    if n == 0:
        return (0.0, 1.0)
    p, d = k / n, 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (float(c - h), float(c + h))


def draw(seed, fam):
    rng = np.random.default_rng([int(seed), 4242])
    return dict(payload=(float(rng.uniform(0, 1.0)), float(rng.uniform(-0.03, 0.03)), float(rng.uniform(-0.02, 0.02))),
                gyro_bias=rng.normal(0, 0.005, 3).tolist(), accel_bias=rng.normal(0, 0.05, 3).tolist(),
                pushes=[(float(t), (0.0, float(s * m), 0.0)) for t, s, m in zip(rng.uniform(4, 12, 2), rng.choice([-1, 1], 2), rng.uniform(0.2, 0.5, 2))])  # fmt: skip


def one(args):
    seed, fam, over = args
    course, v = FAMILIES[fam]
    h = draw(seed, fam)
    out = SCRATCH / f"w{os.getpid()}"
    model = dogcourse.scene(out, course, seed, payload=h["payload"])
    length = dogcourse.info(model)["route_length"]
    p = dict(dogwalk.DEFAULTS, seed=seed, course=course, v_cmd=over.get("v_cmd", v), duration=3.0 + 2.5 * length / v, pushes=h["pushes"],
             imu_bias=(h["gyro_bias"], h["accel_bias"]), leg_ctl=over.get("leg_ctl", "wbc"))  # fmt: skip
    r = dogwalk.run(model, p, log=False)["res"]
    return dict(seed=int(seed), fam=fam, leg_ctl=p["leg_ctl"], **r)


def ok(r):
    return r["reached"] and not (r["fallen"] or r["fault"] or r["diverged"])


def why(r):
    return "fall" if r["fallen"] else "fault" if r["fault"] else "diverged" if r["diverged"] else "timeout" if not r["reached"] else "ok"


def summarize(rows):
    if not rows:
        raise ValueError("no runs")
    if any(r["rangefinders"] for r in rows):
        raise ValueError("a run had the lidar / depth camera on: locomotion MC runs without rays (dogcourse.strip_lidar)")
    n, k_s, k_nf = len(rows), sum(map(ok, rows)), sum(not r["fallen"] for r in rows)
    fails = {}
    for r in rows:
        if not ok(r):
            fails[why(r)] = fails.get(why(r), 0) + 1
    sp = [r["mean_speed"] for r in rows if ok(r)]
    return dict(n=n, k_success=k_s, k_nofall=k_nf, wilson_success=wilson(k_s, n), wilson_nofall=wilson(k_nf, n), failures=fails,
                speed_mean=float(np.mean(sp)) if sp else 0.0, speed_median=float(np.median(sp)) if sp else 0.0,
                slips=int(sum(r["n_slip"] for r in rows)), mpc_fail=int(sum(r["mpc_fail"] for r in rows)))  # fmt: skip
```

  `dogwalk.run` gains `p["imu_bias"]` (sets `gyro_bias` / `accel_bias` of `/devices/compute:imu` before the first step, harness side)
  and `dogcourse.scene` writes `route_length` into course.json. `write_md(out, data)`: per family n, success k/n with Wilson 95 %,
  no-fall k/n with Wilson 95 % (the lower bound is the number compared with 90 %), mean / median speed of successful runs, failure
  causes, slips, MPC failures; a "How to read" block (truth verdicts, held-out seeds, lidar off, routes are ground-truth A*); the
  command line and the code revision (`git rev-parse --short HEAD` passed in by the CLI caller as `--rev`).
- [ ] **Step 4: run the tests, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_mc.py -n 8'`
- [ ] **Step 5: the MC on hfbox** (budget from "Compute budget": runs of 10-25 sim-s at the measured real-time factor on 16
  workers; N = 100 per family fits in about an hour at 0.5x real time; if the measured factor is lower, take the largest N that fits
  2 h and state it): train first if any tunable must move (`--train 30`, TRAIN seeds only, each change with its reason), then
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh bg --slots 2 dogmc ".venv/bin/python examples/dog/dogmc.py --family all --test 100 --jobs 16 --rev $(git rev-parse --short HEAD) --out build/dog-mc"`,
  `tools/remote.sh log dogmc 40` until finished, `tools/remote.sh pull build/dog-mc`, Read the `.md` / `.json`. Then the J^T baseline
  on the same held-out seeds with N = 30 per family (`--leg jt --test 30 --out build/dog-mc-jt`, same `bg` / `log` / `pull`): the
  doc puts both side by side; the spec verdicts use the WBC run.
- [ ] **Step 6: report.** Write `docs/dog/locomotion.md` (Edit): "Monte Carlo" (the WBC table as generated, then the jt baseline table), "Spec criteria" (each
  spec s7 locomotion line with the measured value and MET / NOT met: flat top speed (Task 10), rocky mean speed (rubble successful
  runs), 25 deg + 15 cm (criterion family), >= 90 % no-fall per family (the Wilson lower bound next to the rate)), "Known gaps" (no
  self-righting; SP5 stand-ins: truth height map, truth + noise pose fix, truth routes; inertia approximation; leg dynamics not in
  the contact estimate; linkage ratio constant (SP3b); MC on 30 m fields). Set `FLOORS` in the test to (3, 2) for families whose MC
  success is >= 75 %, (3, 1) for 40-75 %, and drop a family whose MC success is below 40 % from `FLOORS` with a comment pointing to
  the doc (a floor cannot pass on a failing family; the doc is the claim). Add one paragraph + link to `assemblies/dog/README.md`.
- [ ] **Step 7: targeted regression (USER RULE: no full suite), expect all green:**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-phys tools/remote.sh run 'PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_*.py tests/python/test_qdd.py tests/python/test_foot_contact.py tests/python/test_terrain_field.py -n 8 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
  (add `tests/python/test_program.py` and `ctest --preset dev -R "qdd|foot_contact"` after a fresh `cmake --build --preset dev` if
  Task 1 Step 6 changed bindings).
- [ ] **Step 8: commit** `git add examples/dog/dogmc.py examples/dog/dogwalk.py examples/dog/dogcourse.py tests/python/test_dog_mc.py docs/dog/locomotion.md assemblies/dog/README.md && git commit -m "dog loco: terrain-family Monte Carlo (truth verdicts, Wilson bounds, lidar off), locomotion report vs the spec criteria" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`
