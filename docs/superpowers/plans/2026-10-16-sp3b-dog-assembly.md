# SP3b: dog assembly (leg / head / power / compute sub-assemblies, chassis, wiring, BOM) + speed sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the ~12 kg recon dog exists as linked napl sub-assemblies (`assemblies/dog/{leg,head,power,compute,dog}.toml`) built
from real priced parts, pin-level wired and DRC clean, with a BOM against the 3-4.5 k USD target; it stands and squats in
simulation on its 12 Qdd actuators under a plain joint PD (golden state hash); and a sweep harness measures, on the full model,
everything about speed that does not need a gait controller, for the three study candidates x leg lengths, written to
`docs/dog/speed-sweep.md`, with a slot SP4 fills by rerunning it with its controller.

**Architecture:** no new kernel or platform code. SP1 sub-assemblies (flatten on load, `[exports]`, `[[subassemblies]]`
mirror / params, `report["bom"]`, `report["mirror_signs"]`) carry the structure; SP2 devices (`qdd`, `foot_contact`, rosette
`rangefinder`) carry the physics; SP3a parts plus the parts and CAD fixes of Tasks 1-3 carry the numbers. The leg is written once
(left hand), mirrored `"xz"` for the right side, and the rear pair is the front pair turned 180 deg about z by the rear bulkhead
(no `"yz"` mirror needed). Simulation helpers live in `examples/dog/dogsim.py` (bench code SP4 reuses); the sweep is
`tools/dog_sweep.py` (variant files in a temp dir, one built model per point, stdlib multiprocessing).

**Tech Stack:** TOML assemblies and parts, Python 3.12 (napl.design, napl.Sim, numpy), build123d CAD (`cad/dog`, cad-venv on
hfbox only), pytest (+xdist), hfbox via `tools/remote.sh`.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` sections 5 (SP3), 7 (targets), 9 (risk 3); inputs
`docs/dog/actuator-study.md`, `docs/superpowers/plans/2026-10-16-sp1-subassemblies-core.md`,
`docs/superpowers/plans/2026-10-16-sp2-ground-kernel.md`, `docs/guide.md` "Sub-assemblies", `parts/README.md` "Mirroring
([mirror]) and price".

## Global Constraints

- **Prerequisite:** branch `p12-ground` (SP2) is merged into `napl-v1` before Task 1 starts (Task 1 Step 1 checks it). Work in
  worktree `../napl-wt/dog`, branch `p12-dog` from `napl-v1`, remote dir `/workspace/wt/napl-dog` (the orchestrator may assign
  another; then replace it in every command).
- **GROUND names this plan depends on**, read from the in-progress branch `p12-ground` at `704f001` (`../napl-wt/ground`):
  `assembly.DEVICE_TYPES` gains `"qdd"`, `"foot_contact"`; `assembly.BARE_OK` gains `timeout_mode`, `mode`, `shape`; schema
  `Qdd {node, joint, motor, gear_ratio, efficiency, encoder_cpr, cmd_hz, latency_s, i_limit, kp_max, kd_max, tau_ff_max, timeout_s,
  timeout_mode, kd_timeout, t_max, derate_start}`; `FootContact {geom, mode, f_max, noise_std, hysteresis_on, hysteresis_off,
  debounce_s, rate_hz, seed}` with binding key `shape` (geometry index of the foot part); Qdd settable attrs `p_des v_des kp kd
  tau_ff enable`, readable `q dq tau_est tau_cmd i_q temp fault bus_i voltage derate timeout`; FootContact attrs `force contact`;
  `auto_devices.QDD_MOTOR` / `auto_devices.QDD` (device field -> accepted part param names: `kv`, `r_phase_to_phase`,
  `rotor_inertia`, `c_th_winding`, `r_th_winding`, `gear_ratio`, `efficiency`, `encoder_cpr`, `can_cmd_hz`, `can_latency`,
  `i_limit_phase`, `mit_kp_max`, `mit_kd_max`, `mit_tau_ff_max`, `can_timeout`, `kd_timeout`, `temp_fault`,
  `temp_derate_start`); `devices.FIELD_UNITS` for those fields; Rangefinder `projection = "rosette"` with `fov_h, fov_v_min,
  fov_v_max, rays_per_frame, pattern_seed`. Re-check each against the merged code in Task 1 Step 1; a renamed field is fixed in this
  plan's files, never by editing SP2 code.
- Numbers rule: every numeric leaf is `"<inst>.<param>"` or a provenance table `{value, unit, fidelity, src | derivation |
  rationale}` (`assembly.py` docstring; bare only for `BARE_OK` keys). No physics defaults: every Qdd / FootContact field comes from
  a part param (device map) or a provenance-tagged `set`.
- All compute on hfbox, laptop = edits + git only:
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '<cmd>'` (add `--slots 2` for the sweep). First build of the
  worktree and after any merge: `cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH
  UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh`. Tests: `.venv/bin/pytest <files> -n 12`.
  `.venv/bin/python tools/check_parts.py` and `.venv/bin/python tools/check_licenses.py`: 0 problems at every commit that touches
  parts. CAD: `cad-venv/bin/python cad/dog/build.py ...` on hfbox only.
- Every existing test and golden state hash stays green (full suite before the last commit). No new dependencies; no GPL. No
  media committed (no screenshots / video; generated STL meshes under `meshes/dog/` are CAD outputs as in SP3a, not media).
- Files are edited only with the Edit / Write tools (no sed / heredoc / shell redirection into the tree). Generated text outputs
  that come back from hfbox (`tools/remote.sh pull`) are read with Read and written into the tree with Write; binary STL outputs
  are copied with `cp remote-out/... meshes/dog/` (if the edit guard refuses `cp`, stop and ask the orchestrator).
- Every commit message ends with `Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5`.
- Recon robot only: no weapon, launcher or payload-release part, device or wiring anywhere in the dog.
- **Decisions this plan takes** (from the merged SP3a / SP1 / SP2 code; the hand-back lists them for confirmation):
  1. Knee drive = crank + push-rod linkage from a knee motor coaxial with the hip pitch axis, not a belt: the knee actuator's 22 N m
     on a 20 T GT2 pulley (pitch radius 6.37 mm) is 3.45 kN of belt tension, an order above a 6 mm GT2 belt; the 30T / 40T pulley gap
     of the study is therefore closed by not using pulleys. The SP3a knee-drive housing (GIM6010 pocket at the knee) is removed.
  2. Battery margin: two 8S1P packs, each with its own 60 A Daly BMS, in parallel (140 A pack / 120 A BMS continuous vs the study's
     73-87 A peak). Uses parts that exist; no 100 A BMS part needed.
  3. E-stop: the 10 A NC e-stop switches the 12 V coil of a new 100 A DC contactor in the motor feed; the compute and lidar
     DC-DCs are fed before the contactor (the e-stop removes joint power only, the recon payload keeps logging). Inrush precharge is
     an open item in the README (not modelled), not a part.
  4. SP3a printed parts only had `frame.origin` placeholder ports and an abduction housing that also carried the pitch seat (no
     abduction DOF). Task 2 reworks the leg chain: abad mount (chassis) -> abad hinge -> hip housing (pitch + knee motors) -> thigh
     -> shank; hips bolt to front / rear bulkheads (abduction axis along x), not to the side plates.
  5. Exports cannot move ports or swap parts (SP1), so leg length and actuator are not exported params: length variants are real
     printed parts (`-200`, `-250`), the sweep writes variant files. `knee_drive_ratio` (device gear ratio) and `lidar_rays`
     (ray budget) are the overridable exports.
  6. FootContact is a virtual sensor (stand-in for the SP4 torque-based contact estimate); there is no foot sensor hardware in
     the BOM.
  7. RTK receiver on the Jetson UART and the IMU on its I2C header, so the 4 USB-A ports take the 4 CANables (one CAN bus per leg,
     3 Qdds each: ~40 % bus load at 1 Mbit/s, 500 Hz) and the D435i takes the USB-C port.
  8. Front knees point backward, rear knees forward (rear legs are the front legs turned 180 deg).

## Review Focus

1. Knee linkage strength: the push rod must not buckle and the rod ends must carry the peak rod force at the actuator's stall torque. Pinned by `tests/python/test_dog_parts.py::test_knee_rod_does_not_buckle_at_peak_knee_torque` (Task 3).
2. Qdd physics only from part files, knee through the linkage: every one of the 12 joints of the built dog stalls at the datasheet torque (x the knee linkage ratio at the knee) from the bus it is wired to. Pinned by `tests/python/test_dog_assembly.py::test_every_joint_stalls_at_its_datasheet_torque` (Task 7).
3. Mirror / rotation correctness: a wrong-hand part, a wrong mirror sign or a rotated rear leg commanded with the front's sign shows as an asymmetric stance. Pinned by `tests/python/test_dog_stand.py::test_stance_is_symmetric` (Task 8).
4. Power safety: the e-stop switches only the contactor coil (never motor current) and two packs + two BMS + the contactor + the PDB input are rated for the study's worst simultaneous peak. Pinned by `tests/python/test_dog_power.py::test_estop_switches_only_the_coil_and_ratings_cover_the_peak` (Task 5).
5. Sweep honesty: every column of the sweep is labelled measured / derived / bom / sp4, nothing measured is called a top speed, and the run is deterministic. Pinned by `tests/python/test_dog_sweep.py::test_report_labels_every_column_and_is_deterministic` (Task 9).

---

### Task 1: prerequisites — GROUND merged, sub-assemblies know the new devices, Qdd params on the SteadyWin parts

**Files:**
- Modify: `python/napl/design/subasm.py` (`_TYPES`), `parts/actuators/steadywin-gim8108-8.toml`,
  `parts/actuators/steadywin-gim6010-8.toml`
- Test: `tests/python/test_dog_parts.py`

**Interfaces:**
- Consumes: `assembly.DEVICE_TYPES`, `auto_devices.QDD_MOTOR`, `auto_devices.QDD` (GROUND, see Global Constraints).
- Produces: `subasm.REF_FIELDS["qdd"]`, `subasm.REF_FIELDS["foot_contact"]` (= `_COMMON`: `instance`, `joint`, `map`, ...);
  part params on both SteadyWin parts: `kv` (rpm/V, motor side), `c_th_winding` (J/K), `r_th_winding` (K/W), `encoder_cpr` (1),
  `i_limit_phase` (A), `can_cmd_hz` (Hz), `can_latency` (s), `mit_kp_max` (N*m/rad), `mit_kd_max` (N*m*s/rad), `mit_tau_ff_max`
  (N*m), `can_timeout` (s), `kd_timeout` (N*m*s/rad), `temp_fault` (degC), `temp_derate_start` (degC).

- [ ] **Step 1: check the prerequisite.** Run `git log --oneline napl-v1 | grep -m1 "Merge p12-ground"` and
  `grep -n '"qdd"\|"foot_contact"' python/napl/design/assembly.py`. No merge commit or no match: stop and report to the
  orchestrator. Then re-read the GROUND names of Global Constraints in the merged `schema/devices.fbs`,
  `python/napl/design/devices.py`, `python/napl/design/auto_devices.py` and note any difference in the task report.

- [ ] **Step 2: failing tests** (append to `tests/python/test_dog_parts.py`):

```python
from napl.design import assembly as A
from napl.design import auto_devices as AD
from napl.design import subasm

STEADYWIN = QDD[:2]  # steadywin-gim8108-8, steadywin-gim6010-8


def test_subasm_prefixes_the_ground_devices():
    assert {"qdd", "foot_contact"} <= A.DEVICE_TYPES
    assert A.DEVICE_TYPES <= set(subasm.REF_FIELDS)  # a device type without reference fields cannot sit in a leg.toml


@pytest.mark.parametrize("rel", STEADYWIN)
def test_steadywin_part_feeds_every_qdd_field(rel):
    p = load(rel)
    for field, names in {**AD.QDD_MOTOR, **AD.QDD}.items():
        if field == "motor.i0":  # optional in the kernel (docs/models.md "Qdd")
            continue
        assert any(n in p["params"] for n in names), (rel, field, names)
    kt = 60 / (2 * math.pi * v(p, "kv"))  # motor-side Kt from KV: the Qdd's own law
    stall = kt * v(p, "gear_ratio") * v(p, "efficiency") * v(p, "i_limit_phase")
    assert stall == pytest.approx(v(p, "torque_peak"), rel=0.05)
    assert v(p, "mit_tau_ff_max") >= v(p, "torque_peak")
    assert 25.0 < v(p, "temp_derate_start") < v(p, "temp_fault")
    assert v(p, "can_latency") <= 14 / v(p, "can_cmd_hz")  # kernel bind-time rule
```

- [ ] **Step 3: run, expect FAIL** (`REF_FIELDS` lacks `qdd`; parts lack `kv` ...):
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && .venv/bin/pytest tests/python/test_dog_parts.py -n 12'`

- [ ] **Step 4: implement.** In `subasm.py` add `"qdd"` and `"foot_contact"` to `_TYPES` (and any other entry of
  `A.DEVICE_TYPES` still missing, e.g. `"rain"` if the merge added it there); their reference fields are covered by `_COMMON`
  (`instance`, `joint`, `map`); `shape` / `mode` / `timeout_mode` are numbers (BARE_OK), not references. Add to the `[params]` of
  `steadywin-gim8108-8.toml` (same keys with the GIM6010-8's own numbers and derivations in `steadywin-gim6010-8.toml`):

```toml
# --- Qdd device params (SP2 kernel, docs/models.md "Qdd"; MIT-mode command limits are the convention of the open MIT-mode firmware
#     family; the GDS68 driver's MIT support is NOT stated by the vendor (can_protocol above): verify before a real build) ---
kv = { value = 53.36, unit = "rpm/V", fidelity = "derived", derivation = "Motor-side KV = kv_output 6.67 rpm/V x gear_ratio 8 = 53.36 rpm/V; Kt = 60 / (2 pi KV) = 0.1790 N*m/A = kt_motor." }
i_limit_phase = { value = 22.0, unit = "A", fidelity = "derived", derivation = "= current_peak 22 A. efficiency 0.699 was derived from the same current, so Kt N eta i_limit = 0.179 x 8 x 0.699 x 22 = 22.0 N*m = torque_peak (the pair reproduces the published stall torque)." }
encoder_cpr = { value = 65536, unit = "1", src = "S1", fidelity = "datasheet" }  # "Resolution of Encoder on Driver | 16 bit"
c_th_winding = { value = 60.0, unit = "J/K", fidelity = "estimated", rationale = "Not published. Copper + stator teeth of a 378 g 21-pole-pair outrunner, ~150 g at ~0.4 J/(g K) = 60 J/K (lumped winding node; +-50 %)." }
r_th_winding = { value = 1.35, unit = "K/W", fidelity = "derived", derivation = "current_rated 7 A must settle at temp_derate_start 80 degC from 25 degC ambient under the Qdd thermal law (docs/models.md 'Qdd': C dT/dt = i_q^2 R(T) - (T - T_amb) / R_th, R(T) = r20 (1 + 0.00393 (T - 20))): R(80) = 0.67 x 1.236 = 0.828 ohm, P = 49 x 0.828 = 40.6 W, R_th = 55 K / 40.6 W = 1.35 K/W." }
can_cmd_hz = { value = 1000.0, unit = "Hz", fidelity = "estimated", rationale = "Driver command processing rate; MIT-mode drivers of this class process CAN frames at 1 kHz or more. The dog sends 500 Hz (spec s6)." }
can_latency = { value = 0.001, unit = "s", fidelity = "estimated", rationale = "One CAN frame at 1 Mbit/s (~0.13 ms) + driver loop; 1 ms (one physics tick) taken." }
mit_kp_max = { value = 500.0, unit = "N*m/rad", fidelity = "estimated", rationale = "MIT-mode protocol range convention (KP 0..500)." }
mit_kd_max = { value = 5.0, unit = "N*m*s/rad", fidelity = "estimated", rationale = "MIT-mode protocol range convention (KD 0..5)." }
mit_tau_ff_max = { value = 22.0, unit = "N*m", fidelity = "derived", derivation = "= torque_peak (output side): a larger feed-forward is saturated by i_limit_phase anyway." }
can_timeout = { value = 0.1, unit = "s", fidelity = "estimated", rationale = "Command-loss watchdog of the driver; 0.1 s = 50 missed 500 Hz frames." }
kd_timeout = { value = 1.0, unit = "N*m*s/rad", fidelity = "estimated", rationale = "Damping-mode gain on command loss (timeout_mode 0): soft enough not to slam a falling leg." }
temp_fault = { value = 100.0, unit = "degC", fidelity = "estimated", rationale = "GDS68 over-temperature threshold not retrieved; class-H-insulated small BLDC drivers fault near 100 degC." }
temp_derate_start = { value = 80.0, unit = "degC", fidelity = "estimated", rationale = "20 K below temp_fault (linear current derate, docs/models.md 'Qdd')." }
```

- [ ] **Step 5: run the tests and the checks, expect PASS / 0 problems:**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_parts.py tests/python/test_subasm.py -n 12 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
- [ ] **Step 6: commit** `git add python/napl/design/subasm.py parts/actuators/steadywin-gim8108-8.toml parts/actuators/steadywin-gim6010-8.toml tests/python/test_dog_parts.py && git commit -m "subasm+parts: Qdd/FootContact in sub-assemblies, Qdd params on the SteadyWin actuators" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 2: CAD rework — leg chain with abduction, knee linkage cranks, bulkhead, compute tray, length variants

**Files:**
- Modify: `cad/dog/params.py`, `cad/dog/parts.py`, `cad/dog/build.py` (`--only`, `--suffix`), part TOMLs
  `parts/structure/dog-hip-housing-{l,r}.toml`, `dog-thigh-bracket-{l,r}.toml`, `dog-shank.toml`, `dog-foot-core.toml`,
  `dog-head-mount.toml`, `dog-side-plate-{l,r}.toml` (ports only), `tests/python/test_dog_parts.py` (`HANDED` list)
- Create: `parts/structure/dog-abad-mount.toml`, `dog-knee-crank.toml`, `dog-bulkhead.toml`, `dog-compute-tray.toml`,
  `dog-thigh-bracket-{l,r}-{200,250}.toml`, `dog-shank-{200,250}.toml`, meshes `meshes/dog/<id>.stl` for every new / changed part
- Delete: `parts/structure/dog-knee-drive-housing-{l,r}.toml`, `meshes/dog/dog-knee-drive-housing-{l,r}.stl`
- Test: `tests/python/test_dog_cad.py`

**Interfaces:**
- Consumes: actuator port types `bolt-pattern.steadywin-gim8108-base` (`mount`), `flange.steadywin-gim8108-output` (`output`);
  `sphere-seat.35` (ball `seat`); `bolt-pattern.livox-mid360-base.m3` (Mid-360 `base`); `flat-mount` (D435i `mount`, Jetson
  `base`, RTK `base`, ANN-MB `base`); `board.20x13-header` (MinIMU `base`).
- Produces (part ids and ports used by Tasks 4-7):
  - `dog-abad-mount` (`[mirror] symmetry = "xz"`): `chassis` (frame.origin, on the abduction axis in the bulkhead plane, axis
    toward the body), `motor` (`bolt-pattern.steadywin-gim8108-base`, same point, axis away from the body), `hub` (flat-mount).
  - `dog-hip-housing-l/-r` (opposite pair): `abad` (`flange.steadywin-gim8108-output`), `pitch_seat` and `knee_seat`
    (`bolt-pattern.steadywin-gim8108-base`, coaxial on the hip pitch axis, hip motor output outboard, knee motor output inboard).
  - `dog-thigh-bracket-{l,r}[-200|-250]`: `origin` (`flange.steadywin-gim8108-output`, hip pitch axis), `knee` (frame.origin,
    z = -L), `rod` (frame.origin, mid-thigh, inboard); params `length`, `knee_range`.
  - `dog-shank[-200|-250]` (`symmetry = "xz"`, lug on both faces so it stays its own mirror): `origin` (frame.origin, knee
    axis), `foot` (frame.origin, z = -L); params `length`, `lug_radius`.
  - `dog-knee-crank` (`symmetry = "xz"`): `motor` (`flange.steadywin-gim8108-output`); param `crank_radius`.
  - `dog-foot-core`: new port `ball` (`sphere-seat.35`) so the ball centre sits on the core origin.
  - `dog-bulkhead` (`symmetry = "xz"`): `hip_l`, `hip_r` (frame.origin), `deck_top`, `deck_bottom` (frame.origin).
  - `dog-head-mount`: `lidar` retyped `bolt-pattern.livox-mid360-base.m3`, new `camera` (flat-mount) on the nose shelf.
  - `dog-compute-tray` (`symmetry = "xz"`): `origin`, `jetson`, `rtk`, `antenna` (flat-mount), `imu` (`board.20x13-header`),
    `can_fl`, `can_fr`, `can_rl`, `can_rr` (flat-mount).
  - `cad/dog/params.py` constants `KNEE_R1 = 20.0`, `KNEE_RATIO = 1.5`, `KNEE_R2 = KNEE_R1 * KNEE_RATIO`, `HIP_Y`, `PITCH_Y`,
    `PITCH_X`, `BULK_W`, `BULK_H`.

- [ ] **Step 1: failing test** `tests/python/test_dog_cad.py`:

```python
"""Dog printed parts after the SP3b leg rework (docs/superpowers/plans/2026-10-16-sp3b-dog-assembly.md Task 2): every printed seat
takes the port type of the bought part it holds, handed pairs reference each other, length variants put the knee / foot at L."""

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
    ("dog-knee-crank", "motor", "steadywin-gim8108-8", "output"),
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
    r1 = LIB.get("dog-knee-crank").value("crank_radius")
    for sfx in ("", "-200", "-250"):
        assert LIB.get(f"dog-shank{sfx}").value("lug_radius") / r1 == pytest.approx(1.5)
```

- [ ] **Step 2: run, expect FAIL** (parts missing): `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_cad.py -n 12'`
- [ ] **Step 3: CAD.** In `cad/dog/params.py` add (mm; `[est]` = design estimate, final values are what makes the Task 7 DRC
  `INTERFERENCE` clean, recorded here):

```python
KNEE_R1 = 20.0  # knee motor crank radius [est]: rod force = 22 N m / 0.020 m = 1.1 kN (Task 3 sizes the rod for it)
KNEE_RATIO = float(os.environ.get("DOG_KNEE_RATIO", 1.5))  # study candidate 1 (docs/dog/actuator-study.md)
KNEE_R2 = KNEE_R1 * KNEE_RATIO  # shank lug radius
HIP_Y = 60.0  # |y| of the abduction axes on a bulkhead [est]
PITCH_X, PITCH_Y = 45.0, 80.0  # hip pitch axis from the abad flange: forward along the abad axis, outboard [est]
GAP_MOTORS = 5.0  # between the back-to-back hip and knee actuators on the pitch axis [est]
BULK_W, BULK_H, BULK_T = 214.0, 120.0, 6.0  # bulkhead (fits the 220 x 220 bed) [est]
HIP_BOLT_Y, HIP_BOLT_Z = 40.0, 45.0  # abad mount <-> bulkhead: 4 x M3 (the SP3a hip housing's 80 x 90 pattern)
```

  In `cad/dog/parts.py` replace `hip_housing` and `knee_drive_housing`, extend `shank`, add the new functions, and update
  `REGISTRY` (`"dog-knee-drive-housing"` removed; `"dog-abad-mount": (abad_mount, 4, "pla", False)`, `"dog-knee-crank":
  (knee_crank, 4, "pla", False)`, `"dog-bulkhead": (bulkhead, 2, "pla", False)`, `"dog-compute-tray": (compute_tray, 1, "pla",
  False)`):

```python
def abad_mount():
    """Abduction actuator cup: origin on the abad axis in the bulkhead plane; the actuator body sits INSIDE the body (x < 0), its
    output face flush with the flange at x = 0. Symmetric about XZ."""
    s = bx(6, 2 * HIP_FLANGE, 2 * HIP_FLANGE + 10, x=3)  # flange on the bulkhead's outer face
    s += cyl_x(P.AB_D + 8, 58, x=-29)  # cup through the bulkhead window
    s -= cyl_x(P.AB_D, 56, x=-28)  # actuator bore (92 x 55 body)
    s -= cyl_x(60, 10, x=3)  # output opening
    for yy in (-P.HIP_BOLT_Y, P.HIP_BOLT_Y):
        for zz in (-P.HIP_BOLT_Z, P.HIP_BOLT_Z):
            s -= cyl_x(P.M3_CLR, 8, x=3, y=yy, z=zz)
    s += bx(4, 40, 30, x=-60, z=-P.AB_D / 2 - 15)  # leg hub PCB pad under the cup
    return rounded(s, 2.0)


HIP_FLANGE = 52.0  # half-size of the abad mount flange (covers the 80 x 90 bolt pattern) [est]


def hip_housing():
    """Abduction link: origin on the abad axis at the actuator output flange (x = 0); an arm reaches forward (+x, PITCH_X) and
    outboard (+y) to a sleeve on the hip pitch axis that holds the hip actuator (output outboard) and the knee actuator (output
    inboard) back to back. Left hand (+y outboard); the right is its XZ mirror."""
    s = cyl_x(60, 8, x=4)  # flange on the abad output (6 x M3 on a 50 mm circle)
    for k in range(6):
        a = math.radians(60 * k)
        s -= cyl_x(P.M3_CLR, 10, x=4, y=25 * math.cos(a), z=25 * math.sin(a))
    sleeve_len = 2 * 55 + P.GAP_MOTORS
    y0 = P.PITCH_Y - sleeve_len / 2  # sleeve centre on the pitch axis
    s += bx(P.PITCH_X, 30, 30, x=P.PITCH_X / 2 + 4, y=y0 / 2)  # arm
    s += cyl_y(P.AB_D + 8, sleeve_len, x=P.PITCH_X, y=y0)
    s -= cyl_y(P.AB_D, sleeve_len + 2, x=P.PITCH_X, y=y0)
    return rounded(s, 2.0)


def knee_crank():
    """Crank on the knee actuator output: origin on the axis at the flange face, arm along +x to the rod-end pin at KNEE_R1. Symmetric."""
    s = cyl_y(40, 8)
    s += bx(P.KNEE_R1, 8, 14, x=P.KNEE_R1 / 2)
    s += cyl_y(14, 8, x=P.KNEE_R1)
    s -= cyl_y(5.3, 10, x=P.KNEE_R1)  # M5 rod-end pin
    for k in range(6):
        a = math.radians(60 * k)
        s -= cyl_y(P.M3_CLR, 10, x=12 * math.cos(a), z=12 * math.sin(a))
    return s


def bulkhead():
    """Front / rear bulkhead in the YZ plane: origin at its centre; two windows for the abad cups at y = +-HIP_Y with the 4 x M3
    pattern; flanges for the decks at z = +-BULK_H / 2. Symmetric about XZ (one part, front and rear)."""
    s = bx(P.BULK_T, P.BULK_W, P.BULK_H)
    for sy in (-1, 1):
        s -= cyl_x(P.AB_D + 9, P.BULK_T + 2, y=sy * P.HIP_Y)
        for yy in (-P.HIP_BOLT_Y, P.HIP_BOLT_Y):
            for zz in (-P.HIP_BOLT_Z, P.HIP_BOLT_Z):
                s -= cyl_x(P.M3_CLR, P.BULK_T + 2, y=sy * P.HIP_Y + yy, z=zz)
    for sz in (-1, 1):
        s += bx(14, P.BULK_W, 4, x=-7, z=sz * (P.BULK_H / 2 - 2))  # deck flange toward the inside (-x)
    return rounded(s, 2.0)


def compute_tray():
    """Electronics tray on the top deck: Jetson devkit (100 x 79 mm), 4 CANables, MinIMU on the centre line, RTK board, antenna
    pad at the rear. Origin at the floor centre, +x forward. Symmetric."""
    s = bx(200, 120, 3, z=1.5)
    for sy in (-1, 1):
        s += bx(200, 3, 12, y=sy * 58.5, z=6)  # side walls
    return s
```

  `shank()` gains the lug on both faces (keeps the XZ symmetry):
  `for sy in (-1, 1): s += bx(P.KNEE_R2, 6, 12, x=-P.KNEE_R2 / 2, y=sy * 10); s -= cyl_y(5.3, 30, x=-P.KNEE_R2)`.
  `build.py` gains `--only NAME ...` (REGISTRY keys) and `--suffix S` (appended to every variant id, e.g. `-200`).
- [ ] **Step 4: build on hfbox** (default L = 220, then the two length variants of thigh + shank):
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run 'cad-venv/bin/python cad/dog/build.py --out build/cad/dog && DOG_L_THIGH=200 DOG_L_SHANK=200 cad-venv/bin/python cad/dog/build.py --out build/cad/dog-200 --only dog-thigh-bracket dog-shank --suffix -200 && DOG_L_THIGH=250 DOG_L_SHANK=250 cad-venv/bin/python cad/dog/build.py --out build/cad/dog-250 --only dog-thigh-bracket dog-shank --suffix -250'`
  then `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh pull build/cad/dog build/cad/dog-200 build/cad/dog-250`, copy the
  `napl/*.stl` into `meshes/dog/`, and read each `manifest.json` (Read) for volume, mass, bbox.
- [ ] **Step 5: part TOMLs.** For every new / changed printed part write the TOML as the SP3a ones (header comment naming the CAD
  function, `[mirror]`, print params, `mass` derived from the manifest with the shell + infill derivation, `[geometry]` mesh +
  bbox from the manifest, ports at the CAD coordinates in metres). Port types per the Produces list; printed-to-printed joints keep
  `frame.origin` (neutral). Thigh `knee_range` param: `{ value = [lo, hi], unit = "rad", fidelity = "derived", derivation = ... }`
  with the knee angles where the linkage transmission angle (rod vs. shank lug) leaves [30, 150] deg, computed from KNEE_R1,
  KNEE_R2, L and the crank / lug rest angles, and q = 0 (straight leg, the build pose) inside the range (if the linkage cannot
  reach straight, rotate the lug's rest angle in the CAD until it can). Delete the knee-drive-housing pair. In
  `tests/python/test_dog_parts.py` set `HANDED = ["hip-housing", "thigh-bracket", "side-plate"]` and add `"abad-mount"`,
  `"knee-crank"`, `"bulkhead"`, `"compute-tray"` to the singles list. Side plates: add a `top` port (frame.origin, on the
  inboard flange) only; their hip windows stay as lightening.
- [ ] **Step 6: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_cad.py tests/python/test_dog_parts.py -n 12 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
- [ ] **Step 7: commit** `git add cad/dog parts/structure meshes/dog tests/python/test_dog_cad.py tests/python/test_dog_parts.py && git commit -m "cad+parts: dog leg chain with abduction, knee crank linkage, bulkhead, compute tray, 200/250 mm leg variants" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 3: bought and custom parts — knee rods + rod ends, contactor, leg hub, decks, Li-ion OCV, missing ports and prices

**Files:**
- Create: `parts/structure/generic-phs5-rod-end-m5.toml`, `parts/structure/dog-knee-rod-{200,220,250}.toml`,
  `parts/electronics/generic-zjw100a-dc-contactor-12v.toml`, `parts/electronics/dog-leg-hub.toml`,
  `parts/structure/dog-deck-top-cf3.toml`, `parts/structure/dog-deck-bottom-cf3.toml`,
  `parts/common/liion-cell-ocv-chen2020-lgm50.toml`
- Modify: `parts/electronics/dog-power-distribution-board.toml` (BAT input XT90, `i_max` 90 A, `base` port),
  `parts/propulsion/auline-21700-s70-8s1p-4500mah.toml` (`base` port), `parts/electronics/daly-k-series-smart-bms-8s-60a.toml`
  (`base` port), `parts/electronics/makerbase-mks-canable-v2-pro.toml` (`base` flat-mount port),
  `parts/electronics/nvidia-jetson-orin-nano-devkit-8gb.toml` (`usb_j5` port on connector J5, `uart1` port on J12 pins 8 / 10 /
  6, `i2c1` port on J12 pins 3 / 5 / 1 / 6), `parts/electronics/sparkfun-gps-rtk-sma-16481.toml` (`uart` port on its UART1
  header pins), `parts/electronics/pololu-minimu-9-v6.toml` and `parts/electronics/ublox-ann-mb-00.toml` (`price` if missing)
- Test: `tests/python/test_dog_parts.py`

**Interfaces:**
- Consumes: `dog-knee-crank.crank_radius` (Task 2), `steadywin-gim8108-8.torque_peak`, `cf-tube-10x8` params.
- Produces:
  - `generic-phs5-rod-end-m5`: ports `thread` (`thread.m5`, male), `pin` (frame.origin); params `mass`, `price`,
    `load_static_radial` (N).
  - `dog-knee-rod-{200,220,250}` (cf-tube-10x8 cut to length + 2 bonded M5 inserts; custom): ports `mount` (frame.origin, the
    rod centre), `end_a`, `end_b` (`thread.m5`, female); params `pin_length` (m, rod-end pin centres = L), `outer_diameter`,
    `inner_diameter`, `youngs_modulus_axial`, `buckling_load` (derived), `mass`, `price` (derived).
  - `generic-zjw100a-dc-contactor-12v`: ports `base` (flat-mount), `line`, `load` (power, screw studs, `joinable = true`),
    `coil` (power, 2 pins); params `i_rated_continuous` (A), `v_rated_contact` (V), `coil_voltage` (V), `coil_current` (A),
    `r_contact` (ohm), `mass`, `price`.
  - `dog-leg-hub` (custom 2-layer PCB, labelled CUSTOM): ports `base` (flat-mount), `power_in` (XT60 male, pins `PIN.1` V+,
    `PIN.2` GND), `can_in` (JST-GH 4P, pins `CANIN.1` CAN_H, `CANIN.2` CAN_L, `CANIN.3` GND), for j in abad / hip / knee:
    `out_<j>` (power, XT30(2+2)-M pins `O<J>.1` V+, `O<J>.2` GND) and `can_<j>` (signal, pins `O<J>.3` CAN_H, `O<J>.4` CAN_L)
    with `<J>` = `A`, `H`, `K`; params `r_path` (ohm, per output), `can_termination` (ohm, 120), `mass`, `price`.
  - `dog-deck-top-cf3` (3 mm CF sheet cut, custom): ports `bulk_f`, `bulk_r` (frame.origin; `bulk_r`'s frame is `bulk_f`'s
    turned 180 deg about z), `side_l`, `side_r`, `head`, `compute` (frame.origin).
  - `dog-deck-bottom-cf3`: ports `bulk_f` (frame.origin), `tray_a`, `tray_b` (frame.origin), `bms_a`, `bms_b`, `pdb`,
    `contactor`, `dcdc_jetson`, `dcdc_aux` (each the type of the held part's base port), `estop` (the e-stop part's panel port
    type, on a rear lip, head pointing -x).
  - `liion-cell-ocv-chen2020-lgm50`: category `battery.cell_curve`, the OCV-SOC table of the LG M50T 21700 (NMC811) from Chen et
    al. 2020, J. Electrochem. Soc. 167 080534 (the Auline S70 cell chemistry is not published: NMC 21700 assumed, `literature`).

- [ ] **Step 1: failing tests** (append to `tests/python/test_dog_parts.py`):

```python
from napl.design import Library
from napl.design.snap import incompatible

PEAK_BUS_A = 87.0  # worst simultaneous stance peak, docs/dog/actuator-study.md finding 2 (GIM8108-8, r = 1.0)


def test_knee_rod_does_not_buckle_at_peak_knee_torque():  # Review Focus 1
    act = load("actuators/steadywin-gim8108-8")
    r1 = v(load("structure/dog-knee-crank"), "crank_radius")
    f_rod = v(act, "torque_peak") / r1  # the crank turns the whole stall torque into rod force (~1.1 kN)
    assert v(load("structure/generic-phs5-rod-end-m5"), "load_static_radial") >= 2.0 * f_rod
    for L in ("200", "220", "250"):
        rod = load(f"structure/dog-knee-rod-{L}")
        od, idd = v(rod, "outer_diameter"), v(rod, "inner_diameter")
        i_area = math.pi / 64 * (od**4 - idd**4)
        p_cr = math.pi**2 * v(rod, "youngs_modulus_axial") * i_area / v(rod, "pin_length") ** 2  # Euler, pinned-pinned
        assert p_cr >= 2.0 * f_rod, (L, p_cr, f_rod)
        assert v(rod, "buckling_load") == pytest.approx(p_cr, rel=1e-3)


def test_contactor_and_parallel_packs_cover_the_study_peak():
    c = load("electronics/generic-zjw100a-dc-contactor-12v")
    assert v(c, "i_rated_continuous") >= PEAK_BUS_A and v(c, "v_rated_contact") >= 33.6
    assert v(c, "coil_voltage") == pytest.approx(12.0) and v(c, "coil_current") <= 1.0
    lib = Library(ROOT / "parts")
    assert 2 * v(load("propulsion/auline-21700-s70-8s1p-4500mah"), "i_max_continuous") >= PEAK_BUS_A
    bms_port = next(p for p in lib.get("daly-k-series-smart-bms-8s-60a").ports if p["name"] == "pack")
    assert 2 * bms_port["i_max"]["value"] >= PEAK_BUS_A
    pdb_in = next(p for p in lib.get("dog-power-distribution-board").ports if p["name"] == "bat_in")
    assert pdb_in["i_max"]["value"] >= PEAK_BUS_A


DECK_SEATS = [  # (custom part, port, held part, port)
    ("dog-deck-bottom-cf3", "bms_a", "daly-k-series-smart-bms-8s-60a", "base"),
    ("dog-deck-bottom-cf3", "pdb", "dog-power-distribution-board", "base"),
    ("dog-deck-bottom-cf3", "contactor", "generic-zjw100a-dc-contactor-12v", "base"),
    ("dog-deck-bottom-cf3", "dcdc_jetson", "pololu-d24v22f12", "base"),
    ("dog-deck-bottom-cf3", "estop", "generic-estop-22mm-nc", "panel"),
    ("dog-compute-tray", "can_fl", "makerbase-mks-canable-v2-pro", "base"),
    ("dog-abad-mount", "hub", "dog-leg-hub", "base"),
    ("dog-battery-tray", "origin", "auline-21700-s70-8s1p-4500mah", "base"),
    ("dog-knee-rod-220", "end_a", "generic-phs5-rod-end-m5", "thread"),
]


@pytest.mark.parametrize("pp,pport,bp,bport", DECK_SEATS)
def test_custom_seat_mates_its_part(pp, pport, bp, bport):
    lib = Library(ROOT / "parts")
    port = lambda pid, n: next(p for p in lib.get(pid).ports if p["name"] == n)
    assert incompatible(port(pp, pport), port(bp, bport)) is None
```

  Also extend `PRICED` with `electronics/generic-zjw100a-dc-contactor-12v`, `electronics/dog-leg-hub`,
  `structure/generic-phs5-rod-end-m5`, `structure/dog-knee-rod-220`, `structure/dog-deck-top-cf3`,
  `structure/dog-deck-bottom-cf3`, `electronics/pololu-minimu-9-v6`, `electronics/ublox-ann-mb-00`,
  `electronics/generic-estop-22mm-nc`. (Use the e-stop's real panel-port name from its file in `DECK_SEATS`; `panel` is the
  expected name: if the file names it differently, use that name.)
- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_parts.py -n 12'`
- [ ] **Step 3: research and write the parts** (real listings, dated USD prices, `[[sources]]` with quotes, as SP3a):
  - PHS5 (DIN ISO 12240-4 series K, female M5 rod end): static radial load from a catalogue (e.g. a PHS5 datasheet table), mass,
    price; `[mirror] symmetry = "xz"`.
  - Knee rods: `pin_length` = 0.200 / 0.220 / 0.250 m (rod-end pin centres = thigh length, the linkage is a near-parallelogram);
    tube length = pin_length - 2 x (rod-end centre-to-face + insert shoulder) from the PHS5 table; mass, price and
    `youngs_modulus_axial` derived from `cf-tube-10x8` (if that part lacks an axial modulus, take it from its vendor table and add
    it there with the source); `buckling_load` derived with the Euler formula of the test. `[mirror] symmetry = "xz"`.
  - Contactor ZJW100A-class 100 A DC contactor with 12 V coil (a listing with a datasheet table: contact rating, coil resistance
    or current, contact resistance, mass). Pin-level `coil` connector (2 pins), screw studs for `line` / `load`.
  - Leg hub: CUSTOM PCB (as `dog-power-distribution-board`), 2 oz copper, XT60 in, 3 x XT30(2+2)-M out, JST-GH 4P CAN in, 120 ohm
    CAN termination (the far end of each leg bus; the CANable's own switch terminates the near end); `r_path` from trace geometry
    (derived); price from a dated PCB-fab quote page + connector prices (derived).
  - Decks: as `rover-chassis-plate-custom` (box geometry, `mass` = 4.11 kg/m^2 x outline area, price from `cf-plate-3mm` sheet
    price x area + a dated cutting-service price, derived). Seat port positions from the Task 7 layout; the seat types copy the
    held part's base-port type.
  - Li-ion OCV curve: same table format as `parts/common/lipo-cell-ocv-chen2006.toml`; values from Chen 2020 (literature, the
    paper's OCV fit / data table at 25 degC); the pack files keep `cells_series` 8.
  - Ports added to existing parts get their position from the source drawing / product photo cited in that file (fidelity
    `estimated` with a rationale where no drawing exists); the Jetson J12 / J5 pins already exist in its connector tables.
- [ ] **Step 4: run, expect PASS + 0 problems:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_parts.py tests/python/test_dog_cad.py -n 12 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
- [ ] **Step 5: commit** `git add parts tests/python/test_dog_parts.py && git commit -m "parts: dog knee push rods + PHS5 rod ends, 100 A contactor, leg hub, CF decks, Li-ion OCV, base/USB/UART/I2C ports" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 4: `assemblies/dog/leg.toml` — the leg sub-assembly

**Files:**
- Create: `assemblies/dog/leg.toml`
- Test: `tests/python/test_dog_leg.py`

**Interfaces:**
- Consumes: Task 1-3 parts; GROUND devices `qdd`, `foot_contact`.
- Produces (used by Tasks 7-9): instances `abad_mount` (root), `abad_motor`, `hip_housing`, `hip_motor`, `knee_motor`,
  `knee_crank`, `thigh`, `knee_rod`, `rod_end_a`, `rod_end_b`, `shank`, `foot_core`, `foot_ball`, `hub`; hinge mates `abad_j`,
  `hip_j`, `knee_j`; devices `abad`, `hip`, `knee` (qdd), `foot` (foot_contact); `[exports] ports = { hip_mount =
  "abad_mount.chassis" }`, `connectors = { power = "hub.power_in", can = "hub.can_in" }`, `params = { knee_drive_ratio =
  "knee_motor.drive_ratio" }`; instance param `knee_motor.drive_ratio` (= 8 x 1.5 = 12.0, derived); `[device_maps.qdd]`.

- [ ] **Step 1: failing test** `tests/python/test_dog_leg.py`:

```python
"""Dog leg sub-assembly (assemblies/dog/leg.toml): builds alone, DRC clean apart from being unpowered, three Qdds + a foot contact
with every physical field from the part files, the knee on the linkage ratio."""

import json
import pathlib
import xml.etree.ElementTree as ET

import pytest
from napl import design
from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, LEG = ROOT / "parts", ROOT / "assemblies" / "dog" / "leg.toml"
LIB = Library(PARTS)
STANDALONE_OK = {"NET_NO_SOURCE"}  # a leg alone has no battery; anything else is a real error


def devices_json(xml_text):
    return json.loads(ET.fromstring(xml_text).find("custom/text[@name='napl.devices']").get("data"))


def test_leg_drc_has_no_errors_but_the_missing_source():
    fs = check(A.load(LEG), LIB)
    assert [f.code for f in fs if f.severity == "error" and f.code not in STANDALONE_OK] == []


def test_leg_exports_and_mass(tmp_path):
    asm = A.load(LEG)
    ex = asm.raw["exports"]
    assert ex["ports"] == {"hip_mount": "abad_mount.chassis"}
    assert ex["connectors"] == {"power": "hub.power_in", "can": "hub.can_in"}
    assert ex["params"] == {"knee_drive_ratio": "knee_motor.drive_ratio"}
    _, rep = design.compute(asm, PARTS, tmp_path)
    assert 1.4 < rep["mass"]["total"] < 2.4  # 3 x 0.396 kg actuators + printed parts + rod + foot + hub + cables


def test_leg_qdds_come_from_the_part_and_the_knee_from_the_linkage(tmp_path):
    xml, rep = design.compute(A.load(LEG), PARTS, tmp_path)
    dev = devices_json(xml)
    q = {d["name"]: d for d in dev["qdds"]}
    assert sorted(q) == ["abad", "hip", "knee"]
    act = LIB.get("steadywin-gim8108-8")
    for n in ("abad", "hip"):
        assert q[n]["gear_ratio"] == act.value("gear_ratio")
        assert q[n]["i_limit"] == act.value("i_limit_phase")
        assert q[n]["motor"]["kv"] == pytest.approx(act.value("kv"))
    r = LIB.get("dog-shank").value("lug_radius") / LIB.get("dog-knee-crank").value("crank_radius")
    assert q["knee"]["gear_ratio"] == pytest.approx(act.value("gear_ratio") * r)
    assert [j["joint"] for j in dev["qdds"]] == ["abad_j", "hip_j", "knee_j"]
    (fc,) = dev["foot_contacts"]
    assert fc["name"] == "foot" and fc["geom"] == "foot_ball"
```

- [ ] **Step 2: run, expect FAIL** (file missing): `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_leg.py -n 12'`
- [ ] **Step 3: write `assemblies/dog/leg.toml`.** Header + structure (fill each `range` / `length` / `set` provenance from the
  Task 2-3 part files; the shape below is binding, the rationales are the worker's):

```toml
# One leg of the dog (SP3b, docs/superpowers/plans/2026-10-16-sp3b-dog-assembly.md Task 4). Written as the LEFT leg; dog.toml mirrors it
# ("xz") for the right side and mounts the rear pair turned 180 deg about z on the rear bulkhead.
# LEG FRAME = the abad mount's part frame: origin on the abduction axis in the bulkhead plane, x along the abduction axis away from the
# body, y outboard, z up. Bodies:
#   abad_mount (root, on a bulkhead; abad motor stator inside the body) -(abad_j)-> hip_housing -> {hip_motor, knee_motor (coaxial with
#   the hip pitch axis, inboard), knee_crank} -(hip_j)-> thigh -> {knee_rod, rod_end_a, rod_end_b} -(knee_j)-> shank -> foot_core
#   -> foot_ball;  abad_mount -> hub
# KNEE DRIVE: knee_motor turns knee_crank (dog-knee-crank.crank_radius); knee_rod pushes the shank lug (dog-shank.lug_radius). The Qdd
# "knee" acts on knee_j with gear_ratio = knee_motor.drive_ratio = 8 x lug / crank, and joint-side MIT limits scaled by the ratio.
# NOT MODELLED (ponytail; upgrade = an angle-dependent ratio in the Qdd): the linkage ratio's change with knee angle (constant at its
# nominal value), the hip / knee cross-coupling of a hip-mounted knee motor (real motor angle = q_hip + r q_knee), rod-end friction,
# the linkage's own loss (efficiency = the actuator's), self-collision (vehicle geoms collide with the world only).
# CAN: the hub joins the leg's bus (from its CANable) to the three drives on short stubs and terminates it (120 ohm).

[assembly]
name = "dog-leg"
description = "Dog leg: 3 x SteadyWin GIM8108-8 (abduction, hip pitch, knee via crank + push rod at 1.5:1), printed PLA links, rubber-ball foot, CAN/power hub"
base = "fixed"   # standalone bench: hangs from the hip mount
timestep = 0.001
seed = 1

[exports]
ports = { hip_mount = "abad_mount.chassis" }
connectors = { power = "hub.power_in", can = "hub.can_in" }
params = { knee_drive_ratio = "knee_motor.drive_ratio" }

[[instances]]
id = "abad_mount"
part = "dog-abad-mount"
root = true
group = "leg"
pos = [0.0, 0.0, 0.6]   # bench only (dropped when included): the straight leg (~0.49 m) clears the floor

[[instances]]
id = "abad_motor"
part = "steadywin-gim8108-8"
group = "leg"

[[instances]]
id = "hip_housing"
part = "dog-hip-housing-l"
group = "leg"

[[instances]]
id = "hip_motor"
part = "steadywin-gim8108-8"
group = "leg"

[[instances]]
id = "knee_motor"
part = "steadywin-gim8108-8"
group = "leg"
params = { drive_ratio = { value = 12.0, unit = "1", fidelity = "derived", derivation = "gear_ratio 8 (steadywin-gim8108-8) x linkage lug / crank = dog-shank.lug_radius 0.030 / dog-knee-crank.crank_radius 0.020 = 1.5 -> 12.0 motor turns per knee turn (study candidate 1)." } }

[[instances]]
id = "knee_crank"
part = "dog-knee-crank"
group = "leg"

[[instances]]
id = "thigh"
part = "dog-thigh-bracket-l"
group = "leg"

[[instances]]
id = "knee_rod"
part = "dog-knee-rod-220"
group = "leg"

[[instances]]
id = "rod_end_a"
part = "generic-phs5-rod-end-m5"
group = "leg"

[[instances]]
id = "rod_end_b"
part = "generic-phs5-rod-end-m5"
group = "leg"

[[instances]]
id = "shank"
part = "dog-shank"
group = "leg"

[[instances]]
id = "foot_core"
part = "dog-foot-core"
group = "leg"

[[instances]]
id = "foot_ball"
part = "mcmaster-natural-rubber-ball-1-3-8in"
group = "leg"

[[instances]]
id = "hub"
part = "dog-leg-hub"
group = "leg"

[[mates]]
name = "abad_seat"
a = "abad_mount.motor"
b = "abad_motor.mount"

[[mates]]
name = "abad_j"
a = "abad_motor.output"
b = "hip_housing.abad"
joint = { type = "hinge", range = { value = [-0.6, 0.6], unit = "rad", fidelity = "estimated", rationale = "Abduction travel before the hip housing meets the bulkhead / the rear leg (Task 7 INTERFERENCE check at both limits)." } }

[[mates]]
name = "hip_seat"
a = "hip_housing.pitch_seat"
b = "hip_motor.mount"

[[mates]]
name = "knee_seat"
a = "hip_housing.knee_seat"
b = "knee_motor.mount"

[[mates]]
name = "crank_fix"
a = "knee_motor.output"
b = "knee_crank.motor"

[[mates]]
name = "hip_j"
a = "hip_motor.output"
b = "thigh.origin"
joint = { type = "hinge", range = { value = [-1.6, 1.6], unit = "rad", fidelity = "estimated", rationale = "Hip pitch travel limited by the thigh meeting the hip housing arm (CAD)." } }

[[mates]]
name = "rod_fix"
a = "thigh.rod"
b = "knee_rod.mount"

[[mates]]
name = "rod_end_a_fix"
a = "knee_rod.end_a"
b = "rod_end_a.thread"

[[mates]]
name = "rod_end_b_fix"
a = "knee_rod.end_b"
b = "rod_end_b.thread"

[[mates]]
name = "knee_j"
a = "thigh.knee"
b = "shank.origin"
joint = { type = "hinge", range = "thigh.knee_range" }

[[mates]]
name = "foot_fix"
a = "shank.foot"
b = "foot_core.shank"

[[mates]]
name = "ball_fix"
a = "foot_core.ball"
b = "foot_ball.seat"

[[mates]]
name = "hub_fix"
a = "abad_mount.hub"
b = "hub.base"

# ---- harness: one CubeMars-style XT30(2+2) set per drive = a power pair (pins 1-2) and a CAN pair (pins 3-4) of the same cable ----
[[wires]]
name = "abad_pwr"
from = "hub.out_abad"
to = "abad_motor.power"
from_pins = ["OA.1", "OA.2"]
to_pins = ["PWR.1", "PWR.2"]
part = "cubemars-ak-v3-power-can-cable"
conductors = 2
length = { value = 0.15, unit = "m", fidelity = "estimated", rationale = "Hub under the abad cup to the abad drive's rear connector: about 0.12 m with slack." }

[[wires]]
name = "abad_can"
from = "hub.can_abad"
to = "abad_motor.can"
from_pins = ["OA.3", "OA.4"]
to_pins = ["PWR.3", "PWR.4"]
part = "cubemars-ak-v3-power-can-cable"
conductors = 2
length = { value = 0.15, unit = "m", fidelity = "estimated", rationale = "Same cable as abad_pwr (its CAN pair)." }
# hip_pwr / hip_can (OH.*, to hip_motor), knee_pwr / knee_can (OK.*, to knee_motor): same pattern, lengths 0.30 m (they cross the
# abduction hinge: the routed length + a service loop), each with its own rationale.

[device_maps.qdd]
"motor.kv" = "kv"
"motor.r20" = "r_phase_to_phase"
"motor.rotor_inertia" = "rotor_inertia"
"motor.c_th" = "c_th_winding"
"motor.r_th" = "r_th_winding"
gear_ratio = "gear_ratio"
efficiency = "efficiency"
encoder_cpr = "encoder_cpr"
cmd_hz = "can_cmd_hz"
latency_s = "can_latency"
i_limit = "i_limit_phase"
kp_max = "mit_kp_max"
kd_max = "mit_kd_max"
tau_ff_max = "mit_tau_ff_max"
timeout_s = "can_timeout"
kd_timeout = "kd_timeout"
t_max = "temp_fault"
derate_start = "temp_derate_start"

[[devices]]
type = "qdd"
name = "abad"
instance = "abad_motor"
port = "power"
joint = "abad_j"
map = "qdd"
timeout_mode = 0

[[devices]]
type = "qdd"
name = "hip"
instance = "hip_motor"
port = "power"
joint = "hip_j"
map = "qdd"
timeout_mode = 0

[[devices]]
type = "qdd"
name = "knee"
instance = "knee_motor"
port = "power"
joint = "knee_j"
map = "qdd"
timeout_mode = 0
set = { gear_ratio = "knee_motor.drive_ratio", tau_ff_max = { value = 44.0, unit = "N*m", fidelity = "derived", derivation = "Joint-side feed-forward range = mit_tau_ff_max 22 N*m x the largest swept linkage ratio 2.0 (the current limit bounds the torque anyway)." }, kp_max = { value = 2000.0, unit = "N*m/rad", fidelity = "derived", derivation = "Joint-side = mit_kp_max 500 x 2.0^2." }, kd_max = { value = 20.0, unit = "N*m*s/rad", fidelity = "derived", derivation = "Joint-side = mit_kd_max 5 x 2.0^2." } }

[[devices]]
type = "foot_contact"
name = "foot"
instance = "foot_ball"
shape = 0
mode = 1
set = { f_max = { value = 600.0, unit = "N", fidelity = "estimated", rationale = "VIRTUAL sensor: no foot sensor is fitted; this stands in for SP4's joint-torque contact estimate. Range above the ~250 N peak foot force of the study." }, noise_std = { value = 5.0, unit = "N", fidelity = "estimated", rationale = "Torque-based foot force estimates of QDD legs are good to a few N to ~10 N (Bledt et al., 2018, contact detection on MIT Cheetah 3); 5 N taken." }, hysteresis_on = { value = 30.0, unit = "N", fidelity = "estimated", rationale = "Touchdown above ~25 % of the static per-foot load (12 kg x g / 4 = 29 N)." }, hysteresis_off = { value = 15.0, unit = "N", fidelity = "estimated", rationale = "Half the on threshold." }, debounce_s = { value = 0.004, unit = "s", fidelity = "estimated", rationale = "Two 500 Hz control periods." }, rate_hz = { value = 500.0, unit = "Hz", fidelity = "estimated", rationale = "The estimator runs at the joint command rate (spec s6)." } }
```

- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_leg.py -n 12'`.
  If the DRC reports another code that only an unpowered sub-assembly can have, add it to `STANDALONE_OK` with a one-line reason
  (never a mechanical, wiring or rating code).
- [ ] **Step 5: commit** `git add assemblies/dog/leg.toml tests/python/test_dog_leg.py && git commit -m "assemblies/dog: leg sub-assembly (abduction, hip, knee linkage, Qdds, virtual foot contact, CAN/power hub)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 5: `assemblies/dog/power.toml` — two packs, two BMS, contactor, e-stop, DC-DCs, PDB

**Files:**
- Create: `assemblies/dog/power.toml`
- Test: `tests/python/test_dog_power.py`

**Interfaces:**
- Consumes: Task 3 parts, `auline-21700-s70-8s1p-4500mah`, `daly-k-series-smart-bms-8s-60a`, `dog-battery-tray`,
  `dog-power-distribution-board`, `pololu-d24v22f12`, `generic-estop-22mm-nc`, `liion-cell-ocv-chen2020-lgm50`.
- Produces: instances `deck_bottom` (root), `tray_a`, `tray_b`, `pack_a`, `pack_b`, `bms_a`, `bms_b`, `contactor`, `pdb`,
  `dcdc_jetson`, `dcdc_aux`, `estop`; devices `pack_a`, `pack_b` (battery), `contactor_path`, `pdb_fl`, `pdb_fr`, `pdb_rl`,
  `pdb_rr` (wire), `load_jetson`, `load_aux` (load); wires `estop_in`, `estop_out`, `coil_ret` plus the pack / BMS / contactor /
  PDB / DC-DC harness; `[exports] ports = { chassis = "deck_bottom.bulk_f" }`, `connectors = { leg_fl = "pdb.leg_fl", leg_fr =
  "pdb.leg_fr", leg_rl = "pdb.leg_rl", leg_rr = "pdb.leg_rr", v12_jetson = "dcdc_jetson.vout", v12_lidar = "dcdc_aux.vout" }`.

- [ ] **Step 1: failing test** `tests/python/test_dog_power.py`:

```python
"""Dog power module (assemblies/dog/power.toml): 2 x (8S1P pack + 60 A BMS) in parallel, a 100 A contactor switched by the e-stop
through its 12 V coil, compute / lidar DC-DCs fed before the contactor, the PDB after it."""

import pathlib

from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, POWER = ROOT / "parts", ROOT / "assemblies" / "dog" / "power.toml"
LIB = Library(PARTS)
PEAK_BUS_A = 87.0  # docs/dog/actuator-study.md finding 2


def test_power_module_is_drc_clean():
    fs = check(A.load(POWER), LIB)
    assert [f.code for f in fs if f.severity == "error"] == []


def test_estop_switches_only_the_coil_and_ratings_cover_the_peak():  # Review Focus 4
    asm = A.load(POWER)
    ends = [(w["from"], w["to"]) for w in asm.doc["wires"] if "estop." in w["from"] + w["to"]]
    assert sorted(w["name"] for w in asm.doc["wires"] if "estop." in w["from"] + w["to"]) == ["estop_in", "estop_out"]
    assert {e for pair in ends for e in pair if not e.startswith("estop.")} == {"dcdc_aux.vout", "contactor.coil"}
    diag = {}
    check(asm, LIB, diag=diag)
    coil = LIB.get("generic-zjw100a-dc-contactor-12v").value("coil_current")
    for w in ("estop_in", "estop_out"):
        peak, _ = diag["currents"][w]
        assert peak <= 1.05 * coil, (w, peak)
    # every series element of the motor feed is rated for the study's simultaneous peak
    port = lambda pid, n: next(p for p in LIB.get(pid).ports if p["name"] == n)
    assert 2 * LIB.get("auline-21700-s70-8s1p-4500mah").value("i_max_continuous") >= PEAK_BUS_A
    assert 2 * port("daly-k-series-smart-bms-8s-60a", "pack")["i_max"]["value"] >= PEAK_BUS_A
    assert LIB.get("generic-zjw100a-dc-contactor-12v").value("i_rated_continuous") >= PEAK_BUS_A
    assert port("dog-power-distribution-board", "bat_in")["i_max"]["value"] >= PEAK_BUS_A
    # the compute / lidar DC-DCs hang on the contactor's LINE side: the e-stop never drops the recon payload
    vin = {w["to"] for w in asm.doc["wires"] if w["from"] == "contactor.line"}
    assert {"dcdc_jetson.vin", "dcdc_aux.vin"} <= vin


def test_two_batteries_with_the_liion_curve():
    asm = A.load(POWER)
    bats = {d["name"]: d for d in asm.doc["devices"] if d["type"] == "battery"}
    assert sorted(bats) == ["pack_a", "pack_b"]
    assert {b["ocv"] for b in bats.values()} == {"liion-cell-ocv-chen2020-lgm50"}
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_power.py -n 12'`
- [ ] **Step 3: write `assemblies/dog/power.toml`.** Topology (header comment states it; every length a provenance table):
  - mates: `deck_bottom.tray_a` <- `tray_a.origin`, `tray_a.origin`... (each pack on its tray via the pack `base` port), BMS /
    PDB / contactor / both DC-DCs / e-stop on their deck seats.
  - pack positive: `pack_a.power_out` pin OUT.1 and `pack_b.power_out` OUT.1 -> `contactor.line` (`awg-12-silicone-wire`,
    joinable stud); pack negative OUT.2 -> `bms_a.pack` pin `PWR.1` (B-) / `bms_b.pack` `PWR.1`; balance leads
    `pack_x.balance` -> `bms_x.balance` (9 pins, `included_in = "pack_x"`).
  - `contactor.line` -> `dcdc_jetson.vin`, `dcdc_aux.vin` (VIN pads; `awg-20-silicone-wire`); `contactor.load` -> `pdb.bat_in`
    V+; `bms_a.pack` `PWR.2` (P-) and `bms_b.pack` `PWR.2` -> `pdb.bat_in` GND, `dcdc_jetson.vin` GND, `dcdc_aux.vin` GND.
  - e-stop loop: `estop_in` = `dcdc_aux.vout` (VOUT) -> `estop.contact` pin 1; `estop_out` = `estop.contact` pin 2 ->
    `contactor.coil` pin 1; `coil_ret` = `contactor.coil` pin 2 -> `dcdc_aux.vout` GND.
  - devices: `battery` `pack_a` / `pack_b` (`ocv = "liion-cell-ocv-chen2020-lgm50"`, map `capacity_ah`, `r0 =
    "r_internal_pack"`; `r1` / `c1` as `assemblies/skydio.toml` with the pack's own provenance or omitted if the battery device
    does not require them); `wire` `contactor_path` (instance `contactor`, a `line`, b `load`, `resistance = "r_contact"` via a
    map); `wire` `pdb_fl` ... `pdb_rr` (instance `pdb`, a `bat_in`, b `leg_xx`, map `r_path`); `load` `load_jetson` on
    `dcdc_jetson.vin` and `load_aux` on `dcdc_aux.vin`, `i0` derived at the pack side (as `assemblies/skydio.toml` `load_fc`):
    Jetson 15 W (the devkit's 15 W power mode; datasheet) / (D24V22 efficiency at 12 V out from its curve, datasheet) / 29.6 V
    nominal; aux = (Mid-360 6.5 W + 12 V x contactor coil_current) / (efficiency x 29.6 V). The BMS FET on-resistance is not
    modelled (< 2 % of r_internal_pack; say so in the header).
  - root `deck_bottom` placed `pos = [0.0, 0.0, 0.2]` (bench only).
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_power.py -n 12'`
- [ ] **Step 5: commit** `git add assemblies/dog/power.toml tests/python/test_dog_power.py && git commit -m "assemblies/dog: power module (2 x 8S1P + BMS in parallel, 100 A contactor on an e-stop coil loop, pre-contactor DC-DCs)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 6: `assemblies/dog/head.toml` and `assemblies/dog/compute.toml`

**Files:**
- Create: `assemblies/dog/head.toml`, `assemblies/dog/compute.toml`
- Test: `tests/python/test_dog_head_compute.py`

**Interfaces:**
- Consumes: Task 2-3 parts; GROUND rangefinder rosette fields; the D435i / IMU / GNSS device maps already used in
  `assemblies/depth-bench.toml`, `assemblies/skydio.toml` (copy the map tables, do not invent fields).
- Produces:
  - head: instances `head_mount` (root), `lidar` (`livox-mid-360`), `cam` (`intel-realsense-d435i`); devices `lidar`
    (rangefinder, `projection = "rosette"`, `site = "lidar"`, `site_port = "optical"`, `rays_per_frame` from the instance param
    `lidar.rays_per_frame_sim`), `depth` (rangefinder pinhole on `cam`); `[exports] ports = { mount = "head_mount.origin" }`,
    `connectors = { lidar_power = "lidar.power", lidar_eth = "lidar.eth", cam_usb = "cam.usb" }`, `params = { lidar_rays =
    "lidar.rays_per_frame_sim" }`.
  - compute: instances `tray` (root, `dog-compute-tray`), `jetson`, `can_fl`, `can_fr`, `can_rl`, `can_rr`, `imu`
    (`pololu-minimu-9-v6`), `rtk` (`sparkfun-gps-rtk-sma-16481`), `ant` (`ublox-ann-mb-00`); devices `imu`, `gnss`; wires
    `usb_can_fl` ... (CANable `usb` -> `jetson.usb_j6` / `jetson.usb_j7`, two each), `rtk_uart` (`rtk.uart` -> `jetson.uart1`),
    `imu_i2c` (`imu` I2C pins -> `jetson.i2c1`), `rtk_ant` (`rtk.antenna` -> `ant`, `included_in = "ant"`); `[exports] ports =
    { mount = "tray.origin" }`, `connectors = { power_in = "jetson.power_in", eth = "jetson.<RJ45 port name>", usb_cam =
    "jetson.usb_j5", can_fl = "can_fl.can", can_fr = "can_fr.can", can_rl = "can_rl.can", can_rr = "can_rr.can" }` (the RJ45 port
    name from the Jetson file).

- [ ] **Step 1: failing test** `tests/python/test_dog_head_compute.py`:

```python
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
                out[s.get("name")] = Rb @ quat_R([float(v) for v in s.get("quat", "1 0 0 0").split()])
            walk(b, Rb)

    walk(ET.fromstring(xml_text).find("worldbody"), np.eye(3))
    return out


def test_head_lidar_scans_the_mid360_envelope_tilted_25_deg(tmp_path):
    asm = A.load(DOG / "head.toml")
    assert [f.code for f in check(asm, LIB) if f.severity == "error" and f.code not in STANDALONE_OK] == []
    xml, _ = design.compute(asm, PARTS, tmp_path)
    dev = json.loads(ET.fromstring(xml).find("custom/text[@name='napl.devices']").get("data"))
    lid = {r["name"]: r for r in dev["rangefinders"]}["lidar"]
    assert lid["projection"] == "rosette"
    assert lid["fov_v_min"] == pytest.approx(math.radians(-7), abs=1e-6)
    assert lid["fov_v_max"] == pytest.approx(math.radians(52), abs=1e-6)
    up = site_frames(xml)[lid["site"]][:, 0]  # rosette: site +x = the sensor's up axis (docs/models.md "Rosette scan pattern")
    assert up[0] > 0  # nose-down tilt leans the up axis forward
    assert math.degrees(math.acos(up[2])) == pytest.approx(25.0, abs=0.5)


def test_compute_wires_four_can_buses_and_the_sensors():
    asm = A.load(DOG / "compute.toml")
    assert [f.code for f in check(asm, LIB) if f.severity == "error" and f.code not in STANDALONE_OK] == []
    cans = sorted(i["id"] for i in asm.doc["instances"] if i["part"] == "makerbase-mks-canable-v2-pro")
    assert cans == ["can_fl", "can_fr", "can_rl", "can_rr"]
    usb = {w["from"].split(".")[0]: w for w in asm.doc["wires"] if w["from"].split(".")[0] in cans}
    assert sorted(usb) == cans and {w["to"] for w in usb.values()} == {"jetson.usb_j6", "jetson.usb_j7"}
    pins = [p for w in usb.values() for p in w["to_pins"]]
    assert len(pins) == len(set(pins))  # four distinct USB-A sockets
    wires = {w["name"]: w for w in asm.doc["wires"]}
    assert wires["rtk_uart"]["to"] == "jetson.uart1" and wires["imu_i2c"]["to"] == "jetson.i2c1"
    ex = asm.raw["exports"]["connectors"]
    assert {"power_in", "eth", "usb_cam", "can_fl", "can_fr", "can_rl", "can_rr"} <= set(ex)
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_head_compute.py -n 12'`
- [ ] **Step 3: write both files.** head: mates `head_mount.lidar` <- `lidar.base`, `head_mount.camera` <- `cam.mount`; lidar
  device `set` = `{ fov_h = {value = 6.283185, unit = "rad", ..datasheet from livox-mid-360}, fov_v_min, fov_v_max, min_range,
  max_range, rate_hz (10 Hz), rays_per_frame = "lidar.rays_per_frame_sim", pattern_seed }`, numbers from the Mid-360 part params
  through a `[device_maps.mid360]` where the part has them; instance param `rays_per_frame_sim = { value = 5000, unit = "1",
  fidelity = "estimated", rationale = "Sim ray budget, not the sensor's 20 000 / frame: the 5k column of docs/models.md 'Ground sim
  budget' (0.1-4.6x real time per slot on the terrain kinds). SP5 overrides lidar_rays for mapping fidelity." }`; depth device:
  copy `assemblies/depth-bench.toml`'s D435i binding (resolution / rate as that file, reasoned there). compute: mates on the tray
  seats; USB wires (CANable `USB` connector -> Jetson `J6` / `J7` socket pins), UART (RTK UART1 TX/RX/GND -> J12 pins 10 / 8 / 6,
  crossed TX->RX), I2C (MinIMU VIN/GND/SDA/SCL -> J12 1 / 6 / 3 / 5), `rtk_ant`; devices `imu` (copy the MinIMU-class map of an
  existing assembly or map from the part's own params) and `gnss` (RTK map as in `assemblies/skydio-land.toml`). Root `pos =
  [0.0, 0.0, 0.1]` (bench only).
- [ ] **Step 4: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_head_compute.py -n 12'`
- [ ] **Step 5: commit** `git add assemblies/dog/head.toml assemblies/dog/compute.toml tests/python/test_dog_head_compute.py && git commit -m "assemblies/dog: head (Mid-360 rosette at 25 deg, D435i) and compute (Jetson, 4 CAN buses, IMU on I2C, RTK on UART)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 7: `assemblies/dog/dog.toml` — chassis + 4 legs + head + power + compute, wiring, BOM, mass

**Files:**
- Create: `assemblies/dog/dog.toml`, `assemblies/dog/README.md`
- Modify: `tests/python/test_wire_routing.py` (`WIRED += ["dog/dog"]`)
- Test: `tests/python/test_dog_assembly.py`

**Interfaces:**
- Consumes: Tasks 4-6 exports; `report["bom"]`, `report["mirror_signs"]`, `report["mass"]` (SP1).
- Produces: `[assembly] name = "dog"`, `base = "free"`; parent instances `deck_top` (root), `bulk_f`, `bulk_r`, `side_l`,
  `side_r`; `[[subassemblies]]` `fl` (leg, no mirror), `fr` (leg, `mirror = "xz"`), `rl` (leg, `mirror = "xz"`, on the rear
  bulkhead), `rr` (leg, no mirror, on the rear bulkhead), `head`, `power`, `compute`; parent wires `can_<leg>` and `pwr_<leg>`
  for each leg, `lidar_pwr`, `lidar_eth`, `cam_usb`, `jetson_pwr`; groups `chassis`, `legs`, `head`, `power`, `compute`; targets
  `dog_total_mass`; device names after flattening `<leg>:abad|hip|knee|foot`, `power:pack_a|pack_b`, `head:lidar`, `compute:imu`.

- [ ] **Step 1: failing test** `tests/python/test_dog_assembly.py`:

```python
"""The whole dog (assemblies/dog/dog.toml): DRC clean, mass ~12 kg with the CoM between the hips, BOM with every price, mirrored and
rotated legs in place, and every joint stalling at its datasheet torque from the real pack (Review Focus 2)."""

import json
import pathlib
import re
import xml.etree.ElementTree as ET

import napl
import numpy as np
import pytest
from napl import design
from napl.design import Library
from napl.design import assembly as A
from napl.design.drc import check

ROOT = pathlib.Path(__file__).resolve().parents[2]
PARTS, DOG = ROOT / "parts", ROOT / "assemblies" / "dog"
LIB = Library(PARTS)
LEGS = ("fl", "fr", "rl", "rr")
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    return design.build(DOG / "dog.toml", PARTS, tmp_path_factory.mktemp("dog"))


def test_dog_drc_has_no_errors():
    fs = check(A.load(DOG / "dog.toml"), LIB)
    assert [(f.code, f.message) for f in fs if f.severity == "error"] == []


def test_mass_bom_and_readme(built):
    rep = built["report"]
    assert 10.0 <= rep["mass"]["total"] <= 13.0  # spec s1: ~12 kg
    bom = rep["bom"]
    assert bom["missing"] == []  # every part priced (bom.py: wires / connectors are not counted, README says so)
    assert set(bom["by_sub"]) == {"fl", "fr", "rl", "rr", "head", "power", "compute"}
    readme = (DOG / "README.md").read_text()
    m = re.search(r"BOM total \(lower bound, wires excluded\): ([0-9,]+) USD", readme)
    assert m and abs(int(m.group(1).replace(",", "")) - round(bom["total"]["cost"])) <= 1


def test_mirrors_and_knee_ratio_override():
    asm = A.load(DOG / "dog.toml")
    subs = {s["id"]: s for s in asm.raw["subassemblies"]}
    assert subs["fr"].get("mirror") == "xz" and subs["rl"].get("mirror") == "xz"
    assert "mirror" not in subs["fl"] and "mirror" not in subs["rr"]
    p = DOG / "dog.toml"
    text = p.read_text().replace('id = "rl"\n', 'id = "rl"\nparams = { knee_drive_ratio = { value = 16.0, unit = "1", fidelity = "derived", derivation = "test" } }\n', 1)
    flat = A.loads(text, path=p).doc
    by = {i["id"]: i for i in flat["instances"]}
    assert by["rl:knee_motor"]["params"]["drive_ratio"]["value"] == 16.0
    assert by["fl:knee_motor"]["params"]["drive_ratio"]["value"] == 12.0


@needs_kernel
def test_feet_touch_the_floor_and_com_is_between_the_hips(built):
    sim = napl.Sim.load(built["xml"])
    pos = {p.rsplit("/", 1)[-1]: np.asarray(x) for p, x in sim.body_positions().items()}
    r_ball = LIB.get("mcmaster-natural-rubber-ball-1-3-8in").geometry["shapes"][0]["size"][0]
    for leg in LEGS:
        assert 0.0 <= pos[f"{leg}:foot_ball"][2] - r_ball <= 0.002  # build pose: straight legs standing on the floor
    rep = built["report"]
    m = {r["id"]: r["mass"] for r in rep["instances"] if r["id"] in pos}
    com = sum(m[i] * pos[i] for i in m) / sum(m.values())  # point masses at the part frames: an approximation, README says so
    hips = np.mean([pos[f"{leg}:abad_mount"] for leg in LEGS], axis=0)
    assert np.linalg.norm((com - hips)[:2]) <= 0.02


@needs_kernel
def test_every_joint_stalls_at_its_datasheet_torque(built):  # Review Focus 2
    sim = napl.Sim.load(built["xml"])
    act = LIB.get("steadywin-gim8108-8")
    r = LIB.get("dog-shank").value("lug_radius") / LIB.get("dog-knee-crank").value("crank_radius")
    for leg in LEGS:
        for j, ratio in (("abad", 1.0), ("hip", 1.0), ("knee", r)):
            s = sim.fork()
            d = f"/devices/{leg}:{j}"
            s.set(d, "enable", 1.0)
            s.set(d, "tau_ff", 2.0 * act.value("torque_peak"))  # saturating: the current limit decides
            s.step(3)  # 1 tick of CAN latency + 1 command period
            tau = float(s.query(d, "tau_est")[0][0])
            assert abs(tau) == pytest.approx(act.value("torque_peak") * ratio, rel=0.05), (d, tau)
            assert float(s.query(d, "voltage")[0][0]) > 8 * 3.6  # fed from the packs through contactor, PDB, hub, cable
```

  In `tests/python/test_wire_routing.py` change the `WIRED` definition's last line to
  `) + ["dog/dog"]` (the test builds `assemblies/<name>.toml`, so `"dog/dog"` resolves to `assemblies/dog/dog.toml`).
- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_assembly.py -n 12'`
- [ ] **Step 3: write `assemblies/dog/dog.toml`.** Header comment: world frame (x forward, y left, z up), the leg placement rule
  (front bulkhead: `fl` on `hip_l`, `fr` mirrored on `hip_r`; rear bulkhead is the same part turned 180 deg about z by
  `deck_top.bulk_r`, so its `hip_l` port lies at world -y: `rr` (unmirrored) on `bulk_r.hip_l`, `rl` (mirrored) on
  `bulk_r.hip_r`; front knees point backward, rear knees forward), the CAN plan (one bus per leg), the power plan (packs ->
  contactor -> PDB -> leg hubs; DC-DCs before the contactor). Sections:

```toml
[assembly]
name = "dog"
description = "Recon quadruped (SP3b): 12 x GIM8108-8 QDD legs (knee by crank + push rod 1.5:1), Mid-360 + D435i head, Jetson Orin Nano + 4 CAN buses, 2 x 8S1P Li-ion with BMS, contactor + e-stop"
base = "free"
timestep = 0.001
seed = 1

[[instances]]
id = "deck_top"
part = "dog-deck-top-cf3"
root = true
group = "chassis"
pos = [0.0, 0.0, 0.0]   # worker: z = the derived standing height of the deck at the build pose (straight legs, balls on the floor): the
                        # value test_feet_touch_the_floor_and_com_is_between_the_hips accepts; state the arithmetic in a comment

[[instances]]
id = "bulk_f"
part = "dog-bulkhead"
group = "chassis"

[[instances]]
id = "bulk_r"
part = "dog-bulkhead"
group = "chassis"
# side_l (dog-side-plate-l), side_r (dog-side-plate-r): group "chassis"

[[mates]]
name = "bulk_f_fix"
a = "deck_top.bulk_f"
b = "bulk_f.deck_top"

[[mates]]
name = "bulk_r_fix"
a = "deck_top.bulk_r"
b = "bulk_r.deck_top"
# side_l_fix / side_r_fix: deck_top.side_l -> side_l.top, deck_top.side_r -> side_r.top

[[subassemblies]]
id = "fl"
file = "leg.toml"
group = "legs"

[[subassemblies]]
id = "fr"
file = "leg.toml"
mirror = "xz"
group = "legs"

[[subassemblies]]
id = "rl"
file = "leg.toml"
mirror = "xz"
group = "legs"

[[subassemblies]]
id = "rr"
file = "leg.toml"
group = "legs"

[[subassemblies]]
id = "head"
file = "head.toml"
group = "head"
params = { lidar_rays = { value = 2000, unit = "1", fidelity = "estimated", rationale = "Flat-floor bench and stand runs: 2000 rays / frame keeps the lidar on without dominating the step (docs/models.md 'Ground sim budget'); SP5 raises it." } }

[[subassemblies]]
id = "power"
file = "power.toml"
group = "power"

[[subassemblies]]
id = "compute"
file = "compute.toml"
group = "compute"

[[mates]]
name = "fl_mount"
a = "bulk_f.hip_l"
b = "fl:hip_mount"

[[mates]]
name = "fr_mount"
a = "bulk_f.hip_r"
b = "fr:hip_mount"

[[mates]]
name = "rl_mount"
a = "bulk_r.hip_r"
b = "rl:hip_mount"

[[mates]]
name = "rr_mount"
a = "bulk_r.hip_l"
b = "rr:hip_mount"

[[mates]]
name = "head_mount"
a = "deck_top.head"
b = "head:mount"

[[mates]]
name = "compute_mount"
a = "deck_top.compute"
b = "compute:mount"

[[mates]]
name = "power_mount"
a = "bulk_f.deck_bottom"
b = "power:chassis"

[[wires]]
name = "can_fl"
from = "compute:can_fl"
to = "fl:can"
from_pins = ["CAN.1", "CAN.2", "CAN.3"]
to_pins = ["CANIN.1", "CANIN.2", "CANIN.3"]
part = "awg-24-silicone-wire"
conductors = 3
length = { value = 0.35, unit = "m", fidelity = "estimated", rationale = "CANable on the compute tray to the fl hub under the front-left abad cup: routed length + slack." }

[[wires]]
name = "pwr_fl"
from = "power:leg_fl"
to = "fl:power"
from_pins = ["FL.1", "FL.2"]
to_pins = ["PIN.1", "PIN.2"]
part = "awg-12-silicone-wire"
conductors = 2
length = { value = 0.25, unit = "m", fidelity = "estimated", rationale = "PDB on the bottom deck to the fl hub: routed length + slack." }
# can_fr / can_rl / can_rr, pwr_fr / pwr_rl / pwr_rr: same pattern, each with its own length rationale.
# lidar_pwr (power:v12_lidar -> head:lidar_power, part livox-mid-360-splitter-cable), lidar_eth (head:lidar_eth -> compute:eth,
# included_in = "head:lidar" is NOT allowed across exports: give it part = "livox-mid-360-splitter-cable" and lidar_pwr the
# awg-20 lead of the splitter's power tail), cam_usb (head:cam_usb -> compute:usb_cam, the D435i's own USB-C cable),
# jetson_pwr (power:v12_jetson -> compute:power_in, awg-18 to the J16 DC jack): pins from each part's connector table.

[[targets]]
name = "dog_total_mass"
all = true
value = { value = 12.0, unit = "kg", fidelity = "estimated", rationale = "Spec s1 design target '~12 kg' (Go2 / Mini Cheetah class), not a measurement." }
```

  Iterate the deck seat positions (Task 3 part) and `cad/dog/params.py` constants until `check()` has no error (in particular
  `INTERFERENCE` between the abad cups, the packs and the trays; `WIRE_*`; voltage windows: the D24V22 `v_max` 36 V vs the
  33.6 V full pack is a WARNING at most, note regen spikes in the README).
- [ ] **Step 4: README.** `assemblies/dog/README.md` (Write; numbers read from the hfbox build report, pulled with
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/napl design build assemblies/dog/dog.toml --out build/dog' && NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh pull build/dog`):
  sub-assembly tree; decisions 1-8 of this plan with their numbers (belt tension, rod buckling margin, pack / BMS / contactor
  margins); the mass table per group and per sub-assembly vs the 12 kg target; the point-mass CoM and trunk inertia estimate
  (Sum m_i (|r_i|^2 I - r_i r_i^T) about the CoM, labelled an approximation for SP4 to identify); the BOM per sub-assembly and
  the line `BOM total (lower bound, wires excluded): N USD` vs the 3-4.5 k target with the estimated share, plus the cable /
  fastener allowance of the study (80 USD) as a separate estimated line; open items (precharge, GDS68 MIT protocol, linkage
  ratio variation, D24V22 36 V margin vs regen, no foot sensor hardware).
- [ ] **Step 5: run, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_assembly.py tests/python/test_wire_routing.py -n 12'`
- [ ] **Step 6: commit** `git add assemblies/dog/dog.toml assemblies/dog/README.md tests/python/test_dog_assembly.py tests/python/test_wire_routing.py && git commit -m "assemblies/dog: the dog (chassis, 4 legs mirrored/rotated, head, power, compute), pin-level wiring, BOM and mass" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 8: stand / squat sanity sim with joint PD on the Qdds + golden state hash

**Files:**
- Create: `examples/dog/dogsim.py`, `examples/dog/scene.py`, `examples/dog/stand.py`, `examples/dog/stand.scenario.toml`
- Test: `tests/python/test_dog_stand.py`

**Interfaces:**
- Consumes: built `dog` (device paths `/devices/<leg>:<joint>`, `/devices/power:pack_a`, body prim names ending in the flat
  instance id, root body `/world/deck_top`), `report["mirror_signs"]` keyed by mate name (`fr:hip_j` ...).
- Produces (`examples/dog/dogsim.py`, reused by Task 9 and SP4): `ASM`, `PARTS`, `LEGS`, `JOINTS`, `KNEE_BACK`, `KP`, `KD`,
  `SQUAT`, `STAND`, `CMD_HZ`, `TRACE_COLS`, `build(out, asm=ASM) -> (Path, dict)`, `timestep(xml) -> float`,
  `signs(report) -> dict`, `body(sim, inst) -> np.ndarray`, `leg_lengths(sim, leg="fl") -> (l1, l2)`,
  `ik(d, l1, l2) -> (q_hip, q_knee)`, `enable(sim, sg)`, `command(sim, sg, q, kp=KP, kd=KD)`,
  `ramp(sim, sg, l1, l2, a, b, T, rows, z0=None)`, `sample(sim, sg, z_expected) -> list`, `finish(sim, run_dir, rows, dt)`;
  scenario `examples/dog/stand.scenario.toml` (`[interfaces.native]`, `[model] script = "scene.py"`); `GOLDEN` in
  `tests/python/test_dog_stand.py`.

- [ ] **Step 1: failing test** `tests/python/test_dog_stand.py`:

```python
"""Dog stand / squat sanity (examples/dog): the assembly loads and simulates on its 12 Qdds under a plain joint PD (no MPC yet), the
stance is symmetric (Review Focus 3), and the scenario replays to its golden state hash."""

import sys
from pathlib import Path

import napl
import numpy as np
import pytest
from napl import program

REPO = Path(__file__).resolve().parents[2]
EX = REPO / "examples" / "dog"
sys.path.insert(0, str(EX))
import dogsim as D  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")
GOLDEN = None  # stand.scenario.toml, seed 1: recorded in Step 5 from res["sim"]["state_hash"]


def test_golden_is_recorded():
    assert GOLDEN, "record the stand scenario's state hash (Task 8 Step 5)"


def test_ik_round_trip():
    l1, l2 = 0.22, 0.245
    for d in (0.30, 0.38, l1 + l2):
        qh, qk = D.ik(d, l1, l2)
        a, phi = -D.KNEE_BACK * qh, D.KNEE_BACK * qk  # thigh angle off vertical, knee flexion
        foot_x = l1 * np.sin(a) - l2 * np.sin(phi - a)
        foot_z = -l1 * np.cos(a) - l2 * np.cos(phi - a)
        assert abs(foot_x) < 1e-9 and foot_z == pytest.approx(-d)


@needs_kernel
def test_stance_is_symmetric(tmp_path):  # Review Focus 3
    xml, rep = D.build(tmp_path)
    sim = napl.Sim.load(xml)
    sg = D.signs(rep)
    assert {k for k, s in sg.items() if s == -1} <= {f"{leg}:{j}" for leg in ("fr", "rl") for j in D.JOINTS}
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    rows = []
    D.ramp(sim, sg, l1, l2, 1.0, D.SQUAT, 1.5, rows)
    D.ramp(sim, sg, l1, l2, D.SQUAT, D.SQUAT, 1.0, rows)
    hip = {leg: D.body(sim, f"{leg}:thigh") for leg in D.LEGS}
    knee = {leg: D.body(sim, f"{leg}:shank") for leg in D.LEGS}
    foot = {leg: D.body(sim, f"{leg}:foot_ball") for leg in D.LEGS}
    ctr = np.mean(list(hip.values()), axis=0)
    rel = {leg: foot[leg] - ctr for leg in D.LEGS}
    fy, fxy = np.array([1, -1, 1]), np.array([-1, -1, 1])
    assert np.allclose(rel["fr"], rel["fl"] * fy, atol=3e-3)  # left / right mirror
    assert np.allclose(rel["rr"], rel["fl"] * fxy, atol=1e-2)  # front / rear: the rear pair is the front pair turned 180 deg
    assert np.allclose(rel["rl"], rel["fr"] * fxy, atol=1e-2)
    for leg in ("fl", "fr"):
        assert knee[leg][0] < hip[leg][0] - 0.02  # front knees point backward
    for leg in ("rl", "rr"):
        assert knee[leg][0] > hip[leg][0] + 0.02  # rear knees point forward
    for leg in D.LEGS:
        assert abs(foot[leg][0] - hip[leg][0]) < 0.015  # the IK keeps each foot under its hip
    assert max(r[D.TRACE_COLS.index("fault")] for r in rows) == 0


@needs_kernel
def test_stand_scenario_passes_replays_and_matches_golden(tmp_path):
    res, _ = program.run(EX / "stand.scenario.toml", tmp_path / "runs")
    assert res["pass"], res["fail_reasons"]
    assert res["replay"]["available"] and res["replay"]["match"]
    assert GOLDEN is None or res["sim"]["state_hash"] == GOLDEN, res["sim"]["state_hash"]
```

- [ ] **Step 2: run, expect FAIL** (no `dogsim`): `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_stand.py -n 12'`
- [ ] **Step 3: implement `examples/dog/dogsim.py`:**

```python
"""Dog sim bench (SP3b): build the dog, per-leg joint signs from the build report, planar leg IK, MIT-mode joint PD on the 12 Qdd
devices, stand / squat ramps, and the [interfaces.native] artefacts. SP4 replaces the PD with its controller; this stays the bench.

Leg convention: q = 0 is the build pose (straight leg, foot under the hip). For the UNMIRRORED leg (fl) q_knee = KNEE_BACK * flexion
bends the knee backward; a mirrored leg multiplies by its report["mirror_signs"] entry; the rear legs are the front ones turned
180 deg, so the same q gives the turned pose (their knees point forward)."""

import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from napl import design

REPO = Path(__file__).resolve().parents[2]
ASM, PARTS = REPO / "assemblies" / "dog" / "dog.toml", REPO / "parts"
LEGS, JOINTS = ("fl", "fr", "rl", "rr"), ("abad", "hip", "knee")
KNEE_BACK = -1.0  # pinned by tests/python/test_dog_stand.py::test_stance_is_symmetric (flip it if the knees point the wrong way)
KP, KD = 60.0, 1.5  # N*m/rad, N*m*s/rad, joint side: stand gains
SQUAT, STAND = 0.65, 0.82  # hip-to-ball distance as a fraction of l1 + l2
CMD_HZ = 500.0  # joint command rate (spec s6)
TRACE_COLS = ["t", "h_err", "tilt", "fault", "v_bat"]


def build(out, asm=ASM):
    r = design.build(asm, PARTS, out)
    return Path(r["xml"]), r["report"]


def timestep(xml):
    return float(ET.parse(xml).getroot().find("option").get("timestep"))


def signs(report):
    ms = report["mirror_signs"]
    return {f"{leg}:{j}": ms.get(f"{leg}:{j}_j", 1) for leg in LEGS for j in JOINTS}


def body(sim, inst):
    for p, x in sim.body_positions().items():
        if p.rsplit("/", 1)[-1] == inst:
            return np.asarray(x, dtype=float)
    raise KeyError(inst)


def leg_lengths(sim, leg="fl"):
    hip, knee, ball = (body(sim, f"{leg}:{i}") for i in ("thigh", "shank", "foot_ball"))
    return float(np.linalg.norm(knee - hip)), float(np.linalg.norm(ball - knee))


def ik(d, l1, l2):
    c = (l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2)
    phi = math.pi - math.acos(max(-1.0, min(1.0, c)))  # knee flexion
    a = math.acos(max(-1.0, min(1.0, (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d))))  # thigh off the vertical
    return -KNEE_BACK * a, KNEE_BACK * phi


def enable(sim, sg):
    for dev in sg:
        sim.set(f"/devices/{dev}", "enable", 1.0)


def command(sim, sg, q, kp=KP, kd=KD):
    for dev, s in sg.items():
        path, j = f"/devices/{dev}", dev.split(":")[1]
        for attr, val in (("p_des", s * q[j]), ("v_des", 0.0), ("kp", kp), ("kd", kd), ("tau_ff", 0.0)):
            sim.set(path, attr, val)


def tilt_deg(sim):
    w, x, y, z = sim.query("/world/deck_top", "quat")[0]
    zz = 1 - 2 * (x * x + y * y)  # body z axis . world z
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def sample(sim, sg, z_expected, dt):
    fault = max(float(sim.query(f"/devices/{d}", "fault")[0][0]) for d in sg)
    v = min(float(sim.query(f"/devices/power:{p}", "voltage")[0][0]) for p in ("pack_a", "pack_b"))
    z = float(sim.query("/world/deck_top", "pos")[0][2])
    return [sim.tick * dt, z - z_expected, tilt_deg(sim), fault, v]


def ramp(sim, sg, l1, l2, a, b, T, rows, z0=None, dt=0.001):
    """Hip-to-ball distance from a to b (fractions of l1 + l2) over T s at CMD_HZ; rows get one sample per command."""
    z0 = float(sim.query("/world/deck_top", "pos")[0][2]) + (l1 + l2) * (1 - a) if z0 is None else z0  # deck z of straight legs
    n, k = max(1, round(T * CMD_HZ)), round(1 / (CMD_HZ * dt))
    for i in range(n):
        f = a + (b - a) * (i + 1) / n
        qh, qk = ik(f * (l1 + l2), l1, l2)
        command(sim, sg, {"abad": 0.0, "hip": qh, "knee": qk})
        sim.step(k)
        rows.append(sample(sim, sg, z0 - (l1 + l2) * (1 - f), dt))
    return z0


def finish(sim, run_dir, rows, dt):
    r = Path(run_dir)
    with open(r / "trace.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(TRACE_COLS)
        w.writerows(rows)
    (r / "replay.naplog").write_bytes(sim.log())
    (r / "native.json").write_text(json.dumps(dict(tick=sim.tick, dt=dt, state_hash=f"{sim.state_hash():016x}", diverged=bool(sim.diverged))))
```

  (`ramp`'s `z0` is computed once by the caller's first ramp and passed on to the following ones so `h_err` measures the same
  reference; `test_stance_is_symmetric` ignores it.) `scene.py`: `def build(outdir): return str(dogsim.build(Path(outdir))[0])`.
  `stand.py`:

```python
"""Stand / squat sanity program for examples/dog/stand.scenario.toml ([interfaces.native]): straight legs -> squat -> stand on a
plain joint PD over the Qdd MIT-mode inputs. Writes trace.csv / replay.naplog / native.json for the runner."""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dogsim as D  # noqa: E402
import napl  # noqa: E402

PHASES = [(0.5, 1.0, 1.0), (1.5, 1.0, D.SQUAT), (1.0, D.SQUAT, D.SQUAT), (1.0, D.SQUAT, D.STAND), (1.0, D.STAND, D.STAND)]  # (s, from, to)


def main():
    model, run = Path(os.environ["NAPL_MODEL"]), Path(os.environ["NAPL_RUN_DIR"])
    rep = json.loads((model.parent / f"{model.stem}.report.json").read_text())
    sim = napl.Sim.load(model)
    sim.start_log()
    dt, sg = D.timestep(model), D.signs(rep)
    l1, l2 = D.leg_lengths(sim)
    D.enable(sim, sg)
    rows, z0 = [], None
    for T, a, b in PHASES:
        z0 = D.ramp(sim, sg, l1, l2, a, b, T, rows, z0, dt)
    D.finish(sim, run, rows, dt)


if __name__ == "__main__":
    main()
```

  `stand.scenario.toml`:

```toml
# napl program examples/dog/stand.scenario.toml --runs-dir <dir>
# SP3b sanity: the dog (assemblies/dog/dog.toml) on a flat floor, 12 Qdds under a plain joint PD: straight legs -> squat (0.65 of the
# leg length) -> stand (0.82). Proves the assembly loads and simulates on its Qdd devices; not a controller (SP4).
name = "dog-stand"

[model]
script = "scene.py"

[interfaces.native]

[program]
cmd = ["python", "{dir}/stand.py"]

[limits]
wall_s = 900
cpu_s = 900

[[monitors]]
name = "body_at_the_commanded_height"   # deck z vs the IK height at the end (PD sag under 12 kg allowed: 15 mm)
column = "h_err"
op = "final"
lo = -0.015
hi = 0.015

[[monitors]]
name = "stays_level"                     # body tilt never above 5 deg
column = "tilt"
op = "always"
hi = 5.0

[[monitors]]
name = "no_qdd_fault"
column = "fault"
op = "always"
hi = 0.0

[[monitors]]
name = "packs_above_cutoff"              # 8 x 3.0 V
column = "v_bat"
op = "always"
lo = 24.0
```

- [ ] **Step 4: run, expect PASS except `test_golden_is_recorded`:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_stand.py -n 12'`.
  If `test_stance_is_symmetric` fails on the knee direction only, flip `KNEE_BACK`; any other failure is a mirror / port bug in
  Tasks 2-7: fix it there, never loosen the tolerances.
- [ ] **Step 5: record the golden hash.** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/napl program examples/dog/stand.scenario.toml --runs-dir runs/dog-stand && cat runs/dog-stand/*/result.json'`;
  copy `sim.state_hash` into `GOLDEN` (Edit), rerun Step 4: all PASS.
- [ ] **Step 6: commit** `git add examples/dog tests/python/test_dog_stand.py && git commit -m "examples/dog: stand/squat sanity sim on the 12 Qdds (joint PD), symmetric-stance test, golden hash" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`

### Task 9: speed sweep harness on the full model + `docs/dog/speed-sweep.md` + full regression

**Files:**
- Create: `tools/dog_sweep.py`, `docs/dog/speed-sweep.md`, `docs/dog/speed-sweep.json`
- Modify: `assemblies/dog/README.md` (chosen point + Pareto table), `assemblies/dog/leg.toml` and `dog.toml` only if the chosen
  point differs from candidate 1 (then re-record `GOLDEN` in `tests/python/test_dog_stand.py`)
- Test: `tests/python/test_dog_sweep.py`

**Interfaces:**
- Consumes: `examples/dog/dogsim.py` (all of Task 8's Produces), `tools/dog_actuator_study.py` constants `THETA`, `DYN`,
  `PEAK_FACTOR`, `G`; part params `price`, `mass`, `gear_ratio`, `ke_si`, `mit_tau_ff_max`; Qdd attrs; battery attrs
  `voltage`, `current`; FootContact `force`.
- Produces: `dog_sweep.COLUMNS: dict[str, str]` (column -> `"measured" | "derived" | "bom" | "sp4"`), `dog_sweep.DEFAULT_POINTS`,
  `dog_sweep.write_variant(tmp, act, ratio, L, base="free") -> Path`, `dog_sweep.run_point(point, quick, gait) -> dict`,
  CLI `python tools/dog_sweep.py [--points act:ratio:L ...] [--quick] [--gait module:function] [--out DIR] [--jobs N]` writing
  `<out>/speed-sweep.json` (`{"columns", "points", "chosen", "generated"}`) and `<out>/speed-sweep.md`. `--gait` is the SP4
  slot: `function(sim, sg, report) -> mean forward speed (m/s)` run for 10 s on flat ground fills `v_top_trot`.

- [ ] **Step 1: failing test** `tests/python/test_dog_sweep.py`:

```python
"""Speed sweep harness (tools/dog_sweep.py): variant files load for every default point; the report labels every column and claims no
gait speed it did not measure (Review Focus 5); two runs give the same numbers."""

import json
import subprocess
import sys
from pathlib import Path

import napl
import pytest
from napl.design import assembly as A

REPO = Path(__file__).resolve().parents[2]
TOOL = REPO / "tools" / "dog_sweep.py"
sys.path.insert(0, str(REPO / "tools"))
import dog_sweep as S  # noqa: E402

needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.mark.parametrize("point", S.DEFAULT_POINTS)
def test_variant_files_load(tmp_path, point):
    act, ratio, L = point
    asm = A.load(S.write_variant(tmp_path, act, ratio, L))
    by = {i["id"]: i for i in asm.doc["instances"]}
    assert by["fl:knee_motor"]["params"]["drive_ratio"]["value"] == pytest.approx(8 * ratio)
    assert by["rr:thigh"]["part"] == "dog-thigh-bracket-l" + S.LSFX[L]
    probe = [d for d in asm.doc["devices"] if d["type"] == "qdd" and "map" not in d]
    assert bool(probe) == (act == "gim6010")  # the probe carries the GIM6010-8 electrics as provenance-tagged `set`


@needs_kernel
def test_report_labels_every_column_and_is_deterministic(tmp_path):  # Review Focus 5
    outs = []
    for k in range(2):
        out = tmp_path / f"s{k}"
        r = subprocess.run([sys.executable, str(TOOL), "--quick", "--points", "gim8108:1.5:0.22", "--out", str(out), "--jobs", "1"],
                           capture_output=True, text=True, timeout=3000)  # fmt: skip
        assert r.returncode == 0, r.stderr[-3000:]
        outs.append(json.loads((out / "speed-sweep.json").read_text()))
    data = outs[0]
    assert set(data["columns"]) == set(S.COLUMNS) and set(S.COLUMNS.values()) <= {"measured", "derived", "bom", "sp4"}
    assert S.COLUMNS["v_top_trot"] == "sp4" and all(p["v_top_trot"] is None for p in data["points"])
    assert all(S.COLUMNS[c] != "measured" for c in ("v_swing", "v_stance", "v_trot_bound"))
    md = (tmp_path / "s0" / "speed-sweep.md").read_text()
    assert "not measured (SP4)" in md and "derived (study formulas, sim-measured inputs)" in md
    assert outs[0]["points"] == outs[1]["points"]
```

- [ ] **Step 2: run, expect FAIL:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_sweep.py -n 12'`
- [ ] **Step 3: implement `tools/dog_sweep.py`** (key code; the measurement functions follow the docstring exactly):

```python
#!/usr/bin/env python3
"""SP3b speed design sweep on the FULL dog model (spec s5): actuator x knee drive ratio x leg length.

Measured on the built dog, per point (no gait controller needed):
  stall   saturating command at dq = 0 -> hip / knee stall torque as the Qdd delivers it from this bus (N m)
  swing   (base fixed 0.3 m up) fl hip minimum-time move over the study's 25 deg stride sweep, bang-bang at the Qdd's own envelope,
          rest of the leg held at the stand pose -> t_move (s), dq_peak (rad/s)
  push    standing at STAND, every hip / knee commanded fully extended with kp = kp_max for 0.3 s -> peak sum of foot forces / (m g)
  jump    from SQUAT the same command -> apex rise of the deck (m), peak / mean pack current (A), min pack voltage (V)
  drop    the standing dog lifted 0.25 m and released under the stand gains -> peak |tau_est| (N m), any fault, min deck clearance (m)
  hold    30 s standing -> max winding temperature rise (K), mean pack power (W)
Derived (tools/dog_actuator_study.py formulas, sim-measured inputs): v_swing = h sin(THETA) / t_move; v_stance = h cos(THETA) w0 (1 -
tau_hip / stall_hip) with w0 = V_pack / (ke_si N), tau_hip = DYN PEAK_FACTOR m g / 2 h sin(THETA); v_trot_bound = min; pack_margin
= 2 x 70 A / peak pack current. Not measured: a gait's top speed and cost of transport (SP4: --gait module:function).
GIM6010-8 is a probe: its electrics and mass in the GIM8108-8 seats (no GIM6010 printed parts); its cost swaps the 12 actuator prices.

    python tools/dog_sweep.py [--points act:ratio:L ...] [--quick] [--gait module:function] [--out docs/dog] [--jobs N]
"""

import argparse
import importlib
import json
import math
import multiprocessing as mp
import os
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "examples" / "dog"), str(REPO / "tools")]
import dog_actuator_study as STUDY  # noqa: E402
import dogsim as D  # noqa: E402
from napl.design import Library  # noqa: E402
from napl.design import assembly as A  # noqa: E402

DOG = REPO / "assemblies" / "dog"
ACTS = {"gim8108": "steadywin-gim8108-8", "gim6010": "steadywin-gim6010-8"}
LSFX = {0.20: "-200", 0.22: "", 0.25: "-250"}
DEFAULT_POINTS = [(a, r, L) for a in ACTS for r in (1.5, 2.0) for L in (0.20, 0.22, 0.25)]  # study candidates 1-3 + L neighbourhood
COLUMNS = {
    "actuator": "bom", "knee_ratio": "bom", "leg_length": "bom", "mass": "bom", "cost": "bom", "cost_estimated_share": "bom",
    "stall_hip": "measured", "stall_knee": "measured", "t_move": "measured", "dq_peak": "measured",
    "push_force_ratio": "measured", "jump_rise": "measured", "i_pack_peak": "measured", "i_pack_mean_jump": "measured",
    "v_pack_min": "measured", "drop_tau_peak": "measured", "drop_fault": "measured", "drop_clearance": "measured",
    "hold_temp_rise": "measured", "hold_power": "measured",
    "w0": "derived", "pack_margin": "derived", "v_swing": "derived", "v_stance": "derived", "v_trot_bound": "derived",
    "v_top_trot": "sp4", "cot": "sp4",
}  # fmt: skip


def _replace_once(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new)


def write_variant(tmp, act, ratio, L, base="free"):
    """dog.toml + its sub files into tmp for one point; -> the dog.toml path."""
    tmp = Path(tmp)
    tmp.mkdir(parents=True, exist_ok=True)
    s = LSFX[L]
    leg = (DOG / "leg.toml").read_text()
    leg = _replace_once(leg, 'part = "dog-thigh-bracket-l"', f'part = "dog-thigh-bracket-l{s}"')
    leg = _replace_once(leg, 'part = "dog-shank"', f'part = "dog-shank{s}"')
    leg = _replace_once(leg, 'part = "dog-knee-rod-220"', f'part = "dog-knee-rod-{round(L * 1000)}"')
    (tmp / "leg.toml").write_text(leg)
    if act != "gim8108":
        lasm = A.load(tmp / "leg.toml")
        P = Library(REPO / "parts").get(ACTS[act])
        assert P.value("gear_ratio") == 8  # drive_ratio = 8 x ratio stays right
        fields = lasm.raw["device_maps"]["qdd"]
        st = {f: {"value": P.value(pp), "unit": P.params[pp]["unit"], "fidelity": "derived",
                  "derivation": f"{P.id}.{pp}: sweep probe, {P.id} electrics in the GIM8108-8 seat (docs/dog/speed-sweep.md)"}
              for f, pp in fields.items()}  # fmt: skip
        for d in lasm.raw["devices"]:
            if d["type"] == "qdd":
                d.pop("map")
                d["set"] = {**st, **d.get("set", {})}
        for i in lasm.raw["instances"]:
            if i["part"] == "steadywin-gim8108-8":
                i.setdefault("params", {})["mass"] = {"value": P.value("mass"), "unit": "kg", "fidelity": "derived", "derivation": f"{P.id}.mass (sweep probe)"}
        (tmp / "leg.toml").write_text(A.dumps(lasm))
    for f in ("head.toml", "power.toml", "compute.toml", "dog.toml"):
        (tmp / f).write_text((DOG / f).read_text())
    dasm = A.load(tmp / "dog.toml")
    if base != "free":
        dasm.raw["assembly"]["base"] = base
        root = next(i for i in dasm.raw["instances"] if i.get("root"))
        root["pos"] = [root["pos"][0], root["pos"][1], root["pos"][2] + 0.3]
    for sub in dasm.raw["subassemblies"]:
        if sub["file"] == "leg.toml":
            sub.setdefault("params", {})["knee_drive_ratio"] = {"value": 8 * ratio, "unit": "1", "fidelity": "derived", "derivation": f"8 x knee linkage ratio {ratio} (sweep point)"}
    (tmp / "dog.toml").write_text(A.dumps(dasm))
    return tmp / "dog.toml"
```

  then `move_time(sim, dev, a, b, tau, dt)` (bang-bang to the midpoint, reverse until `dq` changes sign; RuntimeError after 2 s),
  `measure_stall`, `measure_push`, `measure_jump`, `measure_drop`, `measure_hold` (durations divided by 10 under `--quick`),
  `derive(m)` (the formulas of the docstring with `STUDY.THETA`, `STUDY.DYN`, `STUDY.PEAK_FACTOR`, `STUDY.G`), `run_point`
  (two builds per point: `write_variant(..., base="free")` and `base="fixed"` for the swing; `design.build` with
  `NAPL_ROUTE_CACHE` set to a shared dir), `cost` = `report["bom"]["total"]["cost"]` (probe: minus 12 x (GIM8108 price - GIM6010
  price)), the Pareto front over (`v_trot_bound` max, `cost` min, `mass` min) among points with `pack_margin >= 1.0`, no
  `drop_fault` and `push_force_ratio >= 2.0`, `chosen` = the front point with the highest `v_trot_bound` (ties: lower cost), and
  `write_md(out, data)`: a "How to read" block (each kind: `measured` = on the full model in this run; `derived (study formulas,
  sim-measured inputs)`; `bom` = build report; `sp4` = `not measured (SP4)`), the per-point table, the Pareto table, the chosen
  point, the honest gap to the 3.5 m/s criterion (spec s7) and to the 4.5 k USD target, the probe caveat, and the command line.
  Points run in `mp.Pool(args.jobs or int(os.environ.get("NAPL_THREADS", "1")))`, sorted by point for determinism.
- [ ] **Step 4: run the tests, expect PASS:** `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run '.venv/bin/pytest tests/python/test_dog_sweep.py -n 12'`
- [ ] **Step 5: full sweep on hfbox (whole box, background):**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh bg --slots 2 dogsweep '.venv/bin/python tools/dog_sweep.py --out build/dog-sweep --jobs 16'`;
  follow with `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh log dogsweep 40`; when done
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh pull build/dog-sweep`, Read `remote-out/build/dog-sweep/speed-sweep.md`
  and `.json`, Write them to `docs/dog/speed-sweep.md` / `docs/dog/speed-sweep.json`. Add the chosen point and the Pareto table to
  `assemblies/dog/README.md`. If the chosen point is not candidate 1 (GIM8108-8, 1.5, 0.22): set it in `leg.toml` (part ids and
  `drive_ratio`; the CAD constant `KNEE_RATIO` and a regenerated crank / shank pair if the ratio changed), rerun Tasks 4-8 tests and
  re-record `GOLDEN` (Task 8 Step 5).
- [ ] **Step 6: full regression, expect all green and every existing golden unchanged:**
  `NAPL_REMOTE_DIR=/workspace/wt/napl-dog tools/remote.sh run --slots 2 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh && ctest --preset dev --output-on-failure && .venv/bin/pytest tests/python -n 12 && .venv/bin/python tools/check_parts.py && .venv/bin/python tools/check_licenses.py'`
- [ ] **Step 7: commit** `git add tools/dog_sweep.py tests/python/test_dog_sweep.py docs/dog/speed-sweep.md docs/dog/speed-sweep.json assemblies/dog && git commit -m "dog: speed sweep on the full model (stall, swing, push, jump, drop, hold; study bounds from measured inputs; SP4 gait slot)" -m "Claude-Session: https://claude.ai/code/session_01JZzeM8P1116bAsZtZ2j6h5"`
