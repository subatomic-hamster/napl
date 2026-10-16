# SP2: ground kernel (Qdd actuator, FootContact, Mid-360 pattern, small-scale terrain) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the kernel can simulate a legged ground robot honestly: QDD actuators in MIT mode over CAN with a real
torque-speed envelope and thermal limits, foot-contact sensing, a Livox Mid-360-like lidar, and 100 m rocky / sloped /
slippery / ruined terrain with per-patch friction and ground truth.

**Architecture:** devices follow the existing pattern: table in `schema/devices.fbs`, C++ in `src/mujoco/devices/`,
builder in `python/napl/design/devices.py` (+ `auto_devices.py` defaults from part params), assembly `[[devices]]` type
names. Terrain kinds extend `python/napl/terrain.py` with a small-scale tiled generator.

**Tech Stack:** C++20, MuJoCo 3.x, FlatBuffers, nanobind, numpy, pytest, gtest.

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` (section 4; risks 1-2).

## Global Constraints

- No physics defaults: a missing physical parameter fails the load (as `Servo`). Parameters come from part files with
  provenance.
- Determinism: same seed → same state hash; device noise via the device's own seeded RNG; replay equal hash.
- Existing tests and goldens unchanged. No GPL. Edit only with Edit/Write tools. Compute on hfbox via `tools/remote.sh`.
- Document every model in `docs/models.md` (assumptions, what is not modelled) as earlier devices are.

## Review Focus

1. Command torque above the voltage-limited envelope at speed — the device must saturate to the envelope, never deliver the
   commanded torque. Test: Task 1 envelope sweep.
2. CAN command loss (no command for > timeout) — the driver's safe behaviour (damping mode or hold, per the part's firmware
   docs) must engage. Test in Task 1.
3. A foot touching two geoms at once (rock + hfield) — the contact device sums forces of all contacts of the foot geom.
   Test in Task 2.
4. Lidar rays starting inside the robot's own geoms — self-hits excluded by group mask (as Rotor's `ray_groups`). Test in
   Task 3.
5. Terrain seed reproducibility across processes and patch boundaries (no seams: heights continuous across tile edges).
   Test in Task 4.

---

### Task 1: `Qdd` device (MIT mode, averaged FOC, CAN latency, thermal)

**Files:** Modify `schema/devices.fbs` (table `Qdd`, add to the device union), Create `src/mujoco/devices/qdd.cpp` (+ header
entry where Servo is declared), modify `src/mujoco/devices/CMakeLists.txt` or the devices source list,
`python/napl/design/devices.py` (builder `qdd`), `python/napl/design/assembly.py` (`DEVICE_TYPES` += `qdd`),
`python/napl/design/auto_devices.py` (map part params of `actuator.bldc.qdd` parts), `docs/models.md`.
Test: `tests/cpp/test_qdd.cpp`, `tests/python/test_qdd.py`.

**Interfaces:**
- Config: `name, parts, node (electrical), joint, motor: MotorParams, gear_ratio, efficiency, encoder_cpr, cmd_hz, latency_s,
  i_limit, kp_max, kd_max, tau_ff_max, timeout_s, timeout_mode (0 hold-damping kd_timeout, 1 open), kd_timeout,
  thermal (r_th, c_th, t_max, derate_start), seed`.
- Runtime set attrs: `p_des, v_des, kp, kd, tau_ff, enable`; read attrs: `q, dq, tau_est, i_q, temp, fault, bus_i`.
- Model (each tick): command sampled at `cmd_hz`, applied after `latency_s` (integer ticks, checked at bind);
  `tau_cmd = kp (p_des - q_enc) + kd (v_des - dq_enc) + tau_ff`; `i_q* = tau_cmd / (Kt N eta)` clamped to `i_limit` and to
  the voltage limit `|R i_q + Ke N w| <= V_bus / sqrt(3)`-style averaged bound (state the exact form in models.md);
  output torque `Kt N eta_dir i_q`; reflected inertia as armature `J_rotor N^2`; copper loss heats the winding (first-order
  thermal), derating above `derate_start`, fault above `t_max`; bus current `(V_phase i_q + losses) / V_bus` into the
  electrical node.
- [ ] Step 1: failing tests: (a) stall torque = `min(i_limit, …) Kt N eta` within 1%; (b) torque-speed sweep at 24 V vs
  the analytic envelope (and vs the part's published no-load speed within 5%); (c) step response of kp/kd hold; (d)
  latency: response delayed by exactly `latency_s`; (e) timeout → damping; (f) thermal: sustained over-rated torque rises
  temp and derates; (g) determinism + replay hash.
- [ ] Step 2-4: implement; run `ctest --preset dev -R qdd` and `pytest tests/python/test_qdd.py`.
- [ ] Step 5: commit `devices: Qdd (MIT-mode QDD actuator over CAN)`.

### Task 2: `FootContact` device

**Files:** `schema/devices.fbs` (`FootContact`), `src/mujoco/devices/contact.cpp`, builder in `devices.py`,
`DEVICE_TYPES` += `foot_contact`, models.md. Tests `tests/cpp/test_foot_contact.cpp`, `tests/python/test_foot_contact.py`.

**Interfaces:** config `name, parts, geom, mode (0 force, 1 binary), f_max, noise_std, hysteresis_on, hysteresis_off,
debounce_s, rate_hz, seed`; read attrs `force` (N, normal, summed over all contacts of the geom), `contact` (0/1).
- [ ] Failing tests: force equals sum of MuJoCo contact normal forces on the geom (sphere resting on box + hfield,
  Review Focus 3); saturation at f_max; binary hysteresis + debounce timing; determinism. Implement. Commit
  `devices: FootContact`.

### Task 3: Mid-360 scan pattern

**Files:** `schema/devices.fbs` (`Rangefinder.pattern` enum value `rosette` + params `fov_v_min, fov_v_max, rays_per_frame,
pattern_seed`), `src/mujoco/devices/rangefinder.cpp`, builder, part file for the Mid-360 comes from SP3a (use a test part
in `tests/data/` meanwhile), models.md (note: the real Mid-360 pattern is proprietary; ours is a documented non-repetitive
rosette approximation with the published FoV, point rate and range).

- [ ] Failing tests: all rays within the FoV; coverage of the FoV after 1 s ≥ 95% of 1°x1° bins (non-repetitive); self-hit
  mask (Review Focus 4); cost benchmark printed (rays/s on a 2001² hfield) and recorded in models.md; determinism.
- [ ] Implement. Commit `devices: Rangefinder rosette pattern (Mid-360-like)`.

### Task 4: small-scale terrain kinds + materials + ground truth

**Files:** `python/napl/terrain.py` (new `field(kind, size=100.0, res=0.05, seed, **p)` for kinds `rubble`, `slopes`,
`scree`, `ruins`; tiling into patches; `add_field(spec, field, ...)` emitting one hfield geom per patch with its material
friction / solref; `height(x, y)` ground truth incl. boxes; `materials` table rock / gravel / grass / wet_rock / mud with
sourced friction values), `docs/models.md`. Tests `tests/python/test_terrain_field.py`.

**Interfaces:** `Field` dataclass: `kind, size, res, seed, heights (np.ndarray), patches: list[Patch(x0,y0,x1,y1,material)],
boxes: list[Box(pos, size, rpy, material)], height(x, y) -> float, material_at(x, y) -> str`; `add_field(spec_or_xml,
field) -> list[geom names]`.
- [ ] Failing tests: determinism across processes (hash of heights); no seams at patch edges (heights continuous;
  Review Focus 5); `rubble` rock heights in 5-20 cm; `slopes` contains 15-30° slopes and 10-18 cm steps; `scree` has
  low-friction patches; `ruins` wall maze with gaps ≥ 0.6 m; MuJoCo contact friction at a probe point equals the patch
  material; ground-truth `height` equals a ray cast within 1 mm on 1000 random points.
- [ ] Contact validation (risk 2): a 12 kg box-with-4-spheres "dummy dog" dropped and dragged on each kind at dt 1 ms and
  2 ms: no tunnelling, no jitter above 1 mm at rest, solref/solimp choice documented.
- [ ] Commit `terrain: small-scale field kinds with materials and ground truth`.

### Task 5: sim-speed budget (risk 1)

- [ ] Benchmark script `tools/bench_ground.py`: dummy dog (12 Qdd joints in hold mode, 4 FootContact, rosette lidar at 10 Hz
  with N rays) on each kind, report sim-seconds per wall-second for N in {5k, 10k, 20k}; table into `docs/models.md`.
  Commit `bench: ground sim budget`.
