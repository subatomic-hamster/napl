# Quadruped recon dog + napl sub-assemblies — design

Date: 2026-10-16. Status: approved in conversation (sections 1-2 explicitly; the user then said "continue with everything as
planned, no need to wait for approval, I will interrupt if something is amiss").

## 1. Intent

**User's words (2026-10-16):** design a quadruped robot "dog" for recon missions, optimized for speed and for low cost with
consumer hardware; larger than the Skydio is fine. Multiple individually complex, sensor-loaded parts are built separately
and connected to the main chassis. First large ground simulation (napl so far: air and water). Introduce "parts" in napl:
make smaller components and use them inside larger ones in a workspace, like CAD. Move the controls panel out of the main
area into a toggleable controls icon in the top bar. End product: a quadruped with sensors for high-accuracy terrain
mapping, high stability and speed on rocky terrain, full autonomy and path creation to map a terrain fully, and a video:
high-level construction breakdown, brief overview of the complex programming, simulation/demo.

**Decisions taken with the user:**

| Topic            | Decision                                                                                                          |
| ---------------- | ----------------------------------------------------------------------------------------------------------------- |
| Size / budget    | ~12 kg Go2 / Mini-Cheetah class, 12 low-cost QDD BLDC actuators, BOM target ~$3-4.5k                              |
| Locomotion       | model-based: convex MPC on ground reaction forces + whole-body control + terrain-aware footstep planning          |
| Sensors          | 360-degree 3D lidar (Livox Mid-360 class) + forward depth camera + IMU + joint encoders + foot contact + RTK GNSS |
| Terrain families | rocky field + boulders; slopes + stairs/ledges; loose/slippery ground; ruins/urban rubble                         |
| Sub-assemblies   | linked, CAD-component style, per-instance overrides of exported params; flatten-on-load (approach A)              |
| Targets          | "ambitious" (section 7)                                                                                           |

**Assumptions (not stated by the user, kept from earlier rounds):** sim-first, real purchasable parts with provenance,
printable custom parts, pin-level wiring, wires never through parts, golden hashes, honest MC numbers, no media in git,
no GPL deps. Recon only: no weapon or payload-delivery features.

## 2. Decomposition

| #   | Sub-project                                                                                                                                         | Depends on                                        | Output                              |
| --- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------- | ----------------------------------- |
| SP1 | Platform: sub-assemblies (schema, flatten, mirror, BOM roll-up, viewer workspace, backend ops) + controls icon + stale DRC message fix              | -                                                 | napl feature, tests, docs           |
| SP2 | Ground kernel: QDD actuator device (MIT mode), foot-contact device, Mid-360 scan pattern, small-scale terrain kinds with per-patch friction         | -                                                 | devices, terrain kinds, tests       |
| SP3 | Dog hardware: parts (actuators, lidar, power, printed brackets), leg / head / power-module sub-assemblies, chassis, wiring, BOM, speed design sweep | SP1, SP2 parts schema                             | `assemblies/dog/*.toml`, parts, CAD |
| SP4 | Locomotion: estimator, gait scheduler, convex MPC, WBC, swing/footstep planning, slip recovery                                                      | SP2, SP3                                          | `examples/dog/` controller, MC      |
| SP5 | Autonomy: lidar-inertial-leg-RTK localization, elevation map, traversability, frontier/coverage planner, mapping-accuracy metrics                   | SP2 (perception can start on a kinematic carrier) | `examples/dog/` autonomy, MC        |
| SP6 | Integration missions (4 terrain families) + MC + E2E video                                                                                          | SP3-SP5                                           | scenarios, goldens, video           |

SP1 and SP2 run in parallel; SP3 parts research runs alongside them; SP5 perception starts on a kinematic sensor carrier
(a floating base driven along recorded trajectories) so it does not wait for walking.

## 3. SP1 — sub-assemblies (approach A: flatten on load)

### 3.1 Schema

Child (any assembly file) gains an optional `[exports]` table:

```toml
[exports]
ports      = { hip_mount = "hip_bracket.chassis_face" }        # "<inst>.<port>"
connectors = { can = "hip_motor.CAN", power = "hip_motor.XT30" } # "<inst>.<connector>"
params     = { shank_len = "shank.length", cable_len = "harness.length" }  # "<inst>.<param>", overridable
```

Parent gains `[[subassemblies]]` (keys: `id, file, pos, rpy, mirror, params, group, color`):

```toml
[[subassemblies]]
id = "fl"
file = "leg.toml"          # relative to the parent file
mirror = "xz"              # optional reflection plane (parent root frame): "xz" | "yz" | "xy"
params = { cable_len = { value = 0.32, unit = "m", fidelity = "derived", derivation = "..." } }
```

References from the parent into a child go through exports only: `"fl:hip_mount"` (mates), `"fl:power"` (wires). A
reference that reaches inside (`"fl:hip_motor.mount"`) is an error naming the export to add. A sub-assembly is attached by
a parent mate on an exported port, or welded at `pos` / `rpy` like a placed instance.

### 3.2 Flatten

`assembly.load` (and `loads`) expands sub-assemblies before `validate`:

- instances, mates, wires, devices, device_maps and targets of the child are copied with ids prefixed `"<id>:"`; nested
  sub-assemblies give `"fl:knee:motor"`, any depth; a file that includes itself (directly or not) is an error listing the
  cycle;
- every internal reference (`"<inst>.<x>"` strings, device `parts` / `joint` / body names) is rewritten with the prefix;
- the child's `root` flag is dropped;
- overrides apply to exported params only (unknown name = error listing the exports); override values follow the numbers
  rule;
- every flattened item records `origin = {sub: "fl", file: "leg.toml", id: "hip_motor"}` (kept out of the saved TOML; the
  saved parent keeps only its `[[subassemblies]]`), so findings read "fl:hip_motor (from leg.toml)".

Everything downstream (`compute`, DRC, devices, MJCF, wiring, replay, state hashes) sees a flat assembly and is unchanged.
MJCF names keep the colon; prim paths become `/world/fl:hip_motor`.

### 3.3 Mirror

Reflection M across the plane. For each child part placed at T = (R, p) in the sub-assembly frame, the mirrored placement is
(M R M, M p): a proper rotation, but the part's geometry in its own frame must be the mirror image. Each part therefore
declares one of, in a new top-level `[mirror]` table of the part file:

- `symmetry = "<plane>"` (local plane "xy" | "yz" | "xz"): the part is symmetric about that local plane, so the mirrored copy is
  the same part (motors, tubes, spheres, symmetric plates);
- `opposite = "<part-id>"`: the opposite-hand part (printed left/right brackets; napl's CAD pipeline generates the mirrored
  mesh of its own printed parts and the pair cross-references).

A part with neither is a DRC error when mirrored (never silent). Port frames and joint axes transform with the part; each
mirrored hinge's sign relative to the unmirrored design is written to the build report (`report["mirror_signs"]`), so a
program can command "+abduction = outward" on both sides. Wires inside a mirrored child keep their topology.

### 3.4 BOM / cost

`price` becomes a regular part param (`{value, unit = "USD", fidelity, src}`; other currencies are converted at a dated
rate recorded as a `derivation`). The existing ad-hoc `price_*` params are migrated. The build report gains a BOM:
per part line (count, unit price, fidelity), per sub-assembly subtotal (mass + cost), totals, and the share of the cost that
is `estimated`. Missing prices are listed, never assumed zero.

### 3.5 Viewer workspace

- Document stack: the viewer holds open designs; the top-bar title becomes a breadcrumb (`dog > fl (leg.toml)`), a crumb
  click returns up; each document keeps its own undo stack; saving a child rebuilds every open parent using it.
- **New part** (BUILD ribbon): opens an empty design. Inspector **Export as...** on a selected port / connector / param;
  an EXPORTS section in the browser pane lists and removes exports.
- LIBRARY gains **MY PARTS**: workspace assemblies with `[exports]`, with thumbnail, exports, mass and cost; drag-in snaps
  the exported mount port (reusing ghost placement) and writes the `[[subassemblies]]` entry + mate. **Mirror** in the ribbon
  makes the opposite-hand copy.
- BUILD tree: a sub-assembly is one collapsible node (`fl  leg.toml  mirrored`), children greyed. Selecting a child item shows
  "from leg.toml" in the inspector; internals are read-only in the parent except exported params (edited as overrides);
  **Open part** / double-click switches to the child document. (Edit-in-context with the faded parent: later, not v1.)
- WIRING: a sub-assembly is one block with only its exported connectors (hierarchical schematic); internal wiring is edited
  in the child.
- Status bar: DRC, mass and cost roll up; the mass/BOM panel breaks them down per sub-assembly.
- Every action is a Backend op usable by MCP, agents and recorders: `sub.new`, `sub.export`, `sub.unexport`, `sub.insert`,
  `sub.mirror`, `sub.override`, `sub.open`, `sub.close`.

### 3.6 Controls icon

Remove the BUILD viewport overlay (`controls_legend`, viewer.cpp) and the CONTROLS frame inside the WIRING legend (the symbol
legend stays). Add a keyboard icon next to undo / redo / save in `top_bar()` that toggles a popover anchored under it,
listing the controls of the current page (BUILD, WIRING, SIMULATE camera, PROGRAM shortcuts). Closed by default.
Also fix the known bug: the status-bar DRC message stays stale while the DRC chip shows the right count.

### 3.7 SP1 tests

- flatten: leg + 4-leg parent → instance/mate/wire counts, prefixed ids, origins; nested 3 levels; cycle error; reach-inside
  error; unknown override error; saved parent round-trips without flattened items;
- equivalence: an existing assembly split into a child + parent builds to the **same MJCF and state hash** as the original;
- mirror: mirrored leg geometry equals the hand-built mirror (body poses within 1e-12), DRC error for an unmirrorable part,
  `mirror_signs` correct;
- BOM: totals = sum of lines; missing price listed;
- viewer: ops tests through Backend (insert, export, override, open/close/breadcrumb, undo per document); gpu smoke frames of
  the breadcrumb, MY PARTS, the collapsed tree node, the hierarchical WIRING block, and the controls popover (and its absence
  from the viewport);
- every existing test and golden hash unchanged.

## 4. SP2 — ground kernel

- **`Qdd` device** (C++, devices.fbs): integrated-driver QDD actuator in MIT mode: command (p_des, v_des, kp, kd, tau_ff)
  received over a CAN bus at `cmd_hz`, with configurable latency; averaged FOC: q-axis current from the command torque,
  `i_limit`, voltage saturation from the bus voltage and back-EMF (gives the real torque-speed envelope), gearbox ratio,
  efficiency, reflected rotor inertia as armature, output encoder quantization, winding thermal model with derating; bus
  current into the existing electrical network (battery sag). Parameters only from part files (no defaults for physics).
- **`FootContact` device**: from MuJoCo contact forces on the foot geom; output either force (FSR-like: noise, saturation,
  hysteresis) or a debounced binary; per-foot.
- **Mid-360 scan pattern** for `Rangefinder`: non-repetitive rosette over 360 x (-7..52) degrees, N rays per frame advancing
  each frame, 10 Hz, range noise and dropout from the part; the ray budget per frame is a parameter (cost measured and
  documented).
- **Small-scale terrain** (`napl.terrain`): kinds `rubble` (5-20 cm rubble + boulders), `slopes` (15-30 degrees + 10-18 cm
  steps and ledges), `scree` (loose/low-friction patches), `ruins` (collapsed-wall maze with gaps); ~100 x 100 m at ~5 cm
  cells; materials (rock, gravel, grass, wet rock, mud) with friction/solref per patch (the terrain is tiled into patches,
  each its own hfield geom with its material); ground-truth height function for map-error metrics.
- Tests: Qdd torque-speed envelope vs the part's datasheet curve, MIT-mode step response, latency, thermal derating; contact
  device vs MuJoCo contact force; rosette coverage; terrain determinism (seed → identical hfields) and material assignment.

## 5. SP3 — dog hardware

- **Parts** (real, priced, with sources): 12 QDD actuators (candidates: Steadywin GIM8108-8 class with integrated CAN driver,
  CubeMars AK70-9 for comparison; final choice by the speed sweep), Livox Mid-360, RealSense D435i (exists), ZED-F9P RTK +
  antenna (exists), Jetson Orin Nano (exists), CAN interface(s), 6S-8S Li-ion pack + BMS, DC-DC, e-stop, power distribution,
  connectors, fasteners, carbon tubes, bearings, belts/linkages for the knee, rubber feet. Printed parts via the CAD pipeline
  (brackets, hip housings, head mount, foot cores) with mirrored pairs.
- **Sub-assemblies:** `leg.toml` (abduction + hip + knee actuators, thigh, shank with knee drive, foot with contact sensor,
  harness; exports the hip mount, CAN, power, and length/cable params), used 4x (FR/RL mirrored as needed);
  `head.toml` (lidar, depth camera, mount, cabling); `power.toml` (battery, BMS, DC-DC, e-stop, distribution);
  `compute.toml` (Jetson, CAN interfaces, IMU, RTK receiver); `dog.toml` = chassis + those sub-assemblies.
- **Speed design sweep** (napl sweep): actuator choice, knee reduction, thigh/shank length, bus voltage → simulated top
  speed (trot and flying trot), cost of transport, cost; the chosen point and the Pareto table go in the README.
- Pin-level wiring (CAN daisy chains per leg, power per leg), DRC 0 errors, mass ~12 kg, BOM report.

## 6. SP4 / SP5 — software

**Rates:** physics 1 kHz; joint commands (WBC) 500 Hz over CAN; MPC 30-50 Hz; mapping 10 Hz; global planning 1-2 Hz.

**SP4 locomotion (examples/dog):**

- estimator: contact-aided EKF (IMU + leg kinematics + foot contact) at 500 Hz, RTK and lidar pose corrections at 10 Hz;
- gait scheduler: trot, flying trot (top speed), walk/crawl (steps, ledges); contact-event handling (early/late touchdown);
- convex MPC (single rigid body, friction cones, 10-16 step horizon) with OSQP (Apache-2.0);
- whole-body control: stance forces + swing-foot Cartesian tracking → joint torques and MIT-mode kp/kd/tau_ff;
- footstep planner: Raibert + capture-point nominal, adjusted to the elevation map (foothold score from slope, roughness,
  edge distance); body posture follows the terrain slope; step-height adaptation;
- slip detection (foot velocity in stance) and recovery (lower friction estimate in the MPC, widen stance, slow down);
- controller uses its own model (leg kinematics/inertia from the design report), never MuJoCo internals.

**SP5 autonomy (examples/dog):**

- localization: EKF fusion of leg-inertial odometry, RTK (10 Hz), and lidar scan-to-map registration (point-to-plane ICP)
  for degraded-GNSS areas (ruins: multipath/blocked sky);
- elevation map (5 cm cells, height + variance, Fankhauser-style fusion), traversability (slope, step height, roughness);
- coverage autonomy: frontier exploration on the elevation map with A* on traversability cost, a coverage goal "every
  traversable cell observed with variance below threshold", return-to-start; replanning on new obstacles;
- metrics against the terrain's ground truth: height RMS, coverage %, time, distance, falls.

## 7. Success criteria (MC over seeds; reported honestly if missed)

- flat top speed >= 3.5 m/s;
- rocky field mean speed >= 1.5 m/s;
- 25-degree slopes and 15 cm steps traversed;
- > = 90% no-fall per terrain family (MC n per family stated, with a 95% lower bound);
- full autonomous coverage of a 100 x 100 m rocky field in <= 20 min with map height RMS <= 5 cm and >= 95% of the
  traversable area mapped;
- ruins: the same criteria (<= 20 min, RMS <= 5 cm, >= 95% mapped) on a 50 x 50 m ruins site (~100 rooms). User decision
  2026-10-17: 100 x 100 m ruins (400 rooms) needs >= 1995 m of path (~1330 s at 1.5 m/s), so 95% in 20 min is unreachable;
- BOM ~$3-4.5k with the estimated share stated.

## 8. Video (SP6)

One E2E video like the Skydio landing video: high-level construction (New part → build the leg → export → insert 4x with
mirror → head, power, compute → chassis → wiring overview at pin level), brief programming overview (PROGRAM page: the
estimator / MPC / WBC / mapping / planner modules, a bench test of one leg), and the simulation demo (each terrain family
live or tick-exact replay, top-speed run, full coverage mission with the map building, MC summary card). Orbit camera,
CAD-like UI, honest captions. Recorder `tools/record_dog.py`, non-GPL encoder, no media committed.

## 9. Risks

1. **Sim speed:** 20-minute missions at 1 kHz with Python control at 500 Hz, plus lidar rays on large hfields. Mitigations:
   measure early (SP2); move hot loops to numpy/C++ devices; cap the ray budget; MC n chosen to fit the CPU budget.
2. **Contact on rocky hfields:** MuJoCo hfield contact with small feet can be stiff or jittery. Choose solref/solimp and
   timestep from a contact-validation test in SP2.
3. **3.5 m/s with low-cost actuators:** a voltage/torque-speed limit may cap speed. The SP3 sweep decides early; the shortfall
   is reported honestly.
4. **Mirror correctness:** a wrong-hand part is invisible in tests that only count items. Hence the pose-equality test and
   the unmirrorable-part DRC.
5. **Sub-assembly refactor ripple:** flatten-on-load keeps downstream unchanged; the split-equivalence test (same state
   hash) guards it.
