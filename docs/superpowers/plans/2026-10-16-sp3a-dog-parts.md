# SP3a: dog parts library + speed pre-study Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** every purchasable part the dog needs exists in `parts/` with sourced params and a USD `price`, the custom printed
parts exist as CAD with mirrored pairs, and an analytic pre-study picks the actuator / knee ratio / leg length / bus voltage
candidates for the full sim sweep in SP3b.

**Architecture:** part TOML files following `parts/README.md` (params with provenance, ports, connectors, visual,
sources). Printed parts through the existing CAD pipeline (see `cad/` and how cuas-lite printed parts are generated).
Pre-study is a script + markdown table (`docs/dog/actuator-study.md`).

**Tech Stack:** TOML, Python CAD pipeline already in repo, web sources (datasheets, vendor pages).

**Spec:** `docs/superpowers/specs/2026-10-16-quadruped-design.md` (sections 5, 7).

## Global Constraints

- Real parts only; every number sourced (`datasheet` / `literature` / `measured`) or marked `estimated` with rationale.
- `price` param: `{value, unit = "USD", fidelity, src}` with the date the price was seen in the source note; prices from
  vendor pages preferred over marketplaces; state the region.
- Part frames + ports + pin-level connectors (full headers) per `parts/README.md`; the `[mirror]` table (`symmetry` or
  `opposite`) on every part used in a leg (schema defined by SP1-core Task 5; until it merges, add it and keep
  `check_parts` passing by adding the key to its allowed set in the same commit if needed — coordinate via the
  orchestrator).
- `tools/check_parts.py` 0 problems; `tools/check_licenses.py` 0 problems. No media committed. Edit only with Edit/Write.

## Review Focus

1. Integrated-driver QDDs publish torque at the motor or output side inconsistently — every torque/speed/current is labelled
   output/motor side and AC/DC as the CubeMars part does. Check: a pytest that the Qdd builder inputs (Kt, N, eta, R, Ke)
   reproduce the published peak torque and no-load speed within 10%.
2. CAN connector pinouts differ between vendors (GH1.25 vs XT30 2+2) — pin-level connectors from the vendor drawing, not
   assumed.
3. Battery: Li-ion vs LiPo current limits at the dog's peak draw (12 joints x peak) — the pack's continuous discharge must
   cover the estimated peak; the study states the margin.
4. Lidar mount: Mid-360 minimum range and blind cone — the head placement must see the ground ahead of the front feet; the
   study gives the geometry.
5. Price drift — prices dated; BOM states the estimated share.

---

### Task 1: actuators

- [ ] Research and add 2-4 low-cost integrated-driver QDDs (Steadywin GIM8108-8 class; also GIM6010-8 class; MyActuator
  RMD-X8 class; CubeMars AK70-9 exists) with motor constants, gear ratio, rotor inertia, peak/rated torque and current,
  no-load speed at stated voltage, mass, dimensions, CAN protocol notes (MIT-mode support), connectors, price.
- [ ] Test `tests/python/test_dog_parts.py::test_qdd_constants_reproduce_datasheet` (Review Focus 1).
- [ ] Commit `parts: low-cost QDD actuators for the dog`.

### Task 2: sensors, compute, power, interconnect

- [ ] Livox Mid-360 (FoV, point rate, range, noise, min range, mass, power, connector M12/aviation pinout, price); verify
  existing RealSense D435i, ZED-F9P, SparkFun RTK, Jetson Orin Nano parts have `price` and the connectors the dog needs.
- [ ] USB-CAN / CAN-FD interfaces (e.g. a CANable-class adapter or a Jetson-carrier CAN transceiver), 6S-8S Li-ion pack
  options + BMS, DC-DC (bus → 19 V Jetson, 12 V lidar, 5 V), e-stop switch, distribution board, XT30/XT60 harnessing, rubber
  feet, bearings, timing belts / pulleys or linkage rods for the knee drive, carbon tubes, fasteners.
- [ ] Commit `parts: dog sensors, compute, power, interconnect`.

### Task 3: printed parts (CAD) with mirrored pairs

- [ ] Hip housing (abduction motor to chassis), thigh bracket, knee drive housing, shank, foot core, lidar/camera head
  mount, battery tray, chassis side plates — parametric where the leg study needs it (thigh/shank length). Mirrored pairs
  `-l` / `-r` with `[mirror] opposite`. Mass from mesh volume x material density x infill (derived).
- [ ] Commit `cad+parts: dog printed parts`.

### Task 4: speed pre-study (analytic)

- [ ] `tools/dog_actuator_study.py`: for each actuator x knee ratio {1, 1.5, 2} x leg length {0.20, 0.22, 0.25} m x bus
  {24, 33.6} V: peak foot force, max joint speed → foot speed bound, trot top-speed estimate (stride-frequency bound and
  torque bound, cite the model: e.g. Mini Cheetah literature), mass, cost. Output `docs/dog/actuator-study.md` with the Pareto
  table and 2-3 candidates for the SP3b full-sim sweep. Commit `docs: dog actuator pre-study`.
