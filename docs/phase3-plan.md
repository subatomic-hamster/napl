# Phase 3: a design system, not two fixed robots

Goal (user, 2026-10-07): a much larger real-parts library; builds that can be altered freely,
with error checks, intelligent part snapping and robot programming. Phase 2 milestones (N2–N7)
still apply; phase 3 generalises them so the X500 V2 and SO-101 become two example designs.

## Pillars

1. **Library** — hundreds of sourced parts (parts/README.md rules), every connecting part with
   `[[ports]]`. Categories: multirotor propulsion (motors with thrust tables, props, ESCs,
   batteries), actuators (bus servos, hobby servos, steppers + drivers, DC gearmotors),
   electronics (flight controllers, companion computers, sensors, BECs, PDBs), connectors and
   wire, structure (tubes, plates, extrusions, bearings, standoffs, fasteners), materials.
2. **Assemblies as data** — `assemblies/<name>.toml`: instances of parts, mates between ports
   (snapping solves transforms), fasteners per mate, wires between power/signal ports, device
   bindings (which motor drives which joint, which ESC feeds which motor), joints for moving
   mates. Builder turns it into MJCF + device config.
3. **Snapping** — given two instances and port names (or just two instances), find compatible
   ports, solve the transform, pick fasteners that fit (thread, length >= clamp stack +
   engagement), report what was chosen. Suggest compatible parts for an open port.
4. **Design-rule checks (DRC/ERC)**, all with a code, severity, message and suggested fix:
   - mechanical: unmated required ports, fastener thread/length mismatch, geometry
     interference at build (contacts at rest), prop disc overlap/clearance, CG vs thrust centre
   - electrical: connector type/gender mismatch, voltage outside a port's range, current above
     connector/wire/ESC rating at worst case, PSU/battery C-rating capacity, floating nets
   - performance: thrust-to-weight, hover throttle, estimated flight time, servo torque margin
     at worst-case pose, payload limit
5. **Editing API** — Python `napl.design` (load, add/remove/swap part, mate, wire, set param,
   check, build, simulate) and matching MCP tools so agents edit designs; every edit re-runs DRC.
6. **Programming** — the robots run real or real-protocol code:
   - drone: real PX4 SITL; missions/offboard via MAVLink from user scripts
   - arm: a virtual serial port (pty) speaking the Feetech STS protocol, so unmodified host code
     (Feetech SDK / LeRobot-style bus code) drives the simulated servos
   - user scripts run as separate sandboxed processes against the bus: `napl program <scenario.toml>`
     (python/napl/program.py; examples/so101, examples/x500). The program is USER CODE run from the CLI only;
     there is deliberately no MCP tool for it (v1 rule: agents author data - designs, TestSpecs - never code).
     Limits are rlimits + a process-group kill, not a security sandbox.
7. **Showcase + report** — benches validated against datasheets, scenario tests, sweeps,
   rendered frames, and a fidelity report listing every model's provenance and error.

## Fault injection (F3)

Scenario `[[faults]]` (schema: `python/napl/program.py` docstring; examples: `examples/faults/`; tests: `tests/python/test_faults.py`,
`test_faults_px4.py`). A fault is a scripted SetAttr through the bus on a device input, so it is in the replay log and the replayed
hash matches. Trigger: `at` (sim s) or `after = "<program>+s"` (optionally from a program's `log_marker` line); optional `duration`
undoes it. Monitors can use `phase = "fault:<name>"` (from / to relative to the fault window) and the derived trace column `tilt`.

| kind | device attr (new unless noted) | physics |
|---|---|---|
| motor_out | esc `fault_open` | phase open: no drive torque, no back-EMF damping, the rotor coasts on friction + prop torque |
| motor_degraded | rotor `damage` (thrust_loss) | C_T and C_Q x (1 - damage): a chipped prop. (Not modelled: a Kt / efficiency drop of the motor itself.) |
| battery_cell_fault | battery `r_fault` (ohm), `ocv_offset` (V) | extra series resistance / lost EMF of one cell in the Thevenin pack: V = OCV + offset - v_rc - I (r0 + r_fault) |
| gnss_outage | gnss `fix_lost` | output holds; the PX4 bridge sends HIL_GPS fix_type 0, 0 satellites |
| gnss_jump | gnss `jump` (E, N, U m) | position step in all later samples (through the latency ring) |
| baro_drift | baro `drift_rate` (Pa/s), `bias_extra` | bias ramp; stays when the drift stops |
| imu_bias_step | imu `gyro_bias`, `accel_bias` (existing state, now settable) | the scenario adds the step to the current bias |
| servo_overload | servo `ext_torque` | external torque on the output joint |
| servo_stall | servo `jam` | joint seized: spring 200 N m/rad + damper 1 N m s/rad at the angle it had |
| comms_drop | port `drop` (bus side) | the pty discards the host's bytes: real timeouts in the SDK; not in the replay log (the commands that did arrive are) |

Measured on real PX4 SITL (X500 fixture, 5 m HOLD, fault 3 s after the hold starts, PX4 instance 4):
- **GNSS outage 10 s**: PX4 rides it out on baro / IMU for 5.0 s, then "Failsafe activated" and LAND at +5.7 s (position-health flag
  false at +7.7 s); the quad lands in place at +11.5 s with 0.2 m horizontal drift; GPS health is back 1.7 s after the fix returns.
- **One motor open (esc0)**: no failsafe and no mode change at all. PX4 prints only "Preflight Fail: Attitude failure (roll)" at +1.0 s;
  the quad is below 0.5 m at +1.3 s, max tilt 180 deg (upside down, altitude -0.46 m). SITL gives PX4 no ESC feedback.
- **+60 mOhm cell**: PX4's battery voltage 16.48 V -> 15.82 V (-0.66 V; hover current 10.3 A, I R = 0.62 V).
- **SO-101 comms drop 0.5 s mid-move**: 13 consecutive read timeouts in the user program, the pan holds its last goal (0.14 rad vs 0.32
  commanded), the program resumes and reaches the 0.5 rad target (final error 3 mrad).

## Milestones (after N2)

| # | Scope | Exit check |
|---|---|---|
| P1 | Library expansion (3 research tracks, ports on every part, retrofit ports on N1 parts) | check_parts 0 problems; >= 200 parts; cold verifier sample >= 90% confirmed |
| N3' | Assembly schema + builder + snapping + fastener selection; X500 V2 + SO-101 assemblies | both build and load; mass budget reported vs published; snapping tests (pattern match, symmetry, gender, fastener choice) |
| D1 | DRC engine (mechanical, electrical, performance) | seeded-fault tests: each rule fires on a broken design and not on the reference designs |
| N4–N6 | benches, PX4 SITL, arm + Feetech pty | as phase-2 plan, plus: unmodified Feetech-protocol client moves the sim arm |
| E1 | Editing API + MCP design tools | agent-style test: swap motor -> DRC -> build -> hover test, all through MCP |
| N7 | Showcase + fidelity report | scenario TestSpecs pass; report generated |
