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

## Milestones (after N2)

| # | Scope | Exit check |
|---|---|---|
| P1 | Library expansion (3 research tracks, ports on every part, retrofit ports on N1 parts) | check_parts 0 problems; >= 200 parts; cold verifier sample >= 90% confirmed |
| N3' | Assembly schema + builder + snapping + fastener selection; X500 V2 + SO-101 assemblies | both build and load; mass budget reported vs published; snapping tests (pattern match, symmetry, gender, fastener choice) |
| D1 | DRC engine (mechanical, electrical, performance) | seeded-fault tests: each rule fires on a broken design and not on the reference designs |
| N4–N6 | benches, PX4 SITL, arm + Feetech pty | as phase-2 plan, plus: unmodified Feetech-protocol client moves the sim arm |
| E1 | Editing API + MCP design tools | agent-style test: swap motor -> DRC -> build -> hover test, all through MCP |
| N7 | Showcase + fidelity report | scenario TestSpecs pass; report generated |
