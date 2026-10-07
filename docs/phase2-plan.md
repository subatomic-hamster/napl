# Phase 2: real-parts drone and robot arm

Goal (user, 2026-10-06): simulate a drone and a robot arm built entirely from real parts (motors,
props, screws, wiring, PCBs, batteries), physics-based end to end, and use them to showcase what
the kernel can do. Custom CAD is allowed only for parts we design ourselves.

Chosen platforms:
- **Drone:** Holybro X500 V2 quad with Pixhawk 6C running real **PX4** firmware (BSD-3) in the
  loop over MAVLink lockstep.
- **Arm:** **SO-101** (TheRobotStudio SO-ARM100 repo, Apache-2.0 CAD), 6x Feetech STS3215 bus
  servos, real driver board, PSU and cabling.

## Ground rules

1. **Every number has provenance.** Each part parameter carries a source and a fidelity tag:
   `datasheet | literature | measured | derived | estimated`. `derived` states its derivation,
   `estimated` states its rationale. No unsourced numbers. See `parts/README.md`.
2. **Physics, not lookup.** Motors spin real rotor bodies with real inertia; thrust/torque come
   from prop coefficients and inflow; current comes from back-EMF and winding resistance; the
   battery sags; servos are DC motor + gearbox + firmware PID. Behaviour (e.g. motor response
   time, flight time) must emerge, never be scripted.
3. **Validate against published data** and report the error. Fidelity limits are stated per
   model; we never claim more accuracy than the validation shows.
4. All compute on hfbox via `tools/remote.sh` (house rules apply). Commit each verified milestone
   on `napl-v1`; never push.
5. Proprietary repo: permissive licenses only in-tree (Apache/MIT/BSD). PX4 and other external
   firmware are fetched on the build host, not vendored.

## Architecture additions

- **Parts DB** `parts/<domain>/<id>.toml` — one file per real part (schema in `parts/README.md`).
- **Assemblies** `assemblies/<name>.toml` — BOM (part ids x qty), placements, fasteners, wiring
  graph (nets with wire gauge/length and connectors), device bindings.
- **Builder** `python/napl/build.py` — assembly -> MJCF + device config. Mass/inertia come from
  parts (CAD meshes with material density, or datasheet mass on the part's geometry).
- **Devices** (C++, `src/mujoco/devices/`) stepped inside `MujocoSolver::step` before each
  `mj_step`, deterministic, included in `save()`/hash/replay, exposed as prims with units
  (`/devices/<name>.<attr>`):
  - electrical network: battery (OCV(SoC) + Thevenin RC), wires (AWG resistance), connectors,
    power module shunt + ADC
  - ESC (averaged duty model, current limit) + BLDC motor (Kv, R, I0, thermal R(T))
  - rotor aero: thrust/torque from C_T(J), C_Q(J), rotor drag, ground effect; applied to the
    spinning rotor body
  - servo: DC motor + gear train (ratio, efficiency, backlash, friction) + encoder + firmware PID
    + bus (baud, packet latency)
  - sensors: IMU/baro/mag/GPS with datasheet noise density, bias, quantization, mount dynamics
  - environment: atmosphere (ISA), wind (steady + Dryden turbulence)
- **PX4 bridge** `napl-px4-bridge` (C++, MAVLink c_library_v2, MIT): lockstep HIL_SENSOR /
  HIL_GPS out, HIL_ACTUATOR_CONTROLS in, owns a World; PX4 SITL built on hfbox.

## Milestones

| # | Scope | Exit check |
|---|---|---|
| N1 | Parts research: drone propulsion/power, drone airframe/avionics/sensors, SO-101 full BOM; `tools/check_parts.py` | every param sourced + fidelity-tagged (checker exits 0); cold verifier re-fetches a sample of sources and confirms quotes; BOM masses summed |
| N2 | Device framework + electrical/motor/rotor/servo/battery/sensor models in C++ | analytic tests: motor no-load speed, stall current, battery step response, rotor thrust = C_T rho n^2 D^4; devices in save/hash/replay (determinism + replay tests still pass) |
| N3 | Assembly schema + builder; X500 V2 and SO-101 assemblies | built models load; mass budget vs published kit/arm weight within stated tolerance; every geom traces to a part id |
| N4 | Drone benches: motor/prop thrust stand, hover | thrust stand reproduces vendor thrust table (thrust, current vs throttle) within +-10%; open-loop hover throttle reported |
| N5 | PX4 SITL bridge + PX4 build on hfbox | PX4 arms, takes off to 5 m, holds 30 s (monitor: alt within +-0.5 m), lands; battery sag visible in PX4 telemetry |
| N6 | SO-101 arm: STS3215 device, bus, PSU, IK pick-and-place | servo bench reproduces datasheet stall torque and no-load speed within +-10%; pick-and-place test passes; payload-limit sweep reported |
| N7 | Showcase: combined scene (arm loads payload onto drone, drone flies, lands, arm unloads), wind/gust requirement tests, Monte Carlo flight-time sweep, MCP-driven run, rendered frames, fidelity report page | scenario TestSpecs pass; report lists every model's fidelity and validation error |
