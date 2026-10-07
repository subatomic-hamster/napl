# napl

**Start with the [user guide](docs/guide.md)**: install, parts library, building and checking a robot, simulating it, flying it under
PX4, driving it over its real serial protocol, agents, and what is and is not validated. Every command in it was run.

What napl is now, in ten lines:

1. A deterministic physics kernel (C++20 on MuJoCo) with device models: battery, ESC + BLDC motor, rotors, bus servos, IMU / baro / mag / GNSS, wind.
2. A sourced parts library (`parts/`, hundreds of real parts): every number has a source and a fidelity tag (datasheet / literature / measured / derived / estimated).
3. Robots as data (`assemblies/`): instances of parts, port mates, fasteners, wires, device bindings; `napl design build` makes MJCF plus a mass / BOM / provenance report.
4. An editing API (`napl.design.Design`), the same operations as the MCP `design_*` tools and the viewer's Design window: add, snap, swap, remove, autowire, undo.
5. A design-rule checker (mechanical, electrical, performance): every finding has a code, a severity and a concrete fix.
6. Fast design simulation (hover / hold), virtual thrust-stand benches against vendor tables, and a generated fidelity report.
7. `napl program`: your own program runs against the simulated robot over the real protocol (PX4 SITL via MAVLink, Feetech STS, Dynamixel 2.0), with monitors and a replay hash.
8. Fault injection (motor out, GNSS outage, weak cell, comms drop, ...) in the same scenario files.
9. An MCP server so agents can edit, check and simulate designs; destructive tools need approval, and programs are never an agent tool.
10. Everything builds and runs on a remote host through `tools/remote.sh` (hfbox today, any Ubuntu 24.04 box tomorrow).

Reference: [assemblies/README.md](assemblies/README.md) (schema, editing API, design rules), [parts/README.md](parts/README.md),
[docs/models.md](docs/models.md), [docs/validation/](docs/validation/), [docs/fidelity.md](docs/fidelity.md) (generated), [docs/phase3-plan.md](docs/phase3-plan.md).

---

A collection of code written for the Neegy Applied Physics Lab, including proprietary simulation software, embedded systems, high frequency trading algorithms, and more, all run through old reliable hfbox.
