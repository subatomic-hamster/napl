# EDF hopper (phase 5)

A throttleable thrust-vectored VTVL hopper from cheap, open parts: an 80 mm 12-blade EDF on 6S, four aluminium jet vanes on four XL330 servos, a Teensy 4.0 flight computer, lidar +
baro + IMU + GNSS, a sensor-only Python flight stack. It replaces the solid-motor TVC lander (`docs/lander.md`, F15 net Delta-v about 0.4 m/s at 1.45 kg, 35 % held-out) as the
showcase landing vehicle: a fan can be throttled, so the vehicle hovers, diverts and lands softly.

Full write-up (parts and prices, wiring, flight software, missions, Monte Carlo tables, honesty notes, telemetry schema, commands): `examples/hopper/README.md`.

Kernel side (equations in `docs/models.md`, rows "Jet vane" and "Rotor swirl recovery"): `schema/devices.fbs` (`JetVane`, rotor `stator_body` / `swirl_recovery`),
`src/mujoco/devices/vane.cpp`, `aero.cpp`; tests `tests/cpp/test_jetvane.cpp`. Both are appended last in the saved state, so every older state hash is unchanged.

Design: `assemblies/hopper.toml` (`napl design check assemblies/hopper.toml`: 0 errors, 0 warnings), parts `parts/propulsion/qx-*`, `hobbywing-platinum-120a-v5-esc`,
`parts/rocket/generic-jet-vane-alu-45x34`, `hopper-deck-frame-edf80`, `parts/electronics/ublox-zed-f9p`.

Tests: `tests/python/test_hopper.py` (allocation signs against the built model, trajectory, estimator, design constants, builder keys, params, three flights, replay hash, MC
summary).
