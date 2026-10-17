# Dog locomotion (SP4)

The dog (`assemblies/dog/dog.toml`, 12 Qdd actuators, 11.65 kg) walks on its own controller, a Python package `examples/dog/loco/` that is the
robot's software (what would run on the Jetson). `loco/hw.py` is its only door to the robot (Qdd and IMU device attributes), `model.py` builds
its own kinematic and inertial model from the design, `estimator.py` fuses IMU and legs (Mahony attitude + a linear Kalman filter on body and
feet, torque-based contact), `mpc.py` is the Di Carlo 2018 convex MPC on ground reaction forces (OSQP), `gait.py` schedules phases and
footsteps, and the leg controller turns forces into MIT-mode joint frames at 500 Hz. Rates: physics 1 kHz, leg control 500 Hz, MPC 50 Hz.
Harness modules beside the package (`dogtruth.py`, `dogcourse.py`, `dogwalk.py`, ...) play the world and are never imported by `loco`.
Everything below is measured on hfbox; numbers are quoted from the run that produced them.

## Compute budget

Measured by `tools/dog_loco_bench.py --sim 2.0` on the flat scene (lidar removed), the 500 Hz loop of the dog under a held stand PD. `sim-s per
wall-s` is the whole loop (physics + Python calls); the last two columns are per 2 ms control tick.

| run                                                                    | sim-s per wall-s | read us / tick | write us / tick |
| ---------------------------------------------------------------------- | ---------------- | -------------- | --------------- |
| (a) physics only, held stand                                           | 13.05            | 0              | 0               |
| (b) + Hw.joints() + Hw.imu()                                           | 10.33            | 39             | 0               |
| (c) + Hw.send, kp/kd constant (36 SetAttr via the JSON bus, `Sim.set`) | 0.63             | 49             | 2940            |
| (d) + Hw.send, kp/kd changing (72 SetAttr, `Sim.set`)                  | 0.39             | 49             | 4826            |
| (c) with `Sim.set_many`                                                | 7.94             | 40             | 54              |
| (d) with `Sim.set_many`                                                | 6.99             | 40             | 90              |

Decision: **I/O per control tick with `Sim.set` is 2.99 ms (rule: > 1.0 ms -> Step 6)**, so the C++ batch write was added: `Sim.set_many`
(`src/python/bindings.cpp`, one Envelope per entry through the same command bus, so logs, replay and every existing hash are unchanged;
pinned by `tests/python/test_dog_loco_io.py::test_set_many_is_set_in_a_loop`). I/O per control tick is now 0.09 ms (read 40 us + write 54 us),
about 5 % of the 2 ms tick; `Hw` uses `set_many` when the Sim has it. Reads stay Python (`Sim.query` is a direct solver read).
