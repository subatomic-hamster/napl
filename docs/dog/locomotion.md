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

Convex MPC (`tests/python/test_dog_mpc.py::test_solve_time_budget`, 100 solves, OSQP 1.1.3, N = 12 steps of 25 ms, sparse fixed pattern, warm
start, hfbox with `OMP_NUM_THREADS=1`): **median 5.21 ms, max 10.33 ms** per solve against the 8 ms budget (40 % of the 20 ms MPC period); N
stays 12. The MPC runs every 10th control tick, so it adds 0.5 ms per tick on average.

## Flat ground (Task 6)

Controller on flat ground: MPC ground-reaction forces on the stance feet, mapped to joint torque by `tau = -J^T R^T f` plus the torque that
carries the leg's own weight; swing legs follow a min-jerk curve on a joint PD (KP 40, KD 1.2). Heading hold, capture-point footholds. The trot
(period 0.32 s, duty 0.5) was debugged with a diagnostic ladder, every rung on GROUND-TRUTH state (`examples/dog/dogladder.py`), each with a test.

| rung                                 | test                                   | result                                                                                                                                  |
| ------------------------------------ | -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| a. Jacobian + force map, 4 legs      | `test_dog_legs.py` (3 tests)           | J matches finite differences of the sim foot for all 4 legs (fr mirrored, rl / rr turned 180 deg); sign and frame of `J^T f` right      |
| b. MPC-only stance, steps, shove     | `test_dog_stance.py`                   | 12 s: 5 cm height step, 7 deg roll step, 0.3 m/s shove, no fall; max torque fraction abad 0.25 / hip 0.04 / knee 0.16, never above 0.95 |
| c. trot in place, 12 s               | `test_dog_trot_truth.py[0.0]`          | no fall, x drift 1 cm, touchdown error mean 2.1 cm (max 8.3 cm), hip torque above 95 % of limit in 0.4 % of ticks                       |
| d. trot 0.5 and 1.0 m/s, truth state | `test_dog_trot_truth.py[0.5]`, `[1.0]` | no fall in 14 s, mean speed 0.46 and 0.93 m/s, y drift 5 cm at 1.0 m/s                                                                  |

Force map (rung a, first 16 ms of a 15 N change at one foot): x gain 1.0, y 1.4-1.8, z 0.5-0.6 of the commanded; the leg's operational mass at
the foot (reflected rotor inertia: 0.1 / 0.27 / 1.6 kg in x / y / z, 6.4 kg for four legs against 11.65 kg) explains it.

Root causes the ladder found, in the order found (none was a threshold):

1. **Leg weight not carried by the stance torque.** In a four-foot stand the feet walked inward at 1.3 cm/s and the dog fell at 13 s; the
   distal leg mass (about 0.56 kg) was missing from the torque, so the realized ground force differed from the command. Adding
   `Model.leg_required(...)` (the same function the contact estimator subtracts) removed it (`test_dog_stance.py`). It was also what kept
   the trot inside a narrow basin.
2. **MPC stand weights.** Roll / pitch weights 2.5 (MIT 0.25), roll / pitch rate weights 1.0 (MIT 0), force weight 4e-6 (MIT 4e-5); lever arms
   about the predicted CoM; the stand holds the CoM over the feet centroid. Each was shown unstable in an ideal-plant closed loop first
   (a 2 cm CoM offset grows to a fall in 3 s with the MIT values). xy velocity weight 3.0 (MIT 0.2): the trot settled 7-10 % under the command.
3. **Estimator, in the loop.** With the controller on truth the estimator is accurate (velocity error 0.04-0.1 m/s); in the loop it was not.
   Truth-substitution per output (`dogladder.truth_state(parts)`, 4 seeds, forced trot, 12 s) put the velocity first:
   truth v alone 4/4 no fall, truth R alone 1-2/4. The cause was a stance foot flagged in contact while still moving (landing, lift-off),
   whose kinematic velocity reads its own speed (up to 5 m/s): a velocity innovation above 0.6 m/s (clean trot: below 0.2) is now rejected
   (`V_GATE`). Also: the tilt correction uses the accelerometer and predicted gravity averaged over 0.3 s and pauses while the velocity
   command changes (the specific force is not g then; tilt error 2 deg to 0.1 deg in a steady trot); the feet handed to the controller are
   forward kinematics on the estimated pose (the KF foot states lag them).
4. **Launch.** A trot started while accelerating at 2 m/s^2 flew and fell (2 of 4 seeds). The gait now steps in place for 0.64 s (two periods)
   before the velocity command is released, and the slew is 1.0 m/s^2: 8 of 8 seeds hold 1.0 m/s for 8 s (seed 3 stumbles to y = 0.79 m).

Tried and not kept (measured, in this order): latency compensation of the MPC state (14 ms): stable in the ideal plant to 60 ms of lag, chatter
in the sim, removed; a 0.3 s accelerometer average without the acceleration pause (worse in the loop, 0 of 4).

Scenario `walk/trot.scenario.toml` (1.0 m/s, 8 s, seed 1): speed 0.96 m/s (late mean), no fall, estimator velocity error max 0.07 m/s, contact
agreement 0.98, replay hash `b3a9d5997271ab5a` (golden, J^T controller; the default controller is now the WBC, see below). Open: robustness is a
sample of 8 seeds, not yet the Task 10 Monte Carlo; a pack-current peak per gait is not measured yet (Task 10).

## Whole-body control (Task 7)

`loco/wbc.py` is a WBIC-style whole-body controller (Kim et al. 2019, arXiv:1909.06586) on the controller's own model: kinematic tasks by
null-space priority (stance feet still, trunk orientation, trunk position, swing feet) give joint targets and a commanded generalized
acceleration; a QP (OSQP) then adjusts the floating-base acceleration and the ground forces around the MPC forces so that the friction pyramids,
`F_MIN..F_MAX` on the normal force and the drivers' torque limits all hold, and its joint torques are the feed-forward of the MIT-mode frames
(`leg_ctl = "wbc"`, the default). `leg_ctl = "jt"` keeps the Jacobian-transpose controller of Task 6 as the switchable baseline.
If the QP fails (status not solved) that tick goes to J^T (`wbc_fail`): 0 of 4000 ticks in each run below.

Solve time (`test_dog_wbc.py::test_solve_time_budget`, trot pose, joint speeds drawn from N(0, 3 rad/s), hfbox, `OMP_NUM_THREADS=1`):

| implementation       | median  | p99      | against the 1.0 ms budget                                                                      |
| -------------------- | ------- | -------- | ---------------------------------------------------------------------------------------------- |
| Python (numpy)       | 9.21 ms | 10.10 ms | over (the rest of the tick is Hw I/O 0.09 ms, estimator, gait)                                 |
| C++ (`loco_wbc_cpp`) | 0.26 ms | 0.27 ms  | under (a loaded host showed p99 2.7-4.2 ms from OSQP polish spikes; the median stayed 0.27 ms) |

Profile of the Python solve: 86 % of the time is the kinematics (`_elements` is evaluated four times per solve and `np.cross` costs 60 % of
it), the QP itself is small. Decision (Step 6 rule: Python over budget): **C++ port**, `examples/dog/loco/cpp/wbc.cpp` (Eigen 3.4.0 MPL-2.0
with `EIGEN_MPL2_ONLY`, OSQP C v1.0.0 Apache-2.0, nanobind), built with `cmake -S . -B build -DNAPL_LOCO_CPP=ON && cmake --build build
--target loco_wbc_cpp` into `build/loco/`; `loco.wbc.make()` loads it when it is there and falls back to the Python class (same maths, ~35 times
slower, different last bits: the trot golden below is the C++ one and is skipped without the module). The kinematics are computed once per solve.
Python and C++ agree to 1e-3 relative (`test_cpp_dynamics_match_python`, `test_cpp_wbc_matches_python`). The WBC therefore runs at the full
500 Hz; no spec deviation.

Trot at 1.0 m/s on flat ground, 8 s, seeds 1-4 (4 parallel runs on hfbox; late = t > 5 s; force tracking = RMS over feet in true contact and in
the MPC stance of |F_true| - |f_mpc|, per m g, t > 3 s):

| seed | wbc vx | wbc max abs y | wbc force RMS | jt vx | jt max abs y | jt force RMS |
| ---- | ------ | ------------- | ------------- | ----- | ------------ | ------------ |
| 1    | 0.975  | 0.073         | 0.073         | 0.963 | 0.046        | 0.132        |
| 2    | 0.974  | 0.009         | 0.080         | 0.961 | 0.014        | 0.156        |
| 3    | 0.974  | 0.045         | 0.080         | 0.921 | 0.790        | 0.459        |
| 4    | 0.974  | 0.018         | 0.075         | 0.963 | 0.012        | 0.140        |

No fall in any run. The WBC realizes the MPC forces about twice as accurately as J^T and has no stumbling seed (J^T seed 3 drifts to y = 0.79 m).
Scenario `walk/trot.scenario.toml` (default `leg_ctl = "wbc"`, seed 1): passes, replay matches, golden `9ed76c26537ef5fa`. Hardening that was
needed (not tuning): damped pseudo-inverses (an undamped one sent a swing joint to +-100 rad at the straight-leg singularity) and the same
joint-step (0.3 rad) and velocity (40 rad/s) clips as the J^T path's IK. No gain of `wbc.py` was changed from the plan's values.

Naming deviation: the WBC's body reference is `Controller.wbc_ref` (the plan called it `body_ref`, which the ladder harness already uses for its
(dz, droll, dpitch) array).
