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

Convex MPC (`tests/python/test_dog_mpc.py::test_solve_time_budget`, 100 solves, OSQP 1.1.3, N = 12 steps of 25 ms, condensed form, warm
start, `EPS = 1e-6`, hfbox with `OMP_NUM_THREADS=1`): **median about 6.0 ms, max about 40 ms (first solves and load spikes)** per solve against the
8 ms budget (the test asserts the median); N stays 12. The MPC runs every 10th control tick, so it adds 0.6 ms per tick on average.
The first version (states kept as equality constraints, 300 variables) measured 9.2 ms median on an idle box and failed the test: profiling put
7.3 ms of 8.4-9.9 in the OSQP call itself (300-850 ADMM iterations at 13 us), 1.3 ms in model building, 0.36 ms in `update`; settings (rho,
alpha, sigma, check_termination) changed nothing robust, and Python overhead is not the cost, so a C++ port would not help. Eliminating the states
(x = Sx x0 + Su u, 144 variables, dense Hessian) needs about 75 iterations and its forces agree with a 1e-7 solve to 0.03 N median (the sparse
form: 39 N max). ADMM stops at a residual, so the first-step forces are projected onto the exact friction pyramid and force bounds before use
(0.07 N over the pyramid and 1e-3 N on swing feet otherwise). Tolerance trade (median / max, idle box): 1e-4 4.2 / 36.6 ms, 1e-5 5.0 / 38.1,
1e-6 6.0 / 41.5; 1e-4 let the 15 deg ramp test fall (the closed loop is sensitive to force accuracy), so 1e-6.

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
--target loco_wbc_cpp` into `build/loco/` (the default build has `NAPL_LOCO_CPP` OFF; on a fresh build dir the configure uses `.venv/bin/python`
when `-DPython_EXECUTABLE=...` is not given; `test_dog_wbc.py::test_solve_time_budget` fails with the build line when the module is missing);
the controller pins BLAS / OpenMP to one thread at `import loco` (an unpinned BLAS measured 366 ms per MPC solve on a loaded box;
`test_solve_time_budget_does_not_depend_on_the_callers_thread_env`); `loco.wbc.make()` loads it when it is there and falls back to the Python class (same maths, ~35 times
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
Scenario `walk/trot.scenario.toml` (default `leg_ctl = "wbc"`, seed 1): passes, replay matches, golden `092d213d8df922d8` (Task 8 made the
swing-joint torque rows free, see below; it was `9ed76c26537ef5fa` before). Hardening that was
needed (not tuning): damped pseudo-inverses (an undamped one sent a swing joint to +-100 rad at the straight-leg singularity) and the same
joint-step (0.3 rad) and velocity (40 rad/s) clips as the J^T path's IK. No gain of `wbc.py` was changed from the plan's values.

Naming deviation: the WBC's body reference is `Controller.wbc_ref` (the plan called it `body_ref`, which the ladder harness already uses for its
(dz, droll, dpitch) array).

## Terrain (Task 8)

Built (all of it pure map maths on the height-map interface, `loco/terrain.py`; the flat map takes a fast path that returns the nominal, so the
flat trot is bit-identical apart from the WBC change below): footholds off edges and holes (`foothold`), swing apex over the highest ground on the
path capped at `APEX_MAX = 0.12 m` (`apex_over`; an uncapped apex of 0.4-0.5 m folded a hind leg under the body in a rubble run), a ground
normal per foot (`normal`) that tilts the friction pyramids of the MPC and the WBC (`mpc.pyramid(mu, n)`), a posture plane and a support height
from the four feet (stance feet where they are, swing feet at their targets) instead of the terrain under the CoM, both rate-limited
(`Z_RATE` 0.25 m/s, `ANG_RATE` 0.5 rad/s), and a reach clip (`Controller._reach_clip`: the support height keeps every hip pitch point within
0.6-0.95 of the straight leg from its foot at the posture the MPC is asked for). `rough_ahead` (a riser above 8 cm per 5 cm) and `steep_ahead`
(over 12 deg per 0.3 m, separate because the plan's test wants the 25 deg ramp not to be `rough_ahead`) cap the speed at 0.5 m/s.
The WBC's swing-joint torque rows are free (only the stance legs' torques are bound) and, when a QP is infeasible, it is solved once more with
all torque rows dropped and the torques clipped afterwards (`wbc_relaxed`); with bound swing rows the 25 deg ramp made the QP infeasible
whenever a swing foot was asked for more than the drivers give.

Measured (hfbox, seed 1 unless stated, WBC C++). Criterion course (flat, ramp up, plateau, two 15 cm steps up and down, ramp down), trot at
0.5 m/s, `dogcourse.RAMP_DEG` varied:

| ramp   | result                                                                                      |
| ------ | ------------------------------------------------------------------------------------------- |
| 10 deg | climbed to the plateau, no fall; falls at the first 15 cm step (x = -4.1 m, t = 20 s)       |
| 15 deg | climbed to the plateau (tilt 9 deg on the ramp), no fall; falls or stalls at the first step |
| 20 deg | falls at the foot of the ramp (t = 10 s, tilt 37 deg at x = -8.9 m)                         |
| 25 deg | falls at the foot of the ramp (0.25 and 0.5 m/s alike, so not a speed limit)                |

`test_trot_climbs_a_15_degree_ramp_and_stands_on_the_plateau` pins the 15 deg result. **The 25 deg ramp, the 15 cm steps and the rubble field
are NOT achieved**: `stairs` and `rubble` fail their monitors (`reaches_the_end`, `never_falls`), so they have no golden and their scenario test
is `xfail(strict=True)`. What the diagnosis found, in order:

- At the foot of a 25 deg ramp the feet are 0.25-0.3 m apart in height over a 0.65 m span while the posture reference follows only 70 % of the
  plane and is rate-limited; the hind legs, still on the flat, then reach full extension (knee 0.06 rad) and the QP has no feasible torque
  (`wbc_relaxed` climbs from 0 to 100 in 0.6 s). The reach clip and a full-plane posture (`POSTURE = 1.0`) move the failure, they do not remove it
  (stalled low at the ramp foot instead of flipping). J^T falls at the same place, so it is not the WBC.
- Walk gait: forced on FLAT ground at 0.3 m/s it falls with both leg controllers (tilt grows 11 -> 29 deg; falls at 4-5 s with the plan's
  period 0.6 s, at 8 s with 1.2 s). The cause is not found (the contact set the MPC loads and the feet that touch differ: all four feet
  leave the ground within a step). `WALK_ROUGH = False` in `controller.py` therefore keeps rough ground on the trot (a deviation from the plan,
  where `rough` selects the walk); `gait.choose(.., rough=True)` still returns walk and is unchanged.
- Trot speed: the swing-leg feed-forward torque saturates (knee 44 N*m) at about 1.3 m/s; a flat trot commanded 1.6 m/s falls at 5 s. This is
  the Task 10 top-speed question, but it also means the plan's rubble scenario (1.8 m/s) is out of reach at the moment.
- Rubble seed 9 (a route entirely on rock) at 1.0 m/s: 7 m of the route covered in 8 s, then a stance leg runs to the end of its workspace on flat ground
  (hip 0.27, knee -0.06 rad), the knee torque pegs at 44 N*m and the leg is thrown up to 0.5 m. Seed 1 starts on wet rock (mu 0.35, below
  `MU_NOM = 0.5`) and slips at launch; with `MU_NOM = 0.3` it reaches 1.6 m/s and then fails the same way. Slip handling is Task 9.
- Untested idea, not a finding: the estimator takes the map height under the estimated stance foot position, which at a 15 cm riser is a 15 cm
  error for a 2 cm position error (its `feet_z` read 0.1-0.3 m above the true ground on the step).

Deviations: `dogcourse` scales the rock / boulder / scree counts of the 30 m fields by (30/100)^2 (the generator's defaults are for 100 m);
`rubble` uses seed 1 as in the plan's scenario (wet rock start), no seed was picked for a result.

## Slips, pushes, falls (Task 9)

`loco/safety.py`. Slip: a stance foot (estimated contact, > 20 ms after touchdown) whose world velocity exceeds `SLIP_V = 0.25 m/s` for
`SLIP_TICKS` ticks gets its MPC friction lowered by 0.15 (floor 0.2), the stance widened by 3 cm and the command scaled to 0.6 for 2 s; mu
recovers at 0.05 per s. Fall: tilt > 40 deg, or > 25 deg while rolling faster than 4 rad/s, or body under half the stand height -> legs fold to a
sit with soft gains (kp 20 / kd 1), then limp; after 1.5 s the dog stands up again if tilt < 30 deg, else stays down. **There is no self-righting:
a dog on its back or side stays down.**

Deviation: `SLIP_TICKS = 40` (80 ms), the plan has 3. Evidence: the estimator's lateral velocity lags the truth after a push (est 0.05-0.14 vs
true 0.5 m/s for about 50 ms; an error of 0.2-0.4 m/s lasts up to 26 ticks), so with 3 ticks a 0.5 m/s push read as a slip on three feet, mu fell
to 0.35 / 0.21, the command dropped to 0.6 and the dog fell; with 40 ticks it recovers (vx 1.02 at 6 s). On the mud strip at 1.0 m/s the dog does
not slide at all (only 20 ms touchdown transients of 0.2-0.4 m/s): 3 ticks fired 4 false slips and cost 15 % of the speed (mean 0.845), 40 ticks
give n_slip 0, mean 0.903, strip crossed. The plan's "slip detected on the mud crossing" is therefore replaced by two tests: no false slip on the
crossing, and a real slide (1 m/s kick on the mud) is detected (n_slip >= 1, mu lowered). Kicks of 1.0 and 1.5 m/s on mud are detected (n_slip 2
and 4) but the dog falls anyway; recovery from a slide is not claimed.

Push envelope (sideways velocity step at t = 5 s, flat, trot 1.0 m/s, 10 s, WBC C++, 5 seeds each):

| push (m/s)         | no fall  | note                                      |
| ------------------ | -------- | ----------------------------------------- |
| 0.25               | 5/5      | speed back to 0.96-0.97, tilt max 2.3 deg |
| 0.50               | 5/5      | speed back to 0.96-0.98, tilt max 4.3 deg |
| 0.75               | 0/5      | all fall                                  |
| 1.00 / 1.25 / 1.50 | 0/5 each | all fall                                  |

The limit lies between 0.5 and 0.75 m/s. It is not the slip logic: with slip detection off (`SLIP_TICKS` huge) 0.75 and 1.0 m/s kicks still fall
(seeds 1, 2: 4/4 fell, n_slip 0). The plan's scenario push (0.5 m/s) passes with a golden (`push.scenario.toml`).
All of these pushes are at t = 5.0 s, i.e. one gait phase; other phases were not measured, and the Monte Carlo's random-phase pushes of 0.2-0.5 m/s do topple the dog (see "Monte Carlo").

## Top speed (Task 10)

`examples/dog/dogtop.py`: each run stands up, ramps the command at 1 m/s^2 to `v_try`, holds 4 s; ok = TRUE mean forward speed over the last 3 s

> = 0.95 `v_try`, no fall, no driver fault, MPC failures < 5 %. A ladder climbs `v_try` in 0.25 m/s steps from 0.5 until two consecutive failures;
> `v_top` = the last ok rung. The gait is forced (`Controller.forced_gait`), so the flying trot is flown at any speed. Training seed 1001
> (PoseFix noise), held-out seeds 21000-21004 (payload 0-1 kg at +-3 / +-2 cm, IMU biases, as the Monte Carlo of Task 11).

```
- flat top speed (measured, flying_trot, wbc): 2.25 m/s; held-out seeds n = 5: median 2.00, min 2.00 m/s
- spec target 3.5 m/s: NOT met (gap 1.25 m/s)
- leg controller: wbc 2.25 m/s, jt 2.50 m/s
- at the first failing speed (2.5 m/s): hip torque at its limit 19 % of ticks, knee 6 %; 0 slips; pack peak 128 A
```

The 3.5 m/s criterion is **NOT met**. The validated flying-trot speed of the WBC is 2.0 m/s (median and minimum of 5 held-out seeds), 2.25 m/s on
the training seed. The J^T baseline is faster at its best: flying trot at the default stand height, 2.5 m/s on the training seed, median 2.25 on the
held-out seeds (2.25 / 2.25 / 2.25 / 2.5 / 2.75). **J^T beats the WBC at speed**; the WBC stays the design's leg controller (it is better at 1 m/s:
force RMS 0.073-0.080 vs 0.132-0.156 of m g, Task 7) and the gap is a finding for the orchestrator. Its cause is not established (see below).

| leg | gait        | stand height fraction | v_top train m/s  | held-out v_top (n = 5)  | pack peak A at v_top / at the first failing rung | CoT at v_top | force RMS / m g at v_top |
| --- | ----------- | --------------------- | ---------------- | ----------------------- | ------------------------------------------------ | ------------ | ------------------------ |
| wbc | trot        | 0.95 (default)        | 0.75             | not run                 | 52 / 122                                         | 1.51         | 0.125                    |
| wbc | flying_trot | 0.95 (default)        | 1.00             | not run                 | 59 / 112                                         | 1.40         | 0.134                    |
| wbc | bound       | 0.95 (default)        | 0.50             | not run                 | 122 / 123                                        | 10.2         | 0.722                    |
| wbc | trot        | 0.85                  | 2.00             | 2.25 2.0 1.75 2.0 2.0   | 28 / 147                                         | 1.04         | 0.131                    |
| wbc | flying_trot | 0.85                  | **2.25**         | 2.0 2.25 2.0 2.0 2.25   | 42 / 128                                         | 1.29         | 0.188                    |
| wbc | bound       | 0.85                  | 0 (fails at 0.5) | not run                 | - / 57                                           | -            | -                        |
| jt  | trot        | 0.95 (default)        | 1.25             | not run                 | 71 / 71                                          | 1.10         | 0.150                    |
| jt  | flying_trot | 0.95 (default)        | **2.50**         | 2.25 2.25 2.25 2.5 2.75 | 58 / 73                                          | 1.98         | 0.497                    |
| jt  | bound       | 0.95 (default)        | 0 (fails at 0.5) | not run                 | - / 93                                           | -            | -                        |
| jt  | trot        | 0.85                  | 2.00             | 1.75 2.0 1.75 2.0 1.75  | 49 / 70                                          | 1.37         | 0.227                    |
| jt  | flying_trot | 0.85                  | 0.75             | not run                 | 22 / 22                                          | 1.86         | 0.177                    |
| jt  | bound       | 0.85                  | 0 (fails at 0.5) | not run                 | - / 80                                           | -            | -                        |

All gaits are at their built-in period / duty / apex (`gait.GAITS`); `v_try` rungs are 0.25 m/s, so a value is good to one rung, and a rung
is one run: the held-out spreads (1.75-2.75) are the honest uncertainty. The tests in `test_dog_topspeed.py` fly the flying trot at
1.9 m/s (`FLOOR`; the plan rule min(2.0, floor(0.85 x 2.25, 0.1))) with the WBC at stand height 0.85 and J^T at 0.95.

**What limits the speed, measured.**

- **The stand height was the first limit of the WBC (root cause found, mechanism verified from a trace).** At the shipped `H_FRAC = 0.95` the leg is
  95 % extended while standing, so a stance foot that moves 12 cm from under the hip is straight (hip-pitch-to-foot distance 0.95 of the straight
  leg, so the horizontal reach is sqrt(1 - 0.95^2) = 0.31 of it). Trace of a WBC flying trot at 1.25 m/s (`dogtop`, seed 1001): the rear-right
  stance leg's knee goes from -0.6 to +0.1 rad (straight and past it) at the end of stance, the swing IK then picks the other elbow branch
  (swing target knee +0.4 rad against a current -0.6 / 0.1), `v_des` pegs at 40 rad/s and tau_ff at 44 N*m, the hip is driven to -1.8 rad, the pack
  current goes to 120 A, tilt grows, and the controller declares the fall at 3.9 s. Setting the stand height fraction to 0.85 (one change, `ctl.h_frac`)
  takes the WBC from 0.75-1.0 m/s to 2.0-2.25 m/s (flying trot: the 1.25 m/s rung fails at 112 A at 0.95, the 1.5 m/s rung passes at 42 A at 0.85). The cost is swing clearance: the knee stop leaves
  (H_FRAC - 0.79) x 0.465 m under the hip, 2.8 cm at 0.85 against 7.4 cm at 0.95 (the comment at `H_FRAC`). So `H_FRAC` was **not changed**:
  the terrain results of Tasks 8 and 9 (15 deg ramp, push envelope) were recorded at 0.95 and the 15 cm steps need the clearance. Recommendation: a
  stand height scheduled by the speed command (0.95 below about 1 m/s, 0.85 above), with a terrain regression; it is an SP4 follow-up, not done here.
  The same cause is probably behind the Task 8 observation of stance legs at the end of their workspace, but 0.85 did not rescue the 25 deg ramp
  (both fell at the ramp foot) or rubble seeds 1 and 9 at 1.0 m/s (fell), so it is not the only one there.
- **Beyond that, the WBC's failures coincide with the pack current**: every failing rung of the WBC at speed (the falls) shows a pack peak of
  112-147 A, against 120 A on the BMS (the sim has no BMS cut-off, so the run is not stopped by it; real hardware would brown out or trip at the first
  of these). J^T's failing rungs are lower (70-107 A; its trot at 1.5 / 1.75 m/s only reaches 0.94 of the command, no fall). At the highest ok rung the
  peak is 28-59 A (WBC), 49-58 A (J^T). Whether the current is the cause of the fall or its consequence (a leg thrown about) was not separated.
- Hip torque at its limit 19-26 % of ticks, knee 5-6 % at the first failing rung (WBC), 3-11 % (J^T), 0 % at the ok rungs: the 22 / 44 N*m limits
  are reached when the run is already failing, not before.
- Joint speed: the 99th percentile of the worst joint is 1.0-1.1 x the estimated no-load speed (V_pack / (ke N), 23 rad/s) in EVERY run, also at 0.5 m/s
  and in the ok rungs, so this statistic does not discriminate and is not used as a finding (it may be touchdown spikes or the estimate; not investigated).
- Slips: 0 in every ladder run.
- **Bound**: not viable with the current controller. At its default parameters the WBC bound manages 0.5 m/s with a pack peak of 122 A, cost of
  transport 10.2 and force RMS 0.72 m g; at stand height 0.85 neither controller completes a 0.5 m/s rung. It was not tuned (no search).
- The old finding of Task 8 ("swing feed-forward saturates at about 1.3 m/s") is superseded: it was this reach limit.

**Pack-current peaks per gait** (both packs summed, whole run incl. stand-up and launch; the contract keeps the 120 A BMS limit): WBC trot 52 A at
0.75 m/s (default height) and 28 A at 2.0 m/s (0.85); WBC flying trot 59 A at 1.0 m/s (0.95) and 42 A at 2.25 m/s (0.85); J^T trot 71 A at 1.25 m/s;
J^T flying trot 58 A at 2.5 m/s; bound 122 A at 0.5 m/s (an ok rung, over the 120 A limit: the bound is not acceptable on that ground alone). In the ok
rungs of trot and flying trot the 120 A limit is not reached; it is exceeded in the WBC's failing rungs. The measured joint torque (`tau_est`) exceeded the
22 N*m `tau_ff_max` in failing runs (up to 31.5 N*m at a hip: the PD term adds to the feed-forward); the actuator `i_limit` compliance was not audited
separately.

**Deviations (Task 10).** (1) No `--search` over the plan's GRID was run (coordinator: root-cause, no parameter sweeps); the CLI implements it
(`--search`, untested at scale). The search that was run is the two stand-height candidates 0.95 / 0.85 at the built-in gait parameters. (2) The
ladder starts at 0.5 m/s in 0.25 steps (plan: 1.5, 1.75, ...; the flat WBC fails there at the default height). (3) `sweep_gait` flies a short
ladder (1.0, 1.5, 2.0, 2.5 m/s, flying trot, WBC, height 0.85, one run per rung, stop at the first failure) instead of one 10 s ramp: a ramp to
4 m/s ends in a fall and a "speed of the last 3 s" of 0. (4) `tools/dog_sweep.py` got the `{**report, "xml": ...}` call as specified and, in addition, a
measured-gait column and conditional wording in `write_md` (otherwise the filled column is not shown and the report would still say that no gait speed
is measured). (5) `GAITS` values and `v_max` of the gaits are unchanged (no gait parameters were tuned; the default `choose()` therefore still switches
to the flying trot at 2.2 m/s, which the default stand height cannot reach). (6) `dogwalk.setup` takes an optional `sim` / `inf`, the controller has
`forced_gait`.

**Sweep (SP3b slot).** `docs/dog/speed-sweep.md` / `.json` have the measured `v_top` column: 1.5-2.5 m/s for all 12 points (GIM8108-8 and the GIM6010-8
probe alike), against a derived `v_trot_bound` of 0-1.36 m/s. The derived bound is not a bound of this controller. The sweep's chosen point (GIM8108-8
r2.0 L200) is unchanged and the hardware is unchanged. The probe having the same rungs as the 8108 means the flat speed up to 2.5 m/s is not set by the
hip stall torque (11 vs 22 N*m) or the no-load speed (61 vs 23 rad/s): by the controller. One seed per point, rung-limited, no ranking intended.

**Options for the 1.25-1.5 m/s gap, none adopted here** (cost / evidence): speed-scheduled stand height (software; the evidence above: +1.25 m/s for the
WBC); a swing IK that cannot flip the elbow branch (software; prevents the whip-through when a stance leg reaches full extension); find why J^T
at the default height reaches 2.5 m/s where the WBC fails at 1.0 (J^T does not command the stance leg's tau_ff from the QP; WBC retuning only if its
force tracking is what fails: it is not, force RMS 0.13-0.19 m g at the top rungs); a higher bus voltage within the drivers' rating (hardware: raises
the no-load speed, but the speed was not what failed); the other sweep points (knee ratio 2.0, leg 0.25 m: the measured column does not rank them); the AK70-9
class (actuator study: 2.8 m/s trot bound, +2.2 k USD; not measured here).

## Monte Carlo (Task 11)

`examples/dog/dogmc.py`: per run a new 30 x 30 m field of the family (terrain seed = run seed; criterion: the fixed course), a ground-truth A*
route (15-20 m asked, `dogcourse.route`; ruins routes came out at about 38 m), plant dispersion the controller does not know (payload 0-1 kg at
+-3 / +-2 cm, IMU gyro bias N(0, 0.005 rad/s) and accel bias N(0, 0.05 m/s^2), two lateral pushes of 0.2-0.5 m/s at random times in 4-12 s), pose-fix
noise, lidar off. Verdicts from ground truth (`dogtruth`), never from the controller's fall detector. Commanded speeds: rubble 1.8, slopes 1.0, scree
1.2, ruins 1.2, criterion 0.8 m/s. Code revision `136d4e7` + the uncommitted `dogmc.py` (the commit of this task). Held-out seeds 21000...,
flown once with the final code; **nothing was tuned, so the tuning block (seeds 1000...) was not used for tuning** (it was used for one stand-height
A/B below).

**n per family.** WBC: n = 20 per family (100 runs, 58 min at 4 parallel jobs; real-time factor 0.33-0.73 sim-s per wall-s per job, longest the ruins
routes). J^T baseline: n = 10 per family. The plan's n = 100 / 30 were not run: at this speed n = 100 per family is about 4.5 h of the shared box
with 4 jobs, and the result could not have moved (no run in 100 passes any criterion but 2). With n = 20 the best possible no-fall lower bound is
84 % (20 of 20), with n = 10 it is 72 %: **the 90 % criterion cannot be demonstrated with these n even by a perfect controller**; the Wilson
bound is reported next to the rate as the spec asks.

WBC (default stand height 0.95; `build/dog-mc` = `locomotion-mc.md`), held-out seeds 21000-21019:

| family    | n                            | success k/n (Wilson 95 %) | no-fall k/n (Wilson 95 %) | no-fall lower bound | mean / median speed of successful runs m/s | failures           | slips | MPC failures |
| --------- | ---------------------------- | ------------------------- | ------------------------- | ------------------- | ------------------------------------------ | ------------------ | ----- | ------------ |
| rubble    | 20                           | 0/20 (0-16 %)             | 0/20 (0-16 %)             | 0 %                 | -                                          | fall 20            | 0     | 0            |
| slopes    | 20                           | 0/20 (0-16 %)             | 1/20 (1-24 %)             | 1 %                 | -                                          | fall 19, timeout 1 | 0     | 0            |
| scree     | 20                           | 2/20 (3-30 %)             | 2/20 (3-30 %)             | 3 %                 | 1.01 / 1.01                                | fall 18            | 1     | 0            |
| ruins     | 19 (+1 seed without a route) | 0/19 (0-17 %)             | 0/19 (0-17 %)             | 0 %                 | -                                          | fall 19            | 1     | 0            |
| criterion | 20                           | 0/20 (0-16 %)             | 0/20 (0-16 %)             | 0 %                 | -                                          | fall 20            | 0     | 0            |

J^T baseline (same seeds 21000-21009, `build/dog-mc-jt`):

| family    | n   | success k/n (Wilson 95 %) | no-fall k/n (Wilson 95 %) | no-fall lower bound | mean / median speed of successful runs m/s | failures          | slips | MPC failures |
| --------- | --- | ------------------------- | ------------------------- | ------------------- | ------------------------------------------ | ----------------- | ----- | ------------ |
| rubble    | 10  | 0/10 (0-28 %)             | 0/10 (0-28 %)             | 0 %                 | -                                          | fall 10           | 5     | 0            |
| slopes    | 10  | 0/10 (0-28 %)             | 1/10 (2-40 %)             | 2 %                 | -                                          | fall 9, timeout 1 | 0     | 0            |
| scree     | 10  | 2/10 (6-51 %)             | 2/10 (6-51 %)             | 6 %                 | 0.98 / 0.98                                | fall 8            | 6     | 0            |
| ruins     | 10  | 0/10 (0-28 %)             | 0/10 (0-28 %)             | 0 %                 | -                                          | fall 10           | 0     | 0            |
| criterion | 10  | 0/10 (0-28 %)             | 0/10 (0-28 %)             | 0 %                 | -                                          | fall 10           | 1     | 0            |

The two controllers fail alike; neither is a usable terrain walker in this MC. Every run that did not succeed is a fall by ground truth (one slopes
run timed out upright). The 2 successful scree runs are seeds 21001 and 21009; their speed is 1.0 m/s against the commanded 1.2.

**Where the failures come from (diagnostics, held-out seeds, WBC).** (1) _Terrain alone_: the same fields and routes without any dispersion
(`--over '{"no_disp": 1}'`, n = 10 per family, `build/dog-mc-nodisp`): rubble 0/10, slopes 0/10 (no-fall 1/10), scree 3/10, ruins 0/10, criterion 0/10.
The terrain results of Task 8 stand: the dog does not traverse these fields with or without dispersion. (2) _Dispersion alone is also not
survivable_ (flat ground, 1.0 m/s, 14 s, an ad-hoc harness script that is not committed, seeds 21000-21007 with the MC's draw): 7 of 8 fell. Ablated on seeds 21000-21003: the two
pushes alone: 1 of 4 stayed up (falls with pushes of 0.48 m/s at 7.6 s and 0.45 m/s at 11.6 s on seed 21000, 0.46 m/s at 6.0 s and 0.34 m/s at 7.0 s on
21001, 0.27 m/s at 10.4 s and 0.4 m/s at 11.7 s on 21003); IMU biases alone: 4 of 4 up; a 1.0 kg payload at +-3 / +-2 cm alone (seed 21001, payload 1.00 kg): fell, the other three payloads
(0.03-0.85 kg) stayed up. Two caveats on the Task 9 push envelope follow: it was measured at one gait phase only (a push at exactly 5.0 s, 15.6 gait
periods in, so essentially one gait phase for all seeds), and the MC pushes arrive at random phases and the second one may land during the recovery of the
first. The 0.5 m/s "recovered 5/5" is therefore a value at one phase, not an envelope (the phase dependence was not measured). Not investigated further: which phase is worst, and why a 1 kg payload
(9 % of the mass, 3 cm offset, unmodelled by the controller) topples the trot.
(3) _Stand height_: a paired A/B of the stand height fraction 0.95 (default) vs 0.85 on the tuning seeds 1000-1004 of each family (25 runs each):
0/25 successes and 0/25 no-fall at both. The stand height that lifts the flat top speed (Task 10) does not make the terrain pass.

**Compute: Python vs C++.** WBC solve on a trot pose: Python 9.21 ms median, C++ 0.26 ms median (budget 1.0 ms at 500 Hz): **C++** (the `loco_wbc_cpp`
module, built by `cmake -DNAPL_LOCO_CPP=ON`; the Python QP stays as the fallback and the reference). Everything in this MC ran the C++ WBC. The MPC
(OSQP, 50 Hz) stays Python: median 5.2 ms unloaded (13.9-19.7 ms on a loaded hfbox, where its timing test fails; also at HEAD before this work).

## LOCO fix round 1 (after Task 11)

**MPC budget.** Condensed MPC (6 ms median, then 4.4 ms after replacing `expm` by its exact 3-term series and pinning BLAS threads in
`loco/__init__.py`; pinned by `test_dog_mpc.py` in fresh interpreters with the thread env unset and set to 64).

**Flat ground, pushes: root cause found.** A 0.4 m/s lateral push at 8 gait phases x 3 seeds (seeds 21000-21002, trot 1 m/s, no payload/bias):
WBC stayed up in 5/24, the J^T leg controller in 21/24. Swapping one WBC output at a time for J^T's: stance torque -> 24/24, stance velocity target
-> 0/24, both -> 1/24. So the WBC's own stance joint torque was the cause. Fix (`WBC_STANCE_JT`): stance joints take `J^T(-R^T f_mpc)` plus the
leg's required torque; the WBC keeps swing torques and all velocity targets. Full MC dispersion (payload, IMU bias, two pushes, 1.0 m/s, 14 s,
seeds 21000-21015, `dogflat.py`): WBC 9/16 -> 13/16 up (J^T 13/16). The 3 remaining falls follow a double push of 0.4-0.5 m/s within 1-2 s.
The old finding "pushes survived to 0.5 m/s" held at one gait phase only; the phase sweep showed 19/24 falls of the old WBC. `POSTURE` is 0.7
(0.3 and 0.5 fall at the ramp foot with the J^T stance torque; 0.7 and 0.9 climb the 15 deg ramp).

Levers tried and NOT adopted (counts of runs staying up): trot duty 0.5 / 0.54 / 0.58 / 0.60 -> 5 / 7 / 17 / 13 of 24 (0.62, 0.66 fall undisturbed),
but on the full dispersion 0.58 gave 1/8 vs 5/8; capture-gain boost 13/24 alone, 9/24 with duty 0.58; contact-held stance 0/8 and 5/8; gyro-bias
integrator in the attitude filter (tilt error 2.2 -> 0.8 deg, no MC gain); velocity-jump covariance inflation 6/24 and 5/16 (worse); `MU_NOM` 0.7
10/16 vs 9/16 (not significant). Single runs are chaotic: compare at n >= 16 only.

**Speed.** `H_SCHED`: stand height fraction 0.95 -> 0.85 between 1.0 and 1.8 m/s commanded (quantised 0.01, slewed 0.05 /s). Flat auto gait, 4 seeds, no
dispersion: constant 0.95 at 1.5 m/s 0/4 up; scheduled 1.5 m/s 4/4, 1.8 m/s 4/4, 2.0 m/s 1/4. Swing clearance at 0.85 is 2.8 cm (7.4 cm at
0.95), below the 6 cm asked for. `dogtop` with the J^T stance torque (forced gaits, held-out n = 5, 0.95 / 0.90 / 0.85): v_top 1.25 / 1.75 / 2.25 m/s
(trot and flying trot alike; held-out median 1.25 / 2.0 / 2.0). **Flat 3.5 m/s is NOT met (about 2.0-2.25 m/s).** Failing rungs end in a velocity
shortfall (v_mean < 0.95 v_try) or a fall; the pack peak at the failing 0.85 rungs is 114-142 A, over the 120 A BMS limit. Tried without effect:
`D_MAX` 0.23 (speed is not stride-limited), a velocity-reference integral trim.

**Why J^T beat the WBC (answered).** The WBC's stance torque, not its force plan: with J^T stance torque the WBC matches J^T on pushes (above)
and reaches 2.25 m/s at 0.85 (J^T's best in the older table: 2.5 m/s, flying trot at 0.95; not re-measured after this round).

**Terrain families: no lever landed.** Training MC (10 seeds per family, 1000-1009, dispersion on): rubble 0/10, slopes 0/10, scree 1/10,
ruins 0/10, criterion 0/6. Classified so far, from traces:

- _slopes_: all 10 seeds fall within 2-4 s of the START: the harness drops the dog level at the highest ground point +1 cm on 8-29 deg ground
  (20-40 cm height range under the footprint); even 6-9 deg starts fall. A start filter finds no route (the family has almost no flat ground).
  A harness change to set the dog down on the fitted plane, body z along the normal, with the estimator initialised to that attitude, was built
  and measured on seeds 1000-1007: still 8/8 falls inside 2 s (tilt 33-176 deg), so it was reverted. Standing on a 10-30 deg slope is outside
  what the controller does; the 25 deg ramp xfail is the same limit.
- _ruins_ (seed 1000): trot is clean to x = -9.8, then a step down/up gives a flight phase (no foot in contact, 0.4 s), tilt 14-19 deg, the
  constraint relaxation fires 579 times, and the dog falls at 8-9 s. No swing apex or foothold adaptation to local height exists.
- _rubble_, _scree_ seed 1000 are clean to 9 s / 13 s at the current code (fell at 4-12 s before the speed schedule and J^T stance torque:
  the single-seed outcome changed, the family result did not).
  Levers still open (not built; each is a feature, not a tune): foothold selection on the height map, swing apex from local terrain height,
  body pitch to the slope, a step-up trajectory, the walk gait on rough ground (its flat-ground fall has no cause yet).

**Final MC, fresh held-out seeds 31000-31039** (n = 40 per family, WBC with J^T stance torque, code `e9cc4f1`, `build/mc-final`, 200 runs, about
2 h at 4 jobs; `dogmc.TEST_BASE` moved from 21000 to 31000 because 21000-21015 were used for the flat tuning):

| family    | n   | success k/n (Wilson 95 %) | no-fall k/n (Wilson 95 %) | no-fall lower bound | mean speed of successful runs m/s |
| --------- | --- | ------------------------- | ------------------------- | ------------------- | --------------------------------- |
| rubble    | 40  | 0/40 (0-9 %)              | 0/40 (0-9 %)              | 0 %                 | -                                 |
| slopes    | 40  | 0/40 (0-9 %)              | 0/40 (0-9 %)              | 0 %                 | -                                 |
| scree     | 40  | 2/40 (1-17 %)             | 2/40 (1-17 %)             | 1 %                 | 1.06                              |
| ruins     | 40  | 1/40 (0-13 %)             | 1/40 (0-13 %)             | 0 %                 | 1.03                              |
| criterion | 40  | 0/40 (0-9 %)              | 0/40 (0-9 %)              | 0 %                 | -                                 |

3 successes of 200 (previous round: 2 of 100). The terrain families are unchanged by this round. Flat ground with the same dispersion:
13/16 up for 14 s (Wilson 95 %: 57-93 %), also below the 90 % target.

**SP5 on the dog** (`coverage_smoke_dog` rubble training seeds 1-8, 0.4 m/s / 0.3 rad/s, C++ WBC, cap 120 s): falls at 33 / 4 / 4 / 47 / 10 / (up at
120 s) / 26 / 69 s, 7 of 8 (before this round 31 / 4 / 116 / 51 / 9 / 74 / 26 / 11 s, 8 of 8). Within noise of the same locomotion limit. `dog_smoke`
golden re-recorded (`27ffca55529022aa`): the J^T stance torque, POSTURE 0.7 and the stand-height schedule change the trajectory.

**Recommendations (not decisions).** (1) 90 % no-fall on rubble / slopes / scree / ruins / criterion needs terrain features that do not exist yet
(foothold selection, swing height from the map, slope-attitude control, step-up); the spec numbers (3.5 m/s flat, 1.5 m/s rocky, 25 deg, 15 cm,
90 %) are not reachable by tuning of the present controller. (2) 3.5 m/s needs more than the 0.85 stand height: the pack hits 114-142 A at 2.5 m/s, so
either a higher-voltage / second-class actuator pack or a lower spec (about 2 m/s is what is demonstrated). (3) Decide whether slope starts are
in scope; a standing start on 10-30 deg ground is a different task from walking up it.

## LOCO fix round 2 (terrain)

Branch `p12-loco`. Training seeds 1000-1009 (n = 10 per family, WBC, full dispersion) for every lever, then one fresh test run, seeds 41000-41039.

**Result (n = 40 per family, seeds 41000+, WBC, Wilson 95 %; round 1 on 31000-31039 in brackets).** No-fall: rubble 0/40 (0-9 %) [0], slopes 0/40
(0-9 %) [0], scree 10/40 (14-40 %) [2], ruins 15/39 (25-54 %, one seed had no route) [1], criterion 0/40 (0-9 %) [0]. Scree and ruins improved
(real, intervals no longer overlap round 1); rubble, slopes and the criterion course did not move. The 90 % target is not met in any family.

**Lever 0 (the one that worked): the tilt correction was paused forever.** `controller._slew` paused the Mahony tilt correction (`t_acc`) whenever the
commanded velocity changed. A route follower changes the command every tick, so the correction never ran, and a 0.005 rad/s gyro bias walked the
pitch estimate 3 deg off in 6 s: the dog fell on FLAT ground (flat floor + rubble route follower + MC dispersion, 0.8 m/s, seeds 1000-1009: 5/10 up at
25 s; nogyro / nobias runs passed, so the bias and payload, not pushes, were the killer). Now the pause applies only while the low-passed commanded
acceleration exceeds `A_HOLD` = 0.3 m/s^2 (`TAU_ACC` 0.2 s): 10/10 up. Lever 0b: a Mahony integral term (`KI_ACC` 0.2, `B_MAX` 0.05 rad/s) estimates the
gyro bias (flat 1.8 m/s route following with bias: 3/8 -> 4/8 up). It overshoots a static 10 deg tilt to 11.2 deg at 5 s and settles, so
`test_attitude_converges_to_a_static_tilt` runs 12 s. Also in this commit, from the predecessor's diagnosis: `K_YAW` 0 (a yaw sinusoid of 0.5 rad/s
on flat ground: K_YAW 0.5 0/8 up, 0.2 2/8, 0 8/8), the turn envelope `A_LAT` 0.6 m/s^2 / `WZ_MAX` 1.2 rad/s (v x wz: 0.6 8/8 up, 0.81 6/8, 0.96 4/8,
1.44 0/8), a median instead of a minimum for `h_map` (false FALLEN on ruins), and the dogtop target of 2.0 m/s.

Training counts (rubble / slopes / scree / ruins / criterion, no-fall of 10): inherited tree 0/0/2/1/0; lever 0 0/0/3/4/0; lever 0b 0/0/5/3/0.

**Levers tried that did NOT move the counts (kept out of the tree).**

- Slope starts (lever 3): dog set down on the fitted plane with the trunk along the normal, estimator attitude initialised from the static
  accelerometer, STANDUP of 0.4-0.8 s: slopes still 0/10 (standing start on 17-31 deg slopes slides or rolls during STANDUP; with T_STANDUP 0.4 one
  seed stayed up 6 s, three others fell). Slope starts look out of reach of this controller; the criterion ramp (25 deg) also fails at the ramp foot
  (t = 7.9 s).
- `LATE_REACH` = True (keep reaching down when a stance is scheduled without contact), 8 rubble seeds: all fall at 4-5 s as before.
- Friction override mu 1.5: no gain. Flat or zero controller height map on the real terrain: no gain, so the controller's own terrain code is not
  the cause. Ablating foothold, apex, normal, reach and roughness code (predecessor): no gain.
- Stand height down earlier (`H_V0` 0.3) on the 5 scree seeds that fall: 0/5 up (baseline 1/5, all within noise).
- Swing reach fraction 0.97 -> 0.92 / 0.88 (`REACH_FRAC` experiment): single runs 1/4 and 1/4 vs 0/4, and the training MC at 0.92 gave exactly the
  lever-0b counts (0/0/5/3/0): no gain, reverted.
- Terrain code of Task 8 (`Tr.foothold`, `apex_over`, `plane`, `normal`, `rough_ahead`, `steep_ahead`) already does basic foothold selection and
  map-based swing apex; the round-1 statement that "none exists" is out of date. The proposed better apex (terrain envelope over a swath along the
  swing path, `apex_over` samples only 12 points on a line and one value at s = 0.5) was designed but not built: with the ablation above showing
  that removing the existing terrain code changes nothing, the limit is elsewhere.
- Step-up (lever 4): not built. The knee stop limits the lift under the hip to 7.4 cm at `H_FRAC` 0.95, so a 15 cm step needs the body raised first
  (`Z_RATE` 0.25 m/s). Rocks up to 17 cm exist in the rubble routes (the route allows 20 cm steps).
- Walk-gait flat fall (lever 5, `WALK_ROUGH` False): not investigated, no time left after levers 0-3.

**Remaining failure mechanism (observed, not fixed).** Rubble commanded 1.8 m/s: even on a FLAT floor with the route follower and dispersion 3-5 of 8
fall (4/8 up with the bias integrator); on the real rubble all 10 training seeds fall at 3-5 s, at 0.8 m/s 2/10 stay up (no rocks: 5/10). On scree the
5 failing seeds all stay up on a flat floor with the same route, so terrain matters there, and the falls are at near-flat spots (range 1-5 cm).
The traces (rubble 1007 rl at 3.1 s, scree 1003 rr at 2.5-2.8 s): a swing leg does not follow its reference (hip stalls, knee torque saturates), the
stance is scheduled with the foot still ~9 cm in the air, the MPC pushes 100-266 N through an airborne leg, then the leg re-enters swing with a huge
IK step (hip reference -0.9 ... -2 rad, velocity pegged at V_MAX = 40 rad/s, torque feed-forward saturated, knee near straight at the 0.95 stand
height). The listed option "a swing IK that cannot flip the elbow branch" and a swing target rate limit are the next things to try.

**Regressions checked.** Flat dispersion MC `dogflat.py` seeds 21000-21015, WBC: 13/16 up (unchanged). Flat top speed 2.0 m/s: `test_dog_topspeed` passes.
Goldens re-recorded (one reason: the tilt correction no longer stops, plus the bias integrator, change every trajectory): push `d63fd6b4e3399485`,
trot `3956925a79494e9d`, dog_smoke `9eb34232ef72640e`.

**SP5 on the dog** (`coverage_smoke_dog` rubble seeds 1-8, C++ WBC, 120 s cap, `[dog]` limits overridden in-process, `autonomy.toml` untouched). 0.4 m/s / 0.3
rad/s: 2/8 reach 120 s (seeds 2, 3: coverage 0.92 / 0.95), the others end at 30 / 40 / 9 / 23 (truth fall) / 76 (truth fall) / 21 s (before: 1/8 up, the
others at 4-69 s). Four of the six early ends (seeds 1, 4, 5, 8) are the controller's own FALLEN latch with the
truth never falling, which aborts the mission (seeds 4, 5, 8 re-run with a probe: estimated tilt 25-26 deg while rolling at 4-6 rad/s; seed 1 not probed). 0.8 m/s / 0.5 rad/s: 0/8 up, aborts at 18 / 4 / 14 / 24 / 7 / 38 / 15 / 31 s (6 truth falls; round 1: 3-29 s).
The safe `[dog]` limits stay v_max 0.4 m/s, wz_max 0.3 rad/s (still only 2/8 on rubble); 0.8 / 0.5 is not safe.

## LOCO fix round 3 (terrain)

Branch `p12-loco`. Training seeds 1000-1009 (n = 10 per family) for every lever; fresh test seeds 61000-61039 (n = 40, WBC, Wilson 95 %).

**Result (n = 40 per family, seeds 61000+; round 2 on 41000+ in brackets).** No-fall: rubble 12/40 (18-45 %) [0/40], slopes 0/40 (0-9 %) [0/40], scree 26/40
(50-78 %) [10/40], ruins 17/40 (29-58 %) [15/39], criterion 0/40 (0-9 %) [0/40]. Success (route end reached) equals no-fall in every family (no run timed out
upright). Mean speed of the successful runs 0.74-0.75 m/s. The 90 % target is not met in any family; rubble and scree improved, but these are runs at a LOWER
speed (below).

**The lever that moved the counts: a terrain speed cap (`V_TERRAIN` 0.8 m/s on any non-flat height map).** Not a better foot: slower. The MC commands rubble at
1.8 m/s; on a FLAT floor with the same route, route follower and dispersion the dog stays up 16/16 at 0.8 and 1.2 m/s, 7/8 at 1.5 and 4/8 at 1.8 (so 1.8 is
beyond the controller even without terrain). Rubble real terrain, training n = 10 no-fall (rubble / scree / ruins): inherited tree (commanded 1.8, 1.2, 1.2) 0 / 5 / 3;
cap 1.2: 4 / 5 / 3; cap 0.8: 4 / 8 / 5; cap 0.5: 6 / 9 / 4 (but 0 of the rubble runs reach the goal in time). Slopes (1.0 m/s command) and the criterion course (0.8) stay
0/10 at every cap. 0.8 was chosen: the most no-fall that still reaches the goal. Flat ground is untouched (the cap applies only when the height map is not flat). The spec
"rocky field mean >= 1.5 m/s" is therefore further away: the supported mean is 0.75 m/s. Side effect: `test_the_mud_strip_is_crossed_without_a_false_slip` runs the
slip course at 0.8 m/s for 40 s (it commanded 1.0).

**Lever 1, swing-IK guard (the round-2 failure: a lagging swing leg, stance scheduled with the foot ~9 cm up, a huge IK step on re-entry). Tried, no gain.**

- Touchdown-gated stance (`LATE_REACH` True, round 2): rubble 8 seeds all still fall at 4-5 s.
- Re-plan from the actual foot (swing foot reference never more than `SWING_ERR_MAX` from the measured foot, WBC path): 0.06 m: 8 single runs (4 rubble, 4 scree)
  all fall, scree earlier (3.4-5.0 s vs 3.4-13 s); 0.12 m: all fall, scree 3.3-6.8 s. Worse or equal; reverted.
- Swing reach fraction 0.97 -> 0.92 / 0.88 and an earlier stand-height drop (`H_V0` 0.3): within noise (round 2).
- A plane-fit relief governor (speed cap from the height spread of a 1.6 x 0.5 m swath ahead): the routes are mostly smooth (relief median 0.2 cm, 90th percentile
  2-3 cm; only 5-14 % of waypoints above 3 cm), so it never slowed the dog and rubble stayed 0/10; replaced by the plain cap. A fixed elbow branch was not built:
  the rate clip above already bounds the IK step and changed nothing.
- Dominant rubble fall cause, after this: speed, not terrain. The rubble falls occur on smooth cells (range 0-1 cm, riser 0.1 cm at the fall) 3-5 s after the
  start at 1.8 m/s, a rock of 11 cm sat 0.5 m to the side in the one case inspected (seed 1007). With rocks above 8 cm kept 0.3 m clear of the route (`max_step` 0.08
  instead of 0.20), 0.8 m/s single runs: 5/8 up (20 cm routes: round 2 2/10); at 1.2 m/s 0.08: 4/8, 0.20: 4/10; not adopted (not separable from noise at this n).
  Rubble at 1.8 m/s with 0.08 routes: 0/8. Runs without rocks (hfield only) at 0.8 m/s: 8/8 up for 20 s.

**FALLEN latch (estimated tilt 25 deg while rolling > 4 rad/s, or 40 deg, or the map height below half the stand height).** False positives against truth: 0, false
negatives: 0 on the 200 final runs (every run with a latch has a truth fall, every run without has none; 200/200). In the coverage smoke the latch ends the mission
without a truth fall because the safe-sit stops the run: that is not a false alarm. Latch ablation (tilt thresholds set to 90 deg so only the truth fall remains),
8 training runs (4 rubble, 4 scree): the truth fall happens at the same time as with the latch (scree 9.1 / 3.4 / 8.0 / 13.0 s vs 9.1 / 3.3 / 8.0 / 13.0; rubble
4.3-5.2 s unchanged): the dog is genuinely falling when the latch fires, so a "sustained tilt" rule would only delay the safe-sit. Not changed.

**Slopes and the criterion course: out of reach for this controller.** 0/40 and 0/40 again. Slope starts (round 2: a standing start on 17-31 deg ground falls in
under 2 s) and the 25 deg ramp (falls at the ramp foot) need a controller that leans and plants on the plane before it moves; the cap does not help (they are below it).
No concrete lever found in this round.

**Coverage smoke (`coverage_smoke_dog` rubble seeds 1-8, 120 s cap, `[dog]` limits overridden in-process, `autonomy.toml` untouched).** 0.4 m/s / 0.3 rad/s: 2/8 reach
120 s (seeds 2, 3), the others end at 30 / 40 / 9 / 23 / 76 / 21 s: identical to round 2 (the cap is above 0.4, so nothing changed). 0.4 did not improve, but 0.6 / 0.4 was also
run: 1/8 reach 120 s (seed 8, in the RETURN phase at the cap), others end at 23 / 4 / 82 / 22 / 7 / 16 / 18 s. Safe limits stay `[dog]` v_max 0.4, wz_max 0.3.

**Regressions.** Flat dispersion MC `dogflat.py` seeds 21000-21015, WBC: 13/16 up (unchanged). Flat 2.0 m/s: `test_dog_topspeed` passes. No golden changed (flat runs and
the 0.4 m/s dog_smoke are below the cap). Targeted loco tests: 62 passed, 1 skipped, 2 xfailed after fixing the mud-strip test (9/9 in `test_dog_safety.py`).

## Spec criteria (quadruped design s7)

| criterion                                            | measured                                                                                                              | verdict                                    |
| ---------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| flat top speed >= 2.0 m/s (was 3.5; user 2026-10-18) | WBC (J^T stance torque, h 0.85) 2.0 m/s held-out median and minimum, 2.25 on the training seed; LOCO fix round 1      | **met**, no margin (3.5: gap 1.25-1.5 m/s) |
| rocky field mean speed >= 1.5 m/s                    | round 3, terrain speed cap 0.8 m/s: rubble 12/40 reach the goal, mean 0.74 m/s (command 1.8 is beyond the controller) | **NOT met**                                |
| 25-degree slopes and 15 cm steps traversed           | criterion course 0/20 (WBC), 0/10 (J^T); the 15 deg ramp is climbed (Task 8 test); 25 deg and 15 cm steps are not     | **NOT met**                                |
| >= 90 % no-fall per terrain family, 95 % lower bound | no-fall lower bounds (WBC, n = 40, 61000-61039, round 3): rubble 18 %, slopes 0 %, scree 50 %, ruins 29 %, crit. 0 %  | **NOT met** in every family                |
| BOM ~ 3-4.5 k USD                                    | 5,554 USD (SP3b, unchanged)                                                                                           | outside the plan's range (SP3b)            |

(The coverage criterion is SP5's.)

## Known gaps

- **No self-righting**: a dog that falls stays down (Task 9); the MC counts the fall as the failure.
- **Terrain** (round 3, n = 40, seeds 61000+: rubble 12, slopes 0, scree 26, ruins 17, criterion 0 no-fall, at the 0.8 m/s terrain speed cap; "LOCO fix round 3"): the 25 deg ramp, 15 cm steps, rubble, slopes, scree and ruins families are not reliably traversed; the walk gait falls on flat
  ground and is disabled (`WALK_ROUGH = False`), so rough ground is trotted with a 0.5 m/s cap; causes in the "Terrain (Task 8)" section.
- **Disturbance rejection**: after the J^T stance torque fix the flat dispersion MC stays up 13/16 (was 9/16); double pushes of 0.4-0.5 m/s still topple it. The slip handling is validated on a synthetic statistic and a mud strip, not on a slide the dog recovers from.
- **Stand height**: now scheduled (`H_SCHED`, see "LOCO fix round 1"); swing clearance is 2.8 cm at speed; terrain regression of the schedule was
  the existing terrain tests only (they pass), not a terrain MC at speed.
- **J^T beat the WBC at speed** because of the WBC's stance torque, fixed in LOCO fix round 1; the WBC is the design's leg controller.
- **Bound gait** is not viable with the current controller; the flying trot gains little over the trot (2.0 vs 2.25 m/s).
- **SP5 stand-ins**: the height map is the ground truth, the pose fix is truth plus noise, routes are ground-truth A* (SP5 plans its own); the lidar is off.
- **Inertia approximation** in the controller's model (point masses per link); **leg dynamics are not in the contact estimate**; the knee linkage ratio is
  constant 1.5 (SP3b measured 1.46-1.61).
- **MC scale**: 30 x 30 m fields (object counts scaled by (30/100)^2 from the 100 m generator), n = 20 (WBC) / 10 (J^T) per family, so the 90 % criterion
  is unprovable at this n; one seed in 100 had no route (ruins, 21015) and is not counted.
- **MPC timing test** `test_dog_mpc.py::test_solve_time_budget` passes on an idle box after the condensed rewrite (median about 6 ms); under heavy
  load (other jobs on hfbox) the median still rises above 8 ms.
- **Stand creep**: the four-foot stand walks each foot inward at 2.4 mm/s (4.8 mm/s of width), unchanged between the sparse and the condensed MPC and
  independent of the MPC lateral force (about 0.03 N), the stance joint damping (0.8 to 8 N m s/rad: same rate).
  Cause not found; rung b now pins the creep rate (< 10 mm/s of width) and the shove slide (< 3 cm) separately instead of one cumulative width.
