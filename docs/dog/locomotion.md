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

## Spec criteria (quadruped design s7)

| criterion                                            | measured                                                                                                              | verdict                         |
| ---------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------- |
| flat top speed >= 3.5 m/s                            | WBC flying trot 2.0 m/s (median and minimum of 5 held-out seeds; 2.25 on the training seed), J^T 2.25 median; Task 10 | **NOT met** (gap 1.25-1.5 m/s)  |
| rocky field mean speed >= 1.5 m/s                    | no rubble run succeeded (0/20), so there is no mean speed; the only successful MC runs (scree) averaged 1.0 m/s       | **NOT met**                     |
| 25-degree slopes and 15 cm steps traversed           | criterion course 0/20 (WBC), 0/10 (J^T); the 15 deg ramp is climbed (Task 8 test); 25 deg and 15 cm steps are not     | **NOT met**                     |
| >= 90 % no-fall per terrain family, 95 % lower bound | no-fall lower bounds (WBC, n = 20 / 19): rubble 0 %, slopes 1 %, scree 3 %, ruins 0 %, criterion 0 %                  | **NOT met** in every family     |
| BOM ~ 3-4.5 k USD                                    | 5,554 USD (SP3b, unchanged)                                                                                           | outside the plan's range (SP3b) |

(The coverage criterion is SP5's.)

## Known gaps

- **No self-righting**: a dog that falls stays down (Task 9); the MC counts the fall as the failure.
- **Terrain**: the 25 deg ramp, 15 cm steps, rubble, slopes, scree and ruins families are not traversed (0-10 % success); the walk gait falls on flat
  ground and is disabled (`WALK_ROUGH = False`), so rough ground is trotted with a 0.5 m/s cap; causes in the "Terrain (Task 8)" section.
- **Disturbance rejection is weaker than the Task 9 envelope suggests**: random-phase pushes of 0.2-0.5 m/s, and a 1 kg payload, topple the dog on flat ground
  (see "Where the failures come from"). The slip handling is validated on a synthetic statistic and a mud strip, not on a slide the dog recovers from.
- **Stand height**: the shipped `H_FRAC = 0.95` caps the WBC's flat speed near 1 m/s (0.85 gives 2.0 m/s); a speed-scheduled height with a terrain
  regression is the open follow-up (Task 10).
- **J^T beats the WBC at speed** (2.25 vs 2.0 m/s median) and the cause is not established; the WBC stays the design's leg controller.
- **Bound gait** is not viable with the current controller; the flying trot gains little over the trot (2.0 vs 2.25 m/s).
- **SP5 stand-ins**: the height map is the ground truth, the pose fix is truth plus noise, routes are ground-truth A* (SP5 plans its own); the lidar is off.
- **Inertia approximation** in the controller's model (point masses per link); **leg dynamics are not in the contact estimate**; the knee linkage ratio is
  constant 1.5 (SP3b measured 1.46-1.61).
- **MC scale**: 30 x 30 m fields (object counts scaled by (30/100)^2 from the 100 m generator), n = 20 (WBC) / 10 (J^T) per family, so the 90 % criterion
  is unprovable at this n; one seed in 100 had no route (ruins, 21015) and is not counted.
- **MPC timing test** `test_dog_mpc.py::test_solve_time_budget` fails when hfbox is loaded (13.9-19.7 ms vs 8 ms) and also at HEAD without these changes.
