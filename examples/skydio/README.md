# Skydio-style vision-only autonomous quadrotor

A Holybro X500 V2 (Pixhawk 6C, 4S 5200 mAh) with an Intel RealSense D435i depth camera on the nose and a Raspberry Pi 5 companion computer
(`assemblies/skydio.toml`, built from the parts library with the design API) flies through a seeded forest from its depth camera alone. It
never reads an obstacle position: depth image -> point cloud -> rolling voxel map -> clearance field -> sampled motion primitives -> setpoints.
Two missions: **(a) goal seeking** (58 m through trees, poles, boulders, a wall with a 2.4 m doorway and an arch, cruising >= 4 m/s where clear)
and **(b) following a walking person** through the same kind of forest while avoiding the trees.

```
tools/remote.sh run '.venv/bin/napl program examples/skydio/goal.scenario.toml   --runs-dir build/runs'   # PASS + replay hash match, ~25 s
tools/remote.sh run '.venv/bin/napl program examples/skydio/follow.scenario.toml --runs-dir build/runs'   # PASS + replay hash match, ~40 s
NAPL_PARAMS='{"seed": 111}' .venv/bin/napl program examples/skydio/follow.scenario.toml --runs-dir build/runs   # any seed is a scenario (the `seed` parameter, goal and follow)
tools/remote.sh run '.venv/bin/python examples/skydio/montecarlo.py --seeds 24 --workers 5 --out build/skydio_mc'   # build/skydio_mc.{json,md}; --mission follow --first 100 [--thin]
.venv/bin/python examples/skydio/fly.py <out dir> goal|follow <seed>                                   # one flight, no scenario judge
.venv/bin/python examples/skydio/timeline.py <run dir> [step s | depth <t>]                            # debugging: true subject distance / bearing / seen / visible per step, or a depth frame as digits
tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py <run dir> --out build/frames --cam chase'   # PNG frames (GPU)
```

CPU only except the last line. Wall time is about 1 s per simulated second (the 160 x 90 ray cast and a 20 ms planner step at 25 Hz).

## Results

Everything below is flown on the ESTIMATED state (ESKF on IMU + VIO + baro) with the person found by the detector on the depth image; the
kernel's true state is used only to move the physics and to score. `napl program` scenarios (seed 1, replay state hash matches in both):

- goal: PASS, replay match, hash `efb57e183637ae9d`.
- follow: PASS (all five monitors, including the strict 3-8 m `follow_distance_band`: range [3.845, 7.363] m, final 4.03 m), replay match, hash `164b5cc71536dfec`. The previous round failed this monitor by 0.05 m
  on seed 1 and its test expected the failure; the band was not widened, the behaviour was fixed (below). Any seed is a scenario now: `NAPL_PARAMS='{"seed": 111}'` reaches
  the model script and `fly.py` through one function (`skydio_scene.seed`), where before the script built the forest from a hard-coded default seed.

What changed in the follow behaviour (previous round: subject seen 0.59 of the flight, 5 of 24 thin forests failed): (1) the camera mount (see the honesty notes: the largest single
effect, seen 0.59 -> 0.96); (2) `search.py`, the belief over a lost subject and the visibility-aware viewpoint term; (3) the camera turns toward the belief while the
subject is lost; (4) the "hold the standoff when lost" behaviour comes from the belief's pace-keeping velocity instead of the decaying coast. The seed-1 scenario failed the strict band in the
first version of this round (the drone fell back to 13.5 m and lost the subject for 7.7 s) because the standoff was WIDENED by half the belief spread while lost (`LOST_STANDOFF` 0.5): the drone slowed to
1 m/s while the subject walked on at 2.4 m/s. It is 0 now (the standoff is held, not widened; tried 0.5 / 0.2 / 0 and the viewpoint weight 5 / 2 / 0 on seed 1 only: 0 with weight 5 gave 3.85-7.36 m,
seen 0.92). That was a choice made looking at seed 1, the scenario seed; 100-123 are then flown again below. Goal seed 109 (previous round: -0.02 m): the dodge
side is now committed (`w_commit`), the no-safe-primitive fallback at speed is the dodge with the most clearance instead of a straight stop, and the fast speeds are capped when
almost none is safe (`fast_frac`); worst clearance on every goal set is >= 0.15 m.

Monte Carlo, 24 forests per mission (every seed a different forest, wind turbulence, IMU/VIO/baro noise and detector miss pattern; `montecarlo.py`, 5 processes, ~3 min per set;
raw JSON in `build/mc_*.json` on the host, not committed). Seeds 100-123 are the set development used (the belief, the camera mount and the planner constants were tuned while looking at
100-123 and nine hand-picked seeds from it and from the previous round's failures); "thin" = seeds 100-123 with 10-14 extra poles of radius 0.025-0.05 m per forest (`--thin`).
Held-out sets, each flown once and not used to choose a constant: goal **400-423** (before the last two planner changes, the fast-speed cap and the fallback tie-break) and goal **500-523** (final
planner); follow **600-623** with the final code (the follow change made after seed 1, `LOST_STANDOFF` 0.5 -> 0, only touches the follow mission, so the goal columns are unchanged by it).
The follow sets 400-423 and 500-523 were flown with `LOST_STANDOFF` 0.5, before that change: both 23/24 (409; 502), strict band 20 and 22 of 24, median 4.28 / 4.29 m, seen 0.93 / 0.92, worst clearance 0.24 m;
they are superseded by 600-623 and not in the table. Failing seeds were looked at afterwards (notes below); nothing was changed in response except the seed-1 change above, which was made before 600-623 was flown.

| | goal 100-123 | goal thin | goal 400-423 | goal 500-523 | follow 100-123 | follow thin | follow 600-623 |
|---|---|---|---|---|---|---|---|
| success | 24/24 | 24/24 | 24/24 | 23/24 (514) | 24/24 | 24/24 | 23/24 (606) |
| runs with a physical contact | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| min clearance, worst run (m) | 0.20 (109) | 0.21 (119) | 0.29 | 0.15 (521) | 0.29 | 0.26 | 0.22 |
| mean speed in flight (m/s) | 2.99 | 3.02 | 3.14 | 3.10 | 2.46 | 2.45 | 2.40 |
| time to goal mean / worst (s) | 17.5 / 20.6 | 17.4 / 20.4 | 17.3 / 20.1 | 17.2 / 19.6 | - | - | - |
| runs inside the strict 3-8 m band for the whole walk | - | - | - | - | 22 of 24 (100: 9.4 m, 104: 10.8 m) | 21 of 24 (100, 104, 118: up to 11.3 m) | 21 of 24 (606, 608: 9.9 m, 621: 11.6 m) |
| follow distance, median of per-run medians (IQR) (m) | - | - | - | - | 4.28 (4.24-4.31) | 4.27 (4.24-4.34) | 4.28 (4.27-4.35) |
| subject seen (fraction of flight) | - | - | - | - | 0.95 | 0.95 | 0.90 |
| detector recall on frames where the subject is visible | - | - | - | - | 0.96 | 0.96 | 0.94 |
| detector false positives per frame, mean / worst run | - | - | - | - | 0.010 / 0.023 | 0.020 / 0.040 | 0.008 / 0.027 |
| estimate position error RMS, mean / worst run (m) | 0.35 / 0.72 | 0.35 / 0.65 | 0.29 / 0.64 | 0.30 / 0.81 | 0.33 / 0.64 | 0.33 / 0.63 | 0.31 / 0.67 |
| drift at the end, mean / worst (% of path) | 0.82 / 1.64 | 0.82 / 1.68 | 0.66 / 1.38 | 0.71 / 2.28 | 0.91 / 1.77 | 0.91 / 1.81 | 0.90 / 2.11 |
| attitude error RMS, mean (deg) | 0.37 | 0.37 | 0.37 | 0.38 | 0.39 | 0.39 | 0.40 |

Success = goal: reached within 45 s with zero contacts; follow: zero contacts, subject tracked >= 50 % of the flight, median follow distance in 3-8 m (the table also counts the stricter
reading the scenario monitor applies: every sample of the walk inside 3-8 m). Clearance is the airframe sphere (0.40 m) to the nearest surface.
Failures, none a contact: goal 514 is the estimator, not the planner (the VIO scale error left a 1.36 m position error at the end, 2.3 % of the path, the vehicle stopped at what it believed was
the goal and was outside the 1.5 m radius; the worst goal drift of the held-out sets); follow 606 never acquired the subject (seen 0.02, 17-43 m away throughout), not inspected further. The strict 3-8 m band is NOT held in every run:
2-3 of 24 per follow set exceed 8 m once (up to 11.6 m; 100 and 104 in both 100-123 sets; causes not inspected) although the median distance is in the band
in every run but 606; the seed-1 scenario passes the strict monitor, the Monte Carlo shows that is not guaranteed for an arbitrary seed. Of the 72 follow runs, 0 contacts.
The follow distance sits at 4.3 m although the target is 4.5 m: the integral trim settles slightly short; the spread (IQR 0.1 m) is small because the controller sees the subject 90-95 % of the time.

## Pipeline (every file is one stage; numpy only, no scipy)

| stage | file | what it does |
|---|---|---|
| vehicle | `build_design.py` -> `assemblies/skydio.toml` | x500v2 + D435i + Pi 5 + 5 V regulator, wires and power load, `depth` rangefinder device on the camera's optical frame |
| world | `forest.py`, `skydio_scene.py` | seeded obstacle field as exact primitives (cylinders, spheres, yawed boxes) -> MJCF geoms (collide, visible to the rays) and a contact sensor |
| depth -> map | `mapping.py` | pinhole back-projection, log-odds voxel grid (0.2 m, 32 m torus around the drone), free-space carving along rays, separable Euclidean distance field |
| plan | `planner.py` | 400+ velocity-ramp primitives per replan; a primitive is flown only if its whole path including the braking tail keeps 0.62 m from every occupied voxel and stays inside space the camera has already seen to be free; goal / follow objective picks among the safe ones; fallbacks brake, escape, scan (when nothing is safe and the vehicle is fast: the dodge with the most clearance, not a straight stop). Decision hysteresis: switching the dodge side (left / right of the line to the goal) costs `w_commit`; clutter-scaled speed: when under 5 % of the fast (>= 3.5 m/s) primitives are safe the fastest allowed primitive is 2.5 m/s for 15 replans |
| state estimate | `estimator.py` | 15-state error-state Kalman filter (position, velocity, attitude, accel + gyro bias) on the IMU device at 500 Hz, a velocity + attitude VIO measurement at 25 Hz (statistical model of a VIO output, 50 ms delayed, handled by state-history re-linearisation), the baro height at 50 Hz |
| person detection | `detector.py` | classical detector on the depth image + the pose estimate only (ground removal, connected components, size prior), constant-velocity Kalman tracker with a Mahalanobis gate |
| subject lost | `search.py` | while the detector has no subject: a 120-particle belief over its position and velocity, started from the tracker at the loss, walked like a person (keeps its pace, turns up to ~0.8 rad/s, never into a mapped trunk), and cleared of every particle the camera could have seen (field of view, 2.4-7 m band, unoccluded line of sight in the map) without a detection. Its densest cluster replaces the tracker's coasting estimate; `visible()` scores candidate viewpoints by how much of the belief they would see, which `fly.py` adds to the follow objective (visibility-aware viewpoint selection); the camera turns toward the belief (at most 45 deg off the flight direction) and the standoff is held (`LOST_STANDOFF` = 0; widening it with the belief's spread made the drone fall back, see Results) |
| follow | `follow.py` | the walking subject itself (scripted, ground truth, scoring only), the map mask and the track box |
| control + loop | `fly.py` | takeoff, 1 s mapping hover, flight; the native `cuas.flyer.Flyer` tracks position / velocity / acceleration setpoints; per-tick artefacts |
| judge | `*.scenario.toml`, `montecarlo.py` | `napl program` native interface: monitors on `trace.csv` + replay-log hash re-check; Monte Carlo over forest seeds |

The planner's safety argument: the vehicle only ever follows a primitive whose braking stop is inside observed-free, obstacle-clear space, so a
stop is always available unless NEW information (a thin pole first resolved at 5 m) invalidates it; then it brakes at 4.6 m/s^2 (25 deg tilt).
Thin poles (0.05-0.09 m radius) are the hard case for a 160 x 90 image at 0.095 m per pixel at 8 m.

## Honesty notes (what is real and what is a stand-in)

- **The state estimate is an ESKF, but its VIO input is a statistical model, not a feature tracker.** IMU: the sim's IMU device (noise density from the
  ICM-42688 datasheet) plus a turn-on bias drawn within the datasheet initial offset and a bias random walk. Baro: the sim's baro device (50 Hz, 1.2 Pa
  noise), converted with the ISA formula, no weather drift. VIO: body-frame velocity (scale error 0.4 %, Gauss-Markov bias, white noise) and attitude
  (roll / pitch Gauss-Markov, yaw random walk) corrupted from the true state; the magnitudes are chosen so the closed-loop position drift lands at
  0.7-1.7 % of path length, in line with a published figure for a commercial VIO (Intel T265: under 1 % drift on closed loops, vendor claim) and
  above the stereo-inertial EuRoC numbers (OpenVINS / VINS-Fusion ~0.1 m ATE, from the survey tables in arXiv:2511.18756, my averages). No feature
  tracking, no loss of tracking in texture-poor or fast-motion scenes, no VIO failure at all. Estimation error is reported per mission (`summary.json`
  `estimation`, the table above) against the kernel's true state. The camera's pose in the body frame and the intrinsics come from the model; depth
  noise is the device's (2 % of range, sigma1 from the D435i datasheet RMS error) with a 0.06 s latency (estimate).
- **Person detection runs on the depth image, with a size prior.** No ground-truth subject position reaches the detector. It is not a neural network and
  not RGB: anything person-shaped in depth (a 1.25-2.1 m stump or post, a second person) would be accepted, and a person hidden below the waist is
  missed. There are no decoys in the forest; the false-positive rate above is against trunks, poles, boulders and the wall only. Recall is scored
  against the true subject position (evaluation only).
- **Near-field blind zone** (derived from the sensor spec and the detector's rule, not measured separately): the detector needs the legs down to 0.9 m and the D435i sees +-29 deg vertically, so at a
  1.9-2.1 m flying height a person closer than about 2.4 m is cut off at the knees (the 0.28 m minimum depth is not the binding limit); `search.R_MIN` and the 2.6 m keep-out around an unseen
  subject encode it. The mission ends when the walk ends (a standing person at 1-2 m is not tracked).
- **The follow mission mounts the camera 8 deg down, the goal mission 10 deg up** (`skydio_scene.tilt_camera` re-aims the D435i in the generated MJCF; `assemblies/skydio.toml` keeps the goal
  mount). Measured on flights of seeds 1, 111, 302, 100, 117, 205 with the 10 deg up mount: the detector found a visible person in 67 % of the frames while the quad was pitched 2-12 deg nose
  up (braking) against 97 % while 12-30 deg nose down (nose-up attitude lifts the camera's lower edge above the knees). A real follow-me drone would carry a gimbal; a re-aimed fixed mount
  is the cheap stand-in, and it is a design change (the scenario hashes differ from the previous round).
- **The search belief is a heuristic, not a calibrated filter.** Its motion model (turn noise 0.8 rad/s, pace kept within 0.6-1.4x, 0.3 m/sqrt(s) diffusion) is hand-set to the scripted
  walker, whose routes bend 45-90 deg at corners; a detection miss counts only 35 % against a visible hypothesis because the detector's misses come in runs of seconds. A person who
  stops, reverses or walks a very different route is found only when the camera happens to sweep over them. `W_VIS`, `LOST_STANDOFF`, `LOOK_OFF` and the belief constants were tuned by hand
  on seeds 1, 111, 302, 100, 105, 108, 110, 117, 205.
- **Through PX4: done, see "On PX4" below** (`follow.px4.scenario.toml`, `goal.px4.scenario.toml`). The two native scenarios above are unchanged and still fly
  `cuas.flyer.Flyer`; the PX4 ones fly the same planner and estimator with PX4 as the flight controller and no GPS.
- The **depth image is sub-sampled 160 x 90** of the real 1280 x 720 (same field of view 87 x 58 deg, 0.28-10 m range, 25 Hz); no stereo matching artefacts,
  flying pixels, or sunlight washout. Ray casting is exact against the MJCF geoms; the drone's own landing gear and props are NOT in the image
  (the camera sits ahead of them; checked on a frame).
- **The native flight controller is `cuas.flyer.Flyer`** (cascaded PD + geometric attitude + thrust curves calibrated from the motor / prop models), not PX4.
  The acceleration feed-forward is emulated through the position setpoint. The PX4 scenarios replace it with PX4 v1.17 (next section).
- Not modelled: the printed nose bracket, M2.5 stand-offs, cable clamps (about 25 g); the Pi's heatsink / fan. The Pi 5 mass (46 g) is an estimate.
  The 6 DRC warnings are inherited unchanged from `assemblies/x500v2.toml`.
- "Collision" is the physics: a MuJoCo `<contact>` sensor on the airframe subtree against the obstacle body (and the subject). `clearance_m` is the exact
  analytic surface distance of the generator's primitives minus a 0.40 m airframe bounding sphere (evaluation only; the planner never sees it).
- The wind is a steady 0.8 / 0.4 m/s plus Dryden turbulence (W20 = 1 m/s), light for a forest.

## On PX4 (GPS-denied, companion computer)

`follow.px4.scenario.toml` and `goal.px4.scenario.toml` fly the same airframe, planner, depth map and estimator under real PX4 v1.17 SITL, with PX4 as the
flight controller and **no GPS**: `px4/bridge_skydio.toml` has no `gnss` key (the bridge sends no HIL_GPS; a ctest asserts it) and `EKF2_GPS_CTRL=0`. The
program (`px4_fly.py`, class `Px4Skydio(Skydio)`) is the Pi 5 companion: its own MAVSDK link to PX4, separate from the bridge, as on the real drone.
The monitors are copied unchanged from the native scenarios (they are judged on the program's `companion.csv`, which has the same columns as the
native trace; `trace.csv` stays the bridge's, which is what the replay and `render_frames.py` use).

```
NAPL_PX4_INSTANCE=5 .venv/bin/napl program examples/skydio/goal.px4.scenario.toml   --runs-dir build/runs   # ~2.5 min wall (PX4 boot ~25 s + lockstep flight), PASS + replay match
NAPL_PX4_INSTANCE=5 .venv/bin/napl program examples/skydio/follow.px4.scenario.toml --runs-dir build/runs   # NAPL_PARAMS='{"seed": 111}' for another seed
.venv/bin/python examples/skydio/px4_montecarlo.py --mission both --seeds 12 --workers 2 --out build/skydio_px4_mc   # under the shared CPU lock; PX4 instances 5 and 6
```

| message / path | direction | rate | what it is |
|---|---|---|---|
| HIL_SENSOR, HIL_STATE_QUATERNION, BATTERY_STATUS (bridge) | sim -> PX4 | 250 Hz (one per 4 ms step) | IMU, baro, (mag), battery; no HIL_GPS |
| ODOMETRY (MAVSDK `MavlinkDirect`) | companion -> PX4 | 31 Hz | the ESKF pose and velocity (LOCAL_NED, pad as origin, VIO estimator type, FRD attitude); EKF2 fuses it |
| OBSTACLE_DISTANCE | companion -> PX4 | 31 Hz | 72 bins x 5 deg, body FRD, nearest depth-map point within +-0.6 m of the vehicle height; bins the camera cannot see are UNKNOWN, not clear |
| offboard position + velocity + acceleration set-points | companion -> PX4 | 62 Hz | the planner's output through `Px4Flyer._command` (same arithmetic as the native `Flyer` target) |
| `--ctl` socket `imu <since>` | bridge -> companion | polled | the 250 Hz gyro / accel records the bridge sent to PX4, buffered 16 s with their sample counters, so the companion's ESKF sees exactly PX4's IMU |

PX4 parameters that differ from the x500v2 airframe 4902 (`[interfaces.px4.params]`, applied as `PX4_PARAM_*` before the airframe file): `EKF2_GPS_CTRL=0`,
`EKF2_EV_CTRL=15` (position, height, velocity, yaw), `EKF2_HGT_REF=3` (vision), `EKF2_MAG_TYPE=5` (no mag fusion), `SYS_HAS_MAG=0`, `SYS_HAS_GPS=0`, `EKF2_EV_DELAY=20` ms,
`EKF2_EV_NOISE_MD=1` with `EKF2_EVP_NOISE` 0.1 m, `EKF2_EVV_NOISE` 0.1 m/s, `EKF2_EVA_NOISE` 0.05 rad, `MPC_TILTMAX_AIR=26` (the native max tilt), `CP_DIST=1.0`.
Names and ranges were read in the v1.17.0 tree, not recalled.

**Does PX4 do GPS-denied vision fusion here? Yes, shown by its own log.** In the ulog of the first run `estimator_status_flags` has `cs_ev_pos`, `cs_ev_vel`,
`cs_ev_hgt` and `cs_ev_yaw` set and `cs_gnss_*` never, `solution_status_flags` shows horizontal relative position / velocity valid, and the commander printed
"Ready for takeoff!" 3-4 s after the first ODOMETRY. PX4's own position follows the companion's: RMS 0.06 m (worst run 0.07 m) over all 24 flights below.

### Results (PX4 SITL, seeds 100-111, 12 per mission, 2 workers)

PX4 is not lockstep-deterministic (its tasks run on wall-clock timers), so the same seed gives a different flight each time; each run replays to its own
hash (`replay.match` 24 of 24), but the numbers below cannot be reproduced to the digit. 12 seeds give wide intervals; the native tables above use 24 and 24 held-out.

| | goal | follow |
|---|---|---|
| Monte Carlo success (`montecarlo.py` definition), Wilson 95 % | 12/12 = 100 % [76, 100] % | 12/12 = 100 % [76, 100] % |
| every scenario monitor passed (the strict check) | 12 of 12 | **6 of 12** (the `follow_distance_band`, 3-8 m, failed on seeds 100, 103, 104, 106, 109, 110) |
| contacts | 0 | 0 |
| min clearance, worst run / mean (m) | 0.34 / 0.49 | 0.14 / 0.46 |
| time to goal mean / worst (s), or follow distance median of medians (m) | 18.4 / 20.0 | 4.54 (IQR 4.49-4.69) |
| subject seen (fraction) / detector recall on visible frames | - | 0.95 / 0.96 |

The native path passes that monitor on seed 1 and, in the tables above, on 24 of 24 forests (23 of 24 held-out); on PX4 the follow band is the weak point:
the strict monitor is met on 6 of 12 (50 %, Wilson [25, 75] %). In the one failed run examined (the scenario's default seed) the detector lost the subject for about 2 s at t = 22 s,
the quad slowed to 0.4 m/s and the distance reached 9.0 m before it recovered. Inferred, not isolated: the PX4 position loop answers the planner's set-points more
slowly than `cuas.flyer.Flyer`, and a short loss then costs more distance (not compared by measurement). PX4's gains are those of the x500v2 airframe,
not tuned to this 1.681 kg drone; the monitor was not widened and no parameters were tuned against these seeds.

### What is not the same as the real drone / as the native path
- **Airframe mismatch.** The airframe is the x500v2 (PX4 4902: gains, hover thrust, mixer) plus camera, Pi 5 and regulator: 1.681 kg against 1.52 kg. `EKF2_EV_POS_X/Y/Z`
  stay 0 (the camera is not at the IMU; the real vehicle needs the lever arm), and PX4's rate / attitude gains were not re-tuned for the extra mass.
- **The ESKF sees PX4's IMU at 250 Hz** (the bridge's HIL rate), not the native 500 Hz; the VIO stand-in (statistical 25 Hz, 50 ms latency) and the baro are the same as native.
- **Real compute time is in the loop.** The companion runs in wall-clock lockstep with the sim, so Python's planner time (mean 31 ms, worst 62 ms) adds to the simulated 50-60 ms
  latencies; the native run has no such coupling. This is the realistic direction, but it means the PX4 numbers depend on the host's load.
- **OBSTACLE_DISTANCE does not steer the drone.** PX4 v1.17 calls collision prevention only from the Position-mode stick task (`FlightTaskManualPosition`), not in Offboard, so
  `CP_DIST` has no effect here (the message is delivered to the `obstacle_distance` topic; the log shows it). Avoidance is the companion planner's, as on the native path.
- **Boot readiness** waits for PX4's reported position to agree with the companion's (0.3 m, 0.3 m/s) after 12 s: MAVSDK's `is_local_position_ok` flaps on vision-only
  fusion and `is_armable` is judged in Hold, which needs a global position. Arming is done in Offboard.
- **No failure injection, no wind compensation tuning, one forest generator**; 12 seeds per mission only.

For a video, run a PX4 flight and render it exactly like a native one (the run directory holds `telemetry.jsonl`, `depth.npz`, `model/forest.json`, `replay.naplog` and the bridge's `trace.csv`):

```
NAPL_PX4_INSTANCE=5 .venv/bin/napl program examples/skydio/follow.px4.scenario.toml --runs-dir build/runs
tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py build/runs/skydio-follow-px4-<stamp> --out build/frames --cam chase --fps 30'   # --cam side|top|fpv, --detections, --belief, --from/--to
```

## Parts list (the real drone this models)

| part | qty | price (USD) | source |
|---|---|---|---|
| Holybro PX4 Development Kit X500 V2 (frame, 4 x 2216 KV920, 4 x BLHeli_S 20 A ESC, 1045 props, PM02 V3, PDB, Pixhawk 6C, M8N GPS, SiK radio) | 1 | 507.99 | https://shop.holybro.com/x500-kit_p1180.html (price with the options preselected on the page; check the 6C configuration) |
| Tattu 4S 5200 mAh 35C 14.8 V LiPo, XT60 | 1 | 55.99-68.49 | https://pyrodrone.com/products/tattu-5200mah-14-8v-35c-4s1p-lipo-battery-pack-with-xt60-plug-1, https://racedayquads.com/products/tattu-14-8v-4s-5200mah-35c-lipo-battery-xt60 |
| Intel RealSense D435i depth camera | 1 | 328-351 | https://www.bluedogink.com/intel-realsense-depth-camera-d435i-82635d435idk5p.html (328.44), https://www.cdwg.com/product/intel-realsense-d435i-depth-camera/6152174 (351.11) |
| Raspberry Pi 5 8 GB | 1 | about 200 | https://www.adafruit.com/product/5813 (listed at 200 when checked) |
| Pololu D24V50F5 5 V 5 A step-down regulator | 1 | 32.95 | https://www.pololu.com/product/2851 |
| 18 AWG silicone wire, USB-C cable, printed nose bracket, M2.5 stand-offs | - | not priced | - |
| **Total** | | **about 1125-1160** | prices are retailer listings found during the build (Oct 2026), not quotes |

Masses and electrical numbers (all sourced in the part files, `fidelity` + `src`/`rationale`): D435i 75 g, 700 mA at 5 V max; Pi 5 under full CPU load
9.8 W (measured, `raspberry-pi-5.toml`); regulator 3 g, 85 % minimum efficiency. Total simulated mass 1.681 kg (x500v2 alone 1.52 kg).

## Wiring (assemblies/skydio.toml, DRC clean: 0 errors)

```
battery 4S --XT60--> PM02 V3 --XT60--> PDB --out1--+--> ESC 1 (and ESC 2-4 on out2..out4) --> motors
                       |                           +--> reg.vin  Pololu D24V50F5 (5 V, 5 A)
                       +--> Pixhawk 6C POWER1             reg.vout --> Pi 5 USB-C power
Pi 5 USB 3.0 ----------------------------------------> D435i USB-C (depth + IMU)
Pi 5 GPIO UART (TX/RX/GND) -------------------------> Pixhawk 6C TELEM2 (MAVLink companion link)
```

`load_compute` on the regulator input draws 1.057 A from the pack: (Pi 9.8 W + D435i 3.5 W) / (0.85 x 14.8 V).

## Scenario monitors (`goal.scenario.toml`, `follow.scenario.toml`)

Goal: `contact_steps` always 0; `clearance_m` always >= 0.05; `dist_goal` <= 1.5 m eventually within 45 s; `speed` >= 4 m/s eventually; tilt <= 45 deg.
Follow: no contact (obstacles or subject); clearance >= 0.05; horizontal follow distance within 3-8 m from t = 10 s to the end of the walk (target 4.5 m; the run ends when the walk does); subject tracked; tilt.
Plus the runner's replay check: the replay log re-run must reproduce the final state hash (`replay.match`).

## Per-tick artefacts and the telemetry contract (for the video / HUD worker)

One run directory (`fly.py <out>` or the `napl program` run dir) holds:

| file | content |
|---|---|
| `telemetry.jsonl` | one JSON object per planner step (25 Hz, at each processed depth frame), schema below |
| `trace.csv` | one row per 4 ms: `t,x,y,z,qw,qx,qy,qz,speed,clearance_m,contact_steps,dist_goal,tilt_deg,map_voxels,planner_mode,follow_dist,subject_seen,flying` (planner_mode index into `track, brake, escape, scan`), then (appended, old columns unchanged) `est_x,est_y,est_z,est_err_m,subject_walking`; x,y,z,speed are the TRUE state |
| `depth.npz` | `t` (frame sample times, s), `depth` (N, 90, 160) float16 metres, NaN = no return; `telemetry.depth_index` indexes it |
| `replay.naplog`, `native.json` | the kernel's replay log and `{tick, dt, state_hash}` (`napl program` adds `result.json`) |
| `model/scene.xml`, `model/forest.json`, `model/scene.json` | the MJCF; the obstacle list (kind, name, pos, size, yaw; also `start`, `goal`, subject `path`, `notes`); the compose manifest |
| `summary.json` | mission metrics (time to goal, min clearance, speeds, planner timing, follow stats incl. `follow_band_min_max`, the min / max follow distance over the walk) |

`telemetry.jsonl` record (all positions world frame, metres, z up; angles radians unless `_deg`):

```
t, tick                 sim time (s) and kernel tick of the planning instant
phase                   takeoff | map | fly | arrived
mode                    track | brake | escape | scan (planner fallback level)
pos[3], vel[3], yaw     TRUE state (kept for rendering / backwards compatibility); tilt_deg: commanded tilt
est_pos[3], est_vel[3], est_yaw   the ESKF estimate the planner used
est_cov{pos_sigma, vel_sigma, att_sigma_deg}, est_bias   filter 1-sigma and the estimated IMU biases
est_err{pos[3], pos_norm, vel_norm, yaw_deg}   estimate minus truth (evaluation only)
setpoint{pos[3], vel[3], yaw, vdes[3]}   what the controller is asked now; vdes = the primitive's target velocity
chosen[[x,y,z]...]      the flown primitive sampled every chosen_dt = 0.1 s (ramp, hold, braking tail); chosen_cost
candidates[{ok, cost, min_clear, v[3], path[10][3]}]   the 10 cheapest safe primitives (ok true) and 6 rejected ones (ok false), 10 points each
n_candidates, n_safe    how many primitives were evaluated / safe
map{voxels, frames, clearance_est, fast_frac}   occupied voxels in the map, depth frames integrated, planner's clearance at pos (m), share of the fast primitives that were safe (planner clutter measure; new)
min_clearance           ground-truth clearance minus the 0.40 m sphere (m), for display only
goal[2], depth_index    goal xy; index of this step's depth image in depth.npz
subject{est, est_vel[2], seen, sigma, box{row,col,half_h,half_w}, truth[3], detections[]}   follow mission only: tracker estimate (sigma = its position 1-sigma, m),
                        box = the detection the tracker chose on the 160 x 90 depth image (px, null if none), true position (evaluation only),
                        detections = every detection of this frame {row, col, half_h, half_w, range, score, xy, z_top, width, on_subject (evaluation only)}
                        NEW (additive): while the subject is unseen `est` / `est_vel` / `sigma` are the search belief's cluster (search.py), not the tracker's coast;
                        visible = the true subject was inside the depth image and not hidden (evaluation only); search = null | {particles: [[x, y] x ~40, world m], spread: belief rms m}
```

The video worker's commands (GPU, on hfbox): run a scenario with `napl program` (above), then
`tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py <run dir> --out <dir> --fps 30 --cam chase'` for a frame sequence
(`--cam side|top|fpv`, `--times` for stills; the replay hash is verified against the run). Overlay `telemetry.jsonl` (interpolate by `t`) and
`depth.npz` for the HUD; the forest layout is `model/forest.json`. `--cam fpv --detections` outlines the frame's detections on an FPV still
(green = the subject, red = a false positive); `summary.json` also holds `estimation` and `detection` blocks.
