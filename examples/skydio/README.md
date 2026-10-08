# Skydio-style vision-only autonomous quadrotor

A Holybro X500 V2 (Pixhawk 6C, 4S 5200 mAh) with an Intel RealSense D435i depth camera on the nose and a Raspberry Pi 5 companion computer
(`assemblies/skydio.toml`, built from the parts library with the design API) flies through a seeded forest from its depth camera alone. It
never reads an obstacle position: depth image -> point cloud -> rolling voxel map -> clearance field -> sampled motion primitives -> setpoints.
Two missions: **(a) goal seeking** (58 m through trees, poles, boulders, a wall with a 2.4 m doorway and an arch, cruising >= 4 m/s where clear)
and **(b) following a walking person** through the same kind of forest while avoiding the trees.

```
tools/remote.sh run '.venv/bin/napl program examples/skydio/goal.scenario.toml   --runs-dir build/runs'   # PASS + replay hash match, ~25 s
tools/remote.sh run '.venv/bin/napl program examples/skydio/follow.scenario.toml --runs-dir build/runs'   # replay hash match; FAILS the strict band monitor by 0.05 m on seed 1 (Results), ~40 s
tools/remote.sh run '.venv/bin/python examples/skydio/montecarlo.py --seeds 24 --workers 5 --out build/skydio_mc'   # build/skydio_mc.{json,md}
.venv/bin/python examples/skydio/fly.py <out dir> goal|follow <seed>                                   # one flight, no scenario judge
tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py <run dir> --out build/frames --cam chase'   # PNG frames (GPU)
```

CPU only except the last line. Wall time is about 1 s per simulated second (the 160 x 90 ray cast and a 20 ms planner step at 25 Hz).

## Results

Everything below is flown on the ESTIMATED state (ESKF on IMU + VIO + baro) with the person found by the detector on the depth image; the
kernel's true state is used only to move the physics and to score. `napl program` scenarios (seed 1, replay state hash matches in both):

- goal: PASS, hash `1fe6d3d440cb1c04`.
- follow: **FAIL** on the strict 3-8 m `follow_distance_band` monitor: range [3.87, 8.05] m, i.e. 0.05 m over for a moment while the subject was
  briefly lost (hash `354b8f9d4c3154dc`; no contact, subject tracked at the end). Not hidden by loosening the band or by picking another seed: the
  model script builds the forest from the default seed, so the scenario cannot change seed on its own, and seed 2 flown through `fly.py`
  stays within 3.94-5.97 m while the subject walks. The Monte Carlo below is the representative number.

Monte Carlo, 24 forests per mission (every seed a different forest, wind turbulence, IMU/VIO/baro noise and detector miss pattern; `montecarlo.py`,
5 processes, ~10 min per set; raw JSON in `build/mc_*.json`, not committed). Seeds 100-123 are the set development used (and the planner was tuned on
in the first version); 300-323 were run once after the last change; "thin" = seeds 100-123 with 10-14 extra poles of radius 0.025-0.05 m per forest
(`montecarlo.py --thin`):

| | goal 100-123 | goal 300-323 (held out) | goal thin | follow 100-123 | follow 300-323 (held out) | follow thin |
|---|---|---|---|---|---|---|
| success | 24/24 | 24/24 | 24/24 | 23/24 (seed 111) | 23/24 (seed 302) | 19/24 (105, 108, 110, 111, 116) |
| runs with a physical contact | 0 | 0 | 0 | 0 | 0 | 0 |
| min clearance, worst run (m) | -0.02 (seed 109) | 0.31 | 0.28 | 0.24 | 0.23 | 0.40 |
| mean speed in flight (m/s) | 3.09 | 3.17 | 3.16 | 2.49 | 2.47 | 2.47 |
| time to goal mean / worst (s) | 16.9 / 18.6 | 16.6 / 19.0 | 16.7 / 18.3 | - | - | - |
| follow distance, median of per-run medians (IQR) (m) | - | - | - | 4.77 (4.40-5.08) | 4.81 (4.32-5.16) | 4.90 (4.63-5.40) |
| subject seen (fraction of flight) | - | - | - | 0.59 | 0.59 | 0.53 |
| detector recall on frames where the subject is visible | - | - | - | 0.88 | 0.88 | 0.89 |
| detector false positives per frame, mean / worst run | - | - | - | 0.008 / 0.023 | 0.008 / 0.023 | 0.016 / 0.037 |
| estimate position error RMS, mean / worst run (m) | 0.34 / 0.68 | 0.29 / 0.51 | 0.33 / 0.60 | 0.34 / 0.66 | 0.30 / 0.52 | 0.34 / 0.66 |
| estimate position error, worst max (m) | 0.96 | 0.79 | 0.93 | 1.00 | 0.82 | 1.03 |
| drift at the end, mean / worst (% of path) | 0.76 / 1.61 | 0.71 / 1.36 | 0.78 / 1.53 | 0.91 / 1.66 | 0.80 / 1.74 | 0.89 / 1.72 |
| attitude error RMS, mean (deg) | 0.37 | 0.38 | 0.37 | 0.37 | 0.39 | 0.38 |

Success = goal: reached within 45 s with zero contacts; follow: zero contacts, subject tracked >= 50 % of the flight, median follow distance in 3-8 m.
No follow failure is a contact; the ones inspected by hand (111, 302) are "subject lost" (seen fraction 0.39 / 0.27, a long unseen stretch), the other five were not inspected. Clearance is the airframe sphere (0.40 m)
to the nearest surface; goal seed 109 is the one run that got through the bounding sphere (-0.02 m: about 0.38 m from the trunk's surface to the vehicle's
centre, 0 physical contacts): at 4.3 m/s it was threaded between three trunks when a late-resolved trunk made the planner flip its dodge direction and brake at
the vehicle's limit; the cause of the flip was not isolated (a 0-hold map change gave the same result), so treat it as an open planner weakness.
Why the follow distance is now centred on 4.5 m (4.77 / 4.81 median, was 5.9): the controller leads the subject by the sensing latency, an integral term
trims a persistent trailing bias on the measured distance, and the person is no longer a static keep-out sphere (a predicted-track keep-out of 1.1 m
instead). The subject is hidden behind trunks about 40 % of the flight, so the median is fed by the detected frames and the coasting track.
Thin-pole forests cost goal missions nothing but the follow mission 5 of 24 (inferred: more of the camera's view is cluttered, so the subject is visible less).

## Pipeline (every file is one stage; numpy only, no scipy)

| stage | file | what it does |
|---|---|---|
| vehicle | `build_design.py` -> `assemblies/skydio.toml` | x500v2 + D435i + Pi 5 + 5 V regulator, wires and power load, `depth` rangefinder device on the camera's optical frame |
| world | `forest.py`, `skydio_scene.py` | seeded obstacle field as exact primitives (cylinders, spheres, yawed boxes) -> MJCF geoms (collide, visible to the rays) and a contact sensor |
| depth -> map | `mapping.py` | pinhole back-projection, log-odds voxel grid (0.2 m, 32 m torus around the drone), free-space carving along rays, separable Euclidean distance field |
| plan | `planner.py` | 400+ velocity-ramp primitives per replan; a primitive is flown only if its whole path including the braking tail keeps 0.62 m from every occupied voxel and stays inside space the camera has already seen to be free; goal / follow objective picks among the safe ones; fallbacks brake, escape, scan |
| state estimate | `estimator.py` | 15-state error-state Kalman filter (position, velocity, attitude, accel + gyro bias) on the IMU device at 500 Hz, a velocity + attitude VIO measurement at 25 Hz (statistical model of a VIO output, 50 ms delayed, handled by state-history re-linearisation), the baro height at 50 Hz |
| person detection | `detector.py` | classical detector on the depth image + the pose estimate only (ground removal, connected components, size prior), constant-velocity Kalman tracker with a Mahalanobis gate |
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
- Near-field blind zone (inferred from traces, not measured): the camera's lower edge cuts off a person closer than a few metres, so a subject the drone overtakes is lost; the mission therefore ends when the
  walk ends (a standing person at 1-2 m is not tracked).
- **Not attempted: the goal mission through PX4.** `napl program` can run PX4 SITL in lockstep, but its bridge owns the World and would have to
  carry the depth sensor, the IMU / baro feed and a vision-odometry injection path (EKF2 `vision` fusion) alongside the Python planner; that is a
  rewrite of `fly.py`'s plumbing, not a configuration. `cuas.flyer.Flyer` gains were not compared with PX4 defaults (the default values were not fetched).
- The **depth image is sub-sampled 160 x 90** of the real 1280 x 720 (same field of view 87 x 58 deg, 0.28-10 m range, 25 Hz); no stereo matching artefacts,
  flying pixels, or sunlight washout. Ray casting is exact against the MJCF geoms; the drone's own landing gear and props are NOT in the image
  (the camera sits ahead of them; checked on a frame).
- **The flight controller is `cuas.flyer.Flyer`** (cascaded PD + geometric attitude + thrust curves calibrated from the motor / prop models), not PX4.
  The acceleration feed-forward is emulated through the position setpoint.
- Not modelled: the printed nose bracket, M2.5 stand-offs, cable clamps (about 25 g); the Pi's heatsink / fan. The Pi 5 mass (46 g) is an estimate.
  The 6 DRC warnings are inherited unchanged from `assemblies/x500v2.toml`.
- "Collision" is the physics: a MuJoCo `<contact>` sensor on the airframe subtree against the obstacle body (and the subject). `clearance_m` is the exact
  analytic surface distance of the generator's primitives minus a 0.40 m airframe bounding sphere (evaluation only; the planner never sees it).
- The wind is a steady 0.8 / 0.4 m/s plus Dryden turbulence (W20 = 1 m/s), light for a forest.

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
| `summary.json` | mission metrics (time to goal, min clearance, speeds, planner timing, follow stats) |

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
map{voxels, frames, clearance_est}   occupied voxels in the map, depth frames integrated, planner's clearance at pos (m)
min_clearance           ground-truth clearance minus the 0.40 m sphere (m), for display only
goal[2], depth_index    goal xy; index of this step's depth image in depth.npz
subject{est, est_vel[2], seen, sigma, box{row,col,half_h,half_w}, truth[3], detections[]}   follow mission only: tracker estimate (sigma = its position 1-sigma, m),
                        box = the detection the tracker chose on the 160 x 90 depth image (px, null if none), true position (evaluation only),
                        detections = every detection of this frame {row, col, half_h, half_w, range, score, xy, z_top, width, on_subject (evaluation only)}
```

The video worker's commands (GPU, on hfbox): run a scenario with `napl program` (above), then
`tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py <run dir> --out <dir> --fps 30 --cam chase'` for a frame sequence
(`--cam side|top|fpv`, `--times` for stills; the replay hash is verified against the run). Overlay `telemetry.jsonl` (interpolate by `t`) and
`depth.npz` for the HUD; the forest layout is `model/forest.json`. `--cam fpv --detections` outlines the frame's detections on an FPV still
(green = the subject, red = a false positive); `summary.json` also holds `estimation` and `detection` blocks.
