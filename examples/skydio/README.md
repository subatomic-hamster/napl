# Skydio-style vision-only autonomous quadrotor

A Holybro X500 V2 (Pixhawk 6C, 4S 5200 mAh) with an Intel RealSense D435i depth camera on the nose and a Raspberry Pi 5 companion computer
(`assemblies/skydio.toml`, built from the parts library with the design API) flies through a seeded forest from its depth camera alone. It
never reads an obstacle position: depth image -> point cloud -> rolling voxel map -> clearance field -> sampled motion primitives -> setpoints.
Two missions: **(a) goal seeking** (58 m through trees, poles, boulders, a wall with a 2.4 m doorway and an arch, cruising >= 4 m/s where clear)
and **(b) following a walking person** through the same kind of forest while avoiding the trees.

```
tools/remote.sh run '.venv/bin/napl program examples/skydio/goal.scenario.toml   --runs-dir build/runs'   # PASS + replay hash match, ~25 s
tools/remote.sh run '.venv/bin/napl program examples/skydio/follow.scenario.toml --runs-dir build/runs'   # PASS + replay hash match, ~35 s
tools/remote.sh run '.venv/bin/python examples/skydio/montecarlo.py --seeds 24 --workers 5 --out build/skydio_mc'   # build/skydio_mc.{json,md}
.venv/bin/python examples/skydio/fly.py <out dir> goal|follow <seed>                                   # one flight, no scenario judge
tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py <run dir> --out build/frames --cam chase'   # PNG frames (GPU)
```

CPU only except the last line. Wall time is about 1 s per simulated second (the 160 x 90 ray cast and a 20 ms planner step at 25 Hz).

## Results

`napl program` scenarios (seed 1): both PASS, replay state hash matches. Monte Carlo, 24 forests per mission (every seed a different forest, wind
turbulence and sensor noise; `montecarlo.py`, 5 processes, ~6 min per set; raw JSON in `build/skydio_mc_seeds*.json`, not committed):

| | goal, seeds 100-123 | goal, seeds 200-223 (held out) | follow, 100-123 | follow, 200-223 (held out) |
|---|---|---|---|---|
| success | 24/24 | 24/24 | 24/24 | 24/24 |
| runs with a physical contact | 0 | 0 | 0 | 0 |
| min clearance, worst run (m) | 0.31 | 0.25 | 0.29 | 0.32 |
| min clearance, mean (m) | 0.45 | 0.44 | 0.54 | 0.47 |
| mean speed in flight (m/s) | 3.09 | 3.06 | 2.19 | 2.19 |
| 90th-percentile speed (m/s) | 4.20 | 4.20 | 2.94 | 3.02 |
| time to goal mean / worst (s) | 17.0 / 25.4 | 16.8 / 19.7 | - | - |
| follow distance (median per run, mean) (m) | - | - | 5.87 | 5.97 |
| subject seen (fraction of flight) | - | - | 0.70 | 0.70 |

Success = goal: reached within 45 s with zero contacts; follow: zero contacts, subject tracked >= 50 % of the flight, median follow distance in 3-8 m.
Clearance is the airframe sphere (0.40 m) to the nearest surface, so 0.25-0.3 m is the closest any run came to touching anything.
Honest reading: the first 24 seeds are the set the planner was tuned on (a first pass on them failed 3 runs, a thin-pole trap, a wall local minimum
and a lost subject, which led to the emergency-braking authority, the cost-to-go field and longer subject coasting); the second set was run once, after
the last change. Cruise is 4-4.6 m/s where the way is clear (90th percentile 4.2 m/s), the mean is lower because the vehicle accelerates from rest,
slows for gaps and brakes at the goal. The follow distance sits at 5.9 m against the 4.5 m target: the drone trails the subject (it avoids trees
the subject walks between). Likely cause of the early failure (inferred from the trace, not isolated): a 160 x 90 image resolves a 0.05-0.09 m-radius pole only a few pixels wide, so poles are first mapped late.

## Pipeline (every file is one stage; numpy only, no scipy)

| stage | file | what it does |
|---|---|---|
| vehicle | `build_design.py` -> `assemblies/skydio.toml` | x500v2 + D435i + Pi 5 + 5 V regulator, wires and power load, `depth` rangefinder device on the camera's optical frame |
| world | `forest.py`, `scene.py` | seeded obstacle field as exact primitives (cylinders, spheres, yawed boxes) -> MJCF geoms (collide, visible to the rays) and a contact sensor |
| depth -> map | `mapping.py` | pinhole back-projection, log-odds voxel grid (0.2 m, 32 m torus around the drone), free-space carving along rays, separable Euclidean distance field |
| plan | `planner.py` | 400+ velocity-ramp primitives per replan; a primitive is flown only if its whole path including the braking tail keeps 0.62 m from every occupied voxel and stays inside space the camera has already seen to be free; goal / follow objective picks among the safe ones; fallbacks brake, escape, scan |
| follow | `follow.py` | simulated person detector (pixel box with jitter, 3 % misses, occlusion by depth) + depth-based alpha-beta tracker; the walking subject itself |
| control + loop | `fly.py` | takeoff, 1 s mapping hover, flight; the native `cuas.flyer.Flyer` tracks position / velocity / acceleration setpoints; per-tick artefacts |
| judge | `*.scenario.toml`, `montecarlo.py` | `napl program` native interface: monitors on `trace.csv` + replay-log hash re-check; Monte Carlo over forest seeds |

The planner's safety argument: the vehicle only ever follows a primitive whose braking stop is inside observed-free, obstacle-clear space, so a
stop is always available unless NEW information (a thin pole first resolved at 5 m) invalidates it; then it brakes at 4.6 m/s^2 (25 deg tilt).
Thin poles (0.05-0.09 m radius) are the hard case for a 160 x 90 image at 0.095 m per pixel at 8 m.

## Honesty notes (what is real and what is a stand-in)

- **State estimate is ground truth** (position, velocity, attitude from the kernel): a stand-in for VIO. The camera's pose in the body frame and the
  intrinsics come from the model; depth noise is the device's (2 % of range, sigma1 from the D435i datasheet RMS error) with a 0.06 s latency (estimate).
- The **depth image is sub-sampled 160 x 90** of the real 1280 x 720 (same field of view 87 x 58 deg, 0.28-10 m range, 25 Hz); no stereo matching artefacts,
  flying pixels, or sunlight washout. Ray casting is exact against the MJCF geoms; the drone's own landing gear and props are NOT in the image
  (the camera sits ahead of them; checked on a frame).
- The **person detector is simulated** (a pixel box from the true body position with noise and misses), not a neural network.
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
Follow: no contact (obstacles or subject); clearance >= 0.05; horizontal follow distance within 3-8 m from t = 10 s (target 4.5 m); subject tracked; tilt.
Plus the runner's replay check: the replay log re-run must reproduce the final state hash (`replay.match`).

## Per-tick artefacts and the telemetry contract (for the video / HUD worker)

One run directory (`fly.py <out>` or the `napl program` run dir) holds:

| file | content |
|---|---|
| `telemetry.jsonl` | one JSON object per planner step (25 Hz, at each processed depth frame), schema below |
| `trace.csv` | one row per 4 ms: `t,x,y,z,qw,qx,qy,qz,speed,clearance_m,contact_steps,dist_goal,tilt_deg,map_voxels,planner_mode,follow_dist,subject_seen,flying` (planner_mode index into `track, brake, escape, scan`) |
| `depth.npz` | `t` (frame sample times, s), `depth` (N, 90, 160) float16 metres, NaN = no return; `telemetry.depth_index` indexes it |
| `replay.naplog`, `native.json` | the kernel's replay log and `{tick, dt, state_hash}` (`napl program` adds `result.json`) |
| `model/scene.xml`, `model/forest.json`, `model/scene.json` | the MJCF; the obstacle list (kind, name, pos, size, yaw; also `start`, `goal`, subject `path`, `notes`); the compose manifest |
| `summary.json` | mission metrics (time to goal, min clearance, speeds, planner timing, follow stats) |

`telemetry.jsonl` record (all positions world frame, metres, z up; angles radians unless `_deg`):

```
t, tick                 sim time (s) and kernel tick of the planning instant
phase                   takeoff | map | fly | arrived
mode                    track | brake | escape | scan (planner fallback level)
pos[3], vel[3], yaw     state estimate used (ground truth); tilt_deg: commanded tilt
setpoint{pos[3], vel[3], yaw, vdes[3]}   what the controller is asked now; vdes = the primitive's target velocity
chosen[[x,y,z]...]      the flown primitive sampled every chosen_dt = 0.1 s (ramp, hold, braking tail); chosen_cost
candidates[{ok, cost, min_clear, v[3], path[10][3]}]   the 10 cheapest safe primitives (ok true) and 6 rejected ones (ok false), 10 points each
n_candidates, n_safe    how many primitives were evaluated / safe
map{voxels, frames, clearance_est}   occupied voxels in the map, depth frames integrated, planner's clearance at pos (m)
min_clearance           ground-truth clearance minus the 0.40 m sphere (m), for display only
goal[2], depth_index    goal xy; index of this step's depth image in depth.npz
subject{est[3], est_vel[2], seen, box{row,col,half_h,half_w}, truth[3]}   follow mission only: tracker estimate, detector box on the 160 x 90 depth image (px, null if missed), true position
```

The video worker's commands (GPU, on hfbox): run a scenario with `napl program` (above), then
`tools/remote.sh run --gpu '.venv/bin/python examples/skydio/render_frames.py <run dir> --out <dir> --fps 30 --cam chase'` for a frame sequence
(`--cam side|top|fpv`, `--times` for stills; the replay hash is verified against the run). Overlay `telemetry.jsonl` (interpolate by `t`) and
`depth.npz` for the HUD; the forest layout is `model/forest.json`.
