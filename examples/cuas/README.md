# Counter-UAS demo: a pan / tilt turret against a programmed drone swarm

A turret (pan servo + tilt servo + camera + a generic pneumatic pellet launcher, `assemblies/turret.toml`) watches four programmed drones
(three `x500v2`, one `aos5`, flown by `swarm.py`), tracks them from camera pixels, and fires only after an operator authorizes. Everything is
one deterministic `napl` World: the turret program, the swarm and the physics.

**Scope.** The effector is a generic 4.5 mm (.177) pneumatic *pellet launcher* (150-250 m/s class) modelled as point-mass ballistics with air
drag and wind. No real firearm, munition or weapon system is modelled; the numbers are air-rifle class. The turret *proposes*, a human
*authorizes*: the operator is scripted here (`operator()` in `turret.py`) and every authorization is logged as `operator authorized <id>`.

## Run it

```sh
# hfbox (the renderer needs the GPU: --gpu takes the shared GPU lock); ~95 s wall for 60 s of sim
tools/remote.sh run --gpu '.venv/bin/napl program examples/cuas/cuas.scenario.toml --runs-dir /workspace/wt/napl-cuas/out/runs'
tools/remote.sh pull out/runs/<run dir>/{hud.jsonl,truth.csv,events.json,summary.json,trace.csv,result.json,program.log,frames}
python examples/cuas/report.py remote-out remote-out/report     # laptop: annotated frames, contact sheet, trajectories.png, report.txt
```

`napl program` builds the scene (`scene.py`), starts `turret.py` with `NAPL_MODEL` / `NAPL_RUN_DIR`, judges the monitors of
`cuas.scenario.toml` on `trace.csv`, and re-runs `replay.naplog` through `napl-replay` to check the state hash. Rendered media is never
committed: frames and plots stay in the run dir on hfbox and in `remote-out/` (gitignored).

## Result of the committed configuration (seed 7, 60 s of sim, hfbox, Mesa RADV)

| | |
|---|---|
| rounds fired | 23 (first at 0.6 s) |
| hits on a drone | 4 (hit rate **17 %**, impact speed 146-158 m/s); 13 rounds missed and hit the ground 298-337 m away (the monitor's box is 400 m), 6 were still in flight or expired when the run ended |
| hits with functional damage | 2: `drone4` arm `con2` -> ESC 2 open (t = 17.8 s), `drone3` arm `con4` -> ESC 4 open (t = 37.3 s); the other 2 hit `drone1`'s `hanger8` and `platform` (frame parts: no effect) |
| drones destroyed | **2 of 4** (`drone4` at 18.4 s, `drone3` at 37.8 s: tilted past 80 deg a few hundred ms after losing a motor, then fell) |
| drones fired at | 4 (`drone1`..`drone4`; a round counts as fired at a drone when it passed within 3 m of it) |
| authorizations | 16 requests: 13 granted, 3 denied (target beyond the 55 m envelope); 0 shots refused for lack of authorization |
| closest approach of the 23 rounds to the drone it was fired at | median 0.21 m, best 0.05 m (seen from the side a drone's physical cross-section is a few cm of arm, motor and plate edge) |
| replay | state hash `5e91ef685aa7c360` at tick 60000 in two separate runs; `napl-replay` reproduces it |
| monitors | all 6 pass |

The hit rate is low and it is what the model gives: the quads in the library are thin (2 mm plates, 15 mm arm tubes, 28 mm motors, 254 x 30 mm
blades), and the turret's aim is good to about 0.1-0.2 m at 40 m, not better: it fires only when the muzzle is within `AIM_TOL` = 0.003 rad (0.12 m
at 40 m) of the lead solution (over all AIM frames the pointing error has median 0.0054 rad), and the solution itself carries the velocity
estimate's error and the centroid jitter of a rotating dark blob. A different seed or a tweak to a gain will move these numbers; nothing here is
tuned to a target score.

## Layout

| file | what |
|---|---|
| `assemblies/turret.toml`, `parts/cuas/*`, `parts/electronics/raspberry-pi-camera-module-3.toml` | the turret: 2 Dynamixel XL430-W250-T, a base plate, a yoke, the launcher, a Pi Camera 3. `napl design check`: 0 errors, 0 warnings (SERVO_TORQUE margin 33). Sources of every number are in the part files; the launcher is `estimated` throughout, the pellet (H&N Field Target Trophy .177: 0.5598 g, BC G1 0.021) is datasheet |
| `src/mujoco/devices/launcher.cpp`, `schema/devices.fbs` | launcher + camera-pose devices (docs/models.md) |
| `python/napl/cuas/scene.py` | `compose`: several assemblies in one MJCF, namespaced |
| `python/napl/cuas/flyer.py` | napl-native drone controller (`Flyer`) + `calibrate` |
| `python/napl/cuas/world.py` | `World`: stepping, turret interface, hits -> damage, HUD, artefacts |
| `python/napl/cuas/camera.py`, `tracking.py`, `ballistics.py` | camera frames, sky detection + tracker, lead solution |
| `examples/cuas/swarm.py`, `scene.py`, `turret.py`, `cuas.scenario.toml`, `report.py` | the demo |
| `tests/python/test_cuas.py`, `tests/cpp/test_launcher.cpp` | tests (no GPU needed) |

## Multi-vehicle scene

`compose(outdir, [Vehicle("turret", "turret", pos=(0, 0, 1), pedestal=1), Vehicle("drone1", "x500v2", pos=...), ...], Environment(wind=...))`
builds each assembly with `napl.design`, prefixes every MJCF name with `<vehicle>/` and every device with `<vehicle>_` (`/devices/drone1_esc1`;
body prims `/world/drone1_bottom/...`: a free joint must be on a top-level body, so there is no container body and the vehicle's root is named
`<vehicle>_<root>`), moves each vehicle to its placement (position + yaw), offsets electrical node numbers per vehicle (each keeps its own
network; the kernel's limit rose from 32 to 128 nodes, the block-diagonal solve is bit-identical for a single network), replaces the vehicles'
environment devices with one `air` (steady wind + Dryden turbulence), and adds the 16 mocap tracer bodies of the launcher. It writes `scene.xml`
and `scene.json` (the body-id order, prim paths, device names and configs the `World` needs).

## Drones: napl-native controller, not PX4 multi-instance

`Flyer` (`python/napl/cuas/flyer.py`) is the controller of `Design.simulate("hover")` grown into a setpoint tracker (position PD+I -> thrust
vector -> geometric attitude -> torques -> a mixer from the vehicle's own rotor geometry -> per-rotor throttle through the command bus), with a
calibration of every rotor's thrust-vs-throttle curve on a fork. Ground-truth state, no estimator: a flight-control stand-in, not an avionics
model. The attitude loop is slower than `hover`'s (12 rad/s, not 20) because 20 rad/s limit-cycles an x500 in turbulence.

Why not PX4 SITL: the `px4` interface of `napl program` is one World per bridge in wall-clock lockstep with one PX4 process (~6 s of boot each, real
time); four vehicles would need four Worlds or a multi-vehicle bridge, and the turret, the pellets and the drones would no longer be one World
with one state hash and one replay log. With the native controller everything is one 1 kHz World (control period 4 ms) and `replay.naplog` /
the hash cover all of it. The swarm program sees `Drone.goto(pos, vel)`.

`swarm.py`: seeded boids (separation, cohesion, alignment, wander) plus an arena force: 15-52 m from the turret, 5-25 m altitude, within +-55 deg
of the turret's +x axis, <= 5 m/s, <= 3 m/s^2; each agent is the *virtual leader* of one drone.

## The turret program (`turret.py`)

Uses only: camera pixels, the camera pose, the pan / tilt readouts, a mast anemometer (true wind + 0.3 m/s noise, low-passed), the launcher's
counters, and the `authorize` callback. Stages (`Turret.stage`, in every HUD record):

| stage | what |
|---|---|
| SENSE | no tracks: pan sweep +-60 deg at 0.45 rad/s |
| TRACK | detections not yet confirmed (< 3 consistent frames) |
| PICK | a new target is chosen (nearest by estimated range, not recently given up on); aim starts, no shot this frame |
| AIM | lead solution -> pan / tilt goals; fire needs pointing error < 0.003 rad |
| FIRE | aimed and authorized: one round every >= 0.5 s, 3 rounds per authorization, 5 rounds per target before it moves on for 6 s |

Per frame (14.7 Hz, 1280 x 720, vertical FOV 41 deg as the Pi Camera 3 -> `fov_v` = 0.7156 rad): `detect` (sky colour = median above the horizon row,
foreground = colour distance > 18, own-barrel mask, 2 px dilation, flood fill) -> `Tracker` (association in pixels after projecting each track's
world estimate with the current camera pose, so it follows through pans; IDs `A`, `B`, ... never reused) -> **range from apparent size**
(`f_px * 0.55 m / size_px`: an assumed 0.55 m target; the size is the median of the last 5 boxes) -> bearing from the pixel ray and the camera pose
(pan / tilt) -> `world_xyz_est` = camera + ray x range -> alpha-beta velocity (tight across the line of sight, loose along it) ->
`ballistics.solve_lead` (RK4 with the same drag law as the kernel, wind estimate, constant-velocity target, iterates the intercept) -> pan /
tilt goals set `SERVO_LAG` ahead (the goal is held between frames; `p_gain` raised 640 -> 1500, which cuts a 0.2 rad/s ramp's lag from 65 ms to
15 ms). The depth image of the renderer is ground truth: `Camera.grab(depth=True)` exists for tests and plots, the turret never asks for it.

Measured in the committed run (against ground truth, 882 frames): range error mean -1 %, std 14 % (the aos5 is half the assumed size, so it reads
twice as far; the swarm mostly flies 25-50 m), world-position error median 4 m (almost all along the line of sight, where it hardly matters for a
pellet flying that line). **Track IDs are not perfect**: 10 IDs were issued for 4 drones; `A`, `B`, `C`, `E`, `H` follow a single drone, `D`, `F`, `G`,
`I`, `J` swap between drones when they cross in the image (a re-acquired target, after leaving the field of view, is a new ID).

## HUD telemetry

`hud.jsonl`, one JSON object per camera frame, and the same dict to every live callback (`console` in `turret.py` prints one line per second to
`program.log`): `t, frame, stage, selected, tracks[{id, bbox_px, range_m, world_xyz_est, vel_est, confirmed, coasting}], rounds, hits, destroyed,
damaged, fired, projectiles[{round, pos}], tracers[{round, p0, p1, t}] (last 0.5 s), muzzle{pos, dir, pan, tilt}, wind_est, lead{az, el, t_flight, ok,
aim_err}, camera_frame` (`frames/f00123.png` in the run dir). `events.json`: every request / authorization / fire / hit (with the body and its
effect) / destroyed. `truth.csv` (drone positions, 10 Hz) and the per-round closest approach in `summary.json` are for evaluation only; the turret
never reads them.

## Hit -> damage (an assumption of this scenario, not a lethality claim)

The kernel reports which body each pellet struck. `world.py` maps that to the existing device fault attrs: a prop -> that rotor's `damage` = 1
(blade lost); an arm / tube / motor / ESC body -> `fault_open` of the ESC of the *nearest rotor* to the hit point; the flight controller / power
board (`fc`, `pdb`, `pm02`, aos5 `esc`) -> every ESC open; the battery pack -> `r_fault` 0.5 ohm; anything else (plates, legs, hangers, mast, GPS)
-> no functional damage. **Destroyed** = a drone with functional damage that then drops below 1 m or tilts past 80 deg. A pellet is 0.56 g at
~150 m/s on impact (about 6 J): whether that really kills a motor or an arm is not something this model knows.

## Determinism

The physics, the SetAttr stream (throttles, servo goals, fire) and the state hash are the kernel's: two runs on the same machine give the same
hash (`5e91ef685aa7c360` twice, and again after code edits that did not change behaviour), and `napl-replay` of `replay.naplog` reproduces it.
**Camera pixels are deterministic per GPU / driver only** (`Sim.render`'s contract). The hash does not depend on the frames, but the program's
decisions do (they come from pixels): on another GPU, driver or a software rasteriser a threshold can flip on one edge pixel, the SetAttr stream
can then differ, and so can the hash and the hit rate. The detector uses margins (threshold 18 on a 25/31/41 sky versus a drone at 5/5/6), so
tracks and IDs are expected to match, but that is an expectation, not a guarantee. The tests need no GPU; the scenario does.

## Limits

One launcher per scene (the tracer pool is shared); the pellet is a point and the drone geometry is the library's thin visual-scale model, which
makes a hit rare; no muzzle blast, recoil, spin drift or Magnus; the drones fly on ground-truth state with an idealised controller; the camera has
no noise, blur or latency; the sky is the renderer's flat colour and the drones are dark on it (the detector would need a rework for a lit sky or
a cluttered horizon); the operator is a script.
