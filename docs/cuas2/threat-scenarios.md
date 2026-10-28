# Threat scenarios: generic kinematic test targets for the tracker

Package `python/napl/threats/` (pure numpy). Scope, set by the user on 2026-10-28: generic kinematic test targets at backyard scale
(3-40 m range, speeds up to about 15 m/s) for tracker and discrimination work. There is no homing, pursuit, dive or missile-like
profile and nothing that targets a person; routes follow arbitrary waypoints, weaving and bird flight are random walks in turn rate,
and the only "steering" is a keep-in box that turns a weaving drone or a bird away from the walls of the working volume (a geofence).
`napl.track.scenarios` (`NAMES`, `build`) is untouched; `tests/python/test_threats.py` pins its outputs.

## Structure

| file         | contents                                                                                                  |
| ------------ | --------------------------------------------------------------------------------------------------------- |
| `gen.py`     | generators `name(rng, T, **params) -> (P, V)` at 100 Hz and `name_params(rng)` randomised-parameter draws |
| `scenes.py`  | `SUITE`, `build(name, seed)`, `spawn(kind, rng, T, over)`, `random_objs`, `ThreatScenario`, `CameraLidar` |
| `bench.py`   | alive-mask-aware runner of the `napl.track` variants, aggregation, tables                                 |
| `discrim.py` | features of a position history, numpy softmax regression, confusion/ablation report                       |

`ThreatScenario` is the `napl.track` `Scenario` dataclass (`name, dt, ids, truth (T,n,6), frames [(t, meas, zs)], lam, pd, c, n_clutter`)
plus `labels` (one of drone / balloon / hanging / bird / debris per truth object), `kinds` (generator), `alive (T,n)` (birth/death:
only existing objects are scored), `diam (n,)` physical size, `sensor` (`pos` or `cam`) and `lidar`.

Determinism: `build(name, seed)` draws truth from `default_rng([sum(ord(name)), seed])` and measurements from a second stream, so
`<name>_lidar` has exactly the truth of `<name>`. Truth is integrated at 0.01 s and subsampled to dt = 0.05 s (20 Hz, the camera rate used
by the `cam` scenario), so wingbeat heave above 10 Hz aliases exactly as it would on a 20 Hz sensor.

## Measurement models (reuse of `napl.track.models`)

- `pos`: `PositionModel(0.5 m)`, Pd 0.85-0.9, uniform Poisson clutter in the scene bounding box (as `napl.track.scenarios`).
- `cam`: `CameraModel(size_range=True)` on a pan/tilt head that follows the centroid of the alive objects with one frame of lag
  (as the existing `cam` scenario): 2 px noise, size-based range with 15 % noise, clutter uniform in the image.
- `<name>_lidar` (second sensor, optional): `CameraLidar`, the same camera whose range channel is a boresight rangefinder
  (TF-Luna-like: 0.2-8 m, std 1.5 cm, 2 degree beam; values from the brief, beam angle [est]) while the target is inside beam and 8 m,
  and the size-based range elsewhere. A range-only sensor co-located and co-pointed with the camera is exactly this per-measurement
  range-noise switch, so it needs no change to the trackers (they take one measurement model per scan).
- Occlusion gaps (1-3 s intervals per object, Poisson rate `occl`), missed detections (Pd) and births/deaths (windows) are applied to
  every object in the mixed scenes.

## Generators and parameters

Randomised mode = `name_params(rng)`; ranges below. `[est]` = textbook-scale guess, not from a source.

| generator    | model                                                                                                                                                                                                                                                                                                                             | parameters (random mode)                                                                                                                        |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `route`      | point-mass multirotor, waypoint list with dwell (hover) times; velocity set-point from a braking profile `sqrt(vp^2 + 2*0.7*a*d)`, accel command lagged 0.6 s, acceleration and jerk clipped (slew); hovers after the last waypoint                                                                                               | stock: vmax 4-8 m/s, a 3.0*(0.8-1.2) m/s^2, jerk 4.0*(0.8-1.2) m/s^3, vz up 3 / down 1.5 m/s, 3-6 waypoints in the 8-35 m box, 40 % hover 1-4 s |
| `route_fast` | same, tuned airframe                                                                                                                                                                                                                                                                                                              | vmax 8-15 m/s, limits x1-2 of stock [est]                                                                                                       |
| `survey`     | lawnmower lanes along x, corners flown through                                                                                                                                                                                                                                                                                    | lane 3-6 m, 10-18 m x 8-18 m patch, z 5-12 m, vmax 3-9 m/s                                                                                      |
| `orbit`      | circle about a fixed point, altitude bob, radius wobble                                                                                                                                                                                                                                                                           | R 4-12 m, v 2-min(10, sqrt(0.5 g R)) (centripetal <= 0.5 g), bob 0-1.5 m at 0.05-0.2 Hz                                                         |
| `weave`      | Singer-like: turn rate, speed and vertical acceleration are smooth Ornstein-Uhlenbeck                                                                                                                                                                                                                                             | v0 3-11 m/s, turn-rate std 0.3-1.2 rad/s, tau 1-3 s, speed std 0.5-1.5 m/s, lateral accel <= 8 m/s^2 [est]                                      |
| `swarm`      | leader route plus formation offsets in a slew-limited leader frame; formations line / V / grid; smooth formation change over 4 s; optional split (last k agents peel off sideways 6-12 m and rejoin); spacing jitter                                                                                                              | n 4-9, spacing 2-5 m, jitter std 0.1-0.4 m, leader vmax <= 7 m/s, change at 5-12 s                                                              |
| `balloon`    | velocity relaxes to the local wind with time constant tau; Dryden turbulence plus 1-cosine discrete gusts; net buoyancy drift                                                                                                                                                                                                     | mean wind 0.5-3 m/s, tau 1-4 s [est], gust gain x1-3, 0-2 gusts of 1.5-4 m/s lasting 1.5-4 s, buoyancy +-0.3 m/s                                |
| `hanging`    | tethered load: horizontal pendulum (g/L spring, damping, drag toward the local wind), bob height from the string constraint                                                                                                                                                                                                       | L 0.5-3 m, initial swing 0.05-0.5 rad, damping 0.03-0.15 /s, wind 0.3-2.5 m/s                                                                   |
| `bird`       | alternating flap and glide phases (exponential durations per species), flapping adds body heave and +-8 % speed modulation at the wingbeat frequency, glide decays to 0.75 v0 with a sink rate; erratic turns = OU turn rate (0.15 rad/s) plus Poisson turn shocks (rate 0.1-0.4 /s, 0.3-0.8 s, N(0, 1.0-2.0 rad/s)); keep-in box | species pigeon / corvid / gull / small, cruise 6-14 m/s, heave 2-7 cm [est]                                                                     |
| `leaf`       | drifting debris: wind relaxation (tau 0.5-1.5 s), sink 0.4-1.2 m/s, tumble flutter 10-30 cm at 1-3 Hz, lateral OU 0.4 m/s; ends on the ground                                                                                                                                                                                     | start 8-18 m high [est]                                                                                                                         |
| `bag`        | plastic bag: near neutral buoyancy, vertical gusts x2, weak altitude restoring                                                                                                                                                                                                                                                    | wind 0.5-3 m/s [est]                                                                                                                            |

### Sources

- PX4 default multicopter motion limits (main branch, `src/modules/mc_pos_control/*_params.yaml`, read 2026-10-28): auto mode
  MPC_XY_CRUISE 5.0 m/s, MPC_ACC_HOR 3.0 m/s^2, MPC_JERK_AUTO 4.0 m/s^3, MPC_Z_V_AUTO_UP 3.0 / DN 1.5 m/s; limits MPC_XY_VEL_MAX
  12.0 m/s; manual mode MPC_VEL_MANUAL 10 m/s, MPC_ACC_HOR_MAX 5.0, MPC_JERK_MAX 8.0.
  https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_pos_control ; jerk-limited S-curve trajectory description:
  https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory.html . An older, not re-verified mirror listed 5 / 10 / 20, so
  tuned airframes exceed the defaults: stock = the yaml values, `route_fast` = up to ~2x [est].
- Dryden low-altitude gust model, MIL-STD-1797A (1990, pp. 673, 678-685), via https://en.wikipedia.org/wiki/Dryden_Wind_Turbulence_Model
  and https://en.wikipedia.org/wiki/Continuous_gusts : L_w = h/2, L_u = 2 L_v = h/(0.177+0.000823 h)^1.2 (h in ft AGL, clamped at 10 ft),
  sigma_w = 0.1 W20, sigma_u/sigma_w = sigma_v/sigma_w = 1/(0.177+0.000823 h)^0.4. Implemented with first-order shaping per axis
  (frozen-field time constant L/V, V = max(wind speed, 1 m/s)) [est: the exact v/w filters are second order]; W20 is taken as the
  backyard mean wind of 0.5-3 m/s [est]; `gain` 1-3 scales sigma [est] because the formal sigma (0.1 W20) is only 0.05-0.3 m/s at these
  wind speeds. Discrete gust shape: 1-cosine, amplitude and length [est].
- Bird wingbeat: C. J. Pennycuick (1996), J Exp Biol 199(7):1613-1618, https://doi.org/10.1242/jeb.199.7.1613 (abstract read; fitted on 47
  species; f scales as m^(-1/6) between geometrically similar birds). The equation f = m^(3/8) g^(1/2) b^(-23/24) S^(-1/3) rho^(-3/8)
  (SI) is **recalled from the paper, not re-read**. Species (m kg, b m, S m^2): pigeon 0.35 / 0.65 / 0.07 gives 7.2 Hz, gull 0.8 / 1.4 / 0.2 gives
  3.2 Hz, small passerine 0.03 / 0.25 / 0.012 gives 12.9 Hz (all [est] inputs). Flap-bounding of small birds alternates flap bursts and folded-wing
  intervals (https://en.wikipedia.org/wiki/Bird_flight); burst and glide durations, heave amplitude and turn-shock statistics are [est].
- Not sourced, simple drag/gust models marked [est]: balloon response time, buoyancy drift, pendulum wind drag, debris flutter and sink rates, sizes
  (drone 0.25-0.6 m, balloon 0.3-0.9, hanging 0.1-0.4, bird 0.2-0.5, debris 0.05-0.4).

## Suite (`threats.NAMES`, seed-deterministic, 20 s, 401 frames)

| name                            | sensor          | objects                                                                                                | what it stresses                          |
| ------------------------------- | --------------- | ------------------------------------------------------------------------------------------------------ | ----------------------------------------- |
| `route_hover`                   | pos             | 1 drone, stock waypoint route with hovers                                                              | stop-and-go, CV vs manoeuvre              |
| `route_fast`                    | pos             | 1 drone, tuned (vmax 8-15)                                                                             | high speed, harder turns                  |
| `orbit`                         | pos             | 1 drone loitering                                                                                      | constant turn rate, altitude bob          |
| `survey`                        | pos             | 1 drone lawnmower                                                                                      | periodic 180-degree turns                 |
| `weave`                         | pos             | 1 drone random weaving                                                                                 | stochastic manoeuvre (Singer)             |
| `swarm_v`                       | pos             | 6 drones, V to line                                                                                    | close spacing, formation change           |
| `swarm_split`                   | pos             | 8 drones grid, 3 split off and merge                                                                   | split/merge, ID switches                  |
| `balloons`                      | pos             | 3 balloons in gusting wind                                                                             | slow, wind-correlated drift               |
| `hanging`                       | pos             | 2 swinging loads                                                                                       | oscillatory low-speed motion              |
| `birds`                         | pos             | 3 birds + 1 drone                                                                                      | erratic turns, flap/glide speed changes   |
| `clutter_mix`                   | pos             | drone, balloon, bird, leaf, 2 bags; births/deaths, occlusion gaps, Pd 0.85                             | clutter-like targets                      |
| `busy`                          | pos             | 2 weavers, 4-drone swarm, 2 birds, balloon, hanging; births/deaths, gaps                               | 10 objects                                |
| `cam_mix`                       | cam             | drone, bird, balloon; births/deaths, gaps                                                              | size-range camera, FOV exits              |
| `close_cam` / `close_cam_lidar` | cam / cam+range | hanging load + slow drone at 3.5-8 m                                                                   | fusion check (range inside TF-Luna reach) |
| `random`, `random_cam`          | pos / cam       | 1-5 objects, label uniform, every parameter randomised, random windows/gaps, Pd 0.8-0.95, clutter 2-10 | ML training and held-out data             |

## Checks (tests/python/test_threats.py)

Determinism in (name, seed), `_lidar` shares truth, labels/shapes/finite truth for every scene, every random-mode generator below 16 m/s
and inside ~120 m with multirotor acceleration bounds, Pennycuick sanity (pigeon 4-9 Hz, gull slower), births/deaths present in `busy`,
a discrimination smoke test, and the unchanged-baseline pin of the six existing `napl.track` scenarios (truth sum, measurement sum, frame count).
