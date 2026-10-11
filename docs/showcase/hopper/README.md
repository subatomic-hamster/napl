# EDF jet-vane hopper: build, wiring, programming, simulation (showcase video)

A 5-minute screen recording (1600x900, VP9) of the napl viewer: the Grasshopper-style hopper of `assemblies/hopper.toml` (1.29 kg, 33 N ducted fan, four jet vanes on smart
servos, Teensy 4.0 flight computer with IMU, barometer, lidar and RTK GNSS) is built from an empty design, wired pin by pin, its flight software is read, and its flight is
replayed with a HUD. The video file is not in git (no media is committed): it is produced on the build host as `out/hop/hopper.webm`.

## What is shown

| Time | Segment | What happens on screen |
|---|---|---|
| 0:00 | title | what the video is and where the decisions are documented (this file) |
| 0:12 | build (library and 3D) | from an empty design in the CAD-style UI: frame, duct, motor, fan on a spin joint, ESC, battery (moved 14.8 mm to balance the CG), four servos and vanes, four legs, Teensy, two regulators, harness, lidar; the chips that have no body; colours; camera orbits after the big steps; live mass and DRC in the status bar |
| 2:00 | wiring | the 2D KiCad-like sheet with real connector pins; the power chain; **a deliberate pin mistake** (pack plus to the Teensy VIN) that the DRC rejects with `PIN_VOLTAGE_MISMATCH: BAT+ drives 25.2 V, above the Vin range 3.6..5.5 V`; the fix; servo chains and logic feeds; back in 3D the conductors are tubes routed around the parts; device bindings; DRC 0 errors |
| 3:00 | programming | the PROGRAM page: `flight.py` (state machine), `estimator.py`, `trajectory.py`, `control.py` (attitude loop, vane allocation, thrust loop), and `divert25.scenario.toml` (the mission and its judges) |
| 3:33 | simulation | the divert25 flight (25 m climb, 8 m east / 6 m north to a second pad, descent, landing) as the replay of the run the kernel judged: HUD with flight state, estimator values, vane angles, altitude profile, ground track; follow, wide side and low cameras; slow motion at lift-off and touchdown; then a gusts run (5 m/s wind with gusts, a vane servo losing 65 % of its torque at 14 s, a 1.5 V pack step at 17 s) |
| 4:45 | result | the project's Monte Carlo table (below), and the replay hashes of the two flights shown |

## Results (from `examples/hopper/README.md`, not produced by this video)

Held-out seeds, each flown once (report seeds 13000+), simulated vehicle, nothing on a test stand.

| Mission | Seeds | Success [95 % Wilson] | Touchdown speed p50/90/99 m/s | Pad distance p50/90/99 m |
|---|---|---|---|---|
| hop10 (10 m hop, hover, land) | 150 | 99.3 % [96.3, 99.9] | 0.27 / 0.30 / 0.34 | 0.03 / 0.06 / 0.08 |
| divert25 (25 m, 10 m divert, wind up to 3 m/s) | 300 | 99.3 % [97.6, 99.8] | 0.24 / 0.34 / 0.48 | 0.05 / 0.09 / 0.15 |
| gusts (5 m/s + servo fault + 1.5 V step) | 200 | 87.5 % [82.2, 91.4] | 0.24 / 0.29 / 0.34 | 0.11 / 0.33 / 10.4 |

The two flights in the video are single seed-1 runs, not the Monte Carlo. Their replay ends on the state hashes the kernel recorded (divert25 `8fefff2e164b775e`,
gusts `777a7d76b2a70483`; hop10 `93cc314eb1d458f0` was re-run for this stage and matches the documented hash as well).

## Who decided what

* Before this video (earlier stages, not redone here): the vehicle (`assemblies/hopper.toml`: parts, poses, wires), the flight software, the three scenarios and their
  Monte Carlo. The video does not change any of it: `napl program` ran the unchanged scenarios and the hashes above are the documented ones.
* The video author (an LLM agent, Claude) chose: the order of the build, what each step says, the camera moves and where they pause, the pin mistake as the wiring
  demonstration, the pacing, which flight states get slow motion, the captions and the end card.
* **The build is a scripted replay, not live improvisation.** `oplog.json` is a list of the same requests a user's mouse and keyboard produce (library search, drop,
  mate, move, flatten, wire, ...). Where a placement or a parameter has to equal the reference design exactly (mate poses, rotations, part parameters, wire names), the
  op log refers to `assemblies/hopper.toml` and `record_hopper.py` expands the reference value at run time. The result is checked against the reference: the design built
  in the window equals `assemblies/hopper.toml` (parts, poses `max_pose_diff` 3.5e-7 m, mass 1.29133524 kg, DRC 0 errors and 0 warnings); the build fails otherwise.
* **The flown design is the reference assembly, not the saved built one.** The simulation segment replays runs of the scenarios as they are in the repo (`scene.py` builds
  the reference assembly). The built design matches it to the numbers above, but it was not flown separately.

## How it is made

All on the build host (the laptop only edits and syncs); outputs live outside the synced tree.

1. `napl program examples/hopper/{divert25,gusts}.scenario.toml --runs-dir out/runs_ref`: the runs (`replay.naplog`, `telemetry.jsonl`, `native.json`).
2. `tools/record_hopper.py build` (headless viewer under Xvfb, llvmpipe): replays `oplog.json` through the real viewer Backend (`camera.orbit`, `ui.wire_view`, ...)
   and screen-grabs at 12 fps. The build is recorded in four parts (steps 0-19, 20-31, 32-39, 40-47; no part starts inside a wiring session), each loading the design
   the previous part saved; the last asserts the design equals `assemblies/hopper.toml`. `NAPL_ROUTE_CACHE=<dir>` keeps the pure-Python wire router's results between parts.
3. `tools/record_hopper.py frames`: the replay of a run on a `Host` (tick-exact, the recorded inputs), the HUD document per frame in lockstep with the viewer, the camera from
   `sim_shots.json`. Frames come in chunks of different rates: real time, 4x, and slow motion (60 or 40 fps for 1/3 or 1/2). The last chunk recomputes the state hash and fails
   if it differs from `native.json`.
4. `tools/record_hopper.py encode` (PyAV, VP9 only): title and chapter cards, the caption strip (`oplog.json` and `sim_captions.json`), per-step speed, the end card.
5. `tools/hopall.sh` chains 2-4 in one job; `tools/hopframes.sh` the frame chunks; `tools/hopwrap.sh` runs a command under `timeout` in its own process group;
   `tools/hopsample.py` extracts evenly spaced frames from the webm to read them back.

## Honest notes

* The landing-pad markings (rims, 0.3 m acceptance ring, cross), the trail and the pad labels are drawn by the HUD overlay, not put in the scene. Pad geometry added to the
  scene (even non-colliding, even in another geom group) changed the replay hash of divert25 (`8fefff2e164b775e` became `d961148e2c822d3b`), so it was not used; the overlay
  leaves the world untouched. The cause of that hash change was not investigated.
* The altitude and speed readouts are the flight computer's estimates (telemetry `est`); before the pad calibration is done they are null and the HUD shows dashes.
* The wiring zoom in the window is the sheet's fit view, so small pin labels are legible on the 1600x900 frame but not large. `ui.wire_view` zoom steps were not tuned further.
* Known cosmetic flaws: the step "Four skid legs" is shown for about a second with a featureless grey view (the camera sits on the legs' own axis); the step speed in the caption
  strip prints more decimals than needed (for example `1.5088x`).
* Fixed in the code for this stage (physics unchanged): `design/edit.py` (`assembly` accepts `seed`; `place` keeps a caller-given `rpy` as written; `wire` accepts a pin of a
  `joinable` connector that is not on the named port, as the assembly loader already did) and `design/wireroute.py` (per-wire routing cache, optional on-disk cache) with tests.
