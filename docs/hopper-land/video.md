# hopper-land end-to-end video: how it is made

Media is never committed. Frames, run directories and the webm stay on the host (hfbox, `$NAPL_REMOTE_DIR/out/hl/`; locally `./remote-out/` after `tools/remote.sh pull`, gitignored).
Everything is driven by `tools/record_hopper_land.py`; it reuses `tools/record_hopper.py` (`resolve`: the op log's references into an assembly), `tools/record_turret.py` (xvfb viewer capture of the Design backend),
`tools/record_cuas_lite.py` (cards, caption strip, build-frame composer), `tools/record_skydio.py` (`project`) and `tools/videnc.py` (GPU encode, GStreamer VA AV1 webm; nothing GPL unless `--gpl`).
Same shape as `docs/skydio/land_video.md`.

## What is on screen

| segment | what | source of truth |
|---|---|---|
| Build | `assemblies/hopper-land.toml` built from an EMPTY design in the viewer: the deck frame, duct, motor, fan on a spin joint, ESC, battery, four servos and vanes, four legs, Teensy 4.0, two regulators, the ZED-F9P RTK GNSS harness, the downward TF-Luna lidar, the body-less chips, colours, device bindings; library search, animated drop onto the mate ports, camera orbits, live mass and DRC in the status bar | the op log (`docs/showcase/hopper/oplog.json` resolved against `assemblies/hopper-land.toml`; the two assemblies are the same vehicle): the built design is compared with the assembly (parts, poses 3.5e-7 m, mass 1.29133524 kg, wires, devices, DRC 0 / 0) and the capture fails otherwise |
| Wiring | the WIRING page: the Wire Components picker (`ui.wire_pick`), the pin-level sheet (legend + controls top-left, zoom % in the top bar), power chain, a deliberate pin mistake (pack plus onto the Teensy VIN: `PIN_VOLTAGE_MISMATCH`) and its fix, servo chains and logic feeds, panel fold / open, parts hidden with the eye (their cards and wires vanish in 2D and the tubes in 3D) and shown again, three zoom pans to read the pin labels, then the wires as tubes in 3D | `flatten.*` / `ui.*` ops on the real Backend |
| Program | the PROGRAM IDE driven by `prog.ui`: `landflight.py` (class Flight), `landveh.py` (density-aware `t_full`), F12 on `LandEstimator`, find / replace / find in files on `k_dob`, a device hover on `/devices/lidar` in `landhop.py` (part + pin tooltip), typing and an F2 rename in an untracked scratch file, the TERMINAL tab (`ls`, `python _scratch.py`); Run slot BENCH on the `lidar` (rangefinder bench, PASS chip), the scenario combo stepped through all eight, FULL SIM of `mesa/down.scenario.toml` live in SIMULATE (LIVE label, FOLLOW VEHICLE chase camera, "LAST FULL RUN PASS" with the monitors), back on PROGRAM the chip and OUTPUT | the files, the real Run slot (`run.start` bench / full) |
| Simulate | the eight scenarios of the Run combo (`mesa/down`, `mesa/up`, `ridge/summit`, `ledge/lee`, `rain/rain25`, `rain/rain100`, `deck/scripted`, `osv/underway`), each a `napl program` run replayed tick-exact (`replay.naplog` on a fresh World, final hash checked), camera close to the 1.3 kg vehicle (establishing shot of the pad or ship over the calibration, orbit while climbing and descending, chase and side in transit, a low tripod at touchdown; the Sim's own raycast keeps the camera above the ground and with a clear line to the vehicle), HUD (phase chip + state strip, estimator and lidar readout, throttle / thrust, air density and T/W margin, wind, rain and gust banners, jet-vane gauges, touchdown banner) and the monitors' verdict panel at the end | the run's `trace.csv`, `telemetry.jsonl`, `result.json`, `native.json` |
| Result | the runs' own numbers and hashes, then the README's Monte Carlo tables quoted (not rerun), including the OSV envelope (Hs 0.25 / 0.5 m 87.5 %, 1.0 m 50 %, 1.5 m 37.5 %; 80 % at n = 40; the >= 90 % target NOT met on the ship), and a stand-ins card | `examples/hopper-land/README.md` |

Not real, also said on the closing cards: the scripted deck's motion and link noise are invented, rain catch / wet areas are estimates, `T_full` is extrapolated above 5500 m, standard-day air only, the terrain is synthetic.
The HUD shows the pad truth and the touchdown distance: scoring information the flight software does not have. Slow motion: 1/4 speed from 0.6 s before to 1.2 s after the first contact; calibration is 4x, transit and hover 2x, labelled on screen.

## Commands (from the worktree; `export NAPL_REMOTE_DIR=...`; `O=$NAPL_REMOTE_DIR/out/hl`)

```sh
# 0. host: kernel + editable install (see docs of the current stage); a throwaway venv with Pillow + numpy for encode / png:
#    python3 -m venv $NAPL_REMOTE_DIR/vidvenv && $NAPL_REMOTE_DIR/vidvenv/bin/pip install pillow numpy
# 1. the scenarios (CPU; goldens are in examples/hopper-land/README.md); symlink each run dir as $O/<name>:
#    mesa_down mesa_up ridge_summit ledge_lee rain25 rain100 deck_scripted osv_underway
tools/remote.sh run --slots 1 "export NAPL_ROUTE_CACHE=$O/rcache; for s in mesa/down mesa/up ridge/summit ledge/lee rain/rain25 rain/rain100 deck/scripted osv/underway; do .venv/bin/napl program examples/hopper-land/\$s.scenario.toml --runs-dir $O/runs; done"
# 2. sim frames per scenario (GPU lane; ends with the replay hash check, exit 1 if it differs)
for s in mesa_down mesa_up ridge_summit ledge_lee rain25 rain100 deck_scripted osv_underway; do tools/remote.sh run --gpu ".venv/bin/python tools/record_hopper_land.py sim $O/$s $O/$s/sim"; done
# 3. op log (CPU), then the viewer capture (GPU, xvfb, about 12 min). The Program segment types into examples/hopper-land/_scratch.py:
#    create it LOCALLY (same text as SCRATCH_TEXT) before the first remote.sh call of the capture and delete it after, or the next rsync --delete removes it from the host mid-capture
tools/remote.sh run ".venv/bin/python tools/record_hopper_land.py oplog $O/oplog.json"
tools/remote.sh bg --gpu build "export NAPL_ROUTE_CACHE=$O/rcache; .venv/bin/python tools/record_hopper_land.py build $O/oplog.json $O/build --expect assemblies/hopper-land.toml --timeout 2800 --save $O/built.toml"
#    (no remote.sh call while it runs; poll with plain ssh)
# 4. encode (GPU: GStreamer VA, AV1 webm)
tools/remote.sh run --gpu "$NAPL_REMOTE_DIR/vidvenv/bin/python tools/record_hopper_land.py encode $O $O/hopper_land_e2e.webm --speedup 1.6"
# looking at frames: png SIM_OR_BUILD_DIR OUT_DIR N...   (with the HUD for a sim dir)
```

`explore OPLOG --expect assemblies/hopper-land.toml` runs the build and wiring steps headless through the Backend and compares the result with the assembly (no window).
After a behaviour change of the flight software the hashes move: rerun steps 1, 2 and 4 (the build capture depends only on the assembly and the program files shown).

## Measured (hfbox, this run)

1280x828 @ 20 fps, 817 s, AV1 webm (GStreamer VA), 128 MB. Sim frames render about 10 per second (1288 frames for mesa_down). The capture holds the GPU lane about 12 min, the encode about 8 min.
