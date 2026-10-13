# Skydio-style quadcopter end-to-end video: how it is made

Media is never committed. Frames, run directories and the webm stay on the host (hfbox, `/workspace/wt/napl-skydio/out/skydio/` for the worktree `napl-wt/skydio`; locally `./remote-out/` after `tools/remote.sh pull`, gitignored).
Everything is driven by `tools/record_skydio.py`; it reuses `tools/record_turret.py` (xvfb viewer capture of the Design backend, `build --open`), `tools/record_cuas_lite.py` (cards, caption strip, build-frame composer) and `tools/videnc.py`
(GPU encode, GStreamer VA AV1 webm; nothing GPL unless `--gpl`). Pattern: docs/cuas-lite/video.md.

## What is on screen (and what is not real)

| segment | what | source of truth |
|---|---|---|
| Build | `assemblies/skydio.toml` (X500 V2 + D435i + Pi 5 + regulator) opened in the viewer, parts revealed in six build stages, orbiting camera, mass + DRC | the finished design (BROWSER eye hides / shows parts), not placed one by one |
| Wiring | three pin-level WIRING sheets (power, motors / PWM, companion computer + crossed UART) on the full-page canvas: `ui.wire_pick` ticks the components in the Wire Components picker (name-only pinout preview), `flatten.begin`, legend + controls top-left, zoom % in the top bar, `ui.wire_view panel fold / open`, then pm02, pdb and the four ESCs are hidden (`ui.visibility`): their cards and wires vanish in 2D and the tubes in 3D, then shown again; finally the 21 wires as tubes in 3D | `flatten.begin` on the assembly's own wires |
| Program | the PROGRAM IDE driven by `prog.ui`: `planner.py` highlighted, find / replace / find in files windows, a pin hover tooltip in `build_design.py` (`pdb.out1`), typing in an untracked `examples/skydio/_scratch.py`, the TERMINAL tab (`ls`, `python -c`), Run slot BENCH on `esc1` (OUTPUT table, PASS chip), then FULL SIM on `follow.scenario.toml`: the window's own World flies the scenario live on SIMULATE (LIVE label, the viewer's own chase camera, not the director), back on PROGRAM the chip and the monitors; a pipeline card | the files, the real Run slot |
| Simulate | goal (seed 1) and follow (seed 1) `napl program` runs, replayed (`replay.naplog` on a fresh World, final hash checked), directed camera (orbit, chase, side, tripod pans, pair), HUD | the run's `trace.csv`, `telemetry.jsonl`, `depth.npz` |
| Result | the runs' own numbers + the README's Monte Carlo and PX4 figures (quoted, not rerun) | `summary.json`, `result.json`, README |

HUD: mode, speed, altitude, nearest depth return (from the depth image with the ground removed; ray angles approximate), truth clearance (scoring only), follow distance, safe primitives / candidates, map voxels, estimate error, the planned primitive (green) and trail (white) projected into the view, the camera's colour view (a render) and the 160 x 90 depth image the planner sees.

Stand-ins, also said on the closing card: the VIO is a statistical model, the detector is classical on depth with no decoys, the depth image is sub-sampled, the person is a scripted walker, the forest is generated primitives, the colour inset is a render the planner never sees.
Not in the video: the PX4 SITL scenarios (not run for this video; their README figures are quoted on the result card). Known cosmetic limits: in the "every part hidden" step a ~1 cm yellow block and a dark speck remain (not identified; not a part, the browser shows every instance hidden). The camera director (`Director` in `tools/record_skydio.py`) keeps the quad in line of sight through the generated forest (cylinder / sphere / box ray tests, scene geometry used only to place the camera): the quad is unobstructed in 96 % (goal) and 93 % (follow) of frames, so a few frames still have a trunk across it.
Viewer fix behind the build segment (`src/viewer/viewer.cpp`): the browser eye hid a geom only when its origin lay inside its instance's box, so propeller hubs 5 mm above the 4 mm thick prop box stayed visible, and cables were never hidden; geoms now fall back to the nearest instance box within 3 cm and cables are dropped when an end sits on a hidden instance. The op log no longer calls `ui.frame` (the viewer's frame event selects what it frames) and turns tooltips off.
Slow motion: 1.5 s around the closest pass to an obstacle in each run (minimum truth clearance while flying), played at 1/4 speed with a HUD badge.

## Commands (from the worktree; `export NAPL_REMOTE_DIR=/workspace/wt/napl-skydio UV_CACHE_DIR=/workspace/wt/napl/uv-cache`; `O=$NAPL_REMOTE_DIR/out/skydio`)

```sh
# 0. host setup: current kernel build + editable install; a throwaway venv with Pillow + numpy for the encode / png steps.
#    The editable install REBUILDS the extension and installs its own copy under .venv: give it NAPL_SCCACHE=OFF too, or stale sccache objects
#    (after a changed solver.h) segfault in Sim.start_log and every napl program run fails.
tools/remote.sh run --slots 2 'cmake --preset dev -DNAPL_SCCACHE=OFF && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH uv pip install -e ".[dev,px4]" --config-settings=cmake.define.NAPL_SCCACHE=OFF'
# the Program segment types into examples/skydio/_scratch.py (untracked, one comment line): create it locally before the first remote.sh call, delete it after
# the Program segment's FULL SIM step needs the viewer's Backend.sim (set by tools/record_turret.py _build) and takes about 45 s wall for follow
python3 -m venv $NAPL_REMOTE_DIR/vidvenv && $NAPL_REMOTE_DIR/vidvenv/bin/pip install pillow numpy
# 1. the scenarios (CPU, ~40 s each): PASS + replay match; symlink the run dirs as $O/goal and $O/follow
tools/remote.sh run ".venv/bin/napl program examples/skydio/goal.scenario.toml --runs-dir $O/runs"      # hash efb57e183637ae9d
tools/remote.sh run ".venv/bin/napl program examples/skydio/follow.scenario.toml --runs-dir $O/runs"    # hash 164b5cc71536dfec
# 2. the sim frames (GPU, ~1 min each; ends with the replay hash check, exit 1 if it differs)
for m in goal follow; do tools/remote.sh run --gpu ".venv/bin/python tools/record_skydio.py sim $O/$m $O/$m/sim"; done
# 3. build / wiring / program: op log (CPU) then the viewer capture (GPU, xvfb; ~3 min)
tools/remote.sh run ".venv/bin/python tools/record_skydio.py oplog $O/oplog.json"
tools/remote.sh run --gpu ".venv/bin/python tools/record_turret.py build $O/oplog.json $O/build --open assemblies/skydio.toml --timeout 2800"
# 4. encode (GPU: GStreamer VA, AV1 webm)
tools/remote.sh run --gpu "../vidvenv/bin/python tools/record_skydio.py encode $O $O/skydio_e2e.webm --speedup 1.5"
# looking at frames with their HUD: ../vidvenv/bin/python tools/record_skydio.py png $O/goal/sim $O/png 10 150 300
```

## Measured (hfbox)

1280x828 @ 20 fps, 253.55 s, AV1 webm, 54.1 MB; GTT 0.34 GB at rest, 0.37 GB peak (limit 4 GB); sim frames render about 10 per second.
After a behaviour change the hashes move: rerun steps 1, 2, 4 (the build segment only depends on the assembly).
