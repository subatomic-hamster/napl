# Skydio-style quadcopter end-to-end video: how it is made

Media is never committed. Frames, run directories and the webm stay on the host (hfbox, `/workspace/wt/napl-skydio/out/skydio/` for the worktree `napl-wt/skydio`; locally `./remote-out/` after `tools/remote.sh pull`, gitignored).
Everything is driven by `tools/record_skydio.py`; it reuses `tools/record_turret.py` (xvfb viewer capture of the Design backend, `build --open`), `tools/record_cuas_lite.py` (cards, caption strip, build-frame composer) and `tools/videnc.py`
(GPU encode, GStreamer VA AV1 webm; nothing GPL unless `--gpl`). Pattern: docs/cuas-lite/video.md.

## What is on screen (and what is not real)

| segment | what | source of truth |
|---|---|---|
| Build | `assemblies/skydio.toml` (X500 V2 + D435i + Pi 5 + regulator) opened in the viewer, parts revealed in six build stages, orbiting camera, mass + DRC | the finished design (BROWSER eye hides / shows parts), not placed one by one |
| Wiring | three pin-level WIRING sheets (power, motors / PWM, companion computer + crossed UART), then the 21 wires as tubes in 3D | `flatten.begin` on the assembly's own wires |
| Program | PROGRAM page on `mapping.py`, `planner.py`, `fly.py`, a scenario file; a pipeline card | the files |
| Simulate | goal (seed 1) and follow (seed 1) `napl program` runs, replayed (`replay.naplog` on a fresh World, final hash checked), directed camera (orbit, chase, side, tripod pans, pair), HUD | the run's `trace.csv`, `telemetry.jsonl`, `depth.npz` |
| Result | the runs' own numbers + the README's Monte Carlo and PX4 figures (quoted, not rerun) | `summary.json`, `result.json`, README |

HUD: mode, speed, altitude, nearest depth return (from the depth image with the ground removed; ray angles approximate), truth clearance (scoring only), follow distance, safe primitives / candidates, map voxels, estimate error, the planned primitive (green) and trail (white) projected into the view, the camera's colour view (a render) and the 160 x 90 depth image the planner sees.

Stand-ins, also said on the closing card: the VIO is a statistical model, the detector is classical on depth with no decoys, the depth image is sub-sampled, the person is a scripted walker, the forest is generated primitives, the colour inset is a render the planner never sees.
Not in the video: the PX4 SITL scenarios (not run for this video; their README figures are quoted on the result card). Known cosmetic limits: parts hidden in the build segment still show props and wire tubes (the viewer's hide acts on instances), selection outlines flash on a few reveal frames, and in the "side" shots tree trunks sometimes occlude the quad.

## Commands (from the worktree; `export NAPL_REMOTE_DIR=/workspace/wt/napl-skydio UV_CACHE_DIR=/workspace/wt/napl/uv-cache`; `O=$NAPL_REMOTE_DIR/out/skydio`)

```sh
# 0. host setup: current kernel build + editable install; a throwaway venv with Pillow + numpy for the encode / png steps
tools/remote.sh run --slots 2 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH uv pip install -e ".[dev,px4]"'
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

1280x828 @ 20 fps, 247.7 s, AV1 webm, 50.9 MB; GTT 0.34 GB at rest, 0.38 GB peak (limit 4 GB); sim frames render about 10 per second.
After a behaviour change the hashes move: rerun steps 1, 2, 4 (the build segment only depends on the assembly).
