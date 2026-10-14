# Skydio landing end-to-end video: how it is made

Media is never committed. Frames, run directories and the webm stay on the host (hfbox, `/workspace/wt/napl-video/out/land/`); the final file is pulled to `./remote-out/videos/skydio_land_e2e.webm` (gitignored).
Everything is driven by `tools/record_skydio_land.py`, the landing version of `tools/record_skydio.py` (see `docs/skydio/video.md`); it reuses `tools/record_turret.py` (xvfb viewer capture), `tools/record_cuas_lite.py`
(cards, caption strip, build-frame composer) and `tools/videnc.py` (GPU encode, GStreamer VA AV1 webm; nothing GPL unless `--gpl`).

## What is on screen

| segment | what | source of truth |
|---|---|---|
| Build | an EMPTY design in the viewer: the X500 V2 bottom plate, then every part of `assemblies/skydio-land.toml` placed in seven stages (library search typed, animated drag and drop onto the mate ports for the first part of each kind, `design_edit` batches for the repeats and the loop-closing mates, the vision payload and the landing sensors at their poses), chips without a body, parameter provenance, 26 device bindings, DRC | the op log generated from the assembly (`oplog`); `build --expect` checks that the design the window built equals the committed assembly (parts, poses to 1 um, mass, mates, wires, devices) |
| Wiring | four pin-level sheets on the full-page canvas: wire picker, `flatten.begin`, every wire drawn pin to pin with `flatten.wire` (sheet 4 is new: the TF-Luna's power and UART on the Pi 5's 40-pin J8 header), legend + controls top-left, `ui.wire_view` zoom, panel fold, hide a part and its card and wires vanish in 2D and 3D, then the 24 wires as tubes in 3D | the Design's own wire ops |
| Program | the PROGRAM IDE on `examples/skydio-land` via `prog.ui`: `land_fly.py` highlighted, go to definition (`GroundMap` -> `land.py`), find / replace / find in files, a pin hover tooltip in `build_design.py`, typing and F2 rename in a scratch file (created in the program root for the session, removed after), the TERMINAL tab, Run slot BENCH on the `lidar` (TF-Luna model), the Run slot's scenario combo stepped through all 8 scenarios, then FULL SIM of `forest/clearing` live in SIMULATE (chase camera), the verdict chip and OUTPUT | the real Run slot (`run.start`), the window's own World |
| Simulate | the 8 scenarios (`forest/clearing`, `rain/rain25`, `rain/rain100`, `summit`, `ledge`, `mesa_brink`, `deck/osv`, `deck/asv`, seed 1) as `napl program` runs, replayed tick-exact (`replay.naplog` on a fresh World, final hash checked), directed camera (orbit, chase, side, high, tripod at touchdown), HUD; cruise legs are sped up and labelled (3-4x), descent in real time, 1/4 slow motion at the first gear contact; zone rejections, gust front and touchdown get banners; every scenario ends on the monitors' verdict panel | the run's `trace.csv`, `telemetry.jsonl`, `depth.npz`, `result.json` |
| Result | the 8 runs' own numbers, the README's Monte Carlo table (quoted, not rerun) and its limits | `summary.json`, `result.json`, `examples/skydio-land/README.md` |

## Commands (from the worktree; `export NAPL_REMOTE_DIR=/workspace/wt/napl-video`; `O=$NAPL_REMOTE_DIR/out/land`)

```sh
# 0. build + editable install (see docs/ for the current command); a throwaway venv with Pillow + numpy for the encode / png steps
tools/remote.sh run --slots 2 'cmake --preset dev && cmake --build --preset dev && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache tools/install_dev.sh'
python3 -m venv $NAPL_REMOTE_DIR/vidvenv && PATH=/workspace/wt/napl/env/bin:$PATH UV_CACHE_DIR=/workspace/wt/napl/uv-cache uv pip install --python $NAPL_REMOTE_DIR/vidvenv/bin/python pillow numpy   # the venv has no pip
# 1. the scenarios (CPU; 2-4 min forest / rain / terrain, ~8 min the OSV): PASS + replay match; the run dirs are linked as $O/<forest rain25 rain100 summit ledge mesa_brink osv asv>
#    ($NAPL_REMOTE_DIR is NOT set inside a remote job: write the absolute /workspace/wt/napl-video/out/land)
tools/remote.sh bg scn 'O=/workspace/wt/napl-video/out/land; for p in forest/clearing:forest rain/rain25:rain25 rain/rain100:rain100 summit/summit:summit ledge/ledge:ledge mesa_brink/mesa_brink:mesa_brink deck/osv:osv deck/asv:asv; do
  .venv/bin/napl program examples/skydio-land/${p%%:*}.scenario.toml --runs-dir $O/runs; d=$(ls -dt $O/runs/*/ | head -1); ln -sfn ${d%/} $O/${p##*:}; done'
# 2. the sim frames (GPU, one run at a time; ends with the replay hash check, exit 1 if it differs)
tools/remote.sh run --gpu ".venv/bin/python tools/record_skydio_land.py sim $O/forest $O/forest/sim"          # ... and the other seven
# 3. build / wiring / program: op log (CPU, no window: `explore` is the dry run that must end 'final design == expected') then the viewer capture (GPU, xvfb)
tools/remote.sh run ".venv/bin/python tools/record_skydio_land.py oplog $O/oplog.json"
tools/remote.sh run ".venv/bin/python tools/record_skydio_land.py explore $O/oplog.json --expect assemblies/skydio-land.toml"
#    ~50 min (the live forest run is 230 s wall plus the replay check, FULL_WAIT = 380 s). Do NOT run another tools/remote.sh command (it rsyncs with --delete) while this runs:
#    it deletes the untracked examples/skydio-land/_scratch.py the typing / rename demo needs (poll with plain ssh instead)
tools/remote.sh bg --gpu vbld ".venv/bin/python tools/record_skydio_land.py build $O/oplog.json $O/build --expect assemblies/skydio-land.toml --timeout 4000"
# 4. encode (GPU: GStreamer VA, AV1 webm); some frames as PNG to look at:
tools/remote.sh run --gpu "../vidvenv/bin/python tools/record_skydio_land.py encode $O $O/skydio_land_e2e.webm"
../vidvenv/bin/python tools/record_skydio_land.py png $O/forest/sim $O/png 10 400 800
```

## Found while recording

- `flatten.wire` with an explicit lead (`part`) dropped the `connectors` of the wire (the pinned XT60 / XT30 / JST parts): fixed in `python/napl/design/wire2d.py`, test `test_explicit_lead_keeps_its_pinned_connectors`.
- A live FULL SIM ended `failed: replay diverged ... permission_denied` although the flight passed (same state hash as `napl program`): the viewer World's replay log starts with the window's own design `LoadModel`, and `napl-replay` was run with `--root <run>/model`. `program.run(live=)` now carries the Sim's root (`runops` passes `b.out`) and `program._replay_root` uses it; test `test_live_replay_check_covers_the_windows_own_design_load`.
- `tools/record_cuas_lite.py`: the chapter map had no entry for the Library page (`LIBRARY` steps), added as part of the build chapter.
- The PDB's `TAP` solder pads (assemblies/skydio.toml taps the rail there for the 5 V regulator) were not marked `joinable`, so the editing ops refused `from_pins = ["TAP.1", "TAP.2"]` on a wire from `pdb.out1` although the assembly loader accepts it, and the DRC flagged `PIN_ALREADY_USED` when the tap was drawn without them: `parts/drone/holybro-pdb-60a.toml` marks the connector joinable (solder pads, `parts/README.md`), test `test_a_tap_on_the_pdb_solder_pads_can_be_drawn`.
