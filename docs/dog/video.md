# The dog video (SP6): construction, programming overview, simulation

DRAFT recording of 2026-10-19. The video, its frames and its taps live on hfbox under `/workspace/wt/napl-skydio/out/dog/` and are never committed. The simulation
part is a draft: it is re-recorded after the locomotion terrain round. Recorder: `tools/record_dog.py`; tests: `tests/python/test_dog_video.py`.

## What is on screen and what is real

| part                      | on screen                                                                                                                                                                                                                     | what is real                                                                                                                                                                  | what is a stand-in                                                                                               |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| title, cards              | what the dog is, "recon and mapping only"                                                                                                                                                                                     | the numbers on the cards come from files                                                                                                                                      | none                                                                                                             |
| 1 Build (9-53 s)          | the napl viewer, build workspace on `assemblies/dog/dog.toml`: the chassis, one leg opened as its own document (parts revealed in build order), the leg used four times, head, power, compute; the orbit camera; mass and DRC | the Design backend, the parts library, the viewer, the DRC and mass numbers (11.65 kg, 85 parts, 5,553.76 USD, DRC 0 errors 1 warning)                                        | none                                                                                                             |
| 1 Wiring (53-101 s)       | pin-level wiring sheets (power: packs, BMS, contactor, PDB, one leg hub; the CAN chain of the front-left leg), hide-parts demo, then 57 wires as tubes in 3D, orbiting                                                        | the wires of the design, routed through free space by the viewer's router (tubes with their gauge's diameter, none through a part)                                            | none                                                                                                             |
| 1 Program (101-130 s)     | the PROGRAM workspace on `examples/dog` (`loco/controller.py`, `autonomy/planner.py`)                                                                                                                                         | the real files                                                                                                                                                                | none                                                                                                             |
| 1 Exploded view (12 s)    | the built MJCF with each sub-assembly's topmost body displaced, orbit camera                                                                                                                                                  | the built model                                                                                                                                                               | a render of a displaced copy of the MJCF; the wires are not drawn (the viewer has no explode op)                 |
| 2 Programming (130-237 s) | seven block diagrams: estimator, MPC + WBC, gait and footholds, elevation map, localization, planner, executive                                                                                                               | every number is read from the tap of the ruins run shown in part 3 (`videotap.py`), the rates from `loco.controller` and `autonomy.toml`, the file names are the repository's | the diagrams are schematic drawings                                                                              |
| 3 Simulation (237-325 s)  | the ruins showcase run replayed tick-exact (orbit camera, real time for 25 s then 4x, map shots with the mapped cells, planned path, trail and the estimated against the true pose), then the rubble short                    | `replay.naplog` applied to a fresh World; the replay hash is checked against the run's (exit code 1 otherwise)                                                                | lidar scan pattern is the published field of view and rosette; footfall contact is virtual; terrain is generated |
| results cards (325-355 s) | Monte Carlo and criteria, then "what is real and what is a stand-in"                                                                                                                                                          | `docs/dog/missions.json` (pending: the draft says so on the card)                                                                                                             | none                                                                                                             |

Showcase runs (seeds fixed before the runs, no search): `fam_ruins` seed 7002, `fam_rubble` seed 7001, both with the `[dog]` limits 0.4 m/s / 0.3 rad/s.

| run                           | outcome                                                                                                                               | replay                                             |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| ruins 7002 (30 m, flat floor) | walked 207 s, then fell (`never_falls` FAIL); coverage 35.7 %, map RMS 1.98 cm, localization error at most 13 cm, did not return home | tick 207100, hash a95f02b26e1b9330, replay MATCHES |
| rubble 7001 (30 m)            | fell at 15.1 s; coverage 13.5 %, map RMS 3.2 cm                                                                                       | tick 15100, hash 4146133955fc0207, replay MATCHES  |

The video shows both runs as they are; the captions state the outcome ("fell at 207 s", "fell at 15 s").

## Commands (hfbox, from the worktree; `export NAPL_REMOTE_DIR=/workspace/wt/napl-skydio`, `O=$NAPL_REMOTE_DIR/out/dog`)

Setup once: the C++ whole-body controller (`cmake --preset dev -DNAPL_LOCO_CPP=ON -DPython_EXECUTABLE=$PWD/.venv/bin/python && cmake --build --preset dev --target loco_wbc_cpp`; rerun it after
every `tools/install_dev.sh`, which reconfigures with `NAPL_LOCO_CPP=OFF`), `python3 -m venv $NAPL_REMOTE_DIR/vidvenv && $NAPL_REMOTE_DIR/vidvenv/bin/pip install pillow numpy`, and
Pillow in the test venv (`uv pip install --python .venv/bin/python pillow`) so the diagram tests run instead of skipping.

```
# the showcase runs, with a tap (OMP_NUM_THREADS=1); move the tap into the run dir afterwards
DOG_TAP_DIR=$O/showcase_ruins/tap NAPL_PARAMS='{"seed":7002}' .venv/bin/napl program examples/dog/missions/fam_ruins/fam_ruins.scenario.toml --runs-dir $O/showcase_ruins
DOG_TAP_DIR=$O/showcase_rubble/tap NAPL_PARAMS='{"seed":7001}' .venv/bin/napl program examples/dog/missions/fam_rubble/fam_rubble.scenario.toml --runs-dir $O/showcase_rubble
mv $O/showcase_ruins/tap $O/showcase_ruins/dog-fam-ruins-*/tap      # likewise for rubble
# part 1
.venv/bin/python tools/record_dog.py oplog $O/oplog.json
tools/remote.sh run --gpu '.venv/bin/python tools/record_turret.py build $O/oplog.json $O/build --open assemblies/dog/dog.toml --program-root examples/dog --timeout 2800'
tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py explode $O/explode'
# part 2 (CPU) and part 3 (GPU, exit 1 on a replay mismatch)
.venv/bin/python tools/record_dog.py diagrams $O/showcase_ruins/dog-fam-ruins-* $O/diagrams
tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py sim $O/showcase_ruins/dog-fam-ruins-* $O/showcase/sim'
tools/remote.sh run --gpu '.venv/bin/python tools/record_dog.py sim $O/showcase_rubble/dog-fam-rubble-* $O/short_rubble/sim'
# encode (video venv; GStreamer VA AV1 .webm, LGPL; ffmpeg / libx264 only with --gpl)
tools/remote.sh run --gpu '../vidvenv/bin/python tools/record_dog.py encode $O $O/dog_e2e_draft.webm --speedup 1.5'
../vidvenv/bin/python tools/record_dog.py png $O/build $O/png 200 500 900      # frames as PNG, to look at (a sim dir gets its HUD)
```

## Measured (draft)

| item         | value                                                                                                                                                                       |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| output       | `dog_e2e_draft.webm`, 1280 x 828 (720 picture + 108 caption strip), 20 fps, 354.9 s, 58.2 MB, encoder gst-va (AV1)                                                          |
| timeline (s) | title 0, part 1 5, build 9, wiring 53, program 101, part 2 130, part 3 237, results 325, stand-ins 341                                                                      |
| frames       | build capture 2179 files (viewer capture, 1280 x 720), explode 40, diagrams 2080 (104 s), ruins sim 1410 (70.5 s: 25 s real time + 182 s of run at 4x), rubble sim 302      |
| GTT          | 185-272 MB used at rest and between jobs (the card's `mem_info_gtt_used`), 211-250 MB around the render jobs; limit 4 GB; the encode job printed 185196544 before and after |
| wall         | viewer capture about 4 min, explode 0.5 min, diagrams 0.6 min, ruins sim 11 min, rubble sim 1.4 min, encode about 3 min (one GPU job at a time)                             |
| mission runs | ruins 7002: 838 s wall for 207 s of sim (0.25 sim-s per wall-s); rubble 7001: 118 s wall for 15.1 s                                                                         |

## Known limits

- The exploded view has no wires (a displaced part would drag straight wire chords through other parts).
- The sim part shows two runs that fall. It is a draft: the ruins clip is replaced after the locomotion terrain round, and the Monte Carlo card is empty until `docs/dog/missions.json` exists.
- The slopes family is not shown; the diagrams are schematic and read their numbers from one run.
- The leg document's `sub.open` / `sub.close` brings the whole dog back on close (seen in the first capture), so the op log hides the not-yet-built groups again.
- `tools/record_turret.py build` needed its `--program-root` forwarding (commit 201d547) so the PROGRAM workspace opens `examples/dog` instead of the assembly's folder.
