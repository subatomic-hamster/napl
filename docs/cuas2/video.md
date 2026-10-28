# cuas2 end-to-end video: how it is made

Media is never committed. Frames, run directories and the webm stay on the host (hfbox, `/workspace/wt/napl-c2video/out/` for the worktree `napl-wt/c2video`; locally `./remote-out/` after `tools/remote.sh pull`, gitignored).
Everything is driven by `tools/record_cuas2.py`. It imports `tools/record_cuas_lite.py` (cards and caption strip, the build-capture reader, the operator-page capture, the replay player and the auto-plan) and reuses `tools/record_turret.py`
(xvfb viewer capture of the Design backend), `tools/record_sim.py` (replay of a run in the viewer; `NAPL_PLAYER=record_cuas2:Cuas2Player` selects the cuas2 HUD) and `tools/videnc.py` (GPU encode; libx264 only with `--gpl`, nothing GPL is used by default).
Pattern: `docs/cuas-lite/video.md`. What differs from the cuas-lite video is only the content below.

## What is on screen (and what is not real)

| segment    | what                                                                                                                                                                                                                                                                                                                                                                                                  | source of truth                                                                                                                                                                                                                    |
| ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0 Cards    | title card, then "What is not real"                                                                                                                                                                                                                                                                                                                                                                   | this file                                                                                                                                                                                                                          |
| 1 Build    | `cad/cuas2/params.py` and `modules/feeder.py` on the PROGRAM page, then the 106 on-board parts of `assemblies/cuas2.toml` revealed in eight build stages (base and power box, pan drive, lazy susan + yokes + tilt drive, launcher cartridge, one-ball feeder, Brio sensor head, Jetson / Pico / TMC2209, weapon-rail safety electronics), orbiting camera, mass + DRC                                | the finished design opened in the viewer (the BROWSER eye hides / shows parts); parts are not placed one by one through the library                                                                                                |
| 2 Wiring   | five WIRING sheets with the real connector pins (power + key / E-stop / relay coil, weapon rail with ESCs and servos, supervisor + feed limiter + shot counter, Pico to the TMC2209 + steppers, Pico sensing), zoomed on the busiest cards, then the 153 wires as tubes routed around the parts in 3D                                                                                                 | `flatten.begin` on the assembly's own wires; the router (`wireroute.py`) keeps tubes out of solids                                                                                                                                 |
| 3 Program  | PROGRAM page on `feeder.c` (the gate state machine), `launcher.c` (flywheel loop, READY window), `fsm.c` (safety FSM), `python/napl/cuas2/app.py` (Ph gate), `examples/cuas2/run.py` (harness + Referee2); the real operator page in headless Chromium; the two state machines (safety FSM above, one-ball gate ring below) walked through the transitions of the `s1_balloon_static` run (two balls) | the files; `python/napl/lite/web.py` + `static/ui.html` served unchanged (cuas2 has no page of its own: it shows the lite fields, not Ph / READY); transitions from the run's `lite_meta.json` (`transitions`, `gate_transitions`) |
| 4 Simulate | `s1_balloon_static`, `s2_balloon_drift`, `s5_mix`, `s6_person_walk` (HUD: FIRING INHIBITED: PERSON IN VIEW), `s7_fault_estop` (HUD: FIRING INHIBITED: E-STOP PRESSED, MCU SAFE; the two state machines walked through the E-stop). HUD adds: Ph of the last decision against the gate, wheel rpm and WHEELS READY, FEED READY, gate state, balls fed / hopper left, weapon rail                       | lockstep runs of the real Jetson app (App2) + simulated RP2040 (Sim2RP2040) + the cuas2 weapon, judged by Referee2; replay of each `replay.naplog`, final state hash checked                                                       |

Not real, said on screen too (title-follow-up card, every Simulate caption, the HUD's bottom line, the gate-walk caption): the detector is the stand-in (ground-truth projection with the spec noise model), not YOLO; the sim camera has no pixels,
so the HUD's DETECTOR VIEW and the web page's feed are drawn from the detector's boxes; the RP2040 is a python twin of the C core (tests compare them tick by tick), not a Pico; firmware timings (gate travel, pusher stroke, feed cycle, wheel dip) are estimates until the bench items of
`bench_validation.md`; the simulated turret body is the cuas-lite CAD (the cuas2 assembly carries no launcher / camera devices into the sim yet), with the cuas2 launcher and feeder model behind it. In the web capture the page shows the run's recorded `App.snapshot()` stream and the clicks (sector, ARM, HOLD TO FIRE) are timed to the run's own script; they do not steer the run.
Belts are closed GT2 loops without a model, so they are described, not shown.

## One-time host setup (hfbox; throwaway, outside the repo)

The browser capture needs a venv with playwright + Pillow + numpy and Chrome's headless shell; the encode and png steps need only Pillow + numpy. They already exist under `/workspace/wt/napl-video/{webvenv,chrome}`; `tools/record_cuas_lite.py` points at that chrome
(`NAPL_CHROME` overrides); the commands below use `../webvenv` of the remote dir, which is a symlink to it:

```sh
export NAPL_REMOTE_DIR=/workspace/wt/napl-c2video UV_CACHE_DIR=/workspace/wt/napl/uv-cache
ssh hfbox 'ln -s /workspace/wt/napl-video/webvenv /workspace/wt/napl-c2video/webvenv'
cp -a /workspace/wt/napl-skydio/env /workspace/wt/napl-c2video/env             # on the host: the cmake / ninja / uv toolchain
tools/remote.sh bg --slots 2 install 'uv venv --python 3.12 .venv && source .venv/bin/activate && tools/install_dev.sh'   # builds the kernel (sccache), about 10 min
```

`run`, `plan`, `verify`, `sim`, `oplog` need the project venv (`.venv/bin/python`); `web`, `encode`, `png` run with `../webvenv/bin/python`.

## Commands (from the worktree; `O=/workspace/wt/napl-c2video/out` is the work root: `runs/<scenario>/`, `sim/<scenario>/`, `build/`, `web/`)

```sh
# 0. the lockstep runs (CPU) -> O/runs/<scenario>: lite_hud.jsonl (App.snapshot + Ph / wheels / gate / hopper every 50 ms), lite_meta.json (incl. state hash, gate transitions), replay.naplog, native.json
for s in s1_balloon_static s2_balloon_drift s5_mix s6_person_walk s7_fault_estop; do
  tools/remote.sh run ".venv/bin/python tools/record_cuas2.py run $s $O/runs/$s"; done        # exit 1 = the scenario's own checks failed

# 1. the build / wiring / program op log, generated from the assembly (CPU; the first design build routes 153 wires: many minutes), then the viewer capture (GPU, under xvfb)
tools/remote.sh bg oplog ".venv/bin/python tools/record_cuas2.py oplog $O/oplog.json"
tools/remote.sh bg --gpu build ".venv/bin/python tools/record_turret.py build $O/oplog.json $O/build --open assemblies/cuas2.toml --timeout 3000"

# 2. the operator page (CPU, headless Chromium)
tools/remote.sh run "../webvenv/bin/python tools/record_cuas2.py web $O/runs/s6_person_walk $O/web --fps 10"

# 3. the sim frames (GPU lane, one viewer per window; windows from the run's own events: dead time fast, real time around balls and the person, 1/4 slow motion around a hit)
for s in s1_balloon_static s2_balloon_drift s5_mix s6_person_walk s7_fault_estop; do
  tools/remote.sh run --gpu ".venv/bin/python tools/record_cuas2.py verify $O/runs/$s && .venv/bin/python tools/record_cuas2.py sim $O/runs/$s $O/sim/$s"; done   # verify = replay ends on the run's hash

# 4. encode (GPU: GStreamer VA, AV1 webm; LGPL path, no libx264)
tools/remote.sh run --gpu "../webvenv/bin/python tools/record_cuas2.py encode $O $O/cuas2_e2e.webm --speedup 1.5"

# looking at frames (the store is raw .rgz): tools/remote.sh run --nolock "../webvenv/bin/python tools/record_cuas2.py png $O/sim/s6_person_walk $O/png 10 300 500" ; tools/remote.sh pull out/png
# the auto-plan of a run: tools/remote.sh run --nolock ".venv/bin/python tools/record_cuas2.py plan $O/runs/s6_person_walk"
```

GPU rules (shared host): every viewer / encode job under `--gpu`, one at a time; check `cat /sys/class/drm/card1/device/mem_info_gtt_used` (limit 4 GB) and `df -h /workspace` after each segment.

## After a fix that changes behaviour

The frames are of the run, so a change of `python/napl/cuas2`, `firmware`, the scenes or the Referee changes the hashes: merge main, rerun step 0 (hashes are printed by `run` and again on the result card), then 3 (and 2 if the page matters).
The build / wiring segments only depend on the assembly (rerun 1 if it changed).
