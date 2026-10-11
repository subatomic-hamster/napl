# cuas-lite end-to-end video: how it is made

Media is never committed. Frames, run directories and the webm stay on the host (hfbox, `/workspace/wt/napl-video/out/lite/` for the worktree `napl-wt/video`; locally `./remote-out/` after `tools/remote.sh pull`, gitignored).
Everything is driven by `tools/record_cuas_lite.py`; it reuses `tools/record_turret.py` (xvfb viewer capture of the Design backend), `tools/record_sim.py` (replay of a run in the viewer with a HUD,
`--lite` selects the cuas-lite HUD) and `tools/videnc.py` (GPU encode; libx264 only with `--gpl`, nothing GPL is used by default). Pattern: the turret video (docs/showcase/turret/README.md).

## What is on screen (and what is not real)

| segment | what | source of truth |
|---|---|---|
| 1 Build | `cad/cuas-lite/params.py` / `parts.py` on the PROGRAM page, then the 74 on-board parts of `assemblies/cuas-lite.toml` revealed in eight build stages (frame, pan drive, lazy susan + rotor, yokes + bearings + tilt drive, hopper, launcher, camera, electronics), orbiting camera, mass + DRC | the finished design opened in the viewer (the BROWSER eye hides / shows parts); parts are **not** placed one by one through the library as in the turret video |
| 2 Wiring | four WIRING sheets with the real connector pins of the assembly (power, weapon-rail interlock, Pico to the TMC2209 + steppers, Pico sensing), zoomed on the busiest cards, then the 72 wires as tubes routed around the parts in 3D | `flatten.begin` on the assembly's own wires; the router (`wireroute.py`) keeps tubes out of solids |
| 3 Program | the PROGRAM page on `fsm.c`, `app.py`, `fire.py`, `ui.html`, a scenario file; the real operator web page (headless Chromium) with sector / ARM / HOLD TO FIRE; the MCU state machine as a diagram walked through the transitions of the S7 E-stop run | the files; `python/napl/lite/web.py` + `static/ui.html` served unchanged; transitions from the run's `lite_meta.json` |
| 4 Simulate | s2 drifting balloons, s3 hanging targets, s4 crossing quad, s5 mix, s6 person walks in (HUD: FIRING INHIBITED: PERSON IN VIEW), s7 E-stop mid-engagement (HUD: FIRING INHIBITED: E-STOP PRESSED, MCU SAFE) | lockstep runs of the real Jetson app + simulated RP2040 + napl kernel; replay of each `replay.naplog`, final state hash checked |

Not real, said on screen too: the detector is the stand-in (ground-truth projection with the spec 7.2 noise model), not YOLO; the sim camera has no pixels, so the HUD's DETECTOR VIEW and the web page's feed are drawn from the detector's boxes
(labelled); in the web capture the page shows the run's **recorded** `App.snapshot()` stream and the clicks (sector set, ARM, HOLD TO FIRE held, real dead-man posts reach the server) are timed to the run's own script, they do not steer the run.
Belts are closed GT2 loops without a model (assembly header), so they are described, not shown.

## One-time host setup (hfbox; throwaway, outside the repo)

```sh
# vidvenv: PyAV + Pillow (software fallback of the encoder); webvenv: the browser capture
python3 -m venv /workspace/wt/napl-video/webvenv && /workspace/wt/napl-video/webvenv/bin/pip install playwright pillow numpy
# the playwright CDN is slow from the host: take Chrome for Testing's headless shell directly (BSD-style Chromium licence; never shipped)
mkdir -p /workspace/wt/napl-video/chrome && cd /workspace/wt/napl-video/chrome && curl -sO https://storage.googleapis.com/chrome-for-testing-public/153.0.8010.12/linux64/chrome-headless-shell-linux64.zip && python3 -c "import zipfile;zipfile.ZipFile('chrome-headless-shell-linux64.zip').extractall('.')"
```
`NAPL_CHROME` overrides the browser path. The encode / png / web steps only need Pillow + numpy (+ playwright for `web`): run them with `webvenv/bin/python`; `run`, `plan`, `verify`, `sim`, `oplog` need the project venv (`.venv/bin/python`).

## Commands (from the worktree; `export NAPL_REMOTE_DIR=/workspace/wt/napl-video UV_CACHE_DIR=/workspace/wt/napl/uv-cache`)

`O=/workspace/wt/napl-video/out/lite` is the work root: `runs/<scenario>/`, `sim/<scenario>/`, `build/`, `web/`.

```sh
# 0. the lockstep runs (CPU, about 20-100 s each after the first composes the turret base) -> O/runs/<scenario>: lite_hud.jsonl (App.snapshot every 50 ms), lite_meta.json, replay.naplog, native.json
for s in s2_balloon_drift s3_hanging_targets s4_quad_3ms s5_mix s6_person_walk s7_fault_estop; do
  tools/remote.sh run ".venv/bin/python tools/record_cuas_lite.py run $s $O/runs/$s"; done        # exit 1 = the scenario's own checks failed
# the same scene through `napl program examples/cuas-lite/<scenario>.scenario.toml` ends on the same hash (checked for s6)

# 1. the sim frames (GPU lane, one viewer per window, 20 fps video frames; windows come from the run's own events: dead time fast, real time around shots and the person, 1/4 slow motion around a hit/cut)
tools/remote.sh run --gpu ".venv/bin/python tools/record_cuas_lite.py verify $O/runs/s6_person_walk && .venv/bin/python tools/record_cuas_lite.py sim $O/runs/s6_person_walk $O/sim/s6_person_walk"   # per scenario; verify = replay ends on the run's hash

# 2. the build / wiring / program op log, generated from the assembly (CPU; the first design build routes 72 wires: about 6 min), then the viewer capture (GPU, under xvfb)
tools/remote.sh bg oplog ".venv/bin/python tools/record_cuas_lite.py oplog $O/oplog.json"
tools/remote.sh bg --gpu build ".venv/bin/python tools/record_turret.py build $O/oplog.json $O/build --open assemblies/cuas-lite.toml --timeout 3000"

# 3. the operator page (CPU, headless Chromium; about 0.4 s per frame)
tools/remote.sh run "../webvenv/bin/python tools/record_cuas_lite.py web $O/runs/s6_person_walk $O/web --fps 10"

# 4. encode (GPU: GStreamer VA, AV1 webm; LGPL path, no libx264)
tools/remote.sh run --gpu "../webvenv/bin/python tools/record_cuas_lite.py encode $O $O/cuas_lite_e2e.webm --speedup 1.5"

# looking at frames (the store is raw .rgz): tools/remote.sh run --nolock "../webvenv/bin/python tools/record_cuas_lite.py png $O/sim/s6_person_walk $O/png 10 300 500" ; tools/remote.sh pull out/lite/png
```

GPU rules (shared host): every viewer / encode job under `--gpu`, one at a time; `cat /sys/class/drm/card1/device/mem_info_gtt_used` stayed at 0.34 GB before and after the sim and encode jobs (limit 4 GB).

## After a fix that changes behaviour

The frames are of the run, so a change of `fire.py` / `aim.py` / `app.py` / the scenes changes the hashes: `git merge napl-v1`, rerun step 0 (hashes are printed by `run` and again on the result card), then 1, 3, 4; the build / wiring segments only depend on the assembly (rerun 2 if it changed).
The auto-plan (`record_cuas_lite.py plan RUN`) re-derives the windows from the new events.
