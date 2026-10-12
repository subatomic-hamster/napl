# ASV end-to-end video: how it is made

Media is never committed. Runs, frames and the webm stay on the host (hfbox, `/workspace/wt/napl-asv/out/v/` for the worktree `napl-wt/asv`; locally `./remote-out/` after
`tools/remote.sh pull`, gitignored). Everything is driven by `tools/record_asv.py`; it reuses `tools/screenshots.py` (`Replay`: a run's `replay.naplog` applied to a fresh World, hash checked),
`tools/videnc.py` (raw frame store + GPU encode through GStreamer VA; nothing GPL: no libx264, no GPL ffmpeg) and `examples/asv/` (the scenarios, `encounters.py`, `perception.scan_points`).
Pattern: the turret / cuas-lite videos (`tools/record_turret.py`, `docs/cuas-lite/video.md`), but everything is **offscreen** (`Sim.render`, Vulkan) with a Pillow HUD composited on
the frame: no xvfb viewer, so there is no viewer code path to crash on the water (the water plane is the one named `water`, group 6, as `examples/asv/README.md` describes; unchanged).

## What is on screen (and what is not real)

| segment | what | source of truth |
|---|---|---|
| title | the claim, in words | text |
| build | the vessel of `assemblies/asv.toml` revealed part by part in six stages (hull, propulsion, power, avionics, sensors, wiring), orbit camera closing in, parts / mass / DRC counters | `napl.design.compute` (mass per instance from its report, summed as parts appear), `Design.check()` (DRC counts on screen are its answer), the compiled scene (`examples/asv/scene.py`, flat water). **Stand-in:** the finished design is revealed (parts of the compiled model made transparent until their turn); nothing is placed live through the Design API |
| 7 scenarios | head-on, crossing give way, crossing stand on, stand-on against a vessel that does NOT comply, overtaking, three vessels, patrol: chase / overhead / orbit / abeam cameras that blend into each other, HUD with the Rule, the ASV's role, the manoeuvre, the contact table (range, bearing, CPA, TCPA, role), a heading-up display of the lidar returns, the tracks and the chosen path, trails, fixed objects | the scenario's own `napl program` run (`telemetry.jsonl`, `replay.naplog`); the picture is the replay of that log in a fresh World (hash checked); lidar returns are the lidar device's own latest scan in the replayed state |
| cards | the 40-run Monte Carlo table of `examples/asv/README.md` with its stated weaknesses; the runs of this video (seed, Hs, closest approach, monitors, state hash); what is real and what is a stand-in | README (copied into `MC_ROWS`, not recomputed); `video_case.json` of each run |

Stand-ins, said on the last card too: the autonomy runs on the host computer standing in for the Pi 5 + Pixhawk (no PX4 firmware flies the boat); the lidar is idealised and sees another vessel only
through a cabin proxy box; AIS on a 1.2 m craft is an idealisation; the other vessels are scripted; the hull and skeg coefficients are estimates; collisions are geometric.
Camera time warp (x1 around the manoeuvre and the closest approach, up to x3.5 elsewhere; the patrol x3) is shown on every frame.

Seeds (all from the untouched-by-tuning 50000 block, calm sea; `examples/asv/README.md` "For the video"): 50025 for head-on, both crossings, the non-complying stand-on and patrol; 50015 overtaking; 50037 three vessels.
The non-complying stand-on (Rule 17(a)(ii)) uses seed 50025 with `traffic_complies = false`: it passed (no collision, closest approach above 5 m), so it is in the video; it was the first and only seed tried.

## One-time host setup (hfbox; throwaway, outside the repo)

Pillow is the only extra and is not a project dependency. Keep it away from the project venv's numpy: a directory holding only `PIL`, put on `PYTHONPATH`:

```sh
python3 -m venv /workspace/wt/napl-asv/vidvenv && /workspace/wt/napl-asv/vidvenv/bin/pip install pillow numpy
mkdir -p /workspace/wt/napl-asv/pil && ln -sfn ../vidvenv/lib/python3.12/site-packages/PIL /workspace/wt/napl-asv/pil/PIL && ln -sfn ../vidvenv/lib/python3.12/site-packages/pillow.libs /workspace/wt/napl-asv/pil/pillow.libs
```

## Commands (from the worktree; `export NAPL_REMOTE_DIR=/workspace/wt/napl-asv`)

`O=/workspace/wt/napl-asv/out/v`, `P="PYTHONPATH=/workspace/wt/napl-asv/pil .venv/bin/python tools/record_asv.py"`.

```sh
# 0. the runs (CPU, 4-6 min each with visuals; the first compiles and routes the wires once, about 5 min): $O/runs/<case>/asv-*/ with video_case.json (seed, Hs, monitors, state hash, replay match)
for c in head_on crossing_give_way crossing_stand_on stand_on_noncomplying overtaking multi_traffic patrol; do tools/remote.sh run "$P run $c $O"; done   # exit 1 = a monitor failed or the replay hash differs

# 1. the build segment (GPU, about 4 min) and every scenario's frames (GPU, 3-8 min each; the frames of a case are one chunk: replay from t = 0, so a case is one job)
tools/remote.sh run --gpu --nolock "$P build $O"
for c in head_on crossing_give_way crossing_stand_on stand_on_noncomplying overtaking multi_traffic patrol; do tools/remote.sh run --gpu --nolock "$P sim $c $O"; done   # prints the replay hash check

# 2. encode (GPU: GStreamer VA, AV1 webm; LGPL path)
tools/remote.sh run --gpu --nolock "$P encode $O $O/asv_e2e.webm"

# looking at frames (the store is raw .rgz):  $P png $O/sim/head_on $O/png 100 300 500 ;  tools/remote.sh pull out/v/png
# a quick look at one case without rendering all of it: $P sim CASE $O --step 4   (one frame every 4 sim seconds)
```

`--nolock` skips the CPU slots (a render job is one thread plus the GPU); the GPU lane (`--gpu`) is still taken, one job at a time box-wide. Do not run the `bg` form of a GPU job while both CPU slots are busy: it waits for a slot first.

## Measured (final render, hfbox)

`asv_e2e.webm`: AV1 (gst-va), 1280x720 @ 20 fps, 8758 frames = 437.9 s, 89.2 MB; encode 2 m 08 s. Scenario runs 235-372 s each (CPU, with visuals); each case's frame job about 10 min
(including about 1.5 min replay); build segment about 4 min. amdgpu GTT peak over all render and encode jobs (box-wide, 2 s samples): 0.37 GB (baseline 0.34 GB).

| case | seed | Hs m | closest approach m | state hash (replay matches) |
|---|---|---|---|---|
| head_on | 50025 | 0.312 | 26.5 | 23e5b6882c08eb12 |
| crossing_give_way | 50025 | 0.312 | 18.59 | 61aac4b8f0ec823a |
| crossing_stand_on | 50025 | 0.312 | 16.2 | cfb207da55b3ca53 |
| stand_on_noncomplying | 50025 | 0.312 | 6.61 | 90a7dd858c0e66a0 |
| overtaking | 50015 | 0.253 | 11.61 | 96e8d09b0482a4c3 |
| multi_traffic | 50037 | 0.373 | 15.16 | 0c46f1605384812d |
| patrol | 50025 | 0.312 | 2.72 (to a float) | 8bbc9b676b20b397 |

The build segment reports 17 parts, 16.16 kg, DRC 0 errors / 0 warnings (the README quotes 14.5 / 16.9 kg for earlier revisions).

## Notes from making it

- Making a part's geoms invisible by deleting them crashes the solver when a body is left with no geom (core dump, no message); the build segment sets their alpha to 0 instead (checked in a render).
- An exactly vertical camera makes the renderer ignore the camera's roll (the picture is north-up whatever `lookat` offset below ~1 m per 30 m of height says); the overhead camera is therefore a deliberate north-up map view tilted 6 degrees, and the HUD projection (`projector`) was checked against rendered pixels (a few pixels agreement).
- `result.json` of a `napl program` run is the runner's (monitors, replay hash); the scalar results of `run.py` (`cpa_m`, ...) are the monitors' columns (`min_cpa_5m.final` is the closest approach).
- `sp.course_offset_deg` of the telemetry is a counter-clockwise (math yaw) offset: negative = to starboard.
