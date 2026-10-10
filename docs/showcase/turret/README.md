# The counter-UAS turret, built from an empty design in the viewer, wired, programmed and flown

`turret_e2e2.webm` (1280x828, VP9, 4 min 51 s, 12.8 MB, encoded with `--speedup 1.7`; stays on the host, media is never committed). One video, five parts, all through the CAD-style Design UI (ribbon, browser tree, properties, status bar, view presets, measure, visibility):

1. **LIBRARY** - the agent searches the parts library (typed queries, a category and a numeric filter, thumbnails) and drags each part of the turret onto its port.
2. **BUILD** - gizmos with edge snapping and guide lines, align, colours, exact coordinates, an undo, the DRC window, a measure, hiding a part, and a camera orbit after each major step.
3. **WIRING** - two KiCad-like sheets with real pin-level connectors: 12 V supply, then the Raspberry Pi 5 with its full 40-pin header (panned and zoomed so the pin numbers and GPIO names are legible), UART TX to RX crossed, ground returns, the 15-conductor camera ribbon, one deliberate pin mistake flagged by the pin DRC (reason in a red caption) and then fixed. Back in 3D the routed tubes go around the parts, orbited.
4. **PROGRAM** - the PROGRAM page: the turret program (`examples/cuas/turret.py`), the swarm program (`swarm.py`), the scenario file.
5. **SIMULATE** - `napl program examples/cuas/cuas.scenario.toml` on **the assembly built in part 1-3**, replayed in the viewer with the HUD.

## What is new versus the first video

- The CAD-style UI is on screen throughout; the caption strip now sits **below** the picture (frame 1280x828) so the window's status bar stays visible.
- Pin-level wiring replaces the simplified port wiring: the Pi is wired at connector pins (`flatten.wire` with `from_pins` / `to_pins`, `included_in`), the pin DRC rules run (`PIN_*`), and the first UART attempt is wrong on purpose (not crossed): the DRC finding is captioned, `flatten.unwire` removes it, the crossed wires follow.
- New op `ui.wire_view {zoom?, fit?, center?: [x, y], seconds?}` (viewer `wire2d::Canvas::view`, eased glide): a recorded pan / zoom over the wiring sheet (moving cards makes the viewer auto-fit, so it could not be done by card moves).
- Camera moves in every segment: `camera.orbit` in BUILD and the 3D wire view; SIMULATE uses new HUD presets `orbit` and `swarmorbit` (`python/napl/cuas/hudview.py`) next to the close / chase cameras and slow-motion kills. Turret-camera picture-in-picture as before.
- The wire router needs 40-60 s per wire step; the encoder plays those spans fast and labels them "(wire router running)".
- The view cube cannot be clicked through an op; the view presets (front / top / iso) stand in for it. Honest limitation.

## Who decided what (read this before believing the video)

**Decided by the agent** (an LLM, Claude, as the stage "VIDEO" of a larger task; it wrote this README): every design request in
`oplog.json` - which part, to which port, the hinge limits, the wire parts and lengths, the bus IDs, the colours, the device bindings - and
the reason shown in each caption (`why`). It explored the sequence headlessly first (`tools/record_turret.py explore`, no window) to see
which requests the Design accepts and refuses, then wrote the log.

**The recorded take is a replay of that log** through the same `napl.design.viewer.Backend` the window talks to
(`tools/record_turret.py build`): the window opens on an empty floor, and each step is sent as a Backend request - `library.search` typed
into the search box, `library.drag` (the window flies the ghost from the part's card to the port and sends the drop itself), `gizmo.*`,
`set_color`, `flatten.*`, `ui.program` ... The screen is grabbed from Xvfb at 15 fps; captions are drawn onto the frames by the encoder from the
log (`title`, `why`) and from the Design's own replies (part count, mass, DRC).

**Pre-written, not decided inside the session** (it existed before the video task): the turret reference assembly `assemblies/turret.toml`, the
custom plate and yoke parts (`parts/cuas/`), the launcher and camera devices, `examples/cuas/` (scene, `turret.py`, `swarm.py`, the scenario),
the HUD and `tools/record_sim.py`. The agent's build is compared with the reference, not derived from it.

## The built design equals the reference

`record_turret.py build ... --expect assemblies/turret.toml` compares the design the window built (written by `Design.save`) with the reference:
same instances (id, part, group, parameter overrides), mates (including the joints), wires, device maps and devices after normalising list
order, wire / mate names and the default `rotation = 0`; every instance pose identical (max difference 0.0 m, 1e-6 tolerance); mass
2.2703549 kg both; DRC 0 errors, 0 warnings. What differs by design: the agent's **colours** (cosmetic, not in the reference). Nothing else
(printed by the tool; it exits non-zero on any other difference).

To make that possible the agent's session needed three small tool additions (they are in this branch): an `assembly` edit op (fixed base,
timestep, description: a new design is a free-floating vehicle, and the reference is bolted down), `flatten.wire` with an explicit `part`
(autowire refuses to bridge the supply cord / servo lead connector families, as it should; the reference wires are deliberate leads), and
the PROGRAM page's `ui.program` file view.

## The simulation

`CUAS_TURRET=<the saved built assembly> napl program examples/cuas/cuas.scenario.toml` (the scene takes the turret assembly from that env
variable; default `assemblies/turret.toml`). Four drones (3 x `x500v2`, 1 x `aos5`) fly `swarm.py` (seeded boids + random-walk wander, seed 7).
The pellet is the launcher device's point mass with air drag, gravity and the scene's wind (3, -2, 0 m/s plus Dryden turbulence); the
operator is scripted and every shot needs its authorization. `tools/record_sim.py frames` replays the run's `replay.naplog` in the viewer
(clean mode + HUD) and checks the replay hash against the run's `native.json`; slow-motion chunks are the same replay rendered at 4x the frame
density around the two kills (drone3 at 7.5 s, drone4 at 18 s); quiet stretches are shown at 2x and 4x.

## Result of the recorded run

| | |
|---|---|
| assembly | the one built in the window: `--expect assemblies/turret.toml` gave 17 parts, max pose diff 0.0, mass 2.2703549 kg, DRC 0 errors / 0 warnings, no differences |
| pins | the `--expect` comparison now includes the wire pin lists (`from_pins` / `to_pins`, `included_in`): `differences: []`, 39 of 39 oplog steps applied, 17 parts |
| `napl program` | PASS, 60 s of sim, about 127 s wall (shared box) |
| rounds / hits on a drone | 13 / 3 (hit rate 23 %; a drone and its pellet are both physical, so most pellets miss) |
| drones destroyed | 2 of 4 (drone3 at 8.0 s, drone4 at 20.7 s); fires without authorization: 0 |
| replay hash | `10ba0ae29d1f4ae0` at tick 60000: the viewer replay ends on exactly this hash (MATCH). The reference `assemblies/turret.toml` gives `59fe2bdd7b2009c8` (examples/cuas/README.md) with the same rounds / hits / kills: the built design has the same parts, poses and mass but lists its instances in build order, so the MJCF bodies (and the state vector the hash runs over) come in a different order. Pin-level wiring did not change the hash: `included_in` pins leave mass and body order unchanged, so it equals the first video's hash |

## How the video is made

```sh
# remote (hfbox): everything under tools/remote.sh, GPU jobs under the GPU lock
tools/remote.sh run --nolock '.venv/bin/python tools/record_turret.py explore docs/showcase/turret/oplog.json --expect assemblies/turret.toml --save out/turret_built.toml'   # headless check
tools/remote.sh run --gpu '.venv/bin/python tools/record_turret.py build docs/showcase/turret/oplog.json out/p4 --expect assemblies/turret.toml --save out/turret_built.toml'
tools/remote.sh run --gpu 'CUAS_TURRET=$PWD/out/turret_built.toml .venv/bin/napl program examples/cuas/cuas.scenario.toml --runs-dir out/runs'
tools/remote.sh run --gpu '.venv/bin/python tools/record_sim.py frames <run> out/sim --t0 0 --t1 15 --fps 20 --camera "0:overview,..."'   # chunks; one call per chunk (--start-index continues the numbering):
#   (t0 t1 fps camera) 0 6 20 "0:overview,3:chase" | 6 7 20 target | 7 8.4 80 close@drone3 | 8.4 16.4 10 swarm | 16.4 17.6 20 target
#                      | 17.6 18.6 80 close@drone4 | 18.6 24 10 swarm | 24 60 5 "0:swarm,40:overview"   (fps 80 = 4x slow motion; 10 / 5 = 2x / 4x fast)
<vidvenv>/bin/python tools/record_turret.py encode out/p4 out/sim2 out/turret_e2e2.webm --run <run>
```

The video venv (`av` + Pillow) is a throwaway outside the repo; VP9 (libvpx) is used, never libx264.
