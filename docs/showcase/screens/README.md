# Screenshots of the simulation interface and the models in action

Real captures, nothing edited, all 1280x720 and under 400 KB. Produced on hfbox (AMD radv Vulkan, Xvfb) by `tools/screenshots.py`.
`manifest.json` holds the sim time, tick and state hash of every model shot. Renders are deterministic per GPU / driver only; a PX4
run is not bit-reproducible between runs (real PX4 in lockstep), so a re-run gives other hashes and slightly different frames.

Reproduce everything (about 8 minutes: five scenarios, PX4 ones ~1 min each, then renders):

```
tools/remote.sh run --gpu 'NAPL_PX4_INSTANCE=7 .venv/bin/python tools/screenshots.py all --runs /tmp/napl-shots-runs --out build/screens'
rsync -e 'ssh -o BatchMode=yes' -a hfbox:/workspace/wt/napl/src/build/screens/ docs/showcase/screens/
```

Single steps: `tools/screenshots.py scenarios|frames|quad|ui [NAME...]` (`list` prints the names).
The command column below is the `tools/screenshots.py` call (prefix `tools/remote.sh run --gpu '.venv/bin/python ...'`; `frames` needs the
matching `scenarios` run under `--runs`).

## Viewer UI (X500 V2 design, `Sim.show(design=Backend)`, scripted with the viewer's `--script` actions)

| Image | Shows | Command / viewer script |
|---|---|---|
| `ui-main-x500.png` | main window: scene tree (devices, bodies), inspector of `mot1` with units (`[m]`), run controls with the sim running (tick 3862), Design window, DRC window | `screenshots.py ui ui-main-x500`; script `2 inst mot1; 4 run 1.0` |
| `ui-design-library.png` | Design window, Library tab after removing the PDB and snapping one back: library parts ranked for `bottom.pdb`, and the 4 ranked placement alternatives of the snap (third one previewed), DRC finding with its fix | `ui ui-design-library`; script `2 dremove pdb; 4 dsuggest bottom.pdb pcb; 6 dsnap holybro-pdb-60a bottom.pdb pdb; 8 dcycle 2; 12 dfinding 4` |
| `ui-design-wiring.png` | Wiring tab after two ESCs were removed, snapped back and `autowire all` ran: the 2 ambiguous decisions (motor_out to mot1 / mot2, signal to 14 FC ports) listed with pickers, nothing guessed | `ui ui-design-wiring`; script `2 dremove esc1; 4 dremove esc2; 6 dsnap holybro-x500v2-esc-blheli-s-20a tube1.esc esc1; 8 dsnap ... tube2.esc esc2; 12 dautowire_all` |
| `ui-drc.png` | DRC window after removing the PDB: 4 new findings starred (`erro*`), one warning clicked with its tooltip / fix text | `ui ui-drc`; script `2 dremove pdb; 8 dfinding 4` |
| `ui-simulate-hover.png` | Simulate tab after the built-in hover run: mass 1.5466 kg, T/W 2.56, altitude error 0.099 m max, hover throttle 0.426, pass true | `ui ui-simulate-hover`; script `2 dsim hover` |

(The 3D view of the design sits behind the panels at this window size; these shots are about the panels.)

## Models in action (offscreen renderer, state of a real run, replayed from its replay log)

Each frame comes from replaying the run's log to the given sim time; the replay's final hash matched the run's in every case.

| Image | Shows | Sim time / tick | State hash at the frame | Run |
|---|---|---|---|---|
| `x500-hover-px4.png` | X500 V2 (parts-built) hovering at 5 m under real PX4 SITL | 23.104 s / 23104 | a1a6b2e058fc6ff8 | `napl program examples/x500/hover.design.scenario.toml` (`HOVER_HOLD_S=8`) |
| `x500-motor-out.png` | motor-out fault, 0.5 s after esc0 went open circuit: the quad tumbling (the scenario's fixture model, hence plain grey) | 29.040 s / 29040 | 729b6a9e55189fa3 | `napl program examples/faults/x500_motor_out.scenario.toml` |
| `so101-pick.png` | SO-101 mid-pick, cube lifted in the gripper (Feetech STS pty driven by `pick.py`) | 29.084 s / 29084 | 229a148d8e5f418e | `napl program examples/so101/pick.scenario.toml` |
| `koch-pick.png` | Koch arm mid-move with the cube in the gripper | 12.281 s / 12281 | e73f22b6b7069e70 | `napl program examples/koch/pick.scenario.toml` |
| `showcase-handoff.png` | arm-loads-drone: the SO-101 holds the payload cube over the drone's top plate (just before release) | 33.500 s / 33500 | ad957b133bd5379b | `napl program examples/showcase/arm_loads_drone.scenario.toml` |
| `showcase-flight.png` | arm-loads-drone: the loaded drone in flight at its 4.39 m peak, payload on the plate | 65.804 s / 65804 | 7f495212b11f9307 | same run as the hand-off |
| `so101-scratch-pick.png` | the SO-101 built from an empty design (`tests/python/test_design_scratch_arm.py` `build_arm`) mid-pick | 29.231 s / 29231 | de2a14232034d93e | `examples/so101/pick.scenario.toml` with `[design]` = the saved scratch design (`pick_on`) |
| `scratch-quad-hover.png` | the quad built from an empty design (`tests/python/test_design_scratch.py` `build_scratch`) 2 m up under the built-in hover controller (`Design.simulate("hover", altitude=2, climb_s=3, hold_s=4)`; pass, throttle 0.424) | 5.000 s / 2500 | f66566d8e7e65c0a | `screenshots.py quad` |
| `rover-square.png` | `assemblies/rover.toml` driving its 1 m square, mid-leg (it builds and its scenario passes) | 5.531 s / 5531 | f6be873f761e053e | `napl program examples/rover/square.scenario.toml` |

Camera of every model shot: orbit camera on a prim / the ground-truth position at that time (see `FRAMES` in `tools/screenshots.py`), 2x2 supersampled.
