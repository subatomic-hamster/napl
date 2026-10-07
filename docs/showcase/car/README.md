# A 4-wheel car, built from an empty design by an agent, then driven

`docs/showcase/car_from_scratch.mp4` (1280x720, H.264, 20 fps, 105 s, 4 MB). Part 1: the viewer's Design window while a design grows edit by
edit. Part 2: the finished car driving a 1 m square, replayed from its replay log. Stills: `docs/showcase/car/*.png`.

## Who decided what (read this before believing the video)

**Decided by the agent** (an LLM, Claude, working as the stage "VID1" of a larger task; it is the author of this README): every design request
in `oplog.json` - which part, to which port, which placement, which wire, which controller channel drives which wheel. It sent them one at a time
through `napl.design.viewer.Backend` (the object the viewer window itself talks to; the same arguments as the MCP `design_*` tools) in an
**exploratory session on hfbox with no window**, and read each reply (snap scores, alternatives, DRC delta) before choosing the next request.
`oplog.json` holds all 25 requests of that session in order, including the ones the Design refused and the one the agent rejected and undid
(request 15 -> 16), each with the reason the agent wrote when it sent it (`why`).

**The recorded take is a replay.** `tools/record_build.py build` starts the viewer on an empty floor, then applies the logged requests in order
through the same `Backend` from a thread (the window notices the new document revision within ~1 s and loads the rebuilt model, exactly as it does
for an MCP agent editing a shared design), and grabs the Xvfb screen at 6 fps. It checks that every request is answered with the same accepted /
refused result as in the exploration and that the final design is byte-identical to `assemblies/car4.toml` (it was: `final design == ...: True`).
Nothing in the video was decided live while recording. The captions are drawn onto the frames from the Design's own replies plus the agent's
recorded `why`; the "agent's reason" text is the agent's words from the exploration, not text generated afterwards.

**Pre-written (not decisions made inside the session):**
- the harness (`tools/record_build.py`) and the throwaway exploration server (a request loop around `Backend`; not committed);
- the custom plate part `parts/structure/car4-chassis-plate-custom.toml` (4 bracket seats, battery seat, 2 controller seats; geometry derived from the part
  dimensions in the file's header and labelled CUSTOM). The agent wrote it before starting the session, after checking that the library has no
  plate with four 37D bracket seats (the rover's has two). The session's request 1 adds it as the root;
- the drive program `examples/car4/square.py` and its scenarios (ported from `examples/rover/square.py`), and a small extension of `napl program`
  (`python/napl/program.py`: `[interfaces.roboclaw] controllers = [...]` = more RoboClaws, each on its own pty, env `NAPL_SERIAL_2`...);
- the wheel-drive device map (`wheel_drive`) is the rover's (`assemblies/rover.toml`), applied by the agent with `device_map` / `device` ops (request 23);
- the video's title / closing cards, the floor markings in Part 2 and the inset layout.

**What in the pictures is not the viewer window:** the top-left inset in Part 1 is an offscreen render (`Sim.render`) of the same built MJCF the
window just loaded (the window's own viewport is a small slot between its panels, half of it under the Design panel); the caption panel is drawn over
the window's Library tab, which stays empty because no request goes through the window's own buttons. The instance list, the Design header (mass,
undo depth) and the **DRC window** (bottom left) are the real window. Part 2 is offscreen renders of a replay of the run's `replay.naplog` (the same
state hash as the run: `a80a0c5dfe41386f`, checked by the harness); the floor grid and the blue 1 m square are visual-only boxes added to the replay's
model (no collision, no mass; the hash still matches); the HUD (sim time, speed, wheel currents, yaw) and the yellow trail are drawn by the encoder
from state queried at each frame (speed = ground-truth plate displacement over 0.2 s).

## What the agent built (`assemblies/car4.toml`, written by `Design.save`, not by hand)

20 instances, 1.5873 kg: the custom 3 mm aluminium plate (240 x 190 mm, 369 g), 4 x Pololu machined 37D bracket (1995), 4 x Pololu 37D 19:1
gearmotor with encoder (4751), 4 x Pololu 6 mm hub (1083), 4 x Pololu 90x10 mm wheel (1436), a Gens Ace 3S 2200 mAh pack, 2 x Basicmicro RoboClaw 2x7A
(3682), one per side (left: M1 front-left, M2 rear-left; right: M1 front-right, M2 rear-right = skid steer). Wheelbase 0.18 m, wheel-centre track 0.223 m
(the rover's brackets). Wheel joints are spin hinges, the right side `reverse = true` so positive `q` rolls forward.

DRC after each request (errors / warnings; the full list is in `oplog.json` replies when replayed):

| request | op | DRC errors |
|---|---|---|
| 1 | add root plate | 0 |
| 3-4 | 4 brackets snapped to the plate seats | 0 |
| 6 | 4 motors on the bracket faces | 4 (FLOATING_PORT: motor power pads not wired yet) |
| 9, 11 | 4 hubs (spin joints), 4 wheels | 4 |
| 14 | battery + 2 controllers | 6 (controller supplies not wired) |
| 15 | `autowire_all` | 4 (**rejected, see below**) |
| 16 | undo | 6 |
| 18 | explicit battery -> controller supply wires | 4 |
| 20 | explicit controller -> motor wires | **0** |
| 21 | encoder wires | 0 |
| 22-23 | `auto_devices` (battery), wheel_drive map + 4 servo devices | 0 errors, 0 warnings, 1 info (`SERVO_ID_UNSET`, a known false positive: the S5 servo-bus rule reads the 4 wheel `servo` devices as a shared serial bus; `assemblies/rover.toml` has the same) |

Decisions and reasons (from `oplog.json`, abridged; the whole text is there):
1. **Root**: the new `car4-chassis-plate-custom`; the rover plate has only two bracket seats.
2. **Brackets**: the 37D bracket is the only `bracket` whose `bolt-circle.31.6.m3` face matches the motor. The first snap ranked placements k=0 and k=1 equal
   (score 145.0, a bracket flipped 180 degrees; the record says `ambiguous: false`, see the findings), so the agent pinned k=0 on the other three
   (the seat `ref`s were written so that k=0 puts the motor inboard).
3. **Motors**: 4751 (the only 37D with the 64 CPR encoder; the RoboClaw velocity mode and the odometry need it; the sim's motor parameters are validated
   against it). The snap record: placement k=0, score 124.2 (clearance 3.2 mm), 5 alternatives (rotations about the motor axis).
4. **Hubs / wheels**: the only candidates for `shaft.6d` / `flat-mount` hub seat. Right side `reverse` as on the rover.
5. **Battery**: 3S (the 6S candidate exceeds the 12 V motors / controllers).
6. **Wiring**: `autowire_all` was tried first and **rejected** (request 15 -> 16): it wired `ctrl_l.m1_out` into `ctrl_r.power_in` and `ctrl_r.m1_out` into
   `ctrl_l.power_in` (both are `screw-terminal`, one is role source, one is role sink: a PWM motor output feeding the other controller's battery input).
   The agent undid it and wired explicitly: pack lead -> left controller (the pack's own 14 AWG lead), an estimated 0.10 m 14 AWG tail -> right controller,
   motor leads with the rover's 22 AWG recipe (`included_in` the motor), the 4 encoder pairs (`autowire` with explicit ends: the tool correctly called the
   4 x 4 channel assignment ambiguous). `autowire` also refused pack `wire-lead` -> `screw-terminal` and `screw-terminal` -> `solder-pad` (different
   connector families), hence explicit `wire` ops for those.
7. **Devices**: `auto_devices` made only the battery device; the 4 RoboClaw+gearmotor wheel drives (the sim's `servo` device in duty / velocity-PID mode)
   are not derived from topology, so they are added with `device_map` / `device` ops.

## Simulation numbers (hfbox; `tests/python/test_car4.py -s`, `napl program examples/car4/square.scenario.toml`)

- Straight, duty 0.3 on all four wheels: 0.735 m/s (equal to wheel rate x radius within 1 %), lateral drift < 0.01 mm/s, yaw 0.000 deg (a symmetric model),
  plate height 21.73 mm constant, wheel current 0.128 A each.
- **Skid steer scrubs.** Spinning in place (left -duty, right +duty): the true yaw rate is only **0.53** of what the geometric track (0.223 m) predicts.
  Effective track 0.418 m at duty 0.3 and 0.5 (0.435 m at 0.15; at duty 0.08 the wheels barely move, 2 deg/s). Commanded vs measured 90 degree turn
  by encoder odometry through the RoboClaws (`turn90*.scenario.toml`, true yaw from the plate quaternion): with the geometric track the car turned only
  **48 - 50 deg** (three runs: 49.6, 48.6, 48.0; odometry said 93); with the effective track 0.42 m (a constant measured once in the sim, in
  `examples/car4/square.py`) it turned 90.8 - 93.5 deg (three runs), i.e. within 3.5 degrees of the command, run to run.
- 1 m square (`examples/car4/square.scenario.toml`: left RoboClaw 0x80, right 0x81, the unmodified `basicmicro` 2.0.10, 38400 baud, real time):
  18.6 s of sim; all four corners visited; it ends **34 - 90 mm** from the start across 11 runs (the recorded run: 84.8 mm, true end yaw +5.5 deg while
  the odometry's own end estimate was (1.0, -57.2) mm and 1.75 deg: the heading loop only sees the encoders, so the scrub error is invisible to it; each
  corner overshoots by ~2 deg, which costs 3 - 5 cm on the next side). A heading-hold gain of 4 or 8 instead of 2 did not change that beyond the run-to-run spread
  (`CAR4_HOLD`). The monitors allow 12 cm (the rover's allow 5 cm and it ends 14 mm out). No motor alarm, pack never below 12.1 V, no tipping.
  Replay hash of the recorded run `a80a0c5dfe41386f` (tick 18644), matched by `napl-replay`. Runs are not bit-identical to each other (the program
  talks to the sim in wall-clock real time over a pty, so command times differ); each run's replay is exact.

## Findings in the existing tools (not fixed, except the one noted)

- `autowire_all` pairs a controller's motor output (`m*_out`, role source, `screw-terminal`) with the other controller's supply input (`power_in`, sink,
  `screw-terminal`). Port type + roles match, the electrical meaning does not. Not changed (the pairing rule is shared with the x500); the video shows it
  being rejected.
- A snap whose best placements differ only by the rotation k (bracket k=0 / k=1, 180 degrees apart, equal score) reports `ambiguous: false`; the flag
  only considers other port pairs. A pure-rotation tie is listed under `alternatives` but is not flagged.
- DRC does not check the motor-output nets (controller 7.5 A continuous / 15 A burst per channel vs the motor): the car's 4 motors stall at 5.5 A each, below the
  channel ratings, but the DRC would not have said. As documented for the rover (docs/research/rover.md).
- `napl program` could drive one RoboClaw only; extended (documented in `program.py`) with `controllers`. Existing scenarios are unchanged
  (`tests/python/test_rover.py`, `test_roboclaw.py`, `test_program.py` pass).

## Reproduce

```bash
# design (no kernel needed): DRC and the build
napl design check assemblies/car4.toml            # 0 errors, 0 warnings, 1 info
python tools/check_parts.py                       # 0 problems (the new plate part included)

# sim: drive + tests (hfbox: tools/remote.sh run '...')
napl program examples/car4/square.scenario.toml   # needs the `rover` extra (basicmicro); monitors + replay hash
pytest tests/python/test_car4.py -s

# the video: Part 1 (xvfb + GPU), the drive run, Part 2 (GPU), encode in a throwaway venv (`pip install av pillow`; neither is a project dependency)
python tools/record_build.py build docs/showcase/car/oplog.json OUT/p1 --expect assemblies/car4.toml
napl program examples/car4/square.scenario.toml --runs-dir OUT/runs
python tools/record_build.py drive OUT/runs/car4-square-<time> OUT/p2
<throwaway venv>/bin/python tools/record_build.py encode OUT/p1 OUT/p2 docs/showcase/car_from_scratch.mp4
python tools/record_build.py oplog SESSION.jsonl docs/showcase/car/oplog.json   # how oplog.json was produced from the exploration's request log
```

Encoding note: the PyAV 19 wheel's bundled FFmpeg offers `libx264` (a GPL encoder) next to `libvpx-vp9`; the MP4 was made with libx264 as a one-off tool, nothing of
it is linked into or shipped with napl. `encode --no-gpl` selects VP9/WebM instead.
