# cuas2 RP2040 firmware

Firmware for the cuas2 turret (`docs/cuas2/spec.md`): the cuas-lite safety FSM plus a closed-loop dual-flywheel launcher (bidirectional DShot300), the
single-pocket gate and pusher feeder with the beam cross-checks, protocol 1.1 (STATUS 45 B, FLYWHEEL differential tail, one ball per FIRE) and the hardware shot
counter label. `firmware/cuas-lite-rp2040` is never edited: its `cobs.c crc16.c proto.c planner.c homing.c` and `tmc2209.c` are compiled by path.

| path                             | what                                                                                                                                                                     |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `include/fw_core.h`              | superset of lite's header: protocol 1.1, fault bits b6-b14, EVENT 10-16, results 12-15, cfg keys 23-41 (config image version 3, 176 B), the feeder/launcher unit's types |
| `include/pins.h`                 | GPIO map (spec 6.5)                                                                                                                                                      |
| `src/launcher.c`, `src/feeder.c` | the launcher PI loop (integer, 1 kHz) and the gate/pusher cycle; `fdr_step()` is mirrored statement for statement by `python/napl/cuas2/fsm.py`                          |
| `src/fsm.c`                      | lite's `fsm.c` with the weapon functions replaced; the shared safety functions stay byte-identical (`tools/cuas2_fsm_drift.sh`)                                          |
| `src/dshot.c`                    | DShot frame + eRPM reply codec (pure functions)                                                                                                                          |
| `src/pico/`                      | Pico only: `hal_cuas2.c` (steppers, servos, beams, IRQ muzzle capture), `dshot.pio` + `dshot_pio.c`, `main.c`. UNTESTED on hardware                                      |
| `host/`                          | x86 shared library for the python tests (`fwh_*` whole MCU, `fdrh_*` feeder alone) and `tests/selftest.c`                                                                |

## Build and test

```
tools/remote.sh run --slots 8 tools/cuas2_fw_host_test.sh     # host build, ctest, drift check, pytest -n 8 tests/cuas2/test_{protocol2,fsm,fw_vectors2,sim2,plant}.py
tools/remote.sh run --slots 1 tools/cuas2_fw_build.sh         # UF2 (Arm GNU 15.2.rel1 + pico-sdk from /workspace/wt/napl-cache; tools/fw_setup_toolchain.sh)
python tests/cuas2/gen_vectors.py {protocol|fsm} > tests/cuas2/{protocol_vectors2|fsm_vectors}.json   # regenerate after an intended behaviour change
```

## Hardware shot counter (spec 14 Q6, fault row 30)

The Pico has no spare GPIO (GP0-22 and GP26-28 are used, GP23/24/25/29 are wired inside the board), so the counter has no pin of its own.
**Wiring (MECH, `docs/cuas2/wiring.md` 3.4 / 3.2):** two 74HCT4017 count the muzzle-beam pulses (the GP16 node; N = 10 by default, the tens trip wire is jumperable to
20/30). The Nth pulse trips a 2N3904 clamp on the gate of the supervisor's weapon MOSFET (IRLB8721). The relay coil, the key and the E-stop stay **closed**, so
GP10 sees **nothing**: the counter is not upstream of any sensed line. Only a key power cycle resets it (POR from the key-fed 5 V logic).

**What the firmware does about it.** The trip is _not sensed_; it is inferred. A trip looks like weapon power lost while the key is on and the E-stop is closed:
both ESC telemetry streams drop within a few ms of each other and the pusher servo loses its supply. The launcher raises its ordinary `telemetry_lost` fault
(50 ms; the wheels' throttle goes to 0, KICK stops, the supervisor relay follows), and `feeder_stage()` in `src/fsm.c` additionally labels it EVENT 16
(`arg` = pulses counted) + fault bit b15 `hw_counter` when, at that moment, key on, E-stop closed, cfg 41 `hw_counter_n` > 0 (default 10, 0 = no counter fitted),
the two streams' ages differ by <= 5 ms (`FDR_HWC_SKEW_MS`) and the **per-key-session** count of raw muzzle-beam rising edges has reached cfg 41. The count is the
hardware's: it is reset only while the key is off, not by DISARM/ARM, and it counts every pulse, also unexpected ones. The label latches until `CLEAR_FAULT`.

Limits, stated: (1) it is a label from a telemetry-loss pattern and a pulse count, not a sensed trip: a real power fault (a blown fuse, a loose battery lead) at
exactly the N-th ball is labelled the same way, and a counter that trips while the telemetry was already lost is an ordinary `telemetry_lost`; (2) it depends on the
wiring above: the day the clamp moves (e.g. onto the relay chain) the pattern changes; (3) the count is only as good as the GP16 edge capture; (4) the trip itself
is the hardware's job and needs no firmware: with a dead MCU the clamp still cuts the weapon. A direct sense line would cost the buzzer pin (GP17).
Tests: `tests/cuas2/test_fsm.py::test_hw_counter_*` (labelled, one ball short, N = 0, E-stop / key at N, streams 30 ms apart vs 4 ms apart, per-key-session count).

## Timing (MECH geometry)

Pusher stroke 47.2 mm at 0.4 mm/ms (118 ms out, 118 ms back), limiter windows 254 ms power / 338 ms lockout, geometric cycle 466 ms
(`docs/cuas2/assembly.md` C-CYCLE). `FDR_*` in `fw_core.h` follow `docs/cuas2/spec.md` 5.4: commit = the earlier of 105 ms and the breech beam clearing (~71 ms; the
beam sits 9 mm behind the plug axis, so it clears after ~2.5 mm of ball travel and about 30 ms before the nip), `pusher_dwell_ms` 120 (range 50-130 so that dwell

- retract fit the 254 ms window), `min_shot_interval_ms` default 500 with a firmware clamp of 470.

## Python twin of the whole core

`python/napl/cuas2/sim_mcu.py` has `Sim2RP2040`: lite's `SimRP2040` safety FSM with the cuas2 feeder (`fsm.py`) wired in as `fsm.c` wires the C one, and `RealFsm`, the
real feeder FSM behind MODEL's `Plant2`. `tests/cuas2/test_sim2.py` drives the C core and the twin with the same host script and the same plant and compares every
ACK, EVENT and STATUS word tick by tick (21 scenarios, the hardware-counter branches, a fuzz sample). Not modelled: the receive-queue overflow, flash write failures,
the boot DEBUG note, LEDs and buzzer.

## Not done / needs the bench

- Every `src/pico/*` file is cross-compiled but untested on hardware. Bench list: DShot timing with a scope (TX 3.33 us/bit, reply 21 bits at 2.67 us), eRPM
  decode against a real ESC (Bluejay, user-flashed, spec 4.5), the muzzle IRQ pulse width at a real ball speed, servo end stops, KICK against the supervisor.
- The status LED (GP15, WS2812) has no PIO program: the pin is held low.
- Gains, feed-forward, window and the timing constants in `fw_core.h` (`FDR_*`) are [est] and need the bench (spec 5).
