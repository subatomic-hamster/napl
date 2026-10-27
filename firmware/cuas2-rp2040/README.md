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
tools/remote.sh run --slots 8 tools/cuas2_fw_host_test.sh     # host build, ctest, drift check, pytest -n 8 tests/cuas2/test_{protocol2,fsm,fw_vectors2}.py
tools/remote.sh run --slots 1 tools/cuas2_fw_build.sh         # UF2 (Arm GNU 15.2.rel1 + pico-sdk from /workspace/wt/napl-cache; tools/fw_setup_toolchain.sh)
python tests/cuas2/gen_vectors.py {protocol|fsm} > tests/cuas2/{protocol_vectors2|fsm_vectors}.json   # regenerate after an intended behaviour change
```

## Hardware shot counter (spec 14 Q6, fault row 30)

The Pico has no spare GPIO (GP0-22 and GP26-28 are used, GP23/24/25/29 are wired inside the board), so the CD4017-class counter has no pin of its own.
**Wiring contract (MECH):** the counter's relay contact sits in the key -> E-stop chain _upstream_ of the GP10 tap, so a trip reads as "E-stop chain open, key on".
The firmware (`hw_counter_note()` in `src/fsm.c`) labels that edge with EVENT 16 (`arg` = balls counted) when the balls counted this arm (confirmed exits plus a
muzzle pulse in the same tick) have reached cfg 41 `hw_counter_n` (default 10, 0 = off); either way the rail is open and the state goes to SAFE (fault row 23).
Limit: a real E-stop press at exactly the N-th ball is labelled as a counter trip. If a direct line is wanted, give up the buzzer (GP17) for it.

## Not done / needs the bench

- Every `src/pico/*` file is cross-compiled but untested on hardware. Bench list: DShot timing with a scope (TX 3.33 us/bit, reply 21 bits at 2.67 us), eRPM
  decode against a real ESC (Bluejay, user-flashed, spec 4.5), the muzzle IRQ pulse width at a real ball speed, servo end stops, KICK against the supervisor.
- The status LED (GP15, WS2812) has no PIO program: the pin is held low.
- Gains, feed-forward, window and the timing constants in `fw_core.h` (`FDR_*`) are [est] and need the bench (spec 5).
