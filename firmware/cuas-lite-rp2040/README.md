# cuas-lite RP2040 firmware

C firmware for the Raspberry Pi Pico (RP2040) in the cuas-lite turret: USB CDC serial protocol v1.0 (`docs/cuas-lite/spec.md` section 2), the six-state
safety machine with the 300 ms heartbeat watchdog (section 3), a 1 kHz trapezoid motion planner and step generator for two TMC2209 drivers, hall
homing, shot accounting, flywheel/servo PWM, flash config. pico-sdk (BSD-3-Clause) + TinyUSB (MIT) for the Pico build; the core compiles unchanged
on x86 for the host tests. Licence of this directory: BSD-3-Clause (SPDX headers in each file).

**Hardware status: nothing here has run on a real Pico.** The core logic is tested on the host (below); every hardware path is listed under "UNTESTED"
with the bench test that proves it.

## Layout

```
include/fw_core.h    core API: fw_t, fw_in_t (sampled inputs), fw_out_t (outputs), fw_hal_t (tx, flash write, trace), protocol constants
include/pins.h       the GPIO map (single source)
src/                 hardware-independent core: crc16.c cobs.c proto.c (framing) config.c fsm.c (commands, state machine, gates, watchdog, shots,
                     flywheel, pusher, STATUS) planner.c homing.c
src/pico/            HAL + main: main.c (USB, 1 kHz tick, 8 s hardware watchdog) hal_pico.c (step ISR, PWM, GPIO, flash) tmc2209.c (UART config)
host/                CMake for the x86 build: libcuas_lite_fwcore.so (ctypes, for the Python tests) and the C selftest
tests/selftest.c     stand-alone C check (CRC, COBS, frame, watchdog, ARM refusal)
CMakeLists.txt       the Pico/UF2 build
```
The core has no globals and no malloc: the platform calls `fw_rx()` with received bytes and `fw_tick(now_ms, inputs)` once per millisecond; the core
answers through `fw_hal_t.tx` and the `fw_out_t` outputs (steppers enable, microsteps to emit this millisecond, flywheel permille, servo pulse, LEDs).

## Toolchain and versions (hfbox, no apt)

| item | version | where | licence |
|---|---|---|---|
| Arm GNU Toolchain arm-none-eabi | 15.2.Rel1 (Build arm-15.86), GCC 15.2.1 20251203 | `/workspace/wt/napl-cache/arm-gnu/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi` | bundle `license.txt`: GPL-3.0 for binutils/gdb, GCC under GPL-3.0 with the GCC Runtime Library Exception, newlib/libgcc runtime under permissive terms. The toolchain is a build tool, not part of napl and not redistributed; what is linked into the firmware (libgcc, newlib-nano) carries the runtime exception / BSD-style terms |
| tarball SHA-256 | `597893282ac8c6ab1a4073977f2362990184599643b4c5ee34870a8215783a16` | matches the `.sha256asc` published beside the tarball | |
| pico-sdk | 2.3.1 (tag, commit `079c6f39023649b154152db30f1d781e884879bc`) | `/workspace/wt/napl-cache/pico-sdk` | BSD-3-Clause |
| TinyUSB (submodule of pico-sdk 2.3.1) | commit `86ad6e56c1700e85f1c5678607a762cfe3aa2f47` (no tag at that commit) | `pico-sdk/lib/tinyusb` | MIT |
| CMake / Ninja (hfbox env) | 4.4.4 / 1.13.2 | `/workspace/wt/napl-skydio/env/bin` | |
| host compiler for the core tests | gcc 13.3.0 (Ubuntu 24.04) | | |

The cache is built by `tools/fw_setup_toolchain.sh` (idempotent, verifies the SHA). It lives outside the repo and outside `build/_deps`, so
`tools/check_licenses.py` never sees it (run on hfbox: 74 license files, 0 problems).

## Build and test

```
tools/remote.sh run --slots 1 tools/fw_host_test.sh     # host core: C selftest + vectors + gate matrix + twin cross-check (no Arm toolchain)
tools/remote.sh run --slots 1 tools/fw_build.sh         # UF2 -> build/fw-rp2040/cuas_lite_rp2040.uf2 (needs the cache above)
tools/remote.sh pull build/fw-rp2040/cuas_lite_rp2040.uf2
```
Both are optional: napl's default build and `tools/check_licenses.py` do not touch them. `tests/lite/test_fw_vectors.py` and `test_fw_core.py` skip when
`build/fw-host/libcuas_lite_fwcore.so` is absent, so the default `pytest tests/lite` is unchanged. To run the twin rigs against the C core by hand:
`pytest tests/lite/test_twin.py tests/lite/test_gates.py --fw-core=build/fw-host/libcuas_lite_fwcore.so` (or `NAPL_FW_CORE=...`).

The SDK's own UF2 step needs picotool, which needs libusb headers (not on hfbox); `tools/bin2uf2.py` converts the SDK's `.bin` instead (family 0xE48BFF56).

Size of the release build (arm-none-eabi-size -A): flash 50,540 bytes (`.bin`, includes boot2 256 B; 101,376 bytes as UF2, 198 blocks); RAM static
about 9.8 kB of 264 kB (`.data` 4,976 + `.bss` 4,632 + vector table 192; heap 2 kB and stack 2 kB reserved by the linker script).

## Flashing

- **Drag and drop:** unplug, hold BOOTSEL, plug USB, release: a drive `RPI-RP2` appears; copy `cuas_lite_rp2040.uf2` onto it; the Pico reboots into the firmware.
- **From the app (no button):** with the turret powered and the app connected, the state must be SAFE (watchdog tripped, disarmed): send `BOOTSEL` with
  `magic = 0xB007B007` (`gim._send(protocol.Bootsel())`). The MCU drives all outputs safe and resets into the ROM bootloader. In any other state it answers
  ACK `BAD_STATE`; a wrong magic gets `BAD_ARG`. There is no vendor reset interface and no 1200-baud touch (disabled in `CMakeLists.txt`), so a stray host
  cannot reset an armed turret.

## Pin map (`include/pins.h`, checked against `parts/electronics/raspberry-pi-pico-h.toml`)

| GPIO | function | direction | notes |
|---|---|---|---|
| GP0 / GP1 | tilt TMC2209 UART TX / RX (UART0) | out / in | TX through 1 kohm to the driver's PDN_UART node, RX on the node |
| GP2 / GP3 / GP4 | pan STEP / DIR / EN | out | EN active low |
| GP5 / GP6 / GP7 | tilt STEP / DIR / EN | out | |
| GP8 / GP9 | pan / tilt hall (A3144, open collector) | in, pull-up | active low |
| GP10 | E-stop sense | in, pull-down | 1 = circuit closed (released) |
| GP11 | arm key sense | in, pull-down | 1 = key on |
| GP12 | flywheel PWM, 20 kHz, both gate drivers | out | PWM6 A |
| GP13 | flywheel tach (unused in v1) | in | plain GPIO |
| GP14 | pusher servo, 50 Hz | out | PWM7 A |
| GP15 / GP16 / GP17 | red LED / green LED / buzzer | out | |
| GP20 / GP21 | pan TMC2209 UART TX / RX (UART1) | out / in | as the tilt wiring |
| GP22 | TMC2209 DIAG, both drivers wired-OR | in, pull-down | active high |

**Differences from the spec 6.6 proposal, and why (flagged for the spec owner):**

1. **Servo GP13 -> GP14, tach GP14 -> GP13.** Confirmed in the Pico part's alternate-function table: GP12 is PWM6 A and GP13 is PWM6 B, the same PWM slice
   (one frequency per slice). A 50 Hz servo on GP13 would force the 20 kHz flywheel gates to 50 Hz. GP14 is PWM7 A, its own slice.
2. **TMC2209 UART added** (the spec pin map has none, though the task asks for UART configuration). Two separate single-wire links: the spec's
   16-microstep strap (MS1 = MS2 = VIO) gives *both* drivers UART address 3, so they cannot share one wire. Each driver keeps the 16-microstep strap, so a
   dead UART link still runs the axis at the right resolution with the VREF-pot current; the firmware reports `tmc pan=<r> tilt=<r>` in a DEBUG frame after HELLO
   (0 = configured and read back, negative = no reply). The spec's "debug UART = GP0/1" is dropped (DEBUG frames replace it).
3. **DIAG on GP22** added for `fault_bits` b1 (`driver_diag`), which the spec defines but gives no pin for.
4. No GPIO used here conflicts with USB, SWD or the on-board LED (GP25), and all are exposed on the Pico H header (GP0..GP22).

## Wiring (Pico GPIO to the rest)

| from | to | notes |
|---|---|---|
| GP2 / GP3 / GP4 | pan TMC2209 STEP / DIR / EN (control row pins 7, 8, 1 of the BTT module) | 10 kohm pull-up from EN to VIO so the driver is off while the Pico resets (assumption: not verified that the module has one) |
| GP5 / GP6 / GP7 | tilt TMC2209 STEP / DIR / EN | same |
| GP20 (TX) -- 1 kohm -- node -- GP21 (RX); node to pan driver PDN_UART (control row pin 4) | pan driver | MS1 = MS2 = VIO (16 microsteps, UART address 3) |
| GP0 (TX) -- 1 kohm -- node -- GP1 (RX); node to tilt driver PDN_UART | tilt driver | same strap |
| GP22 | DIAG of both drivers (wired-OR) | the BTT module's DIAG pin position is board-revision dependent (part file note): check yours; unused if left open |
| GP12 | both IRLB8721 gates through ~100 ohm, 10 kohm from each gate to GND | logic-level FET (Vgs(th) 1.35-2.35 V), 3.3 V drive is enough; the pull-downs keep the flywheels off while the Pico resets |
| GP14 | servo signal (pusher) | servo power from the weapon-rail 5 V buck, common ground |
| GP8 / GP9 | A3144 OUT (pan / tilt) | A3144 powered at 5 V (4.5-24 V supply); OUT is open collector pulled up by the Pico's internal pull-up to 3.3 V, never above 3.3 V |
| GP10 | E-stop sense: a second NC contact, or a divider across the weapon rail ending at <= 3.3 V | 1 when the circuit is closed; a broken wire reads "pressed" |
| GP11 | key sense, same | 1 when the key is on |
| GP15 / GP16 / GP17 | red LED / green LED (with series resistors), buzzer (transistor) | |
| USB | Jetson USB | CDC-ACM; the Pico is bus-powered or fed by the logic 5 V buck |

Hardware cut (not firmware): key switch and NC E-stop in series feed the weapon rail (flywheel drains and the servo 5 V buck). The firmware also reads both and
goes SAFE; it never gates the cut.

## TMC2209 current for the 17HS4401S

Motor: generic 17HS4401S, rated 1.7 A per phase, 0.43 N*m holding (`parts/actuators/generic-17hs4401s-nema17-42x40.toml`). Driver: BTT TMC2209 V1.3, R_SENSE 0.11 ohm.
Formula (TMC2209 datasheet; the same one the part file gives for VREF): `I_RMS = (CS + 1) / 32 x 0.325 V / (R_SENSE + 0.02 ohm) / sqrt(2)`, vsense = 0, so full scale
(CS = 31) is 1.768 A RMS. Not re-fetched from the datasheet in this session; confirm with a meter on the bench.

| setting | CS | I_RMS |
|---|---|---|
| **IRUN (default, `TMC_IRUN_CS`)** | 21 | 1.215 A (spec: 1.2 A) |
| **IHOLD (`TMC_IHOLD_CS`)** | 10 | 0.61 A |
| for reference: 1.0 A / 1.4 A / 1.7 A | 17 / 24 / 30 | 0.99 / 1.38 / 1.71 A |

Register writes at boot (`src/pico/tmc2209.c`, each driver on its own UART at 115200): `GCONF = 0x1C0` (pdn_disable, mstep_reg_select, multistep_filt; I_scale_analog = 0 so
the current comes from IRUN, not the pot), `CHOPCONF = 0x14000053` (reset value with MRES = 4: 16 microsteps, interpolated to 256), `IHOLD_IRUN = IRUN 21 | IHOLD 10 |
IHOLDDELAY 6`, `TPOWERDOWN = 20`, `TPWMTHRS = 234` (stealthChop below about 60 rpm of motor = 72 deg/s pan, 90 deg/s tilt; spreadCycle above, where the
torque is needed). The spec names no chopper mode ("stealthChop/spreadCycle as the spec says" has nothing to follow), so this split is the firmware's choice; both
CHOPCONF and IHOLD_IRUN are read back and checked. Keep VM at 12 V or more (spec 6.3) and cool the drivers.

## Behaviour notes: where the C core differs from `sim_mcu.py`

The C core is what runs on the turret, so the twin (`python/napl/lite/sim_mcu.py`) was brought into line with it (v1.1): rows 1-7 and 12 below were differences in v1.0 and are now
**aligned**; `tests/lite` (175 tests) pass identically against the twin and against the C core (`pytest --fw-core`). What still differs:

| # | twin | firmware | resolution |
|---|---|---|---|
| 1-7 | aligned in v1.1: spun = applied duty >= requested (4 permille/ms ramp up, immediate down); a burst aborted by a lost refresh goes to ARMED; planner P term braked to `sqrt(2*amax*err)`; lost steps raise a DIAG pin, `driver_diag` after 50 ms, no resync; STATUS every 10 ms with no catch-up; homing CLEAR judged on the hall pin (ends 2 deg after release); `cfg_crc` = CRC-32 of the 100-byte flash image | the same | the twin follows the firmware where the firmware follows the spec |
| 14 | (v1.2) fire sector: cfg 17, 20, 21, 22 (mdeg); `FIRE` refused (RANGE) outside; the position is checked every planner tick from the FIRE to the feed point, leaving the sector aborts the stroke (pusher retracts), EVENT 9 `SECTOR_ABORT`, state ARMED, flywheel request dropped | the same in the twin (`_in_sector`, `_pusher`) | spec 3.2 rule 4; the sector is UNSET at every boot (`sector_set` mask, cleared at power-up only): ARM and FIRE answer NO_SECTOR (11) until keys 17, 20, 21, 22 were all written since boot, whatever flash holds; the position is the planner position (no encoder), so a stalled or skipped motor is not seen; the flash image is version 2 (22 keys, 100 bytes) |
| 7b | `home_off_deg` constructor default 0 (as cfg 7/8 default 0); the test rig and `Lockstep` pass -1 deg (a calibrated unit) | cfg 7/8 default 0; `FwMcu` passes -1 deg in a flash image | same calibration in both |
| 8 | `CFG_SET` unchecked beyond wd/spr/hard cap | also: pusher pulse 500-2500 us, dir mask 0-3, homing speed 1-3000, times 0-60000 ms, hard timeout 0-65535 s (`BAD_ARG`); changing a steps/rev clears that axis' homed flag | safety additions, spec silent |
| 9 | E-stop released / key on take effect at once | must be stable 10 ms; E-stop open / key off act at once | contact bounce on the inputs |
| 10 | no DEBUG frames | one DEBUG frame (`tmc pan=.. tilt=..`) after the first HELLO | bench aid; `gimbal.py` ignores id 0x86 |
| 11 | no rx overflow | `fault_bits` b5 `usb_overflow` when the 512-byte rx queue overflows (a flood without delimiters) | spec fault list |
| 12 | aligned: one `EVENT FAULT` per newly set fault bit (cfg_invalid at boot is quiet) | the same | spec: "4 FAULT (bit index)" |
| 13 | no `shot_counter_inconsistent` check | `shots_arm > shot_cap` while armed sets b3 | spec fault list |

Unchanged semantics worth knowing (identical in both, spec-conformant): a `fire_id` stays remembered even when refused, so a retry needs a new id; every command
but `HELLO` before the first `HELLO` gets ACK `BAD_STATE` (heartbeats too); the first `0x00` on the wire is a resync byte (a cold decoder drops the partial frame
before it, so the first real frame needs a leading `0x00`, which `SerialGimbal` sends).

**Homing (v1.1): from any start position.** SEEK_POS at 100 deg/s for the negative half-range + 10 deg (pan 180 deg); no sensor: SEEK_NEG for the full range + 20 deg (pan
360 deg); any hit goes through CLEAR (out the window's negative edge) to the slow APPROACH at `homing_speed`, so the final approach is always from the negative side. Worst
case, pan from +170 deg: about 1.8 + 3.5 + 0.7 = 6 s of the 15 s timeout (so the search speed 100 deg/s stays, the timeout is unchanged). Tested from positive, negative and
in-window starts (`test_twin.py::test_homing_from_any_start`, both implementations) and with no sensor (`test_homing_without_a_sensor_times_out_in_the_fault`). Caveat:
SEEK_POS can drive up to 180 deg (pan) positive from an unknown start, past the +170 deg soft limit; the cable loop and stops must allow it, or park the turret on the
negative side before power-up. (b) CFG 7/8 default 0 ("calibrate") in both. (c) The spec's `driver_diag` has no clearing rule beyond `CLEAR_FAULT`; a DIAG pin that stays high re-raises it.

## Tests (host)

- `tools/fw_host_test.sh` -> ctest `fw_selftest` (C) passes; `tests/lite/test_fw_vectors.py` + `test_fw_core.py` (55 tests, the C core through ctypes): every frame in
  `protocol_vectors.json` encodes and decodes byte-identically (25 frames, 11 COBS vectors, bad-frame and stream vectors), a 1,000-case differential fuzz against
  `napl.lite.protocol`, the gate matrix (13 violated preconditions, each must get no ball and the expected ACK code), the watchdog (still up 290 ms after the last heartbeat;
  trips at 300-301 ms; no re-arm), homing from the window and from 40 deg before it, homing timeout, planner vmax/amax/step-rate measured on the emitted microstep
  stream (not on the core's own bookkeeping), soft limits, config store, corrupt/blank flash, BOOTSEL, hall_stuck, rx overflow.
- `--fw-core`: `test_twin.py` + `test_gates.py` against the C core: 63 passed; the whole `tests/lite` (175 tests) passes against the twin and against the C core, same set. A mutation check (inhibit gate removed, tilt-minimum removed, watchdog +100 ms in a scratch copy)
  fails 3 of the gate/watchdog tests, so the tests can fail.

## UNTESTED (hardware only) and how to test on the bench

Make the bench safe first: weapon rail unpowered (key off or fuse out), flywheel motors and servo unplugged, steppers unloaded.

| path | what could be wrong | bench test |
|---|---|---|
| USB enumeration, `stdio_usb` binary traffic, no DTR dependence, `stdio_put_string` raw bytes | CRLF or timing | plug in; `ls /dev/serial/by-id/` shows `usb-Raspberry_Pi_Pico_*`; run `python -m napl.lite.app --gimbal <path>` or a 10-line HELLO script: HELLO_ACK with the right uid, STATUS at 100 Hz (count over 10 s), 0 bad frames in `EVENT 8` |
| 1 kHz tick on the real clock | main-loop overrun | toggle a spare GPIO each tick and scope it: 1.000 ms period, jitter < 100 us under USB load |
| step generator: 50 kHz alarm ISR, Bresenham distribution, DIR setup, 25 ksteps/s | missed or doubled steps, pulse width | scope STEP at AIM speeds (pulse 20 us, spacing even); command +90 deg then -90 deg and check the mark returns (no drift over 100 round trips); at vmax check no stall |
| EN polarity, EN while resetting | motor energised at boot | scope EN while pressing RUN; motors must be free until the firmware enables at IDLE |
| TMC2209 UART config, echo handling, CRC, read-back | wrong wiring/address/echo | read the DEBUG frame after HELLO: `tmc pan=0 tilt=0`; measure motor phase current (RMS meter or scope with a probe) at IRUN: 1.2 A; hold 0.6 A; confirm 16 microsteps (full revolution = 16,000 pan steps, 200 x 16 x 5) |
| stealthChop / spreadCycle switch (TPWMTHRS 234) | noise or lost torque at the transition | sweep speed 20-240 deg/s; listen and check no stall |
| DIAG fault | pin position, polarity | stall a motor by hand (low speed): `fault_bits` b1 within ~50 ms and state FAULT |
| hall homing, A3144 polarity, magnet position, 15 s timeout | direction, edge, home offset | `HOME` each axis from the negative side, from the positive side and with the magnet under the sensor (all three must end at the same zero within 0.1 deg); STATUS flags b2/b3 set; calibrate cfg 7/8 (`pan_home_off`, `tilt_home_off`) so home is the window centre; `CFG_SAVE` |
| E-stop and key sense | polarity, noise, pull-downs | with the weapon rail live: E-stop pressed -> SAFE within 5 ms (EVENT 3 and `estop_pressed` in STATUS), homed flags cleared; key off -> SAFE (reason 4); unplug the sense wire: reads pressed |
| flywheel PWM, ramp, gate pull-downs | frequency, inrush, off at reset | scope GP12: 20 kHz, duty = FLYWHEEL permille; ramp 0 to 80 % in 200 ms; the gates read 0 V with the Pico in reset |
| servo pulse and timing | travel, dwell | measure 1100/1900 us (cfg 12/13); fire one ball with no balls loaded: extend 120 ms, retract 120 ms, `EVENT 2` after the retract |
| heartbeat watchdog on the real board | USB stall behaviour | arm with the flywheel on a scope; pull the USB cable: PWM to 0 within 300 ms; replug: SAFE, then IDLE after 1 s of heartbeats, never ARMED |
| hardware watchdog (8 s) | reset into SAFE | hang the main loop with a debug build (`while(1);`): reset after about 8 s; boot note shows `wd-reset` |
| config in flash (last sector) | erase while stepping, persistence | `CFG_SET` + `CFG_SAVE` in SAFE and IDLE, power-cycle, `CFG_GET` returns the value; corrupt the sector (picotool or a flash overwrite): boot is FAULT with `cfg_invalid`, `CLEAR_FAULT` -> SAFE with defaults |
| BOOTSEL message | reset path | in SAFE send `Bootsel()`: the `RPI-RP2` drive appears |
| LEDs, buzzer | pins, polarity | red solid in SAFE, green solid in IDLE, blinking green ARMED, both + beeps when engaging |

## Bench bring-up checklist

1. Weapon rail dead (key off, fuse removed), motors and flywheels disconnected, logic 5 V only. Build and flash the UF2; confirm red LED solid (SAFE).
2. Enumeration and protocol: by-id device present; HELLO/HELLO_ACK; STATUS 100 Hz; send a bad-CRC frame and see `EVENT 8` within a second.
3. Inputs: toggle E-stop and key (state and `flags1` b0/b1 follow); put a magnet at each A3144 (b4/b5 follow).
4. Steppers on the bench supply (12 V), motors free: confirm the DEBUG line shows both TMC links OK; measure the current; confirm EN behaviour at reset.
5. Homing and motion: `HOME`, calibrate the home offsets, `CFG_SAVE`; AIM sweeps at 20, 60, 120, 240 deg/s; check the soft limits and the mechanical stops are inside them.
6. Watchdog: stop heartbeats (unplug USB under motion): axes hold, SAFE within 300 ms, `wd_tripped` clears only after a second of heartbeats.
7. Weapon rail with the flywheel motors still unplugged and a dummy load on the gate drivers: PWM duty, ramp, E-stop/key hardware cut independent of the firmware (pull the RUN pin low: the cut must still work).
8. Pusher with no balls: timing, `EVENT 2`, cap and timeout (ARM with cap 2 / 5 s): ARM refused when not homed, disarm at cap and at timeout.
9. Only then flywheels and foam balls, behind a barrier, per `docs/cuas-lite/bench_validation.md` (the hardware worker's plan).
