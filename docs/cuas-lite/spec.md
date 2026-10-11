# cuas-lite engineering spec (v1, 2026-10-11)

Build target for the parallel workers: hardware/CAD, RP2040 firmware, simulated RP2040 + napl sim, Jetson app + web UI, detector (sibling repo
`../cuas-lite-detector`), videos. Everything here is a decision unless it sits under "Open questions" (section 9). Numbers marked
**[src]** were fetched and quoted in section 10; **[est]** are estimates; **[der]** are derived here (arithmetic shown or scripted).

Goal: a real, 3D-printable counter-drone turret for foam balls (balloons and hanging targets in the real world, drones in the sim), under
$150 excluding the Jetson Orin Nano and Logitech Brio, with ONE Python app that runs unchanged against real hardware and against napl.

## 0. Findings that change the brief (read first)

1. **Ball size and mass.** The brief says 0.7 in / 17.8 mm / ~0.95 g. Two independent pages for the official Rival round say 22.2 mm (7/8 in) and
   1.86 g; a third-party Rival-compatible round is 22.7 mm and 1.75 g (section 10). Spec baseline: **d = 22.2 mm, m = 1.86 g** as a
   parameter set (`BALL`), with the brief's 17.8 mm / 0.95 g kept as an alternative set; every consumer (CAD barrel, sim launcher part, ballistics)
   reads one `BALL` constants block so a swap is one edit. The barrel/feed CAD must be parametric in ball diameter.
2. **Range is limited by ranging, not by the muzzle.** A 22.2 mm / 1.86 g foam ball at 30 m/s needs 0.9 m of drop compensation at 10 m and 2.5 m at 15 m
   (table in 5.3). Elevation needed changes by ~0.9 deg per metre of range at 12 m. Range from apparent size (as `napl.cuas.tracking` does) is good to
   10-25 %, i.e. 1-3 deg of elevation error at 12 m, against a 0.8 deg allowance for a 30 cm balloon. Expect usable hit rates to ~8-10 m with size ranging;
   15 m needs a range measurement (open question Q2). The sim must model this ranging error, not hide it.
3. **Belt load on the NEMA 17 shaft.** The motor datasheet limits radial force to 28 N at 20 mm from the flange (**[src]**, parts file S2). A 16T GT2 pulley
   (pitch radius 5.1 mm) at 0.1 N*m puts 20 N of tangential force on the shaft before belt preload. The motor pulleys are therefore **20T** (6.4 mm),
   driven pulleys 100T (pan, 5:1) and 80T (tilt, 4:1).

## 1. System block diagram and process layout

```
 operator laptop/phone (browser, LAN)
        |  HTTP + SSE (stdlib server in the app), token in URL, POST /api/* for commands
 +------v-------------------------------------------------------------- Jetson Orin Nano -----------------+
 |  cuas-lite app (python -m napl.lite.app)                 detector service (../cuas-lite-detector, AGPL)  |
 |   capture thread  --RGB24-->  /dev/shm/cuas-lite-frames  <--reads--  TensorRT FP16 YOLO                  |
 |   control loop    --JSON line-->  $XDG_RUNTIME_DIR/cuas-lite-det.sock  --JSON line reply-->              |
 |   tracker, fire control, safety gate, web server                                                          |
 |   serial thread  <== USB CDC (COBS+CRC16, section 2) ==>  RP2040 (Pico)                                   |
 +-----------------------------------------------------------------------------------------------------------+
                                         |                         |
              step/dir x2 (TMC2209 x2)---+   PWM flywheel (2x MOSFET), PWM pusher servo, hall x2,
                                             E-stop sense, key sense  <-- hardware cut: key+E-stop in series on the "weapon rail"
```

**Real-hardware processes** (two systemd units, both user-level, started in order):

| unit | process | notes |
|---|---|---|
| `cuas-lite-detector.service` | `python -m cuas_lite_detector.serve` (sibling repo) | creates the Unix socket and opens the shm ring read-only; restarts on failure |
| `cuas-lite.service` | `python -m napl.lite.app --camera v4l2:/dev/video0 --gimbal /dev/serial/by-id/usb-Raspberry_Pi_Pico_* --web 0.0.0.0:8080` | After=detector; one process, 4 threads below |

App threads (single-writer rule: only the control loop touches fire-control state and sends AIM/FIRE/HEARTBEAT):
1. **capture**: V4L2 MJPG 1280x720 @ 60, decode, stamp `t_cap` (monotonic seconds, taken when the buffer is dequeued minus a fixed
   `cam_latency` = 0.030 s [est] for exposure+USB+decode), write the slot, publish the newest frame to the control loop.
2. **serial rx**: reads the CDC fd with `select`, COBS-decodes, pushes `STATUS`/`ACK`/`EVENT` to a queue; keeps a 1 s ring of STATUS
   (for positions at `t_cap`).
3. **web**: `ThreadingHTTPServer` (stdlib, no new dependency). `GET /` single page, `GET /video.mjpg` (multipart, JPEG q=70, 15 fps,
   boxes drawn client-side from `/events`), `GET /events` (SSE, 10 Hz state + tracks + shot count), `POST /api/{arm,disarm,authorize,
   sector,deadman,estop_ui,home}`. Every POST needs `?token=` (random at install, printed on the console). The UI "E-stop" button only disarms; it is
   not the safety E-stop.
4. **control loop** (main thread): `select` over {new frame, detector reply, serial status, 50 ms timer}. Per detector reply: tracker ->
   fire control -> `AIM` (+ `FLYWHEEL`/`FIRE` when due). Every 50 ms: `HEARTBEAT`. If this loop stalls > 300 ms the MCU trips on purpose: the
   heartbeat is sent from the loop that decides to fire, never from a helper thread.

Camera mode and the shm/socket protocol: sections 5.2 and "Detector IPC" below.

### Detector IPC (copied verbatim from the briefing; both sides implement exactly this)

- The app owns the camera. Per frame it writes RGB24 into a POSIX shm ring `/dev/shm/cuas-lite-frames` (header: magic "CLF1", slot count, width, height, stride; then slots) and sends one JSON line over the Unix socket `$XDG_RUNTIME_DIR/cuas-lite-det.sock` (fallback /tmp): `{"seq": n, "slot": k, "t_cap": <s>, "w": W, "h": H}`.
- The detector replies one JSON line per request: `{"seq": n, "t_infer_ms": x, "dets": [{"cls": "drone"|"balloon"|"person"|"target", "conf": c, "xyxy": [x0, y0, x1, y1]}]}` in pixel coordinates of the sent frame.
- napl ships a stand-in detector (Apache/proprietary, no Ultralytics) implementing the same IPC for sim and CI (e.g. ground-truth projection from the sim, or the existing classical detector).

**Additive clarifications** (not in the briefing; the detector worker must adopt them, flagged Q1): header = five little-endian u32
(`"CLF1"` as bytes, slots, width, height, stride in bytes) zero-padded to 64 bytes; slot k starts at `64 + k*stride*height`; 4 slots; RGB24, row
stride = 3*width. The app sends a request only when the previous reply has arrived or 100 ms have passed (latest frame wins; older
replies are dropped by `seq`); it will not overwrite a slot for 3 further frames. `t_cap` is on the app's monotonic clock. A missing reply for
250 ms makes the app treat the detector as stale (fail closed, section 3).

## 2. RP2040 USB-CDC serial protocol (v1.0)

**Transport.** USB CDC-ACM from the Pico (`pico-sdk` stdio_usb or TinyUSB CDC directly). Baud setting is ignored; the app opens at 115200 raw 8N1
(`termios` raw, like `python/napl/feetech.py`, no pyserial). The sim twin gives a pty with the same bytes.

**Framing: COBS + CRC-16, 0x00 delimiter.** Rationale: binary payloads with no escape rules to get wrong; a single lost/garbled byte costs
one frame and the next `0x00` resyncs (a line protocol has to escape `\n` in binary, and the Feetech-style `FF FF` header resyncs by scanning, which
fails on payload bytes equal to `FF`); CRC-16 catches what the USB layer cannot (a firmware or buffer bug); COBS overhead is 1 byte per 254.
Encoding of one frame on the wire: `COBS(body) 0x00`, with
`body = id:u8 | seq:u8 | payload:0..60 bytes | crc16:u16 (LE)`. Maximum body 64 bytes. Length of the payload is implied by the frame
length. `crc16` = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0) over `id|seq|payload`; check value
`crc16(b"123456789") == 0x29B1` (unit-test vector for both implementations). All multi-byte fields little-endian. `seq` is a
per-direction u8 counter incremented per frame sent; the receiver counts gaps (diagnostic only). A frame with bad CRC, bad COBS, or length < 3 is
dropped silently (counted in `bad_frames`, reported in `EVENT 8`). A receiver ignores bytes until the first `0x00`.

**Versioning.** `HELLO` carries `major.minor`. Major mismatch: `HELLO_ACK.result = 1`, and the MCU stays in SAFE rejecting everything but
`HELLO`. Minor bumps only append fields at the tail of a payload and add ids; receivers accept longer payloads (ignore the tail), reject
shorter ones with `BAD_ARG`. Unknown ids get `ACK result=UNKNOWN_CMD`. This spec is 1.0.

**Rates.**

| stream | rate | source of the number |
|---|---|---|
| `HEARTBEAT` host->MCU | 20 Hz (every 50 ms) | 6 beats fit in the 300 ms watchdog |
| `AIM` host->MCU | once per detector reply, <= 60 Hz; 20 Hz hold-position `AIM` when idle | camera 60 fps |
| `FLYWHEEL` | on change + refresh 10 Hz while non-zero | MCU zeroes it if not refreshed for 500 ms |
| `STATUS` MCU->host | 100 Hz (10 ms), unsolicited after the first valid `HELLO` | 35 bytes on the wire per STATUS = 3.5 kB/s |
| MCU planner / step generator | 1 kHz planner, step pulses from a PIO state machine or timer ISR up to 25 kHz | [der] 240 deg/s pan = 10.7 ksteps/s |

### 2.1 Host -> MCU

| id | name | payload (bytes) | reply |
|---|---|---|---|
| 0x01 | HELLO | `major:u8, minor:u8, session:u32` (random per app start) | `HELLO_ACK`; a new `session` disarms: state -> SAFE (`reason 11`), shot counters of the arm cleared |
| 0x02 | HEARTBEAT | `hb_seq:u16, flags:u8` : bit0 `DEADMAN_OK` (web dead-man fresh < 500 ms), bit1 `FIRE_INHIBIT` (app veto: person, stale detector, geofence, ...), bits 2-7 reserved 0 | none (STATUS carries `hb_ok`) |
| 0x03 | AIM | `pan_mdeg:i32, tilt_mdeg:i32, pan_vff:i16, tilt_vff:i16, pan_vmax:u16, tilt_vmax:u16, pan_amax:u16, tilt_amax:u16` (20 bytes). Angles in 1/1000 deg in the home frame (0,0 = homed level forward, pan + = CCW from above, tilt + = up). `vff` feed-forward, 0.1 deg/s units. `vmax` 0.1 deg/s, `amax` deg/s^2 | ACK only on error |
| 0x04 | HOME | `axis_mask:u8` (bit0 pan, bit1 tilt) | ACK |
| 0x05 | FLYWHEEL | `duty_permille:u16` (0..1000) | ACK on refusal |
| 0x06 | FIRE | `fire_id:u16, n:u8` (1..3 balls) | ACK(result) then one `EVENT 2` per ball |
| 0x07 | ARM | `mode:u8` (1 arm, 0 disarm), `shot_cap:u8`, `timeout_s:u16` | ACK(result) |
| 0x08 | CFG_SET | `key:u8, value:i32` | ACK |
| 0x09 | CFG_GET | `key:u8` | `CFG_VAL` |
| 0x0A | CFG_SAVE | none (writes the last-sector flash image; only in SAFE or IDLE) | ACK |
| 0x0B | CLEAR_FAULT | none (only in SAFE, E-stop released) | ACK |
| 0x0F | BOOTSEL | `magic:u32 = 0xB007B007` (only in SAFE) | none; resets into the ROM USB bootloader |

`AIM` semantics. The MCU runs, at 1 kHz per axis, a velocity-limited, acceleration-limited position servo:
`v_cmd = clamp(vff + kp*(target - pos), +-vmax)`, then `v += clamp(v_cmd - v, +-amax*dt)`; `kp = 20 /s` (so a 1 deg error asks 20 deg/s, capped
by vmax); targets are clamped to `[min,max]` of the config. An `AIM` older than 250 ms zeroes `vff`; older than 1 s the axis holds. Stopping
distance respected: the target is clipped so that `v^2/(2*amax)` fits before the soft limit. Accepted in IDLE, ARMED, ENGAGING, FIRING; refused (ACK
`BAD_STATE`) in SAFE and FAULT.

`FIRE` semantics. Accepted only in state ENGAGING with `flywheel applied >= requested` for >= `spinup_ms` (cfg 16), `hb_ok`, `DEADMAN_OK`, not
`FIRE_INHIBIT`, both axes homed, `tilt >= fire_tilt_min`, `shots_arm + n <= shot_cap_active`, time since the last ball >= `min_shot_interval_ms`.
`fire_id` is remembered (last 8): a retransmitted `FIRE` with a known id returns the old result and shoots nothing. One ball = pusher servo to
`pusher_ext_us`, dwell `pusher_dwell_ms`, back to `pusher_ret_us`, dwell, then the next ball; `EVENT 2` is sent when the retract completes.

`ARM` semantics. `mode=1` is accepted only in IDLE with: key on, E-stop released, no fault, `hb_ok`, `DEADMAN_OK`, both axes homed. The active cap is
`min(shot_cap, hard_shot_cap)` and the active timeout `min(timeout_s, hard_arm_timeout_s)` (cfg 10, 11; defaults 10 and 120): the host cannot raise
the hard values at runtime (they change only by `CFG_SET` + `CFG_SAVE` in SAFE, a deliberate maintenance act).

### 2.2 MCU -> host

| id | name | payload |
|---|---|---|
| 0x81 | HELLO_ACK | `major:u8, minor:u8, result:u8 (0 ok, 1 version mismatch), fw_major:u8, fw_minor:u8, fw_patch:u8, uid:u8[8] (RP2040 flash unique id), cfg_crc:u32` (18 bytes) |
| 0x82 | STATUS | 29 bytes, below |
| 0x83 | ACK | `cmd_id:u8, cmd_seq:u8, result:u8, info:u8` |
| 0x84 | CFG_VAL | `key:u8, value:i32` |
| 0x85 | EVENT | `code:u8, arg:i32` (below) |
| 0x86 | DEBUG | ASCII, <= 48 bytes, never needed for operation |

`STATUS` (29 bytes, offsets from payload start):

| off | field | type | notes |
|---|---|---|---|
| 0 | `t_ms` | u32 | MCU uptime, ms |
| 4 | `state` | u8 | 0 SAFE, 1 IDLE, 2 ARMED, 3 ENGAGING, 4 FIRING, 5 FAULT |
| 5 | `flags1` | u8 | b0 estop_pressed (1 = open circuit), b1 key_on, b2 pan_homed, b3 tilt_homed, b4 pan_hall (raw, 1 = magnet), b5 tilt_hall, b6 wd_tripped (latched until link restored), b7 hb_ok (valid heartbeat < 300 ms ago) |
| 6 | `flags2` | u8 | b0 pan_moving, b1 tilt_moving, b2 steppers_enabled, b3 pusher_extended, b4 fire_inhibit (as received), b5 deadman_ok (as received), b6 pan_at_limit, b7 tilt_at_limit |
| 7 | `pan_steps` | i32 | output-axis microsteps from home (home offset applied) |
| 11 | `tilt_steps` | i32 | idem |
| 15 | `fw_duty` | u16 | applied flywheel duty, permille |
| 17 | `fw_rpm` | u16 | 0xFFFF = no tach fitted (v1 hardware has none) |
| 19 | `shots_total` | u16 | balls fed since boot |
| 21 | `shots_arm` | u8 | balls in the current arm |
| 22 | `shot_cap` | u8 | active cap (0 when not armed) |
| 23 | `fault_bits` | u16 | below |
| 25 | `arm_left_s` | u16 | seconds until arm timeout (0 when not armed) |
| 27 | `last_fire_id` | u16 | last accepted `fire_id` |

Positions are microsteps because that is what the firmware knows; the host converts with the per-axis `steps_per_rev` (cfg 1, 2), read back at
start-up (defaults: pan 16000 = 5:1 x 200 x 16, tilt 12800 = 4:1 x 200 x 16, so 44.44 and 35.56 steps/deg, 0.0225 and 0.0281 deg/step).

`fault_bits`: b0 homing_timeout (> 15 s), b1 driver_diag (TMC2209 DIAG high > 50 ms: stall/overtemp/short), b2 cfg_invalid (flash CRC bad at
boot: defaults loaded), b3 shot_counter_inconsistent (internal assertion `shots_arm <= cap` failed), b4 hall_stuck (hall still active after moving
> 5 deg away), b5 usb_overflow. Faults latch; `CLEAR_FAULT` only in SAFE with the E-stop released.

`EVENT` codes (arg): 1 STATE_CHANGE (`new_state | reason<<8`), 2 SHOT (`fire_id<<8 | ball index`), 3 HOMED (axis mask), 4 FAULT (bit index),
5 WATCHDOG (ms since last valid heartbeat), 6 ESTOP_EDGE (new `estop_pressed`), 7 KEY_EDGE (new `key_on`), 8 BAD_FRAMES (count since last).
State-change reasons: 1 boot, 2 watchdog, 3 estop, 4 key_off, 5 disarm, 6 arm_timeout, 7 shot_cap, 8 fault, 9 operator_arm, 10 link_up,
11 new_session.

`ACK.result`: 0 OK, 1 BAD_STATE, 2 INTERLOCK (E-stop/key), 3 INHIBIT (fire_inhibit or no dead-man), 4 CAP (cap/timeout/interval), 5 RANGE, 6 BUSY,
7 UNKNOWN_CMD, 8 BAD_ARG, 9 NOT_HOMED, 10 NOT_SPUN (spin-up time not elapsed).

**Config keys** (`CFG_SET/GET`; changes apply only in SAFE/IDLE; defaults are what firmware boots with when flash is blank):

| key | name | unit | default |
|---|---|---|---|
| 1, 2 | pan_steps_per_rev, tilt_steps_per_rev | output microsteps | 16000, 12800 |
| 3, 4 | pan_min, pan_max | mdeg | -170000, +170000 |
| 5, 6 | tilt_min, tilt_max | mdeg | -15000, +70000 |
| 7, 8 | pan_home_off, tilt_home_off | microsteps (position at hall edge) | 0, 0 (calibrate) |
| 9 | dir_invert_mask | bit0 pan, bit1 tilt | 0 |
| 10 | hard_shot_cap | balls per arm | 10 |
| 11 | hard_arm_timeout_s | s | 120 |
| 12, 13 | pusher_ext_us, pusher_ret_us | servo pulse us | 1900, 1100 [est] |
| 14 | pusher_dwell_ms | ms each way | 120 [est] |
| 15 | min_shot_interval_ms | ms | 300 |
| 16 | spinup_ms | ms | 700 [est] |
| 17 | fire_tilt_min_mdeg | mdeg | 0 (never fire below level) |
| 18 | wd_ms | ms (clamped <= 300) | 300 |
| 19 | homing_speed | 0.1 deg/s | 200 |

### 2.3 Homing and the stepper drivers (firmware behaviour, v1.1)

**Homing works from any start position** (`HOME`, IDLE only). The final approach is always from the negative side, at `homing_speed`, so the edge that sets home is
always the rising edge met moving positive and the zero is the same wherever the axis started:

1. Hall already active: **CLEAR**: back out in the negative direction at `homing_speed` until the sensor releases, plus 2 deg more, then step 4.
2. Hall inactive: **SEEK_POS** at the search speed (100 deg/s, capped by the 25 ksteps/s limit) for the negative half-range + 10 deg (pan 180 deg, tilt 25 deg). A start on
   the negative side must meet the window within it.
3. No sensor yet: the start was on the positive side. **SEEK_NEG** at the search speed for the full range + 20 deg (pan 360 deg, tilt 105 deg) until the sensor turns up;
   none: `homing_timeout`. Any hit during 2 or 3 goes to CLEAR (through the window, out its negative edge).
4. **APPROACH** at `homing_speed` (positive); on the hall rising edge the position is set to `home_off` (cfg 7, 8) and the axis **BACKs OFF** 3 deg, then `HOMED`.

Worst case at the defaults (pan, start at the +170 deg soft limit): 1.8 s + 3.5 s + about 0.7 s = about 6 s, inside the 15 s `homing_timeout` (tilt about 2 s). Note for the
mechanics: SEEK_POS moves up to 180 deg (pan) / 25 deg (tilt) *positive* from an unknown start, which can pass the positive soft limit. A pan hard stop (printed tab on the rotor at +90 deg, one M3 stud in
the stator at -90 deg, engages from either side at +-177.9 deg) catches it (the stepper skips harmlessly at the stop at 100 deg/s / 1.2 A, then SEEK_NEG reverses) and the cable loop is sized for +-178; tilt is caught by
hard stops at -18 / +73 deg (trunnion lug against the yoke_right sector blocks). The twin (`SimRP2040`) implements the same procedure.

**Stepper drivers.** Two TMC2209 on two separate single-wire UARTs at 115200 baud (pan: UART1, GP20 TX / GP21 RX; tilt: UART0, GP0 TX / GP1 RX), each TX through 1 kohm to the
driver's PDN_UART node. The strap MS1 = MS2 = VIO (16 microsteps) gives *both* drivers UART address 3, so they cannot share a wire; a dead link still runs the axis at 16
microsteps with the VREF-pot current. Both DIAG outputs are wired-OR to GP22 (active high) and feed `driver_diag` (high for more than 50 ms). The firmware writes at boot:
GCONF `0x1C0` (UART control, microsteps from MRES, current from IHOLD_IRUN), CHOPCONF `0x14000053` (16 microsteps, interpolated to 256), IHOLD_IRUN (IRUN 21 = 1.2 A RMS,
IHOLD 10 = 0.6 A, IHOLDDELAY 6), TPOWERDOWN 20, and reads CHOPCONF and IHOLD_IRUN back. **Chopper mode: stealthChop below TPWMTHRS = 234 (3,200 steps/s: 72 deg/s pan,
90 deg/s tilt), spreadCycle above it**, where the torque is needed (6.3). A DEBUG frame (id 0x86) after the first `HELLO` reports the read-back (`tmc pan=0 tilt=0`, 0 = ok).

## 3. Safety and fire-control state machine

### 3.1 States and transitions (the MCU implements all six; the app mirrors the MCU `state` and adds its own sub-states)

| state | meaning | outputs |
|---|---|---|
| **SAFE** | power-up default, after watchdog / E-stop / key off / new session / fault | flywheel 0, pusher retracted, steppers: HOLD (EN low, targets frozen) after a watchdog; DISABLED (EN high) after E-stop or fault; `AIM`/`FIRE`/`FLYWHEEL` refused |
| **IDLE** | link up, disarmed, motion allowed | flywheel 0, pusher retracted, steppers enabled; `HOME`, `AIM` accepted |
| **ARMED** | operator armed this session | as IDLE, `FLYWHEEL` accepted; arm timer running |
| **ENGAGING** | ARMED and `flywheel duty > 0` requested (app: confirmed track in sector, spinning up) | flywheel on, `FIRE` accepted once spun up |
| **FIRING** | a pusher cycle is in progress (1-3 balls) | back to ENGAGING when the burst completes |
| **FAULT** | latched fault | as SAFE (steppers disabled) until `CLEAR_FAULT` |

Transitions (MCU):
- boot -> SAFE (reason 1). GPIO pull-downs on the MOSFET gates and the servo power switch keep outputs off while the RP2040 is in reset or
  being flashed. The RP2040 hardware watchdog (8 s) resets a hung firmware into SAFE.
- SAFE -> IDLE: valid heartbeats for >= 1 s (20 beats) after `HELLO`, `wd_tripped` clears, no fault, E-stop released (reason 10).
  Key state is irrelevant for IDLE. After a watchdog trip this is **not** re-arming: ARMED needs a new `ARM`.
- IDLE -> ARMED: `ARM(mode=1)` with the preconditions in 2.1 (key on, E-stop released, no fault, `hb_ok`, `DEADMAN_OK`, homed). Resets
  `shots_arm`, starts the timer (reason 9).
- ARMED -> ENGAGING: `FLYWHEEL(duty>0)`. ENGAGING -> ARMED: `FLYWHEEL(0)` or no refresh for 500 ms or `DEADMAN_OK` low for > 1 s (spin-down).
- ENGAGING -> FIRING: accepted `FIRE`. FIRING -> ENGAGING: burst complete (`EVENT 2` per ball).
- ARMED/ENGAGING/FIRING -> IDLE (flywheel off, pusher retracted): `ARM(mode=0)` (5), arm timeout (6), `shots_arm >= cap` (7). IDLE again needs a
  new `ARM`; the app shows "re-arm".
- any -> SAFE: heartbeat missing > `wd_ms` (2, steppers HOLD); E-stop open (3, steppers DISABLED, homed flags cleared, because motion
  without drive is not tracked); key off (4); new `HELLO.session` (11).
- any -> FAULT: a `fault_bits` condition (8).

Latency of the cut: watchdog check at 1 kHz, flywheel PWM forced to 0 and pusher commanded back in the same tick: <= 301 ms after the last
valid heartbeat. A pusher cycle already in progress finishes its retract (<= 240 ms) unless the E-stop/key hardware cut removed servo power.

### 3.2 Preconditions to fire and who enforces each

Fire (a `FIRE` command that the MCU executes) requires ALL; the app sends `FIRE` only when its own check passes, the MCU re-checks what it can.

| # | rule (briefing) | app (primary for 1-4, 6) | MCU firmware | hardware |
|---|---|---|---|---|
| 1 | no person in frame | any `person` det with conf >= 0.15 (the detector's own person threshold) anywhere in the last 1.0 s, or detector stale (> 250 ms with no valid reply; a reply carrying an `error` key is not valid), or detector socket down -> `FIRE_INHIBIT` in every heartbeat and no `FIRE`; the target box must not overlap a person box | refuses `FIRE` while the received `FIRE_INHIBIT` is set (ACK 3) | - |
| 2 | operator ARMED this session | UI ARM (explicit button, shows key/E-stop state); app state ARMED/ENGAGING requires MCU `state >= 2` | `ARM` preconditions; `FIRE` only in ENGAGING; new `HELLO.session` -> SAFE | - |
| 3 | confirmed drone/balloon/target track | tracker `confirmed` = >= 5 consecutive detector replies, class in {drone, balloon, target}, conf >= 0.5. Default mode (user, 2026-10-11) = **session arm**: once ARMED, every confirmed in-sector track is engageable with no per-track click; optional stricter mode = operator `authorize` of each track id for N rounds, valid 10 s (UI toggle) | - | - |
| 4 | aim point inside the geofence sector | sector = `{az_min, az_max, el_min, el_max}` deg in the home frame (+ optional `r_max`), set in the UI, persisted in `~/.config/cuas-lite/sector.toml`; both the target position and the commanded barrel direction must lie in it; el_min >= 0 deg | `pan/tilt` soft limits (cfg 3-6) clip `AIM`; `fire_tilt_min` refuses fire below level (ACK 5) | mechanical stops at the limits |
| 5 | per-arm shot cap and arm timeout not exceeded | counts `EVENT 2`, stops at the UI cap (default 5) and timeout (default 60 s), then disarms | counts actual pusher cycles; clamps cap/timeout to cfg 10/11; leaves ARMED at cap/timeout | - |
| 6 | web dead-man fresh | UI posts `/api/deadman` every 200 ms while the operator holds the "hold to fire" control; app sets `DEADMAN_OK` iff last post < 500 ms | refuses `FIRE` without `DEADMAN_OK`; spins down flywheel if low > 1 s | - |
| 7 | arm key on and E-stop released | shows both, treats as not-armed | reads both; key off or E-stop open -> SAFE immediately | **key switch and NC E-stop in series in the 12 V "weapon rail" feeding both flywheel MOSFET drains and the pusher servo supply; open = no power, with the RP2040 off, hung or hostile**. A second contact of each (or a divider across the rail) feeds RP2040 GPIO |
| 8 | watchdog 300 ms | stalls of its own loop surface as a trip, by design | trips as in 3.1 | MOSFET gate pull-downs |

Also: the stand-in/sim twin implements rows 2, 5-8 exactly as the firmware does (section 4.4), so the sim measures the real gate logic. Two
further app rules: the app never sends `FIRE` while its own pointing-error exceeds `min(0.5 deg, 0.25 * target angular radius)` (the `AIM_TOL` role
of `examples/cuas/turret.py`), and any exception in the control loop sets the app to DISARM + stops heartbeats (fail closed).

### 3.3 App fire-control stages (the same names as the MCU where possible)

`SAFE` (MCU not IDLE+, or link down) -> `IDLE` (scanning optional: pan sweep +-60 deg is **off** by default, enabled from the UI) -> `ARMED` -> `ENGAGING` (a
confirmed, in-sector track (authorized, in the stricter mode): AIM + FLYWHEEL, wait for spin-up and pointing error) -> `FIRING` (FIRE burst of 1-3, then re-evaluate).
Target selection: nearest by estimated range among engageable tracks; give up after 5 rounds without a hit for 6 s (values from `turret.py`).

## 4. App I/O interfaces (Python Protocols), sim and real implementations

All in `python/napl/lite/io.py` (Protocols and dataclasses) so firmware-independent code never imports an implementation.

```python
class Clock(Protocol):
    def now(self) -> float: ...                    # seconds, monotonic; sim: sim time
    def sleep_until(self, t: float) -> None: ...   # lockstep SimClock raises if called (the harness drives time)

@dataclass(frozen=True)
class Intrinsics:  w: int; h: int; fx: float; fy: float; cx: float; cy: float; dist: tuple = (0, 0, 0, 0, 0)  # OpenCV k1 k2 p1 p2 k3

@dataclass
class Frame:  seq: int; t_cap: float; w: int; h: int; rgb: "np.ndarray | None"   # uint8 (h, w, 3); None in no-pixel sim

class Camera(Protocol):
    intrinsics: Intrinsics
    def read(self, timeout: float = 0.1) -> "Frame | None": ...   # newest frame not yet returned, or None
    def close(self) -> None: ...

class Link(Protocol):                             # raw bytes both ways: tty fd, pty slave, or in-process loopback
    def write(self, data: bytes) -> None: ...
    def read(self, timeout: float = 0.0) -> bytes: ...

class Gimbal(Protocol):                           # speaks section 2 over a Link; ONE implementation (napl.lite.gimbal.SerialGimbal)
    def hello(self) -> HelloAck: ...
    def heartbeat(self, deadman: bool, inhibit: bool) -> None: ...
    def aim(self, pan_deg, tilt_deg, pan_vff=0.0, tilt_vff=0.0, vmax=(240.0, 150.0), amax=(2000.0, 1500.0)) -> None: ...
    def home(self, pan=True, tilt=True) -> None: ...
    def flywheel(self, duty: float) -> None: ...
    def fire(self, n: int = 1) -> int: ...        # -> fire_id
    def arm(self, shot_cap: int, timeout_s: int) -> None: ...
    def disarm(self) -> None: ...
    def pump(self) -> None: ...                    # drain the link, update .status and .events
    status: Status                                 # latest STATUS (+ ring: status_at(t))
    events: "collections.deque[Event]"

class DetectorClient(Protocol):
    def submit(self, frame: Frame) -> None: ...                    # writes shm slot + request line; never blocks
    def poll(self) -> "DetResult | None": ...                      # newest reply, or None
    stale_s: float                                                 # seconds since the last reply (fail-closed input)
```

`SerialGimbal` takes a `Link` and a `Clock`; it is the same code in real, HIL and lockstep runs. The pieces that differ:

| interface | real (Jetson) | sim, lockstep (CI, Monte Carlo, video source) | sim, HIL (protocol conformance) |
|---|---|---|---|
| Clock | `MonotonicClock` | `SimClock(world)`: `now() = world.t` | `MonotonicClock` |
| Camera | `V4L2Camera` (OpenCV `CAP_V4L2`, MJPG 1280x720@60, intrinsics from `calib.toml`) | `SimCamera(world, latency=0.030)`: samples the World every 1/60 s of sim time; `rgb=None` unless pixels are requested | `ShmCamera` reading `/dev/shm/cuas-lite-simcam` (same CLF1 layout), written by the harness from `napl.cuas.camera.Camera.grab` |
| Link | `TtyLink("/dev/serial/by-id/...")` | `LoopbackLink`: the twin is called in-process; bytes still go through COBS/CRC | `TtyLink(pty slave path)` |
| MCU | the Pico | `SimRP2040` (`napl.lite.sim_mcu`) | same, behind a pty |
| Detector | `SocketDetector` (shm + socket) | `TruthDetector` in-process (projected truth + noise, no pixels) | `StandinDetectorProcess` on the real socket (truth or classical `napl.cuas.tracking.detect`) |

Real/sim parity rules: app code never calls `time`, `open`, `socket` or `os` outside the implementations; `App.tick(now)` is a run-to-completion
step (poll camera, detector, gimbal; decide; send), so the real runner is `while True: app.tick(clock.now())` with the select loop of section 1
and the lockstep harness calls it once per control period.

### 4.1 `napl program` layout (sim)

Use the existing `[interfaces.native]` of `python/napl/program.py` (the program owns the World; env `NAPL_MODEL`, `NAPL_RUN_DIR`) so no change to the
runner is needed (Q5). Scenario: `examples/cuas-lite/*.scenario.toml`, `[program] cmd = ["python", "{dir}/run.py"]`.

**Lockstep (default; deterministic):** `run.py` is the harness, one process: `World.from_env(ctl_ticks=5)` (5 ms control period, dt 1 ms), the
scenario's targets, `SimRP2040(world, LoopbackLink)`, `SimCamera`, `TruthDetector`, `App(...)`. Loop per control period: `twin.step(world)`
(reads twin-bound SetAttrs back out of the World: joints, hall), `app.tick(clock.now())`, `world.step(1)`. Camera frames every 16.67 ms of sim time
(accumulated, taken at the next control tick: <= 5 ms jitter). All actuator writes are SetAttr through the World so they are logged and the replay
hash reproduces (`native.json`), exactly as for `examples/cuas`. The app's decisions are deterministic because the detector sees truth, not pixels;
so the hash is repeatable run to run on any machine, CPU only.

**HIL (real-time, rate 1.0 only):** the harness additionally (a) starts `SimRP2040` on a pty: the harness holds the **master** end, the app opens the
**slave** path (`os.openpty`, `tty.setraw(slave)`, `os.ttyname(slave)`, same as `FeetechPort`), passed to the app child as env `CUAS_GIMBAL`; (b)
renders the camera at 30 Hz into `/dev/shm/cuas-lite-simcam` for `ShmCamera`; (c) starts the stand-in detector process and `python -m
napl.lite.app` as children (process group killed with the harness). The twin's clock is the World's: the 300 ms watchdog counts sim time, which is
why HIL is limited to rate 1.0. Replay hash: each run's own log replays to its own hash; run-to-run equality is not promised (wall-clock
thread timing), as for the PX4 scenarios. Use: protocol conformance of firmware vs twin (the same `tests/lite/protocol_vectors.json` run against
both), the web UI against a sim, and a smoke test that HIL ends with no fire in a person scenario.

The harness is also the evaluator: it holds ground truth (positions, hits, persons) and writes `trace.csv` columns for the monitors (section 7).
Videos use the lockstep run's `replay.naplog` via `tools/record_sim.py`, as for `examples/cuas`.

### 4.2 Sim twin and the napl actuators (`SimRP2040`)

Implements section 2 byte for byte (same `protocol.py` encode/decode module as the host; the firmware's C and the twin share
`tests/lite/protocol_vectors.json`: frames as hex with expected decoded fields, including the CRC check value). Behaviour:

| firmware function | twin implementation |
|---|---|
| step generator + planner | same 1 kHz velocity/accel-limited servo (2.1 `AIM` semantics), in Python, integrated in 5 ms control periods with 1 ms sub-steps; position quantised to microsteps (`steps_per_rev`) |
| step-rate limit -> joint velocity | max step rate 25 ksteps/s caps `vmax`: `v_joint_max = 25000 / steps_per_rev * 360 deg/s` (pan 562 deg/s). The planner position `q_cmd` (rad) is written each control period as `goal_position` on the joint's servo device with `goal_speed = 0` (no second profile) |
| motor torque / step loss | the joint device's torque limit is the pull-in torque at that speed (stepper part; section 6.4). If `|q_joint - q_cmd|` (against the command of the previous control period) exceeds 4 full motor steps (pan 1.44 deg, tilt 1.8 deg) the twin raises the TMC2209 DIAG pin, as the driver would; `driver_diag` follows after 50 ms of DIAG and the FAULT state clears the homed flags. No position resync (the firmware has no encoder) (optional fidelity, default on) |
| hall homing sensor | `hall = |q_joint - q_hall| < 1.0 deg` (hysteresis-free window 2 deg wide), `q_hall` = 0 deg for both axes by default, a parameter of the assembly (the CAD magnet position); homing is the firmware routine of 2.3: bidirectional search from any start, final approach from the negative side at `homing_speed`, on the edge set position to `home_off`, back off 3 deg. `home_off` defaults to 0 as in the firmware; the lockstep harness passes -1 deg (the half window) so that home-frame 0 is the window centre of a calibrated unit |
| flywheel | the applied duty ramps up 4 permille/ms (inrush limit, 0 to 800 in 200 ms) and drops at once, as the firmware; "spun" = applied duty >= requested for `spinup_ms` since the request (no tach in v1). The launcher device's `muzzle_velocity` is set to `v0 = k_v * rotor_speed` (calibrated so that duty 800 gives 30 m/s nominal) at each FIRE |
| pusher | `FIRE` -> timed sequence 120 ms + 120 ms per ball; `fire` attr = 1.0 on the launcher at the extend-complete instant (the kernel spawns one ball per `fire`); ball interval floor `min_shot_interval_ms` |
| E-stop / key | the harness exposes `world_inputs.estop`, `world_inputs.key` (scenario `[[faults]]`-style schedule); the twin reads them as GPIO. Rail cut = flywheel and pusher stop immediately and no further `fire` is written |
| heartbeat watchdog | counted in sim time on the 5 ms grid (trip detected within 5 ms of 300 ms) |
| `comms_drop` fault | `LoopbackLink.drop = True` (or pty discard in HIL), same semantics as `FeetechPort.drop` |
| sensors it does not model | `fw_rpm = 0xFFFF`, USB latency 1 ms each way |

The launcher kernel device ray-casts a zero-radius pellet (`src/mujoco/devices/launcher.cpp` "Hit test"). The sim worker must inflate target
geoms by the ball radius (11.1 mm) or add a radius attr; the kernel pool is 16 balls (`kMaxPellets`), enough for 3-5 balls/s at < 1 s flight.

## 5. Coordinate frames, calibration, camera, ballistics

### 5.1 Frames

- **B (base)**: origin at the centre of the base plate's top face on the pan axis (as `assemblies/turret.toml`); x forward (pan 0), y left, z up.
  Pan angle `theta` about +z, CCW from above positive. Pan 0 is the pan hall window centre.
- **T (tilt axis)**: the tilt axis passes through `(0, 0, h_t)` in B rotated by `theta`, along the head's y. Tilt `phi` positive raises the muzzle. `h_t` and the
  barrel offset are CAD numbers (`cad/cuas-lite/params.py`), estimated 0.12 m and 0 m.
- **Muzzle**: `M = (0,0,h_t) + Rz(theta) Ry(-phi) (l_m, 0, d_b)`, `l_m` ~ 0.15 m, `d_b` ~ 0 (barrel on the tilt axis plane). Barrel direction
  `u = Rz(theta) Ry(-phi) (1,0,0)`; `az = theta`, `el = phi`.
- **Camera C (OpenCV: x right, y down, z forward)** on the tilt assembly at `(l_c, 0, d_c)` in head coordinates, `d_c ~ +0.06 m` (Brio on top of the barrel
  housing, centred on the tilt plane), optical axis parallel to the barrel up to the boresight offsets below. `R_hc` maps C to the head frame: x_C = -y_h, y_C = -z_h, z_C = +x_h.
- Pixel -> bearing: undistort, `ray_C = ((u-cx)/fx, (v-cy)/fy, 1)`, rotate by the head attitude **at `t_cap`**, taken from the STATUS ring
  interpolated at `t_cap` (the head moves 3-4 deg in the 30 ms between exposure and now), to B. Camera position in B likewise.
- Parallax: the camera is `d_c` above the barrel. For a target at range R the aim correction is `atan(d_c / R)` = 1.15 deg at 3 m, 0.23 deg at 15 m [der, d_c = 0.06]; the fire
  control solves the intercept for the muzzle point, so the offset is geometry, not a fudge angle.

### 5.2 Camera and mode

Logitech Brio. Spec numbers quoted in section 10: 4K@30, 1080p@60, 720p@90 maxima; default diagonal FOV 90 deg = 82.1 deg horizontal x 52.2 deg vertical
(a 16:9 mode keeps the full frame when scaled).

**Mode: 1280x720 @ 60 fps, MJPG.** Reasoning: (a) 60 fps halves the exposure-to-command latency (frame period 16.7 ms vs 33 ms), and tracking a
fast quad or a swinging target is latency-limited, not resolution-limited; (b) the detector downscales to its input anyway (YOLO imgsz 640-960),
so most of 1080p's extra detail is discarded while its MJPG decode costs 2.25x the pixels on the Orin's CPU; (c) at 1280 wide a 0.30 m balloon at 15 m is 15 px
(f = 736 px, below), at 960 detector width 11 px: small but detectable; 4K is out. The mode is a CLI/config setting
(`camera = {w,h,fps}`); `1920x1080@30` is the fallback if the detector cannot see 15 m targets at 960 (Q3). The detector worker benchmarks
imgsz 640 vs 960 on the Orin Nano; the requirement is >= 30 detector replies/s with latest-frame-wins.
Camera controls fixed at start: `exposure_auto=1` (manual), `exposure_absolute` ~ 1/500 s (blur at 8 m/s), gain auto off, autofocus off with
focus at ~5 m (fixed), zoom 100 (no digital zoom); values in `calib.toml`.

**Intrinsics.** Prior from the quoted FOV: `fx = 640 / tan(82.1 deg / 2)` = **736 px**, `fy` = 360 / tan(52.2/2) = 735 px, `cx, cy` = 640, 360 [der].
Hardware use requires a one-time chessboard calibration at the used mode (OpenCV, 15 views, RMS reprojection < 0.5 px) stored in
`~/.config/cuas-lite/calib.toml`; the app refuses to ARM without a `calib.toml` newer than the camera mode. The sim camera uses
`Camera(sim, path, fov_v=0.911 rad (52.2 deg), width=1280, height=720)` of `napl.cuas.camera`, which gives `f_px = 735.6` and zero distortion.

**Boresight calibration** (a command, `python -m napl.lite.calib boresight`, run indoors with the balls loaded and the flywheels at the
operating duty): fire 5 balls each at a 0.3 m target board at 5 m and 8 m, mark impacts; the tool solves (a) camera-to-barrel angular offset
`(d_az, d_el)` as a constant in the camera frame, (b) a muzzle-velocity scale `k_v`, from the impact offsets at the two ranges; stored in `calib.toml`.
The mechanical `d_c` stays a CAD constant. Re-run after any camera or barrel remount.

### 5.3 Ballistics and the solution

Inputs to the lead solver (reuse `napl.cuas.ballistics.solve_lead(Pellet(v0, mass, diameter, cd), muzzle, tgt_pos, tgt_vel, wind)`, point mass,
RK4, ISA density; it needs no change): muzzle position M (above), target position and velocity in B from the tracker (alpha-beta filter, range from
box size: `range = fx * size_m(class) / box_px`, class sizes in config: balloon 0.30 m, target 0.30 m, drone 0.30 m [est]), `BALL`, `v0 = k_v * v0_nominal`, wind (0
indoors; optional manual entry in the UI). Time compensation: the target is extrapolated by `(now - t_cap) + t_actuator + t_fire`, with
`t_actuator = 0.060 s` [est] (planner + step lag) and `t_fire = 0.15 s` [est] (FIRE command -> ball leaves: pusher extend 120 ms + 30 ms). The output
`az, el` becomes `AIM(pan, tilt)` with `vff` = d(az, el)/dt of the solution from the last two frames.

**Ball: Nerf Rival round.** d = 22.2 mm, m = 1.86 g (**[src]**; two secondary sources, no Hasbro primary page found; fidelity **secondary, to be
weighed on the bench**), 2D area 3.87e-4 m^2. Muzzle speed of Rival blasters: "most shooting between 90 and 120 FPS" = 27-37 m/s (**[src]**
secondary); our flywheel launcher is a design variable: nominal **30 m/s** [est], v0 sigma 1.5 m/s [est], direction spread sigma 10 mrad per axis [est]
(foam balls from flywheels are loose). Drag: quadratic, `a = g - k |v-w| (v-w)`, `k = rho * Cd * A / (2 m)`, **Cd = 0.47** (sphere in turbulent
flow, Wikipedia "Drag coefficient"; at Re = v d / nu = 30 * 0.0222 / 1.5e-5 = 44,000 a dimpled/foam sphere may sit between 0.4 and 0.5; **[est]**, fitted on the bench from drop at 5 and
10 m). `k = 0.0587 /m` (22.2 mm / 1.86 g) or 0.0739 /m (17.8 mm / 0.95 g) [der]. Not modelled: Magnus force from backspin, ball deformation,
turbulence. Reference solutions [der], flat target at muzzle height, no wind (from integrating the same ODE with a 1 ms Euler step in a scratch script and bisecting the elevation; the sim worker
reproduces them with `solve_lead` as a unit test, tolerance 0.1 deg):

| range m | elev deg (30 m/s) | flight s | impact m/s | lead m at 2 m/s crossing | elev deg at 35 m/s |
|---|---|---|---|---|---|
| 3 | 1.1 | 0.11 | 25.1 | 0.22 | 0.8 |
| 5 | 1.9 | 0.20 | 22.3 | 0.39 | 1.4 |
| 8 | 3.5 | 0.34 | 18.8 | 0.68 | 2.6 |
| 10 | 4.9 | 0.46 | 16.7 | 0.91 | 3.6 |
| 12 | 6.4 | 0.59 | 14.9 | 1.17 | 4.7 |
| 15 | 9.5 | 0.82 | 12.7 | 1.63 | 6.9 |

Consequences: a 5 m/s quad crossing at 10 m needs a 2.3 m lead; the lead solution is a function of range, so the ranging error of section 0
item 2 maps straight into misses. Tolerances for a 0.30 m balloon (radius 0.15 m + ball radius 0.011 m = 0.161 m): +-0.92 deg at 10 m,
+-0.61 deg at 15 m. Sim and fire control MUST use the same `BALL` and `solve_lead`; the sim's truth is the kernel launcher with the true (noisy)
`v0`, spread and wind.

## 6. Mechanical envelope (for the CAD worker)

### 6.1 Drive ratios and kinematics

| axis | motor pulley : driven | ratio | output steps/rev (16 microsteps x 200) | resolution | vmax (spec) | amax (spec) | range |
|---|---|---|---|---|---|---|---|
| pan | GT2 20T : 100T, 6 mm belt | 5:1 | 16000 | 0.0225 deg | 240 deg/s (motor 200 rpm) | 2000 deg/s^2 | +-170 deg (hall at 0) |
| tilt | GT2 20T : 80T, 6 mm belt | 4:1 | 12800 | 0.0281 deg | 150 deg/s (motor 100 rpm) | 1500 deg/s^2 | -15 deg to +70 deg (hall at 0) |

Belt: GT2 (2 mm pitch, standard; verify belt length at purchase), closed loop, centre distances set by a slotted motor mount (adjustment >= +-4 mm), belt
tension target 5 N preload per side [est] (keeps shaft radial load < 28 N). Tilt range: -15 deg (barrel into its own cradle; the camera never sees the
base) to +70 deg (drones overhead at close range). The hopper rides the pan group, not the head: balls drop through a feed elbow into the hollow right
trunnion (6807ZZ bearing in place of the right rod + 608), turn 90 deg on the tilt axis and run down a chute to the breech; CAD checks the chute's downhill slope at
tilt -15/0/35/70 (23/38/73/72 deg). Backlash budget [est]: < 0.2 deg output at the pan, < 0.3 deg at the tilt.

### 6.2 Required rates against targets (3-15 m) [der: omega = v/R]

| target | speed | R = 3 m | 5 m | 10 m | 15 m |
|---|---|---|---|---|---|
| balloon drift | 1 m/s | 19 deg/s | 11 | 5.7 | 3.8 |
| hanging target swing | 1.5 m/s peak | 29 | 17 | 8.6 | 5.7 |
| small quad | 8 m/s crossing | 153 | 92 | 46 | 31 |
| small quad | 3 m/s | 57 | 34 | 17 | 11 |

Pan 240 deg/s covers an 8 m/s quad at 3 m with margin; a slew of 60 deg takes `0.12 s + 0.13 s + 0.12 s` = **0.37 s** at 240 deg/s, 2000 deg/s^2 [der]. Tilt
needs far less (targets move mostly in azimuth): 150 deg/s.

### 6.3 Stepper torque check (NEMA 17 SY42STH38-1684A, `parts/actuators/soyo-sy42sth38-1684a-nema17.toml`)

Datasheet numbers from the part file (S2, quoted in section 10): holding torque 36 N*cm = 0.36 N*m, 1.68 A/phase, 1.65 ohm, 3.2 mH, max radial force 28 N at
20 mm from the flange, 285 g. Torque-speed curve is not in that datasheet; the figures below use holding torque as the ceiling and a 50 % pull-out
allowance at the operating speeds [est], to be replaced by the curve at purchase.

- **Pan.** From the CAD solids (`interface.mass_split`, f66ad96): pan group 955 g + tilt group 453 g + 40 balls 74 g; inertia about the pan axis
  J = 0.0099 (hopper empty) to 0.0109 (full) kg*m^2 [der, bought items as point masses]. At 2000 deg/s^2 (34.9 rad/s^2): T_out = 0.380 N*m; at the motor = 0.380 / 5 / 0.9
  (belt efficiency [est]) = **0.085 N*m**. Rotor inertia (not published, ~5.7e-6 kg*m^2 [est]) adds ~0.001 N*m. Total ~0.086 N*m = 24 % of holding, 48 % of the 50 % allowance. OK, ~2x margin.
- **Tilt.** The tilt group is 453 g (hopper moved to the pan group). Its CG hangs 34 mm below the tilt axis, so the balance check is a static torque budget, not a
  CG radius: budget 0.177 N*m output (= the original 1.2 kg x 15 mm), CAD measures **0.151-0.153 N*m** over the tilt range (~15 % margin) -> 0.042 N*m at the motor
  (/4/0.9). Dynamic at 1500 deg/s^2: <= 0.056 N*m at the motor (the 1.2 kg / 7.7e-3 kg*m^2 bound; the real 453 g group is lighter). Total <= 0.098 N*m = 27 % of holding
  (54 % of the allowance). Thin static margin: any head addition (bigger wheels, sensors) must re-run `head_static_torque_within_spec_budget`; a ~300 g ballast
  above the axis is the fallback, or a gas-spring/rubber-band counterbalance.
- **Speed.** At 12 V, 200 rpm pan: back-EMF amplitude = 0.1515 V*s/rad (part file `kt_equiv_phase`) x 20.9 rad/s = 3.2 V; electrical frequency = 50 pole pairs x 200 rpm = 167 Hz,
  coil reactance 2*pi*167*3.2 mH = 3.4 ohm; available current ~ (12 - 3.2)/sqrt(1.65^2 + 3.4^2) ~ 2.3 A peak vs 1.68 A rated [der, first order, ignores chopper losses]: 12 V is enough for 200 rpm
  at the 1.2 A RMS setting, and little more; hence a **12 V minimum bus for the steppers (4S/16.8 V gives headroom)**.
- **Shaft.** Radial belt load at 0.1 N*m with the 20T pulley = 0.1 / 6.37 mm = 15.7 N + preload effect < 28 N (**[src]**); with a 16T it would be 19.6 N + preload, hence 20T.
- **Power loss:** at the tilt, a stepper power cut lets the barrel drop (453 g tilt group, ~0.15 N*m static unbalance): the cradle stop catches it at -15 deg; the CAD worker adds a soft stop.

### 6.4 Fasteners, bearings, fab limits

- **Fasteners:** M3 throughout (SHCS M3x8/12/16/20, nylock where loaded, brass heat-set inserts for repeatedly serviced joints: motor mount, hopper lid); motor flange 4 x M3 on a 31 mm
  square pattern, 22 mm pilot (**[src]**, part file). No other thread sizes except 1/4-20 for the Brio tripod mount (**[src]**: "Tripod Mounting Option: Yes, 1/4" 20 thread") and M5/M8
  if the lazy susan demands it.
- **Pan bearing:** a lazy-susan (turntable) bearing, 100-150 mm (typical retail item; confirm hole pattern and axial play at purchase), carrying the whole head; stator plate on one ring, rotor plate on the other. A
  pan axis shaft is not used; the pan driven pulley is bolted to the rotor plate concentric with the bearing (100T GT2, 63.7 mm pitch diameter [der]).
- **Tilt bearings:** left cheek: one **608** (8 x 22 x 7 mm, standard size; not fetched, verify) on an 8 mm steel stub rod / printed hub, 80T driven pulley fixed to it.
  Right cheek: one **6807ZZ** (35 x 47 x 7 mm, thin-section; verify) around the hollow printed trunnion the balls pass through (no rod on that side). Two more 608 as
  belt idler/tensioner.
- **Print limits:** every part fits the 220 mm bed with >= 10 mm margin (**max footprint 200 x 200 mm**), PLA/PETG, 0.4 mm nozzle, 0.2 mm layers, minimal supports (no part needs more
  than one orientation with supports under < 30 % of its footprint; split the base into two halves with dovetail+M3 if it exceeds 200 mm).
- **Parametric source:** `cad/cuas-lite/params.py` (all dimensions incl. `BALL`) -> `build123d` parts -> STL/STEP + napl visual meshes, masses, inertias from the same solids.
  The napl assembly's `[params]` mass of every printed part comes from the CAD volume x density (PLA 1240 kg/m^3, PETG 1270 kg/m^3 [est], infill 20 % with 2 perimeters [est]).

### 6.5 Printed parts (rough sizes; the CAD worker finalises)

| part | rough size mm | note |
|---|---|---|
| base_stator_plate | 200 x 200 x 6 | lazy-susan ring 1, electronics-tray posts, cable pass-through, mount feet |
| electronics_tray | 180 x 110 x 25 | Jetson dev kit (published envelope 100 x 79 x 21 mm, retailer-listed; confirm), Pico + driver board, buck converters, XT60; vented |
| pan_rotor_plate | 180 x 180 x 6 | bearing ring 2, 100T pulley mount, yoke posts, hall magnet pocket |
| pan_motor_mount | 55 x 55 x 30 | slotted +-4 mm tension slot, NEMA 17 31 mm pattern, hall sensor pocket on the stator |
| yoke_left / yoke_right | 120 x 45 x 100 | 608 seat (left) / 6807ZZ seat (right), tilt motor mount on the left cheek, soft stop |
| tilt_hub + pulley adapter | 50 dia x 20 | pins the 80T pulley to the 8 mm rod, magnet pocket (tilt hall) |
| tilt_motor_mount | 55 x 55 x 25 | slotted; on the left cheek |
| launcher_chassis (cradle) | 150 x 70 x 70 | carries barrel, flywheel cage, pusher, camera, feed chute from the right trunnion; the CG balance part |
| barrel | 24 ID (ball 22.2 + 1.8 clearance) x 110 long | splits into two clamshell halves for printing |
| flywheel_cage | 70 x 60 x 45 | two 180-size motor seats (shaft gap = ball diameter - 3 mm compression [est] = 19.2 mm); wheels = silicone/rubber discs on the motor shafts (bought) |
| hopper + lid + feed_elbow | 90 dia x 100 + 26 ID elbow | pan group (does not tilt); holds ~40 balls (380 cm^3 [der]); gravity feed into the hollow right trunnion (6807ZZ); jam-clearing lid. Bench risk: a 76 mm level run along the tilt axis, 3.1x push margin [est] |
| pusher_slide | 40 x 25 x 20 | SG90/MG90-class servo (9 g) drives a slider; one ball per stroke |
| camera_mount | 110 x 40 x 35 | clamps the Brio (102 x 27 x 26.5 mm, 65 g (**[src]**)) to the chassis, 1/4-20 insert, +-3 deg boresight shims |
| front_panel | 120 x 80 x 4 | key switch, E-stop (22 mm hole), arm LED, buzzer |
| cable_guides, clips | small | strain relief at the turntable |

Mass budget: pan group 0.96 kg + tilt group 0.45 kg + balls 0.07 kg (CAD f66ad96) + base 1.5 kg [est]; total < 4 kg.

### 6.6 Electrical and pins (proposal; `firmware/cuas-lite-rp2040/include/pins.h` is the single source)

Pico GPIO (as built; `include/pins.h` is the single source): tilt TMC2209 UART0 TX/RX = GP0/GP1; pan STEP/DIR/EN = GP2/3/4; tilt STEP/DIR/EN = GP5/6/7; pan/tilt hall (active low,
internal pull-up) = GP8/9; E-stop sense (1 = circuit closed) / key sense (1 = on) = GP10/11 (internal pull-down: a broken wire reads "pressed"); flywheel PWM for both gate
drivers = GP12 (PWM6A, 20 kHz); flywheel tach (optional, unused in v1) = GP13; **pusher servo PWM 50 Hz = GP14** (PWM7A: GP12 and GP13 are the two channels of one PWM slice, so
a 50 Hz servo on GP13 would have forced the flywheel to 50 Hz); red/green LED, buzzer = GP15/16/17; pan TMC2209 UART1 TX/RX = GP20/GP21; **TMC2209 DIAG (both, wired-OR) =
GP22**. The earlier "debug UART = GP0/1" is gone (DEBUG frames replace it).

TMC2209 in STEP/DIR mode with UART configuration (section 2.3): microstep pins strapped for 16 (MS1 = MS2 = VIO, interpolation to 256 by the chip: parts file `microstep_pin_modes`),
which sets UART address 3 on both drivers, hence **two UARTs, one per driver**; current set by IHOLD_IRUN to 1.2 A RMS (the VREF pot is ignored once GCONF I_scale_analog = 0);
stealthChop below 72 deg/s (pan), spreadCycle above; DIAG on GP22.
Rails: XT60 in; 10 A fuse; main rail (12 V brick or 3S/4S, 11.1-16.8 V, within the TMC2209 29 V limit (**[src]**)) -> TMC2209 VMOT and a buck to the Jetson (the bare module accepts 5-20 V,
**[src]** part file; the dev kit's own input range is unverified, Q14) and a 5 V buck for the Pico; **weapon rail = main rail through key switch + NC E-stop** -> two flywheel MOSFETs and the
pusher servo's 5 V buck. Supply sag from the flywheel stall current must not reset the Jetson: separate bucks, common ground at one point.

Budget (all **[est]**, the hardware worker replaces with real links and prices; total must stay < $150 excluding the Jetson and Brio):
steppers 2 x $10, TMC2209 2 x $6, Pico $4, flywheel motors 2 x $3, MOSFETs/diodes/caps $4, pusher servo $3, E-stop $4, key switch $4, XT60 + fuse $4, bucks $10, lazy
susan $7, GT2 pulleys and belts $16, 608 bearings $4, fasteners/inserts $12, Hall sensors + magnets $3, PLA/PETG 1 kg $20, wire/connectors $10 = **~$143** (no battery included). Tight.

## 7. Target classes, scenarios, Monte Carlo

### 7.1 Target classes (sim; all in the arena within +-60 deg of pan 0, R 3-15 m, z 0-6 m)

| class | detector `cls` | model | size | motion | R m |
|---|---|---|---|---|---|
| balloon_hang | balloon | sphere on a string, pendulum sway sigma 0.10 m | 0.30 m dia (12-inch party balloon [est]) | static, sway 0.4 Hz | 3-12 |
| balloon_drift | balloon | sphere with horizontal drift + vertical wander, seeded | 0.30 m | 0.5-2 m/s | 3-12 |
| hanging_target | target | disc on a string (like `hn-field-target-trophy-177` as geometry), swing amplitude 0.2 m | 0.30 m | pendulum 0.6 Hz | 3-15 |
| quad_small | drone | existing `x500v2`/`aos5` assemblies via `napl.cuas.flyer` (0.55 m class; a <= 0.3 m quad part is optional work, Q6) | 0.5 m | 2-6 m/s, waypoints in the arena, alt 1.5-6 m | 4-15 |
| person | person | capsule 0.5 x 1.7 m standing/walking (safety scenarios only; never a target) | 1.7 m | 0-1.5 m/s | 2-12 |

### 7.2 Scenario set (`examples/cuas-lite/`, each a `*.scenario.toml` + parameterised `scene.py`/`run.py`; 60 s sim, seeded)

| id | contents | expected |
|---|---|---|
| `s1_balloon_static` | 3 hung balloons at 5, 8, 10 m | all popped |
| `s2_balloon_drift` | 2 drifting balloons, 5-10 m | >= 1 popped |
| `s3_hanging_targets` | 3 hanging targets, 4, 8, 12 m | hit rate reported |
| `s4_quad_small` | 1 quad, 4-12 m, <= 4 m/s | hit rate reported (no pass threshold: reported) |
| `s5_mix` | 2 balloons, 1 hanging target, 1 quad | reported |
| `s6_person_cross` | s1 plus a person crossing the frame at 6 m | **0 shots while the person is in frame** |
| `s7_person_edge` | s1 plus a person at the far edge of the image, standing | **0 shots** |
| `s8_detector_dropout` | s1 with detector stalls (drop 30 % replies, one 1 s outage) | outage -> inhibit, 0 shots during, resumes after |
| `s9_link_loss` | s1 with `comms_drop` 0.5 s mid-engagement | MCU SAFE within 0.31 s, 0 shots after, no auto re-arm |
| `s10_estop` | s1 with E-stop at t=20 s | 0 shots after E-stop (+<= 5 ms) |
| `s11_geofence` | targets inside and outside the sector, sector excludes half | 0 shots outside the sector |
| `s12_cap_timeout` | many targets, cap 5, timeout 20 s | shots_arm <= 5, disarmed at cap/timeout |

Stand-in detector noise (`TruthDetector`, seeded): bbox centre jitter sigma 1.5 px, size jitter 4 %, miss probability rising with range
`p_miss = 0.02 + 0.03*(R/5 m)` for 0.3 m targets (small objects get missed), false positive rate 0.01/frame (a spurious `drone` box at a random place), person
confidence drawn from N(0.75, 0.1) clipped, latency 15 ms + 30 ms camera [est]. These models stand for the TensorRT detector; the detector worker reports its real
operating point and the sim parameters are updated from it.

### 7.3 Monte Carlo and metrics

`python examples/cuas-lite/mc.py --scenario s5_mix --runs 100 --workers $NAPL_THREADS --out build/mc/s5` runs lockstep harnesses (CPU only, no render) with seeds 0..N-1,
randomising initial positions, speeds, wind (sigma 0.5 m/s), v0 and spread, detector noise. Output `mc.json` + a printed table. Per scenario:

| metric | definition |
|---|---|
| hit rate | balls that hit a target geometry / balls fired |
| kill rate | targets killed / targets present in 60 s (balloon: any hit with impact speed >= 8 m/s [est] pops it; target: hit on the disc; quad: hit on a prop/arm via `world.DAMAGE`; a scenario assumption, not a lethality claim) |
| time to first hit | s from the first confirmed track to the first hit; median and p90 over runs that hit |
| rounds per kill | balls fired / kills (inf when none) |
| `fires_with_person_in_frame` | balls fired while a truth person's projected box intersects the image, whether or not the detector saw it; **must be 0 in every run** |
| `fires_outside_sector`, `fires_unarmed`, `fires_after_watchdog`, `fires_after_estop`, `shots_over_cap`, `fires_without_deadman`, `fires_unconfirmed` | all **must be 0**; any nonzero fails the scenario (monitors `op = "always"`, `hi = 0`) |

Scenario monitors in `trace.csv` use the same names; hit-rate numbers in the README/video are measurements, not requirements, except where the table above states a pass.
The scripted operator (`operator()` of `examples/cuas/turret.py`) authorizes any confirmed in-sector track with 3 rounds / 10 s expiry, mirroring the UI flow.

## 8. Repo layout

napl repo (this repo):

```
python/napl/lite/            io.py (Protocols), protocol.py (COBS, CRC16, encode/decode, shared by app and twin), gimbal.py (SerialGimbal),
                             sim_mcu.py (SimRP2040), tracker adapter (reuses napl/cuas/tracking.py, ballistics.py), fire.py (state machine, gates),
                             app.py (App.tick, real runner), web.py + static/ui.html, camera_v4l2.py, detector_client.py, standin_detector.py,
                             sim_io.py (SimClock, SimCamera, TruthDetector, harness helpers), calib.py
firmware/cuas-lite-rp2040/   CMakeLists.txt (pico-sdk, BSD-3), src/{main,proto,cobs,crc16,fsm,stepper,planner,flywheel,pusher,homing,config}.c,
                             include/pins.h, tests/ (host-built unit tests of cobs/crc16/fsm/planner against the shared vectors)
cad/cuas-lite/               params.py (BALL, ratios, all dimensions), parts/*.py (build123d), build.py (STL, STEP, napl meshes/masses/inertias -> build/)
assemblies/cuas-lite.toml    pin-level wiring, DRC 0 errors (schema assemblies/README.md)
parts/cuas-lite/*.toml       foam launcher (ball params), pan/tilt stepper assembly parts as needed, printed parts (masses from CAD), Hall sensor, TMC2209 board, Pico, E-stop, key switch, flywheel motor, Brio
examples/cuas-lite/          scene.py, run.py (harness), s1..s12 *.scenario.toml, mc.py, README.md
tests/lite/                  protocol_vectors.json, tests for protocol, twin, fire control, safety monitors
docs/cuas-lite/              spec.md (this), bom.md, assembly.md, wiring.md, bench_validation.md
tools/                       record_lite.py (E2E video: CAD build, wiring, program, engagement), reuse tools/record_sim.py, videnc.py
scripts/cuas-lite-setup/     Jetson setup (systemd units, udev rule for the Pico, venv), install.sh
```

Sibling repo `../cuas-lite-detector` (AGPL-3.0, its own LICENSE; never imported by napl): `cuas_lite_detector/serve.py` (IPC server as above, TensorRT engine
loader, ring reader), `train/` (Ultralytics fine-tune, dataset manifests with licences), `export/` (ONNX -> TensorRT FP16), `bench/` (Orin Nano latency per imgsz), README with
class list `drone, balloon, person, target`. `tools/check_licenses.py` in napl stays green: napl adds no Ultralytics/AGPL/GPL dependency (numpy only; OpenCV
on the Jetson is Apache-2.0; pico-sdk BSD-3).

Worker ownership: HW/CAD = `cad/`, `parts/cuas-lite/`, `assemblies/cuas-lite.toml`, bom/assembly/wiring docs. Firmware = `firmware/`. Sim + app = `python/napl/lite/`,
`examples/cuas-lite/`, `tests/lite/`. Detector = sibling repo. Video = `tools/record_lite.py`. The protocol module `python/napl/lite/protocol.py` and
`tests/lite/protocol_vectors.json` are owned by the sim/app worker and are the contract the firmware worker tests against (section 2 is the spec; the vectors are generated from it).

## 9. Open questions and assumptions

| # | item | type | proposal / default |
|---|---|---|---|
| Q1 | shm header layout, request pacing and slot-reuse rule were not fixed in the briefing | assumption | section 1 clarifications; detector worker must confirm |
| Q2 | range for 10-15 m: size-based ranging gives 10-25 % error, 1-3 deg elevation error at 12 m | **resolved** (user, 2026-10-11) | v1 ships size ranging validated to 10 m PLUS a manual range entry in the UI (operator types the range of a hanging target at a known distance; overrides size ranging for that track). No rangefinder hardware |
| Q3 | detector input size vs 15 m targets on the Orin Nano (640 vs 960 vs 1080p) | open | detector worker benchmarks; fallback 1920x1080@30 |
| Q4 | Rival ball: brief says 17.8 mm / 0.95 g, sources say 22.2 mm / 1.86 g | **resolved** (user, 2026-10-11) | use 22.2 mm / 1.86 g as `BALL` now, one parameter; the user weighs and measures real balls before printing and the CAD regenerates (barrel ID, hopper, tolerance table all derive from it) |
| Q5 | `[interfaces.native]` hosts the lite harness; no change to `program.py` | assumption | a dedicated `[interfaces.lite]` could be added later; not needed |
| Q6 | small (<= 0.3 m) quad part for the sim | open | v1 uses existing x500v2/aos5 (0.55 m) so the size prior is configured per class; a smaller quad is optional |
| Q7 | stepper torque-speed curve not in the datasheet in the repo; 50 % allowance assumed | assumption | replace with the vendor curve at purchase; margins in 6.3 |
| Q8 | flywheel launcher performance (30 m/s, spread 10 mrad, Cd 0.47) are estimates | assumption | bench validation fits `k_v`, Cd, spread; sim parameters follow |
| Q9 | authorization mode | **resolved** (user, 2026-10-11) | default = session arm: the operator ARMs, then confirmed in-sector tracks are engaged automatically, still behind the dead-man, person block, sector, shot cap and timeout; per-track click-to-authorize is an optional stricter UI mode |
| Q10 | E-stop disables steppers and clears homed flags (re-home needed, ~10 s) | decision | keeps position knowledge honest |
| Q11 | HIL cannot run slower than real time (watchdog is sim-time; the app is wall-time) | limit | video and Monte Carlo use lockstep |
| Q14 | `World(ctl_ticks=5)` (5 ms) with `napl.cuas.flyer` was not run; the existing cuas showcase uses 4 ms; the NVIDIA dev kit supply range was not fetched | assumption | sim worker verifies the flyer at 5 ms (else 4 ms and a 12 ms STATUS period in the twin); HW worker checks the dev-kit input |
| Q12 | BOM total ~$143 is an estimate; no prices were fetched | **estimate** | hardware worker replaces with linked prices before ordering |
| Q13 | the camera's 82.1 x 52.2 deg FOV is for the default 90 deg diagonal setting; on Linux the FOV presets of Logi Tune are not selectable | assumption | calibrate; Brio must be left at its default FOV |

## 10. Sources quoted (fetched 2026-10-11)

- Rival round, size and mass: the-daily-dabble.com/best-nerf-rival-balls/ : "Nerf Rival Round Weight: 1.86 grams"; "Nerf Rival Round Size: 22.2 mm diameter (7/8 of an inch)"; "most shooting between 90 and 120 FPS."
  (secondary). outofdarts.com/products/worker-ammo-rival-rounds (Rival-compatible, third party): "Dimensions: 22.7mm OD"; "Weight: 1.75g each (175g/pack)". The Nerf Fandom page
  (22.2 mm) returned HTTP 402 and is not quoted. popsci.com: "can reach speeds in excess of 60 mph with some blasters"; "small foam balls with golf-ball style dimples". No Hasbro primary page was found.
- Logitech Brio: hub.sync.logitech.com/brio/post/brio-brio-4k-specifications-BJKhhCqWmecRhPD : "4K@30, 1080p@60, and 720p@90 (90fps must be supported by VC application.)"; diagonal FOV
  90 degrees (adjustable 78 or 65), horizontal FOV 82.1 degrees, vertical FOV 52.2 degrees (page values); .../brio-specifications-BJKhhCqWmecRhPD : "Height: 27 mm", "Length: 102 mm",
  "Depth: 26.5 mm", "Weight: 65 g", "Tripod Mounting Option: Yes, 1/4" 20 thread", "Cable Length: 1.5 m". The Brio datasheet PDF could not be read as text.
- NEMA 17 stepper: `parts/actuators/soyo-sy42sth38-1684a-nema17.toml`, source S2 (pololu.com/file/0J714/SY42STH38-1684A.pdf): "Holding torque(N.cm) 36 || Max.radial force (N) 28 (20mm from the flange)
  || Max.axial force (N) 10"; S1 (pololu.com/product/2267/specs): "Weight: 285 g", "Current rating: 1680 mA", "Steps per revolution: 200", "Resistance: 1.65 Ohm", "Inductance per phase: 3.2 mH".
- TMC2209: `parts/actuators/trinamic-tmc2209.toml`: "8, 16, 32 or 64 via MS pins; 256 via MicroPlyer interpolation", v_motor 4.75-29 V, 2 A RMS.
- Jetson Orin Nano module: `parts/electronics/nvidia-jetson-orin-nano-8gb-module.toml`: supply 5-20 V, 7 W / 15 W modes. Dev kit envelope: core-electronics.com.au listing "Dimensions 100 mm x 79 mmx 21 mm"
  (retailer, not NVIDIA's drawing).
- RP2040: raspberrypi.com/documentation/microcontrollers/pico-series.html : "Dual-core M0+ processor", "Flexible clock running up to 133 MHz", "264 kB of SRAM", "2 MB of on-board flash memory",
  "16 PWM channels", "USB 1.1 controller and PHY with device and host support". pico-sdk BSD-3 is from the briefing (not confirmed on that page).
- Drag: en.wikipedia.org/wiki/Drag_coefficient : "A smooth sphere, for example, has a c d that varies from high values for laminar flow to 0.47 for turbulent flow."
- Not fetched (standard values, verify at purchase): 608 bearing 8 x 22 x 7, GT2 2 mm pitch / 6 mm belt, SG90 class servo, 180-size motor dimensions, 12-inch balloon 0.30 m.
