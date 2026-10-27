# cuas2 engineering spec (v2, 2026-10-26)

From-scratch redesign of the cuas-lite counter-UAS turret around a Nerf-Rival-style launcher: a custom **dual flywheel (spinwheel) launcher**, a **single-pocket gate that
meters exactly one ball per FIRE**, and a **modular** build (six mechanical modules, one keyed connector each). cuas-lite stays in the repo untouched; all cuas2 code lives in new
paths (section 2.4). Three workers (MECH, MODEL, FW) build in parallel from this document without talking to each other; section 12 gives file ownership and failable checks.

Tags: **[src]** fetched and quoted (URL in section 13); **[est]** estimate (to be replaced by bench data, section 10); **[der]** derived here (arithmetic shown). A number
with no tag is a decision. Everything the user must decide is in section 14. v1 means `docs/cuas-lite/spec.md`.

Hard rules carried over unchanged: engage drones/balloons/hanging targets only; a person in frame blocks fire; a human ARMs a session; geofenced sector; dead-man + link
timeout; hardware E-stop + arm key cut the weapon rail independent of any software; threat models are generic kinematic test targets (no real-weapon content). No GPL
code is linked into napl (section 4.5).

## 0. What changes against v1, and why (read first)

| #   | v1 (cuas-lite)                                                                                                                                          | cuas2                                                                                                                                      | reason                                                                                                                                                         |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | 2 brushed 180 motors on one open-loop PWM, no tach, 35 mm rubber wheels                                                                                 | 2 brushless 2207 + 2 ESCs on bidirectional DShot (eRPM telemetry), 40 mm wheels, closed-loop rpm per wheel                                 | "fire only when both wheels are in window" needs a measured speed; backspin needs matched/controlled wheels (a 5 % mismatch is 0.1-0.2 m at 10 m [est, R1 s1]) |
| 2   | burst 1-3 per FIRE, cap 10                                                                                                                              | **one ball per FIRE**, `n` fixed 1                                                                                                         | user requirement; also removes the burst-in-flight abort logic                                                                                                 |
| 3   | hopper on the pan group, balls through a hollow trunnion + 90 deg turn + 76 mm unmeasured level run                                                     | hopper **on the tilt head**, top-fed at the rear, 24 balls, no hollow trunnion, no elbow                                                   | removes the unmeasured run and the 6807ZZ; gravity feed proven for tilt -15..+70 (section 5.1)                                                                 |
| 4   | tilt static torque 0.154-0.156 N*m, "12 % margin" (against a CAD budget of 0.177 N*m; against the motor it is 27 % of holding torque [der from v1 6.3]) | head balanced about the tilt axis: static torque <= 0.06 N*m, CG offset <= 8 mm                                                            | thin CAD margin; every head addition re-opened it                                                                                                              |
| 5   | pusher stroke 44 mm moves a ball from a chute                                                                                                           | quarter-turn **single-pocket plug-valve gate** upstream, short pusher (32 mm)                                                              | exactly one ball by geometry, mechanically; cycle 0.365 s [der]                                                                                                |
| 6   | safety: key + NC E-stop -> relay -> weapon rail                                                                                                         | same, plus a **hardware shot-interval limiter** on the pusher servo supply and a **hardware link-alive supervisor** (MCU must toggle KICK) | "hung or runaway MCU cannot cycle faster than the min interval"                                                                                                |
| 7   | STATUS 29 B                                                                                                                                             | STATUS 45 B (tail appended), proto 1.1, `HELLO_ACK.fw_major = 2`                                                                           | v1 app/twin tooling keeps working (section 6.1)                                                                                                                |

## 1. Requirements and safety invariants

### 1.1 Functional requirements

| id  | requirement                                                                                                                                                                  | check                                         |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------- |
| R1  | muzzle speed 25-35 m/s nominal 30 m/s [est], **hard cap 38 m/s**                                                                                                             | chrono (10.2), firmware cap (4.3), test F-CAP |
| R2  | exactly one ball leaves per accepted FIRE; zero balls leave without an accepted FIRE                                                                                         | fault table 1.3, tests F-ONE, F-NOFIRE        |
| R3  | command-to-ready interval (FIRE accept to next FIRE accept) <= 0.50 s, nominal 0.466 s [der: MECH C-CYCLE, 47.2 mm stroke]; hard minimum 0.47 s (relaxed from 0.40 s, 14 Q2) | timeline test T-CYCLE                         |
| R4  | hopper >= 24 balls; refill without tools in < 20 s                                                                                                                           | CAD check, assembly doc                       |
| R5  | gimbal as v1: pan +-170 deg 240 deg/s, tilt -15..+70 deg 150 deg/s; same step/dir drivers                                                                                    | v1 numbers kept                               |
| R6  | head mass <= 0.60 kg, static tilt torque <= 0.06 N*m over -15..+70, CG offset from tilt axis <= 8 mm                                                                         | CAD check C-BAL                               |
| R7  | every module removable with <= 4 screws + 1-2 connectors; defined datum and bolt pattern (section 3)                                                                         | CAD check C-DOCK                              |
| R8  | cost target section 9: <= $190 hardware + filament excl. Jetson, Brio, rounds (v1 $151.93)                                                                                   | bom.md sum                                    |
| R9  | per-shot data logged: wheel rpm at feed, rpm dip, chrono speed, gate/feed phase times                                                                                        | STATUS/EVENT (6.2, 6.3)                       |

### 1.2 Safety invariants (all must hold in firmware, twin and CAD; each has a test in section 12)

Kept from v1 (names as v1 section 3.2): **I1** person in frame -> `FIRE_INHIBIT` (app) and no FIRE (MCU refuses while inhibit set). **I2** operator ARM per session; new `HELLO.session`
disarms. **I3** only confirmed engageable tracks. **I4** geofence sector twice: app clamps `AIM`, MCU holds fire sector (cfg 17, 20-22), unset after every boot (NO_SECTOR), checked every
1 ms tick **until the ball is committed** (5.4). **I5** per-arm shot cap and arm timeout (hard values cfg 10, 11). **I6** dead-man fresh. **I7** key on and E-stop released; **hardware**:
key + NC E-stop in series feed the weapon-rail relay coil, open = ESCs and servo supplies dead with the MCU off or hostile. **I8** 300 ms heartbeat watchdog -> SAFE. Also kept: `fire_id`
idempotence (last 8 ids), faults latch and clear only through `CLEAR_FAULT` in SAFE with the E-stop released.

New in cuas2:

- **I9 one ball per FIRE, mechanical.** The pocket is a bore whose length admits one ball (CAD check C-POCKET: centres of two balls cannot fit); while the pocket is at the bore the valve body
  blocks the hopper outlet; while the pocket is at the hopper the pusher path is closed by the valve wall. One pusher stroke can therefore move at most the one staged ball.
- **I9b one ball per FIRE, firmware.** FIRE is accepted only from state `READY` (section 5.5), consumes the staged ball, and a second FIRE is refused (`ACK BUSY`) until the cycle returns to READY and
  `min_shot_interval_ms` (cfg 15, firmware clamp >= 470) has passed since the last FEED start. `n != 1` -> `BAD_ARG`.
- **I10 independent cross-check of what actually left.** The muzzle beam counts balls that physically exit. Muzzle events without an accepted feed, more than one muzzle event per feed, more than
  `shot_cap`, or two muzzle events closer than 300 ms -> FAULT `unexpected_exit`/`shot_overcount`, which stops the supervisor KICK and opens the relay (rail cut in <= 150 ms).
- **I11 breech/muzzle lockouts.** No feed unless: breech beam blocked (a staged ball), gate at BORE, pusher home, muzzle beam **clear**, no jam latch. After a feed: breech must clear within 150 ms
  and the muzzle beam must see the exit within 250 ms of the feed, else FAULT `no_exit` (jam) requiring operator `CLEAR_FAULT` (reset).
- **I12 speed verified closed-loop before feed.** Feed only if both wheels have measured rpm (telemetry age <= 20 ms) within `rpm_window_pm` (default 10 permille = 1.0 %) of the setpoint
  continuously for `settle_ms` (100) and the commanded differential matches. Lost telemetry > `telemetry_loss_ms` (50) -> wheels to 0 and FAULT.
- **I13 velocity cap.** `rpm_cap = floor(v_cap * 60 / (pi * wheel_d))` computed in firmware from cfg 23, 24 (`v_cap_dmps <= 380` clamped at CFG_SET; maintenance keys, SAFE only). Since
  `v_ball <= (v_top + v_bot)/2 <= max wheel surface speed` for any slip (k_slip <= 1, energy cannot increase), a wheel held <= rpm_cap cannot launch above 38 m/s **independent of calibration** [der].
  Measured rpm > 1.05 rpm_cap for 20 ms -> FAULT `overspeed`, throttle 0, KICK stops (relay opens).
- **I14 hardware minimum interval.** A 74HC123-class edge-triggered one-shot pair powers the pusher servo (5 V) for `T_on = 254 ms` after a rising edge on FEED_REQ and ignores further edges for
  `T_hold = 338 ms +-15 %` from that edge (as built, wiring.md: 120 k x 4.7 uF and 75 k x 10 uF; worst case 338 x 1.15 = 389 ms < the firmware clamp of 470 ms [der]), so the hardware is a backstop that never rejects a healthy shot: firmware default
  `min_shot_interval_ms = 500`, clamp >= 470 (the 466 ms geometric cycle, MECH C-CYCLE). The best case of the hardware (338 x 0.85 = 287 ms) is the fastest a runaway MCU could cycle it, 3.5 strokes/s at most.
  The whole stroke (118 ms out, `pusher_dwell_ms` 120 default, 118 ms back) has to fit the 254 ms power window, hence cfg 14 <= 130: a rod still out when the window closes is `pusher_stuck`.
  A held-high FEED_REQ gives one stroke, not many (edge-triggered); a stuck-on servo output holds the rod at one position and moves nothing.
- **I15 link-alive supervisor.** A 74HC123-class retriggerable one-shot (150 ms) must be re-triggered by the MCU toggling KICK (GP28) at 100 Hz from the 1 kHz tick; its output is the third series
  contact of the relay coil (key -> E-stop -> supervisor). The tick toggles KICK **only while all of**: main loop alive, `fw_assert_ok` (shots_arm <= cap, interval ok, muzzle count <= feed count,
  state valid), no fault latched. A hung MCU, a failed assertion or a fault opens the rail within 150 ms.
- **I16 power-up.** All weapon outputs default off: ESC power behind the relay; DShot lines idle low with 10 k pull-downs (ESC sees no signal = no throttle); pusher servo supply behind the one-shot
  (off at power-up); gate servo signal pull-down; FEED_REQ and KICK pull-down.

### 1.3 Fault table: each single fault -> outcome (none gives runaway fire)

"Runaway" = a ball leaving without an accepted FIRE, or more than one ball per accepted FIRE, or a sustained rate above 1 per 0.4 s.

| #   | single fault                                                                                     | immediate outcome                                                                                                                                                                                                                                                                      | final state / who stops it                                                                                                                                                                                                                                                      |
| --- | ------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | app crash / host link lost / USB unplugged                                                       | heartbeat > 300 ms -> SAFE, wheels to 0, pusher home                                                                                                                                                                                                                                   | FIRE impossible; no auto re-arm (I8)                                                                                                                                                                                                                                            |
| 2   | RP2040 hang                                                                                      | KICK stops -> supervisor opens relay <= 150 ms; DShot idles low (ESC no signal); pusher supply dies after <= 250 ms                                                                                                                                                                    | rail dead; hardware watchdog (8 s) resets into SAFE                                                                                                                                                                                                                             |
| 3   | firmware logic bug repeatedly requesting FEED_REQ (MCU alive, KICK running)                      | I14 allows at most 1 stroke per 280-380 ms; muzzle-beam cross-check (I10) detects an exit with no accepted feed or an interval < 300 ms -> FAULT, KICK stops, rail opens                                                                                                               | first extra ball may leave; then rail cut. **Residual**: a bug that _also_ corrupts the cross-check (common mode) is bounded only by the 24-ball hopper at 2.5/s = 9.6 s and the operator E-stop (stated, see 14 Q6)                                                            |
| 4   | FEED_REQ line stuck high                                                                         | hardware edge-triggered: one stroke at most, then no more                                                                                                                                                                                                                              | fire impossible afterwards (stroke count from muzzle beam; MCU sees pusher not moving -> `pusher_stuck` fault)                                                                                                                                                                  |
| 5   | FEED_REQ line stuck low                                                                          | no stroke                                                                                                                                                                                                                                                                              | `pusher_stuck` fault at the next FIRE, shot refused                                                                                                                                                                                                                             |
| 6   | pusher servo signal stuck extended                                                               | rod stays in the bore: valve cannot turn (wall) and no further ball is staged                                                                                                                                                                                                          | one ball at most; fault `pusher_stuck` when PUSH_HOME never returns                                                                                                                                                                                                             |
| 7   | pusher servo stuck retracted / dead                                                              | no stroke: PUSH_HOME never drops, so FAULT `pusher_stuck` 40 ms after the feed (`FDR_PUSH_LEAVE_MS`); `no_exit` at 250 ms stays the backstop                                                                                                                                           | staged ball stays; operator reset                                                                                                                                                                                                                                               |
| 8   | gate servo runaway / stuck-on                                                                    | 60 deg swing bounded by hard stops, 180-deg servo cannot rotate continuously; a ball can reach the bore only via LOAD -> BORE; cycling alone feeds nothing without a pusher stroke                                                                                                     | no shot; gate faults `gate_stuck` if LOAD hall not seen (test row 8: its end state at a hard stop, the same as row 9)                                                                                                                                                           |
| 9   | gate stuck at BORE (jam)                                                                         | hopper outlet blocked by the valve body, no refill                                                                                                                                                                                                                                     | `hopper_empty`/`gate_stuck`, no fire                                                                                                                                                                                                                                            |
| 10  | gate stuck at LOAD                                                                               | breech beam stays clear, no staged ball                                                                                                                                                                                                                                                | FIRE refused (NO_BALL)                                                                                                                                                                                                                                                          |
| 11  | two balls in pocket (bad part dimension)                                                         | prevented by pocket length (C-POCKET); if a bad print admits two: breech beam sees 2-ball occupancy as a longer block (> 1.3x expected)                                                                                                                                                | `breech_double` fault, no feed                                                                                                                                                                                                                                                  |
| 12  | breech beam fails (stuck clear)                                                                  | FIRE refused (no staged ball)                                                                                                                                                                                                                                                          | no fire                                                                                                                                                                                                                                                                         |
| 13  | breech beam fails (stuck blocked)                                                                | feed allowed, but exit must follow: muzzle beam sees nothing -> `no_exit` fault after the first stroke                                                                                                                                                                                 | at most one stroke, then lockout                                                                                                                                                                                                                                                |
| 14  | muzzle beam fails (stuck clear)                                                                  | exit never confirmed -> `no_exit` after each feed                                                                                                                                                                                                                                      | lockout after the first feed                                                                                                                                                                                                                                                    |
| 15  | muzzle beam fails (stuck blocked)                                                                | FIRE refused (muzzle not clear, I11)                                                                                                                                                                                                                                                   | no fire                                                                                                                                                                                                                                                                         |
| 16  | one ESC fails to spin                                                                            | measured rpm out of window                                                                                                                                                                                                                                                             | FIRE refused, `wheel_not_ready` after timeout 1.5 s                                                                                                                                                                                                                             |
| 17  | one ESC runs away at full throttle (ESC firmware/signal fault)                                   | measured rpm > 1.05 rpm_cap -> FAULT `overspeed`, DShot 0, KICK stops, relay opens <= 150 ms; feed gate already refuses (rpm out of window). Peak transient: no-load rpm of 1800 KV x 12.6 V = 22.7k rpm (47 m/s surface) for <= 150 ms with no ball present                           | no ball staged or fed while out of window                                                                                                                                                                                                                                       |
| 18  | telemetry wire broken (rpm unknown)                                                              | `telemetry_loss_ms` 50 -> wheels 0, FAULT                                                                                                                                                                                                                                              | no fire                                                                                                                                                                                                                                                                         |
| 19  | DShot line stuck high or noisy                                                                   | ESC rejects invalid frames (CRC) and, when no valid frames arrive, disarms [est, ESC behaviour] ; telemetry mismatch vs commanded -> out of window                                                                                                                                     | no fire; overspeed path as 17 (test row 19: both ESCs ignore the frames, no spin, `wheel_not_ready`)                                                                                                                                                                            |
| 20  | hardware limiter MOSFET shorted on (pusher supply always on)                                     | servo still obeys MCU PWM; MCU enforces the interval (firmware clamp 470); I14 backstop is lost but I10 cross-check and I15 stay                                                                                                                                                       | single fault does not enable a stroke by itself                                                                                                                                                                                                                                 |
| 21  | supervisor one-shot stuck high                                                                   | I15 lost, relay still behind key + E-stop; MCU watchdog and heartbeat still active                                                                                                                                                                                                     | operator E-stop remains                                                                                                                                                                                                                                                         |
| 22  | relay contacts weld                                                                              | rail stays on after key/E-stop open                                                                                                                                                                                                                                                    | **residual hardware fault**: ESCs powered but the MCU still refuses FEED in SAFE; no ball staged -> no fire; documented; relay is a 40 A automotive part, derated (peak 12.5 A [der])                                                                                           |
| 23  | E-stop pressed / key off                                                                         | coil opens: ESCs unpowered, pusher supply dead, MCU sees edge -> SAFE                                                                                                                                                                                                                  | any stroke in progress is dead on the servo; a ball already past the commit point (5.4) still leaves the barrel (it is in the nip)                                                                                                                                              |
| 24  | detector stale / person detected                                                                 | app sets `FIRE_INHIBIT` in every heartbeat; MCU refuses FIRE (ACK INHIBIT) and aborts a stroke before commit                                                                                                                                                                           | no fire                                                                                                                                                                                                                                                                         |
| 25  | barrel leaves the fire sector during the stroke                                                  | MCU 1 ms check -> abort before commit (retract, EVENT 9/11), after commit a margin (5.4) covers the final ms                                                                                                                                                                           | ARMED, no flywheel request                                                                                                                                                                                                                                                      |
| 26  | stepper stall / driver fault                                                                     | DIAG -> `driver_diag` fault -> FAULT, flywheels 0, KICK stops                                                                                                                                                                                                                          | as v1                                                                                                                                                                                                                                                                           |
| 27  | flash config corrupt                                                                             | `cfg_invalid`, defaults, `v_cap` defaults to 380 dmps and wheel_d 40000 um; sector unset so ARM refused                                                                                                                                                                                | no fire until reconfigured                                                                                                                                                                                                                                                      |
| 28  | hopper empty                                                                                     | repeated empty breech after 3 refill cycles -> EVENT 12 EMPTY, state READY refused                                                                                                                                                                                                     | no fire                                                                                                                                                                                                                                                                         |
| 29  | wheel gap or ball diameter out of tolerance                                                      | exit speed outside expected band; the cap in I13 is still absolute                                                                                                                                                                                                                     | cap holds                                                                                                                                                                                                                                                                       |
| 30  | hardware shot counter trips (N-th muzzle-beam pulse since key-on; default N = 10, wiring.md 3.4) | the counter clamps the gate of the supervisor's weapon MOSFET: ESCs and servos lose power; relay coil, key and E-stop stay closed, so GP10 sees nothing and there is no spare GPIO. Firmware sees both ESC telemetry streams drop together (within 5 ms) with key on and E-stop closed | ordinary `telemetry_lost` fault (weapon stopped, KICK off); if the per-key-session beam-pulse count has reached cfg 41 it is also labelled EVENT 16 + fault bit b15 `hw_counter` [inference, not a sensed trip]. Reset only by a key power cycle; the label needs `CLEAR_FAULT` |

## 2. System architecture and repo layout

### 2.1 Process layout

Unchanged from v1 section 1 (capture, detector over shm + socket, serial rx, web, control loop, systemd units). New content: `STATUS` carries launcher data (6.2), the control loop calls the
fire control of section 8 instead of v1's pointing-tolerance trigger, sensor modules other than the main camera (rangefinder, second camera) connect to the **Jetson over USB**, never to the
RP2040 (the safety MCU has no sensor drivers).

### 2.2 Module graph

```
 [BASE/PAN M1] ---- [POWER/SAFETY BOX M6] ---- Jetson (USB: RP2040, Brio, optional 2nd camera, TF-Luna via USB-TTL)
      | pan axis + harness H1 (weapon 12 V XT30, 16-pin signals)
 [TILT HEAD M2] -- dock D1 --> [LAUNCHER CARTRIDGE M3] -- spigot F1 --> [FEEDER M4: hopper+gate+pusher]
      +-- dock D2 --> [SENSOR HEAD M5: Brio now; 2nd camera / TF-Luna / thermal optional]
```

### 2.3 Decisions on reuse (minimise duplication, keep cuas-lite untouched)

| area                                                                    | decision                                                                                                                                                                                                                                                                                                  | justification                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| framing, CRC, COBS, `Msg`/`msg` dataclass machinery, `Decoder`, `Frame` | **import** from `napl.lite.protocol`                                                                                                                                                                                                                                                                      | identical bytes; zero fork                                                                                                                                                                                                                                                                                                                      |
| messages and STATUS                                                     | new `napl.cuas2.protocol`: re-exports the unchanged messages, defines `Status2`, `Fire` (n fixed 1), `Flywheel` (+ tail), constants                                                                                                                                                                       | tail-append only (proto 1.1), section 6                                                                                                                                                                                                                                                                                                         |
| firmware                                                                | **new tree `firmware/cuas2-rp2040/`** that compiles lite's `cobs.c crc16.c planner.c homing.c src/pico/{tmc2209,hal_pico}.c` **by relative path** against **cuas2's own `include/fw_core.h`** (a superset of the lite one, same field and enum names), and forks `fsm.c`, `proto.c`, `config.c`, `main.c` | the weapon section of `fsm.c` (pusher, flywheel, fire gates) is inline with the safety FSM, so no clean reuse without editing lite; sharing framing/planner/homing/TMC drivers avoids ~350 duplicated lines. A v2 target inside the lite tree would edit files whose hashes (`tests/lite/protocol_vectors.json`, replay hashes) must stay fixed |
| safety FSM                                                              | forked from `fsm.c` @ 8f173e6; the diff is limited to the weapon sections; `tools/cuas2_fsm_drift.sh` prints a diff stat of the non-weapon functions so drift is visible                                                                                                                                  | FSM must be edited (new states, gates)                                                                                                                                                                                                                                                                                                          |
| python plant, twin                                                      | `napl.cuas2.sim_mcu.Sim2RP2040(napl.lite.sim_mcu.SimRP2040)` overrides the weapon methods (`_flywheel_cmd`, `_flywheel`, `_fire`, `_fire_check`, `_spun`, `_pusher`, `_burst_done`, `_status`, `_write_plant`; names from lite) and reuses axes, homing, watchdog, ARM, CFG handling                      | reuse ~800 of 1017 lines                                                                                                                                                                                                                                                                                                                        |
| app, web UI, `io.py` Protocols, `SerialGimbal`                          | reuse by import / subclass in `napl.cuas2` (`Gimbal2(SerialGimbal)` adds `launcher_status`); web UI static page reused, status JSON gains launcher fields                                                                                                                                                 | v1 protocols keep working; integration (not in this wave)                                                                                                                                                                                                                                                                                       |
| kernel `launcher` device (`src/mujoco/devices/launcher.cpp`)            | **additive attrs** with default 0: `spin_rps`, `cl_k`, `spin_tau`, `dir_offset_x`, `dir_offset_y` (section 7.4). Default 0 -> bit-identical output, so cuas-lite replay hashes stay                                                                                                                       | Magnus is the only physics missing; the alternative (Python-side flight) cannot hit the kernel's hit test                                                                                                                                                                                                                                       |
| CAD                                                                     | new `cad/cuas2/` (build123d), shares nothing but copies the constants block; test asserts `BALL` equals `cad/cuas-lite/params.py`                                                                                                                                                                         | the lite folder name has a hyphen (not importable)                                                                                                                                                                                                                                                                                              |
| docs                                                                    | `docs/cuas2/{spec,assembly,bom,wiring,bench_validation}.md`                                                                                                                                                                                                                                               |                                                                                                                                                                                                                                                                                                                                                 |

### 2.4 Repo layout (new paths only)

```
cad/cuas2/                params.py (BALL, wheels, gap, dock tables), interfaces.py (docks as data), modules/{base_pan,tilt_head,launcher_cartridge,feeder,sensor_head,power_box}.py,
                          checks.py, build.py            (same CLI as lite: --out --only --no-checks --no-preview)
parts/cuas2/*.toml        brushless-2207, esc-30a-bluejay, sg92r (alias of lite part if identical), ir-beam, 74hc123, relay-40a, ... ; reuse parts/cuas-lite where identical (NEMA17, TMC2209, Pico, Hall, E-stop, key)
assemblies/cuas2.toml     pin-level wiring, DRC 0 errors
firmware/cuas2-rp2040/    CMakeLists.txt, include/{fw_core.h,pins.h}, src/{fsm.c,proto.c,config.c,launcher.c,feeder.c,dshot_pio.c(pico/),main.c}, host/ (x86 build), tests/
python/napl/cuas2/        __init__, protocol.py, calib.py, launcher.py, ballistics.py, fsm.py (FW twin of launcher+feeder), sim_mcu.py, sim_io.py, gimbal.py
python/napl/track/        TRACK worker (separate wave): trackers, fusion, discrimination, firectl.py (section 8)
tests/cuas2/              protocol_vectors2.json, fsm_vectors.json, test_*.py
docs/cuas2/               this spec, bom.md, assembly.md, wiring.md, bench_validation.md
src/mujoco/devices/launcher.cpp   (additive attrs only, owned by MODEL)
```

## 3. Module split and mechanical interfaces

General rules: M3 hardware + brass heat-set inserts; every module has datum A (mating plane), B (rail side), C (stop); dovetails 60 deg, 0.3 mm clearance per side [est, FDM], print
orientation chosen so the dovetail faces are layer-parallel; every module prints on the 220 mm bed (max 200 x 200). One connector per module, polarised (keyed JST-XH or XT30; a male/female
pairing that cannot be reversed), harness length +30 mm service loop.

| module                    | contents                                                                                                                                                                                                    | mechanical interface                                                                                                                                                                 | connector                                                                                                                 | mass target          |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------- | -------------------- |
| **M1 BASE/PAN**           | base plate 200 x 200, lazy susan 6 in (reuse v1 pan, ratio 5:1), pan NEMA17 + 20T/100T GT2, pan hall, hard stops, Jetson tray                                                                               | top: rotor plate ring 6 x M4 on `LS_BOLT_R` (measure on the delivered bearing, v1 open item), cable loop +-178 deg                                                                   | H1 pass-through (below)                                                                                                   | 1.5 kg [est]         |
| **M2 TILT HEAD**          | two yoke cheeks, **two 608 trunnion bearings** (no hollow trunnion), tilt NEMA17 4:1, tilt hall, hard stops -18/+73, cable chain                                                                            | **D1** launcher dock, **D2** sensor dock; yoke interface to M1: 4 x M4 on a 50 x 50 pattern                                                                                          | H1 head side                                                                                                              | 0.20 kg + payload    |
| **M3 LAUNCHER CARTRIDGE** | cage, 2 brushless motors + 40 mm wheels, 2 ESCs (rear), barrel 70 mm with muzzle beam, gap adjuster, breech spigot                                                                                          | **D1**: 60 deg dovetail 12 mm wide along the barrel axis, slides in from the rear, 2 x M3 captive clamp screws, rear stop; boresight shim pack 0-1.5 mm under the dovetail (+-3 deg) | C3 = XT30 (weapon 12 V, 2 ESC) + JST-XH 8-pin (DShot x2, beam, GND, 3V3, shield)                                          | 0.22 kg              |
| **M4 FEEDER**             | hopper 24 balls, single-pocket plug-valve gate + SG92R, pusher slider + SG92R, breech beam, gate-home and push-home halls                                                                                   | **F1** breech spigot: 26 mm bore flush with barrel axis, 4 x M3 on a 50 x 40 pattern, O-lip aligns bores to 0.2 mm; sits on the cartridge top-rear                                   | C4 = JST-XH 10-pin (5V_GATE, 5V_PUSH(limited), GND, GATE_SIG, PUSH_SIG, GATE_HOME, PUSH_HOME, BREECH_RX, BREECH_VCC, GND) | 0.10 kg + 45 g balls |
| **M5 SENSOR HEAD**        | Brio clamp (1/4-20 + 2 dowels), bar with M3 slots for an optional second camera (baseline 0.3-0.5 m is a _separate_ printed beam with dovetail), TF-Luna pocket (boresighted to the barrel), thermal pocket | **D2**: 2 x M4 on 40 mm pitch + a 6 mm dowel, +-3 deg boresight shims; also mountable on the base for a fixed second camera                                                          | USB-A/C leads (direct to Jetson), 4-wire JST for the optional TF-Luna on USB-TTL                                          | 0.09 kg              |
| **M6 POWER/SAFETY BOX**   | Pico, 2 x TMC2209, relay, limiter + supervisor board, 5 V buck, 20 A fuse, XT60 in, key/E-stop terminals, 12 V Jetson output                                                                                | 4 x M3 to the base tray, DIN-style 100 x 80 x 40                                                                                                                                     | XT60 in; XT30 weapon out; JST-XH for halls/motors; front panel key/E-stop/LED on screw terminals                          | 0.30 kg              |

**H1 head harness** (M6 -> M2 through the pan axis, one loop): XT30 pair (weapon 12 V, 2 x 16 AWG, peak 12.5 A [der]) + one 16-conductor 26 AWG ribbon-cable (DShot top, DShot bot, GATE_SIG, PUSH_SIG,
GATE_HOME, PUSH_HOME, BREECH_RX, MUZZLE_RX, 5V_LOGIC, 5V_GATE, 5V_PUSH, 3V3, GND x4). Tilt hall and tilt motor wires are part of the same loop (separate 4-pin). USB for the Brio and optional
second camera: separate thin cables on the same loop.

**Tilt torque (the thin 12 % margin).** The v1 number is a CAD budget check, not the motor limit: the motor sees 0.098 N*m of 0.36 N*m holding (27 %, 54 % of the 50 % allowance) [der v1 6.3]. cuas2
treats the CG as a design variable: R6 (CG within 8 mm of the axis, static torque <= 0.06 N*m). Mass table [est, g, x forward / z up in mm from the tilt axis]: motors+wheels 82 (30, 0); cage+barrel 70 (30, 0);
valve+gate servo 30 (-10, 10); pusher+servo 25 (-45, 5); ESC x2 16 (-50, 15); hopper 40 (-5, 45); balls 44.6 (-5, 40); sensor bar+Brio 90 (35, 40); yoke hub/pulley/bearings 80 (0, 0);
wiring 20 (-30, 0) = 497.6 g, CG (9.0, 15.8) mm, static torque 0.062 N*m at -15 deg, 0.044 at 0, -0.008 at 35, -0.057 at 70 [der]. That already beats v1 (0.155) by 2.5x and is the _worst-case start_:
MECH moves the ESCs, battery-free ballast (<= 60 g printed lead-shot pocket at the rear, allowed) and the camera until CG offset <= 8 mm and torque <= 0.06 at all four tilts. Dynamic: 1500 deg/s^2 on
a 0.5 kg head of ~60 mm radius needs 1.6e-3 kg*m^2 x 26 rad/s^2 = 0.04 N*m at the output [der], inside v1's 0.056 bound. A rear-ballast slot, not a spring, is the fallback.

## 4. Launcher

### 4.1 Layout decisions

| item                   | decision                                                                                                                                                                                    | reason                                                                                                                                                        |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| wheel count            | **2**, one above one below the ball (axes parallel to the head y axis, horizontal)                                                                                                          | spin from the speed difference; horizontal axes keep Magnus in the vertical plane, which elevation/lead can compensate; vertical axes curve laterally (R1 s1) |
| wheel diameter / width | **40 mm / 14 mm**                                                                                                                                                                           | 40 mm hits 30 m/s at 17.9k rpm for k_slip 0.8 [der]; small inertia (fast recovery); 50 mm is a cartridge variant if slip is worse than 0.7 (see 4.3)          |
| wheel material         | printed **TPU 90A-95A** one-piece tyre on a PETG hub (baseline); cast **silicone 40A-60A** tyre as variant B; durometers compared on the bench (10.2)                                       | printable in-house, zero BOM cost; R1 starts at 40-60A [est]                                                                                                  |
| gap                    | squeeze 3.0 mm (13.5 % of 22.2 mm) -> wheel-surface gap **19.2 mm**, adjustable **18.2-20.2 mm** (squeeze 2.0-4.0 mm); wheel centre separation = gap + D = 59.2 mm [der]                    | R1 s2 band 11-18 %, 2.5-4.0 mm; v1 used 3.0 mm                                                                                                                |
| gap adjustment         | top wheel fixed to the cage; bottom motor on a slotted plate with a push-pull M3 screw (0.5 mm/turn), locked by 2 x M3; scale marks every 0.25 mm                                           | bench tuning instead of guessing                                                                                                                              |
| motors                 | **2 x brushless 2207, 1800 KV, 3S** (stator 22 x 07 mm, 14 poles = 7 pole pairs)                                                                                                            | section 4.2                                                                                                                                                   |
| ESCs                   | **2 x 30 A BLHeli_S-class (Bluejay-capable) single ESCs**, bidirectional DShot300, one per motor, powered from the weapon rail                                                              | section 4.2, 4.5                                                                                                                                              |
| rpm range              | cap = `38*60/(pi*0.040)` = **18,144 rpm** at D 40 [der]; default `rpm_max` 17,500; idle pre-rev **4,500 rpm** (26 % of 17,500, ~13 % of v0 energy)                                          | idle cuts spin-up to 0.33 s at 40k rpm/s ramp [der]; idle fraction [est]                                                                                      |
| velocity cap           | 38 m/s = 125 fps, firmware-enforced from `wheel_d` (I13), absolute                                                                                                                          | R1 s2: 120-130 fps event caps; 38 m/s is above a 120 fps (36.6 m/s) event cap; set cfg 24 to 365 for such events                                              |
| differential           | bottom wheel faster by `diff_pm` permille of the mean wheel speed; default **15 permille** -> dv = 0.56 m/s at 37.5 m/s surface, backspin 250 rpm, lift/weight ~0.09 [der, CL = 0.75 S est] | knob measured on the bench; sign convention: positive = bottom faster = backspin                                                                              |
| barrel                 | ID 23.5 mm (ball 22.2 + 1.3), 70 mm after the nip; muzzle beam 55 mm after the nip centre                                                                                                   | shorter than v1's 110 mm: a flywheel gains nothing from length; guides the ball only                                                                          |
| feed entry             | ball first touches both wheels with its centre **9.5 mm before** the wheel centre line [der: sqrt((20+11.1)^2 - 29.6^2)]                                                                    | pusher stroke sizing (5.2)                                                                                                                                    |

### 4.2 Motor decision: brushless + DShot with RPM telemetry (not brushed + tach)

Pick **brushless 2207 + bidirectional-DShot ESCs**. Reasons: (1) the requirement "fire only when both wheels are within 1 % of setpoint, measured" needs a tach per wheel; brushless gives it
with **no extra sensor, wire or magnet** (eRPM on the signal wire); brushed needs a hall/IR tach, a magnet or slotted disc per wheel, and 2 MOSFET stages plus flyback parts; (2) brushed 130 cans
have high loaded-rpm droop and failures on 3S and continuous use [src R1 s2: outofdarts Kraken page]; (3) brushless costs about $24 more than a brushed pair with MOSFETs, diodes and tach (34.00 vs 9.8 [est]) and removes the two IRLB8721
stages; (4) the closed loop is what lets us hold dv to < 2 % and tune backspin deliberately. Cost accepted: ESC arm time (RP2040 sends DShot 0 for 0.5 s at boot), a PIO program, and the licence note below.
Fallback kept in the spec, not built: brushed 130 + IR tach (the v1 tach pin GP13 is repurposed, see 6/pins); the cartridge connector C3 is the same so the swap is a cartridge change.

Why 1800 KV: no-load 1800 x 12.6 V = 22.7k rpm [der]; loaded 85-92 % -> 19.3-20.9k; 17.9k needed for 30 m/s at k_slip 0.8 leaves 8-14 % throttle headroom for recovery. 2205-2207 1800-2300 KV on 3S
is the class that R1 estimated (18-20k loaded for 40 mm wheels) [est].

Spin-up and recovery [der, J = 4e-6 kg*m^2 per wheel+rotor, est]: stored energy 6.7 J per wheel at 17.5k rpm; ramp-limit `ramp_rpm_s` 40,000 -> 61 W for both wheels = 6 A at 12 V (limits inrush; 0 -> 17.5k in 0.44 s,
idle -> full in 0.33 s). Per shot: ball KE 0.84 J at 30 m/s; transfer efficiency 0.6 [est] -> 1.4 J from the wheels -> ~930 rpm (5.3 %) dip per wheel, recovered in ~28 ms at 25 W net [der]. With a >= 0.4 s
interval the dip is irrelevant for the next shot; it is logged (EVENT 10) and modelled (7.3).

### 4.3 Calibration (calib.toml)

`~/.config/cuas2/calib.toml`, schema below; every key has a built-in `[est]` default so sim and app run without a file; each key may carry `<key>_src = "est" | "bench"` (provenance printed at app start).

```toml
[ball]            d_mm = 22.2, m_g = 1.86, cd = 0.47, cd_sigma = 0.05, d_sigma_mm = 0.3, m_sigma_g = 0.08       # [est]; cd/mass/d per-ball scatter
[launcher]
wheel_d_mm        = 40.0       # cartridge variant
gap_mm            = 19.2       # measured with feeler stock after assembly
k_slip            = 0.80       # v0 = k_slip * (v_top + v_bot)/2 [est 0.70-0.90]
k_slip_pm_per_krpm = 0.0       # optional linear slope of k_slip with rpm [est 0]
sigma_v_rel       = 0.03       # fractional shot-to-shot speed scatter [est]
sigma_v_floor_mps = 0.5
spread_mrad       = [10.0, 10.0]   # direction scatter per axis (horizontal, vertical) [est, v1]
bias_mrad         = [0.0, 0.0]     # mean direction offset relative to the camera boresight (the v1 boresight tool writes it)
rpm_max           = 17500      # setpoint ceiling; firmware clamps to rpm_cap
rpm_idle          = 4500
diff_pm           = 15         # bottom faster, permille of mean wheel speed
k_spin            = 0.80       # fraction of the wheel surface-speed difference that becomes ball spin [est 0.5-1.0]
cl_k              = 0.75       # Magnus C_L = cl_k * S, S = omega r / v, saturated at cl_max=0.25 [est 0.5-1.0 small-S rule]
spin_tau_s        = 6.0        # spin decay constant [est]
v_cap_mps         = 38.0       # firmware keys 23/24 are written from these two
[launcher.wheel]  J_kgm2 = 4.0e-6, kv = 1800, pole_pairs = 7, tau_motor_s = 0.04, p_max_w = 100.0, eta_transfer = 0.6, rpm_sigma_pm = 3     # telemetry/control noise
[launcher.feed]   t_push_ms = 80, t_commit_ms = 45, t_exit_ms = 8, t_exit_sigma_ms = 2          # timing, section 5.4
```

The firmware keys 23-24 (`wheel_d_um`, `v_cap_dmps`) are the only calib entries the MCU enforces independently; `rpm_max`, `rpm_idle`, `diff_pm` are runtime requests clamped by it.

### 4.4 Closed-loop speed control (firmware)

2 kHz per wheel: DShot300 frame (16 bits x 3.33 us = 53 us [est]) with the telemetry bit set, inverted line, ESC replies a 21-bit GRC frame (~56 us at 5/4 x 300 kbit/s) carrying 16 bits:
3-bit exponent, 9-bit mantissa period in us, 4-bit CRC; `eRPM = 60e6 / period_us`, `rpm = eRPM / pole_pairs` [est, from the bidirectional-DShot community description; **not fetched in this session**, FW worker verifies against the
Bluejay documentation before coding]. PI on rpm error: `u = u_ff(rpm_set) + Kp e + Ki integral e`, `u` clipped to `[0, throttle_max_pm]` (cfg, default 900 of 1000), setpoint ramp `ramp_rpm_s`. Setpoints: top = `mean*(1 - diff/2)`, bottom = `mean*(1 + diff/2)` with
`diff = diff_pm/1000`. Window test `|rpm - set| <= rpm_window_pm` for both wheels for `settle_ms`.

### 4.5 Licence position on the ESC firmware (state this plainly)

The ESC firmware (Bluejay, GPL-3.0 [src: https://github.com/bird-sanctuary/bluejay/blob/main/COPYING, fetched 2026-10-26]; AM32 is also GPL-3.0 [src: https://github.com/am32-firmware/AM32]; BLHeli_S-class ESCs
are bought pre-flashed with BLHeli_S or flashed by the user) **runs on the ESC's own microcontroller. It is not linked into, vendored in, or distributed with napl**: the repo contains no ESC firmware source
or binary, and napl and the RP2040 firmware talk to the ESC only over the DShot electrical protocol (an interface, not a derivative work). The user flashes the ESC themselves; the docs name the
firmware and version as a purchase/flash instruction only. This is an engineering position, not legal advice. `tools/check_licenses.py` must stay green; the DShot PIO program and telemetry decoder are written
from the public protocol description (BSD-3-Clause, same as the rest of the RP2040 firmware). The RP2040 firmware itself remains pico-sdk BSD-3.

## 5. Feeder

### 5.1 Hopper location and feed path (decision)

**Hopper on the tilt head, above and behind the breech, gravity top-feed through a rear-bottom outlet, capacity 24 balls (229 cm^3 at 0.60 packing [der]; 70 x 60 x 55 mm), refill lid on top.** Rejected:
(a) pan-mounted hopper + hollow trunnion (v1): unmeasured 76 mm level run, 6807ZZ, elbow, 90 deg turn; (b) head hopper with a front outlet: at tilt +70 gravity in the head frame is (-0.94 x, -0.34 z) [der: g_x = -sin theta,
g_z = -cos theta], so balls pile at the **rear** and a front outlet starves.

Why it works over -15..+70 [der]: the outlet is at the rear-bottom corner; the floor facet in front of it descends toward it at angle alpha = 50 deg to the head x-y plane. World slope of that facet at tilt theta is
alpha + theta: 35 deg at -15, 50 at 0, 85 at 35, overhang (balls fall to the rear wall, where the outlet is) at 70. The outlet is a **one-ball-wide throat** (26 mm = 1.17 D, straight 40 mm long): two balls cannot bridge a
throat narrower than 2 D, so no arching and **no agitator** (paintball hoppers need one because their throats are wide). The check C-FEED (section 12) asserts slope >= 15 deg for every path segment at tilt -15, 0, 35, 70.
Bench risk (Q3 in 14): foam balls are tacky; if the bench shows hangs, the fallback is a coin ERM vibration motor ($1) on the hopper wall, run only on `EMPTY_RETRY`.

The head carries the balls (45 g, 9 % of the head mass): they sit 40 mm above the axis (+0.02 N*m full, 0 empty; counted in the CG table).

### 5.2 Single-pocket gate and pusher

```
 side view (head frame, x forward, z up), ball staged
   hopper -> outlet (26 mm, rear-bottom)
              |
        [ VALVE ]  plug of 30 mm dia, axis along y, diametral through-bore 23.8 mm dia x 26 mm long (one ball)
   LOAD (bore axis 60 deg off the barrel axis, open to the outlet; ball falls in and rests on the housing floor)
   BORE (bore axis = barrel axis: ball staged in line with the pusher rod and the nip)
   rod -> [ball] ... 9.5 mm to first contact ... (|)(|) wheels ... barrel ... muzzle beam
```

- **Pocket (the "single-pocket rotary gate").** A quarter-ish-turn plug valve: a printed cylinder (axis along y) with a through-bore long enough for one ball (26 mm = 1.17 D: two ball centres need >= 2 D = 44.4 mm; C-POCKET). At LOAD the bore opens
  to the hopper outlet; at BORE the valve body closes the outlet (blocks the column) and the bore is coaxial with the barrel. Swing `GATE_SWING_DEG` = 60 (param; 45-90).
- **Gate drive and indexing.** SG92R-class 9 g micro servo (180-deg bounded -> **cannot rotate continuously**, the mechanical form of "stuck-on gives no runaway") through a 1:1 link; hard stops at both ends (printed), servo
  commands overshoot the stop by 2 deg (stall for the dwell only); positions cfg 34 (`gate_load_us`), 35 (`gate_bore_us`) defaults 1150 / 1700 us [est]. **Home sensor:** A3144 hall + 3 x 1 mm magnet on the valve arm at LOAD
  (GATE_HOME, active low). Homed at boot by commanding LOAD and waiting for the hall (timeout 1 s, `gate_stuck`).
- **Pusher.** SG92R + scotch-yoke slider (v1 mechanism, stroke trimmed 44 -> 32 mm [der: 9.5 mm to first contact + 5 mm margin + 17.5 mm to push the ball well into the squeeze, valve clearance: rod must retract out of the bore: 26 mm
  bore + 6 mm wall]). PUSH_HOME hall at retract. Rod tip is a 20 mm soft pad; the rod cannot enter the valve unless the bore is at BORE (rod aperture is in the housing wall facing the bore only).
- **Interlock geometry (I9).** With the rod extended it is inside the bore volume: the valve cannot turn (printed shoulder), and with the valve at LOAD the housing wall closes the rod aperture: rod stroke is physically impossible. Servo stalls
  would be limited by the 250 ms power window (I14).
- **Beams.** `BM_BREECH`: IR LED (940 nm) + phototransistor across the bore at BORE, 3 mm hole each side, reads "ball staged" (blocked). `BM_MUZ`: same, across the barrel 55 mm after the nip, reads "ball exit"; chrono: a ball of diameter 22.2 mm
  blocks the beam for `22.2 mm / v` = 0.74 ms at 30 m/s [der]; the RP2040 captures both edges with a 1 us timer on GPIO IRQ; `v_chrono = k_beam * 22.2 mm / dt` with `k_beam` calibrated (beam width and off-centre chord are errors of a few %: stated, calibrated against the bench chronograph).
  Both beams sample at 1 kHz plus edge IRQ; DC-coupled reading, sunlight rejection by a 940 nm LED with a daylight-blocking phototransistor and the closed barrel; the beam board is inside the printed cage (shaded).
- **No agitator, no belt, no continuous drive anywhere in the feed path.**

### 5.3 Cycle phases

`GATE_HOME_TO_LOAD` -> `LOADING` (ball falls into the pocket, `fill_ms` 30) -> `TO_BORE` (gate swing) -> `STAGED` (breech beam blocked) -> **FIRE** -> `PUSH` (extend) -> `COMMIT` -> `EXIT` (muzzle beam seen) -> `RETRACT` (pusher home) -> `TO_LOAD` ... repeat until `STAGED`.
Refill happens **after** the shot and the ball waits at BORE (staged) so FIRE latency is only the pusher stroke.

### 5.4 Timing diagram (cycle 0.466 s, target 0.50 s) [der unless tagged]

Servo speed 0.1 s per 60 deg [vendor-class SG92R figure at 4.8 V, not fetched this session, bench-verified item B-TIME]; a 60 deg gate swing is 100 ms; pusher stroke **47.2 mm** at 0.4 mm/ms = 118 ms out, 118 ms back
(MECH C-NIP / C-CYCLE, docs/cuas2/assembly.md: the rod must clear the plug cylinder; the first draft of this section assumed 32 mm / 80 ms and a 365 ms cycle, R3 was 0.40 s). The firmware timing constants
(`FDR_*` in `fw_core.h`) follow this diagram.

```
t (ms)    0     40  65 71        101  118 120      238      338      368        468
FEED_REQ  ^ rising edge (limiter as built: servo power on for 254 ms, no new edge accepted for 338 +-15 %)
pusher    |extend-----------------118|dwell 120|retract-----------238 (PUSH_HOME back; <= 254 = window)
ball      |bore   rod meets ball ~65; nip ~101 (commit at the latest 105); exit ~116, muzzle beam ~118
breech    |blocked      clears ~71 (beam 9 mm behind the plug axis: after ~2.5 mm of ball travel)
gate      |STAGED(BORE)                                 238|to LOAD------338|fill 338-368|to BORE------468
READY     |                                                                                    468 -> next FIRE (>= 470 interval)
```

Sum: extend 118 + retract 118 + gate to LOAD 100 + fill 30 + gate to BORE 100 = **466 ms** [der, MECH C-CYCLE]; the firmware state machine lands at 468 ms (rod home at 238). R3 is relaxed to 0.50 s (14 Q2), margin 32 ms (6 %);
the firmware clamp of `min_shot_interval_ms` is 470 ms, default 500, so the interval never asks for a ball the gate has not staged. If the bench shows a slower
servo (B-TIME) then `min_shot_interval_ms` follows the measured ready time and R3 is renegotiated again. The rod retract overlaps the gate only after the rod clears the valve shoulder (interlock above).

**Commit point (geofence "through the stroke").** The commit point is the **earlier of** `t_commit` = pusher start + 105 ms [est: the ball is in the nip and will leave regardless] **and the breech beam clearing after 40 ms** (~71 ms): once the staged ball has left the breech it has been moved off the position the
valve can turn it back from, so no abort may follow (an abort after that point would retract the rod behind a displaced ball; a clear before 40 ms is a vanished ball and aborts, reason 5). From FEED start to the commit point the MCU rechecks every 1 ms: sector (shrunk by `|v_axis| * t_exit`, t_exit = 8 ms -> 1.9 deg at 240 deg/s),
dead-man, inhibit, rail, and **wheels still in window (the recheck runs until the commit, not only the first 30 ms)**; any failure retracts the rod (no ball leaves: it is still in the bore) and sends EVENT 11 (reason) / EVENT 9 for a sector abort.
With the breech beam 9 mm behind the plug axis the clear comes ~30 ms before the ball reaches the nip, so a rail loss in that window leaves a counted but displaced ball (no exit: `no_exit` on recovery; nothing leaves the barrel). The retract does not hand the gate over until PUSH_HOME reads home (`TO_LOAD` needs `push_home`, else `pusher_stuck` at 330 ms). After commit the app has already clamped AIM to the braking-distance-shrunk sector (v1 rule); the
final 8 ms of drift is inside the 0.5 deg margin of v1 plus the shrink above.

### 5.5 Feeder state machine (identical in C and in the Python twin `napl.cuas2.fsm`)

```
 INIT -> GATE_HOMING -> LOADING -> TO_BORE -> STAGED --(FIRE accepted)--> PUSH -> (commit) -> EXIT_WAIT -> RETRACT -> TO_LOAD -> LOADING ...
 STAGED = READY for FIRE (all of I9b, I11, I12 true) ;  LOADING with empty breech after 3 retries -> EMPTY (EVENT 12), READY refused
 any timeout or beam inconsistency -> JAM fault (FAULT bits) ; E-stop/watchdog -> SAFE: pusher powered off by the limiter timeout, gate to LOAD
```

`READY` = `STAGED` and flywheels in window and `state == ENGAGING`. A FIRE outside READY returns the specific ACK (NOT_SPUN, NO_BALL, BUSY, JAMMED, NO_SECTOR, RANGE, INHIBIT, ...).

## 6. Firmware and protocol delta vs v1

### 6.1 Framing and versioning (unchanged structure)

COBS + CRC-16/CCITT-FALSE, 0x00 delimiter, `id | seq | payload | crc16`, max payload 60. **Protocol 1.1** (minor bump = tail-append only, per v1 section 2 rule): ids 1-0x0F and 0x81-0x86 keep their meaning; payload tails are
added; new codes only. `HELLO_ACK.fw_major = 2` (cuas2 firmware) lets the app tell the target apart; a v1 app on cuas2 still works (FIRE n=1, FLYWHEEL as a speed fraction, STATUS tail ignored); a v2 app on lite firmware sees `fw_major = 1` and
runs v1 behaviour.

### 6.2 Host -> MCU changes

| id            | v1                         | cuas2                                                                                                                                                                                                           |
| ------------- | -------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0x05 FLYWHEEL | `duty_permille:u16`        | `speed_permille:u16` (fraction of `rpm_max`, 0 = off; the app sends `rpm_idle/rpm_max` as pre-rev) + tail `diff_pm:i16` (override cfg 29, clamped +-60); refresh 10 Hz, MCU zeroes after 500 ms without refresh |
| 0x06 FIRE     | `fire_id:u16, n:u8 (1..3)` | `fire_id:u16, n:u8 = 1` (n != 1 -> BAD_ARG); **`fire_id` idempotence kept** (last 8 ids); ACK(result) then **EVENT 2 SHOT (fed) and EVENT 10 MUZZLE (exit)**                                                    |
| 0x07 ARM      |                            | unchanged; `shot_cap` is balls, one per FIRE                                                                                                                                                                    |
| 0x08-0x0B     | CFG, SAVE, CLEAR_FAULT     | CFG keys extended below; CLEAR_FAULT also clears the jam/no_exit latches                                                                                                                                        |

`ACK.result` additions: 12 `NO_BALL` (no staged ball), 13 `JAMMED` (feeder lock-out), 14 `NOT_CLEAR` (muzzle beam blocked), 15 `TOO_SOON` (interval; v1 used CAP).

### 6.3 STATUS (45 bytes, v1 offsets 0-28 unchanged; `fw_rpm` at 17 = mean wheel rpm)

| off  | field             | type | notes                                                                                                                                                                         |
| ---- | ----------------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0-28 | v1 fields         |      | `fw_duty` (15) now = throttle permille mean; `fw_rpm` (17) = (top+bot)/2, 0xFFFF = telemetry lost                                                                             |
| 29   | `rpm_top`         | u16  | rpm, 0xFFFF lost                                                                                                                                                              |
| 31   | `rpm_bot`         | u16  |                                                                                                                                                                               |
| 33   | `rpm_set`         | u16  | commanded mean rpm                                                                                                                                                            |
| 35   | `beams`           | u8   | b0 breech blocked, b1 muzzle blocked, b2 gate_home, b3 push_home, b4 wheels_ready (window held >= settle), b5 feed_busy, b6 limiter_enabled (EN sense, if wired), b7 reserved |
| 36   | `gate_state`      | u8   | 0 UNKNOWN, 1 HOMING, 2 LOADING, 3 TO_BORE, 4 STAGED, 5 PUSH, 6 EXIT_WAIT, 7 RETRACT, 8 TO_LOAD, 9 EMPTY, 10 JAM                                                               |
| 37   | `ready_flags`     | u8   | b0 READY, b1 empty, b2 jam latch, b3 hopper_low (3 failed refill tries), b4-7 reserved                                                                                        |
| 38   | `last_v_dmps`     | u16  | last chrono speed, 0.1 m/s, 0xFFFF none                                                                                                                                       |
| 40   | `last_dip_pm`     | u16  | rpm dip of the mean wheel at the nip, permille                                                                                                                                |
| 42   | `shots_confirmed` | u16  | muzzle-beam count since boot                                                                                                                                                  |
| 44   | `feed_ms`         | u8   | last measured FEED-to-exit time, 4 ms units (bench timing)                                                                                                                    |

Total 45 B; wire 51 B at 100 Hz = 5.1 kB/s.

### 6.4 Faults, events, config

`fault_bits` (u16): v1 b0-b5 kept; new b6 `telemetry_lost`, b7 `overspeed`, b8 `wheel_not_ready` (rpm window not reached in 1.5 s after request), b9 `breech_double` (blocked > 1.3x expected), b10 `no_exit`,
b11 `unexpected_exit` (muzzle without feed, or > 1 per feed, interval < 300 ms), b12 `pusher_stuck`, b13 `gate_stuck`, b14 `shot_overcount` (muzzle exits > feeds, set by the feeder), b15 `hw_counter` (the label of fault row 30; always accompanied by b6 `telemetry_lost`, which stops the weapon). Each of b6-b14 stops KICK (I15) except where noted in 1.3.
The safety FSM has its own internal assertion on b3 (the v1 `shot_counter` bit): `shots_arm > shot_cap` sets it, so b3 is the "more accepted feeds than the cap" check of v1 and b14 is the muzzle-beam "more exits than feeds" check; both stop KICK.

`EVENT` (v1 1-9 kept): **10 MUZZLE** (`arg = v_dmps | dip_pm<<16`), **11 FEED_ABORT** (`fire_id<<8 | reason`: 1 sector, 2 deadman, 3 inhibit, 4 wheels, 5 beam, 6 rail), **12 EMPTY** (hopper empty after 3 retries),
**13 JAM** (`phase`), **14 WHEELS_READY** (edge, arg = mean rpm), **15 WHEELS_LOST**, **16 HW_COUNTER** (arg = muzzle-beam pulses counted since key-on; see fault row 30, cfg 41). EVENT 2 SHOT = ball fed (compatible with v1 counters, `ball index` 0).

Config keys (image version 3, 41 keys; the cuas2 firmware refuses v1/v2 images). Keys 1-22 are v1's; 12-14 now mean pusher ext/ret/dwell (14 default **120**, range 50-130: dwell + the 118 ms retract must fit the 254 ms limiter window); 15 `min_shot_interval_ms` default **500** (clamp >= 470); 16 `spinup_ms` becomes the **window settle** time default 100 (unused open-loop meaning removed).

| key | name                          | unit             | default                               |
| --- | ----------------------------- | ---------------- | ------------------------------------- |
| 23  | wheel_d_um                    | um               | 40000 (maintenance: SAFE + CFG_SAVE)  |
| 24  | v_cap_dmps                    | 0.1 m/s          | 380 (clamped <= 380)                  |
| 25  | rpm_max                       | rpm              | 17500 (clamped to rpm_cap)            |
| 26  | rpm_idle                      | rpm              | 4500                                  |
| 27  | rpm_window_pm                 | permille         | 10 (CFG_SET range 1-20)               |
| 28  | settle_ms                     | ms               | 100 (CFG_SET range 20-2000)           |
| 29  | diff_pm                       | permille, signed | 15                                    |
| 30  | pole_pairs                    |                  | 7                                     |
| 31  | ramp_rpm_s                    | rpm/s            | 40000                                 |
| 32  | throttle_max_pm               | permille         | 900                                   |
| 33  | kp (x1e-3), 34 ki (x1e-3)     |                  | 600, 40 [est, tune on the bench]      |
| 35  | gate_load_us, 36 gate_bore_us | us               | 1150, 1700 [est]                      |
| 37  | fill_ms                       | ms               | 30                                    |
| 38  | exit_timeout_ms               | ms               | 250                                   |
| 39  | telemetry_loss_ms             | ms               | 50                                    |
| 40  | k_beam (x1e-3)                |                  | 1000                                  |
| 41  | hw_counter_n                  | balls            | 10 (0 = no counter fitted; SAFE only) |

(Final numbering is by the FW worker; the table is a contract on names, units and defaults.) The fire sector (17, 20-22) remains unset after every boot.

### 6.5 Pins (RP2040, `firmware/cuas2-rp2040/include/pins.h` is the single source)

Kept as v1: GP0/1 tilt TMC UART, GP2/3/4 pan STEP/DIR/EN (**EN shared with tilt**, so GP7 frees), GP5/6 tilt STEP/DIR, GP8/9 halls, GP10 E-stop sense, GP11 key sense, GP14 pusher servo (PWM7A), GP17 buzzer, GP20/21 pan TMC UART, GP22 DIAG.

| pin  | cuas2 function                                    | v1 use                    |
| ---- | ------------------------------------------------- | ------------------------- |
| GP7  | BM_BREECH (in, phototransistor)                   | tilt EN (merged into GP4) |
| GP12 | DSHOT_TOP (PIO0 SM0, bidirectional)               | flywheel PWM              |
| GP13 | DSHOT_BOT (PIO0 SM1)                              | tach (unused)             |
| GP15 | status RGB LED (WS2812, PIO1)                     | red LED                   |
| GP16 | BM_MUZ (in, GPIO IRQ, 1 us timestamp)             | green LED                 |
| GP18 | GATE_SERVO (PWM1A, 50 Hz)                         | free                      |
| GP19 | FEED_REQ (out -> limiter trigger, 10 k pull-down) | free                      |
| GP26 | GATE_HOME (in, A3144)                             | free                      |
| GP27 | PUSH_HOME (in, A3144)                             | free                      |
| GP28 | KICK (out, 100 Hz toggle -> supervisor)           | free                      |

Zero spare GPIO is left: further modules (rangefinder, second camera, thermal) attach to the Jetson over USB, not to the RP2040 (decision; no I2C expander).

### 6.6 Test vectors to add (`tests/cuas2/protocol_vectors2.json`; C and Python decode the same file, plus the v1 file stays green against the shared code)

1. `FLYWHEEL` with and without the tail (accepted both, tail ignored by a v1 receiver). 2. `FIRE n=1` ok, `n=0`, `n=2` -> ACK BAD_ARG; duplicated `fire_id` returns the old result and **does not feed** (check muzzle count). 3. `STATUS` 45 B round trip and a 29 B (v1) STATUS accepted by `Status2.unpack` with defaults.
2. `EVENT 10..15` payloads. 5. `CFG_SET` of keys 23-40 with range errors (`v_cap_dmps` 381 -> BAD_ARG; `wheel_d_um` 0 -> BAD_ARG). 6. `CFG_SET v_cap` outside SAFE -> BAD_STATE. 7. CRC check value `0x29B1`.
   Time-stamped FSM vectors (`tests/cuas2/fsm_vectors.json`, consumed by the C host build and the Python FSM): (a) nominal shot, (b) FIRE with wheels out of window -> NOT_SPUN, (c) empty breech -> NO_BALL, (d) muzzle beam blocked -> NOT_CLEAR, (e) sector abort at 20 ms (before commit) and at 60 ms (after commit: the ball leaves), (f) dead-man drop before commit, (g) no_exit jam, (h) unexpected exit with no feed -> FAULT + KICK stop, (i) interval violation (<400 ms), (j) overspeed, (k) telemetry loss, (l) E-stop mid-stroke, (m) watchdog trip, (n) stuck FEED_REQ line (hardware model: second edge ignored for 330 ms), (o) empty hopper (3 retries, EVENT 12).

## 7. Simulation model contract (MODEL worker)

All launcher physics is parameterised from `calib.toml` (4.3) with `[est]` defaults, in `python/napl/cuas2/launcher.py` (`LauncherModel`) and `ballistics.py` (`fly`, `fly_batch`, `solve_lead2`).

### 7.1 Wheel dynamics (per wheel, 1 ms step)

State `omega_i` (rad/s), commanded throttle `u_i in [0,1]` from the DShot value (0..2000 -> u = (dshot-48)/1952). Target `omega_t = 2pi/60 * kv * V_bus * u * (1 - droop*load)`, `V_bus` 12 V (a parameter; sagging model optional).
`d omega/dt = clamp((omega_t - omega)/tau_motor, -a_brake, +P_max/(J omega + eps))`. Telemetry: `rpm_meas = omega*60/2pi * (1 + N(0, rpm_sigma_pm/1000))`, refreshed at 2 kHz with probability 0.999 (dropout [est]), age field available to the twin. Idle current etc. not modelled (the power budget is checked in `bench`, not simulated).

### 7.2 Shot (at the nip crossing = `t_feed + t_commit + t_exit_jitter`)

```
v_t = pi D rpm_top/60 ;  v_b = pi D rpm_bot/60 ;  v_mean = (v_t + v_b)/2
v0     = k_slip * v_mean * (1 - dip_in_contact) * (1 + N(0, sigma_v_rel)) + N(0, sigma_v_floor)           # clipped to <= v_cap
omega_ball = k_spin * (v_t - v_b) / (2 r_b)       # rad/s, negative = backspin (bottom faster); about the head y axis
direction  = barrel axis tilted by bias + N(0, spread_mrad) per axis (horizontal, vertical)
ball: d ~ N(d, d_sigma), m ~ N(m, m_sigma), cd ~ N(cd, cd_sigma)
energy:  dE_ball = 0.5 m v0^2 ; each wheel loses  dE_i = (dE_ball / eta_transfer) * (v_i / (v_t + v_b)) ;   omega_i' = sqrt(max(omega_i^2 - 2 dE_i / J, 0))
```

The ball exits with `v0` after the dip is applied to the wheel state (the dip enters `dip_in_contact` as half of the fractional rpm drop). `k_slip` etc. may depend on `gap_mm` (bench lookup table `k_slip_gap = [[gap, k], ...]` optional).

### 7.3 Ballistics with Magnus

`a = g - k |v - w| (v - w) + a_M`, `k = rho cd A/(2 m)` as v1; `a_M = (0.5 rho A C_L |v|^2 / m) * unit(omega_hat x v_hat)`, `C_L = clip(cl_k * S, 0, cl_max)`, `S = |omega| r / |v|`, `omega(t) = omega0 exp(-t/spin_tau)`, with the spin axis lateral (y) so
backspin lifts. At 30 m/s: dv 1 m/s -> 430 rpm, S 0.017, lift/weight 0.15 at CL = 0.75 S [der]. RK4 1 ms in the kernel; `fly_batch` (vectorised numpy, dt 8 ms midpoint) for fire control; the two must agree to 2 cm at 10 m (test M-FLY).
`solve_lead2` extends v1's `solve_lead` (same fixed point) with the spin term and `v0 = LauncherModel.mean_v0(set point)`.

### 7.4 Consumers

- **Twin MCU (`Sim2RP2040`)**: runs the wheel model each ms behind its DShot outputs and returns eRPM telemetry; the feeder FSM (`napl.cuas2.fsm`, FW-owned) is stepped with beam inputs generated by `FeederPlant` (MODEL-owned): the pusher/gate servo are first-order lags with the 7.2 timings, the breech beam is blocked while a ball is in the bore, the muzzle beam pulses for `22.2/v` at the exit time. At the nip crossing it calls `LauncherModel.sample_shot()` and writes the launcher device attrs.
- **napl kernel `launcher` device**: additive attrs (default 0 -> bit-identical v1 behaviour): `muzzle_velocity` (set per shot), `muzzle_velocity_std = 0`, `spread_std = 0` (the Python side draws, so the replay hash is deterministic), `dir_offset_x/y` (rad, deterministic aim error), `spin_rps` (signed), `cl_k`, `spin_tau`. The device implements 7.3 in the pellet integrator. Target geoms keep the v1 rule (inflate by the 11.1 mm ball radius).
- **Fire control (TRACK)** calls `LauncherModel.sample_shots(n, rng, setpoint)` -> arrays `(v0, dir_err[n,2], omega, d, m, cd)` and `fly_batch`; the same sampler as the sim so Ph is calibrated against the plant.
- Plain-Python HIL path: same model behind the pty, rpm telemetry in STATUS.

## 8. Interface to tracker and fire control (TRACK builds `python/napl/track/`)

```python
@dataclass
class Track:
    id: int; t: float                         # state time (monotonic s, same clock as Frame.t_cap)
    x: np.ndarray                             # (6,) or (9,) mean in the base frame B: pos(3) m, vel(3) m/s [, acc(3)]
    P: np.ndarray                             # covariance, same size
    cls_prob: dict[str, float]                # {"drone","balloon","bird","decoy","person","unknown"} sums to 1
    mode_prob: np.ndarray | None              # IMM mode probabilities
    age: int; hits: int; misses: int; confirmed: bool
    range_src: str                            # "size" | "stereo" | "lidar" | "radar"
    def predict(self, t) -> tuple[np.ndarray, np.ndarray]:       # mean (3,), cov (3,3) of position at time t >= self.t
    def sample(self, t, n, rng) -> np.ndarray:                     # (n,3) positions at time t (IMM mixture aware)
```

Tracker input is unchanged in spirit from R3: `update(dets_with_t_cap, pose_at_t_cap) -> list[Track]`.

**Fire control** (TRACK, `firectl.py`): for each candidate track and the current setpoint,

1. `t_imp` from `solve_lead2` (nominal); `t_dead = latency_now + t_actuator(0.06) + t_feed(0.045, = t_commit) `.
2. Sample `N = 500` target positions `p_i = track.sample(t_imp, N, rng)` (+ range and class noise) and `N` shots `LauncherModel.sample_shots(N, rng, setpoint)`, fly with `fly_batch` aimed at the **nominal** solution, record the closest-approach point at `t_imp`.
3. `hit_i = |p_ball_i - p_i| <= r_target(cls) + r_ball`; `Ph = mean(hit)`, with a Wilson interval; target shape sphere `r_target` by class (balloon 0.15 m, drone 0.15-0.25 m).
4. **Fire iff** `Ph >= tau` (default 0.25 [est, R3 6.2; sweep 0.1-0.5]) **and** all app-side gates (person, sector, armed, dead-man, cap, class prob of a target class >= 0.7 [est]) **and** `status.ready_flags & READY`. One `FIRE` per decision (single ball); shoot-look-shoot: after the shot, the next decision is not before `t_flight + t_obs (0.15 s)` and the track's post-shot residual updates the aim bias online.
5. Budget: `Ph` <= 10 ms laptop, <= 40 ms Jetson per call [est]; top-3 candidates per 50 ms. If over budget use the linearised sensitivity map (Jacobian of impact point wrt v0, direction, spin from the nominal solve; same sampler, O(N)).

Against the MCU the contract is unchanged: `AIM` (+ `FLYWHEEL`) then one `FIRE(fire_id, 1)`; hit/miss accounting by `EVENT 10` and the simulator's referee. MODEL provides `LauncherModel`/`fly_batch`; TRACK owns Track, filters, `firectl.py`; the gate and Ph policy never move into the MCU (the MCU only refuses).

## 9. BOM target and estimate

v1 hardware + filament: **$151.93** [src docs/cuas-lite/bom.md, excl. Jetson, Brio, rounds]. Target cuas2: **<= $190** (v1 + 25 %; estimate below is $187). All cuas2 line prices are **[est]** (not fetched); the MECH worker replaces them with linked prices.

Removed (v1 lines): 180 motors 3.16, rubber wheels 1.38, 6807ZZ 7.49, IRLB8721 4.50, 1N5822 0.68, Songle relay 0.44 = **-17.65** (pusher servo stays: v2 has _two_ SG92R, so +5.95 in the adds).

Added: 2 x 2207 1800 KV brushless 18.00; 2 x 30 A ESC 16.00; gate servo 5.95; 608ZZ (second trunnion bearing) 0.80; 40 A automotive relay + socket 3.50; 3 IR LED/phototransistor pairs 1.50; 2 more A3144 + magnets 2.04; limiter + supervisor (74HC123 x2, P-MOSFET, passives) 1.50; WS2812 0.30; connectors (3 x JST-XH sets, XT30 x2) 4.00; extra wire and resistors 1.25 = **54.84**.

Hardware: 129.94 - 17.65 + 54.84 = **167.13**. Filament: v1 21.99 minus the removed trunnion/elbow/hopper-on-pan, plus hopper/cage/gate/pusher on the head, net **-2.00** [est] = 19.99. **Total ~ $187.12 [der]**, $2.9 under the $190 target; levers to about $180 (not applied): 2205 motors (-$4), 4-in-1 ESC board for the pair (-$2..-$4), drop WS2812 for two LEDs (-$0), skip the 20 A fuse upgrade, 3D-print the sensor-bar from scraps. Optional modules: TF-Luna $21-45, second USB camera $30-60, Lepton 3.5 $164, IWR6843 EVM $186 [src R1 s5 URLs in section 13]. Rounds (100 pack) $14.99 outside the total.

## 10. Bench validation plan and how results update calib.toml

Procedure order (nothing is committed as media; results go in `docs/cuas2/bench_validation.md` and `calib.toml`):

| step   | what                     | how                                                                                                                                                     | updates                                                                                                                                                         |
| ------ | ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| B0     | measure 50 Rival rounds  | micrometer d (2 axes), scale m                                                                                                                          | `[ball] d_mm, m_g, d_sigma_mm, m_sigma_g` + CAD `BALL` regenerates                                                                                              |
| B-TIME | gate and pusher timing   | scope/logic on PUSH_HOME, GATE_HOME, FEED_REQ, BM_*; 30 cycles                                                                                          | `t_push_ms`, `fill_ms`, `gate_*_us`, min interval; renegotiate R3 if cycle > 0.50 s                                                                             |
| B1     | motor/ESC bring-up       | prop-free spin, DShot arm, eRPM vs scope on one phase wire (pole check), `ramp_rpm_s`, current on the bench supply                                      | `pole_pairs`, `kp/ki`, `throttle_max_pm`, `J_kgm2` (from spin-down time)                                                                                        |
| B2     | chrono sweep             | external chronograph (or video at known distance) at the muzzle; 3 gaps (18.2, 19.2, 20.2) x 5 speeds (12k-17.5k rpm), 20 shots each, 2 tyre durometers | `k_slip` (and gap table), `sigma_v_rel`, `sigma_v_floor`, the v_cap margin check (**no shot above 36 m/s while tuning**), `k_beam` from beam chrono vs external |
| B3     | hit grid                 | 0.6 x 0.6 m grid at 5 m and 10 m, 30 shots each at fixed setpoint, diff_pm 0                                                                            | `spread_mrad`, `bias_mrad`                                                                                                                                      |
| B4     | backspin sweep           | diff_pm in {-15, 0, 15, 30, 45}, 20 shots at 10 m, measure vertical impact height                                                                       | `cl_k`, `k_spin`, default `diff_pm` (choose the value minimising drop/spread)                                                                                   |
| B5     | drop/drag                | 5 m and 10 m impact height at diff 0 vs `solve_lead2`                                                                                                   | `cd`                                                                                                                                                            |
| B6     | dispersion vs rpm window | tighten `rpm_window_pm` 20 -> 5 and `settle_ms`; scatter vs window                                                                                      | window defaults                                                                                                                                                 |
| B7     | jam/hopper test          | 24 balls through the head at tilt -15, 0, 35, 70 (hold the head on a jig), 10 runs each; hang/ double-feed count                                        | hopper geometry or ERM fallback                                                                                                                                 |
| B8     | faults                   | run the fault table 1.3 rows 3-15, 17, 18, 22-25 on the bench (hopper empty of balls first)                                                             | pass/fail table                                                                                                                                                 |

Rule: a calibration key is changed only from a step in this table and its `_src` flips to "bench"; the sim and the fire control read the same file.

## 11. Open risks (carried in the work packages as bench items)

Servo speed assumption (B-TIME); foam tack/hang in the throat (B7); `k_slip` could be < 0.7 (then wheels 45 mm, cap recomputed); DShot bidirectional PIO timing (FW vs ESC firmware version); hardware limiter tolerance (worst case 402 ms vs
firmware 450 default); TPU tyre wear; Brio is not a global-shutter sensor (v1 issue).

## 12. Work packages (file ownership, no overlaps) and failable acceptance checks

Common: Python tests with `pytest -n 8 -p no:cacheprovider` on the named files only; CAD on the build host (`cad-venv`); firmware host build with `tools/fw_host_test.sh`-style script `tools/cuas2_fw_host_test.sh` (FW creates). No file in `cad/cuas-lite`, `firmware/cuas-lite-rp2040`, `python/napl/lite`, `tests/lite`, `assemblies/cuas-lite.toml`, `docs/cuas-lite` is edited by any package; a gap found there is reported, not patched.

### MECH (CAD + assembly + BOM)

Owns: `cad/cuas2/**`, `parts/cuas2/**`, `assemblies/cuas2.toml`, `docs/cuas2/{assembly,bom,wiring}.md`. Reads: section 3, 4.1, 5.1-5.2, 6.5 (pins), 9.

Deliver: parametric CAD for the six modules with the dock definitions as data (`interfaces.py`, consumed by both mating modules so a bolt-pattern edit changes both), STEP/STL/napl meshes + masses/inertias, the assembly with pin-level wiring per 6.5, BOM with fetched prices.

Acceptance (all must print a line `PASS` and exit 0; each fails if the property is broken):

1. `cad-venv/bin/python cad/cuas2/build.py --out build/cad/cuas2` runs all checks in `checks.py`: `C-GAP` (wheel gap 19.2 +-0.05 at nominal and 18.2/20.2 reachable by the slot travel), `C-POCKET` (two ball centres cannot fit in the pocket bore: bore length < 2 x BALL_D x 0.9), `C-FEED` (hopper->outlet->pocket path slope >= 15 deg at tilt -15/0/35/70), `C-NIP` (pusher stroke >= 9.5 + 5 mm and the rod clears the valve shoulder when retracted; valve cannot turn with the rod extended: swept-volume intersection > 0), `C-INTERLOCK` (rod aperture closed when gate at LOAD), `C-BAL` (R6: CG offset <= 8 mm, |static torque| <= 0.06 N*m at -15/0/35/70, head mass <= 0.60 kg), `C-DOCK` (each of D1, D2, F1, H1: bolt pattern and datum identical on both sides, clearance 0.3 +-0.1 mm), `C-PATH` (ball path clear in the swept head at all tilts), `C-FIT` (every printed part <= 200 x 200 x 200 mm).
2. `build.py` with a deliberately changed `GATE_SWING_DEG`, `WHEEL_D`, and `BALL_D` (+0.8 mm) each regenerates and the checks that depend on it change value (negative control: set the pocket bore to 2.2 D and `C-POCKET` must FAIL).
3. Assembly DRC of `assemblies/cuas2.toml` = 0 errors (same tool as v1 `assemblies/cuas-lite.toml`), every pin in 6.5 wired to the named parts, wire list diff vs v1 in `wiring.md`.
4. `docs/cuas2/bom.md` total recomputed by a script equal to the sum shown; every line has a link or an explicit `[est]`; total <= $190 or the overage explained.
5. `tests/cuas2/test_cad_consts.py`: `cad/cuas2/params.BALL == cad/cuas-lite/params` BALL (reads the lite file by path).

### MODEL (launcher + ballistics sim + twin plant)

Owns: `python/napl/cuas2/{calib,launcher,ballistics,sim_io,sim_mcu}.py`, `src/mujoco/devices/launcher.cpp` (additive attrs only), `parts/cuas/cuas2-foam-launcher.toml`, `tests/cuas2/test_{calib,launcher,ballistics,plant,kernel_launcher}.py`, `docs/cuas2/bench_validation.md`. Needs from FW: the `napl.cuas2.fsm` module interface (`FeederIn`/`FeederOut`, section 12 FW), fixed as the class signature below; until FW merges MODEL uses a stub with the same signature.

`FeederIn`: `t_ms, rpm_top, rpm_bot (None = lost), beam_breech, beam_muzzle, gate_home, push_home, estop_open, key_on, hb_ok, deadman, inhibit, in_sector, cmd (None|"FIRE"), speed_req_pm, diff_pm, cfg`; `FeederOut`: `dshot_top, dshot_bot, gate_us, push_us, feed_req, kick, status_fields, events, ack`.

Acceptance:

1. `pytest -n 8 tests/cuas2/test_calib.py`: defaults load with no file; each key round-trips; unknown key rejected; `v_cap` > 38 rejected; `_src` provenance parsed.
2. `test_launcher.py`: (a) `v0 = k_slip*(v_t+v_b)/2`: k=0.8, rpm 17,905/17,905 at D=40 -> 30.0 +-0.05 m/s; (b) spin sign: bottom faster -> negative omega; 1 m/s dv -> 430 rpm (+-5 rpm, k_spin=1); (c) rpm dip per shot at 17.5k rpm, J=4e-6, eta 0.6: 5.3 % +-0.5 % of the wheel; (d) over 5000 sampled shots: sigma_v, spread match the calib within 3 %; (e) cap: a requested rpm above `rpm_cap` never yields `v0 > 38`.
3. `test_ballistics.py`: with `cl_k = 0` and spin 0 `fly` reproduces v1's `ballistics_table.py` rows (elev. 1.06/1.93/3.54/4.88/6.49/9.61 deg at 3/5/8/10/12/15 m, tolerance 0.01 deg); 1 m/s backspin raises the 10 m impact height by the closed-form lift +-15 %; `fly_batch` vs RK4 kernel `fly` <= 2 cm at 10 m.
4. `test_kernel_launcher.py`: with the new attrs left at 0 the lite replay hashes are **unchanged** (`tests/lite/test_lockstep.py -k hash` passes on a rebuild: the negative control for "additive"); with `spin_rps` set the pellet height at 10 m differs from the no-spin run by the model value +-1 cm.
5. `test_plant.py`: with the stub FSM, a FIRE sequence produces exactly one pellet spawn, the muzzle-beam pulse width is `22.2 mm/v +-5 %`, wheels dip then recover within 40 ms; no `fire` attr is written without a FEED_REQ.
6. `calib.toml` example in `docs/cuas2/bench_validation.md` loads in `calib.py` (doc test).

### FW (firmware + protocol + twin FSM)

Owns: `firmware/cuas2-rp2040/**`, `python/napl/cuas2/{protocol,fsm,gimbal}.py`, `tests/cuas2/{protocol_vectors2.json,fsm_vectors.json,gen_vectors.py,test_protocol2.py,test_fsm.py,test_fw_vectors2.py}`, `tools/cuas2_fw_host_test.sh`, `tools/cuas2_fsm_drift.sh`. Reads: sections 1.2-1.3, 5, 6. Does not touch lite sources (compiles them by path).

Acceptance:

1. Host build + `ctest` of `firmware/cuas2-rp2040/host` passes; the x86 core builds with `-Wall -Wextra -Wshadow` clean; UF2 target configured (`tools/fw_build.sh` analogue) if the toolchain exists.
2. `pytest -n 8 tests/cuas2/test_protocol2.py`: all vectors of 6.6 items 1-7 decode identically in C (via the host lib) and Python; CRC check value `0x29B1`; v1 file `tests/lite/protocol_vectors.json` still passes against the **shared** framing code (`tests/lite/test_protocol.py` run unchanged).
3. `pytest -n 8 tests/cuas2/test_fsm.py tests/cuas2/test_fw_vectors2.py`: every vector (a)-(o) of 6.6 gives identical outputs from the C core and the Python `napl.cuas2.fsm` (byte-exact events and ACK results, timing within 1 ms). Negative controls required: flipping any single gate in the C code (e.g. drop the muzzle-clear check, drop the interval clamp, drop the `unexpected_exit` fault) makes at least one vector fail.
4. Safety matrix: a table-driven test that applies each single fault of 1.3 (rows 1-28 that are software-observable) and asserts "no FEED_REQ after the fault except the allowed first" and "KICK stops where specified"; count of balls exited vs accepted FIRE = 1:1 in all runs; a fuzz of 20k random command sequences never yields `shots_confirmed > accepted FIRE` and never two feeds < 400 ms apart.
5. `v_cap`: `CFG_SET` 381 rejected, `wheel_d_um` 25000 gives `rpm_cap = 29,000` but the abs maximum surface-speed check still caps at 38 m/s; a simulated rpm > 1.05 cap for 20 ms faults within 25 ms.
6. `tools/cuas2_fsm_drift.sh` prints the diff stat of the non-weapon functions against `firmware/cuas-lite-rp2040/src/fsm.c` and fails if it is not empty for the functions listed in `tests/cuas2/fsm_shared_functions.txt` (watchdog, ARM, heartbeat, E-stop/key edge handling).

### Not assigned in this wave (stated so nobody builds it by accident)

TRACK worker: `python/napl/track/**` incl. `firectl.py` (section 8). THREAT/eval workers: scenarios. INTEG (after TRACK + FW merge): `napl.cuas2.app`, web UI fields, `examples/cuas2`, HIL harness, video.

## 13. Sources

- R1 (all [src] below are from the research file `p13-research/R1-launcher-modular.md`; URLs as fetched there): Rival round 22.2 mm / 1.86 g (the-daily-dabble.com/best-nerf-rival-balls/, secondary); 22.7 mm third-party round https://outofdarts.com/products/worker-ammo-rival-rounds ; Nemesis ~101 fps https://blasterpiece.com/reviews/nerf-rival-nemesis-mxvii-10k-review/ ; Prometheus mod kit 125 fps [vendor] https://outofdarts.com/collections/full-mod-kits-2019/products/prometheus-mod-kit ; dual-wheel model https://robot.mbhs.edu/media/whitepapers/2026/flywheel-calculations-latest.pdf ; AIMY table-tennis launcher (ramp-up, pinch, presence sensor) https://arxiv.org/pdf/2210.06048 ; brushed 130 data https://outofdarts.com/products/ood-kraken-motor ; brushless blaster build https://projecthub.arduino.cc/Clark3DPR/full-auto-3d-printed-brushless-nerf-blaster-arduino-control-ffd509 ; flywheel compression practice https://www.vexforum.com/t/flywheel-compression-and-launch-angle/103666 ; drag/Magnus https://www.physics.usyd.edu.au/~cross/TRAJECTORIES/Sports%20Balls.pdf ; paintball loader jam patent https://patents.google.com/patent/US8950386 ; SAPIENT naming template https://www.gov.uk/guidance/sapient-autonomous-sensor-system ; TF-Luna https://files.waveshare.com/upload/c/c3/SJ-GU-TF-Luna_A05_Datasheet.pdf , https://www.robotshop.com/products/benewake-tf-luna-8m-lidar-distance-sensor ; Lepton 3.5 https://groupgets.com/products/flir-lepton-3-5 ; IWR6843 https://punchouttest.digikey.de/en/products/detail/texas-instruments/IWR6843LEVM/18159132 .
- R3 (`R3-tracking-ml.md`): Ph by Monte Carlo over track covariance x launcher dispersion, tau start 0.15-0.3, shoot-look-shoot, track fusion by covariance intersection https://stonesoup.readthedocs.io/en/v1.2/auto_examples/Track2Track_Fusion_Example.html ; doctrine reference https://iaac.technion.ac.il/workshops/2025/iaac3/Merkulov.pdf .
- Fetched this session: Bluejay licence GPL-3.0 (COPYING at https://github.com/bird-sanctuary/bluejay/blob/main/COPYING) and its bidirectional-DShot RPM telemetry feature (https://github.com/bird-sanctuary/bluejay); AM32 GPL-3.0 and "Bi-directional Dshot" (https://github.com/am32-firmware/AM32; RPM telemetry not stated on that page).
- v1: `docs/cuas-lite/spec.md`, `bom.md`, `cad/cuas-lite/params.py`, `S0-code-survey.md` (code map with file:line).
- NOT fetched (marked [est]/"not fetched" above): SG92R speed 0.1 s/60 deg, bidirectional-DShot frame format, all cuas2 prices, TPU/silicone durometer behaviour, 2207 motor mass and inertia.

## 14. Open questions for the user

Orchestrator 2026-10-26 (user standing instruction: proceed on defaults, user interrupts if amiss): all defaults below are taken, except Q6, where the hardware shot
counter is added because it is the only bound on a firmware fault that also defeats the muzzle-beam cross-check, for about $1.5. The user can still overrule any row.

| #   | question                                                                                                                                                                                                                                                | default taken                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Q1  | Is a head-mounted 24-ball hopper acceptable (vs 40 balls on the pan group in v1)? Refill every 24 shots. A bigger head hopper costs head mass; a pan reservoir needs a transfer path again                                                              | 24 on the head                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| Q2  | Cycle 0.365 s depends on a 0.1 s/60 deg servo (not verified); if the bench shows 0.15 s the cycle is ~0.47 s. Accept relaxing R3 to 0.5 s, or spend on a faster servo (+$3)?                                                                            | RELAXED (MECH C-CYCLE 466 ms with the 47.2 mm stroke): R3 0.50 s, min interval default 500 ms, clamp 470 ms                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| Q3  | Foam balls and a 26 mm throat without an agitator: accept the ERM fallback if it hangs?                                                                                                                                                                 | yes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| Q4  | Budget: ~$187 vs the $190 target and v1 $152; is +$35 for brushless + gate + connectors acceptable, or should cuas2 ship a "cheap" variant (brushed + tach, $-25, open-loop speed, no backspin control)?                                                | brushless only                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| Q5  | OK to add five default-0 attrs to the kernel `launcher` device (additive, lite hashes unchanged, tested)?                                                                                                                                               | yes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| Q6  | Residual in fault 3: a firmware bug that also defeats the cross-check is bounded by 9.6 s of hopper and the E-stop only. Accept, or add a hardware shot counter (a CD4017 decade counter on the muzzle beam that opens the relay after N balls) ~ $1.5? | ADDED (orchestrator 2026-10-26): CD4017-class counter on the muzzle beam opens the relay after `N` balls per arm (DIP/jumper, default 10), reset only by the key; MECH adds it to BOM + wiring, FW adds its sense line and fault row. AS BUILT (wiring.md 3.4): two 74HCT4017, N = 10 default (jumper 20/30), it counts muzzle-beam pulses and clamps the gate of the supervisor's weapon MOSFET; relay, key and E-stop stay closed, so GP10 sees nothing and there is no spare GPIO for a sense line. FW therefore infers the trip: weapon power lost (both ESC telemetry streams drop within 5 ms, key on, E-stop closed) with the per-key-session pulse count at cfg 41 = EVENT 16 + b15 (fault row 30); the trip itself is unsensed, per key session, and a real power fault at exactly the N-th ball is labelled the same way. Needs no wiring change; a direct sense line would cost the buzzer pin (GP17) |
| Q7  | Velocity cap 38 m/s is above a typical 120 fps event cap; add a "club mode" `v_cap` 36.5 m/s default?                                                                                                                                                   | keep 38, `rpm_max` default gives ~30 m/s                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| Q8  | ESC firmware: the user flashes Bluejay (GPL-3.0) or buys ESCs pre-flashed; the repo only documents it. OK?                                                                                                                                              | yes, section 4.5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Q9  | Wheel tyre: TPU print first, silicone cast second?                                                                                                                                                                                                      | TPU                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| Q10 | cuas2 as its own firmware tree (this spec) vs a target in the lite tree: this spec forks `fsm.c`; confirm that the safety-FSM fork is acceptable                                                                                                        | forked, drift script                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
