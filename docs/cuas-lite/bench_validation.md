# cuas-lite bench validation plan (v1)

Order matters: each section assumes the ones above it passed. Every section ends with **Pass/fail** (numbers are proposals derived from the spec's tolerances, marked [est] where
they are a judgment call) and **Feed back** (which sim parameter the measurement replaces). Do not move on after a FAIL: fix it, repeat the section, log the run.

Spec references: protocol 2, homing and drivers 2.3, state machine 3, boresight and ballistics 5.2-5.3, mechanics 6, firmware UNTESTED table in
`firmware/cuas-lite-rp2040/README.md`. Sim parameters live in `python/napl/lite/aim.py` (`BALL`, `V0_NOMINAL`, `V0_SIGMA`, `SPREAD_SIGMA`, `CLASS_SIZE_M`, `T_ACTUATOR`,
`T_FIRE`, `MIN_CONF`), `~/.config/cuas-lite/calib.toml` (`k_v`, boresight, intrinsics) and the firmware config (`CFG_SET`/`CFG_SAVE`, spec 2.2).

## 0. Safety rules for every session

- Eye protection for everyone in the room. Never aim at a person or a pet, even with the flywheels off; the barrier (a plywood or foam board behind the targets, a net
  at the sides) is in place before any ball is loaded. Foam balls only (the Rival round, 22.2 mm / 1.86 g, spec 5.3): no darts, no pellets.
- The weapon rail is dead (key out, fuse removed) in sections 1-2. It is made live only in 1.5 onward, by one person, announced aloud, with the E-stop within reach.
- First live test of anything is **dry**: flywheels at 0, pusher cycling with no balls. Balls go in only after the dry test passed.
- Keep a log (date, firmware git hash, UF2, config dump via `CFG_GET` of keys 1-19, every measurement). Re-run sections 1 and 3 after any firmware or mechanical change.
- Meters: a DMM, a scope (or a logic analyser with at least 4 channels), a clamp meter or current probe, a chronograph or a phone with a 240 fps slow-motion mode, a laser
  pointer, a steel ruler and a tape measure, a mannequin or a life-size poster of a person.

## 1. Firmware bring-up (weapon rail dead)

### 1.1 USB enumeration and protocol
Plug the Pico into the Jetson (or the laptop). `ls /dev/serial/by-id/` shows `usb-Raspberry_Pi_Pico_*`. Run `python -m napl.lite.app --gimbal <path>` or a 10-line HELLO script.
Count STATUS frames over 10 s; leave it 10 minutes and read the bad-frame counter (`EVENT 8`).
**Pass:** HELLO_ACK with the right uid and `major.minor = 1.0`; STATUS 100 Hz +-5 % (950-1050 frames in 10 s); 0 bad frames in 10 minutes; unplug and replug: link recovers, state SAFE,
never ARMED. **Fail:** any other. **Feed back:** nothing (if STATUS is below 100 Hz, set the twin's `STATUS_PERIOD` to the measured value).

### 1.2 TMC2209 read-back and current set by meter
Power the drivers from the bench supply at 12 V, motors connected, free to turn. After HELLO read the DEBUG frame: `tmc pan=0 tilt=0` (0 = configured and read back; negative =
no reply: check the 1 kohm and the single-wire node, the address strap MS1 = MS2 = VIO = address 3, and that pan is on GP20/21 and tilt on GP0/1).
Measure the motor phase current with a clamp meter or a current probe on one phase wire while the axis turns at 20 deg/s (RMS, pan and tilt each), and at standstill (hold).
Confirm 16 microsteps: one commanded turn of the output axis is 16,000 steps (pan) / 12,800 (tilt); mark the axis, command +360 deg, the mark returns within 1 deg.
**Pass:** both DEBUG values 0; run current 1.2 A RMS +-10 % (1.08-1.32 A) and hold 0.6 A +-15 %; mark returns within 1 deg per 360 deg; the motors are free (no EN glitch) while the
Pico is held in reset. **Fail:** current outside 1.0-1.45 A is a fail (the motor is rated 1.7 A, spec 6.3). **Feed back:** if the measured current needs a different CS, change
`TMC_IRUN_CS` and re-check the torque margin in spec 6.3; steps per rev (cfg 1, 2) if the measured turn is not 360 deg (belt teeth count).

### 1.3 Hall polarity and homing
With a magnet at each A3144, STATUS `flags1` b4/b5 follow it. Home each axis from three starts: the negative side, the positive side and with the magnet under the sensor
(spec 2.3). Do 20 homings per axis (alternate starts) and record the joint angle at which each finishes (laser spot on the grid, 1.3 below, or an angle dial).
**Pass:** b4/b5 = 1 only with the magnet in place (active-low sensor read correctly); HOMED within 8 s (pan) / 3 s (tilt) from every start; no `homing_timeout`; the 20 final angles
span at most 0.1 deg; calibrate cfg 7/8 so the home position is the window centre, `CFG_SAVE`, power-cycle, `CFG_GET` returns them. **Fail:** span > 0.2 deg (magnet or sensor loose,
or a hysteresis issue), or a start that does not home. **Feed back:** `q_hall` of the assembly (the CAD magnet position) and the hall half width (`HALL_HALF_WIDTH_DEG` in `sim_mcu.py`) to the
measured window; measured home-position span becomes the repeatability term of the pointing error budget (1.3).

### 1.4 E-stop, key and the interlock (weapon rail measured dead)
Rail live for this test but flywheel gates, servo and motors **disconnected**; a DMM (or scope) on the weapon rail at the MOSFET drains.
(a) E-stop pressed: rail < 0.5 V within 20 ms of the contact opening; STATUS shows `estop_pressed` and state SAFE (reason 3) within 50 ms; homed flags cleared; `FLYWHEEL` is refused (ACK
INTERLOCK); `ARM` refused. (b) Key off: same with reason 4. (c) Unplug the E-stop sense wire and the key sense wire in turn: each reads "pressed"/"off" (fail safe).
(d) E-stop released: no motion, no arming until the 1 s of heartbeats and a `HOME`. (e) With the key on and the E-stop closed, rail reads the supply voltage and falls to 0 V within
20 ms of either switch, **with the Pico unplugged** (the cut is hardware, not firmware).
**Pass:** all five. **Fail:** any rail voltage above 0.5 V after 50 ms, or any path that leaves the rail live with a switch open: stop, fix the wiring, do not proceed to section 3.
**Feed back:** the twin's reaction time (`SUB`/tick and the harness `estop` delay) if the measured SAFE latency exceeds 5 ms.

### 1.5 Watchdog trip (scope or LED)
Scope on GP12 (flywheel PWM) and a second channel on the last heartbeat (or the green LED). Flywheel gates connected to a dummy load (lamp or resistor), rail live. Arm, request
the flywheel at 300 permille, then pull the USB cable.
**Pass:** PWM at 0 within 300-320 ms of the last heartbeat; the red LED solid (SAFE); after replug, SAFE then IDLE only after 1 s of heartbeats, never ARMED; the hardware
watchdog (a `while(1)` debug build) resets the board in 8 s +-1 s into SAFE. **Fail:** PWM still on after 350 ms. **Feed back:** `wd_ms` (cfg 18) if you shorten it.

### 1.6 Step generator and soft limits
Scope STEP at AIM speeds 20, 60, 120, 240 deg/s: 20 us pulses, even spacing; 100 round trips of +-90 deg (the mark returns, no drift); soft limits hold at AIM beyond them.
**Pass:** no drift above 0.1 deg after 100 round trips at 240 deg/s pan / 150 deg/s tilt; no stall at vmax; stops are outside the soft limits. **Feed back:** `vmax`/`amax` caps of the
app (`AppCfg`) to the highest speed and acceleration that passed with 25 % margin, and the pull-in curve assumption (spec Q7).

## 2. Mechanical

### 2.1 Belt tension and backlash
Tension each GT2 belt until it does not skip at 1.2 x the app's `amax` (pan 2000, tilt 1500 deg/s^2) and the deflection under a fixed load (a 1 N weight at mid-span) is the same on the
two belts; record the deflection. Measure backlash per axis: laser on the barrel, grid at 5 m (1 mm = 0.0115 deg). Command angle A approached from below, note the spot; command A
approached from above, note the spot; backlash = the difference. Repeat at 5 positions per axis, with the head pitched at 0 and 35 deg for the tilt.
**Pass:** no skipped teeth in 100 round trips at 240 / 150 deg/s; backlash <= 0.2 deg per axis (a third of the +-0.61 deg tolerance at 15 m, spec 5.3); belts do not sing or ride off the flange.
**Fail:** backlash > 0.3 deg: tighten, re-seat the pulleys, or shim; if it stays, plan the sim run with the measured value. **Feed back:** the measured backlash (rad) into the pan/tilt
joint device's `backlash` field (the servo part file's `backlash`, which feeds the kernel `backlash_joint`; spec 4.2 / `src/mujoco/devices`), one value per axis, and into the
pointing-error budget; re-run `examples/cuas-lite` s1 and the Monte Carlo to see the hit-rate change.

### 2.2 Pointing accuracy
Laser pointer rigidly on the barrel (or the muzzle axis, not the camera). Target grid on the wall at 5 m, 1 cm squares, with the pan/tilt zero marked. Command 25 angles (a 5 x 5 grid
of pan -20..20, tilt 5..25 deg), each approached from the negative side, and record the spot position; then 100 random moves and a return to the zero. Convert spot offsets to degrees.
**Pass:** RMS error <= 0.15 deg, worst case <= 0.3 deg; return-to-zero spread <= 0.1 deg after 100 moves. **Fail:** RMS > 0.25 deg (the tolerance budget at 10 m is +-0.92 deg for a balloon and
the camera, ranging and ballistics errors are added on top). **Feed back:** a constant error is a home offset (cfg 7/8) or a mount tilt (CAD); a scale error is steps per rev (cfg 1/2);
the RMS value is the commanded-vs-actual noise to add to the sim's joint (backlash and `status_from_joint` noise), and sets `T_ACTUATOR` if the move time (step response to 5 deg) is
not about 60 ms.

## 3. Launcher

### 3.1 Muzzle velocity at each duty
Chronograph at 1 m from the muzzle (or a phone at 240 fps with a ruler in frame: v = displacement per frame x 240; 30 m/s is 0.125 m per frame, so use two frames and a 1 m
ruler). Flywheel duties 400, 500, 600, 700, 800, 900 permille (the app default is 800); allow `spinup_ms` (700 ms) and a settled supply. 10 balls per duty, firing singly then
bursts of 3. Record v0 for every ball and the pack voltage.
**Pass:** the v0 vs duty curve is monotonic; at 800 permille the mean is 30 +-3 m/s [est] and sigma <= 2 m/s; the maximum measured v0 <= 37 m/s (the toy-class bound of spec 5.3);
burst balls 2 and 3 within 2 m/s of ball 1. **Fail:** sigma > 3 m/s (inconsistent feed or a wheel slipping), or any ball above 37 m/s (cap the duty in the app). **Feed back:**
`k_v` = mean v0 at 800 / 30 into `calib.toml` (`aim.pellet(k_v)` uses `k_v * V0_NOMINAL`), `V0_SIGMA` in `aim.py` = the measured sigma, and `spinup_ms` (cfg 16) = the time for the
flywheel to reach steady speed (scope PWM plus the chronograph on the first ball after 300, 500, 700 ms). Check that the sim's `set_flywheel` ramp (4 permille/ms) matches the motor
if you add a tach.

### 3.2 Dispersion at 3, 5 and 10 m
Barrel fixed (clamped, motors disabled), flywheel at the operating duty. Target sheet with 1 cm grid at 3, 5 and 10 m. 20 balls per distance; record every impact (x right, y up)
in metres relative to the aim point; compute mean offset and sigma per axis, convert to mrad (sigma / range).
**Pass:** sigma <= 12 mrad per axis at every range [est] (the sim assumes 10 mrad; a balloon of 0.161 m radius at 10 m is a 16 mrad radius, so 12 mrad gives about 60 % hit chance
per ball and 10 mrad about 72 %); the mean offset the same at the three ranges within 1 sigma (it is a boresight, not a range-dependent effect). **Fail:** sigma > 15 mrad
(the launcher is the weak link: check wheel alignment, the barrel fit and the ball seat) or sigma growing with range faster than linearly (ball tumbling). **Feed back:**
`SPREAD_SIGMA` in `aim.py` = the measured sigma in rad; the mean offset goes to the boresight (4.2).

### 3.3 Drag (Cd) from the drop
Horizontal barrel (check with a level) at a known height, firing at a vertical sheet at 5 m and at 10 m; measure the vertical drop against the muzzle line and the time of flight from
the video. Fit `Cd` by integrating the same ODE as `solve_lead` (`aim.pellet` with a trial `cd`) so that the predicted drop at both ranges matches; the flight time check is the 240 fps video.
**Pass:** a single Cd in 0.4-0.55 reproduces the drop at 5 and 10 m within 3 cm, and the flight time within 10 %; **Fail:** no single Cd fits both (the ball is not a sphere in the model,
e.g. spinning with Magnus lift: note the backspin direction and re-test with the wheels' ratio changed). **Feed back:** `BALL["cd"]` in `aim.py`; weigh 20 balls and measure the diameter
(callipers) and put the means in `BALL["m"]` and `BALL["d"]` (Q4: the CAD regenerates the barrel ID, hopper and tolerances from the same `BALL`).

### 3.4 Pusher timing and hopper feed at -15, 0, 35, 70 deg
Hopper loaded with 50 balls. The tilt is set to each of -15, 0, 35 and 70 deg (the soft limits, spec 6.1) by hand or with AIM, flywheels at the operating duty, the barrel pointed at the
barrier. At each angle fire the whole hopper (50 balls: 20 singles, then 10 bursts of 3), reloading between angles. Record the jams, double-feeds, misfeeds and the
`EVENT 2` (SHOT) count against the balls that actually left. Measure the FIRE-to-ball-leaves time on video (the pusher extend dwell, cfg 14).
**Pass:** 0 jams, 0 double-feeds in 50 balls at every angle, SHOT events equal balls fired (shot counter honest), FIRE-to-muzzle <= 170 ms [est] (spec: 120 ms + 30 ms).
**Fail:** any jam at 0-35 deg; a jam at -15 or 70 deg only is a limit: tighten the soft limit (cfg 3-6) and record it. **Feed back:** the measured delay
replaces the twin's (`PUSHER_DWELL_MS`, cfg 14, and the 0.122 s FIRE-to-feed that `examples/cuas-lite/latency.py` measures in the sim). **Bench calibration of the lead:** the app defaults are
`T_ACTUATOR = 0.02 s`, `T_FIRE = 0` in `aim.py`, fitted in the sim where FIRE-to-ball is 0.122 s (not simply set to it: the AIM is re-solved continuously with feed-forward, and summing the
pusher delay and the head lag led the ball 0.15-0.3 m ahead of a 1 m/s target). On the bench, re-run the sweep with the measured numbers: fire at a target on a pendulum or a rail at
a known speed (0.5-1.5 m/s) with `mc.py --fire-cfg '{"t_fire": ..., "t_actuator": ...}'` in the sim and a few lead offsets on the hardware, and keep the total that centres the impacts.
Also `PUSHER_DWELL_MS` (cfg 14) and `min_shot_interval_ms` (cfg 15), the soft limits of the tilt, and the fire-tilt-min (cfg 17) if the low angle is unreliable.

## 4. Camera: calibration and boresight

### 4.1 Chessboard intrinsics
Camera at the used mode (1280 x 720 @ 30 default, spec 5.2), exposure and focus fixed as in `calib.toml`. 15 views of a printed chessboard (flat on a rigid board) covering the corners
of the frame and tilts up to 30 deg. OpenCV `calibrateCamera`.
**Pass:** RMS reprojection error < 0.5 px; fx within 5 % of the 736 px prior; distortion small enough that the corner residual < 1 px. **Fail:** RMS > 0.8 px: re-shoot with a flatter
board. **Feed back:** `calib.toml` intrinsics (the app refuses to ARM without a fresh file); the sim's `Camera` fov if fx differs by more than 3 %.

### 4.2 Boresight
Per spec 5.2: indoors, balls loaded, flywheels at the operating duty; fire 5 balls each at a 0.3 m target board at 5 m and at 8 m with the app's lead solution off (boards static);
mark impacts; the tool (`python -m napl.lite.calib boresight`) or a hand calculation solves the camera-to-barrel offset `(d_az, d_el)` and `k_v`.
**Pass:** after applying the offsets the mean impact is within 0.3 deg of the aim point at both ranges; `k_v` within 0.9-1.1 (1.0 = the 30 m/s nominal, agreeing with 3.1 within 5 %); the two solutions (5 m and 8 m) agree within 0.3 deg.
**Fail:** the offset changes with range by more than 0.5 deg (the camera-to-barrel parallax `d_c` in the CAD is wrong). **Feed back:** `calib.toml` (`d_az`, `d_el`, `k_v`); the
CAD constant `d_c` if the parallax disagrees; re-run the sim with the measured offset as a bias to see its effect.

## 5. Detector

### 5.1 Balloons and hanging targets
Targets: a 0.30 m balloon and a hanging target of the app's class sizes (`CLASS_SIZE_M`), at 3, 5, 8, 10 and 15 m, static and swung (a pendulum as in s1), indoor light at
two levels and a backlit window scene. 1000 frames per condition. Log the detector replies per second, the per-frame recall, the false positives on the empty scene (5 minutes), the
confirmed-track time (5 consecutive replies) and the size-ranging error against the tape measure.
**Pass:** >= 30 detector replies/s on the Jetson (Q3); recall >= 95 % at <= 8 m and >= 85 % at 10 m (15 m: report only); 0 false-positive confirmed tracks in 5 minutes of empty scene;
range error <= 15 % at 10 m (spec 0 item 2 says 10-25 %). **Fail:** recall < 85 % at 8 m or any confirmed false positive in the firing sector. **Feed back:** `CLASS_SIZE_M` to the
measured apparent size per class, `MIN_CONF` and the per-class threshold to the point where the false-positive rate is 0 with recall as above, the camera latency (`cam_latency` 0.030 s
[est]; measure it with an LED flashed at a known time) and the detector's round trip, which add to `T_ACTUATOR`; the TruthDetector's miss and jitter model in the sim to the measured
recall and box error.

### 5.2 Person block
A mannequin (or a life-size poster of a person, 1.7 m) standing at 3, 5 and 10 m, in the sector, front and sideways, in two clothing contrasts, lit as in 5.1. With the flywheels
**off** and the pusher unloaded first: ARM with a balloon visible (a) alone, (b) beside the person at 0.5 m, (c) in front of the person, (d) behind the person (partial occlusion), (e)
with the person walking into the frame of a visible balloon. Then repeat with the weapon rail live and the barrel pointed at the barrier, balls loaded, but only after the dry runs
all passed.
**Pass:** zero FIRE commands while the person block is active (the block holds in at most 1 detector frame after the person appears and for the configured hold after it leaves); the
mannequin is detected as `person` in >= 95 % of frames at <= 10 m in every pose; (a) still fires. **Fail:** any shot that crosses the person's silhouette or a FIRE while a person
is visible in the frame is a **hard fail**: stop, do not reduce the threshold to pass, fix the detector (more person data, a lower person threshold, a larger block margin) and repeat
5.1 and 5.2. **Feed back:** the detector's person threshold (0.15 default) and the block margin and hold times in `AppCfg`, and the sim's person-injection scenarios (their miss rate
set to 1 - the measured recall).

## 6. Closing the loop: parameters to update from the measurements

| measurement (section) | sim / firmware parameter | where | then |
|---|---|---|---|
| ball mass, diameter, Cd (3.3) | `BALL = dict(d, m, cd)` | `python/napl/lite/aim.py`; CAD derives barrel ID, hopper from it | rebuild CAD, re-run unit tests of `solve_lead` |
| muzzle velocity at duty 800 (3.1) | `k_v` (calib.toml), `V0_NOMINAL`, `V0_SIGMA` | `aim.py`, `~/.config/cuas-lite/calib.toml` | re-run the elevation table (spec 5.3) |
| dispersion (3.2) | `SPREAD_SIGMA` (rad per axis) | `aim.py` (used by `build_scene` as `spread_std`) | re-run the Monte Carlo hit rates |
| belt backlash (2.1), pointing RMS (2.2) | `backlash` (rad, per axis), joint noise | the pan/tilt servo part file / assembly `backlash_joint`; `SimRP2040` joint noise | re-run s1-s7 and the MC |
| FIRE-to-muzzle, move response, camera latency (2.2, 3.4, 5.1) | `T_FIRE`, `T_ACTUATOR`, `cam_latency` | `aim.py`, `camera_v4l2.py` | lead solution timing |
| spin-up time (3.1) | `spinup_ms` (cfg 16) and the ramp (4 permille/ms) | firmware config, `FW_RAMP_PER_MS` in `sim_mcu.py` + `fw_core` | `CFG_SAVE` |
| hall window, home offsets, repeatability (1.3) | cfg 7/8, `q_hall`, `HALL_HALF_WIDTH_DEG` | firmware config, `sim_mcu.py` | `CFG_SAVE` |
| steps per rev, speeds, accelerations (1.2, 1.6) | cfg 1/2, `vmax`/`amax` caps | firmware config, `AppCfg` | |
| class sizes, thresholds, recall (5.1, 5.2) | `CLASS_SIZE_M`, `MIN_CONF`, person threshold; truth-detector miss model | `aim.py`, detector repo, sim scenario | re-run the person scenarios |

After updating the parameters, re-run `napl program examples/cuas-lite/s1_balloon_static.scenario.toml` (and the Monte Carlo of spec 7.3): the new hit rates are the first
prediction made with measured numbers; compare them with the bench hit rate of 5.1 on a real balloon, and log the difference as the sim-to-real gap.
