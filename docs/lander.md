# TVC propulsive-landing model rocket (examples/lander)

A 1.45 kg civilian hobby rocket that lifts off on an Estes E16, flies a guided hop on a two-axis servo thrust-vector gimbal, coasts, and lights a second
motor (Estes F15) at a computed moment to stop on its four legs: the kind of vehicle BPS.space's Scout and the Grasshopper / New Shepard hops stand for,
at model-rocket scale. Everything is simulated in napl: the physics devices of the assembly, a seeded world around them, and a flight computer that reads
only simulated sensors.

| What | Where |
|---|---|
| Vehicle | `assemblies/tvc-lander.toml` (+ `parts/rocket/*-140`, `tvc-gimbal-2x-*`, `generic-lander-leg-skid`, `generic-printed-tail-fins-140`, `parts/electronics/pjrc-teensy-4-0`, `generic-avionics-harness`, `parts/propulsion/tattu-2s-450mah-75c-jst-syp`, `parts/rocket/estes-2302-starter`) |
| Flight software | `examples/lander/flight.py` (state machine, control, guidance), `estimator.py`, `guidance.py`, `vehicle.py` (firmware constants) |
| World + harness | `examples/lander/land.py` (owns the Sim, injects the scatter), `scene.py` |
| Scenario | `examples/lander/land.scenario.toml`: `napl program examples/lander/land.scenario.toml --runs-dir runs` |
| Monte Carlo | `examples/lander/mc.py` |
| Tests | `tests/python/test_lander.py` |
| Parameters & telemetry facility | `python/napl/params.py` (generic: any scenario may declare `[[parameters]]` and `[[telemetry]]`) |

## 1. Vehicle

`napl design check assemblies/tvc-lander.toml`: 0 errors, 0 warnings. `tools/check_parts.py`: 0 problems. Mass report (both motors loaded, 1.4505 kg, under the
1.5 kg hobby-model limit; CG 0.399 m above the feet, 0.252 m above the tube's aft end; Ixx = Iyy = 0.0879 kg m2, Izz = 0.0042 kg m2):

| Part | Mass | Provenance |
|---|---|---|
| Lead nose weight (60 mm disc x 16 mm) | 520 g | estimated; sized so the vehicle weighs 1.45 kg (section 3) |
| 5.5 in fiberglass tube, 0.40 m, 1 mm wall | 314 g | estimated (1800 kg/m3 laminate); not a catalogue part |
| Estes F15 landing motor (49.6 Ns, 3.5 s) | 103 g | thrustcurve.org / NAR cert data, `parts/rocket/estes-f15` |
| Printed ogive nose, 0.25 m | 90 g | derived from the shell volume |
| Estes E16 ascent motor (28 Ns class) | 82 g | thrustcurve.org cert data |
| Gimbal: cradle 43.7 + plate 42.3 + ring 28.3 g | 114 g | derived from the PLA volumes (1240 kg/m3 x 0.95, Ingeo 4043D data sheet) |
| 2 x ROBOTIS XL330-M288-T servos | 36 g | datasheet |
| Tail fins (4 x 9.3 g PLA) | 37 g | derived |
| Avionics harness incl. GNSS, regulator deck, wires | 30 g | estimated allowance |
| Tattu 2S 450 mAh 75C LiPo | 29 g | datasheet |
| 4 legs (10 mm carbon tube + printed foot) | 81 g | derived (`cf-tube-10x8` 45.6 g/m) |
| Teensy 4.0 + ICM-42688-P + MS5611 | 3 g (chips) | datasheet |
| Pololu D24V50F5 regulator | 5 g | estimated (not published) |

Sensors and actuators, all real parts with datasheet numbers: ICM-42688-P IMU (gyro noise, ZRO 0.5 dps, accel noise; 500 Hz), MS5611 barometer (1.2 Pa
RMS, 50 Hz), a NEO-M8N-class GNSS (5 Hz, 0.2 s latency, 0.05 m/s velocity, 2.5 m CEP, slow Gauss-Markov position error), the 2S pack and regulator with their
internal resistance, two XL330-M288-T servos (P/D gains set by the flight computer; the device models the gear train's torque limit, the 5 V stall torque
0.52 N m, velocity limit and electrical limits), a logic-level pyro FET per motor through an Estes 2302 starter. The gimbal moves the motor cluster +-8 deg
(mechanical clearance of the ring), the software stops 1 deg inside. Gimbal loads (`SERVO_TORQUE` design rule): 0.011 N m static gravity torque and about
0.28 N m peak from the off-axis motor against 0.52 N m stall.

**Parts without a `[visual]`** (new, not drawn): `generic-fiberglass-tube-140`, `generic-printed-ogive-nose-140`, `generic-lead-nose-weight-140`,
`tvc-gimbal-2x-plate`, `tvc-gimbal-2x-ring`, `tvc-gimbal-2x-cradle`, `pjrc-teensy-4-0`, `tattu-2s-450mah-75c-jst-syp`, `estes-2302-starter`,
`generic-avionics-harness`. (`generic-printed-tail-fins-140` and `generic-lander-leg-skid` reuse the `fin_set` and `landing_leg` generators of `visuals/gens_rocket.py`.)

### Why two motors, and the timing scheme

A solid motor cannot be throttled or shut off, so a propulsive landing with one is a *timing* problem. The design came out of 1-D Monte Carlo of the whole
problem (impulse scatter, igniter delay, state-estimate error, hardest-contact inelastic floor, `examples/lander/guidance.py`) over motors and masses:

* The velocity change a landing motor can give is about `I/M - g t_burn`. A motor with thrust far above the weight (E16/G40 as a lander) stops the vehicle in
  a window of 0.1-0.3 m of ignition height: a knife edge no barometer-and-GNSS estimate resolves.
* A motor whose thrust is about the vehicle's weight (**near-hover regime**: the F15's 13-14.6 N tail against a 14.2 N weight) has windows of a metre or more,
  but then the vehicle must be heavy enough that the motor cannot lift it again after the stop (the floor is inelastic: a hover that ends above the ground
  falls back, and the *hardest* contact of the burn is the one that counts, not the first): at 1.43-1.47 kg the odds are best. That is why there is a 520 g lead
  slug in the nose: with the bare 0.85 kg airframe the hop reaches ~30 m and returns at ~17 m/s, faster than any motor of this class can stop.
* The E16 (28 Ns, 1.6 s) is the ascent: a ~7-9 m hop at T/W 1.2, so the vehicle returns at ~4 m/s, inside the F15's capability.
* The landing motor is **lit before the apogee**: its igniter + motor delay (0.3 s curve delay + the pyro channel's ~0.1 s, learned from the ascent motor's
  measured delay) means it must be commanded while the vehicle is still at the top of the hop, so that thrust starts at an onset speed of about -3.5 to -4.5 m/s.
  The rule (`guidance.Planner`): predict the height and speed at which thrust will start, `delay` seconds from now (ballistic with drag), and fire when the
  predicted onset height is at or below the table height for that onset speed (`ignition_table`: vectorised forward integration of the vertical burn with the
  certified curve, per onset speed the middle of the lowest window whose hardest touchdown is within `max(0.8 m/s, best + 0.3)`), plus one tunable `aim_bias_m`.

1-D result at the chosen design point (3 000 flights, `tools/` scratch script `mc1f`): P(touchdown < 2 m/s) = 0.33, P(< 3) = 0.57, P(< 5) = 0.76,
median 2.6 m/s with the scatter below; 98 % under 2 m/s with none. The three-dimensional Monte Carlo in section 6 is the claim that matters.

### Why tail fins

The finless version is aerodynamically unstable nose-first (Barrowman nose CP 0.54 m above the aft end, CG 0.25 m above it): with a 1.5 m/s wind and the ~1.2 s
unpowered coast between the ascent burnout and the landing motor's thrust, the weathercock torque turned it by 20-35 deg before the landing motor lit, and the
landing burn's sideways thrust then threw the vehicle 3-5 m off the pad. Four small tail fins (Cr 0.10, Ct 0.05, span 0.07 m, 37 g, `barrowman.fins`: CNa 2.50)
bring the combined centre of pressure to 0.2707 m, within 2 cm of the CG, i.e. nearly neutral both nose-first and tail-first. They do not stabilise the
vehicle; attitude is still thrust vectoring. The same 12 seeds that landed with 5 m of drift and ascent tilts of 8-14 deg land within 1.8 m with ascent tilts
of 2-4 deg after this change.

### Legs

Four fixed splayed 10 mm carbon tubes, feet at radius 0.155 m, 20 deg from the axis. A sliding vehicle overturns when the foot friction coefficient exceeds
`R / h` (0.155 / 0.399 = 0.39; in the diagonal direction between two feet 0.28), and a grippy TPU ball foot (0.8) tripped the vehicle whenever it was still
moving sideways at the end of the burn. The lander's feet are hard printed pads with `mu = 0.3` (estimated: hard plastic on dry concrete/asphalt, between
the 0.2-0.5 range of hard polymers and Engineering ToolBox's 0.30-0.70 for metal on stone; a rougher ground would bring the tripping back). Contact softness:
MuJoCo `solref = [0.02, 1]` per foot (stiffness ~1.4e3 N/m, critically damped).

Drop test of the unpowered vehicle (peak accel is the ICM-42688-P's +-16 g full scale, so it clips; rebound and rest tilt are from the sim):

| Impact speed | upright drop | dropped 8 deg tilted |
|---|---|---|
| 1 m/s | stands, 0.2 cm rebound | stands, 1.2 cm |
| 2 m/s | stands | stands |
| 3 m/s | stands | stands, 2.5 deg rest tilt, 3 cm rebound |
| 5 m/s | stands | falls over (98 deg), 7 cm rebound |
| 7 m/s | stands | falls over |

so a landing under ~2 m/s with the allowed tilt survives and above ~5 m/s tilted one overturns; the monitors register `touchdown_speed` at 2 m/s and
`no_hard_landing` at 5 m/s. (`invweight0` / `meaninertia` are not refreshed after the motors' mass changes, `docs/models.md`; the leg contact softness
was tuned with the full vehicle mass at rest, and the effect of the stale values on a 1.45 kg vehicle that loses 0.1 kg is small compared with the foot softness.)

## 2. Physics fidelity (all seeded)

Every random quantity is drawn from `numpy` PCG64 seeded `[seed, 5150]` in a fixed order (`land.draw_hidden`), or lives inside the sim (sensor noise, Dryden
turbulence: seeded by the model build `scene.build_seeded`). The same (seed, parameters) replays to the same state hash.

| Effect | Model | Value | Source |
|---|---|---|---|
| Motor impulse scatter | per-motor `thrust_scale`, normal, clipped at 3 sigma | sigma 3 % (swept 2 / 3 / 6.67 %) | NFPA 1125 as quoted on thrustcurve.org/info/certification.html: "The total impulse must not have a standard deviation greater than 6.7%"; model-motor absolute limit +-20 % (R. Krech, Rocketry Forum, 2004, quoting NFPA 1125); the same page: "most modern manufacturers hold tighter tolerances" |
| Ignition delay | the motor device's delay is a constant, so the harness delays the `ignite` command: mean 0.10 s + normal jitter | sigma 0.04 s (swept 0 / 0.02 / 0.08) | estimated; NFPA 1125 allows +-1 s on *ejection* delays only, no figure for ignition |
| Thrust misalignment | constant per-flight offset of each gimbal axis against what the servo reports | sigma 0.3 deg | estimated (horn zero, nozzle seat) |
| Servo lag, rate limit | the `servo` device (position loop, P=2000 D=6000, torque limit, velocity limit) | XL330-M288-T datasheet | `parts/arm/robotis-xl330-m288-t` |
| Servo backlash | play operator on both goals, half width 0.25 deg (total 0.5 deg) | `backlash_deg` | not published for the XL330-M288-T; bound of the comparable Feetech STS3215 (<= 0.5 deg) |
| Wind | steady wind from a random direction + Dryden turbulence (the `environment` device, `w20` = the mean wind) | 0-4 m/s in the Monte Carlo | scenario parameter |
| IMU | noise densities and zero-rate offset of the ICM-42688-P; per-flight bias draws (gyro 0.25 dps, accel 0.1 m/s2, 1 sigma) | datasheet | `parts/drone/tdk-icm-42688-p` |
| Baro | MS5611 1.2 Pa RMS noise, 50 Hz, quantised | datasheet | `parts/drone/te-ms5611-01ba03` |
| GNSS | 5 Hz, 0.2 s latency, position error slow Gauss-Markov, 0.05 m/s velocity | NEO-M8N datasheet | `parts/drone/ublox-neo-m8n`, `assemblies/x500v2.toml` pattern |

Not modelled: roll authority (the two-axis gimbal cannot torque about z; roll is left to its small natural damping), motor casing temperature, thrust variation
with ambient pressure, exhaust momentum, wind shear above the hop, structural flexibility of the legs and tube, the servo's own firmware limits other than
its torque and velocity limits.

## 3. Flight software (`examples/lander/flight.py`)

Runs at 250 Hz on sensor samples only: gyro, accelerometer, baro pressure (when a new sample arrives), GNSS fix (when a new one arrives).

States: `PAD` (2 s calibration: gyro bias, up vector from the mean accelerometer, pad pressure and GNSS reference) -> `PAD_READY` -> `BOOST` (ascent motor lit
at t = 3 s) -> `COAST` (burnout detected on the accelerometer) -> `APOGEE` -> `DESCENT` -> `LANDING_BURN` -> `TOUCHDOWN`; `ABORT` if the attitude estimate
exceeds 35 deg under power or the landing motor cannot be lit safely (> 60 deg off vertical). The landing rule is evaluated from the burnout on (`COAST`,
`DESCENT`) because the motor must be commanded before the apogee.

* **Attitude**: quaternion integrated from the bias-corrected gyro; a Mahony tilt correction only while the accelerometer reads 1 g +- 15 % (never under thrust or in
  free fall), i.e. gyro-only in flight. Unit test: 2 deg tilt and 5e-3 rad/s bias found by the calibration; 0.2 rad/s rotation under 3 g tracked to 0.3 deg; 10 s
  of gyro-only flight within 0.3 deg.
* **Position / velocity**: one 3-state Kalman filter per world axis (position, velocity, accelerometer bias) propagated with the rotated specific force, corrected
  by the barometer (z) and GNSS position / velocity (x, y) with delayed-measurement compensation (0.2 s latency).
* **TVC attitude control**: cascade with natural frequency 13 rad/s, damping 0.8; torque command `I * alpha` converted to gimbal angle by `torque / (L * T)`
  with `T` the thrust **measured on the accelerometer** (gain scheduling over the 10-25 N thrust range and the changing motor), plus the motor seat's trim as a
  feed-forward (the two motors sit on the diagonal 15 mm off the gimbal axis) and an integrator whose gain is scaled by `T_NOM / T` with anti-windup; the thrust
  tail after burnout detection keeps the trim until the curve ends.
* **Ascent guidance** keeps the vehicle vertical and fights wind: a PD law on estimated position and velocity (0.35 / 1.1) limited to 8 deg of lean.
  Near the ground in the landing burn the position hold fades out and sideways-speed damping fades in (`z_slow`, `kd_land`): a foot that slides trips the vehicle.
* **Landing ignition timing**: section 1. The planner table is built once (cached) from the motor curve; `aim_bias_m` shifts it. On the ground, the attitude
  correction is held to `td_gimbal_deg` (2.5 deg) so the almost-weightless feet are not pushed along the ground while the F15's tail still burns.

## 4. Scenario and monitors (`land.scenario.toml`)

`napl program examples/lander/land.scenario.toml`: native interface, the program (`land.py`) owns the Sim and writes `trace.csv`, `replay.naplog`, `native.json` and
`telemetry.jsonl`; the runner judges the monitors on the trace and re-runs the log for the state hash. Monitors: `stands_on_the_pad` (tilt < 0.5 deg before
launch), `lifts_off` (z reaches 4 m), `flight_tilt_limit` (35 deg), `touchdown_speed` (< 2 m/s, the hardest ground contact of the flight), `no_hard_landing`
(< 5 m/s), `touchdown_tilt` (< 10 deg), `lands_near_the_pad` (X = 3 m), `stays_upright` (< 15 deg two seconds after touchdown), `servo_saturation`
(Y = 10 % of the powered time with a gimbal goal at the software limit).

## 5. Parameters & telemetry contract (UI / panel)

Generic facility in `python/napl/params.py`, additive to any scenario:

```toml
[[parameters]]                     # one knob
name = "wind_ms"  type = "float"   # float | int | bool | choice
default = 2.0  min = 0.0  max = 8.0  unit = "m/s"  group = "environment"  label = "Mean wind"  doc = "tooltip"
[[telemetry]]                      # one channel of the stream
name = "est.alt_m"  unit = "m"  label = "Altitude (estimate)"  group = "state"  kind = "number"   # number | string | bool
```

* `napl.params.describe(scenario)` -> `{"parameters": [{name, type, default, min?, max?, choices?, unit, group, label, doc}...], "telemetry": [{name, unit, label,
  group, kind, min?, max?, doc}...]}`; on the command line `python -m napl.params describe <scenario.toml>`.
* A run takes `NAPL_PARAMS='{"wind_ms": 3.5, "seed": 7}'` (JSON object; unknown names and out-of-range values are errors, `ParamError`); the program reads them back
  with `napl.params.values(scenario)`. 21 parameters for the lander (seed, wind, scatter widths, backlash, controller gains, aim bias ...).
* The program streams `<run dir>/telemetry.jsonl`, one compact JSON object per line (flushed per record, every 48 ms of sim time): `{"t": <sim s>, "state": "...",
  "est": {...}, "gimbal": {...}, "thrust_N": ..., "motors": {"a": {"phase", "thrust_N"}, "b": {...}}, "landing": {...}, "truth": {...}}`. `napl.params.get(record,
  "est.alt_m")` resolves a dotted channel; `channel_series(records, name)` gives `[(t, value)]`; `python -m napl.params table <telemetry.jsonl> est.alt_m,state`
  prints a text table. `TelemetryWriter` accepts callbacks for a live panel.
* The 3-D viewer panel is not built (it would be C++ in `src/viewer`, outside this task's scope); a panel needs only the contract above: `describe` to build the
  controls and the plots, `NAPL_PARAMS` to start a run, `telemetry.jsonl` to follow it.

## 6. Monte Carlo (`examples/lander/mc.py`) and results

Protocol (the point of the two seed ranges): (1) *training* seeds 1000... choose the one tunable, the landing-ignition aim bias, from a list; (2) the *report*
seeds 5000... are flown once with that bias, and the numbers below are whatever came out; (3) the same report seeds are re-flown with the igniter jitter,
the impulse scatter and the wind changed, for the sensitivity table. Each flight draws its own wind (speed uniform 0-4 m/s, direction uniform). A flight is a
success when it lands (not aborted) with touchdown speed < 2 m/s, touchdown tilt < 10 deg, within 3 m of the pad, upright at the end, servo saturation < 10 %.

Command: `mc.py --train 48 --test 300 --workers 6 --aims=-0.3,0,0.3,0.6,1.0` (636 s wall on hfbox). Training (48 seeds per candidate, aim in m: success / v<2):
-0.3: 31 % / 40 %, 0.0: 29 % / 33 %, +0.3: 27 % / 35 %, +0.6: 25 % / 31 %, +1.0: 27 % / 33 %. The five are inside the binomial noise (+-6 %) of each other: the
bias is picked (-0.3, the default of the scenario) but it is not a tuned advantage. **Report seeds 5000-5299, flown once: success 26.0 % (soft landing inside every
criterion)**; touchdown speed < 2 m/s 35 %, < 5 m/s 86 %; tilt < 10 deg 98 %; within 3 m 89 %; upright 68 %; aborts 0 %. Touchdown speed p50/p90/p99 =
2.43 / 6.13 / 9.06 m/s; distance from the pad p50/p90/p99 = 1.40 / 3.09 / 6.02 m.

Sensitivity (same 300 seeds, one scatter changed; success / v<2 / dist<3):

| case | success | v<2 | dist<3 | speed p50/p90 |
|---|---|---|---|---|
| igniter jitter 0 / 0.02 / 0.08 s (report: 0.04) | 39.7 / 33.0 / 19.7 % | 55 / 44 / 26 % | 89 / 88 / 90 % | 1.83/5.66, 2.24/5.27, 2.99/7.69 |
| impulse sigma 0 / 2 / 6.67 % (NFPA cap; report: 3) | 34.7 / 31.3 / 17.0 % | 46 / 43 / 24 % | 87 / 88 / 86 % | 2.17/4.59, 2.22/5.52, 3.43/8.66 |
| wind fixed 0 / 4 m/s (report: uniform 0-4) | 28.7 / 10.3 % | 34 / 37 % | 99 / 40 % | 2.36/5.91, 2.29/5.81 |

Reading: even with no scatter at all roughly 40 % land softly: near hover the F15's 14 N tail is about the vehicle weight, so the landing is a knife edge on the thrust
scale, and scatter turns it into a coin toss. The 4 m/s wind costs position (60 % land more than 3 m from the pad: the drift in the glide is not steered out), not
speed. The per-seed list is in the `test_flights` of the JSON the tool writes (`--out`).

The scenario's default seed (13) is a demonstration seed picked after the fact from seeds 1-24 (7 of 24 are fully inside the monitors at 2 m/s wind); it is the
run to film, not evidence. Replay hash of that run, two separate `napl program` runs: identical, state hash `be85c642e12fd226` (10296 ticks, replay match, all nine monitors pass). Touchdown 1.15 m/s, 3.7 deg, 0.56 m from the pad.

## 7. Honest limitations

* A solid motor cannot be throttled: the dominant uncertainties are the motor's impulse scatter (1 sigma 3 % of 49.6 Ns is 1 m/s of velocity change on a 1.45 kg
  vehicle) and the ignition-delay jitter. The flight computer measures the thrust after the fact; it cannot compensate it. A stronger motor-to-motor tolerance
  than NFPA 1125's worst case is assumed (3 % against the 6.7 % cap).
* The success rate is a property of this vehicle class, not of the sim: the 1-D design Monte Carlo that picked the design point predicts the same ~30-40 % soft
  landings, and the knife-edge nature of the landing is the reason the claim is a distribution rather than one flight.
* Roll is uncontrolled; wind above 4 m/s is outside the report; the leg friction 0.3 is an estimate; the servo backlash is not measured for this servo.
