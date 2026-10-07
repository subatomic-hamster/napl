# DYNAMIXEL XL430-W250-T / XL330-M288-T servo-model calibration evidence (stage A3 / S5)

Accessed 2026-10-07. Builds on `docs/research/dynamixel.md` (specs, control tables, section 10 gain-scale recommendation); that material is not repeated. Same evidence-chain style as `docs/research/sts3215-calibration.md`. Labels: **[F]** = documented fact from a source fetched this session, **[I]** = inference (basis stated), **[V]** = verified by running the bench / arithmetic this session. Bench: `python -m napl.servo_bench dxl` (code `python/napl/servo_bench.py`, tests `tests/python/test_servo_bench_dxl.py`), fixture built from `assemblies/koch.toml` through `napl.design.compute` (device-map values, nothing hand-copied).

Sources fetched this session: ROBOTIS e-Manual `https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/` and `.../xl330-m288/` (HTML, grepped); the older XL430-W250 e-Manual PDF copy `https://shop.pishrobot.com/wp-content/uploads/2020/01/dynamixel-xl430-w250.pdf` (pdftotext); Rhoban BAM `https://github.com/Rhoban/bam` at commit `e9a619d56da5236206f4de6ceec2c1ee1b497b5c` (`bam/dynamixel/actuator.py`, `bam/params/xl330/m1.json ... m6.json`, tree listing through the GitHub API). A web search for a published control-loop rate / PID-unit statement for the X series returned nothing beyond the e-Manual pages themselves.

## 0. Decision summary

| Item | Value | Basis |
|---|---|---|
| `pid_scale` (P) | **113,280 = 128 x 885**, unchanged, both servo parts | 1 (documented divisors and PWM span; error unit inferred) |
| `d_scale` | **14,160 = 16 x 885**, unchanged | 1 |
| Device `i` from register I | **I_reg / 512** (= I x pid_scale / (65,536 x 885)); port constant `napl.dynamixel.I_SCALE = 128/65536` already is this | 3 |
| Schema `i_scale` | **not needed, no schema change** | 3 |
| Plausible range of the P scale | 113,280 (manual) to 226,560 (BAM's measured XL330 divisor 256). XL430: no measurement exists, band assumed the same | 2 |
| Firmware rate | 1 kHz, **not evidenced** (nothing published); sim needs only an integer number of steps | 4 |
| Datasheet bench | 18 comparisons all within +-10 % except one held-out XL430 point (9 V stall torque, +17 %, a datasheet-internal inconsistency, 5.2) | 5 |
| Changes to `assemblies/koch.toml` / part files | **none required**; one optional part refit (XL330 no-load current, 5.3) reported, not applied | 5, 8 |

## 1. The register-to-duty chain: documented vs inferred

**[F] e-Manual, identical on XL430-W250 and XL330-M288** (control table description, `Position PID Gain(80, 82, 84)`): "Position D Gain(80) KPD = KPD(TBL) / 16", "Position I Gain(82) KPI = KPI(TBL) / 65,536", "Position P Gain(84) KPP = KPP(TBL) / 128" (HTML grep of both pages: `65,536` and `/ 128` present; the three rows are in the old PDF at lines 925-935 of its text layer). Range 0 ~ 16,383. Velocity PI: KVI = KVI(TBL)/65,536, KVP = KVP(TBL)/128.

**[F] Signal flow, XL430 page, Position Control Mode:** "The Feedforward and PID controller calculate PWM output for the motor based on desired trajectories. Goal PWM(100) is used as a limit on the calculated PWM output and decides the final output PWM value. The final PWM value is applied to the motor through an inverter". XL330 page: "Feedforward and PID controller calculate PWM output for the motor based on desired trajectories. Goal PWM(100) sets a limit ... The final PWM value is applied to the motor through an Inverter". The block diagram (`position_controller_pid_gain.jpg`, read in `dynamixel.md` 5.1) is error = trajectory position - Present Position into parallel KPP, KPD s, KPI / s, summed with the feedforward branch, then a limiter with an anti-windup gain Ka (not user-adjustable).

**[F] PWM span:** `PWM Limit(36)` "0(0 [%]) ~ 885(100 [%] )", unit "0.113 [%]". So the PID output is a duty fraction PWM/885 of the supply.

**[F] Sampling time, old manual only:** the old XL430 e-Manual PDF says, in both the Velocity PI and Position PID sections, "Gains of DYNAMIXEL's internal controller can be calculated from Gains of the Control Table as shown below. The constant in each equations include sampling time." The current e-Manual HTML has no `sampling` string (grep: 0 hits on both pages).

**What the e-Manual does not state** (grep of both current pages: `kHz` 0, `control period` 0, no error-unit sentence): the unit of the position error entering KPP, the loop rate, the discretisation of KPD / KPI.

| Link | Status |
|---|---|
| KPP = P/128, KPI = I/65,536, KPD = D/16 | **[F]** |
| PID output is a PWM, 885 = 100 % duty | **[F]** |
| Error is in encoder pulses (0.088 deg) | **[I]** (every position register is in pulses; BAM's XL330 error gain is built from counts/rad, 5.2) |
| No extra scaling between the PID sum and the PWM | **[I]** (the XL430 page says the PID "calculate[s] PWM output"; BAM's divisor-256 finding is the one measurement against it) |
| D and I act per firmware period; the constants "include sampling time" | **[I]** from the old manual (KPI = Ki Ts, KPD = Kd / Ts); matches the device's per-step `i (integ + e)` and `d (e - e_prev)` |

Result: duty = (P/128) e / 885 = P e / 113,280 -> `pid_scale = 128 x 885 = 113,280`; D: duty_D = (D/16) de / 885 = D de / 14,160 -> `d_scale = 16 x 885 = 14,160`. Both part files already carry exactly these (`pid_scale`, `d_scale`), and `[V]` the built Koch servos have them (`test_built_gain_scales_follow_the_emanual_and_the_port_factors`).

## 2. Independent cross-checks: Rhoban BAM

### 2.1 The XL330 gain divisor (the one measurement of the firmware gain)

`bam/dynamixel/actuator.py` (class `XL330Actuator`) **[F]**:

```
XL330_ENCODER_COUNTS_PER_REV = 4096
XL330_KP_DIVISOR = 256  # Empirically observed for XL330 (manual mentions 128)
XL330_PWM_LIMIT = 885  # Default Present PWM limit for XL330
...  vin=7.5, kp=400,
     error_gain=(XL330_ENCODER_COUNTS_PER_REV / (2 * np.pi)) / (XL330_KP_DIVISOR * XL330_PWM_LIMIT),  max_pwm=1.0
```

BAM's control law is `duty = (q_target - q) * kp * error_gain` with the error in radians (`bam/actuator.py` pattern quoted in `sts3215-calibration.md` 1.2), so duty = e_counts * kp / (256 x 885): **pid_scale 226,560**, twice the manual's 113,280. This confirms the structure (error in counts, PWM / 885 as duty, no extra factors) [F] and disputes the divisor by 2x for the XL330 [F]. The comment carries no data; the same file's STS3215 / MX / XL320 gains are stated as oscilloscope-determined, the XL330 one only as "empirically observed". **No XL430 entry exists in BAM** (tree listing of `bam/params`: `erob80_100`, `erob80_50`, `feetech_sts3215_7_4V`, `mx106`, `mx64`, `waveshare_st3025`, `xl320`, `xl330`), so the XL430 has no independent gain measurement at all. Unresolved; the recommendation keeps the documented 128 and treats 226,560 as the other end of the band (section 3 shows what it changes).

### 2.2 BAM-identified XL330-M288 motor parameters vs the part's datasheet-derived model

`bam/params/xl330/m1.json ... m6.json` fetched [F] (m1, m2: unloaded identification; m3-m6: with a load model). BAM's fit starts from `kt` 1.6 (range 0.1-3), `R` 2.6 (2-5) and `armature` 0.005, i.e. blind to the datasheet, so kt, R, friction are **independent** of the part; armature is not (the part's `rotor_inertia_motor` is derived from it, circular).

| Quantity | BAM (m1 / m2 unloaded) | BAM (m5 / m6 loaded) | Part model (datasheet-derived) | Verdict |
|---|---|---|---|---|
| kt (output, N*m/A) | 0.3706 / 0.3701 | 0.3456 / 0.3460 | eta N Ke = 0.851 x 288.4 x 1.5148e-3 = 0.372; datasheet stall torque / current 0.354 at 5 V | **independent**, agrees to 0.3 % (m1), 6-7 % (m5, m6) |
| R (ohm) | 3.452 / 3.461 | 2.494 / 2.502 | 3.42 (datasheet V / I_stall) | **independent**, agrees to 1 % (m1, m2); the loaded fits drift to 2.5 |
| armature at output (kg*m^2) | 0.001609 / 0.001611 | 0.001568 / 0.001573 | 1.89e-8 x 288.4^2 = 0.001572 | **circular** (the part takes m6) |
| friction_base / friction_viscous (N*m, N*m*s/rad at output) | 0.01909 / 0.003924 (m1); 0.01846 / 0.004171 (m2) | 0.01211 / 0.005720 (m5) | part i0 = 0.088 A -> 0.0385 N*m at the 5 V no-load output speed | BAM total at 10.79 rad/s = 0.0191 + 0.0039 x 10.79 = 0.0615 N*m, **1.6x the part** (5.3) |
| command_delay (s) | 0.00593 / 0.00578 | 0.0103 / 0.0102 | not modelled | see 7 |

Static stall torque check [I, arithmetic]: BAM kt V / R at its 7.5 V = 0.3706 x 7.5 / 3.452 = 0.805 N*m vs part 0.372 x 7.5 / 3.42 = 0.816 N*m (+1.4 %). The motor model's kt / R is therefore corroborated by an unrelated identification; what BAM cannot corroborate is the PID-to-duty scale (2.1).

## 3. Resulting scales, plausible range, and the I factor

| Quantity | Documented (recommended) | BAM-divisor case (XL330 only measured) |
|---|---|---|
| `pid_scale` | 113,280 | 226,560 |
| `d_scale` | 14,160 | 28,320 **if** D scales like P (BAM measured only P; unknown) |
| device `i` per I register unit | 1/512 (I x pid_scale / (65,536 x 885)) | 1/1024 if pid_scale doubles and KPI keeps /65,536 (unknown) |
| static stiffness K = (P / pid_scale) tau_stall x 651.9 | XL330 P=400: 1.20 N*m/rad; LeRobot elbow P=1500: 4.5; XL430 P=640: 5.5 | half |

[I, arithmetic] Consequence at the XL330 defaults: a joint holding 0.1 N*m sags 0.1 / 1.20 = 0.083 rad = 4.8 deg at P = 400 (9.5 deg under the BAM divisor), 1.3 deg at LeRobot's P = 1500. The factor-2 uncertainty therefore matters for gravity sag of the Koch wrist and gripper and is the main calibration knob; it is a config value (`pid_scale` in the device map, one edit), as for the STS3215.

**I term and `i_scale` decision.** The device law is `u = (p e + i (integ + e) + d (e - e_prev)) / pid_scale` (`docs/models.md`, `servo.cpp`). With the firmware's KPI = I / 65,536 on the same PWM scale, device `i = I_reg x pid_scale / (65,536 x 885) = I_reg / 512` for 113,280. The conversion is a constant factor on the register write, and `napl.dynamixel` already applies it (`I_SCALE = 128 / 65536`, with `P_SCALE = 1`, `D_SCALE = 1`). `[V]` the test asserts `pid_scale / (65536 x 885) == I_SCALE`, `pid_scale / (128 x 885) == P_SCALE`, `d_scale / (16 x 885) == D_SCALE` on the built Koch servos, so a retuned `pid_scale` that forgets the port constants fails a test. Range: I_reg up to 16,383 gives device i up to 32, well inside a double. Register default I = 0 (also LeRobot's Koch writes) makes the term inert. **Conclusion: no schema `i_scale`; no change to the Servo table.** One residual: the firmware's anti-windup gain Ka (back-calculation) is not the device's conditional integration (stop accumulating while saturated and `u e > 0`); only visible with I != 0 and saturation, not modelled.

## 4. Firmware loop rate

No source states the position-loop period: the current e-Manual pages have no rate (`kHz`, `control period`: 0 hits); `Realtime Tick(120)` has a 1 ms unit (a communication timestamp resolution, not a loop rate); the old manual says only that the constants include the sampling time. BAM measured a 5.8-10.6 ms command-to-motion delay on the XL330 (`command_delay`), which does not bound the loop rate. The part files' `firmware_hz = 1000` is an estimate (fidelity "estimated"), kept. [V] Sensitivity from the bench (`servo_bench dxl`, 500 Hz = every second sim step, `d_scale` unchanged): the XL330 at default P=400, D=0 is unaffected (t 0.195 s, overshoot 4.8 -> 5.0 %); the XL430 at default D=4000 is the one sensitive case: settle(2 %) 0.451 -> 0.650 s, t to 2 counts 0.868 -> 1.212 s. If a real loop rate is ever found, set `firmware_hz` and scale `d_scale` by Ts_sim / Ts_real and `i` by Ts_real / Ts_sim (`dynamixel.md` 10.1 item 7). 2 kHz is not representable (the sim needs an integer `1 / (firmware_hz dt)` at dt = 1 ms).

## 5. Datasheet bench (`python -m napl.servo_bench dxl`)

Fixture identical to the STS3215 bench (one servo, regulated PSU with 1 mohm in series, gravity off, 50 g link; stalled variant +-0.05 rad limit at goal 1 rad, free variant goal 100 rad, both fixed-step at 1 ms). Tolerance +-10 %.

### 5.1 Table (run on hfbox through `tools/remote.sh run`)

| servo | V | quantity | datasheet | sim | err % | pass | kind |
|---|---|---|---|---|---|---|---|
| xl430 | 9 | stall torque N*m | 1 | 1.173 | +17.3 | FAIL (limit widened to 20 %, 5.2) | independent (held out) |
| xl430 | 9 | stall current A | 1 | 1.026 | +2.6 | PASS | independent (held out) |
| xl430 | 9 | no-load speed rad/s | 4.922 | 4.778 | -2.9 | PASS | independent (held out: Ke fitted to 11.1 / 12 V) |
| xl430 | 11.1 | stall torque N*m | 1.4 | 1.442 | +3.0 | PASS | circular (eta fitted to the two top-voltage stall torques) |
| xl430 | 11.1 | stall current A | 1.3 | 1.262 | -2.9 | PASS | circular (r_effective = mean V / I_stall of the top two voltages) |
| xl430 | 11.1 | no-load speed rad/s | 5.969 | 5.922 | -0.8 | PASS | circular (Ke) |
| xl430 | 12 | stall torque N*m | 1.5 | 1.557 | +3.8 | PASS | circular |
| xl430 | 12 | stall current A | 1.4 | 1.363 | -2.7 | PASS | circular |
| xl430 | 12 | no-load speed rad/s | 6.388 | 6.412 | +0.4 | PASS | circular |
| xl330 | 3.7 | stall torque N*m | 0.42 | 0.3916 | -6.8 | PASS | independent (held out) |
| xl330 | 3.7 | stall current A | 1.11 | 1.053 | -5.1 | PASS | independent (held out) |
| xl330 | 3.7 | no-load speed rad/s | 7.959 | 7.898 | -0.8 | PASS | circular (Ke = mean of all three) |
| xl330 | 5 | stall torque N*m | 0.52 | 0.5262 | +1.2 | PASS | circular |
| xl330 | 5 | stall current A | 1.47 | 1.415 | -3.8 | PASS | circular |
| xl330 | 5 | no-load speed rad/s | 10.79 | 10.75 | -0.4 | PASS | circular |
| xl330 | 6 | stall torque N*m | 0.6 | 0.6281 | +4.7 | PASS | circular |
| xl330 | 6 | stall current A | 1.74 | 1.688 | -3.0 | PASS | circular |
| xl330 | 6 | no-load speed rad/s | 12.88 | 12.94 | +0.4 | PASS | circular |
| xl330 | 5 | no-load current A | 0.14 (BAM-implied) | 0.0878 | -37.3 | n/a | independent, informational (no datasheet figure exists) |

Reading: the circular rows only show the fits are reproduced through the full device chain (they are not evidence of model validity). The held-out points are the evidence: XL430 9 V current (+2.6 %) and no-load speed (-2.9 %), XL330 3.7 V stall torque (-6.8 %) and current (-5.1 %) predict well from fits made at the other voltages.

### 5.2 Findings (each confirmed in the run above)

1. **XL430 9 V stall torque +17.3 %.** The datasheet's own 9 V row is inconsistent with the other two: its stated torque-per-current is 1.000 N*m/A (1.0 N*m / 1.0 A) against 1.077 and 1.071 at 11.1 / 12 V, and its current has one digit (a 5 % granularity). A motor whose torque is proportional to V / R cannot satisfy both; the part's derivation already excluded this point from R_eff. The model is not bent to it; the test widens that single row's tolerance to 20 % with the reason in the test comment.
2. **Stall current is 2.7-3.8 % below V / R_eff on every voltage** (also on the STS3215 bench, -2.0 / -3.7 %). `[V]` observed: with the stall held, the current falls as the winding warms (XL430 12 V: 1.3885 A at 50 ms, 1.3724 A at 0.5 s, 1.3635 A at 1.5 s against V / R_eff = 1.4035 A; printed `temperature` 25.0 -> 25.8 -> 27.4 C; XL330 5 V: 1.4775 -> 1.4276 -> 1.416 A against 1.462 A, 25.1 -> 26.1 -> 28.2 C). `[I]` attribution: winding resistance rising with temperature (bench ambient 25 C against the `r20` reference, plus stall heating over the 1.5 s run); the temperature coefficient was not checked against these numbers (they do not follow one constant coefficient between the 50 ms and 1.5 s samples). Either way it is a fixture / reference-temperature effect on the circular rows, not a gain or fit error, and it partly offsets the missing current-offset of finding 3.
3. **Stall torque carries no i0 correction in the sim.** The part derived eta with the no-load loss subtracted (`I_stall - I0`); the sim's loss torque is `Kt i0 (1 - v) tanh(w / ws)` which is zero at standstill, so the stalled torque is eta N Kt V / R. Net bias +1 to +5 % on the top-voltage circular rows (partly cancelled by finding 2). Within tolerance, not changed.

### 5.3 No-load current vs BAM (XL330, informational)

The part's `no_load_current_5v = 0.088 A` is an estimate (0.06 x I_stall; no published value). BAM's identified friction (m1: Coulomb 0.01909 N*m + viscous 0.003924 x 10.786 rad/s = 0.0615 N*m at the 5 V no-load output speed) converts to 0.13-0.15 A (`robotis-xl330-m288-t.toml` shows the conversion), 1.6x the part. The bench row (sim 0.0878 vs 0.14 A) makes the gap visible; `test_xl330_no_load_current_matches_bam_friction` is a strict xfail that will flip when the part is refitted. Effect of the gap: the loss torque at the output is 0.023 N*m too small at full speed, 4 % of the stall torque; at P = 400 (K = 1.2 N*m/rad) that is about 1 deg of extra static error. No datasheet row depends on it. See section 8.

## 6. Step response and holding (model only)

No published Koch or BAM step-response data was found (BAM's raw logs live in an HF bucket, not fetched; its repo has parameters, not step plots), so these are **model-only** numbers, not comparisons. 90 deg no-load steps at the Koch supply (XL430 12 V, XL330 5 V), `servo_bench dxl`:

| servo | registers | t to 2 counts s | rise 10-90 % s | overshoot % | settle (2 %) s | final err counts | peak rad/s |
|---|---|---|---|---|---|---|---|
| xl430 12 V | e-Manual default P=640 D=4000 | 0.868 | 0.246 | 0.0 | 0.451 | -1.0 | 6.37 |
| xl430 12 V | same, BAM divisor (pid_scale, d_scale x2) | 1.605 | 0.301 | 0.0 | 0.599 | -2.0 | 6.29 |
| xl430 12 V | same, 500 Hz loop | 1.212 | 0.329 | 0.0 | 0.650 | -1.0 | 6.24 |
| xl330 5 V | e-Manual default P=400 D=0 | 0.195 | 0.132 | 4.8 | 0.296 | 0.0 | 10.53 |
| xl330 5 V | same, BAM divisor | 0.279 | 0.155 | 0.1 | 0.243 | 0.0 | 10.09 |
| xl330 5 V | same, 500 Hz loop | 0.195 | 0.132 | 5.0 | 0.296 | 0.0 | 10.53 |
| xl330 5 V | LeRobot elbow P=1500 D=600 | 0.185 | 0.130 | 5.4 | 0.235 | 0.0 | 10.64 |
| xl330 5 V | same, BAM divisor | 0.188 | 0.130 | 5.4 | 0.261 | 0.0 | 10.60 |
| xl330 5 V | same, 500 Hz loop | 0.186 | 0.130 | 4.0 | 0.232 | 0.0 | 10.63 |

Reading: every step is speed-limited (peak speed = the no-load speed within 3-6 %, `[V]` the full-duty step excess equals the spin-up constant J / b: XL430 0.038 vs 0.037 s, XL330 0.037 vs 0.033 s); the XL430 default D = 4000 (D/P ratio 50 steps) is strongly overdamped and creeps the last counts (0.868 s vs 0.451 s to 2 %); the XL330 default (D = 0) rings once by 4.8 %, damped only by back-EMF. Holding: `[V]` at a partial error the device duty equals the register law exactly (goal 0.2 rad, 96 counts: XL430 0.5424 = 640 x 96 / 113,280, XL330 0.3390 = 400 x 96 / 113,280), and the unloaded final error is 0-2 counts. Whether the real servos look like this is untested: the model-vs-hardware comparison needs a measured step or a hold-torque-vs-error curve of a real XL430 / XL330 (the same suggestion as `sts3215-calibration.md` 1.6).

## 7. What is not modelled

- **Feedforward gains** (KFF1st / KFF2nd, registers 88 / 90, defaults 0): the device has no feedforward path. Default 0 on both servos, so only a client that writes them is affected.
- **Velocity-based profile acceleration** (Profile Acceleration 108, unit 214.577 rev/min^2) and the time-based profile: the device has a rectangular speed profile (`goal_speed`, from Profile Velocity 112) only; defaults 0 (infinite) on both, so default behaviour matches. LeRobot's Koch follower leaves them at 0.
- **Current-based position mode (mode 5), the gripper**: LeRobot puts joint 6 in mode 5 where the PID output is a current command limited by Goal Current(102) (`dynamixel.md` 5.1); the device has no current mode, so the gripper under mode 5 is the PWM-limited approximation `max_torque = 1000 x Goal_Current / I_stall(V)` or a device extension (not built). Under the device's duty law the gripper holds a grip at a duty set by P e rather than a regulated current, which behaves differently on a stalled grasp (BAM's `XL330CurrentActuator` notes the XL330 regulates the **bus** current, duty-dependent torque ~ sqrt(I_bus); its docstring marks that model untested).
- **The XL330 current loop**: the XL330 measures current at its input supply, which equals motor current only at 100 % duty (e-Manual note); `reg_present_current` through the port is the device `current` and is not the firmware's bus-side value at partial duty.
- **PWM slope** (XL330 reg 62, 140 x 1.977 mV/ms: a 0 to 100 % step takes 18 ms at 5 V) and **command latency** (BAM `command_delay` 5.8-10.6 ms): not modelled; the real step starts 6-10 ms later than the sim's.
- **Max duty**: BAM measured 0.97 on STS3215 oscilloscope; for the Dynamixels 885 = 100 % is the documented span, 1.0 here.
- **Stiction / dead zone**: tanh-smoothed friction (as the STS bench), `deadband` 0; the In-Position window and Following Error bit are not modelled.
- **Anti-windup Ka** (3), the Velocity PI loop (modes 1; no velocity-mode path), the D-term filter if any (the e-Manual does not show one).

## 8. Recommended changes (not applied: `assemblies/koch.toml` and the robotis part files are S4's)

1. **`assemblies/koch.toml`, `parts/arm/robotis-xl430-w250-t.toml`, `robotis-xl330-m288-t.toml`: `pid_scale` 113,280 / `d_scale` 14,160 stay as they are.** The only measurement that disputes them (BAM, XL330, 2x) is not decisive; the 2x band is the calibration knob. No change to device-map numbers is required by the bench.
2. **Optional, low priority, `robotis-xl330-m288-t.toml`: `no_load_current_5v` 0.088 -> about 0.14 A** (BAM friction, 5.3), with `ke` (1.515e-3 -> about 1.45e-3 = (5 - 0.14 x 3.42) / (288.4 x 10.786)), `kv_equivalent`, `gear_efficiency` refitted together, `i0_visc` kept at 0.69. The 18 datasheet rows would stay within tolerance (the stall points depend on R_eff and eta; refit eta to the same stall torques). Do not do it as a bare number swap: i0 feeds the Ke / eta derivation. Apply only if the wrist / gripper low-torque behaviour matters; the evidence is one third-party identification at 7.5 V.
3. **No schema change** (no `i_scale`, no `d_scale` change): the port-side I factor `I_SCALE = 128 / 65536` is the correct conversion for the built `pid_scale`; if `pid_scale` is ever retuned (e.g. 226,560), `P_SCALE`, `D_SCALE`, `I_SCALE` in `python/napl/dynamixel.py` must change in lockstep (`test_built_gain_scales_follow_the_emanual_and_the_port_factors` fails otherwise). A more robust port would read the device's `pid_scale` at bind; that is a port edit, not built here.
4. The XL430 9 V datasheet row (stall 1.0 N*m at 1.0 A, stated Kt 1.000) is internally inconsistent with the 11.1 / 12 V rows; nothing to change in the part, noted for anyone who later fits the 9 V point.

## 9. Gaps

- No firmware rate, error unit or XL430 gain measurement exists in any fetched source. A hold-torque-vs-position-error measurement on one real servo of each model (known weight on a lever, read Present Position at two P values) would settle the 128 / 256 dispute and the XL430 scale at once.
- ROBOTIS rotor inertia ("Moment of Inertia" download) is still unreachable (`dynamixel.md` section 1); the XL330 value is BAM-derived (circular above), the XL430 value a mass-scaled estimate. The step times scale with it (the XL430 spin-up constant J / b = 0.037 s).
- No published Koch / Dynamixel step-response data was found for either servo; the step table is model-only.
