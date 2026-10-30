# cuas2 closed-loop simulation results (P13 INTEG, re-run in P13 PERF, re-measured in P13 MC-3)

What was run: the cuas2 app (`python/napl/cuas2/app.py`: napl.track tracker with covariance, cuas2 lead solver, Monte-Carlo hit-probability gate Ph, shoot-look-shoot, protocol 1.1) in lockstep with the RP2040 firmware twin (`Sim2RP2040`) and the cuas2 weapon (flywheel launcher kernel model + one-ball gate feeder plant), against the cuas-lite scene kinds (balloon, drifting balloon, hanging targets, quad 3 m/s, quad 8 m/s, a walking person). Every ball is judged by the ground-truth `Referee2` (person, hold, geofence, armed per FIRE id, cap, E-stop/key, watchdog, dead-man, stale detector, after-cut, hit-person).

Safety result, from the P13 MC-3 re-run under `Referee2` as merged in p13-gates (52c6329; closest-approach measurement still the lite 5 ms rule, see "Aim bias") with the P13 RPMF firmware (telemetry rpm filter, rpm window 1 %, the firmware default): **0 violations in 1485 Monte-Carlo episodes** (9 configurations x 165, every one finished) **and the 13 named scenarios (seed 1); 0 crashed episodes.** The strict `person` violation count is 0 in every configuration. The looser `person_raw` (a ball fed while a person is anywhere in the image, inside the 0.25 s detector/link grace) is **0 in imm2, imm2_s2, imm2old, noph, imm3 and abnn, and 1 in three configurations: `ph0`, `abnn_noph` and `imm2r05`** (see "Person detection in the MC"). So the earlier statement "`person_raw` 0 in every deployed-gate configuration" does not hold for `imm2r05` (deployed Ph gate, changed measurement noise). Threat models are the generic kinematic test targets of cuas-lite; no new profiles.

P13 MC-3 changed the harness from the 2 % rpm window on raw telemetry (PERF) to the 1 % firmware default with the filtered telemetry (RPMF) and re-ran every table below. Result, stated plainly: **nothing changed measurably.** Balls differ by 0 to 2 per PERF-comparable configuration and kills/episode by at most 0.013 (section "P13 MC-3"). The numbers are not bit-identical to PERF (126 vs 124 balls in imm2): READY timing moved by a few ms. All MC tables in this file are from the filtered 1 % window; the three configurations PERF could not finish (`abnn`, `abnn_noph`, `imm2r05`) now have full 165-episode rows.

## Reproduce

```
export NAPL_REMOTE_DIR=/workspace/wt/napl-c2perf
tools/remote.sh bg --slots 1 mc  'examples/cuas2/ablation.sh'     # configs, 165 episodes each, 6 workers
tools/remote.sh bg --slots 2 sc  'examples/cuas2/scenarios.sh'    # the 13 named scenarios, seed 1
python examples/cuas2/mc.py --report imm2=build/mc/imm2 noph=build/mc/noph ...   # cross-config tables
python examples/cuas2/run.py s1_balloon_static --seed 1 --brief                  # one scenario
```

`mc.py` appends every finished episode to `<out>/episodes.jsonl` and, when restarted with the same arguments, runs only the missing episodes (resumable after a killed box session). Second seed block: `mc.py --seed0 20`.

Seeds: `mc.py --seed0 0`; episode i of a cell has seed `100000*(kind_index*3 + band_index) + i + 1 + seed0` (person episodes `900000 + i + 1 + seed0`), the same draw as `examples/cuas-lite/mc.py`; every configuration sees the identical 165 draws (paired). 10 episodes per kind x band cell (5 kinds x 3 bands = 150) + 15 person episodes. CIs are 95 % Wilson.

## P13 PERF: what was changed, before and after

"Before" is the INTEG head (0655e83) or, where stated, the `imm2old` configuration (mixture off, n_ph 300, with the wheel hold); "after" is from runs under the final `Referee2`.

| #   | problem                                                                                         | change                                                                                                                                                                                                                                                               | before                                               | after                                                                                                                                                                                                                           |
| --- | ----------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | READY only 23 % of the time on quad8; every 1-frame track dropout idled the wheels, then respin | `spin_down_s` (lite `FireCfg` field, default 2.0 s): the flywheel stays at `flywheel_duty` for `spin_down_s` after the last target while the dead-man is alive (`spin_down_s = 0` is the old behaviour); safety path (ARM, dead-man, person block, sector) untouched | 5-10 m kills/episode 0.46 (INTEG imm2, n=50)         | `imm2old` (hold only): 5-10 m 0.40 [0.28, 0.54], total 0.38: no measurable kill change; the hold removes the idle/respin, it did not by itself create engagement                                                                |
| 2   | quad8 never engaged                                                                             | not fixed; attributed (below)                                                                                                                                                                                                                                        | engaged 0/30, 0 balls                                | imm2 1/30 (2 balls), imm2_s2 3/30 (7 balls), noph 7/30 (15 balls), ph0 19/30 (68 balls); 0 kills in every configuration                                                                                                         |
| 3   | 10-15 m: 0 shots, Ph pessimistic                                                                | Ph from the IMM mixture with n_ph 500 (default); tau sweep                                                                                                                                                                                                           | 0 shots                                              | still 0 shots at tau 0.3-0.5 (mixture and Gaussian), 1 ball in imm3. With the gate off the same shots hit 0.09 (10-15 m, 152 balls, 11.7 balls/kill): the limit is the real dispersion and range noise, not only gate pessimism |
| 4   | `s6_person_walk` `resumed_after_person` false at seed 1                                         | the Ph change; person hold untouched (ended 19.9 to 20.9 s, 1.0 s)                                                                                                                                                                                                   | false; remaining balloons had Ph 0.04-0.17 for 40+ s | true; 5 shots, 4 hits, `engaged_before_person` true, 0 violations                                                                                                                                                               |
| 5   | static-target aim bias z -0.061 m                                                               | analysed per kind; no code change                                                                                                                                                                                                                                    | -0.061 pooled over all kinds                         | see "Aim bias"                                                                                                                                                                                                                  |
| 6   | rpm window 1 % never READY                                                                      | measured in PERF; fixed in RPMF (per-wheel telemetry IIR, READY 100 % at 1 %)                                                                                                                                                                                        |                                                      | see "rpm window"                                                                                                                                                                                                                |

Net effect of items 1 and 3 on the MC, stated plainly: **no demonstrated gain in kills/episode.** INTEG imm2 0.40 [0.33, 0.48] (89 balls, 1.48 balls/kill), new imm2 0.45 [0.37, 0.53] in seed block 1 and 0.41 [0.34, 0.49] in seed block 2, pooled n=300 0.43 [0.38, 0.49]: the intervals overlap. What changed is the trade: more balls (268 per 300 target episodes = 0.89 per episode, against 89 per 150 = 0.59 in INTEG) at lower hit/ball (0.48 vs 0.67) and 2.08 balls/kill vs 1.48. The mixture Ph is better calibrated (below) and the s6 resume check now passes; it does not make more kills. The 5-10 m kill rate of the first block (0.60) did not reproduce in the second (0.48); pooled 0.54 [0.44, 0.63] against INTEG 0.46 [0.33, 0.60].

### quad8 attribution (measured on 10 episodes, 5-10 m)

The quad flies back and forth (leg about 1.5 s at +-35 deg azimuth). With the harness gimbal limits (100 / 70 deg/s, 600 / 500 deg/s^2): old = 0 balls; wheel hold alone = 0 balls; a gimbal 2.4x faster (240 / 150 deg/s, 2000 / 1500 deg/s^2) plus the hold = 5/10 engaged, 12 balls, 1 hit (0.10 kills/episode; mean Ph 0.34 against a realised 0.08); the same fast gimbal with the gate off = 10/10 engaged, 40 balls, 0 hits. Trace of one episode: 53 % of ticks lie in a window that is valid for a constant-velocity shot (time to the next reversal longer than flight time + 0.1 s), but the pointing error is under 3 deg in 0 % of them: the aim point jumps about 50 deg at each reversal and the gimbal needs more than 0.7 s. Root cause (inferred from the trace): gimbal slew limits plus track velocity lag after each reversal; then constant-velocity lead is wrong at the reversal (1 hit in 12 balls even with the fast gimbal). Not fixed: the gimbal limits are hardware numbers. Needs roughly a 2.4x faster gimbal and a reversal-aware lead.

### Ph tau sweep (balloon, drift, hanging, quad3; 5-10 and 10-15 m; 10 episodes per kind x band, 40 episodes per band; kills/episode and balls/kill at 5-10 m)

| variant                          | kills/episode | balls/kill | note                    |
| -------------------------------- | ------------- | ---------- | ----------------------- |
| INTEG (n 300, Gaussian, no hold) | 0.57          | 1.52       | hit/ball 0.66           |
| Gaussian + hold (tau 0.3)        | 0.50          | 2.05       | hit/ball 0.49           |
| Gaussian, tau 0.4                | 0.42          | 2.12       |                         |
| mixture n 500, tau 0.3 (chosen)  | 0.75          | 2.47       | hit/ball 0.41, 74 balls |
| mixture n 500, tau 0.4           | 0.57          | 2.22       |                         |
| mixture n 500, tau 0.5           | 0.45          | 2.17       |                         |

All 10-15 m rows are 0 shots. Criterion: maximise kills/episode subject to balls/kill <= 2.5. Higher tau lowered kills without improving balls/kill, so the default stays at tau 0.3 with the mixture and n_ph 500. Tau 0.2 was only 47 of 80 episodes and is not tabulated. Hanging pendulum targets dominate the balls/kill (6-8, hit/ball 0.08-0.16 in the sweep); balloons and drift are 1.0-1.6.

The sweep (40 episodes, 4 kinds) has wide intervals, and its 0.75 for the chosen point is the high end: the 5-10 m figure at n=100 (5 kinds incl. quad8) is 0.54 [0.44, 0.63]. The chosen point stays a defensible one by the stated criterion (2.08 balls/kill pooled), not a demonstrated kill gain. The n_ph effect (300 to 500) was not isolated from the mixture switch.

### Aim bias (static targets)

**Superseded by P13 BIAS: see [aim-bias.md](aim-bias.md).** Findings there: about 40 % of the "-4 cm on static balloons" was the lite referee's 5 ms-sampled closest approach (a hit ball is removed at the target surface, so a climbing ball reads low; lite minus refined = -0.017 +- 0.004 m per shot), fixed in `Referee2` after the runs in this file; the real balloon z is **-0.022 +- 0.026 m** (not distinguishable from zero), and the hanging-target z (-0.07 m) is pendulum lead (arc curvature against a constant-velocity lead) plus a lead-time error, not a static aim offset. The `z` figures quoted from the MC below (balloon -0.04 m, hanging -0.08 m) are the lite sampled rule, because these MC episodes ran before that fix; see the "imm2 + refined referee" row for the same episodes under the refined rule. No online bias update was implemented (none is worth removing on static balloons).

Closest-approach z by kind from this MC-3 run, lite sampled rule (m, mean +- se over shots with closest approach under 0.6 m; shots, not episodes, so the se is optimistic): imm2 balloon -0.038 +- 0.013 (n=22), imm2 pooled blocks 1+2 balloon -0.042 +- 0.015 (n=47), balloon + hanging -0.077 +- 0.011 (n=125). The per-band z of PERF is not recomputed.

### rpm window (10 / 15 / 20 permille), READY duty

Plant2 plus the firmware state machine (python twin, byte-exact with the C core), 8 seeds, 5 s after 1.5 s of spin-up, constant request 17,500 rpm. First two columns: raw telemetry (P13 PERF, before the filter; re-run in P13 RPMF with the filter switched off: 0.62 / 84.6 / 99.75 %). Last: P13 RPMF firmware filter (first-order IIR, tau 8 ms, `FDR_RPM_FILT_DIV` 8).

| rpm window                          | duty, raw telemetry (PERF)                                        | losses per 5 s, raw | duty, filtered (RPMF) | losses per 5 s, filtered |
| ----------------------------------- | ----------------------------------------------------------------- | ------------------- | --------------------- | ------------------------ |
| 10 permille (1 %, firmware default) | 0.17 % (RPMF re-run 0.62 %, 7 of 8 seeds latched wheel_not_ready) | n/a                 | 100.00 % (min 100 %)  | 0.0                      |
| 15 permille                         | 83.1 % (min 78 %)                                                 | 7.6                 | 100.00 % (min 100 %)  | 0.0                      |
| 20 permille (2 %)                   | 99.75 % (min 98 %)                                                | 0.1                 | 100.00 % (min 100 %)  | 0.0                      |

Cause (unchanged): telemetry sigma 3 permille plus ripple 1.6 permille per wheel; the launcher checked the wheel difference `|(rb-rt) - (st1-st0)| <= window * target` on the raw telemetry (sigma of the difference about 4.8 permille, so a 10 permille window is 2.1 sigma, about 3.6 % exceedance per ms). Fix: the window and difference tests now read a per-wheel IIR `rpm_f += (rpm - rpm_f) / 8` (noise sigma x 0.26, so the difference is about 8 sigma inside a 10 permille window). The PI loop, overspeed fault and ball-dip measurement keep the raw rpm. The harness (`sim_lock.py`, `test_plant.py`) is back on the firmware default of 10 permille.

Latency added to the READY drop: a real slowdown of k x the window leaves the window after at most 6 ms (k = 2), 4 ms (k = 3), 2 ms (k = 5, about a ball dip at 1 %); the first-order bound is (7/8)^n <= 1 - 1/k samples at 1 kHz. A slowdown only just past the window (k near 1) takes longer by construction. Missing telemetry: not in window once a frame is older than 20 ms (unchanged), the filter is re-seeded from the first raw sample after such a gap, and 50 ms of silence still raises telemetry_lost. Tested in `tests/cuas2/test_rpm_filter.py` (C and twin identical) and two negative controls in `test_fw_vectors2.py` (filter too slow, age gate removed). Side effects: READY after spin-up arrives about 4 ms later, the WHEELS_LOST event of a shot about 1 ms later (event order SHOT before WHEELS_LOST). Gain and timing are estimates pending bench session B-TIME; the plant's noise model (3 permille white plus 1.6 permille ripple) is not a measurement of the real ESC telemetry.

## P13 MC-3: every MC config re-run under the filtered 1 % rpm window, against the PERF tables

What differs from PERF: the harness rpm window (2 % raw -> 1 % with the RPMF telemetry IIR; READY 100 % at both). Same 165 draws per configuration, same code otherwise (`ce8cc2c`; the MC episodes ran before the BIAS referee change). Differences are **unpaired** (the PERF per-episode files were wiped, so no seed pairing) and the PERF counts are back-computed from the 2-decimal rates in the PERF tables (so +-1 kill / +-1 hit); 95 % Newcombe interval on the difference of two proportions, kills out of 150 target episodes each, hit/ball out of the balls.

| config           | balls PERF -> now | kills/episode PERF -> now (difference [95 % CI]) | hit/ball PERF -> now (difference [95 % CI]) |
| ---------------- | ----------------- | ------------------------------------------------ | ------------------------------------------- |
| imm2             | 124 -> 126        | 0.45 -> 0.45 (-0.007 [-0.118, +0.105])           | 0.54 -> 0.53 (-0.009 [-0.130, +0.113])      |
| imm2_s2          | 144 -> 143        | 0.41 -> 0.42 (+0.013 [-0.097, +0.123])           | 0.43 -> 0.44 (+0.010 [-0.103, +0.123])      |
| imm2old          | 96 -> 97          | 0.38 -> 0.38 (+0.000 [-0.109, +0.109])           | 0.59 -> 0.59 (-0.006 [-0.142, +0.130])      |
| noph             | 346 -> 346        | 0.47 -> 0.47 (+0.000 [-0.112, +0.112])           | 0.20 -> 0.20 (+0.003 [-0.057, +0.063])      |
| ph0              | 380 -> 380        | 0.53 -> 0.53 (-0.007 [-0.118, +0.105])           | 0.21 -> 0.21 (-0.003 [-0.060, +0.055])      |
| imm3             | 135 -> 134        | 0.44 -> 0.44 (+0.000 [-0.111, +0.111])           | 0.49 -> 0.49 (+0.004 [-0.114, +0.121])      |
| imm2 pooled n=20 | 268 -> 269        | 0.43 -> 0.43 (difference [-0.075, +0.082])       | 0.48 -> 0.48                                |

Plainly: **no configuration moved; every interval contains 0 and the point differences are at most 0.013 in kills/episode and 0.010 in hit/ball.** Ball counts are within 0-2 of PERF in all seven comparable rows (`noph` and `ph0` identical), so the runs are close to the same episodes with READY timing a few ms apart. The interval width (about +-0.11 per config) means a real change of a few percentage points cannot be excluded by this comparison alone; the near-identical ball counts argue that none exists (inferred). The three configurations PERF could not finish (`abnn`, `abnn_noph`, `imm2r05`) have no PERF value to compare and are in the ablation table below.

### imm2 + refined referee (BIAS closest-approach rule; same imm2 episodes, blocks 1+2)

Same 330 episodes (imm2 blocks 1+2, seeds 0 and 20), run again on main after P13 BIAS (`Referee2` records the refined closest approach of `napl.cuas2.miss`; shots and hits are unchanged by construction).

| row                    | episodes | balls | hits | kills | violations | person_raw | z balloon, pooled (m)  | z balloon + hanging (m) |
| ---------------------- | -------- | ----- | ---- | ----- | ---------- | ---------- | ---------------------- | ----------------------- |
| imm2 (lite 5 ms rule)  | 330      | 269   | 130  | 130   | 0          | 0          | -0.042 +- 0.015 (n=47) | -0.077 +- 0.011 (n=125) |
| imm2 + refined referee | 330      | 269   | 130  | 130   | 0          | 0          | -0.023 +- 0.016 (n=47) | -0.066 +- 0.011 (n=125) |

Result: **0 of 330 episodes differ in shots, hits or kill** (zero, not merely within noise), so the refined rule changes the `miss` vectors only. Paired per shot on the balloon: refined minus lite z = +0.019 +- 0.003 m (n=47), the lite rule reading about 2 cm low as in [aim-bias.md](aim-bias.md) (-0.017 +- 0.004 there). The refined balloon z -0.023 +- 0.016 m agrees with the BIAS figure -0.022 +- 0.026 m and is not distinguishable from zero. The se here counts shots, not episodes, so it is optimistic. Hanging stays about -0.07 m (pendulum lead, see aim-bias.md). The "-0.04 m static balloon bias" quoted from the lite rule is therefore about half measurement artefact.

### Why `abnn` never fires (Ph against the gate)

`abnn` (ab-nn tracker, Ph gate on) fired 0 balls in 165 episodes while `abnn_noph` (same tracker, gate off) fired 177, so the zero is the gate, not safety behaviour and not a dead tracker: tracks were confirmed in 161 of 165 episodes, and summed over the episodes the reason `ph` rejected 298,146 decision ticks (`pointing` 266,399, `not_ready` 214,667). Measured on 20 episodes (balloon and quad3, 3-5 and 5-10 m, 5 per cell) at every decision where Ph is computed: ab-nn Ph median 0.012, p90 0.020, p99 0.026, **max 0.036 over 19,355 evaluations, against the gate 0.3: 0 % reach it**. The cause is the covariance the ab-nn adapter hands to fire control: sigma_pos 1.0 m and sigma_vel 2.0 m/s, constant (medians over the evaluations), against imm2-gnn's 0.34 m and 0.76 m/s on the same 20 episodes, where Ph has median 0.10, p90 0.20, p99 0.36, max 0.90 and 1.8 % of 1,532 evaluations reach 0.3 (28 balls). A 1 m position sigma against a target radius of 0.15-0.2 m leaves Ph at a few percent whatever the geometry (inferred). This is the documented design (ab-nn has no covariance; the adapter exports a fixed one), not a bug found in the code; the baseline is only usable with the gate off (`abnn_noph`, its fair arm), and `abnn` must not be read as a safe configuration. Not changed.

## Headline (imm2-gnn tracker, Ph from the IMM mixture, n_ph 500, tau 0.3), seed block 1 (n=10/cell)

| range band m | episodes | engaged | balls | hit/ball [95 % CI] | kills/episode [CI] | t first shot s (med/p90) | balls/kill | violations |
| ------------ | -------- | ------- | ----- | ------------------ | ------------------ | ------------------------ | ---------- | ---------- |
| 3-5          | 50       | 39      | 50    | 0.74 [0.60, 0.84]  | 0.74 [0.60, 0.84]  | 0.67 / 1.27              | 1.35       | 0          |
| 5-10         | 50       | 37      | 76    | 0.39 [0.29, 0.51]  | 0.60 [0.46, 0.72]  | 0.69 / 4.01              | 2.53       | 0          |
| 10-15        | 50       | 0       | 0     | n/a                | 0.00 [0.00, 0.07]  | -                        | -          | 0          |

Totals: 126 balls, hit/ball 0.53 [0.44, 0.62], kills/episode 0.45 [0.37, 0.53], 1.88 balls/kill, mean Ph of fired balls 0.43. (PERF: 124 balls, 0.54, 0.45, 1.85.)

By kind (hit/ball, kills/episode, balls/kill): balloon 0.91 / 0.67 / 1.10, drift 0.69 / 0.60 / 1.44, quad3 0.50 / 0.50 / 2.00, hanging 0.30 / 0.47 / 3.29, quad8 0 kills (engaged 1/30, 2 balls).

### Second seed block (`--seed0 20`) and pooled n=20 per cell (300 episodes)

| range band m    | episodes | engaged | balls | hit/ball [CI]     | kills/episode [CI] | balls/kill |
| --------------- | -------- | ------- | ----- | ----------------- | ------------------ | ---------- |
| 3-5 (block 2)   | 50       | 43      | 63    | 0.60 [0.48, 0.71] | 0.76 [0.63, 0.86]  | 1.66       |
| 5-10 (block 2)  | 50       | 32      | 80    | 0.31 [0.22, 0.42] | 0.50 [0.37, 0.63]  | 3.20       |
| 10-15 (block 2) | 50       | 0       | 0     | n/a               | 0.00 [0.00, 0.07]  | -          |
| 3-5 pooled      | 100      | 82      | 113   | 0.66 [0.57, 0.74] | 0.75 [0.66, 0.82]  | 1.51       |
| 5-10 pooled     | 100      | 69      | 156   | 0.35 [0.28, 0.43] | 0.55 [0.45, 0.64]  | 2.84       |
| 10-15 pooled    | 100      | 0       | 0     | n/a               | 0.00 [0.00, 0.04]  | -          |

Pooled totals (n=300): 269 balls, hit/ball 0.48 [0.42, 0.54], kills/episode 0.43 [0.38, 0.49], 2.07 balls/kill, mean Ph of fired balls 0.43, 0 violations, 0 crashed. By kind, pooled (hit/ball, kills/episode, balls/kill): balloon 0.81 / 0.63 / 1.24 (60 episodes), drift 0.70 / 0.62 / 1.43, hanging 0.30 / 0.43 / 3.31, quad3 0.39 / 0.48 / 2.55, quad8 0.00 / 0.00 (4 of 60 engaged, 9 balls). Person episodes: 82 balls over 30 episodes, `person_raw` 0.

Read hit/ball with care: the Ph gate refuses poor shots, so hit/ball is conditional on a shot having been judged good. The like-for-like figures are kills/episode and balls/kill.

### Against cuas-lite

cuas-lite (no Ph gate, flywheel scatter model v1): hit/ball 0.54 / 0.21 / 0.08 for 3-5 / 5-10 / 10-15 m, quad8 0 / 0.

| cuas2 configuration                                       | hit/ball 3-5      | 5-10              | 10-15             |
| --------------------------------------------------------- | ----------------- | ----------------- | ----------------- |
| lite (reference, not re-run)                              | 0.54              | 0.21              | 0.08              |
| cuas2, Ph gate off (`noph`, lite pointing tolerance)      | 0.63 [0.50, 0.74] | 0.15 [0.10, 0.22] | 0.09 [0.05, 0.14] |
| cuas2, Ph gate on (`imm2`, n=10; pooled n=20 in brackets) | 0.74 (0.66)       | 0.39 (0.35)       | n/a (0 shots)     |

quad8 stays at 0 kills in every cuas2 configuration (see "quad8 attribution").

## Ablations (same 165 draws per config, final Referee2, 0 violations in all)

| config    | change                                                 | balls | hit/ball [CI]     | kills/episode [CI] | balls/kill | mean Ph of FIREs | person-episode balls |
| --------- | ------------------------------------------------------ | ----- | ----------------- | ------------------ | ---------- | ---------------- | -------------------- |
| imm2      | new default: mixture Ph, n_ph 500, tau 0.3, wheel hold | 126   | 0.53 [0.44, 0.62] | 0.45 [0.37, 0.53]  | 1.88       | 0.43             | 43                   |
| imm2_s2   | same, seed block 2                                     | 143   | 0.44 [0.36, 0.52] | 0.42 [0.34, 0.50]  | 2.27       | 0.44             | 39                   |
| imm2old   | mixture off, n_ph 300, wheel hold (INTEG Ph)           | 97    | 0.59 [0.49, 0.68] | 0.38 [0.31, 0.46]  | 1.70       | 0.44             | 24                   |
| noph      | Ph gate off (lite pointing tolerance)                  | 346   | 0.20 [0.16, 0.25] | 0.47 [0.39, 0.55]  | 4.94       | 0.28             | 43                   |
| ph0       | Ph computed, threshold 0 (3 deg pointing bound only)   | 380   | 0.21 [0.17, 0.25] | 0.53 [0.45, 0.60]  | 4.81       | 0.16             | 49                   |
| imm3      | imm3-gnn tracker, Ph gate on (new default Ph)          | 134   | 0.49 [0.41, 0.58] | 0.44 [0.36, 0.52]  | 2.03       | 0.46             | 39                   |
| abnn      | ab-nn tracker, Ph gate on                              | 0     | n/a               | 0.00 [0.00, 0.02]  | n/a        | n/a              | 0                    |
| abnn_noph | ab-nn tracker, Ph gate off (lite pointing tolerance)   | 177   | 0.36 [0.29, 0.43] | 0.43 [0.35, 0.51]  | 2.77       | 0.01             | 57                   |
| imm2r05   | imm2-gnn, sigma_px 1.5, range_rel 0.05                 | 176   | 0.36 [0.29, 0.43] | 0.42 [0.34, 0.50]  | 2.79       | 0.44             | 60                   |

(kills/episode and balls are over the 150 target episodes; the 15 person episodes are listed separately: balls fed there, never with a person beyond the grace except as listed under "Person detection in the MC".) `abnn` fires nothing: ab-nn exports a fixed 1 m / 2 m/s covariance, so Ph stays about 0.01 (mean Ph of `abnn_noph` FIREs 0.01) and the gate refuses every shot. `abnn_noph` (lite pointing tolerance with the ab-nn tracker) kills as many targets as the default (0.43 vs 0.45) at 2.77 balls/kill, but 1 of 50 10-15 m episodes only. INTEG-era numbers for the two that PERF could not finish, under the previous Referee and the INTEG Ph (not comparable, different code): `abnn_noph` 151 balls, hit/ball 0.42, 0.43 kills/episode; `imm2r05` 107 balls, hit/ball 0.55, 0.39 kills/episode.

kills/episode by band (3-5 / 5-10 / 10-15 m):

| config    | 3-5  | 5-10 | 10-15             |
| --------- | ---- | ---- | ----------------- |
| imm2      | 0.74 | 0.60 | 0.00              |
| imm2_s2   | 0.76 | 0.50 | 0.00              |
| imm2old   | 0.74 | 0.40 | 0.00              |
| imm3      | 0.74 | 0.58 | 0.00              |
| noph      | 0.72 | 0.42 | 0.26 [0.16, 0.40] |
| ph0       | 0.72 | 0.56 | 0.30 [0.19, 0.44] |
| abnn_noph | 0.76 | 0.50 | 0.02 (1 of 50)    |
| imm2r05   | 0.76 | 0.50 | 0.00              |

What the ablations show (measured; the explanations are inferences):

- **The Ph gate trades kills for ammunition.** It cuts balls/kill from 4.94 (noph) / 4.81 (ph0) to 1.88-2.27 and raises hit/ball from 0.20 / 0.21 to 0.44-0.53, at a kills/episode that is statistically indistinguishable overall (0.43 pooled vs 0.47 / 0.53), because the gate's lost 10-15 m kills (0.26-0.30 per episode with the gate off, at 11.7-13.4 balls/kill) are offset at 5-10 m, where the gated runs kill more than `noph` (0.60 / 0.50 vs 0.42; cause not isolated). At 10-15 m the track covariance (range noise from the box size, `range_rel` 0.15) never gives Ph >= 0.3, so nothing is fired.
- **Mixture vs Gaussian Ph (`imm2` vs `imm2old`)**: kills 0.45 / 0.42 vs 0.38, balls 126 / 143 vs 97, hit/ball 0.53 / 0.44 vs 0.59. The mixture lets through more shots on near-static targets and is better calibrated (reliability table below), but does not raise kills beyond the overlapping intervals.
- **Tracker.** imm3-gnn and imm2-gnn are not distinguishable here (kills 0.44 vs 0.45, balls 134 vs 126, hit/ball 0.49 vs 0.53).
- **Measurement noise** (`imm2r05`, sigma_px 1.5 and range_rel 0.05, now complete): kills 0.42 [0.34, 0.50] against 0.45 for the default, i.e. no gain from a tighter measurement model, but 176 balls instead of 126 and hit/ball 0.36 against 0.53: the tighter covariance lets more marginal shots through, the same trade as the Ph threshold. The kill difference (-0.03) is inside the noise.
- **ab-nn tracker.** With the Ph gate it never fires (fixed exported covariance); without it, it kills about as many as the default (0.43) at a higher ball cost (2.77 balls/kill) and no 10-15 m kills worth the name (1 of 50).

## Ph reliability

New default (`imm2`, mixture Ph, n=300 episodes: imm2 + imm2_s2; 269 balls in the target episodes, mean Ph 0.43, hit/ball 0.48; recomputed in MC-3 with `mc.py`'s binning. The PERF version of this table counted 412 balls, which does not reconcile with the 268 balls of the same 300 episodes and is replaced):

| Ph bin  | balls | hits | hit/ball [95 % CI] | mean Ph |
| ------- | ----- | ---- | ------------------ | ------- |
| 0.3-0.4 | 151   | 59   | 0.39 [0.32, 0.47]  | 0.33    |
| 0.4-0.5 | 44    | 19   | 0.43 [0.30, 0.58]  | 0.44    |
| 0.5-0.6 | 37    | 22   | 0.59 [0.43, 0.74]  | 0.55    |
| 0.6-0.8 | 27    | 21   | 0.78 [0.59, 0.89]  | 0.69    |
| 0.8-1.0 | 10    | 9    | 0.90 [0.60, 0.98]  | 0.86    |

With the mixture, realised hit/ball follows mean Ph (0.39 vs 0.33 at the 0.3-0.4 bin, 0.43 vs 0.44, 0.59 vs 0.55, 0.78 vs 0.69, 0.90 vs 0.86): calibrated to within the intervals (slightly conservative at 0.3-0.4), no longer pessimistic. (INTEG, Gaussian Ph: 0.3-0.4 bin realised 0.49 vs mean 0.34.) Hit/ball here includes hanging pendulum targets and quad3, which are the miss-prone kinds.

All four first-block configurations pooled (imm2, imm2old, noph, ph0; 949 balls, mean Ph 0.27, hit/ball 0.29), for Ph below the gate:

| Ph bin  | balls | hits | hit/ball [95 % CI] | mean Ph |
| ------- | ----- | ---- | ------------------ | ------- |
| 0.0-0.1 | 358   | 21   | 0.06 [0.04, 0.09]  | 0.05    |
| 0.1-0.2 | 104   | 19   | 0.18 [0.12, 0.27]  | 0.14    |
| 0.2-0.3 | 52    | 12   | 0.23 [0.14, 0.36]  | 0.25    |
| 0.3-0.4 | 188   | 84   | 0.45 [0.38, 0.52]  | 0.34    |
| 0.4-0.5 | 70    | 22   | 0.31 [0.22, 0.43]  | 0.45    |
| 0.5-0.6 | 69    | 36   | 0.52 [0.41, 0.64]  | 0.55    |
| 0.6-0.8 | 86    | 59   | 0.69 [0.58, 0.77]  | 0.70    |
| 0.8-1.0 | 22    | 20   | 0.91 [0.72, 0.97]  | 0.85    |

Ph ranks shots correctly (0.06 at Ph < 0.1, 0.18-0.23 at 0.1-0.3, 0.46 at 0.3-0.4, 0.69-0.91 above 0.6).

## Named scenarios (`scenarios.sh`, seed 1, Referee2 and defaults, re-run in MC-3 with the filtered 1 % window; 0 violations in all 13; shots and hits identical to PERF)

| scenario                                     | shots      | hits | checks                                                                             |
| -------------------------------------------- | ---------- | ---- | ---------------------------------------------------------------------------------- |
| s1_balloon_static                            | 2          | 2    |                                                                                    |
| s2_balloon_drift                             | 1          | 1    |                                                                                    |
| s3_hanging_targets                           | 5          | 1    |                                                                                    |
| s4_quad_3ms                                  | 0          | 0    | Ph never >= 0.3                                                                    |
| s4_quad_8ms                                  | 0          | 0    |                                                                                    |
| s5_mix                                       | 5          | 1    |                                                                                    |
| s6_person_walk                               | 5          | 4    | `engaged_before_person` true; `resumed_after_person` **true** (was false in INTEG) |
| s6b_person_beside_balloon                    | 0          | 0    | no_shots, target_was_engageable_but_for_the_person: true                           |
| s7 dead-man / detector / E-stop / key / link | all faults |      | all fault checks true                                                              |

The person hold after the person leaves is unchanged (19.9 s to 20.9 s, 1.0 s, the spec value). `resumed_after_person` failed in INTEG because the remaining balloons had Ph 0.04-0.17 (reason `ph` / `pointing` alternating) for 40+ s; with the mixture Ph it resumes.

## Throttle headroom at the 38 m/s v_cap (2205 2300 KV motors, BOM merged)

Computed with `LauncherModel` (wheel surface 37.999 m/s is rpm_cap 18143; scratch script, standalone) and confirmed in the sim: dshot 1474 (u = 0.731) holds 18146 rpm.

| bus V | throttle to hold rpm_cap, no load | ball in the nip (droop 0.10) |
| ----- | --------------------------------- | ---------------------------- |
| 12.0  | 0.657                             | **0.730**                    |
| 11.1  | 0.711                             | 0.790                        |
| 10.5  | 0.751                             | 0.835                        |

Margin to throttle_max 0.90 at 12 V: 0.17 (0.065 at 10.5 V); rpm available at 0.90 under load 22356 at 12 V. rpm_max 17500 needs 0.705 at 12 V under load. Agrees with the BOM worker's 0.73. The 38 m/s cap is the wheel surface speed: at rpm_cap the ball leaves at about 30.4 m/s (k_slip 0.8); a 38 m/s ball would need 22680 rpm (47.5 m/s surface), beyond the motor under load.

## Fixes made on the way (so the numbers above are not artefacts)

- Plant and stub constants aligned to MECH and firmware (stroke 47.2 mm, nip window 105-130 ms, hold 0.338 s, min FIRE interval 500 ms, commit 105 ms; kv 2300). The spec's `[launcher.feed]` calib example still shows 80 / 45 / 8 ms; that spec is not changed here.
- READY blinked about 1 time in 40 at the plant's 3 permille telemetry noise with the spec default rpm window 1 % (cfg 27 = 10), so a FIRE was refused (NOT_SPUN). The MC runs in this document set cfg 27 = 20 (2 %, the CFG_SET maximum) on raw telemetry in PERF. Measured in P13 PERF (table above): at 1 % the wheels were practically never READY, at 1.5 % 83 %, at 2 % 99.75 %. Since P13 RPMF the firmware filters the telemetry rpm and the harness uses the 1 % default (READY 100 %); P13 MC-3 re-ran every MC table under it (section "P13 MC-3"): no measurable change.
- The lite `Referee` judged the cap-th ball against the app's armed flag after the same step in which the app disarms on that shot; 3 of the first 165 gate-off episodes read as `unarmed`. `Referee2` (`examples/cuas2/run.py`) judges each ball against the exact arm sessions: `Lockstep2.arm_iv` records ARM / disarm in sim time, `Lockstep2.fire_sess` is a dict `{fire_id: session}` (the session of every FIRE sent), and a ball is `unarmed` if its feed time (from the twin) is outside every session or in a session other than that of its own FIRE id (pure function `ball_unarmed`; merged from p13-gates, tests in `tests/cuas2/test_referee.py`). A ball fed after a disarm, or after a quick re-arm from an older session's FIRE, is flagged. All tables in this document are from runs under this final Referee2.
- Rail-cut fault injection order in `Weapon2.step`.
- P13 PERF: wheel hold after the last target (`spin_down_s`), IMM-mixture Ph (`ph_mixture`, `n_ph` 500), resumable MC (`episodes.jsonl`).

## Deviations from `spec.md` (fire control, section 8)

| spec                                                       | built                                                                                                                                                                                                                                 | why                                                                                                                                                                                                                                                                                                                                             |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| fire iff Ph >= tau, default 0.25 [est]                     | tau = 0.3 (`FireCfg2.ph_tau`)                                                                                                                                                                                                         | swept 0.3 / 0.4 / 0.5 (mixture) and 0.3 / 0.4 / 0.5 (Gaussian) at 5-15 m, 40 episodes per point (see "P13 PERF"); 0.2 only partly run (47 of 80 episodes). Criterion: maximum kills/episode subject to balls/kill <= 2.5; higher tau only lowered kills. 0.25 itself was not run; the Ph reliability table puts the 0.2-0.3 bin at 0.2 hit/ball |
| N = 500 samples                                            | n_ph = 500 (`FireCfg2.n_ph`, now the spec value)                                                                                                                                                                                      | the n_ph effect was not isolated from the mixture change (both switched together). No timing of Ph per call against the spec's 10 ms laptop / 40 ms Jetson budget was made                                                                                                                                                                      |
| class probability of a target class >= 0.7 [est]           | detector `conf >= MIN_CONF` (0.5, lite `FireCfg.min_conf`) and the class in `TARGET_CLASSES`                                                                                                                                          | the detector reports one confidence, no class posterior; 0.5 is lite's value. Looser than the spec: a 0.5-0.7 class confidence engages here                                                                                                                                                                                                     |
| `t_dead = latency + t_actuator 0.06 + t_feed 0.045`        | `t_actuator` 0.02 + `t_fire` 0 (lite `aim.T_ACTUATOR`, `T_FIRE`) plus the measured tracker latency; the feed delay is in the flight model (T_EXIT 0.118 for the next-shot spacing)                                                    | not re-derived for the cuas2 gimbal (none exists, see Limitations); a `t_actuator` sweep was not run                                                                                                                                                                                                                                            |
| `track.sample(t_imp)` IMM-mixture aware                    | `hit_probability(..., mixture=True)` (default, `FireCfg2.ph_mixture`): mode drawn from the IMM mode probabilities, state from that mode's Gaussian propagated to `t_imp`; moment-matched Gaussian fallback for trackers without modes | the IMM's moment-matched Gaussian is inflated by the manoeuvre mode on near-static targets (Ph 0.05-0.17 on a static balloon)                                                                                                                                                                                                                   |
| range and class noise added to the samples                 | the range noise is inside the track covariance (`range_rel`) only                                                                                                                                                                     |                                                                                                                                                                                                                                                                                                                                                 |
| the track's post-shot residual updates the aim bias online | not implemented                                                                                                                                                                                                                       | the 4 cm low bias on static balloons (see "Aim bias") is what it would remove                                                                                                                                                                                                                                                                   |
| top-3 candidates per 50 ms, linearised fallback            | one target per decision (nearest engageable, sticky)                                                                                                                                                                                  |                                                                                                                                                                                                                                                                                                                                                 |

## Person detection in the MC

`person_raw` (balls fed while a person was visible in the image, summed over the 15 person episodes of each configuration): **0** for imm2 (43 balls fed in those episodes), imm2_s2 (39), imm2old (24), noph (43), imm3 (39) and abnn (0 balls); **1 for `ph0`** (49 balls, seed 900015), **`abnn_noph`** (57 balls, seed 900005) and **`imm2r05`** (60 balls, seed 900011). Each is a ball fed within the 0.25 s detector/link latency grace of the person entering the image (confirmed: the strict `person` violation count, which only counts balls fed more than the grace after the person became visible, is 0 in all three). `ph0` and `abnn_noph` are ablations without the Ph gate (the Ph test removed; ARM, dead-man, person block and sector unchanged). `imm2r05` has the deployed Ph gate with a changed measurement noise model, so the old statement that every deployed-gate configuration has `person_raw` 0 is not true; the three configurations with `person_raw` 1 are also the three that fed the most balls in the person episodes (49-60 against 24-43). This is the brief's "0 balls with a person anywhere in the image" not met by those configurations; the race is the detector-to-link latency (the person appears in the world before the frame that shows him arrives), which no gate can close, and a configuration that fires more balls near the person episodes meets it more often (inferred from the ball counts; not tested). The previous INTEG `ph0` run had 0. Not hidden, not fixed. The blocking is also tested at App2 level without the kernel: `tests/cuas2/test_app.py::test_app2_whole_frame_person_latch_blocks_a_person_away_from_the_target` (a person box that does not overlap the target still holds fire for the hold time; fails with the latch line removed).

## Tests

`pytest -n 4` (`PYTHONPATH=python:tests`) on `tests/cuas2/test_app.py`, `test_referee.py`, `test_perf.py` (new: wheel hold, mixture Ph), `test_plant.py`, `test_launcher.py`, `test_calib.py`, `test_sim2.py`, `test_kernel_launcher.py`: 200 passed on hfbox at the P13 PERF head. P13 MC-3, at the RPMF head (`-n 8`): `tests/lite/test_lockstep.py`, `test_app.py`, `test_referee.py`, `test_perf.py` together 51 passed in 397 s, including the kernel pin below; the other modules were not re-run. That count needs the built kernel and the built host firmware library (`build/cuas2-fw-host/libcuas2_fwcore.so` for `test_sim2`, the napl kernel for `test_kernel_launcher`); without them those modules error or skip. `tests/lite/test_lockstep.py::test_kernel_value_is_pinned` pins the lite kernel (balloon seed 1, lockstep seed 3): tick 11015, state_hash 35634cd3ed9cf0f0 (passes at the RPMF head, P13 MC-3).

## Limitations (what this does not show)

- The scene is the cuas-lite turret assembly with the cuas2 launcher device: `assemblies/cuas2.toml` has no servo/camera/launcher devices. The gimbal, camera and servo numbers are v1's (spec R5), not a cuas2 gimbal. A real cuas2 turret will move these numbers; quad8 in particular needs a faster gimbal.
- No online aim-bias update after a shot; the closest-approach test uses a fixed target radius (0.15 m) for every class. Hanging (pendulum) targets are led with a constant-velocity model; they take 3.3 balls/kill.
- 10-15 m: 0 shots under the gate (with the gate off 9-13 balls per kill). Not a gate-calibration problem alone.
- The MC runs use the 1 % rpm window on filtered telemetry (the firmware default; the plant's telemetry noise model is not a measurement of the real ESC); the dispersion model (`spread`, `v0_err`, cd) is the lite draw mapped onto the cuas2 plant calibration.
- No per-scenario `.scenario.toml` files and no web-UI snapshot support for cuas2 (existing scenario kinds only).
- Single seed per named scenario; the MC has 10 episodes per kind x band cell (20 in the pooled imm2 table), so per-cell CIs are wide (see tables). The tau sweep points are 40 episodes each.
- The MC-3 comparison against PERF is unpaired (the PERF per-episode files no longer exist) with an interval of about +-0.11 on kills/episode: it shows no change large enough to matter, not that there is none. A seed-paired control would need the harness forced back to the 2 % raw window; not run.
- Not done: HIL, video, a `t_actuator` sweep, and the stage-2 threats (on hold).
