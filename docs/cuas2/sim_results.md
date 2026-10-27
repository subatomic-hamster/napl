# cuas2 closed-loop simulation results (P13 INTEG)

What was run: the cuas2 app (`python/napl/cuas2/app.py`: napl.track tracker with covariance, cuas2 lead solver, Monte-Carlo hit-probability gate Ph, shoot-look-shoot, protocol 1.1) in lockstep with the RP2040 firmware twin (`Sim2RP2040`) and the cuas2 weapon (flywheel launcher kernel model + one-ball gate feeder plant), against the cuas-lite scene kinds (balloon, drifting balloon, hanging targets, quad 3 m/s, quad 8 m/s, a walking person). Every ball is judged by the ground-truth `Referee` of cuas-lite (person, hold, geofence, armed, cap, E-stop/key, watchdog, dead-man, stale detector, after-cut, hit-person).

Safety result: **0 violations in 1155 Monte-Carlo episodes (7 configurations x 165) and 13 named scenarios; the person episodes fed balls only with no person in the image.** Threat models are the generic kinematic test targets of cuas-lite; no new profiles.

## Reproduce

```
export NAPL_REMOTE_DIR=/workspace/wt/napl-c2integ
tools/remote.sh bg --slots 2 mc  'examples/cuas2/ablation.sh'     # seven configs, 165 episodes each, 4 workers, ~35-100 min per config
tools/remote.sh bg --slots 2 sc  'examples/cuas2/scenarios.sh'    # the 13 named scenarios, seed 1
python examples/cuas2/mc.py --report imm2=build/mc/imm2 noph=build/mc/noph ...   # cross-config tables
python examples/cuas2/run.py s1_balloon_static --seed 1 --brief                  # one scenario
```

Seeds: `mc.py --seed0 0`; episode i of a cell has seed `100000*(kind_index*3 + band_index) + i + 1` (person episodes `900000 + i + 1`), the same draw as `examples/cuas-lite/mc.py`; every configuration sees the identical 165 draws (paired). 10 episodes per kind x band cell (5 kinds x 3 bands = 150) + 15 person episodes. CIs are 95 % Wilson.

## Headline (baseline: imm2-gnn tracker, Ph >= 0.3), all kinds

| range band m | episodes | engaged | balls | hit/ball [95 % CI] | kills/episode [CI] | t first shot s (med/p90) | balls/kill | violations |
| ------------ | -------- | ------- | ----- | ------------------ | ------------------ | ------------------------ | ---------- | ---------- |
| 3-5          | 50       | 38      | 54    | 0.69 [0.55, 0.79]  | 0.74 [0.60, 0.84]  | 0.91 / 2.37              | 1.46       | 0          |
| 5-10         | 50       | 25      | 35    | 0.66 [0.49, 0.79]  | 0.46 [0.33, 0.60]  | 1.34 / 8.95              | 1.52       | 0          |
| 10-15        | 50       | 0       | 0     | n/a                | 0.00 [0.00, 0.07]  | -                        | -          | 0          |

By kind: balloon 0.94 hit/ball (18 balls), drift 0.94 (16), quad3 0.70 [0.48, 0.85] (20), hanging 0.40 [0.26, 0.56] (35), quad8 0 balls fired (engaged 0/30). Hanging targets are small and close to a wall: many balls per kill (2.5).

Read hit/ball with care: the Ph gate refuses poor shots, so hit/ball is conditional on a shot having been judged good. The like-for-like figures are kills/episode and balls/kill (next section).

### Against cuas-lite

cuas-lite (no Ph gate, flywheel scatter model v1): hit/ball 0.54 / 0.21 / 0.08 for 3-5 / 5-10 / 10-15 m, quad8 0 / 0.

| cuas2 configuration                                  | hit/ball 3-5      | 5-10              | 10-15             |
| ---------------------------------------------------- | ----------------- | ----------------- | ----------------- |
| lite (reference, not re-run)                         | 0.54              | 0.21              | 0.08              |
| cuas2, Ph gate off (`noph`, lite pointing tolerance) | 0.67 [0.54, 0.78] | 0.32 [0.23, 0.42] | 0.11 [0.06, 0.19] |
| cuas2, Ph gate on (`imm2`)                           | 0.69 [0.55, 0.79] | 0.66 [0.49, 0.79] | n/a (0 shots)     |

quad8 stays at 0 kills in every cuas2 configuration (0/30 episodes engaged with the gate on; 3 balls, 0 hits with it off). The 8 m/s crossing quad is not engaged by this loop (cause not isolated).

## Ablations (same 165 draws, 0 violations in all)

| config    | change                                                   | balls | hit/ball [CI]     | kills/episode [CI] | balls/kill | mean Ph of FIREs | person-episode balls |
| --------- | -------------------------------------------------------- | ----- | ----------------- | ------------------ | ---------- | ---------------- | -------------------- |
| imm2      | baseline: imm2-gnn, Ph >= 0.3                            | 89    | 0.67 [0.57, 0.76] | 0.40 [0.33, 0.48]  | 1.48       | 0.46             | 23                   |
| noph      | Ph gate off (lite pointing tolerance)                    | 242   | 0.32 [0.26, 0.38] | 0.51 [0.43, 0.59]  | 3.14       | 0.24             | 43                   |
| ph0       | Ph computed, threshold 0 (3 deg pointing bound only)     | 378   | 0.20 [0.17, 0.25] | 0.51 [0.43, 0.59]  | 4.91       | 0.11             | 54                   |
| abnn      | ab-nn tracker, Ph gate on                                | 0     | n/a               | 0.00 [0.00, 0.02]  | inf        | n/a              | 0                    |
| abnn_noph | ab-nn tracker, gate off                                  | 151   | 0.42 [0.35, 0.50] | 0.43 [0.35, 0.51]  | 2.36       | (0.01)           | 44                   |
| imm3      | imm3-gnn tracker, Ph gate on                             | 111   | 0.57 [0.47, 0.66] | 0.42 [0.34, 0.50]  | 1.76       | 0.46             | 31                   |
| imm2r05   | imm2-gnn, measurement noise sigma_px 1.5, range_rel 0.05 | 107   | 0.55 [0.46, 0.64] | 0.39 [0.32, 0.47]  | 1.81       | 0.47             | 31                   |

(kills/episode and balls are over the 150 target episodes; the 15 person episodes are listed separately: balls fed there, never with a person in the image.)

kills/episode by band (3-5 / 5-10 / 10-15 m):

| config    | 3-5  | 5-10 | 10-15             |
| --------- | ---- | ---- | ----------------- |
| imm2      | 0.74 | 0.46 | 0.00              |
| noph      | 0.74 | 0.60 | 0.20 [0.11, 0.33] |
| ph0       | 0.72 | 0.60 | 0.22 [0.13, 0.35] |
| abnn_noph | 0.74 | 0.54 | 0.00              |
| imm3      | 0.74 | 0.52 | 0.00              |
| imm2r05   | 0.76 | 0.42 | 0.00              |

What the ablations show (measured; the explanations are inferences):

- **The Ph gate trades kills for ammunition.** It cuts balls/kill from 3.14 (noph) / 4.91 (ph0) to 1.48 and raises hit/ball from 0.32 / 0.20 to 0.67, but kills/episode drops 0.51 to 0.40 (5-10 m 0.60 to 0.46; 10-15 m 0.20 to 0). The CIs of the totals overlap only barely (0.43-0.59 vs 0.33-0.48). The gate is conservative: **calibration is pessimistic**, mean Ph of the fired balls 0.46 against a realised 0.67 (see the table below). At 10-15 m the track covariance (range noise from the box size, `range_rel` 0.15) never gives Ph >= 0.3, so nothing is fired; with the gate off those same shots hit 0.11 and kill 0.20 per episode at 9 balls per kill.
- **Tracker.** imm3-gnn and imm2-gnn are not distinguishable here (kills 0.42 vs 0.40; hit/ball 0.57 vs 0.67, CIs overlap). ab-nn (alpha-beta, no covariance) cannot drive the Ph gate at all: it exports a fixed 1 m / 2 m/s covariance, Ph stays about 0.01 and `abnn` fires 0 balls in 150 episodes (verified: tracks confirmed, reason `ph`). That is a property of the tracker, not a fire-control bug; its fair arm is `abnn_noph`. With the gate off ab-nn has hit/ball 0.42 vs imm2 0.32 (`noph`, overlapping CIs), kills 0.43 vs 0.51.
- **Measurement noise** (`imm2r05`): matching the filter noise to the simulated detector (sigma_px 1.5, range_rel 0.05) did not raise Ph enough to change engagement (kills 0.39) and lowered hit/ball (0.55 vs 0.67); the 5-10 m kill rate fell to 0.42. Not a free lever; do not retune the gate on this evidence.

## Ph reliability (all configurations pooled, every ball fed; 927 balls)

| Ph bin  | balls | hits | hit/ball [95 % CI] | mean Ph |
| ------- | ----- | ---- | ------------------ | ------- |
| 0.0-0.1 | 354   | 27   | 0.08 [0.05, 0.11]  | 0.03    |
| 0.1-0.2 | 71    | 18   | 0.25 [0.17, 0.37]  | 0.14    |
| 0.2-0.3 | 56    | 16   | 0.29 [0.18, 0.41]  | 0.24    |
| 0.3-0.4 | 195   | 95   | 0.49 [0.42, 0.56]  | 0.34    |
| 0.4-0.5 | 73    | 54   | 0.74 [0.63, 0.83]  | 0.45    |
| 0.5-0.6 | 62    | 48   | 0.77 [0.66, 0.86]  | 0.54    |
| 0.6-0.8 | 81    | 53   | 0.65 [0.55, 0.75]  | 0.69    |
| 0.8-1.0 | 35    | 25   | 0.71 [0.55, 0.84]  | 0.86    |

Ph ranks shots correctly (0.08 at Ph < 0.1, about 0.5 at 0.3-0.4, 0.65-0.77 above 0.4) but saturates: above 0.5 it does not keep rising, and it under-predicts below it (0.3-0.4 bin: realised 0.49 vs mean Ph 0.34). The gate threshold of 0.3 therefore passes shots that hit about half the time.

Static-target closest approach (aim bias, all balls, 369 samples): mean x -0.078 m, y -0.028 m, **z -0.061 m (se 0.007)**, std y 0.20 / z 0.14 m. A consistent 6 cm low bias; lateral about 0. Cause not found (candidates: lift/lead model in the lite turret site, the spin-axis frame of the lite launcher site, camera/muzzle lag). No online aim-bias update after a shot is implemented (it would remove this).

## Named scenarios (`scenarios.sh`, seed 1; 0 violations in all 13)

| scenario                                     | shots             | hits | popped | checks                                                                                           |
| -------------------------------------------- | ----------------- | ---- | ------ | ------------------------------------------------------------------------------------------------ |
| s1_balloon_static                            | 2                 | 2    | 2/3    |                                                                                                  |
| s2_balloon_drift                             | 2                 | 1    | 1/2    |                                                                                                  |
| s3_hanging_targets                           | 2                 | 1    | 1/3    |                                                                                                  |
| s4_quad_3ms                                  | 0                 | 0    | 0/1    | Ph never >= 0.3                                                                                  |
| s4_quad_8ms                                  | 0                 | 0    | 0/1    |                                                                                                  |
| s5_mix                                       | 2                 | 1    | 1/3    |                                                                                                  |
| s6_person_walk                               | 6                 | 3    | 3/5    | `engaged_before_person` true; **`resumed_after_person` false** (the harness exits 1 on this one) |
| s6b_person_beside_balloon                    | 0                 | 0    | 0/1    | no_shots, target_was_engageable_but_for_the_person: true                                         |
| s7 dead-man / detector / E-stop / key / link | 1 / 1 / 0 / 0 / 1 |      |        | all fault checks true                                                                            |

`resumed_after_person` fails at seed 1: after the person leaves, the gate (`ph` reason) holds fire longer than the lite resume window. Safe, not a pass of that check; same root cause as the quad rows above (a conservative Ph).

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
- READY blinked about 1 time in 40 at the plant's 3 permille telemetry noise with the spec default rpm window 1 % (cfg 27 = 10), so a FIRE was refused (NOT_SPUN). The harness sets cfg 27 = 20 (2 %, the CFG_SET maximum), as `test_plant` does. Bench item: the window default or the rpm filter needs a look on real hardware.
- The lite `Referee` judged the cap-th ball against the app's armed flag after the same step in which the app disarms on that shot; 3 of the first 165 gate-off episodes read as `unarmed`. `Referee2` (`examples/cuas2/run.py`) judges against armed in the previous or current control period; a ball fed more than one period after a disarm is still flagged. The first imm2 run (0 violations, unaffected) was kept; noph, ph0 and later were rerun with the fix.
- Rail-cut fault injection order in `Weapon2.step`.

## Limitations (what this does not show)

- The scene is the cuas-lite turret assembly with the cuas2 launcher device: `assemblies/cuas2.toml` has no servo/camera/launcher devices. The gimbal, camera and servo numbers are v1's (spec R5), not a cuas2 gimbal. A real cuas2 turret will move these numbers.
- Ph samples a Gaussian at the impact time from `track.predict_to`, not the IMM mixture; no online aim-bias update after a shot; the closest-approach test uses a fixed target radius (0.15 m) for every class.
- rpm window 2 % (harness setting, see above); the dispersion model (`spread`, `v0_err`, cd) is the lite draw mapped onto the cuas2 plant calibration.
- No per-scenario `.scenario.toml` files and no web-UI snapshot support for cuas2 (existing scenario kinds only).
- Single seed per named scenario; the MC has 10 episodes per kind x band cell, so per-cell CIs are wide (see tables).
- Not done: HIL, video, a `t_actuator` sweep, and the stage-2 threats (on hold).
