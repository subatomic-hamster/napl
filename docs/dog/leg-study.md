# Leg study: are stronger leg components what the dog is missing?

User request (2026-10-18): "the reason why the robot could be failing could be the instability in the legs. If we had stronger leg components,
this could be resolved automatically. Try out different leg components to see if structural changes help the overall product."

Tools: `examples/dog/legdiag.py` (per-tick actuator diagnosis, same evaluation for every variant), `examples/dog/legreport.py` (tables),
`tools/dog_sweep.py --emit` (variant assemblies from real parts). Every variant flies the napl-v1 controller unchanged (MPC + C++ QP WBC, J^T stance,
height schedule); only the assembly differs (`NAPL_DOG_ASM`). Raw rows: `logs/leg-eval/*.jsonl` (training seeds), `logs/leg-fresh/*.jsonl` (fresh seeds) on hfbox.

## Result in one paragraph

Hypothesis "stronger actuators fix the falls": **not supported.** Real stronger actuators (AK70, AK80, AK10, RobStride 03, mixed) did not reduce falls and
made the flat push test worse (heavier legs). An idealised oracle actuator (GIM8108 with 2.5x torque and 2x speed, same mass, same price) removed the
torque saturation (17 % -> 2 % of ticks) and still left 3/40 terrain runs up against 1/40 for the baseline: saturation accompanies the falls but is not what causes them.
What did move the numbers is **geometry**: a longer leg (0.25 m) and a wider knee range (knee linkage ratio 1.0 instead of 1.5). The best cheap options use the
**lighter, cheaper GIM6010-8** (BOM 4,570-4,572 USD instead of 5,554): on 80 fresh terrain runs they went 4/80 and 2/80 no-fall against 1/80 for the baseline, with the
terrain still failing in nearly every run (95-99 %). The dog is not fixed by any leg-hardware variant tried here; the remaining cause is controller / planning.

## Hypothesis and limits

H0: falls are limited by the legs' torque / speed headroom, so stronger actuators remove them. MuJoCo links are rigid, so "strength" here means torque and speed headroom, gear ratio,
mass / inertia, leg geometry (length, joint range) and contact. Structural bending / yield is **not** simulated and no bending-stress margin check was run (optional in the brief; not done).
Foot pad: slip is counted in the diagnosis (0-3 events per family over 10-20 runs, none in the 0.5 s before collapse for rubble / slopes / ruins), so no foot-pad variant was built.

## Step 1: diagnosis of the baseline (12x GIM8108-8, ratio 1.5, L 0.22 m)

Training seeds 1000-1009 per family (10 each), flat push seeds 21000-21015 (16). For each fall: `W_on` = the 0.5 s before the collapse onset (last tick with trunk tilt < 15 deg before the
fall, at most 3 s earlier). "Saturated" = any joint at >= 95 % of the torque the driver can deliver at that tick's joint speed and bus voltage (torque and speed limit in one number).
Rule (ours): actuator-limited if saturated on >= 10 % of `W_on` ticks or pack current > 120 A on >= 5 %; else kinematic if a joint is within 0.03 rad of a range stop on >= 10 %; else control / planning.

| family    | n   | falls | actuator-limited | kinematic | control | saturated in W_on | saturated in normal operation | range stop in W_on | knee above corner speed in W_on | slips (falls) | WBC relaxed ticks (W_on) |
| --------- | --- | ----- | ---------------- | --------- | ------- | ----------------- | ----------------------------- | ------------------ | ------------------------------- | ------------- | ------------------------ |
| rubble    | 10  | 10    | 9                | 0         | 1       | 50 %              | 25 %                          | 80 %               | 35 %                            | 0             | 550                      |
| slopes    | 10  | 10    | 1                | 9         | 0       | 4 %               | n/a (fall at start)           | 28 %               | 2 %                             | 0             | 45                       |
| scree     | 10  | 9     | 9                | 0         | 0       | 54 %              | 16 %                          | 89 %               | 31 %                            | 3             | 780                      |
| ruins     | 10  | 10    | 8                | 2         | 0       | 35 %              | 12 %                          | 78 %               | 23 %                            | 0             | 494                      |
| flat push | 16  | 3     | 3                | 0         | 0       | 71 %              | 14 %                          | 97 %               | 36 %                            | 1             | 490                      |

Flat runs that stayed up are saturated on 2 % of ticks; pack peak current 131 A (limit 120 A) over the training set. By our rule the answer is "mostly actuator-limited" (35 of 41 terrain falls plus
all 3 flat falls) and slopes are kinematic / start-up falls (every slopes run falls near t = 0, progress about -0.01). But the rule only measures correlation: the oracle runs below show that
removing the saturation does not remove the falls. The range-stop column (78-97 %) says the legs are at a joint limit when the trunk goes, i.e. the leg runs out of reach or knee range.
Classification therefore reads: terrain falls are mainly **kinematic reach / control-planning with the torque saturation as a symptom**.

## Step 2: variants (real consumer parts; numbers rule)

Actuator figures are the part files' datasheet values (output side); prices are the part files' prices (USD, ex shipping, seen 2026-10-16 / 18; RobStride 03 from a retailer, backorder). All new parts pass `tools/check_parts.py`.
Probes (GIM6010, AK70, AK80, AK10, RS03) put the actuator's electrics, mass and price in the GIM8108-8 seats; no printed mounts were redrawn for them, so mounting is not proven.

| actuator                       | peak N m | no-load rad/s | mass kg | price USD | source (part file)                                                                          |
| ------------------------------ | -------- | ------------- | ------- | --------- | ------------------------------------------------------------------------------------------- |
| SteadyWin GIM8108-8 (baseline) | 22       | 33.5          | 0.396   | 213.99    | parts/actuators/steadywin-gim8108-8.toml (openelab.com)                                     |
| SteadyWin GIM6010-8            | 11       | 44.0          | 0.388   | 131.99    | parts/actuators/steadywin-gim6010-8.toml (openelab.com, 12-56 V)                            |
| CubeMars AK70-9 V3 KV60        | 29.2     | 33.5          | 0.540   | 398.90    | parts/actuators/cubemars-ak70-9-v3-kv60.toml                                                |
| CubeMars AK80-9 V3 KV100       | 22       | 59.7          | 0.490   | 479.90    | parts/actuators/cubemars-ak80-9-v3-kv100.toml                                               |
| CubeMars AK10-9 V3 KV60        | 53       | 33.5          | 0.940   | 698.90    | parts/actuators/cubemars-ak10-9-v3-kv60.toml                                                |
| RobStride 03                   | 60       | 20.4          | 0.880   | 245       | parts/actuators/robstride-03.toml (aifitlab.com)                                            |
| oracle orcT*S* (not a part)    | 22 T     | 33.5 S        | 0.396   | 213.99    | GIM8108 with torque x T, speed x S, same mass and price: upper bound on "stronger / faster" |

Geometry: knee linkage ratio 1.0 / 1.5 / 2.0 with the real crank / rod / lug parts (`dog-*-k10`, `-k20`); leg length 0.20 / 0.22 / 0.25 m (existing length parts); "exact" = knee feed-forward range scaled by the ratio rather than the 2.0 convention.
Ratio 2.0 with real geometry **cannot be flown**: the four-bar keeps both transmission angles in 30-150 deg only over a knee range of [-1.02, 0] rad (baseline [-1.32, 0.02], ratio 1.0 [-1.995, 0.005]), and the controller's
IK finds no valid knee on it (`ValueError: neither knee direction bends the designed way`, all 57 runs). Ratio 2.0 is kinematically infeasible for this controller, which fits the range-stop finding above.
Link mass / inertia: only through the actuator mass (AK / RS variants are 1.1-6.5 kg heavier). A lighter, stiffer structure is the separate LEGSURVEY work (see Pending).

## Step 3: results on the training seeds (n = 10 per family, flat 16, same seeds as the baseline)

No-fall k/40 = sum over rubble, slopes, scree, ruins. Progress = mean fraction of the route covered. Flat = flat push up k/16 (seeds 21000-21015). v_top = flat top-speed ladder (target 2.0 m/s). Saturation = ticks with any joint saturated (terrain / flat).
Pack peak = max pack current (BMS limit 120 A). Wilson 95 % for k/40: 0 -> 0-9 %, 1 -> 0.4-13 %, 3 -> 3-20 %, 5 -> 5-26 % (per family n = 10: 0 -> 0-28 %, 1 -> 2-40 %, 3 -> 11-60 %).

| variant                                          | mass kg | BOM USD | no-fall k/40                       | progress | flat  | v_top | saturation  | pack peak A |
| ------------------------------------------------ | ------- | ------- | ---------------------------------- | -------- | ----- | ----- | ----------- | ----------- |
| baseline gim8108 r1.5 L220                       | 11.65   | 5,554   | 1                                  | 0.20     | 13/16 | 2.25  | 17 % / 4 %  | 131         |
| gim8108 r1.5 L220 exact ktau                     | 11.65   | 5,554   | 1                                  | 0.20     | 13/16 | 2.50  | 17 % / 3 %  | 116         |
| gim8108 r1.0 L220 exact                          | 11.65   | 5,554   | 4                                  | 0.31     | 15/16 | 2.25  | 6 % / 1 %   | 131         |
| gim8108 r2.0 L220                                | 11.66   | 5,553   | fails to build a gait (knee range) | -        | -     | -     | -           | -           |
| gim8108 r1.5 L200                                | 11.63   | 5,552   | 0                                  | 0.13     | 3/16  | 2.00  | 25 % / 10 % | 132         |
| gim8108 r1.5 L250                                | 11.69   | 5,556   | 3                                  | 0.27     | 16/16 | 2.25  | 11 % / 0 %  | 169         |
| gim6010 r1.5 L220                                | 11.56   | 4,570   | 1                                  | 0.19     | 15/16 | 2.00  | 26 % / 2 %  | 120         |
| ak70 r1.5 L220                                   | 13.38   | 7,773   | 1                                  | 0.17     | 8/16  | 2.75  | 19 % / 3 %  | 132         |
| ak80 r1.5 L220                                   | 12.78   | 8,745   | 2                                  | 0.17     | 1/16  | 2.50  | 20 % / 33 % | 122         |
| ak10 r1.5 L220                                   | 18.18   | 11,373  | 2                                  | 0.16     | 6/16  | 2.25  | 8 % / 9 %   | 240         |
| rs03 r1.5 L220                                   | 17.46   | 5,926   | 0                                  | 0.10     | 0/16  | 2.00  | 22 % / 13 % | 131         |
| gim8108 + ak70 hip + ak70 knee, r1.5             | 12.81   | 7,033   | 1                                  | 0.19     | 7/16  | 2.25  | 12 % / 1 %  | 132         |
| gim8108 + ak10 hip + ak10 knee, r1.5             | 16.01   | 9,433   | 0                                  | 0.10     | 8/16  | 2.25  | 13 % / 6 %  | 194         |
| oracle T1 S2                                     | 11.65   | 5,554   | 2                                  | 0.20     | 9/16  | 2.25  | 6 % / 3 %   | 297         |
| oracle T2.5 S1                                   | 11.65   | 5,554   | 2                                  | 0.20     | 12/16 | 2.00  | 11 % / 12 % | 294         |
| oracle T2.5 S2                                   | 11.65   | 5,554   | 3                                  | 0.20     | 12/16 | 2.25  | 2 % / 1 %   | 641         |
| gim6010 r1.0 L220 exact                          | 11.55   | 4,570   | 2                                  | 0.31     | 16/16 | 2.25  | 20 % / 2 %  | 103         |
| gim6010 r1.5 L250                                | 11.60   | 4,572   | 4                                  | 0.34     | 16/16 | 2.25  | 21 % / 0 %  | 117         |
| gim8108 hip, gim8108 abad, ak70 knee, r1.0 exact | 12.22   | 6,294   | 5                                  | 0.30     | 16/16 | 2.25  | 10 % / 1 %  | 140         |

(The `base.jsonl` file is an earlier run of the baseline from before `prog` was logged; the baseline row above is `gim8108-r1.5-L220`, identical per-run fall verdicts.)

Paired with the baseline (same seeds): stronger real actuators broke flat runs (ak70 5 broken of 16, ak80 12, ak10 7, rs03 13) and fixed at most 1 terrain run each; the oracle with 2.5x torque and 2x speed fixed 2 terrain runs
and broke 2 flat runs. The geometry variants fixed 1-3 terrain runs in total (scree, rubble) and 2-3 flat runs, broke none. The slopes family fails at the start for every variant (progress about -0.01 to -0.02).

## Step 4: fresh seeds 51000-51019 (n = 20 per family, no tuning), baseline on the same seeds

Chosen for no-fall and progress gain per added cost (both are cheaper than the baseline and lighter): gim6010 r1.5 L250 (best progress, 4/40) and gim6010 r1.0 L220 exact (2/40, progress 0.31 as on the GIM8108 version, the widest knee range). The 5/40 mixed-actuator variant costs +740 USD and was not chosen.

| variant                 | BOM USD | mass kg | rubble        | slopes        | scree         | ruins         | total no-fall (Wilson 95 %) | mean progress | pack peak A |
| ----------------------- | ------- | ------- | ------------- | ------------- | ------------- | ------------- | --------------------------- | ------------- | ----------- |
| baseline                | 5,554   | 11.65   | 0/20 (0-16 %) | 0/20 (0-16 %) | 1/20 (1-24 %) | 0/20 (0-16 %) | 1/80 (0.2-6.7 %)            | 0.17          | 128         |
| gim6010 r1.5 L250       | 4,572   | 11.60   | 0/20 (0-16 %) | 0/20 (0-16 %) | 4/20 (8-42 %) | 0/20 (0-16 %) | 4/80 (2.0-12.2 %)           | 0.24          | 149         |
| gim6010 r1.0 L220 exact | 4,570   | 11.55   | 0/20 (0-16 %) | 0/20 (0-16 %) | 2/20 (3-30 %) | 0/20 (0-16 %) | 2/80 (0.7-8.7 %)            | 0.26          | 108         |

Paired progress (baseline -> variant, sign test p on per-run progress): gim6010 r1.5 L250: rubble 0.137 -> 0.249 (p 0.003), scree 0.338 -> 0.526 (p < 0.001), ruins 0.205 -> 0.222 (p 0.26), slopes about -0.02 (no change);
gim6010 r1.0: rubble 0.137 -> 0.342 (p 0.012), scree 0.338 -> 0.498 (p 0.12; 2 fixed, 1 broken), ruins 0.205 -> 0.236 (p 0.82). The no-fall counts overlap within Wilson bounds (1/80 vs 4/80), so the no-fall gain
is **not significant**; the progress gain on rubble and scree is. Pack peak current: baseline 131 A (training) / 128 A (fresh), above the 120 A BMS limit already; gim6010 L250 149 A fresh (gim8108 L250: 169 A training), so the L250 pack current needs a BMS check; gim6010 r1.0 stays at 108 A.

## Verdict

1. Leg actuator strength is not the cause of the failures. Stronger real actuators do not help and the heavier ones hurt (flat push 0-8/16 versus 13/16); an ideal 2.5x torque / 2x speed oracle gives 3/40 versus 1/40 (not distinguishable).
2. Geometry helps a little: more reach (L 0.25) or more knee range (ratio 1.0) raise route progress on rubble and scree by 0.1-0.2 and lift flat push from 13/16 to 15-16/16. Shorter legs (0.20) are worse. Ratio 2.0 cannot be built with the real linkage (knee range 1.02 rad).
3. The GIM6010-8 matches or beats the GIM8108-8 in this controller at 982 USD less BOM. Inference (not tested): the lower peak torque costs nothing because the saturation is not what ends the runs.
4. The terrain MC stays at 95-99 % falls for every variant, so the fix lies in the controller (planning / foothold / knee-range handling) and not in leg hardware.

## Recommendation (the default dog assembly is not changed)

If the BOM target matters: switch the 12 actuators to GIM6010-8 and raise the leg to 0.25 m (**gim6010 r1.5 L250**): BOM 4,572 USD, -982 USD against 5,554 and +72 USD over the 4,500 target (4,500 is
reachable only with another ~72 USD saved, e.g. cheaper structure / electronics; not designed here); mass 11.60 kg (-0.05). Needs: GIM6010 mounting (the probe uses the GIM8108 seats), pack current check at L250, re-validation of every controller number for
the new leg length. Do not buy AK / RobStride actuators for stability: +370 (RS03) to +5,800 (AK10) USD and +1.1 to +6.5 kg for no gain; the two cheapest (mixed AK70, RS03) were worse on flat push.

## Sizing and wall time

Runs: 57 per variant on the training set (40 terrain + 16 flat + 1 top ladder), 4 jobs, 9-13 min each; 18 variant evaluations incl. the failed ratio 2.0 and the baseline = about 3.4 h of that, plus the fresh set
(3 x 80 terrain runs, 10 min each, 0.5 h) and the diagnosis from the baseline's own rows. Total wall about 4.5 h including tests. One more variant on the training set costs about 12 min; one on fresh seeds about 10 min (room for the structure variant).

## Pending

The LEGSURVEY structure variant (aluminium / CF thigh, CF shank) and its combination with the best actuator option will be added to the training-seed pairing and the fresh-seed confirmation once branch p12-legsurvey is merged.

## Checks run

`tools/check_parts.py`: 821 parts, 10841 values, 0 problems. `tools/check_licenses.py`: 83 files, 0 problems. pytest `test_dog_sweep.py test_dog_parts.py test_dog_assembly.py test_dog_legdiag.py` (-n 4): 141 passed.
