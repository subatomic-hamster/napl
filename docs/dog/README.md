# Recon quadruped (the dog): results against the targets

Every number below is measured and has a source; nothing is rounded in the dog's favour, and a miss is a miss with its number. The full-stack numbers are in `docs/dog/missions.json`
(a test pins this page to that file). Recon and mapping only: no weapon, no targeting of people.

## What the dog is

A 11.65 kg legged recon robot (`assemblies/dog/dog.toml`, built from napl's parts library, pin-level wiring): 12 GIM8108-8 quasi-direct-drive actuators (4 legs x
abduction, hip, knee), Livox Mid-360 lidar, RealSense D435i, RTK GNSS, IMU, Jetson Orin Nano, two 8S packs. Its software is a Python package of the robot's own
(`examples/dog/loco`: estimator, convex MPC, QP whole-body controller, gait and footholds; `examples/dog/autonomy`: elevation map, EKF with RTK and ICP,
coverage planner, executive). BOM 5,554 USD (SP3b build report, `docs/dog/speed-sweep.md`; part of the prices are estimated).

## Spec section 7 against the measurements

Full-stack Monte Carlo = SP6, whole dog (lidar on, planner, map handed to the controller), **held-out** seeds 71001+, flown once on rev `a884aa3`. _Dog_ = the legged robot; _carrier_ = the kinematic carrier without legs.

| target (spec s7)                                                  | measured                                                                                                                                                                                                                                                                                                                                | verdict                                                                                                    | source                                                          |
| ----------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| flat top speed >= 2.0 m/s                                         | 2.0 m/s held-out median and minimum, 2.25 on the training seed (the pack draws 114-142 A against its 120 A limit). The target was lowered from 3.5 m/s on 2026-10-18 by the user: the pack current caps the dog at 2.0-2.25 m/s                                                                                                         | MET, at the target with no margin                                                                          | `docs/dog/locomotion.md` "Spec criteria"                        |
| rocky field mean speed >= 1.5 m/s                                 | about 0.75 m/s mean over LOCO round 3's terrain MC (the controller caps non-flat height maps at 0.8 m/s); in the full-stack MC the median distance walked before the end of the run is 4-59 m                                                                                                                                           | NOT MET                                                                                                    | `docs/dog/locomotion.md` (LOCO round 3), `docs/dog/missions.md` |
| 25 deg slopes and 15 cm steps traversed                           | criterion course 0/40 (LOCO round 3, seeds 61000+); the 15 deg ramp is climbed, 25 deg and 15 cm steps are not; full-stack slopes family aborts at a median 2 s                                                                                                                                                                         | NOT MET                                                                                                    | `docs/dog/locomotion.md`, `docs/dog/missions.md`                |
| >= 90 % no-fall per terrain family                                | held-out full stack, no-fall (ground truth) k/n with Wilson 95 % lower bound: rubble 6/20 (0.15), slopes 2/12 (0.05), scree 11/28 (0.24), ruins 4/34 (0.05). Mission success (goal reached, home, no abort): rubble 0/20, slopes 0/12, scree 6/28, ruins 0/34. Aborted runs: 20, 12, 21, 34, of which with no truth fall 6, 2, 4, 4     | NOT MET in all four families (best rate 0.39); also N too small to show a bound of 0.90 even with no falls | `docs/dog/missions.md` "Held-out Monte Carlo"                   |
| 100 x 100 m rocky coverage <= 20 min, RMS <= 5 cm, >= 95 % mapped | **dog**: rubble 0/6 and ruins 0/4, every run fell or aborted within 6-300 s with 0.7-8.5 % mapped. **carrier** (no legs): rubble 5/6 meet all three (coverage 95.9-97.1 %, RMS 2.9-3.0 cm; the sixth got home 18 s late), ruins 0/4 (2.5-37 % mapped; the user decision stands: the 95 % target stays and the ruins miss is documented) | NOT MET on the dog; carrier: rubble MET in 5 of 6 runs, ruins NOT MET                                      | `docs/dog/missions.md` "100 m missions"                         |
| BOM about 3-4.5 k USD                                             | 5,554 USD (default dog); 4,855 USD with the recommended -al leg and GIM6010-8 (below)                                                                                                                                                                                                                                                   | NOT MET (over by 1,054 USD against the top of the range; 355 USD with the recommended build)               | `docs/dog/speed-sweep.md`, `docs/dog/leg-study.md`              |

What does hold on the dog today: mapping and localization when it stays up (map RMS 2-4 cm, localization error 2-11 cm), a trot on flat
ground with push rejection and a safe sit, and tick-exact replays of every run (`docs/dog/autonomy.md` "On the dog"). What does not: staying up on any of the four terrain families.

## Full-stack Monte Carlo per terrain family (held-out, dog)

| family | N   | no-fall k/n | Wilson 95 %  | mission success | aborted (no truth fall) |
| ------ | --- | ----------- | ------------ | --------------- | ----------------------- |
| rubble | 20  | 6/20        | [0.15, 0.52] | 0/20            | 20 (6)                  |
| slopes | 12  | 2/12        | [0.05, 0.45] | 0/12            | 12 (2)                  |
| scree  | 28  | 11/28       | [0.24, 0.58] | 6/28            | 21 (4)                  |
| ruins  | 34  | 4/34        | [0.05, 0.27] | 0/34            | 34 (4)                  |

Failures are locomotion falls (the cause SP4's terrain MC found; `docs/dog/locomotion.md`). No-fall (ground truth) and mission success are separate columns: an ABORT (the controller's own FALLEN detector) is a mission failure
even when the ground truth shows no fall. LOCO round 3 measured 0 latch false positives on 200 plain locomotion runs, yet 16 of 94 mission runs aborted with no truth fall; that is an open item (`docs/dog/missions.md`).
Locomotion was frozen for SP6; nothing was tuned to hide a fall.

## Hardware recommendation (the default dog assembly is not changed)

From `docs/dog/leg-study.md` and `docs/dog/leg-survey.md`: (1) build the **-al leg** (`dog-al.toml`: 6061 thigh shell, CF shank) rather than the printed one: the printed leg's lowest safety factor is 0.11 (yields at about 12 N side force), -al has 2.47, and the sim shows no penalty for the +0.81 kg;
(2) use **GIM6010-8 actuators** on it: BOM **4,855 USD** (-699 against 5,554, still +355 over the 4,500 target), 12.36 kg, flat push 15/16, v_top 2.00 m/s, fresh no-fall 0/80 (same as the baseline 1/80 within bounds).
The sim shows leg hardware is **not** what ends the runs (all variants stay at 95-99 % falls): the fix lies in the controller. These are structural and cost recommendations, not terrain results. Before buying: GIM6010 mounting, a pack-current check, and re-validation for any new leg length (L 0.25 -al is the next iteration).

## How the numbers were taken

- Two seed blocks, disjoint from each other and from SP4's and SP5's: **tuning** 5001-5030 (every parameter change is logged with its reason in `docs/dog/missions.md`; two rounds, one change: the ruins budget 420 to 700 s),
  **held-out** 71001+ (flown once with the final code; each row records the code revision). SP4's terrain MC used 31000-61000+, SP5's held-out 1001-1025.
- The full-stack MC (`examples/dog/missionmc.py`) runs the whole stack per seed: lidar on, SP5 planner, SP5's map handed to SP4 as its height map. A fall is the
  ground-truth `fallen` column, ever (`fell_ever`), not the last row and not the controller's own detector; crashed and timed-out runs stay in the denominator.
- N per family is set by the measured sim speed (0.2-0.35 sim-s per wall-s on the legged dog), arithmetic in `docs/dog/missions.md` "Budget": 20 / 12 / 28 / 34. No family could reach a Wilson lower bound of 0.90.
- User decisions: the flat top-speed target is 2.0 m/s (lowered from 3.5); the 95 % ruins coverage target is kept and its miss documented.

## Known gaps

See `docs/dog/autonomy.md` (open items), `docs/dog/locomotion.md` "Known gaps". Also: self-righting is out of scope; the lidar scan pattern is the published
field of view and rosette, not the proprietary pattern; the GNSS outage model is a sky-view mask; coverage is scored on a 5k-ray pooled grid; slopes are not run at 100 m;
the abort-without-fall runs are undiagnosed; the 100 m carrier rubble numbers (n = 6) differ from `docs/dog/autonomy.md`'s 93.7 % median and are not reconciled.

## Video

One video, three parts (construction, programming overview, simulation): `docs/dog/video.md`; recorder `tools/record_dog.py`. The video is never committed.

## Further work

Controller work on foothold and knee-range handling (the leg study's finding), GPU ray caster for the lidar, 6-DOF ICP, depth-camera fusion for the 0.72 m lidar blind zone.
