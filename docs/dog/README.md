# Recon quadruped (the dog): results against the targets

SKELETON (SP6 draft). Cells marked "pending LOCO round 2" wait for the locomotion terrain round and the SP6 Monte Carlo that runs after it; every
other number is a measured one with its source. Nothing here is rounded in the dog's favour: a miss is a miss with its number.

## What the dog is

A 11.65 kg legged recon robot (`assemblies/dog/dog.toml`, built from napl's parts library, pin-level wiring): 12 GIM8108-8 quasi-direct-drive actuators (4 legs x
abduction, hip, knee), Livox Mid-360 lidar, RealSense D435i, RTK GNSS, IMU, Jetson Orin Nano, two 8S packs. Its software is a Python package of the robot's own
(`examples/dog/loco`: estimator, convex MPC, QP whole-body controller, gait and footholds; `examples/dog/autonomy`: elevation map, EKF with RTK and ICP,
coverage planner, executive). BOM 5,554 USD (SP3b build report, `docs/dog/speed-sweep.md`). Recon and mapping only: no weapon, no targeting of people.

## Spec section 7 against the measurements

| target (spec s7)                                                                                          | measured                                                                                                                                                                                                                                             | verdict                                                             | source                                                                                                                                       |
| --------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| flat top speed >= 2.0 m/s (lowered from 3.5 on 2026-10-18: the pack current caps the dog at 2.0-2.25 m/s) | 2.0 m/s held-out median and minimum, 2.25 on the training seed (the pack draws 114-142 A against its 120 A limit)                                                                                                                                    | MET, at the target with no margin                                   | `docs/dog/locomotion.md` "Spec criteria"                                                                                                     |
| rocky field mean speed >= 1.5 m/s                                                                         | no rubble run in SP4's terrain MC succeeded (0/20 on the first set, 0/40 on the fresh set), so there is no mean speed; the autonomy's `[dog]` limit is 0.4 m/s                                                                                       | NOT MET (pending LOCO round 2 for a new number)                     | `docs/dog/locomotion.md`, `docs/dog/autonomy.md` "On the dog"                                                                                |
| 25 deg slopes and 15 cm steps traversed                                                                   | criterion course 0/20 (WBC); the 15 deg ramp is climbed, 25 deg and 15 cm steps are not                                                                                                                                                              | NOT MET (pending LOCO round 2)                                      | `docs/dog/locomotion.md`                                                                                                                     |
| >= 90 % no-fall per terrain family, Wilson 95 % lower bound shown                                         | SP4 locomotion-only MC (n = 40, seeds 31000-31039): rubble 0/40, slopes 0/40, scree 2/40, ruins 1/40, criterion course 0/40. SP6 full-stack MC per family: pending LOCO round 2                                                                      | NOT MET on SP4's numbers; SP6 full-stack cells pending LOCO round 2 | `docs/dog/locomotion.md`; `docs/dog/missions.md` (sizing: N = 20 / 12 / 28 / 34, no family reaches a lower bound of 0.90 even with no falls) |
| 100 x 100 m coverage <= 20 min, RMS <= 5 cm, >= 95 % mapped, on rubble and ruins                          | pending LOCO round 2. On the kinematic carrier (no legs): rubble held-out coverage median 93.7 % (the 95 % target is missed on the carrier, by 0.1-8.6 points per seed); ruins missed (user decision: the 95 % target stays, the miss is documented) | NOT MET on the carrier; dog pending LOCO round 2                    | `docs/dog/autonomy.md` "Monte Carlo"                                                                                                         |
| BOM about 3-4.5 k USD                                                                                     | 5,554 USD                                                                                                                                                                                                                                            | NOT MET (over by 1,054 USD against the top of the range)            | `docs/dog/speed-sweep.md`, SP3b build report                                                                                                 |

What does hold on the dog today: mapping and localization (map RMS 2-4 cm, localization error 2-11 cm, no bumps at 0.4 m/s before the fall), a trot on flat
ground with push rejection and a safe sit, and tick-exact replays of every run (`docs/dog/autonomy.md` "On the dog").

## How the numbers are taken

- Two seed blocks, disjoint from each other and from SP4's and SP5's: tuning 5001-5030 (every parameter change is logged with its reason in `docs/dog/missions.md`),
  held-out 33001+ (flown once with the final code; the row records the code revision). SP4's terrain MC used 31000-31039, SP5's held-out 1001-1025.
- The full-stack MC (`examples/dog/missionmc.py`) runs the whole stack per seed: lidar on, SP5 planner, SP5's map handed to SP4 as its height map. A fall is the
  ground-truth `fallen` column, ever (`fell_ever`), not the last row and not the controller's own detector; crashed and timed-out runs stay in the denominator.
- N per family is set by the measured sim speed (0.2-0.35 sim-s per wall-s on the legged dog), arithmetic in `docs/dog/missions.md` "Budget".

## Known gaps

See `docs/dog/autonomy.md` (open items), `docs/dog/locomotion.md` "Known gaps". Also: self-righting is out of scope; the lidar scan pattern is the published
field of view and rosette, not the proprietary pattern; the GNSS outage model is a sky-view mask; coverage is scored on a 5k-ray pooled grid; slopes are not run at 100 m.

## Video

One video, three parts (construction, programming overview, simulation): `docs/dog/video.md`; recorder `tools/record_dog.py`. The video is never committed.

## Further work

GPU ray caster for the lidar, 6-DOF ICP, depth-camera fusion for the 0.72 m lidar blind zone, a cheaper actuator study.
