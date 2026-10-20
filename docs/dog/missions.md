# Dog missions: full-stack Monte Carlo per terrain family, 100 m headline missions (SP6)

The whole dog (SP3b hardware, SP4 locomotion `examples/dog/loco`, SP5 autonomy through `examples/dog/autonomy/dogloco.py`) on full-stack missions: lidar on, SP5 planner,
SP5's map handed to SP4 as its height map. Verdicts are ground truth (`summary.json` `fell_ever`, the max of the `fallen` column). The locomotion limits
that make most of these runs fall are in `docs/dog/locomotion.md` and `docs/dog/autonomy.md` ("On the dog"); nothing here is tuned to hide them.

## Budget

Measured on hfbox, C++ WBC, `OMP_NUM_THREADS=1`, 5k rays, pool 2, seed 5001 (tuning block), `AUTO_T_MAX=60` (cap of 60 sim-s), one job at a time
(`../logs/probe_<family>`). The wall time includes ~20 s of setup (field, model, estimator start), so `w` below is an upper bound per sim-s.

| family | outcome of the 60 s probe                                             | sim-s run | wall (s) | w = wall / sim-s        | source of w used for sizing                                                                                                                             |
| ------ | --------------------------------------------------------------------- | --------- | -------- | ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| rubble | fell at 5 s (phase 5 ABORT, coverage 7 %)                             | 5         | 24       | 4.8 (too short to read) | docs/dog/autonomy.md: 0.19-0.20 sim-s per wall-s on rubble, w = 5.1                                                                                     |
| slopes | fell at 2 s                                                           | 2         | 31       | not readable            | derived: carrier kernel 1 / 0.31 = 3.2 (autonomy.md Sim budget, slopes 5 m 5k) + the legs' ~4.8 (rubble dog 5.1 - carrier kernel 1 / 3.06 = 0.33) = 8.0 |
| scree  | fell at 40 s (coverage 27 %, 14 m walked)                             | 40        | 142      | 3.55                    | measured                                                                                                                                                |
| ruins  | no fall in 60 s (coverage 18.6 %, 20.2 m walked = 0.34 m/s, not home) | 60        | 179      | 2.98                    | measured                                                                                                                                                |

No probe reached home, so `T_budget` is not 1.3 x a measured time-to-home. It is a path / speed estimate: ~4 lanes of 30 m (lane spacing 9 m) plus fill and return = ~140 m, at the
0.34 m/s the dog averaged in the one probe that kept walking (the `[dog]` limit is 0.4 m/s): 140 / 0.34 = 412 s, **`t_budget` = 420 s** in all four `fam_*` scenarios (`estimated`).
Whether 420 s is enough for >= 95 % coverage is itself a finding of the MC, not assumed. (Tuning round 1 raised the **ruins** budget to 700 s; the N table below was sized at 420 s.
At 700 s the worst-case formula gives 20 for ruins, but runs that fall end early (in tuning 10 of 12 within 560 sim-s, most within 160 s), so N = 34 is kept: expected cost ~1-2 h on 4 jobs.)

**N per family = min(36, floor(6 h x 2 slots / (T_budget x w)))**, worst case (every run walks the full budget; runs that fall end early and cost much less):

| family | T_budget (s) | w    | T_budget x w (s) | 43200 / (T_budget x w) | **N**  | clean n/n Wilson lower bound n / (n + 3.84) |
| ------ | ------------ | ---- | ---------------- | ---------------------- | ------ | ------------------------------------------- |
| rubble | 420          | 5.1  | 2142             | 20.2                   | **20** | 0.84                                        |
| slopes | 420          | 8.0  | 3360             | 12.9                   | **12** | 0.76                                        |
| scree  | 420          | 3.55 | 1491             | 29.0                   | **28** | 0.88                                        |
| ruins  | 420          | 2.98 | 1252             | 34.5                   | **34** | 0.90 (0.8985)                               |

36 is the smallest N at which a clean 36/36 has a Wilson lower bound >= 0.90 (36 / 39.84 = 0.904). **No family reaches it** (largest N is 34, 0.8985): with these N the
95 % lower bound of the no-fall rate cannot reach 90 % even with no falls, and the README states that next to each rate. Slopes (N = 12) and rubble (N = 20) are far below.
The 100 x 100 m headline missions are not run here (Task 5).

## Tuning log

Seed blocks: **tuning 5001-5030** (rounds below use 5001-5012), **held-out 71001+** (flown once, final code, `--rev` recorded in every row). The blocks 1-30, 1000-1025,
21000+, 31000+, 41000+, 51000+ and 61000+ belong to SP4 and the training runs (a test pins the disjointness), so the plan's 31001+ moved to 71001+.

Locomotion is frozen (LOCO round 3), so the tuning here is diagnosis plus the one mission parameter that is not locomotion. Nothing was tuned to hide a fall.

| round | family   | seeds / rev        | symptom                                                                                           | cause                                                                                                                                                                                                                                                            | change                                                      | result                                                                                                                                                                                                           |
| ----- | -------- | ------------------ | ------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | all four | 5001-5012, fdcacc9 | no-fall: rubble 3/12, slopes 1/12, scree 6/12, ruins 3/12; mission success 0/12, 0/12, 4/12, 0/12 | locomotion falls (same cause as SP4's MC; slopes abort at 2 s on every seed). Ruins runs that kept walking were at ~70 % coverage when the 420 s budget ended: 5001 and 5006 walked 146-161 m at 0.34 m/s, the maze path is ~1.7x the 140 m open-field estimate. | none                                                        | baseline                                                                                                                                                                                                         |
| 1     | ruins    | 5001-5012, 4a445cd | as round 0                                                                                        | budget-limited coverage in the non-falling runs                                                                                                                                                                                                                  | `fam_ruins` `t_budget` 420 -> 700 s (estimated -> measured) | deterministic: 10 of 12 runs identical to round 0; 5001 now ends ABORT at 488 s (0.70 coverage, 169 m, no ground-truth fall), 5006 walked on to 558 s (0.77) and fell. No-fall stays 3/12, mission success 0/12. |

Round 1 did not raise success. It is kept because the 420 s figure was an estimate the data contradict, and a longer budget only lets the runs that survive show how far they get; the cost is
that a longer mission gives falls more time to happen (5006 fell at 558 s, after the old budget would have sent it home late). No further round: the remaining failures are falls in the first
10-150 s, which is the locomotion limit in `docs/dog/locomotion.md`, not something a mission parameter fixes.

**Aborts without a ground-truth fall.** Some runs end in phase 5 (ABORT) although `fell_ever` is 0: the controller's own FALLEN detector fired in the mission setting.
Tuning round 0, aborted with no truth fall: rubble 3/12 (5001, 5009, 5011), slopes 1/12 (5006), scree 2/12 (5002, 5012), ruins 2/12 (5002, 5010); round 1 adds ruins 5001.
LOCO round 3 measured 0 latch false positives on 200 plain locomotion runs, so these appear only with the mission stack (lidar map as height map, planner commands).
Under the plan's ground-truth rule they count as no-fall, but an aborted run is a mission failure and never counts toward mission success; the tables carry no-fall, mission success and the aborted count as separate columns.

## Held-out Monte Carlo

Held-out seeds 71001+, flown once on rev `a884aa3` (the committed final code; every row records the revision), body = **dog** (legged, full stack). Raw output: `mc.md` / `mc.json` of that run;
the numbers below are copied into `docs/dog/missions.json`.

| family | N   | no-fall k/n | rate | Wilson 95 %  | mission success k/n | aborted (of which no truth fall) | failures (crashed fell coverage rms not_home late) | median coverage / RMS / time / distance |
| ------ | --- | ----------- | ---- | ------------ | ------------------- | -------------------------------- | -------------------------------------------------- | --------------------------------------- |
| rubble | 20  | 6/20        | 0.30 | [0.15, 0.52] | 0/20                | 20 (6)                           | 0 14 20 7 20 0                                     | 13.2 % / 4.3 cm / 15 s / 4 m            |
| slopes | 12  | 2/12        | 0.17 | [0.05, 0.45] | 0/12                | 12 (2)                           | 0 10 12 11 12 0                                    | 6.3 % / 21.7 cm / 2 s / 2 m             |
| scree  | 28  | 11/28       | 0.39 | [0.24, 0.58] | 6/28                | 21 (4)                           | 0 17 19 5 22 1                                     | 66.3 % / 2.9 cm / 162 s / 59 m          |
| ruins  | 34  | 4/34        | 0.12 | [0.05, 0.27] | 0/34                | 34 (4)                           | 0 30 34 6 34 0                                     | 17.1 % / 3.4 cm / 72 s / 21 m           |

No run crashed or timed out; the longest took 1569 s wall of the 3600 s (slopes 5400) limit.

- **No-fall** is ground truth (`fell_ever`). **Mission success** needs coverage >= 95 %, map RMS <= 5 cm, home in the budget and no ABORT; an aborted run never counts. They are separate columns on purpose:
  scree is the only family with any success (6/28, Wilson [0.10, 0.40]); its 11 no-fall runs include 4 that the controller itself aborted with no truth fall.
- **Aborts without a truth fall** in the mission setting: rubble 6, slopes 2, scree 4, ruins 4 (16 of 94 runs). Every ruins no-fall run (4/4) ended in such an abort. LOCO round 3 measured 0 latch false positives on 200 plain locomotion runs;
  these aborts appear only with the mission stack. They were not tuned away, and whether they are real near-falls the ground truth did not catch (a stumble recovered by the sim) or the mission map/commands provoking the detector is **not
  diagnosed**; open item.
- The **95 % no-fall target is missed in every family**: the best point rate is 0.39 (scree). With these N a clean run could not have shown a lower bound of 0.90 either (largest N = 34, 0.8985).
- Against SP4's locomotion-only MC on seeds 61000+ (n = 40: rubble 12/40, slopes 0/40, scree 26/40, ruins 17/40) the full-stack no-fall rates are lower on rubble (6/20 vs 12/40, within bounds) and ruins (4/34 vs 17/40); scree 11/28 vs 26/40 and slopes 2/12 vs 0/40. Different seeds and different
  mission lengths (ruins 700 s here), same controller; the cause of the ruins gap is not diagnosed (inference: a longer, planner-driven walk gives falls more time).

## 100 m missions

100 x 100 m, 20 min budget, seeds 71001+, run with `examples/dog/automc.py` (one process per seed). **The body is named in every line**: _carrier_ = the kinematic carrier (no legs, SP5's validated pipeline), _dog_ = the legged robot.
No dog run reached 1200 s of mission (all ended by 300 s), so the 3 h wall cap was never reached; no cap was needed.

| mission                       | body        | seeds       | success (cov >= 95 %, RMS <= 5 cm, home <= 1200 s) | fell (ground truth)                | median coverage | per seed (coverage; ended at)                                                                         |
| ----------------------------- | ----------- | ----------- | -------------------------------------------------- | ---------------------------------- | --------------- | ----------------------------------------------------------------------------------------------------- |
| rubble, `coverage_rubble_dog` | **dog**     | 71001-71006 | 0/6                                                | 6/6                                | 3.8 %           | 2.4 % at 40 s; 8.5 % at 139 s; 1.5 % at 22 s; 5.2 % at 97 s; 5.7 % at 108 s; 0.7 % at 6 s (all ABORT) |
| ruins, `dog_ruins`            | **dog**     | 71001-71004 | 0/4                                                | 2/4 (+2 aborts with no truth fall) | 2.7 %           | 4.1 % at 181 s (fell); 4.7 % at 297 s (abort); 1.3 % at 80 s (abort); 1.2 % at 68 s (fell)            |
| rubble, `coverage_rubble`     | **carrier** | 71001-71006 | 5/6 (71002 got home at 1217.9 s, 18 s late)        | 0/6                                | 96.5 %          | 96.4, 96.8, 96.6, 96.0, 95.9, 97.1 %; RMS 2.9-3.0 cm                                                  |
| ruins, `coverage_ruins`       | **carrier** | 71001-71004 | 0/4                                                | 0/4                                | 27.6 %          | 37.3 %, 34.9 % (RMS 5.8 cm, not home), 20.3 %, 2.5 % (RMS 6.4 cm, not home)                           |

- The dog does not complete either 100 m mission: it falls or aborts within 6-300 s having mapped under 9 %. The carrier shows what the autonomy stack does without legs: rubble meets the targets in 5 of 6 held-out runs;
  ruins does not (the user decision stands: the 95 % target is kept and the miss documented; the carrier maps 2-37 % of the maze in 20 min).
- The carrier rubble numbers differ from `docs/dog/autonomy.md` "Monte Carlo" (held-out median 93.7 %, target missed on every seed there): different seeds (71001+ vs 1001-1025), n = 6, current code; the two sets are not reconciled here.
- The spec row "100 x 100 m coverage" is **NOT MET on the dog** (0/6 and 0/4) and NOT MET on the carrier for ruins.
- Seeds: `automc.py` takes `lo:hi` with an exclusive upper bound, so the first call covered fewer seeds than intended; 71006 (rubble dog) and 71004 (ruins dog) were run as separate calls (same code, same scenario files). All seeds are 71001+ and none was rerun.
