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

(held-out block 71001+, flown once with the final code.)

## 100 m missions

(rubble `coverage_rubble_dog`, ruins `dog_ruins`)
