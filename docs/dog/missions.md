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
Whether 420 s is enough for >= 95 % coverage is itself a finding of the MC, not assumed.

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

(filled by the tuning rounds on seeds 5001-5030)

## Held-out Monte Carlo

(held-out block 31001+, flown once with the final code)

## 100 m missions

(rubble `coverage_rubble_dog`, ruins `dog_ruins`)
