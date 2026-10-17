# Dog autonomy (SP5)

The robot's autonomy software is the package `examples/dog/autonomy/` (numbers in `autonomy.toml`, numbers rule). Tasks 1-10 run it on a
kinematic carrier (a pose-commanded body with the same sensors); Task 11 (SP3b + SP4) puts it on the dog. Recon and mapping only.

## Sim budget

`tools/bench_autonomy.py --sim 5.0` (hfbox, one job, the kinematic carrier: rosette lidar at 10 Hz + RTK GNSS, no legs, driving 1.5 m/s
with a slow turn over a 100 x 100 m field, `max_range` 40 m, terrain seed 3). Sim-seconds per wall-second, kernel only:

| kind   | patch | 2.5k  | 5k   | 10k  | 20k  |
| ------ | ----- | ----- | ---- | ---- | ---- |
| rubble | 10 m  | 4.33  | 2.20 | 1.13 | 0.57 |
| rubble | 5 m   | 5.91  | 3.06 | 1.59 | 0.81 |
| slopes | 10 m  | 0.29  | 0.14 | 0.07 | 0.04 |
| slopes | 5 m   | 0.63  | 0.31 | 0.16 | 0.08 |
| scree  | 10 m  | 9.64  | 4.95 | 2.54 | 1.27 |
| scree  | 5 m   | 12.30 | 6.43 | 3.33 | 1.68 |
| ruins  | 10 m  | 2.71  | 2.25 | 1.73 | 1.17 |
| ruins  | 5 m   | 2.64  | 2.18 | 1.60 | 1.06 |

Compared with `docs/models.md` "Ground sim budget" (10 m patches, dog physics on top): the carrier is faster on rubble (2.2 vs 1.27 at
5k), scree (4.95 vs 2.81) and slopes (0.14 vs 0.13), but slower on ruins (2.25 vs 4.56 at 5k; cause not investigated, the carrier starts in a walled cell). Unlike the doc's note that 10 m patches are 2.6-6x faster than one hfield, **5 m patches are 1.4x (rubble), 2.2x (slopes) and 1.3x
(scree) faster than 10 m patches on the carrier**, and equal on ruins (the walls are boxes, the ground is flat). Noise level of
the table: single 5 sim-s runs, other jobs on the box (load ~2): read to +-10 %.

## Wall-time model

`wall = T_mission (1 / s_kernel + c_py)`, `c_py` = the autonomy's Python per sim-second.

Measured `c_py` (Task 9, `coverage_smoke`, 20 x 20 m rubble, 5k rays, 1401 frames, wall ms per 10 Hz frame):

| module                           | ms / frame |
| -------------------------------- | ---------- |
| points (levelled, angular noise) | 0.47       |
| ICP match + keyframe add         | 0.96       |
| elevation-map fusion             | 0.19       |
| traversability (windowed)        | 4.50       |
| planning + pure pursuit + fix    | 0.69       |
| scoring (harness, 1 per 10 s)    | 0.03       |

Total 6.8 ms/frame -> **`c_py` = 0.07 wall-s per sim-s** (the plan budgeted 0.25). This is measured on a 20 m map; on the
100 x 100 m map the 200 x 200 block grid makes A* / Dijkstra and `mapped()` heavier (re-measured by the 100 m missions, see Coverage).
Mission of 1200 sim-s with the measured `s_kernel` of the better patch size and `c_py = 0.07`:

| kind   | rays | patch | s_kernel | 1200 s mission wall                |
| ------ | ---- | ----- | -------- | ---------------------------------- |
| rubble | 5k   | 5 m   | 3.06     | 7.9 min                            |
| rubble | 20k  | 5 m   | 0.81     | 26 min                             |
| ruins  | 5k   | 10 m  | 2.25     | 10.3 min                           |
| ruins  | 20k  | 10 m  | 1.17     | 18.6 min                           |
| scree  | 5k   | 5 m   | 6.43     | 4.5 min                            |
| slopes | 5k   | 5 m   | 0.31     | 66 min (not run as a full mission) |

## Decision: rays and patches

- Patch size is the faster of 10 m / 5 m per kind: **rubble, scree, slopes 5 m; ruins 10 m** (the ruins difference is inside the noise,
  10 m kept). The patch is a `field()` argument; heights and boxes are identical, only the material grid (friction) differs, which the
  kinematic carrier does not use.
- Missions and Monte Carlo run at `lidar.rays_mission` = **5000** rays / frame and are scored on the pooled grid (`plan.pool` = 2,
  10 cm scoring cells; `20000 / 0.05^2 = 5000 / 0.10^2`). The rule "if rubble 5k measures below 1.0 sim-s per wall-s, use 2.5k and
  pool 3" is **not triggered** (rubble 5k = 2.20 / 3.06). Task 10's density test pins the equivalence.
- Mapping quality (RMS <= 5 cm) is measured at the real point rate, 20k rays / frame, on short runs; one 20k headline mission runs in the
  background (`remote.sh bg`). The map itself always stays at 5 cm.

## Swath

`python -m autonomy.swath` (in `examples/dog`): a 10 s straight pass at 1.5 m/s over 20 m of rubble, mapped with the TRUE pose (isolates the
sensor from localization); the 95 % half-swath is the contiguous distance from the track where >= 95 % of scoring cells are mapped.
Cells within 1.5 m of the start are blind (ground first seen 0.72 m ahead) and not scored.

| seed | 5k rays, pool 2: w95 right / left | 20k rays, pool 1: w95 right / left |
| ---- | --------------------------------- | ---------------------------------- |
| 1    | 2.5 / 2.0 m                       | 2.5 / 2.0 m                        |
| 2    | 2.0 / 2.0 m                       | 2.0 / 2.0 m                        |
| 3    | 2.5 / 1.0 m                       | 2.0 / 1.0 m                        |

5k at pool 2 and 20k at pool 1 agree (the density equivalence, pinned by `test_dog_autonomy_density.py`). A single pass covers only
+-1-2.5 m at 95 %, so the swath-derived lane is 1.8-3.6 m; see Coverage for why the mission uses 8 m lanes plus gap filling.

## Localization

EKF over (x, y, z, yaw): odometry prediction, gated RTK GNSS, ICP against keyframes. Finding: the raw ICP covariance is over-confident
(a few mm), so a drifting scan match pulled the filter off RTK; once the gate rejected RTK for good, the estimate ran away. Fixes:
(1) `icp.floor_xyz` 5 cm / `icp.floor_yaw` 0.01 rad are added to every match covariance; (2) after `loc.rtk_reset_n` = 20 consecutive RTK
rejections the filter resets its position to the fix with the initial covariance. Pinned by the persistent-rejection and
covariance-floor tests in `test_dog_autonomy_localize.py`. Localization error stays <= 0.134 m (the initial operator error) on the
rubble missions.

## Coverage

100 x 100 m rubble, 5k rays, pool 2, seed 1, 1200 s budget (`coverage_rubble`):

| lane | coverage at the return          |
| ---- | ------------------------------- |
| 6 m  | 78.9 %                          |
| 7 m  | 87.1 %                          |
| 8 m  | 95.5 % (adopted, `plan.lane_m`) |

At 8 m: home at t = 1130 s, map RMS 3.0 cm, localization error <= 0.133 m, 0 bumps. Narrower lanes cannot finish the field in 20 min at
1.5 m/s; wider lanes plus gap filling win. The plan's swath-derived lane (1.8-3.6 m) is therefore replaced by 8 m.

Tuning round (training seeds 1-8 only; held-out 1001-1016 untouched until the end). The held-out MC of the lane-8 / margin-60 setup
was 0/16 (median 93.7 %): the sweep never finished before the return trigger, and the return left 60-80 s unused. Results on seeds
1-8 (coverage >= 95 % count; latest home time):

| lane | return margin | seeds >= 95 %      | latest home   |
| ---- | ------------- | ------------------ | ------------- |
| 8 m  | 15 s          | 3/8                | 1181 s        |
| 9 m  | 15 s          | 8/8                | 1215 s (late) |
| 9 m  | 22 s          | 8/8 (95.2-96.9 %)  | 1179 s        |
| 9 m  | 30 s          | 7/8 (94.5 % worst) | 1169 s        |
| 10 m | 30 s          | 4/8                | 1181 s        |

Adopted for the 100 m rubble scenarios: **lane 9 m, return margin 22 s** (scenario overrides; the library defaults stay for the 20 m
smoke goldens).

Wall time: `c_py` on the 100 m map is about 0.142 wall-s per sim-s (0.07 on the 20 m smoke: the 200 x 200 block grid makes A* /
Dijkstra / `mapped()` heavier), so a 5k rubble mission is about 1200 x (1/3.06 + 0.142) = 9.5 min.

20k headline (`coverage_rubble_20k`, pool 1, seed 1): home at t = 1149 s, map RMS 3.07 cm, localization <= 0.134 m, 1 bump, wall 22 min
(model: 26 min). Only **89.0 %** mapped at pool 1 against 95.5 % at 5k / pool 2, so its `covers_95_percent` monitor fails: the pool-1
target set is stricter than the density equivalence suggests. The replay check inside `napl program` timed out (60 MB log, 600 s hard
limit); replayed afterwards with `napl-replay` and no limit (a bg job): tick 114910, state hash `223644ac3765e1fe` = the run's
`native.json` hash: **replay matches**. (These 20k numbers are from before the lane 9 / margin 22 tuning below.)

Re-run with the final rubble tuning (lane 9 m, margin 18 s, edge keep-out 0.5 m, seed 1): **90.4 %** mapped (was 89.0 %), home at
t = 1184 s, map RMS 3.0 cm, localization <= 0.022 m at the end, 0 bumps, 1508 m driven. The replay check inside `napl program` timed
out again (600 s limit); the earlier 20k run's replay was verified separately (above), this one was not replayed.

Ruins (`coverage_ruins`, seed 1). Three defects found, in the order they broke (trace of the failed run `ruins3`: map RMS 0.2-0.4 m from
t = 20 s, first bump t = 48 s, stuck for good by t = 270 s):

1. **Carrier trap.** A blocked step kept its new yaw: a foot ended inside the wall, every later step was blocked, bumps +10 per
   second until the end (7441 bumps; the robot stood almost still from t = 510 s to the end). Fix: a blocked step slides (the move alone, else
   the turn alone, else stays) and never leaves the body in contact (`test_a_bump_while_turning_never_leaves_the_feet_in_the_wall`).
2. **Clearance and tracking.** At 0.3 m blocks the path through a 0.8-1.6 m door grazes the jambs (feet 0.29 m from the body centre).
   Fixes: wall-adjacent block cost 6 (scenario `near_lethal`), speed 0.8 m/s and lookahead 0.45 m within 0.6 m of a wall, back-off when
   commanded but not moving for 4 s (`Robot._unstick`). Bumps 7441 -> 48.
3. **Scoring.** Map RMS counted floor cells next to walls, where a pose error smears the wall face onto the floor: within 2 cells of a
   wall the floor RMS was 1.09 m, at 4-6 cells 0.2 m, beyond 8 cells 3.4 cm. The scenario now scores floor cells >= 0.30 m (the
   localization tolerance) from a discontinuity (`map.edge_margin`; rubble keeps the 1-cell default, so its numbers are unchanged).
   Also, the way-home estimate (straight line x 1.4) was a third of the real path in a maze: the robot was still out at the budget
   (phase RETURN at 1260 s). The executive now plans the actual way home in the last 10 min.

After the fixes (`ruins6`): home at 1148 s, 48 bumps, 1009 m driven, map RMS 5.7 cm (3.8 cm until a 10-s GNSS-denied stretch at
t = 890 s smeared a few walls), localization error up to 0.37 m in two GNSS-denied corridor stretches, and **37.8 %** mapped.

**Blocker (physical, not a bug): 95 % in 20 min is not reachable in this maze.** Coverage grew by 0.037 % per metre driven; 1200 s at
v_max = 1.5 m/s is at most 1800 m, i.e. about 67 % at that efficiency even without a single slow-down or turn. Visiting each of the
400 rooms at least once needs a route of at least the spanning tree, 399 edges x 5 m = 1995 m (1330 s at v_max); seeing into rooms from
doorways shortens this, but the measured efficiency says by far too little. The ruins scenario keeps all four monitors and fails
`covers_95_percent` (and, by 0.7 cm and 7 cm of localization, `map_rms_within_5cm` and `localized_within_30cm`); a spec change
(larger budget, a smaller maze or a 60 % ruins target) is a user decision.

## Monte Carlo

`examples/dog/automc.py <scenario> --seeds a:b --jobs N --out DIR` runs one `automission.py` process per seed on HELD-OUT seeds 1001+
(dev seeds 1-30); success = coverage >= 95 %, RMS <= 5 cm, home within budget; Wilson interval on the success rate.

Rubble, `coverage_rubble` (5k rays, pool 2, lane 8 m), held-out seeds 1001-1016, **N = 16** (shrunk from the planned 25 to fit the
1.5 h wall budget: 6 runs in parallel, 3 waves, 57 min on a shared box):

- success **0/16**, Wilson 95 % interval [0.000, 0.194].
- coverage sorted: 86.4, 89.6, 89.6, 90.9, 91.9, 91.9, 92.1, 93.4, 93.9, 94.4, 94.4, 94.6, 94.7, 94.8, 94.8, 94.9 %; median 93.7 %.
- every other criterion holds on all 16: map RMS median 3.0 cm (<= 5 cm), home at t = 1118-1142 s, localization error <= 3.8 cm,
  0 bumps, 0 crashes.
- reading: the 95 % spec target is missed on every held-out seed, by 0.1-8.6 points. Seed 1 (95.5 %) was the dev seed on which
  `plan.lane_m` = 8 was chosen, so it was optimistic. The mission is robust (no failure but coverage); lacking coverage is a planning
  tuning problem (lane width / fill time / return margin), not a localization or mapping one. Open issue.

Re-run after the tuning round (lane 9 m, margin 22 s, the ruins-round carrier / executive fixes), same held-out seeds 1001-1016, N = 16, run
once:

- success **11/16**, Wilson 95 % interval [0.444, 0.858] (before: 0/16).
- coverage sorted: 92.3, 93.6, 93.7, 94.1, 95.3, 95.7, 95.8, 95.9, 95.9, 95.9, 96.0, 96.2, 96.2, 96.3, 96.7, 97.0 %; median 95.9 %.
- failures: 4 on coverage (92.3-94.1 %), and seed 1015 (95-96 % covered) was not home at the end (phase RETURN at t = 1222 s, counted
  late and not-home); the other 15 were home at 1163-1196 s. Map RMS <= 3.15 cm everywhere, localization <= 3.6 cm, 0 crashes.
- new since the first MC: 11 of 16 runs bump (1-508 bumps each, 1852 total; before 0). Root cause (found on training seeds 6-8, bump
  positions in the traces): the bumps sit at |x| or |y| = 49.7, the field edge. Lane 9 m made the planner detour round rocks along the
  edge; the carrier stops at 49.75 and the pursuit overshoots corners. Fix: `plan.border_m` = 0.5, a keep-out strip along the field edge
  in every plan (`test_plans_keep_out_of_the_field_edge_strip`).
- reading: the 95 % target is met on 11/16; the shortfall seeds are the ones with the most bumps (1007-1010: 123-508), so the contacts cost
  sweep time.

Third held-out run (edge keep-out 0.5 m, return margin 18 s; margin re-tuned on training seeds 1-8: 8/8 >= 95.2 %, bumps 0 except one
42-contact seed), seeds 1001-1016, N = 16, run once:

- success **14/16**, Wilson 95 % interval [0.640, 0.965]; median coverage 96.1 %, map RMS <= 3.2 cm, localization <= 4.7 cm.
- coverage sorted: 94.4, 95.2, 95.4, 95.8, 95.9, 95.9, 96.1, 96.1, 96.1, 96.1, 96.2, 96.2, 96.3, 96.4, 96.8, 97.0 %.
- bumps: 0 on 14 runs, 1 and 42 on two (was 1852 in total).
- failures: seed 1008 (94.4 %, home at 1185 s) and seed 1015 (97.0 % covered but still on its way home at the end, t = 1218 s, phase
  RETURN). Seed 1015 failed the same way in the previous held-out run; its cause was not diagnosed (held-out seeds are not used for
  diagnosis; no training seed in 1-8 returned later than 1182 s).

### Ruins, 50 x 50 m site (`coverage_ruins_50`)

Spec: the ruins criterion is a 50 x 50 m site (about 100 rooms), 20 min, RMS <= 5 cm, >= 95 % mapped. Same carrier / planner as above,
lane 5 m (one lane per row of rooms), return margin 40 s, 0.3 m blocks. Numbers from seed 1: home at 1106 s, 89.8 % mapped with margin
120 s; 91.8 % with margin 40 s (home 1168 s); map RMS 4.2 cm and localization <= 0.29 m (both monitors met on that seed; the two
near-misses of the 100 m site did not reappear, but 0.29 m is 1 cm from the limit and the cause, ICP drift along GNSS-denied corridors
and wall-face smear at 6-8 cells, was not removed).

Training seeds 1-4 (lane 5 / 10 / 25 m, margin 40 s): coverage 91.8 / 38.8 / 54.0 / 85.5 % for lane 5; lane 10 and 25 are worse
(seed 1: 86.6 and 88.9 %). Seeds 2-4 are limited by robot wedging, not by lanes: bumps 517, 42, 44 on seeds 2, 3, 4.

Held-out seeds 2001-2016, N = 16, run once (lane 5 m, margin 40 s): **0/16**, Wilson [0.000, 0.194]. Coverage sorted: 18.3, 56.2, 66.2,
66.4, 71.9, 73.4, 77.6, 81.3, 83.5, 85.0, 85.6, 85.7, 86.2, 86.3, 86.6, 88.5 % (median 83.5 %); 7 of 16 were not home at the end;
map RMS 2.4-13.3 cm (only 5 of 16 within 5 cm); localization error <= 0.37 m (one run); bumps 0-538.

Root causes found, not fixed (seed 2, traces and cost-map dumps): (1) the robot enters a 0.8-1.6 m doorway at a skew heading, the pure
pursuit turns on the spot to align, and a foot (0.29 m from the body centre) wedges against a jamb; (2) (inferred from the code and one cost-map dump, not
verified by a test) the planner's doorway is narrower than the real one and off-centre: a step between two blocks counts for the lower-index block only, so the lethal band of a
wall grows by a block on one side, and the path hugs one jamb; (3) after a back-off the same approach is retried. Tried on training
seeds 1-4, all worse and reverted: holonomic strafing near walls without turning, remembering the wedge point as impassable (it sealed
the robot into a doorway), 0.2 m blocks with two near-wall cost rings, symmetric step pairs with strafing and wedge memory (bumps fell to 8-97 on
0.3 m blocks, but coverage stayed at 13-76 %: the robot stopped short of doors it now considered closed).

#### Footprint-aware doorway planner (`autonomy/clearance.py`, `trav.clearance = 1`, ruins only)

Block lethality is replaced by the geometry itself. Each 1 Hz plan tick: obstacle cells of the 5 cm elevation map (known cells more than
`step_max` above the ground of their 2 m block, the ground being its 20th percentile, isolated cells dropped) -> chamfer distance transform
(numpy, error < 9 %) -> block cost from the clearance at the block (largest of its central 2 x 2 cells): 1 above `clear_margin` 0.45 m, rising
to `near_lethal` at `clear_lethal` 0.20 m, closed below. The path from A* is then refined: every waypoint moves sideways (up to 0.3 m) to the
best clearance (a doorway centres, open ground does not pull) and a waypoint 0.5 m before every narrow stretch lies on its line, so the robot
enters a door straight. Next to a wall (< 0.6 m) a target behind the robot (> 90 deg) is reached by reversing at 0.3 m/s, never by turning on
the spot (the foot wedge, root cause 1). When no way home exists at the strict clearance the return plans again at 0.08 m instead of standing
still. Tests (`test_dog_autonomy_clearance.py`): EDT accuracy, a centred 0.8 m door gives a centred straight path with a run-up on its axis
(root cause 2, now verified), a 0.3 m gap is closed and opens at the relaxed lethal, a low outlier cell and single spikes are not obstacles,
the pursuit reverses instead of turning, open ground is not pulled.

What the training seeds (1-8, never the held-out ones) showed, in the order found: with lethal 0.30 m and then 0.25 m the 0.8 m doors read
closed (a mapped 0.8 m door is only about 0.6 m wide, the wall faces smear 5-10 cm onto the floor cells) and the robot stood still in
RETURN (0 of 8 home, coverage 6 %); one -0.2 m outlier cell lowered the ground of a 2 m block and read the floor as a wall; with 0.20 m and the
robust ground 7 of 8 were home, coverage 91 / 68 / 87 / 77 / 92 / 84 / 57 / 79 %, but bumps 140-997 (turning on the spot at a wall); with
the reversing 5 of 8 had 2-238 bumps; with the relaxed return 7 of 8 home. Near-wall speed 1.2 m/s instead of 0.8 gained nothing (coverage
mean 80 % both, bumps up) and was reverted.

| ruins 50 m                   | block lethality (before)    | clearance planner               |
| ---------------------------- | --------------------------- | ------------------------------- |
| training seeds 1-4 coverage  | 91.8 / 38.8 / 54.0 / 85.5 % | 87 / 67 / 88 / 76 %             |
| training seeds 1-8 home      | not measured                | 7 of 8                          |
| held-out 2001-2016 success   | 0/16, Wilson [0.000, 0.194] | **0/16, Wilson [0.000, 0.194]** |
| held-out median coverage     | 83.5 % (range 18.3-88.5)    | 84.7 % (range 10.9-90.8)        |
| held-out home at the end     | 9 of 16                     | 15 of 16                        |
| held-out map RMS within 5 cm | 5 of 16                     | 8 of 16                         |

Rubble is unaffected (`clearance` defaults to 0): the `coverage_smoke` and `map_quality` goldens still match (`test_mission_passes_replays_and_matches_its_golden_hash`),
and `coverage_rubble` on training seeds 1-8 with the final code gives 8/8 >= 95.2 % (95.2-97.0 %, median 96.1 %), map RMS 3.0 cm, localization
<= 2 cm at the end, home by t = 1182 s (no late return like held-out seed 1015 appeared), bumps 0 except 42 and 1. Seed 1015 itself was not
reproduced on a training seed and stays undiagnosed.

Held-out outcome (run once): the 95 % target is still missed on every seed (coverage sorted: 10.9, 74.2, 74.7, 75.5, 78.3, 81.6, 82.1, 84.1,
85.2, 85.4, 85.7, 86.1, 87.0, 87.1, 88.3, 90.8 %; bumps 0-312). The wedging and the stuck returns are gone (bumps mostly < 50, 15 of 16
home), but the mission is now limited by distance, not by doors: about 1000 m driven in 20 min at 0.85 m/s, and 0.037 % mapped per metre
would put the ceiling near 90 % at that rate (inferred from the earlier per-metre figure, not measured again); the lane sweep also shuttles between far lane ends through the maze (training seed 6: the same two places visited
every 180 s). Open: (a) coverage planning for the maze (visit rooms in a tour, FILL by room instead of lane ends), the real fix for the last
5-10 points; (b) map RMS, 8 of 16 over 5 cm, from the wall-face smear next to doors and from a robot that presses against walls (a mapped
wall 0.3 m too thick at the contact point); (c) held-out seed 2001 returned home at t = 165 s with 10.9 % mapped (probably the first sweep goals all
read unreachable and the fill found nothing, as seen on training seed 3 before the lethal fix; not verified) - not diagnosed, held-out seeds are not used for diagnosis; (d) training seed 2
still wedges (1388 bumps) in one room.

Ruins 100 x 100 m: unchanged (not reachable, see Coverage).

## Upgrade path: GPU raycaster

The lidar dominates the cost. A batched GPU ray caster (MuJoCo Warp's ray kernels, or a CUDA/HIP BVH over the hfield triangles) would
make the 20k-ray missions cheap and remove the pooled-scoring workaround. Out of scope for SP5.
