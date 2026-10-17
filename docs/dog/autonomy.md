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

Wall time: `c_py` on the 100 m map is about 0.142 wall-s per sim-s (0.07 on the 20 m smoke: the 200 x 200 block grid makes A* /
Dijkstra / `mapped()` heavier), so a 5k rubble mission is about 1200 x (1/3.06 + 0.142) = 9.5 min.

20k headline (`coverage_rubble_20k`, pool 1, seed 1): home at t = 1149 s, map RMS 3.07 cm, localization <= 0.134 m, 1 bump, wall 22 min
(model: 26 min). Only **89.0 %** mapped at pool 1 against 95.5 % at 5k / pool 2, so its `covers_95_percent` monitor fails: the pool-1
target set is stricter than the density equivalence suggests. The replay check of that run timed out (the 60 MB log, 600 s limit in
`napl program`), so its replay is unverified.

Ruins (`coverage_ruins`, seed 1): with the default 0.5 m planning blocks the 0.8-1.6 m doors read closed (1177 of 40000 blocks
reachable); `block = 0.3` fixes reachability (108515 of 110889), but the full mission still fails: at the return (t = 1260 s) it had
mapped **26 %**, map RMS 0.40 m, 7441 bumps. Open issue; the spec field is the rubble one.

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

Ruins: no Monte Carlo (the seed-1 mission already fails at 26 %, see Coverage).

## Upgrade path: GPU raycaster

The lidar dominates the cost. A batched GPU ray caster (MuJoCo Warp's ray kernels, or a CUDA/HIP BVH over the hfield triangles) would
make the 20k-ray missions cheap and remove the pooled-scoring workaround. Out of scope for SP5.
