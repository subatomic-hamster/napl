# Dog autonomy (SP5)

The robot's autonomy software is the package `examples/dog/autonomy/` (numbers in `autonomy.toml`, numbers rule). Tasks 1-10 run it on a
kinematic carrier (a pose-commanded body with the same sensors); Task 11 (SP3b + SP4) puts it on the dog. Recon and mapping only.

## Sim budget

`tools/bench_autonomy.py --sim 5.0` (hfbox, one job, the kinematic carrier: rosette lidar at 10 Hz + RTK GNSS, no legs, driving 1.5 m/s
with a slow turn over a 100 x 100 m field, `max_range` 40 m, terrain seed 3). Sim-seconds per wall-second, kernel only:

| kind | patch | 2.5k | 5k | 10k | 20k |
|---|---|---|---|---|---|
| rubble | 10 m | 4.33 | 2.20 | 1.13 | 0.57 |
| rubble | 5 m | 5.91 | 3.06 | 1.59 | 0.81 |
| slopes | 10 m | 0.29 | 0.14 | 0.07 | 0.04 |
| slopes | 5 m | 0.63 | 0.31 | 0.16 | 0.08 |
| scree | 10 m | 9.64 | 4.95 | 2.54 | 1.27 |
| scree | 5 m | 12.30 | 6.43 | 3.33 | 1.68 |
| ruins | 10 m | 2.71 | 2.25 | 1.73 | 1.17 |
| ruins | 5 m | 2.64 | 2.18 | 1.60 | 1.06 |

Compared with `docs/models.md` "Ground sim budget" (10 m patches, dog physics on top): the carrier is faster on rubble (2.2 vs 1.27 at
5k), scree (4.95 vs 2.81) and slopes (0.14 vs 0.13), but slower on ruins (2.25 vs 4.56 at 5k; cause not investigated, the carrier starts in a walled cell). Unlike the doc's note that 10 m patches are 2.6-6x faster than one hfield, **5 m patches are 1.4x (rubble), 2.2x (slopes) and 1.3x
(scree) faster than 10 m patches on the carrier**, and equal on ruins (the walls are boxes, the ground is flat). Noise level of
the table: single 5 sim-s runs, other jobs on the box (load ~2): read to +-10 %.

## Wall-time model

`wall = T_mission (1 / s_kernel + c_py)`, `c_py` = the autonomy's Python per sim-second.

Measured `c_py` (Task 9, `coverage_smoke`, 20 x 20 m rubble, 5k rays, 1401 frames, wall ms per 10 Hz frame):

| module | ms / frame |
|---|---|
| points (levelled, angular noise) | 0.47 |
| ICP match + keyframe add | 0.96 |
| elevation-map fusion | 0.19 |
| traversability (windowed) | 4.50 |
| planning + pure pursuit + fix | 0.69 |
| scoring (harness, 1 per 10 s) | 0.03 |

Total 6.8 ms/frame -> **`c_py` = 0.07 wall-s per sim-s** (the plan budgeted 0.25). This is measured on a 20 m map; on the
100 x 100 m map the 200 x 200 block grid makes A* / Dijkstra and `mapped()` heavier (re-measured by the 100 m missions, see Coverage).
Mission of 1200 sim-s with the measured `s_kernel` of the better patch size and `c_py = 0.07`:

| kind | rays | patch | s_kernel | 1200 s mission wall |
|---|---|---|---|---|
| rubble | 5k | 5 m | 3.06 | 7.9 min |
| rubble | 20k | 5 m | 0.81 | 26 min |
| ruins | 5k | 10 m | 2.25 | 10.3 min |
| ruins | 20k | 10 m | 1.17 | 18.6 min |
| scree | 5k | 5 m | 6.43 | 4.5 min |
| slopes | 5k | 5 m | 0.31 | 66 min (not run as a full mission) |

## Decision: rays and patches

- Patch size is the faster of 10 m / 5 m per kind: **rubble, scree, slopes 5 m; ruins 10 m** (the ruins difference is inside the noise,
  10 m kept). The patch is a `field()` argument; heights and boxes are identical, only the material grid (friction) differs, which the
  kinematic carrier does not use.
- Missions and Monte Carlo run at `lidar.rays_mission` = **5000** rays / frame and are scored on the pooled grid (`plan.pool` = 2,
  10 cm scoring cells; `20000 / 0.05^2 = 5000 / 0.10^2`). The rule "if rubble 5k measures below 1.0 sim-s per wall-s, use 2.5k and
  pool 3" is **not triggered** (rubble 5k = 2.20 / 3.06). Task 10's density test pins the equivalence.
- Mapping quality (RMS <= 5 cm) is measured at the real point rate, 20k rays / frame, on short runs; one 20k headline mission runs in the
  background (`remote.sh bg`). The map itself always stays at 5 cm.

## Upgrade path: GPU raycaster

The lidar dominates the cost. A batched GPU ray caster (MuJoCo Warp's ray kernels, or a CUDA/HIP BVH over the hfield triangles) would
make the 20k-ray missions cheap and remove the pooled-scoring workaround. Out of scope for SP5.
