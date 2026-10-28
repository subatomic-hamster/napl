# Threat-suite benchmark

Scenes, generators and sources: `threat-scenarios.md`. Everything here is generic kinematic test data at backyard scale; the numbers
say how the `napl.track` variants behave on it, not how any real system performs.

## Reproduce

```
# tracker benchmark: 50 seeds (1..50) x 17 scenes (14 suite + random, random_cam, close_cam_lidar) x 7 variants (5950 runs, ~28 min on 6 processes)
NAPL_REMOTE_DIR=/workspace/wt/napl-c2threat tools/remote.sh bg threat \
  'OMP_NUM_THREADS=1 python tools/threat_bench.py track --runs 50 --jobs 6 > out/threat-track.md'
# fusion check (camera vs camera + boresight range), 50 seeds
OMP_NUM_THREADS=1 python tools/threat_bench.py fusion --runs 50 --jobs 6
# discrimination: train seeds 1..400 of the `random` family, held-out seeds 10001..10150, two position-noise levels, feature ablations
OMP_NUM_THREADS=1 python tools/threat_bench.py discrim --train 400 --test 150
# tests
PYTHONPATH=python python -m pytest tests/python/test_threats.py -q
```

Metrics are those of `napl.track.metrics.Evaluator` (GOSPA alpha 2, p 2, cut-off c per scene; loc / missed / false are its parts;
breaks = a truth object that was matched and then lost; ID sw = matched track id changes). Only objects that exist in a frame are
scored (births/deaths), occluded or missed objects still count as truth. All objects of a scene are scored, birds and debris included:
the tracker's job is to follow them, discrimination (below) decides what to engage. +-95 % is the half-width of the GOSPA mean over the
50 seeds; cells are paired by seed. ms/update is wall time per `tracker.step`, measured while other jobs shared the box (compare within a
table, not across hosts). `ab-nn` (the repo's alpha-beta nearest-neighbour baseline) works on position measurements only, so camera scenes show n/a.
Variants use the tuned options of `variants.KW` (`imm2-gnn-s`, `imm2-jpda`, `imm2-mht`, `phd-singer`).

## Tracker results (seeds 1..50, 5950 runs in 1657 s on 6 processes)

Reading the tables: the cut-off c is 2 m, so a GOSPA near 0.4 on a one-object scene is the 0.5 m sensor noise plus start-up misses; "false"
is dominated by duplicate or clutter tracks (the loose-hit-rule `imm2-gnn`/`imm3-gnn` keep extra tracks that `imm2-gnn-s` and JPDA/MHT drop).

### route_hover (pos, 1 drone)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.397 | 0.080 | 0.633 | 0.193  | 1.404 | 4.52   | 1.64  | 0.32      |
| imm2-gnn   | 0.609 | 0.039 | 0.412 | 0.110  | 0.632 | 0.00   | 0.28  | 1.51      |
| imm2-gnn-s | 0.436 | 0.012 | 0.413 | 0.110  | 0.287 | 0.00   | 0.12  | 1.46      |
| imm3-gnn   | 0.605 | 0.038 | 0.407 | 0.110  | 0.631 | 0.00   | 0.24  | 2.60      |
| imm2-jpda  | 0.399 | 0.009 | 0.408 | 0.110  | 0.133 | 0.00   | 0.00  | 1.39      |
| imm2-mht   | 0.391 | 0.008 | 0.408 | 0.112  | 0.072 | 0.00   | 0.00  | 2.46      |
| phd-singer | 0.416 | 0.006 | 0.438 | 0.142  | 0.011 | 0.08   | 0.08  | 1.49      |

### route_fast (pos, 1 drone, tuned limits, up to 15 m/s)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.295 | 0.080 | 0.624 | 0.190  | 1.295 | 3.94   | 1.08  | 0.31      |
| imm2-gnn   | 0.586 | 0.027 | 0.435 | 0.110  | 0.558 | 0.00   | 0.12  | 1.50      |
| imm2-gnn-s | 0.454 | 0.011 | 0.439 | 0.110  | 0.270 | 0.00   | 0.14  | 1.46      |
| imm3-gnn   | 0.586 | 0.027 | 0.436 | 0.110  | 0.557 | 0.00   | 0.16  | 2.58      |
| imm2-jpda  | 0.421 | 0.010 | 0.434 | 0.112  | 0.118 | 0.02   | 0.02  | 1.39      |
| imm2-mht   | 0.414 | 0.008 | 0.433 | 0.111  | 0.078 | 0.00   | 0.00  | 2.46      |
| phd-singer | 0.422 | 0.006 | 0.447 | 0.137  | 0.012 | 0.06   | 0.08  | 1.47      |

### orbit (pos, 1 drone)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.846 | 0.078 | 0.649 | 0.232  | 1.862 | 6.88   | 3.76  | 0.32      |
| imm2-gnn   | 0.848 | 0.042 | 0.430 | 0.108  | 0.940 | 0.02   | 0.44  | 1.57      |
| imm2-gnn-s | 0.493 | 0.016 | 0.436 | 0.109  | 0.379 | 0.02   | 0.22  | 1.47      |
| imm3-gnn   | 0.840 | 0.044 | 0.419 | 0.108  | 0.940 | 0.02   | 0.44  | 2.72      |
| imm2-jpda  | 0.425 | 0.011 | 0.426 | 0.108  | 0.183 | 0.00   | 0.00  | 1.38      |
| imm2-mht   | 0.409 | 0.010 | 0.425 | 0.115  | 0.074 | 0.00   | 0.00  | 2.48      |
| phd-singer | 0.412 | 0.005 | 0.436 | 0.136  | 0.016 | 0.10   | 0.10  | 1.60      |

### survey (pos, 1 drone, lawnmower)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.941 | 0.035 | 0.654 | 0.231  | 1.948 | 6.62   | 4.66  | 0.32      |
| imm2-gnn   | 0.867 | 0.024 | 0.418 | 0.110  | 0.980 | 0.02   | 0.22  | 1.58      |
| imm2-gnn-s | 0.486 | 0.012 | 0.425 | 0.112  | 0.396 | 0.02   | 0.30  | 1.47      |
| imm3-gnn   | 0.869 | 0.024 | 0.420 | 0.110  | 0.982 | 0.02   | 0.46  | 2.73      |
| imm2-jpda  | 0.411 | 0.009 | 0.415 | 0.119  | 0.166 | 0.00   | 0.00  | 1.38      |
| imm2-mht   | 0.394 | 0.007 | 0.416 | 0.118  | 0.041 | 0.00   | 0.00  | 2.49      |
| phd-singer | 0.415 | 0.006 | 0.437 | 0.143  | 0.020 | 0.16   | 0.18  | 1.62      |

### weave (pos, 1 drone, random weaving)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 0.863 | 0.063 | 0.616 | 0.156  | 0.743 | 2.06   | 0.18  | 0.30      |
| imm2-gnn   | 0.500 | 0.020 | 0.479 | 0.108  | 0.262 | 0.00   | 0.22  | 1.45      |
| imm2-gnn-s | 0.464 | 0.010 | 0.484 | 0.108  | 0.122 | 0.00   | 0.20  | 1.43      |
| imm3-gnn   | 0.492 | 0.021 | 0.473 | 0.108  | 0.259 | 0.00   | 0.22  | 2.49      |
| imm2-jpda  | 0.451 | 0.009 | 0.478 | 0.108  | 0.065 | 0.00   | 0.00  | 1.38      |
| imm2-mht   | 0.450 | 0.009 | 0.477 | 0.109  | 0.051 | 0.00   | 0.00  | 2.41      |
| phd-singer | 0.449 | 0.007 | 0.475 | 0.137  | 0.012 | 0.10   | 0.26  | 1.36      |

### swarm_v (pos, 6 drones, V to line, clutter 10/scan)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw  | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ------ | --------- |
| ab-nn      | 2.971 | 0.095 | 1.750 | 0.937  | 2.309 | 98.10  | 149.44 | 1.20      |
| imm2-gnn   | 1.401 | 0.052 | 1.044 | 0.267  | 1.002 | 0.04   | 5.40   | 4.31      |
| imm2-gnn-s | 1.131 | 0.013 | 1.052 | 0.272  | 0.460 | 0.16   | 4.54   | 4.15      |
| imm3-gnn   | 1.399 | 0.051 | 1.043 | 0.267  | 1.002 | 0.04   | 5.50   | 7.16      |
| imm2-jpda  | 1.064 | 0.010 | 1.034 | 0.291  | 0.246 | 0.34   | 4.68   | 3.76      |
| imm2-mht   | 1.085 | 0.016 | 1.066 | 0.297  | 0.185 | 0.94   | 10.30  | 7.39      |
| phd-singer | 1.206 | 0.036 | 1.122 | 0.501  | 0.218 | 13.36  | 24.20  | 4.72      |

### swarm_split (pos, 8 drones grid, 3 split and rejoin, clutter 10/scan)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw  | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ------ | --------- |
| ab-nn      | 3.139 | 0.061 | 1.989 | 1.069  | 2.283 | 128.68 | 168.26 | 2.35      |
| imm2-gnn   | 1.515 | 0.032 | 1.191 | 0.340  | 0.993 | 0.84   | 2.70   | 8.79      |
| imm2-gnn-s | 1.281 | 0.017 | 1.200 | 0.351  | 0.460 | 0.74   | 1.66   | 8.41      |
| imm3-gnn   | 1.511 | 0.031 | 1.186 | 0.341  | 0.994 | 0.88   | 2.68   | 14.58     |
| imm2-jpda  | 1.249 | 0.024 | 1.190 | 0.430  | 0.257 | 3.90   | 4.64   | 8.42      |
| imm2-mht   | 1.250 | 0.025 | 1.218 | 0.370  | 0.217 | 1.74   | 7.68   | 15.85     |
| phd-singer | 1.425 | 0.066 | 1.319 | 0.549  | 0.300 | 19.06  | 39.78  | 10.13     |

### balloons (pos, 3 balloons, gusting wind)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.294 | 0.036 | 1.063 | 0.300  | 0.808 | 8.30   | 2.36  | 0.73      |
| imm2-gnn   | 0.685 | 0.015 | 0.641 | 0.193  | 0.286 | 0.00   | 0.08  | 3.49      |
| imm2-gnn-s | 0.648 | 0.008 | 0.642 | 0.193  | 0.116 | 0.00   | 0.14  | 3.32      |
| imm3-gnn   | 0.644 | 0.015 | 0.600 | 0.193  | 0.285 | 0.00   | 0.08  | 6.04      |
| imm2-jpda  | 0.639 | 0.006 | 0.641 | 0.195  | 0.060 | 0.00   | 0.00  | 3.45      |
| imm2-mht   | 0.638 | 0.006 | 0.643 | 0.190  | 0.039 | 0.00   | 0.24  | 6.23      |
| phd-singer | 0.746 | 0.007 | 0.743 | 0.252  | 0.016 | 0.84   | 0.70  | 3.04      |

### hanging (pos, 2 swinging loads)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 2.329 | 0.074 | 0.959 | 0.379  | 2.192 | 17.40  | 21.02 | 0.62      |
| imm2-gnn   | 1.195 | 0.057 | 0.546 | 0.156  | 1.242 | 0.10   | 1.50  | 2.94      |
| imm2-gnn-s | 0.663 | 0.020 | 0.551 | 0.161  | 0.504 | 0.12   | 1.30  | 2.61      |
| imm3-gnn   | 1.175 | 0.059 | 0.514 | 0.156  | 1.241 | 0.10   | 1.22  | 4.70      |
| imm2-jpda  | 0.554 | 0.009 | 0.534 | 0.165  | 0.220 | 0.06   | 2.12  | 2.42      |
| imm2-mht   | 0.546 | 0.026 | 0.535 | 0.201  | 0.068 | 0.78   | 0.68  | 4.67      |
| phd-singer | 0.632 | 0.033 | 0.615 | 0.232  | 0.042 | 2.00   | 0.54  | 2.83      |

### birds (pos, 3 birds + 1 drone)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.398 | 0.014 | 1.241 | 0.332  | 0.696 | 8.82   | 1.70  | 0.68      |
| imm2-gnn   | 1.058 | 0.016 | 1.025 | 0.249  | 0.366 | 1.14   | 2.98  | 3.21      |
| imm2-gnn-s | 1.059 | 0.018 | 1.039 | 0.267  | 0.308 | 1.80   | 3.44  | 2.65      |
| imm3-gnn   | 1.054 | 0.016 | 1.024 | 0.251  | 0.367 | 1.22   | 3.02  | 4.73      |
| imm2-jpda  | 1.021 | 0.015 | 1.024 | 0.259  | 0.157 | 1.36   | 1.18  | 2.67      |
| imm2-mht   | 1.022 | 0.016 | 1.026 | 0.243  | 0.163 | 0.90   | 1.24  | 5.27      |
| phd-singer | 1.079 | 0.013 | 1.071 | 0.301  | 0.145 | 1.42   | 3.38  | 2.24      |

### clutter_mix (pos, drone + balloon + bird + leaf + 2 bags; births/deaths, occlusion gaps, Pd 0.85)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.810 | 0.041 | 1.309 | 0.993  | 0.931 | 16.92  | 7.10  | 0.81      |
| imm2-gnn   | 1.290 | 0.036 | 0.919 | 0.908  | 0.497 | 4.48   | 5.02  | 4.01      |
| imm2-gnn-s | 1.274 | 0.034 | 0.926 | 0.913  | 0.424 | 4.88   | 5.26  | 3.75      |
| imm3-gnn   | 1.271 | 0.036 | 0.895 | 0.908  | 0.498 | 4.54   | 5.04  | 5.28      |
| imm2-jpda  | 1.257 | 0.034 | 0.916 | 0.913  | 0.383 | 4.58   | 4.30  | 2.89      |
| imm2-mht   | 1.264 | 0.033 | 0.924 | 0.895  | 0.431 | 4.44   | 4.56  | 5.25      |
| phd-singer | 1.340 | 0.033 | 0.994 | 0.977  | 0.300 | 5.48   | 6.10  | 3.80      |

### busy (pos, 10 objects, births/deaths, gaps, Pd 0.85, clutter 10/scan)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 2.444 | 0.046 | 1.765 | 1.418  | 1.078 | 65.84  | 43.50 | 1.29      |
| imm2-gnn   | 1.704 | 0.034 | 1.295 | 1.055  | 0.589 | 7.00   | 8.60  | 4.65      |
| imm2-gnn-s | 1.694 | 0.034 | 1.307 | 1.061  | 0.520 | 7.62   | 8.90  | 7.47      |
| imm3-gnn   | 1.690 | 0.035 | 1.280 | 1.052  | 0.590 | 7.02   | 8.64  | 13.60     |
| imm2-jpda  | 1.669 | 0.035 | 1.291 | 1.065  | 0.461 | 7.64   | 7.98  | 7.49      |
| imm2-mht   | 1.692 | 0.034 | 1.316 | 1.038  | 0.531 | 7.56   | 11.66 | 14.05     |
| phd-singer | 1.772 | 0.036 | 1.362 | 1.166  | 0.416 | 14.02  | 19.68 | 10.21     |

### cam_mix (camera, drone + bird + balloon, births/deaths, gaps; c = 4 m)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | n/a   |       |       |        |       |        |       |           |
| imm2-gnn   | 2.488 | 0.112 | 1.958 | 1.731  | 0.962 | 8.00   | 5.12  | 2.07      |
| imm2-gnn-s | 2.487 | 0.112 | 1.957 | 1.731  | 0.959 | 8.02   | 5.12  | 2.11      |
| imm3-gnn   | 2.506 | 0.113 | 1.973 | 1.743  | 0.980 | 8.00   | 5.04  | 3.40      |
| imm2-jpda  | 2.488 | 0.112 | 1.959 | 1.741  | 0.935 | 8.18   | 4.50  | 2.06      |
| imm2-mht   | 2.491 | 0.110 | 1.958 | 1.722  | 0.979 | 8.04   | 4.52  | 3.51      |
| phd-singer | 2.654 | 0.106 | 2.123 | 1.845  | 0.848 | 11.24  | 5.12  | 1.55      |

### close_cam (camera, hanging load + slow drone at 3.5-8 m, c = 2 m)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | n/a   |       |       |        |       |        |       |           |
| imm2-gnn   | 0.417 | 0.013 | 0.456 | 0.172  | 0.016 | 0.02   | 0.26  | 1.95      |
| imm2-gnn-s | 0.416 | 0.013 | 0.456 | 0.172  | 0.008 | 0.02   | 0.26  | 1.94      |
| imm3-gnn   | 0.411 | 0.014 | 0.449 | 0.172  | 0.019 | 0.02   | 0.28  | 3.24      |
| imm2-jpda  | 0.417 | 0.013 | 0.456 | 0.173  | 0.007 | 0.04   | 0.30  | 2.05      |
| imm2-mht   | 0.415 | 0.013 | 0.455 | 0.170  | 0.004 | 0.02   | 0.26  | 3.20      |
| phd-singer | 0.545 | 0.015 | 0.592 | 0.208  | 0.003 | 0.04   | 0.28  | 1.64      |

### random (pos, random family; objects, Pd 0.8-0.95 and clutter 2-10 vary per seed)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | 1.782 | 0.242 | 1.047 | 0.563  | 1.233 | 15.68  | 10.46 | 0.87      |
| imm2-gnn   | 1.103 | 0.157 | 0.728 | 0.431  | 0.643 | 1.54   | 2.08  | 4.02      |
| imm2-gnn-s | 0.894 | 0.083 | 0.736 | 0.443  | 0.355 | 1.88   | 2.34  | 3.86      |
| imm3-gnn   | 1.086 | 0.158 | 0.708 | 0.433  | 0.647 | 1.60   | 2.06  | 6.71      |
| imm2-jpda  | 0.833 | 0.084 | 0.723 | 0.444  | 0.218 | 1.60   | 1.58  | 3.08      |
| imm2-mht   | 0.834 | 0.088 | 0.726 | 0.450  | 0.193 | 1.42   | 2.96  | 5.72      |
| phd-singer | 0.893 | 0.095 | 0.786 | 0.498  | 0.144 | 2.92   | 4.16  | 4.11      |

### random_cam (camera, random family)

| variant    | GOSPA | +-95% | loc   | missed | false | breaks | ID sw | ms/update |
| ---------- | ----- | ----- | ----- | ------ | ----- | ------ | ----- | --------- |
| ab-nn      | n/a   |       |       |        |       |        |       |           |
| imm2-gnn   | 2.754 | 0.333 | 2.002 | 1.632  | 1.298 | 7.96   | 11.32 | 5.15      |
| imm2-gnn-s | 2.617 | 0.344 | 2.001 | 1.627  | 0.997 | 7.56   | 11.54 | 4.23      |
| imm3-gnn   | 2.734 | 0.334 | 1.980 | 1.631  | 1.302 | 7.80   | 11.70 | 7.06      |
| imm2-jpda  | 2.560 | 0.339 | 1.998 | 1.642  | 0.835 | 8.02   | 8.88  | 3.67      |
| imm2-mht   | 2.573 | 0.345 | 2.008 | 1.620  | 0.839 | 7.58   | 9.96  | 6.55      |
| phd-singer | 2.797 | 0.339 | 2.251 | 1.783  | 0.700 | 12.80  | 9.90  | 5.86      |

(`close_cam_lidar` is reported in the fusion section. Its track-benchmark rows are the same cell as the fusion table below.)

### What the tracker tables show

- One-object scenes (route, orbit, survey, weave): all IMM variants hold the track (0 breaks); they differ in duplicate/clutter tracks
  (false). `imm2-gnn` and `imm3-gnn` carry 0.6-0.98 false GOSPA, the strict-hit variants (`imm2-gnn-s`, JPDA, MHT, PHD) 0.01-0.4.
  `ab-nn` is worst everywhere (GOSPA 0.9-1.9, 2-7 breaks): the fixed gate cannot follow turns and hovers. The IMM-3 turn mode (`imm3-gnn`)
  buys nothing on orbit or weave here (0.840 vs 0.848 on orbit, within the 95 % bands).
- Swarms (spacing 2-5 m, jitter, formation change) are the hard cases: GOSPA 1.06-1.4 against c = 2 m, 4-5 ID switches per run for the
  GNN/JPDA variants, 8-10 for MHT (18-24 before TUNE-2, see `tracking-tuning.md`), 24-40 for PHD, 150-170 for `ab-nn`. Split/merge adds
  breaks (JPDA 3.9, MHT 1.7, PHD 19). The `imm2-mht` rows were re-run after TUNE-2 (50 seeds, same seeds); only their quality columns were
  replaced, the ms/update column is from the original run (a 4-process re-run measured +0.1 ms for the joint assignment, 7.3 -> 7.5 on `swarm_v`).
- Slow movers: balloons are easy (0.64); swinging loads cost the loose-hit GNN variants 1.2 GOSPA from duplicate tracks, JPDA/MHT 0.54.
- Birds (erratic turns, flap/glide speed changes) cost every IMM/PHD variant 1.0-1.1 GOSPA and 1-1.8 breaks (`ab-nn`: 1.4, 8.8): nothing in the library has a motion model
  for a bird, and the tracker has no way to decline them (that is the discrimination job).
- Mixed scenes with births/deaths, gaps and Pd 0.85 are dominated by missed (0.9-1.1): objects are unobservable for seconds and a track
  starts only after confirmation. The IMM variants are within about 0.03 of each other (clutter_mix 1.26-1.29, busy 1.67-1.71); PHD is worse (1.34 / 1.77) with the most breaks.
- Camera scenes: `cam_mix` GOSPA 2.5 is mostly missed objects that leave the field of view (the head follows the centroid, not every object).
  `close_cam` (one head target, 3.5-8 m) is easy (0.41). PHD is the weakest on camera measurements (0.545 vs 0.41).
- Cost: IMM-3 is 1.7x the update time of IMM-2 on the one-object scenes and 2.9x on `busy` (13.6 vs 4.65 ms); MHT 1.6x to 3x; the worst cell is 15.9 ms per update (`swarm_split`, MHT), inside the 50 ms frame (box was shared, see the note above).

## Fusion check: camera size-range vs camera + boresight range (TF-Luna-like)

Scene `close_cam` (hanging load + slow drone at 3.5-8 m, same truth in `close_cam_lidar`; 50 paired seeds, 58 s):
the camera's range channel is the boresight rangefinder (std 1.5 cm) inside the 2 degree beam within 8 m, size-based (15 %) elsewhere. The two
objects are in the beam 33 % of the time (seed 1), because the head follows their centroid.

| variant    | camera GOSPA | +-95% | camera+range GOSPA | +-95% | breaks cam | breaks cam+range | ID sw cam | ID sw cam+range |
| ---------- | ------------ | ----- | ------------------ | ----- | ---------- | ---------------- | --------- | --------------- |
| imm2-gnn   | 0.417        | 0.013 | 0.412              | 0.016 | 0.02       | 0.02             | 0.26      | 0.02            |
| imm2-gnn-s | 0.416        | 0.013 | 0.411              | 0.016 | 0.02       | 0.02             | 0.26      | 0.02            |
| imm2-jpda  | 0.417        | 0.013 | 0.412              | 0.016 | 0.04       | 0.04             | 0.30      | 0.06            |

Result: GOSPA does not change (0.005 difference against a 0.013-0.016 band). ID switches drop from 0.26-0.30 to 0.02-0.06 per run, i.e.
the range channel resolves the two-object ambiguity (swing crossing the drone's path) but does not improve position accuracy, which is
already limited by the pixel noise and the 0.5 m GOSPA floor of start-up misses. Two caveats found on the way: (1) a hard in/out-of-beam
switch of the range noise (first implementation) made the EKF inconsistent when the predicted and the true state straddled the beam edge
and was worse than camera-only (GOSPA 0.479 vs 0.430 on 8 seeds, 4.75 vs 1.5 ID switches); the smooth log-linear hand-over between 0.5
and 1.5 beam widths fixes it. (2) This is one configuration; the benefit would be larger with the head locked on a single target
(more time inside the beam) or a target nearer than 3.5 m. Not tested.

## Discrimination baseline

Setup: 21 features of a position history (speed mean/std/max, vertical speed std, horizontal 95th percentile, acceleration mean/std,
turn-rate std/mean, vertical high-pass rms and its spectrum in 0.5-2 / 2-5 / 5-10 Hz bands and peak frequency, horizontal high-pass rms,
straightness, height mean/std, log apparent size from a noisy size-based diameter estimate, log range, log angular size; `discrim.FEATURES`).
The history is the object's true track plus Gaussian noise (0.1 m or 0.5 m), oracle-associated, so tracker errors are excluded and this
measures the classifier and the generator, not the tracking chain. Classifier: multinomial logistic regression in numpy (standardised
features, L2 0.01, class-balanced weights, Adam, 1500 iterations; no sklearn dependency). Train: every object (alive >= 3 s) of
`random` seeds 1..400 (1284 objects). Test: seeds 10001..10150, held out (498 objects: drone 129, balloon 92, hanging 94, bird 95, debris 88).

Confusion matrices, rows true, columns predicted (full, with the 0.5 m noise, features "all"):

| 0.1 m noise | drone | balloon | hanging | bird | debris | recall |
| ----------- | ----- | ------- | ------- | ---- | ------ | ------ |
| drone       | 129   | 0       | 0       | 0    | 0      | 1.00   |
| balloon     | 0     | 91      | 1       | 0    | 0      | 0.99   |
| hanging     | 0     | 0       | 94      | 0    | 0      | 1.00   |
| bird        | 0     | 0       | 0       | 95   | 0      | 1.00   |
| debris      | 0     | 2       | 4       | 0    | 82     | 0.93   |

| 0.5 m noise | drone | balloon | hanging | bird | debris | recall |
| ----------- | ----- | ------- | ------- | ---- | ------ | ------ |
| drone       | 124   | 2       | 0       | 3    | 0      | 0.96   |
| balloon     | 0     | 88      | 2       | 0    | 2      | 0.96   |
| hanging     | 0     | 0       | 94      | 0    | 0      | 1.00   |
| bird        | 0     | 0       | 0       | 95   | 0      | 1.00   |
| debris      | 2     | 5       | 6       | 0    | 75     | 0.85   |

**Bird misclassified as drone (the costly error): 0 of 95 in every configuration below**, 0.1 m and 0.5 m noise, all four feature sets.
Drone misclassified as bird (loses an engagement): 0 / 129 at 0.1 m, 3 / 129 at 0.5 m.

Feature ablations (accuracy; per-class recall drone / balloon / hanging / bird / debris; bird -> drone count):

| noise | features                                                  | accuracy | recall                           | bird -> drone |
| ----- | --------------------------------------------------------- | -------- | -------------------------------- | ------------- |
| 0.1 m | all                                                       | 0.986    | 1.00 / 0.99 / 1.00 / 1.00 / 0.93 | 0 / 95        |
| 0.1 m | no size                                                   | 0.968    | 0.98 / 0.99 / 1.00 / 1.00 / 0.85 | 0 / 95        |
| 0.1 m | no oscillation spectrum                                   | 0.988    | 1.00 / 0.99 / 1.00 / 1.00 / 0.94 | 0 / 95        |
| 0.1 m | motion statistics only (no size, spectrum, height, range) | 0.978    | 0.97 / 0.99 / 1.00 / 1.00 / 0.93 | 0 / 95        |
| 0.5 m | all                                                       | 0.956    | 0.96 / 0.96 / 1.00 / 1.00 / 0.85 | 0 / 95        |
| 0.5 m | no size                                                   | 0.924    | 0.95 / 0.90 / 1.00 / 1.00 / 0.75 | 0 / 95        |
| 0.5 m | no oscillation spectrum                                   | 0.964    | 0.98 / 0.95 / 1.00 / 1.00 / 0.88 | 0 / 95        |
| 0.5 m | motion statistics only                                    | 0.876    | 0.95 / 0.74 / 1.00 / 1.00 / 0.65 | 0 / 95        |

How much to trust this: the numbers are an optimistic upper bound, not a field estimate. The five classes were generated by five very
different motion models, so they separate by construction; birds are always caught (erratic turn-rate shocks and speed changes) even
without the wingbeat spectrum, which carries almost nothing at 20 Hz sampling (the spectrum feature does not help: 0.986 with it, 0.988
without). There is no hard negative in the data: a drone that weaves like a bird, a bird gliding straight, or a balloon-speed drone hover
is not generated at the rates real scenes would have (a `weave` drone and the `hanging`/`route_hover` cases are the nearest). Debris is
the weakest class (0.65-0.93) because it overlaps balloons and hanging loads in speed and size. The costly-error result would need
adversarial scenes (bird-like drones, straight-gliding birds) and real data before any weight is put on it. Not done: gradient-boosted
model, classification from tracker output (rather than oracle-associated noisy tracks), per-class costs in the loss.
