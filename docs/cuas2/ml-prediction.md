# Learned motion prediction for fire-control lead (P13 stage ML-1)

Question: from the tracker output history, does a small learned model predict the target position 0.2-1.0 s ahead (the ball's time
of flight) better, and with a better-calibrated covariance, than the IMM's own extrapolation? And is a post-hoc scalar fix of the IMM
covariance enough to cure the pessimistic Ph (mean Ph 0.46 vs realised 0.67 on p13-integ)?

## Conclusion

1. **The IMM covariance is pessimistic, as the Ph measurement suggested.** Pooled over the six scenario kinds, the IMM2 90 % ellipsoid
   contains 0.986 of the truth (ideal 0.90) and the 50 % ellipsoid 0.867 (ideal 0.50); mean Mahalanobis d2/3 = 0.43, i.e. the
   predicted variance is about 2.3x too large (sigma about 1.5x). That is the right sign and size to explain Ph too low.
2. **A scalar or per-horizon inflation (really deflation, s about 0.4-0.5) is NOT enough as a general fix.** Fitted on the val seeds,
   it calibrates the pooled set (90 % cover 0.89-0.91) and the benign kinds (cv, cross, swarm5, cam), but it makes the manoeuvring
   kinds **over-confident**: turn 90 % cover 0.77, ca 0.70-0.68 (ca at 1.0 s: 0.59), and their NLL gets worse with a paired CI
   excluding 0 (turn +0.25 [0.22, 0.28], ca +0.22 [0.19, 0.24] vs IMM2). The IMM's pessimism is not a constant: the needed variance
   factor (d2/3 of raw IMM2) runs from 0.29 (cam) and 0.34 (cv) to 0.71-0.72 (turn, ca). Only a per-kind factor (an oracle: the tracker does
   not know the kind) calibrates everywhere (90 % cover 0.88-0.94 per kind). Over-confidence is the dangerous direction for Ph (wasted shots), so a
   single global factor tuned on benign data would be a regression on dodging targets. A usable calibration has to be conditioned on
   manoeuvre state, e.g. on the IMM mode probability (not tried, see "Not done").
3. **Learning beats IMM in-distribution, by a lot, on accuracy and on calibration:** pooled RMSE at 1.0 s 1.83 m (IMM2) -> 0.92 m
   (MLP) / 1.02 m (GRU); NLL 2.78 -> -0.25 (MLP) (mean over horizons, includes the 2.76 constant); 90 % cover 0.89-0.91 per horizon and
   kind (turn 0.87-0.89, ca 0.88, swarm5 0.89 are slightly under). All paired CIs by run exclude 0 for every kind. MLP is better than GRU
   (GRU also overfits: val NLL rises after epoch ~10).
4. **But this does not transfer, and the gain is mostly the scenario generator's prior.** The scenarios are generated from fixed
   start states and fixed velocities (cv, turn, ca are fully deterministic paths; swarm5 has fixed starts and small random accelerations),
   and the model sees velocity, so it can learn "which scenario am I in" and remove filter error that the IMM cannot. Leave-one-kind-out
   (MLP trained without the kind, tested on it) is **worse than IMM2 on every kind and grossly over-confident**: cv RMSE@1.0 s 2.10 m vs 1.33 m, 90 % cover
   0.06; turn 13.9 m vs 2.8 m, cover 0.24; swarm5 cover 0.08; ca 0.14; cam 0.01 (the cam kind also has a different dt, so it is
   the extreme case). Even held-out cross and swarm5, which are plain constant-velocity-like targets close to cv, fail (cover 0.15 / 0.08).
   So: **do not ship these weights into fire control.** The experiment shows an upper bound of what a learned prior can give when the
   threat distribution is known and narrow, plus a failure mode (confidently wrong off-distribution) that the Ph would inherit.
5. **Recommendation:** (a) fix Ph now with a calibrated IMM covariance, conditioned on mode (e.g. fit s as a function of the IMM manoeuvre
   probability on the manoeuvring kinds), not with one scalar; (b) IMM3 is a cheap accuracy win over IMM2 on turn (RMSE@1.0 s 2.34 vs
   2.83 m, 90 % cover 0.96) at about equal cost; (c) revisit a learned residual only with a much wider and randomised training
   distribution (random start states, speeds, headings, accelerations, a yaw/speed augmentation) and a held-out-generator test that it
   passes; until the user decides on new threat profiles, that data cannot be built.

Caveats on how to read the tables: scenario kinds are the only ones in `napl.track.scenarios` (no new threat profiles were added);
the pooled rows weight kinds by sample count (swarm5 38 %, cross 21 %, cam 12 %, cv 10 %, turn 10 %, ca 8 %) so they follow the
benign kinds; samples inside one run are strongly correlated (CIs are bootstrapped over runs, 60 test runs per kind); coverage is 3-D
(ellipsoids, chi-square with 3 dof, 50 % = 2.366, 90 % = 6.251) of the position error only; the Ph in cuas2 also includes launcher dispersion, which is
not in these numbers; trained only for 40 epochs, 3 training seeds per model, no hyper-parameter search (the IMMs were tuned in
tracking-tuning.md, the nets were not), so the learned numbers are not tuned either way.

## Setup

- **Data** (`napl.track.learned.data`): for every scan of a run, each confirmed IMM2 track matched one-to-one to a truth target (GOSPA
  cut-off, 2 m, 6 m for cam) gives a sample: last N = 10 track means (p, v), their stds, IMM mode probabilities, the current 6x6 covariance, the
  IMM2 and IMM3 extrapolations (mean + 3x3 position covariance) at 0.2/0.4/0.6/0.8/1.0 s, and the truth position at those times. Both
  trackers run on the same detections; a sample is kept when both have a matched track. All six `scenarios.NAMES` kinds (cv, turn, ca,
  cross, swarm5, cam), trackers `imm2-gnn` and `imm3-gnn` of `variants`, default options. Splits by seed, disjoint from the 1..100 of the
  benchmark: train 1001-1160 (290 061 samples), val 2001-2040 (72 556), test 3001-3060 (108 791). Test samples per kind: cv 11 319, turn 11 310,
  ca 8 919, cross 22 619, swarm5 41 501, cam 13 123.
- **Baselines**: IMM2 and IMM3 (`Track.predict_to`, a single predict over the horizon: mixture mean and moment-matched covariance).
- **Learned** (`napl.track.learned.models`): input = history relative to the current position, log stds, mode probabilities, valid
  flags, current covariance, the IMM2 displacement and Cholesky factor of its predicted covariance per horizon, dt. Output per horizon:
  3 mean residual, 3 log-scales of the Cholesky diagonal of the IMM2 covariance, 3 lower-triangle terms; Gaussian NLL; last layer
  zero-initialised so training starts at IMM2. MLP: 2 x 128 SiLU, 50 221 parameters. GRU (48 units over the 10 scans) + same head, 46 525 parameters. AdamW, one-cycle lr 2e-3,
  batch 4096, 40 epochs, best-val-NLL epoch kept, 3 seeds each. Inference is plain numpy (`forward_np`, checked equal to the torch
  forward to 1e-7 in NLL).
- **Post-hoc IMM calibration**: covariance multiplied by s, fit on the val split only: ML scalar (s = mean d2 / 3 over all horizons), ML per horizon,
  and the q90 variants (s so that exactly 90 % of the val samples are inside the 90 % ellipsoid; robust to heavy tails). The oracle row fits one s_h per scenario kind.

## Results (test split, 3-D)

NLL includes the constant 1.5 ln(2 pi) = 2.757, so it is comparable between rows but not to NLLs elsewhere.

Pooled over kinds, per horizon (0.2 / 0.4 / 0.6 / 0.8 / 1.0 s):

| model                       | RMSE m                                | NLL                                 | cover 50 %                       | cover 90 %                       |
| --------------------------- | ------------------------------------- | ----------------------------------- | -------------------------------- | -------------------------------- |
| IMM2                        | 0.746 / 0.963 / 1.218 / 1.507 / 1.829 | 0.97 / 1.95 / 2.85 / 3.68 / 4.44    | 0.73 / 0.84 / 0.90 / 0.93 / 0.94 | 0.98 / 0.99 / 0.99 / 0.99 / 0.99 |
| IMM2 x s (ML scalar)        | same                                  | 0.95 / 1.66 / 2.38 / 3.08 / 3.74    | 0.35 / 0.47 / 0.59 / 0.70 / 0.78 | 0.77 / 0.87 / 0.92 / 0.94 / 0.95 |
| IMM2 x s_h (ML per horizon) | same                                  | 0.83 / 1.65 / 2.38 / 3.04 / 3.64    | 0.51 / 0.53 / 0.56 / 0.59 / 0.62 | 0.90 / 0.91 / 0.91 / 0.91 / 0.91 |
| IMM3                        | 0.719 / 0.922 / 1.159 / 1.426 / 1.719 | 0.84 / 1.74 / 2.58 / 3.35 / 4.06    | 0.71 / 0.81 / 0.88 / 0.92 / 0.93 | 0.97 / 0.98 / 0.99 / 0.99 / 0.99 |
| IMM3 x s_h (ML per horizon) | same                                  | 0.71 / 1.48 / 2.17 / 2.78 / 3.34    | 0.51 / 0.53 / 0.55 / 0.58 / 0.60 | 0.90 / 0.90 / 0.91 / 0.91 / 0.91 |
| IMM2 x s_h,kind (oracle)    | same as IMM2                          | 0.81 / 1.60 / 2.30 / 2.92 / 3.47    | 0.50 / 0.52 / 0.53 / 0.54 / 0.55 | 0.90 / 0.91 / 0.91 / 0.91 / 0.92 |
| MLP (3 seeds)               | 0.455 / 0.530 / 0.637 / 0.769 / 0.924 | -0.91 / -0.59 / -0.25 / 0.09 / 0.44 | 0.51 / 0.52 / 0.53 / 0.55 / 0.56 | 0.89 / 0.89 / 0.90 / 0.90 / 0.90 |
| GRU (3 seeds)               | 0.499 / 0.585 / 0.702 / 0.845 / 1.015 | -0.56 / -0.13 / 0.24 / 0.63 / 1.01  | 0.51 / 0.52 / 0.55 / 0.57 / 0.58 | 0.89 / 0.89 / 0.90 / 0.91 / 0.91 |

Per scenario kind (RMSE m at 0.2 / 0.6 / 1.0 s; NLL, cover 50 % and 90 % averaged over the five horizons):

| kind   | model             | RMSE m                | NLL    | cover 50 % | cover 90 % |
| ------ | ----------------- | --------------------- | ------ | ---------- | ---------- |
| cv     | IMM2              | 0.582 / 0.930 / 1.329 | 2.634  | 0.914      | 0.997      |
| cv     | IMM3              | 0.567 / 0.897 / 1.273 | 2.409  | 0.897      | 0.996      |
| cv     | IMM2 x s (scalar) | 0.582 / 0.930 / 1.329 | 2.046  | 0.632      | 0.934      |
| cv     | MLP               | 0.314 / 0.418 / 0.546 | -1.154 | 0.517      | 0.902      |
| cv     | GRU               | 0.357 / 0.494 / 0.664 | -0.508 | 0.534      | 0.914      |
| turn   | IMM2              | 0.788 / 1.616 / 2.834 | 3.641  | 0.733      | 0.931      |
| turn   | IMM3              | 0.697 / 1.365 / 2.339 | 2.905  | 0.794      | 0.959      |
| turn   | IMM2 x s (scalar) | 0.788 / 1.616 / 2.834 | 3.803  | 0.444      | 0.765      |
| turn   | MLP               | 0.515 / 0.941 / 1.627 | 0.593  | 0.521      | 0.870      |
| turn   | GRU               | 0.580 / 1.070 / 1.821 | 1.102  | 0.564      | 0.887      |
| ca     | IMM2              | 0.751 / 1.393 / 2.246 | 3.584  | 0.650      | 0.968      |
| ca     | IMM3              | 0.768 / 1.434 / 2.323 | 3.725  | 0.540      | 0.925      |
| ca     | IMM2 x s (scalar) | 0.751 / 1.393 / 2.246 | 3.758  | 0.291      | 0.703      |
| ca     | MLP               | 0.437 / 0.656 / 0.953 | -0.024 | 0.508      | 0.880      |
| ca     | GRU               | 0.437 / 0.637 / 0.898 | 0.231  | 0.529      | 0.884      |
| cross  | IMM2              | 0.600 / 0.979 / 1.414 | 2.710  | 0.909      | 0.994      |
| cross  | IMM3              | 0.588 / 0.953 / 1.371 | 2.514  | 0.893      | 0.993      |
| cross  | IMM2 x s (scalar) | 0.600 / 0.979 / 1.414 | 2.184  | 0.627      | 0.931      |
| cross  | MLP               | 0.353 / 0.545 / 0.782 | -0.496 | 0.585      | 0.921      |
| cross  | GRU               | 0.383 / 0.588 / 0.839 | -0.066 | 0.577      | 0.920      |
| swarm5 | IMM2              | 0.639 / 1.089 / 1.606 | 2.779  | 0.896      | 0.994      |
| swarm5 | IMM3              | 0.624 / 1.058 / 1.554 | 2.558  | 0.875      | 0.993      |
| swarm5 | IMM2 x s (scalar) | 0.639 / 1.089 / 1.606 | 2.271  | 0.598      | 0.920      |
| swarm5 | MLP               | 0.467 / 0.657 / 0.896 | 1.155  | 0.509      | 0.887      |
| swarm5 | GRU               | 0.470 / 0.673 / 0.925 | 1.158  | 0.506      | 0.890      |
| cam    | IMM2              | 1.219 / 1.607 / 2.063 | 1.731  | 0.927      | 0.998      |
| cam    | IMM3              | 1.164 / 1.530 / 1.959 | 1.295  | 0.915      | 0.997      |
| cam    | IMM2 x s (scalar) | 1.219 / 1.607 / 2.063 | 1.044  | 0.703      | 0.945      |
| cam    | MLP               | 0.605 / 0.527 / 0.568 | -4.327 | 0.574      | 0.914      |
| cam    | GRU               | 0.763 / 0.755 / 0.894 | -2.256 | 0.617      | 0.926      |

The full per-kind table (all calibration variants, d2/3, cover at 1.0 s) is the output of the `eval` command below.

Paired differences vs IMM2 (bootstrap over runs, 95 % CI, seed-0 models; negative = better). Mean squared 3-D error in m^2 (RMSE alone hides
this), NLL averaged over horizons:

| kind   | model      | d NLL                    | d sq. err @0.6 s     | d sq. err @1.0 s     |
| ------ | ---------- | ------------------------ | -------------------- | -------------------- |
| cv     | MLP        | -4.01 [-4.10, -3.92]     | -0.71 [-0.74, -0.68] | -1.49 [-1.56, -1.43] |
| cv     | IMM2 x s_h | -0.69 [-0.70, -0.67]     | 0                    | 0                    |
| turn   | MLP        | -3.18 [-3.23, -3.12]     | -1.75 [-1.81, -1.69] | -5.53 [-5.65, -5.38] |
| turn   | IMM2 x s_h | **+0.25 [+0.22, +0.28]** | 0                    | 0                    |
| ca     | MLP        | -3.65 [-3.73, -3.58]     | -1.52 [-1.57, -1.46] | -4.17 [-4.28, -4.04] |
| ca     | IMM2 x s_h | **+0.22 [+0.19, +0.24]** | 0                    | 0                    |
| cross  | MLP        | -3.38 [-3.46, -3.29]     | -0.68 [-0.70, -0.65] | -1.42 [-1.47, -1.36] |
| cross  | IMM2 x s_h | -0.61 [-0.63, -0.58]     | 0                    | 0                    |
| swarm5 | MLP        | -1.56 [-1.71, -1.33]     | -0.76 [-0.84, -0.69] | -1.78 [-1.99, -1.60] |
| swarm5 | IMM2 x s_h | -0.60 [-0.62, -0.57]     | 0                    | 0                    |
| cam    | MLP        | -6.03 [-6.12, -5.92]     | -2.30 [-2.52, -2.07] | -3.90 [-4.33, -3.45] |
| cam    | IMM2 x s_h | -0.76 [-0.80, -0.73]     | 0                    | 0                    |

(GRU rows are in the `eval` output; they are 0-2 NLL behind the MLP on every kind (cam the largest gap) and also exclude 0.)

Leave-one-kind-out, MLP seed 0 trained on the other five kinds (train + val filtered), tested on the kind:

| kind   | model        | RMSE m @0.2 / 0.6 / 1.0 s | NLL (mean h) | cover 50 % | cover 90 % |
| ------ | ------------ | ------------------------- | ------------ | ---------- | ---------- |
| cv     | MLP held out | 1.073 / 1.608 / 2.096     | 12.7         | 0.024      | 0.057      |
| cv     | IMM2         | 0.582 / 0.930 / 1.329     | 2.63         | 0.914      | 0.997      |
| turn   | MLP held out | 5.893 / 9.799 / 13.902    | 6.8e6        | 0.079      | 0.237      |
| turn   | IMM2         | 0.788 / 1.616 / 2.834     | 3.64         | 0.733      | 0.931      |
| ca     | MLP held out | 4.214 / 8.279 / 12.546    | 5.8e5        | 0.076      | 0.139      |
| ca     | IMM2         | 0.751 / 1.393 / 2.246     | 3.58         | 0.650      | 0.968      |
| cross  | MLP held out | 1.054 / 1.640 / 2.195     | 61.7         | 0.024      | 0.154      |
| cross  | IMM2         | 0.600 / 0.979 / 1.414     | 2.71         | 0.909      | 0.994      |
| swarm5 | MLP held out | 1.188 / 1.994 / 2.787     | 40.2         | 0.034      | 0.076      |
| swarm5 | IMM2         | 0.639 / 1.089 / 1.606     | 2.78         | 0.896      | 0.994      |
| cam    | MLP held out | 2.382 / 4.417 / 6.781     | 6.6e8        | 0.005      | 0.011      |
| cam    | IMM2         | 1.219 / 1.607 / 2.063     | 1.73         | 0.927      | 0.998      |

## Inference cost per track (CPU, plain numpy, 1 thread, batch 1)

Measured on hfbox (x86, `tools/ml_predict.py bench`, median of 900 calls; 300 for the IMM line). Jetson Orin Nano A78AE 1.5 GHz is not
available here; R3's scale factor of 3-5x is an estimate, not a measurement.

| item                                                      | us per track, hfbox | Jetson est. (x3-5) | parameters |
| --------------------------------------------------------- | ------------------- | ------------------ | ---------- |
| IMM2 extrapolation, 5 horizons (copy + predict + pv each) | 326                 | 1000-1600          |            |
| MLP: featurize + forward + decode                         | 84                  | 250-420            | 50 221     |
| GRU: featurize + forward + decode                         | 202                 | 600-1000           | 46 525     |

The learned residual is added on top of the IMM extrapolation (it takes it as input), so the learned total is IMM + 84 or + 202 us. The
IMM extrapolation itself dominates and could be vectorised over horizons (not done). Torch CPU latency was not measured.

## Reproduce

Needs torch for training only (`/opt/venv` on hfbox has torch 2.14+rocm7.2, numpy 2.5; nothing was installed). BSD (torch) licence, no GPL.

```
export NAPL_REMOTE_DIR=/workspace/wt/napl-c2ml            # data/ and models/ live next to src/, outside the rsync --delete
D=$NAPL_REMOTE_DIR; PY='PYTHONPATH=python /opt/venv/bin/python3 tools/ml_predict.py'
tools/remote.sh bg gen   "mkdir -p $D/data && $PY gen --out $D/data/ml.npz --jobs 2"                      # 19 min, 2 CPU processes, 290 MB
tools/remote.sh bg --gpu train "$PY train --data $D/data/ml.npz --out $D/models/main --seeds 3 --epochs 40"   # about 4 min total, MLP + GRU
for k in cv turn ca cross swarm5 cam; do tools/remote.sh run --gpu "$PY train --data $D/data/ml.npz --out $D/models/x_$k --seeds 1 --kinds mlp --exclude $k"; done
tools/remote.sh run "$PY eval --data $D/data/ml.npz --models $D/models/main --heldout $D/models --md $D/eval.md --json $D/eval.json"
tools/remote.sh run "$PY bench --data $D/data/ml.npz --models $D/models/main"
PYTHONPATH=python python -m pytest tests/python/test_track_learned.py -q     # 3 tests, 12 s, torch optional
```

Weights (about 200 KB per model) and the dataset are not committed; they stay on hfbox under `/workspace/wt/napl-c2ml/models`
(dataset deleted after the run).

## Not done

- No Ph computed: the numbers here are position-error calibration, not the cuas2 Ph (which also has dispersion and target size). The mapping
  from "IMM covariance 2.3x too large in variance" to "Ph 0.46 vs 0.67" is an inference, not a measurement.
- No mode-conditioned calibration of the IMM (s as a function of the IMM manoeuvre probability), the one remedy that the results point to.
- No randomised-generator training set and no augmentation (yaw, speed), because new threat profiles are pending a user decision; so the
  learned model's in-distribution gain is not shown to be anything beyond learning the scenario priors.
- No hyper-parameter search, no transformer, no Jetson measurement, no torch-CPU latency, no end-to-end hit-rate effect.
- Only IMM2 tracker output was used as the learned input; JPDA / MHT / PHD tracks were not used.
