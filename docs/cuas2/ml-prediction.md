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
   passes; until the user decides on new threat profiles, that data cannot be built. **(c) was run in ML-3 below: the mean generalises, the covariance does not (3 of 8 groups fail the cover criterion).**

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

## ML-2: mode- and innovation-conditioned IMM2 covariance calibration (negative result)

Goal: fit the covariance factor as a function of the IMM state so every kind gets 90 % cover within +-0.05. Method (`napl/track/calib.py`,
`ml_predict.py calib`): cov' = exp(w . phi) cov, phi = [1, h, h^2, q, q h, n, n h, a, a h], q = IMM2 manoeuvre (Singer) mode probability,
n = ln(mean NIS of the last k = 5 updates / 3), a = ln(|velocity change over ~0.5 s| / time + 0.2); w by maximum likelihood (convex Newton),
fitted on the val split (72 556 samples, seeds 2001-2040), reported on the test split (seeds 3001-3060). The dataset was regenerated with the
NIS history added (val/test identical to ML-1). Leave-one-kind-out refits without the kind's val samples.

**Result: not achieved.** Worst-kind |cover90 - 0.90| (mean over horizons):

| calibration                           | fit on all val kinds | leave-one-kind-out |
| ------------------------------------- | -------------------- | ------------------ |
| IMM2 raw                              | 0.098                | 0.098              |
| scalar s                              | 0.197                | 0.226              |
| s(h)                                  | 0.219                | 0.253              |
| s(h, mode)                            | 0.212                | 0.238              |
| s(h, NIS)                             | 0.218                | 0.251              |
| s(h, accel)                           | 0.207                | 0.236              |
| s(h, mode, NIS, accel)                | 0.212                | 0.237              |
| oracle s(h) per kind (needs the kind) | 0.040                | n/a                |

Per kind, cover 50 % / cover 90 % (mean over horizons) / cover 90 % @1.0 s / NLL, fit on all val kinds:

| model                  | cv                          | turn                        | ca                          | cross                       | swarm5                      | cam                         |
| ---------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- |
| IMM2 raw               | 0.91 / 0.997 / 0.999 / 2.63 | 0.73 / 0.931 / 0.923 / 3.64 | 0.65 / 0.968 / 0.992 / 3.58 | 0.91 / 0.994 / 0.996 / 2.71 | 0.90 / 0.994 / 0.998 / 2.78 | 0.93 / 0.998 / 1.000 / 1.73 |
| scalar s               | 0.63 / 0.934 / 0.994 / 2.05 | 0.44 / 0.765 / 0.801 / 3.80 | 0.29 / 0.703 / 0.780 / 3.76 | 0.63 / 0.931 / 0.992 / 2.18 | 0.60 / 0.920 / 0.989 / 2.27 | 0.70 / 0.945 / 0.981 / 1.04 |
| s(h)                   | 0.61 / 0.958 / 0.981 / 1.95 | 0.43 / 0.774 / 0.728 / 3.89 | 0.28 / 0.681 / 0.589 / 3.80 | 0.61 / 0.955 / 0.979 / 2.10 | 0.58 / 0.943 / 0.967 / 2.18 | 0.71 / 0.952 / 0.947 / 0.97 |
| s(h, mode)             | 0.58 / 0.954 / 0.981 / 1.88 | 0.52 / 0.866 / 0.856 / 3.71 | 0.34 / 0.688 / 0.593 / 3.97 | 0.57 / 0.949 / 0.977 / 2.05 | 0.54 / 0.940 / 0.969 / 2.10 | 0.67 / 0.935 / 0.922 / 0.90 |
| s(h, mode, NIS, accel) | 0.57 / 0.954 / 0.981 / 1.88 | 0.52 / 0.867 / 0.858 / 3.71 | 0.34 / 0.688 / 0.592 / 3.97 | 0.56 / 0.949 / 0.978 / 2.05 | 0.54 / 0.940 / 0.968 / 2.10 | 0.67 / 0.935 / 0.922 / 0.90 |
| oracle s(h) per kind   | 0.50 / 0.919 / 0.923 / 1.89 | 0.62 / 0.879 / 0.866 / 3.56 | 0.49 / 0.902 / 0.906 / 3.50 | 0.56 / 0.940 / 0.958 / 2.08 | 0.49 / 0.907 / 0.913 / 2.17 | 0.57 / 0.893 / 0.891 / 0.87 |

Leave-one-kind-out, same cells, for three of the models (the others are in `calib.md` from the command below; all have a worst kind above 0.2):

| model                  | cv                          | turn                        | ca                          | cross                       | swarm5                      | cam                         |
| ---------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- | --------------------------- |
| scalar s               | 0.64 / 0.938 / 0.994 / 2.05 | 0.42 / 0.744 / 0.788 / 3.89 | 0.27 / 0.674 / 0.750 / 3.83 | 0.64 / 0.935 / 0.992 / 2.19 | 0.63 / 0.934 / 0.991 / 2.29 | 0.72 / 0.950 / 0.984 / 1.06 |
| s(h, mode)             | 0.59 / 0.956 / 0.984 / 1.89 | 0.52 / 0.866 / 0.849 / 3.75 | 0.33 / 0.662 / 0.557 / 4.10 | 0.57 / 0.950 / 0.979 / 2.05 | 0.58 / 0.951 / 0.981 / 2.13 | 0.69 / 0.941 / 0.928 / 0.92 |
| s(h, mode, NIS, accel) | 0.58 / 0.957 / 0.984 / 1.89 | 0.52 / 0.867 / 0.851 / 3.75 | 0.33 / 0.663 / 0.557 / 4.10 | 0.57 / 0.950 / 0.980 / 2.05 | 0.58 / 0.952 / 0.981 / 2.13 | 0.69 / 0.941 / 0.928 / 0.92 |

Reading (facts from the tables, then inference):

- The raw IMM2 is the safest of everything fitted: its worst kind is 0.098 (always on the pessimistic side, cover90 up to 0.998), while every
  fitted calibration has a worst kind of 0.20-0.25, and the worst kinds are on the over-confident side (ca 0.66-0.70, turn 0.75-0.87).
- The mode probability helps turn (cover90 0.77 -> 0.87) and costs nothing elsewhere, but does not help ca (0.68-0.69; at 1.0 s 0.59);
  NIS of the last 5 updates and the velocity-change (acceleration) feature add essentially nothing (the cells of s(h, mode) and
  s(h, mode, NIS, accel) agree to 0.01). Inference: for ca the IMM state in these runs does not look different enough from the benign kinds at
  fit time for a smooth function of (q, n, a, h) to separate them; I did not diagnose further. A fixed 5-scan NIS mean is also a noisy
  statistic (3-dof chi-square, 5 values).
- Only the per-kind oracle gets within 0.05 (worst 0.040), so the needed information is the target class, not anything the IMM carries.
- Because ML (mean d2) is dominated by heavy-tailed benign samples, the fitted factors favour them; a quantile objective or class-conditioned
  fit (e.g. by discrimination class) were not tried.

The coefficients are not exported: `napl.track.calib` is not imported by `napl.track`, `DEFAULT_W` is the identity, and the Ph should keep the
raw (pessimistic, hence safe-side) covariance, or a scalar from a class known at runtime, until a better conditioning exists.

Reproduce ML-2 (dataset ~10 min on 2 CPU processes (40 train seeds), then the fit about 3 min; on hfbox the CPU lane may queue for hours behind MC runs):

```
$PY gen --out $D/data/ml2.npz --train 40 --jobs 2
$PY calib --data $D/data/ml2.npz --md $D/calib.md
PYTHONPATH=python python -m pytest tests/python/test_track_calib.py -q     # ML fit recovers a known factor; NaN-safe NIS feature
```

## ML-3: learned predictor on the randomised threat suite, leave-one-generator-out (ML-1 recommendation c)

Question: with a wide randomised training distribution (every `napl.threats` scene, the `random` / `random_cam` families with many seeds, yaw
and scale augmentation), does a learned residual predictor beat IMM2 extrapolation at 0.2-1.0 s and stay calibrated on a generator it never
saw? Scope: only the narrowed threat suite (waypoint routes, orbit / survey, weave, swarms, balloons, hanging loads, birds, debris); nothing
homing / pursuit / dive; nothing wired into cuas2 fire control.

### Conclusion

1. **Accuracy: yes, everywhere.** On every held-out generator the learned model has lower RMSE at 1.0 s than IMM2 (all 3 seeds, 9 of 9 groups,
   paired CI of the squared error excludes 0 on all 9): -8 % (route+swarm) to -46 % (hanging); pooled in-distribution 3.41 -> 2.45 m (-28 %),
   NLL 3.56 -> 2.03, with 90 % cover 0.891 (IMM2 0.918, IMM3 0.889). MLP and GRU are equal (2.452 vs 2.465 m); augmentation changes little (below).
2. **Calibration on a held-out generator: no, not reliably.** The pass criterion (held-out 90 % cover in [0.80, 0.97] AND RMSE@1.0 s not
   worse than IMM2) holds for **route, pattern, balloon, hanging, debris** (3/3 seeds; hanging 2/3 if all five horizons must be in range) and fails
   for **weave (cover 0.684), swarm (0.788), bird (0.676)** (0/3 seeds). The failures are over-confidence on random-walk manoeuvrers (the model
   learned the covariance of the generators it saw); the cover is 0.11-0.22 below the ideal 0.90.
3. **The route pass is partly leakage.** A swarm follows a leader route, so "route held out" still trains on route-like motion. Holding out
   route and swarm together fails (cover 0.675, NLL +1.11 [+0.97, +1.30] worse than IMM2 although RMSE is 8 % better). Read the route pass as
   "interpolation", not as a clean held-out test.
4. **Verdict: not usable in fire control as a calibrated predictor (fails 3 of 8 groups, 4 of 9 with route+swarm).** The mean is a robust gain; the
   learned covariance must not drive Ph on an unknown target type. A safe use would be learned mean with a covariance that cannot be smaller
   than a runtime-known floor (not tested). IMM2 itself is not a clean reference either: its cover is outside [0.80, 0.97] on weave (0.795), bird
   (0.687), balloon (0.997), hanging (0.994), i.e. it is over-confident on the hard classes and pessimistic on the benign ones.
5. Difference to ML-1: there the held-out kinds failed catastrophically (RMSE worse than IMM2, cover 0.01-0.24) because the six kinds were fixed
   paths; with random start states, speeds, parameters and many seeds the held-out RMSE is better than IMM2 on all groups and the cover loss is
   moderate (0.68-0.96 instead of 0.01-0.24). So recommendation (c) is borne out for the mean, not for the covariance.

### Setup

- **Data** (`napl.track.learned.data.collect(..., suite="threats")`, `tools/ml_predict.py gen --suite threats`): same sample definition as ML-1
  (last 10 IMM2 track means, IMM2 and IMM3 extrapolations at 0.2 ... 1.0 s, truth at those times), tracker `imm2-gnn` / `imm3-gnn` on the
  `napl.threats` scene's own sensor (pos or cam). Birth/death is honoured (`alive`: dead objects cannot match, the truth must exist at +1.0 s).
  Every 3rd scan (0.15 s). Per seed: the 14 named scenes plus `random` and `random_cam` (5 x as many seeds, seed + 10000 j). Splits by seed, disjoint
  from each other and from the benchmark: train 1001-1060 (+10000 j), val 2001-2015, test 3001-3030; samples train 531 878, val 133 278,
  test 271 332 (720 test runs). Each sample carries the generator group of its **object** (`meta[:, 5]`) and the class label (`meta[:, 6]`),
  so mixed scenes are split per object, not per scene.
- **Groups** (`data.GROUPS`): route (`route`, `route_fast`), pattern (`orbit`, `survey`), weave, swarm, balloon, hanging, bird, debris (`leaf`, `bag`).
  Test samples: route 27 118, pattern 12 435, weave 11 971, swarm 77 441, balloon 40 116, hanging 34 830, bird 41 170, debris 26 251.
- **Leave-one-generator-out (LOGO)**: train and val without the group's objects (the tracker still ran on the full scenes), same hyper-parameters, 3
  seeds per group, tested on the group's test samples. Extra: route+swarm held out together.
- **Models**: unchanged from ML-1 (MLP 2 x 128 SiLU, GRU 48 units + same head, Gaussian NLL, zero-initialised head, 40 epochs, best-val epoch).
  Main models: 3 seeds MLP and GRU on train + one augmented copy; ablation: 3 seeds MLP without augmentation.
- **Augmentation** (`data.augment`): per sample a random yaw about the vertical axis (positions, velocities, 6x6 / 3x3 covariances, truth rotate) and
  a log-uniform world scale 0.7-1.4 (positions, velocities x s; covariances x s^2: a faster or slower target on a bigger or smaller range at the same
  sensor rate). Approximations: per-scan x/y stds rotated as if uncorrelated; the scaled IMM covariance is not what the tracker would output on a
  scaled scene. Test data are not augmented.
- **Pass criterion**: held-out 90 % cover (3-D ellipsoid, mean over the five horizons) in [0.80, 0.97] AND RMSE@1.0 s <= IMM2. The table also
  shows the stricter rule (all five horizons in range). Paired bootstrap CIs are over runs (1000 resamples), seed-0 models.

### Results (test split)

In-distribution (main models, seeds averaged; the model saw all groups), RMSE m at 0.2 / 0.6 / 1.0 s, NLL (mean over horizons, includes the constant
2.757), cover 90 % (mean over horizons):

| scope (n)        | IMM2                                | IMM3                                | MLP (3 seeds)                        | GRU (3 seeds)                        | MLP no aug (3 seeds)                 |
| ---------------- | ----------------------------------- | ----------------------------------- | ------------------------------------ | ------------------------------------ | ------------------------------------ |
| pooled (271 332) | 1.04 / 2.03 / 3.41 \| 3.56 \| 0.918 | 1.03 / 2.01 / 3.38 \| 3.40 \| 0.889 | 0.81 / 1.47 / 2.45 \| 2.03 \| 0.891  | 0.81 / 1.48 / 2.47 \| 2.04 \| 0.894  | 0.81 / 1.47 / 2.44 \| 2.08 \| 0.891  |
| route (27 118)   | 0.95 / 1.65 / 2.59 \| 2.84 \| 0.958 | 0.99 / 1.70 / 2.65 \| 2.72 \| 0.906 | 0.79 / 1.30 / 2.03 \| 1.94 \| 0.867  | 0.79 / 1.30 / 2.04 \| 1.94 \| 0.870  | 0.79 / 1.32 / 2.07 \| 1.98 \| 0.866  |
| pattern (12 435) | 1.01 / 1.82 / 2.93 \| 3.42 \| 0.946 | 1.00 / 1.78 / 2.84 \| 3.22 \| 0.899 | 0.82 / 1.35 / 2.13 \| 2.46 \| 0.894  | 0.81 / 1.37 / 2.18 \| 2.53 \| 0.888  | 0.81 / 1.35 / 2.14 \| 2.46 \| 0.897  |
| weave (11 971)   | 1.21 / 2.57 / 4.56 \| 4.75 \| 0.795 | 1.16 / 2.48 / 4.40 \| 4.66 \| 0.761 | 0.99 / 2.00 / 3.47 \| 4.14 \| 0.794  | 1.00 / 2.03 / 3.54 \| 4.17 \| 0.792  | 1.00 / 2.04 / 3.55 \| 4.21 \| 0.788  |
| swarm (77 441)   | 0.82 / 1.57 / 2.58 \| 3.44 \| 0.951 | 0.84 / 1.59 / 2.60 \| 3.40 \| 0.893 | 0.72 / 1.31 / 2.11 \| 2.92 \| 0.848  | 0.72 / 1.31 / 2.11 \| 2.92 \| 0.851  | 0.72 / 1.32 / 2.14 \| 2.95 \| 0.850  |
| balloon (40 116) | 0.89 / 1.32 / 1.83 \| 2.40 \| 0.997 | 0.85 / 1.25 / 1.71 \| 1.88 \| 0.996 | 0.71 / 0.91 / 1.18 \| 0.84 \| 0.955  | 0.70 / 0.91 / 1.17 \| 0.86 \| 0.959  | 0.71 / 0.92 / 1.20 \| 0.93 \| 0.955  |
| hanging (34 830) | 0.79 / 1.25 / 1.79 \| 2.15 \| 0.994 | 0.75 / 1.18 / 1.66 \| 1.64 \| 0.990 | 0.51 / 0.60 / 0.76 \| -0.96 \| 0.965 | 0.48 / 0.58 / 0.73 \| -0.94 \| 0.969 | 0.50 / 0.60 / 0.76 \| -0.85 \| 0.966 |
| bird (41 170)    | 1.58 / 3.59 / 6.42 \| 6.66 \| 0.687 | 1.58 / 3.58 / 6.40 \| 7.05 \| 0.667 | 1.14 / 2.52 / 4.50 \| 3.43 \| 0.884  | 1.14 / 2.53 / 4.53 \| 3.46 \| 0.887  | 1.14 / 2.48 / 4.38 \| 3.45 \| 0.883  |
| debris (26 251)  | 1.08 / 1.71 / 2.50 \| 2.95 \| 0.968 | 1.05 / 1.66 / 2.40 \| 2.53 \| 0.961 | 0.86 / 1.16 / 1.56 \| 1.87 \| 0.898  | 0.85 / 1.16 / 1.55 \| 1.93 \| 0.902  | 0.86 / 1.17 / 1.58 \| 1.98 \| 0.899  |

Even in-distribution the learned 90 % cover is 0.85-0.87 on route and swarm and 0.79 on weave: the model is not perfectly calibrated on the
manoeuvring groups it was trained on. Augmentation is not what makes the result: without it the pooled numbers are the same within 0.02 m / 0.05 NLL
(NLL is lower with it on 7 of 8 groups, tied on pattern; RMSE@1.0 s is 0.00-0.08 m lower with it on 7 of 8 groups and 0.13 m higher on bird, 0.02 m higher pooled).

Leave-one-generator-out (MLP trained without the group, mean of 3 seeds; "in-dist" = seed-0 main model that saw the group):

| held-out group | RMSE m @1.0 s: IMM2 / IMM3 / LOGO / in-dist | NLL: IMM2 / LOGO / in-dist | cover 90 %: IMM2 / LOGO (min-max over h) / in-dist | cover 50 % LOGO | RMSE@1.0 s <= IMM2 | cover in [0.80, 0.97] | **PASS**          |
| -------------- | ------------------------------------------- | -------------------------- | -------------------------------------------------- | --------------- | ------------------ | --------------------- | ----------------- |
| route          | 2.59 / 2.65 / 2.10 / 2.03                   | 2.84 / 2.45 / 1.95         | 0.958 / 0.838 (0.81-0.87) / 0.865                  | 0.546           | 3/3                | 3/3                   | 3/3               |
| pattern        | 2.93 / 2.84 / 2.23 / 2.15                   | 3.42 / 2.71 / 2.48         | 0.946 / 0.864 (0.85-0.89) / 0.890                  | 0.523           | 3/3                | 3/3                   | 3/3               |
| weave          | 4.56 / 4.40 / 3.64 / 3.46                   | 4.75 / 4.84 / 4.15         | 0.795 / 0.684 (0.62-0.77) / 0.795                  | 0.324           | 3/3                | 0/3                   | **0/3**           |
| swarm          | 2.58 / 2.60 / 2.23 / 2.11                   | 3.44 / 3.45 / 2.93         | 0.951 / 0.788 (0.77-0.82) / 0.847                  | 0.445           | 3/3                | 0/3                   | **0/3**           |
| balloon        | 1.83 / 1.71 / 1.35 / 1.18                   | 2.40 / 1.45 / 0.83         | 0.997 / 0.951 (0.94-0.97) / 0.954                  | 0.729           | 3/3                | 3/3                   | 3/3               |
| hanging        | 1.79 / 1.66 / 0.97 / 0.76                   | 2.15 / 0.54 / -0.95        | 0.994 / 0.957 (0.94-0.97) / 0.964                  | 0.781           | 3/3                | 3/3                   | 3/3 (2/3 every-h) |
| bird           | 6.42 / 6.40 / 5.25 / 4.51                   | 6.66 / 5.82 / 3.43         | 0.687 / 0.676 (0.66-0.71) / 0.883                  | 0.373           | 3/3                | 0/3                   | **0/3**           |
| debris         | 2.50 / 2.40 / 1.73 / 1.56                   | 2.95 / 2.32 / 1.89         | 0.968 / 0.863 (0.85-0.88) / 0.897                  | 0.565           | 3/3                | 3/3                   | 3/3               |
| route+swarm    | 2.58 / 2.62 / 2.37 / 2.09                   | 3.28 / 4.41 / 2.68         | 0.953 / 0.675 (0.63-0.75) / 0.852                  | 0.350           | 3/3                | 0/3                   | **0/3**           |

Paired differences of the seed-0 LOGO model against the baselines (95 % CI over runs; negative = learned better; full table with the in-distribution
rows and the per-label scopes are in the `eval-threats` output, `/workspace/wt/napl-c2ml3/eval.md` on hfbox, not committed):

| held-out group | d NLL vs IMM2            | d sq. err m^2 @1.0 s vs IMM2 | d NLL vs IMM3            | d sq. err m^2 @1.0 s vs IMM3 |
| -------------- | ------------------------ | ---------------------------- | ------------------------ | ---------------------------- |
| route          | -0.39 [-0.60, -0.19]     | -2.35 [-2.66, -2.06]         | -0.27 [-0.47, -0.07]     | -2.65 [-3.02, -2.29]         |
| pattern        | -0.72 [-0.81, -0.62]     | -3.53 [-4.52, -2.75]         | -0.51 [-0.62, -0.41]     | -3.01 [-3.86, -2.30]         |
| weave          | +0.10 [-0.08, +0.27]     | -7.60 [-9.18, -6.12]         | +0.19 [+0.01, +0.38]     | -6.17 [-7.50, -4.93]         |
| swarm          | +0.00 [-0.11, +0.11]     | -1.70 [-1.91, -1.51]         | +0.03 [-0.05, +0.12]     | -1.82 [-2.06, -1.59]         |
| balloon        | -0.97 [-1.02, -0.90]     | -1.51 [-1.75, -1.28]         | -0.44 [-0.50, -0.38]     | -1.07 [-1.31, -0.84]         |
| hanging        | -1.87 [-2.05, -1.58]     | -2.26 [-2.50, -2.05]         | -1.36 [-1.54, -1.07]     | -1.83 [-2.06, -1.60]         |
| bird           | -0.89 [-1.04, -0.74]     | -13.88 [-14.99, -12.91]      | -1.29 [-1.45, -1.12]     | -13.57 [-14.60, -12.66]      |
| debris         | -0.65 [-0.83, -0.36]     | -3.28 [-4.10, -2.59]         | -0.23 [-0.41, +0.06]     | -2.81 [-3.64, -2.15]         |
| route+swarm    | **+1.11 [+0.97, +1.30]** | -1.07 [-1.27, -0.90]         | **+1.17 [+1.03, +1.34]** | -1.23 [-1.47, -1.04]         |

Pooled in-distribution (seed-0 MLP vs IMM2): d NLL -1.53 [-1.60, -1.46], d sq. err @1.0 s -5.62 [-6.10, -5.18] m^2; vs IMM3: -1.37 [-1.45, -1.30] and -5.39 [-5.85, -4.95].

Reading (facts from the tables, then inference):

- RMSE gain over IMM2 holds on all nine hold-outs, so the mean prediction generalises across these generators. The NLL gain does not: weave and swarm
  are tied with IMM2 (CI contains 0), route+swarm is clearly worse, because the covariance is too small there (d2/3 of the learned model 2.0 / 1.7 / 2.7
  on weave / swarm / route+swarm against 1.36 / 0.74 / 0.72 for IMM2; 1.0 = calibrated).
- The cover failures (weave, swarm, bird, route+swarm) are the high-uncertainty generators: random-walk turn rate and speed. A model that has not seen
  that kind of unpredictability does not scale its covariance up enough. Inference: the predicted covariance is learned mostly as a function of the
  generator's typical dispersion, so a held-out generator with larger dispersion than any training generator is under-covered; I did not test this directly.
- On weave and bird IMM2 itself has cover below 0.80 (0.795, 0.687), so those two are hard for any predictor here; on swarm (IMM2 0.951) and
  route+swarm (0.953) IMM2 is fine and the learned model is the one that loses cover. The criterion tests the learned model, not IMM2.

### Reproduce

```
export NAPL_REMOTE_DIR=/workspace/wt/napl-c2ml3; D=$NAPL_REMOTE_DIR; PY='OMP_NUM_THREADS=1 PYTHONPATH=python /opt/venv/bin/python3 tools/ml_predict.py'
G="$PY gen --suite threats --rand-mult 5 --stride 3 --jobs 8"          # CPU lane, 1 slot; one file per split survives a killed job
tools/remote.sh bg --slots 1 gen "$G --splits test --test 30 --out $D/data/ml3_test.npz; $G --splits val --val 15 --out $D/data/ml3_val.npz; $G --splits train --train 60 --out $D/data/ml3_train.npz"   # 10 + 6 + 24 min under load
T="NAPL_GPU_FRAC=0.2 $PY train --suite threats --data $D/data/ml3_train.npz,$D/data/ml3_val.npz"                 # --data takes comma-joined files
tools/remote.sh run --gpu "$T --out $D/models/main --seeds 3 --aug 1; $T --out $D/models/main_noaug --seeds 3 --kinds mlp"   # one GPU job at a time
for g in route pattern weave swarm balloon hanging bird debris route+swarm; do tools/remote.sh run --gpu "$T --out $D/models/x_$g --seeds 3 --kinds mlp --aug 1 --exclude-group $g"; done
tools/remote.sh run "$PY eval-threats --data $D/data/ml3_test.npz --models $D/models/main --noaug $D/models/main_noaug --logo $D/models --md $D/eval.md --json $D/eval.json"
PYTHONPATH=python python -m pytest tests/python/test_track_learned.py -q       # threat collect path + augmentation consistency
```

`NAPL_GPU_FRAC` (default 0.1) is the CUDA memory cap as a fraction of 16 GB; the augmented set needs 0.2 (3.2 GB, under the 4 GB GTT rule). Training
takes 50-90 s per MLP and 165 s per GRU on hfbox; several models of one process run out of memory unless the GPU is freed between seeds (fixed in `models.train`).
Operational notes: a first multi-seed attempt in one process hit the memory cap at the second seed; one run was killed (exit 137) after 2 of 3 `route+swarm`
seeds and resumed with `--first-seed`; models differ only by the seed, so resuming does not change them.

Kept on hfbox: dataset files `/workspace/wt/napl-c2ml3/data/ml3_{train,val,test}.npz` (about 0.55 GB) and models `/workspace/wt/napl-c2ml3/models` (about 5 MB, 36 models: main
3 MLP + 3 GRU, no-aug 3 MLP, 27 LOGO MLP); not committed.

### ML-3 not done

- Held-out sensors (train on pos, test on cam) and held-out trackers (JPDA / MHT / PHD tracks as input) were not tried; `random_cam` and the cam scenes are in the training mix.
- LOGO used one hyper-parameter setting and 3 seeds per group; no search, no model with a covariance floor, no hybrid "learned mean + IMM covariance". Whether
  such a hybrid passes the cover criterion was not measured.
- No Ph or hit rate computed, no Jetson latency; the ML-1 inference cost table is the only cost estimate (same architecture: MLP 50 221 parameters).
- The only held-out unit is the generator. The tests share the random parameter ranges of `gen.*_params` with training, so a target outside those ranges
  (faster, more agile than the suite) was not tested; scale augmentation 0.7-1.4 only partly covers that.
- Group definitions are mine (route = route + route_fast, pattern = orbit + survey, debris = leaf + bag); other splits (e.g. route_fast alone) were not run.

## ML-4: learned mean + safe covariance hybrids (negative result)

### Conclusion

1. **No hybrid passes all groups.** Best counts of groups passing the ML-3 criterion in 3/3 seeds (held-out 90 % cover, mean over horizons, in [0.80, 0.97]
   AND RMSE@1.0 s <= IMM2), out of the 8 groups (9 with route+swarm): plain learned 5/8 (route+swarm: 0/3), IMM2 itself 4/8, `mix/guard` 5/8,
   `floor/guard` 4/8, `imm2cov/guard` 2/8. The hybrids move which groups fail; they do not remove the failures.
2. **The mean gain is kept** (the hybrids never change the mean: RMSE@1.0 s is the learned model's on every group, -8 % to -46 % vs IMM2, ML-3).
3. **Covariances that repair the hard groups break the benign ones.** `imm2cov/guard` (IMM2 covariance at the learned mean, scaled k=1 plus the
   disagreement term) fixes weave (0.684 -> 0.909), swarm (0.788 -> 0.964) and route+swarm (0.675 -> 0.961) but is pessimistic on route 0.975, pattern
   0.975, balloon 0.999, hanging 0.999, debris 0.989 (all above 0.97). `floor/guard` (learned covariance floored at 0.25 x IMM2 plus 0.5 x disagreement)
   keeps route / pattern / swarm / debris in range but leaves weave 0.783, bird 0.742, balloon 0.985, hanging 0.988, route+swarm 0.785 (2/3 seeds).
4. **Bird fails under every variant** (best 0.749 with `mix/guard`; IMM2 itself is 0.687): a random-walk manoeuvrer is beyond both the IMM2 and the learned
   covariance, so no combination of the two inflates enough, and the fit on the other groups has no way to know.
5. **Calibration is better in NLL even where cover fails**: held-out NLL (mean h) is below IMM2 for weave (`imm2cov/guard` 4.44 vs IMM2 4.75 vs learned 4.84),
   swarm (`floor` 3.17 vs 3.44), bird (`floor/guard` 5.35 vs 6.66), route+swarm (`imm2cov/nll` 3.21 vs 3.28). The learned-mean hybrids are strictly better
   predictors than IMM2 in mean and NLL; they are not reliably calibrated to the [0.80, 0.97] band on an unseen generator.
6. **Verdict**: still not a fire-control Ph source for an unknown target type. If the mean alone is used for lead and Ph keeps the IMM2-based covariance,
   `imm2cov` with k fitted on the training groups is the option that was best on the groups where IMM2 is over-confident (weave / swarm / route+swarm);
   the pessimism on benign classes is IMM2's own (cover 0.99+ on balloon / hanging).

### Method

Code: `napl.track.learned.hybrid` (`hybrid`, `fit`), driver `tools/ml_predict.py eval-threats --hybrid`. Inputs: the 27 ML-3 LOGO MLPs (9 held-out units x 3
seeds), the ML-3 val / test splits. With mu_l, C_l the learned mean / covariance, mu2, C2 the IMM2 extrapolation, d = mu_l - mu2:

| mode          | covariance (mean is always mu_l)                                                                                                  |
| ------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `imm2cov` (a) | k C2 + g d d^T                                                                                                                    |
| `floor` (b)   | C_l + (k C2 - C_l)_+ + g d d^T; (.)_+ = positive eigen-part, so C is >= both C_l and k C2 in the Loewner order (the matrix "max") |
| `mix` (b)     | (1-k) C_l + k C2 + g d d^T, k in [0, 1]                                                                                           |

g d d^T is option (c), the disagreement inflation. (k, g) come from a grid (k: 0.25 ... 4 or 0 ... 1 in 0.1 steps; g in {0, 0.5, 1, 2}) fitted **per LOGO model
on the val split without the held-out group** (every 2nd val sample), never on the held-out group or on test. Two fit objectives: `nll` = minimum pooled val
NLL; `guard` = minimum val NLL among the grid points whose worst training-group val 90 % cover (mean over horizons) is >= 0.85 (best worst-group cover if
none reaches it).

### Pass table (test split; seeds passing mean-h rule of 3; cover = 90 % cover, mean over horizons, mean over the 3 seeds)

| held out          | IMM2       | learned (ML-3) | imm2cov/guard | floor/guard | mix/guard | imm2cov/nll | floor/nll |
| ----------------- | ---------- | -------------- | ------------- | ----------- | --------- | ----------- | --------- |
| route             | 0.958 pass | 0.838 3/3      | 0.975 0/3     | 0.903 3/3   | 0.874 3/3 | 0.883 3/3   | 0.876 3/3 |
| pattern           | 0.946 pass | 0.864 3/3      | 0.975 0/3     | 0.901 3/3   | 0.883 3/3 | 0.848 3/3   | 0.887 3/3 |
| weave             | 0.795 FAIL | 0.684 0/3      | 0.909 3/3     | 0.783 0/3   | 0.785 0/3 | 0.722 0/3   | 0.783 0/3 |
| swarm             | 0.951 pass | 0.788 0/3      | 0.964 3/3     | 0.831 3/3   | 0.825 3/3 | 0.842 3/3   | 0.831 3/3 |
| balloon           | 0.997 FAIL | 0.951 3/3      | 0.999 0/3     | 0.985 0/3   | 0.968 3/3 | 0.983 0/3   | 0.976 0/3 |
| hanging           | 0.994 FAIL | 0.957 3/3      | 0.999 0/3     | 0.988 0/3   | 0.987 0/3 | 0.987 0/3   | 0.982 0/3 |
| bird              | 0.687 FAIL | 0.676 0/3      | 0.726 0/3     | 0.742 0/3   | 0.749 0/3 | 0.530 0/3   | 0.692 0/3 |
| debris            | 0.968 pass | 0.863 3/3      | 0.989 0/3     | 0.952 3/3   | 0.918 3/3 | 0.953 3/3   | 0.929 3/3 |
| route+swarm       | 0.953 pass | 0.675 0/3      | 0.961 3/3     | 0.785 2/3   | 0.751 0/3 | 0.829 3/3   | 0.746 0/3 |
| groups 3/3 (of 8) | 4          | 5              | 2             | 4           | 5         | 4           | 4         |

(IMM2 "pass" = cover in range; its RMSE is the reference.) Under the stricter every-horizon rule: `floor/guard` keeps route, pattern, swarm (3/3), loses
debris (0/3); `imm2cov/guard` passes weave only; `mix/guard` passes route, pattern, swarm, debris 2/3. Full table with k / g, NLL, RMSE, min-max cover over
horizons and both rules: `tools/ml_predict.py eval-threats --hybrid` output (not committed; on hfbox `/workspace/wt/napl-c2ml3/hybrid.md`; the `--json` write crashed on numpy ints in that run, fixed afterwards and not re-run).

### Caveats

- The `floor` fits sit at the lower grid edge (k = 0.25) for the `nll` and `guard` objectives on every group: smaller floors were not tried. for `floor`, `guard` raises g, not k.
- The fit set is the training groups' val split, which the LOGO models early-stopped on (mildly optimistic for the learned covariance). On those groups the
  learned model is already near-calibrated, so the fit has no signal about how over-confident it is on an unseen generator; that is the core problem and
  the reason `nll` fits barely move away from the plain learned covariance. `guard` at 0.85 is a design choice, not tuned.
- Test splits and models are ML-3's: same suite, same random parameter ranges. Cover is position-error calibration, not Ph.

### Reproduce

```
export NAPL_REMOTE_DIR=/workspace/wt/napl-c2ml3; D=$NAPL_REMOTE_DIR     # data + 36 ML-3 models live here (outside the rsync --delete of src/)
tools/remote.sh run --slots 1 "OMP_NUM_THREADS=1 PYTHONPATH=python /opt/venv/bin/python3 tools/ml_predict.py eval-threats --hybrid --data $D/data/ml3_test.npz --val $D/data/ml3_val.npz --logo $D/models --md $D/hybrid.md --json $D/hybrid.json"   # CPU only, ~30 min
PYTHONPATH=python python -m pytest tests/python/test_track_learned.py -q -k hybrid
```

### ML-4 not done

- No retraining (e.g. a loss that penalises over-confidence across groups, or group-wise robust training); no floors below k = 0.25; no per-horizon k.
- Not applied to the GRU or to the main (in-distribution) models; not wired into cuas2 fire control; no Ph or hit-rate evaluation.
- Bird-like random-walk targets remain uncovered by every option tried.

## Not done

- No Ph computed: the numbers here are position-error calibration, not the cuas2 Ph (which also has dispersion and target size). The mapping
  from "IMM covariance 2.3x too large in variance" to "Ph 0.46 vs 0.67" is an inference, not a measurement.
- Mode/NIS/acceleration-conditioned calibration was tried (ML-2 above) and did not work; class-conditioned and quantile-objective fits were not tried.
- (Done in ML-3 above: randomised-generator training set, yaw / scale augmentation, held-out-generator test. The ML-1 numbers stand as the narrow-distribution case.)
- No hyper-parameter search, no transformer, no Jetson measurement, no torch-CPU latency, no end-to-end hit-rate effect.
- Only IMM2 tracker output was used as the learned input; JPDA / MHT / PHD tracks were not used.
