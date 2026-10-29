# cuas2 low aim on static targets: root cause (P13 BIAS)

## Conclusion

1. **About 40 % of the "-4 cm static balloon bias" was the measurement, not the aim.** The lite referee takes the smallest 5 ms-sampled ball-target distance. A ball that hits is removed at the (ball-radius inflated) target surface, so its last sample lies short of the sphere on the incoming side; a ball climbing at 10-20 deg then reads about `-r sin(el)` low. Paired per shot (balloon, 103 balls): lite referee minus refined closest approach = **-0.017 +- 0.004 m**. Fixed in `Referee2` (`napl/cuas2/miss.py`, segment minimum plus 20 ms extrapolation of a vanished ball); the sim is untouched (final tree reproduces V0 shots and hits episode for episode, 12 of 12).
2. **The real aim of static balloons is -0.022 +- 0.026 m (refined, 80 episodes, 103 balls): not distinguishable from zero.** Nothing geometric is wrong: kernel muzzle minus the assumed `geom.pose` muzzle is 0.000 m, camera position error 0.000 m, python ballistics vs the kernel ball (`T_phys`) +0.000 +- 0.001 m, tracker position error (z) -0.003 +- 0.003 m. The best-resolved remaining term is launch speed: the plant fires **0.42 +- 0.32 m/s (1.5 %) slower than the app's `mean_v0`** (wheels about 0.3 % under the set point at the nip plus dip), worth -0.018 +- 0.012 m (`T_spd`; -0.2 cm at 3 m, about -3 cm at 10 m by the solver sensitivity). Not changed: `Calib.launcher.k_slip` is the knob, and the bench fit B2 (`calib.fit_*`) sets it from measured speed (bench item below).
3. **Hanging targets (-0.073 m refined, 80 episodes, 145 balls) are not the same term; they are pendulum lead plus a lead-time error.** `T_est` -0.063 +- 0.019 m: pendulum arc curvature against the constant-velocity lead (`E_curv`, computed from the TRUTH state so independent of the tracker) -0.039 +- 0.010 m, lead-time error `E_time` -0.009 +- 0.003 m z and **+0.029 +- 0.011 m lateral**; speed -0.016 +- 0.014 m. The lead-time error is a real model bug (below), but correcting it made the kill rate worse in this sim and was reverted.

## The lead horizon is 0.1 s short (found, not fixed)

`FireControl2` leads a target by `(now - tracker_t) + t_actuator + t_fire` with lite's `t_actuator 0.020, t_fire 0.0`. In the cuas2 plant the ball reaches the muzzle **0.120 +- 0.001 s** after the FIRE decision (`rig.fire` time minus `fire_sent`; `T_EXIT` 0.118 in `app.py`; spec 8 says 0.06 + 0.045). The app extrapolated over 0.338 s where the ball arrived 0.437 s after the scan (hanging, 80 episodes). With `t_fire = 0.10` the two agree (0.352 s = 0.352 s) and `E_time` goes to 0.000 +- 0.001 m, but on the same seeds (paired):

| config                                   | episodes | balls | hit/ball          | kills/episode     | episodes with more / fewer kills |
| ---------------------------------------- | -------- | ----- | ----------------- | ----------------- | -------------------------------- |
| hanging, `t_fire` 0.00                   | 50       | 109   | 0.30 [0.22, 0.39] | 0.66 [0.52, 0.78] |                                  |
| hanging, `t_fire` 0.10                   | 50       | 140   | 0.15 [0.10, 0.22] | 0.42 [0.29, 0.56] | 4 / 16                           |
| hanging, `t_fire` 0.12 (40 ep of the 50) | 40       | 109   | 0.13 [0.08, 0.20] | 0.35 [0.22, 0.50] | 1 / 13                           |
| balloon, `t_fire` 0.00                   | 80       | 103   | 0.73 [0.64, 0.80] | 0.94 [0.86, 0.97] |                                  |
| balloon, `t_fire` 0.10                   | 80       | 92    | 0.72 [0.62, 0.80] | 0.82 [0.73, 0.89] | 0 / 9                            |

A longer horizon multiplies the tracker velocity error (`E_vel` +0.024 +- 0.008 m on a static balloon, the track "moves") and the pendulum curvature (grows with the horizon squared), and the Ph gate sees a wider cloud. Which of these costs the kills was not isolated (inferred). So `FireCfg2.t_fire` stays 0.0 (knob unchanged) and the value is a bench item; a useful fix needs a pendulum/acceleration-aware lead and velocity smoothing first. This matters most for the crossing quad scenarios (not run here): a 3-8 m/s target is led 0.3-0.8 m short by 0.1 s.

## Decomposition (balloon, `examples/cuas2/aimdiag.py`, V0 = default config, 80 episodes, 103 balls; z in m, 95 % CI clustered by episode)

Terms are exact sequential differences of five path evaluations at the plane through the true target centre normal to the line of sight (`aimdiag.py` docstring): `T_est` solved aim vs true target, `T_muz` kernel muzzle vs assumed, `T_dir` launch direction vs solved (`T_scat` launcher scatter draw, zero-mean by construction, `T_point` the barrel as pointed), `T_spd` plant v0 / spin / ball / wind vs model, `T_phys` python vs kernel flight.

| term                                 | balloon z       | hanging z       | hanging y       |
| ------------------------------------ | --------------- | --------------- | --------------- |
| lite referee (5 ms sampled)          | -0.039 +- 0.020 | -0.078 +- 0.025 |                 |
| refined closest approach             | -0.022 +- 0.026 | -0.073 +- 0.024 |                 |
| at the plane (sum below)             | -0.022 +- 0.026 | -0.070 +- 0.024 | -0.019 +- 0.025 |
| `T_est`                              | +0.003 +- 0.008 | -0.063 +- 0.019 | +0.022 +- 0.031 |
| - `E_pos` tracker position           | -0.003 +- 0.003 | +0.002 +- 0.004 | +0.005 +- 0.004 |
| - `E_vel` tracker velocity x horizon | +0.014 +- 0.007 | +0.006 +- 0.014 | +0.009 +- 0.010 |
| - `E_curv` CV lead on the true arc   | 0.000           | -0.039 +- 0.010 | -0.021 +- 0.016 |
| - `E_time` horizon 0.1 s short       | 0.000           | -0.009 +- 0.003 | +0.029 +- 0.011 |
| `T_muz` muzzle geometry              | 0.000 +- 0.000  | 0.000 +- 0.000  | -0.002 +- 0.001 |
| `T_dir` launch direction             | -0.007 +- 0.019 | +0.008 +- 0.018 | -0.040 +- 0.025 |
| `T_spd` speed / spin / ball / wind   | -0.018 +- 0.012 | -0.016 +- 0.014 | +0.004 +- 0.004 |
| `T_phys` python vs kernel            | 0.000 +- 0.001  | +0.001 +- 0.001 | -0.003 +- 0.002 |

(`E_pos + E_vel + E_curv + E_time + E_solver` sum to `icpt - truth`; `T_est` also holds the small `P0 - icpt` shift of the plane, -0.008 m balloon, so the E rows do not add to `T_est` exactly.) By range (balloon, plane z): 0-5 m -0.016 +- 0.017, 5-8 m -0.030 +- 0.062, 8-11 m -0.022 +- 0.051: flat, as a non-drop effect should be. `T_point` / `T_scat` were added after V0 and exist in the "after" runs only (balloon: `T_point` -0.024 +- 0.014, `T_scat` +0.000 +- 0.009, so the launcher scatter averages out and the 0.9 deg mean pointing error at release is where `T_dir` comes from; the sign is not stable across runs: V0's approximate split was -0.005 +- 0.015). The mean angle between the barrel at release and the solved direction is 0.89 +- 0.10 deg on a static balloon (the Ph gate admits up to `point_gate_deg` 3 deg): a hit-rate lever (8 cm at 5 m), not a bias.

## Before / after (the requested slice)

The metric fix changes `miss` only. Final tree vs V0 on 12 balloon episodes (3-5 m): identical shots and hits (12/12); referee z -0.024 (was -0.054 with the lite rule on the same shots; refined -0.024; max difference 0.0005 m). Slice sizes: balloon 80 episodes (3-5 and 5-10 m), hanging 80 episodes; hit/ball and kills of the unchanged configuration are in the V0 rows above (balloon 0.73 [0.64, 0.80], hanging 0.30 [0.22, 0.39]); the kill rate is by construction unchanged by the fix that was kept.

## Reproduce

```
NAPL_REMOTE_DIR=/workspace/wt/napl-c2bias tools/remote.sh bg --slots 1 diag \
  'OMP_NUM_THREADS=1 python -u examples/cuas2/aimdiag.py --runs 40 --kinds balloon hanging --bands 3-5 5-10 --out build/aimdiag/v0'
python examples/cuas2/aimdiag.py --report build/aimdiag/v0            # tables above (resumable: episodes.jsonl)
# t_fire experiment: add --fire-cfg '{"t_fire": 0.10}'
PYTHONPATH=python:tests pytest tests/cuas2/test_miss.py               # fails with the lite sampled rule (z < -0.02) vs refined within 2 mm
```

Seeds are those of `examples/cuas-lite/mc.py` (`draw`). Episodes are single-target; a killed target ends the episode, so most episodes have one ball.

## Not done / bench items

- Pendulum-aware lead (acceleration / arc model) and a smoothed track velocity, then re-measure `t_fire` 0.10 (it should be re-tried afterwards).
- Launch speed: B2 (`k_slip`, rpm slope) on the real launcher; optionally lead from the last measured muzzle speed (STATUS `last_v_dmps`). The sim plant is 1.5 % slower than the model at READY (1 % window, dip): `k_slip` 0.8 would read about 0.788 against the plant.
- No online post-shot bias update (no bias worth removing was found on static balloons).
- Quad / drift scenarios not run with the refined referee or `t_fire`; the lite referee (cuas-lite) still has the sampled rule (its `closest` is evaluation only).
- Aim-bias figures in `sim_results.md` ("Aim bias") are not edited here (MC-3 owns it): the -0.04 m static balloon figure is the lite rule; refined -0.02 +- 0.03 m.
