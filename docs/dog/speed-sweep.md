# Dog speed sweep (SP3b)

Generated 2026-10-17 by `python tools/dog_sweep.py --out build/dog-sweep --jobs 4` on the full dog model (assemblies/dog/dog.toml, 12 Qdd, flat floor).

## How to read

Every column is labelled by how it was obtained:

- **measured**: on the full model in this run (stall, swing, push, jump, drop, hold; see `tools/dog_sweep.py`).
- **derived (study formulas, sim-measured inputs)**: `tools/dog_actuator_study.py` formulas fed with measured t_move, stall torque and pack voltage: `w0`, `pack_margin`, `v_swing`, `v_stance`, `v_trot_bound`. These are bounds of a trot, NOT a gait speed.
- **bom**: from the build report (`design.build` BOM / mass).
- **not measured (SP4)**: `v_top_trot`, `cot` need a gait controller (`--gait module:function`); they stay empty here.

GIM6010-8 points are a probe: GIM6010-8 electrics and mass in the GIM8108-8 seats (there are no GIM6010 printed parts). Knee ratio only sets the Qdd `gear_ratio` = 8 x ratio; the crank / lug geometry stays 1.5, so ratio 2.0 is an electrical what-if.

## Points

| point               | mass kg | cost USD | stall hip/knee N m | t_move s | push F/mg | jump rise m | I peak A | margin | drop tau N m | drop fault | hold dT K | hold W | v_swing | v_stance | v_trot_bound m/s |
| ------------------- | ------- | -------- | ------------------ | -------- | --------- | ----------- | -------- | ------ | ------------ | ---------- | --------- | ------ | ------- | -------- | ---------------- |
| gim6010-8 r1.5 L200 | 11.531  | 4568     | 11.0 / 19.2        | 0.051    | 6.29      | 0.323       | 144.0    | 0.97   | 19.2         | 0          | 4.3       | 90     | 2.54    | 0.00     | 0.00             |
| gim6010-8 r1.5 L220 | 11.557  | 4570     | 11.0 / 19.2        | 0.055    | 6.06      | 0.352       | 144.3    | 0.97   | 19.2         | 0          | 4.4       | 95     | 2.59    | 0.00     | 0.00             |
| gim6010-8 r1.5 L250 | 11.597  | 4572     | 11.0 / 19.2        | 0.063    | 5.71      | 0.390       | 143.0    | 0.98   | 19.2         | 0          | 4.6       | 104    | 2.57    | 0.00     | 0.00             |
| gim6010-8 r2.0 L200 | 11.531  | 4568     | 11.0 / 25.5        | 0.051    | 6.56      | 0.275       | 141.7    | 0.99   | 20.4         | 0          | 4.3       | 81     | 2.54    | 0.00     | 0.00             |
| gim6010-8 r2.0 L220 | 11.557  | 4570     | 11.0 / 25.5        | 0.055    | 6.05      | 0.306       | 141.2    | 0.99   | 21.6         | 0          | 4.4       | 84     | 2.59    | 0.00     | 0.00             |
| gim6010-8 r2.0 L250 | 11.597  | 4572     | 11.0 / 25.5        | 0.063    | 5.43      | 0.351       | 139.9    | 1.00   | 23.4         | 0          | 4.6       | 90     | 2.57    | 0.00     | 0.00             |
| gim8108-8 r1.5 L200 | 11.627  | 5552     | 22.0 / 44.0        | 0.038    | 9.33      | 0.174       | 141.2    | 0.99   | 20.3         | 0          | 1.3       | 55     | 3.41    | 1.36     | 1.36             |
| gim8108-8 r1.5 L220 | 11.653  | 5554     | 22.0 / 44.0        | 0.041    | 8.88      | 0.195       | 149.9    | 0.93   | 21.2         | 0          | 1.4       | 57     | 3.47    | 0.92     | 0.92             |
| gim8108-8 r1.5 L250 | 11.693  | 5556     | 22.0 / 44.0        | 0.045    | 8.20      | 0.227       | 147.6    | 0.95   | 23.0         | 0          | 1.4       | 60     | 3.60    | 0.05     | 0.05             |
| gim8108-8 r2.0 L200 | 11.627  | 5552     | 22.0 / 44.0        | 0.038    | 8.81      | 0.127       | 137.5    | 1.02   | 18.9         | 0          | 1.3       | 52     | 3.41    | 1.36     | 1.36             |
| gim8108-8 r2.0 L220 | 11.653  | 5554     | 22.0 / 44.0        | 0.041    | 8.85      | 0.143       | 146.1    | 0.96   | 20.0         | 0          | 1.4       | 53     | 3.47    | 0.92     | 0.92             |
| gim8108-8 r2.0 L250 | 11.693  | 5556     | 22.0 / 44.0        | 0.045    | 8.50      | 0.167       | 139.0    | 1.01   | 21.7         | 0          | 1.4       | 55     | 3.60    | 0.05     | 0.05             |

Eligible: pack_margin >= 1.0, no drop fault, push F/mg >= 2.0.

## Pareto front (v_trot_bound max, cost min, mass min)

| point               | v_trot_bound m/s | cost USD | mass kg |
| ------------------- | ---------------- | -------- | ------- |
| gim6010-8 r2.0 L250 | 0.00             | 4572     | 11.597  |
| gim8108-8 r2.0 L200 | 1.36             | 5552     | 11.627  |

## Chosen point

**gim8108-8 r2.0 L200**: v_trot_bound 1.36 m/s (derived, not a measured gait), cost 5552 USD, mass 11.627 kg.

Gap to the 3.5 m/s criterion (spec s7): 2.14 m/s short; to the 4.5 k USD target: 1052 USD over. The real top speed needs SP4's gait (flying trot / bound); this table claims none.

## Reading of the result (hand-written, not generated)

- The rule's pick (GIM8108-8, ratio 2.0, L = 0.20) beats the shipped candidate 1 (GIM8108-8, 1.5, 0.22) by 0.44 m/s of a derived bound. The bound is the hip torque limit of the study formulas (`v_stance` = 0 when the stance hip torque exceeds the 22 N m stall): a shorter leg lowers `h sin(THETA)`. `v_swing` (3.4-3.6 m/s) is not the limit.
- The eligibility cut that separates ratio 2.0 from 1.5 at L200 is `pack_margin` 1.018 vs 0.991 against a 140 A (2 x 70 A) threshold: 3 % of a worst-case current (all 12 joints saturating at once, 1 s) that no gait will draw. That is inside the noise of the measurement, so the ratio 2.0 preference is not a robust result.
- Ratio 2.0 needs a 40 mm shank lug (or a 15 mm crank), which is not drawn: the sweep varies only the Qdd `gear_ratio`. The shipped dog therefore keeps candidate 1; the L = 0.20 variant parts exist (`-200`) and would be the cheap follow-up.
- Every 8108 point exceeds 4.5 k USD by about 1 k USD (the 12 actuators are 2.6 k USD). The GIM6010-8 probe meets the 4.5 k USD order of magnitude (4.57 k USD) but its 11 N m hip cannot hold the stance torque of the study's trot (`v_stance` = 0).
- Pack current: 137-150 A peak in a 1 s all-joints-saturating jump, against 2 x 60 A on the BMS (the README's PDB / BMS margins are for the study's 87 A trot peak).
