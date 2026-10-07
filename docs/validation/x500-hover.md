# X500 V2 hover and endurance: parts-built design flown by PX4 (N5b, revised by F1)

PX4 v1.17.0 SITL flies the parts-built design: `assemblies/x500v2.toml` -> `napl design build` -> `build/assemblies/x500v2.xml`
(`tests/python/test_px4_flight.py[asm]` builds it, then flies it through `px4/bridge_x500v2_asm.toml`). The hand-written
`tests/data/px4/x500_fixture.xml` is kept as `[fixture]`, an independent reference flown by the same test.
Numbers below were generated on hfbox (`pytest -m px4 -k asm`, `napl bench all`; `build/px4/x500v2_flight.json`, `build/bench/hover.json`),
pulled with `tools/remote.sh pull`. Two consecutive flights of the assembly after F1 gave the same numbers (hover throttle 0.4274 / 0.4273, per-ESC current 2.75 / 2.96 / 2.76 / 2.96 A in both). The final state hash differs between runs (PX4 is not run in lockstep with a deterministic clock),
but each run's bridge log replays to that run's own hash (asserted by the test).

## F1: why the flown motors drew 12-18 % less current than the vendor table, and what changed

Problem: at hover the flown assembly drew 2.45 / 2.64 A per motor (mean 2.54 A) against 2.99 A from the Holybro 2216 KV920 vendor table at the same
thrust (3.79 N, 16 V), while the bench thrust stand (3 training rows) was 6 % low and endurance came out 20-24 min against Holybro's ~18 min.

Cause (measured, table below): the flight rotor used the prop part's torque coefficient C_Q = 0.00526, derived from the vendor torque column, which
the vendor prints to 0.01 N m; the stand fit gives 0.00592 because it also pays for everything else in the vendor's electrical power. The two models of
the same motor therefore disagreed by 12 %, and the ESC (previously an ideal buck stage with no loss at all) was not modelled separately, so nothing
in the flight model carried those losses. Both are fixed:

1. The ESC device got three loss fields (`r_on`, `sw_loss`, `i_q`; C++ `electrical.cpp`, `devices.cpp`, schema, analytic tests), carried from the BLHeli_S 20A part
   (estimated, rationale in the part). The stand fit now includes them, so the fitted C_Q0 no longer absorbs them (0.00592 -> 0.00579).
2. The assembly's rotor C_Q is the stand's ESC-separated C_Q0 (`cq_ref = 0.005794` in `assemblies/x500v2.toml`, asserted equal to the bench by `test_bench.py`),
   not the torque-column value. It is a training-row-derived number (3 rows), not tuned to the endurance claim.

Which C_Q is physically right for the flight model: the stand-derived one, on two independent grounds. (a) Vendor energy balance: with the datasheet R(T), the datasheet I0
friction and any plausible ESC loss, C_Q 0.00526 leaves 16 % of the vendor's electrical input (47.8 W at 3.79 N) unexplained, and taking the torque column at face value leaves a positive
unexplained remainder (mean 7.9 W) in all 15 rows. (b) Prop family: the UIUC static tables of APC 9x4.5 E (C_T 0.098) and 10x4.7 SF (C_T 0.12, in the parts DB) give C_Q / C_T = 0.062 .. 0.064;
the part's value gives 0.0579 (10 % below), the stand value 0.0645. (APC props, not this exact T1045II blade: a cross-check, not a measurement.)

### Hypotheses, measured

Per-motor hover bus current at 3.793 N (1.5466 kg / 4), one motor, closed form of the device equations (`f1` scratch script reusing `napl.bench` Reynolds shape; same
equations as the C++, agrees with the simulated bench hover to < 1 %). Baseline = the flight model before F1 (C_Q 0.00526, ideal ESC, ESC input 16.02 V, winding 28 degC): 2.498 A (flown: 2.54 A).

| # | hypothesis | verdict | effect on hover current (one at a time vs baseline) |
|---|---|---|---|
| 1 | flight C_Q (vendor torque column) too low vs the stand fit | confirmed, dominant. Stand C_Q refit with the ESC modelled separately = 0.005794 (was 0.00592); adopted for the flight model | C_Q 0.00526 -> 0.005794: +0.247 A (+9.9 %); at the stand's old 0.00592 (C_T0 0.0898) +0.349 A (+14 %), i.e. modelling the ESC separately took 0.10 A back out of the C_Q term |
| 2 | ESC losses (conduction, switching, quiescent) missing | confirmed missing (no ESC loss existed); added, but small. BLHeli_S is complementary "damped light" PWM with active freewheeling (manual), so no body-diode freewheel loss; remaining: r_on 12 mOhm (estimated), sw_loss 0.0015 (24 kHz x 30 ns edges + 300 ns dead time x 0.8 V, estimated), i_q 15 mA (estimated) | r_on +0.028 A, sw_loss +0.009 A, i_q +0.015 A: total +0.052 A (+2.1 %); 0.9 W of 45 W per motor. The MOSFET is not identifiable (no teardown / datasheet found for the Holybro part), so these stay estimated |
| 3 | throttle -> duty map | no effect on current at fixed thrust (current is set by the torque + loss balance; the map only chooses which duty reaches it). Consistency check: vendor-table power balance gives d_eff = 0.427 at 3.79 N (R(T) = 1.2 R20); PX4 flies 0.4274 (bench governor 0.4185; the stand's fitted map `0.772 thr + 0.085` at the table's 45.9 % gives 0.439). The vendor rig's own map reaches only d_eff 0.84 at 100 % throttle (property of the vendor rig, kept in the bench, not in the ESC device) | 0.000 A |
| 4 | wiring / connector R, battery R0 / RC under 10 A | already modelled: pack to ESC drop 0.38 V at 11.6 A (r0 0.020 + RC + XT60 / AWG10 / AWG12 / AWG16 leads) *raises* hover current vs a lossless 16.41 V source. Not modelled: contact resistances (assumed ~1 mOhm per XT contact, so <= +0.3 %; +0.03 Ohm would be +2.8 % but nothing supports it) | pack at 16.41 V instead of 16.02 V: -0.059 A (-2.4 %) |
| 5 | battery voltage / comparison at like voltage | like voltage: the flight ESC input is 16.03 V at hover (PX4-reported 16.027 V, pack 16.41 V idle) against the table's 16 V; Chen OCV 16.41 V full is the unloaded value | 16.02 -> 16.00 V: +0.003 A (+0.1 %) |
| - | not in the list: I0 +25 % / winding 60 degC (for scale) | datasheet I0 and R are fixed | +0.091 A (+3.7 %) / +0.034 A (+1.3 %) |

Waterfall, flight model (closed form): 2.498 A -> + C_Q 0.005794: 2.745 A -> + r_on: 2.778 A -> + sw_loss: 2.788 A -> + i_q: 2.803 A. Flown (PX4): 2.858 A mean.
Power at the vendor's hover point: model 44.9 W (prop shaft 34.1, copper 5.2, I0 friction 4.7, ESC 0.9) against the vendor's 47.97 W (16 V x 2.998 A): 3.1 W (6 %) remains unexplained.

### Before / after

| quantity | before (N5b) | after (F1) | vendor table / claim |
|---|---|---|---|
| per-motor bus current, PX4 flight (m1 / m2 / m3 / m4) | 2.45 / 2.64 / 2.45 / 2.63 A, mean 2.54 A | 2.75 / 2.96 / 2.76 / 2.96 A, mean 2.86 A | 2.998 A at 3.793 N, 16 V |
| flight mean vs table | -15 % (range -18 .. -12 %) | -4.7 % (range -8.2 .. -1.1 %) | |
| bench hover per-motor current | 2.82 A (-6 %) | 2.82 A (-5.8 %), unchanged (the stand fit already carried the loss) | |
| hover duty (PX4 controls mean) | 0.417 (0.411 / 0.423 / 0.411 / 0.423) | 0.427 (0.421 / 0.433 / 0.422 / 0.433); bench governor 0.415 -> 0.419 | table power balance 0.427 |
| pack current, flight | 10.31 A | 11.58 A | 4 x 2.998 + 0.147 = 12.14 A |
| endurance, flight-slope, to 20 % SoC / to 0 % | 24.0 / 30.0 min | 21.4 / 26.7 min | Holybro ~18 min (5000 mAh) |
| endurance, bench (Coulomb count, voltage-tracking governor), 80 % DoD / 3.5 V per cell loaded cutoff | 20.4 / 23.6 min | 20.4 / 23.7 min | |
| stand fit C_Q0 / r_th (2216) | 0.00592 / 0.927 K/W | 0.00579 / 0.962 K/W | |
| 2216 held-out MAPE thrust / current / rpm, stand | 5.4 / 1.9 / 2.2 % | 5.3 / 2.0 / 2.1 % | limit (7, 3): unchanged, not loosened |

Endurance against Holybro: the vendor table itself, held at its 2.998 A per motor (pack 12.14 A), gives 25.7 min to 0 % and 20.5 min to a 20 % reserve for the 5.2 Ah pack (24.7 / 19.8 min for 5.0 Ah),
so the table and the ~18 min claim are 10-40 % apart before any model is involved. The flight model is now 4.7 % under the table in current and 4 % above it in flight time (21.4 vs 20.5 min to 20 %).
Holybro gives no AUW, battery chemistry / C rating, cutoff, reserve or flight profile (exact wording below), so the remaining difference to 18 min is not attributable. The claim was not used as a fit target.

## What the design carries (builder output, `python/napl/design/devices.py`)

- Rotors: pooled UIUC Reynolds factor tables `re_x / re_ft / re_fq` (the bench's shape, shared code in `napl.design.reynolds`) via the binding
  flag `reynolds = true`. The prop part's `ct / cq` (means over its own static table, whose rows sit at higher `x`) are re-referenced to `x = 4.5 m^2/s`
  by the least-squares fit of `C_ref f(x)` to that table: C_T 0.0950 -> 0.0908. C_Q: the binding field `cq_ref` (new, F1) then scales the cq list to the stand's
  ESC-separated C_Q0 = 0.005794 (the part's own 0.00530 -> 0.00526 re-reference is superseded; the part is unchanged).
- ESCs: `r_on` 0.012 ohm, `sw_loss` 0.0015, `i_q` 0.015 A from `holybro-x500v2-esc-blheli-s-20a` (estimated; device map `esc2216`), `i_limit` 20 A.
- Motors: `r_th` = 0.962 K/W (the bench's calibration against the vendor 80 degC, now with the ESC loss in the fit; `test_bench.py` asserts the assembly value equals the bench's), `c_th` 25.6 J/K
  (estimated, unchanged), kv / R / I0 / rotor inertia from the part.
- Battery: `lipo-cell-ocv-chen2006` x 4 cells (full 16.41 V), r0 / r1 / c1 from the Tattu part; discharge chain wires; three constant-current avionics loads
  (0.1083 + 0.0124 + 0.0258 = 0.1465 A from the battery); GNSS 5 Hz with 0.2 s latency (one sample).
- Mass: 1.547 kg (build report `mass.total`, avionics included).

## Comparison (after F1)

Hover thrust per motor = 1.5466 kg x 9.81 / 4 = 3.79 N. Vendor table = `holybro-2216-kv920 [tables.thrust_test]` at 16 V, linearly interpolated at that thrust.

| quantity | PX4 flies the assembly | bench hover (2216 stand fit, assembly AUW / battery / loads) | 2216 vendor table | Holybro claim |
|---|---|---|---|---|
| hover throttle (ESC duty; PX4 controls[i]) | 0.427 (per motor 0.421 / 0.433 / 0.422 / 0.433) | 0.419 (= vendor throttle 43 % through the fitted duty map) | 46 % | - |
| per-motor current | 2.75 / 2.96 / 2.76 / 2.96 A (mean 2.86 A; -4.7 % vs table) | 2.82 A (-5.8 %) | 3.00 A | - |
| rpm | not logged | 5403 | 5390 | - |
| pack current | 11.58 A | 11.44 A | 4 x 3.00 + 0.15 = 12.1 A | - |
| pack voltage in hover | 16.03 V (PX4-reported 16.03 V) | 16.12 V at start | 16 V (stand) | - |
| endurance to 20 % SoC reserve | 21.4 min (Coulomb count, SoC slope x 5.2 Ah, constant current) | 20.4 min | - | |
| endurance to empty / cutoff | 26.7 min to 0 % SoC (same slope) | 23.7 min to 3.5 V/cell under load (4.85 Ah used) | - | ~18 min |
| hold altitude (target 5 m, window 4.7..5.7 m abs; rest 0.217 m) | 5.05 .. 5.29 m abs, rms 5.24 | - | - | - |
| replay | final tick 67328, hash 45468de71d7e0e26 == napl-replay | | | |

Holybro, exact wording (docs.holybro.com/drone-development-kit/px4-development-kit-x500v2, "Mechanical Specification", fetched 2026-10-07):
"Flight time: ~18 minutes hover with no additional payload. Tested with a 5000mAh Battery." The page gives no AUW, battery chemistry / C rating,
cutoff voltage or reserve, ambient or flight profile, and no log, so the claim cannot be reproduced, only bracketed. Same section: "Max payload: ~1kg (w/ 4S 5000 mAh battery @70% Throttle)".

## Reading the results (confirmed vs inferred)

Confirmed (measured above):
- The parts-built drone flies the full mission under real PX4: arm, takeoff to 5 m, 30 s hold inside the window, land, disarm, replay hash equal to `napl-replay`.
  Hover throttle 0.427 agrees with the vendor-table power balance (0.427) and with the bench governor (0.419); the hand-written fixture (own motor / prop numbers, ideal ESC) hovers at 0.412.
- Per-motor current is -4.7 % vs the vendor table in flight (was -15 %), -5.8 % in the bench. The stand reproduces the table to ~2 % on held-out rows (current).
- C_Q of the flight rotor and the stand now agree by construction; the previous 12 % inconsistency (0.00526 vs 0.00592) was the largest single cause (+0.25 A of the 0.49 A gap); the ESC loss is real but small (+0.05 A).
- Rear motors (m2, m4) draw 2.96 A against 2.75 A at the front (52 / 48 split of the current).
- Held-out stand limits (`LIMITS` in `tests/python/test_bench.py`) are unchanged: 2216 thrust 5.3 % (<= 7), current 2.0 % (<= 3). The 2216 current MAPE moved 1.9 -> 2.0 %.

Inferred (arithmetic matches, not separately tested):
- The remaining 4.7-6 % current gap (3.1 W at the hover point) is unmodelled loss in the vendor's rig or motor, candidates: PWM-ripple copper and iron loss (needs the winding inductance, not published),
  commutation-timing advance (BLHeli_S timing setting costs efficiency, manual), R tolerance (115 +- 10 mOhm is +-0.5 W at hover), rounding of the vendor current / torque columns, a different (lossier) vendor ESC.
  None is tested; the stand's I0 split scan (thrust-stands.md) rejects a speed-dependent I0 split.
- The flown mean (2.86 A) sits 2 % above the single-motor closed form (2.80 A): the front / rear thrust split and attitude-control thrust modulation, because current is convex in thrust.
- The front/rear split is consistent with the assembly's centre of mass sitting aft of the plate centre (CoM not read out of the model here).
- The flight-slope endurance extrapolates the early-hover current at constant slope, while the bench (voltage-tracking governor) needs duty 0.419 -> 0.45 and
  more current as the pack sags (SoC 1.0 -> 0.52 in 12.5 min), so the flight-slope figures are optimistic against the bench ones.

Not changed, recommended: BLHeli_S is complementary (damped-light) PWM per its manual; the assembly leaves `complementary` at the schema default (false, diode freewheel, no regen braking). Setting it
true changes spin-down dynamics and was out of scope (it needs a flight re-validation).

## Deviations from the fixture / defaults, why

- GNSS 5 Hz (`max_nav_rate_gps_glonass`, datasheet; the fixture's 10 Hz is the GPS-only rate); `gps_rate_hz = 5` in the bridge config. Home = the assembly's GNSS origin
  (47.3769 N, 8.5417 E, 408 m), not the fixture's PX4 SITL default.
- `px4/params/napl_sitl.params`: `EKF2_HGT_REF 0` (baro) and `EKF2_GPS_CTRL 5` (no GNSS vertical position). With the v1.17.0 defaults (GPS height reference) the 3.2 m vertical noise
  of the datasheet-accuracy M8N model made the assembly's hold miss the window in every run (absolute altitude seen 4.3-5.1 m, 6.0-6.5 m, 4.5-5.0 m against 4.7-5.7 m; the fixture, which has the same GNSS noise at 10 Hz,
  passed). Cause not isolated beyond that: GNSS vertical noise (sigma 3.2 m) against the EKF's default 0.5 m GNSS height noise is the likely driver (inferred). The change applies to both models.
- The ESC device has no throttle-to-duty map (the stand fit's `duty = 0.772 thr + 0.085` is a property of the vendor rig, kept in `build/bench`), so PX4's throttle is the duty.
- The ESC loss numbers are estimates (part rationale); the vendor test's ESC is not named, the kit's BLHeli_S 20A is the stand-in.
- Not validated here: a real flight log (none exists), the CoM position (CAD-density mass model), thrust at J > 0 (borrowed APC-SF shape), battery r0 / r1 (estimated), the 3.1 W remainder above.
