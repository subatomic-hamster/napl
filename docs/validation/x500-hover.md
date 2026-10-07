# X500 V2 hover and endurance: parts-built design flown by PX4 (N5b)

PX4 v1.17.0 SITL now flies the parts-built design: `assemblies/x500v2.toml` -> `napl design build` -> `build/assemblies/x500v2.xml`
(`tests/python/test_px4_flight.py[asm]` builds it, then flies it through `px4/bridge_x500v2_asm.toml`). The hand-written
`tests/data/px4/x500_fixture.xml` is kept as `[fixture]`, an independent reference flown by the same test.
Numbers below were generated on hfbox (`pytest -m px4`, `napl bench hover`; `build/px4/x500v2_flight.json`, `build/bench/hover.json`),
pulled with `tools/remote.sh pull`. Two consecutive flights of the assembly gave the same numbers (hover throttle 0.4171, per-ESC current identical to 2 decimals);
the final state hash differs between runs (PX4 is not run in lockstep with a deterministic clock), but each run's bridge log replays to that run's own hash (asserted by the test).

## What the design carries (builder output, `python/napl/design/devices.py`)

- Rotors: pooled UIUC Reynolds factor tables `re_x / re_ft / re_fq` (the bench's shape, now shared code in `napl.design.reynolds`) via the binding
  flag `reynolds = true`. The prop part's `ct / cq` (means over its own static table, whose rows sit at higher `x`) are re-referenced to `x = 4.5 m^2/s`
  by the least-squares fit of `C_ref f(x)` to that table: C_T 0.0950 -> 0.0908, C_Q 0.00530 -> 0.00526 (the thrust-stand fit of the same table gives
  C_T0 0.0898, C_Q0 0.00592).
- Motors: `r_th` = 0.927 K/W (the bench's calibration against the vendor 80 degC; `test_bench.py` asserts the assembly value equals the bench's), `c_th` 25.6 J/K
  (estimated, unchanged), kv / R / I0 / rotor inertia from the part.
- Battery: `lipo-cell-ocv-chen2006` x 4 cells (full 16.41 V), r0 / r1 / c1 from the Tattu part; discharge chain wires; three constant-current avionics loads
  (0.1083 + 0.0124 + 0.0258 = 0.1465 A from the battery); GNSS 5 Hz with 0.2 s latency (one sample).
- Mass: 1.547 kg (build report `mass.total`, avionics included).

## Comparison

Hover thrust per motor = 1.5466 kg x 9.81 / 4 = 3.79 N. Vendor table = `holybro-2216-kv920 [tables.thrust_test]` at 16 V, linearly interpolated at that thrust.

| quantity | PX4 flies the assembly | bench hover (2216 stand fit, assembly AUW / battery / loads) | 2216 vendor table | Holybro claim |
|---|---|---|---|---|
| hover throttle (ESC duty; PX4 controls[i]) | 0.417 (per motor 0.411 / 0.423 / 0.411 / 0.423) | 0.415 (= vendor throttle 43 % through the fitted duty map) | 46 % | - |
| per-motor current | 2.45 / 2.64 / 2.45 / 2.63 A (mean 2.54 A; -15 % vs table) | 2.82 A (-6 %) | 2.99 A | - |
| rpm | not logged | 5405 | 5390 | - |
| pack current | 10.31 A | 11.44 A | 4 x 2.99 + 0.15 = 12.1 A | - |
| pack voltage in hover | 16.07 V (PX4-reported 16.07 V) | 16.12 V at start | 16 V (stand) | - |
| endurance to 20 % SoC reserve | 24.0 min (Coulomb count, SoC slope x 5.2 Ah, constant current) | 20.4 min | - | |
| endurance to empty / cutoff | 30.0 min to 0 % SoC (same slope) | 23.6 min to 3.5 V/cell under load (4.85 Ah used) | - | ~18 min |
| hold altitude (target 5 m, window 4.5..5.5 m above rest) | 5.07 .. 5.28 m abs (rest 0.217 m): 4.86 .. 5.06 m above rest, rms 5.23 | - | - | - |

Holybro, exact wording (docs.holybro.com/drone-development-kit/px4-development-kit-x500v2, "Mechanical Specification", fetched 2026-10-07):
"Flight time: ~18 minutes hover with no additional payload. Tested with a 5000mAh Battery." The page gives no AUW, battery chemistry / C rating,
cutoff voltage or reserve, ambient or flight profile, and no log, so the claim cannot be reproduced, only bracketed. Same section: "Max payload: ~1kg (w/ 4S 5000 mAh battery @70% Throttle)".

## Reading the results (confirmed vs inferred)

Confirmed (measured above):
- The parts-built drone flies the full mission under real PX4: arm, takeoff to 5 m, 30 s hold inside +-0.5 m (the window is 4.5..5.5 m above rest; the flight stays
  within 0.15 m of 5 m), land, disarm, replay hash equal to `napl-replay`. Hover throttle (0.417) agrees with the bench governor (0.415) and with the fixture (0.412).
- Per-motor current is inside the +-20 % gate of the vendor table in flight (-12 % .. -18 %) and -6 % in the bench. The stand reproduces the table to ~2 % on held-out rows,
  so the gap is in the hover-side numbers, not in the table.
- Endurance: 20.4 min to 80 % depth of discharge from the bench (+13 % vs 18 min), 23.6 min to the 3.5 V/cell loaded cutoff; the flight-slope figures (24.0 min to 20 % reserve, 30.0 min
  to 0 %) are longer (+33 % / +67 %). All are longer than Holybro's ~18 min. The previous parts-DB bench said 8.9 min / 17.2 min (generic `lipo-cell-ocv`, which is low:
  3.5 V/cell at 50 % SoC); with the measured Chen 2006 curve (3.80 V at 50 %) the voltage cutoff no longer trips early.
- Rear motors (m2, m4) draw 2.64 A against 2.45 A at the front (52 / 48 split of the current).

Inferred (arithmetic matches, not separately tested):
- The front/rear split is consistent with the assembly's centre of mass sitting aft of the plate centre (CoM not read out of the model here).
- The flight/bench current gap (2.54 vs 2.82 A, -10 %) matches the C_Q ratio 0.00526 / 0.00592 = 0.89: the prop part's torque coefficient is derived from the vendor torque column
  (printed to 0.01 N m), the stand fit's C_Q0 also absorbs ESC and wiring loss that the vendor current includes. Using the part value, the assembly's motors draw less current than the vendor table.
- The flight-slope endurance is optimistic: it extrapolates the early-hover current at constant slope, while the bench (voltage-tracking governor) needs duty 0.418 -> 0.45 and
  more current as the pack sags (SoC 1.0 -> 0.52 in 12.5 min). The vendor figure may include a conservative reserve and a lower-quality battery; none of that is stated.

## Deviations from the fixture / defaults, why

- GNSS 5 Hz (`max_nav_rate_gps_glonass`, datasheet; the fixture's 10 Hz is the GPS-only rate); `gps_rate_hz = 5` in the bridge config. Home = the assembly's GNSS origin
  (47.3769 N, 8.5417 E, 408 m), not the fixture's PX4 SITL default.
- `px4/params/napl_sitl.params`: `EKF2_HGT_REF 0` (baro) and `EKF2_GPS_CTRL 5` (no GNSS vertical position). With the v1.17.0 defaults (GPS height reference) the 3.2 m vertical noise
  of the datasheet-accuracy M8N model made the assembly's hold miss the window in every run (absolute altitude seen 4.3-5.1 m, 6.0-6.5 m, 4.5-5.0 m against 4.7-5.7 m; the fixture, which has the same GNSS noise at 10 Hz,
  passed). Cause not isolated beyond that: GNSS vertical noise (sigma 3.2 m) against the EKF's default 0.5 m GNSS height noise is the likely driver (inferred). The change applies to both models.
- The ESC device has no throttle-to-duty map (the stand fit's `duty = 0.757 thr + 0.087` is a property of the vendor rig, kept in `build/bench`), so PX4's throttle is the duty.
- `r_th` 0.927 K/W replaces the earlier hand figure 1.8 K/W (cold 16.4 A / 0.115 ohm heat balance; ignores R(T) and the fitted current).
- Not validated here: a real flight log (none exists), the CoM position (CAD-density mass model), thrust at J > 0 (borrowed APC-SF shape), battery r0 / r1 (estimated).
