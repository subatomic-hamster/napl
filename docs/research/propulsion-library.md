# Multirotor propulsion library (stage P1a)

Date: 2026-10-07. Scope: parts/propulsion (53 files). All values carry a source and fidelity tag per parts/README.md. Checker result at completion: `270 parts, 4023 values (datasheet=3323, derived=429, estimated=98, literature=169, measured=4), 0 problem(s)` (total includes other agents' parts).

## Counts for these files (53 files)

By category: 18 motors (motor.bldc), 14 props (8 `prop`, 6 `prop.reference`), 8 ESCs (esc), 13 batteries (10 battery.lipo, 3 battery.liion).

Fidelity of tagged values in params and ports: datasheet 721, derived 124, literature 54, estimated 24, measured 0. Tables: 16 datasheet (vendor thrust tests), 12 literature (UIUC wind-tunnel data).

Motors with a vendor thrust table (`[tables.thrust_test]`, thrust_N = thrust_g x 0.00980665):

| Motor | Prop (as printed) | Voltage | Rows |
|---|---|---|---|
| tmotor-mn4006-kv380 | T-MOTOR 15*5 CF | 24 V | 7 |
| tmotor-mn4004-kv300 | T-MOTOR 15*5 CF | 24 V | 7 |
| tmotor-mn5006-kv300 | T-MOTOR P17*5.8 CF | ~23 V (per row) | 11 |
| tmotor-mn5008-kv340 | T-MOTOR P18*6.1 CF | ~23 V (per row) | 11 |
| tmotor-mn4110-kv340 | T-MOTOR P18*6.1 | ~23 V (per row) | 7 |
| tmotor-mn3508-kv700 | T-MOTOR 12*4CF | 14.8 V | 5 |
| tmotor-mn2806-kv400 | T-MOTOR P12*4 | ~23 V (per row) | 7 |
| tmotor-air2213-kv1000 | MF9545 (tmotor-mf9545) | 16 V | 8 |
| tmotor-2807-kv1250 | T-MOTOR C7.5*4.6*3 | ~22 V (per row) | 10 |
| tmotor-2814-kv900 | T-MOTOR C7.5*4.6*3 | ~29 V (per row) | 10 |
| tmotor-3110-kv1580 | T-MOTOR C7.5*4.6*3 | ~21 V (per row) | 10 |
| tmotor-4214-kv380 | T-MOTOR C15*8*3 | ~29 V (per row) | 10 |
| iflight-xing2-2207-1855kv | 51466; second table "5140" | 24 V | 6 + 6 |
| iflight-xing2-2207-2755kv | 51466; second table "5140" | 24 V | 6 + 6 |

Motors without a thrust table: emax-eco2-2807-1300kv, sunnysky-x2212-kv980, iflight-xing2-1806-2500kv, iflight-xing2-1404 (vendor publishes only specs).

## Sources used

- T-MOTOR store pages (spec text, thrust test tables in page text): MN2806, MN3508, MN4004, MN4006, MN4110, MN5006, MN5008, NS14x4.8 to NS18x6 props, AIR GEAR 355 combo (AIR 20A ESC), ALPHA 40A 6S and 60A 6S ESCs. Mount patterns and bores read from the technical-drawing images on store.tmotor.com (cited by image URL).
- T-MOTOR UAV site (uav-en.tmotor.com/2026/Motor_0126): 2807, 2814, 3110, 4214 specs plus thrust tables.
- iFlight spec images hosted by rcdrone.top (XING2 2207 1855KV and 2755KV thrust tests, read visually and cross-checked with P = V x I and g/W); rcdrone.top / Thingbits listings for XING2 1404 and 1806.
- EMAX ECO II 2807 (ReadyMadeRC, Wrekd), SunnySky X2212 (ReadyMadeRC), Gemfan Hurricane 51466 (GetFPV, MyFPVStore), HQProp 5x4.3x3 V2S (PyroDrone).
- UIUC Propeller Database Volume 1 (m-selig.ae.illinois.edu/props/volume-1): APC thin-electric 9x4.5, 11x7, 14x12, 17x12 and slow-flyer 8x3.8, 9x4.7; one static file and one advancing-ratio run each.
- ESCs: ReadyMadeRC (Holybro Tekko32 F4 4in1 50A AM32), rcdrone.top iFlight listings (BLITZ E55, E55S, E55R 4-in-1; E80 single).
- Batteries: rcdrone.top (authorized Tattu distributor) spec tables for ten Tattu LiPo packs; iFlight Fullsend Li-ion pack listings (rcdrone.top); cell datasheets for Samsung INR21700-40T (piekarz.pl hosted copy of the Samsung SDI spec) and EVE INR21700-50PL (eurolamp.ua hosted copy).

## Gaps

- No vendor thrust table found for the EMAX ECO II, SunnySky X2212, XING2 1404 and XING2 1806 motors, nor for any 3S/4S micro-class motor; micro and 5-inch coverage is therefore thin on validation data (only XING2 2207). MQTB (miniquadtestbench) results sit behind links and were not used.
- ESC firmware and PWM-rate details are missing for the T-MOTOR AIR 20A, ALPHA 40A and ALPHA 60A pages (throttle ranges only). ALPHA product names (40A, 60A) are the peak ratings; continuous ratings in the spec tables are 20 A and 40 A.
- ESC board outline/thickness unpublished for E55S and E55R (box geometry estimated), E55 height estimated; pad positions on every ESC and battery are unpublished (ports carry no pos).
- 20x20 stack thread (M2 vs M3) is not stated for the E55R; M3 chosen as an inference.
- Battery internal resistance: none published for any Tattu pack. All ten LiPo values are `estimated` with a shared rationale, R_cell = 0.026 ohm*Ah / capacity (anchored to parts/drone/tattu-4s-5200mah-35c.toml); high-C racing packs probably run lower. The Tattu 1050 mAh 3S listing carries a chart caption of per-cell mOhm figures (not used: pack and units unclear). The three Li-ion packs use cell-datasheet AC 1 kHz impedance ceilings (12 mOhm Samsung 40T, 7 mOhm EVE 50PL) multiplied out to pack level, tagged `estimated`.
- Battery connector gender is modelled as neutral; real pack-side XT60/XT30 polarity is not captured.
- UIUC tables are transcriptions of one static run and one advancing run per prop; full J sweeps at several rpm are not included. No UIUC data exists for the T-MOTOR NS carbon props, so tmotor-ns* and hqprop/gemfan props carry no C_T(J)/C_P(J).
- Prop inertia (j_prop) in the T-MOTOR NS, Gemfan 51466 and HQProp files is a rod-model upper bound (m L^2 / 12), not a measured value.
