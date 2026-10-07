# Electronics, connectors and structure parts library (stage P1c)

Date: 2026-10-07. Scope: parts/electronics (34), parts/connectors (16), parts/structure (45), plus six new materials in parts/common. All values carry a source and fidelity tag per parts/README.md. Checker result at completion: `262 parts, 3865 values, 0 problem(s)` (total includes other agents' parts).

## Counts for these files (101 files, 894 tagged values, 210 ports)

Fidelity: datasheet 815, literature 42, derived 33, measured 4, estimated 0.

Structure breakdown: 6 carbon tubes, 4 carbon plates, 2 aluminium extrusions, 4 bearings, 7 standoffs, 13 ISO 4762 screws, 4 ISO 7380-1 screws, 5 ISO 4032 nuts.

## Sources used

- Holybro docs (.md pages, curl-able): Pixhawk 6X, 6C Mini, Kakute H7 V2, Kakute H743-Wing, M9N/M10 GPS, PM02D/PM03D/PM06D/PM06 V2/PM07/PM08-CAN.
- Matek pages (`?portfolio=<slug>`): H743-SLIM V4, H743-WLITE, M10Q-5883, M9N-5883, 3901-L0X, BEC12S-PRO, PM12S-3, ELRS R24-D/S.
- ArduPilot docs for CUAV V5+ (CUAV docs unreachable, PX4 page removed).
- SpeedyBee F405 V4, Pololu D24V22F5/D24V50F5 product pages, Benewake TFmini-S/TF-Luna, Raspberry Pi datasheets, NVIDIA Jetson Orin Nano module sheet, Bosch BMI270/BMP388, TDK ICM-20948 (SparkFun mirror).
- Connectors: Amass XT30/XT90S/MR30 (vendor pages), EC5 via Electrokit, JST GH/PH/XH/SH series PDFs, Wurth WR-PHD headers, Wikipedia AWG table.
- Structure: Easy Composites (tubes, sheet), haluminium 20-series datasheet (Fab Academy mirror), NSK rolling-bearing catalog e1102m, element14.cn standoff pages, fasten.it ISO 4762 / 7380-1 / 4032 pages.
- Materials: Prusament PETG TDS, Ultimaker ABS v5.00 and TPU 95A V2.01 TDS, Xometry PA12 CF, ASM 7075 data sheet, Proto Labs 304 sheet.

## Conventions

- Port families: `bolt-pattern.<a>x<b>.<thread>` (boards), `thread.<m>` (fasteners/standoffs, `length` extra key), `tube.<od>`, `bore.<d>` + `housing.<od>` (bearings), connector families `xt30 xt90 ec5 mr30 jst-gh.6 jst-ph.2 jst-xh.3 jst-sh.8 servo-3pin wire-lead`.
- Revolute-symmetric parts use `symmetry = 360` as a discrete stand-in for continuous.
- Fastener ports: pos at underside of head, axis points out of the shank tip. Standoff male stud length is derived (overall minus body).
- FC power-in ports are male headers; power-module cable-side outputs are female. Battery-side gender is a modelling choice.
- Carbon plate `density_effective` is derived from areal mass over actual thickness; the vendor tolerance is +/-0.2 mm, so the 2 mm value (1647 kg/m3) is high.
- ISO 4032 nut masses are derived (hex prism minus bore, chamfers ignored) because fasten.it has no nut mass table.

## Gaps and lower-confidence items

Missing parts (nothing fetchable or no published number):
- No Mauch power module, LightWare SF45, CUAV X7, Matek H743-WING itself (page 404; H743-WLITE and SLIM V4 used instead), DPS310 (datasheet unreachable).
- No Raspberry Pi 5 or Zero 2 W mass, no Zero 2 W power draw, no Jetson dev-kit mass (module only).
- No M5 standoffs, no aluminium or nylon standoffs: the Duratool aluminium M4 page lists the same pack weight (.05 kg per pack of 50) for 10 mm and 15 mm lengths, so it was judged unreliable; the nylon Ettinger 5.30.310 weight (.00032 kg) only came through a search summary and could not be re-fetched verbatim (element14.cn began returning a JS challenge page).
- No shaft couplers (no source sought), no AWG 10-18 (already in parts/drone), no Pololu mass, no JST mating-cycle counts.
- Pololu efficiency curves are image-only; only the 85-95% range is recorded.

Lower confidence:
- Pi Camera Module 3 FOV, mass and focal length came via a page summariser (Pi docs returned 403 to curl).
- Easy Composites 15x12 tube page is internally inconsistent (101 g/m vs 0.045 kg gross), so no 15x12 tube was added.
- Bearing loads are NSK catalog open-type values; NSK mass column units differ between the miniature table (g) and the 68/69 table (kg).
- Standoff thread-bore depth for male-female types is assumed equal to body length.
