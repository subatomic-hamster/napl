# Holybro X500 V2 airframe, avionics, sensors, fasteners, materials (N1b)

Accessed 2026-10-06/07. Parts files: `parts/drone/` (frame, plates, tubes, 6C, GPS, SiK, sensor chips) and
`parts/common/` (fasteners, materials). Propulsion/power parts are another worker's (`docs/research/x500v2-kit.md`).

## Sources

| What | URL |
|---|---|
| Kit docs (specs, weight, flight time) | https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2.md |
| Shop page (frame kit SKU 30120, ARF 30125; links to guide/CAD) | https://shop.holybro.com/x500-v2-kit_p1288.html |
| Assembly guide PDF (2 pages, exploded views, screw counts) | https://cdn.shopifycdn.net/s/files/1/0604/5905/7341/files/X500_V2_Assembly_Guide_en.pdf?v=1720853913 |
| Frame STEP (CAD) | https://2367252986-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2FLIgtGDAvVGkCKGOJb1bR%2Fuploads%2F8i7OGwxNrtr6nOP4BgQ0%2Fx500v2-frame.step?alt=media&token=c06776fd-bd31-40e5-aaa3-9fb56c6ccf5a (24 MB, Autodesk translation, 2022-07-19) |
| 3D-print files (landing-gear reinforcement) | https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_X500_V2_3D_Print.rar?v=1665561017 (RAR v5; 7z here could not open it, content unseen) |
| PX4 build guide for this kit (screw lists) | https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/docs/en/frames_multicopter/holybro_x500v2_pixhawk6c.md |
| Pixhawk 6C spec | https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification.md and `/overview.md` |
| GPS family spec | https://docs.holybro.com/gps-and-rtk-system/m8n-m9n-m10-gps/standard-m10-m9n-m8n-gps/overview.md |

CAD license: **Holybro states none** on the page, the STEP file header, or the docs. Treat as all-rights-reserved
reference geometry: we take dimensions (facts) and re-model our own CAD; we do not copy the STEP into the tree. All
`cad_*` values in the parts files were measured by loading the STEP with OpenCascade (`cadquery-ocp` in a scratch venv)
and reading solid volumes, bounding boxes and cylinder-face radii. 244 solids; X500 frame is Y-up in that file.

## Validation targets (published)

| Quantity | Value | Where | Caveat |
|---|---|---|---|
| Weight | **610 g** | docs page "Mechanical Specification" (shop page repeats under frame kit) | ambiguous: not stated whether frame only or with motors/ESC/PDB. Structural CAD sum below is ~0.33 kg, so 610 g plausibly includes screws, boards, hangers and maybe the arm electronics. Not the ARF all-up weight. |
| Shipping weight | frame kit 800 g, ARF kit 1500 g | Shopify variant weights on shop page | includes packaging; upper bound only |
| Hover flight time | **~18 min**, no payload, 5000 mAh battery | docs page | battery chemistry/C-rating/flight profile not stated |
| Max payload | ~1 kg (4S 5000 mAh, 70% throttle) docs; 1500 g (without battery, 70% throttle) shop page | the two pages disagree | |
| Recommended battery | 4S 3000-5000 mAh 20C+ XT60 (shop); 4S 2000-5000 mAh (docs) | | |
| Recommended / max takeoff weight | **not published** | | gap; derive from motor thrust table + 70% throttle statement |
| Wheelbase 500 mm; plates 144x144x2 mm; plate gap 28 mm; landing gear height 215 mm; motor pattern 16x16 (shop adds 19x19) | | docs/shop | |

## Geometry (published text vs STEP)

| Item | Holybro text | STEP-derived |
|---|---|---|
| Arm tube | "16mm carbon fiber tubes" | OD 15.4, ID 14.0, length 161.1 mm (volume 5209 mm3), 45 deg in X frame, centre 148 mm from centre |
| Landing-gear pole | 16 mm | OD 16.0, ID 14.0, length 197.7 mm (9315 mm3), leans out ~17.5 deg |
| Landing-gear cross bar | 10 mm | OD 9.4, ID 8.0, length 250.0 mm (4783 mm3) |
| Rail rods | dual 10 mm x 250 mm | OD 10.0, ID 8.0, length 248 mm (7012 mm3) |
| Top plate / bottom plate | 144x144x2 mm CF twill | 143.72 x 2.00 x 143.72 bbox; volumes 37310 / 34549 mm3; at heights +31 / +1 mm (gap 28 mm) |
| Nylon (per arm) | "fiber reinforced nylon connectors" | arm-end clamp 11984 mm3, 2 body halves 4835 mm3 each, motor base plate 2171 mm3 (roles inferred from position) |
| Motor | 2216 | solid 28.7 dia x 51.4 mm (15832 mm3) at (+-177.1, +-177.1) mm; 4 off. Motor x,y = wheelbase/2/sqrt(2) = 176.8 mm |
| Flange nut M3 | | 7.6 mm dia x 4.8 mm, 102 mm3, 16 off |

Tube OD 15.4 (CAD) vs "16mm" (text) is unresolved; the parts file records both. For mass-budget purposes CAD volumes are
consistent with each other; if a real tube is weighed, rescale density.

CAD-derived structural masses (density estimates, not weighed): top 57.8 g, bottom 53.6 g, 4 arm tubes 32.3 g,
2 landing poles 28.9 g, 2 cross bars 14.8 g, 2 rail rods 21.7 g, nylon 4 x 31.0 g = 124 g. Sum 0.333 kg. Not modelled
yet: battery mounting board (#31108), platform board (#31109), 8 hangers + rubber rings, slide-bar clips, tee connectors,
foam grips, XT60 holder, screws. Mass-budget closure belongs to the assembly stage.

## Fasteners (counts from the assembly guide; per-arm counts in step 1 are inferred x4)

The guide is a single exploded-view page; step 1 draws one arm. The PX4 build guide lists totals only for step 5
(M3x38 x16 + flange locknut M3 x16), consistent with 4 arms x 4 bolts. Treating step-1 quantities as per-arm is an
inference. The kit ships arms with motors and ESCs pre-installed, so step-1 screws may already be in place.

| Part id (parts/common) | Guide label | Qty (vehicle) | Step |
|---|---|---|---|
| iso4762-m2.5x6 | Socket Cap Screw M2.5*6 | 4/arm = 16 (step 1) + 8 (step 3, "For XT60 Holder") = 24 | 1, 3 |
| iso4762-m2.5x10 | Socket Cap Screw M2.5*10 | 2/arm = 8 | 1 |
| iso4762-m2.5x12 | Socket Cap Screw M2.5*12 | 2/arm = 8 (+4 optional companion computer) | 1, extras |
| iso4762-m3x6 | Socket Cap Screw M3*6 | 4/arm = 16 (motor to motor base) | 1 |
| iso10642-m2.5x6 | Sunk Screw M2.5*6 | 12 | 2 |
| iso7045-m3x14 | Screw M3*14 | 4 | 3 |
| din985-m3-locknut | Locknut M3 | 4 (step 3) + 4 (GPS mount) = 8 | 3, extras |
| nylon-standoff-m3x5 | Nylon Standoff M3*5 | 4 | 3 |
| iso4762-m3x8 | Socket Cap Screw M3*8 | 8 | 4 |
| iso4762-m3x38 | Socket Cap Screw M3*38 | 16 | 5 |
| m3-flange-locknut | Flange Locknut M3 | 16 | 5 |
| iso7045-m3x10 | Screw M3*10 (GPS mount) | 4 | extras |
| (not filed) Nylon standoff M2.5*5, Locknut M2.5 | companion computer, 4 each | optional | extras |

Other guide items without parts files: hanger x8, hanger rubber ring gasket x8, slide bar clip x2, battery pad,
XT60 holder (#22), 2 battery straps (shop page).
Mass source: fasten.it tables of DIN EN ISO 4762 / 7045 / DIN 7991 / DIN 985 ("Approximate mass, in kg x1000 pcs,
rho = 7.85 kg/dm3"). Non-table values: M3x38 (derived by extrapolation), M2.5x6 countersunk (scaled from M3x6),
flange locknut and nylon standoff (estimated). Head types for the two plain "Screw M3*10/14" entries are assumed pan
head from STEP names (`M3-14-PAN`).

## Avionics

- **Kit contents** (docs): Pixhawk 6C, PM02 V3-12S, **M8N GPS**, SiK Telemetry Radio V3 433/915 MHz, frame kit, 4x 2216 KV920,
  4x BLHeli_S 20A ESC, 6x 1045 props, PDB. The kit GPS is the **M8N**, not M10/M9N; Holybro's M8N/M9N/M10 table has
  no M8N column, so the M8N GPS module mass and size are estimates from the M10/M9N V1 column (32 g, phi50 x 14.4).
- **Pixhawk 6C**: 84.8 x 44 x 12.4 mm, 59.3 g (aluminium case; 34.6 g plastic). Sensors: ICM-42688-P and BMI088
  (Holybro now; PX4 docs v1.17.0 still say BMI055, discontinued), IST8310, MS5611. Integrated vibration isolation and
  IMU heater (text only; no stiffness/frequency). The BMI055 datasheet fetch failed (HTML returned), so no BMI055 file.
- RC receiver: not in the kit (RadioMaster R81 sold separately), nothing filed.

### Sensor noise summary (datasheet; rad/s, m/s2 conversions in the TOML)

| Chip (file) | Gyro noise density | Accel noise density | Zero offset | Range | ODR | Resolution |
|---|---|---|---|---|---|---|
| ICM-42688-P (`tdk-icm-42688-p`) | 0.0028 deg/s/rtHz | 65 ug/rtHz XY, 70 Z | gyro +-0.5 deg/s, +-0.005 deg/s/degC; accel +-20 mg, +-0.15 mg/degC | +-15.6..2000 deg/s; +-2..16 g | gyro 12.5-32000 Hz, accel 1.5625-32000 | 16 bit; 2048 LSB/g @16 g; 16.4 LSB/dps @2000 |
| BMI088 (`bosch-bmi088`) | 0.014 deg/s/rtHz | 160 ug/rtHz XY, 190 Z (+-3 g, normal) | gyro +-1 deg/s, +-0.015 deg/s/K; accel 20 mg, <0.2 mg/K | +-125..2000 deg/s; +-3..24 g | gyro 100-2000 Hz, accel 12.5-1600 | 10920 LSB/g @3 g; 61.0 mdps/LSB @2000 |
| MS5611 (`te-ms5611-01ba03`) | | | +-1.5 mbar @25C 750 mbar | 10-1200 mbar | | RMS 1.2 Pa @OSR4096, 6.5 Pa @OSR256; temp 0.002 K |
| IST8310 (`isentek-ist8310`) | | | zero-gauss +-0.3 uT RMS | +-1600 uT XY, +-2500 Z | 1-200 Hz | 0.3 uT/LSB; **noise not published** (estimated 0.3 uT) |
| NEO-M8N (`ublox-neo-m8n`) | | | | | 10 Hz GPS-only, 5 Hz GPS+GLONASS | 2.5 m CEP (2.0 SBAS), vel 0.05 m/s, heading 0.3 deg; vertical not published (estimated) |

PX4 SITL's simulated IMU 0 is configured as a +-16 g, 2048 LSB/g FIFO accel (SimulatorMavlink.cpp
`ACCEL_FIFO_SCALE = CONSTANTS_ONE_G / 2048.f`, `ACCEL_FIFO_RANGE = 16.f * CONSTANTS_ONE_G`), matching the ICM-42688-P.

## Gaps

1. Takeoff weight / max AUW not published; ARF all-up weight not published (only shipping weight).
2. 610 g scope ambiguity; tube OD 15.4 vs 16 mm; GPS-pole and platform/battery-board geometry not filed.
3. No Holybro CAD license; no CF layup/resin data (density and plate modulus estimated).
4. IMU in-run bias instability / ARW (only initial offsets and tempco published), IST8310 noise, GNSS vertical accuracy,
   6C isolation stiffness/frequency and mounting-hole pattern (drawing image only): none published; omitted or estimated.
5. BMI055 datasheet not retrieved. M8N-specific GPS mass/size.
6. Per-arm screw counts in step 1 are inferred; nylon-connector roles are inferred from positions.
7. ICM-42688-P datasheet is a third-party mirror of TDK DS-000347 rev 1.6; official TDK URL returned HTML to curl.
8. Assembly-guide RAR (3D print) unreadable here.
