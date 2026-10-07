# Actuator library: sources index and gaps

Scope: `parts/actuators/` (53 parts, stage P1b). Accessed 2026-10-07. Validator: `python3 tools/check_parts.py`, 0 problems.

## Counts

| Category | Parts |
|---|---|
| Feetech bus servos (STS3215 C018/C044/C046, STS3250, STS3032, SCS0009, SCS15, HL3606, HL3612) | 9 |
| Dynamixel (XL330 x2, XL430, XC430, XM430 x2, XM540 x2) | 8 |
| Hobby PWM servos (MG996R, SG92R, DS3218MG Pro, Savox SB-2290SG, Hitec HS-485HB) | 5 |
| Steppers (NEMA 8/11/14 x2/17 x2/23 x2, SOYO) | 8 |
| Stepper drivers (DRV8825, A4988, TMC2209, TMC2226) | 4 |
| DC gearmotors (Pololu micro metal x7, 25D x2, 37D x3) | 12 |
| DC motor drivers (DRV8833, TB6612FNG) | 2 |
| BLDC/QDD (CubeMars AK80-9, AK10-9, AK60-6, AK70-9, AKE60-8) | 5 |

## Sources index

- Feetech: OpenELAB product pages (STS3215 C018/C044/C046, STS3250 C002, HL3606, HL3612), AIFITLAB product pages (STS3032, STS3250, SCS0009, SCS15), Pakronics (SCS0009).
- Dynamixel: ROBOTIS e-Manual `https://emanual.robotis.com/docs/en/dxl/x/<model>/` for xl330-m077, xl330-m288, xl430-w250, xc430-w150, xm430-w210, xm430-w350, xm540-w150, xm540-w270.
- Hobby servos: Adafruit 1142 (MG996R), Adafruit 169 (SG92R), rcdrone.top (DS3218 Pro), rc-modelshop and rcvisions (Savox SB-2290SG), servocity (HS-485HB).
- Steppers: Pololu product and /specs pages 1200, 1204, 1205, 1208, 1209, 1473, 1476, 2267; SOYO datasheet PDFs hosted by Pololu (`/file/0J627`, `0J635`, `0J673`, `0J686`, `0J714`).
- Stepper drivers: Pololu 1182 (A4988), 2133 (DRV8825); BTT wiki and LCSC C465949 (TMC2209); radiolocman datasheet 618187 (TMC2226).
- DC gearmotors and drivers: Pololu /specs pages 1093, 998, 1101, 994, 2367, 992, 3041, 3204, 3231, 4741, 4742, 4746, 2130, 713; category page `/category/60/micro-metal-gearmotors`; product pages 3204 (25D family) and 4742 (37D family) for mounting and load limits.
- BLDC: CubeMars product pages `https://www.cubemars.com/product/<slug>.html` (ak80-9-v3-0-robotic-actuator, ak10-9-v3-0-kv60-robotic-actuator, ak60-6-v3-0-kv80-robotic-actuator, ak70-9-v3-0-kv60-robotic-actuator, ake60-8-kv80-quasi-direct-drive-actuator).

Full per-part quotes are in each file's `[[sources]]`.

## Gaps and caveats

1. Mounting and horn/flange patterns. ROBOTIS drawing PDFs returned 404 and CubeMars drawings (PDF/STEP) were not retrieved, so Dynamixel, CubeMars, Feetech and 25D mount ports use opaque family tags (e.g. `bolt-pattern.pololu-25d-2hole.m3`); hole spacing is not numeric. Only the 37D mount (`bolt-circle.31.6.m3`, derived from 15.5 mm hexagon side) and the NEMA17 31x31 M3 pattern are numeric.
2. CubeMars electrical model: Kt/Ke/R/L/inertia are published, gearbox efficiency is not (peak current is DC bus current), so no eta. Published Kt disagrees with Ke in SI for AK60-6 (0.135 vs 0.119) and AKE60-8 (0.13 vs 0.119); both kept. AK80-9 no-load speed (570 rpm) is 7 % above 48 V x KV / N. Published inertia is assumed rotor-only; reflected inertia is tagged `estimated`. No absolute-maximum or minimum voltage, CAN bit rate or logic levels on the product pages (manual not retrieved). AKE60-8 has no driver board, so no CAN/UART port. AK60-6 connector type is unnamed.
3. ROBOTIS spec tables give stall torque, current and Kt at three voltages but no no-load current, so friction is not derived for Dynamixels. Rotor inertia is missing for most servos.
4. Pololu stall values are theoretical extrapolations; stall torque exceeds Pololu's recommended limits for 25D (4/8 kg.cm), 37D (10/25 kg.cm) and micro metal (2 kg.cm). The limits are stored as separate params. 37D 12 V and 6 V points give R of 2.18 and 2.0 ohm (Pololu inconsistency); 12 V used. Micro metal body shaft offset in the 10x12 section is not published.
5. Feetech: STS3032 and SCS15 gear ratios are unpublished (output-referred models only). Some Feetech/Dynamixel pages give eta > 1 or inconsistent figures; these are flagged in the files, not clamped.
6. Hobby servos: little or no stall-current data (MG996R estimate was removed as unsourced); several values rely on reseller pages.
7. TMC2209/TMC2226: the analog.com datasheet host was unreachable; values come from the BTT wiki, LCSC and radiolocman. A TMC2226 element14 source returned no usable text and was dropped, along with package and pin-compatibility claims.
8. DRV8833: logic-level thresholds not on the Pololu page, so the signal port has no levels. Header pin counts and spacing for all driver carriers are opaque `pinout.*` tags (except where already given in the file).
9. Minor: SY35 NEMA14 rotor inertia not published; omitted.
