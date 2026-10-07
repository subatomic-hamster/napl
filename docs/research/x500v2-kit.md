# Holybro X500 V2 propulsion and power chain: kit contents and sources

Researched 2026-10-06. Part files: `parts/drone/` (ids listed at the end).

## Kit contents (official Holybro)

PX4 Development Kit X500 v2 (https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2.md) includes:

- Pixhawk 6C, PM02 V3-12S Power Module, M8N GPS, SiK radio V3, X500 V2 Frame Kit (not covered here)
- Preinstalled: "Holybro 2216 KV920 Motor (4 pcs) with XT30 Plug"; "BLHeli S ESC 20A (4 pcs) with XT30 Plugs - Compatible with 4S Battery"
- "1045 Propellers (6 pcs)"
- "Power Distribution Board (XT60 plug for battery & XT30 plug for ESCs & peripherals)"
- "Recommend 4S 2000-5000mAh Lipo Battery (Not Included)"; the ARF kit page (https://holybro.com/products/x500-v2-kits) says "Battery Recommendation: 4S 3000-5000mAh 20C+ with XT60 Lipo Battery (Not Included)"
- Frame weight 610 g, "Motor mount pattern: 16x16mm & 19x19mm" (ARF page), "Flight time: ~18 minutes hover ... Tested with 5000mAh Battery."

Motor, ESC and prop numbers come from Holybro's own spec image, linked as "Motor Specification" on the ARF kit page:
https://cdn.shopify.com/s/files/1/0604/5905/7341/files/X500MotorSpec.png?v=1678791632
(read visually; text transcribed). Its title blocks are "AIR2216II", "BLHeli S 20A" and "T1045II". The same test table is published by LIGPOWER (https://www.ligpower.com/product/air-gear-450II-combo-set.html) for AIR2216II-KV920 + T1045, and the 15 rows match the Holybro image exactly. The Holybro motor is therefore the LIGPOWER AIR2216II rebadged (inference from identical spec text and table).

| Item | Value | Source |
|---|---|---|
| Motor | AIR2216II-KV920: Kv 920, R 115+-10 mOhm, I0 0.8 A @ 10 V, peak 17 A (180 s), 272 W (180 s), 64+-2 g incl. cable, M6 shaft thread, O3 shaft, O27.7 body, 42.5 mm overall, 4xM3 on 16 and 19 mm patterns, 150+-3 mm leads | Holybro spec image |
| Prop | T1045II 10"x4.5, 12.5 g, nylon+glass fibre, optimum 6000-7000 rpm, thrust limit 1.2 kg | Holybro spec image |
| ESC | BLHeli_S 20 A, 30 A peak (10 s), 26x14x5 mm, 21 g incl. cable, 3-4S, signal 50-600 Hz, no BEC | Holybro spec image + kit page |
| ESC signalling | FMU PWM Out (PX4 X500 v2 build guide) | https://raw.githubusercontent.com/PX4/PX4-user_guide/main/en/frames_multicopter/holybro_x500v2_pixhawk6c.md |
| Power module | PM02 V3 (12S analog): divider 18.182, 36.364 A/V, 60 A continuous PCB, 120 A max sensing, 5.2 V 3 A BEC, 20 g, pre-soldered XT60 + 12 AWG, 220 uF 63 V cap | https://shop.holybro.com/power-modulepm02-v3_p1185.html ; https://docs.holybro.com/power-module-and-pdb/power-module/analog-power-module-setup.md |
| PDB | 60 A rated, 120 A max (<60 s), 45x45 mm holes, XT60 in, XT30 pre-soldered out | https://holybro.com/products/power-distribution-board-pdb |
| XT60 | Amass: 0.55 mOhm CR, 30 A rated, 60 A instantaneous, 12 AWG recommended | https://mrsdprojects.ri.cmu.edu/2025teamb/wp-content/uploads/sites/84/2025/05/xt60.pdf (copy of Amass V1.2 datasheet) |
| Wire | 12 AWG on PM02 pre-soldered leads (Holybro connector and wire rating page) | https://docs.holybro.com/power-module-and-pdb/power-module/connector-and-wire-rating.md |

## Chosen battery

Tattu 14.8 V 4S1P 5200 mAh 35C XT60, 436.5+-20 g, 133x45x33.5 mm, 10 AWG leads 120 mm. Chosen because Holybro recommends 4S 3000-5000 mAh 20C+ XT60 and the PX4 X500 v2 guide's reference battery is "4S 16.8V 5200 mAh". Spec taken from an authorized distributor listing (rcdrone.top); no Tattu-published internal resistance.

## Validation target

`parts/drone/holybro-2216-kv920.toml` `[tables.thrust_test]`: 15 rows, 30-100 % throttle, constant 16 V, one motor + T1045II, thrust 2.06-13.06 N, current 1.44-16.37 A, rpm 4042-9857. Vendor caveats: "(Date above based on benchtest are for reference only...)" and thrust is printed in grams (converted with g = 9.80665 m/s^2). Torque is printed to 0.01 N*m (low resolution). Test air density and temperature are not stated.

Derived from this table (not vendor values): static C_T of the 1045 is about 0.095 (0.089 at 4000 rpm rising to 0.096 at 9000 rpm, assuming rho = 1.225) and C_Q about 0.0053. The APC SF 10x4.7 reference shows static C_T 0.116-0.130 over 4000-6500 rpm, so the Holybro prop is about 20-25 % lower in static thrust coefficient; use the UIUC curves only for the J-shape and scale them by the 1045's own static values.

## Gaps (values estimated, no source found)

- Motor: stator diameter/height (from the "2216" naming convention), rotor bell inertia (4.2e-6 kg*m^2 estimated), no thermal model data (only "80 C surface at 100 % throttle after 10 min").
- Prop: blade count (2, standard 1045), moment of inertia (derived rod upper bound 6.72e-5 kg*m^2; no mass distribution published), no J > 0 data for this exact prop (borrowing the UIUC APC SF 10x4.7 shape).
- ESC: FET on-resistance (12 mOhm assumed), PWM carrier (24 kHz assumed), DShot support not stated on Holybro pages; PX4 guide wires ESCs to PWM outputs, signal spec 50-600 Hz.
- PM02 V3: shunt resistance and amplifier gain not published (only end-to-end 36.364 A/V); board resistance (1.5 mOhm) and ADC full scale (3.3 V) estimated; no dimensions drawing found in text.
- PDB: board mass and copper resistance estimated; dimensions only as an image.
- XT60: Amass lists CR 0.55 mOhm for each of XT60-F and XT60-M; whether a mated pair is 0.55 or 1.1 mOhm is not stated, both are recorded.
- Wires: ESC and motor lead gauges inside the kit are unpublished (12 AWG is confirmed only for the PM02 leads; 10 AWG for the Tattu pack). Files for 14/16/18 AWG are table entries only. Silicone wire mass per metre is estimated (copper mass derived + 1.4x insulation factor); resistance is solid-copper 20 C table value.
- Battery: internal resistance and Thevenin RC values are estimated; no Tattu pack data published.
- LiPo OCV(SoC): no accessible peer-reviewed numeric table was found (graph-only or paywalled/403). The table recorded is the Modelica Standard Library 4.0.0 example OCV/SOC data (OCVmax 4.2 V), tagged `estimated`, not `literature`.

## Uncertainties in the kit BOM

- Holybro pages for the same kit differ slightly: "4S 2000-5000mAh" (dev kit page) vs "4S 3000-5000mAh 20C+" (ARF page); motor mount "16x16mm" (dev kit) vs "16x16mm & 19x19mm" (ARF).
- The ARF page says "less than 15 minutes" assembly, the dev kit page "~30 minutes".
- The spec image says the T1045II is "(260mmx30mm)" while the title string says 10"x4.5 (254 mm x 114.3 mm in the LIGPOWER text); recorded diameter 0.254 m, pitch 0.1143 m. The image's "30mm" is unexplained (possibly hub width).
- LIGPOWER's page prints a 13 g prop and a 53x22x7 mm / 3.5 mm-banana ESC, Holybro's image prints 12.5 g and 26x14x5 mm / 2.0 mm; Holybro numbers are used.
- Holybro shop "weight" fields (motor 80 g, ESC 50 g, PDB 50 g, prop pair 80 g) are shipping weights and were not used.
- X500 V2 gets its own ESC/motor/propeller preinstalled in the ARF kit; the dev kit page does not state the prop-retainer part, which is a frame item (out of scope).

## Part ids written

holybro-2216-kv920, holybro-1045-prop, uiuc-apc-sf-10x4.7, holybro-x500v2-esc-blheli-s-20a, holybro-pm02-v3, holybro-pdb-60a, amass-xt60, awg-10-silicone-wire, awg-12-silicone-wire, awg-14-silicone-wire, awg-16-silicone-wire, awg-18-silicone-wire, tattu-4s-5200mah-35c, lipo-cell-ocv
