# DYNAMIXEL servos, Protocol 2.0, electronics and gain scaling for the Koch v1.1 follower (stage A3 / S1)

Accessed 2026-10-07 (every URL below was fetched on this date; raw page text kept in the stage scratchpad, not committed). Labels used throughout: **[F]** = documented fact from a fetched source, **[I]** = inference (stated basis), **[M]** = from training memory (none is relied on; any appearance is flagged). Part files built from this page: `parts/arm/robotis-xl430-w250-t.toml`, `robotis-xl330-m288-t.toml`, `robotis-xl330-m077-t.toml`, `robotis-u2d2.toml`, `robotis-robot-cable-x3p-180mm.toml`, `facmogu-12v-5a-psu.toml`, `eplzon-12v-5v-reducer.toml`. Servo mech ports / geometry are NOT in those files yet (stage D, `tools/koch_ports.py`).

Note [F]: both e-Manual and the protocol pages carry the banner "ROBOTIS e-Manual has moved ... The e-Manual has been integrated into ROBOTIS Docs ... This e-Manual website will be removed in July." The new site has the same control table (`https://docs.robotis.com/docs/dxl/model_reference/x_series/xl_series/xl330-m288/` fetched, identical rows). URLs below are the e-Manual ones the part files cite.

## 0. Summary of the numbers that matter

| Item | XL430-W250-T | XL330-M288-T | XL330-M077-T |
|---|---|---|---|
| Role in Koch v1.1 | follower shoulder_pan, shoulder_lift (2x) | follower elbow, wrist_flex, wrist_roll, gripper (4x) | leader, all 6 joints |
| Supply in Koch | **12 V** (12 V PSU through the Waveshare board) | **5 V** (12 V to 5 V "Voltage Reducer") | 5 V (5 V PSU) |
| Voltage range [F] | 6.5-12.0 V (rec. 11.1) | 3.7-6.0 V (rec. 5.0) | 3.7-6.0 V (rec. 5.0) |
| Gear ratio [F] | 258.5 | 288.4 | 77.5 |
| Stall torque / current [F] | 1.0 N*m @9 V 1.0 A; 1.4 @11.1 V 1.3 A; 1.5 @12 V 1.4 A | 0.42 @3.7 V 1.11 A; 0.52 @5 V 1.47 A; 0.60 @6 V 1.74 A | 0.180 @3.7 V; 0.215 @5 V; 0.228 @6 V (same currents) |
| No-load speed [F] | 47 / 57 / 61 rev/min @ 9 / 11.1 / 12 V | 76 / 103 / 123 rev/min @ 3.7 / 5 / 6 V | 278 / 383 / 456 rev/min |
| Published no-load current | none (standby 52 mA) | none (standby 17 mA) | none (standby 17 mA) |
| Mass / size [F] | 57.2 g, 28.5 x 46.5 x 34 mm | 18 g, 20 x 34 x 26 mm | 18 g, 20 x 34 x 26 mm |
| Model number [F] | 1060 | 1200 | 1190 |
| Operating modes [F] | 1, 3, 4, 16 | 0, 1, 3, 4, 5, 16 | 0, 1, 3, 4, 5, 16 |
| Position P / I / D default [F] | 640 / 0 / 4000 | 400 / 0 / 0 | 400 / 0 / 400 |
| Shutdown default [F] | 52 (0x34) | 53 (0x35) | 53 (0x35) |
| Model r_effective / ke / eta [derived] | 8.55 ohm / 6.8e-3 / 0.650 | 3.42 ohm / 1.515e-3 / 0.851 | 3.42 ohm / 1.518e-3 / 1.0 (clamped, raw 1.254) |
| Recommended `pid_scale` / `d_scale` / `i` factor | 113,280 / 14,160 / I/512 | same | same |

The Koch **follower** is mixed-voltage: 2x XL430 at 12 V, 4x XL330 at 5 V on one data bus (section 8). XL330 max is 6.0 V, XL430 min is 6.5 V, so the arm needs the reducer; this is the only way every servo is in range.

## 1. Sources fetched

| # | URL | What |
|---|---|---|
| E1 | https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ | XL430-W250-T spec + control table (also contains the "Old XL430-W250 (Discontinued)" variant on the same page; the single table below applies) |
| E2 | https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ | XL330-M288-T |
| E3 | https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/ | XL330-M077-T |
| E4 | https://emanual.robotis.com/docs/en/dxl/protocol2/ | Protocol 2.0 |
| E5 | https://emanual.robotis.com/docs/en/dxl/crc/ | CRC calculation |
| E6 | https://emanual.robotis.com/docs/en/parts/interface/u2d2/ , `.../u2d2_power_hub/` | U2D2, U2D2 Power Hub |
| E7 | https://shop.pishrobot.com/wp-content/uploads/2020/01/dynamixel-xl430-w250.pdf | older e-Manual PDF copy of the XL430-W250 (2018-2020 revision): "The constant in each equations include sampling time." |
| K1 | https://github.com/jess-moss/koch-v1-1/blob/1a553e1fa7d5404b51bbd83b99b0f7eba54888e0/README.md | Koch v1.1 README (commit 1a553e1 = head of main, 2024-09-17) |
| K2 | https://github.com/jess-moss/koch-v1-1/blob/1a553e1fa7d5404b51bbd83b99b0f7eba54888e0/electrical/Wiring_Diagram_Alexander_Koch_V1.1.pdf | Koch wiring diagram (EasyEDA, 2024-07-08; PDF read as image and text layer) |
| L1 | https://github.com/huggingface/lerobot/blob/70a0bf63da117a8585dcc13abf58ac5d12c121c5/src/lerobot/robots/koch_follower/koch_follower.py ; `.../motors/dynamixel/dynamixel.py` @df0763a ; `.../motors/dynamixel/tables.py` @a855570 ; `.../docs/source/koch.mdx` (main) | LeRobot Koch follower, model table, defaults |
| B1 | https://github.com/Rhoban/bam/blob/e9a619d56da5236206f4de6ceec2c1ee1b497b5c/bam/dynamixel/actuator.py ; `bam/params/xl330/m1.json ... m6.json` | Rhoban BAM (Apache-2.0) identified XL330-M288-T models and an empirical XL330 gain divisor |
| A1 | https://www.amazon.com/dp/B0BYVK73BP ; https://www.amazon.com/EPLZON-Converter-5V-5-3V-Transformer-Regulator/dp/B09R4DBZJK | Facmogu 12 V 5 A and EPLZON reducer product titles (Amazon blocks scripted GET; titles were read through the web-fetch tool, not curl) |
| J1 | http://www.jst-mfg.com/product/pdf/eng/eEH.pdf ; https://robotis.us/robot-cable-x3p-180mm-10pcs/ | JST EH connector rating; Robot Cable-X3P store page |

Not fetchable: ROBOTIS "Moment of Inertia" downloads (`https://www.robotis.com/service/download.php?no=2136` / `2134`): HTTP 302 to `http://se-cu.com/error.html`, an unrelated host (not followed). Old `support.robotis.com` pages: DNS failure. So no published rotor inertia.

## 2. Specifications (verbatim, E1-E3)

XL430-W250-T (E1): "MCU | ARM CORTEX-M3 (72 [MHz], 32Bit) || Position Sensor | Contactless absolute encoder (12Bit, 360 [°]) Maker : ams(www.ams.com), Part No : AS5601 || Motor | Cored || Baud Rate | 9,600 [bps] ~ 4.5 [Mbps] || Control Algorithm | PID control || Resolution | 4096 [pulse/rev] || Operating Modes | Velocity Control Mode, Position Control Mode (0 ~ 360 [°]), Extended Position Control Mode (Multi-turn), PWM Control Mode (Voltage Control Mode) || Weight | 57.2 [g] || Dimensions (W x H x D) | 28.5 x 46.5 x 34 [mm] || Gear Ratio | 258.5 : 1 || Stall Torque | 1.0 [N.m] (at 9.0 [V], 1.0 [A], 1.000 [Nm/A]) 1.4 [N.m] (at 11.1 [V], 1.3 [A], 1.077 [Nm/A]) 1.5 [N.m] (at 12.0 [V], 1.4 [A], 1.071 [Nm/A]) || No Load Speed | 47 [rev/min] (at 9.0 [V]) 57 [rev/min] (at 11.1 [V]) 61 [rev/min] (at 12.0 [V]) || Operating Temperature | -5 ~ +72 [°C] || Input Voltage | 6.5 ~ 12.0 [V] (Recommended : 11.1 [V]) (lithium-ion battery 3 Cell) || Physical Connection | TTL Multidrop Bus (5V Logic) TTL Half Duplex Asynchronous Serial Communication (8bit, 1stop, No Parity) || ID | 253 ID (0 ~ 252) || Feedback | Position, Velocity, Load, Realtime tick, Trajectory, Temperature, Input Voltage, etc || Standby Current | 52 [mA]"

XL330-M288-T (E2): "MCU | ARM CORTEX-M0+ (64 [MHz], 32Bit) || ... Baud Rate | 9,600 [bps] ~ 4 [Mbps] || Resolution | 4096 [pulse/rev] || Operating Modes | Current Control Mode, Velocity Control Mode, Position Control Mode (0 ~ 360 [°]), Extended Position Control Mode (Multi-turn), Current-based Position Control Mode, PWM Control Mode (Voltage Control Mode) || Weight | 18 [g] || Dimensions (W x H x D) | 20.0 x 34.0 x 26.0 [mm] || Gear Ratio | 288.4 : 1 || Stall Torque | 0.42 [N.m] (at 3.7 [V], 1.11 [A], 0.378 [Nm/A]) 0.52 [N.m] (at 5.0 [V], 1.47 [A], 0.354 [Nm/A]) 0.60 [N.m] (at 6.0 [V], 1.74 [A], 0.345 [Nm/A]) || No Load Speed | 76 [rev/min] (at 3.7 [V]) 103 [rev/min] (at 5.0 [V]) 123 [rev/min] (at 6.0 [V]) || Operating Temperature | -5 ~ +70 [°C] || Input Voltage | 3.7 ~ 6.0 [V] (Recommended : 5.0 [V]) || Physical Connection | TTL Multidrop Bus (3.3V Logic, 5V Compatible) ... || Feedback | Position, Velocity, Current, Realtime tick, Trajectory, Temperature, Input Voltage || Standby Current | 17 [mA]"

XL330-M077-T (E3): same rows except "Gear Ratio | 77.5 : 1 || Stall Torque | 0.180 [N.m] (at 3.7 [V], 1.11 [A], 0.162 [Nm/A]) 0.215 [N.m] (at 5.0 [V], 1.47 [A], 0.146 [Nm/A]) 0.228 [N.m] (at 6.0 [V], 1.74 [A], 0.131 [Nm/A]) || No Load Speed | 278 [rev/min] (at 3.7 [V]) 383 [rev/min] (at 5.0 [V]) 456 [rev/min] (at 6.0 [V])".

Connector (E1-E3): "Pinout | 1 GND 2 VDD 3 DATA || Housing | JST EHR-03 || PCB Header | JST B3B-EH-A || Crimp Terminal | JST SEH-001T-P0.6 || Wire Gauge for DYNAMIXEL | 21 AWG". Model numbers verified in the control tables (1,060 / 1,200 / 1,190) and in LeRobot `MODEL_NUMBER_TABLE` (`xl330-m077: 1190, xl330-m288: 1200, xl430-w250: 1060`, tables.py).

Observations [F, from the numbers above]: (a) the XL330 stall data of M288 and M077 have identical currents; (b) motor-side no-load speed N x omega: M288 5 V 103 x 288.4 = 29,705 rpm, M077 383 x 77.5 = 29,683 rpm: the **same motor**; (c) no no-load current is published for any of the three; (d) the XL330 note: "XL330 series measures curret at its input power source unlike other DYNAMIXE-X series supporting a current control. Therefore, you may have a different results in measuring current of XL330 series comparing with measuring phase current which quickly changes of a DC motor."

## 3. Control tables (E1-E3, rows verbatim; "-" = blank)

Columns: address | size (byte) | name | access | default (XL430 / XL330-M288 / XL330-M077, one value if all equal) | range | unit. "n/a" = register does not exist on that model. Little endian for multi-byte values; "Two's complement is applied for the negative value" (E1).

### 3.1 EEPROM (writable only while Torque Enable(64) = 0, E2 CAUTION)

| Addr | Size | Name | Acc | Default | Range | Unit |
|---|---|---|---|---|---|---|
| 0 | 2 | Model Number | R | 1,060 / 1,200 / 1,190 | - | - |
| 2 | 4 | Model Information | R | - | - | - |
| 6 | 1 | Firmware Version | R | - | - | - |
| 7 | 1 | ID | RW | 1 | 0 ~ 252 | - |
| 8 | 1 | Baud Rate | RW | 1 | XL430 0 ~ 7; XL330 0 ~ 6 | - |
| 9 | 1 | Return Delay Time | RW | 250 | 0 ~ 254 | 2 us |
| 10 | 1 | Drive Mode | RW | 0 | 0 ~ 13 | - |
| 11 | 1 | Operating Mode | RW | 3 | 0 ~ 16 | - |
| 12 | 1 | Secondary(Shadow) ID | RW | 255 | 0 ~ 252 | - |
| 13 | 1 | Protocol Type | RW | 2 | XL430 1 ~ 2; XL330 2 ~ 22 | - |
| 20 | 4 | Homing Offset | RW | 0 | -1,044,479 ~ 1,044,479 | 1 pulse |
| 24 | 4 | Moving Threshold | RW | 10 | 0 ~ 1,023 | 0.229 rev/min |
| 31 | 1 | Temperature Limit | RW | 72 / 70 / 70 | 0 ~ 100 | 1 degC |
| 32 | 2 | Max Voltage Limit | RW | 140 / 70 / 70 | XL430 60 ~ 140; XL330 31 ~ 70 | 0.1 V |
| 34 | 2 | Min Voltage Limit | RW | 60 / 35 / 35 | XL430 60 ~ 140; XL330 31 ~ 70 | 0.1 V |
| 36 | 2 | PWM Limit | RW | 885 | 0 ~ 885 | 0.113 % |
| 38 | 2 | Current Limit | RW | n/a / 1,750 / 1,750 | 0 ~ 1,750 | 1 mA |
| 44 | 4 | Velocity Limit | RW | 265 / 445 / 1,620 | XL430 0 ~ 1,023; XL330 0 ~ 2,047 | 0.229 rev/min |
| 48 | 4 | Max Position Limit | RW | 4,095 | 0 ~ 4,095 | 1 pulse |
| 52 | 4 | Min Position Limit | RW | 0 | 0 ~ 4,095 | 1 pulse |
| 60 | 1 | Startup Configuration | RW | 0 | 3 | - |
| 62 | 1 | PWM Slope | RW | n/a / 140 / 140 | 1 ~ 255 | 1.977 mV/msec |
| 63 | 1 | Shutdown | RW | 52 / 53 / 53 | - | - |

### 3.2 RAM

| Addr | Size | Name | Acc | Default | Range | Unit |
|---|---|---|---|---|---|---|
| 64 | 1 | Torque Enable | RW | 0 | 0 ~ 1 | - |
| 65 | 1 | LED | RW | 0 | 0 ~ 1 | - |
| 68 | 1 | Status Return Level | RW | 2 | 0 ~ 2 | - |
| 69 | 1 | Registered Instruction | R | 0 | 0 ~ 1 | - |
| 70 | 1 | Hardware Error Status | R | 0 | - | - |
| 76 | 2 | Velocity I Gain | RW | 1,000 / 1,600 / 400 | 0 ~ 16,383 | - |
| 78 | 2 | Velocity P Gain | RW | 100 / 180 / 40 | 0 ~ 16,383 | - |
| 80 | 2 | Position D Gain | RW | 4,000 / 0 / 400 | 0 ~ 16,383 | - |
| 82 | 2 | Position I Gain | RW | 0 | 0 ~ 16,383 | - |
| 84 | 2 | Position P Gain | RW | 640 / 400 / 400 | 0 ~ 16,383 | - |
| 88 | 2 | Feedforward 2nd Gain | RW | 0 | 0 ~ 16,383 | - |
| 90 | 2 | Feedforward 1st Gain | RW | 0 | 0 ~ 16,383 | - |
| 98 | 1 | Bus Watchdog | RW | 0 | 1 ~ 127 (also 0, -1) | 20 msec |
| 100 | 2 | Goal PWM | RW | - | -PWM Limit(36) ~ PWM Limit(36) | 0.113 % |
| 102 | 2 | Goal Current | RW | n/a / - / - | -Current Limit(38) ~ Current Limit(38) | 1 mA |
| 104 | 4 | Goal Velocity | RW | - | -Velocity Limit(44) ~ Velocity Limit(44) | 0.229 rev/min |
| 108 | 4 | Profile Acceleration | RW | 0 | 0 ~ 32,767 (velocity-based) / 0 ~ 32,737 (time-based) | 214.577 rev/min^2 / 1 ms |
| 112 | 4 | Profile Velocity | RW | 0 | 0 ~ 32,767 (velocity-based) / 0 ~ 32,737 (time-based) | 0.229 rev/min / 1 ms |
| 116 | 4 | Goal Position | RW | - | Min Position Limit(52) ~ Max Position Limit(48) (modes 4, 5: -1,048,575 ~ 1,048,575) | 1 pulse |
| 120 | 2 | Realtime Tick | R | - | 0 ~ 32,767 | 1 msec |
| 122 | 1 | Moving | R | 0 | 0 ~ 1 | - |
| 123 | 1 | Moving Status | R | 0 | - | - |
| 124 | 2 | Present PWM | R | - | - | (same unit as Goal PWM, signed) |
| 126 | 2 | XL430: Present Load; XL330: Present Current | R | - | XL430 -1,000 ~ 1,000 | XL430 0.1 %; XL330 1 mA |
| 128 | 4 | Present Velocity | R | - | - | 0.229 rev/min |
| 132 | 4 | Present Position | R | - | - | 1 pulse |
| 136 | 4 | Velocity Trajectory | R | - | - | 0.229 rev/min |
| 140 | 4 | Position Trajectory | R | - | - | 1 pulse |
| 144 | 2 | Present Input Voltage | R | - | - | 0.1 V |
| 146 | 1 | Present Temperature | R | - | - | 1 degC |
| 147 | 1 | Backup Ready | R | - | 0 ~ 1 | - |
| 168-222 (even) | 2 | Indirect Address 1-28 | RW | 224-251 (one per entry) | XL430 64 ~ 661; XL330 64 ~ 251 | - |
| 224-251 | 1 | Indirect Data 1-28 | RW | 0 | 0 ~ 255 | - |
| 578-632 (even) | 2 | Indirect Address 29-56 (XL430 only) | RW | 634-661 | 64 ~ 661 | - |
| 634-661 | 1 | Indirect Data 29-56 (XL430 only) | RW | 0 | 0 ~ 255 | - |

[F] XL330 note: "Firmware versions prior to V53 support up to 20 Indirect Address/Data items. The address range for Indirect Address is 168 ~ 207, and for Indirect Data is 208 ~ 227."
[F] Discrepancy inside E1: the table gives Min/Max Voltage Limit range "60 ~ 140" but the description section says "Value Range 65 ~ 140 | 6.5 ~ 14.0 [V]". The part file uses the table's default 60 (6.0 V). The XL430 page also says "Hardware Error Status(70) ... refer to the Shutdown(48) control table item" (stale address; Shutdown is 63).
[F] The Goal PWM Unit cell of the XL330-M288 table is "-" but the description (and M077's table) says "about 0.113 [%]".

## 4. Register semantics the protocol/servo stages need (E1-E3)

**Operating Mode(11)** [F]: 0 Current Control (XL330 only), 1 Velocity, 3 Position (default; "Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52)"), 4 Extended Position ("512 turns are supported(-256[rev] ~ 256[rev])"; limits not used), 5 Current-based Position (XL330 only; "controls both position and current(torque). Up to 512 turns"; limits not used), 16 PWM ("directly controls PWM output"). Supported per model: **XL430-W250: 1, 3, 4, 16; XL330-M288 and M077: 0, 1, 3, 4, 5, 16** (identical to LeRobot's `MODEL_OPERATING_MODES`). Switching mode resets: Velocity PI(76,78), Position PID(80,82,84), Feedforward(88,90) "will be reset fitting to a selected Operating Mode"; Profile Velocity(112) and Profile Acceleration(108) reset to 0; Goal PWM(100) and Goal Current(102) reset to PWM Limit / Current Limit; entering Current-based Position also resets Position PID and PWM Limit.
**Present Position(132)** [F]: continuous 4-byte range when torque is off regardless of mode, but reset to an absolute value within one rotation when the mode changes to Position Control, when torque is turned on in Position Control Mode, and at power-on/reboot; "still affected by the configured Homing Offset(20)".
**Homing Offset(20)** [F]: "Present Position(132) = Actual Position + Homing Offset(20)", unit 0.088 deg, range -1,044,479 ~ 1,044,479 (-255 ~ 255 rev); in Position Control Mode (<360 deg) "any invalid Homing Offset(20) values will be ignored(valid range : -1,024 ~ 1,024)"; the sign is not reversed by Reverse Mode.
**Drive Mode(10)** [F]: bit 3 (0x08) Torque On by Goal Update (XL430 FW >= V45, XL330 FW >= V46); bit 2 (0x04) Profile Configuration [0] velocity-based [1] time-based (XL430 FW >= V42); bit 0 (0x01) Normal/Reverse (CCW positive / negative); bits 7-4 and 1 unused. Position increases CCW seen from the front of the servo.
**Moving Threshold(24)** [F]: "When the absolute value of Present Velocity(128) is greater than the Moving Threshold(24), Moving(122) is set to '1'." Moving(122) "will always be set to '1' regardless of Present Velocity(128) while Profile is in progress with Goal Position(116) instruction."
**Moving Status(123)** [F]: bit 0 In-Position (1 arrived; "set when the positional deviation is smaller than a predefined value", value not published), bit 1 Profile Ongoing, bits 5:4 velocity profile (11 trapezoidal, 10 triangular, 01 rectangular, 00 step/not used), bit 3 Following Error (0 following, 1 not following; In-Position and Following Error exist only in position modes), bit 2, 6, 7 reserved.
**Limits** [F]: Temperature Limit(31): above it Overheating Error bit 0x04 and Alert Bit 0x80 are set; "Do not set this value higher than its default". Voltage limits(32,34): outside [Min, Max] sets Input Voltage Error bit **0x01** (the e-Manual text writes "Bit(0x10)" in the voltage-limit paragraph but the Shutdown/Hardware Error table puts Input Voltage at bit 0 = 0x01 and Electrical Shock at bit 4 = 0x10: stale text, the bit table is used). PWM Limit(36): "maximum PWM output. Goal PWM(100) can't be configured with any values exceeding PWM Limit(36)... used in all operating mode as an output limit, therefore decreasing PWM output will result in decreasing torque and velocity"; unit 0.113 %, "0(0 [%]) ~ 885(100 [%] )". Current Limit(38) (XL330): "used in Torque Control Mode and Current-based Position Control Mode"; unit 1 mA, 0 ~ 1,750. Velocity Limit(44) bounds Goal Velocity; Min/Max Position Limit(52,48) bound Goal Position in Position Control Mode only.
**Shutdown(63) / Hardware Error Status(70) bits** [F] (same layout; Shutdown selects which errors turn torque off, Hardware Error Status reports them, the Alert bit 0x80 of the status-packet ERR byte mirrors any set bit):

| Bit | Mask | Error | Detects | XL430 default | XL330 default |
|---|---|---|---|---|---|
| 0 | 0x01 | Input Voltage Error | "input voltage exceeds the configured operating voltage" | off | on |
| 1 | 0x02 | - | unused, always 0 | - | - |
| 2 | 0x04 | Overheating Error | "internal temperature exceeds the configured operating temperature" | on | on |
| 3 | 0x08 | Motor Encoder Error | "malfunction of the motor encoder" (XL430 table only; XL330 table: "-") | off | - |
| 4 | 0x10 | Electrical Shock Error | "electric shock on the circuit or insufficient power to operate the motor" | on | on |
| 5 | 0x20 | Overload Error | "persistent load that exceeds maximum output" | on | on |
| 6, 7 | | unused | | | |

Defaults: XL430 52 = 0x34 (bits 2, 4, 5); XL330 53 = 0x35 (bits 0, 2, 4, 5). When a selected error occurs "Torque Enable(64) is cleared to '0' and the motor's output becomes 0 [%]. REBOOT is the only method to reset Torque Enable(64) to '1'"; LED blinks every second. The overload / electrical-shock trip thresholds (load level, duration) are **not published**.
**Torque Enable(64)** [F]: 1 = torque on and EEPROM area locked; writing an EEPROM register with torque on gives Access Error (protocol page). **LED(65)** 0/1; also shows status (boot blink once, factory reset 4 quick blinks, shutdown blinking, bootloader on). **Status Return Level(68)** [F]: 0 = status packet for PING only, 1 = PING and READ, 2 = all instructions (default); a PING always answers; broadcast ID 0xFE returns status packets only for Ping, Sync Read and Bulk Read. **Registered Instruction(69)** [F]: 1 while a REG_WRITE is pending, back to 0 after ACTION.
**Bus Watchdog(98)** [F]: 0 off (and clears the error), 1-127 active with unit 20 ms, -1 = error state; it monitors the interval between Instruction Packets while torque is on; on timeout the servo stops (Profile Acceleration/Velocity applied as 0), the register becomes -1 and "the Goal Value (Goal PWM(100), Goal Current(102), Goal Velocity(104), Goal Position(116)) will be changed to read-only-access ... a new value written to the Goal Value ... the Status Packet will send the Data Range Error"; writing 0 clears it.
**Return Delay Time(9)** [F]: status packet is returned after value x 2 us; "Default value '250'(500[us]) Maximum value: '508'[us]"; LeRobot writes 0 (S7 of the part files).
**Baud Rate(8)** [F] (margin of error in brackets): XL330: 6 = 4 Mbps, 5 = 3 M, 4 = 2 M, 3 = 1 M, 2 = 115,200 [0.0064 %], 1 = 57,600 (default) [0.0016 %], 0 = 9,600. XL430: same with 7 = 4.5 Mbps and 2 = 115,200 [0.000 %]. LeRobot `X_SERIES_BAUDRATE_TABLE` = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6}, `DEFAULT_BAUDRATE = 1_000_000`. "Less than 3% of the baud rate error margin will not affect to UART communication."
**Secondary ID(12)** [F]: 0-252 activates, 253-255 deactivates (default 255); a packet addressed to the secondary ID changes RAM only (not EEPROM) and returns **no** status packet; the primary ID has priority when equal.
**Startup Configuration(60)** [F]: bit 1 RAM Restore (restore RAM from the backup at startup), bit 0 Startup Torque On.
**PWM Slope(62)** (XL330) [F]: "The PWM duty will be linearly interpolated with a set slope by PWM Slope(62)", unit 1.977 mV/ms; default 140 = 276.84 mV/ms, so a 0 to 885 step at 5 V takes 5,000 mV / 276.84 mV/ms = 18.06 ms. (An output-voltage slew limit the sim servo device does not model.)
**Profile** [F]: Velocity-based: Profile Velocity(112) = max speed (0.229 rev/min), Profile Acceleration(108) unit 214.577 rev/min^2, 0 = infinite; Time-based: both in ms. Profile type by `VPRFL(112)`, `APRF(108)`: VPRFL = 0 -> no profile (step); VPRFL != 0 and APRF = 0 -> rectangular; both != 0 -> trapezoidal (triangular when the rectangle cannot reach VPRFL). "Velocity-based Profile : t1 = 64 * {Profile Velocity(112) / Profile Acceleration(108)}" [ms, since 214.577 rev/min^2 = 0.229 rev/min per 64 ms; inference on the unit of t1]; "Time-based Profile : t1 = Profile Acceleration(108)"; acceleration time "will not exceed 50% of Profile Velocity". The profile generator writes Position Trajectory(140) and Velocity Trajectory(136); the PID follows Position Trajectory.
**Indirect addressing** [F]: addresses 64..(227 XL330 / 661 XL430) only ("EEPROM address can't be assigned"); an Indirect Data byte inherits size/range/access of the target byte; a multi-byte register needs every byte mapped.

## 5. Control law: what the PID output feeds

### 5.1 Documented [F]

Gain conversions (E1-E3, identical on all three servos): "Position D Gain(80) KPD KPD = KPD(TBL) / 16 || Position I Gain(82) KPI KPI = KPI(TBL) / 65,536 || Position P Gain(84) KPP KPP = KPP(TBL) / 128 || Feedforward 2nd Gain(88) KFF2nd KFF2nd(TBL) / 4 (Feedforward Acceleration Gain) || Feedforward 1st Gain(90) KFF1st KFF1st(TBL) / 4 (Feedforward Velocity Gain)"; Velocity: "KVI = KVI(TBL) / 65,536", "KVP = KVP(TBL) / 128". All 0 ~ 16,383.
Signal flow in Position Control Mode (E2): "Goal Position(116) is converted to desired position trajectory and desired velocity trajectory by Profile Velocity(112) and Profile Acceleration(108). ... The desired position trajectory and desired velocity trajectory is stored at Position Trajectory(140) and Velocity Trajectory(136) respectively. Feedforward and PID controller calculate PWM output for the motor based on desired trajectories. Goal PWM(100) sets a limit on the calculated PWM output and decides the final PWM value. The final PWM value is applied to the motor through an Inverter".
Block diagram (image `position_controller_pid_gain.jpg`, read visually): Profile -> position/velocity trajectories; error = trajectory position - Present Position(132) -> parallel KPP, KPD*s, KPI/s -> summed with the feedforward branch (KFF1st*s and KFF2nd*s^2 acting on the trajectory) -> limiter (limit = Goal PWM / PWM Limit) -> Inverter -> motor; an anti-windup feedback gain Ka around the limiter ("Ka is an Anti-windup Gain that cannot be modified by users"). The diagram labels the D and I branches with addresses (82) and (80), swapped relative to the table (D = 80, I = 82); the table is used.
PWM Control Mode [F]: "both PID controller and Feedforward controller are deactivated while Goal PWM(100) value is directly controlling the motor through an inverter. In this manner, users can directly control the supplying voltage to the motor."
PWM unit [F]: "PWM Limit ... about 0.113 [%] ... 0(0 [%]) ~ 885(100 [%] )": 885 counts = 100 % duty, so the PID output is a duty fraction PWM/885 of the supply voltage. Present Load (XL430) = the same quantity in 0.1 % ("an inferred value based on the internal output value").
Current-based Position Control (XL330, mode 5) [F]: "Feedforward and PID controller calculates desired current based on desired trajectory. Goal Current(102) decides the final desired current by setting a limit on the calculated desired current. Current controller calculates PWM output ... Goal PWM(100) sets a limit". So in mode 5 the position PID output is a current command, not a duty. LeRobot puts the **gripper** servo in this mode (section 8.3).
Units of velocity and current [F]: velocity 0.229 rev/min per count (all velocity data), i.e. 0.0239808 rad/s; XL330 current 1 mA per count.
Update rate [F]: Realtime Tick(120) unit 1 ms; "Representes the internal time when the Status Packet was created" (E2). **The control-loop rate is not published** in any fetched current page.

### 5.2 Sampling time [F, older manual only]

The older XL430-W250 e-Manual copy (E7) says in both the Velocity PI and Position PID sections: "Gains of DYNAMIXEL's internal controller can be calculated from Gains of the Control Table as shown below. The constant in each equations include sampling time." The current e-Manual dropped that sentence. [I] Reading: KPI = I/65536 already contains the sample period (KPI = Ki*Ts) and KPD = D/16 contains 1/Ts (KPD = Kd/Ts), so the firmware law is the discrete form `PWM = KPP*e + KPI*sum(e) + KPD*(e - e_prev)` evaluated once per control period, with no further time scaling. This is exactly the structure of the napl servo device (per-firmware-step sum and difference).

### 5.3 Independent measurement [F, third party]

Rhoban BAM (Apache-2.0, commit e9a619d) models the XL330 with `duty = (q_target - q) * kp * error_gain` and sets `error_gain=(XL330_ENCODER_COUNTS_PER_REV / (2 * np.pi)) / (XL330_KP_DIVISOR * XL330_PWM_LIMIT)` with `XL330_KP_DIVISOR = 256  # Empirically observed for XL330 (manual mentions 128)` and `XL330_PWM_LIMIT = 885`. That confirms the structure [F: error in encoder counts, output PWM/885 as duty] and reports a **2x lower** gain than the manual for the XL330 (divisor 256, not 128). The comment gives no data; the XL430 was not measured. BAM's XL330 identification used `vin=7.5`. The same repo's identified XL330-M288-T parameters (`m1.json`: kt 0.3706 N*m/A, R 3.452 ohm, armature 0.001609 kg*m^2, friction_base 0.01909 N*m, friction_viscous 0.003924 N*m*s/rad, command_delay 5.93 ms; `m6.json`: kt 0.3460, R 2.502, armature 0.001573, command_delay 10.2 ms) are used as cross-checks in the part file (kt agrees with the datasheet 0.354 N*m/A to 5 %, R with 3.4 ohm to the m1 fit).

## 6. DYNAMIXEL Protocol 2.0 (E4, E5) for the protocol stage

### 6.1 Physical layer [F]

"DYNAMIXEL servos utilize half-duplex 8 bit Asynchronous Serial Communication with 1 Stop bit, and no Parity." Multi-drop single DATA line; only one device drives it at a time. **Byte-to-byte time:** "The allowed delay time between bytes when sending an instruction packet is 1.5ms. If the delay time is over 1.5ms, then DYNAMIXEL actuator recognizes this as a communication problem and waits for the next header (0xff 0xff 0xfd) before continuing to process packets." Servo answers after Return Delay Time x 2 us.

### 6.2 Packet structure [F]

Instruction packet: `FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAM 1..N | CRC_L CRC_H`. Header = 0xFF 0xFF 0xFD, then a Reserved 0x00. ID 0 ~ 252 (0x00 ~ 0xFC); Broadcast ID 254 (0xFE). **Length = number of Parameters + 3** (counts INST, params, CRC); the Status Packet length also counts its 1-byte ERR. Status packet: `FF FF FD 00 | ID | LEN_L LEN_H | 0x55 | ERR | PARAM 1..N | CRC_L CRC_H`. Total packet bytes = 7 + LEN.
ERR byte: bit 7 = Alert ("When there is some hard ware issue with the Device, this field is set as 1. Checking the Hardware error status value of the Control Table can indicate the cause"), bits 6-0 = Error Number:

| Code | Name | Meaning (E4) |
|---|---|---|
| 0x01 | Result Fail | Failed to process the sent Instruction Packet |
| 0x02 | Instruction Error | An undefined Instruction has been used; Action has been used without Reg Write |
| 0x03 | CRC Error | The CRC of the sent Packet does not match the expected value |
| 0x04 | Data Range Error | Data to be written to the specified Address is outside the range of the minimum/maximum value |
| 0x05 | Data Length Error | Attempted to write Data that is shorter than the required data length of the specified Address (e.g. 2 bytes of a 4-byte register) |
| 0x06 | Data Limit Error | Data to be written to the specified Address is outside of the configured Limit value |
| 0x07 | Access Error | Attempted to write a Read Only or undefined Address; read a Write Only or undefined Address; write an EEPROM register while Torque was Enabled |
| 0x80 (bit 7) | Alert | hardware error present (see Hardware Error Status(70)) |

Also: a READ outside the control table returns an Access Error with no parameters; the Bus Watchdog error returns Data Range Error on Goal writes (section 4); Clear with the servo moving returns Result Fail.

### 6.3 Instructions [F] (code | name | parameters / behaviour)

| Code | Name | Parameters |
|---|---|---|
| 0x01 | Ping | none. Status: Param1 Model Number LSB, Param2 Model Number MSB, Param3 Firmware Version. "Regardless of the Status Return Level ... a Status Packet will always be sent in response to a ping packet." To broadcast 0xFE every device answers (in ID order). |
| 0x02 | Read | P1,P2 start address (LE), P3,P4 data length (LE). Status: the bytes. Not executed on broadcast. |
| 0x03 | Write | P1,P2 address, P3..PN data. |
| 0x04 | Reg Write | like Write but stored; sets Registered Instruction(69) = 1. |
| 0x05 | Action | none; executes the Reg Write; "Instruction Error" if nothing registered. |
| 0x06 | Factory Reset | P1 option: 0xFF reset all, 0x01 all except ID, 0x02 all except ID and baud rate. Device reboots, LED blinks 4 times; broadcast with 0xFF is NOT executed. |
| 0x08 | Reboot | none. |
| 0x10 | Clear | P1 = 0x01 then P2-P5 fixed 0x44 0x58 0x4C 0x22: reset multi-turn Present Position to within one rotation (only while stopped, else Result Fail); P1 = 0x02 with 0x45 0x52 0x43 0x4C clears errors (Y series only). |
| 0x20 | Control Table Backup | P1 = 0x01 store / 0x02 restore, P2-P5 fixed 0x43 0x54 0x52 0x4C; only with torque off (else Result Fail); restore reboots. Stores EEPROM, Velocity PI, Position PID, Feedforward, Profile Accel/Vel, Indirect Addresses. [P1 option values are read from the example packets, not stated in prose.] |
| 0x55 | Status (Return) | the status packet's INST |
| 0x82 | Sync Read | ID must be 0xFE; P1,P2 address, P3,P4 length, P5.. = list of IDs; each device returns its own status packet in list order. |
| 0x83 | Sync Write | ID 0xFE; P1,P2 address, P3,P4 length; then for each device: ID followed by its data bytes. No status packets. |
| 0x8A | Fast Sync Read | like Sync Read, one combined status packet for all devices (each device appends `ERR ID data CRC`; no byte stuffing). XL430 FW >= v45, XL330 FW >= v46. |
| 0x92 | Bulk Read | ID 0xFE; per device: ID, address LE, length LE (one entry per ID); each device returns its own status packet. |
| 0x93 | Bulk Write | ID 0xFE; per device: ID, address LE, length LE, data. |
| 0x9A | Fast Bulk Read | like Bulk Read, combined status. |

Broadcast rule [F]: "The DYNAMIXEL Broadcast ID (254 (0xFE)) will only return Status Packets for Ping, Sync Read and Bulk Read commands". Status Return Level(68) limits which instructions answer (section 4).

### 6.4 Byte stuffing [F]

"When the byte pattern 0xFF 0xFF 0xFD appears in a packet, byte stuffing adds 0xFD to the end of the pattern to convert it to 0xFF 0xFF 0xFD 0xFD to ensure that it is not interpreted as the header at the start of another packet." Transmission: build packet, apply stuffing to the payload, update LENGTH to include the stuffed bytes, compute CRC over the stuffed packet. Reception: find header 0xFF 0xFF 0xFD (ignoring stuffed 0xFF 0xFF 0xFD 0xFD), read LENGTH bytes, check CRC over the received (stuffed) bytes, then remove stuffing and execute. [I] The pattern is applied to the bytes after the 4-byte header (ID, length and later); the CRC covers header through last parameter.

### 6.5 CRC-16 [F] (E5)

"CRC-16 (IBM/ANSI) Polynomial : x16 + x15 + x2 + 1 (polynomial representation : 0x8005) Initial Value : 0". CRC covers every byte from the first header byte through the last parameter (i.e. 5 + LENGTH bytes), is sent little-endian (`CRC_L = CRC & 0xFF`, `CRC_H = (CRC >> 8) & 0xFF`). `data_blk_size = 5 + Packet Length`, `Packet Length = (LEN_H << 8) + LEN_L`. Algorithm (verbatim from E5):

```c
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}
```

Verified this session (script run on the table text itself): the 256-entry table equals the table generated from polynomial 0x8005, MSB-first, no reflection, init 0 (`c = i << 8; 8x: c = (c << 1) ^ 0x8005 if c & 0x8000 else c << 1`, 16-bit; table[1] = 0x8005, table[255] = 0x0202). CRC test vectors (bytes before the CRC -> CRC as sent, L then H); the first five are the E4 example packets, all reproduced:

| Packet (without CRC) | CRC | Sent as |
|---|---|---|
| Ping ID 1: `FF FF FD 00 01 03 00 01` | 0x4E19 | `19 4E` |
| Ping broadcast: `FF FF FD 00 FE 03 00 01` | 0x4231 | `31 42` |
| Ping status ID 1 (XM430, model 0x0406 shown in the example as 06 04, FW 0x26): `FF FF FD 00 01 07 00 55 00 06 04 26` | 0x5D65 | `65 5D` |
| Read ID 1 addr 0x0084 len 4: `FF FF FD 00 01 07 00 02 84 00 04 00` | 0x151D | `1D 15` |
| Read status (position 0xA6): `FF FF FD 00 01 08 00 55 00 A6 00 00 00` | 0xC08C | `8C C0` |
| Write ID 1 Goal Position(0x0074) = 0x00000200: `FF FF FD 00 01 09 00 03 74 00 00 02 00 00` | 0x89CA | `CA 89` |
| Reboot ID 1: `FF FF FD 00 01 03 00 08` | 0x4E2F | `2F 4E` |
| Status OK, no params: `FF FF FD 00 01 04 00 55 00` | 0x0CA1 | `A1 0C` |
| E5's example Read of 2 bytes from 0x0000: `FF FF FD 00 01 07 00 02 00 00 02 00` | 0x5121 [computed, E5 gives no value] | `21 51` |

### 6.5b Worked examples worth keeping as tests [F, E4]
Ping reply of an XM430-W210: `FF FF FD 00 01 07 00 55 00 06 04 26 65 5D` (model 0x0406 = 1030, FW 38). Write Goal Position = 512 to ID 1: instruction `FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89`, status `FF FF FD 00 01 04 00 55 00 A1 0C`. Read Present Position (132, 4 bytes) = 166: reply `FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0`. Reboot ID 1: `FF FF FD 00 01 03 00 08 2F 4E`, reply `FF FF FD 00 01 04 00 55 00 A1 0C`.

## 7. U2D2 and the cable/connectors (E6, J1)

U2D2 [F]: "Weight | 9g || Dimensions | 48mm x 18mm x 14.9mm || Ports | 3Pin TTL Level ... 4Pin RS-485 ... 4Pin UART ... || Baudrate | Maximum 6Mbps". Baud table: 1,000,000 / 2,000,000 / 3,000,000 / 4,000,000 / 6,000,000 all 0.00 % error; 57,600 -0.02 %; 115,200 +0.04 %; 4,500,000 +1.56 %. "U2D2 does not supply power to DYNAMIXEL, therefore, an external power supply should provide power". USB Type-C since August 2025 (Micro-B before); FTDI virtual COM port ("FTDI devices such as the U2D2"); "Set the Latency Timer (msec) to 1ms". The Koch BOM does **not** use a U2D2 (section 8). U2D2 Power Hub [F]: "Operating Voltage | 3.5V ~ 24.0V || Maximum Current | 10.0A"; connectors JST EHR-03/EHR-04, Molex 39-01-2020 power, DC barrel 2.5 mm ID / 5.5 mm OD, screw terminal; "Do NOT use multiple power inputs at the same time". Not modelled (not in the Koch BOM).
Robot Cable-X3P 180mm [F, store page]: "This cable is for DYNAMIXEL X-Series TTL version only"; 10 per pack; length from the product name. Connector/gauge are taken from the servo e-Manual connector tables ([I] for the cable): JST EHR-03, 21 AWG, SEH-001T-P0.6 crimp. JST EH datasheet [F]: "Current rating: 3 A AC/DC (AWG #22) || Voltage rating: 250 V AC/DC || Contact resistance: Initial value/ 10 mΩ max. After environmental tests/ 20 mΩ max. || Conductor size/ AWG #32 to AWG #22" (so the 21 AWG gauge is outside the EH wire range; the e-Manual says it anyway). 21 AWG copper: 0.410 mm^2, 42.00 mΩ/m (Wikipedia AWG table).

## 8. Koch v1.1 power architecture (K1, K2, L1, A1)

### 8.1 Follower and leader BOM and supply [F]

K1 follower BOM rows: "Dynamixel XL430-W250-T | 2", "Dynamixel XL330-M288-T | 4", "XL330 Frame and Idler Wheel 4pcs set", "XL430 Idler Wheel set", "Waveshare Serial Bus Servo Driver Board | 1", "**Voltage Reducer | 1** | $14" (US link EPLZON B09R4DBZJK), "**12V Power Supply | 1** | $12" (US link a.co/d/40o8uMN -> Amazon B0BYVK73BP, Facmogu 12V 5A), "Jumper Wires 3*40 pcs set (M-M, M-F, F-F)", "USB C-A or C-C 2pcs set". Leader BOM: "Dynamixel XL330-M077-T | 6", "Waveshare Serial Bus Servo Driver Board | 1", "5V Power Supply | 1" (no U2D2 anywhere).
K1 assembly text, verbatim: "After you assemble the two arms from the video, power the leader arm using the 5V power supply<sup>[14]</sup>, and the follower arm using the 12V power supply. In addition, plug each arm into your computer using a USB-C cable." Footnote 14: "The observant technician may realize the Serial Bus Servo Driver Board suggests an input DC voltage between 9 to 12.6V, where for the leader arm we are only applying 5V. This lower voltage will not hurt the board, and must be done to correctly power the servos on the leader arm."

### 8.2 Wiring diagram (K2, read as image; text layer confirms the labels) [F]
Follower: the 12 V supply feeds the Waveshare board ("SERVO_DRIVER_BOARD", marked B0CTMM4LWK, the same Amazon ASIN the existing `waveshare-bus-servo-adapter-a` part's SO-101 sibling cites). The board has two servo outputs, each with pins D(ATA), V, G. Output 2 -> "MOTOR_CABLE Robot Cable-X3P 180mm" with pin 2 = **12V**, pin 1 = GND, pin 3 = DATA -> **two XL430-W250-T** in a daisy chain (Plug 1 -> Plug 2, part code 902-0135-000). Output 1 -> its 12 V and GND feed a "VOLTAGE_REDUCER" (IN+/IN-, label EP-JY-DC5; OUT+ labelled **5V**, OUT- GND); the reducer's 5 V and GND plus output 1's DATA make the cable (pin 2 = **5V**, 1 = GND, 3 = DATA) that daisy-chains **four XL330-M288-T** (902-0163-000). Both servo chains therefore share the board's single half-duplex DATA bus; the supply is split by voltage. Leader: board powered from 5 V ("5V PWR"), all 6 XL330-M077-T (902-0162-000) on 5 V (pin 2 labelled 5V) with the board's V pin.
So **each follower servo's supply: shoulder_pan and shoulder_lift (XL430-W250-T) 12 V; elbow_flex, wrist_flex, wrist_roll, gripper (XL330-M288-T) 5 V (reducer output).** The reducer product title [F, A1]: "EPLZON DC 6V-36V Step Down to DC 5V-5.3V 5.2V 3.5-6A Transformer Dual Output Voltage Regulator Power Module (Pack of 2 pcs)". The 12 V supply title [F, A1]: "Facmogu 12V 5A Power Supply Adapter, AC 100-240V to DC 12V Converter | 50/60Hz | Wall Plug Adapter Transformers | 150cm/ 4.92ft Long Cable | 5.5mmx 2.5mm & 2.1mm Output Jack | For LED Strip Lights". The Waveshare board's input range is 9-12.6 V (existing part), so the 12 V follower supply is in range (the leader's 5 V is not, per footnote 14).

### 8.3 LeRobot Koch follower software facts [F, L1]
Motors: `shoulder_pan` id 1 and `shoulder_lift` id 2 = `xl430-w250`; `elbow_flex` id 3, `wrist_flex` id 4, `wrist_roll` id 5, `gripper` id 6 = `xl330-m288`. `configure()`: Return_Delay_Time 0 (`configure_motors(return_delay_time=0)`, "By default, Dynamixel motors have a 500µs delay response time (corresponding to a value of 250 ..."); Operating_Mode = EXTENDED_POSITION (4) for all but the gripper; gripper = CURRENT_POSITION (5), "to be limited by the limit of the current"; `Position_P_Gain` 1500, `Position_I_Gain` 0, `Position_D_Gain` 600 written to the `elbow_flex` servo **only**. Baud 1,000,000 (`DEFAULT_BAUDRATE`). Reads `Present_Position`, writes `Goal_Position` by sync read/write.
[F] Inconsistency (confirmed in the fetched text): LeRobot's `koch.mdx` ID-setup step says "Connect the usb cable from your computer and the 5V power supply to the follower arm's controller board" while K1 and the wiring diagram power the follower from 12 V; at 5 V the XL430 (6.5 V minimum) is below its range (Min Voltage Limit default 6.0 V; Input Voltage Error is not in the XL430 default Shutdown 52, so it still answers). The sim should use the 12 V architecture; the 5 V text only matters for one-motor ID setup.

## 9. DC-motor model derivation summary (details in the part files)

Convention as `feetech-sts3215-c001`: motor-side Ke, Kt = Ke, r_effective = V/I_stall (mean of the top two voltages), eta from the stall torque, loss law `tau_f = Kt i0 [(1 - v) tanh(w/ws) + v w/w0]` (`docs/models.md`) with i0 at the supply voltage the Koch uses and w0 the datasheet no-load motor speed at that voltage.
No servo publishes a no-load current, so the two-point i0_visc recipe of the STS3215 is impossible. i0 is an **estimate** (0.06 x stall current, the STS3215 ratio), ke is refitted with it (`(V - I0 R)/(N w)`), and i0_visc = 0.69 is borrowed from BAM's XL330-M288 friction (viscous 0.0039 N*m*s/rad x 10.8 rad/s = 0.042 N*m vs Coulomb 0.019 N*m at the 5 V no-load output speed) [estimated]. BAM's absolute friction implies i0 ~ 0.13-0.15 A at 5 V (vs 0.088 A used): the low end.
Per servo: XL430 (12 V): r_eff 8.55 ohm, ke 6.8e-3, eta 0.650, i0 0.084 A, w0 1651 rad/s, Kv 1405 rpm/V, model stall torque 1.507 N*m (datasheet 1.5). XL330-M288 (5 V): 3.42 ohm, 1.515e-3, 0.851, 0.088 A, 3110.7 rad/s, 6305 rpm/V, 0.511 N*m (0.52). XL330-M077: 3.42 ohm, 1.518e-3, eta **1.0 (clamped)**, 0.088 A, 3108 rad/s, model stall 0.162 N*m vs 0.215 published.
**M077 inconsistency** [derived, confirmed by arithmetic]: same motor as M288 (equal motor-side no-load speed, equal stall current) but 77.5:1 stall torque 0.215 N*m at 1.47 A needs eta = 0.215/(77.5 x 1.512e-3 x (1.47 - 0.088)) = 1.33 (1.108 even with I0 neglected, as in `parts/actuators/robotis-dynamixel-xl330-m077`). The schema rejects eta > 1, so the part clamps to 1.0 and keeps `gear_efficiency_raw`. The leader arm is hand-driven, so this matters little.
Rotor inertia: XL330 from BAM's identified armature 0.0015732 kg*m^2 (reflected at the output) / 288.4^2 = 1.89e-8 kg*m^2; XL430 estimated by mass scaling (1.3e-7). No published value (inertia download unreachable).
Firmware rate: not published; 1 kHz **estimated** (Realtime Tick 1 ms; MCU 64-72 MHz). This number only has to be an integer number of sim steps; D and I are insensitive to it if the sampling-time statement of section 5.2 holds (they are per-step in the firmware law).

## 10. Gain scale recommendation for the napl servo device

Device law (`src/mujoco/devices/servo.cpp`, `docs/models.md` servo row): per firmware step `u = (p e + i (integ + e) + d (e - e_prev)) / pid_scale`, optional `d_scale > 0`: `u = (p e + i (integ + e)) / pid_scale + d (e - e_prev) / d_scale`; e in encoder counts (4096/rev), `cmd = clamp(u, +-max_torque/1000)` where `max_torque` is the duty limit in 0.1 %. There is **no i_scale** in the schema.

### 10.1 Chain
1. [F] Firmware: `PWM = KPP e + KPI sum(e) + KPD (e - e_prev)` (structure per section 5.2 [I]), `KPP = P/128`, `KPI = I/65,536`, `KPD = D/16`; PWM 885 = 100 % duty; PWM Limit(36)/Goal PWM(100) cap it.
2. [I] e is in encoder pulses (0.088 deg) and the PID output is in PWM units. Basis: the e-Manual says "PWM output" and lists every position quantity in pulses; BAM's measured XL330 error_gain is built from counts/rad and 885 (section 5.3); the manual does not state the error unit.
3. duty = PWM/885, so for the P term: `duty = (P/128) e / 885 = P e / 113,280` -> **pid_scale = 128 x 885 = 113,280**.
4. D term: `duty_D = (D/16) de / 885 = D de / 14,160` -> **d_scale = 16 x 885 = 14,160** (8x smaller than pid_scale because KPD = D/16 vs KPP = P/128: per register unit D weighs 8x more than P). This matches the device's optional `d_scale` slot exactly; D is entered as the register value.
5. I term: `duty_I = (I/65,536) sum(e) / 885`; the device computes `i sum(e) / pid_scale`, so device `i = I_reg x pid_scale / (65,536 x 885) = I_reg / 512` for pid_scale 113,280. Convert in the port/assembly (the protocol stage writes registers, so on a Goal/gain register write set `i = I_reg x pid_scale / 57,999,360`); no schema change needed. With the shipped defaults I = 0 the term is inert (also 0 in LeRobot's Koch writes). `i_per_reg_i` in each part file carries the factor.
6. PWM Limit: `max_torque = 1000 x PWM_Limit / 885` (default 885 -> 1000); `torque_limit` follows Goal PWM in PWM Control Mode.
7. Time base [F/I]: the sim `d (e - e_prev)` is per **firmware step**; D and I constants were defined per real control period (older manual). So the recommendation assumes sim firmware period = real period. If the real rate Fr differs from the sim `firmware_hz` Fs, then scale `d_scale` by `Fr/Fs`... precisely: D acts per step in both, so `d_scale_sim = d_scale x (Ts_sim / Ts_real)` and `i_sim = i x (Ts_real / Ts_sim)`. With both at 1 kHz (the estimated rate) these factors are 1.

### 10.2 Numbers at the registers' defaults / LeRobot values (arithmetic on the above)
| Servo | P | duty per count | full duty at | static stiffness K = (P/pid_scale) tau_stall 651.9 |
|---|---|---|---|---|
| XL430 @12 V | 640 | 5.65e-3 | 177 counts (15.5 deg) | 5.5 N*m/rad (tau 1.5) |
| XL330-M288 @5 V | 400 | 3.53e-3 | 283 counts (24.9 deg) | 1.20 N*m/rad (tau 0.52) |
| XL330-M288 @5 V, LeRobot elbow | 1500 | 1.32e-2 | 76 counts (6.6 deg) | 4.5 N*m/rad |
D/P damping time per register unit ratio: XL430 default D 4000 / P 640 -> (4000/14,160)/(640/113,280) = 50 steps; XL330-M288 LeRobot elbow 600 / 1500 -> 3.2 steps; XL330-M288 default D = 0.

### 10.3 Confidence
Documented: PWM 885 = 100 %, the three divisors, PID -> PWM -> inverter, sampling time inside the constants (older manual). Inferred: encoder-pulse error unit; same structure on all three; no extra scaling between PID output and PWM. **Disputed by one measurement**: BAM's empirical XL330 divisor 256 (not 128) halves every gain: pid_scale 226,560, d_scale 28,320 (if D scales likewise; BAM measured only the P gain), i = I/256. The disagreement is unresolved: BAM provides no data in the comment, the e-Manual is silent on units. Recommended default: the documented 113,280 (/14,160, /512) for all three servos; treat 113,280-226,560 as the plausible band for the XL330 and expose `pid_scale` as the calibration knob (as for the STS3215). A hold-torque-vs-position-error measurement on a real servo would settle it. Mode 5 (gripper): the PID output there is a current command and Goal Current limits torque; the sim servo has no current mode, so the gripper needs either the PWM-limited approximation `max_torque = 1000 x Goal_Current / I_stall(V)` or a device extension (not built here).

## 11. Not sourced / gaps

- Rotor inertia of all three servos (estimated); no-load current of all three (estimated); loss split i0_visc (borrowed); control-loop rate (estimated 1 kHz); overload/electrical-shock thresholds and In-Position window; XL430 e-Manual stall-test Kt at 9 V is quoted with 1-digit current.
- Servo horn spline and mounting hole patterns (drawings are PDF/STEP only; parallel stage S2/D derives ports from Koch CAD).
- Which reducer the Koch BOM product actually is (the diagram says EP-JY-DC5; the BOM links EPLZON B09R4DBZJK) and its real regulation; Facmogu 12 V output at no load.
- The leader arm's 5 V supply product (BOM short links not resolved): no part created.
- Connector mismatch to watch in the DRC: `waveshare-bus-servo-adapter-a` declares its servo port `5264-3p` (2.54 mm) with signal type `feetech-ttl`, the Dynamixel parts use `jst-eh.3` (2.5 mm EH) and `dynamixel-ttl`. The Koch build bridges them with the BOM's jumper wires; an assembly will need a pinned `connectors` entry or a bridge part.
