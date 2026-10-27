# cuas2 wiring (pin level)

Source of truth: the `[[wires]]` of `assemblies/cuas2.toml` (153 wires; v1 `assemblies/cuas-lite.toml` has 72). Every wire below is one named wire there; the DRC (`napl.design.check`) types, power and pin rules are green (0 findings).
Pico pin names are physical pins `J.n` of the Pico H (GP number in brackets). Part pin ids are the library ids (`U.16` = IC pin 16). IC pin-outs of the 74HCT123 and 74HCT4017 and the ESC, relay and IR-diode connectors are
**from memory / vendor pages and marked estimated in the part files**: check against the datasheet of the parts you buy before powering (bench item).

## 1. Safety architecture (what the wiring guarantees without firmware)

```
XT60 -> 20 A fuse -> 12 V bus ---+-- Jetson barrel, TMC2209 VM x2
                                 +-- KEY (K.1 -> K.2) -> E-STOP NC -> relay coil + -> coil - -> IRLB8721 drain (source = GND)
                                 |                      KEYED12 (key output) also -> L7805 -> 5 V logic (supervisor, limiter, counter, head halls)
                                 +-- relay contact 30 (SW.1) -> contact 87 (SW.2) -> ESC top BAT, ESC bottom BAT, LM2596 IN
```

Weapon power exists only while: key on AND E-stop not pressed (coil circuit) AND the supervisor 74HCT123 sees the Pico KICK (GP28) toggle (gate of the IRLB8721) AND the shot counter has not tripped (2N3904 clamp pulls the gate low).
Pressing E-stop or turning the key off opens the coil circuit directly; stopping the Pico kicks or counting N balls closes the MOSFET. The counter is reset only by power-cycling the key (power-on-reset). The pusher servo is powered through a
P-FET window from a second 74HCT123 (limiter), so one FEED_REQ pulse can move the pusher for at most 254 ms and a second one within 338 ms is ignored. Human ARM, dead-man and the sector inhibit stay in firmware (spec 6).

## 2. Pico (spec 6.5) pin table

| Pico pin    | GP          | Function                             | Wired to                                                                        | Wire(s)                              | v1                        |
| ----------- | ----------- | ------------------------------------ | ------------------------------------------------------------------------------- | ------------------------------------ | ------------------------- |
| J.1         | GP0         | tilt TMC UART TX                     | r_su_tilt (1 k) -> tmc_tilt CTL.4                                               | tilt_uart_tx, tilt_uart_line         | kept                      |
| J.2         | GP1         | tilt TMC UART RX                     | r_su_tilt.ser_b                                                                 | tilt_uart_rx                         | kept                      |
| J.4 / J.5   | GP2 / GP3   | pan STEP / DIR                       | tmc_pan CTL.7 / CTL.8 (J.3 GND to PWR.8)                                        | pan_step_dir                         | kept                      |
| J.6         | GP4         | EN, shared pan + tilt                | r_en 10 k pull-up; tmc_pan CTL.1 and tmc_tilt CTL.1                             | en_a, pan_en_b, tilt_en_b            | merged (v1 GP4 + GP7)     |
| J.7 / J.9   | GP5 / GP6   | tilt STEP / DIR                      | tmc_tilt CTL.7 / CTL.8 (J.8 GND to PWR.8)                                       | tilt_step_dir                        | kept                      |
| J.10        | GP7         | BM_BREECH (phototransistor node)     | r_pt_br 10 k pull-up / pt_br collector                                          | bm_breech_gp7                        | was tilt EN               |
| J.11        | GP8         | pan hall                             | r_hp 10 k pull-up, hall_pan OUT                                                 | hall_pan_gp8                         | kept                      |
| J.12        | GP9         | tilt hall                            | r_ht 10 k pull-up, hall_tilt OUT                                                | hall_tilt_gp9                        | kept                      |
| J.14        | GP10        | E-stop sense                         | divider r_et 33 k / r_eb 10 k from E-stop NC.2                                  | sense_estop_*                        | kept                      |
| J.15        | GP11        | key sense                            | divider r_kt 33 k / r_kb 10 k from key K.2                                      | sense_key_*                          | kept                      |
| J.16        | GP12        | DSHOT_TOP                            | esc_t SIG.1 (J.18 GND to SIG.2)                                                 | dshot_top                            | was flywheel PWM          |
| J.17        | GP13        | DSHOT_BOT                            | esc_b SIG.1 (J.18 GND to SIG.2)                                                 | dshot_bot                            | was tach (unwired)        |
| J.19        | GP14        | PUSH_SERVO                           | servo_push SRV.3                                                                | push_servo_sig                       | kept (was the only servo) |
| J.20        | GP15        | status WS2812 DIN                    | led_status L.4 (V from the 3V3 chain)                                           | led_status_din                       | was red LED               |
| J.21        | GP16        | BM_MUZ (muzzle phototransistor node) | r_pt_mz 10 k pull-up / pt_mz collector (same node feeds the counter CLK)        | bm_muz_gp16                          | was green LED             |
| J.22        | GP17        | buzzer                               | not wired (no part, no effect)                                                  |                                      | unchanged                 |
| J.24        | GP18        | GATE_SERVO                           | servo_gate SRV.3                                                                | gate_servo_sig                       | free                      |
| J.25        | GP19        | FEED_REQ (to limiter)                | ic_lim U.2 (1B), U.10 (2B), r_fr 10 k pull-down                                 | feed_req_b, feed_req_b2, feed_req_pd | free                      |
| J.26 / J.27 | GP20 / GP21 | pan TMC UART TX / RX                 | r_su_pan (1 k) -> tmc_pan CTL.4                                                 | pan_uart_*                           | kept                      |
| J.29        | GP22        | DIAG                                 | not wired (BTT module has no DIAG pin)                                          |                                      | unchanged                 |
| J.31        | GP26        | GATE_HOME (A3144)                    | r_hg 10 k pull-up, hall_gate OUT                                                | hall_gate_gp26                       | free                      |
| J.32        | GP27        | PUSH_HOME (A3144)                    | r_hpu 10 k pull-up, hall_push OUT                                               | hall_push_gp27                       | free                      |
| J.34        | GP28        | KICK (100 Hz toggle)                 | ic_sup U.2 (1B)                                                                 | kick_b                               | free                      |
| J.36        | 3V3         | pull-up and LED rail chain           | r_hp, r_ht, r_hg, r_hpu, r_en, r_ms_pan, r_ms_tilt, r_led__, r_pt__, status LED | v3_*                                 | partly kept               |
| J.38 / J.39 | GND / VSYS  | 5 V from the Jetson header           | jetson J12.2/J12.6 to J.39 / J.38                                               | jetson_5v_pico                       | kept                      |
| USB         |             | to Jetson                            | pico.usb to jetson J7                                                           | usb_pico_jetson                      | kept                      |

The 3V3 pin takes one wire, so the 3V3 consumers are chained through the rail pin of each resistor (`v3_*`, rail ports of the resistors). GND is J.13 (sense, supervisor, counter returns), J.18 (beams, ESC signal returns), J.23 (feed-request pull-down, status LED) and J.3/J.8 (TMC returns).

## 3. Harnesses

### 3.1 Power (12 V bus)

| Wire                                                   | From                   | To                               | Gauge |
| ------------------------------------------------------ | ---------------------- | -------------------------------- | ----- |
| psu_cord                                               | psu OUT.1/2            | xt60 WF.1/2                      | 18    |
| xt60_fuse                                              | xt60 WM.1              | fuse F.1                         | 18    |
| fuse_jetson, gnd_jetson                                | fuse F.2 / xt60 WM.2   | jetson J16.1 / J16.2             | 18    |
| fuse_tmc_pan, gnd_tmc_pan, fuse_tmc_tilt, gnd_tmc_tilt | fuse F.2 / xt60 WM.2   | tmc_pan / tmc_tilt PWR.1 / PWR.2 | 18    |
| fuse_relay                                             | fuse F.2               | relay contact SW.1 (terminal 30) | 18    |
| relay_esc_top, relay_esc_bot                           | relay SW.2 (87)        | esc_t, esc_b BAT.1               | 18    |
| gnd_esc_top, gnd_esc_bot                               | esc BAT.2              | xt60 WM.2                        | 18    |
| relay_buck, gnd_buck                                   | relay SW.2 / xt60 WM.2 | buck IO.1 / IO.2                 | 22    |

The fuse is 20 A (spec 9): two ESCs at 30 A each are limited by the motors (2205 2300 KV at 12 V, an estimated 8 A each at full throttle, to be measured on the bench).

### 3.2 Relay coil, key, E-stop (series, hardware only)

| Wire             | From                                              | To                                   |
| ---------------- | ------------------------------------------------- | ------------------------------------ |
| fuse_key         | fuse F.2                                          | key K.1                              |
| key_estop        | key K.2                                           | estop NC.1                           |
| estop_coil       | estop NC.2                                        | relay COIL.1 (terminal 86)           |
| coil_drain       | relay COIL.2 (85)                                 | q_sup Q.2 (drain)                    |
| q_sup_source     | q_sup Q.3                                         | xt60 WM.2 (GND)                      |
| fly_d            | d_fly 1N5822 (cathode to COIL.1, anode to COIL.2) | across the coil                      |
| key_reg, gnd_reg | key K.2 / xt60 WM.2                               | reg REG.1 / REG.2 (L7805 in and GND) |

The key and E-stop carry the coil current only (an estimated 0.15 to 0.25 A for a 40 A automotive relay coil, to be measured: the E-switch KO106A102 is rated 1 A). The E-stop is wired NC.

### 3.3 Supervisor (74HCT123 `ic_sup`, one half used)

| Wire                                            | From                           | To                                                      | Note                                                                   |
| ----------------------------------------------- | ------------------------------ | ------------------------------------------------------- | ---------------------------------------------------------------------- |
| kick_b                                          | Pico J.34 (GP28)               | U.2 (1B)                                                | rising edge triggers                                                   |
| sup_low_1a, 2a, 2b, 2clr                        | r_sup_dn.node_dn (10 k to GND) | U.1 (1A), U.9, U.10, U.11                               | unused inputs and the unused half held low (its CLR low keeps it idle) |
| sup_clr_rail, sup_clr                           | reg 5 V via r_sup_up 10 k      | U.3 (1CLR)                                              | high                                                                   |
| sup_rext, sup_rext_pin, sup_c_plus, sup_c_minus | 5 V via r_sup 100 k            | U.15 (1REXT/CEXT), C 3.3 u + on U.15, - on U.14 (1CEXT) | tW = 0.45 x 100 k x 3.3 u = 149 ms                                     |
| sup_out, sup_gate                               | U.13 (1Q)                      | r_gate 1 k to q_sup Q.1 (gate)                          |                                                                        |
| sup_gate_pd                                     | r_gpd 100 k                    | q_sup gate to GND                                       | relay stays off if the IC is not powered                               |
| reg_power_*                                     | reg 5 V                        | all IC pin 16, pin 8 to GND                             |                                                                        |

Kick period 10 ms (100 Hz) against 149 ms: about 14 missed kicks are tolerated; hardware stops the relay 149 ms after the last kick.

### 3.4 Shot counter (two 74HCT4017, N = 10 balls per key session)

| Wire                                                   | From                                                    | To                                                 | Note                                            |
| ------------------------------------------------------ | ------------------------------------------------------- | -------------------------------------------------- | ----------------------------------------------- |
| cnt_clk                                                | muzzle PT node (r_pt_mz.node_up)                        | cnt_u U.14 (CLK)                                   | rising edge = ball blocks the beam; shares GP16 |
| cnt_ce_pd, cnt_ce_pd_gnd                               | r_ce 10 k to GND                                        | cnt_u U.13 (/CE)                                   | units always enabled                            |
| cnt_carry                                              | cnt_u U.12 (CARRY)                                      | cnt_t U.14 (CLK)                                   | CARRY rises on wrap 9 to 0 (the 10th pulse)     |
| cnt_trip_ce                                            | cnt_t U.2 (Q1)                                          | cnt_t U.13 (/CE)                                   | latches the tens stage at the trip              |
| cnt_trip_base, cnt_clamp_b                             | cnt_t U.2                                               | r_cb 10 k to q_clamp base T.2                      |                                                 |
| cnt_clamp_c, cnt_clamp_e                               | q_clamp collector T.3 to q_sup gate; emitter T.1 to GND |                                                    | trip pulls the supervisor gate low: relay open  |
| por_rail, por_reset_u, por_reset_t, por_pd, por_pd_gnd | 5 V via c_por 1 u                                       | cnt_u U.15, cnt_t U.15 (RESET); r_por 100 k to GND | reset pulse at power-up only                    |

Unit stage Q-outputs are left open. Default N = 10 is tens Q1; moving the tens trip wire (`cnt_trip_ce` / `cnt_trip_base`) from pin 2 to pin 4 (Q2) or pin 7 (Q3) by jumper gives N = 20 or 30. Reset happens only on a key power-cycle
(5 V logic is from the key output), so the user must re-arm the weapon with the key. The counter counts physical balls through the muzzle, independent of firmware.

### 3.5 Feed limiter (74HCT123 `ic_lim`, both halves)

| Wire                                 | From                    | To                                           | Note                                              |
| ------------------------------------ | ----------------------- | -------------------------------------------- | ------------------------------------------------- |
| feed_req_b, feed_req_b2, feed_req_pd | Pico J.25 (GP19)        | U.2 (1B), U.10 (2B); r_fr 10 k to GND        | both halves triggered together                    |
| lim_lockout_1a, lim_lockout_2a       | U.5 (2Q)                | U.1 (1A), U.9 (2A)                           | 2Q high blocks both triggers during the lockout   |
| lim_clr_*                            | 5 V via r_clr_lim 10 k  | U.3 (1CLR), U.11 (2CLR)                      |                                                   |
| lim_rext_on, lim_c_on                | 5 V via r_on 120 k      | U.15, c_on 4.7 u to U.14                     | tW = 0.45 x 120 k x 4.7 u = 254 ms (power window) |
| lim_rext_hold, lim_c_hold            | 5 V via r_hold 75 k     | U.7 (2REXT/CEXT), c_hold 10 u to U.6 (2CEXT) | tW = 0.45 x 75 k x 10 u = 338 ms (lockout)        |
| lim_out_gate                         | U.4 (1Qbar)             | q_p Q.1 (IRF9540N gate)                      | low during the window: P-FET on                   |
| lim_pg_rail, lim_pg_gate             | buck 5 V via r_pg 100 k | q_p gate                                     | pull-up: off if the IC dies                       |
| lim_pfet_source, push_supply         | buck vout               | q_p source Q.3; drain Q.2 to servo_push V+   | pusher is powered only inside the window          |

### 3.6 Flywheels

| Wire                             | From                          | To                                                                      |
| -------------------------------- | ----------------------------- | ----------------------------------------------------------------------- |
| dshot_top, dshot_bot             | Pico J.16 / J.17 (+ J.18 GND) | esc_t, esc_b SIG.1 / SIG.2                                              |
| relay_esc__, gnd_esc__           | see 3.1                       | esc BAT                                                                 |
| motor_top_leads, motor_bot_leads | esc MOT.1-3                   | motor_t, motor_b PH.1-3 (22 AWG, three phases, swap any two to reverse) |

Top and bottom wheels spin in opposite directions: set rotation at the ESC (swap two phases of one motor). Bluejay (GPL) is only documented, no firmware is vendored.

### 3.7 Beams, halls, status, servos, USB

| Wire                                                         | From                                     | To                                                                                                          |
| ------------------------------------------------------------ | ---------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| v3_led_*, led_mz_a/k, led_br_a/k                             | 3V3 chain via r_led_mz, r_led_br (100 R) | IR LEDs anode D.1, cathode D.2 to J.18                                                                      |
| v3_pt__, pt___c, pt_*_e                                      | 3V3 chain via r_pt_mz, r_pt_br (10 k)    | phototransistor collector T.1, emitter T.2 to J.18; node to GP16 / GP7 (and the counter CLK for the muzzle) |
| reg_5v_hall_gate, hall_gate_to_push_power                    | reg 5 V                                  | hall_gate, hall_push H.1/H.2 (A3144 needs 4.5 V; OUT pulled up to 3V3 by r_hg, r_hpu)                       |
| jetson_5v_hall_pan, hall_pan_to_tilt_power                   | Jetson 5 V header                        | hall_pan, hall_tilt                                                                                         |
| hall_gate_out, hall_gate_gp26; hall_push_out, hall_push_gp27 | hall OUT H.3                             | pull-up r_hg, r_hpu to 3V3, then J.31 / J.32                                                                |
| rail_servo_gate                                              | buck vout IO.3 / IO.4                    | servo_gate SRV.2 / SRV.1 (5 V, always on)                                                                   |
| push_gnd                                                     | buck IO.4                                | servo_push SRV.1 (V+ through the P-FET window)                                                              |
| led_status_power, led_status_gnd, led_status_din             | 3V3 chain / J.23 / J.20 (GP15)           | WS2812B L.1 / L.3 / L.4 (run from 3.3 V: below the 3.5 V datasheet minimum, bench item B-LED)               |
| usb_pico_jetson, usb_brio_jetson, jetson_5v_pico             | USB data + 5 V                           | Jetson J7 / J6 / J12                                                                                        |

## 4. Passives (30 resistors in the wiring model + 4 capacitors)

| Id                      | Value        | Role                                          |
| ----------------------- | ------------ | --------------------------------------------- |
| r_en                    | 10 k         | EN pull-up (shared by both TMCs)              |
| r_ms_pan, r_ms_tilt     | 1 k          | MS1/MS2 straps to 3V3 (VIO)                   |
| r_su_pan, r_su_tilt     | 1 k          | TMC UART single-wire series                   |
| r_hp, r_ht, r_hg, r_hpu | 10 k         | hall outputs pull-up to 3V3                   |
| r_kt, r_kb, r_et, r_eb  | 33 k, 10 k   | key / E-stop sense dividers (12 V to 2.8 V)   |
| r_led_mz, r_led_br      | 100 R        | IR LED current limit (about 15 mA from 3.3 V) |
| r_pt_mz, r_pt_br        | 10 k         | phototransistor pull-ups                      |
| r_fr                    | 10 k         | FEED_REQ pull-down                            |
| r_on, c_on              | 120 k, 4.7 u | limiter power window 254 ms                   |
| r_hold, c_hold          | 75 k, 10 u   | limiter lockout 338 ms                        |
| r_clr_lim, r_sup_up     | 10 k         | CLR pull-ups to 5 V                           |
| r_sup, c_sup            | 100 k, 3.3 u | supervisor 149 ms                             |
| r_sup_dn                | 10 k         | supervisor unused inputs low                  |
| r_gate, r_gpd           | 1 k, 100 k   | relay MOSFET gate series, pull-down           |
| r_cb                    | 10 k         | clamp transistor base                         |
| r_pg                    | 100 k        | P-FET gate pull-up                            |
| r_por, c_por            | 100 k, 1 u   | counter power-on reset                        |
| r_ce                    | 10 k         | counter units /CE low                         |

(The table groups pairs; the 30 resistors match BOM line 29.)

## 5. IC pin-outs used (DIP-16)

74HCT123: 1A 1, 1B 2, 1CLR 3, 1Qbar 4, 2Q 5, 2CEXT 6, 2REXT/CEXT 7, GND 8, 2A 9, 2B 10, 2CLR 11, 2Qbar 12, 1Q 13, 1CEXT 14, 1REXT/CEXT 15, VCC 16. Pulse width tW = 0.45 R C (R in ohm, C in farad; the 0.45 constant is the 74HC123 datasheet approximation, to be verified on the bench).
74HCT4017: Q5 1, Q1 2, Q0 3, Q2 4, Q6 5, Q7 6, Q3 7, GND 8, Q8 9, Q4 10, Q9 11, CARRY OUT 12, /CE 13, CLK 14, RESET 15, VCC 16.

## 6. Wire-list diff vs v1 (`assemblies/cuas-lite.toml`, 72 wires -> 153)

Common wires: 53 (50 byte-identical, 3 changed endpoint: `pan_en_b`, `tilt_en_b`, `v3_ms_pan` now use the shared `r_en`).

Removed (19 wires, parts gone: two IRLB8721 flywheel switches `q1`/`q2`, two flyback diodes `d1`/`d2`, two brushed 180 motors, the Songle relay (replaced by a 40 A automotive relay), the single SG92R `servo` (now `servo_gate` and `servo_push`), `r_en_pan` and `r_en_tilt` (merged to one `r_en`)):
`coil_gnd`, `fb_q1`, `fb_q2`, `fw_neg_drain`, `fw_pos_drain`, `gate_pulldown_gnd`, `gate_q1`, `gate_q2`, `gp12_gate`, `gp14_servo`, `pan_en_a`, `q1_source`, `q2_source`, `rail_fw_neg`, `rail_fw_pos`, `rail_servo`, `tilt_en_a`, `v3_en_pan`, `v3_en_tilt`.

Added (100 wires): ESC power and signal (`relay_esc_*`, `gnd_esc_*`, `dshot_*`, `motor_*_leads`), relay chain (`coil_drain`, `q_sup_source`, `fly_d`, `key_reg`, `gnd_reg`), supervisor (`kick_b`, `sup_*`, `reg_power_*`), shot counter (`cnt_*`, `por_*`), feed limiter (`feed_req_*`, `lim_*`), pusher power window (`push_*`), gate servo (`gate_servo_sig`, `rail_servo_gate`), beams (`v3_led_*`, `v3_pt_*`, `led_*`, `pt_*`, `bm_*`), head halls (`hall_gate_*`, `hall_push_*`, `reg_5v_hall_gate`), status LED (`led_status_*`), shared EN (`en_a`, `v3_en`, `v3_hg`, `v3_hpu`).

Parts: 13 new library part ids (brushless motor, ESC, 40 A relay, L7805, IRF9540N, 2N3904, 74HCT123, 74HCT4017, IR LED, IR phototransistor, WS2812B, two capacitor variants) and 19 printed parts; v1 parts gone: brushed 180 motor, Songle relay, 6807ZZ and its rod, the printed elbow/pan hopper.
