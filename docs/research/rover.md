# Differential-drive rover on a RoboClaw (stage A4)

Accessed 2026-10-07. Labels: **[F]** documented fact from a fetched source, **[I]** inference (basis stated), **[E]** estimate (no source; flagged in the part file as `fidelity = "estimated"`). Part files: `parts/actuators/pololu-37d-19-1-12v-4751.toml`, `parts/electronics/basicmicro-roboclaw-2x7a-3682.toml`, `parts/structure/pololu-{wheel-90x10mm-1436, hub-6mm-4-40-1083, 37d-bracket-machined-1995, ball-caster-3-4in-metal-955}.toml`, `parts/structure/rover-chassis-plate-custom.toml`, `parts/propulsion/gensace-2200mah-3s-25c-gea223s25t3gt.toml`. Assembly `assemblies/rover.toml`; port `python/napl/roboclaw.py`; example `examples/rover/`; tests `tests/python/test_roboclaw.py`, `tests/python/test_rover.py`.

## 1. What it is

Third robot class: a ground rover. Two Pololu 37D 19:1 gearmotors with 64 CPR encoders (item 4751, 12 V) on machined brackets, 90x10 mm wheels on 6 mm hubs, a 3/4 inch ball caster, a Basicmicro RoboClaw 2x7A motor controller and a 3S 2200 mAh LiPo on a 220x190x3 mm aluminium plate (1.0445 kg). It is driven over a REAL controller protocol: the RoboClaw Packet Serial wire format through a pty, and the unmodified official `basicmicro` Python library (PyPI `basicmicro` 2.0.10, MIT, dependency pyserial; extra `rover`) talks to it. Note [F]: the LICENSE file of that package still carries the template line "Copyright (c) 2025 [Your Name or Organization]" above the MIT text.

## 2. Parts and sources

| Part | Source (fetched 2026-10-07) | What is taken |
|---|---|---|
| Pololu 4751 gearmotor | https://www.pololu.com/product/4751/specs (S1), https://www.pololu.com/product/4751 (S2) | 12 V / 6 V no-load, stall, max-efficiency points, 18.75:1, 1200 counts/rev, 200 g, 16 mm D shaft, 20 cm leads. Load limits (10 kg cm continuous, 25 kg cm instantaneous), mounting holes: product page text |
| Basicmicro RoboClaw 2x7A (Pololu 3682) | https://www.pololu.com/product/3682/specs (S1), data sheet v2.4 https://core-electronics.com.au/attachments/localcontent/roboclaw_datasheet_2x7A_42292ec1521.pdf (S2), manual rev 5.5 https://www.electrokit.com/upload/product/41015/41015436/roboclaw_user_manual.pdf (S3) | 6-34 V, 7.5 A / 15 A per channel, 8 mOhm, 32 bit counters, 10 ms packet timeout, command set, defaults QPPS 44000 / P 0x10000 / I 0x8000 / D 0x4000 |
| Pololu wheel 90x10 mm (1436) | https://www.pololu.com/product/1436/specs | diameter, width, mass |
| Pololu universal hub 6 mm (1083) | https://www.pololu.com/product/1083/specs | bore, mass |
| Pololu 37D machined bracket (1995) | https://www.pololu.com/product/1995/specs | mounting; the bracket's axis height is [E] (its drawing is an image), chosen so the wheel and the caster ball touch the floor together |
| Pololu ball caster 3/4 in (955) | https://www.pololu.com/product/955/specs | 19.05 mm ball, 0.8 in spacer stack, 30 g |
| Gens Ace 2200 mAh 3S 25C | https://rcdrone.top/products/src-gea223s25t3gt-gens-ace-2200mah-3s-11-1v-25c-g-tech-lipo-battery-pack-with-ec3-deans-and-xt60-adapter-for-rc-plane | 2.2 Ah, 3S, 25C, 169 g; internal resistance [E] |
| custom plate | aluminium 6061 density as `parts/common/aluminium-6061.toml` (Quickparts / MatWeb sheet) | 3 mm x 220 x 190 mm, a design choice |
| `basicmicro` library | https://github.com/basicmicro/basicmicro_python (v2.0.10, cloned and read; PyPI METADATA MIT) | the byte layout the port was checked against (section 4) |

## 3. Motor and controller model

The motor is the existing `servo` device (DC motor + gearbox + encoder + H-bridge, `src/mujoco/devices/devices.cpp`, `drive_motor`) with two new firmware modes in `servo.cpp` (mode 0 = the old position firmware, unchanged and still the default):

- mode 1, duty: `cmd = cmd + clamp(goal_duty - cmd, +-duty_accel*Ts)` (no clamp on the step when `duty_accel` = 0), limited to +-min(max_torque, torque_limit)/1000.
- mode 2, velocity PID: `vsp += clamp(goal_velocity - vsp, +-vel_accel*Ts)`, `e = vsp - vmeas`, `duty = (vsp + vel_p*e + vel_i*Int(e) + vel_d*de/Ts) / qpps` (feed-forward `vsp/qpps` plus PID on the encoder speed), anti-windup by not integrating while the duty is saturated and still growing. `vmeas` is the encoder speed `(counts_k - counts_k-1)/Ts` through a first-order filter `tau = 20 ms`. The loop runs at `firmware_hz` = 333.3 Hz.
- the motor law is unchanged: `Kt = Ke`, `R(T)`, no-load friction `i0` split into a coulomb and a viscous part, gearbox efficiency `eta`, thermal state.

**Estimated (not documented), flagged [E]**: the velocity-loop law, its units (the manual publishes only the default constants and "QPPS is the speed of the encoder when the motor is at 100 % power"), the 333 Hz loop rate (command 30 reads "the pulses counted in that last 300th of a second"), the 20 ms speed filter. Consequence [I]: the manual's default gains (P 1, I 0.5, D 0.25 in /65536 units) give a noisy duty on this plant (D acts on a speed quantised to 333 counts/s per step); `examples/rover/square.py` sets P 1, I 4, D 0. A real RoboClaw's loop will differ in detail.

Motor parameters (all in `parts/actuators/pololu-37d-19-1-12v-4751.toml`, derivations in the file): `R = 12/5.5 = 2.182 ohm` (datasheet stall values are theoretical extrapolations [F]), `Ke = Kt = (12 - 0.2 R)/(18.75 * 55.501 rad/s) = 0.011112 V s/rad` (equivalent 859.4 rpm/V at the motor), `eta = tau_stall/(N Kt (I_stall - I0)) = 0.755`, no-load friction torque 2.22e-3 N m at the motor shaft with viscous share 0.51 (from the two published no-load points, 12 V and 6 V), rotor inertia 2.5e-6 kg m^2 [E], `c_th` 36 J/K and `r_th` 6.3 K/W [E] (scaled from the STS3215 estimates). Encoder 1200 counts per wheel turn.

Electrical wiring in the assembly: battery -> RoboClaw main terminal (14 AWG, the pack's own lead); the two wheel `servo` devices sit on the controller's power node with `motor` = the motor instance of their channel. Not in the loop: the 8 mOhm FET resistance and the 20 cm motor leads (about 0.4 % of the 2.18 ohm winding together), the RoboClaw's current limit (command 133 / 134 is stored and reported in the status, never applied), the 5 V encoder supply, the logic battery. Caster: the ball is a fixed sphere in the model; its friction coefficient 0.01 stands in for a rolling ball's rolling resistance [E] (0.1, a sliding-like value, added 13 % to the no-load current: 0.228 A against 0.202 A with 0.01).

## 4. RoboClaw Packet Serial (what the port implements)

Fetched from the manual rev 5.5 [F]: address 0x80-0x87; a write is `[addr, cmd, data..., CRC16]` and is answered with 0xFF; a read request is `[addr, cmd]` (no CRC) and answered with `[data..., CRC16]` where the CRC covers addr, cmd and the reply data; CRC16 is the XMODEM CRC (poly 0x1021, init 0, MSB first, high byte first on the wire; changelog 4.1.12 "CRC-CCITT(CRC16 xmodem, polynomial 0x1021)"); multi-byte values are big-endian; a packet with a gap > 10 ms between bytes is discarded; an invalid packet gets no acknowledgement. Commands (`python/napl/roboclaw.py` docstring has the full table): 0/1/4/5/6/7 7-bit drive, 16/17 encoder count + status byte, 18/19 speed + direction byte, 20/22/23 reset / set encoders, 21 version, 24 main voltage (0.1 V), 28/29 set velocity PID (order D, P, I, QPPS, each x65536; QPPS plain), 55/56 read PID (order P, I, D, QPPS), 32-34 signed duty, 35-37 signed speed, 38-40 speed + acceleration, 52-54 duty + acceleration, 48 PWM, 49 current (10 mA), 90 status, 133-136 max current. Not implemented (no reply): mixed mode 8-13, buffered and position commands, timeout, logic voltage, temperatures, EEPROM, 57-60, 78/79, 91-93, PWM mode.

Where the manual and the library disagree the port follows the library (the thing users run) and says so [F, checked against the v2.0.10 source]: command 52/53 take duty (2 bytes) + accel (4 bytes) in the library, the manual says accel 2 bytes; command 90 is read as one 4-byte long by the library, the manual shows only the 16-bit mask table (the port replies with 4 bytes and sets only bits 0x0001/0x0002 overcurrent warning against the stored max current, 0x0400/0x0800 main battery high/low); the library's `ERROR_*` constants are a different layout from the manual's status table (the port follows the manual's). Commands 57/59 (voltage limits) carry an extra byte in the library (`auto_offset`), not implemented for that reason.

Mapping to the simulator: every write is a `SetAttr` (source `roboclaw`, logged, so the run replays to the same hash) on `/devices/<servo>` attributes `mode`, `goal_duty`, `duty_accel` (accel/32767 per s), `goal_velocity`, `vel_accel`, `qpps`, `vel_p`, `vel_i`, `vel_d`; reads query `counts` (minus the reset offset kept in the port), `speed`, `duty`, `current`, `voltage`. The encoder status byte keeps the manual's bits: underflow (bit 0) and overflow (bit 2) set when the unsigned 32-bit counter passes 0 and clear after the read, direction (bit 1). The port starts the motors in duty mode, torque on, duty 0, with the manual's default QPPS / PID. A `[interfaces.roboclaw]` scenario section (`motors = {1 = ..., 2 = ...}`, `address`, `baud`) selects it in `napl program` (`python/napl/program.py`).

Byte-exact vectors (CRC from the manual's C function, transliterated in `tests/python/test_roboclaw.py`; the manual prints no packet with a CRC so there is no vector to copy; the standard check `CRC("123456789") = 0x31C3`):

| request | bytes |
|---|---|
| drive M1 forward 64 | `80 00 40 73 9e` |
| duty M1 / M2 +16384 / -16384 | `80 22 40 00 c0 00 e0 df` |
| velocity PID M1: D 0x4000, P 0x10000, I 0x8000, QPPS 44000 | `80 1c 00 00 40 00 00 01 00 00 00 00 80 00 00 00 ab e0 26 fa` |
| reset encoders | `80 14 49 2d` |
| speed M1 / M2 +1000 / -1000 | `80 25 00 00 03 e8 ff ff fc 18 82 90` |
| reply to read voltage at 12.3 V | `00 7b f8 06` (CRC over `80 18 00 7b`) |

## 5. Validation (hfbox, `tests/python/test_rover.py -s`, 6 passed)

Motor points at exactly 12.000 V at the motor (the battery's `ocv_offset` fault input is trimmed until the wheel servo reads 12.000 V). Free run and stall are what the parameters were derived from, so they check the implementation, not the data; the maximum-efficiency point is the one independent check.

| Point | Datasheet | Simulation | Error |
|---|---|---|---|
| free-run speed, 12 V | 530 rpm | 529.4 rpm | -0.12 % |
| free-run current, 12 V | 0.2 A | 0.202 A | +1.1 % |
| stall current, 12 V (jammed wheel, 0.18 s after start) | 5.5 A | 5.384 A | -2.1 % (voltage trimmed to 12.004 V; R(T) already includes the heating of the first 0.18 s) |
| stall torque, 12 V | 8.5 kg cm | 8.61 kg cm | +1.25 % |
| speed at the max-efficiency load (1.0 kg cm applied as `ext_torque`) | 470 rpm | 467.0 rpm | -0.64 % |
| current at the same load | 0.76 A | 0.813 A | +7.0 % |
| efficiency / output power there | 55 % / 5.0 W | 49.3 % / 4.81 W | -10.4 % (follows from the current) |

Straight line on the floor (duty 0.5 on the 12.30 V pack, 3 s window after 1.5 s): 1.2537 m/s plate speed; wheel rate x radius 1.2552 m/s (rolling slip -0.12 %); the datasheet-scaled prediction `530 rpm * (0.5 V - i0 R)/(12 - i0 R) * 0.045 m` = 1.2344 m/s (+1.57 %); lateral drift 0.00 mm over 3.76 m, yaw 0.000 deg (a symmetric model: no random wheel mismatch). Velocity mode 2000 counts/s: 0.4705 m/s against 0.4712 commanded (-0.15 %).

1 m square (`napl program examples/rover/square.scenario.toml`, the unmodified `basicmicro` library, 38400 baud, real time): 18.5 s, true end 13.9 mm from the start (monitor limit 50 mm), odometry's own end estimate (0.3, -19.7) mm (the odometry error is the wheelbase / slip: the heading loop only sees the encoders), all three other corners within 50 mm, plate height constant (no tipping), no motor alarm, pack above 9 V, replay hash matches (`napl-replay`, 18484 ticks).

Observations from building it [F, measured]: spinning in place at 0.15 duty gave an effective track of 0.2237 m against 0.223 m from the geometry (0.3 %); with the manual's default PID (P 1, I 0.5, D 0.25) the heading drifted 23 degrees over one straight 1 m leg (noisy duty, see section 3); with I 4, D 0 it drifted 5 degrees, and the heading hold in the example brought the leg error to under 1 degree.

## 6. Limitations

- Velocity-loop law, 333 Hz rate and 20 ms filter are estimates; the manual's default gains are not tuned for this plant in this model.
- Max-efficiency point current is 7 % high: eta (0.755) is derived from the stall torque, the datasheet's stall values are extrapolations, so the model is a little lossy at light load.
- Stall: no further heating study (thermal parameters estimated); `max-current` is stored, never applied.
- DRC does not check the motor-net voltage / current against the RoboClaw rating (only the battery -> controller net). `napl design check assemblies/rover.toml`: 0 errors, 0 warnings, 1 info (`SERVO_ID_UNSET`: the S5 servo-bus rule treats the two wheel `servo` devices as shared-bus serial servos; they are not).
- The status-byte bits and the version string are partly placeholders (string "USB Roboclaw 2x7A v4.1.12 (napl sim)", the manual's example is "RoboClaw 10.2A v4.1.11").
