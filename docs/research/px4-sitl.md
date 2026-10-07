# PX4 SITL interface notes (N1b)

Everything below was read from the PX4 source tree at tag **v1.17.0** (raw GitHub, 2026-10-07) unless marked
*inferred* (reasoned from source, not executed) or *unverified*. No build was run.

## Release

- Stable: **v1.17.0**, "v1.17.0 - Stable Release", published 2026-05-13T19:03:26Z
  (https://api.github.com/repos/PX4/PX4-Autopilot/releases/latest). Newer tags are pre-releases only
  (v1.18.0-rc1, 2026-09-10; beta2, beta1, alpha1). Previous stable v1.16.2 (2026-04-22). Pin `v1.17.0`.
- Base URL for all source quotes: https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/

## How an external simulator connects (no Gazebo)

- Docs, `docs/en/simulation/index.md`: "All simulators except for Gazebo communicate with PX4 using the Simulator MAVLink API. This API
  defines a set of MAVLink messages that supply sensor data from the simulated world to PX4 and return motor and actuator values from the
  flight code that will be applied to the simulated vehicle." and "PX4 uses a simulation-specific module to connect to the simulator's
  local TCP port 4560."
- Module `src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp`. **PX4 is the TCP client**: `run()` loops
  `connect(_fd, (struct sockaddr *)&_myaddr, myaddr_len)` ("Waiting for simulator to accept connection on TCP port %u"). So our
  `napl-px4-bridge` must be the **TCP server listening on 4560 + px4_instance**, then PX4 connects (retries every 500 us).
  UDP mode exists (`-u`): PX4 binds and waits for the first datagram ("Waiting for simulator to connect on UDP port").
  Usage string: `simulator_mavlink {start -[spt] [-u udp_port / -c tcp_port] |stop|status}`.
- Start script `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlinksim` (selected by `px4-rc.simulator` when SYS_AUTOSTART != 0, SIM_GZ_EN != 1,
  not jMAVSim 10017, not SIH): `simulator_tcp_port=$((4560+px4_instance))`, then `simulator_mavlink start -c $simulator_tcp_port`
  (localhost), or `-t "${PX4_SIM_HOST_ADDR}" port`, or `-h "${PX4_SIM_HOSTNAME}" port`. It also sets
  `param set-default EKF2_GPS_DELAY 10`, `EKF2_MULTI_IMU 3`, `SENS_IMU_MODE 0`.
- Handshake: PX4 first sends a MAVLink HEARTBEAT (`hb.autopilot = 12`) "to initiate the SITL simulator to start sending sensor data which
  will set the time and get everything rolling. Without this, we get stuck at px4_poll which waits for a time update."
- PX4 requests ground truth: `request_hil_state_quaternion()` sends COMMAND_LONG `MAV_CMD_SET_MESSAGE_INTERVAL` for
  HIL_STATE_QUATERNION with `param2 = 5e3` us (200 Hz).

### Message set (handler `SimulatorMavlink::handle_message`, from the sim)

Handled IDs: HIL_SENSOR, HIL_OPTICAL_FLOW, ODOMETRY, VISION_POSITION_ESTIMATE, DISTANCE_SENSOR, HIL_GPS, RC_CHANNELS, LANDING_TARGET,
HIL_STATE_QUATERNION, RAW_RPM. Nothing else (notably **not** BATTERY_STATUS, see battery section).

**HIL_SENSOR (107), sim -> PX4**, units from MAVLink `common.xml`: `time_usec` us; `xacc,yacc,zacc` m/s/s; `xgyro,ygyro,zgyro` rad/s (body,
FRD/"NED body frame"); `xmag,ymag,zmag` **gauss**; `abs_pressure` **hPa**; `diff_pressure` hPa; `pressure_alt` (m); `temperature` degC;
`fields_updated` bitmask; extension `id` uint8 sensor index.
`HIL_SENSOR_UPDATED_FLAGS`: XACC=1, YACC=2, ZACC=4, XGYRO=8, YGYRO=16, ZGYRO=32, XMAG=64, YMAG=128, ZMAG=256, ABS_PRESSURE=512,
DIFF_PRESSURE=1024, PRESSURE_ALT=2048, TEMPERATURE=4096, RESET=2^31. PX4 tests them as groups (`SimulatorMavlink.hpp`):
`ACCEL = 0b111` (7), `GYRO = 0b111000` (56), `MAG = 0b111000000` (448), `BARO = 0b1101000000000` (abs_pressure+pressure_alt+temperature = 6656),
`DIFF_PRESS = 0b10000000000` (1024). **All three bits of a group must be set** (`(fields_updated & SensorSource::BARO) == SensorSource::BARO`).
Typical all-sensors value: 7+56+448+6656 = 7167 (0x1BFF). Temperature is only taken with the baro group. Pressure: PX4 applies
`hPa -> Pa` itself for diff pressure; abs_pressure handling is in `update_sensors`.
IMU instances: `id` selects accel/gyro instance (`ACCEL_COUNT_MAX` bound); **`id == 0` is the primary IMU and drives lockstep** and is
configured as a +-16 g FIFO accel with `ACCEL_FIFO_SCALE = CONSTANTS_ONE_G / 2048.f`.

**HIL_GPS (113), sim -> PX4**: `time_usec` us; `fix_type` (3 = 3D); `lat`, `lon` degE7; `alt` **mm** MSL (PX4: `hil_gps.alt / 1e3` m);
`eph`, `epv` (cm-scaled, `*1e-2` in PX4, "HDOP*100" per MAVLink); `vel` cm/s ground speed; `vn, ve, vd` cm/s NED; `cog` cdeg (65535 = unknown);
`satellites_visible`. PX4 hardcodes `s_variance_m_s = 0.25`, `c_variance_rad = 0.5`, `vel_ned_valid = true`. Send at the GPS rate (the
M8N: 5-10 Hz, see parts file `ublox-neo-m8n`).

**HIL_ACTUATOR_CONTROLS (93), PX4 -> sim** (`actuator_controls_from_outputs`): `time_usec`; `controls[16]` float = `actuator_outputs.output[i]`
**only while armed, else zeros**; `mode` = `custom(1)` | `armed(128)`; `flags` bit 0 = `HIL_ACTUATOR_CONTROLS_FLAGS_LOCKSTEP` set when the
scheduler is compiled in. Motor channels are scaled by `PWMSim.cpp` to **[0, 1]** (`(pwm - 1000)/(2000 - 1000)` for outputs whose function is
Motor1..MotorMax and not reversible), all other functions to [-1, 1]. So for a quad `controls[0..3]` = motor throttle 0..1 (duty command).

**HIL_STATE_QUATERNION (115), sim -> PX4 (ground truth)**: attitude quaternion w,x,y,z, rates rad/s, `lat/lon` degE7, `alt` mm,
`vx,vy,vz` cm/s, accel mG. PX4 logs it for estimator comparison; the flight code does not fly on it.

### Lockstep semantics (source)

- Built in: `boards/px4/sitl/sitl.cmake` sets `ENABLE_LOCKSTEP_SCHEDULER yes` unless `replay` env is set.
- Receive side (`handle_message_hil_sensor`): for `imu.id == 0` PX4 sets the sim clock from the message, `px4_clock_settime(CLOCK_MONOTONIC, ts(imu.time_usec))`, runs
  `update_sensors`, then `px4_lockstep_progress(_lockstep_component)`. PX4 time therefore **only advances on HIL_SENSOR id 0**.
- Send side (`send()`): thread waits (`px4_poll`, 100 ms timeout) for new `actuator_outputs_sim`, updates vehicle_status and battery_status, calls
  `px4_lockstep_wait_for_components()` ("Wait for other modules, such as logger or ekf2"), then `send_controls()`.
- Resulting loop for the bridge (*inferred from the above*): per step send HIL_SENSOR(id 0, time_usec = sim time) [plus HIL_GPS when due, other ids,
  HIL_STATE_QUATERNION]; **block until HIL_ACTUATOR_CONTROLS arrives**; apply controls; step the world; repeat. The existing SITL default for the
  IMU stream is 250 Hz (`param set-default IMU_INTEG_RATE 250` in `px4-rc.simulator`; a commented debug check in the source assumes 4000 us steps), so use
  dt = 4 ms unless there is reason otherwise. `time_usec` must be monotonic; first message may arrive any time after the heartbeat.
- Speed: `PX4_SIM_SPEED_FACTOR` env scales timeouts only; lockstep itself makes sim speed bridge-controlled (docs: "run in lockstep, which means that
  they are locked to run at the same speed").

## Building `px4_sitl` without Gazebo

- Target: `make px4_sitl_default` builds `build/px4_sitl_default/bin/px4`. `boards/px4/sitl/default.px4board` has `CONFIG_COMMON_SIMULATION=y`,
  which (`src/modules/simulation/Kconfig`) selects `SIMULATOR_MAVLINK`, `BATTERY_SIMULATOR`, `PWM_OUT_SIM`, `SENSOR_{AIRSPEED,BARO,GPS,MAG}_SIM`,
  `SYSTEM_POWER_SIMULATOR`, `SIMULATOR_SIH`. `gz_bridge` is only compiled `if (gz-transport_FOUND)` (`gz_bridge/CMakeLists.txt`), so with no Gazebo installed
  it is skipped silently. **No simulator package is needed to build.** (Whether the build host has gz installed is unchecked.)
- `make px4_sitl_default none_iris` is a legacy helper target in `simulator_mavlink/CMakeLists.txt`:
  `add_custom_target(none_iris COMMAND env PX4_SYS_AUTOSTART=10016 $<TARGET_FILE:px4> ...)`. Equivalent manual launch:
  `PX4_SYS_AUTOSTART=10016 ./build/px4_sitl_default/bin/px4` from a working dir (*inferred*; not run). `rcS` honours `PX4_SYS_AUTOSTART`, then
  `PX4_SIM_MODEL` (looks up `init.d-posix/airframes/<id>_<model>`), and `PX4_SIM_MODEL=none` => autostart 0 => SIH (see below) so do not set it.
- Airframes: `4001_gz_x500` sets `param set-default SIM_GZ_EN 1` and `PX4_SIMULATOR=gz` => routes to Gazebo, **not usable** without gz.
  `10016_none_iris` ("Generic Iris Quadrotor SITL") goes through `px4-rc.mavlinksim` and is the non-gz path. SIH (`10040_sihsim_quadx`, or SYS_AUTOSTART=0)
  is PX4's *built-in* simulator and would bypass our world.
- **Real X500 V2 airframe**: `ROMFS/px4fmu_common/init.d/airframes/4019_x500_v2` ("Holybro X500 V2", `@type Quadrotor x`) exists for hardware but
  SITL autostart only searches `init.d-posix/airframes/` (`rcS`: `ls ${R}etc/init.d-posix/airframes`). Options (*not verified*): add our own
  `init.d-posix/airframes/10100_none_x500v2` copying 4019's params (the posix airframes CMakeLists lists files by name, so also add it there, in the
  build-host checkout only), or boot 10016 and `param set` the CA_ROTOR* geometry. Do this in N5 on the build host; PX4 is fetched there, not vendored.
- Build deps (`Tools/setup/ubuntu.sh`, `Tools/setup/requirements.txt`): apt needs `build-essential`, `gcc/g++`, `git`, `make`, `cmake`,
  `ninja-build`, `python3-dev`, `libssl-dev`, `libxml2-dev`, `libxml2-utils`, `ccache`, `zip/unzip`, etc.; the Python set
  (`argcomplete cerberus coverage empy>=3.3,<4 future jinja2>=2.8 jsonschema kconfiglib lark lxml matplotlib numpy nunavut>=1.1.0 packaging pandas
  pkgconfig psutil pycryptodome pygments pymavlink pyros-genmsg pyserial pyulog>=0.5.0 pyyaml requests setuptools six sympy>=1.10.1 toml wheel`) is all
  PyPI, so `pip install -r Tools/setup/requirements.txt` satisfies it without apt. `cmake` and `ninja` also exist as PyPI wheels (*inferred*), so only
  a C/C++ compiler, make, git (plus `libxml2-utils` for `xmllint`, used by the build's metadata step: *inferred*) need the OS. The script supports
  `--no-nuttx` and `--no-sim-tools` ("jMAVSim and Gazebo9 simulator (omit with arg: --no-sim-tools)"), confirmed in its header; supports Ubuntu 24.04 and 22.04.

## Battery

- Internal sim: `battery_simulator` (`BatterySimulator.cpp`) is started by `rcS` iff `param compare SIM_BAT_ENABLE 1`. Model: linear in time:
  `_battery_percentage -= dt / (SIM_BAT_DRAIN s)` while armed (default **60 s** drain interval), clamped at `SIM_BAT_MIN_PCT` (default 50 %);
  `vbatt = interpolate(pct, empty_cell_voltage .. full_cell_voltage) * cell_count`; `current = -1` ("no current sensor in simulation"); `BAT1_N_CELLS 4` set in
  `rcS`. No sag, no load dependence.
- Disable and feed real values: param doc (`battery_simulator_params.c`): "**Simulator Battery enabled**: Enable or disable the internal battery simulation.
  This is useful when the battery is simulated externally and interfaced with PX4 through MAVLink for example." => `param set SIM_BAT_ENABLE 0`
  (set-default in the airframe file, or `PX4_PARAM_...` equivalent in the launch).
- Inbound path: `MavlinkReceiver::handle_message_battery_status` takes **BATTERY_STATUS (147)** and publishes `battery_status`:
  `voltages[]` mV (cell voltages, up to 10, summed, stops at UINT16_MAX; so send per-cell mV), `current_battery` cA (1/100 A) -> `current_a`,
  `battery_remaining` percent -> `remaining`, `current_consumed` mAh -> `discharged_mah`, `temperature`, `connected = true`; warning levels from
  `BAT_LOW_THR / BAT_CRIT_THR / BAT_EMERGEN_THR`. The message is **ignored if `msg->sysid != mavlink_system.sysid` or `msg->compid == mavlink_system.compid`**,
  i.e. it must carry PX4's own system id (MAV_SYS_ID, default 1) and a *different* component id.
- **HIL_SENSOR has no battery fields**, and the `simulator_mavlink` TCP handler does not dispatch BATTERY_STATUS (list above). Therefore the battery must be
  sent over an ordinary MAVLink UDP link to the `mavlink` module, not the 4560 socket. *Inferred from `px4-rc.mavlink`*: PX4 listens on local UDP
  `14580 + instance` (offboard) and `18570 + instance` (GCS); remote ports 14540/14550. Bridge sends BATTERY_STATUS to `127.0.0.1:14580` and reads PX4
  telemetry (battery, status) back on 14540. Rate: 1-10 Hz is enough (the sim battery updates at `BATTERY_SIMLATOR_SAMPLE_INTERVAL_US`).
- Also note `simulator_mavlink` reuses `battery_status.voltage_v` for fake ESC telemetry, and fills `esc_current = 1 + |control| * 15 A`, `esc_rpm = control * 6000`
  ("TODO: magic number") only inside PX4; irrelevant to our physics.

## X500 airframe: motor geometry, numbering, mapping

- PX4 control-allocator frame is FRD (x forward, y right). `4001_gz_x500` (Gazebo x500, not the Holybro kit): `CA_ROTOR0 (0.13, 0.22)`, `CA_ROTOR1 (-0.13,-0.20)`,
  `CA_ROTOR2 (0.13,-0.22)`, `CA_ROTOR3 (-0.13, 0.20)`, `KM +0.05, +0.05, -0.05, -0.05`; outputs `SIM_GZ_EC_FUNC1..4 = 101..104`.
  `10016_none_iris`: positions (+-0.1515, +0.245 / -0.1875 ...), same KM signs, `PWM_MAIN_FUNC1..4 = 101..104`.
- **`4019_x500_v2` (real Holybro X500 V2)**: "Square quadrotor X PX4 numbering": `CA_ROTOR_COUNT 4`;
  `CA_ROTOR0_PX/PY = 1, 1`; `ROTOR1 = -1, -1`; `ROTOR2 = 1, -1`, `KM -0.05`; `ROTOR3 = -1, 1`, `KM -0.05`. Positions are normalised (+-1 =
  square, scaled by the allocator), so the real arm length does not enter. Mapping: **rotor0 front-right, rotor1 rear-left, rotor2 front-left, rotor3 rear-right**;
  rotor0/1 default `KM` (= +0.05, *inferred default*; not set in the file) are the same spin sense, rotor2/3 opposite (`-0.05`). That is PX4 quad-X: motor 1 FR CCW,
  2 RL CCW, 3 FL CW, 4 RR CW. Physical position: +-176.8 mm per axis (wheelbase 500 mm). Outputs map via `PWM_MAIN_FUNC1..4 = 101..104` (Motor1..4 = HIL controls[0..3]).
- Gains from 4019: `IMU_GYRO_CUTOFF 30`, `MC_ROLLRATE_P/PITCHRATE_P 0.14`, `_I 0.3`, `_D 0.004`; `MPC_THR_HOVER` is **0.60 in the gz_x500 file**, not in 4019 (default).
- Spin direction must match the real kit: CCW props go on CCW motors (PX4 kit docs: "The bottom plate indicates the direction of the motor").

## MAVLink library license

`mavlink/c_library_v2` (generated headers; repo https://github.com/mavlink/c_library_v2) has no LICENSE file of its own (`LICENSE` returned 404; GitHub reports no license).
The generator repo's COPYING (https://github.com/mavlink/mavlink/blob/master/COPYING) says: "The generator itself is (L)GPL v3 licensed, while the generated code is
subject to different licenses ... As an exception, if you use this Software to compile your source code and portions of this Software, including messages
('the generator output'), are embedded into the binary product as a result, you may redistribute such product and such product is hereby licensed under the
following MIT license". => the generated C headers are **MIT**; usable in our proprietary bridge. Do not vendor the generator; vendoring generated headers
should carry the MIT text. Plan note (phase2-plan) "c_library_v2, MIT" is consistent but rests on this exception rather than a per-repo licence file.

## Gaps / risks

1. Nothing was built or run; launch commands and `none_iris` flow are inferred from source.
2. How to supply the X500 V2 geometry to SITL (new posix airframe vs `param set` overlay) is unresolved until N5.
3. PX4 default sim noise/params (`SENS_*`, `EKF2_*`) were not catalogued; PX4 consumes our noisy HIL_SENSOR directly (no extra noise added by `simulator_mavlink`).
4. Exact default `CA_ROTORn_KM` for rotors 0/1 not read from the param metadata.
5. Battery over UDP 14580 to the mavlink module: path inferred from `px4-rc.mavlink`; confirm in N5 that `battery_status` is published and which battery source PX4
   selects when `SIM_BAT_ENABLE 0` (BAT1_SOURCE not examined).
6. ESC-level telemetry fed back by PX4 is fake; real current/rpm come from our motor devices.
