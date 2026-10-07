// Servo firmware (S3): output encoder + position loop, phase 2 of the device tick. The motor / gearbox / H-bridge side
// is the shared drive code (electrical.cpp: network, devices.cpp: drive_motor).
//
// Fidelity notes:
//  - Encoder: counts = floor(q_out cpr / 2 pi + 0.5) of the gear OUTPUT joint, an unwrapped linear count (no wrap at
//    one revolution; a real STS3215 reports 0..4095 on a single turn). Ideal: no noise, no gear-lash hysteresis in the
//    reading, no sampling delay. counts / position are refreshed every tick from the state at the start of the tick.
//  - Firmware runs on its own period (firmware_hz); the duty it computes is held between steps (zero-order hold).
//    Order of one step: ramp the setpoint, error in integer counts, deadband, PID, saturation. Anti-windup: the sum is
//    not accumulated while the output is saturated and the error pushes further into saturation.
//  - pid_scale (register units -> duty): duty = (p e + i sum(e) + d de) / pid_scale with e in counts and sum(e), de per
//    firmware step. Calibrated for the STS3215 per docs/research/sts3215-calibration.md: official table P "(1/4)" with the
//    0.1 % duty unit -> 4000, Rhoban BAM oscilloscope gain 0.166 -> 3927. The table scales D by 1/8, not modelled
//    separately (one pid_scale for p, i, d). The vendor's firmware structure (loop rate, integral limit, speed profile
//    shape) is not modelled beyond what is listed here.
//  - torque_enable = 0: open phase, integrator reset, the setpoint follows the measured position (so enabling with a
//    goal speed ramps from where the servo is; without one the setpoint jumps to the goal). Disabling acts on the next
//    tick, enabling takes effect at the next firmware step. Power-up state is torque off, goal 0.
//  - Runtime registers: p, i, d, deadband, max_torque, torque_limit are state (settable attrs, initial = config); the
//    duty limit is min(max_torque, torque_limit) / 1000. They are read at every firmware step.
//  - Protection (modelled on the Feetech STS memory table: Max_Temperature, Min / Max_Input_Voltage, Overload_Torque,
//    Protection_Time, Unloading_Condition; bit values of its status byte). Evaluated every tick from the outputs of the
//    previous solve, a threshold of 0 switches that check off:
//      bit 1  voltage: node voltage < min_voltage or > max_voltage;
//      bit 4  temperature: winding temperature > max_temp (the sim's thermal state, not a board sensor);
//      bit 32 overload: |duty| > overload_load (fraction of FULL duty, not of max_torque) continuously for overload_time.
//    Bits latch. A tick with torque_enable = 0 clears them (conditions that still hold set them again at once); there
//    is no other way to clear them (a real STS also needs the torque to be cycled). A latched bit that is also in
//    unload_mask releases the torque: open phase from the same tick, firmware reset like torque off, until the bits are
//    cleared. Without the mask bit the alarm is only reported. Not modelled: encoder (2), overcurrent (8) and angle (16)
//    bits, holding torque after overload (reg 34), LED alarm mask, and the real chip's thresholds' exact filtering.
//    Overload counts any time at high duty, including a long fast move (a real servo does too).
#include <algorithm>
#include <cmath>

#include "mujoco/devices/devices.h"

namespace devs {
namespace {
constexpr unsigned kAlarmVoltage = 1, kAlarmTemp = 4, kAlarmOverload = 32;  // STS status byte bits
}  // namespace

void step_firmware(const mjModel* m, const mjData* d, Devices& dv) {
  const double dt = m->opt.timestep;
  for (std::size_t i = 0; i < dv.servo.cfg.size(); ++i) {
    const ServoCfg& c = dv.servo.cfg[i];
    ServoState& s = dv.servo.st[i];
    const double per_rad = c.cpr / (2.0 * kPi);
    s.counts = std::floor(d->qpos[c.qadr] * per_rad + 0.5);
    s.position = s.counts / per_rad;

    // Protection, every tick, from the outputs of the previous solve (none exist before the first tick).
    const bool on = s.torque_enable > 0.5;
    unsigned a = on ? static_cast<unsigned>(s.alarm) : 0u;  // a tick with torque off clears the latches
    if (s.ticks > 0) {
      if ((s.min_voltage > 0 && s.v < s.min_voltage) || (s.max_voltage > 0 && s.v > s.max_voltage)) a |= kAlarmVoltage;
      if (s.max_temp > 0 && s.temp > s.max_temp) a |= kAlarmTemp;
    }
    s.ovl_t = on && s.overload_load > 0 && std::abs(s.duty) > s.overload_load ? s.ovl_t + dt : 0.0;
    if (s.ovl_t > 0 && s.ovl_t >= s.overload_time) a |= kAlarmOverload;
    s.alarm = a;

    if (s.ticks++ % static_cast<std::uint64_t>(c.period) != 0) continue;  // between firmware steps: duty held
    if (servo_open(s)) {  // torque off or released by an alarm (the drive itself is open from this very tick)
      s.setpoint = s.position;
      s.integ = s.e_prev = s.cmd = 0;
      continue;
    }
    const double step = s.goal_speed * c.period * dt;
    s.setpoint = s.goal_speed > 0 ? s.setpoint + std::clamp(s.goal_position - s.setpoint, -step, step)
                                  : s.goal_position;
    double e = std::floor(s.setpoint * per_rad + 0.5) - s.counts;
    if (std::abs(e) <= s.deadband) e = 0;
    const double lim = std::min(s.max_torque, s.torque_limit) / 1000.0;
    auto pid = [&](double sum) { return (s.p * e + s.i * sum + s.d * (e - s.e_prev)) / c.pid_scale; };
    double u = pid(s.integ + e);
    if (std::abs(u) > lim && u * e > 0) u = pid(s.integ);  // saturated and still pushing: freeze the integrator
    else s.integ += e;
    s.e_prev = e;
    s.cmd = std::clamp(u, -lim, lim);  // lim <= 1 (both limits <= 1000): the H-bridge duty limit
  }
}

}  // namespace devs
