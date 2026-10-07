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
//  - pid_scale (register units -> duty) is a calibration knob: duty = (p e + i sum(e) + d de) / pid_scale with e in
//    counts and sum(e), de per firmware step. The true Feetech register scaling is not public as far as we know; do not
//    read the p / i / d numbers as the vendor's. Likewise the vendor's actual firmware structure (loop rate, integral
//    limit, speed profile shape, protection thresholds) is not modelled beyond what is listed here.
//  - torque_enable = 0: open phase, integrator reset, the setpoint follows the measured position (so enabling with a
//    goal speed ramps from where the servo is; without one the setpoint jumps to the goal). Disabling acts on the next
//    tick, enabling takes effect at the next firmware step. Power-up state is torque off, goal 0.
#include <algorithm>
#include <cmath>

#include "mujoco/devices/devices.h"

namespace devs {

void step_firmware(const mjModel* m, const mjData* d, Devices& dv) {
  const double dt = m->opt.timestep;
  for (std::size_t i = 0; i < dv.servo.cfg.size(); ++i) {
    const ServoCfg& c = dv.servo.cfg[i];
    ServoState& s = dv.servo.st[i];
    const double per_rad = c.cpr / (2.0 * kPi);
    s.counts = std::floor(d->qpos[c.qadr] * per_rad + 0.5);
    s.position = s.counts / per_rad;
    if (s.ticks++ % static_cast<std::uint64_t>(c.period) != 0) continue;  // between firmware steps: duty held
    if (!(s.torque_enable > 0.5)) {
      s.setpoint = s.position;
      s.integ = s.e_prev = s.cmd = 0;
      continue;
    }
    const double step = s.goal_speed * c.period * dt;
    s.setpoint = s.goal_speed > 0 ? s.setpoint + std::clamp(s.goal_position - s.setpoint, -step, step)
                                  : s.goal_position;
    double e = std::floor(s.setpoint * per_rad + 0.5) - s.counts;
    if (std::abs(e) <= c.deadband) e = 0;
    const double lim = c.max_torque / 1000.0;
    auto pid = [&](double sum) { return (c.p * e + c.i * sum + c.d * (e - s.e_prev)) / c.pid_scale; };
    double u = pid(s.integ + e);
    if (std::abs(u) > lim && u * e > 0) u = pid(s.integ);  // saturated and still pushing: freeze the integrator
    else s.integ += e;
    s.e_prev = e;
    s.cmd = std::clamp(u, -lim, lim);  // lim <= 1 (max_torque <= 1000): the H-bridge duty limit
  }
}

}  // namespace devs
