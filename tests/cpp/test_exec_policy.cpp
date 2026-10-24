#include "core/exec_policy.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <optional>
#include <thread>

using Clock = std::chrono::steady_clock;

namespace {
// Duty a governed loop of 50 us steps delivers for 1.5 s under the background profile. With `command_every`, another
// thread posts a command that often (sets the thread's pending flag and wakes rests, as Host::enqueue does) and the
// loop drains it after the step, as Host::loop does. nullopt: live telemetry already changed the background duty.
std::optional<double> background_duty(std::optional<std::chrono::milliseconds> command_every) {
  struct Restore {
    std::string previous = execution::status().requested;
    ~Restore() { execution::configure(previous); }
  } restore;
  execution::configure("background");
  if (execution::status().duty != .5) return std::nullopt;
  std::atomic<bool> done{false}, pending{false};
  std::thread commands;
  if (command_every)
    commands = std::thread([&] {
      while (!done.load()) {
        pending = true;
        execution::wake_rests();
        std::this_thread::sleep_for(*command_every);
      }
    });
  double busy = 0;
  const auto t0 = Clock::now();
  {
    const execution::RestWakeScope wake(&pending);
    execution::QosScope qos;
    execution::Governor governor;
    while (Clock::now() - t0 < std::chrono::milliseconds(1500)) {
      governor.starting_step(qos);
      const auto b = Clock::now();
      while (Clock::now() - b < std::chrono::microseconds(50)) {}
      busy += std::chrono::duration<double>(Clock::now() - b).count();
      governor.completed_step(qos);
      pending = false;  // drained
    }
  }
  const double duty = busy / std::chrono::duration<double>(Clock::now() - t0).count();
  done = true;
  if (commands.joinable()) commands.join();
  return duty;
}
}  // namespace

// Background QoS timers coalesce heavily on macOS (a 10 ms sleep can last ~200 ms). Before overslept rest was
// carried as credit, a governed bulk call of short steps delivered ~8% duty instead of the background 50%.
TEST(exec_policy, governed_short_steps_deliver_profile_duty) {
  const auto duty = background_duty(std::nullopt);
  if (!duty) GTEST_SKIP() << "live telemetry changed the background duty (" << execution::status().reason << ")";
  EXPECT_GT(*duty, .3);
  EXPECT_LT(*duty, .7);
}

// A rest ended by a command must be resumed after the next step, not deferred a whole work period: deferring let
// commands every 2 ms push a background thread to 0.86 duty while banking an unbounded stall for later.
TEST(exec_policy, frequent_commands_keep_profile_duty) {
  const auto duty = background_duty(std::chrono::milliseconds(2));
  if (!duty) GTEST_SKIP() << "live telemetry changed the background duty (" << execution::status().reason << ")";
  EXPECT_GT(*duty, .3);
  EXPECT_LT(*duty, .7);
}
