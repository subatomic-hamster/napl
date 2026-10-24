#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

namespace execution {
struct Snapshot {
  std::string requested, effective, thermal, pressure, reason;
  unsigned cores = 1, workers = 1;
  std::uint64_t physical_bytes = 0, available_bytes = 0;
  double duty = 1, memory_fraction = .75;
  bool low_power = false;
};
// Process-local policy: deliberately excluded from World snapshots and replay logs.
void initialize();
void thread_entry();
void configure(const std::string& profile);
Snapshot status(double gb_per_worker = 1.0);
void signals(int thermal, bool low_power, int pressure, std::uint64_t available_bytes);
void start_apple_telemetry();
// Wakes resting governors to re-check their pending-work flag and the policy generation; call after setting a flag.
void wake_rests();
// Registers, for the current thread, a flag meaning "commands are waiting": a governed step resting on this thread
// stops (keeping its debt, paid after the next step) once it is set. nullptr: no flag for this scope (a running Host
// job: queued commands cannot run before it returns). Without one, only policy changes wake a rest.
class RestWakeScope {
 public:
  explicit RestWakeScope(const std::atomic<bool>* pending);
  ~RestWakeScope();
  RestWakeScope(const RestWakeScope&) = delete;
  RestWakeScope& operator=(const RestWakeScope&) = delete;
 private:
  const std::atomic<bool>* previous_;
};
unsigned presentation_fps();  // Atomic-only presentation ceiling; zero keeps the renderer's existing cadence.

// Scoped on native simulation calls; never permanently demotes a Python/UI thread.
class QosScope {
 public:
  QosScope();
  ~QosScope();
  void refresh();
 private:
  unsigned previous_ = 0;
  int relative_ = 0;
  bool changed_ = false;
  unsigned target_ = 3;
};

class Governor {
 public:
  void starting_step(QosScope& qos);
  void completed_step(QosScope& qos);
 private:
  std::uint64_t generation_ = 0;
  double duty_ = 1;
  std::chrono::steady_clock::time_point started_{}, finished_{};  // finished_: last governed completed_step
  double work_seconds_ = 0;
  double rest_seconds_ = 0;  // positive: owed rest; negative: overslept rest credit
  void refresh(QosScope& qos);
};
}  // namespace execution
