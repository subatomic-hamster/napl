#include "core/exec_policy.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#ifdef __APPLE__
#include <pthread.h>
#include <sys/sysctl.h>
#include <sys/qos.h>
#endif

namespace execution {
namespace {
using Clock = std::chrono::steady_clock;
thread_local bool dedicated_thread = false;
constexpr double kMaxRest = .25;  // seconds, either sign; background QoS oversleeps a <=20 ms request by up to ~200 ms
// Governor rests wait here so queued Host commands or a policy change end them at once: utility/background QoS timers
// coalesce, and an unwakeable 20 ms request was measured lasting up to ~200 ms (Host Pause p95 180 ms).
struct Rest {
  std::mutex mutex;
  std::condition_variable cv;
};
Rest& rest() {  // leaked like Policy: a thread resting at exit must not touch a destroyed condvar
  static Rest* r = new Rest;
  return *r;
}
thread_local const std::atomic<bool>* rest_pending = nullptr;
struct Policy {
  std::mutex mutex;
  Snapshot snapshot;
  unsigned requested_workers = 0, nested = 0;
  int thermal = -1, pressure = -1;
  double held_duty = 1;
  double held_memory_fraction = .75;
  unsigned held_workers = std::numeric_limits<unsigned>::max();
  Clock::time_point clear_since{};
  bool restricted = false;
  std::atomic<std::uint64_t> generation{1};
  std::atomic<unsigned> qos{2};
  Policy() {
    snapshot.cores = std::max(1u, std::thread::hardware_concurrency());
#ifdef __APPLE__
    snapshot.requested = "background";
    std::size_t bytes = sizeof snapshot.physical_bytes;
    sysctlbyname("hw.memsize", &snapshot.physical_bytes, &bytes, nullptr, 0);
#else
    snapshot.requested = "maximum";
    std::ifstream mem("/proc/meminfo");
    std::string name, unit;
    std::uint64_t kb;
    while (mem >> name >> kb >> unit) {
      if (name == "MemTotal:") snapshot.physical_bytes = kb * 1024;
      if (name == "MemAvailable:") snapshot.available_bytes = kb * 1024;
    }
    thermal = 0;
    pressure = 0;
#endif
    if (const char* p = std::getenv("NAPL_PROFILE")) snapshot.requested = p;
    if (snapshot.requested != "background" && snapshot.requested != "balanced" && snapshot.requested != "maximum")
      throw std::invalid_argument("NAPL_PROFILE must be background, balanced or maximum");
    auto read_count = [](const char* key) -> unsigned {
      const char* p = std::getenv(key);
      if (!p) return 0;
      char* end = nullptr;
      const unsigned long n = std::strtoul(p, &end, 10);
      if (!*p || *p == '-' || *end || !n || n > std::numeric_limits<unsigned>::max())
        throw std::invalid_argument(std::string(key) + " must be a positive integer");
      return static_cast<unsigned>(n);
    };
    requested_workers = read_count("NAPL_THREADS");
    nested = read_count("NAPL_WORKER_CEILING");
  }
};
Policy& policy() { static Policy* p = new Policy; return *p; }
unsigned profile_workers(const Snapshot& s) {
  if (s.effective == "background") return std::max(1u, std::min(2u, s.cores / 4));
  if (s.effective == "balanced") return std::max(1u, s.cores / 2);
  return s.cores;
}
void derive(Policy& p) {
  auto& s = p.snapshot;
  s.effective = s.requested;
  s.reason = "requested profile";
#ifdef __APPLE__
  const bool unknown = p.thermal < 0 || p.pressure < 0;
#else
  const bool unknown = false;
#endif
  if (s.low_power || unknown) {
    s.effective = "background";
    s.reason = unknown ? "telemetry unavailable" : "Low Power Mode";
  }
  s.duty = s.effective == "background" ? .5 : s.effective == "balanced" ? .75 : 1;
  s.memory_fraction = s.effective == "background" ? .25 : s.effective == "balanced" ? .5 : .75;
  s.workers = profile_workers(s);
  const bool serious = p.thermal >= 2 || p.pressure >= 1;
  const bool critical = p.thermal >= 3 || p.pressure >= 2;
  if (serious) {
    s.workers = 1;
    s.duty = std::min(s.duty, .25);
    s.reason = "thermal or memory pressure";
  }
  if (critical) {
    s.workers = 0;
    if (p.thermal >= 3) s.duty = std::min(s.duty, .1);
    s.reason = "critical resource pressure: admission closed";
  }
  const bool restricting = serious || s.low_power || unknown;
  const auto now = Clock::now();
  if (restricting) {
    if (!p.restricted) {
      p.held_duty = s.duty;
      p.held_workers = s.workers;
      p.held_memory_fraction = s.memory_fraction;
    } else {
      p.held_duty = std::min(p.held_duty, s.duty);
      p.held_workers = std::min(p.held_workers, s.workers);
      p.held_memory_fraction = std::min(p.held_memory_fraction, s.memory_fraction);
    }
    p.restricted = true;
    p.clear_since = {};
  } else if (p.restricted) {
    if (p.clear_since == Clock::time_point{}) p.clear_since = now;
    if (now - p.clear_since >= std::chrono::seconds(20)) p.restricted = false;
    else s.reason = "20-second resource recovery hysteresis";
  }
  if (p.restricted) {
    s.duty = std::min(s.duty, p.held_duty);
    s.workers = std::min(s.workers, p.held_workers);
    s.memory_fraction = std::min(s.memory_fraction, p.held_memory_fraction);
  }
  if (s.duty <= .5) s.effective = "background";
  if (p.requested_workers) s.workers = std::min(s.workers, p.requested_workers);
  p.qos.store(s.effective == "maximum" ? 2u : s.effective == "balanced" ? 1u : 0u,
              std::memory_order_relaxed);
  if (p.nested) s.workers = std::min(s.workers, p.nested);
  static const char* thermals[] = {"nominal", "fair", "serious", "critical"};
  static const char* pressures[] = {"normal", "warning", "critical"};
  s.thermal = p.thermal < 0 ? "unknown" : thermals[std::min(p.thermal, 3)];
  s.pressure = p.pressure < 0 ? "unknown" : pressures[std::min(p.pressure, 2)];
}
}  // namespace

void initialize() {
  (void)policy();
#ifdef __APPLE__
  static const bool started = [] { start_apple_telemetry(); return true; }();
  (void)started;
#endif
}
void configure(const std::string& profile) {
  if (profile != "background" && profile != "balanced" && profile != "maximum")
    throw std::invalid_argument("profile must be background, balanced or maximum");
  initialize();
  auto& p = policy();
  std::lock_guard lock(p.mutex);
  p.snapshot.requested = profile;
  derive(p);
  p.generation.fetch_add(1, std::memory_order_release);
  wake_rests();
}
Snapshot status(double gb_per_worker) {
  if (!std::isfinite(gb_per_worker) || gb_per_worker <= 0)
    throw std::invalid_argument("gb_per_worker must be positive and finite");
  initialize();
  auto& p = policy();
#ifndef __APPLE__
  std::uint64_t available = 0, kb = 0;
  std::string name, unit;
  std::ifstream mem("/proc/meminfo");
  while (mem >> name >> kb >> unit) {
    if (name == "MemAvailable:") { available = kb * 1024; break; }
  }
#endif
  std::lock_guard lock(p.mutex);
#ifndef __APPLE__
  p.snapshot.available_bytes = available;
#endif
  derive(p);
  Snapshot s = p.snapshot;
  const auto budget = std::min(static_cast<double>(s.physical_bytes) * s.memory_fraction,
                               static_cast<double>(s.available_bytes));
  const double count = std::floor(budget / (gb_per_worker * 1073741824.));
  s.workers = std::min(s.workers, static_cast<unsigned>(std::clamp(count, 0., static_cast<double>(s.cores))));
  if (!s.workers && s.reason == "requested profile") s.reason = "insufficient worker memory budget";
  return s;
}
void signals(int thermal, bool low_power, int pressure, std::uint64_t available_bytes) {
  auto& p = policy();
  std::lock_guard lock(p.mutex);
  p.thermal = thermal;
  p.pressure = pressure;
  p.snapshot.low_power = low_power;
  p.snapshot.available_bytes = available_bytes;
  derive(p);
  p.generation.fetch_add(1, std::memory_order_release);
  wake_rests();
}
void wake_rests() {
  auto& r = rest();
  std::lock_guard lock(r.mutex);  // a resting thread checks its predicate under this lock: no lost wakeup
  r.cv.notify_all();
}
RestWakeScope::RestWakeScope(const std::atomic<bool>* pending) : previous_(rest_pending) { rest_pending = pending; }
RestWakeScope::~RestWakeScope() { rest_pending = previous_; }
unsigned presentation_fps() {
  initialize();
  const auto qos = policy().qos.load(std::memory_order_relaxed);
  return qos == 0 ? 15u : qos == 1 ? 30u : 0u;
}

void thread_entry() {
  dedicated_thread = true;
  QosScope qos;
}
QosScope::QosScope() { refresh(); }
void QosScope::refresh() {
#ifdef __APPLE__
  const auto qos = policy().qos.load(std::memory_order_relaxed);
  if (qos == target_) return;
  target_ = qos;
  if (qos == 2 && !changed_ && !dedicated_thread) return;
  qos_class_t previous;
  int relative = 0;
  if (pthread_get_qos_class_np(pthread_self(), &previous, &relative) != 0) return;
  if (!changed_) {
    previous_ = previous;
    relative_ = relative;
  }
  const qos_class_t target = qos == 0 ? QOS_CLASS_BACKGROUND : qos == 1 ? QOS_CLASS_UTILITY :
      dedicated_thread ? QOS_CLASS_DEFAULT : static_cast<qos_class_t>(previous_);
  if (previous != target)
    changed_ = pthread_set_qos_class_self_np(target, 0) == 0 || changed_;
#endif
}
QosScope::~QosScope() {
#ifdef __APPLE__
  if (changed_ && !dedicated_thread)
    pthread_set_qos_class_self_np(static_cast<qos_class_t>(previous_), relative_);
#endif
}
void Governor::refresh(QosScope& qos) {
  auto& p = policy();
  const auto generation = p.generation.load(std::memory_order_acquire);
  if (generation != generation_) {
    const double old_duty = duty_;
    duty_ = status().duty;
    generation_ = generation;
    qos.refresh();
    if (duty_ >= 1) work_seconds_ = rest_seconds_ = 0;
    else if (old_duty < 1 && rest_seconds_ > 0)  // unpaid debt is owed at the new duty
      rest_seconds_ = std::min(rest_seconds_ * ((1 - duty_) / duty_) / ((1 - old_duty) / old_duty), kMaxRest);
  }
}
void Governor::starting_step(QosScope& qos) {
  // Idle time since the last governed step pays owed rest (it never becomes credit): debt left by a rest that a Pause
  // ended must not stall a Step or Run issued later. Settled before refresh rescales the unpaid rest to a new duty.
  if (rest_seconds_ > 0)  // only nonzero while governed: no clock read per maximum-profile step
    rest_seconds_ = std::max(0., rest_seconds_ - std::chrono::duration<double>(Clock::now() - finished_).count());
  refresh(qos);
  if (duty_ < 1) started_ = Clock::now();
}
void Governor::completed_step(QosScope& qos) {
  const double prior_duty = duty_;
  refresh(qos);
  if (duty_ < 1) {
    // Only completed solver work earns rest debt. Idle/Python time earns no credit.
    if (prior_duty < 1)
      work_seconds_ += std::chrono::duration<double>(Clock::now() - started_).count();
    // Signed rest balance: utility/background QoS timers coalesce, so a 10 ms sleep can last ~70-200 ms (measured,
    // M5). Overslept rest stays as credit against later debt; only governor-imposed rest earns it, never idle time.
    // Queued commands on this thread (RestWakeScope) end a rest; its debt is paid after the next step. Viewer frame
    // requests and telemetry samples do not end rests, so they cannot cancel the duty. Both signs are capped at one
    // observed coalescing window: a suspended sleep (SIGSTOP, system sleep) cannot buy a burst, and commands
    // arriving every step cannot bank a later stall.
    if (work_seconds_ >= .020 * duty_ || rest_seconds_ > 0) {
      rest_seconds_ = std::min(rest_seconds_ + work_seconds_ * (1 - duty_) / duty_, kMaxRest);
      work_seconds_ = 0;
      auto& r = rest();
      const auto& p = policy();
      const auto pending = [] { return rest_pending && rest_pending->load(std::memory_order_acquire); };
      while (rest_seconds_ > 0 && !pending()) {
        refresh(qos);
        if (duty_ >= 1) break;
        const auto start = Clock::now();
        {
          std::unique_lock lock(r.mutex);
          r.cv.wait_for(lock, std::chrono::duration<double>(std::min(.020, rest_seconds_)), [&] {
            return pending() || p.generation.load(std::memory_order_acquire) != generation_;
          });
        }
        rest_seconds_ -= std::chrono::duration<double>(Clock::now() - start).count();
      }
      rest_seconds_ = std::max(rest_seconds_, -kMaxRest);
    }
    finished_ = Clock::now();
  }
}
}  // namespace execution
