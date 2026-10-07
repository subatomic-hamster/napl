#include "core/host.h"

#include <chrono>
#include <stdexcept>

#include "napl_generated.h"

Host::Host(World world, Session session) : world_(std::move(world)), session_(std::move(session)) {
  world_.start_log();
  thread_ = std::thread([this] { loop(); });
}

std::future<flatbuffers::DetachedBuffer> Host::submit(std::vector<std::uint8_t> envelope) {
  Job j{std::move(envelope), {}};
  auto f = j.reply.get_future();
  {
    std::lock_guard lk(m_);
    if (stop_) throw std::runtime_error("Host is stopped");
    q_.push_back(std::move(j));
  }
  cv_.notify_one();
  return f;
}

void Host::stop() {
  {
    std::lock_guard lk(m_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

World& Host::world() {
  if (thread_.joinable()) throw std::logic_error("Host::world() requires stop() first");
  return world_;
}

void Host::loop() {
  using clock = std::chrono::steady_clock;
  bool running = false;
  double rate = 0;
  clock::time_point t0;
  std::uint64_t base = 0;  // pacing: tick `base` was due at t0; tick base+k is due at t0 + k*dt/rate
  auto anchor = [&] {
    t0 = clock::now();
    base = world_.tick();
  };

  std::unique_lock lk(m_);
  for (;;) {
    if (stop_) return;
    while (!q_.empty()) {  // tick boundary: drain
      Job j = std::move(q_.front());
      q_.pop_front();
      lk.unlock();
      const std::uint64_t tick0 = world_.tick();
      flatbuffers::DetachedBuffer reply = world_.execute(j.env, session_);
      if (flatbuffers::GetRoot<napl::Reply>(reply.data())->ok()) {  // execute verified the envelope
        const napl::Envelope& env = *napl::GetEnvelope(j.env.data());
        if (env.cmd_type() == napl::Command::Run) {
          running = true;
          rate = env.cmd_as_Run()->rate();
          anchor();
        } else if (env.cmd_type() == napl::Command::Pause) {
          running = false;
        }
      }
      if (world_.tick() != tick0) anchor();  // Step/Restore/LoadModel moved the clock: no catch-up burst
      j.reply.set_value(std::move(reply));
      lk.lock();
    }
    if (!running) {
      cv_.wait(lk, [&] { return stop_ || !q_.empty(); });
      continue;
    }
    if (rate > 0) {
      const auto due = t0 + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(
                                static_cast<double>(world_.tick() + 1 - base) * world_.solver().dt() / rate));
      if (cv_.wait_until(lk, due, [&] { return stop_ || !q_.empty(); })) continue;  // woken early: drain first
    }
    lk.unlock();
    world_.step(1);
    lk.lock();
    if (world_.diverged()) running = false;  // a diverged solver no longer advances: do not spin
  }
}
