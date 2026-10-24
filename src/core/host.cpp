#include "core/host.h"
#include "core/exec_policy.h"

#include <chrono>
#include <stdexcept>

#include "napl_generated.h"

Host::Host(World world, Session session)
    : world_(std::move(world)), session_(std::move(session)), elevated_(session_) {
  elevated_.caps = {"read", "control", "destructive"};
  if (!world_.log()) {
    try {
      world_.start_log();
    } catch (const std::logic_error&) {  // snapshots exist (Sim.show on a used Sim): their ids could not replay; run unlogged
    }
  }
  thread_ = std::thread([this] { loop(); });
}

std::future<flatbuffers::DetachedBuffer> Host::enqueue(Job j) {
  auto f = j.reply.get_future();
  {
    std::lock_guard lk(m_);
    if (stop_) throw std::runtime_error("Host is stopped");
    q_.push_back(std::move(j));
    has_work_.store(true, std::memory_order_release);
  }
  cv_.notify_one();
  execution::wake_rests();  // a governed step resting under coalesced QoS timers must not delay the command
  return f;
}

std::future<flatbuffers::DetachedBuffer> Host::submit(std::vector<std::uint8_t> envelope, bool elevate) {
  Job j;
  j.env = std::move(envelope);
  j.elevate = elevate;
  return enqueue(std::move(j));
}

void Host::exec(std::function<void(World&)> fn) {
  Job j;
  j.fn = std::move(fn);
  enqueue(std::move(j)).get();
}

bool Host::take_frame(Frame& out) {
  bool got = false;
  {
    std::lock_guard lk(m_);
    if (have_frame_) {
      std::swap(out, back_);
      have_frame_ = false;
      got = true;
    }
    want_frame_ = true;
  }
  cv_.notify_one();  // no wake_rests: frame requests must not cancel the profile duty; the frame follows the rest
  return got;
}

void Host::stop() {
  {
    std::lock_guard lk(m_);
    stop_ = true;
    has_work_.store(true, std::memory_order_release);
  }
  cv_.notify_all();
  execution::wake_rests();
  if (thread_.joinable()) thread_.join();
}

World& Host::world() {
  if (thread_.joinable()) throw std::logic_error("Host::world() requires stop() first");
  return world_;
}

// Sim thread. Pure reads of the World (poses/visuals are derived data, never hashed).
Frame Host::snapshot(bool running, double rate) {
  Frame f;
  f.tick = world_.tick();
  f.scene = scene_;
  f.dt = world_.solver().dt();
  f.running = running;
  f.rate = rate;
  f.diverged = world_.diverged();
  f.poses = world_.solver().poses();
  f.tubes = world_.solver().tubes();
  f.tube_ends = world_.solver().tube_ends();
  f.weather = world_.solver().weather();
  if (!scene_sent_) {
    f.new_scene = true;
    f.visuals = world_.solver().visuals();
    scene_sent_ = true;
  }
  return f;
}

void Host::loop() {
  execution::initialize();
  execution::thread_entry();
  const execution::RestWakeScope wake_on_work(&has_work_);
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
      // Commands queued behind this job cannot run before it returns: they must not end its governed rests.
      const execution::RestWakeScope in_job(nullptr);
      const std::uint64_t tick0 = world_.tick();
      if (j.fn) {
        try {
          j.fn(world_);
          j.reply.set_value({});
        } catch (...) {
          j.reply.set_exception(std::current_exception());
        }
      } else {
        flatbuffers::DetachedBuffer reply = world_.execute(j.env, j.elevate ? elevated_ : session_);
        if (flatbuffers::GetRoot<napl::Reply>(reply.data())->ok()) {  // execute verified the envelope
          const napl::Envelope& env = *napl::GetEnvelope(j.env.data());
          if (env.cmd_type() == napl::Command::Run) {
            running = true;
            rate = env.cmd_as_Run()->rate();
            anchor();
          } else if (env.cmd_type() == napl::Command::Pause) {
            running = false;
          } else if (env.cmd_type() == napl::Command::LoadModel || env.cmd_type() == napl::Command::Spawn ||
                     env.cmd_type() == napl::Command::Delete) {  // geometry changed: the viewer needs the new scene
            ++scene_;
            scene_sent_ = false;
          }
        }
        j.reply.set_value(std::move(reply));
      }
      if (world_.tick() != tick0) anchor();  // Step/Restore/LoadModel moved the clock: no catch-up burst
      lk.lock();
    }
    has_work_.store(stop_, std::memory_order_release);  // drained (under m_)
    if (want_frame_) {  // viewer asked: snapshot at this tick boundary
      lk.unlock();
      Frame f = snapshot(running, rate);
      lk.lock();
      back_ = std::move(f);
      have_frame_ = true;
      want_frame_ = false;
    }
    const auto wake = [&] { return stop_ || !q_.empty() || want_frame_; };
    if (!running) {
      cv_.wait(lk, wake);
      continue;
    }
    if (rate > 0) {
      const auto due = t0 + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(
                                static_cast<double>(world_.tick() + 1 - base) * world_.solver().dt() / rate));
      if (cv_.wait_until(lk, due, wake)) continue;  // woken early: drain first
    }
    lk.unlock();
    world_.step(1);
    lk.lock();
    if (world_.diverged()) running = false;  // a diverged solver no longer advances: do not spin
  }
}
