#pragma once
// Host: owns a World on its own thread; the only thread that ever touches the World.
// Clients submit raw napl.Envelope bytes from any thread (MPSC queue) and get the napl.Reply back.
// The queue is drained only at tick boundaries. The Host starts paused; Run{rate} free-runs one tick at
// a time paced to rate sim-seconds per wall-second (0 = as fast as possible), Pause freezes the tick
// (the thread then blocks on the queue). Run/Pause are logged like any control command, so the log
// (World::log, replay()) reproduces the session, including the ticks advanced between commands.
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include "core/world.h"

class Host {
 public:
  // Takes the World (must be fresh: the Host starts its replay log). One Session for all submitters.
  // ponytail: per-submit sessions arrive with the MCP/viewer front ends (M8+).
  Host(World world, Session session);
  ~Host() { stop(); }
  Host(const Host&) = delete;
  Host& operator=(const Host&) = delete;

  // Thread-safe. Throws std::runtime_error after stop(). A job still queued when stop() runs is dropped:
  // its future throws std::future_error (broken_promise).
  std::future<flatbuffers::DetachedBuffer> submit(std::vector<std::uint8_t> envelope);
  flatbuffers::DetachedBuffer call(std::vector<std::uint8_t> envelope) { return submit(std::move(envelope)).get(); }

  void stop();  // idempotent; joins the sim thread
  World& world();  // only after stop() (throws std::logic_error otherwise)

 private:
  struct Job {
    std::vector<std::uint8_t> env;
    std::promise<flatbuffers::DetachedBuffer> reply;
  };
  void loop();

  World world_;
  Session session_;
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<Job> q_;
  bool stop_ = false;
  std::thread thread_;  // last: starts after everything above is constructed
};
