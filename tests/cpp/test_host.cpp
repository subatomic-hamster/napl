// Host tests: real-time pacing, rate 0, pause, concurrent submitters, replay of a session with Run periods.
// Timing checks use the median of several windows (the CI box is shared and noisy); thresholds are the plan's.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "core/exec_policy.h"
#include "core/host.h"
#include "mujoco/mujoco_solver.h"
#include "napl_generated.h"

namespace {
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Env = std::vector<std::uint8_t>;

const std::string kData = NAPL_TEST_DATA_DIR;
const std::string kHinge = "/world/pole/hinge";
constexpr double kDt = 0.002;  // tests/data/pendulum.xml timestep

Session all_caps() {
  Session s;
  s.caps = {"read", "control", "destructive"};
  s.root = kData;
  return s;
}
World pendulum() { return World(load_mujoco, kData + "/pendulum.xml"); }

Env finish(flatbuffers::FlatBufferBuilder& fbb, std::uint64_t id, napl::Command t, flatbuffers::Offset<void> cmd) {
  fbb.Finish(napl::CreateEnvelope(fbb, id, 0, t, cmd));
  return Env(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize());
}
Env run(double rate) {
  flatbuffers::FlatBufferBuilder f;
  return finish(f, 1, napl::Command::Run, napl::CreateRun(f, rate).Union());
}
Env pause_cmd() {
  flatbuffers::FlatBufferBuilder f;
  return finish(f, 1, napl::Command::Pause, napl::CreatePause(f).Union());
}
Env step(int n, std::uint64_t id = 1) {
  flatbuffers::FlatBufferBuilder f;
  return finish(f, id, napl::Command::Step, napl::CreateStep(f, n).Union());
}
Env query_prims(std::uint64_t id = 1) {  // empty path: lists prims; the reply carries the tick
  flatbuffers::FlatBufferBuilder f;
  return finish(f, id, napl::Command::Query, napl::CreateQueryDirect(f).Union());
}
Env setattr(double v, std::uint64_t id = 1) {
  flatbuffers::FlatBufferBuilder f;
  const std::vector<double> vals{v};
  return finish(f, id, napl::Command::SetAttr, napl::CreateSetAttrDirect(f, kHinge.c_str(), "qpos", "", &vals).Union());
}

struct Rep {
  napl::Err err;
  std::uint64_t tick, id;
};
Rep call(Host& h, Env e) {
  const flatbuffers::DetachedBuffer b = h.call(std::move(e));
  const napl::Reply* r = flatbuffers::GetRoot<napl::Reply>(b.data());
  return {r->error(), r->tick(), r->id()};
}
Rep call(World& w, const Env& e) {
  const flatbuffers::DetachedBuffer b = w.execute(e, all_caps());
  const napl::Reply* r = flatbuffers::GetRoot<napl::Reply>(b.data());
  return {r->error(), r->tick(), r->id()};
}

struct Sample {
  Clock::time_point wall;
  std::uint64_t tick;
};
Sample sample(Host& h) {
  const Rep r = call(h, query_prims());
  return {Clock::now(), r.tick};
}
double sim_per_wall(const Sample& a, const Sample& b) {
  return static_cast<double>(b.tick - a.tick) * kDt / std::chrono::duration<double>(b.wall - a.wall).count();
}

std::uint64_t footer_count(const std::vector<std::byte>& log) {
  std::uint64_t n;
  std::memcpy(&n, log.data() + log.size() - kReplayFooterSize, 8);
  return n;
}
}  // namespace

// Pacing logic at full responsiveness. Under background QoS the pacing wakes coalesce (Apple default profile): the
// overall rate stays ~0.99 but 0.4 s windows measured medians up to 1.07, outside this band by design of that profile.
TEST(host, realtime_rate1_tracks_wall_time) {
  struct Restore {
    std::string previous = execution::status().requested;
    ~Restore() { execution::configure(previous); }
  } restore;
  execution::configure("maximum");
  if (execution::status().effective != "maximum")
    GTEST_SKIP() << "live telemetry restricts the maximum profile (" << execution::status().reason << ")";
  Host h(pendulum(), all_caps());
  ASSERT_EQ(call(h, run(1.0)).err, napl::Err::none);
  std::vector<Sample> s{sample(h)};
  for (int i = 0; i < 6; ++i) {
    std::this_thread::sleep_for(400ms);
    s.push_back(sample(h));
  }
  std::vector<double> ratios;
  for (std::size_t i = 1; i < s.size(); ++i) ratios.push_back(sim_per_wall(s[i - 1], s[i]));
  std::sort(ratios.begin(), ratios.end());
  const double median = ratios[ratios.size() / 2];
  std::cout << "[ rate=1 ] sim/wall per 0.4 s window: min=" << ratios.front() << " median=" << median
            << " max=" << ratios.back() << " overall=" << sim_per_wall(s.front(), s.back()) << "\n";
  EXPECT_NEAR(median, 1.0, 0.05);
}

TEST(host, rate0_runs_far_faster_than_realtime) {
  Host h(pendulum(), all_caps());
  ASSERT_EQ(call(h, run(0)).err, napl::Err::none);
  const Sample a = sample(h);
  std::this_thread::sleep_for(300ms);
  const Sample b = sample(h);
  ASSERT_EQ(call(h, pause_cmd()).err, napl::Err::none);
  const double x = sim_per_wall(a, b);
  std::cout << "[ rate=0 ] " << x << "x real time (" << b.tick - a.tick << " ticks)\n";
  EXPECT_GE(x, 50.0);
}

// Background QoS coalesces timers: a governor rest that is a plain 20 ms sleep lasted ~180 ms, so Pause during a
// rate-0 Run took ~180 ms (p95). Host work ends the rest at once; its unpaid debt is paid after the next step.
TEST(host, background_profile_pause_is_observed_promptly) {
  struct Restore {
    std::string previous = execution::status().requested;
    ~Restore() { execution::configure(previous); }
  } restore;
  execution::configure("background");
  std::vector<double> ms;
  {
    Host h(pendulum(), all_caps());
    for (int i = 0; i < 60; ++i) {
      ASSERT_EQ(call(h, run(0)).err, napl::Err::none);
      std::this_thread::sleep_for(30ms);
      const auto t0 = Clock::now();
      ASSERT_EQ(call(h, pause_cmd()).err, napl::Err::none);
      ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    }
  }
  std::sort(ms.begin(), ms.end());
  std::cout << "[ background pause ] p50=" << ms[ms.size() / 2] << " ms max=" << ms.back() << " ms\n";
  EXPECT_LT(ms[(ms.size() * 95 + 99) / 100 - 1], 100.0);  // nearest-rank p95
}

// A job's own enqueue left the Host's pending-work flag set while the job ran, so every rest inside a Step command
// (Sim.step on a HostSim, the viewer STEP button) ended at once: Host-run steps ignored the profile duty.
TEST(host, background_profile_governs_steps_inside_host_jobs) {
  struct Restore {
    std::string previous = execution::status().requested;
    ~Restore() { execution::configure(previous); }
  } restore;
  execution::configure("background");
  if (execution::status().duty != .5)
    GTEST_SKIP() << "live telemetry changed the background duty (" << execution::status().reason << ")";
  constexpr int kSteps = 400000;
  const auto seconds = [](auto&& fn) {
    const auto t0 = Clock::now();
    fn();
    return std::chrono::duration<double>(Clock::now() - t0).count();
  };
  World bare = pendulum();
  const double direct = seconds([&] { bare.step(kSteps); });  // governed on this thread at the same QoS
  Host h(pendulum(), all_caps());
  const double in_job = seconds([&] { h.exec([](World& w) { w.step(kSteps); }); });
  std::cout << "[ background step ] direct=" << direct << " s in Host job=" << in_job << " s\n";
  EXPECT_GT(in_job / direct, .7);
}

TEST(host, paused_freezes_tick_and_step_advances_exactly) {
  Host h(pendulum(), all_caps());  // starts paused
  const Sample a = sample(h);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(sample(h).tick, a.tick);
  EXPECT_EQ(call(h, step(5)).tick, a.tick + 5);
  EXPECT_EQ(sample(h).tick, a.tick + 5);

  ASSERT_EQ(call(h, run(0)).err, napl::Err::none);
  std::this_thread::sleep_for(20ms);
  const std::uint64_t at_pause = call(h, pause_cmd()).tick;  // Pause is applied at a tick boundary
  EXPECT_GT(at_pause, a.tick + 5);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(sample(h).tick, at_pause);
  EXPECT_EQ(call(h, step(3)).tick, at_pause + 3);
}

TEST(host, four_threads_250_commands_all_applied) {
  Host h(pendulum(), all_caps());  // paused: exact tick accounting
  std::atomic<std::uint64_t> logged{0}, steps{0}, bad{0};
  std::vector<std::thread> ts;
  for (int t = 0; t < 4; ++t)
    ts.emplace_back([&, t] {
      for (int j = 0; j < 250; ++j) {
        const std::uint64_t id = static_cast<std::uint64_t>(t) * 1000 + static_cast<std::uint64_t>(j);
        Env e;
        switch (j % 3) {
          case 0: e = setattr(0.01 * j, id); ++logged; break;
          case 1: e = step(1 + j % 4, id); ++logged; steps += static_cast<std::uint64_t>(1 + j % 4); break;
          default: e = query_prims(id);
        }
        const Rep r = call(h, std::move(e));
        if (r.err != napl::Err::none || r.id != id) ++bad;
      }
    });
  for (auto& t : ts) t.join();
  EXPECT_EQ(bad.load(), 0u);
  h.stop();
  EXPECT_EQ(h.world().tick(), steps.load());
  EXPECT_EQ(footer_count(h.world().log()->bytes()), logged.load());
  std::cout << "[ 4x250 ] replies=1000 logged=" << logged << " ticks=" << steps << "\n";
}

TEST(host, recorded_log_with_run_periods_replays_to_live_state) {
  Host h(pendulum(), all_caps());
  std::vector<std::thread> ts;
  ASSERT_EQ(call(h, run(0)).err, napl::Err::none);
  for (int t = 0; t < 4; ++t)  // concurrent writers while the sim free-runs
    ts.emplace_back([&, t] {
      for (int j = 0; j < 60; ++j) {
        call(h, setattr(0.005 * (t + 1) * j));
        call(h, query_prims());
        if (j % 10 == 0) call(h, step(2));
      }
    });
  for (auto& t : ts) t.join();
  EXPECT_EQ(call(h, run(-1)).err, napl::Err::invalid_argument);  // logged with its error, replays to the same error
  ASSERT_EQ(call(h, pause_cmd()).err, napl::Err::none);
  ASSERT_EQ(call(h, run(50)).err, napl::Err::none);              // a paced period too
  std::this_thread::sleep_for(60ms);
  ASSERT_EQ(call(h, run(0)).err, napl::Err::none);
  const std::uint64_t last_cmd_tick = call(h, query_prims()).tick;  // reads are not logged
  std::this_thread::sleep_for(50ms);                                // ticks after the last logged command
  h.stop();

  World& live = h.world();
  EXPECT_GT(live.tick(), last_cmd_tick);
  const std::vector<std::byte> log = live.log()->bytes();
  World r = replay(log, load_mujoco_bytes, load_mujoco, kData);
  std::cout << "[ replay ] records=" << footer_count(log) << " live tick=" << live.tick() << " replayed tick=" << r.tick() << "\n";
  EXPECT_EQ(r.tick(), live.tick());
  EXPECT_EQ(r.state_hash(), live.state_hash());
  EXPECT_EQ(r.save(), live.save());
  World again = replay(log, load_mujoco_bytes, load_mujoco, kData, /*record=*/true);
  EXPECT_TRUE(again.log()->bytes() == log) << "re-recorded host log differs";
}

TEST(host, errors_and_bare_world_behaviour) {
  Host h(pendulum(), all_caps());
  EXPECT_EQ(call(h, run(-0.5)).err, napl::Err::invalid_argument);
  EXPECT_EQ(call(h, run(std::nan(""))).err, napl::Err::invalid_argument);
  const std::uint64_t t = call(h, query_prims()).tick;
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(sample(h).tick, t) << "a rejected Run must not start the clock";
  h.stop();
  EXPECT_THROW(h.submit(query_prims()), std::runtime_error);

  // Bare World (no Host): Run/Pause validate and are otherwise no-ops (documented in napl.fbs / bus.cpp).
  World w = pendulum();
  EXPECT_EQ(call(w, run(2.0)).err, napl::Err::none);
  EXPECT_EQ(call(w, pause_cmd()).err, napl::Err::none);
  EXPECT_EQ(call(w, run(-1)).err, napl::Err::invalid_argument);
  EXPECT_EQ(w.tick(), 0u);
}
