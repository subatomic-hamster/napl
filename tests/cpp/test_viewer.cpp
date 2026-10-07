// M10 exit checks: the viewer under a display (xvfb-run) with a Vulkan device (lavapipe / RADV / ANV).
//   - scripted 120 frames: ticks advance, the viewer's commands are in the replay log with source "viewer"
//   - the log replays (in process and with napl-replay) to the live run's hash
//   - the same scripted commands without any viewer give the same hash: the viewer is a pure reader
//   - error paths: no DISPLAY / bad model / bad script exit with a clear message, not a crash
#include <gtest/gtest.h>

#include <sys/wait.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "core/host.h"
#include "mujoco/mujoco_solver.h"
#include "napl_generated.h"
#include "viewer/design.h"
#include "viewer/viewer.h"

namespace {
const std::string kData = NAPL_TEST_DATA_DIR;
const std::string kModel = kData + "/pendulum.xml";

// Free-running (rate 0, then rate 1) plus Step / SetAttr: the number of ticks between commands depends on timing, so
// only the replay (which records applied ticks) is expected to reproduce it. `select` exercises the inspector's reads.
const char* kScriptRun =
    "2 select /world/pole/hinge; 5 run 0; 40 pause; 50 step 5; 60 setattr /world/pole/hinge qpos 0.3; 80 run 1; 110 pause";
// Only commands whose effect does not depend on timing: live, replayed and viewer-less runs must all agree bit for bit.
const char* kScriptDet =
    "2 select /world/pole/hinge; 3 step 5; 10 setattr /world/pole/hinge qpos 0.4; 12 select /world/pole; 20 step 20; "
    "30 setattr /actuators/torque ctrl 1.5; 40 step 100; 60 step 7; 70 setattr /world/pole/hinge qvel 0.5; 80 step 3";

Session session() {
  Session s;
  s.caps = {"read", "control"};
  s.root = kData;
  return s;
}
World pendulum() { return World(load_mujoco, kModel); }

bool have_display() { return std::getenv("DISPLAY") && *std::getenv("DISPLAY"); }
#define NEED_DISPLAY() \
  if (!have_display()) GTEST_SKIP() << "needs a display: run under xvfb-run"

struct Rec {
  std::uint64_t applied_tick;
  std::string source;
  napl::Command type;
};
template <class T>
T rd(const std::vector<std::byte>& b, std::size_t& pos) {
  T v;
  std::memcpy(&v, b.data() + pos, sizeof v);
  pos += sizeof v;
  return v;
}
// Walks the documented log layout (core/replay.h) and decodes every record's envelope.
std::vector<Rec> records(const std::vector<std::byte>& log) {
  std::size_t pos = 8 + 4 + 8;  // magic, format version, schema hash
  pos += rd<std::uint32_t>(log, pos);  // solver version (rd advanced pos past the length)
  pos += rd<std::uint64_t>(log, pos);  // model
  pos += rd<std::uint64_t>(log, pos);  // initial state
  std::vector<Rec> out;
  const std::size_t end = log.size() - kReplayFooterSize;
  while (pos < end) {
    const auto tick = rd<std::uint64_t>(log, pos);
    rd<std::uint8_t>(log, pos);  // expected error
    const auto n = rd<std::uint32_t>(log, pos);
    const napl::Envelope& e = *napl::GetEnvelope(reinterpret_cast<const std::uint8_t*>(log.data() + pos));
    out.push_back({tick, e.source() ? e.source()->str() : "", e.cmd_type()});
    pos += n;
  }
  EXPECT_EQ(pos, end);
  return out;
}

struct Outcome {
  std::uint64_t tick;
  std::uint64_t hash;
  std::vector<std::byte> log;
};
Outcome finish(Host& h) {
  h.stop();
  return {h.world().tick(), h.world().state_hash(), h.world().log()->bytes()};
}
Outcome with_viewer(const std::string& script, int frames) {
  Host h(pendulum(), session());
  ViewerOptions o;
  o.frames = frames;
  o.script = script;
  o.width = 640;
  o.height = 480;
  run_viewer(h, o);
  return finish(h);
}
// The same scripted commands, submitted in order straight to a Host: no window, no viewer.
Outcome without_viewer(const std::string& script) {
  Host h(pendulum(), session());
  for (const ScriptCmd& c : parse_script(script))
    if (c.action != "select") h.call(script_envelope(c, "viewer"));
  return finish(h);
}
std::string hex(std::uint64_t v) {
  char b[17];
  std::snprintf(b, sizeof b, "%016llx", static_cast<unsigned long long>(v));
  return b;
}

struct Out {
  int code;
  std::string text;  // stdout + stderr
};
Out shell(const std::string& cmd) {
  Out o{-1, ""};
  FILE* p = popen((cmd + " 2>&1").c_str(), "r");
  if (!p) return o;
  char buf[512];
  while (std::fgets(buf, sizeof buf, p)) o.text += buf;
  const int st = pclose(p);
  o.code = WIFEXITED(st) ? WEXITSTATUS(st) : 128 + WTERMSIG(st);  // >= 128: killed by a signal (a crash)
  return o;
}
std::string field(const std::string& text, const std::string& key) {  // "key=value" token
  const auto at = text.find(key + "=");
  if (at == std::string::npos) return "";
  const auto b = at + key.size() + 1;
  return text.substr(b, text.find_first_of(" \n", b) - b);
}
}  // namespace

TEST(viewer_smoke, scripted_120_frames_logged_as_viewer_and_replays_to_live_hash) {
  NEED_DISPLAY();
  const Outcome live = with_viewer(kScriptRun, 120);
  const std::vector<Rec> recs = records(live.log);
  std::cout << "[ viewer ] 120 frames: ticks=" << live.tick << " records=" << recs.size() << " live hash=" << hex(live.hash) << "\n";
  EXPECT_GT(live.tick, 100u) << "ticks must advance while the viewer runs the sim";

  // Run, Pause, Step, SetAttr, Run, Pause: all state-changing commands came from the viewer; its reads are not logged.
  const std::vector<napl::Command> want = {napl::Command::Run,     napl::Command::Pause, napl::Command::Step,
                                           napl::Command::SetAttr, napl::Command::Run,   napl::Command::Pause};
  ASSERT_EQ(recs.size(), want.size());
  std::size_t viewer_records = 0;
  for (std::size_t i = 0; i < recs.size(); ++i) {
    EXPECT_EQ(recs[i].type, want[i]) << "record " << i;
    if (recs[i].source == "viewer") ++viewer_records;
    if (i) EXPECT_GE(recs[i].applied_tick, recs[i - 1].applied_tick);
  }
  EXPECT_EQ(viewer_records, recs.size());
  std::cout << "[ viewer ] source=viewer records: " << viewer_records << "/" << recs.size() << "\n";

  const World r = replay(live.log, load_mujoco_bytes, load_mujoco, kData);  // no viewer, no Host thread
  std::cout << "[ viewer ] replay (no viewer): tick=" << r.tick() << " hash=" << hex(r.state_hash()) << "\n";
  EXPECT_EQ(r.tick(), live.tick);
  EXPECT_EQ(r.state_hash(), live.hash);
}

TEST(viewer_smoke, same_commands_without_viewer_give_the_same_hash) {
  NEED_DISPLAY();
  const Outcome live = with_viewer(kScriptDet, 120);
  const Outcome bare = without_viewer(kScriptDet);
  const World r = replay(live.log, load_mujoco_bytes, load_mujoco, kData);
  std::cout << "[ viewer ] deterministic script: live=" << hex(live.hash) << " (tick " << live.tick << ")  no-viewer=" << hex(bare.hash)
            << " (tick " << bare.tick << ")  replay=" << hex(r.state_hash()) << "\n";
  EXPECT_EQ(live.tick, 135u);  // 5 + 20 + 100 + 7 + 3
  EXPECT_EQ(bare.tick, live.tick);
  EXPECT_EQ(bare.hash, live.hash);
  EXPECT_EQ(r.state_hash(), live.hash);
  EXPECT_EQ(records(live.log).size(), 8u);  // 5 steps + 3 setattrs; the inspector's queries are reads
}

TEST(viewer_smoke, napl_view_cli_log_replays_with_napl_replay) {
  NEED_DISPLAY();
  std::filesystem::create_directories(NAPL_VIEWER_OUT_DIR);
  const std::string log = std::string(NAPL_VIEWER_OUT_DIR) + "/viewer_smoke.nlog";
  const Out v = shell(std::string(NAPL_VIEW_EXE) + " " + kModel + " --frames 120 --script '" + kScriptRun + "' --log " + log);
  ASSERT_EQ(v.code, 0) << v.text;
  const Out r = shell(std::string(NAPL_CLI_EXE) + " " + log);
  ASSERT_EQ(r.code, 0) << r.text;
  std::cout << "[ viewer ] napl-view : " << v.text << "[ viewer ] napl-replay: " << r.text;
  EXPECT_EQ(field(v.text, "state_hash"), field(r.text, "state_hash"));
  EXPECT_EQ(field(v.text, "tick"), field(r.text, "tick"));
  EXPECT_FALSE(field(v.text, "state_hash").empty());
}

// Not window tests: they must hold with or without a display, so they never skip.
TEST(viewer_smoke, no_display_is_a_clear_error_not_a_crash) {
  const Out o = shell(std::string("env -u DISPLAY -u WAYLAND_DISPLAY ") + NAPL_VIEW_EXE + " " + kModel + " --frames 3");
  std::cout << "[ viewer ] no DISPLAY: exit " << o.code << ": " << o.text;
  EXPECT_EQ(o.code, 1);
  EXPECT_NE(o.text.find("napl-view: error: cannot open a window"), std::string::npos) << o.text;
  EXPECT_NE(o.text.find("DISPLAY"), std::string::npos) << o.text;
}

TEST(viewer_smoke, bad_model_bad_script_and_usage_exit_cleanly) {
  const std::string exe = NAPL_VIEW_EXE;
  Out o = shell(exe + " /no/such/model.xml --frames 1");
  EXPECT_EQ(o.code, 1) << o.text;
  EXPECT_NE(o.text.find("napl-view: error:"), std::string::npos) << o.text;
  o = shell(exe + " " + kModel + " --script '5 bogus'");  // rejected before any window exists
  EXPECT_EQ(o.code, 1) << o.text;
  EXPECT_NE(o.text.find("unknown action 'bogus'"), std::string::npos) << o.text;
  o = shell(exe);
  EXPECT_EQ(o.code, 2) << o.text;
  EXPECT_NE(o.text.find("usage:"), std::string::npos) << o.text;
}

TEST(viewer_smoke, script_parser_and_envelopes) {
  const std::vector<ScriptCmd> s = parse_script("5 run 0.5; 7 setattr /a/b qpos 1 2; 9 pause;");
  ASSERT_EQ(s.size(), 3u);
  EXPECT_EQ(s[1].frame, 7);
  EXPECT_EQ(s[1].args.size(), 4u);
  const std::vector<std::uint8_t> e = script_envelope(s[0], "viewer");
  const napl::Envelope& env = *napl::GetEnvelope(e.data());
  EXPECT_EQ(env.cmd_type(), napl::Command::Run);
  EXPECT_DOUBLE_EQ(env.cmd_as_Run()->rate(), 0.5);
  EXPECT_EQ(env.source()->str(), "viewer");
  EXPECT_THROW(parse_script("x run 1"), std::runtime_error);
  EXPECT_THROW(parse_script("1 step"), std::runtime_error);
  EXPECT_THROW(parse_script("1 setattr /a b"), std::runtime_error);
  EXPECT_EQ(parse_script("3 inst arm").at(0).action, "inst");
  EXPECT_THROW(parse_script("3 inst"), std::runtime_error);
}

// ---- Design window (report of a `napl design build`) -------------------------------------------------------------------
const std::string kReport = kData + "/design_demo.report.json";

TEST(viewer_design, report_parses_and_instances_resolve_to_body_prims) {  // no window needed
  const design::Report r = design::load_report(kReport);
  EXPECT_EQ(r.assembly, "pendulum_demo");
  EXPECT_NEAR(r.mass_total, 1.0025, 1e-12);
  ASSERT_EQ(r.instances.size(), 2u);
  EXPECT_EQ(r.instances[0].part, "demo-rod");
  EXPECT_EQ(r.instances[0].body, "pole");
  EXPECT_NEAR(r.instances[1].mass, 2.5e-3, 1e-15);
  ASSERT_EQ(r.by_group.size(), 1u);
  EXPECT_EQ(r.by_group[0].first, "frame");
  ASSERT_EQ(r.targets.size(), 1u);
  EXPECT_FALSE(r.targets[0].within);
  ASSERT_EQ(r.findings.size(), 3u);
  EXPECT_EQ(r.findings[0].severity, "error");
  EXPECT_EQ(r.findings[0].code, "DEMO_ERROR");
  EXPECT_NE(r.findings[0].message.find("\xC3\xA9"), std::string::npos) << "\\u00e9 decodes to UTF-8";
  EXPECT_EQ(r.findings[0].where, (std::vector<std::string>{"port x", "arm"}));
  const std::vector<std::string> prims = {"/world", "/world/base", "/world/base/pole", "/world/base/pole/hinge"};
  EXPECT_EQ(design::body_path(prims, "pole"), "/world/base/pole");
  EXPECT_EQ(design::body_path(prims, "ole"), "");
  EXPECT_EQ(design::body_path(prims, "ghost"), "");
  EXPECT_THROW(design::parse_report("{\"assembly\": "), std::runtime_error);
  EXPECT_THROW(design::load_report("/no/such/report.json"), std::runtime_error);
}

TEST(viewer_design, window_opens_with_a_report_and_instance_clicks_do_not_touch_the_sim) {
  NEED_DISPLAY();
  const std::string script = "3 inst arm; 10 inst ghost; 20 inst nothing; 30 step 4";
  Host h(pendulum(), session());
  ViewerOptions o;
  o.frames = 60;
  o.script = script;
  o.design_report = kReport;
  run_viewer(h, o);
  const Outcome with = finish(h);
  Host bare(pendulum(), session());
  bare.call(script_envelope(parse_script("0 step 4").at(0), "viewer"));
  const Outcome ref = finish(bare);
  std::cout << "[ viewer ] design window, 60 frames: tick=" << with.tick << " hash=" << hex(with.hash) << "\n";
  EXPECT_EQ(with.tick, 4u);  // only the Step ran: selecting instances is UI state, not a bus command
  EXPECT_EQ(with.hash, ref.hash);
  EXPECT_EQ(records(with.log).size(), 1u);
}

TEST(viewer_design, napl_view_finds_the_report_beside_the_model_and_rejects_a_bad_one) {
  NEED_DISPLAY();
  const Out ok = shell(std::string(NAPL_VIEW_EXE) + " " + kModel + " --frames 30 --design " + kReport + " --script '2 inst arm'");
  EXPECT_EQ(ok.code, 0) << ok.text;
  const Out bad = shell(std::string(NAPL_VIEW_EXE) + " " + kModel + " --frames 3 --design /no/such.report.json");
  EXPECT_EQ(bad.code, 1) << bad.text;
  EXPECT_NE(bad.text.find("design report: cannot read"), std::string::npos) << bad.text;
}
