#pragma once
// The Test / Run controls of the PROGRAM page (drawn in ProgramUi::run_slot) and the state of the run behind them. The backend
// (python/napl/design/runops.py) owns the runs: run.targets / run.start / run.poll / run.stop ride the viewer's design queue
// (`send`); this side picks the mode (BENCH = the selected component's device model + the tests of the open file, FULL SIM = the
// scenario's program live in this window's World), starts, polls every ~200 ms, and streams the output into the OUTPUT tab.
#include <functional>
#include <string>
#include <vector>

#include <imgui.h>

#include "design.h"
#include "viewer/program_ui.h"

namespace runui {

class RunUi {
 public:
  explicit RunUi(progui::ProgramUi& prog) : prog_(prog) {}

  // ---- wiring (set by the viewer) -------------------------------------------------------------------------------------
  std::function<void(const std::string& kind, const std::string& json)> send;  // queue a design request (the kind comes back to on_reply)
  std::function<std::string()> selected;  // id of the instance selected in BUILD / WIRING ("" = none)

  // ---- called by the viewer -------------------------------------------------------------------------------------------
  void draw(ImVec2 size);                                         // inside ProgramUi's run slot
  void on_reply(const std::string& kind, const design::Json& r);  // kinds "run.*"
  bool poll(double now, bool page_visible, std::string* kind, std::string* json);  // targets / run.poll when due
  bool command(const std::string& name);  // scripts / tests: "bench", "full", "run", "stop", "target:<id|scenario>"; false = not a Run command
  void toggle();                                                // Run, or Stop while running (the SIMULATE ribbon)
  bool running() const { return state_ == State::Starting || state_ == State::Running; }
  bool live() const { return running() && full_; }               // a program drives this window's World
  std::string live_label() const { return "LIVE: " + target_label_; }
  std::string chip() const;                                       // "", "running", "PASS", "FAIL", "stopped"

 private:
  enum class State { Idle, Starting, Running, Done };
  struct Comp {
    std::string id, part, kind;
  };
  void start();
  void stop();
  void finish(const design::Json& r);
  void note(const std::string& text, const char* colour = "");

  progui::ProgramUi& prog_;
  bool full_ = false;                    // false = BENCH, true = FULL SIM
  std::vector<Comp> comps_;
  std::vector<std::string> scenarios_;
  std::string bench_pick_, full_pick_;   // the user's choice ("" = the default)
  State state_ = State::Idle;
  std::string run_id_, target_label_, verdict_;  // verdict_: PASS / FAIL / failed / stopped of the last run
  bool targets_wanted_ = true, targets_flight_ = false, poll_flight_ = false, was_visible_ = false, stop_sent_ = false;
  double last_poll_ = 0;
};

}  // namespace runui
