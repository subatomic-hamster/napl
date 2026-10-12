#include "viewer/run_ui.h"

#include <algorithm>
#include <cstdio>

#include "viewer/theme.h"

namespace runui {
namespace {
std::string base(std::string s) {
  if (const auto k = s.rfind('/'); k != std::string::npos) s = s.substr(k + 1);
  if (s.ends_with(".scenario.toml")) s.resize(s.size() - 14);
  return s;
}
}  // namespace

std::string RunUi::chip() const { return running() ? "running" : verdict_; }

void RunUi::note(const std::string& text, const char* colour) {
  prog_.output_append(std::string(colour) + text + (*colour ? "\x1b[0m" : "") + "\r\n");
}

void RunUi::toggle() {
  if (running()) stop();
  else start();
}

bool RunUi::command(const std::string& name) {
  if (name == "bench" || name == "full") full_ = name == "full";
  else if (name == "run") start();
  else if (name == "stop") stop();
  else if (name.rfind("target:", 0) == 0) (full_ ? full_pick_ : bench_pick_) = name.substr(7);
  else return false;
  return true;
}

void RunUi::start() {
  if (running()) return;
  std::string args;
  if (full_) {
    const std::string sc = !full_pick_.empty() ? full_pick_ : scenarios_.empty() ? "" : scenarios_.front();
    if (sc.empty()) return;
    target_label_ = base(sc);
    args = "{\"mode\":\"full\",\"scenario\":" + design::quote(sc) + "}";
  } else {
    std::string id = bench_pick_;
    if (id.empty() && selected) id = selected();
    const auto has = [&](const std::string& i) { return std::any_of(comps_.begin(), comps_.end(), [&](const Comp& c) { return c.id == i; }); };
    if (!has(id)) id = comps_.empty() ? "" : comps_.front().id;
    if (id.empty()) return;
    target_label_ = id;
    const std::string f = prog_.active_path();
    args = "{\"mode\":\"bench\",\"component\":" + design::quote(id) + (f.empty() || f[0] == '@' || f.rfind("ext:", 0) == 0 ? "" : ",\"files\":[" + design::quote(f) + "]") + "}";
  }
  prog_.save_all();  // queued before run.start: the run sees the editor's text
  prog_.output_clear();
  prog_.show_output();
  state_ = State::Starting;
  verdict_.clear();
  stop_sent_ = false;
  send("run.start", design::request("run.start", args));
}

void RunUi::stop() {
  if (!running() || stop_sent_) return;
  if (state_ == State::Starting) return;  // the run id is not known yet; the button works again in a moment
  stop_sent_ = true;
  send("run.stop", design::request("run.stop", "{\"run_id\":" + design::quote(run_id_) + "}"));
}

bool RunUi::poll(double now, bool page_visible, std::string* kind, std::string* json) {
  if (page_visible && !was_visible_) targets_wanted_ = true;  // (re)read the component / scenario lists when the page opens
  was_visible_ = page_visible;
  if (targets_wanted_ && page_visible && !targets_flight_) {
    targets_wanted_ = false, targets_flight_ = true;
    *kind = "run.targets", *json = design::request("run.targets");
    return true;
  }
  if (state_ == State::Running && !poll_flight_ && now - last_poll_ >= 0.2) {
    last_poll_ = now, poll_flight_ = true;
    *kind = "run.poll", *json = design::request("run.poll", "{\"run_id\":" + design::quote(run_id_) + "}");
    return true;
  }
  return false;
}

void RunUi::on_reply(const std::string& kind, const design::Json& r) {
  const bool ok = r.at("ok").num != 0;
  const design::Json& res = r.at("result");
  if (kind == "run.targets") {
    targets_flight_ = false;
    if (!ok) return;
    scenarios_.clear(), comps_.clear();
    for (const design::Json& s : res.at("scenarios").arr) scenarios_.push_back(s.str);
    for (const design::Json& c : res.at("components").arr) comps_.push_back({c.s("id"), c.s("part"), c.s("kind")});
  } else if (kind == "run.start") {
    if (!ok) {
      note("run: " + r.s("error"), "\x1b[31m");
      state_ = State::Done, verdict_ = "failed";
      return;
    }
    run_id_ = res.s("run_id");
    state_ = State::Running;
    last_poll_ = 0;
  } else if (kind == "run.poll") {
    poll_flight_ = false;
    if (state_ != State::Running) return;
    if (!ok) {
      note("run: " + r.s("error"), "\x1b[31m");
      state_ = State::Done, verdict_ = "failed";
      return;
    }
    if (const std::string out = res.s("out"); !out.empty()) {
      std::string crlf;  // the OUTPUT view is a terminal: a bare \n would not return the carriage
      for (std::size_t i = 0; i < out.size(); ++i) {
        if (out[i] == '\n' && (i == 0 || out[i - 1] != '\r')) crlf += '\r';
        crlf += out[i];
      }
      prog_.output_append(crlf);
    }
    if (res.s("state") != "running") finish(res);
  } else if (kind == "run.stop" && !ok) {
    note("stop: " + r.s("error"), "\x1b[31m");
  }
}

void RunUi::finish(const design::Json& r) {
  const std::string st = r.s("state");
  const design::Json& result = r.at("result");
  const bool pass = result.at("pass").kind == design::Json::Bool || result.at("pass").kind == design::Json::Num ? result.at("pass").num != 0 : false;
  state_ = State::Done;
  verdict_ = st == "stopped" ? "stopped" : st == "failed" ? "failed" : pass ? "PASS" : "FAIL";
  std::string line = "== " + verdict_ + "  " + (full_ ? "full: " : "bench: ") + target_label_;
  int n = 0;
  for (const auto& [k, v] : result.at("metrics").obj) {
    if (v.kind != design::Json::Num || n++ >= 4) continue;
    char b[64];
    std::snprintf(b, sizeof b, "  %s=%.4g", k.c_str(), v.num);
    line += b;
  }
  if (!result.s("run_dir").empty()) line += "  run dir " + result.s("run_dir");
  note(line, verdict_ == "PASS" ? "\x1b[32m" : verdict_ == "stopped" ? "\x1b[33m" : "\x1b[31m");
}

void RunUi::draw(ImVec2 /*size*/) {
  const bool busy = running();
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {4, 0});
  ImGui::BeginDisabled(busy);
  if (theme::button("BENCH", !full_, {52, 0})) full_ = false;
  ImGui::SetItemTooltip("the selected component's device model on a short bench, then the tests of the open file");
  ImGui::SameLine();
  if (theme::button("FULL SIM", full_, {66, 0})) full_ = true;
  ImGui::SetItemTooltip("the scenario's program, live in this window's 3D view");
  ImGui::EndDisabled();
  ImGui::SameLine();
  // target combo
  std::string cur;
  if (full_) cur = base(!full_pick_.empty() ? full_pick_ : scenarios_.empty() ? "" : scenarios_.front());
  else {
    cur = bench_pick_;
    if (cur.empty() && selected) cur = selected();
    if (std::none_of(comps_.begin(), comps_.end(), [&](const Comp& c) { return c.id == cur; })) cur = comps_.empty() ? "" : comps_.front().id;
  }
  ImGui::BeginDisabled(busy);
  ImGui::SetNextItemWidth(118);
  if (ImGui::BeginCombo("##runtarget", cur.empty() ? "(none)" : cur.c_str())) {
    if (full_) {
      for (const std::string& s : scenarios_)
        if (ImGui::Selectable(base(s).c_str(), base(s) == cur)) full_pick_ = s;
    } else {
      for (const Comp& c : comps_) {
        const std::string lab = c.id + (c.kind.empty() ? "   (tests only)" : "   " + c.kind);
        if (ImGui::Selectable(lab.c_str(), c.id == cur)) bench_pick_ = c.id;
      }
    }
    ImGui::EndCombo();
  }
  ImGui::EndDisabled();
  ImGui::SetItemTooltip(full_ ? "scenario to run (*.scenario.toml in the program root)" : "component to bench (defaults to the one selected in BUILD)");
  ImGui::SameLine();
  const bool can = busy || !cur.empty();
  ImGui::BeginDisabled(!can || (busy && state_ == State::Starting));
  if (theme::button(busy ? "Stop" : "Run", !busy, {50, 0})) toggle();
  ImGui::EndDisabled();
  ImGui::SetItemTooltip(busy ? "stop the run (kills the program)" : "save all, then run");
  if (const std::string c = chip(); !c.empty()) {
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(c == "PASS" ? theme::kOk : c == "running" ? theme::kAccent : c == "stopped" ? theme::kWarn : theme::kErr, "%s", c.c_str());
  }
  ImGui::PopStyleVar();
}

}  // namespace runui
