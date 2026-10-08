#pragma once
// The 2D "flatten" wiring canvas. Components picked in the Design window are shown as cards (name + the chosen terminals as
// labelled, colour-coded pins); dragging pin -> pin wires them. The viewer holds no wiring logic: every action is a
// `flatten.*` design_api request (python/napl/design/wire2d.py, listed there) and the canvas draws the state the replies and
// the rev poll bring back -- so a scripted agent driving the same ops through napl.design.viewer.Backend sees the UI react:
// cards pop in, a wire is drawn with a drag animation.
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "viewer/design.h"

namespace wire2d {

struct Pin {
  std::string name, kind, type, gender, role, side;  // side: "left" | "right"
  int row = 0;
  bool wired = false;
};
struct Card {
  std::string id, part, group;
  float x = 0, y = 0, w = 200, h = 60;  // canvas units
  std::vector<Pin> pins;
  double born = 0;  // ImGui time the card pops in
};
struct Wire {
  std::string name, from, to, part, kind;
  double awg = 0, length = 0;
  int conductors = 1;
  double born = -1e9;  // ImGui time its draw animation starts (far past: no animation)
};

class Canvas {
 public:
  // send(tool, args_json): queues one design_api request (the viewer's request queue); replies come back to on_reply.
  using Send = std::function<void(const std::string& tool, const std::string& args, bool quiet)>;
  explicit Canvas(Send send) : send_(std::move(send)) {}

  bool active() const { return active_; }
  // Once per frame inside the ImGui frame: the selection window (3D mode) or the canvas overlay (flattened).
  // `selected`: the instance selected in the Design window ("" = none).
  void ui(const design::State& st, const std::string& selected);
  // The reply (parsed JSON) of a flatten.* request, failed ones included.
  void on_reply(const std::string& tool, const design::Json& reply);
  // From the viewer's rev poll: the backend's canvas revision; a change fetches the state.
  void poll(int flat_rev);

 private:
  struct Drag {
    enum Kind { none, on_pin, on_card } kind = none;
    std::string card, pin;  // pin drag: where it started
    float dx = 0, dy = 0;   // card drag: grab offset (canvas units)
  };
  void apply(const design::Json& f);
  void select_ui(const design::State& st, const std::string& selected);
  void canvas_ui();
  void side_ui();
  void fit();
  const Card* card(const std::string& id) const;
  bool pin_pos(const Card& c, const Pin& p, float& x, float& y) const;
  void toast(const std::string& s, bool bad);

  Send send_;
  bool active_ = false;            // the backend's flatten mode
  double alpha_ = 0;               // 0..1, eased: the canvas overlay fades in / out
  int seen_rev_ = -1;
  std::vector<Card> cards_;
  std::vector<Wire> wires_;
  int hidden_wires_ = 0;
  std::string sel_wire_, hover_wire_;
  Drag drag_;
  float zoom_ = 1, ox_ = 40, oy_ = 40;  // canvas -> screen: screen = o + canvas * zoom
  bool fit_pending_ = false;
  std::string toast_;
  bool toast_bad_ = false;
  double toast_t_ = -1e9;
  double new_wire_slot_ = 0;       // staggers the draw animations of wires that arrive together
  // selection (3D mode)
  std::map<std::string, std::set<std::string>> pick_;  // instance -> chosen terminals
  char filter_[64] = "";
  char len_buf_[32] = "";
  int awg_idx_ = 5;
};

}  // namespace wire2d
