#pragma once
// The single call between the viewer's WIRING tab and the 2D "flatten" wiring mode (src/viewer/wire2d.cpp, the LOOK worker's).
// viewer.cpp builds this struct and, when wire2d.cpp is part of the build (CMake defines NAPL_HAS_WIRE2D if the file exists),
// calls wire2d_draw(slot) from inside an ImGui window that covers `pos` / `size`; otherwise it shows a placeholder.
#include <imgui.h>

#include <functional>
#include <string>

#include "viewer/design.h"

struct Wire2dSlot {
  ImVec2 pos, size;                // content rectangle in window coordinates
  const design::State* state;      // report (instances), ports per instance (kind / type / used, world pos), geo
  std::string selected;            // selected instance id ("" none)
  // Queue a request for the editing backend, exactly the JSON of design::request(tool, args) (e.g. an autowire edit:
  // design::request("design_edit", "{\"ops\":[{\"op\":\"autowire\",\"a\":\"esc1.signal\"}]}")). Applied in order; the state
  // above is refreshed with the reply.
  std::function<void(const std::string& request_json)> request;
};
void wire2d_draw(const Wire2dSlot& slot);  // defined by wire2d.cpp
