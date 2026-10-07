#pragma once
// The viewer: a GLFW window with a Vulkan swapchain (WindowRenderer) and a Dear ImGui overlay (scene tree, inspector with
// SI units, pause/run/step/rate). It runs on the calling thread -- GLFW needs the main thread on macOS -- while the Host's
// own thread runs the simulation. The viewer never touches the World: it draws Host::take_frame() snapshots, and every
// UI action is an ordinary bus command (napl.Envelope with source "viewer") sent through Host::submit(), so it is
// capability-checked, logged and replayable like any other client's.
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "core/host.h"

struct ViewerOptions {
  int frames = 0;                           // > 0: leave after this many frames (smoke tests); 0: until the window closes
  std::string script;                       // UI-equivalent commands issued at given frames, see parse_script
  std::string design_report;                // optional <name>.report.json of a `napl design build`: shows the Design window
  int width = 1280, height = 720;
  const std::atomic<bool>* stop = nullptr;  // optional: another thread sets it to close the viewer
};

// Throws std::runtime_error with a plain-language reason: no display, no Vulkan loader or device, window creation
// failed. Returns when the window is closed, `frames` frames were drawn, or *stop is set; commands still in flight
// are answered before it returns.
void run_viewer(Host& host, const ViewerOptions& opts);

// Script syntax: "FRAME ACTION [ARGS...]; ..." with ACTION one of
//   run RATE | pause | step N | setattr PATH ATTR V [V...]   (bus commands, see script_envelope)
//   select PATH                                              (no command: shows that prim in the inspector)
//   inst ID                                                  (no command: a Design-window click on that instance)
// executed (in order) when that frame is reached, exactly like the matching UI control would.
struct ScriptCmd {
  int frame = 0;
  std::string action;
  std::vector<std::string> args;
};
std::vector<ScriptCmd> parse_script(const std::string& script);  // throws std::runtime_error naming the bad entry
// The napl.Envelope bytes of one scripted command (what the viewer submits); throws std::runtime_error on bad arguments.
std::vector<std::uint8_t> script_envelope(const ScriptCmd& cmd, const std::string& source = "viewer");
