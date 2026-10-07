// napl-view <model> [--frames N] [--script "FRAME ACTION ARGS; ..."] [--log OUT] [--root DIR]
// Opens the model (MJCF/URDF) in the viewer. --frames N: leave after N frames; --script: UI-equivalent commands issued at
// given frames (see viewer/viewer.h); --log: write the session's replay log on exit (napl-replay reproduces it).
// Prints the final tick and state hash, in napl-replay's format.
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <string>

#include "core/host.h"
#include "mujoco/mujoco_solver.h"
#include "viewer/viewer.h"

namespace {
int usage() {
  std::fprintf(stderr, "usage: napl-view <model> [--frames N] [--script \"FRAME ACTION ARGS; ...\"] [--log OUT] [--root DIR]\n");
  return 2;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) return usage();
  const std::string model = argv[1];
  ViewerOptions opts;
  std::string log_path, root;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (i + 1 >= argc) return usage();
    const std::string v = argv[++i];
    if (a == "--frames") opts.frames = std::atoi(v.c_str());
    else if (a == "--script") opts.script = v;
    else if (a == "--log") log_path = v;
    else if (a == "--root") root = v;
    else return usage();
  }
  try {
    parse_script(opts.script);  // a bad script fails before anything else happens
    Session session;
    session.caps = {"read", "control"};
    session.root = root.empty() ? std::filesystem::path(model).parent_path() : std::filesystem::path(root);
    Host host(World(load_mujoco, model), session);
    try {
      run_viewer(host, opts);
    } catch (...) {
      host.stop();
      throw;
    }
    host.stop();
    if (!log_path.empty()) host.world().log()->write(log_path);
    std::printf("tick=%" PRIu64 " state_hash=%016" PRIx64 "\n", host.world().tick(), host.world().state_hash());
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "napl-view: error: %s\n", e.what());
    return 1;
  }
}
