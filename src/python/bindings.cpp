// _napl: thin nanobind layer over World. All logic stays in C++; python/napl/__init__.py adds only
// dict<->JSON and numpy conveniences. Every call that can run long releases the GIL; a World is
// single-threaded, so one Sim must not be used from two threads at once (different Sims are independent).
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <climits>
#include <filesystem>
#include <stdexcept>
#include <tuple>

#include "core/world.h"
#include "mujoco/mujoco_solver.h"
#include "napl_generated.h"
#include "napl_schema_json.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace {
struct Sim {
  World w;
  Session s;

  Sim(const std::string& path, const std::string& root, int max_sweep_runs, double max_wall_s,
      std::vector<std::string> caps, int max_steps_per_call)
      : w(load_mujoco, path) {
    s.caps = std::move(caps);  // plain Python: all caps (trusted caller); the MCP layer passes read+control
    s.max_steps_per_call = max_steps_per_call;  // plain Python: INT_MAX; the MCP layer passes a finite limit
    s.max_sweep_runs = max_sweep_runs;  // plain Python trusts its caller: generous defaults (see Sim.load)
    s.max_wall_s = max_wall_s;
    s.root = root.empty() ? std::filesystem::path(path).parent_path() : std::filesystem::path(root);
  }
  Sim(World&& world, Session session) : w(std::move(world)), s(std::move(session)) {}
};

using Array = nb::ndarray<nb::numpy, double>;
}  // namespace

NB_MODULE(_napl, m) {
  install_mujoco_handlers();  // before any thread exists

  m.def("command_cap", [](const std::string& command) { return command_cap(command); }, "command"_a);
  m.attr("schema_json") = kSchemaJson;  // flatc --jsonschema output, embedded at build time

  nb::class_<Sim>(m, "Sim")
      .def("__init__",
           [](Sim* self, const std::string& path, const std::string& root, int max_sweep_runs, double max_wall_s,
              std::vector<std::string> caps, int max_steps_per_call) {
             nb::gil_scoped_release nogil;
             new (self) Sim(path, root, max_sweep_runs, max_wall_s, std::move(caps), max_steps_per_call);  // throws std::runtime_error (-> RuntimeError) with MuJoCo's message
           },
           "path"_a, "root"_a = "", "max_sweep_runs"_a = 100000, "max_wall_s"_a = 0.0,
           "caps"_a = std::vector<std::string>{"read", "control", "destructive"}, "max_steps_per_call"_a = INT_MAX)
      // Step goes through the bus (not World::step) so it is logged and replayable.
      .def("step",
           [](Sim& s, int n) {
             flatbuffers::FlatBufferBuilder fbb;
             fbb.Finish(napl::CreateEnvelope(fbb, 0, fbb.CreateString("python"), napl::Command::Step,
                                             napl::CreateStep(fbb, n).Union()));
             flatbuffers::DetachedBuffer reply;
             {
               nb::gil_scoped_release nogil;
               reply = s.w.execute({fbb.GetBufferPointer(), fbb.GetSize()}, s.s);
             }
             const napl::Reply* r = flatbuffers::GetRoot<napl::Reply>(reply.data());
             if (!r->ok()) throw std::runtime_error(r->message()->str());
             return r->tick();
           },
           "n"_a = 1)
      .def_prop_ro("tick", [](Sim& s) { return s.w.tick(); })
      .def_prop_ro("diverged", [](Sim& s) { return s.w.diverged(); })
      .def("call",
           [](Sim& s, const std::string& json, bool elevate) {
             Session session = s.s;
             if (elevate) session.caps = {"read", "control", "destructive"};  // one call, after the caller's approval
             std::string out;
             {
               nb::gil_scoped_release nogil;
               out = execute_json(s.w, json, session);
             }
             return out;
           },
           "json"_a, "elevate"_a = false)
      .def_prop_ro("caps", [](Sim& s) { return s.s.caps; })
      // Direct solver read (reads are never logged and change no state). Writes must use call().
      .def("get",
           [](Sim& s, const std::string& path, const std::string& attr, const std::string& frame) {
             std::vector<double> v;
             const Result r = s.w.solver().get(path, attr, frame, v);
             if (r.code == Result::not_found) throw nb::key_error(r.message.c_str());
             if (r.code != Result::ok) throw nb::value_error(r.message.c_str());
             auto* data = new double[v.size()];
             std::copy(v.begin(), v.end(), data);
             nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<double*>(p); });
             return std::make_tuple(Array(data, {v.size()}, owner), s.w.solver().unit(path, attr));
           },
           "path"_a, "attr"_a, "frame"_a = "")
      .def("save", [](Sim& s) {
        const auto b = s.w.save();
        return nb::bytes(reinterpret_cast<const char*>(b.data()), b.size());
      })
      .def("restore",
           [](Sim& s, nb::bytes b) {
             s.w.restore({reinterpret_cast<const std::byte*>(b.c_str()), b.size()});  // throws on malformed
           },
           "state"_a)
      .def("state_hash", [](Sim& s) { return s.w.state_hash(); })
      .def("clone", [](Sim& s) {
        nb::gil_scoped_release nogil;
        return Sim(s.w.clone(), s.s);
      })
      .def("start_log", [](Sim& s) { s.w.start_log(); })
      .def("log", [](Sim& s) -> nb::object {
        if (!s.w.log()) return nb::none();
        const auto b = s.w.log()->bytes();
        return nb::bytes(reinterpret_cast<const char*>(b.data()), b.size());
      });
}
