#pragma once
// The ONE solver interface (seam). napl_core must never include a solver's headers.
// visuals/poses/raycast arrive in later milestones.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

struct Prim {
  std::string path;  // "/world/<body>[/<child>...]", "/world/<body>/<joint>", "/actuators/<n>", "/sensors/<n>"
  std::string kind;  // "body" | "joint" | "actuator" | "sensor"
};

// Outcome of get/set; the bus maps it to Reply.error.
struct Result {
  enum Code { ok, not_found, invalid_argument } code = ok;
  std::string message;
};

class Solver {
 public:
  virtual ~Solver() = default;

  virtual double dt() const = 0;
  // Advance up to n steps; stops early on divergence. Returns steps actually taken.
  virtual int step(int n) = 0;
  virtual bool diverged() const = 0;  // sticky

  virtual std::vector<Prim> prims() const = 0;
  // frame: "" or "world", or a body prim path. SI units; attribute table lives in the solver.
  virtual Result get(const std::string& prim, const std::string& attr, const std::string& frame,
                     std::vector<double>& out) const = 0;
  virtual Result set(const std::string& prim, const std::string& attr, const std::string& frame,
                     std::span<const double> values) = 0;
  virtual std::string unit(const std::string& prim, const std::string& attr) const = 0;  // "" if unknown

  virtual std::vector<std::byte> save() const = 0;
  virtual void restore(std::span<const std::byte> bytes) = 0;  // throws on size mismatch
  virtual std::unique_ptr<Solver> clone() const = 0;
  virtual std::uint64_t state_hash() const = 0;

  // Replay-log header support: the loaded model as a self-contained buffer (a BytesLoader rebuilds a
  // solver from it) and a version string; replay refuses logs whose version differs.
  virtual std::vector<std::byte> model_bytes() const = 0;
  virtual std::string version() const = 0;
};
