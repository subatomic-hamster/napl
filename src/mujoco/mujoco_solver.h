#pragma once
#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "core/solver.h"

class MujocoSolver final : public Solver {
 public:
  MujocoSolver(mjModel* m, mjData* d);  // takes ownership; builds the prim registry
  ~MujocoSolver() override;

  double dt() const override { return m_->opt.timestep; }
  int step(int n) override;
  bool diverged() const override { return diverged_; }
  std::vector<Prim> prims() const override { return prims_; }
  Result get(const std::string& prim, const std::string& attr, const std::string& frame,
             std::vector<double>& out) const override;
  Result set(const std::string& prim, const std::string& attr, const std::string& frame,
             std::span<const double> values) override;
  std::string unit(const std::string& prim, const std::string& attr) const override;
  std::vector<std::byte> save() const override;
  void restore(std::span<const std::byte> bytes) override;
  std::unique_ptr<Solver> clone() const override;
  std::uint64_t state_hash() const override;
  std::vector<std::byte> model_bytes() const override;
  std::string version() const override { return std::string("mujoco ") + mj_versionString(); }

  const mjModel& model() const { return *m_; }
  const mjData& data() const { return *d_; }

 private:
  struct Ref { enum Kind { body, joint, actuator, sensor } kind; int id; };
  struct Loc { double* p; int n; bool read_only; };
  bool locate(const Ref& r, const std::string& attr, Loc& loc) const;
  bool is_servo(int a) const;  // actuator a is a position servo: kp/kv are settable

  mjModel* m_;
  mjData* d_;
  bool diverged_ = false;
  std::vector<Prim> prims_;
  std::unordered_map<std::string, Ref> index_;
};

// Installs the MuJoCo error/warning handlers (idempotent). Errors throw std::runtime_error.
void install_mujoco_handlers();
// World::Loader for MJCF/URDF. Throws std::runtime_error with MuJoCo's message on failure.
std::unique_ptr<Solver> load_mujoco(const std::string& path);
// Same, from an mj_saveModel buffer (replay-log header). Throws std::runtime_error on failure.
std::unique_ptr<Solver> load_mujoco_bytes(std::span<const std::byte> bytes);
