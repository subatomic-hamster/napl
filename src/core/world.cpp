#include "core/world.h"
#include "core/exec_policy.h"

#include <cstring>
#include <stdexcept>

void World::step(int n) {
  execution::initialize();
  execution::QosScope qos;
  thread_local execution::Governor governor;
  std::uint64_t advanced = 0;
  for (int i = 0; i < n; ++i) {
    governor.starting_step(qos);
    const int done = solver_->step(1);
    advanced += static_cast<std::uint64_t>(done);
    governor.completed_step(qos);
    if (done == 0) break;
  }
  tick_ += advanced;
  if (log_) log_->set_final_tick(tick_);
}

std::vector<std::byte> World::save() const {
  std::vector<std::byte> s = solver_->save();
  std::vector<std::byte> out(sizeof tick_ + s.size());
  std::memcpy(out.data(), &tick_, sizeof tick_);
  std::memcpy(out.data() + sizeof tick_, s.data(), s.size());
  return out;
}

void World::restore(std::span<const std::byte> bytes) {
  if (bytes.size() < sizeof tick_) throw std::runtime_error("World::restore: truncated snapshot");
  solver_->restore(bytes.subspan(sizeof tick_));  // validates before we touch the tick
  std::memcpy(&tick_, bytes.data(), sizeof tick_);
  if (log_) log_->set_final_tick(tick_);
}
