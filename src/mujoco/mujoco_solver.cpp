#include "mujoco/mujoco_solver.h"

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>

namespace {
constexpr int kStateSig = mjSTATE_INTEGRATION;

[[noreturn]] void on_error(const char* msg) { throw std::runtime_error(std::string("mujoco: ") + msg); }
// Not throwing: divergence (mjWARN_BADQACC etc.) is detected from d->warning counters in step();
// a throwing handler would abort mid-step instead.
void on_warning(const char* msg) { std::fprintf(stderr, "mujoco warning: %s\n", msg); }
}  // namespace

void install_mujoco_handlers() {  // call_once: Python threads load models concurrently
  static std::once_flag once;
  std::call_once(once, [] {
    mju_user_error = on_error;
    mju_user_warning = on_warning;
  });
}

std::unique_ptr<Solver> load_mujoco(const std::string& path) {
  install_mujoco_handlers();
  char err[1000] = "";
  mjModel* m = mj_loadXML(path.c_str(), nullptr, err, sizeof err);
  if (!m) throw std::runtime_error("mj_loadXML failed for '" + path + "': " + err);
  mjData* d = mj_makeData(m);  // no thread pool is ever bound: stepping is single-threaded
  if (!d) {
    mj_deleteModel(m);
    throw std::runtime_error("mj_makeData failed for '" + path + "'");
  }
  return std::make_unique<MujocoSolver>(m, d);
}

std::unique_ptr<Solver> load_mujoco_bytes(std::span<const std::byte> bytes) {
  install_mujoco_handlers();
  if (bytes.empty() || bytes.size() > static_cast<std::size_t>(INT_MAX))
    throw std::runtime_error("model buffer has invalid size");
  mjModel* m = mj_loadModelBuffer(bytes.data(), static_cast<int>(bytes.size()));
  if (!m) throw std::runtime_error("mj_loadModelBuffer failed (corrupt or incompatible model bytes)");
  mjData* d = mj_makeData(m);
  if (!d) {
    mj_deleteModel(m);
    throw std::runtime_error("mj_makeData failed for model buffer");
  }
  return std::make_unique<MujocoSolver>(m, d);
}

std::vector<std::byte> MujocoSolver::model_bytes() const {
  std::vector<std::byte> b(static_cast<std::size_t>(mj_sizeModel(m_)));
  mj_saveModel(m_, nullptr, b.data(), static_cast<int>(b.size()));
  return b;
}

MujocoSolver::~MujocoSolver() {
  mj_deleteData(d_);
  mj_deleteModel(m_);
}

int MujocoSolver::step(int n) {
  int done = 0;
  while (done < n && !diverged_) {
    mj_step(m_, d_);
    ++done;
    if (d_->warning[mjWARN_BADQACC].number > 0 || d_->warning[mjWARN_BADQPOS].number > 0 ||
        d_->warning[mjWARN_BADQVEL].number > 0) {
      diverged_ = true;
    }
    for (int i = 0; i < m_->nq && !diverged_; ++i) diverged_ = !std::isfinite(d_->qpos[i]);
  }
  return done;
}

namespace {
std::string elem_name(const mjModel* m, int type, int id, const char* fallback) {
  const char* n = mj_id2name(m, type, id);
  return n && *n ? std::string(n) : fallback + std::to_string(id);  // unnamed: stable index-based name
}
bool is_world(const std::string& f) { return f.empty() || f == "world"; }
}  // namespace

MujocoSolver::MujocoSolver(mjModel* m, mjData* d) : m_(m), d_(d) {
  auto add = [&](std::string path, const char* kind, Ref::Kind k, int id) {
    index_[path] = {k, id};
    prims_.push_back({std::move(path), kind});
  };
  std::vector<std::string> body_path(m_->nbody);  // parents precede children in MuJoCo's body order
  for (int i = 0; i < m_->nbody; ++i) {
    body_path[i] = i == 0 ? "/world" : body_path[m_->body_parentid[i]] + "/" + elem_name(m_, mjOBJ_BODY, i, "body");
    add(body_path[i], "body", Ref::body, i);
  }
  for (int j = 0; j < m_->njnt; ++j)
    add(body_path[m_->jnt_bodyid[j]] + "/" + elem_name(m_, mjOBJ_JOINT, j, "joint"), "joint", Ref::joint, j);
  for (int a = 0; a < m_->nu; ++a)
    add("/actuators/" + elem_name(m_, mjOBJ_ACTUATOR, a, "actuator"), "actuator", Ref::actuator, a);
  for (int s = 0; s < m_->nsensor; ++s)
    add("/sensors/" + elem_name(m_, mjOBJ_SENSOR, s, "sensor"), "sensor", Ref::sensor, s);
}

// A position servo has a fixed gain and an affine bias with biasprm[1] == -gainprm[0] (what <position> compiles to).
// Motors (no bias) and velocity actuators (biasprm[1] == 0) are not servos. kp = 0 is still a servo (and stays one).
bool MujocoSolver::is_servo(int a) const {
  return m_->actuator_gaintype[a] == mjGAIN_FIXED && m_->actuator_biastype[a] == mjBIAS_AFFINE &&
         m_->actuator_biasprm[a * mjNBIAS + 1] == -m_->actuator_gainprm[a * mjNGAIN];
}

// Attribute table for the non-body prims: where the values live in mjData. Body attrs are derived (see get).
bool MujocoSolver::locate(const Ref& r, const std::string& attr, Loc& loc) const {
  switch (r.kind) {
    case Ref::joint: {
      const int type = m_->jnt_type[r.id];
      if (attr == "qpos") {
        loc = {d_->qpos + m_->jnt_qposadr[r.id], type == mjJNT_FREE ? 7 : type == mjJNT_BALL ? 4 : 1, false};
        return true;
      }
      if (attr == "qvel") {
        loc = {d_->qvel + m_->jnt_dofadr[r.id], type == mjJNT_FREE ? 6 : type == mjJNT_BALL ? 3 : 1, false};
        return true;
      }
      return false;
    }
    case Ref::actuator:
      if (attr != "ctrl") return false;
      loc = {d_->ctrl + r.id, 1, false};
      return true;
    case Ref::sensor:
      if (attr != "value") return false;
      loc = {d_->sensordata + m_->sensor_adr[r.id], m_->sensor_dim[r.id], true};
      return true;
    default:
      return false;
  }
}

Result MujocoSolver::get(const std::string& prim, const std::string& attr, const std::string& frame,
                         std::vector<double>& out) const {
  auto it = index_.find(prim);
  if (it == index_.end()) return {Result::not_found, "unknown prim '" + prim + "'"};
  const Ref r = it->second;
  out.clear();
  if (r.kind == Ref::actuator && (attr == "kp" || attr == "kv")) {
    if (!is_world(frame)) return {Result::invalid_argument, "frame only applies to body attributes"};
    if (!is_servo(r.id)) return {Result::invalid_argument, prim + " is not a position servo (no kp/kv)"};
    out.push_back(attr == "kp" ? m_->actuator_gainprm[r.id * mjNGAIN] : -m_->actuator_biasprm[r.id * mjNBIAS + 2]);
    return {};
  }
  if (r.kind == Ref::body) {
    if (attr != "pos" && attr != "quat") return {Result::not_found, "unknown attr '" + attr + "' on body"};
    int fb = -1;  // frame body; -1 = world
    if (!is_world(frame)) {
      auto f = index_.find(frame);
      if (f == index_.end() || f->second.kind != Ref::body)
        return {Result::not_found, "unknown frame '" + frame + "' (need a body prim path or \"world\")"};
      fb = f->second.id;
    }
    mj_kinematics(m_, d_);  // xpos/xquat lag one step behind qpos after mj_step; derived data only, not in the hash
    if (attr == "pos") {
      mjtNum v[3];
      mju_copy3(v, d_->xpos + 3 * r.id);
      if (fb >= 0) {
        mjtNum diff[3];
        mju_sub3(diff, v, d_->xpos + 3 * fb);
        mju_mulMatTVec3(v, d_->xmat + 9 * fb, diff);  // R^T (p - p_b)
      }
      out.assign(v, v + 3);
    } else {
      mjtNum q[4];
      mju_copy4(q, d_->xquat + 4 * r.id);
      if (fb >= 0) {
        mjtNum inv[4];
        mju_negQuat(inv, d_->xquat + 4 * fb);
        mju_mulQuat(q, inv, d_->xquat + 4 * r.id);
      }
      out.assign(q, q + 4);
    }
    return {};
  }
  if (!is_world(frame)) return {Result::invalid_argument, "frame only applies to body attributes"};
  Loc loc;
  if (!locate(r, attr, loc)) return {Result::not_found, "unknown attr '" + attr + "' on " + prim};
  out.assign(loc.p, loc.p + loc.n);
  return {};
}

Result MujocoSolver::set(const std::string& prim, const std::string& attr, const std::string& frame,
                         std::span<const double> values) {
  auto it = index_.find(prim);
  if (it == index_.end()) return {Result::not_found, "unknown prim '" + prim + "'"};
  const Ref r = it->second;
  if (r.kind == Ref::body) {
    if (attr == "pos" || attr == "quat") return {Result::invalid_argument, "attr '" + attr + "' is read-only"};
    return {Result::not_found, "unknown attr '" + attr + "' on body"};
  }
  if (!is_world(frame)) return {Result::invalid_argument, "frame only applies to body attributes"};
  if (r.kind == Ref::actuator && (attr == "kp" || attr == "kv")) {
    if (!is_servo(r.id)) return {Result::invalid_argument, prim + " is not a position servo (no kp/kv)"};
    if (values.size() != 1 || !std::isfinite(values[0]) || values[0] < 0)
      return {Result::invalid_argument, "attr '" + attr + "' takes one finite value >= 0"};
    // MuJoCo position actuator: force = kp*(ctrl - q) - kv*qdot  =>  gainprm[0] = kp, biasprm[1] = -kp, biasprm[2] = -kv.
    if (attr == "kp") {
      m_->actuator_gainprm[r.id * mjNGAIN] = values[0];
      m_->actuator_biasprm[r.id * mjNBIAS + 1] = -values[0];
    } else {
      m_->actuator_biasprm[r.id * mjNBIAS + 2] = -values[0];
    }
    return {};
  }
  Loc loc;
  if (!locate(r, attr, loc)) return {Result::not_found, "unknown attr '" + attr + "' on " + prim};
  if (loc.read_only) return {Result::invalid_argument, "attr '" + attr + "' is read-only"};
  if (static_cast<int>(values.size()) != loc.n)
    return {Result::invalid_argument, "attr '" + attr + "' takes " + std::to_string(loc.n) + " values, got " +
                                          std::to_string(values.size())};
  for (double v : values)
    if (!std::isfinite(v)) return {Result::invalid_argument, "values must be finite"};
  std::memcpy(loc.p, values.data(), values.size_bytes());
  if (r.kind == Ref::joint) mj_forward(m_, d_);  // refresh derived state after a state edit
  return {};
}

// SI units, one table. ctrl is "1": the raw actuator input; its physical unit depends on gear/transmission.
// Mixed-unit attrs (free-joint qpos/qvel) and unlisted sensor types give "".
std::string MujocoSolver::unit(const std::string& prim, const std::string& attr) const {
  auto it = index_.find(prim);
  if (it == index_.end()) return "";
  const Ref r = it->second;
  switch (r.kind) {
    case Ref::body:
      return attr == "pos" ? "m" : attr == "quat" ? "1" : "";
    case Ref::joint: {
      const int t = m_->jnt_type[r.id];
      if (attr == "qpos") return t == mjJNT_HINGE ? "rad" : t == mjJNT_SLIDE ? "m" : t == mjJNT_BALL ? "1" : "";
      if (attr == "qvel") return t == mjJNT_SLIDE ? "m/s" : t == mjJNT_FREE ? "" : "rad/s";
      return "";
    }
    case Ref::actuator: {
      if (attr == "ctrl") return "1";
      if ((attr != "kp" && attr != "kv") || !is_servo(r.id)) return "";
      // Gains are per ctrl unit (ctrl is the target joint position) only for a unit-gear joint transmission.
      if (m_->actuator_trntype[r.id] != mjTRN_JOINT || m_->actuator_gear[r.id * 6] != 1) return "";
      const int t = m_->jnt_type[m_->actuator_trnid[r.id * 2]];
      if (t == mjJNT_SLIDE) return attr == "kp" ? "N/m" : "N*s/m";
      return t == mjJNT_HINGE ? (attr == "kp" ? "N*m/rad" : "N*m*s/rad") : "";
    }
    case Ref::sensor: {
      if (attr != "value") return "";
      const bool slide = (m_->sensor_type[r.id] == mjSENS_JOINTPOS || m_->sensor_type[r.id] == mjSENS_JOINTVEL) &&
                         m_->jnt_type[m_->sensor_objid[r.id]] == mjJNT_SLIDE;
      switch (m_->sensor_type[r.id]) {
        case mjSENS_JOINTPOS: return slide ? "m" : "rad";
        case mjSENS_JOINTVEL: return slide ? "m/s" : "rad/s";
        case mjSENS_FRAMEPOS: return "m";
        case mjSENS_FRAMELINVEL: case mjSENS_VELOCIMETER: return "m/s";
        case mjSENS_FRAMEANGVEL: case mjSENS_GYRO: return "rad/s";
        case mjSENS_ACCELEROMETER: return "m/s^2";
        case mjSENS_FORCE: case mjSENS_TOUCH: return "N";
        case mjSENS_TORQUE: return "N*m";
        default: return "";
      }
    }
  }
  return "";
}

// Snapshot = mjSTATE_INTEGRATION followed by every model parameter a SetAttr can change (today: the actuator
// gain/bias arrays holding kp/kv). Model parameters are not in mjSTATE_INTEGRATION, so without this block
// restore() would silently keep a later kp. Add any new settable model attribute here too. state_hash() covers it.
std::vector<std::byte> MujocoSolver::save() const {
  const int ns = mj_stateSize(m_, kStateSig), np = m_->nu * (mjNGAIN + mjNBIAS);
  std::vector<mjtNum> s(static_cast<std::size_t>(ns + np));
  mj_getState(m_, d_, s.data(), kStateSig);
  mju_copy(s.data() + ns, m_->actuator_gainprm, m_->nu * mjNGAIN);
  mju_copy(s.data() + ns + m_->nu * mjNGAIN, m_->actuator_biasprm, m_->nu * mjNBIAS);
  std::vector<std::byte> out(s.size() * sizeof(mjtNum));
  std::memcpy(out.data(), s.data(), out.size());
  return out;
}

void MujocoSolver::restore(std::span<const std::byte> bytes) {
  const int ns = mj_stateSize(m_, kStateSig);
  if (bytes.size() != sizeof(mjtNum) * static_cast<std::size_t>(ns + m_->nu * (mjNGAIN + mjNBIAS)))
    throw std::runtime_error("MujocoSolver::restore: snapshot size mismatch");
  std::vector<mjtNum> s(bytes.size() / sizeof(mjtNum));  // copy: span bytes may be unaligned
  std::memcpy(s.data(), bytes.data(), bytes.size());
  mju_copy(m_->actuator_gainprm, s.data() + ns, m_->nu * mjNGAIN);
  mju_copy(m_->actuator_biasprm, s.data() + ns + m_->nu * mjNGAIN, m_->nu * mjNBIAS);
  mj_setState(m_, d_, s.data(), kStateSig);
  mj_forward(m_, d_);
  // step() reads these sticky counters: a restored healthy state must not inherit the divergence it replaced.
  for (mjWarningStat& w : d_->warning) w.number = 0;
  diverged_ = false;
}

std::unique_ptr<Solver> MujocoSolver::clone() const {
  mjModel* m = mj_copyModel(nullptr, m_);
  mjData* d = mj_copyData(nullptr, m, d_);
  auto c = std::make_unique<MujocoSolver>(m, d);
  c->diverged_ = diverged_;
  return c;
}

std::uint64_t MujocoSolver::state_hash() const {
  std::vector<std::byte> s = save();
  std::uint64_t h = 14695981039346656037ull;  // FNV-1a 64
  for (std::byte b : s) h = (h ^ static_cast<std::uint64_t>(b)) * 1099511628211ull;
  return h;
}
