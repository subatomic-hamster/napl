// Device framework: config parse / bind, per-tick pipeline, state block, prim attributes. Physics: electrical.cpp, aero.cpp.
#include "mujoco/devices/devices.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string_view>
#include <type_traits>

#include "devices_bfbs_generated.h"
#include "devices_generated.h"
#include "flatbuffers/idl.h"

namespace fb = napl::dev;  // generated config tables (devices.fbs)

namespace devs {

// ---- RNG ----
namespace {
std::uint64_t splitmix64(std::uint64_t& x) {
  std::uint64_t z = (x += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}
std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
}  // namespace

void Rng::seed(std::uint64_t x) {
  for (std::uint64_t& w : s) w = splitmix64(x);  // splitmix64 never yields an all-zero xoshiro state
}
std::uint64_t Rng::next() {
  const std::uint64_t r = rotl(s[1] * 5, 7) * 9, t = s[1] << 17;
  s[2] ^= s[0];
  s[3] ^= s[1];
  s[1] ^= s[2];
  s[0] ^= s[3];
  s[2] ^= t;
  s[3] = rotl(s[3], 45);
  return r;
}
double Rng::uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
double Rng::gauss() {
  const double u1 = 1.0 - uniform(), u2 = uniform();  // u1 in (0, 1]: log is finite
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

const std::string& Devices::name(int slot) const {
  const Slot& s = slots[static_cast<std::size_t>(slot)];
  switch (s.type) {
    case Slot::battery: return battery.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::psu: return psu.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::wire: return wire.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::load: return load.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::rotor: return rotor.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::env: return env.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::drag: return drag.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::servo: return servo.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::imu: return imu.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::baro: return baro.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::mag: return mag.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::gnss: return gnss.cfg[static_cast<std::size_t>(s.i)].meta.name;
    case Slot::esc: break;
  }
  return esc.cfg[static_cast<std::size_t>(s.i)].meta.name;
}

// ---- parse ----
std::optional<std::string> config_text(const mjModel* m) {
  for (int i = 0; i < m->ntext; ++i) {
    const char* n = mj_id2name(m, mjOBJ_TEXT, i);
    if (n && std::string_view(n) == "napl.devices") return std::string(m->text_data + m->text_adr[i]);
  }
  return std::nullopt;
}

namespace {
[[noreturn]] void fail(const std::string& dev, const std::string& what) {
  throw std::runtime_error("napl.devices: device '" + dev + "': " + what);
}
// `ok` is written as a positive comparison so NaN fails it.
void need(bool ok, const std::string& dev, const char* what) {
  if (!ok) fail(dev, what);
}
Meta meta_of(const flatbuffers::String* name, const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* parts,
             std::set<std::string>& seen) {
  Meta m;
  m.name = name->str();
  if (m.name.empty() || m.name.find('/') != std::string::npos) fail(m.name, "name must be non-empty and contain no '/'");
  if (!seen.insert(m.name).second) fail(m.name, "duplicate device name");
  if (parts)
    for (const flatbuffers::String* p : *parts) m.parts.push_back(p->str());
  return m;
}
void node_ok(int node, const std::string& dev, int lo = 1) {
  if (node < lo || node > kMaxNodes) fail(dev, "node must be in [" + std::to_string(lo) + ", " + std::to_string(kMaxNodes) + "]");
}
MotorCfg motor_of(const fb::MotorParams* mp, const std::string& dev) {  // Esc and Servo
  const MotorCfg c{mp->kv(), mp->r20(), mp->i0(), mp->rotor_inertia(), mp->w_s(), mp->c_th(), mp->r_th(), mp->t_amb(),
                   mp->alpha()};
  need(c.kv > 0 && c.r20 > 0 && c.inertia > 0 && c.w_s > 0 && c.c_th > 0 && c.r_th > 0, dev,
       "motor kv, r20, rotor_inertia, w_s, c_th, r_th must be > 0");
  need(c.i0 >= 0 && std::isfinite(c.t_amb) && c.alpha >= 0, dev, "motor i0 and alpha must be >= 0, t_amb finite");
  return c;
}
// 3 values or none (= 0), finite.
void vec3_of(const flatbuffers::Vector<double>* v, double out[3], const std::string& dev, const char* what) {
  need(!v || v->size() == 0 || v->size() == 3, dev, what);
  for (int k = 0; k < 3; ++k) {
    out[k] = v && v->size() ? v->Get(static_cast<flatbuffers::uoffset_t>(k)) : 0.0;
    need(std::isfinite(out[k]), dev, what);
  }
}
InertialErrCfg inertial_of(const fb::InertialErr* e, const std::string& dev) {
  InertialErrCfg c{};
  vec3_of(e->bias(), c.bias, dev, "gyro / accel bias needs 3 finite values (or none)");
  vec3_of(e->scale(), c.scale, dev, "gyro / accel scale needs 3 finite values (or none)");
  c.noise_density = e->noise_density(); c.rw_density = e->rw_density();
  c.full_scale = e->full_scale(); c.bits = e->bits();
  need(c.noise_density >= 0 && std::isfinite(c.noise_density) && c.rw_density >= 0 && std::isfinite(c.rw_density), dev,
       "noise_density and rw_density must be finite and >= 0");
  need(c.full_scale >= 0 && std::isfinite(c.full_scale), dev, "full_scale must be >= 0 (0 = no saturation)");
  need(c.bits >= 0 && c.bits <= 32 && (c.bits == 0 || c.full_scale > 0), dev,
       "bits must be in [0, 32] (0 = no quantisation) and needs full_scale > 0");
  return c;
}
void sensor_ok(const std::string& dev, const std::string& site, double rate_hz) {
  need(!site.empty(), dev, "site must be non-empty");
  need(rate_hz > 0 && std::isfinite(rate_hz), dev, "rate_hz must be > 0");
}
void std_ok(const std::string& dev, double sd, const char* what) { need(sd >= 0 && std::isfinite(sd), dev, what); }
// Seed of device slot k: its own `seed` if given, else derived from the top-level seed and the slot index.
std::uint64_t seed_of(std::uint64_t own, std::uint64_t top, std::size_t slot) { return own ? own : top + slot; }
}  // namespace

Devices parse(const std::string& json) {
  flatbuffers::IDLOptions opts;
  opts.strict_json = true;  // quoted names; skip_unexpected_fields_in_json stays false => an unknown field is an error
  flatbuffers::Parser parser(opts);
  if (!parser.Deserialize(fb::DevicesBinarySchema::data(), fb::DevicesBinarySchema::size()))
    throw std::runtime_error("napl.devices: embedded schema failed to load");
  if (!parser.Parse(json.c_str())) throw std::runtime_error("napl.devices: " + parser.error_);
  const fb::Devices* root = flatbuffers::GetRoot<fb::Devices>(parser.builder_.GetBufferPointer());

  Devices dv;
  std::set<std::string> seen;
  const std::uint64_t top = root->seed();
  auto slot = [&](Devices::Slot::Type t, int i) { dv.slots.push_back({t, i}); };
  auto use_node = [&](int node) { if (node > dv.n_nodes) dv.n_nodes = node; };

  if (root->batteries())
    for (const fb::Battery* b : *root->batteries()) {
      BatteryCfg c{};
      c.meta = meta_of(b->name(), b->parts(), seen);
      const std::string& n = c.meta.name;
      c.node = b->node();
      node_ok(c.node, n);
      if (b->ocv_soc()) c.ocv_soc.assign(b->ocv_soc()->begin(), b->ocv_soc()->end());
      if (b->ocv_v()) c.ocv_v.assign(b->ocv_v()->begin(), b->ocv_v()->end());
      need(!c.ocv_soc.empty() && c.ocv_soc.size() == c.ocv_v.size(), n, "ocv_soc / ocv_v must be equal-length and non-empty");
      for (std::size_t k = 0; k < c.ocv_soc.size(); ++k) {
        need(c.ocv_soc[k] >= 0 && c.ocv_soc[k] <= 1 && std::isfinite(c.ocv_v[k]), n, "ocv_soc must lie in [0,1], ocv_v finite");
        need(k == 0 || c.ocv_soc[k] > c.ocv_soc[k - 1], n, "ocv_soc must be strictly increasing");
      }
      c.capacity_ah = b->capacity_ah(); c.r0 = b->r0(); c.r1 = b->r1(); c.c1 = b->c1(); c.soc0 = b->soc0();
      need(c.capacity_ah > 0, n, "capacity_ah must be > 0");
      need(c.r0 > 0, n, "r0 must be > 0");
      need(c.r1 >= 0 && (c.r1 == 0 || c.c1 > 0), n, "r1 must be >= 0 and c1 > 0 when r1 > 0");
      need(c.soc0 >= 0 && c.soc0 <= 1, n, "soc0 must lie in [0,1]");
      c.seed = b->seed();
      use_node(c.node);
      slot(Devices::Slot::battery, static_cast<int>(dv.battery.cfg.size()));
      dv.battery.cfg.push_back(std::move(c));
    }
  if (root->psus())
    for (const fb::Psu* p : *root->psus()) {
      PsuCfg c{};
      c.meta = meta_of(p->name(), p->parts(), seen);
      const std::string& n = c.meta.name;
      c.node = p->node();
      node_ok(c.node, n);
      c.voltage = p->voltage(); c.r_series = p->r_series(); c.i_limit = p->i_limit(); c.enabled0 = p->enabled();
      need(c.voltage > 0, n, "voltage must be > 0");
      need(c.r_series > 0, n, "r_series must be > 0");
      need(c.i_limit >= 0, n, "i_limit must be >= 0 (0 = none)");
      use_node(c.node);
      slot(Devices::Slot::psu, static_cast<int>(dv.psu.cfg.size()));
      dv.psu.cfg.push_back(std::move(c));
    }
  if (root->wires())
    for (const fb::Wire* w : *root->wires()) {
      WireCfg c{};
      c.meta = meta_of(w->name(), w->parts(), seen);
      const std::string& n = c.meta.name;
      c.a = w->a(); c.b = w->b();
      node_ok(c.a, n, 0);
      node_ok(c.b, n, 0);
      need(c.a != c.b, n, "a and b must differ");
      if (w->resistance() != 0) {
        need(w->resistance() > 0 && w->awg() == 0 && w->length_m() == 0, n, "give either resistance > 0, or awg + length_m, not both");
        c.r = w->resistance();
      } else {
        need(w->awg() >= 1 && w->awg() <= 60 && w->length_m() > 0 && w->conductors() >= 1, n,
             "need resistance > 0, or awg in [1,60] with length_m > 0 and conductors >= 1");
        // AWG: d = 0.127 mm * 92^((36 - AWG)/39); R = rho_Cu(20 degC) L / (pi d^2 / 4), x conductors for a round trip.
        const double d = 0.127e-3 * std::pow(92.0, (36.0 - w->awg()) / 39.0);
        c.r = 1.724e-8 * w->length_m() * w->conductors() / (kPi * d * d / 4.0);
      }
      use_node(c.a);
      use_node(c.b);
      slot(Devices::Slot::wire, static_cast<int>(dv.wire.cfg.size()));
      dv.wire.cfg.push_back(std::move(c));
    }
  if (root->loads())
    for (const fb::Load* l : *root->loads()) {
      LoadCfg c{};
      c.meta = meta_of(l->name(), l->parts(), seen);
      const std::string& n = c.meta.name;
      c.node = l->node(); c.g = l->g(); c.i0 = l->i0();
      node_ok(c.node, n);
      need(c.g >= 0 && std::isfinite(c.i0), n, "g must be >= 0 and i0 finite");
      use_node(c.node);
      slot(Devices::Slot::load, static_cast<int>(dv.load.cfg.size()));
      dv.load.cfg.push_back(std::move(c));
    }
  if (root->escs())
    for (const fb::Esc* e : *root->escs()) {
      EscCfg c{};
      c.meta = meta_of(e->name(), e->parts(), seen);
      const std::string& n = c.meta.name;
      c.node = e->node();
      node_ok(c.node, n);
      c.joint = e->joint()->str();
      need(!c.joint.empty(), n, "joint must be non-empty");
      c.motor = motor_of(e->motor(), n);
      c.complementary = e->complementary();
      c.i_limit = e->i_limit();
      need(c.i_limit >= 0, n, "i_limit must be >= 0 (0 = none)");
      c.seed = e->seed();
      use_node(c.node);
      slot(Devices::Slot::esc, static_cast<int>(dv.esc.cfg.size()));
      dv.esc.cfg.push_back(std::move(c));
    }
  if (root->rotors())
    for (const fb::Rotor* r : *root->rotors()) {
      RotorCfg c{};
      c.meta = meta_of(r->name(), r->parts(), seen);
      const std::string& n = c.meta.name;
      c.body = r->body()->str();
      c.joint = r->joint()->str();
      need(!c.body.empty() && !c.joint.empty(), n, "body and joint must be non-empty");
      c.diameter = r->diameter();
      need(c.diameter > 0 && std::isfinite(c.diameter), n, "diameter must be > 0");
      c.dir = r->dir();
      need(c.dir == 1 || c.dir == -1, n, "dir must be +1 or -1");
      if (r->j()) c.j.assign(r->j()->begin(), r->j()->end());
      if (r->ct()) c.ct.assign(r->ct()->begin(), r->ct()->end());
      if (r->cq()) c.cq.assign(r->cq()->begin(), r->cq()->end());
      need(!c.j.empty() && c.j.size() == c.ct.size() && c.j.size() == c.cq.size(), n,
           "j / ct / cq must be equal-length and non-empty");
      for (std::size_t k = 0; k < c.j.size(); ++k) {
        need(std::isfinite(c.j[k]) && std::isfinite(c.ct[k]) && std::isfinite(c.cq[k]), n, "j / ct / cq must be finite");
        need(k == 0 || c.j[k] > c.j[k - 1], n, "j must be strictly increasing");
      }
      c.k_h = r->k_h();
      need(c.k_h >= 0 && std::isfinite(c.k_h), n, "k_h must be >= 0");
      c.ground_effect = r->ground_effect();
      c.ray_groups = r->ray_groups();
      need(c.ray_groups < 64, n, "ray_groups is a 6-bit mask (geom groups 0..5)");
      slot(Devices::Slot::rotor, static_cast<int>(dv.rotor.cfg.size()));
      dv.rotor.cfg.push_back(std::move(c));
    }
  if (root->environments())
    for (const fb::Environment* e : *root->environments()) {
      EnvCfg c{};
      c.meta = meta_of(e->name(), e->parts(), seen);
      const std::string& n = c.meta.name;
      need(dv.env.cfg.empty(), n, "only one environment device is allowed");
      c.origin_alt = e->origin_alt();
      need(std::isfinite(c.origin_alt) && c.origin_alt > -1000 && c.origin_alt < 10000, n,
           "origin_alt must be in (-1000, 10000) m");
      need(!e->wind() || e->wind()->size() == 3, n, "wind needs 3 values (or none)");
      for (int k = 0; k < 3 && e->wind(); ++k) {
        c.wind0[k] = e->wind()->Get(static_cast<flatbuffers::uoffset_t>(k));
        need(std::isfinite(c.wind0[k]), n, "wind must be finite");
      }
      c.w20 = e->w20();
      need(c.w20 >= 0 && std::isfinite(c.w20), n, "w20 must be >= 0");
      c.seed = e->seed();
      if (e->ref_body()) c.ref_body = e->ref_body()->str();
      c.ref_height = e->ref_height();
      need(c.ref_height >= 0 && std::isfinite(c.ref_height), n, "ref_height must be >= 0");
      slot(Devices::Slot::env, static_cast<int>(dv.env.cfg.size()));
      dv.env.cfg.push_back(std::move(c));
    }
  if (root->body_drags())
    for (const fb::BodyDrag* b : *root->body_drags()) {
      DragCfg c{};
      c.meta = meta_of(b->name(), b->parts(), seen);
      const std::string& n = c.meta.name;
      c.body = b->body()->str();
      need(!c.body.empty(), n, "body must be non-empty");
      c.cd = b->cd();
      c.area = b->area();
      need(c.cd >= 0 && std::isfinite(c.cd) && c.area >= 0 && std::isfinite(c.area), n, "cd and area must be >= 0");
      slot(Devices::Slot::drag, static_cast<int>(dv.drag.cfg.size()));
      dv.drag.cfg.push_back(std::move(c));
    }
  if (root->servos())
    for (const fb::Servo* sv : *root->servos()) {
      ServoCfg c{};
      c.meta = meta_of(sv->name(), sv->parts(), seen);
      const std::string& n = c.meta.name;
      c.node = sv->node();
      node_ok(c.node, n);
      c.joint = sv->joint()->str();
      need(!c.joint.empty(), n, "joint must be non-empty");
      c.motor = motor_of(sv->motor(), n);
      c.n = sv->gear_ratio(); c.eta = sv->efficiency(); c.cpr = sv->encoder_cpr(); c.rate_hz = sv->firmware_hz();
      need(c.n > 0 && std::isfinite(c.n), n, "gear_ratio must be > 0");
      need(c.eta > 0 && c.eta <= 1, n, "efficiency must be in (0, 1]");
      need(c.cpr > 0, n, "encoder_cpr must be > 0");
      need(c.rate_hz > 0 && std::isfinite(c.rate_hz), n, "firmware_hz must be > 0");
      c.p = sv->p(); c.i = sv->i(); c.d = sv->d(); c.pid_scale = sv->pid_scale(); c.deadband = sv->deadband();
      need(c.p >= 0 && c.i >= 0 && c.d >= 0 && std::isfinite(c.p + c.i + c.d), n, "p, i, d must be finite and >= 0");
      need(c.pid_scale > 0 && std::isfinite(c.pid_scale), n, "pid_scale must be > 0");
      need(c.deadband >= 0 && std::isfinite(c.deadband), n, "deadband must be >= 0 (counts)");
      c.max_torque = sv->max_torque();
      need(c.max_torque > 0 && c.max_torque <= 1000, n, "max_torque must be in (0, 1000]");
      c.i_limit = sv->i_limit();
      need(c.i_limit >= 0 && std::isfinite(c.i_limit), n, "i_limit must be >= 0 (0 = none)");
      if (sv->backlash_joint()) c.backlash_joint = sv->backlash_joint()->str();
      use_node(c.node);
      slot(Devices::Slot::servo, static_cast<int>(dv.servo.cfg.size()));
      dv.servo.cfg.push_back(std::move(c));
    }
  if (root->imus())
    for (const fb::Imu* s : *root->imus()) {
      ImuCfg c{};
      c.meta = meta_of(s->name(), s->parts(), seen);
      const std::string& n = c.meta.name;
      c.site = s->site()->str(); c.rate_hz = s->rate_hz(); c.seed = s->seed();
      sensor_ok(n, c.site, c.rate_hz);
      c.gyro = inertial_of(s->gyro(), n);
      c.accel = inertial_of(s->accel(), n);
      slot(Devices::Slot::imu, static_cast<int>(dv.imu.cfg.size()));
      dv.imu.cfg.push_back(std::move(c));
    }
  if (root->baros())
    for (const fb::Baro* s : *root->baros()) {
      BaroCfg c{};
      c.meta = meta_of(s->name(), s->parts(), seen);
      const std::string& n = c.meta.name;
      c.site = s->site()->str(); c.rate_hz = s->rate_hz(); c.seed = s->seed();
      sensor_ok(n, c.site, c.rate_hz);
      c.bias = s->bias(); c.noise_std = s->noise_std(); c.resolution = s->resolution();
      need(std::isfinite(c.bias), n, "bias must be finite");
      std_ok(n, c.noise_std, "noise_std must be finite and >= 0");
      std_ok(n, c.resolution, "resolution must be finite and >= 0 (0 = none)");
      slot(Devices::Slot::baro, static_cast<int>(dv.baro.cfg.size()));
      dv.baro.cfg.push_back(std::move(c));
    }
  if (root->mags())
    for (const fb::Mag* s : *root->mags()) {
      MagCfg c{};
      c.meta = meta_of(s->name(), s->parts(), seen);
      const std::string& n = c.meta.name;
      c.site = s->site()->str(); c.rate_hz = s->rate_hz(); c.seed = s->seed();
      sensor_ok(n, c.site, c.rate_hz);
      need(s->earth_field() && s->earth_field()->size() == 3, n, "earth_field needs 3 values (world frame, T)");
      vec3_of(s->earth_field(), c.field, n, "earth_field must be finite");
      vec3_of(s->bias(), c.bias, n, "bias needs 3 finite values (or none)");
      c.noise_std = s->noise_std();
      std_ok(n, c.noise_std, "noise_std must be finite and >= 0");
      slot(Devices::Slot::mag, static_cast<int>(dv.mag.cfg.size()));
      dv.mag.cfg.push_back(std::move(c));
    }
  if (root->gnss_receivers())
    for (const fb::Gnss* s : *root->gnss_receivers()) {
      GnssCfg c{};
      c.meta = meta_of(s->name(), s->parts(), seen);
      const std::string& n = c.meta.name;
      c.site = s->site()->str(); c.rate_hz = s->rate_hz(); c.seed = s->seed();
      sensor_ok(n, c.site, c.rate_hz);
      c.lat0 = s->lat0(); c.lon0 = s->lon0(); c.alt0 = s->alt0();
      need(std::abs(c.lat0) <= 89.0 && std::abs(c.lon0) <= 180.0 && std::isfinite(c.alt0), n,
           "lat0 must be in [-89, 89] deg, lon0 in [-180, 180] deg, alt0 finite");
      c.horiz_std = s->horiz_std(); c.vert_std = s->vert_std(); c.vel_std = s->vel_std(); c.latency_s = s->latency_s();
      std_ok(n, c.horiz_std, "horiz_std must be finite and >= 0");
      std_ok(n, c.vert_std, "vert_std must be finite and >= 0");
      std_ok(n, c.vel_std, "vel_std must be finite and >= 0");
      std_ok(n, c.latency_s, "latency_s must be finite and >= 0");
      const double delay = c.latency_s * c.rate_hz;
      need(delay < kGnssRing - 0.5, n, "latency_s * rate_hz must round to at most 63 samples (the ring capacity)");
      c.delay = static_cast<int>(std::lround(delay));
      slot(Devices::Slot::gnss, static_cast<int>(dv.gnss.cfg.size()));
      dv.gnss.cfg.push_back(std::move(c));
    }
  if (dv.slots.empty()) throw std::runtime_error("napl.devices: no devices declared");

  // Initial state. RNG stream k uses the device's flat slot index (battery, psu, wire, load, esc order).
  std::size_t k = 0;
  for (const BatteryCfg& c : dv.battery.cfg) {
    BatteryState s{};
    s.soc = c.soc0;
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.battery.st.push_back(s);
  }
  for (const PsuCfg& c : dv.psu.cfg) dv.psu.st.push_back(PsuState{c.enabled0 ? 1.0 : 0.0, 0, 0, 0}), ++k;
  k += dv.wire.cfg.size() + dv.load.cfg.size();
  dv.wire.st.assign(dv.wire.cfg.size(), WireState{});
  dv.load.st.assign(dv.load.cfg.size(), LoadState{});
  for (const EscCfg& c : dv.esc.cfg) {
    EscState s{};
    s.temp = c.motor.t_amb;
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.esc.st.push_back(s);
  }
  dv.rotor.st.assign(dv.rotor.cfg.size(), RotorState{0, 0, 0, 0, 1, 0});
  k += dv.rotor.cfg.size();
  for (const EnvCfg& c : dv.env.cfg) {
    EnvState s{};
    std::copy(c.wind0, c.wind0 + 3, s.wind);
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.env.st.push_back(s);
    dv.env.st.back().density = rho_at(dv, 0.0);  // valid before the first tick
  }
  dv.drag.st.assign(dv.drag.cfg.size(), DragState{});
  k += dv.drag.cfg.size();
  for (const ServoCfg& c : dv.servo.cfg) {  // no RNG stream: servos draw no noise
    ServoState s{};
    s.temp = c.motor.t_amb;
    dv.servo.st.push_back(s);
  }
  k += dv.servo.cfg.size();
  for (const ImuCfg& c : dv.imu.cfg) {
    ImuState s{};
    std::copy(c.gyro.bias, c.gyro.bias + 3, s.gyro_bias);
    std::copy(c.accel.bias, c.accel.bias + 3, s.accel_bias);
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.imu.st.push_back(s);
  }
  for (const BaroCfg& c : dv.baro.cfg) {
    BaroState s{};
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.baro.st.push_back(s);
  }
  for (const MagCfg& c : dv.mag.cfg) {
    MagState s{};
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.mag.st.push_back(s);
  }
  for (const GnssCfg& c : dv.gnss.cfg) {
    GnssState s{};
    s.lat = c.lat0; s.lon = c.lon0; s.alt = c.alt0;  // before the first sample (and the first `delay` ones): the origin
    s.rng.seed(seed_of(c.seed, top, k++));
    dv.gnss.st.push_back(s);
  }
  return dv;
}

void bind(mjModel* m, Devices& dv) {
  if (m->opt.integrator == mjINT_RK4)
    throw std::runtime_error("napl.devices: devices step with mj_step1/mj_step2, which RK4 does not support; use Euler, "
                             "implicit or implicitfast");
  std::set<int> used;
  auto hinge = [&](const std::string& dev, const std::string& joint) {
    const int j = mj_name2id(m, mjOBJ_JOINT, joint.c_str());
    if (j < 0) fail(dev, "joint '" + joint + "' not found in the model");
    if (m->jnt_type[j] != mjJNT_HINGE) fail(dev, "joint '" + joint + "' is not a hinge");
    return j;
  };
  for (EscCfg& c : dv.esc.cfg) {
    c.jnt = hinge(c.meta.name, c.joint);
    if (!used.insert(c.jnt).second) fail(c.meta.name, "joint '" + c.joint + "' is already driven by another device");
    c.dof = m->jnt_dofadr[c.jnt];
  }
  // Period in ticks of a firmware / sensor rate: 1 / (rate dt) must be an integer >= 1.
  auto period_of = [&](const std::string& dev, const char* key, double rate_hz, const char* what) {
    const double ticks = 1.0 / (rate_hz * m->opt.timestep);
    const int period = ticks < 2e9 ? static_cast<int>(std::lround(ticks)) : 0;  // 0: out of range, fails below
    if (period < 1 || std::abs(ticks - period) > 1e-9 * ticks)
      fail(dev, std::string(key) + " " + std::to_string(rate_hz) + " gives " + std::to_string(ticks) + " ticks per " +
                    what + " at the model timestep; need an integer >= 1");
    return period;
  };
  for (std::size_t i = 0; i < dv.servo.cfg.size(); ++i) {
    ServoCfg& c = dv.servo.cfg[i];
    const std::string& n = c.meta.name;
    c.jnt = hinge(n, c.joint);
    if (!used.insert(c.jnt).second) fail(n, "joint '" + c.joint + "' is already driven by another device");
    c.dof = m->jnt_dofadr[c.jnt];
    c.qadr = m->jnt_qposadr[c.jnt];
    c.period = period_of(n, "firmware_hz", c.rate_hz, "firmware step");
    ServoState& s = dv.servo.st[i];
    s.backlash = 0;
    if (!c.backlash_joint.empty()) {  // single source of truth: the MJCF range of the (undriven) backlash hinge
      const int bj = hinge(n, c.backlash_joint);
      if (bj == c.jnt) fail(n, "backlash_joint must differ from joint");
      const double* r = m->jnt_range + 2 * bj;
      if (!m->jnt_limited[bj] || !(r[1] > 0) || r[0] != -r[1])
        fail(n, "backlash_joint '" + c.backlash_joint + "' must be limited with a symmetric range -b..+b, b > 0");
      s.backlash = r[1];
    }
  }
  auto body_id = [&](const std::string& dev, const std::string& body) {
    const int b = mj_name2id(m, mjOBJ_BODY, body.c_str());
    if (b <= 0) fail(dev, "body '" + body + "' not found in the model (or is the world)");
    return b;
  };
  for (RotorCfg& c : dv.rotor.cfg) {
    c.bid = body_id(c.meta.name, c.body);
    c.jnt = mj_name2id(m, mjOBJ_JOINT, c.joint.c_str());
    if (c.jnt < 0) fail(c.meta.name, "joint '" + c.joint + "' not found in the model");
    if (m->jnt_type[c.jnt] != mjJNT_HINGE) fail(c.meta.name, "joint '" + c.joint + "' is not a hinge");
    if (m->jnt_bodyid[c.jnt] != c.bid) fail(c.meta.name, "joint '" + c.joint + "' is not on body '" + c.body + "'");
    c.dof = m->jnt_dofadr[c.jnt];
  }
  for (EnvCfg& c : dv.env.cfg) c.bid = c.ref_body.empty() ? -1 : body_id(c.meta.name, c.ref_body);
  for (DragCfg& c : dv.drag.cfg) c.bid = body_id(c.meta.name, c.body);
  auto site_id = [&](const std::string& dev, const std::string& site) {
    const int s = mj_name2id(m, mjOBJ_SITE, site.c_str());
    if (s < 0) fail(dev, "site '" + site + "' not found in the model");
    return s;
  };
  for (ImuCfg& c : dv.imu.cfg) {
    c.sid = site_id(c.meta.name, c.site);
    c.period = period_of(c.meta.name, "rate_hz", c.rate_hz, "sample");
  }
  for (BaroCfg& c : dv.baro.cfg) {
    c.sid = site_id(c.meta.name, c.site);
    c.period = period_of(c.meta.name, "rate_hz", c.rate_hz, "sample");
  }
  for (MagCfg& c : dv.mag.cfg) {
    c.sid = site_id(c.meta.name, c.site);
    c.period = period_of(c.meta.name, "rate_hz", c.rate_hz, "sample");
  }
  for (GnssCfg& c : dv.gnss.cfg) {
    c.sid = site_id(c.meta.name, c.site);
    c.period = period_of(c.meta.name, "rate_hz", c.rate_hz, "sample");
  }
  // Only after every check passed: model edits. SET (not added), so rebinding a reloaded / recompiled model is
  // idempotent. The rotor inertia of an ESC motor is the dof armature (N = 1); a servo reflects it to the gear output:
  // J_rotor N^2, large next to the load.
  for (const EscCfg& c : dv.esc.cfg) {
    m->dof_armature[c.dof] = c.motor.inertia;
    m->dof_damping[c.dof] = 0;  // owned by the device from here on (MJCF damping on this joint is ignored)
  }
  for (const ServoCfg& c : dv.servo.cfg) {
    m->dof_armature[c.dof] = c.motor.inertia * c.n * c.n;
    m->dof_damping[c.dof] = 0;
  }
  // Precomputed constants hold the MJCF armature: a "simple" dof (one hinge on a body with no other dofs) reads its
  // inertia from dof_M0, and dof_invweight0 is MuJoCo's reference weight of limit / friction constraints (with the
  // stale value a servo's joint limit or backlash stop came out ~60x too soft). Refresh every qpos0-derived constant
  // with mj_setConst on a scratch mjData: a pure function of the model, so idempotent, exactly what compile does.
  if (!dv.esc.cfg.empty() || !dv.servo.cfg.empty()) {
    mjData* scratch = mj_makeData(m);
    if (!scratch) throw std::runtime_error("napl.devices: mj_makeData failed while refreshing model constants");
    mj_setConst(m, scratch);
    mj_deleteData(scratch);
  }
}

// ---- tick ----
namespace {
// Motor stage of one drive on `dof`, shared by the ESC (n = 1, eta = 1) and the servo (n = gear ratio, eta = efficiency).
// Reads s.{omega (motor speed), temp, duty, v, i_m} of this tick's solve, writes s.{b, drive, torque, temp}. Damping split
// (stability): the back-EMF damping goes into dof_damping (integrated implicitly by mj_step2's Euler), only the driving term
// is an explicit applied torque. With tau_m = Kt I_m - tau_f, tau_f = Kt I0 tanh(w_m / w_s), and the gear loss factor k
// (below), decided from the state at tick start:
//   b = k Kt Ke n^2 / R,   applied = k n (Kt d V / R) - k n tau_f,   so  -b w_out + applied = k n tau_m = s.torque  exactly.
// k = eta while the motor delivers power (tau_m w_out >= 0; w = 0 counts as driving), 1 when back-driven. Strictly that
// switch is discontinuous at w = 0, and against a stop (stall, w ~ 0) it chatters: k flips every tick with the sign of the
// noise velocity (measured: +-0.04 rad/s, torque dithering between N eta tau_m and N tau_m). So the back-driven side is
// ramped in over the friction smoothing speed: g = tanh(w_m / w_s) sign(tau_m), k = eta for g >= 0 else
// eta + (1 - eta)(-g). Equal to the sharp rule for |w_m| >> w_s and to the driving value at w = 0. Fidelity: a modelling
// regularisation, not a measured gearbox efficiency curve. The ESC passes eta = 1: k = 1.
// The torque acts on the joint, so its reaction lands on the parent body (stator / housing). Open phase: b = 0, no drive;
// friction (bearings + iron loss) stays. Thermal: explicit Euler, C_th dT/dt = I_m^2 R(T) - (T - T_amb)/R_th, R(T) of the
// tick start.
template <class S>
void drive_motor(const MotorCfg& mc, mjModel* m, mjData* d, int dof, double n, double eta, bool open, S& s, double dt) {
  const double kt = mc.ke(), r = mc.r_at(s.temp);
  const double fric = kt * mc.i0 * std::tanh(s.omega / mc.w_s);
  const double dq = d->qvel[dof];  // dof_damping was 0 in step1: add the whole -b*qvel to the passive force arrays
  const double tau_m = kt * s.i_m - fric;
  const double g = std::tanh(s.omega / mc.w_s) * (tau_m > 0 ? 1.0 : tau_m < 0 ? -1.0 : 0.0);  // > 0: motor delivers power
  const double k = g >= 0 ? eta : eta + (1.0 - eta) * -g;
  s.b = open ? 0.0 : k * kt * mc.ke() * n * n / r;
  s.drive = open ? 0.0 : k * n * kt * s.duty * s.v / r;
  s.torque = k * n * (kt * s.i_m - fric);
  d->qfrc_applied[dof] += s.drive - k * n * fric;
  d->qfrc_passive[dof] -= s.b * dq;
  d->qfrc_damper[dof] -= s.b * dq;
  m->dof_damping[dof] = s.b;  // same b for mj_step2's implicit damping
  s.temp += dt / mc.c_th * (s.i_m * s.i_m * r - (s.temp - mc.t_amb) / mc.r_th);
}
}  // namespace

// Order of device phases (fixed): sensors-read -> controllers -> electrical solve -> motors -> aero/forces -> mj_step2
// -> post-step hook (sensors sample). Sensors-read: the held samples already sit in the state; nothing consumes them yet.
void step(mjModel* m, mjData* d, Devices& dv) {
  // dof_damping of a driven dof is overwritten every tick: zero it BEFORE step1 so qfrc_passive is computed with a canonical
  // b_old = 0 whatever a restore / clone left in the (unsaved) model array. Restored runs are then bitwise identical.
  for (const EscCfg& c : dv.esc.cfg) m->dof_damping[c.dof] = 0;
  for (const ServoCfg& c : dv.servo.cfg) m->dof_damping[c.dof] = 0;
  mj_step1(m, d);
  mju_zero(d->qfrc_applied, m->nv);  // devices own the applied forces entirely
  mju_zero(d->xfrc_applied, 6 * m->nbody);
  const double dt = m->opt.timestep;

  // 1. sensors-read (held samples: nothing to do)   2. controllers: servo encoder + firmware.
  step_firmware(m, d, dv);

  // 3. electrical solve, with the shaft speeds of the current state (servo: motor side, N x output).
  for (std::size_t e = 0; e < dv.esc.cfg.size(); ++e) dv.esc.st[e].omega = d->qvel[dv.esc.cfg[e].dof];
  for (std::size_t i = 0; i < dv.servo.cfg.size(); ++i) dv.servo.st[i].omega = dv.servo.cfg[i].n * d->qvel[dv.servo.cfg[i].dof];
  solve_electrical(dv, dt);

  // 4. motors (drive_motor above).
  for (std::size_t e = 0; e < dv.esc.cfg.size(); ++e)
    drive_motor(dv.esc.cfg[e].motor, m, d, dv.esc.cfg[e].dof, 1.0, 1.0, dv.esc.st[e].mode == 1, dv.esc.st[e], dt);
  for (std::size_t i = 0; i < dv.servo.cfg.size(); ++i) {
    const ServoCfg& c = dv.servo.cfg[i];
    drive_motor(c.motor, m, d, c.dof, c.n, c.eta, !(dv.servo.st[i].torque_enable > 0.5), dv.servo.st[i], dt);
  }

  // 5. aero / forces: environment (turbulence), rotors, body drag.
  step_forces(m, d, dv);
  if (!dv.imu.cfg.empty()) dv.qvel_k.assign(d->qvel, d->qvel + m->nv);  // the IMU acceleration needs s_k's qvel
  mj_step2(m, d);

  // 6. post-step hook: sensors sample the derived data of s_k (sensors.cpp).
  sample_sensors(m, d, dv);
}

// ---- state block ----
namespace {
template <class D, class F> void for_groups(D& dv, F&& f) {
  f(dv.battery);
  f(dv.psu);
  f(dv.wire);
  f(dv.load);
  f(dv.esc);
  f(dv.rotor);
  f(dv.env);
  f(dv.drag);
  f(dv.servo);
  f(dv.imu);
  f(dv.baro);
  f(dv.mag);
  f(dv.gnss);
}
template <class G> constexpr bool pod_ok = std::is_trivially_copyable_v<typename G::State> && sizeof(typename G::State) % 8 == 0;
}  // namespace

std::size_t state_bytes(const Devices& dv) {
  std::size_t n = 0;
  for_groups(dv, [&](const auto& g) {
    static_assert(pod_ok<std::decay_t<decltype(g)>>);
    n += g.st.size() * sizeof(typename std::decay_t<decltype(g)>::State);
  });
  return n;
}
void save_state(const Devices& dv, std::byte* out) {
  for_groups(dv, [&](const auto& g) {
    const std::size_t n = g.st.size() * sizeof(typename std::decay_t<decltype(g)>::State);
    if (n) std::memcpy(out, g.st.data(), n);
    out += n;
  });
}
void load_state(Devices& dv, const std::byte* in) {
  for_groups(dv, [&](auto& g) {
    const std::size_t n = g.st.size() * sizeof(typename std::decay_t<decltype(g)>::State);
    if (n) std::memcpy(g.st.data(), in, n);
    in += n;
  });
}

// ---- prim attributes ----
namespace {
// rw: settable input, valid in [lo, hi] (bin: exactly 0 or 1). SI units; temperature is in degC.
template <class S> struct Attr {
  const char* name; const char* unit; double S::* f;
  bool rw = false; bool bin = false; double lo = 0, hi = 0;
};
constexpr Attr<BatteryState> kBattery[] = {{"voltage", "V", &BatteryState::v}, {"current", "A", &BatteryState::i},
                                           {"soc", "1", &BatteryState::soc}, {"ocv", "V", &BatteryState::ocv},
                                           {"v_rc", "V", &BatteryState::v_rc}};
constexpr Attr<PsuState> kPsu[] = {{"voltage", "V", &PsuState::v}, {"current", "A", &PsuState::i},
                                   {"limited", "1", &PsuState::limited}, {"enable", "1", &PsuState::enable, true, true, 0, 1}};
constexpr Attr<WireState> kWire[] = {{"current", "A", &WireState::i}, {"voltage", "V", &WireState::v}};
constexpr Attr<LoadState> kLoad[] = {{"voltage", "V", &LoadState::v}, {"current", "A", &LoadState::i}};
constexpr Attr<EscState> kEsc[] = {
    {"throttle", "1", &EscState::throttle, true, false, 0, 1}, {"duty", "1", &EscState::duty},
    {"current", "A", &EscState::i_m}, {"bus_current", "A", &EscState::i_bus}, {"voltage", "V", &EscState::v},
    {"omega", "rad/s", &EscState::omega}, {"torque", "N*m", &EscState::torque},
    {"temperature", "degC", &EscState::temp}};

constexpr Attr<RotorState> kRotor[] = {
    {"omega", "rad/s", &RotorState::omega}, {"thrust", "N", &RotorState::thrust}, {"torque", "N*m", &RotorState::torque},
    {"advance_ratio", "1", &RotorState::advance_ratio}, {"ge_ratio", "1", &RotorState::ge_ratio},
    {"rho", "kg/m^3", &RotorState::rho}};
constexpr Attr<ServoState> kServo[] = {
    {"goal_position", "rad", &ServoState::goal_position, true, false, -1e6, 1e6},
    {"goal_speed", "rad/s", &ServoState::goal_speed, true, false, 0, 1e4},
    {"torque_enable", "1", &ServoState::torque_enable, true, true, 0, 1},
    {"position", "rad", &ServoState::position}, {"counts", "1", &ServoState::counts},
    {"setpoint", "rad", &ServoState::setpoint}, {"duty", "1", &ServoState::duty},
    {"current", "A", &ServoState::i_m}, {"bus_current", "A", &ServoState::i_bus}, {"voltage", "V", &ServoState::v},
    {"torque", "N*m", &ServoState::torque}, {"temperature", "degC", &ServoState::temp},
    {"backlash", "rad", &ServoState::backlash}};
constexpr Attr<EnvState> kEnv[] = {{"density", "kg/m^3", &EnvState::density}};
// 3-vector attrs (same fields as Attr, member is a double[3]).
template <class S> struct Attr3 {
  const char* name; const char* unit; double (S::*f)[3];
  bool rw = false; double lo = 0, hi = 0;
};
constexpr Attr3<EnvState> kEnv3[] = {{"wind", "m/s", &EnvState::wind, true, -200, 200},
                                     {"turbulence", "m/s", &EnvState::turb}};
constexpr Attr3<DragState> kDrag3[] = {{"force", "N", &DragState::force}};
// Sensors: the held sample (all read-only).
constexpr Attr3<ImuState> kImu3[] = {{"gyro", "rad/s", &ImuState::gyro}, {"accel", "m/s^2", &ImuState::accel}};
constexpr Attr<BaroState> kBaro[] = {{"pressure", "Pa", &BaroState::pressure}};
constexpr Attr3<MagState> kMag3[] = {{"field", "T", &MagState::field}};
constexpr Attr<GnssState> kGnss[] = {{"lat", "deg", &GnssState::lat}, {"lon", "deg", &GnssState::lon},
                                     {"alt", "m", &GnssState::alt}};
constexpr Attr3<GnssState> kGnss3[] = {{"vel_ned", "m/s", &GnssState::vel_ned}};

template <class P> struct Found { P p; const char* unit; bool rw, bin; double lo, hi; std::size_t n = 1; };
template <class D> auto locate(D& dv, int slot, const std::string& attr) {
  using P = std::conditional_t<std::is_const_v<D>, const double*, double*>;
  Found<P> f{nullptr, "", false, false, 0, 0};
  const Devices::Slot s = dv.slots[static_cast<std::size_t>(slot)];
  auto pick = [&](const auto& tab, auto& states) {
    for (const auto& a : tab)
      if (attr == a.name) f = {&(states[static_cast<std::size_t>(s.i)].*(a.f)), a.unit, a.rw, a.bin, a.lo, a.hi};
  };
  auto pick3 = [&](const auto& tab, auto& states) {
    for (const auto& a : tab)
      if (attr == a.name) f = {(states[static_cast<std::size_t>(s.i)].*(a.f)), a.unit, a.rw, false, a.lo, a.hi, 3};
  };
  switch (s.type) {
    case Devices::Slot::battery: pick(kBattery, dv.battery.st); break;
    case Devices::Slot::psu: pick(kPsu, dv.psu.st); break;
    case Devices::Slot::wire: pick(kWire, dv.wire.st); break;
    case Devices::Slot::load: pick(kLoad, dv.load.st); break;
    case Devices::Slot::esc: pick(kEsc, dv.esc.st); break;
    case Devices::Slot::rotor: pick(kRotor, dv.rotor.st); break;
    case Devices::Slot::env: pick(kEnv, dv.env.st); pick3(kEnv3, dv.env.st); break;
    case Devices::Slot::drag: pick3(kDrag3, dv.drag.st); break;
    case Devices::Slot::servo: pick(kServo, dv.servo.st); break;
    case Devices::Slot::imu: pick3(kImu3, dv.imu.st); break;
    case Devices::Slot::baro: pick(kBaro, dv.baro.st); break;
    case Devices::Slot::mag: pick3(kMag3, dv.mag.st); break;
    case Devices::Slot::gnss: pick(kGnss, dv.gnss.st); pick3(kGnss3, dv.gnss.st); break;
  }
  return f;
}
std::string path_of(const Devices& dv, int slot) { return "/devices/" + dv.name(slot); }
}  // namespace

Result get(const Devices& dv, int slot, const std::string& attr, std::vector<double>& out) {
  const auto f = locate(dv, slot, attr);
  if (!f.p) return {Result::not_found, "unknown attr '" + attr + "' on " + path_of(dv, slot)};
  out.assign(f.p, f.p + f.n);
  return {};
}

Result set(Devices& dv, int slot, const std::string& attr, std::span<const double> values) {
  const auto f = locate(dv, slot, attr);
  if (!f.p) return {Result::not_found, "unknown attr '" + attr + "' on " + path_of(dv, slot)};
  if (!f.rw) return {Result::invalid_argument, "attr '" + attr + "' is read-only"};
  if (values.size() != f.n)
    return {Result::invalid_argument, "attr '" + attr + "' takes " + std::to_string(f.n) + " value(s), got " + std::to_string(values.size())};
  for (const double v : values) {  // validate everything before writing anything
    if (!std::isfinite(v)) return {Result::invalid_argument, "values must be finite"};
    if (f.bin ? (v != 0 && v != 1) : (v < f.lo || v > f.hi))
      return {Result::invalid_argument, "attr '" + attr + "' must be " + (f.bin ? "0 or 1" : "in [" + std::to_string(f.lo) + ", " + std::to_string(f.hi) + "]")};
  }
  std::copy(values.begin(), values.end(), f.p);
  return {};
}

std::string unit(const Devices& dv, int slot, const std::string& attr) { return locate(dv, slot, attr).unit; }

}  // namespace devs
