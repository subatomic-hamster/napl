#pragma once
// Devices (milestone N2): deterministic C++ physics models stepped inside MujocoSolver::step, exposed as prims
// /devices/<name>. Data-oriented: per device type one vector of plain config structs and one of POD state structs, free
// functions over them, no class hierarchy. Config: schema/devices.fbs, carried in the MJCF as <custom><text
// name="napl.devices" data='{json}'/>. A new device type is a Group plus one line in each of: for_groups(), the slot
// table, parse(), bind(), step()'s phase list, the attribute tables in devices.cpp.
#include <mujoco/mujoco.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "core/solver.h"

namespace devs {

constexpr int kMaxNodes = 32;  // non-ground electrical nodes (ground = node 0, implicit)
constexpr double kPi = 3.14159265358979323846;

// xoshiro256** seeded through splitmix64; one stream per device that needs noise. gauss() is our own Box-Muller:
// std::normal_distribution is implementation-defined, which would break bitwise replay across toolchains.
struct Rng {
  std::uint64_t s[4];
  void seed(std::uint64_t x);
  std::uint64_t next();
  double uniform();  // [0, 1), 53 bits
  double gauss();    // N(0,1); two uniforms per sample, no cached spare (so the state is exactly `s`)
};

// ---- config (immutable after parse; resolved MuJoCo ids are filled by bind) ----
struct Meta {
  std::string name;
  std::vector<std::string> parts;  // part ids: provenance for the fidelity report, never used by the physics
};
struct MotorCfg {  // see schema; shared by Esc and Servo
  double kv, r20, i0, inertia, w_s, c_th, r_th, t_amb, alpha;
  double ke() const { return 60.0 / (2.0 * kPi * kv); }  // = Kt in SI
  double r_at(double temp_c) const { return r20 * (1.0 + alpha * (temp_c - 20.0)); }
};
struct BatteryCfg {
  Meta meta; int node; std::vector<double> ocv_soc, ocv_v; double capacity_ah, r0, r1, c1, soc0; std::uint64_t seed;
};
struct PsuCfg { Meta meta; int node; double voltage, r_series, i_limit; bool enabled0; };
struct WireCfg { Meta meta; int a, b; double r; };
struct LoadCfg { Meta meta; int node; double g, i0; };
struct EscCfg {
  Meta meta; int node; std::string joint; MotorCfg motor; bool complementary; double i_limit; std::uint64_t seed;
  int jnt = -1, dof = -1;  // resolved by bind()
};
struct ServoCfg {
  Meta meta; int node; std::string joint; MotorCfg motor; double n, eta, rate_hz, p, i, d, pid_scale, deadband, max_torque;
  double i_limit; int cpr; std::string backlash_joint;
  int jnt = -1, dof = -1, qadr = -1, period = 0;  // resolved by bind(): firmware period in ticks
};
struct RotorCfg {
  Meta meta; std::string body, joint; double diameter; int dir; std::vector<double> j, ct, cq; double k_h;
  bool ground_effect; unsigned ray_groups;
  int bid = -1, jnt = -1, dof = -1;  // resolved by bind()
};
struct EnvCfg {
  Meta meta; double origin_alt, wind0[3], w20; std::uint64_t seed; std::string ref_body; double ref_height;
  int bid = -1;  // ref_body, resolved by bind() (-1: fixed ref_height)
};
struct DragCfg { Meta meta; std::string body; double cd, area; int bid = -1; };
// Sensors (S4): each on a site; sid / period (ticks per sample) are resolved by bind().
struct InertialErrCfg { double bias[3], scale[3], noise_density, rw_density, full_scale; int bits; };
struct ImuCfg {
  Meta meta; std::string site; double rate_hz; InertialErrCfg gyro, accel; std::uint64_t seed;
  int sid = -1, period = 0;
};
struct BaroCfg {
  Meta meta; std::string site; double rate_hz, bias, noise_std, resolution; std::uint64_t seed;
  int sid = -1, period = 0;
};
struct MagCfg {
  Meta meta; std::string site; double rate_hz, field[3], bias[3], noise_std; std::uint64_t seed;
  int sid = -1, period = 0;
};
constexpr int kGnssRing = 64;  // samples held for the latency: delay <= kGnssRing - 1
struct GnssCfg {
  Meta meta; std::string site; double rate_hz, lat0, lon0, alt0, horiz_std, vert_std, vel_std, latency_s;
  std::uint64_t seed; int sid = -1, period = 0, delay = 0;  // delay (samples) = round(latency_s rate_hz), set by parse
};

// ---- state: every mutable quantity, doubles / uint64 only (no padding), saved and hashed byte-for-byte ----
// "Outputs" (last solved values) live here too: they are cheap, deterministic, and make get() a plain field read.
struct BatteryState { double soc, v_rc, v, i, ocv; Rng rng; };  // v, i, ocv: of the last solve (i > 0 = discharging)
struct PsuState { double enable, v, i, limited; };              // enable: input (0/1); limited: 1 while current-limited
struct WireState { double i, v; };                              // a -> b current, voltage drop a - b
struct LoadState { double v, i; };
struct EscState {
  double throttle;  // input [0,1]
  double temp;      // winding temperature, degC
  double omega;     // shaft speed used by the last solve, rad/s
  double v, duty, i_m, i_bus, torque, b, drive, mode;  // mode: 0 normal, 1 phase open, 2 current-limited
  Rng rng;
};

struct ServoState {
  double goal_position, goal_speed, torque_enable;  // inputs (rad, rad/s, 0/1)
  double setpoint, integ, e_prev, cmd;               // firmware: ramped goal (rad), sum(e), last e (counts), held duty
  double counts, position;                           // encoder reading: counts, quantised angle (rad); every tick
  double temp, omega;                                // winding degC; motor shaft speed (N x output) of the last solve
  double v, duty, i_m, i_bus, torque, b, drive;      // as EscState; duty: effective (after current limit)
  double backlash;                                   // half-range of the backlash joint, rad (0: none); set by bind()
  std::uint64_t ticks;                               // ticks stepped; firmware runs when ticks % period == 0
};
struct RotorState { double omega, thrust, torque, advance_ratio, ge_ratio, rho; };  // outputs of the last tick
struct EnvState {
  double wind[3];  // steady wind, world (input)
  double turb[3];  // Dryden turbulence, world, of the last tick
  double density;  // ISA density at world z = 0
  double xa[3], xb[3];  // Dryden filter states (unit-variance-scaled): u: xa[0]; v, w: first / second stage in [1], [2]
  Rng rng;
};
struct DragState { double force[3]; };
// Sensor outputs are the held sample (see sensors.cpp for the timing); ticks counts ticks, a sample is taken when
// ticks % period == 0 (the first tick samples).
struct ImuState { double gyro[3], accel[3], gyro_bias[3], accel_bias[3]; Rng rng; std::uint64_t ticks; };
struct BaroState { double pressure; Rng rng; std::uint64_t ticks; };
struct MagState { double field[3]; Rng rng; std::uint64_t ticks; };
struct GnssState {
  double lat, lon, alt, vel_ned[3];  // output: deg, deg, m, m/s (the sample taken `delay` samples ago)
  double ring[kGnssRing][6];         // lat, lon, alt, vel_ned of the last samples, sample k at ring[k % kGnssRing]
  Rng rng;
  std::uint64_t ticks, samples;      // samples: taken so far
};

template <class C, class S> struct Group {
  using Cfg = C;
  using State = S;
  std::vector<C> cfg;
  std::vector<S> st;
};

struct Devices {
  Group<BatteryCfg, BatteryState> battery;
  Group<PsuCfg, PsuState> psu;
  Group<WireCfg, WireState> wire;
  Group<LoadCfg, LoadState> load;
  Group<EscCfg, EscState> esc;
  Group<RotorCfg, RotorState> rotor;
  Group<EnvCfg, EnvState> env;  // 0 or 1 element
  Group<DragCfg, DragState> drag;
  Group<ServoCfg, ServoState> servo;
  Group<ImuCfg, ImuState> imu;
  Group<BaroCfg, BaroState> baro;
  Group<MagCfg, MagState> mag;
  Group<GnssCfg, GnssState> gnss;
  int n_nodes = 0;  // highest node id used
  struct Slot {
    enum Type { battery, psu, wire, load, esc, rotor, env, drag, servo, imu, baro, mag, gnss } type;
    int i;
  };
  std::vector<Slot> slots;  // prim order: type by type, config order within a type
  const std::string& name(int slot) const;
  // Scratch of solve_electrical (rebuilt from scratch every tick, never saved). drv_*: ESCs first, then servos.
  std::vector<int> drv_mode;
  std::vector<double> drv_ic;
  std::vector<int> psu_lim;
  std::vector<double> qvel_k, qvel_k1;  // scratch of the IMU acceleration (sensors.cpp): qvel before / after mj_step2
};

std::optional<std::string> config_text(const mjModel* m);  // the "napl.devices" custom text, if any
Devices parse(const std::string& json);  // throws std::runtime_error naming the device
// Resolves joint names to ids and applies the model-side effects (armature SET, damping owned). Throws on a missing
// joint, a non-hinge joint, or RK4. Idempotent: safe on a model reloaded from saved bytes and after Spawn / Delete.
void bind(mjModel* m, Devices& dv);
// One tick (replaces mj_step): step1 -> device phases -> damping split -> step2.
void step(mjModel* m, mjData* d, Devices& dv);

// Electrical network + ESC phase decisions for the current state (electrical.cpp). dt advances battery RC / SoC.
void solve_electrical(Devices& dv, double dt);

// Environment + aero (aero.cpp). ISA density at world height z (origin altitude 0 without an environment device);
// air velocity at the scene (steady + turbulence, world; zero without an environment device).
double rho_at(const Devices& dv, double z);
double pressure_at(const Devices& dv, double z);  // ISA static pressure, Pa, same altitude convention
void wind_at(const Devices& dv, double out[3]);
// Phase 2 (servo.cpp): encoder reading of every servo, and the firmware position loop on its own period.
void step_firmware(const mjModel* m, const mjData* d, Devices& dv);
// Phase 5: advances the environment (turbulence), then applies rotor and body-drag forces to d->xfrc_applied.
void step_forces(const mjModel* m, mjData* d, Devices& dv);
// Post-step hook (sensors.cpp): samples the sensors that are due, from the derived data of the tick just stepped.
void sample_sensors(const mjModel* m, mjData* d, Devices& dv);

std::size_t state_bytes(const Devices& dv);
void save_state(const Devices& dv, std::byte* out);
void load_state(Devices& dv, const std::byte* in);

Result get(const Devices& dv, int slot, const std::string& attr, std::vector<double>& out);
Result set(Devices& dv, int slot, const std::string& attr, std::span<const double> values);
std::string unit(const Devices& dv, int slot, const std::string& attr);  // "" if unknown

}  // namespace devs
