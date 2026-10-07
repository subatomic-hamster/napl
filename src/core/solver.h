#pragma once
// The ONE solver interface (seam). napl_core must never include a solver's headers.
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

// Static look of one geom (what the renderer tessellates once). visuals() and poses() are index-parallel.
// size: plane = half extents x,y (<= 0: unbounded); sphere = r; capsule = r, half length (along local z);
// ellipsoid = semi-axes; cylinder = r, half length; box = half extents. `other` (heightfield, SDF, ...) is not drawn.
enum class GeomType { plane, sphere, capsule, ellipsoid, cylinder, box, mesh, other };
struct GeomVisual {
  GeomType type = GeomType::other;
  double size[3] = {0, 0, 0};
  float rgba[4] = {0.5f, 0.5f, 0.5f, 1};
  int group = 0;            // renderers hide group >= 3 (MuJoCo's default: collision geoms)
  std::vector<float> mesh;  // mesh only: unrolled triangles, per vertex position xyz then normal xyz (geom frame)
};
// World pose of a geom frame: position in DOUBLE (the renderer subtracts the camera in double) + row-major 3x3 rotation.
struct GeomPose {
  double pos[3];
  double rot[9];
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

  // Rendering / sensing seam. Pure reads: they change no state (derived kinematics only, never hashed).
  virtual std::vector<GeomVisual> visuals() const = 0;
  virtual std::vector<GeomPose> poses() const = 0;
  // Ray distances in metres from `origin` (3 doubles) along each direction (3 doubles each, any nonzero length; the
  // solver normalises). -1 = no hit. Deterministic. Nothing is excluded.
  virtual std::vector<double> raycast(std::span<const double> origin, std::span<const double> directions) const = 0;

  // Structural edits (Spawn / Delete commands). Existing entities keep their state; prims() changes; a snapshot taken
  // before the edit makes restore() throw. `pos` / `quat`: empty or 3 / 4 values. On failure nothing has changed.
  virtual Result spawn(const std::string& model_path, const std::string& at, std::span<const double> pos,
                       std::span<const double> quat) = 0;
  virtual Result remove(const std::string& prim) = 0;

  // Replay-log header support: the loaded model as a self-contained buffer (a BytesLoader rebuilds a
  // solver from it) and a version string; replay refuses logs whose version differs.
  virtual std::vector<std::byte> model_bytes() const = 0;
  virtual std::string version() const = 0;
};
