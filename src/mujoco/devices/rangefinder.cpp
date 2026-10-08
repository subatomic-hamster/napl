// Range sensors: depth camera, ToF, lidar (phase 5). Post-step hook of the device tick (after mj_step2), like the other sensors: the
// sample describes the state at the START of the sampling tick (xpos, site pose of s_k) and is visible from its end on; the first
// tick samples (ticks % period == 0). Between samples the image is held.
//
// Geometry: mj_multiRay from the site origin along the precomputed unit rays rotated by the site frame, against the geoms of
// ray_groups, the sensor's own body excluded, cutoff max_range. mj_ray skips fully transparent geoms (alpha 0) and has no notion of
// contype / conaffinity, so what the sensor sees is the set of geoms in the mask, visible or not collidable. Deterministic: a pure
// function of the pose, the geometry and the device RNG.
//
// Fidelity (docs/models.md): ideal pinhole / angular grid, no lens distortion, no rolling shutter, no motion blur (the pose of the
// sampling tick), a ray hits the nearest surface of any material (no specular / transparent / dark-surface dropouts, no multipath, no
// edge flying pixels, no stereo occlusion shadow), range-dependent Gaussian noise sigma0 + sigma1 r + sigma2 r^2 per valid pixel,
// uncorrelated between pixels and frames (a real stereo camera's error is spatially correlated and surface dependent).
#include <algorithm>
#include <cmath>

#include "mujoco/devices/devices.h"

namespace devs {

void sample_ranges(const mjModel* m, mjData* d, Devices& dv) {
  if (dv.range.cfg.empty()) return;
  dv.range_img.resize(dv.range.cfg.size());
  for (std::size_t i = 0; i < dv.range.cfg.size(); ++i) {
    const RangefinderCfg& c = dv.range.cfg[i];
    RangefinderState& s = dv.range.st[i];
    std::vector<double>& img = dv.range_img[i];
    const std::size_t n = static_cast<std::size_t>(c.n_h) * static_cast<std::size_t>(c.n_v);
    if (img.size() != n) img.assign(n, 0.0);
    if (s.ticks++ % static_cast<std::uint64_t>(c.period) != 0) continue;
    const double* R = d->site_xmat + 9 * c.sid;
    const double* o = d->site_xpos + 3 * c.sid;
    std::vector<double> dirs(3 * n), dist(n);
    std::vector<int> geom(n);
    for (std::size_t k = 0; k < n; ++k)
      for (int r = 0; r < 3; ++r)
        dirs[3 * k + static_cast<std::size_t>(r)] =
            R[3 * r] * c.dir[3 * k] + R[3 * r + 1] * c.dir[3 * k + 1] + R[3 * r + 2] * c.dir[3 * k + 2];
    mjtByte groups[mjNGROUP] = {0};
    for (int g = 0; g < 6; ++g) groups[g] = (c.ray_groups >> g) & 1;
    mj_multiRay(m, d, o, dirs.data(), groups, 1, c.body, geom.data(), dist.data(), nullptr, static_cast<int>(n), c.cutoff);
    double valid = 0;
    for (std::size_t k = 0; k < n; ++k) {
      double v = 0.0;
      if (dist[k] >= 0) {
        double r = dist[k];
        const double sg = c.s0 + c.s1 * r + c.s2 * r * r;
        if (sg > 0) r += sg * s.rng.gauss();
        if (c.res > 0) r = std::round(r / c.res) * c.res;
        const bool drop = c.dropout > 0 && s.rng.uniform() < c.dropout;
        v = r * c.scale[k];
        if (drop || !(v >= c.min_r && v <= c.max_r)) v = 0.0;
      }
      img[k] = v;
      valid += v > 0 ? 1.0 : 0.0;
    }
    s.frames += 1;
    s.valid = valid;
    s.t = d->time;
  }
}

}  // namespace devs
