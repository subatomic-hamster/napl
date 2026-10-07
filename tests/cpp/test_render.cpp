// M9 exit checks (ctest label `gpu`, names render.*): the headless Vulkan renderer and lidar.
//
// Golden provenance: tests/data/golden/scene.png is what THIS test renders (kGoldenCam, render_scene.xml, 320x240) on
// lavapipe (Mesa llvmpipe, VK_DRIVER_FILES=.../lvp_icd.json) -- lavapipe is the source of truth. Every run writes its
// own image to <build>/render-out/ (scene.png, scene_shifted.png); to regenerate, run the tests on lavapipe, check the
// image by eye (tools/remote.sh pull build/render-out/scene.png, then view it) and copy it over the golden. Other
// GPUs/drivers are compared against it with the same PSNR threshold; camera images are deterministic per driver only.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <vector>

#include "core/world.h"
#include "flatbuffers/flatbuffers.h"
#include "lodepng.h"
#include "mujoco/mujoco_solver.h"
#include "napl_generated.h"
#include "render/renderer.h"

namespace fs = std::filesystem;

namespace {
const std::string kScene = NAPL_TEST_DATA_DIR "/render_scene.xml";
const std::string kGolden = NAPL_TEST_DATA_DIR "/golden/scene.png";
constexpr double kMinPsnr = 35;
constexpr int kW = 320, kH = 240;
const Camera kGoldenCam = {{2.5, -2.2, 1.6}, {0, 0, 0.25}, 45};

std::unique_ptr<Solver> scene() { return load_mujoco(kScene); }

fs::path out_dir() {
  fs::create_directories(NAPL_RENDER_OUT_DIR);
  return NAPL_RENDER_OUT_DIR;
}

void write_png(const fs::path& p, const Image& im) {
  ASSERT_EQ(lodepng::encode(p.string(), im.rgba, static_cast<unsigned>(im.width), static_cast<unsigned>(im.height)), 0u);
}

double psnr_rgb(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
  EXPECT_EQ(a.size(), b.size());
  double se = 0;
  std::size_t n = 0;
  for (std::size_t i = 0; i + 3 < a.size() && i + 3 < b.size(); i += 4)
    for (int c = 0; c < 3; ++c, ++n) {
      const double d = static_cast<double>(a[i + c]) - b[i + c];
      se += d * d;
    }
  return se == 0 ? std::numeric_limits<double>::infinity() : 10 * std::log10(255.0 * 255.0 / (se / static_cast<double>(n)));
}

std::vector<std::uint8_t> load_golden() {
  std::vector<std::uint8_t> px;
  unsigned w = 0, h = 0;
  EXPECT_EQ(lodepng::decode(px, w, h, kGolden), 0u) << "cannot read golden " << kGolden;
  EXPECT_EQ(w, static_cast<unsigned>(kW));
  EXPECT_EQ(h, static_cast<unsigned>(kH));
  return px;
}
}  // namespace

// (a) Camera 2.75 m from the box face x = -0.25, optical axis along +x through the face centre: the centre pixel's
// linear depth is exactly 2.75 m.
TEST(render, depth_center_matches_analytic) {
  Renderer r;
  std::cout << "device: " << r.device_name() << "\n";
  const auto s = scene();
  const Image im = r.render(*s, Camera{{-3, 0, 0.25}, {0, 0, 0.25}, 45}, 161, 121);  // odd size: a pixel sits on the axis
  const float d = im.depth[static_cast<std::size_t>(60) * 161 + 80];
  std::cout << "center depth " << d << " m, analytic 2.75, error " << std::abs(d - 2.75) << " m\n";
  EXPECT_NEAR(d, 2.75, 0.01);
  EXPECT_TRUE(std::isinf(im.depth[0]) || im.depth[0] > 2.75f);  // top-left corner looks over the floor/background
}

// The solver seam's mesh visuals: unrolled triangles whose stored normals agree with the winding (outward).
TEST(render, mesh_visuals_unrolled_with_outward_normals) {
  const auto s = scene();
  const std::vector<GeomVisual> vis = s->visuals();
  const GeomVisual* mesh = nullptr;
  for (const GeomVisual& v : vis)
    if (v.type == GeomType::mesh) mesh = &v;
  ASSERT_NE(mesh, nullptr);
  ASSERT_EQ(mesh->mesh.size(), 4u * 3 * 6);  // tetrahedron: 4 triangles x 3 vertices x (pos + normal)
  double c[3] = {0, 0, 0};  // centroid: every vertex appears 3 times
  for (std::size_t i = 0; i < 12; ++i)
    for (int k = 0; k < 3; ++k) c[k] += mesh->mesh[i * 6 + k] / 12.0;
  for (std::size_t t = 0; t < 4; ++t) {
    const float* v = mesh->mesh.data() + t * 18;
    double a[3], b[3], n[3];
    for (int k = 0; k < 3; ++k) {
      a[k] = v[6 + k] - v[k];
      b[k] = v[12 + k] - v[k];
    }
    n[0] = a[1] * b[2] - a[2] * b[1];
    n[1] = a[2] * b[0] - a[0] * b[2];
    n[2] = a[0] * b[1] - a[1] * b[0];
    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    EXPECT_GT(n[0] * (v[0] - c[0]) + n[1] * (v[1] - c[1]) + n[2] * (v[2] - c[2]), 0) << "triangle " << t << " winds inward";
    for (int i = 0; i < 3; ++i) {
      const float* nv = v + 6 * i + 3;
      const double d = (n[0] * nv[0] + n[1] * nv[1] + n[2] * nv[2]) / len;
      EXPECT_GT(d, 0.99) << "triangle " << t << " vertex " << i;
    }
  }
}

// (b) golden image
TEST(render, golden_psnr) {
  Renderer r;
  const auto s = scene();
  const Image im = r.render(*s, kGoldenCam, kW, kH);
  write_png(out_dir() / "scene.png", im);
  const double p = psnr_rgb(im.rgba, load_golden());
  std::cout << "device: " << r.device_name() << "\nPSNR vs golden: " << p << " dB\n";
  EXPECT_GT(p, kMinPsnr);
}

// (c) the same scene and camera, everything shifted by 1e5 m in double: same image. float(1e5 + x) has 8 mm
// resolution, so a renderer that uploaded world coordinates would smear the geometry visibly.
TEST(render, shifted_1e5_same_image) {
  Renderer r;
  const auto s = scene();
  r.set_scene(s->visuals());
  std::vector<GeomPose> poses = s->poses();
  constexpr double kShift = 1e5;
  for (GeomPose& p : poses)
    for (double& x : p.pos) x += kShift;
  Camera cam = kGoldenCam;
  for (int k = 0; k < 3; ++k) {
    cam.pos[k] += kShift;
    cam.lookat[k] += kShift;
  }
  const Image im = r.draw(poses, cam, kW, kH);
  write_png(out_dir() / "scene_shifted.png", im);
  const double p = psnr_rgb(im.rgba, load_golden());
  const Image base = r.draw(s->poses(), kGoldenCam, kW, kH);
  std::cout << "device: " << r.device_name() << "\nPSNR shifted vs golden: " << p << " dB (unshifted vs shifted: "
            << psnr_rgb(im.rgba, base.rgba) << " dB)\n";
  EXPECT_GT(p, kMinPsnr);
}

// (d) lidar vs analytic distances to the box (centre (0,0,0.25), half size 0.25) and the floor, to 1e-9; also through
// the bus (binary reply: full double precision).
TEST(render, lidar_matches_analytic_box) {
  const auto s = scene();
  const double o1[3] = {-2, 0, 0.25}, o2[3] = {0, 0, 3}, o3[3] = {2, 2, 1};
  const double a1 = std::atan2(0.1, 1);  // ray (1, 0.1, 0): hits the face x = -0.25 at t = 1.75 / cos(a)
  struct Case { const double* o; std::vector<double> d; double want; } cases[] = {
      {o1, {1, 0, 0}, 1.75},
      {o1, {2, 0, 0}, 1.75},  // length does not matter
      {o1, {1, 0.1, 0}, 1.75 / std::cos(a1)},
      {o2, {0, 0, -1}, 2.5},  // down onto the box top
      {o3, {0, 0, -1}, 1.0},  // down onto the floor
      {o1, {0, 0, 1}, -1.0},  // up into the sky: miss
  };
  double worst = 0;
  for (const Case& c : cases) {
    const std::vector<double> got = s->raycast({c.o, 3}, c.d);
    ASSERT_EQ(got.size(), 1u);
    worst = std::max(worst, std::abs(got[0] - c.want));
    EXPECT_NEAR(got[0], c.want, 1e-9);
  }
  std::cout << "lidar max error " << worst << " m\n";

  World w(load_mujoco, kScene);
  flatbuffers::FlatBufferBuilder fbb;
  const double origin[3] = {-2, 0, 0.25}, dirs[6] = {1, 0, 0, 1, 0.1, 0};
  fbb.Finish(napl::CreateEnvelope(fbb, 1, fbb.CreateString("test"), napl::Command::Raycast,
                                  napl::CreateRaycast(fbb, fbb.CreateVector(origin, 3), fbb.CreateVector(dirs, 6)).Union()));
  Session session;
  session.caps = {"read"};
  const flatbuffers::DetachedBuffer reply = w.execute({fbb.GetBufferPointer(), fbb.GetSize()}, session);
  const napl::Reply* rep = flatbuffers::GetRoot<napl::Reply>(reply.data());
  ASSERT_TRUE(rep->ok()) << rep->message()->str();
  ASSERT_EQ(rep->distances()->size(), 2u);
  EXPECT_NEAR((*rep->distances())[0], 1.75, 1e-9);
  EXPECT_NEAR((*rep->distances())[1], 1.75 / std::cos(a1), 1e-9);
}

// (e) error paths: bad size is invalid_argument (before any GPU work); with no Vulkan device the reply is a clear
// error, not a crash.
TEST(render, error_bad_size) {
  World w(load_mujoco, kScene);
  w.set_render_hook(make_render_hook());
  Session session;
  session.caps = {"read"};
  const std::string cam = R"("pos":[-3,0,1],"lookat":[0,0,0.25])";
  for (const char* size : {R"("width":0,"height":10)", R"("width":10,"height":4097)"}) {
    const std::string reply = execute_json(w, R"({"cmd_type":"Render","cmd":{)" + cam + "," + size + "}}", session);
    EXPECT_NE(reply.find(R"("ok": false)"), std::string::npos) << reply;
    EXPECT_NE(reply.find("invalid_argument"), std::string::npos) << reply;
  }
}

TEST(render, error_no_vulkan_device) {
  ASSERT_EQ(setenv("VK_DRIVER_FILES", "/nonexistent/none_icd.json", 1), 0);  // own process: ctest runs one test each
  try {
    Renderer r;
    FAIL() << "Renderer constructed without a driver";
  } catch (const std::runtime_error& e) {
    std::cout << "no-device error: " << e.what() << "\n";
    EXPECT_NE(std::string(e.what()).find("Vulkan"), std::string::npos);
  }
  World w(load_mujoco, kScene);
  w.set_render_hook(make_render_hook());
  Session session;
  session.caps = {"read"};
  const std::string reply = execute_json(
      w, R"({"cmd_type":"Render","cmd":{"pos":[-3,0,1],"lookat":[0,0,0.25],"width":32,"height":32}})", session);
  EXPECT_NE(reply.find(R"("ok": false)"), std::string::npos) << reply;
  EXPECT_NE(reply.find("solver_error"), std::string::npos) << reply;
  EXPECT_NE(reply.find("Vulkan"), std::string::npos) << reply;
}
