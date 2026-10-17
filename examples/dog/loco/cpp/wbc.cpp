// C++ port of loco/wbc.py (SP4 Task 7 Step 6): the same whole-body controller, same constants (read from the Python modules at construction), same
// tasks, same QP, so test_cpp_wbc_matches_python can compare the two. Eigen 3.4 (MPL-2.0) for the algebra, the OSQP C library (Apache-2.0) for the QP.
// Kinematics are computed ONCE per solve (Python recomputes them four times) and each leg chain once for all its links.
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <limits>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <osqp.h>
#include <vector>

namespace nb = nanobind;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat3x18 = Eigen::Matrix<double, 3, 18>;
using Vec18 = Eigen::Matrix<double, 18, 1>;
using Mat18 = Eigen::Matrix<double, 18, 18>;
using Mat12x18 = Eigen::Matrix<double, 12, 18>;
using Mat43 = Eigen::Matrix<double, 4, 3, Eigen::RowMajor>;
using MatX = Eigen::MatrixXd;
using VecX = Eigen::VectorXd;

static Mat3 skew(const Vec3 &v) {
  Mat3 K;
  K << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return K;
}
// model.rot: axis w (unit), angle th
static Mat3 rot_axis(const Vec3 &w, double th) {
  Mat3 K = skew(w);
  return Mat3::Identity() + std::sin(th) * K + (1 - std::cos(th)) * K * K;
}
// wbc._rot: rotation vector w * th
static Mat3 rot_vec(const Vec3 &w, double th) {
  double nw = w.norm(), n = nw * th;
  if (n < 1e-12) return Mat3::Identity();
  Mat3 K = skew(w / nw);
  return Mat3::Identity() + std::sin(n) * K + (1 - std::cos(n)) * K * K;
}
static Vec3 log_so3(const Mat3 &E) {
  double c = std::clamp((E.trace() - 1) / 2, -1.0, 1.0), th = std::acos(c);
  Vec3 v(E(2, 1) - E(1, 2), E(0, 2) - E(2, 0), E(1, 0) - E(0, 1));
  return th < 1e-6 ? Vec3(v / 2) : Vec3(v * th / (2 * std::sin(th)));
}

struct Leg {
  Mat3 w, p;  // rows = joint axes / axis points
  Vec3 foot0, tau_max;
};
struct Link {
  int leg, upto;
  double mass;
  Vec3 com;
  Mat3 inertia;
};
struct Chain {  // Leg._chain for upto = 0..3
  Mat3 R[4];
  Vec3 t[4], ws[3], ps[3];
};
struct Kin {
  std::vector<Mat3x18> Jv, Jw;
  std::vector<Mat3> I;
  Mat12x18 Jc;
  std::array<Vec3, 4> r;  // foot ball centre, trunk frame
};

struct Wbc {
  std::array<Leg, 4> legs;
  std::vector<Link> links;
  VecX armature;
  VecX tau_max;
  double KP_BODY, KD_BODY, KP_ORI, KD_ORI, KP_FOOT, KD_FOOT, W_FB, W_F, LAM, LAM_DYN, STEP_MAX, V_MAX, EPS, G, F_MIN, F_MAX;
  nb::object WbcOut;
  // OSQP
  static constexpr int NC = 6 + 20 + 12, NV = 18;
  OSQPSolver *solver = nullptr;
  OSQPCscMatrix Pm, Am;
  std::vector<OSQPFloat> Px, Ax, lv, uv, qv;
  std::vector<OSQPInt> Pi, Pp, Ai, Ap;
  int n_fail = 0;

  explicit Wbc(nb::object model) {
    nb::module_ W = nb::module_::import_("loco.wbc"), Mp = nb::module_::import_("loco.mpc");
    auto f = [&](const char *n) { return nb::cast<double>(W.attr(n)); };
    KP_BODY = f("KP_BODY"), KD_BODY = f("KD_BODY"), KP_ORI = f("KP_ORI"), KD_ORI = f("KD_ORI"), KP_FOOT = f("KP_FOOT"), KD_FOOT = f("KD_FOOT");
    W_FB = f("W_FB"), W_F = f("W_F"), LAM = f("LAM"), LAM_DYN = f("LAM_DYN"), STEP_MAX = f("STEP_MAX"), V_MAX = f("V_MAX"), EPS = f("EPS");
    G = nb::cast<double>(Mp.attr("G")), F_MIN = nb::cast<double>(Mp.attr("F_MIN")), F_MAX = nb::cast<double>(Mp.attr("F_MAX"));
    WbcOut = W.attr("WbcOut");
    nb::list lg(model.attr("legs").attr("values")());
    tau_max.resize(12);
    for (int i = 0; i < 4; i++) {
      nb::object L = lg[i];
      legs[i].w = nb::cast<Mat3>(L.attr("w")), legs[i].p = nb::cast<Mat3>(L.attr("p"));
      legs[i].foot0 = nb::cast<Vec3>(L.attr("foot0")), legs[i].tau_max = nb::cast<Vec3>(L.attr("tau_max"));
      tau_max.segment<3>(3 * i) = legs[i].tau_max;
    }
    for (nb::handle h : model.attr("links")) {
      nb::object lk = nb::borrow(h);
      links.push_back({nb::cast<int>(lk.attr("leg")), nb::cast<int>(lk.attr("upto")), nb::cast<double>(lk.attr("mass")),
                       nb::cast<Vec3>(lk.attr("com")), nb::cast<Mat3>(lk.attr("inertia"))});
    }
    armature = nb::cast<VecX>(model.attr("armature"));
    // QP skeleton: P = diag(W_FB x 6, W_F x 12) (upper triangle), A dense NC x 18 in CSC
    for (int j = 0; j < NV; j++) Pp.push_back(j), Pi.push_back(j), Px.push_back(j < 6 ? W_FB : W_F);
    Pp.push_back(NV);
    for (int j = 0; j < NV; j++) {
      Ap.push_back(j * NC);
      for (int r = 0; r < NC; r++) Ai.push_back(r);
    }
    Ap.push_back(NV * NC);
    Ax.assign(NC * NV, 1.0), lv.assign(NC, 0.0), uv.assign(NC, 0.0), qv.assign(NV, 0.0);
  }
  ~Wbc() {
    if (solver) osqp_cleanup(solver);
  }
  Wbc(const Wbc &) = delete;

  Chain chain(int i, const double *q) const {
    const Leg &L = legs[i];
    Chain c;
    c.R[0].setIdentity(), c.t[0].setZero();
    for (int j = 0; j < 3; j++) {
      Vec3 wj = L.w.row(j), pj = L.p.row(j);
      c.ws[j] = c.R[j] * wj, c.ps[j] = c.R[j] * pj + c.t[j];
      Mat3 Rj = rot_axis(wj, q[j]);
      c.t[j + 1] = c.R[j] * (pj - Rj * pj) + c.t[j], c.R[j + 1] = c.R[j] * Rj;
    }
    return c;
  }

  // loco.wbc._elements
  void kin(const Mat3 &R, const double *q, Kin &k) const {
    Chain ch[4];
    for (int i = 0; i < 4; i++) ch[i] = chain(i, q + 3 * i);
    k.Jc.setZero();
    for (int i = 0; i < 4; i++) {
      const Chain &c = ch[i];
      Vec3 x = c.R[3] * legs[i].foot0 + c.t[3];
      k.r[i] = x;
      k.Jc.block<3, 3>(3 * i, 0).setIdentity();
      k.Jc.block<3, 3>(3 * i, 3) = -skew(R * x);
      for (int j = 0; j < 3; j++) k.Jc.block<3, 1>(3 * i, 6 + 3 * i + j) = R * c.ws[j].cross(x - c.ps[j]);
    }
    k.Jv.assign(links.size(), Mat3x18::Zero()), k.Jw.assign(links.size(), Mat3x18::Zero()), k.I.resize(links.size());
    for (size_t n = 0; n < links.size(); n++) {
      const Link &lk = links[n];
      Mat3x18 &Jv = k.Jv[n], &Jw = k.Jw[n];
      Jv.block<3, 3>(0, 0).setIdentity(), Jw.block<3, 3>(0, 3).setIdentity();
      Vec3 x;
      Mat3 Rk;
      if (lk.leg < 0) {
        x = lk.com, Rk.setIdentity();
      } else {
        const Chain &c = ch[lk.leg];
        Rk = c.R[lk.upto], x = Rk * lk.com + c.t[lk.upto];
        for (int j = 0; j < lk.upto; j++) {
          Jv.col(6 + 3 * lk.leg + j) = R * c.ws[j].cross(x - c.ps[j]);
          Jw.col(6 + 3 * lk.leg + j) = R * c.ws[j];
        }
      }
      Jv.block<3, 3>(0, 3) = -skew(R * x);
      Mat3 Rw = R * Rk;
      k.I[n] = Rw * lk.inertia * Rw.transpose();
    }
  }

  // loco.wbc.dynamics: A, b = C(q, u) by finite difference, g; k1 = the kinematics advanced by EPS along u
  void dynamics(const Mat3 &R, const VecX &q, const Vec18 &u, Mat18 &A, Vec18 &b, Vec18 &g, Kin &k0, Kin &k1) const {
    kin(R, q.data(), k0);
    kin(rot_vec(u.segment<3>(3), EPS) * R, (q + EPS * u.tail(12)).eval().data(), k1);
    A = Mat18::Zero();
    A.diagonal().tail(12) = armature;
    b = Vec18::Zero(), g = Vec18::Zero();
    for (size_t n = 0; n < links.size(); n++) {
      const Mat3x18 &Jv = k0.Jv[n], &Jw = k0.Jw[n];
      double m = links[n].mass;
      A += m * Jv.transpose() * Jv + Jw.transpose() * k0.I[n] * Jw;
      g += m * G * Jv.row(2).transpose();
      Vec3 wk = Jw * u;
      b += m * Jv.transpose() * ((k1.Jv[n] - Jv) * u / EPS) + Jw.transpose() * (k0.I[n] * ((k1.Jw[n] - Jw) * u / EPS) + wk.cross(k0.I[n] * wk));
    }
  }
  // (A, b, g, Jc (12, 18), Jc-dot u (12)) for the tests
  nb::tuple debug(const Mat3 &R, const VecX &q, const Vec18 &u) const {
    Kin k0, k1;
    Mat18 A;
    Vec18 b, g;
    dynamics(R, q, u, A, b, g, k0, k1);
    MatX Jc = k0.Jc;
    VecX Jd = (k1.Jc - k0.Jc) * u / EPS;
    return nb::make_tuple(A, b, g, Jc, Jd);
  }

  MatX pinv(const MatX &J) const {
    MatX S = J * J.transpose() + LAM * MatX::Identity(J.rows(), J.rows());
    return J.transpose() * S.inverse();
  }
  MatX pinv_dyn(const MatX &J, const Mat18 &Ainv) const {
    MatX B = Ainv * J.transpose(), S = J * B + LAM_DYN * MatX::Identity(J.rows(), J.rows());
    return B * S.inverse();
  }

  nb::object fail(const VecX &q, const VecX &dpos, const VecX &dvel, const Mat43 &f_mpc) const {
    VecX p = q + dpos.tail(12).cwiseMax(-STEP_MAX).cwiseMin(STEP_MAX), v = dvel.tail(12).cwiseMax(-V_MAX).cwiseMin(V_MAX), tau = VecX::Zero(12);
    return WbcOut(p, v, tau, f_mpc, false);
  }

  nb::object solve(const Vec3 &p, const Mat3 &R, const Vec3 &w, const Vec3 &v, const VecX &q, const VecX &dq, const Mat43 &f_mpc, nb::object stance_o,
                   const VecX &mu, nb::tuple body_ref, nb::tuple swing) {
    bool stance[4];
    for (int i = 0; i < 4; i++) stance[i] = PyObject_IsTrue(stance_o[nb::int_(i)].ptr()) == 1;
    Vec3 p_ref = nb::cast<Vec3>(body_ref[0]), v_ref = nb::cast<Vec3>(body_ref[1]);
    Mat3 R_ref = nb::cast<Mat3>(body_ref[2]);
    Mat43 sw_x = nb::cast<Mat43>(swing[0]), sw_v = nb::cast<Mat43>(swing[1]);

    Vec18 u;
    u << v, w, dq;
    Kin k0, k1;
    Mat18 A;
    Vec18 b, g;
    dynamics(R, q, u, A, b, g, k0, k1);
    Mat18 Ainv = A.inverse();
    const Mat12x18 &Jc = k0.Jc;
    VecX Jcd_u = (k1.Jc - Jc) * u / EPS;  // (12)

    // ---- 1. kinematic tasks, strict priorities
    std::vector<int> st, sw;
    for (int i = 0; i < 4; i++) (stance[i] ? st : sw).push_back(i);
    VecX qdd = VecX::Zero(18);
    MatX N = MatX::Identity(18, 18);
    if (!st.empty()) {
      MatX Jst(3 * st.size(), 18);
      VecX rhs(3 * st.size());
      for (size_t a = 0; a < st.size(); a++) Jst.block(3 * a, 0, 3, 18) = Jc.block<3, 18>(3 * st[a], 0), rhs.segment<3>(3 * a) = -Jcd_u.segment<3>(3 * st[a]);
      qdd = pinv_dyn(Jst, Ainv) * rhs;
      N -= pinv(Jst) * Jst;
    }
    struct Task {
      MatX J;
      VecX Jd_u, e, vd;
      double kp, kd;
    };
    std::vector<Task> tasks;
    {
      MatX Jo = MatX::Zero(3, 18), Jp = MatX::Zero(3, 18);
      Jo.block<3, 3>(0, 3).setIdentity(), Jp.block<3, 3>(0, 0).setIdentity();
      tasks.push_back({Jo, Vec3::Zero(), log_so3(R_ref * R.transpose()), Vec3::Zero(), KP_ORI, KD_ORI});
      tasks.push_back({Jp, Vec3::Zero(), p_ref - p, v_ref, KP_BODY, KD_BODY});
    }
    for (int i : sw) {
      Vec3 x = p + R * k0.r[i];
      tasks.push_back({Jc.block<3, 18>(3 * i, 0), Jcd_u.segment<3>(3 * i), Vec3(sw_x.row(i).transpose()) - x, Vec3(sw_v.row(i).transpose()), KP_FOOT, KD_FOOT});
    }
    VecX dpos = VecX::Zero(18), dvel = VecX::Zero(18);
    for (const Task &T : tasks) {
      MatX JN = T.J * N, JNp = pinv(JN);
      dpos += JNp * (T.e - T.J * dpos);
      dvel += JNp * (T.vd - T.J * dvel);
      qdd += pinv_dyn(JN, Ainv) * (T.kp * T.e + T.kd * (T.vd - T.J * u) - T.Jd_u - T.J * qdd);
      N = N * (MatX::Identity(18, 18) - JNp * JN);
    }

    // ---- 2. dynamics QP
    Eigen::Matrix<double, 12, 1> f_ref;
    for (int i = 0; i < 4; i++) f_ref.segment<3>(3 * i) = stance[i] ? Vec3(f_mpc.row(i).transpose()) : Vec3::Zero();
    Eigen::Matrix<double, 18, 12> JcT = Jc.transpose();
    Vec18 h = A * qdd + b + g - JcT * f_ref;
    Eigen::Matrix<double, NC, NV> M = Eigen::Matrix<double, NC, NV>::Zero();
    VecX lo = VecX::Zero(NC), up = VecX::Zero(NC);
    M.block<6, 6>(0, 0) = A.block<6, 6>(0, 0), M.block<6, 12>(0, 6) = -JcT.topRows<6>();
    lo.head(6) = up.head(6) = -h.head(6);
    for (int i = 0; i < 4; i++) {
      double m = mu[i];
      Eigen::Matrix<double, 5, 3> rows;
      rows << 1, 0, -m, -1, 0, -m, 0, 1, -m, 0, -1, -m, 0, 0, 1;
      int r0 = 6 + 5 * i;
      M.block<5, 3>(r0, 6 + 3 * i) = rows;
      Vec3 fr = f_ref.segment<3>(3 * i);
      Eigen::Matrix<double, 4, 1> ub = -(rows.topRows<4>() * fr);
      for (int r = 0; r < 4; r++) lo[r0 + r] = -OSQP_INFTY, up[r0 + r] = ub[r];
      lo[r0 + 4] = F_MIN * stance[i] - fr[2], up[r0 + 4] = F_MAX * stance[i] - fr[2];
    }
    M.block<12, 6>(26, 0) = A.block<12, 6>(6, 0), M.block<12, 12>(26, 6) = -JcT.bottomRows<12>();
    lo.tail(12) = -tau_max - h.tail(12), up.tail(12) = tau_max - h.tail(12);
    for (int j = 0; j < NV; j++)
      for (int r = 0; r < NC; r++) Ax[j * NC + r] = M(r, j);
    for (int r = 0; r < NC; r++) lv[r] = std::max(lo[r], -OSQP_INFTY), uv[r] = std::min(up[r], OSQP_INFTY);
    bool ok = true;
    if (!solver) {
      OSQPCscMatrix_set_data(&Pm, NV, NV, NV, Px.data(), Pi.data(), Pp.data());
      OSQPCscMatrix_set_data(&Am, NC, NV, NC * NV, Ax.data(), Ai.data(), Ap.data());
      OSQPSettings s;
      osqp_set_default_settings(&s);
      s.verbose = 0, s.warm_starting = 1, s.polishing = 1, s.eps_abs = 1e-5, s.eps_rel = 1e-5, s.max_iter = 2000;
      s.adaptive_rho_interval = 50;  // iteration-count based: 0 would time the setup and make the result depend on the machine's speed
      ok = osqp_setup(&solver, &Pm, qv.data(), &Am, lv.data(), uv.data(), NC, NV, &s) == 0;
    } else {
      ok = osqp_update_data_mat(solver, nullptr, nullptr, 0, Ax.data(), nullptr, 0) == 0 &&
           osqp_update_data_vec(solver, nullptr, lv.data(), uv.data()) == 0;
    }
    if (ok) ok = osqp_solve(solver) == 0 && (solver->info->status_val == OSQP_SOLVED || solver->info->status_val == OSQP_SOLVED_INACCURATE);
    VecX qj = q;
    if (!ok) {
      n_fail++;
      return fail(qj, dpos, dvel, f_mpc);
    }
    Eigen::Map<const Eigen::Matrix<double, NV, 1>> z(solver->solution->x);
    Mat43 f;
    Eigen::Matrix<double, 12, 1> ff = f_ref + z.tail(12);
    for (int i = 0; i < 4; i++) f.row(i) = ff.segment<3>(3 * i).transpose();
    VecX tau = (h.tail(12) + A.block<12, 6>(6, 0) * z.head(6) - JcT.bottomRows<12>() * z.tail(12)).cwiseMax(-tau_max).cwiseMin(tau_max);
    VecX pd = qj + dpos.tail(12).cwiseMax(-STEP_MAX).cwiseMin(STEP_MAX), vd = dvel.tail(12).cwiseMax(-V_MAX).cwiseMin(V_MAX);
    return WbcOut(pd, vd, tau, f, true);
  }
};

NB_MODULE(loco_wbc_cpp, m) {
  nb::class_<Wbc>(m, "Wbc")
      .def(nb::init<nb::object>())
      .def("solve", &Wbc::solve)
      .def("debug", &Wbc::debug)
      .def_prop_ro("n_fail", [](const Wbc &w) { return w.n_fail; });
}
