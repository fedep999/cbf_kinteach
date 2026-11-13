// Comparison of the joint-space torque CBF (jtorque_cbf, tau as decision variable) with torque_cbf
// (normal force as decision variable).
//
//   1) Equivalence: without rate limit and with max_lateral_force = max_moment = 0, both must give the
//      same CBF torque on varied robot states.
//   2) Closed loop on Panda dynamics (Pinocchio, 1 kHz control, 10 integration substeps, viscous joint
//      friction 1 Nm s/rad unknown to the controllers) with the robot's per-joint torque-rate limiter
//      (999 Nm/s, as the Simulink Rate Limiter in front of the Franka): normal approach, operator
//      push + release, late fast entry.
//      Pushback = velocity the APPLIED CBF torque adds away from the plane while the end-effector moves
//      away: integral of max(0, n^T J M^-1 tau_applied) dt over dzeta > 0.
//
// Usage: compare_jtorque_cbf <path/to/Franka_arm.urdf> [tip_frame=panda_link8]

#include "jtorque_cbf/joint_torque_cbf_controller.hpp"
#include "torque_cbf/torque_cbf_controller.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cstdio>
#include <random>
#include <string>

namespace
{
  constexpr double kRobotRate = 999.0; // [Nm/s] robot torque-rate limiter (Simulink Rate Limiter)
  constexpr double kDt = 0.001;
  constexpr double kFriction = 1.0;    // [Nm s/rad] viscous joint friction of the simulated robot

  struct Kin
  {
    Eigen::MatrixXd M, C, J, dJ;
    Eigen::VectorXd g;
    Eigen::Vector3d x;
    Eigen::Matrix<double, 6, 1> V;
  };

  // Robot state with the TRUE time derivative of the LOCAL_WORLD_ALIGNED Jacobian (classical acceleration)
  Kin kinematics(const pinocchio::Model &m, pinocchio::Data &d, pinocchio::FrameIndex f,
                 const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    Kin k;
    pinocchio::forwardKinematics(m, d, q, dq);
    pinocchio::computeJointJacobians(m, d, q);
    pinocchio::computeJointJacobiansTimeVariation(m, d, q, dq);
    pinocchio::updateFramePlacements(m, d);
    k.J = Eigen::MatrixXd::Zero(6, m.nv);
    k.dJ = Eigen::MatrixXd::Zero(6, m.nv);
    pinocchio::getFrameJacobian(m, d, f, pinocchio::LOCAL_WORLD_ALIGNED, k.J);
    pinocchio::getFrameJacobianTimeVariation(m, d, f, pinocchio::LOCAL_WORLD_ALIGNED, k.dJ);
    pinocchio::crba(m, d, q);
    d.M.triangularView<Eigen::StrictlyLower>() = d.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeCoriolisMatrix(m, d, q, dq);
    pinocchio::computeGeneralizedGravity(m, d, q);
    k.M = d.M;
    k.C = d.C;
    k.g = d.g;
    k.x = d.oMf[f].translation();
    k.V = k.J * dq;
    Eigen::Matrix3d vs;
    const Eigen::Vector3d v = k.V.head<3>();
    vs << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    k.dJ.topRows<3>() -= vs * k.J.bottomRows<3>();
    return k;
  }

  // Same barrier parameters for both controllers (init_torque_cbf.m values)
  torque_cbf::TorqueCBFConfig torqueConfig()
  {
    torque_cbf::TorqueCBFConfig c;
    c.p_ths = 0.30;
    c.v_ths = -0.003;
    c.v_0 = -0.8;
    c.gamma = 12.0;
    c.w_lin = 1e6;
    c.w_slack = 1e4;
    c.dt = kDt;
    c.latch_entry_velocity = true;
    c.zone_exit_hysteresis = 0.005;
    c.latch_velocity_filter_alpha = 0.2;
    c.tau_max = (Eigen::VectorXd(7) << 87, 87, 87, 87, 12, 12, 12).finished();
    c.tau_min = -c.tau_max;
    c.enforce_total_torque_limits = true;
    return c;
  }

  jtorque_cbf::JointTorqueCBFConfig jointConfig(double rate, double eps_f, double eps_m)
  {
    const torque_cbf::TorqueCBFConfig t = torqueConfig();
    jtorque_cbf::JointTorqueCBFConfig c;
    c.p_ths = t.p_ths;
    c.v_ths = t.v_ths;
    c.v_0 = t.v_0;
    c.gamma = t.gamma;
    c.dt = t.dt;
    c.latch_entry_velocity = t.latch_entry_velocity;
    c.zone_exit_hysteresis = t.zone_exit_hysteresis;
    c.latch_velocity_filter_alpha = t.latch_velocity_filter_alpha;
    c.tau_max = t.tau_max;
    c.tau_min = t.tau_min;
    c.enforce_total_torque_limits = true;
    c.tau_rate_limit = rate;
    c.max_lateral_force = eps_f;
    c.max_moment = eps_m;
    return c;
  }

  // ---------------------------------------------------------------- 1) equivalence
  void equivalence(const pinocchio::Model &m, pinocchio::Data &d, pinocchio::FrameIndex f)
  {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uq(-0.4, 0.4), udq(-0.6, 0.6), uf(-15.0, 15.0), uz(0.01, 0.25);
    Eigen::VectorXd q_home(7);
    q_home << 0, 0, 0, -M_PI / 2, 0, M_PI / 2, -M_PI / 4;
    double max_err = 0.0, max_tau = 0.0;
    int n_active = 0;
    for (int k = 0; k < 200; ++k)
    {
      Eigen::VectorXd q = q_home, dq(7);
      for (int j = 0; j < 7; ++j)
      {
        q(j) += uq(rng);
        dq(j) = udq(rng);
      }
      const Kin s = kinematics(m, d, f, q, dq);
      cbf::Hyperplane p;
      p.normal << 0, 0, 1;
      p.offset = s.x - Eigen::Vector3d(0, 0, uz(rng));
      Eigen::Matrix<double, 6, 1> Fe;
      Fe << uf(rng), uf(rng), uf(rng), 0.1 * uf(rng), 0.1 * uf(rng), 0.1 * uf(rng);

      torque_cbf::TorqueCBFController tc(torqueConfig(), p);
      jtorque_cbf::JointTorqueCBFController jc(jointConfig(0.0, 0.0, 0.0), p);
      torque_cbf::RobotState ts;
      ts.q = q; ts.dq = dq; ts.x = s.x; ts.dx = s.V.head<3>(); ts.M = s.M; ts.C = s.C; ts.g = s.g;
      ts.J = s.J; ts.dJ = s.dJ; ts.F_ext = Fe;
      jtorque_cbf::RobotState js;
      js.q = q; js.dq = dq; js.x = s.x; js.dx = s.V.head<3>(); js.M = s.M; js.C = s.C; js.g = s.g;
      js.J = s.J; js.dJ = s.dJ; js.F_ext = Fe;
      torque_cbf::TorqueCBFResult tr;
      jtorque_cbf::JointTorqueCBFResult jr;
      tc.computeControlTorque(ts, Eigen::VectorXd(), &tr);
      jc.computeControlTorque(js, Eigen::VectorXd(), &jr);
      if (tr.f_n_scalar > 1e-6) ++n_active;
      max_err = std::max(max_err, (tr.tau_cbf - jr.tau_cbf).cwiseAbs().maxCoeff());
      max_tau = std::max(max_tau, tr.tau_cbf.cwiseAbs().maxCoeff());
    }
    std::printf("1) Equivalence (no rate limit, eps = 0), 200 random states, %d with CBF force:\n", n_active);
    std::printf("   max |tau_jtorque - tau_torque| = %.2e Nm   (largest CBF torque %.2f Nm)\n\n", max_err, max_tau);
  }

  // ---------------------------------------------------------------- 2) closed loop
  struct Metrics
  {
    double v_contact = 0.0, min_zeta = 1e9, push_stop = 0.0, push_release = 0.0;
    double mismatch = 0.0, lateral = 0.0, power_pos = 0.0, sigma_n = 0.0, sigma_p = 0.0;
    bool contact = false;
  };

  enum class Ctrl { Torque, Joint };

  Metrics simulate(const pinocchio::Model &m, pinocchio::FrameIndex f, Ctrl ctrl,
                   const jtorque_cbf::JointTorqueCBFConfig &jcfg, double z0, double v0, double push)
  {
    pinocchio::Data d(m);
    Eigen::VectorXd q(7);
    q << 0, 0, 0, -M_PI / 2, 0, M_PI / 2, -M_PI / 4;
    Kin k = kinematics(m, d, f, q, Eigen::VectorXd::Zero(7));
    Eigen::Matrix<double, 6, 1> V0;
    V0 << 0, 0, -v0, 0, 0, 0;
    Eigen::VectorXd dq = k.J.completeOrthogonalDecomposition().solve(V0);
    cbf::Hyperplane p;
    p.normal << 0, 0, 1;
    p.offset = k.x - Eigen::Vector3d(0, 0, z0);

    torque_cbf::TorqueCBFController tc(torqueConfig(), p);
    jtorque_cbf::JointTorqueCBFController jc(jcfg, p);

    Eigen::VectorXd tau_app = Eigen::VectorXd::Zero(7);
    Metrics r;
    const int release_step = 1000;
    for (int step = 0; step < 2000; ++step)
    {
      k = kinematics(m, d, f, q, dq);
      Eigen::Matrix<double, 6, 1> Fh = Eigen::Matrix<double, 6, 1>::Zero();
      Fh(2) = (step < release_step) ? -push : 0.0;

      Eigen::VectorXd tau_cmd;
      if (ctrl == Ctrl::Torque)
      {
        torque_cbf::RobotState s;
        s.q = q; s.dq = dq; s.x = k.x; s.dx = k.V.head<3>(); s.M = k.M; s.C = k.C; s.g = k.g;
        s.J = k.J; s.dJ = k.dJ; s.F_ext = Fh;
        torque_cbf::TorqueCBFResult res;
        tc.computeControlTorque(s, Eigen::VectorXd(), &res);
        tau_cmd = res.tau_cbf;
      }
      else
      {
        jtorque_cbf::RobotState s;
        s.q = q; s.dq = dq; s.x = k.x; s.dx = k.V.head<3>(); s.M = k.M; s.C = k.C; s.g = k.g;
        s.J = k.J; s.dJ = k.dJ; s.F_ext = Fh;
        jtorque_cbf::JointTorqueCBFResult res;
        jc.computeControlTorque(s, Eigen::VectorXd(), &res);
        tau_cmd = res.tau_cbf;
        r.sigma_n = std::max(r.sigma_n, res.sigma_normal);
        r.sigma_p = std::max(r.sigma_p, res.sigma_power);
      }

      // Robot: per-joint rate limiter (gravity is added by the robot itself)
      const double dmax = kRobotRate * kDt;
      tau_app += (tau_cmd - tau_app).cwiseMax(-dmax).cwiseMin(dmax);
      r.mismatch = std::max(r.mismatch, (tau_cmd - tau_app).cwiseAbs().maxCoeff());

      // Applied end-effector force and CBF power
      const Eigen::MatrixXd JMinv = k.M.ldlt().solve(k.J.transpose()).transpose();
      const Eigen::Matrix<double, 6, 6> Lam = (k.J * JMinv.transpose()).inverse();
      const Eigen::Matrix<double, 6, 1> F = Lam * JMinv * tau_app;
      r.lateral = std::max(r.lateral, F.head<2>().cwiseAbs().maxCoeff());
      r.power_pos = std::max(r.power_pos, tau_app.dot(dq));

      const double zeta = k.x(2) - p.offset(2), dzeta = k.V(2);
      r.min_zeta = std::min(r.min_zeta, zeta);
      if (!r.contact && zeta <= 0.0)
      {
        r.contact = true;
        r.v_contact = dzeta;
      }
      // Pushback: normal acceleration of the applied CBF torque while moving away from the plane
      const double a_n_cbf = (JMinv * tau_app)(2);
      if (dzeta > 0.0 && a_n_cbf > 0.0)
      {
        (step < release_step ? r.push_stop : r.push_release) += a_n_cbf * kDt;
      }

      for (int sub = 0; sub < 10; ++sub)
      {
        const Kin ks = kinematics(m, d, f, q, dq);
        const Eigen::VectorXd ddq =
            ks.M.ldlt().solve(tau_app + ks.J.transpose() * Fh - ks.C * dq - kFriction * dq);
        dq += ddq * (kDt / 10.0);
        q += dq * (kDt / 10.0);
      }
    }
    return r;
  }

  void scenario(const pinocchio::Model &m, pinocchio::FrameIndex f, const char *title,
                double z0, double v0, double push)
  {
    std::printf("%s\n", title);
    std::printf("  %-34s | %10s %9s %11s %11s %9s %8s %8s %8s %8s\n", "controller", "v@contact", "min zeta",
                "push@stop", "push@rel", "|cmd-app|", "lateral", "P>0 max", "sig_n", "sig_P");
    struct Row { const char *name; Ctrl c; jtorque_cbf::JointTorqueCBFConfig cfg; };
    const Row rows[] = {
        {"torque_cbf (normal force)", Ctrl::Torque, jointConfig(0, 0, 0)},
        {"jtorque, no rate model, eps=0", Ctrl::Joint, jointConfig(0.0, 0.0, 0.0)},
        {"jtorque, rate 999, eps=0", Ctrl::Joint, jointConfig(kRobotRate, 0.0, 0.0)},
        {"jtorque, rate 999, eps=5N/0.5Nm", Ctrl::Joint, jointConfig(kRobotRate, 5.0, 0.5)},
        {"jtorque, rate 999, eps=20N/2Nm", Ctrl::Joint, jointConfig(kRobotRate, 20.0, 2.0)},
    };
    for (const Row &row : rows)
    {
      const Metrics r = simulate(m, f, row.c, row.cfg, z0, v0, push);
      std::printf("  %-34s | %8.4fm/s %8.4fm %9.4fm/s %9.4fm/s %7.2fNm %6.2fN %7.3fW %8.1e %8.1e\n", row.name,
                  r.contact ? r.v_contact : 0.0, r.min_zeta, r.push_stop, r.push_release, r.mismatch,
                  r.lateral, r.power_pos, r.sigma_n, r.sigma_p);
    }
    std::printf("\n");
  }
} // namespace

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: %s <path/to/Franka_arm.urdf> [tip_frame=panda_link8]\n", argv[0]);
    return 1;
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(argv[1], model);
  pinocchio::Data data(model);
  const pinocchio::FrameIndex f = model.getFrameId(argc > 2 ? argv[2] : "panda_link8");

  equivalence(model, data, f);
  std::printf("2) Closed loop with a %.0f Nm/s per-joint torque-rate limiter\n", kRobotRate);
  std::printf("   v@contact: normal velocity when reaching the plane (target v_ths = -0.003 m/s)\n");
  std::printf("   push@stop / push@rel: velocity the CBF torque adds away from the plane (before / after release)\n");
  std::printf("   |cmd-app|: commanded vs applied torque; lateral: applied lateral EE force; P>0: CBF power\n\n");
  scenario(model, f, "a) normal approach: 25 cm above the plane at 0.4 m/s", 0.25, 0.4, 0.0);
  scenario(model, f, "b) operator push 15 N toward the plane, released at t = 1 s", 0.25, 0.4, 15.0);
  scenario(model, f, "c) late fast entry: 3 cm above the plane at 0.8 m/s", 0.03, 0.8, 0.0);
  return 0;
}
