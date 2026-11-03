#include "damping_cbf/damping_cbf_controller.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace damping_cbf;

namespace {

struct SimTracePoint {
  double t;
  double z;
  double zeta;
  double dzeta;
  double v_min;
  double B;
  double d_cbf;
  double f_imp;
};

// Simulate 1kHz task-space approach toward the horizontal hyperplane at z = 0.1m
std::vector<SimTracePoint> runApproachSimulation(
    DampingCBFController& ctrl,
    double z_init,
    double dz_init,
    double rho_eff,
    double f_ext_z,
    double duration_s = 1.2,
    double dt = 0.001) {
  ctrl.resetEntryState();
  std::vector<SimTracePoint> trace;
  const int steps = static_cast<int>(duration_s / dt);

  Eigen::Vector3d x(0.4, 0.0, z_init);
  Eigen::Vector3d dx(0.0, 0.0, dz_init);

  for (int k = 0; k <= steps; ++k) {
    const double t = k * dt;
    const double zeta = ctrl.computeDistance(x);
    const double dzeta = ctrl.computeNormalVelocity(dx);
    const double a_drift = ctrl.getConfig().compensate_external_force ? (f_ext_z / rho_eff) : 0.0;

    CBFStepResult res = ctrl.solveDampingQP(zeta, dzeta, rho_eff, a_drift);

    trace.push_back({t, x.z(), zeta, dzeta, res.v_min, res.barrier_B,
                     res.d_cbf, res.f_imp_scalar});

    // Rigid contact stop if surface is reached
    if (zeta <= 0.0 && dzeta < 0.0) {
      dx.z() = 0.0;
      x.z() = ctrl.getHyperplane().offset.z();
      continue;
    }

    // Forward Euler integration of 1kHz dynamics: rho * ddzeta = -d_cbf * dzeta + f_ext_z
    const double ddz = (-res.f_imp_scalar + f_ext_z) / rho_eff;
    dx.z() += ddz * dt;
    x.z() += dx.z() * dt;
  }
  return trace;
}

}  // namespace

int main() {
  std::cout << "=====================================================================\n";
  std::cout << "  CBF Variable-Damping Compliance Controller - Verification Suite\n";
  std::cout << "=====================================================================\n\n";

  // 1. Define horizontal contact surface at z = 0.10 m with upward normal [0, 0, 1]^T
  Hyperplane plane;
  plane.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  plane.offset = Eigen::Vector3d(0.0, 0.0, 0.10);

  ControllerConfig cfg;
  cfg.p_ths = 0.20;      // Activate 20 cm above surface (z <= 0.30 m)
  cfg.v_ths = -0.002;    // Target touchdown velocity: -2 mm/s
  cfg.v_0 = -0.35;       // Entry speed envelope at zeta = p_ths: -0.35 m/s
  cfg.gamma = 20.0;      // CBF convergence rate [1/s]
  cfg.d_max = 250.0;     // Max damping coefficient [Ns/m]

  const double rho_eff = 3.5;  // Effective Cartesian mass along z [kg]
  const double z_start = 0.30; // Start right at entry boundary zeta = 0.20 m

  // Determine examples/csv directory
  fs::path examples_dir = fs::path(__FILE__).parent_path();
  fs::path csv_dir = examples_dir / "csv";
  if (!fs::exists(examples_dir)) {
    if (fs::exists("examples")) {
      csv_dir = fs::path("examples") / "csv";
    } else {
      csv_dir = fs::path("csv");
    }
  }
  fs::create_directories(csv_dir);

  // -------------------------------------------------------------------------
  // Experiment 1: Compare Force Profiles h_1 ... h_5
  // -------------------------------------------------------------------------
  std::cout << "[Experiment 1] Comparing Profiles h_1..h_5 (entry dz = -0.35 m/s, F_ext = 0 N)\n";
  std::cout << std::left << std::setw(18) << "Profile"
            << std::setw(16) << "Min zeta [mm]"
            << std::setw(20) << "Vel at z->0 [mm/s]"
            << std::setw(18) << "Peak |f_IMP| [N]"
            << "Peak d_CBF [Ns/m]\n";
  std::cout << std::string(86, '-') << "\n";

  const std::vector<std::pair<std::string, ProfileType>> profiles = {
      {"h1 (Constant)", ProfileType::CONSTANT},
      {"h2 (Root x^1/5)", ProfileType::ROOT_FIFTH},
      {"h3 (Linear x)", ProfileType::LINEAR},
      {"h4 (Quintic x^5)", ProfileType::QUINTIC},
      {"h5 (Smoothstep)", ProfileType::SMOOTHSTEP}};

  std::ofstream csv_profiles(csv_dir / "sim_profiles.csv");
  csv_profiles << "profile,t,z,zeta,dzeta,v_min,B,d_cbf,f_imp\n";

  for (const auto& [name, ptype] : profiles) {
    cfg.profile = ptype;
    DampingCBFController ctrl(cfg, plane);
    auto trace = runApproachSimulation(ctrl, z_start, -0.35, rho_eff, 0.0, 3.5);

    double peak_force = 0.0;
    double peak_d = 0.0;
    double min_zeta = 1.0;
    double terminal_vel = 0.0;

    for (const auto& pt : trace) {
      csv_profiles << "\"" << name << "\"," << pt.t << "," << pt.z << ","
                   << pt.zeta << "," << pt.dzeta << "," << pt.v_min << ","
                   << pt.B << "," << pt.d_cbf << "," << pt.f_imp << "\n";
      peak_force = std::max(peak_force, std::abs(pt.f_imp));
      peak_d = std::max(peak_d, pt.d_cbf);
      if (pt.zeta < min_zeta) {
        min_zeta = pt.zeta;
        terminal_vel = pt.dzeta;
      }
    }

    std::cout << std::left << std::setw(18) << name
              << std::fixed << std::setprecision(2)
              << std::setw(16) << (min_zeta * 1e3)
              << std::setw(20) << (terminal_vel * 1e3)
              << std::setw(18) << peak_force
              << peak_d << "\n";
  }
  csv_profiles.close();

  // -------------------------------------------------------------------------
  // Experiment 2: Free Motion Inside Safe Set (B > 0) vs. Envelope Recovery (B < 0)
  // -------------------------------------------------------------------------
  std::cout << "\n[Experiment 2] CBF Behavior Across Entry Velocities (Profile h5)\n";
  cfg.profile = ProfileType::SMOOTHSTEP;
  DampingCBFController ctrl_h5(cfg, plane);

  std::ofstream csv_entry(csv_dir / "sim_entry_speeds.csv");
  csv_entry << "case,t,z,zeta,dzeta,v_min,B,d_cbf,f_imp\n";

  const std::vector<std::pair<std::string, double>> entry_cases = {
      {"Slow (B > 0, Inside Safe Zone)", -0.15},
      {"Nominal Border (B = 0)", -0.35},
      {"Fast (B < 0, Outside Safe Zone)", -0.50}};

  for (const auto& [label, v_init] : entry_cases) {
    auto trace = runApproachSimulation(ctrl_h5, z_start, v_init, rho_eff, 0.0, 2.0);
    double activation_zeta = -1.0;
    for (const auto& pt : trace) {
      csv_entry << "\"" << label << "\"," << pt.t << "," << pt.z << ","
                << pt.zeta << "," << pt.dzeta << "," << pt.v_min << ","
                << pt.B << "," << pt.d_cbf << "," << pt.f_imp << "\n";
      if (activation_zeta < 0.0 && pt.d_cbf > 0.1) {
        activation_zeta = pt.zeta;
      }
    }
    std::cout << "  - " << std::left << std::setw(34) << label
              << " | v_init = " << std::setw(6) << v_init << " m/s"
              << " | Damping engages at zeta = " << (activation_zeta * 100.0) << " cm\n";
  }
  csv_entry.close();

  // -------------------------------------------------------------------------
  // Experiment 3: Active Human Downward Push (F_ext,z = -15 N)
  // -------------------------------------------------------------------------
  std::cout << "\n[Experiment 3] Downward Operator Push F_ext,z = -15 N (With vs. Without Drift Term)\n";
  std::ofstream csv_push(csv_dir / "sim_external_push.csv");
  csv_push << "mode,t,z,zeta,dzeta,v_min,B,d_cbf,f_imp\n";

  for (bool comp : {false, true}) {
    cfg.compensate_external_force = comp;
    DampingCBFController ctrl_push(cfg, plane);
    auto trace = runApproachSimulation(ctrl_push, z_start, -0.35, rho_eff, -15.0, 2.0);
    std::string mode = comp ? "With CBF Drift Term (a_drift)" : "Unforced Damping Only";
    double impact_vel = trace.back().dzeta;
    for (const auto& pt : trace) {
      csv_push << "\"" << mode << "\"," << pt.t << "," << pt.z << ","
               << pt.zeta << "," << pt.dzeta << "," << pt.v_min << ","
               << pt.B << "," << pt.d_cbf << "," << pt.f_imp << "\n";
      if (pt.zeta > 0.001) {
        impact_vel = pt.dzeta;
      }
    }
    std::cout << "  - " << std::left << std::setw(32) << mode
              << " | Velocity at zeta = 1 mm: " << (impact_vel * 1e3) << " mm/s\n";
  }
  csv_push.close();

  // -------------------------------------------------------------------------
  // Experiment 4: Full 7-DoF Joint-Space Compliance Control Torque Evaluation
  // -------------------------------------------------------------------------
  std::cout << "\n[Experiment 4] 7-DoF Manipulator Compliance Torque & Multi-Surface QP Check\n";
  RobotState rstate;
  const int n_dof = 7;
  rstate.q = Eigen::VectorXd::Zero(n_dof);
  rstate.dq = Eigen::VectorXd::Constant(n_dof, -0.08);
  rstate.x = Eigen::Vector3d(0.45, 0.0, 0.18);   // zeta = 0.08 m above surface
  rstate.phi = Eigen::Vector3d::Zero();
  rstate.dx = Eigen::Vector3d(0.0, 0.0, -0.28);  // approaching at -0.28 m/s
  rstate.dphi = Eigen::Vector3d::Zero();
  rstate.M = 2.5 * Eigen::MatrixXd::Identity(n_dof, n_dof);
  rstate.C = 0.1 * Eigen::MatrixXd::Identity(n_dof, n_dof);
  rstate.g = Eigen::VectorXd::Constant(n_dof, 5.0);
  rstate.J = Eigen::MatrixXd::Zero(6, n_dof);
  rstate.J.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
  rstate.J(2, 6) = 0.5;
  rstate.dJ = Eigen::MatrixXd::Zero(6, n_dof);
  rstate.F_ext << 0.0, 0.0, -10.0, 0.0, 0.0, 0.0;

  cfg.compensate_external_force = true;
  DampingCBFController full_ctrl(cfg, plane);
  CBFStepResult step_res;
  Eigen::VectorXd tau = full_ctrl.computeControlTorque(
      rstate, rstate.x, rstate.phi, Eigen::VectorXd::Zero(n_dof), &step_res);

  std::cout << "  - Distance zeta      : " << step_res.zeta << " m\n"
            << "  - Velocity dzeta     : " << step_res.dzeta << " m/s (v_min = " << step_res.v_min << " m/s)\n"
            << "  - 2D QP z*=[d, eps]  : [" << step_res.z_opt(0) << ", " << step_res.z_opt(1) << "]\n"
            << "  - Optimal d_CBF      : " << step_res.d_cbf << " Ns/m\n"
            << "  - Damping force F_z  : " << step_res.F_imp.z() << " N\n"
            << "  - Joint torque ||tau||: " << tau.norm() << " Nm\n";

  // -------------------------------------------------------------------------
  // Experiment 5: 2D Multi-Surface Corner QP via Goldfarb-Idnani (solve_quadprog)
  // -------------------------------------------------------------------------
  Hyperplane plane_wall;
  plane_wall.normal = Eigen::Vector3d(0.70710678, 0.0, 0.70710678);  // 45-deg inclined surface
  plane_wall.offset = Eigen::Vector3d(0.35, 0.0, 0.10);
  std::vector<Hyperplane> two_planes = {plane, plane_wall};

  Eigen::VectorXd d_2d;
  const Eigen::Matrix<double, 6, 6> Lambda_test =
      DampingCBFController::computeCartesianInertia(rstate.M, rstate.J);
  const Eigen::Matrix3d D_imp_2d = full_ctrl.solveMultiSurfaceDampingQP(
      two_planes, rstate.x, rstate.dx, Lambda_test, rstate.F_ext, &d_2d);

  std::cout << "  - 2-Surface QP d*    : [" << d_2d(0) << ", " << d_2d(1) << "] Ns/m"
            << " (trace(D_IMP) = " << D_imp_2d.trace() << " Ns/m)\n";

  // Simulate 1 kHz closed-loop 7-DoF manipulator trajectory approaching the 2-wall corner
  std::ofstream csv_7dof(csv_dir / "sim_manipulator_7dof.csv");
  csv_7dof << "t,x,z,zeta1,dzeta1,v_min1,zeta2,dzeta2,v_min2,"
           << "d_single,d1_multi,d2_multi,trace_D_imp,"
           << "tau1,tau2,tau3,tau4,tau5,tau6,tau7,tau_cbf3,tau_cbf7,tau_norm\n";

  RobotState sim_state = rstate;
  sim_state.q = Eigen::VectorXd::Zero(n_dof);
  // Start inside both braking zones: x0 = (0.45, 0.0, 0.24) m -> zeta1 = 0.14 m, zeta2 = 0.17 m
  // Set initial joint velocities so dx = J_lin * dq = [-0.22, 0.0, -0.28]^T m/s
  sim_state.dq = Eigen::VectorXd::Zero(n_dof);
  sim_state.dq(0) = -0.22;  // x velocity toward 45-deg wall [m/s]
  sim_state.dq(2) = -0.224; // primary z joint velocity [rad/s]
  sim_state.dq(6) = -0.112; // redundant z joint velocity (J(2,6)=0.5 -> dz = -0.224 + 0.5*(-0.112) = -0.28 m/s)
  const Eigen::Vector3d x0(0.45, 0.0, 0.24);
  sim_state.x = x0;
  sim_state.phi = Eigen::Vector3d::Zero();
  sim_state.dx = (sim_state.J * sim_state.dq).head<3>();
  sim_state.dphi = (sim_state.J * sim_state.dq).tail<3>();
  // Gentle downward/leftward operator push F_ext = [-2, 0, -3, 0, 0, 0]^T N into the corner
  sim_state.F_ext << -2.0, 0.0, -3.0, 0.0, 0.0, 0.0;

  const Eigen::Vector3d x_ref = x0;
  const Eigen::Vector3d phi_ref = Eigen::Vector3d::Zero();
  const Eigen::VectorXd tau_null = Eigen::VectorXd::Zero(n_dof);

  DampingCBFController sim_ctrl(cfg, plane);
  const double dt = 0.001;
  const int steps_7dof = 1500;

  for (int k = 0; k <= steps_7dof; ++k) {
    const double t = k * dt;

    // 1) Evaluate single-surface 7-DoF compliance control torque
    CBFStepResult sres;
    const Eigen::VectorXd tau_single = sim_ctrl.computeControlTorque(
        sim_state, x_ref, phi_ref, tau_null, &sres);

    // 2) Evaluate multi-surface 2-wall corner QP damping
    Eigen::VectorXd d_multi(2);
    const Eigen::Matrix3d D_imp_corner = sim_ctrl.solveMultiSurfaceDampingQP(
        two_planes, sim_state.x, sim_state.dx, Lambda_test, sim_state.F_ext, &d_multi);

    // Compute distances and envelopes for both planes
    const Eigen::Vector3d n1 = two_planes[0].normal.normalized();
    const Eigen::Vector3d n2 = two_planes[1].normal.normalized();
    const double zeta1 = n1.dot(sim_state.x - two_planes[0].offset);
    const double dzeta1 = n1.dot(sim_state.dx);
    const double v_min1 = sim_ctrl.computeVelocityEnvelope(zeta1);
    const double zeta2 = n2.dot(sim_state.x - two_planes[1].offset);
    const double dzeta2 = n2.dot(sim_state.dx);
    const double v_min2 = sim_ctrl.computeVelocityEnvelope(zeta2);

    // Assemble full 7-DoF torque using the 2-surface corner D_IMP matrix
    const Eigen::Matrix<double, 6, 6> Gamma =
        DampingCBFController::computeCartesianCoriolis(
            sim_state.M, sim_state.C, sim_state.J, sim_state.dJ, Lambda_test);
    Eigen::Matrix<double, 6, 1> X_dot;
    X_dot.head<3>() = sim_state.dx;
    X_dot.tail<3>() = sim_state.dphi;
    Eigen::Matrix<double, 6, 1> X_tilde;
    X_tilde.head<3>() = sim_state.x - x_ref;
    X_tilde.tail<3>() = sim_state.phi - phi_ref;

    Eigen::Matrix<double, 6, 6> D_imp_6 = Eigen::Matrix<double, 6, 6>::Zero();
    D_imp_6.topLeftCorner<3, 3>() = D_imp_corner;

    const Eigen::VectorXd coriolis_comp =
        sim_state.J.transpose() * Lambda_test *
        (sim_state.J * sim_state.M.ldlt().solve(sim_state.C) - sim_state.dJ) * sim_state.dq;
    const Eigen::VectorXd tau_cbf_vec =
        -sim_state.J.transpose() * D_imp_6 * X_dot;
    const Eigen::VectorXd tau_total =
        sim_state.g + tau_null + coriolis_comp +
        sim_state.J.transpose() * (-(Gamma + cfg.D) * X_dot - cfg.K * X_tilde) +
        tau_cbf_vec;

    csv_7dof << t << "," << sim_state.x.x() << "," << sim_state.x.z() << ","
             << zeta1 << "," << dzeta1 << "," << v_min1 << ","
             << zeta2 << "," << dzeta2 << "," << v_min2 << ","
             << sres.d_cbf << "," << d_multi(0) << "," << d_multi(1) << ","
             << D_imp_corner.trace() << ","
             << tau_total(0) << "," << tau_total(1) << "," << tau_total(2) << ","
             << tau_total(3) << "," << tau_total(4) << "," << tau_total(5) << ","
             << tau_total(6) << "," << tau_cbf_vec(2) << "," << tau_cbf_vec(6) << ","
             << tau_total.norm() << "\n";

    // Rigid contact stop when either surface is reached
    if ((zeta1 <= 0.0 && dzeta1 < 0.0) || (zeta2 <= 0.0 && dzeta2 < 0.0)) {
      sim_state.dq.setZero();
      sim_state.dx.setZero();
      sim_state.dphi.setZero();
      continue;
    }

    // Forward-Euler integration of 7-DoF manipulator dynamics:
    // M * ddq + C * dq + g = tau_total + J^T * F_ext
    const Eigen::VectorXd ddq = sim_state.M.ldlt().solve(
        tau_total - sim_state.C * sim_state.dq - sim_state.g +
        sim_state.J.transpose() * sim_state.F_ext);
    sim_state.dq += ddq * dt;
    sim_state.q += sim_state.dq * dt;
    sim_state.x = x0 + (sim_state.J * sim_state.q).head<3>();
    sim_state.phi = (sim_state.J * sim_state.q).tail<3>();
    sim_state.dx = (sim_state.J * sim_state.dq).head<3>();
    sim_state.dphi = (sim_state.J * sim_state.dq).tail<3>();
  }
  csv_7dof.close();

  std::cout << "\nAll simulations completed via solve_quadprog. CSV logs written to " << csv_dir << "\n";
  return 0;
}
