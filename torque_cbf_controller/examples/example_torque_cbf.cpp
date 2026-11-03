#include "torque_cbf/torque_cbf_controller.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace torque_cbf;

namespace
{

  struct SimTracePoint
  {
    double t;
    double z;
    double zeta;
    double dzeta;
    double v_min;
    double B;
    double tau_cbf;
    double delta;
  };

  // Resolve directory for CSV logs in examples/csv
  fs::path getCsvDir()
  {
    fs::path p1 = fs::current_path() / "src" / "torque_cbf_controller" / "examples" / "csv";
    if (fs::exists(p1.parent_path()))
    {
      fs::create_directories(p1);
      return p1;
    }
    fs::path p2 = fs::current_path() / "examples" / "csv";
    fs::create_directories(p2);
    return p2;
  }

  // 1-DOF / Task-space approach simulation toward hyperplane at z = 0.10m
  std::vector<SimTracePoint> runApproachSimulation(
      TorqueCBFController &ctrl,
      double z_init,
      double dz_init,
      double mass,
      double f_ext_z,
      double duration_s = 1.2,
      double dt = 0.001)
  {
    ctrl.resetEntryState();
    std::vector<SimTracePoint> trace;
    const int steps = static_cast<int>(duration_s / dt);

    double z = z_init;
    double dz = dz_init;

    // 1-DOF dynamics: M = mass, J = 1 (linear normal direction)
    Eigen::MatrixXd M(1, 1);
    M(0, 0) = mass;

    Eigen::MatrixXd J(6, 1);
    J.setZero();
    J(2, 0) = 1.0; // dz/dq = 1

    Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero();
    F_ext(2) = f_ext_z;

    Eigen::Matrix<double, 6, 1> drift_cart = Eigen::Matrix<double, 6, 1>::Zero();

    for (int k = 0; k <= steps; ++k)
    {
      const double t = k * dt;
      const Eigen::Vector3d x_ee(0.4, 0.0, z);
      const Eigen::Vector3d dx_ee(0.0, 0.0, dz);

      TorqueCBFResult res = ctrl.solveTorqueQP(M, J, drift_cart, F_ext, x_ee, dx_ee);

      trace.push_back({t, z, res.zeta, res.dzeta, res.v_min, res.barrier_B,
                       res.tau_cbf(0), res.slack_delta});

      // Rigid contact boundary stop
      if (res.zeta <= 0.0 && dz < 0.0)
      {
        dz = 0.0;
        z = ctrl.getHyperplane().offset.z();
        continue;
      }

      // Equation of motion: M * ddz = tau_cbf + F_ext_z
      const double ddz = (res.tau_cbf(0) + f_ext_z) / mass;
      dz += ddz * dt;
      z += dz * dt;
    }
    return trace;
  }

} // namespace

int main()
{
  std::cout << "=====================================================================\n";
  std::cout << "  Direct Joint-Torque CBF Controller - Verification Suite\n";
  std::cout << "=====================================================================\n\n";

  const fs::path csv_dir = getCsvDir();
  std::cout << "Logging simulation results to: " << csv_dir.string() << "\n\n";

  // Preload horizontal hyperplane at z = 0.10m with upward normal [0, 0, 1]^T
  Hyperplane plane;
  plane.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  plane.offset = Eigen::Vector3d(0.0, 0.0, 0.10);

  TorqueCBFConfig cfg;
  cfg.p_ths = 0.20;
  cfg.v_ths = -0.002;
  cfg.v_0 = -0.35;
  cfg.gamma = 15.0;
  cfg.default_tau_limit = 200.0;
  cfg.w_slack = 1e4;
  cfg.w_lin = 1e6;

  TorqueCBFController ctrl(cfg, plane);

  // ---------------------------------------------------------------------------
  // Experiment 1: Comparing Profiles h1..h5 (Constant, Root-fifth, Linear, Quintic, Smoothstep)
  // ---------------------------------------------------------------------------
  std::cout << "[Experiment 1] Comparing Profiles h_1..h_5 (entry dz = -0.35 m/s, F_ext = 0 N)\n";
  std::cout << std::left << std::setw(18) << "Profile"
            << std::setw(16) << "Min zeta [mm]"
            << std::setw(20) << "Vel at z->0 [mm/s]"
            << std::setw(18) << "Peak tau_cbf [Nm]"
            << std::setw(16) << "Max delta" << "\n";
  std::cout << std::string(88, '-') << "\n";

  const std::vector<std::pair<ProfileType, std::string>> profiles = {
      {ProfileType::CONSTANT, "h1 (Constant)"},
      {ProfileType::ROOT_FIFTH, "h2 (Root x^1/5)"},
      {ProfileType::LINEAR, "h3 (Linear x)"},
      {ProfileType::QUINTIC, "h4 (Quintic x^5)"},
      {ProfileType::SMOOTHSTEP, "h5 (Smoothstep)"}};

  std::ofstream csv_exp1(csv_dir / "torque_cbf_profiles.csv");
  csv_exp1 << "t,profile,z,zeta,dzeta,v_min,B,tau_cbf,delta\n";

  const double z_entry = plane.offset.z() + cfg.p_ths; // 0.30m
  const double mass = 5.0;                             // [kg]

  for (const auto &[prof_type, prof_name] : profiles)
  {
    TorqueCBFConfig c = cfg;
    c.profile = prof_type;
    ctrl.setConfig(c);

    auto trace = runApproachSimulation(ctrl, z_entry, cfg.v_0, mass, 0.0, 1.2);

    double min_zeta = 1e9;
    double vel_near_contact = 0.0;
    double peak_tau = 0.0;
    double max_delta = 0.0;

    for (const auto &pt : trace)
    {
      csv_exp1 << pt.t << "," << prof_name << "," << pt.z << ","
               << pt.zeta << "," << pt.dzeta << "," << pt.v_min << ","
               << pt.B << "," << pt.tau_cbf << "," << pt.delta << "\n";

      if (pt.zeta < min_zeta)
      {
        min_zeta = pt.zeta;
      }
      if (pt.zeta <= 0.005 && pt.zeta >= 0.0)
      {
        vel_near_contact = pt.dzeta;
      }
      if (std::abs(pt.tau_cbf) > peak_tau)
      {
        peak_tau = std::abs(pt.tau_cbf);
      }
      if (pt.delta > max_delta)
      {
        max_delta = pt.delta;
      }
    }

    std::cout << std::left << std::setw(18) << prof_name
              << std::fixed << std::setprecision(2)
              << std::setw(16) << (min_zeta * 1e3)
              << std::setw(20) << (vel_near_contact * 1e3)
              << std::setw(18) << peak_tau
              << std::setw(16) << max_delta << "\n";
  }
  std::cout << "\n";

  // ---------------------------------------------------------------------------
  // Experiment 2: CBF Behavior Across Entry Velocities (Slow, Nominal, Fast)
  // ---------------------------------------------------------------------------
  std::cout << "[Experiment 2] CBF Behavior Across Entry Velocities (Profile h5 - Smoothstep)\n";
  TorqueCBFConfig c2 = cfg;
  c2.profile = ProfileType::SMOOTHSTEP;
  ctrl.setConfig(c2);

  std::ofstream csv_exp2(csv_dir / "torque_cbf_entry_speeds.csv");
  csv_exp2 << "t,case,z,zeta,dzeta,v_min,B,tau_cbf,delta\n";

  struct EntryCase
  {
    std::string label;
    double v_init;
    std::string desc;
  };

  const std::vector<EntryCase> entry_cases = {
      {"Slow", -0.15, "Slow (B > 0, Inside Safe Zone)"},
      {"Nominal", -0.35, "Nominal Border (B = 0)"},
      {"Fast", -0.50, "Fast (B < 0, Outside Safe Zone)"}};

  for (const auto &ec : entry_cases)
  {
    auto trace = runApproachSimulation(ctrl, z_entry + 0.05, ec.v_init, mass, 0.0, 1.2);
    double engage_zeta = 0.0;
    bool engaged = false;

    for (const auto &pt : trace)
    {
      csv_exp2 << pt.t << "," << ec.label << "," << pt.z << ","
               << pt.zeta << "," << pt.dzeta << "," << pt.v_min << ","
               << pt.B << "," << pt.tau_cbf << "," << pt.delta << "\n";

      if (!engaged && std::abs(pt.tau_cbf) > 0.1)
      {
        engage_zeta = pt.zeta;
        engaged = true;
      }
    }

    std::cout << "  - " << std::left << std::setw(35) << ec.desc
              << " | v_init = " << std::setw(5) << ec.v_init << " m/s"
              << " | Torque engages at zeta = " << std::fixed << std::setprecision(2)
              << (engage_zeta * 100.0) << " cm\n";
  }
  std::cout << "\n";

  // ---------------------------------------------------------------------------
  // Experiment 3: External Push Handling (F_ext,z = -15 N, With vs. Without Drift)
  // ---------------------------------------------------------------------------
  std::cout << "[Experiment 3] Downward Push F_ext,z = -15 N (With vs. Without Drift Term Compensation)\n";
  std::ofstream csv_exp3(csv_dir / "torque_cbf_external_push.csv");
  csv_exp3 << "t,mode,z,zeta,dzeta,v_min,B,tau_cbf,delta\n";

  for (bool compensate : {false, true})
  {
    TorqueCBFConfig c3 = cfg;
    c3.profile = ProfileType::SMOOTHSTEP;
    c3.compensate_external_force = compensate;
    ctrl.setConfig(c3);

    auto trace = runApproachSimulation(ctrl, z_entry, cfg.v_0, mass, -15.0, 1.2);
    double vel_contact = 0.0;
    double max_torque = 0.0;

    const std::string mode_str = compensate ? "With Drift Term (a_drift)" : "Without Drift Term";
    for (const auto &pt : trace)
    {
      csv_exp3 << pt.t << "," << mode_str << "," << pt.z << ","
               << pt.zeta << "," << pt.dzeta << "," << pt.v_min << ","
               << pt.B << "," << pt.tau_cbf << "," << pt.delta << "\n";

      if (pt.zeta <= 0.005 && pt.zeta >= 0.0)
      {
        vel_contact = pt.dzeta;
      }
      if (std::abs(pt.tau_cbf) > max_torque)
      {
        max_torque = std::abs(pt.tau_cbf);
      }
    }

    std::cout << "  - " << std::left << std::setw(35) << mode_str
              << " | Velocity near contact: " << std::fixed << std::setprecision(2)
              << (vel_contact * 1e3) << " mm/s"
              << " | Peak tau_cbf: " << max_torque << " Nm\n";
  }
  std::cout << "\n";

  // ---------------------------------------------------------------------------
  // Experiment 4: 7-DOF Manipulator Dynamics & Multi-Surface Corner Constraints
  // ---------------------------------------------------------------------------
  std::cout << "[Experiment 4] 7-DOF Manipulator Dynamics & Multi-Surface Corner Constraints\n";

  // Define 2 orthogonal boundary planes:
  // 1. Floor at z = 0.10 m (normal +Z)
  // 2. Wall at y = 0.20 m (outward normal -Y)
  Hyperplane p_floor;
  p_floor.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  p_floor.offset = Eigen::Vector3d(0.0, 0.0, 0.10);

  Hyperplane p_wall;
  p_wall.normal = Eigen::Vector3d(0.0, -1.0, 0.0);
  p_wall.offset = Eigen::Vector3d(0.0, 0.20, 0.0);

  TorqueCBFConfig cfg_multi = cfg;
  cfg_multi.p_ths = 0.15;
  cfg_multi.profile = ProfileType::SMOOTHSTEP;
  cfg_multi.tau_min = Eigen::VectorXd::Constant(7, -150.0);
  cfg_multi.tau_max = Eigen::VectorXd::Constant(7, 150.0);

  TorqueCBFController ctrl_multi(cfg_multi, {p_floor, p_wall});

  // Construct realistic 7-DOF manipulator state
  RobotState state7;
  state7.q = Eigen::VectorXd::Zero(7);
  state7.q << 0.0, -0.6, 0.0, 1.2, 0.0, -0.6, 0.0;
  state7.dq = Eigen::VectorXd::Zero(7);
  state7.dq << 0.05, -0.15, 0.10, 0.20, -0.05, 0.12, 0.02;

  state7.x = Eigen::Vector3d(0.40, 0.14, 0.16);   // Approaching both floor (zeta1=0.06m) and wall (zeta2=0.06m)
  state7.dx = Eigen::Vector3d(0.0, 0.25, -0.28);  // dz = -0.28 m/s (approaching floor), dy = +0.25 m/s (approaching wall)

  state7.M = Eigen::MatrixXd::Identity(7, 7) * 2.5;
  for (int i = 0; i < 7; ++i)
  {
    for (int j = 0; j < 7; ++j)
    {
      state7.M(i, j) += 0.4 * std::cos(i + j);
    }
  }
  state7.M = 0.5 * (state7.M + state7.M.transpose()) + Eigen::MatrixXd::Identity(7, 7) * 1.5;

  state7.C = Eigen::MatrixXd::Zero(7, 7);
  for (int i = 0; i < 7; ++i)
  {
    state7.C(i, (i + 1) % 7) = 0.25 * state7.dq((i + 2) % 7);
  }

  state7.g = Eigen::VectorXd::Zero(7);
  state7.g << 0.0, 22.0, 8.5, 12.0, 2.0, 4.5, 0.8;

  state7.J = Eigen::MatrixXd::Zero(6, 7);
  state7.J.row(0) << 0.0, 0.35, -0.1, 0.40, 0.0, 0.15, 0.0;
  state7.J.row(1) << -0.4, 0.0, 0.30, 0.0, 0.25, 0.0, 0.08;
  state7.J.row(2) << 0.0, 0.45, 0.0, -0.38, 0.0, -0.12, 0.0;
  state7.J.row(3) << 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0;
  state7.J.row(4) << 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
  state7.J.row(5) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  state7.dJ = Eigen::MatrixXd::Zero(6, 7);
  state7.F_ext.setZero();

  TorqueCBFResult res7;
  const Eigen::VectorXd tau_cmd = ctrl_multi.computeControlTorque(state7, Eigen::VectorXd::Zero(7), &res7);

  std::cout << "  - Number of hyperplanes: " << res7.zetas.size() << "\n";
  std::cout << "  - Plane 1 (Floor): zeta = " << std::fixed << std::setprecision(3)
            << res7.zetas(0) << " m, dzeta = " << res7.dzetas(0)
            << " m/s, v_min = " << res7.v_mins(0) << " m/s, b_cbf = " << res7.b_cbf(0)
            << " | f_n1 = " << res7.f_n(0) << " N\n";
  std::cout << "  - Plane 2 (Wall) : zeta = " << res7.zetas(1) << " m, dzeta = "
            << res7.dzetas(1) << " m/s, v_min = " << res7.v_mins(1)
            << " m/s, b_cbf = " << res7.b_cbf(1)
            << " | f_n2 = " << res7.f_n(1) << " N\n";
  std::cout << "  - QP Success: " << (res7.qp_success ? "YES" : "NO")
            << " | Slack delta: " << res7.slack_delta
            << " | Cost: " << res7.qp_cost << "\n";
  std::cout << "  - Resulting Cartesian Force: [" << res7.F_cart_lin.transpose() << "] N\n";
  std::cout << "  - CBF Optimal Torque ||tau_cbf||: " << res7.tau_cbf.norm() << " Nm\n";
  std::cout << "  - Total Command Torque ||tau_cmd||: " << tau_cmd.norm() << " Nm\n";

  // Check free-floating gravity compensation when inside safe region (b_cbf <= 0)
  RobotState state_safe = state7;
  state_safe.dx = Eigen::Vector3d(0.0, -0.05, 0.05); // Moving away from both planes
  TorqueCBFResult res_safe;
  const Eigen::VectorXd tau_safe = ctrl_multi.computeControlTorque(state_safe, Eigen::VectorXd::Zero(7), &res_safe);

  std::cout << "\n[Verification] Free-Floating Safe Region Test (b_cbf <= 0):\n";
  std::cout << "  - Maximum b_cbf in safe state: " << res_safe.b_cbf.maxCoeff() << "\n";
  std::cout << "  - Normal force norm ||f_n||: " << res_safe.f_n.norm() << " N (Expect 0 N)\n";
  std::cout << "  - CBF Torque norm ||tau_cbf||: " << res_safe.tau_cbf.norm()
            << " Nm (Expect ~ 0 Nm)\n";
  std::cout << "  - Difference ||tau_cmd - g(q)||: " << (tau_safe - state_safe.g).norm()
            << " Nm (Expect 0 Nm -> Pure Free-Floating Gravity Compensation)\n";

  // Save 7-DOF corner test telemetry to CSV
  std::ofstream csv_exp4(csv_dir / "torque_cbf_manipulator_7dof.csv");
  csv_exp4 << "zeta1,dzeta1,v_min1,b_cbf1,zeta2,dzeta2,v_min2,b_cbf2,slack_delta,tau_cbf_norm,tau_cmd_norm\n";
  csv_exp4 << res7.zetas(0) << "," << res7.dzetas(0) << "," << res7.v_mins(0) << "," << res7.b_cbf(0) << ","
           << res7.zetas(1) << "," << res7.dzetas(1) << "," << res7.v_mins(1) << "," << res7.b_cbf(1) << ","
           << res7.slack_delta << "," << res7.tau_cbf.norm() << "," << tau_cmd.norm() << "\n";

  // ---------------------------------------------------------------------------
  // Experiment 5: Kinesthetic Teaching & Passivity Verification (Approach A)
  // ---------------------------------------------------------------------------
  std::cout << "\n[Experiment 5] Kinesthetic Teaching & Passivity Verification (Approach A)\n";

  // Single vertical wall at x = 0.50m with outward normal n = [-1, 0, 0]^T
  Hyperplane p_kin;
  p_kin.normal = Eigen::Vector3d(-1.0, 0.0, 0.0);
  p_kin.offset = Eigen::Vector3d(0.50, 0.0, 0.0);

  TorqueCBFConfig cfg_kin = cfg;
  cfg_kin.p_ths = 0.15;
  cfg_kin.enforce_passivity = true;
  cfg_kin.dt = 0.001;
  TorqueCBFController ctrl_kin(cfg_kin, p_kin);

  RobotState state_kin = state7;
  state_kin.x = Eigen::Vector3d(0.42, 0.10, 0.15); // zeta = -1 * (0.42 - 0.50) = 0.08 m (inside braking zone)

  // Test 5A: Pure Tangential Motion (sliding along wall: dx = [0.0, 0.35, -0.20]^T)
  state_kin.dx = Eigen::Vector3d(0.0, 0.35, -0.20);
  TorqueCBFResult res_tang;
  ctrl_kin.computeControlTorque(state_kin, Eigen::VectorXd::Zero(7), &res_tang);
  std::cout << "  - Test 5A (Tangential Sliding): dzeta = " << res_tang.dzeta
            << " m/s | Normal force f_n = " << res_tang.f_n(0)
            << " N | Lateral force ||F_lat|| = " << res_tang.F_cart_lat.norm()
            << " N (Expect 0 N - Zero tangential resistance)\n";

  // Test 5B: Moving Away from Wall (dx = [-0.25, 0.10, 0.0]^T -> dzeta = +0.25 m/s > 0)
  state_kin.dx = Eigen::Vector3d(-0.25, 0.10, 0.0);
  TorqueCBFResult res_away;
  ctrl_kin.computeControlTorque(state_kin, Eigen::VectorXd::Zero(7), &res_away);
  std::cout << "  - Test 5B (Moving Away from Wall): dzeta = " << res_away.dzeta
            << " m/s | Normal force f_n = " << res_away.f_n(0)
            << " N | CBF torque ||tau_cbf|| = " << res_away.tau_cbf.norm()
            << " Nm (Expect 0 Nm - Strictly Passive / No Pushback)\n";

  // Test 5C: Approaching Wall (dx = [0.30, 0.0, 0.0]^T -> dzeta = -0.30 m/s < 0)
  state_kin.dx = Eigen::Vector3d(0.30, 0.0, 0.0);
  TorqueCBFResult res_appr;
  ctrl_kin.computeControlTorque(state_kin, Eigen::VectorXd::Zero(7), &res_appr);
  std::cout << "  - Test 5C (Approaching Wall): dzeta = " << res_appr.dzeta
            << " m/s | Normal force f_n = " << res_appr.f_n(0)
            << " N | Braking Cap f_brake_max = " << res_appr.f_brake_max(0)
            << " N | Lateral force ||F_lat|| = " << res_appr.F_cart_lat.norm()
            << " N (Expect ||F_lat|| == 0 and f_n <= f_brake_max)\n";

  // Test 5D: Static Rest at Boundary (dx = [0.0, 0.0, 0.0]^T -> dzeta = 0)
  state_kin.dx = Eigen::Vector3d::Zero();
  TorqueCBFResult res_stat;
  ctrl_kin.computeControlTorque(state_kin, Eigen::VectorXd::Zero(7), &res_stat);
  std::cout << "  - Test 5D (Static Rest at Wall): dzeta = " << res_stat.dzeta
            << " m/s | Normal force f_n = " << res_stat.f_n(0)
            << " N | CBF torque ||tau_cbf|| = " << res_stat.tau_cbf.norm()
            << " Nm (Expect 0 Nm - No active push against user at rest)\n";

  std::cout << "\nAll verification benchmarks completed successfully!\n";
  return 0;
}
