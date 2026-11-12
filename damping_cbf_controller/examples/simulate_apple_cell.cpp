#include "damping_cbf/damping_cbf_controller.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

  std::string runXacro(const std::string &xacro_path)
  {
    std::string cmd = "xacro " + xacro_path;
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
    {
      throw std::runtime_error("Failed to execute command: " + cmd);
    }
    char buffer[4096];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
      result += buffer;
    }
    int status = pclose(pipe);
    if (status != 0)
    {
      throw std::runtime_error("xacro execution failed with exit code: " + std::to_string(status));
    }
    return result;
  }

  struct DesiredKinematics
  {
    Eigen::Vector3d x;
    Eigen::Vector3d v;
  };

  struct TrajectoryConfig
  {
    double x_center = 0.48;
    double x_amp1 = 0.09;
    double x_amp2 = -0.03;

    double y_center = 0.15;
    double y_amp1 = 0.17;

    double z_center = 1.18;
    double z_amp1 = -0.125;
    double z_amp2 = 0.02;
  };

  DesiredKinematics getDesiredTrajectory(double t, double period, const TrajectoryConfig &tc)
  {
    constexpr double pi = 3.14159265358979323846;
    const double tau = 2.0 * pi * t / period;
    const double dtau = 2.0 * pi / period;

    const double x_d = tc.x_center + tc.x_amp1 * std::cos(tau) + tc.x_amp2 * std::cos(2.0 * tau);
    const double y_d = tc.y_center + tc.y_amp1 * std::sin(tau);
    const double z_d = tc.z_center + tc.z_amp1 * std::cos(tau) + tc.z_amp2 * std::cos(2.0 * tau);

    const double vx_d = -tc.x_amp1 * std::sin(tau) * dtau - 2.0 * tc.x_amp2 * std::sin(2.0 * tau) * dtau;
    const double vy_d = tc.y_amp1 * std::cos(tau) * dtau;
    const double vz_d = -tc.z_amp1 * std::sin(tau) * dtau - 2.0 * tc.z_amp2 * std::sin(2.0 * tau) * dtau;

    return {Eigen::Vector3d(x_d, y_d, z_d), Eigen::Vector3d(vx_d, vy_d, vz_d)};
  }

} // namespace

int main(int argc, char **argv)
{
  std::string config_file = "";
  std::string xacro_override = "";
  std::string csv_override = "";
  int periods_override = -1;
  double period_override = -1.0;
  double duration_override = -1.0;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc)
      config_file = argv[++i];
    else if (arg == "--xacro" && i + 1 < argc)
      xacro_override = argv[++i];
    else if (arg == "--csv" && i + 1 < argc)
      csv_override = argv[++i];
    else if (arg == "--periods" && i + 1 < argc)
      periods_override = std::stoi(argv[++i]);
    else if (arg == "--period" && i + 1 < argc)
      period_override = std::stod(argv[++i]);
    else if (arg == "--duration" && i + 1 < argc)
      duration_override = std::stod(argv[++i]);
    else if (arg == "-h" || arg == "--help")
    {
      std::cout << "Usage: " << argv[0]
                << " [--config <path>] [--xacro <path>] [--csv <path>] [--periods <int>] [--period <double>] [--duration <double>]\n";
      return 0;
    }
  }

  // Find configuration file if not provided explicitly
  if (config_file.empty())
  {
    const std::vector<std::string> search_paths = {
        "config/apple_cell_cbf.yaml",
        "../config/apple_cell_cbf.yaml",
        "/home/galileo/projects/damping_cbf_ws/src/damping_cbf_controller/config/apple_cell_cbf.yaml",
        "/home/galileo/projects/damping_cbf_ws/install/damping_cbf/share/damping_cbf/config/apple_cell_cbf.yaml"};
    for (const auto &p : search_paths)
    {
      if (fs::exists(p))
      {
        config_file = p;
        break;
      }
    }
  }

  // Initial defaults matching the hardcoded code
  std::string xacro_file =
      "/home/galileo/projects/apple_cell_ws/src/apple_cell/apple_picking_cell_description/urdf/apple_picking_cell.urdf.xacro";
  std::string out_csv_path =
      "/home/galileo/projects/damping_cbf_ws/src/damping_cbf_controller/examples/csv/sim_apple_cell_trajectory.csv";
  std::string tool_frame = "ur10e_tool0";

  double dt = 0.001;
  double period = 6.5;
  int num_periods = 2;

  TrajectoryConfig traj_cfg;

  damping_cbf::ControllerConfig cfg;
  cfg.p_ths = 0.35;
  cfg.v_ths = -0.003;
  cfg.v_0 = -0.4;
  cfg.gamma = 2.0;
  cfg.d_max = 50.0;
  cfg.profile = damping_cbf::ProfileType::SMOOTHSTEP;
  cfg.compensate_external_force = true;
  cfg.K.setZero();
  cfg.K.block<3, 3>(0, 0) = 450.0 * Eigen::Matrix3d::Identity();
  cfg.K.block<3, 3>(3, 3) = 60.0 * Eigen::Matrix3d::Identity();
  cfg.D.setZero();
  cfg.D.block<3, 3>(0, 0) = 55.0 * Eigen::Matrix3d::Identity();
  cfg.D.block<3, 3>(3, 3) = 10.0 * Eigen::Matrix3d::Identity();

  Eigen::MatrixXd K_posture = 25.0 * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd D_posture = 6.0 * Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd q_init = Eigen::VectorXd::Zero(6);
  q_init << 1.57, -1.8, 1.8, -1.57, -1.57, 0.0;

  std::vector<damping_cbf::Hyperplane> planes(2);
  planes[0].normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  planes[0].offset = Eigen::Vector3d(0.48, 0.16, 1.05);
  planes[1].normal = Eigen::Vector3d(-1.0 / std::sqrt(2.0), 0.0, 1.0 / std::sqrt(2.0));
  planes[1].offset = Eigen::Vector3d(0.55, 0.16, 1.05);

  if (!config_file.empty() && fs::exists(config_file))
  {
    std::cout << "Loading configuration from: " << config_file << "\n";
    try
    {
      YAML::Node yaml = YAML::LoadFile(config_file);
      if (yaml["paths"])
      {
        if (yaml["paths"]["xacro_file"])
          xacro_file = yaml["paths"]["xacro_file"].as<std::string>();
        if (yaml["paths"]["csv_output"])
        {
          std::string p = yaml["paths"]["csv_output"].as<std::string>();
          if (fs::path(p).is_absolute())
            out_csv_path = p;
          else
          {
            fs::path pkg_dir = "/home/galileo/projects/damping_cbf_ws/src/damping_cbf_controller";
            out_csv_path = fs::exists(pkg_dir) ? (pkg_dir / p).string() : p;
          }
        }
      }

      if (yaml["simulation"])
      {
        if (yaml["simulation"]["dt"])
          dt = yaml["simulation"]["dt"].as<double>();
        if (yaml["simulation"]["period"])
          period = yaml["simulation"]["period"].as<double>();
        if (yaml["simulation"]["num_periods"])
          num_periods = yaml["simulation"]["num_periods"].as<int>();
        if (yaml["simulation"]["tool_frame"])
          tool_frame = yaml["simulation"]["tool_frame"].as<std::string>();
      }

      if (yaml["planes"])
      {
        planes.clear();
        for (const auto &kv : yaml["planes"])
        {
          YAML::Node pnode = kv.second;
          damping_cbf::Hyperplane hp;
          auto n_vec = pnode["normal"].as<std::vector<double>>();
          auto o_vec = pnode["offset"].as<std::vector<double>>();
          hp.normal = Eigen::Vector3d(n_vec[0], n_vec[1], n_vec[2]);
          hp.offset = Eigen::Vector3d(o_vec[0], o_vec[1], o_vec[2]);
          planes.push_back(hp);
        }
      }

      if (yaml["cbf"])
      {
        if (yaml["cbf"]["p_ths"])
          cfg.p_ths = yaml["cbf"]["p_ths"].as<double>();
        if (yaml["cbf"]["v_ths"])
          cfg.v_ths = yaml["cbf"]["v_ths"].as<double>();
        if (yaml["cbf"]["v_0"])
          cfg.v_0 = yaml["cbf"]["v_0"].as<double>();
        if (yaml["cbf"]["gamma"])
          cfg.gamma = yaml["cbf"]["gamma"].as<double>();
        if (yaml["cbf"]["d_max"])
          cfg.d_max = yaml["cbf"]["d_max"].as<double>();
        if (yaml["cbf"]["d_max_inertia_gain"])
          cfg.d_max_inertia_gain = yaml["cbf"]["d_max_inertia_gain"].as<double>();
        if (yaml["cbf"]["compensate_external_force"])
          cfg.compensate_external_force = yaml["cbf"]["compensate_external_force"].as<bool>();
        if (yaml["cbf"]["profile"])
        {
          std::string prof = yaml["cbf"]["profile"].as<std::string>();
          if (prof == "quadratic")
            cfg.profile = damping_cbf::ProfileType::QUINTIC;
          else if (prof == "linear")
            cfg.profile = damping_cbf::ProfileType::LINEAR;
          else
            cfg.profile = damping_cbf::ProfileType::SMOOTHSTEP;
        }
      }

      if (yaml["impedance"])
      {
        if (yaml["impedance"]["K_pos"])
        {
          auto kp = yaml["impedance"]["K_pos"].as<std::vector<double>>();
          cfg.K.block<3, 3>(0, 0) = Eigen::Vector3d(kp[0], kp[1], kp[2]).asDiagonal();
        }
        if (yaml["impedance"]["K_rot"])
        {
          auto kr = yaml["impedance"]["K_rot"].as<std::vector<double>>();
          cfg.K.block<3, 3>(3, 3) = Eigen::Vector3d(kr[0], kr[1], kr[2]).asDiagonal();
        }
        if (yaml["impedance"]["D_pos"])
        {
          auto dp = yaml["impedance"]["D_pos"].as<std::vector<double>>();
          cfg.D.block<3, 3>(0, 0) = Eigen::Vector3d(dp[0], dp[1], dp[2]).asDiagonal();
        }
        if (yaml["impedance"]["D_rot"])
        {
          auto dr = yaml["impedance"]["D_rot"].as<std::vector<double>>();
          cfg.D.block<3, 3>(3, 3) = Eigen::Vector3d(dr[0], dr[1], dr[2]).asDiagonal();
        }
      }

      if (yaml["nullspace"])
      {
        if (yaml["nullspace"]["K_posture"])
        {
          auto kp = yaml["nullspace"]["K_posture"].as<std::vector<double>>();
          Eigen::VectorXd v(6);
          for (int i = 0; i < 6; ++i)
            v(i) = kp[i];
          K_posture = v.asDiagonal();
        }
        if (yaml["nullspace"]["D_posture"])
        {
          auto dp = yaml["nullspace"]["D_posture"].as<std::vector<double>>();
          Eigen::VectorXd v(6);
          for (int i = 0; i < 6; ++i)
            v(i) = dp[i];
          D_posture = v.asDiagonal();
        }
        if (yaml["nullspace"]["q_init"])
        {
          auto qi = yaml["nullspace"]["q_init"].as<std::vector<double>>();
          for (int i = 0; i < 6; ++i)
            q_init(i) = qi[i];
        }
      }

      if (yaml["trajectory"])
      {
        if (yaml["trajectory"]["x"])
        {
          if (yaml["trajectory"]["x"]["center"])
            traj_cfg.x_center = yaml["trajectory"]["x"]["center"].as<double>();
          if (yaml["trajectory"]["x"]["amp_cos1"])
            traj_cfg.x_amp1 = yaml["trajectory"]["x"]["amp_cos1"].as<double>();
          if (yaml["trajectory"]["x"]["amp_cos2"])
            traj_cfg.x_amp2 = yaml["trajectory"]["x"]["amp_cos2"].as<double>();
        }
        if (yaml["trajectory"]["y"])
        {
          if (yaml["trajectory"]["y"]["center"])
            traj_cfg.y_center = yaml["trajectory"]["y"]["center"].as<double>();
          if (yaml["trajectory"]["y"]["amp_sin1"])
            traj_cfg.y_amp1 = yaml["trajectory"]["y"]["amp_sin1"].as<double>();
        }
        if (yaml["trajectory"]["z"])
        {
          if (yaml["trajectory"]["z"]["center"])
            traj_cfg.z_center = yaml["trajectory"]["z"]["center"].as<double>();
          if (yaml["trajectory"]["z"]["amp_cos1"])
            traj_cfg.z_amp1 = yaml["trajectory"]["z"]["amp_cos1"].as<double>();
          if (yaml["trajectory"]["z"]["amp_cos2"])
            traj_cfg.z_amp2 = yaml["trajectory"]["z"]["amp_cos2"].as<double>();
        }
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "Warning: Failed to parse YAML config (" << e.what() << "). Using built-in defaults.\n";
    }
  }

  // Apply CLI overrides if specified
  if (!xacro_override.empty())
    xacro_file = xacro_override;
  if (!csv_override.empty())
    out_csv_path = csv_override;
  if (periods_override > 0)
    num_periods = periods_override;
  if (period_override > 0.0)
    period = period_override;

  const double duration = (duration_override > 0.0) ? duration_override : (num_periods * period);
  const int n_steps = static_cast<int>(duration / dt);

  std::cout << "======================================================================\n";
  std::cout << "  UR10e Apple Picking Cell — Pinocchio Dynamics & C++ CBF Controller\n";
  std::cout << "======================================================================\n";
  std::cout << "Expanding Xacro: " << xacro_file << "\n";

  std::string urdf_xml;
  try
  {
    urdf_xml = runXacro(xacro_file);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error running xacro: " << e.what() << "\n";
    return 1;
  }

  // Parse Pinocchio model from URDF XML
  pinocchio::Model full_model;
  pinocchio::urdf::buildModelFromXML(urdf_xml, full_model);

  // Identify arm joints and lock gripper/mimic joints
  const std::vector<std::string> arm_joints = {
      "ur10e_shoulder_pan_joint",
      "ur10e_shoulder_lift_joint",
      "ur10e_elbow_joint",
      "ur10e_wrist_1_joint",
      "ur10e_wrist_2_joint",
      "ur10e_wrist_3_joint"};

  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (std::size_t i = 1; i < full_model.names.size(); ++i)
  {
    const auto &name = full_model.names[i];
    bool is_arm = false;
    for (const auto &aj : arm_joints)
    {
      if (name == aj)
      {
        is_arm = true;
        break;
      }
    }
    if (!is_arm)
    {
      joints_to_lock.push_back(static_cast<pinocchio::JointIndex>(i));
    }
  }

  const Eigen::VectorXd q_neutral = pinocchio::neutral(full_model);
  pinocchio::Model model;
  pinocchio::buildReducedModel(full_model, joints_to_lock, q_neutral, model);
  pinocchio::Data data(model);

  const pinocchio::FrameIndex fid = model.getFrameId(tool_frame);
  if (fid >= model.frames.size())
  {
    std::cerr << "Error: frame '" << tool_frame << "' not found in model!\n";
    return 1;
  }

  std::cout << "Loaded Pinocchio model: " << model.nq
            << " joints (arm joints locked + gripper compounded)\n";

  // Configure C++ DampingCBFController
  damping_cbf::DampingCBFController cbf_ctrl(cfg, planes[1]);

  // Initial joint configuration via numerical IK to match x_des(0)
  Eigen::VectorXd q = q_init;
  const DesiredKinematics des0 = getDesiredTrajectory(0.0, period, traj_cfg);
  pinocchio::Data::Matrix6x J6(6, model.nv);

  for (int iter = 0; iter < 100; ++iter)
  {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const Eigen::Vector3d err = des0.x - data.oMf[fid].translation();
    if (err.norm() < 1e-5)
      break;

    J6.setZero();
    pinocchio::computeFrameJacobian(model, data, q, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6);
    const Eigen::Matrix<double, 3, 6> J_pos = J6.topRows<3>();
    const Eigen::Matrix3d JJt = J_pos * J_pos.transpose();
    const Eigen::VectorXd delta_q = J_pos.transpose() * JJt.ldlt().solve(err);
    q += delta_q;
  }

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  J6.setZero();
  pinocchio::computeFrameJacobian(model, data, q, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6);
  const Eigen::Matrix<double, 3, 6> J_pos0 = J6.topRows<3>();
  const Eigen::Matrix3d JJt0 = J_pos0 * J_pos0.transpose();
  Eigen::VectorXd dq = J_pos0.transpose() * JJt0.ldlt().solve(des0.v);

  const Eigen::VectorXd q_posture = q;
  const Eigen::Matrix3d R_0 = data.oMf[fid].rotation();

  // Prepare output CSV
  fs::create_directories(fs::path(out_csv_path).parent_path());
  std::ofstream csv(out_csv_path);
  if (!csv.is_open())
  {
    std::cerr << "Error: unable to open output CSV: " << out_csv_path << "\n";
    return 1;
  }

  csv << "t,x,y,z,dx,dy,dz,"
      << "zeta1,dzeta1,v_min1,B1,act1,"
      << "zeta2,dzeta2,v_min2,B2,act2,"
      << "d1,d2,trace_D,"
      << "q0,q1,q2,q3,q4,q5,"
      << "dq0,dq1,dq2,dq3,dq4,dq5,"
      << "tau0,tau1,tau2,tau3,tau4,tau5,tau_norm\n";
  csv << std::setprecision(8) << std::fixed;

  std::cout << "Simulating " << n_steps << " steps (dt = " << (dt * 1e3)
            << " ms, period = " << period << " s, periods = " << num_periods
            << ", duration = " << duration << " s) using C++ CBF Controller...\n";

  pinocchio::Data::Matrix6x dJ6(6, model.nv);

  // 1 kHz Simulation & Control Loop
  for (int step = 0; step < n_steps; ++step)
  {
    const double t = step * dt;

    // Pinocchio Kinematics & Jacobians
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
    pinocchio::updateFramePlacements(model, data);

    const Eigen::Vector3d x = data.oMf[fid].translation();
    const Eigen::Matrix3d R_ee = data.oMf[fid].rotation();

    J6.setZero();
    dJ6.setZero();
    pinocchio::getFrameJacobian(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6);
    pinocchio::getFrameJacobianTimeVariation(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, dJ6);

    const Eigen::Matrix<double, 6, 1> dx6 = J6 * dq;
    const Eigen::Vector3d dx = dx6.head<3>();

    // Pinocchio's LOCAL_WORLD_ALIGNED dJ gives the SPATIAL acceleration; make it the true time derivative
    // of J (classical acceleration, as Body2Hybrid in the Simulink model): dJ_lin += -[v]x * J_ang
    {
      Eigen::Matrix3d v_skew;
      v_skew << 0.0, -dx.z(), dx.y(), dx.z(), 0.0, -dx.x(), -dx.y(), dx.x(), 0.0;
      dJ6.topRows<3>() -= v_skew * J6.bottomRows<3>();
    }

    // Pinocchio Full Non-Linear Dynamics
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::computeCoriolisMatrix(model, data, q, dq);
    pinocchio::computeGeneralizedGravity(model, data, q);

    // Fill RobotState for the C++ controller
    damping_cbf::RobotState rstate;
    rstate.q = q;
    rstate.dq = dq;
    rstate.x = x;
    rstate.dx = dx;
    // Orientation error expressed in world frame: log3(R_ee * R_0^T)
    rstate.phi = pinocchio::log3(R_ee * R_0.transpose());
    rstate.dphi = dx6.tail<3>();
    rstate.M = data.M;
    rstate.C = data.C;
    rstate.g = data.g;
    rstate.J = J6;
    rstate.dJ = dJ6;
    rstate.F_ext.setZero();

    // Desired kinematic reference position
    const DesiredKinematics des = getDesiredTrajectory(t, period, traj_cfg);
    const Eigen::Vector3d phi_ref = Eigen::Vector3d::Zero();

    // Dynamically consistent nullspace posture regulation
    const Eigen::MatrixXd J6_pinv_T = (J6 * J6.transpose()).ldlt().solve(J6);
    const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(6, 6) - J6.transpose() * J6_pinv_T;
    const Eigen::VectorXd tau_null = N * (-K_posture * (q - q_posture) - D_posture * dq);

    // Call the C++ Controller's single-surface computeControlTorque for the 45° wall!
    damping_cbf::CBFStepResult cbf_res;
    const Eigen::VectorXd tau = cbf_ctrl.computeControlTorque(
        rstate, des.x, phi_ref, tau_null, &cbf_res);
    // Non-linear forward dynamics integration
    const Eigen::VectorXd ddq = data.M.ldlt().solve(tau - data.C * dq - data.g);
    dq += ddq * dt;
    q += dq * dt;

    // Distances and barrier values for telemetry
    const double zeta1 = planes[0].normal.dot(x - planes[0].offset);
    const double dzeta1 = planes[0].normal.dot(dx);
    const double v_min1 = cbf_ctrl.computeVelocityEnvelope(zeta1);
    const double B1 = dzeta1 - v_min1;

    const double d1_val = 0.0;           // Table has no CBF active
    const double d2_val = cbf_res.d_cbf; // 45° wall optimal damping

    csv << t << "," << x.x() << "," << x.y() << "," << x.z() << ","
        << dx.x() << "," << dx.y() << "," << dx.z() << ","
        << zeta1 << "," << dzeta1 << "," << v_min1 << "," << B1 << "," << 0 << "," // Table inactive
        << cbf_res.zeta << "," << cbf_res.dzeta << "," << cbf_res.v_min << "," << cbf_res.barrier_B << ","
        << (cbf_res.is_active ? 1 : 0) << ","
        << d1_val << "," << d2_val << "," << cbf_res.D_imp.trace() << ",";

    for (int j = 0; j < 6; ++j)
      csv << q(j) << ",";
    for (int j = 0; j < 6; ++j)
      csv << dq(j) << ",";
    for (int j = 0; j < 6; ++j)
      csv << tau(j) << ",";
    csv << tau.norm() << "\n";
  }

  csv.close();
  std::cout << "Saved trajectory data: " << out_csv_path << " (" << n_steps << " points)\n";
  return 0;
}
