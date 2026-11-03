#include "torque_cbf/torque_cbf_controller.hpp"

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
  int kinesthetic_override = -1; // -1: use YAML, 1: force kinesthetic teaching, 0: force autonomous

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
    else if (arg == "--kinesthetic" || arg == "--lead-through")
      kinesthetic_override = 1;
    else if (arg == "--autonomous")
      kinesthetic_override = 0;
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: " << argv[0]
                << " [--config <path>] [--xacro <path>] [--csv <path>] [--periods <int>] [--period <double>] [--duration <double>] [--kinesthetic|--autonomous]\n";
      return 0;
    }
  }

  // Fallback search paths for configuration YAML
  if (config_file.empty())
  {
    std::vector<std::string> search_paths = {
        "config/apple_cell_torque_cbf.yaml",
        "../config/apple_cell_torque_cbf.yaml",
        "src/torque_cbf_controller/config/apple_cell_torque_cbf.yaml",
        "/home/galileo/projects/damping_cbf_ws/src/torque_cbf_controller/config/apple_cell_torque_cbf.yaml"};
    for (const auto &p : search_paths)
    {
      if (fs::exists(p))
      {
        config_file = p;
        break;
      }
    }
  }

  // Default parameters
  double dt = 0.001;
  double period = 4.5;
  int num_periods = 2;
  std::string tool_frame = "ur10e_tool0";
  std::string xacro_file = "/home/galileo/projects/apple_cell_ws/src/apple_cell/apple_picking_cell_description/urdf/apple_picking_cell.urdf.xacro";
  std::string out_csv_path = "src/torque_cbf_controller/examples/csv/sim_apple_cell_torque_trajectory.csv";

  torque_cbf::TorqueCBFConfig cfg;
  cfg.p_ths = 0.25;
  cfg.v_ths = -0.003;
  cfg.v_0 = -0.40;
  cfg.gamma = 20.0;
  cfg.w_slack = 10000.0;
  cfg.w_lin = 1000000.0;
  cfg.profile = torque_cbf::ProfileType::SMOOTHSTEP;
  cfg.compensate_external_force = true;
  cfg.default_tau_limit = 150.0;

  Eigen::Matrix<double, 6, 6> K_cart = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> D_cart = Eigen::Matrix<double, 6, 6>::Zero();
  K_cart.topLeftCorner<3, 3>() = Eigen::Vector3d(450.0, 450.0, 450.0).asDiagonal();
  K_cart.bottomRightCorner<3, 3>() = Eigen::Vector3d(60.0, 60.0, 60.0).asDiagonal();
  D_cart.topLeftCorner<3, 3>() = Eigen::Vector3d(55.0, 55.0, 55.0).asDiagonal();
  D_cart.bottomRightCorner<3, 3>() = Eigen::Vector3d(10.0, 10.0, 10.0).asDiagonal();

  // Kinesthetic Teaching (Lead-Through) Parameters
  bool kinesthetic_teaching = true;
  double max_human_force = 80.0;
  Eigen::Matrix<double, 6, 6> K_operator = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> D_operator = Eigen::Matrix<double, 6, 6>::Zero();
  K_operator.topLeftCorner<3, 3>() = Eigen::Vector3d(400.0, 400.0, 400.0).asDiagonal();
  K_operator.bottomRightCorner<3, 3>() = Eigen::Vector3d(50.0, 50.0, 50.0).asDiagonal();
  D_operator.topLeftCorner<3, 3>() = Eigen::Vector3d(45.0, 45.0, 45.0).asDiagonal();
  D_operator.bottomRightCorner<3, 3>() = Eigen::Vector3d(8.0, 8.0, 8.0).asDiagonal();

  Eigen::MatrixXd K_posture = Eigen::VectorXd::Constant(6, 25.0).asDiagonal();
  Eigen::MatrixXd D_posture = Eigen::VectorXd::Constant(6, 6.0).asDiagonal();
  Eigen::VectorXd q_init(6);
  q_init << 1.57, -1.8, 1.8, -1.57, -1.57, 0.0;

  TrajectoryConfig traj_cfg;

  std::vector<torque_cbf::Hyperplane> planes(2);
  // Plane 1: Table surface
  planes[0].normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  planes[0].offset = Eigen::Vector3d(0.48, 0.16, 1.05);
  // Plane 2: 45° Wall
  planes[1].normal = Eigen::Vector3d(-0.7071067811865475, 0.0, 0.7071067811865475);
  planes[1].offset = Eigen::Vector3d(0.55, 0.16, 1.05);

  if (!config_file.empty() && fs::exists(config_file))
  {
    std::cout << "Loading configuration from: " << config_file << "\n";
    try
    {
      YAML::Node yaml = YAML::LoadFile(config_file);
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

      if (yaml["paths"])
      {
        if (yaml["paths"]["xacro_file"])
          xacro_file = yaml["paths"]["xacro_file"].as<std::string>();
        if (yaml["paths"]["csv_output"])
          out_csv_path = yaml["paths"]["csv_output"].as<std::string>();
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
        if (yaml["cbf"]["w_slack"])
          cfg.w_slack = yaml["cbf"]["w_slack"].as<double>();
        if (yaml["cbf"]["w_lin"])
          cfg.w_lin = yaml["cbf"]["w_lin"].as<double>();
        if (yaml["cbf"]["tau_limit"])
          cfg.default_tau_limit = yaml["cbf"]["tau_limit"].as<double>();
        if (yaml["cbf"]["compensate_external_force"])
          cfg.compensate_external_force = yaml["cbf"]["compensate_external_force"].as<bool>();
      }

      if (yaml["kinesthetic_teaching"])
      {
        if (yaml["kinesthetic_teaching"]["enabled"])
          kinesthetic_teaching = yaml["kinesthetic_teaching"]["enabled"].as<bool>();
        if (yaml["kinesthetic_teaching"]["max_force"])
          max_human_force = yaml["kinesthetic_teaching"]["max_force"].as<double>();
        if (yaml["kinesthetic_teaching"]["K_operator_pos"])
        {
          auto kp = yaml["kinesthetic_teaching"]["K_operator_pos"].as<std::vector<double>>();
          K_operator.topLeftCorner<3, 3>() = Eigen::Vector3d(kp[0], kp[1], kp[2]).asDiagonal();
        }
        if (yaml["kinesthetic_teaching"]["K_operator_rot"])
        {
          auto kr = yaml["kinesthetic_teaching"]["K_operator_rot"].as<std::vector<double>>();
          K_operator.bottomRightCorner<3, 3>() = Eigen::Vector3d(kr[0], kr[1], kr[2]).asDiagonal();
        }
        if (yaml["kinesthetic_teaching"]["D_operator_pos"])
        {
          auto dp = yaml["kinesthetic_teaching"]["D_operator_pos"].as<std::vector<double>>();
          D_operator.topLeftCorner<3, 3>() = Eigen::Vector3d(dp[0], dp[1], dp[2]).asDiagonal();
        }
        if (yaml["kinesthetic_teaching"]["D_operator_rot"])
        {
          auto dr = yaml["kinesthetic_teaching"]["D_operator_rot"].as<std::vector<double>>();
          D_operator.bottomRightCorner<3, 3>() = Eigen::Vector3d(dr[0], dr[1], dr[2]).asDiagonal();
        }
      }

      if (yaml["impedance"])
      {
        if (yaml["impedance"]["K_pos"])
        {
          auto kp = yaml["impedance"]["K_pos"].as<std::vector<double>>();
          K_cart.topLeftCorner<3, 3>() = Eigen::Vector3d(kp[0], kp[1], kp[2]).asDiagonal();
        }
        if (yaml["impedance"]["K_rot"])
        {
          auto kr = yaml["impedance"]["K_rot"].as<std::vector<double>>();
          K_cart.bottomRightCorner<3, 3>() = Eigen::Vector3d(kr[0], kr[1], kr[2]).asDiagonal();
        }
        if (yaml["impedance"]["D_pos"])
        {
          auto dp = yaml["impedance"]["D_pos"].as<std::vector<double>>();
          D_cart.topLeftCorner<3, 3>() = Eigen::Vector3d(dp[0], dp[1], dp[2]).asDiagonal();
        }
        if (yaml["impedance"]["D_rot"])
        {
          auto dr = yaml["impedance"]["D_rot"].as<std::vector<double>>();
          D_cart.bottomRightCorner<3, 3>() = Eigen::Vector3d(dr[0], dr[1], dr[2]).asDiagonal();
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
      std::cerr << "Warning: Failed to parse YAML config (" << e.what() << "). Using defaults.\n";
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
  if (kinesthetic_override != -1)
    kinesthetic_teaching = (kinesthetic_override == 1);

  const double duration = (duration_override > 0.0) ? duration_override : (num_periods * period);
  const int n_steps = static_cast<int>(duration / dt);

  std::cout << "======================================================================\n";
  std::cout << "  UR10e Apple Picking Cell — Direct Joint-Torque CBF Controller\n";
  std::cout << "======================================================================\n";
  std::cout << "Mode: "
            << (kinesthetic_teaching
                    ? "Kinesthetic Teaching (Zero-g gravity compensation + CBF torque braking; Operator applying external wrench F_ext)"
                    : "Autonomous Impedance Trajectory Tracking")
            << "\n";
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

  // Arm joints: lock gripper / finger joints
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

  // Preload hyperplanes into C++ TorqueCBFController (45° obstacle wall)
  cfg.dt = dt;
  cfg.enforce_passivity = true;
  torque_cbf::TorqueCBFController cbf_ctrl(cfg, planes[1]);

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

  // Ensure CSV output path exists in examples/csv
  fs::path csv_path(out_csv_path);
  if (!csv_path.is_absolute())
  {
    csv_path = fs::current_path() / out_csv_path;
  }
  fs::create_directories(csv_path.parent_path());
  std::ofstream csv(csv_path);
  if (!csv.is_open())
  {
    std::cerr << "Error: unable to open output CSV: " << csv_path << "\n";
    return 1;
  }

  csv << "t,x,y,z,dx,dy,dz,"
      << "zeta1,dzeta1,v_min1,B1,act1,"
      << "zeta2,dzeta2,v_min2,B2,act2,"
      << "tau_cbf_norm,slack_delta,"
      << "q0,q1,q2,q3,q4,q5,"
      << "dq0,dq1,dq2,dq3,dq4,dq5,"
      << "tau_cmd0,tau_cmd1,tau_cmd2,tau_cmd3,tau_cmd4,tau_cmd5,tau_cmd_norm,"
      << "tau_cbf0,tau_cbf1,tau_cbf2,tau_cbf3,tau_cbf4,tau_cbf5,"
      << "f_ext_x,f_ext_y,f_ext_z,f_ext_norm,"
      << "f_cbf_x,f_cbf_y,f_cbf_z,f_cbf_norm\n";
  csv << std::setprecision(8) << std::fixed;

  std::cout << "Simulating " << n_steps << " steps (dt = " << (dt * 1e3)
            << " ms, period = " << period << " s, duration = " << duration
            << " s) using Direct Joint-Torque CBF Controller...\n";

  // 1 kHz Simulation & Control Loop
  for (int step = 0; step < n_steps; ++step)
  {
    const double t = step * dt;

    // Pinocchio Kinematics & Jacobians
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const Eigen::Vector3d x = data.oMf[fid].translation();
    const Eigen::Matrix3d R_ee = data.oMf[fid].rotation();

    J6.setZero();
    pinocchio::getFrameJacobian(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, J6);
    const Eigen::Matrix<double, 6, 1> dx6 = J6 * dq;
    const Eigen::Vector3d dx = dx6.head<3>();

    // Trajectory tracking error & desired state
    const DesiredKinematics des = getDesiredTrajectory(t, period, traj_cfg);
    Eigen::Matrix<double, 6, 1> X_dot;
    X_dot.head<3>() = dx - des.v;
    X_dot.tail<3>() = dx6.tail<3>();

    Eigen::Matrix<double, 6, 1> X_tilde;
    X_tilde.head<3>() = x - des.x;
    X_tilde.tail<3>() = pinocchio::log3(R_ee * R_0.transpose());

    // Dynamically consistent nullspace posture regulation
    const Eigen::MatrixXd J6_pinv_T = (J6 * J6.transpose()).ldlt().solve(J6);
    const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(6, 6) - J6.transpose() * J6_pinv_T;
    const Eigen::VectorXd tau_null = N * (-K_posture * (q - q_posture) - D_posture * dq);

    Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::VectorXd tau_nom = Eigen::VectorXd::Zero(model.nv);

    if (kinesthetic_teaching)
    {
      // 1. Controller Reference:
      //    The controller is given ZERO Cartesian tracking reference (F_nominal = 0).
      //    The robot operates in pure zero-gravity lead-through: tau_cmd = g(q) + tau_null + tau_cbf.
      //    Only posture nullspace damping is active to prevent redundant joint drift.
      tau_nom = tau_null;

      // 2. Human Operator Force:
      //    The operator holds the end-effector and applies an external physical wrench F_ext to
      //    guide the robot along the intended teaching reference path.
      Eigen::Vector3d F_human_pos = K_operator.topLeftCorner<3, 3>() * (des.x - x) +
                                    D_operator.topLeftCorner<3, 3>() * (des.v - dx);
      if (F_human_pos.norm() > max_human_force)
      {
        F_human_pos = F_human_pos.normalized() * max_human_force;
      }
      Eigen::Vector3d F_human_rot = -K_operator.bottomRightCorner<3, 3>() * X_tilde.tail<3>() -
                                     D_operator.bottomRightCorner<3, 3>() * dx6.tail<3>();

      F_ext.head<3>() = F_human_pos;
      F_ext.tail<3>() = F_human_rot;
    }
    else
    {
      // Autonomous impedance trajectory tracking
      const Eigen::Matrix<double, 6, 1> F_nominal = -K_cart * X_tilde - D_cart * X_dot;
      tau_nom = J6.transpose() * F_nominal + tau_null;
      F_ext.setZero();
    }

    // Direct Joint-Torque CBF Controller invocation
    torque_cbf::TorqueCBFResult cbf_res;
    const Eigen::VectorXd tau_cmd = cbf_ctrl.computeControlTorque(
        model, data, q, dq, fid, F_ext, tau_nom, &cbf_res);

    // Forward non-linear dynamics integration:
    // M(q)*ddq + C(q, dq)*dq + g(q) = tau_cmd + J(q)^T * F_ext
    pinocchio::computeCoriolisMatrix(model, data, q, dq);
    const Eigen::VectorXd tau_total_joints = tau_cmd + J6.transpose() * F_ext - data.C * dq - data.g;
    const Eigen::VectorXd ddq = data.M.ldlt().solve(tau_total_joints);
    dq += ddq * dt;
    q += dq * dt;

    // Barrier values for reporting
    const double zeta1 = planes[0].normal.dot(x - planes[0].offset);
    const double dzeta1 = planes[0].normal.dot(dx);
    const double v_min1 = cbf_ctrl.computeVelocityEnvelope(zeta1, 0);
    const double B1 = dzeta1 - v_min1;

    csv << t << "," << x.x() << "," << x.y() << "," << x.z() << ","
        << dx.x() << "," << dx.y() << "," << dx.z() << ","
        << zeta1 << "," << dzeta1 << "," << v_min1 << "," << B1 << "," << 0 << ","
        << cbf_res.zeta << "," << cbf_res.dzeta << "," << cbf_res.v_min << "," << cbf_res.barrier_B << ","
        << (cbf_res.is_active ? 1 : 0) << ","
        << cbf_res.tau_cbf.norm() << "," << cbf_res.slack_delta << ",";

    for (int j = 0; j < 6; ++j)
      csv << q(j) << ",";
    for (int j = 0; j < 6; ++j)
      csv << dq(j) << ",";
    for (int j = 0; j < 6; ++j)
      csv << tau_cmd(j) << ",";
    csv << tau_cmd.norm() << ",";
    for (int j = 0; j < 6; ++j)
      csv << cbf_res.tau_cbf(j) << ",";

    const Eigen::Vector3d f_ext_lin = F_ext.head<3>();
    const Eigen::Vector3d f_cbf_lin = cbf_res.F_cart_lin;
    csv << f_ext_lin.x() << "," << f_ext_lin.y() << "," << f_ext_lin.z() << "," << f_ext_lin.norm() << ","
        << f_cbf_lin.x() << "," << f_cbf_lin.y() << "," << f_cbf_lin.z() << "," << f_cbf_lin.norm() << "\n";
  }

  csv.close();
  std::cout << "Saved simulation trajectory data to: " << csv_path.string()
            << " (" << n_steps << " points)\n";
  return 0;
}
