#pragma once

#include "cbf_common/hyperplane.hpp"
#include "cbf_common/profiles.hpp"
#include "cbf_common/quadprog.hpp"

// Simulink Coder builds pass -DRT on the command line, which collides with the template
// parameter named `RT` in Boost.Math (pulled in by Pinocchio). Hide it while including
// Pinocchio/Boost and restore it afterwards.
#pragma push_macro("RT")
#undef RT
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma pop_macro("RT")

#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <vector>

namespace jtorque_cbf
{

  using cbf::Hyperplane;
  using cbf::ProfileType;

  /**
   * @brief Configuration of the joint-space torque CBF controller.
   *
   * Decision variable: the CBF joint torque tau_cbf itself (instead of a normal force f_n with
   * tau_cbf = J^T N f_n as in torque_cbf). This lets the QP use exactly the torques the robot can
   * reach under a per-joint torque-rate limit. The properties of the normal-force design are kept as
   * explicit constraints (passivity, one-way normal force, bounded lateral force), see the class doc.
   */
  struct JointTorqueCBFConfig
  {
    // --- Barrier (same meaning as torque_cbf::TorqueCBFConfig) ---
    double p_ths = 0.20;                           ///< Distance activation threshold [m] (p_ths > 0)
    double v_ths = -0.002;                         ///< Target contact velocity threshold [m/s] (v_ths < 0)
    double v_0 = -0.35;                            ///< Nominal entry approach velocity [m/s] (v_0 < v_ths < 0)
    double gamma = 15.0;                           ///< CBF class-K decay gain [1/s] (gamma > 0)
    double w_slack = 1e3;                          ///< CBF slack quadratic penalty
    double w_lin = 1e4;                            ///< CBF slack linear penalty
    bool latch_entry_velocity = false;             ///< One-sided entry latch: v_0 <- min(dzeta_entry, v_0)
    double zone_exit_hysteresis = 0.0;             ///< Latch zone left only at zeta > p_ths + hysteresis [m]
    double latch_velocity_filter_alpha = 1.0;      ///< EMA coefficient (0, 1] of the latched entry velocity
    bool compensate_external_force = true;         ///< Include Lambda^-1 * F_ext in the drift
    ProfileType profile = ProfileType::SMOOTHSTEP; ///< Velocity envelope shaping function

    double dt = 0.001; ///< Control period [s]

    // --- Guarantees of the normal-force design, kept as constraints ---
    bool enforce_passivity = true;       ///< tau_cbf^T dq <= 0 (the CBF never does positive work) and one-way
                                         ///< normal action 0 <= f_eq,i <= cap_i (cap_i = 0 while separating), with
                                         ///< f_eq,i = rho_i n_i^T J M^-1 tau the normal-EQUIVALENT force: the normal
                                         ///< acceleration tau produces (also through lateral forces / moments) times rho_i
    bool brake_cap_include_drift = true; ///< Cap also allows cancelling the approaching drift (as torque_cbf)
    double max_lateral_force = 5.0;      ///< |component of (I - N N^+) F_lin| <= this [N]   (0 = pure normal force)
    double max_moment = 0.5;             ///< |component of F_rot| <= this [Nm]               (0 = no moment)

    // --- Cost: weights of the end-effector force produced by tau_cbf, F = Lambda J M^-1 tau ---
    double w_lateral = 1e4; ///< Weight of the lateral force (normal-force weight is 1)
    double w_moment = 1e4;  ///< Weight of the moment
    double w_null = 1e5;    ///< Weight of the nullspace torque (I - J^T Jbar^T) tau (no effect on the elbow)
    double w_reg = 1e-6;    ///< Small Tikhonov term on tau (strict convexity)

    /// Legacy compatibility field (unused: guarantee constraints are hard and only relaxed by the exact
    /// hardware decay torque tau_zero = clamp(0, lo, hi) when 0 is outside the rate-limit box).
    double guarantee_slack_factor = 1e3;

    // --- Actuator limits (hard: they describe what the robot can apply) ---
    Eigen::VectorXd tau_min;                  ///< Joint torque limits [Nm] (size n, optional)
    Eigen::VectorXd tau_max;                  ///< Joint torque limits [Nm] (size n, optional)
    double default_tau_limit = 150.0;         ///< Symmetric fallback limit if tau_min/tau_max are empty
    bool enforce_total_torque_limits = false; ///< Limits on g + tau_nom + tau_cbf (relaxed to contain tau_cbf = 0)

    /// Per-joint torque-rate limit of the robot [Nm/s] (Franka: 1000), applied to the TOTAL commanded
    /// torque tau_nom + tau_cbf (gravity excluded, the robot adds it). 0 = disabled.
    /// tau_cbf is then boxed to exactly the torques reachable in one step:
    ///   |tau_nom + tau_cbf - (tau_nom + tau_cbf)_prev| <= tau_rate_limit * dt   (per joint)
    double tau_rate_limit = 0.0;
  };

  /**
   * @brief Step result and telemetry.
   */
  struct JointTorqueCBFResult
  {
    bool is_active = false;    ///< True if any plane barrier row is binding-relevant (b_cbf > 0)
    bool qp_success = false;   ///< QP converged
    double qp_cost = 0.0;      ///< Optimal QP objective
    Eigen::VectorXd tau_cbf;   ///< CBF joint torque [Nm]
    Eigen::VectorXd tau_cmd;   ///< g + tau_nom + tau_cbf [Nm]
    double slack_delta = 0.0;  ///< CBF slack (barrier relaxed by this much) [m/s^2]
    double sigma_normal = 0.0;  ///< One-way normal action relaxed by this much [N]
    double sigma_power = 0.0;   ///< Passivity relaxed by this much [W]
    double sigma_lateral = 0.0; ///< Lateral force / moment bound relaxed by this much [N or Nm]
    bool rate_limited = false; ///< True if a joint rate bound is active at the solution

    // End-effector force produced by tau_cbf: F = Lambda J M^-1 tau_cbf (same acceleration as F at the EE)
    Eigen::Matrix<double, 6, 1> F_cbf = Eigen::Matrix<double, 6, 1>::Zero(); ///< [N, Nm]
    Eigen::VectorXd f_n;        ///< Normal-equivalent force rho_i n_i^T J M^-1 tau_cbf per plane [N]
    double f_n_scalar = 0.0;    ///< Critical plane normal-equivalent force [N]
    double F_lateral = 0.0;     ///< Max |lateral force component| [N]
    double M_norm = 0.0;        ///< Max |moment component| [Nm]
    double power = 0.0;         ///< CBF power tau_cbf^T dq [W] (<= 0: dissipative)
    Eigen::VectorXd f_brake_max; ///< Normal-force caps per plane [N]
    Eigen::VectorXd rho;         ///< Effective inertia along each normal [kg]

    // Per-plane telemetry
    Eigen::VectorXd zetas, dzetas, v_mins, barriers, b_cbf, a_drift;
    std::vector<bool> active_planes;

    // Critical plane telemetry
    int critical_plane_idx = -1;
    double zeta = 0.0, dzeta = 0.0, v_min = 0.0, dv_min_dzeta = 0.0, barrier_B = 0.0;
    double b_cbf_scalar = 0.0, a_drift_scalar = 0.0;
  };

  /**
   * @brief Robot state at one control cycle (same layout as torque_cbf::RobotState).
   */
  struct RobotState
  {
    Eigen::VectorXd q, dq;
    Eigen::Vector3d x = Eigen::Vector3d::Zero();  ///< End-effector position
    Eigen::Vector3d dx = Eigen::Vector3d::Zero(); ///< End-effector linear velocity
    Eigen::MatrixXd M, C;                         ///< Joint inertia and Coriolis matrices
    Eigen::VectorXd g;                            ///< Gravity torque
    Eigen::MatrixXd J, dJ;                        ///< Geometric Jacobian (6 x n, [lin; ang]) and its TRUE time derivative
    Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero(); ///< External wrench
  };

  /**
   * @class JointTorqueCBFController
   * @brief Joint-space torque CBF: tau_cbf is the QP decision variable.
   *
   *   min_{tau, delta}  1/2 F^T W_F F + 1/2 w_null |N_tau tau|^2 + 1/2 w_reg |tau|^2
   *                     + 1/2 w_slack delta^2 + w_lin delta
   *   with F = Lambda J M^-1 tau (EE force producing the same acceleration), W_F = 1 along the normals,
   *   w_lateral laterally, w_moment on moments.
   *
   *   s.t. barrier (per plane in zone):   n_i^T J M^-1 tau + delta >= min(b_cbf,i, cap_i / rho_i)
   *        one-way normal action:         0 <= f_eq,i <= cap_i,  f_eq,i = rho_i n_i^T J M^-1 tau
   *        passivity:                     -dq^T tau >= 0
   *        bounded lateral force/moment:  |(P_lat F_lin)_k| <= eps_lat,  |F_rot,k| <= eps_rot
   *        actuator (hard box):           torque limits  and  per-joint rate limit
   *
   * Whenever tau = 0 is inside the actuator box [lo, hi], all guarantee constraints are hard. When the
   * rate limit traps tau away from 0, any guarantee row violated at tau_zero = clamp(0, lo, hi) is relaxed
   * to its value at tau_zero so the QP stays strictly feasible without extra slack variables.
   */
  class JointTorqueCBFController
  {
  public:
    explicit JointTorqueCBFController(const JointTorqueCBFConfig &config = JointTorqueCBFConfig(),
                                      const Hyperplane &plane = Hyperplane());
    JointTorqueCBFController(const JointTorqueCBFConfig &config, const std::vector<Hyperplane> &planes);

    /// Does not reset the entry-latch or rate state (safe to call every cycle)
    void setConfig(const JointTorqueCBFConfig &config);
    const JointTorqueCBFConfig &getConfig() const { return config_; }

    /// Latch and rate state are reset only if the (normalized) planes change
    void setHyperplane(const Hyperplane &plane);
    void setHyperplanes(const std::vector<Hyperplane> &planes);
    const std::vector<Hyperplane> &getHyperplanes() const { return planes_; }

    void setCustomProfile(std::function<double(double)> h_func, std::function<double(double)> H_func);
    void resetEntryState();
    /// Forget the previously commanded torque (rate limit then starts from tau_nom + 0)
    void resetRateState();

    double computeVelocityEnvelope(double zeta, std::size_t plane_idx = 0) const;
    double computeVelocityEnvelopeDerivative(double zeta, std::size_t plane_idx = 0) const;

    /// tau_c = g + tau_nom + tau_cbf from a RobotState (dJ must be the true time derivative of J)
    Eigen::VectorXd computeControlTorque(const RobotState &state,
                                         const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
                                         JointTorqueCBFResult *result_out = nullptr);

    /// tau_c = g + tau_nom + tau_cbf with Pinocchio (LOCAL_WORLD_ALIGNED frame Jacobian, classical dJ)
    Eigen::VectorXd computeControlTorque(const pinocchio::Model &model, pinocchio::Data &data,
                                         const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                         pinocchio::FrameIndex frame_id,
                                         const Eigen::Matrix<double, 6, 1> &F_ext = Eigen::Matrix<double, 6, 1>::Zero(),
                                         const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
                                         JointTorqueCBFResult *result_out = nullptr);

    /// Core QP (drift_cart = dJ dq - J M^-1 C dq, the uncompensated Cartesian drift)
    JointTorqueCBFResult solveQP(const Eigen::MatrixXd &M, const Eigen::MatrixXd &J,
                                 const Eigen::Matrix<double, 6, 1> &drift_cart,
                                 const Eigen::Matrix<double, 6, 1> &F_ext,
                                 const Eigen::Vector3d &x_ee, const Eigen::Vector3d &dx_ee,
                                 const Eigen::VectorXd &dq,
                                 const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
                                 const Eigen::VectorXd &g = Eigen::VectorXd());

  private:
    void assignPlanes(std::vector<Hyperplane> new_planes);
    void ensureEntryStateSize();
    void updateActiveV0(std::size_t i);

    JointTorqueCBFConfig config_;
    std::vector<Hyperplane> planes_;
    std::vector<double> active_v0_;
    std::vector<bool> was_in_zone_;
    std::vector<bool> has_entry_velocity_;
    std::vector<double> entry_dzeta_;
    std::vector<bool> dzeta_filt_init_;
    std::vector<double> dzeta_filt_;
    Eigen::VectorXd tau_total_prev_; ///< tau_nom + tau_cbf commanded at the previous call (rate limit)
    bool has_tau_prev_ = false;
    std::function<double(double)> custom_h_;
    std::function<double(double)> custom_H_;
  };

} // namespace jtorque_cbf
