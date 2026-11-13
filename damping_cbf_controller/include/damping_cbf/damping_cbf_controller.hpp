#pragma once

#include "cbf_common/hyperplane.hpp"
#include "cbf_common/profiles.hpp"
#include "cbf_common/quadprog.hpp"
#include "damping_cbf/quadprog.hpp"

#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace damping_cbf
{

  using ProfileType = cbf::ProfileType;
  using Hyperplane = cbf::Hyperplane;

  /**
   * @brief Configuration parameters for the CBF-based variable damping controller.
   */
  struct ControllerConfig
  {
    double p_ths = 0.20;                           ///< Distance activation threshold [m] (p_ths > 0)
    double v_ths = -0.002;                         ///< Target (maximum magnitude) contact velocity [m/s] (v_ths < 0)
    double v_0 = -0.35;                            ///< Nominal entry velocity at zeta = p_ths [m/s] (v_0 < v_ths < 0)
    double gamma = 15.0;                           ///< CBF class-K decay gain [1/s] (gamma > 0)
    double d_max = 300.0;                          ///< Maximum allowable damping factor [Ns/m]
    double d_max_inertia_gain = 0.0;               ///< kappa [1/s]: inertia-dependent damping limit
                                                   ///< d_max_eff = min(d_max, kappa * rho), rho = effective inertia
                                                   ///< along the plane normal. Caps the damping bandwidth d/rho <= kappa
                                                   ///< (sampled-data stability). 0 = disabled (fixed d_max)
    double slack_linear_penalty = 1e6;             ///< Exact L1 slack penalty weight in 2D QP
    double slack_quadratic_penalty = 1e4;          ///< Quadratic L2 slack penalty weight in 2D QP
    bool latch_entry_velocity = false;             ///< If true, one-sided latch on zone entry: v_0 <- min(dzeta_entry, v_0),
                                                   ///< i.e. only entries FASTER than v_0 reshape the envelope (B = 0 at entry,
                                                   ///< smooth onset); slower entries keep v_0 (no CBF damping at entry)
    double zone_exit_hysteresis = 0.0;             ///< Zone is left only when zeta > p_ths + zone_exit_hysteresis [m] (>= 0);
                                                   ///< avoids re-latching when hovering around p_ths
    double latch_velocity_filter_alpha = 1.0;      ///< EMA coefficient in (0, 1] for the velocity sampled by the latch
                                                   ///< (1 = unfiltered; e.g. 0.2 at 1 kHz ~ 5 ms time constant).
                                                   ///< Affects ONLY the latched value, never the QP velocity
    bool compensate_external_force = true;         ///< If true, includes external wrench F_ext in a_drift (if false, only nominal impedance forces are included)
    double tau_rate_limit = 0.0;                   ///< Actuator torque-rate budget for the CBF damping torque [Nm/s] (e.g. 900 with
                                                   ///< the Franka 1000 Nm/s limit). 0 = disabled. When enabled the normal damping
                                                   ///< force f = d*|dzeta| may INCREASE by at most r*dt per call above the force the
                                                   ///< rate-limited robot still applies, r = tau_rate_limit / max_j |(J^T n)_j| [N/s].
                                                   ///< Decreases are not constrained (a lingering damping force only dissipates more).
                                                   ///< Single-plane solvers only
    double dt = 0.001;                             ///< Control period [s] used by the rate limit
    ProfileType profile = ProfileType::SMOOTHSTEP; ///< Shaping function h(u)

    /// Cartesian compliance gains (Eq. 3)
    Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
  };

  /**
   * @brief Detailed telemetry returned by a single evaluation of the CBF-QP damping law (single or multi-surface).
   */
  struct CBFStepResult
  {
    bool is_active = false;                          ///< True if inside braking zone and approaching surface
    double zeta = 0.0;                               ///< Orthogonal distance to hyperplane [m] (or most critical plane)
    double dzeta = 0.0;                              ///< Normal velocity along n_hp [m/s]
    double rho = 0.0;                                ///< Projected effective Cartesian inertia [kg]
    double v_min = 0.0;                              ///< Safe velocity lower bound v_min(zeta) [m/s]
    double dv_min_dzeta = 0.0;                       ///< Spatial derivative v_min'(zeta) [1/s]
    double barrier_B = 0.0;                          ///< CBF value B(zeta, dzeta) = dzeta - v_min(zeta) [m/s]
    double Lf_B = 0.0;                               ///< Drift Lie derivative L_f B = a_drift - v_min'(zeta)*dzeta [m/s^2]
    double Lg_B = 0.0;                               ///< Control Lie derivative L_g B = -dzeta / rho [1/(kg*s)]
    double d_max_eff = 0.0;                          ///< Damping upper bound used by the QP: min(d_max, kappa*rho) [Ns/m]
    double f_applied_est = 0.0;                      ///< Estimated damping force still applied by the rate-limited robot [N]
    double d_rate_max = 0.0;                         ///< Ramp-up upper bound on d [Ns/m]
    double f_rate = 0.0;                             ///< Normal-force rate limit r used [N/s] (inf if disabled)
    bool rate_limited = false;                       ///< True if the ramp-up bound is active at the solution
    double d_cbf = 0.0;                              ///< Optimal QP damping factor in [0, d_max_eff] [Ns/m] (primary / critical plane)
    double slack_eps = 0.0;                          ///< Optimal QP relaxation slack variable eps >= 0 [m/s^2]
    double qp_cost = 0.0;                            ///< Optimal objective value returned by solve_quadprog
    Eigen::Vector2d z_opt = Eigen::Vector2d::Zero(); ///< 2D QP solution vector z* = [d_cbf, slack_eps]^T
    double dB_dt = 0.0;                              ///< Closed-loop barrier time derivative dB/dt [m/s^2]
    double f_imp_scalar = 0.0;                       ///< Scalar damping force magnitude ||F_imp|| [N]
    Eigen::Vector3d F_imp = Eigen::Vector3d::Zero(); ///< 3D damping force sum n_i * f_IMP_i [N]
    Eigen::Matrix3d D_imp = Eigen::Matrix3d::Zero(); ///< 3x3 PSD damping matrix sum d_i * n_i * n_i^T [Ns/m]

    // Multi-surface per-plane telemetry (size m)
    Eigen::VectorXd d_opt;                           ///< Optimal per-plane damping coefficients in R^m
    Eigen::VectorXd zetas;                           ///< Orthogonal distances to each hyperplane in R^m
    Eigen::VectorXd dzetas;                          ///< Normal velocities along each hyperplane in R^m
    Eigen::VectorXd v_mins;                          ///< Velocity bounds for each hyperplane in R^m
    Eigen::VectorXd barriers;                        ///< Barrier values for each hyperplane in R^m
  };

  /**
   * @brief Full-state robot model data at a single control cycle for Cartesian compliance control.
   */
  struct RobotState
  {
    Eigen::VectorXd q;                                                       ///< Joint positions in R^n
    Eigen::VectorXd dq;                                                      ///< Joint velocities in R^n
    Eigen::Vector3d x = Eigen::Vector3d::Zero();                             ///< End-effector position in R^3
    Eigen::Vector3d phi = Eigen::Vector3d::Zero();                           ///< End-effector orientation (Euler/axis-angle) in R^3
    Eigen::Vector3d dx = Eigen::Vector3d::Zero();                            ///< End-effector linear velocity in R^3
    Eigen::Vector3d dphi = Eigen::Vector3d::Zero();                          ///< End-effector angular velocity in R^3
    Eigen::MatrixXd M;                                                       ///< Joint inertia matrix M(q) in R^{n x n}
    Eigen::MatrixXd C;                                                       ///< Joint Coriolis matrix C(q, dq) in R^{n x n}
    Eigen::VectorXd g;                                                       ///< Joint gravity torque vector g(q) in R^n
    Eigen::MatrixXd J;                                                       ///< Geometric Jacobian J(q) in R^{6 x n}
    Eigen::MatrixXd dJ;                                                      ///< Jacobian time derivative dJ(q, dq) in R^{6 x n}
    Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero(); ///< External wrench in R^6
  };

  /**
   * @class DampingCBFController
   * @brief Implements a Control Barrier Function (CBF) variable-damping Cartesian compliance controller
   *        supporting single and multi-hyperplane constraints via Goldfarb-Idnani QP optimization.
   */
  class DampingCBFController
  {
  public:
    /// Constructor for a single preloaded hyperplane
    explicit DampingCBFController(const ControllerConfig &config = ControllerConfig(),
                                  const Hyperplane &plane = Hyperplane());

    /// Constructor for multiple preloaded hyperplanes
    explicit DampingCBFController(const ControllerConfig &config,
                                  const std::vector<Hyperplane> &planes);

    /// Update controller configuration.
    /// Does NOT reset the entry-latch state, so it is safe to call every control cycle
    /// (e.g. from an S-function with tunable parameters). Use resetEntryState() to reset it.
    void setConfig(const ControllerConfig &config);
    const ControllerConfig &getConfig() const { return config_; }

    /// Hyperplane setters and getters
    void setHyperplane(const Hyperplane &plane);
    void setHyperplanes(const std::vector<Hyperplane> &planes);
    const Hyperplane &getHyperplane() const { return plane_; }
    const std::vector<Hyperplane> &getHyperplanes() const { return planes_; }

    /// Set custom profile functions h(u) and H(u) = int_0^u h(s) ds
    void setCustomProfile(std::function<double(double)> h_func,
                          std::function<double(double)> H_func);

    /// Reset any latched entry velocity state
    void resetEntryState();

    /// Forget the previously applied damping force (rate limit then starts from zero force)
    void resetRateState();

    /// Normal-force rate limit [N/s] for a joint torque-rate limit tau_rate [Nm/s]:
    /// r = tau_rate / max_j |(J^T n6)_j|  (infinity if tau_rate <= 0)
    static double computeNormalForceRate(const Eigen::MatrixXd &J, const Eigen::Vector3d &normal,
                                         double tau_rate);

    /// Evaluate normalized profile h(u) and antiderivative H(u) for u in [0, 1]
    double evaluateProfileH(double u) const;
    double evaluateAntiderivativeH(double u) const;

    /// Kinematic & inertial hyperplane projections
    double computeDistance(const Eigen::Vector3d &x) const;
    double computeNormalVelocity(const Eigen::Vector3d &dx) const;
    static double computeEffectiveInertia(const Eigen::Matrix<double, 6, 6> &Lambda,
                                          const Eigen::Vector3d &normal);

    /// Compute task-space inertia Lambda(q) and Coriolis matrix Gamma(q, dq) (Eq. 2)
    static Eigen::Matrix<double, 6, 6> computeCartesianInertia(const Eigen::MatrixXd &M,
                                                               const Eigen::MatrixXd &J);
    static Eigen::Matrix<double, 6, 6> computeCartesianCoriolis(
        const Eigen::MatrixXd &M, const Eigen::MatrixXd &C,
        const Eigen::MatrixXd &J, const Eigen::MatrixXd &dJ,
        const Eigen::Matrix<double, 6, 6> &Lambda);

    /// Evaluate the safe velocity boundary v_min(zeta) and its spatial derivative v_min'(zeta)
    double computeVelocityEnvelope(double zeta) const;
    double computeVelocityEnvelopeDerivative(double zeta) const;

    /// Evaluate the Control Barrier Function B(zeta, dzeta) = dzeta - v_min(zeta)
    double computeBarrierValue(double zeta, double dzeta) const;

    /// Damping upper bound for effective inertia rho: min(d_max, d_max_inertia_gain * rho)
    /// (d_max if d_max_inertia_gain <= 0)
    double computeEffectiveDMax(double rho) const;

    // -------------------------------------------------------------------------
    // Aligned High-Level Torque Computation
    // Both accept ONLY parameters that cannot be computed internally or preloaded
    // -------------------------------------------------------------------------

    /**
     * @brief Compute full joint control torque vector tau in R^n (Eq. 3) for preloaded hyperplane(s):
     *        If multiple hyperplanes were preloaded, computes multi-surface CBF torque;
     *        otherwise computes single-surface CBF torque.
     *
     * @param state        Current robot state (q, dq, x, dx, phi, dphi, M, C, g, J, dJ, F_ext)
     * @param x_ref        Target Cartesian position in R^3
     * @param phi_ref      Target Cartesian orientation in R^3
     * @param tau_null     Nullspace posture torque vector in R^n (optional, default zero)
     * @param cbf_result_out Optional telemetry output pointer
     * @return Commanded joint torque vector tau in R^n
     */
    Eigen::VectorXd computeControlTorque(
        const RobotState &state,
        const Eigen::Vector3d &x_ref,
        const Eigen::Vector3d &phi_ref,
        const Eigen::VectorXd &tau_null = Eigen::VectorXd(),
        CBFStepResult *cbf_result_out = nullptr);

    /**
     * @brief Compute full joint control torque vector tau in R^n for multiple preloaded hyperplanes.
     *        Accepts the exact same parameters as computeControlTorque.
     *
     * @param state        Current robot state (q, dq, x, dx, phi, dphi, M, C, g, J, dJ, F_ext)
     * @param x_ref        Target Cartesian position in R^3
     * @param phi_ref      Target Cartesian orientation in R^3
     * @param tau_null     Nullspace posture torque vector in R^n (optional, default zero)
     * @param cbf_result_out Optional telemetry output pointer
     * @return Commanded joint torque vector tau in R^n
     */
    Eigen::VectorXd computeMultiSurfaceControlTorque(
        const RobotState &state,
        const Eigen::Vector3d &x_ref,
        const Eigen::Vector3d &phi_ref,
        const Eigen::VectorXd &tau_null = Eigen::VectorXd(),
        CBFStepResult *cbf_result_out = nullptr) const;

    /**
     * @brief Legacy multi-surface torque computation with explicit planes vector.
     */
    Eigen::VectorXd computeMultiSurfaceControlTorque(
        const RobotState &state,
        const std::vector<Hyperplane> &planes,
        const Eigen::Vector3d &x_ref,
        const Eigen::Vector3d &phi_ref,
        const Eigen::VectorXd &tau_null,
        Eigen::Matrix3d *D_imp_out = nullptr,
        Eigen::VectorXd *d_opt_out = nullptr) const;

    // -------------------------------------------------------------------------
    // Aligned QP Solvers (from RobotState)
    // -------------------------------------------------------------------------

    /**
     * @brief Solve single-plane CBF-QP from robot state.
     *        Computes Lambda, drift, zeta, dzeta, rho internally from preloaded plane.
     */
    CBFStepResult solveDampingQP(
        const RobotState &state,
        CBFStepResult *cbf_result_out = nullptr);

    /**
     * @brief Solve multi-surface CBF-QP from robot state using preloaded hyperplanes.
     *        Accepts the exact same parameters as solveDampingQP.
     */
    CBFStepResult solveMultiSurfaceDampingQP(
        const RobotState &state,
        CBFStepResult *cbf_result_out = nullptr) const;

    // -------------------------------------------------------------------------
    // Aligned QP Solvers (from Cartesian Operational-Space Kinematics)
    // -------------------------------------------------------------------------

    /**
     * @brief Solve single-plane CBF-QP from operational-space state (x, dx, Lambda, F_ext).
     */
    CBFStepResult solveDampingQP(
        const Eigen::Vector3d &x,
        const Eigen::Vector3d &dx,
        const Eigen::Matrix<double, 6, 6> &Lambda,
        const Eigen::Matrix<double, 6, 1> &F_ext = Eigen::Matrix<double, 6, 1>::Zero(),
        CBFStepResult *cbf_result_out = nullptr);

    /**
     * @brief Solve multi-surface CBF-QP from operational-space state using preloaded hyperplanes.
     *        Accepts the exact same parameters as solveDampingQP.
     */
    CBFStepResult solveMultiSurfaceDampingQP(
        const Eigen::Vector3d &x,
        const Eigen::Vector3d &dx,
        const Eigen::Matrix<double, 6, 6> &Lambda,
        const Eigen::Matrix<double, 6, 1> &F_ext = Eigen::Matrix<double, 6, 1>::Zero(),
        CBFStepResult *cbf_result_out = nullptr) const;

    // -------------------------------------------------------------------------
    // Low-Level / Legacy Solvers
    // -------------------------------------------------------------------------

    /**
     * @brief Low-level scalar 2D CBF-QP solver (pre-projected normal coordinates).
     * @param normal_force_rate  Rate limit r of the normal damping force [N/s] (see computeNormalForceRate);
     *                           used only if config.tau_rate_limit > 0. Infinity = no rate limit.
     */
    CBFStepResult solveDampingQP(
        double zeta, double dzeta, double rho, double a_drift = 0.0,
        double normal_force_rate = std::numeric_limits<double>::infinity(),
        const Eigen::MatrixXd &A_extra = Eigen::MatrixXd(0, 2),
        const Eigen::VectorXd &b_extra = Eigen::VectorXd(0));

    /**
     * @brief General n-D CBF-QP solver using Goldfarb-Idnani (solve_quadprog_box).
     */
    Eigen::VectorXd solveGeneralCBFQP(
        const Eigen::MatrixXd &G,
        const Eigen::VectorXd &g0,
        const Eigen::MatrixXd &A_cbf,
        const Eigen::VectorXd &b_cbf,
        const Eigen::VectorXd &lb,
        const Eigen::VectorXd &ub,
        double *optimal_cost_out = nullptr) const;

    /**
     * @brief Legacy multi-surface CBF-QP solver with explicit planes argument.
     */
    Eigen::Matrix3d solveMultiSurfaceDampingQP(
        const std::vector<Hyperplane> &planes,
        const Eigen::Vector3d &x,
        const Eigen::Vector3d &dx,
        const Eigen::Matrix<double, 6, 6> &Lambda,
        const Eigen::Matrix<double, 6, 1> &F_ext,
        Eigen::VectorXd *d_opt_out = nullptr) const;

  private:
    /// Recompute active_v0_ from config_ and the latched entry velocity
    void updateActiveV0();

    ControllerConfig config_;
    Hyperplane plane_;
    std::vector<Hyperplane> planes_;
    double active_v0_ = -0.35;          ///< Envelope entry velocity currently in use
    bool was_in_zone_ = false;          ///< Zone membership at the previous call (with exit hysteresis)
    bool has_entry_velocity_ = false;   ///< True while inside the zone after an entry was recorded
    double entry_dzeta_ = 0.0;          ///< (Filtered) normal velocity recorded at zone entry
    bool dzeta_filt_init_ = false;      ///< Latch-velocity filter initialised
    double dzeta_filt_ = 0.0;           ///< Latch-velocity filter state
    double f_prev_ = 0.0;               ///< Estimated normal damping force applied after the previous call [N]
    std::function<double(double)> custom_h_;
    std::function<double(double)> custom_H_;
  };

} // namespace damping_cbf
