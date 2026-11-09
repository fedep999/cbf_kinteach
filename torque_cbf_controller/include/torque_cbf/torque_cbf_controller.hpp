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
#include <string>
#include <vector>

namespace torque_cbf
{

  using cbf::Hyperplane;
  using cbf::ProfileType;

  /**
   * @brief Configuration parameters for the Direct Joint-Torque CBF Controller.
   *
   * Formulated using Approach A (Cartesian Force Parameterization with One-Way Dissipation):
   * tau_cbf = J(q)^T * N_6 * f_n, with f_n >= 0 along hyperplane normals.
   */
  struct TorqueCBFConfig
  {
    double p_ths = 0.20;                           ///< Distance activation threshold [m] (p_ths > 0)
    double v_ths = -0.002;                         ///< Target contact velocity threshold [m/s] (v_ths < 0)
    double v_0 = -0.35;                            ///< Nominal entry approach velocity [m/s] (v_0 < v_ths < 0)
    double gamma = 15.0;                           ///< CBF class-K decay gain [1/s] (gamma > 0)
    double w_slack = 1e4;                          ///< Slack quadratic penalty weight w_delta
    double w_lin = 1e6;                            ///< Slack linear penalty weight w_lin
    bool latch_entry_velocity = false;             ///< If true, one-sided latch on zone entry: v_0 <- min(dzeta_entry, v_0),
                                                   ///< i.e. only entries FASTER than v_0 reshape the envelope (B = 0 at entry,
                                                   ///< smooth onset); slower entries keep v_0 (no CBF action at entry)
    double zone_exit_hysteresis = 0.0;             ///< Latch zone is left only when zeta > p_ths + zone_exit_hysteresis [m] (>= 0);
                                                   ///< avoids re-latching when hovering around p_ths
    double latch_velocity_filter_alpha = 1.0;      ///< EMA coefficient in (0, 1] for the velocity sampled by the latch
                                                   ///< (1 = unfiltered; e.g. 0.2 at 1 kHz ~ 5 ms time constant).
                                                   ///< Affects ONLY the latched value, never the QP velocity
    bool compensate_external_force = true;         ///< If true, includes F_ext in a_drift; if false, neglects external force impact on a_drift
    ProfileType profile = ProfileType::SMOOTHSTEP; ///< Velocity shaping profile function h(u)

    /// Kinesthetic teaching / passivity parameters
    double dt = 0.001;             ///< Control cycle period [s] for velocity braking cap (rho/dt)*|dzeta|
    bool enforce_passivity = true; ///< If true, caps f_n to prevent pushback:
                                   ///< f_n <= rho*(|dzeta|/dt + max(0,-a_drift)) if dzeta <= 0, else 0
    bool brake_cap_include_drift = true; ///< If true, the cap also allows cancelling the approaching drift
                                         ///< acceleration (e.g. the estimated human push) within the cycle, so the
                                         ///< robot can be held at v_ths / at rest instead of creeping at a_drift*dt.
                                         ///< If false: legacy cap f_n <= (rho/dt)*|dzeta|

    /// Joint torque limits in Nm (optional, size n)
    Eigen::VectorXd tau_min;
    Eigen::VectorXd tau_max;
    double default_tau_limit = 150.0;         ///< Default symmetric limit [-limit, limit] if tau_min/max empty
    bool enforce_total_torque_limits = false; ///< If true, bounds g + tau_nom + tau_cbf in [tau_min, tau_max]
                                              ///< (bounds on tau_cbf relaxed to always contain 0, so the QP stays
                                              ///< feasible when g + tau_nom alone is already out of limits)
  };

  /**
   * @brief Step result and telemetry returned by the Direct Joint-Torque CBF Controller.
   */
  struct TorqueCBFResult
  {
    bool is_active = false;   ///< True if any plane barrier constraint is active (b_cbf > 0)
    bool qp_success = false;  ///< True if QP solver converged to a finite optimal solution
    double qp_cost = 0.0;     ///< Optimal QP objective value
    Eigen::VectorXd tau_cbf;  ///< Optimal CBF joint torque vector tau_cbf* in R^n
    Eigen::VectorXd tau_cmd;  ///< Total commanded joint torque tau_c = g(q) + tau_nom + tau_cbf* in R^n
    double slack_delta = 0.0; ///< Optimal slack variable delta* >= 0

    // Approach A Cartesian normal force telemetry
    Eigen::VectorXd f_n;                                  ///< Optimal normal scalar forces f_n,i >= 0 in R^m [N]
    double f_n_scalar = 0.0;                              ///< Critical plane normal force [N]
    Eigen::Vector3d F_cart_lin = Eigen::Vector3d::Zero(); ///< Cartesian linear force vector at EE: F_cart = sum(n_i * f_n,i) [N]
    Eigen::Vector3d F_cart_lat = Eigen::Vector3d::Zero(); ///< Lateral Cartesian force component (strictly zero) [N]
    Eigen::VectorXd rho;                                  ///< Operational space effective inertia along normals rho_i in R^m [kg]
    Eigen::VectorXd f_brake_max;                          ///< One-way velocity braking cap values in R^m [N]

    // Multi-surface per-plane telemetry (size m)
    Eigen::VectorXd zetas;           ///< Orthogonal distances to each hyperplane [m]
    Eigen::VectorXd dzetas;          ///< Normal approach velocities along each hyperplane [m/s]
    Eigen::VectorXd v_mins;          ///< Velocity envelope lower bounds v_min(zeta_i) [m/s]
    Eigen::VectorXd barriers;        ///< Barrier values B_i = dzeta_i - v_min(zeta_i) [m/s]
    Eigen::VectorXd b_cbf;           ///< Constraint lower bounds b_cbf,i
    Eigen::VectorXd a_drift;         ///< Acceleration drift terms a_drift,i [m/s^2]
    std::vector<bool> active_planes; ///< Activity flag for each plane

    // Representative telemetry (most critical plane)
    int critical_plane_idx = -1; ///< Index of most critical plane (maximum b_cbf)
    double zeta = 0.0;           ///< Distance to critical plane [m]
    double dzeta = 0.0;          ///< Velocity along critical plane normal [m/s]
    double v_min = 0.0;          ///< Critical plane v_min [m/s]
    double dv_min_dzeta = 0.0;   ///< Spatial derivative v_min'(zeta) [1/s]
    double barrier_B = 0.0;      ///< Critical plane barrier value [m/s]
    double b_cbf_scalar = 0.0;   ///< Critical plane b_cbf
    double a_drift_scalar = 0.0; ///< Critical plane a_drift [m/s^2]
  };

  /**
   * @brief Full-state robot dynamic model data at a single control cycle.
   */
  struct RobotState
  {
    Eigen::VectorXd q;                                                       ///< Joint positions in R^n
    Eigen::VectorXd dq;                                                      ///< Joint velocities in R^n
    Eigen::Vector3d x = Eigen::Vector3d::Zero();                             ///< End-effector position in R^3
    Eigen::Vector3d dx = Eigen::Vector3d::Zero();                            ///< End-effector linear velocity in R^3
    Eigen::MatrixXd M;                                                       ///< Joint inertia matrix M(q) in R^{n x n}
    Eigen::MatrixXd C;                                                       ///< Joint Coriolis matrix C(q, dq) in R^{n x n}
    Eigen::VectorXd g;                                                       ///< Joint gravity torque vector g(q) in R^n
    Eigen::MatrixXd J;                                                       ///< Geometric Jacobian J(q) in R^{6 x n}
    Eigen::MatrixXd dJ;                                                      ///< Jacobian time variation dJ(q, dq) in R^{6 x n}
    Eigen::Matrix<double, 6, 1> F_ext = Eigen::Matrix<double, 6, 1>::Zero(); ///< External wrench in R^6
  };

  /**
   * @class TorqueCBFController
   * @brief Direct Joint-Torque Control Barrier Function (CBF) Controller for an n-DOF manipulator
   *        with Cartesian Force Parameterization & One-Way Dissipation (Approach A for Kinesthetic Teaching).
   *
   * Joint torques are strictly synthesized along hyperplane Cartesian normals:
   *   tau_cbf = sum_{i=1}^m J(q)^T * n_{6,i} * f_{n,i} = J(q)^T * N_6 * f_n
   *
   * Where f_{n,i} >= 0 are scalar normal braking forces determined by a fast (m+1)-variable QP:
   *   min_{f_n in R^m, delta >= 0} 0.5 * ||f_n||_2^2 + 0.5 * w_slack * delta^2 + w_lin * delta
   *   subject to
   *     A_cbf(q) * f_n + 1_m * delta >= b_cbf(q, dq, F_ext)
   *     0 <= f_{n,i} <= rho_i * (|dzeta_i| / dt + max(0, -a_drift_i))  (if dzeta_i <= 0, else 0: No Pushback)
   *     tau_min <= tau_nom + J^T * N_6 * f_n <= tau_max
   *     delta >= 0
   *
   * Properties:
   *   - Pure normal reaction force: (I - n n^T) F_cart == 0 (zero lateral deflection / zero tangential resistance).
   *   - Strictly passive / no pushback: f_n == 0 when moving away (dzeta >= 0); decelerates to at most zero velocity.
   */
  class TorqueCBFController
  {
  public:
    /// Constructor for a single preloaded hyperplane
    explicit TorqueCBFController(const TorqueCBFConfig &config = TorqueCBFConfig(),
                                 const Hyperplane &plane = Hyperplane());

    /// Constructor for multiple preloaded hyperplanes
    explicit TorqueCBFController(const TorqueCBFConfig &config,
                                 const std::vector<Hyperplane> &planes);

    /// Update controller configuration
    /// Update configuration. Does NOT reset the entry-latch state, so it is safe to call
    /// every control cycle (e.g. from an S-function). Use resetEntryState() to reset it.
    void setConfig(const TorqueCBFConfig &config);
    const TorqueCBFConfig &getConfig() const { return config_; }

    /// Hyperplane setters and getters. The entry-latch state is reset only if the
    /// (normalized) planes actually change, so re-setting identical planes every cycle is safe.
    void setHyperplane(const Hyperplane &plane);
    void setHyperplanes(const std::vector<Hyperplane> &planes);
    const Hyperplane &getHyperplane() const { return plane_; }
    const std::vector<Hyperplane> &getHyperplanes() const { return planes_; }

    /// Set custom profile functions h(u) and H(u)
    void setCustomProfile(std::function<double(double)> h_func,
                          std::function<double(double)> H_func);

    /// Reset latched entry velocities
    void resetEntryState();

    /// Velocity envelope helpers
    double computeDistance(const Eigen::Vector3d &x, std::size_t plane_idx = 0) const;
    double computeNormalVelocity(const Eigen::Vector3d &dx, std::size_t plane_idx = 0) const;
    double computeVelocityEnvelope(double zeta, std::size_t plane_idx = 0) const;
    double computeVelocityEnvelopeDerivative(double zeta, std::size_t plane_idx = 0) const;
    double computeBarrierValue(double zeta, double dzeta, std::size_t plane_idx = 0) const;

    // -------------------------------------------------------------------------
    // Aligned High-Level Torque Computation Methods
    // Both accept ONLY parameters that cannot be computed internally or preloaded
    // -------------------------------------------------------------------------

    /**
     * @brief Compute full joint control torque vector tau_c in R^n via Pinocchio:
     *        tau_c = g(q) + tau_nom + tau_cbf*
     *
     * @param model      Pinocchio dynamic model
     * @param data       Pinocchio dynamic data
     * @param q          Joint positions in R^n
     * @param dq         Joint velocities in R^n
     * @param frame_id   FrameIndex of the end-effector frame
     * @param F_ext      Cartesian external wrench in R^6 (default zero)
     * @param tau_nom    Nominal feedforward/impedance joint torque vector in R^n (default zero)
     * @param result_out Optional telemetry output pointer
     * @return Commanded joint torque vector tau_c in R^n
     */
    Eigen::VectorXd computeControlTorque(
        const pinocchio::Model &model,
        pinocchio::Data &data,
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq,
        pinocchio::FrameIndex frame_id,
        const Eigen::Matrix<double, 6, 1> &F_ext = Eigen::Matrix<double, 6, 1>::Zero(),
        const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
        TorqueCBFResult *result_out = nullptr);

    /**
     * @brief Compute full joint control torque vector tau_c in R^n from RobotState:
     *        tau_c = g(q) + tau_nom + tau_cbf*
     *
     * @param state      Robot state (q, dq, x, dx, M, C, g, J, dJ, F_ext)
     * @param tau_nom    Nominal feedforward/impedance joint torque vector in R^n (default zero)
     * @param result_out Optional telemetry output pointer
     * @return Commanded joint torque vector tau_c in R^n
     */
    Eigen::VectorXd computeControlTorque(
        const RobotState &state,
        const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
        TorqueCBFResult *result_out = nullptr);

    /**
     * @brief Multi-surface variant accepting identical parameters as computeControlTorque.
     */
    Eigen::VectorXd computeMultiSurfaceControlTorque(
        const pinocchio::Model &model,
        pinocchio::Data &data,
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &dq,
        pinocchio::FrameIndex frame_id,
        const Eigen::Matrix<double, 6, 1> &F_ext = Eigen::Matrix<double, 6, 1>::Zero(),
        const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
        TorqueCBFResult *result_out = nullptr)
    {
      return computeControlTorque(model, data, q, dq, frame_id, F_ext, tau_nom, result_out);
    }

    Eigen::VectorXd computeMultiSurfaceControlTorque(
        const RobotState &state,
        const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
        TorqueCBFResult *result_out = nullptr)
    {
      return computeControlTorque(state, tau_nom, result_out);
    }

    // -------------------------------------------------------------------------
    // Core QP Solver
    // -------------------------------------------------------------------------

    /**
     * @brief Formulates and solves the Direct Joint-Torque CBF Quadratic Program.
     *
     * @param M          Joint inertia matrix M(q) in R^{n x n}
     * @param J          Geometric Jacobian J(q) in R^{6 x n}
     * @param drift_cart Cartesian operational-space drift vector (dJ*dq - J*M^{-1}*C*dq) in R^6
     * @param F_ext      Cartesian external wrench in R^6
     * @param x_ee       Cartesian end-effector position in R^3
     * @param dx_ee      Cartesian end-effector linear velocity in R^3
     * @param tau_nom    Nominal joint torque vector in R^n (optional)
     * @param g          Gravity torque vector in R^n (optional, for total torque limits)
     * @return Step result and telemetry
     */
    TorqueCBFResult solveTorqueQP(
        const Eigen::MatrixXd &M,
        const Eigen::MatrixXd &J,
        const Eigen::Matrix<double, 6, 1> &drift_cart,
        const Eigen::Matrix<double, 6, 1> &F_ext,
        const Eigen::Vector3d &x_ee,
        const Eigen::Vector3d &dx_ee,
        const Eigen::VectorXd &tau_nom = Eigen::VectorXd(),
        const Eigen::VectorXd &g = Eigen::VectorXd());

  private:
    /// Replace planes_; resets the latch state only if the planes differ from the current ones
    void assignPlanes(std::vector<Hyperplane> new_planes);
    /// Resize latch state to planes_.size() (resetting it) if needed
    void ensureEntryStateSize();
    /// Recompute active_v0_[i] from config_ and the latched entry velocity of plane i
    void updateActiveV0(std::size_t i);

    TorqueCBFConfig config_;
    Hyperplane plane_;
    std::vector<Hyperplane> planes_;
    std::vector<double> active_v0_;          ///< Envelope entry velocity currently in use, per plane
    std::vector<bool> was_in_zone_;          ///< Latch-zone membership at previous call (with exit hysteresis)
    std::vector<bool> has_entry_velocity_;   ///< True while inside the latch zone after an entry was recorded
    std::vector<double> entry_dzeta_;        ///< (Filtered) normal velocity recorded at zone entry
    std::vector<bool> dzeta_filt_init_;      ///< Latch-velocity filter initialised
    std::vector<double> dzeta_filt_;         ///< Latch-velocity filter state
    std::function<double(double)> custom_h_;
    std::function<double(double)> custom_H_;
  };

} // namespace torque_cbf
