#include "torque_cbf/torque_cbf_controller.hpp"

// See torque_cbf_controller.hpp: hide Simulink's -DRT from Boost.Math while including Pinocchio
#pragma push_macro("RT")
#undef RT
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma pop_macro("RT")

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace torque_cbf
{

  TorqueCBFController::TorqueCBFController(const TorqueCBFConfig &config,
                                           const Hyperplane &plane)
  {
    setConfig(config);
    setHyperplane(plane);
  }

  TorqueCBFController::TorqueCBFController(const TorqueCBFConfig &config,
                                           const std::vector<Hyperplane> &planes)
  {
    setConfig(config);
    setHyperplanes(planes);
  }

  void TorqueCBFController::setConfig(const TorqueCBFConfig &config)
  {
    if (config.p_ths <= 0.0)
    {
      throw std::invalid_argument("p_ths must be strictly positive.");
    }
    if (config.v_ths >= 0.0)
    {
      throw std::invalid_argument("v_ths must be strictly negative (approach velocity threshold).");
    }
    if (config.v_0 >= config.v_ths)
    {
      throw std::invalid_argument("v_0 must be strictly more negative than v_ths (v_0 < v_ths < 0).");
    }
    if (config.gamma <= 0.0)
    {
      throw std::invalid_argument("gamma must be strictly positive.");
    }
    if (config.w_slack <= 0.0)
    {
      throw std::invalid_argument("w_slack must be strictly positive.");
    }
    if (config.w_lin <= 0.0)
    {
      throw std::invalid_argument("w_lin must be strictly positive.");
    }
    if (config.default_tau_limit <= 0.0)
    {
      throw std::invalid_argument("default_tau_limit must be strictly positive.");
    }
    if (config.dt <= 0.0)
    {
      throw std::invalid_argument("dt must be strictly positive.");
    }
    if (config.zone_exit_hysteresis < 0.0)
    {
      throw std::invalid_argument("zone_exit_hysteresis must be non-negative.");
    }
    if (!(config.latch_velocity_filter_alpha > 0.0 && config.latch_velocity_filter_alpha <= 1.0))
    {
      throw std::invalid_argument("latch_velocity_filter_alpha must be in (0, 1].");
    }
    if (!(config.tau_rate_limit >= 0.0))
    {
      throw std::invalid_argument("tau_rate_limit must be non-negative (0 = disabled).");
    }
    config_ = config;
    // Entry-latch state is intentionally preserved (see header): only the envelope
    // velocities are refreshed, so a changed v_0 / latch flag takes effect immediately.
    ensureEntryStateSize();
    for (std::size_t i = 0; i < active_v0_.size(); ++i)
    {
      updateActiveV0(i);
    }
  }

  void TorqueCBFController::setHyperplane(const Hyperplane &plane)
  {
    setHyperplanes({plane});
  }

  void TorqueCBFController::setHyperplanes(const std::vector<Hyperplane> &planes)
  {
    std::vector<Hyperplane> normalized;
    normalized.reserve(planes.size());
    for (const auto &p : planes)
    {
      const double norm = p.normal.norm();
      if (norm < 1e-12)
      {
        throw std::invalid_argument("Hyperplane normal vector must be non-zero.");
      }
      Hyperplane normalized_p;
      normalized_p.normal = p.normal / norm;
      normalized_p.offset = p.offset;
      normalized.push_back(normalized_p);
    }
    assignPlanes(std::move(normalized));
  }

  void TorqueCBFController::assignPlanes(std::vector<Hyperplane> new_planes)
  {
    bool same = (new_planes.size() == planes_.size());
    for (std::size_t i = 0; same && i < new_planes.size(); ++i)
    {
      same = (new_planes[i].normal == planes_[i].normal) &&
             (new_planes[i].offset == planes_[i].offset);
    }
    planes_ = std::move(new_planes);
    plane_ = planes_.empty() ? Hyperplane() : planes_.front();
    if (!same)
    {
      resetEntryState();
      resetRateState();
    }
  }

  void TorqueCBFController::resetRateState()
  {
    has_f_prev_ = false;
    f_prev_.resize(0);
  }

  void TorqueCBFController::ensureEntryStateSize()
  {
    if (active_v0_.size() != planes_.size() || was_in_zone_.size() != planes_.size() ||
        has_entry_velocity_.size() != planes_.size() || entry_dzeta_.size() != planes_.size() ||
        dzeta_filt_init_.size() != planes_.size() || dzeta_filt_.size() != planes_.size())
    {
      resetEntryState();
    }
  }

  void TorqueCBFController::updateActiveV0(std::size_t i)
  {
    // One-sided latch: only an entry faster (more negative) than v_0 replaces v_0.
    // min(entry, v_0) <= v_0 < v_ths, so the envelope stays well defined.
    active_v0_[i] = (config_.latch_entry_velocity && has_entry_velocity_[i])
                        ? std::min(entry_dzeta_[i], config_.v_0)
                        : config_.v_0;
  }

  void TorqueCBFController::setCustomProfile(
      std::function<double(double)> h_func,
      std::function<double(double)> H_func)
  {
    custom_h_ = std::move(h_func);
    custom_H_ = std::move(H_func);
    config_.profile = ProfileType::CUSTOM;
  }

  void TorqueCBFController::resetEntryState()
  {
    const std::size_t m = planes_.size();
    active_v0_.assign(m, config_.v_0);
    was_in_zone_.assign(m, false);
    has_entry_velocity_.assign(m, false);
    entry_dzeta_.assign(m, 0.0);
    dzeta_filt_init_.assign(m, false);
    dzeta_filt_.assign(m, 0.0);
  }

  double TorqueCBFController::computeDistance(
      const Eigen::Vector3d &x, std::size_t plane_idx) const
  {
    if (plane_idx >= planes_.size())
    {
      throw std::out_of_range("computeDistance: plane_idx out of range.");
    }
    return planes_[plane_idx].normal.dot(x - planes_[plane_idx].offset);
  }

  double TorqueCBFController::computeNormalVelocity(
      const Eigen::Vector3d &dx, std::size_t plane_idx) const
  {
    if (plane_idx >= planes_.size())
    {
      throw std::out_of_range("computeNormalVelocity: plane_idx out of range.");
    }
    return planes_[plane_idx].normal.dot(dx);
  }

  double TorqueCBFController::computeVelocityEnvelope(
      double zeta, std::size_t plane_idx) const
  {
    const double v0 = (plane_idx < active_v0_.size()) ? active_v0_[plane_idx] : config_.v_0;
    return cbf::computeVelocityEnvelope(
        zeta, config_.p_ths, v0, config_.v_ths, config_.profile, custom_H_);
  }

  double TorqueCBFController::computeVelocityEnvelopeDerivative(
      double zeta, std::size_t plane_idx) const
  {
    const double v0 = (plane_idx < active_v0_.size()) ? active_v0_[plane_idx] : config_.v_0;
    return cbf::computeVelocityEnvelopeDerivative(
        zeta, config_.p_ths, v0, config_.v_ths, config_.profile, custom_h_, custom_H_);
  }

  double TorqueCBFController::computeBarrierValue(
      double zeta, double dzeta, std::size_t plane_idx) const
  {
    return dzeta - computeVelocityEnvelope(zeta, plane_idx);
  }

  Eigen::VectorXd TorqueCBFController::computeControlTorque(
      const pinocchio::Model &model,
      pinocchio::Data &data,
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq,
      pinocchio::FrameIndex frame_id,
      const Eigen::Matrix<double, 6, 1> &F_ext,
      const Eigen::VectorXd &tau_nom,
      TorqueCBFResult *result_out)
  {
    if (frame_id >= model.frames.size())
    {
      throw std::runtime_error("computeControlTorque: frame_id not found in Pinocchio model.");
    }

    // Kinematics and Jacobian calculations
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
    pinocchio::updateFramePlacements(model, data);

    const Eigen::Vector3d x_ee = data.oMf[frame_id].translation();

    pinocchio::Data::Matrix6x J(6, model.nv);
    pinocchio::Data::Matrix6x dJ(6, model.nv);
    J.setZero();
    dJ.setZero();
    pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    pinocchio::getFrameJacobianTimeVariation(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ);

    const Eigen::Matrix<double, 6, 1> dx6 = J * dq;
    const Eigen::Vector3d dx_ee = dx6.head<3>();

    // Pinocchio's LOCAL_WORLD_ALIGNED dJ gives J*ddq + dJ*dq = SPATIAL acceleration; the classical linear
    // acceleration of the frame origin (= d/dt of dx_ee, what the CBF needs) is that plus omega x v.
    // Make dJ the true time derivative of J: dJ_lin += -[v]x * J_ang  (so dJ*dq gains omega x v).
    {
      const Eigen::Vector3d v = dx6.head<3>();
      Eigen::Matrix3d v_skew;
      v_skew << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
      dJ.topRows<3>() -= v_skew * J.bottomRows<3>();
    }

    // Dynamics computations: M(q), C(q, dq)*dq, g(q)
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::computeCoriolisMatrix(model, data, q, dq);
    pinocchio::computeGeneralizedGravity(model, data, q);

    const Eigen::VectorXd C_dq = data.C * dq;
    const auto ldlt = data.M.ldlt();
    const Eigen::VectorXd Minv_C_dq = ldlt.solve(C_dq);
    const Eigen::Matrix<double, 6, 1> drift_cart = dJ * dq - J * Minv_C_dq;

    TorqueCBFResult res = solveTorqueQP(data.M, J, drift_cart, F_ext, x_ee, dx_ee, tau_nom, data.g);

    const Eigen::VectorXd tau_nom_eff =
        (tau_nom.size() == model.nv) ? tau_nom : Eigen::VectorXd::Zero(model.nv);
    res.tau_cmd = data.g + tau_nom_eff + res.tau_cbf;

    if (result_out)
    {
      *result_out = res;
    }
    return res.tau_cmd;
  }

  Eigen::VectorXd TorqueCBFController::computeControlTorque(
      const RobotState &state,
      const Eigen::VectorXd &tau_nom,
      TorqueCBFResult *result_out)
  {
    const Eigen::Index n = state.q.size();
    const Eigen::VectorXd C_dq = state.C * state.dq;
    const auto ldlt = state.M.ldlt();
    const Eigen::VectorXd Minv_C_dq = ldlt.solve(C_dq);
    const Eigen::Matrix<double, 6, 1> drift_cart = state.dJ * state.dq - state.J * Minv_C_dq;

    TorqueCBFResult res = solveTorqueQP(
        state.M, state.J, drift_cart, state.F_ext, state.x, state.dx, tau_nom, state.g);

    const Eigen::VectorXd tau_nom_eff =
        (tau_nom.size() == n) ? tau_nom : Eigen::VectorXd::Zero(n);
    res.tau_cmd = state.g + tau_nom_eff + res.tau_cbf;

    if (result_out)
    {
      *result_out = res;
    }
    return res.tau_cmd;
  }

  TorqueCBFResult TorqueCBFController::solveTorqueQP(
      const Eigen::MatrixXd &M,
      const Eigen::MatrixXd &J,
      const Eigen::Matrix<double, 6, 1> &drift_cart,
      const Eigen::Matrix<double, 6, 1> &F_ext,
      const Eigen::Vector3d &x_ee,
      const Eigen::Vector3d &dx_ee,
      const Eigen::VectorXd &tau_nom,
      const Eigen::VectorXd &g)
  {
    const Eigen::Index n = M.rows();
    const Eigen::Index m = static_cast<Eigen::Index>(planes_.size());

    if (m == 0)
    {
      throw std::runtime_error("solveTorqueQP: No hyperplanes configured.");
    }
    ensureEntryStateSize();

    TorqueCBFResult res;
    res.tau_cbf = Eigen::VectorXd::Zero(n);
    res.tau_cmd = Eigen::VectorXd::Zero(n);
    res.f_n = Eigen::VectorXd::Zero(m);
    res.rho = Eigen::VectorXd::Zero(m);
    res.f_brake_max = Eigen::VectorXd::Zero(m);
    res.f_applied_est = Eigen::VectorXd::Zero(m);
    res.f_rate_max = Eigen::VectorXd::Constant(m, std::numeric_limits<double>::infinity());
    res.f_rate = Eigen::VectorXd::Constant(m, std::numeric_limits<double>::infinity());
    res.zetas.resize(m);
    res.dzetas.resize(m);
    res.v_mins.resize(m);
    res.barriers.resize(m);
    res.b_cbf.resize(m);
    res.a_drift.resize(m);
    res.active_planes.resize(m, false);

    // LDLT decomposition of joint inertia matrix M(q)
    const auto ldlt = M.ldlt();
    const Eigen::MatrixXd Minv_JT = ldlt.solve(J.transpose()); // n x 6
    const Eigen::MatrixXd JM_inv = Minv_JT.transpose();        // 6 x n
    const Eigen::Matrix<double, 6, 6> Lambda_inv = J * Minv_JT;// 6 x 6

    // Acceleration contributions to Cartesian drift
    const Eigen::Matrix<double, 6, 1> nom_accel =
        (tau_nom.size() == n) ? static_cast<Eigen::Matrix<double, 6, 1>>(JM_inv * tau_nom)
                              : Eigen::Matrix<double, 6, 1>::Zero();

    const Eigen::Matrix<double, 6, 1> ext_accel =
        config_.compensate_external_force
            ? static_cast<Eigen::Matrix<double, 6, 1>>(Lambda_inv * F_ext)
            : Eigen::Matrix<double, 6, 1>::Zero();

    const Eigen::Matrix<double, 6, 1> a_drift_total = drift_cart + nom_accel + ext_accel;

    // Assemble spatial normals N_6 in R^{6 x m}
    Eigen::MatrixXd N6(6, m);
    for (Eigen::Index i = 0; i < m; ++i)
    {
      const Eigen::Vector3d n_i = planes_[static_cast<std::size_t>(i)].normal.normalized();
      N6.block<3, 1>(0, i) = n_i;
      N6.block<3, 1>(3, i).setZero();
    }

    // Operational space compliance matrix A_cbf = N_6^T * Lambda^{-1} * N_6 in R^{m x m}
    Eigen::MatrixXd A_cbf = N6.transpose() * Lambda_inv * N6;
    A_cbf = 0.5 * (A_cbf + A_cbf.transpose());

    // Effective operational-space inertia along each hyperplane normal: rho_i = 1 / A_cbf(i, i)
    for (Eigen::Index i = 0; i < m; ++i)
    {
      res.rho(i) = 1.0 / std::max(1e-9, A_cbf(i, i));
    }

    // Control mapping matrix K = J^T * N_6 in R^{n x m}
    const Eigen::MatrixXd K = J.transpose() * N6;

    // Torque-rate limit -> normal-force rate r_i [N/s]: a change df of f_n,i changes joint j by K(j,i)*df.
    // With m planes changing together the per-plane budget is split (conservative).
    const bool rate_on = config_.tau_rate_limit > 0.0;
    if (rate_on)
    {
      for (Eigen::Index i = 0; i < m; ++i)
      {
        const double k_max = std::max(1e-9, K.col(i).cwiseAbs().maxCoeff());
        res.f_rate(i) = config_.tau_rate_limit / (k_max * static_cast<double>(m));
      }
    }
    // Force the robot applied after the previous call (zero if none: the robot starts without CBF force)
    const Eigen::VectorXd f_prev =
        (rate_on && has_f_prev_ && f_prev_.size() == m) ? f_prev_ : Eigen::VectorXd::Zero(m);

    int best_plane_idx = 0;
    double max_b = -std::numeric_limits<double>::infinity();
    std::vector<Eigen::Index> zone_planes; // planes inside the CBF zone -> one QP row each
    zone_planes.reserve(static_cast<std::size_t>(m));

    for (Eigen::Index i = 0; i < m; ++i)
    {
      const Eigen::Matrix<double, 6, 1> n6_i = N6.col(i);
      const double zeta_i = computeDistance(x_ee, static_cast<std::size_t>(i));
      const double dzeta_i = computeNormalVelocity(dx_ee, static_cast<std::size_t>(i));

      const std::size_t si = static_cast<std::size_t>(i);

      // CBF activation zone (unchanged)
      const bool in_zone = (zeta_i <= config_.p_ths && zeta_i >= -0.05);

      // Velocity sampled by the entry latch (EMA, does not affect the QP itself)
      if (!dzeta_filt_init_[si])
      {
        dzeta_filt_[si] = dzeta_i;
        dzeta_filt_init_[si] = true;
      }
      else
      {
        dzeta_filt_[si] += config_.latch_velocity_filter_alpha * (dzeta_i - dzeta_filt_[si]);
      }

      // Latch-zone membership with exit hysteresis: enter at zeta <= p_ths,
      // leave only at zeta > p_ths + zone_exit_hysteresis
      const bool in_latch_zone = was_in_zone_[si]
                                     ? (zeta_i <= config_.p_ths + config_.zone_exit_hysteresis)
                                     : (zeta_i <= config_.p_ths);
      if (in_latch_zone && !was_in_zone_[si])
      {
        entry_dzeta_[si] = dzeta_filt_[si];
        has_entry_velocity_[si] = true;
      }
      else if (!in_latch_zone)
      {
        has_entry_velocity_[si] = false;
      }
      was_in_zone_[si] = in_latch_zone;

      // One-sided latch (if enabled): v_0 <- min(entry velocity, v_0)
      updateActiveV0(si);

      const double v_min_i = computeVelocityEnvelope(zeta_i, static_cast<std::size_t>(i));
      const double dv_min_i = computeVelocityEnvelopeDerivative(zeta_i, static_cast<std::size_t>(i));
      const double B_i = dzeta_i - v_min_i;
      const double a_drift_i = n6_i.dot(a_drift_total);

      // One-way velocity braking cap (Passivity / Kinesthetic Teaching)
      // When moving away (dzeta >= 0), cap is 0 -> f_n == 0 identically.
      // When approaching (dzeta < 0), cap is (rho_i / dt) * |dzeta_i|.
      // One-cycle model: dzeta_next = dzeta + (a_drift + f / rho) * dt.
      //   - |dzeta| * rho / dt      : force that stops the current approach velocity in one cycle
      //   - max(0, -a_drift) * rho  : (optional) force that cancels the approaching drift
      //                               acceleration (human push, gravity residual, ...) in the same cycle
      // With both terms dzeta_next <= 0: the CBF can stop the robot but never reverse it.
      // The cap is zero while separating (dzeta > 0), so the CBF power f * dzeta is never positive.
      double f_brake_i = std::numeric_limits<double>::infinity();
      if (config_.enforce_passivity)
      {
        if (dzeta_i <= 0.0)
        {
          const double a_cancel =
              config_.brake_cap_include_drift ? std::max(0.0, -a_drift_i) : 0.0;
          f_brake_i = res.rho(i) * (std::abs(dzeta_i) / config_.dt + a_cancel);

          // Pushback-aware cap (rate-limited actuator): the force above the holding force
          // f_hold = rho*max(0,-a_drift) decelerates the end-effector. Ramping an excess e down at
          // r [N/s] removes e^2/(2r) of impulse, i.e. e^2/(2 r rho) of velocity; requiring this to be
          // <= |dzeta| gives e <= sqrt(2 r rho |dzeta|): the excess is gone when dzeta reaches 0.
          if (rate_on)
          {
            const double f_hold = res.rho(i) * std::max(0.0, -a_drift_i);
            const double f_unwind = f_hold + std::sqrt(2.0 * res.f_rate(i) * res.rho(i) * std::abs(dzeta_i));
            f_brake_i = std::min(f_brake_i, f_unwind);
          }
        }
        else
        {
          f_brake_i = 0.0;
        }
      }
      res.f_brake_max(i) = f_brake_i;

      // Lower bound b_cbf,i (capped by the one-way braking cap so the barrier never fights passivity)
      double b_i = 0.0;
      if (in_zone)
      {
        b_i = dv_min_i * dzeta_i - a_drift_i - config_.gamma * B_i;
        if (config_.enforce_passivity && std::isfinite(f_brake_i))
        {
          b_i = std::min(b_i, f_brake_i / res.rho(i));
        }
        res.active_planes[static_cast<std::size_t>(i)] = (b_i > 0.0);
        zone_planes.push_back(i);
      }
      else
      {
        // Out of zone: no CBF row is added to the QP; finite sentinel kept for telemetry only
        b_i = -1e3;
        res.active_planes[static_cast<std::size_t>(i)] = false;
      }

      // Ramp-up bound f <= f_app + r*dt; f_app = what the robot still applies (decays by <= r*dt)
      if (rate_on)
      {
        const double df = res.f_rate(i) * config_.dt;
        res.f_applied_est(i) = std::max(0.0, f_prev(i) - df);
        res.f_rate_max(i) = f_prev(i) + df;
      }

      res.zetas(i) = zeta_i;
      res.dzetas(i) = dzeta_i;
      res.v_mins(i) = v_min_i;
      res.barriers(i) = B_i;
      res.b_cbf(i) = b_i;
      res.a_drift(i) = a_drift_i;

      if (b_i > max_b)
      {
        max_b = b_i;
        best_plane_idx = static_cast<int>(i);
      }
    }

    res.critical_plane_idx = best_plane_idx;
    res.zeta = res.zetas(best_plane_idx);
    res.dzeta = res.dzetas(best_plane_idx);
    res.v_min = res.v_mins(best_plane_idx);
    res.dv_min_dzeta = computeVelocityEnvelopeDerivative(res.zeta, static_cast<std::size_t>(best_plane_idx));
    res.barrier_B = res.barriers(best_plane_idx);
    res.b_cbf_scalar = res.b_cbf(best_plane_idx);
    res.a_drift_scalar = res.a_drift(best_plane_idx);
    res.is_active = (max_b > 0.0);

    // Box bounds on f_n: [0, min(f_brake_max, f_rate_max)] (passivity / pushback cap and ramp-up bound)
    const Eigen::VectorXd f_lb = Eigen::VectorXd::Zero(m);
    const Eigen::VectorXd f_ub = res.f_brake_max.cwiseMin(res.f_rate_max);

    // Joint torque limits on tau_cbf = K * f_n
    //   enforce_total_torque_limits == false:  tau_min <= tau_cbf <= tau_max
    //   enforce_total_torque_limits == true:   tau_min <= g + tau_nom + tau_cbf <= tau_max
    // In the total case the bounds on tau_cbf are relaxed to always contain 0: if g + tau_nom
    // alone already violates a limit, the CBF is only prevented from making it worse
    // (it cannot repair torques it does not own), and the QP stays feasible.
    const Eigen::VectorXd tau_min_abs = (config_.tau_min.size() == n)
                                            ? config_.tau_min
                                            : Eigen::VectorXd::Constant(n, -config_.default_tau_limit);
    const Eigen::VectorXd tau_max_abs = (config_.tau_max.size() == n)
                                            ? config_.tau_max
                                            : Eigen::VectorXd::Constant(n, config_.default_tau_limit);
    Eigen::VectorXd tau_min_eff = tau_min_abs;
    Eigen::VectorXd tau_max_eff = tau_max_abs;
    if (config_.enforce_total_torque_limits)
    {
      const Eigen::VectorXd tau_nom_eff =
          (tau_nom.size() == n) ? tau_nom : Eigen::VectorXd::Zero(n);
      const Eigen::VectorXd g_eff =
          (g.size() == n) ? g : Eigen::VectorXd::Zero(n);
      const Eigen::VectorXd total_base_torque = tau_nom_eff + g_eff;
      tau_min_eff = (tau_min_abs - total_base_torque).cwiseMin(0.0);
      tau_max_eff = (tau_max_abs - total_base_torque).cwiseMax(0.0);
    }

    // Fast path: if not active and zero force is strictly feasible, return zero torque
    const bool zero_torque_feasible =
        (tau_min_eff.array() <= 0.0).all() && (tau_max_eff.array() >= 0.0).all();
    if (!res.is_active && zero_torque_feasible)
    {
      f_prev_ = res.f_applied_est;   // the robot is still ramping down whatever it applied before
      has_f_prev_ = true;
      res.tau_cbf.setZero();
      res.f_n.setZero();
      res.f_n_scalar = 0.0;
      res.F_cart_lin.setZero();
      res.F_cart_lat.setZero();
      res.slack_delta = 0.0;
      res.qp_cost = 0.0;
      res.qp_success = true;
      return res;
    }

    // Setup QP objective in z = [f_n, delta] in R^{m + 1}:
    // min_{z} 0.5 * z^T * G * z + g0^T * z
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(m + 1, m + 1);
    G.topLeftCorner(m, m).setIdentity();
    G(m, m) = config_.w_slack;

    Eigen::VectorXd g0 = Eigen::VectorXd::Zero(m + 1);
    g0(m) = config_.w_lin;

    // Box bounds on z:
    // 0 <= f_n,i <= f_ub,i   (passivity / pushback cap and ramp-up bound, see above)
    // 0 <= delta < inf
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(m + 1);
    Eigen::VectorXd ub(m + 1);
    lb.head(m) = f_lb;
    ub.head(m) = f_ub;
    ub(m) = std::numeric_limits<double>::infinity();

    // Linear inequalities A_ineq * z >= b_ineq
    // 1) CBF constraints (m_z rows, only planes inside the zone): A_cbf(i,:) * f_n + delta >= b_cbf,i
    // 2) Lower torque limits (n rows): K * f_n >= tau_min_eff
    // 3) Upper torque limits (n rows): -K * f_n >= -tau_max_eff
    const Eigen::Index m_z = static_cast<Eigen::Index>(zone_planes.size());
    const Eigen::Index m_ineq = m_z + 2 * n;
    Eigen::MatrixXd A_ineq = Eigen::MatrixXd::Zero(m_ineq, m + 1);
    Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(m_ineq);

    // CBF block
    for (Eigen::Index r = 0; r < m_z; ++r)
    {
      const Eigen::Index i = zone_planes[static_cast<std::size_t>(r)];
      A_ineq.block(r, 0, 1, m) = A_cbf.row(i);
      A_ineq(r, m) = 1.0;
      b_ineq(r) = res.b_cbf(i);
    }

    // Torque limits block
    A_ineq.block(m_z, 0, n, m) = K;
    b_ineq.segment(m_z, n) = tau_min_eff;

    A_ineq.block(m_z + n, 0, n, m) = -K;
    b_ineq.segment(m_z + n, n) = -tau_max_eff;

    // Solve strictly convex QP
    Eigen::VectorXd z_opt(m + 1);
    const double qp_cost = cbf::solve_quadprog_box(G, g0, A_ineq, b_ineq, lb, ub, z_opt);

    if (std::isfinite(qp_cost))
    {
      res.qp_success = true;
      res.qp_cost = qp_cost;
      res.f_n = z_opt.head(m);
      res.slack_delta = std::max(0.0, z_opt(m));
      res.tau_cbf = K * res.f_n;
      res.f_n_scalar = res.f_n(best_plane_idx);

      // Operational-space Cartesian force telemetry
      res.F_cart_lin = N6.topRows<3>() * res.f_n;
      const Eigen::Vector3d n_crit = planes_[static_cast<std::size_t>(best_plane_idx)].normal.normalized();
      res.F_cart_lat = res.F_cart_lin - n_crit * (n_crit.dot(res.F_cart_lin));
    }
    else
    {
      // Fallback: no CBF force (the robot ramps any previous force down by itself)
      res.qp_success = false;
      res.qp_cost = qp_cost;
      res.f_n.setZero();
      res.tau_cbf.setZero();
      res.f_n_scalar = 0.0;
      res.F_cart_lin.setZero();
      res.F_cart_lat.setZero();
      res.slack_delta = 0.0;
    }

    if (rate_on)
    {
      for (Eigen::Index i = 0; i < m; ++i)
      {
        const double tol = 1e-9 * std::max(1.0, res.f_rate_max(i));
        if (res.f_rate_max(i) < res.f_brake_max(i) && res.f_n(i) >= res.f_rate_max(i) - tol)
        {
          res.rate_limited = true;
        }
      }
    }
    // Applied force after this call: the new command, or the previous force still ramping down
    f_prev_ = rate_on ? Eigen::VectorXd(res.f_n.cwiseMax(res.f_applied_est)) : res.f_n;
    has_f_prev_ = true;

    return res;
  }

} // namespace torque_cbf
