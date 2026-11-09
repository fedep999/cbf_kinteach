#include "damping_cbf/damping_cbf_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace damping_cbf
{

  DampingCBFController::DampingCBFController(const ControllerConfig &config,
                                             const Hyperplane &plane)
  {
    setConfig(config);
    setHyperplane(plane);
  }

  DampingCBFController::DampingCBFController(const ControllerConfig &config,
                                             const std::vector<Hyperplane> &planes)
  {
    setConfig(config);
    setHyperplanes(planes);
  }

  void DampingCBFController::setConfig(const ControllerConfig &config)
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
    if (config.d_max < 0.0)
    {
      throw std::invalid_argument("d_max must be non-negative.");
    }
    if (config.slack_quadratic_penalty <= 0.0)
    {
      throw std::invalid_argument("slack_quadratic_penalty must be strictly positive.");
    }
    if (config.zone_exit_hysteresis < 0.0)
    {
      throw std::invalid_argument("zone_exit_hysteresis must be non-negative.");
    }
    if (!(config.latch_velocity_filter_alpha > 0.0 && config.latch_velocity_filter_alpha <= 1.0))
    {
      throw std::invalid_argument("latch_velocity_filter_alpha must be in (0, 1].");
    }
    config_ = config;
    // Entry-latch state is intentionally preserved (see header): only the envelope
    // velocity is refreshed, so a changed v_0 / latch flag takes effect immediately.
    updateActiveV0();
  }

  void DampingCBFController::updateActiveV0()
  {
    // One-sided latch: only an entry faster (more negative) than v_0 replaces v_0.
    // min(entry, v_0) <= v_0 < v_ths, so the envelope stays well defined.
    active_v0_ = (config_.latch_entry_velocity && has_entry_velocity_)
                     ? std::min(entry_dzeta_, config_.v_0)
                     : config_.v_0;
  }

  void DampingCBFController::setHyperplane(const Hyperplane &plane)
  {
    const double norm = plane.normal.norm();
    if (norm < 1e-12)
    {
      throw std::invalid_argument("Hyperplane normal vector must be non-zero.");
    }
    plane_.normal = plane.normal / norm;
    plane_.offset = plane.offset;
    planes_ = {plane_};
  }

  void DampingCBFController::setHyperplanes(const std::vector<Hyperplane> &planes)
  {
    planes_.clear();
    planes_.reserve(planes.size());
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
      planes_.push_back(normalized_p);
    }
    plane_ = planes_.empty() ? Hyperplane() : planes_.front();
  }

  void DampingCBFController::setCustomProfile(
      std::function<double(double)> h_func,
      std::function<double(double)> H_func)
  {
    custom_h_ = std::move(h_func);
    custom_H_ = std::move(H_func);
    config_.profile = ProfileType::CUSTOM;
  }

  void DampingCBFController::resetEntryState()
  {
    was_in_zone_ = false;
    has_entry_velocity_ = false;
    entry_dzeta_ = 0.0;
    dzeta_filt_init_ = false;
    dzeta_filt_ = 0.0;
    updateActiveV0();
  }

  double DampingCBFController::evaluateProfileH(double u) const
  {
    return cbf::evaluateProfileH(config_.profile, u, custom_h_);
  }

  double DampingCBFController::evaluateAntiderivativeH(double u) const
  {
    return cbf::evaluateAntiderivativeH(config_.profile, u, custom_H_);
  }

  double DampingCBFController::computeDistance(const Eigen::Vector3d &x) const
  {
    return plane_.normal.dot(x - plane_.offset);
  }

  double DampingCBFController::computeNormalVelocity(
      const Eigen::Vector3d &dx) const
  {
    return plane_.normal.dot(dx);
  }

  double DampingCBFController::computeEffectiveInertia(
      const Eigen::Matrix<double, 6, 6> &Lambda,
      const Eigen::Vector3d &normal)
  {
    Eigen::Matrix<double, 6, 1> n6 = Eigen::Matrix<double, 6, 1>::Zero();
    n6.head<3>() = normal.normalized();
    const double inv_rho = n6.transpose() * Lambda.ldlt().solve(n6);
    if (inv_rho <= 1e-12)
    {
      throw std::runtime_error("Invalid non-positive inverse effective inertia.");
    }
    return 1.0 / inv_rho;
  }

  Eigen::Matrix<double, 6, 6> DampingCBFController::computeCartesianInertia(
      const Eigen::MatrixXd &M, const Eigen::MatrixXd &J)
  {
    const Eigen::MatrixXd Minv_JT = M.ldlt().solve(J.transpose());
    const Eigen::Matrix<double, 6, 6> J_Minv_JT = J * Minv_JT;
    return J_Minv_JT.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
  }

  Eigen::Matrix<double, 6, 6> DampingCBFController::computeCartesianCoriolis(
      const Eigen::MatrixXd &M, const Eigen::MatrixXd &C,
      const Eigen::MatrixXd &J, const Eigen::MatrixXd &dJ,
      const Eigen::Matrix<double, 6, 6> &Lambda)
  {
    const Eigen::MatrixXd Minv_JT_Lambda = M.ldlt().solve(J.transpose()) * Lambda;
    return Lambda * (J * M.ldlt().solve(C) - dJ) * Minv_JT_Lambda;
  }

  double DampingCBFController::computeVelocityEnvelope(double zeta) const
  {
    return cbf::computeVelocityEnvelope(zeta, config_.p_ths, active_v0_, config_.v_ths, config_.profile, custom_H_);
  }

  double DampingCBFController::computeVelocityEnvelopeDerivative(
      double zeta) const
  {
    return cbf::computeVelocityEnvelopeDerivative(zeta, config_.p_ths, active_v0_, config_.v_ths, config_.profile, custom_h_, custom_H_);
  }

  double DampingCBFController::computeBarrierValue(double zeta,
                                                   double dzeta) const
  {
    return cbf::computeBarrierValue(zeta, dzeta, config_.p_ths, active_v0_, config_.v_ths, config_.profile, custom_H_);
  }

  Eigen::VectorXd DampingCBFController::solveGeneralCBFQP(
      const Eigen::MatrixXd &G,
      const Eigen::VectorXd &g0,
      const Eigen::MatrixXd &A_cbf,
      const Eigen::VectorXd &b_cbf,
      const Eigen::VectorXd &lb,
      const Eigen::VectorXd &ub,
      double *optimal_cost_out) const
  {
    Eigen::VectorXd z_opt = Eigen::VectorXd::Zero(G.rows());
    const double cost = solve_quadprog_box(G, g0, A_cbf, b_cbf, lb, ub, z_opt);
    if (optimal_cost_out)
    {
      *optimal_cost_out = cost;
    }
    return z_opt;
  }

  CBFStepResult DampingCBFController::solveDampingQP(
      double zeta, double dzeta, double rho, double a_drift,
      const Eigen::MatrixXd &A_extra,
      const Eigen::VectorXd &b_extra)
  {
    CBFStepResult res;
    res.zeta = zeta;
    res.dzeta = dzeta;
    res.rho = rho;

    // Velocity sampled by the entry latch (EMA, does not affect the QP itself)
    if (!dzeta_filt_init_)
    {
      dzeta_filt_ = dzeta;
      dzeta_filt_init_ = true;
    }
    else
    {
      dzeta_filt_ += config_.latch_velocity_filter_alpha * (dzeta - dzeta_filt_);
    }

    // Zone membership with exit hysteresis: enter at zeta <= p_ths,
    // leave only at zeta > p_ths + zone_exit_hysteresis
    const bool in_zone = was_in_zone_
                             ? (zeta <= config_.p_ths + config_.zone_exit_hysteresis)
                             : (zeta <= config_.p_ths);
    if (in_zone && !was_in_zone_)
    {
      entry_dzeta_ = dzeta_filt_;
      has_entry_velocity_ = true;
    }
    else if (!in_zone)
    {
      has_entry_velocity_ = false;
    }
    was_in_zone_ = in_zone;

    // One-sided latch (if enabled): v_0 <- min(entry velocity, v_0)
    updateActiveV0();

    res.v_min = computeVelocityEnvelope(zeta);
    res.dv_min_dzeta = computeVelocityEnvelopeDerivative(zeta);
    res.barrier_B = dzeta - res.v_min;

    res.Lf_B = a_drift - res.dv_min_dzeta * dzeta;
    res.Lg_B = -dzeta / rho;

    // Activation criteria:
    // 1) zeta(t) <= p_ths (inside braking zone, INCLUDING zeta < 0: the controller must
    //    stay active at/below the surface, otherwise damping drops to zero exactly at contact;
    //    for zeta < 0 the envelope is clamped to v_min = v_ths)
    // 2) dzeta(t) < v_ths < 0 (approaching faster than terminal contact speed threshold)
    if (zeta <= config_.p_ths && dzeta < config_.v_ths)
    {
      res.is_active = true;

      // Formulate 2D QP over decision vector z = [d, eps]^T in R^2:
      //   min_{z in R^2}  0.5 * z^T * G * z + g0^T * z
      //   subject to      [L_g B,  1.0] * z >= -(L_f B + gamma * B)
      //                   A_extra       * z >= b_extra   (optional extra 2D CBF / backup CBF rows)
      //                   [0, 0]^T <= z <= [d_max, +inf]^T
      Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2, 2);
      G(0, 0) = 1.0;
      G(1, 1) = config_.slack_quadratic_penalty;

      Eigen::VectorXd g0 = Eigen::VectorXd::Zero(2);
      g0(0) = 0.0;
      g0(1) = config_.slack_linear_penalty;

      const Eigen::Index m_extra = A_extra.rows();
      if (m_extra > 0 && (A_extra.cols() != 2 || b_extra.size() != m_extra))
      {
        throw std::invalid_argument(
            "solveDampingQP: A_extra must have 2 columns and match b_extra size.");
      }

      Eigen::MatrixXd A_ineq(1 + m_extra, 2);
      Eigen::VectorXd b_ineq(1 + m_extra);

      // Row 0: Primary phase-space CBF constraint L_g B * d + 1 * eps >= -(L_f B + gamma * B)
      A_ineq(0, 0) = res.Lg_B;
      A_ineq(0, 1) = 1.0;
      b_ineq(0) = -(res.Lf_B + config_.gamma * res.barrier_B);

      // Additional rows (if any): e.g. backup CBF or secondary barrier constraints
      if (m_extra > 0)
      {
        A_ineq.bottomRows(m_extra) = A_extra;
        b_ineq.tail(m_extra) = b_extra;
      }

      Eigen::VectorXd lb(2);
      lb << 0.0, 0.0;
      Eigen::VectorXd ub(2);
      ub << config_.d_max, std::numeric_limits<double>::infinity();

      // Solve 2D QP numerically via Goldfarb-Idnani dual active-set algorithm (quadprog)
      Eigen::VectorXd z_sol = solveGeneralCBFQP(
          G, g0, A_ineq, b_ineq, lb, ub, &res.qp_cost);

      res.z_opt = z_sol.head<2>();
      res.d_cbf = res.z_opt(0);
      res.slack_eps = res.z_opt(1);
    }
    else
    {
      res.is_active = false;
      res.z_opt.setZero();
      res.d_cbf = 0.0;
      res.slack_eps = 0.0;
      res.qp_cost = 0.0;
    }

    res.dB_dt = res.Lf_B + res.Lg_B * res.d_cbf;
    res.f_imp_scalar = res.d_cbf * std::abs(dzeta);
    res.F_imp = plane_.normal * (res.d_cbf * dzeta);
    res.D_imp = res.d_cbf * (plane_.normal * plane_.normal.transpose());

    res.d_opt = Eigen::VectorXd::Constant(1, res.d_cbf);
    res.zetas = Eigen::VectorXd::Constant(1, res.zeta);
    res.dzetas = Eigen::VectorXd::Constant(1, res.dzeta);
    res.v_mins = Eigen::VectorXd::Constant(1, res.v_min);
    res.barriers = Eigen::VectorXd::Constant(1, res.barrier_B);

    return res;
  }

  CBFStepResult DampingCBFController::solveDampingQP(
      const Eigen::Vector3d &x,
      const Eigen::Vector3d &dx,
      const Eigen::Matrix<double, 6, 6> &Lambda,
      const Eigen::Matrix<double, 6, 1> &F_ext,
      CBFStepResult *cbf_result_out)
  {
    const double zeta = computeDistance(x);
    const double dzeta = computeNormalVelocity(dx);
    const double rho = computeEffectiveInertia(Lambda, plane_.normal);

    Eigen::Matrix<double, 6, 1> n6 = Eigen::Matrix<double, 6, 1>::Zero();
    n6.head<3>() = plane_.normal;
    const double a_drift = config_.compensate_external_force
                               ? static_cast<double>(n6.transpose() * Lambda.ldlt().solve(F_ext))
                               : 0.0;

    CBFStepResult res = solveDampingQP(zeta, dzeta, rho, a_drift);
    if (cbf_result_out)
    {
      *cbf_result_out = res;
    }
    return res;
  }

  CBFStepResult DampingCBFController::solveDampingQP(
      const RobotState &state,
      CBFStepResult *cbf_result_out)
  {
    const Eigen::Matrix<double, 6, 6> Lambda =
        computeCartesianInertia(state.M, state.J);
    return solveDampingQP(state.x, state.dx, Lambda, state.F_ext, cbf_result_out);
  }

  CBFStepResult DampingCBFController::solveMultiSurfaceDampingQP(
      const Eigen::Vector3d &x,
      const Eigen::Vector3d &dx,
      const Eigen::Matrix<double, 6, 6> &Lambda,
      const Eigen::Matrix<double, 6, 1> &F_ext,
      CBFStepResult *cbf_result_out) const
  {
    CBFStepResult res;
    const int m = static_cast<int>(planes_.size());
    res.d_opt = Eigen::VectorXd::Zero(m);
    res.zetas = Eigen::VectorXd::Zero(m);
    res.dzetas = Eigen::VectorXd::Zero(m);
    res.v_mins = Eigen::VectorXd::Zero(m);
    res.barriers = Eigen::VectorXd::Zero(m);

    if (m == 0)
    {
      if (cbf_result_out)
      {
        *cbf_result_out = res;
      }
      return res;
    }

    const Eigen::Matrix<double, 6, 6> Lambda_inv =
        Lambda.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    std::vector<Eigen::Matrix<double, 6, 1>> n6_vec(m);
    for (int i = 0; i < m; ++i)
    {
      n6_vec[i].setZero();
      n6_vec[i].head<3>() = planes_[i].normal.normalized();
      res.dzetas(i) = n6_vec[i].head<3>().dot(dx);
      res.zetas(i) = n6_vec[i].head<3>().dot(x - planes_[i].offset);
      res.v_mins(i) = computeVelocityEnvelope(res.zetas(i));
      res.barriers(i) = res.dzetas(i) - res.v_mins(i);
    }

    // Collect active surface constraints
    std::vector<int> active_rows;
    Eigen::MatrixXd G_cbf = Eigen::MatrixXd::Zero(m, m);
    Eigen::VectorXd rhs_cbf = Eigen::VectorXd::Zero(m);

    int best_active_idx = -1;
    double min_zeta_act = std::numeric_limits<double>::infinity();

    for (int i = 0; i < m; ++i)
    {
      const double zeta_i = res.zetas(i);
      const double dzeta_i = res.dzetas(i);
      // Stay active below the surface (zeta_i < 0) as well: see solveDampingQP
      if (zeta_i <= config_.p_ths && dzeta_i < config_.v_ths)
      {
        active_rows.push_back(i);
        if (zeta_i < min_zeta_act)
        {
          min_zeta_act = zeta_i;
          best_active_idx = i;
        }
        const double dv_min_i = computeVelocityEnvelopeDerivative(zeta_i);
        const double B_i = res.barriers(i);
        const double a_drift_i =
            static_cast<double>(n6_vec[i].transpose() * Lambda_inv * F_ext);

        for (int j = 0; j < m; ++j)
        {
          const double M_ij = n6_vec[i].transpose() * Lambda_inv * n6_vec[j];
          G_cbf(i, j) = -M_ij * res.dzetas(j);
        }
        rhs_cbf(i) = dv_min_i * dzeta_i - a_drift_i - config_.gamma * B_i;
      }
    }

    const int m_act = static_cast<int>(active_rows.size());
    if (m_act > 0)
    {
      res.is_active = true;
      const int n_vars = m + m_act;
      Eigen::MatrixXd G = Eigen::MatrixXd::Zero(n_vars, n_vars);
      G.topLeftCorner(m, m) = Eigen::MatrixXd::Identity(m, m);
      G.bottomRightCorner(m_act, m_act) =
          config_.slack_quadratic_penalty * Eigen::MatrixXd::Identity(m_act, m_act);

      Eigen::VectorXd g0 = Eigen::VectorXd::Zero(n_vars);
      g0.tail(m_act).setConstant(config_.slack_linear_penalty);

      Eigen::MatrixXd A_ineq = Eigen::MatrixXd::Zero(m_act, n_vars);
      Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(m_act);
      for (int r = 0; r < m_act; ++r)
      {
        const int i = active_rows[static_cast<std::size_t>(r)];
        A_ineq.block(r, 0, 1, m) = G_cbf.row(i);
        A_ineq(r, m + r) = 1.0;
        b_ineq(r) = rhs_cbf(i);
      }

      Eigen::VectorXd lb = Eigen::VectorXd::Zero(n_vars);
      Eigen::VectorXd ub(n_vars);
      ub.head(m).setConstant(config_.d_max);
      ub.tail(m_act).setConstant(std::numeric_limits<double>::infinity());

      const Eigen::VectorXd z_sol = solveGeneralCBFQP(
          G, g0, A_ineq, b_ineq, lb, ub, &res.qp_cost);
      res.d_opt = z_sol.head(m);
      res.slack_eps = z_sol.tail(m_act).maxCoeff();
    }
    else
    {
      res.is_active = false;
      res.d_opt.setZero();
      res.slack_eps = 0.0;
      res.qp_cost = 0.0;
    }

    res.D_imp.setZero();
    res.F_imp.setZero();
    for (int i = 0; i < m; ++i)
    {
      const Eigen::Vector3d n_i = planes_[i].normal.normalized();
      res.D_imp += res.d_opt(i) * (n_i * n_i.transpose());
      res.F_imp += n_i * (res.d_opt(i) * res.dzetas(i));
    }
    res.f_imp_scalar = res.F_imp.norm();

    // Populate representative scalar telemetry
    const int rep_idx = (best_active_idx >= 0) ? best_active_idx : 0;
    res.zeta = res.zetas(rep_idx);
    res.dzeta = res.dzetas(rep_idx);
    res.v_min = res.v_mins(rep_idx);
    res.dv_min_dzeta = computeVelocityEnvelopeDerivative(res.zeta);
    res.barrier_B = res.barriers(rep_idx);
    res.d_cbf = res.d_opt(rep_idx);
    res.rho = computeEffectiveInertia(Lambda, planes_[rep_idx].normal);
    res.z_opt << res.d_cbf, res.slack_eps;
    res.Lg_B = -res.dzeta / res.rho;
    const double a_drift_rep =
        static_cast<double>(n6_vec[rep_idx].transpose() * Lambda_inv * F_ext);
    res.Lf_B = a_drift_rep - res.dv_min_dzeta * res.dzeta;
    res.dB_dt = res.Lf_B + res.Lg_B * res.d_cbf;

    if (cbf_result_out)
    {
      *cbf_result_out = res;
    }
    return res;
  }

  CBFStepResult DampingCBFController::solveMultiSurfaceDampingQP(
      const RobotState &state,
      CBFStepResult *cbf_result_out) const
  {
    const Eigen::Matrix<double, 6, 6> Lambda =
        computeCartesianInertia(state.M, state.J);
    return solveMultiSurfaceDampingQP(state.x, state.dx, Lambda, state.F_ext, cbf_result_out);
  }

  Eigen::Matrix3d DampingCBFController::solveMultiSurfaceDampingQP(
      const std::vector<Hyperplane> &planes,
      const Eigen::Vector3d &x,
      const Eigen::Vector3d &dx,
      const Eigen::Matrix<double, 6, 6> &Lambda,
      const Eigen::Matrix<double, 6, 1> &F_ext,
      Eigen::VectorXd *d_opt_out) const
  {
    DampingCBFController temp_ctrl = *this;
    temp_ctrl.setHyperplanes(planes);
    CBFStepResult res = temp_ctrl.solveMultiSurfaceDampingQP(x, dx, Lambda, F_ext);
    if (d_opt_out)
    {
      *d_opt_out = res.d_opt;
    }
    return res.D_imp;
  }

  Eigen::VectorXd DampingCBFController::computeControlTorque(
      const RobotState &state,
      const Eigen::Vector3d &x_ref,
      const Eigen::Vector3d &phi_ref,
      const Eigen::VectorXd &tau_null,
      CBFStepResult *cbf_result_out)
  {
    if (planes_.size() > 1)
    {
      return computeMultiSurfaceControlTorque(state, x_ref, phi_ref, tau_null, cbf_result_out);
    }

    const Eigen::Matrix<double, 6, 6> Lambda =
        computeCartesianInertia(state.M, state.J);
    const Eigen::Matrix<double, 6, 6> Gamma =
        computeCartesianCoriolis(state.M, state.C, state.J, state.dJ, Lambda);

    const double zeta = computeDistance(state.x);
    const double dzeta = computeNormalVelocity(state.dx);
    const double rho = computeEffectiveInertia(Lambda, plane_.normal);

    Eigen::Matrix<double, 6, 1> X_dot;
    X_dot.head<3>() = state.dx;
    X_dot.tail<3>() = state.dphi;

    Eigen::Matrix<double, 6, 1> X_tilde;
    X_tilde.head<3>() = state.x - x_ref;
    X_tilde.tail<3>() = state.phi - phi_ref;

    Eigen::Matrix<double, 6, 1> n6 = Eigen::Matrix<double, 6, 1>::Zero();
    n6.head<3>() = plane_.normal;

    // Nominal Cartesian impedance wrench: virtual spring and damping pulling toward reference
    const Eigen::Matrix<double, 6, 1> F_nominal =
        -(Gamma + config_.D) * X_dot - config_.K * X_tilde;
    // Include external force only if compensate_external_force is true; otherwise keep nominal impedance drift
    const Eigen::Matrix<double, 6, 1> F_drift =
        config_.compensate_external_force ? (F_nominal + state.F_ext) : F_nominal;
    const double a_drift = n6.transpose() * Lambda.ldlt().solve(F_drift);

    CBFStepResult cbf_res = solveDampingQP(zeta, dzeta, rho, a_drift);
    cbf_res.d_opt = Eigen::VectorXd::Constant(1, cbf_res.d_cbf);
    cbf_res.zetas = Eigen::VectorXd::Constant(1, cbf_res.zeta);
    cbf_res.dzetas = Eigen::VectorXd::Constant(1, cbf_res.dzeta);
    cbf_res.v_mins = Eigen::VectorXd::Constant(1, cbf_res.v_min);
    cbf_res.barriers = Eigen::VectorXd::Constant(1, cbf_res.barrier_B);

    if (cbf_result_out)
    {
      *cbf_result_out = cbf_res;
    }

    Eigen::Matrix<double, 6, 6> D_imp_6 = Eigen::Matrix<double, 6, 6>::Zero();
    D_imp_6.topLeftCorner<3, 3>() = cbf_res.D_imp;

    const Eigen::VectorXd coriolis_comp =
        state.J.transpose() * Lambda *
        (state.J * state.M.ldlt().solve(state.C) - state.dJ) * state.dq;

    const Eigen::VectorXd task_wrench_torque =
        state.J.transpose() *
        (-(Gamma + config_.D + D_imp_6) * X_dot - config_.K * X_tilde);

    const Eigen::VectorXd tau_n =
        (tau_null.size() == state.q.size()) ? tau_null : Eigen::VectorXd::Zero(state.q.size());

    return state.g + tau_n + coriolis_comp + task_wrench_torque;
  }

  Eigen::VectorXd DampingCBFController::computeMultiSurfaceControlTorque(
      const RobotState &state,
      const Eigen::Vector3d &x_ref,
      const Eigen::Vector3d &phi_ref,
      const Eigen::VectorXd &tau_null,
      CBFStepResult *cbf_result_out) const
  {
    const Eigen::Matrix<double, 6, 6> Lambda =
        computeCartesianInertia(state.M, state.J);
    const Eigen::Matrix<double, 6, 6> Gamma =
        computeCartesianCoriolis(state.M, state.C, state.J, state.dJ, Lambda);

    Eigen::Matrix<double, 6, 1> X_dot;
    X_dot.head<3>() = state.dx;
    X_dot.tail<3>() = state.dphi;

    Eigen::Matrix<double, 6, 1> X_tilde;
    X_tilde.head<3>() = state.x - x_ref;
    X_tilde.tail<3>() = state.phi - phi_ref;

    // Nominal Cartesian impedance wrench driving the end-effector
    const Eigen::Matrix<double, 6, 1> F_nominal =
        -(Gamma + config_.D) * X_dot - config_.K * X_tilde;
    // Include external force only if compensate_external_force is true; otherwise keep nominal impedance drift
    const Eigen::Matrix<double, 6, 1> F_drift =
        config_.compensate_external_force ? (F_nominal + state.F_ext) : F_nominal;

    // Solve multi-surface CBF-QP using preloaded hyperplanes
    CBFStepResult cbf_res = solveMultiSurfaceDampingQP(state.x, state.dx, Lambda, F_drift);

    if (cbf_result_out)
    {
      *cbf_result_out = cbf_res;
    }

    Eigen::Matrix<double, 6, 6> D_imp_6 = Eigen::Matrix<double, 6, 6>::Zero();
    D_imp_6.topLeftCorner<3, 3>() = cbf_res.D_imp;

    // Operational-space dynamic Coriolis compensation (Eq. 3)
    const Eigen::VectorXd coriolis_comp =
        state.J.transpose() * Lambda *
        (state.J * state.M.ldlt().solve(state.C) - state.dJ) * state.dq;

    // Total Cartesian task wrench torque (Eq. 3)
    const Eigen::VectorXd task_wrench_torque =
        state.J.transpose() *
        (-(Gamma + config_.D + D_imp_6) * X_dot - config_.K * X_tilde);

    const Eigen::VectorXd tau_n =
        (tau_null.size() == state.q.size()) ? tau_null : Eigen::VectorXd::Zero(state.q.size());

    return state.g + tau_n + coriolis_comp + task_wrench_torque;
  }

  Eigen::VectorXd DampingCBFController::computeMultiSurfaceControlTorque(
      const RobotState &state,
      const std::vector<Hyperplane> &planes,
      const Eigen::Vector3d &x_ref,
      const Eigen::Vector3d &phi_ref,
      const Eigen::VectorXd &tau_null,
      Eigen::Matrix3d *D_imp_out,
      Eigen::VectorXd *d_opt_out) const
  {
    DampingCBFController temp_ctrl = *this;
    temp_ctrl.setHyperplanes(planes);
    CBFStepResult cbf_res;
    const Eigen::VectorXd tau = temp_ctrl.computeMultiSurfaceControlTorque(
        state, x_ref, phi_ref, tau_null, &cbf_res);

    if (D_imp_out)
    {
      *D_imp_out = cbf_res.D_imp;
    }
    if (d_opt_out)
    {
      *d_opt_out = cbf_res.d_opt;
    }
    return tau;
  }

} // namespace damping_cbf
