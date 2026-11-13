#include "jtorque_cbf/joint_torque_cbf_controller.hpp"

// See header: hide Simulink's -DRT from Boost.Math while including Pinocchio
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
#include <stdexcept>

namespace jtorque_cbf
{

  namespace
  {
    constexpr double kInf = std::numeric_limits<double>::infinity();
  }

  JointTorqueCBFController::JointTorqueCBFController(const JointTorqueCBFConfig &config,
                                                     const Hyperplane &plane)
  {
    setConfig(config);
    setHyperplane(plane);
  }

  JointTorqueCBFController::JointTorqueCBFController(const JointTorqueCBFConfig &config,
                                                     const std::vector<Hyperplane> &planes)
  {
    setConfig(config);
    setHyperplanes(planes);
  }

  void JointTorqueCBFController::setConfig(const JointTorqueCBFConfig &c)
  {
    if (c.p_ths <= 0.0) throw std::invalid_argument("p_ths must be strictly positive.");
    if (c.v_ths >= 0.0) throw std::invalid_argument("v_ths must be strictly negative.");
    if (c.v_0 >= c.v_ths) throw std::invalid_argument("v_0 must be strictly more negative than v_ths.");
    if (c.gamma <= 0.0) throw std::invalid_argument("gamma must be strictly positive.");
    if (c.w_slack <= 0.0 || c.w_lin <= 0.0) throw std::invalid_argument("w_slack and w_lin must be strictly positive.");
    if (c.dt <= 0.0) throw std::invalid_argument("dt must be strictly positive.");
    if (c.zone_exit_hysteresis < 0.0) throw std::invalid_argument("zone_exit_hysteresis must be non-negative.");
    if (!(c.latch_velocity_filter_alpha > 0.0 && c.latch_velocity_filter_alpha <= 1.0))
      throw std::invalid_argument("latch_velocity_filter_alpha must be in (0, 1].");
    if (!(c.max_lateral_force >= 0.0) || !(c.max_moment >= 0.0))
      throw std::invalid_argument("max_lateral_force and max_moment must be non-negative.");
    if (!(c.w_lateral > 0.0) || !(c.w_moment > 0.0) || !(c.w_null > 0.0) || !(c.w_reg >= 0.0))
      throw std::invalid_argument("w_lateral, w_moment, w_null must be > 0 and w_reg >= 0.");
    if (!(c.guarantee_slack_factor >= 1.0)) throw std::invalid_argument("guarantee_slack_factor must be >= 1.");
    if (!(c.tau_rate_limit >= 0.0)) throw std::invalid_argument("tau_rate_limit must be non-negative (0 = disabled).");
    if (c.default_tau_limit <= 0.0) throw std::invalid_argument("default_tau_limit must be strictly positive.");
    config_ = c;
    ensureEntryStateSize();
    for (std::size_t i = 0; i < active_v0_.size(); ++i)
    {
      updateActiveV0(i);
    }
  }

  void JointTorqueCBFController::setHyperplane(const Hyperplane &plane)
  {
    setHyperplanes({plane});
  }

  void JointTorqueCBFController::setHyperplanes(const std::vector<Hyperplane> &planes)
  {
    std::vector<Hyperplane> normalized;
    normalized.reserve(planes.size());
    for (const auto &p : planes)
    {
      const double nrm = p.normal.norm();
      if (nrm < 1e-12) throw std::invalid_argument("Hyperplane normal vector must be non-zero.");
      Hyperplane h;
      h.normal = p.normal / nrm;
      h.offset = p.offset;
      normalized.push_back(h);
    }
    assignPlanes(std::move(normalized));
  }

  void JointTorqueCBFController::assignPlanes(std::vector<Hyperplane> new_planes)
  {
    bool same = (new_planes.size() == planes_.size());
    for (std::size_t i = 0; same && i < new_planes.size(); ++i)
    {
      same = (new_planes[i].normal == planes_[i].normal) && (new_planes[i].offset == planes_[i].offset);
    }
    planes_ = std::move(new_planes);
    if (!same)
    {
      resetEntryState();
      resetRateState();
    }
  }

  void JointTorqueCBFController::ensureEntryStateSize()
  {
    const std::size_t m = planes_.size();
    if (active_v0_.size() != m || was_in_zone_.size() != m || has_entry_velocity_.size() != m ||
        entry_dzeta_.size() != m || dzeta_filt_init_.size() != m || dzeta_filt_.size() != m)
    {
      resetEntryState();
    }
  }

  void JointTorqueCBFController::updateActiveV0(std::size_t i)
  {
    active_v0_[i] = (config_.latch_entry_velocity && has_entry_velocity_[i])
                        ? std::min(entry_dzeta_[i], config_.v_0)
                        : config_.v_0;
  }

  void JointTorqueCBFController::setCustomProfile(std::function<double(double)> h_func,
                                                  std::function<double(double)> H_func)
  {
    custom_h_ = std::move(h_func);
    custom_H_ = std::move(H_func);
    config_.profile = ProfileType::CUSTOM;
  }

  void JointTorqueCBFController::resetEntryState()
  {
    const std::size_t m = planes_.size();
    active_v0_.assign(m, config_.v_0);
    was_in_zone_.assign(m, false);
    has_entry_velocity_.assign(m, false);
    entry_dzeta_.assign(m, 0.0);
    dzeta_filt_init_.assign(m, false);
    dzeta_filt_.assign(m, 0.0);
  }

  void JointTorqueCBFController::resetRateState()
  {
    has_tau_prev_ = false;
    tau_total_prev_.resize(0);
  }

  double JointTorqueCBFController::computeVelocityEnvelope(double zeta, std::size_t i) const
  {
    const double v0 = (i < active_v0_.size()) ? active_v0_[i] : config_.v_0;
    return cbf::computeVelocityEnvelope(zeta, config_.p_ths, v0, config_.v_ths, config_.profile, custom_H_);
  }

  double JointTorqueCBFController::computeVelocityEnvelopeDerivative(double zeta, std::size_t i) const
  {
    const double v0 = (i < active_v0_.size()) ? active_v0_[i] : config_.v_0;
    return cbf::computeVelocityEnvelopeDerivative(zeta, config_.p_ths, v0, config_.v_ths, config_.profile,
                                                  custom_h_, custom_H_);
  }

  Eigen::VectorXd JointTorqueCBFController::computeControlTorque(const RobotState &s,
                                                                 const Eigen::VectorXd &tau_nom,
                                                                 JointTorqueCBFResult *result_out)
  {
    const Eigen::Index n = s.q.size();
    const Eigen::VectorXd Minv_C_dq = s.M.ldlt().solve(s.C * s.dq);
    const Eigen::Matrix<double, 6, 1> drift_cart = s.dJ * s.dq - s.J * Minv_C_dq;

    JointTorqueCBFResult res = solveQP(s.M, s.J, drift_cart, s.F_ext, s.x, s.dx, s.dq, tau_nom, s.g);
    const Eigen::VectorXd tau_nom_eff = (tau_nom.size() == n) ? tau_nom : Eigen::VectorXd::Zero(n);
    res.tau_cmd = s.g + tau_nom_eff + res.tau_cbf;
    if (result_out) *result_out = res;
    return res.tau_cmd;
  }

  Eigen::VectorXd JointTorqueCBFController::computeControlTorque(
      const pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &q,
      const Eigen::VectorXd &dq, pinocchio::FrameIndex frame_id, const Eigen::Matrix<double, 6, 1> &F_ext,
      const Eigen::VectorXd &tau_nom, JointTorqueCBFResult *result_out)
  {
    if (frame_id >= model.frames.size())
    {
      throw std::runtime_error("computeControlTorque: frame_id not found in Pinocchio model.");
    }
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
    pinocchio::updateFramePlacements(model, data);

    RobotState s;
    s.q = q;
    s.dq = dq;
    s.x = data.oMf[frame_id].translation();
    s.J = Eigen::MatrixXd::Zero(6, model.nv);
    s.dJ = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, s.J);
    pinocchio::getFrameJacobianTimeVariation(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, s.dJ);
    const Eigen::Matrix<double, 6, 1> V = s.J * dq;
    s.dx = V.head<3>();
    // Pinocchio's LOCAL_WORLD_ALIGNED dJ gives the SPATIAL acceleration; make it the true time derivative
    // of J (classical acceleration of the frame origin): dJ_lin += -[v]x * J_ang
    Eigen::Matrix3d v_skew;
    v_skew << 0.0, -s.dx.z(), s.dx.y(), s.dx.z(), 0.0, -s.dx.x(), -s.dx.y(), s.dx.x(), 0.0;
    s.dJ.topRows<3>() -= v_skew * s.J.bottomRows<3>();

    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeCoriolisMatrix(model, data, q, dq);
    pinocchio::computeGeneralizedGravity(model, data, q);
    s.M = data.M;
    s.C = data.C;
    s.g = data.g;
    s.F_ext = F_ext;
    return computeControlTorque(s, tau_nom, result_out);
  }

  JointTorqueCBFResult JointTorqueCBFController::solveQP(
      const Eigen::MatrixXd &M, const Eigen::MatrixXd &J, const Eigen::Matrix<double, 6, 1> &drift_cart,
      const Eigen::Matrix<double, 6, 1> &F_ext, const Eigen::Vector3d &x_ee, const Eigen::Vector3d &dx_ee,
      const Eigen::VectorXd &dq, const Eigen::VectorXd &tau_nom, const Eigen::VectorXd &g)
  {
    const Eigen::Index n = M.rows();
    const Eigen::Index m = static_cast<Eigen::Index>(planes_.size());
    if (m == 0) throw std::runtime_error("solveQP: no hyperplanes configured.");
    if (J.rows() != 6 || J.cols() != n || dq.size() != n)
      throw std::invalid_argument("solveQP: inconsistent dimensions of M, J, dq.");
    ensureEntryStateSize();

    const Eigen::VectorXd tau_nom_eff = (tau_nom.size() == n) ? tau_nom : Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd g_eff = (g.size() == n) ? g : Eigen::VectorXd::Zero(n);

    JointTorqueCBFResult res;
    res.tau_cbf = Eigen::VectorXd::Zero(n);
    res.tau_cmd = Eigen::VectorXd::Zero(n);
    res.f_n = Eigen::VectorXd::Zero(m);
    res.f_brake_max = Eigen::VectorXd::Constant(m, kInf);
    res.rho = Eigen::VectorXd::Zero(m);
    res.zetas.resize(m);
    res.dzetas.resize(m);
    res.v_mins.resize(m);
    res.barriers.resize(m);
    res.b_cbf.resize(m);
    res.a_drift.resize(m);
    res.active_planes.assign(static_cast<std::size_t>(m), false);

    // ---------------- Dynamics in task space ----------------
    const auto M_ldlt = M.ldlt();
    const Eigen::MatrixXd Minv_JT = M_ldlt.solve(J.transpose()); // n x 6
    const Eigen::MatrixXd JMinv = Minv_JT.transpose();           // 6 x n  (= J M^-1, M symmetric)
    Eigen::Matrix<double, 6, 6> Lambda_inv = J * Minv_JT;
    Lambda_inv = 0.5 * (Lambda_inv + Lambda_inv.transpose());
    const Eigen::Matrix<double, 6, 6> Lambda = Lambda_inv.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    const Eigen::MatrixXd A_F = Lambda * JMinv;                                 // F = A_F tau (6 x n)
    const Eigen::MatrixXd N_tau = Eigen::MatrixXd::Identity(n, n) - J.transpose() * A_F; // nullspace part

    const Eigen::Matrix<double, 6, 1> nom_accel = JMinv * tau_nom_eff;
    const Eigen::Matrix<double, 6, 1> ext_accel =
        config_.compensate_external_force ? Eigen::Matrix<double, 6, 1>(Lambda_inv * F_ext)
                                          : Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix<double, 6, 1> a_total = drift_cart + nom_accel + ext_accel;

    // Normals, normal-force extraction f_n = P_n F_lin and lateral projector P_lat
    Eigen::MatrixXd N6 = Eigen::MatrixXd::Zero(6, m);
    for (Eigen::Index i = 0; i < m; ++i)
    {
      N6.block<3, 1>(0, i) = planes_[static_cast<std::size_t>(i)].normal;
    }
    const Eigen::MatrixXd N_lin = N6.topRows(3);
    const Eigen::MatrixXd P_n = N_lin.completeOrthogonalDecomposition().pseudoInverse(); // m x 3
    const Eigen::Matrix3d P_span = N_lin * P_n;                                           // onto span(normals)
    const Eigen::Matrix3d P_lat = Eigen::Matrix3d::Identity() - P_span;

    // Torque-rate -> normal-force rate (for the pushback-aware cap), as in torque_cbf
    const bool rate_on = config_.tau_rate_limit > 0.0;
    const Eigen::MatrixXd K = J.transpose() * N6; // n x m

    // ---------------- Per-plane barrier rows and caps ----------------
    int best = 0;
    double max_b = -kInf;
    std::vector<Eigen::Index> zone;
    for (Eigen::Index i = 0; i < m; ++i)
    {
      const std::size_t si = static_cast<std::size_t>(i);
      const Eigen::Matrix<double, 6, 1> n6 = N6.col(i);
      const double zeta = planes_[si].normal.dot(x_ee - planes_[si].offset);
      const double dzeta = planes_[si].normal.dot(dx_ee);
      const bool in_zone = (zeta <= config_.p_ths && zeta >= -0.05);

      // Entry latch (identical to torque_cbf)
      if (!dzeta_filt_init_[si])
      {
        dzeta_filt_[si] = dzeta;
        dzeta_filt_init_[si] = true;
      }
      else
      {
        dzeta_filt_[si] += config_.latch_velocity_filter_alpha * (dzeta - dzeta_filt_[si]);
      }
      const bool in_latch_zone = was_in_zone_[si] ? (zeta <= config_.p_ths + config_.zone_exit_hysteresis)
                                                  : (zeta <= config_.p_ths);
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
      updateActiveV0(si);

      const double v_min = computeVelocityEnvelope(zeta, si);
      const double dv_min = computeVelocityEnvelopeDerivative(zeta, si);
      const double B = dzeta - v_min;
      const double a_drift = n6.dot(a_total);
      res.rho(i) = 1.0 / std::max(1e-9, static_cast<double>(n6.transpose() * Lambda_inv * n6));

      // One-way normal-force cap (as torque_cbf) + pushback-aware cap under a rate limit
      double cap = kInf;
      if (config_.enforce_passivity)
      {
        if (dzeta <= 0.0)
        {
          const double a_cancel = config_.brake_cap_include_drift ? std::max(0.0, -a_drift) : 0.0;
          cap = res.rho(i) * (std::abs(dzeta) / config_.dt + a_cancel);
          if (rate_on)
          {
            const double k_max = std::max(1e-9, K.col(i).cwiseAbs().maxCoeff());
            const double r = config_.tau_rate_limit / (k_max * static_cast<double>(m));
            const double f_hold = res.rho(i) * std::max(0.0, -a_drift);
            cap = std::min(cap, f_hold + std::sqrt(2.0 * r * res.rho(i) * std::abs(dzeta)));
          }
        }
        else
        {
          cap = 0.0;
        }
      }
      res.f_brake_max(i) = cap;

      double b = -1e3; // sentinel (no row) outside the zone
      if (in_zone)
      {
        b = dv_min * dzeta - a_drift - config_.gamma * B;
        if (config_.enforce_passivity && std::isfinite(cap))
        {
          b = std::min(b, cap / res.rho(i));
        }
        res.active_planes[si] = (b > 0.0);
        zone.push_back(i);
      }

      res.zetas(i) = zeta;
      res.dzetas(i) = dzeta;
      res.v_mins(i) = v_min;
      res.barriers(i) = B;
      res.b_cbf(i) = b;
      res.a_drift(i) = a_drift;
      if (b > max_b)
      {
        max_b = b;
        best = static_cast<int>(i);
      }
    }
    res.critical_plane_idx = best;
    res.zeta = res.zetas(best);
    res.dzeta = res.dzetas(best);
    res.v_min = res.v_mins(best);
    res.dv_min_dzeta = computeVelocityEnvelopeDerivative(res.zeta, static_cast<std::size_t>(best));
    res.barrier_B = res.barriers(best);
    res.b_cbf_scalar = res.b_cbf(best);
    res.a_drift_scalar = res.a_drift(best);
    res.is_active = (max_b > 0.0);

    // ---------------- Actuator box on tau_cbf (hard) ----------------
    const Eigen::VectorXd tau_min_abs =
        (config_.tau_min.size() == n) ? config_.tau_min : Eigen::VectorXd::Constant(n, -config_.default_tau_limit);
    const Eigen::VectorXd tau_max_abs =
        (config_.tau_max.size() == n) ? config_.tau_max : Eigen::VectorXd::Constant(n, config_.default_tau_limit);
    Eigen::VectorXd lim_lo = tau_min_abs;
    Eigen::VectorXd lim_hi = tau_max_abs;
    if (config_.enforce_total_torque_limits)
    {
      const Eigen::VectorXd base = g_eff + tau_nom_eff;
      lim_lo = (tau_min_abs - base).cwiseMin(0.0);
      lim_hi = (tau_max_abs - base).cwiseMax(0.0);
    }
    Eigen::VectorXd lo = lim_lo;
    Eigen::VectorXd hi = lim_hi;
    Eigen::VectorXd rate_lo = Eigen::VectorXd::Constant(n, -kInf);
    Eigen::VectorXd rate_hi = Eigen::VectorXd::Constant(n, kInf);
    if (rate_on)
    {
      // Reachable in one step: |tau_nom + tau - prev_total| <= rate*dt (prev_total = tau_nom if no history)
      const Eigen::VectorXd prev_total =
          (has_tau_prev_ && tau_total_prev_.size() == n) ? tau_total_prev_ : tau_nom_eff;
      const double d = config_.tau_rate_limit * config_.dt;
      rate_lo = prev_total - tau_nom_eff - Eigen::VectorXd::Constant(n, d);
      rate_hi = prev_total - tau_nom_eff + Eigen::VectorXd::Constant(n, d);
      for (Eigen::Index j = 0; j < n; ++j)
      {
        lo(j) = std::max(lim_lo(j), rate_lo(j));
        hi(j) = std::min(lim_hi(j), rate_hi(j));
        if (lo(j) > hi(j))
        {
          // Limits unreachable within one step: physics wins, move toward them as fast as possible
          const double v = (rate_hi(j) < lim_lo(j)) ? rate_hi(j) : rate_lo(j);
          lo(j) = v;
          hi(j) = v;
        }
      }
    }

    // Normal-equivalent force f_eq,i = rho_i n_i^T J M^-1 tau (normal acceleration times rho, all sources)
    Eigen::MatrixXd Feq_rows(m, n);
    for (Eigen::Index i = 0; i < m; ++i)
    {
      Feq_rows.row(i) = res.rho(i) * (N6.col(i).transpose() * JMinv);
    }

    auto finish = [&](const Eigen::VectorXd &tau) {
      res.tau_cbf = tau;
      res.F_cbf = A_F * tau;
      res.f_n = Feq_rows * tau;
      res.f_n_scalar = res.f_n(best);
      res.F_lateral = (P_lat * res.F_cbf.head<3>()).cwiseAbs().maxCoeff();
      res.M_norm = res.F_cbf.tail<3>().cwiseAbs().maxCoeff();
      res.power = tau.dot(dq);
      if (rate_on)
      {
        for (Eigen::Index j = 0; j < n; ++j)
        {
          const double tol = 1e-9 * std::max(1.0, std::abs(tau(j)));
          if ((rate_lo(j) >= lim_lo(j) && tau(j) <= rate_lo(j) + tol) ||
              (rate_hi(j) <= lim_hi(j) && tau(j) >= rate_hi(j) - tol))
          {
            res.rate_limited = true;
          }
        }
      }
      tau_total_prev_ = tau_nom_eff + tau;
      has_tau_prev_ = true;
    };

    // Fast path: nothing to enforce and zero torque reachable
    const bool zero_in_box = (lo.array() <= 0.0).all() && (hi.array() >= 0.0).all();
    if (!res.is_active && zero_in_box)
    {
      res.qp_success = true;
      finish(Eigen::VectorXd::Zero(n));
      return res;
    }

    // ---------------- QP in z = [tau (n), delta] ----------------
    const Eigen::Index nz = n + 1;
    const Eigen::Index id = n;
    const Eigen::VectorXd tau_zero = Eigen::VectorXd::Zero(n).cwiseMax(lo).cwiseMin(hi);

    Eigen::Matrix<double, 6, 6> W_F = Eigen::Matrix<double, 6, 6>::Zero();
    W_F.topLeftCorner<3, 3>() = P_span + config_.w_lateral * P_lat;
    W_F.bottomRightCorner<3, 3>() = config_.w_moment * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nz, nz);
    G.topLeftCorner(n, n) = A_F.transpose() * W_F * A_F + config_.w_null * N_tau.transpose() * N_tau +
                            config_.w_reg * Eigen::MatrixXd::Identity(n, n);
    G.topLeftCorner(n, n) = 0.5 * (G.topLeftCorner(n, n) + G.topLeftCorner(n, n).transpose());
    G(id, id) = config_.w_slack;
    Eigen::VectorXd g0 = Eigen::VectorXd::Zero(nz);
    g0(id) = config_.w_lin;

    std::vector<Eigen::VectorXd> rows;
    std::vector<double> rhs;
    auto add_barrier_row = [&](const Eigen::VectorXd &a_tau, double b) {
      Eigen::VectorXd r = Eigen::VectorXd::Zero(nz);
      r.head(n) = a_tau;
      r(id) = 1.0;
      rows.push_back(r);
      rhs.push_back(b);
    };
    auto add_guarantee_row = [&](const Eigen::VectorXd &a_tau, double b, double &sigma_out) {
      const double val_zero = a_tau.dot(tau_zero);
      double b_eff = b;
      if (val_zero < b)
      {
        sigma_out = std::max(sigma_out, b - val_zero);
        b_eff = val_zero - 1e-7;
      }
      Eigen::VectorXd r = Eigen::VectorXd::Zero(nz);
      r.head(n) = a_tau;
      rows.push_back(r);
      rhs.push_back(b_eff);
    };

    // Barrier rows: n_i^T J M^-1 tau + delta >= b_i
    for (Eigen::Index i : zone)
    {
      add_barrier_row(JMinv.transpose() * N6.col(i), res.b_cbf(i));
    }

    if (config_.enforce_passivity)
    {
      // One-way normal action 0 <= f_eq,i <= cap_i (no pushback, also not through lateral forces / moments)
      for (Eigen::Index i = 0; i < m; ++i)
      {
        add_guarantee_row(Feq_rows.row(i).transpose(), 0.0, res.sigma_normal);
        if (std::isfinite(res.f_brake_max(i)))
        {
          const double cap_i = std::max(1e-7, res.f_brake_max(i));
          add_guarantee_row(-Feq_rows.row(i).transpose(), -cap_i, res.sigma_normal);
        }
      }
      // CBF power tau^T dq <= 0
      add_guarantee_row(-dq, 0.0, res.sigma_power);
    }

    // Bounded lateral force and moment: |row * tau| <= eps
    const double eps_lat = std::max(1e-7, config_.max_lateral_force);
    const double eps_rot = std::max(1e-7, config_.max_moment);
    const Eigen::MatrixXd Lat = P_lat * A_F.topRows(3);
    for (Eigen::Index k = 0; k < 3; ++k)
    {
      if (Lat.row(k).norm() > 1e-9)
      {
        add_guarantee_row(Lat.row(k).transpose(), -eps_lat, res.sigma_lateral);
        add_guarantee_row(-Lat.row(k).transpose(), -eps_lat, res.sigma_lateral);
      }
      add_guarantee_row(A_F.row(3 + k).transpose(), -eps_rot, res.sigma_lateral);
      add_guarantee_row(-A_F.row(3 + k).transpose(), -eps_rot, res.sigma_lateral);
    }

    Eigen::MatrixXd A_ineq(static_cast<Eigen::Index>(rows.size()), nz);
    Eigen::VectorXd b_ineq(static_cast<Eigen::Index>(rows.size()));
    for (std::size_t r = 0; r < rows.size(); ++r)
    {
      A_ineq.row(static_cast<Eigen::Index>(r)) = rows[r].transpose();
      b_ineq(static_cast<Eigen::Index>(r)) = rhs[r];
    }

    Eigen::VectorXd lb(nz), ub(nz);
    lb.head(n) = lo;
    ub.head(n) = hi;
    lb(id) = 0.0;
    ub(id) = kInf;

    Eigen::VectorXd z = Eigen::VectorXd::Zero(nz);
    const double cost = cbf::solve_quadprog_box(G, g0, A_ineq, b_ineq, lb, ub, z);
    if (std::isfinite(cost))
    {
      res.qp_success = true;
      res.qp_cost = cost;
      res.slack_delta = std::max(0.0, z(id));
      finish(z.head(n));
    }
    else
    {
      // Fallback: the reachable torque closest to zero (no CBF action beyond what the actuator forces)
      res.qp_success = false;
      res.qp_cost = cost;
      finish(tau_zero);
    }
    return res;
  }

} // namespace jtorque_cbf
