#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cbf {

/**
 * @brief Goldfarb-Idnani dual active-set Quadratic Programming solver (quadprog).
 *
 * Reference:
 *   D. Goldfarb and A. Idnani, "A numerically stable dual method for solving
 *   strictly convex quadratic programs," Mathematical Programming, vol. 27,
 *   pp. 1-33, 1983.
 *
 * Solves the strictly convex Quadratic Program:
 *
 *   min_{x in R^n}   0.5 * x^T * G * x + g0^T * x
 *   subject to       CE^T * x + ce0  = 0   (me equality constraints)
 *                    CI^T * x + ci0 >= 0   (mi inequality constraints)
 *
 * Matrix dimensions:
 *   - G   : n x n symmetric positive-definite Hessian matrix (G > 0)
 *   - g0  : n x 1 gradient vector
 *   - CE  : n x me matrix of equality constraint normals (each column is a constraint)
 *   - ce0 : me x 1 vector of equality constraint offsets
 *   - CI  : n x mi matrix of inequality constraint normals (each column is a constraint)
 *   - ci0 : mi x 1 vector of inequality constraint offsets
 *   - x   : n x 1 output optimal decision vector
 *
 * @return Optimal objective value 0.5 * x^T * G * x + g0^T * x, or
 *         std::numeric_limits<double>::infinity() if the QP is infeasible.
 */
inline double solve_quadprog(const Eigen::MatrixXd& G,
                             const Eigen::VectorXd& g0,
                             const Eigen::MatrixXd& CE,
                             const Eigen::VectorXd& ce0,
                             const Eigen::MatrixXd& CI,
                             const Eigen::VectorXd& ci0,
                             Eigen::VectorXd& x) {
  const Eigen::Index n = G.rows();
  const Eigen::Index me = CE.cols();
  const Eigen::Index mi = CI.cols();

  if (G.cols() != n || g0.size() != n) {
    throw std::invalid_argument("solve_quadprog: inconsistent dimensions for G and g0.");
  }
  if ((me > 0 && CE.rows() != n) || ce0.size() != me) {
    throw std::invalid_argument("solve_quadprog: inconsistent dimensions for CE and ce0.");
  }
  if ((mi > 0 && CI.rows() != n) || ci0.size() != mi) {
    throw std::invalid_argument("solve_quadprog: inconsistent dimensions for CI and ci0.");
  }

  constexpr double kEps = 1e-12;
  const double inf = std::numeric_limits<double>::infinity();

  // 1. Cholesky factorization of SPD Hessian G = L * L^T
  Eigen::LLT<Eigen::MatrixXd> llt_G(G);
  if (llt_G.info() != Eigen::Success) {
    throw std::runtime_error("solve_quadprog: Hessian matrix G is not positive definite.");
  }
  const Eigen::MatrixXd G_inv = llt_G.solve(Eigen::MatrixXd::Identity(n, n));

  // 2. Unconstrained minimum: x = -G^{-1} * g0
  x = -G_inv * g0;

  // Active set indices:
  //   First me entries (if any) are equality constraints; subsequent entries
  //   store indices k in [0, mi - 1] of active inequality constraints.
  std::vector<Eigen::Index> active_ineq;
  active_ineq.reserve(static_cast<std::size_t>(mi));
  std::vector<bool> is_active(static_cast<std::size_t>(mi), false);

  // Enforce equality constraints CE^T * x + ce0 = 0 first
  Eigen::VectorXd u_eq = Eigen::VectorXd::Zero(me);
  if (me > 0) {
    const Eigen::MatrixXd N_eq_Ginv_Neq = CE.transpose() * G_inv * CE;
    Eigen::LDLT<Eigen::MatrixXd> ldlt_eq(N_eq_Ginv_Neq);
    if (ldlt_eq.info() != Eigen::Success) {
      return inf;
    }
    const Eigen::VectorXd s_eq = CE.transpose() * x + ce0;
    u_eq = ldlt_eq.solve(s_eq);
    x -= G_inv * CE * u_eq;
    if ((CE.transpose() * x + ce0).lpNorm<Eigen::Infinity>() > 1e-7) {
      return inf;
    }
  }

  // Dual multipliers for active inequality constraints
  Eigen::VectorXd u_ineq = Eigen::VectorXd::Zero(0);

  // Helper to assemble active normal matrix N_A = [CE, CI(:, active_ineq)]
  auto buildActiveNormals = [&]() -> Eigen::MatrixXd {
    const Eigen::Index q_ineq = static_cast<Eigen::Index>(active_ineq.size());
    Eigen::MatrixXd N_A(n, me + q_ineq);
    if (me > 0) {
      N_A.leftCols(me) = CE;
    }
    for (Eigen::Index j = 0; j < q_ineq; ++j) {
      N_A.col(me + j) = CI.col(active_ineq[static_cast<std::size_t>(j)]);
    }
    return N_A;
  };

  constexpr int kMaxIter = 1000;
  for (int iter = 0; iter < kMaxIter; ++iter) {
    // 3. Find most violated inequality constraint: k* = argmin_k (CI(:,k)^T * x + ci0(k))
    Eigen::Index most_violated_k = -1;
    double min_violation = -kEps;

    for (Eigen::Index k = 0; k < mi; ++k) {
      if (is_active[static_cast<std::size_t>(k)]) {
        continue;
      }
      const double val = CI.col(k).dot(x) + ci0(k);
      if (val < min_violation) {
        min_violation = val;
        most_violated_k = k;
      }
    }

    // No violated constraints -> current primal point x is optimal!
    if (most_violated_k < 0) {
      return 0.5 * x.dot(G * x) + g0.dot(x);
    }

    const Eigen::VectorXd n_plus = CI.col(most_violated_k);
    const double c0_plus = ci0(most_violated_k);

    double u_plus = 0.0;  // multiplier of the constraint being added
    // 4. Dual active-set inner loop: step towards the violated constraint hyperplane
    while (true) {
      const Eigen::Index q_ineq = static_cast<Eigen::Index>(active_ineq.size());
      const Eigen::Index q_total = me + q_ineq;

      Eigen::VectorXd z(n);
      Eigen::VectorXd r(q_total);

      if (q_total >= n) {
        // Active set already spans R^n: n_plus is linearly dependent -> no primal direction.
        z.setZero();
        const Eigen::MatrixXd N_A = buildActiveNormals();
        r = N_A.colPivHouseholderQr().solve(n_plus);
      } else if (q_total == 0) {
        // Unconstrained step: z = G^{-1} * n_plus
        z = G_inv * n_plus;
      } else {
        const Eigen::MatrixXd N_A = buildActiveNormals();
        const Eigen::MatrixXd N_A_T_Ginv = N_A.transpose() * G_inv;
        const Eigen::MatrixXd M_A = N_A_T_Ginv * N_A;

        Eigen::LDLT<Eigen::MatrixXd> ldlt_M(M_A);
        if (ldlt_M.info() != Eigen::Success) {
          return inf; // Rank deficient active set
        }

        const Eigen::VectorXd rhs_r = N_A.transpose() * G_inv * n_plus;
        r = ldlt_M.solve(rhs_r);
        z = G_inv * (n_plus - N_A * r);
      }

      const double z_norm_sq = z.squaredNorm();

      // Check if step direction z is non-zero
      double t1 = inf;
      if (z_norm_sq > kEps * kEps * std::max(1.0, n_plus.squaredNorm())) {
        const double n_plus_dot_z = n_plus.dot(z);
        if (n_plus_dot_z > kEps) {
          t1 = -(n_plus.dot(x) + c0_plus) / n_plus_dot_z;
        }
      }

      // Check maximum step length t2 preserving dual feasibility (u_ineq >= 0)
      double t2 = inf;
      Eigen::Index drop_idx = -1;
      for (Eigen::Index j = 0; j < q_ineq; ++j) {
        const double r_val = r(me + j);
        if (r_val > kEps) {
          const double ratio = u_ineq(j) / r_val;
          if (ratio < t2) {
            t2 = ratio;
            drop_idx = j;
          }
        }
      }

      // Neither primal step nor dual step possible -> QP is infeasible
      if (std::isinf(t1) && std::isinf(t2)) {
        return inf;
      }

      // Case A: Full primal step (t1 <= t2)
      if (t1 <= t2) {
        x += t1 * z;
        if (q_ineq > 0) {
          u_ineq -= t1 * r.tail(q_ineq);
        }
        // Append newly active constraint
        active_ineq.push_back(most_violated_k);
        is_active[static_cast<std::size_t>(most_violated_k)] = true;

        Eigen::VectorXd u_new(q_ineq + 1);
        if (q_ineq > 0) {
          u_new.head(q_ineq) = u_ineq;
        }
        u_new(q_ineq) = u_plus + t1;
        u_ineq = u_new;
        break; // Inner loop finished, check next violation
      }

      // Case B: Partial step (t2 < t1) -> drop a blocking active constraint
      x += t2 * z;
      if (q_ineq > 0) {
        u_ineq -= t2 * r.tail(q_ineq);
      }
      u_plus += t2;

      // Drop constraint at drop_idx from the active set
      if (drop_idx >= 0 && drop_idx < q_ineq) {
        const Eigen::Index dropped_k = active_ineq[static_cast<std::size_t>(drop_idx)];
        is_active[static_cast<std::size_t>(dropped_k)] = false;
        active_ineq.erase(active_ineq.begin() + drop_idx);

        Eigen::VectorXd u_new(q_ineq - 1);
        for (Eigen::Index j = 0, k = 0; j < q_ineq; ++j) {
          if (j != drop_idx) {
            u_new(k++) = u_ineq(j);
          }
        }
        u_ineq = u_new;
      } else {
        // Fallback: drop least negative multiplier if any
        Eigen::Index drop_pos = -1;
        double min_u = 0.0;
        for (Eigen::Index j = 0; j < q_ineq; ++j) {
          if (u_ineq(j) < min_u) {
            min_u = u_ineq(j);
            drop_pos = j;
          }
        }
        if (drop_pos < 0) {
          return inf;
        }
        const Eigen::Index dropped_k = active_ineq[static_cast<std::size_t>(drop_pos)];
        is_active[static_cast<std::size_t>(dropped_k)] = false;
        active_ineq.erase(active_ineq.begin() + drop_pos);

        Eigen::VectorXd u_new(q_ineq - 1);
        for (Eigen::Index i = 0, j = 0; i < q_ineq; ++i) {
          if (i != drop_pos) {
            u_new(j++) = u_ineq(i);
          }
        }
        u_ineq = u_new;
      }
    }
  }

  return 0.5 * x.dot(G * x) + g0.dot(x);
}

/**
 * @brief Convenience wrapper around solve_quadprog for problems with linear
 *        inequalities A_ineq * x >= b_ineq and box bounds lb <= x <= ub:
 *
 *   min_{x in R^n}   0.5 * x^T * G * x + g0^T * x
 *   subject to       A_ineq * x >= b_ineq
 *                    lb <= x <= ub
 */
inline double solve_quadprog_box(const Eigen::MatrixXd& G,
                                 const Eigen::VectorXd& g0,
                                 const Eigen::MatrixXd& A_ineq,
                                 const Eigen::VectorXd& b_ineq,
                                 const Eigen::VectorXd& lb,
                                 const Eigen::VectorXd& ub,
                                 Eigen::VectorXd& x) {
  const Eigen::Index n = G.rows();
  const Eigen::Index m_lin = A_ineq.rows();

  // Count finite lower and upper box bounds
  Eigen::Index m_lb = 0;
  Eigen::Index m_ub = 0;
  for (Eigen::Index i = 0; i < lb.size(); ++i) {
    if (std::isfinite(lb(i))) ++m_lb;
  }
  for (Eigen::Index i = 0; i < ub.size(); ++i) {
    if (std::isfinite(ub(i))) ++m_ub;
  }

  const Eigen::Index mi = m_lin + m_lb + m_ub;
  Eigen::MatrixXd CI(n, mi);
  Eigen::VectorXd ci0(mi);

  Eigen::Index col = 0;
  // 1) A_ineq * x - b_ineq >= 0  <=>  (A_ineq^T)^T * x + (-b_ineq) >= 0
  for (Eigen::Index r = 0; r < m_lin; ++r) {
    CI.col(col) = A_ineq.row(r).transpose();
    ci0(col) = -b_ineq(r);
    ++col;
  }
  // 2) x_i - lb_i >= 0
  for (Eigen::Index i = 0; i < lb.size(); ++i) {
    if (std::isfinite(lb(i))) {
      CI.col(col).setZero();
      CI(i, col) = 1.0;
      ci0(col) = -lb(i);
      ++col;
    }
  }
  // 3) -x_i + ub_i >= 0
  for (Eigen::Index i = 0; i < ub.size(); ++i) {
    if (std::isfinite(ub(i))) {
      CI.col(col).setZero();
      CI(i, col) = -1.0;
      ci0(col) = ub(i);
      ++col;
    }
  }

  const Eigen::MatrixXd CE(n, 0);
  const Eigen::VectorXd ce0(0);
  return solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
}

} // namespace cbf
