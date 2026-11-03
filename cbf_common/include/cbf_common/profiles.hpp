#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

namespace cbf
{

/**
 * @brief Predefined normalized force/damping profile functions h(u) on u in [0, 1].
 *
 * Here u = (p_ths - zeta) / p_ths in [0, 1] denotes the normalized penetration
 * depth into the braking zone, with u = 0 at entry (zeta = p_ths) and u = 1
 * at contact (zeta = 0).
 */
enum class ProfileType
{
  CONSTANT,   ///< h_1(u) = 1,            H_1(u) = u,                  H_1(1) = 1
  ROOT_FIFTH, ///< h_2(u) = u^(1/5),      H_2(u) = (5/6)*u^(6/5),      H_2(1) = 5/6
  LINEAR,     ///< h_3(u) = u,            H_3(u) = (1/2)*u^2,          H_3(1) = 1/2
  QUINTIC,    ///< h_4(u) = u^5,          H_4(u) = (1/6)*u^6,          H_4(1) = 1/6
  SMOOTHSTEP, ///< h_5(u) = -2*u^3 + 3u^2, H_5(u) = -0.5*u^4 + u^3,    H_5(1) = 1/2
  CUSTOM      ///< User-supplied h(u) and H(u) functions
};

/**
 * @brief Evaluates normalized profile h(u) for u in [0, 1].
 */
double evaluateProfileH(
    ProfileType profile,
    double u,
    const std::function<double(double)> &custom_h = nullptr);

/**
 * @brief Evaluates profile antiderivative H(u) = \int_0^u h(s) ds for u in [0, 1].
 */
double evaluateAntiderivativeH(
    ProfileType profile,
    double u,
    const std::function<double(double)> &custom_H = nullptr);

/**
 * @brief Computes safe velocity envelope v_min(zeta).
 *
 * v_min(zeta) = v_ths - (v_ths - v_0) * (1 - H(u)/H(1))
 * where u = clamp((p_ths - zeta)/p_ths, 0, 1).
 */
double computeVelocityEnvelope(
    double zeta,
    double p_ths,
    double v_0,
    double v_ths,
    ProfileType profile,
    const std::function<double(double)> &custom_H = nullptr);

/**
 * @brief Computes spatial derivative v_min'(zeta) = d(v_min)/dzeta.
 *
 * v_min'(zeta) = -((v_ths - v_0) / (p_ths * H(1))) * h(u) for zeta in [0, p_ths],
 * and 0 outside.
 */
double computeVelocityEnvelopeDerivative(
    double zeta,
    double p_ths,
    double v_0,
    double v_ths,
    ProfileType profile,
    const std::function<double(double)> &custom_h = nullptr,
    const std::function<double(double)> &custom_H = nullptr);

/**
 * @brief Computes Control Barrier Function value B(zeta, dzeta) = dzeta - v_min(zeta).
 */
double computeBarrierValue(
    double zeta,
    double dzeta,
    double p_ths,
    double v_0,
    double v_ths,
    ProfileType profile,
    const std::function<double(double)> &custom_H = nullptr);

} // namespace cbf
