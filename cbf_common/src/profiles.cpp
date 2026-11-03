#include "cbf_common/profiles.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cbf
{

double evaluateProfileH(
    ProfileType profile,
    double u,
    const std::function<double(double)> &custom_h)
{
  const double u_clamped = std::clamp(u, 0.0, 1.0);
  switch (profile)
  {
  case ProfileType::CONSTANT:
    return 1.0;
  case ProfileType::ROOT_FIFTH:
    return std::pow(u_clamped, 0.2);
  case ProfileType::LINEAR:
    return u_clamped;
  case ProfileType::QUINTIC:
    return std::pow(u_clamped, 5.0);
  case ProfileType::SMOOTHSTEP:
    return -2.0 * u_clamped * u_clamped * u_clamped +
           3.0 * u_clamped * u_clamped;
  case ProfileType::CUSTOM:
    if (!custom_h)
    {
      throw std::runtime_error("Custom profile h(u) function is not set.");
    }
    return custom_h(u_clamped);
  }
  return u_clamped;
}

double evaluateAntiderivativeH(
    ProfileType profile,
    double u,
    const std::function<double(double)> &custom_H)
{
  const double u_clamped = std::clamp(u, 0.0, 1.0);
  switch (profile)
  {
  case ProfileType::CONSTANT:
    return u_clamped;
  case ProfileType::ROOT_FIFTH:
    return (5.0 / 6.0) * std::pow(u_clamped, 1.2);
  case ProfileType::LINEAR:
    return 0.5 * u_clamped * u_clamped;
  case ProfileType::QUINTIC:
    return (1.0 / 6.0) * std::pow(u_clamped, 6.0);
  case ProfileType::SMOOTHSTEP:
    return -0.5 * std::pow(u_clamped, 4.0) + std::pow(u_clamped, 3.0);
  case ProfileType::CUSTOM:
    if (!custom_H)
    {
      throw std::runtime_error("Custom profile antiderivative H(u) function is not set.");
    }
    return custom_H(u_clamped);
  }
  return 0.5 * u_clamped * u_clamped;
}

double computeVelocityEnvelope(
    double zeta,
    double p_ths,
    double v_0,
    double v_ths,
    ProfileType profile,
    const std::function<double(double)> &custom_H)
{
  if (p_ths <= 0.0)
  {
    throw std::invalid_argument("p_ths must be strictly positive.");
  }
  const double u = std::clamp((p_ths - zeta) / p_ths, 0.0, 1.0);
  const double H_u = evaluateAntiderivativeH(profile, u, custom_H);
  const double H_1 = evaluateAntiderivativeH(profile, 1.0, custom_H);
  return v_ths - (v_ths - v_0) * (1.0 - (H_u / H_1));
}

double computeVelocityEnvelopeDerivative(
    double zeta,
    double p_ths,
    double v_0,
    double v_ths,
    ProfileType profile,
    const std::function<double(double)> &custom_h,
    const std::function<double(double)> &custom_H)
{
  if (p_ths <= 0.0)
  {
    throw std::invalid_argument("p_ths must be strictly positive.");
  }
  if (zeta < 0.0 || zeta > p_ths)
  {
    return 0.0;
  }
  const double u = std::clamp((p_ths - zeta) / p_ths, 0.0, 1.0);
  const double h_u = evaluateProfileH(profile, u, custom_h);
  const double H_1 = evaluateAntiderivativeH(profile, 1.0, custom_H);
  return -((v_ths - v_0) / (p_ths * H_1)) * h_u;
}

double computeBarrierValue(
    double zeta,
    double dzeta,
    double p_ths,
    double v_0,
    double v_ths,
    ProfileType profile,
    const std::function<double(double)> &custom_H)
{
  return dzeta - computeVelocityEnvelope(zeta, p_ths, v_0, v_ths, profile, custom_H);
}

} // namespace cbf
