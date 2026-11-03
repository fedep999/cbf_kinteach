#pragma once

#include <Eigen/Dense>

namespace cbf
{

/**
 * @brief Geometry of a 3D static hyperplane (contact surface).
 */
struct Hyperplane
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(); ///< Unit normal vector n_hp in R^3
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();  ///< Surface point p_hp in R^3
};

} // namespace cbf
