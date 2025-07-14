#pragma once

#include <eigen3/Eigen/Dense>

#include "rio_utils/data_types.h"
#include "rio_utils/math_helper.h"

namespace rio {
/**
 * @brief The Strapdown class implements a strapdown which propagates a
 * kinematic state using an IMU measurement
 * @note Uses the North East Down (NED) convention
 */
class Strapdown {
 public:
  /**
   * @brief Strapdown constructor
   * @param local_gravity  local gravity using the NED convention
   */
  Strapdown(const double local_gravity = 9.80665);

  /**
   * @brief Propagates the given navigation solution using an IMU measurement
   * @param nav_sol_prev   previous navigation solution
   * @param a_hat          measured body-frame acceleration (bias removed)
   * @param w_hat          measured body frame angular velocity (bias removed)
   * @param dt             propagation time
   * @return
   */
  NavigationSolution propagate(const NavigationSolution nav_sol_prev,
                               const Vector3 a_hat, const Vector3 w_hat,
                               const Real dt);

 private:
  Eigen::Matrix4d getQLeftMatrix(const Vector4& v);

  Vector3 local_gravity_;
};
}  // namespace rio
