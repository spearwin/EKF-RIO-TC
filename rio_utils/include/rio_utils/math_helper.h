#pragma once

#include <rio_utils/data_types.h>

#include <Eigen/Dense>

namespace rio {
namespace math_helper {
/**
 * @brief Calculates the skew matrix from a 3D vector
 */
static Matrix3 skewVec(const Vector3& v) {
  Matrix3 S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

/**
 * @brief Performs a quaternion multiplication following the Hamilton convention
 */
static Quaternion quaternionMultiplicationHamilton(const Quaternion& q,
                                                   const Quaternion& p) {
  const Vector3 q_v(q.x(), q.y(), q.z());
  const Vector3 p_v(p.x(), p.y(), p.z());

  const Real q_p_w = p.w() * q.w() - q_v.transpose() * p_v;
  const Vector3 q_p_ = q.w() * p_v + p.w() * q_v + skewVec(q_v) * p_v;
  return Quaternion(q_p_w, q_p_.x(), q_p_.y(), q_p_.z());
}

/**
 * @brief Wraps a scalar (e.g. an angle) to the intervall [0, max]
 */
static Real wrapToPositive(const Real d, const Real max = 360) {
  const auto m = std::fmod(d, max);
  if (m < 0)
    return m + max;
  else
    return m;
}

/**
 * @brief Wraps a scalar (e.g. an angle) to the intervall [-abs_max, abs_max]
 */
static Real wrapToCentered(const Real d, const Real abs_max = 180) {
  return wrapToPositive(d + abs_max, abs_max * 2) - abs_max;
}

/**
 * @brief Converts from Cartesian to Spherical coordinates
 * @param[in] v      Cartesian vector
 * @param [out] r    range
 * @param [out] azi  azimuth
 * @param [out] ele  elevation
 */
static void cartesianToSpherical(const Vector3& v, Real& r, Real& azi,
                                 Real& ele) {
  r = v.norm();
  azi = std::atan2(v.y(), v.x());
  ele = std::atan2(std::sqrt(v.x() * v.x() + v.y() * v.y()), v.z());
}

/**
 * @brief The ConvolveType enum
 */
enum class ConvolveType {
  VALID,  // consider only where both vectors overlap completely
  FULL    // consider each point of overlap
};

/**
 * @brief Convolution of v with k
 * @param[in] v     vector to be convolved
 * @param[in] k     convolution kernel
 * @param[in] type  convolution type --> see ConvolveType
 * @param[out] c    convolution result
 * @returns true, if result valid
 */
static bool convolve(const Vector v, const Vector k, const ConvolveType type,
                     Vector& c) {
  if (k.size() < v.size()) {
    if (type == ConvolveType::FULL) {
      c = Vector::Zero(v.size() + k.size() - 1, 1);
      for (uint i = 0; i < c.size(); ++i) {
        if (i < k.size() - 1)
          c[i] = v.head(i + 1).transpose() * k.head(i + 1).reverse();
        else if (i > v.size() - 1)
          c[i] =
              v.tail(c.size() - i).transpose() * k.tail(c.size() - i).reverse();
        else
          c[i] =
              v.segment(i - k.size() + 1, k.size()).transpose() * k.reverse();
      }
      return true;
    } else {
      c = Vector::Zero(v.size() - k.size() + 1, 1);
      for (uint i = 0; i < c.size(); ++i)
        c[i] = v.segment(i, k.size()).transpose() * k.reverse();

      return true;
    }
  }
  return false;
}

/**
 * @brief Returns the corrected quaternion based on the given euler error and
 * quaternion (Hamilton convention!)
 * @param err_euler   Euler error angle
 * @param q           Quaterion to be corrected
 * @returns the corrected quaternion
 */
static Quaternion getCorrectedQuaternion(const Vector3& err_euler,
                                         const Quaternion& q) {
  return math_helper::quaternionMultiplicationHamilton(
      Quaternion(1, -0.5 * err_euler.x(), -0.5 * err_euler.y(),
                 -0.5 * err_euler.z()),
      q);
}

/**
 * @brief Initializes the attitude in roll and pitch from acceleration
 * @param acc_mean      mean acceleration (assuming being static)
 * @param gravity       local gravity
 * @returns the initialized attitude in EulerAngels (yaw = 0)
 */
static EulerAngles initFromAcc(const Vector3& acc_mean, const Real& gravity) {
  EulerAngles attitude(0, 0, 0);
  attitude.roll() = -1 * std::atan2(acc_mean.y(), -acc_mean.z());
  attitude.pitch() =
      std::asin(std::max(-1.0 + std::numeric_limits<Real>::epsilon(),
                         std::min(1.0 - std::numeric_limits<Real>::epsilon(),
                                  acc_mean.x() / gravity)));
  return attitude;
}
}  // namespace math_helper
}  // namespace rio
