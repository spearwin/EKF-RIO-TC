#pragma once

#include <ekf_rio_tc/EkfRioConfig.h>
#include <ekf_rio_tc/data_types.h>
#include <rio_utils/data_types.h>
#include <rio_utils/strapdown.h>
#include <sensor_msgs/Imu.h>

namespace rio {
/**
 * @brief The EkfRioFilter class provides the EKF based Radar Inertial Odometry
 * Filter \note Uses the North East Down (NED) convention --> the z-axis point
 * downwards
 */
class EkfRioFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * @brief Reconfigure callback, enables online reconfigure using
   * rqt_reconfigure
   * @param config   must contain the members of EkfRioConfig
   */
  template <class ConfigContainingEkfRioConfig>
  bool configure(ConfigContainingEkfRioConfig &config);

  /**
   * @brief Initializes the navigation filter
   * @param imu_init_vec   vector of imu measurements used for initialization
   * @param baro_h0        initial height from the barometer
   * @returns true if init successful
   */
  bool init(const std::vector<ImuDataStamped> &imu_init_vec,
            const Real &baro_h0);

  /**
   * @brief Propagates the filter state using the provided IMU measurement
   * @param imu   IMU measurement
   * @returns true if successful
   */
  bool propagate(const ImuDataStamped &imu);

  /**
   * @brief Implements the altimeter kalman filter update model
   * @param neg_rel_h   relative negative height [m]
   * @param sigma       sigma of this measurement
   * @returns true if successful
   */
  bool updateAltimeter(const Real neg_rel_h, const Real &sigma);

  /**
   * @brief Implements the radar ego velocity Kalman filter update model
   * @param v_RinR                     3D radar ego velocity
   * @param sigma_v_RinR               sigma of 3D velocity
   * @param a_meas                     linear acceleration measurement
   * @param w_meas                     angular velocity measurement
   * @param outlier_rejection_thresh   threshold for Mahalanobis distance check
   * @param use_temp_calib             use temporal calibration for the radar
   * @param use_ext_calib              use extrinsic calibration for the radar
   * @returns true if successful
   */
  bool updateRadarEgoVelocity(const Vector3 v_RinR, const Vector3 sigma_v_RinR,
                              const Vector3 a_meas, const Vector3 w_meas,
                              const Real outlier_rejection_thresh,
                              const bool use_temp_calib,
                              const bool use_ext_calib);

  /**
   * @brief Returns the current filter ros time stamp
   */
  ros::Time getTimestamp() const { return time_stamp_; }

  /**
   * @brief Returns the covariance matrix
   */
  Matrix getCovarianceMatrix() const { return covariance_; }

  /**
   * @brief Returns the error state idx struct
   */
  EkfRioFilterStateIdx getErrorIdx() const { return error_idx_; }

  /**
   * @brief Returns the current navigation solution
   */
  NavigationSolution getNavigationSolution() const { return nav_sol_; }

  /**
   * @brief Returns the estimated biases (acc, gyro, altimeter)
   */
  Offsets getBias() const { return bias_; }

  /**
   * @brief Returns the estimated translation of the extrinsic radar transform
   */
  Vector3 getExtTrans() const { return T_RtoI_.translation(); }

  /**
   * @brief Returns the estimated rotational part of the extrinsic radar
   * transform
   */
  Matrix3 getExtRot() const { return T_RtoI_.linear(); }

  /**
   * @brief Returns the extrinsic radar transform as Isometry
   */
  Isometry getExtT() const { return T_RtoI_; }

 protected:
  /**
   * @brief Returns the state transtition matrix Phi used for the propagation of
   * the covariance
   * @param a_hat    bias-corrected accelerometer measurement
   * @param w_hat    bias-corrected angular velocity
   * @param T         discretization time
   * @returns Phi
   */
  Matrix getPhi(Vector3 a_hat, Vector3 w_hat, const Real &T) const;

  /**
   * @brief Corrects the nominal filter state based on the given error state
   * @returns true if successful
   */
  bool correctNominalState(const Vector x_error);

  /**
   * @brief Implements an Extended Kalman Filter update
   * @param r       observed residuum
   * @param H       measurement jacobian
   * @param R_diag  diagonal elements of the measurement noise matrix
   * @returns true if successful
   */
  bool kfUpdate(const Vector &r, const Matrix &H, const Vector &R_diag);

  /**
   * @brief Returns the corrected quaternion based on the given euler error and
   * quaternion (Hamilton convention!)
   * @param q           Quaternion to be corrected
   * @param err_euler   Euler error angles
   * @returns the corrected quaternion
   */
  Quaternion getCorrectedQuaternion(const Quaternion &q,
                                    const Vector3 &err_euler) const;

  std::string kStreamingPrefix = "[EkfRioFilter]: ";

  Strapdown strapdown_;

  InitStruct init_struct_;
  SystemNoisePsd system_noise_;

  ros::Time time_stamp_;
  NavigationSolution nav_sol_;
  Offsets bias_;
  Isometry T_RtoI_;

  Vector x_error_;
  EkfRioFilterStateIdx error_idx_;
  EkfRioFilterNoiseIdx noise_idx_;

  Matrix covariance_;
};

template <typename ConfigContainingEkfRioConfig>
bool EkfRioFilter::configure(ConfigContainingEkfRioConfig &config) {
  bool success = true;

  success |= init_struct_.configure(error_idx_, config);
  success |= system_noise_.configure(config);

  return success;
}

}  // namespace rio
