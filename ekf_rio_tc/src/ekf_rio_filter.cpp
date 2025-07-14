#include <ekf_rio_tc/ekf_rio_filter.h>
#include <rio_utils/math_helper.h>

#include <boost/math/distributions/chi_squared.hpp>

using namespace rio;

bool EkfRioFilter::init(const std::vector<ImuDataStamped> &imu_init_vec,
                        const Real &baro_h0) {
  x_error_ = Vector::Zero(error_idx_.base_state_length);

  nav_sol_.setPosition(init_struct_.p_IinG_0);
  nav_sol_.v_IinG = init_struct_.v_IinG_0;

  // init att from acc
  Vector3 a_accu(0, 0, 0), w_accu(0, 0, 0);
  for (const auto &imu_data : imu_init_vec) {
    a_accu += imu_data.a_meas;
    w_accu += imu_data.w_meas;
  }
  const Vector3 acc_mean = a_accu / imu_init_vec.size();
  const Vector3 w_mean = w_accu / imu_init_vec.size();

  EulerAngles roll_pitch =
      math_helper::initFromAcc(acc_mean, init_struct_.gravity);

  ROS_INFO_STREAM(kStreamingPrefix
                  << "Initialized attitude: " << roll_pitch.to_degrees().x()
                  << "deg, " << roll_pitch.to_degrees().y() << "deg");

  nav_sol_.setEuler_n_b(
      EulerAngles(roll_pitch.roll(), roll_pitch.pitch(), init_struct_.yaw_0));

  bias_.acc = init_struct_.b_a_0;
  bias_.gyro = init_struct_.omega_calibration ? w_mean + init_struct_.b_g_0
                                              : init_struct_.b_g_0;
  bias_.alt = baro_h0;

  T_RtoI_.translation() = init_struct_.p_RinI;
  T_RtoI_.linear() = Matrix3(init_struct_.q_RtoI);

  ROS_INFO_STREAM(kStreamingPrefix
                  << "Initialized w_bias: "
                  << EulerAngles(bias_.gyro).to_degrees().transpose());

  covariance_ = init_struct_.P_kk_0;

  strapdown_ = Strapdown(init_struct_.gravity);

  time_stamp_ = imu_init_vec.back().time_stamp;

  bias_.t_d = init_struct_.t_d_0;

  return true;
}

bool EkfRioFilter::propagate(const ImuDataStamped &imu) {
  time_stamp_ = imu.time_stamp;

  const Vector3 a_hat = imu.a_meas - bias_.acc;
  const Vector3 w_hat = imu.w_meas - bias_.gyro;
  nav_sol_ = strapdown_.propagate(nav_sol_, a_hat, w_hat, imu.dt);

  const Matrix Phi = getPhi(a_hat, w_hat, imu.dt);

  Matrix G = system_noise_.getG(nav_sol_.getR_ItoG(), error_idx_, noise_idx_);

  // when a_b_ib_corrected and w_b_ib_corrected are too small, set zero time
  // offset noise acc - R * gravity
  const Vector3 a_hat_corrected =
      a_hat +
      nav_sol_.getR_ItoG().transpose() * Vector3(0, 0, init_struct_.gravity);
  if (a_hat_corrected.norm() < 0.15 && w_hat.norm() < 0.015) {
    G(error_idx_.time_offset, noise_idx_.time_offset) = 0.0;
  }

  // TODO: consider more efficient implementation!!
  covariance_ = Phi * covariance_ * Phi.transpose() +
                G * system_noise_.getQ(imu.dt, noise_idx_) * G.transpose();

  return true;
}

Matrix EkfRioFilter::getPhi(Vector3 a_hat, Vector3 w_hat, const Real &T) const {
  Matrix F =
      Matrix::Zero(error_idx_.base_state_length, error_idx_.base_state_length);
  F.block<3, 3>(error_idx_.attitude, error_idx_.attitude) =
      -math_helper::skewVec(w_hat);
  F.block<3, 3>(error_idx_.attitude, error_idx_.bias_gyro) =
      -Matrix::Identity(3, 3);
  F.block<3, 3>(error_idx_.velocity, error_idx_.attitude) =
      -nav_sol_.getR_ItoG() * math_helper::skewVec(a_hat);
  F.block<3, 3>(error_idx_.velocity, error_idx_.bias_acc) =
      -nav_sol_.getR_ItoG();
  F.block<3, 3>(error_idx_.position, error_idx_.velocity) =
      Matrix::Identity(3, 3);

  Matrix Fdt = F * T;
  Matrix Fdt2 = Fdt * Fdt;
  Matrix Fdt3 = Fdt2 * Fdt;
  Matrix Phi = Matrix::Identity(error_idx_.base_state_length,
                                error_idx_.base_state_length) +
               Fdt + 0.5 * Fdt2 + (1.0 / 6.0) * Fdt3;
  return Phi;
}

bool EkfRioFilter::kfUpdate(const Vector &r, const Matrix &H,
                            const Vector &R_diag) {
  const Matrix R = R_diag.asDiagonal();
  const Matrix S = H * covariance_ * H.transpose() + R;

  Matrix S_inv = Matrix::Identity(S.rows(), S.cols());
  S.llt().solveInPlace(S_inv);

  const Matrix K = covariance_ * H.transpose() * S_inv;
  const Vector x_error = K * r;

  covariance_ = covariance_ - K * H * covariance_;
  correctNominalState(x_error);

  return true;  // TODO add check for invalid result
}

bool EkfRioFilter::correctNominalState(const Vector x_error) {
  nav_sol_.setAttitude(getCorrectedQuaternion(
      nav_sol_.getAttitude(), x_error.segment(error_idx_.attitude, 3)));
  bias_.gyro += x_error.segment(error_idx_.bias_gyro, 3);
  nav_sol_.v_IinG += x_error.segment(error_idx_.velocity, 3);
  bias_.acc += x_error.segment(error_idx_.bias_acc, 3);
  nav_sol_.setPosition(nav_sol_.getPosition() +
                       x_error.segment(error_idx_.position, 3));

  bias_.alt += x_error(error_idx_.bias_alt);
  bias_.t_d += x_error(error_idx_.time_offset);

  T_RtoI_.translation() =
      T_RtoI_.translation() + x_error.segment(error_idx_.ext_trans, 3);
  T_RtoI_.linear() =
      getCorrectedQuaternion(Quaternion(T_RtoI_.linear()),
                             x_error.segment(error_idx_.ext_rot, 3))
          .normalized()
          .toRotationMatrix();

  return true;
}

bool EkfRioFilter::updateAltimeter(const Real neg_rel_h, const Real &sigma) {
  Matrix H = Matrix::Zero(1, getCovarianceMatrix().cols());
  Vector r(1);
  Vector R_diag(1);

  H(0, error_idx_.position + 2) = 1;
  H(0, error_idx_.bias_alt) = 1;
  const Real h_filter = getNavigationSolution().getPosition().z();
  const Real h_meas = neg_rel_h - getBias().alt;
  r(0) = h_meas - h_filter;
  R_diag(0) = sigma * sigma;
  kfUpdate(r, H, R_diag);

  return true;
}

bool EkfRioFilter::updateRadarEgoVelocity(
    const Vector3 v_RinR, const Vector3 sigma_v_RinR, const Vector3 a_meas,
    const Vector3 w_meas, const Real outlier_rejection_thresh,
    const bool use_temp_calib, const bool use_ext_calib) {
  Matrix H = Matrix::Zero(3, error_idx_.base_state_length);
  Vector r(3);
  Vector R_diag(3);

  const Matrix R_ItoG = nav_sol_.getR_ItoG();
  const Matrix R_RtoI = T_RtoI_.linear();
  const Vector3 p_RinI = T_RtoI_.translation();

  const Vector3 v_rot = math_helper::skewVec(w_meas - bias_.gyro) * p_RinI;
  const Vector3 v_IinG = nav_sol_.v_IinG;
  const Vector3 v_IinI = R_ItoG.transpose() * v_IinG;
  const Vector3 v_RinR_filter = R_RtoI.transpose() * (v_IinI + v_rot);

  const Matrix3 H_q_ItoG =
      R_RtoI.transpose() * math_helper::skewVec(R_ItoG.transpose() * v_IinG);
  const Matrix3 H_b_g = R_RtoI.transpose() * math_helper::skewVec(p_RinI);
  const Matrix3 H_v_IinG = R_RtoI.transpose() * R_ItoG.transpose();
  const Vector3 H_t_d = H_q_ItoG * (w_meas - getBias().gyro) +
                        H_v_IinG * (R_ItoG * (a_meas - getBias().acc) +
                                    Vector3(0, 0, init_struct_.gravity));

  const Matrix3 H_p_RinI =
      R_RtoI.transpose() * math_helper::skewVec(w_meas - getBias().gyro);
  const Matrix3 H_q_RtoI =
      math_helper::skewVec(R_RtoI.transpose() * v_RinR_filter);

  H.block<3, 3>(0, error_idx_.attitude) = H_q_ItoG;
  H.block<3, 3>(0, error_idx_.bias_gyro) = H_b_g;
  H.block<3, 3>(0, error_idx_.velocity) = H_v_IinG;
  if (use_temp_calib) H.block<3, 1>(0, error_idx_.time_offset) = H_t_d;
  if (use_ext_calib) {
    H.block<3, 3>(0, error_idx_.ext_trans) = H_p_RinI;
    H.block<3, 3>(0, error_idx_.ext_rot) = H_q_RtoI;
  }

  r = v_RinR - v_RinR_filter;
  R_diag = (sigma_v_RinR).array().square();

  // outlier rejection
  if (outlier_rejection_thresh > 0.001) {
    const Real gamma = r.transpose() *
                       (H * getCovarianceMatrix() * H.transpose() +
                        Matrix(R_diag.asDiagonal()))
                           .inverse() *
                       r;
    boost::math::chi_squared chiSquaredDist(3.0);
    const double gamma_thresh =
        boost::math::quantile(chiSquaredDist, 1 - outlier_rejection_thresh);

    if (gamma < gamma_thresh) {
      kfUpdate(r, H, R_diag);
    } else {
      ROS_INFO_STREAM(kStreamingPrefix << "Outlier radar");
      return false;
    }
  } else
    kfUpdate(r, H, R_diag);

  return true;
}

Quaternion EkfRioFilter::getCorrectedQuaternion(
    const Quaternion &q, const Vector3 &err_euler) const {
  return math_helper::quaternionMultiplicationHamilton(
      q, Quaternion(1, 0.5 * err_euler.x(), 0.5 * err_euler.y(),
                    0.5 * err_euler.z()));
}