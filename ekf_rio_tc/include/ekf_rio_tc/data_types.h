#pragma once

#include <rio_utils/data_types.h>

namespace rio {
struct EkfRioFilterStateIdx {
  const uint position = 0;
  const uint velocity = 3;
  const uint attitude = 6;
  const uint bias_acc = 9;
  const uint bias_gyro = 12;
  const uint bias_alt = 15;
  const uint time_offset = 16;
  const uint ext_trans = 17;
  const uint ext_rot = 20;

  const uint base_state_length = 23;
};

struct EkfRioFilterNoiseIdx {
  const uint gyro = 0;
  const uint bias_gyro = 3;
  const uint acc = 6;
  const uint bias_acc = 9;
  const uint bias_alt = 12;
  const uint time_offset = 13;

  const uint base_noise_length = 14;
};

struct Offsets {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Vector3 acc = Vector3(-1, -1, -1);
  Vector3 gyro = Vector3(-1, -1, -1);
  Real alt = -1;
  Real t_d = -1;
};

struct SystemNoisePsd {
  template <class ConfigContainingEkfRioConfig>
  bool configure(ConfigContainingEkfRioConfig &config) {
    acc = config.noise_psd_a;
    gyro = angles::from_degrees(config.noise_psd_g_deg);
    bias_acc = config.noise_psd_b_a;
    bias_gyro = angles::from_degrees(config.noise_psd_b_g_deg);
    bias_alt = config.noise_psd_b_alt;
    if (config.use_temp_calib)
      t_d = config.noise_psd_t_d;
    else
      t_d = 0.0;

    return true;
  }

  Matrix getQ(Real T, const EkfRioFilterNoiseIdx &noise) const {
    Matrix Q = Matrix::Zero(noise.base_noise_length, noise.base_noise_length);
    Q.block<3, 3>(noise.acc, noise.acc).diagonal() =
        Vector::Ones(3) * std::pow(acc, 2) * T;
    Q.block<3, 3>(noise.gyro, noise.gyro).diagonal() =
        Vector::Ones(3) * std::pow(gyro, 2) * T;
    Q.block<3, 3>(noise.bias_acc, noise.bias_acc).diagonal() =
        Vector::Ones(3) * std::pow(bias_acc, 2) / T;
    Q.block<3, 3>(noise.bias_gyro, noise.bias_gyro).diagonal() =
        Vector::Ones(3) * std::pow(bias_gyro, 2) / T;
    Q(noise.bias_alt, noise.bias_alt) = std::pow(bias_alt, 2) / T;
    Q(noise.time_offset, noise.time_offset) = std::pow(t_d, 2) / T;

    return Q;
  }

  Matrix getG(const Matrix3 &R_ItoG, const EkfRioFilterStateIdx &error,
              const EkfRioFilterNoiseIdx &noise) const {
    Matrix G = Matrix::Zero(error.base_state_length, noise.base_noise_length);

    G.block<3, 3>(error.attitude, noise.gyro) = -Matrix3::Identity();
    G.block<3, 3>(error.bias_gyro, noise.bias_gyro) = Matrix3::Identity();
    G.block<3, 3>(error.velocity, noise.acc) = -R_ItoG;
    G.block<3, 3>(error.bias_acc, noise.bias_acc) = Matrix3::Identity();
    G(error.bias_alt, noise.bias_alt) = 1;
    G(error.time_offset, noise.time_offset) = 1;

    return G;
  }

  Real acc = -1;
  Real gyro = -1;
  Real bias_acc = -1;
  Real bias_gyro = -1;
  Real bias_alt = -1;
  Real t_d = -1;
};

struct InitStruct {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  template <class Config>
  bool configure(const EkfRioFilterStateIdx &error, Config &config) {
    p_IinG_0 = Vector3(config.p_IinG_x_0, config.p_IinG_y_0, config.p_IinG_z_0);
    v_IinG_0 = Vector3(config.v_IinG_x_0, config.v_IinG_y_0, config.v_IinG_z_0);
    yaw_0 = angles::from_degrees(config.q_ItoG_yaw_0_deg);
    b_a_0 = Vector3(config.b_a_x_0, config.b_a_y_0, config.b_a_z_0);
    b_g_0 = Vector3(angles::from_degrees(config.b_g_x_0_deg),
                    angles::from_degrees(config.b_g_y_0_deg),
                    angles::from_degrees(config.b_g_z_0_deg));
    b_alt_0 = config.b_alt_0;
    t_d_0 = config.t_d_0;
    p_RinI = Vector3(config.p_RinI_x, config.p_RinI_y, config.p_RinI_z);
    q_RtoI = Quaternion(config.q_RtoI_w, config.q_RtoI_x, config.q_RtoI_y,
                        config.q_RtoI_z);

    P_kk_0 = Matrix::Zero(error.base_state_length, error.base_state_length);
    P_kk_0.block(error.position, error.position, 3, 3).diagonal() =
        Vector3(1, 1, 1) * std::pow(config.sigma_p_IinG, 2);

    P_kk_0.block(error.velocity, error.velocity, 3, 3).diagonal() =
        Vector3(1, 1, 1) * std::pow(config.sigma_v_IinG, 2);

    P_kk_0.block(error.attitude, error.attitude, 3, 3).diagonal() =
        Vector3(angles::from_degrees(config.sigma_q_ItoG_roll_pitch_0_deg),
                angles::from_degrees(config.sigma_q_ItoG_roll_pitch_0_deg),
                angles::from_degrees(config.sigma_q_ItoG_yaw_0_deg))
            .array()
            .pow(2);

    P_kk_0.block(error.bias_acc, error.bias_acc, 3, 3).diagonal() =
        Vector3(1, 1, 1) * std::pow(config.sigma_b_a, 2);

    P_kk_0.block(error.bias_gyro, error.bias_gyro, 3, 3).diagonal() =
        Vector3(1, 1, 1) *
        std::pow(angles::from_degrees(config.sigma_b_g_deg), 2);

    P_kk_0(error.bias_alt, error.bias_alt) = std::pow(config.sigma_b_alt, 2);

    if (config.use_ext_calib) {
      P_kk_0.block(error.ext_trans, error.ext_trans, 3, 3).diagonal() =
          Vector3(config.sigma_p_RinI_x, config.sigma_p_RinI_y,
                  config.sigma_p_RinI_z)
              .array()
              .pow(2);
      P_kk_0.block(error.ext_rot, error.ext_rot, 3, 3).diagonal() =
          Vector3(angles::from_degrees(config.sigma_q_RtoI_roll_deg),
                  angles::from_degrees(config.sigma_q_RtoI_pitch_deg),
                  angles::from_degrees(config.sigma_q_RtoI_yaw_deg))
              .array()
              .pow(2);
    }

    if (config.use_temp_calib) {
      P_kk_0.block(error.time_offset, error.time_offset, 1, 1) =
          Vector::Ones(1) * std::pow(config.sigma_t_d, 2);
    }

    gravity = config.g_n;
    omega_calibration = config.calib_gyro;

    return true;
  }

  Vector3 p_IinG_0 = Vector3(0, 0, 0);
  Vector3 v_IinG_0 = Vector3(0, 0, 0);
  Real yaw_0 = 0.0;
  Vector3 b_a_0 = Vector3(0, 0, 0);
  Vector3 b_g_0 = Vector3(0, 0, 0);
  Real b_alt_0 = 0.0;
  Real t_d_0 = 0.0;
  Vector3 p_RinI = Vector3(0, 0, 0);
  Quaternion q_RtoI = Quaternion(1, 0, 0, 0);

  Matrix P_kk_0;

  Real gravity = 0;
  bool omega_calibration = true;
};

}  // namespace rio
