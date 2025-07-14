#pragma once

#include <ekf_rio_tc/EkfRioCovariance.h>
#include <ekf_rio_tc/EkfRioState.h>
#include <ekf_rio_tc/ekf_rio_filter.h>
#include <geometry_msgs/Vector3.h>
#include <rio_utils/data_types.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rio {
namespace msg_conversion {
/**
 * @brief Creates an ekf_rio::EkfRioState message
 * @param ekf_rio_filter  filter
 * @param frame_id        frame_id of the global states
 * @returns an ekf_rio::EkfRioState
 */
static inline ekf_rio_tc::EkfRioState toStateMsg(
    const EkfRioFilter &ekf_rio_filter, const std::string &frame_id) {
    ekf_rio_tc::EkfRioState state_msg;

    state_msg.header.stamp = ekf_rio_filter.getTimestamp();
    state_msg.header.frame_id = frame_id;

    const Quaternion q_ItoG =
        ekf_rio_filter.getNavigationSolution().getAttitude();
    tf2::Quaternion q_tf2(q_ItoG.x(), q_ItoG.y(), q_ItoG.z(), q_ItoG.w());
    Real roll, pitch, yaw;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);

    state_msg.q_ItoG = tf2::toMsg(q_ItoG);
    tf2::toMsg(EulerAngles(roll, pitch, yaw).to_degrees(),
               state_msg.q_ItoG_eul_deg);

    tf2::toMsg(EulerAngles(ekf_rio_filter.getBias().gyro).to_degrees(),
               state_msg.b_g_deg);

    tf2::toMsg(ekf_rio_filter.getNavigationSolution().v_IinG, state_msg.v_IinG);

    tf2::toMsg(ekf_rio_filter.getBias().acc, state_msg.b_a);

    tf2::toMsg(ekf_rio_filter.getNavigationSolution().getPosition(),
               state_msg.p_IinG);

    state_msg.b_alt = ekf_rio_filter.getBias().alt;

    tf2::toMsg(ekf_rio_filter.getExtTrans(), state_msg.p_RinI);

    const Quaternion q_RtoI = Quaternion(ekf_rio_filter.getExtRot());
    tf2::Quaternion q_RtoI_tf2(q_RtoI.x(), q_RtoI.y(), q_RtoI.z(), q_RtoI.w());
    Real q_RtoI_roll, q_RtoI_pitch, q_RtoI_yaw;
    tf2::Matrix3x3(q_RtoI_tf2).getRPY(q_RtoI_roll, q_RtoI_pitch, q_RtoI_yaw);

    state_msg.q_RtoI = tf2::toMsg(q_RtoI);
    tf2::toMsg(EulerAngles(q_RtoI_roll, q_RtoI_pitch, q_RtoI_yaw).to_degrees(),
               state_msg.q_RtoI_eul_deg);

    state_msg.t_d = ekf_rio_filter.getBias().t_d;

    return state_msg;
}

/**
 * @brief Creates an ekf_rio::EkfRioCovariance message
 * @param ekf_rio_filter  filter
 * @param frame_id        frame_id of the global states
 * @returns an ekf_rio::EkfRioCovariance
 */
static inline ekf_rio_tc::EkfRioCovariance toCovMsg(
    const EkfRioFilter &ekf_rio_filter, const std::string &frame_id) {
    ekf_rio_tc::EkfRioCovariance cov_msg;

    cov_msg.header.stamp = ekf_rio_filter.getTimestamp();
    cov_msg.header.frame_id = frame_id;
    const Matrix C = ekf_rio_filter.getCovarianceMatrix();
    const auto idx = ekf_rio_filter.getErrorIdx();

    cov_msg.sigma_q_ItoG_eul_deg.x =
        angles::to_degrees(std::sqrt(C(idx.attitude, idx.attitude)));
    cov_msg.sigma_q_ItoG_eul_deg.y =
        angles::to_degrees(std::sqrt(C(idx.attitude + 1, idx.attitude + 1)));
    cov_msg.sigma_q_ItoG_eul_deg.z =
        angles::to_degrees(std::sqrt(C(idx.attitude + 2, idx.attitude + 2)));

    cov_msg.sigma_b_g_deg.x =
        angles::to_degrees(std::sqrt(C(idx.bias_gyro, idx.bias_gyro)));
    cov_msg.sigma_b_g_deg.y =
        angles::to_degrees(std::sqrt(C(idx.bias_gyro + 1, idx.bias_gyro + 1)));
    cov_msg.sigma_b_g_deg.z =
        angles::to_degrees(std::sqrt(C(idx.bias_gyro + 2, idx.bias_gyro + 2)));

    cov_msg.sigma_v_IinG.x = std::sqrt(C(idx.velocity, idx.velocity));
    cov_msg.sigma_v_IinG.y = std::sqrt(C(idx.velocity + 1, idx.velocity + 1));
    cov_msg.sigma_v_IinG.z = std::sqrt(C(idx.velocity + 2, idx.velocity + 2));

    cov_msg.sigma_b_a.x = std::sqrt(C(idx.bias_acc, idx.bias_acc));
    cov_msg.sigma_b_a.y = std::sqrt(C(idx.bias_acc + 1, idx.bias_acc + 1));
    cov_msg.sigma_b_a.z = std::sqrt(C(idx.bias_acc + 2, idx.bias_acc + 2));

    cov_msg.sigma_p_IinG.x = std::sqrt(C(idx.position, idx.position));
    cov_msg.sigma_p_IinG.y = std::sqrt(C(idx.position + 1, idx.position + 1));
    cov_msg.sigma_p_IinG.z = std::sqrt(C(idx.position + 2, idx.position + 2));

    cov_msg.sigma_b_alt = std::sqrt(C(idx.bias_alt, idx.bias_alt));

    cov_msg.sigma_p_RinI.x = std::sqrt(C(idx.ext_trans, idx.ext_trans));
    cov_msg.sigma_p_RinI.y = std::sqrt(C(idx.ext_trans + 1, idx.ext_trans + 1));
    cov_msg.sigma_p_RinI.z = std::sqrt(C(idx.ext_trans + 2, idx.ext_trans + 2));

    cov_msg.sigma_q_RtoI_eul_deg.x =
        angles::to_degrees(std::sqrt(C(idx.ext_rot, idx.ext_rot)));
    cov_msg.sigma_q_RtoI_eul_deg.y =
        angles::to_degrees(std::sqrt(C(idx.ext_rot + 1, idx.ext_rot + 1)));
    cov_msg.sigma_q_RtoI_eul_deg.z =
        angles::to_degrees(std::sqrt(C(idx.ext_rot + 2, idx.ext_rot + 2)));

    cov_msg.sigma_t_d = std::sqrt(C(idx.time_offset, idx.time_offset));

    return cov_msg;
}

} // namespace msg_conversion
} // namespace rio
