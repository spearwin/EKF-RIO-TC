#pragma once

#include <angles/angles.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>

namespace rio {
typedef double Real;

typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;
typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef Eigen::Matrix<double, 11, 1> Vector11;
typedef Eigen::VectorXd Vector;
typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Isometry3d Isometry;
typedef Eigen::AngleAxisd AngleAxis;

struct EulerAngles : public Vector3 {
  EulerAngles() : Vector3(0, 0, 0) {}

  EulerAngles(const Real roll, const Real pitch, const Real yaw)
      : Vector3(roll, pitch, yaw) {}

  EulerAngles(const Vector3 &eul_n_b) : Vector3(eul_n_b) {}

  EulerAngles from_degrees(const Vector3 &eul_rad) {
    x() = angles::from_degrees(eul_rad.x());
    y() = angles::from_degrees(eul_rad.y());
    z() = angles::from_degrees(eul_rad.z());
    return EulerAngles(x(), y(), z());
  }

  Vector3 to_degrees() {
    return Vector3(angles::to_degrees(x()), angles::to_degrees(y()),
                   angles::to_degrees(z()));
  }

  Real &roll() { return Vector3::x(); }
  Real roll() const { return Vector3::x(); }

  Real &pitch() { return Vector3::y(); }
  Real pitch() const { return Vector3::y(); }

  Real &yaw() { return Vector3::z(); }
  Real yaw() const { return Vector3::z(); }
};

struct NavigationSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NavigationSolution() {
    T_ItoG.linear().setIdentity();
    T_ItoG.translation().setZero();
    v_IinG.setZero();
  }
  NavigationSolution(const Isometry &T_ItoG, const Vector3 &v_IinG)
      : T_ItoG{T_ItoG}, v_IinG{v_IinG} {}
  NavigationSolution(const Vector3 &p_IinG, const Quaternion &q_ItoG,
                     const Vector3 v_IinG)
      : v_IinG{v_IinG} {
    setAttitude(q_ItoG);
    setPosition(p_IinG);
  }

  Vector3 getPosition() const { return T_ItoG.translation(); }

  void setPosition(const Vector3 &p_IinG) { T_ItoG.translation() = p_IinG; }

  Quaternion getAttitude() const { return Quaternion(T_ItoG.linear()); }

  void setAttitude(const Quaternion &q_ItoG) {
    T_ItoG.linear() = q_ItoG.normalized().toRotationMatrix();
  }

  EulerAngles getEuler_n_b() const {
    const Quaternion q = getAttitude();
    tf2::Quaternion q_msg(q.x(), q.y(), q.z(), q.w());
    tf2::Matrix3x3 R(q_msg);
    EulerAngles eul;
    R.getEulerYPR(eul.yaw(), eul.pitch(), eul.roll());

    return eul;
  }

  void setEuler_n_b(const EulerAngles &eul_n_b) {
    tf2::Quaternion q_ItoG;
    q_ItoG.setRPY(eul_n_b.roll(), eul_n_b.pitch(), eul_n_b.yaw());
    T_ItoG.linear() =
        Matrix3(Quaternion(q_ItoG.w(), q_ItoG.x(), q_ItoG.y(), q_ItoG.z()));
  }

  Matrix3 getR_ItoG() const { return T_ItoG.linear(); }

  Isometry getPose() const { return T_ItoG; }

  Isometry getPoseRos() const {
    tf2::Quaternion q_n_b;
    q_n_b.setRPY(M_PI, 0, 0);
    return Matrix3(Quaternion(q_n_b.w(), q_n_b.x(), q_n_b.y(), q_n_b.z())) *
           T_ItoG;
  }

  Vector3 getVelocityRos() const {
    return Vector3(v_IinG.x(), -v_IinG.y(), -v_IinG.z());
  }

  Isometry T_ItoG;  // position and attitude in navigation frame
  Vector3 v_IinG;   // [m/s]
};

struct ImuData {
  ImuData() : dt{0} {}
  ImuData(const double dt, const Vector3 &a_meas, const Vector3 &w_meas)
      : dt{dt}, a_meas{a_meas}, w_meas{w_meas} {}

  Real dt;         // [s]
  Vector3 a_meas;  // [m/s^2]
  Vector3 w_meas;  // [rad/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuDataStamped {
  ImuDataStamped() : dt{0} {}
  ImuDataStamped(const ros::Time &time_stamp, const std::string frame_id,
                 const double dt, const Vector3 &a_meas, const Vector3 &w_meas)
      : time_stamp{time_stamp},
        frame_id{frame_id},
        dt{dt},
        a_meas{a_meas},
        w_meas{w_meas} {}

  ImuDataStamped(const sensor_msgs::ImuConstPtr &imu_msg, const Real dt)
      : time_stamp{imu_msg->header.stamp},
        frame_id{imu_msg->header.frame_id},
        dt{dt},
        a_meas{Vector3(imu_msg->linear_acceleration.x,
                       imu_msg->linear_acceleration.y,
                       imu_msg->linear_acceleration.z)},
        w_meas{Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                       imu_msg->angular_velocity.z)} {}

  sensor_msgs::Imu toImuMsg() {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = time_stamp;
    imu_msg.angular_velocity.x = w_meas.x();
    imu_msg.angular_velocity.y = w_meas.y();
    imu_msg.angular_velocity.z = w_meas.z();
    imu_msg.linear_acceleration.x = a_meas.x();
    imu_msg.linear_acceleration.y = a_meas.y();
    imu_msg.linear_acceleration.z = a_meas.z();
    return imu_msg;
  }

  ros::Time time_stamp;  // ros::Time
  std::string frame_id;  // frame id
  Real dt;               // [s]
  Vector3 a_meas;        // [m/s^2]
  Vector3 w_meas;        // [rad/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace rio
