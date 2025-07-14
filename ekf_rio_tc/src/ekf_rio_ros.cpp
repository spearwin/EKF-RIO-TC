#include <ekf_rio_tc/EkfRioCovariance.h>
#include <ekf_rio_tc/EkfRioState.h>
#include <ekf_rio_tc/ekf_rio_ros.h>
#include <ekf_rio_tc/msg_conversion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>
#include <rio_utils/math_helper.h>
#include <rio_utils/ros_helper.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

#include <numeric>

using namespace rio;

EkfRioRos::EkfRioRos(ros::NodeHandle &nh) : initialized_{false} {
  reconfigure_server_.setCallback(
      boost::bind(&EkfRioRos::reconfigureCallback, this, _1, _2));

  // subscribers
  sub_imu_ = nh.subscribe<sensor_msgs::Imu>(
      config_.topic_imu, 2, boost::bind(&EkfRioRos::callbackIMU, this, _1));
  sub_baro_ = nh.subscribe<sensor_msgs::FluidPressure>(
      config_.topic_baro_altimeter, 2,
      boost::bind(&EkfRioRos::callbackBaroAltimter, this, _1));
  sub_radar_ = nh.subscribe<sensor_msgs::PointCloud2>(
      config_.topic_radar_scan, 2,
      boost::bind(&EkfRioRos::callbackRadarScan, this, _1));
  sub_radar_trigger_ = nh.subscribe<std_msgs::Header>(
      config_.topic_radar_trigger, 2,
      boost::bind(&EkfRioRos::callbackRadarTrigger, this, _1));

  // publishers
  pub_cov_ = nh.advertise<ekf_rio_tc::EkfRioCovariance>("covariance", 5);
  pub_nom_ = nh.advertise<ekf_rio_tc::EkfRioState>("state", 5);
  pub_pose_path_ = nh.advertise<nav_msgs::Path>("pose_path", 1);
  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_twist_ = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
  pub_radar_scan_inlier_ =
      nh.advertise<sensor_msgs::PointCloud2>("radar_scan_inlier", 10);

  if (config_.republish_ground_truth) {
    pub_ground_truth_pose_ = nh.advertise<geometry_msgs::PoseStamped>(
        config_.topic_ground_truth_pose, 1000);
    pub_ground_truth_twist_ = nh.advertise<geometry_msgs::TwistStamped>(
        config_.topic_ground_truth_twist, 1000);
    pub_ground_truth_twist_body_ = nh.advertise<geometry_msgs::TwistStamped>(
        config_.topic_ground_truth_twist_body, 1000);
  }

  ros::Duration(0.5).sleep();
}

bool EkfRioRos::initImu(const ImuDataStamped &imu_data) {
  imu_init_.emplace_back(imu_data);

  const auto T_init_so_far =
      (imu_init_.back().time_stamp - imu_init_.front().time_stamp).toSec();
  if (T_init_so_far > config_.T_init) {
    Real baro_h_0 = 0.0;
    if (config_.altimeter_update) {
      if (baro_init_vec_.size() > 0) {
        baro_h_0 =
            std::accumulate(baro_init_vec_.begin(), baro_init_vec_.end(), 0.0) /
            baro_init_vec_.size();
        ROS_INFO_STREAM(kStreamingPrefix << "Initialized baro h_0: "
                                         << baro_h_0);
        baro_initialized_ = true;
      } else {
        ROS_ERROR_STREAM(
            kStreamingPrefix
            << "Unable to init baro --> no measurements received!");
      }
    }

    initialized_ = ekf_rio_filter_.init(imu_init_, baro_h_0);
    filter_start_stamp_ = ekf_rio_filter_.getTimestamp();
    filter_start_wall_time_ = ros::WallTime::now();
    clearQueues(filter_start_stamp_);
  } else {
    const uint mod = 1.0 / imu_data.dt + 1;
    if (imu_init_.size() % mod == 0)
      ROS_INFO("%s Init progress: %0.2f / %0.2f seconds",
               kStreamingPrefix.c_str(), T_init_so_far, config_.T_init);
  }
  return initialized_;
}

void EkfRioRos::run() {
  ROS_INFO_STREAM(kStreamingPrefix << "Navigation filter started!");

  ros::WallRate r(1000);
  while (ros::ok()) {
    iterate();
    ros::spinOnce();
    r.sleep();
  }
}

void EkfRioRos::runFromRosbag(const std::string &rosbag_path,
                              const Real bag_start, const Real bag_duration,
                              const Real sleep_ms) {
  rosbag::Bag source_bag;
  source_bag.open(rosbag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(config_.topic_imu);
  topics.push_back(config_.topic_baro_altimeter);
  topics.push_back(config_.topic_radar_scan);
  topics.push_back(config_.topic_radar_trigger);

  if (config_.republish_ground_truth) {
    topics.push_back(config_.topic_ground_truth_pose);
    topics.push_back(config_.topic_ground_truth_twist);
    topics.push_back(config_.topic_ground_truth_twist_body);
  }

  rosbag::View view(source_bag, rosbag::TopicQuery(topics));

  auto first_timestamp = ros::TIME_MIN;

  for (const rosbag::MessageInstance &m : view) {
    if (first_timestamp == ros::TIME_MIN) first_timestamp = m.getTime();

    if ((m.getTime() - first_timestamp).toSec() < bag_start) continue;

    if ((m.getTime() - first_timestamp).toSec() > bag_duration) break;

    const auto topic = m.getTopic();
    if (topic == config_.topic_imu) {
      const auto imu_msg_bag = m.instantiate<sensor_msgs::Imu>();
      if (imu_msg_bag != NULL) {
        callbackIMU(imu_msg_bag);
      }
    } else if (topic == config_.topic_baro_altimeter) {
      const auto baro_msg = m.instantiate<sensor_msgs::FluidPressure>();
      if (baro_msg != NULL) callbackBaroAltimter(baro_msg);
    } else if (topic == config_.topic_radar_scan) {
      const auto radar_scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (radar_scan != NULL) callbackRadarScan(radar_scan);

      if (sleep_ms > 0) ros::Duration(sleep_ms / 1.0e3).sleep();
    } else if (topic == config_.topic_radar_trigger) {
      const auto radar_trigger_msg = m.instantiate<std_msgs::Header>();
      if (radar_trigger_msg != NULL) callbackRadarTrigger(radar_trigger_msg);
    } else if (config_.republish_ground_truth) {
      if (topic == config_.topic_ground_truth_pose) {
        const auto msg = m.instantiate<geometry_msgs::PoseStamped>();
        if (msg) pub_ground_truth_pose_.publish(msg);
      } else if (topic == config_.topic_ground_truth_twist) {
        const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
        if (msg) pub_ground_truth_twist_.publish(msg);
      } else if (topic == config_.topic_ground_truth_twist_body) {
        const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
        if (msg) pub_ground_truth_twist_body_.publish(msg);
      }
    }

    iterate();
    ros::spinOnce();
  }

  //   publish();
  printStats();

  return;
}

void EkfRioRos::iterate() {
  mutex_.lock();
  if (!initialized_) {
    if (!deque_baro_.empty()) {
      auto baro_msg = deque_baro_.front();
      baro_init_vec_.emplace_back(
          baro_altimeter_.calculate_rel_neg_height(baro_msg));
      deque_baro_.pop_front();
    }

    if (!deque_imu_.empty()) {
      imu_data_ = deque_imu_.front();
      initImu(imu_data_);
      deque_imu_.pop_front();
    }
  } else {
    if (!deque_radar_.empty()) {
      auto radar_data_msg = deque_radar_.front();
      deque_radar_.pop_front();

      // 1. Determine radar update time (using radar trigger if available)
      ros::Time radar_update_time =
          radar_data_msg.header.stamp +
          ros::Duration(ekf_rio_filter_.getBias().t_d);

      if (config_.run_without_radar_trigger) {
        deque_radar_trigger_.clear();
      } else {
        auto radar_trigger_msg = deque_radar_trigger_.front();
        deque_radar_trigger_.pop_front();

        // check time diff
        if (std::fabs(radar_update_time.toSec() -
                      radar_trigger_msg.stamp.toSec()) >
            1.0 / config_.radar_rate) {
          ROS_WARN_STREAM("Radar trigger message is too old!");
        }
        radar_update_time = radar_trigger_msg.stamp;
        config_.use_temp_calib = false;
      }

      // 2. Collect IMU, barometer, and IMU for radar velocity estimation
      std::vector<ImuDataStamped> imu_process;
      std::vector<sensor_msgs::FluidPressure> baro_process;
      std::vector<Event> events;
      findProcessMsg(radar_update_time, imu_process, baro_process,
                     radar_w_deque_, events);

      // 3. Propagate IMU data and perform barometer measurement updates
      for (const auto &event : events) {
        if (event.type == Event::IMU) {
          profiler_.start("processIMU");
          imu_data_ = imu_process[event.index];
          ekf_rio_filter_.propagate(imu_data_);
          profiler_.stop("processIMU");

          if (1.0 / (imu_data_.time_stamp - last_timestamp_pub_).toSec() <
              30.0) {
            profiler_.start("publishSpinOnce");
            publish();
            last_timestamp_pub_ = ekf_rio_filter_.getTimestamp();
            profiler_.stop("publishSpinOnce");
          }
        }
        if (config_.altimeter_update) {
          if (event.type == Event::BARO) {
            profiler_.start("baro_altimeter_update");
            const auto &baro_msg = baro_process[event.index];
            const auto h_rel =
                baro_altimeter_.calculate_rel_neg_height(baro_msg);
            ekf_rio_filter_.updateAltimeter(h_rel, config_.sigma_altimeter);
            profiler_.stop("baro_altimeter_update");
          }
        }
      }

      deque_imu_.erase(deque_imu_.begin(),
                       deque_imu_.begin() + imu_process.size());
      deque_baro_.erase(deque_baro_.begin(),
                        deque_baro_.begin() + baro_process.size());

      // 4. Perform radar ego-velocity estimation and update
      Vector3 a_mean(0, 0, 0);
      Vector3 w_mean(0, 0, 0);
      if (!radar_w_deque_.empty()) {
        for (const auto &imu : radar_w_deque_) {
          a_mean += imu.a_meas;
          w_mean += imu.w_meas;
        }
        a_mean /= radar_w_deque_.size();
        w_mean /= radar_w_deque_.size();
      } else {
        a_mean = imu_data_.a_meas;
        w_mean = imu_data_.w_meas;
      }

      Vector3 v_RinR, sigma_v_RinR;
      sensor_msgs::PointCloud2 inlier_radar_scan;

      profiler_.start("estimate_radar_velocity");
      if (radar_ego_velocity_.estimate(radar_data_msg, v_RinR, sigma_v_RinR,
                                       inlier_radar_scan)) {
        profiler_.stop("estimate_radar_velocity");

        profiler_.start("radar_velocity_kf_update");
        bool valid = false;
        if (config_.use_w)
          valid = ekf_rio_filter_.updateRadarEgoVelocity(
              v_RinR, sigma_v_RinR, a_mean, w_mean,
              config_.outlier_percentil_radar, config_.use_temp_calib,
              config_.use_ext_calib);
        else
          valid = ekf_rio_filter_.updateRadarEgoVelocity(
              v_RinR, sigma_v_RinR, Vector3(0, 0, 0), Vector3(0, 0, 0),
              config_.outlier_percentil_radar, config_.use_temp_calib,
              config_.use_ext_calib);
        profiler_.stop("radar_velocity_kf_update");
      }

      inlier_radar_scan.header.frame_id = config_.radar_frame_id;
      pub_radar_scan_inlier_.publish(inlier_radar_scan);
    }
  }
  mutex_.unlock();

  return;
}

void EkfRioRos::clearQueues(const ros::Time &time_stamp) {
  while (!deque_baro_.empty() &&
         deque_baro_.front().header.stamp < time_stamp) {
    deque_baro_.pop_front();
  }
  while (!deque_radar_trigger_.empty() &&
         deque_radar_trigger_.front().stamp < time_stamp) {
    deque_radar_trigger_.pop_front();
  }
  while (!deque_radar_.empty() &&
         deque_radar_.front().header.stamp +
                 ros::Duration(ekf_rio_filter_.getBias().t_d) <
             time_stamp) {
    deque_radar_.pop_front();
  }

  return;
}

void EkfRioRos::findProcessMsg(
    const ros::Time &time_stamp, std::vector<ImuDataStamped> &imu_data_vec,
    std::vector<sensor_msgs::FluidPressure> &baro_data_vec,
    std::vector<ImuDataStamped> &radar_w_queue, std::vector<Event> &events) {
  imu_data_vec.clear();
  baro_data_vec.clear();
  radar_w_queue.clear();
  events.clear();

  if (!deque_imu_.empty() || !deque_baro_.empty()) {
    for (const auto &imu : deque_imu_) {
      if (imu.time_stamp <= time_stamp) {
        imu_data_vec.push_back(imu);
      } else
        break;
    }

    for (const auto &imu : deque_imu_) {
      if (imu.time_stamp > time_stamp &&
          imu.time_stamp <=
              time_stamp + ros::Duration(config_.radar_frame_ms / 1.0e3)) {
        radar_w_queue.push_back(imu);
      }
    }

    for (const auto &baro : deque_baro_) {
      if (baro.header.stamp <= time_stamp) {
        baro_data_vec.push_back(baro);
      } else
        break;
    }
  }

  for (size_t i = 0; i < imu_data_vec.size(); ++i)
    events.push_back({imu_data_vec[i].time_stamp, Event::IMU, i});

  for (size_t i = 0; i < baro_data_vec.size(); ++i)
    events.push_back({baro_data_vec[i].header.stamp, Event::BARO, i});

  std::sort(events.begin(), events.end(),
            [](const Event &a, const Event &b) { return a.stamp < b.stamp; });

  return;
}

void EkfRioRos::reconfigureCallback(ekf_rio_tc::EkfRioConfig &config,
                                    uint32_t level) {
  ekf_rio_filter_.configure(config);
  radar_ego_velocity_.configure(config);
  config_ = config;

  return;
}

void EkfRioRos::callbackIMU(const sensor_msgs::ImuConstPtr &imu_msg) {
  mutex_.lock();
  Real dt = 2.4e-3;
  if (std::fabs(last_imu_.dt) > 1.0e-6)
    dt = (imu_msg->header.stamp - last_imu_.time_stamp).toSec();
  last_imu_ = ImuDataStamped(imu_msg, dt);

  if (config_.invert_imu) {
    last_imu_.a_meas.y() = -last_imu_.a_meas.y();
    last_imu_.a_meas.z() = -last_imu_.a_meas.z();
    last_imu_.w_meas.y() = -last_imu_.w_meas.y();
    last_imu_.w_meas.z() = -last_imu_.w_meas.z();
  }

  deque_imu_.push_back(last_imu_);
  mutex_.unlock();

  return;
}

void EkfRioRos::callbackBaroAltimter(
    const sensor_msgs::FluidPressureConstPtr &baro_msg) {
  mutex_.lock();
  deque_baro_.push_back(*baro_msg);
  mutex_.unlock();

  return;
}

void EkfRioRos::callbackRadarScan(
    const sensor_msgs::PointCloud2ConstPtr &radar_msg) {
  mutex_.lock();
  // offset delay
  sensor_msgs::PointCloud2 msg = *radar_msg;
  msg.header.stamp = msg.header.stamp + ros::Duration(config_.offset_delay);
  deque_radar_.push_back(msg);
  mutex_.unlock();

  return;
}

void EkfRioRos::callbackRadarTrigger(
    const std_msgs::HeaderConstPtr &trigger_msg) {
  mutex_.lock();
  deque_radar_trigger_.push_back(*trigger_msg);
  mutex_.unlock();

  return;
}

void EkfRioRos::publish() {
  const NavigationSolution nav_sol = ekf_rio_filter_.getNavigationSolution();
  const Isometry pose_ros = nav_sol.getPoseRos();

  // pose
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ekf_rio_filter_.getTimestamp();
  pose_stamped.header.frame_id = config_.global_frame_id;
  pose_stamped.pose = tf2::toMsg(pose_ros);
  pub_pose_.publish(pose_stamped);

  // twist
  geometry_msgs::TwistStamped twist_stamped;
  twist_stamped.header = pose_stamped.header;
  tf2::toMsg(nav_sol.getVelocityRos(), twist_stamped.twist.linear);
  tf2::toMsg(
      pose_ros.linear() * (imu_data_.w_meas - ekf_rio_filter_.getBias().gyro),
      twist_stamped.twist.angular);
  pub_twist_.publish(twist_stamped);

  // pose path (5Hz)
  pose_path_.poses.emplace_back(pose_stamped);
  if ((ekf_rio_filter_.getTimestamp() - last_timestamp_pose_pub_).toSec() >
      1.0 / 5.0) {
    pose_path_.header = pose_stamped.header;
    pub_pose_path_.publish(pose_path_);
    last_timestamp_pose_pub_ = ekf_rio_filter_.getTimestamp();
  }

  // covariance
  pub_cov_.publish(
      msg_conversion::toCovMsg(ekf_rio_filter_, config_.global_frame_id));

  // filter state
  pub_nom_.publish(
      msg_conversion::toStateMsg(ekf_rio_filter_, config_.global_frame_id));

  // tf global -> body
  geometry_msgs::TransformStamped T_global_body;
  T_global_body = tf2::eigenToTransform(pose_ros);
  T_global_body.header = pose_stamped.header;
  T_global_body.child_frame_id = config_.body_frame_id;
  tf_broadcaster_.sendTransform(T_global_body);

  // tf body -> radar
  geometry_msgs::TransformStamped T_body_radar;
  T_body_radar = tf2::eigenToTransform(ekf_rio_filter_.getExtT());
  T_body_radar.header.stamp = ekf_rio_filter_.getTimestamp();
  T_body_radar.header.frame_id = config_.body_frame_id;
  T_body_radar.child_frame_id = config_.radar_frame_id;
  tf_broadcaster_.sendTransform(T_body_radar);

  return;
}

void EkfRioRos::printStats() {
  const auto dataset_length =
      (ekf_rio_filter_.getTimestamp() - filter_start_stamp_).toSec();
  const auto processing_time =
      (ros::WallTime::now() - filter_start_wall_time_).toSec();
  ROS_INFO_STREAM(kStreamingPrefix << "Analysis");

  std::cout << "Detailed runtimes:\n" << profiler_.toString() << std::endl;

  std::cout << "Runtime" << std::endl;
  printf("  Took %0.2fs to process %0.2fs --> %0.2f x realtime\n",
         processing_time, dataset_length, dataset_length / processing_time);

  printf("  Pure processing took %0.2fs --> %0.2f x realtime\n",
         profiler_.getTotalRuntime(),
         dataset_length / profiler_.getTotalRuntime());

  Real trajectory_length = 0.0;
  const uint step = 50;
  for (uint k = 0; k < pose_path_.poses.size() - 2 * step - 1; k += step) {
    const auto p_0 = pose_path_.poses.at(k).pose.position;
    const auto p_1 = pose_path_.poses.at(k + step).pose.position;
    trajectory_length +=
        Vector3(p_0.x - p_1.x, p_0.y - p_1.y, p_0.z - p_1.z).norm();
  }

  const auto idx = ekf_rio_filter_.getErrorIdx();
  const Matrix C = ekf_rio_filter_.getCovarianceMatrix();

  const Vector3 p_final = ekf_rio_filter_.getNavigationSolution().getPosition();
  const Vector3 att_final =
      ekf_rio_filter_.getNavigationSolution().getEuler_n_b().to_degrees();

  const Vector3 p_0(pose_path_.poses.front().pose.position.x,
                    pose_path_.poses.front().pose.position.y,
                    pose_path_.poses.front().pose.position.z);

  const Quaternion q_0(pose_path_.poses.front().pose.orientation.w,
                       pose_path_.poses.front().pose.orientation.x,
                       pose_path_.poses.front().pose.orientation.y,
                       pose_path_.poses.front().pose.orientation.z);

  const Quaternion q_final_err =
      q_0.inverse() * ekf_rio_filter_.getNavigationSolution().getAttitude();
  NavigationSolution q_err_tmp;
  q_err_tmp.setAttitude(q_final_err);

  const Vector3 p_error = p_final - p_0;

  // clang-format off
  std::cout << "Evaluation (assuming the start and end pose are equal:" << std::endl;
  printf("  Trajectory length: %0.2fm\n", trajectory_length);
  printf("  Final pose: %0.2f m, %0.2f m, %0.2f m, %0.2f deg, %0.2f deg, %0.2f deg\n",
           p_final.x(), p_final.y(), p_final.z(),att_final.x(), att_final.y(), att_final.z());

  printf("  Final 3-Sigma Position: %0.2fm, %0.2fm, %0.2fm\n",
           3 * std::sqrt(C(idx.position, idx.position)),
           3 * std::sqrt(C(idx.position + 1, idx.position + 1)),
           3 * std::sqrt(C(idx.position + 2, idx.position + 2)));

  printf("  Position Error 3D: %0.2f m, %0.2f m, %0.2f m -> %0.2f m -> %0.2f percent\n",
           p_error.x(), p_error.y(), p_error.z(), p_error.norm(), p_error.norm() / trajectory_length * 100.0);

  printf("  Position Error 2D: %0.2f m, %0.2f m -> %0.2f m -> %0.2f percent\n",
           p_error.x(), p_error.y(), p_error.head(2).norm(), p_error.head(2).norm() / trajectory_length * 100.0);

  printf("  Final 3-Sigma Yaw: %0.2fdeg\n",
         3 * angles::to_degrees(std::sqrt(C(idx.attitude + 2, idx.attitude + 2))));

  printf("  Attitude Error: %0.2fdeg\n\n", config_.q_ItoG_yaw_0_deg - att_final.z());
  // clang-format on

  return;
}
