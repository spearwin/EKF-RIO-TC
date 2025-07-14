#include <ekf_rio_tc/ekf_rio_ros.h>
#include <rio_utils/ros_helper.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace rio;

const std::string kNodeName = "ekf_rio_tc";
const std::string kPrefix = "[" + kNodeName + "]: ";

int main(int argc, char** argv) {
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  EkfRioRos ekf_rio(nh);

  double bag_start = -1.0;
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "bag_start",
                  bag_start);

  double bag_duration = -1.0;
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "bag_duration",
                  bag_duration);
  if (bag_duration <= 0) bag_duration = 1.0e6;

  double sleep_ms = -1.0;
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "sleep_ms",
                  sleep_ms);

  std::string rosbag_path = "";
  if (getRosParameter(nh, kPrefix, RosParameterType::Required, "rosbag_path",
                      rosbag_path))
    ekf_rio.runFromRosbag(rosbag_path, bag_start, bag_duration, sleep_ms);

  return 0;
}
