#include <ekf_rio_tc/ekf_rio_ros.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace rio;

const std::string kNodeName = "ekf_rio_tc";
const std::string kPrefix = "[" + kNodeName + "]: ";

int main(int argc, char** argv) {
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  EkfRioRos ekf_rio(nh);

  ekf_rio.run();

  return 0;
}
