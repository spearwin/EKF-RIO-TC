#pragma once

#include <ros/node_handle.h>

namespace rio {
enum class RosParameterType { Required, Recommended, Optional };

template <typename T>
static bool getRosParameter(const ros::NodeHandle& nh,
                            const std::string kPrefix,
                            const RosParameterType& param_type,
                            const std::string& param_name, T& param) {
  if (!nh.getParam(param_name, param)) {
    if (param_type == RosParameterType::Optional) {
      ROS_INFO_STREAM(
          kPrefix << "<" << param_name
                  << "> is optional but not configured. Using default value: "
                  << param);
      nh.setParam(param_name, param);
    } else if (param_type == RosParameterType::Recommended) {
      ROS_WARN_STREAM(
          kPrefix << "<" << param_name
                  << "> is recommeded but not configured. Using default value: "
                  << param);
      nh.setParam(param_name, param);
    } else {
      ROS_ERROR_STREAM(kPrefix << "<" << param_name
                               << "> is required but not configured. Exiting!");
      return false;
    }
  }

  return true;
}

}  // namespace rio
