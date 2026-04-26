#pragma once

#include "decision_simple/core/types.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace decision_simple {

  /// Convert ROS Quaternion message to domain Quaternion
  Quaternion ConvertQuaternion(const geometry_msgs::msg::Quaternion& ros_quat);

  /// Convert ROS Pose message to domain Pose3D
  Pose3D ConvertPose(const geometry_msgs::msg::Pose& ros_pose);

  /// Convert ROS RobotStatus message to domain RobotStatus
  /// @throws std::invalid_argument if msg is nullptr
  RobotStatus ConvertRobotStatus(
      const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

}  // namespace decision_simple
