#pragma once

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "decision_simple/core/types.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace decision_simple {

  /// Convert ROS Quaternion message to domain Quaternion
  Quaternion ConvertQuaternion(const geometry_msgs::msg::Quaternion& ros_quat);

  /// Convert ROS Pose message to domain Pose3D
  Position ConvertPoint(const geometry_msgs::msg::Point& ros_point);

  Pose3D ConvertPose(const geometry_msgs::msg::Pose& ros_pose);

  /// Convert ROS RobotStatus message to domain RobotStatus
  /// @throws std::invalid_argument if msg is nullptr
  RobotStatus ConvertRobotStatus(
      const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

  /// Convert ROS GameStatus message to domain GameStatus
  /// @throws std::invalid_argument if msg is nullptr
  GameStatus ConvertGameStatus(
      const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

  /// Convert ROS Armors message to domain Armors
  /// @throws std::invalid_argument if msg is nullptr
  Armors ConvertArmors(
      const auto_aim_interfaces::msg::Armors::SharedPtr& ros_armorsmsg);

  Armor ConvertArmor(const auto_aim_interfaces::msg::Armor& ros_armormsg);

  /// Convert ROS Target message to domain Target
  /// @throws std::invalid_argument if msg is nullptr
  Target ConvertTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg);

}  // namespace decision_simple
