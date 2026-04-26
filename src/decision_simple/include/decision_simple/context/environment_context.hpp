#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "decision_simple/core/interfaces.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace decision_simple {

  /// Environment state aggregation layer
  /// Translates low-level ROS messages into high-level business facts
  class EnvironmentContext {
  public:
    EnvironmentContext() = default;

    /// Update environment state from robot status
    void onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

    /// Update environment state from armors
    void onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg);

    /// Update environment state from target
    void onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg);

    /// Update environment state from game status
    void onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

    /// Get robot pose in map frame
    bool getRobotPoseMap(double& x, double& y, double& yaw);

    /// Check if robot is near target position
    bool isNear(double gx, double gy, double tol_xy);

    /// Set robot state
    void setState(State s);

    /// Publish chassis mode
    void publishChassisMode(ChassisMode mode);

    /// Set chassis mode
    void setChassisMode(ChassisMode mode);

    /// Publish goal pose with throttling
    void publishGoalThrottled(const geometry_msgs::msg::PoseStamped& goal,
                              rclcpp::Time& last_pub, double hz);
  };

}  // namespace decision_simple
