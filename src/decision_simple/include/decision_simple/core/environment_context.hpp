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
#include "types.hpp"

namespace decision_simple {

  /// Environment state aggregation layer
  /// Translates low-level ROS messages into high-level business facts
  class EnvironmentContext {
  public:
    EnvironmentContext() = default;

    /// Update environment state from robot status
    void onRobotStatus(const RobotStatus& robot_status);

    /// Update environment state from armors
    void onArmors(const Armors& msg);

    /// Update environment state from target
    void onTarget(const Target msg);

    /// Update environment state from game status
    void onGameStatus(const GameStatus msg);

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

    void tickForContext();

  private:
    mutable std::mutex mtx_;
    bool has_robot_status_{false};
    RobotStatus last_robot_status_{};
    bool has_game_status_{false};
    bool match_started_{false};
    std::chrono::nanoseconds match_start_time_{};
    bool has_armors_{false};
    Armors last_armors_{};
    std::optional<Target> last_target_opt_;
    uint8_t last_game_status_{0};
  };

}  // namespace decision_simple
