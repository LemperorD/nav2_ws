#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "Interfaces.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
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

  class EnvironmentContext {
    void tick();

    bool getRobotPoseMap(double& x, double& y, double& yaw);
    bool isNear(double gx, double gy, double tol_xy);

    void setState(State s);

    void publishChassisMode(chassisMode mode);
    void setChassisMode(chassisMode mode);

    void publishGoalThrottled(const geometry_msgs::msg::PoseStamped& goal,
                              rclcpp::Time& last_pub, double hz);
  };
}  // namespace decision_simple