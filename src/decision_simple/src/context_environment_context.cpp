#include "decision_simple/core/environment_context.hpp"

namespace decision_simple {

  void EnvironmentContext::onRobotStatus(const RobotStatus& robot_status) {
    // TODO: Implement state aggregation from robot status
  }

  void EnvironmentContext::onArmors(const Armors& msg) {
    // TODO: Implement state aggregation from armors
  }

  void EnvironmentContext::onTarget(
      const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    // TODO: Implement state aggregation from target
  }

  void EnvironmentContext::onGameStatus(
      const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
    // TODO: Implement state aggregation from game status
  }

  bool EnvironmentContext::getRobotPoseMap(double& x, double& y, double& yaw) {
    // TODO: Implement robot pose query
    return false;
  }

  bool EnvironmentContext::isNear(double gx, double gy, double tol_xy) {
    // TODO: Implement proximity check
    return false;
  }

  void EnvironmentContext::setState(State s) {
    // TODO: Implement state setter
  }

  void EnvironmentContext::publishChassisMode(ChassisMode mode) {
    // TODO: Implement chassis mode publishing
  }

  void EnvironmentContext::setChassisMode(ChassisMode mode) {
    // TODO: Implement chassis mode setter
  }

  void EnvironmentContext::publishGoalThrottled(
      const geometry_msgs::msg::PoseStamped& goal, rclcpp::Time& last_pub,
      double hz) {
    // TODO: Implement throttled goal publishing
  }

}  // namespace decision_simple
