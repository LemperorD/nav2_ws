#include "decision_simple/core/environment_context.hpp"

namespace decision_simple {

  void EnvironmentContext::onRobotStatus(const RobotStatus& robot_status) {
    // TODO: Implement state aggregation from robot status
    RobotStatus last_robot_status = robot_status;
  }

  void EnvironmentContext::onArmors(const Armors& msg) {
    // TODO: Implement state aggregation from armors
  }

  void EnvironmentContext::onTarget(const Target msg) {
    // TODO: Implement state aggregation from target
  }

  void EnvironmentContext::onGameStatus(const GameStatus msg) {
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

  void EnvironmentContext::tickForContext() {
    RobotStatus rs;
    bool has_rs = false;
    bool has_gs = false;
    bool match_started = false;
    rclcpp::Time match_start_time;
    Armors armors;
    bool has_armors = false;
    std::optional<Target> target_opt;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      has_rs = has_robot_status_;
      if (has_rs) {
        rs = last_robot_status_;
      }
      has_gs = has_game_status_;
      match_started = match_started_;
      match_start_time = match_start_time_;
      has_armors = has_armors_;
      if (has_armors) {
        armors = last_armors_;
      }
      target_opt = last_target_opt_;
    }
  }
}  // namespace decision_simple
