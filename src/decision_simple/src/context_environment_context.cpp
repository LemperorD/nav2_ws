#include "decision_simple/core/environment_context.hpp"

namespace decision_simple {

  void EnvironmentContext::onRobotStatus(const RobotStatus& robot_status) {
    std::lock_guard<std::mutex> lk(mtx_);

    last_robot_status_ = robot_status;
    has_robot_status_ = true;
  }

  // it is not work
  void EnvironmentContext::onGameStatus(const GameStatus game_status) {
  }

  void EnvironmentContext::onArmors(const Armors& armors) {
    std::lock_guard<std::mutex> lk(mtx_);

    last_armors_ = armors;
    has_armors_ = true;
  }

  void EnvironmentContext::onTarget(const Target target) {
    std::lock_guard<std::mutex> lk(mtx_);

    last_target_opt_ = target;
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
    Snapshot snapshot;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      snapshot.has_rs = has_robot_status_;
      if (snapshot.has_rs) {
        snapshot.rs = last_robot_status_;
      }
      snapshot.has_gs = has_game_status_;
      snapshot.match_started = match_started_;
      snapshot.match_start_time = match_start_time_;
      snapshot.has_armors = has_armors_;
      if (snapshot.has_armors) {
        snapshot.armors = last_armors_;
      }
      snapshot.target_opt = last_target_opt_;
    }
  }
}  // namespace decision_simple
