#include "decision_simple/core/environment_context.hpp"

namespace decision_simple {

  void EnvironmentContext::onRobotStatus(const RobotStatus& robot_status) {
    std::lock_guard<std::mutex> lk(mtx_);

    last_robot_status_ = robot_status;
    has_robot_status_ = true;
  }

  void EnvironmentContext::onGameStatus(const GameStatus game_status,
                                        int64_t match_start_time_ns) {
    std::lock_guard<std::mutex> lk(mtx_);

    const uint8_t prev = last_game_status_;
    last_game_status_ = game_status.game_progress;
    has_game_status_ = true;

    if (prev != 4 && last_game_status_ == 4) {
      match_started_ = true;
      match_start_time_ = match_start_time_ns;
      is_game_started = true;
    }

    // 从“比赛中(4)” -> “非比赛中”，复位，为下一局做准备
    if (prev == 4 && last_game_status_ != 4) {
      match_started_ = false;
      match_start_time_ = 0;
      // 清掉上一局遗留的默认点小陀螺锁存
      default_spin_latched_ = false;
      is_game_started = false;
      is_game_over = true;
    }
  }

  bool EnvironmentContext::isGameStarted() {
    return is_game_started;
  }

  bool EnvironmentContext::isGameOver() {
    return is_game_over;
  }

  void EnvironmentContext::resetGameOver() {
    is_game_over = false;
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

  void EnvironmentContext::tickForContext(Snapshot& snapshot) {
    snapshot.has_rs = has_robot_status_;
    if (snapshot.has_rs) {
      snapshot.rs = last_robot_status_;
    }
    snapshot.has_gs = has_game_status_;
    snapshot.match_started = match_started_;
    snapshot.match_start_time.sec = static_cast<int32_t>(match_start_time_
                                                         / 1000000000LL);
    snapshot.match_start_time.nanosec = static_cast<uint32_t>(match_start_time_
                                                              % 1000000000LL);
    snapshot.has_armors = has_armors_;
    if (snapshot.has_armors) {
      snapshot.armors = last_armors_;
    }
    snapshot.target_opt = last_target_opt_;
  }
}  // namespace decision_simple
