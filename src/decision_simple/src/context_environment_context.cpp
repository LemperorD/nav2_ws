#include "decision_simple/core/environment_context.hpp"

namespace decision_simple {

  EnvironmentContext::EnvironmentContext(const ContextConfig& context_config)
      : hp_enter_supply_(context_config.hp_enter_supply),
        hp_exit_supply_(context_config.hp_exit_supply),
        ammo_min_(context_config.ammo_min),
        combat_max_distance_(context_config.combat_max_distance),
        require_game_running_(context_config.require_game_running),
        start_delay_sec_(context_config.start_delay_sec) {
  }

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

    // 从“比赛中(4)” -> “非比赛中”，复位，为下一局做准~备
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

  bool EnvironmentContext::detectEnemy(
      const Armors& armors, const std::optional<Target>& target_opt) const {
    if (target_opt.has_value() && target_opt->tracking) {
      const double x = target_opt->position.x;
      const double y = target_opt->position.y;
      const double z = target_opt->position.z;
      const double dist = std::sqrt(x * x + y * y + z * z);
      return dist <= combat_max_distance_;
    }

    for (const auto& a : armors.armors) {
      const double x = a.pose.position.x;
      const double y = a.pose.position.y;
      const double z = a.pose.position.z;
      const double dist = std::sqrt(x * x + y * y + z * z);
      if (dist <= combat_max_distance_) {
        return true;
      }
    }
    return false;
  }

  Snapshot EnvironmentContext::getSnapshot(Stamp now) {
    Snapshot snapshot;
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
    snapshot.state = state_;
    if (snapshot.has_armors) {
      snapshot.armors = last_armors_;
    }
    snapshot.target_opt = last_target_opt_;

    if (snapshot.has_armors || snapshot.target_opt.has_value()) {
      snapshot.enemy = detectEnemy(snapshot.armors, snapshot.target_opt);
    }
    if (snapshot.enemy) {
      snapshot.last_enemy_seen_ = now;
    }
    const int64_t last_enemy_ns =
        (static_cast<int64_t>(snapshot.last_enemy_seen_.sec) * 1000000000LL)
        + snapshot.last_enemy_seen_.nanosec;
    snapshot.enemy_recent = (last_enemy_ns != 0)
                         && ((now.nanosec - last_enemy_ns) * 1e-9
                             <= attack_hold_sec_);

    if (snapshot.rs.is_hp_deduced) {
      snapshot.last_attacked_ = now;
    }
    const int64_t last_attacked_ns =
        (static_cast<int64_t>(snapshot.last_attacked_.sec) * 1000000000LL)
        + snapshot.last_attacked_.nanosec;
    snapshot.attacked_recent = (last_attacked_ns != 0)
                            && ((now.nanosec - last_attacked_ns) * 1e-9
                                <= attacked_hold_sec_);

    snapshot.default_spin_latched = default_spin_latched_;
    return snapshot;
  }

  bool EnvironmentContext::isStatusBad(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp < hp_enter_supply_) || (ammo <= ammo_min_);
  }

  void EnvironmentContext::setState(State s) {
    if (state_ == s) {
      is_state_changed = false;
      return;
    }
    is_state_changed = true;
    state_ = s;
  }

  bool EnvironmentContext::isStateChanged() {
    return is_state_changed;
  }

  Readiness EnvironmentContext::checkReadiness(nanoseconds now) {
    if (!has_robot_status_) {
      return {Readiness::Status::NO_RS};
    }
    if (require_game_running_) {
      if (!has_game_status_) {
        return {Readiness::Status::NO_GS};
      }
      if (!match_started_) {
        return {Readiness::Status::NOT_STARTED};
      }

      const double current_elapsed = (now - match_start_time_) * 1e-9;
      if (current_elapsed < start_delay_sec_) {
        return {Readiness::Status::IN_DELAY, current_elapsed};
      }
    }

    return {Readiness::Status::READY};
  }

  void EnvironmentContext::updatePose(const double x, const double y,
                                      const double yaw) {
    std::lock_guard<std::mutex> lk(mtx_);
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    has_pose_ = true;
  }

  bool EnvironmentContext::isNear(double target_x, double target_y,
                                  double tolerance) const {
    if (!has_pose_) {
      return false;
    }
    double dx = x_ - target_x;
    double dy = y_ - target_y;
    return std::hypot(dx, dy) <= tolerance;
  }

}  // namespace decision_simple
