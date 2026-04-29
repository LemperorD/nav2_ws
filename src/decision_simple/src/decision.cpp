#include "decision_simple/core/decision.hpp"

namespace decision_simple {

  Decision::Decision(const ContextConfig& context_config)
      : hp_enter_supply_(context_config.hp_enter_supply),
        hp_exit_supply_(context_config.hp_exit_supply),
        ammo_min_(context_config.ammo_min),
        combat_max_distance_(context_config.combat_max_distance),
        require_game_running_(context_config.require_game_running),
        start_delay_sec_(context_config.start_delay_sec),
        supply_x_(context_config.supply_x_),
        supply_y_(context_config.supply_y_),
        supply_yaw_(context_config.supply_yaw),
        default_x_(context_config.default_x_),
        default_y_(context_config.default_y_),
        default_yaw_(context_config.default_yaw_) {
  }
  DecisionAction Decision::compute(const Snapshot& s) {
    DecisionAction action;
    if (s.state == State::SUPPLY) {
      if (!isStatusRecovered(s.rs)) {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
        action.target_x = supply_x_;
        action.target_y = supply_y_;
        action.target_yaw = supply_yaw_;
        action.next_state = State::SUPPLY;
        return action;
      }
    }

    if (isStatusBad(s.rs)) {
      action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      action.target_x = supply_x_;
      action.target_y = supply_y_;
      action.target_yaw = supply_yaw_;
      action.next_state = State::SUPPLY;
      return action;
    }

    if (s.enemy_recent) {
      action.next_state = State::ATTACK;
      action.default_spin_latched = false;
      if (s.attacked_recent) {
        action.chassis_mode = ChassisMode::LITTLE_TES;
      } else {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      }
    }

    action.next_state = State::DEFAULT;
    // 1. 先进入 0.3m 小圈 -> 小陀螺模式
    if (s.at_center) {
      action.default_spin_latched = true;
    }
    // 2. 出了 0.8m 大圈，取消陀螺
    if (!s.in_center_keep_spin) {
      action.default_spin_latched = false;
    }
    if (s.attacked_recent) {
      action.chassis_mode = ChassisMode::LITTLE_TES;

    } else {
      if (action.default_spin_latched) {
        action.chassis_mode = ChassisMode::LITTLE_TES;

      } else {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      }
    }
    action.target_x = supply_x_;
    action.target_y = supply_y_;
    action.target_yaw = supply_yaw_;
    return action;
  }

  bool Decision::isStatusRecovered(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp >= hp_exit_supply_) && (ammo > ammo_min_);
  }

  bool Decision::isStatusBad(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp < hp_enter_supply_) || (ammo <= ammo_min_);
  }

  DecisionAction Decision::computeAction(const Snapshot& snapshot) const {
    DecisionAction action;
    action.should_publish_goal = true;

    // ================= 1) 补给保持 =================
    if (snapshot.state == State::SUPPLY) {
      if (!isStatusRecovered(snapshot.rs)) {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
        action.next_state = State::SUPPLY;
        action.target_x = supply_x_;
        action.target_y = supply_y_;
        action.target_yaw = supply_yaw_;
        return action;
      }
    }

    // ================= 2) 状态差 -> 进补给 =================
    if (isStatusBad(snapshot.rs)) {
      action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      action.next_state = State::SUPPLY;
      action.target_x = supply_x_;
      action.target_y = supply_y_;
      action.target_yaw = supply_yaw_;
      return action;
    }

    // ================= 3) 状态好 -> 有敌攻击 =================
    if (snapshot.enemy_recent) {
      action.next_state = State::ATTACK;
      action.default_spin_latched = false;
      action.target_x = default_x_;
      action.target_y = default_y_;
      action.target_yaw = default_yaw_;

      // Compute attack goal (modifies snapshot in-place)
      buildAttackGoal(const_cast<Snapshot&>(snapshot), snapshot.armors,
                      snapshot.target_opt);

      if (snapshot.attacked_recent) {
        action.chassis_mode = ChassisMode::LITTLE_TES;
      } else {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      }

      // Use cached attack goal position if available
      if (snapshot.has_attack_goal) {
        action.target_x = snapshot.last_attack_position.x;
        action.target_y = snapshot.last_attack_position.y;
        action.target_yaw = snapshot.last_attack_yaw;
      }
      return action;
    }

    // ================= 4) 默认：去中心点 =================
    action.next_state = State::DEFAULT;
    action.target_x = default_x_;
    action.target_y = default_y_;
    action.target_yaw = default_yaw_;

    // 1. 先进入 0.3m 小圈 -> 小陀螺模式
    if (snapshot.at_center) {
      action.default_spin_latched = true;
    }
    // 2. 出了 0.8m 大圈，取消陀螺
    if (!snapshot.in_center_keep_spin) {
      action.default_spin_latched = false;
    }

    if (snapshot.attacked_recent) {
      action.chassis_mode = ChassisMode::LITTLE_TES;
    } else {
      action.chassis_mode = action.default_spin_latched
                              ? ChassisMode::LITTLE_TES
                              : ChassisMode::CHASSIS_FOLLOWED;
    }

    return action;
  }

  bool Decision::buildAttackGoal(
      Snapshot& snapshot, const Armors& armors,
      const std::optional<Target>& target_opt) const {
    // Use tracked target if available and tracking
    if (target_opt.has_value() && target_opt->tracking) {
      snapshot.last_attack_position = target_opt->position;
      snapshot.last_attack_yaw = target_opt->yaw;
      snapshot.has_attack_goal = true;
      return true;
    }

    // Otherwise use closest armor
    if (armors.armors.empty()) {
      return false;
    }

    const auto* best = &armors.armors.front();
    double best_dist = 1e18;
    for (const auto& a : armors.armors) {
      const double x = a.pose.position.x;
      const double y = a.pose.position.y;
      const double z = a.pose.position.z;
      const double dist = std::sqrt(x * x + y * y + z * z);
      if (dist < best_dist) {
        best_dist = dist;
        best = &a;
      }
    }

    snapshot.last_attack_position = best->pose.position;
    snapshot.last_attack_yaw = 0.0;
    snapshot.has_attack_goal = true;
    return true;
  }

}  // namespace decision_simple