#include "decision_simple/core/decision.hpp"

namespace decision_simple {

  Decision::Decision(const ContextConfig& context_config)
      : config(context_config) {
  }

  DecisionAction Decision::compute(const Snapshot& s) {
    DecisionAction action;
    if (s.state == State::SUPPLY) {
      if (!isStatusRecovered(s.rs)) {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
        action.target_x = config.supply_x;
        action.target_y = config.supply_y;
        action.target_yaw = config.supply_yaw;
        action.next_state = State::SUPPLY;
        return action;
      }
    }

    if (isStatusBad(s.rs)) {
      action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      action.target_x = config.supply_x;
      action.target_y = config.supply_y;
      action.target_yaw = config.supply_yaw;
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
    action.target_x = config.supply_x;
    action.target_y = config.supply_y;
    action.target_yaw = config.supply_yaw;
    return action;
  }

  bool Decision::isStatusRecovered(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp >= config.hp_exit_supply) && (ammo > config.ammo_min);
  }

  bool Decision::isStatusBad(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp < config.hp_enter_supply) || (ammo <= config.ammo_min);
  }

  DecisionAction Decision::computeAction(const Snapshot& snapshot) const {
    DecisionAction action;
    action.should_publish_goal = true;

    // ================= 1) 补给保持 =================
    if (snapshot.state == State::SUPPLY) {
      if (!isStatusRecovered(snapshot.rs)) {
        action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
        action.next_state = State::SUPPLY;
        action.target_x = config.supply_x;
        action.target_y = config.supply_y;
        action.target_yaw = config.supply_yaw;
        return action;
      }
    }

    // ================= 2) 状态差 -> 进补给 =================
    if (isStatusBad(snapshot.rs)) {
      action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
      action.next_state = State::SUPPLY;
      action.target_x = config.supply_x;
      action.target_y = config.supply_y;
      action.target_yaw = config.supply_yaw;
      return action;
    }

    // ================= 3) 状态好 -> 有敌攻击 =================
    if (snapshot.enemy_recent) {
      action.next_state = State::ATTACK;
      action.default_spin_latched = false;
      action.target_x = config.default_x;
      action.target_y = config.default_y;
      action.target_yaw = config.default_yaw;

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
    action.target_x = config.default_x;
    action.target_y = config.default_y;
    action.target_yaw = config.default_yaw;

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