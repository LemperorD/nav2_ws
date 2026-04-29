#include "decision_simple/core/decision.hpp"

namespace decision_simple {

  Decision::Decision(const ContextConfig& context_config)
      : hp_enter_supply_(context_config.hp_enter_supply),
        hp_exit_supply_(context_config.hp_exit_supply),
        ammo_min_(context_config.ammo_min),
        combat_max_distance_(context_config.combat_max_distance),
        require_game_running_(context_config.require_game_running),
        start_delay_sec_(context_config.start_delay_sec) {
  }
  DecisionAction Decision::compute(const Snapshot& s, double now_sec) {
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

}  // namespace decision_simple