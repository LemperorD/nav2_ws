#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace decision_simple {
  using nanoseconds = int64_t;

  /// Chassis movement mode
  enum class ChassisMode : uint8_t {
    CHASSIS_FOLLOWED = 1,
    LITTLE_TES = 2,
    GO_HOME = 3,
  };

  enum class State : uint8_t {
    DEFAULT = 1,
    ATTACK = 2,
    SUPPLY = 3,
  };

  struct ContextConfig {
    int hp_enter_supply{120};
    int hp_exit_supply{300};
    int ammo_min{0};
    double combat_max_distance{8.0};
    bool require_game_running{false};
    double start_delay_sec{5.0};
    double default_x_{0};
    double default_y_{0};
    double default_yaw_{0};
    double supply_x_{0};
    double supply_y_{0};
    double supply_yaw{0};
    double default_arrive_xy_tol{0.0};
    double default_spin_keep_xy_tol{0.0};
  };

  struct DecisionAction {
    State next_state;
    ChassisMode chassis_mode;
    double target_x;
    double target_y;
    double target_yaw;
    bool should_publish_goal;
    bool default_spin_latched;
  };

  struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct Pose2D {
    double x;
    double y;
    double yaw;
  };
  struct Pose3D {
    Position position;
    Quaternion orientation;
  };

  struct RobotStatus {
    uint8_t robot_id = 0;
    uint8_t robot_level = 0;
    uint16_t current_hp = 0;
    uint16_t maximum_hp = 0;

    uint16_t shooter_barrel_cooling_value = 0;
    uint16_t shooter_barrel_heat_limit = 0;
    uint16_t shooter_17mm_1_barrel_heat = 0;

    Pose3D robot_pos;

    uint8_t armor_id = 0;
    uint8_t hp_deduction_reason = 0;
    uint16_t projectile_allowance_17mm = 0;
    uint16_t remaining_gold_coin = 0;
    bool is_hp_deduced = false;

    static constexpr uint8_t ARMOR_HIT = 0u;
    static constexpr uint8_t SYSTEM_OFFLINE = 1u;
    static constexpr uint8_t OVER_SHOOT_SPEED = 2u;
    static constexpr uint8_t OVER_HEAT = 3u;
    static constexpr uint8_t OVER_POWER = 4u;
    static constexpr uint8_t ARMOR_COLLISION = 5u;
  };

  struct GameStatus {
    uint8_t game_progress = 0;
    int32_t stage_remain_time = 0;

    static constexpr uint8_t NOT_START = 0u;
    static constexpr uint8_t PREPARATION = 1u;
    static constexpr uint8_t SELF_CHECKING = 2u;
    static constexpr uint8_t COUNT_DOWN = 3u;
    static constexpr uint8_t RUNNING = 4u;
    static constexpr uint8_t GAME_OVER = 5u;
  };

  struct Stamp {
    int32_t sec;
    uint32_t nanosec;
  };
  struct Header {
    Stamp stamp;
    std::string frame_id;
  };

  struct Armor {
    Pose3D pose;
  };

  struct Armors {
    Header header;

    std::vector<Armor> armors;
  };

  struct Target {
    Header header;

    Position position;
    double yaw = 0.0;
    bool tracking = false;
  };

  struct Snapshot {
    RobotStatus rs;
    bool has_rs = false;
    bool has_gs = false;
    bool match_started = false;
    Stamp match_start_time;
    Armors armors;
    bool has_armors = false;
    std::optional<Target> target_opt;
    State state{State::DEFAULT};
    bool enemy{false};
    Stamp last_enemy_seen_{0, 0};
    Stamp last_attacked_{0, 0};
    bool enemy_recent;
    bool attacked_recent;
    bool at_center{false};
    bool in_center_keep_spin{false};
    bool default_spin_latched{false};
    Position last_attack_position{0.0, 0.0, 0.0};
    double last_attack_yaw{0.0};
    bool has_attack_goal{false};
  };

}  // namespace decision_simple
