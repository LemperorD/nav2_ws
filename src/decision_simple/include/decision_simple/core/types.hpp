#ifndef DECISION_SIMPLE__CORE__TYPES_HPP_
#define DECISION_SIMPLE__CORE__TYPES_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace decision_simple {

  /// 3D quaternion (for rotation representation)
  struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  /// 3D pose with position and orientation
  struct Pose3D {
    /// Position coordinates
    double pos_x = 0.0;
    double pos_y = 0.0;
    double pos_z = 0.0;

    /// Orientation as quaternion
    Quaternion orientation;
  };

  /// Robot status domain model (pure C++, no ROS dependencies)
  struct RobotStatus {
    // Identity and state
    uint8_t robot_id = 0;
    uint8_t robot_level = 0;
    uint16_t current_hp = 0;
    uint16_t maximum_hp = 0;

    // Shooter/thermal management
    uint16_t shooter_barrel_cooling_value = 0;
    uint16_t shooter_barrel_heat_limit = 0;
    uint16_t shooter_17mm_1_barrel_heat = 0;

    // Position and orientation
    Pose3D robot_pos;

    // Status information
    uint8_t armor_id = 0;
    uint8_t hp_deduction_reason = 0;
    uint16_t projectile_allowance_17mm = 0;
    uint16_t remaining_gold_coin = 0;
    bool is_hp_deduced = false;

    // HP deduction reason constants
    static constexpr uint8_t ARMOR_HIT = 0u;
    static constexpr uint8_t SYSTEM_OFFLINE = 1u;
    static constexpr uint8_t OVER_SHOOT_SPEED = 2u;
    static constexpr uint8_t OVER_HEAT = 3u;
    static constexpr uint8_t OVER_POWER = 4u;
    static constexpr uint8_t ARMOR_COLLISION = 5u;
  };

  /// Game status (match state)
  struct GameStatus {
    uint8_t game_progress = 0;      // 当前比赛阶段
    int32_t stage_remain_time = 0;  // 当前阶段剩余时间，单位：秒

    // Game progress constants
    static constexpr uint8_t NOT_START = 0u;      // 未开始比赛
    static constexpr uint8_t PREPARATION = 1u;    // 准备阶段
    static constexpr uint8_t SELF_CHECKING = 2u;  // 十五秒裁判系统自检阶段
    static constexpr uint8_t COUNT_DOWN = 3u;     // 五秒倒计时
    static constexpr uint8_t RUNNING = 4u;        // 比赛中
    static constexpr uint8_t GAME_OVER = 5u;      // 比赛结算中
  };

  /// Single armor target
  struct Armor {
    Pose3D pose;           // Armor position and orientation
    uint8_t armor_id = 0;  // Armor identifier
  };

  /// Collection of visible armors
  struct Armors {
    std::string frame_id;      // Coordinate frame ID
    std::vector<Armor> items;  // List of detected armors
  };

  /// Tracked target (single enemy)
  struct Target {
    std::string frame_id;   // Coordinate frame ID
    Pose3D position;        // Position (x, y, z)
    double yaw = 0.0;       // Yaw angle for orientation
    bool tracking = false;  // Whether actively tracking
  };

  // Type aliases for smart pointers
  using RobotStatusPtr = std::shared_ptr<RobotStatus>;
  using RobotStatusConstPtr = std::shared_ptr<const RobotStatus>;
  using GameStatusPtr = std::shared_ptr<GameStatus>;
  using GameStatusConstPtr = std::shared_ptr<const GameStatus>;
  using ArmorsPtr = std::shared_ptr<Armors>;
  using ArmorsConstPtr = std::shared_ptr<const Armors>;
  using TargetPtr = std::shared_ptr<Target>;
  using TargetConstPtr = std::shared_ptr<const Target>;

}  // namespace decision_simple

#endif  // DECISION_SIMPLE__CORE__TYPES_HPP_
