#ifndef DECISION_SIMPLE__CORE__TYPES_HPP_
#define DECISION_SIMPLE__CORE__TYPES_HPP_

#include <cstdint>
#include <memory>

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

  // Type aliases for smart pointers
  using RobotStatusPtr = std::shared_ptr<RobotStatus>;
  using RobotStatusConstPtr = std::shared_ptr<const RobotStatus>;

}  // namespace decision_simple

#endif  // DECISION_SIMPLE__CORE__TYPES_HPP_
