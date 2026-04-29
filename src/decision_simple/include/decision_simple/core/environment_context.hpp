#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "decision_simple/core/interfaces.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "types.hpp"

namespace decision_simple {
  struct ContextConfig {
    int hp_enter_supply;
    int hp_exit_supply;
    int ammo_min;
    // ... 其他参数
  };
  /// Environment state aggregation layer
  /// Translates low-level ROS messages into high-level business facts
  class EnvironmentContext {
  public:
    explicit EnvironmentContext() = default;

    explicit EnvironmentContext(const ContextConfig& context_config);

    /// Update environment state from robot status
    void onRobotStatus(const RobotStatus& robot_status);

    /// Update environment state from armors
    void onArmors(const Armors& msg);

    /// Update environment state from target
    void onTarget(const Target msg);

    /// Update environment state from game status
    void onGameStatus(const GameStatus msg, int64_t match_start_time_ns);
    /// Get robot pose in map frame
    bool isGameStarted();
    bool isGameOver();
    void resetGameOver();

    bool getRobotPoseMap(double& x, double& y, double& yaw);

    void tickForContext(Snapshot& snapshot);

    bool isStatusBad(const RobotStatus& rs) const;
    bool isStatusRecovered(const RobotStatus& rs) const;

    bool detectEnemy(const Armors& armors,
                     const std::optional<Target>& target_opt) const;

    void setState(State s);
    bool isStateChanged();

    void setChassisMode(ChassisMode mode);

    State state_{State::DEFAULT};

  private:
    mutable std::mutex mtx_;
    bool has_robot_status_{false};
    bool match_started_;
    RobotStatus last_robot_status_{};
    bool has_game_status_{false};
    int64_t match_start_time_{};
    bool has_armors_{false};
    Armors last_armors_{};
    std::optional<Target> last_target_opt_;
    uint8_t last_game_status_{0};
    bool default_spin_latched_{false};
    bool is_game_started{false};
    bool is_game_over{false};
    bool is_state_changed{false};

    int hp_enter_supply_{120};
    int hp_exit_supply_{300};
    int ammo_min_{0};
    double combat_max_distance_{8.0};
  };

}  // namespace decision_simple
