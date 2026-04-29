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
  using nanoseconds = int64_t;
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
  };

  struct Readiness {
    enum class Status { READY, NO_RS, NO_GS, NOT_STARTED, IN_DELAY };
    Status status;
    double elapsed = 0.0;  // 仅供 Log 使用
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
    Readiness checkReadiness(int64_t now);
    void updatePose(const double x, const double y, const double z);
    bool isNear(double target_x, double target_y, double tolerance) const;

    State state_{State::DEFAULT};

  private:
    mutable std::mutex mtx_;
    bool has_robot_status_{false};
    bool match_started_{false};
    bool has_pose_{false};
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
    bool require_game_running_{false};
    double start_delay_sec_{5.0};

    double supply_x_{0.0}, supply_y_{0.0}, supply_yaw_{0.0};
    double default_x_{1.0}, default_y_{0.0}, default_yaw_{0.0};
    double x_{0.0}, y_{0.0}, yaw_{0.0};
  };

}  // namespace decision_simple
