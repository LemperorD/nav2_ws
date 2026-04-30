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

    void onRobotStatus(const RobotStatus& robot_status);
    void onArmors(const Armors& msg);
    void onTarget(const Target msg);
    void onGameStatus(const GameStatus msg, int64_t match_start_time_ns);

    bool isGameStarted();
    bool isGameOver();
    void resetGameOver();

    bool getRobotPoseMap(double& x, double& y, double& yaw);

    Snapshot getSnapshot(Stamp now);

    bool isStatusBad(const RobotStatus& rs) const;
    bool isStatusRecovered(const RobotStatus& rs) const;

    bool detectEnemy(const Armors& armors,
                     const std::optional<Target>& target_opt) const;

    void setState(State s);
    bool isStateChanged();

    void setChassisMode(ChassisMode mode);
    Readiness checkReadiness(int64_t now);
    void updatePose(const double x, const double y, const double z);
    bool isNearRobotPose(double target_x, double target_y,
                         double tolerance) const;

  private:
    const ContextConfig config;
    mutable std::mutex mtx_;
    bool has_robot_status_{false};
    bool match_started_{false};
    bool has_pose_{false};
    bool has_game_status_{false};
    bool default_spin_latched_{false};
    bool is_game_started{false};
    bool is_game_over{false};
    bool is_state_changed{false};
    bool has_armors_{false};

    double attack_hold_sec_{1.5};
    double attacked_hold_sec_{1.5};

    RobotStatus last_robot_status_{};
    int64_t match_start_time_{};
    Armors last_armors_{};
    std::optional<Target> last_target_opt_;
    State state_{State::DEFAULT};
    uint8_t last_game_status_{0};

    double x_{0.0}, y_{0.0}, yaw_{0.0};
  };

}  // namespace decision_simple
