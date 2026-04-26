#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "decision_simple/context/environment_context.hpp"
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

namespace decision_simple {

  class DecisionSimple : public rclcpp::Node {
  public:
    explicit DecisionSimple(const rclcpp::NodeOptions& options);

  private:
    // callbacks
    void onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

    void onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
    void onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg);
    void onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

    // tick
    void tick();

    // helpers
    bool isStatusBad(const pb_rm_interfaces::msg::RobotStatus& rs) const;
    bool isStatusRecovered(const pb_rm_interfaces::msg::RobotStatus& rs) const;

    geometry_msgs::msg::PoseStamped makePoseXYZYaw(const std::string& frame,
                                                   double x, double y,
                                                   double yaw) const;

    bool detectEnemy(const auto_aim_interfaces::msg::Armors& armors,
                     const std::optional<auto_aim_interfaces::msg::Target>&
                         target_opt) const;

    bool buildAttackGoal(geometry_msgs::msg::PoseStamped& out,
                         const auto_aim_interfaces::msg::Armors& armors,
                         const std::optional<auto_aim_interfaces::msg::Target>&
                             target_opt) const;

    bool buildSupplyGoal(
        geometry_msgs::msg::PoseStamped& out,
        const pb_rm_interfaces::msg::RobotStatus& status) const;

    // ====== chassis mode & arrival / attacked helpers ======
    bool getRobotPoseMap(double& x, double& y, double& yaw);
    bool isNear(double gx, double gy, double tol_xy);

    void setState(State s);

    void publishChassisMode(ChassisMode mode);
    void setChassisMode(ChassisMode mode);

    void publishGoalThrottled(const geometry_msgs::msg::PoseStamped& goal,
                              rclcpp::Time& last_pub, double hz);

    // ===== Parameters =====
    std::string frame_id_{"map"};
    std::string base_frame_id_{"base_link"};

    std::string robot_status_topic_{"referee/robot_status"};
    std::string goal_pose_topic_{"goal_pose"};
    std::string chassis_mode_topic_{"chassis_mode"};
    std::string debug_attack_pose_topic_{"debug_attack_pose"};
    std::string game_status_topic_{"referee/game_status"};

    std::string detector_armors_topic_{"detector/armors"};
    std::string tracker_target_topic_{"tracker/target"};

    double supply_x_{0.0}, supply_y_{0.0}, supply_yaw_{0.0};
    double default_x_{1.0}, default_y_{0.0}, default_yaw_{0.0};

    double default_arrive_xy_tol_{0.30};
    double supply_arrive_xy_tol_{0.30};

    int hp_enter_supply_{120};
    int hp_exit_supply_{300};
    int ammo_min_{0};

    double combat_max_distance_{8.0};
    double attack_hold_sec_{1.0};
    double attacked_hold_sec_{1.5};

    double tick_hz_{20.0};
    double default_goal_hz_{2.0};
    double supply_goal_hz_{2.0};
    double attack_goal_hz_{10.0};

    double default_spin_keep_xy_tol_{0.80};
    bool default_spin_latched_{false};

    bool require_game_running_{false};
    double start_delay_sec_{5.0};

    // ===== Publishers & Subscribers =====
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr chassis_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        debug_attack_pose_pub_;

    rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr
        robot_status_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr
        game_status_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr
        armors_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr
        target_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ===== Cache/State =====
    mutable std::mutex mtx_;
    pb_rm_interfaces::msg::RobotStatus last_robot_status_{};
    bool has_robot_status_{false};

    auto_aim_interfaces::msg::Armors last_armors_{};
    bool has_armors_{false};
    std::optional<auto_aim_interfaces::msg::Target> last_target_opt_;

    State state_{State::DEFAULT};

    ChassisMode current_chassis_mode_{ChassisMode::CHASSIS_FOLLOWED};

    rclcpp::Time last_default_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_supply_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_attack_pub_{0, 0, RCL_ROS_TIME};

    rclcpp::Time last_enemy_seen_{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::PoseStamped last_attack_goal_{};
    bool has_last_attack_goal_{false};

    bool has_game_status_{false};
    uint8_t last_game_status_{0};
    bool match_started_{false};
    rclcpp::Time last_attacked_{0, 0, RCL_ROS_TIME};
    rclcpp::Time match_start_time_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<EnvironmentContext> environment_;
  };

}  // namespace decision_simple
