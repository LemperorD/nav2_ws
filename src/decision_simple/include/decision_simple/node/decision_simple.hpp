#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "decision_simple/adapter/transform.hpp"
#include "decision_simple/core/decision.hpp"
#include "decision_simple/core/environment_context.hpp"
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
    void onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);
    void onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
    void onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg);

    // tick
    void tick();

    // helpers
    Stamp makeStamped(const rclcpp::Time time);
    void executeAction(DecisionAction action);
    void publishGoal(const DecisionAction& action);
    geometry_msgs::msg::PoseStamped makePoseXYZYaw(
        const std::string& frame, const Pose2D& position) const;

    // ====== chassis mode & arrival / attacked helpers ======

    bool getRobotPoseMap(Pose2D position);
    void setAndLogState(State s);
    void publishChassisMode(ChassisMode mode);
    void publishGoalThrottled(const geometry_msgs::msg::PoseStamped& goal,
                              rclcpp::Time& last_pub, double hz);
    void handleGateLog(Readiness& readiness);

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

    // supply/default coordinates are provided to ContextConfig; node keeps

    // Health/combat config moved to ContextConfig (not stored as node members)

    double tick_hz_{20.0};
    double default_goal_hz_{2.0};
    double supply_goal_hz_{2.0};
    double attack_goal_hz_{10.0};
    double start_delay_sec_{5.0};

    bool default_spin_latched_{false};

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

    State state_{State::DEFAULT};

    ChassisMode current_chassis_mode_{ChassisMode::CHASSIS_FOLLOWED};

    rclcpp::Time last_default_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_supply_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_attack_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_enemy_seen_{0, 0, RCL_ROS_TIME};

    geometry_msgs::msg::PoseStamped last_attack_goal_{};

    std::unique_ptr<EnvironmentContext> environment_;
    std::unique_ptr<Decision> controller_;
  };

}  // namespace decision_simple
