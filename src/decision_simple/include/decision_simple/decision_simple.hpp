#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "std_msgs/msg/u_int8.hpp"

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#endif

namespace decision_simple
{

class DecisionSimple : public rclcpp::Node
{
public:
  explicit DecisionSimple(const rclcpp::NodeOptions & options);

private:
  enum class State : uint8_t { DEFAULT = 1, ATTACK = 2, SUPPLY = 3 };

  // callbacks
  void onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  void onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
  void onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg);
#endif

  // tick
  void tick();

  // helpers
  bool isStatusBad(const pb_rm_interfaces::msg::RobotStatus & rs) const;
  bool isStatusRecovered(const pb_rm_interfaces::msg::RobotStatus & rs) const;

  geometry_msgs::msg::PoseStamped makePoseXYZYaw(const std::string & frame, double x, double y, double yaw) const;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  bool detectEnemy(const auto_aim_interfaces::msg::Armors & armors,
                   const std::optional<auto_aim_interfaces::msg::Target> & target_opt) const;

  bool buildAttackGoal(geometry_msgs::msg::PoseStamped & out,
                       const auto_aim_interfaces::msg::Armors & armors,
                       const std::optional<auto_aim_interfaces::msg::Target> & target_opt) const;
#endif

  void setState(State s);
  void publishChassisMode(uint8_t mode);
  void publishGoalThrottled(const geometry_msgs::msg::PoseStamped & goal, rclcpp::Time & last_pub, double hz);

  // params
  std::string frame_id_{"map"};

  std::string robot_status_topic_{"referee/robot_status"};
  std::string goal_pose_topic_{"goal_pose"};

  // 你 bridge.cpp 订阅的是绝对话题 "/chassis_mode"
  std::string chassis_mode_topic_{"/chassis_mode"};

  // 可选调试：可视化攻击点
  std::string debug_attack_pose_topic_{"debug_attack_pose"};

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  std::string detector_armors_topic_{"detector/armors"};
  std::string tracker_target_topic_{"tracker/target"};
#endif

  double supply_x_{0.0}, supply_y_{0.0}, supply_yaw_{0.0};
  double default_x_{4.65}, default_y_{-3.5}, default_yaw_{0.0};

  int hp_enter_supply_{120};
  int hp_exit_supply_{300};   // 退出补给阈值（回到默认/攻击）
  int ammo_min_{0};

  double combat_max_distance_{8.0};
  double attack_hold_sec_{1.0};

  double tick_hz_{20.0};
  double default_goal_hz_{2.0};
  double supply_goal_hz_{2.0};
  double attack_goal_hz_{10.0};

  // pubs/subs
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr chassis_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_attack_pose_pub_;

  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
#endif

  rclcpp::TimerBase::SharedPtr timer_;

  // cache/state
  mutable std::mutex mtx_;
  pb_rm_interfaces::msg::RobotStatus last_robot_status_{};
  bool has_robot_status_{false};

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  auto_aim_interfaces::msg::Armors last_armors_{};
  bool has_armors_{false};
  std::optional<auto_aim_interfaces::msg::Target> last_target_opt_;
#endif

  State state_{State::DEFAULT};

  rclcpp::Time last_default_pub_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_supply_pub_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attack_pub_{0, 0, RCL_ROS_TIME};

  rclcpp::Time last_enemy_seen_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_attack_goal_{};
  bool has_last_attack_goal_{false};
};

}  // namespace decision_simple
