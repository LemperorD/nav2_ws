#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "nav2_msgs/msg/costmap.hpp"

// auto_aim_interfaces 可能不是每个环境都有：CMake 会做 QUIET 探测
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
  // ====== “BT节点”同名能力：条件 ======
  bool IsStatusOK(int ammo_min, int heat_max, int hp_min) const;
  bool IsGameStatus(int expected_progress, int min_remain, int max_remain) const;
  bool IsRfidDetected_FriendlySupplyNonExchange() const;
  bool IsCombatCooldownReady() const;
  bool IsAttacked(uint8_t & armor_id_out) const;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  bool IsDetectEnemy() const;
#endif

  // ====== “BT节点”同名能力：动作 ======
  void SetChassisMode(int mode);
  void PublishSpinSpeed(double spin_speed);
  void PubNav2Goal(const geometry_msgs::msg::PoseStamped & goal);
  void PubNav2GoalXYZYaw(double x, double y, double yaw);
  void PublishTwistFor(const geometry_msgs::msg::Twist & tw, std::chrono::milliseconds dur);
  void PublishGimbalAbsoluteFor(double yaw, double pitch, std::chrono::milliseconds dur);

  // ====== CalculateAttackPose 的代码版 ======
  bool CalculateAttackPose(geometry_msgs::msg::PoseStamped & attack_pose_out);

  // ====== 对应 BT 子树：rmul_supply / Execute / ModeSelect / SubmodeCheck ======
  void tickMain();               // 对应 Main
  void SubmodeCheck();           // 对应 SubmodeCheck
  void ModeSelect();             // 对应 ModeSelect
  void Execute();                // 对应 Execute
  void rmul_supply();            // 对应 rmul_supply

  // ====== test_* 场景（可选） ======
  void tickTest();               // 根据参数 scenario 选择 test_* 行为
  void test_attack_pose();
  void test_attacked_feedback();
#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  void test_is_detect_enemy();
#endif

  // ====== ROS 回调 ======
  void onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);
  void onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);
  void onRfidStatus(const pb_rm_interfaces::msg::RfidStatus::SharedPtr msg);
  void onSubMode(const std_msgs::msg::Bool::SharedPtr msg);
  void onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg);


#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  void onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
  void onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg);
#endif

  // ====== 工具 ======
  geometry_msgs::msg::PoseStamped makePoseXYZYaw(const std::string & frame, double x, double y, double yaw) const;

  // ====== 参数（尽量沿用 XML 命名） ======
  std::string referee_robot_status_topic_;
  std::string referee_game_status_topic_;
  std::string referee_rfid_status_topic_;

  std::string goal_pose_topic_;
  std::string debug_attack_pose_topic_;

  std::string chassis_mode_topic_;
  std::string cmd_spin_topic_;
  std::string cmd_vel_topic_;
  std::string cmd_gimbal_topic_;

  std::string sub_mode_topic_;
  std::string nav_globalCostmap_topic_;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  std::string detector_armors_topic_;
  std::string tracker_target_topic_;
#endif

  // goals（XML 固定值）
  std::string frame_id_;
  double supply_x_, supply_y_, supply_yaw_;                 // 0,0,0
  double default_x_, default_y_, default_yaw_;              // 4.65,-3.5,0

  // 阈值（来自 XML）
  int hp_survival_enter_;      // 120
  int hp_survival_exit_;       // 300
  int hp_supply_done_;         // 399
  int heat_ok_default_;        // 350
  int heat_supply_done_;       // 100
  int ammo_min_;               // 0

  double combat_max_distance_; // 8.0
  double combat_cooldown_sec_; // 3.0

  double default_goal_hz_;     // 5.0
  double tick_hz_;             // 20.0
  double spin_speed_;          // 7.0

  bool require_game_running_;  // 是否强制 RUNNING 才执行（XML不强制，但你可选）
  std::string scenario_;       // "main" 或 "test_*"
  bool enable_attacked_feedback_;

  // ====== 状态（黑板变量） ======
  int current_mode_{-1};
  double combat_cooldown_ready_time_{0.0}; // steady_clock seconds
  bool sub_mode_{false};

  // ====== 最新裁判/传感数据 ======
  pb_rm_interfaces::msg::RobotStatus last_robot_status_;
  pb_rm_interfaces::msg::GameStatus  last_game_status_;
  pb_rm_interfaces::msg::RfidStatus  last_rfid_status_;
  bool has_robot_status_{false};
  bool has_game_status_{false};
  bool has_rfid_status_{false};

  nav2_msgs::msg::Costmap last_costmap_;

  bool has_costmap_{false};

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  auto_aim_interfaces::msg::Armors last_armors_;
  bool has_armors_{false};

  auto_aim_interfaces::msg::Target last_target_;
  bool has_target_{false};
#endif

  // ====== 发布器/订阅器 ======
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr chassis_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_spin_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_attack_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_gimbal_pub_;

  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mode_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
#endif

  rclcpp::TimerBase::SharedPtr timer_;

  // ====== attacked_feedback 的动作序列状态机 ======
  enum class AttackSeqState { IDLE, GIMBAL, TW1, TW2, TW3 };
  AttackSeqState attack_seq_state_{AttackSeqState::IDLE};
  rclcpp::Time attack_seq_deadline_;
};

} // namespace decision_simple
