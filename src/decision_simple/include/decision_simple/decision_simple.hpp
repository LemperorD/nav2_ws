#ifndef DECISION_SIMPLE_HPP_
#define DECISION_SIMPLE_HPP_

#include <mutex>
#include <optional>
#include <string>

#include "decision_simple/macro_autoaim.hpp" 
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"          
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"                                 
#include "tf2_ros/transform_listener.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

typedef enum {
  chassisFollowed = 1, littleTES, goHome,
} chassisMode;

typedef enum {
  attack = 0, heal,
} state;

namespace decision_simple
{

class DecisionSimpleNode : public rclcpp::Node
{
public:
  explicit DecisionSimpleNode(const rclcpp::NodeOptions & options);
  ~DecisionSimpleNode() override;

private: // callbacks & function_to_judge
  void onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);
  void onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

private:
  void onConfigure();
  void tick();
  void setState(uint8_t s);
  void setChassisMode(); void setChassisMode(uint8_t mode);  
  void pubGoal();

private: // utils
  inline bool isNear();
  inline bool isStatusBad(const pb_rm_interfaces::msg::RobotStatus & rs);

private: // params
  std::string map_frame_id_{"map"};
  std::string base_frame_id_{"base_link"};                              
  std::string robot_status_topic_{"referee/robot_status"};
  std::string game_status_topic_{"referee/game_status"};
  std::string goal_pose_topic_{"goal_pose"};
  std::string chassis_mode_topic_{"chassis_mode"};

  double supply_x_{0.0}, supply_y_{0.0};
  double default_x_{4.65}, default_y_{-3.5};
  
  double tol_radius_{0.30};
  double tol_radius_min_{0.30};
  double tol_radius_max_{0.80};

  int hp_enter_supply_{120};
  int hp_exit_supply_{300};

  double combat_max_distance_{8.0};
  double attack_hold_sec_{1.0};

  double attacked_hold_sec_{1.5};

  double tick_hz_{20.0};
  double goal_hz_{10.0};
  //小陀螺保持半径
  double default_spin_keep_xy_tol_{0.80};
  bool default_spin_latched_{false};

private: // rclcpp
  // pubs
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr chassis_mode_pub_;
  // subs
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  // timer to tick
  rclcpp::TimerBase::SharedPtr timer_;
  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // action for nav2
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options_;

  pb_rm_interfaces::msg::RobotStatus last_robot_status_{};
  uint8_t last_game_status_{0};
  bool isNeedHeal_;
  bool require_game_running_{true};
  double start_delay_sec_{3.0};

  nav2_msgs::action::NavigateToPose::Goal current_goal_;
  uint8_t state_{attack};
  uint8_t chassis_mode_{chassisFollowed};

  bool match_started_{false};
  rclcpp::Time match_start_time_;

}; // class DecisionSimpleNode

}  // namespace decision_simple

#endif  // DECISION_SIMPLE_HPP_